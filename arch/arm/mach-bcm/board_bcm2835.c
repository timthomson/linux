/*
 * Copyright (C) 2010 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/irqchip.h>
#include <linux/mm.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <asm/system_info.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/memory.h>
#include <asm/pgtable.h>

#include "platsmp.h"

static void __init bcm2835_init(void)
{
	struct device_node *np = of_find_node_by_path("/system");
	u32 val;
	u64 val64;

	if (!of_property_read_u32(np, "linux,revision", &val))
		system_rev = val;
	if (!of_property_read_u64(np, "linux,serial", &val64))
		system_serial_low = val64;
}

/*
 * fiq_map_check_node() - check if #size-cells, #address-cells provided
 * matches the values supported by the current implementation,
 */
static int __init fiq_map_check_node(unsigned long node)
{
    const char *status;
    const __be32 *prop;

    /* not currently checking #size-cells as many entries contain
     * #size-cells <0> rather than <1>.
     */
    // prop = of_get_flat_dt_prop(node, "#size-cells", NULL);
    // if (prop && be32_to_cpup(prop) != dt_root_size_cells) {
	//     pr_err("FIQ Premap: 1, size: %d. Expected %d.\n", be32_to_cpup(prop), dt_root_size_cells);
    //     return -EINVAL;
    // }

    prop = of_get_flat_dt_prop(node, "#address-cells", NULL);
    if (prop && be32_to_cpup(prop) != dt_root_addr_cells) {
        return -EINVAL;
    }

    status = of_get_flat_dt_prop(node, "status", NULL);
    if (status && strcmp(status, "ok") && strcmp(status, "okay")) {
        return -EINVAL;
    }
    return 0;
}

// TODO: potentially track previous allocation to minimise repeated
// allocations of small blocks within same page?
struct fiq_map_config {
    unsigned long next_free;
    unsigned long p2b_offset;
};
/*
 * We need to map registers that are going to be accessed by the FIQ
 * very early, before any kernel threads are spawned. Because if done
 * later, the mapping tables are not updated instantly but lazily upon
 * first access through a data abort handler. While that is fine
 * when executing regular kernel code, if the first access in a specific
 * thread happens while running FIQ code this will result in a panic.
 *
 * For more background see the following old mailing list thread:
 * https://www.spinics.net/lists/arm-kernel/msg325250.html
 */
static int __init bcm2835_map_fiq(unsigned long node, const char *uname,
                    int depth, void *data)
{
    struct map_desc map[1];
    const __be32 *reg;
    int len;
    u64 base, size;
    struct fiq_map_config *fmc = data;
    int t_len = (dt_root_addr_cells + dt_root_size_cells) * sizeof(__be32);

    if (of_get_flat_dt_prop(node, "raspberrypi,fiq-premap", NULL) == NULL) {
        return 0;
    }
	if (fiq_map_check_node(node)) {
	    pr_err("FIQ Premap: unsupported node format in %s, ignoring\n", uname);
	    return 0;
	}
	reg = of_get_flat_dt_prop(node, "reg", &len);
	if (!reg) {
	    pr_err("FIQ Premap: no reg entry found in %s, skipping node\n", uname);
		return 0;
	}
	if (len && len % t_len != 0) {
	    pr_err("FIQ Premap: invalid reg property in '%s', skipping node.", uname);
	    return 0;
	}

	pr_err("FIQ premap node %s, reg size %d. Item len = %d.\n", uname, len, t_len);

	while (len >= t_len) {
	    base = dt_mem_next_cell(dt_root_addr_cells, &reg) - fmc->p2b_offset;
	    size = dt_mem_next_cell(dt_root_size_cells, &reg);

	    if (size == 0) {
	        pr_err("FIQ premap node size 0, skipping\n");
	        continue;
	    }
	    pr_err("FIQ premap mapping - %llx ,  len %llx. Mapping to %lx.\n", (unsigned long long)base,
	        (unsigned long long)size, fmc->next_free);
	    /*
	     * pre-align allocation so we can keep track of where next
	     * allocation begins
	     */
	    size = PAGE_ALIGN(size + (base & ~PAGE_MASK));
	    base &= PAGE_MASK;
	    pr_err("FIQ premap mapping aligned - %llx ,  len %llx.\n", (unsigned long long)base,
	        (unsigned long long)size);

	    /* Use information about the physical addresses of the
	     * registers from the device tree, but use legacy
	     * iotable_init() static mapping function to map them,
	     * as ioremap() is not functional at this stage in boot.
	     */
	    map[0].virtual = fmc->next_free;
	    map[0].pfn = __phys_to_pfn(base);
	    map[0].length = size;
	    map[0].type = MT_DEVICE;
	    iotable_init(map, 1);

	    fmc->next_free += PAGE_ALIGN(size);
	    len -= t_len;
	}
	// TODO: return 1 means we only map one device right??
	return 0;
}

static void __init bcm2835_map_io(void)
{
	const __be32 *ranges, *address_cells;
	unsigned long root, addr_cells;
	int soc, len;
	struct fiq_map_config fmc;

	debug_ll_io_init();

	root = of_get_flat_dt_root();
	/* Find out how to map bus to physical address first from soc/ranges */
	soc = of_get_flat_dt_subnode_by_name(root, "soc");
	if (soc < 0)
		return;
	address_cells = of_get_flat_dt_prop(root, "#address-cells", &len);
	if (!address_cells || len < (sizeof(unsigned long)))
		return;
	addr_cells = be32_to_cpu(address_cells[0]);
	ranges = of_get_flat_dt_prop(soc, "ranges", &len);
	if (!ranges || len < (sizeof(unsigned long) * (2 + addr_cells)))
		return;
	fmc.p2b_offset = be32_to_cpu(ranges[0]) - be32_to_cpu(ranges[addr_cells]);

	/* Now search for bcm2708-usb node in device tree */
	fmc.next_free = VMALLOC_START;
	of_scan_flat_dt(bcm2835_map_fiq, &fmc);
	pr_err("FIQ premapping complete. Next free address is %lx.\n", fmc.next_free);
}

static const char * const bcm2835_compat[] = {
#ifdef CONFIG_ARCH_MULTI_V6
	"brcm,bcm2835",
#endif
#ifdef CONFIG_ARCH_MULTI_V7
	"brcm,bcm2836",
	"brcm,bcm2837",
	"brcm,bcm2711",
	// Temporary, for backwards-compatibility with old DTBs
	"brcm,bcm2838",
#endif
	NULL
};

DT_MACHINE_START(BCM2835, "BCM2835")
#if defined(CONFIG_ZONE_DMA) && defined(CONFIG_ARM_LPAE)
	.dma_zone_size	= SZ_1G,
#endif
	.map_io = bcm2835_map_io,
	.init_machine = bcm2835_init,
	.dt_compat = bcm2835_compat,
	.smp = smp_ops(bcm2836_smp_ops),
MACHINE_END
