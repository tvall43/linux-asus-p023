/*
 * Device tree based initialization code for reserved memory.
 *
 * Copyright (c) 2013, The Linux Foundation. All Rights Reserved.
 * Copyright (c) 2013,2014 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 * Copyright (C) 2014-2015 Intel Mobile Communications GmbH
 *
 * Author: Marek Szyprowski <m.szyprowski@samsung.com>
 * Author: Josh Cartwright <joshc@codeaurora.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License or (at your optional) any later version of the license.
 */

#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_platform.h>
#include <linux/mm.h>
#include <linux/sizes.h>
#include <linux/of_reserved_mem.h>
#include <linux/dma-contiguous.h>

#define MAX_RESERVED_REGIONS	16
static struct reserved_mem reserved_mem[MAX_RESERVED_REGIONS];
static int reserved_mem_count;

extern int Read_PCB_ID(void);

#if defined(CONFIG_HAVE_MEMBLOCK)
#include <linux/memblock.h>
int __init __weak early_init_dt_alloc_reserved_memory_arch(phys_addr_t size,
	phys_addr_t align, phys_addr_t start, phys_addr_t end, bool nomap,
	phys_addr_t *res_base)
{
	/*
	 * We use __memblock_alloc_base() because memblock_alloc_base()
	 * panic()s on allocation failure.
	 */
	phys_addr_t base = __memblock_alloc_base(size, align, end);
	if (!base)
		return -ENOMEM;

	/*
	 * Check if the allocated region fits in to start..end window
	 */
	if (base < start) {
		memblock_free(base, size);
		return -ENOMEM;
	}

	*res_base = base;
	if (nomap)
		return memblock_remove(base, size);
	return 0;
}
#else
int __init __weak early_init_dt_alloc_reserved_memory_arch(phys_addr_t size,
	phys_addr_t align, phys_addr_t start, phys_addr_t end, bool nomap,
	phys_addr_t *res_base)
{
	pr_err("Reserved memory not supported, ignoring region 0x%llx%s\n",
		  size, nomap ? " (nomap)" : "");
	return -ENOSYS;
}
#endif

/**
 * res_mem_save_node() - save fdt node for second pass initialization
 */
void __init fdt_reserved_mem_save_node(unsigned long node, const char *uname,
				      phys_addr_t base, phys_addr_t size)
{
	struct reserved_mem *rmem = &reserved_mem[reserved_mem_count];

	if (reserved_mem_count == ARRAY_SIZE(reserved_mem)) {
		pr_err("Reserved memory: not enough space all defined regions.\n");
		return;
	}

	rmem->fdt_node = node;
	rmem->name = uname;
	rmem->base = base;
	rmem->size = size;

	reserved_mem_count++;
	return;
}

/**
 * res_mem_alloc_size() - allocate reserved memory described by 'size', 'align'
 *			  and 'alloc-ranges' properties
 */
static int __init __reserved_mem_alloc_size(unsigned long node,
	const char *uname, phys_addr_t *res_base, phys_addr_t *res_size)
{
	int t_len = (dt_root_addr_cells + dt_root_size_cells) * sizeof(__be32);
	phys_addr_t start = 0, end = 0;
	phys_addr_t base = 0, align = 0, size = 0;
	unsigned long len;
	__be32 *prop;
	int nomap;
	int ret;
	int pcb_id;

	/* reserve cma default area */
	if (of_get_flat_dt_prop(node, "linux,cma-default", NULL) != NULL) {
		phys_addr_t limit = 0;
		prop = of_get_flat_dt_prop(node, "limit", &len);
		if (prop)
			limit = dt_mem_next_cell(dt_root_size_cells,
						&prop);
		pcb_id = Read_PCB_ID();
		if(((pcb_id>>11) & 0x3) != 2) {
			prop = of_get_flat_dt_prop(node, "size1", &len);
		} else {
			prop = of_get_flat_dt_prop(node, "size", &len);
		}
		if (prop)
			size = dt_mem_next_cell(dt_root_size_cells,
						&prop);

		pr_info("Reserved memory: '%s' reserve CMA default limit %pa, size %ld MiB\n",
			uname, &limit, (unsigned long)size / SZ_1M);
		dma_contiguous_reserve(limit, size);
		return 0;
	}

	prop = of_get_flat_dt_prop(node, "size", &len);
	if (!prop)
		return -EINVAL;

	if (len != dt_root_size_cells * sizeof(__be32)) {
		pr_err("Reserved memory: invalid size property in '%s' node.\n",
				uname);
		return -EINVAL;
	}
	size = dt_mem_next_cell(dt_root_size_cells, &prop);

	nomap = of_get_flat_dt_prop(node, "no-map", NULL) != NULL;

	prop = of_get_flat_dt_prop(node, "alignment", &len);
	if (prop) {
		if (len != dt_root_addr_cells * sizeof(__be32)) {
			pr_err("Reserved memory: invalid alignment property in '%s' node.\n",
				uname);
			return -EINVAL;
		}
		align = dt_mem_next_cell(dt_root_addr_cells, &prop);
	}

	prop = of_get_flat_dt_prop(node, "alloc-ranges", &len);
	if (prop) {

		if (len % t_len != 0) {
			pr_err("Reserved memory: invalid alloc-ranges property in '%s', skipping node.\n",
			       uname);
			return -EINVAL;
		}

		base = 0;

		while (len > 0) {
			start = dt_mem_next_cell(dt_root_addr_cells, &prop);
			end = start + dt_mem_next_cell(dt_root_size_cells,
						       &prop);

			ret = early_init_dt_alloc_reserved_memory_arch(size,
					align, start, end, nomap, &base);
			if (ret == 0) {
				pr_debug("Reserved memory: allocated memory for '%s' node: base %pa, size %ld MiB\n",
					uname, &base,
					(unsigned long)size / SZ_1M);
				break;
			}
			len -= t_len;
		}

	} else {
		ret = early_init_dt_alloc_reserved_memory_arch(size, align,
							0, 0, nomap, &base);
		if (ret == 0)
			pr_debug("Reserved memory: allocated memory for '%s' node: base %pa, size %ld MiB\n",
				uname, &base, (unsigned long)size / SZ_1M);
	}

	if (base == 0) {
		pr_info("Reserved memory: failed to allocate memory for node '%s'\n",
			uname);
		return -ENOMEM;
	}

	*res_base = base;
	*res_size = size;

	return 0;
}

static const struct of_device_id __rmem_of_table_sentinel
	__used __section(__reservedmem_of_table_end);

/**
 * res_mem_init_node() - call region specific reserved memory init code
 */
static int __init __reserved_mem_init_node(struct reserved_mem *rmem)
{
	extern const struct of_device_id __reservedmem_of_table[];
	const struct of_device_id *i;

	for (i = __reservedmem_of_table; i < &__rmem_of_table_sentinel; i++) {
		reservedmem_of_init_fn initfn = i->data;
		const char *compat = i->compatible;

		if (!of_flat_dt_is_compatible(rmem->fdt_node, compat))
			continue;

		if (initfn(rmem, rmem->fdt_node, rmem->name) == 0) {
			pr_info("Reserved memory: initialized node %s, compatible id %s\n",
				rmem->name, compat);
			return 0;
		}
	}
	return -ENOENT;
}

/**
 * fdt_init_reserved_mem - allocate and init all saved reserved memory regions
 */
void __init fdt_init_reserved_mem(void)
{
	int i;
	for (i = 0; i < reserved_mem_count; i++) {
		struct reserved_mem *rmem = &reserved_mem[i];
		unsigned long node = rmem->fdt_node;
		int err = 0;

		if (rmem->size == 0)
			err = __reserved_mem_alloc_size(node, rmem->name,
						 &rmem->base, &rmem->size);
		if (err == 0)
			__reserved_mem_init_node(rmem);
	}
}

int of_get_reserved_memory_region(struct device_node *node,
		phys_addr_t *size, phys_addr_t *base,
		struct cma **cma_area)
{
	int ret;
	struct device_node *nregion = NULL;
	int pcb_id;
	u32 array[2] = {0, 0};

	*cma_area = NULL;

	nregion = of_parse_phandle(node, "memory-region", 0);
	if (!nregion) {
		pr_err("%s: Can't find memory region in '%s'.\n",
				__func__, node->name);
		return -ENOENT;
	}
	ret = of_property_read_u32_array(nregion, "reg", array, 2);
	if (ret) {
		ret = of_property_read_u32(nregion, "size", &array[1]);
		if (ret) {
			pr_err("%s: Can't find memory definiition '%s'\n",
					__func__, nregion->name);
			return -ENOENT;
		}
	}

	*base = array[0];
	*size = array[1];

	if (of_property_read_bool(nregion, "linux,cma"))
		*cma_area = cma_area_lookup(*size, *base);
	else if (of_property_read_bool(nregion, "linux,cma-default")) {
		*cma_area = dma_contiguous_default_area;
                pcb_id = Read_PCB_ID();
                if(((pcb_id>>11) & 0x3) != 2) {
			ret = of_property_read_u32(nregion, "size1", &array[1]);
                        *size = array[1];
                }
	}

	pr_debug("found cma area %p %d MiB at %pa\n",
			*cma_area, array[1] / SZ_1M, base);

	return (*cma_area) ? 0 : -EINVAL;
}

