/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __ION_DRV_PRIV_H__
#define __ION_DRV_PRIV_H__

#include "ion_priv.h"

/* STRUCT ION_HEAP *G_ION_HEAPS[ION_HEAP_IDX_MAX]; */

/* Import from multimedia heap */
long ion_mm_ioctl(struct ion_client *client, unsigned int cmd,
		unsigned long arg, int from_kernel);

void smp_inner_dcache_flush_all(void);
#ifdef CONFIG_MTK_CACHE_FLUSH_RANGE_PARALLEL
int mt_smp_cache_flush(struct sg_table *table, unsigned int sync_type, int npages);

extern int (*ion_sync_kernel_func)(unsigned long start, size_t size,
		unsigned int sync_type);
#endif

#ifdef ION_HISTORY_RECORD
int ion_history_init(void);
void ion_history_count_kick(bool allc, size_t len);
#else
static inline int ion_history_init(void)
{
	return 0;
}

void ion_history_count_kick(bool allc, size_t len)
{
	/*do nothing*/
}
#endif

int ion_mm_heap_for_each_pool(int (*fn)(int high, int order, int cache, size_t size));
struct ion_heap *ion_drv_get_heap(struct ion_device *dev, int heap_id, int need_lock);
int ion_drv_create_heap(struct ion_platform_heap *heap_data);
struct ion_buffer *ion_drv_file_to_buffer(struct file *file);

#ifdef CONFIG_PM
extern void shrink_ion_by_scenario(int need_lock);
#endif

#endif
