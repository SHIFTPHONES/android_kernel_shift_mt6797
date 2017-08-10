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

#ifndef __MTK_MEMCFG_H__
#define __MTK_MEMCFG_H__
#include <linux/fs.h>

/* late warning flags */
#define WARN_MEMBLOCK_CONFLICT	(1 << 0)	/* memblock overlap */
#define WARN_MEMSIZE_CONFLICT	(1 << 1)	/* dram info missing */
#define WARN_API_NOT_INIT	(1 << 2)	/* API is not initialized */

#define MTK_MEMCFG_MEMBLOCK_PHY 0x1
#define MTK_MEMCFG_MEMBLOCK_DEBUG 0x2

#ifdef CONFIG_MTK_MEMCFG

extern int kptr_restrict;
#define MAX_FREE_RESERVED 20

struct freed_reserved_memory {
	phys_addr_t start;
	phys_addr_t end;
};

extern struct freed_reserved_memory freed_reserved_memory[MAX_FREE_RESERVED];
extern int freed_reserved_memory_count;

void mtk_memcfg_record_freed_reserved(phys_addr_t start, phys_addr_t end);

#include <linux/memblock.h>
extern int memblock_reserve_count;
extern struct memblock_record memblock_record[MAX_MEMBLOCK_RECORD];
extern struct memblock_stack_trace memblock_stack_trace[MAX_MEMBLOCK_RECORD];
extern struct kernel_reserve_meminfo kernel_reserve_meminfo;

#define MTK_MEMCFG_LOG_AND_PRINTK(fmt, arg...)  \
	do {    \
		pr_alert(fmt, ##arg); \
		mtk_memcfg_write_memory_layout_buf(fmt, ##arg); \
	} while (0)

extern void mtk_memcfg_write_memory_layout_buf(char *, ...);
extern void mtk_memcfg_late_warning(unsigned long);

#include <mt-plat/mtk_memcfg_reserve_info.h>
int check_mblock_support(void);
int parse_memory_layout_buf(struct reserved_mem_ext *reserved_mem);
void mtk_memcfg_write_memory_layout_info(int type, const char *name,
		unsigned long start, unsigned long size);

#ifdef CONFIG_SLUB_DEBUG
extern int slabtrace_open(struct inode *inode, struct file *file);
#endif /* end of CONFIG_SLUB_DEBUG */

#if defined(CONFIG_MTK_FB)
extern unsigned int DISP_GetVRamSizeBoot(char *cmdline);
#endif	/* end of CONFIG_MTK_FB */

#ifdef CONFIG_OF
extern phys_addr_t mtkfb_get_fb_base(void);
extern phys_addr_t mtkfb_get_fb_size(void);
#endif	/* end of CONFIG_OF */

#ifdef CONFIG_HIGHMEM
extern unsigned long totalhigh_pages;
#endif /* end of CONFIG_HIGHMEM */

extern void split_page(struct page *page, unsigned int order);

#else

#define MTK_MEMCFG_LOG_AND_PRINTK(fmt, arg...) pr_info(fmt, ##arg)

#define mtk_memcfg_get_force_inode_gfp_lowmem()  do { } while (0)
#define mtk_memcfg_set_force_inode_gfp_lowmem(flag)  do { } while (0)
#define mtk_memcfg_get_bypass_slub_debug_flag()  do { } while (0)
#define mtk_memcfg_set_bypass_slub_debug_flag(flag)  do { } while (0)
#define mtk_memcfg_write_memory_layout_buf(fmt, arg...) do { } while (0)
#define mtk_memcfg_late_warning(flag) do { } while (0)
#define mtk_memcfg_write_memory_layout_info(arg...) do { } while (0)
#define mtk_memcfg_record_freed_reserved(start, end) do {} while (0)
#endif /* end CONFIG_MTK_MEMCFG */

#endif /* end __MTK_MEMCFG_H__ */
