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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/seq_file.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/setup.h>
#include <mt-plat/mtk_memcfg.h>
#include <mt-plat/mtk_memcfg_reserve_info.h>
#include <mt-plat/aee.h>
#include <linux/of_fdt.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/mod_devicetable.h>
#include <linux/io.h>
#include <linux/sort.h>
#include <linux/mm.h>
#include <linux/memblock.h>
#ifdef CONFIG_MTK_AEE_FEATURE
#include <mt-plat/aee.h>
#endif

#define MTK_MEMCFG_SIMPLE_BUFFER_LEN 16
#define MTK_MEMCFG_LARGE_BUFFER_LEN (2048)

struct mtk_memcfg_info_buf {
	unsigned long max_len;
	unsigned long curr_pos;
	char buf[MTK_MEMCFG_LARGE_BUFFER_LEN];
};

static struct mtk_memcfg_info_buf mtk_memcfg_layout_buf = {
	.buf = {[0 ... (MTK_MEMCFG_LARGE_BUFFER_LEN - 1)] = 0,},
	.max_len = MTK_MEMCFG_LARGE_BUFFER_LEN,
	.curr_pos = 0,
};

static void mtk_memcfg_show_layout_region(struct seq_file *m, const char *name,
		unsigned long long end, unsigned long long size, int nomap, int is_end);

static void mtk_memcfg_show_layout_region_kernel(struct seq_file *m,
		unsigned long long end, unsigned long long size);

static int mtk_memcfg_layout_phy_count;
static int sort_layout;

#define MAX_INFO_NAME 20
struct mtk_memcfg_layout_info {
	char name[MAX_INFO_NAME];
	unsigned long long start;
	unsigned long long size;
};

#define MAX_LAYOUT_INFO 30
static struct mtk_memcfg_layout_info mtk_memcfg_layout_info_phy[MAX_LAYOUT_INFO];

int check_mblock_support(void)
{
	char *path = "/memory";
	struct device_node *dt_node;
	const struct mblock_info *mb_info = NULL;

	dt_node = of_find_node_by_path(path);
	if (!dt_node) {
		pr_info("Failed to get dts not \"memory\"\n");
		return 0;
	}

	mb_info = of_get_property(dt_node, "mblock_info", NULL);
	if (!mb_info || !(mb_info->mblock_magic == 0x99999999 && mb_info->mblock_version >= 2)) {
		pr_info("Memory layout does not support mblock version < 2\n");
		return 1;
	}
	return 2;
}

int mtk_memcfg_memory_layout_info_compare(const void *p1, const void *p2)
{
	if (((struct mtk_memcfg_layout_info *)p1)->start > ((struct mtk_memcfg_layout_info *)p2)->start)
		return 1;
	return -1;
}

void mtk_memcfg_sort_memory_layout(void)
{
	if (sort_layout != 0)
		return;
	sort(&mtk_memcfg_layout_info_phy, mtk_memcfg_layout_phy_count,
			sizeof(struct mtk_memcfg_layout_info),
			mtk_memcfg_memory_layout_info_compare, NULL);
	sort_layout = 1;
}

void mtk_memcfg_write_memory_layout_info(int type, const char *name, unsigned long
		start, unsigned long size)
{
	struct mtk_memcfg_layout_info *info = NULL;
	int len = 0;

	if (type == MTK_MEMCFG_MEMBLOCK_PHY &&
			mtk_memcfg_layout_phy_count < MAX_LAYOUT_INFO)
		info = &mtk_memcfg_layout_info_phy[mtk_memcfg_layout_phy_count++];
	else
#ifdef CONFIG_MTK_AEE_FEATURE
		aee_kernel_warning("memory layout info", "count > MAX_LAYOUT_INFO");
#else
		pr_info("memory layout info: count > MAX_LAYOUT_INFO");
#endif

	if (info) {
		len = sizeof(info->name) > MAX_INFO_NAME ? MAX_INFO_NAME : sizeof(info->name);
		strncpy(info->name, name, len - 1);
		info->name[len - 1] = '\0';
		info->start = start;
		info->size = size;
	}
}

static unsigned long mtk_memcfg_late_warning_flag;

void mtk_memcfg_write_memory_layout_buf(char *fmt, ...)
{
	va_list ap;
	struct mtk_memcfg_info_buf *layout_buf = &mtk_memcfg_layout_buf;

	if (layout_buf->curr_pos <= layout_buf->max_len) {
		va_start(ap, fmt);
		layout_buf->curr_pos +=
		    vsnprintf((layout_buf->buf + layout_buf->curr_pos),
			      (layout_buf->max_len - layout_buf->curr_pos), fmt,
			      ap);
		va_end(ap);
	}
}

void mtk_memcfg_late_warning(unsigned long flag)
{
	mtk_memcfg_late_warning_flag |= flag;
}

/* kenerl memory information */

int parse_memory_layout_log(struct reserved_mem_ext *reserved_mem, int count, const char *log)
{
	int len;
	int ret = 0;
	unsigned long long addr;
	char buf[30];
	char *start, *end;
	char *name;

	if (count + 1 > MAX_RESERVED_REGIONS)
		return -1;

	/* reserve memory name */
	len = strlen(log);
	start = strchr(log, ']') + 1;
	end = strchr(start, ' ');
	name = kmalloc(end - start + 1, GFP_KERNEL);
	strncpy(name, start, end - start);
	name[end - start] = '\0';
	reserved_mem[count].name = name;

	/* reserve memory base */
	start = strstr(end, "0x");
	end = strchr(start, ' ');
	strncpy(buf, start, end - start);
	buf[end - start] = '\0';
	ret = kstrtoll(buf, 16, &addr);
	if (ret)
		return -1;
	reserved_mem[count].base = addr;

	/*reserve memory size */
	start = strchr(end, '(') + 1;
	end = strchr(start, ')');
	strncpy(buf, start, end - start);
	buf[end - start] = '\0';
	ret = kstrtoll(buf, 16, &addr);
	if (ret)
		return -1;
	reserved_mem[count].size = addr;

	reserved_mem[count].nomap = RESERVED_NO_MAP;
	count++;

	return count;
}

int parse_memory_layout_buf(struct reserved_mem_ext *reserved_mem)
{
	char *buf = mtk_memcfg_layout_buf.buf;
	char *start, *end;
	char temp[100];
	int count = 0;

	start = buf;
	end = strchr(start, '\n');
	while (end) {
		strncpy(temp, start, end - start);
		temp[end - start] = '\0';

		if (strstr(temp, "[PHY layout]")) {
			if (!strstr(temp, "kernel")) {
				count = parse_memory_layout_log(reserved_mem, count, temp);
				if (count == -1)
					return -1;
			}
		}

		start = end + 1;
		end = strchr(start, '\n');
	}

	return count;
}

static int mtk_memcfg_memory_layout_show(struct seq_file *m, void *v)
{
	int i = 0;
	int reserved_count = 0, mblock_version;
	phys_addr_t dram_end = 0;
	struct reserved_mem_ext *reserved_mem = NULL;
	struct reserved_mem_ext *rmem, *prmem;

	if (kptr_restrict == 2) {
		seq_puts(m, "Do not show memory layout because kptr_restrict level\n");
		goto debug_info;
	}

	reserved_mem = kcalloc(MAX_RESERVED_REGIONS,
				sizeof(struct reserved_mem_ext),
				GFP_KERNEL);
	if (!reserved_mem) {
		seq_puts(m, "Can't get memory to parse reserve memory.(Is it OOM?)\n");
		return 0;
	}

	mblock_version = check_mblock_support();
	seq_printf(m, "mblock version: %d\n", mblock_version);
	if (mblock_version >= 2) {
		reserved_count = mtk_memcfg_get_reserved_memory(reserved_mem);
		if (reserved_count == 0) {
			pr_info("Can't get reserve memory.\n");
			kfree(reserved_mem);
			goto debug_info;
		}

		reserved_count = mtk_memcfg_parse_reserved_memory(reserved_mem, reserved_count);
		if (reserved_count <= 0 || reserved_count > MAX_RESERVED_REGIONS) {
			seq_printf(m, "reserved_count(%d) over limit after parsing!\n",
					reserved_count);
			kfree(reserved_mem);
			goto debug_info;
		}
	} else if (mblock_version == 1) { /* parse memory layout buf for mblockV1 */
		reserved_count = parse_memory_layout_buf(reserved_mem);
		if (reserved_count <= 0 || reserved_count > MAX_RESERVED_REGIONS) {
			free_reserved_mem(reserved_mem, reserved_count);
			goto debug_info;
		}
	} else {
		kfree(reserved_mem);
		goto debug_info;
	}

	clean_reserved_mem_by_name(reserved_mem, reserved_count, "zone-movable-cma-memory");

	sort(reserved_mem, reserved_count,
			sizeof(struct reserved_mem_ext),
			reserved_mem_ext_compare, NULL);

	seq_puts(m, "Reserve Memory Layout (prefix with \"*\" is no-map)\n");

	/* get the last reserve memory */
	i = reserved_count - 1;
	rmem = &reserved_mem[i];
	dram_end = memblock_end_of_DRAM();

	if (dram_end > rmem->base + rmem->size) {
		mtk_memcfg_show_layout_region_kernel(m, dram_end,
		dram_end - (rmem->base + rmem->size));
	}

	for (i = reserved_count - 1; i >= 0; i--) {
		rmem = &reserved_mem[i];

		if (i == 0)
			prmem = NULL;
		else
			prmem = &reserved_mem[i - 1];

		if (rmem->size == 0 && rmem->base == 0)
			break;

		mtk_memcfg_show_layout_region(m, rmem->name,
				rmem->base + rmem->size,
				rmem->size, rmem->nomap, 0);

		if (prmem && prmem->base != 0 && rmem->base > prmem->base + prmem->size) {
			mtk_memcfg_show_layout_region_kernel(m, rmem->base,
					rmem->base - (prmem->base + prmem->size));
		}
	}

	rmem = &reserved_mem[0];
	while (rmem->base == 0)
		rmem++;

	if (rmem->base != memblock_start_of_DRAM()) {
		unsigned long size = (rmem->base) - memblock_start_of_DRAM();

		mtk_memcfg_show_layout_region(m, "kernel",
				memblock_start_of_DRAM() + size,
				size, RESERVED_MAP, 1);
	}

	if (mblock_version >= 2)
		kfree(reserved_mem);
	else
		free_reserved_mem(reserved_mem, reserved_count);

debug_info:
	seq_puts(m, "\n");
	seq_puts(m, "Debug Info:\n");
	seq_printf(m, "Memory: %lluK/%lluK available, %lluK kernel code, %lluK rwdata, %lluK rodata, %lluK init, %lluK bss, %lluK reserved"
#ifdef CONFIG_HIGHMEM
		", %lluK highmem"
#endif
		, kernel_reserve_meminfo.available >> 10
		, kernel_reserve_meminfo.total >> 10
		, kernel_reserve_meminfo.kernel_code >> 10
		, kernel_reserve_meminfo.rwdata >> 10
		, kernel_reserve_meminfo.rodata >> 10
		, kernel_reserve_meminfo.init >> 10
		, kernel_reserve_meminfo.bss >> 10
		, kernel_reserve_meminfo.reserved >> 10
#ifdef CONFIG_HIGHMEM
		, kernel_reserve_meminfo.highmem >> 10
#endif
		);
	seq_puts(m, "\n");

#ifdef CONFIG_SPARSEMEM_VMEMMAP
	seq_printf(m, "vmemmap : 0x%16lx - 0x%16lx   (%6ld MB actual)\n",
			(unsigned long)virt_to_page(PAGE_OFFSET),
			(unsigned long)virt_to_page(high_memory),
			((unsigned long)virt_to_page(high_memory) -
			 (unsigned long)virt_to_page(PAGE_OFFSET)) >> 20);
#else
#ifndef CONFIG_NEED_MULTIPLE_NODES
	seq_printf(m, "memmap : %lu MB\n", mem_map_size >> 20);
#endif
#endif
	return 0;
}

static void mtk_memcfg_show_layout_region(struct seq_file *m, const char *name,
		unsigned long long end, unsigned long long size, int nomap, int is_end)
{
	int i = 0;
	int name_length = strlen(name);
	int padding = (40 - name_length - 2) / 2;
	int odd = (40 - name_length - 2) % 2;

	seq_printf(m, "----------------------------------------  0x%08llx\n", end);
	seq_puts(m, "-");
	for (i = 0; i < padding; i++)
		seq_puts(m, " ");
	if (nomap) {
		seq_puts(m, "*");
		padding -= 1;
	}
	seq_printf(m, "%s", name);
	for (i = 0; i < padding + odd; i++)
		seq_puts(m, " ");
	seq_printf(m, "-  size : (0x%0llx)\n", size);

	if (is_end)
		seq_printf(m, "----------------------------------------  0x%0llx\n"
				, end - size);
}

static void mtk_memcfg_show_layout_region_kernel(struct seq_file *m,
		unsigned long long end, unsigned long long size)
{
	seq_printf(m, "----------------------------------------  0x%08llx\n", end);
	seq_printf(m, "-               kernel                 -  size : (0x%0llx)\n", size);
}

static int mtk_memcfg_memory_layout_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtk_memcfg_memory_layout_show, NULL);
}

/* end of kenerl memory information */

#ifdef CONFIG_MTK_ENG_BUILD
/* memblock reserve information */
static int mtk_memcfg_memblock_reserved_show(struct seq_file *m, void *v)
{
	int i = 0, j = 0, record_count = 0;
	unsigned long start, end, rstart, rend;
	unsigned long bt;
	struct memblock_stack_trace *trace;
	struct memblock_record *record;
	struct memblock_type *type = &memblock.reserved;
	struct memblock_region *region;
	unsigned long total_size = 0;

	record_count = min(memblock_reserve_count, MAX_MEMBLOCK_RECORD);

	for (i = 0; i < type->cnt; i++) {
		region = &type->regions[i];
		start = region->base;
		end = region->base + region->size;
		total_size += region->size;
		seq_printf(m, "region: %lx %lx-%lx\n", (unsigned long)region->size,
				start, end);
		for (j = 0; j < record_count; j++) {
			record = &memblock_record[j];
			trace = &memblock_stack_trace[j];
			rstart = record->base;
			rend = record->end;
			if ((rstart >= start && rstart < end) ||
				(rend > start && rend <= end) ||
				(rstart >= start && rend <= end)) {
				bt = trace->count - 3;
				seq_printf(m, "bt    : %lx %lx-%lx\n%pF %pF %pF %pF\n",
						(unsigned long)record->size,
						rstart, rend,
						(void *)trace->addrs[bt],
						(void *)trace->addrs[bt - 1],
						(void *)trace->addrs[bt - 2],
						(void *)trace->addrs[bt - 3]);
			}
		}
		seq_puts(m, "\n");
	}

	seq_printf(m, "Total memblock reserve count: %d\n", memblock_reserve_count);
	if (memblock_reserve_count >= MAX_MEMBLOCK_RECORD)
		seq_puts(m, "Total count > MAX_MEMBLOCK_RECORD\n");
	seq_printf(m, "Memblock reserve total size: 0x%lx\n", total_size);

	return 0;
}

static int mtk_memcfg_memblock_reserved_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtk_memcfg_memblock_reserved_show, NULL);
}
/* end of memblock reserve information */

/* kenerl memory fragmentation trigger */

static LIST_HEAD(frag_page_list);
static DEFINE_SPINLOCK(frag_page_list_lock);
static DEFINE_MUTEX(frag_task_lock);
static unsigned long mtk_memcfg_frag_round;
static struct kmem_cache *frag_page_cache;

struct frag_page {
	struct list_head list;
	struct page *page;
};

static int mtk_memcfg_frag_show(struct seq_file *m, void *v)
{
	int cnt = 0;
	struct frag_page *frag_page, *n_frag_page;

	spin_lock(&frag_page_list_lock);
	list_for_each_entry_safe(frag_page, n_frag_page,
			&frag_page_list, list) {
		cnt++;
	}
	spin_unlock(&frag_page_list_lock);
	seq_printf(m, "round: %lu, fragmentation-trigger held %d pages, %d MB\n",
		   mtk_memcfg_frag_round,
		   cnt, (cnt << PAGE_SHIFT) >> 20);

	return 0;
}

static int mtk_memcfg_frag_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtk_memcfg_frag_show, NULL);
}

static int do_fragmentation(void *n)
{
	struct frag_page *frag_page, *n_frag_page;
	struct page *page;
	gfp_t gfp_mask = GFP_ATOMIC;
	unsigned int max_order = 2;
	int cnt = 0, i;

	/* trigger fragmentation */
	/*
	 * Allocate an order-2-page, split it into 4 order-0-pages,
	 * and free 3 of them, repeatedly.
	 * In this way, we split all high order pages to
	 * order-0-pages and order-1-pages to create a
	 * fragmentation scenario.
	 *
	 * In current stage, we only trigger fragmentation in
	 * normal zone.
	 */
	while (1) {
#if 1
		if (cnt >= 10000) {
			/*
			 * release all memory and restart the fragmentation
			 * Allocating too much frag_page consumes
			 * too mush order-0 pages
			 */
			spin_lock(&frag_page_list_lock);
			list_for_each_entry_safe(frag_page, n_frag_page,
						 &frag_page_list, list) {
				list_del(&frag_page->list);
				__free_page(frag_page->page);
				kmem_cache_free(frag_page_cache, frag_page);
				cnt--;
			}
			spin_unlock(&frag_page_list_lock);
			pr_alert("round: %lu, fragmentation-trigger free pages %d left\n",
				 mtk_memcfg_frag_round, cnt);
		}
#endif
		while (1) {
			frag_page = kmem_cache_alloc(frag_page_cache, gfp_mask);
			if (!frag_page)
				break;
			page = alloc_pages(gfp_mask, max_order);
			if (!page) {
				kfree(frag_page);
				break;
			}
			split_page(page, 0);
			INIT_LIST_HEAD(&frag_page->list);
			frag_page->page = page;
			spin_lock(&frag_page_list_lock);
			list_add(&frag_page->list, &frag_page_list);
			spin_unlock(&frag_page_list_lock);
			for (i = 1; i < (1 << max_order); i++)
				__free_page(page + i);
			cnt++;
		}
		mtk_memcfg_frag_round++;
		pr_alert("round: %lu, fragmentation-trigger allocate %d pages %d MB\n",
			 mtk_memcfg_frag_round, cnt, (cnt << PAGE_SHIFT) >> 20);
		msleep(500);
	}

	return 0;
}

static ssize_t
mtk_memcfg_frag_write(struct file *file, const char __user *buffer,
		      size_t count, loff_t *pos)
{
	static char state;
	static struct task_struct *p;

	if (count > 0) {
		if (get_user(state, buffer))
			return -EFAULT;
		state -= '0';
		pr_alert("%s state = %d\n", __func__, state);

		mutex_lock(&frag_task_lock);
		if (state && !p) {
			pr_alert("activate do_fragmentation kthread\n");
			p = kthread_create(do_fragmentation, NULL,
					   "fragmentationd");
			if (!IS_ERR(p))
				wake_up_process(p);
			else
				p = 0;
		}
		mutex_unlock(&frag_task_lock);
	}
	return count;
}

/* end of kenerl memory fragmentation trigger */

static int mtk_memcfg_oom_show(struct seq_file *m, void *v)
{
	seq_puts(m, "oom-trigger\n");

	return 0;
}

static int mtk_memcfg_oom_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtk_memcfg_oom_show, NULL);
}

static void oom_reboot(unsigned long data)
{
	BUG();
}

static ssize_t
mtk_memcfg_oom_write(struct file *file, const char __user *buffer,
		      size_t count, loff_t *pos)
{
	static char state;
	struct timer_list timer;

	/* oom may cause system hang, reboot after 60 sec */
	init_timer(&timer);
	timer.function = oom_reboot;
	timer.expires = jiffies + 300 * HZ;
	add_timer(&timer);

	if (count > 0) {
		if (get_user(state, buffer))
			return -EFAULT;
		state -= '0';
		pr_alert("%s state = %d\n", __func__, state);
		if (state) {
			pr_alert("oom test, trying to kill system under oom scenario\n");
			/* exhaust all memory */
			for (;;)
				alloc_pages(GFP_HIGHUSER_MOVABLE, 0);
		}
	}
	return count;
}

/* end of kenerl out-of-memory(oom) trigger */
#endif /* end of CONFIG_MTK_ENG_BUILD */

static int __init mtk_memcfg_init(void)
{
	return 0;
}

static void __exit mtk_memcfg_exit(void)
{
}

static const struct file_operations mtk_memcfg_memory_layout_operations = {
	.open = mtk_memcfg_memory_layout_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

#ifdef CONFIG_MTK_ENG_BUILD
static const struct file_operations mtk_memcfg_memblock_reserved_operations = {
	.open = mtk_memcfg_memblock_reserved_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations mtk_memcfg_frag_operations = {
	.open = mtk_memcfg_frag_open,
	.write = mtk_memcfg_frag_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations mtk_memcfg_oom_operations = {
	.open = mtk_memcfg_oom_open,
	.write = mtk_memcfg_oom_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

#ifdef CONFIG_SLUB_DEBUG
static const struct file_operations proc_slabtrace_operations = {
	.open = slabtrace_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif
#endif /* end of CONFIG_MTK_ENG_BUILD */

static int __init mtk_memcfg_late_init(void)
{
	struct proc_dir_entry *entry = NULL;
	struct proc_dir_entry *mtk_memcfg_dir = NULL;

	mtk_memcfg_dir = proc_mkdir("mtk_memcfg", NULL);

	if (!mtk_memcfg_dir) {
		pr_err("[%s]: mkdir /proc/mtk_memcfg failed\n", __func__);
	} else {
		/* display kernel memory layout */
		entry = proc_create("memory_layout",
				    S_IRUGO | S_IWUSR, mtk_memcfg_dir,
				    &mtk_memcfg_memory_layout_operations);

		if (!entry)
			pr_err("create memory_layout proc entry failed\n");

#ifdef CONFIG_MTK_ENG_BUILD
		/* memblock reserved */
		entry = proc_create("memblock_reserved", S_IRUGO | S_IWUSR,
				mtk_memcfg_dir,
				&mtk_memcfg_memblock_reserved_operations);
		if (!entry)
			pr_err("create memblock_reserved proc entry failed\n");
		pr_info("create memblock_reserved proc entry success!!!!!\n");

		/* fragmentation test */
		entry = proc_create("frag-trigger",
				    S_IRUGO | S_IWUSR, mtk_memcfg_dir,
				    &mtk_memcfg_frag_operations);

		if (!entry)
			pr_err("create frag-trigger proc entry failed\n");

		frag_page_cache = kmem_cache_create("frag_page_cache",
						    sizeof(struct frag_page),
						    0, SLAB_PANIC, NULL);

		if (!frag_page_cache)
			pr_err("create frag_page_cache failed\n");

		/* oom test */
		entry = proc_create("oom-trigger",
				    S_IRUGO | S_IWUSR, mtk_memcfg_dir,
				    &mtk_memcfg_oom_operations);

		if (!entry)
			pr_err("create oom entry failed\n");

#ifdef CONFIG_SLUB_DEBUG
		/* slabtrace - full slub object backtrace */
		entry = proc_create("slabtrace",
				    S_IRUSR, mtk_memcfg_dir,
				    &proc_slabtrace_operations);

		if (!entry)
			pr_err("create slabtrace proc entry failed\n");
#endif
#endif /* end of CONFIG_MTK_ENG_BUILD */
	}
	mtk_memcfg_reserve_info_init(mtk_memcfg_dir);

	return 0;
}

module_init(mtk_memcfg_init);
module_exit(mtk_memcfg_exit);

static int __init mtk_memcfg_late_sanity_test(void)
{
#if 0
	/* trigger kernel warning if warning flag is set */
	if (mtk_memcfg_late_warning_flag & WARN_MEMBLOCK_CONFLICT) {
		aee_kernel_warning("[memory layout conflict]",
					mtk_memcfg_layout_buf.buf);
	}

	if (mtk_memcfg_late_warning_flag & WARN_MEMSIZE_CONFLICT) {
		aee_kernel_warning("[memory size conflict]",
					mtk_memcfg_layout_buf.buf);
	}

	if (mtk_memcfg_late_warning_flag & WARN_API_NOT_INIT) {
		aee_kernel_warning("[API is not initialized]",
					mtk_memcfg_layout_buf.buf);
	}

#ifdef CONFIG_HIGHMEM
	/* check highmem zone size */
	if (unlikely
		(totalhigh_pages && (totalhigh_pages << PAGE_SHIFT) < SZ_8M)) {
		aee_kernel_warning("[high zone lt 8MB]", __func__);
	}
#endif /* end of CONFIG_HIGHMEM */

#endif
	return 0;
}

/* scan memory layout */
#ifdef CONFIG_OF
static int __init dt_scan_memory(unsigned long node, const char *uname,
		int depth, void *data)
{
	const char *type = of_get_flat_dt_prop(node, "device_type", NULL);
	int i;
	int l;
	u64 kernel_mem_sz = 0;
	u64 phone_dram_sz = 0x0;	/* original phone DRAM size */
	u64 dram_sz = 0;	/* total DRAM size of all modules */
	struct dram_info *dram_info;
	struct mem_desc *mem_desc;
	struct mblock_info *mblock_info;
	const __be32 *reg, *endp;
	u64 fb_base = 0x12345678, fb_size = 0;

	/* We are scanning "memory" nodes only */
	if (type == NULL) {
		/*
		 * The longtrail doesn't have a device_type on the
		 * /memory node, so look for the node called /memory@0.
		 */
		if (depth != 1 || strcmp(uname, "memory@0") != 0)
			return 0;
	} else if (strcmp(type, "memory") != 0) {
		return 0;
	}

		reg = of_get_flat_dt_prop(node, "reg", &l);
	if (reg == NULL)
		return 0;

	endp = reg + (l / sizeof(__be32));

	while ((endp - reg) >= (dt_root_addr_cells + dt_root_size_cells)) {
		u64 base, size;

		base = dt_mem_next_cell(dt_root_addr_cells, &reg);
		size = dt_mem_next_cell(dt_root_size_cells, &reg);

		if (size == 0)
			continue;

		kernel_mem_sz += size;
	}

	/* orig_dram_info */
	dram_info = (struct dram_info *)of_get_flat_dt_prop(node,
			"orig_dram_info", NULL);
	if (dram_info) {
		for (i = 0; i < dram_info->rank_num; i++)
			phone_dram_sz += dram_info->rank_info[i].size;
	}

	/* mblock_info */
	mblock_info = (struct mblock_info *)of_get_flat_dt_prop(node,
			"mblock_info", NULL);
	if (mblock_info) {
		for (i = 0; i < mblock_info->mblock_num; i++)
			dram_sz += mblock_info->mblock[i].size;
	}

	/* lca reserved memory */
	mem_desc = (struct mem_desc *)of_get_flat_dt_prop(node,
			"lca_reserved_mem", NULL);
	if (mem_desc && mem_desc->size) {
		MTK_MEMCFG_LOG_AND_PRINTK(
			"[PHY layout]lca_reserved_mem   :   0x%08llx - 0x%08llx (0x%llx)\n",
			mem_desc->start,
			mem_desc->start +
			mem_desc->size - 1,
			mem_desc->size
			);
		dram_sz += mem_desc->size;
	}

	/* tee reserved memory */
	mem_desc = (struct mem_desc *)of_get_flat_dt_prop(node,
			"tee_reserved_mem", NULL);
	if (mem_desc && mem_desc->size) {
		MTK_MEMCFG_LOG_AND_PRINTK(
			"[PHY layout]tee_reserved_mem   :   0x%08llx - 0x%08llx (0x%llx)\n",
			mem_desc->start,
			mem_desc->start +
			mem_desc->size - 1,
			mem_desc->size
			);
		dram_sz += mem_desc->size;
	}

	/* frame buffer */
	fb_size = (u64)mtkfb_get_fb_size();
	fb_base = (u64)mtkfb_get_fb_base();

	dram_sz += fb_size;

	/* print memory information */
	MTK_MEMCFG_LOG_AND_PRINTK(
		"[debug]available DRAM size = 0x%llx\n[PHY layout]FB (dt) :  0x%llx - 0x%llx  (0x%llx)\n",
			(unsigned long long)kernel_mem_sz,
			(unsigned long long)fb_base,
			(unsigned long long)fb_base + fb_size - 1,
			(unsigned long long)fb_size);

	return node;
}

static int __init display_early_memory_info(void)
{
	int node;
	/* system memory */
	node = of_scan_flat_dt(dt_scan_memory, NULL);
	return 0;
}


#endif /* end of CONFIG_OF */

late_initcall(mtk_memcfg_late_init);
late_initcall(mtk_memcfg_late_sanity_test);
#ifdef CONFIG_OF
pure_initcall(display_early_memory_info);
#endif /* end of CONFIG_OF */

#if 0	/* test code of of_reserve */
/* test memory-reservd code */
phys_addr_t test_base = 0;
phys_addr_t test_size = 0;
reservedmem_of_init_fn reserve_memory_test_fn(struct reserved_mem *rmem,
				      unsigned long node, const char *uname)
{
	pr_alert("%s, name: %s, uname: %s, base: 0x%llx, size: 0x%llx\n",
		 __func__,  rmem->name, uname,
		 (unsigned long long)rmem->base,
		 (unsigned long long)rmem->size);
	/* memblock_free(rmem->base, rmem->size); */
	test_base = rmem->base;
	test_size = rmem->size;

	return 0;
}

static int __init init_test_reserve_memory(void)
{
	void *p = 0;

	p = ioremap(test_base, (size_t)test_size);
	if (p) {
		pr_alert("%s:%d ioremap ok: %p\n", __func__, __LINE__,
			 p);
	} else {
		pr_alert("%s:%d ioremap failed\n", __func__, __LINE__);
	}
	return 0;
}
late_initcall(init_test_reserve_memory);

reservedmem_of_init_fn mrdump_reserve_initfn(struct reserved_mem *rmem,
				      unsigned long node, const char *uname)
{
	pr_alert("%s, name: %s, uname: %s, base: 0x%llx, size: 0x%llx\n",
		 __func__,  rmem->name, uname,
		 (unsigned long long)rmem->base,
		 (unsigned long long)rmem->size);

	return 0;
}

RESERVEDMEM_OF_DECLARE(reserve_memory_test1, "reserve-memory-test",
		       reserve_memory_test_fn);
RESERVEDMEM_OF_DECLARE(mrdump_reserved_memory, "mrdump-reserved-memory",
		       mrdump_reserve_initfn);
#endif /* end of test code of of_reserve */
