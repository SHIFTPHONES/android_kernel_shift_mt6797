/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/hardirq.h>
#include <linux/init.h>
#include <linux/kallsyms.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <disp_assert_layer.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <mt-plat/aee.h>
#include <linux/seq_file.h>
#include <linux/jiffies.h>
#include <linux/ptrace.h>
#include <asm/stacktrace.h>
#include "aed.h"
#include <linux/pid.h>
#include <mt-plat/mt_boot_common.h>


#ifdef CONFIG_MTK_ION
#include <mtk/ion_drv.h>
#endif

#ifdef CONFIG_MTK_GPU_SUPPORT
#include <mt-plat/mtk_gpu_utility.h>
#endif

static DEFINE_SPINLOCK(pwk_hang_lock);
static int wdt_kick_status;
static int hwt_kick_times;
static int pwk_start_monitor;

#define AEEIOCTL_RT_MON_Kick _IOR('p', 0x0A, int)
#define MaxHangInfoSize (1024*1024)
#define MAX_STRING_SIZE 256
char Hang_Info[MaxHangInfoSize];	/* 1M info */
static int Hang_Info_Size;
static bool Hang_Detect_first;


#define HD_PROC "hang_detect"

/* static DEFINE_SPINLOCK(hd_locked_up); */
#define HD_INTER 30

static int hd_detect_enabled;
static int hd_timeout = 0x7fffffff;
static int hang_detect_counter = 0x7fffffff;
static int dump_bt_done;
DECLARE_WAIT_QUEUE_HEAD(dump_bt_start_wait);
DECLARE_WAIT_QUEUE_HEAD(dump_bt_done_wait);
static long monitor_hang_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
#ifdef CONFIG_MTK_ENG_BUILD
#define SEQ_printf(m, x...) \
do {                \
	if (m)          \
		seq_printf(m, x);   \
	else            \
		pr_debug(x);        \
} while (0)



static int monitor_hang_show(struct seq_file *m, void *v)
{
	SEQ_printf(m, "[Hang_Detect] show Hang_info size %d\n ", (int)strlen(Hang_Info));
	SEQ_printf(m, "%s", Hang_Info);
	return 0;
}

static int monitor_hang_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, monitor_hang_show, inode->i_private);
}


static ssize_t monitor_hang_proc_write(struct file *filp, const char *ubuf, size_t cnt, loff_t *data)
{
	return 0;
}

static const struct file_operations monitor_hang_fops = {
	.open = monitor_hang_proc_open,
	.write = monitor_hang_proc_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif
/******************************************************************************
 * hang detect File operations
 *****************************************************************************/
static int monitor_hang_open(struct inode *inode, struct file *filp)
{
	/* LOGD("%s\n", __func__); */
	/* aee_kernel_RT_Monitor_api (600) ; */
	return 0;
}

static int monitor_hang_release(struct inode *inode, struct file *filp)
{
	/* LOGD("%s\n", __func__); */
	return 0;
}

static unsigned int monitor_hang_poll(struct file *file, struct poll_table_struct *ptable)
{
	/* LOGD("%s\n", __func__); */
	return 0;
}

static ssize_t monitor_hang_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	/* LOGD("%s\n", __func__); */
	return 0;
}

static ssize_t monitor_hang_write(struct file *filp, const char __user *buf, size_t count,
		loff_t *f_pos)
{

	/* LOGD("%s\n", __func__); */
	return 0;
}


/* QHQ RT Monitor */
/* QHQ RT Monitor    end */




/*
 * aed process daemon and other command line may access me
 * concurrently
 */
static long monitor_hang_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	static long long monitor_status;

	if (cmd == AEEIOCTL_WDT_KICK_POWERKEY) {
		if ((int)arg == WDT_SETBY_WMS_DISABLE_PWK_MONITOR) {
			/* pwk_start_monitor=0; */
			/* wdt_kick_status=0; */
			/* hwt_kick_times=0; */
		} else if ((int)arg == WDT_SETBY_WMS_ENABLE_PWK_MONITOR) {
			/* pwk_start_monitor=1; */
			/* wdt_kick_status=0; */
			/* hwt_kick_times=0; */
		} else if ((int)arg < 0xf) {
			aee_kernel_wdt_kick_Powkey_api("Powerkey ioctl", (int)arg);
		}
		return ret;

	}
	/* QHQ RT Monitor */
	if (cmd == AEEIOCTL_RT_MON_Kick) {
		LOGE("AEEIOCTL_RT_MON_Kick ( %d)\n", (int)arg);
		aee_kernel_RT_Monitor_api((int)arg);
		return ret;
	}
	/* LOGE("AEEIOCTL_RT_MON_Kick unknown cmd :(%d)( %d)\n",(int)cmd, (int)arg); */
	/* LOGE("AEEIOCTL_RT_MON_Kick known cmd :(%d)( %d)\n",
	   (int)AEEIOCTL_WDT_KICK_POWERKEY,
	   (int)AEEIOCTL_RT_MON_Kick); */
	/* QHQ RT Monitor end */

	if ((cmd == AEEIOCTL_SET_SF_STATE) && (!strncmp(current->comm, "surfaceflinger", 10) ||
						!strncmp(current->comm, "SWWatchDog", 10))) {
		if (copy_from_user(&monitor_status, (void __user *)arg, sizeof(long long)))
			ret = -1;
		LOGE("AEE_MONITOR_SET[status]: 0x%llx", monitor_status);
		return ret;
	} else if (cmd == AEEIOCTL_GET_SF_STATE) {
		if (copy_to_user((void __user *)arg, &monitor_status, sizeof(long long)))
			ret = -1;
		return ret;
	}

	return -1;
}




/* QHQ RT Monitor */
static const struct file_operations aed_wdt_RT_Monitor_fops = {
	.owner = THIS_MODULE,
	.open = monitor_hang_open,
	.release = monitor_hang_release,
	.poll = monitor_hang_poll,
	.read = monitor_hang_read,
	.write = monitor_hang_write,
	.unlocked_ioctl = monitor_hang_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = monitor_hang_ioctl,
#endif
};


static struct miscdevice aed_wdt_RT_Monitor_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "RT_Monitor",
	.fops = &aed_wdt_RT_Monitor_fops,
};



/* bleow code is added for monitor_hang_init */
static int monitor_hang_init(void);

static int hang_detect_init(void);
/* bleow code is added for hang detect */



static int __init monitor_hang_init(void)
{
	int err = 0;
#ifdef CONFIG_MTK_ENG_BUILD
	struct proc_dir_entry *pe;
#endif
	/* bleow code is added by QHQ  for hang detect */
	err = misc_register(&aed_wdt_RT_Monitor_dev);
	if (unlikely(err)) {
		pr_err("aee: failed to register aed_wdt_RT_Monitor_dev device!\n");
		return err;
	}
	hang_detect_init();
	/* bleow code is added by QHQ  for hang detect */
	/* end */
#ifdef CONFIG_MTK_ENG_BUILD
	pe = proc_create("monitor_hang", 0664, NULL, &monitor_hang_fops);
	if (!pe)
		return -ENOMEM;
#endif
	return err;
}

static void __exit monitor_hang_exit(void)
{
	int err;

	err = misc_deregister(&aed_wdt_RT_Monitor_dev);
	if (unlikely(err))
		LOGE("failed to unregister RT_Monitor device!\n");
}




static int FindTaskByName(char *name)
{
	struct task_struct *task;
	int ret = -1;

	if (!name)
		return ret;

	read_lock(&tasklist_lock);
	for_each_process(task) {
		if (task && (strncmp(task->comm, name, strlen(name)) == 0)) {
			LOGE("[Hang_Detect] %s found pid:%d.\n", task->comm, task->pid);
			ret = task->pid;
			break;
		}
	}
	read_unlock(&tasklist_lock);
	return ret;
}
static void Log2HangInfo(const char *fmt, ...)
{
	unsigned long len = 0;
	static int times;
	va_list ap;

	LOGV("len 0x%lx+++, times:%d, MaxSize:%lx\n", (long)(Hang_Info_Size), times++,
	     (unsigned long)MaxHangInfoSize);
	if ((Hang_Info_Size + MAX_STRING_SIZE) >= (unsigned long)MaxHangInfoSize) {
		LOGE("HangInfo Buffer overflow len(0x%x), MaxHangInfoSize:0x%lx !!!!!!!\n",
		     Hang_Info_Size, (long)MaxHangInfoSize);
		return;
	}
	va_start(ap, fmt);
	len = vscnprintf(&Hang_Info[Hang_Info_Size], MAX_STRING_SIZE, fmt, ap);
	va_end(ap);
	Hang_Info_Size += len;
}


static void get_kernel_bt(struct task_struct *tsk)
{
	struct stack_trace trace;
	unsigned long stacks[32];
	int i;

	trace.entries = stacks;
	/*save backtraces */
	trace.nr_entries = 0;
	trace.max_entries = 32;
	trace.skip = 0;
	save_stack_trace_tsk(tsk, &trace);
	for (i = 0; i < trace.nr_entries; i++)
		Log2HangInfo("[<%lx>] %pS\n", (long)trace.entries[i], (void *)trace.entries[i]);
}


void sched_show_task_local(struct task_struct *p)
{
	unsigned long free = -1;
	int ppid;
	unsigned state;
	char stat_nam[] = TASK_STATE_TO_CHAR_STR;
	pid_t pid;

	state = p->state ? __ffs(p->state) + 1 : 0;
	LOGV("%-15.15s %c", p->comm, state < sizeof(stat_nam) - 1 ? stat_nam[state] : '?');
#if BITS_PER_LONG == 32
	if (state == TASK_RUNNING)
		LOGV(" running  ");
	else
		LOGV(" %08lx ", thread_saved_pc(p));
#else
	if (state == TASK_RUNNING)
		LOGV("  running task    ");
	else
		LOGV(" %016lx ", thread_saved_pc(p));
#endif
#ifdef CONFIG_DEBUG_STACK_USAGE
	free = stack_not_used(p);
#endif
	rcu_read_lock();
	ppid = task_pid_nr(rcu_dereference(p->real_parent));
	pid = task_pid_nr(p);
	rcu_read_unlock();
	LOGV("%5lu %5d %6d 0x%08lx\n", free,
	     task_pid_nr(p), ppid, (unsigned long)task_thread_info(p)->flags);

	Log2HangInfo("%-15.15s %c ", p->comm, state < sizeof(stat_nam) - 1 ? stat_nam[state] : '?');
	Log2HangInfo("%5lu %5d %6d 0x%08lx\n", free, task_pid_nr(p), ppid,
		     (unsigned long)task_thread_info(p)->flags);
	get_kernel_bt(p);	/* Catch kernel-space backtrace */

}

static int DumpThreadNativeMaps(pid_t pid)
{
	struct task_struct *current_task;
	struct vm_area_struct *vma;
	int mapcount = 0;
	struct file *file;
	int flags;
	struct mm_struct *mm;
	struct pt_regs *user_ret;

	current_task = find_task_by_vpid(pid);	/* get tid task */
	if (current_task == NULL)
		return -ESRCH;
	user_ret = task_pt_regs(current_task);

	if (!user_mode(user_ret)) {
		LOGE(" %s,%d:%s: in user_mode", __func__, pid, current_task->comm);
		return -1;
	}

	if (current_task->mm == NULL) {
		LOGE(" %s,%d:%s: current_task->mm == NULL", __func__, pid, current_task->comm);
		return -1;
	}

	vma = current_task->mm->mmap;
	Log2HangInfo("Dump native maps files:\n");
	while (vma && (mapcount < current_task->mm->map_count)) {
		file = vma->vm_file;
		flags = vma->vm_flags;
		if (file) {	/* !!!!!!!!only dump 1st mmaps!!!!!!!!!!!! */
			LOGV("%08lx-%08lx %c%c%c%c    %s\n", vma->vm_start, vma->vm_end,
			     flags & VM_READ ? 'r' : '-',
			     flags & VM_WRITE ? 'w' : '-',
			     flags & VM_EXEC ? 'x' : '-',
			     flags & VM_MAYSHARE ? 's' : 'p',
			     (unsigned char *)(file->f_path.dentry->d_iname));

			if (flags & VM_EXEC) {	/* we only catch code section for reduce maps space */
				Log2HangInfo("%08lx-%08lx %c%c%c%c    %s\n", vma->vm_start,
					     vma->vm_end, flags & VM_READ ? 'r' : '-',
					     flags & VM_WRITE ? 'w' : '-',
					     flags & VM_EXEC ? 'x' : '-',
					     flags & VM_MAYSHARE ? 's' : 'p',
					     (unsigned char *)(file->f_path.dentry->d_iname));
			}
		} else {
			const char *name = arch_vma_name(vma);

			mm = vma->vm_mm;
			if (!name) {
				if (mm) {
					if (vma->vm_start <= mm->start_brk &&
					    vma->vm_end >= mm->brk) {
						name = "[heap]";
					} else if (vma->vm_start <= mm->start_stack &&
						   vma->vm_end >= mm->start_stack) {
						name = "[stack]";
					}
				} else {
					name = "[vdso]";
				}
			}

			LOGV("%08lx-%08lx %c%c%c%c    %s\n", vma->vm_start, vma->vm_end,
			     flags & VM_READ ? 'r' : '-',
			     flags & VM_WRITE ? 'w' : '-',
			     flags & VM_EXEC ? 'x' : '-', flags & VM_MAYSHARE ? 's' : 'p', name);
			if (flags & VM_EXEC) {
				Log2HangInfo("%08lx-%08lx %c%c%c%c    %s\n", vma->vm_start,
					     vma->vm_end, flags & VM_READ ? 'r' : '-',
					     flags & VM_WRITE ? 'w' : '-',
					     flags & VM_EXEC ? 'x' : '-',
					     flags & VM_MAYSHARE ? 's' : 'p', name);
			}
		}
		vma = vma->vm_next;
		mapcount++;
	}

	return 0;
}



static int DumpThreadNativeInfo_By_tid(pid_t tid)
{
	struct task_struct *current_task;
	struct pt_regs *user_ret;
	struct vm_area_struct *vma;
	unsigned long userstack_start = 0;
	unsigned long userstack_end = 0, length = 0;
	int ret = -1;

	/* current_task = get_current(); */
	current_task = find_task_by_vpid(tid);	/* get tid task */
	if (current_task == NULL)
		return -ESRCH;
	user_ret = task_pt_regs(current_task);

	if (!user_mode(user_ret)) {
		LOGE(" %s,%d:%s,fail in user_mode", __func__, tid, current_task->comm);
		return ret;
	}

	if (current_task->mm == NULL) {
		LOGE(" %s,%d:%s, current_task->mm == NULL", __func__, tid, current_task->comm);
		return ret;
	}
#ifndef __aarch64__		/* 32bit */
	LOGV(" pc/lr/sp 0x%08lx/0x%08lx/0x%08lx\n", user_ret->ARM_pc, user_ret->ARM_lr,
	     user_ret->ARM_sp);

	userstack_start = (unsigned long)user_ret->ARM_sp;
	vma = current_task->mm->mmap;

	while (vma != NULL) {
		if (vma->vm_start <= userstack_start && vma->vm_end >= userstack_start) {
			userstack_end = vma->vm_end;
			break;
		}
		vma = vma->vm_next;
		if (vma == current_task->mm->mmap)
			break;
	}

	if (userstack_end == 0) {
		LOGE(" %s,%d:%s,userstack_end == 0", __func__, tid, current_task->comm);
		return ret;
	}
	LOGV("Dump K32 stack range (0x%08lx:0x%08lx)\n", userstack_start, userstack_end);
	length = userstack_end - userstack_start;


	/* dump native stack to buffer */
	{
		unsigned long SPStart = 0, SPEnd = 0;
		int tempSpContent[4], copied;

		SPStart = userstack_start;
		SPEnd = SPStart + length;
		Log2HangInfo("UserSP_start:0x%016x: UserSP_Length:0x%08x ,UserSP_End:0x%016x\n",
				SPStart, length, SPEnd);
		while (SPStart < SPEnd) {
			/* memcpy(&tempSpContent[0],(void *)(userstack_start+i),4*4); */
			copied =
			    access_process_vm(current_task, SPStart, &tempSpContent,
					      sizeof(tempSpContent), 0);
			if (copied != sizeof(tempSpContent)) {
				LOGE("access_process_vm  SPStart error,sizeof(tempSpContent)=%x\n",
				     (unsigned int)sizeof(tempSpContent));
				/* return -EIO; */
			}
			if (tempSpContent[0] != 0 || tempSpContent[1] != 0 ||
					tempSpContent[2] != 0 || tempSpContent[3] != 0) {
				Log2HangInfo("0x%08x:%08x %08x %08x %08x\n", SPStart, tempSpContent[0],
				     tempSpContent[1], tempSpContent[2], tempSpContent[3]);
			}
			SPStart += 4 * 4;
		}
	}
	LOGV("u+k 32 copy_from_user ret(0x%08x),len:%lx\n", ret, length);
	LOGV("end dump native stack:\n");
#else				/* 64bit, First deal with K64+U64, the last time to deal with K64+U32 */

	/* K64_U32 for current task */
	if (compat_user_mode(user_ret)) {	/* K64_U32 for check reg */
		LOGV(" K64+ U32 pc/lr/sp 0x%16lx/0x%16lx/0x%16lx\n",
		     (long)(user_ret->user_regs.pc),
		     (long)(user_ret->user_regs.regs[14]), (long)(user_ret->user_regs.regs[13]));
		userstack_start = (unsigned long)user_ret->user_regs.regs[13];
		vma = current_task->mm->mmap;
		while (vma != NULL) {
			if (vma->vm_start <= userstack_start && vma->vm_end >= userstack_start) {
				userstack_end = vma->vm_end;
				break;
			}
			vma = vma->vm_next;
			if (vma == current_task->mm->mmap)
				break;
		}

		if (userstack_end == 0) {
			LOGE("Dump native stack failed:\n");
			return ret;
		}

		LOGV("Dump K64+ U32 stack range (0x%08lx:0x%08lx)\n", userstack_start,
		     userstack_end);
		length = userstack_end - userstack_start;

		/*  dump native stack to buffer */
		{
			unsigned long SPStart = 0, SPEnd = 0;
			int tempSpContent[4], copied;

			SPStart = userstack_start;
			SPEnd = SPStart + length;
			Log2HangInfo("UserSP_start:0x%016x: UserSP_Length:0x%08x ,UserSP_End:0x%016x\n",
				SPStart, length, SPEnd);
			while (SPStart < SPEnd) {
				/*  memcpy(&tempSpContent[0],(void *)(userstack_start+i),4*4); */
				copied =
				    access_process_vm(current_task, SPStart, &tempSpContent,
						      sizeof(tempSpContent), 0);
				if (copied != sizeof(tempSpContent)) {
					LOGE("access_process_vm  SPStart error,sizeof(tempSpContent)=%x\n",
						(unsigned int)sizeof(tempSpContent));
					/* return -EIO; */
				}
				if (tempSpContent[0] != 0 || tempSpContent[1] != 0 ||
					tempSpContent[2] != 0 || tempSpContent[3] != 0) {
					Log2HangInfo("0x%08x:%08x %08x %08x %08x\n", SPStart,
						     tempSpContent[0], tempSpContent[1], tempSpContent[2],
						     tempSpContent[3]);
				}
				SPStart += 4 * 4;
			}
		}
	} else {		/*K64+U64 */
		LOGV(" K64+ U64 pc/lr/sp 0x%16lx/0x%16lx/0x%16lx\n",
		     (long)(user_ret->user_regs.pc),
		     (long)(user_ret->user_regs.regs[30]), (long)(user_ret->user_regs.sp));
		userstack_start = (unsigned long)user_ret->user_regs.sp;
		vma = current_task->mm->mmap;

		while (vma != NULL) {
			if (vma->vm_start <= userstack_start && vma->vm_end >= userstack_start) {
				userstack_end = vma->vm_end;
				break;
			}
			vma = vma->vm_next;
			if (vma == current_task->mm->mmap)
				break;
		}
		if (userstack_end == 0) {
			LOGE("Dump native stack failed:\n");
			return ret;
		}

		{
			unsigned long tmpfp, tmp, tmpLR;
			int copied, frames;
			unsigned long native_bt[16];

			native_bt[0] = user_ret->user_regs.pc;
			native_bt[1] = user_ret->user_regs.regs[30];
			tmpfp = user_ret->user_regs.regs[29];
			frames = 2;
			while (tmpfp < userstack_end && tmpfp > userstack_start) {
				copied =
				    access_process_vm(current_task, (unsigned long)tmpfp, &tmp,
						      sizeof(tmp), 0);
				if (copied != sizeof(tmp)) {
					LOGE("access_process_vm  fp error\n");
					return -EIO;
				}
				copied =
				    access_process_vm(current_task, (unsigned long)tmpfp + 0x08,
						      &tmpLR, sizeof(tmpLR), 0);
				if (copied != sizeof(tmpLR)) {
					LOGE("access_process_vm  pc error\n");
					return -EIO;
				}
				tmpfp = tmp;
				native_bt[frames] = tmpLR;
				frames++;
				if (frames >= 16)
					break;
			}
			for (copied = 0; copied < frames; copied++) {
				LOGV("frame:#%d: pc(%016lx)\n", copied, native_bt[copied]);
				/*  #00 pc 000000000006c760  /system/lib64/ libc.so (__epoll_pwait+8) */
				Log2HangInfo(" #%d pc %016lx\n", copied, native_bt[copied]);
			}
			LOGE("tid(%d:%s),frame %d. tmpfp(0x%lx),userstack_start(0x%lx),userstack_end(0x%lx)\n",
				tid, current_task->comm, frames, tmpfp, userstack_start, userstack_end);
		}
	}
#endif

	return 0;
}

static void show_bt_by_pid(int task_pid)
{
	struct task_struct *t, *p;
	struct pid *pid;
	int count = 0;
	unsigned state;
	char stat_nam[] = TASK_STATE_TO_CHAR_STR;

	pid = find_get_pid(task_pid);
	t = p = get_pid_task(pid, PIDTYPE_PID);

	if (p != NULL) {
		LOGE("show_bt_by_pid: %d: %s\n", task_pid, t->comm);
		Log2HangInfo("show_bt_by_pid: %d: %s.\n", task_pid, t->comm);
		DumpThreadNativeMaps(task_pid);	/* catch maps to Userthread_maps */
		do {
			if (t) {
				pid_t tid = 0;

				tid = task_pid_vnr(t);
				state = t->state ? __ffs(t->state) + 1 : 0;
				LOGV("lhd: %-15.15s %c pid(%d),tid(%d)",
				     t->comm, state < sizeof(stat_nam) - 1 ? stat_nam[state] : '?',
				     task_pid, tid);

				sched_show_task_local(t);	/* catch kernel bt */

				Log2HangInfo("%s sysTid=%d, pid=%d\n", t->comm, tid, task_pid);

				do_send_sig_info(SIGSTOP, SEND_SIG_FORCED, t, true);
				/* change send ptrace_stop to send signal stop */
				DumpThreadNativeInfo_By_tid(tid);	/* catch user-space bt */


				/*
				* !!!!!!!!!do need to send sigcontinue\A3\BF\A3\BF\A3\BF
				* if thread has been stopped before do_send_sig_info(SIGSTOP, SEND_SIG_FORCED, t, true),
				* so need not to send sigcont
				*/
				if (stat_nam[state] != 'T')	/* change send ptrace_stop to send signal stop */
					do_send_sig_info(SIGCONT, SEND_SIG_FORCED, t, true);
			}
			if ((++count) % 5 == 4)
				msleep(20);
			Log2HangInfo("---\n");
		} while_each_thread(p, t);
		put_task_struct(t);
	}
	put_pid(pid);
}

static void show_state_filter_local(void)
{
	struct task_struct *g, *p;

	do_each_thread(g, p) {
		/*
		 * reset the NMI-timeout, listing all files on a slow
		 * console might take a lot of time:
		 *discard wdtk-* for it always stay in D state
		 */
		if ((p->state == TASK_RUNNING || p->state & TASK_UNINTERRUPTIBLE) && !strstr(p->comm, "wdtk"))
			sched_show_task_local(p);
	} while_each_thread(g, p);
}


static void ShowStatus(void)
{
#define DUMP_PROCESS_NUM 10
	int dumppids[DUMP_PROCESS_NUM];
	int dump_count = 0;
	int i = 0;
	struct task_struct *task;

	read_lock(&tasklist_lock);
	for_each_process(task) {
		if (dump_count >= DUMP_PROCESS_NUM)
			break;

		if ((strcmp(task->comm, "surfaceflinger") == 0) || (strcmp(task->comm, "init") == 0) ||
			(strcmp(task->comm, "system_server") == 0) || (strcmp(task->comm, "mmcqd/0") == 0) ||
			(strcmp(task->comm, "debuggerd64") == 0) || (strcmp(task->comm, "mmcqd/1") == 0) ||
			(strcmp(task->comm, "debuggerd") == 0)) {
			dumppids[dump_count++] = task->pid;
			continue;
		}
	}
	read_unlock(&tasklist_lock);

	for (i = 0; i < dump_count; i++)
		show_bt_by_pid(dumppids[i]);

	show_state_filter_local();

	/* debug_locks = 1; */
	debug_show_all_locks();

	show_free_areas(0);

#ifdef CONFIG_MTK_ION
	ion_mm_heap_memory_detail();
#endif
#ifdef CONFIG_MTK_GPU_SUPPORT
	if (mtk_dump_gpu_memory_usage() == false)
		LOGE("[Hang_Detect] mtk_dump_gpu_memory_usage not support\n");
#endif
}

void reset_hang_info(void)
{
	Hang_Detect_first = false;
	memset(&Hang_Info, 0, MaxHangInfoSize);
	Hang_Info_Size = 0;
}

static int hang_detect_dump_thread(void *arg)
{

	/* unsigned long flags; */
	struct sched_param param = {
		.sched_priority = 99
	};

	sched_setscheduler(current, SCHED_FIFO, &param);
	msleep(120 * 1000);
	dump_bt_done = 1;
	while (1) {
		wait_event_interruptible(dump_bt_start_wait, dump_bt_done == 0);
		ShowStatus();
		dump_bt_done = 1;
		wake_up_interruptible(&dump_bt_done_wait);
	}
	pr_err("[Hang_Detect] hang_detect dump thread exit.\n");
	return 0;
}

static int hang_detect_thread(void *arg)
{

	/* unsigned long flags; */
	struct sched_param param = {
		.sched_priority = 99
	};

	sched_setscheduler(current, SCHED_FIFO, &param);
	reset_hang_info();
	msleep(120 * 1000);
	pr_debug("[Hang_Detect] hang_detect thread starts.\n");
	while (1) {
		pr_info("[Hang_Detect] hang_detect thread counts down %d:%d, status %d.\n",
				hang_detect_counter, hd_timeout, hd_detect_enabled);
		if ((hd_detect_enabled == 1)
		    && (FindTaskByName("system_server") != -1)) {

			if (hang_detect_counter <= 0) {
				Log2HangInfo("[Hang_detect]Dump the %d time process bt.\n", Hang_Detect_first ? 2 : 1);
				dump_bt_done = 0;
				wake_up_interruptible(&dump_bt_start_wait);
				if (dump_bt_done != 1)
					wait_event_interruptible_timeout(dump_bt_done_wait, dump_bt_done == 1, HZ*10);

				if (Hang_Detect_first == true) {
					pr_err("[Hang_Detect] aee mode is %d, we should triger KE...\n", aee_mode);
					BUG();
				} else
					Hang_Detect_first = true;
			}
			hang_detect_counter--;
		} else {
			/* incase of system_server restart, we give 2 mins more.(4*HD_INTER) */
			if (hd_detect_enabled == 1) {
				hang_detect_counter = hd_timeout + 4;
				hd_detect_enabled = 0;
			}
			reset_hang_info();
		}

		msleep((HD_INTER) * 1000);
	}
	return 0;
}

void hd_test(void)
{
	hang_detect_counter = 0;
	hd_timeout = 0;
}

void aee_kernel_RT_Monitor_api(int lParam)
{
	if (0 == lParam) {
		hd_detect_enabled = 0;
		hang_detect_counter =
			hd_timeout;
		pr_info("[Hang_Detect] hang_detect disabled\n");
	} else if (lParam > 0) {
		/* lParem=0x1000|timeout,only set in aee call when NE in system_server
		*  so only change hang_detect_counter when call from AEE
		*  Others ioctl, will change hd_detect_enabled & hang_detect_counter
		*/
		if (lParam & 0x1000) {
			hang_detect_counter =
			hd_timeout = ((long)(lParam & 0x0fff) + HD_INTER - 1) / (HD_INTER);
		} else {
			hd_detect_enabled = 1;
			hang_detect_counter =
				hd_timeout = ((long)lParam + HD_INTER - 1) / (HD_INTER);
		}
		pr_info("[Hang_Detect] hang_detect enabled %d\n", hd_timeout);
	}
}

int hang_detect_init(void)
{

	struct task_struct *hd_thread;

	pr_debug("[Hang_Detect] Initialize proc\n");
	hd_thread = kthread_create(hang_detect_thread, NULL, "hang_detect");
	if (hd_thread != NULL)
		wake_up_process(hd_thread);

	hd_thread = kthread_create(hang_detect_dump_thread, NULL, "hang_detect1");
	if (hd_thread != NULL)
		wake_up_process(hd_thread);

	return 0;
}

/* added by QHQ  for hang detect */
/* end */


int aee_kernel_Powerkey_is_press(void)
{
	int ret = 0;

	ret = pwk_start_monitor;
	return ret;
}
EXPORT_SYMBOL(aee_kernel_Powerkey_is_press);

void aee_kernel_wdt_kick_Powkey_api(const
		char
		*module,
		int msg)
{
	spin_lock(&pwk_hang_lock);
	wdt_kick_status |= msg;
	spin_unlock(&pwk_hang_lock);
	/*  //reduce kernel log
	if (pwk_start_monitor)
		LOGE("powerkey_kick:%s:%x,%x\r", module, msg, wdt_kick_status);
	*/

}
EXPORT_SYMBOL
(aee_kernel_wdt_kick_Powkey_api);


void aee_powerkey_notify_press(unsigned long
		pressed) {
	if (pressed) {	/* pwk down or up ???? need to check */
		spin_lock(&pwk_hang_lock);
		wdt_kick_status = 0;
		spin_unlock(&pwk_hang_lock);
		hwt_kick_times = 0;
		pwk_start_monitor = 1;
		LOGE("(%s) HW keycode powerkey\n", pressed ? "pressed" : "released");
	}
}
EXPORT_SYMBOL(aee_powerkey_notify_press);

void get_hang_detect_buffer(unsigned long *addr, unsigned long *size,
			    unsigned long *start)
{
	*addr = (unsigned long)Hang_Info;
	*start = 0;
	*size = MaxHangInfoSize;
}

int aee_kernel_wdt_kick_api(int kinterval)
{
	int ret = 0;

	if (pwk_start_monitor
			&& (get_boot_mode() ==
				NORMAL_BOOT)
			&&
			(FindTaskByName("system_server")
			 != -1)) {
		/* Only in normal_boot! */
		LOGE("Press powerkey!!	g_boot_mode=%d,wdt_kick_status=0x%x,tickTimes=0x%x,g_kinterval=%d,RT[%lld]\n",
				get_boot_mode(), wdt_kick_status, hwt_kick_times, kinterval, sched_clock());
		hwt_kick_times++;
		if ((kinterval * hwt_kick_times > 180)) {	/* only monitor 3 min */
			pwk_start_monitor =
				0;
			/* check all modules is ok~~~ */
			if ((wdt_kick_status & (WDT_SETBY_Display | WDT_SETBY_SF))
					!= (WDT_SETBY_Display | WDT_SETBY_SF)) {
				if (aee_mode != AEE_MODE_CUSTOMER_USER)	{	/* disable for display not ready */
					/* ShowStatus();  catch task kernel bt */
						/* LOGE("[WDK] Powerkey Tick fail,kick_status 0x%08x,RT[%lld]\n ", */
						/* wdt_kick_status, sched_clock()); */
						/* aee_kernel_warning_api(__FILE__, __LINE__,
						   DB_OPT_NE_JBT_TRACES|DB_OPT_DISPLAY_HANG_DUMP,
						   "\nCRDISPATCH_KEY:UI Hang(Powerkey)\n", */
						/* "Powerkey Monitor"); */
						/* msleep(30 * 1000); */
				} else {
					/* ShowStatus(); catch task kernel bt */
						/* LOGE("[WDK] Powerkey Tick fail,kick_status 0x%08x,RT[%lld]\n ", */
						/* wdt_kick_status, sched_clock()); */
						/* aee_kernel_exception_api(__FILE__, __LINE__,
						   DB_OPT_NE_JBT_TRACES|DB_OPT_DISPLAY_HANG_DUMP,
						   "\nCRDISPATCH_KEY:UI Hang(Powerkey)\n", */
						/* "Powerkey Monitor"); */
						/* msleep(30 * 1000); */
						/* ret = WDT_PWK_HANG_FORCE_HWT; trigger HWT */
				}
			}
		}
		if ((wdt_kick_status &
					(WDT_SETBY_Display |
					 WDT_SETBY_SF)) ==
				(WDT_SETBY_Display |
				 WDT_SETBY_SF)) {
			pwk_start_monitor =
				0;
			LOGE("[WDK] Powerkey Tick ok,kick_status 0x%08x,RT[%lld]\n ", wdt_kick_status, sched_clock());
		}
	}
	return ret;
}
EXPORT_SYMBOL(aee_kernel_wdt_kick_api);


module_init(monitor_hang_init);
module_exit(monitor_hang_exit);


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek AED Driver");
MODULE_AUTHOR("MediaTek Inc.");
