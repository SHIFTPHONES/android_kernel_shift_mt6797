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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/mm_types.h>
#include <linux/mm.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <asm/page.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <mach/irqs.h>
/* #include <mach/x_define_irq.h> */
#include <linux/wait.h>
#include <linux/proc_fs.h>
#include <linux/semaphore.h>
#include <mt-plat/dma.h>
#include <linux/delay.h>
#include "mt-plat/sync_write.h"

#ifndef CONFIG_MTK_CLKMGR
#include <linux/clk.h>
#else
#include "mach/mt_clkmgr.h"
#endif

#ifdef CONFIG_MTK_HIBERNATION
#include <mtk_hibernate_dpm.h>
#include <mach/diso.h>
#endif

#include "videocodec_kernel_driver.h"
#include "../videocodec_kernel.h"
#include <asm/cacheflush.h>
#include <asm/io.h>
#include <asm/sizes.h>
#include "val_types_private.h"
#include "val_api_private.h"
#include "val_log.h"
#include "drv_api.h"

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#if IS_ENABLED(CONFIG_COMPAT)
#include <linux/uaccess.h>
#include <linux/compat.h>
#endif

/* memory signature for memory protection */
#define MEM_SIGNATURE 0x56636F64

#define VDO_HW_WRITE(ptr, data)     mt_reg_sync_writel(data, ptr)
#define VDO_HW_READ(ptr)           (*((volatile unsigned int * const)(ptr)))

#define VCODEC_DEVNAME     "Vcodec"
#define VCODEC_DEV_MAJOR_NUMBER 160   /* 189 */
/* #define VENC_USE_L2C */

static dev_t vcodec_devno = MKDEV(VCODEC_DEV_MAJOR_NUMBER, 0);
static struct cdev *vcodec_cdev;
static struct class *vcodec_class;
static struct device *vcodec_device;

#ifndef CONFIG_MTK_CLKMGR
static struct clk *clk_MT_CG_DISP0_SMI_COMMON;  /* MM_DISP0_SMI_COMMON */
static struct clk *clk_MT_CG_VDEC0_VDEC;        /* VDEC0_VDEC */
static struct clk *clk_MT_CG_VDEC1_LARB;        /* VDEC1_LARB */

static struct clk *clk_MT_CG_VENC_VENC;         /* VENC_VENC */
static struct clk *clk_MT_CG_VENC_LARB;         /* VENC_LARB */

static struct clk *clk_MT_SCP_SYS_VDE;          /* SCP_SYS_VDE */
static struct clk *clk_MT_SCP_SYS_VEN;          /* SCP_SYS_VEN */
static struct clk *clk_MT_SCP_SYS_DIS;          /* SCP_SYS_DIS */
#endif

static DEFINE_MUTEX(IsOpenedLock);
static DEFINE_MUTEX(PWRLock);
static DEFINE_MUTEX(VdecHWLock);
static DEFINE_MUTEX(VencHWLock);
static DEFINE_MUTEX(EncEMILock);
static DEFINE_MUTEX(L2CLock);
static DEFINE_MUTEX(DecEMILock);
static DEFINE_MUTEX(DriverOpenCountLock);
static DEFINE_MUTEX(DecHWLockEventTimeoutLock);
static DEFINE_MUTEX(EncHWLockEventTimeoutLock);

static DEFINE_MUTEX(VdecPWRLock);
static DEFINE_MUTEX(VencPWRLock);
static DEFINE_MUTEX(LogCountLock);

static DEFINE_SPINLOCK(DecIsrLock);
static DEFINE_SPINLOCK(EncIsrLock);
static DEFINE_SPINLOCK(LockDecHWCountLock);
static DEFINE_SPINLOCK(LockEncHWCountLock);
static DEFINE_SPINLOCK(DecISRCountLock);
static DEFINE_SPINLOCK(EncISRCountLock);


static VAL_EVENT_T DecHWLockEvent;    /* mutex : HWLockEventTimeoutLock */
static VAL_EVENT_T EncHWLockEvent;    /* mutex : HWLockEventTimeoutLock */
static VAL_EVENT_T DecIsrEvent;    /* mutex : HWLockEventTimeoutLock */
static VAL_EVENT_T EncIsrEvent;    /* mutex : HWLockEventTimeoutLock */
static VAL_INT32_T Driver_Open_Count;         /* mutex : DriverOpenCountLock */
static VAL_UINT32_T gu4PWRCounter;      /* mutex : PWRLock */
static VAL_UINT32_T gu4EncEMICounter;   /* mutex : EncEMILock */
static VAL_UINT32_T gu4DecEMICounter;   /* mutex : DecEMILock */
static VAL_UINT32_T gu4L2CCounter;      /* mutex : L2CLock */
static VAL_BOOL_T bIsOpened = VAL_FALSE;    /* mutex : IsOpenedLock */
static VAL_UINT32_T gu4HwVencIrqStatus; /* hardware VENC IRQ status (VP8/H264) */

static VAL_UINT32_T gu4VdecPWRCounter;  /* mutex : VdecPWRLock */
static VAL_UINT32_T gu4VencPWRCounter;  /* mutex : VencPWRLock */

static VAL_UINT32_T gu4LogCountUser;  /* mutex : LogCountLock */
static VAL_UINT32_T gu4LogCount;

static VAL_UINT32_T gLockTimeOutCount;

static VAL_UINT32_T gu4VdecLockThreadId;

/* #define VCODEC_DEBUG */
#ifdef VCODEC_DEBUG
#undef VCODEC_DEBUG
#define VCODEC_DEBUG MODULE_MFV_LOGE
#undef MODULE_MFV_LOGD
#define MODULE_MFV_LOGD  MODULE_MFV_LOGE
#else
#define VCODEC_DEBUG(...)
#undef MODULE_MFV_LOGD
#define MODULE_MFV_LOGD(...)
#endif

/* VENC physical base address */
#undef VENC_BASE
#define VENC_BASE       0x17002000
#define VENC_REGION     0x1000

/* VDEC virtual base address */
#define VDEC_BASE_PHY   0x16000000
#define VDEC_REGION     0x29000

#define HW_BASE         0x7FFF000
#define HW_REGION       0x2000

#define INFO_BASE       0x10000000
#define INFO_REGION     0x1000

#if 0
#define VENC_IRQ_STATUS_addr        (VENC_BASE + 0x05C)
#define VENC_IRQ_ACK_addr           (VENC_BASE + 0x060)
#define VENC_MP4_IRQ_ACK_addr       (VENC_BASE + 0x678)
#define VENC_MP4_IRQ_STATUS_addr    (VENC_BASE + 0x67C)
#define VENC_ZERO_COEF_COUNT_addr   (VENC_BASE + 0x688)
#define VENC_BYTE_COUNT_addr        (VENC_BASE + 0x680)
#define VENC_MP4_IRQ_ENABLE_addr    (VENC_BASE + 0x668)

#define VENC_MP4_STATUS_addr        (VENC_BASE + 0x664)
#define VENC_MP4_MVQP_STATUS_addr   (VENC_BASE + 0x6E4)
#endif


#define VENC_IRQ_STATUS_SPS         0x1
#define VENC_IRQ_STATUS_PPS         0x2
#define VENC_IRQ_STATUS_FRM         0x4
#define VENC_IRQ_STATUS_DRAM        0x8
#define VENC_IRQ_STATUS_PAUSE       0x10
#define VENC_IRQ_STATUS_SWITCH      0x20

#if 0
/* VDEC virtual base address */
#define VDEC_MISC_BASE  (VDEC_BASE + 0x0000)
#define VDEC_VLD_BASE   (VDEC_BASE + 0x1000)
#endif


VAL_ULONG_T KVA_VENC_IRQ_ACK_ADDR, KVA_VENC_IRQ_STATUS_ADDR, KVA_VENC_BASE;
VAL_ULONG_T KVA_VDEC_MISC_BASE, KVA_VDEC_VLD_BASE, KVA_VDEC_BASE, KVA_VDEC_GCON_BASE;
VAL_UINT32_T VENC_IRQ_ID, VDEC_IRQ_ID;

/* #define KS_POWER_WORKAROUND */

/* extern unsigned long pmem_user_v2p_video(unsigned long va); */

#if defined(VENC_USE_L2C)
/* extern int config_L2(int option); */
#endif

void *mt_venc_base_get(void)
{
	return (void *)KVA_VENC_BASE;
}
EXPORT_SYMBOL(mt_venc_base_get);

void *mt_vdec_base_get(void)
{
	return (void *)KVA_VDEC_BASE;
}
EXPORT_SYMBOL(mt_vdec_base_get);

void vdec_power_on(void)
{
	int ret = 0;

	mutex_lock(&VdecPWRLock);
	gu4VdecPWRCounter++;
	mutex_unlock(&VdecPWRLock);
	ret = 0;

#ifdef CONFIG_MTK_CLKMGR
	/* Central power on */
	enable_clock(MT_CG_DISP0_SMI_COMMON, "VDEC");
	enable_clock(MT_CG_VDEC0_VDEC, "VDEC");
	enable_clock(MT_CG_VDEC1_LARB, "VDEC");
#ifdef VDEC_USE_L2C
	/* enable_clock(MT_CG_INFRA_L2C_SRAM, "VDEC"); */
#endif
#else
	ret = clk_prepare_enable(clk_MT_SCP_SYS_DIS);
	if (ret) {
		/* print error log & error handling */
		MODULE_MFV_LOGE("[VCODEC][ERROR][vdec_power_on] clk_MT_SCP_SYS_DIS is not enabled, ret = %d\n", ret);
	}

	ret = clk_prepare_enable(clk_MT_CG_DISP0_SMI_COMMON);
	if (ret) {
		/* print error log & error handling */
		MODULE_MFV_LOGE("[VCODEC][ERROR][vdec_power_on] clk_MT_CG_DISP0_SMI_COMMON is not enabled, ret = %d\n",
		    ret);
	}

	ret = clk_prepare_enable(clk_MT_SCP_SYS_VDE);
	if (ret) {
		/* print error log & error handling */
		MODULE_MFV_LOGE("[VCODEC][ERROR][vdec_power_on] clk_MT_SCP_SYS_VDE is not enabled, ret = %d\n", ret);
	}

	ret = clk_prepare_enable(clk_MT_CG_VDEC0_VDEC);
	if (ret) {
		/* print error log & error handling */
		MODULE_MFV_LOGE("[VCODEC][ERROR][vdec_power_on] clk_MT_CG_VDEC0_VDEC is not enabled, ret = %d\n", ret);
	}

	ret = clk_prepare_enable(clk_MT_CG_VDEC1_LARB);
	if (ret) {
		/* print error log & error handling */
		MODULE_MFV_LOGE("[VCODEC][ERROR][vdec_power_on] clk_MT_CG_VDEC1_LARB is not enabled, ret = %d\n", ret);
	}
#endif
}

void vdec_power_off(void)
{
	mutex_lock(&VdecPWRLock);
	if (gu4VdecPWRCounter == 0) {
		MODULE_MFV_LOGD("[VCODEC] gu4VdecPWRCounter = 0\n");
	} else {
		gu4VdecPWRCounter--;
#ifdef CONFIG_MTK_CLKMGR
		/* Central power off */
		disable_clock(MT_CG_VDEC0_VDEC, "VDEC");
		disable_clock(MT_CG_VDEC1_LARB, "VDEC");
		disable_clock(MT_CG_DISP0_SMI_COMMON, "VDEC");
#ifdef VDEC_USE_L2C
		/* disable_clock(MT_CG_INFRA_L2C_SRAM, "VDEC"); */
#endif
#else
		clk_disable_unprepare(clk_MT_CG_VDEC1_LARB);
		clk_disable_unprepare(clk_MT_CG_VDEC0_VDEC);
		clk_disable_unprepare(clk_MT_SCP_SYS_VDE);
		clk_disable_unprepare(clk_MT_CG_DISP0_SMI_COMMON);
		clk_disable_unprepare(clk_MT_SCP_SYS_DIS);
#endif
	}
	mutex_unlock(&VdecPWRLock);
}

void venc_power_on(void)
{
	int ret = 0;

	mutex_lock(&VencPWRLock);
	gu4VencPWRCounter++;
	mutex_unlock(&VencPWRLock);
	ret = 0;

	MODULE_MFV_LOGD("[VCODEC] venc_power_on +\n");
#ifdef CONFIG_MTK_CLKMGR
	enable_clock(MT_CG_DISP0_SMI_COMMON, "VENC");
	enable_clock(MT_CG_VENC_VENC, "VENC");
	enable_clock(MT_CG_VENC_LARB , "VENC");

#ifdef VENC_USE_L2C
	enable_clock(MT_CG_INFRA_L2C_SRAM, "VENC");
#endif
#else
	ret = clk_prepare_enable(clk_MT_SCP_SYS_DIS);
	if (ret) {
		/* print error log & error handling */
		MODULE_MFV_LOGE("[VCODEC][ERROR][venc_power_on] clk_MT_SCP_SYS_DIS is not enabled, ret = %d\n", ret);
	}

	ret = clk_prepare_enable(clk_MT_CG_DISP0_SMI_COMMON);
	if (ret) {
		/* print error log & error handling */
		MODULE_MFV_LOGE("[VCODEC][ERROR][venc_power_on] clk_MT_CG_DISP0_SMI_COMMON is not enabled, ret = %d\n",
		    ret);
	}

	ret = clk_prepare_enable(clk_MT_SCP_SYS_VEN);
	if (ret) {
		/* print error log & error handling */
		MODULE_MFV_LOGE("[VCODEC][ERROR][venc_power_on] clk_MT_SCP_SYS_VEN is not enabled, ret = %d\n", ret);
	}

	ret = clk_prepare_enable(clk_MT_CG_VENC_VENC);
	if (ret) {
		/* print error log & error handling */
		MODULE_MFV_LOGE("[VCODEC][ERROR][venc_power_on] clk_MT_CG_VENC_VENC is not enabled, ret = %d\n", ret);
	}

	ret = clk_prepare_enable(clk_MT_CG_VENC_LARB);
	if (ret) {
		/* print error log & error handling */
		MODULE_MFV_LOGE("[VCODEC][ERROR][venc_power_on] clk_MT_CG_VENC_LARB is not enabled, ret = %d\n", ret);
	}
#endif

	MODULE_MFV_LOGD("[VCODEC] venc_power_on -\n");
}

void venc_power_off(void)
{
	mutex_lock(&VencPWRLock);
	if (gu4VencPWRCounter == 0) {
		MODULE_MFV_LOGD("[VCODEC] gu4VencPWRCounter = 0\n");
	} else {
		gu4VencPWRCounter--;
		MODULE_MFV_LOGD("[VCODEC] venc_power_off +\n");
#ifdef CONFIG_MTK_CLKMGR
		disable_clock(MT_CG_VENC_VENC, "VENC");
		disable_clock(MT_CG_VENC_LARB, "VENC");
		disable_clock(MT_CG_DISP0_SMI_COMMON, "VENC");
#ifdef VENC_USE_L2C
		disable_clock(MT_CG_INFRA_L2C_SRAM, "VENC");
#endif
#else
		clk_disable_unprepare(clk_MT_CG_VENC_LARB);
		clk_disable_unprepare(clk_MT_CG_VENC_VENC);
		clk_disable_unprepare(clk_MT_SCP_SYS_VEN);
		clk_disable_unprepare(clk_MT_CG_DISP0_SMI_COMMON);
		clk_disable_unprepare(clk_MT_SCP_SYS_DIS);
#endif
		MODULE_MFV_LOGD("[VCODEC] venc_power_off -\n");
	}
	mutex_unlock(&VencPWRLock);
}

void dec_isr(void)
{
	VAL_RESULT_T    eValRet;
	VAL_ULONG_T     ulFlags, ulFlagsISR, ulFlagsLockHW;

	VAL_UINT32_T u4TempDecISRCount = 0;
	VAL_UINT32_T u4TempLockDecHWCount = 0;
	VAL_UINT32_T u4CgStatus = 0;
	VAL_UINT32_T u4DecDoneStatus = 0;

	u4CgStatus = VDO_HW_READ(KVA_VDEC_GCON_BASE);
	if ((u4CgStatus & 0x10) != 0) {
		MODULE_MFV_LOGE("[VCODEC][ERROR] DEC ISR, VDEC active is not 0x0 (0x%08x)", u4CgStatus);
		return;
	}

	u4DecDoneStatus = VDO_HW_READ(KVA_VDEC_BASE + 0xA4);
	if ((u4DecDoneStatus & (0x1 << 16)) != 0x10000) {
		MODULE_MFV_LOGE("[VCODEC][ERROR] DEC ISR, Decode done status is not 0x1 (0x%08x)", u4DecDoneStatus);
		return;
	}


	spin_lock_irqsave(&DecISRCountLock, ulFlagsISR);
	gu4DecISRCount++;
	u4TempDecISRCount = gu4DecISRCount;
	spin_unlock_irqrestore(&DecISRCountLock, ulFlagsISR);

	spin_lock_irqsave(&LockDecHWCountLock, ulFlagsLockHW);
	u4TempLockDecHWCount = gu4LockDecHWCount;
	spin_unlock_irqrestore(&LockDecHWCountLock, ulFlagsLockHW);

	if (u4TempDecISRCount != u4TempLockDecHWCount) {
		/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
		/* MODULE_MFV_LOGE("[INFO] Dec ISRCount: 0x%x, LockHWCount:0x%x\n",
		u4TempDecISRCount, u4TempLockDecHWCount); */
	}

	/* Clear interrupt */
	VDO_HW_WRITE(KVA_VDEC_MISC_BASE + 41 * 4, VDO_HW_READ(KVA_VDEC_MISC_BASE + 41 * 4) | 0x11);
	VDO_HW_WRITE(KVA_VDEC_MISC_BASE + 41 * 4, VDO_HW_READ(KVA_VDEC_MISC_BASE + 41 * 4) & ~0x10);


	spin_lock_irqsave(&DecIsrLock, ulFlags);
	eValRet = eVideoSetEvent(&DecIsrEvent, sizeof(VAL_EVENT_T));
	if (VAL_RESULT_NO_ERROR != eValRet) {
		/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
		MODULE_MFV_LOGE("[VCODEC][ERROR] ISR set DecIsrEvent error\n");
	}
	spin_unlock_irqrestore(&DecIsrLock, ulFlags);
}


void enc_isr(void)
{
	VAL_RESULT_T  eValRet;
	VAL_ULONG_T ulFlagsISR, ulFlagsLockHW;


	VAL_UINT32_T u4TempEncISRCount = 0;
	VAL_UINT32_T u4TempLockEncHWCount = 0;
	/* ---------------------- */

	spin_lock_irqsave(&EncISRCountLock, ulFlagsISR);
	gu4EncISRCount++;
	u4TempEncISRCount = gu4EncISRCount;
	spin_unlock_irqrestore(&EncISRCountLock, ulFlagsISR);

	spin_lock_irqsave(&LockEncHWCountLock, ulFlagsLockHW);
	u4TempLockEncHWCount = gu4LockEncHWCount;
	spin_unlock_irqrestore(&LockEncHWCountLock, ulFlagsLockHW);

	if (u4TempEncISRCount != u4TempLockEncHWCount) {
		/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
		/* MODULE_MFV_LOGE("[INFO] Enc ISRCount: 0x%x, LockHWCount:0x%x\n",
		u4TempEncISRCount, u4TempLockEncHWCount); */
	}

	if (grVcodecEncHWLock.pvHandle == 0) {
		MODULE_MFV_LOGE("[VCODEC][ERROR] NO one Lock Enc HW, please check!!\n");

		/* Clear all status */
		/* VDO_HW_WRITE(KVA_VENC_MP4_IRQ_ACK_ADDR, 1); */
		VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR, VENC_IRQ_STATUS_PAUSE);
		/* VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR, VENC_IRQ_STATUS_DRAM_VP8); */
		VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR, VENC_IRQ_STATUS_SWITCH);
		VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR, VENC_IRQ_STATUS_DRAM);
		VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR, VENC_IRQ_STATUS_SPS);
		VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR, VENC_IRQ_STATUS_PPS);
		VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR, VENC_IRQ_STATUS_FRM);
		return;
	}

	if (grVcodecEncHWLock.eDriverType == VAL_DRIVER_TYPE_H264_ENC) { /* hardwire */
		gu4HwVencIrqStatus = VDO_HW_READ(KVA_VENC_IRQ_STATUS_ADDR);
		if (gu4HwVencIrqStatus & VENC_IRQ_STATUS_PAUSE) {
			/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
			VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR, VENC_IRQ_STATUS_PAUSE);
		}
		if (gu4HwVencIrqStatus & VENC_IRQ_STATUS_SWITCH) {
			/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
			VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR, VENC_IRQ_STATUS_SWITCH);
		}
		if (gu4HwVencIrqStatus & VENC_IRQ_STATUS_DRAM) {
			/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
			VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR, VENC_IRQ_STATUS_DRAM);
		}
		if (gu4HwVencIrqStatus & VENC_IRQ_STATUS_SPS) {
			/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
			VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR, VENC_IRQ_STATUS_SPS);
		}
		if (gu4HwVencIrqStatus & VENC_IRQ_STATUS_PPS) {
			/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
			VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR, VENC_IRQ_STATUS_PPS);
		}
		if (gu4HwVencIrqStatus & VENC_IRQ_STATUS_FRM) {
			/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
			VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR, VENC_IRQ_STATUS_FRM);
		}
	} else if (grVcodecEncHWLock.eDriverType == VAL_DRIVER_TYPE_HEVC_ENC) { /* hardwire */
		MODULE_MFV_LOGE("[VCODEC][enc_isr] VAL_DRIVER_TYPE_HEVC_ENC!!\n");
	} else {
		MODULE_MFV_LOGE("[VCODEC][ERROR] Invalid lock holder driver type = %d\n",
			grVcodecEncHWLock.eDriverType);
	}

	eValRet = eVideoSetEvent(&EncIsrEvent, sizeof(VAL_EVENT_T));
	if (VAL_RESULT_NO_ERROR != eValRet) {
		/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
		MODULE_MFV_LOGE("[VCODEC][ERROR] ISR set EncIsrEvent error\n");
	}
}

static irqreturn_t video_intr_dlr(int irq, void *dev_id)
{
	dec_isr();
	return IRQ_HANDLED;
}

static irqreturn_t video_intr_dlr2(int irq, void *dev_id)
{
	enc_isr();
	return IRQ_HANDLED;
}

static long vcodec_alloc_non_cache_buffer(unsigned long arg)
{
	VAL_UINT8_T *user_data_addr;
	VAL_MEMORY_T rTempMem;
	VAL_LONG_T ret;

	MODULE_MFV_LOGE("VCODEC_ALLOC_NON_CACHE_BUFFER + tid = %d\n", current->pid);

	user_data_addr = (VAL_UINT8_T *)arg;
	ret = copy_from_user(&rTempMem, user_data_addr, sizeof(VAL_MEMORY_T));
	if (ret) {
		MODULE_MFV_LOGE("[ERROR] VCODEC_ALLOC_NON_CACHE_BUFFER, copy_from_user failed: %lu\n", ret);
		return -EFAULT;
	}

	rTempMem.u4MemSign = MEM_SIGNATURE;
	rTempMem.u4ReservedSize /*kernel va*/ =
		(VAL_ULONG_T)dma_alloc_coherent(0, rTempMem.u4MemSize, (dma_addr_t *)&rTempMem.pvMemPa, GFP_KERNEL);
	if ((0 == rTempMem.u4ReservedSize) || (0 == rTempMem.pvMemPa)) {
		MODULE_MFV_LOGE("[ERROR] dma_alloc_coherent fail in VCODEC_ALLOC_NON_CACHE_BUFFER\n");
		return -EFAULT;
	}

	MODULE_MFV_LOGD("[VCODEC] kernel va = 0x%lx, kernel pa = 0x%lx, memory size = %lu\n",
		 (VAL_ULONG_T)rTempMem.u4ReservedSize,
		 (VAL_ULONG_T)rTempMem.pvMemPa,
		 (VAL_ULONG_T)rTempMem.u4MemSize);

	/* mutex_lock(&NonCacheMemoryListLock); */
	/* Add_NonCacheMemoryList(rTempMem.u4ReservedSize, (VAL_UINT32_T)rTempMem.pvMemPa,
				    (VAL_UINT32_T)rTempMem.u4MemSize, 0, 0); */
	/* mutex_unlock(&NonCacheMemoryListLock); */

	ret = copy_to_user(user_data_addr, &rTempMem, sizeof(VAL_MEMORY_T));
	if (ret) {
		MODULE_MFV_LOGE("[ERROR] VCODEC_ALLOC_NON_CACHE_BUFFER, copy_to_user failed: %lu\n", ret);
		return -EFAULT;
	}

	MODULE_MFV_LOGE("VCODEC_ALLOC_NON_CACHE_BUFFER - tid = %d\n", current->pid);

	return 0;
}

static long vcodec_free_non_cache_buffer(unsigned long arg)
{
	VAL_UINT8_T *user_data_addr;
	VAL_MEMORY_T rTempMem;
	VAL_LONG_T ret;

	MODULE_MFV_LOGE("VCODEC_FREE_NON_CACHE_BUFFER + tid = %d\n", current->pid);

	user_data_addr = (VAL_UINT8_T *)arg;
	ret = copy_from_user(&rTempMem, user_data_addr, sizeof(VAL_MEMORY_T));
	if (ret) {
		MODULE_MFV_LOGE("[ERROR] VCODEC_FREE_NON_CACHE_BUFFER, copy_from_user failed: %lu\n", ret);
		return -EFAULT;
	}

	if (rTempMem.u4MemSign != MEM_SIGNATURE) {
		MODULE_MFV_LOGE("[ERROR] VCODEC_FREE_NON_CACHE_BUFFER, memory illegal: %d\n",
		rTempMem.u4MemSign);
		return -EFAULT;
	}
	if (rTempMem.u4MemSize == 0 || rTempMem.u4ReservedSize == 0) {
		MODULE_MFV_LOGE("[ERROR] VCODEC_FREE_NON_CACHE_BUFFER, memory size illegal\n");
		return -EFAULT;
	}

	dma_free_coherent(0, rTempMem.u4MemSize, (void *)rTempMem.u4ReservedSize,
							(dma_addr_t)(VAL_ULONG_T)rTempMem.pvMemPa);

	/* mutex_lock(&NonCacheMemoryListLock); */
	/* Free_NonCacheMemoryList(rTempMem.u4ReservedSize, (VAL_UINT32_T)rTempMem.pvMemPa); */
	/* mutex_unlock(&NonCacheMemoryListLock); */

	rTempMem.u4ReservedSize = 0;
	rTempMem.pvMemPa = NULL;

	ret = copy_to_user(user_data_addr, &rTempMem, sizeof(VAL_MEMORY_T));
	if (ret) {
		MODULE_MFV_LOGE("[ERROR] VCODEC_FREE_NON_CACHE_BUFFER, copy_to_user failed: %lu\n", ret);
		return -EFAULT;
	}

	MODULE_MFV_LOGE("VCODEC_FREE_NON_CACHE_BUFFER - tid = %d\n", current->pid);

	return 0;
}

static long vcodec_lockhw_dec_fail(VAL_HW_LOCK_T rHWLock, VAL_UINT32_T FirstUseDecHW)
{
	MODULE_MFV_LOGE("[ERROR] VCODEC_LOCKHW, DecHWLockEvent TimeOut, CurrentTID = %d\n", current->pid);
	if (FirstUseDecHW != 1) {
		mutex_lock(&VdecHWLock);
		if (grVcodecDecHWLock.pvHandle == 0) {
			/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
			MODULE_MFV_LOGE("[WARNING] VCODEC_LOCKHW, maybe mediaserver restart before, please check!!\n");
		} else {
			/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
			MODULE_MFV_LOGE("[WARNING] VCODEC_LOCKHW, someone use HW, and check timeout value!!\n");
		}
		mutex_unlock(&VdecHWLock);
	}

	return 0;
}

static long vcodec_lockhw_enc_fail(VAL_HW_LOCK_T rHWLock, VAL_UINT32_T FirstUseEncHW)
{
	MODULE_MFV_LOGE("[ERROR] VCODEC_LOCKHW EncHWLockEvent TimeOut, CurrentTID = %d\n", current->pid);

	if (FirstUseEncHW != 1) {
		mutex_lock(&VencHWLock);
		if (grVcodecEncHWLock.pvHandle == 0) {
			MODULE_MFV_LOGE("[WARNING] VCODEC_LOCKHW, maybe mediaserver restart before, please check!!\n");
		} else {
			MODULE_MFV_LOGE("[WARNING] VCODEC_LOCKHW, someone use HW, and check timeout value!! %d\n",
				 gLockTimeOutCount);
			++gLockTimeOutCount;
			if (gLockTimeOutCount > 30) {
				MODULE_MFV_LOGE("[ERROR] VCODEC_LOCKHW - ID %d fail\n", current->pid);
				MODULE_MFV_LOGE("someone locked HW time out more than 30 times 0x%lx,%lx,0x%lx,type:%d\n",
					 (VAL_ULONG_T)grVcodecEncHWLock.pvHandle,
					 pmem_user_v2p_video((VAL_ULONG_T)rHWLock.pvHandle),
					 (VAL_ULONG_T)rHWLock.pvHandle,
					 rHWLock.eDriverType);
				gLockTimeOutCount = 0;
				mutex_unlock(&VencHWLock);
				return -EFAULT;
			}

			if (rHWLock.u4TimeoutMs == 0) {
				MODULE_MFV_LOGE("[ERROR] VCODEC_LOCKHW - ID %d fail\n", current->pid);
				MODULE_MFV_LOGE("someone locked HW already 0x%lx,%lx,0x%lx,type:%d\n",
					 (VAL_ULONG_T)grVcodecEncHWLock.pvHandle,
					 pmem_user_v2p_video((VAL_ULONG_T)rHWLock.pvHandle),
					 (VAL_ULONG_T)rHWLock.pvHandle,
					 rHWLock.eDriverType);
				gLockTimeOutCount = 0;
				mutex_unlock(&VencHWLock);
				return -EFAULT;
			}
		}
		mutex_unlock(&VencHWLock);
	}

	return 0;
}

static long vcodec_lockhw(unsigned long arg)
{
	VAL_UINT8_T *user_data_addr;
	VAL_HW_LOCK_T rHWLock;
	VAL_RESULT_T eValRet;
	VAL_LONG_T ret;
	VAL_BOOL_T bLockedHW = VAL_FALSE;
	VAL_UINT32_T FirstUseDecHW = 0;
	VAL_UINT32_T FirstUseEncHW = 0;
	VAL_TIME_T rCurTime;
	VAL_UINT32_T u4TimeInterval;
	VAL_ULONG_T ulFlagsLockHW;

	MODULE_MFV_LOGD("VCODEC_LOCKHW + tid = %d\n", current->pid);

	user_data_addr = (VAL_UINT8_T *)arg;
	ret = copy_from_user(&rHWLock, user_data_addr, sizeof(VAL_HW_LOCK_T));
	if (ret) {
		MODULE_MFV_LOGE("[ERROR] VCODEC_LOCKHW, copy_from_user failed: %lu\n", ret);
		return -EFAULT;
	}

	MODULE_MFV_LOGD("[VCODEC] LOCKHW eDriverType = %d\n", rHWLock.eDriverType);
	eValRet = VAL_RESULT_INVALID_ISR;
	if (rHWLock.eDriverType == VAL_DRIVER_TYPE_MP4_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_HEVC_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_H264_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_MP1_MP2_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_VC1_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_VC1_ADV_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_VP8_DEC) {
		while (bLockedHW == VAL_FALSE) {
			mutex_lock(&DecHWLockEventTimeoutLock);
			if (DecHWLockEvent.u4TimeoutMs == 1) {
				MODULE_MFV_LOGE("VCODEC_LOCKHW, First Use Dec HW!!\n");
				FirstUseDecHW = 1;
			} else {
				FirstUseDecHW = 0;
			}
			mutex_unlock(&DecHWLockEventTimeoutLock);

			if (FirstUseDecHW == 1) {
				/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
				eValRet = eVideoWaitEvent(&DecHWLockEvent, sizeof(VAL_EVENT_T));
			}
			mutex_lock(&DecHWLockEventTimeoutLock);
			if (DecHWLockEvent.u4TimeoutMs != 1000) {
				DecHWLockEvent.u4TimeoutMs = 1000;
				FirstUseDecHW = 1;
			} else {
				FirstUseDecHW = 0;
			}
			mutex_unlock(&DecHWLockEventTimeoutLock);

			mutex_lock(&VdecHWLock);
			/* one process try to lock twice */
			if (grVcodecDecHWLock.pvHandle ==
			    (VAL_VOID_T *)pmem_user_v2p_video((VAL_ULONG_T)rHWLock.pvHandle)) {
				MODULE_MFV_LOGE("[WARNING] VCODEC_LOCKHW, one decoder instance try to lock twice\n");
				MODULE_MFV_LOGE("may cause lock HW timeout!! instance = 0x%lx, CurrentTID = %d\n",
					 (VAL_ULONG_T)grVcodecDecHWLock.pvHandle, current->pid);
			}
			mutex_unlock(&VdecHWLock);

			if (FirstUseDecHW == 0) {
				MODULE_MFV_LOGD("VCODEC_LOCKHW, Not first time use HW, timeout = %d\n",
					 DecHWLockEvent.u4TimeoutMs);
				eValRet = eVideoWaitEvent(&DecHWLockEvent, sizeof(VAL_EVENT_T));
			}

			if (VAL_RESULT_INVALID_ISR == eValRet) {
				ret = vcodec_lockhw_dec_fail(rHWLock, FirstUseDecHW);
				if (ret) {
					MODULE_MFV_LOGE("[ERROR] vcodec_lockhw_dec_fail failed: %lu\n", ret);
					return -EFAULT;
				}
			} else if (VAL_RESULT_RESTARTSYS == eValRet) {
				MODULE_MFV_LOGE("[WARNING] VCODEC_LOCKHW, VAL_RESULT_RESTARTSYS return when HWLock!!\n");
				return -ERESTARTSYS;
			}

			mutex_lock(&VdecHWLock);
			if (grVcodecDecHWLock.pvHandle == 0) { /* No one holds dec hw lock now */
				gu4VdecLockThreadId = current->pid;
				grVcodecDecHWLock.pvHandle =
					(VAL_VOID_T *)pmem_user_v2p_video((VAL_ULONG_T)rHWLock.pvHandle);
				grVcodecDecHWLock.eDriverType = rHWLock.eDriverType;
				eVideoGetTimeOfDay(&grVcodecDecHWLock.rLockedTime, sizeof(VAL_TIME_T));

				MODULE_MFV_LOGD("VCODEC_LOCKHW, No process use dec HW, so current process can use HW\n");
				MODULE_MFV_LOGD("LockInstance = 0x%lx CurrentTID = %d, rLockedTime(s, us) = %d, %d\n",
					 (VAL_ULONG_T)grVcodecDecHWLock.pvHandle,
					 current->pid,
					 grVcodecDecHWLock.rLockedTime.u4Sec, grVcodecDecHWLock.rLockedTime.u4uSec);

				bLockedHW = VAL_TRUE;
				if (VAL_RESULT_INVALID_ISR == eValRet && FirstUseDecHW != 1) {
					MODULE_MFV_LOGE("[WARNING] VCODEC_LOCKHW, reset power/irq when HWLock!!\n");
#ifndef KS_POWER_WORKAROUND
					vdec_power_off();
#endif
					disable_irq(VDEC_IRQ_ID);
				}
#ifndef KS_POWER_WORKAROUND
				vdec_power_on();
#endif
				if (rHWLock.bSecureInst == VAL_FALSE) {
					/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
					enable_irq(VDEC_IRQ_ID);
				}
			} else { /* Another one holding dec hw now */
				MODULE_MFV_LOGE("VCODEC_LOCKHW E\n");
				eVideoGetTimeOfDay(&rCurTime, sizeof(VAL_TIME_T));
				u4TimeInterval = (((((rCurTime.u4Sec - grVcodecDecHWLock.rLockedTime.u4Sec) * 1000000)
						    + rCurTime.u4uSec) - grVcodecDecHWLock.rLockedTime.u4uSec) / 1000);

				MODULE_MFV_LOGD("VCODEC_LOCKHW, someone use dec HW, and check timeout value\n");
				MODULE_MFV_LOGD("TimeInterval(ms) = %d, TimeOutValue(ms)) = %d\n",
					 u4TimeInterval, rHWLock.u4TimeoutMs);
				MODULE_MFV_LOGD("Lock Instance = 0x%lx, Lock TID = %d, CurrentTID = %d\n",
					 (VAL_ULONG_T)grVcodecDecHWLock.pvHandle,
					 gu4VdecLockThreadId,
					 current->pid);
				MODULE_MFV_LOGD("rLockedTime(%d s, %d us), rCurTime(%d s, %d us)\n",
					 grVcodecDecHWLock.rLockedTime.u4Sec, grVcodecDecHWLock.rLockedTime.u4uSec,
					 rCurTime.u4Sec, rCurTime.u4uSec);

				/* 2012/12/16. Cheng-Jung Never steal hardware lock */
			}
			mutex_unlock(&VdecHWLock);
			spin_lock_irqsave(&LockDecHWCountLock, ulFlagsLockHW);
			gu4LockDecHWCount++;
			spin_unlock_irqrestore(&LockDecHWCountLock, ulFlagsLockHW);
		}
	} else if (rHWLock.eDriverType == VAL_DRIVER_TYPE_H264_ENC ||
		   rHWLock.eDriverType == VAL_DRIVER_TYPE_HEVC_ENC ||
		   rHWLock.eDriverType == VAL_DRIVER_TYPE_JPEG_ENC) {
		while (bLockedHW == VAL_FALSE) {
			/* Early break for JPEG VENC */
			if (rHWLock.u4TimeoutMs == 0) {
				if (grVcodecEncHWLock.pvHandle != 0) {
					/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
					break;
				}
			}

			/* Wait to acquire Enc HW lock */
			mutex_lock(&EncHWLockEventTimeoutLock);
			if (EncHWLockEvent.u4TimeoutMs == 1) {
				MODULE_MFV_LOGE("VCODEC_LOCKHW, First Use Enc HW %d!!\n", rHWLock.eDriverType);
				FirstUseEncHW = 1;
			} else {
				FirstUseEncHW = 0;
			}
			mutex_unlock(&EncHWLockEventTimeoutLock);
			if (FirstUseEncHW == 1) {
				/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
				eValRet = eVideoWaitEvent(&EncHWLockEvent, sizeof(VAL_EVENT_T));
			}

			mutex_lock(&EncHWLockEventTimeoutLock);
			if (EncHWLockEvent.u4TimeoutMs == 1) {
				EncHWLockEvent.u4TimeoutMs = 1000;
				FirstUseEncHW = 1;
			} else {
				FirstUseEncHW = 0;
				if (rHWLock.u4TimeoutMs == 0) {
					/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
					EncHWLockEvent.u4TimeoutMs = 0; /* No wait */
				} else {
					EncHWLockEvent.u4TimeoutMs = 1000; /* Wait indefinitely */
				}
			}
			mutex_unlock(&EncHWLockEventTimeoutLock);

			mutex_lock(&VencHWLock);
			/* one process try to lock twice */
			if (grVcodecEncHWLock.pvHandle ==
			    (VAL_VOID_T *)pmem_user_v2p_video((VAL_ULONG_T)rHWLock.pvHandle)) {
				MODULE_MFV_LOGE("[WARNING] VCODEC_LOCKHW, one encoder instance try to lock twice\n");
				MODULE_MFV_LOGE("may cause lock HW timeout!! instance=0x%lx, CurrentTID=%d, type:%d\n",
					 (VAL_ULONG_T)grVcodecEncHWLock.pvHandle, current->pid, rHWLock.eDriverType);
			}
			mutex_unlock(&VencHWLock);

			if (FirstUseEncHW == 0) {
				/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
				eValRet = eVideoWaitEvent(&EncHWLockEvent, sizeof(VAL_EVENT_T));
			}

			if (VAL_RESULT_INVALID_ISR == eValRet) {
				ret = vcodec_lockhw_enc_fail(rHWLock, FirstUseEncHW);
				if (ret) {
					MODULE_MFV_LOGE("[ERROR] vcodec_lockhw_enc_fail failed: %lu\n", ret);
					return -EFAULT;
				}
			} else if (VAL_RESULT_RESTARTSYS == eValRet) {
				return -ERESTARTSYS;
			}

			mutex_lock(&VencHWLock);
			if (grVcodecEncHWLock.pvHandle == 0) { /* No process use HW, so current process can use HW */
				if (rHWLock.eDriverType == VAL_DRIVER_TYPE_H264_ENC ||
				    rHWLock.eDriverType == VAL_DRIVER_TYPE_HEVC_ENC ||
				    rHWLock.eDriverType == VAL_DRIVER_TYPE_JPEG_ENC) {
					grVcodecEncHWLock.pvHandle =
						(VAL_VOID_T *)pmem_user_v2p_video((VAL_ULONG_T)rHWLock.pvHandle);
					grVcodecEncHWLock.eDriverType = rHWLock.eDriverType;
					eVideoGetTimeOfDay(&grVcodecEncHWLock.rLockedTime, sizeof(VAL_TIME_T));

					MODULE_MFV_LOGD("VCODEC_LOCKHW, No process use HW, so current process can use HW\n");
					MODULE_MFV_LOGD("VCODEC_LOCKHW, handle = 0x%lx\n",
						 (VAL_ULONG_T)grVcodecEncHWLock.pvHandle);
					MODULE_MFV_LOGD("LockInstance = 0x%lx CurrentTID = %d, rLockedTime(s, us) = %d, %d\n",
						 (VAL_ULONG_T)grVcodecEncHWLock.pvHandle,
						 current->pid,
						 grVcodecEncHWLock.rLockedTime.u4Sec,
						 grVcodecEncHWLock.rLockedTime.u4uSec);

					bLockedHW = VAL_TRUE;
					if (rHWLock.eDriverType == VAL_DRIVER_TYPE_H264_ENC ||
					    rHWLock.eDriverType == VAL_DRIVER_TYPE_HEVC_ENC) {
#ifndef KS_POWER_WORKAROUND
						venc_power_on();
#endif
						enable_irq(VENC_IRQ_ID);
					}
				}
			} else { /* someone use HW, and check timeout value */
				if (rHWLock.u4TimeoutMs == 0) {
					bLockedHW = VAL_FALSE;
					mutex_unlock(&VencHWLock);
					break;
				}

				eVideoGetTimeOfDay(&rCurTime, sizeof(VAL_TIME_T));
				u4TimeInterval = (((((rCurTime.u4Sec - grVcodecEncHWLock.rLockedTime.u4Sec) * 1000000)
						    + rCurTime.u4uSec) - grVcodecEncHWLock.rLockedTime.u4uSec) / 1000);

				MODULE_MFV_LOGD("VCODEC_LOCKHW, someone use enc HW, and check timeout value\n");
				MODULE_MFV_LOGD("TimeInterval(ms) = %d, TimeOutValue(ms) = %d\n",
					 u4TimeInterval, rHWLock.u4TimeoutMs);
				MODULE_MFV_LOGD("rLockedTime(s, us) = %d, %d, rCurTime(s, us) = %d, %d\n",
					 grVcodecEncHWLock.rLockedTime.u4Sec, grVcodecEncHWLock.rLockedTime.u4uSec,
					 rCurTime.u4Sec, rCurTime.u4uSec);
				MODULE_MFV_LOGD("LockInstance = 0x%lx, CurrentInstance = 0x%lx, CurrentTID = %d\n",
					 (VAL_ULONG_T)grVcodecEncHWLock.pvHandle,
					 pmem_user_v2p_video((VAL_ULONG_T)rHWLock.pvHandle),
					 current->pid);

				++gLockTimeOutCount;
				if (gLockTimeOutCount > 30) {
					MODULE_MFV_LOGE("[ERROR] VCODEC_LOCKHW %d fail,someone locked HW over 30 times\n",
						 current->pid);
					MODULE_MFV_LOGE("without timeout 0x%lx,%lx,0x%lx,type:%d\n",
						 (VAL_ULONG_T)grVcodecEncHWLock.pvHandle,
						 pmem_user_v2p_video((VAL_ULONG_T)rHWLock.pvHandle),
						 (VAL_ULONG_T)rHWLock.pvHandle,
						 rHWLock.eDriverType);
					gLockTimeOutCount = 0;
					mutex_unlock(&VencHWLock);
					return -EFAULT;
				}

				/* 2013/04/10. Cheng-Jung Never steal hardware lock */
			}

			if (VAL_TRUE == bLockedHW) {
				MODULE_MFV_LOGD("VCODEC_LOCKHW, Lock ok grVcodecEncHWLock.pvHandle = 0x%lx, va:%lx, type:%d\n",
					 (VAL_ULONG_T)grVcodecEncHWLock.pvHandle,
					 (VAL_ULONG_T)rHWLock.pvHandle,
					 rHWLock.eDriverType);
				gLockTimeOutCount = 0;
			}
			mutex_unlock(&VencHWLock);
		}

		if (VAL_FALSE == bLockedHW) {
			MODULE_MFV_LOGE("[ERROR] VCODEC_LOCKHW %d fail,someone locked HW already,0x%lx,%lx,0x%lx,type:%d\n",
				 current->pid,
				 (VAL_ULONG_T)grVcodecEncHWLock.pvHandle,
				 pmem_user_v2p_video((VAL_ULONG_T)rHWLock.pvHandle),
				 (VAL_ULONG_T)rHWLock.pvHandle,
				 rHWLock.eDriverType);
			gLockTimeOutCount = 0;
			return -EFAULT;
		}

		spin_lock_irqsave(&LockEncHWCountLock, ulFlagsLockHW);
		gu4LockEncHWCount++;
		spin_unlock_irqrestore(&LockEncHWCountLock, ulFlagsLockHW);

		MODULE_MFV_LOGD("VCODEC_LOCKHW, get locked - ObjId =%d\n", current->pid);

		MODULE_MFV_LOGD("VCODEC_LOCKHWed - tid = %d\n", current->pid);
	} else {
		MODULE_MFV_LOGE("[WARNING] VCODEC_LOCKHW Unknown instance\n");
		return -EFAULT;
	}

	MODULE_MFV_LOGD("VCODEC_LOCKHW - tid = %d\n", current->pid);

	return 0;
}

static long vcodec_unlockhw(unsigned long arg)
{
	VAL_UINT8_T *user_data_addr;
	VAL_HW_LOCK_T rHWLock;
	VAL_RESULT_T eValRet;
	VAL_LONG_T ret;

	MODULE_MFV_LOGD("VCODEC_UNLOCKHW + tid = %d\n", current->pid);

	user_data_addr = (VAL_UINT8_T *)arg;
	ret = copy_from_user(&rHWLock, user_data_addr, sizeof(VAL_HW_LOCK_T));
	if (ret) {
		MODULE_MFV_LOGE("[ERROR] VCODEC_UNLOCKHW, copy_from_user failed: %lu\n", ret);
		return -EFAULT;
	}

	MODULE_MFV_LOGD("VCODEC_UNLOCKHW eDriverType = %d\n", rHWLock.eDriverType);
	eValRet = VAL_RESULT_INVALID_ISR;
	if (rHWLock.eDriverType == VAL_DRIVER_TYPE_MP4_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_HEVC_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_H264_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_MP1_MP2_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_VC1_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_VC1_ADV_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_VP8_DEC) {
		mutex_lock(&VdecHWLock);
		/* Current owner give up hw lock */
		if (grVcodecDecHWLock.pvHandle == (VAL_VOID_T *)pmem_user_v2p_video((VAL_ULONG_T)rHWLock.pvHandle)) {
			grVcodecDecHWLock.pvHandle = 0;
			grVcodecDecHWLock.eDriverType = VAL_DRIVER_TYPE_NONE;
			if (rHWLock.bSecureInst == VAL_FALSE) {
				/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
				disable_irq(VDEC_IRQ_ID);
			}
			/* TODO: check if turning power off is ok */
#ifndef KS_POWER_WORKAROUND
			vdec_power_off();
#endif
		} else { /* Not current owner */
			MODULE_MFV_LOGE("[ERROR] VCODEC_UNLOCKHW\n");
			MODULE_MFV_LOGE("Not owner trying to unlock dec hardware 0x%lx\n",
				 pmem_user_v2p_video((VAL_ULONG_T)rHWLock.pvHandle));
			mutex_unlock(&VdecHWLock);
			return -EFAULT;
		}
		mutex_unlock(&VdecHWLock);
		eValRet = eVideoSetEvent(&DecHWLockEvent, sizeof(VAL_EVENT_T));
	} else if (rHWLock.eDriverType == VAL_DRIVER_TYPE_H264_ENC ||
		   rHWLock.eDriverType == VAL_DRIVER_TYPE_HEVC_ENC ||
		   rHWLock.eDriverType == VAL_DRIVER_TYPE_JPEG_ENC) {
		mutex_lock(&VencHWLock);
		/* Current owner give up hw lock */
		if (grVcodecEncHWLock.pvHandle == (VAL_VOID_T *)pmem_user_v2p_video((VAL_ULONG_T)rHWLock.pvHandle)) {
			grVcodecEncHWLock.pvHandle = 0;
			grVcodecEncHWLock.eDriverType = VAL_DRIVER_TYPE_NONE;
			if (rHWLock.eDriverType == VAL_DRIVER_TYPE_H264_ENC ||
			    rHWLock.eDriverType == VAL_DRIVER_TYPE_HEVC_ENC) {
				disable_irq(VENC_IRQ_ID);
				/* turn venc power off */
#ifndef KS_POWER_WORKAROUND
				venc_power_off();
#endif
			}
		} else { /* Not current owner */
			/* [TODO] error handling */
			MODULE_MFV_LOGE("[ERROR] VCODEC_UNLOCKHW\n");
			MODULE_MFV_LOGE("Not owner trying to unlock enc hardware 0x%lx, pa:%lx, va:%lx type:%d\n",
				 (VAL_ULONG_T)grVcodecEncHWLock.pvHandle,
				 pmem_user_v2p_video((VAL_ULONG_T)rHWLock.pvHandle),
				 (VAL_ULONG_T)rHWLock.pvHandle,
				 rHWLock.eDriverType);
			mutex_unlock(&VencHWLock);
			return -EFAULT;
		}
		mutex_unlock(&VencHWLock);
		eValRet = eVideoSetEvent(&EncHWLockEvent, sizeof(VAL_EVENT_T));
	} else {
		MODULE_MFV_LOGE("[WARNING] VCODEC_UNLOCKHW Unknown instance\n");
		return -EFAULT;
	}

	MODULE_MFV_LOGD("VCODEC_UNLOCKHW - tid = %d\n", current->pid);

	return 0;
}

static long vcodec_waitisr(unsigned long arg)
{
	VAL_UINT8_T *user_data_addr;
	VAL_ISR_T val_isr;
	VAL_BOOL_T bLockedHW = VAL_FALSE;
	VAL_ULONG_T ulFlags;
	VAL_LONG_T ret;
	VAL_RESULT_T eValRet;

	MODULE_MFV_LOGD("VCODEC_WAITISR + tid = %d\n", current->pid);

	user_data_addr = (VAL_UINT8_T *)arg;
	ret = copy_from_user(&val_isr, user_data_addr, sizeof(VAL_ISR_T));
	if (ret) {
		MODULE_MFV_LOGE("[ERROR] VCODEC_WAITISR, copy_from_user failed: %lu\n", ret);
		return -EFAULT;
	}

	if (val_isr.eDriverType == VAL_DRIVER_TYPE_MP4_DEC ||
	    val_isr.eDriverType == VAL_DRIVER_TYPE_HEVC_DEC ||
	    val_isr.eDriverType == VAL_DRIVER_TYPE_H264_DEC ||
	    val_isr.eDriverType == VAL_DRIVER_TYPE_MP1_MP2_DEC ||
	    val_isr.eDriverType == VAL_DRIVER_TYPE_VC1_DEC ||
	    val_isr.eDriverType == VAL_DRIVER_TYPE_VC1_ADV_DEC ||
	    val_isr.eDriverType == VAL_DRIVER_TYPE_VP8_DEC) {
		mutex_lock(&VdecHWLock);
		if (grVcodecDecHWLock.pvHandle == (VAL_VOID_T *)pmem_user_v2p_video((VAL_ULONG_T)val_isr.pvHandle)) {
			/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
			bLockedHW = VAL_TRUE;
		} else {
		}
		mutex_unlock(&VdecHWLock);

		if (bLockedHW == VAL_FALSE) {
			MODULE_MFV_LOGE("[ERROR] VCODEC_WAITISR, DO NOT have HWLock, so return fail\n");
			return -EFAULT;
		}

		spin_lock_irqsave(&DecIsrLock, ulFlags);
		DecIsrEvent.u4TimeoutMs = val_isr.u4TimeoutMs;
		spin_unlock_irqrestore(&DecIsrLock, ulFlags);

		eValRet = eVideoWaitEvent(&DecIsrEvent, sizeof(VAL_EVENT_T));
		if (VAL_RESULT_INVALID_ISR == eValRet) {
			return -2;
		} else if (VAL_RESULT_RESTARTSYS == eValRet) {
			MODULE_MFV_LOGE("[WARNING] VCODEC_WAITISR, VAL_RESULT_RESTARTSYS return when WAITISR!!\n");
			return -ERESTARTSYS;
		}
	} else if (val_isr.eDriverType == VAL_DRIVER_TYPE_H264_ENC ||
		   val_isr.eDriverType == VAL_DRIVER_TYPE_HEVC_ENC) {
		mutex_lock(&VencHWLock);
		if (grVcodecEncHWLock.pvHandle == (VAL_VOID_T *)pmem_user_v2p_video((VAL_ULONG_T)val_isr.pvHandle)) {
			/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
			bLockedHW = VAL_TRUE;
		} else {
		}
		mutex_unlock(&VencHWLock);

		if (bLockedHW == VAL_FALSE) {
			MODULE_MFV_LOGE("[ERROR] VCODEC_WAITISR, DO NOT have enc HWLock, so return fail pa:%lx, va:%lx\n",
				 pmem_user_v2p_video((VAL_ULONG_T)val_isr.pvHandle),
				 (VAL_ULONG_T)val_isr.pvHandle);
			return -EFAULT;
		}

		spin_lock_irqsave(&EncIsrLock, ulFlags);
		EncIsrEvent.u4TimeoutMs = val_isr.u4TimeoutMs;
		spin_unlock_irqrestore(&EncIsrLock, ulFlags);

		eValRet = eVideoWaitEvent(&EncIsrEvent, sizeof(VAL_EVENT_T));
		if (VAL_RESULT_INVALID_ISR == eValRet) {
			return -2;
		} else if (VAL_RESULT_RESTARTSYS == eValRet) {
			MODULE_MFV_LOGE("[WARNING] VCODEC_WAITISR, VAL_RESULT_RESTARTSYS return when WAITISR!!\n");
			return -ERESTARTSYS;
		}

		if (val_isr.u4IrqStatusNum > 0) {
			val_isr.u4IrqStatus[0] = gu4HwVencIrqStatus;
			ret = copy_to_user(user_data_addr, &val_isr, sizeof(VAL_ISR_T));
			if (ret) {
				MODULE_MFV_LOGE("[ERROR] VCODEC_WAITISR, copy_to_user failed: %lu\n", ret);
				return -EFAULT;
			}
		}
	} else {
		MODULE_MFV_LOGE("[WARNING] VCODEC_WAITISR Unknown instance\n");
		return -EFAULT;
	}

	MODULE_MFV_LOGD("VCODEC_WAITISR - tid = %d\n", current->pid);

	return 0;
}

static long vcodec_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	VAL_LONG_T ret;
	VAL_UINT8_T *user_data_addr;
	VAL_VCODEC_CORE_LOADING_T rTempCoreLoading;
	VAL_VCODEC_CPU_OPP_LIMIT_T rCpuOppLimit;
	VAL_INT32_T temp_nr_cpu_ids;
	VAL_POWER_T rPowerParam;
	VAL_BOOL_T rIncLogCount;

#if 0
	VCODEC_DRV_CMD_QUEUE_T rDrvCmdQueue;
	P_VCODEC_DRV_CMD_T cmd_queue = VAL_NULL;
	VAL_UINT32_T u4Size, uValue, nCount;
#endif

	switch (cmd) {
	case VCODEC_SET_THREAD_ID:
	{
		/* MODULE_MFV_LOGE("VCODEC_SET_THREAD_ID [EMPTY] + tid = %d\n", current->pid); */
		/* MODULE_MFV_LOGE("VCODEC_SET_THREAD_ID [EMPTY] - tid = %d\n", current->pid); */
	}
	break;

	case VCODEC_ALLOC_NON_CACHE_BUFFER:
	{
		ret = vcodec_alloc_non_cache_buffer(arg);
		if (ret) {
			MODULE_MFV_LOGE("[ERROR] VCODEC_ALLOC_NON_CACHE_BUFFER failed! %lu\n", ret);
			return ret;
		}
	}
	break;

	case VCODEC_FREE_NON_CACHE_BUFFER:
	{
		ret = vcodec_free_non_cache_buffer(arg);
		if (ret) {
			MODULE_MFV_LOGE("[ERROR] VCODEC_FREE_NON_CACHE_BUFFER failed! %lu\n", ret);
			return ret;
		}
	}
	break;

	case VCODEC_INC_DEC_EMI_USER:
	{
		MODULE_MFV_LOGD("VCODEC_INC_DEC_EMI_USER + tid = %d\n", current->pid);

		mutex_lock(&DecEMILock);
		gu4DecEMICounter++;
		MODULE_MFV_LOGD("[VCODEC] DEC_EMI_USER = %d\n", gu4DecEMICounter);
		user_data_addr = (VAL_UINT8_T *)arg;
		ret = copy_to_user(user_data_addr, &gu4DecEMICounter, sizeof(VAL_UINT32_T));
		if (ret) {
			MODULE_MFV_LOGE("[ERROR] VCODEC_INC_DEC_EMI_USER, copy_to_user failed: %lu\n", ret);
			mutex_unlock(&DecEMILock);
			return -EFAULT;
		}
		mutex_unlock(&DecEMILock);

		MODULE_MFV_LOGD("VCODEC_INC_DEC_EMI_USER - tid = %d\n", current->pid);
	}
	break;

	case VCODEC_DEC_DEC_EMI_USER:
	{
		MODULE_MFV_LOGD("VCODEC_DEC_DEC_EMI_USER + tid = %d\n", current->pid);

		mutex_lock(&DecEMILock);
		gu4DecEMICounter--;
		MODULE_MFV_LOGD("[VCODEC] DEC_EMI_USER = %d\n", gu4DecEMICounter);
		user_data_addr = (VAL_UINT8_T *)arg;
		ret = copy_to_user(user_data_addr, &gu4DecEMICounter, sizeof(VAL_UINT32_T));
		if (ret) {
			MODULE_MFV_LOGE("[ERROR] VCODEC_DEC_DEC_EMI_USER, copy_to_user failed: %lu\n", ret);
			mutex_unlock(&DecEMILock);
			return -EFAULT;
		}
		mutex_unlock(&DecEMILock);

		MODULE_MFV_LOGD("VCODEC_DEC_DEC_EMI_USER - tid = %d\n", current->pid);
	}
	break;

	case VCODEC_INC_ENC_EMI_USER:
	{
		MODULE_MFV_LOGD("VCODEC_INC_ENC_EMI_USER + tid = %d\n", current->pid);

		mutex_lock(&EncEMILock);
		gu4EncEMICounter++;
		MODULE_MFV_LOGE("[VCODEC] ENC_EMI_USER = %d\n", gu4EncEMICounter);
		user_data_addr = (VAL_UINT8_T *)arg;
		ret = copy_to_user(user_data_addr, &gu4EncEMICounter, sizeof(VAL_UINT32_T));
		if (ret) {
			MODULE_MFV_LOGE("[ERROR] VCODEC_INC_ENC_EMI_USER, copy_to_user failed: %lu\n", ret);
			mutex_unlock(&EncEMILock);
			return -EFAULT;
		}
		mutex_unlock(&EncEMILock);

		MODULE_MFV_LOGD("VCODEC_INC_ENC_EMI_USER - tid = %d\n", current->pid);
	}
	break;

	case VCODEC_DEC_ENC_EMI_USER:
	{
		MODULE_MFV_LOGD("VCODEC_DEC_ENC_EMI_USER + tid = %d\n", current->pid);

		mutex_lock(&EncEMILock);
		gu4EncEMICounter--;
		MODULE_MFV_LOGE("[VCODEC] ENC_EMI_USER = %d\n", gu4EncEMICounter);
		user_data_addr = (VAL_UINT8_T *)arg;
		ret = copy_to_user(user_data_addr, &gu4EncEMICounter, sizeof(VAL_UINT32_T));
		if (ret) {
			MODULE_MFV_LOGE("[ERROR] VCODEC_DEC_ENC_EMI_USER, copy_to_user failed: %lu\n", ret);
			mutex_unlock(&EncEMILock);
			return -EFAULT;
		}
		mutex_unlock(&EncEMILock);

		MODULE_MFV_LOGD("VCODEC_DEC_ENC_EMI_USER - tid = %d\n", current->pid);
	}
	break;

	case VCODEC_LOCKHW:
	{
		ret = vcodec_lockhw(arg);
		if (ret) {
			MODULE_MFV_LOGE("[ERROR] VCODEC_LOCKHW failed! %lu\n", ret);
			return ret;
		}
	}
	break;

	case VCODEC_UNLOCKHW:
	{
		ret = vcodec_unlockhw(arg);
		if (ret) {
			MODULE_MFV_LOGE("[ERROR] VCODEC_UNLOCKHW failed! %lu\n", ret);
			return ret;
		}
	}
	break;

	case VCODEC_INC_PWR_USER:
	{
		MODULE_MFV_LOGD("VCODEC_INC_PWR_USER + tid = %d\n", current->pid);
		user_data_addr = (VAL_UINT8_T *)arg;
		ret = copy_from_user(&rPowerParam, user_data_addr, sizeof(VAL_POWER_T));
		if (ret) {
			MODULE_MFV_LOGE("[ERROR] VCODEC_INC_PWR_USER, copy_from_user failed: %lu\n", ret);
			return -EFAULT;
		}
		MODULE_MFV_LOGD("[VCODEC] INC_PWR_USER eDriverType = %d\n", rPowerParam.eDriverType);
		mutex_lock(&L2CLock);

#ifdef VENC_USE_L2C
		if (rPowerParam.eDriverType == VAL_DRIVER_TYPE_H264_ENC) {
			gu4L2CCounter++;
			MODULE_MFV_LOGD("[VCODEC] INC_PWR_USER L2C counter = %d\n", gu4L2CCounter);

			if (1 == gu4L2CCounter) {
				if (config_L2(0)) {
					MODULE_MFV_LOGE("[VCODEC][ERROR] Switch L2C size to 512K failed\n");
					mutex_unlock(&L2CLock);
					return -EFAULT;
				}
				MODULE_MFV_LOGE("[VCODEC] Switch L2C size to 512K successful\n");

			}
		}
#endif
		mutex_unlock(&L2CLock);
		MODULE_MFV_LOGD("VCODEC_INC_PWR_USER - tid = %d\n", current->pid);
	}
	break;

	case VCODEC_DEC_PWR_USER:
	{
		MODULE_MFV_LOGD("VCODEC_DEC_PWR_USER + tid = %d\n", current->pid);
		user_data_addr = (VAL_UINT8_T *)arg;
		ret = copy_from_user(&rPowerParam, user_data_addr, sizeof(VAL_POWER_T));
		if (ret) {
			MODULE_MFV_LOGE("[ERROR] VCODEC_DEC_PWR_USER, copy_from_user failed: %lu\n", ret);
			return -EFAULT;
		}
		MODULE_MFV_LOGD("[VCODEC] DEC_PWR_USER eDriverType = %d\n", rPowerParam.eDriverType);

		mutex_lock(&L2CLock);

#ifdef VENC_USE_L2C
		if (rPowerParam.eDriverType == VAL_DRIVER_TYPE_H264_ENC) {
			gu4L2CCounter--;
			MODULE_MFV_LOGD("[VCODEC] DEC_PWR_USER L2C counter  = %d\n", gu4L2CCounter);

			if (0 == gu4L2CCounter) {
				if (config_L2(1)) {
					MODULE_MFV_LOGE("[VCODEC][ERROR] Switch L2C size to 0K failed\n");
					mutex_unlock(&L2CLock);
					return -EFAULT;
				}
				MODULE_MFV_LOGE("[VCODEC] Switch L2C size to 0K successful\n");
			}
		}
#endif
		mutex_unlock(&L2CLock);
		MODULE_MFV_LOGD("VCODEC_DEC_PWR_USER - tid = %d\n", current->pid);
	}
	break;

	case VCODEC_WAITISR:
	{
		ret = vcodec_waitisr(arg);
		if (ret) {
			MODULE_MFV_LOGE("[ERROR] VCODEC_WAITISR failed! %lu\n", ret);
			return ret;
		}
	}
	break;

	case VCODEC_INITHWLOCK:
	{
		MODULE_MFV_LOGE("VCODEC_INITHWLOCK [EMPTY] + - tid = %d\n", current->pid);
		MODULE_MFV_LOGE("VCODEC_INITHWLOCK [EMPTY] - - tid = %d\n", current->pid);
	}
	break;

	case VCODEC_DEINITHWLOCK:
	{
		MODULE_MFV_LOGE("VCODEC_DEINITHWLOCK [EMPTY] + - tid = %d\n", current->pid);
		MODULE_MFV_LOGE("VCODEC_DEINITHWLOCK [EMPTY] - - tid = %d\n", current->pid);
	}
	break;

#if 0
	case VCODEC_GET_CPU_LOADING_INFO:
	{
		VAL_UINT8_T *user_data_addr;
		VAL_VCODEC_CPU_LOADING_INFO_T _temp;

		MODULE_MFV_LOGD("VCODEC_GET_CPU_LOADING_INFO +\n");
		user_data_addr = (VAL_UINT8_T *)arg;
		/* TODO: */
#if 0 /* Morris Yang 20120112 mark temporarily */
		_temp._cpu_idle_time = mt_get_cpu_idle(0);
		_temp._thread_cpu_time = mt_get_thread_cputime(0);
		spin_lock_irqsave(&OalHWContextLock, ulFlags);
		_temp._inst_count = getCurInstanceCount();
		spin_unlock_irqrestore(&OalHWContextLock, ulFlags);
		_temp._sched_clock = mt_sched_clock();
#endif
		ret = copy_to_user(user_data_addr, &_temp, sizeof(VAL_VCODEC_CPU_LOADING_INFO_T));
		if (ret) {
			MODULE_MFV_LOGE("[ERROR] VCODEC_GET_CPU_LOADING_INFO, copy_to_user failed: %lu\n", ret);
			return -EFAULT;
		}

		MODULE_MFV_LOGD("VCODEC_GET_CPU_LOADING_INFO -\n");
	}
	break;
#endif

	case VCODEC_GET_CORE_LOADING:
	{
		MODULE_MFV_LOGD("VCODEC_GET_CORE_LOADING + - tid = %d\n", current->pid);

		user_data_addr = (VAL_UINT8_T *)arg;
		ret = copy_from_user(&rTempCoreLoading, user_data_addr, sizeof(VAL_VCODEC_CORE_LOADING_T));
		if (ret) {
			MODULE_MFV_LOGE("[ERROR] VCODEC_GET_CORE_LOADING, copy_from_user failed: %lu\n", ret);
			return -EFAULT;
		}
		if (rTempCoreLoading.CPUid < 0) {
			MODULE_MFV_LOGE("[ERROR] rTempCoreLoading.CPUid < 0\n");
			return -EFAULT;
		}
		if (rTempCoreLoading.CPUid > num_possible_cpus()) {
			MODULE_MFV_LOGE("[ERROR] rTempCoreLoading.CPUid(%d) > num_possible_cpus(%d)\n",
			rTempCoreLoading.CPUid, num_possible_cpus());
			return -EFAULT;
		}
		rTempCoreLoading.Loading = get_cpu_load(rTempCoreLoading.CPUid);
		ret = copy_to_user(user_data_addr, &rTempCoreLoading, sizeof(VAL_VCODEC_CORE_LOADING_T));
		if (ret) {
			MODULE_MFV_LOGE("[ERROR] VCODEC_GET_CORE_LOADING, copy_to_user failed: %lu\n", ret);
			return -EFAULT;
		}
		MODULE_MFV_LOGD("VCODEC_GET_CORE_LOADING - - tid = %d\n", current->pid);
	}
	break;

	case VCODEC_GET_CORE_NUMBER:
	{
		MODULE_MFV_LOGD("VCODEC_GET_CORE_NUMBER + - tid = %d\n", current->pid);

		user_data_addr = (VAL_UINT8_T *)arg;
		temp_nr_cpu_ids = nr_cpu_ids;
		ret = copy_to_user(user_data_addr, &temp_nr_cpu_ids, sizeof(int));
		if (ret) {
			MODULE_MFV_LOGE("[ERROR] VCODEC_GET_CORE_NUMBER, copy_to_user failed: %lu\n", ret);
			return -EFAULT;
		}
		MODULE_MFV_LOGD("VCODEC_GET_CORE_NUMBER - - tid = %d\n", current->pid);
	}
	break;

	case VCODEC_SET_CPU_OPP_LIMIT:
	{
		MODULE_MFV_LOGE("VCODEC_SET_CPU_OPP_LIMIT [EMPTY] + - tid = %d\n", current->pid);
		user_data_addr = (VAL_UINT8_T *)arg;
		ret = copy_from_user(&rCpuOppLimit, user_data_addr, sizeof(VAL_VCODEC_CPU_OPP_LIMIT_T));
		if (ret) {
			MODULE_MFV_LOGE("[ERROR] VCODEC_SET_CPU_OPP_LIMIT, copy_from_user failed: %lu\n", ret);
			return -EFAULT;
		}
		MODULE_MFV_LOGE("+VCODEC_SET_CPU_OPP_LIMIT (%d, %d, %d), tid = %d\n",
			 rCpuOppLimit.limited_freq, rCpuOppLimit.limited_cpu, rCpuOppLimit.enable, current->pid);
		/* TODO: Check if cpu_opp_limit is available */
		/*
		ret = cpu_opp_limit(EVENT_VIDEO, rCpuOppLimit.limited_freq,
		rCpuOppLimit.limited_cpu, rCpuOppLimit.enable); // 0: PASS, other: FAIL
		if (ret) {
			MODULE_MFV_LOGE("[VCODEC][ERROR] cpu_opp_limit failed: %lu\n", ret);
			return -EFAULT;
		}
		*/
		MODULE_MFV_LOGE("-VCODEC_SET_CPU_OPP_LIMIT tid = %d, ret = %lu\n", current->pid, ret);
		MODULE_MFV_LOGE("VCODEC_SET_CPU_OPP_LIMIT [EMPTY] - - tid = %d\n", current->pid);
	}
	break;

	case VCODEC_MB:
	{
		mb();
	}
	break;

	case VCODEC_SET_LOG_COUNT:
	{
		MODULE_MFV_LOGD("VCODEC_SET_LOG_COUNT + tid = %d\n", current->pid);

		mutex_lock(&LogCountLock);
		user_data_addr = (VAL_UINT8_T *)arg;
		ret = copy_from_user(&rIncLogCount, user_data_addr, sizeof(VAL_BOOL_T));
		if (ret) {
			MODULE_MFV_LOGE("[ERROR] VCODEC_SET_LOG_COUNT, copy_from_user failed: %lu\n", ret);
			mutex_unlock(&LogCountLock);
			return -EFAULT;
		}

		if (rIncLogCount == VAL_TRUE) {
			if (gu4LogCountUser == 0) {
				gu4LogCount = get_detect_count();
				set_detect_count(gu4LogCount + 100);
			}
			gu4LogCountUser++;
		} else {
			gu4LogCountUser--;
			if (gu4LogCountUser == 0) {
				set_detect_count(gu4LogCount);
				gu4LogCount = 0;
			}
		}
		mutex_unlock(&LogCountLock);

		MODULE_MFV_LOGD("VCODEC_SET_LOG_COUNT - tid = %d\n", current->pid);
	}
	break;

	default:
	{
		MODULE_MFV_LOGE("========[ERROR] vcodec_ioctl default case======== %u\n", cmd);
	}
	break;

	}
	return 0xFF;
}

#if IS_ENABLED(CONFIG_COMPAT)

typedef enum {
	VAL_HW_LOCK_TYPE = 0,
	VAL_POWER_TYPE,
	VAL_ISR_TYPE,
	VAL_MEMORY_TYPE
} STRUCT_TYPE;

typedef enum {
	COPY_FROM_USER = 0,
	COPY_TO_USER,
} COPY_DIRECTION;

typedef struct COMPAT_VAL_HW_LOCK {
	/* [IN]     The video codec driver handle */
	compat_uptr_t       pvHandle;
	/* [IN]     The size of video codec driver handle */
	compat_uint_t       u4HandleSize;
	/* [IN/OUT] The Lock discriptor */
	compat_uptr_t       pvLock;
	/* [IN]     The timeout ms */
	compat_uint_t       u4TimeoutMs;
	/* [IN/OUT] The reserved parameter */
	compat_uptr_t       pvReserved;
	/* [IN]     The size of reserved parameter structure */
	compat_uint_t       u4ReservedSize;
	/* [IN]     The driver type */
	compat_uint_t       eDriverType;
	/* [IN]     True if this is a secure instance // MTK_SEC_VIDEO_PATH_SUPPORT */
	char                bSecureInst;
} COMPAT_VAL_HW_LOCK_T;

typedef struct COMPAT_VAL_POWER {
	/* [IN]     The video codec driver handle */
	compat_uptr_t       pvHandle;
	/* [IN]     The size of video codec driver handle */
	compat_uint_t       u4HandleSize;
	/* [IN]     The driver type */
	compat_uint_t       eDriverType;
	/* [IN]     Enable or not. */
	char                fgEnable;
	/* [IN/OUT] The reserved parameter */
	compat_uptr_t       pvReserved;
	/* [IN]     The size of reserved parameter structure */
	compat_uint_t       u4ReservedSize;
	/* [OUT]    The number of power user right now */
	/* VAL_UINT32_T        u4L2CUser; */
} COMPAT_VAL_POWER_T;

typedef struct COMPAT_VAL_ISR {
	/* [IN]     The video codec driver handle */
	compat_uptr_t       pvHandle;
	/* [IN]     The size of video codec driver handle */
	compat_uint_t       u4HandleSize;
	/* [IN]     The driver type */
	compat_uint_t       eDriverType;
	/* [IN]     The isr function */
	compat_uptr_t       pvIsrFunction;
	/* [IN/OUT] The reserved parameter */
	compat_uptr_t       pvReserved;
	/* [IN]     The size of reserved parameter structure */
	compat_uint_t       u4ReservedSize;
	/* [IN]     The timeout in ms */
	compat_uint_t       u4TimeoutMs;
	/* [IN]     The num of return registers when HW done */
	compat_uint_t       u4IrqStatusNum;
	/* [IN/OUT] The value of return registers when HW done */
	compat_uint_t       u4IrqStatus[IRQ_STATUS_MAX_NUM];
} COMPAT_VAL_ISR_T;

typedef struct COMPAT_VAL_MEMORY {
	/* [IN]     The allocation memory type */
	compat_uint_t       eMemType;
	/* [IN]     The size of memory allocation */
	compat_ulong_t      u4MemSize;
	/* [IN/OUT] The memory virtual address */
	compat_uptr_t       pvMemVa;
	/* [IN/OUT] The memory physical address */
	compat_uptr_t       pvMemPa;
	/* [IN]     The memory byte alignment setting */
	compat_uint_t       eAlignment;
	/* [IN/OUT] The align memory virtual address */
	compat_uptr_t       pvAlignMemVa;
	/* [IN/OUT] The align memory physical address */
	compat_uptr_t       pvAlignMemPa;
	/* [IN]     The memory codec for VENC or VDEC */
	compat_uint_t       eMemCodec;
	compat_uint_t       i4IonShareFd;
	compat_uptr_t       pIonBufhandle;
	/* [IN/OUT] The reserved parameter */
	compat_uptr_t       pvReserved;
	/* [IN]     The size of reserved parameter structure */
	compat_ulong_t      u4ReservedSize;
} COMPAT_VAL_MEMORY_T;

static int get_uptr_to_32(compat_uptr_t *p, void __user **uptr)
{
	void __user *p2p;
	int err = get_user(p2p, uptr);
	*p = ptr_to_compat(p2p);
	return err;
}
static int compat_copy_struct(
	STRUCT_TYPE eType,
	COPY_DIRECTION eDirection,
	void __user *data32,
	void __user *data)
{
	compat_uint_t u;
	compat_ulong_t l;
	compat_uptr_t p;
	char c;
	int err = 0;

	switch (eType) {
	case VAL_HW_LOCK_TYPE:
	{
		if (eDirection == COPY_FROM_USER) {
			COMPAT_VAL_HW_LOCK_T __user *from32 = (COMPAT_VAL_HW_LOCK_T *)data32;
			VAL_HW_LOCK_T __user *to = (VAL_HW_LOCK_T *)data;

			err = get_user(p, &(from32->pvHandle));
			err |= put_user(compat_ptr(p), &(to->pvHandle));
			err |= get_user(u, &(from32->u4HandleSize));
			err |= put_user(u, &(to->u4HandleSize));
			err |= get_user(p, &(from32->pvLock));
			err |= put_user(compat_ptr(p), &(to->pvLock));
			err |= get_user(u, &(from32->u4TimeoutMs));
			err |= put_user(u, &(to->u4TimeoutMs));
			err |= get_user(p, &(from32->pvReserved));
			err |= put_user(compat_ptr(p), &(to->pvReserved));
			err |= get_user(u, &(from32->u4ReservedSize));
			err |= put_user(u, &(to->u4ReservedSize));
			err |= get_user(u, &(from32->eDriverType));
			err |= put_user(u, &(to->eDriverType));
			err |= get_user(c, &(from32->bSecureInst));
			err |= put_user(c, &(to->bSecureInst));
		} else {
			COMPAT_VAL_HW_LOCK_T __user *to32 = (COMPAT_VAL_HW_LOCK_T *)data32;
			VAL_HW_LOCK_T __user *from = (VAL_HW_LOCK_T *)data;

			err = get_uptr_to_32(&p, &(from->pvHandle));
			err |= put_user(p, &(to32->pvHandle));
			err |= get_user(u, &(from->u4HandleSize));
			err |= put_user(u, &(to32->u4HandleSize));
			err |= get_uptr_to_32(&p, &(from->pvLock));
			err |= put_user(p, &(to32->pvLock));
			err |= get_user(u, &(from->u4TimeoutMs));
			err |= put_user(u, &(to32->u4TimeoutMs));
			err |= get_uptr_to_32(&p, &(from->pvReserved));
			err |= put_user(p, &(to32->pvReserved));
			err |= get_user(u, &(from->u4ReservedSize));
			err |= put_user(u, &(to32->u4ReservedSize));
			err |= get_user(u, &(from->eDriverType));
			err |= put_user(u, &(to32->eDriverType));
			err |= get_user(c, &(from->bSecureInst));
			err |= put_user(c, &(to32->bSecureInst));
		}
	}
	break;
	case VAL_POWER_TYPE:
	{
		if (eDirection == COPY_FROM_USER) {
			COMPAT_VAL_POWER_T __user *from32 = (COMPAT_VAL_POWER_T *)data32;
			VAL_POWER_T __user *to = (VAL_POWER_T *)data;

			err = get_user(p, &(from32->pvHandle));
			err |= put_user(compat_ptr(p), &(to->pvHandle));
			err |= get_user(u, &(from32->u4HandleSize));
			err |= put_user(u, &(to->u4HandleSize));
			err |= get_user(u, &(from32->eDriverType));
			err |= put_user(u, &(to->eDriverType));
			err |= get_user(c, &(from32->fgEnable));
			err |= put_user(c, &(to->fgEnable));
			err |= get_user(p, &(from32->pvReserved));
			err |= put_user(compat_ptr(p), &(to->pvReserved));
			err |= get_user(u, &(from32->u4ReservedSize));
			err |= put_user(u, &(to->u4ReservedSize));
		} else {
			COMPAT_VAL_POWER_T __user *to32 = (COMPAT_VAL_POWER_T *)data32;
			VAL_POWER_T __user *from = (VAL_POWER_T *)data;

			err = get_uptr_to_32(&p, &(from->pvHandle));
			err |= put_user(p, &(to32->pvHandle));
			err |= get_user(u, &(from->u4HandleSize));
			err |= put_user(u, &(to32->u4HandleSize));
			err |= get_user(u, &(from->eDriverType));
			err |= put_user(u, &(to32->eDriverType));
			err |= get_user(c, &(from->fgEnable));
			err |= put_user(c, &(to32->fgEnable));
			err |= get_uptr_to_32(&p, &(from->pvReserved));
			err |= put_user(p, &(to32->pvReserved));
			err |= get_user(u, &(from->u4ReservedSize));
			err |= put_user(u, &(to32->u4ReservedSize));
		}
	}
	break;
	case VAL_ISR_TYPE:
	{
		int i = 0;

		if (eDirection == COPY_FROM_USER) {
			COMPAT_VAL_ISR_T __user *from32 = (COMPAT_VAL_ISR_T *)data32;
			VAL_ISR_T __user *to = (VAL_ISR_T *)data;

			err = get_user(p, &(from32->pvHandle));
			err |= put_user(compat_ptr(p), &(to->pvHandle));
			err |= get_user(u, &(from32->u4HandleSize));
			err |= put_user(u, &(to->u4HandleSize));
			err |= get_user(u, &(from32->eDriverType));
			err |= put_user(u, &(to->eDriverType));
			err |= get_user(p, &(from32->pvIsrFunction));
			err |= put_user(compat_ptr(p), &(to->pvIsrFunction));
			err |= get_user(p, &(from32->pvReserved));
			err |= put_user(compat_ptr(p), &(to->pvReserved));
			err |= get_user(u, &(from32->u4ReservedSize));
			err |= put_user(u, &(to->u4ReservedSize));
			err |= get_user(u, &(from32->u4TimeoutMs));
			err |= put_user(u, &(to->u4TimeoutMs));
			err |= get_user(u, &(from32->u4IrqStatusNum));
			err |= put_user(u, &(to->u4IrqStatusNum));
			for (; i < IRQ_STATUS_MAX_NUM; i++) {
				err |= get_user(u, &(from32->u4IrqStatus[i]));
				err |= put_user(u, &(to->u4IrqStatus[i]));
			}
			return err;

		} else {
			COMPAT_VAL_ISR_T __user *to32 = (COMPAT_VAL_ISR_T *)data32;
			VAL_ISR_T __user *from = (VAL_ISR_T *)data;

			err = get_uptr_to_32(&p, &(from->pvHandle));
			err |= put_user(p, &(to32->pvHandle));
			err |= get_user(u, &(from->u4HandleSize));
			err |= put_user(u, &(to32->u4HandleSize));
			err |= get_user(u, &(from->eDriverType));
			err |= put_user(u, &(to32->eDriverType));
			err |= get_uptr_to_32(&p, &(from->pvIsrFunction));
			err |= put_user(p, &(to32->pvIsrFunction));
			err |= get_uptr_to_32(&p, &(from->pvReserved));
			err |= put_user(p, &(to32->pvReserved));
			err |= get_user(u, &(from->u4ReservedSize));
			err |= put_user(u, &(to32->u4ReservedSize));
			err |= get_user(u, &(from->u4TimeoutMs));
			err |= put_user(u, &(to32->u4TimeoutMs));
			err |= get_user(u, &(from->u4IrqStatusNum));
			err |= put_user(u, &(to32->u4IrqStatusNum));
			for (; i < IRQ_STATUS_MAX_NUM; i++) {
				err |= get_user(u, &(from->u4IrqStatus[i]));
				err |= put_user(u, &(to32->u4IrqStatus[i]));
			}
		}
	}
	break;
	case VAL_MEMORY_TYPE:
	{
		if (eDirection == COPY_FROM_USER) {
			COMPAT_VAL_MEMORY_T __user *from32 = (COMPAT_VAL_MEMORY_T *)data32;
			VAL_MEMORY_T __user *to = (VAL_MEMORY_T *)data;

			err = get_user(u, &(from32->eMemType));
			err |= put_user(u, &(to->eMemType));
			err |= get_user(l, &(from32->u4MemSize));
			err |= put_user(l, &(to->u4MemSize));
			err |= get_user(p, &(from32->pvMemVa));
			err |= put_user(compat_ptr(p), &(to->pvMemVa));
			err |= get_user(p, &(from32->pvMemPa));
			err |= put_user(compat_ptr(p), &(to->pvMemPa));
			err |= get_user(u, &(from32->eAlignment));
			err |= put_user(u, &(to->eAlignment));
			err |= get_user(p, &(from32->pvAlignMemVa));
			err |= put_user(compat_ptr(p), &(to->pvAlignMemVa));
			err |= get_user(p, &(from32->pvAlignMemPa));
			err |= put_user(compat_ptr(p), &(to->pvAlignMemPa));
			err |= get_user(u, &(from32->eMemCodec));
			err |= put_user(u, &(to->eMemCodec));
			err |= get_user(u, &(from32->i4IonShareFd));
			err |= put_user(u, &(to->i4IonShareFd));
			err |= get_user(p, &(from32->pIonBufhandle));
			err |= put_user(compat_ptr(p), &(to->pIonBufhandle));
			err |= get_user(p, &(from32->pvReserved));
			err |= put_user(compat_ptr(p), &(to->pvReserved));
			err |= get_user(l, &(from32->u4ReservedSize));
			err |= put_user(l, &(to->u4ReservedSize));
		} else{
			COMPAT_VAL_MEMORY_T __user *to32 = (COMPAT_VAL_MEMORY_T *)data32;

			VAL_MEMORY_T __user *from = (VAL_MEMORY_T *)data;

			err = get_user(u, &(from->eMemType));
			err |= put_user(u, &(to32->eMemType));
			err |= get_user(l, &(from->u4MemSize));
			err |= put_user(l, &(to32->u4MemSize));
			err |= get_uptr_to_32(&p, &(from->pvMemVa));
			err |= put_user(p, &(to32->pvMemVa));
			err |= get_uptr_to_32(&p, &(from->pvMemPa));
			err |= put_user(p, &(to32->pvMemPa));
			err |= get_user(u, &(from->eAlignment));
			err |= put_user(u, &(to32->eAlignment));
			err |= get_uptr_to_32(&p, &(from->pvAlignMemVa));
			err |= put_user(p, &(to32->pvAlignMemVa));
			err |= get_uptr_to_32(&p, &(from->pvAlignMemPa));
			err |= put_user(p, &(to32->pvAlignMemPa));
			err |= get_user(u, &(from->eMemCodec));
			err |= put_user(u, &(to32->eMemCodec));
			err |= get_user(u, &(from->i4IonShareFd));
			err |= put_user(u, &(to32->i4IonShareFd));
			err |= get_uptr_to_32(&p, (void __user **)&(from->pIonBufhandle));
			err |= put_user(p, &(to32->pIonBufhandle));
			err |= get_uptr_to_32(&p, &(from->pvReserved));
			err |= put_user(p, &(to32->pvReserved));
			err |= get_user(l, &(from->u4ReservedSize));
			err |= put_user(l, &(to32->u4ReservedSize));
		}
	}
	break;
	default:
		break;
	}

	return err;
}


static long vcodec_unlocked_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	/* MODULE_MFV_LOGD("vcodec_unlocked_compat_ioctl: 0x%x\n", cmd); */
	switch (cmd) {
	case VCODEC_ALLOC_NON_CACHE_BUFFER:
	case VCODEC_FREE_NON_CACHE_BUFFER:
	{
		COMPAT_VAL_MEMORY_T __user *data32;
		VAL_MEMORY_T __user *data;
		int err;

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(VAL_MEMORY_T));
		if (data == NULL)
			return -EFAULT;

		err = compat_copy_struct(VAL_MEMORY_TYPE, COPY_FROM_USER, (void *)data32, (void *)data);
		if (err)
			return err;

		ret = file->f_op->unlocked_ioctl(file, cmd, (unsigned long)data);

		err = compat_copy_struct(VAL_MEMORY_TYPE, COPY_TO_USER, (void *)data32, (void *)data);

		if (err)
			return err;
		return ret;
	}
	break;
	case VCODEC_LOCKHW:
	case VCODEC_UNLOCKHW:
	{
		COMPAT_VAL_HW_LOCK_T __user *data32;
		VAL_HW_LOCK_T __user *data;
		int err;

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(VAL_HW_LOCK_T));
		if (data == NULL)
			return -EFAULT;

		err = compat_copy_struct(VAL_HW_LOCK_TYPE, COPY_FROM_USER, (void *)data32, (void *)data);
		if (err)
			return err;

		ret = file->f_op->unlocked_ioctl(file, cmd, (unsigned long)data);

		err = compat_copy_struct(VAL_HW_LOCK_TYPE, COPY_TO_USER, (void *)data32, (void *)data);

		if (err)
			return err;
		return ret;
	}
	break;

	case VCODEC_INC_PWR_USER:
	case VCODEC_DEC_PWR_USER:
	{
		COMPAT_VAL_POWER_T __user *data32;
		VAL_POWER_T __user *data;
		int err;

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(VAL_POWER_T));
		if (data == NULL)
			return -EFAULT;

		err = compat_copy_struct(VAL_POWER_TYPE, COPY_FROM_USER, (void *)data32, (void *)data);

		if (err)
			return err;

		ret = file->f_op->unlocked_ioctl(file, cmd, (unsigned long)data);

		err = compat_copy_struct(VAL_POWER_TYPE, COPY_TO_USER, (void *)data32, (void *)data);

		if (err)
			return err;
		return ret;
	}
	break;

	case VCODEC_WAITISR:
	{
		COMPAT_VAL_ISR_T __user *data32;
		VAL_ISR_T __user *data;
		int err;

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(VAL_ISR_T));
		if (data == NULL)
			return -EFAULT;

		err = compat_copy_struct(VAL_ISR_TYPE, COPY_FROM_USER, (void *)data32, (void *)data);
		if (err)
			return err;

		ret = file->f_op->unlocked_ioctl(file, VCODEC_WAITISR, (unsigned long)data);

		err = compat_copy_struct(VAL_ISR_TYPE, COPY_TO_USER, (void *)data32, (void *)data);

		if (err)
			return err;
		return ret;
	}
	break;

	default:
		return vcodec_unlocked_ioctl(file, cmd, arg);
	}
	return 0;
}
#else
#define vcodec_unlocked_compat_ioctl NULL
#endif
static int vcodec_open(struct inode *inode, struct file *file)
{
	MODULE_MFV_LOGD("vcodec_open\n");

	mutex_lock(&DriverOpenCountLock);
	Driver_Open_Count++;

	MODULE_MFV_LOGE("vcodec_open pid = %d, Driver_Open_Count %d\n", current->pid, Driver_Open_Count);
	mutex_unlock(&DriverOpenCountLock);


	/* TODO: Check upper limit of concurrent users? */

	return 0;
}

static int vcodec_flush(struct file *file, fl_owner_t id)
{
	MODULE_MFV_LOGD("vcodec_flush, curr_tid =%d\n", current->pid);
	MODULE_MFV_LOGE("vcodec_flush pid = %d, Driver_Open_Count %d\n", current->pid, Driver_Open_Count);

	return 0;
}

static int vcodec_release(struct inode *inode, struct file *file)
{
	VAL_ULONG_T ulFlagsLockHW, ulFlagsISR;

	/* dump_stack(); */
	MODULE_MFV_LOGD("vcodec_release, curr_tid =%d\n", current->pid);
	mutex_lock(&DriverOpenCountLock);
	MODULE_MFV_LOGE("vcodec_release pid = %d, Driver_Open_Count %d\n", current->pid, Driver_Open_Count);
	Driver_Open_Count--;

	if (Driver_Open_Count == 0) {
		mutex_lock(&VdecHWLock);
		gu4VdecLockThreadId = 0;
		grVcodecDecHWLock.pvHandle = 0;
		grVcodecDecHWLock.eDriverType = VAL_DRIVER_TYPE_NONE;
		grVcodecDecHWLock.rLockedTime.u4Sec = 0;
		grVcodecDecHWLock.rLockedTime.u4uSec = 0;
		mutex_unlock(&VdecHWLock);

		mutex_lock(&VencHWLock);
		grVcodecEncHWLock.pvHandle = 0;
		grVcodecEncHWLock.eDriverType = VAL_DRIVER_TYPE_NONE;
		grVcodecEncHWLock.rLockedTime.u4Sec = 0;
		grVcodecEncHWLock.rLockedTime.u4uSec = 0;
		mutex_unlock(&VencHWLock);

		mutex_lock(&DecEMILock);
		gu4DecEMICounter = 0;
		mutex_unlock(&DecEMILock);

		mutex_lock(&EncEMILock);
		gu4EncEMICounter = 0;
		mutex_unlock(&EncEMILock);

		mutex_lock(&PWRLock);
		gu4PWRCounter = 0;
		mutex_unlock(&PWRLock);

#if defined(VENC_USE_L2C)
		mutex_lock(&L2CLock);
		if (gu4L2CCounter != 0) {
			MODULE_MFV_LOGE("vcodec_flush pid = %d, L2 user = %d, force restore L2 settings\n",
				 current->pid, gu4L2CCounter);
			if (config_L2(1)) {
				/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
				MODULE_MFV_LOGE("[VCODEC][ERROR] restore L2 settings failed\n");
			}
		}
		gu4L2CCounter = 0;
		mutex_unlock(&L2CLock);
#endif
		spin_lock_irqsave(&LockDecHWCountLock, ulFlagsLockHW);
		gu4LockDecHWCount = 0;
		spin_unlock_irqrestore(&LockDecHWCountLock, ulFlagsLockHW);

		spin_lock_irqsave(&LockEncHWCountLock, ulFlagsLockHW);
		gu4LockEncHWCount = 0;
		spin_unlock_irqrestore(&LockEncHWCountLock, ulFlagsLockHW);

		spin_lock_irqsave(&DecISRCountLock, ulFlagsISR);
		gu4DecISRCount = 0;
		spin_unlock_irqrestore(&DecISRCountLock, ulFlagsISR);

		spin_lock_irqsave(&EncISRCountLock, ulFlagsISR);
		gu4EncISRCount = 0;
		spin_unlock_irqrestore(&EncISRCountLock, ulFlagsISR);
	}

	mutex_unlock(&DriverOpenCountLock);

	return 0;
}

void vcodec_vma_open(struct vm_area_struct *vma)
{
	MODULE_MFV_LOGD("vcodec VMA open, virt %lx, phys %lx\n", vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}

void vcodec_vma_close(struct vm_area_struct *vma)
{
	MODULE_MFV_LOGD("vcodec VMA close, virt %lx, phys %lx\n", vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}

static struct vm_operations_struct vcodec_remap_vm_ops = {
	.open = vcodec_vma_open,
	.close = vcodec_vma_close,
};

static int vcodec_mmap(struct file *file, struct vm_area_struct *vma)
{
#if 1
	VAL_UINT32_T u4I = 0;
	VAL_ULONG_T length;
	VAL_ULONG_T pfn;

	length = vma->vm_end - vma->vm_start;
	pfn = vma->vm_pgoff << PAGE_SHIFT;

	if (((length > VENC_REGION) || (pfn < VENC_BASE) || (pfn > VENC_BASE + VENC_REGION)) &&
	    ((length > VDEC_REGION) || (pfn < VDEC_BASE_PHY) || (pfn > VDEC_BASE_PHY + VDEC_REGION)) &&
	    ((length > HW_REGION) || (pfn < HW_BASE) || (pfn > HW_BASE + HW_REGION)) &&
	    ((length > INFO_REGION) || (pfn < INFO_BASE) || (pfn > INFO_BASE + INFO_REGION))
	   ) {
		VAL_ULONG_T ulAddr, ulSize;

		for (u4I = 0; u4I < VCODEC_MULTIPLE_INSTANCE_NUM_x_10; u4I++) {
			if ((grNonCacheMemoryList[u4I].ulKVA != -1L) && (grNonCacheMemoryList[u4I].ulKPA != -1L)) {
				ulAddr = grNonCacheMemoryList[u4I].ulKPA;
				ulSize = (grNonCacheMemoryList[u4I].ulSize + 0x1000 - 1) & ~(0x1000 - 1);
				if ((length == ulSize) && (pfn == ulAddr)) {
					MODULE_MFV_LOGD("[VCODEC] cache idx %d\n", u4I);
					break;
				}
			}
		}

		if (u4I == VCODEC_MULTIPLE_INSTANCE_NUM_x_10) {
			MODULE_MFV_LOGE("[VCODEC][ERROR] mmap region error: Length(0x%lx), pfn(0x%lx)\n",
				 (VAL_ULONG_T)length, pfn);
			return -EAGAIN;
		}
	}
#endif
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	MODULE_MFV_LOGD("[VCODEC][mmap] vma->start 0x%lx, vma->end 0x%lx, vma->pgoff 0x%lx\n",
		 (VAL_ULONG_T)vma->vm_start, (VAL_ULONG_T)vma->vm_end, (VAL_ULONG_T)vma->vm_pgoff);
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		return -EAGAIN;
	}

	vma->vm_ops = &vcodec_remap_vm_ops;
	vcodec_vma_open(vma);

	return 0;
}

static const struct file_operations vcodec_fops = {
	.owner      = THIS_MODULE,
	.unlocked_ioctl = vcodec_unlocked_ioctl,
	.open       = vcodec_open,
	.flush      = vcodec_flush,
	.release    = vcodec_release,
	.mmap       = vcodec_mmap,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl   = vcodec_unlocked_compat_ioctl,
#endif

};

static int vcodec_probe(struct platform_device *dev)
{
	int ret;

	MODULE_MFV_LOGD("+vcodec_probe\n");

	mutex_lock(&DecEMILock);
	gu4DecEMICounter = 0;
	mutex_unlock(&DecEMILock);

	mutex_lock(&EncEMILock);
	gu4EncEMICounter = 0;
	mutex_unlock(&EncEMILock);

	mutex_lock(&PWRLock);
	gu4PWRCounter = 0;
	mutex_unlock(&PWRLock);

	mutex_lock(&L2CLock);
	gu4L2CCounter = 0;
	mutex_unlock(&L2CLock);

	ret = register_chrdev_region(vcodec_devno, 1, VCODEC_DEVNAME);
	if (ret) {
		/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
		MODULE_MFV_LOGE("[ERROR] Can't Get Major number for VCodec Device\n");
	}

	vcodec_cdev = cdev_alloc();
	vcodec_cdev->owner = THIS_MODULE;
	vcodec_cdev->ops = &vcodec_fops;

	ret = cdev_add(vcodec_cdev, vcodec_devno, 1);
	if (ret) {
		/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
		MODULE_MFV_LOGE("[ERROR] Can't add Vcodec Device\n");
	}

	vcodec_class = class_create(THIS_MODULE, VCODEC_DEVNAME);
	if (IS_ERR(vcodec_class)) {
		ret = PTR_ERR(vcodec_class);
		MODULE_MFV_LOGE("[VCODEC][ERROR] Unable to create class, err = %d", ret);
		return ret;
	}

	vcodec_device = device_create(vcodec_class, NULL, vcodec_devno, NULL, VCODEC_DEVNAME);

	if (request_irq(VDEC_IRQ_ID , (irq_handler_t)video_intr_dlr, IRQF_TRIGGER_LOW, VCODEC_DEVNAME, NULL) < 0) {
		/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
		MODULE_MFV_LOGE("[VCODEC][ERROR] error to request dec irq\n");
	} else {
		MODULE_MFV_LOGD("[VCODEC] success to request dec irq: %d\n", VDEC_IRQ_ID);
	}

	if (request_irq(VENC_IRQ_ID , (irq_handler_t)video_intr_dlr2, IRQF_TRIGGER_LOW, VCODEC_DEVNAME, NULL) < 0) {
		/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
		MODULE_MFV_LOGD("[VCODEC][ERROR] error to request enc irq\n");
	} else {
		MODULE_MFV_LOGD("[VCODEC] success to request enc irq: %d\n", VENC_IRQ_ID);
	}

	disable_irq(VDEC_IRQ_ID);
	disable_irq(VENC_IRQ_ID);

#ifndef CONFIG_MTK_CLKMGR
	clk_MT_CG_DISP0_SMI_COMMON = devm_clk_get(&dev->dev, "MT_CG_DISP0_SMI_COMMON");
	if (IS_ERR(clk_MT_CG_DISP0_SMI_COMMON)) {
		MODULE_MFV_LOGE("[VCODEC][ERROR] Unable to devm_clk_get MT_CG_DISP0_SMI_COMMON\n");
		return PTR_ERR(clk_MT_CG_DISP0_SMI_COMMON);
	}

	clk_MT_CG_VDEC0_VDEC = devm_clk_get(&dev->dev, "MT_CG_VDEC0_VDEC");
	if (IS_ERR(clk_MT_CG_VDEC0_VDEC)) {
		MODULE_MFV_LOGE("[VCODEC][ERROR] Unable to devm_clk_get MT_CG_VDEC0_VDEC\n");
		return PTR_ERR(clk_MT_CG_VDEC0_VDEC);
	}

	clk_MT_CG_VDEC1_LARB = devm_clk_get(&dev->dev, "MT_CG_VDEC1_LARB");
	if (IS_ERR(clk_MT_CG_VDEC1_LARB)) {
		MODULE_MFV_LOGE("[VCODEC][ERROR] Unable to devm_clk_get MT_CG_VDEC1_LARB\n");
		return PTR_ERR(clk_MT_CG_VDEC1_LARB);
	}

	clk_MT_CG_VENC_VENC = devm_clk_get(&dev->dev, "MT_CG_VENC_VENC");
	if (IS_ERR(clk_MT_CG_VENC_VENC)) {
		MODULE_MFV_LOGE("[VCODEC][ERROR] Unable to devm_clk_get MT_CG_VENC_VENC\n");
		return PTR_ERR(clk_MT_CG_VENC_VENC);
	}

	clk_MT_CG_VENC_LARB = devm_clk_get(&dev->dev, "MT_CG_VENC_LARB");
	if (IS_ERR(clk_MT_CG_VENC_LARB)) {
		MODULE_MFV_LOGE("[VCODEC][ERROR] Unable to devm_clk_get MT_CG_VENC_LARB\n");
		return PTR_ERR(clk_MT_CG_VENC_LARB);
	}

	clk_MT_SCP_SYS_VDE = devm_clk_get(&dev->dev, "MT_SCP_SYS_VDE");
	if (IS_ERR(clk_MT_SCP_SYS_VDE)) {
		MODULE_MFV_LOGE("[VCODEC][ERROR] Unable to devm_clk_get MT_SCP_SYS_VDE\n");
		return PTR_ERR(clk_MT_SCP_SYS_VDE);
	}

	clk_MT_SCP_SYS_VEN = devm_clk_get(&dev->dev, "MT_SCP_SYS_VEN");
	if (IS_ERR(clk_MT_SCP_SYS_VEN)) {
		MODULE_MFV_LOGE("[VCODEC][ERROR] Unable to devm_clk_get MT_SCP_SYS_VEN\n");
		return PTR_ERR(clk_MT_SCP_SYS_VEN);
	}

	clk_MT_SCP_SYS_DIS = devm_clk_get(&dev->dev, "MT_SCP_SYS_DIS");
	if (IS_ERR(clk_MT_SCP_SYS_DIS)) {
		MODULE_MFV_LOGE("[VCODEC][ERROR] Unable to devm_clk_get MT_SCP_SYS_DIS\n");
		return PTR_ERR(clk_MT_SCP_SYS_DIS);
	}
#endif

	MODULE_MFV_LOGD("vcodec_probe Done\n");

#ifdef KS_POWER_WORKAROUND
	vdec_power_on();
	venc_power_on();
#endif

	return 0;
}

static int vcodec_remove(struct platform_device *pDev)
{
	MODULE_MFV_LOGD("vcodec_remove\n");
	return 0;
}

#ifdef CONFIG_MTK_HIBERNATION
/* extern void mt_irq_set_sens(unsigned int irq, unsigned int sens); */
/* extern void mt_irq_set_polarity(unsigned int irq, unsigned int polarity); */
static int vcodec_pm_restore_noirq(struct device *device)
{
	/* vdec: IRQF_TRIGGER_LOW */
	mt_irq_set_sens(VDEC_IRQ_ID, MT_LEVEL_SENSITIVE);
	mt_irq_set_polarity(VDEC_IRQ_ID, MT_POLARITY_LOW);
	/* venc: IRQF_TRIGGER_LOW */
	mt_irq_set_sens(VENC_IRQ_ID, MT_LEVEL_SENSITIVE);
	mt_irq_set_polarity(VENC_IRQ_ID, MT_POLARITY_LOW);

	return 0;
}
#endif

static const struct of_device_id vcodec_of_match[] = {
	{ .compatible = "mediatek,mt6755-vdec_gcon", },
	{/* sentinel */}
};

MODULE_DEVICE_TABLE(of, vcodec_of_match);

static struct platform_driver vcodec_driver = {
	.probe = vcodec_probe,
	.remove = vcodec_remove,
	/*
	    .suspend = vcodec_suspend,
	    .resume = vcodec_resume,
	*/
	.driver = {
		.name  = VCODEC_DEVNAME,
		.owner = THIS_MODULE,
		.of_match_table = vcodec_of_match,
	},
};

static int __init vcodec_driver_init(void)
{
	VAL_RESULT_T  eValHWLockRet;
	VAL_ULONG_T ulFlags, ulFlagsLockHW, ulFlagsISR;

	MODULE_MFV_LOGD("+vcodec_driver_init !!\n");

	mutex_lock(&DriverOpenCountLock);
	Driver_Open_Count = 0;
	mutex_unlock(&DriverOpenCountLock);

	mutex_lock(&LogCountLock);
	gu4LogCountUser = 0;
	gu4LogCount = 0;
	mutex_unlock(&LogCountLock);

	{
		struct device_node *node = NULL;

		node = of_find_compatible_node(NULL, NULL, "mediatek,mt6755-venc");
		KVA_VENC_BASE = (VAL_ULONG_T)of_iomap(node, 0);
		VENC_IRQ_ID =  irq_of_parse_and_map(node, 0);
		KVA_VENC_IRQ_STATUS_ADDR =    KVA_VENC_BASE + 0x05C;
		KVA_VENC_IRQ_ACK_ADDR  = KVA_VENC_BASE + 0x060;
	}

	{
		struct device_node *node = NULL;

		node = of_find_compatible_node(NULL, NULL, "mediatek,mt6755-vdec_full_top");
		KVA_VDEC_BASE = (VAL_ULONG_T)of_iomap(node, 0);
		VDEC_IRQ_ID =  irq_of_parse_and_map(node, 0);
		KVA_VDEC_MISC_BASE = KVA_VDEC_BASE + 0x0000;
		KVA_VDEC_VLD_BASE = KVA_VDEC_BASE + 0x1000;
	}
	{
		struct device_node *node = NULL;

		node = of_find_compatible_node(NULL, NULL, "mediatek,mt6755-vdec_gcon");
		KVA_VDEC_GCON_BASE = (VAL_ULONG_T)of_iomap(node, 0);

		MODULE_MFV_LOGD("[VCODEC][DeviceTree] KVA_VENC_BASE(0x%lx), KVA_VDEC_BASE(0x%lx), KVA_VDEC_GCON_BASE(0x%lx)",
			 KVA_VENC_BASE, KVA_VDEC_BASE, KVA_VDEC_GCON_BASE);
		MODULE_MFV_LOGD("[VCODEC][DeviceTree] VDEC_IRQ_ID(%d), VENC_IRQ_ID(%d)",
			 VDEC_IRQ_ID, VENC_IRQ_ID);
	}

	/* KVA_VENC_IRQ_STATUS_ADDR =    (VAL_ULONG_T)ioremap(VENC_IRQ_STATUS_addr, 4); */
	/* KVA_VENC_IRQ_ACK_ADDR  = (VAL_ULONG_T)ioremap(VENC_IRQ_ACK_addr, 4); */

	spin_lock_irqsave(&LockDecHWCountLock, ulFlagsLockHW);
	gu4LockDecHWCount = 0;
	spin_unlock_irqrestore(&LockDecHWCountLock, ulFlagsLockHW);

	spin_lock_irqsave(&LockEncHWCountLock, ulFlagsLockHW);
	gu4LockEncHWCount = 0;
	spin_unlock_irqrestore(&LockEncHWCountLock, ulFlagsLockHW);

	spin_lock_irqsave(&DecISRCountLock, ulFlagsISR);
	gu4DecISRCount = 0;
	spin_unlock_irqrestore(&DecISRCountLock, ulFlagsISR);

	spin_lock_irqsave(&EncISRCountLock, ulFlagsISR);
	gu4EncISRCount = 0;
	spin_unlock_irqrestore(&EncISRCountLock, ulFlagsISR);

	mutex_lock(&VdecPWRLock);
	gu4VdecPWRCounter = 0;
	mutex_unlock(&VdecPWRLock);

	mutex_lock(&VencPWRLock);
	gu4VencPWRCounter = 0;
	mutex_unlock(&VencPWRLock);

	mutex_lock(&IsOpenedLock);
	if (VAL_FALSE == bIsOpened) {
		bIsOpened = VAL_TRUE;
		/* vcodec_probe(NULL); */
	}
	mutex_unlock(&IsOpenedLock);

	mutex_lock(&VdecHWLock);
	gu4VdecLockThreadId = 0;
	grVcodecDecHWLock.pvHandle = 0;
	grVcodecDecHWLock.eDriverType = VAL_DRIVER_TYPE_NONE;
	grVcodecDecHWLock.rLockedTime.u4Sec = 0;
	grVcodecDecHWLock.rLockedTime.u4uSec = 0;
	mutex_unlock(&VdecHWLock);

	mutex_lock(&VencHWLock);
	grVcodecEncHWLock.pvHandle = 0;
	grVcodecEncHWLock.eDriverType = VAL_DRIVER_TYPE_NONE;
	grVcodecEncHWLock.rLockedTime.u4Sec = 0;
	grVcodecEncHWLock.rLockedTime.u4uSec = 0;
	mutex_unlock(&VencHWLock);

	/* HWLockEvent part */
	mutex_lock(&DecHWLockEventTimeoutLock);
	DecHWLockEvent.pvHandle = "DECHWLOCK_EVENT";
	DecHWLockEvent.u4HandleSize = sizeof("DECHWLOCK_EVENT") + 1;
	DecHWLockEvent.u4TimeoutMs = 1;
	mutex_unlock(&DecHWLockEventTimeoutLock);
	eValHWLockRet = eVideoCreateEvent(&DecHWLockEvent, sizeof(VAL_EVENT_T));
	if (VAL_RESULT_NO_ERROR != eValHWLockRet) {
		/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
		MODULE_MFV_LOGE("[VCODEC][ERROR] create dec hwlock event error\n");
	}

	mutex_lock(&EncHWLockEventTimeoutLock);
	EncHWLockEvent.pvHandle = "ENCHWLOCK_EVENT";
	EncHWLockEvent.u4HandleSize = sizeof("ENCHWLOCK_EVENT") + 1;
	EncHWLockEvent.u4TimeoutMs = 1;
	mutex_unlock(&EncHWLockEventTimeoutLock);
	eValHWLockRet = eVideoCreateEvent(&EncHWLockEvent, sizeof(VAL_EVENT_T));
	if (VAL_RESULT_NO_ERROR != eValHWLockRet) {
		/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
		MODULE_MFV_LOGE("[VCODEC][ERROR] create enc hwlock event error\n");
	}

	/* IsrEvent part */
	spin_lock_irqsave(&DecIsrLock, ulFlags);
	DecIsrEvent.pvHandle = "DECISR_EVENT";
	DecIsrEvent.u4HandleSize = sizeof("DECISR_EVENT") + 1;
	DecIsrEvent.u4TimeoutMs = 1;
	spin_unlock_irqrestore(&DecIsrLock, ulFlags);
	eValHWLockRet = eVideoCreateEvent(&DecIsrEvent, sizeof(VAL_EVENT_T));
	if (VAL_RESULT_NO_ERROR != eValHWLockRet) {
		/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
		MODULE_MFV_LOGE("[VCODEC][ERROR] create dec isr event error\n");
	}

	spin_lock_irqsave(&EncIsrLock, ulFlags);
	EncIsrEvent.pvHandle = "ENCISR_EVENT";
	EncIsrEvent.u4HandleSize = sizeof("ENCISR_EVENT") + 1;
	EncIsrEvent.u4TimeoutMs = 1;
	spin_unlock_irqrestore(&EncIsrLock, ulFlags);
	eValHWLockRet = eVideoCreateEvent(&EncIsrEvent, sizeof(VAL_EVENT_T));
	if (VAL_RESULT_NO_ERROR != eValHWLockRet) {
		/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
		MODULE_MFV_LOGE("[VCODEC][ERROR] create enc isr event error\n");
	}

	MODULE_MFV_LOGD("vcodec_driver_init Done\n");

#ifdef CONFIG_MTK_HIBERNATION
	register_swsusp_restore_noirq_func(ID_M_VCODEC, vcodec_pm_restore_noirq, NULL);
#endif

	return platform_driver_register(&vcodec_driver);
}

static void __exit vcodec_driver_exit(void)
{
	VAL_RESULT_T  eValHWLockRet;

	MODULE_MFV_LOGD("vcodec_driver_exit\n");

	mutex_lock(&IsOpenedLock);
	if (VAL_TRUE == bIsOpened) {
		/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
		bIsOpened = VAL_FALSE;
	}
	mutex_unlock(&IsOpenedLock);

	cdev_del(vcodec_cdev);
	unregister_chrdev_region(vcodec_devno, 1);

	/* [TODO] iounmap the following? */
#if 0
	iounmap((void *)KVA_VENC_IRQ_STATUS_ADDR);
	iounmap((void *)KVA_VENC_IRQ_ACK_ADDR);
#endif

	free_irq(VENC_IRQ_ID, NULL);
	free_irq(VDEC_IRQ_ID, NULL);

	/* MT6589_HWLockEvent part */
	eValHWLockRet = eVideoCloseEvent(&DecHWLockEvent, sizeof(VAL_EVENT_T));
	if (VAL_RESULT_NO_ERROR != eValHWLockRet) {
		/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
		MODULE_MFV_LOGE("[VCODEC][ERROR] close dec hwlock event error\n");
	}

	eValHWLockRet = eVideoCloseEvent(&EncHWLockEvent, sizeof(VAL_EVENT_T));
	if (VAL_RESULT_NO_ERROR != eValHWLockRet) {
		/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
		MODULE_MFV_LOGE("[VCODEC][ERROR] close enc hwlock event error\n");
	}

	/* MT6589_IsrEvent part */
	eValHWLockRet = eVideoCloseEvent(&DecIsrEvent, sizeof(VAL_EVENT_T));
	if (VAL_RESULT_NO_ERROR != eValHWLockRet) {
		/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
		MODULE_MFV_LOGE("[VCODEC][ERROR] close dec isr event error\n");
	}

	eValHWLockRet = eVideoCloseEvent(&EncIsrEvent, sizeof(VAL_EVENT_T));
	if (VAL_RESULT_NO_ERROR != eValHWLockRet) {
		/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
		MODULE_MFV_LOGE("[VCODEC][ERROR] close enc isr event error\n");
	}

#ifdef CONFIG_MTK_HIBERNATION
	unregister_swsusp_restore_noirq_func(ID_M_VCODEC);
#endif

	platform_driver_unregister(&vcodec_driver);
}

module_init(vcodec_driver_init);
module_exit(vcodec_driver_exit);
MODULE_AUTHOR("Legis, Lu <legis.lu@mediatek.com>");
MODULE_DESCRIPTION("Jade Vcodec Driver");
MODULE_LICENSE("GPL");
