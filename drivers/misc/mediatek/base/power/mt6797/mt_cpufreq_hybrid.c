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
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/hrtimer.h>
#include <linux/kthread.h>
#include <linux/sched/rt.h>
#include <linux/atomic.h>
#include <linux/mutex.h>

#include <mt-plat/sync_write.h>
#include <mt-plat/mt_lpae.h>
#include "mt_cpufreq_hybrid.h"


/**************************************
 * [Hybrid DVFS] Config
 **************************************/


/**************************************
 * [CPU SPM] Parameter
 **************************************/
#define CSRAM_BASE		0x0012a000
#define CSRAM_SIZE		0x3000		/* 12K bytes */

#define OFFS_DATA_S		0x02a0
#define OFFS_WDT_LATCH0		0x02a4
#define OFFS_WDT_LATCH1		0x02a8
#define OFFS_WDT_LATCH2		0x02ac
#define OFFS_FW_RSV3		0x02b0
#define OFFS_FW_RSV4		0x02b4
#define OFFS_FW_RSV5		0x02b8
#define OFFS_FW_RSV6		0x02bc
#define OFFS_FW_RSV0		0x02c0
#define OFFS_FW_RSV1		0x02c4
#define OFFS_FW_RSV2		0x02c8
#define OFFS_TIMER_OUT1		0x02d0
#define OFFS_TIMER_OUT2		0x02d4
#define OFFS_PCM_PC1		0x02d8
#define OFFS_PCM_PC2		0x02dc
#define OFFS_INIT_OPP		0x02e0
#define OFFS_INIT_FREQ		0x02f0
#define OFFS_INIT_VOLT		0x0300
#define OFFS_INIT_VSRAM		0x0310
#define OFFS_SW_RSV0		0x0320
#define OFFS_SW_RSV1		0x0324
#define OFFS_SW_RSV2		0x0328
#define OFFS_PAUSE_SRC		0x0330
#define OFFS_DVFS_WAIT		0x0334
#define OFFS_FUNC_ENTER		0x0338
#define OFFS_NXLOG_OFFS		0x033c
#define OFFS_HWGOV_EN		0x0340
#define OFFS_HWGOV_STIME	0x0344
#define OFFS_HWGOV_ETIME	0x0348
#define OFFS_PAUSE_TIME		0x0350
#define OFFS_UNPAUSE_TIME	0x0354
#define OFFS_LOG_S		0x03d0
#define OFFS_LOG_E		0x2ff8

#define DVFS_TIMEOUT		6000		/* us */
#define SEMA_GET_TIMEOUT	2000		/* us */
#define PAUSE_TIMEOUT		2000		/* us */

#define DVFS_NOTIFY_INTV	20		/* ms */
#define MAX_LOG_FETCH		20


/**************************************
 * [Hybrid DVFS] Parameter
 **************************************/
#define ENTRY_EACH_LOG		6		/* need to sync with fetch_dvfs_log_and_notify */


/**************************************
 * [CPU SPM] Definition
 **************************************/
#define CSPM_BASE		cspm_base	/* map 0x11015000 */

#define CSPM_POWERON_CONFIG_EN		(CSPM_BASE + 0x000)
#define CSPM_POWER_ON_VAL0		(CSPM_BASE + 0x004)
#define CSPM_POWER_ON_VAL1		(CSPM_BASE + 0x008)
#define CSPM_CLK_CON			(CSPM_BASE + 0x00c)
#define CSPM_CLK_SETTLE			(CSPM_BASE + 0x010)
#define CSPM_AP_STANDBY_CON		(CSPM_BASE + 0x014)
#define CSPM_PCM_CON0			(CSPM_BASE + 0x018)
#define CSPM_PCM_CON1			(CSPM_BASE + 0x01c)
#define CSPM_PCM_IM_PTR			(CSPM_BASE + 0x020)
#define CSPM_PCM_IM_LEN			(CSPM_BASE + 0x024)
#define CSPM_PCM_REG_DATA_INI		(CSPM_BASE + 0x028)
#define CSPM_PCM_PWR_IO_EN		(CSPM_BASE + 0x02c)
#define CSPM_PCM_TIMER_VAL		(CSPM_BASE + 0x030)
#define CSPM_PCM_WDT_VAL		(CSPM_BASE + 0x034)
#define CSPM_PCM_IM_HOST_RW_PTR		(CSPM_BASE + 0x038)
#define CSPM_PCM_IM_HOST_RW_DAT		(CSPM_BASE + 0x03c)
#define CSPM_PCM_EVENT_VECTOR0		(CSPM_BASE + 0x040)
#define CSPM_PCM_EVENT_VECTOR1		(CSPM_BASE + 0x044)
#define CSPM_PCM_EVENT_VECTOR2		(CSPM_BASE + 0x048)
#define CSPM_PCM_EVENT_VECTOR3		(CSPM_BASE + 0x04c)
#define CSPM_PCM_EVENT_VECTOR4		(CSPM_BASE + 0x050)
#define CSPM_PCM_EVENT_VECTOR5		(CSPM_BASE + 0x054)
#define CSPM_PCM_EVENT_VECTOR6		(CSPM_BASE + 0x058)
#define CSPM_PCM_EVENT_VECTOR7		(CSPM_BASE + 0x05c)
#define CSPM_PCM_EVENT_VECTOR8		(CSPM_BASE + 0x060)
#define CSPM_PCM_EVENT_VECTOR9		(CSPM_BASE + 0x064)
#define CSPM_PCM_EVENT_VECTOR10		(CSPM_BASE + 0x068)
#define CSPM_PCM_EVENT_VECTOR11		(CSPM_BASE + 0x06c)
#define CSPM_PCM_EVENT_VECTOR12		(CSPM_BASE + 0x070)
#define CSPM_PCM_EVENT_VECTOR13		(CSPM_BASE + 0x074)
#define CSPM_PCM_EVENT_VECTOR14		(CSPM_BASE + 0x078)
#define CSPM_PCM_EVENT_VECTOR15		(CSPM_BASE + 0x07c)
#define CSPM_PCM_EVENT_VECTOR_EN	(CSPM_BASE + 0x080)
#define CSPM_SW_INT_SET			(CSPM_BASE + 0x090)
#define CSPM_SW_INT_CLEAR		(CSPM_BASE + 0x094)
#define CSPM_SCP_MAILBOX		(CSPM_BASE + 0x098)
#define CSPM_SCP_IRQ			(CSPM_BASE + 0x09c)
#define CSPM_TWAM_CON			(CSPM_BASE + 0x0a0)
#define CSPM_TWAM_WINDOW_LEN		(CSPM_BASE + 0x0a4)
#define CSPM_TWAM_IDLE_SEL		(CSPM_BASE + 0x0a8)
#define CSPM_CPU_WAKEUP_EVENT		(CSPM_BASE + 0x0b0)
#define CSPM_IRQ_MASK			(CSPM_BASE + 0x0b4)
#define CSPM_SRC_REQ			(CSPM_BASE + 0x0b8)
#define CSPM_SRC_MASK			(CSPM_BASE + 0x0bc)
#define CSPM_SRC2_MASK			(CSPM_BASE + 0x0c0)
#define CSPM_WAKEUP_EVENT_MASK		(CSPM_BASE + 0x0c4)
#define CSPM_WAKEUP_EVENT_EXT_MASK	(CSPM_BASE + 0x0c8)
#define CSPM_SCP_CLK_CON		(CSPM_BASE + 0x0d0)
#define CSPM_PCM_DEBUG_CON		(CSPM_BASE + 0x0d4)
#define CSPM_TWAM_CON0			(CSPM_BASE + 0x0e0)
#define CSPM_TWAM_CON1			(CSPM_BASE + 0x0e4)
#define CSPM_TWAM_CON2			(CSPM_BASE + 0x0e8)
#define CSPM_TWAM_CON3			(CSPM_BASE + 0x0ec)
#define CSPM_PCM_REG0_DATA		(CSPM_BASE + 0x100)
#define CSPM_PCM_REG1_DATA		(CSPM_BASE + 0x104)
#define CSPM_PCM_REG2_DATA		(CSPM_BASE + 0x108)
#define CSPM_PCM_REG3_DATA		(CSPM_BASE + 0x10c)
#define CSPM_PCM_REG4_DATA		(CSPM_BASE + 0x110)
#define CSPM_PCM_REG5_DATA		(CSPM_BASE + 0x114)
#define CSPM_PCM_REG6_DATA		(CSPM_BASE + 0x118)
#define CSPM_PCM_REG7_DATA		(CSPM_BASE + 0x11c)
#define CSPM_PCM_REG8_DATA		(CSPM_BASE + 0x120)
#define CSPM_PCM_REG9_DATA		(CSPM_BASE + 0x124)
#define CSPM_PCM_REG10_DATA		(CSPM_BASE + 0x128)
#define CSPM_PCM_REG11_DATA		(CSPM_BASE + 0x12c)
#define CSPM_PCM_REG12_DATA		(CSPM_BASE + 0x130)
#define CSPM_PCM_REG13_DATA		(CSPM_BASE + 0x134)
#define CSPM_PCM_REG14_DATA		(CSPM_BASE + 0x138)
#define CSPM_PCM_REG15_DATA		(CSPM_BASE + 0x13c)
#define CSPM_PCM_REG12_MASK_B_STA	(CSPM_BASE + 0x140)
#define CSPM_PCM_REG12_EXT_DATA		(CSPM_BASE + 0x144)
#define CSPM_PCM_REG12_EXT_MASK_B_STA	(CSPM_BASE + 0x148)
#define CSPM_PCM_EVENT_REG_STA		(CSPM_BASE + 0x14c)
#define CSPM_PCM_TIMER_OUT		(CSPM_BASE + 0x150)
#define CSPM_PCM_WDT_OUT		(CSPM_BASE + 0x154)
#define CSPM_IRQ_STA			(CSPM_BASE + 0x158)
#define CSPM_WAKEUP_STA			(CSPM_BASE + 0x15c)
#define CSPM_WAKEUP_EXT_STA		(CSPM_BASE + 0x160)
#define CSPM_WAKEUP_MISC		(CSPM_BASE + 0x164)
#define CSPM_BUS_PROTECT_RDY		(CSPM_BASE + 0x168)
#define CSPM_BUS_PROTECT2_RDY		(CSPM_BASE + 0x16c)
#define CSPM_SUBSYS_IDLE_STA		(CSPM_BASE + 0x170)
#define CSPM_CPU_IDLE_STA		(CSPM_BASE + 0x174)
#define CSPM_PCM_FSM_STA		(CSPM_BASE + 0x178)
#define CSPM_PCM_WDT_LATCH0		(CSPM_BASE + 0x190)
#define CSPM_PCM_WDT_LATCH1		(CSPM_BASE + 0x194)
#define CSPM_PCM_WDT_LATCH2		(CSPM_BASE + 0x198)
#define CSPM_DRAMC_DBG_LATCH		(CSPM_BASE + 0x19c)
#define CSPM_TWAM_LAST_STA0		(CSPM_BASE + 0x1a0)
#define CSPM_TWAM_LAST_STA1		(CSPM_BASE + 0x1a4)
#define CSPM_TWAM_LAST_STA2		(CSPM_BASE + 0x1a8)
#define CSPM_TWAM_LAST_STA3		(CSPM_BASE + 0x1ac)
#define CSPM_TWAM_LAST_STA4		(CSPM_BASE + 0x1b0)
#define CSPM_TWAM_LAST_STA5		(CSPM_BASE + 0x1b4)
#define CSPM_TWAM_LAST_STA6		(CSPM_BASE + 0x1b8)
#define CSPM_TWAM_LAST_STA7		(CSPM_BASE + 0x1bc)
#define CSPM_TWAM_LAST_STA8		(CSPM_BASE + 0x1c0)
#define CSPM_TWAM_LAST_STA9		(CSPM_BASE + 0x1c4)
#define CSPM_TWAM_LAST_STA10		(CSPM_BASE + 0x1c8)
#define CSPM_TWAM_LAST_STA11		(CSPM_BASE + 0x1cc)
#define CSPM_TWAM_LAST_STA12		(CSPM_BASE + 0x1d0)
#define CSPM_TWAM_LAST_STA13		(CSPM_BASE + 0x1d4)
#define CSPM_TWAM_LAST_STA14		(CSPM_BASE + 0x1d8)
#define CSPM_TWAM_LAST_STA15		(CSPM_BASE + 0x1dc)
#define CSPM_TWAM_CURR_STA0		(CSPM_BASE + 0x1e0)
#define CSPM_TWAM_CURR_STA1		(CSPM_BASE + 0x1e4)
#define CSPM_TWAM_CURR_STA2		(CSPM_BASE + 0x1e8)
#define CSPM_TWAM_CURR_STA3		(CSPM_BASE + 0x1ec)
#define CSPM_TWAM_CURR_STA4		(CSPM_BASE + 0x1f0)
#define CSPM_TWAM_CURR_STA5		(CSPM_BASE + 0x1f4)
#define CSPM_TWAM_CURR_STA6		(CSPM_BASE + 0x1f8)
#define CSPM_TWAM_CURR_STA7		(CSPM_BASE + 0x1fc)
#define CSPM_TWAM_CURR_STA8		(CSPM_BASE + 0x200)
#define CSPM_TWAM_CURR_STA9		(CSPM_BASE + 0x204)
#define CSPM_TWAM_CURR_STA10		(CSPM_BASE + 0x208)
#define CSPM_TWAM_CURR_STA11		(CSPM_BASE + 0x20c)
#define CSPM_TWAM_CURR_STA12		(CSPM_BASE + 0x210)
#define CSPM_TWAM_CURR_STA13		(CSPM_BASE + 0x214)
#define CSPM_TWAM_CURR_STA14		(CSPM_BASE + 0x218)
#define CSPM_TWAM_CURR_STA15		(CSPM_BASE + 0x21c)
#define CSPM_TWAM_TIMER_OUT		(CSPM_BASE + 0x220)
#define CSPM_M0_REC0			(CSPM_BASE + 0x300)
#define CSPM_M0_REC1			(CSPM_BASE + 0x304)
#define CSPM_M0_REC2			(CSPM_BASE + 0x308)
#define CSPM_M0_REC3			(CSPM_BASE + 0x30c)
#define CSPM_M0_REC4			(CSPM_BASE + 0x310)
#define CSPM_M0_REC5			(CSPM_BASE + 0x314)
#define CSPM_M0_REC6			(CSPM_BASE + 0x318)
#define CSPM_M0_REC7			(CSPM_BASE + 0x31c)
#define CSPM_M0_REC8			(CSPM_BASE + 0x320)
#define CSPM_M0_REC9			(CSPM_BASE + 0x324)
#define CSPM_M0_REC10			(CSPM_BASE + 0x328)
#define CSPM_M0_REC11			(CSPM_BASE + 0x32c)
#define CSPM_M0_REC12			(CSPM_BASE + 0x330)
#define CSPM_M0_REC13			(CSPM_BASE + 0x334)
#define CSPM_M0_REC14			(CSPM_BASE + 0x338)
#define CSPM_M0_REC15			(CSPM_BASE + 0x33c)
#define CSPM_M0_REC16			(CSPM_BASE + 0x340)
#define CSPM_M0_REC17			(CSPM_BASE + 0x344)
#define CSPM_M0_REC18			(CSPM_BASE + 0x348)
#define CSPM_M0_REC19			(CSPM_BASE + 0x34c)
#define CSPM_M1_REC0			(CSPM_BASE + 0x350)
#define CSPM_M1_REC1			(CSPM_BASE + 0x354)
#define CSPM_M1_REC2			(CSPM_BASE + 0x358)
#define CSPM_M1_REC3			(CSPM_BASE + 0x35c)
#define CSPM_M1_REC4			(CSPM_BASE + 0x360)
#define CSPM_M1_REC5			(CSPM_BASE + 0x364)
#define CSPM_M1_REC6			(CSPM_BASE + 0x368)
#define CSPM_M1_REC7			(CSPM_BASE + 0x36c)
#define CSPM_M1_REC8			(CSPM_BASE + 0x370)
#define CSPM_M1_REC9			(CSPM_BASE + 0x374)
#define CSPM_M1_REC10			(CSPM_BASE + 0x378)
#define CSPM_M1_REC11			(CSPM_BASE + 0x37c)
#define CSPM_M1_REC12			(CSPM_BASE + 0x380)
#define CSPM_M1_REC13			(CSPM_BASE + 0x384)
#define CSPM_M1_REC14			(CSPM_BASE + 0x388)
#define CSPM_M1_REC15			(CSPM_BASE + 0x38c)
#define CSPM_M1_REC16			(CSPM_BASE + 0x390)
#define CSPM_M1_REC17			(CSPM_BASE + 0x394)
#define CSPM_M1_REC18			(CSPM_BASE + 0x398)
#define CSPM_M1_REC19			(CSPM_BASE + 0x39c)
#define CSPM_M2_REC0			(CSPM_BASE + 0x3a0)
#define CSPM_M2_REC1			(CSPM_BASE + 0x3a4)
#define CSPM_M2_REC2			(CSPM_BASE + 0x3a8)
#define CSPM_M2_REC3			(CSPM_BASE + 0x3ac)
#define CSPM_M2_REC4			(CSPM_BASE + 0x3b0)
#define CSPM_M2_REC5			(CSPM_BASE + 0x3b4)
#define CSPM_M2_REC6			(CSPM_BASE + 0x3b8)
#define CSPM_M2_REC7			(CSPM_BASE + 0x3bc)
#define CSPM_M2_REC8			(CSPM_BASE + 0x3c0)
#define CSPM_M2_REC9			(CSPM_BASE + 0x3c4)
#define CSPM_M2_REC10			(CSPM_BASE + 0x3c8)
#define CSPM_M2_REC11			(CSPM_BASE + 0x3cc)
#define CSPM_M2_REC12			(CSPM_BASE + 0x3d0)
#define CSPM_M2_REC13			(CSPM_BASE + 0x3d4)
#define CSPM_M2_REC14			(CSPM_BASE + 0x3d8)
#define CSPM_M2_REC15			(CSPM_BASE + 0x3dc)
#define CSPM_M2_REC16			(CSPM_BASE + 0x3e0)
#define CSPM_M2_REC17			(CSPM_BASE + 0x3e4)
#define CSPM_M2_REC18			(CSPM_BASE + 0x3e8)
#define CSPM_M2_REC19			(CSPM_BASE + 0x3ec)
#define CSPM_DVFS_CON			(CSPM_BASE + 0x400)
#define CSPM_SEMA0_M0			(CSPM_BASE + 0x410)
#define CSPM_SEMA0_M1			(CSPM_BASE + 0x414)
#define CSPM_SEMA0_M2			(CSPM_BASE + 0x418)
#define CSPM_SEMA1_M0			(CSPM_BASE + 0x420)	/* EMI SEMA (LP SPM) */
#define CSPM_SEMA1_M1			(CSPM_BASE + 0x424)	/* EMI SEMA (CPU SPM) */
#define CSPM_SEMA1_M2			(CSPM_BASE + 0x428)	/* EMI SEMA (X) */
#define CSPM_SEMA2_M0			(CSPM_BASE + 0x430)	/* EMI SEMA (LP SPM) */
#define CSPM_SEMA2_M1			(CSPM_BASE + 0x434)	/* EMI SEMA (SCP) */
#define CSPM_SEMA2_M2			(CSPM_BASE + 0x438)	/* EMI SEMA (X) */
#define CSPM_SEMA3_M0			(CSPM_BASE + 0x440)	/* MCU SEMA (FH) */
#define CSPM_SEMA3_M1			(CSPM_BASE + 0x444)	/* MCU SEMA (CPU SPM) */
#define CSPM_SEMA3_M2			(CSPM_BASE + 0x448)	/* MCU SEMA (ATF) */
#define CSPM_MP0_CPU0_WFI_EN		(CSPM_BASE + 0x530)
#define CSPM_MP0_CPU1_WFI_EN		(CSPM_BASE + 0x534)
#define CSPM_MP0_CPU2_WFI_EN		(CSPM_BASE + 0x538)
#define CSPM_MP0_CPU3_WFI_EN		(CSPM_BASE + 0x53c)
#define CSPM_MP1_CPU0_WFI_EN		(CSPM_BASE + 0x540)
#define CSPM_MP1_CPU1_WFI_EN		(CSPM_BASE + 0x544)
#define CSPM_MP1_CPU2_WFI_EN		(CSPM_BASE + 0x548)
#define CSPM_MP1_CPU3_WFI_EN		(CSPM_BASE + 0x54c)
#define CSPM_SW_FLAG			(CSPM_BASE + 0x600)
#define CSPM_SW_DEBUG			(CSPM_BASE + 0x604)
#define CSPM_SW_RSV0			(CSPM_BASE + 0x608)
#define CSPM_SW_RSV1			(CSPM_BASE + 0x60c)
#define CSPM_SW_RSV2			(CSPM_BASE + 0x610)
#define CSPM_SW_RSV3			(CSPM_BASE + 0x614)
#define CSPM_SW_RSV4			(CSPM_BASE + 0x618)
#define CSPM_SW_RSV5			(CSPM_BASE + 0x61c)
#define CSPM_SW_RSV6			(CSPM_BASE + 0x620)
#define CSPM_SW_RSV7			(CSPM_BASE + 0x624)
#define CSPM_SW_RSV8			(CSPM_BASE + 0x628)
#define CSPM_SW_RSV9			(CSPM_BASE + 0x62c)
#define CSPM_SW_RSV10			(CSPM_BASE + 0x630)
#define CSPM_SW_RSV11			(CSPM_BASE + 0x634)
#define CSPM_SW_RSV12			(CSPM_BASE + 0x638)
#define CSPM_SW_RSV13			(CSPM_BASE + 0x63c)
#define CSPM_SW_RSV14			(CSPM_BASE + 0x640)
#define CSPM_SW_RSV15			(CSPM_BASE + 0x644)
#define CSPM_RSV_CON			(CSPM_BASE + 0x648)
#define CSPM_RSV_STA			(CSPM_BASE + 0x64c)
#define CSPM_M3_REC0			(CSPM_BASE + 0x800)
#define CSPM_M3_REC1			(CSPM_BASE + 0x804)
#define CSPM_M3_REC2			(CSPM_BASE + 0x808)
#define CSPM_M3_REC3			(CSPM_BASE + 0x80c)
#define CSPM_M3_REC4			(CSPM_BASE + 0x810)
#define CSPM_M3_REC5			(CSPM_BASE + 0x814)
#define CSPM_M3_REC6			(CSPM_BASE + 0x818)
#define CSPM_M3_REC7			(CSPM_BASE + 0x81c)
#define CSPM_M3_REC8			(CSPM_BASE + 0x820)
#define CSPM_M3_REC9			(CSPM_BASE + 0x824)
#define CSPM_M3_REC10			(CSPM_BASE + 0x828)
#define CSPM_M3_REC11			(CSPM_BASE + 0x82c)
#define CSPM_M3_REC12			(CSPM_BASE + 0x830)
#define CSPM_M3_REC13			(CSPM_BASE + 0x834)
#define CSPM_M3_REC14			(CSPM_BASE + 0x838)
#define CSPM_M3_REC15			(CSPM_BASE + 0x83c)
#define CSPM_M3_REC16			(CSPM_BASE + 0x840)
#define CSPM_M3_REC17			(CSPM_BASE + 0x844)
#define CSPM_M3_REC18			(CSPM_BASE + 0x848)
#define CSPM_M3_REC19			(CSPM_BASE + 0x84c)

#define POWER_ON_VAL1_DEF	0x20		/* PCM clock is 26M */

#define PCM_FSM_STA_DEF		0x48490

#define PROJECT_CODE		0xb16

#define REGWR_EN		(1U << 0)
#define REGWR_CFG_KEY		(PROJECT_CODE << 16)

#define CON0_PCM_KICK		(1U << 0)
#define CON0_IM_KICK		(1U << 1)
#define CON0_PCM_CK_EN		(1U << 2)
#define CON0_PCM_SW_RESET	(1U << 15)
#define CON0_CFG_KEY		(PROJECT_CODE << 16)

#define CON1_IM_SLAVE		(1U << 0)
#define CON1_MIF_APBEN		(1U << 3)
#define CON1_PCM_TIMER_EN	(1U << 5)
#define CON1_IM_NONRP_EN	(1U << 6)
#define CON1_PCM_WDT_EN		(1U << 8)
#define CON1_PCM_WDT_WAKE_MODE	(1U << 9)
#define CON1_SPM_SRAM_SLP_B	(1U << 10)
#define CON1_SPM_SRAM_ISO_B	(1U << 11)
#define CON1_EVENT_LOCK_EN	(1U << 12)
#define CON1_CFG_KEY		(PROJECT_CODE << 16)

#define PCM_PWRIO_EN_R7		(1U << 7)
#define PCM_RF_SYNC_R0		(1U << 16)
#define PCM_RF_SYNC_R1		(1U << 17)
#define PCM_RF_SYNC_R7		(1U << 23)

#define PCM_SW_INT0		(1U << 0)
#define PCM_SW_INT1		(1U << 1)
#define PCM_SW_INT2		(1U << 2)
#define PCM_SW_INT3		(1U << 3)
#define PCM_SW_INT_ALL		(PCM_SW_INT3 | PCM_SW_INT2 | PCM_SW_INT1 | PCM_SW_INT0)

#define TWAM_ENABLE		(1U << 0)
#define TWAM_SPEED_MODE_EN	(1U << 1)
#define TWAM_SW_RST		(1U << 2)

#define IRQM_TWAM		(1U << 2)
#define IRQM_PCM_RETURN		(1U << 3)
#define IRQM_RET_IRQ0		(1U << 8)
#define IRQM_RET_IRQ1		(1U << 9)
#define IRQM_RET_IRQ2		(1U << 10)
#define IRQM_RET_IRQ3		(1U << 11)

#define IRQM_RET_IRQ_SEC	(IRQM_RET_IRQ3 | IRQM_RET_IRQ2 | IRQM_RET_IRQ1)
#define IRQM_ALL_EXC_RET0	(IRQM_RET_IRQ_SEC | IRQM_TWAM)
#define IRQM_ALL		(IRQM_ALL_EXC_RET0 | IRQM_RET_IRQ0 | IRQM_PCM_RETURN)

#define WAKE_SRC_CPU		(1U << 0)
#define WAKE_SRC_TWAM		(1U << 1)

#define TMT_RISING		0
#define TMT_FALLING		1
#define TMT_HIGH_LEVEL		2
#define TMT_LOW_LEVEL		3

#define TWAM_MON_TYPE0(val)	(((val) & 0x7) << 0)
#define TWAM_MON_TYPE1(val)	(((val) & 0x7) << 3)
#define TWAM_MON_TYPE2(val)	(((val) & 0x7) << 6)
#define TWAM_MON_TYPE3(val)	(((val) & 0x7) << 9)
#define TWAM_SIG_SEL0(val)	(((val) & 0x1f) << 12)
#define TWAM_SIG_SEL1(val)	(((val) & 0x1f) << 17)
#define TWAM_SIG_SEL2(val)	(((val) & 0x1f) << 22)
#define TWAM_SIG_SEL3(val)	(((val) & 0x1f) << 27)

#define R7_MULT_DIV_SEL		(1U << 0)	/* 0: mult, 1: div */
#define R7_DIV_QRD_SEL		(1U << 1)	/* 0: div_q, 1: div_r */
#define R7_PCM_TIMER_SET	(1U << 9)

#define IRQS_TWAM		(1U << 2)
#define IRQS_PCM_RETURN		(1U << 3)
#define IRQS_SW_INT0		(1U << 4)
#define IRQS_SW_INT1		(1U << 5)
#define IRQS_SW_INT2		(1U << 6)
#define IRQS_SW_INT3		(1U << 7)

#define IRQC_TWAM		IRQS_TWAM
#define IRQC_PCM_RETURN		IRQS_PCM_RETURN
#define IRQC_ALL		(IRQC_PCM_RETURN | IRQC_TWAM)

#define FSM_PC_IDLE		(1U << 4)
#define FSM_PC_INCR		(1U << 5)
#define FSM_PC_STAL		(1U << 6)
#define FSM_IM_EMPT		(1U << 7)
#define FSM_IM_FILL		(1U << 8)
#define FSM_IM_REDY		(1U << 9)
#define FSM_PCM_KICK		(1U << 21)

#define SW_F_MIN(val)		(((val) & 0xf) << 0)
#define SW_F_MAX(val)		(((val) & 0xf) << 4)
#define SW_F_DES(val)		(((val) & 0xf) << 8)
#define SW_F_ASSIGN		(1U << 12)
#define SW_PAUSE		(1U << 13)
#define CLUSTER_EN		(1U << 14)
#define FIT_EN			(1U << 15)

#define SW_F_MIN_MASK		(0xf << 0)
#define SW_F_MAX_MASK		(0xf << 4)
#define SW_F_DES_MASK		(0xf << 8)

#define F_CURR(val)		(((val) & 0xf) << 0)
#define F_DES(val)		(((val) & 0xf) << 4)
#define V_CURR(val)		(((val) & 0x7f) << 8)
#define FW_DONE			(1U << 15)
#define VS_CURR(val)		(((val) & 0x7f) << 16)

#define F_CURR_MASK		(0xf << 0)
#define F_DES_MASK		(0xf << 4)
#define V_CURR_MASK		(0x7f << 8)
#define VS_CURR_MASK		(0x7f << 16)

#define LOG_V_MASK		(0x7f << 0)
#define LOG_F_MASK		(0xf << 7)
#define LOG_VS_MASK		(0x3f << 26)	/* only 6 bits */

/* pause source flag */
#define PSF_PAUSE_INIT		(1U << PAUSE_INIT)
#define PSF_PAUSE_I2CDRV	(1U << PAUSE_I2CDRV)
#define PSF_PAUSE_IDLE		(1U << PAUSE_IDLE)
#define PSF_PAUSE_SUSPEND	(1U << PAUSE_SUSPEND)
#define PSF_PAUSE_HWGOV		(1U << PAUSE_HWGOV)

/* debug log flag */
#define DLF_KICK		(1U << 0)
#define DLF_CLUSTER		(1U << 1)
#define DLF_DVFS		(1U << 2)
#define DLF_SEMA		(1U << 3)
#define DLF_PAUSE		(1U << 4)
#define DLF_STOP		(1U << 5)
#define DLF_LIMIT		(1U << 6)
#define DLF_NOTIFY		(1U << 7)
#define DLF_HWGOV		(1U << 8)

/* function enter flag */
#define FEF_DVFS		(1U << 0)
#define FEF_CLUSTER_OFF		(1U << 1)

struct pcm_desc;

struct pwr_ctrl {
	u8 twam_wfi_init;
	u8 r7_ctrl_en;

	u32 wake_src;
};


/**************************************
 * [Hybrid DVFS] Definition
 **************************************/
#define DBG_REPO_S		CSRAM_BASE
#define DBG_REPO_E		(DBG_REPO_S + CSRAM_SIZE)
#define DBG_REPO_DATA_S		(DBG_REPO_S + OFFS_DATA_S)
#define DBG_REPO_DATA_E		(DBG_REPO_S + OFFS_LOG_S)
#define DBG_REPO_LOG_S		(DBG_REPO_S + OFFS_LOG_S)
#define DBG_REPO_LOG_E		(DBG_REPO_S + OFFS_LOG_E)

#define DBG_REPO_SIZE		(DBG_REPO_E - DBG_REPO_S)
#define DBG_REPO_NUM		(DBG_REPO_SIZE / sizeof(u32))

#define REPO_I_DATA_S		(OFFS_DATA_S / sizeof(u32))
#define REPO_I_WDT_LATCH0	(OFFS_WDT_LATCH0 / sizeof(u32))
#define REPO_I_WDT_LATCH1	(OFFS_WDT_LATCH1 / sizeof(u32))
#define REPO_I_WDT_LATCH2	(OFFS_WDT_LATCH2 / sizeof(u32))
#define REPO_I_LOG_S		(OFFS_LOG_S / sizeof(u32))

#define REPO_GUARD0		0x55aa55aa
#define REPO_GUARD1		0xaa55aa55

struct cpuhvfs_dvfsp {
	struct pcm_desc *pcmdesc;
	struct pwr_ctrl *pwrctrl;

	void __iomem *log_repo;

	u32 init_done;		/* for dvfsp and log_repo */

	u32 hw_gov_en;

	int (*init_dvfsp)(struct cpuhvfs_dvfsp *dvfsp);
	int (*kick_dvfsp)(struct cpuhvfs_dvfsp *dvfsp, struct init_sta *sta);
	bool (*is_kicked)(struct cpuhvfs_dvfsp *dvfsp);

	void (*cluster_on)(struct cpuhvfs_dvfsp *dvfsp, unsigned int cluster);
	void (*cluster_off)(struct cpuhvfs_dvfsp *dvfsp, unsigned int cluster);

	int (*set_target)(struct cpuhvfs_dvfsp *dvfsp, unsigned int cluster, unsigned int index,
			  unsigned int *ret_volt);
	unsigned int (*get_volt)(struct cpuhvfs_dvfsp *dvfsp, unsigned int cluster);

	void (*set_limit)(struct cpuhvfs_dvfsp *dvfsp, unsigned int cluster, unsigned int ceiling,
			  unsigned int floor);

	void (*set_notify)(struct cpuhvfs_dvfsp *dvfsp, dvfs_notify_t callback);

	int (*enable_hwgov)(struct cpuhvfs_dvfsp *dvfsp, struct init_sta *sta);
	int (*disable_hwgov)(struct cpuhvfs_dvfsp *dvfsp, struct init_sta *ret_sta);

	int (*get_sema)(struct cpuhvfs_dvfsp *dvfsp, enum sema_user user);
	void (*release_sema)(struct cpuhvfs_dvfsp *dvfsp, enum sema_user user);

	int (*pause_dvfsp)(struct cpuhvfs_dvfsp *dvfsp, enum pause_src src);
	void (*unpause_dvfsp)(struct cpuhvfs_dvfsp *dvfsp, enum pause_src src);

	int (*stop_dvfsp)(struct cpuhvfs_dvfsp *dvfsp);

	void (*dump_info)(struct cpuhvfs_dvfsp *dvfsp, const char *fmt, ...);
};

struct cpuhvfs_data {
	struct cpuhvfs_dvfsp *dvfsp;

	u32 *dbg_repo;		/* => dvfsp->log_repo */
	struct dentry *dbg_fs;
};


#ifdef CONFIG_HYBRID_CPU_DVFS

/**************************************
 * [CPU SPM] Declaration
 **************************************/
#include "mt_cpufreq_hybrid_fw.h"

static struct pwr_ctrl dvfs_ctrl = {
	.twam_wfi_init	= 1,
	.r7_ctrl_en	= 1,

	.wake_src	= WAKE_SRC_TWAM | WAKE_SRC_CPU,
};

static void __iomem *cspm_base;
static void __iomem *csram_base;

static void __iomem *swctrl_reg[NUM_CPU_CLUSTER];
static void __iomem *hwsta_reg[NUM_CPU_CLUSTER];
static void __iomem *sema_reg[NUM_SEMA_USER];

static unsigned int swctrl_offs[NUM_CPU_CLUSTER];
static unsigned int hwsta_offs[NUM_CPU_CLUSTER];

static struct clk *i2c_clk;

static u32 cspm_probe_done;

static struct init_sta suspend_sta = {
	.opp		= { [0 ... (NUM_CPU_CLUSTER - 1)] = UINT_MAX },
	.volt		= { [0 ... (NUM_CPU_CLUSTER - 1)] = UINT_MAX },
	.vsram		= { [0 ... (NUM_CPU_CLUSTER - 1)] = UINT_MAX },
	.ceiling	= { [0 ... (NUM_CPU_CLUSTER - 1)] = UINT_MAX },
	.floor		= { [0 ... (NUM_CPU_CLUSTER - 1)] = UINT_MAX },
};

/* keep PAUSE_INIT set until cluster-on notification */
static u32 pause_src_map = PSF_PAUSE_INIT;

static u32 pause_log_en = /*PSF_PAUSE_HWGOV |*/
			  /*PSF_PAUSE_SUSPEND |*/
			  /*PSF_PAUSE_IDLE |*/
			  /*PSF_PAUSE_I2CDRV |*/
			  PSF_PAUSE_INIT;

/**
 * dbgx_log_en[15:0] : show on UART/MobileLog
 * dbgx_log_en[31:16]: show on MobileLog
 */
static u32 dbgx_log_en = /*(DLF_DVFS << 16) |*/
			 /*DLF_HWGOV |*/
			 /*DLF_NOTIFY |*/
			 /*DLF_LIMIT |*/
			 DLF_STOP |
			 DLF_PAUSE |
			 /*DLF_SEMA |*/
			 /*DLF_DVFS |*/
			 DLF_CLUSTER |
			 DLF_KICK;

#ifdef __TRIAL_RUN__
static u32 dvfs_fail_ke = 1;
static u32 sema_fail_ke = 1;
static u32 pause_fail_ke = 1;
#else
static u32 dvfs_fail_ke;
static u32 sema_fail_ke;
static u32 pause_fail_ke;
#endif

static u32 wa_get_emi_sema = 1;

static DEFINE_SPINLOCK(dvfs_lock);

static DECLARE_WAIT_QUEUE_HEAD(dvfs_wait);

#ifdef CPUHVFS_HW_GOVERNOR
static struct hrtimer nfy_trig_timer;

static struct task_struct *dvfs_nfy_task;
static atomic_t dvfs_nfy_req = ATOMIC_INIT(0);

/* log_box[MAX_LOG_FETCH] is also used to save last log entry */
static struct dvfs_log log_box[1 + MAX_LOG_FETCH];
static unsigned int next_log_offs = OFFS_LOG_S;

static dvfs_notify_t notify_dvfs_change_cb;
static DEFINE_MUTEX(notify_mutex);
#endif

static int cspm_module_init(struct cpuhvfs_dvfsp *dvfsp);
static int cspm_go_to_dvfs(struct cpuhvfs_dvfsp *dvfsp, struct init_sta *sta);
static bool cspm_is_pcm_kicked(struct cpuhvfs_dvfsp *dvfsp);

static void cspm_cluster_notify_on(struct cpuhvfs_dvfsp *dvfsp, unsigned int cluster);
static void cspm_cluster_notify_off(struct cpuhvfs_dvfsp *dvfsp, unsigned int cluster);

static int cspm_set_target_opp(struct cpuhvfs_dvfsp *dvfsp, unsigned int cluster, unsigned int index,
			       unsigned int *ret_volt);
static unsigned int cspm_get_curr_volt(struct cpuhvfs_dvfsp *dvfsp, unsigned int cluster);

static void cspm_set_opp_limit(struct cpuhvfs_dvfsp *dvfsp, unsigned int cluster,
			       unsigned int ceiling, unsigned int floor);

#ifdef CPUHVFS_HW_GOVERNOR
static void cspm_set_dvfs_notify(struct cpuhvfs_dvfsp *dvfsp, dvfs_notify_t callback);

static int cspm_enable_hw_governor(struct cpuhvfs_dvfsp *dvfsp, struct init_sta *sta);
static int cspm_disable_hw_governor(struct cpuhvfs_dvfsp *dvfsp, struct init_sta *ret_sta);
#endif

static int cspm_get_semaphore(struct cpuhvfs_dvfsp *dvfsp, enum sema_user user);
static void cspm_release_semaphore(struct cpuhvfs_dvfsp *dvfsp, enum sema_user user);

static int cspm_pause_pcm_running(struct cpuhvfs_dvfsp *dvfsp, enum pause_src src);
static void cspm_unpause_pcm_to_run(struct cpuhvfs_dvfsp *dvfsp, enum pause_src src);

static int cspm_stop_pcm_running(struct cpuhvfs_dvfsp *dvfsp);

static void cspm_dump_debug_info(struct cpuhvfs_dvfsp *dvfsp, const char *fmt, ...);


/**************************************
 * [Hybrid DVFS] Declaration
 **************************************/
static struct cpuhvfs_dvfsp g_dvfsp = {
	.pcmdesc	= &dvfs_pcm,
	.pwrctrl	= &dvfs_ctrl,

	.init_dvfsp	= cspm_module_init,
	.kick_dvfsp	= cspm_go_to_dvfs,
	.is_kicked	= cspm_is_pcm_kicked,

	.cluster_on	= cspm_cluster_notify_on,
	.cluster_off	= cspm_cluster_notify_off,

	.set_target	= cspm_set_target_opp,
	.get_volt	= cspm_get_curr_volt,

	.set_limit	= cspm_set_opp_limit,

#ifdef CPUHVFS_HW_GOVERNOR
	.set_notify	= cspm_set_dvfs_notify,

	.enable_hwgov	= cspm_enable_hw_governor,
	.disable_hwgov	= cspm_disable_hw_governor,
#endif

	.get_sema	= cspm_get_semaphore,
	.release_sema	= cspm_release_semaphore,

	.pause_dvfsp	= cspm_pause_pcm_running,
	.unpause_dvfsp	= cspm_unpause_pcm_to_run,

	.stop_dvfsp	= cspm_stop_pcm_running,

	.dump_info	= cspm_dump_debug_info,
};

static struct cpuhvfs_data g_cpuhvfs = {
	.dvfsp		= &g_dvfsp,
};

static u32 dbg_repo_bak[DBG_REPO_NUM];


/**************************************
 * [CPU SPM] Macro / Inline
 **************************************/
#define cspm_read(addr)			__raw_readl(addr)
#define cspm_write(addr, val)		mt_reg_sync_writel(val, addr)

#define csram_read(offs)		__raw_readl(csram_base + (offs))
#define csram_write(offs, val)		mt_reg_sync_writel(val, csram_base + (offs))

#define csram_write_fw_sta()					\
do {								\
	csram_write(OFFS_FW_RSV3, cspm_read(CSPM_SW_RSV3));	\
	csram_write(OFFS_FW_RSV4, cspm_read(CSPM_SW_RSV4));	\
	csram_write(OFFS_FW_RSV5, cspm_read(CSPM_SW_RSV5));	\
	csram_write(OFFS_FW_RSV6, cspm_read(CSPM_SW_RSV6));	\
	csram_write(OFFS_FW_RSV0, cspm_read(CSPM_SW_RSV0));	\
	csram_write(OFFS_FW_RSV1, cspm_read(CSPM_SW_RSV1));	\
	csram_write(OFFS_FW_RSV2, cspm_read(CSPM_SW_RSV2));	\
} while (0)

#define cspm_get_timestamp()		cspm_read(CSPM_PCM_TIMER_OUT)
#define cspm_get_pcmpc()		cspm_read(CSPM_PCM_REG15_DATA)

#define cspm_min_freq(reg)		((cspm_read(reg) & SW_F_MIN_MASK) >> 0)
#define cspm_max_freq(reg)		((cspm_read(reg) & SW_F_MAX_MASK) >> 4)
#define cspm_is_paused(reg)		(!!(cspm_read(reg) & SW_PAUSE))

#define cspm_curr_freq(reg)		((cspm_read(reg) & F_CURR_MASK) >> 0)
#define cspm_curr_volt(reg)		((cspm_read(reg) & V_CURR_MASK) >> 8)
#define cspm_curr_vsram(reg)		((cspm_read(reg) & VS_CURR_MASK) >> 16)
#define cspm_is_done(reg)		(!!(cspm_read(reg) & FW_DONE))

#define cspm_is_fw_all_done()		(cspm_is_done(CSPM_SW_RSV3) &&	\
					 cspm_is_done(CSPM_SW_RSV4) &&	\
					 cspm_is_done(CSPM_SW_RSV5))

#define log_time_curr_offs()		(cspm_read(CSPM_SW_RSV7) - 0xa000)

#define log_get_volt(log)		(((log) & LOG_V_MASK) >> 0)
#define log_get_freq(log)		(((log) & LOG_F_MASK) >> 7)

#define cspm_err(fmt, args...)		pr_err("[CPUHVFS] " fmt, ##args)
#define cspm_warn(fmt, args...)		pr_warn("[CPUHVFS] " fmt, ##args)
#define cspm_debug(fmt, args...)	pr_debug("[CPUHVFS] " fmt, ##args)

#define cspm_dbgx(flag, fmt, args...)			\
do {							\
	if (dbgx_log_en & DLF_##flag)			\
		cspm_err(fmt, ##args);			\
	else if (dbgx_log_en & (DLF_##flag << 16))	\
		cspm_debug(fmt, ##args);		\
	else						\
		;					\
} while (0)

#define wait_complete_us(condition, delay, timeout)		\
({								\
	int _i = 0;						\
	int _n = DIV_ROUND_UP(timeout, delay);			\
	csram_write(OFFS_PCM_PC1, cspm_get_pcmpc());		\
	csram_write(OFFS_TIMER_OUT1, cspm_get_timestamp());	\
	while (!(condition)) {					\
		if (_i >= _n) {					\
			_i = -_i;				\
			break;					\
		}						\
		udelay(delay);					\
		_i++;						\
	}							\
	csram_write(OFFS_TIMER_OUT2, cspm_get_timestamp());	\
	csram_write(OFFS_PCM_PC2, cspm_get_pcmpc());		\
	_i;							\
})

static inline u32 base_va_to_pa(const u32 *base)
{
	phys_addr_t pa = virt_to_phys(base);

	MAPPING_DRAM_ACCESS_ADDR(pa);	/* for 4GB mode */

	return (u32)pa;
}

static inline u32 opp_sw_to_fw(unsigned int index)
{
	return index < NUM_CPU_OPP ? (NUM_CPU_OPP - 1) - index : 0 /* lowest frequency */;
}

static inline unsigned int opp_fw_to_sw(u32 freq)
{
	return freq < NUM_CPU_OPP ? (NUM_CPU_OPP - 1) - freq : 0 /* highest frequency */;
}

#ifdef CPUHVFS_HW_GOVERNOR
static inline void start_notify_trigger_timer(u32 intv_ms)
{
	hrtimer_start(&nfy_trig_timer, ms_to_ktime(intv_ms), HRTIMER_MODE_REL);
}

static inline void stop_notify_trigger_timer(void)
{
	hrtimer_cancel(&nfy_trig_timer);
}

static inline void kick_kthread_to_notify(void)
{
	atomic_inc(&dvfs_nfy_req);
	wake_up_process(dvfs_nfy_task);
}
#endif


/**************************************
 * [Hybrid DVFS] Macro / Inline
 **************************************/
#define set_dvfsp_init_done(dvfsp)	((dvfsp)->init_done = 1)
#define is_dvfsp_uninit(dvfsp)		(!(dvfsp)->init_done)

#define cpuhvfs_crit(fmt, args...)	pr_err("[CPUHVFS] " fmt, ##args)
#define cpuhvfs_err(fmt, args...)	pr_err("[CPUHVFS] " fmt, ##args)
#define cpuhvfs_warn(fmt, args...)	pr_warn("[CPUHVFS] " fmt, ##args)
#define cpuhvfs_debug(fmt, args...)	pr_debug("[CPUHVFS] " fmt, ##args)

#define DEFINE_FOPS_RO(fname)						\
static int fname##_open(struct inode *inode, struct file *file)		\
{									\
	return single_open(file, fname##_show, inode->i_private);	\
}									\
static const struct file_operations fname##_fops = {			\
	.open		= fname##_open,					\
	.read		= seq_read,					\
	.llseek		= seq_lseek,					\
	.release	= single_release,				\
}

#define DEFINE_FOPS_RW(fname)						\
static int fname##_open(struct inode *inode, struct file *file)		\
{									\
	return single_open(file, fname##_show, inode->i_private);	\
}									\
static const struct file_operations fname##_fops = {			\
	.open		= fname##_open,					\
	.read		= seq_read,					\
	.llseek		= seq_lseek,					\
	.write		= fname##_write,				\
	.release	= single_release,				\
}


/**************************************
 * [CPU SPM] Function
 **************************************/
static void __cspm_pcm_sw_reset(void)
{
	u32 sta;

	cspm_write(CSPM_PCM_CON0, CON0_CFG_KEY | CON0_PCM_SW_RESET);
	cspm_write(CSPM_PCM_CON0, CON0_CFG_KEY);

	udelay(10);

	sta = cspm_read(CSPM_PCM_FSM_STA);
	if (sta != PCM_FSM_STA_DEF) {
		cspm_err("FAILED TO RESET PCM (0x%x)\n", sta);
		BUG();
	}
}

static void __cspm_register_init(void)
{
	/* enable register write */
	cspm_write(CSPM_POWERON_CONFIG_EN, REGWR_CFG_KEY | REGWR_EN);

	/* init power control register */
	cspm_write(CSPM_POWER_ON_VAL1, POWER_ON_VAL1_DEF | R7_PCM_TIMER_SET);
	cspm_write(CSPM_PCM_PWR_IO_EN, 0);

	/* reset PCM */
	__cspm_pcm_sw_reset();

	/* init PCM control register */
	cspm_write(CSPM_PCM_CON0, CON0_CFG_KEY | CON0_PCM_CK_EN);
	cspm_write(CSPM_PCM_CON1, CON1_CFG_KEY | CON1_EVENT_LOCK_EN |
				  CON1_SPM_SRAM_ISO_B | CON1_SPM_SRAM_SLP_B |
				  CON1_MIF_APBEN);
	cspm_write(CSPM_PCM_IM_PTR, 0);
	cspm_write(CSPM_PCM_IM_LEN, 0);

	/* clean wakeup event raw status */
	cspm_write(CSPM_WAKEUP_EVENT_MASK, ~0);

	/* clean IRQ status */
	cspm_write(CSPM_IRQ_MASK, IRQM_ALL);
	cspm_write(CSPM_IRQ_STA, IRQC_ALL);
	cspm_write(CSPM_SW_INT_CLEAR, PCM_SW_INT_ALL);
}

static void __cspm_reset_and_init_pcm(const struct pcm_desc *pcmdesc)
{
	/* do basic init because of Infra power-on reset */
#if 0
	/* others will set REGWR_EN for SEMA1/2/3 access */
	if (!(cspm_read(CSPM_POWERON_CONFIG_EN) & REGWR_EN))
#else
	if (!(cspm_read(CSPM_PCM_CON1) & CON1_MIF_APBEN))
#endif
		__cspm_register_init();

	/* reset PCM */
	__cspm_pcm_sw_reset();

	/* init PCM_CON0 */
	cspm_write(CSPM_PCM_CON0, CON0_CFG_KEY | CON0_PCM_CK_EN);

	/* init PCM_CON1 (disable PCM timer and PCM WDT) */
	cspm_write(CSPM_PCM_CON1, CON1_CFG_KEY | CON1_EVENT_LOCK_EN |
				  CON1_SPM_SRAM_ISO_B | CON1_SPM_SRAM_SLP_B |
				  (pcmdesc->replace ? 0 : CON1_IM_NONRP_EN) |
				  CON1_MIF_APBEN);
}

static void __cspm_kick_im_to_fetch(const struct pcm_desc *pcmdesc)
{
	u32 ptr, len, con0;
	bool emi_access = true;

	/* tell IM where is PCM code (use slave mode if code existed) */
	ptr = base_va_to_pa(pcmdesc->base);
	len = pcmdesc->size - 1;
	if (cspm_read(CSPM_PCM_IM_PTR) != ptr || cspm_read(CSPM_PCM_IM_LEN) != len ||
	    pcmdesc->sess > 2) {
		cspm_write(CSPM_PCM_IM_PTR, ptr);
		cspm_write(CSPM_PCM_IM_LEN, len);
	} else {
		cspm_write(CSPM_PCM_CON1, cspm_read(CSPM_PCM_CON1) | CON1_CFG_KEY | CON1_IM_SLAVE);
		emi_access = false;
	}

	if (wa_get_emi_sema && emi_access) {
		int i, n = DIV_ROUND_UP(100000, 10) /* 100ms */;

		for (i = 0; i < n; i++) {
			cspm_write(CSPM_SEMA1_M1, 0x1);
			if (cspm_read(CSPM_SEMA1_M1) & 0x1)	/* get EMI SEMA */
				break;
			udelay(10);
		}
		if (i >= n) {
			cspm_err("SEMA1 0x(%x %x %x) GET TIMEOUT\n",
					   cspm_read(CSPM_SEMA1_M0),
					   cspm_read(CSPM_SEMA1_M1),
					   cspm_read(CSPM_SEMA1_M2));
			BUG();
		}
	}

	/* kick IM to fetch (only toggle IM_KICK) */
	con0 = cspm_read(CSPM_PCM_CON0) & ~(CON0_IM_KICK | CON0_PCM_KICK);
	cspm_write(CSPM_PCM_CON0, con0 | CON0_CFG_KEY | CON0_IM_KICK);
	cspm_write(CSPM_PCM_CON0, con0 | CON0_CFG_KEY);

	if (wa_get_emi_sema && emi_access) {
		while (!(cspm_read(CSPM_PCM_FSM_STA) & FSM_IM_REDY))	/* wait for IM fetch done */
			;
		cspm_write(CSPM_SEMA1_M1, 0x1);		/* release EMI SEMA */
	}
}

static void __cspm_init_pcm_register(void)
{
	/* init r7 with POWER_ON_VAL1 */
	cspm_write(CSPM_PCM_REG_DATA_INI, cspm_read(CSPM_POWER_ON_VAL1));
	cspm_write(CSPM_PCM_PWR_IO_EN, PCM_RF_SYNC_R7);
	cspm_write(CSPM_PCM_PWR_IO_EN, 0);
}

static void __cspm_init_event_vector(const struct pcm_desc *pcmdesc)
{
	/* init event vector register */
	cspm_write(CSPM_PCM_EVENT_VECTOR0, pcmdesc->vec0);
#if 0
	/* used for FiT parameter */
	cspm_write(CSPM_PCM_EVENT_VECTOR1, pcmdesc->vec1);
#endif

	/* disable event vector (enabled by FW) */
	cspm_write(CSPM_PCM_EVENT_VECTOR_EN, 0);
}

static void __cspm_set_twam_event(const struct pwr_ctrl *pwrctrl)
{
	if (!pwrctrl->twam_wfi_init)
		return;

	/* reset TWAM */
	cspm_write(CSPM_TWAM_CON, TWAM_SW_RST);
	cspm_write(CSPM_TWAM_CON, 0);
	udelay(10);
	BUG_ON(cspm_read(CSPM_TWAM_TIMER_OUT) != 0);

	/* use TWAM to monitor WFI signals (enabled by FW) */
	cspm_write(CSPM_TWAM_IDLE_SEL, 0);
	cspm_write(CSPM_TWAM_CON3, TWAM_SIG_SEL3(15) | TWAM_MON_TYPE3(TMT_HIGH_LEVEL) |
				   TWAM_SIG_SEL2(14) | TWAM_MON_TYPE2(TMT_HIGH_LEVEL) |
				   TWAM_SIG_SEL1(13) | TWAM_MON_TYPE1(TMT_HIGH_LEVEL) |
				   TWAM_SIG_SEL0(12) | TWAM_MON_TYPE0(TMT_HIGH_LEVEL));
	cspm_write(CSPM_TWAM_CON2, TWAM_SIG_SEL3(11) | TWAM_MON_TYPE3(TMT_HIGH_LEVEL) |
				   TWAM_SIG_SEL2(10) | TWAM_MON_TYPE2(TMT_HIGH_LEVEL) |
				   TWAM_SIG_SEL1(9)  | TWAM_MON_TYPE1(TMT_HIGH_LEVEL) |
				   TWAM_SIG_SEL0(8)  | TWAM_MON_TYPE0(TMT_HIGH_LEVEL));
	cspm_write(CSPM_TWAM_CON1, TWAM_SIG_SEL3(7)  | TWAM_MON_TYPE3(TMT_HIGH_LEVEL) |
				   TWAM_SIG_SEL2(6)  | TWAM_MON_TYPE2(TMT_HIGH_LEVEL) |
				   TWAM_SIG_SEL1(5)  | TWAM_MON_TYPE1(TMT_HIGH_LEVEL) |
				   TWAM_SIG_SEL0(4)  | TWAM_MON_TYPE0(TMT_HIGH_LEVEL));
	cspm_write(CSPM_TWAM_CON0, TWAM_SIG_SEL3(3)  | TWAM_MON_TYPE3(TMT_HIGH_LEVEL) |
				   TWAM_SIG_SEL2(2)  | TWAM_MON_TYPE2(TMT_HIGH_LEVEL) |
				   TWAM_SIG_SEL1(1)  | TWAM_MON_TYPE1(TMT_HIGH_LEVEL) |
				   TWAM_SIG_SEL0(0)  | TWAM_MON_TYPE0(TMT_HIGH_LEVEL));
	cspm_write(CSPM_TWAM_CON, TWAM_SPEED_MODE_EN);
	cspm_write(CSPM_TWAM_WINDOW_LEN, 0x733f);	/* 1.135ms */
}

static void __cspm_set_wakeup_event(const struct pwr_ctrl *pwrctrl)
{
	/* start PCM timer (as free-run timer) */
	cspm_write(CSPM_PCM_TIMER_VAL, 0xffffffff);
	cspm_write(CSPM_PCM_CON1, cspm_read(CSPM_PCM_CON1) | CON1_CFG_KEY | CON1_PCM_TIMER_EN);

	/* unmask PCM wakeup source */
	cspm_write(CSPM_WAKEUP_EVENT_MASK, ~pwrctrl->wake_src);

	/* unmask CSPM IRQ */
	cspm_write(CSPM_IRQ_MASK, IRQM_ALL_EXC_RET0);
}

static void __cspm_kick_pcm_to_run(const struct pwr_ctrl *pwrctrl, const struct init_sta *sta)
{
	int i;
	u32 con0;

	/* init register to match FW expectation */
	cspm_write(CSPM_M0_REC13, 0x1001af34);
	cspm_write(CSPM_M0_REC14, 0x1001af38);
	cspm_write(CSPM_M0_REC15, 0x1001af3c);
	cspm_write(CSPM_M0_REC16, 0x1001af40);
	cspm_write(CSPM_M0_REC17, 0x1001af44);
	cspm_write(CSPM_M0_REC18, 0x1001a204);
	cspm_write(CSPM_M0_REC19, 0x1);
	cspm_write(CSPM_M1_REC0,  0x200);
	cspm_write(CSPM_M1_REC1,  0xffffc1ff);
	cspm_write(CSPM_M1_REC2,  0x4);
	cspm_write(CSPM_M1_REC3,  0x8);
	cspm_write(CSPM_M1_REC4,  0xfffffff3);

	cspm_write(CSPM_M1_REC5,  0x1001af48);
	cspm_write(CSPM_M1_REC6,  0x1001af4c);
	cspm_write(CSPM_M1_REC7,  0x1001af50);
	cspm_write(CSPM_M1_REC8,  0x1001af54);
	cspm_write(CSPM_M1_REC9,  0x1001af58);
	cspm_write(CSPM_M1_REC10, 0x1001a214);
	cspm_write(CSPM_M1_REC11, 0x2);
	cspm_write(CSPM_M1_REC12, 0x20000);
	cspm_write(CSPM_M1_REC13, 0xffc1ffff);
	cspm_write(CSPM_M1_REC14, 0x10);
	cspm_write(CSPM_M1_REC15, 0x20);
	cspm_write(CSPM_M1_REC16, 0xffffffcf);

	cspm_write(CSPM_M1_REC17, 0x1001af5c);
	cspm_write(CSPM_M1_REC18, 0x1001af60);
	cspm_write(CSPM_M1_REC19, 0x1001af64);
	cspm_write(CSPM_M2_REC0,  0x1001af68);
	cspm_write(CSPM_M2_REC1,  0x1001af6c);
	cspm_write(CSPM_M2_REC2,  0x1001a224);
	cspm_write(CSPM_M2_REC3,  0x4);
	cspm_write(CSPM_M2_REC4,  0x2);
	cspm_write(CSPM_M2_REC5,  0xffffffc1);
	cspm_write(CSPM_M2_REC6,  0x40);
	cspm_write(CSPM_M2_REC7,  0x80);
	cspm_write(CSPM_M2_REC8,  0xffffff3f);

	cspm_write(CSPM_SW_RSV7,  0xa000 + OFFS_LOG_S + 0x0);
	cspm_write(CSPM_SW_RSV8,  0xa000 + OFFS_LOG_S + 0x4);
	cspm_write(CSPM_SW_RSV9,  0xa000 + OFFS_LOG_S + 0x8);
	cspm_write(CSPM_SW_RSV10, 0xa000 + OFFS_LOG_S + 0xc);
	cspm_write(CSPM_SW_RSV11, 0xa000 + OFFS_LOG_S + 0x10);
	cspm_write(CSPM_SW_RSV12, 0xa000 + OFFS_LOG_S + 0x14);

	cspm_write(CSPM_RSV_CON,  0x0);

	cspm_write(CSPM_PCM_EVENT_VECTOR1,  0x3e60000);
	cspm_write(CSPM_PCM_EVENT_VECTOR6,  0x3e60000);
	cspm_write(CSPM_PCM_EVENT_VECTOR11, 0x3e60000);

	cspm_write(CSPM_PCM_EVENT_VECTOR4,  0x0);
	cspm_write(CSPM_PCM_EVENT_VECTOR9,  0x0);
	cspm_write(CSPM_PCM_EVENT_VECTOR14, 0x0);

	for (i = 0; i < NUM_PHY_CLUSTER; i++) {
		cspm_write(swctrl_reg[i], SW_F_MAX(opp_sw_to_fw(sta->ceiling[i])) |
					  SW_F_MIN(opp_sw_to_fw(sta->floor[i])) |
					  SW_F_DES(opp_sw_to_fw(sta->opp[i])) |
					  SW_F_ASSIGN |
					  SW_PAUSE);
		csram_write(swctrl_offs[i], cspm_read(swctrl_reg[i]));
	}

	for (i = 0; i < NUM_CPU_CLUSTER; i++) {
		cspm_write(hwsta_reg[i], F_CURR(opp_sw_to_fw(sta->opp[i])) |
					 V_CURR(sta->volt[i]) |
					 VS_CURR(sta->vsram[i]));
		csram_write(hwsta_offs[i], cspm_read(hwsta_reg[i]));
	}

	/* init variable to match FW state */
	pause_src_map |= PSF_PAUSE_INIT;
	csram_write(OFFS_PAUSE_SRC, pause_src_map);

#ifdef CPUHVFS_HW_GOVERNOR
	log_box[MAX_LOG_FETCH].time = 0;
	next_log_offs = OFFS_LOG_S;
	csram_write(OFFS_NXLOG_OFFS, next_log_offs);
#endif

	/* enable r7 to control power */
	cspm_write(CSPM_PCM_PWR_IO_EN, pwrctrl->r7_ctrl_en ? PCM_PWRIO_EN_R7 : 0);

	/*cspm_dbgx(KICK, "kick PCM to run\n");*/

	/* kick PCM to run (only toggle PCM_KICK) */
	con0 = cspm_read(CSPM_PCM_CON0) & ~(CON0_IM_KICK | CON0_PCM_KICK);
	cspm_write(CSPM_PCM_CON0, con0 | CON0_CFG_KEY | CON0_PCM_KICK);
	cspm_write(CSPM_PCM_CON0, con0 | CON0_CFG_KEY);
}

static void __cspm_save_curr_sta(struct init_sta *sta)
{
	int i;

	csram_write_fw_sta();

	for (i = 0; i < NUM_CPU_CLUSTER; i++) {
		sta->opp[i] = opp_fw_to_sw(cspm_curr_freq(hwsta_reg[i]));
		sta->volt[i] = cspm_curr_volt(hwsta_reg[i]);
		sta->vsram[i] = cspm_curr_vsram(hwsta_reg[i]);

		if (i < NUM_PHY_CLUSTER) {
			sta->ceiling[i] = opp_fw_to_sw(cspm_max_freq(swctrl_reg[i]));
			sta->floor[i] = opp_fw_to_sw(cspm_min_freq(swctrl_reg[i]));
		} else {
			sta->ceiling[i] = 0;
			sta->floor[i] = NUM_CPU_OPP - 1;
		}
	}
}

static void __cspm_clean_after_pause(void)
{
	/* disable TWAM timer */
	cspm_write(CSPM_TWAM_CON, cspm_read(CSPM_TWAM_CON) & ~TWAM_ENABLE);

	/* disable r7 to control power */
	cspm_write(CSPM_PCM_PWR_IO_EN, 0);

	/* clean PCM timer event */
	cspm_write(CSPM_PCM_CON1, CON1_CFG_KEY | (cspm_read(CSPM_PCM_CON1) & ~CON1_PCM_TIMER_EN));

	/* clean wakeup event raw status */
	cspm_write(CSPM_WAKEUP_EVENT_MASK, ~0);

	/* clean IRQ status */
	cspm_write(CSPM_IRQ_MASK, IRQM_ALL);
	cspm_write(CSPM_IRQ_STA, IRQC_ALL);
	cspm_write(CSPM_SW_INT_CLEAR, PCM_SW_INT_ALL);
}

static void cspm_dump_debug_info(struct cpuhvfs_dvfsp *dvfsp, const char *fmt, ...)
{
	u32 timer, reg15, sema3_m0, sema3_m1, sema3_m2, sema1_m0, sema1_m1, sema1_m2;
	u32 rsv0, rsv1, rsv2, rsv3, rsv4, rsv5, rsv6;
	char msg[320];
	va_list args;

	timer = cspm_read(CSPM_PCM_TIMER_OUT);
	reg15 = cspm_read(CSPM_PCM_REG15_DATA);
	sema3_m0 = cspm_read(CSPM_SEMA3_M0);
	sema3_m1 = cspm_read(CSPM_SEMA3_M1);
	sema3_m2 = cspm_read(CSPM_SEMA3_M2);
	sema1_m0 = cspm_read(CSPM_SEMA1_M0);
	sema1_m1 = cspm_read(CSPM_SEMA1_M1);
	sema1_m2 = cspm_read(CSPM_SEMA1_M2);

	rsv3 = cspm_read(CSPM_SW_RSV3);
	rsv4 = cspm_read(CSPM_SW_RSV4);
	rsv5 = cspm_read(CSPM_SW_RSV5);
	rsv6 = cspm_read(CSPM_SW_RSV6);
	rsv0 = cspm_read(CSPM_SW_RSV0);
	rsv1 = cspm_read(CSPM_SW_RSV1);
	rsv2 = cspm_read(CSPM_SW_RSV2);

	va_start(args, fmt);
	vsnprintf(msg, sizeof(msg), fmt, args);
	va_end(args);

	cspm_err("(%u) %s\n"   , dvfsp->hw_gov_en, msg);
	cspm_err("FW_VER: %s\n", dvfsp->pcmdesc->version);

	cspm_err("PCM_TIMER: %08x\n", timer);
	cspm_err("PCM_REG15: %u, SEMA3: 0x(%x %x %x), SEMA1: 0x(%x %x %x)\n",
					   reg15, sema3_m0, sema3_m1, sema3_m2,
					   sema1_m0, sema1_m1, sema1_m2);

	cspm_err("SW_RSV0: 0x%x\n", rsv0);
	cspm_err("SW_RSV1: 0x%x\n", rsv1);
	cspm_err("SW_RSV2: 0x%x\n", rsv2);
	cspm_err("SW_RSV3: 0x%x\n", rsv3);
	cspm_err("SW_RSV4: 0x%x\n", rsv4);
	cspm_err("SW_RSV5: 0x%x\n", rsv5);
	cspm_err("SW_RSV6: 0x%x\n", rsv6);

	cspm_err("M3_RECx: 0x(%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x)\n",
			      cspm_read(CSPM_M3_REC1),
			      cspm_read(CSPM_M3_REC2),
			      cspm_read(CSPM_M3_REC3),
			      cspm_read(CSPM_M3_REC4),
			      cspm_read(CSPM_M3_REC5),
			      cspm_read(CSPM_M3_REC6),
			      cspm_read(CSPM_M3_REC7),
			      cspm_read(CSPM_M3_REC8),
			      cspm_read(CSPM_M3_REC9),
			      cspm_read(CSPM_M3_REC10),
			      cspm_read(CSPM_M3_REC11),
			      cspm_read(CSPM_M3_REC12),
			      cspm_read(CSPM_M3_REC13),
			      cspm_read(CSPM_M3_REC14),
			      cspm_read(CSPM_M3_REC15),
			      cspm_read(CSPM_M3_REC16),
			      cspm_read(CSPM_M3_REC17),
			      cspm_read(CSPM_M3_REC18),
			      cspm_read(CSPM_M3_REC19));
	cspm_err("SW_DEBUG: 0x%x\n", cspm_read(CSPM_SW_DEBUG));

	cspm_err("EVT_VECx: 0x(%x,%x,%x,%x,%x,%x,%x,%x,%x)\n",
			       cspm_read(CSPM_PCM_EVENT_VECTOR2),
			       cspm_read(CSPM_PCM_EVENT_VECTOR3),
			       cspm_read(CSPM_PCM_EVENT_VECTOR5),
			       cspm_read(CSPM_PCM_EVENT_VECTOR7),
			       cspm_read(CSPM_PCM_EVENT_VECTOR8),
			       cspm_read(CSPM_PCM_EVENT_VECTOR10),
			       cspm_read(CSPM_PCM_EVENT_VECTOR12),
			       cspm_read(CSPM_PCM_EVENT_VECTOR13),
			       cspm_read(CSPM_PCM_EVENT_VECTOR15));

	cspm_err("PCM_FSM_STA: 0x%x\n", cspm_read(CSPM_PCM_FSM_STA));
}

static int __cspm_pause_pcm_running(struct cpuhvfs_dvfsp *dvfsp, u32 psf)
{
	int i, r = 0;

	/* @dvfsp may be uninitialized (PAUSE_INIT is set) */

	if (pause_src_map == 0) {
		for (i = 0; i < NUM_PHY_CLUSTER; i++) {
			cspm_write(swctrl_reg[i], cspm_read(swctrl_reg[i]) | SW_PAUSE);
			csram_write(swctrl_offs[i], cspm_read(swctrl_reg[i]));
		}
		csram_write(OFFS_PAUSE_TIME, cspm_get_timestamp());

		udelay(10);	/* skip FW_DONE 1->0 transition */

		r = wait_complete_us(cspm_is_fw_all_done(), 10, PAUSE_TIMEOUT);
		csram_write_fw_sta();

		if (r >= 0) {	/* pause done */
			r = 0;
			clk_disable(i2c_clk);

#ifdef CPUHVFS_HW_GOVERNOR
			if (dvfsp->hw_gov_en)
				stop_notify_trigger_timer();
#endif
		}
	}

	if (!r) {
		pause_src_map |= psf;
		if (csram_base)
			csram_write(OFFS_PAUSE_SRC, pause_src_map);

		if (cspm_base && psf == PSF_PAUSE_SUSPEND)
			__cspm_save_curr_sta(&suspend_sta);
	} else {
		cspm_dump_debug_info(dvfsp, "PAUSE TIMEOUT, psf = 0x%x", psf);
		BUG_ON(pause_fail_ke);
	}

	return r;
}

static void __cspm_unpause_pcm_to_run(struct cpuhvfs_dvfsp *dvfsp, u32 psf)
{
	int i, r, all_pause = 1;
	u32 swctrl;

	/* @dvfsp may be uninitialized (PAUSE_INIT is set) */

	pause_src_map &= ~psf;
	if (csram_base)
		csram_write(OFFS_PAUSE_SRC, pause_src_map);

	if (pause_src_map == 0 && csram_base /* avoid Coverity complaining */) {
		r = clk_enable(i2c_clk);
		if (!r) {
			for (i = 0; i < NUM_PHY_CLUSTER; i++) {
				swctrl = cspm_read(swctrl_reg[i]);
				csram_write(swctrl_offs[i], swctrl);

				if (!(swctrl & CLUSTER_EN))
					continue;

#ifdef CPUHVFS_HW_GOVERNOR
				if (dvfsp->hw_gov_en && psf == PSF_PAUSE_SUSPEND)
					swctrl &= ~SW_F_ASSIGN;		/* set in PCM re-kick */
#endif
				cspm_write(swctrl_reg[i], swctrl & ~SW_PAUSE);
				csram_write(swctrl_offs[i], cspm_read(swctrl_reg[i]));
				all_pause = 0;
			}
			csram_write(OFFS_UNPAUSE_TIME, cspm_get_timestamp());

			BUG_ON(all_pause);	/* no cluster on!? */

			wake_up(&dvfs_wait);	/* for set_target_opp */
			csram_write(OFFS_DVFS_WAIT, 0);

#ifdef CPUHVFS_HW_GOVERNOR
			if (dvfsp->hw_gov_en)
				start_notify_trigger_timer(DVFS_NOTIFY_INTV);
#endif
		} else {
			cspm_err("FAILED TO ENABLE I2C CLOCK (%d)\n", r);
			BUG();
		}
	}
}

static int cspm_stop_pcm_running(struct cpuhvfs_dvfsp *dvfsp)
{
	int r;

	spin_lock(&dvfs_lock);
	cspm_dbgx(STOP, "(%u) [%08x] stop pcm\n", dvfsp->hw_gov_en, cspm_get_timestamp());

	r = __cspm_pause_pcm_running(dvfsp, PSF_PAUSE_INIT);
	if (!r) {
		__cspm_clean_after_pause();

		__cspm_pcm_sw_reset();

		dvfsp->hw_gov_en = 0;
		csram_write(OFFS_HWGOV_EN, dvfsp->hw_gov_en);
	}
	spin_unlock(&dvfs_lock);

	return r;
}

static int cspm_pause_pcm_running(struct cpuhvfs_dvfsp *dvfsp, enum pause_src src)
{
	int r;
	u32 psf = (1U << src);

	/* @dvfsp may be uninitialized (PAUSE_INIT is set) */

	spin_lock(&dvfs_lock);
	if (pause_log_en & psf)
		cspm_dbgx(PAUSE, "pause pcm, src = %u, map = 0x%x\n", src, pause_src_map);

	r = __cspm_pause_pcm_running(dvfsp, psf);
	spin_unlock(&dvfs_lock);

	return r;
}

static void cspm_unpause_pcm_to_run(struct cpuhvfs_dvfsp *dvfsp, enum pause_src src)
{
	u32 psf = (1U << src);

	/* @dvfsp may be uninitialized (PAUSE_INIT is set) */

	spin_lock(&dvfs_lock);
	if (pause_log_en & psf)
		cspm_dbgx(PAUSE, "unpause pcm, src = %u, map = 0x%x\n", src, pause_src_map);

	if (psf & pause_src_map)
		__cspm_unpause_pcm_to_run(dvfsp, psf);
	spin_unlock(&dvfs_lock);
}

static int cspm_get_semaphore(struct cpuhvfs_dvfsp *dvfsp, enum sema_user user)
{
	int i;
	int n = DIV_ROUND_UP(SEMA_GET_TIMEOUT, 10);

	if (user == SEMA_I2C_DRV)
		return cspm_pause_pcm_running(dvfsp, PAUSE_I2CDRV);

	if (is_dvfsp_uninit(dvfsp))
		return 0;

	cspm_dbgx(SEMA, "sema get, user = %u\n", user);

	cspm_write(CSPM_POWERON_CONFIG_EN, REGWR_CFG_KEY | REGWR_EN);	/* enable register write */

	for (i = 0; i < n; i++) {
		cspm_write(sema_reg[user], 0x1);
		if (cspm_read(sema_reg[user]) & 0x1)
			return 0;

		udelay(10);
	}

	cspm_dump_debug_info(dvfsp, "SEMA_USER%u GET TIMEOUT", user);
	BUG_ON(sema_fail_ke);

	return -EBUSY;
}

static void cspm_release_semaphore(struct cpuhvfs_dvfsp *dvfsp, enum sema_user user)
{
	if (user == SEMA_I2C_DRV) {
		cspm_unpause_pcm_to_run(dvfsp, PAUSE_I2CDRV);
		return;
	}

	if (is_dvfsp_uninit(dvfsp))
		return;

	cspm_dbgx(SEMA, "sema release, user = %u\n", user);

	cspm_write(CSPM_POWERON_CONFIG_EN, REGWR_CFG_KEY | REGWR_EN);	/* enable register write */

	if (cspm_read(sema_reg[user]) & 0x1) {
		cspm_write(sema_reg[user], 0x1);
		BUG_ON(cspm_read(sema_reg[user]) & 0x1);	/* semaphore release failed */
	}
}

#ifdef CPUHVFS_HW_GOVERNOR
static int cspm_enable_hw_governor(struct cpuhvfs_dvfsp *dvfsp, struct init_sta *sta)
{
	int i;
	u32 f_max, f_min, swctrl;

	spin_lock(&dvfs_lock);
	cspm_dbgx(HWGOV, "(%u) [%08x] enable hw gov\n", dvfsp->hw_gov_en, cspm_get_timestamp());

	if (dvfsp->hw_gov_en)
		goto UNLOCK;

	for (i = 0; i < NUM_PHY_CLUSTER; i++) {
		f_max = opp_sw_to_fw(sta->ceiling[i]);
		f_min = opp_sw_to_fw(sta->floor[i]);

		swctrl = cspm_read(swctrl_reg[i]);
		swctrl &= ~(SW_F_ASSIGN | SW_F_MAX_MASK | SW_F_MIN_MASK);
		cspm_write(swctrl_reg[i], swctrl | SW_F_MAX(f_max) | SW_F_MIN(f_min));
		csram_write(swctrl_offs[i], cspm_read(swctrl_reg[i]));
	}
	csram_write(OFFS_HWGOV_STIME, cspm_get_timestamp());

	dvfsp->hw_gov_en = 1;
	csram_write(OFFS_HWGOV_EN, dvfsp->hw_gov_en);

	start_notify_trigger_timer(DVFS_NOTIFY_INTV);

UNLOCK:
	spin_unlock(&dvfs_lock);

	return 0;
}

static int cspm_disable_hw_governor(struct cpuhvfs_dvfsp *dvfsp, struct init_sta *ret_sta)
{
	int i, r = 0;
	u32 f_curr[NUM_CPU_CLUSTER], swctrl;

	spin_lock(&dvfs_lock);
	cspm_dbgx(HWGOV, "(%u) [%08x] disable hw gov\n", dvfsp->hw_gov_en, cspm_get_timestamp());

	if (!dvfsp->hw_gov_en)
		goto UNLOCK;

	r = __cspm_pause_pcm_running(dvfsp, PSF_PAUSE_HWGOV);
	if (!r) {
		for (i = 0; i < NUM_CPU_CLUSTER; i++) {
			f_curr[i] = cspm_curr_freq(hwsta_reg[i]);
			ret_sta->opp[i] = opp_fw_to_sw(f_curr[i]);
		}

		for (i = 0; i < NUM_PHY_CLUSTER; i++) {
			swctrl = cspm_read(swctrl_reg[i]);
			swctrl &= ~(SW_F_DES_MASK | SW_F_MAX_MASK | SW_F_MIN_MASK);
			swctrl |= SW_F_MAX(NUM_CPU_OPP - 1) | SW_F_MIN(0);	/* no limit */
			cspm_write(swctrl_reg[i], swctrl | SW_F_ASSIGN | SW_F_DES(f_curr[i]));
			csram_write(swctrl_offs[i], cspm_read(swctrl_reg[i]));
		}
		csram_write(OFFS_HWGOV_ETIME, cspm_get_timestamp());

		dvfsp->hw_gov_en = 0;	/* before unpause to avoid starting nfy_trig_timer */
		csram_write(OFFS_HWGOV_EN, dvfsp->hw_gov_en);

		__cspm_unpause_pcm_to_run(dvfsp, PSF_PAUSE_HWGOV);
	}

UNLOCK:
	spin_unlock(&dvfs_lock);

	return r;
}

static void cspm_set_dvfs_notify(struct cpuhvfs_dvfsp *dvfsp, dvfs_notify_t callback)
{
	mutex_lock(&notify_mutex);
	notify_dvfs_change_cb = callback;
	mutex_unlock(&notify_mutex);
}
#endif

static void cspm_set_opp_limit(struct cpuhvfs_dvfsp *dvfsp, unsigned int cluster,
			       unsigned int ceiling, unsigned int floor)
{
	u32 f_max, f_min, swctrl;

	if (cluster >= NUM_PHY_CLUSTER)		/* CCI */
		return;

	spin_lock(&dvfs_lock);
	if (!dvfsp->hw_gov_en) {	/* no limit */
		ceiling = 0;
		floor = NUM_CPU_OPP - 1;
	}

	f_max = opp_sw_to_fw(ceiling);
	f_min = opp_sw_to_fw(floor);

	cspm_dbgx(LIMIT, "(%u) [%08x] cluster%u limit, opp = (%u - %u) <%u - %u>\n",
			 dvfsp->hw_gov_en, cspm_get_timestamp(), cluster,
			 ceiling, floor, f_max, f_min);

	swctrl = cspm_read(swctrl_reg[cluster]);
	swctrl &= ~(SW_F_MAX_MASK | SW_F_MIN_MASK);
	cspm_write(swctrl_reg[cluster], swctrl | SW_F_MAX(f_max) | SW_F_MIN(f_min));
	csram_write(swctrl_offs[cluster], cspm_read(swctrl_reg[cluster]));
	spin_unlock(&dvfs_lock);
}

static int cspm_set_target_opp(struct cpuhvfs_dvfsp *dvfsp, unsigned int cluster, unsigned int index,
			       unsigned int *ret_volt)
{
	int r = 0;
	u32 f_des = opp_sw_to_fw(index);
	u32 swctrl;

	if (cluster >= NUM_PHY_CLUSTER) {	/* CCI */
		if (ret_volt)
			*ret_volt = cspm_curr_volt(hwsta_reg[cluster]);
		return r;
	}

	spin_lock(&dvfs_lock);
	csram_write(OFFS_FUNC_ENTER, (cluster << 24) | (index << 16) | FEF_DVFS);

	cspm_dbgx(DVFS, "(%u) cluster%u dvfs, opp = %u <%u>, pause = 0x%x\n",
			dvfsp->hw_gov_en, cluster, index, f_des, pause_src_map);

	while (pause_src_map != 0) {
		csram_write(OFFS_DVFS_WAIT, (cluster << 8) | index);
		spin_unlock(&dvfs_lock);

		wait_event(dvfs_wait, pause_src_map == 0);	/* wait for PCM to be unpaused */

		spin_lock(&dvfs_lock);
	}

	swctrl = cspm_read(swctrl_reg[cluster]);
	csram_write(swctrl_offs[cluster], swctrl);

	if (!(swctrl & SW_PAUSE) /* cluster on */ && !dvfsp->hw_gov_en) {
		swctrl &= ~SW_F_DES_MASK;
		cspm_write(swctrl_reg[cluster], swctrl | SW_F_ASSIGN | SW_F_DES(f_des));
		csram_write(swctrl_offs[cluster], cspm_read(swctrl_reg[cluster]));

		r = wait_complete_us(cspm_curr_freq(hwsta_reg[cluster]) == f_des, 10, DVFS_TIMEOUT);
		csram_write_fw_sta();
	}

	if (r >= 0) {	/* DVFS done */
		r = 0;
		if (ret_volt)
			*ret_volt = cspm_curr_volt(hwsta_reg[cluster]);
	} else {
		cspm_dump_debug_info(dvfsp, "CLUSTER%u DVFS TIMEOUT, opp = %u <%u>",
					    cluster, index, f_des);
		BUG_ON(dvfs_fail_ke);
	}

	csram_write(OFFS_FUNC_ENTER, 0);
	spin_unlock(&dvfs_lock);

	return r;
}

static unsigned int cspm_get_curr_volt(struct cpuhvfs_dvfsp *dvfsp, unsigned int cluster)
{
	return cspm_curr_volt(hwsta_reg[cluster]);
}

static void cspm_cluster_notify_on(struct cpuhvfs_dvfsp *dvfsp, unsigned int cluster)
{
	int i;
	u32 time, hwsta, swctrl;

	if (cluster >= NUM_PHY_CLUSTER)		/* CCI */
		return;

	spin_lock(&dvfs_lock);
	time = cspm_get_timestamp();
	hwsta = cspm_read(hwsta_reg[cluster]);
	swctrl = cspm_read(swctrl_reg[cluster]);

	csram_write(hwsta_offs[cluster], hwsta);
	csram_write(swctrl_offs[cluster], swctrl);

	cspm_dbgx(CLUSTER, "(%u) [%08x] cluster%u on, pause = 0x%x, swctrl = 0x%x (0x%x)\n",
			   dvfsp->hw_gov_en, time, cluster, pause_src_map, swctrl, hwsta);

	WARN_ON(!(swctrl & SW_PAUSE));	/* not paused at cluster off */

	if (pause_src_map == 0)
		swctrl &= ~SW_PAUSE;
	cspm_write(swctrl_reg[cluster], swctrl | CLUSTER_EN);
	csram_write(swctrl_offs[cluster], cspm_read(swctrl_reg[cluster]));

	if (pause_src_map & PSF_PAUSE_INIT) {
		for (i = 0; i < NUM_PHY_CLUSTER; i++) {
			swctrl = cspm_read(swctrl_reg[i]);
			csram_write(swctrl_offs[i], swctrl);

			BUG_ON(!(swctrl & SW_PAUSE));	/* not paused after kick */
		}

		__cspm_unpause_pcm_to_run(dvfsp, PSF_PAUSE_INIT);
	}
	spin_unlock(&dvfs_lock);
}

static void cspm_cluster_notify_off(struct cpuhvfs_dvfsp *dvfsp, unsigned int cluster)
{
	int r;
	u32 time, hwsta, swctrl;

	if (cluster >= NUM_PHY_CLUSTER)		/* CCI */
		return;

	spin_lock(&dvfs_lock);
	csram_write(OFFS_FUNC_ENTER, (cluster << 24) | FEF_CLUSTER_OFF);

	time = cspm_get_timestamp();
	hwsta = cspm_read(hwsta_reg[cluster]);
	swctrl = cspm_read(swctrl_reg[cluster]);

	csram_write(hwsta_offs[cluster], hwsta);
	csram_write(swctrl_offs[cluster], swctrl);

	cspm_dbgx(CLUSTER, "(%u) [%08x] cluster%u off, pause = 0x%x, swctrl = 0x%x (0x%x)\n",
			   dvfsp->hw_gov_en, time, cluster, pause_src_map, swctrl, hwsta);

	if (WARN_ON(!(swctrl & CLUSTER_EN)))		/* already off */
		goto out;

	cspm_write(swctrl_reg[cluster], swctrl & ~(CLUSTER_EN | SW_PAUSE | SW_F_ASSIGN));
	csram_write(swctrl_offs[cluster], cspm_read(swctrl_reg[cluster]));

	/* FW will set SW_PAUSE when done */
	if (cluster == CPU_CLUSTER_B) {
		r = wait_complete_us(cspm_is_paused(swctrl_reg[cluster]), 10, DVFS_TIMEOUT);
		csram_write_fw_sta();
	} else {	/* LL, L: DFS only to the lowest frequency */
		r = wait_complete_us(cspm_curr_freq(hwsta_reg[cluster]) == 0 &&
				     cspm_is_paused(swctrl_reg[cluster]), 10, DVFS_TIMEOUT);
		csram_write_fw_sta();
	}

	if (r < 0) {
		cspm_dump_debug_info(dvfsp, "CLUSTER%u OFF TIMEOUT", cluster);
		BUG();
	}
out:
	csram_write(OFFS_FUNC_ENTER, 0);
	spin_unlock(&dvfs_lock);
}

static bool cspm_is_pcm_kicked(struct cpuhvfs_dvfsp *dvfsp)
{
	return !!(cspm_read(CSPM_PCM_FSM_STA) & FSM_PCM_KICK);
}

static void __cspm_check_and_update_sta(struct cpuhvfs_dvfsp *dvfsp, struct init_sta *sta)
{
	int i;
	bool suspend = false;

	for (i = 0; i < NUM_CPU_CLUSTER; i++) {
		if (!dvfsp->hw_gov_en) {	/* no limit */
			sta->ceiling[i] = 0;
			sta->floor[i] = NUM_CPU_OPP - 1;
		}

		if (sta->opp[i] == OPP_AT_SUSPEND) {
			BUG_ON(suspend_sta.opp[i] == UINT_MAX);		/* without suspend */

			sta->opp[i] = suspend_sta.opp[i];
			suspend_sta.opp[i] = UINT_MAX;
			suspend = true;
		}
		if (sta->volt[i] == VOLT_AT_SUSPEND) {
			BUG_ON(suspend_sta.volt[i] == UINT_MAX);	/* without suspend */

			sta->volt[i] = suspend_sta.volt[i];
			suspend_sta.volt[i] = UINT_MAX;
		}
		if (sta->vsram[i] == VSRAM_AT_SUSPEND) {
			BUG_ON(suspend_sta.vsram[i] == UINT_MAX);	/* without suspend */

			sta->vsram[i] = suspend_sta.vsram[i];
			suspend_sta.vsram[i] = UINT_MAX;
		}
		if (sta->ceiling[i] == CEILING_AT_SUSPEND) {
			BUG_ON(suspend_sta.ceiling[i] == UINT_MAX);	/* without suspend */

			sta->ceiling[i] = suspend_sta.ceiling[i];
			suspend_sta.ceiling[i] = UINT_MAX;
		}
		if (sta->floor[i] == FLOOR_AT_SUSPEND) {
			BUG_ON(suspend_sta.floor[i] == UINT_MAX);	/* without suspend */

			sta->floor[i] = suspend_sta.floor[i];
			suspend_sta.floor[i] = UINT_MAX;
		}

		csram_write(OFFS_INIT_OPP + i * sizeof(u32), sta->opp[i]);
		csram_write(OFFS_INIT_FREQ + i * sizeof(u32), sta->freq[i]);
		csram_write(OFFS_INIT_VOLT + i * sizeof(u32), sta->volt[i]);
		csram_write(OFFS_INIT_VSRAM + i * sizeof(u32), sta->vsram[i]);

		if (!suspend || i <= CPU_CLUSTER_L) {
			cspm_dbgx(KICK, "(%u) cluster%d: opp = %u (%u - %u), freq = %u, volt = 0x%x (0x%x)\n",
					dvfsp->hw_gov_en, i, sta->opp[i], sta->ceiling[i], sta->floor[i],
					sta->freq[i], sta->volt[i], sta->vsram[i]);
		}
	}
}

static int cspm_go_to_dvfs(struct cpuhvfs_dvfsp *dvfsp, struct init_sta *sta)
{
	struct pcm_desc *pcmdesc = dvfsp->pcmdesc;
	struct pwr_ctrl *pwrctrl = dvfsp->pwrctrl;

	spin_lock(&dvfs_lock);
	__cspm_check_and_update_sta(dvfsp, sta);

	__cspm_reset_and_init_pcm(pcmdesc);

	__cspm_kick_im_to_fetch(pcmdesc);

	__cspm_init_pcm_register();

	__cspm_init_event_vector(pcmdesc);

	__cspm_set_twam_event(pwrctrl);

	__cspm_set_wakeup_event(pwrctrl);

	__cspm_kick_pcm_to_run(pwrctrl, sta);

	spin_unlock(&dvfs_lock);

	return 0;
}

#ifdef CPUHVFS_HW_GOVERNOR
static void fetch_dvfs_log_and_notify(struct cpuhvfs_dvfsp *dvfsp)
{
	int i, j;
	u32 log;

	spin_lock(&dvfs_lock);
	if (!dvfsp->hw_gov_en || (log_box[MAX_LOG_FETCH].time == 0 &&
					log_time_curr_offs() <= OFFS_LOG_S)) {
		spin_unlock(&dvfs_lock);
		return;
	}

	log_box[0] = log_box[MAX_LOG_FETCH];

	for (i = 1; i <= MAX_LOG_FETCH; i++) {
		log_box[i].time = csram_read(next_log_offs);
		if (log_box[i].time <= log_box[i - 1].time)
			break;

		next_log_offs += 4;
		for (j = 0; j < NUM_CPU_CLUSTER; j++) {
			log = csram_read(next_log_offs);
			log_box[i].opp[j] = opp_fw_to_sw(log_get_freq(log));
			next_log_offs += 4;
		}

		next_log_offs += 4;	/* skip UE log */
		if (next_log_offs >= OFFS_LOG_E)
			next_log_offs = OFFS_LOG_S;
		csram_write(OFFS_NXLOG_OFFS, next_log_offs);

		cspm_dbgx(NOTIFY, "[%08x] log%d opp = (%u %u %u %u), offs = 0x%x (0x%x)\n",
				  log_box[i].time, i, log_box[i].opp[0], log_box[i].opp[1],
				  log_box[i].opp[2], log_box[i].opp[3],
				  next_log_offs, log_time_curr_offs());
	}

	if (log_box[0].time == 0 && i >= 2)
		log_box[0] = log_box[1];

	if (i >= 2 && i <= MAX_LOG_FETCH)
		log_box[MAX_LOG_FETCH] = log_box[i - 1];
	spin_unlock(&dvfs_lock);

	mutex_lock(&notify_mutex);
	if (notify_dvfs_change_cb && log_box[0].time > 0)
		notify_dvfs_change_cb(log_box, i);
	mutex_unlock(&notify_mutex);
}

static int dvfs_nfy_task_fn(void *data)
{
	struct cpuhvfs_dvfsp *dvfsp = data;

	while (1) {
		set_current_state(TASK_UNINTERRUPTIBLE);

		if (atomic_read(&dvfs_nfy_req) <= 0) {
			schedule();
			continue;
		}

		__set_current_state(TASK_RUNNING);

		fetch_dvfs_log_and_notify(dvfsp);
		atomic_dec(&dvfs_nfy_req);
	}

	return 0;
}

static enum hrtimer_restart nfy_trig_timer_fn(struct hrtimer *timer)
{
	kick_kthread_to_notify();

	hrtimer_forward_now(timer, ms_to_ktime(DVFS_NOTIFY_INTV));

	return HRTIMER_RESTART;
}

static int create_resource_for_dvfs_notify(struct cpuhvfs_dvfsp *dvfsp)
{
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 2 };	/* lower than WDK */

	/* init hrtimer */
	hrtimer_init(&nfy_trig_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	nfy_trig_timer.function = nfy_trig_timer_fn;

	/* create kthread */
	dvfs_nfy_task = kthread_create(dvfs_nfy_task_fn, dvfsp, "dvfs_nfy");
	if (IS_ERR(dvfs_nfy_task))
		return PTR_ERR(dvfs_nfy_task);

	sched_setscheduler_nocheck(dvfs_nfy_task, SCHED_FIFO, &param);
	get_task_struct(dvfs_nfy_task);

	return 0;
}
#endif

static int cspm_probe(struct platform_device *pdev)
{
	int r;

	cspm_base = of_iomap(pdev->dev.of_node, 0);
	if (!cspm_base) {
		cspm_err("FAILED TO MAP CSPM REGISTER\n");
		return -ENOMEM;
	}

	csram_base = of_iomap(pdev->dev.of_node, 1);
	if (!csram_base) {
		cspm_err("FAILED TO MAP CSRAM MEMORY\n");
		return -ENOMEM;
	}

	i2c_clk = devm_clk_get(&pdev->dev, "i2c");
	if (IS_ERR(i2c_clk)) {
		cspm_err("FAILED TO GET I2C CLOCK (%ld)\n", PTR_ERR(i2c_clk));
		return PTR_ERR(i2c_clk);
	}
	r = clk_prepare(i2c_clk);
	if (r) {
		cspm_err("FAILED TO PREPARE I2C CLOCK (%d)\n", r);
		return r;
	}

	/* build register mapping for general access */
	swctrl_reg[CPU_CLUSTER_LL] = CSPM_SW_RSV0;
	swctrl_reg[CPU_CLUSTER_L] = CSPM_SW_RSV1;
	swctrl_reg[CPU_CLUSTER_B] = CSPM_SW_RSV2;

	hwsta_reg[CPU_CLUSTER_LL] = CSPM_SW_RSV3;
	hwsta_reg[CPU_CLUSTER_L] = CSPM_SW_RSV4;
	hwsta_reg[CPU_CLUSTER_B] = CSPM_SW_RSV5;
	hwsta_reg[CPU_CLUSTER_CCI] = CSPM_SW_RSV6;

	sema_reg[SEMA_FHCTL_DRV] = CSPM_SEMA0_M0;

	swctrl_offs[CPU_CLUSTER_LL] = OFFS_SW_RSV0;
	swctrl_offs[CPU_CLUSTER_L] = OFFS_SW_RSV1;
	swctrl_offs[CPU_CLUSTER_B] = OFFS_SW_RSV2;

	hwsta_offs[CPU_CLUSTER_LL] = OFFS_FW_RSV3;
	hwsta_offs[CPU_CLUSTER_L] = OFFS_FW_RSV4;
	hwsta_offs[CPU_CLUSTER_B] = OFFS_FW_RSV5;
	hwsta_offs[CPU_CLUSTER_CCI] = OFFS_FW_RSV6;

	__cspm_register_init();

	cspm_probe_done = 1;

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id cspm_of_match[] = {
	{ .compatible = "mediatek,mt6797-dvfsp", },
	{}
};
#endif

static struct platform_driver cspm_driver = {
	.probe	= cspm_probe,
	.driver	= {
		.name		= "cspm",
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(cspm_of_match),
	},
};

static int cspm_module_init(struct cpuhvfs_dvfsp *dvfsp)
{
	int r;

	r = platform_driver_register(&cspm_driver);
	if (r) {
		cspm_err("FAILED TO REGISTER CSPM DRIVER (%d)\n", r);
		return r;
	}

	if (!cspm_probe_done) {
		cspm_err("FAILED TO PROBE CSPM DEVICE\n");
		return -ENODEV;
	}

#ifdef CPUHVFS_HW_GOVERNOR
	r = create_resource_for_dvfs_notify(dvfsp);
	if (r) {
		cspm_err("FAILED TO CREATE RESOURCE FOR NOTIFY (%d)\n", r);
		return r;
	}
#endif

	dvfsp->log_repo = csram_base;

	return 0;
}


/**************************************
 * [Hybrid DVFS] Function
 **************************************/
static int dvfsp_fw_show(struct seq_file *m, void *v)
{
	struct cpuhvfs_dvfsp *dvfsp = m->private;
	struct pcm_desc *pcmdesc = dvfsp->pcmdesc;

	seq_printf(m, "version = %s\n"  , pcmdesc->version);
	seq_printf(m, "base    = 0x%p -> 0x%x\n", pcmdesc->base, base_va_to_pa(pcmdesc->base));
	seq_printf(m, "size    = %u\n"  , pcmdesc->size);
	seq_printf(m, "sess    = %u\n"  , pcmdesc->sess);
	seq_printf(m, "replace = %u\n"  , pcmdesc->replace);
	seq_printf(m, "vec0    = 0x%x\n", pcmdesc->vec0);

	return 0;
}

static int dvfsp_reg_show(struct seq_file *m, void *v)
{
	struct cpuhvfs_dvfsp *dvfsp = m->private;

	seq_printf(m, "PCM_TIMER  : %08x\n", cspm_read(CSPM_PCM_TIMER_OUT));
	seq_printf(m, "PCM_REG15  : %u\n"  , cspm_read(CSPM_PCM_REG15_DATA));
	seq_puts(m,   "============\n");
	seq_printf(m, "SW_RSV0    : 0x%x\n", cspm_read(CSPM_SW_RSV0));
	seq_printf(m, "SW_RSV1    : 0x%x\n", cspm_read(CSPM_SW_RSV1));
	seq_printf(m, "SW_RSV2    : 0x%x\n", cspm_read(CSPM_SW_RSV2));
	seq_printf(m, "SW_RSV3    : 0x%x\n", cspm_read(CSPM_SW_RSV3));
	seq_printf(m, "SW_RSV4    : 0x%x\n", cspm_read(CSPM_SW_RSV4));
	seq_printf(m, "SW_RSV5    : 0x%x\n", cspm_read(CSPM_SW_RSV5));
	seq_printf(m, "SW_RSV6    : 0x%x\n", cspm_read(CSPM_SW_RSV6));
	seq_puts(m,   "============\n");
	seq_printf(m, "M3_REC1    : 0x%x\n", cspm_read(CSPM_M3_REC1));
	seq_printf(m, "M3_REC2    : 0x%x\n", cspm_read(CSPM_M3_REC2));
	seq_printf(m, "M3_REC3    : 0x%x\n", cspm_read(CSPM_M3_REC3));
	seq_printf(m, "M3_REC4    : 0x%x\n", cspm_read(CSPM_M3_REC4));
	seq_printf(m, "M3_REC5    : 0x%x\n", cspm_read(CSPM_M3_REC5));
	seq_printf(m, "M3_REC6    : 0x%x\n", cspm_read(CSPM_M3_REC6));
	seq_printf(m, "M3_REC7    : 0x%x\n", cspm_read(CSPM_M3_REC7));
	seq_printf(m, "M3_REC8    : 0x%x\n", cspm_read(CSPM_M3_REC8));
	seq_printf(m, "M3_REC9    : 0x%x\n", cspm_read(CSPM_M3_REC9));
	seq_printf(m, "M3_REC10   : 0x%x\n", cspm_read(CSPM_M3_REC10));
	seq_printf(m, "M3_REC11   : 0x%x\n", cspm_read(CSPM_M3_REC11));
	seq_printf(m, "M3_REC12   : 0x%x\n", cspm_read(CSPM_M3_REC12));
	seq_printf(m, "M3_REC13   : 0x%x\n", cspm_read(CSPM_M3_REC13));
	seq_printf(m, "M3_REC14   : 0x%x\n", cspm_read(CSPM_M3_REC14));
	seq_printf(m, "M3_REC15   : 0x%x\n", cspm_read(CSPM_M3_REC15));
	seq_printf(m, "M3_REC16   : 0x%x\n", cspm_read(CSPM_M3_REC16));
	seq_printf(m, "M3_REC17   : 0x%x\n", cspm_read(CSPM_M3_REC17));
	seq_printf(m, "M3_REC18   : 0x%x\n", cspm_read(CSPM_M3_REC18));
	seq_printf(m, "M3_REC19   : 0x%x\n", cspm_read(CSPM_M3_REC19));
	seq_printf(m, "SW_DEBUG   : 0x%x\n", cspm_read(CSPM_SW_DEBUG));
	seq_puts(m,   "============\n");
	seq_printf(m, "EVT_VEC2   : 0x%x\n", cspm_read(CSPM_PCM_EVENT_VECTOR2));
	seq_printf(m, "EVT_VEC3   : 0x%x\n", cspm_read(CSPM_PCM_EVENT_VECTOR3));
	seq_printf(m, "EVT_VEC5   : 0x%x\n", cspm_read(CSPM_PCM_EVENT_VECTOR5));
	seq_printf(m, "EVT_VEC7   : 0x%x\n", cspm_read(CSPM_PCM_EVENT_VECTOR7));
	seq_printf(m, "EVT_VEC8   : 0x%x\n", cspm_read(CSPM_PCM_EVENT_VECTOR8));
	seq_printf(m, "EVT_VEC10  : 0x%x\n", cspm_read(CSPM_PCM_EVENT_VECTOR10));
	seq_printf(m, "EVT_VEC12  : 0x%x\n", cspm_read(CSPM_PCM_EVENT_VECTOR12));
	seq_printf(m, "EVT_VEC13  : 0x%x\n", cspm_read(CSPM_PCM_EVENT_VECTOR13));
	seq_printf(m, "EVT_VEC15  : 0x%x\n", cspm_read(CSPM_PCM_EVENT_VECTOR15));
	seq_puts(m,   "============\n");
	seq_printf(m, "PCM_FSM_STA: 0x%x\n", cspm_read(CSPM_PCM_FSM_STA));
	seq_printf(m, "HW_GOV*    : %u\n"  , dvfsp->hw_gov_en);

	return 0;
}

static int dbg_repo_show(struct seq_file *m, void *v)
{
	int i;
	u32 *repo = m->private;
	char ch;

	/* only save current status to debug repo */
	if (repo != dbg_repo_bak) {
		repo[REPO_I_WDT_LATCH0] = cspm_read(CSPM_PCM_FSM_STA);
		repo[REPO_I_WDT_LATCH1] = cspm_read(CSPM_PCM_REG6_DATA);
		repo[REPO_I_WDT_LATCH2] = cspm_read(CSPM_SW_RSV5);
	}

	for (i = 0; i < DBG_REPO_NUM; i++) {
		if (i >= REPO_I_LOG_S && (i - REPO_I_LOG_S) % ENTRY_EACH_LOG == 0)
			ch = ':';	/* timestamp */
		else
			ch = '.';

		seq_printf(m, "%4d%c%08x%c", i, ch, repo[i], i % 4 == 3 ? '\n' : ' ');
	}

	return 0;
}

DEFINE_FOPS_RO(dvfsp_fw);
DEFINE_FOPS_RO(dvfsp_reg);
DEFINE_FOPS_RO(dbg_repo);

static int create_cpuhvfs_debug_fs(struct cpuhvfs_data *cpuhvfs)
{
	struct dentry *root;

	/* create /sys/kernel/debug/cpuhvfs */
	root = debugfs_create_dir("cpuhvfs", NULL);
	if (!root)
		return -EPERM;

	debugfs_create_file("dvfsp_fw", 0444, root, cpuhvfs->dvfsp, &dvfsp_fw_fops);
	debugfs_create_file("dvfsp_reg", 0444, root, cpuhvfs->dvfsp, &dvfsp_reg_fops);

	debugfs_create_file("dbg_repo", 0444, root, cpuhvfs->dbg_repo, &dbg_repo_fops);
	debugfs_create_file("dbg_repo_bak", 0444, root, dbg_repo_bak, &dbg_repo_fops);

	debugfs_create_x32("pause_src_map", 0444, root, &pause_src_map);
	debugfs_create_x32("pause_log_en", 0644, root, &pause_log_en);

	debugfs_create_x32("dbgx_log_en", 0644, root, &dbgx_log_en);

	debugfs_create_bool("dvfs_fail_ke", 0644, root, &dvfs_fail_ke);
	debugfs_create_bool("sema_fail_ke", 0644, root, &sema_fail_ke);
	debugfs_create_bool("pause_fail_ke", 0644, root, &pause_fail_ke);

	debugfs_create_bool("wa_get_emi_sema", 0444, root, &wa_get_emi_sema);

	cpuhvfs->dbg_fs = root;

	return 0;
}

static void init_cpuhvfs_debug_repo(struct cpuhvfs_data *cpuhvfs)
{
	u32 __iomem *dbg_repo = cpuhvfs->dvfsp->log_repo;

	/* backup debug repo for later analysis */
	memcpy_fromio(dbg_repo_bak, dbg_repo, DBG_REPO_SIZE);
	dbg_repo_bak[REPO_I_WDT_LATCH0] = cspm_read(CSPM_PCM_WDT_LATCH0);
	dbg_repo_bak[REPO_I_WDT_LATCH1] = cspm_read(CSPM_PCM_WDT_LATCH1);
	dbg_repo_bak[REPO_I_WDT_LATCH2] = cspm_read(CSPM_PCM_WDT_LATCH2);
	cpuhvfs_crit("WDT_LATCH0: 0x%x, LATCH1: 0x%x, LATCH2: 0x%x\n",
		     dbg_repo_bak[REPO_I_WDT_LATCH0],
		     dbg_repo_bak[REPO_I_WDT_LATCH1],
		     dbg_repo_bak[REPO_I_WDT_LATCH2]);

	dbg_repo[0] = REPO_GUARD0;
	dbg_repo[1] = REPO_GUARD1;
	dbg_repo[2] = REPO_GUARD0;
	dbg_repo[3] = REPO_GUARD1;

	memset_io((void __iomem *)dbg_repo + DBG_REPO_DATA_S - DBG_REPO_S,
		  0,
		  DBG_REPO_E - DBG_REPO_DATA_S);

	dbg_repo[REPO_I_DATA_S] = REPO_GUARD0;

	cpuhvfs->dbg_repo = dbg_repo;
}


/**************************************
 * [Hybrid DVFS] API
 **************************************/
void cpuhvfs_get_pause_status_i2c(void)		/* deprecated */
{
	cspm_err("pause_src_map = 0x%x\n", pause_src_map);
	cspm_err("SW_RSV0: 0x%x\n", cspm_read(CSPM_SW_RSV0));
	cspm_err("SW_RSV1: 0x%x\n", cspm_read(CSPM_SW_RSV1));
	cspm_err("SW_RSV2: 0x%x\n", cspm_read(CSPM_SW_RSV2));
}

void cpuhvfs_dump_dvfsp_info(void)
{
	struct cpuhvfs_dvfsp *dvfsp = g_cpuhvfs.dvfsp;

	if (is_dvfsp_uninit(dvfsp))
		return;

	dvfsp->dump_info(dvfsp, "DVFSP INFO DUMP");
}

/**
 * Should be called at syscore_suspend stage (1 CPU/1 cluster)
 */
int cpuhvfs_dvfsp_suspend(void)
{
	struct cpuhvfs_dvfsp *dvfsp = g_cpuhvfs.dvfsp;

	if (is_dvfsp_uninit(dvfsp))
		return 0;

	return dvfsp->pause_dvfsp(dvfsp, PAUSE_SUSPEND);
}

/**
 * Should be called at syscore_resume stage (1 CPU/1 cluster)
 */
void cpuhvfs_dvfsp_resume(unsigned int on_cluster, struct init_sta *sta)
{
	int r;
	struct cpuhvfs_dvfsp *dvfsp = g_cpuhvfs.dvfsp;

	if (on_cluster >= NUM_CPU_CLUSTER || !sta)
		return;

	if (is_dvfsp_uninit(dvfsp))
		return;

	if (!dvfsp->is_kicked(dvfsp)) {
		r = dvfsp->kick_dvfsp(dvfsp, sta);
		BUG_ON(r);

		dvfsp->cluster_on(dvfsp, on_cluster);
	}

	dvfsp->unpause_dvfsp(dvfsp, PAUSE_SUSPEND);
}

int cpuhvfs_stop_dvfsp_running(void)
{
	struct cpuhvfs_dvfsp *dvfsp = g_cpuhvfs.dvfsp;

	if (is_dvfsp_uninit(dvfsp))
		return 0;

	return dvfsp->stop_dvfsp(dvfsp);
}

int cpuhvfs_restart_dvfsp_running(struct init_sta *sta)
{
	int r, i;
	struct cpuhvfs_dvfsp *dvfsp = g_cpuhvfs.dvfsp;

	if (!sta)
		return -EINVAL;

	if (is_dvfsp_uninit(dvfsp))
		return -ENODEV;

	if (dvfsp->is_kicked(dvfsp))
		return -EPERM;

	r = dvfsp->kick_dvfsp(dvfsp, sta);
	if (r)
		return r;

	for (i = 0; i < NUM_CPU_CLUSTER; i++) {
		if (sta->is_on[i])
			dvfsp->cluster_on(dvfsp, i);
	}

	return 0;
}

int cpuhvfs_pause_dvfsp_running(enum pause_src src)
{
	struct cpuhvfs_dvfsp *dvfsp = g_cpuhvfs.dvfsp;

	if (src >= NUM_PAUSE_SRC || src == PAUSE_INIT || src == PAUSE_SUSPEND ||
	    src == PAUSE_HWGOV)
		return -EINVAL;

	return dvfsp->pause_dvfsp(dvfsp, src);
}

void cpuhvfs_unpause_dvfsp_to_run(enum pause_src src)
{
	struct cpuhvfs_dvfsp *dvfsp = g_cpuhvfs.dvfsp;

	if (src >= NUM_PAUSE_SRC || src == PAUSE_INIT || src == PAUSE_SUSPEND ||
	    src == PAUSE_HWGOV)
		return;

	dvfsp->unpause_dvfsp(dvfsp, src);
}

int cpuhvfs_get_dvfsp_semaphore(enum sema_user user)
{
	struct cpuhvfs_dvfsp *dvfsp = g_cpuhvfs.dvfsp;

	if (user >= NUM_SEMA_USER)
		return -EINVAL;

	return dvfsp->get_sema(dvfsp, user);
}

void cpuhvfs_release_dvfsp_semaphore(enum sema_user user)
{
	struct cpuhvfs_dvfsp *dvfsp = g_cpuhvfs.dvfsp;

	if (user >= NUM_SEMA_USER)
		return;

	dvfsp->release_sema(dvfsp, user);
}

#ifdef CPUHVFS_HW_GOVERNOR
/**
 * Init state (must: ceiling, floor) needs to be passed by @sta
 */
int cpuhvfs_enable_hw_governor(struct init_sta *sta)
{
	struct cpuhvfs_dvfsp *dvfsp = g_cpuhvfs.dvfsp;

	if (!sta)
		return -EINVAL;

	if (is_dvfsp_uninit(dvfsp))
		return -ENODEV;

	if (!dvfsp->is_kicked(dvfsp)) {
		cpuhvfs_err("CANNOT ENABLE HWGOV, DVFSP IS NOT KICKED\n");
		return -EPERM;
	}

	return dvfsp->enable_hwgov(dvfsp, sta);
}

/**
 * Current state (must: OPP) will be saved in @ret_sta if succeed
 */
int cpuhvfs_disable_hw_governor(struct init_sta *ret_sta)
{
	struct cpuhvfs_dvfsp *dvfsp = g_cpuhvfs.dvfsp;

	if (!ret_sta)
		return -EINVAL;

	if (is_dvfsp_uninit(dvfsp))
		return -ENODEV;

	if (!dvfsp->is_kicked(dvfsp)) {
		cpuhvfs_err("CANNOT DISABLE HWGOV, DVFSP IS NOT KICKED\n");
		return -EPERM;
	}

	return dvfsp->disable_hwgov(dvfsp, ret_sta);
}

void cpuhvfs_register_dvfs_notify(dvfs_notify_t callback)
{
	struct cpuhvfs_dvfsp *dvfsp = g_cpuhvfs.dvfsp;

	dvfsp->set_notify(dvfsp, callback);
}
#endif

void cpuhvfs_set_opp_limit(unsigned int cluster, unsigned int ceiling, unsigned int floor)
{
	struct cpuhvfs_dvfsp *dvfsp = g_cpuhvfs.dvfsp;

	if (cluster >= NUM_CPU_CLUSTER || ceiling >= NUM_CPU_OPP || floor >= NUM_CPU_OPP)
		return;

	if (ceiling > floor) {
		cpuhvfs_err("OPP FLOOR IS OVER CEILING\n");
		return;
	}

	if (is_dvfsp_uninit(dvfsp))
		return;

	dvfsp->set_limit(dvfsp, cluster, ceiling, floor);
}

/**
 * Current voltage (PMIC value) will be saved in @ret_volt if succeed
 */
int cpuhvfs_set_target_opp(unsigned int cluster, unsigned int index, unsigned int *ret_volt)
{
	struct cpuhvfs_dvfsp *dvfsp = g_cpuhvfs.dvfsp;

	if (cluster >= NUM_CPU_CLUSTER || index >= NUM_CPU_OPP)
		return -EINVAL;

	if (is_dvfsp_uninit(dvfsp))
		return -ENODEV;

	if (!dvfsp->is_kicked(dvfsp)) {
		cpuhvfs_err("CANNOT SET OPP, DVFSP IS NOT KICKED\n");
		return -EPERM;
	}

	return dvfsp->set_target(dvfsp, cluster, index, ret_volt);
}

/**
 * Get current voltage (PMIC value) from DVFSP. Return UINT_MAX if unavailable
 */
unsigned int cpuhvfs_get_curr_volt(unsigned int cluster)
{
	struct cpuhvfs_dvfsp *dvfsp = g_cpuhvfs.dvfsp;

	if (cluster >= NUM_CPU_CLUSTER)
		return UINT_MAX;

	if (is_dvfsp_uninit(dvfsp))
		return UINT_MAX;

	if (!dvfsp->is_kicked(dvfsp))
		return UINT_MAX;

	return dvfsp->get_volt(dvfsp, cluster);
}

void cpuhvfs_notify_cluster_on(unsigned int cluster)
{
	struct cpuhvfs_dvfsp *dvfsp = g_cpuhvfs.dvfsp;

	if (cluster >= NUM_CPU_CLUSTER)
		return;

	if (is_dvfsp_uninit(dvfsp))
		return;

	if (!dvfsp->is_kicked(dvfsp))
		return;

	dvfsp->cluster_on(dvfsp, cluster);
}

/**
 * Require scaling to the lowest frequency for cluster off
 */
void cpuhvfs_notify_cluster_off(unsigned int cluster)
{
	struct cpuhvfs_dvfsp *dvfsp = g_cpuhvfs.dvfsp;

	BUG_ON(cluster >= NUM_CPU_CLUSTER);
	BUG_ON(is_dvfsp_uninit(dvfsp));
	BUG_ON(!dvfsp->is_kicked(dvfsp));

	dvfsp->cluster_off(dvfsp, cluster);
}

int cpuhvfs_kick_dvfsp_to_run(struct init_sta *sta)
{
	struct cpuhvfs_dvfsp *dvfsp = g_cpuhvfs.dvfsp;

	if (!sta)
		return -EINVAL;

	if (is_dvfsp_uninit(dvfsp))
		return -ENODEV;

	if (dvfsp->is_kicked(dvfsp))
		return 0;

	return dvfsp->kick_dvfsp(dvfsp, sta);
}

int cpuhvfs_module_init(void)
{
	int r;
	struct cpuhvfs_data *cpuhvfs = &g_cpuhvfs;

	if (!cpuhvfs->dbg_repo) {
		cpuhvfs_err("FAILED TO PRE-INIT CPUHVFS\n");
		return -ENODEV;
	}

	set_dvfsp_init_done(cpuhvfs->dvfsp);

	r = create_cpuhvfs_debug_fs(cpuhvfs);
	if (r) {
		cpuhvfs_err("FAILED TO CREATE DEBUG FILESYSTEM (%d)\n", r);
		return r;
	}

	return 0;
}

static int cpuhvfs_pre_module_init(void)
{
	int r;
	struct cpuhvfs_data *cpuhvfs = &g_cpuhvfs;

	r = cpuhvfs->dvfsp->init_dvfsp(cpuhvfs->dvfsp);
	if (r) {
		cpuhvfs_err("FAILED TO INIT DVFS PROCESSOR (%d)\n", r);
		return r;
	}

	init_cpuhvfs_debug_repo(cpuhvfs);

	return 0;
}
fs_initcall(cpuhvfs_pre_module_init);

#endif	/* CONFIG_HYBRID_CPU_DVFS */

MODULE_DESCRIPTION("Hybrid CPU DVFS Driver v0.7");
