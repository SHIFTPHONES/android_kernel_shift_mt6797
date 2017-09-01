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

#ifndef _MT_EEM_
#define _MT_EEM_

#ifdef __KERNEL__
#include <linux/kernel.h>
#include <mt-plat/sync_write.h>
#endif

#define EN_EEM (1) /* enable/disable EEM (SW) */

/**
 * 1: Select VCORE_AO eem detector
 * 0: Select VCORE_PDN eem detector
 */

/*
#define PERI_VCORE_EEM_CON0	(PERICFG_BASE + 0x408)
#define VCORE_EEMSEL		0:0
#define SEL_VCORE_AO		1
#define SEL_VCORE_PDN		0
*/

/* Thermal Register Definition */

/* EEM Structure */
typedef struct {
	unsigned int ADC_CALI_EN;
	unsigned int PTPINITEN;
	unsigned int PTPMONEN;

	unsigned int MDES;
	unsigned int BDES;
	unsigned int DCCONFIG;
	unsigned int DCMDET;
	unsigned int DCBDET;
	unsigned int AGECONFIG;
	unsigned int AGEM;
	unsigned int AGEDELTA;
	unsigned int DVTFIXED;
	unsigned int VCO;
	unsigned int MTDES;
	unsigned int MTS;
	unsigned int BTS;

	unsigned char FREQPCT0;
	unsigned char FREQPCT1;
	unsigned char FREQPCT2;
	unsigned char FREQPCT3;
	unsigned char FREQPCT4;
	unsigned char FREQPCT5;
	unsigned char FREQPCT6;
	unsigned char FREQPCT7;

	unsigned int DETWINDOW;
	unsigned int VMAX;
	unsigned int VMIN;
	unsigned int DTHI;
	unsigned int DTLO;
	unsigned int VBOOT;
	unsigned int DETMAX;

	unsigned int DCVOFFSETIN;
	unsigned int AGEVOFFSETIN;
} PTP_INIT_T;


enum eem_ctrl_id {
	EEM_CTRL_BIG = 0,
	EEM_CTRL_GPU = 1,
	EEM_CTRL_SOC = 2,
	EEM_CTRL_L = 3,
	EEM_CTRL_2L = 4,
	EEM_CTRL_CCI = 5,
	NR_EEM_CTRL,
};

enum eem_det_id {
	EEM_DET_BIG	= EEM_CTRL_BIG,
	EEM_DET_GPU	= EEM_CTRL_GPU,
	EEM_DET_SOC	= EEM_CTRL_SOC,
	EEM_DET_L	= EEM_CTRL_L,
	EEM_DET_2L	= EEM_CTRL_2L,
	EEM_DET_CCI	= EEM_CTRL_CCI,
	NR_EEM_DET,
};

enum mt_eem_cpu_id {
	MT_EEM_CPU_LL,
	MT_EEM_CPU_L,
	MT_EEM_CPU_B,
	MT_EEM_CPU_CCI,

	NR_MT_EEM_CPU,
};

/* Global variable for SW EFUSE*/
/* TODO: FIXME #include "devinfo.h" */
extern u32 get_devinfo_with_index(u32 index);

/* Global variabel for Idvfs */
extern unsigned int infoIdvfs;

#ifdef CONFIG_MTK_RAM_CONSOLE
	#define CONFIG_EEM_AEE_RR_REC 1
#endif

#ifdef CONFIG_EEM_AEE_RR_REC
enum eem_state {
	EEM_CPU_BIG_IS_SET_VOLT = 0,    /* B */
	EEM_GPU_IS_SET_VOLT,            /* G */
	EEM_CPU_LITTLE_IS_SET_VOLT, /* L */
	EEM_CPU_2_LITTLE_IS_SET_VOLT, /* 2L */
	EEM_CPU_CCI_IS_SET_VOLT, /* CCI */
};

extern void aee_rr_rec_ptp_60(u32 val);
extern void aee_rr_rec_ptp_64(u32 val);
extern void aee_rr_rec_ptp_68(u32 val);
extern void aee_rr_rec_ptp_6C(u32 val);
extern void aee_rr_rec_ptp_78(u32 val);
extern void aee_rr_rec_ptp_7C(u32 val);
extern void aee_rr_rec_ptp_80(u32 val);
extern void aee_rr_rec_ptp_84(u32 val);
extern void aee_rr_rec_ptp_88(u32 val);
extern void aee_rr_rec_ptp_8C(u32 val);
extern void aee_rr_rec_ptp_9C(u32 val);
extern void aee_rr_rec_ptp_A0(u32 val);
extern void aee_rr_rec_ptp_vboot(u64 val);
extern void aee_rr_rec_ptp_cpu_big_volt(u64 val);
extern void aee_rr_rec_ptp_cpu_big_volt_1(u64 val);
extern void aee_rr_rec_ptp_cpu_big_volt_2(u64 val);
extern void aee_rr_rec_ptp_cpu_big_volt_3(u64 val);
extern void aee_rr_rec_ptp_gpu_volt(u64 val);
extern void aee_rr_rec_ptp_gpu_volt_1(u64 val);
extern void aee_rr_rec_ptp_gpu_volt_2(u64 val);
extern void aee_rr_rec_ptp_gpu_volt_3(u64 val);
extern void aee_rr_rec_ptp_cpu_little_volt(u64 val);
extern void aee_rr_rec_ptp_cpu_little_volt_1(u64 val);
extern void aee_rr_rec_ptp_cpu_little_volt_2(u64 val);
extern void aee_rr_rec_ptp_cpu_little_volt_3(u64 val);
extern void aee_rr_rec_ptp_cpu_2_little_volt(u64 val);
extern void aee_rr_rec_ptp_cpu_2_little_volt_1(u64 val);
extern void aee_rr_rec_ptp_cpu_2_little_volt_2(u64 val);
extern void aee_rr_rec_ptp_cpu_2_little_volt_3(u64 val);
extern void aee_rr_rec_ptp_cpu_cci_volt(u64 val);
extern void aee_rr_rec_ptp_cpu_cci_volt_1(u64 val);
extern void aee_rr_rec_ptp_cpu_cci_volt_2(u64 val);
extern void aee_rr_rec_ptp_cpu_cci_volt_3(u64 val);
extern void aee_rr_rec_ptp_temp(u64 val);
extern void aee_rr_rec_ptp_status(u8 val);
extern void aee_rr_rec_eem_pi_offset(u8 val);

extern u32 aee_rr_curr_ptp_60(void);
extern u32 aee_rr_curr_ptp_64(void);
extern u32 aee_rr_curr_ptp_68(void);
extern u32 aee_rr_curr_ptp_6C(void);
extern u32 aee_rr_curr_ptp_78(void);
extern u32 aee_rr_curr_ptp_7C(void);
extern u32 aee_rr_curr_ptp_80(void);
extern u32 aee_rr_curr_ptp_84(void);
extern u32 aee_rr_curr_ptp_88(void);
extern u32 aee_rr_curr_ptp_8C(void);
extern u32 aee_rr_curr_ptp_9C(void);
extern u32 aee_rr_curr_ptp_A0(void);
extern u64 aee_rr_curr_ptp_vboot(void);
extern u64 aee_rr_curr_ptp_cpu_big_volt(void);
extern u64 aee_rr_curr_ptp_cpu_big_volt_1(void);
extern u64 aee_rr_curr_ptp_cpu_big_volt_2(void);
extern u64 aee_rr_curr_ptp_cpu_big_volt_3(void);
extern u64 aee_rr_curr_ptp_gpu_volt(void);
extern u64 aee_rr_curr_ptp_gpu_volt_1(void);
extern u64 aee_rr_curr_ptp_gpu_volt_2(void);
extern u64 aee_rr_curr_ptp_gpu_volt_3(void);
extern u64 aee_rr_curr_ptp_cpu_little_volt(void);
extern u64 aee_rr_curr_ptp_cpu_little_volt_1(void);
extern u64 aee_rr_curr_ptp_cpu_little_volt_2(void);
extern u64 aee_rr_curr_ptp_cpu_little_volt_3(void);
extern u64 aee_rr_curr_ptp_cpu_2_little_volt(void);
extern u64 aee_rr_curr_ptp_cpu_2_little_volt_1(void);
extern u64 aee_rr_curr_ptp_cpu_2_little_volt_2(void);
extern u64 aee_rr_curr_ptp_cpu_2_little_volt_3(void);
extern u64 aee_rr_curr_ptp_cpu_cci_volt(void);
extern u64 aee_rr_curr_ptp_cpu_cci_volt_1(void);
extern u64 aee_rr_curr_ptp_cpu_cci_volt_2(void);
extern u64 aee_rr_curr_ptp_cpu_cci_volt_3(void);
extern u64 aee_rr_curr_ptp_temp(void);
extern u8 aee_rr_curr_ptp_status(void);
#endif


/* EEM Extern Function */
extern void mt_ptp_lock(unsigned long *flags);
extern void mt_ptp_unlock(unsigned long *flags);
extern int mt_eem_status(enum eem_det_id id);
extern int is_have_550(void);
extern unsigned int get_vcore_ptp_volt(int uv);
extern void eem_set_pi_offset(enum eem_ctrl_id id, int step);
extern void eem_set_big_efuse(enum eem_ctrl_id id, unsigned int big_efuse);
extern unsigned int get_efuse_status(void);
extern unsigned int get_eem_status_for_gpu(void);
extern unsigned int get_turbo_status(void);
#if defined(__MTK_SLT_)
/* extern int mt_ptp_idle_can_enter(void); */
extern void ptp_init01_ptp(int id);
extern int ptp_isr(void);
#endif


#endif
