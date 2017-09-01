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

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include "mt-plat/mtk_thermal_monitor.h"
#include "mach/mt_thermal.h"
#include "mt-plat/mtk_thermal_platform.h"
#include <mach/mt_clkmgr.h>
#include <mt_ptp.h>
#include <mach/wd_api.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <tscpu_settings.h>
#include <mt-plat/aee.h>
#include <linux/uidgid.h>
#include <ap_thermal_limit.h>
#ifdef ATM_USES_PPM
#include "mach/mt_ppm_api.h"
#else
#include "mt_cpufreq.h"
#endif

#ifdef FAST_RESPONSE_ATM
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#endif
/*=============================================================
 *Local variable definition
 *=============================================================*/
static int print_cunt;
static int adaptive_limit[5][2];
static struct apthermolmt_user ap_atm;
static char *ap_atm_log = "ap_atm";

static kuid_t uid = KUIDT_INIT(0);
static kgid_t gid = KGIDT_INIT(1000);
unsigned int adaptive_cpu_power_limit = 0x7FFFFFFF;
unsigned int adaptive_gpu_power_limit = 0x7FFFFFFF;
static unsigned int prv_adp_cpu_pwr_lim;
static unsigned int prv_adp_gpu_pwr_lim;
unsigned int gv_cpu_power_limit = 0x7FFFFFFF;
unsigned int gv_gpu_power_limit = 0x7FFFFFFF;
#if CPT_ADAPTIVE_AP_COOLER
static int TARGET_TJ = 65000;
static int cpu_target_tj = 65000;
static int cpu_target_offset = 10000;
static int TARGET_TJ_HIGH = 66000;
static int TARGET_TJ_LOW = 64000;
static int PACKAGE_THETA_JA_RISE = 10;
static int PACKAGE_THETA_JA_FALL = 10;
static int MINIMUM_CPU_POWER = 500;
static int MAXIMUM_CPU_POWER = 1240;
static int MINIMUM_GPU_POWER = 676;
static int MAXIMUM_GPU_POWER = 676;
static int MINIMUM_TOTAL_POWER = 500 + 676;
static int MAXIMUM_TOTAL_POWER = 1240 + 676;
static int FIRST_STEP_TOTAL_POWER_BUDGET = 1750;

/* 1. MINIMUM_BUDGET_CHANGE = 0 ==> thermal equilibrium maybe at higher than TARGET_TJ_HIGH */
/* 2. Set MINIMUM_BUDGET_CHANGE > 0 if to keep Tj at TARGET_TJ */
static int MINIMUM_BUDGET_CHANGE = 50;
static int g_total_power;
#endif

#if CPT_ADAPTIVE_AP_COOLER
static struct thermal_cooling_device *cl_dev_adp_cpu[MAX_CPT_ADAPTIVE_COOLERS] = { NULL };
static unsigned int cl_dev_adp_cpu_state[MAX_CPT_ADAPTIVE_COOLERS] = { 0 };
#if defined(CLATM_SET_INIT_CFG)
int TARGET_TJS[MAX_CPT_ADAPTIVE_COOLERS] = {
	CLATM_INIT_CFG_0_TARGET_TJ, CLATM_INIT_CFG_1_TARGET_TJ, CLATM_INIT_CFG_2_TARGET_TJ };
#else
int TARGET_TJS[MAX_CPT_ADAPTIVE_COOLERS] = { 85000, 0 };
#endif

static unsigned int cl_dev_adp_cpu_state_active;
#endif	/* end of CPT_ADAPTIVE_AP_COOLER */

#if CPT_ADAPTIVE_AP_COOLER
char *adaptive_cooler_name = "cpu_adaptive_";

#if defined(CLATM_SET_INIT_CFG)
static int FIRST_STEP_TOTAL_POWER_BUDGETS[MAX_CPT_ADAPTIVE_COOLERS] = {
	CLATM_INIT_CFG_0_FIRST_STEP, CLATM_INIT_CFG_1_FIRST_STEP, CLATM_INIT_CFG_2_FIRST_STEP };
static int PACKAGE_THETA_JA_RISES[MAX_CPT_ADAPTIVE_COOLERS] = {
	CLATM_INIT_CFG_0_THETA_RISE, CLATM_INIT_CFG_1_THETA_RISE, CLATM_INIT_CFG_2_THETA_RISE };
static int PACKAGE_THETA_JA_FALLS[MAX_CPT_ADAPTIVE_COOLERS] = {
	CLATM_INIT_CFG_0_THETA_FALL, CLATM_INIT_CFG_1_THETA_FALL, CLATM_INIT_CFG_2_THETA_FALL };
static int MINIMUM_BUDGET_CHANGES[MAX_CPT_ADAPTIVE_COOLERS] = {
	CLATM_INIT_CFG_0_MIN_BUDGET_CHG,
	CLATM_INIT_CFG_1_MIN_BUDGET_CHG,
	CLATM_INIT_CFG_2_MIN_BUDGET_CHG };
static int MINIMUM_CPU_POWERS[MAX_CPT_ADAPTIVE_COOLERS] = {
	CLATM_INIT_CFG_0_MIN_CPU_PWR, CLATM_INIT_CFG_1_MIN_CPU_PWR, CLATM_INIT_CFG_2_MIN_CPU_PWR };
static int MAXIMUM_CPU_POWERS[MAX_CPT_ADAPTIVE_COOLERS] = {
	CLATM_INIT_CFG_0_MAX_CPU_PWR, CLATM_INIT_CFG_1_MAX_CPU_PWR, CLATM_INIT_CFG_2_MAX_CPU_PWR };
static int MINIMUM_GPU_POWERS[MAX_CPT_ADAPTIVE_COOLERS] = {
	CLATM_INIT_CFG_0_MIN_GPU_PWR, CLATM_INIT_CFG_1_MIN_GPU_PWR, CLATM_INIT_CFG_2_MIN_GPU_PWR };
static int MAXIMUM_GPU_POWERS[MAX_CPT_ADAPTIVE_COOLERS] = {
	CLATM_INIT_CFG_0_MAX_GPU_PWR, CLATM_INIT_CFG_1_MAX_GPU_PWR, CLATM_INIT_CFG_2_MAX_GPU_PWR };
#else
static int FIRST_STEP_TOTAL_POWER_BUDGETS[MAX_CPT_ADAPTIVE_COOLERS] = { 3300, 0 };
static int PACKAGE_THETA_JA_RISES[MAX_CPT_ADAPTIVE_COOLERS] = { 35, 0 };
static int PACKAGE_THETA_JA_FALLS[MAX_CPT_ADAPTIVE_COOLERS] = { 25, 0 };
static int MINIMUM_BUDGET_CHANGES[MAX_CPT_ADAPTIVE_COOLERS] = { 50, 0 };
static int MINIMUM_CPU_POWERS[MAX_CPT_ADAPTIVE_COOLERS] = { 1200, 0 };
static int MAXIMUM_CPU_POWERS[MAX_CPT_ADAPTIVE_COOLERS] = { 4400, 0 };
static int MINIMUM_GPU_POWERS[MAX_CPT_ADAPTIVE_COOLERS] = { 350, 0 };
static int MAXIMUM_GPU_POWERS[MAX_CPT_ADAPTIVE_COOLERS] = { 960, 0 };
#endif

#if defined(CLATM_SET_INIT_CFG)
static int active_adp_cooler = CLATM_INIT_CFG_ACTIVE_ATM_COOLER;
#else
static int active_adp_cooler;
#endif

static int GPU_L_H_TRIP = 80, GPU_L_L_TRIP = 40;

/* tscpu_atm
   0: ATMv1 (default)
   1: ATMv2 (FTL)
   2: CPU_GPU_Weight ATM v2
   3: Precise Power Budgeting + Hybrid Power Budgeting
*/
#ifdef PHPB_DEFAULT_ON
static int tscpu_atm = 3;
#else
static int tscpu_atm = 1;
#endif
static int tt_ratio_high_rise = 1;
static int tt_ratio_high_fall = 1;
static int tt_ratio_low_rise = 1;
static int tt_ratio_low_fall = 1;
static int tp_ratio_high_rise = 1;
static int tp_ratio_high_fall;
static int tp_ratio_low_rise;
static int tp_ratio_low_fall;
/* static int cpu_loading = 0; */

static int (*_adaptive_power_calc)(long prev_temp, long curr_temp, unsigned int gpu_loading);

#if PRECISE_HYBRID_POWER_BUDGET
/* initial value: assume 1 degreeC for temp. <=> 1 unit for total_power(0~100) */
struct phpb_param {
	int tt, tp;
	char type[8];
};

enum {
	PHPB_PARAM_CPU = 0,
	PHPB_PARAM_GPU,
	NR_PHPB_PARAMS
};

static struct phpb_param phpb_params[NR_PHPB_PARAMS];
static const int phpb_theta_min = 1;
static int phpb_theta_max = 4;
static int tj_stable_range = 1000;

#if 0
#define MAX_GPU_POWER_SMA_LEN	(32)
static unsigned int gpu_power_sma_len = 1;
static unsigned int gpu_power_history[MAX_GPU_POWER_SMA_LEN];
static unsigned int gpu_power_history_idx;
#endif
#endif

#if THERMAL_HEADROOM
static int p_Tpcb_correlation;
static int Tpcb_trip_point;
static int thp_max_cpu_power;
static int thp_p_tj_correlation;
static int thp_threshold_tj;
#endif

#if CONTINUOUS_TM
#if defined(CLATM_SET_INIT_CFG)
static int ctm_on = CLATM_INIT_CFG_CATM; /* 2: cATM+, 1: cATMv1, 0: off */
#else
static int ctm_on = -1; /* 2: cATM+, 1: cATMv1, 0: off */
#endif
static int MAX_TARGET_TJ = -1;
static int STEADY_TARGET_TJ = -1;
static int TRIP_TPCB = -1;
static int STEADY_TARGET_TPCB = -1;
static int MAX_EXIT_TJ = -1;
static int STEADY_EXIT_TJ = -1;
static int COEF_AE = -1;
static int COEF_BE = -1;
static int COEF_AX = -1;
static int COEF_BX = -1;
/* static int current_TTJ = -1; */
static int current_ETJ = -1;

/* +++ cATM+ parameters +++ */
/* slope of base_ttj, automatically calculated */
static int K_TT = 4000;
#define MAX_K_SUM_TT	(K_TT * 10)
/* for ATM polling delay 50ms, increase this based on polling delay */
static int K_SUM_TT_LOW = 10;
/* for ATM polling delay 50ms, increase this based on polling delay */
static int K_SUM_TT_HIGH = 10;
/* clamp sum_tt (err_integral) between MIN_SUM_TT ~ MAX_SUM_TT, automatically calculated */
static int MIN_SUM_TT = -800000;
static int MAX_SUM_TT = 800000;
static int MIN_TTJ = 65000;
static int CATMP_STEADY_TTJ_DELTA = 10000; /* magic number decided by experience */
/* --- cATM+ parameters --- */
#endif
#endif	/* end of CPT_ADAPTIVE_AP_COOLER */

#ifdef FAST_RESPONSE_ATM
#define TS_MS_TO_NS(x) (x * 1000 * 1000)
static struct hrtimer atm_hrtimer;

static unsigned long atm_hrtimer_polling_delay = TS_MS_TO_NS(20); /* default 20ms*/

static int atm_curr_maxtj;
static int atm_prev_maxtj;
static int krtatm_curr_maxtj;
static int krtatm_prev_maxtj;
static struct task_struct *krtatm_thread_handle;
#endif

/*=============================================================
 *Local function prototype
 *=============================================================*/
static void set_adaptive_gpu_power_limit(unsigned int limit);
/*=============================================================
 *Weak functions
 *=============================================================*/
#if 0
#ifdef ATM_USES_PPM
void __attribute__ ((weak))
mt_ppm_cpu_thermal_protect(unsigned int limited_power)
{
	pr_err("E_WF: %s doesn't exist\n", __func__);
}
#else
void __attribute__ ((weak))
mt_cpufreq_thermal_protect(unsigned int limited_power)
{
	pr_err("E_WF: %s doesn't exist\n", __func__);
}
#endif
#endif

bool __attribute__((weak))
mtk_get_gpu_loading(unsigned int *pLoading)
{
	pr_err("E_WF: %s doesn't exist\n", __func__);
	return 0;
}
unsigned int  __attribute__((weak))
mt_gpufreq_get_min_power(void)
{
	pr_err("E_WF: %s doesn't exist\n", __func__);
	return 0;
}
unsigned int  __attribute__((weak))
mt_gpufreq_get_max_power(void)
{
	pr_err("E_WF: %s doesn't exist\n", __func__);
	return 0;
}
/*=============================================================*/

/*
   static int step0_mask[11] = {1,1,1,1,1,1,1,1,1,1,1};
   static int step1_mask[11] = {0,1,1,1,1,1,1,1,1,1,1};
   static int step2_mask[11] = {0,0,1,1,1,1,1,1,1,1,1};
   static int step3_mask[11] = {0,0,0,1,1,1,1,1,1,1,1};
   static int step4_mask[11] = {0,0,0,0,1,1,1,1,1,1,1};
   static int step5_mask[11] = {0,0,0,0,0,1,1,1,1,1,1};
   static int step6_mask[11] = {0,0,0,0,0,0,1,1,1,1,1};
   static int step7_mask[11] = {0,0,0,0,0,0,0,1,1,1,1};
   static int step8_mask[11] = {0,0,0,0,0,0,0,0,1,1,1};
   static int step9_mask[11] = {0,0,0,0,0,0,0,0,0,1,1};
   static int step10_mask[11]= {0,0,0,0,0,0,0,0,0,0,1};
 */

int tsatm_thermal_get_catm_type(void)
{
	tscpu_dprintk("tsatm_thermal_get_catm_type ctm_on = %d\n", ctm_on);
	return ctm_on;
}

int mtk_thermal_get_tpcb_target(void)
{
	return STEADY_TARGET_TPCB;
}
EXPORT_SYMBOL(mtk_thermal_get_tpcb_target);

/**
 * TODO: What's the diff from get_cpu_target_tj?
 */
int get_target_tj(void)
{
	return TARGET_TJ;
}

static void set_adaptive_cpu_power_limit(unsigned int limit)
{
	prv_adp_cpu_pwr_lim = adaptive_cpu_power_limit;
	adaptive_cpu_power_limit = (limit != 0) ? limit : 0x7FFFFFFF;

	if (prv_adp_cpu_pwr_lim != adaptive_cpu_power_limit) {
		/* print debug log */
		adaptive_limit[print_cunt][0] =
			(int) (adaptive_cpu_power_limit != 0x7FFFFFFF) ? adaptive_cpu_power_limit : 0;
#ifdef FAST_RESPONSE_ATM
		adaptive_limit[print_cunt][1] = krtatm_curr_maxtj;
#else
		adaptive_limit[print_cunt][1] = tscpu_get_curr_temp();
#endif
		print_cunt++;
		if (print_cunt == 5) {
			tscpu_warn("%s %d T=%d, %d T=%d, %d T=%d, %d T=%d, %d T=%d\n",
				__func__,
				adaptive_limit[4][0], adaptive_limit[4][1],
				adaptive_limit[3][0], adaptive_limit[3][1],
				adaptive_limit[2][0], adaptive_limit[2][1],
				adaptive_limit[1][0], adaptive_limit[1][1],
				adaptive_limit[0][0], adaptive_limit[0][1]);

			print_cunt = 0;
		} else {
#ifdef FAST_RESPONSE_ATM
			if ((0x7FFFFFFF != prv_adp_cpu_pwr_lim) &&
				((adaptive_cpu_power_limit + 1000) < prv_adp_cpu_pwr_lim))
				tscpu_warn("%s Big delta power %u curr_T=%d, %u prev_T=%d\n", __func__,
					adaptive_cpu_power_limit, krtatm_curr_maxtj,
					prv_adp_cpu_pwr_lim, krtatm_prev_maxtj);
#endif
		}

		apthermolmt_set_cpu_power_limit(&ap_atm, adaptive_cpu_power_limit);
	}
}

static void set_adaptive_gpu_power_limit(unsigned int limit)
{
	prv_adp_gpu_pwr_lim = adaptive_gpu_power_limit;
	adaptive_gpu_power_limit = (limit != 0) ? limit : 0x7FFFFFFF;
	if (prv_adp_gpu_pwr_lim != adaptive_gpu_power_limit) {
		tscpu_dprintk("%s %d\n", __func__,
		     (adaptive_gpu_power_limit != 0x7FFFFFFF) ? adaptive_gpu_power_limit : 0);

		apthermolmt_set_gpu_power_limit(&ap_atm, adaptive_gpu_power_limit);
	}
}

#if CPT_ADAPTIVE_AP_COOLER
int is_cpu_power_unlimit(void)
{
	return (g_total_power == 0 || g_total_power >= MAXIMUM_TOTAL_POWER) ? 1 : 0;
}
EXPORT_SYMBOL(is_cpu_power_unlimit);

int is_cpu_power_min(void)
{
	return (g_total_power <= MINIMUM_TOTAL_POWER) ? 1 : 0;
}
EXPORT_SYMBOL(is_cpu_power_min);

/**
 * TODO: What's the diff from get_target_tj?
 */
int get_cpu_target_tj(void)
{
	return cpu_target_tj;
}
EXPORT_SYMBOL(get_cpu_target_tj);

int get_cpu_target_offset(void)
{
	return cpu_target_offset;
}
EXPORT_SYMBOL(get_cpu_target_offset);

/*add for DLPT*/
int tscpu_get_min_cpu_pwr(void)
{
	return MINIMUM_CPU_POWER;
}
EXPORT_SYMBOL(tscpu_get_min_cpu_pwr);

int tscpu_get_min_gpu_pwr(void)
{
	return MINIMUM_GPU_POWER;
}
EXPORT_SYMBOL(tscpu_get_min_gpu_pwr);

#if CONTINUOUS_TM
/**
 * @brief update cATM+ ttj control loop parameters
 * everytime related parameters are changed, we need to recalculate.
 * from thermal config: MAX_TARGET_TJ, STEADY_TARGET_TJ, MIN_TTJ, TRIP_TPCB...etc
 * cATM+'s parameters: K_SUM_TT_HIGH, K_SUM_TT_LOW
 */

struct CATM_T thermal_atm_t;

static void catmplus_update_params(void)
{

	int ret = 0;

	thermal_atm_t.t_catm_par.CATM_ON = ctm_on;
	thermal_atm_t.t_catm_par.K_TT = K_TT;
	thermal_atm_t.t_catm_par.K_SUM_TT_LOW = K_SUM_TT_LOW;
	thermal_atm_t.t_catm_par.K_SUM_TT_HIGH = K_SUM_TT_HIGH;
	thermal_atm_t.t_catm_par.MIN_SUM_TT = MIN_SUM_TT;
	thermal_atm_t.t_catm_par.MAX_SUM_TT = MAX_SUM_TT;
	thermal_atm_t.t_catm_par.MIN_TTJ = MIN_TTJ;
	thermal_atm_t.t_catm_par.CATMP_STEADY_TTJ_DELTA = CATMP_STEADY_TTJ_DELTA;


	thermal_atm_t.t_continuetm_par.STEADY_TARGET_TJ = STEADY_TARGET_TJ;
	thermal_atm_t.t_continuetm_par.MAX_TARGET_TJ = MAX_TARGET_TJ;
	thermal_atm_t.t_continuetm_par.TRIP_TPCB  =  TRIP_TPCB;
	thermal_atm_t.t_continuetm_par.STEADY_TARGET_TPCB  =  STEADY_TARGET_TPCB;

	ret = wakeup_ta_algo(TA_CATMPLUS);
	/*tscpu_warn("catmplus_update_params : ret %d\n" , ret);*/
}

#endif

#if PRECISE_HYBRID_POWER_BUDGET
static int _get_current_gpu_power(void)
{
	unsigned int cur_gpu_freq = mt_gpufreq_get_cur_freq();
	unsigned int cur_gpu_power = 0;
	int i = 0;

	if (mtk_gpu_power != NULL) {
		for (; i < Num_of_GPU_OPP; i++)
			if (mtk_gpu_power[i].gpufreq_khz == cur_gpu_freq)
				cur_gpu_power = mtk_gpu_power[i].gpufreq_power;
	}

	return (int) cur_gpu_power;
}

#if 0
static void reset_gpu_power_history(void)
{
	int i = 0;
	/*	Be careful when this can be invoked and error values. */
	unsigned int max_gpu_power = mt_gpufreq_get_max_power();

	if (gpu_power_sma_len > MAX_GPU_POWER_SMA_LEN)
		gpu_power_sma_len = MAX_GPU_POWER_SMA_LEN;

	for (i = 0; i < MAX_GPU_POWER_SMA_LEN; i++)
		gpu_power_history[i] = max_gpu_power;

	gpu_power_history_idx = 0;
}

/* we'll calculate SMA for gpu power, but the output will still be aligned to OPP */
static int adjust_gpu_power(int power)
{
	int i, total = 0, sma_power;

	/*	FIXME: debug only, this check should be moved to some setter functions.
		or deleted if we don't want sma_len is changeable during runtime */
	/*
	if (gpu_power_sma_len > MAX_GPU_POWER_SMA_LEN)
		gpu_power_sma_len = MAX_GPU_POWER_SMA_LEN;
	*/

	if (power == 0)
		power = MAXIMUM_GPU_POWER;

	gpu_power_history[gpu_power_history_idx] = power;
	for (i = 0; i < gpu_power_sma_len; i++)
		total += gpu_power_history[i];

	gpu_power_history_idx = (gpu_power_history_idx + 1) % gpu_power_sma_len;
	sma_power = total / gpu_power_sma_len;

	for (i = 0; i < Num_of_GPU_OPP; i++) {
		if (mtk_gpu_power[i].gpufreq_power <= sma_power)
			break;
	}

	if (i >= Num_of_GPU_OPP)
		power = MINIMUM_GPU_POWER;
	else
		power = MAX(MINIMUM_GPU_POWER, (int)mtk_gpu_power[i].gpufreq_power);

	return power;
}
#endif
#endif

static int P_adaptive(int total_power, unsigned int gpu_loading)
{
	/*
	Here the gpu_power is the gpu power limit for the next interval,
	not exactly what gpu power selected by GPU DVFS
	But the ground rule is real gpu power should always under gpu_power for the same time interval
	*/
	static int cpu_power = 0, gpu_power;
	static int last_cpu_power = 0, last_gpu_power;

	last_cpu_power = cpu_power;
	last_gpu_power = gpu_power;
	g_total_power = total_power;

	if (total_power == 0) {
		cpu_power = gpu_power = 0;
#if THERMAL_HEADROOM
		if (thp_max_cpu_power != 0)
			set_adaptive_cpu_power_limit((unsigned int)
						     MAX(thp_max_cpu_power, MINIMUM_CPU_POWER));
		else
			set_adaptive_cpu_power_limit(0);
#else
		set_adaptive_cpu_power_limit(0);
#endif
		set_adaptive_gpu_power_limit(0);
#if (CONFIG_THERMAL_AEE_RR_REC == 1)
			aee_rr_rec_thermal_ATM_status(ATM_DONE);
#endif
		return 0;
	}

	if (total_power <= MINIMUM_TOTAL_POWER) {
		cpu_power = MINIMUM_CPU_POWER;
		gpu_power = MINIMUM_GPU_POWER;
	} else if (total_power >= MAXIMUM_TOTAL_POWER) {
		cpu_power = MAXIMUM_CPU_POWER;
		gpu_power = MAXIMUM_GPU_POWER;
	} else {
		if (mtk_gpu_power != NULL) {
			int max_allowed_gpu_power =
			MIN((total_power - MINIMUM_CPU_POWER), MAXIMUM_GPU_POWER);
			int max_gpu_power = (int) mt_gpufreq_get_max_power();
			int highest_possible_gpu_power = (max_allowed_gpu_power > max_gpu_power) ?
			(max_gpu_power+1) : -1;
			/* int highest_possible_gpu_power_idx = 0; */
			int i = 0;

			unsigned int cur_gpu_freq = mt_gpufreq_get_cur_freq();
			/* int cur_idx = 0; */
			unsigned int cur_gpu_power = 0;
			unsigned int next_lower_gpu_power = 0;

			/* get GPU highest possible power and index and current power and index and next lower power */
			for (; i < Num_of_GPU_OPP; i++) {
				if ((mtk_gpu_power[i].gpufreq_power <= max_allowed_gpu_power) &&
				(-1 == highest_possible_gpu_power)) {
					/* choose OPP with power "<=" limit */
					highest_possible_gpu_power = mtk_gpu_power[i].gpufreq_power + 1;
					/* highest_possible_gpu_power_idx = i; */
			}

			if (mtk_gpu_power[i].gpufreq_khz == cur_gpu_freq) {
				next_lower_gpu_power = cur_gpu_power =
				    (mtk_gpu_power[i].gpufreq_power + 1); /* choose OPP with power "<=" limit */
				/* cur_idx = i; */

				if ((i != Num_of_GPU_OPP - 1)
				    && (mtk_gpu_power[i + 1].gpufreq_power >= MINIMUM_GPU_POWER)) {
					/* choose OPP with power "<=" limit */
					next_lower_gpu_power = mtk_gpu_power[i + 1].gpufreq_power + 1;
				}
			}
		}

		/* decide GPU power limit by loading */
		if (gpu_loading > GPU_L_H_TRIP) {
			gpu_power = highest_possible_gpu_power;
		} else if (gpu_loading <= GPU_L_L_TRIP) {
			gpu_power = MIN(next_lower_gpu_power, highest_possible_gpu_power);
			gpu_power = MAX(gpu_power, MINIMUM_GPU_POWER);
		} else {
			gpu_power = MIN(highest_possible_gpu_power, cur_gpu_power);
			gpu_power = MAX(gpu_power, MINIMUM_GPU_POWER);
		}
		}  else {
			gpu_power = 0;
		}

		cpu_power = MIN((total_power - gpu_power), MAXIMUM_CPU_POWER);
	}

#if 0
	/* TODO: check if this segment can be used in original design
       GPU SMA */
	if ((gpu_power_sma_len > 1) && (tscpu_atm == 3)) {
		total_power = gpu_power + cpu_power;
		gpu_power = adjust_gpu_power(gpu_power);
		cpu_power = total_power - gpu_power;
	}
#endif

	if (cpu_power != last_cpu_power)
		set_adaptive_cpu_power_limit(cpu_power);

	if ((gpu_power != last_gpu_power) && (mtk_gpu_power != NULL)) {
		/* Work-around for unsync GPU power table problem 1. */
		if (gpu_power > mtk_gpu_power[0].gpufreq_power)
			set_adaptive_gpu_power_limit(0);
		else
			set_adaptive_gpu_power_limit(gpu_power);
	}

#ifdef FAST_RESPONSE_ATM
	if (atm_curr_maxtj >= 100000 || (atm_curr_maxtj - atm_prev_maxtj >= 15000))
		tscpu_warn("%s total %d, cpu %d, gpu %d\n", __func__, total_power, cpu_power, gpu_power);
	else
#endif
		tscpu_dprintk("%s cpu %d, gpu %d\n", __func__, cpu_power, gpu_power);

#if (CONFIG_THERMAL_AEE_RR_REC == 1)
	aee_rr_rec_thermal_ATM_status(ATM_DONE);
#endif
	return 0;
}

#if PRECISE_HYBRID_POWER_BUDGET
static int __phpb_dynamic_theta(int max_theta)
{
	int theta;
	/*	temp solution as CATM
		FIXME: API? how to get tj trip? */
	int tj_trip = TARGET_TJS[0];

	if (TARGET_TJ <= tj_trip)
		theta = max_theta;
	else if (TARGET_TJ >= MAX_TARGET_TJ)
		theta = phpb_theta_min;
	else {
		theta = max_theta - (TARGET_TJ - tj_trip) *
			(max_theta - phpb_theta_min) /
			(MAX_TARGET_TJ - tj_trip);
	}

	return theta;
}

/**
 * TODO: target_tj is adjusted by catmv2, therefore dynamic_theta would not
 * changed frequently.
 */
static int __phpb_calc_delta(int curr_temp, int prev_temp, int phpb_param_idx)
{
	struct phpb_param *p = &phpb_params[phpb_param_idx];
	int tt = TARGET_TJ - curr_temp;
	int tp = prev_temp - curr_temp;
	int delta_power = 0, delta_power_tt, delta_power_tp;

	/* *2 is to cover Tj jump betwen [TTJ-tj_stable_range, TTJ+tj_stable_range] */
	if ((abs(tt) > tj_stable_range) || (abs(tp) > (tj_stable_range * 2))) {
		delta_power_tt = tt / p->tt;
		delta_power_tp = tp / p->tp;
		/* When Tj is rising, double power cut. */
		if (delta_power_tp < 0)
			delta_power_tp *= 2;
		delta_power = (delta_power_tt + delta_power_tp) /
			      __phpb_dynamic_theta(phpb_theta_max);
	}

	return delta_power;
}

static int phpb_calc_delta(int curr_temp, int prev_temp)
{
	int delta, param_idx;

	if (tscpu_curr_cpu_temp >= tscpu_curr_gpu_temp)
		param_idx = PHPB_PARAM_CPU;
	else
		param_idx = PHPB_PARAM_GPU;

	delta = __phpb_calc_delta(curr_temp, prev_temp, param_idx);

	return delta;
}

/* calculated total power based on current opp */
static int get_total_curr_power(void)
{
	int cpu_power = 0, gpu_power = 0;

#ifdef ATM_USES_PPM
	cpu_power = (int) mt_ppm_thermal_get_cur_power() + 1; /* choose OPP with power "<=" limit */
#else
	/* maybe you should disable PRECISE_HYBRID_POWER_BUDGET if current cpu power unavailable .*/
	cpu_power = 0;
#endif
	/* avoid idle power too small to hurt another unit performance */
	if (cpu_power < MINIMUM_CPU_POWER)
		cpu_power = MINIMUM_CPU_POWER;

	if (cpu_power > MAXIMUM_CPU_POWER) {
		tscpu_warn("%s cpu_power %d > MAXIMUM_CPU_POWER %d\n",
						__func__, cpu_power, MAXIMUM_CPU_POWER);
		cpu_power = MAXIMUM_CPU_POWER;
	}
	gpu_power = _get_current_gpu_power() + 1; /* choose OPP with power "<=" limit */
	if (gpu_power < MINIMUM_GPU_POWER)
		gpu_power = MINIMUM_GPU_POWER;

	if (gpu_power > MAXIMUM_GPU_POWER) {
		tscpu_warn("%s gpu_power %d > MAXIMUM_GPU_POWER %d\n",
						__func__, gpu_power, MAXIMUM_GPU_POWER);
		gpu_power = MAXIMUM_GPU_POWER;
	}

	return cpu_power + gpu_power;
}

static int phpb_calc_total(int prev_total_power, long curr_temp, long prev_temp)
{
	/* total_power is new power limit, which curr_power is current power
	 * calculated based on current opp */
	int delta_power, total_power, curr_power;
	int tt = TARGET_TJ - curr_temp;
	int tp = prev_temp - curr_temp;

	delta_power = phpb_calc_delta(curr_temp, prev_temp);
	if (delta_power == 0)
		return prev_total_power;

	curr_power = get_total_curr_power();
	/* In some conditions, we will consider using current request power to
	 * avoid giving unlimit power budget.
	 * Temp. rising is large,  requset power is of course less than power
	 * limit (but it sometime goes over...)
	 */
	if (curr_power < prev_total_power &&
	    ((-tt) >= tj_stable_range * 2 || (-tp) >= tj_stable_range * 4)) {
		tscpu_dprintk("%s prev_temp %ld, curr_temp %ld, curr %d, delta %d\n", __func__,
				prev_temp, curr_temp, curr_power, delta_power);
		total_power = curr_power + delta_power;
	} else {
		tscpu_dprintk("%s prev_temp %ld, curr_temp %ld, prev %d, delta %d\n", __func__,
				prev_temp, curr_temp, prev_total_power, delta_power);
		total_power = prev_total_power + delta_power;
	}

	total_power = clamp(total_power, MINIMUM_TOTAL_POWER, MAXIMUM_TOTAL_POWER);

	return total_power;
}

static int _adaptive_power_ppb(long prev_temp, long curr_temp, unsigned int gpu_loading)
{
	static int triggered, total_power;
	int delta_power = 0;

	if (cl_dev_adp_cpu_state_active == 1) {
		tscpu_dprintk("%s %d %d %d %d %d %d %d\n", __func__,
			PACKAGE_THETA_JA_RISE, PACKAGE_THETA_JA_FALL, MINIMUM_BUDGET_CHANGE,
			MINIMUM_CPU_POWER, MAXIMUM_CPU_POWER, MINIMUM_GPU_POWER, MAXIMUM_GPU_POWER);

		/* Check if it is triggered */
		if (!triggered) {
			triggered = 1;
			total_power = phpb_calc_total(get_total_curr_power(), curr_temp, prev_temp);
			if (curr_temp >= 100000)
				tscpu_warn("%s triggered:0->1 Tp %ld, Tc %ld, TARGET_TJ %d, Pt %d\n",
								__func__, prev_temp, curr_temp, TARGET_TJ, total_power);
			else
				tscpu_dprintk("%s triggered:0->1 Tp %ld, Tc %ld, TARGET_TJ %d, Pt %d\n",
								__func__, prev_temp, curr_temp, TARGET_TJ, total_power);
			return P_adaptive(total_power, gpu_loading);
		}

		/* Adjust total power budget if necessary */
		total_power = phpb_calc_total(total_power, curr_temp, prev_temp);
		/*	TODO: delta_power is not changed but printed. */
		if (curr_temp >= 100000)
			tscpu_warn("%s TARGET_TJ %d, delta_power %d, total_power %d\n",
							__func__, TARGET_TJ, delta_power, total_power);
		else
			tscpu_dprintk("%s TARGET_TJ %d, delta_power %d, total_power %d\n",
							__func__, TARGET_TJ, delta_power, total_power);
		tscpu_dprintk("%s Tp %ld, Tc %ld, Pt %d\n", __func__, prev_temp, curr_temp, total_power);
		return P_adaptive(total_power, gpu_loading);
		/* end of cl_dev_adp_cpu_state_active == 1 */
	} else {
		if (triggered) {
			triggered = 0;
			tscpu_dprintk("%s Tp %ld, Tc %ld, Pt %d\n", __func__, prev_temp, curr_temp, total_power);
			return P_adaptive(0, 0);
#if THERMAL_HEADROOM
		} else {
			if (thp_max_cpu_power != 0)
				set_adaptive_cpu_power_limit((unsigned int) MAX(thp_max_cpu_power, MINIMUM_CPU_POWER));
			else
				set_adaptive_cpu_power_limit(0);
		}
#else
		}
#endif
		/* reset_gpu_power_history(); */
	}

	return 0;
}
#endif

static int _adaptive_power(long prev_temp, long curr_temp, unsigned int gpu_loading)
{
	static int triggered = 0, total_power;
	int delta_power = 0;

	if (cl_dev_adp_cpu_state_active == 1) {
		tscpu_dprintk("%s %d %d %d %d %d %d %d %d\n", __func__,
			      FIRST_STEP_TOTAL_POWER_BUDGET, PACKAGE_THETA_JA_RISE,
			      PACKAGE_THETA_JA_FALL, MINIMUM_BUDGET_CHANGE, MINIMUM_CPU_POWER,
			      MAXIMUM_CPU_POWER, MINIMUM_GPU_POWER, MAXIMUM_GPU_POWER);

		/* Check if it is triggered */
		if (!triggered) {
			if (curr_temp < TARGET_TJ)
				return 0;

			triggered = 1;
			switch (tscpu_atm) {
			case 1:	/* FTL ATM v2 */
			case 2:	/* CPU_GPU_Weight ATM v2 */
#if MTKTSCPU_FAST_POLLING
				total_power =
					FIRST_STEP_TOTAL_POWER_BUDGET -
					((curr_temp - TARGET_TJ) * tt_ratio_high_rise +
					(curr_temp - prev_temp) * tp_ratio_high_rise) /
					(PACKAGE_THETA_JA_RISE * tscpu_cur_fp_factor);
#else
				total_power =
					FIRST_STEP_TOTAL_POWER_BUDGET -
					((curr_temp - TARGET_TJ) * tt_ratio_high_rise +
					(curr_temp - prev_temp) * tp_ratio_high_rise) /
					PACKAGE_THETA_JA_RISE;
#endif
				break;
			case 0:
			default:	/* ATM v1 */
				total_power = FIRST_STEP_TOTAL_POWER_BUDGET;
			}
			tscpu_dprintk("%s Tp %ld, Tc %ld, Pt %d\n", __func__, prev_temp, curr_temp, total_power);
			return P_adaptive(total_power, gpu_loading);
		}

		/* Adjust total power budget if necessary */
		switch (tscpu_atm) {
		case 1:	/* FTL ATM v2 */
		case 2:	/* CPU_GPU_Weight ATM v2 */
			if ((curr_temp >= TARGET_TJ_HIGH) && (curr_temp > prev_temp)) {
#if MTKTSCPU_FAST_POLLING
				total_power -=
				    MAX(((curr_temp - TARGET_TJ) * tt_ratio_high_rise +
					 (curr_temp -
					  prev_temp) * tp_ratio_high_rise) /
					(PACKAGE_THETA_JA_RISE * tscpu_cur_fp_factor),
					MINIMUM_BUDGET_CHANGE);
#else
				total_power -=
				    MAX(((curr_temp - TARGET_TJ) * tt_ratio_high_rise +
					 (curr_temp -
					  prev_temp) * tp_ratio_high_rise) / PACKAGE_THETA_JA_RISE,
					MINIMUM_BUDGET_CHANGE);
#endif
			} else if ((curr_temp >= TARGET_TJ_HIGH) && (curr_temp <= prev_temp)) {
#if MTKTSCPU_FAST_POLLING
				total_power -=
				    MAX(((curr_temp - TARGET_TJ) * tt_ratio_high_fall -
					 (prev_temp -
					  curr_temp) * tp_ratio_high_fall) /
					(PACKAGE_THETA_JA_FALL * tscpu_cur_fp_factor),
					MINIMUM_BUDGET_CHANGE);
#else
				total_power -=
				    MAX(((curr_temp - TARGET_TJ) * tt_ratio_high_fall -
					 (prev_temp -
					  curr_temp) * tp_ratio_high_fall) / PACKAGE_THETA_JA_FALL,
					MINIMUM_BUDGET_CHANGE);
#endif
			} else if ((curr_temp <= TARGET_TJ_LOW) && (curr_temp > prev_temp)) {
#if MTKTSCPU_FAST_POLLING
				total_power +=
				    MAX(((TARGET_TJ - curr_temp) * tt_ratio_low_rise -
					 (curr_temp -
					  prev_temp) * tp_ratio_low_rise) / (PACKAGE_THETA_JA_RISE *
									     tscpu_cur_fp_factor),
					MINIMUM_BUDGET_CHANGE);
#else
				total_power +=
				    MAX(((TARGET_TJ - curr_temp) * tt_ratio_low_rise -
					 (curr_temp -
					  prev_temp) * tp_ratio_low_rise) / PACKAGE_THETA_JA_RISE,
					MINIMUM_BUDGET_CHANGE);
#endif
			} else if ((curr_temp <= TARGET_TJ_LOW) && (curr_temp <= prev_temp)) {
#if MTKTSCPU_FAST_POLLING
				total_power +=
				    MAX(((TARGET_TJ - curr_temp) * tt_ratio_low_fall +
					 (prev_temp -
					  curr_temp) * tp_ratio_low_fall) / (PACKAGE_THETA_JA_FALL *
									     tscpu_cur_fp_factor),
					MINIMUM_BUDGET_CHANGE);
#else
				total_power +=
				    MAX(((TARGET_TJ - curr_temp) * tt_ratio_low_fall +
					 (prev_temp -
					  curr_temp) * tp_ratio_low_fall) / PACKAGE_THETA_JA_FALL,
					MINIMUM_BUDGET_CHANGE);
#endif
			}

			total_power =
			    (total_power > MINIMUM_TOTAL_POWER) ? total_power : MINIMUM_TOTAL_POWER;
			total_power =
			    (total_power < MAXIMUM_TOTAL_POWER) ? total_power : MAXIMUM_TOTAL_POWER;
			break;

		case 0:
		default:	/* ATM v1 */
			if ((curr_temp > TARGET_TJ_HIGH) && (curr_temp >= prev_temp)) {
#if MTKTSCPU_FAST_POLLING
				delta_power =
				    (curr_temp -
				     prev_temp) / (PACKAGE_THETA_JA_RISE * tscpu_cur_fp_factor);
#else
				delta_power = (curr_temp - prev_temp) / PACKAGE_THETA_JA_RISE;
#endif
				if (prev_temp > TARGET_TJ_HIGH) {
					delta_power =
					    (delta_power >
					     MINIMUM_BUDGET_CHANGE) ? delta_power :
					    MINIMUM_BUDGET_CHANGE;
				}
				total_power -= delta_power;
				total_power =
				    (total_power >
				     MINIMUM_TOTAL_POWER) ? total_power : MINIMUM_TOTAL_POWER;
			}

			if ((curr_temp < TARGET_TJ_LOW) && (curr_temp <= prev_temp)) {
#if MTKTSCPU_FAST_POLLING
				delta_power =
				    (prev_temp -
				     curr_temp) / (PACKAGE_THETA_JA_FALL * tscpu_cur_fp_factor);
#else
				delta_power = (prev_temp - curr_temp) / PACKAGE_THETA_JA_FALL;
#endif
				if (prev_temp < TARGET_TJ_LOW) {
					delta_power =
					    (delta_power >
					     MINIMUM_BUDGET_CHANGE) ? delta_power :
					    MINIMUM_BUDGET_CHANGE;
				}
				total_power += delta_power;
				total_power =
				    (total_power <
				     MAXIMUM_TOTAL_POWER) ? total_power : MAXIMUM_TOTAL_POWER;
			}
			break;
		}

		tscpu_dprintk("%s Tp %ld, Tc %ld, Pt %d\n", __func__, prev_temp, curr_temp,
			      total_power);
		return P_adaptive(total_power, gpu_loading);
	}
#if CONTINUOUS_TM
	else if ((cl_dev_adp_cpu_state_active == 1) && (ctm_on) && (curr_temp < current_ETJ)) {
		tscpu_printk("CTM exit curr_temp %d cetj %d\n", TARGET_TJ, current_ETJ);
		/* even cooler not exit, when CTM is on and current Tj < current_ETJ, leave ATM */
		if (triggered) {
			triggered = 0;
			tscpu_dprintk("%s Tp %ld, Tc %ld, Pt %d\n", __func__, prev_temp, curr_temp,
				      total_power);
			return P_adaptive(0, 0);
		}
#if THERMAL_HEADROOM
		else {
			if (thp_max_cpu_power != 0)
				set_adaptive_cpu_power_limit((unsigned int)
							     MAX(thp_max_cpu_power,
								 MINIMUM_CPU_POWER));
			else
				set_adaptive_cpu_power_limit(0);
		}
#endif
	}
#endif
	else {
		if (triggered) {
			triggered = 0;
			tscpu_dprintk("%s Tp %ld, Tc %ld, Pt %d\n", __func__, prev_temp, curr_temp,
				      total_power);
			return P_adaptive(0, 0);
		}
#if THERMAL_HEADROOM
		else {
			if (thp_max_cpu_power != 0)
				set_adaptive_cpu_power_limit((unsigned int)
							     MAX(thp_max_cpu_power,
								 MINIMUM_CPU_POWER));
			else
				set_adaptive_cpu_power_limit(0);
		}
#endif
	}

	return 0;
}

static int decide_ttj(void)
{
	int i = 0;
	int active_cooler_id = -1;
	int ret = 117000;	/* highest allowable TJ */
	int temp_cl_dev_adp_cpu_state_active = 0;

	for (; i < MAX_CPT_ADAPTIVE_COOLERS; i++) {
		if (cl_dev_adp_cpu_state[i]) {
			ret = MIN(ret, TARGET_TJS[i]);
			temp_cl_dev_adp_cpu_state_active = 1;

			if (ret == TARGET_TJS[i])
				active_cooler_id = i;
		}
	}
	cl_dev_adp_cpu_state_active = temp_cl_dev_adp_cpu_state_active;
	TARGET_TJ = ret;
#if CONTINUOUS_TM
	if (ctm_on) {
		int curr_tpcb = mtk_thermal_get_temp(MTK_THERMAL_SENSOR_AP);

		if (ctm_on == 1) {
			TARGET_TJ =
				MIN(MAX_TARGET_TJ,
				MAX(STEADY_TARGET_TJ, (COEF_AE - COEF_BE * curr_tpcb / 1000)));
		} else if (ctm_on == 2) {
			/* +++ cATM+ +++ */
				TARGET_TJ = ta_get_ttj();
			/* --- cATM+ --- */
		}
		current_ETJ =
		    MIN(MAX_EXIT_TJ, MAX(STEADY_EXIT_TJ, (COEF_AX - COEF_BX * curr_tpcb / 1000)));
		/* tscpu_printk("cttj %d cetj %d tpcb %d\n", TARGET_TJ, current_ETJ, curr_tpcb); */
	}
#endif
	cpu_target_tj = TARGET_TJ;
#if CONTINUOUS_TM
	cpu_target_offset = TARGET_TJ - current_ETJ;
#endif

	TARGET_TJ_HIGH = TARGET_TJ + 1000;
	TARGET_TJ_LOW = TARGET_TJ - 1000;

	if (0 <= active_cooler_id && MAX_CPT_ADAPTIVE_COOLERS > active_cooler_id) {
		PACKAGE_THETA_JA_RISE = PACKAGE_THETA_JA_RISES[active_cooler_id];
		PACKAGE_THETA_JA_FALL = PACKAGE_THETA_JA_FALLS[active_cooler_id];
		MINIMUM_CPU_POWER = MINIMUM_CPU_POWERS[active_cooler_id];
		MAXIMUM_CPU_POWER = MAXIMUM_CPU_POWERS[active_cooler_id];

		/*	get GPU min/max power from GPU DVFS should be
			done when configuring ATM instead of decide_ttj */
#if 0
		{
			MAXIMUM_GPU_POWER = (int)mt_gpufreq_get_max_power();
			MINIMUM_GPU_POWER = (int)mt_gpufreq_get_min_power();
			tscpu_printk("decide_ttj: MAXIMUM_GPU_POWER=%d,MINIMUM_GPU_POWER=%d\n",
			       MAXIMUM_GPU_POWER, MINIMUM_GPU_POWER);
		}
#else
		MINIMUM_GPU_POWER = MINIMUM_GPU_POWERS[active_cooler_id];
		MAXIMUM_GPU_POWER = MAXIMUM_GPU_POWERS[active_cooler_id];
#endif
		MINIMUM_TOTAL_POWER = MINIMUM_CPU_POWER + MINIMUM_GPU_POWER;
		MAXIMUM_TOTAL_POWER = MAXIMUM_CPU_POWER + MAXIMUM_GPU_POWER;
		FIRST_STEP_TOTAL_POWER_BUDGET = FIRST_STEP_TOTAL_POWER_BUDGETS[active_cooler_id];
		MINIMUM_BUDGET_CHANGE = MINIMUM_BUDGET_CHANGES[active_cooler_id];
	} else {
		MINIMUM_CPU_POWER = MINIMUM_CPU_POWERS[0];
		MAXIMUM_CPU_POWER = MAXIMUM_CPU_POWERS[0];
	}

#if THERMAL_HEADROOM
	MAXIMUM_CPU_POWER -= p_Tpcb_correlation * MAX((bts_cur_temp - Tpcb_trip_point), 0) / 1000;

	/* tscpu_printk("max_cpu_pwr %d %d\n", bts_cur_temp, MAXIMUM_CPU_POWER); */
	/* TODO: use 0 as current P */
	thp_max_cpu_power = (thp_threshold_tj - tscpu_read_curr_temp) * thp_p_tj_correlation / 1000 + 0;

	if (thp_max_cpu_power != 0)
		MAXIMUM_CPU_POWER = MIN(MAXIMUM_CPU_POWER, thp_max_cpu_power);

	MAXIMUM_CPU_POWER = MAX(MAXIMUM_CPU_POWER, MINIMUM_CPU_POWER);

	/* tscpu_printk("thp max_cpu_pwr %d %d\n", thp_max_cpu_power, MAXIMUM_CPU_POWER); */
#endif

	return ret;
}
#endif

#if CPT_ADAPTIVE_AP_COOLER
static int adp_cpu_get_max_state(struct thermal_cooling_device *cdev, unsigned long *state)
{
	/* tscpu_dprintk("adp_cpu_get_max_state\n"); */
	*state = 1;
	return 0;
}

static int adp_cpu_get_cur_state(struct thermal_cooling_device *cdev, unsigned long *state)
{
	/* tscpu_dprintk("adp_cpu_get_cur_state\n"); */
	*state = cl_dev_adp_cpu_state[(cdev->type[13] - '0')];
	/* *state = cl_dev_adp_cpu_state; */
	return 0;
}

static int adp_cpu_set_cur_state(struct thermal_cooling_device *cdev, unsigned long state)
{
	int ttj = 117000;

	cl_dev_adp_cpu_state[(cdev->type[13] - '0')] = state;

	ttj = decide_ttj();	/* TODO: no exit point can be obtained in mtk_ts_cpu.c */

	/* tscpu_dprintk("adp_cpu_set_cur_state[%d] =%d, ttj=%d\n", (cdev->type[13] - '0'), state, ttj); */

	if (active_adp_cooler == (int)(cdev->type[13] - '0')) {
#ifndef FAST_RESPONSE_ATM
		/* = (NULL == mtk_thermal_get_gpu_loading_fp) ? 0 : mtk_thermal_get_gpu_loading_fp(); */
		unsigned int gpu_loading;

		if (!mtk_get_gpu_loading(&gpu_loading))
			gpu_loading = 0;

		_adaptive_power_calc(tscpu_g_prev_temp, tscpu_g_curr_temp, (unsigned int) gpu_loading);
#endif
	}
	return 0;
}
#endif

#if CPT_ADAPTIVE_AP_COOLER
static struct thermal_cooling_device_ops mtktscpu_cooler_adp_cpu_ops = {
	.get_max_state = adp_cpu_get_max_state,
	.get_cur_state = adp_cpu_get_cur_state,
	.set_cur_state = adp_cpu_set_cur_state,
};
#endif

#if CPT_ADAPTIVE_AP_COOLER
static int tscpu_read_atm_setting(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < MAX_CPT_ADAPTIVE_COOLERS; i++) {
		seq_printf(m, "%s%02d\n", adaptive_cooler_name, i);
		seq_printf(m, " first_step = %d\n", FIRST_STEP_TOTAL_POWER_BUDGETS[i]);
		seq_printf(m, " theta rise = %d\n", PACKAGE_THETA_JA_RISES[i]);
		seq_printf(m, " theta fall = %d\n", PACKAGE_THETA_JA_FALLS[i]);
		seq_printf(m, " min_budget_change = %d\n", MINIMUM_BUDGET_CHANGES[i]);
		seq_printf(m, " m cpu = %d\n", MINIMUM_CPU_POWERS[i]);
		seq_printf(m, " M cpu = %d\n", MAXIMUM_CPU_POWERS[i]);
		seq_printf(m, " m gpu = %d\n", MINIMUM_GPU_POWERS[i]);
		seq_printf(m, " M gpu = %d\n", MAXIMUM_GPU_POWERS[i]);
	}

	return 0;
}

static ssize_t tscpu_write_atm_setting(struct file *file, const char __user *buffer, size_t count,
				       loff_t *data)
{
	char desc[128];
	/* char arg_name[32] = {0}; */
	/* int arg_val = 0; */
	int len = 0;

	int i_id = -1, i_first_step = -1, i_theta_r = -1, i_theta_f = -1, i_budget_change =
	    -1, i_min_cpu_pwr = -1, i_max_cpu_pwr = -1, i_min_gpu_pwr = -1, i_max_gpu_pwr = -1;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;

	desc[len] = '\0';

	if (9 <= sscanf(desc, "%d %d %d %d %d %d %d %d %d", &i_id, &i_first_step, &i_theta_r, &i_theta_f,
		   &i_budget_change, &i_min_cpu_pwr, &i_max_cpu_pwr, &i_min_gpu_pwr,
		   &i_max_gpu_pwr)) {
		tscpu_printk("tscpu_write_atm_setting input %d %d %d %d %d %d %d %d %d\n", i_id,
			     i_first_step, i_theta_r, i_theta_f, i_budget_change, i_min_cpu_pwr,
			     i_max_cpu_pwr, i_min_gpu_pwr, i_max_gpu_pwr);

		if (i_id >= 0 && i_id < MAX_CPT_ADAPTIVE_COOLERS) {
			if (i_first_step > 0)
				FIRST_STEP_TOTAL_POWER_BUDGETS[i_id] = i_first_step;
			else
				#ifdef CONFIG_MTK_AEE_FEATURE
				aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_DEFAULT,
					"tscpu_write_atm_setting", "Wrong thermal policy");
				#endif

			if (i_theta_r > 0)
				PACKAGE_THETA_JA_RISES[i_id] = i_theta_r;
			else
				#ifdef CONFIG_MTK_AEE_FEATURE
				aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_DEFAULT,
					"tscpu_write_atm_setting", "Wrong thermal policy");
				#endif

			if (i_theta_f > 0)
				PACKAGE_THETA_JA_FALLS[i_id] = i_theta_f;
			else
				#ifdef CONFIG_MTK_AEE_FEATURE
				aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_DEFAULT,
					"tscpu_write_atm_setting", "Wrong thermal policy");
				#endif

			if (i_budget_change >= 0)
				MINIMUM_BUDGET_CHANGES[i_id] = i_budget_change;
			else
				#ifdef CONFIG_MTK_AEE_FEATURE
				aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_DEFAULT,
					"tscpu_write_atm_setting", "Wrong thermal policy");
				#endif

			if (i_min_cpu_pwr > 0)
				MINIMUM_CPU_POWERS[i_id] = i_min_cpu_pwr;
#ifdef ATM_USES_PPM
			else if (i_min_cpu_pwr == 0)
				MINIMUM_CPU_POWERS[i_id] = mt_ppm_thermal_get_min_power() + 1;
				/* choose OPP with power "<=" limit */
#endif
			else
				#ifdef CONFIG_MTK_AEE_FEATURE
				aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_DEFAULT,
					"tscpu_write_atm_setting", "Wrong thermal policy");
				#endif

			if (i_max_cpu_pwr > 0)
				MAXIMUM_CPU_POWERS[i_id] = i_max_cpu_pwr;
#ifdef ATM_USES_PPM
			else if (i_max_cpu_pwr == 0)
				MAXIMUM_CPU_POWERS[i_id] = mt_ppm_thermal_get_max_power() + 1;
				/* choose OPP with power "<=" limit */
#endif
			else
				#ifdef CONFIG_MTK_AEE_FEATURE
				aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_DEFAULT,
					"tscpu_write_atm_setting", "Wrong thermal policy");
				#endif

			if (i_min_gpu_pwr > 0) {
				/* choose OPP with power "<=" limit */
				int min_gpuopp_power = (int) mt_gpufreq_get_min_power() + 1;

				MINIMUM_GPU_POWERS[i_id] = MAX(i_min_gpu_pwr, min_gpuopp_power);
			} else if (i_min_gpu_pwr == 0)
				MINIMUM_GPU_POWERS[i_id] = (int) mt_gpufreq_get_min_power() + 1;
				/* choose OPP with power "<=" limit */
			else
				#ifdef CONFIG_MTK_AEE_FEATURE
				aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_DEFAULT,
					"tscpu_write_atm_setting", "Wrong thermal policy");
				#endif

			if (i_max_gpu_pwr > 0) {
				/* choose OPP with power "<=" limit */
				int min_gpuopp_power = (int) mt_gpufreq_get_min_power() + 1;

				MAXIMUM_GPU_POWERS[i_id] = MAX(i_max_gpu_pwr, min_gpuopp_power);
			} else if (i_max_gpu_pwr == 0)
				MAXIMUM_GPU_POWERS[i_id] = (int) mt_gpufreq_get_max_power() + 1;
				/* choose OPP with power "<=" limit */
			else
				#ifdef CONFIG_MTK_AEE_FEATURE
				aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_DEFAULT,
					"tscpu_write_atm_setting", "Wrong thermal policy");
				#endif

			active_adp_cooler = i_id;

			tscpu_printk("tscpu_write_dtm_setting applied %d %d %d %d %d %d %d %d %d\n",
				     i_id, FIRST_STEP_TOTAL_POWER_BUDGETS[i_id],
				     PACKAGE_THETA_JA_RISES[i_id], PACKAGE_THETA_JA_FALLS[i_id],
				     MINIMUM_BUDGET_CHANGES[i_id], MINIMUM_CPU_POWERS[i_id],
				     MAXIMUM_CPU_POWERS[i_id], MINIMUM_GPU_POWERS[i_id],
				     MAXIMUM_GPU_POWERS[i_id]);
		} else {
			#ifdef CONFIG_MTK_AEE_FEATURE
			aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_DEFAULT,
				"tscpu_write_atm_setting", "Wrong thermal policy");
			#endif
		}

		return count;
	}
	tscpu_dprintk("tscpu_write_dtm_setting bad argument\n");
	return -EINVAL;
}

static int tscpu_read_gpu_threshold(struct seq_file *m, void *v)
{
	seq_printf(m, "H %d L %d\n", GPU_L_H_TRIP, GPU_L_L_TRIP);
	return 0;
}

static ssize_t tscpu_write_gpu_threshold(struct file *file, const char __user *buffer,
					 size_t count, loff_t *data)
{
	char desc[128];
	int len = 0;

	int gpu_h = -1, gpu_l = -1;


	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;

	desc[len] = '\0';

	if (2 <= sscanf(desc, "%d %d", &gpu_h, &gpu_l)) {
		tscpu_printk("tscpu_write_gpu_threshold input %d %d\n", gpu_h, gpu_l);

		if ((gpu_h > 0) && (gpu_l > 0) && (gpu_h > gpu_l)) {
			GPU_L_H_TRIP = gpu_h;
			GPU_L_L_TRIP = gpu_l;

			tscpu_printk("tscpu_write_gpu_threshold applied %d %d\n", GPU_L_H_TRIP,
				     GPU_L_L_TRIP);
		} else {
			tscpu_dprintk("tscpu_write_gpu_threshold out of range\n");
		}

		return count;
	}
	tscpu_dprintk("tscpu_write_gpu_threshold bad argument\n");
	return -EINVAL;
}

/* +ASC+ */
static int tscpu_read_atm(struct seq_file *m, void *v)
{

	seq_printf(m, "[tscpu_read_atm] ver = %d\n", tscpu_atm);
	seq_printf(m, "tt_ratio_high_rise = %d\n", tt_ratio_high_rise);
	seq_printf(m, "tt_ratio_high_fall = %d\n", tt_ratio_high_fall);
	seq_printf(m, "tt_ratio_low_rise = %d\n", tt_ratio_low_rise);
	seq_printf(m, "tt_ratio_low_fall = %d\n", tt_ratio_low_fall);
	seq_printf(m, "tp_ratio_high_rise = %d\n", tp_ratio_high_rise);
	seq_printf(m, "tp_ratio_high_fall = %d\n", tp_ratio_high_fall);
	seq_printf(m, "tp_ratio_low_rise = %d\n", tp_ratio_low_rise);
	seq_printf(m, "tp_ratio_low_fall = %d\n", tp_ratio_low_fall);

	return 0;
}

static ssize_t tscpu_write_atm(struct file *file, const char __user *buffer, size_t count,
			       loff_t *data)
{
	char desc[128];
	int atm_ver;
	int tmp_tt_ratio_high_rise;
	int tmp_tt_ratio_high_fall;
	int tmp_tt_ratio_low_rise;
	int tmp_tt_ratio_low_fall;
	int tmp_tp_ratio_high_rise;
	int tmp_tp_ratio_high_fall;
	int tmp_tp_ratio_low_rise;
	int tmp_tp_ratio_low_fall;
	int len = 0;


	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;

	desc[len] = '\0';

	if (sscanf(desc, "%d %d %d %d %d %d %d %d %d ",
		   &atm_ver, &tmp_tt_ratio_high_rise, &tmp_tt_ratio_high_fall,
		   &tmp_tt_ratio_low_rise, &tmp_tt_ratio_low_fall, &tmp_tp_ratio_high_rise,
		   &tmp_tp_ratio_high_fall, &tmp_tp_ratio_low_rise, &tmp_tp_ratio_low_fall) == 9)
		/* if (5 <= sscanf(desc, "%d %d %d %d %d", &log_switch, &hot, &normal, &low, &lv_offset)) */
	{
		tscpu_atm = atm_ver;
		tt_ratio_high_rise = tmp_tt_ratio_high_rise;
		tt_ratio_high_fall = tmp_tt_ratio_high_fall;
		tt_ratio_low_rise = tmp_tt_ratio_low_rise;
		tt_ratio_low_fall = tmp_tt_ratio_low_fall;
		tp_ratio_high_rise = tmp_tp_ratio_high_rise;
		tp_ratio_high_fall = tmp_tp_ratio_high_fall;
		tp_ratio_low_rise = tmp_tp_ratio_low_rise;
		tp_ratio_low_fall = tmp_tp_ratio_low_fall;

#if PRECISE_HYBRID_POWER_BUDGET
		if (tscpu_atm == 3)
			_adaptive_power_calc = _adaptive_power_ppb;
		else
			_adaptive_power_calc = _adaptive_power;
#endif

		return count;
	}
	tscpu_printk("tscpu_write_atm bad argument\n");
	return -EINVAL;

}

/* -ASC- */
#if THERMAL_HEADROOM
static int tscpu_read_thp(struct seq_file *m, void *v)
{
	seq_printf(m, "Tpcb pt coef %d\n", p_Tpcb_correlation);
	seq_printf(m, "Tpcb threshold %d\n", Tpcb_trip_point);
	seq_printf(m, "Tj pt coef %d\n", thp_p_tj_correlation);
	seq_printf(m, "thp tj threshold %d\n", thp_threshold_tj);

	return 0;
}

static ssize_t tscpu_write_thp(struct file *file, const char __user *buffer, size_t count,
			       loff_t *data)
{
	char desc[128];
	int len = 0;
	int tpcb_coef = -1, tpcb_trip = -1, thp_coef = -1, thp_threshold = -1;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;

	desc[len] = '\0';

	if (4 <= sscanf(desc, "%d %d %d %d", &tpcb_coef, &tpcb_trip, &thp_coef, &thp_threshold)) {
		tscpu_printk("%s input %d %d %d %d\n", __func__, tpcb_coef, tpcb_trip, thp_coef,
			     thp_threshold);

		p_Tpcb_correlation = tpcb_coef;
		Tpcb_trip_point = tpcb_trip;
		thp_p_tj_correlation = thp_coef;
		thp_threshold_tj = thp_threshold;

		return count;
	}
	tscpu_dprintk("%s bad argument\n", __func__);
	return -EINVAL;
}
#endif

#if CONTINUOUS_TM
static int tscpu_read_ctm(struct seq_file *m, void *v)
{
	seq_printf(m, "ctm %d\n", ctm_on);
	seq_printf(m, "Target Tj 0 %d\n", MAX_TARGET_TJ);
	seq_printf(m, "Target Tj 2 %d\n", STEADY_TARGET_TJ);
	seq_printf(m, "Tpcb 1 %d\n", TRIP_TPCB);
	seq_printf(m, "Tpcb 2 %d\n", STEADY_TARGET_TPCB);
	seq_printf(m, "Exit Tj 0 %d\n", MAX_EXIT_TJ);
	seq_printf(m, "Exit Tj 2 %d\n", STEADY_EXIT_TJ);
	seq_printf(m, "Enter_a %d\n", COEF_AE);
	seq_printf(m, "Enter_b %d\n", COEF_BE);
	seq_printf(m, "Exit_a %d\n", COEF_AX);
	seq_printf(m, "Exit_b %d\n", COEF_BX);

	/* +++ cATM+ parameters +++ */
	seq_printf(m, "K_TT %d\n", K_TT);
	seq_printf(m, "MAX_K_SUM_TT %d\n", MAX_K_SUM_TT);
	seq_printf(m, "K_SUM_TT_LOW %d\n", K_SUM_TT_LOW);
	seq_printf(m, "K_SUM_TT_HIGH %d\n", K_SUM_TT_HIGH);
	seq_printf(m, "MIN_SUM_TT %d\n", MIN_SUM_TT);
	seq_printf(m, "MAX_SUM_TT %d\n", MAX_SUM_TT);
	seq_printf(m, "MIN_TTJ %d\n", MIN_TTJ);
	seq_printf(m, "CATMP_STEADY_TTJ_DELTA %d\n", CATMP_STEADY_TTJ_DELTA);
	/* --- cATM+ parameters --- */

	return 0;
}

static ssize_t tscpu_write_ctm(struct file *file, const char __user *buffer, size_t count,
			       loff_t *data)
{
	char desc[256];
	int len = 0;
	int t_ctm_on = -1, t_MAX_TARGET_TJ = -1, t_STEADY_TARGET_TJ = -1, t_TRIP_TPCB =
	    -1, t_STEADY_TARGET_TPCB = -1, t_MAX_EXIT_TJ = -1, t_STEADY_EXIT_TJ = -1, t_COEF_AE =
	    -1, t_COEF_BE = -1, t_COEF_AX = -1, t_COEF_BX = -1,
	    t_K_SUM_TT_HIGH = -1, t_K_SUM_TT_LOW = -1, t_CATMP_STEADY_TTJ_DELTA = -1;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;

	desc[len] = '\0';

	if (11 <= sscanf(desc, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d", &t_ctm_on, &t_MAX_TARGET_TJ,
		   &t_STEADY_TARGET_TJ, &t_TRIP_TPCB, &t_STEADY_TARGET_TPCB, &t_MAX_EXIT_TJ,
		   &t_STEADY_EXIT_TJ, &t_COEF_AE, &t_COEF_BE, &t_COEF_AX, &t_COEF_BX,
		   &t_K_SUM_TT_HIGH, &t_K_SUM_TT_LOW, &t_CATMP_STEADY_TTJ_DELTA)) {
		tscpu_printk("%s input %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n", __func__, t_ctm_on,
			     t_MAX_TARGET_TJ, t_STEADY_TARGET_TJ, t_TRIP_TPCB, t_STEADY_TARGET_TPCB,
			     t_MAX_EXIT_TJ, t_STEADY_EXIT_TJ, t_COEF_AE, t_COEF_BE, t_COEF_AX,
			     t_COEF_BX, t_K_SUM_TT_HIGH, t_K_SUM_TT_LOW, t_CATMP_STEADY_TTJ_DELTA);

		if (t_ctm_on < 0 || t_ctm_on > 2)
			#ifdef CONFIG_MTK_AEE_FEATURE
			aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_DEFAULT,
				"tscpu_write_ctm", "Wrong thermal policy");
			#endif

		if (t_MAX_TARGET_TJ < -20000 || t_MAX_TARGET_TJ > 200000)
			#ifdef CONFIG_MTK_AEE_FEATURE
			aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_DEFAULT,
				"tscpu_write_ctm", "Wrong thermal policy");
			#endif

		if (t_STEADY_TARGET_TJ < -20000 || t_STEADY_TARGET_TJ > 200000)
			#ifdef CONFIG_MTK_AEE_FEATURE
			aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_DEFAULT,
				"tscpu_write_ctm", "Wrong thermal policy");
			#endif

		if (t_TRIP_TPCB < -20000 || t_TRIP_TPCB > 200000)
			#ifdef CONFIG_MTK_AEE_FEATURE
			aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_DEFAULT,
				"tscpu_write_ctm", "Wrong thermal policy");
			#endif

		if (t_STEADY_TARGET_TPCB < -20000 || t_STEADY_TARGET_TPCB > 200000)
			#ifdef CONFIG_MTK_AEE_FEATURE
			aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_DEFAULT,
				"tscpu_write_ctm", "Wrong thermal policy");
			#endif

		if (t_MAX_EXIT_TJ < -20000 || t_MAX_EXIT_TJ > 200000)
			#ifdef CONFIG_MTK_AEE_FEATURE
			aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_DEFAULT,
				"tscpu_write_ctm", "Wrong thermal policy");
			#endif

		if (t_STEADY_EXIT_TJ < -20000 || t_STEADY_EXIT_TJ > 200000)
			#ifdef CONFIG_MTK_AEE_FEATURE
			aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_DEFAULT,
				"tscpu_write_ctm", "Wrong thermal policy");
			#endif

		if (t_COEF_AE < 0 || t_COEF_BE < 0 || t_COEF_AX < 0 || t_COEF_BX < 0)
			#ifdef CONFIG_MTK_AEE_FEATURE
			aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_DEFAULT,
				"tscpu_write_ctm", "Wrong thermal policy");
			#endif

		/* no parameter checking here */
		ctm_on = t_ctm_on;	/* 2: cATM+, 1: cATMv1, 0: off */

		MAX_TARGET_TJ = t_MAX_TARGET_TJ;
		STEADY_TARGET_TJ = t_STEADY_TARGET_TJ;
		TRIP_TPCB = t_TRIP_TPCB;
		STEADY_TARGET_TPCB = t_STEADY_TARGET_TPCB;
		MAX_EXIT_TJ = t_MAX_EXIT_TJ;
		STEADY_EXIT_TJ = t_STEADY_EXIT_TJ;

		COEF_AE = t_COEF_AE;
		COEF_BE = t_COEF_BE;
		COEF_AX = t_COEF_AX;
		COEF_BX = t_COEF_BX;

		/* +++ cATM+ parameters +++ */
		if (ctm_on == 2) {
			if (t_K_SUM_TT_HIGH >= 0 && t_K_SUM_TT_HIGH < MAX_K_SUM_TT)
				K_SUM_TT_HIGH = t_K_SUM_TT_HIGH;

			if (t_K_SUM_TT_LOW >= 0 && t_K_SUM_TT_LOW < MAX_K_SUM_TT)
				K_SUM_TT_LOW = t_K_SUM_TT_LOW;

			if (t_CATMP_STEADY_TTJ_DELTA >= 0)
				CATMP_STEADY_TTJ_DELTA = t_CATMP_STEADY_TTJ_DELTA;

			catmplus_update_params();
		}
		/* --- cATM+ parameters --- */

		return count;
	}

	tscpu_dprintk("%s bad argument\n", __func__);
	return -EINVAL;
}
#endif

#if PRECISE_HYBRID_POWER_BUDGET
static int tscpu_read_phpb(struct seq_file *m, void *v)
{
	int i;
	struct phpb_param *p;

	for (i = 0; i < NR_PHPB_PARAMS; i++) {
		p = &phpb_params[i];
		seq_printf(m, "[%s] %d %d\n", p->type, p->tt, p->tp);
	}
	seq_printf(m, "[common] %d\n", phpb_theta_max);

	return 0;
}

static ssize_t tscpu_write_phpb(struct file *file, const char __user *buffer,
		size_t count, loff_t *data)
{
	char desc[128];
	char *buf = NULL;
	int len = 0;
	int i, tt, tp;
	int __theta;
	int ret = -EINVAL;
	struct phpb_param *p;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);

	if (copy_from_user(desc, buffer, len)) {
		ret = -EFAULT;
		goto exit;
	}

	desc[len] = '\0';
	buf = desc;

	for (i = 0; i < NR_PHPB_PARAMS; i++) {
		p = &phpb_params[i];
		if (strstr(buf, p->type))
			break;
	}

	if (i < NR_PHPB_PARAMS) {
		strsep(&buf, " ");
		if (sscanf(buf, "%d %d", &tt, &tp) != 2)
			goto exit;
		/* TODO verify values */
		p->tt = tt;
		p->tp = tp;
	} else {
		if (strstr(buf, "common") == NULL)
			goto exit;
		strsep(&buf, " ");
		if (kstrtoint(buf, 10, &__theta) != 0)
			goto exit;

		if (__theta < phpb_theta_min)
			goto exit;
		phpb_theta_max = __theta;
	}
	ret = len;

exit:
	return ret;
}

static void phpb_params_init(void)
{
#if defined(CLATM_SET_INIT_CFG)
	phpb_params[PHPB_PARAM_CPU].tt = CLATM_INIT_CFG_PHPB_CPU_TT;
	phpb_params[PHPB_PARAM_CPU].tp = CLATM_INIT_CFG_PHPB_CPU_TP;
#else
	phpb_params[PHPB_PARAM_CPU].tt = 20;
	phpb_params[PHPB_PARAM_CPU].tp = 20;
#endif
	strncpy(phpb_params[PHPB_PARAM_CPU].type, "cpu", 3);
	phpb_params[PHPB_PARAM_CPU].type[3] = '\0';

#if defined(CLATM_SET_INIT_CFG)
	phpb_params[PHPB_PARAM_GPU].tt = CLATM_INIT_CFG_PHPB_GPU_TT;
	phpb_params[PHPB_PARAM_GPU].tp = CLATM_INIT_CFG_PHPB_GPU_TP;
#else
	phpb_params[PHPB_PARAM_GPU].tt = 80;
	phpb_params[PHPB_PARAM_GPU].tp = 80;
#endif
	strncpy(phpb_params[PHPB_PARAM_GPU].type, "gpu", 3);
	phpb_params[PHPB_PARAM_GPU].type[3] = '\0';
}
#endif	/* PRECISE_HYBRID_POWER_BUDGET */

#endif	/* CPT_ADAPTIVE_AP_COOLER */

#if CPT_ADAPTIVE_AP_COOLER
static int tscpu_atm_setting_open(struct inode *inode, struct file *file)
{
	return single_open(file, tscpu_read_atm_setting, NULL);
}

static const struct file_operations mtktscpu_atm_setting_fops = {
	.owner = THIS_MODULE,
	.open = tscpu_atm_setting_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = tscpu_write_atm_setting,
	.release = single_release,
};

static int tscpu_gpu_threshold_open(struct inode *inode, struct file *file)
{
	return single_open(file, tscpu_read_gpu_threshold, NULL);
}

static const struct file_operations mtktscpu_gpu_threshold_fops = {
	.owner = THIS_MODULE,
	.open = tscpu_gpu_threshold_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = tscpu_write_gpu_threshold,
	.release = single_release,
};

/* +ASC+ */
static int tscpu_open_atm(struct inode *inode, struct file *file)
{
	return single_open(file, tscpu_read_atm, NULL);
}

static const struct file_operations mtktscpu_atm_fops = {
	.owner = THIS_MODULE,
	.open = tscpu_open_atm,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = tscpu_write_atm,
	.release = single_release,
};
/* -ASC- */

#if THERMAL_HEADROOM
static int tscpu_thp_open(struct inode *inode, struct file *file)
{
	return single_open(file, tscpu_read_thp, NULL);
}

static const struct file_operations mtktscpu_thp_fops = {
	.owner = THIS_MODULE,
	.open = tscpu_thp_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = tscpu_write_thp,
	.release = single_release,
};
#endif

#if CONTINUOUS_TM
static int tscpu_ctm_open(struct inode *inode, struct file *file)
{
	return single_open(file, tscpu_read_ctm, NULL);
}

static const struct file_operations mtktscpu_ctm_fops = {
	.owner = THIS_MODULE,
	.open = tscpu_ctm_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = tscpu_write_ctm,
	.release = single_release,
};
#endif				/* CONTINUOUS_TM */

#if PRECISE_HYBRID_POWER_BUDGET
static int tscpu_phpb_open(struct inode *inode, struct file *file)
{
	return single_open(file, tscpu_read_phpb, NULL);
}

static const struct file_operations mtktscpu_phpb_fops = {
	.owner = THIS_MODULE,
	.open = tscpu_phpb_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = tscpu_write_phpb,
	.release = single_release,
};
#endif
#endif				/* CPT_ADAPTIVE_AP_COOLER */

#if PRECISE_HYBRID_POWER_BUDGET
static void phpb_init(struct proc_dir_entry *mtktscpu_dir)
{
	struct proc_dir_entry *entry;

	phpb_params_init();

	entry = proc_create("clphpb", S_IRUGO | S_IWUSR | S_IWGRP, mtktscpu_dir, &mtktscpu_phpb_fops);
	if (entry)
		proc_set_user(entry, uid, gid);
}
#endif

static void tscpu_cooler_create_fs(void)
{
	struct proc_dir_entry *entry = NULL;
	struct proc_dir_entry *mtktscpu_dir = NULL;

	mtktscpu_dir = mtk_thermal_get_proc_drv_therm_dir_entry();
	if (!mtktscpu_dir) {
		tscpu_printk("[%s]: mkdir /proc/driver/thermal failed\n", __func__);
	} else {
#if CPT_ADAPTIVE_AP_COOLER
		entry =
		    proc_create("clatm_setting", S_IRUGO | S_IWUSR | S_IWGRP, mtktscpu_dir,
				&mtktscpu_atm_setting_fops);
		if (entry)
			proc_set_user(entry, uid, gid);

		entry =
		    proc_create("clatm_gpu_threshold", S_IRUGO | S_IWUSR | S_IWGRP, mtktscpu_dir,
				&mtktscpu_gpu_threshold_fops);
		if (entry)
			proc_set_user(entry, uid, gid);
#endif				/* #if CPT_ADAPTIVE_AP_COOLER */


		/* +ASC+ */
		entry = proc_create("clatm", S_IRUGO | S_IWUSR | S_IWGRP, mtktscpu_dir, &mtktscpu_atm_fops);
		if (entry)
			proc_set_user(entry, uid, gid);
		/* -ASC- */

#if THERMAL_HEADROOM
		entry = proc_create("clthp", S_IRUGO | S_IWUSR | S_IWGRP, mtktscpu_dir, &mtktscpu_thp_fops);
		if (entry)
			proc_set_user(entry, uid, gid);
#endif

#if CONTINUOUS_TM
		entry = proc_create("clctm", S_IRUGO | S_IWUSR | S_IWGRP, mtktscpu_dir, &mtktscpu_ctm_fops);
		if (entry)
			proc_set_user(entry, uid, gid);
#endif

#if PRECISE_HYBRID_POWER_BUDGET
		phpb_init(mtktscpu_dir);
#endif
	}
}

#ifdef FAST_RESPONSE_ATM
void atm_cancel_hrtimer(void)
{
	hrtimer_try_to_cancel(&atm_hrtimer);
}

void atm_restart_hrtimer(void)
{
	ktime_t ktime;

	ktime = ktime_set(0, atm_hrtimer_polling_delay);
	hrtimer_start(&atm_hrtimer, ktime, HRTIMER_MODE_REL);
}

static unsigned long atm_get_timeout_time(int curr_temp)
{

	/*
	* curr_temp can't smaller than -30'C
	*/
	curr_temp = (curr_temp < -30000) ? -30000 : curr_temp;

	if (curr_temp >= 65000)
		return atm_hrtimer_polling_delay;
	else
		return (atm_hrtimer_polling_delay << ((81394 - curr_temp) >> 14));
}

static enum hrtimer_restart atm_loop(struct hrtimer *timer)
{
	ktime_t ktime;
	int temp;
	static int hasDisabled;


	unsigned long polling_time;
	unsigned long polling_time_s;
	unsigned long polling_time_ns;


	tscpu_workqueue_start_timer();

	atm_prev_maxtj = atm_curr_maxtj;
	atm_curr_maxtj = tscpu_get_curr_temp();

	if (atm_curr_maxtj >= 100000 || (atm_curr_maxtj - atm_prev_maxtj >= 15000))
		tscpu_warn("%s c %d p %d l %d b %d L %d LL %d\n", __func__,
			atm_curr_maxtj, atm_prev_maxtj, adaptive_cpu_power_limit,
			get_immediate_big_wrap(), get_immediate_cpuL_wrap(),
			get_immediate_cpuLL_wrap());
	else
		tscpu_dprintk("%s c %d p %d l %d b %d L %d LL %d\n", __func__,
			atm_curr_maxtj, atm_prev_maxtj, adaptive_cpu_power_limit,
			get_immediate_big_wrap(), get_immediate_cpuL_wrap(),
			get_immediate_cpuLL_wrap());
#ifdef ENALBE_UART_LIMIT
#if ENALBE_UART_LIMIT
	temp = atm_curr_maxtj;
	if ((TEMP_DIS_UART - TEMP_TOLERANCE) < temp) {
		/*************************************************
			Disable UART log
		*************************************************/
		if (mt_get_uartlog_status()) {
			hasDisabled = 1;
			set_uartlog_status(FALSE);
		}
	}

	if (temp < (TEMP_EN_UART + TEMP_TOLERANCE)) {
		/*************************************************
			Restore UART log
		*************************************************/
		if (!mt_get_uartlog_status() && hasDisabled)
			set_uartlog_status(TRUE);

		hasDisabled = 0;
	}
#endif
#endif

	wake_up_process(krtatm_thread_handle);

	polling_time = atm_get_timeout_time(atm_curr_maxtj);

	/*avoid overflow*/
	if (polling_time > (1000000000-1)) {
		polling_time_s = polling_time / 1000000000;
		polling_time_ns = polling_time % 1000000000;
		ktime = ktime_set(polling_time_s, polling_time_ns);
		/*tscpu_warn("%s polling_time_s=%ld  polling_time_ns=%ld\n", __func__,polling_time_s,polling_time_ns);*/
	} else {
		ktime = ktime_set(0, polling_time);
	}

	hrtimer_forward_now(timer, ktime);

	return HRTIMER_RESTART;
}

static void atm_hrtimer_init(void)
{
	ktime_t ktime;

	tscpu_dprintk("%s\n", __func__);

	/*100000000 = 100 ms,polling delay can't larger than 100ms*/
	atm_hrtimer_polling_delay = (atm_hrtimer_polling_delay < 100000000) ?
		atm_hrtimer_polling_delay : 100000000;

	ktime = ktime_set(0, atm_hrtimer_polling_delay);

	hrtimer_init(&atm_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

	atm_hrtimer.function = atm_loop;
	hrtimer_start(&atm_hrtimer, ktime, HRTIMER_MODE_REL);
}

#define KRTATM_RT	(1)
#define KRTATM_CFS	(2)
#define KRTATM_SCH	KRTATM_CFS

static int krtatm_thread(void *arg)
{
#if KRTATM_SCH == KRTATM_RT
	struct sched_param param = {.sched_priority = 98 };

	sched_setscheduler(current, SCHED_FIFO, &param);
#elif KRTATM_SCH == KRTATM_CFS
	set_user_nice(current, MIN_NICE);
#endif
	set_current_state(TASK_INTERRUPTIBLE);

	tscpu_dprintk("%s 1st run\n", __func__);

	schedule();

	for (;;) {
		tscpu_dprintk("%s awake\n", __func__);
#if (CONFIG_THERMAL_AEE_RR_REC == 1)
		aee_rr_rec_thermal_ATM_status(ATM_WAKEUP);
#endif
		if (kthread_should_stop())
			break;

		{
			unsigned int gpu_loading;

			if (!mtk_get_gpu_loading(&gpu_loading))
				gpu_loading = 0;

			/* use separate prev/curr in krtatm because krtatm may be blocked by PPM */
			krtatm_prev_maxtj = krtatm_curr_maxtj;
			krtatm_curr_maxtj = atm_curr_maxtj;
			if (krtatm_prev_maxtj == 0)
				krtatm_prev_maxtj = atm_prev_maxtj;
			_adaptive_power_calc(krtatm_prev_maxtj, krtatm_curr_maxtj, (unsigned int) gpu_loading);
		}
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
	}

	tscpu_warn("%s stopped\n", __func__);

	return 0;
}
#endif	/* FAST_RESPONSE_ATM */

static int __init mtk_cooler_atm_init(void)
{
	int err = 0;

	tscpu_dprintk("%s: start\n", __func__);

	err = apthermolmt_register_user(&ap_atm, ap_atm_log);
	if (err < 0)
		return err;

#if CPT_ADAPTIVE_AP_COOLER
	_adaptive_power_calc = _adaptive_power; /* default use old version */
#if PRECISE_HYBRID_POWER_BUDGET
	if (tscpu_atm == 3)
		_adaptive_power_calc = _adaptive_power_ppb;
#endif
	cl_dev_adp_cpu[0] = mtk_thermal_cooling_device_register("cpu_adaptive_0", NULL,
								&mtktscpu_cooler_adp_cpu_ops);

	cl_dev_adp_cpu[1] = mtk_thermal_cooling_device_register("cpu_adaptive_1", NULL,
								&mtktscpu_cooler_adp_cpu_ops);

	cl_dev_adp_cpu[2] = mtk_thermal_cooling_device_register("cpu_adaptive_2", NULL,
								&mtktscpu_cooler_adp_cpu_ops);

#if defined(CLATM_SET_INIT_CFG)
	mtk_thermal_cooling_device_add_exit_point(cl_dev_adp_cpu[0], CLATM_INIT_CFG_0_EXIT_POINT);
	mtk_thermal_cooling_device_add_exit_point(cl_dev_adp_cpu[1], CLATM_INIT_CFG_1_EXIT_POINT);
	mtk_thermal_cooling_device_add_exit_point(cl_dev_adp_cpu[2], CLATM_INIT_CFG_2_EXIT_POINT);
#endif

#endif
	if (err) {
		tscpu_printk("%s fail\n", __func__);
		return err;
	}
	tscpu_cooler_create_fs();

#ifdef FAST_RESPONSE_ATM
	atm_hrtimer_init();

	tscpu_dprintk("%s creates krtatm\n", __func__);
	krtatm_thread_handle = kthread_create(krtatm_thread, (void *)NULL, "krtatm");
	if (IS_ERR(krtatm_thread_handle)) {
		krtatm_thread_handle = NULL;
		tscpu_printk("%s krtatm creation fails\n", __func__);
	} else
		wake_up_process(krtatm_thread_handle);
#endif
#if 0
	reset_gpu_power_history();
#endif
	tscpu_dprintk("%s: end\n", __func__);
	return 0;
}

static void __exit mtk_cooler_atm_exit(void)
{
#ifdef FAST_RESPONSE_ATM
	hrtimer_cancel(&atm_hrtimer);

	if (krtatm_thread_handle)
		kthread_stop(krtatm_thread_handle);
#endif

#if CPT_ADAPTIVE_AP_COOLER
	if (cl_dev_adp_cpu[0]) {
		mtk_thermal_cooling_device_unregister(cl_dev_adp_cpu[0]);
		cl_dev_adp_cpu[0] = NULL;
	}

	if (cl_dev_adp_cpu[1]) {
		mtk_thermal_cooling_device_unregister(cl_dev_adp_cpu[1]);
		cl_dev_adp_cpu[1] = NULL;
	}

	if (cl_dev_adp_cpu[2]) {
		mtk_thermal_cooling_device_unregister(cl_dev_adp_cpu[2]);
		cl_dev_adp_cpu[2] = NULL;
	}
#endif

	apthermolmt_unregister_user(&ap_atm);
}

module_init(mtk_cooler_atm_init);
module_exit(mtk_cooler_atm_exit);
