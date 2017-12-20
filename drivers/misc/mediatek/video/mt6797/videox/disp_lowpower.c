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

#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/ktime.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include "ion_drv.h"
#include "mtk_ion.h"
#include "mt_idle.h"
/* #include "mt_spm_reg.h" */ /* FIXME: tmp comment */
#include "mt_boot_common.h"
/* #include "pcm_def.h" */ /* FIXME: tmp comment */
#include "mt_spm_idle.h"
#include "mt-plat/mt_smi.h"
#include "m4u.h"
#include "m4u_port.h"

#include "disp_drv_platform.h"
#include "debug.h"
#include "disp_drv_log.h"
#include "disp_lcm.h"
#include "disp_utils.h"
#include "disp_session.h"
#include "primary_display.h"
#include "disp_helper.h"
#include "cmdq_def.h"
#include "cmdq_record.h"
#include "cmdq_reg.h"
#include "cmdq_core.h"
#include "ddp_manager.h"
#include "disp_lcm.h"
#include "ddp_clkmgr.h"
#include "mt_smi.h"
#include "disp_drv_log.h"
#include "disp_lowpower.h"
#include "disp_rect.h"

/* device tree */
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/io.h>

#include "mmdvfs_mgr.h"
#define MMSYS_CLK_LOW (0)
#define MMSYS_CLK_HIGH (1)

#define idlemgr_pgc		_get_idlemgr_context()
#define golden_setting_pgc	_get_golden_setting_context()

#define kick_dump_max_length (1024 * 16 * 4)
static unsigned char kick_string_buffer_analysize[kick_dump_max_length] = { 0 };
static unsigned int kick_buf_length;
static atomic_t idlemgr_task_wakeup = ATOMIC_INIT(1);

/*#define NO_SPM*/

/* wait for mmdvfs_mgr.h ready */
#define mmdvfs_notify_mmclk_switch_request(...)
#define MMDVFS_EVENT_OVL_SINGLE_LAYER_ENTER 0
#define MMDVFS_EVENT_OVL_SINGLE_LAYER_EXIT 1
#define MMDVFS_EVENT_UI_IDLE_ENTER 2
#define MMDVFS_EVENT_UI_IDLE_EXIT 3


/* Local API */
/*********************************************************************************************************************/
static int _primary_path_idlemgr_monitor_thread(void *data);

static disp_idlemgr_context *_get_idlemgr_context(void)
{
	static int is_inited;
	static disp_idlemgr_context g_idlemgr_context;

	if (!is_inited) {
		init_waitqueue_head(&(g_idlemgr_context.idlemgr_wait_queue));
		g_idlemgr_context.session_mode_before_enter_idle = DISP_INVALID_SESSION_MODE;
		g_idlemgr_context.is_primary_idle = 0;
		g_idlemgr_context.enterulps = 0;
		g_idlemgr_context.idlemgr_last_kick_time = ~(0ULL);
		g_idlemgr_context.cur_lp_cust_mode = 0;
		g_idlemgr_context.primary_display_idlemgr_task
			= kthread_create(_primary_path_idlemgr_monitor_thread, NULL, "disp_idlemgr");

		/* wakeup process when idlemgr init */
		/* wake_up_process(g_idlemgr_context.primary_display_idlemgr_task); */

		is_inited = 1;
	}

	return &g_idlemgr_context;
}

int primary_display_idlemgr_init(void)
{
	wake_up_process(idlemgr_pgc->primary_display_idlemgr_task);
	return 0;
}

static golden_setting_context *_get_golden_setting_context(void)
{
	static int is_inited;
	static golden_setting_context g_golden_setting_context;

	if (!is_inited) {

		/* default setting */
		g_golden_setting_context.is_one_layer = 0;
		g_golden_setting_context.fps = 60;
		g_golden_setting_context.is_dc = 0;
		g_golden_setting_context.is_display_idle = 0;
		g_golden_setting_context.is_wrot_sram = 0;
		g_golden_setting_context.mmsys_clk = MMSYS_CLK_HIGH; /* 320: low ; 450: high */

		/* primary_display */
		g_golden_setting_context.dst_width	= primary_get_lcm()->params->width;
		g_golden_setting_context.dst_height	= primary_get_lcm()->params->height;
		g_golden_setting_context.rdma_width = g_golden_setting_context.dst_width;
		g_golden_setting_context.rdma_height = g_golden_setting_context.dst_height;
		if (g_golden_setting_context.dst_width == 1080
			&& g_golden_setting_context.dst_height == 1920)
			g_golden_setting_context.hrt_magicnum = 4;
		else if (g_golden_setting_context.dst_width == 1440
			&& g_golden_setting_context.dst_height == 2560)
			g_golden_setting_context.hrt_magicnum = 4;

		/* set hrtnum max */
		g_golden_setting_context.hrt_num = g_golden_setting_context.hrt_magicnum + 1;

		/* fifo mode : 0/1/2 */
		if (g_golden_setting_context.is_display_idle)
			g_golden_setting_context.fifo_mode = 0;
		else if (g_golden_setting_context.hrt_num > g_golden_setting_context.hrt_magicnum)
			g_golden_setting_context.fifo_mode = 2;
		else
			g_golden_setting_context.fifo_mode = 1;
		/* ext_display */
		g_golden_setting_context.ext_dst_width = g_golden_setting_context.dst_width;
		g_golden_setting_context.ext_dst_height = g_golden_setting_context.dst_height;
		g_golden_setting_context.ext_hrt_magicnum = g_golden_setting_context.hrt_magicnum;
		g_golden_setting_context.ext_hrt_num = g_golden_setting_context.hrt_num;

		is_inited = 1;
	}

	return &g_golden_setting_context;
}

static void set_fps(unsigned int fps)
{
	golden_setting_pgc->fps = fps;
}

static void set_is_display_idle(unsigned int is_displayidle)
{

	if (golden_setting_pgc->is_display_idle != is_displayidle) {
		golden_setting_pgc->is_display_idle = is_displayidle;

		if (is_displayidle)
			golden_setting_pgc->fifo_mode = 0;
		else if (golden_setting_pgc->hrt_num <= golden_setting_pgc->hrt_magicnum)
			golden_setting_pgc->fifo_mode = 1;
		else
			golden_setting_pgc->fifo_mode = 2;
		/* notify to mmsys mgr */
		if (disp_helper_get_option(DISP_OPT_DYNAMIC_SWITCH_MMSYSCLK)) {
			if (is_displayidle)
				;/*mmdvfs_notify_mmclk_switch_request(MMDVFS_EVENT_UI_IDLE_ENTER);*/
			else
				;/*mmdvfs_notify_mmclk_switch_request(MMDVFS_EVENT_UI_IDLE_EXIT);*/
		}
	}

}


#if 0 /* defined but not used */
static unsigned int get_hrtnum(void)
{
	return golden_setting_pgc->hrt_num;
}
#endif


static void set_share_sram(unsigned int is_share_sram)
{
	if (golden_setting_pgc->is_wrot_sram != is_share_sram)
		golden_setting_pgc->is_wrot_sram = is_share_sram;
}

static unsigned int use_wrot_sram(void)
{
	return golden_setting_pgc->is_wrot_sram;
}

static void set_mmsys_clk(unsigned int clk)
{
	if (golden_setting_pgc->mmsys_clk != clk)
		golden_setting_pgc->mmsys_clk = clk;
}

static unsigned int get_mmsys_clk(void)
{
	return golden_setting_pgc->mmsys_clk;
}

static int primary_display_set_idle_stat(int is_idle)
{
	int old_stat = idlemgr_pgc->is_primary_idle;

	idlemgr_pgc->is_primary_idle = is_idle;
	return old_stat;
}

/* Need blocking for stop trigger loop  */
int _blocking_flush(void)
{
	int ret = 0;
	cmdqRecHandle handle = NULL;

	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);
	if (ret) {
		DISPERR("%s:%d, create cmdq handle fail!ret=%d\n", __func__, __LINE__, ret);
		return -1;
	}
	cmdqRecReset(handle);
	_cmdq_insert_wait_frame_done_token_mira(handle);

	cmdqRecFlush(handle);
	cmdqRecDestroy(handle);
	return ret;
}

int primary_display_dsi_vfp_change(int state)
{
	int ret = 0;
	cmdqRecHandle handle = NULL;

	cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);
	cmdqRecReset(handle);

	/* make sure token rdma_sof is clear */
	cmdqRecClearEventToken(handle, CMDQ_EVENT_DISP_RDMA0_SOF);

	/* wait rdma0_sof: only used for video mode & trigger loop need wait and clear rdma0 sof */
	cmdqRecWaitNoClear(handle, CMDQ_EVENT_DISP_RDMA0_SOF);

	if (state == 1) {
		/* need calculate fps by vdo mode params */
		/* set_fps(55); */
		dpmgr_path_ioctl(primary_get_dpmgr_handle(), handle,
			DDP_DSI_PORCH_CHANGE,
			&primary_get_lcm()->params->dsi.vertical_frontporch_for_low_power);
	} else if (state == 0) {
		dpmgr_path_ioctl(primary_get_dpmgr_handle(), handle,
			DDP_DSI_PORCH_CHANGE,
			&primary_get_lcm()->params->dsi.vertical_frontporch);
	}
	cmdqRecFlushAsync(handle);
	cmdqRecDestroy(handle);
	return ret;
}

void _idle_set_golden_setting(void)
{
	cmdqRecHandle handle;
	disp_ddp_path_config *pconfig = dpmgr_path_get_last_config_notclear(primary_get_dpmgr_handle());

	/* no need lock */
	/* 1.create and reset cmdq */
	cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);

	cmdqRecReset(handle);

	/* 2.wait mutex0_stream_eof: only used for video mode */
	cmdqRecWaitNoClear(handle, CMDQ_EVENT_MUTEX0_STREAM_EOF);


	/* 3.golden setting */
	dpmgr_path_ioctl(primary_get_dpmgr_handle(), handle, DDP_RDMA_GOLDEN_SETTING, pconfig);

	/* 4.flush */
	cmdqRecFlushAsync(handle);
	cmdqRecDestroy(handle);
}

/* Share wrot sram for vdo mode increase enter sodi ratio */
void _acquire_wrot_resource_nolock(CMDQ_EVENT_ENUM resourceEvent)
{
	cmdqRecHandle handle;

	int32_t acquireResult;
	disp_ddp_path_config *pconfig = dpmgr_path_get_last_config_notclear(primary_get_dpmgr_handle());

	DISPMSG("[disp_lowpower]%s\n", __func__);
	if (use_wrot_sram())
		return;

	if (is_mipi_enterulps())
		return;

	/* 1.create and reset cmdq */
	cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);

	cmdqRecReset(handle);

	/* 2. wait eof */
	_cmdq_insert_wait_frame_done_token_mira(handle);

	/* 3.try to share wrot sram */
	acquireResult = cmdqRecWriteForResource(handle, resourceEvent,
		0x1400F000+0xb0, 1, ~0);
	if (acquireResult < 0) {
		/* acquire resource fail */
		DISPMSG("acquire resource fail\n");

		cmdqRecDestroy(handle);
		return;

	} else {
		/* acquire resource success */
		/* cmdqRecClearEventToken(handle, resourceEvent); //???cmdq do it */

		/* set rdma golden setting parameters*/
		set_share_sram(1);

		/* add instr for modification rdma fifo regs */
		/* dpmgr_handle can cover both dc & dl */
		if (disp_helper_get_option(DISP_OPT_DYNAMIC_RDMA_GOLDEN_SETTING))
			dpmgr_path_ioctl(primary_get_dpmgr_handle(), handle, DDP_RDMA_GOLDEN_SETTING, pconfig);

	}

	cmdqRecFlushAsync(handle);
	cmdqRecDestroy(handle);
}

static int32_t _acquire_wrot_resource(CMDQ_EVENT_ENUM resourceEvent)
{
	primary_display_manual_lock();
	_acquire_wrot_resource_nolock(resourceEvent);
	primary_display_manual_unlock();

	return 0;
}


void _release_wrot_resource_nolock(CMDQ_EVENT_ENUM resourceEvent)
{
	cmdqRecHandle handle;
	disp_ddp_path_config *pconfig = dpmgr_path_get_last_config_notclear(primary_get_dpmgr_handle());

	DISPMSG("[disp_lowpower]%s\n", __func__);
	if (use_wrot_sram() == 0)
		return;

	/* 1.create and reset cmdq */
	cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);

	cmdqRecReset(handle);

	/* 2.wait eof */
	_cmdq_insert_wait_frame_done_token_mira(handle);

	/* 3.release share sram */
	cmdqRecWrite(handle, 0x1400F000+0xb0, 0, ~0); /* why need ??? */
	cmdqRecReleaseResource(handle, resourceEvent);

	/* set rdma golden setting parameters*/
	set_share_sram(0);

	/* 4.add instr for modification rdma fifo regs */
	/* rdma: dpmgr_handle can cover both dc & dl */
	if (disp_helper_get_option(DISP_OPT_DYNAMIC_RDMA_GOLDEN_SETTING))
		dpmgr_path_ioctl(primary_get_dpmgr_handle(), handle, DDP_RDMA_GOLDEN_SETTING, pconfig);

	cmdqRecFlushAsync(handle);
	cmdqRecDestroy(handle);
}

static int32_t _release_wrot_resource(CMDQ_EVENT_ENUM resourceEvent)
{
	/* need lock  */
	primary_display_manual_lock();
	_release_wrot_resource_nolock(resourceEvent);
	primary_display_manual_unlock();

	return 0;
}

int _switch_mmsys_clk_callback(unsigned int need_disable_pll)
{
	/* disable vencpll */
	if (need_disable_pll == MM_VENCPLL) {
		ddp_clk_set_parent(MUX_MM, SYSPLL2_D2);
		ddp_clk_disable_unprepare(MUX_MM);
		ddp_clk_disable_unprepare(MM_VENCPLL);
	} else if (need_disable_pll == SYSPLL2_D2) {
		ddp_clk_set_parent(MUX_MM, MM_VENCPLL);
		ddp_clk_disable_unprepare(MUX_MM);
		ddp_clk_disable_unprepare(SYSPLL2_D2);
	}

	return 0;
}

int _switch_mmsys_clk(int mmsys_clk_old, int mmsys_clk_new)
{
	int ret = 0;
	cmdqRecHandle handle;
	/*unsigned int need_disable_pll = 0;*/
	disp_ddp_path_config *pconfig = dpmgr_path_get_last_config_notclear(primary_get_dpmgr_handle());

	DISPMSG("[disp_lowpower]%s\n", __func__);
	if (mmsys_clk_new == get_mmsys_clk())
		return ret;

	if (primary_get_state() != DISP_ALIVE || is_mipi_enterulps()) {
		DISPMSG("[disp_lowpower]_switch_mmsys_clk when display suspend old = %d & new = %d.\n",
			mmsys_clk_old, mmsys_clk_new);
		return ret;
	}
	/* 1.create and reset cmdq */
	cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);

	cmdqRecReset(handle);

	if (mmsys_clk_old == MMSYS_CLK_HIGH && mmsys_clk_new == MMSYS_CLK_LOW) {
		/* 2.wait sof */
		_cmdq_insert_wait_frame_done_token_mira(handle);
#if 0
		ddp_clk_prepare_enable(MUX_MM);
		ddp_clk_prepare_enable(SYSPLL2_D2);
		cmdqRecWrite(handle, 0x10000048, 0x07000000, 0x07000000); /* clear */
		cmdqRecWrite(handle, 0x10000044, 0x04000000, 0x04000000); /* set syspll2_d2 */
		cmdqRecWrite(handle, 0x10000004, 8, 8); /* update */
#endif
		/* set rdma golden setting parameters */
		set_mmsys_clk(MMSYS_CLK_LOW);
		/*need_disable_pll = MM_VENCPLL;*/

	} else if (mmsys_clk_old == MMSYS_CLK_LOW && mmsys_clk_new == MMSYS_CLK_HIGH) {
		/* 2.wait sof */
		_cmdq_insert_wait_frame_done_token_mira(handle);
#if 0
		/* enable vencpll */
		ddp_clk_prepare_enable(MUX_MM);
		ddp_clk_prepare_enable(MM_VENCPLL);
		cmdqRecWrite(handle, 0x10000048, 0x07000000, 0x07000000); /* clear */
		cmdqRecWrite(handle, 0x10000044, 0x02000000, 0x02000000); /* set vencpll */
		cmdqRecWrite(handle, 0x10000004, 8, 8); /* update */
#endif
		/* set rdma golden setting parameters */
		set_mmsys_clk(MMSYS_CLK_HIGH);
		/*need_disable_pll = SYSPLL2_D2;*/

	} else {
		goto cmdq_d;
	}

	/* 4.add instr for modification rdma fifo regs */
	/* rdma: dpmgr_handle can cover both dc & dl */
	if (disp_helper_get_option(DISP_OPT_DYNAMIC_RDMA_GOLDEN_SETTING))
		dpmgr_path_ioctl(primary_get_dpmgr_handle(), handle, DDP_RDMA_GOLDEN_SETTING, pconfig);


	cmdqRecFlush(handle);

cmdq_d:
	cmdqRecDestroy(handle);

	/*_switch_mmsys_clk_callback(need_disable_pll);*/
	return get_mmsys_clk();
}

int primary_display_switch_mmsys_clk(int mmsys_clk_old, int mmsys_clk_new)
{
	/* need lock */
	DISPMSG("[disp_lowpower]%s\n", __func__);
	primary_display_manual_lock();
	_switch_mmsys_clk(mmsys_clk_old, mmsys_clk_new);
	primary_display_manual_unlock();

	return 0;
}

void _primary_display_disable_mmsys_clk(void)
{
	if (primary_get_sess_mode() == DISP_SESSION_RDMA_MODE) {
		/* switch back to DL mode before suspend */
		do_primary_display_switch_mode(DISP_SESSION_DIRECT_LINK_MODE,
					primary_get_sess_id(), 0, NULL, 1);
	}
	if (primary_get_sess_mode() != DISP_SESSION_DIRECT_LINK_MODE)
		return;

	/* blocking flush before stop trigger loop */
	_blocking_flush();
	/* no  need lock now */
	if (disp_helper_get_option(DISP_OPT_USE_CMDQ)) {
		DISPCHECK("[LP]1.display cmdq trigger loop stop[begin]\n");
		_cmdq_stop_trigger_loop();
		DISPDBG("[LP]1.display cmdq trigger loop stop[end]\n");
	}

	DISPDBG("[LP]2.primary display path stop[begin]\n");
	dpmgr_path_stop(primary_get_dpmgr_handle(), CMDQ_DISABLE);
	DISPCHECK("[LP]2.primary display path stop[end]\n");

	if (dpmgr_path_is_busy(primary_get_dpmgr_handle())) {
		DISPERR("[LP]2.stop display path failed, still busy\n");
		dpmgr_path_reset(primary_get_dpmgr_handle(), CMDQ_DISABLE);
		/* even path is busy(stop fail), we still need to continue power off other module/devices */
	}

	/* can not release fence here */
	DISPCHECK("[LP]3.dpmanager path power off[begin]\n");
	dpmgr_path_power_off_bypass_pwm(primary_get_dpmgr_handle(), CMDQ_DISABLE);

	if (primary_display_is_decouple_mode()) {
		DISPCHECK("[LP]3.1 dpmanager path power off: ovl2men [begin]\n");
		if (primary_get_ovl2mem_handle())
			dpmgr_path_power_off(primary_get_ovl2mem_handle(), CMDQ_DISABLE);
		else
			DISPERR("display is decouple mode, but ovl2mem_path_handle is null\n");

		DISPCHECK("[LP]3.1 dpmanager path power off: ovl2men [end]\n");
	}
	DISPCHECK("[LP]3.dpmanager path power off[end]\n");
	if (disp_helper_get_option(DISP_OPT_MET_LOG))
		set_enterulps(1);
}

void _primary_display_enable_mmsys_clk(void)
{
	disp_ddp_path_config *data_config;
	struct ddp_io_golden_setting_arg gset_arg;

	if (primary_get_sess_mode() != DISP_SESSION_DIRECT_LINK_MODE)
		return;

	/* do something */
	DISPCHECK("[LP]1.dpmanager path power on[begin]\n");
	memset(&gset_arg, 0, sizeof(gset_arg));
	gset_arg.dst_mod_type = dpmgr_path_get_dst_module_type(primary_get_dpmgr_handle());
	if (primary_display_is_decouple_mode()) {
		if (primary_get_ovl2mem_handle() == NULL) {
			DISPERR("display is decouple mode, but ovl2mem_path_handle is null\n");
			return;
		}

		gset_arg.is_decouple_mode = 1;
		DISPDBG("[LP]1.1 dpmanager path power on: ovl2men [begin]\n");
		dpmgr_path_power_on(primary_get_ovl2mem_handle(), CMDQ_DISABLE);
		DISPCHECK("[LP]1.1 dpmanager path power on: ovl2men [end]\n");
	}

	dpmgr_path_power_on_bypass_pwm(primary_get_dpmgr_handle(), CMDQ_DISABLE);
	DISPCHECK("[LP]1.dpmanager path power on[end]\n");
	if (disp_helper_get_option(DISP_OPT_MET_LOG))
		set_enterulps(0);

	DISPDBG("[LP]2.dpmanager path config[begin]\n");

	/* disconnect primary path first *
	* because MMsys config register may not power off during early suspend
	* BUT session mode may change in primary_display_switch_mode() */
	ddp_disconnect_path(DDP_SCENARIO_PRIMARY_ALL, NULL);
	ddp_disconnect_path(DDP_SCENARIO_PRIMARY_RDMA0_COLOR0_DISP, NULL);


	dpmgr_path_connect(primary_get_dpmgr_handle(), CMDQ_DISABLE);
	if (primary_display_is_decouple_mode())
		dpmgr_path_connect(primary_get_ovl2mem_handle(), CMDQ_DISABLE);

	data_config = dpmgr_path_get_last_config(primary_get_dpmgr_handle());
	if (primary_display_partial_support())
		primary_display_config_full_roi(data_config, primary_get_dpmgr_handle(), NULL);

	data_config->dst_dirty = 1;
	data_config->ovl_dirty = 1;
	data_config->rdma_dirty = 1;
	dpmgr_path_config(primary_get_dpmgr_handle(), data_config, NULL);

	if (primary_display_is_decouple_mode()) {

		data_config = dpmgr_path_get_last_config(primary_get_dpmgr_handle());
		data_config->rdma_dirty = 1;
		dpmgr_path_config(primary_get_dpmgr_handle(), data_config, NULL);


		data_config = dpmgr_path_get_last_config(primary_get_ovl2mem_handle());
		data_config->dst_dirty = 1;
		dpmgr_path_config(primary_get_ovl2mem_handle(), data_config, NULL);
		dpmgr_path_ioctl(primary_get_ovl2mem_handle(), NULL, DDP_OVL_GOLDEN_SETTING, &gset_arg);
	} else {
		dpmgr_path_ioctl(primary_get_dpmgr_handle(), NULL, DDP_OVL_GOLDEN_SETTING, &gset_arg);
	}


	DISPCHECK("[LP]2.dpmanager path config[end]\n");


	DISPDBG("[LP]3.dpmgr path start[begin]\n");
	dpmgr_path_start(primary_get_dpmgr_handle(), CMDQ_DISABLE);

	if (primary_display_is_decouple_mode())
		dpmgr_path_start(primary_get_ovl2mem_handle(), CMDQ_DISABLE);

	DISPCHECK("[LP]3.dpmgr path start[end]\n");
	if (dpmgr_path_is_busy(primary_get_dpmgr_handle()))
		DISPERR("[LP]3.Fatal error, we didn't trigger display path but it's already busy\n");


	if (disp_helper_get_option(DISP_OPT_USE_CMDQ)) {
		DISPDBG("[LP]4.start cmdq[begin]\n");
		_cmdq_start_trigger_loop();
		DISPCHECK("[LP]4.start cmdq[end]\n");
	}

	/* (in suspend) when we stop trigger loop
	* if no other thread is running, cmdq may disable its clock
	* all cmdq event will be cleared after suspend */
	cmdqCoreSetEvent(CMDQ_EVENT_DISP_WDMA0_EOF);
}

/* Share wrot sram end */
void _vdo_mode_enter_idle(void)
{
	DISPMSG("[disp_lowpower]%s\n", __func__);

	/* backup for DL <-> DC */
	idlemgr_pgc->session_mode_before_enter_idle = primary_get_sess_mode();

	/* DL -> DC*/
	if (!primary_is_sec() &&
	    primary_get_sess_mode() == DISP_SESSION_DIRECT_LINK_MODE &&
	    (disp_helper_get_option(DISP_OPT_IDLEMGR_SWTCH_DECOUPLE) ||
	     disp_helper_get_option(DISP_OPT_SMART_OVL))) {

		/* smart_ovl_try_switch_mode_nolock(); */
		/* switch to decouple mode */
		do_primary_display_switch_mode(DISP_SESSION_DECOUPLE_MODE, primary_get_sess_id(), 0, NULL, 0);

		set_is_dc(1);

		/* merge setting when the last one */
		/*
		if (disp_helper_get_option(DISP_OPT_DYNAMIC_RDMA_GOLDEN_SETTING))
			_idle_set_golden_setting();
		*/
	}

	/* Disable irq & increase vfp */
	if (!primary_is_sec()) {
		if (disp_helper_get_option(DISP_OPT_IDLEMGR_DISABLE_ROUTINE_IRQ)) {
			/* disable routine irq before switch to decouple mode, otherwise we need to disable two paths */
			dpmgr_path_enable_irq(primary_get_dpmgr_handle(), NULL, DDP_IRQ_LEVEL_ERROR);
		}

		if (get_lp_cust_mode() > LP_CUST_DISABLE && get_lp_cust_mode() < PERFORMANC_MODE + 1) {
			switch (get_lp_cust_mode()) {
			case LOW_POWER_MODE: /* 50 */
			case JUST_MAKE_MODE: /* 55 */
				set_fps(50);
				primary_display_dsi_vfp_change(1);
				idlemgr_pgc->cur_lp_cust_mode = 1;
				break;
			case PERFORMANC_MODE: /* 60 */
				set_fps(primary_display_get_fps_nolock()/100);
				primary_display_dsi_vfp_change(0);
				idlemgr_pgc->cur_lp_cust_mode = 0;
				break;
			}
		} else {
			if (get_backup_vfp() != primary_get_lcm()->params->dsi.vertical_frontporch_for_low_power)
				primary_get_lcm()->params->dsi.vertical_frontporch_for_low_power = get_backup_vfp();

			if (primary_get_lcm()->params->dsi.vertical_frontporch_for_low_power) {
				set_fps(50);
				primary_display_dsi_vfp_change(1);
				idlemgr_pgc->cur_lp_cust_mode = 1;
			}
		}

	}

	/* DC homeidle share wrot sram */
	/*
	if (disp_helper_get_option(DISP_OPT_SHARE_SRAM)
		&& (primary_get_sess_mode() == DISP_SESSION_DECOUPLE_MODE
		|| primary_get_sess_mode() == DISP_SESSION_RDMA_MODE))
		enter_share_sram(CMDQ_SYNC_RESOURCE_WROT0);
	*/

	/* set golden setting  , merge fps/dc */
	set_is_display_idle(1);
	if (disp_helper_get_option(DISP_OPT_DYNAMIC_RDMA_GOLDEN_SETTING))
		_idle_set_golden_setting();

	/* Enable sodi - need wait golden setting done ??? */
#if 0
#ifndef CONFIG_MTK_FPGA
#ifndef NO_SPM
	if (disp_helper_get_option(DISP_OPT_SODI_SUPPORT)) {
		/* set power down mode forbidden */
		spm_sodi_mempll_pwr_mode(1);
		spm_enable_sodi(1);
	}
#endif
#endif
#endif
}

void _vdo_mode_leave_idle(void)
{
	DISPMSG("[disp_lowpower]%s\n", __func__);

	/* Disable sodi */
#if 0
#ifndef CONFIG_MTK_FPGA
#ifndef NO_SPM
	if (disp_helper_get_option(DISP_OPT_SODI_SUPPORT))
		spm_enable_sodi(0);
#endif
#endif
#endif

	/* set golden setting */
	set_is_display_idle(0);
	if (disp_helper_get_option(DISP_OPT_DYNAMIC_RDMA_GOLDEN_SETTING))
		_idle_set_golden_setting();

	/* DC homeidle share wrot sram */
	/*
	if (disp_helper_get_option(DISP_OPT_SHARE_SRAM)
		&& (primary_get_sess_mode() == DISP_SESSION_DECOUPLE_MODE
		|| primary_get_sess_mode() == DISP_SESSION_RDMA_MODE))
		leave_share_sram(CMDQ_SYNC_RESOURCE_WROT0);
	*/

	/* Enable irq & restore vfp */
	if (!primary_is_sec()) {

		if (idlemgr_pgc->cur_lp_cust_mode != 0) {
			set_fps(primary_display_get_fps_nolock()/100);
			primary_display_dsi_vfp_change(0);
			idlemgr_pgc->cur_lp_cust_mode = 0;
			if (disp_helper_get_option(DISP_OPT_DYNAMIC_RDMA_GOLDEN_SETTING))
				_idle_set_golden_setting();
		}
		if (disp_helper_get_option(DISP_OPT_IDLEMGR_DISABLE_ROUTINE_IRQ)) {
			/* enable routine irq after switch to directlink mode, otherwise we need to disable two paths */
			dpmgr_path_enable_irq(primary_get_dpmgr_handle(), NULL, DDP_IRQ_LEVEL_ALL);
		}
	}

	/* DC -> DL */
	if (disp_helper_get_option(DISP_OPT_IDLEMGR_SWTCH_DECOUPLE) &&
		!disp_helper_get_option(DISP_OPT_SMART_OVL)) {
		/* switch to the mode before idle */
		do_primary_display_switch_mode(idlemgr_pgc->session_mode_before_enter_idle,
			primary_get_sess_id(), 0, NULL, 0);

		set_is_dc(0);
		if (disp_helper_get_option(DISP_OPT_DYNAMIC_RDMA_GOLDEN_SETTING))
			_idle_set_golden_setting();
	}
}

void _cmd_mode_enter_idle(void)
{
	DISPMSG("[disp_lowpower]%s\n", __func__);

	/* need leave share sram for disable mmsys clk */
	if (disp_helper_get_option(DISP_OPT_SHARE_SRAM))
		leave_share_sram(CMDQ_SYNC_RESOURCE_WROT1);

	/* please keep last */
	if (disp_helper_get_option(DISP_OPT_IDLEMGR_ENTER_ULPS)) {
		/* switch to vencpll before disable mmsys clk */
		if (disp_helper_get_option(DISP_OPT_DYNAMIC_SWITCH_MMSYSCLK))
			;/*mmdvfs_notify_mmclk_switch_request(MMDVFS_EVENT_OVL_SINGLE_LAYER_EXIT);*/
		/* need delay to make sure done??? */
		_primary_display_disable_mmsys_clk();
	}
}

void _cmd_mode_leave_idle(void)
{
	DISPMSG("[disp_lowpower]%s\n", __func__);

	if (disp_helper_get_option(DISP_OPT_IDLEMGR_ENTER_ULPS))
		_primary_display_enable_mmsys_clk();


	if (disp_helper_get_option(DISP_OPT_SHARE_SRAM))
		enter_share_sram(CMDQ_SYNC_RESOURCE_WROT1);
}

void primary_display_idlemgr_enter_idle_nolock(void)
{
	if (primary_display_is_video_mode())
		_vdo_mode_enter_idle();
	else
		_cmd_mode_enter_idle();
}

void primary_display_idlemgr_leave_idle_nolock(void)
{
	if (primary_display_is_video_mode())
		_vdo_mode_leave_idle();
	else
		_cmd_mode_leave_idle();
}

static int _primary_path_idlemgr_monitor_thread(void *data)
{
	int ret = 0;

	msleep(16000);
	while (1) {
		msleep(20); /* 20ms */
		ret = wait_event_interruptible(idlemgr_pgc->idlemgr_wait_queue, atomic_read(&idlemgr_task_wakeup));

		primary_display_manual_lock();

		if (primary_get_state() != DISP_ALIVE) {
			primary_display_manual_unlock();
			primary_display_wait_state(DISP_ALIVE, MAX_SCHEDULE_TIMEOUT);
			continue;
		}

		if (primary_display_is_idle()) {
			primary_display_manual_unlock();
			continue;
		}

#ifdef CONFIG_MTK_DISPLAY_120HZ_SUPPORT
		if (primary_display_get_lcm_refresh_rate() == 120) {
			primary_display_manual_unlock();
			continue;
		}
#endif
		if (((local_clock() - idlemgr_pgc->idlemgr_last_kick_time) / 1000) < 50000 /* 50ms */) {
			/* kicked in 50ms, it's not idle */
			primary_display_manual_unlock();
			continue;
		}
		MMProfileLogEx(ddp_mmp_get_events()->idlemgr, MMProfileFlagStart, 0, 0);
		DISPMSG("[disp_lowpower]primary enter idle state\n");

		/* enter idle state */
		primary_display_idlemgr_enter_idle_nolock();
		primary_display_set_idle_stat(1);

		primary_display_manual_unlock();

		wait_event_interruptible(idlemgr_pgc->idlemgr_wait_queue, !primary_display_is_idle());

		if (kthread_should_stop())
			break;
	}

	return 0;
}

void kick_logger_dump(char *string)
{
	if (kick_buf_length + strlen(string) >= kick_dump_max_length)
		kick_logger_dump_reset();

	kick_buf_length += scnprintf(kick_string_buffer_analysize + kick_buf_length,
					kick_dump_max_length - kick_buf_length, string);
}

void kick_logger_dump_reset(void)
{
	kick_buf_length = 0;
	memset(kick_string_buffer_analysize, 0, sizeof(kick_string_buffer_analysize));
}

char *get_kick_dump(void)
{
	return kick_string_buffer_analysize;
}

unsigned int get_kick_dump_size(void)
{
	return kick_buf_length;
}

/* API */
/*********************************************************************************************************************/
golden_setting_context *get_golden_setting_pgc(void)
{
	return golden_setting_pgc;
}

/* for met - begin */
unsigned int is_mipi_enterulps(void)
{
	return idlemgr_pgc->enterulps;
}

unsigned int get_mipi_clk(void)
{
	if (disp_helper_get_option(DISP_OPT_MET_LOG)) {
		if (is_mipi_enterulps())
			return 0;
		else
			return dsi_phy_get_clk(DISP_MODULE_NUM);
	} else {
		return 0;
	}
}

void primary_display_sodi_enable(int flag)
{
	spm_enable_sodi(flag);
}

/* for met - end */
void primary_display_sodi_rule_init(void)
{
	/* enable sodi when display driver is ready */
#ifndef CONFIG_MTK_FPGA
#ifndef NO_SPM
	if (primary_display_is_video_mode()) {
		spm_sodi_set_vdo_mode(1);
		spm_sodi_mempll_pwr_mode(1);
		spm_enable_sodi3(0);
		spm_enable_sodi(1);
	} else {
		spm_enable_sodi3(1);
		spm_enable_sodi(1);
	}
#endif
#endif
}

int primary_display_lowpower_init(void)
{
	set_fps(primary_display_get_fps_nolock()/100);
	backup_vfp_for_lp_cust(primary_get_lcm()->params->dsi.vertical_frontporch_for_low_power);
	/* init idlemgr */
	if (disp_helper_get_option(DISP_OPT_IDLE_MGR) && get_boot_mode() == NORMAL_BOOT)
		primary_display_idlemgr_init();


	if (disp_helper_get_option(DISP_OPT_SODI_SUPPORT))
		primary_display_sodi_rule_init();

	if (disp_helper_get_option(DISP_OPT_DYNAMIC_SWITCH_MMSYSCLK))
		/* callback with lock or without lock ??? */
		;/*register_mmclk_switch_cb(primary_display_switch_mmsys_clk, _switch_mmsys_clk);*/

	/* cmd mode always enable share sram */
	if (disp_helper_get_option(DISP_OPT_SHARE_SRAM))
		enter_share_sram(CMDQ_SYNC_RESOURCE_WROT1);

	return 0;
}

int primary_display_is_idle(void)
{
	return idlemgr_pgc->is_primary_idle;
}

void primary_display_idlemgr_kick(const char *source, int need_lock)
{
	char log[128] = "";

	MMProfileLogEx(ddp_mmp_get_events()->idlemgr, MMProfileFlagPulse, 1, 0);

	snprintf(log, sizeof(log), "[kick]%s kick at %lld\n", source, sched_clock());
	kick_logger_dump(log);

	/* get primary lock to protect idlemgr_last_kick_time and primary_display_is_idle() */
	if (need_lock)
		primary_display_manual_lock();

	/* update kick timestamp */
	idlemgr_pgc->idlemgr_last_kick_time = sched_clock();

	if (primary_display_is_idle()) {
		primary_display_idlemgr_leave_idle_nolock();
		primary_display_set_idle_stat(0);

		MMProfileLogEx(ddp_mmp_get_events()->idlemgr, MMProfileFlagEnd, 0, 0);
		/* wake up idlemgr process to monitor next idle stat */
		wake_up_interruptible(&(idlemgr_pgc->idlemgr_wait_queue));
	}

	if (need_lock)
		primary_display_manual_unlock();
}

#if 0
void exit_pd_by_cmdq(cmdqRecHandle handler)
{
	/* Enable SPM CG Mode(Force 30+ times to ensure write success, need find root cause and fix later) */
	cmdqRecWrite(handler, 0x100062B0, 0x2, ~0);
	/* Polling EMI Status to ensure EMI is enabled */
	cmdqRecPoll(handler, 0x1000611C, 0x10000, 0x10000);
}

void enter_pd_by_cmdq(cmdqRecHandle handler)
{
	cmdqRecWrite(handler, 0x100062B0, 0x0, 0x2);
}
#endif

void enter_share_sram(CMDQ_EVENT_ENUM resourceEvent)
{
	/* 1. register call back first */
	cmdqCoreSetResourceCallback(CMDQ_SYNC_RESOURCE_WROT1,
		_acquire_wrot_resource, _release_wrot_resource);

	/* 2. try to allocate sram at the fisrt time */
	_acquire_wrot_resource_nolock(CMDQ_SYNC_RESOURCE_WROT1);
}

void leave_share_sram(CMDQ_EVENT_ENUM resourceEvent)
{
	/* 1. unregister call back */
	cmdqCoreSetResourceCallback(CMDQ_SYNC_RESOURCE_WROT1, NULL, NULL);

	/* 2. try to release share sram */
	_release_wrot_resource_nolock(CMDQ_SYNC_RESOURCE_WROT1);
}

void set_hrtnum(unsigned int new_hrtnum)
{
	if (golden_setting_pgc->hrt_num != new_hrtnum) {
		if ((golden_setting_pgc->hrt_num > golden_setting_pgc->hrt_magicnum
			&& new_hrtnum <= golden_setting_pgc->hrt_magicnum)
			|| (golden_setting_pgc->hrt_num <= golden_setting_pgc->hrt_magicnum
			&& new_hrtnum > golden_setting_pgc->hrt_magicnum)) {
			/* should not on screenidle when set hrtnum */
			if (new_hrtnum > golden_setting_pgc->hrt_magicnum)
				golden_setting_pgc->fifo_mode = 2;
			else
				golden_setting_pgc->fifo_mode = 1;

		}
		golden_setting_pgc->hrt_num = new_hrtnum;
	}
}

/* set enterulps flag after power on & power off */
void set_enterulps(unsigned flag)
{
	idlemgr_pgc->enterulps = flag;
}

void set_is_dc(unsigned int is_dc)
{
	if (golden_setting_pgc->is_dc != is_dc)
		golden_setting_pgc->is_dc = is_dc;
}

/* return 0: no change / return 1: change */
unsigned int set_one_layer(unsigned int is_onelayer)
{
	if (golden_setting_pgc->is_one_layer == is_onelayer)
		return 0;

	golden_setting_pgc->is_one_layer = is_onelayer;

	return 1;
}

void set_rdma_width_height(unsigned int width, unsigned height)
{
	golden_setting_pgc->rdma_width = width;
	golden_setting_pgc->rdma_height = height;
}

void enable_idlemgr(unsigned int flag)
{
	if (flag) {
		DISPCHECK("[disp_lowpower]enable idlemgr\n");
		atomic_set(&idlemgr_task_wakeup, 1);
		wake_up_interruptible(&(idlemgr_pgc->idlemgr_wait_queue));
	} else {
		DISPCHECK("[disp_lowpower]disable idlemgr\n");
		atomic_set(&idlemgr_task_wakeup, 0);
		primary_display_idlemgr_kick((char *)__func__, 1);
	}
}

unsigned int get_idlemgr_flag(void)
{
	unsigned int idlemgr_flag;

	idlemgr_flag = atomic_read(&idlemgr_task_wakeup);
	return idlemgr_flag;
}

unsigned int set_idlemgr(unsigned int flag, int need_lock)
{
	unsigned int old_flag = atomic_read(&idlemgr_task_wakeup);

	if (flag) {
		DISPCHECK("[disp_lowpower]enable idlemgr\n");
		atomic_set(&idlemgr_task_wakeup, 1);
		wake_up_interruptible(&(idlemgr_pgc->idlemgr_wait_queue));
	} else {
		DISPCHECK("[disp_lowpower]disable idlemgr\n");
		atomic_set(&idlemgr_task_wakeup, 0);
		primary_display_idlemgr_kick((char *)__func__, need_lock);
	}
	return old_flag;
}
