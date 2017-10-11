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

#define LOG_TAG "ddp_manager"

#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/bug.h>

#include "disp_helper.h"
#include "lcm_drv.h"
#include "ddp_reg.h"
#include "ddp_path.h"
#include "ddp_irq.h"
#include "ddp_drv.h"
#include "disp_debug.h"
#include "mtkfb_debug.h"
#include "ddp_manager.h"
#include "ddp_rdma.h"
#include "ddp_ovl.h"

#include "disp_log.h"
/* #pragma GCC optimize("O0") */

static int ddp_manager_init;
#define DDP_MAX_MANAGER_HANDLE (DISP_MUTEX_DDP_COUNT+DISP_MUTEX_DDP_FIRST)

typedef struct {
	volatile unsigned int init;
	DISP_PATH_EVENT event;
	wait_queue_head_t wq;

	volatile unsigned long long data;
} DPMGR_WQ_HANDLE;

typedef struct {
	DDP_IRQ_BIT irq_bit;
} DDP_IRQ_EVENT_MAPPING;

typedef struct {
	cmdqRecHandle cmdqhandle;
	int hwmutexid;
	int power_sate;
	DDP_MODE mode;
	struct mutex mutex_lock;
	DDP_IRQ_EVENT_MAPPING irq_event_map[DISP_PATH_EVENT_NUM];
	DPMGR_WQ_HANDLE wq_list[DISP_PATH_EVENT_NUM];
	DDP_SCENARIO_ENUM scenario;
	DISP_MODULE_ENUM mem_module;
	disp_ddp_path_config last_config;
} ddp_path_handle_t, *ddp_path_handle;

typedef struct {
	int handle_cnt;
	int mutex_idx;
	int power_sate;
	struct mutex mutex_lock;
	int module_usage_table[DISP_MODULE_NUM];
	ddp_path_handle module_path_table[DISP_MODULE_NUM];
	ddp_path_handle handle_pool[DDP_MAX_MANAGER_HANDLE];
} DDP_MANAGER_CONTEXT;

#define DEFAULT_IRQ_EVENT_SCENARIO (4)
static DDP_IRQ_EVENT_MAPPING ddp_irq_event_list[DEFAULT_IRQ_EVENT_SCENARIO][DISP_PATH_EVENT_NUM] = {
	{			/* ovl0 path */
	 {DDP_IRQ_RDMA0_DONE},	/*FRAME_DONE */
	 {DDP_IRQ_RDMA0_START},	/*FRAME_START */
	 {DDP_IRQ_RDMA0_REG_UPDATE},	/*FRAME_REG_UPDATE */
	 {DDP_IRQ_RDMA0_TARGET_LINE},	/*FRAME_TARGET_LINE */
	 {DDP_IRQ_WDMA0_FRAME_COMPLETE},	/*FRAME_COMPLETE */
	 {DDP_IRQ_RDMA0_TARGET_LINE},	/*FRAME_STOP */
	 {DDP_IRQ_RDMA0_REG_UPDATE},	/*IF_CMD_DONE */
	 {DDP_IRQ_DSI0_EXT_TE},	/*IF_VSYNC */
	 {DDP_IRQ_UNKNOWN}, /*TRIGER*/ {DDP_IRQ_AAL_OUT_END_FRAME},	/*AAL_OUT_END_EVENT */
	 },
	{			/* ovl1 path */
	 {DDP_IRQ_RDMA1_DONE},	/*FRAME_DONE */
	 {DDP_IRQ_RDMA1_START},	/*FRAME_START */
	 {DDP_IRQ_RDMA1_REG_UPDATE},	/*FRAME_REG_UPDATE */
	 {DDP_IRQ_RDMA1_TARGET_LINE},	/*FRAME_TARGET_LINE */
	 {DDP_IRQ_WDMA1_FRAME_COMPLETE},	/*FRAME_COMPLETE */
	 {DDP_IRQ_RDMA1_TARGET_LINE},	/*FRAME_STOP */
	 {DDP_IRQ_RDMA1_REG_UPDATE},	/*IF_CMD_DONE */
	 {DDP_IRQ_RDMA1_TARGET_LINE},	/*IF_VSYNC */
	 {DDP_IRQ_UNKNOWN}, /*TRIGER*/ {DDP_IRQ_UNKNOWN},	/*AAL_OUT_END_EVENT */
	 },
	{			/* rdma path */
	 {DDP_IRQ_RDMA2_DONE},	/*FRAME_DONE */
	 {DDP_IRQ_RDMA2_START},	/*FRAME_START */
	 {DDP_IRQ_RDMA2_REG_UPDATE},	/*FRAME_REG_UPDATE */
	 {DDP_IRQ_RDMA2_TARGET_LINE},	/*FRAME_TARGET_LINE */
	 {DDP_IRQ_UNKNOWN},	/*FRAME_COMPLETE */
	 {DDP_IRQ_RDMA2_TARGET_LINE},	/*FRAME_STOP */
	 {DDP_IRQ_RDMA2_REG_UPDATE},	/*IF_CMD_DONE */
	 {DDP_IRQ_RDMA2_TARGET_LINE},	/*IF_VSYNC */
	 {DDP_IRQ_UNKNOWN}, /*TRIGER*/ {DDP_IRQ_UNKNOWN},	/*AAL_OUT_END_EVENT */
	 },
	{			/* ovl0 path */
	 {DDP_IRQ_RDMA0_DONE},	/*FRAME_DONE */
	 {DDP_IRQ_MUTEX1_SOF},	/*FRAME_START */
	 {DDP_IRQ_RDMA0_REG_UPDATE},	/*FRAME_REG_UPDATE */
	 {DDP_IRQ_RDMA0_TARGET_LINE},	/*FRAME_TARGET_LINE */
	 {DDP_IRQ_WDMA0_FRAME_COMPLETE},	/*FRAME_COMPLETE */
	 {DDP_IRQ_RDMA0_TARGET_LINE},	/*FRAME_STOP */
	 {DDP_IRQ_RDMA0_REG_UPDATE},	/*IF_CMD_DONE */
	 {DDP_IRQ_DSI0_EXT_TE},	/*IF_VSYNC */
	 {DDP_IRQ_UNKNOWN}, /*TRIGER*/ {DDP_IRQ_AAL_OUT_END_FRAME},	/*AAL_OUT_END_EVENT */
	 }
};

static char *path_event_name(DISP_PATH_EVENT event)
{
	switch (event) {
	case DISP_PATH_EVENT_FRAME_START:
		return "FRAME_START";
	case DISP_PATH_EVENT_FRAME_DONE:
		return "FRAME_DONE";
	case DISP_PATH_EVENT_FRAME_REG_UPDATE:
		return "REG_UPDATE";
	case DISP_PATH_EVENT_FRAME_TARGET_LINE:
		return "TARGET_LINE";
	case DISP_PATH_EVENT_FRAME_COMPLETE:
		return "FRAME COMPLETE";
	case DISP_PATH_EVENT_FRAME_STOP:
		return "FRAME_STOP";
	case DISP_PATH_EVENT_IF_CMD_DONE:
		return "FRAME_STOP";
	case DISP_PATH_EVENT_IF_VSYNC:
		return "VSYNC";
	case DISP_PATH_EVENT_TRIGGER:
		return "TRIGGER";
	case DISP_PATH_EVENT_DELAYED_TRIGGER_33ms:
		return "DELAY_TRIG";
	default:
		return "unknown event";
	}
	return "unknown event";
}

static DDP_MANAGER_CONTEXT *_get_context(void)
{
	static int is_context_inited;
	static DDP_MANAGER_CONTEXT context;

	if (!is_context_inited) {
		memset((void *)&context, 0, sizeof(DDP_MANAGER_CONTEXT));
		context.mutex_idx = (1 << DISP_MUTEX_DDP_COUNT) - 1;
		mutex_init(&context.mutex_lock);
		is_context_inited = 1;
	}
	return &context;
}

static int path_top_clock_off(void)
{
	int i = 0;
	DDP_MANAGER_CONTEXT *context = _get_context();

	if (!context->power_sate)
		return 0;
	for (i = 0; i < DDP_MAX_MANAGER_HANDLE; i++) {
		if (context->handle_pool[i] != NULL
			  && context->handle_pool[i]->power_sate != 0)
			return 0;
	}
	context->power_sate = 0;
	ddp_path_top_clock_off();
	return 0;
}

static int path_top_clock_on(void)
{
	DDP_MANAGER_CONTEXT *context = _get_context();

	if (context->power_sate)
		return 0;
	context->power_sate = 1;
	ddp_path_top_clock_on();
	return 0;
}

static int module_power_off(DISP_MODULE_ENUM module)
{
	if (module == DISP_MODULE_DSI0)
		return 0;

	if (ddp_modules_driver[module] != 0) {
		if (ddp_modules_driver[module]->power_off != 0) {
			DISPDBG("%s power off\n", ddp_get_module_name(module));
			ddp_modules_driver[module]->power_off(module, NULL);	/* now just 0; */
		}
	}
	return 0;
}

static int module_power_on(DISP_MODULE_ENUM module)
{
	if (module == DISP_MODULE_DSI0)
		return 0;

	if (ddp_modules_driver[module] != 0) {
		if (ddp_modules_driver[module]->power_on != 0) {
			DISPDBG("%s power on\n", ddp_get_module_name(module));
			ddp_modules_driver[module]->power_on(module, NULL);	/* now just 0; */
		}
	}
	return 0;
}

static ddp_path_handle find_handle_by_module(DISP_MODULE_ENUM module)
{
	return _get_context()->module_path_table[module];
}

int dpmgr_module_notify(DISP_MODULE_ENUM module, DISP_PATH_EVENT event)
{
	ddp_path_handle handle = find_handle_by_module(module);

	MMProfileLogEx(ddp_mmp_get_events()->primary_display_aalod_trigger, MMProfileFlagPulse,
		       module, event);
	return dpmgr_signal_event(handle, event);
	return 0;
}

static int assign_default_irqs_table(DDP_SCENARIO_ENUM scenario, DDP_IRQ_EVENT_MAPPING *irq_events)
{
	int idx = 0;

	switch (scenario) {
	case DDP_SCENARIO_PRIMARY_DISP:
	case DDP_SCENARIO_PRIMARY_RDMA0_COLOR0_DISP:
	case DDP_SCENARIO_PRIMARY_RDMA0_DISP:
	case DDP_SCENARIO_PRIMARY_BYPASS_RDMA:
	case DDP_SCENARIO_PRIMARY_DITHER_MEMOUT:
	case DDP_SCENARIO_PRIMARY_UFOE_MEMOUT:
	case DDP_SCENARIO_PRIMARY_ALL:
	case DDP_SCENARIO_DITHER_1TO2:
	case DDP_SCENARIO_UFOE_1TO2:
		idx = 0;
		break;
	case DDP_SCENARIO_SUB_DISP:
	case DDP_SCENARIO_SUB_RDMA1_DISP:
	case DDP_SCENARIO_SUB_OVL_MEMOUT:
	case DDP_SCENARIO_SUB_ALL:
		idx = 1;
		break;
	case DDP_SCENARIO_PRIMARY_OVL_MEMOUT:
		idx = 3;
		break;
	default:
		DISPERR("unknown scenario %d\n", scenario);
	}
	memcpy(irq_events, ddp_irq_event_list[idx], sizeof(ddp_irq_event_list[idx]));
	return 0;
}

static int acquire_mutex(DDP_SCENARIO_ENUM scenario)
{
/* /: primay use mutex 0 */
	int mutex_id = 0;
	DDP_MANAGER_CONTEXT *content = _get_context();
	int mutex_idx_free = content->mutex_idx;

	ASSERT(scenario >= 0 && scenario < DDP_SCENARIO_MAX);
	while (mutex_idx_free) {
		if (mutex_idx_free & 0x1) {
			content->mutex_idx &= (~(0x1 << mutex_id));
			mutex_id += DISP_MUTEX_DDP_FIRST;
			break;
		}
		mutex_idx_free >>= 1;
		++mutex_id;
	}
	ASSERT(mutex_id < (DISP_MUTEX_DDP_FIRST + DISP_MUTEX_DDP_COUNT));
	DISPDBG("scenario %s acquire mutex %d , left mutex 0x%x!\n",
		   ddp_get_scenario_name(scenario), mutex_id, content->mutex_idx);
	return mutex_id;
}

static int release_mutex(int mutex_idx)
{
	DDP_MANAGER_CONTEXT *content = _get_context();

	ASSERT(mutex_idx < (DISP_MUTEX_DDP_FIRST + DISP_MUTEX_DDP_COUNT));
	content->mutex_idx |= 1 << (mutex_idx - DISP_MUTEX_DDP_FIRST);
	DISPDBG("release mutex %d , left mutex 0x%x!\n", mutex_idx, content->mutex_idx);
	return 0;
}

int dpmgr_path_set_video_mode(disp_path_handle dp_handle, int is_vdo_mode)
{
	ddp_path_handle handle = NULL;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	handle->mode = is_vdo_mode ? DDP_VIDEO_MODE : DDP_CMD_MODE;
	DISPDBG("set scenario %s mode: %s\n", ddp_get_scenario_name(handle->scenario),
		   is_vdo_mode ? "Video_Mode" : "Cmd_Mode");
	return 0;
}

disp_path_handle dpmgr_create_path(DDP_SCENARIO_ENUM scenario, cmdqRecHandle cmdq_handle)
{
	int i = 0;
	int module_name;
	ddp_path_handle path_handle = NULL;
	int *modules = ddp_get_scenario_list(scenario);
	int module_num = ddp_get_module_num(scenario);
	DDP_MANAGER_CONTEXT *content = _get_context();

	path_handle = kzalloc(sizeof(ddp_path_handle_t), GFP_KERNEL);
	if (NULL != path_handle) {
		path_handle->cmdqhandle = cmdq_handle;
		path_handle->scenario = scenario;
		path_handle->hwmutexid = acquire_mutex(scenario);
		path_handle->last_config.path_handle = path_handle;
		assign_default_irqs_table(scenario, path_handle->irq_event_map);
		DISPDBG("create handle %p on scenario %s\n", path_handle,
			   ddp_get_scenario_name(scenario));
		for (i = 0; i < module_num; i++) {
			module_name = modules[i];
			DISPDBG(" scenario %s include module %s\n",
				   ddp_get_scenario_name(scenario),
				   ddp_get_module_name(module_name));
			content->module_usage_table[module_name]++;
			content->module_path_table[module_name] = path_handle;
		}
		content->handle_cnt++;
		content->handle_pool[path_handle->hwmutexid] = path_handle;
	} else {
		DISPERR("Fail to create handle on scenario %s\n",
			   ddp_get_scenario_name(scenario));
	}
	return path_handle;
}

int dpmgr_get_scenario(disp_path_handle dp_handle)
{
	ddp_path_handle handle;

	handle = (ddp_path_handle) dp_handle;
	return handle->scenario;
}

static int _dpmgr_path_connect(DDP_SCENARIO_ENUM scenario, void *handle)
{
	int i = 0, module;
	int *modules = ddp_get_scenario_list(scenario);
	int module_num = ddp_get_module_num(scenario);

	ddp_connect_path(scenario, handle);

	for (i = 0; i < module_num; i++) {
		module = modules[i];
		if (ddp_modules_driver[module] && ddp_modules_driver[module]->connect) {
			int prev = i == 0 ? DISP_MODULE_UNKNOWN : modules[i - 1];
			int next = i == module_num - 1 ? DISP_MODULE_UNKNOWN : modules[i + 1];

			ddp_modules_driver[module]->connect(module, prev, next, 1, handle);
		}
	}

	return 0;
}

static int _dpmgr_path_disconnect(DDP_SCENARIO_ENUM scenario, void *handle)
{
	int i = 0, module;
	int *modules = ddp_get_scenario_list(scenario);
	int module_num = ddp_get_module_num(scenario);

	ddp_disconnect_path(scenario, handle);

	for (i = 0; i < module_num; i++) {
		module = modules[i];
		if (ddp_modules_driver[module] && ddp_modules_driver[module]->connect) {
			int prev = i == 0 ? DISP_MODULE_UNKNOWN : modules[i - 1];
			int next = i == module_num - 1 ? DISP_MODULE_UNKNOWN : modules[i + 1];

			ddp_modules_driver[module]->connect(module, prev, next, 1, handle);
		}
	}

	return 0;
}


int dpmgr_modify_path_power_on_new_modules(disp_path_handle dp_handle,
					   DDP_SCENARIO_ENUM new_scenario, int sw_only)
{
	int i = 0;
	int module_name = 0;
	DDP_MANAGER_CONTEXT *content = _get_context();
	ddp_path_handle handle;
	int *new_modules;
	int new_module_num;

	handle = (ddp_path_handle) dp_handle;
	new_modules = ddp_get_scenario_list(new_scenario);
	new_module_num = ddp_get_module_num(new_scenario);

	for (i = 0; i < new_module_num; i++) {
		module_name = new_modules[i];
		if (content->module_usage_table[module_name] == 0) {	/* new module 's count =0 */
			content->module_usage_table[module_name]++;
			content->module_path_table[module_name] = handle;
			if (!sw_only)
				module_power_on(module_name);
		}
	}
	return 0;

}

/* NOTES: modify path should call API like this :
	old_scenario = dpmgr_get_scenario(handle);
	dpmgr_modify_path_power_on_new_modules();
	dpmgr_modify_path();
  after cmdq handle exec done:
	dpmgr_modify_path_power_off_old_modules();
*/
int dpmgr_modify_path(disp_path_handle dp_handle, DDP_SCENARIO_ENUM new_scenario,
			cmdqRecHandle cmdq_handle, DDP_MODE mode, int sw_only)
{
	ddp_path_handle handle;
	DDP_SCENARIO_ENUM old_scenario;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	old_scenario = handle->scenario;
	handle->cmdqhandle = cmdq_handle;
	handle->scenario = new_scenario;
	DISPMSG("modify handle %p from %s to %s\n", handle, ddp_get_scenario_name(old_scenario),
		   ddp_get_scenario_name(new_scenario));

	if (!sw_only) {
		/* mutex set will clear old settings */
		ddp_mutex_set(handle->hwmutexid, new_scenario, mode, cmdq_handle);

		ddp_mutex_Interrupt_enable(handle->hwmutexid, cmdq_handle);
		/* disconnect old path first */
		_dpmgr_path_disconnect(old_scenario, cmdq_handle);

		/* connect new path */
		_dpmgr_path_connect(new_scenario, cmdq_handle);

	}
	return 0;
}

int dpmgr_modify_path_power_off_old_modules(DDP_SCENARIO_ENUM old_scenario,
					    DDP_SCENARIO_ENUM new_scenario, int sw_only)
{
	int i = 0;
	int module_name = 0;

	DDP_MANAGER_CONTEXT *content = _get_context();

	int *old_modules = ddp_get_scenario_list(old_scenario);
	int old_module_num = ddp_get_module_num(old_scenario);

	for (i = 0; i < old_module_num; i++) {
		module_name = old_modules[i];
		if (!ddp_is_module_in_scenario(new_scenario, module_name)) {
			content->module_usage_table[module_name]--;
			content->module_path_table[module_name] = NULL;
			if (!sw_only)
				module_power_off(module_name);
		}
	}

	return 0;
}


int dpmgr_destroy_path_handle(disp_path_handle dp_handle)
{
	int i = 0;
	int module_name;
	ddp_path_handle handle;
	int *modules;
	int module_num;
	DDP_MANAGER_CONTEXT *content;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;

	if (handle != NULL) {
		modules = ddp_get_scenario_list(handle->scenario);
		module_num = ddp_get_module_num(handle->scenario);
		content = _get_context();

		DISPDBG("destroy path handle %p on scenario %s\n", handle,
		ddp_get_scenario_name(handle->scenario));

		release_mutex(handle->hwmutexid);
		for (i = 0; i < module_num; i++) {
			module_name = modules[i];
			content->module_usage_table[module_name]--;
			content->module_path_table[module_name] = NULL;
		}
		content->handle_cnt--;
		ASSERT(content->handle_cnt >= 0);
		content->handle_pool[handle->hwmutexid] = NULL;
		kfree(handle);
	}
	return 0;
}

int dpmgr_destroy_path(disp_path_handle dp_handle, cmdqRecHandle cmdq_handle)
{

	ddp_path_handle handle;

	handle = (ddp_path_handle) dp_handle;

	if (handle)
		_dpmgr_path_disconnect(handle->scenario, cmdq_handle);

	dpmgr_destroy_path_handle(dp_handle);
	return 0;
}

int dpmgr_path_memout_clock(disp_path_handle dp_handle, int clock_switch)
{
#if 0
	ASSERT(dp_handle != NULL);
	ddp_path_handle handle;

	handle = (ddp_path_handle) dp_handle;
	handle->mem_module =
	    ddp_is_scenario_on_primary(handle->scenario) ? DISP_MODULE_WDMA0 : DISP_MODULE_WDMA1;
	if (handle->mem_module == DISP_MODULE_WDMA0 || handle->mem_module == DISP_MODULE_WDMA1) {
		if (ddp_modules_driver[handle->mem_module] != 0) {
			if (clock_switch) {
				if (ddp_modules_driver[handle->mem_module]->power_on != 0)
					ddp_modules_driver[handle->mem_module]->power_on(handle->
											 mem_module,
											 NULL);

			} else {
				if (ddp_modules_driver[handle->mem_module]->power_off != 0)
					ddp_modules_driver[handle->mem_module]->power_off(handle->
											  mem_module,
											  NULL);

				handle->mem_module = DISP_MODULE_UNKNOWN;
			}
		}
		return 0;
	}
	return -1;
#endif
	return 0;
}

int dpmgr_path_add_memout(disp_path_handle dp_handle, DISP_MODULE_ENUM engine, void *cmdq_handle)
{

	ddp_path_handle handle;
	DISP_MODULE_ENUM wdma;
	DDP_MANAGER_CONTEXT *context;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	if (handle->scenario != DDP_SCENARIO_PRIMARY_DISP
	       && handle->scenario != DDP_SCENARIO_SUB_DISP
	       && handle->scenario != DDP_SCENARIO_PRIMARY_OVL_MEMOUT){
		DISPERR("dpmgr_path_add_memout error, wdma can not be added to scenario=%s\n",
			       ddp_get_scenario_name(handle->scenario));
		return -1;
	} else {
		wdma =
		    ddp_is_scenario_on_primary(handle->scenario) ? DISP_MODULE_WDMA0 : DISP_MODULE_WDMA1;

		if (ddp_is_module_in_scenario(handle->scenario, wdma) == 1) {
			DISPERR("dpmgr_path_add_memout error, wdma is already in scenario=%s\n",
			       ddp_get_scenario_name(handle->scenario));
			return -1;
		}
		/* update contxt */
		context = _get_context();

		context->module_usage_table[wdma]++;
		context->module_path_table[wdma] = handle;
		if (engine == DISP_MODULE_OVL0) {
			handle->scenario = DDP_SCENARIO_PRIMARY_ALL;
		} else if (engine == DISP_MODULE_OVL1) {
			handle->scenario = DDP_SCENARIO_SUB_ALL;
		} else if (engine == DISP_MODULE_DITHER) {
			handle->scenario = DDP_SCENARIO_DITHER_1TO2;
		} else if (engine == DISP_MODULE_UFOE) {
			handle->scenario = DDP_SCENARIO_UFOE_1TO2;
		} else {
			pr_err("%s error: engine=%d\n", __func__, engine);
			BUG();
		}
		/* update connected */
		_dpmgr_path_connect(handle->scenario, cmdq_handle);
		ddp_mutex_set(handle->hwmutexid, handle->scenario, handle->mode, cmdq_handle);

		/* wdma just need start. */
		if (ddp_modules_driver[wdma] != 0) {
			if (ddp_modules_driver[wdma]->init != 0)
				ddp_modules_driver[wdma]->init(wdma, cmdq_handle);

			if (ddp_modules_driver[wdma]->start != 0)
				ddp_modules_driver[wdma]->start(wdma, cmdq_handle);

		}
		return 0;
	}
}

int dpmgr_path_remove_memout(disp_path_handle dp_handle, void *cmdq_handle)
{

	DDP_SCENARIO_ENUM dis_scn, new_scn;
	ddp_path_handle handle;
	DISP_MODULE_ENUM wdma;
	DDP_MANAGER_CONTEXT *context;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	ASSERT(handle->scenario == DDP_SCENARIO_PRIMARY_DISP
	       || handle->scenario == DDP_SCENARIO_PRIMARY_ALL
	       || handle->scenario == DDP_SCENARIO_DITHER_1TO2
	       || handle->scenario == DDP_SCENARIO_UFOE_1TO2
	       || handle->scenario == DDP_SCENARIO_SUB_DISP
	       || handle->scenario == DDP_SCENARIO_SUB_ALL);
	wdma =
	    ddp_is_scenario_on_primary(handle->scenario) ? DISP_MODULE_WDMA0 : DISP_MODULE_WDMA1;

	if (ddp_is_module_in_scenario(handle->scenario, wdma) == 0) {
		DISPERR("dpmgr_path_remove_memout error, wdma is not in scenario=%s\n",
		       ddp_get_scenario_name(handle->scenario));
		return -1;
	}
	/* update contxt */
	context = _get_context();

	context->module_usage_table[wdma]--;
	context->module_path_table[wdma] = 0;
	/* wdma just need stop */
	if (ddp_modules_driver[wdma] != 0) {
		if (ddp_modules_driver[wdma]->stop != 0)
			ddp_modules_driver[wdma]->stop(wdma, cmdq_handle);

		if (ddp_modules_driver[wdma]->deinit != 0)
			ddp_modules_driver[wdma]->deinit(wdma, cmdq_handle);

	}
	if (handle->scenario == DDP_SCENARIO_PRIMARY_ALL) {
		dis_scn = DDP_SCENARIO_PRIMARY_OVL_MEMOUT;
		new_scn = DDP_SCENARIO_PRIMARY_DISP;
	} else if (handle->scenario == DDP_SCENARIO_DITHER_1TO2) {
		dis_scn = DDP_SCENARIO_PRIMARY_DITHER_MEMOUT;
		new_scn = DDP_SCENARIO_PRIMARY_DISP;
	} else if (handle->scenario == DDP_SCENARIO_UFOE_1TO2) {
		dis_scn = DDP_SCENARIO_PRIMARY_UFOE_MEMOUT;
		new_scn = DDP_SCENARIO_PRIMARY_DISP;

	} else if (handle->scenario == DDP_SCENARIO_SUB_ALL) {
		dis_scn = DDP_SCENARIO_SUB_OVL_MEMOUT;
		new_scn = DDP_SCENARIO_SUB_DISP;
	} else {
		pr_err("%s: error scenario =%d\n", __func__, handle->scenario);
		BUG();
	}
	_dpmgr_path_disconnect(dis_scn, cmdq_handle);
	handle->scenario = new_scn;
	/* update connected */
	_dpmgr_path_connect(handle->scenario, cmdq_handle);
	ddp_mutex_set(handle->hwmutexid, handle->scenario, handle->mode, cmdq_handle);

	return 0;
}

int dpmgr_insert_ovl1_sub(disp_path_handle dp_handle, void *cmdq_handle)
{
	int ret = 0;
	ddp_path_handle handle;
	DDP_MANAGER_CONTEXT *context;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle)dp_handle;
	if ((handle->scenario != DDP_SCENARIO_SUB_RDMA1_DISP) && (handle->scenario != DDP_SCENARIO_SUB_DISP)) {
		DISPERR("dpmgr_insert_ovl1_sub error, handle->scenario=%s\n",
			ddp_get_scenario_name(handle->scenario));
		return -1;
	}

	if (ddp_is_module_in_scenario(handle->scenario, DISP_MODULE_OVL1) == 1) {
		DISPMSG("dpmgr_insert_ovl1_sub, OVL1 already in the path\n");
		return -1;
	}

	DISPDBG("dpmgr_insert_ovl1_sub\n");
	/* update contxt*/
	context = _get_context();
	context->module_usage_table[DISP_MODULE_OVL1]++;
	context->module_path_table[DISP_MODULE_OVL1] = handle;

	/* update connected*/
	ddp_connect_path(DDP_SCENARIO_SUB_DISP, cmdq_handle);
	ret = ddp_mutex_set(handle->hwmutexid, DDP_SCENARIO_SUB_DISP, handle->mode, cmdq_handle);

	handle->scenario = DDP_SCENARIO_SUB_DISP;

	/*ovl1 just need start.*/
	if (ddp_modules_driver[DISP_MODULE_OVL1] != 0) {
		if (ddp_modules_driver[DISP_MODULE_OVL1]->init != 0)
			ddp_modules_driver[DISP_MODULE_OVL1]->init(DISP_MODULE_OVL1, cmdq_handle);

		if (ddp_modules_driver[DISP_MODULE_OVL1]->start != 0)
			ddp_modules_driver[DISP_MODULE_OVL1]->start(DISP_MODULE_OVL1, cmdq_handle);
	}

	return ret;
}

int dpmgr_remove_ovl1_sub(disp_path_handle dp_handle, void *cmdq_handle)
{
	int ret = 0;
	ddp_path_handle handle;
	DDP_MANAGER_CONTEXT *context;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle)dp_handle;
	if ((handle->scenario != DDP_SCENARIO_SUB_RDMA1_DISP) && (handle->scenario != DDP_SCENARIO_SUB_DISP)) {
		DISPERR("dpmgr_remove_ovl1_sub error, handle->scenario=%s\n", ddp_get_scenario_name(handle->scenario));
		return -1;
	}

	if (ddp_is_module_in_scenario(handle->scenario, DISP_MODULE_OVL1) == 0) {
		DISPMSG("dpmgr_remove_ovl1_sub, OVL1 already not in the path\n");
		return -1;
	}

	DISPDBG("dpmgr_remove_ovl1_sub\n");
	/* update contxt */
	context = _get_context();
	context->module_usage_table[DISP_MODULE_OVL1]--;
	context->module_path_table[DISP_MODULE_OVL1] = 0;

	ddp_disconnect_path(handle->scenario, cmdq_handle);
	ddp_connect_path(DDP_SCENARIO_SUB_RDMA1_DISP, cmdq_handle);
	ret = ddp_mutex_set(handle->hwmutexid, DDP_SCENARIO_SUB_RDMA1_DISP, handle->mode, cmdq_handle);

	handle->scenario = DDP_SCENARIO_SUB_RDMA1_DISP;

	/*ovl1 just need stop */
	if (ddp_modules_driver[DISP_MODULE_OVL1] != 0) {
		if (ddp_modules_driver[DISP_MODULE_OVL1]->stop != 0)
			ddp_modules_driver[DISP_MODULE_OVL1]->stop(DISP_MODULE_OVL1, cmdq_handle);

		if (ddp_modules_driver[DISP_MODULE_OVL1]->deinit != 0)
			ddp_modules_driver[DISP_MODULE_OVL1]->deinit(DISP_MODULE_OVL1, cmdq_handle);
	}

	return ret;
}

int dpmgr_path_set_dst_module(disp_path_handle dp_handle, DISP_MODULE_ENUM dst_module)
{
	ddp_path_handle handle;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	ASSERT((handle->scenario >= 0 && handle->scenario < DDP_SCENARIO_MAX));
	DISPDBG("set dst module on scenario %s, module %s\n",
		   ddp_get_scenario_name(handle->scenario), ddp_get_module_name(dst_module));
	return ddp_set_dst_module(handle->scenario, dst_module);
}

int dpmgr_path_get_mutex(disp_path_handle dp_handle)
{
	ddp_path_handle handle = NULL;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	return handle->hwmutexid;
}

DISP_MODULE_ENUM dpmgr_path_get_dst_module(disp_path_handle dp_handle)
{
	ddp_path_handle handle;
	DISP_MODULE_ENUM dst_module;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	ASSERT((handle->scenario >= 0 && handle->scenario < DDP_SCENARIO_MAX));
	dst_module = ddp_get_dst_module(handle->scenario);

	DISPDBG("get dst module on scenario %s, module %s\n",
		   ddp_get_scenario_name(handle->scenario), ddp_get_module_name(dst_module));
	return dst_module;
}

enum dst_module_type dpmgr_path_get_dst_module_type(disp_path_handle dp_handle)
{
	DISP_MODULE_ENUM dst_module;

	dst_module = dpmgr_path_get_dst_module(dp_handle);
	if (dst_module == DISP_MODULE_WDMA0 || dst_module == DISP_MODULE_WDMA1)
		return DST_MOD_WDMA;
	else
		return DST_MOD_REAL_TIME;
}


int dpmgr_path_connect(disp_path_handle dp_handle, int encmdq)
{
	ddp_path_handle handle;
	cmdqRecHandle cmdqHandle;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	cmdqHandle = encmdq ? handle->cmdqhandle : NULL;

	_dpmgr_path_connect(handle->scenario, cmdqHandle);

	ddp_mutex_set(handle->hwmutexid, handle->scenario, handle->mode, cmdqHandle);
	ddp_mutex_Interrupt_enable(handle->hwmutexid, cmdqHandle);
	return 0;
}

int dpmgr_path_disconnect(disp_path_handle dp_handle, int encmdq)
{
	ddp_path_handle handle;
	cmdqRecHandle cmdqHandle;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	cmdqHandle = encmdq ? handle->cmdqhandle : NULL;

	DISPDBG("dpmgr_path_disconnect on scenario %s\n",
		   ddp_get_scenario_name(handle->scenario));
	ddp_mutex_clear(handle->hwmutexid, cmdqHandle);
	ddp_mutex_Interrupt_disable(handle->hwmutexid, cmdqHandle);
	_dpmgr_path_disconnect(handle->scenario, cmdqHandle);
	return 0;
}

int dpmgr_path_init(disp_path_handle dp_handle, int encmdq)
{
	int i = 0;
	int module_name;
	ddp_path_handle handle;
	int *modules;
	int module_num;
	cmdqRecHandle cmdqHandle;
	DDP_MANAGER_CONTEXT *context = _get_context();

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	modules = ddp_get_scenario_list(handle->scenario);
	module_num = ddp_get_module_num(handle->scenario);
	cmdqHandle = encmdq ? handle->cmdqhandle : NULL;

	DISPDBG("path init on scenario %s\n", ddp_get_scenario_name(handle->scenario));
	mutex_lock(&context->mutex_lock);
	/* open top clock */
	path_top_clock_on();
	/* seting mutex */
	ddp_mutex_set(handle->hwmutexid, handle->scenario, handle->mode, cmdqHandle);
	ddp_mutex_Interrupt_enable(handle->hwmutexid, cmdqHandle);
	/* connect path; */
	_dpmgr_path_connect(handle->scenario, cmdqHandle);

	/* each module init */
	for (i = 0; i < module_num; i++) {
		module_name = modules[i];
		if (ddp_modules_driver[module_name] != 0) {
			if (ddp_modules_driver[module_name]->init != 0) {
				DISPDBG("scenario %s init module  %s\n",
					   ddp_get_scenario_name(handle->scenario),
					   ddp_get_module_name(module_name));
				ddp_modules_driver[module_name]->init(module_name, cmdqHandle);
			}
			if (ddp_modules_driver[module_name]->set_listener != 0)
				ddp_modules_driver[module_name]->set_listener(module_name,
									      dpmgr_module_notify);

		}
	}
	/* after init this path will power on; */
	handle->power_sate = 1;
	mutex_unlock(&context->mutex_lock);
	return 0;
}

int dpmgr_path_deinit(disp_path_handle dp_handle, int encmdq)
{
	int i = 0;
	int module_name;
	int *modules;
	int module_num;
	cmdqRecHandle cmdqHandle;
	ddp_path_handle handle;
	DDP_MANAGER_CONTEXT *context = _get_context();

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	modules = ddp_get_scenario_list(handle->scenario);
	module_num = ddp_get_module_num(handle->scenario);
	cmdqHandle = encmdq ? handle->cmdqhandle : NULL;

	DISPDBG("path deinit on scenario %s\n", ddp_get_scenario_name(handle->scenario));
	mutex_lock(&context->mutex_lock);
	ddp_mutex_Interrupt_disable(handle->hwmutexid, cmdqHandle);
	ddp_mutex_clear(handle->hwmutexid, cmdqHandle);
	_dpmgr_path_disconnect(handle->scenario, cmdqHandle);
	for (i = 0; i < module_num; i++) {
		module_name = modules[i];
		if (ddp_modules_driver[module_name] != 0) {
			if (ddp_modules_driver[module_name]->deinit != 0) {
				DISPDBG("scenario %s deinit module  %s\n",
					   ddp_get_scenario_name(handle->scenario),
					   ddp_get_module_name(module_name));
				ddp_modules_driver[module_name]->deinit(module_name, cmdqHandle);
			}
			if (ddp_modules_driver[module_name]->set_listener != 0)
				ddp_modules_driver[module_name]->set_listener(module_name, NULL);

		}
	}
	handle->power_sate = 0;
	/* close top clock when last path init */
	path_top_clock_off();
	mutex_unlock(&context->mutex_lock);
	return 0;
}

int dpmgr_path_start(disp_path_handle dp_handle, int encmdq)
{
	int i = 0;
	int module_name;
	ddp_path_handle handle;
	int *modules;
	int module_num;
	cmdqRecHandle cmdqHandle;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	modules = ddp_get_scenario_list(handle->scenario);
	module_num = ddp_get_module_num(handle->scenario);
	cmdqHandle = encmdq ? handle->cmdqhandle : NULL;

	DISPMSG("path start on scenario %s\n", ddp_get_scenario_name(handle->scenario));
	for (i = 0; i < module_num; i++) {
		module_name = modules[i];
		if (ddp_modules_driver[module_name] != 0) {
			if (ddp_modules_driver[module_name]->start != 0) {
				DISPDBG("scenario %s start module  %s\n",
					   ddp_get_scenario_name(handle->scenario),
					   ddp_get_module_name(module_name));
				ddp_modules_driver[module_name]->start(module_name, cmdqHandle);
			}
		}
	}
	return 0;
}

int dpmgr_path_stop(disp_path_handle dp_handle, int encmdq)
{

	int i = 0;
	int module_name;
	ddp_path_handle handle;
	int *modules;
	int module_num;
	cmdqRecHandle cmdqHandle;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	modules = ddp_get_scenario_list(handle->scenario);
	module_num = ddp_get_module_num(handle->scenario);
	cmdqHandle = encmdq ? handle->cmdqhandle : NULL;

	DISPMSG("path stop on scenario %s\n", ddp_get_scenario_name(handle->scenario));
	for (i = module_num - 1; i >= 0; i--) {
		module_name = modules[i];
		if (ddp_modules_driver[module_name] != 0) {
			if (ddp_modules_driver[module_name]->stop != 0) {
				DISPDBG("scenario %s  stop module %s\n",
					   ddp_get_scenario_name(handle->scenario),
					   ddp_get_module_name(module_name));
				ddp_modules_driver[module_name]->stop(module_name, cmdqHandle);
			}
		}
	}
	return 0;
}

int dpmgr_path_ioctl(disp_path_handle dp_handle, void *cmdq_handle, DDP_IOCTL_NAME ioctl_cmd,
		     void *params)
{
	int i = 0;
	int ret = 0;
	int module_name;
	int *modules;
	int module_num;
	ddp_path_handle handle;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	modules = ddp_get_scenario_list(handle->scenario);
	module_num = ddp_get_module_num(handle->scenario);

	DISPDBG("path IOCTL on scenario %s\n", ddp_get_scenario_name(handle->scenario));
	for (i = module_num - 1; i >= 0; i--) {
		module_name = modules[i];
		if (ddp_modules_driver[module_name] != 0) {
			if (ddp_modules_driver[module_name]->ioctl != 0) {
				ret +=
				    ddp_modules_driver[module_name]->ioctl(module_name, cmdq_handle,
									   ioctl_cmd, params);
			}
		}
	}
	return ret;
}

int dpmgr_path_enable_irq(disp_path_handle dp_handle, void *cmdq_handle, DDP_IRQ_LEVEL irq_level)
{
	int i = 0;
	int ret = 0;
	int module_name;
	ddp_path_handle handle;
	int *modules;
	int module_num;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	modules = ddp_get_scenario_list(handle->scenario);
	module_num = ddp_get_module_num(handle->scenario);

	DISPDBG("path enable irq on scenario %s, level %d\n",
		   ddp_get_scenario_name(handle->scenario), irq_level);

	if (irq_level != DDP_IRQ_LEVEL_ALL)
		ddp_mutex_Interrupt_disable(handle->hwmutexid, cmdq_handle);
	else
		ddp_mutex_Interrupt_enable(handle->hwmutexid, cmdq_handle);


	for (i = module_num - 1; i >= 0; i--) {
		module_name = modules[i];
		if (ddp_modules_driver[module_name] && ddp_modules_driver[module_name]->enable_irq) {
			DISPDBG("scenario %s, module %s enable irq level %d\n",
			       ddp_get_scenario_name(handle->scenario),
			       ddp_get_module_name(module_name), irq_level);
			ret +=
			    ddp_modules_driver[module_name]->enable_irq(module_name, cmdq_handle,
									irq_level);
		}
	}
	return ret;
}

int dpmgr_path_reset(disp_path_handle dp_handle, int encmdq)
{
	int i = 0;
	int ret = 0;
	int error = 0;
	int module_name;
	int *modules;
	int module_num;
	ddp_path_handle handle;
	cmdqRecHandle cmdqHandle;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	modules = ddp_get_scenario_list(handle->scenario);
	module_num = ddp_get_module_num(handle->scenario);
	cmdqHandle = encmdq ? handle->cmdqhandle : NULL;

	DISPMSG("path reset on scenario %s\n", ddp_get_scenario_name(handle->scenario));
	/* first reset mutex */
	ddp_mutex_reset(handle->hwmutexid, cmdqHandle);

	for (i = 0; i < module_num; i++) {
		module_name = modules[i];
		if (ddp_modules_driver[module_name] != 0) {
			if (ddp_modules_driver[module_name]->reset != 0) {
				DISPDBG("scenario %s  reset module %s\n",
					   ddp_get_scenario_name(handle->scenario),
					   ddp_get_module_name(module_name));
				ret =
				    ddp_modules_driver[module_name]->reset(module_name, cmdqHandle);
				if (ret != 0)
					error++;

			}
		}
	}
	return error > 0 ? -1 : 0;
}

static unsigned int dpmgr_is_PQ(DISP_MODULE_ENUM module)
{
	unsigned int isPQ = 0;

	switch (module) {
	case DISP_MODULE_COLOR0:
	case DISP_MODULE_CCORR:
	case DISP_MODULE_AAL:
	case DISP_MODULE_GAMMA:
	case DISP_MODULE_DITHER:
		/* case DISP_MODULE_PWM0  : */
		isPQ = 1;
		break;
	default:
		isPQ = 0;
	}

	return isPQ;
}

int dpmgr_path_config(disp_path_handle dp_handle, disp_ddp_path_config *config, void *cmdq_handle)
{
	int i = 0;
	int module_name;
	ddp_path_handle handle;
	int *modules;
	int module_num;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	modules = ddp_get_scenario_list(handle->scenario);
	module_num = ddp_get_module_num(handle->scenario);

	DISPDBG("path config ovl %d, rdma %d, wdma %d, dst %d on handle %p scenario %s\n",
		   config->ovl_dirty,
		   config->rdma_dirty,
		   config->wdma_dirty,
		   config->dst_dirty, handle, ddp_get_scenario_name(handle->scenario));
	memcpy(&handle->last_config, config, sizeof(disp_ddp_path_config));
	for (i = 0; i < module_num; i++) {
		module_name = modules[i];
		if (ddp_modules_driver[module_name] != 0) {
			if (ddp_modules_driver[module_name]->config != 0) {
				DISPDBG("scenario %s  config module %s\n",
					   ddp_get_scenario_name(handle->scenario),
					   ddp_get_module_name(module_name));
				ddp_modules_driver[module_name]->config(module_name, config,
									cmdq_handle);
			}

			if (disp_helper_get_option(DISP_OPT_BYPASS_PQ)
			    && dpmgr_is_PQ(module_name) == 1) {
				if (ddp_modules_driver[module_name]->bypass != NULL)
					ddp_modules_driver[module_name]->bypass(module_name, 1);
			}

		}
	}
	return 0;
}

disp_ddp_path_config *dpmgr_path_get_last_config_notclear(disp_path_handle dp_handle)
{
	ddp_path_handle handle = (ddp_path_handle) dp_handle;

	ASSERT(dp_handle != NULL);
	return &handle->last_config;
}

disp_ddp_path_config *dpmgr_path_get_last_config(disp_path_handle dp_handle)
{
	ddp_path_handle handle;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	handle->last_config.ovl_dirty = 0;
	handle->last_config.rdma_dirty = 0;
	handle->last_config.wdma_dirty = 0;
	handle->last_config.dst_dirty = 0;
	handle->last_config.ovl_layer_dirty = 0;
	handle->last_config.ovl_layer_scanned = 0;
	return &handle->last_config;
}

void dpmgr_get_input_address(disp_path_handle dp_handle, unsigned long *addr)
{
	ddp_path_handle handle;
	int *modules;

	handle = (ddp_path_handle) dp_handle;
	modules = ddp_get_scenario_list(handle->scenario);

	if (modules[0] == DISP_MODULE_OVL0 || modules[0] == DISP_MODULE_OVL1)
		ovl_get_address(modules[0], addr);
	else if (modules[0] == DISP_MODULE_RDMA0 || modules[0] == DISP_MODULE_RDMA1
		 || modules[0] == DISP_MODULE_RDMA2)
		rdma_get_address(modules[0], addr);
}

int dpmgr_path_build_cmdq(disp_path_handle dp_handle, void *trigger_loop_handle, CMDQ_STATE state,
			  int reverse)
{
	int ret = 0;
	int i = 0;
	int module_name;
	ddp_path_handle handle;
	int *modules;
	int module_num;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	modules = ddp_get_scenario_list(handle->scenario);
	module_num = ddp_get_module_num(handle->scenario);

	if (reverse) {
		for (i = module_num - 1; i >= 0; i--) {
			module_name = modules[i];
			if (ddp_modules_driver[module_name]
			    && ddp_modules_driver[module_name]->build_cmdq) {
				ret =
				    ddp_modules_driver[module_name]->build_cmdq(module_name,
										trigger_loop_handle,
										state);
			}
		}
	} else {
		for (i = 0; i < module_num; i++) {
			module_name = modules[i];
			if (ddp_modules_driver[module_name]
			    && ddp_modules_driver[module_name]->build_cmdq) {
				ret =
				    ddp_modules_driver[module_name]->build_cmdq(module_name,
										trigger_loop_handle,
										state);
			}
		}
	}
	return ret;
}

int dpmgr_path_trigger(disp_path_handle dp_handle, void *trigger_loop_handle, int encmdq)
{
	ddp_path_handle handle;
	int *modules;
	int module_num;
	int i;
	int module_name;
	cmdqRecHandle cmdqHandle;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	DISPDBG("dpmgr_path_trigger on scenario %s\n", ddp_get_scenario_name(handle->scenario));
	cmdqHandle = encmdq ? handle->cmdqhandle : NULL;
	modules = ddp_get_scenario_list(handle->scenario);
	module_num = ddp_get_module_num(handle->scenario);

	ddp_mutex_enable(handle->hwmutexid, handle->scenario, trigger_loop_handle);
	for (i = 0; i < module_num; i++) {
		module_name = modules[i];
		if (ddp_modules_driver[module_name] != 0) {
			if (ddp_modules_driver[module_name]->trigger != 0) {
				DISPDBG("%s trigger\n", ddp_get_module_name(module_name));
				ddp_modules_driver[module_name]->trigger(module_name,
									 trigger_loop_handle);
			}
		}
	}
	return 0;
}

int dpmgr_path_flush(disp_path_handle dp_handle, int encmdq)
{
	ddp_path_handle handle;
	cmdqRecHandle cmdqHandle;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	cmdqHandle = encmdq ? handle->cmdqhandle : NULL;
	DISPDBG("path flush on scenario %s\n", ddp_get_scenario_name(handle->scenario));
	return ddp_mutex_enable(handle->hwmutexid, handle->scenario, cmdqHandle);
}

int dpmgr_path_power_off(disp_path_handle dp_handle, CMDQ_SWITCH encmdq)
{
	int i = 0;
	int module_name;
	ddp_path_handle handle;
	int *modules;
	int module_num;
	DDP_MANAGER_CONTEXT *context = _get_context();

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	modules = ddp_get_scenario_list(handle->scenario);
	module_num = ddp_get_module_num(handle->scenario);

	DISPMSG("path power off on scenario %s\n", ddp_get_scenario_name(handle->scenario));
	mutex_lock(&context->mutex_lock);
	for (i = 0; i < module_num; i++) {
		module_name = modules[i];
		if (ddp_modules_driver[module_name] && ddp_modules_driver[module_name]->power_off) {
			DISPDBG(" %s power off\n", ddp_get_module_name(module_name));
			ddp_modules_driver[module_name]->power_off(module_name,
								   encmdq ? handle->cmdqhandle :
								   NULL);
		}
	}
	handle->power_sate = 0;
	path_top_clock_off();
	mutex_unlock(&context->mutex_lock);
	return 0;
}

int dpmgr_path_power_on(disp_path_handle dp_handle, CMDQ_SWITCH encmdq)
{
	int i = 0;
	int module_name;
	int *modules;
	int module_num;
	ddp_path_handle handle;
	DDP_MANAGER_CONTEXT *context = _get_context();

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	modules = ddp_get_scenario_list(handle->scenario);
	module_num = ddp_get_module_num(handle->scenario);

	DISPMSG("path power on scenario %s\n", ddp_get_scenario_name(handle->scenario));
	mutex_lock(&context->mutex_lock);
	path_top_clock_on();
	for (i = 0; i < module_num; i++) {
		module_name = modules[i];
		if (ddp_modules_driver[module_name] && ddp_modules_driver[module_name]->power_on) {
			DISPDBG("%s power on\n", ddp_get_module_name(module_name));
			ddp_modules_driver[module_name]->power_on(module_name,
								  encmdq ? handle->cmdqhandle :
								  NULL);
		}
	}
	/* modules on this path will resume power on; */
	handle->power_sate = 1;
	mutex_unlock(&context->mutex_lock);
	return 0;
}

int dpmgr_path_power_off_bypass_pwm(disp_path_handle dp_handle, CMDQ_SWITCH encmdq)
{
	int i = 0;
	int module_name;
	ddp_path_handle handle;
	int *modules;
	int module_num;
	DDP_MANAGER_CONTEXT *context = _get_context();

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	modules = ddp_get_scenario_list(handle->scenario);
	module_num = ddp_get_module_num(handle->scenario);

	DISPMSG("path power off on scenario %s\n", ddp_get_scenario_name(handle->scenario));
	mutex_lock(&context->mutex_lock);
	for (i = 0; i < module_num; i++) {
		module_name = modules[i];
		if (ddp_modules_driver[module_name] && ddp_modules_driver[module_name]->power_off) {
			if (module_name == DISP_MODULE_PWM0)
				DISPMSG(" %s power off -- bypass\n", ddp_get_module_name(module_name));
			else {
				DISPDBG(" %s power off\n", ddp_get_module_name(module_name));
				ddp_modules_driver[module_name]->power_off(module_name,
									   encmdq ? handle->cmdqhandle :
									   NULL);
			}
		}
	}
	handle->power_sate = 0;
	path_top_clock_off();
	mutex_unlock(&context->mutex_lock);
	return 0;
}

int dpmgr_path_power_on_bypass_pwm(disp_path_handle dp_handle, CMDQ_SWITCH encmdq)
{
	int i = 0;
	int module_name;
	int *modules;
	int module_num;
	ddp_path_handle handle;
	DDP_MANAGER_CONTEXT *context = _get_context();

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	modules = ddp_get_scenario_list(handle->scenario);
	module_num = ddp_get_module_num(handle->scenario);

	DISPMSG("path power on scenario %s\n", ddp_get_scenario_name(handle->scenario));
	mutex_lock(&context->mutex_lock);
	path_top_clock_on();
	for (i = 0; i < module_num; i++) {
		module_name = modules[i];
		if (ddp_modules_driver[module_name] && ddp_modules_driver[module_name]->power_on) {
			if (module_name == DISP_MODULE_PWM0)
				DISPMSG(" %s power on -- bypass\n", ddp_get_module_name(module_name));
			else {
				DISPDBG("%s power on\n", ddp_get_module_name(module_name));
				ddp_modules_driver[module_name]->power_on(module_name,
									  encmdq ? handle->cmdqhandle :
									  NULL);
			}
		}
	}
	/* modules on this path will resume power on; */
	handle->power_sate = 1;
	mutex_unlock(&context->mutex_lock);
	return 0;
}
static int is_module_in_path(DISP_MODULE_ENUM module, ddp_path_handle handle)
{
	DDP_MANAGER_CONTEXT *context = _get_context();

	ASSERT(module < DISP_MODULE_UNKNOWN);
	if (context->module_path_table[module] == handle)
		return 1;

	return 0;
}

int dpmgr_path_user_cmd(disp_path_handle dp_handle, int msg, unsigned long arg, void *cmdqhandle)
{
	int ret = -1;
	DISP_MODULE_ENUM dst = DISP_MODULE_UNKNOWN;
	ddp_path_handle handle = NULL;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	/* DISPDBG("dpmgr_path_user_cmd msg 0x%08x\n",msg); */
	switch (msg) {
	case DISP_IOCTL_AAL_EVENTCTL:
	case DISP_IOCTL_AAL_GET_HIST:
	case DISP_IOCTL_AAL_INIT_REG:
	case DISP_IOCTL_AAL_SET_PARAM:
		/* TODO: just for verify rootcause, will be removed soon */
#ifndef CONFIG_FOR_SOURCE_PQ
		if (is_module_in_path(DISP_MODULE_AAL, handle)) {
			if (ddp_modules_driver[DISP_MODULE_AAL]->cmd != NULL)
				ret =
				    ddp_modules_driver[DISP_MODULE_AAL]->cmd(DISP_MODULE_AAL, msg,
									     arg, cmdqhandle);

		}
#endif
		break;
	case DISP_IOCTL_SET_GAMMALUT:
		if (ddp_modules_driver[DISP_MODULE_GAMMA]->cmd != NULL)
			ret =
			    ddp_modules_driver[DISP_MODULE_GAMMA]->cmd(DISP_MODULE_GAMMA, msg, arg,
								       cmdqhandle);
		break;
	case DISP_IOCTL_SET_CCORR:
	case DISP_IOCTL_CCORR_EVENTCTL:
	case DISP_IOCTL_CCORR_GET_IRQ:
		if (ddp_modules_driver[DISP_MODULE_CCORR]->cmd != NULL)
			ret =
			    ddp_modules_driver[DISP_MODULE_CCORR]->cmd(DISP_MODULE_CCORR, msg, arg,
								       cmdqhandle);
		break;

	case DISP_IOCTL_SET_PQPARAM:
	case DISP_IOCTL_GET_PQPARAM:
	case DISP_IOCTL_SET_PQINDEX:
	case DISP_IOCTL_GET_PQINDEX:
	case DISP_IOCTL_SET_COLOR_REG:
	case DISP_IOCTL_SET_TDSHPINDEX:
	case DISP_IOCTL_GET_TDSHPINDEX:
	case DISP_IOCTL_SET_PQ_CAM_PARAM:
	case DISP_IOCTL_GET_PQ_CAM_PARAM:
	case DISP_IOCTL_SET_PQ_GAL_PARAM:
	case DISP_IOCTL_GET_PQ_GAL_PARAM:
	case DISP_IOCTL_PQ_SET_BYPASS_COLOR:
	case DISP_IOCTL_PQ_SET_WINDOW:
	case DISP_IOCTL_WRITE_REG:
	case DISP_IOCTL_READ_REG:
	case DISP_IOCTL_MUTEX_CONTROL:
	case DISP_IOCTL_PQ_GET_TDSHP_FLAG:
	case DISP_IOCTL_PQ_SET_TDSHP_FLAG:
	case DISP_IOCTL_PQ_GET_DC_PARAM:
	case DISP_IOCTL_PQ_SET_DC_PARAM:
	case DISP_IOCTL_PQ_GET_DS_PARAM:
	case DISP_IOCTL_PQ_GET_MDP_COLOR_CAP:
	case DISP_IOCTL_PQ_GET_MDP_TDSHP_REG:
	case DISP_IOCTL_WRITE_SW_REG:
	case DISP_IOCTL_READ_SW_REG:
		if (is_module_in_path(DISP_MODULE_COLOR0, handle))
			dst = DISP_MODULE_COLOR0;
		else if (is_module_in_path(DISP_MODULE_COLOR1, handle))
			dst = DISP_MODULE_COLOR1;
		else
			DISPDBG("dpmgr_path_user_cmd color is not on this path\n");

		if (dst != DISP_MODULE_UNKNOWN) {

			if (ddp_modules_driver[dst]->cmd != NULL)
				ret = ddp_modules_driver[dst]->cmd(dst, msg, arg, cmdqhandle);

		}
		break;
	default:
		DISPDBG("dpmgr_path_user_cmd io not supported\n");
		break;
	}
	return ret;
}

int dpmgr_path_set_parameter(disp_path_handle dp_handle, int io_evnet, void *data)
{
	return 0;
}

int dpmgr_path_get_parameter(disp_path_handle dp_handle, int io_evnet, void *data)
{
	return 0;
}

int dpmgr_path_is_idle(disp_path_handle dp_handle)
{
	ASSERT(dp_handle != NULL);
	return !dpmgr_path_is_busy(dp_handle);
}

int dpmgr_path_is_busy(disp_path_handle dp_handle)
{
	int i = 0;
	int module_name;
	ddp_path_handle handle;
	int *modules;
	int module_num;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	modules = ddp_get_scenario_list(handle->scenario);
	module_num = ddp_get_module_num(handle->scenario);

	DISPDBG("path check busy on scenario %s\n", ddp_get_scenario_name(handle->scenario));
	for (i = module_num - 1; i >= 0; i--) {
		module_name = modules[i];
		if (ddp_modules_driver[module_name] != 0) {
			if (ddp_modules_driver[module_name]->is_busy != 0) {
				if (ddp_modules_driver[module_name]->is_busy(module_name)) {
					DISPDBG("%s is busy\n",
						   ddp_get_module_name(module_name));
					return 1;
				}
			}
		}
	}
	return 0;
}

int dpmgr_set_lcm_utils(disp_path_handle dp_handle, void *lcm_drv)
{
	int i = 0;
	int module_name;
	int *modules;
	int module_num;
	ddp_path_handle handle;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	modules = ddp_get_scenario_list(handle->scenario);
	module_num = ddp_get_module_num(handle->scenario);

	DISPDBG("path set lcm drv handle %p\n", handle);
	for (i = 0; i < module_num; i++) {
		module_name = modules[i];
		if (ddp_modules_driver[module_name] != 0) {
			if ((ddp_modules_driver[module_name]->set_lcm_utils != 0) && lcm_drv) {
				DISPDBG("%s set lcm utils\n", ddp_get_module_name(module_name));
				ddp_modules_driver[module_name]->set_lcm_utils(module_name,
									       lcm_drv);
			}
		}
	}
	return 0;
}

int dpmgr_enable_event(disp_path_handle dp_handle, DISP_PATH_EVENT event)
{
	ddp_path_handle handle;
	DPMGR_WQ_HANDLE *wq_handle;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	wq_handle = &handle->wq_list[event];

	DISPDBG("enable event %s on scenario %s, irtbit 0x%x\n",
		   path_event_name(event),
		   ddp_get_scenario_name(handle->scenario), handle->irq_event_map[event].irq_bit);
	if (!wq_handle->init) {
		init_waitqueue_head(&(wq_handle->wq));
		wq_handle->init = 1;
		wq_handle->data = 0;
		wq_handle->event = event;
	}
	return 0;
}

int dpmgr_map_event_to_irq(disp_path_handle dp_handle, DISP_PATH_EVENT event, DDP_IRQ_BIT irq_bit)
{
	ddp_path_handle handle;
	DDP_IRQ_EVENT_MAPPING *irq_table;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	irq_table = handle->irq_event_map;

	if (event < DISP_PATH_EVENT_NUM) {
		DISPDBG("map event %s to irq 0x%x on scenario %s\n", path_event_name(event),
			   irq_bit, ddp_get_scenario_name(handle->scenario));
		irq_table[event].irq_bit = irq_bit;
		return 0;
	}
	DISPERR("fail to map event %s to irq 0x%x on scenario %s\n", path_event_name(event),
		   irq_bit, ddp_get_scenario_name(handle->scenario));
	return -1;
}

int dpmgr_disable_event(disp_path_handle dp_handle, DISP_PATH_EVENT event)
{
	ddp_path_handle handle;
	DPMGR_WQ_HANDLE *wq_handle;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;

	DISPDBG("disable event %s on scenario %s\n", path_event_name(event),
		   ddp_get_scenario_name(handle->scenario));
	wq_handle = &handle->wq_list[event];

	wq_handle->init = 0;
	wq_handle->data = 0;
	return 0;
}

int dpmgr_check_status(disp_path_handle dp_handle)
{
	int i = 0;
	int *modules;
	int module_num;
	ddp_path_handle handle;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	modules = ddp_get_scenario_list(handle->scenario);
	module_num = ddp_get_module_num(handle->scenario);

	DISPDMP("check status on scenario %s\n", ddp_get_scenario_name(handle->scenario));
	ddp_dump_analysis(DISP_MODULE_CONFIG);
	ddp_check_path(handle->scenario);
	ddp_check_mutex(handle->hwmutexid, handle->scenario, handle->mode);

	/* dump path */
	{
		DISPDMP("path:");
		for (i = 0; i < module_num; i++)
			DISPDMP("%s-", ddp_get_module_name(modules[i]));

		DISPDMP("\n");
	}
	ddp_dump_analysis(DISP_MODULE_MUTEX);

	for (i = 0; i < module_num; i++)
		ddp_dump_analysis(modules[i]);

	for (i = 0; i < module_num; i++)
		ddp_dump_reg(modules[i]);


	ddp_dump_reg(DISP_MODULE_CONFIG);
	ddp_dump_reg(DISP_MODULE_MUTEX);

	return 0;
}

void dpmgr_debug_path_status(int mutex_id)
{
	int i = 0;
	DDP_MANAGER_CONTEXT *content = _get_context();
	disp_path_handle handle = NULL;

	if (mutex_id >= DISP_MUTEX_DDP_FIRST
	    && mutex_id < (DISP_MUTEX_DDP_FIRST + DISP_MUTEX_DDP_COUNT)) {
		handle = (disp_path_handle) content->handle_pool[mutex_id];
		if (handle)
			dpmgr_check_status(handle);

	} else {
		for (i = DISP_MUTEX_DDP_FIRST; i < (DISP_MUTEX_DDP_FIRST + DISP_MUTEX_DDP_COUNT);
		     i++) {
			handle = (disp_path_handle) content->handle_pool[i];
			if (handle)
				dpmgr_check_status(handle);

		}
	}
}


int dpmgr_wait_event_timeout(disp_path_handle dp_handle, DISP_PATH_EVENT event, int timeout)
{
	int ret = -1;
	ddp_path_handle handle;
	DPMGR_WQ_HANDLE *wq_handle;
	unsigned long long cur_time;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	wq_handle = &handle->wq_list[event];

	if (wq_handle->init) {
		DISPDBG("wait event %s on scenario %s\n", path_event_name(event),
			   ddp_get_scenario_name(handle->scenario));
		cur_time = ktime_to_ns(ktime_get());/*sched_clock();*/

		ret =
		    wait_event_interruptible_timeout(wq_handle->wq, cur_time < wq_handle->data,
						     timeout);
		if (ret == 0) {
			DISPERR("wait %s timeout on scenario %s\n", path_event_name(event),
				   ddp_get_scenario_name(handle->scenario));
			/* dpmgr_check_status(dp_handle); */
		} else if (ret < 0) {
			DISPERR("wait %s interrupt by other timeleft %d on scenario %s\n",
				   path_event_name(event), ret,
				   ddp_get_scenario_name(handle->scenario));
		} else {
			DISPDBG("received event %s timeleft %d on scenario %s\n",
				   path_event_name(event), ret,
				   ddp_get_scenario_name(handle->scenario));
		}
		return ret;
	}
	DISPERR("wait event %s not initialized on scenario %s\n", path_event_name(event),
		   ddp_get_scenario_name(handle->scenario));
	return ret;
}

int _dpmgr_wait_event(disp_path_handle dp_handle, DISP_PATH_EVENT event, unsigned long long *event_ts)
{
	int ret = -1;
	ddp_path_handle handle;
	DPMGR_WQ_HANDLE *wq_handle;
	unsigned long long cur_time;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	wq_handle = &handle->wq_list[event];

	if (!wq_handle->init) {
		DISPERR("wait event %s not initialized on scenario %s\n", path_event_name(event),
			ddp_get_scenario_name(handle->scenario));
		return -2;
	}

	DISPDBG("wait event %s on scenario %s\n", path_event_name(event),
		   ddp_get_scenario_name(handle->scenario));

	cur_time = ktime_to_ns(ktime_get());/*sched_clock();*/
	ret = wait_event_interruptible(wq_handle->wq, cur_time < wq_handle->data);
	if (ret < 0) {
		DISPERR("wait %s interrupt by other ret %d on scenario %s\n",
			   path_event_name(event), ret,
			   ddp_get_scenario_name(handle->scenario));
	} else {
		DISPDBG("received event %s ret %d on scenario %s\n",
			   path_event_name(event), ret,
			   ddp_get_scenario_name(handle->scenario));
	}
	if (event_ts)
		*event_ts = wq_handle->data;

	return ret;
}

int dpmgr_wait_event(disp_path_handle dp_handle, DISP_PATH_EVENT event)
{
	return _dpmgr_wait_event(dp_handle, event, NULL);
}

int dpmgr_wait_event_ts(disp_path_handle dp_handle, DISP_PATH_EVENT event, unsigned long long *event_ts)
{
	return _dpmgr_wait_event(dp_handle, event, event_ts);
}

int dpmgr_signal_event(disp_path_handle dp_handle, DISP_PATH_EVENT event)
{
	ddp_path_handle handle;
	DPMGR_WQ_HANDLE *wq_handle;

	ASSERT(dp_handle != NULL);
	handle = (ddp_path_handle) dp_handle;
	wq_handle = &handle->wq_list[event];

	if (handle->wq_list[event].init) {
		wq_handle->data = ktime_to_ns(ktime_get());/*sched_clock();*/
		DISPDBG("wake up evnet %s on scenario %s\n", path_event_name(event),
			   ddp_get_scenario_name(handle->scenario));
		wake_up_interruptible(&(handle->wq_list[event].wq));
	}
	return 0;
}

static void dpmgr_irq_handler(DISP_MODULE_ENUM module, unsigned int regvalue)
{
	int i = 0;
	int j = 0;
	int irq_bits_num = 0;
	int irq_bit = 0;
	ddp_path_handle handle = NULL;

	handle = find_handle_by_module(module);
	if (handle == NULL)
		return;

	irq_bits_num = ddp_get_module_max_irq_bit(module);
	for (i = 0; i <= irq_bits_num; i++) {
		if (regvalue & (0x1 << i)) {
			irq_bit = MAKE_DDP_IRQ_BIT(module, i);
			dprec_stub_irq(irq_bit);
			for (j = 0; j < DISP_PATH_EVENT_NUM; j++) {
				if (handle->wq_list[j].init
				    && irq_bit == handle->irq_event_map[j].irq_bit) {
					dprec_stub_event(j);
					handle->wq_list[j].data = ktime_to_ns(ktime_get());/*sched_clock();*/
					DISPIRQ("irq signal event %s on cycle %llu on scenario %s\n",
					       path_event_name(j), handle->wq_list[j].data,
					       ddp_get_scenario_name(handle->scenario));
					wake_up_interruptible(&(handle->wq_list[j].wq));
				}
			}
		}
	}
}

int dpmgr_init(void)
{
	DISPMSG("ddp manager init\n");
	if (ddp_manager_init)
		return 0;
	ddp_manager_init = 1;
	/* ddp_debug_init(); */
	disp_init_irq();
	disp_register_irq_callback(dpmgr_irq_handler);
	return 0;
}

int dpmgr_factory_mode_test(int module_name, void *cmdqhandle, void *config)
{
	if (ddp_modules_driver[module_name] != 0) {
		if (ddp_modules_driver[module_name]->ioctl != 0) {
			DISPMSG(" %s factory_mode_test\n", ddp_get_module_name(DISP_MODULE_DPI));
			ddp_modules_driver[DISP_MODULE_DPI]->ioctl(module_name, cmdqhandle,
								   DDP_DPI_FACTORY_TEST, config);
		}
	}

	return 0;
}

int dpmgr_wait_ovl_available(int ovl_num)
{
	int ret = 1;

	return ret;
}
