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

#include <linux/types.h>
#include "primary_display.h"
#include "ddp_hal.h"
#include "disp_drv_log.h"
#include "disp_assert_layer.h"
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include "ddp_mmp.h"
#include "disp_drv_platform.h"
#include "disp_session.h"
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/semaphore.h>
#include <asm/cacheflush.h>
#include <linux/module.h>

/* /common part */
#define DAL_BPP             (2)
#define DAL_WIDTH           (DISP_GetScreenWidth())
#define DAL_HEIGHT          (DISP_GetScreenHeight())

#ifdef CONFIG_MTK_FB_SUPPORT_ASSERTION_LAYER
/* #if defined(CONFIG_MTK_FB_SUPPORT_ASSERTION_LAYER) */

#include "mtkfb_console.h"

/* --------------------------------------------------------------------------- */
#define DAL_FORMAT          (DISP_FORMAT_RGB565)
#define DAL_BG_COLOR        (dal_bg_color)
#define DAL_FG_COLOR        (dal_fg_color)

#define RGB888_To_RGB565(x) ((((x) & 0xF80000) >> 8) |                      \
			     (((x) & 0x00FC00) >> 5) |                      \
			     (((x) & 0x0000F8) >> 3))

#define MAKE_TWO_RGB565_COLOR(high, low)  (((low) << 16) | (high))

DEFINE_SEMAPHORE(dal_sem);

inline DAL_STATUS DAL_LOCK(void)
{
	if (down_interruptible(&dal_sem)) {
		DISP_LOG_PRINT(ANDROID_LOG_WARN, "DAL", "Can't get semaphore in %s()\n", __func__);
		return DAL_STATUS_LOCK_FAIL;
	}
	return DAL_STATUS_OK;
}

#define DAL_UNLOCK() up(&dal_sem)

inline MFC_STATUS DAL_CHECK_MFC_RET(MFC_STATUS expr)
{
	MFC_STATUS ret = expr;

	if (MFC_STATUS_OK != ret) {
		DISP_LOG_PRINT(ANDROID_LOG_WARN, "DAL",
			       "Warning: call MFC_XXX function failed in %s(), line: %d, ret: %d\n",
			       __func__, __LINE__, ret);
		return ret;
	}
	return MFC_STATUS_OK;
}


inline DISP_STATUS DAL_CHECK_DISP_RET(DISP_STATUS expr)
{
	DISP_STATUS ret = (expr);

	if (DISP_STATUS_OK != ret) {
		DISP_LOG_PRINT(ANDROID_LOG_WARN, "DAL",
			       "Warning: call DISP_XXX function failed in %s(), line: %d, ret: %d\n",
			       __func__, __LINE__, ret);
		return ret;
	}
	return DISP_STATUS_OK;
}

#define DAL_LOG(fmt, arg...) DISP_LOG_PRINT(ANDROID_LOG_INFO, "DAL", fmt, ##arg)
/* --------------------------------------------------------------------------- */

static MFC_HANDLE mfc_handle;
static void *dal_fb_addr;
static unsigned long dal_fb_pa;

/*static BOOL dal_enable_when_resume = FALSE;*/
static bool dal_disable_when_resume;
static unsigned int dal_fg_color = RGB888_To_RGB565(DAL_COLOR_WHITE);
static unsigned int dal_bg_color = RGB888_To_RGB565(DAL_COLOR_RED);
static char dal_print_buffer[1024];

bool dal_shown = false;
unsigned int isAEEEnabled = 0;

/* --------------------------------------------------------------------------- */


uint32_t DAL_GetLayerSize(void)
{
	/* avoid lcdc read buffersize+1 issue */
	return DAL_WIDTH * DAL_HEIGHT * DAL_BPP + 4096;
}

/*
static DAL_STATUS DAL_SetRedScreen(uint32_t *addr)
{
	uint32_t i;
	const uint32_t BG_COLOR = MAKE_TWO_RGB565_COLOR(DAL_BG_COLOR, DAL_BG_COLOR);

	for (i = 0; i < DAL_GetLayerSize() / sizeof(uint32_t); ++i)
		*addr++ = BG_COLOR;
	return DAL_STATUS_OK;
}
*/

DAL_STATUS DAL_SetScreenColor(DAL_COLOR color)
{
	uint32_t i;
	uint32_t size;
	uint32_t BG_COLOR;
	MFC_CONTEXT *ctxt = NULL;
	uint32_t offset;
	unsigned int *addr;
	color = RGB888_To_RGB565(color);
	BG_COLOR = MAKE_TWO_RGB565_COLOR(color, color);

	ctxt = (MFC_CONTEXT *)mfc_handle;
	if (!ctxt)
		return DAL_STATUS_FATAL_ERROR;
	if (ctxt->screen_color == color)
		return DAL_STATUS_OK;
	offset = MFC_Get_Cursor_Offset(mfc_handle);
	addr = (unsigned int *)(ctxt->fb_addr + offset);

	size = DAL_GetLayerSize() - offset;
	for (i = 0; i < size / sizeof(uint32_t); ++i)
		*addr++ = BG_COLOR;
	ctxt->screen_color = color;

	return DAL_STATUS_OK;
}
EXPORT_SYMBOL(DAL_SetScreenColor);

DAL_STATUS DAL_Init(unsigned long layerVA, unsigned long layerPA)
{
	pr_debug("%s, layerVA=0x%lx, layerPA=0x%lx\n", __func__, layerVA, layerPA);

	dal_fb_addr = (void *)layerVA;
	dal_fb_pa = layerPA;
	DAL_CHECK_MFC_RET(MFC_Open(&mfc_handle, dal_fb_addr,
				   DAL_WIDTH, DAL_HEIGHT, DAL_BPP, DAL_FG_COLOR, DAL_BG_COLOR));
	/* DAL_Clean(); */
	DAL_SetScreenColor(DAL_COLOR_RED);

	return DAL_STATUS_OK;
}

DAL_STATUS DAL_SetColor(unsigned int fgColor, unsigned int bgColor)
{
	if (NULL == mfc_handle)
		return DAL_STATUS_NOT_READY;

	DAL_LOCK();
	dal_fg_color = RGB888_To_RGB565(fgColor);
	dal_bg_color = RGB888_To_RGB565(bgColor);
	DAL_CHECK_MFC_RET(MFC_SetColor(mfc_handle, dal_fg_color, dal_bg_color));
	DAL_UNLOCK();

	return DAL_STATUS_OK;
}
EXPORT_SYMBOL(DAL_SetColor);

DAL_STATUS DAL_Dynamic_Change_FB_Layer(unsigned int isAEEEnabled)
{
	return DAL_STATUS_OK;
}

static int show_dal_layer(int enable)
{
	disp_session_input_config *session_input;
	disp_input_config *input;
	int ret;

	session_input = kzalloc(sizeof(*session_input), GFP_KERNEL);
	if (!session_input)
		return -ENOMEM;

	session_input->setter = SESSION_USER_AEE;
	session_input->config_layer_num = 1;
	input = &session_input->config[0];

	input->src_phy_addr = (void *)dal_fb_pa;
	input->layer_id = primary_display_get_option("ASSERT_LAYER");
	input->layer_enable = enable;
	input->src_offset_x = 0;
	input->src_offset_y = 0;
	input->src_width = DAL_WIDTH;
	input->src_height = DAL_HEIGHT;
	input->tgt_offset_x = 0;
	input->tgt_offset_y = 0;
	input->tgt_width = DAL_WIDTH;
	input->tgt_height = DAL_HEIGHT;
	input->alpha = 0x80;
	input->alpha_enable = 1;
	input->next_buff_idx = -1;
	input->src_pitch = DAL_WIDTH;
	input->src_fmt = DAL_FORMAT;
	input->next_buff_idx = -1;
	input->dirty_roi_num = 0;

	ret = primary_display_config_input_multiple(session_input);
	kfree(session_input);
	return ret;
}

DAL_STATUS DAL_Clean(void)
{
	/* const uint32_t BG_COLOR = MAKE_TWO_RGB565_COLOR(DAL_BG_COLOR, DAL_BG_COLOR); */
	DAL_STATUS ret = DAL_STATUS_OK;

	static int dal_clean_cnt;
	MFC_CONTEXT *ctxt = (MFC_CONTEXT *)mfc_handle;
	pr_err("[MTKFB_DAL] DAL_Clean\n");
	if (NULL == mfc_handle)
		return DAL_STATUS_NOT_READY;


	MMProfileLogEx(ddp_mmp_get_events()->dal_clean, MMProfileFlagStart, 0, 0);
	DAL_LOCK();
	if (MFC_STATUS_OK != MFC_ResetCursor(mfc_handle)) {
		pr_err("mfc_handle = %p\n", mfc_handle);
		goto End;
	}
	ctxt->screen_color = 0;
	DAL_SetScreenColor(DAL_COLOR_RED);


	/* TODO: if dal_shown=false, and 3D enabled, mtkfb may disable UI layer, please modify 3D driver */
	if (isAEEEnabled == 1) {
		show_dal_layer(0);
		/* DAL disable, switch UI layer to default layer 3 */
		pr_err("[DDP] isAEEEnabled from 1 to 0, %d\n", dal_clean_cnt++);
		isAEEEnabled = 0;
		DAL_Dynamic_Change_FB_Layer(isAEEEnabled); /* restore UI layer to DEFAULT_UI_LAYER */
	}

	dal_shown = false;
	dal_disable_when_resume = false;

	primary_display_trigger(0, NULL, 0);

End:
	DAL_UNLOCK();
	MMProfileLogEx(ddp_mmp_get_events()->dal_clean, MMProfileFlagEnd, 0, 0);
	return ret;
}
EXPORT_SYMBOL(DAL_Clean);

int is_DAL_Enabled(void)
{
	int ret = 0;
	ret = isAEEEnabled;
	return ret;
}

DAL_STATUS DAL_Printf(const char *fmt, ...)
{
	va_list args;
	uint i;
	DAL_STATUS ret = DAL_STATUS_OK;


	/* printk("[MTKFB_DAL] DAL_Printf mfc_handle=0x%08X, fmt=0x%08X\n", mfc_handle, fmt); */

	DISPFUNC();

	if (NULL == mfc_handle)
		return DAL_STATUS_NOT_READY;

	if (NULL == fmt)
		return DAL_STATUS_INVALID_ARGUMENT;

	MMProfileLogEx(ddp_mmp_get_events()->dal_printf, MMProfileFlagStart, 0, 0);
	DAL_LOCK();
	if (isAEEEnabled == 0) {
		pr_err("[DDP] isAEEEnabled from 0 to 1, ASSERT_LAYER=%d, dal_fb_pa 0x%lx\n",
		       primary_display_get_option("ASSERT_LAYER"), dal_fb_pa);

		isAEEEnabled = 1;
		DAL_Dynamic_Change_FB_Layer(isAEEEnabled); /* default_ui_layer config to changed_ui_layer */

		show_dal_layer(1);
	}
	va_start(args, fmt);
	i = vsprintf(dal_print_buffer, fmt, args);
	BUG_ON(i >= ARRAY_SIZE(dal_print_buffer));
	va_end(args);
	DAL_CHECK_MFC_RET(MFC_Print(mfc_handle, dal_print_buffer));

	__inner_flush_dcache_all();


	if (!dal_shown)
		dal_shown = true;

	ret = primary_display_trigger(0, NULL, 0);


	DAL_UNLOCK();

	MMProfileLogEx(ddp_mmp_get_events()->dal_printf, MMProfileFlagEnd, 0, 0);

	return ret;
}
EXPORT_SYMBOL(DAL_Printf);

DAL_STATUS DAL_OnDispPowerOn(void)
{
	return DAL_STATUS_OK;
}

/* ########################################################################## */
/* !CONFIG_MTK_FB_SUPPORT_ASSERTION_LAYER */
/* ########################################################################## */
#else
unsigned int isAEEEnabled = 0;

uint32_t DAL_GetLayerSize(void)
{
	/* xuecheng, avoid lcdc read buffersize+1 issue */
	return DAL_WIDTH * DAL_HEIGHT * DAL_BPP + 4096;
}

DAL_STATUS DAL_Init(unsigned long layerVA, unsigned long layerPA)
{
	NOT_REFERENCED(layerVA);
	NOT_REFERENCED(layerPA);

	return DAL_STATUS_OK;
}

DAL_STATUS DAL_SetColor(unsigned int fgColor, unsigned int bgColor)
{
	NOT_REFERENCED(fgColor);
	NOT_REFERENCED(bgColor);

	return DAL_STATUS_OK;
}
EXPORT_SYMBOL(DAL_SetColor);

DAL_STATUS DAL_Clean(void)
{
	pr_err("[MTKFB_DAL] DAL_Clean is not implemented\n");
	return DAL_STATUS_OK;
}
EXPORT_SYMBOL(DAL_Clean);

DAL_STATUS DAL_Printf(const char *fmt, ...)
{
	NOT_REFERENCED(fmt);
	pr_err("[MTKFB_DAL] DAL_Printf is not implemented\n");
	return DAL_STATUS_OK;
}
EXPORT_SYMBOL(DAL_Printf);

DAL_STATUS DAL_OnDispPowerOn(void)
{
	return DAL_STATUS_OK;
}

DAL_STATUS DAL_SetScreenColor(DAL_COLOR color)
{
	return DAL_STATUS_OK;
}
EXPORT_SYMBOL(DAL_SetScreenColor);

int is_DAL_Enabled(void)
{
	return 0;
}

#endif /* CONFIG_MTK_FB_SUPPORT_ASSERTION_LAYER */
