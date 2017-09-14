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

#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include "disp_session.h"
#include "disp_drv_log.h"
#include "cmdq_record.h"
#include "disp_rect.h"
#include "ddp_manager.h"
#include "disp_lcm.h"
#include "primary_display.h"

#include "disp_partial.h"

static void _update_layer_dirty(OVL_CONFIG_STRUCT *old, disp_input_config *src,
		struct disp_rect *layer_roi)
{
#if 0
	if (rect_isEmpty(layer_roi)) {
		if (!src->layer_enable) {
			if (old->layer_en) {
				/* layer enable to disable, set full dirty*/
				DISPDBG("roi layer %d disable, change to full roi", old->layer);
				layer_roi->x = src->tgt_offset_x;
				layer_roi->y = src->tgt_offset_y;
				layer_roi->width = src->tgt_width;
				layer_roi->height = src->tgt_height;
			}
		} else {
			if (src->buffer_source == DISP_BUFFER_ALPHA && src->layer_enable) {
				/* dim layer, set full dirty*/
				DISPDBG("roi dim layer %d change to full roi", old->layer);
				assign_full_lcm_roi(layer_roi);
			} /*else if (old->addr != (unsigned long)src->src_phy_addr) {
			    DISPMSG("roi layer %d buffer change, change to full roi", old->layer);
			    layer_roi->x = src->tgt_offset_x;
			    layer_roi->y = src->tgt_offset_y;
			    layer_roi->width = src->tgt_width;
			    layer_roi->height = src->tgt_height;
			    }*/
		}
	}
#endif
}

static void _convert_picture_to_ovl_dirty(disp_input_config *src,
		struct disp_rect *in, struct disp_rect *out)
{
	struct disp_rect layer_roi = {0, 0, 0, 0};
	struct disp_rect pic_roi = {0, 0, 0, 0};
	struct disp_rect result = {0, 0, 0, 0};

	layer_roi.x = src->src_offset_x;
	layer_roi.y = src->src_offset_y;
	layer_roi.width = src->src_width;
	layer_roi.height = src->src_height;

	pic_roi.x = in->x;
	pic_roi.y = in->y;
	pic_roi.width = in->width;
	pic_roi.height = in->height;

	rect_intersect(&layer_roi, &pic_roi, &result);

	out->x = result.x - layer_roi.x + src->tgt_offset_x;
	out->y = result.y - layer_roi.y + src->tgt_offset_y;
	out->width = result.width;
	out->height = result.height;

	DISPDBG("pic to ovl dirty(%d,%d,%d,%d)->(%d,%d,%d,%d)\n",
			pic_roi.x, pic_roi.y, pic_roi.width, pic_roi.height,
			out->x, out->y, out->width, out->height);
}

int disp_partial_compute_ovl_roi(struct disp_frame_cfg_t *cfg,
		disp_ddp_path_config *old_cfg, struct disp_rect *result)
{
	int i = 0;
	int j = 0;
	int size = 0;
	int num = 0;
	int disable_layer = 0;
	void __user *roi_addr = NULL;
	OVL_CONFIG_STRUCT *old_ovl_cfg = NULL;
	static struct layer_dirty_roi layers[20];
	struct layer_dirty_roi *layer_roi_addr = &layers[0];
	struct disp_rect layer_roi = {0, 0, 0, 0};

	if (ddp_debug_force_roi()) {
		assign_full_lcm_roi(result);
		result->x = ddp_debug_force_roi_x();
		result->y = ddp_debug_force_roi_y();
		result->width = ddp_debug_force_roi_w();
		result->height = ddp_debug_force_roi_h();
		return 0;
	}

	for (i = 0; i < cfg->input_layer_num; i++) {
		struct disp_rect layer_total_roi = {0, 0, 0, 0};
		disp_input_config *input_cfg = &cfg->input_cfg[i];

		if (!input_cfg->layer_enable) {
			disable_layer++;
			continue;
		}
		num = input_cfg->dirty_roi_num;
		if (input_cfg->dirty_roi_num) {
			roi_addr = input_cfg->dirty_roi_addr;
			size = num * sizeof(struct layer_dirty_roi);
			DISPDBG("layer %d dirty num %d\n",
					i, input_cfg->dirty_roi_num);
			if (copy_from_user(layer_roi_addr, roi_addr, size)) {
				pr_err("[drity roi]: copy_from_user failed! line:%d\n", __LINE__);
				input_cfg->dirty_roi_num = 0;
			} else {
				/* 1. compute picture dirty roi*/
				for (j = 0; j < input_cfg->dirty_roi_num; j++) {
					layer_roi_addr += j;
					layer_roi.x = layer_roi_addr->dirty_x;
					layer_roi.y = layer_roi_addr->dirty_y;
					layer_roi.width = layer_roi_addr->dirty_w;
					layer_roi.height = layer_roi_addr->dirty_h;
					rect_join(&layer_roi, &layer_total_roi, &layer_total_roi);
				}
				/* 2. convert picture dirty to ovl dirty */
				if (!rect_isEmpty(&layer_total_roi))
					_convert_picture_to_ovl_dirty(input_cfg, &layer_total_roi, &layer_total_roi);
			}
		}

		/* 3. full dirty if num euals 0 */
		if (input_cfg->dirty_roi_num == 0 && input_cfg->layer_enable) {
			DISPDBG("layer %d dirty num 0\n", i);
			if (0) {
				layer_roi.x = input_cfg->tgt_offset_x;
				layer_roi.y = input_cfg->tgt_offset_y;
				layer_roi.width = input_cfg->tgt_width;
				layer_roi.height = input_cfg->tgt_height;
				rect_join(&layer_roi, &layer_total_roi, &layer_total_roi);
			} else {
				assign_full_lcm_roi(result);
				/* break if full lcm roi */
				break;
			}
		}
		/* 4. deal with other cases:layer disable, dim layer*/
		old_ovl_cfg = &(old_cfg->ovl_config[input_cfg->layer_id]);
		_update_layer_dirty(old_ovl_cfg, input_cfg, &layer_total_roi);
		rect_join(&layer_total_roi, result, result);

		/*break if roi is full lcm */
		if (is_equal_full_lcm(result))
			break;

	}
	if (disable_layer >= cfg->input_layer_num) {
		DISPMSG(" all layer disabled, force full roi\n");
		assign_full_lcm_roi(result);
	}

	return 0;
}

int disp_partial_get_project_option(void)
{
	struct device_node *mtkfb_node = NULL;
	static int inited;
	static int supported;

	if (inited)
		return supported;

	mtkfb_node = of_find_compatible_node(NULL, NULL, "mediatek,mtkfb");
	if (!mtkfb_node)
		goto out;

	of_property_read_u32(mtkfb_node, "partial-update", &supported);

out:
	inited = 1;
	return supported;
}


int disp_partial_is_support(void)
{
	disp_lcm_handle *plcm = primary_get_lcm();

	if (disp_partial_get_project_option() &&
		disp_lcm_is_partial_support(plcm) &&
		!disp_lcm_is_video_mode(plcm) &&
		disp_helper_get_option(DISP_OPT_PARTIAL_UPDATE))
		return 1;

	return 0;
}

void assign_full_lcm_roi(struct disp_rect *roi)
{
	roi->x = 0;
	roi->y = 0;
	roi->width = primary_display_get_width();
	roi->height = primary_display_get_height();
}

int is_equal_full_lcm(const struct disp_rect *roi)
{
	static struct disp_rect full_roi;

	if (full_roi.width == 0)
		assign_full_lcm_roi(&full_roi);

	return rect_equal(&full_roi, roi);
}

void disp_patial_lcm_validate_roi(disp_lcm_handle *plcm, struct disp_rect *roi)
{
	int x = roi->x;
	int y = roi->y;
	int w = roi->width;
	int h = roi->height;

	disp_lcm_validate_roi(plcm, &roi->x, &roi->y, &roi->width, &roi->height);
	DISPDBG("lcm verify partial(%d,%d,%d,%d) to (%d,%d,%d,%d)\n",
			x, y, w, h, roi->x, roi->y, roi->width, roi->height);
}

int disp_partial_update_roi_to_lcm(disp_path_handle dp_handle,
		struct disp_rect partial, void *cmdq_handle)
{
	return dpmgr_path_update_partial_roi(dp_handle, partial, cmdq_handle);
}
