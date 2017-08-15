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

#define __MTK_COMMON_H__

/* MTK device context driver private data */
struct mtk_config {

    /* mtk mfg mapped register */
    volatile void *mfg_register;

    struct clk *clk_mfg;
    struct clk *clk_mfg_async;

    struct clk *clk_mfg_core0;
    struct clk *clk_mfg_core1;

    struct clk *clk_mfg_main;

};

#define MTKCLK_prepare_enable(clk) \
	if (config->clk) { \
		if (clk_prepare_enable(config->clk)) \
		pr_alert("MALI: clk_prepare_enable failed when enabling" #clk); }

#define MTKCLK_disable_unprepare(clk) \
	if (config->clk) { \
		clk_disable_unprepare(config->clk); }



#endif /* __MTK_COMMON_H__ */
