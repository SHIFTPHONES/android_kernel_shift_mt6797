#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/upmu_common.h>
	#include <platform/mt_gpio.h>
	#include <platform/mt_i2c.h>
	#include <platform/mt_pmic.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
#ifdef CONFIG_MTK_LEGACY
	#include <mach/mt_pm_ldo.h>
	#include <mach/mt_gpio.h>
#endif
#endif
#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL, fmt)
#else
#define LCD_DEBUG(fmt)  pr_debug(fmt)
#endif
#ifdef CONFIG_MTK_LEGACY
#include <cust_i2c.h>
#include <mach/gpio_const.h>
#include <cust_gpio_usage.h>
#endif


/**
 * Local Constants
 */
#define LCM_DSI_CMD_MODE	1
#define FRAME_WIDTH	(1080)
#define FRAME_HEIGHT	(1920)
#define GPIO_65132_EN GPIO_LCD_BIAS_ENP_PIN

#define REGFLAG_PORT_SWAP	0xFFFA
#define REGFLAG_DELAY	0xFFFC
#define REGFLAG_END_OF_TABLE	0xFFFD   /* END OF REGISTERS MARKER */

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif
/* static unsigned int lcm_esd_test = FALSE; */     /* only for ESD test */
/**
 * Local Variables
 */

static const unsigned int BL_MIN_LEVEL = 20;
static LCM_UTIL_FUNCS lcm_util;

//#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))

/**
 * Local Functions
 */
#define dsi_set_cmd_by_cmdq_dual(handle, cmd, count, ppara, force_update) \
		lcm_util.dsi_set_cmdq_V23(handle, cmd, (unsigned char)(count), \
					  (unsigned char *)(ppara), (unsigned char)(force_update))
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)		lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)					lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define dsi_swap_port(swap)				lcm_util.dsi_swap_port(swap)

#define set_gpio_lcd_enp(cmd) lcm_util.set_gpio_lcd_enp_bias(cmd)
#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/*#include <linux/jiffies.h> */
#include <linux/uaccess.h>
/*#include <linux/delay.h> */
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#endif
//extern struct platform_driver lcm_power_driver;//dongsheng
void lcm_gpio_power_on(void);//dongsheng
void lcm_gpio_power_off(void);//dongsheng
void lcm_gpio_reset(int i);//dongsheng
struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
//{0xFE,1,{0x04}},  // for dimming  
//{0xe1,1,{0x00}},	// for dimming 
{0xFE,1,{0x0D}},     
{0x42,1,{0x00}},     
{0x18,1,{0x08}},     
{0x08,1,{0x41}},     
{0x46,1,{0x02}},     
{0x1E,1,{0x04}},     
{0x1E,1,{0x00}},  
{0xFE,1,{0x0A}},     
{0x24,1,{0x17}},     
{0x04,1,{0x07}},     
{0x1A,1,{0x0C}},     
{0x0F,1,{0x44}},     
{0xFE,1,{0x0B}},     
{0x28,1,{0x40}},     
{0x29,1,{0x4F}},  
{0xFE,1,{0x0D}},  //esd add //error report   
{0x20,1,{0xff}},  //esd add     
{0x21,1,{0xff}},  //esd add  
{0xFE,1,{0x04}},  //TE adjust   
{0x0A,1,{0xFF}},   
{0xFE,1,{0x00}},
{0xC2,1,{0x08}},//0x08  cmd mode
{0x35,1,{0x00}},//for te
{0x44,2,{0x03,0x80}},//for te
{0xFE,1,{0x00}},//0312
{0x51,1,{0x00}},
{0x11,0,{}},
{REGFLAG_DELAY, 120, {}},   
{0x29,0,{}},
{REGFLAG_DELAY, 20, {}},
};


static struct LCM_setting_table lcm_suspend_setting[] = {
{0x28,1,{0x00}},

 {REGFLAG_DELAY, 50, {}},

{0x10,1,{0x00}},

 {REGFLAG_DELAY, 120, {}},
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned cmd;

		cmd = table[i].cmd;

		switch (cmd) {
		case REGFLAG_DELAY:
#ifdef BUILD_LK
			dprintf(0, "[LK]REGFLAG_DELAY\n");
#endif
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;
		case REGFLAG_PORT_SWAP:
#ifdef BUILD_LK
			dprintf(0, "[LK]push_table end\n");
#endif
			dsi_swap_port(1);
			break;
		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

/*
 * LCM Driver Implementations
 */

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->dbi.te_mode 			= LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if 1	//(LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
#else
	params->dsi.mode = BURST_VDO_MODE;
#endif


	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
	params->dsi.packet_size = 256;

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 4;
	params->dsi.vertical_backporch = 16;
	params->dsi.vertical_frontporch = 8;
	params->dsi.vertical_active_line = FRAME_HEIGHT;
	params->dsi.clk_lp_per_line_enable = 0;

	params->dsi.horizontal_sync_active = 10;
	params->dsi.horizontal_backporch = 40;
	params->dsi.horizontal_frontporch = 30;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	
	params->dsi.PLL_CLOCK = 430;


	params->dsi.esd_check_enable = 1;
   	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0A; //53
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C; //24

}

static void lcm_init(void)
{
lcm_gpio_reset(0);

lcm_gpio_power_on();

MDELAY(10);

lcm_gpio_reset(1);

MDELAY(20);
	
push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table),1);

#ifdef BUILD_LK
	dprintf(0, "[LK]lcyshift--push_table end\n");
#else
        printk("[Kernel]lcyshift--lcm_init:push_table end\n");
#endif
     
}

static void lcm_suspend(void)
{
#ifdef BUILD_LK
	dprintf(0, "[LK]lcyshift--lcm_suspend\n");
#else
        printk("[Kernel]lcyshift--lcm_suspend\n");
#endif
push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	
lcm_gpio_reset(0);

lcm_gpio_power_off();	
}

static void lcm_resume(void)
{
#ifdef BUILD_LK
	dprintf(0, "[LK]lcyshift--lcm_resume\n");
#else
        printk("[Kernel]lcyshift--lcm_resume\n");
#endif
	lcm_init();
}

static void lcm_update(unsigned int x, unsigned int y,
		       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);
	unsigned int data_array[16];
	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	unsigned int cmd = 0x51;
	unsigned char count = 1;
	unsigned char value = level;
    printk("[Kernel]lcyshift--lcm_setbacklight_cmdq level=[%d] \n",value);
	dsi_set_cmdq_V22(handle, cmd, count, &value, 1);
}

LCM_DRIVER rm67195_fhd_dsi_cmd_shift6m_lcm_drv = {
	.name           = "rm67195_fhd_dsi_cmd_shift6m",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	//.compare_id     = lcm_compare_id,
	.set_backlight_cmdq  = lcm_setbacklight_cmdq,
	//.set_backlight = lcm_setbacklight,
#if (LCM_DSI_CMD_MODE)
	.update         = lcm_update,
#endif
};

