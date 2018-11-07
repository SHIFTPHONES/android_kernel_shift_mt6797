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

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
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

#ifdef BUILD_LK
#define GPIO_LCD_BIAS_ENN_PIN (41 | 0x80000000)
#define GPIO_LCD_BIAS_ENP_PIN (42 | 0x80000000)
#define GPIO_LCM_18        	(208 | 0x80000000)
#endif

#ifndef BUILD_LK
void lcm_18_pin_enable(bool on);
void lcm_power_enn_enable(bool on);
void lcm_power_enp_enable(bool on);
#endif

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
{0x00,1,{0x00}},     
{0xff,3,{0x87,0x16,0x01}},     
{0x00,1,{0x80}},     
{0xff,2,{0x87,0x16}},     
{0x00,1,{0x80}},     
{0xC0,15,{0x00,0x77,0x00,0x10,0x10,0x00,0x77,0x10,0x10,0x00,0x7e,0x00,0x10,0x10,0x00}},     
{0x00,1,{0x80}},  
{0xF3,1,{0x70}},     
{0x00,1,{0xA0}},     
{0xC0,7,{0x05,0x01,0x01,0x09,0x01,0x19,0x09}},     
{0x00,1,{0xD0}},     
{0xC0,7,{0x05,0x01,0x01,0x09,0x01,0x19,0x09}},     
{0x00,1,{0x82}},     
{0xA5,3,{0x20,0x01,0x0C}},     
{0x00,1,{0x87}},  
{0xA5,4,{0x00,0x00,0x00,0x77}},    
{0x00,1,{0xA0}},      
{0xB3,1,{0x32}},   
{0x00,1,{0xA6}},   
{0xB3,1,{0x48}}, 
{0x00,1,{0x80}},
{0xC2,12,{0x82,0x00,0x00,0x00,0x81,0x00,0x00,0x00,0x84,0x00,0x32,0x8A}},
//CKV Setting //20170308
{0x00,1,{0xB0}},
{0xC2,15,{0x80,0x04,0x00,0x07,0x86,0x01,0x05,0x00,0x07,0x86,0x82,0x02,0x00,0x07,0x86}},
//CKV Setting //20170421 add CKV5 setting for D2U & GAS2
{0x00,1,{0xC0}},
{0xC2,10,{0x81,0x03,0x00,0x07,0x86,0x81,0x03,0x00,0x80,0x00}},
//CKV Setting
{0x00,1,{0xDA}},      
{0xC2,2,{0x33,0x33}},   
{0x00,1,{0xAA}},   
{0xC3,2,{0x9C,0x99}}, 
//CKV5 TP term setting //20170421
{0x00,1,{0xAC}},
{0xC3,1,{0x99}},

{0x00,1,{0xD3}},
{0xC3,1,{0x10}},
{0x00,1,{0xE3}},
{0xC3,1,{0x10}},

//PanelIF Initial Code //20170502
{0x00,1,{0x80}},
{0xCC,12,{0x02,0x03,0x06,0x07,0x08,0x09,0x0A,0x18,0x22,0x22,0x22,0x22}},
{0x00,1,{0x90}},
{0xCC,12,{0x03,0x02,0x09,0x08,0x07,0x06,0x19,0x0A,0x22,0x22,0x22,0x22}},
{0x00,1,{0xA0}},// no dir 1CCA0
{0xCC,15,{0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,0x18,0x19,0x20,0x21,0x04,0x14,0x15,0x0A,0x22}},
{0x00,1,{0xB0}},
{0xCC,5,{0x22,0x22,0x22,0x22,0x22}},
{0x00,1,{0x80}},// slp inCB80 //0508
{0xCB,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0x00,1,{0x90}},// power on 1CB90
{0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0x00,1,{0xA0}},// power on 2CBA0
{0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0x00,1,{0xB0}},// power on 3CBB0
{0xCB,2,{0x00,0x00}},
{0x00,1,{0xC0}},// power off 1CBC0 0508 change poweroff2 timing
{0xCB,15,{0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x00,0x00,0x00,0x00,0x05,0x05,0x05}},
{0x00,1,{0xD0}},
{0xCB,15,{0x00,0x00,0x00,0x05,0x05,0x05,0x05,0x05,0x00,0x00,0x05,0x00,0x00,0x00,0x00}},
{0x00,1,{0xE0}},
{0xCB,2,{0x00,0x00}},
{0x00,1,{0xF0}},
{0xCB,8,{0x0F,0x00,0x00,0x3F,0x00,0xC0,0x00,0x00}},

//20170502
{0x00,1,{0x80}},
{0xCD,15,{0x22,0x22,0x22,0x22,0x01,0x06,0x04,0x08,0x07,0x18,0x17,0x05,0x03,0x1A,0x22}},
{0x00,1,{0x90}},
{0xCD,3,{0x0F,0x0E,0x0D}},
{0x00,1,{0xA0}},
{0xCD,15,{0x22,0x02,0x03,0x05,0x07,0x08,0x18,0x17,0x04,0x06,0x1A,0x22,0x22,0x22,0x22}},
{0x00,1,{0xB0}},
{0xCD,3,{0x0F,0x0E,0x0D}},
//20170313
{0x00,1,{0x81}},
{0xF3,12,{0x10,0x82,0xC0,0x42,0x80,0xC0,0x10,0x82,0xC0,0x42,0x80,0xC0}},
{0x00,1,{0x90}},
{0xCF,4,{0xFF,0x00,0xFE,0x00}},
// TP start&&count //20170308
{0x00,1,{0x94}},
{0xCF,4,{0x00,0x00,0x10,0x20}},
{0x00,1,{0xA4}},
{0xCF,4,{0x00,0x07,0x01,0x80}},
{0x00,1,{0xD0}},
{0xCF,1,{0x08}},
//TP Initial Code
{0x00,1,{0x80}},
{0xCE,9,{0x25,0x00,0x78,0x00,0x78,0xFF,0x00,0x20,0x05}},
{0x00,1,{0x90}},
{0xCE,8,{0x00,0x5C,0x0a,0x35,0x00,0x5C,0x00,0x7b}},
{0x00,1,{0xB0}},
{0xCE,6,{0x00,0x00,0x60,0x60,0x00,0x60}},
{0x00,1,{0xC0}},
{0xF4,2,{0x93,0x36}},
{0x00,1,{0x00}},
{0xE1,24,{0x00,0x07,0x18,0x2B,0x37,0x42,0x55,0x64,0x6B,0x73,0x7d,0x87,0x70,0x67,0x64,0x5d,0x4f,0x44,0x35,0x2c,0x25,0x18,0x09,0x07}},

{0x00,1,{0x00}},
{0xE2,24,{0x00,0x07,0x18,0x2B,0x37,0x42,0x55,0x64,0x6B,0x73,0x7d,0x87,0x70,0x67,0x64,0x5d,0x4f,0x44,0x35,0x2c,0x25,0x18,0x09,0x07}},
{0x00,1,{0x80}},
{0xC5,10,{0x00,0xC1,0xDD,0xC4,0x14,0x1E,0x00,0x55,0x50,0x00}},

{0x00,1,{0x90}},
{0xC5,10,{0x55,0x1E,0x14,0x00,0x88,0x10,0x4B,0x3c,0x55,0x50}},
{0x00,1,{0x00}},
{0xD8,2,{0x31,0x31}},// GVDD 5.3V

{0x00,1,{0x00}},
{0xD9,5,{0x80,0xB1,0xB1,0xB1,0xB1}},
{0x00,1,{0x88}},
{0xC3,2,{0x33,0x33}},
{0x00,1,{0x98}},
{0xC3,2,{0x33,0x33}},

{0x00,1,{0x80}},
{0xC4,1,{0x41}},
{0x00,1,{0x94}},
{0xC5,1,{0x48}},

//vgl pump disable   20170328
{0x00,1,{0xC3}},
{0xF5,1,{0x26}},
{0x00,1,{0xC7}},
{0xF5,1,{0x26}},
{0x00,1,{0xD3}},
{0xF5,1,{0x26}},
{0x00,1,{0xD7}},
{0xF5,1,{0x26}},
{0x00,1,{0x95}},
{0xF5,1,{0x26}},
{0x00,1,{0x98}},
{0xF5,1,{0x26}},
{0x00,1,{0xB1}},
{0xF5,1,{0x21}},


{0x00,1,{0x87}},
{0xC3,2,{0x33,0x33}},
{0x00,1,{0x97}},
{0xC3,2,{0x33,0x33}},
//CKV EQ enable
{0x00,1,{0x83}},
{0xC3,1,{0x44}},
{0x00,1,{0x93}},
{0xC3,1,{0x44}},
//REST EQ enable
{0x00,1,{0x81}},
{0xC3,1,{0x33}},
{0x00,1,{0x91}},
{0xC3,1,{0x33}},

{0x00,1,{0x81}},
{0xCF,1,{0x04}},
{0x00,1,{0x84}},
{0xCF,1,{0x04}},
//0411 add pwroff SD pull GND
{0x00,1,{0x81}},
{0xC4,1,{0xC0}},

{0x00,1,{0x8D}},
{0xF5,1,{0x21}},//pwr off
{0x00,1,{0x8C}},
{0xF5,1,{0x15}},//pwr on

{0x00,1,{0xDA}},
{0xCF,1,{0x16}},

{0x00,1,{0x80}},
{0xCE,1,{0x05}},
//0508 poweroff2 add one frame
{0x00,1,{0xC1}},
{0xC0,1,{0x11}},
{0x00,1,{0x90}},
{0xC5,1,{0x77}},

{0x00,1,{0x00}},
{0xFF,3,{0x00,0x00,0x00}},
{0x00,1,{0x00}},
{0xFF,2,{0x00,0x00}},

{0x11,0,{}},
{REGFLAG_DELAY, 200, {}},   
{0x29,0,{}},
{REGFLAG_DELAY, 20, {}},
{0xB7,2,{0x59,0x02}},//video mode on
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

#if 0	//(LCM_DSI_CMD_MODE)
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
	params->dsi.vertical_frontporch = 16;
	params->dsi.vertical_active_line = FRAME_HEIGHT;
	params->dsi.clk_lp_per_line_enable = 0;

	params->dsi.horizontal_sync_active = 16;
	params->dsi.horizontal_backporch = 32;
	params->dsi.horizontal_frontporch = 32;
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
 #if defined(GPIO_LCM_18)
     lcm_util.set_gpio_mode(GPIO_LCM_18, GPIO_MODE_00);
     lcm_util.set_gpio_dir(GPIO_LCM_18, GPIO_DIR_OUT);
     lcm_util.set_gpio_out(GPIO_LCM_18, GPIO_OUT_ONE);
	 dprintf(0, "[LK]lcyshift--lcm_init GPIO_LCM_18 1\n");
 #else
	lcm_18_pin_enable(1);
 #endif 
	
	
 #if defined(GPIO_LCD_BIAS_ENP_PIN)
     lcm_util.set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
     lcm_util.set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
     lcm_util.set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
 #else
	lcm_power_enp_enable(1);
 #endif
     MDELAY(10);
 
 #if defined(GPIO_LCD_BIAS_ENN_PIN)
     lcm_util.set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
     lcm_util.set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
     lcm_util.set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
 #else
	lcm_power_enn_enable(1);

 #endif
    MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(1);//10
	SET_RESET_PIN(1);
	MDELAY(10);

	/* when phone initial , config output high, enable backlight drv chip */

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
	
	 SET_RESET_PIN(0);
         MDELAY(20);
         SET_RESET_PIN(1);
         MDELAY(20);

 #if defined(GPIO_LCD_BIAS_ENP_PIN)
     lcm_util.set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
     lcm_util.set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
     lcm_util.set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
 #else
	lcm_power_enp_enable(0);
 #endif
     MDELAY(10);
 
 #if defined(GPIO_LCD_BIAS_ENN_PIN)
     lcm_util.set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
     lcm_util.set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
     lcm_util.set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
 #else
	lcm_power_enn_enable(0);
 #endif
    MDELAY(10);
	
 #if defined(GPIO_LCM_18)
     lcm_util.set_gpio_mode(GPIO_LCM_18, GPIO_MODE_00);
     lcm_util.set_gpio_dir(GPIO_LCM_18, GPIO_DIR_OUT);
     lcm_util.set_gpio_out(GPIO_LCM_18, GPIO_OUT_ZERO);
 #else
	lcm_18_pin_enable(0);
 #endif

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

/*
static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	unsigned int cmd = 0x51;
	unsigned char count = 1;
	unsigned char value = level;
#ifdef BUILD_LK
	dprintf(0, "[LK]lcyshift--lcm_setbacklight_cmdq\n");
#else
        printk("[Kernel]lcyshift--lcm_setbacklight_cmdq\n");
#endif
	dsi_set_cmdq_V22(handle, cmd, count, &value, 1);
}
*/

LCM_DRIVER ft8716_fhd_dsi_cmd_shift5me_lcm_drv = {
	.name           = "ft8716_fhd_dsi_cmd_shift5me",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	//.compare_id     = lcm_compare_id,
	//.set_backlight_cmdq  = lcm_setbacklight_cmdq,
	//.set_backlight = lcm_setbacklight,
#if (LCM_DSI_CMD_MODE)
	.update         = lcm_update,
#endif
};

