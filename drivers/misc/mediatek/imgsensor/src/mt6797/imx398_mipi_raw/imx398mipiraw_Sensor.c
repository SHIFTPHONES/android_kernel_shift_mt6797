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

/*****************************************************************************
 *
 * Filename:
 * ---------
 *     IMX398mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "imx398mipiraw_Sensor.h"

/****************************Modify Following Strings for Debug****************************/
#define PFX "IMX398_camera_sensor"
#define LOG_1 LOG_INF("IMX398,MIPI 4LANE\n")
/****************************   Modify end    *******************************************/

#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __func__, ##args)

#define BYTE               unsigned char

static BOOL read_spc_flag = FALSE;

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static BYTE imx398_SPC_data[252] = { 0 };

static imgsensor_info_struct imgsensor_info = {
	.sensor_id = IMX398_SENSOR_ID,	/* record sensor id defined in Kd_imgsensor.h */

	.checksum_value = 0x11dcf259,	/* checksum value for Camera Auto Test */
	.pre = {
		.pclk = 319200000,	/* record different mode's pclk */
		.linelength = 5536,	/* record different mode's linelength */
		.framelength = 1920,	/* record different mode's framelength */
		.startx = 0,	/* record different mode's startx of grabwindow */
		.starty = 0,	/* record different mode's starty of grabwindow */
		.grabwindow_width = 2328,	/* 2096,        //record different mode's width of grabwindow */
		.grabwindow_height = 1748,	/* 1552,       //record different mode's height of grabwindow */
		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 30,
		/* following for GetDefaultFramerateByScenario() */
		.max_framerate = 300,
	},
	.cap = {		/* 30  fps  capture */
		.pclk = 600000000,
		.linelength = 5536,
		.framelength = 3608,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4656,	/* 4192, */
		.grabwindow_height = 3496,	/* 3104, */
		.mipi_data_lp2hs_settle_dc = 30,
		.max_framerate = 300,
	},
	.cap1 = {		/* 24 fps  capture */
		.pclk = 300000000,
		.linelength = 5536,
		.framelength = 3612,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4656,	/* 4192, */
		.grabwindow_height = 3496,	/* 3104, */
		.mipi_data_lp2hs_settle_dc = 30,
		.max_framerate = 150,
	},
	.normal_video = {	/* 30  fps  capture */
		.pclk = 480000000,
		.linelength = 5536,
		.framelength = 2888,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4656,	/* 4192, */
		.grabwindow_height = 2608,	/* 3104, */
		.mipi_data_lp2hs_settle_dc = 30,
		.max_framerate = 300,
	},
	.hs_video = {		/* 120 fps */
		.pclk = 600000000,
		.linelength = 5536,
		.framelength = 902,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2216,
		.grabwindow_height = 834,
		.mipi_data_lp2hs_settle_dc = 30,
		.max_framerate = 1200,
	},
	.slim_video = {
		.pclk = 240000000,
		.linelength = 5536,
		.framelength = 1444,	/* 1640, */
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2216,
		.grabwindow_height = 834,
		.mipi_data_lp2hs_settle_dc = 30,
		.max_framerate = 300,
	},
	.custom1 = {
		.pclk = 300000000,	/* record different mode's pclk */
		.linelength = 5536,	/* record different mode's linelength */
		.framelength = 1806,	/* record different mode's framelength */
		.startx = 0,	/* record different mode's startx of grabwindow */
		.starty = 0,	/* record different mode's starty of grabwindow */
		.grabwindow_width = 2328,	/* 2096,        //record different mode's width of grabwindow */
		.grabwindow_height = 1748,	/* 1552,       //record different mode's height of grabwindow */
		/*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
		.mipi_data_lp2hs_settle_dc = 30,
		/*   following for GetDefaultFramerateByScenario()  */
		.max_framerate = 300,
	},
	.custom2 = {
		.pclk = 300000000,	/* record different mode's pclk */
		.linelength = 5536,	/* record different mode's linelength */
		.framelength = 1806,	/* record different mode's framelength */
		.startx = 0,	/* record different mode's startx of grabwindow */
		.starty = 0,	/* record different mode's starty of grabwindow */
		.grabwindow_width = 2328,	/* 2096,        //record different mode's width of grabwindow */
		.grabwindow_height = 1748,	/* 1552,       //record different mode's height of grabwindow */
		/*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
		.mipi_data_lp2hs_settle_dc = 30,
		/*   following for GetDefaultFramerateByScenario()  */
		.max_framerate = 300,
	},
	.custom3 = {
		.pclk = 300000000,	/* record different mode's pclk */
		.linelength = 5536,	/* record different mode's linelength */
		.framelength = 1806,	/* record different mode's framelength */
		.startx = 0,	/* record different mode's startx of grabwindow */
		.starty = 0,	/* record different mode's starty of grabwindow */
		.grabwindow_width = 2328,	/* 2096,        //record different mode's width of grabwindow */
		.grabwindow_height = 1748,	/* 1552,       //record different mode's height of grabwindow */
		/*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
		.mipi_data_lp2hs_settle_dc = 30,
		/*   following for GetDefaultFramerateByScenario()  */
		.max_framerate = 300,
	},
	.custom4 = {
		.pclk = 300000000,	/* record different mode's pclk */
		.linelength = 5536,	/* record different mode's linelength */
		.framelength = 1806,	/* record different mode's framelength */
		.startx = 0,	/* record different mode's startx of grabwindow */
		.starty = 0,	/* record different mode's starty of grabwindow */
		.grabwindow_width = 2328,	/* 2096,        //record different mode's width of grabwindow */
		.grabwindow_height = 1748,	/* 1552,       //record different mode's height of grabwindow */
		/*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
		.mipi_data_lp2hs_settle_dc = 30,
		/*   following for GetDefaultFramerateByScenario()  */
		.max_framerate = 300,
	},
	.custom5 = {
		.pclk = 300000000,	/* record different mode's pclk */
		.linelength = 5536,	/* record different mode's linelength */
		.framelength = 1806,	/* record different mode's framelength */
		.startx = 0,	/* record different mode's startx of grabwindow */
		.starty = 0,	/* record different mode's starty of grabwindow */
		.grabwindow_width = 2328,	/* record different mode's width of grabwindow */
		.grabwindow_height = 1748,	/* record different mode's height of grabwindow */
		/*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
		.mipi_data_lp2hs_settle_dc = 30,
		/*   following for GetDefaultFramerateByScenario()  */
		.max_framerate = 300,
	},
	.margin = 4,		/* sensor framelength & shutter margin */
	.min_shutter = 1,	/* min shutter */
	.max_frame_length = 0x7fff,	/* max framelength by sensor register's limitation */
	.ae_shut_delay_frame = 0,	/* shutter delay frame for AE cycle,
					 * 2 frame with ispGain_delay-shut_delay=2-0=2
					 */
	.ae_sensor_gain_delay_frame = 0,	/* sensor gain delay frame for AE cycle,
						 * 2 frame with ispGain_delay-sensor_gain_delay=2-0=2
						 */
	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.sensor_mode_num = 10,	/* support sensor mode num */

	.cap_delay_frame = 3,	/* enter capture delay frame num */
	.pre_delay_frame = 3,	/* enter preview delay frame num */
	.video_delay_frame = 3,	/* enter video delay frame num */
	.hs_video_delay_frame = 3,	/* enter high speed video  delay frame num */
	.slim_video_delay_frame = 3,	/* enter slim video delay frame num */

	.isp_driving_current = ISP_DRIVING_2MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,	/* sensor_interface_type */
	.mipi_sensor_type = MIPI_OPHY_NCSI2,	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,	/* 0,MIPI_SETTLEDELAY_AUTO;
								 * 1,MIPI_SETTLEDELAY_MANNUAL
								 */
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,	/* sensor output first pixel color */
	.mclk = 24,		/* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	.mipi_lane_num = SENSOR_MIPI_4_LANE,	/* mipi lane num */
	.i2c_addr_table = {0x34, 0x20, 0xff},	/* record sensor support all write id addr,
						 * only supprt 4must end with 0xff
						 */
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,	/* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT,	/* IMGSENSOR_MODE enum value,record current sensor mode,such as:
						 * INIT, Preview, Capture, Video,High Speed Video, Slim Video
						 */
	.shutter = 0x3D0,	/* current shutter */
	.gain = 0x100,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 300,	/* full size current fps : 24fps for PIP, 30fps for Normal or ZSD */
	.autoflicker_en = KAL_FALSE,	/* auto flicker enable: KAL_FALSE for disable auto flicker,
					 * KAL_TRUE for enable auto flicker
					 */
	.test_pattern = KAL_FALSE,	/* test pattern mode or not. KAL_FALSE for in test pattern mode,
					 * KAL_TRUE for normal output
					 */
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,	/* current scenario id */
	.ihdr_mode = 0,		/* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x34,	/* record current sensor's i2c write id */
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] = {
	{4656, 3496, 0000, 0000, 4656, 3496, 2328, 1748, 0000, 0000, 2328, 1748, 0000, 0000, 2328, 1748},/* Preview */
	{4656, 3496, 0000, 0000, 4656, 3496, 4656, 3496, 0000, 0000, 4656, 3496, 0000, 0000, 4656, 3496},/* capture */
	{4656, 3496, 0000, 0000, 4656, 3496, 4656, 3496, 0000, 0000, 4656, 3496, 0000, 0000, 4656, 3496},/* video */
	{4656, 3496, 0000, 648, 4656, 2199, 1922, 722, 0000, 0000, 1922, 722, 0000, 0000, 1920, 720},/* hs video */
	{4656, 3496, 0000, 492, 4656, 2500, 2216, 834, 0000, 0000, 2216, 834, 0000, 0000, 1476, 834},/* slim video */
	{4656, 3496, 0000, 0000, 4656, 3496, 2328, 1748, 0000, 0000, 2328, 1748, 0000, 0000, 2328, 1748},/* Custom1 */
	{4656, 3496, 0000, 0000, 4656, 3496, 2328, 1748, 0000, 0000, 2328, 1748, 0000, 0000, 2328, 1748},/* Custom2 */
	{4656, 3496, 0000, 0000, 4656, 3496, 2328, 1748, 0000, 0000, 2328, 1748, 0000, 0000, 2328, 1748},/* Custom3 */
	{4656, 3496, 0000, 0000, 4656, 3496, 2328, 1748, 0000, 0000, 2328, 1748, 0000, 0000, 2328, 1748},/* Custom4 */
	{4656, 3496, 0000, 0000, 4656, 3496, 2328, 1748, 0000, 0000, 2328, 1748, 0000, 0000, 2328, 1748},/* Custom5 */
};				/* slim video */

/*VC1 for HDR(DT=0X35) , VC2 for PDAF(DT=0X36), unit : 8bit */
/*pdaf winth=1grabwindow_width*10/8;pdaf support 8bit */
static SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3] = {
	/* Preview mode setting */
	{
		0x03, 0x0a, 0x00, 0x10, 0x40, 0x00,
		0x00, 0x2b, 0x0918, 0x06d4, 0x00, 0x35, 0x0280, 0x0001,
		0x00, 0x36, 0x0B58, 0x0001, 0x03, 0x00, 0x0000, 0x0000
	},
	/* Capture mode setting */
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
		0x00, 0x2b, 0x1230, 0x0DA8, 0x00, 0x35, 0x0280, 0x0001,
		0x00, 0x36, 0x16B8, 0x0001, 0x03, 0x00, 0x0000, 0x0000
	},
	/* Video mode setting */
	{
		0x02, 0x0a, 0x00, 0x08, 0x40, 0x00,
		0x00, 0x2b, 0x0918, 0x06d4, 0x01, 0x00, 0x0000, 0x0000,
		0x02, 0x00, 0x0000, 0x0000, 0x03, 0x00, 0x0000, 0x0000
	}
};

static SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 88,
	.i4OffsetY = 72,
	.i4PitchX = 32,
	.i4PitchY = 32,
	.i4PairNum = 16,
	.i4SubBlkW = 8,
	.i4SubBlkH = 8,
	.i4PosL = {
		{91, 73}, {99, 73}, {107, 73}, {115, 73}, {95, 81}, {103, 81}, {111, 81}, {119, 81},
		{91, 89},
		{99, 89}, {107, 89}, {115, 89}, {95, 97}, {103, 97}, {111, 97}, {119, 97} },
	.i4PosR = {
		{90, 73}, {98, 73}, {106, 73}, {114, 73}, {94, 81}, {102, 81}, {110, 81}, {118, 81},
		{90, 89},
		{98, 89}, {106, 89}, {114, 89}, {94, 97}, {102, 97}, {110, 97}, {118, 97} },
	.iMirrorFlip = 0,	/* 0:IMAGE_NORMAL,1:IMAGE_H_MIRROR,2:IMAGE_V_MIRROR,3:IMAGE_HV_MIRROR */
};


typedef struct {
	MUINT16 DarkLimit_H;
	MUINT16 DarkLimit_L;
	MUINT16 OverExp_Min_H;
	MUINT16 OverExp_Min_L;
	MUINT16 OverExp_Max_H;
	MUINT16 OverExp_Max_L;
} SENSOR_ATR_INFO, *pSENSOR_ATR_INFO;
#if 0
static SENSOR_ATR_INFO sensorATR_Info[4] = {	/* Strength Range Min */
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	/* Strength Range Std */
	{0x00, 0x32, 0x00, 0x3c, 0x03, 0xff},
	/* Strength Range Max */
	{0x3f, 0xff, 0x3f, 0xff, 0x3f, 0xff},
	/* Strength Range Custom */
	{0x3F, 0xFF, 0x00, 0x0, 0x3F, 0xFF}
};
#endif

#define IMX398MIPI_MaxGainIndex (223)
kal_uint16 IMX398MIPI_sensorGainMapping[IMX398MIPI_MaxGainIndex][2] = {
	{64, 0},
	{65, 6},
	{66, 12},
	{67, 20},
	{68, 27},
	{69, 34},
	{70, 43},
	{71, 51},
	{72, 55},
	{73, 63},
	{74, 67},
	{75, 75},
	{76, 79},
	{77, 85},
	{78, 92},
	{79, 96},
	{80, 100},
	{81, 106},
	{82, 112},
	{83, 116},
	{84, 122},
	{85, 125},
	{86, 130},
	{87, 136},
	{88, 139},
	{89, 144},
	{90, 146},
	{91, 152},
	{92, 154},
	{93, 159},
	{94, 162},
	{95, 167},
	{96, 169},
	{97, 173},
	{98, 176},
	{100, 184},
	{101, 186},
	{102, 190},
	{103, 193},
	{104, 196},
	{105, 200},
	{106, 202},
	{107, 206},
	{108, 208},
	{110, 213},
	{111, 216},
	{112, 220},
	{113, 221},
	{114, 224},
	{115, 226},
	{116, 230},
	{117, 231},
	{118, 234},
	{120, 239},
	{121, 242},
	{122, 243},
	{123, 246},
	{124, 247},
	{125, 249},
	{126, 251},
	{127, 253},
	{128, 255},
	{130, 259},
	{131, 261},
	{132, 263},
	{133, 265},
	{134, 267},
	{135, 269},
	{136, 271},
	{137, 272},
	{138, 274},
	{140, 278},
	{141, 279},
	{142, 281},
	{143, 283},
	{144, 284},
	{145, 286},
	{146, 287},
	{147, 289},
	{148, 290},
	{150, 293},
	{151, 295},
	{152, 296},
	{153, 298},
	{154, 299},
	{155, 300},
	{156, 302},
	{157, 303},
	{158, 304},
	{160, 307},
	{161, 308},
	{162, 310},
	{163, 311},
	{164, 312},
	{165, 313},
	{166, 315},
	{167, 316},
	{168, 317},
	{170, 319},
	{171, 320},
	{172, 321},
	{173, 323},
	{174, 324},
	{175, 325},
	{176, 326},
	{177, 327},
	{178, 328},
	{180, 330},
	{181, 331},
	{182, 332},
	{183, 333},
	{184, 334},
	{185, 335},
	{186, 336},
	{187, 337},
	{188, 338},
	{191, 340},
	{192, 341},
	{193, 342},
	{194, 343},
	{195, 344},
	{196, 345},
	{197, 346},
	{199, 347},
	{200, 348},
	{202, 350},
	{204, 351},
	{205, 352},
	{206, 353},
	{207, 354},
	{209, 355},
	{210, 356},
	{211, 357},
	{213, 358},
	{216, 360},
	{217, 361},
	{218, 362},
	{220, 363},
	{221, 364},
	{223, 365},
	{224, 366},
	{226, 367},
	{228, 368},
	{229, 369},
	{231, 370},
	{232, 371},
	{234, 372},
	{236, 373},
	{237, 374},
	{239, 375},
	{241, 376},
	{243, 377},
	{245, 378},
	{246, 379},
	{248, 380},
	{250, 381},
	{252, 382},
	{254, 383},
	{256, 384},
	{258, 385},
	{260, 386},
	{262, 387},
	{264, 388},
	{266, 389},
	{269, 390},
	{271, 391},
	{273, 392},
	{275, 393},
	{278, 394},
	{280, 395},
	{282, 396},
	{285, 397},
	{287, 398},
	{290, 399},
	{293, 400},
	{295, 401},
	{298, 402},
	{301, 403},
	{303, 404},
	{306, 405},
	{309, 406},
	{312, 407},
	{315, 408},
	{318, 409},
	{321, 410},
	{324, 411},
	{328, 412},
	{331, 413},
	{334, 414},
	{338, 415},
	{341, 416},
	{345, 417},
	{349, 418},
	{352, 419},
	{356, 420},
	{360, 421},
	{364, 422},
	{368, 423},
	{372, 424},
	{377, 425},
	{381, 426},
	{386, 427},
	{390, 428},
	{395, 429},
	{400, 430},
	{405, 431},
	{410, 432},
	{415, 433},
	{420, 434},
	{426, 435},
	{431, 436},
	{437, 437},
	{443, 438},
	{449, 439},
	{455, 440},
	{462, 441},
	{468, 442},
	{475, 443},
	{482, 444},
	{489, 445},
	{496, 446},
	{504, 447},
	{512, 448},
};


static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;

	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *) &get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = { (char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF) };

	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

#if 0
static kal_uint32 imx398_ATR(UINT16 DarkLimit, UINT16 OverExp)
{
	/*
	 * write_cmos_sensor(0x6e50,sensorATR_Info[DarkLimit].DarkLimit_H);
	 * write_cmos_sensor(0x6e51,sensorATR_Info[DarkLimit].DarkLimit_L);
	 * write_cmos_sensor(0x9340,sensorATR_Info[OverExp].OverExp_Min_H);
	 * write_cmos_sensor(03x941,sensorATR_Info[OverExp].OverExp_Min_L);
	 * write_cmos_sensor(0x9342,sensorATR_Info[OverExp].OverExp_Max_H);
	 * write_cmos_sensor(0x9343,sensorATR_Info[OverExp].OverExp_Max_L);
	 * write_cmos_sensor(0x9706,0x10);
	 * write_cmos_sensor(0x9707,0x03);
	 * write_cmos_sensor(0x9708,0x03);
	 * write_cmos_sensor(0x9e24,0x00);
	 * write_cmos_sensor(0x9e25,0x8c);
	 * write_cmos_sensor(0x9e26,0x00);
	 * write_cmos_sensor(0x9e27,0x94);
	 * write_cmos_sensor(0x9e28,0x00);
	 * write_cmos_sensor(0x9e29,0x96);
	 * LOG_INF("DarkLimit 0x6e50(0x%x), 0x6e51(0x%x)\n",sensorATR_Info[DarkLimit].DarkLimit_H,
	 * sensorATR_Info[DarkLimit].DarkLimit_L);
	 * LOG_INF("OverExpMin 0x9340(0x%x), 0x9341(0x%x)\n",sensorATR_Info[OverExp].OverExp_Min_H,
	 * sensorATR_Info[OverExp].OverExp_Min_L);
	 * LOG_INF("OverExpMin 0x9342(0x%x), 0x9343(0x%x)\n",sensorATR_Info[OverExp].OverExp_Max_H,
	 * sensorATR_Info[OverExp].OverExp_Max_L);
	 */
	return ERROR_NONE;
}
#endif

static void imx398_apply_SPC(void)
{
	unsigned int start_reg = 0x7E00;
	int i;

	if (read_spc_flag == FALSE) {
		read_imx398_SPC(imx398_SPC_data);
		read_spc_flag = TRUE;
		return;
	}

	for (i = 0; i < 126; i++) {
		write_cmos_sensor(start_reg, imx398_SPC_data[i]);
		LOG_INF("SPC[%d]= %x\n", i, imx398_SPC_data[i]);
		start_reg++;
	}
	start_reg = 0x7f00;
	for (i = 0; i < 126; i++) {
		write_cmos_sensor(start_reg, imx398_SPC_data[i]);
		LOG_INF("SPC[%d]= %x\n", i, imx398_SPC_data[i]);

		start_reg++;
	}

}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/*
	 * you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel,
	 * or you can set dummy by imgsensor.frame_length and imgsensor.line_length
	 */
	write_cmos_sensor(0x0104, 0x01);

	write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
	write_cmos_sensor(0x0342, imgsensor.line_length >> 8);
	write_cmos_sensor(0x0343, imgsensor.line_length & 0xFF);

	write_cmos_sensor(0x0104, 0x00);
}				/*    set_dummy  */

static kal_uint32 return_sensor_id(void)
{
	kal_uint8 tmph, tmpl;

	LOG_INF("Enter\n");
	write_cmos_sensor(0x0A02, 0x1F);
	write_cmos_sensor(0x0A00, 0x01);
	mdelay(10);
	tmph = 0x00;
	tmpl = 0x00;
	tmph = read_cmos_sensor(0x0A29);
	tmpl = read_cmos_sensor(0x0A2A) >> 4;
	LOG_INF("[ylf]tmph = %d, tmpl= %d\n", tmph, tmpl);
	return ((read_cmos_sensor(0x0A29) << 4) | (read_cmos_sensor(0x0A2A) >> 4));
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;
	/* unsigned long flags; */

	LOG_INF("framerate = %d, min framelength should enable %d\n", framerate, min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length)
		? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	/* dummy_line = frame_length - imgsensor.min_frame_length; */
	/* if (dummy_line < 0) */
	/* imgsensor.dummy_line = 0; */
	/* else */
	/* imgsensor.dummy_line = dummy_line; */
	/* imgsensor.frame_length = frame_length + imgsensor.dummy_line; */
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}				/*    set_max_framerate  */



/*************************************************************************
 * FUNCTION
 *    set_shutter
 *
 * DESCRIPTION
 *    This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *    iShutter : exposured lines
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/* write_shutter(shutter); */
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

	/* OV Recommend Solution */
	/* if shutter bigger than frame_length, should extend frame length first */
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor(0x0104, 0x01);
			write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
			write_cmos_sensor(0x0104, 0x00);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor(0x0104, 0x01);
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x0104, 0x00);
	}

	/* Update Shutter */
	write_cmos_sensor(0x0104, 0x01);
	write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor(0x0203, shutter & 0xFF);
	write_cmos_sensor(0x0104, 0x00);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);

}				/*    set_shutter */



static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint8 iI;

	LOG_INF("[IMX398MIPI]enter IMX398MIPIGain2Reg function\n");
	for (iI = 0; iI < IMX398MIPI_MaxGainIndex; iI++) {
		if (gain <= IMX398MIPI_sensorGainMapping[iI][0])
			return IMX398MIPI_sensorGainMapping[iI][1];
	}

	LOG_INF("exit IMX398MIPIGain2Reg function\n");
	return IMX398MIPI_sensorGainMapping[iI - 1][1];
}

/*************************************************************************
 * FUNCTION
 *    set_gain
 *
 * DESCRIPTION
 *    This function is to set global gain to sensor.
 *
 * PARAMETERS
 *    iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *    the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	/* 0x350A[0:1], 0x350B[0:7] AGC real gain */
	/* [0:3] = N meams N /16 X    */
	/* [4:9] = M meams M X         */
	/* Total gain = M + N /16 X   */

	/*  */
	if (gain < BASEGAIN || gain > 8 * BASEGAIN) {
		LOG_INF("Error gain setting");

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 8 * BASEGAIN)
			gain = 8 * BASEGAIN;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor(0x0104, 0x01);
	write_cmos_sensor(0x0204, (reg_gain >> 8) & 0xFF);
	write_cmos_sensor(0x0205, reg_gain & 0xFF);
	write_cmos_sensor(0x0104, 0x00);

	return gain;
}				/*    set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{

	kal_uint16 realtime_fps = 0;
	kal_uint16 reg_gain;

	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n", le, se, gain);
	spin_lock(&imgsensor_drv_lock);
	if (le > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = le + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	if (le < imgsensor_info.min_shutter)
		le = imgsensor_info.min_shutter;
	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			write_cmos_sensor(0x0104, 0x01);
			write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
			write_cmos_sensor(0x0104, 0x00);
		}
	} else {
		write_cmos_sensor(0x0104, 0x01);
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x0104, 0x00);
	}
	write_cmos_sensor(0x0104, 0x01);
	/* Long exposure */
	write_cmos_sensor(0x0202, (le >> 8) & 0xFF);
	write_cmos_sensor(0x0203, le & 0xFF);
	/* Short exposure */
	write_cmos_sensor(0x0224, (se >> 8) & 0xFF);
	write_cmos_sensor(0x0225, se & 0xFF);
	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	/* Global analog Gain for Long expo */
	write_cmos_sensor(0x0204, (reg_gain >> 8) & 0xFF);
	write_cmos_sensor(0x0205, reg_gain & 0xFF);
	/* Global analog Gain for Short expo */
	write_cmos_sensor(0x0216, (reg_gain >> 8) & 0xFF);
	write_cmos_sensor(0x0217, reg_gain & 0xFF);
	write_cmos_sensor(0x0104, 0x00);

}

#if 0
static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	/********************************************************
	 *
	 *   0x3820[2] ISP Vertical flip
	 *   0x3820[1] Sensor Vertical flip
	 *
	 *   0x3821[2] ISP Horizontal mirror
	 *   0x3821[1] Sensor Horizontal mirror
	 *
	 *   ISP and Sensor flip or mirror register bit should be the same!!
	 *
	 ********************************************************/

	switch (image_mirror) {
	case IMAGE_NORMAL:
		write_cmos_sensor(0x0101, 0x00);
		write_cmos_sensor(0x3A27, 0x00);
		write_cmos_sensor(0x3A28, 0x00);
		write_cmos_sensor(0x3A29, 0x01);
		write_cmos_sensor(0x3A2A, 0x00);
		write_cmos_sensor(0x3A2B, 0x00);
		write_cmos_sensor(0x3A2C, 0x00);
		write_cmos_sensor(0x3A2D, 0x01);
		write_cmos_sensor(0x3A2E, 0x01);
		break;
	case IMAGE_H_MIRROR:
		write_cmos_sensor(0x0101, 0x01);
		write_cmos_sensor(0x3A27, 0x01);
		write_cmos_sensor(0x3A28, 0x01);
		write_cmos_sensor(0x3A29, 0x00);
		write_cmos_sensor(0x3A2A, 0x00);
		write_cmos_sensor(0x3A2B, 0x01);
		write_cmos_sensor(0x3A2C, 0x00);
		write_cmos_sensor(0x3A2D, 0x00);
		write_cmos_sensor(0x3A2E, 0x01);
		break;
	case IMAGE_V_MIRROR:
		write_cmos_sensor(0x0101, 0x02);
		write_cmos_sensor(0x3A27, 0x10);
		write_cmos_sensor(0x3A28, 0x10);
		write_cmos_sensor(0x3A29, 0x01);
		write_cmos_sensor(0x3A2A, 0x01);
		write_cmos_sensor(0x3A2B, 0x00);
		write_cmos_sensor(0x3A2C, 0x01);
		write_cmos_sensor(0x3A2D, 0x01);
		write_cmos_sensor(0x3A2E, 0x00);
		break;
	case IMAGE_HV_MIRROR:
		write_cmos_sensor(0x0101, 0x03);
		write_cmos_sensor(0x3A27, 0x11);
		write_cmos_sensor(0x3A28, 0x11);
		write_cmos_sensor(0x3A29, 0x00);
		write_cmos_sensor(0x3A2A, 0x01);
		write_cmos_sensor(0x3A2B, 0x01);
		write_cmos_sensor(0x3A2C, 0x01);
		write_cmos_sensor(0x3A2D, 0x00);
		write_cmos_sensor(0x3A2E, 0x00);
		break;
	default:
		LOG_INF("Error image_mirror setting\n");
	}

}
#endif
/*************************************************************************
 * FUNCTION
 *    night_mode
 *
 * DESCRIPTION
 *    This function night mode of sensor.
 *
 * PARAMETERS
 *    bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void night_mode(kal_bool enable)
{
	/*No Need to implement this function*/
}				/*    night_mode    */

static void sensor_init(void)
{
	LOG_INF("E\n");
	/* external clock setting */
	write_cmos_sensor(0x0100, 0x00);
	write_cmos_sensor(0x0136, 0x18);
	write_cmos_sensor(0x0137, 0x00);
	/* Global setting */
	write_cmos_sensor(0x307B, 0x01);
	write_cmos_sensor(0x307C, 0x01);
	write_cmos_sensor(0x307D, 0x01);
	write_cmos_sensor(0x307E, 0x01);
	write_cmos_sensor(0x30F4, 0x01);
	write_cmos_sensor(0x30F5, 0x7A);
	write_cmos_sensor(0x30F6, 0x00);
	write_cmos_sensor(0x30F7, 0xEC);
	write_cmos_sensor(0x30FC, 0x01);
	write_cmos_sensor(0x3101, 0x01);
	write_cmos_sensor(0x3103, 0x00);
	write_cmos_sensor(0x5B2F, 0x08);
	write_cmos_sensor(0x5D32, 0x05);
	write_cmos_sensor(0x5D7C, 0x00);
	write_cmos_sensor(0x5D7D, 0x00);
	write_cmos_sensor(0x5DB9, 0x01);
	write_cmos_sensor(0x5E43, 0x00);
	write_cmos_sensor(0x6300, 0x00);
	write_cmos_sensor(0x6301, 0xEA);
	write_cmos_sensor(0x6302, 0x00);
	write_cmos_sensor(0x6303, 0xB4);
	write_cmos_sensor(0x6564, 0x00);
	write_cmos_sensor(0x6565, 0xB6);
	write_cmos_sensor(0x6566, 0x00);
	write_cmos_sensor(0x6567, 0xE6);
	write_cmos_sensor(0x6714, 0x01);
	write_cmos_sensor(0x6758, 0x0B);
	write_cmos_sensor(0x6910, 0x04);
	write_cmos_sensor(0x6916, 0x01);
	write_cmos_sensor(0x6918, 0x04);
	write_cmos_sensor(0x691E, 0x01);
	write_cmos_sensor(0x6931, 0x01);
	write_cmos_sensor(0x6937, 0x02);
	write_cmos_sensor(0x693B, 0x02);
	write_cmos_sensor(0x6D00, 0x4A);
	write_cmos_sensor(0x6D01, 0x41);
	write_cmos_sensor(0x6D02, 0x23);
	write_cmos_sensor(0x6D05, 0x4C);
	write_cmos_sensor(0x6D06, 0x10);
	write_cmos_sensor(0x6D08, 0x30);
	write_cmos_sensor(0x6D09, 0x38);
	write_cmos_sensor(0x6D0A, 0x2C);
	write_cmos_sensor(0x6D0B, 0x2D);
	write_cmos_sensor(0x6D0C, 0x34);
	write_cmos_sensor(0x6D0D, 0x42);
	write_cmos_sensor(0x6D19, 0x1C);
	write_cmos_sensor(0x6D1A, 0x71);
	write_cmos_sensor(0x6D1B, 0xC6);
	write_cmos_sensor(0x6D1C, 0x94);
	write_cmos_sensor(0x6D24, 0xE4);
	write_cmos_sensor(0x6D30, 0x0A);
	write_cmos_sensor(0x6D31, 0x01);
	write_cmos_sensor(0x6D33, 0x0B);
	write_cmos_sensor(0x6D34, 0x05);
	write_cmos_sensor(0x6D35, 0x00);
	write_cmos_sensor(0x83C2, 0x03);
	write_cmos_sensor(0x83c3, 0x08);
	write_cmos_sensor(0x83C4, 0x48);
	write_cmos_sensor(0x83C7, 0x08);
	write_cmos_sensor(0x83CB, 0x00);
	write_cmos_sensor(0x927C, 0x00);
	write_cmos_sensor(0x927D, 0x00);
	write_cmos_sensor(0x927E, 0x00);
	write_cmos_sensor(0x927F, 0x00);
	write_cmos_sensor(0xB101, 0xFF);
	write_cmos_sensor(0xB103, 0xFF);
	write_cmos_sensor(0xB105, 0xFF);
	write_cmos_sensor(0xB107, 0xFF);
	write_cmos_sensor(0xB109, 0xFF);
	write_cmos_sensor(0xB10B, 0xFF);
	write_cmos_sensor(0xB10D, 0xFF);
	write_cmos_sensor(0xB10F, 0xFF);
	write_cmos_sensor(0xB111, 0xFF);
	write_cmos_sensor(0xB163, 0x3C);
	write_cmos_sensor(0xC2A0, 0x08);
	write_cmos_sensor(0xC2A3, 0x03);
	write_cmos_sensor(0xC2A5, 0x08);
	write_cmos_sensor(0xC2A6, 0x48);
	write_cmos_sensor(0xC2A9, 0x00);
	write_cmos_sensor(0xF800, 0x5E);
	write_cmos_sensor(0xF801, 0x5E);
	write_cmos_sensor(0xF802, 0xCD);
	write_cmos_sensor(0xF803, 0x20);
	write_cmos_sensor(0xF804, 0x55);
	write_cmos_sensor(0xF805, 0xD4);
	write_cmos_sensor(0xF806, 0x1F);
	write_cmos_sensor(0xF808, 0xF8);
	write_cmos_sensor(0xF809, 0x3A);
	write_cmos_sensor(0xF80A, 0xF1);
	write_cmos_sensor(0xF80B, 0x7E);
	write_cmos_sensor(0xF80C, 0x55);
	write_cmos_sensor(0xF80D, 0x38);
	write_cmos_sensor(0xF80E, 0xE3);
	write_cmos_sensor(0xF810, 0x74);
	write_cmos_sensor(0xF811, 0x41);
	write_cmos_sensor(0xF812, 0xBF);
	write_cmos_sensor(0xF844, 0x40);
	write_cmos_sensor(0xF845, 0xBA);
	write_cmos_sensor(0xF846, 0x70);
	write_cmos_sensor(0xF847, 0x47);
	write_cmos_sensor(0xF848, 0xC0);
	write_cmos_sensor(0xF849, 0xBA);
	write_cmos_sensor(0xF84A, 0x70);
	write_cmos_sensor(0xF84B, 0x47);
	write_cmos_sensor(0xF84C, 0x82);
	write_cmos_sensor(0xF84D, 0xF6);
	write_cmos_sensor(0xF84E, 0x32);
	write_cmos_sensor(0xF84F, 0xFD);
	write_cmos_sensor(0xF851, 0xF0);
	write_cmos_sensor(0xF852, 0x02);
	write_cmos_sensor(0xF853, 0xF8);
	write_cmos_sensor(0xF854, 0x81);
	write_cmos_sensor(0xF855, 0xF6);
	write_cmos_sensor(0xF856, 0xC0);
	write_cmos_sensor(0xF857, 0xFF);
	write_cmos_sensor(0xF858, 0x10);
	write_cmos_sensor(0xF859, 0xB5);
	write_cmos_sensor(0xF85A, 0x0D);
	write_cmos_sensor(0xF85B, 0x48);
	write_cmos_sensor(0xF85C, 0x40);
	write_cmos_sensor(0xF85D, 0x7A);
	write_cmos_sensor(0xF85E, 0x01);
	write_cmos_sensor(0xF85F, 0x28);
	write_cmos_sensor(0xF860, 0x15);
	write_cmos_sensor(0xF861, 0xD1);
	write_cmos_sensor(0xF862, 0x0C);
	write_cmos_sensor(0xF863, 0x49);
	write_cmos_sensor(0xF864, 0x0C);
	write_cmos_sensor(0xF865, 0x46);
	write_cmos_sensor(0xF866, 0x40);
	write_cmos_sensor(0xF867, 0x3C);
	write_cmos_sensor(0xF868, 0x48);
	write_cmos_sensor(0xF869, 0x8A);
	write_cmos_sensor(0xF86A, 0x62);
	write_cmos_sensor(0xF86B, 0x8A);
	write_cmos_sensor(0xF86C, 0x80);
	write_cmos_sensor(0xF86D, 0x1A);
	write_cmos_sensor(0xF86E, 0x8A);
	write_cmos_sensor(0xF86F, 0x89);
	write_cmos_sensor(0xF871, 0xB2);
	write_cmos_sensor(0xF872, 0x10);
	write_cmos_sensor(0xF873, 0x18);
	write_cmos_sensor(0xF874, 0x0A);
	write_cmos_sensor(0xF875, 0x46);
	write_cmos_sensor(0xF876, 0x20);
	write_cmos_sensor(0xF877, 0x32);
	write_cmos_sensor(0xF878, 0x12);
	write_cmos_sensor(0xF879, 0x88);
	write_cmos_sensor(0xF87A, 0x90);
	write_cmos_sensor(0xF87B, 0x42);
	write_cmos_sensor(0xF87D, 0xDA);
	write_cmos_sensor(0xF87E, 0x10);
	write_cmos_sensor(0xF87F, 0x46);
	write_cmos_sensor(0xF880, 0x80);
	write_cmos_sensor(0xF881, 0xB2);
	write_cmos_sensor(0xF882, 0x88);
	write_cmos_sensor(0xF883, 0x81);
	write_cmos_sensor(0xF884, 0x84);
	write_cmos_sensor(0xF885, 0xF6);
	write_cmos_sensor(0xF886, 0xD2);
	write_cmos_sensor(0xF887, 0xF9);
	write_cmos_sensor(0xF888, 0xE0);
	write_cmos_sensor(0xF889, 0x67);
	write_cmos_sensor(0xF88A, 0x85);
	write_cmos_sensor(0xF88B, 0xF6);
	write_cmos_sensor(0xF88C, 0xA1);
	write_cmos_sensor(0xF88D, 0xFC);
	write_cmos_sensor(0xF88E, 0x10);
	write_cmos_sensor(0xF88F, 0xBD);
	write_cmos_sensor(0xF891, 0x18);
	write_cmos_sensor(0xF892, 0x21);
	write_cmos_sensor(0xF893, 0x24);
	write_cmos_sensor(0xF895, 0x18);
	write_cmos_sensor(0xF896, 0x19);
	write_cmos_sensor(0xF897, 0xB4);

	/* load setting */
	write_cmos_sensor(0x4E29, 0x01);

	/* image quality */
	write_cmos_sensor(0x3013, 0x07);
	write_cmos_sensor(0x3035, 0x01);
	write_cmos_sensor(0x3051, 0x00);
	write_cmos_sensor(0x3056, 0x02);
	write_cmos_sensor(0x3057, 0x01);
	write_cmos_sensor(0x3060, 0x00);
	write_cmos_sensor(0x310B, 0x48);
	write_cmos_sensor(0x310E, 0x02);
	write_cmos_sensor(0x310F, 0x2A);
	write_cmos_sensor(0x3150, 0x0F);
	write_cmos_sensor(0x3158, 0x04);
	write_cmos_sensor(0x3159, 0x03);
	write_cmos_sensor(0x315A, 0x02);
	write_cmos_sensor(0x315B, 0x01);
	write_cmos_sensor(0x3160, 0x18);
	write_cmos_sensor(0x8349, 0x00);
	write_cmos_sensor(0x8435, 0x00);
	write_cmos_sensor(0x8455, 0x00);
	write_cmos_sensor(0x847C, 0x00);
	write_cmos_sensor(0x84FB, 0x01);
	write_cmos_sensor(0x8865, 0x8C);
	write_cmos_sensor(0x8867, 0x4B);
	write_cmos_sensor(0x8869, 0x8C);
	write_cmos_sensor(0x886B, 0x4B);
	write_cmos_sensor(0x886D, 0x8C);
	write_cmos_sensor(0x886F, 0x4B);
	write_cmos_sensor(0x8870, 0x40);
	write_cmos_sensor(0x90A8, 0x1A);
	write_cmos_sensor(0x90A9, 0x04);
	write_cmos_sensor(0x90AA, 0x20);
	write_cmos_sensor(0x90AB, 0x20);
	write_cmos_sensor(0x90AC, 0x13);
	write_cmos_sensor(0x90AD, 0x13);
	write_cmos_sensor(0x9236, 0x04);
	write_cmos_sensor(0x9257, 0x96);
	write_cmos_sensor(0x9258, 0x03);
	write_cmos_sensor(0x9308, 0xAA);
	write_cmos_sensor(0x933A, 0x02);
	write_cmos_sensor(0x933B, 0x02);
	write_cmos_sensor(0x933D, 0x05);
	write_cmos_sensor(0x933E, 0x05);
	write_cmos_sensor(0x933F, 0x05);
	write_cmos_sensor(0x934B, 0x1B);
	write_cmos_sensor(0x934C, 0x0A);
	write_cmos_sensor(0x9356, 0x8C);
	write_cmos_sensor(0x9357, 0x50);
	write_cmos_sensor(0x9358, 0x1B);
	write_cmos_sensor(0x9359, 0x8C);
	write_cmos_sensor(0x935A, 0x1B);
	write_cmos_sensor(0x935B, 0x0A);
	write_cmos_sensor(0x9360, 0x1B);
	write_cmos_sensor(0x9361, 0x0A);
	write_cmos_sensor(0x9362, 0x8C);
	write_cmos_sensor(0x9363, 0x50);
	write_cmos_sensor(0x9364, 0x1B);
	write_cmos_sensor(0x9365, 0x8C);
	write_cmos_sensor(0x9366, 0x1B);
	write_cmos_sensor(0x9367, 0x0A);
	write_cmos_sensor(0x940D, 0x07);
	write_cmos_sensor(0x940E, 0x07);
	write_cmos_sensor(0x9414, 0x06);
	write_cmos_sensor(0x942F, 0x09);
	write_cmos_sensor(0x9430, 0x09);
	write_cmos_sensor(0x9431, 0x09);
	write_cmos_sensor(0x9432, 0x09);
	write_cmos_sensor(0x9433, 0x09);
	write_cmos_sensor(0x9434, 0x09);
	write_cmos_sensor(0x9435, 0x09);
	write_cmos_sensor(0x9436, 0x09);
	write_cmos_sensor(0x9437, 0x09);
	write_cmos_sensor(0x9438, 0x09);
	write_cmos_sensor(0x9439, 0x09);
	write_cmos_sensor(0x943A, 0x09);
	write_cmos_sensor(0x943B, 0x09);
	write_cmos_sensor(0x943C, 0x0B);
	write_cmos_sensor(0x943D, 0x0B);
	write_cmos_sensor(0x943F, 0x09);
	write_cmos_sensor(0x9441, 0x09);
	write_cmos_sensor(0x9443, 0x09);
	write_cmos_sensor(0x9445, 0x09);
	write_cmos_sensor(0x9447, 0x09);
	write_cmos_sensor(0x9449, 0x09);
	write_cmos_sensor(0x944B, 0x09);
	write_cmos_sensor(0x944D, 0x09);
	write_cmos_sensor(0x944F, 0x09);
	write_cmos_sensor(0x9451, 0x09);
	write_cmos_sensor(0x9453, 0x09);
	write_cmos_sensor(0x9455, 0x09);
	write_cmos_sensor(0x9456, 0x0A);
	write_cmos_sensor(0x9458, 0x09);
	write_cmos_sensor(0x945A, 0x09);
	write_cmos_sensor(0x945B, 0x09);
	write_cmos_sensor(0x945C, 0x09);
	write_cmos_sensor(0x945D, 0x09);
	write_cmos_sensor(0x945E, 0x09);
	write_cmos_sensor(0x945F, 0x07);
	write_cmos_sensor(0x9460, 0x0B);
	write_cmos_sensor(0x9461, 0x07);
	write_cmos_sensor(0x9462, 0x0B);
	write_cmos_sensor(0x9463, 0x09);
	write_cmos_sensor(0x9464, 0x09);
	write_cmos_sensor(0x9465, 0x09);
	write_cmos_sensor(0x9466, 0x09);
	write_cmos_sensor(0x9467, 0x09);
	write_cmos_sensor(0x9468, 0x09);
	write_cmos_sensor(0x9469, 0x09);
	write_cmos_sensor(0x946A, 0x09);
	write_cmos_sensor(0x9473, 0x08);
	write_cmos_sensor(0x9474, 0x08);
	write_cmos_sensor(0x9475, 0x08);
	write_cmos_sensor(0x9476, 0x08);
	write_cmos_sensor(0x9477, 0x08);
	write_cmos_sensor(0x9478, 0x08);
	write_cmos_sensor(0x9479, 0x08);
	write_cmos_sensor(0x947A, 0x08);
	write_cmos_sensor(0x947B, 0x08);
	write_cmos_sensor(0x9480, 0x01);
	write_cmos_sensor(0x9481, 0x01);
	write_cmos_sensor(0x9484, 0x01);
	write_cmos_sensor(0x9485, 0x01);
	write_cmos_sensor(0x9503, 0x07);
	write_cmos_sensor(0x9504, 0x07);
	write_cmos_sensor(0x9505, 0x07);
	write_cmos_sensor(0x9506, 0x00);
	write_cmos_sensor(0x9507, 0x00);
	write_cmos_sensor(0x9508, 0x00);
	write_cmos_sensor(0x950B, 0x0A);
	write_cmos_sensor(0x950D, 0x0A);
	write_cmos_sensor(0x950F, 0x0A);
	write_cmos_sensor(0x9526, 0x18);
	write_cmos_sensor(0x9527, 0x18);
	write_cmos_sensor(0x9528, 0x18);
	write_cmos_sensor(0x9619, 0xA0);
	write_cmos_sensor(0x961B, 0xA0);
	write_cmos_sensor(0x961D, 0xA0);
	write_cmos_sensor(0x961F, 0x20);
	write_cmos_sensor(0x9621, 0x20);
	write_cmos_sensor(0x9623, 0x20);
	write_cmos_sensor(0x9625, 0xA0);
	write_cmos_sensor(0x9627, 0xA0);
	write_cmos_sensor(0x9629, 0xA0);
	write_cmos_sensor(0x962B, 0x20);
	write_cmos_sensor(0x962D, 0x20);
	write_cmos_sensor(0x962F, 0x20);
	write_cmos_sensor(0x970D, 0x2C);
	write_cmos_sensor(0x9711, 0xC8);
	write_cmos_sensor(0x9713, 0xAA);
	write_cmos_sensor(0x9715, 0x72);
	write_cmos_sensor(0x9717, 0x64);
	write_cmos_sensor(0x9719, 0xA0);
	write_cmos_sensor(0x971B, 0xA0);
	write_cmos_sensor(0x971D, 0xA0);
	write_cmos_sensor(0x971F, 0x20);
	write_cmos_sensor(0x9721, 0x20);
	write_cmos_sensor(0x9723, 0x20);
	write_cmos_sensor(0x9725, 0xA0);
	write_cmos_sensor(0x9727, 0xA0);
	write_cmos_sensor(0x9729, 0xA0);
	write_cmos_sensor(0x972B, 0x20);
	write_cmos_sensor(0x972D, 0x20);
	write_cmos_sensor(0x972F, 0x20);
	write_cmos_sensor(0x9795, 0x0A);
	write_cmos_sensor(0x9797, 0x02);
	write_cmos_sensor(0x9799, 0x00);
	write_cmos_sensor(0x979D, 0x0E);
	write_cmos_sensor(0x979F, 0x0A);
	write_cmos_sensor(0x97A0, 0x20);
	write_cmos_sensor(0x97A1, 0x13);
	write_cmos_sensor(0x97A2, 0x10);
	write_cmos_sensor(0x97A3, 0x20);
	write_cmos_sensor(0x97A4, 0x13);
	write_cmos_sensor(0x97A5, 0x10);
	write_cmos_sensor(0x9901, 0x35);
	write_cmos_sensor(0x9903, 0x23);
	write_cmos_sensor(0x9905, 0x23);
	write_cmos_sensor(0x9906, 0x00);
	write_cmos_sensor(0x9907, 0x31);
	write_cmos_sensor(0x9908, 0x00);
	write_cmos_sensor(0x9909, 0x1B);
	write_cmos_sensor(0x990A, 0x00);
	write_cmos_sensor(0x990B, 0x15);
	write_cmos_sensor(0x990D, 0x3F);
	write_cmos_sensor(0x990F, 0x3F);
	write_cmos_sensor(0x9911, 0x3F);
	write_cmos_sensor(0x9913, 0x64);
	write_cmos_sensor(0x9915, 0x64);
	write_cmos_sensor(0x9917, 0x64);
	write_cmos_sensor(0x9919, 0x50);
	write_cmos_sensor(0x991B, 0x60);
	write_cmos_sensor(0x991D, 0x65);
	write_cmos_sensor(0x991F, 0x01);
	write_cmos_sensor(0x9921, 0x01);
	write_cmos_sensor(0x9923, 0x01);
	write_cmos_sensor(0x9925, 0x23);
	write_cmos_sensor(0x9927, 0x23);
	write_cmos_sensor(0x9929, 0x23);
	write_cmos_sensor(0x992B, 0x2F);
	write_cmos_sensor(0x992D, 0x1A);
	write_cmos_sensor(0x992F, 0x14);
	write_cmos_sensor(0x9931, 0x3F);
	write_cmos_sensor(0x9933, 0x3F);
	write_cmos_sensor(0x9935, 0x3F);
	write_cmos_sensor(0x9937, 0x6B);
	write_cmos_sensor(0x9939, 0x7C);
	write_cmos_sensor(0x993B, 0x81);
	write_cmos_sensor(0x9943, 0x0F);
	write_cmos_sensor(0x9945, 0x0F);
	write_cmos_sensor(0x9947, 0x0F);
	write_cmos_sensor(0x9949, 0x0F);
	write_cmos_sensor(0x994B, 0x0F);
	write_cmos_sensor(0x994D, 0x0F);
	write_cmos_sensor(0x994F, 0x42);
	write_cmos_sensor(0x9951, 0x0F);
	write_cmos_sensor(0x9953, 0x0B);
	write_cmos_sensor(0x9955, 0x5A);
	write_cmos_sensor(0x9957, 0x13);
	write_cmos_sensor(0x9959, 0x0C);
	write_cmos_sensor(0x995A, 0x00);
	write_cmos_sensor(0x995B, 0x00);
	write_cmos_sensor(0x995C, 0x00);
	write_cmos_sensor(0x996B, 0x00);
	write_cmos_sensor(0x996D, 0x10);
	write_cmos_sensor(0x996F, 0x10);
	write_cmos_sensor(0x9971, 0xC8);
	write_cmos_sensor(0x9973, 0x32);
	write_cmos_sensor(0x9975, 0x04);
	write_cmos_sensor(0x9976, 0x0A);
	write_cmos_sensor(0x9977, 0x0A);
	write_cmos_sensor(0x9978, 0x0A);
	write_cmos_sensor(0x99A4, 0x2F);
	write_cmos_sensor(0x99A5, 0x2F);
	write_cmos_sensor(0x99A6, 0x2F);
	write_cmos_sensor(0x99A7, 0x0A);
	write_cmos_sensor(0x99A8, 0x0A);
	write_cmos_sensor(0x99A9, 0x0A);
	write_cmos_sensor(0x99AA, 0x2F);
	write_cmos_sensor(0x99AB, 0x2F);
	write_cmos_sensor(0x99AC, 0x2F);
	write_cmos_sensor(0x99AD, 0x00);
	write_cmos_sensor(0x99AE, 0x00);
	write_cmos_sensor(0x99AF, 0x00);
	write_cmos_sensor(0x99B0, 0x40);
	write_cmos_sensor(0x99B1, 0x40);
	write_cmos_sensor(0x99B2, 0x40);
	write_cmos_sensor(0x99B3, 0x30);
	write_cmos_sensor(0x99B4, 0x30);
	write_cmos_sensor(0x99B5, 0x30);
	write_cmos_sensor(0x99BB, 0x0A);
	write_cmos_sensor(0x99BD, 0x0A);
	write_cmos_sensor(0x99BF, 0x0A);
	write_cmos_sensor(0x99C0, 0x09);
	write_cmos_sensor(0x99C1, 0x09);
	write_cmos_sensor(0x99C2, 0x09);
	write_cmos_sensor(0x99C6, 0x3C);
	write_cmos_sensor(0x99C7, 0x3C);
	write_cmos_sensor(0x99C8, 0x3C);
	write_cmos_sensor(0x99C9, 0xFF);
	write_cmos_sensor(0x99CA, 0xFF);
	write_cmos_sensor(0x99CB, 0xFF);
	write_cmos_sensor(0x9A01, 0x35);
	write_cmos_sensor(0x9A03, 0x14);
	write_cmos_sensor(0x9A05, 0x14);
	write_cmos_sensor(0x9A07, 0x31);
	write_cmos_sensor(0x9A09, 0x1B);
	write_cmos_sensor(0x9A0B, 0x15);
	write_cmos_sensor(0x9A0D, 0x1E);
	write_cmos_sensor(0x9A0F, 0x1E);
	write_cmos_sensor(0x9A11, 0x1E);
	write_cmos_sensor(0x9A13, 0x64);
	write_cmos_sensor(0x9A15, 0x64);
	write_cmos_sensor(0x9A17, 0x64);
	write_cmos_sensor(0x9A19, 0x50);
	write_cmos_sensor(0x9A1B, 0x60);
	write_cmos_sensor(0x9A1D, 0x65);
	write_cmos_sensor(0x9A1F, 0x01);
	write_cmos_sensor(0x9A21, 0x01);
	write_cmos_sensor(0x9A23, 0x01);
	write_cmos_sensor(0x9A25, 0x14);
	write_cmos_sensor(0x9A27, 0x14);
	write_cmos_sensor(0x9A29, 0x14);
	write_cmos_sensor(0x9A2B, 0x2F);
	write_cmos_sensor(0x9A2D, 0x1A);
	write_cmos_sensor(0x9A2F, 0x14);
	write_cmos_sensor(0x9A31, 0x1E);
	write_cmos_sensor(0x9A33, 0x1E);
	write_cmos_sensor(0x9A35, 0x1E);
	write_cmos_sensor(0x9A37, 0x6B);
	write_cmos_sensor(0x9A39, 0x7C);
	write_cmos_sensor(0x9A3B, 0x81);
	write_cmos_sensor(0x9A3D, 0x00);
	write_cmos_sensor(0x9A3F, 0x00);
	write_cmos_sensor(0x9A41, 0x00);
	write_cmos_sensor(0x9A4F, 0x42);
	write_cmos_sensor(0x9A51, 0x0F);
	write_cmos_sensor(0x9A53, 0x0B);
	write_cmos_sensor(0x9A55, 0x5A);
	write_cmos_sensor(0x9A57, 0x13);
	write_cmos_sensor(0x9A59, 0x0C);
	write_cmos_sensor(0x9A5A, 0x00);
	write_cmos_sensor(0x9A5B, 0x00);
	write_cmos_sensor(0x9A5C, 0x00);
	write_cmos_sensor(0x9A6B, 0x00);
	write_cmos_sensor(0x9A6D, 0x10);
	write_cmos_sensor(0x9A6F, 0x10);
	write_cmos_sensor(0x9A71, 0xC8);
	write_cmos_sensor(0x9A73, 0x32);
	write_cmos_sensor(0x9A75, 0x04);
	write_cmos_sensor(0x9AA4, 0x3F);
	write_cmos_sensor(0x9AA5, 0x3F);
	write_cmos_sensor(0x9AA6, 0x3F);
	write_cmos_sensor(0x9AA7, 0x0A);
	write_cmos_sensor(0x9AA8, 0x0A);
	write_cmos_sensor(0x9AA9, 0x0A);
	write_cmos_sensor(0x9AAA, 0x25);
	write_cmos_sensor(0x9AAB, 0x25);
	write_cmos_sensor(0x9AAC, 0x25);
	write_cmos_sensor(0x9AAD, 0x00);
	write_cmos_sensor(0x9AAE, 0x00);
	write_cmos_sensor(0x9AAF, 0x00);
	write_cmos_sensor(0x9AB0, 0x40);
	write_cmos_sensor(0x9AB1, 0x40);
	write_cmos_sensor(0x9AB2, 0x40);
	write_cmos_sensor(0x9AB3, 0x30);
	write_cmos_sensor(0x9AB4, 0x30);
	write_cmos_sensor(0x9AB5, 0x30);
	write_cmos_sensor(0x9AB6, 0xA0);
	write_cmos_sensor(0x9AB7, 0xA0);
	write_cmos_sensor(0x9AB8, 0xA0);
	write_cmos_sensor(0x9ABB, 0x0A);
	write_cmos_sensor(0x9ABD, 0x0A);
	write_cmos_sensor(0x9ABF, 0x0A);
	write_cmos_sensor(0x9AC0, 0x09);
	write_cmos_sensor(0x9AC1, 0x09);
	write_cmos_sensor(0x9AC2, 0x09);
	write_cmos_sensor(0x9AC6, 0x2D);
	write_cmos_sensor(0x9AC7, 0x2D);
	write_cmos_sensor(0x9AC8, 0x2D);
	write_cmos_sensor(0x9AC9, 0xFF);
	write_cmos_sensor(0x9ACA, 0xFF);
	write_cmos_sensor(0x9ACB, 0xFF);
	write_cmos_sensor(0x9B01, 0x35);
	write_cmos_sensor(0x9B03, 0x14);
	write_cmos_sensor(0x9B05, 0x14);
	write_cmos_sensor(0x9B07, 0x31);
	write_cmos_sensor(0x9B09, 0x1B);
	write_cmos_sensor(0x9B0B, 0x15);
	write_cmos_sensor(0x9B0D, 0x1E);
	write_cmos_sensor(0x9B0F, 0x1E);
	write_cmos_sensor(0x9B11, 0x1E);
	write_cmos_sensor(0x9B13, 0x64);
	write_cmos_sensor(0x9B15, 0x64);
	write_cmos_sensor(0x9B17, 0x64);
	write_cmos_sensor(0x9B19, 0x50);
	write_cmos_sensor(0x9B1B, 0x60);
	write_cmos_sensor(0x9B1D, 0x65);
	write_cmos_sensor(0x9B1F, 0x01);
	write_cmos_sensor(0x9B21, 0x01);
	write_cmos_sensor(0x9B23, 0x01);
	write_cmos_sensor(0x9B25, 0x14);
	write_cmos_sensor(0x9B27, 0x14);
	write_cmos_sensor(0x9B29, 0x14);
	write_cmos_sensor(0x9B2B, 0x2F);
	write_cmos_sensor(0x9B2D, 0x1A);
	write_cmos_sensor(0x9B2F, 0x14);
	write_cmos_sensor(0x9B31, 0x1E);
	write_cmos_sensor(0x9B33, 0x1E);
	write_cmos_sensor(0x9B35, 0x1E);
	write_cmos_sensor(0x9B37, 0x6B);
	write_cmos_sensor(0x9B39, 0x7C);
	write_cmos_sensor(0x9B3B, 0x81);
	write_cmos_sensor(0x9B43, 0x0F);
	write_cmos_sensor(0x9B45, 0x0F);
	write_cmos_sensor(0x9B47, 0x0F);
	write_cmos_sensor(0x9B49, 0x0F);
	write_cmos_sensor(0x9B4B, 0x0F);
	write_cmos_sensor(0x9B4D, 0x0F);
	write_cmos_sensor(0x9B4F, 0x2D);
	write_cmos_sensor(0x9B51, 0x0B);
	write_cmos_sensor(0x9B53, 0x08);
	write_cmos_sensor(0x9B55, 0x40);
	write_cmos_sensor(0x9B57, 0x0D);
	write_cmos_sensor(0x9B59, 0x08);
	write_cmos_sensor(0x9B5A, 0x00);
	write_cmos_sensor(0x9B5B, 0x00);
	write_cmos_sensor(0x9B5C, 0x00);
	write_cmos_sensor(0x9B6B, 0x00);
	write_cmos_sensor(0x9B6D, 0x10);
	write_cmos_sensor(0x9B6F, 0x10);
	write_cmos_sensor(0x9B71, 0xC8);
	write_cmos_sensor(0x9B73, 0x32);
	write_cmos_sensor(0x9B75, 0x04);
	write_cmos_sensor(0x9BB0, 0x40);
	write_cmos_sensor(0x9BB1, 0x40);
	write_cmos_sensor(0x9BB2, 0x40);
	write_cmos_sensor(0x9BB3, 0x30);
	write_cmos_sensor(0x9BB4, 0x30);
	write_cmos_sensor(0x9BB5, 0x30);
	write_cmos_sensor(0x9BBB, 0x0A);
	write_cmos_sensor(0x9BBD, 0x0A);
	write_cmos_sensor(0x9BBF, 0x0A);
	write_cmos_sensor(0x9BC0, 0x09);
	write_cmos_sensor(0x9BC1, 0x09);
	write_cmos_sensor(0x9BC2, 0x09);
	write_cmos_sensor(0x9BC6, 0x18);
	write_cmos_sensor(0x9BC7, 0x18);
	write_cmos_sensor(0x9BC8, 0x18);
	write_cmos_sensor(0x9BC9, 0xFF);
	write_cmos_sensor(0x9BCA, 0xFF);
	write_cmos_sensor(0x9BCB, 0xFF);
	write_cmos_sensor(0x9C01, 0x35);
	write_cmos_sensor(0x9C03, 0x14);
	write_cmos_sensor(0x9C05, 0x14);
	write_cmos_sensor(0x9C07, 0x31);
	write_cmos_sensor(0x9C09, 0x1B);
	write_cmos_sensor(0x9C0B, 0x15);
	write_cmos_sensor(0x9C0D, 0x1E);
	write_cmos_sensor(0x9C0F, 0x1E);
	write_cmos_sensor(0x9C11, 0x1E);
	write_cmos_sensor(0x9C13, 0x64);
	write_cmos_sensor(0x9C15, 0x64);
	write_cmos_sensor(0x9C17, 0x64);
	write_cmos_sensor(0x9C19, 0x50);
	write_cmos_sensor(0x9C1B, 0x60);
	write_cmos_sensor(0x9C1D, 0x65);
	write_cmos_sensor(0x9C1F, 0x01);
	write_cmos_sensor(0x9C21, 0x01);
	write_cmos_sensor(0x9C23, 0x01);
	write_cmos_sensor(0x9C25, 0x14);
	write_cmos_sensor(0x9C27, 0x14);
	write_cmos_sensor(0x9C29, 0x14);
	write_cmos_sensor(0x9C2B, 0x2F);
	write_cmos_sensor(0x9C2D, 0x1A);
	write_cmos_sensor(0x9C2F, 0x14);
	write_cmos_sensor(0x9C31, 0x1E);
	write_cmos_sensor(0x9C33, 0x1E);
	write_cmos_sensor(0x9C35, 0x1E);
	write_cmos_sensor(0x9C37, 0x6B);
	write_cmos_sensor(0x9C39, 0x7C);
	write_cmos_sensor(0x9C3B, 0x81);
	write_cmos_sensor(0x9C3D, 0x00);
	write_cmos_sensor(0x9C3F, 0x00);
	write_cmos_sensor(0x9C41, 0x00);
	write_cmos_sensor(0x9C4F, 0x42);
	write_cmos_sensor(0x9C51, 0x0B);
	write_cmos_sensor(0x9C53, 0x08);
	write_cmos_sensor(0x9C55, 0x5A);
	write_cmos_sensor(0x9C57, 0x0D);
	write_cmos_sensor(0x9C59, 0x08);
	write_cmos_sensor(0x9C5A, 0x00);
	write_cmos_sensor(0x9C5B, 0x00);
	write_cmos_sensor(0x9C5C, 0x00);
	write_cmos_sensor(0x9C6B, 0x00);
	write_cmos_sensor(0x9C6D, 0x10);
	write_cmos_sensor(0x9C6F, 0x10);
	write_cmos_sensor(0x9C71, 0xC8);
	write_cmos_sensor(0x9C73, 0x32);
	write_cmos_sensor(0x9C75, 0x04);
	write_cmos_sensor(0x9CB0, 0x50);
	write_cmos_sensor(0x9CB1, 0x50);
	write_cmos_sensor(0x9CB2, 0x50);
	write_cmos_sensor(0x9CB3, 0x40);
	write_cmos_sensor(0x9CB4, 0x40);
	write_cmos_sensor(0x9CB5, 0x40);
	write_cmos_sensor(0x9CBB, 0x0A);
	write_cmos_sensor(0x9CBD, 0x0A);
	write_cmos_sensor(0x9CBF, 0x0A);
	write_cmos_sensor(0x9CC0, 0x09);
	write_cmos_sensor(0x9CC1, 0x09);
	write_cmos_sensor(0x9CC2, 0x09);
	write_cmos_sensor(0x9CC6, 0x18);
	write_cmos_sensor(0x9CC7, 0x18);
	write_cmos_sensor(0x9CC8, 0x18);
	write_cmos_sensor(0x9CC9, 0xFF);
	write_cmos_sensor(0x9CCA, 0xFF);
	write_cmos_sensor(0x9CCB, 0xFF);
	write_cmos_sensor(0x9D01, 0x35);
	write_cmos_sensor(0x9D03, 0x14);
	write_cmos_sensor(0x9D05, 0x14);
	write_cmos_sensor(0x9D07, 0x31);
	write_cmos_sensor(0x9D09, 0x1B);
	write_cmos_sensor(0x9D0B, 0x15);
	write_cmos_sensor(0x9D0D, 0x1E);
	write_cmos_sensor(0x9D0F, 0x1E);
	write_cmos_sensor(0x9D11, 0x1E);
	write_cmos_sensor(0x9D13, 0x64);
	write_cmos_sensor(0x9D15, 0x64);
	write_cmos_sensor(0x9D17, 0x64);
	write_cmos_sensor(0x9D19, 0x50);
	write_cmos_sensor(0x9D1B, 0x60);
	write_cmos_sensor(0x9D1D, 0x65);
	write_cmos_sensor(0x9D25, 0x14);
	write_cmos_sensor(0x9D27, 0x14);
	write_cmos_sensor(0x9D29, 0x14);
	write_cmos_sensor(0x9D2B, 0x2F);
	write_cmos_sensor(0x9D2D, 0x1A);
	write_cmos_sensor(0x9D2F, 0x14);
	write_cmos_sensor(0x9D31, 0x1E);
	write_cmos_sensor(0x9D33, 0x1E);
	write_cmos_sensor(0x9D35, 0x1E);
	write_cmos_sensor(0x9D37, 0x6B);
	write_cmos_sensor(0x9D39, 0x7C);
	write_cmos_sensor(0x9D3B, 0x81);
	write_cmos_sensor(0x9D3D, 0x00);
	write_cmos_sensor(0x9D3F, 0x00);
	write_cmos_sensor(0x9D41, 0x00);
	write_cmos_sensor(0x9D4F, 0x42);
	write_cmos_sensor(0x9D51, 0x0F);
	write_cmos_sensor(0x9D53, 0x0B);
	write_cmos_sensor(0x9D55, 0x5A);
	write_cmos_sensor(0x9D57, 0x13);
	write_cmos_sensor(0x9D59, 0x0C);
	write_cmos_sensor(0x9D5B, 0x35);
	write_cmos_sensor(0x9D5D, 0x14);
	write_cmos_sensor(0x9D5F, 0x14);
	write_cmos_sensor(0x9D61, 0x35);
	write_cmos_sensor(0x9D63, 0x14);
	write_cmos_sensor(0x9D65, 0x14);
	write_cmos_sensor(0x9D67, 0x31);
	write_cmos_sensor(0x9D69, 0x1B);
	write_cmos_sensor(0x9D6B, 0x15);
	write_cmos_sensor(0x9D6D, 0x31);
	write_cmos_sensor(0x9D6F, 0x1B);
	write_cmos_sensor(0x9D71, 0x15);
	write_cmos_sensor(0x9D73, 0x1E);
	write_cmos_sensor(0x9D75, 0x1E);
	write_cmos_sensor(0x9D77, 0x1E);
	write_cmos_sensor(0x9D79, 0x1E);
	write_cmos_sensor(0x9D7B, 0x1E);
	write_cmos_sensor(0x9D7D, 0x1E);
	write_cmos_sensor(0x9D7F, 0x64);
	write_cmos_sensor(0x9D81, 0x64);
	write_cmos_sensor(0x9D83, 0x64);
	write_cmos_sensor(0x9D85, 0x64);
	write_cmos_sensor(0x9D87, 0x64);
	write_cmos_sensor(0x9D89, 0x64);
	write_cmos_sensor(0x9D8B, 0x50);
	write_cmos_sensor(0x9D8D, 0x60);
	write_cmos_sensor(0x9D8F, 0x65);
	write_cmos_sensor(0x9D91, 0x50);
	write_cmos_sensor(0x9D93, 0x60);
	write_cmos_sensor(0x9D95, 0x65);
	write_cmos_sensor(0x9D97, 0x01);
	write_cmos_sensor(0x9D99, 0x01);
	write_cmos_sensor(0x9D9B, 0x01);
	write_cmos_sensor(0x9D9D, 0x01);
	write_cmos_sensor(0x9D9F, 0x01);
	write_cmos_sensor(0x9DA1, 0x01);
	write_cmos_sensor(0x9E01, 0x35);
	write_cmos_sensor(0x9E03, 0x14);
	write_cmos_sensor(0x9E05, 0x14);
	write_cmos_sensor(0x9E07, 0x31);
	write_cmos_sensor(0x9E09, 0x1B);
	write_cmos_sensor(0x9E0B, 0x15);
	write_cmos_sensor(0x9E0D, 0x1E);
	write_cmos_sensor(0x9E0F, 0x1E);
	write_cmos_sensor(0x9E11, 0x1E);
	write_cmos_sensor(0x9E13, 0x64);
	write_cmos_sensor(0x9E15, 0x64);
	write_cmos_sensor(0x9E17, 0x64);
	write_cmos_sensor(0x9E19, 0x50);
	write_cmos_sensor(0x9E1B, 0x60);
	write_cmos_sensor(0x9E1D, 0x65);
	write_cmos_sensor(0x9E25, 0x14);
	write_cmos_sensor(0x9E27, 0x14);
	write_cmos_sensor(0x9E29, 0x14);
	write_cmos_sensor(0x9E2B, 0x2F);
	write_cmos_sensor(0x9E2D, 0x1A);
	write_cmos_sensor(0x9E2F, 0x14);
	write_cmos_sensor(0x9E31, 0x1E);
	write_cmos_sensor(0x9E33, 0x1E);
	write_cmos_sensor(0x9E35, 0x1E);
	write_cmos_sensor(0x9E37, 0x6B);
	write_cmos_sensor(0x9E39, 0x7C);
	write_cmos_sensor(0x9E3B, 0x81);
	write_cmos_sensor(0x9E3D, 0x00);
	write_cmos_sensor(0x9E3F, 0x00);
	write_cmos_sensor(0x9E41, 0x00);
	write_cmos_sensor(0x9E4F, 0x42);
	write_cmos_sensor(0x9E51, 0x0B);
	write_cmos_sensor(0x9E53, 0x08);
	write_cmos_sensor(0x9E55, 0x5A);
	write_cmos_sensor(0x9E57, 0x0D);
	write_cmos_sensor(0x9E59, 0x08);
	write_cmos_sensor(0x9E5B, 0x35);
	write_cmos_sensor(0x9E5D, 0x14);
	write_cmos_sensor(0x9E5F, 0x14);
	write_cmos_sensor(0x9E61, 0x35);
	write_cmos_sensor(0x9E63, 0x14);
	write_cmos_sensor(0x9E65, 0x14);
	write_cmos_sensor(0x9E67, 0x31);
	write_cmos_sensor(0x9E69, 0x1B);
	write_cmos_sensor(0x9E6B, 0x15);
	write_cmos_sensor(0x9E6D, 0x31);
	write_cmos_sensor(0x9E6F, 0x1B);
	write_cmos_sensor(0x9E71, 0x15);
	write_cmos_sensor(0x9E73, 0x1E);
	write_cmos_sensor(0x9E75, 0x1E);
	write_cmos_sensor(0x9E77, 0x1E);
	write_cmos_sensor(0x9E79, 0x1E);
	write_cmos_sensor(0x9E7B, 0x1E);
	write_cmos_sensor(0x9E7D, 0x1E);
	write_cmos_sensor(0x9E7F, 0x64);
	write_cmos_sensor(0x9E81, 0x64);
	write_cmos_sensor(0x9E83, 0x64);
	write_cmos_sensor(0x9E85, 0x64);
	write_cmos_sensor(0x9E87, 0x64);
	write_cmos_sensor(0x9E89, 0x64);
	write_cmos_sensor(0x9E8B, 0x50);
	write_cmos_sensor(0x9E8D, 0x60);
	write_cmos_sensor(0x9E8F, 0x65);
	write_cmos_sensor(0x9E91, 0x50);
	write_cmos_sensor(0x9E93, 0x60);
	write_cmos_sensor(0x9E95, 0x65);
	write_cmos_sensor(0x9E97, 0x01);
	write_cmos_sensor(0x9E99, 0x01);
	write_cmos_sensor(0x9E9B, 0x01);
	write_cmos_sensor(0x9E9D, 0x01);
	write_cmos_sensor(0x9E9F, 0x01);
	write_cmos_sensor(0x9EA1, 0x01);
	write_cmos_sensor(0x9F01, 0x14);
	write_cmos_sensor(0x9F03, 0x14);
	write_cmos_sensor(0x9F05, 0x14);
	write_cmos_sensor(0x9F07, 0x14);
	write_cmos_sensor(0x9F09, 0x14);
	write_cmos_sensor(0x9F0B, 0x14);
	write_cmos_sensor(0x9F0D, 0x2F);
	write_cmos_sensor(0x9F0F, 0x1A);
	write_cmos_sensor(0x9F11, 0x14);
	write_cmos_sensor(0x9F13, 0x2F);
	write_cmos_sensor(0x9F15, 0x1A);
	write_cmos_sensor(0x9F17, 0x14);
	write_cmos_sensor(0x9F19, 0x1E);
	write_cmos_sensor(0x9F1B, 0x1E);
	write_cmos_sensor(0x9F1D, 0x1E);
	write_cmos_sensor(0x9F1F, 0x1E);
	write_cmos_sensor(0x9F21, 0x1E);
	write_cmos_sensor(0x9F23, 0x1E);
	write_cmos_sensor(0x9F25, 0x6B);
	write_cmos_sensor(0x9F27, 0x7C);
	write_cmos_sensor(0x9F29, 0x81);
	write_cmos_sensor(0x9F2B, 0x6B);
	write_cmos_sensor(0x9F2D, 0x7C);
	write_cmos_sensor(0x9F2F, 0x81);
	write_cmos_sensor(0x9F31, 0x00);
	write_cmos_sensor(0x9F33, 0x00);
	write_cmos_sensor(0x9F35, 0x00);
	write_cmos_sensor(0x9F37, 0x00);
	write_cmos_sensor(0x9F39, 0x00);
	write_cmos_sensor(0x9F3B, 0x00);
	write_cmos_sensor(0x9F3C, 0x00);
	write_cmos_sensor(0x9F3D, 0x00);
	write_cmos_sensor(0x9F3E, 0x00);
	write_cmos_sensor(0x9F41, 0x00);
	write_cmos_sensor(0x9F43, 0x10);
	write_cmos_sensor(0x9F45, 0x10);
	write_cmos_sensor(0x9F47, 0xC8);
	write_cmos_sensor(0x9F49, 0x32);
	write_cmos_sensor(0x9F4B, 0x04);
	write_cmos_sensor(0x9F4D, 0x00);
	write_cmos_sensor(0x9F4F, 0x10);
	write_cmos_sensor(0x9F51, 0x10);
	write_cmos_sensor(0x9F53, 0x00);
	write_cmos_sensor(0x9F55, 0x10);
	write_cmos_sensor(0x9F57, 0x10);
	write_cmos_sensor(0x9F59, 0x20);
	write_cmos_sensor(0x9F5B, 0x04);
	write_cmos_sensor(0x9F5D, 0x04);
	write_cmos_sensor(0x9F5F, 0x20);
	write_cmos_sensor(0x9F61, 0x04);
	write_cmos_sensor(0x9F63, 0x04);
	write_cmos_sensor(0x9F77, 0x42);
	write_cmos_sensor(0x9F79, 0x0F);
	write_cmos_sensor(0x9F7B, 0x0B);
	write_cmos_sensor(0x9F7D, 0x42);
	write_cmos_sensor(0x9F7F, 0x0F);
	write_cmos_sensor(0x9F81, 0x0B);
	write_cmos_sensor(0x9F83, 0x5A);
	write_cmos_sensor(0x9F85, 0x13);
	write_cmos_sensor(0x9F87, 0x0C);
	write_cmos_sensor(0x9F89, 0x5A);
	write_cmos_sensor(0x9F8B, 0x13);
	write_cmos_sensor(0x9F8D, 0x0C);
	write_cmos_sensor(0x9FA6, 0x3F);
	write_cmos_sensor(0x9FA7, 0x3F);
	write_cmos_sensor(0x9FA8, 0x3F);
	write_cmos_sensor(0x9FA9, 0x0A);
	write_cmos_sensor(0x9FAA, 0x0A);
	write_cmos_sensor(0x9FAB, 0x0A);
	write_cmos_sensor(0x9FAC, 0x25);
	write_cmos_sensor(0x9FAD, 0x25);
	write_cmos_sensor(0x9FAE, 0x25);
	write_cmos_sensor(0x9FAF, 0x00);
	write_cmos_sensor(0x9FB0, 0x00);
	write_cmos_sensor(0x9FB1, 0x00);
	write_cmos_sensor(0xA001, 0x14);
	write_cmos_sensor(0xA003, 0x14);
	write_cmos_sensor(0xA005, 0x14);
	write_cmos_sensor(0xA007, 0x14);
	write_cmos_sensor(0xA009, 0x14);
	write_cmos_sensor(0xA00B, 0x14);
	write_cmos_sensor(0xA00D, 0x2F);
	write_cmos_sensor(0xA00F, 0x1A);
	write_cmos_sensor(0xA011, 0x14);
	write_cmos_sensor(0xA013, 0x2F);
	write_cmos_sensor(0xA015, 0x1A);
	write_cmos_sensor(0xA017, 0x14);
	write_cmos_sensor(0xA019, 0x1E);
	write_cmos_sensor(0xA01B, 0x1E);
	write_cmos_sensor(0xA01D, 0x1E);
	write_cmos_sensor(0xA01F, 0x1E);
	write_cmos_sensor(0xA021, 0x1E);
	write_cmos_sensor(0xA023, 0x1E);
	write_cmos_sensor(0xA025, 0x6B);
	write_cmos_sensor(0xA027, 0x7C);
	write_cmos_sensor(0xA029, 0x81);
	write_cmos_sensor(0xA02B, 0x6B);
	write_cmos_sensor(0xA02D, 0x7C);
	write_cmos_sensor(0xA02F, 0x81);
	write_cmos_sensor(0xA031, 0x00);
	write_cmos_sensor(0xA033, 0x00);
	write_cmos_sensor(0xA035, 0x00);
	write_cmos_sensor(0xA037, 0x00);
	write_cmos_sensor(0xA039, 0x00);
	write_cmos_sensor(0xA03B, 0x00);
	write_cmos_sensor(0xA03C, 0x00);
	write_cmos_sensor(0xA03D, 0x00);
	write_cmos_sensor(0xA03E, 0x00);
	write_cmos_sensor(0xA041, 0x00);
	write_cmos_sensor(0xA043, 0x10);
	write_cmos_sensor(0xA045, 0x10);
	write_cmos_sensor(0xA047, 0xC8);
	write_cmos_sensor(0xA049, 0x32);
	write_cmos_sensor(0xA04B, 0x04);
	write_cmos_sensor(0xA04D, 0x00);
	write_cmos_sensor(0xA04F, 0x10);
	write_cmos_sensor(0xA051, 0x10);
	write_cmos_sensor(0xA053, 0x00);
	write_cmos_sensor(0xA055, 0x10);
	write_cmos_sensor(0xA057, 0x10);
	write_cmos_sensor(0xA059, 0x20);
	write_cmos_sensor(0xA05B, 0x04);
	write_cmos_sensor(0xA05D, 0x04);
	write_cmos_sensor(0xA05F, 0x20);
	write_cmos_sensor(0xA061, 0x04);
	write_cmos_sensor(0xA063, 0x04);
	write_cmos_sensor(0xA077, 0x42);
	write_cmos_sensor(0xA079, 0x0B);
	write_cmos_sensor(0xA07B, 0x08);
	write_cmos_sensor(0xA07D, 0x42);
	write_cmos_sensor(0xA07F, 0x0B);
	write_cmos_sensor(0xA081, 0x08);
	write_cmos_sensor(0xA083, 0x5A);
	write_cmos_sensor(0xA085, 0x0D);
	write_cmos_sensor(0xA087, 0x08);
	write_cmos_sensor(0xA089, 0x5A);
	write_cmos_sensor(0xA08B, 0x0D);
	write_cmos_sensor(0xA08D, 0x08);
	write_cmos_sensor(0xA716, 0x13);
	write_cmos_sensor(0xA801, 0x08);
	write_cmos_sensor(0xA803, 0x0C);
	write_cmos_sensor(0xA805, 0x10);
	write_cmos_sensor(0xA806, 0x00);
	write_cmos_sensor(0xA807, 0x18);
	write_cmos_sensor(0xA808, 0x00);
	write_cmos_sensor(0xA809, 0x20);
	write_cmos_sensor(0xA80A, 0x00);
	write_cmos_sensor(0xA80B, 0x30);
	write_cmos_sensor(0xA80C, 0x00);
	write_cmos_sensor(0xA80D, 0x40);
	write_cmos_sensor(0xA80E, 0x00);
	write_cmos_sensor(0xA80F, 0x60);
	write_cmos_sensor(0xA810, 0x00);
	write_cmos_sensor(0xA811, 0x80);
	write_cmos_sensor(0xA812, 0x00);
	write_cmos_sensor(0xA813, 0xC0);
	write_cmos_sensor(0xA814, 0x01);
	write_cmos_sensor(0xA815, 0x00);
	write_cmos_sensor(0xA816, 0x01);
	write_cmos_sensor(0xA817, 0x80);
	write_cmos_sensor(0xA818, 0x02);
	write_cmos_sensor(0xA819, 0x00);
	write_cmos_sensor(0xA81B, 0x00);
	write_cmos_sensor(0xA81D, 0xAC);
	write_cmos_sensor(0xA838, 0x03);
	write_cmos_sensor(0xA83C, 0x28);
	write_cmos_sensor(0xA83D, 0x5F);
	write_cmos_sensor(0xA881, 0x08);
	write_cmos_sensor(0xA883, 0x0C);
	write_cmos_sensor(0xA885, 0x10);
	write_cmos_sensor(0xA886, 0x00);
	write_cmos_sensor(0xA887, 0x18);
	write_cmos_sensor(0xA888, 0x00);
	write_cmos_sensor(0xA889, 0x20);
	write_cmos_sensor(0xA88A, 0x00);
	write_cmos_sensor(0xA88B, 0x30);
	write_cmos_sensor(0xA88C, 0x00);
	write_cmos_sensor(0xA88D, 0x40);
	write_cmos_sensor(0xA88E, 0x00);
	write_cmos_sensor(0xA88F, 0x60);
	write_cmos_sensor(0xA890, 0x00);
	write_cmos_sensor(0xA891, 0x80);
	write_cmos_sensor(0xA892, 0x00);
	write_cmos_sensor(0xA893, 0xC0);
	write_cmos_sensor(0xA894, 0x01);
	write_cmos_sensor(0xA895, 0x00);
	write_cmos_sensor(0xA896, 0x01);
	write_cmos_sensor(0xA897, 0x80);
	write_cmos_sensor(0xA898, 0x02);
	write_cmos_sensor(0xA899, 0x00);
	write_cmos_sensor(0xA89B, 0x00);
	write_cmos_sensor(0xA89D, 0xAC);
	write_cmos_sensor(0xA8B8, 0x03);
	write_cmos_sensor(0xA8BB, 0x13);
	write_cmos_sensor(0xA8BC, 0x28);
	write_cmos_sensor(0xA8BD, 0x25);
	write_cmos_sensor(0xA8BE, 0x1D);
	write_cmos_sensor(0xA8C0, 0x3A);
	write_cmos_sensor(0xA8C1, 0xE0);
	write_cmos_sensor(0xB2B2, 0x01);
	write_cmos_sensor(0xB2D6, 0x80);
	write_cmos_sensor(0xC068, 0x03);
	write_cmos_sensor(0xC0CD, 0x00);
	write_cmos_sensor(0xC65C, 0x01);
	write_cmos_sensor(0xC669, 0x01);

	write_cmos_sensor(0x0100, 0x00);

}				/*    sensor_init  */


static void preview_setting(void)
{
	LOG_INF("E\n");


	/*************mode setting************/
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0220, 0x00);
	write_cmos_sensor(0x0221, 0x11);
	write_cmos_sensor(0x0222, 0x10);
	write_cmos_sensor(0x0340, 0x07);
	write_cmos_sensor(0x0341, 0x80);
	write_cmos_sensor(0x0342, 0x15);
	write_cmos_sensor(0x0343, 0xA0);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x00);
	write_cmos_sensor(0x0347, 0x00);
	write_cmos_sensor(0x0348, 0x12);
	write_cmos_sensor(0x0349, 0x2F);
	write_cmos_sensor(0x034A, 0x0D);
	write_cmos_sensor(0x034B, 0xA7);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x01);
	write_cmos_sensor(0x0901, 0x22);
	write_cmos_sensor(0x0902, 0x02);
	write_cmos_sensor(0x3010, 0x65);
	write_cmos_sensor(0x3011, 0x01);
	write_cmos_sensor(0x30C0, 0x11);
	write_cmos_sensor(0x300D, 0x00);
	write_cmos_sensor(0x30FD, 0x00);
	write_cmos_sensor(0x8493, 0x00);
	write_cmos_sensor(0x8863, 0x00);
	write_cmos_sensor(0x90D7, 0x19);
	/************output size setting**********/
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x034C, 0x09);
	write_cmos_sensor(0x034D, 0x18);
	write_cmos_sensor(0x034E, 0x06);
	write_cmos_sensor(0x034F, 0xD4);
	write_cmos_sensor(0x0401, 0x00);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x10);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x00);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x09);
	write_cmos_sensor(0x040D, 0x18);
	write_cmos_sensor(0x040E, 0x06);
	write_cmos_sensor(0x040F, 0xD4);
	/***********clock setting************/
	write_cmos_sensor(0x0301, 0x05);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0x85);
	write_cmos_sensor(0x0309, 0x0A);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x0F);
	write_cmos_sensor(0x030E, 0x03);
	write_cmos_sensor(0x030F, 0x41);
	write_cmos_sensor(0x0310, 0x00);
	/*************data rateing setting*********/
	write_cmos_sensor(0x0820, 0x0C);
	write_cmos_sensor(0x0821, 0x78);
	write_cmos_sensor(0x0822, 0x00);
	write_cmos_sensor(0x0823, 0x00);
	/*************integration time setting******/
	write_cmos_sensor(0x0202, 0x07);
	write_cmos_sensor(0x0203, 0x76);
	write_cmos_sensor(0x0224, 0x01);
	write_cmos_sensor(0x0225, 0xF4);
	/************gain setting****************/
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x0216, 0x00);
	write_cmos_sensor(0x0217, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	write_cmos_sensor(0x0210, 0x01);
	write_cmos_sensor(0x0211, 0x00);
	write_cmos_sensor(0x0212, 0x01);
	write_cmos_sensor(0x0213, 0x00);
	write_cmos_sensor(0x0214, 0x01);
	write_cmos_sensor(0x0215, 0x00);
	/*******NR Setting*****************/
	write_cmos_sensor(0x3058, 0x00);

}				/*    preview_setting  */

static void preview_setting_HDR_ES2(void)
{
	LOG_INF("Enter\n");

	/*************mode setting***************/
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0220, 0x03);
	write_cmos_sensor(0x0221, 0x22);
	write_cmos_sensor(0x0222, 0x10);
	write_cmos_sensor(0x302b, 0x01);
	write_cmos_sensor(0x0340, 0x07);
	write_cmos_sensor(0x0341, 0x0E);
	write_cmos_sensor(0x0342, 0x15);
	write_cmos_sensor(0x0343, 0xA0);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x00);
	write_cmos_sensor(0x0347, 0x00);
	write_cmos_sensor(0x0348, 0x12);
	write_cmos_sensor(0x0349, 0x2F);
	write_cmos_sensor(0x034A, 0x0D);
	write_cmos_sensor(0x034B, 0xA7);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x00);
	write_cmos_sensor(0x0901, 0x11);
	write_cmos_sensor(0x0902, 0x00);
	write_cmos_sensor(0x3010, 0x75);
	write_cmos_sensor(0x3011, 0x01);
	write_cmos_sensor(0x30C0, 0x11);
	write_cmos_sensor(0x300D, 0x00);
	write_cmos_sensor(0x30FD, 0x01);
	write_cmos_sensor(0x8493, 0x00);
	write_cmos_sensor(0x8863, 0x02);
	write_cmos_sensor(0x90D7, 0x1E);
	/*************output size setting***************/
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x034C, 0x09);
	write_cmos_sensor(0x034D, 0x18);
	write_cmos_sensor(0x034E, 0x06);
	write_cmos_sensor(0x034F, 0xD4);
	write_cmos_sensor(0x0401, 0x00);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x10);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x00);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x09);
	write_cmos_sensor(0x040D, 0x18);
	write_cmos_sensor(0x040E, 0x06);
	write_cmos_sensor(0x040F, 0xD4);
	/************clock setting**************/
	write_cmos_sensor(0x0301, 0x05);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0x7D);
	write_cmos_sensor(0x0309, 0x0A);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x0F);
	write_cmos_sensor(0x030E, 0x03);
	write_cmos_sensor(0x030F, 0x41);
	write_cmos_sensor(0x0310, 0x00);
	/************data rate setting*************/
	write_cmos_sensor(0x0820, 0x0B);
	write_cmos_sensor(0x0821, 0xB8);
	write_cmos_sensor(0x0822, 0x00);
	write_cmos_sensor(0x0823, 0x00);
	/************integration time setting*******/
	write_cmos_sensor(0x0202, 0x07);
	write_cmos_sensor(0x0203, 0x04);
	write_cmos_sensor(0x0224, 0x00);
	write_cmos_sensor(0x0225, 0x70);
	/************integration time setting*******/
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x0216, 0x00);
	write_cmos_sensor(0x0217, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	write_cmos_sensor(0x0210, 0x01);
	write_cmos_sensor(0x0211, 0x00);
	write_cmos_sensor(0x0212, 0x01);
	write_cmos_sensor(0x0213, 0x00);
	write_cmos_sensor(0x0214, 0x01);
	write_cmos_sensor(0x0215, 0x00);
	/************NR setting*******/
	write_cmos_sensor(0x3058, 0x00);
	/************pd setting*******/
	write_cmos_sensor(0x3103, 0x01);	/* PD_OUT_EN */
	write_cmos_sensor(0x3165, 0x01);	/* 0:16*12 window,1:8*6window;2:flexible */
	write_cmos_sensor(0x3166, 0x01);	/* flexible window mode */
	/************flexible window*************/
	/* write_cmos_sensor(0x3100,0x01);//x_start */
	/* write_cmos_sensor(0x3101,0x01); */
	/* write_cmos_sensor(0x3102,0x01);//y_start */
	/* write_cmos_sensor(0x3103,0x01); */
	/* write_cmos_sensor(0x3104,0x01);//x_end */
	/* write_cmos_sensor(0x3105,0x01); */
	/* write_cmos_sensor(0x3106,0x01);//x_end */
	/* write_cmos_sensor(0x3107,0x01); */
	/************PD 8*6 window*************/
	write_cmos_sensor(0x3108, 0x00);	/* x_start */
	write_cmos_sensor(0x3109, 0x2C);
	write_cmos_sensor(0x310a, 0x00);	/* y_start */
	write_cmos_sensor(0x310b, 0x24);
	write_cmos_sensor(0x310c, 0x01);	/* x_end */
	write_cmos_sensor(0x310d, 0x18);
	write_cmos_sensor(0x310e, 0x01);	/* x_end */
	write_cmos_sensor(0x310f, 0x16);

	mdelay(50);
}				/*    preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);


	/*************mode setting************/
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0220, 0x00);
	write_cmos_sensor(0x0221, 0x11);
	write_cmos_sensor(0x0222, 0x10);
	write_cmos_sensor(0x0340, 0x0E);
	write_cmos_sensor(0x0341, 0x18);
	write_cmos_sensor(0x0342, 0x15);
	write_cmos_sensor(0x0343, 0xA0);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x00);
	write_cmos_sensor(0x0347, 0x00);
	write_cmos_sensor(0x0348, 0x12);
	write_cmos_sensor(0x0349, 0x2F);
	write_cmos_sensor(0x034A, 0x0D);
	write_cmos_sensor(0x034B, 0xA7);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x00);
	write_cmos_sensor(0x0901, 0x11);
	write_cmos_sensor(0x0902, 0x00);
	write_cmos_sensor(0x3010, 0x65);
	write_cmos_sensor(0x3011, 0x01);
	write_cmos_sensor(0x30C0, 0x11);
	write_cmos_sensor(0x300D, 0x00);
	write_cmos_sensor(0x30FD, 0x00);
	write_cmos_sensor(0x8493, 0x00);
	write_cmos_sensor(0x8863, 0x00);
	write_cmos_sensor(0x90D7, 0x19);
	/************output size setting**********/
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x034C, 0x12);
	write_cmos_sensor(0x034D, 0x30);
	write_cmos_sensor(0x034E, 0x0D);
	write_cmos_sensor(0x034F, 0xA8);
	write_cmos_sensor(0x0401, 0x00);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x10);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x00);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x12);
	write_cmos_sensor(0x040D, 0x30);
	write_cmos_sensor(0x040E, 0x0D);
	write_cmos_sensor(0x040F, 0xA8);
	/***********clock setting************/
	write_cmos_sensor(0x0301, 0x05);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0xFA);
	write_cmos_sensor(0x0309, 0x0A);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x0F);
	write_cmos_sensor(0x030E, 0x03);
	write_cmos_sensor(0x030F, 0x41);
	write_cmos_sensor(0x0310, 0x00);
	/*************data rateing setting*********/
	write_cmos_sensor(0x0820, 0x17);
	write_cmos_sensor(0x0821, 0x70);
	write_cmos_sensor(0x0822, 0x00);
	write_cmos_sensor(0x0823, 0x00);
	/*************integration time setting******/
	write_cmos_sensor(0x0202, 0x0E);
	write_cmos_sensor(0x0203, 0x0E);
	write_cmos_sensor(0x0224, 0x01);
	write_cmos_sensor(0x0225, 0xF4);
	/************gain setting****************/
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x0216, 0x00);
	write_cmos_sensor(0x0217, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	write_cmos_sensor(0x0210, 0x01);
	write_cmos_sensor(0x0211, 0x00);
	write_cmos_sensor(0x0212, 0x01);
	write_cmos_sensor(0x0213, 0x00);
	write_cmos_sensor(0x0214, 0x01);
	write_cmos_sensor(0x0215, 0x00);
	/*******NR Setting*****************/
	write_cmos_sensor(0x3058, 0x00);

}

static void capture_setting_pdaf(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);
	if (currefps == 150) {

		/*************mode setting************/
		write_cmos_sensor(0x0114, 0x03);
		write_cmos_sensor(0x0220, 0x00);
		write_cmos_sensor(0x0221, 0x11);
		write_cmos_sensor(0x0222, 0x10);
		write_cmos_sensor(0x302b, 0x00);
		write_cmos_sensor(0x0340, 0x0E);
		write_cmos_sensor(0x0341, 0x1C);
		write_cmos_sensor(0x0342, 0x15);
		write_cmos_sensor(0x0343, 0xA0);
		write_cmos_sensor(0x0344, 0x00);
		write_cmos_sensor(0x0345, 0x00);
		write_cmos_sensor(0x0346, 0x00);
		write_cmos_sensor(0x0347, 0x00);
		write_cmos_sensor(0x0348, 0x12);
		write_cmos_sensor(0x0349, 0x2F);
		write_cmos_sensor(0x034A, 0x0D);
		write_cmos_sensor(0x034B, 0xA7);
		write_cmos_sensor(0x0381, 0x01);
		write_cmos_sensor(0x0383, 0x01);
		write_cmos_sensor(0x0385, 0x01);
		write_cmos_sensor(0x0387, 0x01);
		write_cmos_sensor(0x0900, 0x00);
		write_cmos_sensor(0x0901, 0x11);
		write_cmos_sensor(0x0902, 0x00);
		write_cmos_sensor(0x3010, 0x65);
		write_cmos_sensor(0x3011, 0x01);
		write_cmos_sensor(0x30C0, 0x11);
		write_cmos_sensor(0x300D, 0x00);
		write_cmos_sensor(0x30FD, 0x00);
		write_cmos_sensor(0x8493, 0x00);
		write_cmos_sensor(0x8863, 0x00);
		write_cmos_sensor(0x90D7, 0x19);
		/************output size setting**********/
		write_cmos_sensor(0x0112, 0x0A);
		write_cmos_sensor(0x0113, 0x0A);
		write_cmos_sensor(0x034C, 0x12);
		write_cmos_sensor(0x034D, 0x30);
		write_cmos_sensor(0x034E, 0x0D);
		write_cmos_sensor(0x034F, 0xA8);
		write_cmos_sensor(0x0401, 0x00);
		write_cmos_sensor(0x0404, 0x00);
		write_cmos_sensor(0x0405, 0x10);
		write_cmos_sensor(0x0408, 0x00);
		write_cmos_sensor(0x0409, 0x00);
		write_cmos_sensor(0x040A, 0x00);
		write_cmos_sensor(0x040B, 0x00);
		write_cmos_sensor(0x040C, 0x12);
		write_cmos_sensor(0x040D, 0x30);
		write_cmos_sensor(0x040E, 0x0D);
		write_cmos_sensor(0x040F, 0xA8);
		/***********clock setting************/
		write_cmos_sensor(0x0301, 0x05);
		write_cmos_sensor(0x0303, 0x02);
		write_cmos_sensor(0x0305, 0x04);
		write_cmos_sensor(0x0306, 0x00);
		write_cmos_sensor(0x0307, 0x7D);
		write_cmos_sensor(0x0309, 0x0A);
		write_cmos_sensor(0x030B, 0x01);
		write_cmos_sensor(0x030D, 0x0F);
		write_cmos_sensor(0x030E, 0x03);
		write_cmos_sensor(0x030F, 0x41);
		write_cmos_sensor(0x0310, 0x00);
		/*************data rateing setting*********/
		write_cmos_sensor(0x0820, 0x0B);
		write_cmos_sensor(0x0821, 0xB8);
		write_cmos_sensor(0x0822, 0x00);
		write_cmos_sensor(0x0823, 0x00);
		/*************integration time setting******/
		write_cmos_sensor(0x0202, 0x0E);
		write_cmos_sensor(0x0203, 0x12);
		write_cmos_sensor(0x0224, 0x01);
		write_cmos_sensor(0x0225, 0xF4);
		/************gain setting****************/
		write_cmos_sensor(0x0204, 0x00);
		write_cmos_sensor(0x0205, 0x00);
		write_cmos_sensor(0x0216, 0x00);
		write_cmos_sensor(0x0217, 0x00);
		write_cmos_sensor(0x020E, 0x01);
		write_cmos_sensor(0x020F, 0x00);
		write_cmos_sensor(0x0210, 0x01);
		write_cmos_sensor(0x0211, 0x00);
		write_cmos_sensor(0x0212, 0x01);
		write_cmos_sensor(0x0213, 0x00);
		write_cmos_sensor(0x0214, 0x01);
		write_cmos_sensor(0x0215, 0x00);
		/*******NR Setting*****************/
		write_cmos_sensor(0x3058, 0x00);
		/***********PD setting************/
		write_cmos_sensor(0x3103, 0x01);	/* PD_OUT_EN */
		write_cmos_sensor(0x3165, 0x01);	/* 0:16*12 window,1:8*6window;2:flexible */
		write_cmos_sensor(0x3166, 0x01);	/* flexible window mode */
		/************flexible window*************/
		/* write_cmos_sensor(0x3100,0x01);//x_start */
		/* write_cmos_sensor(0x3101,0x01); */
		/* write_cmos_sensor(0x3102,0x01);//y_start */
		/* write_cmos_sensor(0x3103,0x01); */
		/* write_cmos_sensor(0x3104,0x01);//x_end */
		/* write_cmos_sensor(0x3105,0x01); */
		/* write_cmos_sensor(0x3106,0x01);//x_end */
		/* write_cmos_sensor(0x3107,0x01); */
		/************PD 8*6 window*************/
		write_cmos_sensor(0x3108, 0x00);	/* x_start */
		write_cmos_sensor(0x3109, 0x58);
		write_cmos_sensor(0x310a, 0x00);	/* y_start */
		write_cmos_sensor(0x310b, 0x48);
		write_cmos_sensor(0x310c, 0x02);	/* x_end */
		write_cmos_sensor(0x310d, 0x30);
		write_cmos_sensor(0x310e, 0x02);	/* y_end */
		write_cmos_sensor(0x310f, 0x2c);

	} else {

		/*************mode setting************/
		write_cmos_sensor(0x0114, 0x03);
		write_cmos_sensor(0x0220, 0x00);
		write_cmos_sensor(0x0221, 0x11);
		write_cmos_sensor(0x0222, 0x10);
		write_cmos_sensor(0x302b, 0x00);
		write_cmos_sensor(0x0340, 0x0E);
		write_cmos_sensor(0x0341, 0x1C);
		write_cmos_sensor(0x0342, 0x15);
		write_cmos_sensor(0x0343, 0xA0);
		write_cmos_sensor(0x0344, 0x00);
		write_cmos_sensor(0x0345, 0x00);
		write_cmos_sensor(0x0346, 0x00);
		write_cmos_sensor(0x0347, 0x00);
		write_cmos_sensor(0x0348, 0x12);
		write_cmos_sensor(0x0349, 0x2F);
		write_cmos_sensor(0x034A, 0x0D);
		write_cmos_sensor(0x034B, 0xA7);
		write_cmos_sensor(0x0381, 0x01);
		write_cmos_sensor(0x0383, 0x01);
		write_cmos_sensor(0x0385, 0x01);
		write_cmos_sensor(0x0387, 0x01);
		write_cmos_sensor(0x0900, 0x00);
		write_cmos_sensor(0x0901, 0x11);
		write_cmos_sensor(0x0902, 0x00);
		write_cmos_sensor(0x3010, 0x65);
		write_cmos_sensor(0x3011, 0x01);
		write_cmos_sensor(0x30C0, 0x11);
		write_cmos_sensor(0x300D, 0x00);
		write_cmos_sensor(0x30FD, 0x00);
		write_cmos_sensor(0x8493, 0x00);
		write_cmos_sensor(0x8863, 0x00);
		write_cmos_sensor(0x90D7, 0x19);
		/************output size setting**********/
		write_cmos_sensor(0x0112, 0x0A);
		write_cmos_sensor(0x0113, 0x0A);
		write_cmos_sensor(0x034C, 0x12);
		write_cmos_sensor(0x034D, 0x30);
		write_cmos_sensor(0x034E, 0x0D);
		write_cmos_sensor(0x034F, 0xA8);
		write_cmos_sensor(0x0401, 0x00);
		write_cmos_sensor(0x0404, 0x00);
		write_cmos_sensor(0x0405, 0x10);
		write_cmos_sensor(0x0408, 0x00);
		write_cmos_sensor(0x0409, 0x00);
		write_cmos_sensor(0x040A, 0x00);
		write_cmos_sensor(0x040B, 0x00);
		write_cmos_sensor(0x040C, 0x12);
		write_cmos_sensor(0x040D, 0x30);
		write_cmos_sensor(0x040E, 0x0D);
		write_cmos_sensor(0x040F, 0xA8);
		/***********clock setting************/
		write_cmos_sensor(0x0301, 0x05);
		write_cmos_sensor(0x0303, 0x02);
		write_cmos_sensor(0x0305, 0x04);
		write_cmos_sensor(0x0306, 0x00);
		write_cmos_sensor(0x0307, 0xFA);
		write_cmos_sensor(0x0309, 0x0A);
		write_cmos_sensor(0x030B, 0x01);
		write_cmos_sensor(0x030D, 0x0F);
		write_cmos_sensor(0x030E, 0x03);
		write_cmos_sensor(0x030F, 0x41);
		write_cmos_sensor(0x0310, 0x00);
		/*************data rateing setting*********/
		write_cmos_sensor(0x0820, 0x17);
		write_cmos_sensor(0x0821, 0x70);
		write_cmos_sensor(0x0822, 0x00);
		write_cmos_sensor(0x0823, 0x00);
		/*************integration time setting******/
		write_cmos_sensor(0x0202, 0x0E);
		write_cmos_sensor(0x0203, 0x12);
		write_cmos_sensor(0x0224, 0x01);
		write_cmos_sensor(0x0225, 0xF4);
		/************gain setting****************/
		write_cmos_sensor(0x0204, 0x00);
		write_cmos_sensor(0x0205, 0x00);
		write_cmos_sensor(0x0216, 0x00);
		write_cmos_sensor(0x0217, 0x00);
		write_cmos_sensor(0x020E, 0x01);
		write_cmos_sensor(0x020F, 0x00);
		write_cmos_sensor(0x0210, 0x01);
		write_cmos_sensor(0x0211, 0x00);
		write_cmos_sensor(0x0212, 0x01);
		write_cmos_sensor(0x0213, 0x00);
		write_cmos_sensor(0x0214, 0x01);
		write_cmos_sensor(0x0215, 0x00);
		/*******NR Setting*****************/
		write_cmos_sensor(0x3058, 0x00);
		/***********PD setting************/
		write_cmos_sensor(0x3103, 0x01);	/* PD_OUT_EN */
		write_cmos_sensor(0x3165, 0x01);	/* 0:16*12 window,1:8*6window;2:flexible */
		write_cmos_sensor(0x3166, 0x01);	/* flexible window mode */
		/************flexible window*************/
		/* write_cmos_sensor(0x3100,0x01);//x_start */
		/* write_cmos_sensor(0x3101,0x01); */
		/* write_cmos_sensor(0x3102,0x01);//y_start */
		/* write_cmos_sensor(0x3103,0x01); */
		/* write_cmos_sensor(0x3104,0x01);//x_end */
		/* write_cmos_sensor(0x3105,0x01); */
		/* write_cmos_sensor(0x3106,0x01);//x_end */
		/* write_cmos_sensor(0x3107,0x01); */
		/************PD 8*6 window*************/
		write_cmos_sensor(0x3108, 0x00);	/* x_start */
		write_cmos_sensor(0x3109, 0x58);
		write_cmos_sensor(0x310a, 0x00);	/* y_start */
		write_cmos_sensor(0x310b, 0x48);
		write_cmos_sensor(0x310c, 0x02);	/* x_end */
		write_cmos_sensor(0x310d, 0x30);
		write_cmos_sensor(0x310e, 0x02);	/* y_end */
		write_cmos_sensor(0x310f, 0x2c);

	}
}


static void normal_video_setting(kal_uint16 currefps)
{


	/*************mode setting************/
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0220, 0x00);
	write_cmos_sensor(0x0221, 0x11);
	write_cmos_sensor(0x0222, 0x10);
	write_cmos_sensor(0x0340, 0x0B);
	write_cmos_sensor(0x0341, 0x48);
	write_cmos_sensor(0x0342, 0x15);
	write_cmos_sensor(0x0343, 0xA0);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x01);
	write_cmos_sensor(0x0347, 0xBC);
	write_cmos_sensor(0x0348, 0x12);
	write_cmos_sensor(0x0349, 0x2F);
	write_cmos_sensor(0x034A, 0x0B);
	write_cmos_sensor(0x034B, 0xEB);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x00);
	write_cmos_sensor(0x0901, 0x11);
	write_cmos_sensor(0x0902, 0x00);
	write_cmos_sensor(0x3010, 0x65);
	write_cmos_sensor(0x3011, 0x01);
	write_cmos_sensor(0x30C0, 0x11);
	write_cmos_sensor(0x300D, 0x00);
	write_cmos_sensor(0x30FD, 0x00);
	write_cmos_sensor(0x8493, 0x00);
	write_cmos_sensor(0x8863, 0x00);
	write_cmos_sensor(0x90D7, 0x19);
	/************output size setting**********/
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x034C, 0x12);
	write_cmos_sensor(0x034D, 0x30);
	write_cmos_sensor(0x034E, 0x0A);
	write_cmos_sensor(0x034F, 0x30);
	write_cmos_sensor(0x0401, 0x00);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x10);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x00);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x12);
	write_cmos_sensor(0x040D, 0x30);
	write_cmos_sensor(0x040E, 0x0A);
	write_cmos_sensor(0x040F, 0x30);
	/***********clock setting************/
	write_cmos_sensor(0x0301, 0x05);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0xC8);
	write_cmos_sensor(0x0309, 0x0A);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x0F);
	write_cmos_sensor(0x030E, 0x03);
	write_cmos_sensor(0x030F, 0x41);
	write_cmos_sensor(0x0310, 0x00);
	/*************data rateing setting*********/
	write_cmos_sensor(0x0820, 0x12);
	write_cmos_sensor(0x0821, 0xC0);
	write_cmos_sensor(0x0822, 0x00);
	write_cmos_sensor(0x0823, 0x00);
	/*************integration time setting******/
	write_cmos_sensor(0x0202, 0x0B);
	write_cmos_sensor(0x0203, 0x3E);
	write_cmos_sensor(0x0224, 0x01);
	write_cmos_sensor(0x0225, 0xF4);
	/************gain setting****************/
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x0216, 0x00);
	write_cmos_sensor(0x0217, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	write_cmos_sensor(0x0210, 0x01);
	write_cmos_sensor(0x0211, 0x00);
	write_cmos_sensor(0x0212, 0x01);
	write_cmos_sensor(0x0213, 0x00);
	write_cmos_sensor(0x0214, 0x01);
	write_cmos_sensor(0x0215, 0x00);
	/*******NR Setting*****************/
	write_cmos_sensor(0x3058, 0x00);


}

static void hs_video_setting(void)
{

	/*************mode setting******/
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0220, 0x00);
	write_cmos_sensor(0x0221, 0x11);
	write_cmos_sensor(0x0222, 0x10);
	write_cmos_sensor(0x0340, 0x03);
	write_cmos_sensor(0x0341, 0x86);
	write_cmos_sensor(0x0342, 0x15);
	write_cmos_sensor(0x0343, 0xA0);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x01);
	write_cmos_sensor(0x0347, 0xEC);
	write_cmos_sensor(0x0348, 0x12);
	write_cmos_sensor(0x0349, 0x2F);
	write_cmos_sensor(0x034A, 0x0B);
	write_cmos_sensor(0x034B, 0xAF);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x03);
	write_cmos_sensor(0x0387, 0x03);
	write_cmos_sensor(0x0900, 0x01);
	write_cmos_sensor(0x0901, 0x22);
	write_cmos_sensor(0x0902, 0x00);
	write_cmos_sensor(0x3010, 0x65);
	write_cmos_sensor(0x3011, 0x01);
	write_cmos_sensor(0x30C0, 0x11);
	write_cmos_sensor(0x300D, 0x01);
	write_cmos_sensor(0x30FD, 0x00);
	write_cmos_sensor(0x8493, 0x00);
	write_cmos_sensor(0x8863, 0x00);
	write_cmos_sensor(0x90D7, 0x19);
	/************output size setting***********/
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x034C, 0x05);
	write_cmos_sensor(0x034D, 0xC4);
	write_cmos_sensor(0x034E, 0x03);
	write_cmos_sensor(0x034F, 0x42);
	write_cmos_sensor(0x0401, 0x01);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x18);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x38);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x08);
	write_cmos_sensor(0x040D, 0xA8);
	write_cmos_sensor(0x040E, 0x03);
	write_cmos_sensor(0x040F, 0x42);
	/***********clock setting**************/
	write_cmos_sensor(0x0301, 0x05);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0xFA);
	write_cmos_sensor(0x0309, 0x0A);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x0F);
	write_cmos_sensor(0x030E, 0x03);
	write_cmos_sensor(0x030F, 0x41);
	write_cmos_sensor(0x0310, 0x00);
	/*************data rateing setting**********/
	write_cmos_sensor(0x0820, 0x17);
	write_cmos_sensor(0x0821, 0x70);
	write_cmos_sensor(0x0822, 0x00);
	write_cmos_sensor(0x0823, 0x00);
	/*************integration time **/
	write_cmos_sensor(0x0202, 0x03);
	write_cmos_sensor(0x0203, 0x7C);
	write_cmos_sensor(0x0224, 0x01);
	write_cmos_sensor(0x0225, 0xF4);
	/************gain setting******* **/
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x0216, 0x00);
	write_cmos_sensor(0x0217, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	write_cmos_sensor(0x0210, 0x01);
	write_cmos_sensor(0x0211, 0x00);
	write_cmos_sensor(0x0212, 0x01);
	write_cmos_sensor(0x0213, 0x00);
	write_cmos_sensor(0x0214, 0x01);
	write_cmos_sensor(0x0215, 0x00);
	/*******NR Setting****************/
	write_cmos_sensor(0x3058, 0x00);


}

static void slim_video_setting(void)
{


	/*************mode setting*********/
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0220, 0x00);
	write_cmos_sensor(0x0221, 0x11);
	write_cmos_sensor(0x0222, 0x10);
	write_cmos_sensor(0x0340, 0x05);
	write_cmos_sensor(0x0341, 0xA4);
	write_cmos_sensor(0x0342, 0x15);
	write_cmos_sensor(0x0343, 0xA0);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x01);
	write_cmos_sensor(0x0347, 0xEC);
	write_cmos_sensor(0x0348, 0x12);
	write_cmos_sensor(0x0349, 0x2F);
	write_cmos_sensor(0x034A, 0x0B);
	write_cmos_sensor(0x034B, 0xAF);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x03);
	write_cmos_sensor(0x0387, 0x03);
	write_cmos_sensor(0x0900, 0x01);
	write_cmos_sensor(0x0901, 0x22);
	write_cmos_sensor(0x0902, 0x00);
	write_cmos_sensor(0x3010, 0x65);
	write_cmos_sensor(0x3011, 0x01);
	write_cmos_sensor(0x30C0, 0x11);
	write_cmos_sensor(0x300D, 0x01);
	write_cmos_sensor(0x30FD, 0x00);
	write_cmos_sensor(0x8493, 0x00);
	write_cmos_sensor(0x8863, 0x00);
	write_cmos_sensor(0x90D7, 0x19);
	/************output size setting*********/
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x034C, 0x05);
	write_cmos_sensor(0x034D, 0xC4);
	write_cmos_sensor(0x034E, 0x03);
	write_cmos_sensor(0x034F, 0x42);
	write_cmos_sensor(0x0401, 0x01);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x18);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x38);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x08);
	write_cmos_sensor(0x040D, 0xA8);
	write_cmos_sensor(0x040E, 0x03);
	write_cmos_sensor(0x040F, 0x42);
	/***********clock setting****************/
	write_cmos_sensor(0x0301, 0x05);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0x64);
	write_cmos_sensor(0x0309, 0x0A);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x0F);
	write_cmos_sensor(0x030E, 0x03);
	write_cmos_sensor(0x030F, 0x41);
	write_cmos_sensor(0x0310, 0x00);
	/*************data rateing setting*********/
	write_cmos_sensor(0x0820, 0x09);
	write_cmos_sensor(0x0821, 0x60);
	write_cmos_sensor(0x0822, 0x00);
	write_cmos_sensor(0x0823, 0x00);
	/*************integration time setting*********/
	write_cmos_sensor(0x0202, 0x05);
	write_cmos_sensor(0x0203, 0x9A);
	write_cmos_sensor(0x0224, 0x01);
	write_cmos_sensor(0x0225, 0xF4);
	/************gain setting*********/
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x0216, 0x00);
	write_cmos_sensor(0x0217, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	write_cmos_sensor(0x0210, 0x01);
	write_cmos_sensor(0x0211, 0x00);
	write_cmos_sensor(0x0212, 0x01);
	write_cmos_sensor(0x0213, 0x00);
	write_cmos_sensor(0x0214, 0x01);
	write_cmos_sensor(0x0215, 0x00);
	/*******NR setting*********/
	write_cmos_sensor(0x3058, 0x00);


}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);
	if (enable)
		write_cmos_sensor(0x0601, 0x02);
	else
		write_cmos_sensor(0x0601, 0x00);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 *    get_imgsensor_id
 *
 * DESCRIPTION
 *    This function get the sensor ID
 *
 * PARAMETERS
 *    *sensorID : return the sensor ID
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n",
				imgsensor.i2c_write_id, *sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *    open
 *
 * DESCRIPTION
 *    This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *    None
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;

	LOG_1;
	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n",
				imgsensor.i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;
	/* initail sequence write in  */
	sensor_init();
	imx398_apply_SPC();
	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_mode = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}				/*    open  */



/*************************************************************************
 * FUNCTION
 *    close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *    None
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function */

	return ERROR_NONE;
}				/*    close  */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *    This function start the sensor preview.
 *
 * PARAMETERS
 *    *image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	/* imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	if (imgsensor.ihdr_mode == 2)
		preview_setting_HDR_ES2();	/*HDR + PDAF */
	else
		preview_setting();	/*PDAF only */
	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */
	return ERROR_NONE;
}				/*    preview   */

/*************************************************************************
 * FUNCTION
 *    capture
 *
 * DESCRIPTION
 *    This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
		/* PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M */
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
				imgsensor.current_fps, imgsensor_info.cap.max_framerate / 10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	if (imgsensor.pdaf_mode == 1)
		capture_setting_pdaf(imgsensor.current_fps);	/*PDAF only */
	else
		capture_setting(imgsensor.current_fps);	/*Full mode */

	return ERROR_NONE;
}				/* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			       MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */
	return ERROR_NONE;
}				/*    normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */
	return ERROR_NONE;
}				/*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			     MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */

	return ERROR_NONE;
}				/*    slim_video     */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}				/*    get_resolution    */

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);


	/* sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; // not use */
	/* sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; // not use */
	/* imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; // not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;	/* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;	/* inverse with datasheet */
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;	/* not use */
	sensor_info->SensorResetActiveHigh = FALSE;	/* not use */
	sensor_info->SensorResetDelayCount = 5;	/* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0;	/* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;	/* The frame of setting shutter
										 * default 0 for TG int
										 */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting
												 * sensor gain
												 */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = PDAF_SUPPORT_RAW;	/*0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode */

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;	/* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;	/* not use */
	sensor_info->SensorPixelClockCount = 3;	/* not use */
	sensor_info->SensorDataLatchCount = 2;	/* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

		sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}				/*    get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}				/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{				/* This Function not used after ROME */
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable)		/* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else			/* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id,
						MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/* set_dummy(); */
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk / framerate * 10 /
			imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength)
			? (frame_length - imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/* set_dummy(); */
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength)
				? (frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
				LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
					framerate, imgsensor_info.cap.max_framerate / 10);
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength)
				? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		/* set_dummy(); */
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength)
			? (frame_length - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/* set_dummy(); */
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength)
			? (frame_length - imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/* set_dummy(); */
		break;
	default:		/* coding with  preview scenario by default */
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/* set_dummy(); */
		LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id,
						    MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}


static kal_uint32 imx398_awb_gain(SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
	UINT32 rgain_32, grgain_32, gbgain_32, bgain_32;

	LOG_INF("imx398_awb_gain\n");
	grgain_32 = (pSetSensorAWB->ABS_GAIN_GR << 8) >> 9;
	rgain_32 = (pSetSensorAWB->ABS_GAIN_R << 8) >> 9;
	bgain_32 = (pSetSensorAWB->ABS_GAIN_B << 8) >> 9;
	gbgain_32 = (pSetSensorAWB->ABS_GAIN_GB << 8) >> 9;

	LOG_INF("[imx398_awb_gain] ABS_GAIN_GR:%d, grgain_32:%d\n", pSetSensorAWB->ABS_GAIN_GR,
		grgain_32);
	LOG_INF("[imx398_awb_gain] ABS_GAIN_R:%d, rgain_32:%d\n", pSetSensorAWB->ABS_GAIN_R,
		rgain_32);
	LOG_INF("[imx398_awb_gain] ABS_GAIN_B:%d, bgain_32:%d\n", pSetSensorAWB->ABS_GAIN_B,
		bgain_32);
	LOG_INF("[imx398_awb_gain] ABS_GAIN_GB:%d, gbgain_32:%d\n", pSetSensorAWB->ABS_GAIN_GB,
		gbgain_32);

	write_cmos_sensor(0x0b8e, (grgain_32 >> 8) & 0xFF);
	write_cmos_sensor(0x0b8f, grgain_32 & 0xFF);
	write_cmos_sensor(0x0b90, (rgain_32 >> 8) & 0xFF);
	write_cmos_sensor(0x0b91, rgain_32 & 0xFF);
	write_cmos_sensor(0x0b92, (bgain_32 >> 8) & 0xFF);
	write_cmos_sensor(0x0b93, bgain_32 & 0xFF);
	write_cmos_sensor(0x0b94, (gbgain_32 >> 8) & 0xFF);
	write_cmos_sensor(0x0b95, gbgain_32 & 0xFF);
	return ERROR_NONE;
}

static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable)
		write_cmos_sensor(0x0100, 0X01);
	else
		write_cmos_sensor(0x0100, 0x00);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	/* unsigned long long *feature_return_data = (unsigned long long*)feature_para; */

	SET_PD_BLOCK_INFO_T *PDAFinfo;

	SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	SENSOR_VC_INFO_STRUCT *pvcinfo;
	SET_SENSOR_AWB_GAIN *pSetSensorAWB = (SET_SENSOR_AWB_GAIN *) feature_para;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL) *feature_data);
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE */
		/* if EEPROM does not exist in camera module. */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data_16);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL) *feature_data_16, *(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM) *feature_data,
					      *(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM) *feature_data,
						  (MUINT32 *) (uintptr_t) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
		read_imx398_DCC((kal_uint16) (*feature_data),
				(char *)(uintptr_t) (*(feature_data + 1)),
				(kal_uint32) (*(feature_data + 2)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL) *feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:	/* for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32) *feature_data);
		wininfo = (SENSOR_WINSIZE_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%llu\n", *feature_data);
		PDAFinfo = (SET_PD_BLOCK_INFO_T *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info,
			       sizeof(SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			break;
		}
		break;
		/*HDR CMD */
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_mode = (UINT8)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n", (UINT16) *feature_data,
			(UINT16) *(feature_data + 1), (UINT16) *(feature_data + 2));
		ihdr_write_shutter_gain(*feature_data, *(feature_data + 1), *(feature_data + 2));
		break;
	case SENSOR_FEATURE_GET_VC_INFO:
		LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16) *feature_data);
		pvcinfo = (SENSOR_VC_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],
			       sizeof(SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2],
			       sizeof(SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
			       sizeof(SENSOR_VC_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		imx398_awb_gain(pSetSensorAWB);
		break;
		/*END OF HDR CMD */
		/*PDAF CMD */
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%llu\n", *feature_data);
		/* PDAF capacity enable or not, 2p8 only full size support PDAF */
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 1;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		default:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_SET_PDAF:
		LOG_INF("PDAF mode :%d\n", *feature_data_16);
		imgsensor.pdaf_mode = *feature_data_16;
		break;
		/*End of PDAF */
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;

	default:
		break;
	}

	return ERROR_NONE;
}				/*    feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 IMX398_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}				/*    IMX398_MIPI_RAW_SensorInit    */
