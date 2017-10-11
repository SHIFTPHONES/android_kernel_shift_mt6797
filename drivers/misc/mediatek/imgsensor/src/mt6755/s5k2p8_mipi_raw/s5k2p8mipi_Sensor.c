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

/*****************************************************************************
 *
 * Filename:
 * ---------
 *   S5K2P8mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
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
#include <asm/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k2p8mipi_Sensor.h"

#define PFX "S5K2P8_camera_sensor"
#define LOG_1 LOG_INF("S5K2P8,MIPI 4LANE\n")
#define LOG_2 LOG_INF("preview 2664*1500@30fps,888Mbps/lane; video 5328*3000@30fps,1390Mbps/lane; capture 16M@30fps,1390Mbps/lane\n")
#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
#define LOGE(format, args...)   pr_err(PFX "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);

#define SLOW_MOTION_120FPS
#define FIX_VIEW_ANGLE
#define MULTI_WRITE 0

extern bool read_eeprom( kal_uint16 addr, BYTE* data, kal_uint32 size);

//#define PDAF_TEST 1
#ifdef PDAF_TEST
extern bool wrtie_eeprom(kal_uint16 addr, BYTE data[],kal_uint32 size );
char data[4096]= {0};
char data2[4096]= {0};
#endif
static imgsensor_info_struct imgsensor_info = {
	.sensor_id = S5K2P8_SENSOR_ID,
#ifdef FIX_VIEW_ANGLE
	.checksum_value = 0xfe9e1a79,
#else
	.checksum_value = 0x33c8c938,
#endif

	.pre = {
		.pclk = 371200000,				//record different mode's pclk
		.linelength = 5880,				//record different mode's linelength
		.framelength = 2052,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
#ifdef FIX_VIEW_ANGLE
		.grabwindow_width = 2656,		//record different mode's width of grabwindow
		.grabwindow_height = 1488,		//record different mode's height of grabwindow
#else
		.grabwindow_width = 2664,		//record different mode's width of grabwindow
		.grabwindow_height = 1500,		//record different mode's height of grabwindow
#endif
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 560000000,
		.linelength =5880,
		.framelength = 3174,
		.startx = 0,
		.starty = 0,
#ifdef FIX_VIEW_ANGLE
		.grabwindow_width = 5312,
		.grabwindow_height = 2976,
#else
		.grabwindow_width = 5328,//5334,
		.grabwindow_height = 3000,
#endif
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
		},
	.cap1 = {
		.pclk = 371200000,
		.linelength = 5880,
		.framelength = 3174,
		.startx = 0,
		.starty = 0,
#ifdef FIX_VIEW_ANGLE
		.grabwindow_width = 5312,
		.grabwindow_height = 2976,
#else
		.grabwindow_width =5328,
		.grabwindow_height = 3000,
#endif
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 200,
	},
    .cap2 = {
        .pclk = 560000000,
		.linelength = 12224,
		.framelength = 3174,
		.startx = 0,
		.starty = 0,
#ifdef FIX_VIEW_ANGLE
		.grabwindow_width = 5312,
		.grabwindow_height = 2976,
#else
		.grabwindow_width = 5328,//5334,
		.grabwindow_height = 3000,
#endif
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 150,
        },
	.normal_video = {
		.pclk = 560000000,
		.linelength = 5880,
		.framelength = 3174,
		.startx = 0,
		.starty = 0,
#ifdef FIX_VIEW_ANGLE
		.grabwindow_width = 5312,
		.grabwindow_height = 2976,
#else
		.grabwindow_width = 5328,//5334,
		.grabwindow_height = 3000,
#endif
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
	},
	.normal_video1 = {
		.pclk = 371200000,
		.linelength = 5880,
		.framelength = 3174,
		.startx = 0,
		.starty = 0,
#ifdef FIX_VIEW_ANGLE
		.grabwindow_width = 5312,
		.grabwindow_height = 2976,
#else
		.grabwindow_width = 5328,
		.grabwindow_height = 3000,
#endif
		.mipi_data_lp2hs_settle_dc = 85,/*unit , ns*/
		.max_framerate = 200,
	},
	.normal_video2 = {
		.pclk = 560000000,
		.linelength = 12224,
		.framelength = 3174,
		.startx = 0,
		.starty = 0,
#ifdef FIX_VIEW_ANGLE
		.grabwindow_width = 5312,
		.grabwindow_height = 2976,
#else
		.grabwindow_width = 5328,/* 5334, */
		.grabwindow_height = 3000,
#endif
		.mipi_data_lp2hs_settle_dc = 85,/* unit , ns */
		.max_framerate = 150,
	},
#ifdef SLOW_MOTION_120FPS
	.hs_video = {
		.pclk = 560000000,
		.linelength = 5880,
		.framelength = 793,//793,
		.startx = 0,
		.starty = 0,
		.grabwindow_width =1328, //1920,
		.grabwindow_height =748,// 1080,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 1200,
	},
#else
	.hs_video = {
		.pclk = 560000000,
		.linelength = 5880,
		.framelength = 1587,
		.startx = 0,
		.starty = 0,
#ifdef FIX_VIEW_ANGLE
		.grabwindow_width = 2656,		//record different mode's width of grabwindow
		.grabwindow_height = 1488,		//record different mode's height of grabwindow
#else
		.grabwindow_width = 2664,		//record different mode's width of grabwindow
		.grabwindow_height = 1500,		//record different mode's height of grabwindow

#endif

		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 600,
	},
#endif
	.slim_video = {
		.pclk = 560000000,
		.linelength = 5880,
		.framelength = 3172,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1328,//1280,
		.grabwindow_height =748,// 720,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
	},
	.custom1= {
		.pclk = 560000000,				//record different mode's pclk
		.linelength = 5880,				//record different mode's linelength
		.framelength = 3168,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
#ifdef FIX_VIEW_ANGLE
		.grabwindow_width = 2656,		//record different mode's width of grabwindow
		.grabwindow_height = 1488,		//record different mode's height of grabwindow
#else
		.grabwindow_width = 2664,		//record different mode's width of grabwindow
		.grabwindow_height = 1500,		//record different mode's height of grabwindow
#endif
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
	},
	.custom2= {
		.pclk = 560000000,				//record different mode's pclk
		.linelength = 5880,				//record different mode's linelength
		.framelength = 3168,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
#ifdef FIX_VIEW_ANGLE
		.grabwindow_width = 2656,		//record different mode's width of grabwindow
		.grabwindow_height = 1488,		//record different mode's height of grabwindow
#else
		.grabwindow_width = 2664,		//record different mode's width of grabwindow
		.grabwindow_height = 1500,		//record different mode's height of grabwindow
#endif
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
	},
	.custom3= {
			.pclk = 560000000,				//record different mode's pclk
			.linelength = 5880, 			//record different mode's linelength
			.framelength = 3168,			//record different mode's framelength
			.startx = 0,					//record different mode's startx of grabwindow
			.starty = 0,					//record different mode's starty of grabwindow
#ifdef FIX_VIEW_ANGLE
			.grabwindow_width = 2656,		//record different mode's width of grabwindow
			.grabwindow_height = 1488,		//record different mode's height of grabwindow
#else
			.grabwindow_width = 2664,		//record different mode's width of grabwindow
			.grabwindow_height = 1500,		//record different mode's height of grabwindow
#endif
			/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
			.mipi_data_lp2hs_settle_dc = 85,//unit , ns
			/*	 following for GetDefaultFramerateByScenario()	*/
			.max_framerate = 300,
		},
		.custom4= {
			.pclk = 560000000,				//record different mode's pclk
			.linelength = 5880, 			//record different mode's linelength
			.framelength = 3168,			//record different mode's framelength
			.startx = 0,					//record different mode's startx of grabwindow
			.starty = 0,					//record different mode's starty of grabwindow
#ifdef FIX_VIEW_ANGLE
			.grabwindow_width = 2656,		//record different mode's width of grabwindow
			.grabwindow_height = 1488,		//record different mode's height of grabwindow
#else
			.grabwindow_width = 2664,		//record different mode's width of grabwindow
			.grabwindow_height = 1500,		//record different mode's height of grabwindow
#endif
			/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
			.mipi_data_lp2hs_settle_dc = 85,//unit , ns
			/*	 following for GetDefaultFramerateByScenario()	*/
			.max_framerate = 300,
		},
		.custom5= {
			.pclk = 560000000,				//record different mode's pclk
			.linelength = 5880, 			//record different mode's linelength
			.framelength = 3168,			//record different mode's framelength
			.startx = 0,					//record different mode's startx of grabwindow
			.starty = 0,					//record different mode's starty of grabwindow
#ifdef FIX_VIEW_ANGLE
			.grabwindow_width = 2656,		//record different mode's width of grabwindow
			.grabwindow_height = 1488,		//record different mode's height of grabwindow
#else
			.grabwindow_width = 2664,		//record different mode's width of grabwindow
			.grabwindow_height = 1500,		//record different mode's height of grabwindow
#endif
			/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
			.mipi_data_lp2hs_settle_dc = 85,//unit , ns
			/*	 following for GetDefaultFramerateByScenario()	*/
			.max_framerate = 300,
		},

	.margin = 16,
	.min_shutter = 1,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 1,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 10,	  //support sensor mode num

	.cap_delay_frame = 3,
	.pre_delay_frame = 3,
	.video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,
    .custom1_delay_frame = 2,
    .custom2_delay_frame = 2,
    .custom3_delay_frame = 2,
    .custom4_delay_frame = 2,
    .custom5_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_MANUAL,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x20, 0x5A, 0xff},
	.i2c_speed = 300, // i2c read/write speed
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x3D0,					//current shutter
	.gain = 0x100,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = KAL_FALSE, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x20,
};


/* Sensor output window information */
#ifndef FIX_VIEW_ANGLE
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] = {
	{5344, 3008, 16, 8, 5328, 3000, 2664, 1500, 0000, 0000, 2664, 1500, 0, 0, 2664, 1500},	/* Preview */
	{5344, 3008, 16, 8, 5328, 3000, 5328, 3000, 0000, 0000, 5328, 3000, 0, 0, 5328, 3000},	/* capture */
	{5344, 3008, 16, 8, 5328, 3000, 5328, 3000, 0000, 0000, 5328, 3000, 0, 0, 5328, 3000},	/* video */
#ifdef SLOW_MOTION_120FPS
	{5344, 3008, 20, 12, 5312, 2992, 1328, 748, 0000, 0000, 1328, 748, 0, 0, 1328, 748},	/* hight video 120 */
#else
	{5344, 3008, 16, 8, 5328, 3000, 2664, 1500, 0000, 0000, 2664, 1500, 0, 0, 2664, 1500},	/* hight speed video */
#endif
	{5344, 3008, 20, 12, 5312, 2992, 1328, 748, 0000, 0000, 1328, 748, 0, 0, 1328, 748},	/* slim video */
	{5344, 3008, 16, 8, 5328, 3000, 2664, 1500, 0000, 0000, 2664, 1500, 0, 0, 2664, 1500},
	{5344, 3008, 16, 8, 5328, 3000, 2664, 1500, 0000, 0000, 2664, 1500, 0, 0, 2664, 1500},
	{5344, 3008, 16, 8, 5328, 3000, 2664, 1500, 0000, 0000, 2664, 1500, 0, 0, 2664, 1500},
	{5344, 3008, 16, 8, 5328, 3000, 2664, 1500, 0000, 0000, 2664, 1500, 0, 0, 2664, 1500},
	{5344, 3008, 16, 8, 5328, 3000, 2664, 1500, 0000, 0000, 2664, 1500, 0, 0, 2664, 1500}
};
#else
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] = {
	{5344, 3008, 24, 20, 5312, 2976, 2656, 1488, 0000, 0000, 2656, 1488, 0, 0, 2656, 1488},	/* Preview */
	{5344, 3008, 24, 20, 5312, 2976, 5312, 2976, 0000, 0000, 5312, 2976, 0, 0, 5312, 2976},	/* capture */
	{5344, 3008, 24, 20, 5312, 2976, 5312, 2976, 0000, 0000, 5312, 2976, 0, 0, 5312, 2976},	/* video */
#ifdef SLOW_MOTION_120FPS
	{5344, 3008, 20, 12, 5312, 2992, 1328, 748, 0000, 0000, 1328, 748, 0, 0, 1328, 748},	/* hight video 120 */
#else
	{5344, 3008, 24, 20, 5312, 2976, 2656, 1488, 0000, 0000, 2656, 1488, 0, 0, 2656, 1488},	/* hight speed video */
#endif
	{5344, 3008, 20, 12, 5312, 2992, 1328, 748, 0000, 0000, 1328, 748, 0, 0, 1328, 748},	/* slim video */
	{5344, 3008, 24, 20, 5312, 2976, 2656, 1488, 0000, 0000, 2656, 1488, 0, 0, 2656, 1488},
	{5344, 3008, 24, 20, 5312, 2976, 2656, 1488, 0000, 0000, 2656, 1488, 0, 0, 2656, 1488},
	{5344, 3008, 24, 20, 5312, 2976, 2656, 1488, 0000, 0000, 2656, 1488, 0, 0, 2656, 1488},
	{5344, 3008, 24, 20, 5312, 2976, 2656, 1488, 0000, 0000, 2656, 1488, 0, 0, 2656, 1488},
	{5344, 3008, 24, 20, 5312, 2976, 2656, 1488, 0000, 0000, 2656, 1488, 0, 0, 2656, 1488}
};
#endif

static SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
	.i4OffsetX =  8,
	.i4OffsetY = 35,
	.i4PitchX  = 64,
	.i4PitchY  = 64,
	.i4PairNum  =16,
	.i4SubBlkW  =16,
	.i4SubBlkH  =16,
	.i4BlockNumX = 83,
	.i4BlockNumY = 46,
	.i4PosL = {{8,35},{60,35},{24,39},{44,39},{12,55},{56,55},{28,59},{40,59},{28,67},{40,67},{12,71},{56,71},{24,87},{44,87},{8,91},{60,91}},
	.i4PosR = {{8,39},{60,39},{24,43},{44,43},{12,51},{56,51},{28,55},{40,55},{28,71},{40,71},{12,75},{56,75},{24,83},{44,83},{8,87},{60,87}},
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 2, imgsensor.i2c_write_id);
    return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}


static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
    kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}

static int read_cmos_sensor_8(kal_uint16 addr)
{
    kal_int16 get_byte=0;
	int ret = -1;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    if(iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id)==0)
		ret = get_byte;
    return ret;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
    kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    iWriteRegI2C(pusendcmd , 3, imgsensor.i2c_write_id);
}

#if MULTI_WRITE
#define S5K2P8MIPI_table_write_cmos_sensor S5K2P8MIPI_table_write_cmos_sensor_multi
#define I2C_BUFFER_LEN 225
static kal_uint16 S5K2P8MIPI_table_write_cmos_sensor_multi(kal_uint16* para, kal_uint32 len)
{

  // LOG_INF("enter S5K2PP8MIPI_table_write_cmos_sensor_multi len  %d\n",len);

   char puSendCmd[I2C_BUFFER_LEN];
   kal_uint32 tosend, IDX;
   kal_uint16 addr = 0, addr_last = 0, data;

   tosend = 0;
   IDX = 0;

   while(IDX < len)
   {
      addr = para[IDX];

       {
           puSendCmd[tosend++] = (char)(addr >> 8);
           puSendCmd[tosend++] = (char)(addr & 0xFF);
           data = para[IDX+1];
		   puSendCmd[tosend++] = (char)(data >> 8);
           puSendCmd[tosend++] = (char)(data & 0xFF);
           IDX += 2;
           addr_last = addr;

       }

	   if (tosend >= I2C_BUFFER_LEN || IDX == len || addr != addr_last)
       {
       	   //LOG_INF("IDX %d,tosend %d addr_last 0x%x,addr 0x%x\n",IDX, tosend, addr_last, addr);
           iBurstWriteReg_multi(puSendCmd , tosend, imgsensor.i2c_write_id, 4);
           tosend = 0;
       }
   }
   //LOG_INF("exit S5K2P8MIPI_table_write_cmos_sensor_multi\n");

   return 0;
}
#if 0
static kal_uint16 S5K2P8MIPI_table_write_cmos_sensor_burst(kal_uint16* para, kal_uint32 len)
{

	LOG_INF("enter S5K2P8MIPI_table_write_cmos_sensor_burst len  %d\n",len);

   char puSendCmd[I2C_BUFFER_LEN];
   kal_uint32 tosend, IDX, same_to_send;
   kal_uint16 addr = 0, addr_last = 0, data;

   tosend = 0;
   IDX = 0;
   same_to_send = 0;
   while(IDX < len)
   {
      addr = para[IDX];
      if (tosend == 0) // new (addr, data) to send
       {
       LOG_INF("tosend == 0");
           puSendCmd[tosend++] = (char)(addr >> 8);
           puSendCmd[tosend++] = (char)(addr & 0xFF);
           data = para[IDX+1];
           puSendCmd[tosend++] = (char)(data & 0xFF);
           IDX += 2;
           addr_last = addr;

       }
       else if (addr == addr_last+1) // to multiple write the data to sequential register address
       {
           data = para[IDX+1];
           puSendCmd[tosend++] = (char)(data & 0xFF);
		   addr_last = addr;
           IDX += 2;
       }
	   if (tosend >= I2C_BUFFER_LEN || IDX == len || addr != addr_last)// ||same_to_send==1)
       {
       	   LOG_INF("IDX %d,tosend %d addr_last 0x%x,addr 0x%x\n",IDX, tosend, addr_last, addr);
           iBurstWriteReg_multi(puSendCmd , tosend, imgsensor.i2c_write_id, tosend);
           tosend = 0;
		  // same_to_send = 0;
       }
   }
   LOG_INF("exit S5K2P8MIPI_table_write_cmos_sensor_burst\n");

   return 0;
}
#endif
#endif


static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d ", imgsensor.dummy_line, imgsensor.dummy_pixel);
    write_cmos_sensor_8(0x0104, 0x01);
    write_cmos_sensor(0x0340, imgsensor.frame_length);
    write_cmos_sensor(0x0342, imgsensor.line_length);
    write_cmos_sensor_8(0x0104, 0x00);

}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{

	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable = %d\n", framerate,min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
		//imgsensor.dummy_line = 0;
	//else
		//imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */


static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;

    spin_lock(&imgsensor_drv_lock);
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

    if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);
    } else {
        // Extend frame length
        write_cmos_sensor_8(0x0104,0x01);
        write_cmos_sensor(0x0340, imgsensor.frame_length);
        write_cmos_sensor_8(0x0104,0x00);
    }

    // Update Shutter
    write_cmos_sensor_8(0x0104,0x01);
	write_cmos_sensor(0x0340, imgsensor.frame_length);
    write_cmos_sensor(0x0202, shutter);
    write_cmos_sensor_8(0x0104,0x00);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

	//LOG_INF("frame_length = %d ", frame_length);

}	/*	write_shutter  */



/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}	/*	set_shutter */



static kal_uint16 gain2reg(const kal_uint16 gain)
{
	 kal_uint16 reg_gain = 0x0;

    reg_gain = gain/2;
    return (kal_uint16)reg_gain;
}

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	  kal_uint16 reg_gain;

    /* 0x350A[0:1], 0x350B[0:7] AGC real gain */
    /* [0:3] = N meams N /16 X  */
    /* [4:9] = M meams M X       */
    /* Total gain = M + N /16 X   */

    //
    if (gain < BASEGAIN || gain > 32 * BASEGAIN) {
        LOG_INF("Error gain setting");

        if (gain < BASEGAIN)
            gain = BASEGAIN;
        else if (gain > 32 * BASEGAIN)
            gain = 32 * BASEGAIN;
    }

    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain;
    spin_unlock(&imgsensor_drv_lock);
    LOG_INF("gain = %d , reg_gain = 0x%x ", gain, reg_gain);

    write_cmos_sensor_8(0x0104, 0x01);
    write_cmos_sensor_8(0x0204,(reg_gain>>8));
    write_cmos_sensor_8(0x0205,(reg_gain&0xff));
    write_cmos_sensor_8(0x0104, 0x00);

    return gain;
}	/*	set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
#if 1
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
	if (imgsensor.ihdr_en) {

		spin_lock(&imgsensor_drv_lock);
			if (le > imgsensor.min_frame_length - imgsensor_info.margin)
				imgsensor.frame_length = le + imgsensor_info.margin;
			else
				imgsensor.frame_length = imgsensor.min_frame_length;
			if (imgsensor.frame_length > imgsensor_info.max_frame_length)
				imgsensor.frame_length = imgsensor_info.max_frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
			if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;


				// Extend frame length first
	 	write_cmos_sensor_8(0x0104,0x01);
		write_cmos_sensor(0x0340, imgsensor.frame_length);

		//write_cmos_sensor(0x0202, se);
		//write_cmos_sensor(0x021e,le);
		write_cmos_sensor(0x602A,0x021e);
		write_cmos_sensor(0x6f12,le);
		write_cmos_sensor(0x602A,0x0202);
		write_cmos_sensor(0x6f12,se);
		 write_cmos_sensor_8(0x0104,0x00);
	LOG_INF("iHDR:imgsensor.frame_length=%d\n",imgsensor.frame_length);
		set_gain(gain);
	}

#endif





}



static void set_mirror_flip(kal_uint8 image_mirror)
{
	/* LOG_INF("image_mirror = %d", image_mirror); */

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
	spin_lock(&imgsensor_drv_lock);
    imgsensor.mirror= image_mirror;
    spin_unlock(&imgsensor_drv_lock);
    switch (image_mirror) {

        case IMAGE_NORMAL:
            write_cmos_sensor_8(0x0101,0x00);   // Gr
            break;
        case IMAGE_H_MIRROR:
            write_cmos_sensor_8(0x0101,0x01);
            break;
        case IMAGE_V_MIRROR:
            write_cmos_sensor_8(0x0101,0x02);
            break;
        case IMAGE_HV_MIRROR:
            write_cmos_sensor_8(0x0101,0x03);//Gb
            break;
        default:
			LOG_INF("Error image_mirror setting\n");
    }

}
#if MULTI_WRITE
kal_uint16 addr_data_pair_init_RWB[] =
{
 0x6214,0x7970,    //open all clocks
	0x6218,0x7150,    //open all clocks
	0x6028,0x2000,
	0x602A,0x2E00,
	0x6F12,0x0448,
	0x6F12,0x0349,
	0x6F12,0x0160,
	0x6F12,0xC26A,
	0x6F12,0x511A,
	0x6F12,0x8180,
	0x6F12,0x00F0,
	0x6F12,0x10B9,
	0x6F12,0x2000,
	0x6F12,0x315C,
	0x6F12,0x2000,
	0x6F12,0x1AE0,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x2DE9,
	0x6F12,0xF041,
	0x6F12,0x0646,
	0x6F12,0x9A48,
	0x6F12,0x0F46,
	0x6F12,0x9046,
	0x6F12,0x4068,
	0x6F12,0x0022,
	0x6F12,0x85B2,
	0x6F12,0x040C,
	0x6F12,0x2946,
	0x6F12,0x2046,
	0x6F12,0x00F0,
	0x6F12,0x4CF9,
	0x6F12,0x4246,
	0x6F12,0x3946,
	0x6F12,0x3046,
	0x6F12,0x00F0,
	0x6F12,0x4CF9,
	0x6F12,0x9348,
	0x6F12,0x807E,
	0x6F12,0x28B1,
	0x6F12,0x9248,
	0x6F12,0x90F8,
	0x6F12,0xFA00,
	0x6F12,0x08B1,
	0x6F12,0x0122,
	0x6F12,0x00E0,
	0x6F12,0x0022,
	0x6F12,0x9048,
	0x6F12,0x0280,
	0x6F12,0x2946,
	0x6F12,0x2046,
	0x6F12,0xBDE8,
	0x6F12,0xF041,
	0x6F12,0x0122,
	0x6F12,0x00F0,
	0x6F12,0x34B9,
	0x6F12,0x8C4A,
	0x6F12,0x10B5,
	0x6F12,0x1268,
	0x6F12,0xB2F8,
	0x6F12,0xC230,
	0x6F12,0x874A,
	0x6F12,0x546B,
	0x6F12,0xB2F8,
	0x6F12,0x2E21,
	0x6F12,0x6409,
	0x6F12,0x6343,
	0x6F12,0x4FF4,
	0x6F12,0x7A74,
	0x6F12,0x6243,
	0x6F12,0x5209,
	0x6F12,0xB3FB,
	0x6F12,0xF2F3,
	0x6F12,0x021D,
	0x6F12,0x8A42,
	0x6F12,0x02D2,
	0x6F12,0x0A1A,
	0x6F12,0x121F,
	0x6F12,0x00E0,
	0x6F12,0x0022,
	0x6F12,0x9A42,
	0x6F12,0x00D8,
	0x6F12,0x1A46,
	0x6F12,0x8048,
	0x6F12,0x0280,
	0x6F12,0x10BD,
	0x6F12,0x2DE9,
	0x6F12,0xF34F,
	0x6F12,0x0446,
	0x6F12,0x7848,
	0x6F12,0x83B0,
	0x6F12,0x0022,
	0x6F12,0xC068,
	0x6F12,0x010C,
	0x6F12,0x80B2,
	0x6F12,0xCDE9,
	0x6F12,0x0001,
	0x6F12,0x0146,
	0x6F12,0x0198,
	0x6F12,0x00F0,
	0x6F12,0x07F9,
	0x6F12,0xDFF8,
	0x6F12,0xE091,
	0x6F12,0xDFF8,
	0x6F12,0xD4B1,
	0x6F12,0x0021,
	0x6F12,0x99F8,
	0x6F12,0x2A70,
	0x6F12,0x89F8,
	0x6F12,0x2A10,
	0x6F12,0xDFF8,
	0x6F12,0xBCA1,
	0x6F12,0x734D,
	0x6F12,0xDBF8,
	0x6F12,0x0000,
	0x6F12,0x9AF8,
	0x6F12,0xFA80,
	0x6F12,0xAE8A,
	0x6F12,0xB0F8,
	0x6F12,0xC400,
	0x6F12,0x20B1,
	0x6F12,0xA08A,
	0x6F12,0x296E,
	0x6F12,0x4843,
	0x6F12,0x000B,
	0x6F12,0xA882,
	0x6F12,0x2046,
	0x6F12,0x0499,
	0x6F12,0x00F0,
	0x6F12,0xF4F8,
	0x6F12,0xAE82,
	0x6F12,0xB9F8,
	0x6F12,0x1C00,
	0x6F12,0x9AF8,
	0x6F12,0xFA50,
	0x6F12,0xC0F3,
	0x6F12,0x0030,
	0x6F12,0x4545,
	0x6F12,0x08D0,
	0x6F12,0x38B1,
	0x6F12,0xDBF8,
	0x6F12,0x0000,
	0x6F12,0xB0F8,
	0x6F12,0xC600,
	0x6F12,0x10B1,
	0x6F12,0x0020,
	0x6F12,0xCAF8,
	0x6F12,0x0001,
	0x6F12,0x17F0,
	0x6F12,0xFF00,
	0x6F12,0x89F8,
	0x6F12,0x2A00,
	0x6F12,0x03D0,
	0x6F12,0xA18A,
	0x6F12,0x2068,
	0x6F12,0x00F0,
	0x6F12,0xDEF8,
	0x6F12,0xDDE9,
	0x6F12,0x0010,
	0x6F12,0x05B0,
	0x6F12,0x0122,
	0x6F12,0xBDE8,
	0x6F12,0xF04F,
	0x6F12,0x00F0,
	0x6F12,0xC7B8,
	0x6F12,0x70B5,
	0x6F12,0x0446,
	0x6F12,0x5148,
	0x6F12,0x0022,
	0x6F12,0x0169,
	0x6F12,0x0D0C,
	0x6F12,0x8EB2,
	0x6F12,0x3146,
	0x6F12,0x2846,
	0x6F12,0x00F0,
	0x6F12,0xBCF8,
	0x6F12,0x2046,
	0x6F12,0x00F0,
	0x6F12,0xCDF8,
	0x6F12,0x94F8,
	0x6F12,0x6000,
	0x6F12,0x0128,
	0x6F12,0x07D1,
	0x6F12,0x5148,
	0x6F12,0x0068,
	0x6F12,0x8078,
	0x6F12,0xF528,
	0x6F12,0x02D0,
	0x6F12,0x0020,
	0x6F12,0x84F8,
	0x6F12,0x6000,
	0x6F12,0x3146,
	0x6F12,0x2846,
	0x6F12,0xBDE8,
	0x6F12,0x7040,
	0x6F12,0x0122,
	0x6F12,0x00F0,
	0x6F12,0xA6B8,
	0x6F12,0x70B5,
	0x6F12,0x0446,
	0x6F12,0x4048,
	0x6F12,0x0022,
	0x6F12,0x4069,
	0x6F12,0x86B2,
	0x6F12,0x050C,
	0x6F12,0x3146,
	0x6F12,0x2846,
	0x6F12,0x00F0,
	0x6F12,0x9BF8,
	0x6F12,0x2046,
	0x6F12,0x00F0,
	0x6F12,0xB1F8,
	0x6F12,0x207C,
	0x6F12,0x40B1,
	0x6F12,0x424A,
	0x6F12,0xA168,
	0x6F12,0x2068,
	0x6F12,0x92F8,
	0x6F12,0xB921,
	0x6F12,0x00F0,
	0x6F12,0xADF8,
	0x6F12,0x4049,
	0x6F12,0x0880,
	0x6F12,0x3146,
	0x6F12,0x2846,
	0x6F12,0xBDE8,
	0x6F12,0x7040,
	0x6F12,0x0122,
	0x6F12,0x00F0,
	0x6F12,0x86B8,
	0x6F12,0x2DE9,
	0x6F12,0xF041,
	0x6F12,0x304C,
	0x6F12,0x8046,
	0x6F12,0x0022,
	0x6F12,0xA069,
	0x6F12,0x87B2,
	0x6F12,0x060C,
	0x6F12,0x3946,
	0x6F12,0x3046,
	0x6F12,0x00F0,
	0x6F12,0x7AF8,
	0x6F12,0x2F4D,
	0x6F12,0xB8F1,
	0x6F12,0x000F,
	0x6F12,0x0BD1,
	0x6F12,0x3248,
	0x6F12,0x0078,
	0x6F12,0x40B1,
	0x6F12,0x2868,
	0x6F12,0xB0F8,
	0x6F12,0x6A00,
	0x6F12,0x20B1,
	0x6F12,0x2188,
	0x6F12,0x8842,
	0x6F12,0x01D0,
	0x6F12,0x0120,
	0x6F12,0x00E0,
	0x6F12,0x0020,
	0x6F12,0x38B1,
	0x6F12,0x2449,
	0x6F12,0x0020,
	0x6F12,0xC1F8,
	0x6F12,0xDC01,
	0x6F12,0x81F8,
	0x6F12,0xCC01,
	0x6F12,0x00F0,
	0x6F12,0x83F8,
	0x6F12,0x2868,
	0x6F12,0xB0F8,
	0x6F12,0x6A00,
	0x6F12,0x2080,
	0x6F12,0x4046,
	0x6F12,0x00F0,
	0x6F12,0x81F8,
	0x6F12,0x3946,
	0x6F12,0x3046,
	0x6F12,0xBDE8,
	0x6F12,0xF041,
	0x6F12,0x0122,
	0x6F12,0x00F0,
	0x6F12,0x52B8,
	0x6F12,0x10B5,
	0x6F12,0x0022,
	0x6F12,0xAFF2,
	0x6F12,0x1721,
	0x6F12,0x2048,
	0x6F12,0x00F0,
	0x6F12,0x78F8,
	0x6F12,0x144C,
	0x6F12,0x0022,
	0x6F12,0xAFF2,
	0x6F12,0xD711,
	0x6F12,0x6060,
	0x6F12,0x1D48,
	0x6F12,0x00F0,
	0x6F12,0x70F8,
	0x6F12,0x0022,
	0x6F12,0xAFF2,
	0x6F12,0x0D11,
	0x6F12,0xA060,
	0x6F12,0x1B48,
	0x6F12,0x00F0,
	0x6F12,0x69F8,
	0x6F12,0x0022,
	0x6F12,0xAFF2,
	0x6F12,0xB711,
	0x6F12,0x2061,
	0x6F12,0x1848,
	0x6F12,0x00F0,
	0x6F12,0x62F8,
	0x6F12,0x0022,
	0x6F12,0xAFF2,
	0x6F12,0xE701,
	0x6F12,0xE060,
	0x6F12,0x1648,
	0x6F12,0x00F0,
	0x6F12,0x5BF8,
	0x6F12,0x0022,
	0x6F12,0xAFF2,
	0x6F12,0xB301,
	0x6F12,0x6061,
	0x6F12,0x1348,
	0x6F12,0x00F0,
	0x6F12,0x54F8,
	0x6F12,0xA061,
	0x6F12,0x0020,
	0x6F12,0x2080,
	0x6F12,0x10BD,
	0x6F12,0x0000,
	0x6F12,0x2000,
	0x6F12,0x3140,
	0x6F12,0x2000,
	0x6F12,0x1B10,
	0x6F12,0x2000,
	0x6F12,0x2530,
	0x6F12,0x4000,
	0x6F12,0x9802,
	0x6F12,0x2000,
	0x6F12,0x0550,
	0x6F12,0x4000,
	0x6F12,0xF000,
	0x6F12,0x2000,
	0x6F12,0x1300,
	0x6F12,0x2000,
	0x6F12,0x2890,
	0x6F12,0x2000,
	0x6F12,0x02C0,
	0x6F12,0x2000,
	0x6F12,0x1410,
	0x6F12,0x4000,
	0x6F12,0xA358,
	0x6F12,0x0000,
	0x6F12,0x15F7,
	0x6F12,0x0000,
	0x6F12,0x7A4F,
	0x6F12,0x0000,
	0x6F12,0x4A69,
	0x6F12,0x0000,
	0x6F12,0x1349,
	0x6F12,0x0000,
	0x6F12,0x3091,
	0x6F12,0x0000,
	0x6F12,0x6057,
	0x6F12,0x40F2,
	0x6F12,0xE96C,
	0x6F12,0xC0F2,
	0x6F12,0x000C,
	0x6F12,0x6047,
	0x6F12,0x41F2,
	0x6F12,0xF75C,
	0x6F12,0xC0F2,
	0x6F12,0x000C,
	0x6F12,0x6047,
	0x6F12,0x41F2,
	0x6F12,0x493C,
	0x6F12,0xC0F2,
	0x6F12,0x000C,
	0x6F12,0x6047,
	0x6F12,0x41F2,
	0x6F12,0x9D1C,
	0x6F12,0xC0F2,
	0x6F12,0x000C,
	0x6F12,0x6047,
	0x6F12,0x44F6,
	0x6F12,0x692C,
	0x6F12,0xC0F2,
	0x6F12,0x000C,
	0x6F12,0x6047,
	0x6F12,0x43F2,
	0x6F12,0x910C,
	0x6F12,0xC0F2,
	0x6F12,0x000C,
	0x6F12,0x6047,
	0x6F12,0x40F2,
	0x6F12,0x156C,
	0x6F12,0xC0F2,
	0x6F12,0x000C,
	0x6F12,0x6047,
	0x6F12,0x42F2,
	0x6F12,0xC71C,
	0x6F12,0xC0F2,
	0x6F12,0x000C,
	0x6F12,0x6047,
	0x6F12,0x46F2,
	0x6F12,0x570C,
	0x6F12,0xC0F2,
	0x6F12,0x000C,
	0x6F12,0x6047,
	0x6F12,0x4AF6,
	0x6F12,0x850C,
	0x6F12,0xC0F2,
	0x6F12,0x000C,
	0x6F12,0x6047,
	0x3EA6,0xFFFF, 	//fe_isp_dadlc_analog_gain_thrshold_for_ladlc_low
	0x3EA8,0xFFFF, 	//fe_isp_dadlc_analog_gain_thrshold_for_ladlc_high
	0x3A56,0x3BB8,    //Sampling enable
	0xF476,0x0040,    //DBR
	0xF40C,0x1180,    //CLP on, LDB off
	0xF480,0x0015,    //CLP LVL
	0x39EE,0x0507,
	0x3A06,0x0605,
	0x39D0,0x0707,
	0xF432,0x0100,    //DBS
	0xF4A6,0x00C0,    //RDV
	0xF42E,0x0060,    //CDS
	0xF49C,0x0000,    //RMP
	0xF496,0x0000,    //REF
	0xF49E,0x005E,    //ADC_SAT
	0xF47A,0x0017,    //VRD 3.2V
	0xF462,0x0000,    //VPIX 2.92V
	0xF460,0x0020,    //VTG 3.8V
	0x3A64,0x0010,
	0x3A66,0x0010,
	0x3A68,0x0010,
	0xF426,0x0007,    //LAT START
	0xF428,0x0004,    //LAT WIDTH
	0xF42A,0x0004,    //HOLD START
	0xF42C,0x0004,    //HOLD WIDTH
	0x32BC,0x000D,    //pixel boost
	0x3298,0x005E,
	0x3880,0x010B,
	0x3886,0x010A,
	0x388C,0x021A,
	0x389E,0x021C,
	0x38A4,0x0000,
	0x38AA,0x015C,
	0x38B0,0x015B,
	0x38B6,0x021A,
	0x38C2,0x0070,
	0x38C8,0x021C,
	0x38CE,0x0000,
	0x3976,0x0006,
	0x31C0,0x0C40,    //default 0004
	0x31C2,0x0C48,    //default 0B1C
	0x31B2,0x080C,
	0x31B4,0x0401,
	0x31B2,0x0804,
	0x39D6,0x0F0F,
	0x39DC,0x3030,
	0x39B0,0x0000,
	0x30C2,0x0300,
	0x3AF2,0x0000,
	0x3AF4,0x0000,
	0x3AF6,0x0000,
	0x3AF8,0x0000,
	0x3AFA,0x0000,
	0x3AFC,0x0000,
	0x3AFE,0x0000,
	0x3B00,0x0000,
	0x3B02,0x0000,
	0x3B04,0x0000,
	0x3B06,0x0000,
	0x3B08,0x0000,
	0x3B0A,0x0000,
	0x3B0C,0x0000,
	0x3B0E,0x0000,
	0x3B10,0x0000,
	0x3B12,0x0000,
	0x3B14,0x0000,
	0x3B16,0x0000,
	0x3B18,0x0000,
	0x3B1A,0x0000,
	0x3B1C,0x0000,
	0x3B1E,0x0000,
	0x3B20,0x0000,
	0x3B22,0x0000,
	0x3B24,0x0000,
	0x3B26,0x0000,
	0x3B28,0x0000,
	0x3B2A,0x0000,
	0x3B2C,0x0000,
	0x3B2E,0x0000,
	0x3B30,0x0000,
	0x3B32,0x0000,
	0x3B34,0x0000,
	0x3B36,0x0000,
	0x3B38,0x0000,
	0x3B3A,0x0000,
	0x3B3C,0x0000,
	0x3B3E,0x0000,
	0x3B40,0x0000,
	0x3B42,0x0000,
	0x3B44,0x0000,
	0x3B46,0x0000,
	0x3B48,0x0000,
	0x3B4A,0x0000,
	0x3B4C,0x0000,
	0x3B4E,0x0000,
	0x3B50,0x0000,
	0x3B52,0x0000,
	0x3B54,0x0000,
	0x3B56,0x0000,
	0x3B58,0x0000,
	0x3B5A,0x0000,
	0x3B5C,0x0000,
	0x3B5E,0x0000,
	0x3B60,0x0000,
	0x3B62,0x0000,
	0x3B64,0x0000,
	0x3B66,0x0000,
	0x3B68,0x0000,
	0x3B6A,0x0000,
	0x3B6C,0x0000,
	0x3B6E,0x0000,
	0x3B70,0x0000,
	0x3B72,0x0000,
	0x3B74,0x0000,
	0x3B76,0x0000,
	0x3B78,0x0000,
	0x3B7A,0x0000,
	0x3B7C,0x0000,
	0x3B7E,0x0000,
	0x3B80,0x0000,
	0x3B82,0x0000,
	0x3B84,0x0000,
	0x3B86,0x0000,
	0x3B88,0x0000,
	0x3B8A,0x0000,
	0x3B8C,0x0000,
	0x3B8E,0x0000,
	0x3B90,0x0000,
	0x3B92,0x0000,
	0x3B94,0x0000,
	0x3B96,0x0000,
	0x3B98,0x0000,
	0x3B9A,0x0000,
	0x3B9C,0x0000,
	0x3B9E,0x0000,
	0x3BA0,0x0000,
	0x3BA2,0x0000,
	0x3BA4,0x0000,
	0x3BA6,0x0000,
	0x3BA8,0x0000,
	0x3BAA,0x0000,
	0x3BAC,0x0000,
	0x3BAE,0x0000,
	0x3BB0,0x0000,
	0x3BB2,0x0000,
	0x3BB4,0x0000,
	0x3BB6,0x0000,
	0x3BB8,0x0000,
	0x3BBA,0x0000,
	0x3BBC,0x0000,
	0x3BBE,0x0000,
	0x3BC0,0x0000,
	0x3BC2,0x0000,
	0x3BC4,0x0000,
	0x3BC6,0x0000,
	0x3BC8,0x0000,
	0x3BCA,0x0000,
	0x3BCC,0x0000,
	0x3BCE,0x0000,
	0x3BD0,0x0000,
	0x3BD2,0x0000,
	0x3BD4,0x0000,
	0x3BD6,0x0000,
	0x3BD8,0x0000,
	0x3BDA,0x0000,
	0x3BDC,0x0000,
	0x3BDE,0x0000,
	0x3BE0,0x0000,
	0x3BE2,0x0000,
	0x3BE4,0x0000,
	0x3BE6,0x0000,
	0x3BE8,0x0000,
	0x3BEA,0x0000,
	0x3BEC,0x0000,
	0x3BEE,0x0000,
	0x3BF0,0x0000,
	0x3BF2,0x0000,
	0x3BF4,0x0000,
	0x3BF6,0x0000,
	0x3BF8,0x0000,
	0x3BFA,0x0000,
	0x3BFC,0x0000,
	0x3BFE,0x0000,
	0x3C00,0x0000,
	0x3C02,0x0000,
	0x3C04,0x0000,
	0x3C06,0x0000,
	0x3C08,0x0000,
	0x3C0A,0x0000,
	0x3C0C,0x0000,
	0x3C0E,0x0000,
	0x3C10,0x0000,
	0x3C12,0x0000,
	0x3C14,0x0000,
	0x3C16,0x0000,
	0x3C18,0x0000,
	0x3C1A,0x0000,
	0x3C1C,0x0000,
	0x3C1E,0x0000,
	0x3C20,0x0000,
	0x3C22,0x0000,
	0x3C24,0x0000,
	0x3C26,0x0000,
	0x3C28,0x0000,
	0x3C2A,0x0000,
	0x3C2C,0x0000,
	0x3C2E,0x0000,
	0x3C30,0x0000,
	0x3C32,0x0000,
	0x3C34,0x0000,
	0x3C36,0x0000,
	0x3C38,0x0000,
	0x3C3A,0x0000,
	0x3C3C,0x0000,
	0x3C3E,0x0000,
	0x3C40,0x0000,
	0x3C42,0x0000,
	0x3C44,0x0000,
	0x3C46,0x0000,
	0x3C48,0x0000,
	0x3C4A,0x0000,
	0x3C4C,0x0000,
	0x3C4E,0x0000,
	0x3C50,0x0000,
	0x3C52,0x0000,
	0x3C54,0x0000,
	0x3C56,0x0000,
	0x3C58,0x0000,
	0x3C5A,0x0000,
	0x3C5C,0x0000,
	0x3C5E,0x0000,
	0x3C60,0x0000,
	0x3C62,0x0000,
	0x3C64,0x0000,
	0x3C66,0x0000,
	0x3C68,0x0000,
	0x3C6A,0x0000,
	0x3C6C,0x0000,
	0x3C6E,0x0000,
	0x3C70,0x0000,
	0x3C72,0x0000,
	0x3C74,0x0000,
	0x3C76,0x0000,
	0x3C78,0x0000,
	0x3C7A,0x0000,
	0x3C7C,0x0000,
	0x3C7E,0x0000,
	0x3C80,0x0000,
	0x3C82,0x0000,
	0x3C84,0x0000,
	0x3C86,0x0000,
	0x3C88,0x0000,
	0x3C8A,0x0000,
	0x3C8C,0x0000,
	0x3C8E,0x0000,
	0x3C90,0x0000,
	0x3C92,0x0000,
	0x3C94,0x0000,
	0x3C96,0x0000,
	0x3C98,0x0000,
	0x3C9A,0x0000,
	0x3C9C,0x0000,
	0x3C9E,0x0000,
	0x3CA0,0x0000,
	0x3CA2,0x0000,
	0x3CA4,0x0000,
	0x3CA6,0x0000,
	0x3CA8,0x0000,
	0x3CAA,0x0000,
	0x3CAC,0x0000,
	0x3CAE,0x0000,
	0x3CB0,0x0000,
	0x3CB2,0x0000,
	0x3CB4,0x0000,
	0x3CB6,0x0000,
	0x3CB8,0x0000,
	0x3CBA,0x0000,
	0x3CBC,0x0000,
	0x3CBE,0x0000,
	0x3CC0,0x0000,
	0x3CC2,0x0000,
	0x3CC4,0x0000,
	0x3CC6,0x0000,
	0x3CC8,0x0000,
	0x3CCA,0x0000,
	0x3CCC,0x0000,
	0x3CCE,0x0000,
	0x3CD0,0x0000,
	0x3CD2,0x0000,
	0x3CD4,0x0000,
	0x3CD6,0x0000,
	0x3CD8,0x0000,
	0x3CDA,0x0000,
	0x3CDC,0x0000,
	0x3CDE,0x0000,
	0x3CE0,0x0000,
	0x3CE2,0x0000,
	0x3CE4,0x0000,
	0x3CE6,0x0000,
	0x3CE8,0x0000,
	0x3CEA,0x0000,
	0x3CEC,0x0000,
	0x3CEE,0x0000,
	0x3CF0,0x0000,
	0x3CF2,0x0000,
	0x3DD4,0x2000,
	0x3DD6,0x2000,
	0x3DDC,0x2000,    //
	0x3DDE,0x2000,    //
	0x3DF4,0x2000,    //
	0x3DF6,0x2000,    //
	0x3DFC,0x2000,    //
	0x3DFE,0x2000,    //
	0x3E14,0x2000,    //
	0x3E16,0x2000,    //
	0x3E1C,0x2000,    //
	0x3E1E,0x2000,    //
	0x3E76,0x0A04,
	0x6226,0x0001,    //Open clock to access ELG memory
	0x70B6,0x0001,    //Disable ELG
	0x3050,0x0002,
	0x3068,0x0000,
	0x6028,0x2000,
	0x602A,0x1410,
	0x6F12,0x0000,
	0x602A,0x1416,
	0x6F12,0x0108,
	0x6F12,0x0108,
	0x6F12,0x0A08,
	0x602A,0x141A, //modify address from 141B to 141A
	0x6F12,0x0A08,
	//S6F120203 : delete this register
	0x6F12,0x0202,
	0x6F12,0x0603,
	//S6F120603 : delete this register

	0x602A,0x1412,
	0x6F12,0x0100,
	0x3E9E,0x0011,
	0x3EA2,0x0033,
	//BPC
	0x6028,0x2000,
	0x602A,0x1748,
	0x6F12,0x0101,
	0x0B04,0x0101,
	0x306E,0x039C,    //smiaRegs_vendor_bpc_otp_clusters_address
	0x3072,0x00FF,    //smiaRegs_vendor_bpc_max_clusters_in_otp
	0x30C4,0x0001,    //smiaRegs_vendor_tnp_use_dgains_for_ladlc_enable
	0x30C6,0x0001,    //smiaRegs_vendor_tnp_reset_iir_on_ladlc_on_off
	0x3E86,0x0104,
	0x302E,0x0102,
	//Immediate abort
	0x3028,0x0000, //smiaRegs_vendor_sensor_abort_timing_method_on_rolling_sh
	0x302A,0x0000,  //smiaRegs_vendor_sensor_abort_timing_on_sw_stby
	0x3A70,0x0000,
	0x3A72,0xFB98,
	0x3A74,0x0000,
	0x3A76,0xFB98,
	0x3A78,0xFD32,
	0x3A7A,0x0000,
	0x3A7C,0xFB32,
	0x3A7E,0x0000,
	0x3A80,0x0000,
	0x3A82,0xFB98,
	0x3A84,0x0000,
	0x3A86,0xFB98,
	0x3A88,0xFD32,
	0x3A8A,0x0000,
	0x3A8C,0xFD32,
	0x3A8E,0x0000,
	0x3AB2,0x0000,
	0x3AB4,0x0000,
	0x3AB6,0x0000,
	0x3AB8,0x0000,
	0x3ABA,0x0000,
	0x3ABC,0x0000,
	0x3ABE,0x0000,
	0x3AC0,0x0000,
	0x3AC2,0x0000,
	0x3AC4,0x0000,
	0x3AC6,0x0000,
	0x3AC8,0x0000,
	0x3ACA,0x0000,
	0x3ACC,0x0000,
	0x3ACE,0x0000,
};


 kal_uint16 addr_data_pair_init[] =
{
	0x6214,0x7970,	  //open all clocks
	0x6218,0x7150,	  //open all clocks
	0x6028,0x2000,
	0x602A,0x2E00,
	0x6F12,0x0448,
	0x6F12,0x0349,
	0x6F12,0x0160,
	0x6F12,0xC26A,
	0x6F12,0x511A,
	0x6F12,0x8180,
	0x6F12,0x00F0,
	0x6F12,0x10B9,
	0x6F12,0x2000,
	0x6F12,0x315C,
	0x6F12,0x2000,
	0x6F12,0x1AE0,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x2DE9,
	0x6F12,0xF041,
	0x6F12,0x0646,
	0x6F12,0x9A48,
	0x6F12,0x0F46,
	0x6F12,0x9046,
	0x6F12,0x4068,
	0x6F12,0x0022,
	0x6F12,0x85B2,
	0x6F12,0x040C,
	0x6F12,0x2946,
	0x6F12,0x2046,
	0x6F12,0x00F0,
	0x6F12,0x4CF9,
	0x6F12,0x4246,
	0x6F12,0x3946,
	0x6F12,0x3046,
	0x6F12,0x00F0,
	0x6F12,0x4CF9,
	0x6F12,0x9348,
	0x6F12,0x807E,
	0x6F12,0x28B1,
	0x6F12,0x9248,
	0x6F12,0x90F8,
	0x6F12,0xFA00,
	0x6F12,0x08B1,
	0x6F12,0x0122,
	0x6F12,0x00E0,
	0x6F12,0x0022,
	0x6F12,0x9048,
	0x6F12,0x0280,
	0x6F12,0x2946,
	0x6F12,0x2046,
	0x6F12,0xBDE8,
	0x6F12,0xF041,
	0x6F12,0x0122,
	0x6F12,0x00F0,
	0x6F12,0x34B9,
	0x6F12,0x8C4A,
	0x6F12,0x10B5,
	0x6F12,0x1268,
	0x6F12,0xB2F8,
	0x6F12,0xC230,
	0x6F12,0x874A,
	0x6F12,0x546B,
	0x6F12,0xB2F8,
	0x6F12,0x2E21,
	0x6F12,0x6409,
	0x6F12,0x6343,
	0x6F12,0x4FF4,
	0x6F12,0x7A74,
	0x6F12,0x6243,
	0x6F12,0x5209,
	0x6F12,0xB3FB,
	0x6F12,0xF2F3,
	0x6F12,0x021D,
	0x6F12,0x8A42,
	0x6F12,0x02D2,
	0x6F12,0x0A1A,
	0x6F12,0x121F,
	0x6F12,0x00E0,
	0x6F12,0x0022,
	0x6F12,0x9A42,
	0x6F12,0x00D8,
	0x6F12,0x1A46,
	0x6F12,0x8048,
	0x6F12,0x0280,
	0x6F12,0x10BD,
	0x6F12,0x2DE9,
	0x6F12,0xF34F,
	0x6F12,0x0446,
	0x6F12,0x7848,
	0x6F12,0x83B0,
	0x6F12,0x0022,
	0x6F12,0xC068,
	0x6F12,0x010C,
	0x6F12,0x80B2,
	0x6F12,0xCDE9,
	0x6F12,0x0001,
	0x6F12,0x0146,
	0x6F12,0x0198,
	0x6F12,0x00F0,
	0x6F12,0x07F9,
	0x6F12,0xDFF8,
	0x6F12,0xE091,
	0x6F12,0xDFF8,
	0x6F12,0xD4B1,
	0x6F12,0x0021,
	0x6F12,0x99F8,
	0x6F12,0x2A70,
	0x6F12,0x89F8,
	0x6F12,0x2A10,
	0x6F12,0xDFF8,
	0x6F12,0xBCA1,
	0x6F12,0x734D,
	0x6F12,0xDBF8,
	0x6F12,0x0000,
	0x6F12,0x9AF8,
	0x6F12,0xFA80,
	0x6F12,0xAE8A,
	0x6F12,0xB0F8,
	0x6F12,0xC400,
	0x6F12,0x20B1,
	0x6F12,0xA08A,
	0x6F12,0x296E,
	0x6F12,0x4843,
	0x6F12,0x000B,
	0x6F12,0xA882,
	0x6F12,0x2046,
	0x6F12,0x0499,
	0x6F12,0x00F0,
	0x6F12,0xF4F8,
	0x6F12,0xAE82,
	0x6F12,0xB9F8,
	0x6F12,0x1C00,
	0x6F12,0x9AF8,
	0x6F12,0xFA50,
	0x6F12,0xC0F3,
	0x6F12,0x0030,
	0x6F12,0x4545,
	0x6F12,0x08D0,
	0x6F12,0x38B1,
	0x6F12,0xDBF8,
	0x6F12,0x0000,
	0x6F12,0xB0F8,
	0x6F12,0xC600,
	0x6F12,0x10B1,
	0x6F12,0x0020,
	0x6F12,0xCAF8,
	0x6F12,0x0001,
	0x6F12,0x17F0,
	0x6F12,0xFF00,
	0x6F12,0x89F8,
	0x6F12,0x2A00,
	0x6F12,0x03D0,
	0x6F12,0xA18A,
	0x6F12,0x2068,
	0x6F12,0x00F0,
	0x6F12,0xDEF8,
	0x6F12,0xDDE9,
	0x6F12,0x0010,
	0x6F12,0x05B0,
	0x6F12,0x0122,
	0x6F12,0xBDE8,
	0x6F12,0xF04F,
	0x6F12,0x00F0,
	0x6F12,0xC7B8,
	0x6F12,0x70B5,
	0x6F12,0x0446,
	0x6F12,0x5148,
	0x6F12,0x0022,
	0x6F12,0x0169,
	0x6F12,0x0D0C,
	0x6F12,0x8EB2,
	0x6F12,0x3146,
	0x6F12,0x2846,
	0x6F12,0x00F0,
	0x6F12,0xBCF8,
	0x6F12,0x2046,
	0x6F12,0x00F0,
	0x6F12,0xCDF8,
	0x6F12,0x94F8,
	0x6F12,0x6000,
	0x6F12,0x0128,
	0x6F12,0x07D1,
	0x6F12,0x5148,
	0x6F12,0x0068,
	0x6F12,0x8078,
	0x6F12,0xF528,
	0x6F12,0x02D0,
	0x6F12,0x0020,
	0x6F12,0x84F8,
	0x6F12,0x6000,
	0x6F12,0x3146,
	0x6F12,0x2846,
	0x6F12,0xBDE8,
	0x6F12,0x7040,
	0x6F12,0x0122,
	0x6F12,0x00F0,
	0x6F12,0xA6B8,
	0x6F12,0x70B5,
	0x6F12,0x0446,
	0x6F12,0x4048,
	0x6F12,0x0022,
	0x6F12,0x4069,
	0x6F12,0x86B2,
	0x6F12,0x050C,
	0x6F12,0x3146,
	0x6F12,0x2846,
	0x6F12,0x00F0,
	0x6F12,0x9BF8,
	0x6F12,0x2046,
	0x6F12,0x00F0,
	0x6F12,0xB1F8,
	0x6F12,0x207C,
	0x6F12,0x40B1,
	0x6F12,0x424A,
	0x6F12,0xA168,
	0x6F12,0x2068,
	0x6F12,0x92F8,
	0x6F12,0xB921,
	0x6F12,0x00F0,
	0x6F12,0xADF8,
	0x6F12,0x4049,
	0x6F12,0x0880,
	0x6F12,0x3146,
	0x6F12,0x2846,
	0x6F12,0xBDE8,
	0x6F12,0x7040,
	0x6F12,0x0122,
	0x6F12,0x00F0,
	0x6F12,0x86B8,
	0x6F12,0x2DE9,
	0x6F12,0xF041,
	0x6F12,0x304C,
	0x6F12,0x8046,
	0x6F12,0x0022,
	0x6F12,0xA069,
	0x6F12,0x87B2,
	0x6F12,0x060C,
	0x6F12,0x3946,
	0x6F12,0x3046,
	0x6F12,0x00F0,
	0x6F12,0x7AF8,
	0x6F12,0x2F4D,
	0x6F12,0xB8F1,
	0x6F12,0x000F,
	0x6F12,0x0BD1,
	0x6F12,0x3248,
	0x6F12,0x0078,
	0x6F12,0x40B1,
	0x6F12,0x2868,
	0x6F12,0xB0F8,
	0x6F12,0x6A00,
	0x6F12,0x20B1,
	0x6F12,0x2188,
	0x6F12,0x8842,
	0x6F12,0x01D0,
	0x6F12,0x0120,
	0x6F12,0x00E0,
	0x6F12,0x0020,
	0x6F12,0x38B1,
	0x6F12,0x2449,
	0x6F12,0x0020,
	0x6F12,0xC1F8,
	0x6F12,0xDC01,
	0x6F12,0x81F8,
	0x6F12,0xCC01,
	0x6F12,0x00F0,
	0x6F12,0x83F8,
	0x6F12,0x2868,
	0x6F12,0xB0F8,
	0x6F12,0x6A00,
	0x6F12,0x2080,
	0x6F12,0x4046,
	0x6F12,0x00F0,
	0x6F12,0x81F8,
	0x6F12,0x3946,
	0x6F12,0x3046,
	0x6F12,0xBDE8,
	0x6F12,0xF041,
	0x6F12,0x0122,
	0x6F12,0x00F0,
	0x6F12,0x52B8,
	0x6F12,0x10B5,
	0x6F12,0x0022,
	0x6F12,0xAFF2,
	0x6F12,0x1721,
	0x6F12,0x2048,
	0x6F12,0x00F0,
	0x6F12,0x78F8,
	0x6F12,0x144C,
	0x6F12,0x0022,
	0x6F12,0xAFF2,
	0x6F12,0xD711,
	0x6F12,0x6060,
	0x6F12,0x1D48,
	0x6F12,0x00F0,
	0x6F12,0x70F8,
	0x6F12,0x0022,
	0x6F12,0xAFF2,
	0x6F12,0x0D11,
	0x6F12,0xA060,
	0x6F12,0x1B48,
	0x6F12,0x00F0,
	0x6F12,0x69F8,
	0x6F12,0x0022,
	0x6F12,0xAFF2,
	0x6F12,0xB711,
	0x6F12,0x2061,
	0x6F12,0x1848,
	0x6F12,0x00F0,
	0x6F12,0x62F8,
	0x6F12,0x0022,
	0x6F12,0xAFF2,
	0x6F12,0xE701,
	0x6F12,0xE060,
	0x6F12,0x1648,
	0x6F12,0x00F0,
	0x6F12,0x5BF8,
	0x6F12,0x0022,
	0x6F12,0xAFF2,
	0x6F12,0xB301,
	0x6F12,0x6061,
	0x6F12,0x1348,
	0x6F12,0x00F0,
	0x6F12,0x54F8,
	0x6F12,0xA061,
	0x6F12,0x0020,
	0x6F12,0x2080,
	0x6F12,0x10BD,
	0x6F12,0x0000,
	0x6F12,0x2000,
	0x6F12,0x3140,
	0x6F12,0x2000,
	0x6F12,0x1B10,
	0x6F12,0x2000,
	0x6F12,0x2530,
	0x6F12,0x4000,
	0x6F12,0x9802,
	0x6F12,0x2000,
	0x6F12,0x0550,
	0x6F12,0x4000,
	0x6F12,0xF000,
	0x6F12,0x2000,
	0x6F12,0x1300,
	0x6F12,0x2000,
	0x6F12,0x2890,
	0x6F12,0x2000,
	0x6F12,0x02C0,
	0x6F12,0x2000,
	0x6F12,0x1410,
	0x6F12,0x4000,
	0x6F12,0xA358,
	0x6F12,0x0000,
	0x6F12,0x15F7,
	0x6F12,0x0000,
	0x6F12,0x7A4F,
	0x6F12,0x0000,
	0x6F12,0x4A69,
	0x6F12,0x0000,
	0x6F12,0x1349,
	0x6F12,0x0000,
	0x6F12,0x3091,
	0x6F12,0x0000,
	0x6F12,0x6057,
	0x6F12,0x40F2,
	0x6F12,0xE96C,
	0x6F12,0xC0F2,
	0x6F12,0x000C,
	0x6F12,0x6047,
	0x6F12,0x41F2,
	0x6F12,0xF75C,
	0x6F12,0xC0F2,
	0x6F12,0x000C,
	0x6F12,0x6047,
	0x6F12,0x41F2,
	0x6F12,0x493C,
	0x6F12,0xC0F2,
	0x6F12,0x000C,
	0x6F12,0x6047,
	0x6F12,0x41F2,
	0x6F12,0x9D1C,
	0x6F12,0xC0F2,
	0x6F12,0x000C,
	0x6F12,0x6047,
	0x6F12,0x44F6,
	0x6F12,0x692C,
	0x6F12,0xC0F2,
	0x6F12,0x000C,
	0x6F12,0x6047,
	0x6F12,0x43F2,
	0x6F12,0x910C,
	0x6F12,0xC0F2,
	0x6F12,0x000C,
	0x6F12,0x6047,
	0x6F12,0x40F2,
	0x6F12,0x156C,
	0x6F12,0xC0F2,
	0x6F12,0x000C,
	0x6F12,0x6047,
	0x6F12,0x42F2,
	0x6F12,0xC71C,
	0x6F12,0xC0F2,
	0x6F12,0x000C,
	0x6F12,0x6047,
	0x6F12,0x46F2,
	0x6F12,0x570C,
	0x6F12,0xC0F2,
	0x6F12,0x000C,
	0x6F12,0x6047,
	0x6F12,0x4AF6,
	0x6F12,0x850C,
	0x6F12,0xC0F2,
	0x6F12,0x000C,
	0x6F12,0x6047,
	0x3EA6,0xFFFF,	   //fe_isp_dadlc_analog_gain_thrshold_for_ladlc_low
	0x3EA8,0xFFFF,	   //fe_isp_dadlc_analog_gain_thrshold_for_ladlc_high
	0x3A56,0x3BB8,	  //Sampling enable
	0xF476,0x0040,	  //DBR
	0xF40C,0x1180,	  //CLP on, LDB off
	0xF480,0x0015,	  //CLP LVL
	0x39EE,0x0507,
	0x3A06,0x0605,
	0x39D0,0x0707,
	0xF432,0x0100,	  //DBS
	0xF4A6,0x00C0,	  //RDV
	0xF42E,0x0060,	  //CDS
	0xF49C,0x0000,	  //RMP
	0xF496,0x0000,	  //REF
	0xF49E,0x005E,	  //ADC_SAT
	0xF47A,0x0017,	  //VRD 3.2V
	0xF462,0x0000,	  //VPIX 2.92V
	0xF460,0x0020,	  //VTG 3.8V
	0x3A64,0x0010,
	0x3A66,0x0010,
	0x3A68,0x0010,
	0xF426,0x0007,	  //LAT START
	0xF428,0x0004,	  //LAT WIDTH
	0xF42A,0x0004,	  //HOLD START
	0xF42C,0x0004,	  //HOLD WIDTH
	0x32BC,0x000D,	  //pixel boost
	0x3298,0x005E,
	0x3880,0x010B,
	0x3886,0x010A,
	0x388C,0x021A,
	0x389E,0x021C,
	0x38A4,0x0000,
	0x38AA,0x015C,
	0x38B0,0x015B,
	0x38B6,0x021A,
	0x38C2,0x0070,
	0x38C8,0x021C,
	0x38CE,0x0000,
	0x3976,0x0006,
	0x31C0,0x0C40,	  //default 0004
	0x31C2,0x0C48,	  //default 0B1C
	0x31B2,0x080C,
	0x31B4,0x0401,
	0x31B2,0x0804,
	0x39D6,0x0F0F,
	0x39DC,0x3030,
	0x39B0,0x0000,
	0x30C2,0x0300,
	0x3AF2,0x0000,
	0x3AF4,0x0000,
	0x3AF6,0x0000,
	0x3AF8,0x0000,
	0x3AFA,0x0000,
	0x3AFC,0x0000,
	0x3AFE,0x0000,
	0x3B00,0x0000,
	0x3B02,0x0000,
	0x3B04,0x0000,
	0x3B06,0x0000,
	0x3B08,0x0000,
	0x3B0A,0x0000,
	0x3B0C,0x0000,
	0x3B0E,0x0000,
	0x3B10,0x0000,
	0x3B12,0x0000,
	0x3B14,0x0000,
	0x3B16,0x0000,
	0x3B18,0x0000,
	0x3B1A,0x0000,
	0x3B1C,0x0000,
	0x3B1E,0x0000,
	0x3B20,0x0000,
	0x3B22,0x0000,
	0x3B24,0x0000,
	0x3B26,0x0000,
	0x3B28,0x0000,
	0x3B2A,0x0000,
	0x3B2C,0x0000,
	0x3B2E,0x0000,
	0x3B30,0x0000,
	0x3B32,0x0000,
	0x3B34,0x0000,
	0x3B36,0x0000,
	0x3B38,0x0000,
	0x3B3A,0x0000,
	0x3B3C,0x0000,
	0x3B3E,0x0000,
	0x3B40,0x0000,
	0x3B42,0x0000,
	0x3B44,0x0000,
	0x3B46,0x0000,
	0x3B48,0x0000,
	0x3B4A,0x0000,
	0x3B4C,0x0000,
	0x3B4E,0x0000,
	0x3B50,0x0000,
	0x3B52,0x0000,
	0x3B54,0x0000,
	0x3B56,0x0000,
	0x3B58,0x0000,
	0x3B5A,0x0000,
	0x3B5C,0x0000,
	0x3B5E,0x0000,
	0x3B60,0x0000,
	0x3B62,0x0000,
	0x3B64,0x0000,
	0x3B66,0x0000,
	0x3B68,0x0000,
	0x3B6A,0x0000,
	0x3B6C,0x0000,
	0x3B6E,0x0000,
	0x3B70,0x0000,
	0x3B72,0x0000,
	0x3B74,0x0000,
	0x3B76,0x0000,
	0x3B78,0x0000,
	0x3B7A,0x0000,
	0x3B7C,0x0000,
	0x3B7E,0x0000,
	0x3B80,0x0000,
	0x3B82,0x0000,
	0x3B84,0x0000,
	0x3B86,0x0000,
	0x3B88,0x0000,
	0x3B8A,0x0000,
	0x3B8C,0x0000,
	0x3B8E,0x0000,
	0x3B90,0x0000,
	0x3B92,0x0000,
	0x3B94,0x0000,
	0x3B96,0x0000,
	0x3B98,0x0000,
	0x3B9A,0x0000,
	0x3B9C,0x0000,
	0x3B9E,0x0000,
	0x3BA0,0x0000,
	0x3BA2,0x0000,
	0x3BA4,0x0000,
	0x3BA6,0x0000,
	0x3BA8,0x0000,
	0x3BAA,0x0000,
	0x3BAC,0x0000,
	0x3BAE,0x0000,
	0x3BB0,0x0000,
	0x3BB2,0x0000,
	0x3BB4,0x0000,
	0x3BB6,0x0000,
	0x3BB8,0x0000,
	0x3BBA,0x0000,
	0x3BBC,0x0000,
	0x3BBE,0x0000,
	0x3BC0,0x0000,
	0x3BC2,0x0000,
	0x3BC4,0x0000,
	0x3BC6,0x0000,
	0x3BC8,0x0000,
	0x3BCA,0x0000,
	0x3BCC,0x0000,
	0x3BCE,0x0000,
	0x3BD0,0x0000,
	0x3BD2,0x0000,
	0x3BD4,0x0000,
	0x3BD6,0x0000,
	0x3BD8,0x0000,
	0x3BDA,0x0000,
	0x3BDC,0x0000,
	0x3BDE,0x0000,
	0x3BE0,0x0000,
	0x3BE2,0x0000,
	0x3BE4,0x0000,
	0x3BE6,0x0000,
	0x3BE8,0x0000,
	0x3BEA,0x0000,
	0x3BEC,0x0000,
	0x3BEE,0x0000,
	0x3BF0,0x0000,
	0x3BF2,0x0000,
	0x3BF4,0x0000,
	0x3BF6,0x0000,
	0x3BF8,0x0000,
	0x3BFA,0x0000,
	0x3BFC,0x0000,
	0x3BFE,0x0000,
	0x3C00,0x0000,
	0x3C02,0x0000,
	0x3C04,0x0000,
	0x3C06,0x0000,
	0x3C08,0x0000,
	0x3C0A,0x0000,
	0x3C0C,0x0000,
	0x3C0E,0x0000,
	0x3C10,0x0000,
	0x3C12,0x0000,
	0x3C14,0x0000,
	0x3C16,0x0000,
	0x3C18,0x0000,
	0x3C1A,0x0000,
	0x3C1C,0x0000,
	0x3C1E,0x0000,
	0x3C20,0x0000,
	0x3C22,0x0000,
	0x3C24,0x0000,
	0x3C26,0x0000,
	0x3C28,0x0000,
	0x3C2A,0x0000,
	0x3C2C,0x0000,
	0x3C2E,0x0000,
	0x3C30,0x0000,
	0x3C32,0x0000,
	0x3C34,0x0000,
	0x3C36,0x0000,
	0x3C38,0x0000,
	0x3C3A,0x0000,
	0x3C3C,0x0000,
	0x3C3E,0x0000,
	0x3C40,0x0000,
	0x3C42,0x0000,
	0x3C44,0x0000,
	0x3C46,0x0000,
	0x3C48,0x0000,
	0x3C4A,0x0000,
	0x3C4C,0x0000,
	0x3C4E,0x0000,
	0x3C50,0x0000,
	0x3C52,0x0000,
	0x3C54,0x0000,
	0x3C56,0x0000,
	0x3C58,0x0000,
	0x3C5A,0x0000,
	0x3C5C,0x0000,
	0x3C5E,0x0000,
	0x3C60,0x0000,
	0x3C62,0x0000,
	0x3C64,0x0000,
	0x3C66,0x0000,
	0x3C68,0x0000,
	0x3C6A,0x0000,
	0x3C6C,0x0000,
	0x3C6E,0x0000,
	0x3C70,0x0000,
	0x3C72,0x0000,
	0x3C74,0x0000,
	0x3C76,0x0000,
	0x3C78,0x0000,
	0x3C7A,0x0000,
	0x3C7C,0x0000,
	0x3C7E,0x0000,
	0x3C80,0x0000,
	0x3C82,0x0000,
	0x3C84,0x0000,
	0x3C86,0x0000,
	0x3C88,0x0000,
	0x3C8A,0x0000,
	0x3C8C,0x0000,
	0x3C8E,0x0000,
	0x3C90,0x0000,
	0x3C92,0x0000,
	0x3C94,0x0000,
	0x3C96,0x0000,
	0x3C98,0x0000,
	0x3C9A,0x0000,
	0x3C9C,0x0000,
	0x3C9E,0x0000,
	0x3CA0,0x0000,
	0x3CA2,0x0000,
	0x3CA4,0x0000,
	0x3CA6,0x0000,
	0x3CA8,0x0000,
	0x3CAA,0x0000,
	0x3CAC,0x0000,
	0x3CAE,0x0000,
	0x3CB0,0x0000,
	0x3CB2,0x0000,
	0x3CB4,0x0000,
	0x3CB6,0x0000,
	0x3CB8,0x0000,
	0x3CBA,0x0000,
	0x3CBC,0x0000,
	0x3CBE,0x0000,
	0x3CC0,0x0000,
	0x3CC2,0x0000,
	0x3CC4,0x0000,
	0x3CC6,0x0000,
	0x3CC8,0x0000,
	0x3CCA,0x0000,
	0x3CCC,0x0000,
	0x3CCE,0x0000,
	0x3CD0,0x0000,
	0x3CD2,0x0000,
	0x3CD4,0x0000,
	0x3CD6,0x0000,
	0x3CD8,0x0000,
	0x3CDA,0x0000,
	0x3CDC,0x0000,
	0x3CDE,0x0000,
	0x3CE0,0x0000,
	0x3CE2,0x0000,
	0x3CE4,0x0000,
	0x3CE6,0x0000,
	0x3CE8,0x0000,
	0x3CEA,0x0000,
	0x3CEC,0x0000,
	0x3CEE,0x0000,
	0x3CF0,0x0000,
	0x3CF2,0x0000,
	0x3DD4,0x2000,
	0x3DD6,0x2000,
	0x3DDC,0x2000,	  //
	0x3DDE,0x2000,	  //
	0x3DF4,0x2000,	  //
	0x3DF6,0x2000,	  //
	0x3DFC,0x2000,	  //
	0x3DFE,0x2000,	  //
	0x3E14,0x2000,	  //
	0x3E16,0x2000,	  //
	0x3E1C,0x2000,	  //
	0x3E1E,0x2000,	  //
	0x3E76,0x0A04,
	0x6226,0x0001,	  //Open clock to access ELG memory
	0x70B6,0x0001,	  //Disable ELG
	0x3050,0x0002,
	0x3068,0x0000,
	0x6028,0x2000,
	0x602A,0x1410,
	0x6F12,0x0000,
	0x602A,0x1416,
	0x6F12,0x0108,
	0x6F12,0x0108,
	0x6F12,0x0A08,
	0x602A,0x141A, //modify address from 141B to 141A
	0x6F12,0x0A08,
	//S6F120203 : delete this register
	0x6F12,0x0202,
	0x6F12,0x0603,
	//S6F120603 : delete this register

	0x602A,0x1412,
	0x6F12,0x0100,
	0x3E9E,0x0011,
	0x3EA2,0x0033,
	//BPC
	0x6028,0x2000,
	0x602A,0x1748,
	0x6F12,0x0101,
	0x0B04,0x0101,
	0x306E,0x039C,	  //smiaRegs_vendor_bpc_otp_clusters_address
	0x3072,0x00FF,	  //smiaRegs_vendor_bpc_max_clusters_in_otp
	0x30C4,0x0001,	  //smiaRegs_vendor_tnp_use_dgains_for_ladlc_enable
	0x30C6,0x0001,	  //smiaRegs_vendor_tnp_reset_iir_on_ladlc_on_off
	0x3E86,0x0104,
	0x302E,0x0102,
	//Immediate abort
	0x3028,0x0000, //smiaRegs_vendor_sensor_abort_timing_method_on_rolling_sh
	0x302A,0x0000,	//smiaRegs_vendor_sensor_abort_timing_on_sw_stby
	0x3A70,0x0000,
	0x3A72,0x0000,
	0x3A74,0x0000,
	0x3A76,0x0000,
	0x3A78,0x0000,
	0x3A7A,0x0000,
	0x3A7C,0x0000,
	0x3A7E,0x0000,
	0x3A80,0x0000,
	0x3A82,0x0000,
	0x3A84,0x0000,
	0x3A86,0x0000,
	0x3A88,0x0000,
	0x3A8A,0x0000,
	0x3A8C,0x0000,
	0x3A8E,0x0000,
	0x3AB2,0x0000,
	0x3AB4,0x0000,
	0x3AB6,0x0000,
	0x3AB8,0x0000,
	0x3ABA,0x0000,
	0x3ABC,0x0000,
	0x3ABE,0x0000,
	0x3AC0,0x0000,
	0x3AC2,0x0000,
	0x3AC4,0x0000,
	0x3AC6,0x0000,
	0x3AC8,0x0000,
	0x3ACA,0x0000,
	0x3ACC,0x0000,
	0x3ACE,0x0000,
        0xB0CA,0xC000,
};

kal_uint16 addr_data_pair_capture_pip_15[] =
{
#ifdef FIX_VIEW_ANGLE
    0x0344,0x0014,	/* smiaRegs_rw_frame_timing_x_addr_start */
    0x0346,0x0014,	/*smiaRegs_rw_frame_timing_y_addr_start */
    0x0348,0x14D3,	/*smiaRegs_rw_frame_timing_x_addr_end*/
    0x034A,0x0BB3,	/*smiaRegs_rw_frame_timing_y_addr_end*/
    0x034C,0x14C0,	//smiaRegs_rw_frame_timing_x_output_size
    0x034E,0x0BA0,	/*smiaRegs_rw_frame_timing_y_output_size*/
#else
    0x0344,0x0010,	/* smiaRegs_rw_frame_timing_x_addr_start */
    0x0346,0x0008,	/*smiaRegs_rw_frame_timing_y_addr_start */
    0x0348,0x14DF,	/*smiaRegs_rw_frame_timing_x_addr_end*/
    0x034A,0x0BBF,	/*smiaRegs_rw_frame_timing_y_addr_end*/
    0x034C,0x14D0,	//smiaRegs_rw_frame_timing_x_output_size
    0x034E,0x0BB8,	/*smiaRegs_rw_frame_timing_y_output_size*/
#endif
    0x0382,0x0001,	/*smiaRegs_rw_sub_sample_x_odd_inc*/
    0x0380,0x0001,	/*smiaRegs_rw_sub_sample_x_even_inc*/
    0x0386,0x0001,	/*smiaRegs_rw_sub_sample_y_odd_inc*/
    0x0384,0x0001,	/*smiaRegs_rw_sub_sample_y_even_inc*/
    0x0900,0x0011,	/*smiaRegs_rw_binning_mode*/
//		write_cmos_sensor_8(0x0901,0x11,	/*smiaRegs_rw_binning_type*/
    0x0400,0x0000,	/*smiaRegs_rw_scaling_scaling_mode*/
    0x0404,0x0010,	/*smiaRegs_rw_scaling_scale_m*/
    0x0114,0x0300,
    0x0110,0x0002,	/*smiaRegs_rw_output_signalling_mode*/
    0x0136,0x1800,
    0x0304,0x0006,
    0x0306,0x00AF,	//smiaRegs_rw_clocks_pll_multiplier // 175
    0x0302,0x0001,	//smiaRegs_rw_clocks_vt_pix_clk_div
    0x0300,0x0005,	//smiaRegs_rw_clocks_vt_sys_clk_div
    0x030C,0x0004,	//smiaRegs_rw_clocks_secnd_pre_pll_clk_div
    0x030E,0x003A,	//smiaRegs_rw_clocks_secnd_pll_multiplier // 50
    0x030A,0x0001,	//smiaRegs_rw_clocks_op_sys_clk_div
    0x0308,0x0008,	//smiaRegs_rw_clocks_op_pix_clk_div
    0x1118,0x4100,
    0x1124,0x4100,
    0x112C,0x4100,
    0x1164,0x4100,
    0x1170,0x4100,
    0x301C,0x4100,
    0x0342,0x2FC0,	//smiaRegs_rw_frame_timing_line_length_pck // 3216
    0x0340,0x0C66,	//smiaRegs_rw_frame_timing_frame_length_lines // 580
    0x0200,0x0100,	//smiaRegs_rw_integration_time_fine_integration_time
    0x0202,0x0100,	//smiaRegs_rw_integration_time_coarse_integration_time
    0x0216,0x0000,	//smiaRegs_rw_wdr_multiple_exp_mode
    0x3054,0x0100,	//smiaRegs_vendor_sensor_enable_af_pixels
    0x3A6A,0x8000,
    0x3A6A,0x0D00,
    0x39BA,0x0002,
    0x3004,0x0005,
    0x3A58,0x0061,
    0x39E2,0x0102,
    0x3238,0x0529,	// SenAnalog_AIG_pDefaultNormalPtrs_3__1_
    0x324A,0x00DE,	// SenAnalog_AIG_pDefaultNormalPtrs_6__1_
    0x3250,0x011F,	 //SenAnalog_AIG_pDefaultNormalPtrs_7__1_
    0x3274,0x013E,	 //SenAnalog_AIG_pDefaultNormalPtrs_13__1_
    0x32C2,0x011F,	 //SenAnalog_AIG_pDefaultNormalPtrs_26__1_
    0x32C8,0x0140,	// SenAnalog_AIG_pDefaultNormalPtrs_27__1_
    0x32DA,0x011F,	// SenAnalog_AIG_pDefaultNormalPtrs_30__1_
    0x32E0,0x0120,  //  SenAnalog_AIG_pDefaultNormalPtrs_31__1_
    0x35FE,0x0078,  //  SenAnalog_AIG_pDefaultNormalPtrs_164__1_
    0x37C6,0x0083,	// SenAnalog_AIG_pDefaultVdaAndShPtrs_30__1_
    0x37CC,0x005D,	 //SenAnalog_AIG_pDefaultVdaAndShPtrs_31__1_
    0x37D2,0x0057,	// SenAnalog_AIG_pDefaultVdaAndShPtrs_32__1_
    0x37DE,0x0081,	// SenAnalog_AIG_pDefaultVdaAndShPtrs_34__1_
    0x37E4,0x005F,	// SenAnalog_AIG_pDefaultVdaAndShPtrs_35__1_
    0x37EA,0x0055,	// SenAnalog_AIG_pDefaultVdaAndShPtrs_36__1_
    0x37F6,0x0081,	// SenAnalog_AIG_pDefaultVdaAndShPtrs_38__1_
    0x37FC,0x005F,	// SenAnalog_AIG_pDefaultVdaAndShPtrs_39__1_
    0x3802,0x0055,	 //SenAnalog_AIG_pDefaultVdaAndShPtrs_40__1_
    0x6028,0x2000,
    0x602A,0x15b8,
    0x6F12,0x8011,
    0x3140,0x0FE2,
    0x31B4,0x0400,
    0x6028,0x2000,
    0x602A,0x1759,
    0x6F12,0x0001,
    0x6F12,0x0048,
    0x6F12,0x0050,
    0x6F12,0x0060,
    0x6F12,0x0060,
    0x6F12,0xF42E,
    0x6F12,0x006D,
    0x6F12,0x006A,
    0x6F12,0xF51E,

};

kal_uint16 addr_data_pair_capture_pip[] =
{
#ifdef FIX_VIEW_ANGLE
	    0x0344,0x0018,	//smiaRegs_rw_frame_timing_x_addr_start
	    0x0346,0x0014,	//smiaRegs_rw_frame_timing_y_addr_start
	    0x0348,0x14D7,	//smiaRegs_rw_frame_timing_x_addr_end
	    0x034A,0x0BB3,	//smiaRegs_rw_frame_timing_y_addr_end
	    0x034C,0x14C0,	//smiaRegs_rw_frame_timing_x_output_size
	    0x034E,0x0BA0,	//smiaRegs_rw_frame_timing_y_output_size
#else
        0x0344,0x0008,   /* smiaRegs_rw_frame_timing_x_addr_start */
        0x0346,0x0008,   /*smiaRegs_rw_frame_timing_y_addr_start */
        0x0348,0x14DF,   /*smiaRegs_rw_frame_timing_x_addr_end*/
        0x034A,0x0BBF,   /*smiaRegs_rw_frame_timing_y_addr_end*/
	    0x034C,0x14D0,	//smiaRegs_rw_frame_timing_x_output_size
        0x034E,0x0BB8,   /*smiaRegs_rw_frame_timing_y_output_size*/
#endif
        0x0382,0x0001,   /*smiaRegs_rw_sub_sample_x_odd_inc*/
        0x0380,0x0001,   /*smiaRegs_rw_sub_sample_x_even_inc*/
        0x0386,0x0001,   /*smiaRegs_rw_sub_sample_y_odd_inc*/
        0x0384,0x0001,   /*smiaRegs_rw_sub_sample_y_even_inc*/
        0x0900,0x0011,   /*smiaRegs_rw_binning_mode*/
//    write_cmos_sensor_8(0x0901,0x11,   /*smiaRegs_rw_binning_type*/
        0x0400,0x0000,   /*smiaRegs_rw_scaling_scaling_mode*/
        0x0404,0x0010,   /*smiaRegs_rw_scaling_scale_m*/
	    0x0114,0x0300,
        0x0110,0x0002,   /*smiaRegs_rw_output_signalling_mode*/
        0x0136,0x1800,
        0x0304,0x0006,
	    0x0306,0x0074,	//smiaRegs_rw_clocks_pll_multiplier // 175
	    0x0300,0x0005,	//smiaRegs_rw_clocks_vt_pix_clk_div
	    0x0302,0x0001,	//smiaRegs_rw_clocks_vt_sys_clk_div
	    0x030C,0x0004,	//smiaRegs_rw_clocks_secnd_pre_pll_clk_div
	    0x030E,0x004B,	//smiaRegs_rw_clocks_secnd_pll_multiplier // 50
	    0x030A,0x0001,	//smiaRegs_rw_clocks_op_sys_clk_div
	    0x0308,0x0008,	//smiaRegs_rw_clocks_op_pix_clk_div
	    0x1118,0x4100,
	    0x1124,0x4100,
	    0x112C,0x4100,
	    0x1164,0x4100,
	    0x1170,0x4100,
	    0x301C,0x4100,
	    0x0342,0x16f8,	//smiaRegs_rw_frame_timing_line_length_pck // 3216
	    0x0340,0x0C66,	//smiaRegs_rw_frame_timing_frame_length_lines // 580
	    0x0200,0x0100,	//smiaRegs_rw_integration_time_fine_integration_time
	    0x0202,0x0100,	//smiaRegs_rw_integration_time_coarse_integration_time
	    0x0216,0x0000,	//smiaRegs_rw_wdr_multiple_exp_mode
	    0x3054,0x0100,	//smiaRegs_vendor_sensor_enable_af_pixels
	    0x3A6A,0x8000,
	    0x3A6A,0x0D00,
	    0x39BA,0x0002,
	    0x3A58,0x0061,
	    0x39E2,0x0102,
	    0x3238,0x0529,   // SenAnalog_AIG_pDefaultNormalPtrs_3__1_
	    0x324A,0x00DE,   // SenAnalog_AIG_pDefaultNormalPtrs_6__1_
	    0x3250,0x011F,    //SenAnalog_AIG_pDefaultNormalPtrs_7__1_
	    0x3274,0x013E,    //SenAnalog_AIG_pDefaultNormalPtrs_13__1_
	    0x32C2,0x011F,    //SenAnalog_AIG_pDefaultNormalPtrs_26__1_
	    0x32C8,0x0140,   // SenAnalog_AIG_pDefaultNormalPtrs_27__1_
	    0x32DA,0x011F,   // SenAnalog_AIG_pDefaultNormalPtrs_30__1_
	    0x32E0,0x0120,  //  SenAnalog_AIG_pDefaultNormalPtrs_31__1_
	    0x35FE,0x0078,  //  SenAnalog_AIG_pDefaultNormalPtrs_164__1_
	    0x37C6,0x0083,   // SenAnalog_AIG_pDefaultVdaAndShPtrs_30__1_
	    0x37CC,0x005D,    //SenAnalog_AIG_pDefaultVdaAndShPtrs_31__1_
	    0x37D2,0x0057,   // SenAnalog_AIG_pDefaultVdaAndShPtrs_32__1_
	    0x37DE,0x0081,   // SenAnalog_AIG_pDefaultVdaAndShPtrs_34__1_
	    0x37E4,0x005F,   // SenAnalog_AIG_pDefaultVdaAndShPtrs_35__1_
	    0x37EA,0x0055,   // SenAnalog_AIG_pDefaultVdaAndShPtrs_36__1_
	    0x37F6,0x0081,   // SenAnalog_AIG_pDefaultVdaAndShPtrs_38__1_
	    0x37FC,0x005F,   // SenAnalog_AIG_pDefaultVdaAndShPtrs_39__1_
	    0x3802,0x0055,    //SenAnalog_AIG_pDefaultVdaAndShPtrs_40__1_
	    0x6028,0x2000,
	    0x602A,0x15b8,
	    0x6F12,0x8011,
	    0x3140,0x0FE2,
	    0x31B4,0X0400,
	    0x6028,0x2000,
	    0x602A,0x1759,
	    0x6F12,0x0001,
	    0x6F12,0x0048,
	    0x6F12,0x0050,
	    0x6F12,0x0060,
	    0x6F12,0x0060,
	    0x6F12,0xF42E,
	    0x6F12,0x006D,
	    0x6F12,0x006A,
	    0x6F12,0xF51E,
};

kal_uint16 addr_data_pair_capture[] =
{

#ifdef FIX_VIEW_ANGLE
	    0x0344,0x0018,	//smiaRegs_rw_frame_timing_x_addr_start
	    0x0346,0x0014,	//smiaRegs_rw_frame_timing_y_addr_start
	    0x0348,0x14D7,	//smiaRegs_rw_frame_timing_x_addr_end
	    0x034A,0x0BB3,	//smiaRegs_rw_frame_timing_y_addr_end
	    0x034C,0x14C0,	//smiaRegs_rw_frame_timing_x_output_size
	    0x034E,0x0BA0,	//smiaRegs_rw_frame_timing_y_output_size
#else
	    0x0344,0x0010,	//smiaRegs_rw_frame_timing_x_addr_start
	    0x0346,0x0008,	//smiaRegs_rw_frame_timing_y_addr_start
	    0x0348,0x14DF,	//smiaRegs_rw_frame_timing_x_addr_end
	    0x034A,0x0BBF,	//smiaRegs_rw_frame_timing_y_addr_end
	    0x034C,0x14D0,	//smiaRegs_rw_frame_timing_x_output_size
	    0x034E,0x0BB8,	//smiaRegs_rw_frame_timing_y_output_size
#endif
	    0x0382,0x0001,	//smiaRegs_rw_sub_sample_x_odd_inc
	    0x0380,0x0001,	//smiaRegs_rw_sub_sample_x_even_inc
	    0x0386,0x0001,	//smiaRegs_rw_sub_sample_y_odd_inc
	    0x0384,0x0001,	//smiaRegs_rw_sub_sample_y_even_inc
	    0x0900,0x0011,	//smiaRegs_rw_binning_mode
	//write_cmos_sensor_8(0x0901,0x11,	//smiaRegs_rw_binning_type
	    0x0400,0x0000,	//smiaRegs_rw_scaling_scaling_mode
	    0x0404,0x0010,	//smiaRegs_rw_scaling_scale_m
	    0x0114,0x0300,
	    0x0110,0x0002,	//smiaRegs_rw_output_signalling_mode
	    0x0136,0x1800,	//smiaRegs_rw_op_cond_extclk_frequency_mhz
	    0x0304,0x0006,	//smiaRegs_rw_clocks_pre_pll_clk_div
	    0x0306,0x00AF,	//smiaRegs_rw_clocks_pll_multiplier // 175
	    0x0300,0x0005,	//smiaRegs_rw_clocks_vt_pix_clk_div
	    0x0302,0x0001,	//smiaRegs_rw_clocks_vt_sys_clk_div
	    0x030C,0x0004,	//smiaRegs_rw_clocks_secnd_pre_pll_clk_div
	    0x030E,0x0074,	//smiaRegs_rw_clocks_secnd_pll_multiplier // 50
	    0x030A,0x0001,	//smiaRegs_rw_clocks_op_sys_clk_div
	    0x0308,0x0008,	//smiaRegs_rw_clocks_op_pix_clk_div
	    0x1118,0x43FA,
	    0x1124,0x43FA,
	    0x112C,0x42C0,
	    0x1164,0x4280,
	    0x1170,0x4100,
	    0x301C,0x4396,
	    0x0342,0x16F8,	//smiaRegs_rw_frame_timing_line_length_pck // 3216
	    0x0340,0x0C66,	//smiaRegs_rw_frame_timing_frame_length_lines // 580
	    0x0200,0x0100,	//smiaRegs_rw_integration_time_fine_integration_time
	    0x0202,0x0100,	//smiaRegs_rw_integration_time_coarse_integration_time
	    0x0216,0x0000,	//smiaRegs_rw_wdr_multiple_exp_mode
	    0x3054,0x0100,	//smiaRegs_vendor_sensor_enable_af_pixels
	    0x306A,0x8000,
	    0x3A6A,0x0D00,
	    0x39BA,0x0002,
	    0x3A58,0x0061,
	    0x39E2,0x0102,
	    0x3238,0x0219,   // SenAnalog_AIG_pDefaultNormalPtrs_3__1_
	    0x324A,0x00DE,   // SenAnalog_AIG_pDefaultNormalPtrs_6__1_
	    0x3250,0x011F,    //SenAnalog_AIG_pDefaultNormalPtrs_7__1_
	    0x3274,0x013E,    //SenAnalog_AIG_pDefaultNormalPtrs_13__1_
	    0x32C2,0x011F,    //SenAnalog_AIG_pDefaultNormalPtrs_26__1_
	    0x32C8,0x0140,   // SenAnalog_AIG_pDefaultNormalPtrs_27__1_
	    0x32DA,0x011F,   // SenAnalog_AIG_pDefaultNormalPtrs_30__1_
	    0x32E0,0x0120,  //  SenAnalog_AIG_pDefaultNormalPtrs_31__1_
	    0x35FE,0x0078,  //  SenAnalog_AIG_pDefaultNormalPtrs_164__1_
	    0x37C6,0x0083,   // SenAnalog_AIG_pDefaultVdaAndShPtrs_30__1_
	    0x37CC,0x005D,    //SenAnalog_AIG_pDefaultVdaAndShPtrs_31__1_
	    0x37D2,0x0057,   // SenAnalog_AIG_pDefaultVdaAndShPtrs_32__1_
	    0x37DE,0x0081,   // SenAnalog_AIG_pDefaultVdaAndShPtrs_34__1_
	    0x37E4,0x005F,   // SenAnalog_AIG_pDefaultVdaAndShPtrs_35__1_
	    0x37EA,0x0055,   // SenAnalog_AIG_pDefaultVdaAndShPtrs_36__1_
	    0x37F6,0x0081,   // SenAnalog_AIG_pDefaultVdaAndShPtrs_38__1_
	    0x37FC,0x005F,   // SenAnalog_AIG_pDefaultVdaAndShPtrs_39__1_
	    0x3802,0x0055,    //SenAnalog_AIG_pDefaultVdaAndShPtrs_40__1_
	    0x6028,0x2000,
	    0x602A,0x15b8,
	    0x6F12,0x8011,
	    0x3140,0x0FE2,
	    0x31B4,0x0400,
	    0x6028,0x2000,
	    0x602A,0x1759,
	    0x6F12,0x0001,
	    0x6F12,0x0048,
	    0x6F12,0x0050,
	    0x6F12,0x0060,
	    0x6F12,0x0060,
	    0x6F12,0xF42E,
	    0x6F12,0x006D,
	    0x6F12,0x006A,
	    0x6F12,0xF51E,
};

kal_uint16 addr_data_pair_preview[] =
{
#ifdef FIX_VIEW_ANGLE
	0x0344,0x0018,   /*smiaRegs_rw_frame_timing_x_addr_start*/
	0x0346,0x0014,   /*smiaRegs_rw_frame_timing_y_addr_start*/
	0x0348,0x14D7,   /*smiaRegs_rw_frame_timing_x_addr_end*/
	0x034A,0x0BB3,   /*smiaRegs_rw_frame_timing_y_addr_end*/
	0x034C,0x0A60,   /*smiaRegs_rw_frame_timing_x_output_size*/
	0x034E,0x05D0,   /*smiaRegs_rw_frame_timing_y_output_size*/
#else
	0x0344,0x0010,   /*smiaRegs_rw_frame_timing_x_addr_start*/
	0x0346,0x0008,   /*smiaRegs_rw_frame_timing_y_addr_start*/
	0x0348,0x14DF,   /*smiaRegs_rw_frame_timing_x_addr_end*/
	0x034A,0x0BBF,   /*smiaRegs_rw_frame_timing_y_addr_end*/
	0x034C,0x0A68,   /*smiaRegs_rw_frame_timing_x_output_size*/
	0x034E,0x05DC,   /*smiaRegs_rw_frame_timing_y_output_size*/
#endif
	0x0382,0x0003,   /*smiaRegs_rw_sub_sample_x_odd_inc*/
	0x0380,0x0001,   /*smiaRegs_rw_sub_sample_x_even_inc*/
	0x0386,0x0003,   /*smiaRegs_rw_sub_sample_y_odd_inc*/
	0x0384,0x0001,   /*smiaRegs_rw_sub_sample_y_even_inc*/
	0x0900,0x0122,   /*smiaRegs_rw_binning_mode*/
	0x0400,0x0000,   /*smiaRegs_rw_scaling_scaling_mode*/
	0x0404,0x0010,   /*smiaRegs_rw_scaling_scale_m*/
	0x0114,0x0300,
	0x0110,0x0002,   /*smiaRegs_rw_output_signalling_mode*/
	0x0136,0x1800,   /*smiaRegs_rw_op_cond_extclk_frequency_mhz*/
	0x0304,0x0006,   /*smiaRegs_rw_clocks_pre_pll_clk_div*/
	0x0306,0x0074,   /*smiaRegs_rw_clocks_pll_multiplier // 175*/
	0x0300,0x0005,   /*smiaRegs_rw_clocks_vt_pix_clk_div*/
	0x0302,0x0001,   /*smiaRegs_rw_clocks_vt_sys_clk_div*/
	0x030C,0x0004,   /*smiaRegs_rw_clocks_secnd_pre_pll_clk_div*/
	0x030E,0x004A,   /*smiaRegs_rw_clocks_secnd_pll_multiplier  50*/
	0x030A,0x0001,   /*smiaRegs_rw_clocks_op_sys_clk_div*/
	0x0308,0x0008,   /*smiaRegs_rw_clocks_op_pix_clk_div*/
	0x1118,0x4100,
	0x1124,0x4100,
	0x112C,0x4100,
	0x1164,0x4100,
	0x1170,0x4100,
	0x301C,0x4100,
	0x0342,0x16F8,   /*smiaRegs_rw_frame_timing_line_length_pck // 3216*/
	0x0340,0x0804,   /*smiaRegs_rw_frame_timing_frame_length_lines // 580*/
	0x0200,0x0100,   /*smiaRegs_rw_integration_time_fine_integration_time*/
	0x0202,0x0100,   /*smiaRegs_rw_integration_time_coarse_integration_time*/
	0x0216,0x0000,   /*smiaRegs_rw_wdr_multiple_exp_mode*/
	0x3054,0x0000,   /*smiaRegs_vendor_sensor_enable_af_pixels*/
	0x306A,0x8110,
	0x3A6A,0x0D00,
	0x39BA,0x0002,
	0x3A58,0x0060,
	0x39E2,0x0102,
	0x3238,0x0519,   /* SenAnalog_AIG_pDefaultNormalPtrs_3__1_              */
	0x324A,0x00DE,   /* SenAnalog_AIG_pDefaultNormalPtrs_6__1_              */
	0x3250,0x011F,    /*SenAnalog_AIG_pDefaultNormalPtrs_7__1_              */
	0x3274,0x013E,    /*SenAnalog_AIG_pDefaultNormalPtrs_13__1_             */
	0x32C2,0x011F,    /*SenAnalog_AIG_pDefaultNormalPtrs_26__1_             */
	0x32C8,0x0140,   /* SenAnalog_AIG_pDefaultNormalPtrs_27__1_             */
	0x32DA,0x011F,   /* SenAnalog_AIG_pDefaultNormalPtrs_30__1_             */
	0x32E0,0x0120,  /*  SenAnalog_AIG_pDefaultNormalPtrs_31__1_             */
	0x35FE,0x0078,  /*  SenAnalog_AIG_pDefaultNormalPtrs_164__1_            */
	0x37C6,0x0083,   /* SenAnalog_AIG_pDefaultVdaAndShPtrs_30__1_           */
	0x37CC,0x005D,    /*SenAnalog_AIG_pDefaultVdaAndShPtrs_31__1_           */
	0x37D2,0x0057,   /* SenAnalog_AIG_pDefaultVdaAndShPtrs_32__1_           */
	0x37DE,0x0081,   /* SenAnalog_AIG_pDefaultVdaAndShPtrs_34__1_           */
	0x37E4,0x005F,   /* SenAnalog_AIG_pDefaultVdaAndShPtrs_35__1_           */
	0x37EA,0x0055,   /* SenAnalog_AIG_pDefaultVdaAndShPtrs_36__1_           */
	0x37F6,0x0081,   /* SenAnalog_AIG_pDefaultVdaAndShPtrs_38__1_           */
	0x37FC,0x005F,   /* SenAnalog_AIG_pDefaultVdaAndShPtrs_39__1_           */
	0x3802,0x0055,    /*SenAnalog_AIG_pDefaultVdaAndShPtrs_40__1_   */
	0x6028,0x2000,
	0x602A,0x15b8,
	0x6F12,0x8011,
	0x3140,0x0FE2,
	0x31B4,0x0400,
	0x6028,0x2000,
	0x602A,0x1759,
	0x6F12,0x0001,
	0x6F12,0x0048,
	0x6F12,0x0050,
	0x6F12,0x0060,
	0x6F12,0x0060,
	0x6F12,0xF42E,
	0x6F12,0x006D,
	0x6F12,0x006A,
	0x6F12,0xF51E,
};
#endif

static void sensor_init(void)
{
#if MULTI_WRITE
	LOG_INF("multi-write E\n");
#else
	LOG_INF("E\n");
#endif
#ifdef PDAF_TEST
    //int j = 0;
    int size=0x600;
    //memset(data, (char)0x2, 1024);
    //memset(data+1024, (char)0x3, 1024);
    //memset(data+2048, (char)0x4, 2048);
    //memset(data2, (char)0x0, 4096);

    wrtie_eeprom(0x0100, data, size);
    //read_eeprom(0x0000, data2, size);
    //LOG_INF("final data2 ");
    //for(j=0;j<size;j++)
    //	LOG_INF(" %d\n",data2[j]);

#endif
   /*****************************************************************************
	0x3098[0:1] pll3_prediv
	pll3_prediv_map[] = {2, 3, 4, 6}

	0x3099[0:4] pll3_multiplier
	pll3_multiplier

	0x309C[0] pll3_rdiv
	pll3_rdiv + 1

	0x309A[0:3] pll3_sys_div
	pll3_sys_div + 1

	0x309B[0:1] pll3_div
	pll3_div[] = {2, 2, 4, 5}

	VCO = XVCLK * 2 / pll3_prediv * pll3_multiplier * pll3_rdiv
	sysclk = VCO * 2 * 2 / pll3_sys_div / pll3_div

	XVCLK = 24 MHZ
	0x3098, 0x03
	0x3099, 0x1e
	0x309a, 0x02
	0x309b, 0x01
	0x309c, 0x00


	VCO = 24 * 2 / 6 * 31 * 1
	sysclk = VCO * 2  * 2 / 3 / 2
	sysclk = 160 MHZ
	*/


   write_cmos_sensor(0x6010,0x0001);	  //Reset
   msleep(3);
#if MULTI_WRITE

	if(imgsensor_info.sensor_output_dataformat == SENSOR_OUTPUT_FORMAT_RAW_RWB_Wr)
		S5K2P8MIPI_table_write_cmos_sensor(addr_data_pair_init_RWB, sizeof(addr_data_pair_init_RWB)/sizeof(kal_uint16));
	else
		S5K2P8MIPI_table_write_cmos_sensor(addr_data_pair_init, sizeof(addr_data_pair_init)/sizeof(kal_uint16));
#else
   write_cmos_sensor(0x6214,0x7970);	  //open all clocks
   write_cmos_sensor(0x6218,0x7150);	  //open all clocks
   write_cmos_sensor(0x6028,0x2000);
   write_cmos_sensor(0x602A,0x2E00);
   write_cmos_sensor(0x6F12,0x0448);
   write_cmos_sensor(0x6F12,0x0349);
   write_cmos_sensor(0x6F12,0x0160);
   write_cmos_sensor(0x6F12,0xC26A);
   write_cmos_sensor(0x6F12,0x511A);
   write_cmos_sensor(0x6F12,0x8180);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0x10B9);
   write_cmos_sensor(0x6F12,0x2000);
   write_cmos_sensor(0x6F12,0x315C);
   write_cmos_sensor(0x6F12,0x2000);
   write_cmos_sensor(0x6F12,0x1AE0);
   write_cmos_sensor(0x6F12,0x0000);
   write_cmos_sensor(0x6F12,0x0000);
   write_cmos_sensor(0x6F12,0x0000);
   write_cmos_sensor(0x6F12,0x0000);
   write_cmos_sensor(0x6F12,0x2DE9);
   write_cmos_sensor(0x6F12,0xF041);
   write_cmos_sensor(0x6F12,0x0646);
   write_cmos_sensor(0x6F12,0x9A48);
   write_cmos_sensor(0x6F12,0x0F46);
   write_cmos_sensor(0x6F12,0x9046);
   write_cmos_sensor(0x6F12,0x4068);
   write_cmos_sensor(0x6F12,0x0022);
   write_cmos_sensor(0x6F12,0x85B2);
   write_cmos_sensor(0x6F12,0x040C);
   write_cmos_sensor(0x6F12,0x2946);
   write_cmos_sensor(0x6F12,0x2046);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0x4CF9);
   write_cmos_sensor(0x6F12,0x4246);
   write_cmos_sensor(0x6F12,0x3946);
   write_cmos_sensor(0x6F12,0x3046);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0x4CF9);
   write_cmos_sensor(0x6F12,0x9348);
   write_cmos_sensor(0x6F12,0x807E);
   write_cmos_sensor(0x6F12,0x28B1);
   write_cmos_sensor(0x6F12,0x9248);
   write_cmos_sensor(0x6F12,0x90F8);
   write_cmos_sensor(0x6F12,0xFA00);
   write_cmos_sensor(0x6F12,0x08B1);
   write_cmos_sensor(0x6F12,0x0122);
   write_cmos_sensor(0x6F12,0x00E0);
   write_cmos_sensor(0x6F12,0x0022);
   write_cmos_sensor(0x6F12,0x9048);
   write_cmos_sensor(0x6F12,0x0280);
   write_cmos_sensor(0x6F12,0x2946);
   write_cmos_sensor(0x6F12,0x2046);
   write_cmos_sensor(0x6F12,0xBDE8);
   write_cmos_sensor(0x6F12,0xF041);
   write_cmos_sensor(0x6F12,0x0122);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0x34B9);
   write_cmos_sensor(0x6F12,0x8C4A);
   write_cmos_sensor(0x6F12,0x10B5);
   write_cmos_sensor(0x6F12,0x1268);
   write_cmos_sensor(0x6F12,0xB2F8);
   write_cmos_sensor(0x6F12,0xC230);
   write_cmos_sensor(0x6F12,0x874A);
   write_cmos_sensor(0x6F12,0x546B);
   write_cmos_sensor(0x6F12,0xB2F8);
   write_cmos_sensor(0x6F12,0x2E21);
   write_cmos_sensor(0x6F12,0x6409);
   write_cmos_sensor(0x6F12,0x6343);
   write_cmos_sensor(0x6F12,0x4FF4);
   write_cmos_sensor(0x6F12,0x7A74);
   write_cmos_sensor(0x6F12,0x6243);
   write_cmos_sensor(0x6F12,0x5209);
   write_cmos_sensor(0x6F12,0xB3FB);
   write_cmos_sensor(0x6F12,0xF2F3);
   write_cmos_sensor(0x6F12,0x021D);
   write_cmos_sensor(0x6F12,0x8A42);
   write_cmos_sensor(0x6F12,0x02D2);
   write_cmos_sensor(0x6F12,0x0A1A);
   write_cmos_sensor(0x6F12,0x121F);
   write_cmos_sensor(0x6F12,0x00E0);
   write_cmos_sensor(0x6F12,0x0022);
   write_cmos_sensor(0x6F12,0x9A42);
   write_cmos_sensor(0x6F12,0x00D8);
   write_cmos_sensor(0x6F12,0x1A46);
   write_cmos_sensor(0x6F12,0x8048);
   write_cmos_sensor(0x6F12,0x0280);
   write_cmos_sensor(0x6F12,0x10BD);
   write_cmos_sensor(0x6F12,0x2DE9);
   write_cmos_sensor(0x6F12,0xF34F);
   write_cmos_sensor(0x6F12,0x0446);
   write_cmos_sensor(0x6F12,0x7848);
   write_cmos_sensor(0x6F12,0x83B0);
   write_cmos_sensor(0x6F12,0x0022);
   write_cmos_sensor(0x6F12,0xC068);
   write_cmos_sensor(0x6F12,0x010C);
   write_cmos_sensor(0x6F12,0x80B2);
   write_cmos_sensor(0x6F12,0xCDE9);
   write_cmos_sensor(0x6F12,0x0001);
   write_cmos_sensor(0x6F12,0x0146);
   write_cmos_sensor(0x6F12,0x0198);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0x07F9);
   write_cmos_sensor(0x6F12,0xDFF8);
   write_cmos_sensor(0x6F12,0xE091);
   write_cmos_sensor(0x6F12,0xDFF8);
   write_cmos_sensor(0x6F12,0xD4B1);
   write_cmos_sensor(0x6F12,0x0021);
   write_cmos_sensor(0x6F12,0x99F8);
   write_cmos_sensor(0x6F12,0x2A70);
   write_cmos_sensor(0x6F12,0x89F8);
   write_cmos_sensor(0x6F12,0x2A10);
   write_cmos_sensor(0x6F12,0xDFF8);
   write_cmos_sensor(0x6F12,0xBCA1);
   write_cmos_sensor(0x6F12,0x734D);
   write_cmos_sensor(0x6F12,0xDBF8);
   write_cmos_sensor(0x6F12,0x0000);
   write_cmos_sensor(0x6F12,0x9AF8);
   write_cmos_sensor(0x6F12,0xFA80);
   write_cmos_sensor(0x6F12,0xAE8A);
   write_cmos_sensor(0x6F12,0xB0F8);
   write_cmos_sensor(0x6F12,0xC400);
   write_cmos_sensor(0x6F12,0x20B1);
   write_cmos_sensor(0x6F12,0xA08A);
   write_cmos_sensor(0x6F12,0x296E);
   write_cmos_sensor(0x6F12,0x4843);
   write_cmos_sensor(0x6F12,0x000B);
   write_cmos_sensor(0x6F12,0xA882);
   write_cmos_sensor(0x6F12,0x2046);
   write_cmos_sensor(0x6F12,0x0499);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0xF4F8);
   write_cmos_sensor(0x6F12,0xAE82);
   write_cmos_sensor(0x6F12,0xB9F8);
   write_cmos_sensor(0x6F12,0x1C00);
   write_cmos_sensor(0x6F12,0x9AF8);
   write_cmos_sensor(0x6F12,0xFA50);
   write_cmos_sensor(0x6F12,0xC0F3);
   write_cmos_sensor(0x6F12,0x0030);
   write_cmos_sensor(0x6F12,0x4545);
   write_cmos_sensor(0x6F12,0x08D0);
   write_cmos_sensor(0x6F12,0x38B1);
   write_cmos_sensor(0x6F12,0xDBF8);
   write_cmos_sensor(0x6F12,0x0000);
   write_cmos_sensor(0x6F12,0xB0F8);
   write_cmos_sensor(0x6F12,0xC600);
   write_cmos_sensor(0x6F12,0x10B1);
   write_cmos_sensor(0x6F12,0x0020);
   write_cmos_sensor(0x6F12,0xCAF8);
   write_cmos_sensor(0x6F12,0x0001);
   write_cmos_sensor(0x6F12,0x17F0);
   write_cmos_sensor(0x6F12,0xFF00);
   write_cmos_sensor(0x6F12,0x89F8);
   write_cmos_sensor(0x6F12,0x2A00);
   write_cmos_sensor(0x6F12,0x03D0);
   write_cmos_sensor(0x6F12,0xA18A);
   write_cmos_sensor(0x6F12,0x2068);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0xDEF8);
   write_cmos_sensor(0x6F12,0xDDE9);
   write_cmos_sensor(0x6F12,0x0010);
   write_cmos_sensor(0x6F12,0x05B0);
   write_cmos_sensor(0x6F12,0x0122);
   write_cmos_sensor(0x6F12,0xBDE8);
   write_cmos_sensor(0x6F12,0xF04F);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0xC7B8);
   write_cmos_sensor(0x6F12,0x70B5);
   write_cmos_sensor(0x6F12,0x0446);
   write_cmos_sensor(0x6F12,0x5148);
   write_cmos_sensor(0x6F12,0x0022);
   write_cmos_sensor(0x6F12,0x0169);
   write_cmos_sensor(0x6F12,0x0D0C);
   write_cmos_sensor(0x6F12,0x8EB2);
   write_cmos_sensor(0x6F12,0x3146);
   write_cmos_sensor(0x6F12,0x2846);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0xBCF8);
   write_cmos_sensor(0x6F12,0x2046);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0xCDF8);
   write_cmos_sensor(0x6F12,0x94F8);
   write_cmos_sensor(0x6F12,0x6000);
   write_cmos_sensor(0x6F12,0x0128);
   write_cmos_sensor(0x6F12,0x07D1);
   write_cmos_sensor(0x6F12,0x5148);
   write_cmos_sensor(0x6F12,0x0068);
   write_cmos_sensor(0x6F12,0x8078);
   write_cmos_sensor(0x6F12,0xF528);
   write_cmos_sensor(0x6F12,0x02D0);
   write_cmos_sensor(0x6F12,0x0020);
   write_cmos_sensor(0x6F12,0x84F8);
   write_cmos_sensor(0x6F12,0x6000);
   write_cmos_sensor(0x6F12,0x3146);
   write_cmos_sensor(0x6F12,0x2846);
   write_cmos_sensor(0x6F12,0xBDE8);
   write_cmos_sensor(0x6F12,0x7040);
   write_cmos_sensor(0x6F12,0x0122);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0xA6B8);
   write_cmos_sensor(0x6F12,0x70B5);
   write_cmos_sensor(0x6F12,0x0446);
   write_cmos_sensor(0x6F12,0x4048);
   write_cmos_sensor(0x6F12,0x0022);
   write_cmos_sensor(0x6F12,0x4069);
   write_cmos_sensor(0x6F12,0x86B2);
   write_cmos_sensor(0x6F12,0x050C);
   write_cmos_sensor(0x6F12,0x3146);
   write_cmos_sensor(0x6F12,0x2846);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0x9BF8);
   write_cmos_sensor(0x6F12,0x2046);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0xB1F8);
   write_cmos_sensor(0x6F12,0x207C);
   write_cmos_sensor(0x6F12,0x40B1);
   write_cmos_sensor(0x6F12,0x424A);
   write_cmos_sensor(0x6F12,0xA168);
   write_cmos_sensor(0x6F12,0x2068);
   write_cmos_sensor(0x6F12,0x92F8);
   write_cmos_sensor(0x6F12,0xB921);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0xADF8);
   write_cmos_sensor(0x6F12,0x4049);
   write_cmos_sensor(0x6F12,0x0880);
   write_cmos_sensor(0x6F12,0x3146);
   write_cmos_sensor(0x6F12,0x2846);
   write_cmos_sensor(0x6F12,0xBDE8);
   write_cmos_sensor(0x6F12,0x7040);
   write_cmos_sensor(0x6F12,0x0122);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0x86B8);
   write_cmos_sensor(0x6F12,0x2DE9);
   write_cmos_sensor(0x6F12,0xF041);
   write_cmos_sensor(0x6F12,0x304C);
   write_cmos_sensor(0x6F12,0x8046);
   write_cmos_sensor(0x6F12,0x0022);
   write_cmos_sensor(0x6F12,0xA069);
   write_cmos_sensor(0x6F12,0x87B2);
   write_cmos_sensor(0x6F12,0x060C);
   write_cmos_sensor(0x6F12,0x3946);
   write_cmos_sensor(0x6F12,0x3046);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0x7AF8);
   write_cmos_sensor(0x6F12,0x2F4D);
   write_cmos_sensor(0x6F12,0xB8F1);
   write_cmos_sensor(0x6F12,0x000F);
   write_cmos_sensor(0x6F12,0x0BD1);
   write_cmos_sensor(0x6F12,0x3248);
   write_cmos_sensor(0x6F12,0x0078);
   write_cmos_sensor(0x6F12,0x40B1);
   write_cmos_sensor(0x6F12,0x2868);
   write_cmos_sensor(0x6F12,0xB0F8);
   write_cmos_sensor(0x6F12,0x6A00);
   write_cmos_sensor(0x6F12,0x20B1);
   write_cmos_sensor(0x6F12,0x2188);
   write_cmos_sensor(0x6F12,0x8842);
   write_cmos_sensor(0x6F12,0x01D0);
   write_cmos_sensor(0x6F12,0x0120);
   write_cmos_sensor(0x6F12,0x00E0);
   write_cmos_sensor(0x6F12,0x0020);
   write_cmos_sensor(0x6F12,0x38B1);
   write_cmos_sensor(0x6F12,0x2449);
   write_cmos_sensor(0x6F12,0x0020);
   write_cmos_sensor(0x6F12,0xC1F8);
   write_cmos_sensor(0x6F12,0xDC01);
   write_cmos_sensor(0x6F12,0x81F8);
   write_cmos_sensor(0x6F12,0xCC01);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0x83F8);
   write_cmos_sensor(0x6F12,0x2868);
   write_cmos_sensor(0x6F12,0xB0F8);
   write_cmos_sensor(0x6F12,0x6A00);
   write_cmos_sensor(0x6F12,0x2080);
   write_cmos_sensor(0x6F12,0x4046);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0x81F8);
   write_cmos_sensor(0x6F12,0x3946);
   write_cmos_sensor(0x6F12,0x3046);
   write_cmos_sensor(0x6F12,0xBDE8);
   write_cmos_sensor(0x6F12,0xF041);
   write_cmos_sensor(0x6F12,0x0122);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0x52B8);
   write_cmos_sensor(0x6F12,0x10B5);
   write_cmos_sensor(0x6F12,0x0022);
   write_cmos_sensor(0x6F12,0xAFF2);
   write_cmos_sensor(0x6F12,0x1721);
   write_cmos_sensor(0x6F12,0x2048);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0x78F8);
   write_cmos_sensor(0x6F12,0x144C);
   write_cmos_sensor(0x6F12,0x0022);
   write_cmos_sensor(0x6F12,0xAFF2);
   write_cmos_sensor(0x6F12,0xD711);
   write_cmos_sensor(0x6F12,0x6060);
   write_cmos_sensor(0x6F12,0x1D48);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0x70F8);
   write_cmos_sensor(0x6F12,0x0022);
   write_cmos_sensor(0x6F12,0xAFF2);
   write_cmos_sensor(0x6F12,0x0D11);
   write_cmos_sensor(0x6F12,0xA060);
   write_cmos_sensor(0x6F12,0x1B48);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0x69F8);
   write_cmos_sensor(0x6F12,0x0022);
   write_cmos_sensor(0x6F12,0xAFF2);
   write_cmos_sensor(0x6F12,0xB711);
   write_cmos_sensor(0x6F12,0x2061);
   write_cmos_sensor(0x6F12,0x1848);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0x62F8);
   write_cmos_sensor(0x6F12,0x0022);
   write_cmos_sensor(0x6F12,0xAFF2);
   write_cmos_sensor(0x6F12,0xE701);
   write_cmos_sensor(0x6F12,0xE060);
   write_cmos_sensor(0x6F12,0x1648);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0x5BF8);
   write_cmos_sensor(0x6F12,0x0022);
   write_cmos_sensor(0x6F12,0xAFF2);
   write_cmos_sensor(0x6F12,0xB301);
   write_cmos_sensor(0x6F12,0x6061);
   write_cmos_sensor(0x6F12,0x1348);
   write_cmos_sensor(0x6F12,0x00F0);
   write_cmos_sensor(0x6F12,0x54F8);
   write_cmos_sensor(0x6F12,0xA061);
   write_cmos_sensor(0x6F12,0x0020);
   write_cmos_sensor(0x6F12,0x2080);
   write_cmos_sensor(0x6F12,0x10BD);
   write_cmos_sensor(0x6F12,0x0000);
   write_cmos_sensor(0x6F12,0x2000);
   write_cmos_sensor(0x6F12,0x3140);
   write_cmos_sensor(0x6F12,0x2000);
   write_cmos_sensor(0x6F12,0x1B10);
   write_cmos_sensor(0x6F12,0x2000);
   write_cmos_sensor(0x6F12,0x2530);
   write_cmos_sensor(0x6F12,0x4000);
   write_cmos_sensor(0x6F12,0x9802);
   write_cmos_sensor(0x6F12,0x2000);
   write_cmos_sensor(0x6F12,0x0550);
   write_cmos_sensor(0x6F12,0x4000);
   write_cmos_sensor(0x6F12,0xF000);
   write_cmos_sensor(0x6F12,0x2000);
   write_cmos_sensor(0x6F12,0x1300);
   write_cmos_sensor(0x6F12,0x2000);
   write_cmos_sensor(0x6F12,0x2890);
   write_cmos_sensor(0x6F12,0x2000);
   write_cmos_sensor(0x6F12,0x02C0);
   write_cmos_sensor(0x6F12,0x2000);
   write_cmos_sensor(0x6F12,0x1410);
   write_cmos_sensor(0x6F12,0x4000);
   write_cmos_sensor(0x6F12,0xA358);
   write_cmos_sensor(0x6F12,0x0000);
   write_cmos_sensor(0x6F12,0x15F7);
   write_cmos_sensor(0x6F12,0x0000);
   write_cmos_sensor(0x6F12,0x7A4F);
   write_cmos_sensor(0x6F12,0x0000);
   write_cmos_sensor(0x6F12,0x4A69);
   write_cmos_sensor(0x6F12,0x0000);
   write_cmos_sensor(0x6F12,0x1349);
   write_cmos_sensor(0x6F12,0x0000);
   write_cmos_sensor(0x6F12,0x3091);
   write_cmos_sensor(0x6F12,0x0000);
   write_cmos_sensor(0x6F12,0x6057);
   write_cmos_sensor(0x6F12,0x40F2);
   write_cmos_sensor(0x6F12,0xE96C);
   write_cmos_sensor(0x6F12,0xC0F2);
   write_cmos_sensor(0x6F12,0x000C);
   write_cmos_sensor(0x6F12,0x6047);
   write_cmos_sensor(0x6F12,0x41F2);
   write_cmos_sensor(0x6F12,0xF75C);
   write_cmos_sensor(0x6F12,0xC0F2);
   write_cmos_sensor(0x6F12,0x000C);
   write_cmos_sensor(0x6F12,0x6047);
   write_cmos_sensor(0x6F12,0x41F2);
   write_cmos_sensor(0x6F12,0x493C);
   write_cmos_sensor(0x6F12,0xC0F2);
   write_cmos_sensor(0x6F12,0x000C);
   write_cmos_sensor(0x6F12,0x6047);
   write_cmos_sensor(0x6F12,0x41F2);
   write_cmos_sensor(0x6F12,0x9D1C);
   write_cmos_sensor(0x6F12,0xC0F2);
   write_cmos_sensor(0x6F12,0x000C);
   write_cmos_sensor(0x6F12,0x6047);
   write_cmos_sensor(0x6F12,0x44F6);
   write_cmos_sensor(0x6F12,0x692C);
   write_cmos_sensor(0x6F12,0xC0F2);
   write_cmos_sensor(0x6F12,0x000C);
   write_cmos_sensor(0x6F12,0x6047);
   write_cmos_sensor(0x6F12,0x43F2);
   write_cmos_sensor(0x6F12,0x910C);
   write_cmos_sensor(0x6F12,0xC0F2);
   write_cmos_sensor(0x6F12,0x000C);
   write_cmos_sensor(0x6F12,0x6047);
   write_cmos_sensor(0x6F12,0x40F2);
   write_cmos_sensor(0x6F12,0x156C);
   write_cmos_sensor(0x6F12,0xC0F2);
   write_cmos_sensor(0x6F12,0x000C);
   write_cmos_sensor(0x6F12,0x6047);
   write_cmos_sensor(0x6F12,0x42F2);
   write_cmos_sensor(0x6F12,0xC71C);
   write_cmos_sensor(0x6F12,0xC0F2);
   write_cmos_sensor(0x6F12,0x000C);
   write_cmos_sensor(0x6F12,0x6047);
   write_cmos_sensor(0x6F12,0x46F2);
   write_cmos_sensor(0x6F12,0x570C);
   write_cmos_sensor(0x6F12,0xC0F2);
   write_cmos_sensor(0x6F12,0x000C);
   write_cmos_sensor(0x6F12,0x6047);
   write_cmos_sensor(0x6F12,0x4AF6);
   write_cmos_sensor(0x6F12,0x850C);
   write_cmos_sensor(0x6F12,0xC0F2);
   write_cmos_sensor(0x6F12,0x000C);
   write_cmos_sensor(0x6F12,0x6047);
   write_cmos_sensor(0x3EA6,0xFFFF);	   //fe_isp_dadlc_analog_gain_thrshold_for_ladlc_low
   write_cmos_sensor(0x3EA8,0xFFFF);	   //fe_isp_dadlc_analog_gain_thrshold_for_ladlc_high
   write_cmos_sensor(0x3A56,0x3BB8);	  //Sampling enable
   write_cmos_sensor(0xF476,0x0040);	  //DBR
   write_cmos_sensor(0xF40C,0x1180);	  //CLP on, LDB off
   write_cmos_sensor(0xF480,0x0015);	  //CLP LVL
   write_cmos_sensor(0x39EE,0x0507);
   write_cmos_sensor(0x3A06,0x0605);
   write_cmos_sensor(0x39D0,0x0707);
   write_cmos_sensor(0xF432,0x0100);	  //DBS
   write_cmos_sensor(0xF4A6,0x00C0);	  //RDV
   write_cmos_sensor(0xF42E,0x0060);	  //CDS
   write_cmos_sensor(0xF49C,0x0000);	  //RMP
   write_cmos_sensor(0xF496,0x0000);	  //REF
   write_cmos_sensor(0xF49E,0x005E);	  //ADC_SAT
   write_cmos_sensor(0xF47A,0x0017);	  //VRD 3.2V
   write_cmos_sensor(0xF462,0x0000);	  //VPIX 2.92V
   write_cmos_sensor(0xF460,0x0020);	  //VTG 3.8V
   write_cmos_sensor(0x3A64,0x0010);
   write_cmos_sensor(0x3A66,0x0010);
   write_cmos_sensor(0x3A68,0x0010);
   write_cmos_sensor(0xF426,0x0007);	  //LAT START
   write_cmos_sensor(0xF428,0x0004);	  //LAT WIDTH
   write_cmos_sensor(0xF42A,0x0004);	  //HOLD START
   write_cmos_sensor(0xF42C,0x0004);	  //HOLD WIDTH
   write_cmos_sensor(0x32BC,0x000D);	  //pixel boost
   write_cmos_sensor(0x3298,0x005E);
   write_cmos_sensor(0x3880,0x010B);
   write_cmos_sensor(0x3886,0x010A);
   write_cmos_sensor(0x388C,0x021A);
   write_cmos_sensor(0x389E,0x021C);
   write_cmos_sensor(0x38A4,0x0000);
   write_cmos_sensor(0x38AA,0x015C);
   write_cmos_sensor(0x38B0,0x015B);
   write_cmos_sensor(0x38B6,0x021A);
   write_cmos_sensor(0x38C2,0x0070);
   write_cmos_sensor(0x38C8,0x021C);
   write_cmos_sensor(0x38CE,0x0000);
   write_cmos_sensor(0x3976,0x0006);
   write_cmos_sensor(0x31C0,0x0C40);	  //default 0004
   write_cmos_sensor(0x31C2,0x0C48);	  //default 0B1C
   write_cmos_sensor(0x31B2,0x080C);
   write_cmos_sensor(0x31B4,0x0401);
   write_cmos_sensor(0x31B2,0x0804);
   write_cmos_sensor(0x39D6,0x0F0F);
   write_cmos_sensor(0x39DC,0x3030);
   write_cmos_sensor(0x39B0,0x0000);
   write_cmos_sensor(0x30C2,0x0300);
   write_cmos_sensor(0x3AF2,0x0000);
   write_cmos_sensor(0x3AF4,0x0000);
   write_cmos_sensor(0x3AF6,0x0000);
   write_cmos_sensor(0x3AF8,0x0000);
   write_cmos_sensor(0x3AFA,0x0000);
   write_cmos_sensor(0x3AFC,0x0000);
   write_cmos_sensor(0x3AFE,0x0000);
   write_cmos_sensor(0x3B00,0x0000);
   write_cmos_sensor(0x3B02,0x0000);
   write_cmos_sensor(0x3B04,0x0000);
   write_cmos_sensor(0x3B06,0x0000);
   write_cmos_sensor(0x3B08,0x0000);
   write_cmos_sensor(0x3B0A,0x0000);
   write_cmos_sensor(0x3B0C,0x0000);
   write_cmos_sensor(0x3B0E,0x0000);
   write_cmos_sensor(0x3B10,0x0000);
   write_cmos_sensor(0x3B12,0x0000);
   write_cmos_sensor(0x3B14,0x0000);
   write_cmos_sensor(0x3B16,0x0000);
   write_cmos_sensor(0x3B18,0x0000);
   write_cmos_sensor(0x3B1A,0x0000);
   write_cmos_sensor(0x3B1C,0x0000);
   write_cmos_sensor(0x3B1E,0x0000);
   write_cmos_sensor(0x3B20,0x0000);
   write_cmos_sensor(0x3B22,0x0000);
   write_cmos_sensor(0x3B24,0x0000);
   write_cmos_sensor(0x3B26,0x0000);
   write_cmos_sensor(0x3B28,0x0000);
   write_cmos_sensor(0x3B2A,0x0000);
   write_cmos_sensor(0x3B2C,0x0000);
   write_cmos_sensor(0x3B2E,0x0000);
   write_cmos_sensor(0x3B30,0x0000);
   write_cmos_sensor(0x3B32,0x0000);
   write_cmos_sensor(0x3B34,0x0000);
   write_cmos_sensor(0x3B36,0x0000);
   write_cmos_sensor(0x3B38,0x0000);
   write_cmos_sensor(0x3B3A,0x0000);
   write_cmos_sensor(0x3B3C,0x0000);
   write_cmos_sensor(0x3B3E,0x0000);
   write_cmos_sensor(0x3B40,0x0000);
   write_cmos_sensor(0x3B42,0x0000);
   write_cmos_sensor(0x3B44,0x0000);
   write_cmos_sensor(0x3B46,0x0000);
   write_cmos_sensor(0x3B48,0x0000);
   write_cmos_sensor(0x3B4A,0x0000);
   write_cmos_sensor(0x3B4C,0x0000);
   write_cmos_sensor(0x3B4E,0x0000);
   write_cmos_sensor(0x3B50,0x0000);
   write_cmos_sensor(0x3B52,0x0000);
   write_cmos_sensor(0x3B54,0x0000);
   write_cmos_sensor(0x3B56,0x0000);
   write_cmos_sensor(0x3B58,0x0000);
   write_cmos_sensor(0x3B5A,0x0000);
   write_cmos_sensor(0x3B5C,0x0000);
   write_cmos_sensor(0x3B5E,0x0000);
   write_cmos_sensor(0x3B60,0x0000);
   write_cmos_sensor(0x3B62,0x0000);
   write_cmos_sensor(0x3B64,0x0000);
   write_cmos_sensor(0x3B66,0x0000);
   write_cmos_sensor(0x3B68,0x0000);
   write_cmos_sensor(0x3B6A,0x0000);
   write_cmos_sensor(0x3B6C,0x0000);
   write_cmos_sensor(0x3B6E,0x0000);
   write_cmos_sensor(0x3B70,0x0000);
   write_cmos_sensor(0x3B72,0x0000);
   write_cmos_sensor(0x3B74,0x0000);
   write_cmos_sensor(0x3B76,0x0000);
   write_cmos_sensor(0x3B78,0x0000);
   write_cmos_sensor(0x3B7A,0x0000);
   write_cmos_sensor(0x3B7C,0x0000);
   write_cmos_sensor(0x3B7E,0x0000);
   write_cmos_sensor(0x3B80,0x0000);
   write_cmos_sensor(0x3B82,0x0000);
   write_cmos_sensor(0x3B84,0x0000);
   write_cmos_sensor(0x3B86,0x0000);
   write_cmos_sensor(0x3B88,0x0000);
   write_cmos_sensor(0x3B8A,0x0000);
   write_cmos_sensor(0x3B8C,0x0000);
   write_cmos_sensor(0x3B8E,0x0000);
   write_cmos_sensor(0x3B90,0x0000);
   write_cmos_sensor(0x3B92,0x0000);
   write_cmos_sensor(0x3B94,0x0000);
   write_cmos_sensor(0x3B96,0x0000);
   write_cmos_sensor(0x3B98,0x0000);
   write_cmos_sensor(0x3B9A,0x0000);
   write_cmos_sensor(0x3B9C,0x0000);
   write_cmos_sensor(0x3B9E,0x0000);
   write_cmos_sensor(0x3BA0,0x0000);
   write_cmos_sensor(0x3BA2,0x0000);
   write_cmos_sensor(0x3BA4,0x0000);
   write_cmos_sensor(0x3BA6,0x0000);
   write_cmos_sensor(0x3BA8,0x0000);
   write_cmos_sensor(0x3BAA,0x0000);
   write_cmos_sensor(0x3BAC,0x0000);
   write_cmos_sensor(0x3BAE,0x0000);
   write_cmos_sensor(0x3BB0,0x0000);
   write_cmos_sensor(0x3BB2,0x0000);
   write_cmos_sensor(0x3BB4,0x0000);
   write_cmos_sensor(0x3BB6,0x0000);
   write_cmos_sensor(0x3BB8,0x0000);
   write_cmos_sensor(0x3BBA,0x0000);
   write_cmos_sensor(0x3BBC,0x0000);
   write_cmos_sensor(0x3BBE,0x0000);
   write_cmos_sensor(0x3BC0,0x0000);
   write_cmos_sensor(0x3BC2,0x0000);
   write_cmos_sensor(0x3BC4,0x0000);
   write_cmos_sensor(0x3BC6,0x0000);
   write_cmos_sensor(0x3BC8,0x0000);
   write_cmos_sensor(0x3BCA,0x0000);
   write_cmos_sensor(0x3BCC,0x0000);
   write_cmos_sensor(0x3BCE,0x0000);
   write_cmos_sensor(0x3BD0,0x0000);
   write_cmos_sensor(0x3BD2,0x0000);
   write_cmos_sensor(0x3BD4,0x0000);
   write_cmos_sensor(0x3BD6,0x0000);
   write_cmos_sensor(0x3BD8,0x0000);
   write_cmos_sensor(0x3BDA,0x0000);
   write_cmos_sensor(0x3BDC,0x0000);
   write_cmos_sensor(0x3BDE,0x0000);
   write_cmos_sensor(0x3BE0,0x0000);
   write_cmos_sensor(0x3BE2,0x0000);
   write_cmos_sensor(0x3BE4,0x0000);
   write_cmos_sensor(0x3BE6,0x0000);
   write_cmos_sensor(0x3BE8,0x0000);
   write_cmos_sensor(0x3BEA,0x0000);
   write_cmos_sensor(0x3BEC,0x0000);
   write_cmos_sensor(0x3BEE,0x0000);
   write_cmos_sensor(0x3BF0,0x0000);
   write_cmos_sensor(0x3BF2,0x0000);
   write_cmos_sensor(0x3BF4,0x0000);
   write_cmos_sensor(0x3BF6,0x0000);
   write_cmos_sensor(0x3BF8,0x0000);
   write_cmos_sensor(0x3BFA,0x0000);
   write_cmos_sensor(0x3BFC,0x0000);
   write_cmos_sensor(0x3BFE,0x0000);
   write_cmos_sensor(0x3C00,0x0000);
   write_cmos_sensor(0x3C02,0x0000);
   write_cmos_sensor(0x3C04,0x0000);
   write_cmos_sensor(0x3C06,0x0000);
   write_cmos_sensor(0x3C08,0x0000);
   write_cmos_sensor(0x3C0A,0x0000);
   write_cmos_sensor(0x3C0C,0x0000);
   write_cmos_sensor(0x3C0E,0x0000);
   write_cmos_sensor(0x3C10,0x0000);
   write_cmos_sensor(0x3C12,0x0000);
   write_cmos_sensor(0x3C14,0x0000);
   write_cmos_sensor(0x3C16,0x0000);
   write_cmos_sensor(0x3C18,0x0000);
   write_cmos_sensor(0x3C1A,0x0000);
   write_cmos_sensor(0x3C1C,0x0000);
   write_cmos_sensor(0x3C1E,0x0000);
   write_cmos_sensor(0x3C20,0x0000);
   write_cmos_sensor(0x3C22,0x0000);
   write_cmos_sensor(0x3C24,0x0000);
   write_cmos_sensor(0x3C26,0x0000);
   write_cmos_sensor(0x3C28,0x0000);
   write_cmos_sensor(0x3C2A,0x0000);
   write_cmos_sensor(0x3C2C,0x0000);
   write_cmos_sensor(0x3C2E,0x0000);
   write_cmos_sensor(0x3C30,0x0000);
   write_cmos_sensor(0x3C32,0x0000);
   write_cmos_sensor(0x3C34,0x0000);
   write_cmos_sensor(0x3C36,0x0000);
   write_cmos_sensor(0x3C38,0x0000);
   write_cmos_sensor(0x3C3A,0x0000);
   write_cmos_sensor(0x3C3C,0x0000);
   write_cmos_sensor(0x3C3E,0x0000);
   write_cmos_sensor(0x3C40,0x0000);
   write_cmos_sensor(0x3C42,0x0000);
   write_cmos_sensor(0x3C44,0x0000);
   write_cmos_sensor(0x3C46,0x0000);
   write_cmos_sensor(0x3C48,0x0000);
   write_cmos_sensor(0x3C4A,0x0000);
   write_cmos_sensor(0x3C4C,0x0000);
   write_cmos_sensor(0x3C4E,0x0000);
   write_cmos_sensor(0x3C50,0x0000);
   write_cmos_sensor(0x3C52,0x0000);
   write_cmos_sensor(0x3C54,0x0000);
   write_cmos_sensor(0x3C56,0x0000);
   write_cmos_sensor(0x3C58,0x0000);
   write_cmos_sensor(0x3C5A,0x0000);
   write_cmos_sensor(0x3C5C,0x0000);
   write_cmos_sensor(0x3C5E,0x0000);
   write_cmos_sensor(0x3C60,0x0000);
   write_cmos_sensor(0x3C62,0x0000);
   write_cmos_sensor(0x3C64,0x0000);
   write_cmos_sensor(0x3C66,0x0000);
   write_cmos_sensor(0x3C68,0x0000);
   write_cmos_sensor(0x3C6A,0x0000);
   write_cmos_sensor(0x3C6C,0x0000);
   write_cmos_sensor(0x3C6E,0x0000);
   write_cmos_sensor(0x3C70,0x0000);
   write_cmos_sensor(0x3C72,0x0000);
   write_cmos_sensor(0x3C74,0x0000);
   write_cmos_sensor(0x3C76,0x0000);
   write_cmos_sensor(0x3C78,0x0000);
   write_cmos_sensor(0x3C7A,0x0000);
   write_cmos_sensor(0x3C7C,0x0000);
   write_cmos_sensor(0x3C7E,0x0000);
   write_cmos_sensor(0x3C80,0x0000);
   write_cmos_sensor(0x3C82,0x0000);
   write_cmos_sensor(0x3C84,0x0000);
   write_cmos_sensor(0x3C86,0x0000);
   write_cmos_sensor(0x3C88,0x0000);
   write_cmos_sensor(0x3C8A,0x0000);
   write_cmos_sensor(0x3C8C,0x0000);
   write_cmos_sensor(0x3C8E,0x0000);
   write_cmos_sensor(0x3C90,0x0000);
   write_cmos_sensor(0x3C92,0x0000);
   write_cmos_sensor(0x3C94,0x0000);
   write_cmos_sensor(0x3C96,0x0000);
   write_cmos_sensor(0x3C98,0x0000);
   write_cmos_sensor(0x3C9A,0x0000);
   write_cmos_sensor(0x3C9C,0x0000);
   write_cmos_sensor(0x3C9E,0x0000);
   write_cmos_sensor(0x3CA0,0x0000);
   write_cmos_sensor(0x3CA2,0x0000);
   write_cmos_sensor(0x3CA4,0x0000);
   write_cmos_sensor(0x3CA6,0x0000);
   write_cmos_sensor(0x3CA8,0x0000);
   write_cmos_sensor(0x3CAA,0x0000);
   write_cmos_sensor(0x3CAC,0x0000);
   write_cmos_sensor(0x3CAE,0x0000);
   write_cmos_sensor(0x3CB0,0x0000);
   write_cmos_sensor(0x3CB2,0x0000);
   write_cmos_sensor(0x3CB4,0x0000);
   write_cmos_sensor(0x3CB6,0x0000);
   write_cmos_sensor(0x3CB8,0x0000);
   write_cmos_sensor(0x3CBA,0x0000);
   write_cmos_sensor(0x3CBC,0x0000);
   write_cmos_sensor(0x3CBE,0x0000);
   write_cmos_sensor(0x3CC0,0x0000);
   write_cmos_sensor(0x3CC2,0x0000);
   write_cmos_sensor(0x3CC4,0x0000);
   write_cmos_sensor(0x3CC6,0x0000);
   write_cmos_sensor(0x3CC8,0x0000);
   write_cmos_sensor(0x3CCA,0x0000);
   write_cmos_sensor(0x3CCC,0x0000);
   write_cmos_sensor(0x3CCE,0x0000);
   write_cmos_sensor(0x3CD0,0x0000);
   write_cmos_sensor(0x3CD2,0x0000);
   write_cmos_sensor(0x3CD4,0x0000);
   write_cmos_sensor(0x3CD6,0x0000);
   write_cmos_sensor(0x3CD8,0x0000);
   write_cmos_sensor(0x3CDA,0x0000);
   write_cmos_sensor(0x3CDC,0x0000);
   write_cmos_sensor(0x3CDE,0x0000);
   write_cmos_sensor(0x3CE0,0x0000);
   write_cmos_sensor(0x3CE2,0x0000);
   write_cmos_sensor(0x3CE4,0x0000);
   write_cmos_sensor(0x3CE6,0x0000);
   write_cmos_sensor(0x3CE8,0x0000);
   write_cmos_sensor(0x3CEA,0x0000);
   write_cmos_sensor(0x3CEC,0x0000);
   write_cmos_sensor(0x3CEE,0x0000);
   write_cmos_sensor(0x3CF0,0x0000);
   write_cmos_sensor(0x3CF2,0x0000);
   write_cmos_sensor(0x3DD4,0x2000);
   write_cmos_sensor(0x3DD6,0x2000);
   write_cmos_sensor(0x3DDC,0x2000);	  //
   write_cmos_sensor(0x3DDE,0x2000);	  //
   write_cmos_sensor(0x3DF4,0x2000);	  //
   write_cmos_sensor(0x3DF6,0x2000);	  //
   write_cmos_sensor(0x3DFC,0x2000);	  //
   write_cmos_sensor(0x3DFE,0x2000);	  //
   write_cmos_sensor(0x3E14,0x2000);	  //
   write_cmos_sensor(0x3E16,0x2000);	  //
   write_cmos_sensor(0x3E1C,0x2000);	  //
   write_cmos_sensor(0x3E1E,0x2000);	  //
   write_cmos_sensor(0x3E76,0x0A04);
   write_cmos_sensor(0x6226,0x0001);	  //Open clock to access ELG memory
   write_cmos_sensor(0x70B6,0x0001);	  //Disable ELG
   write_cmos_sensor(0x3050,0x0002);
   write_cmos_sensor(0x3068,0x0000);
   write_cmos_sensor(0x6028,0x2000);
   write_cmos_sensor(0x602A,0x1410);
   write_cmos_sensor(0x6F12,0x0000);
   write_cmos_sensor(0x602A,0x1416);
   write_cmos_sensor(0x6F12,0x0108);
   write_cmos_sensor(0x6F12,0x0108);
   write_cmos_sensor(0x6F12,0x0A08);
   write_cmos_sensor(0x602A,0x141A); //modify address from 141B to 141A
   write_cmos_sensor(0x6F12,0x0A08);
//S6F120203 : delete this register
   write_cmos_sensor(0x6F12,0x0202);
   write_cmos_sensor(0x6F12,0x0603);
//S6F120603 : delete this register

   write_cmos_sensor(0x602A,0x1412);
   write_cmos_sensor(0x6F12,0x0100);
   write_cmos_sensor(0x3E9E,0x0011);
   write_cmos_sensor(0x3EA2,0x0033);
	//BPC
   write_cmos_sensor(0x6028,0x2000);
   write_cmos_sensor(0x602A,0x1748);
   write_cmos_sensor(0x6F12,0x0101);
   write_cmos_sensor(0x0B04,0x0101);
   write_cmos_sensor(0x306E,0x039C);	  //smiaRegs_vendor_bpc_otp_clusters_address
   write_cmos_sensor(0x3072,0x00FF);	  //smiaRegs_vendor_bpc_max_clusters_in_otp
   write_cmos_sensor(0x30C4,0x0001);	  //smiaRegs_vendor_tnp_use_dgains_for_ladlc_enable
   write_cmos_sensor(0x30C6,0x0001);	  //smiaRegs_vendor_tnp_reset_iir_on_ladlc_on_off
   write_cmos_sensor(0x3E86,0x0104);
   write_cmos_sensor(0x302E,0x0102);
	//Immediate abort
   write_cmos_sensor(0x3028,0x0000); //smiaRegs_vendor_sensor_abort_timing_method_on_rolling_sh
   write_cmos_sensor(0x302A,0x0000);  //smiaRegs_vendor_sensor_abort_timing_on_sw_stby
   write_cmos_sensor(0x3A70,0x0000);
   if(imgsensor_info.sensor_output_dataformat == SENSOR_OUTPUT_FORMAT_RAW_RWB_Wr){
        write_cmos_sensor(0x3A72,0xFB98);
        write_cmos_sensor(0x3A74,0x0000);
        write_cmos_sensor(0x3A76,0xFB98);
        write_cmos_sensor(0x3A78,0xFD32);
        write_cmos_sensor(0x3A7A,0x0000);
        write_cmos_sensor(0x3A7C,0xFB32);
        write_cmos_sensor(0x3A7E,0x0000);
        write_cmos_sensor(0x3A80,0x0000);
        write_cmos_sensor(0x3A82,0xFB98);
        write_cmos_sensor(0x3A84,0x0000);
        write_cmos_sensor(0x3A86,0xFB98);
        write_cmos_sensor(0x3A88,0xFD32);
        write_cmos_sensor(0x3A8A,0x0000);
        write_cmos_sensor(0x3A8C,0xFD32);
   } else {
        write_cmos_sensor(0x3A72,0x0000);
        write_cmos_sensor(0x3A74,0x0000);
        write_cmos_sensor(0x3A76,0x0000);
        write_cmos_sensor(0x3A78,0x0000);
        write_cmos_sensor(0x3A7A,0x0000);
        write_cmos_sensor(0x3A7C,0x0000);
        write_cmos_sensor(0x3A7E,0x0000);
        write_cmos_sensor(0x3A80,0x0000);
        write_cmos_sensor(0x3A82,0x0000);
        write_cmos_sensor(0x3A84,0x0000);
        write_cmos_sensor(0x3A86,0x0000);
        write_cmos_sensor(0x3A88,0x0000);
        write_cmos_sensor(0x3A8A,0x0000);
        write_cmos_sensor(0x3A8C,0x0000);
   }
   write_cmos_sensor(0x3A8E,0x0000);
   write_cmos_sensor(0x3AB2,0x0000);
   write_cmos_sensor(0x3AB4,0x0000);
   write_cmos_sensor(0x3AB6,0x0000);
   write_cmos_sensor(0x3AB8,0x0000);
   write_cmos_sensor(0x3ABA,0x0000);
   write_cmos_sensor(0x3ABC,0x0000);
   write_cmos_sensor(0x3ABE,0x0000);
   write_cmos_sensor(0x3AC0,0x0000);
   write_cmos_sensor(0x3AC2,0x0000);
   write_cmos_sensor(0x3AC4,0x0000);
   write_cmos_sensor(0x3AC6,0x0000);
   write_cmos_sensor(0x3AC8,0x0000);
   write_cmos_sensor(0x3ACA,0x0000);
   write_cmos_sensor(0x3ACC,0x0000);
   write_cmos_sensor(0x3ACE,0x0000);
#endif
   write_cmos_sensor_8(0x0114,0x03);

}	/*	sensor_init  */


static void preview_setting(void)
{
	int retry=0;
	int stream = 0;


	//====================================================
	//2P8XX
	//Full resolution
	//x_output_size: 5336
	//y_output_size: 3000
	//frame_rate: 30.003
	//output_format: RAW10
	//output_interface: MIPI
	//output_lanes: 4
	//mipi_clock_mhz: 888.000
	//vt_pixel_clock:560.000
	//input_clock_mhz: 24.00
	//====================================================
	//$MV1[mclk:24,width:2668,height:1500,format:MIPI_RAW10,mipi_lane:4,mipi_hssettle:19,pvi_pclk_inverse:0]


	 write_cmos_sensor(0x0100,0x0000);
#if  MULTI_WRITE
S5K2P8MIPI_table_write_cmos_sensor(addr_data_pair_preview, sizeof(addr_data_pair_preview)/sizeof(kal_uint16));
#else
#ifdef FIX_VIEW_ANGLE
	write_cmos_sensor(0x0344,0x0018);   /*smiaRegs_rw_frame_timing_x_addr_start*/
    write_cmos_sensor(0x0346,0x0014);   /*smiaRegs_rw_frame_timing_y_addr_start*/
    write_cmos_sensor(0x0348,0x14D7);   /*smiaRegs_rw_frame_timing_x_addr_end*/
    write_cmos_sensor(0x034A,0x0BB3);   /*smiaRegs_rw_frame_timing_y_addr_end*/
    write_cmos_sensor(0x034C,0x0A60);   /*smiaRegs_rw_frame_timing_x_output_size*/
    write_cmos_sensor(0x034E,0x05D0);   /*smiaRegs_rw_frame_timing_y_output_size*/
#else
    write_cmos_sensor(0x0344,0x0010);   /*smiaRegs_rw_frame_timing_x_addr_start*/
    write_cmos_sensor(0x0346,0x0008);   /*smiaRegs_rw_frame_timing_y_addr_start*/
    write_cmos_sensor(0x0348,0x14DF);   /*smiaRegs_rw_frame_timing_x_addr_end*/
    write_cmos_sensor(0x034A,0x0BBF);   /*smiaRegs_rw_frame_timing_y_addr_end*/
    write_cmos_sensor(0x034C,0x0A68);   /*smiaRegs_rw_frame_timing_x_output_size*/
    write_cmos_sensor(0x034E,0x05DC);   /*smiaRegs_rw_frame_timing_y_output_size*/
#endif
    write_cmos_sensor(0x0382,0x0003);   /*smiaRegs_rw_sub_sample_x_odd_inc*/
    write_cmos_sensor(0x0380,0x0001);   /*smiaRegs_rw_sub_sample_x_even_inc*/
    write_cmos_sensor(0x0386,0x0003);   /*smiaRegs_rw_sub_sample_y_odd_inc*/
    write_cmos_sensor(0x0384,0x0001);   /*smiaRegs_rw_sub_sample_y_even_inc*/
    write_cmos_sensor_8(0x0900,0x01);   /*smiaRegs_rw_binning_mode*/
    write_cmos_sensor_8(0x0901,0x22);   /*smiaRegs_rw_binning_type*/
    write_cmos_sensor(0x0400,0x0000);   /*smiaRegs_rw_scaling_scaling_mode*/
    write_cmos_sensor(0x0404,0x0010);   /*smiaRegs_rw_scaling_scale_m*/
	write_cmos_sensor_8(0x0114,0x03);
    write_cmos_sensor_8(0x0111,0x02);   /*smiaRegs_rw_output_signalling_mode*/
    write_cmos_sensor(0x0136,0x1800);   /*smiaRegs_rw_op_cond_extclk_frequency_mhz*/
    write_cmos_sensor(0x0304,0x0006);   /*smiaRegs_rw_clocks_pre_pll_clk_div*/
    write_cmos_sensor(0x0306,0x0074);   /*smiaRegs_rw_clocks_pll_multiplier // 175*/
    write_cmos_sensor(0x0300,0x0005);   /*smiaRegs_rw_clocks_vt_pix_clk_div*/
    write_cmos_sensor(0x0302,0x0001);   /*smiaRegs_rw_clocks_vt_sys_clk_div*/
    write_cmos_sensor(0x030C,0x0004);   /*smiaRegs_rw_clocks_secnd_pre_pll_clk_div*/
    write_cmos_sensor(0x030E,0x004A);   /*smiaRegs_rw_clocks_secnd_pll_multiplier  50*/
    write_cmos_sensor(0x030A,0x0001);   /*smiaRegs_rw_clocks_op_sys_clk_div*/
    write_cmos_sensor(0x0308,0x0008);   /*smiaRegs_rw_clocks_op_pix_clk_div*/
	write_cmos_sensor(0x1118,0x4100);
	write_cmos_sensor(0x1124,0x4100);
	write_cmos_sensor(0x112C,0x4100);
	write_cmos_sensor(0x1164,0x4100);
	write_cmos_sensor(0x1170,0x4100);
	write_cmos_sensor(0x301C,0x4100);
	write_cmos_sensor(0x0342,0x16F8);   /*smiaRegs_rw_frame_timing_line_length_pck // 3216*/
    write_cmos_sensor(0x0340,0x0804);   /*smiaRegs_rw_frame_timing_frame_length_lines // 580*/
    write_cmos_sensor(0x0200,0x0100);   /*smiaRegs_rw_integration_time_fine_integration_time*/
    write_cmos_sensor(0x0202,0x0100);   /*smiaRegs_rw_integration_time_coarse_integration_time*/
    write_cmos_sensor_8(0x0216,0x00);   /*smiaRegs_rw_wdr_multiple_exp_mode*/
    write_cmos_sensor_8(0x3054,0x00);   /*smiaRegs_vendor_sensor_enable_af_pixels*/
	write_cmos_sensor(0x306A,0x8110);
	write_cmos_sensor_8(0x3A6B,0x00);
    write_cmos_sensor_8(0x39BB,0x02);
    write_cmos_sensor(0x3A58,0x0060);
    write_cmos_sensor_8(0x39E3,0x02);
    write_cmos_sensor(0x3238,0x0519);   /* SenAnalog_AIG_pDefaultNormalPtrs_3__1_              */
    write_cmos_sensor(0x324A,0x00DE);   /* SenAnalog_AIG_pDefaultNormalPtrs_6__1_              */
    write_cmos_sensor(0x3250,0x011F);    /*SenAnalog_AIG_pDefaultNormalPtrs_7__1_              */
    write_cmos_sensor(0x3274,0x013E);    /*SenAnalog_AIG_pDefaultNormalPtrs_13__1_             */
    write_cmos_sensor(0x32C2,0x011F);    /*SenAnalog_AIG_pDefaultNormalPtrs_26__1_             */
    write_cmos_sensor(0x32C8,0x0140);   /* SenAnalog_AIG_pDefaultNormalPtrs_27__1_             */
    write_cmos_sensor(0x32DA,0x011F);   /* SenAnalog_AIG_pDefaultNormalPtrs_30__1_             */
    write_cmos_sensor(0x32E0,0x0120);  /*  SenAnalog_AIG_pDefaultNormalPtrs_31__1_             */
    write_cmos_sensor(0x35FE,0x0078);  /*  SenAnalog_AIG_pDefaultNormalPtrs_164__1_            */
    write_cmos_sensor(0x37C6,0x0083);   /* SenAnalog_AIG_pDefaultVdaAndShPtrs_30__1_           */
    write_cmos_sensor(0x37CC,0x005D);    /*SenAnalog_AIG_pDefaultVdaAndShPtrs_31__1_           */
    write_cmos_sensor(0x37D2,0x0057);   /* SenAnalog_AIG_pDefaultVdaAndShPtrs_32__1_           */
    write_cmos_sensor(0x37DE,0x0081);   /* SenAnalog_AIG_pDefaultVdaAndShPtrs_34__1_           */
    write_cmos_sensor(0x37E4,0x005F);   /* SenAnalog_AIG_pDefaultVdaAndShPtrs_35__1_           */
    write_cmos_sensor(0x37EA,0x0055);   /* SenAnalog_AIG_pDefaultVdaAndShPtrs_36__1_           */
    write_cmos_sensor(0x37F6,0x0081);   /* SenAnalog_AIG_pDefaultVdaAndShPtrs_38__1_           */
    write_cmos_sensor(0x37FC,0x005F);   /* SenAnalog_AIG_pDefaultVdaAndShPtrs_39__1_           */
    write_cmos_sensor(0x3802,0x0055);    /*SenAnalog_AIG_pDefaultVdaAndShPtrs_40__1_   */
	write_cmos_sensor(0x6028,0x2000);
	write_cmos_sensor(0x602A,0x15b8);
	write_cmos_sensor(0x6F12,0x8011);
	write_cmos_sensor(0x3140,0x0FE2);
	write_cmos_sensor_8(0x31B5,0x00);
	write_cmos_sensor(0x6028,0x2000);
	write_cmos_sensor(0x602A,0x1760);
	write_cmos_sensor_8(0x6F12,0x01);
	write_cmos_sensor(0x6F12,0x0048);
	write_cmos_sensor(0x6F12,0x0050);
	write_cmos_sensor(0x6F12,0x0060);
	write_cmos_sensor(0x6F12,0x0060);
	write_cmos_sensor(0x6F12,0xF42E);
	write_cmos_sensor(0x6F12,0x006D);
	write_cmos_sensor(0x6F12,0x006A);
	write_cmos_sensor(0x6F12,0xF51E);
#endif
	if(imgsensor.ihdr_en)
	{
		write_cmos_sensor_8(0x0216,0x02);
		write_cmos_sensor_8(0x0217,0x00);
		write_cmos_sensor_8(0x021B,0x00);
	}
	else
	{
		write_cmos_sensor_8(0x0216,0x00);
		write_cmos_sensor_8(0x0217,0x00);
		write_cmos_sensor_8(0x021B,0x00);
	}
	LOG_INF("before stream 0x0100 = %x\n",read_cmos_sensor(0x0100));
	write_cmos_sensor(0x0100,0x0100);	//smiaRegs_rw_general_setup // Stream on
    /*EVB seldom output fail issue. Need retry*/
	while(retry<21)
	{
 	 stream = read_cmos_sensor_8(0x0005);
	if(stream==0xff || stream ==-1)
		{
		  msleep(5);
		  retry++;
		  LOG_INF("Sensor has not output stream %x\n",read_cmos_sensor(0x0100));
		}
	 else
		{
			retry=0;
			LOGE("Sensor has output %x\n",read_cmos_sensor_8(0x0005));
			break;
		}
	}

//	write_cmos_sensor(0x3A70,0x0000);
//	write_cmos_sensor(0x3A72,0x0000);
//	write_cmos_sensor(0x3A74,0x0000);
//	write_cmos_sensor(0x3A76,0x0000);
//	write_cmos_sensor(0x3A78,0x0000);
//	write_cmos_sensor(0x3A7A,0x0000);
//	write_cmos_sensor(0x3A7C,0x0000);
//	write_cmos_sensor(0x3A7E,0x0000);
//	write_cmos_sensor(0x3A80,0x0000);
//	write_cmos_sensor(0x3A82,0x0000);
//	write_cmos_sensor(0x3A84,0x0000);
//	write_cmos_sensor(0x3A86,0x0000);
//	write_cmos_sensor(0x3A88,0x0000);
//	write_cmos_sensor(0x3A8A,0x0000);
//	write_cmos_sensor(0x3A8C,0x0000);
//	write_cmos_sensor(0x3A8E,0x0000);
//	write_cmos_sensor(0x3AB2,0x0000);
//	write_cmos_sensor(0x3AB4,0x0000);
//	write_cmos_sensor(0x3AB6,0x0000);
//	write_cmos_sensor(0x3AB8,0x0000);
//	write_cmos_sensor(0x3ABA,0x0000);
//	write_cmos_sensor(0x3ABC,0x0000);
//	write_cmos_sensor(0x3ABE,0x0000);
//	write_cmos_sensor(0x3AC0,0x0000);
//	write_cmos_sensor(0x3AC2,0x0000);
//	write_cmos_sensor(0x3AC4,0x0000);
//	write_cmos_sensor(0x3AC6,0x0000);
//	write_cmos_sensor(0x3AC8,0x0000);
//	write_cmos_sensor(0x3ACA,0x0000);
//	write_cmos_sensor(0x3ACC,0x0000);
//	write_cmos_sensor(0x3ACE,0x0000);
}	/*	preview_setting  */


static void normal_capture_setting(void)
{
	int stream = 0;
	int retry=0;
	LOG_INF("E! ");
	//====================================================
//2P8XX
//Full resolution
//x_output_size: 5336
//y_output_size: 3000
//frame_rate: 30.003
//output_format: RAW10
//output_interface: MIPI
//output_lanes: 4
//mipi_clock_mhz: 1392.000
//vt_pixel_clock:560.000
//input_clock_mhz: 24.00
//====================================================
//$MV1[mclk:24,width:5336,height:3000,format:MIPI_RAW10,mipi_lane:4,mipi_hssettle:19,pvi_pclk_inverse:0]
	write_cmos_sensor(0x0100,0x0000);
#if MULTI_WRITE
	S5K2P8MIPI_table_write_cmos_sensor(addr_data_pair_capture, sizeof(addr_data_pair_capture)/sizeof(kal_uint16));
#else
#ifdef FIX_VIEW_ANGLE
	write_cmos_sensor(0x0344,0x0018);	//smiaRegs_rw_frame_timing_x_addr_start
	write_cmos_sensor(0x0346,0x0014);	//smiaRegs_rw_frame_timing_y_addr_start
	write_cmos_sensor(0x0348,0x14D7);	//smiaRegs_rw_frame_timing_x_addr_end
	write_cmos_sensor(0x034A,0x0BB3);	//smiaRegs_rw_frame_timing_y_addr_end
	write_cmos_sensor(0x034C,0x14C0);	//smiaRegs_rw_frame_timing_x_output_size
	write_cmos_sensor(0x034E,0x0BA0);	//smiaRegs_rw_frame_timing_y_output_size
#else
	write_cmos_sensor(0x0344,0x0010);	//smiaRegs_rw_frame_timing_x_addr_start
	write_cmos_sensor(0x0346,0x0008);	//smiaRegs_rw_frame_timing_y_addr_start
	write_cmos_sensor(0x0348,0x14DF);	//smiaRegs_rw_frame_timing_x_addr_end
	write_cmos_sensor(0x034A,0x0BBF);	//smiaRegs_rw_frame_timing_y_addr_end
	write_cmos_sensor(0x034C,0x14D0);	//smiaRegs_rw_frame_timing_x_output_size
	write_cmos_sensor(0x034E,0x0BB8);	//smiaRegs_rw_frame_timing_y_output_size
#endif
	write_cmos_sensor(0x0382,0x0001);	//smiaRegs_rw_sub_sample_x_odd_inc
	write_cmos_sensor(0x0380,0x0001);	//smiaRegs_rw_sub_sample_x_even_inc
	write_cmos_sensor(0x0386,0x0001);	//smiaRegs_rw_sub_sample_y_odd_inc
	write_cmos_sensor(0x0384,0x0001);	//smiaRegs_rw_sub_sample_y_even_inc
	write_cmos_sensor_8(0x0900,0x00);	//smiaRegs_rw_binning_mode
	write_cmos_sensor_8(0x0901,0x11);	//smiaRegs_rw_binning_type
	write_cmos_sensor(0x0400,0x0000);	//smiaRegs_rw_scaling_scaling_mode
	write_cmos_sensor(0x0404,0x0010);	//smiaRegs_rw_scaling_scale_m
	write_cmos_sensor_8(0x0114,0x03);
	write_cmos_sensor_8(0x0111,0x02);	//smiaRegs_rw_output_signalling_mode
	write_cmos_sensor(0x0136,0x1800);	//smiaRegs_rw_op_cond_extclk_frequency_mhz
	write_cmos_sensor(0x0304,0x0006);	//smiaRegs_rw_clocks_pre_pll_clk_div
	write_cmos_sensor(0x0306,0x00AF);	//smiaRegs_rw_clocks_pll_multiplier // 175
	write_cmos_sensor(0x0300,0x0005);	//smiaRegs_rw_clocks_vt_pix_clk_div
	write_cmos_sensor(0x0302,0x0001);	//smiaRegs_rw_clocks_vt_sys_clk_div
	write_cmos_sensor(0x030C,0x0004);	//smiaRegs_rw_clocks_secnd_pre_pll_clk_div
	write_cmos_sensor(0x030E,0x0074);	//smiaRegs_rw_clocks_secnd_pll_multiplier // 50
	write_cmos_sensor(0x030A,0x0001);	//smiaRegs_rw_clocks_op_sys_clk_div
	write_cmos_sensor(0x0308,0x0008);	//smiaRegs_rw_clocks_op_pix_clk_div
	write_cmos_sensor(0x1118,0x43FA);
	write_cmos_sensor(0x1124,0x43FA);
	write_cmos_sensor(0x112C,0x42C0);
	write_cmos_sensor(0x1164,0x4280);
	write_cmos_sensor(0x1170,0x4100);
	write_cmos_sensor(0x301C,0x4396);
	write_cmos_sensor(0x0342,0x16F8);	//smiaRegs_rw_frame_timing_line_length_pck // 3216
	write_cmos_sensor(0x0340,0x0C66);	//smiaRegs_rw_frame_timing_frame_length_lines // 580
	write_cmos_sensor(0x0200,0x0100);	//smiaRegs_rw_integration_time_fine_integration_time
	write_cmos_sensor(0x0202,0x0100);	//smiaRegs_rw_integration_time_coarse_integration_time
	write_cmos_sensor_8(0x0216,0x00);	//smiaRegs_rw_wdr_multiple_exp_mode
	write_cmos_sensor_8(0x3054,0x01);	//smiaRegs_vendor_sensor_enable_af_pixels
	write_cmos_sensor(0x306A,0x8000);
	write_cmos_sensor_8(0x3A6B,0x00);
	write_cmos_sensor_8(0x39BB,0x02);
	write_cmos_sensor(0x3A58,0x0061);
	write_cmos_sensor_8(0x39E3,0x02);
	write_cmos_sensor(0x3238,0x0219);   // SenAnalog_AIG_pDefaultNormalPtrs_3__1_
	write_cmos_sensor(0x324A,0x00DE);   // SenAnalog_AIG_pDefaultNormalPtrs_6__1_
	write_cmos_sensor(0x3250,0x011F);    //SenAnalog_AIG_pDefaultNormalPtrs_7__1_
	write_cmos_sensor(0x3274,0x013E);    //SenAnalog_AIG_pDefaultNormalPtrs_13__1_
	write_cmos_sensor(0x32C2,0x011F);    //SenAnalog_AIG_pDefaultNormalPtrs_26__1_
	write_cmos_sensor(0x32C8,0x0140);   // SenAnalog_AIG_pDefaultNormalPtrs_27__1_
	write_cmos_sensor(0x32DA,0x011F);   // SenAnalog_AIG_pDefaultNormalPtrs_30__1_
	write_cmos_sensor(0x32E0,0x0120);  //  SenAnalog_AIG_pDefaultNormalPtrs_31__1_
	write_cmos_sensor(0x35FE,0x0078);  //  SenAnalog_AIG_pDefaultNormalPtrs_164__1_
	write_cmos_sensor(0x37C6,0x0083);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_30__1_
	write_cmos_sensor(0x37CC,0x005D);    //SenAnalog_AIG_pDefaultVdaAndShPtrs_31__1_
	write_cmos_sensor(0x37D2,0x0057);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_32__1_
	write_cmos_sensor(0x37DE,0x0081);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_34__1_
	write_cmos_sensor(0x37E4,0x005F);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_35__1_
	write_cmos_sensor(0x37EA,0x0055);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_36__1_
	write_cmos_sensor(0x37F6,0x0081);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_38__1_
	write_cmos_sensor(0x37FC,0x005F);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_39__1_
	write_cmos_sensor(0x3802,0x0055);    //SenAnalog_AIG_pDefaultVdaAndShPtrs_40__1_
	write_cmos_sensor(0x6028,0x2000);
	write_cmos_sensor(0x602A,0x15b8);
	write_cmos_sensor(0x6F12,0x8011);
	write_cmos_sensor(0x3140,0x0FE2);
	write_cmos_sensor_8(0x31B5,0x00);
	write_cmos_sensor(0x6028,0x2000);
	write_cmos_sensor(0x602A,0x1760);
	write_cmos_sensor_8(0x6F12,0x01);
	write_cmos_sensor(0x6F12,0x0048);
	write_cmos_sensor(0x6F12,0x0050);
	write_cmos_sensor(0x6F12,0x0060);
	write_cmos_sensor(0x6F12,0x0060);
	write_cmos_sensor(0x6F12,0xF42E);
	write_cmos_sensor(0x6F12,0x006D);
	write_cmos_sensor(0x6F12,0x006A);
	write_cmos_sensor(0x6F12,0xF51E);
#endif
	write_cmos_sensor(0x0100,0x0100);	//smiaRegs_rw_general_setup // Stream on
	while(retry<21)
		{
		 stream = read_cmos_sensor_8(0x0005);
		if(stream==0xff || stream ==-1)
			{
			  msleep(5);
			  retry++;
			  LOG_INF("Sensor has not output stream %x\n",read_cmos_sensor(0x0100));
			}
		 else
			{
				retry=0;
				LOGE("Sensor has output %x\n",read_cmos_sensor_8(0x0005));
				break;
			}
		}


//		write_cmos_sensor(0x3A70,0x0000);
//		write_cmos_sensor(0x3A72,0x0000);
//		write_cmos_sensor(0x3A74,0x0000);
//		write_cmos_sensor(0x3A76,0x0000);
//		write_cmos_sensor(0x3A78,0x0000);
//		write_cmos_sensor(0x3A7A,0x0000);
//		write_cmos_sensor(0x3A7C,0x0000);
//		write_cmos_sensor(0x3A7E,0x0000);
//		write_cmos_sensor(0x3A80,0x0000);
//		write_cmos_sensor(0x3A82,0x0000);
//		write_cmos_sensor(0x3A84,0x0000);
//		write_cmos_sensor(0x3A86,0x0000);
//		write_cmos_sensor(0x3A88,0x0000);
//		write_cmos_sensor(0x3A8A,0x0000);
//		write_cmos_sensor(0x3A8C,0x0000);
//		write_cmos_sensor(0x3A8E,0x0000);
//		write_cmos_sensor(0x3AB2,0x0000);
//		write_cmos_sensor(0x3AB4,0x0000);
//		write_cmos_sensor(0x3AB6,0x0000);
//		write_cmos_sensor(0x3AB8,0x0000);
//		write_cmos_sensor(0x3ABA,0x0000);
//		write_cmos_sensor(0x3ABC,0x0000);
//		write_cmos_sensor(0x3ABE,0x0000);
//		write_cmos_sensor(0x3AC0,0x0000);
//		write_cmos_sensor(0x3AC2,0x0000);
//		write_cmos_sensor(0x3AC4,0x0000);
//		write_cmos_sensor(0x3AC6,0x0000);
//		write_cmos_sensor(0x3AC8,0x0000);
//		write_cmos_sensor(0x3ACA,0x0000);
//		write_cmos_sensor(0x3ACC,0x0000);
//		write_cmos_sensor(0x3ACE,0x0000);
		if(imgsensor.ihdr_en)
			{
				write_cmos_sensor(0x6028,0x4000);
				write_cmos_sensor(0x602A,0x0B05);
				write_cmos_sensor_8(0x6F12,0x00);
				write_cmos_sensor(0x6028,0x4000);
				write_cmos_sensor(0x602A,0x0216);
				write_cmos_sensor(0x6F12,0x0200);
				//write_cmos_sensor(0x602A,0x021e);
				//write_cmos_sensor(0x6f12,0x0800);
				//write_cmos_sensor(0x602A,0x0202);
				//write_cmos_sensor(0x6f12,0x0200);
				//write_cmos_sensor_8(0x0216,0x02);
				//write_cmos_sensor_8(0x0217,0x01);
				//write_cmos_sensor_8(0x0218,0x01);
				//write_cmos_sensor_8(0x021a,0x01);
				write_cmos_sensor_8(0x021B,0x00);

			}
			else
			{
				write_cmos_sensor(0x6028,0x4000);
				write_cmos_sensor(0x602A,0x0B05);
				write_cmos_sensor_8(0x6F12,0x01);
				write_cmos_sensor_8(0x0216,0x00);
				write_cmos_sensor_8(0x0217,0x00);
				write_cmos_sensor_8(0x021B,0x00);

			}

    LOG_INF( "Exit!");

}

static void pip_capture_setting(void)
{
	int stream = 0;
	int retry=0;
    LOG_INF( "S5K2P8 PIP setting Enter!");

    write_cmos_sensor(0x0100,0x0000);
#if MULTI_WRITE
	S5K2P8MIPI_table_write_cmos_sensor(addr_data_pair_capture_pip, sizeof(addr_data_pair_capture_pip)/sizeof(kal_uint16));
#else
#ifdef FIX_VIEW_ANGLE
	write_cmos_sensor(0x0344,0x0018);	//smiaRegs_rw_frame_timing_x_addr_start
	write_cmos_sensor(0x0346,0x0014);	//smiaRegs_rw_frame_timing_y_addr_start
	write_cmos_sensor(0x0348,0x14D7);	//smiaRegs_rw_frame_timing_x_addr_end
	write_cmos_sensor(0x034A,0x0BB3);	//smiaRegs_rw_frame_timing_y_addr_end
	write_cmos_sensor(0x034C,0x14C0);	//smiaRegs_rw_frame_timing_x_output_size
	write_cmos_sensor(0x034E,0x0BA0);	//smiaRegs_rw_frame_timing_y_output_size
#else
    write_cmos_sensor(0x0344,0x0008);   /* smiaRegs_rw_frame_timing_x_addr_start */
    write_cmos_sensor(0x0346,0x0008);   /*smiaRegs_rw_frame_timing_y_addr_start */
    write_cmos_sensor(0x0348,0x14DF);   /*smiaRegs_rw_frame_timing_x_addr_end*/
    write_cmos_sensor(0x034A,0x0BBF);   /*smiaRegs_rw_frame_timing_y_addr_end*/
	write_cmos_sensor(0x034C,0x14D0);	//smiaRegs_rw_frame_timing_x_output_size
    write_cmos_sensor(0x034E,0x0BB8);   /*smiaRegs_rw_frame_timing_y_output_size*/
#endif
    write_cmos_sensor(0x0382,0x0001);   /*smiaRegs_rw_sub_sample_x_odd_inc*/
    write_cmos_sensor(0x0380,0x0001);   /*smiaRegs_rw_sub_sample_x_even_inc*/
    write_cmos_sensor(0x0386,0x0001);   /*smiaRegs_rw_sub_sample_y_odd_inc*/
    write_cmos_sensor(0x0384,0x0001);   /*smiaRegs_rw_sub_sample_y_even_inc*/
    write_cmos_sensor_8(0x0900,0x00);   /*smiaRegs_rw_binning_mode*/
    write_cmos_sensor_8(0x0901,0x11);   /*smiaRegs_rw_binning_type*/
    write_cmos_sensor(0x0400,0x0000);   /*smiaRegs_rw_scaling_scaling_mode*/
    write_cmos_sensor(0x0404,0x0010);   /*smiaRegs_rw_scaling_scale_m*/
	 write_cmos_sensor_8(0x0114,0x03);
    write_cmos_sensor_8(0x0111,0x02);   /*smiaRegs_rw_output_signalling_mode*/
    write_cmos_sensor(0x0136,0x1800);
    write_cmos_sensor(0x0304,0x0006);
	write_cmos_sensor(0x0306,0x0074);	//smiaRegs_rw_clocks_pll_multiplier // 175
	write_cmos_sensor(0x0300,0x0005);	//smiaRegs_rw_clocks_vt_pix_clk_div
	write_cmos_sensor(0x0302,0x0001);	//smiaRegs_rw_clocks_vt_sys_clk_div
	write_cmos_sensor(0x030C,0x0004);	//smiaRegs_rw_clocks_secnd_pre_pll_clk_div
	write_cmos_sensor(0x030E,0x004B);	//smiaRegs_rw_clocks_secnd_pll_multiplier // 50
	write_cmos_sensor(0x030A,0x0001);	//smiaRegs_rw_clocks_op_sys_clk_div
	write_cmos_sensor(0x0308,0x0008);	//smiaRegs_rw_clocks_op_pix_clk_div
	write_cmos_sensor(0x1118,0x4100);
	write_cmos_sensor(0x1124,0x4100);
	write_cmos_sensor(0x112C,0x4100);
	write_cmos_sensor(0x1164,0x4100);
	write_cmos_sensor(0x1170,0x4100);
	write_cmos_sensor(0x301C,0x4100);
	write_cmos_sensor(0x0342,0x16f8);	//smiaRegs_rw_frame_timing_line_length_pck // 3216
	write_cmos_sensor(0x0340,0x0C66);	//smiaRegs_rw_frame_timing_frame_length_lines // 580
	write_cmos_sensor(0x0200,0x0100);	//smiaRegs_rw_integration_time_fine_integration_time
	write_cmos_sensor(0x0202,0x0100);	//smiaRegs_rw_integration_time_coarse_integration_time
	write_cmos_sensor_8(0x0216,0x00);	//smiaRegs_rw_wdr_multiple_exp_mode
	write_cmos_sensor_8(0x3054,0x01);	//smiaRegs_vendor_sensor_enable_af_pixels
	write_cmos_sensor(0x3A6A,0x8000);
	write_cmos_sensor_8(0x3A6B,0x00);
	write_cmos_sensor_8(0x39BB,0x02);
	write_cmos_sensor(0x3A58,0x0061);
	write_cmos_sensor_8(0x39E3,0x02);
	write_cmos_sensor(0x3238,0x0529);   // SenAnalog_AIG_pDefaultNormalPtrs_3__1_
	write_cmos_sensor(0x324A,0x00DE);   // SenAnalog_AIG_pDefaultNormalPtrs_6__1_
	write_cmos_sensor(0x3250,0x011F);    //SenAnalog_AIG_pDefaultNormalPtrs_7__1_
	write_cmos_sensor(0x3274,0x013E);    //SenAnalog_AIG_pDefaultNormalPtrs_13__1_
	write_cmos_sensor(0x32C2,0x011F);    //SenAnalog_AIG_pDefaultNormalPtrs_26__1_
	write_cmos_sensor(0x32C8,0x0140);   // SenAnalog_AIG_pDefaultNormalPtrs_27__1_
	write_cmos_sensor(0x32DA,0x011F);   // SenAnalog_AIG_pDefaultNormalPtrs_30__1_
	write_cmos_sensor(0x32E0,0x0120);  //  SenAnalog_AIG_pDefaultNormalPtrs_31__1_
	write_cmos_sensor(0x35FE,0x0078);  //  SenAnalog_AIG_pDefaultNormalPtrs_164__1_
	write_cmos_sensor(0x37C6,0x0083);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_30__1_
	write_cmos_sensor(0x37CC,0x005D);    //SenAnalog_AIG_pDefaultVdaAndShPtrs_31__1_
	write_cmos_sensor(0x37D2,0x0057);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_32__1_
	write_cmos_sensor(0x37DE,0x0081);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_34__1_
	write_cmos_sensor(0x37E4,0x005F);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_35__1_
	write_cmos_sensor(0x37EA,0x0055);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_36__1_
	write_cmos_sensor(0x37F6,0x0081);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_38__1_
	write_cmos_sensor(0x37FC,0x005F);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_39__1_
	write_cmos_sensor(0x3802,0x0055);    //SenAnalog_AIG_pDefaultVdaAndShPtrs_40__1_
	write_cmos_sensor(0x6028,0x2000);
	write_cmos_sensor(0x602A,0x15b8);
	write_cmos_sensor(0x6F12,0x8011);
	write_cmos_sensor(0x3140,0x0FE2);
	write_cmos_sensor_8(0x31B5,0X00);
	write_cmos_sensor(0x6028,0x2000);
	write_cmos_sensor(0x602A,0x1760);
	write_cmos_sensor_8(0x6F12,0x01);
	write_cmos_sensor(0x6F12,0x0048);
	write_cmos_sensor(0x6F12,0x0050);
	write_cmos_sensor(0x6F12,0x0060);
	write_cmos_sensor(0x6F12,0x0060);
	write_cmos_sensor(0x6F12,0xF42E);
	write_cmos_sensor(0x6F12,0x006D);
	write_cmos_sensor(0x6F12,0x006A);
	write_cmos_sensor(0x6F12,0xF51E);
#endif
	write_cmos_sensor(0x0100,0x0100);	//smiaRegs_rw_general_setup // Stream on
	while(retry<21)
		{
		 stream = read_cmos_sensor_8(0x0005);
		if(stream==0xff || stream ==-1)
			{
			  msleep(5);
			  retry++;
			  LOG_INF("Sensor has not output stream %x\n",read_cmos_sensor(0x0100));
			}
		 else
			{
				retry=0;
				LOGE("Sensor has output %x\n",read_cmos_sensor_8(0x0005));
				break;
			}
		}


//	write_cmos_sensor(0x3A70,0x0000);
//	write_cmos_sensor(0x3A72,0x0000);
//	write_cmos_sensor(0x3A74,0x0000);
//	write_cmos_sensor(0x3A76,0x0000);
//	write_cmos_sensor(0x3A78,0x0000);
//	write_cmos_sensor(0x3A7A,0x0000);
//	write_cmos_sensor(0x3A7C,0x0000);
//	write_cmos_sensor(0x3A7E,0x0000);
//	write_cmos_sensor(0x3A80,0x0000);
//	write_cmos_sensor(0x3A82,0x0000);
//	write_cmos_sensor(0x3A84,0x0000);
//	write_cmos_sensor(0x3A86,0x0000);
//	write_cmos_sensor(0x3A88,0x0000);
//	write_cmos_sensor(0x3A8A,0x0000);
//	write_cmos_sensor(0x3A8C,0x0000);
//	write_cmos_sensor(0x3A8E,0x0000);
//	write_cmos_sensor(0x3AB2,0x0000);
//	write_cmos_sensor(0x3AB4,0x0000);
//	write_cmos_sensor(0x3AB6,0x0000);
//	write_cmos_sensor(0x3AB8,0x0000);
//	write_cmos_sensor(0x3ABA,0x0000);
//	write_cmos_sensor(0x3ABC,0x0000);
//	write_cmos_sensor(0x3ABE,0x0000);
//	write_cmos_sensor(0x3AC0,0x0000);
//	write_cmos_sensor(0x3AC2,0x0000);
//	write_cmos_sensor(0x3AC4,0x0000);
//	write_cmos_sensor(0x3AC6,0x0000);
//	write_cmos_sensor(0x3AC8,0x0000);
//	write_cmos_sensor(0x3ACA,0x0000);
//	write_cmos_sensor(0x3ACC,0x0000);
//	write_cmos_sensor(0x3ACE,0x0000);

}

static void pip_capture_15fps_setting(void)
{
	int stream = 0;
    int retry=0;
    LOG_INF( "S5K2P8 PIP 15FPS setting Enter!");


    write_cmos_sensor(0x0100,0x0000);
#if MULTI_WRITE
	S5K2P8MIPI_table_write_cmos_sensor(addr_data_pair_capture_pip_15, sizeof(addr_data_pair_capture_pip_15)/sizeof(kal_uint16));
#else
#ifdef FIX_VIEW_ANGLE
    write_cmos_sensor(0x0344,0x0014);   /* smiaRegs_rw_frame_timing_x_addr_start */
    write_cmos_sensor(0x0346,0x0014);   /*smiaRegs_rw_frame_timing_y_addr_start */
    write_cmos_sensor(0x0348,0x14D3);   /*smiaRegs_rw_frame_timing_x_addr_end*/
    write_cmos_sensor(0x034A,0x0BB3);   /*smiaRegs_rw_frame_timing_y_addr_end*/
    write_cmos_sensor(0x034C,0x14C0);   //smiaRegs_rw_frame_timing_x_output_size
    write_cmos_sensor(0x034E,0x0BA0);   /*smiaRegs_rw_frame_timing_y_output_size*/
#else
    write_cmos_sensor(0x0344,0x0010);   /* smiaRegs_rw_frame_timing_x_addr_start */
    write_cmos_sensor(0x0346,0x0008);   /*smiaRegs_rw_frame_timing_y_addr_start */
    write_cmos_sensor(0x0348,0x14DF);   /*smiaRegs_rw_frame_timing_x_addr_end*/
    write_cmos_sensor(0x034A,0x0BBF);   /*smiaRegs_rw_frame_timing_y_addr_end*/
    write_cmos_sensor(0x034C,0x14D0);   //smiaRegs_rw_frame_timing_x_output_size
    write_cmos_sensor(0x034E,0x0BB8);   /*smiaRegs_rw_frame_timing_y_output_size*/
#endif
    write_cmos_sensor(0x0382,0x0001);   /*smiaRegs_rw_sub_sample_x_odd_inc*/
    write_cmos_sensor(0x0380,0x0001);   /*smiaRegs_rw_sub_sample_x_even_inc*/
    write_cmos_sensor(0x0386,0x0001);   /*smiaRegs_rw_sub_sample_y_odd_inc*/
    write_cmos_sensor(0x0384,0x0001);   /*smiaRegs_rw_sub_sample_y_even_inc*/
    write_cmos_sensor_8(0x0900,0x00);   /*smiaRegs_rw_binning_mode*/
    write_cmos_sensor_8(0x0901,0x11);   /*smiaRegs_rw_binning_type*/
    write_cmos_sensor(0x0400,0x0000);   /*smiaRegs_rw_scaling_scaling_mode*/
    write_cmos_sensor(0x0404,0x0010);   /*smiaRegs_rw_scaling_scale_m*/
    write_cmos_sensor_8(0x0114,0x03);
    write_cmos_sensor_8(0x0111,0x02);   /*smiaRegs_rw_output_signalling_mode*/
    write_cmos_sensor(0x0136,0x1800);
    write_cmos_sensor(0x0304,0x0006);
    write_cmos_sensor(0x0306,0x00AF);   //smiaRegs_rw_clocks_pll_multiplier // 175
    write_cmos_sensor(0x0302,0x0001);   //smiaRegs_rw_clocks_vt_pix_clk_div
    write_cmos_sensor(0x0300,0x0005);   //smiaRegs_rw_clocks_vt_sys_clk_div
    write_cmos_sensor(0x030C,0x0004);   //smiaRegs_rw_clocks_secnd_pre_pll_clk_div
    write_cmos_sensor(0x030E,0x003A);   //smiaRegs_rw_clocks_secnd_pll_multiplier // 50
    write_cmos_sensor(0x030A,0x0001);   //smiaRegs_rw_clocks_op_sys_clk_div
    write_cmos_sensor(0x0308,0x0008);   //smiaRegs_rw_clocks_op_pix_clk_div
    write_cmos_sensor(0x1118,0x4100);
    write_cmos_sensor(0x1124,0x4100);
    write_cmos_sensor(0x112C,0x4100);
    write_cmos_sensor(0x1164,0x4100);
    write_cmos_sensor(0x1170,0x4100);
    write_cmos_sensor(0x301C,0x4100);
    write_cmos_sensor(0x0342,0x2FC0);   //smiaRegs_rw_frame_timing_line_length_pck // 3216
    write_cmos_sensor(0x0340,0x0C66);   //smiaRegs_rw_frame_timing_frame_length_lines // 580
    write_cmos_sensor(0x0200,0x0100);   //smiaRegs_rw_integration_time_fine_integration_time
    write_cmos_sensor(0x0202,0x0100);   //smiaRegs_rw_integration_time_coarse_integration_time
    write_cmos_sensor_8(0x0216,0x00);   //smiaRegs_rw_wdr_multiple_exp_mode
    write_cmos_sensor_8(0x3054,0x01);   //smiaRegs_vendor_sensor_enable_af_pixels
    write_cmos_sensor(0x3A6A,0x8000);
    write_cmos_sensor_8(0x3A6B,0x00);
    write_cmos_sensor_8(0x39BB,0x02);
    write_cmos_sensor_8(0x3005,0x05);
    write_cmos_sensor(0x3A58,0x0061);
    write_cmos_sensor_8(0x39E3,0x02);
    write_cmos_sensor(0x3238,0x0529);   // SenAnalog_AIG_pDefaultNormalPtrs_3__1_
    write_cmos_sensor(0x324A,0x00DE);   // SenAnalog_AIG_pDefaultNormalPtrs_6__1_
    write_cmos_sensor(0x3250,0x011F);    //SenAnalog_AIG_pDefaultNormalPtrs_7__1_
    write_cmos_sensor(0x3274,0x013E);    //SenAnalog_AIG_pDefaultNormalPtrs_13__1_
    write_cmos_sensor(0x32C2,0x011F);    //SenAnalog_AIG_pDefaultNormalPtrs_26__1_
    write_cmos_sensor(0x32C8,0x0140);   // SenAnalog_AIG_pDefaultNormalPtrs_27__1_
    write_cmos_sensor(0x32DA,0x011F);   // SenAnalog_AIG_pDefaultNormalPtrs_30__1_
    write_cmos_sensor(0x32E0,0x0120);  //  SenAnalog_AIG_pDefaultNormalPtrs_31__1_
    write_cmos_sensor(0x35FE,0x0078);  //  SenAnalog_AIG_pDefaultNormalPtrs_164__1_
    write_cmos_sensor(0x37C6,0x0083);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_30__1_
    write_cmos_sensor(0x37CC,0x005D);    //SenAnalog_AIG_pDefaultVdaAndShPtrs_31__1_
    write_cmos_sensor(0x37D2,0x0057);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_32__1_
    write_cmos_sensor(0x37DE,0x0081);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_34__1_
    write_cmos_sensor(0x37E4,0x005F);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_35__1_
    write_cmos_sensor(0x37EA,0x0055);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_36__1_
    write_cmos_sensor(0x37F6,0x0081);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_38__1_
    write_cmos_sensor(0x37FC,0x005F);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_39__1_
    write_cmos_sensor(0x3802,0x0055);    //SenAnalog_AIG_pDefaultVdaAndShPtrs_40__1_
    write_cmos_sensor(0x6028,0x2000);
    write_cmos_sensor(0x602A,0x15b8);
    write_cmos_sensor(0x6F12,0x8011);
    write_cmos_sensor(0x3140,0x0FE2);
    write_cmos_sensor_8(0x31B5,0x00);
    write_cmos_sensor(0x6028,0x2000);
    write_cmos_sensor(0x602A,0x1760);
    write_cmos_sensor_8(0x6F12,0x01);
    write_cmos_sensor(0x6F12,0x0048);
    write_cmos_sensor(0x6F12,0x0050);
    write_cmos_sensor(0x6F12,0x0060);
    write_cmos_sensor(0x6F12,0x0060);
    write_cmos_sensor(0x6F12,0xF42E);
    write_cmos_sensor(0x6F12,0x006D);
    write_cmos_sensor(0x6F12,0x006A);
    write_cmos_sensor(0x6F12,0xF51E);
#endif
    write_cmos_sensor(0x0100,0x0100);   //smiaRegs_rw_general_setup // Stream on
	while(retry<21)
		{
		 stream = read_cmos_sensor_8(0x0005);
		if(stream==0xff || stream ==-1)
			{
			  msleep(5);
			  retry++;
			  LOG_INF("Sensor has not output stream %x\n",read_cmos_sensor(0x0100));
			}
		 else
			{
				retry=0;
				LOGE("Sensor has output %x\n",read_cmos_sensor_8(0x0005));
				break;
			}
		}


}

static void capture_setting(kal_uint16 currefps)
{
	//capture_20fps();
	if(currefps==300)
		normal_capture_setting();
	else if(currefps==200) // PIP
		pip_capture_setting();
	else if(currefps==150)
        pip_capture_15fps_setting();
    else// default
		pip_capture_setting();
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);

	if (currefps == 150)
		pip_capture_15fps_setting();
	else if (currefps == 200)
		pip_capture_setting();
	else
		normal_capture_setting();

}
static void hs_video_setting(void)
{
int stream = 0;
int retry=0;
	LOG_INF("E");
#ifdef SLOW_MOTION_120FPS
	LOG_INF("slow motion fps:120fps");
	write_cmos_sensor(0x0100,0x0000);
	write_cmos_sensor(0x0344,0x0014);	//smiaRegs_rw_frame_timing_x_addr_start
	write_cmos_sensor(0x0346,0x000C);	//smiaRegs_rw_frame_timing_y_addr_start
	write_cmos_sensor(0x0348,0x14D3);	//smiaRegs_rw_frame_timing_x_addr_end
	write_cmos_sensor(0x034A,0x0BBB);	//smiaRegs_rw_frame_timing_y_addr_end
	write_cmos_sensor(0x034C,0x0530);	//smiaRegs_rw_frame_timing_x_output_size
	write_cmos_sensor(0x034E,0x02EC);	//smiaRegs_rw_frame_timing_y_output_size
	write_cmos_sensor(0x0382,0x0001);	//smiaRegs_rw_sub_sample_x_odd_inc
	write_cmos_sensor(0x0380,0x0001);	//smiaRegs_rw_sub_sample_x_even_inc
	write_cmos_sensor(0x0386,0x0007);	//smiaRegs_rw_sub_sample_y_odd_inc
	write_cmos_sensor(0x0384,0x0001);	//smiaRegs_rw_sub_sample_y_even_inc
	write_cmos_sensor_8(0x0900,0x01);	//smiaRegs_rw_binning_mode
	write_cmos_sensor_8(0x0901,0x14);	//smiaRegs_rw_binning_type
	write_cmos_sensor(0x0400,0x0001);	//smiaRegs_rw_scaling_scaling_mode
	write_cmos_sensor(0x0404,0x0040);	//smiaRegs_rw_scaling_scale_m
	write_cmos_sensor_8(0x0114,0x03);	//smiaRegs_rw_output_signalling_mode
	write_cmos_sensor_8(0x0111,0x02);	//smiaRegs_rw_output_signalling_mode
	write_cmos_sensor(0x0136,0x1800);	//smiaRegs_rw_op_cond_extclk_frequency_mhz
	write_cmos_sensor(0x0304,0x0006);	//smiaRegs_rw_clocks_pre_pll_clk_div
	write_cmos_sensor(0x0306,0x00AF);	//smiaRegs_rw_clocks_pll_multiplier // 175
	write_cmos_sensor(0x0300,0x0005);	//smiaRegs_rw_clocks_vt_pix_clk_div
	write_cmos_sensor(0x0302,0x0001);	//smiaRegs_rw_clocks_vt_sys_clk_div
	write_cmos_sensor(0x030C,0x0004);	//smiaRegs_rw_clocks_secnd_pre_pll_clk_div
	write_cmos_sensor(0x030E,0x0032);	//smiaRegs_rw_clocks_secnd_pll_multiplier // 50
	write_cmos_sensor(0x030A,0x0001);	//smiaRegs_rw_clocks_op_sys_clk_div
	write_cmos_sensor(0x0308,0x0008);	//smiaRegs_rw_clocks_op_pix_clk_div
	write_cmos_sensor(0x1118,0x43FA);
	write_cmos_sensor(0x1124,0x43FA);
	write_cmos_sensor(0x112C,0x42C0);
	write_cmos_sensor(0x1164,0x4280);
	write_cmos_sensor(0x1170,0x4100);
	write_cmos_sensor(0x301C,0x4396);
	write_cmos_sensor(0x0342,0x16F8);	//smiaRegs_rw_frame_timing_line_length_pck // 3216
	write_cmos_sensor(0x0340,0x0319);	//smiaRegs_rw_frame_timing_frame_length_lines // 580
	write_cmos_sensor(0x0200,0x0100);	//smiaRegs_rw_integration_time_fine_integration_time
	write_cmos_sensor(0x0202,0x0100);	//smiaRegs_rw_integration_time_coarse_integration_time
	write_cmos_sensor_8(0x0216,0x00);	//smiaRegs_rw_wdr_multiple_exp_mode
	write_cmos_sensor_8(0x3054,0x00);	//smiaRegs_vendor_sensor_enable_af_pixels
	write_cmos_sensor(0x306A,0x8220);
	write_cmos_sensor_8(0x3A6B,0x00);
	write_cmos_sensor_8(0x39BB,0x02);
	write_cmos_sensor_8(0x3005,0x05);
	write_cmos_sensor(0x3A58,0x0060);
	write_cmos_sensor_8(0x39E3,0x02);
	write_cmos_sensor(0x3238,0x0219);   // SenAnalog_AIG_pDefaultNormalPtrs_3__1_
	write_cmos_sensor(0x324A,0x00E2);   // SenAnalog_AIG_pDefaultNormalPtrs_6__1_
	write_cmos_sensor(0x3250,0x0114);    //SenAnalog_AIG_pDefaultNormalPtrs_7__1_
	write_cmos_sensor(0x3274,0x013B);    //SenAnalog_AIG_pDefaultNormalPtrs_13__1_
	write_cmos_sensor(0x32C2,0x0114);    //SenAnalog_AIG_pDefaultNormalPtrs_26__1_
	write_cmos_sensor(0x32C8,0x012D);   // SenAnalog_AIG_pDefaultNormalPtrs_27__1_
	write_cmos_sensor(0x32DA,0x0114);   // SenAnalog_AIG_pDefaultNormalPtrs_30__1_
	write_cmos_sensor(0x32E0,0x0115);  //  SenAnalog_AIG_pDefaultNormalPtrs_31__1_
	write_cmos_sensor(0x35FE,0x007A);  //  SenAnalog_AIG_pDefaultNormalPtrs_164__1_
	write_cmos_sensor(0x37C6,0x0073);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_30__1_
	write_cmos_sensor(0x37CC,0x0054);    //SenAnalog_AIG_pDefaultVdaAndShPtrs_31__1_
	write_cmos_sensor(0x37D2,0x0048);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_32__1_
	write_cmos_sensor(0x37DE,0x0071);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_34__1_
	write_cmos_sensor(0x37E4,0x0056);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_35__1_
	write_cmos_sensor(0x37EA,0x0046);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_36__1_
	write_cmos_sensor(0x37F6,0x0071);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_38__1_
	write_cmos_sensor(0x37FC,0x0056);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_39__1_
	write_cmos_sensor(0x3802,0x0046);    //SenAnalog_AIG_pDefaultVdaAndShPtrs_40__1_
	write_cmos_sensor(0x6028,0x2000);
	write_cmos_sensor(0x602A,0x15b8);
	write_cmos_sensor(0x6F12,0x8111);
	write_cmos_sensor(0x3140,0x0F21);
	write_cmos_sensor_8(0x31B5,0x01);
	write_cmos_sensor(0x6028,0x2000);
	write_cmos_sensor(0x602A,0x1760);
	write_cmos_sensor_8(0x6F12,0x01);
	write_cmos_sensor(0x6F12,0x0048);
	write_cmos_sensor(0x6F12,0x0050);
	write_cmos_sensor(0x6F12,0x0060);
	write_cmos_sensor(0x6F12,0x0060);
	write_cmos_sensor(0x6F12,0xF42E);
	write_cmos_sensor(0x6F12,0x006D);
	write_cmos_sensor(0x6F12,0x006A);
	write_cmos_sensor(0x6F12,0xF51E);
	write_cmos_sensor(0x0100,0x0100);	//smiaRegs_rw_general_setup // Stream on
	msleep(100);
	write_cmos_sensor(0x3A70,0x0000);
	write_cmos_sensor(0x3A72,0x0000);
	write_cmos_sensor(0x3A74,0x0000);
	write_cmos_sensor(0x3A76,0x0000);
	write_cmos_sensor(0x3A78,0x0000);
	write_cmos_sensor(0x3A7A,0x0000);
	write_cmos_sensor(0x3A7C,0x0000);
	write_cmos_sensor(0x3A7E,0x0000);
	write_cmos_sensor(0x3A80,0x0000);
	write_cmos_sensor(0x3A82,0x0000);
	write_cmos_sensor(0x3A84,0x0000);
	write_cmos_sensor(0x3A86,0x0000);
	write_cmos_sensor(0x3A88,0x0000);
	write_cmos_sensor(0x3A8A,0x0000);
	write_cmos_sensor(0x3A8C,0x0000);
	write_cmos_sensor(0x3A8E,0x0000);
	write_cmos_sensor(0x3AB2,0x0000);
	write_cmos_sensor(0x3AB4,0x0000);
	write_cmos_sensor(0x3AB6,0x0000);
	write_cmos_sensor(0x3AB8,0x0000);
	write_cmos_sensor(0x3ABA,0x0000);
	write_cmos_sensor(0x3ABC,0x0000);
	write_cmos_sensor(0x3ABE,0x0000);
	write_cmos_sensor(0x3AC0,0x0000);
	write_cmos_sensor(0x3AC2,0x0000);
	write_cmos_sensor(0x3AC4,0x0000);
	write_cmos_sensor(0x3AC6,0x0000);
	write_cmos_sensor(0x3AC8,0x0000);
	write_cmos_sensor(0x3ACA,0x0000);
	write_cmos_sensor(0x3ACC,0x0000);
	write_cmos_sensor(0x3ACE,0x0000);
#else
	write_cmos_sensor(0x0100,0x0000);
#ifdef FIX_VIEW_ANGLE
	write_cmos_sensor(0x0344,0x0018);   /*smiaRegs_rw_frame_timing_x_addr_start*/
    write_cmos_sensor(0x0346,0x0014);   /*smiaRegs_rw_frame_timing_y_addr_start*/
    write_cmos_sensor(0x0348,0x14D7);   /*smiaRegs_rw_frame_timing_x_addr_end*/
    write_cmos_sensor(0x034A,0x0BB3);   /*smiaRegs_rw_frame_timing_y_addr_end*/
    write_cmos_sensor(0x034C,0x0A60);   /*smiaRegs_rw_frame_timing_x_output_size*/
    write_cmos_sensor(0x034E,0x05D0);   /*smiaRegs_rw_frame_timing_y_output_size*/
#else
    write_cmos_sensor(0x0344,0x0010);   /*smiaRegs_rw_frame_timing_x_addr_start*/
	write_cmos_sensor(0x0346,0x0008);	//smiaRegs_rw_frame_timing_y_addr_start
	write_cmos_sensor(0x0348,0x14DF);	//smiaRegs_rw_frame_timing_x_addr_end
	write_cmos_sensor(0x034A,0x0BBF);	//smiaRegs_rw_frame_timing_y_addr_end
    write_cmos_sensor(0x034C,0x0A68);   /*smiaRegs_rw_frame_timing_x_output_size*/
	write_cmos_sensor(0x034E,0x05DC);	//smiaRegs_rw_frame_timing_y_output_size
#endif
	write_cmos_sensor(0x0382,0x0003);	//smiaRegs_rw_sub_sample_x_odd_inc
	write_cmos_sensor(0x0380,0x0001);	//smiaRegs_rw_sub_sample_x_even_inc
	write_cmos_sensor(0x0386,0x0003);	//smiaRegs_rw_sub_sample_y_odd_inc
	write_cmos_sensor(0x0384,0x0001);	//smiaRegs_rw_sub_sample_y_even_inc
	write_cmos_sensor_8(0x0900,0x01);	//smiaRegs_rw_binning_mode
	write_cmos_sensor_8(0x0901,0x22);	//smiaRegs_rw_binning_type
	write_cmos_sensor(0x0400,0x0000);	//smiaRegs_rw_scaling_scaling_mode
	write_cmos_sensor(0x0404,0x0010);	//smiaRegs_rw_scaling_scale_m
	write_cmos_sensor_8(0x0114,0x03);
	write_cmos_sensor_8(0x0111,0x02);	//smiaRegs_rw_output_signalling_mode
	write_cmos_sensor(0x0136,0x1800);	//smiaRegs_rw_op_cond_extclk_frequency_mhz
	write_cmos_sensor(0x0304,0x0006);	//smiaRegs_rw_clocks_pre_pll_clk_div
	write_cmos_sensor(0x0306,0x00AF);	//smiaRegs_rw_clocks_pll_multiplier // 175
	write_cmos_sensor(0x0300,0x0005);	//smiaRegs_rw_clocks_vt_pix_clk_div
	write_cmos_sensor(0x0302,0x0001);	//smiaRegs_rw_clocks_vt_sys_clk_div
	write_cmos_sensor(0x030C,0x0004);	//smiaRegs_rw_clocks_secnd_pre_pll_clk_div
	write_cmos_sensor(0x030E,0x004A);	//smiaRegs_rw_clocks_secnd_pll_multiplier // 50
	write_cmos_sensor(0x030A,0x0001);	//smiaRegs_rw_clocks_op_sys_clk_div
	write_cmos_sensor(0x0308,0x0008);	//smiaRegs_rw_clocks_op_pix_clk_div
	write_cmos_sensor(0x1118,0x43FA);
	write_cmos_sensor(0x1124,0x43FA);
	write_cmos_sensor(0x112C,0x42C0);
	write_cmos_sensor(0x1164,0x4280);
	write_cmos_sensor(0x1170,0x4100);
	write_cmos_sensor(0x301C,0x4396);
	write_cmos_sensor(0x0342,0x16F8);	//smiaRegs_rw_frame_timing_line_length_pck // 3216
	write_cmos_sensor(0x0340,0x0633);	//smiaRegs_rw_frame_timing_frame_length_lines // 580
	write_cmos_sensor(0x0200,0x0100);	//smiaRegs_rw_integration_time_fine_integration_time
	write_cmos_sensor(0x0202,0x0100);	//smiaRegs_rw_integration_time_coarse_integration_time
	write_cmos_sensor_8(0x0216,0x00);	//smiaRegs_rw_wdr_multiple_exp_mode
	write_cmos_sensor_8(0x3054,0x00);	//smiaRegs_vendor_sensor_enable_af_pixels
	write_cmos_sensor(0x306A,0x8110);
	write_cmos_sensor_8(0x3A6B,0x00);
	write_cmos_sensor_8(0x39BB,0x02);
	write_cmos_sensor(0x3A58,0x0060);
	write_cmos_sensor_8(0x39E3,0x02);
	write_cmos_sensor(0x3238,0x0219);   // SenAnalog_AIG_pDefaultNormalPtrs_3__1_
	write_cmos_sensor(0x324A,0x00E2);   // SenAnalog_AIG_pDefaultNormalPtrs_6__1_
	write_cmos_sensor(0x3250,0x0114);    //SenAnalog_AIG_pDefaultNormalPtrs_7__1_
	write_cmos_sensor(0x3274,0x013B);    //SenAnalog_AIG_pDefaultNormalPtrs_13__1_
	write_cmos_sensor(0x32C2,0x0114);    //SenAnalog_AIG_pDefaultNormalPtrs_26__1_
	write_cmos_sensor(0x32C8,0x012D);   // SenAnalog_AIG_pDefaultNormalPtrs_27__1_
	write_cmos_sensor(0x32DA,0x0114);   // SenAnalog_AIG_pDefaultNormalPtrs_30__1_
	write_cmos_sensor(0x32E0,0x0115);  //  SenAnalog_AIG_pDefaultNormalPtrs_31__1_
	write_cmos_sensor(0x35FE,0x007A);  //  SenAnalog_AIG_pDefaultNormalPtrs_164__1_
	write_cmos_sensor(0x37C6,0x0073);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_30__1_
	write_cmos_sensor(0x37CC,0x0054);    //SenAnalog_AIG_pDefaultVdaAndShPtrs_31__1_
	write_cmos_sensor(0x37D2,0x0048);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_32__1_
	write_cmos_sensor(0x37DE,0x0071);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_34__1_
	write_cmos_sensor(0x37E4,0x0056);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_35__1_
	write_cmos_sensor(0x37EA,0x0046);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_36__1_
	write_cmos_sensor(0x37F6,0x0071);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_38__1_
	write_cmos_sensor(0x37FC,0x0056);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_39__1_
	write_cmos_sensor(0x3802,0x0046);    //SenAnalog_AIG_pDefaultVdaAndShPtrs_40__1_
	write_cmos_sensor(0x6028,0x2000);
	write_cmos_sensor(0x602A,0x15b8);
	write_cmos_sensor(0x6F12,0x8011);
	write_cmos_sensor(0x3140,0x0FE2);
	write_cmos_sensor_8(0x31B5,0x00);
	write_cmos_sensor(0x6028,0x2000);
	write_cmos_sensor(0x602A,0x1760);
	write_cmos_sensor_8(0x6F12,0x01);
	write_cmos_sensor(0x6F12,0x0048);
	write_cmos_sensor(0x6F12,0x0050);
	write_cmos_sensor(0x6F12,0x0060);
	write_cmos_sensor(0x6F12,0x0060);
	write_cmos_sensor(0x6F12,0xF42E);
	write_cmos_sensor(0x6F12,0x006D);
	write_cmos_sensor(0x6F12,0x006A);
	write_cmos_sensor(0x6F12,0xF51E);
	write_cmos_sensor(0x0100,0x0100);	//smiaRegs_rw_general_setup // Stream on
//	mdelay(100);
//	write_cmos_sensor(0x3A70,0x0000);
//	write_cmos_sensor(0x3A72,0x0000);
//	write_cmos_sensor(0x3A74,0x0000);
//	write_cmos_sensor(0x3A76,0x0000);
//	write_cmos_sensor(0x3A78,0x0000);
//	write_cmos_sensor(0x3A7A,0x0000);
//	write_cmos_sensor(0x3A7C,0x0000);
//	write_cmos_sensor(0x3A7E,0x0000);
//	write_cmos_sensor(0x3A80,0x0000);
//	write_cmos_sensor(0x3A82,0x0000);
//	write_cmos_sensor(0x3A84,0x0000);
//	write_cmos_sensor(0x3A86,0x0000);
//	write_cmos_sensor(0x3A88,0x0000);
//	write_cmos_sensor(0x3A8A,0x0000);
//	write_cmos_sensor(0x3A8C,0x0000);
//	write_cmos_sensor(0x3A8E,0x0000);
//	write_cmos_sensor(0x3AB2,0x0000);
//	write_cmos_sensor(0x3AB4,0x0000);
//	write_cmos_sensor(0x3AB6,0x0000);
//	write_cmos_sensor(0x3AB8,0x0000);
//	write_cmos_sensor(0x3ABA,0x0000);
//	write_cmos_sensor(0x3ABC,0x0000);
//	write_cmos_sensor(0x3ABE,0x0000);
//	write_cmos_sensor(0x3AC0,0x0000);
//	write_cmos_sensor(0x3AC2,0x0000);
//	write_cmos_sensor(0x3AC4,0x0000);
//	write_cmos_sensor(0x3AC6,0x0000);
//	write_cmos_sensor(0x3AC8,0x0000);
//	write_cmos_sensor(0x3ACA,0x0000);
//	write_cmos_sensor(0x3ACC,0x0000);
//	write_cmos_sensor(0x3ACE,0x0000);
#endif
	while(retry<21)
		{
	 	 stream = read_cmos_sensor_8(0x0005);
		if(stream==0xff || stream ==-1)
			{
			  msleep(5);
			  retry++;
			  LOG_INF("Sensor has not output stream %x\n",read_cmos_sensor(0x0100));
			}
		 else
			{
				retry=0;
				LOGE("Sensor has output %x\n",read_cmos_sensor_8(0x0005));
				break;
			}
		}

}

static void slim_video_setting(void)
{
	LOG_INF("E");
	//$MV1[mclk:24,width:1328,height:748,format:MIPI_RAW10,mipi_lane:4,mipi_hssettle:19,pvi_pclk_inverse:0]
	write_cmos_sensor(0x0100,0x0000);
	write_cmos_sensor(0x0344,0x0014);	//smiaRegs_rw_frame_timing_x_addr_start
	write_cmos_sensor(0x0346,0x000C);	//smiaRegs_rw_frame_timing_y_addr_start
	write_cmos_sensor(0x0348,0x14D3);	//smiaRegs_rw_frame_timing_x_addr_end
	write_cmos_sensor(0x034A,0x0BBB);	//smiaRegs_rw_frame_timing_y_addr_end
	write_cmos_sensor(0x034C,0x0530);	//smiaRegs_rw_frame_timing_x_output_size
	write_cmos_sensor(0x034E,0x02EC);	//smiaRegs_rw_frame_timing_y_output_size
	write_cmos_sensor(0x0382,0x0001);	//smiaRegs_rw_sub_sample_x_odd_inc
	write_cmos_sensor(0x0380,0x0001);	//smiaRegs_rw_sub_sample_x_even_inc
	write_cmos_sensor(0x0386,0x0007);	//smiaRegs_rw_sub_sample_y_odd_inc
	write_cmos_sensor(0x0384,0x0001);	//smiaRegs_rw_sub_sample_y_even_inc
	write_cmos_sensor_8(0x0900,0x01);	//smiaRegs_rw_binning_mode
	write_cmos_sensor_8(0x0901,0x14);	//smiaRegs_rw_binning_type
	write_cmos_sensor(0x0400,0x0001);	//smiaRegs_rw_scaling_scaling_mode
	write_cmos_sensor(0x0404,0x0040);	//smiaRegs_rw_scaling_scale_m
	write_cmos_sensor_8(0x0114,0x03);	//smiaRegs_rw_output_signalling_mode
	write_cmos_sensor_8(0x0111,0x02);	//smiaRegs_rw_output_signalling_mode
	write_cmos_sensor(0x0136,0x1800);	//smiaRegs_rw_op_cond_extclk_frequency_mhz
	write_cmos_sensor(0x0304,0x0006);	//smiaRegs_rw_clocks_pre_pll_clk_div
	write_cmos_sensor(0x0306,0x00AF);	//smiaRegs_rw_clocks_pll_multiplier // 175
	write_cmos_sensor(0x0300,0x0005);	//smiaRegs_rw_clocks_vt_pix_clk_div
	write_cmos_sensor(0x0302,0x0001);	//smiaRegs_rw_clocks_vt_sys_clk_div
	write_cmos_sensor(0x030C,0x0004);	//smiaRegs_rw_clocks_secnd_pre_pll_clk_div
	write_cmos_sensor(0x030E,0x0032);	//smiaRegs_rw_clocks_secnd_pll_multiplier // 50
	write_cmos_sensor(0x030A,0x0001);	//smiaRegs_rw_clocks_op_sys_clk_div
	write_cmos_sensor(0x0308,0x0008);	//smiaRegs_rw_clocks_op_pix_clk_div
	write_cmos_sensor(0x1118,0x43FA);
	write_cmos_sensor(0x1124,0x43FA);
	write_cmos_sensor(0x112C,0x42C0);
	write_cmos_sensor(0x1164,0x4280);
	write_cmos_sensor(0x1170,0x4100);
	write_cmos_sensor(0x301C,0x4396);
	write_cmos_sensor(0x0342,0x16F8);	//smiaRegs_rw_frame_timing_line_length_pck // 3216
	write_cmos_sensor(0x0340,0x0C64);	//smiaRegs_rw_frame_timing_frame_length_lines // 580
	write_cmos_sensor(0x0200,0x0100);	//smiaRegs_rw_integration_time_fine_integration_time
	write_cmos_sensor(0x0202,0x0100);	//smiaRegs_rw_integration_time_coarse_integration_time
	write_cmos_sensor_8(0x0216,0x00);	//smiaRegs_rw_wdr_multiple_exp_mode
	write_cmos_sensor_8(0x3054,0x00);	//smiaRegs_vendor_sensor_enable_af_pixels
	write_cmos_sensor(0x306A,0x8220);
	write_cmos_sensor_8(0x3A6B,0x00);
	write_cmos_sensor_8(0x39BB,0x02);
	write_cmos_sensor_8(0x3005,0x05);
	write_cmos_sensor(0x3A58,0x0060);
	write_cmos_sensor_8(0x39E3,0x02);
	write_cmos_sensor(0x3238,0x0219);   // SenAnalog_AIG_pDefaultNormalPtrs_3__1_
	write_cmos_sensor(0x324A,0x00E2);   // SenAnalog_AIG_pDefaultNormalPtrs_6__1_
	write_cmos_sensor(0x3250,0x0114);    //SenAnalog_AIG_pDefaultNormalPtrs_7__1_
	write_cmos_sensor(0x3274,0x013B);    //SenAnalog_AIG_pDefaultNormalPtrs_13__1_
	write_cmos_sensor(0x32C2,0x0114);    //SenAnalog_AIG_pDefaultNormalPtrs_26__1_
	write_cmos_sensor(0x32C8,0x012D);   // SenAnalog_AIG_pDefaultNormalPtrs_27__1_
	write_cmos_sensor(0x32DA,0x0114);   // SenAnalog_AIG_pDefaultNormalPtrs_30__1_
	write_cmos_sensor(0x32E0,0x0115);  //  SenAnalog_AIG_pDefaultNormalPtrs_31__1_
	write_cmos_sensor(0x35FE,0x007A);  //  SenAnalog_AIG_pDefaultNormalPtrs_164__1_
	write_cmos_sensor(0x37C6,0x0073);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_30__1_
	write_cmos_sensor(0x37CC,0x0054);    //SenAnalog_AIG_pDefaultVdaAndShPtrs_31__1_
	write_cmos_sensor(0x37D2,0x0048);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_32__1_
	write_cmos_sensor(0x37DE,0x0071);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_34__1_
	write_cmos_sensor(0x37E4,0x0056);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_35__1_
	write_cmos_sensor(0x37EA,0x0046);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_36__1_
	write_cmos_sensor(0x37F6,0x0071);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_38__1_
	write_cmos_sensor(0x37FC,0x0056);   // SenAnalog_AIG_pDefaultVdaAndShPtrs_39__1_
	write_cmos_sensor(0x3802,0x0046);    //SenAnalog_AIG_pDefaultVdaAndShPtrs_40__1_
	write_cmos_sensor(0x6028,0x2000);
	write_cmos_sensor(0x602A,0x15b8);
	write_cmos_sensor(0x6F12,0x8111);
	write_cmos_sensor(0x3140,0x0F21);
	write_cmos_sensor_8(0x31B5,0x01);
	write_cmos_sensor(0x6028,0x2000);
	write_cmos_sensor(0x602A,0x1760);
	write_cmos_sensor_8(0x6F12,0x01);
	write_cmos_sensor(0x6F12,0x0048);
	write_cmos_sensor(0x6F12,0x0050);
	write_cmos_sensor(0x6F12,0x0060);
	write_cmos_sensor(0x6F12,0x0060);
	write_cmos_sensor(0x6F12,0xF42E);
	write_cmos_sensor(0x6F12,0x006D);
	write_cmos_sensor(0x6F12,0x006A);
	write_cmos_sensor(0x6F12,0xF51E);

	write_cmos_sensor(0x0100,0x0100);	//smiaRegs_rw_general_setup // Stream on
//	mdelay(100);
//	write_cmos_sensor(0x3A70,0x0000);
//	write_cmos_sensor(0x3A72,0x0000);
//	write_cmos_sensor(0x3A74,0x0000);
//	write_cmos_sensor(0x3A76,0x0000);
//	write_cmos_sensor(0x3A78,0x0000);
//	write_cmos_sensor(0x3A7A,0x0000);
//	write_cmos_sensor(0x3A7C,0x0000);
//	write_cmos_sensor(0x3A7E,0x0000);
//	write_cmos_sensor(0x3A80,0x0000);
//	write_cmos_sensor(0x3A82,0x0000);
//	write_cmos_sensor(0x3A84,0x0000);
//	write_cmos_sensor(0x3A86,0x0000);
//	write_cmos_sensor(0x3A88,0x0000);
//	write_cmos_sensor(0x3A8A,0x0000);
//	write_cmos_sensor(0x3A8C,0x0000);
//	write_cmos_sensor(0x3A8E,0x0000);
//	write_cmos_sensor(0x3AB2,0x0000);
//	write_cmos_sensor(0x3AB4,0x0000);
//	write_cmos_sensor(0x3AB6,0x0000);
//	write_cmos_sensor(0x3AB8,0x0000);
//	write_cmos_sensor(0x3ABA,0x0000);
//	write_cmos_sensor(0x3ABC,0x0000);
//	write_cmos_sensor(0x3ABE,0x0000);
//	write_cmos_sensor(0x3AC0,0x0000);
//	write_cmos_sensor(0x3AC2,0x0000);
//	write_cmos_sensor(0x3AC4,0x0000);
//	write_cmos_sensor(0x3AC6,0x0000);
//	write_cmos_sensor(0x3AC8,0x0000);
//	write_cmos_sensor(0x3ACA,0x0000);
//	write_cmos_sensor(0x3ACC,0x0000);
//	write_cmos_sensor(0x3ACE,0x0000);
}



/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	*sensorID : return the sensor ID
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	char data = 0xff;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
            *sensor_id = read_cmos_sensor(0x0000) ;
            /*PDAF module ID = 0x2102*/
			if ((*sensor_id == imgsensor_info.sensor_id) || (*sensor_id == 0x2102)) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
				if(read_eeprom(0x0F73, &data, 1) && data == 0x01) {
					imgsensor_info.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_RWB_Wr;
					LOG_INF("RWB sensor of S5k2p8\n");
				}
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, write id:0x%x ,sensor Id:0x%x\n", imgsensor.i2c_write_id,*sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		retry = 2;
	}
	if ((*sensor_id != imgsensor_info.sensor_id) || (*sensor_id != 0x2102)) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	//const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;
	LOG_1;
	LOG_2;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
            sensor_id = read_cmos_sensor(0x0000);
			if ((sensor_id == imgsensor_info.sensor_id) || (sensor_id == 0x2102)) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, write id:0x%x id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		if ((sensor_id == imgsensor_info.sensor_id) || (sensor_id == 0x2102))
			break;
		retry = 2;
	}
	if ((imgsensor_info.sensor_id != sensor_id) && (0x2102 != sensor_id))
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = KAL_FALSE;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/

	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	/* LOG_INF("E\n"); */

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	set_mirror_flip(IMAGE_NORMAL);
	LOG_INF("imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

    if (imgsensor.current_fps == imgsensor_info.cap.max_framerate) // 30fps
    {
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	/* PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M */
	else if (imgsensor.current_fps == 240
		|| imgsensor.current_fps == imgsensor_info.cap1.max_framerate)
    {
		if (imgsensor.current_fps != imgsensor_info.cap1.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap1.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	else //PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
    {
		if (imgsensor.current_fps != imgsensor_info.cap2.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap1.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap2.pclk;
		imgsensor.line_length = imgsensor_info.cap2.linelength;
		imgsensor.frame_length = imgsensor_info.cap2.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap2.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}

	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
    set_mirror_flip(IMAGE_NORMAL);
	LOG_INF("imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);

	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;

	if (imgsensor.current_fps == imgsensor_info.normal_video2.max_framerate) { /* 15fps */
		imgsensor.pclk = imgsensor_info.normal_video2.pclk;
		imgsensor.line_length = imgsensor_info.normal_video2.linelength;
		imgsensor.frame_length = imgsensor_info.normal_video2.framelength;
		imgsensor.min_frame_length = imgsensor_info.normal_video2.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else if (imgsensor.current_fps == imgsensor_info.normal_video1.max_framerate) { /* 20fps */
		imgsensor.pclk = imgsensor_info.normal_video1.pclk;
		imgsensor.line_length = imgsensor_info.normal_video1.linelength;
		imgsensor.frame_length = imgsensor_info.normal_video1.framelength;
		imgsensor.min_frame_length = imgsensor_info.normal_video1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else { /* 30fps */
		imgsensor.pclk = imgsensor_info.normal_video.pclk;
		imgsensor.line_length = imgsensor_info.normal_video.linelength;
		imgsensor.frame_length = imgsensor_info.normal_video.framelength;
		imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
		/* imgsensor.current_fps = 300; */
		imgsensor.autoflicker_en = KAL_FALSE;
	}

	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	set_mirror_flip(IMAGE_NORMAL);
	LOG_INF("imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(IMAGE_NORMAL);
	LOG_INF("imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
	return ERROR_NONE;
}	/*	hs_video   */


static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

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
	set_mirror_flip(IMAGE_NORMAL);
	LOG_INF("imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
	return ERROR_NONE;
}	/*	slim_video	 */

/*************************************************************************
* FUNCTION
* Custom1
*
* DESCRIPTION
*   This function start the sensor Custom1.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
    imgsensor.pclk = imgsensor_info.custom1.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom1.linelength;
    imgsensor.frame_length = imgsensor_info.custom1.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
	LOG_INF("imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
    return ERROR_NONE;
}   /*  Custom1   */

static kal_uint32 Custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
    imgsensor.pclk = imgsensor_info.custom2.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom2.linelength;
    imgsensor.frame_length = imgsensor_info.custom2.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
	LOG_INF("imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
    return ERROR_NONE;
}   /*  Custom2   */

static kal_uint32 Custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
    imgsensor.pclk = imgsensor_info.custom3.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom3.linelength;
    imgsensor.frame_length = imgsensor_info.custom3.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
	LOG_INF("imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
    return ERROR_NONE;
}   /*  Custom3   */

static kal_uint32 Custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
    imgsensor.pclk = imgsensor_info.custom4.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom4.linelength;
    imgsensor.frame_length = imgsensor_info.custom4.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
	LOG_INF("imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
    return ERROR_NONE;
}   /*  Custom4   */


static kal_uint32 Custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
    imgsensor.pclk = imgsensor_info.custom5.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom5.linelength;
    imgsensor.frame_length = imgsensor_info.custom5.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
	LOG_INF("imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
    return ERROR_NONE;
}   /*  Custom5   */


static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;

    sensor_resolution->SensorCustom1Width  = imgsensor_info.custom1.grabwindow_width;
    sensor_resolution->SensorCustom1Height     = imgsensor_info.custom1.grabwindow_height;

    sensor_resolution->SensorCustom2Width  = imgsensor_info.custom2.grabwindow_width;
    sensor_resolution->SensorCustom2Height     = imgsensor_info.custom2.grabwindow_height;

    sensor_resolution->SensorCustom3Width  = imgsensor_info.custom3.grabwindow_width;
    sensor_resolution->SensorCustom3Height     = imgsensor_info.custom3.grabwindow_height;

    sensor_resolution->SensorCustom4Width  = imgsensor_info.custom4.grabwindow_width;
    sensor_resolution->SensorCustom4Height     = imgsensor_info.custom4.grabwindow_height;

    sensor_resolution->SensorCustom5Width  = imgsensor_info.custom5.grabwindow_width;
    sensor_resolution->SensorCustom5Height     = imgsensor_info.custom5.grabwindow_height;

	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d", scenario_id);


	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
	//imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
    sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
    sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
    sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;
    sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame;
    sensor_info->Custom5DelayFrame = imgsensor_info.custom5_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = 0;
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
            sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            sensor_info->SensorGrabStartX = imgsensor_info.custom5.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom5.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

            break;
 		default:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}

	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d", scenario_id);
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
        case MSDK_SCENARIO_ID_CUSTOM1:
            Custom1(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            Custom2(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            Custom3(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            Custom4(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            Custom5(image_window, sensor_config_data); // Custom1
            break;
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d ", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable)
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / imgsensor_info.pre.linelength / framerate * 10;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;

			if (imgsensor.current_fps == 150) {  /* 15fps */
				frame_length = imgsensor_info.normal_video2.pclk /
					imgsensor_info.normal_video2.linelength / framerate * 10;
				spin_lock(&imgsensor_drv_lock);
				imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video2.framelength)
					? (frame_length - imgsensor_info.normal_video2.framelength) : 0;
				imgsensor.frame_length = imgsensor_info.normal_video2.framelength
					+ imgsensor.dummy_line;
				imgsensor.min_frame_length = imgsensor.frame_length;
				spin_unlock(&imgsensor_drv_lock);
			} else if (imgsensor.current_fps == 200) {  /* 20fps */
				frame_length = imgsensor_info.normal_video1.pclk /
					imgsensor_info.normal_video1.linelength / framerate * 10;
				spin_lock(&imgsensor_drv_lock);
				imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video1.framelength)
					? (frame_length - imgsensor_info.normal_video1.framelength) : 0;
				imgsensor.frame_length = imgsensor_info.normal_video1.framelength
					+ imgsensor.dummy_line;
				imgsensor.min_frame_length = imgsensor.frame_length;
				spin_unlock(&imgsensor_drv_lock);
			} else {
				frame_length = imgsensor_info.normal_video.pclk /
					imgsensor_info.normal_video.linelength / framerate * 10;
				spin_lock(&imgsensor_drv_lock);
				imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength)
					? (frame_length - imgsensor_info.normal_video.framelength) : 0;
				imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
				imgsensor.min_frame_length = imgsensor.frame_length;
				spin_unlock(&imgsensor_drv_lock);
			}
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			if (imgsensor.current_fps == 300){
				frame_length = imgsensor_info.cap.pclk / imgsensor_info.cap.linelength / framerate * 10;
				spin_lock(&imgsensor_drv_lock);
				imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
				imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
				imgsensor.min_frame_length = imgsensor.frame_length;
				spin_unlock(&imgsensor_drv_lock);
			}
			else if (imgsensor.current_fps == 150) {  //317Mhz, PIP capture: 15fps for less than 13M, 20fps for 16M,15fps for 20M
				frame_length = imgsensor_info.cap2.pclk / imgsensor_info.cap2.linelength / framerate * 10;
				spin_lock(&imgsensor_drv_lock);
				imgsensor.dummy_line = (frame_length > imgsensor_info.cap2.framelength) ? (frame_length - imgsensor_info.cap2.framelength) : 0;
				imgsensor.frame_length = imgsensor_info.cap2.framelength + imgsensor.dummy_line;
				imgsensor.min_frame_length = imgsensor.frame_length;
				spin_unlock(&imgsensor_drv_lock);
			}
			else if (imgsensor.current_fps == 200){
				frame_length = imgsensor_info.cap1.pclk / imgsensor_info.cap1.linelength / framerate * 10;
				spin_lock(&imgsensor_drv_lock);
				imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
				imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
				imgsensor.min_frame_length = imgsensor.frame_length;
				spin_unlock(&imgsensor_drv_lock);
			}
			else{
				frame_length = imgsensor_info.cap2.pclk / imgsensor_info.cap2.linelength / framerate * 10;
				spin_lock(&imgsensor_drv_lock);
				imgsensor.dummy_line = (frame_length > imgsensor_info.cap2.framelength) ? (frame_length - imgsensor_info.cap2.framelength) : 0;
				imgsensor.frame_length = imgsensor_info.cap2.framelength + imgsensor.dummy_line;
				imgsensor.min_frame_length = imgsensor.frame_length;
				spin_unlock(&imgsensor_drv_lock);
			}
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / imgsensor_info.hs_video.linelength / framerate * 10;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / imgsensor_info.slim_video.linelength / framerate * 10;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            frame_length = imgsensor_info.custom1.pclk / imgsensor_info.custom1.linelength / framerate * 10;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            frame_length = imgsensor_info.custom2.pclk / imgsensor_info.custom2.linelength / framerate * 10;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ? (frame_length - imgsensor_info.custom2.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            frame_length = imgsensor_info.custom3.pclk / imgsensor_info.custom3.linelength / framerate * 10;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength) ? (frame_length - imgsensor_info.custom3.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            frame_length = imgsensor_info.custom4.pclk / imgsensor_info.custom4.linelength / framerate * 10;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom4.framelength) ? (frame_length - imgsensor_info.custom4.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom4.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            frame_length = imgsensor_info.custom5.pclk / imgsensor_info.custom5.linelength / framerate * 10 ;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom5.framelength) ? (frame_length - imgsensor_info.custom5.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            set_dummy();
            break;
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk  / imgsensor_info.pre.linelength / framerate * 10;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
			break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	//LOG_INF("scenario_id = %d\n", scenario_id);

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
        case MSDK_SCENARIO_ID_CUSTOM1:
            *framerate = imgsensor_info.custom1.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            *framerate = imgsensor_info.custom2.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            *framerate = imgsensor_info.custom3.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            *framerate = imgsensor_info.custom4.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            *framerate = imgsensor_info.custom5.max_framerate;
            break;
        default:
			break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
        write_cmos_sensor(0x0600, 0x0002);
	} else {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
        write_cmos_sensor(0x0600, 0x0000);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
//    unsigned long long *feature_return_para=(unsigned long long *) feature_para;

	SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    SET_PD_BLOCK_INFO_T *PDAFinfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	//LOG_INF("feature_id = %d", feature_id);
	switch (feature_id) {
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			break;
		case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			if((sensor_reg_data->RegData>>8)>0)
			   write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			else
				write_cmos_sensor_8(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
			break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			get_imgsensor_id(feature_return_para_32);
			break;
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
			break;
        case SENSOR_FEATURE_GET_PDAF_DATA: /* For PDAF EEPROM Read Information*/
			//read_eeprom((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
			spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_SET_HDR:
			//LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data_16);
			LOG_INF("Warning! Not Support IHDR Feature");
			spin_lock(&imgsensor_drv_lock);
			//imgsensor.ihdr_en = (BOOL)*feature_data_16;
            imgsensor.ihdr_en = KAL_FALSE;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
            wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

			switch (*feature_data_32) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
			}
			break;
        case SENSOR_FEATURE_GET_PDAF_INFO:
			LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n", (UINT32)*feature_data);
			PDAFinfo= (SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));

			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(SET_PD_BLOCK_INFO_T));
					break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    break;
            }
            break;
        case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
            LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n", (UINT32)*feature_data);
            //PDAF capacity enable or not, 2p8 only full size support PDAF
            switch (*feature_data) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
                    break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; // video & capture use same setting
                    break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
                    break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
                    break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
                    break;
				default:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
			}
            break;
		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
			break;
		default:
			break;
	}

	return ERROR_NONE;
}	/*	feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 S5K2P8_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	s5k2p8_MIPI_RAW_SensorInit	*/
