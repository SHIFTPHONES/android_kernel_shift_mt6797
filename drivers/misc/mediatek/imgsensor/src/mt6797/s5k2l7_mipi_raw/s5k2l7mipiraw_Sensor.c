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
 *	   s5k2l7mipi_Sensor.c
 *
 * Project:
 * --------
 *	   ALPS
 *
 * Description:
 * ------------
 *	   Source code of Sensor driver
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

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/err.h>
#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k2l7mipiraw_Sensor.h"
#include "s5k2l7_setting.h"

//#include "s5k2l7_otp.h"



#define MARK_HDR

//#define TEST_PATTERN_EN
/*WDR auto ration mode*/
//#define ENABLE_WDR_AUTO_RATION
/****************************Modify Following Strings for Debug****************************/
#define PFX "s5k2l7_camera_sensor"
#define LOG_1 LOG_INF("s5k2l7,MIPI 4LANE\n")
/****************************	Modify end	  *******************************************/
#define LOG_INF(fmt, args...)	pr_debug(PFX "[%s] " fmt, __FUNCTION__, ##args)
static kal_uint32 chip_id = 0;

static DEFINE_SPINLOCK(imgsensor_drv_lock);
#define ORIGINAL_VERSION 1
//#define SLOW_MOTION_120FPS

/* sensor information is defined in each mode. */
static imgsensor_info_struct imgsensor_info =
{
    .sensor_id = S5K2L7_SENSOR_ID,   /* record sensor id defined in Kd_imgsensor.h */

    .checksum_value = 0xafb5098f,    /* checksum value for Camera Auto Test  */

    .pre = {

        .pclk = 960000000,               /* record different mode's pclk */
        .linelength = 10160,             /* record different mode's linelength */
        .framelength =3149,              /* record different mode's framelength */
        .startx = 0,                     /* record different mode's startx of grabwindow */
        .starty = 0,                     /* record different mode's starty of grabwindow */
        .grabwindow_width = 2016,        /* Dual PD: need to tg grab width / 2, p1 drv will * 2 itself */
        .grabwindow_height = 1512,       /* record different mode's height of grabwindow */
        .mipi_data_lp2hs_settle_dc = 85, /* unit , ns */
        .max_framerate = 300

    },
    .cap = {
        .pclk = 960000000,               /* record different mode's pclk */
        .linelength = 10208,             /* record different mode's linelength */
        .framelength =3134,              /* record different mode's framelength */
        .startx = 0,                     /* record different mode's startx of grabwindow */
        .starty = 0,                     /* record different mode's starty of grabwindow */
        .grabwindow_width = 4032,        /* Dual PD: need to tg grab width / 2, p1 drv will * 2 itself */
        .grabwindow_height = 3024,       /* record different mode's height of grabwindow */
        .mipi_data_lp2hs_settle_dc = 85, /* unit , ns */
        .max_framerate = 300

    },
    .cap1 = {
        .pclk = 960000000,               /* record different mode's pclk  */
        .linelength = 10208,             /* record different mode's linelength  */
        .framelength =3134,              /* record different mode's framelength */
        .startx = 0,                     /* record different mode's startx of grabwindow */
        .starty = 0,                     /* record different mode's starty of grabwindow */
        .grabwindow_width = 4032,        /* Dual PD: need to tg grab width / 2, p1 drv will * 2 itself */
        .grabwindow_height = 3024,       /* record different mode's height of grabwindow */
        .mipi_data_lp2hs_settle_dc = 85, /* unit , ns */
        .max_framerate = 300
    },

    .normal_video = {

        .pclk = 960000000,                /* record different mode's pclk  */
        .linelength = 10208,              /* record different mode's linelength */
        .framelength =3134,               /* record different mode's framelength */
        .startx = 0,                      /* record different mode's startx of grabwindow */
        .starty = 0,                      /* record different mode's starty of grabwindow */
        .grabwindow_width = 4032,         /* Dual PD: need to tg grab width / 2, p1 drv will * 2 itself */
        .grabwindow_height = 3024,        /* record different mode's height of grabwindow */
        .mipi_data_lp2hs_settle_dc = 85,  /* unit , ns */
        .max_framerate = 300

    },

    .hs_video = {
        .pclk = 960000000,
        .linelength = 10160,
        .framelength = 786,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1344,        /* record different mode's width of grabwindow */
        .grabwindow_height = 756,        /* record different mode's height of grabwindow */
        .mipi_data_lp2hs_settle_dc = 85, /* unit , ns */
        .max_framerate = 1200,
    },

    .slim_video = {
        .pclk = 960000000,
        .linelength = 10160,
        .framelength = 3149,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1344,        /* record different mode's width of grabwindow */
        .grabwindow_height = 756,        /* record different mode's height of grabwindow */
        .mipi_data_lp2hs_settle_dc = 85, /* unit , ns */
        .max_framerate = 300,

    },
    .margin = 16,
    .min_shutter = 1,
    .max_frame_length = 0xffff,
    .ae_shut_delay_frame = 0,
    .ae_sensor_gain_delay_frame = 0,
    .ae_ispGain_delay_frame = 2,
    .ihdr_support = 0,       /* 1, support; 0,not support */
    .ihdr_le_firstline = 0,  /* 1,le first ; 0, se first */
    .sensor_mode_num = 5,    /* support sensor mode num */
    .cap_delay_frame = 3,
    .pre_delay_frame = 3,
    .video_delay_frame = 3,
    .hs_video_delay_frame = 3,
    .slim_video_delay_frame = 3,
    .isp_driving_current = ISP_DRIVING_8MA,
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_OPHY_NCSI2,             /* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO, /* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
    .mclk = 24,
    .mipi_lane_num = SENSOR_MIPI_4_LANE,
    .i2c_addr_table = { 0x5A, 0x20, 0xFF},
    .i2c_speed = 300,
};

_S5K2L7_MODE1_SENSOR_INFO_
_S5K2L7_MODE2_SENSOR_INFO_
_S5K2L7_MODE3_SENSOR_INFO_


static imgsensor_struct imgsensor =
{
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
    .hdr_mode = KAL_FALSE, //sensor need support LE, SE with HDR feature
    .i2c_write_id = 0x5A,
};


/* Sensor output window information */
/* full_w; full_h; x0_offset; y0_offset; w0_size; h0_size; scale_w; scale_h; x1_offset;  y1_offset;  w1_size;  h1_size;
   x2_tg_offset;   y2_tg_offset;  w2_tg_size;  h2_tg_size;*/
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =
{
    { 4032, 3024, 0,   0, 4032, 3024, 2016, 1512, 0, 0, 2016, 1512, 0, 0, 2016, 1512}, /* Preview */
    { 4032, 3024, 0,   0, 4032, 3024, 4032, 3024, 0, 0, 4032, 3024, 0, 0, 4032, 3024}, /* capture */
    { 4032, 3024, 0,   0, 4032, 3024, 4032, 3024, 0, 0, 4032, 3024, 0, 0, 4032, 3024}, /* normal_video */
    { 4032, 3024, 0, 348, 4032, 2328, 1344,  756, 0, 0, 1344,  756, 0, 0, 1344,  756}, /* hs_video */
    { 4032, 3024, 0, 348, 4032, 2328, 1344,  756, 0, 0, 1336,  756, 0, 0, 1344,  756}, /* slim_video */
};

_S5K2L7_MODE1_WINSIZE_INFO_
_S5K2L7_MODE2_WINSIZE_INFO_
_S5K2L7_MODE3_WINSIZE_INFO_


/*VC1 None , VC2 for PDAF(DT=0X36), unit : 8bit*/
static SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[5]=
{
    /* Preview mode setting */
    {
        0x03, 0x0a,   0x00,   0x08, 0x40, 0x00,
        0x00, 0x2b, 0x0834, 0x0618, 0x00, 0x00, 0x0280, 0x0001,
        0x01, 0x31, 0x09D8, 0x017a, 0x03, 0x00, 0x0000, 0x0000
    },
    /* Capture mode setting */
    {
        0x03, 0x0a,   0x00,   0x08, 0x40, 0x00,
        0x00, 0x2b, 0x1070, 0x0C30, 0x00, 0x00, 0x0280, 0x0001,
        0x01, 0x31, 0x13b0, 0x02f4, 0x03, 0x00, 0x0000, 0x0000
    },
    /* Video mode setting */
    {
        0x02, 0x0a,   0x00,   0x08, 0x40, 0x00,
        0x00, 0x2b, 0x1070, 0x0C30, 0x01, 0x00, 0x0000, 0x0000,
        0x01, 0x31, 0x13b0, 0x02f4, 0x03, 0x00, 0x0000, 0x0000
    },
    /* HS video mode setting */
    {
        0x03, 0x0a,   0x00,   0x08, 0x40, 0x00,
        0x00, 0x2b, 0x1070, 0x0C30, 0x00, 0x00, 0x0280, 0x0001,
        0x01, 0x31, 0x0690, 0x00BD, 0x03, 0x00, 0x0000, 0x0000
    },
    /* Slim video mode setting */
    {
        0x03, 0x0a,   0x00,   0x08, 0x40, 0x00,
        0x00, 0x2b, 0x1070, 0x0C30, 0x00, 0x00, 0x0280, 0x0001,
        0x01, 0x31, 0x0690, 0x00BD, 0x03, 0x00, 0x0000, 0x0000
    }
};


//#define USE_OIS
#ifdef USE_OIS
#define OIS_I2C_WRITE_ID 0x48
#define OIS_I2C_READ_ID 0x49

#define RUMBA_OIS_CTRL	 0x0000
#define RUMBA_OIS_STATUS 0x0001
#define RUMBA_OIS_MODE	 0x0002
#define CENTERING_MODE	 0x05
#define RUMBA_OIS_OFF	 0x0030

#define RUMBA_OIS_SETTING_ADD 0x0002
#define RUMBA_OIS_PRE_SETTING 0x02
#define RUMBA_OIS_CAP_SETTING 0x01


#define RUMBA_OIS_PRE	 0
#define RUMBA_OIS_CAP	 1


static void OIS_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 3, OIS_I2C_WRITE_ID);
}
static kal_uint16 OIS_read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,OIS_I2C_READ_ID);
    return get_byte;
}
static int OIS_on(int mode)
{
    int ret = 0;
    if(mode == RUMBA_OIS_PRE_SETTING)
    {
        OIS_write_cmos_sensor(RUMBA_OIS_SETTING_ADD,RUMBA_OIS_PRE_SETTING);
    }
    if(mode == RUMBA_OIS_CAP_SETTING)
    {
        OIS_write_cmos_sensor(RUMBA_OIS_SETTING_ADD,RUMBA_OIS_CAP_SETTING);
    }
    OIS_write_cmos_sensor(RUMBA_OIS_MODE,CENTERING_MODE);
    ret = OIS_read_cmos_sensor(RUMBA_OIS_MODE);
    LOG_INF("pangfei OIS ret=%d %s %d\n",ret,__func__,__LINE__);

    if(ret != CENTERING_MODE)
    {
        //return -1;
    }
    OIS_write_cmos_sensor(RUMBA_OIS_CTRL,0x01);
    ret = OIS_read_cmos_sensor(RUMBA_OIS_CTRL);
    LOG_INF("pangfei OIS ret=%d %s %d\n",ret,__func__,__LINE__);
    if(ret != 0x01)
    {
        //return -1;
    }

}

static int OIS_off(void)
{
    int ret = 0;
    OIS_write_cmos_sensor(RUMBA_OIS_OFF,0x01);
    ret = OIS_read_cmos_sensor(RUMBA_OIS_OFF);
    LOG_INF("pangfei OIS ret=%d %s %d\n",ret,__func__,__LINE__);
}
#endif
//add for s5k2l7 pdaf
static SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
    .i4OffsetX = 0,
    .i4OffsetY = 0,
    .i4PitchX  = 0,
    .i4PitchY  = 0,
    .i4PairNum	=0,
    .i4SubBlkW	=0,
    .i4SubBlkH	=0,
    .i4PosL = {{0,0}},
    .i4PosR = {{0,0}},
    .i4BlockNumX = 0,
    .i4BlockNumY = 0,
    .i4LeFirst = 0,
    .i4Crop = {{0,0},{0,0},{0,0},{0,0},{0,0}, \
        {0,0},{0,0},{0,0},{0,0},{0,0}
    },
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;

    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    //iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);
    iReadRegI2CTiming(pu_send_cmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id,imgsensor_info.i2c_speed);

    return get_byte;
}
static kal_uint16 read_cmos_sensor_twobyte(kal_uint32 addr)
{
    kal_uint16 get_byte = 0;
    char get_word[2];
    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    //iReadRegI2C(pu_send_cmd, 2, get_word, 2, imgsensor.i2c_write_id);
    iReadRegI2CTiming(pu_send_cmd, 2, get_word, 2, imgsensor.i2c_write_id,imgsensor_info.i2c_speed);
    get_byte = (((int)get_word[0])<<8) | get_word[1];
    return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    //iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
    iWriteRegI2CTiming(pu_send_cmd, 3, imgsensor.i2c_write_id,imgsensor_info.i2c_speed);
}

static void write_cmos_sensor_twobyte(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8),(char)(para & 0xFF)};
    //LOG_INF("write_cmos_sensor_twobyte is %x,%x,%x,%x\n", pu_send_cmd[0], pu_send_cmd[1], pu_send_cmd[2], pu_send_cmd[3]);
    //iWriteRegI2C(pu_send_cmd, 4, imgsensor.i2c_write_id);
    iWriteRegI2CTiming(pu_send_cmd, 4, imgsensor.i2c_write_id,imgsensor_info.i2c_speed);
}
static void set_dummy(void)
{
#if 1
    LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
    //write_cmos_sensor(0x0104, 0x01);
    //write_cmos_sensor_twobyte(0x6028,0x4000);
    //write_cmos_sensor_twobyte(0x602A,0xC340 );
    write_cmos_sensor_twobyte(0x0340, imgsensor.frame_length);

    //write_cmos_sensor_twobyte(0x602A,0xC342 );
    write_cmos_sensor_twobyte(0x0342, imgsensor.line_length);

    //write_cmos_sensor(0x0104, 0x00);
#endif
#if 0
    LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
    write_cmos_sensor(0x0104, 0x01);
    write_cmos_sensor_twobyte(0x0340, imgsensor.frame_length);
    write_cmos_sensor_twobyte(0x0342, imgsensor.line_length);
    write_cmos_sensor(0x0104, 0x00);
#endif
}	 /*    set_dummy  */

static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
    //kal_int16 dummy_line;
    kal_uint32 frame_length = imgsensor.frame_length;
    //unsigned long flags;

    LOG_INF("framerate = %d, min framelength should enable = %d \n", framerate,min_framelength_en);

    frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
    spin_lock(&imgsensor_drv_lock);
    imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
    imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
    {
        imgsensor.frame_length = imgsensor_info.max_frame_length;
        imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
    }
    if (min_framelength_en)
        imgsensor.min_frame_length = imgsensor.frame_length;
    spin_unlock(&imgsensor_drv_lock);
    set_dummy();
}	 /*    set_max_framerate  */


/*************************************************************************
* FUNCTION
*	 set_shutter
*
* DESCRIPTION
*	 This function set e-shutter of sensor to change exposure time.
*	 The registers 0x3500 ,0x3501 and 0x3502 control exposure of s5k2l7.
*	 The exposure value is in number of Tline, where Tline is the time of sensor one line.
*
*	 Exposure = [reg 0x3500]<<12 + [reg 0x3501]<<4 + [reg 0x3502]>>4;
*	 The maximum exposure value is limited by VTS defined by register 0x380e and 0x380f.
	  Maximum Exposure <= VTS -4
*
* PARAMETERS
*	 iShutter : exposured lines
*
* RETURNS
*	 None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
    //LOG_INF("enter xxxx  set_shutter, shutter =%d\n", shutter);

    unsigned long flags;
    //kal_uint16 realtime_fps = 0;
    //kal_uint32 frame_length = 0;
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    LOG_INF("set_shutter =%d\n", shutter);
    // OV Recommend Solution
    // if shutter bigger than frame_length, should extend frame length first
    if(!shutter) shutter = 1; /*avoid 0*/
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


// Frame length :4000 C340
    //write_cmos_sensor_twobyte(0x6028,0x4000);
    //write_cmos_sensor_twobyte(0x602A,0xC340 );
    write_cmos_sensor_twobyte(0x0340, imgsensor.frame_length);

//Shutter reg : 4000 C202
    //write_cmos_sensor_twobyte(0x6028,0x4000);
    //write_cmos_sensor_twobyte(0x602A,0xC202 );
    write_cmos_sensor_twobyte(0x0202,shutter);
    write_cmos_sensor_twobyte(0x0226,shutter);

}	 /*    set_shutter */

static void hdr_write_shutter(kal_uint16 le, kal_uint16 se)
{
    //LOG_INF("enter xxxx  set_shutter, shutter =%d\n", shutter);
    unsigned int iRation;

    unsigned long flags;
    //kal_uint16 realtime_fps = 0;
    //kal_uint32 frame_length = 0;
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = le;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
    if(!le) le = 1; /*avoid 0*/

    spin_lock(&imgsensor_drv_lock);
    if (le > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = le + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;

    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;

    spin_unlock(&imgsensor_drv_lock);

    le = (le < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : le;
    le = (le > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : le;

    // Frame length :4000 C340
    //write_cmos_sensor_twobyte(0x6028,0x4000);
    //write_cmos_sensor_twobyte(0x602A,0xC340 );
    write_cmos_sensor_twobyte(0x0340, imgsensor.frame_length);

    //SET LE/SE ration
    //iRation = (((LE + SE/2)/SE) >> 1 ) << 1 ;
    iRation = ((10 * le / se) + 5) / 10;
    if(iRation < 2)
        iRation = 1;
    else if(iRation < 4)
        iRation = 2;
    else if(iRation < 8)
        iRation = 4;
    else if(iRation < 16)
        iRation = 8;
    else if(iRation < 32)
        iRation = 16;
    else
        iRation = 1;

    /*set ration for auto */
    iRation = 0x100 * iRation;
#if defined(ENABLE_WDR_AUTO_RATION)
    /*LE / SE ration ,	0x218/0x21a =  LE Ration*/
    /*0x218 =0x400, 0x21a=0x100, LE/SE = 4x*/
    write_cmos_sensor_twobyte(0x0218, iRation);
    write_cmos_sensor_twobyte(0x021a, 0x100);
#endif
    /*Short exposure */
    write_cmos_sensor_twobyte(0x0202,se);
    /*Log exposure ratio*/
    write_cmos_sensor_twobyte(0x0226,le);

    LOG_INF("HDR set shutter LE=%d, SE=%d, iRation=0x%x\n", le, se,iRation);

}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
    kal_uint16 reg_gain = 0x0;
    reg_gain = gain/2;
    return (kal_uint16)reg_gain;
}

/*************************************************************************
* FUNCTION
*	 set_gain
*
* DESCRIPTION
*	 This function is to set global gain to sensor.
*
* PARAMETERS
*	 iGain : sensor global gain(base: 0x80)
*
* RETURNS
*	 the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{

    kal_uint16 reg_gain;
    kal_uint32 sensor_gain1 = 0;
    kal_uint32 sensor_gain2 = 0;
    /* 0x350A[0:1], 0x350B[0:7] AGC real gain */
    /* [0:3] = N meams N /16 X	*/
    /* [4:9] = M meams M X		 */
    /* Total gain = M + N /16 X   */

    //
    if (gain < BASEGAIN || gain > 32 * BASEGAIN)
    {
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
    //LOG_INF("gain = %d , reg_gain = 0x%x ", gain, reg_gain);

    //Analog gain HW reg : 4000 C204

    write_cmos_sensor_twobyte(0x6028,0x4000);
    write_cmos_sensor_twobyte(0x602A,0x0204 );
    write_cmos_sensor_twobyte(0x6F12,reg_gain);
    write_cmos_sensor_twobyte(0x6F12,reg_gain);


    write_cmos_sensor_twobyte(0x602C,0x4000);
    write_cmos_sensor_twobyte(0x602E, 0x0204);
    sensor_gain1 = read_cmos_sensor_twobyte(0x6F12);

    write_cmos_sensor_twobyte(0x602C,0x4000);
    write_cmos_sensor_twobyte(0x602E, 0x0206);
    sensor_gain2 = read_cmos_sensor_twobyte(0x6F12);
    LOG_INF("imgsensor.gain(0x%x), gain1(0x%x), gain2(0x%x)\n",imgsensor.gain, sensor_gain1, sensor_gain2);

    return gain;

}	 /*    set_gain  */

static void set_mirror_flip(kal_uint8 image_mirror)
{
    LOG_INF("image_mirror = %d\n", image_mirror);
#if 1
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
    switch (image_mirror)
    {

    case IMAGE_NORMAL:
        write_cmos_sensor(0x0101,0x00);   // Gr
        break;
    case IMAGE_H_MIRROR:
        write_cmos_sensor(0x0101,0x01);
        break;
    case IMAGE_V_MIRROR:
        write_cmos_sensor(0x0101,0x02);
        break;
    case IMAGE_HV_MIRROR:
        write_cmos_sensor(0x0101,0x03);//Gb
        break;
    default:
        LOG_INF("Error image_mirror setting\n");

    }
#endif
}

/*************************************************************************
* FUNCTION
*	 night_mode
*
* DESCRIPTION
*	 This function night mode of sensor.
*
* PARAMETERS
*	 bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	 None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
    /*No Need to implement this function*/
}	 /*    night_mode	 */

// #define	S5K2l7FW


#ifndef MARK_HDR
static void sensor_WDR_zhdr(void)
{
    if(imgsensor.hdr_mode == 9)
    {
        LOG_INF("sensor_WDR_zhdr\n");
        /*it would write 0x21E = 0x1, 0x21F=0x00*/
        /*0x21E=1 , Enable WDR*/
        /*0x21F=0x00, Use Manual mode to set short /long exp */
#if defined(ENABLE_WDR_AUTO_RATION)
        write_cmos_sensor_twobyte(0x021E, 0x0101); /*For WDR auot ration*/
#else
        write_cmos_sensor_twobyte(0x021E, 0x0100); /*For WDR manual ration*/
#endif
        write_cmos_sensor_twobyte(0x0220, 0x0801);
        write_cmos_sensor_twobyte(0x0222, 0x0100);
    }
    else
    {
        write_cmos_sensor_twobyte(0x021E, 0x0000);
        write_cmos_sensor_twobyte(0x0220, 0x0801);
    }
    /*for LE/SE Test*/
    //hdr_write_shutter(3460,800);

}
#endif

static void sensor_init_11_new(void)
{
    LOG_INF("E S5k2L7 sensor init (%d)\n", pdaf_sensor_mode);
    if( pdaf_sensor_mode == 1)
    {
        _S5K2L7_MODE1_INIT_;
    }
    else if( pdaf_sensor_mode == 2)
    {
        _S5K2L7_MODE2_INIT_;
    }
    else
    {
        _S5K2L7_MODE3_INIT_;
    }
}

static void preview_setting_11_new(void)
{
    LOG_INF("E S5k2L7 preview setting (%d)\n", pdaf_sensor_mode);
    if( pdaf_sensor_mode == 1)
    {
        _S5K2L7_MODE1_PREVIEW_;
    }
    else if( pdaf_sensor_mode == 2)
    {
        _S5K2L7_MODE2_PREVIEW_;
    }
    else
    {
        _S5K2L7_MODE3_PREVIEW_;
    }
}

#ifndef MARK_HDR
static void capture_setting_WDR(kal_uint16 currefps)
{
    LOG_INF("E S5k2L7 Capture WDR, fps = %d (%d)\n", currefps, pdaf_sensor_mode);
    if( pdaf_sensor_mode == 1)
    {
        _S5K2L7_MODE1_CAPTURE_WDR_;
    }
    else if( pdaf_sensor_mode == 2)
    {
        _S5K2L7_MODE2_CAPTURE_WDR_;
    }
    else
    {
        _S5K2L7_MODE3_CAPTURE_WDR_;
    }
}
#endif

static void capture_setting(void)
{
    LOG_INF("E S5k2L7 capture setting (%d)\n", pdaf_sensor_mode);
    if( pdaf_sensor_mode == 1)
    {
        _S5K2L7_MODE1_CAPTURE_;
    }
    else if( pdaf_sensor_mode == 2)
    {
        _S5K2L7_MODE2_CAPTURE_;
    }
    else
    {
        _S5K2L7_MODE3_CAPTURE_;
    }
}

#if 0
static void normal_video_setting_11_new(kal_uint16 currefps)
{}
#endif

static void hs_video_setting_11(void)
{
    LOG_INF("E S5k2L7 HS video setting (%d)\n", pdaf_sensor_mode);

    if( pdaf_sensor_mode == 1)
    {
        _S5K2L7_MODE1_HS_VIDEO_;
    }
    else if( pdaf_sensor_mode == 2)
    {
        _S5K2L7_MODE2_HS_VIDEO_;
    }
    else
    {
        _S5K2L7_MODE3_HS_VIDEO_;
    }
}


static void slim_video_setting(void)
{
    LOG_INF("E S5k2L7 slim video setting (%d)\n", pdaf_sensor_mode);

    if( pdaf_sensor_mode == 1)
    {
        _S5K2L7_MODE1_SLIM_VIDEO_;
    }
    else if( pdaf_sensor_mode == 2)
    {
        _S5K2L7_MODE2_SLIM_VIDEO_;
    }
    else
    {
        _S5K2L7_MODE3_SLIM_VIDEO_;
    }

}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
    LOG_INF("enable: %d\n", enable);

    /********************************************************

    *0x5040[7]: 1 enable,  0 disable
    *0x5040[3:2]; color bar style 00 standard color bar
    *0x5040[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
    ********************************************************/


    if (enable)
    {
        write_cmos_sensor(0x0600, 0x000C); // Grayscale
    }
    else
    {
        write_cmos_sensor(0x0600, 0x0000); // Off
    }
    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = enable;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*	 get_imgsensor_id
*
* DESCRIPTION
*	 This function get the sensor ID
*
* PARAMETERS
*	 *sensorID : return the sensor ID
*
* RETURNS
*	 None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 5;

    pdaf_sensor_mode = proc_pdaf_sensor_mode;

    LOG_INF("get_imgsensor_id pdaf sensor mode %d\n", pdaf_sensor_mode);

    if( pdaf_sensor_mode==1)
    {
        _SET_MODE1_SENSOR_INFO_AND_WINSIZE_;
    }
    else if( pdaf_sensor_mode==2)
    {
        _SET_MODE2_SENSOR_INFO_AND_WINSIZE_;
    }
    else
    {
        _SET_MODE3_SENSOR_INFO_AND_WINSIZE_;
    }


    while (imgsensor_info.i2c_addr_table[i] != 0xff)
    {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do
        {

            *sensor_id = read_cmos_sensor_twobyte(0x0000);
            if (*sensor_id == imgsensor_info.sensor_id || *sensor_id == 0x20C1)
            {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);

                return ERROR_NONE;
            }
            LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
            retry--;
        }
        while(retry > 0);
        i++;
        retry = 2;
    }
    if (*sensor_id != imgsensor_info.sensor_id && *sensor_id != 0x20C1)
    {
        // if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	 open
*
* DESCRIPTION
*	 This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	 None
*
* RETURNS
*	 None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{

    kal_uint8 i = 0;
    kal_uint8 retry = 5;
    kal_uint32 sensor_id = 0;
    //kal_uint32 chip_id = 0;

    LOG_1;
    //LOG_2;
#if 1
    while (imgsensor_info.i2c_addr_table[i] != 0xff)
    {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do
        {
            write_cmos_sensor_twobyte(0x602C,0x4000);
            write_cmos_sensor_twobyte(0x602E,0x0000);
            sensor_id = read_cmos_sensor_twobyte(0x6F12);
            //LOG_INF("JEFF get_imgsensor_id-read sensor ID (0x%x)\n", sensor_id );

            write_cmos_sensor_twobyte(0x602C,0x4000);
            write_cmos_sensor_twobyte(0x602E,0x001A);
            chip_id = read_cmos_sensor_twobyte(0x6F12);
            //chip_id = read_cmos_sensor_twobyte(0x001A);
            //LOG_INF("get_imgsensor_id-read chip_id (0x%x)\n", chip_id );

            if (sensor_id == imgsensor_info.sensor_id  || sensor_id == 0x20C1)
            {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x, chip_id (0x%x)\n", imgsensor.i2c_write_id,sensor_id, chip_id);
                break;
            }
            LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
            retry--;
        }
        while(retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id || sensor_id == 0x20C1)
            break;
        retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id && 0x20C1 != sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;
#endif

    write_cmos_sensor_twobyte(0x602C,0x4000);
    write_cmos_sensor_twobyte(0x602E,0x001A);
    chip_id = read_cmos_sensor_twobyte(0x6F12);
    //chip_id = read_cmos_sensor_twobyte(0x001A);
    LOG_INF("JEFF get_imgsensor_id-read chip_id (0x%x)\n", chip_id );
    /* initail sequence write in  */
    //chip_id == 0x022C
    sensor_init_11_new();

#ifdef	USE_OIS
    //OIS_on(RUMBA_OIS_CAP_SETTING);//pangfei OIS
    LOG_INF("pangfei capture OIS setting\n");
    OIS_write_cmos_sensor(0x0002,0x05);
    OIS_write_cmos_sensor(0x0002,0x00);
    OIS_write_cmos_sensor(0x0000,0x01);
#endif

    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en= KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.hdr_mode = KAL_FALSE;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}	 /*    open  */



/*************************************************************************
* FUNCTION
*	 close
*
* DESCRIPTION
*
*
* PARAMETERS
*	 None
*
* RETURNS
*	 None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
    LOG_INF("E\n");

    /*No Need to implement this function*/

    return ERROR_NONE;
}	 /*    close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	 This function start the sensor preview.
*
* PARAMETERS
*	 *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	 None
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
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting_11_new();

    //set_mirror_flip(IMAGE_NORMAL);

#ifdef USE_OIS
    //OIS_on(RUMBA_OIS_PRE_SETTING);	//pangfei OIS
    LOG_INF("pangfei preview OIS setting\n");
    OIS_write_cmos_sensor(0x0002,0x05);
    OIS_write_cmos_sensor(0x0002,0x00);
    OIS_write_cmos_sensor(0x0000,0x01);
#endif
    return ERROR_NONE;
}	 /*    preview	 */

/*************************************************************************
* FUNCTION
*	 capture
*
* DESCRIPTION
*	 This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	 None
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

    /* Mark PIP case for dual pd */
    if (0) // (imgsensor.current_fps == imgsensor_info.cap1.max_framerate)
    {
        //PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
        imgsensor.pclk = imgsensor_info.cap1.pclk;
        imgsensor.line_length = imgsensor_info.cap1.linelength;
        imgsensor.frame_length = imgsensor_info.cap1.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }
    else
    {
        if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
            LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap.max_framerate/10);
        imgsensor.pclk = imgsensor_info.cap.pclk;
        imgsensor.line_length = imgsensor_info.cap.linelength;
        imgsensor.frame_length = imgsensor_info.cap.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }
    spin_unlock(&imgsensor_drv_lock);


    /* Mark HDR setting mode for dual pd sensor */
#ifndef MARK_HDR
    if(imgsensor.hdr_mode == 9)
        capture_setting_WDR(imgsensor.current_fps);
    else
#endif
        capture_setting();


    //set_mirror_flip(IMAGE_NORMAL);
#ifdef	USE_OIS
    //OIS_on(RUMBA_OIS_CAP_SETTING);//pangfei OIS
    LOG_INF("pangfei capture OIS setting\n");
    OIS_write_cmos_sensor(0x0002,0x05);
    OIS_write_cmos_sensor(0x0002,0x00);
    OIS_write_cmos_sensor(0x0000,0x01);
#endif
    return ERROR_NONE;
}	 /* capture() */
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
    //imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    capture_setting();

    //set_mirror_flip(IMAGE_NORMAL);
    return ERROR_NONE;
}	 /*    normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                           MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

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

    hs_video_setting_11();

    set_mirror_flip(IMAGE_NORMAL);
    return ERROR_NONE;
}	 /*    hs_video   */

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
    set_mirror_flip(IMAGE_NORMAL);

    return ERROR_NONE;
}	 /*    slim_video	  */

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E\n");
    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


    sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight	  = imgsensor_info.hs_video.grabwindow_height;

    sensor_resolution->SensorSlimVideoWidth 	= imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;

    return ERROR_NONE;
}	 /*    get_resolution	 */

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
                           MSDK_SENSOR_INFO_STRUCT *sensor_info,
                           MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);


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

    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
    sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

    /*0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode(Full), 3:PDAF VC mode(Binning), 4: PDAF DualPD Raw Data mode, 5: PDAF DualPD VC mode*/
    if( pdaf_sensor_mode==1)
    {
        sensor_info->PDAF_Support = 4;
    }
    else if( pdaf_sensor_mode==3)
    {
        sensor_info->PDAF_Support = 5;
    }
    else
    {
        sensor_info->PDAF_Support = 0;
    }

    sensor_info->HDR_Support = 0; /*0: NO HDR, 1: iHDR, 2:mvHDR, 3:zHDR*/

    /*0: no support, 1: G0,R0.B0, 2: G0,R0.B1, 3: G0,R1.B0, 4: G0,R1.B1*/
    /*					  5: G1,R0.B0, 6: G1,R0.B1, 7: G1,R1.B0, 8: G1,R1.B1*/
    sensor_info->ZHDR_Mode = 0;

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
    sensor_info->SensorHightSampling = 0;	 // 0 is default 1x
    sensor_info->SensorPacketECCOrder = 1;

    switch (scenario_id)
    {
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
    default:
        sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
        break;
    }

    return ERROR_NONE;
}	 /*    get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.current_scenario_id = scenario_id;
    spin_unlock(&imgsensor_drv_lock);
    switch (scenario_id)
    {
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
}	 /* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
    //This Function not used after ROME
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
    LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
    spin_lock(&imgsensor_drv_lock);
    if (enable) //enable auto flicker
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

    switch (scenario_id)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
        imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        //set_dummy();
        break;
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        if(framerate == 0)
            return ERROR_NONE;
        frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
        imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        //set_dummy();
        break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate)
        {
            frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
        }
        else
        {
            if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
            frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
        }
        //set_dummy();
        break;
    case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
        imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        //set_dummy();
        break;
    case MSDK_SCENARIO_ID_SLIM_VIDEO:
        frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
        imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        //set_dummy();
        break;
    default:  //coding with  preview scenario by default
        frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
        imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        //set_dummy();
        LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
        break;
    }
    return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
    LOG_INF("scenario_id = %d\n", scenario_id);

    switch (scenario_id)
    {
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

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                                  UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT32 sensor_id;
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
    //unsigned long long *feature_return_para=(unsigned long long *) feature_para;

    SET_PD_BLOCK_INFO_T *PDAFinfo;
    SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    SENSOR_VC_INFO_STRUCT *pvcinfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

    /*feature_id = SENSOR_FEATURE_SET_ESHUTTER(0x3004)&SENSOR_FEATURE_SET_GAIN(0x3006)*/
    if((feature_id != 0x3004) || (feature_id != 0x3006))
        LOG_INF("feature_id = %d\n", feature_id);

    switch (feature_id)
    {
    case SENSOR_FEATURE_GET_PERIOD:
        *feature_return_para_16++ = imgsensor.line_length;
        *feature_return_para_16 = imgsensor.frame_length;
        *feature_para_len=4;
        break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
        *feature_return_para_32 = imgsensor.pclk;
        *feature_para_len=4;
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
    case SENSOR_FEATURE_GET_PDAF_DATA:
        get_imgsensor_id(&sensor_id);
        //add for s5k2l7 pdaf
        LOG_INF("s5k2l7_read_otp_pdaf_data %x\n",sensor_id);
        s5k2l7_read_otp_pdaf_data((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)), sensor_id);
        break;
    case SENSOR_FEATURE_SET_TEST_PATTERN:
        set_test_pattern_mode((BOOL)*feature_data);
        break;
    case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
        *feature_return_para_32 = imgsensor_info.checksum_value;
        *feature_para_len=4;
        break;
    case SENSOR_FEATURE_SET_FRAMERATE:
        spin_lock(&imgsensor_drv_lock);
        imgsensor.current_fps = *feature_data_32;
        spin_unlock(&imgsensor_drv_lock);
        LOG_INF("current fps :%d\n", imgsensor.current_fps);
        break;
    case SENSOR_FEATURE_SET_HDR:
        LOG_INF("hdr mode :%d\n", (BOOL)*feature_data);
        spin_lock(&imgsensor_drv_lock);
        imgsensor.hdr_mode = (BOOL)*feature_data;
        spin_unlock(&imgsensor_drv_lock);
        break;
    case SENSOR_FEATURE_GET_CROP_INFO:/*0x3080*/
        //LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%lld\n", *feature_data);

        wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

        switch (*feature_data_32)
        {
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

    //add for s5k2l7 pdaf
    case SENSOR_FEATURE_GET_PDAF_INFO:
        LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%lld\n", *feature_data);
        PDAFinfo= (SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));

        switch (*feature_data)
        {
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

    case SENSOR_FEATURE_GET_VC_INFO:
        LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16)*feature_data);
        pvcinfo = (SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
        if( pdaf_sensor_mode==3)
        {
            switch (*feature_data_32)
            {
            case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[1],sizeof(SENSOR_VC_INFO_STRUCT));
                break;
            case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[2],sizeof(SENSOR_VC_INFO_STRUCT));
                break;
            case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[3],sizeof(SENSOR_VC_INFO_STRUCT));
                break;
            case MSDK_SCENARIO_ID_SLIM_VIDEO:
                memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[4],sizeof(SENSOR_VC_INFO_STRUCT));
                break;
            case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            default:
                memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[0],sizeof(SENSOR_VC_INFO_STRUCT));
                break;
            }
        }
        else
        {
            memset((void *)pvcinfo, 0, sizeof(SENSOR_VC_INFO_STRUCT));
        }
        break;


    case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
        LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%lld\n", *feature_data);
        //PDAF capacity enable or not, s5k2l7 only full size support PDAF
        switch (*feature_data)
        {
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            if( pdaf_sensor_mode==1)
                *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            else if( pdaf_sensor_mode==3)
                *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            else
                *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
            break;

        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            if( pdaf_sensor_mode==1)
                *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            else if( pdaf_sensor_mode==3)
                *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            else
                *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
            break;

        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            if( pdaf_sensor_mode==1)
                *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
            else if( pdaf_sensor_mode==3)
                *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            else
                *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
            break;

        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            if( pdaf_sensor_mode==1)
                *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
            else if( pdaf_sensor_mode==3)
                *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            else
                *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
            break;

        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            if( pdaf_sensor_mode==1)
                *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            else if( pdaf_sensor_mode==3)
                *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            else
                *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
            break;


        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
            break;
        }
        break;

    case SENSOR_FEATURE_SET_HDR_SHUTTER:
        LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1));
        hdr_write_shutter((UINT16)*feature_data,(UINT16)*(feature_data+1));
        break;

    case SENSOR_FEATURE_GET_PDAF_TYPE:
        *feature_para = pdaf_sensor_mode;
        if( pdaf_sensor_mode==1)
            sprintf(feature_para, "configure S5K2L7 as mode 1");
        else if( pdaf_sensor_mode==2)
            sprintf(feature_para, "configure S5K2L7 as mode 2");
        else if(pdaf_sensor_mode==3)
            sprintf(feature_para, "configure S5K2L7 as mode 3");
        else
            sprintf(feature_para, "configure S5K2L7 as unknow mode");

        LOG_INF("get PDAF type = %d\n", pdaf_sensor_mode);
        break;

    case SENSOR_FEATURE_SET_PDAF_TYPE:
        if( strstr( &(*feature_para), "mode1"))
        {
            LOG_INF("configure PDAF as mode 1\n");
            proc_pdaf_sensor_mode = 1;
        }
        else if( strstr( &(*feature_para), "mode3"))
        {
            LOG_INF("configure PDAF as mode 3\n");
            proc_pdaf_sensor_mode = 3;
        }
        else if( strstr( &(*feature_para), "mode2"))
        {
            LOG_INF("configure PDAF as mode 2\n");
            proc_pdaf_sensor_mode = 2;
        }
        else
        {
            LOG_INF("configure PDAF as unknow mode\n");
            proc_pdaf_sensor_mode = 1;
        }
        break;


    default:
        break;
    }

    return ERROR_NONE;
}	/*	feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func =
{
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 S5K2L7_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&sensor_func;
    return ERROR_NONE;
}	 /*    s5k2l7_MIPI_RAW_SensorInit	 */
