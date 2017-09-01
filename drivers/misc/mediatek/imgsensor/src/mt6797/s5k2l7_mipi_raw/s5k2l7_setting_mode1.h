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
 *     s5k2l7_setting_mode1.h
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     CMOS sensor setting file
 * 
 * Setting Release Date:
 * ------------
 *     2016.09.01
 *
 ****************************************************************************/
#ifndef _s5k2l7MIPI_SETTING_MODE1_H_
#define _s5k2l7MIPI_SETTING_MODE1_H_

#define _S5K2L7_MODE1_SENSOR_INFO_ static imgsensor_info_struct _imgsensor_info_m1 =                           \
{                                                                                                              \
    .sensor_id = S5K2L7_SENSOR_ID,   /* record sensor id defined in Kd_imgsensor.h */                          \
                                                                                                               \
    .checksum_value = 0xafb5098f,    /* checksum value for Camera Auto Test  */                                \
                                                                                                               \
    .pre = {                                                                                                   \
                                                                                                               \
        .pclk = 960000000,               /* record different mode's pclk */                                    \
        .linelength = 10160,             /* record different mode's linelength */                              \
        .framelength =3149,              /* record different mode's framelength */                             \
        .startx = 0,                     /* record different mode's startx of grabwindow */                    \
        .starty = 0,                     /* record different mode's starty of grabwindow */                    \
        .grabwindow_width = 2016,        /* Dual PD: need to tg grab width / 2, p1 drv will * 2 itself */      \
        .grabwindow_height = 1512,       /* record different mode's height of grabwindow */                    \
        .mipi_data_lp2hs_settle_dc = 85, /* unit , ns */                                                       \
        .max_framerate = 300                                                                                   \
                                                                                                               \
    },                                                                                                         \
    .cap = {                                                                                                   \
        .pclk = 960000000,               /* record different mode's pclk */                                    \
        .linelength = 10208,             /* record different mode's linelength */                              \
        .framelength =3134,              /* record different mode's framelength */                             \
        .startx = 0,                     /* record different mode's startx of grabwindow */                    \
        .starty = 0,                     /* record different mode's starty of grabwindow */                    \
        .grabwindow_width = 4032,        /* Dual PD: need to tg grab width / 2, p1 drv will * 2 itself */      \
        .grabwindow_height = 3024,       /* record different mode's height of grabwindow */                    \
        .mipi_data_lp2hs_settle_dc = 85, /* unit , ns */                                                       \
        .max_framerate = 300                                                                                   \
                                                                                                               \
    },                                                                                                         \
    .cap1 = {                                                                                                  \
        .pclk = 960000000,               /* record different mode's pclk  */                                   \
        .linelength = 10208,             /* record different mode's linelength  */                             \
        .framelength =3134,              /* record different mode's framelength */                             \
        .startx = 0,                     /* record different mode's startx of grabwindow */                    \
        .starty = 0,                     /* record different mode's starty of grabwindow */                    \
        .grabwindow_width = 4032,        /* Dual PD: need to tg grab width / 2, p1 drv will * 2 itself */      \
        .grabwindow_height = 3024,       /* record different mode's height of grabwindow */                    \
        .mipi_data_lp2hs_settle_dc = 85, /* unit , ns */                                                       \
        .max_framerate = 300                                                                                   \
    },                                                                                                         \
                                                                                                               \
    .normal_video = {                                                                                          \
                                                                                                               \
        .pclk = 960000000,                /* record different mode's pclk  */                                  \
        .linelength = 10208,              /* record different mode's linelength */                             \
        .framelength =3134,               /* record different mode's framelength */                            \
        .startx = 0,                      /* record different mode's startx of grabwindow */                   \
        .starty = 0,                      /* record different mode's starty of grabwindow */                   \
        .grabwindow_width = 4032,         /* Dual PD: need to tg grab width / 2, p1 drv will * 2 itself */     \
        .grabwindow_height = 3024,        /* record different mode's height of grabwindow */                   \
        .mipi_data_lp2hs_settle_dc = 85,  /* unit , ns */                                                      \
        .max_framerate = 300                                                                                   \
                                                                                                               \
    },                                                                                                         \
                                                                                                               \
    .hs_video = {                                                                                              \
        .pclk = 960000000,                                                                                     \
        .linelength = 10160,                                                                                   \
        .framelength = 786,                                                                                   \
        .startx = 0,                                                                                           \
        .starty = 0,                                                                                           \
        .grabwindow_width = 1344,        /* record different mode's width of grabwindow */                     \
        .grabwindow_height = 756,        /* record different mode's height of grabwindow */                    \
        .mipi_data_lp2hs_settle_dc = 85, /* unit , ns */                                                       \
        .max_framerate = 1200,                                                                                  \
    },                                                                                                         \
                                                                                                               \
    .slim_video = {                                                                                            \
        .pclk = 960000000,                                                                                     \
        .linelength = 10160,                                                                                   \
        .framelength = 3149,                                                                                   \
        .startx = 0,                                                                                           \
        .starty = 0,                                                                                           \
        .grabwindow_width = 1344,        /* record different mode's width of grabwindow */                     \
        .grabwindow_height = 756,        /* record different mode's height of grabwindow */                    \
        .mipi_data_lp2hs_settle_dc = 85, /* unit , ns */                                                       \
        .max_framerate = 300,                                                                                  \
                                                                                                               \
    },                                                                                                         \
    .margin = 16,                                                                                              \
    .min_shutter = 1,                                                                                          \
    .max_frame_length = 0xffff,                                                                                \
    .ae_shut_delay_frame = 0,                                                                                  \
    .ae_sensor_gain_delay_frame = 0,                                                                           \
    .ae_ispGain_delay_frame = 2,                                                                               \
    .ihdr_support = 0,       /* 1, support; 0,not support */                                                   \
    .ihdr_le_firstline = 0,  /* 1,le first ; 0, se first */                                                    \
    .sensor_mode_num = 5,    /* support sensor mode num */                                                     \
    .cap_delay_frame = 3,                                                                                      \
    .pre_delay_frame = 3,                                                                                      \
    .video_delay_frame = 3,                                                                                    \
    .hs_video_delay_frame = 3,                                                                                 \
    .slim_video_delay_frame = 3,                                                                               \
    .isp_driving_current = ISP_DRIVING_8MA,                                                                    \
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,                                                       \
    .mipi_sensor_type = MIPI_OPHY_NCSI2,             /* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */                \
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO, /* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */ \
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,                                                   \
    .mclk = 24,                                                                                                \
    .mipi_lane_num = SENSOR_MIPI_4_LANE,                                                                       \
    .i2c_addr_table = { 0x5A, 0x20, 0xFF},                                                                     \
    .i2c_speed = 300,                                                                                          \
};

/* full_w; full_h; x0_offset; y0_offset; w0_size; h0_size; scale_w; scale_h; x1_offset;  y1_offset;  w1_size;  h1_size;
 	 x2_tg_offset;	 y2_tg_offset;	w2_tg_size;  h2_tg_size;*/
#define _S5K2L7_MODE1_WINSIZE_INFO_ static SENSOR_WINSIZE_INFO_STRUCT _imgsensor_winsize_info_m1[5] =     \
{                                                                                                         \
    { 4032, 3024, 0,   0, 4032, 3024, 2016, 1512, 0, 0, 2016, 1512, 0, 0, 2016, 1512}, /* Preview */      \
    { 4032, 3024, 0,   0, 4032, 3024, 4032, 3024, 0, 0, 4032, 3024, 0, 0, 4032, 3024}, /* capture */      \
    { 4032, 3024, 0,   0, 4032, 3024, 4032, 3024, 0, 0, 4032, 3024, 0, 0, 4032, 3024}, /* normal_video */ \
    { 4032, 3024, 0, 378, 4032, 2268, 1344,  756, 0, 0, 1344,  756, 0, 0, 1344,  756}, /* hs_video */     \
    { 4032, 3024, 0, 378, 4032, 2268, 1344,  756, 0, 0, 1336,  756, 0, 0, 1344,  756}, /* slim_video */   \
};

#define _SET_MODE1_SENSOR_INFO_AND_WINSIZE_ do{ \
    memcpy((void *)&imgsensor_info, (void *)&_imgsensor_info_m1, sizeof(imgsensor_info_struct)); \
    memcpy((void *)&imgsensor_winsize_info, (void *)&_imgsensor_winsize_info_m1, sizeof(SENSOR_WINSIZE_INFO_STRUCT)*5); \
}while(0)


/*****************************************************************************
 *
 * Description:
 * ------------
 *     mode 1 initial setting
 *
 ****************************************************************************/
#define _S5K2L7_MODE1_INIT_ do{ \
    write_cmos_sensor_twobyte(0X6028, 0X2000); \
    write_cmos_sensor_twobyte(0X602A, 0XBBF8); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X6028, 0X4000); \
    write_cmos_sensor_twobyte(0X6018, 0X0001); \
    write_cmos_sensor_twobyte(0X7002, 0X000C); \
    write_cmos_sensor_twobyte(0X6014, 0X0001); \
    mdelay(8);                                 \
    write_cmos_sensor_twobyte(0X6214, 0X7970); \
    write_cmos_sensor_twobyte(0X6218, 0X7150); \
    write_cmos_sensor_twobyte(0X6028, 0X2000); \
    write_cmos_sensor_twobyte(0X602A, 0X87AC); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X6F12, 0X0449); \
    write_cmos_sensor_twobyte(0X6F12, 0X0348); \
    write_cmos_sensor_twobyte(0X6F12, 0X044A); \
    write_cmos_sensor_twobyte(0X6F12, 0X4860); \
    write_cmos_sensor_twobyte(0X6F12, 0X101A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0881); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F0); \
    write_cmos_sensor_twobyte(0X6F12, 0X2AB9); \
    write_cmos_sensor_twobyte(0X6F12, 0X2000); \
    write_cmos_sensor_twobyte(0X6F12, 0X8B32); \
    write_cmos_sensor_twobyte(0X6F12, 0X2000); \
    write_cmos_sensor_twobyte(0X6F12, 0X5C20); \
    write_cmos_sensor_twobyte(0X6F12, 0X2000); \
    write_cmos_sensor_twobyte(0X6F12, 0XA3D4); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X6F12, 0X30B5); \
    write_cmos_sensor_twobyte(0X6F12, 0XAB4C); \
    write_cmos_sensor_twobyte(0X6F12, 0XB0F8); \
    write_cmos_sensor_twobyte(0X6F12, 0XE232); \
    write_cmos_sensor_twobyte(0X6F12, 0X608C); \
    write_cmos_sensor_twobyte(0X6F12, 0X081A); \
    write_cmos_sensor_twobyte(0X6F12, 0XA18C); \
    write_cmos_sensor_twobyte(0X6F12, 0X401E); \
    write_cmos_sensor_twobyte(0X6F12, 0X1944); \
    write_cmos_sensor_twobyte(0X6F12, 0X8142); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D9); \
    write_cmos_sensor_twobyte(0X6F12, 0X0020); \
    write_cmos_sensor_twobyte(0X6F12, 0X0346); \
    write_cmos_sensor_twobyte(0X6F12, 0XA649); \
    write_cmos_sensor_twobyte(0X6F12, 0X9340); \
    write_cmos_sensor_twobyte(0X6F12, 0X4880); \
    write_cmos_sensor_twobyte(0X6F12, 0X9040); \
    write_cmos_sensor_twobyte(0X6F12, 0XA54A); \
    write_cmos_sensor_twobyte(0X6F12, 0X99B2); \
    write_cmos_sensor_twobyte(0X6F12, 0XD181); \
    write_cmos_sensor_twobyte(0X6F12, 0X1082); \
    write_cmos_sensor_twobyte(0X6F12, 0XA44D); \
    write_cmos_sensor_twobyte(0X6F12, 0XAD79); \
    write_cmos_sensor_twobyte(0X6F12, 0X002D); \
    write_cmos_sensor_twobyte(0X6F12, 0X06D1); \
    write_cmos_sensor_twobyte(0X6F12, 0X33B1); \
    write_cmos_sensor_twobyte(0X6F12, 0X94F8); \
    write_cmos_sensor_twobyte(0X6F12, 0XB730); \
    write_cmos_sensor_twobyte(0X6F12, 0X1BB1); \
    write_cmos_sensor_twobyte(0X6F12, 0X5181); \
    write_cmos_sensor_twobyte(0X6F12, 0X401E); \
    write_cmos_sensor_twobyte(0X6F12, 0X9081); \
    write_cmos_sensor_twobyte(0X6F12, 0X30BD); \
    write_cmos_sensor_twobyte(0X6F12, 0X4FF6); \
    write_cmos_sensor_twobyte(0X6F12, 0XFF70); \
    write_cmos_sensor_twobyte(0X6F12, 0X5081); \
    write_cmos_sensor_twobyte(0X6F12, 0XF9E7); \
    write_cmos_sensor_twobyte(0X6F12, 0X2DE9); \
    write_cmos_sensor_twobyte(0X6F12, 0XF041); \
    write_cmos_sensor_twobyte(0X6F12, 0X0646); \
    write_cmos_sensor_twobyte(0X6F12, 0X9848); \
    write_cmos_sensor_twobyte(0X6F12, 0X0D46); \
    write_cmos_sensor_twobyte(0X6F12, 0X1C38); \
    write_cmos_sensor_twobyte(0X6F12, 0X4268); \
    write_cmos_sensor_twobyte(0X6F12, 0X140C); \
    write_cmos_sensor_twobyte(0X6F12, 0X97B2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0022); \
    write_cmos_sensor_twobyte(0X6F12, 0X3946); \
    write_cmos_sensor_twobyte(0X6F12, 0X2046); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F0); \
    write_cmos_sensor_twobyte(0X6F12, 0X4BF9); \
    write_cmos_sensor_twobyte(0X6F12, 0X2946); \
    write_cmos_sensor_twobyte(0X6F12, 0X3046); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F0); \
    write_cmos_sensor_twobyte(0X6F12, 0X4CF9); \
    write_cmos_sensor_twobyte(0X6F12, 0X9148); \
    write_cmos_sensor_twobyte(0X6F12, 0X9349); \
    write_cmos_sensor_twobyte(0X6F12, 0X0122); \
    write_cmos_sensor_twobyte(0X6F12, 0X4088); \
    write_cmos_sensor_twobyte(0X6F12, 0XA1F8); \
    write_cmos_sensor_twobyte(0X6F12, 0XBC07); \
    write_cmos_sensor_twobyte(0X6F12, 0X3946); \
    write_cmos_sensor_twobyte(0X6F12, 0X2046); \
    write_cmos_sensor_twobyte(0X6F12, 0XBDE8); \
    write_cmos_sensor_twobyte(0X6F12, 0XF041); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F0); \
    write_cmos_sensor_twobyte(0X6F12, 0X3BB9); \
    write_cmos_sensor_twobyte(0X6F12, 0X70B5); \
    write_cmos_sensor_twobyte(0X6F12, 0X0646); \
    write_cmos_sensor_twobyte(0X6F12, 0X8A48); \
    write_cmos_sensor_twobyte(0X6F12, 0X0022); \
    write_cmos_sensor_twobyte(0X6F12, 0X1C38); \
    write_cmos_sensor_twobyte(0X6F12, 0X8168); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C0C); \
    write_cmos_sensor_twobyte(0X6F12, 0X8DB2); \
    write_cmos_sensor_twobyte(0X6F12, 0X2946); \
    write_cmos_sensor_twobyte(0X6F12, 0X2046); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F0); \
    write_cmos_sensor_twobyte(0X6F12, 0X2FF9); \
    write_cmos_sensor_twobyte(0X6F12, 0X3046); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F0); \
    write_cmos_sensor_twobyte(0X6F12, 0X36F9); \
    write_cmos_sensor_twobyte(0X6F12, 0X8749); \
    write_cmos_sensor_twobyte(0X6F12, 0X96F8); \
    write_cmos_sensor_twobyte(0X6F12, 0XB100); \
    write_cmos_sensor_twobyte(0X6F12, 0X0880); \
    write_cmos_sensor_twobyte(0X6F12, 0X2946); \
    write_cmos_sensor_twobyte(0X6F12, 0X2046); \
    write_cmos_sensor_twobyte(0X6F12, 0XBDE8); \
    write_cmos_sensor_twobyte(0X6F12, 0X7040); \
    write_cmos_sensor_twobyte(0X6F12, 0X0122); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F0); \
    write_cmos_sensor_twobyte(0X6F12, 0X21B9); \
    write_cmos_sensor_twobyte(0X6F12, 0X70B5); \
    write_cmos_sensor_twobyte(0X6F12, 0X0646); \
    write_cmos_sensor_twobyte(0X6F12, 0X7D48); \
    write_cmos_sensor_twobyte(0X6F12, 0X0022); \
    write_cmos_sensor_twobyte(0X6F12, 0X1C38); \
    write_cmos_sensor_twobyte(0X6F12, 0XC168); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C0C); \
    write_cmos_sensor_twobyte(0X6F12, 0X8DB2); \
    write_cmos_sensor_twobyte(0X6F12, 0X2946); \
    write_cmos_sensor_twobyte(0X6F12, 0X2046); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F0); \
    write_cmos_sensor_twobyte(0X6F12, 0X15F9); \
    write_cmos_sensor_twobyte(0X6F12, 0X3046); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F0); \
    write_cmos_sensor_twobyte(0X6F12, 0X21F9); \
    write_cmos_sensor_twobyte(0X6F12, 0X26B3); \
    write_cmos_sensor_twobyte(0X6F12, 0X7948); \
    write_cmos_sensor_twobyte(0X6F12, 0XB0F8); \
    write_cmos_sensor_twobyte(0X6F12, 0XA007); \
    write_cmos_sensor_twobyte(0X6F12, 0X00B3); \
    write_cmos_sensor_twobyte(0X6F12, 0X7748); \
    write_cmos_sensor_twobyte(0X6F12, 0X6521); \
    write_cmos_sensor_twobyte(0X6F12, 0X90F8); \
    write_cmos_sensor_twobyte(0X6F12, 0XE407); \
    write_cmos_sensor_twobyte(0X6F12, 0X4843); \
    write_cmos_sensor_twobyte(0X6F12, 0X7649); \
    write_cmos_sensor_twobyte(0X6F12, 0X01EB); \
    write_cmos_sensor_twobyte(0X6F12, 0X8003); \
    write_cmos_sensor_twobyte(0X6F12, 0X01F6); \
    write_cmos_sensor_twobyte(0X6F12, 0X9C02); \
    write_cmos_sensor_twobyte(0X6F12, 0X7549); \
    write_cmos_sensor_twobyte(0X6F12, 0XB3F8); \
    write_cmos_sensor_twobyte(0X6F12, 0X3202); \
    write_cmos_sensor_twobyte(0X6F12, 0X91F8); \
    write_cmos_sensor_twobyte(0X6F12, 0XF719); \
    write_cmos_sensor_twobyte(0X6F12, 0X32F8); \
    write_cmos_sensor_twobyte(0X6F12, 0X1110); \
    write_cmos_sensor_twobyte(0X6F12, 0X0844); \
    write_cmos_sensor_twobyte(0X6F12, 0X40F4); \
    write_cmos_sensor_twobyte(0X6F12, 0X8051); \
    write_cmos_sensor_twobyte(0X6F12, 0X7148); \
    write_cmos_sensor_twobyte(0X6F12, 0X0180); \
    write_cmos_sensor_twobyte(0X6F12, 0XB3F8); \
    write_cmos_sensor_twobyte(0X6F12, 0X3612); \
    write_cmos_sensor_twobyte(0X6F12, 0X6E4B); \
    write_cmos_sensor_twobyte(0X6F12, 0X93F8); \
    write_cmos_sensor_twobyte(0X6F12, 0XF939); \
    write_cmos_sensor_twobyte(0X6F12, 0X32F8); \
    write_cmos_sensor_twobyte(0X6F12, 0X1320); \
    write_cmos_sensor_twobyte(0X6F12, 0X1144); \
    write_cmos_sensor_twobyte(0X6F12, 0X41F4); \
    write_cmos_sensor_twobyte(0X6F12, 0X8051); \
    write_cmos_sensor_twobyte(0X6F12, 0X8180); \
    write_cmos_sensor_twobyte(0X6F12, 0X2946); \
    write_cmos_sensor_twobyte(0X6F12, 0X2046); \
    write_cmos_sensor_twobyte(0X6F12, 0XBDE8); \
    write_cmos_sensor_twobyte(0X6F12, 0X7040); \
    write_cmos_sensor_twobyte(0X6F12, 0X0122); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F0); \
    write_cmos_sensor_twobyte(0X6F12, 0XE5B8); \
    write_cmos_sensor_twobyte(0X6F12, 0X70B5); \
    write_cmos_sensor_twobyte(0X6F12, 0X674D); \
    write_cmos_sensor_twobyte(0X6F12, 0X41F2); \
    write_cmos_sensor_twobyte(0X6F12, 0XF046); \
    write_cmos_sensor_twobyte(0X6F12, 0X1544); \
    write_cmos_sensor_twobyte(0X6F12, 0X049C); \
    write_cmos_sensor_twobyte(0X6F12, 0XAD5D); \
    write_cmos_sensor_twobyte(0X6F12, 0X0DB9); \
    write_cmos_sensor_twobyte(0X6F12, 0X1B78); \
    write_cmos_sensor_twobyte(0X6F12, 0X03B1); \
    write_cmos_sensor_twobyte(0X6F12, 0X0123); \
    write_cmos_sensor_twobyte(0X6F12, 0X2370); \
    write_cmos_sensor_twobyte(0X6F12, 0X634B); \
    write_cmos_sensor_twobyte(0X6F12, 0X03EB); \
    write_cmos_sensor_twobyte(0X6F12, 0XC203); \
    write_cmos_sensor_twobyte(0X6F12, 0XB3F8); \
    write_cmos_sensor_twobyte(0X6F12, 0XD630); \
    write_cmos_sensor_twobyte(0X6F12, 0X2381); \
    write_cmos_sensor_twobyte(0X6F12, 0X90F8); \
    write_cmos_sensor_twobyte(0X6F12, 0XCC50); \
    write_cmos_sensor_twobyte(0X6F12, 0X1346); \
    write_cmos_sensor_twobyte(0X6F12, 0X05B1); \
    write_cmos_sensor_twobyte(0X6F12, 0X0223); \
    write_cmos_sensor_twobyte(0X6F12, 0X5E4D); \
    write_cmos_sensor_twobyte(0X6F12, 0X03EB); \
    write_cmos_sensor_twobyte(0X6F12, 0XC303); \
    write_cmos_sensor_twobyte(0X6F12, 0X05EB); \
    write_cmos_sensor_twobyte(0X6F12, 0X8303); \
    write_cmos_sensor_twobyte(0X6F12, 0X1B79); \
    write_cmos_sensor_twobyte(0X6F12, 0X53B1); \
    write_cmos_sensor_twobyte(0X6F12, 0X544B); \
    write_cmos_sensor_twobyte(0X6F12, 0XB3F8); \
    write_cmos_sensor_twobyte(0X6F12, 0XB855); \
    write_cmos_sensor_twobyte(0X6F12, 0X4F4B); \
    write_cmos_sensor_twobyte(0X6F12, 0X1B78); \
    write_cmos_sensor_twobyte(0X6F12, 0X03F5); \
    write_cmos_sensor_twobyte(0X6F12, 0XFC63); \
    write_cmos_sensor_twobyte(0X6F12, 0X9D42); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D9); \
    write_cmos_sensor_twobyte(0X6F12, 0X0123); \
    write_cmos_sensor_twobyte(0X6F12, 0X2370); \
    write_cmos_sensor_twobyte(0X6F12, 0X90F8); \
    write_cmos_sensor_twobyte(0X6F12, 0X2830); \
    write_cmos_sensor_twobyte(0X6F12, 0X03B1); \
    write_cmos_sensor_twobyte(0X6F12, 0X0123); \
    write_cmos_sensor_twobyte(0X6F12, 0X6370); \
    write_cmos_sensor_twobyte(0X6F12, 0X90F8); \
    write_cmos_sensor_twobyte(0X6F12, 0X1833); \
    write_cmos_sensor_twobyte(0X6F12, 0XE370); \
    write_cmos_sensor_twobyte(0X6F12, 0X90F8); \
    write_cmos_sensor_twobyte(0X6F12, 0X1703); \
    write_cmos_sensor_twobyte(0X6F12, 0XA070); \
    write_cmos_sensor_twobyte(0X6F12, 0X01EB); \
    write_cmos_sensor_twobyte(0X6F12, 0X8200); \
    write_cmos_sensor_twobyte(0X6F12, 0XD0F8); \
    write_cmos_sensor_twobyte(0X6F12, 0X2C11); \
    write_cmos_sensor_twobyte(0X6F12, 0XC1F3); \
    write_cmos_sensor_twobyte(0X6F12, 0X8F11); \
    write_cmos_sensor_twobyte(0X6F12, 0XA180); \
    write_cmos_sensor_twobyte(0X6F12, 0XD0F8); \
    write_cmos_sensor_twobyte(0X6F12, 0X3401); \
    write_cmos_sensor_twobyte(0X6F12, 0XC0F3); \
    write_cmos_sensor_twobyte(0X6F12, 0X8F10); \
    write_cmos_sensor_twobyte(0X6F12, 0XE080); \
    write_cmos_sensor_twobyte(0X6F12, 0X70BD); \
    write_cmos_sensor_twobyte(0X6F12, 0X0346); \
    write_cmos_sensor_twobyte(0X6F12, 0X30B5); \
    write_cmos_sensor_twobyte(0X6F12, 0X4148); \
    write_cmos_sensor_twobyte(0X6F12, 0XB0F8); \
    write_cmos_sensor_twobyte(0X6F12, 0XD007); \
    write_cmos_sensor_twobyte(0X6F12, 0X0844); \
    write_cmos_sensor_twobyte(0X6F12, 0X3E49); \
    write_cmos_sensor_twobyte(0X6F12, 0XB1F8); \
    write_cmos_sensor_twobyte(0X6F12, 0X6E40); \
    write_cmos_sensor_twobyte(0X6F12, 0X001B); \
    write_cmos_sensor_twobyte(0X6F12, 0X3A4C); \
    write_cmos_sensor_twobyte(0X6F12, 0X00B2); \
    write_cmos_sensor_twobyte(0X6F12, 0XA080); \
    write_cmos_sensor_twobyte(0X6F12, 0X01EB); \
    write_cmos_sensor_twobyte(0X6F12, 0X4304); \
    write_cmos_sensor_twobyte(0X6F12, 0X101A); \
    write_cmos_sensor_twobyte(0X6F12, 0XB4F9); \
    write_cmos_sensor_twobyte(0X6F12, 0X7E40); \
    write_cmos_sensor_twobyte(0X6F12, 0X2044); \
    write_cmos_sensor_twobyte(0X6F12, 0X3B4C); \
    write_cmos_sensor_twobyte(0X6F12, 0X9840); \
    write_cmos_sensor_twobyte(0X6F12, 0XD4F8); \
    write_cmos_sensor_twobyte(0X6F12, 0X0459); \
    write_cmos_sensor_twobyte(0X6F12, 0X0524); \
    write_cmos_sensor_twobyte(0X6F12, 0X9C40); \
    write_cmos_sensor_twobyte(0X6F12, 0X2C44); \
    write_cmos_sensor_twobyte(0X6F12, 0X8442); \
    write_cmos_sensor_twobyte(0X6F12, 0X04DD); \
    write_cmos_sensor_twobyte(0X6F12, 0X91F8); \
    write_cmos_sensor_twobyte(0X6F12, 0X7010); \
    write_cmos_sensor_twobyte(0X6F12, 0X9A40); \
    write_cmos_sensor_twobyte(0X6F12, 0X09B1); \
    write_cmos_sensor_twobyte(0X6F12, 0X1044); \
    write_cmos_sensor_twobyte(0X6F12, 0X30BD); \
    write_cmos_sensor_twobyte(0X6F12, 0X1046); \
    write_cmos_sensor_twobyte(0X6F12, 0X30BD); \
    write_cmos_sensor_twobyte(0X6F12, 0X70B5); \
    write_cmos_sensor_twobyte(0X6F12, 0X0446); \
    write_cmos_sensor_twobyte(0X6F12, 0X2C48); \
    write_cmos_sensor_twobyte(0X6F12, 0X0022); \
    write_cmos_sensor_twobyte(0X6F12, 0X1C38); \
    write_cmos_sensor_twobyte(0X6F12, 0X8069); \
    write_cmos_sensor_twobyte(0X6F12, 0X86B2); \
    write_cmos_sensor_twobyte(0X6F12, 0X050C); \
    write_cmos_sensor_twobyte(0X6F12, 0X3146); \
    write_cmos_sensor_twobyte(0X6F12, 0X2846); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F0); \
    write_cmos_sensor_twobyte(0X6F12, 0X74F8); \
    write_cmos_sensor_twobyte(0X6F12, 0X2046); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F0); \
    write_cmos_sensor_twobyte(0X6F12, 0X85F8); \
    write_cmos_sensor_twobyte(0X6F12, 0X94F8); \
    write_cmos_sensor_twobyte(0X6F12, 0X2500); \
    write_cmos_sensor_twobyte(0X6F12, 0X8107); \
    write_cmos_sensor_twobyte(0X6F12, 0X2448); \
    write_cmos_sensor_twobyte(0X6F12, 0X04D5); \
    write_cmos_sensor_twobyte(0X6F12, 0X6188); \
    write_cmos_sensor_twobyte(0X6F12, 0X4078); \
    write_cmos_sensor_twobyte(0X6F12, 0X0844); \
    write_cmos_sensor_twobyte(0X6F12, 0X6080); \
    write_cmos_sensor_twobyte(0X6F12, 0X03E0); \
    write_cmos_sensor_twobyte(0X6F12, 0XE188); \
    write_cmos_sensor_twobyte(0X6F12, 0X4078); \
    write_cmos_sensor_twobyte(0X6F12, 0X081A); \
    write_cmos_sensor_twobyte(0X6F12, 0XE080); \
    write_cmos_sensor_twobyte(0X6F12, 0X3146); \
    write_cmos_sensor_twobyte(0X6F12, 0X2846); \
    write_cmos_sensor_twobyte(0X6F12, 0XBDE8); \
    write_cmos_sensor_twobyte(0X6F12, 0X7040); \
    write_cmos_sensor_twobyte(0X6F12, 0X0122); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F0); \
    write_cmos_sensor_twobyte(0X6F12, 0X5CB8); \
    write_cmos_sensor_twobyte(0X6F12, 0X10B5); \
    write_cmos_sensor_twobyte(0X6F12, 0X0022); \
    write_cmos_sensor_twobyte(0X6F12, 0XAFF2); \
    write_cmos_sensor_twobyte(0X6F12, 0X4B21); \
    write_cmos_sensor_twobyte(0X6F12, 0X2448); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F0); \
    write_cmos_sensor_twobyte(0X6F12, 0X6EF8); \
    write_cmos_sensor_twobyte(0X6F12, 0X184C); \
    write_cmos_sensor_twobyte(0X6F12, 0X1C3C); \
    write_cmos_sensor_twobyte(0X6F12, 0X0122); \
    write_cmos_sensor_twobyte(0X6F12, 0XAFF2); \
    write_cmos_sensor_twobyte(0X6F12, 0X1121); \
    write_cmos_sensor_twobyte(0X6F12, 0X2060); \
    write_cmos_sensor_twobyte(0X6F12, 0X2148); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F0); \
    write_cmos_sensor_twobyte(0X6F12, 0X65F8); \
    write_cmos_sensor_twobyte(0X6F12, 0X0022); \
    write_cmos_sensor_twobyte(0X6F12, 0XAFF2); \
    write_cmos_sensor_twobyte(0X6F12, 0XE111); \
    write_cmos_sensor_twobyte(0X6F12, 0X6060); \
    write_cmos_sensor_twobyte(0X6F12, 0X1E48); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F0); \
    write_cmos_sensor_twobyte(0X6F12, 0X5EF8); \
    write_cmos_sensor_twobyte(0X6F12, 0X0022); \
    write_cmos_sensor_twobyte(0X6F12, 0XAFF2); \
    write_cmos_sensor_twobyte(0X6F12, 0XBD11); \
    write_cmos_sensor_twobyte(0X6F12, 0XA060); \
    write_cmos_sensor_twobyte(0X6F12, 0X1C48); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F0); \
    write_cmos_sensor_twobyte(0X6F12, 0X57F8); \
    write_cmos_sensor_twobyte(0X6F12, 0X0022); \
    write_cmos_sensor_twobyte(0X6F12, 0XAFF2); \
    write_cmos_sensor_twobyte(0X6F12, 0X5111); \
    write_cmos_sensor_twobyte(0X6F12, 0XE060); \
    write_cmos_sensor_twobyte(0X6F12, 0X1948); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F0); \
    write_cmos_sensor_twobyte(0X6F12, 0X50F8); \
    write_cmos_sensor_twobyte(0X6F12, 0X0022); \
    write_cmos_sensor_twobyte(0X6F12, 0XAFF2); \
    write_cmos_sensor_twobyte(0X6F12, 0XDF01); \
    write_cmos_sensor_twobyte(0X6F12, 0X2061); \
    write_cmos_sensor_twobyte(0X6F12, 0X1748); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F0); \
    write_cmos_sensor_twobyte(0X6F12, 0X49F8); \
    write_cmos_sensor_twobyte(0X6F12, 0X0022); \
    write_cmos_sensor_twobyte(0X6F12, 0XAFF2); \
    write_cmos_sensor_twobyte(0X6F12, 0XA301); \
    write_cmos_sensor_twobyte(0X6F12, 0X6061); \
    write_cmos_sensor_twobyte(0X6F12, 0X1448); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F0); \
    write_cmos_sensor_twobyte(0X6F12, 0X42F8); \
    write_cmos_sensor_twobyte(0X6F12, 0XA061); \
    write_cmos_sensor_twobyte(0X6F12, 0X10BD); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X6F12, 0X2000); \
    write_cmos_sensor_twobyte(0X6F12, 0X0AE0); \
    write_cmos_sensor_twobyte(0X6F12, 0X2000); \
    write_cmos_sensor_twobyte(0X6F12, 0X8B2C); \
    write_cmos_sensor_twobyte(0X6F12, 0X4000); \
    write_cmos_sensor_twobyte(0X6F12, 0XF000); \
    write_cmos_sensor_twobyte(0X6F12, 0X2000); \
    write_cmos_sensor_twobyte(0X6F12, 0X0A40); \
    write_cmos_sensor_twobyte(0X6F12, 0X2000); \
    write_cmos_sensor_twobyte(0X6F12, 0X5D00); \
    write_cmos_sensor_twobyte(0X6F12, 0X4000); \
    write_cmos_sensor_twobyte(0X6F12, 0X6B56); \
    write_cmos_sensor_twobyte(0X6F12, 0X2000); \
    write_cmos_sensor_twobyte(0X6F12, 0X68F0); \
    write_cmos_sensor_twobyte(0X6F12, 0X2000); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BB0); \
    write_cmos_sensor_twobyte(0X6F12, 0X4000); \
    write_cmos_sensor_twobyte(0X6F12, 0XF66E); \
    write_cmos_sensor_twobyte(0X6F12, 0X2000); \
    write_cmos_sensor_twobyte(0X6F12, 0X7220); \
    write_cmos_sensor_twobyte(0X6F12, 0X2000); \
    write_cmos_sensor_twobyte(0X6F12, 0X5900); \
    write_cmos_sensor_twobyte(0X6F12, 0X2000); \
    write_cmos_sensor_twobyte(0X6F12, 0X8720); \
    write_cmos_sensor_twobyte(0X6F12, 0X0001); \
    write_cmos_sensor_twobyte(0X6F12, 0X14E7); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X6F12, 0X5059); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X6F12, 0X512F); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X6F12, 0XFB2B); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X6F12, 0XD02D); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X6F12, 0X724F); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X6F12, 0X287F); \
    write_cmos_sensor_twobyte(0X6F12, 0X45F6); \
    write_cmos_sensor_twobyte(0X6F12, 0X576C); \
    write_cmos_sensor_twobyte(0X6F12, 0XC0F2); \
    write_cmos_sensor_twobyte(0X6F12, 0X000C); \
    write_cmos_sensor_twobyte(0X6F12, 0X6047); \
    write_cmos_sensor_twobyte(0X6F12, 0X45F2); \
    write_cmos_sensor_twobyte(0X6F12, 0X590C); \
    write_cmos_sensor_twobyte(0X6F12, 0XC0F2); \
    write_cmos_sensor_twobyte(0X6F12, 0X000C); \
    write_cmos_sensor_twobyte(0X6F12, 0X6047); \
    write_cmos_sensor_twobyte(0X6F12, 0X45F2); \
    write_cmos_sensor_twobyte(0X6F12, 0X2F1C); \
    write_cmos_sensor_twobyte(0X6F12, 0XC0F2); \
    write_cmos_sensor_twobyte(0X6F12, 0X000C); \
    write_cmos_sensor_twobyte(0X6F12, 0X6047); \
    write_cmos_sensor_twobyte(0X6F12, 0X4FF6); \
    write_cmos_sensor_twobyte(0X6F12, 0X2B3C); \
    write_cmos_sensor_twobyte(0X6F12, 0XC0F2); \
    write_cmos_sensor_twobyte(0X6F12, 0X000C); \
    write_cmos_sensor_twobyte(0X6F12, 0X6047); \
    write_cmos_sensor_twobyte(0X6F12, 0X42F6); \
    write_cmos_sensor_twobyte(0X6F12, 0X7F0C); \
    write_cmos_sensor_twobyte(0X6F12, 0XC0F2); \
    write_cmos_sensor_twobyte(0X6F12, 0X000C); \
    write_cmos_sensor_twobyte(0X6F12, 0X6047); \
    write_cmos_sensor_twobyte(0X6F12, 0X43F2); \
    write_cmos_sensor_twobyte(0X6F12, 0X7B5C); \
    write_cmos_sensor_twobyte(0X6F12, 0XC0F2); \
    write_cmos_sensor_twobyte(0X6F12, 0X000C); \
    write_cmos_sensor_twobyte(0X6F12, 0X6047); \
    /* Global */                               \
    write_cmos_sensor_twobyte(0X6028, 0X4000); \
    write_cmos_sensor_twobyte(0X0208, 0X0000); \
    write_cmos_sensor_twobyte(0XF466, 0X000C); \
    write_cmos_sensor_twobyte(0XF468, 0X000D); \
    write_cmos_sensor_twobyte(0X0200, 0X0100); \
    write_cmos_sensor_twobyte(0X0202, 0X0100); \
    write_cmos_sensor_twobyte(0X0224, 0X0100); \
    write_cmos_sensor_twobyte(0X0226, 0X0100); \
    write_cmos_sensor_twobyte(0XF488, 0X0008); \
    write_cmos_sensor_twobyte(0XF414, 0X0007); \
    write_cmos_sensor_twobyte(0XF416, 0X0004); \
    write_cmos_sensor_twobyte(0X0110, 0X1002); \
    write_cmos_sensor_twobyte(0X30C0, 0X0001); \
    write_cmos_sensor_twobyte(0X30C6, 0X0100); \
    write_cmos_sensor_twobyte(0X30CA, 0X0300); \
    write_cmos_sensor_twobyte(0X30C8, 0X05DC); \
    write_cmos_sensor_twobyte(0X6B36, 0X5200); \
    write_cmos_sensor_twobyte(0X6B38, 0X0000); \
    write_cmos_sensor_twobyte(0X0B04, 0X0101); \
    write_cmos_sensor_twobyte(0X3094, 0X2800); \
    write_cmos_sensor_twobyte(0X3096, 0X5400); \
    write_cmos_sensor_twobyte(0X6028, 0X2000); \
    write_cmos_sensor_twobyte(0X602A, 0X16B0); \
    write_cmos_sensor_twobyte(0X6F12, 0X0040); \
    write_cmos_sensor_twobyte(0X6F12, 0X0040); \
    write_cmos_sensor_twobyte(0X602A, 0X167E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0018); \
    write_cmos_sensor_twobyte(0X602A, 0X1682); \
    write_cmos_sensor_twobyte(0X6F12, 0X0010); \
    write_cmos_sensor_twobyte(0X602A, 0X16A0); \
    write_cmos_sensor_twobyte(0X6F12, 0X2101); \
    write_cmos_sensor_twobyte(0X602A, 0X1664); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X602A, 0X2A98); \
    write_cmos_sensor_twobyte(0X6F12, 0X0001); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X6F12, 0X03FF); \
    write_cmos_sensor_twobyte(0X6F12, 0X1000); \
    write_cmos_sensor_twobyte(0X6F12, 0X0FC0); \
    write_cmos_sensor_twobyte(0X602A, 0X2AC2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X6F12, 0X03FF); \
    write_cmos_sensor_twobyte(0X6F12, 0X1000); \
    write_cmos_sensor_twobyte(0X6F12, 0X0FC0); \
    write_cmos_sensor_twobyte(0X602A, 0X2AEA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X6F12, 0X03FF); \
    write_cmos_sensor_twobyte(0X6F12, 0X1000); \
    write_cmos_sensor_twobyte(0X6F12, 0X0FC0); \
    write_cmos_sensor_twobyte(0X602A, 0X2B12); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X6F12, 0X03FF); \
    write_cmos_sensor_twobyte(0X6F12, 0X1000); \
    write_cmos_sensor_twobyte(0X6F12, 0X0FC0); \
    write_cmos_sensor_twobyte(0X602A, 0X2B3A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X6F12, 0X03FF); \
    write_cmos_sensor_twobyte(0X6F12, 0X1000); \
    write_cmos_sensor_twobyte(0X6F12, 0X0FC0); \
    write_cmos_sensor_twobyte(0X602A, 0X2B62); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X6F12, 0X03FF); \
    write_cmos_sensor_twobyte(0X6F12, 0X1000); \
    write_cmos_sensor_twobyte(0X6F12, 0X0FC0); \
    write_cmos_sensor_twobyte(0X602A, 0X2B8A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X6F12, 0X03FF); \
    write_cmos_sensor_twobyte(0X6F12, 0X1000); \
    write_cmos_sensor_twobyte(0X6F12, 0X0FC0); \
    write_cmos_sensor_twobyte(0X602A, 0X2BB2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X6F12, 0X03FF); \
    write_cmos_sensor_twobyte(0X6F12, 0X1000); \
    write_cmos_sensor_twobyte(0X6F12, 0X0FC0); \
    write_cmos_sensor_twobyte(0X602A, 0X3568); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X602A, 0X402C); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X602A, 0X4AF0); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X602A, 0X35C4); \
    write_cmos_sensor_twobyte(0X6F12, 0XFF9A); \
    write_cmos_sensor_twobyte(0X602A, 0X35F6); \
    write_cmos_sensor_twobyte(0X6F12, 0XFF9A); \
    write_cmos_sensor_twobyte(0X602A, 0X35BA); \
    write_cmos_sensor_twobyte(0X6F12, 0XFF9A); \
    write_cmos_sensor_twobyte(0X602A, 0X35EC); \
    write_cmos_sensor_twobyte(0X6F12, 0XFF9A); \
    write_cmos_sensor_twobyte(0X602A, 0X35B0); \
    write_cmos_sensor_twobyte(0X6F12, 0XFF9A); \
    write_cmos_sensor_twobyte(0X602A, 0X35E2); \
    write_cmos_sensor_twobyte(0X6F12, 0XFF9A); \
    write_cmos_sensor_twobyte(0X602A, 0X35A6); \
    write_cmos_sensor_twobyte(0X6F12, 0XFF9A); \
    write_cmos_sensor_twobyte(0X602A, 0X35D8); \
    write_cmos_sensor_twobyte(0X6F12, 0XFF9A); \
    write_cmos_sensor_twobyte(0X602A, 0X4088); \
    write_cmos_sensor_twobyte(0X6F12, 0XFF9A); \
    write_cmos_sensor_twobyte(0X602A, 0X40BA); \
    write_cmos_sensor_twobyte(0X6F12, 0XFF9A); \
    write_cmos_sensor_twobyte(0X602A, 0X407E); \
    write_cmos_sensor_twobyte(0X6F12, 0XFF9A); \
    write_cmos_sensor_twobyte(0X602A, 0X40B0); \
    write_cmos_sensor_twobyte(0X6F12, 0XFF9A); \
    write_cmos_sensor_twobyte(0X602A, 0X4074); \
    write_cmos_sensor_twobyte(0X6F12, 0XFF9A); \
    write_cmos_sensor_twobyte(0X602A, 0X40A6); \
    write_cmos_sensor_twobyte(0X6F12, 0XFF9A); \
    write_cmos_sensor_twobyte(0X602A, 0X406A); \
    write_cmos_sensor_twobyte(0X6F12, 0XFF9A); \
    write_cmos_sensor_twobyte(0X602A, 0X409C); \
    write_cmos_sensor_twobyte(0X6F12, 0XFF9A); \
    write_cmos_sensor_twobyte(0X602A, 0X4B4C); \
    write_cmos_sensor_twobyte(0X6F12, 0XFFCD); \
    write_cmos_sensor_twobyte(0X602A, 0X4B7E); \
    write_cmos_sensor_twobyte(0X6F12, 0XFFCD); \
    write_cmos_sensor_twobyte(0X602A, 0X4B42); \
    write_cmos_sensor_twobyte(0X6F12, 0XFFCD); \
    write_cmos_sensor_twobyte(0X602A, 0X4B74); \
    write_cmos_sensor_twobyte(0X6F12, 0XFFCD); \
    write_cmos_sensor_twobyte(0X602A, 0X4B38); \
    write_cmos_sensor_twobyte(0X6F12, 0XFFCD); \
    write_cmos_sensor_twobyte(0X602A, 0X4B6A); \
    write_cmos_sensor_twobyte(0X6F12, 0XFFCD); \
    write_cmos_sensor_twobyte(0X602A, 0X4B2E); \
    write_cmos_sensor_twobyte(0X6F12, 0XFFCD); \
    write_cmos_sensor_twobyte(0X602A, 0X4B60); \
    write_cmos_sensor_twobyte(0X6F12, 0XFFCD); \
    write_cmos_sensor_twobyte(0X602A, 0X3684); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X602A, 0X4148); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X602A, 0X4C0C); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X602A, 0X3688); \
    write_cmos_sensor_twobyte(0X6F12, 0X000E); \
    write_cmos_sensor_twobyte(0X6F12, 0X000E); \
    write_cmos_sensor_twobyte(0X602A, 0X368E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X602A, 0X414C); \
    write_cmos_sensor_twobyte(0X6F12, 0X000E); \
    write_cmos_sensor_twobyte(0X6F12, 0X000E); \
    write_cmos_sensor_twobyte(0X602A, 0X4152); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X602A, 0X4C10); \
    write_cmos_sensor_twobyte(0X6F12, 0X000E); \
    write_cmos_sensor_twobyte(0X6F12, 0X000E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0002); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X602A, 0X0B52); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X6F12, 0X0001); \
    write_cmos_sensor_twobyte(0X602A, 0X1686); \
    write_cmos_sensor_twobyte(0X6F12, 0X0328); \
    write_cmos_sensor_twobyte(0X602A, 0X0B02); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X602A, 0X4C54); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X602A, 0X4C58); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X602A, 0X4C5C); \
    write_cmos_sensor_twobyte(0X6F12, 0X010F); \
    write_cmos_sensor_twobyte(0X602A, 0X09A2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0080); \
    write_cmos_sensor_twobyte(0X602A, 0X09A6); \
    write_cmos_sensor_twobyte(0X6F12, 0X008C); \
    write_cmos_sensor_twobyte(0X602A, 0X09AE); \
    write_cmos_sensor_twobyte(0X6F12, 0XF408); \
    write_cmos_sensor_twobyte(0X602A, 0X09A0); \
    write_cmos_sensor_twobyte(0X6F12, 0X0101); \
    write_cmos_sensor_twobyte(0X602A, 0X09A4); \
    write_cmos_sensor_twobyte(0X6F12, 0X007F); \
    write_cmos_sensor_twobyte(0X602A, 0X09A8); \
    write_cmos_sensor_twobyte(0X6F12, 0X007F); \
    write_cmos_sensor_twobyte(0X602A, 0X09DA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0009); \
    write_cmos_sensor_twobyte(0X6F12, 0X0012); \
    write_cmos_sensor_twobyte(0X6F12, 0XF482); \
    write_cmos_sensor_twobyte(0X6F12, 0X0009); \
    write_cmos_sensor_twobyte(0X6F12, 0X0012); \
    write_cmos_sensor_twobyte(0X6F12, 0XF484); \
    write_cmos_sensor_twobyte(0X602A, 0X1666); \
    write_cmos_sensor_twobyte(0X6F12, 0X054C); \
    write_cmos_sensor_twobyte(0X6F12, 0X0183); \
    write_cmos_sensor_twobyte(0X6F12, 0X07FF); \
    write_cmos_sensor_twobyte(0X602A, 0X16B4); \
    write_cmos_sensor_twobyte(0X6F12, 0X54C2); \
    write_cmos_sensor_twobyte(0X602A, 0X16AA); \
    write_cmos_sensor_twobyte(0X6F12, 0X2608); \
    write_cmos_sensor_twobyte(0X602A, 0X16AE); \
    write_cmos_sensor_twobyte(0X6F12, 0X2608); \
    write_cmos_sensor_twobyte(0X602A, 0X1620); \
    write_cmos_sensor_twobyte(0X6F12, 0X0408); \
    write_cmos_sensor_twobyte(0X6F12, 0X0808); \
    write_cmos_sensor_twobyte(0X602A, 0X1628); \
    write_cmos_sensor_twobyte(0X6F12, 0X0408); \
    write_cmos_sensor_twobyte(0X6F12, 0X0808); \
    write_cmos_sensor_twobyte(0X602A, 0X168A); \
    write_cmos_sensor_twobyte(0X6F12, 0X001B); \
    write_cmos_sensor_twobyte(0X602A, 0X1640); \
    write_cmos_sensor_twobyte(0X6F12, 0X0505); \
    write_cmos_sensor_twobyte(0X6F12, 0X070E); \
    write_cmos_sensor_twobyte(0X602A, 0X1648); \
    write_cmos_sensor_twobyte(0X6F12, 0X0505); \
    write_cmos_sensor_twobyte(0X6F12, 0X070E); \
    write_cmos_sensor_twobyte(0X602A, 0X1650); \
    write_cmos_sensor_twobyte(0X6F12, 0X0A0A); \
    write_cmos_sensor_twobyte(0X6F12, 0X040A); \
    write_cmos_sensor_twobyte(0X602A, 0X1658); \
    write_cmos_sensor_twobyte(0X6F12, 0X0A0A); \
    write_cmos_sensor_twobyte(0X6F12, 0X040A); \
    write_cmos_sensor_twobyte(0X602A, 0X15E4); \
    write_cmos_sensor_twobyte(0X6F12, 0X0001); \
    write_cmos_sensor_twobyte(0X6F12, 0X0101); \
    write_cmos_sensor_twobyte(0X602A, 0X15EA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0302); \
    write_cmos_sensor_twobyte(0X602A, 0X0BB8); \
    write_cmos_sensor_twobyte(0X6F12, 0X01A0); \
    write_cmos_sensor_twobyte(0X6F12, 0X01A0); \
    write_cmos_sensor_twobyte(0X602A, 0X0BBE); \
    write_cmos_sensor_twobyte(0X6F12, 0X01A0); \
    write_cmos_sensor_twobyte(0X602A, 0X0BC8); \
    write_cmos_sensor_twobyte(0X6F12, 0X06E0); \
    write_cmos_sensor_twobyte(0X6F12, 0X0420); \
    write_cmos_sensor_twobyte(0X602A, 0X0BCE); \
    write_cmos_sensor_twobyte(0X6F12, 0X0420); \
    write_cmos_sensor_twobyte(0X602A, 0X0BD8); \
    write_cmos_sensor_twobyte(0X6F12, 0X000A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0006); \
    write_cmos_sensor_twobyte(0X602A, 0X0BDE); \
    write_cmos_sensor_twobyte(0X6F12, 0X0003); \
    write_cmos_sensor_twobyte(0X602A, 0X16BC); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X602A, 0X0A10); \
    write_cmos_sensor_twobyte(0X6F12, 0X0064); \
    write_cmos_sensor_twobyte(0X6F12, 0X0520); \
    write_cmos_sensor_twobyte(0X6F12, 0X06E0); \
    write_cmos_sensor_twobyte(0X6F12, 0X06DE); \
    write_cmos_sensor_twobyte(0X602A, 0X15FE); \
    write_cmos_sensor_twobyte(0X6F12, 0X0001); \
    write_cmos_sensor_twobyte(0X602A, 0X0C10); \
    write_cmos_sensor_twobyte(0X6F12, 0X0361); \
    write_cmos_sensor_twobyte(0X6F12, 0X028A); \
    write_cmos_sensor_twobyte(0X602A, 0X0C16); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D8); \
    write_cmos_sensor_twobyte(0X602A, 0X0C28); \
    write_cmos_sensor_twobyte(0X6F12, 0X019D); \
    write_cmos_sensor_twobyte(0X6F12, 0X0114); \
    write_cmos_sensor_twobyte(0X602A, 0X0C2E); \
    write_cmos_sensor_twobyte(0X6F12, 0X00BF); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D5); \
    write_cmos_sensor_twobyte(0X6F12, 0X014C); \
    write_cmos_sensor_twobyte(0X602A, 0X0C36); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F7); \
    write_cmos_sensor_twobyte(0X6F12, 0X019C); \
    write_cmos_sensor_twobyte(0X6F12, 0X0113); \
    write_cmos_sensor_twobyte(0X602A, 0X0C3E); \
    write_cmos_sensor_twobyte(0X6F12, 0X00BE); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D6); \
    write_cmos_sensor_twobyte(0X6F12, 0X014D); \
    write_cmos_sensor_twobyte(0X602A, 0X0C46); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F8); \
    write_cmos_sensor_twobyte(0X6F12, 0X0094); \
    write_cmos_sensor_twobyte(0X6F12, 0X0072); \
    write_cmos_sensor_twobyte(0X602A, 0X0C4E); \
    write_cmos_sensor_twobyte(0X6F12, 0X010F); \
    write_cmos_sensor_twobyte(0X6F12, 0X007F); \
    write_cmos_sensor_twobyte(0X6F12, 0X005D); \
    write_cmos_sensor_twobyte(0X602A, 0X0C56); \
    write_cmos_sensor_twobyte(0X6F12, 0X00FA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0069); \
    write_cmos_sensor_twobyte(0X6F12, 0X005B); \
    write_cmos_sensor_twobyte(0X602A, 0X0C5E); \
    write_cmos_sensor_twobyte(0X6F12, 0X005B); \
    write_cmos_sensor_twobyte(0X602A, 0X0C68); \
    write_cmos_sensor_twobyte(0X6F12, 0X008D); \
    write_cmos_sensor_twobyte(0X6F12, 0X006B); \
    write_cmos_sensor_twobyte(0X602A, 0X0C6E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0108); \
    write_cmos_sensor_twobyte(0X6F12, 0X0086); \
    write_cmos_sensor_twobyte(0X6F12, 0X0064); \
    write_cmos_sensor_twobyte(0X602A, 0X0C76); \
    write_cmos_sensor_twobyte(0X6F12, 0X0101); \
    write_cmos_sensor_twobyte(0X6F12, 0X0062); \
    write_cmos_sensor_twobyte(0X6F12, 0X0054); \
    write_cmos_sensor_twobyte(0X602A, 0X0C7E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0054); \
    write_cmos_sensor_twobyte(0X602A, 0X0C88); \
    write_cmos_sensor_twobyte(0X6F12, 0X0063); \
    write_cmos_sensor_twobyte(0X6F12, 0X0055); \
    write_cmos_sensor_twobyte(0X602A, 0X0C8E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0055); \
    write_cmos_sensor_twobyte(0X602A, 0X0C98); \
    write_cmos_sensor_twobyte(0X6F12, 0X008D); \
    write_cmos_sensor_twobyte(0X6F12, 0X006B); \
    write_cmos_sensor_twobyte(0X602A, 0X0C9E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0108); \
    write_cmos_sensor_twobyte(0X6F12, 0X0086); \
    write_cmos_sensor_twobyte(0X6F12, 0X0064); \
    write_cmos_sensor_twobyte(0X602A, 0X0CA6); \
    write_cmos_sensor_twobyte(0X6F12, 0X0101); \
    write_cmos_sensor_twobyte(0X6F12, 0X0062); \
    write_cmos_sensor_twobyte(0X6F12, 0X0054); \
    write_cmos_sensor_twobyte(0X602A, 0X0CAE); \
    write_cmos_sensor_twobyte(0X6F12, 0X0054); \
    write_cmos_sensor_twobyte(0X602A, 0X0CC0); \
    write_cmos_sensor_twobyte(0X6F12, 0X019B); \
    write_cmos_sensor_twobyte(0X6F12, 0X0112); \
    write_cmos_sensor_twobyte(0X602A, 0X0CC6); \
    write_cmos_sensor_twobyte(0X6F12, 0X00BD); \
    write_cmos_sensor_twobyte(0X6F12, 0X0355); \
    write_cmos_sensor_twobyte(0X6F12, 0X0282); \
    write_cmos_sensor_twobyte(0X602A, 0X0CCE); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D5); \
    write_cmos_sensor_twobyte(0X6F12, 0X019D); \
    write_cmos_sensor_twobyte(0X6F12, 0X0114); \
    write_cmos_sensor_twobyte(0X602A, 0X0CD6); \
    write_cmos_sensor_twobyte(0X6F12, 0X00BF); \
    write_cmos_sensor_twobyte(0X6F12, 0X0353); \
    write_cmos_sensor_twobyte(0X6F12, 0X0280); \
    write_cmos_sensor_twobyte(0X602A, 0X0CDE); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D3); \
    write_cmos_sensor_twobyte(0X6F12, 0X019B); \
    write_cmos_sensor_twobyte(0X6F12, 0X0112); \
    write_cmos_sensor_twobyte(0X602A, 0X0CE6); \
    write_cmos_sensor_twobyte(0X6F12, 0X00BD); \
    write_cmos_sensor_twobyte(0X6F12, 0X01F3); \
    write_cmos_sensor_twobyte(0X6F12, 0X016A); \
    write_cmos_sensor_twobyte(0X602A, 0X0CEE); \
    write_cmos_sensor_twobyte(0X6F12, 0X0113); \
    write_cmos_sensor_twobyte(0X602A, 0X0CF8); \
    write_cmos_sensor_twobyte(0X6F12, 0X00DF); \
    write_cmos_sensor_twobyte(0X6F12, 0X005A); \
    write_cmos_sensor_twobyte(0X602A, 0X0D08); \
    write_cmos_sensor_twobyte(0X6F12, 0X00D6); \
    write_cmos_sensor_twobyte(0X6F12, 0X0051); \
    write_cmos_sensor_twobyte(0X602A, 0X0D18); \
    write_cmos_sensor_twobyte(0X6F12, 0X00DD); \
    write_cmos_sensor_twobyte(0X6F12, 0X0058); \
    write_cmos_sensor_twobyte(0X602A, 0X0D20); \
    write_cmos_sensor_twobyte(0X6F12, 0X0018); \
    write_cmos_sensor_twobyte(0X6F12, 0X0018); \
    write_cmos_sensor_twobyte(0X602A, 0X0D26); \
    write_cmos_sensor_twobyte(0X6F12, 0X0018); \
    write_cmos_sensor_twobyte(0X6F12, 0X0193); \
    write_cmos_sensor_twobyte(0X6F12, 0X010A); \
    write_cmos_sensor_twobyte(0X602A, 0X0D2E); \
    write_cmos_sensor_twobyte(0X6F12, 0X00BA); \
    write_cmos_sensor_twobyte(0X602A, 0X0D38); \
    write_cmos_sensor_twobyte(0X6F12, 0X0012); \
    write_cmos_sensor_twobyte(0X6F12, 0X0012); \
    write_cmos_sensor_twobyte(0X602A, 0X0D3E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0010); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D5); \
    write_cmos_sensor_twobyte(0X6F12, 0X014C); \
    write_cmos_sensor_twobyte(0X602A, 0X0D46); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F7); \
    write_cmos_sensor_twobyte(0X6F12, 0X01EB); \
    write_cmos_sensor_twobyte(0X6F12, 0X0162); \
    write_cmos_sensor_twobyte(0X602A, 0X0D4E); \
    write_cmos_sensor_twobyte(0X6F12, 0X010B); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X602A, 0X0D56); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X602A, 0X0D5E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X602A, 0X0D68); \
    write_cmos_sensor_twobyte(0X6F12, 0X0361); \
    write_cmos_sensor_twobyte(0X6F12, 0X028A); \
    write_cmos_sensor_twobyte(0X602A, 0X0D6E); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D8); \
    write_cmos_sensor_twobyte(0X6F12, 0X0012); \
    write_cmos_sensor_twobyte(0X6F12, 0X0012); \
    write_cmos_sensor_twobyte(0X602A, 0X0D76); \
    write_cmos_sensor_twobyte(0X6F12, 0X0012); \
    write_cmos_sensor_twobyte(0X602A, 0X0D88); \
    write_cmos_sensor_twobyte(0X6F12, 0X0361); \
    write_cmos_sensor_twobyte(0X6F12, 0X028A); \
    write_cmos_sensor_twobyte(0X602A, 0X0D8E); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D8); \
    write_cmos_sensor_twobyte(0X602A, 0X0D98); \
    write_cmos_sensor_twobyte(0X6F12, 0X0361); \
    write_cmos_sensor_twobyte(0X6F12, 0X028A); \
    write_cmos_sensor_twobyte(0X602A, 0X0D9E); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D8); \
    write_cmos_sensor_twobyte(0X602A, 0X0DA8); \
    write_cmos_sensor_twobyte(0X6F12, 0X0355); \
    write_cmos_sensor_twobyte(0X6F12, 0X0282); \
    write_cmos_sensor_twobyte(0X602A, 0X0DAE); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D5); \
    write_cmos_sensor_twobyte(0X602A, 0X0DB8); \
    write_cmos_sensor_twobyte(0X6F12, 0X0355); \
    write_cmos_sensor_twobyte(0X6F12, 0X0282); \
    write_cmos_sensor_twobyte(0X602A, 0X0DBE); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D5); \
    write_cmos_sensor_twobyte(0X6F12, 0X015F); \
    write_cmos_sensor_twobyte(0X6F12, 0X00DA); \
    write_cmos_sensor_twobyte(0X602A, 0X0DC6); \
    write_cmos_sensor_twobyte(0X6F12, 0X008D); \
    write_cmos_sensor_twobyte(0X6F12, 0X0193); \
    write_cmos_sensor_twobyte(0X6F12, 0X010A); \
    write_cmos_sensor_twobyte(0X602A, 0X0DCE); \
    write_cmos_sensor_twobyte(0X6F12, 0X00BA); \
    write_cmos_sensor_twobyte(0X6F12, 0X026D); \
    write_cmos_sensor_twobyte(0X6F12, 0X01E4); \
    write_cmos_sensor_twobyte(0X602A, 0X0DD6); \
    write_cmos_sensor_twobyte(0X6F12, 0X013A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0351); \
    write_cmos_sensor_twobyte(0X6F12, 0X027E); \
    write_cmos_sensor_twobyte(0X602A, 0X0DDE); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D1); \
    write_cmos_sensor_twobyte(0X6F12, 0X019B); \
    write_cmos_sensor_twobyte(0X6F12, 0X0112); \
    write_cmos_sensor_twobyte(0X602A, 0X0DE6); \
    write_cmos_sensor_twobyte(0X6F12, 0X00C2); \
    write_cmos_sensor_twobyte(0X6F12, 0X01AD); \
    write_cmos_sensor_twobyte(0X6F12, 0X0124); \
    write_cmos_sensor_twobyte(0X602A, 0X0DEE); \
    write_cmos_sensor_twobyte(0X6F12, 0X00D4); \
    write_cmos_sensor_twobyte(0X6F12, 0X01A0); \
    write_cmos_sensor_twobyte(0X6F12, 0X0117); \
    write_cmos_sensor_twobyte(0X602A, 0X0DF6); \
    write_cmos_sensor_twobyte(0X6F12, 0X00C7); \
    write_cmos_sensor_twobyte(0X6F12, 0X01B2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0129); \
    write_cmos_sensor_twobyte(0X602A, 0X0DFE); \
    write_cmos_sensor_twobyte(0X6F12, 0X00D9); \
    write_cmos_sensor_twobyte(0X6F12, 0X01A5); \
    write_cmos_sensor_twobyte(0X6F12, 0X011C); \
    write_cmos_sensor_twobyte(0X602A, 0X0E06); \
    write_cmos_sensor_twobyte(0X6F12, 0X00CC); \
    write_cmos_sensor_twobyte(0X6F12, 0X01B2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0129); \
    write_cmos_sensor_twobyte(0X602A, 0X0E0E); \
    write_cmos_sensor_twobyte(0X6F12, 0X00D9); \
    write_cmos_sensor_twobyte(0X6F12, 0X019B); \
    write_cmos_sensor_twobyte(0X6F12, 0X0112); \
    write_cmos_sensor_twobyte(0X602A, 0X0E16); \
    write_cmos_sensor_twobyte(0X6F12, 0X00C2); \
    write_cmos_sensor_twobyte(0X6F12, 0X019E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0115); \
    write_cmos_sensor_twobyte(0X602A, 0X0E1E); \
    write_cmos_sensor_twobyte(0X6F12, 0X00C5); \
    write_cmos_sensor_twobyte(0X602A, 0X0E30); \
    write_cmos_sensor_twobyte(0X6F12, 0X01A3); \
    write_cmos_sensor_twobyte(0X6F12, 0X011A); \
    write_cmos_sensor_twobyte(0X602A, 0X0E36); \
    write_cmos_sensor_twobyte(0X6F12, 0X00CA); \
    write_cmos_sensor_twobyte(0X6F12, 0X01B2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0129); \
    write_cmos_sensor_twobyte(0X602A, 0X0E3E); \
    write_cmos_sensor_twobyte(0X6F12, 0X00D9); \
    write_cmos_sensor_twobyte(0X602A, 0X0E50); \
    write_cmos_sensor_twobyte(0X6F12, 0X019B); \
    write_cmos_sensor_twobyte(0X6F12, 0X0112); \
    write_cmos_sensor_twobyte(0X602A, 0X0E56); \
    write_cmos_sensor_twobyte(0X6F12, 0X00C2); \
    write_cmos_sensor_twobyte(0X6F12, 0X019E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0115); \
    write_cmos_sensor_twobyte(0X602A, 0X0E5E); \
    write_cmos_sensor_twobyte(0X6F12, 0X00C5); \
    write_cmos_sensor_twobyte(0X6F12, 0X00FC); \
    write_cmos_sensor_twobyte(0X6F12, 0X0077); \
    write_cmos_sensor_twobyte(0X602A, 0X0E66); \
    write_cmos_sensor_twobyte(0X6F12, 0X0074); \
    write_cmos_sensor_twobyte(0X6F12, 0X0196); \
    write_cmos_sensor_twobyte(0X6F12, 0X010D); \
    write_cmos_sensor_twobyte(0X602A, 0X0E6E); \
    write_cmos_sensor_twobyte(0X6F12, 0X00BD); \
    write_cmos_sensor_twobyte(0X6F12, 0X026B); \
    write_cmos_sensor_twobyte(0X6F12, 0X01E2); \
    write_cmos_sensor_twobyte(0X602A, 0X0E76); \
    write_cmos_sensor_twobyte(0X6F12, 0X0138); \
    write_cmos_sensor_twobyte(0X6F12, 0X0353); \
    write_cmos_sensor_twobyte(0X6F12, 0X0280); \
    write_cmos_sensor_twobyte(0X602A, 0X0E7E); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D2); \
    write_cmos_sensor_twobyte(0X602A, 0X0E90); \
    write_cmos_sensor_twobyte(0X6F12, 0X019A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0111); \
    write_cmos_sensor_twobyte(0X602A, 0X0E96); \
    write_cmos_sensor_twobyte(0X6F12, 0X00C1); \
    write_cmos_sensor_twobyte(0X6F12, 0X01B0); \
    write_cmos_sensor_twobyte(0X6F12, 0X0127); \
    write_cmos_sensor_twobyte(0X602A, 0X0E9E); \
    write_cmos_sensor_twobyte(0X6F12, 0X00D6); \
    write_cmos_sensor_twobyte(0X6F12, 0X01B2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0129); \
    write_cmos_sensor_twobyte(0X602A, 0X0EA6); \
    write_cmos_sensor_twobyte(0X6F12, 0X00D8); \
    write_cmos_sensor_twobyte(0X6F12, 0X01C8); \
    write_cmos_sensor_twobyte(0X6F12, 0X013F); \
    write_cmos_sensor_twobyte(0X602A, 0X0EAE); \
    write_cmos_sensor_twobyte(0X6F12, 0X00ED); \
    write_cmos_sensor_twobyte(0X6F12, 0X01CA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0141); \
    write_cmos_sensor_twobyte(0X602A, 0X0EB6); \
    write_cmos_sensor_twobyte(0X6F12, 0X00EF); \
    write_cmos_sensor_twobyte(0X6F12, 0X01E0); \
    write_cmos_sensor_twobyte(0X6F12, 0X0157); \
    write_cmos_sensor_twobyte(0X602A, 0X0EBE); \
    write_cmos_sensor_twobyte(0X6F12, 0X0104); \
    write_cmos_sensor_twobyte(0X6F12, 0X01E3); \
    write_cmos_sensor_twobyte(0X6F12, 0X015A); \
    write_cmos_sensor_twobyte(0X602A, 0X0EC6); \
    write_cmos_sensor_twobyte(0X6F12, 0X0107); \
    write_cmos_sensor_twobyte(0X6F12, 0X01F9); \
    write_cmos_sensor_twobyte(0X6F12, 0X0170); \
    write_cmos_sensor_twobyte(0X602A, 0X0ECE); \
    write_cmos_sensor_twobyte(0X6F12, 0X011C); \
    write_cmos_sensor_twobyte(0X6F12, 0X01FB); \
    write_cmos_sensor_twobyte(0X6F12, 0X0172); \
    write_cmos_sensor_twobyte(0X602A, 0X0ED6); \
    write_cmos_sensor_twobyte(0X6F12, 0X011E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0211); \
    write_cmos_sensor_twobyte(0X6F12, 0X0188); \
    write_cmos_sensor_twobyte(0X602A, 0X0EDE); \
    write_cmos_sensor_twobyte(0X6F12, 0X0133); \
    write_cmos_sensor_twobyte(0X6F12, 0X0213); \
    write_cmos_sensor_twobyte(0X6F12, 0X018A); \
    write_cmos_sensor_twobyte(0X602A, 0X0EE6); \
    write_cmos_sensor_twobyte(0X6F12, 0X0135); \
    write_cmos_sensor_twobyte(0X6F12, 0X0229); \
    write_cmos_sensor_twobyte(0X6F12, 0X01A0); \
    write_cmos_sensor_twobyte(0X602A, 0X0EEE); \
    write_cmos_sensor_twobyte(0X6F12, 0X014A); \
    write_cmos_sensor_twobyte(0X6F12, 0X022B); \
    write_cmos_sensor_twobyte(0X6F12, 0X01A2); \
    write_cmos_sensor_twobyte(0X602A, 0X0EF6); \
    write_cmos_sensor_twobyte(0X6F12, 0X014C); \
    write_cmos_sensor_twobyte(0X6F12, 0X0241); \
    write_cmos_sensor_twobyte(0X6F12, 0X01B8); \
    write_cmos_sensor_twobyte(0X602A, 0X0EFE); \
    write_cmos_sensor_twobyte(0X6F12, 0X0161); \
    write_cmos_sensor_twobyte(0X6F12, 0X0243); \
    write_cmos_sensor_twobyte(0X6F12, 0X01BA); \
    write_cmos_sensor_twobyte(0X602A, 0X0F06); \
    write_cmos_sensor_twobyte(0X6F12, 0X0163); \
    write_cmos_sensor_twobyte(0X6F12, 0X0259); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D0); \
    write_cmos_sensor_twobyte(0X602A, 0X0F0E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0178); \
    write_cmos_sensor_twobyte(0X6F12, 0X030E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0237); \
    write_cmos_sensor_twobyte(0X602A, 0X0F16); \
    write_cmos_sensor_twobyte(0X6F12, 0X018B); \
    write_cmos_sensor_twobyte(0X6F12, 0X0324); \
    write_cmos_sensor_twobyte(0X6F12, 0X024D); \
    write_cmos_sensor_twobyte(0X602A, 0X0F1E); \
    write_cmos_sensor_twobyte(0X6F12, 0X01A0); \
    write_cmos_sensor_twobyte(0X6F12, 0X0326); \
    write_cmos_sensor_twobyte(0X6F12, 0X024F); \
    write_cmos_sensor_twobyte(0X602A, 0X0F26); \
    write_cmos_sensor_twobyte(0X6F12, 0X01A2); \
    write_cmos_sensor_twobyte(0X6F12, 0X033C); \
    write_cmos_sensor_twobyte(0X6F12, 0X0265); \
    write_cmos_sensor_twobyte(0X602A, 0X0F2E); \
    write_cmos_sensor_twobyte(0X6F12, 0X01B7); \
    write_cmos_sensor_twobyte(0X6F12, 0X033E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0267); \
    write_cmos_sensor_twobyte(0X602A, 0X0F36); \
    write_cmos_sensor_twobyte(0X6F12, 0X01B9); \
    write_cmos_sensor_twobyte(0X6F12, 0X0354); \
    write_cmos_sensor_twobyte(0X6F12, 0X027D); \
    write_cmos_sensor_twobyte(0X602A, 0X0F3E); \
    write_cmos_sensor_twobyte(0X6F12, 0X01CE); \
    write_cmos_sensor_twobyte(0X602A, 0X0FB0); \
    write_cmos_sensor_twobyte(0X6F12, 0X019B); \
    write_cmos_sensor_twobyte(0X6F12, 0X0112); \
    write_cmos_sensor_twobyte(0X602A, 0X0FB6); \
    write_cmos_sensor_twobyte(0X6F12, 0X00C2); \
    write_cmos_sensor_twobyte(0X6F12, 0X01B2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0129); \
    write_cmos_sensor_twobyte(0X602A, 0X0FBE); \
    write_cmos_sensor_twobyte(0X6F12, 0X00D9); \
    write_cmos_sensor_twobyte(0X602A, 0X0FD0); \
    write_cmos_sensor_twobyte(0X6F12, 0X019B); \
    write_cmos_sensor_twobyte(0X6F12, 0X0112); \
    write_cmos_sensor_twobyte(0X602A, 0X0FD6); \
    write_cmos_sensor_twobyte(0X6F12, 0X00C2); \
    write_cmos_sensor_twobyte(0X6F12, 0X019E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0115); \
    write_cmos_sensor_twobyte(0X602A, 0X0FDE); \
    write_cmos_sensor_twobyte(0X6F12, 0X00C5); \
    write_cmos_sensor_twobyte(0X6F12, 0X0012); \
    write_cmos_sensor_twobyte(0X6F12, 0X0012); \
    write_cmos_sensor_twobyte(0X602A, 0X0FE6); \
    write_cmos_sensor_twobyte(0X6F12, 0X0012); \
    write_cmos_sensor_twobyte(0X6F12, 0X0357); \
    write_cmos_sensor_twobyte(0X6F12, 0X0284); \
    write_cmos_sensor_twobyte(0X602A, 0X0FEE); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D7); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F7); \
    write_cmos_sensor_twobyte(0X6F12, 0X0072); \
    write_cmos_sensor_twobyte(0X602A, 0X0FF6); \
    write_cmos_sensor_twobyte(0X6F12, 0X006F); \
    write_cmos_sensor_twobyte(0X602A, 0X1008); \
    write_cmos_sensor_twobyte(0X6F12, 0X00FA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0075); \
    write_cmos_sensor_twobyte(0X602A, 0X100E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0072); \
    write_cmos_sensor_twobyte(0X6F12, 0X019A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0111); \
    write_cmos_sensor_twobyte(0X602A, 0X1016); \
    write_cmos_sensor_twobyte(0X6F12, 0X00C1); \
    write_cmos_sensor_twobyte(0X6F12, 0X019D); \
    write_cmos_sensor_twobyte(0X6F12, 0X0114); \
    write_cmos_sensor_twobyte(0X602A, 0X101E); \
    write_cmos_sensor_twobyte(0X6F12, 0X00C4); \
    write_cmos_sensor_twobyte(0X6F12, 0X0352); \
    write_cmos_sensor_twobyte(0X6F12, 0X027F); \
    write_cmos_sensor_twobyte(0X602A, 0X1026); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0355); \
    write_cmos_sensor_twobyte(0X6F12, 0X0282); \
    write_cmos_sensor_twobyte(0X602A, 0X102E); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D5); \
    write_cmos_sensor_twobyte(0X6F12, 0X019A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0111); \
    write_cmos_sensor_twobyte(0X602A, 0X1036); \
    write_cmos_sensor_twobyte(0X6F12, 0X00C1); \
    write_cmos_sensor_twobyte(0X6F12, 0X0012); \
    write_cmos_sensor_twobyte(0X6F12, 0X0012); \
    write_cmos_sensor_twobyte(0X602A, 0X103E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0012); \
    write_cmos_sensor_twobyte(0X6F12, 0X019F); \
    write_cmos_sensor_twobyte(0X6F12, 0X0116); \
    write_cmos_sensor_twobyte(0X602A, 0X1046); \
    write_cmos_sensor_twobyte(0X6F12, 0X00C6); \
    write_cmos_sensor_twobyte(0X602A, 0X1058); \
    write_cmos_sensor_twobyte(0X6F12, 0X0013); \
    write_cmos_sensor_twobyte(0X6F12, 0X0013); \
    write_cmos_sensor_twobyte(0X602A, 0X105E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0013); \
    write_cmos_sensor_twobyte(0X6F12, 0X0353); \
    write_cmos_sensor_twobyte(0X6F12, 0X0280); \
    write_cmos_sensor_twobyte(0X602A, 0X1066); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D3); \
    write_cmos_sensor_twobyte(0X602A, 0X1098); \
    write_cmos_sensor_twobyte(0X6F12, 0X01BC); \
    write_cmos_sensor_twobyte(0X6F12, 0X0133); \
    write_cmos_sensor_twobyte(0X602A, 0X109E); \
    write_cmos_sensor_twobyte(0X6F12, 0X00E3); \
    write_cmos_sensor_twobyte(0X6F12, 0X01C6); \
    write_cmos_sensor_twobyte(0X6F12, 0X013D); \
    write_cmos_sensor_twobyte(0X602A, 0X10A6); \
    write_cmos_sensor_twobyte(0X6F12, 0X00ED); \
    write_cmos_sensor_twobyte(0X602A, 0X10D8); \
    write_cmos_sensor_twobyte(0X6F12, 0X019B); \
    write_cmos_sensor_twobyte(0X6F12, 0X0112); \
    write_cmos_sensor_twobyte(0X602A, 0X10DE); \
    write_cmos_sensor_twobyte(0X6F12, 0X00C2); \
    write_cmos_sensor_twobyte(0X6F12, 0X01AD); \
    write_cmos_sensor_twobyte(0X6F12, 0X0124); \
    write_cmos_sensor_twobyte(0X602A, 0X10E6); \
    write_cmos_sensor_twobyte(0X6F12, 0X00D4); \
    write_cmos_sensor_twobyte(0X602A, 0X1118); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D3); \
    write_cmos_sensor_twobyte(0X6F12, 0X014A); \
    write_cmos_sensor_twobyte(0X602A, 0X111E); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F5); \
    write_cmos_sensor_twobyte(0X602A, 0X1158); \
    write_cmos_sensor_twobyte(0X6F12, 0X0352); \
    write_cmos_sensor_twobyte(0X6F12, 0X027F); \
    write_cmos_sensor_twobyte(0X602A, 0X115E); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D2); \
    write_cmos_sensor_twobyte(0X602A, 0X1198); \
    write_cmos_sensor_twobyte(0X6F12, 0X01A0); \
    write_cmos_sensor_twobyte(0X6F12, 0X0117); \
    write_cmos_sensor_twobyte(0X602A, 0X119E); \
    write_cmos_sensor_twobyte(0X6F12, 0X00C7); \
    write_cmos_sensor_twobyte(0X6F12, 0X01B2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0129); \
    write_cmos_sensor_twobyte(0X602A, 0X11A6); \
    write_cmos_sensor_twobyte(0X6F12, 0X00D9); \
    write_cmos_sensor_twobyte(0X602A, 0X11B8); \
    write_cmos_sensor_twobyte(0X6F12, 0X0014); \
    write_cmos_sensor_twobyte(0X6F12, 0X0014); \
    write_cmos_sensor_twobyte(0X602A, 0X11BE); \
    write_cmos_sensor_twobyte(0X6F12, 0X0014); \
    write_cmos_sensor_twobyte(0X6F12, 0X0198); \
    write_cmos_sensor_twobyte(0X6F12, 0X010F); \
    write_cmos_sensor_twobyte(0X602A, 0X11C6); \
    write_cmos_sensor_twobyte(0X6F12, 0X00BF); \
    write_cmos_sensor_twobyte(0X6F12, 0X0275); \
    write_cmos_sensor_twobyte(0X6F12, 0X01EC); \
    write_cmos_sensor_twobyte(0X602A, 0X11CE); \
    write_cmos_sensor_twobyte(0X6F12, 0X0142); \
    write_cmos_sensor_twobyte(0X6F12, 0X02C1); \
    write_cmos_sensor_twobyte(0X602A, 0X11D6); \
    write_cmos_sensor_twobyte(0X6F12, 0X018A); \
    write_cmos_sensor_twobyte(0X602A, 0X1258); \
    write_cmos_sensor_twobyte(0X6F12, 0X000B); \
    write_cmos_sensor_twobyte(0X6F12, 0X000B); \
    write_cmos_sensor_twobyte(0X602A, 0X125E); \
    write_cmos_sensor_twobyte(0X6F12, 0X000B); \
    write_cmos_sensor_twobyte(0X602A, 0X1268); \
    write_cmos_sensor_twobyte(0X6F12, 0X0009); \
    write_cmos_sensor_twobyte(0X6F12, 0X0009); \
    write_cmos_sensor_twobyte(0X602A, 0X126E); \
    write_cmos_sensor_twobyte(0X6F12, 0X00C1); \
    write_cmos_sensor_twobyte(0X6F12, 0X0013); \
    write_cmos_sensor_twobyte(0X6F12, 0X0013); \
    write_cmos_sensor_twobyte(0X602A, 0X1276); \
    write_cmos_sensor_twobyte(0X6F12, 0X00CB); \
    write_cmos_sensor_twobyte(0X6F12, 0X01B5); \
    write_cmos_sensor_twobyte(0X602A, 0X127E); \
    write_cmos_sensor_twobyte(0X6F12, 0X00D5); \
    write_cmos_sensor_twobyte(0X6F12, 0X01BF); \
    write_cmos_sensor_twobyte(0X602A, 0X1286); \
    write_cmos_sensor_twobyte(0X6F12, 0X00DF); \
    write_cmos_sensor_twobyte(0X6F12, 0X01C9); \
    write_cmos_sensor_twobyte(0X602A, 0X128E); \
    write_cmos_sensor_twobyte(0X6F12, 0X00E9); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D3); \
    write_cmos_sensor_twobyte(0X602A, 0X1296); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F3); \
    write_cmos_sensor_twobyte(0X6F12, 0X01DD); \
    write_cmos_sensor_twobyte(0X602A, 0X129E); \
    write_cmos_sensor_twobyte(0X6F12, 0X00FD); \
    write_cmos_sensor_twobyte(0X6F12, 0X01E7); \
    write_cmos_sensor_twobyte(0X602A, 0X12A6); \
    write_cmos_sensor_twobyte(0X6F12, 0X0107); \
    write_cmos_sensor_twobyte(0X6F12, 0X01F1); \
    write_cmos_sensor_twobyte(0X602A, 0X12AE); \
    write_cmos_sensor_twobyte(0X6F12, 0X0111); \
    write_cmos_sensor_twobyte(0X6F12, 0X01FB); \
    write_cmos_sensor_twobyte(0X602A, 0X12B6); \
    write_cmos_sensor_twobyte(0X6F12, 0X011B); \
    write_cmos_sensor_twobyte(0X6F12, 0X0205); \
    write_cmos_sensor_twobyte(0X602A, 0X12BE); \
    write_cmos_sensor_twobyte(0X6F12, 0X0125); \
    write_cmos_sensor_twobyte(0X6F12, 0X020F); \
    write_cmos_sensor_twobyte(0X602A, 0X12C6); \
    write_cmos_sensor_twobyte(0X6F12, 0X012F); \
    write_cmos_sensor_twobyte(0X6F12, 0X0219); \
    write_cmos_sensor_twobyte(0X602A, 0X12CE); \
    write_cmos_sensor_twobyte(0X6F12, 0X0139); \
    write_cmos_sensor_twobyte(0X6F12, 0X0223); \
    write_cmos_sensor_twobyte(0X602A, 0X12D6); \
    write_cmos_sensor_twobyte(0X6F12, 0X0143); \
    write_cmos_sensor_twobyte(0X6F12, 0X022D); \
    write_cmos_sensor_twobyte(0X602A, 0X12DE); \
    write_cmos_sensor_twobyte(0X6F12, 0X014D); \
    write_cmos_sensor_twobyte(0X6F12, 0X0237); \
    write_cmos_sensor_twobyte(0X602A, 0X12E6); \
    write_cmos_sensor_twobyte(0X6F12, 0X0157); \
    write_cmos_sensor_twobyte(0X6F12, 0X0241); \
    write_cmos_sensor_twobyte(0X602A, 0X12EE); \
    write_cmos_sensor_twobyte(0X6F12, 0X0161); \
    write_cmos_sensor_twobyte(0X6F12, 0X024B); \
    write_cmos_sensor_twobyte(0X602A, 0X12F6); \
    write_cmos_sensor_twobyte(0X6F12, 0X016B); \
    write_cmos_sensor_twobyte(0X6F12, 0X0255); \
    write_cmos_sensor_twobyte(0X602A, 0X12FE); \
    write_cmos_sensor_twobyte(0X6F12, 0X0175); \
    write_cmos_sensor_twobyte(0X6F12, 0X025F); \
    write_cmos_sensor_twobyte(0X602A, 0X1306); \
    write_cmos_sensor_twobyte(0X6F12, 0X017F); \
    write_cmos_sensor_twobyte(0X6F12, 0X0269); \
    write_cmos_sensor_twobyte(0X602A, 0X130E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0189); \
    write_cmos_sensor_twobyte(0X6F12, 0X0273); \
    write_cmos_sensor_twobyte(0X602A, 0X1316); \
    write_cmos_sensor_twobyte(0X6F12, 0X0193); \
    write_cmos_sensor_twobyte(0X6F12, 0X027D); \
    write_cmos_sensor_twobyte(0X602A, 0X131E); \
    write_cmos_sensor_twobyte(0X6F12, 0X019D); \
    write_cmos_sensor_twobyte(0X6F12, 0X0287); \
    write_cmos_sensor_twobyte(0X602A, 0X1326); \
    write_cmos_sensor_twobyte(0X6F12, 0X01A7); \
    write_cmos_sensor_twobyte(0X6F12, 0X0291); \
    write_cmos_sensor_twobyte(0X602A, 0X132E); \
    write_cmos_sensor_twobyte(0X6F12, 0X01B1); \
    write_cmos_sensor_twobyte(0X6F12, 0X029B); \
    write_cmos_sensor_twobyte(0X602A, 0X1336); \
    write_cmos_sensor_twobyte(0X6F12, 0X01BB); \
    write_cmos_sensor_twobyte(0X6F12, 0X01B0); \
    write_cmos_sensor_twobyte(0X602A, 0X133E); \
    write_cmos_sensor_twobyte(0X6F12, 0X00D0); \
    write_cmos_sensor_twobyte(0X6F12, 0X01C4); \
    write_cmos_sensor_twobyte(0X602A, 0X1346); \
    write_cmos_sensor_twobyte(0X6F12, 0X00E4); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D8); \
    write_cmos_sensor_twobyte(0X602A, 0X134E); \
    write_cmos_sensor_twobyte(0X6F12, 0X00F8); \
    write_cmos_sensor_twobyte(0X6F12, 0X01EC); \
    write_cmos_sensor_twobyte(0X602A, 0X1356); \
    write_cmos_sensor_twobyte(0X6F12, 0X010C); \
    write_cmos_sensor_twobyte(0X6F12, 0X0200); \
    write_cmos_sensor_twobyte(0X602A, 0X135E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0120); \
    write_cmos_sensor_twobyte(0X6F12, 0X0214); \
    write_cmos_sensor_twobyte(0X602A, 0X1366); \
    write_cmos_sensor_twobyte(0X6F12, 0X0134); \
    write_cmos_sensor_twobyte(0X6F12, 0X0228); \
    write_cmos_sensor_twobyte(0X602A, 0X136E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0148); \
    write_cmos_sensor_twobyte(0X6F12, 0X023C); \
    write_cmos_sensor_twobyte(0X602A, 0X1376); \
    write_cmos_sensor_twobyte(0X6F12, 0X015C); \
    write_cmos_sensor_twobyte(0X6F12, 0X0250); \
    write_cmos_sensor_twobyte(0X602A, 0X137E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0170); \
    write_cmos_sensor_twobyte(0X6F12, 0X0264); \
    write_cmos_sensor_twobyte(0X602A, 0X1386); \
    write_cmos_sensor_twobyte(0X6F12, 0X0184); \
    write_cmos_sensor_twobyte(0X6F12, 0X0278); \
    write_cmos_sensor_twobyte(0X602A, 0X138E); \
    write_cmos_sensor_twobyte(0X6F12, 0X0198); \
    write_cmos_sensor_twobyte(0X6F12, 0X028C); \
    write_cmos_sensor_twobyte(0X602A, 0X1396); \
    write_cmos_sensor_twobyte(0X6F12, 0X01AC); \
    write_cmos_sensor_twobyte(0X6F12, 0X02A0); \
    write_cmos_sensor_twobyte(0X602A, 0X139E); \
    write_cmos_sensor_twobyte(0X6F12, 0X01C0); \
    write_cmos_sensor_twobyte(0X602A, 0X14B8); \
    write_cmos_sensor_twobyte(0X6F12, 0X00BE); \
    write_cmos_sensor_twobyte(0X602A, 0X14BE); \
    write_cmos_sensor_twobyte(0X6F12, 0X0105); \
    write_cmos_sensor_twobyte(0X6F12, 0X0024); \
    write_cmos_sensor_twobyte(0X6F12, 0X0012); \
    write_cmos_sensor_twobyte(0X602A, 0X14C6); \
    write_cmos_sensor_twobyte(0X6F12, 0X0012); \
    write_cmos_sensor_twobyte(0X602A, 0X14D0); \
    write_cmos_sensor_twobyte(0X6F12, 0X019A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0111); \
    write_cmos_sensor_twobyte(0X602A, 0X14D6); \
    write_cmos_sensor_twobyte(0X6F12, 0X00C1); \
    write_cmos_sensor_twobyte(0X6F12, 0X0087); \
    write_cmos_sensor_twobyte(0X6F12, 0X0065); \
    write_cmos_sensor_twobyte(0X602A, 0X14DE); \
    write_cmos_sensor_twobyte(0X6F12, 0X0102); \
    write_cmos_sensor_twobyte(0X6F12, 0X0362); \
    write_cmos_sensor_twobyte(0X6F12, 0X028B); \
    write_cmos_sensor_twobyte(0X602A, 0X14E6); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D9); \
    write_cmos_sensor_twobyte(0X6F12, 0X001A); \
    write_cmos_sensor_twobyte(0X6F12, 0X001A); \
    write_cmos_sensor_twobyte(0X602A, 0X14EE); \
    write_cmos_sensor_twobyte(0X6F12, 0X001A); \
    write_cmos_sensor_twobyte(0X602A, 0X1508); \
    write_cmos_sensor_twobyte(0X6F12, 0X0603); \
    write_cmos_sensor_twobyte(0X602A, 0X1550); \
    write_cmos_sensor_twobyte(0X6F12, 0X0306); \
    write_cmos_sensor_twobyte(0X6F12, 0X0606); \
    write_cmos_sensor_twobyte(0X6F12, 0X0606); \
    write_cmos_sensor_twobyte(0X6F12, 0X0603); \
    write_cmos_sensor_twobyte(0X602A, 0X15DE); \
    write_cmos_sensor_twobyte(0X6F12, 0X0107); \
    write_cmos_sensor_twobyte(0X602A, 0X0AEC); \
    write_cmos_sensor_twobyte(0X6F12, 0X0207); \
    write_cmos_sensor_twobyte(0X602A, 0X0A96); \
    write_cmos_sensor_twobyte(0X6F12, 0X1E00); \
    write_cmos_sensor_twobyte(0X602A, 0X2800); \
    write_cmos_sensor_twobyte(0X6F12, 0X0245); \
    write_cmos_sensor_twobyte(0X6F12, 0X0105); \
    write_cmos_sensor_twobyte(0X602A, 0X2816); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X6F12, 0X0180); \
    write_cmos_sensor_twobyte(0X602A, 0X2824); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X602A, 0X2814); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X602A, 0X0998); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X602A, 0X55BE); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X602A, 0X51D0); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X602A, 0X51E0); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X602A, 0X51F0); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X602A, 0X2A12); \
    write_cmos_sensor_twobyte(0X6F12, 0X0102); \
    write_cmos_sensor_twobyte(0X602A, 0X2C68); \
    write_cmos_sensor_twobyte(0X6F12, 0X0005); \
    write_cmos_sensor_twobyte(0X6F12, 0X0005); \
    write_cmos_sensor_twobyte(0X6F12, 0X0005); \
    write_cmos_sensor_twobyte(0X6F12, 0X0005); \
    write_cmos_sensor_twobyte(0X6F12, 0X0003); \
    write_cmos_sensor_twobyte(0X6F12, 0X0003); \
    write_cmos_sensor_twobyte(0X6F12, 0X0003); \
    write_cmos_sensor_twobyte(0X6F12, 0X0003); \
    write_cmos_sensor_twobyte(0X6F12, 0X0005); \
    write_cmos_sensor_twobyte(0X6F12, 0X0005); \
    write_cmos_sensor_twobyte(0X6F12, 0X0005); \
    write_cmos_sensor_twobyte(0X6F12, 0X0005); \
    write_cmos_sensor_twobyte(0X6F12, 0X0003); \
    write_cmos_sensor_twobyte(0X6F12, 0X0003); \
    write_cmos_sensor_twobyte(0X6F12, 0X0003); \
    write_cmos_sensor_twobyte(0X6F12, 0X0003); \
    write_cmos_sensor_twobyte(0X602A, 0X372C); \
    write_cmos_sensor_twobyte(0X6F12, 0X000F); \
    write_cmos_sensor_twobyte(0X6F12, 0X000D); \
    write_cmos_sensor_twobyte(0X6F12, 0X000F); \
    write_cmos_sensor_twobyte(0X6F12, 0X000D); \
    write_cmos_sensor_twobyte(0X6F12, 0X000B); \
    write_cmos_sensor_twobyte(0X6F12, 0X000C); \
    write_cmos_sensor_twobyte(0X6F12, 0X000B); \
    write_cmos_sensor_twobyte(0X6F12, 0X000C); \
    write_cmos_sensor_twobyte(0X6F12, 0X000F); \
    write_cmos_sensor_twobyte(0X6F12, 0X000D); \
    write_cmos_sensor_twobyte(0X6F12, 0X000F); \
    write_cmos_sensor_twobyte(0X6F12, 0X000D); \
    write_cmos_sensor_twobyte(0X6F12, 0X000B); \
    write_cmos_sensor_twobyte(0X6F12, 0X000C); \
    write_cmos_sensor_twobyte(0X6F12, 0X000B); \
    write_cmos_sensor_twobyte(0X6F12, 0X000C); \
    write_cmos_sensor_twobyte(0X602A, 0X41F0); \
    write_cmos_sensor_twobyte(0X6F12, 0X001B); \
    write_cmos_sensor_twobyte(0X6F12, 0X001A); \
    write_cmos_sensor_twobyte(0X6F12, 0X001B); \
    write_cmos_sensor_twobyte(0X6F12, 0X001A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0017); \
    write_cmos_sensor_twobyte(0X6F12, 0X0014); \
    write_cmos_sensor_twobyte(0X6F12, 0X0017); \
    write_cmos_sensor_twobyte(0X6F12, 0X0014); \
    write_cmos_sensor_twobyte(0X6F12, 0X001B); \
    write_cmos_sensor_twobyte(0X6F12, 0X001A); \
    write_cmos_sensor_twobyte(0X6F12, 0X001B); \
    write_cmos_sensor_twobyte(0X6F12, 0X001A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0017); \
    write_cmos_sensor_twobyte(0X6F12, 0X0014); \
    write_cmos_sensor_twobyte(0X6F12, 0X0017); \
    write_cmos_sensor_twobyte(0X6F12, 0X0014); \
    write_cmos_sensor_twobyte(0X602A, 0X0A80); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X6F12, 0X3110); \
    write_cmos_sensor_twobyte(0X602A, 0X51D2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X602A, 0X51E2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0800); \
    write_cmos_sensor_twobyte(0X602A, 0X51F2); \
    write_cmos_sensor_twobyte(0X6F12, 0X1000); \
    write_cmos_sensor_twobyte(0X602A, 0X0A52); \
    write_cmos_sensor_twobyte(0X6F12, 0X2100); \
    write_cmos_sensor_twobyte(0X602A, 0X0A50); \
    write_cmos_sensor_twobyte(0X6F12, 0X2100); \
    write_cmos_sensor_twobyte(0X602A, 0X0B06); \
    write_cmos_sensor_twobyte(0X6F12, 0X0800); \
    write_cmos_sensor_twobyte(0X602A, 0X5840); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X602A, 0X0AF4); \
    write_cmos_sensor_twobyte(0X6F12, 0X0004); \
	/* out pedestal 32 */                      \
}while(0)


/*****************************************************************************
 *
 * Description:
 * ------------
 *     mode 1 preview setting
 *     $MIPI[Width:4032,Height:1512,Format:Raw10,Lane:4,ErrorCheck:0,PolarityData:0,PolarityClock:0,Buffer:4,DataRate:2034,useEmbData:0]
 *     $MV1[MCLK:24,Width:4032,Height:1512,Format:MIPI_Raw10,mipi_lane:4,mipi_datarate:2034,pvi_pclk_inverse:0]
 *
 ****************************************************************************/
#define _S5K2L7_MODE1_PREVIEW_ do{ \
    /* Stream Off */                               \
    write_cmos_sensor(0x0100,0x00);                \
    while(1)                                       \
    {                                              \
        if( read_cmos_sensor(0x0005)==0xFF) break; \
    }                                              \
    write_cmos_sensor_twobyte(0X6028, 0X4000); \
    write_cmos_sensor_twobyte(0X6214, 0X7970); \
    write_cmos_sensor_twobyte(0X6218, 0X7150); \
    write_cmos_sensor_twobyte(0X0344, 0X0000); \
    write_cmos_sensor_twobyte(0X0346, 0X0000); \
    write_cmos_sensor_twobyte(0X0348, 0X1F7F); \
    write_cmos_sensor_twobyte(0X034A, 0X0BDF); \
    write_cmos_sensor_twobyte(0X034C, 0X0FC0); \
    write_cmos_sensor_twobyte(0X034E, 0X05E8); \
    write_cmos_sensor_twobyte(0X0408, 0X0000); \
    write_cmos_sensor_twobyte(0X040A, 0X0004); \
    write_cmos_sensor_twobyte(0X0900, 0X0122); \
    write_cmos_sensor_twobyte(0X0380, 0X0001); \
    write_cmos_sensor_twobyte(0X0382, 0X0003); \
    write_cmos_sensor_twobyte(0X0384, 0X0001); \
    write_cmos_sensor_twobyte(0X0386, 0X0003); \
    write_cmos_sensor_twobyte(0X0400, 0X0000); \
    write_cmos_sensor_twobyte(0X0404, 0X0010); \
    write_cmos_sensor_twobyte(0X3060, 0X0100); \
    write_cmos_sensor_twobyte(0X0114, 0X0300); \
    write_cmos_sensor_twobyte(0X0110, 0X1002); \
    write_cmos_sensor_twobyte(0X0136, 0X1800); \
    write_cmos_sensor_twobyte(0X0304, 0X0006); \
    write_cmos_sensor_twobyte(0X0306, 0X01E0); \
    write_cmos_sensor_twobyte(0X0302, 0X0001); \
    write_cmos_sensor_twobyte(0X0300, 0X0004); \
    write_cmos_sensor_twobyte(0X030C, 0X0001); \
    write_cmos_sensor_twobyte(0X030E, 0X0004); \
    write_cmos_sensor_twobyte(0X0310, 0X0153); \
    write_cmos_sensor_twobyte(0X0312, 0X0000); \
    write_cmos_sensor_twobyte(0X030A, 0X0001); \
    write_cmos_sensor_twobyte(0X0308, 0X0008); \
    write_cmos_sensor_twobyte(0X0342, 0X27B0); \
    write_cmos_sensor_twobyte(0X0340, 0X0C4D); \
    write_cmos_sensor_twobyte(0X021E, 0X0000); \
    write_cmos_sensor_twobyte(0X3098, 0X0400); \
    write_cmos_sensor_twobyte(0X309A, 0X0002); \
    write_cmos_sensor_twobyte(0X30BC, 0X0031); \
    write_cmos_sensor_twobyte(0X30A8, 0X0000); \
    write_cmos_sensor_twobyte(0X30AC, 0X0000); \
    write_cmos_sensor_twobyte(0X30A0, 0X0000); \
    write_cmos_sensor_twobyte(0X30A4, 0X0000); \
    write_cmos_sensor_twobyte(0XF41E, 0X2180); \
    write_cmos_sensor_twobyte(0X6028, 0X2000); \
    write_cmos_sensor_twobyte(0X602A, 0X0990); \
    write_cmos_sensor_twobyte(0X6F12, 0X0020); \
    write_cmos_sensor_twobyte(0X602A, 0X0AF8); \
    write_cmos_sensor_twobyte(0X6F12, 0X0008); \
    write_cmos_sensor_twobyte(0X602A, 0X27A8); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X602A, 0X09AA); \
    write_cmos_sensor_twobyte(0X6F12, 0X1E7F); \
    write_cmos_sensor_twobyte(0X6F12, 0X1E7F); \
    write_cmos_sensor_twobyte(0X602A, 0X16B6); \
    write_cmos_sensor_twobyte(0X6F12, 0X12AF); \
    write_cmos_sensor_twobyte(0X6F12, 0X0328); \
    write_cmos_sensor_twobyte(0X602A, 0X1688); \
    write_cmos_sensor_twobyte(0X6F12, 0X00A0); \
    write_cmos_sensor_twobyte(0X602A, 0X168C); \
    write_cmos_sensor_twobyte(0X6F12, 0X0028); \
    write_cmos_sensor_twobyte(0X6F12, 0X0030); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C18); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C18); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C28); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C28); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C20); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C20); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C30); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C30); \
    write_cmos_sensor_twobyte(0X602A, 0X15F4); \
    write_cmos_sensor_twobyte(0X6F12, 0X0004); \
    write_cmos_sensor_twobyte(0X602A, 0X16BE); \
    write_cmos_sensor_twobyte(0X6F12, 0X06C0); \
    write_cmos_sensor_twobyte(0X6F12, 0X0101); \
    write_cmos_sensor_twobyte(0X602A, 0X0B16); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X602A, 0X11D2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0258); \
    write_cmos_sensor_twobyte(0X602A, 0X127A); \
    write_cmos_sensor_twobyte(0X6F12, 0X01B5); \
    write_cmos_sensor_twobyte(0X602A, 0X1282); \
    write_cmos_sensor_twobyte(0X6F12, 0X01BF); \
    write_cmos_sensor_twobyte(0X602A, 0X128A); \
    write_cmos_sensor_twobyte(0X6F12, 0X01C9); \
    write_cmos_sensor_twobyte(0X602A, 0X1292); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D3); \
    write_cmos_sensor_twobyte(0X602A, 0X129A); \
    write_cmos_sensor_twobyte(0X6F12, 0X01DD); \
    write_cmos_sensor_twobyte(0X602A, 0X12A2); \
    write_cmos_sensor_twobyte(0X6F12, 0X01E7); \
    write_cmos_sensor_twobyte(0X602A, 0X12AA); \
    write_cmos_sensor_twobyte(0X6F12, 0X01F1); \
    write_cmos_sensor_twobyte(0X602A, 0X12B2); \
    write_cmos_sensor_twobyte(0X6F12, 0X01FB); \
    write_cmos_sensor_twobyte(0X602A, 0X12BA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0205); \
    write_cmos_sensor_twobyte(0X602A, 0X12C2); \
    write_cmos_sensor_twobyte(0X6F12, 0X020F); \
    write_cmos_sensor_twobyte(0X602A, 0X12CA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0219); \
    write_cmos_sensor_twobyte(0X602A, 0X12D2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0223); \
    write_cmos_sensor_twobyte(0X602A, 0X12DA); \
    write_cmos_sensor_twobyte(0X6F12, 0X022D); \
    write_cmos_sensor_twobyte(0X602A, 0X12E2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0237); \
    write_cmos_sensor_twobyte(0X602A, 0X12EA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0241); \
    write_cmos_sensor_twobyte(0X602A, 0X12F2); \
    write_cmos_sensor_twobyte(0X6F12, 0X024B); \
    write_cmos_sensor_twobyte(0X602A, 0X12FA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0255); \
    write_cmos_sensor_twobyte(0X602A, 0X1302); \
    write_cmos_sensor_twobyte(0X6F12, 0X025F); \
    write_cmos_sensor_twobyte(0X602A, 0X130A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0269); \
    write_cmos_sensor_twobyte(0X602A, 0X1312); \
    write_cmos_sensor_twobyte(0X6F12, 0X0273); \
    write_cmos_sensor_twobyte(0X602A, 0X131A); \
    write_cmos_sensor_twobyte(0X6F12, 0X027D); \
    write_cmos_sensor_twobyte(0X602A, 0X1322); \
    write_cmos_sensor_twobyte(0X6F12, 0X0287); \
    write_cmos_sensor_twobyte(0X602A, 0X132A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0291); \
    write_cmos_sensor_twobyte(0X602A, 0X1332); \
    write_cmos_sensor_twobyte(0X6F12, 0X029B); \
    write_cmos_sensor_twobyte(0X602A, 0X133A); \
    write_cmos_sensor_twobyte(0X6F12, 0X01B0); \
    write_cmos_sensor_twobyte(0X602A, 0X1342); \
    write_cmos_sensor_twobyte(0X6F12, 0X01C4); \
    write_cmos_sensor_twobyte(0X602A, 0X134A); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D8); \
    write_cmos_sensor_twobyte(0X602A, 0X1352); \
    write_cmos_sensor_twobyte(0X6F12, 0X01EC); \
    write_cmos_sensor_twobyte(0X602A, 0X135A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0200); \
    write_cmos_sensor_twobyte(0X602A, 0X1362); \
    write_cmos_sensor_twobyte(0X6F12, 0X0214); \
    write_cmos_sensor_twobyte(0X602A, 0X136A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0228); \
    write_cmos_sensor_twobyte(0X602A, 0X1372); \
    write_cmos_sensor_twobyte(0X6F12, 0X023C); \
    write_cmos_sensor_twobyte(0X602A, 0X137A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0250); \
    write_cmos_sensor_twobyte(0X602A, 0X1382); \
    write_cmos_sensor_twobyte(0X6F12, 0X0264); \
    write_cmos_sensor_twobyte(0X602A, 0X138A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0278); \
    write_cmos_sensor_twobyte(0X602A, 0X1392); \
    write_cmos_sensor_twobyte(0X6F12, 0X028C); \
    write_cmos_sensor_twobyte(0X602A, 0X139A); \
    write_cmos_sensor_twobyte(0X6F12, 0X02A0); \
    write_cmos_sensor_twobyte(0X602A, 0X14BA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0096); \
    write_cmos_sensor_twobyte(0X602A, 0X0C22); \
    write_cmos_sensor_twobyte(0X6F12, 0X000A); \
    write_cmos_sensor_twobyte(0X602A, 0X15DC); \
    write_cmos_sensor_twobyte(0X6F12, 0X0001); \
    write_cmos_sensor_twobyte(0X602A, 0X0A94); \
    write_cmos_sensor_twobyte(0X6F12, 0X1000); \
    write_cmos_sensor_twobyte(0X602A, 0X0B62); \
    write_cmos_sensor_twobyte(0X6F12, 0X0146); \
    write_cmos_sensor_twobyte(0X602A, 0X8B2C); \
    write_cmos_sensor_twobyte(0X6F12, 0X0008); \
    write_cmos_sensor_twobyte(0X602A, 0X5150); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X602A, 0X0AE6); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BE0); \
    write_cmos_sensor_twobyte(0X602A, 0X29D8); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BE0); \
    write_cmos_sensor_twobyte(0X602A, 0X29E2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BE0); \
    write_cmos_sensor_twobyte(0X602A, 0X2958); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BE0); \
    write_cmos_sensor_twobyte(0X602A, 0X2998); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BE0); \
    write_cmos_sensor_twobyte(0X602A, 0X2962); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BE0); \
    write_cmos_sensor_twobyte(0X602A, 0X29A2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BE0); \
    write_cmos_sensor_twobyte(0X6028, 0X4000); \
    write_cmos_sensor_twobyte(0X6214, 0X79F0); \
    write_cmos_sensor_twobyte(0X6218, 0X79F0); \
    /* Stream On */                            \
    write_cmos_sensor_twobyte(0x0100, 0x0100); \
    mDELAY(10);                                \
}while(0)


/*****************************************************************************
 *
 * Description:
 * ------------
 *     mode 1 capture setting (M1_fullsize_setting)
 *     $MIPI[Width:8064,Height:3024,Format:Raw10,Lane:4,ErrorCheck:0,PolarityData:0,PolarityClock:0,Buffer:4,DataRate:2034,useEmbData:0]
 *     $MV1[MCLK:24,Width:8064,Height:3024,Format:MIPI_Raw10,mipi_lane:4,mipi_datarate:2034,pvi_pclk_inverse:0]
 *
 ****************************************************************************/
#define _S5K2L7_MODE1_CAPTURE_ do{ \
    /* Stream Off */                               \
    write_cmos_sensor(0x0100,0x00);                \
    while(1)                                       \
    {                                              \
        if( read_cmos_sensor(0x0005)==0xFF) break; \
    }                                              \
    write_cmos_sensor_twobyte(0X6028, 0X4000); \
    write_cmos_sensor_twobyte(0X6214, 0X7970); \
    write_cmos_sensor_twobyte(0X6218, 0X7150); \
    write_cmos_sensor_twobyte(0X0344, 0X0000); \
    write_cmos_sensor_twobyte(0X0346, 0X0000); \
    write_cmos_sensor_twobyte(0X0348, 0X1F7F); \
    write_cmos_sensor_twobyte(0X034A, 0X0BDF); \
    write_cmos_sensor_twobyte(0X034C, 0X1F80); \
    write_cmos_sensor_twobyte(0X034E, 0X0BD0); \
    write_cmos_sensor_twobyte(0X0408, 0X0000); \
    write_cmos_sensor_twobyte(0X040A, 0X0008); \
    write_cmos_sensor_twobyte(0X0900, 0X0011); \
    write_cmos_sensor_twobyte(0X0380, 0X0001); \
    write_cmos_sensor_twobyte(0X0382, 0X0001); \
    write_cmos_sensor_twobyte(0X0384, 0X0001); \
    write_cmos_sensor_twobyte(0X0386, 0X0001); \
    write_cmos_sensor_twobyte(0X0400, 0X0000); \
    write_cmos_sensor_twobyte(0X0404, 0X0010); \
    write_cmos_sensor_twobyte(0X3060, 0X0100); \
    write_cmos_sensor_twobyte(0X0114, 0X0300); \
    write_cmos_sensor_twobyte(0X0110, 0X1002); \
    write_cmos_sensor_twobyte(0X0136, 0X1800); \
    write_cmos_sensor_twobyte(0X0304, 0X0006); \
    write_cmos_sensor_twobyte(0X0306, 0X01E0); \
    write_cmos_sensor_twobyte(0X0302, 0X0001); \
    write_cmos_sensor_twobyte(0X0300, 0X0004); \
    write_cmos_sensor_twobyte(0X030C, 0X0001); \
    write_cmos_sensor_twobyte(0X030E, 0X0004); \
    write_cmos_sensor_twobyte(0X0310, 0X0153); \
    write_cmos_sensor_twobyte(0X0312, 0X0000); \
    write_cmos_sensor_twobyte(0X030A, 0X0001); \
    write_cmos_sensor_twobyte(0X0308, 0X0008); \
    write_cmos_sensor_twobyte(0X0342, 0X27E0); \
    write_cmos_sensor_twobyte(0X0340, 0X0C3E); \
    write_cmos_sensor_twobyte(0X021E, 0X0000); \
    write_cmos_sensor_twobyte(0X3098, 0X0400); \
    write_cmos_sensor_twobyte(0X309A, 0X0002); \
    write_cmos_sensor_twobyte(0X30BC, 0X0031); \
    write_cmos_sensor_twobyte(0X30A8, 0X0000); \
    write_cmos_sensor_twobyte(0X30AC, 0X0000); \
    write_cmos_sensor_twobyte(0X30A0, 0X0000); \
    write_cmos_sensor_twobyte(0X30A4, 0X0000); \
    write_cmos_sensor_twobyte(0XF41E, 0X2180); \
    write_cmos_sensor_twobyte(0X6028, 0X2000); \
    write_cmos_sensor_twobyte(0X602A, 0X0990); \
    write_cmos_sensor_twobyte(0X6F12, 0X0020); \
    write_cmos_sensor_twobyte(0X602A, 0X0AF8); \
    write_cmos_sensor_twobyte(0X6F12, 0X0004); \
    write_cmos_sensor_twobyte(0X602A, 0X27A8); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X602A, 0X09AA); \
    write_cmos_sensor_twobyte(0X6F12, 0X1E7F); \
    write_cmos_sensor_twobyte(0X6F12, 0X1E7F); \
    write_cmos_sensor_twobyte(0X602A, 0X16B6); \
    write_cmos_sensor_twobyte(0X6F12, 0X122F); \
    write_cmos_sensor_twobyte(0X6F12, 0X4328); \
    write_cmos_sensor_twobyte(0X602A, 0X1688); \
    write_cmos_sensor_twobyte(0X6F12, 0X00A2); \
    write_cmos_sensor_twobyte(0X602A, 0X168C); \
    write_cmos_sensor_twobyte(0X6F12, 0X0028); \
    write_cmos_sensor_twobyte(0X6F12, 0X0030); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C18); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C18); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C28); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C28); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C20); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C20); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C30); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C30); \
    write_cmos_sensor_twobyte(0X602A, 0X15F4); \
    write_cmos_sensor_twobyte(0X6F12, 0X0004); \
    write_cmos_sensor_twobyte(0X602A, 0X16BE); \
    write_cmos_sensor_twobyte(0X6F12, 0X06C0); \
    write_cmos_sensor_twobyte(0X6F12, 0X0101); \
    write_cmos_sensor_twobyte(0X602A, 0X0B16); \
    write_cmos_sensor_twobyte(0X6F12, 0X0200); \
    write_cmos_sensor_twobyte(0X602A, 0X11D2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0258); \
    write_cmos_sensor_twobyte(0X602A, 0X127A); \
    write_cmos_sensor_twobyte(0X6F12, 0X01B5); \
    write_cmos_sensor_twobyte(0X602A, 0X1282); \
    write_cmos_sensor_twobyte(0X6F12, 0X01BF); \
    write_cmos_sensor_twobyte(0X602A, 0X128A); \
    write_cmos_sensor_twobyte(0X6F12, 0X01C9); \
    write_cmos_sensor_twobyte(0X602A, 0X1292); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D3); \
    write_cmos_sensor_twobyte(0X602A, 0X129A); \
    write_cmos_sensor_twobyte(0X6F12, 0X01DD); \
    write_cmos_sensor_twobyte(0X602A, 0X12A2); \
    write_cmos_sensor_twobyte(0X6F12, 0X01E7); \
    write_cmos_sensor_twobyte(0X602A, 0X12AA); \
    write_cmos_sensor_twobyte(0X6F12, 0X01F1); \
    write_cmos_sensor_twobyte(0X602A, 0X12B2); \
    write_cmos_sensor_twobyte(0X6F12, 0X01FB); \
    write_cmos_sensor_twobyte(0X602A, 0X12BA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0205); \
    write_cmos_sensor_twobyte(0X602A, 0X12C2); \
    write_cmos_sensor_twobyte(0X6F12, 0X020F); \
    write_cmos_sensor_twobyte(0X602A, 0X12CA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0219); \
    write_cmos_sensor_twobyte(0X602A, 0X12D2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0223); \
    write_cmos_sensor_twobyte(0X602A, 0X12DA); \
    write_cmos_sensor_twobyte(0X6F12, 0X022D); \
    write_cmos_sensor_twobyte(0X602A, 0X12E2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0237); \
    write_cmos_sensor_twobyte(0X602A, 0X12EA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0241); \
    write_cmos_sensor_twobyte(0X602A, 0X12F2); \
    write_cmos_sensor_twobyte(0X6F12, 0X024B); \
    write_cmos_sensor_twobyte(0X602A, 0X12FA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0255); \
    write_cmos_sensor_twobyte(0X602A, 0X1302); \
    write_cmos_sensor_twobyte(0X6F12, 0X025F); \
    write_cmos_sensor_twobyte(0X602A, 0X130A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0269); \
    write_cmos_sensor_twobyte(0X602A, 0X1312); \
    write_cmos_sensor_twobyte(0X6F12, 0X0273); \
    write_cmos_sensor_twobyte(0X602A, 0X131A); \
    write_cmos_sensor_twobyte(0X6F12, 0X027D); \
    write_cmos_sensor_twobyte(0X602A, 0X1322); \
    write_cmos_sensor_twobyte(0X6F12, 0X0287); \
    write_cmos_sensor_twobyte(0X602A, 0X132A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0291); \
    write_cmos_sensor_twobyte(0X602A, 0X1332); \
    write_cmos_sensor_twobyte(0X6F12, 0X029B); \
    write_cmos_sensor_twobyte(0X602A, 0X133A); \
    write_cmos_sensor_twobyte(0X6F12, 0X01B0); \
    write_cmos_sensor_twobyte(0X602A, 0X1342); \
    write_cmos_sensor_twobyte(0X6F12, 0X01C4); \
    write_cmos_sensor_twobyte(0X602A, 0X134A); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D8); \
    write_cmos_sensor_twobyte(0X602A, 0X1352); \
    write_cmos_sensor_twobyte(0X6F12, 0X01EC); \
    write_cmos_sensor_twobyte(0X602A, 0X135A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0200); \
    write_cmos_sensor_twobyte(0X602A, 0X1362); \
    write_cmos_sensor_twobyte(0X6F12, 0X0214); \
    write_cmos_sensor_twobyte(0X602A, 0X136A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0228); \
    write_cmos_sensor_twobyte(0X602A, 0X1372); \
    write_cmos_sensor_twobyte(0X6F12, 0X023C); \
    write_cmos_sensor_twobyte(0X602A, 0X137A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0250); \
    write_cmos_sensor_twobyte(0X602A, 0X1382); \
    write_cmos_sensor_twobyte(0X6F12, 0X0264); \
    write_cmos_sensor_twobyte(0X602A, 0X138A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0278); \
    write_cmos_sensor_twobyte(0X602A, 0X1392); \
    write_cmos_sensor_twobyte(0X6F12, 0X028C); \
    write_cmos_sensor_twobyte(0X602A, 0X139A); \
    write_cmos_sensor_twobyte(0X6F12, 0X02A0); \
    write_cmos_sensor_twobyte(0X602A, 0X14BA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0096); \
    write_cmos_sensor_twobyte(0X602A, 0X0C22); \
    write_cmos_sensor_twobyte(0X6F12, 0X000A); \
    write_cmos_sensor_twobyte(0X602A, 0X15DC); \
    write_cmos_sensor_twobyte(0X6F12, 0X0001); \
    write_cmos_sensor_twobyte(0X602A, 0X0A94); \
    write_cmos_sensor_twobyte(0X6F12, 0X2000); \
    write_cmos_sensor_twobyte(0X602A, 0X0B62); \
    write_cmos_sensor_twobyte(0X6F12, 0X0146); \
    write_cmos_sensor_twobyte(0X602A, 0X8B2C); \
    write_cmos_sensor_twobyte(0X6F12, 0X0008); \
    write_cmos_sensor_twobyte(0X602A, 0X5150); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X602A, 0X0AE6); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BE0); \
    write_cmos_sensor_twobyte(0X602A, 0X29D8); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BE0); \
    write_cmos_sensor_twobyte(0X602A, 0X29E2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BE0); \
    write_cmos_sensor_twobyte(0X602A, 0X2958); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BE0); \
    write_cmos_sensor_twobyte(0X602A, 0X2998); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BE0); \
    write_cmos_sensor_twobyte(0X602A, 0X2962); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BE0); \
    write_cmos_sensor_twobyte(0X602A, 0X29A2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BE0); \
    write_cmos_sensor_twobyte(0X6028, 0X4000); \
    write_cmos_sensor_twobyte(0X6214, 0X79F0); \
    write_cmos_sensor_twobyte(0X6218, 0X79F0); \
    /* Stream On */                            \
    write_cmos_sensor_twobyte(0x0100,0x0100);  \
    mDELAY(10);                                \
}while(0)


/*****************************************************************************
 *
 * Description:
 * ------------
 *     mode 1 high speed video setting
 *     $MIPI[Width:2688,Height:756,Format:Raw10,Lane:4,ErrorCheck:0,PolarityData:0,PolarityClock:0,Buffer:4,DataRate:2034,useEmbData:0]
 *     $MV1[MCLK:24,Width:2688,Height:756,Format:MIPI_Raw10,mipi_lane:4,mipi_datarate:2034,pvi_pclk_inverse:0]
 *
 ****************************************************************************/
#define _S5K2L7_MODE1_HS_VIDEO_ do{ \
    /* Stream Off */                               \
    write_cmos_sensor(0x0100,0x00);                \
    while(1)                                       \
    {                                              \
        if( read_cmos_sensor(0x0005)==0xFF) break; \
    }                                              \
    write_cmos_sensor_twobyte(0X6028, 0X4000); \
    write_cmos_sensor_twobyte(0X6214, 0X7970); \
    write_cmos_sensor_twobyte(0X6218, 0X7150); \
    write_cmos_sensor_twobyte(0X0344, 0X0000); \
    write_cmos_sensor_twobyte(0X0346, 0X016E); \
    write_cmos_sensor_twobyte(0X0348, 0X1F7F); \
    write_cmos_sensor_twobyte(0X034A, 0X0A61); \
    write_cmos_sensor_twobyte(0X034C, 0X0A80); \
    write_cmos_sensor_twobyte(0X034E, 0X02F4); \
    write_cmos_sensor_twobyte(0X0408, 0X0000); \
    write_cmos_sensor_twobyte(0X040A, 0X0004); \
    write_cmos_sensor_twobyte(0X0900, 0X0113); \
    write_cmos_sensor_twobyte(0X0380, 0X0001); \
    write_cmos_sensor_twobyte(0X0382, 0X0001); \
    write_cmos_sensor_twobyte(0X0384, 0X0001); \
    write_cmos_sensor_twobyte(0X0386, 0X0005); \
    write_cmos_sensor_twobyte(0X0400, 0X0000); \
    write_cmos_sensor_twobyte(0X0404, 0X0010); \
    write_cmos_sensor_twobyte(0X3060, 0X0103); \
    write_cmos_sensor_twobyte(0X0114, 0X0300); \
    write_cmos_sensor_twobyte(0X0110, 0X1002); \
    write_cmos_sensor_twobyte(0X0136, 0X1800); \
    write_cmos_sensor_twobyte(0X0304, 0X0006); \
    write_cmos_sensor_twobyte(0X0306, 0X01E0); \
    write_cmos_sensor_twobyte(0X0302, 0X0001); \
    write_cmos_sensor_twobyte(0X0300, 0X0004); \
    write_cmos_sensor_twobyte(0X030C, 0X0001); \
    write_cmos_sensor_twobyte(0X030E, 0X0004); \
    write_cmos_sensor_twobyte(0X0310, 0X0153); \
    write_cmos_sensor_twobyte(0X0312, 0X0000); \
    write_cmos_sensor_twobyte(0X030A, 0X0001); \
    write_cmos_sensor_twobyte(0X0308, 0X0008); \
    write_cmos_sensor_twobyte(0X0342, 0X27B0); \
    write_cmos_sensor_twobyte(0X0340, 0X0312); \
    write_cmos_sensor_twobyte(0X021E, 0X0000); \
    write_cmos_sensor_twobyte(0X3098, 0X0400); \
    write_cmos_sensor_twobyte(0X309A, 0X0002); \
    write_cmos_sensor_twobyte(0X30BC, 0X0031); \
    write_cmos_sensor_twobyte(0X30A8, 0X0000); \
    write_cmos_sensor_twobyte(0X30AC, 0X0000); \
    write_cmos_sensor_twobyte(0X30A0, 0X0000); \
    write_cmos_sensor_twobyte(0X30A4, 0X0000); \
    write_cmos_sensor_twobyte(0XF41E, 0X2180); \
    write_cmos_sensor_twobyte(0X6028, 0X2000); \
    write_cmos_sensor_twobyte(0X602A, 0X0990); \
    write_cmos_sensor_twobyte(0X6F12, 0X0020); \
    write_cmos_sensor_twobyte(0X602A, 0X0AF8); \
    write_cmos_sensor_twobyte(0X6F12, 0X000C); \
    write_cmos_sensor_twobyte(0X602A, 0X27A8); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X602A, 0X09AA); \
    write_cmos_sensor_twobyte(0X6F12, 0X1E7F); \
    write_cmos_sensor_twobyte(0X6F12, 0X1E7F); \
    write_cmos_sensor_twobyte(0X602A, 0X16B6); \
    write_cmos_sensor_twobyte(0X6F12, 0X122F); \
    write_cmos_sensor_twobyte(0X6F12, 0X0328); \
    write_cmos_sensor_twobyte(0X602A, 0X1688); \
    write_cmos_sensor_twobyte(0X6F12, 0X00A0); \
    write_cmos_sensor_twobyte(0X602A, 0X168C); \
    write_cmos_sensor_twobyte(0X6F12, 0X0028); \
    write_cmos_sensor_twobyte(0X6F12, 0X0034); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C16); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C16); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C1C); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C1C); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C22); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C22); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C28); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C28); \
    write_cmos_sensor_twobyte(0X602A, 0X15F4); \
    write_cmos_sensor_twobyte(0X6F12, 0X0004); \
    write_cmos_sensor_twobyte(0X602A, 0X16BE); \
    write_cmos_sensor_twobyte(0X6F12, 0X06C0); \
    write_cmos_sensor_twobyte(0X6F12, 0X0101); \
    write_cmos_sensor_twobyte(0X602A, 0X0B16); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X602A, 0X11D2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0258); \
    write_cmos_sensor_twobyte(0X602A, 0X127A); \
    write_cmos_sensor_twobyte(0X6F12, 0X01B5); \
    write_cmos_sensor_twobyte(0X602A, 0X1282); \
    write_cmos_sensor_twobyte(0X6F12, 0X01BF); \
    write_cmos_sensor_twobyte(0X602A, 0X128A); \
    write_cmos_sensor_twobyte(0X6F12, 0X01C9); \
    write_cmos_sensor_twobyte(0X602A, 0X1292); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D3); \
    write_cmos_sensor_twobyte(0X602A, 0X129A); \
    write_cmos_sensor_twobyte(0X6F12, 0X01DD); \
    write_cmos_sensor_twobyte(0X602A, 0X12A2); \
    write_cmos_sensor_twobyte(0X6F12, 0X01E7); \
    write_cmos_sensor_twobyte(0X602A, 0X12AA); \
    write_cmos_sensor_twobyte(0X6F12, 0X01F1); \
    write_cmos_sensor_twobyte(0X602A, 0X12B2); \
    write_cmos_sensor_twobyte(0X6F12, 0X01FB); \
    write_cmos_sensor_twobyte(0X602A, 0X12BA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0205); \
    write_cmos_sensor_twobyte(0X602A, 0X12C2); \
    write_cmos_sensor_twobyte(0X6F12, 0X020F); \
    write_cmos_sensor_twobyte(0X602A, 0X12CA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0219); \
    write_cmos_sensor_twobyte(0X602A, 0X12D2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0223); \
    write_cmos_sensor_twobyte(0X602A, 0X12DA); \
    write_cmos_sensor_twobyte(0X6F12, 0X022D); \
    write_cmos_sensor_twobyte(0X602A, 0X12E2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0237); \
    write_cmos_sensor_twobyte(0X602A, 0X12EA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0241); \
    write_cmos_sensor_twobyte(0X602A, 0X12F2); \
    write_cmos_sensor_twobyte(0X6F12, 0X024B); \
    write_cmos_sensor_twobyte(0X602A, 0X12FA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0255); \
    write_cmos_sensor_twobyte(0X602A, 0X1302); \
    write_cmos_sensor_twobyte(0X6F12, 0X025F); \
    write_cmos_sensor_twobyte(0X602A, 0X130A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0269); \
    write_cmos_sensor_twobyte(0X602A, 0X1312); \
    write_cmos_sensor_twobyte(0X6F12, 0X0273); \
    write_cmos_sensor_twobyte(0X602A, 0X131A); \
    write_cmos_sensor_twobyte(0X6F12, 0X027D); \
    write_cmos_sensor_twobyte(0X602A, 0X1322); \
    write_cmos_sensor_twobyte(0X6F12, 0X0287); \
    write_cmos_sensor_twobyte(0X602A, 0X132A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0291); \
    write_cmos_sensor_twobyte(0X602A, 0X1332); \
    write_cmos_sensor_twobyte(0X6F12, 0X029B); \
    write_cmos_sensor_twobyte(0X602A, 0X133A); \
    write_cmos_sensor_twobyte(0X6F12, 0X01B0); \
    write_cmos_sensor_twobyte(0X602A, 0X1342); \
    write_cmos_sensor_twobyte(0X6F12, 0X01C4); \
    write_cmos_sensor_twobyte(0X602A, 0X134A); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D8); \
    write_cmos_sensor_twobyte(0X602A, 0X1352); \
    write_cmos_sensor_twobyte(0X6F12, 0X01EC); \
    write_cmos_sensor_twobyte(0X602A, 0X135A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0200); \
    write_cmos_sensor_twobyte(0X602A, 0X1362); \
    write_cmos_sensor_twobyte(0X6F12, 0X0214); \
    write_cmos_sensor_twobyte(0X602A, 0X136A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0228); \
    write_cmos_sensor_twobyte(0X602A, 0X1372); \
    write_cmos_sensor_twobyte(0X6F12, 0X023C); \
    write_cmos_sensor_twobyte(0X602A, 0X137A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0250); \
    write_cmos_sensor_twobyte(0X602A, 0X1382); \
    write_cmos_sensor_twobyte(0X6F12, 0X0264); \
    write_cmos_sensor_twobyte(0X602A, 0X138A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0278); \
    write_cmos_sensor_twobyte(0X602A, 0X1392); \
    write_cmos_sensor_twobyte(0X6F12, 0X028C); \
    write_cmos_sensor_twobyte(0X602A, 0X139A); \
    write_cmos_sensor_twobyte(0X6F12, 0X02A0); \
    write_cmos_sensor_twobyte(0X602A, 0X14BA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0096); \
    write_cmos_sensor_twobyte(0X602A, 0X0C22); \
    write_cmos_sensor_twobyte(0X6F12, 0X000A); \
    write_cmos_sensor_twobyte(0X602A, 0X15DC); \
    write_cmos_sensor_twobyte(0X6F12, 0X0001); \
    write_cmos_sensor_twobyte(0X602A, 0X0A94); \
    write_cmos_sensor_twobyte(0X6F12, 0X0800); \
    write_cmos_sensor_twobyte(0X602A, 0X0B62); \
    write_cmos_sensor_twobyte(0X6F12, 0X0126); \
    write_cmos_sensor_twobyte(0X602A, 0X8B2C); \
    write_cmos_sensor_twobyte(0X6F12, 0X000C); \
    write_cmos_sensor_twobyte(0X602A, 0X5150); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X602A, 0X0AE6); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BD0); \
    write_cmos_sensor_twobyte(0X602A, 0X29D8); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BD0); \
    write_cmos_sensor_twobyte(0X602A, 0X29E2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BD0); \
    write_cmos_sensor_twobyte(0X602A, 0X2958); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BD0); \
    write_cmos_sensor_twobyte(0X602A, 0X2998); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BD0); \
    write_cmos_sensor_twobyte(0X602A, 0X2962); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BD0); \
    write_cmos_sensor_twobyte(0X602A, 0X29A2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BD0); \
    write_cmos_sensor_twobyte(0X6028, 0X4000); \
    write_cmos_sensor_twobyte(0X6214, 0X79F0); \
    write_cmos_sensor_twobyte(0X6218, 0X79F0); \
    /* Stream On */                            \
    write_cmos_sensor(0x0100,0x01);            \
    mDELAY(10);                                \
}while(0)

/*****************************************************************************
 *
 * Description:
 * ------------
 *     mode 1 slim video setting
 *     $MIPI[Width:2688,Height:756,Format:Raw10,Lane:4,ErrorCheck:0,PolarityData:0,PolarityClock:0,Buffer:4,DataRate:2034,useEmbData:0]
 *     $MV1[MCLK:24,Width:2688,Height:756,Format:MIPI_Raw10,mipi_lane:4,mipi_datarate:2034,pvi_pclk_inverse:0]
 *
 ****************************************************************************/
#define _S5K2L7_MODE1_SLIM_VIDEO_ do{ \
    /* Stream Off */                               \
    write_cmos_sensor(0x0100,0x00);                \
    while(1)                                       \
    {                                              \
        if( read_cmos_sensor(0x0005)==0xFF) break; \
    }                                              \
    write_cmos_sensor_twobyte(0X6028, 0X4000); \
    write_cmos_sensor_twobyte(0X6214, 0X7970); \
    write_cmos_sensor_twobyte(0X6218, 0X7150); \
    write_cmos_sensor_twobyte(0X0344, 0X0000); \
    write_cmos_sensor_twobyte(0X0346, 0X016E); \
    write_cmos_sensor_twobyte(0X0348, 0X1F7F); \
    write_cmos_sensor_twobyte(0X034A, 0X0A61); \
    write_cmos_sensor_twobyte(0X034C, 0X0A80); \
    write_cmos_sensor_twobyte(0X034E, 0X02F4); \
    write_cmos_sensor_twobyte(0X0408, 0X0000); \
    write_cmos_sensor_twobyte(0X040A, 0X0004); \
    write_cmos_sensor_twobyte(0X0900, 0X0113); \
    write_cmos_sensor_twobyte(0X0380, 0X0001); \
    write_cmos_sensor_twobyte(0X0382, 0X0001); \
    write_cmos_sensor_twobyte(0X0384, 0X0001); \
    write_cmos_sensor_twobyte(0X0386, 0X0005); \
    write_cmos_sensor_twobyte(0X0400, 0X0000); \
    write_cmos_sensor_twobyte(0X0404, 0X0010); \
    write_cmos_sensor_twobyte(0X3060, 0X0103); \
    write_cmos_sensor_twobyte(0X0114, 0X0300); \
    write_cmos_sensor_twobyte(0X0110, 0X1002); \
    write_cmos_sensor_twobyte(0X0136, 0X1800); \
    write_cmos_sensor_twobyte(0X0304, 0X0006); \
    write_cmos_sensor_twobyte(0X0306, 0X01E0); \
    write_cmos_sensor_twobyte(0X0302, 0X0001); \
    write_cmos_sensor_twobyte(0X0300, 0X0004); \
    write_cmos_sensor_twobyte(0X030C, 0X0001); \
    write_cmos_sensor_twobyte(0X030E, 0X0004); \
    write_cmos_sensor_twobyte(0X0310, 0X0153); \
    write_cmos_sensor_twobyte(0X0312, 0X0000); \
    write_cmos_sensor_twobyte(0X030A, 0X0001); \
    write_cmos_sensor_twobyte(0X0308, 0X0008); \
    write_cmos_sensor_twobyte(0X0342, 0X27B0); \
    write_cmos_sensor_twobyte(0X0340, 0X0C4D); \
    write_cmos_sensor_twobyte(0X021E, 0X0000); \
    write_cmos_sensor_twobyte(0X3098, 0X0400); \
    write_cmos_sensor_twobyte(0X309A, 0X0002); \
    write_cmos_sensor_twobyte(0X30BC, 0X0031); \
    write_cmos_sensor_twobyte(0X30A8, 0X0000); \
    write_cmos_sensor_twobyte(0X30AC, 0X0000); \
    write_cmos_sensor_twobyte(0X30A0, 0X0000); \
    write_cmos_sensor_twobyte(0X30A4, 0X0000); \
    write_cmos_sensor_twobyte(0XF41E, 0X2180); \
    write_cmos_sensor_twobyte(0X6028, 0X2000); \
    write_cmos_sensor_twobyte(0X602A, 0X0990); \
    write_cmos_sensor_twobyte(0X6F12, 0X0020); \
    write_cmos_sensor_twobyte(0X602A, 0X0AF8); \
    write_cmos_sensor_twobyte(0X6F12, 0X000C); \
    write_cmos_sensor_twobyte(0X602A, 0X27A8); \
    write_cmos_sensor_twobyte(0X6F12, 0X0000); \
    write_cmos_sensor_twobyte(0X602A, 0X09AA); \
    write_cmos_sensor_twobyte(0X6F12, 0X1E7F); \
    write_cmos_sensor_twobyte(0X6F12, 0X1E7F); \
    write_cmos_sensor_twobyte(0X602A, 0X16B6); \
    write_cmos_sensor_twobyte(0X6F12, 0X122F); \
    write_cmos_sensor_twobyte(0X6F12, 0X0328); \
    write_cmos_sensor_twobyte(0X602A, 0X1688); \
    write_cmos_sensor_twobyte(0X6F12, 0X00A0); \
    write_cmos_sensor_twobyte(0X602A, 0X168C); \
    write_cmos_sensor_twobyte(0X6F12, 0X0028); \
    write_cmos_sensor_twobyte(0X6F12, 0X0034); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C16); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C16); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C1C); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C1C); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C22); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C22); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C28); \
    write_cmos_sensor_twobyte(0X6F12, 0X0C28); \
    write_cmos_sensor_twobyte(0X602A, 0X15F4); \
    write_cmos_sensor_twobyte(0X6F12, 0X0004); \
    write_cmos_sensor_twobyte(0X602A, 0X16BE); \
    write_cmos_sensor_twobyte(0X6F12, 0X06C0); \
    write_cmos_sensor_twobyte(0X6F12, 0X0101); \
    write_cmos_sensor_twobyte(0X602A, 0X0B16); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X602A, 0X11D2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0258); \
    write_cmos_sensor_twobyte(0X602A, 0X127A); \
    write_cmos_sensor_twobyte(0X6F12, 0X01B5); \
    write_cmos_sensor_twobyte(0X602A, 0X1282); \
    write_cmos_sensor_twobyte(0X6F12, 0X01BF); \
    write_cmos_sensor_twobyte(0X602A, 0X128A); \
    write_cmos_sensor_twobyte(0X6F12, 0X01C9); \
    write_cmos_sensor_twobyte(0X602A, 0X1292); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D3); \
    write_cmos_sensor_twobyte(0X602A, 0X129A); \
    write_cmos_sensor_twobyte(0X6F12, 0X01DD); \
    write_cmos_sensor_twobyte(0X602A, 0X12A2); \
    write_cmos_sensor_twobyte(0X6F12, 0X01E7); \
    write_cmos_sensor_twobyte(0X602A, 0X12AA); \
    write_cmos_sensor_twobyte(0X6F12, 0X01F1); \
    write_cmos_sensor_twobyte(0X602A, 0X12B2); \
    write_cmos_sensor_twobyte(0X6F12, 0X01FB); \
    write_cmos_sensor_twobyte(0X602A, 0X12BA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0205); \
    write_cmos_sensor_twobyte(0X602A, 0X12C2); \
    write_cmos_sensor_twobyte(0X6F12, 0X020F); \
    write_cmos_sensor_twobyte(0X602A, 0X12CA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0219); \
    write_cmos_sensor_twobyte(0X602A, 0X12D2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0223); \
    write_cmos_sensor_twobyte(0X602A, 0X12DA); \
    write_cmos_sensor_twobyte(0X6F12, 0X022D); \
    write_cmos_sensor_twobyte(0X602A, 0X12E2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0237); \
    write_cmos_sensor_twobyte(0X602A, 0X12EA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0241); \
    write_cmos_sensor_twobyte(0X602A, 0X12F2); \
    write_cmos_sensor_twobyte(0X6F12, 0X024B); \
    write_cmos_sensor_twobyte(0X602A, 0X12FA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0255); \
    write_cmos_sensor_twobyte(0X602A, 0X1302); \
    write_cmos_sensor_twobyte(0X6F12, 0X025F); \
    write_cmos_sensor_twobyte(0X602A, 0X130A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0269); \
    write_cmos_sensor_twobyte(0X602A, 0X1312); \
    write_cmos_sensor_twobyte(0X6F12, 0X0273); \
    write_cmos_sensor_twobyte(0X602A, 0X131A); \
    write_cmos_sensor_twobyte(0X6F12, 0X027D); \
    write_cmos_sensor_twobyte(0X602A, 0X1322); \
    write_cmos_sensor_twobyte(0X6F12, 0X0287); \
    write_cmos_sensor_twobyte(0X602A, 0X132A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0291); \
    write_cmos_sensor_twobyte(0X602A, 0X1332); \
    write_cmos_sensor_twobyte(0X6F12, 0X029B); \
    write_cmos_sensor_twobyte(0X602A, 0X133A); \
    write_cmos_sensor_twobyte(0X6F12, 0X01B0); \
    write_cmos_sensor_twobyte(0X602A, 0X1342); \
    write_cmos_sensor_twobyte(0X6F12, 0X01C4); \
    write_cmos_sensor_twobyte(0X602A, 0X134A); \
    write_cmos_sensor_twobyte(0X6F12, 0X01D8); \
    write_cmos_sensor_twobyte(0X602A, 0X1352); \
    write_cmos_sensor_twobyte(0X6F12, 0X01EC); \
    write_cmos_sensor_twobyte(0X602A, 0X135A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0200); \
    write_cmos_sensor_twobyte(0X602A, 0X1362); \
    write_cmos_sensor_twobyte(0X6F12, 0X0214); \
    write_cmos_sensor_twobyte(0X602A, 0X136A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0228); \
    write_cmos_sensor_twobyte(0X602A, 0X1372); \
    write_cmos_sensor_twobyte(0X6F12, 0X023C); \
    write_cmos_sensor_twobyte(0X602A, 0X137A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0250); \
    write_cmos_sensor_twobyte(0X602A, 0X1382); \
    write_cmos_sensor_twobyte(0X6F12, 0X0264); \
    write_cmos_sensor_twobyte(0X602A, 0X138A); \
    write_cmos_sensor_twobyte(0X6F12, 0X0278); \
    write_cmos_sensor_twobyte(0X602A, 0X1392); \
    write_cmos_sensor_twobyte(0X6F12, 0X028C); \
    write_cmos_sensor_twobyte(0X602A, 0X139A); \
    write_cmos_sensor_twobyte(0X6F12, 0X02A0); \
    write_cmos_sensor_twobyte(0X602A, 0X14BA); \
    write_cmos_sensor_twobyte(0X6F12, 0X0096); \
    write_cmos_sensor_twobyte(0X602A, 0X0C22); \
    write_cmos_sensor_twobyte(0X6F12, 0X000A); \
    write_cmos_sensor_twobyte(0X602A, 0X15DC); \
    write_cmos_sensor_twobyte(0X6F12, 0X0001); \
    write_cmos_sensor_twobyte(0X602A, 0X0A94); \
    write_cmos_sensor_twobyte(0X6F12, 0X0800); \
    write_cmos_sensor_twobyte(0X602A, 0X0B62); \
    write_cmos_sensor_twobyte(0X6F12, 0X0126); \
    write_cmos_sensor_twobyte(0X602A, 0X8B2C); \
    write_cmos_sensor_twobyte(0X6F12, 0X000C); \
    write_cmos_sensor_twobyte(0X602A, 0X5150); \
    write_cmos_sensor_twobyte(0X6F12, 0X0100); \
    write_cmos_sensor_twobyte(0X602A, 0X0AE6); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BD0); \
    write_cmos_sensor_twobyte(0X602A, 0X29D8); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BD0); \
    write_cmos_sensor_twobyte(0X602A, 0X29E2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BD0); \
    write_cmos_sensor_twobyte(0X602A, 0X2958); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BD0); \
    write_cmos_sensor_twobyte(0X602A, 0X2998); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BD0); \
    write_cmos_sensor_twobyte(0X602A, 0X2962); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BD0); \
    write_cmos_sensor_twobyte(0X602A, 0X29A2); \
    write_cmos_sensor_twobyte(0X6F12, 0X0BD0); \
    write_cmos_sensor_twobyte(0X6028, 0X4000); \
    write_cmos_sensor_twobyte(0X6214, 0X79F0); \
    write_cmos_sensor_twobyte(0X6218, 0X79F0); \
    /* Stream On */                            \
    write_cmos_sensor(0x0100,0x01);            \
    mDELAY(10);                                \
}while(0)

/*****************************************************************************
 *
 * Description:
 * ------------
 *     mode 1 cpature with WDR setting
 *     $MIPI[Width:8064,Height:3024,Format:Raw10,Lane:4,ErrorCheck:0,PolarityData:0,PolarityClock:0,Buffer:4,DataRate:2034,useEmbData:0]
 *     $MV1[MCLK:24,Width:8064,Height:3024,Format:MIPI_Raw10,mipi_lane:4,mipi_datarate:2034,pvi_pclk_inverse:0]
 *
 ****************************************************************************/
#define _S5K2L7_MODE1_CAPTURE_WDR_ do{ \
    /* Streaming  off */                           \
    write_cmos_sensor_twobyte(0x0100, 0x0000);     \
    while(1)                                       \
    {                                              \
        if( read_cmos_sensor(0x0005)==0xFF) break; \
    }                                              \
    write_cmos_sensor_twobyte(0X6028, 0X4000);\
    /* Clk on */                              \
    write_cmos_sensor_twobyte(0X6214, 0X7970);\
    write_cmos_sensor_twobyte(0X6218, 0X7150);\
    write_cmos_sensor_twobyte(0X3064, 0X0020);\
    /* Mode */                                \
    write_cmos_sensor_twobyte(0X6028, 0X4000);\
    write_cmos_sensor_twobyte(0X3064, 0X0020);\
    write_cmos_sensor_twobyte(0X0344, 0X0000);\
    write_cmos_sensor_twobyte(0X0346, 0X0000);\
    write_cmos_sensor_twobyte(0X0348, 0X1F7F);\
    write_cmos_sensor_twobyte(0X034A, 0X0BD7);\
    write_cmos_sensor_twobyte(0X034C, 0X1F80);\
    write_cmos_sensor_twobyte(0X034E, 0X0BD0);\
    write_cmos_sensor_twobyte(0X0408, 0X0000);\
    write_cmos_sensor_twobyte(0X040A, 0X0008);\
    write_cmos_sensor_twobyte(0X0900, 0X0011);\
    write_cmos_sensor_twobyte(0X0380, 0X0001);\
    write_cmos_sensor_twobyte(0X0382, 0X0001);\
    write_cmos_sensor_twobyte(0X0384, 0X0001);\
    write_cmos_sensor_twobyte(0X0386, 0X0001);\
    write_cmos_sensor_twobyte(0X0400, 0X0000);\
    write_cmos_sensor_twobyte(0X0404, 0X0010);\
    write_cmos_sensor_twobyte(0X3060, 0X0100);\
    write_cmos_sensor_twobyte(0X0114, 0X0300);\
    write_cmos_sensor_twobyte(0X0110, 0X1002);\
    write_cmos_sensor_twobyte(0X0136, 0X1800);\
    write_cmos_sensor_twobyte(0X0304, 0X0006);\
    write_cmos_sensor_twobyte(0X0306, 0X01E0);\
    write_cmos_sensor_twobyte(0X0302, 0X0001);\
    write_cmos_sensor_twobyte(0X0300, 0X0004);\
    write_cmos_sensor_twobyte(0X030C, 0X0001);\
    write_cmos_sensor_twobyte(0X030E, 0X0004);\
    write_cmos_sensor_twobyte(0X0310, 0X0153);\
    write_cmos_sensor_twobyte(0X0312, 0X0000);\
    write_cmos_sensor_twobyte(0X030A, 0X0001);\
    write_cmos_sensor_twobyte(0X0308, 0X0008);\
    write_cmos_sensor_twobyte(0X0342, 0X27E0);\
    write_cmos_sensor_twobyte(0X0340, 0X0C3E);\
    write_cmos_sensor_twobyte(0X021E, 0X0000);\
    write_cmos_sensor_twobyte(0X3098, 0X0400);\
    write_cmos_sensor_twobyte(0X309A, 0X0002);\
    write_cmos_sensor_twobyte(0X30BC, 0X0031);\
    write_cmos_sensor_twobyte(0XF41E, 0X2180);\
    write_cmos_sensor_twobyte(0X6028, 0X2000);\
    write_cmos_sensor_twobyte(0X602A, 0X0990);\
    write_cmos_sensor_twobyte(0X6F12, 0X0020);\
    write_cmos_sensor_twobyte(0X602A, 0X0B16);\
    write_cmos_sensor_twobyte(0X6F12, 0X0200);\
    write_cmos_sensor_twobyte(0X602A, 0X16B4);\
    write_cmos_sensor_twobyte(0X6F12, 0X54C2);\
    write_cmos_sensor_twobyte(0X6F12, 0X122F);\
    write_cmos_sensor_twobyte(0X6F12, 0X4328);\
    write_cmos_sensor_twobyte(0X602A, 0X1688);\
    write_cmos_sensor_twobyte(0X6F12, 0X00A2);\
    write_cmos_sensor_twobyte(0X602A, 0X168C);\
    write_cmos_sensor_twobyte(0X6F12, 0X0028);\
    write_cmos_sensor_twobyte(0X6F12, 0X0030);\
    write_cmos_sensor_twobyte(0X6F12, 0X0C18);\
    write_cmos_sensor_twobyte(0X6F12, 0X0C18);\
    write_cmos_sensor_twobyte(0X6F12, 0X0C28);\
    write_cmos_sensor_twobyte(0X6F12, 0X0C28);\
    write_cmos_sensor_twobyte(0X6F12, 0X0C20);\
    write_cmos_sensor_twobyte(0X6F12, 0X0C20);\
    write_cmos_sensor_twobyte(0X6F12, 0X0C30);\
    write_cmos_sensor_twobyte(0X6F12, 0X0C30);\
    write_cmos_sensor_twobyte(0X602A, 0X0A94);\
    write_cmos_sensor_twobyte(0X6F12, 0X2000);\
    write_cmos_sensor_twobyte(0X602A, 0X5840);\
    write_cmos_sensor_twobyte(0X6F12, 0X0000);\
    write_cmos_sensor_twobyte(0X602A, 0X16BC);\
    write_cmos_sensor_twobyte(0X6F12, 0X0100);\
    write_cmos_sensor_twobyte(0X6F12, 0X06C0);\
    write_cmos_sensor_twobyte(0X6F12, 0X0101);\
    write_cmos_sensor_twobyte(0X602A, 0X27A8);\
    write_cmos_sensor_twobyte(0X6F12, 0X0100);\
    write_cmos_sensor_twobyte(0X602A, 0X0B62);\
    write_cmos_sensor_twobyte(0X6F12, 0X0146);\
    write_cmos_sensor_twobyte(0X602A, 0X0B4A);\
    write_cmos_sensor_twobyte(0X6F12, 0X0018);\
    write_cmos_sensor_twobyte(0X602A, 0X0AEC);\
    write_cmos_sensor_twobyte(0X6F12, 0X0207);\
    write_cmos_sensor_twobyte(0X602A, 0X0AE6);\
    write_cmos_sensor_twobyte(0X6F12, 0X0BE0);\
    write_cmos_sensor_twobyte(0X602A, 0X29D8);\
    write_cmos_sensor_twobyte(0X6F12, 0X0BE0);\
    write_cmos_sensor_twobyte(0X602A, 0X29E2);\
    write_cmos_sensor_twobyte(0X6F12, 0X0BE0);\
    write_cmos_sensor_twobyte(0X602A, 0X0AE6);\
    write_cmos_sensor_twobyte(0X6F12, 0X0BE0);\
    write_cmos_sensor_twobyte(0X602A, 0X2958);\
    write_cmos_sensor_twobyte(0X6F12, 0X0BE0);\
    write_cmos_sensor_twobyte(0X602A, 0X2998);\
    write_cmos_sensor_twobyte(0X6F12, 0X0BE0);\
    write_cmos_sensor_twobyte(0X602A, 0X2962);\
    write_cmos_sensor_twobyte(0X6F12, 0X0BE0);\
    write_cmos_sensor_twobyte(0X602A, 0X29A2);\
    write_cmos_sensor_twobyte(0X6F12, 0X0BE0);\
    write_cmos_sensor_twobyte(0X6028, 0X4000);\
    write_cmos_sensor_twobyte(0X6214, 0X79F0);\
    write_cmos_sensor_twobyte(0X6218, 0X79F0);\
    /* Streaming  on */                       \
    write_cmos_sensor_twobyte(0x0100, 0x0100);\
}while(0)

#endif
