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
/*
 * Driver for CAM_CAL
 *
 *
 */

#ifndef CONFIG_MTK_I2C_EXTENSION
#define CONFIG_MTK_I2C_EXTENSION
#endif
#include <linux/i2c.h>
#undef CONFIG_MTK_I2C_EXTENSION
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "cat24c16.h"
/* #include <asm/system.h>  // for SMP */
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif


/* #define CAM_CALGETDLT_DEBUG */
#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
#define PFX "cat2416c"
#define CAM_CALINF(fmt, arg...)    pr_debug("[%s] " fmt, __func__, ##arg)
#define CAM_CALDB(fmt, arg...)    pr_debug("[%s] " fmt, __func__, ##arg)
#define CAM_CALERR(fmt, arg...)    pr_debug("[%s] " fmt, __func__, ##arg)
#else
#define CAM_CALINF(fmt, arg...)
#define CAM_CALDB(fmt, arg...)
#define CAM_CALERR(fmt, arg...)
#endif
#define PAGE_SIZE_ 256
#define BUFF_SIZE 8

static DEFINE_SPINLOCK(g_CAM_CALLock);/*for SMP*/
#define CAM_CAL_I2C_BUSNUM 2

#define CAM_CAL_DEV_MAJOR_NUMBER 226

/* CAM_CAL READ/WRITE ID */
#define CATC24C16_DEVICE_ID							0xA0
/*#define I2C_UNIT_SIZE                                  1 //in byte*/
/*#define OTP_START_ADDR                            0x0A04*/
/*#define OTP_SIZE                                      24*/

/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_ICS_REVISION 1 /* seanlin111208 */
/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "CAM_CAL_DRV"
#define CAM_CAL_I2C_GROUP_ID 0
/*******************************************************************************
*
********************************************************************************/
/*static struct i2c_board_info kd_cam_cal_dev __initdata = {
	I2C_BOARD_INFO(CAM_CAL_DRVNAME, 0xA0 >> 1)
};
*/
/* A0 for page0 A2 for page 2 and so on for 8 pages */

static struct i2c_client *g_pstI2Cclient;
//static int selective_read_region(u32 addr, u8 *data, u16 i2c_id, u32 size);


/* 81 is used for V4L driver */
/*static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER, 0);*/
/*static struct cdev *g_pCAM_CAL_CharDrv;*/
/* static spinlock_t g_CAM_CALLock; */
/* spin_lock(&g_CAM_CALLock); */
/* spin_unlock(&g_CAM_CALLock); */

/*static struct class *CAM_CAL_class;*/
/*static atomic_t g_CAM_CALatomic;*/
/* static DEFINE_SPINLOCK(kdcam_cal_drv_lock); */
/* spin_lock(&kdcam_cal_drv_lock); */
/* spin_unlock(&kdcam_cal_drv_lock); */


#define EEPROM_I2C_SPEED 100
/* #define LSCOTPDATASIZE 0x03c4 //964 */
/* static kal_uint8 lscotpdata[LSCOTPDATASIZE]; */
/*
static void kdSetI2CSpeed(u32 i2cSpeed)
{
#ifdef USE_I2C_MTK_EXT
	spin_lock(&g_CAM_CALLock);
	g_pstI2Cclient->timing = i2cSpeed;
	spin_unlock(&g_CAM_CALLock);
#endif

}
*/

#if 0
static int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId)
{
	int  i4RetValue = 0;

	spin_lock(&g_CAM_CALLock);
	g_pstI2Cclient->addr = (i2cId >> 1);
	g_pstI2Cclient->ext_flag = (g_pstI2Cclient->ext_flag) & (~I2C_DMA_FLAG);


	spin_unlock(&g_CAM_CALLock);
	i4RetValue = i2c_master_send(g_pstI2Cclient, a_pSendData, a_sizeSendData);
	if (i4RetValue != a_sizeSendData) {
		CAM_CALERR("I2C send failed!!, Addr = 0x%x\n", a_pSendData[0]);
		return -1;
	}
	i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_pRecvData, a_sizeRecvData);
	if (i4RetValue != a_sizeRecvData) {
		CAM_CALERR("I2C read failed!!\n");
		return -1;
	}
	return 0;
}


/*******************************************************************************
* iWriteReg
********************************************************************************/
#if 0
static int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId)
{
	int  i4RetValue = 0;
	int u4Index = 0;
	u8 *puDataInBytes = (u8 *)&a_u4Data;
	int retry = 3;

	char puSendCmd[6] = {(char)(a_u2Addr >> 8) , (char)(a_u2Addr & 0xFF) ,
			     0 , 0 , 0 , 0
			    };

	spin_lock(&g_CAM_CALLock);


	g_pstI2Cclient->addr = (i2cId >> 1);
	g_pstI2Cclient->ext_flag = (g_pstI2Cclient->ext_flag) & (~I2C_DMA_FLAG);

	spin_unlock(&g_CAM_CALLock);


	if (a_u4Bytes > 2) {
		CAM_CALERR(" exceed 2 bytes\n");
		return -1;
	}

	if (a_u4Data >> (a_u4Bytes << 3))
		CAM_CALERR(" warning!! some data is not sent!!\n");

	for (u4Index = 0; u4Index < a_u4Bytes; u4Index += 1)
		puSendCmd[(u4Index + 2)] = puDataInBytes[(a_u4Bytes - u4Index - 1)];
	do {
		i4RetValue = i2c_master_send(g_pstI2Cclient, puSendCmd, (a_u4Bytes + 2));

		if (i4RetValue != (a_u4Bytes + 2))
			CAM_CALERR(" I2C send failed addr = 0x%x, data = 0x%x !!\n", a_u2Addr, a_u4Data);
		else
			break;
		mdelay(5);
	} while ((retry--) > 0);
	return 0;
}
#endif

static bool selective_read_byte(u32 addr, u8 *data, u16 i2c_id)
{
	/* CAM_CALDB("selective_read_byte\n"); */

	u8 page = addr / PAGE_SIZE_; /* size of page was 256 */
	u8 offset = addr % PAGE_SIZE_;
	/*kdSetI2CSpeed(EEPROM_I2C_SPEED);*/

	if (iReadRegI2C(&offset, 1, (u8 *)data, 1, i2c_id + (page << 1)) < 0) {
		CAM_CALERR("fail selective_read_byte addr =0x%x data = 0x%x,page %d, offset 0x%x",
		 addr, *data, page, offset);
		return false;
	}
	/* CAM_CALDB("selective_read_byte addr =0x%x data = 0x%x,page %d, offset 0x%x", addr,
	*data,page,offset); */
	return true;
}

static int selective_read_region(u32 addr, u8 *data, u16 i2c_id, u32 size)
{
	/* u32 page = addr/PAGE_SIZE; // size of page was 256 */
	/* u32 offset = addr%PAGE_SIZE; */
	u8 *buff = data;
	u32 size_to_read = size;
	/* kdSetI2CSpeed(EEPROM_I2C_SPEED); */
	int ret = 0;

	while (size_to_read > 0) {
		if (selective_read_byte(addr, buff, i2c_id)) {
			addr += 1;
			buff += 1;
			size_to_read -= 1;
			ret += 1;
		} else {
			break;

		}
#if 0
		if (size_to_read > BUFF_SIZE) {
			CAM_CALDB("offset =%x size %d\n", offset, BUFF_SIZE);
			if (iReadRegI2C(&offset, 1, (u8 *)buff, BUFF_SIZE, i2c_id + (page << 1)) < 0)
				break;
			ret += BUFF_SIZE;
			buff += BUFF_SIZE;
			offset += BUFF_SIZE;
			size_to_read -= BUFF_SIZE;
			page += offset / PAGE_SIZE_;

		} else {
			CAM_CALDB("offset =%x size %d\n", offset, size_to_read);
			if (iReadRegI2C(&offset, 1, (u8 *)buff, (u16) size_to_read, i2c_id + (page << 1)) < 0)
				break;
			ret += size_to_read;
			size_to_read = 0;
		}
#endif
	}
	CAM_CALDB("selective_read_region addr =%x size %d data read = %d\n", addr, size, ret);
	return ret;
}

#endif 
//end1

/* Burst Write Data */
/*
static int iWriteData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char *pinputdata)
{
	CAM_CALDB("not implemented!");
	return 0;
}
*/

#if 1
//#if defined(CONFIG_SHIFT6M_PROJECT)  //pxs_add 20171024
//pxs_add

static int otp_flag=0;
#define S5K3P3OTP_DEVICE_MQ_ID 0xA0

int iReadCAM_CAL(u16 a_u2Addr, u8 *a_puBuff)
{
    int  i4RetValue = 0;    
    char puReadCmd[2] = {(char)(a_u2Addr>>8), (char)(a_u2Addr & 0xFF) };
        
    spin_lock(&g_CAM_CALLock); //for SMP
  
    g_pstI2Cclient->addr = S5K3P3OTP_DEVICE_MQ_ID>>1;
      
	//g_pstI2Cclient->timing=400;
    spin_unlock(&g_CAM_CALLock); // for SMP    
    
    i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 2);

    if (i4RetValue != 2)
    {
      
	    printk("[S5K3P3OTP-CAMERA SENSOR] I2C send failed, addr = 0x%x, data = 0x%x !! \n", a_u2Addr,  *a_puBuff );
        return -1;
    }

    i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, 1);
	  CAM_CALDB("[CAM_CAL][iReadCAM_CAL] Read 0x%x=0x%x \n", a_u2Addr, a_puBuff[0]);
    if (i4RetValue != 1)
    {
        printk("S5K3P3OTP-[CAM_CAL] I2C read data failed!! \n");
        return -1;
    } 
   


    return 0;
}


static u8 Read_AF_Otp(u16 address,u8 *iBuffer,unsigned int buffersize)
{	
	u8 readbuff, i;
	//unsigned int check_sum = 0;
 
	for(i=0;i<buffersize;i++)
		{
	     iReadCAM_CAL((address+i),&readbuff);
			*(iBuffer+i)=readbuff;
	      CAM_CALDB("read af data[%d]=0x%x\n",i,iBuffer[i]);
	}
	//Check_sum  add by ljj
#if 0
	xxx
	for(i=0;i<6;i++)
	{
		iReadCAM_CAL((0x781+i),&readbuff);
		CAM_CALDB("[ljj]read af data[%d]=0x%x\n",i,readbuff);
		check_sum += readbuff;
	}
	iReadCAM_CAL((0x781+6),&readbuff);	
	if(readbuff == (check_sum%256))
	{
		CAM_CALDB("[ljj][AF_info][Check_sum] OK! [0x%x]: %d == [check_sum]:%d  \n",0x781+6,readbuff,check_sum);
	}else{
		printk("[ljj][AF_info][Check_sum] Failed! [0x%x]: %d != [check_sum]:%d  \n",0x781+6,readbuff,check_sum);
	}
#endif
	//end ljj		  
	return 1;

}


static u8 Read_AWB_Otp(u16 address,u8 *iBuffer,unsigned int buffersize)
{
	u8 readbuff, i;
 	//unsigned int check_sum = 0;
	for(i=0;i<buffersize;i++)
		{
	    iReadCAM_CAL((address+i),&readbuff);
			*(iBuffer+i)=readbuff;
	    CAM_CALDB("read awb data[%d]=0x%x\n",i,iBuffer[i]);
	}
	//Check_sum  add by ljj
#if 0
	for(i=0;i<8;i++)
	{
		iReadCAM_CAL((0x11+i),&readbuff);
		CAM_CALDB("[ljj]read awb data[%d]=0x%x\n",i,readbuff);
		check_sum += readbuff;
	}
	iReadCAM_CAL((0x11+8),&readbuff);	
	if(readbuff == (check_sum%256))
	{
		CAM_CALDB("[ljj][AWB_info][Check_sum] OK! [0x%x]: %d == [check_sum]:%d  \n",0x11+8,readbuff,check_sum);
	}else{
		printk("[ljj][AWB_info][Check_sum] Failed! [0x%x]: %d != [check_sum]:%d  \n",0x11+8,readbuff,check_sum);
	}
#endif
	//end ljj	
	return 1;
}


 int Read_LSC_Otp(u16 address,unsigned char *iBuffer,unsigned int buffersize)
 {
	u8 readbuff;
 	int i = 0;
	//unsigned int check_sum = 0;
    CAM_CALDB("S5K3P3OTP-Read_LSC_Otp-66-otp_flag=%d\n",otp_flag);	
	for(i=0;i<buffersize;i++)
	{
	       iReadCAM_CAL((address+i),&readbuff);
		     *(iBuffer+i)=readbuff;
		     // OTPDataMQ[i]=readbuff;
			//  check_sum += readbuff;
	}
	//Check_sum  add by ljj
#if 0
	iReadCAM_CAL(address+buffersize,&readbuff);
	if(readbuff == (check_sum%256))
	{
		
		CAM_CALDB("[ljj][LSC_info][Check_sum] OK! [0x076d]: %d == [check_sum]:%d  \n",readbuff,check_sum);
	}else{
		
		printk("[ljj][LSC_info][Check_sum] Failed! [0x076d]: %d != [check_sum]:%d  \n",readbuff,check_sum);
	}
#endif
	//end ljj	
  	 
	 return 1;
}


 void ReadOtp(u16 address,u8 *iBuffer,unsigned int buffersize)
{
		u16 i = 0;
		u8 readbuff;
		int ret ;
#if 0
		if(buffersize==1)
		{
			ret= iReadCAM_CAL(address-1,&readbuff);
			CAM_CALDB("[ljj][Valid][CAM_CAL]address+i = 0x%x,readbuff = 0x%x\n",(address-1),readbuff );
		}
#endif
		//end ljj		
		//printk("[ljj]S5K3P3OTP-ReadOtp-[CAM_CAL]ENTER address:0x%x buffersize:%d\n ",address, buffersize);
		if (1)
		{
			for(i = 0; i<buffersize; i++)
			{				
				ret= iReadCAM_CAL(address+i,&readbuff);
				CAM_CALDB("[CAM_CAL]address+i = 0x%x,readbuff = 0x%x\n",(address+i),readbuff );
				*(iBuffer+i) =(unsigned char)readbuff;
			}
		}
		
		//Check_sum  add by ljj
#if 0
		for(i = 0; i < 8;i++)	//0x0001~0x0008  check sum
		{
			ret= iReadCAM_CAL(address+i,&readbuff);
			check_sum += readbuff;
			CAM_CALDB("[ljj][Check_sum]address+i = 0x%x,readbuff = 0x%x\n",(address-1),readbuff );
		}		
		iReadCAM_CAL(address+8,&readbuff);//check_sum 0x0009
		if(readbuff == (check_sum%256))
		{
			CAM_CALDB("[ljj][Module_info][Check_sum] OK! [0x0009]: %d == [check_sum]:%d  \n",readbuff,check_sum);
		}else{
			printk("[ljj][Module_info][Check_sum] Failed! [0x0009]: %d != [check_sum]:%d  \n",readbuff,check_sum);
		}
#endif
		//end ljj
}
 
//Burst Write Data
//static int iWriteData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
//{
	//return 0;
//}

//Burst Read Data  iReadData(0x00,932,OTPData);
 unsigned int cat24c16_iReadData(u16 ui4_offset, unsigned int  ui4_length, u8 *pinputdata)
{
    
    CAM_CALDB("S5K3P3OTP-iReadData-ui4_offset=%d, ui4_length=%d\n",ui4_offset,ui4_length);
    
	if(ui4_length ==1) // layout check
    {	
	    ReadOtp(ui4_offset, pinputdata, ui4_length);
	   	
    }
	else if(ui4_length ==4)// awb 
    {
		
	    Read_AWB_Otp(ui4_offset, pinputdata, ui4_length);
	   	
    }
	else if(ui4_length ==2)// af
    {
		
	    Read_AF_Otp(ui4_offset, pinputdata, ui4_length);
	   	
    }
    else{  //lsc
		Read_LSC_Otp(ui4_offset, pinputdata, ui4_length);

    }
   return 0;
}




unsigned int cat24c16_selective_read_region(struct i2c_client *client, unsigned int addr,
	unsigned char *data, unsigned int size)
{
	unsigned int ret;
	g_pstI2Cclient = client;
	otp_flag=0;
	ret=cat24c16_iReadData(addr,size,data);
	 
	 CAM_CALDB("cat24c16.c-cat24c16_selective_read_region-addr=%x,data=%p,size=%d,*data=%d,ret=%d\n",addr,data,size,*data,ret);
	 return 1;


}


#else  // orig_start


static int otp_flag=1;
#define S5K3P3OTP_DEVICE_MQ_ID 0xB0

extern u8 OTPDataMQ[];


int iReadCAM_CAL(u16 a_u2Addr, u8 *a_puBuff)
{
//modify by ljj

    int  i4RetValue = 0;
    
    char puReadCmd[2] = {(char)(a_u2Addr>>8), (char)(a_u2Addr & 0xFF) };
    
   
     
    spin_lock(&g_CAM_CALLock); //for SMP
  


    g_pstI2Cclient->addr = S5K3P3OTP_DEVICE_MQ_ID>>1;
      
	  //g_pstI2Cclient->timing=400;
    spin_unlock(&g_CAM_CALLock); // for SMP    
    
    i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 2);

    if (i4RetValue != 2)
    {
      
	      printk("[S5K3P3OTP-CAMERA SENSOR] I2C send failed, addr = 0x%x, data = 0x%x !! \n", a_u2Addr,  *a_puBuff );
        return -1;
    }

    i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, 1);
	  CAM_CALDB("[CAM_CAL][iReadCAM_CAL] Read 0x%x=0x%x \n", a_u2Addr, a_puBuff[0]);
    if (i4RetValue != 1)
    {
        printk("S5K3P3OTP-[CAM_CAL] I2C read data failed!! \n");
        return -1;
    } 
   


    return 0;
}


static u8 Read_AF_Otp(u16 address,u8 *iBuffer,unsigned int buffersize)
{	
	u8 readbuff, i;
	unsigned int check_sum = 0; 
 
	for(i=0;i<buffersize;i++)
		{
	     iReadCAM_CAL((address+i),&readbuff);
			*(iBuffer+i)=readbuff;
	      CAM_CALDB("read af data[%d]=0x%x\n",i,iBuffer[i]);
	}

#if 1   
//Check_sum  add by ljj
	for(i=0;i<4;i++)
	{
		iReadCAM_CAL((0x01B+i),&readbuff);
		CAM_CALDB("[ljj]read af data[%d]=0x%x\n",i,readbuff);
		check_sum += readbuff;
	}
	iReadCAM_CAL(0x1F,&readbuff);	
	if(readbuff == (check_sum%256))
	{
		CAM_CALDB("[ljj][AF_info][Check_sum] OK! [0x%x]: %d == [check_sum]:%d  \n",0x781+6,readbuff,check_sum);
	}else{
		printk("[ljj][AF_info][Check_sum] Failed! [0x%x]: %d != [check_sum]:%d  \n",0x781+6,readbuff,check_sum);
	}
//end ljj		

#endif 
  
	return 1;

}


static u8 Read_AWB_Otp(u16 address,u8 *iBuffer,unsigned int buffersize)
{
	u8 readbuff, i;
 	unsigned int check_sum = 0;  
 
	for(i=0;i<buffersize;i++)
		{
	    iReadCAM_CAL((address+i),&readbuff);
			*(iBuffer+i)=readbuff;
	    CAM_CALDB("read awb data[%d]=0x%x\n",i,iBuffer[i]);
	  }
	

	
#if 1
//Check_sum  add by ljj
	for(i=0;i<8;i++)
	{
		iReadCAM_CAL((0x12+i),&readbuff);
		CAM_CALDB("[ljj]read awb data[%d]=0x%x\n",i,readbuff);
		check_sum += readbuff;
	}
	iReadCAM_CAL((0x1A),&readbuff);	
	if(readbuff == (check_sum%256))
	{
		CAM_CALDB("[ljj][AWB_info][Check_sum] OK! [0x%x]: %d == [check_sum]:%d  \n",0x11+8,readbuff,check_sum);
	}else{
		printk("[ljj][AWB_info][Check_sum] Failed! [0x%x]: %d != [check_sum]:%d  \n",0x11+8,readbuff,check_sum);
	}
//end ljj	

#endif 

	return 1;

}


 int Read_LSC_Otp(u16 address,unsigned char *iBuffer,unsigned int buffersize)
 {

 u8 readbuff;
 
 int i = 0;
 unsigned int check_sum = 0;
 CAM_CALDB("S5K3P3OTP-Read_LSC_Otp-66-otp_flag=%d\n",otp_flag);
 if(otp_flag){

	for(i=0;i<buffersize;i++)
	{
	   iBuffer[i]=OTPDataMQ[i];
	  
		
	}
 }
else{
	
	for(i=0;i<buffersize;i++)
	{
	       iReadCAM_CAL((address+i),&readbuff);
		     *(iBuffer+i)=readbuff;
		     // OTPDataMQ[i]=readbuff;
		      check_sum += readbuff;
	}
//Check_sum  add by ljj
	iReadCAM_CAL(address+buffersize,&readbuff);
	if(readbuff == (check_sum%256))
	{
		
		CAM_CALDB("[ljj][LSC_info][Check_sum] OK! [0x076d]: %d == [check_sum]:%d  \n",readbuff,check_sum);
	}else{
		
		printk("[ljj][LSC_info][Check_sum] Failed! [0x076d]: %d != [check_sum]:%d  \n",readbuff,check_sum);
	}
//end ljj	

  }
	 
	 return 1;
 
}


 void ReadOtp(u16 address,u8 *iBuffer,unsigned int buffersize)
{
		u16 i = 0;
		u8 readbuff;
		int ret ;
		unsigned int check_sum = 0; /*..no need  checksum for quanpu..*/
//Check Valid  add by ljj
		if(buffersize==1)
		{
			ret= iReadCAM_CAL(0x0,&readbuff);
			CAM_CALDB("[ljj][Valid][CAM_CAL]address=:0x0,readbuff = 0x%x\n",readbuff );
		}
//end ljj		
		//printk("[ljj]S5K3P3OTP-ReadOtp-[CAM_CAL]ENTER address:0x%x buffersize:%d\n ",address, buffersize);
		if (1)
		{
			for(i = 0; i<buffersize; i++)
			{				
				ret= iReadCAM_CAL(address+i,&readbuff);
				CAM_CALDB("[CAM_CAL]address+i = 0x%x,readbuff = 0x%x\n",(address+i),readbuff );
				*(iBuffer+i) =(unsigned char)readbuff;
			}
		}
		
	
	#if 1
//Check_sum  add by ljj
		for(i = 1; i < 9;i++)	//0x0001~0x0008  check sum
		{
			ret= iReadCAM_CAL(i,&readbuff);
			check_sum += readbuff;
			CAM_CALDB("[ljj][Check_sum]address+i = 0x%x,readbuff = 0x%x\n",(address-1),readbuff );
		}		
		iReadCAM_CAL(0x09,&readbuff);//check_sum 0x0009
		if(readbuff == (check_sum%256))
		{
			CAM_CALDB("[ljj][Module_info][Check_sum] OK! [0x0009]: %d == [check_sum]:%d  \n",readbuff,check_sum);
		}else{
			printk("[ljj][Module_info][Check_sum] Failed! [0x0009]: %d != [check_sum]:%d  \n",readbuff,check_sum);
		}
//end ljj
#endif

}
//Burst Write Data
//static int iWriteData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
//{
	//return 0;
//}

//Burst Read Data  iReadData(0x00,932,OTPData);
 unsigned int cat24c16_iReadData(u16 ui4_offset, unsigned int  ui4_length, u8 *pinputdata)
{
  // int  i4RetValue = 0;
   
	 // g_module_id=GetModulesId();
   
    
   // CAM_CALDB("S5K3P3OTP-iReadData-ui4_offset=%d, ui4_length=%d\n",ui4_offset,ui4_length);
    
	if(ui4_length ==1) // layout check
    {	
	    ReadOtp(ui4_offset, pinputdata, ui4_length);
	   	
    }
	else if(ui4_length ==4)// awb 
    {
		
	    Read_AWB_Otp(ui4_offset, pinputdata, ui4_length);
	   	
    }
	else if(ui4_length ==2)// af
    {
		
	    Read_AF_Otp(ui4_offset, pinputdata, ui4_length);
	   	
    }
  else{  //lsc
		Read_LSC_Otp(ui4_offset, pinputdata, ui4_length);

   }

   return 0;
}




unsigned int cat24c16_selective_read_region(struct i2c_client *client, unsigned int addr,
	unsigned char *data, unsigned int size)
{
	unsigned int ret;
	g_pstI2Cclient = client;
	otp_flag=1;
	ret=cat24c16_iReadData(addr,size,data);
	 
	 CAM_CALDB("cat24c16.c-cat24c16_selective_read_region-addr=%x,data=%p,size=%d,*data=%d,ret=%d\n",addr,data,size,*data,ret);
	 return 1;


}
#endif  // orig_end


/*#define CAT24C16_DRIVER_ON 0*/
#ifdef CAT24C16_DRIVER_ON
#ifdef CONFIG_COMPAT
static int compat_put_cal_info_struct(
	COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
	stCAM_CAL_INFO_STRUCT __user *data)
{
	compat_uptr_t p;
	compat_uint_t i;
	int err;

	err = get_user(i, &data->u4Offset);
	err |= put_user(i, &data32->u4Offset);
	err |= get_user(i, &data->u4Length);
	err |= put_user(i, &data32->u4Length);
	/* Assume pointer is not change */
#if 1
	err |= get_user(p, &data->pu1Params);
	err |= put_user(p, &data32->pu1Params);
#endif
	return err;
}
static int compat_get_cal_info_struct(
	COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
	stCAM_CAL_INFO_STRUCT __user *data)
{
	compat_uptr_t p;
	compat_uint_t i;
	int err;

	err = get_user(i, &data32->u4Offset);
	err |= put_user(i, &data->u4Offset);
	err |= get_user(i, &data32->u4Length);
	err |= put_user(i, &data->u4Length);
	err |= get_user(p, &data32->pu1Params);
	err |= put_user(compat_ptr(p), &data->pu1Params);

	return err;
}

static long cat24c16_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret;
	COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
	stCAM_CAL_INFO_STRUCT __user *data;
	int err;
	CAM_CALDB("[CAMERA SENSOR] cat24c16_Ioctl_Compat,%p %p %x ioc size %d\n",
	filp->f_op , filp->f_op->unlocked_ioctl, cmd, _IOC_SIZE(cmd));

	if (!filp->f_op || !filp->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {

	case COMPAT_CAM_CALIOC_G_READ: {
		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_cal_info_struct(data32, data);
		if (err)
			return err;

		ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ, (unsigned long)data);
		err = compat_put_cal_info_struct(data32, data);


		if (err != 0)
			CAM_CALERR("[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
		return ret;
	}
	default:
		return -ENOIOCTLCMD;
	}
}


#endif


/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode *a_pstInode,
			 struct file *a_pstFile,
			 unsigned int a_u4Command,
			 unsigned long a_u4Param)
#else
static long CAM_CAL_Ioctl(
	struct file *file,
	unsigned int a_u4Command,
	unsigned long a_u4Param
)
#endif
{
	int i4RetValue = 0;
	u8 *pBuff = NULL;
	u8 *pu1Params = NULL;
	stCAM_CAL_INFO_STRUCT *ptempbuf;
	CAM_CALDB("[S24CAM_CAL] ioctl\n");

#ifdef CAM_CALGETDLT_DEBUG
	struct timeval ktv1, ktv2;
	unsigned long TimeIntervalUS;
#endif

	if (_IOC_NONE != _IOC_DIR(a_u4Command)) {
		pBuff = kmalloc(sizeof(stCAM_CAL_INFO_STRUCT), GFP_KERNEL);

		if (NULL == pBuff) {
			CAM_CALDB(" ioctl allocate mem failed\n");
			return -ENOMEM;
		}

		if (_IOC_WRITE & _IOC_DIR(a_u4Command)) {
			if (copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT))) {
				/* get input structure address */
				kfree(pBuff);
				CAM_CALDB("[S24CAM_CAL] ioctl copy from user failed\n");
				return -EFAULT;
			}
		}
	}

	ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
	pu1Params = kmalloc(ptempbuf->u4Length, GFP_KERNEL);
	if (NULL == pu1Params) {
		kfree(pBuff);
		CAM_CALDB("ioctl allocate mem failed\n");
		return -ENOMEM;
	}
	CAM_CALDB(" init Working buffer address 0x%p  command is 0x%x\n", pu1Params, a_u4Command);


	if (copy_from_user((u8 *)pu1Params , (u8 *)ptempbuf->pu1Params, ptempbuf->u4Length)) {
		kfree(pBuff);
		kfree(pu1Params);
		CAM_CALDB("[S24CAM_CAL] ioctl copy from user failed\n");
		return -EFAULT;
	}

	switch (a_u4Command) {
	case CAM_CALIOC_S_WRITE:
		CAM_CALDB("[S24CAM_CAL] Write CMD\n");
#ifdef CAM_CALGETDLT_DEBUG
		do_gettimeofday(&ktv1);
#endif
		i4RetValue = iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
#ifdef CAM_CALGETDLT_DEBUG
		do_gettimeofday(&ktv2);
		if (ktv2.tv_sec > ktv1.tv_sec)
			TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
		else
			TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
		CAM_CALDB("Write data %d bytes take %lu us\n", ptempbuf->u4Length, TimeIntervalUS);
#endif
		break;
	case CAM_CALIOC_G_READ:
		CAM_CALDB("[S24CAM_CAL] Read CMD\n");
#ifdef CAM_CALGETDLT_DEBUG
		do_gettimeofday(&ktv1);
#endif
		CAM_CALDB("[CAM_CAL] offset %d\n", ptempbuf->u4Offset);
		CAM_CALDB("[CAM_CAL] length %d\n", ptempbuf->u4Length);
		/* CAM_CALDB("[CAM_CAL] Before read Working buffer address 0x%p\n", pu1Params); */

		i4RetValue = selective_read_region(ptempbuf->u4Offset, pu1Params,
		CATC24C16_DEVICE_ID, ptempbuf->u4Length);
		CAM_CALDB("[S24CAM_CAL] After read Working buffer data  0x%x\n", *pu1Params);


#ifdef CAM_CALGETDLT_DEBUG
		do_gettimeofday(&ktv2);
		if (ktv2.tv_sec > ktv1.tv_sec)
			TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
		else
			TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
		CAM_CALDB("Read data %d bytes take %lu us\n", ptempbuf->u4Length, TimeIntervalUS);
#endif

		break;
	default:
		CAM_CALDB("[S24CAM_CAL] No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	if (_IOC_READ & _IOC_DIR(a_u4Command)) {
		/* copy data to user space buffer, keep other input paremeter unchange. */
		CAM_CALDB("[S24CAM_CAL] to user length %d\n", ptempbuf->u4Length);
		CAM_CALDB("[S24CAM_CAL] to user  Working buffer address 0x%p\n", pu1Params);
		if (copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pu1Params , ptempbuf->u4Length)) {
			kfree(pBuff);
			kfree(pu1Params);
			CAM_CALDB("[S24CAM_CAL] ioctl copy to user failed\n");
			return -EFAULT;
		}
	}

	kfree(pBuff);
	kfree(pu1Params);
	return i4RetValue;
}


static u32 g_u4Opened;
/* #define */
/* Main jobs: */
/* 1.check for device-specified errors, device not ready. */
/* 2.Initialize the device if it is opened for the first time. */
static int CAM_CAL_Open(struct inode *a_pstInode, struct file *a_pstFile)
{
	CAM_CALDB("[S24CAM_CAL] CAM_CAL_Open\n");
	spin_lock(&g_CAM_CALLock);
	if (g_u4Opened) {
		spin_unlock(&g_CAM_CALLock);
		CAM_CALDB("[S24CAM_CAL] Opened, return -EBUSY\n");
		return -EBUSY;
	} else {
		g_u4Opened = 1;
		atomic_set(&g_CAM_CALatomic, 0);
	}
	spin_unlock(&g_CAM_CALLock);
	mdelay(2);
	return 0;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
static int CAM_CAL_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	spin_lock(&g_CAM_CALLock);

	g_u4Opened = 0;

	atomic_set(&g_CAM_CALatomic, 0);

	spin_unlock(&g_CAM_CALLock);

	return 0;
}

static const struct file_operations g_stCAM_CAL_fops = {
	.owner = THIS_MODULE,
	.open = CAM_CAL_Open,
	.release = CAM_CAL_Release,
	/* .ioctl = CAM_CAL_Ioctl */
#ifdef CONFIG_COMPAT
	.compat_ioctl = cat24c16_Ioctl_Compat,
#endif
	.unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
static inline int RegisterCAM_CALCharDrv(void)
{
	struct device *CAM_CAL_device = NULL;

#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
	if (alloc_chrdev_region(&g_CAM_CALdevno, 0, 1, CAM_CAL_DRVNAME)) {
		CAM_CALDB("[S24CAM_CAL] Allocate device no failed\n");

		return -EAGAIN;
	}
#else
	if (register_chrdev_region(g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME)) {
		CAM_CALDB("[S24CAM_CAL] Register device no failed\n");

		return -EAGAIN;
	}
#endif

	/* Allocate driver */
	g_pCAM_CAL_CharDrv = cdev_alloc();

	if (NULL == g_pCAM_CAL_CharDrv) {
		unregister_chrdev_region(g_CAM_CALdevno, 1);

		CAM_CALDB("[S24CAM_CAL] Allocate mem for kobject failed\n");

		return -ENOMEM;
	}

	/* Attatch file operation. */
	cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

	g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

	/* Add to system */
	if (cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1)) {
		CAM_CALDB("[S24CAM_CAL] Attatch file operation failed\n");

		unregister_chrdev_region(g_CAM_CALdevno, 1);

		return -EAGAIN;
	}

	CAM_CAL_class = class_create(THIS_MODULE, "CAM_CALdrv");
	if (IS_ERR(CAM_CAL_class)) {
		int ret = PTR_ERR(CAM_CAL_class);
		CAM_CALDB("Unable to create class, err = %d\n", ret);
		return ret;
	}
	CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

	return 0;
}

static inline void UnregisterCAM_CALCharDrv(void)
{
	/* Release char driver */
	cdev_del(g_pCAM_CAL_CharDrv);

	unregister_chrdev_region(g_CAM_CALdevno, 1);

	device_destroy(CAM_CAL_class, g_CAM_CALdevno);
	class_destroy(CAM_CAL_class);
}


/* //////////////////////////////////////////////////////////////////// */
#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
#elif 0
static int CAM_CAL_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#else
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int CAM_CAL_i2c_remove(struct i2c_client *);

static const struct i2c_device_id CAM_CAL_i2c_id[] = {{CAM_CAL_DRVNAME, 0}, {} };



static struct i2c_driver CAM_CAL_i2c_driver = {
	.probe = CAM_CAL_i2c_probe,
	.remove = CAM_CAL_i2c_remove,
	/* .detect = CAM_CAL_i2c_detect, */
	.driver.name = CAM_CAL_DRVNAME,
	.id_table = CAM_CAL_i2c_id,
};

#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	strcpy(info->type, CAM_CAL_DRVNAME);
	return 0;
}
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i4RetValue = 0;
	CAM_CALDB("[S24CAM_CAL] Attach I2C\n");
	/* spin_lock_init(&g_CAM_CALLock); */

	/* get sensor i2c client */
	spin_lock(&g_CAM_CALLock); /* for SMP */
	g_pstI2Cclient = client;
	g_pstI2Cclient->addr = CATC24C16_DEVICE_ID >> 1;
	spin_unlock(&g_CAM_CALLock); /* for SMP */

	CAM_CALDB("[CAM_CAL] g_pstI2Cclient->addr = 0x%x\n", g_pstI2Cclient->addr);
	/* Register char driver */
	i4RetValue = RegisterCAM_CALCharDrv();

	if (i4RetValue) {
		CAM_CALDB("[S24CAM_CAL] register char device failed!\n");
		return i4RetValue;
	}


	CAM_CALDB("[S24CAM_CAL] Attached!!\n");
	return 0;
}

static int CAM_CAL_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static int CAM_CAL_probe(struct platform_device *pdev)
{
	return i2c_add_driver(&CAM_CAL_i2c_driver);
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
	i2c_del_driver(&CAM_CAL_i2c_driver);
	return 0;
}

/* platform structure */
static struct platform_driver g_stCAM_CAL_Driver = {
	.probe              = CAM_CAL_probe,
	.remove     = CAM_CAL_remove,
	.driver             = {
		.name   = CAM_CAL_DRVNAME,
		.owner  = THIS_MODULE,
	}
};


static struct platform_device g_stCAM_CAL_Device = {
	.name = CAM_CAL_DRVNAME,
	.id = 0,
	.dev = {
	}
};

static int __init CAM_CAL_i2C_init(void)
{
	i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1);
	if (platform_driver_register(&g_stCAM_CAL_Driver)) {
		CAM_CALDB("failed to register S24CAM_CAL driver\n");
		return -ENODEV;
	}

	if (platform_device_register(&g_stCAM_CAL_Device)) {
		CAM_CALDB("failed to register S24CAM_CAL driver, 2nd time\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit CAM_CAL_i2C_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_i2C_init);
module_exit(CAM_CAL_i2C_exit);

MODULE_DESCRIPTION("CAM_CAL driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");

#endif


