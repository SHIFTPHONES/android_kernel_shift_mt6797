#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>
//#include <linux/xlog.h>


/*===FEATURE SWITH===*/
 // #define FPTPDAFSUPPORT   //for pdaf switch
 // #define FANPENGTAO   //for debug log

/*===FEATURE SWITH===*/

/****************************Modify Following Strings for Debug****************************/
#define PFX "S5K3L8PDAF ZK"
#define LOG_INF_NEW(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
#define LOG_INF_LOD(format, args...)    pr_debug(PFX,"[%s] " format, __FUNCTION__, ##args)
#define LOG_INF LOG_INF_LOD
#define LOG_1 LOG_INF("S5K3L8,MIPI 4LANE ZK\n")
#define SENSORDB LOG_INF
/****************************   Modify end    *******************************************/

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
//extern int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId, u16 transfer_length);
extern int iMultiReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId, u8 number);


#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms)          mdelay(ms)

/**************  CONFIG BY SENSOR >>> ************/
#define EEPROM_WRITE_ID   0xa0
#define I2C_SPEED        100
#define MAX_OFFSET		    0xFFFF
#define DATA_SIZE         1404
#define START_ADDR        0X0763
BYTE S5K3L8_eeprom_data2[DATA_SIZE]= {0};

/**************  CONFIG BY SENSOR <<< ************/

static kal_uint16 read_cmos_sensor_byte(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };

    //kdSetI2CSpeed(I2C_SPEED); // Add this func to set i2c speed by each sensor
    iReadRegI2C(pu_send_cmd , 2, (u8*)&get_byte,1,EEPROM_WRITE_ID);
    return get_byte;
}

/*
static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

    kdSetI2CSpeed(I2C_SPEED); // Add this func to set i2c speed by each sensor
    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, EEPROM_WRITE_ID);
    return get_byte;
}


static void write_cmos_sensor_byte(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

    kdSetI2CSpeed(I2C_SPEED); // Add this func to set i2c speed by each sensor
    iWriteRegI2C(pu_send_cmd, 3, EEPROM_WRITE_ID);
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};

    kdSetI2CSpeed(I2C_SPEED); // Add this func to set i2c speed by each sensor
    iWriteRegI2C(pusendcmd , 4, EEPROM_WRITE_ID);
}
*/


static bool _read_eeprom(kal_uint16 addr, kal_uint32 size ){
  //continue read reg by byte:
  int i=0;
  int j=0; 
  for(; i<size; i++){
  	//add by jun 
  	
  	if((addr+i) >= 0x19F1){ // 0x19F1 - 0x1D7C ; 908 bytes
	//		j = (i+2);      // mcnx need to move 2 byte;
			j=i;            // zkyd no need to move 
//			printk("wujun-otp >>>>>>>= 0x19F1      addr = 0x%x  i=%d    j=%d\n",addr, i,j);
  		}
	else{
			j = i;
//			printk("wujun-otp <<<<<<< 0x19F1       addr = 0x%x  i=%d    j=%d\n",addr, i,j);
		}
	

//	printk("wujun-otp addr+j = 0x%x \n",(addr+j));
	S5K3L8_eeprom_data2[i] = read_cmos_sensor_byte(addr+j);
	
//    S5K3L8_eeprom_data2[i] = read_cmos_sensor_byte(addr+i);
//      printk("wujun-otp S5K3L8_read_eeprom addr = %d, \tvalue = 0x%x \n",i, S5K3L8_eeprom_data2[i]);
  }
  return true;
}

bool S5K3L8_read_eeprom_zk( kal_uint16 addr, BYTE* data, kal_uint32 size){
	addr = 0x1801; //START_ADDR
	size = DATA_SIZE;

	//LOG_INF("Read EEPROM, addr = 0x%x, size = 0d%d\n", addr, size);

	if(!_read_eeprom(addr, size)){
	//LOG_INF("error:read_eeprom fail!\n");
	return false;
	}

	memcpy(data, S5K3L8_eeprom_data2, size);
	return true;
}


