#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/slab.h>
#include <linux/types.h>

#define PFX "S5K3P9sx_pdafotp"
#define LOG_INF(fmt, args...)   pr_debug(PFX "[%s] " fmt, __func__, ##args)
#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);


#define USHORT             unsigned short
#define BYTE               unsigned char

#define S5K3P9SX_EEPROM_WRITE_ID   0xA0

BYTE S5K3P9sx_eeprom_data[2048]= {0};
kal_uint8 s5k3p9sx_dual_data[2048] = {0};

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,S5K3P9SX_EEPROM_WRITE_ID);
    return get_byte;
}

bool read_3P9sx_eeprom( kal_uint16 addr, BYTE* data, kal_uint32 size)
{
	kal_uint32 checksum = 0;
	int i;

	for(i = 0; i < 496; i++)
	{
		data[i] = read_cmos_sensor_8(0x772 + i);
		checksum += data[i];
	}

    if((((checksum >> 8) & 0xff) == read_cmos_sensor_8(0x0962)) && ((checksum & 0xff) == read_cmos_sensor_8(0x0963)))
		LOG_INF("PDAF1 Checksum OK\n");
	else
		LOG_INF("PDAF1 Checksum Failed!!!\n");

  checksum = 0;
  for(i = 0; i < 908; i++)
	{
		data[i + 496] = read_cmos_sensor_8(0x964 + i);
		checksum += data[i + 496];
	}

    if((((checksum >> 8) & 0xff) == read_cmos_sensor_8(0x0cf0)) && ((checksum & 0xff) == read_cmos_sensor_8(0x0cf1)))
		LOG_INF("PDAF2 Checksum OK\n");
	else
		LOG_INF("PDAF2 Checksum Failed!!!\n");

  return true;
}

void s5k3p9sx_get_otp_data(void)
{
	int i;
	kal_uint32 checksum = 0;

    if(0x01 != read_cmos_sensor_8(0x0000))
	{
        LOG_INF("OTP DATA Invalid!!!\n");
        return;
    }

//0x010b00ff
	S5K3P9sx_eeprom_data[0] = 0xff;
	S5K3P9sx_eeprom_data[1] = 0x00;
	S5K3P9sx_eeprom_data[2] = 0x0b;
	S5K3P9sx_eeprom_data[3] = 0x01;

	S5K3P9sx_eeprom_data[4] = 0x0b;  //bit0:awb, bit1:af, bit3:lsc

	//AF
	for(i = 0; i < 4; i++)
	{
		S5K3P9sx_eeprom_data[i + 6] = read_cmos_sensor_8(0x076c + i);
		checksum += S5K3P9sx_eeprom_data[i + 6];
	}

	if((((checksum >> 8) & 0xff) == read_cmos_sensor_8(0x0770)) && ((checksum & 0xff) == read_cmos_sensor_8(0x0771)))
	{
		LOG_INF("AF Checksum OK\n");
		S5K3P9sx_eeprom_data[6] ^= S5K3P9sx_eeprom_data[7];
		S5K3P9sx_eeprom_data[7] ^= S5K3P9sx_eeprom_data[6];
		S5K3P9sx_eeprom_data[6] ^= S5K3P9sx_eeprom_data[7];
		S5K3P9sx_eeprom_data[8] ^= S5K3P9sx_eeprom_data[9];
		S5K3P9sx_eeprom_data[9] ^= S5K3P9sx_eeprom_data[8];
		S5K3P9sx_eeprom_data[8] ^= S5K3P9sx_eeprom_data[9];

        LOG_INF("AFInf = %d, AFMarco = %d\n", S5K3P9sx_eeprom_data[7] << 8 | S5K3P9sx_eeprom_data[6], S5K3P9sx_eeprom_data[9] << 8 | S5K3P9sx_eeprom_data[8]);
	}
	else
	{
		LOG_INF("AF Checksum Failed!!!\n");
		S5K3P9sx_eeprom_data[4] = S5K3P9sx_eeprom_data[4] & (~0x02);
	}

//AWB
    checksum = 0;

    for(i = 0; i < 8; i++)
    {
      S5K3P9sx_eeprom_data[i + 10] = read_cmos_sensor_8(0x0008 + i);
      checksum += S5K3P9sx_eeprom_data[i + 10];
    }

    if((((checksum >> 8) & 0xff) == read_cmos_sensor_8(0x0010)) && ((checksum & 0xff) == read_cmos_sensor_8(0x0011)))
    {
      LOG_INF("AWB OTP Checksum OK\n");
      LOG_INF("Unit_R = 0x%x, Unit_Gr = 0x%x, Unit_Gb = 0x%x, Unit_B = 0x%x\n", S5K3P9sx_eeprom_data[10], S5K3P9sx_eeprom_data[11], S5K3P9sx_eeprom_data[12], S5K3P9sx_eeprom_data[13]);
      LOG_INF("Golden_R = 0x%x, Golden_Gr = 0x%x, Golden_Gb = 0x%x, Golden_B = 0x%x\n", S5K3P9sx_eeprom_data[14], S5K3P9sx_eeprom_data[15], S5K3P9sx_eeprom_data[16], S5K3P9sx_eeprom_data[17]);
    }
    else
    {
      LOG_INF("AWB OTP Checksum Failed!!!\n");
      S5K3P9sx_eeprom_data[4] = S5K3P9sx_eeprom_data[4] & (~0x01);
    }

	//LSC
	checksum = 0;
	for(i = 0; i < 1868; i++)
	{
		S5K3P9sx_eeprom_data[20 + i] = read_cmos_sensor_8(0x001e + i);
		checksum += S5K3P9sx_eeprom_data[20 + i];
	}

    if((((checksum >> 8) & 0xff) == read_cmos_sensor_8(0x076a)) && ((checksum & 0xff) == read_cmos_sensor_8(0x076b)))
		LOG_INF("LSC Checksum OK\n");
	else
	{
		LOG_INF("LSC Checksum Failed!!!\n");
		S5K3P9sx_eeprom_data[4] = S5K3P9sx_eeprom_data[4] & (~0x08);
	}

//Dual OTP
    checksum = 0;
    for(i = 0; i < 2048; i++)
    {
        s5k3p9sx_dual_data[i] = read_cmos_sensor_8(0x0d00 + i);
        checksum += s5k3p9sx_dual_data[i];
    }

    if((((checksum >> 8) & 0xff) == read_cmos_sensor_8(0x1500)) && ((checksum & 0xff) == read_cmos_sensor_8(0x1501)))
		LOG_INF("Dual Data Checksum OK\n");
	else
        LOG_INF("Dual Data Checksum Failed!!!\n");

}
