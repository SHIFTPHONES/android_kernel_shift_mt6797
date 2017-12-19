
/* ST  barometer LPS25HB driver
 *
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *version info:
 *    V 1.0.0  init version of barometer function  by jonny


 */
 
#include "lps25hb.h"

//pxs_add 20171214
//#define PRESS_OFFSET 0x44 //4.3hpa *16 = 68 = 0x44
  #define PRESS_OFFSET 0x49 //4.6hpa *16 = 73 = 0x49
  
  
struct lps25hb_data *obj_i2c_data = NULL;

static DEFINE_MUTEX(lps25hb_i2c_mutex);
static DEFINE_MUTEX(lps25hb_op_mutex);
struct baro_hw lps25hb_cust_hw;
int lps25hb_baro_init_flag =-1; // 0<==>OK -1 <==> fail
static struct i2c_driver lps25hb_i2c_driver;

/*For driver get cust info*/
struct baro_hw *lps25hb_get_cust_baro_hw(void)
{
    return &lps25hb_cust_hw;
}

static int lps25hb_baro_read_rawdata(struct lps25hb_baro *baro_obj, s32 *data)
{
    
    struct lps25hb_data *obj = container_of(baro_obj, struct lps25hb_data, lps25hb_baro_data);
	struct i2c_client *client = obj->client;
    s32 baro_data;
    u8 buf[LPS25HB_DATA_LEN] = {0};
    int err = 0;

    if (NULL == client)
    {
        err = -EINVAL;
    }
    
    else
    {
        if ((lps25hb_i2c_read_block(client, LPS25HB_REG_PRESS_OUT_XL, buf, 0x03))<0)
        {
            ST_ERR("read barometer sensor data register err!\n");
            return -1;
        }
     /*   
        if ((lps25hb_i2c_read_block(client, LPS25HB_REG_PRESS_OUT_L, buf+1, 0x01))<0)
        {
            ST_ERR("read barometer sensor data register err!\n");
            return -1;
        }
		
        if ((lps25hb_i2c_read_block(client, LPS25HB_REG_PRESS_OUT_H, buf+2, 0x01))<0)
        {
            ST_ERR("read barometer sensor data register err!\n");
            return -1;
        }
  */		
		baro_data = (s32)((((s8) buf[2]) << 16) | (buf[1] << 8) | (buf[0]));
        

        if (atomic_read(&baro_obj->trace) & ADX_TRC_RAWDATA)
        {
            ST_LOG("[%08X] => [%5d]\n", baro_data, baro_data);
        }
        
#ifdef CONFIG_LPS25HB_LOWPASS
		
		if (atomic_read(&baro_obj->filter)) {
			if (atomic_read(&baro_obj->fir_en) &&
				!atomic_read(&baro_obj->suspend)) {
				int idx, firlen = atomic_read(&baro_obj->firlen);
				if (baro_obj->fir.num < firlen) {
					baro_obj->fir.raw[baro_obj->fir.num] = baro_data;
					baro_obj->fir.sum += baro_data;
					if (atomic_read(&baro_obj->trace) &
						ADX_TRC_FILTER) {
						ST_LOG("add [%2d] [%5d] => [%5d]\n", baro_obj->fir.num,
						    baro_obj->fir.raw[baro_obj->fir.num],
						    baro_obj->fir.sum);
					}
					baro_obj->fir.num++;
					baro_obj->fir.idx++;
				} else {
					idx = baro_obj->fir.idx % firlen;
					baro_obj->fir.sum -= baro_obj->fir.raw[idx];
					baro_obj->fir.raw[idx]= baro_data;
					baro_obj->fir.sum += baro_data;
					baro_obj->fir.idx++;
					baro_data = baro_obj->fir.sum/firlen;
					if (atomic_read(&baro_obj->trace) &
						ADX_TRC_FILTER) {
						ST_LOG("add [%2d][%5d]=>[%5d]:[%5d]\n", idx, baro_obj->fir.raw[idx],
						    baro_obj->fir.sum, baro_data);
					}
				}
			}
		}
#endif
        *data = baro_data;
    }
    return err;
}
#if 0
static int lps25h_baro_read_rawdata_temperature(struct lps25hb_baro *baro_obj, s32 *data)
{
    struct lps25hb_data *obj = container_of(baro_obj, struct lps25hb_data, lps25hb_baro_data);
	struct i2c_client *client = obj->client;
    u8 buf[LPS25HB_DATA_LEN] = {0};
    int res = 0;


	if (NULL == client) 
    {
		ST_ERR("i2c client is null!\n");
		return LPS25HB_ERR_I2C;
	}

    if ((lps25hb_i2c_read_block(client, LPS25HB_REG_TEMP_OUT_L, buf, 0x01))<0)
    {
        ST_ERR("read temperature sensor data register err!\n");
        return LPS25HB_ERR_I2C;
    }
        
    if ((lps25hb_i2c_read_block(client, LPS25HB_REG_PRESS_OUT_H, buf+1, 0x01))<0)
    {
        ST_ERR("read temperature sensor data register err!\n");
        return LPS25HB_ERR_I2C;
    }
	
	*data = (s16) ((((s8) buf[1]) << 8) | (buf[0]));

	return res;
}
#endif
static int lps25hb_baro_set_odr(struct lps25hb_baro *baro_obj, u8 bwrate)
{
    struct lps25hb_data *obj = container_of(baro_obj, struct lps25hb_data, lps25hb_baro_data);
	struct i2c_client *client = obj->client;

    u8 databuf[10];
    u8 addr = LPS25HB_REG_CTRL_REG1;
    int res = 0;

    memset(databuf, 0, sizeof(u8)*10);
    
    if ((lps25hb_i2c_read_block(client, addr, databuf, 0x01))<0)
    {
        ST_ERR("read reg_ctl_reg1 register err!\n");
        return LPS25HB_ERR_I2C;
    }

    databuf[0] &= ~0x70;
    databuf[0] |= bwrate;

    res = lps25hb_i2c_write_block(client, LPS25HB_REG_CTRL_REG1, databuf, 0x1);

    if (res < 0)
    {
        return LPS25HB_ERR_I2C;
    }
    		
    return LPS25HB_SUCCESS;    
}

int lps25hb_baro_set_power_mode(struct lps25hb_baro *baro_obj, bool state)
{
    u8 databuf[10];   
    u8 addr = LPS25HB_REG_CTRL_REG1;
    int res = 0;

    if (state == baro_obj->lps25hb_baro_power)
    {
        ST_LOG("Sensor power status is newest!\n");
        return LPS25HB_SUCCESS;
    }

    memset(databuf, 0, sizeof(u8)*10);
    if (state == true)
    {
	    if ((lps25hb_i2c_read_block(obj_i2c_data ->client, addr, databuf, 0x01))<0)
  	  {
   	     ST_ERR("read reg_ctl_reg1 register err!\n");
     	     return LPS25HB_ERR_I2C;
 	   }
		
    	   databuf[0] |= 0x80;
 	   res = lps25hb_i2c_write_block( obj_i2c_data  ->client, LPS25HB_REG_CTRL_REG1, databuf, 0x1);

    	  if (res < 0)
    	{
        		return LPS25HB_ERR_I2C;
           }
    		
	  res = lps25hb_baro_set_odr(baro_obj, baro_obj->odr);
    }
    else if (state == false)
    {
	    if ((lps25hb_i2c_read_block(obj_i2c_data->client, addr, databuf, 0x01))<0)
  	  {
   	     ST_ERR("read reg_ctl_reg1 register err!\n");
     	     return LPS25HB_ERR_I2C;
 	   }
		
    	   databuf[0] &= ~0x80;
 	   res = lps25hb_i2c_write_block(obj_i2c_data->client, LPS25HB_REG_CTRL_REG1, databuf, 0x1);

    	  if (res < 0)
    	{
        		return LPS25HB_ERR_I2C;
           }
    }
	
    if (res < 0)
    {
        ST_LOG("set power mode failed!\n");
        return LPS25HB_ERR_I2C;
    }
    else if (atomic_read(&baro_obj->trace) & ADX_TRC_INFO)
    {
        ST_LOG("set power mode ok %d!\n", databuf[1]);
    }
	
	baro_obj->lps25hb_baro_power = state;
    return LPS25HB_SUCCESS;    
}
//pxs_add
static int lps25hb_baro_set_offset(struct lps25hb_baro *baro_obj, u16 press)
{
    struct lps25hb_data *obj = container_of(baro_obj, struct lps25hb_data, lps25hb_baro_data);
	struct i2c_client *client = obj->client;

    u8 databuf[10];
    u8 addr1 = 0x39;//pxs_add addr
	u8 addr2 = 0x3a;
    int res = 0;
	int err;
	
    memset(databuf, 0, sizeof(u8)*10); // 
    
    if ((lps25hb_i2c_read_block(client, addr1, databuf, 0x01))<0)
    {
        ST_ERR("read reg_ctl_reg1 register err!\n");
        return LPS25HB_ERR_I2C;
    }

    databuf[0] |= (u8)( press & 0x00ff);
    res = lps25hb_i2c_write_block(client, addr1, &databuf[0], 0x1);
    if (res < 0)
    {
        return LPS25HB_ERR_I2C;
    }

    databuf[1] |= (u8)( press & 0xff00)>>8;
	res = lps25hb_i2c_write_block(client, addr2, &databuf[1], 0x1);

    if (res < 0)
    {
        return LPS25HB_ERR_I2C;
    }
//pxs_add 20171214 read 
#if 1    
	err = lps25hb_i2c_read_block(client, addr1, databuf, 0x02);
    ST_LOG("pxs_LPS25HB 0x39 lsb = 0x%x\n", databuf[0]);

	//err = lps25hb_i2c_read_block(client, 0x3a, databuf, 0x01);
	ST_LOG("pxs_LPS25HB 0x3a msb= 0x%x\n", databuf[1]);
	press = ( databuf[1]<<8 ) |(databuf[0]);
	ST_LOG("pxs_LPS25HB press = 0x%x\n", press);
#endif		
			
    return LPS25HB_SUCCESS;    
}

int lps25hb_baro_init(struct lps25hb_baro *baro_obj, int reset_cali)
{
    struct lps25hb_data *obj = container_of(baro_obj, struct lps25hb_data, lps25hb_baro_data);
    int res = 0;

    baro_obj->odr = 0;
    res = lps25hb_baro_set_odr(baro_obj, LPS25HB_BW_0HZ);//power down
    if (res < 0)
    {
        ST_ERR("lps25hb_baro_init step 2!\n");
        return res;
    }

    res = lps25hb_set_bdu(obj,true);
	
      if (res < 0)
    {
        ST_ERR("lps25hb_baro_init step bdu!\n");
        return res;
    }
//pxs_add
	 lps25hb_baro_set_offset(baro_obj, PRESS_OFFSET );// 4.3 hpa *16  = 0x44
	 	
#ifdef CONFIG_LPS25HB_BARO_DRY
    res = lps25hb_set_interrupt(obj, true);        
    if (res < 0)
    {
        ST_ERR("lps25hb_baro_init step 4!\n");
        return res;
    }
#endif

#ifdef CONFIG_LPS25HB_LOWPASS
    memset(&baro_obj->fir, 0x00, sizeof(baro_obj->fir));
#endif

    return LPS25HB_SUCCESS;
}

static int lps25hb_bero_read_chip_name(struct lps25hb_baro *baro_obj, u8 *buf, int bufsize)
{
    u8 databuf[] = "LPS25HB Barometer";
	
	if (bufsize < sizeof(databuf))
    {
        sprintf(buf, "LPS25HB Barometer");
    }
	else 
    {
        ST_ERR("bufsize is too small\n");
        return LPS25HB_ERR_SETUP_FAILURE;
    }
    return LPS25HB_SUCCESS;
}

static int lps25hb_baro_read_chip_id(struct lps25hb_baro *baro_obj, u8 *data)
{
	struct lps25hb_data *obj = container_of(baro_obj, struct lps25hb_data, lps25hb_baro_data);
    struct i2c_client *client = obj->client;
    int res = 0;

    if (NULL == data)
    {
        return LPS25HB_ERR_SETUP_FAILURE;
    }
    
    if (NULL == client)
    {
        return LPS25HB_ERR_I2C;
    }

	res = lps25hb_i2c_read_block(client, LPS25HB_REG_WHO_AM_I, data, 0x01);
    if (res)
    {
		return LPS25HB_ERR_I2C;
    }
    return LPS25HB_SUCCESS;
}
#if 0
static int lps25hb_baro_read_data_temperature(struct lps25hb_baro *baro_obj, u8 *buf, int bufsize)
{
    struct lps25hb_data *obj = container_of(baro_obj, struct lps25hb_data, lps25hb_baro_data);
	struct i2c_client *client = obj->client;

    u8 databuf[20];
    int res = 0;
	s32 temp_data;

    memset(databuf, 0, sizeof(u8)*10);

    if (NULL == buf)
    {
        return LPS25HB_ERR_SETUP_FAILURE;
    }
    if (NULL == client)
    {
        *buf = 0;
        return LPS25HB_ERR_I2C;
    }

    if (atomic_read(&baro_obj->suspend))
    {
        ST_LOG("sensor in suspend read not data!\n");
        return 0;
    }

    if ((res = lps25h_baro_read_rawdata_temperature(baro_obj, &temp_data)))
    {        
        ST_ERR("I2C error: ret value=%d", res);
        return LPS25HB_ERR_I2C;
    }
    else
    {
        
        temp_data = temp_data / LPS25HB_TEMP_SENSITIVITY;        

        sprintf(buf, "%04x", temp_data);
        if (atomic_read(&baro_obj->trace) & ADX_TRC_FILTER)
        {
            ST_LOG("temperature data: %d!\n", temp_data);
            dumpReg(obj);
        }
    }
    return LPS25HB_SUCCESS;
}
#endif
static int lps25hb_baro_read_data(struct lps25hb_baro *baro_obj, u8 *buf, int bufsize)
{
    struct lps25hb_data *obj = container_of(baro_obj, struct lps25hb_data, lps25hb_baro_data);
    struct i2c_client *client = obj->client;

    u8 databuf[20];
    int baro;
    int res = 0;

    memset(databuf, 0, sizeof(u8)*10);

    if (NULL == buf)
    {
        return LPS25HB_ERR_SETUP_FAILURE;
    }
    if (NULL == client)
    {
        *buf = 0;
        return LPS25HB_ERR_I2C;
    }

    if (atomic_read(&baro_obj->suspend))
    {
        ST_LOG("sensor in suspend read not data!\n");
        return 0;
    }

    if ((res = lps25hb_baro_read_rawdata(baro_obj, &baro_obj->data)))
    {        
        ST_ERR("I2C error: ret value=%d", res);
        return LPS25HB_ERR_I2C;
    }
    else
    {
        
        baro = baro_obj->data;
        baro = baro * LPS25HB_BARO_DIV / LPS25HB_BARO_SENSITIVITY;        

        sprintf(buf, "%04x", baro);
        if (atomic_read(&baro_obj->trace) & ADX_TRC_FILTER)
        {
            ST_LOG("barometer data: %d!\n", baro);
            dumpReg(obj);
        }
    }
    return LPS25HB_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int lps25hb_baro_read_rawdata_string(struct lps25hb_baro *baro_obj, u8 *buf)
{
    struct lps25hb_data *obj = container_of(baro_obj, struct lps25hb_data, lps25hb_baro_data);
	struct i2c_client *client = obj->client;
    int res = 0;

    if (!buf || !client)
    {
        return EINVAL;
    }
    
	res = lps25hb_baro_read_rawdata(baro_obj, &baro_obj->data);
    if (res)
    {        
        ST_ERR("I2C error: ret value=%d", res);
        return LPS25HB_ERR_I2C;
    }
    else
    {
        sprintf(buf, "%04x", baro_obj->data);
    }
    
    return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t lps25hb_attr_baro_show_chipinfo_value(struct device_driver *ddri, char *buf)
{
    struct lps25hb_data *obj = obj_i2c_data;
	struct lps25hb_baro *baro_obj = &obj->lps25hb_baro_data;
    u8 strbuf[LPS25HB_BUFSIZE];
	int res;
    if (NULL == obj->client)
    {
        ST_ERR("i2c client is null!\n");
        return snprintf(buf, PAGE_SIZE, "i2c client is null!\n");
    }

    res = lps25hb_bero_read_chip_name(baro_obj, strbuf, LPS25HB_BUFSIZE);
	if (res)
	{
		return snprintf(buf, PAGE_SIZE, "get chip info error\n");
	}
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t lps25hb_attr_baro_show_chipid_value(struct device_driver *ddri, char *buf)
{
    struct lps25hb_data *obj = obj_i2c_data;
	struct lps25hb_baro *baro_obj = &obj->lps25hb_baro_data;
    u8 chipid = 0x00;
	int res;
    if (NULL == obj->client)
    {
        ST_ERR("i2c client is null!\n");
        return snprintf(buf, PAGE_SIZE, "i2c client is null!\n");
    }

	res = lps25hb_baro_read_chip_id(baro_obj, &chipid);
	if (res)
    {
        return snprintf(buf, PAGE_SIZE, "read chip id error!\n");
    }
	
    return snprintf(buf, PAGE_SIZE, "0x%x\n", chipid);
}
/*----------------------------------------------------------------------------*/
static ssize_t lps25hb_attr_baro_show_sensordata_value(struct device_driver *ddri, char *buf)
{
    struct lps25hb_data *obj = obj_i2c_data;
    struct lps25hb_baro *baro_obj = &obj->lps25hb_baro_data;
    u8 strbuf[LPS25HB_BUFSIZE];
	int res;
    
    if (NULL == obj->client)
    {
        ST_ERR("i2c client is null!!\n");
        return snprintf(buf, PAGE_SIZE, "i2c client is null!\n");;
    }
    res = lps25hb_baro_read_data(baro_obj, strbuf, LPS25HB_BUFSIZE);
	if (res)
    {
        return snprintf(buf, PAGE_SIZE, "read chip id error!\n");
    }
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t lps25hb_attr_baro_show_rawdata_value(struct device_driver *ddri, char *buf)
{   
    struct lps25hb_data *obj = obj_i2c_data;
    struct lps25hb_baro *baro_obj = &obj->lps25hb_baro_data;
    u8 strbuf[LPS25HB_BUFSIZE];
    int res;
    if (NULL == obj->client)
    {
        ST_ERR("i2c client is null!!\n");
        return snprintf(buf, PAGE_SIZE, "read rawdata error !\n");
    }

    res = lps25hb_baro_read_rawdata_string(baro_obj, strbuf);
	if (res)
    {
        return snprintf(buf, PAGE_SIZE, "read chip id error!\n");
    }
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t lps25hb_attr_baro_show_power_status(struct device_driver *ddri, char *buf)
{
    struct lps25hb_data *obj = obj_i2c_data;
    struct i2c_client *client = obj->client;
    u8 data;
    int res;
    if (NULL == obj->client)
    {
        ST_ERR("i2c client is null!!\n");
        return snprintf(buf, PAGE_SIZE, "read power status error !\n");;
    }

    res = lps25hb_i2c_read_block(client, LPS25HB_REG_CTRL_REG1, &data, 0x01);
    if (res)
    {
        return snprintf(buf, PAGE_SIZE, "read chip id error!\n");
    }
    return snprintf(buf, PAGE_SIZE, "%x\n", data);
}
/*----------------------------------------------------------------------------*/
static ssize_t lps25hb_attr_baro_show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_LPS25HB_LOWPASS
    struct lps25hb_data *obj = obj_i2c_data;
    struct lps25hb_baro *baro_obj = &obj->lps25hb_baro_data; 
    if (atomic_read(&baro_obj->firlen))
    {
        int idx, len = atomic_read(&baro_obj->firlen);
        ST_LOG("len = %2d, idx = %2d\n", baro_obj->fir.num, baro_obj->fir.idx);

        for(idx = 0; idx < len; idx++)
        {
            ST_LOG("[%5d %5d %5d]\n", baro_obj->fir.raw[idx], baro_obj->fir.raw[idx], baro_obj->fir.raw[idx]);
        }
        
        ST_LOG("sum = [%5d %5d %5d]\n", baro_obj->fir.sum, baro_obj->fir.sum, baro_obj->fir.sum);
        ST_LOG("avg = [%5d %5d %5d]\n", baro_obj->fir.sum/len, baro_obj->fir.sum/len, baro_obj->fir.sum/len);
    }
    return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&baro_obj->firlen));
#else
    return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t lps25hb_attr_baro_store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_LPS25HB_LOWPASS
    struct lps25hb_data *obj = obj_i2c_data;
    struct lps25hb_baro *baro_obj = &obj->lps25hb_baro_data; 
    int firlen;

    if (1 != sscanf(buf, "%d", &firlen))
    {
        ST_ERR("invallid format\n");
    }
    else if (firlen > C_MAX_FIR_LENGTH)
    {
        ST_ERR("exceeds maximum filter length\n");
    }
    else
    { 
        atomic_set(&baro_obj->firlen, firlen);
        if (0 == firlen)
        {
            atomic_set(&baro_obj->fir_en, 0);
        }
        else
        {
            memset(&baro_obj->fir, 0x00, sizeof(baro_obj->fir));
            atomic_set(&baro_obj->fir_en, 1);
        }
    }
#endif    
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t lps25hb_attr_baro_show_trace_value(struct device_driver *ddri, char *buf)
{
    struct lps25hb_data *obj = obj_i2c_data;
    struct lps25hb_baro *baro_obj = &obj->lps25hb_baro_data; 

    ssize_t res;

    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&baro_obj->trace));     
    return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t lps25hb_attr_baro_store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lps25hb_data *obj = obj_i2c_data;
    struct lps25hb_baro *baro_obj = &obj->lps25hb_baro_data; 
    int trace;
    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    if (1 == sscanf(buf, "0x%x", &trace))
    {
        atomic_set(&baro_obj->trace, trace);
    }    
    else
    {
        ST_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
    }
    
    return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t lps25hb_attr_baro_show_status_value(struct device_driver *ddri, char *buf)
{    
    struct lps25hb_data *obj = obj_i2c_data;
    struct lps25hb_baro *baro_obj = &obj->lps25hb_baro_data; 
    ssize_t len = 0;
    
    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }    
    
    if (NULL != &baro_obj->lps25hb_baro_hw)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: i2c_num=%d, direction=%d, sensitivity = %d,(power_id=%d, power_vol=%d)\n", 
                baro_obj->lps25hb_baro_hw.i2c_num, baro_obj->lps25hb_baro_hw.direction, baro_obj->reso->sensitivity, baro_obj->lps25hb_baro_hw.power_id, baro_obj->lps25hb_baro_hw.power_vol);   
        dumpReg(obj);
    }
    else
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
    }
    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t lps25hb_attr_baro_show_chipinit_value(struct device_driver *ddri, char *buf)
{
    struct lps25hb_data *obj = obj_i2c_data;
    struct lps25hb_baro *baro_obj = &obj->lps25hb_baro_data;

    ssize_t res;

    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&baro_obj->trace)); 
    return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t lps25hb_attr_baro_store_chipinit_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lps25hb_data *obj = obj_i2c_data;
    struct lps25hb_baro *baro_obj = &obj->lps25hb_baro_data;

    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return count;
    }

    lps25hb_baro_init(baro_obj, 0);
    dumpReg(obj);

    return count;
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,             S_IRUGO, lps25hb_attr_baro_show_chipinfo_value,      NULL);
static DRIVER_ATTR(chipid,               S_IRUGO, lps25hb_attr_baro_show_chipid_value,        NULL);
static DRIVER_ATTR(rawdata,              S_IRUGO, lps25hb_attr_baro_show_rawdata_value,       NULL);
static DRIVER_ATTR(sensordata,           S_IRUGO, lps25hb_attr_baro_show_sensordata_value,    NULL);
static DRIVER_ATTR(power,                S_IRUGO, lps25hb_attr_baro_show_power_status,        NULL);
static DRIVER_ATTR(firlen,     S_IWUSR | S_IRUGO, lps25hb_attr_baro_show_firlen_value,        lps25hb_attr_baro_store_firlen_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, lps25hb_attr_baro_show_trace_value,         lps25hb_attr_baro_store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, lps25hb_attr_baro_show_status_value,        NULL);
static DRIVER_ATTR(chipinit,   S_IWUSR | S_IRUGO, lps25hb_attr_baro_show_chipinit_value,      lps25hb_attr_baro_store_chipinit_value);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *lps25hb_attr_baro_list[] = {
    &driver_attr_chipinfo,     /*chip information*/
    &driver_attr_chipid,       /*chip id*/
    &driver_attr_sensordata,   /*dump sensor data*/
    &driver_attr_rawdata,      /*dump sensor raw data*/
    &driver_attr_power,        /*show power reg*/
    &driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
    &driver_attr_trace,        /*trace log*/
    &driver_attr_status,
    &driver_attr_chipinit,

};
/*----------------------------------------------------------------------------*/
int lps25hb_baro_create_attr(struct device_driver *driver) 
{
    int idx, err = 0;
    int num = (int)(sizeof(lps25hb_attr_baro_list)/sizeof(lps25hb_attr_baro_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if ((err = driver_create_file(driver, lps25hb_attr_baro_list[idx])))
        {            
            ST_ERR("driver_create_file (%s) = %d\n", lps25hb_attr_baro_list[idx]->attr.name, err);
            break;
        }
    }    
    return err;
}
/*----------------------------------------------------------------------------*/
int lps25hb_baro_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(lps25hb_attr_baro_list)/sizeof(lps25hb_attr_baro_list[0]));

    if (driver == NULL)
    {
        return -EINVAL;
    }
    

    for(idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, lps25hb_attr_baro_list[idx]);
    }
    

    return err;
}

/*----------------------------------------------------------------------------*/

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int lps25hb_baro_open_report_data_intf(int open)
{
    //should queuq work to report event if  is_report_input_direct=true
    return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int lps25hb_baro_enable_nodata_intf(int en)
{
    struct lps25hb_data *obj = obj_i2c_data;
    struct lps25hb_baro *baro_obj = &obj->lps25hb_baro_data;
    int res =0;
    bool power = false;
    
    if (1==en)
    {
        power = true;
    }
    if (0==en)
    {
        power = false;
    }
	baro_obj->enabled = en;
    res = lps25hb_baro_set_power_mode(baro_obj, power);
    if (res != LPS25HB_SUCCESS)
    {
        ST_ERR("lps25hb_baro_set_power_mode fail!\n");
        return -1;
    }
    ST_LOG("lps25hb_baro_enable_nodata_intf OK!\n");
    return 0;
}

static int lps25hb_baro_set_delay_intf(u64 ns)
{
    struct lps25hb_data *obj = obj_i2c_data;
    struct lps25hb_baro *baro_obj = &obj->lps25hb_baro_data;

    int value =0;
    int sample_delay=0;
    int err;
	
    value = (int)ns/1000/1000;
    if (value <= 15)
    {
        sample_delay = LPS25HB_BW_25HZ;
    }
    else
    {
        sample_delay = LPS25HB_BW_13HZ;
    }

	baro_obj->odr = sample_delay;
	err = lps25hb_baro_set_odr(baro_obj, baro_obj->odr);
    if (err != LPS25HB_SUCCESS ) //0x2C->BW=100Hz
    {
        ST_ERR("Set delay parameter error!\n");
    }

    if (value >= 50)
    {
        atomic_set(&baro_obj->filter, 0);
    }
    else
    {                    
        baro_obj->fir.num = 0;
        baro_obj->fir.idx = 0;
        baro_obj->fir.sum = 0;
        baro_obj->fir.sum = 0;
        baro_obj->fir.sum = 0;
        atomic_set(&baro_obj->filter, 1);
    }
    
    ST_LOG("lps25hb_baro_set_delay_intf (%d)\n",value);
    return 0;
}
//hanzening added
static int lps25hb_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return lps25hb_baro_set_delay_intf(samplingPeriodNs);
}

static int lps25hb_flush(void)
{
	return baro_flush_report();
}
//add end
static int lps25hb_baro_get_data_intf(int* value, int* status)
{
    struct lps25hb_data *obj = obj_i2c_data;
    struct lps25hb_baro *baro_obj = &obj->lps25hb_baro_data;

    u8 buff[LPS25HB_BUFSIZE];
    lps25hb_baro_read_data(baro_obj, buff, LPS25HB_BUFSIZE);
 
    sscanf(buff, "%x", value); 
    *status = SENSOR_STATUS_ACCURACY_HIGH;

    return 0;
}
#if 0
static int lps25hb_baro_open(struct inode *inode, struct file *file)
{
    file->private_data = obj_i2c_data;

    if (file->private_data == NULL)
    {
        ST_ERR("null pointer!!\n");
        return -EINVAL;
    }
    return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int lps25hb_baro_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}
/*----------------------------------------------------------------------------*/
static long lps25hb_baro_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)

{
    struct lps25hb_data *obj = (struct lps25hb_data*)file->private_data;
	struct lps25hb_baro *baro_obj = &obj->lps25hb_baro_data;
    
	u8 strbuf[LPS25HB_BUFSIZE];
    void __user *data;
	u32 dat = 0;
    long err = 0;


    //ST_FUN(f);
    if (_IOC_DIR(cmd) & _IOC_READ)
    {
        err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    }
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
    {
        err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    }

    if (err)
    {
        ST_ERR("baroess error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
        return -EFAULT;
    }

    switch(cmd)
    {
        case BAROMETER_IOCTL_INIT:
            lps25hb_baro_init(baro_obj, 0);            
            break;

        case BAROMETER_IOCTL_READ_CHIPINFO:
            data = (void __user *) arg;
            if (data == NULL)
            {
                err = -EINVAL;
                break;      
            }
            
            lps25hb_bero_read_chip_name(baro_obj, strbuf, LPS25HB_BUFSIZE);
            if (copy_to_user(data, strbuf, strlen(strbuf)+1))
            {
                err = -EFAULT;
                break;
            }                 
            break;      

		case BAROMETER_GET_PRESS_DATA:
			data = (void __user *) arg;
			if (NULL == data) {
				err = -EINVAL;
				break;
			}

			lps25hb_baro_read_data(baro_obj, strbuf, LPS25HB_BUFSIZE);
			sscanf(strbuf, "%x", &dat);
			if (copy_to_user(data, &dat, sizeof(dat))) {
				err = -EFAULT;
				break;
			}
			break;

		case BAROMETER_GET_TEMP_DATA:
			data = (void __user *) arg;
			if (NULL == data) {
				err = -EINVAL;
				break;
			}
			//lps25h_get_temperature(client, strbuf, LPS25H_BUFSIZE);
			lps25hb_baro_read_data_temperature(baro_obj, strbuf, LPS25HB_BUFSIZE);
			sscanf(strbuf, "%x", &dat);
			if (copy_to_user(data, &dat, sizeof(dat))) {
				err = -EFAULT;
				break;
			}
			break;

        default:
            ST_ERR("unknown IOCTL: 0x%08x\n", cmd);
            err = -ENOIOCTLCMD;
            break;
    }

    return err;
}

#ifdef CONFIG_COMPAT
static long lps25hb_baro_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	ST_FUN();

	if (!file->f_op || !file->f_op->unlocked_ioctl) {
		ST_ERR("compat_ion_ioctl file has no f_op or no f_op->unlocked_ioctl.\n");
		return -ENOTTY;
	}

	switch (cmd) {
	case COMPAT_BAROMETER_IOCTL_INIT:
	case COMPAT_BAROMETER_IOCTL_READ_CHIPINFO:
	case COMPAT_BAROMETER_GET_PRESS_DATA:
	case COMPAT_BAROMETER_GET_TEMP_DATA: {
		ST_LOG("compat_ion_ioctl : BAROMETER_IOCTL_XXX command is 0x%x\n", cmd);
		return file->f_op->unlocked_ioctl(file, cmd,
			(unsigned long)compat_ptr(arg));
	}
	default:
		ST_ERR("compat_ion_ioctl : No such command!! 0x%x\n", cmd);
		return -ENOIOCTLCMD;
	}
}
#endif


static struct file_operations lps25hb_baro_fops = {
    .owner = THIS_MODULE,
    .open = lps25hb_baro_open,
    .release = lps25hb_baro_release,
    .unlocked_ioctl = lps25hb_baro_unlocked_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = lps25hb_baro_compat_ioctl,
#endif
};

static struct miscdevice lps25hb_baro_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "barometer",
    .fops = &lps25hb_baro_fops,
};
#endif
/*added*/
static int lps25hb_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
	int err = 0;

	err = lps25hb_baro_enable_nodata_intf(enabledisable == true ? 1 : 0);
	if (err) {
		printk("%s enable sensor failed!\n", __func__);
		return -1;
	}
	err = lps25hb_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		printk("%s enable set batch failed!\n", __func__);
		return -1;
	}
	return 0;
}
static int lps25hb_factory_get_data(int32_t *data)
{
	int err = 0, status = 0;

	err = lps25hb_baro_get_data_intf(data, &status);
	if (err < 0) {
		printk("%s get data fail\n", __func__);
		return -1;
	}
	return 0;
}
static int lps25hb_factory_get_raw_data(int32_t *data)
{
	return 0;
}
static int lps25hb_factory_enable_calibration(void)
{
	return 0;
}
static int lps25hb_factory_clear_cali(void)
{
	return 0;
}
static int lps25hb_factory_set_cali(int32_t offset)
{
	return 0;
}
static int lps25hb_factory_get_cali(int32_t *offset)
{
	return 0;
}
static int lps25hb_factory_do_self_test(void)
{
	return 0;
}

static struct baro_factory_fops lps25hb_factory_fops = {
	.enable_sensor = lps25hb_factory_enable_sensor,
	.get_data = lps25hb_factory_get_data,
	.get_raw_data = lps25hb_factory_get_raw_data,
	.enable_calibration = lps25hb_factory_enable_calibration,
	.clear_cali = lps25hb_factory_clear_cali,
	.set_cali = lps25hb_factory_set_cali,
	.get_cali = lps25hb_factory_get_cali,
	.do_self_test = lps25hb_factory_do_self_test,
};

static struct baro_factory_public lps25hb_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &lps25hb_factory_fops,
};

/*added end*/
/*----------------------------------------------------------------------------*/

static int lps25hb_baro_local_init(void)
{
    if(i2c_add_driver(&lps25hb_i2c_driver))
    {
        ST_ERR("add driver error\n");
        return -1;
    }
	if (-1 == lps25hb_baro_init_flag)
		return -1;

	/* pr_debug("fwq loccal init---\n"); */
	return 0;
}

/*----------------------------------------------------------------------------*/
static int lps25hb_baro_local_remove(void)
{
    ST_FUN(); 
    /* misc_deregister(&lps25hb_device); */
	i2c_del_driver(&lps25hb_i2c_driver);//
		
//    baro_factory_device_deregister(&lps25hb_factory_device);
//    lps25hb_baro_delete_attr(&(lps25hb_baro_init_info.platform_diver_addr->driver));
    return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
struct baro_init_info lps25hb_baro_init_info = {
        .name = "lps25hb",
        .init = lps25hb_baro_local_init,
        .uninit = lps25hb_baro_local_remove,
};



static const struct i2c_device_id lps25hb_i2c_id[] = {{LPS25HB_DEV_NAME,0},{}};

#ifdef CONFIG_OF
static const struct of_device_id lps25hb_of_match[] = {
    {.compatible = "mediatek,barometer"},
    {}
};
#endif





/*--------------------read function----------------------------------*/
int lps25hb_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
    u8 beg = addr;
    int err;
    struct i2c_msg msgs[2]={{0},{0}};
    
    mutex_lock(&lps25hb_i2c_mutex);

   if(len > 1)
   	beg = beg | 0x80;
   
    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len =1;
    msgs[0].buf = &beg;

    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = len;
    msgs[1].buf = data;
    
    if (!client)
    {
        mutex_unlock(&lps25hb_i2c_mutex);
        return -EINVAL;
    }
    else if (len > C_I2C_FIFO_SIZE) 
    {
        ST_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        mutex_unlock(&lps25hb_i2c_mutex);
        return -EINVAL;
    }
    err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
    
    if (err < 0) 
    {
        ST_ERR("i2c_transfer error: (%d %p %d) %d\n",addr, data, len, err);
        err = -EIO;
    } 
    else 
    {
        err = 0;
    }
    mutex_unlock(&lps25hb_i2c_mutex);
    return err;
}

int lps25hb_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{   /*because address also occupies one byte, the maximum length for write is 7 bytes*/
    int err, idx, num;
    u8 buf[C_I2C_FIFO_SIZE];
    err =0;
    mutex_lock(&lps25hb_i2c_mutex);
    if (!client)
    {
        mutex_unlock(&lps25hb_i2c_mutex);
        return -EINVAL;
    }
    else if (len >= C_I2C_FIFO_SIZE) 
    {        
        ST_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        mutex_unlock(&lps25hb_i2c_mutex);
        return -EINVAL;
    }    

    num = 0;

    if(len > 1)
         buf[num++] = addr | 0x80 ;
    else
         buf[num++] = addr;
	
    for (idx = 0; idx < len; idx++)
    {
        buf[num++] = data[idx];
    }

    err = i2c_master_send(client, buf, num);
    if (err < 0)
    {
        ST_ERR("send command error!!\n");
        mutex_unlock(&lps25hb_i2c_mutex);
        return -EFAULT;
    } 
    mutex_unlock(&lps25hb_i2c_mutex);
    return err; //if success will return transfer lenth
}
/*----------------------------------------------------------------------------*/

void dumpReg(struct lps25hb_data *obj)
{
    struct i2c_client *client = obj->client;

    int i=0;
    u8 addr = 0x10;
    u8 regdata=0;
    for(i=0; i<10; i++)
    {
        //dump all
        lps25hb_i2c_read_block(client,addr,&regdata,1);
        ST_LOG("Reg addr=%x regdata=%x\n",addr,regdata);
        addr++;
    }
}
/*--------------------ADXL power control function----------------------------------*/
static void lps25hb_chip_power(struct lps25hb_data *obj, unsigned int on) 
{
    return;
}

int lps25hb_set_bdu(struct lps25hb_data *obj, bool flag)
{
	struct i2c_client *client = obj->client;

    u8 databuf[2];
    u8 addr = LPS25HB_REG_CTRL_REG1;
    int res = 0;

    memset(databuf, 0, sizeof(u8)*2); 

    if((lps25hb_i2c_read_block(client, addr, databuf, 0x01))<0)
    {
        ST_ERR("read reg_ctl_reg1 register err!\n");
        return LPS25HB_ERR_I2C;
    }

   if( true ==flag)
        databuf[0] = databuf[0] | 0x04;
   else
        databuf[0] &= ~0x04;

    res = lps25hb_i2c_write_block(client, LPS25HB_REG_CTRL_REG1, databuf, 0x01);
    if(res < 0)
    {
        return LPS25HB_ERR_I2C;
    }
    
    return LPS25HB_SUCCESS;    
}

#ifdef CONFIG_LPS25HB_BARO_DRY
int lps25hb_set_interrupt(struct lps25hb_data *obj, u8 intenable)
{
	struct i2c_client *client = obj->client;

    u8 databuf[2];
    u8 addr = LPS25HB_REG_CTRL_REG3;
    int res = 0;

    memset(databuf, 0, sizeof(u8)*2); 

    if((lps25hb_i2c_read_block(client, addr, databuf, 0x01))<0)
    {
        ST_ERR("read reg_ctl_reg1 register err!\n");
        return LPS25HB_ERR_I2C;
    }

    databuf[0] = 0x00;

    res = lps25hb_i2c_write_block(client, LPS25HB_REG_CTRL_REG3, databuf, 0x01);
    if(res < 0)
    {
        return LPS25HB_ERR_I2C;
    }
    
    return LPS25HB_SUCCESS;    
}
#endif

int lps25hb_check_device_id(struct lps25hb_data *obj)
{
	struct i2c_client *client = obj->client;
	u8 databuf[2];
	int err;
	
	err = lps25hb_i2c_read_block(client, LPS25HB_REG_WHO_AM_I, databuf, 0x01);
	if (err < 0)
	{
		return LPS25HB_ERR_I2C;
	}
	
    ST_LOG("LPS25HB who am I = 0x%x\n", databuf[0]);
	if(databuf[0]!=LPS25HB_FIXED_DEVID)
	{
		return LPS25HB_ERR_IDENTIFICATION;
	}
	
	return LPS25HB_SUCCESS;
}

static ssize_t lps25hb_attr_i2c_show_reg_value(struct device_driver *ddri, char *buf)
{
    struct lps25hb_data *obj = obj_i2c_data;

    u8 reg_value;
    ssize_t res;
    int err;
	
    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    err = lps25hb_i2c_read_block(obj->client, obj->reg_addr, &reg_value, 0x01);
	if (err < 0)
	{
		res = snprintf(buf, PAGE_SIZE, "i2c read error!!\n");
        return res;
	}
	
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", reg_value); 
    return res;
}

static ssize_t lps25hb_attr_i2c_store_reg_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lps25hb_data *obj = obj_i2c_data;

    u8 reg_value;
	int res;
	
    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    if(1 == sscanf(buf, "0x%hhx", &reg_value))
    {
	    res = lps25hb_i2c_write_block(obj->client, obj->reg_addr, &reg_value, 0x01);
	    if(res < 0)
        {
            return LPS25HB_ERR_I2C;
        }
    }    
    else
    {
        ST_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
    }

    return count;
}

static ssize_t lps25hb_attr_baro_show_reg_addr(struct device_driver *ddri, char *buf)
{
    struct lps25hb_data *obj = obj_i2c_data;

    ssize_t res;

    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", obj->reg_addr); 
    return res;
}

static ssize_t lps25hb_attr_i2c_store_reg_addr(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lps25hb_data *obj = obj_i2c_data;
	u8 reg_addr;

    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    if(1 == sscanf(buf, "0x%hhx", &reg_addr))
    {
        obj->reg_addr = reg_addr;
    }    
    else
    {
        ST_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
    }

    return count;
}
static DRIVER_ATTR(reg_value, S_IWUSR | S_IRUGO, lps25hb_attr_i2c_show_reg_value, lps25hb_attr_i2c_store_reg_value);
static DRIVER_ATTR(reg_addr,  S_IWUSR | S_IRUGO, lps25hb_attr_baro_show_reg_addr,  lps25hb_attr_i2c_store_reg_addr);

static struct driver_attribute *lps25hb_attr_i2c_list[] = {
    &driver_attr_reg_value,
	&driver_attr_reg_addr,
};

int lps25hb_i2c_create_attr(struct device_driver *driver) 
{
    int idx, err = 0;
    int num = (int)(sizeof(lps25hb_attr_i2c_list)/sizeof(lps25hb_attr_i2c_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if((err = driver_create_file(driver, lps25hb_attr_i2c_list[idx])))
        {            
            ST_ERR("driver_create_file (%s) = %d\n", lps25hb_attr_i2c_list[idx]->attr.name, err);
            break;
        }
    }    
    return err;
}

int lps25hb_i2c_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(lps25hb_attr_i2c_list)/sizeof(lps25hb_attr_i2c_list[0]));

    if(driver == NULL)
    {
        return -EINVAL;
    }
    

    for(idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, lps25hb_attr_i2c_list[idx]);
    }
    

    return err;
}

/*----------------------------------------------------------------------------*/
#if 1
static int lps25hb_suspend(struct i2c_client *client, pm_message_t msg) 
{
    struct lps25hb_data *obj = i2c_get_clientdata(client);
    struct lps25hb_baro *baro_obj = &obj->lps25hb_baro_data;
    int err = 0;
	
    ST_FUN();
    
    if((msg.event == PM_EVENT_SUSPEND) && (baro_obj->enabled == 1))
    {   
        mutex_lock(&lps25hb_op_mutex);
        if(obj == NULL)
        {    
            mutex_unlock(&lps25hb_op_mutex);
            ST_ERR("null pointer!!\n");
            return -EINVAL;
        }

        err = lps25hb_baro_set_power_mode(baro_obj, false);		
        if(err)
        {
            ST_ERR("write power control fail!!\n");
            mutex_unlock(&lps25hb_op_mutex);
            return err;        
        }
        
        atomic_set(&baro_obj->suspend, 1);
		mutex_unlock(&lps25hb_op_mutex);
        lps25hb_chip_power(obj, 0);
    }
    
	ST_LOG("lps25hb i2c suspended\n");
    return err;
}
/*----------------------------------------------------------------------------*/
static int lps25hb_resume(struct i2c_client *client)
{
    struct lps25hb_data *obj = i2c_get_clientdata(client);
    struct lps25hb_baro *baro_obj = &obj->lps25hb_baro_data;
    int err;
	
    ST_FUN();
    lps25hb_chip_power(obj, 1);
	if (baro_obj->enabled == 1) 
    {
	    mutex_lock(&lps25hb_op_mutex);
	    if(obj == NULL)
	    {
		    mutex_unlock(&lps25hb_op_mutex);
		    ST_ERR("null pointer!!\n");
		    return -EINVAL;
	    }
    
        err = lps25hb_baro_set_power_mode(baro_obj, true);
        if(err)
        {
            mutex_unlock(&lps25hb_op_mutex);
            ST_ERR("initialize client fail!!\n");
            return err;        
        }
        atomic_set(&baro_obj->suspend, 0);
        mutex_unlock(&lps25hb_op_mutex);
    }
	ST_LOG("lps25hb i2c resumed\n");
    return 0;
}
#endif
/*----------------------------------------------------------------------------*/
static int lps25hb_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
    strcpy(info->type, LPS25HB_DEV_NAME);
    return 0;
}

/*----------------------------------------------------------------------------*/

static int lps25hb_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    	struct lps25hb_data *obj;
    int err = 0;
    int retry = 0;
	struct baro_control_path ctl = { 0 };
	struct baro_data_path data = { 0 };

	ST_FUN();
	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}
              printk("%s start.\n", __func__);
    //	memset(obj, 0, sizeof(struct lps25hb_data));
	err = get_baro_dts_func(client->dev.of_node, &obj->lps25hb_baro_data.lps25hb_baro_hw);
	if (err < 0) {
		printk("get cust_baro dts info fail\n");
		goto exit;
	}
    	obj_i2c_data = obj;
    	obj->client = client;
    	i2c_set_clientdata(client, obj);
	
	err = lps25hb_check_device_id(obj);
	if (err)
	{
        ST_ERR("check device error!\n");
		goto exit_check_device_failed;
	}
	
    for(retry = 0; retry < 3; retry++){
        if ((err = lps25hb_baro_init(&obj->lps25hb_baro_data, 1)))
        {
            ST_ERR("lps25hb_baro_device init cilent fail time: %d\n", retry);
            continue;
        }
    }
    if (err != 0)
        goto exit_init_failed;
			
	/* err = misc_register(&bmp_device); */

    err = baro_factory_device_register(&lps25hb_factory_device);
	if (err) {
		printk("baro_factory device register failed, err = %d\n", err);
		goto exit_misc_device_register_failed;
	}
	
    if ((err = lps25hb_baro_create_attr(&(lps25hb_baro_init_info.platform_diver_addr->driver))))
    {
        ST_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }
    ctl.is_use_common_factory = false;
    ctl.open_report_data= lps25hb_baro_open_report_data_intf;
    ctl.enable_nodata = lps25hb_baro_enable_nodata_intf;
    ctl.set_delay  = lps25hb_baro_set_delay_intf;
    ctl.batch = lps25hb_batch;
    ctl.flush = lps25hb_flush;
    ctl.is_report_input_direct = false;
//	ctl.is_support_batch = baro_obj->lps25hb_baro_hw.is_batch_supported;

    err = baro_register_control_path(&ctl);
    if (err)
    {
        ST_ERR("register baro control path err\n");
        goto exit_kfree;
    }

    data.get_data = lps25hb_baro_get_data_intf;
    data.vender_div = LPS25HB_BARO_DIV;
	
#if 1
    err = baro_register_data_path(&data);
    if (err) {
        ST_ERR("register baro data path err\n");
        goto exit_kfree;
    }
#endif
	
    ST_LOG("%s: OK\n", __func__);
    lps25hb_baro_init_flag = 0;    
    printk("%s normally end.\n", __func__);
    return 0;

exit_create_attr_failed:
    /*misc_deregister(&lps25hb_baro_device);*/
exit_misc_device_register_failed:
	baro_factory_device_register(&lps25hb_factory_device);
exit_init_failed:
exit_check_device_failed:
exit_kfree:
    kfree(obj);
exit:
	obj_i2c_data = NULL;
    ST_ERR("%s: err = %d\n", __func__, err);
    lps25hb_baro_init_flag = -1;        
    return lps25hb_baro_init_flag;


}
/*----------------------------------------------------------------------------*/
static int lps25hb_i2c_remove(struct i2c_client *client)
{

	lps25hb_i2c_delete_attr(&lps25hb_i2c_driver.driver);
    baro_factory_device_register(&lps25hb_factory_device);

//	goto exit_misc_device_register_failed;

	obj_i2c_data = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));
    return 0;
}
/*----------------------------------------------------------------------------*/
static struct i2c_driver lps25hb_i2c_driver = {
    .driver = {
        .name           = LPS25HB_DEV_NAME,
#ifdef CONFIG_OF
        .of_match_table = lps25hb_of_match,
#endif
    },
    .probe              = lps25hb_i2c_probe,
    .remove             = lps25hb_i2c_remove,
    .detect             = lps25hb_i2c_detect,
    .suspend            = lps25hb_suspend,
    .resume             = lps25hb_resume,
    .id_table           = lps25hb_i2c_id,
};

/*----------------------------------------------------------------------------*/
static int __init lps25hb_module_init(void)
{
    ST_FUN();
#if 0
    if(i2c_add_driver(&lps25hb_i2c_driver))
    {
        ST_ERR("add driver error\n");
        return -1;
    }
#else
	baro_driver_add(&lps25hb_baro_init_info);//pxs_add  
#endif	 
    return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit lps25hb_module_exit(void)
{
    i2c_del_driver(&lps25hb_i2c_driver);

    ST_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(lps25hb_module_init);
module_exit(lps25hb_module_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LPS25HB I2C driver");
