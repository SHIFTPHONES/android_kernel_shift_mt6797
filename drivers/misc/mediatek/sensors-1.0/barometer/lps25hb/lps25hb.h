
#ifndef LPS25HB_H
#define LPS25HB_H
	 
#include <linux/ioctl.h>
#include <barometer.h>
#include <hwmsensor.h>
#include <cust_baro.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE
	 
/* LPS25HB Register Map  (Please refer to LPS25HB Specifications) */
#define LPS25HB_REG_WHO_AM_I		0x0F
#define LPS25HB_REG_CTRL_REG1		0x20
//#define LPS25HB_REG_CTRL_REG2		0x11
#define LPS25HB_REG_CTRL_REG3		0x22
//#define LPS25HB_REG_REF_P_XL		0x15
//#define LPS25HB_REG_REF_P_L			0x16
//#define LPS25HB_REG_REF_P_H			0x17
//#define LPS25HB_REG_RES_CONF		0x1A
#define LPS25HB_REG_PRESS_OUT_XL	0x28
#define LPS25HB_REG_PRESS_OUT_L		0x29
#define LPS25HB_REG_PRESS_OUT_H		0x2A
#define LPS25HB_REG_TEMP_OUT_L		0x2B
#define LPS25HB_REG_TEMP_OUT_H		0x2C


#define LPS25HB_FIXED_DEVID			0xBD
	 
#define LPS25HB_BW_25HZ				0x40
#define LPS25HB_BW_13HZ				0x30
#define LPS25HB_BW_7HZ            	0x20
#define LPS25HB_BW_1HZ            	0x10
#define LPS25HB_BW_0HZ             	0x00
	 
#define LPS25HB_SUCCESS						0
#define LPS25HB_ERR_I2C						-1
#define LPS25HB_ERR_STATUS					-3
#define LPS25HB_ERR_SETUP_FAILURE			-4
#define LPS25HB_ERR_GETGSENSORDATA			-5
#define LPS25HB_ERR_IDENTIFICATION			-6

#define LPS25HB_BUFSIZE						256

#define LPS25HB_AXES_NUM        3
#define LPS25HB_DATA_LEN        6
#define LPS25HB_DEV_NAME        "LPS25HB"

#define CONFIG_LPS25HB_LOWPASS   /*apply low pass filter on output*/       

//#define CONFIG_LPS25HB_BARO_DRY

#define LPS25HB_BARO_DIV				100
#define LPS25HB_BARO_SENSITIVITY		4096
#define LPS25HB_TEMP_SENSITIVITY		100

/*----------------------------------------------------------------------------*/
typedef enum {
    ADX_TRC_FILTER  = 0x01,
    ADX_TRC_RAWDATA = 0x02,
    ADX_TRC_IOCTL   = 0x04,
    ADX_TRC_CALI    = 0X08,
    ADX_TRC_INFO    = 0X10,
} ADX_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
    u8  whole;
    u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
    s32 raw[C_MAX_FIR_LENGTH];
    int sum;
    int num;
    int idx;
};

/*----------------------------------------------------------------------------*/
struct lps25hb_baro {
    //struct i2c_client *client;
    //struct baro_hw *lps25hb_baro_hw;hanzening deleted
    struct baro_hw lps25hb_baro_hw;//hanzening added
    struct hwmsen_convert   cvt;
    
    /*misc*/
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
    atomic_t                filter;
    s32                     cali_sw;

    /*data*/
    s32                     offset;
    s32                     data;
    bool                    lps25hb_baro_power;
	int                     odr;
	int                     enabled;
#if defined(CONFIG_LPS25HB_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif 
};

struct lps25hb_data {
    struct i2c_client *client;
    struct lps25hb_baro lps25hb_baro_data;
    u8     reg_addr;
};

#define ST_TAG                  "[ST] "
#define ST_ERR(fmt, args...)    printk(KERN_ERR ST_TAG "%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define ST_LOG(fmt, args...)    printk(KERN_ERR ST_TAG "%s %d : "fmt, __FUNCTION__, __LINE__, ##args)

#if defined(DEBUG)
    #define ST_FUN(f)               printk(KERN_INFO ST_TAG "%s\n", __FUNCTION__)
	#define ST_DBG(fmt, args...)    printk(KERN_ERR ST_TAG fmt, ##args)
#else
	#define ST_FUN(f)
	#define ST_DBG(fmt, args...)
#endif

int lps25hb_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len);
int lps25hb_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len);

void dumpReg(struct lps25hb_data *obj);
int lps25hb_set_bdu(struct lps25hb_data *obj, bool flag);
int lps25hb_set_interrupt(struct lps25hb_data *obj, u8 intenable);

int lps25hb_baro_set_power_mode(struct lps25hb_baro *baro_obj, bool enable);
int lps25hb_baro_init(struct lps25hb_baro *baro_obj, int reset_cali);

//extern struct i2c_client *lps25hb_i2c_client;
extern struct lps25hb_data *obj_i2c_data;
extern int sensor_suspend;
extern struct baro_init_info lps25hb_baro_init_info;
extern int lps25hb_baro_init_flag;

#endif
