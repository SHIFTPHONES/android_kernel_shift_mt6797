/*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*/
#ifndef sx9310_H
#define sx9310_H

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#if 0
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#endif

/*
*  I2C Registers
*/
//-Interrupt and status
#define sx9310_IRQSTAT_REG    	0x00
#define sx9310_STAT0_REG    	0x01
#define sx9310_STAT1_REG    	0x02
#define sx9310_IRQ_ENABLE_REG	0x03
#define sx9310_IRQFUNC_REG		0x04
	
#define sx9310_CPS_CTRL0_REG    0x10
#define sx9310_CPS_CTRL1_REG    0x11
#define sx9310_CPS_CTRL2_REG    0x12
#define sx9310_CPS_CTRL3_REG    0x13
#define sx9310_CPS_CTRL4_REG    0x14
#define sx9310_CPS_CTRL5_REG    0x15
#define sx9310_CPS_CTRL6_REG    0x16
#define sx9310_CPS_CTRL7_REG    0x17
#define sx9310_CPS_CTRL8_REG    0x18
#define sx9310_CPS_CTRL9_REG	0x19
#define sx9310_CPS_CTRL10_REG	0x1A
#define sx9310_CPS_CTRL11_REG	0x1B
#define sx9310_CPS_CTRL12_REG	0x1C
#define sx9310_CPS_CTRL13_REG	0x1D
#define sx9310_CPS_CTRL14_REG	0x1E
#define sx9310_CPS_CTRL15_REG	0x1F
#define sx9310_CPS_CTRL16_REG	0x20
#define sx9310_CPS_CTRL17_REG	0x21
#define sx9310_CPS_CTRL18_REG	0x22
#define sx9310_CPS_CTRL19_REG	0x23
#define sx9310_SAR_CTRL0_REG	0x2A
#define sx9310_SAR_CTRL1_REG	0x2B
#define sx9310_SAR_CTRL2_REG	0x2C

#define sx9310_SOFTRESET_REG  0x7F

#define sx9310_REV_REG
#define sx9310_WHOAMI_REG       0x42

/*      Sensor Readback */
#define sx9310_CPSRD          0x30

#define sx9310_USEMSB         0x31
#define sx9310_USELSB         0x32

#define sx9310_AVGMSB         0x33
#define sx9310_AVGLSB         0x34

#define sx9310_DIFFMSB        0x35
#define sx9310_DIFFLSB        0x36
#define sx9310_OFFSETMSB		0x37
#define sx9310_OFFSETLSB		0x38

#define sx9310_SARMSB			0x39
#define sx9310_SARLSB			0x3A



/*      IrqStat 0:Inactive 1:Active     */
#define sx9310_IRQSTAT_RESET_FLAG      0x80
#define sx9310_IRQSTAT_TOUCH_FLAG      0x40
#define sx9310_IRQSTAT_RELEASE_FLAG    0x20
#define sx9310_IRQSTAT_COMPDONE_FLAG   0x10
#define sx9310_IRQSTAT_CONV_FLAG       0x08
#define sx9310_IRQSTAT_CLOSEALL_FLAG   0x04
#define sx9310_IRQSTAT_FARALL_FLAG     0x02
#define sx9310_IRQSTAT_SMARTSAR_FLAG   0x01


/* CpsStat  */
#define sx9310_TCHCMPSTAT_TCHCOMB_FLAG    0x08
#define sx9310_TCHCMPSTAT_TCHSTAT2_FLAG   0x04
#define sx9310_TCHCMPSTAT_TCHSTAT1_FLAG   0x02
#define sx9310_TCHCMPSTAT_TCHSTAT0_FLAG   0x01


/*      SoftReset */
#define sx9310_SOFTRESET  0xDE

#define LGE_SENSOR

/**************************************
*   define platform data
*
**************************************/
struct smtc_reg_data {
	unsigned char reg;
	unsigned char val;
};
typedef struct smtc_reg_data smtc_reg_data_t;
typedef struct smtc_reg_data *psmtc_reg_data_t;


struct _buttonInfo {
	/*! The Key to send to the input */
	int keycode;
	/*! Mask to look for on Touch Status */
	int mask;
	/*! Current state of button. */
	int state;
};

struct state_pinctrl {
	struct pinctrl *ctrl;
	struct pinctrl_state *active;
	struct pinctrl_state *suspend;
};

struct totalButtonInformation {
	struct _buttonInfo *buttons;
	int buttonSize;
	struct input_dev *input;
};

typedef struct totalButtonInformation buttonInformation_t;
typedef struct totalButtonInformation *pbuttonInformation_t;

/* Define Registers that need to be initialized to values different than
* default
*/
static struct smtc_reg_data sx9310_i2c_reg_setup[] = {
	{
		.reg = sx9310_IRQ_ENABLE_REG,
		.val = 0x70,
	},
	{
		.reg = sx9310_IRQFUNC_REG,
		.val = 0x00,
	},
	{
		.reg = sx9310_CPS_CTRL1_REG,
		.val = 0x00,
	},
	{
		.reg = sx9310_CPS_CTRL2_REG,
		.val = 0x04,
	},
	{
		.reg = sx9310_CPS_CTRL3_REG,
		.val = 0x0A,
	},
	{
		.reg = sx9310_CPS_CTRL4_REG,
		.val = 0x0D,
	},
	{
		.reg = sx9310_CPS_CTRL5_REG,
		.val = 0xC1,
	},
	{
		.reg = sx9310_CPS_CTRL6_REG,
		.val = 0x20,
	},
	{
		.reg = sx9310_CPS_CTRL7_REG,
		.val = 0x4C,
	},
	{
		.reg = sx9310_CPS_CTRL8_REG,
		.val = 0x7E,
	},
	{
		.reg = sx9310_CPS_CTRL9_REG,
		.val = 0x7D,
	},
	{
		.reg = sx9310_CPS_CTRL10_REG,
		.val = 0x00,
	},
	{
		.reg = sx9310_CPS_CTRL11_REG,
		.val = 0x00,
	},
	{
		.reg = sx9310_CPS_CTRL12_REG,
		.val = 0x00,
	},
	{
		.reg = sx9310_CPS_CTRL13_REG,
		.val = 0x00,
	},
	{
		.reg = sx9310_CPS_CTRL14_REG,
		.val = 0x00,
	},
	{
		.reg = sx9310_CPS_CTRL15_REG,
		.val = 0x00,
	},
	{
		.reg = sx9310_CPS_CTRL16_REG,
		.val = 0x00,
	},
	{
		.reg = sx9310_CPS_CTRL17_REG,
		.val = 0x04,
	},
	{
		.reg = sx9310_CPS_CTRL18_REG,
		.val = 0x00,
	},
	{
		.reg = sx9310_CPS_CTRL19_REG,
		.val = 0x00,
	},
	{
		.reg = sx9310_SAR_CTRL0_REG,
		.val = 0x00,
	},
	{
		.reg = sx9310_SAR_CTRL1_REG,
		.val = 0x80,
	},
	{
		.reg = sx9310_SAR_CTRL2_REG,
		.val = 0x0C,
	},
	{
		.reg = sx9310_CPS_CTRL0_REG,
		.val = 0x57,
	},
};


static struct _buttonInfo psmtcButtons[] = {
	{
		.keycode = KEY_SAR0,//KEY_0,
		.mask = sx9310_TCHCMPSTAT_TCHSTAT0_FLAG,
	},
	{
		.keycode = KEY_SAR1,//KEY_1,
		.mask = sx9310_TCHCMPSTAT_TCHSTAT1_FLAG,
	},
	{
		.keycode = KEY_SAR2,//KEY_2
		.mask = sx9310_TCHCMPSTAT_TCHSTAT2_FLAG,
	},
	{
		.keycode = KEY_SAR3,//KEY_3,
		.mask = sx9310_TCHCMPSTAT_TCHCOMB_FLAG,
	},
};
//Startup function
struct _startupcheckparameters {
	s32 dynamicthreshold_ref_offset; // used as an offset for startup (calcualted from temperature test)
	s32 dynamicthreshold_temp_slope; // used for temperature slope
	s32 dynamicthreshold_StartupThreshold;
	s32 dynamicthreshold_StandardCapMain;
	//s32 calibration_margin;
	//u8 startup_touch_regavgthresh; // capacitance(of dynamicthreshold hysteresis) / 128
	//u8 startup_release_regavgthresh; // same value as regproxctrl4[avgthresh]
};
typedef struct _startupcheckparameters *pstartupcheckparameters_t;

struct sx9310_platform_data {
  int i2c_reg_num;
  struct smtc_reg_data *pi2c_reg;
  
  pbuttonInformation_t pbuttonInformation;
  pstartupcheckparameters_t pStartupCheckParameters;//Startup function

  int (*get_is_nirq_low)(void);
  
  int     (*init_platform_hw)(struct i2c_client *client);
  void    (*exit_platform_hw)(struct i2c_client *client);
};
typedef struct sx9310_platform_data sx9310_platform_data_t;
typedef struct sx9310_platform_data *psx9310_platform_data_t;

/***************************************
*  define data struct/interrupt
*
***************************************/
typedef struct sx93XX sx93XX_t, *psx93XX_t;

#define MAX_NUM_STATUS_BITS (8)
struct sx93XX 
{
	void * bus; /* either i2c_client or spi_client */

	struct device *pdev; /* common device struction for linux */

	void *pDevice; /* device specific struct pointer */

	/* Function Pointers */
	int (*init)(psx93XX_t this); /* (re)initialize device */

	/* since we are trying to avoid knowing registers, create a pointer to a
	* common read register which would be to read what the interrupt source
	* is from 
	*/
	int (*refreshStatus)(psx93XX_t this); /* read register status */

	int (*get_nirq_low)(void); /* get whether nirq is low (platform data) */

	/* array of functions to call for corresponding status bit */
	void (*statusFunc[MAX_NUM_STATUS_BITS])(psx93XX_t this); 

	struct state_pinctrl pinctrl;
	
	/* Global variable */
	int 	calData[3];		/*Mainsensor Cap Value , Cal_Offset, Cal_Useful */
	bool 	enable; 		/*Sensor Enable Status -> sx9310_ON(1): Sensor Enable / sx9310_OFF(0):Sensor Disable */
	bool 	cal_done;		/*Calibration Done status -> true : Calibration Done / false : Non Caliration  */
	bool 	proxStatus;		/*Proximity sensor Status -> sx9310_NEAR : Touched / sx9310_FAR : released	*/
	bool 	startupStatus;	/*Proximity sensor Status using startup function -> sx9310_NEAR : Touched / sx9310_FAR : released	*/
	bool 	startup_mode;   /*Startup mode indigator -> true : startup mode / false : irq mode */
	u8 		failStatusCode;	/*Fail status code*/
	bool	reg_in_dts;

	spinlock_t       lock; /* Spin Lock used for nirq worker function */
	int irq; /* irq number used */

	/* whether irq should be ignored.. cases if enable/disable irq is not used
	* or does not work properly */
	char irq_disabled;

	u8 useIrqTimer; /* older models need irq timer for pen up cases */

	int irqTimeout; /* msecs only set if useIrqTimer is true */

	/* struct workqueue_struct *ts_workq;*/  
	/* if want to use non default */
	struct delayed_work dworker; /* work struct for worker function */
};

void sx93XX_suspend(psx93XX_t this);
void sx93XX_resume(psx93XX_t this);
int sx93XX_init(psx93XX_t this);
int sx93XX_remove(psx93XX_t this);

#endif
