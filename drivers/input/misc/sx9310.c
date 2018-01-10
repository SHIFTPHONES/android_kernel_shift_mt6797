/* 
 * Author: yucong xiong <yucong.xion@mediatek.com>
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
 */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/syscalls.h>
#include <linux/wakelock.h>
#include <linux/uaccess.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>

#ifdef CUSTOM_KERNEL_SENSORHUB
#include <SCP_sensorHub.h>
#endif


//#include <mach/mt_typedefs.h>
//#include <mach/mt_gpio.h>
//#include <mach/mt_pm_ldo.h>

//#include <cust_eint.h>       //GPIO info conf
//#include <eint.h> 

#include "sx9310.h"

/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/
#define SX9310_DEV_NAME     "sx9310"

#define GPIO_SAR_EINT_PIN	85

#define IDLE			0
#define ACTIVE			1
/*----------------------------------------------------------------------------*/
#define CAP_TAG                  "[sx9310] "
#define CAP_FUN(f)               printk(KERN_INFO 	CAP_TAG"%s\n", __FUNCTION__)
#define CAP_ERR(fmt, args...)    printk(KERN_ERR  	CAP_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define CAP_LOG(fmt, args...)    printk(KERN_ERR	CAP_TAG fmt, ##args)
#define CAP_DBG(fmt, args...)    printk(KERN_INFO 	CAP_TAG fmt, ##args)    
/*----------------------------------------------------------------------------*/
/*! \struct sx9310
 * Specialized struct containing input event data, platform data, and
 * last cap state read if needed.
 */
typedef struct sx9310
{
	pbuttonInformation_t pbuttonInformation;
	psx9310_platform_data_t hw;		/* specific platform data settings */
} sx9310_t, *psx9310_t;
/*----------------------------------------------------------------------------*/
static int sx9310_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int sx9310_i2c_remove(struct i2c_client *client);
static int sx9310_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int sx9310_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int sx9310_i2c_resume(struct i2c_client *client);
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id sx9310_i2c_id[] ={{SX9310_DEV_NAME, 0}, {} };
/*----------------------------------------------------------------------------*/

#ifdef CONFIG_OF
static const struct of_device_id sar_of_match[] = {
	{.compatible = "mediatek,SAR"},
	{},
};
#endif

static struct i2c_driver sx9310_i2c_driver = {
	.probe	  = sx9310_i2c_probe,
	.remove	  = sx9310_i2c_remove,
	.detect	  = sx9310_i2c_detect,
	.suspend  = sx9310_i2c_suspend,
	.resume	  = sx9310_i2c_resume,
	.id_table = sx9310_i2c_id,
	.driver = {
		.name = SX9310_DEV_NAME,
#ifdef CONFIG_OF
	.of_match_table = sar_of_match,
#endif
	},
};

/*-----------------------sx9310 I2C operations----------------------------------*/
/*! \fn static int write_register(psx93XX_t this, u8 address, u8 value)
 * \brief Sends a write register to the device
 * \param this Pointer to main parent struct 
 * \param address 8-bit register address
 * \param value   8-bit register value to write to address
 * \return Value from i2c_master_send
 */
static int write_register(psx93XX_t this, u8 address, u8 value)
{
	struct i2c_client *i2c = 0;
	char buffer[2];
	int returnValue = 0;

	buffer[0] = address;
	buffer[1] = value;
	returnValue = -ENOMEM;

	if (this && this->bus) {
		i2c = this->bus;
		returnValue = i2c_master_send(i2c,buffer,2);

		CAP_DBG("write_register Address: 0x%x Value: 0x%x Return: %d\n",
				address,value,returnValue);
	}
	return returnValue;
}

/*! \fn static int read_register(psx93XX_t this, u8 address, u8 *value) 
* \brief Reads a register's value from the device
* \param this Pointer to main parent struct 
* \param address 8-Bit address to read from
* \param value Pointer to 8-bit value to save register value to 
* \return Value from i2c_smbus_read_byte_data if < 0. else 0
*/
static int read_register(psx93XX_t this, u8 address, u8 *value)
{
	struct i2c_client *i2c = 0;
	s32 returnValue = 0;
//	s32 test = 0,i =0 ;//pxs_add
	
	if (this && value && this->bus) {
		i2c = this->bus;
		CAP_DBG("i2c->addr: 0x%x\n", i2c->addr);

		returnValue = i2c_smbus_read_byte_data(i2c,address);	
		CAP_DBG("read_register Address: 0x%x Return: 0x%x\n",address,returnValue);
		
#if 0 //pxs
		i=0x00;	
		test = i2c_smbus_read_byte_data(i2c,i);		
			CAP_DBG("pxs_sx9310 read_register addr: 0x%x Return: 0x%x \n",i,test);//pxs_add	
		i=0x01;
		test = i2c_smbus_read_byte_data(i2c,i);		
			CAP_DBG("pxs_sx9310 read_register addr: 0x%x Return: 0x%x \n",i,test);//pxs_add					
		i=0x18;
		test = i2c_smbus_read_byte_data(i2c,i);		
			CAP_DBG("pxs_sx9310 read_register addr: 0x%x Return: 0x%x \n",i,test);//pxs_add			
		i=0x19;
		test = i2c_smbus_read_byte_data(i2c,i);		
			CAP_DBG("pxs_sx9310 read_register addr: 0x%x Return: 0x%x \n",i,test);//pxs_add	
		i=0x30;
		test = i2c_smbus_read_byte_data(i2c,i);		
			CAP_DBG("pxs_sx9310 read_register addr: 0x%x Return: 0x%x \n",i,test);//pxs_add				
		i=0x35;
		test = i2c_smbus_read_byte_data(i2c,i);		
			CAP_DBG("pxs_sx9310 read_register addr: 0x%x Return: 0x%x \n",i,test);//pxs_add			
		i=0x36;
		test = i2c_smbus_read_byte_data(i2c,i);		
			CAP_DBG("pxs_sx9310 read_register addr: 0x%x Return: 0x%x \n",i,test);//pxs_add	
//#else
	for(i = 0; i<=0x38 ; i ++)
	{
		test = i2c_smbus_read_byte_data(i2c,i);
		CAP_DBG("pxs_sx9310 read_register addr: 0x%x Return: 0x%x \n",i,test);//pxs_add	
	}
			
#endif


						
		if (returnValue >= 0) {
			*value = returnValue;
			return 0;
		} 
		else {
			return returnValue;
		}
	}
	return -ENOMEM;
}

/*! \fn static int read_regStat(psx93XX_t this)
 * \brief Shortcut to read what caused interrupt.
 * \details This is to keep the drivers a unified
 * function that will read whatever register(s) 
 * provide information on why the interrupt was caused.
 * \param this Pointer to main parent struct 
 * \return If successful, Value of bit(s) that cause interrupt, else 0
 */
static int read_regStat(psx93XX_t this)
{
	u8 data = 0;
	if (this) {
		if (read_register(this,sx9310_IRQSTAT_REG,&data) == 0)
		return (data & 0x00FF);
	}
	return 0;
}

/*-----------------------dev_atrribute operations-------------------------------*/
static int manual_offset_calibration(psx93XX_t this)
{
	s32 returnValue = 0;
	returnValue = write_register(this,sx9310_IRQSTAT_REG,0xFF);
	return returnValue;
}

static ssize_t manual_offset_calibration_show(struct device *dev,
								struct device_attribute *attr, char *buf)
{
	u8 reg_value = 0;
	psx93XX_t this = dev_get_drvdata(dev);

	dev_info(this->pdev, "Reading IRQSTAT_REG\n");
	read_register(this,sx9310_IRQSTAT_REG,&reg_value);
	return sprintf(buf, "%d\n", reg_value);
}

static ssize_t manual_offset_calibration_store(struct device *dev,
			struct device_attribute *attr,const char *buf, size_t count)
{
	psx93XX_t this = dev_get_drvdata(dev);
	unsigned long val;
	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (val) {
		dev_info( this->pdev, "Performing manual_offset_calibration()\n");
		manual_offset_calibration(this);
	}
	return count;
}

static ssize_t sx9310_register_write_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	int reg_address = 0, val = 0;
	psx93XX_t this = dev_get_drvdata(dev);

	if (sscanf(buf, "%x,%x", &reg_address, &val) != 2) {
		CAP_ERR("The number of data are wrong\n");
		return -EINVAL;
	}

	write_register(this, (unsigned char)reg_address, (unsigned char)val);
	CAP_LOG("%s - Register(0x%x) data(0x%x)\n",__func__, reg_address, val);

	return count;
}

static ssize_t sx9310_register_read_store(struct device *dev,
			   struct device_attribute *attr, const char *buf, size_t count)
{
	u8 val=0;
	int regist = 0;
	psx93XX_t this = dev_get_drvdata(dev);
	int val_gpio = -1;

	CAP_LOG("Reading register\n");

	if (sscanf(buf, "%x", &regist) != 1) {
		CAP_ERR("The number of data are wrong\n");
		return -EINVAL;
	}

	read_register(this, regist, &val);
	CAP_LOG("%s - Register(0x%2x) data(0x%2x)\n",__func__, regist, val);

	mdelay(50);
	val_gpio = gpio_get_value(GPIO_SAR_EINT_PIN);
	CAP_LOG("irq gpio val_gpio=%d\n", val_gpio);

	return count;
}


static ssize_t sx9310_raw_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 msb = 0, lsb = 0;
	u8 i;
	s32 useful;
	s32 average;
	s32 diff;
	u16 offset;
	psx93XX_t this = dev_get_drvdata(dev);
	
    for (i = 0; i < 3; i++){
		write_register(this, sx9310_CPSRD, i);
		read_register(this, sx9310_USEMSB, &msb);
		read_register(this, sx9310_USELSB, &lsb);
		useful = (s32)((msb << 8) | lsb);

		read_register(this, sx9310_AVGMSB, &msb);
		read_register(this, sx9310_AVGLSB, &lsb);
		average = (s32)((msb << 8) | lsb);

		read_register(this, sx9310_OFFSETMSB, &msb);
		read_register(this, sx9310_OFFSETLSB, &lsb);
		offset = (u16)((msb << 8) | lsb);

		read_register(this, sx9310_DIFFMSB, &msb);
		read_register(this, sx9310_DIFFLSB, &lsb);
		diff = (s32)((msb << 8) | lsb);
	
		if (useful > 32767)
			useful -= 65536;
		if ( diff> 32767)
			diff -= 65536;
		if (average > 32767)
			average -= 65536;
		CAP_LOG("PH[%d]: Useful : %d Average : %d, Offset : %d, DIFF : %d\n",
				i,useful,average,offset,diff);
	}
	return 0;
}

static ssize_t chipinfo_value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 reg_value[2] = {0};

	psx93XX_t this = dev_get_drvdata(dev);
	if(NULL == this)
	{
		CAP_ERR("i2c client is null!!\n");
		return 0;
	}
	
	read_register(this,sx9310_WHOAMI_REG,&reg_value[0]);
	return sprintf(buf, "WHOAMI:%d \n", reg_value[0]);       
}

static DEVICE_ATTR(manual_calibrate, 0664, manual_offset_calibration_show,manual_offset_calibration_store);
static DEVICE_ATTR(register_write,  0664, NULL,sx9310_register_write_store);
static DEVICE_ATTR(register_read,0664, NULL,sx9310_register_read_store);
static DEVICE_ATTR(raw_data,0664,sx9310_raw_data_show,NULL);
static DEVICE_ATTR(chipinfo, 0664, chipinfo_value_show,NULL);


static struct attribute *sx9310_attributes[] = {
	&dev_attr_manual_calibrate.attr,
	&dev_attr_register_write.attr,
	&dev_attr_register_read.attr,
	&dev_attr_raw_data.attr,
	&dev_attr_chipinfo.attr,
	NULL,
};
static struct attribute_group sx9310_attr_group = {
	.attrs = sx9310_attributes,
};
/*----------------------------sx9310 function operations-------------------------------*/
static void read_rawData(psx93XX_t this)
{
	u8 msb=0, lsb=0;
    u8 channel = 0,i;
	s32 useful;
	s32 average;
	s32 diff;
	u16 offset;

	read_register(this,sx9310_CPSRD,&channel);
    channel = channel&0x0F;

	if(this){
        for(i = 0; i < 4; i++)
        {
			if(!(channel&(0x01<<i)))
				continue;
			else{
				write_register(this,sx9310_CPSRD,i);//here to check the CS1, also can read other channel		
				read_register(this,sx9310_USEMSB,&msb);
				read_register(this,sx9310_USELSB,&lsb);
				useful = (s32)((msb << 8) | lsb);
		
				read_register(this,sx9310_AVGMSB,&msb);
				read_register(this,sx9310_AVGLSB,&lsb);
				average = (s32)((msb << 8) | lsb);
		
				read_register(this,sx9310_DIFFMSB,&msb);
				read_register(this,sx9310_DIFFLSB,&lsb);
				diff = (s32)((msb << 8) | lsb);
		
				read_register(this,sx9310_OFFSETMSB,&msb);
				read_register(this,sx9310_OFFSETLSB,&lsb);
				offset = (s32)((msb << 8) | lsb);

				if (useful > 32767)
					useful -= 65536;
				if ( diff> 32767)
					diff -= 65536;
				if (average > 32767)
					average -= 65536;
			   
				CAP_LOG("PH[%d]: useful:%d  average:%d diff:%d offset:[%d]",i,useful,average,diff,offset);
			}	
        }
	}
}

/*! 
 * \brief Handle what to do when a touch occurs
 * \param this Pointer to main parent struct 
*/
static void touchProcess(psx93XX_t this)
{
  int counter = 0;
  u8 i = 0;
  int numberOfButtons = 0;
  psx9310_t pDevice = NULL;
  struct _buttonInfo *buttons = NULL;
  struct input_dev *input = NULL;
  
  struct _buttonInfo *pCurrentButton  = NULL;

  CAP_FUN();
  if (this && (pDevice = this->pDevice))
  {
    CAP_DBG("Inside touchProcess()\n");
    read_register(this, sx9310_STAT0_REG, &i);

    buttons = pDevice->pbuttonInformation->buttons;
    input = pDevice->pbuttonInformation->input;
    numberOfButtons = pDevice->pbuttonInformation->buttonSize;
    
    if (unlikely( (buttons==NULL) || (input==NULL) )) {
      CAP_DBG("ERROR!! buttons or input NULL!!!\n");
      return;
    }

    for (counter = 0; counter < numberOfButtons; counter++) {
      pCurrentButton = &buttons[counter];
      if (pCurrentButton==NULL) {
        CAP_ERR("ERROR!! current button at index: %d NULL!!!\n", counter);
        return;
      }
      switch (pCurrentButton->state) {
        case IDLE: /* Button is not being touched! */
          if (((i & pCurrentButton->mask) == pCurrentButton->mask)) {
            /* User pressed button */
            dev_info(this->pdev, "cap button %d touched\n", counter);
            input_report_key(input, pCurrentButton->keycode, 1);

			input_sync(input);
			input_report_key(input, pCurrentButton->keycode, 0);
			input_sync(input);
			
		    pCurrentButton->state = ACTIVE;	
          } else {
            CAP_DBG("Button %d already released.\n",counter);
          }
          break;
        case ACTIVE: /* Button is being touched! */ 
          if (((i & pCurrentButton->mask) != pCurrentButton->mask)) {
            /* User released button */
            CAP_LOG("cap button %d released\n",counter);
            input_report_key(input, pCurrentButton->keycode, 0);
			input_sync(input);
			input_report_key(input, pCurrentButton->keycode, 1);
			input_sync(input);
			
            pCurrentButton->state = IDLE;
          } else {
            CAP_DBG("Button %d still touched.\n",counter);
          }
          break;
        default: /* Shouldn't be here, device only allowed ACTIVE or IDLE */
          break;
      };
    }
    //input_sync(input);

	CAP_DBG("Leaving touchProcess()\n");
  }
}

/*-----------------------------Init operations----------------------------------*/
static int sx9310_init_platform_hw(struct i2c_client *client)
{
	psx93XX_t this = i2c_get_clientdata(client);
	u32 ints[2] = {0, 0};
	struct device_node *dNode;

	CAP_FUN();

    //mt_set_gpio_dir(GPIO_SAR_PIN, GPIO_DIR_IN);
    //mt_set_gpio_pull_enable(GPIO_SAR_PIN, GPIO_PULL_ENABLE);
    //mt_set_gpio_pull_select(GPIO_SAR_PIN, GPIO_PULL_UP);
#if 1
    dNode = of_find_compatible_node(NULL, NULL, "mediatek, IRQ_SAR-eint"); //
    //dNode = of_find_compatible_node(NULL, NULL, "mediatek,sar_eint"); //
    if(dNode){
		of_property_read_u32_array(dNode, "debounce", ints, ARRAY_SIZE(ints));//
        gpio_set_debounce(ints[0], ints[1]);
        CAP_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

        this->irq = irq_of_parse_and_map(dNode, 0);
        CAP_LOG("obj->irq = %d\n", this->irq);
        if (!this->irq) {
            CAP_ERR("irq_of_parse_and_map fail!!\n");
            return -EINVAL;
        }
	}else {
        CAP_ERR("null irq node!!\n");
        return -EINVAL;
	}
#endif

	return 0;
}

static void sx9310_exit_platform_hw(struct i2c_client *client)
{
	return;
}

static int sx9310_parse_dt(struct sx9310_platform_data *pdata, struct device *dev)
{
	u32 data_array_len = 0;
	u32 *data_array;
	int ret, i, j;
	struct device_node *dNode;

	CAP_FUN();	

	dNode = of_find_compatible_node(NULL, NULL, "Semtech,SAR");//
 
	if (dNode == NULL)
		return -ENODEV;

    data_array_len = ARRAY_SIZE(sx9310_i2c_reg_setup);
	data_array = kmalloc(data_array_len * 2 * sizeof(u32), GFP_KERNEL);
	ret = of_property_read_u32_array(dNode, "Semtech,reg-init",  //
									data_array,
									data_array_len*2);
	if (ret < 0) {
		CAP_DBG("data_array_val read error");
		return -ENOMEM;
	}


	for (i = 0; i < ARRAY_SIZE(sx9310_i2c_reg_setup); i++) {
		for (j = 0; j < data_array_len*2; j += 2) {
			if (data_array[j] == sx9310_i2c_reg_setup[i].reg){
				sx9310_i2c_reg_setup[i].val = data_array[j+1];
			}
		}
	}
    kfree(data_array);
	pdata->i2c_reg_num = ARRAY_SIZE(sx9310_i2c_reg_setup);
	pdata->pi2c_reg = sx9310_i2c_reg_setup;
	/***********************************************************************/
	return 0;
}

static int sx9310_get_nirq_state(void)
{
	int val = -1;
	val = !gpio_get_value(GPIO_SAR_EINT_PIN);
	printk("pxs_sar irq gpio val=%d\n", val);
	return val;
}

/*! \brief  Initialize I2C config from platform data
 * \param this Pointer to main parent struct 
*/
static void sx9310_reg_init(psx93XX_t this)
{
	psx9310_t pDevice = 0;
//	psx9310_platform_data_t pdata = 0;
	int i = 0;

	/* configure device */ 
	CAP_LOG("Going to Setup I2C Registers\n");

	if (this){
		while ( i < ARRAY_SIZE(sx9310_i2c_reg_setup)) {
			/* Write all registers/values contained in i2c_reg */
			CAP_LOG("Going to Write Reg: 0x%x Value: 0x%x\n",
					sx9310_i2c_reg_setup[i].reg,sx9310_i2c_reg_setup[i].val);
			write_register(this, sx9310_i2c_reg_setup[i].reg,sx9310_i2c_reg_setup[i].val);
			i++;
		}
	} else {
		CAP_ERR("ERROR! platform data 0x%p\n",pDevice->hw);
	}	
}

/*! \fn static int initialize(psx93XX_t this)
 * \brief Performs all initialization needed to configure the device
 * \param this Pointer to main parent struct 
 * \return Last used command's return value (negative if error)
*/
static int initialize(psx93XX_t this)
{
	if (this) {
		CAP_LOG("sx9310 register initialize\n");
		/* prepare reset by disabling any irq handling */
		this->irq_disabled = 1;
		disable_irq(this->irq);
		/* perform a reset */
		write_register(this,sx9310_SOFTRESET_REG,sx9310_SOFTRESET);
		/* wait until the reset has finished by monitoring NIRQ */
		CAP_DBG("Sent Software Reset. Waiting until device is back from reset to continue.\n");
		/* just sleep for awhile instead of using a loop with reading irq status */
		msleep(300);
		//while(this->get_nirq_low && this->get_nirq_low()) { read_regStat(this); }
		CAP_DBG("Device is back from the reset, continuing. NIRQ = %d\n",this->get_nirq_low());
		
		
		sx9310_reg_init(this);
		msleep(300); /* make sure everything is running */
		manual_offset_calibration(this);

		/* re-enable interrupt handling */
		enable_irq(this->irq);
		this->irq_disabled = 0;
		
		/* make sure no interrupts are pending since enabling irq will only
		* work on next falling edge */
		read_regStat(this);
		msleep(300); 
		CAP_DBG("Exiting initialize(). NIRQ = %d\n",this->get_nirq_low());
		return 0;
	}
	return -ENOMEM;
}

/*-----------------------------i2c operations----------------------------------*/
static int sx9310_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i = 0;
	int err = 0;

	psx93XX_t this = 0;
	psx9310_t pDevice = 0;
	psx9310_platform_data_t pplatData = 0;

	struct totalButtonInformation *pButtonInformationData = NULL;
	struct input_dev *input = NULL;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	CAP_FUN();

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_WORD_DATA)) {
		CAP_ERR("Check i2c functionality.Fail!\n");
		err = -EIO;
		return err;
	}

	this = devm_kzalloc(&client->dev,sizeof(sx93XX_t), GFP_KERNEL); /* create memory for main struct */
	CAP_LOG("Initialized Main Memory: 0x%p\n",this);

	pButtonInformationData = devm_kzalloc(&client->dev , sizeof(struct totalButtonInformation), GFP_KERNEL);
	if (!pButtonInformationData) {
		CAP_ERR("Failed to allocate memory(totalButtonInformation)\n");
		err = -ENOMEM;
		return err;
	}
    pButtonInformationData->buttonSize = ARRAY_SIZE(psmtcButtons);
	pButtonInformationData->buttons    =  psmtcButtons;

	pplatData = devm_kzalloc(&client->dev,sizeof(struct sx9310_platform_data), GFP_KERNEL);
	if (!pplatData) {
		CAP_ERR("platform data is required!\n");
		return -EINVAL;
	}
	pplatData->get_is_nirq_low         = sx9310_get_nirq_state;
	pplatData->pbuttonInformation      = pButtonInformationData;
	pplatData->pStartupCheckParameters = NULL;


	pplatData->init_platform_hw = sx9310_init_platform_hw;
	pplatData->exit_platform_hw = sx9310_exit_platform_hw;


  // for dts parase
	client->dev.platform_data = pplatData;
    err = sx9310_parse_dt(pplatData, &client->dev);
#if 0
	if (err) {
		CAP_ERR("could not setup pin\n");
		return ENODEV;
	}else{
		this->reg_in_dts = true;
	}
#endif	


    CAP_DBG("sx9310 init_platform_hw done!\n");

	if (this){
		CAP_LOG("sx9310 initialize start!!");
		/* In case we need to reinitialize data 
		* (e.q. if suspend reset device) */
		this->init = initialize;
		/* shortcut to read status of interrupt */
		this->refreshStatus = read_regStat;
		/* pointer to function from platform data to get pendown 
		* (1->NIRQ=0, 0->NIRQ=1) */
		this->get_nirq_low = pplatData->get_is_nirq_low;
		/* save irq in case we need to reference it */
		this->irq = client->irq;
		/* do we need to create an irq timer after interrupt ? */
		this->useIrqTimer = 0;

		/* Setup function to call on corresponding reg irq source bit */
		if (MAX_NUM_STATUS_BITS>= 8)
		{
			this->statusFunc[0] = 0; /* TXEN_STAT */
			this->statusFunc[1] = 0; /* UNUSED */
			this->statusFunc[2] = 0; /* UNUSED */
			this->statusFunc[3] = read_rawData; /* CONV_STAT */
			this->statusFunc[4] = 0;//read_regStat; /* COMP_STAT */
			this->statusFunc[5] = touchProcess; /* RELEASE_STAT */
			this->statusFunc[6] = touchProcess; /* TOUCH_STAT  */
		}

		/* setup i2c communication */
		this->bus = client;
		i2c_set_clientdata(client, this);

		/* record device struct */
		this->pdev = &client->dev;

		/* create memory for device specific struct */
		this->pDevice = pDevice = devm_kzalloc(&client->dev,sizeof(sx9310_t), GFP_KERNEL);
		CAP_LOG("Initialized Device Specific Memory: 0x%p\n",pDevice);

		if (pDevice){
			/* for accessing items in user data (e.g. calibrate) */
			err = sysfs_create_group(&client->dev.kobj, &sx9310_attr_group);

			/* Add Pointer to main platform data struct */
			pDevice->hw = pplatData;

			/* Check if we hava a platform initialization function to call*/
			if (pplatData->init_platform_hw)
				pplatData->init_platform_hw(client);

			/* Initialize the button information initialized with keycodes */
			pDevice->pbuttonInformation = pplatData->pbuttonInformation;
			/* Create the input device */
			input = input_allocate_device();
			if (!input) {
				return -ENOMEM;
			}
			/* Set all the keycodes */
			__set_bit(EV_KEY, input->evbit);

			#if 1
			for (i = 0; i < pButtonInformationData->buttonSize; i++) {
				__set_bit(pButtonInformationData->buttons[i].keycode,input->keybit);
				pButtonInformationData->buttons[i].state = IDLE;
			}
			#endif

			/* save the input pointer and finish initialization */
			pButtonInformationData->input = input;
			input->name = "sx9310 Cap Touch";
			input->id.bustype = BUS_I2C;
			if(input_register_device(input)){
				return -ENOMEM;
			}
		}
	}else{
		CAP_ERR("this is NULL function!\n");
		return -1;
	}
	sx93XX_init(this);
	CAP_LOG("sx9310_probe() Done\n");

	return 0;
}

static int sx9310_i2c_remove(struct i2c_client *client)
{
	psx9310_platform_data_t pplatData = 0;
	psx9310_t pDevice = 0;
	psx93XX_t this = i2c_get_clientdata(client);

	CAP_FUN();

	if (this && (pDevice = this->pDevice)){
		input_unregister_device(pDevice->pbuttonInformation->input);

		sysfs_remove_group(&client->dev.kobj, &sx9310_attr_group);
		pplatData = client->dev.platform_data;
		if (pplatData && pplatData->exit_platform_hw)
			pplatData->exit_platform_hw(client);

		kfree(this->pDevice);
	}	
	return sx93XX_remove(this);
}

static int sx9310_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	CAP_FUN();

	return 0;
}

static int sx9310_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
	psx93XX_t this = i2c_get_clientdata(client);

	CAP_FUN();

	sx93XX_suspend(this);
	return 0;
}

static int sx9310_i2c_resume(struct i2c_client *client)
{
	psx93XX_t this = i2c_get_clientdata(client);

	CAP_FUN();

	sx93XX_resume(this);
	return 0;
}


/*----------------------------------------------------------------------------*/
static int  __init sx9310_init(void)
{   
    struct i2c_board_info   sx93xx_i2c_board_info  = { I2C_BOARD_INFO(SX9310_DEV_NAME, 0x28)};

    CAP_FUN();

    i2c_register_board_info(1, &sx93xx_i2c_board_info, 1);
	if (i2c_add_driver(&sx9310_i2c_driver)) {
		CAP_ERR("add driver error\n");
		return -1;
	}
	return 0;
}


static void __exit sx9310_exit(void)
{
	CAP_FUN();	
	i2c_del_driver(&sx9310_i2c_driver);
}


/*----------------------------------------------------------------------------*/
module_init(sx9310_init);
module_exit(sx9310_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Semtech Corp. (http://www.semtech.com/)");
MODULE_DESCRIPTION("SX9311 Capacitive Touch Controller Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");


#ifdef USE_THREADED_IRQ
static void sx93XX_process_interrupt(psx93XX_t this,u8 nirqlow)
{
  int status = 0;
  int counter = 0;
  if (!this) {
    CAP_ERR("sx93XX_worker_func, NULL sx93XX_t\n");
    return;
  }
    /* since we are not in an interrupt don't need to disable irq. */
  status = this->refreshStatus(this);
  counter = -1;
	CAP_DBG("Worker - Refresh Status %d\n",status);

  while((++counter) < MAX_NUM_STATUS_BITS) { /* counter start from MSB */
	  dev_dbg(this->pdev, "Looping Counter %d\n",counter);
    if (((status>>counter) & 0x01) && (this->statusFunc[counter])) {
	    CAP_DBG("Function Pointer Found. Calling\n");
      this->statusFunc[counter](this);
    }
  }
  if (unlikely(this->useIrqTimer && nirqlow)) {
    /* In case we need to send a timer for example on a touchscreen
     * checking penup, perform this here
     */
    cancel_delayed_work(&this->dworker);
    schedule_delayed_work(&this->dworker,msecs_to_jiffies(this->irqTimeout));
    CAP_LOG(\"Schedule Irq timer");
  } 
}


static void sx93XX_worker_func(struct work_struct *work)
{
  psx93XX_t this = 0;
  if (work) {
    this = container_of(work,sx93XX_t,dworker.work);
    if (!this) {
      printk(KERN_ERR "sx93XX_worker_func, NULL sx93XX_t\n");
      return;
    }
    if ((!this->get_nirq_low) || (!this->get_nirq_low())) {
      /* only run if nirq is high */
      sx93XX_process_interrupt(this,0);
    }
  } else {
    CAP_ERR("sx93XX_worker_func, NULL work_struct\n");
  }
}

static irqreturn_t sx93XX_interrupt_thread(int irq, void *data)
{
  psx93XX_t this = 0;
  this = data;
  mutex_lock(&this->mutex);
  dev_dbg(this->pdev, "sx93XX_irq\n");
  if ((!this->get_nirq_low) || this->get_nirq_low()) {
    sx93XX_process_interrupt(this,1);
  }
  else
	CAP_ERR("sx93XX_irq - nirq read high\n");
  mutex_unlock(&this->mutex);
  return IRQ_HANDLED;
}
#else
static void sx93XX_schedule_work(psx93XX_t this, unsigned long delay)
{
  unsigned long flags;

  if (this) {
	CAP_DBG("sx93XX_schedule_work()\n");
	spin_lock_irqsave(&this->lock,flags);
	/*Stop any pending penup queues*/
	cancel_delayed_work(&this->dworker);
	/*after waiting for a delay, this put the job in the kernel-global workqueue. 
	so no need to create new thread in work queue.
	*/
	schedule_delayed_work(&this->dworker,delay);
	spin_unlock_irqrestore(&this->lock,flags);
  }
  else
    CAP_ERR("sx93XX_schedule_work, NULL psx93XX_t\n");
} 

static irqreturn_t sx93XX_irq(int irq, void *pvoid)
{
  psx93XX_t this = 0;

  CAP_FUN();

  if (pvoid) {
	this = (psx93XX_t)pvoid;
	CAP_DBG("sx93XX_irq\n");
    if ((!this->get_nirq_low) || this->get_nirq_low()) {
	    CAP_DBG("sx93XX_irq - Schedule Work\n");
		sx93XX_schedule_work(this,0);
    }else
		CAP_ERR("sx93XX_irq - nirq read high\n");
  }
  else
    CAP_ERR("sx93XX_irq, NULL pvoid\n");
  return IRQ_HANDLED;
}

static void sx93XX_worker_func(struct work_struct *work)
{
  psx93XX_t this = 0;
  int status = 0;
  int counter = 0;
  u8 nirqLow = 0;

  CAP_FUN();

  if (work) {
	this = container_of(work,sx93XX_t,dworker.work);
    if (!this) {
      CAP_ERR("sx93XX_worker_func, NULL sx93XX_t\n");
      return;
    }
    if (unlikely(this->useIrqTimer)) {
      if ((!this->get_nirq_low) || this->get_nirq_low()) {
		nirqLow = 1;
      }
    }
    /* since we are not in an interrupt don't need to disable irq. */
    status = this->refreshStatus(this);
    counter = -1;
	dev_dbg(this->pdev, "Worker - Refresh Status %d\n",status);
    while((++counter) < MAX_NUM_STATUS_BITS) { 
		/* counter start from MSB */
		CAP_DBG("Looping Counter %d\n",counter);
		if (((status>>counter) & 0x01) && (this->statusFunc[counter])) {
			CAP_DBG("Function Pointer Found. Calling\n");
			this->statusFunc[counter](this);
      }
    }
    if (unlikely(this->useIrqTimer && nirqLow)){ 
		/* Early models and if RATE=0 for newer models require a penup timer */
		/* Queue up the function again for checking on penup                 */
		sx93XX_schedule_work(this,msecs_to_jiffies(this->irqTimeout));
	}
  } else {
    CAP_ERR(KERN_ERR "sx93XX_worker_func, NULL work_struct\n");
  }
}
#endif

void sx93XX_suspend(psx93XX_t this)
{
	if (this)
		disable_irq(this->irq);
	    write_register(this,sx9310_CPS_CTRL1_REG,0x20);//make sx9310 in Sleep mode pxs_add 20171027
}

void sx93XX_resume(psx93XX_t this)
{
  if (this) {
#ifdef USE_THREADED_IRQ
	mutex_lock(&this->mutex);
	/* Just in case need to reset any uncaught interrupts */
	sx93XX_process_interrupt(this,0);
	mutex_unlock(&this->mutex);
#else
	sx93XX_schedule_work(this,0);
#endif
	if (this->init)
		this->init(this);
	enable_irq(this->irq);
  }
}

//def CONFIG_HAS_WAKELOCK
#if 0
/*TODO: Should actually call the device specific suspend/resume
 * As long as the kernel suspend/resume is setup, the device
 * specific ones will be called anyways
 */
extern suspend_state_t get_suspend_state(void);
void sx93XX_early_suspend(struct early_suspend *h)
{
	psx93XX_t this = 0;
  dev_dbg(this->pdev, "inside sx93XX_early_suspend()\n");
	this = container_of(h, sx93XX_t, early_suspend);
  sx93XX_suspend(this);
  dev_dbg(this->pdev, "exit sx93XX_early_suspend()\n");
}

void sx93XX_late_resume(struct early_suspend *h)
{
	psx93XX_t this = 0;
  dev_dbg(this->pdev, "inside sx93XX_late_resume()\n");
	this = container_of(h, sx93XX_t, early_suspend);
  sx93XX_resume(this);
  dev_dbg(this->pdev, "exit sx93XX_late_resume()\n");
}
#endif

int sx93XX_init(psx93XX_t this)
{
  int err = 0;

  CAP_FUN();
  if (this && this->pDevice)
  {

#ifdef USE_THREADED_IRQ
    /* initialize worker function */
	INIT_DELAYED_WORK(&this->dworker, sx93XX_worker_func);
    /* initialize mutex */
    mutex_init(&this->mutex);
    /* initailize interrupt reporting */
    this->irq_disabled = 0;
	err = request_threaded_irq(this->irq, NULL, sx93XX_interrupt_thread,
                              IRQF_TRIGGER_FALLING, this->pdev->driver->name,
							  this);
#else
    /* initialize spin lock */
  	spin_lock_init(&this->lock);
    /* initialize worker function */
	INIT_DELAYED_WORK(&this->dworker, sx93XX_worker_func);
    /* initailize interrupt reporting */
    this->irq_disabled = 0;
	err = request_irq(this->irq, sx93XX_irq, IRQF_TRIGGER_FALLING,this->pdev->driver->name,this);
#endif
	if (err) {
	  CAP_ERR("irq %d busy?\n", this->irq);
	  return err;
	}
#ifdef USE_THREADED_IRQ
    CAP_LOG("registered with threaded irq (%d)\n", this->irq);
#else
    CAP_LOG("registered with irq (%d)\n", this->irq);
#endif

#if 0
	//def CONFIG_HAS_WAKELOCK	
    this->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    this->early_suspend.suspend = sx93XX_early_suspend;
    this->early_suspend.resume = sx93XX_late_resume;
    register_early_suspend(&this->early_suspend);
    if (has_wake_lock(WAKE_LOCK_SUSPEND) == 0 && 
        get_suspend_state() == PM_SUSPEND_ON)
     	sx93XX_early_suspend(&this->early_suspend);
	//CONFIG_HAS_WAKELOCK
#endif

    /* call init function pointer (this should initialize all registers */
    if (this->init)
		return this->init(this);
	}else{
		dev_err(this->pdev,"No init function!!!!\n");
	}
	return -ENOMEM;
}

int sx93XX_remove(psx93XX_t this)
{
  if (this) {
	/* Cancel the Worker Func */
    cancel_delayed_work_sync(&this->dworker); 
    /*destroy_workqueue(this->workq); */
#if 0
	//def CONFIG_HAS_WAKELOCK
    unregister_early_suspend(&this->early_suspend);
#endif
    free_irq(this->irq, this);
    kfree(this);
    return 0;
  }
  return -ENOMEM;
}







