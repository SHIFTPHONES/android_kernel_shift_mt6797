#include <linux/types.h>
#include <linux/init.h>     /* For init/exit macros */
#include <linux/module.h>   /* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
//#include <linux/ioctl.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>


#include <linux/atomic.h>
#include <linux/wakelock.h>


#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif


#define KTD_I2C_NAME			"ktd2037"
#define DEV_NAME 		"ktd2037_dev"
#define	CLASS_NAME		"ktd2037_class"
#define MAJOR_DEV_NUM		252	//debug

static struct class *ktd2037_class;

struct ktd2037_priv{
	struct i2c_client *client;
};

static struct i2c_client * ktd2037_client = NULL;
static struct ktd2037_priv *ktd2037_obj = NULL;
unsigned long value = 0;

static int ktd2037_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int ktd2037_i2c_remove(struct i2c_client *client);

void ktd2037_lowbattery_breath_leds(void){
	/* battery level<15%=>Flash Red
	 * flash time period: 2.5s, flash: 0.5s
	 * reg5 = 0xaa, reg1 = 0x12
	 */
        printk("ktd2037_lowbattery_breath_leds\n");
	i2c_smbus_write_byte_data(ktd2037_client, 0x06, 0x13);//set current is 2.5mA
	i2c_smbus_write_byte_data(ktd2037_client, 0x01, 0x12);//dry flash period
	i2c_smbus_write_byte_data(ktd2037_client, 0x02, 0x00);//reset internal counter
	i2c_smbus_write_byte_data(ktd2037_client, 0x04, 0x42);//allocate led1 to timer1;
	i2c_smbus_write_byte_data(ktd2037_client, 0x02, 0x32);//led flashing(curerent ramp-up and down countinuously)
}

void ktd2037_battery_charging_low(void){
	//turn on led when battery level<15% and charging=>solid red
        printk("ktd2037_battery_charging_low\n");
	i2c_smbus_write_byte_data(ktd2037_client, 0x06, 0x13);//set current is 2.5mA
	i2c_smbus_write_byte_data(ktd2037_client, 0x04, 0x41);//turn on all of leds;
}

void ktd2037_battery_charging_middle(void){
	//turn on led when 15%<battert level<=90 and charging=>solid orange(Red+Green)
        printk("ktd2037_battery_charging_middle\n");
	i2c_smbus_write_byte_data(ktd2037_client, 0x06, 0x13);//set current is 2.5mA
        i2c_smbus_write_byte_data(ktd2037_client, 0x08, 0x00);//set current is 0.125mA
	i2c_smbus_write_byte_data(ktd2037_client, 0x04, 0x51);//turn on LED1 and LED3;
}

void ktd2037_battery_charging_High(void){
	//turn on led when battery level>=90% and charging=>solid Green
        printk("ktd2037_battery_charging_High\n");
	i2c_smbus_write_byte_data(ktd2037_client, 0x08, 0x13);//set current is 2.5mA
	i2c_smbus_write_byte_data(ktd2037_client, 0x04, 0x50);//turn on led3;
}

void ktd2037_events_breath_leds(void){
	/* battery level<15%=>Flash Red
	 * flash time period: 2.5s, , flash:0.5s 
	 * reg5 = 0xaa, reg1 = 0x30	
	 */
        printk("ktd2037_events_breath_leds\n");
	i2c_smbus_write_byte_data(ktd2037_client, 0x07, 0x13);//set current is 2.5mA
	i2c_smbus_write_byte_data(ktd2037_client, 0x01, 0x12);//dry flash period
	i2c_smbus_write_byte_data(ktd2037_client, 0x02, 0x00);//reset internal counter
	i2c_smbus_write_byte_data(ktd2037_client, 0x04, 0x48);//allocate led2 to timer1;
	i2c_smbus_write_byte_data(ktd2037_client, 0x02, 0x32);//led flashing(curerent ramp-up and down countinuously)
}

void ktd2037_initialization_LED_off(void){
//reset IC
        printk("ktd2037_initialization_LED_off\n");
	i2c_smbus_write_byte_data(ktd2037_client, 0x00, 0x05);// initialization LED off
	i2c_smbus_write_byte_data(ktd2037_client, 0x04, 0x00);// initialization LED off
        i2c_smbus_write_byte_data(ktd2037_client, 0x09, 0x06);// disable auto blink
}


void ktd2037_reset(void){
	//enable auto blink
	i2c_smbus_write_byte_data(ktd2037_client, 0x00, 0x05);//enable auto blink
}

 void ktd2037_led_off(void){
	//turn on led when 0x02 is not 0x00,there is set to same as breath leds
	i2c_smbus_write_byte_data(ktd2037_client, 0x06, 0x00);//set current is 0.125mA
	i2c_smbus_write_byte_data(ktd2037_client, 0x04, 0x00);//turn off leds
	i2c_smbus_write_byte_data(ktd2037_client, 0x00, 0x05);//enable auto blink
}

void ktd2037_breath_leds_time(int blink){
	//set breath led flash time from blink
	int period, flashtime;
	period = (blink >> 8) & 0xff;
	flashtime = blink & 0xff;
	printk("ktd2037 led write blink = %x, period = %x, flashtime = %x\n", blink, period, flashtime);
	i2c_smbus_write_byte_data(ktd2037_client, 0x00, 0x20);// initialization LED off
	i2c_smbus_write_byte_data(ktd2037_client, 0x04, 0x00);// initialization LED off
	i2c_smbus_write_byte_data(ktd2037_client, 0x06, 0xbf);//set current is 24mA
	i2c_smbus_write_byte_data(ktd2037_client, 0x05, period);//rase time
	i2c_smbus_write_byte_data(ktd2037_client, 0x01, flashtime);//dry flash period
	i2c_smbus_write_byte_data(ktd2037_client, 0x02, 0x00);//reset internal counter
	i2c_smbus_write_byte_data(ktd2037_client, 0x04, 0x42);//allocate led1 to timer1;change 02-> 42 
	i2c_smbus_write_byte_data(ktd2037_client, 0x02, 0x56);//led flashing(curerent ramp-up and down countinuously)

}

void ktd2037_red_led_turn_on(void){
	i2c_smbus_write_byte_data(ktd2037_client, 0x00, 0x05);// initialization LED off
	i2c_smbus_write_byte_data(ktd2037_client, 0x04, 0x00);// initialization LED off
        i2c_smbus_write_byte_data(ktd2037_client, 0x09, 0x06);// disable auto blink
	i2c_smbus_write_byte_data(ktd2037_client, 0x06, 0x13);//set current is 2.5mA
	i2c_smbus_write_byte_data(ktd2037_client, 0x04, 0x41);//turn on led1;
}


void ktd2037_blue_led_turn_on(void){
	i2c_smbus_write_byte_data(ktd2037_client, 0x00, 0x05);// initialization LED off
	i2c_smbus_write_byte_data(ktd2037_client, 0x04, 0x00);// initialization LED off
        i2c_smbus_write_byte_data(ktd2037_client, 0x09, 0x06);// disable auto blink
	i2c_smbus_write_byte_data(ktd2037_client, 0x07, 0x13);//set current is 2.5mA
	i2c_smbus_write_byte_data(ktd2037_client, 0x04, 0x44);//turn on led2;
}

void ktd2037_green_led_turn_on(void){
	i2c_smbus_write_byte_data(ktd2037_client, 0x00, 0x05);// initialization LED off
	i2c_smbus_write_byte_data(ktd2037_client, 0x04, 0x00);// initialization LED off
        i2c_smbus_write_byte_data(ktd2037_client, 0x09, 0x06);// disable auto blink
	i2c_smbus_write_byte_data(ktd2037_client, 0x08, 0x13);//set current is 2.5mA
	i2c_smbus_write_byte_data(ktd2037_client, 0x04, 0x50);//turn on led3;
}


void ktd2037_red_green_turn_on(void){
	i2c_smbus_write_byte_data(ktd2037_client, 0x00, 0x05);// initialization LED off
	i2c_smbus_write_byte_data(ktd2037_client, 0x04, 0x00);// initialization LED off
        i2c_smbus_write_byte_data(ktd2037_client, 0x09, 0x06);// disable auto blink
	i2c_smbus_write_byte_data(ktd2037_client, 0x06, 0x13);//set current is 2.5mA
        i2c_smbus_write_byte_data(ktd2037_client, 0x08, 0x00);//set current is 0.125mA
	i2c_smbus_write_byte_data(ktd2037_client, 0x04, 0x51);//turn on LED1 and LED3;
}

void ktd2037_turn_off_LED(void){
	i2c_smbus_write_byte_data(ktd2037_client, 0x00, 0x05);// initialization LED off
	i2c_smbus_write_byte_data(ktd2037_client, 0x04, 0x00);// initialization LED off
	i2c_smbus_write_byte_data(ktd2037_client, 0x04, 0x00);//turn off all LEDs;
}


static const struct i2c_device_id ktd2037_id[] = {
	{KTD_I2C_NAME, 0},
	{ }
};


static struct of_device_id ktd2037_match_table[] = {
        { .compatible = "mediatek,led_ktd2037",},
        { },
};


static struct i2c_driver ktd2037_driver = {
	.driver = {
		.name	= KTD_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ktd2037_match_table,
	},
	.probe = ktd2037_i2c_probe,
	.remove = ktd2037_i2c_remove,
	.id_table = ktd2037_id,
};

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int ktd2037_open(struct inode *inode, struct file *file)
{
	file->private_data = ktd2037_client;

	if (!file->private_data)
	{
		printk("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static ssize_t ktd2037_read(struct file *file, char __user *buf, size_t size, loff_t * offset)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t ktd2037_write(struct file *file, const char __user *buf, size_t size, loff_t *offset)
{

	static int rec_data[2];
	memset(rec_data, 0, 2);  //

	
	if (copy_from_user(rec_data, buf, 1))  
	{  
		return -EFAULT;  
	}  
	printk("ktd2037_write:rec_data[0]=%d\n",rec_data[0]);
	 if(rec_data[0] == 0){
		ktd2037_initialization_LED_off();
	 }
	 else if( rec_data[0] == 1){
		ktd2037_lowbattery_breath_leds();
	 }
	 else if( rec_data[0] == 2){
		ktd2037_battery_charging_low();
	 }
	 else if( rec_data[0] == 3){
		ktd2037_battery_charging_middle();
	 }
	 else if( rec_data[0] == 4){
		ktd2037_battery_charging_High();
	 }
	 else if( rec_data[0] == 5){
		ktd2037_events_breath_leds();
	 }
     else if( rec_data[0] == 6){
		ktd2037_turn_off_LED();
	 }
	 else if( rec_data[0] == 7){
		ktd2037_red_led_turn_on();
	 }
	 else if( rec_data[0] == 8){
		ktd2037_blue_led_turn_on();
	 }
	 else if( rec_data[0] == 9){
		ktd2037_green_led_turn_on();
	 }
	 else if( rec_data[0] == 10){
		ktd2037_red_green_turn_on();
	 }
	else
        {
                ktd2037_turn_off_LED();
	}

	
    return 1;
}
/*----------------------------------------------------------------------------*/
static int ktd2037_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/
static long ktd2037_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	 //int ret;
	 //void __user *ptr = (void __user*) arg;
#if 0
	 //switch(cmd)
	// {
	 	//case ENCRYPT_IOCTL_GET_CIT_RESULT:
	 			printk("[mly] ENCRYPT_IOCTL_GET_CIT_RESULT\n");
				if(copy_to_user(ptr, &cit_result, sizeof(cit_result)))//返回结果：芯片正常1, 不正常0
				{
					printk("[mly]copy_to_user failed !\n");
					return -3;					
				}
				//break;
	 	
		//default :break;

	// }
#endif
	 printk("[mly]    ioctl   end   =====================\n");

	 return 0;
}
/*----------------------------------------------------------------------------*/
static struct file_operations ktd2037_fops = {
	.owner = THIS_MODULE,
	.open = ktd2037_open,
	.read  = ktd2037_read,
	.write = ktd2037_write,
	.release = ktd2037_release,
	.unlocked_ioctl = ktd2037_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice ktd2037_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEV_NAME,
	.fops = &ktd2037_fops,
};


static int ktd2037_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	struct ktd2037_priv *obj = NULL;
	int err;
	printk("[mly]  ######enter ktd2037_i2c_probe  !!###### \n");
	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (obj == NULL) {
		err = -ENOMEM;
		goto exit;
	}
	ktd2037_obj = obj;
	obj->client = client;
	ktd2037_client = client;
	i2c_set_clientdata(client, obj);

	if((err = misc_register(&ktd2037_device)))
	{
		printk("ktd2037_device register failed\n");
		goto exit;
	}	
	ktd2037_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(ktd2037_class))
	{
		printk("ktd2037_class create failed\n");
		goto exit0;
	}
	device_create(ktd2037_class, NULL, MKDEV(MAJOR_DEV_NUM, 0), NULL, DEV_NAME);
	printk("[mly]ktd2037 i2c_probe end!\n");
	return err;

exit0:
	class_destroy(ktd2037_class);
exit:
	if(obj)
	{
		kfree(obj);
	}
	ktd2037_client = NULL;
	printk("[mly]ktd2037 i2c_probe failed! must some error occurs\n");
	return err;
}

unsigned int unpack32( unsigned char *src )
{
	return (
		(( unsigned int) src[0] )<< 24
		| (( unsigned int) src[1] )<< 16
		| (( unsigned int) src[2] )<< 8
		| ( unsigned int) src[3]
	);
}

/*----------------------------------------------------------------------------*/
static int ktd2037_i2c_remove(struct i2c_client *client)
{
	int err;
	printk("[mly]  ######ktd2037_i2c_remove !!###### \n");
	class_destroy(ktd2037_class);
	if((err = misc_deregister(&ktd2037_device)))
	{
		printk("misc_deregister fail: %d\n", err);    
	}

	
	ktd2037_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static int __init ktd2037_init(void)
{
	int err;
	printk("%s\n",__func__);
	err = i2c_add_driver(&ktd2037_driver);
	if (err) {
		printk("ktd2037 driver failed "
		       "(errno = %d)\n", err);
	} else {
		printk( "Successfully added driver %s\n",
		          ktd2037_driver.driver.name);
	}
	return err;
}

static void __exit ktd2037_exit(void)
{
	printk("%s\n",__func__);
	i2c_del_driver(&ktd2037_driver);
}

module_init(ktd2037_init);
module_exit(ktd2037_exit);

MODULE_LICENSE("GPL");

