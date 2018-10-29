#include <linux/types.h>
#include <linux/init.h>     /* For init/exit macros */
#include <linux/module.h>   /* For MODULE_ marcros  */
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#define LIGHTS_DBG_ON  0

#define KTD_I2C_NAME	"ktd2037"

#define MAX_CURRENT 0x9

/* Constants taken borrowed from libhardware */
#define LIGHT_FLASH_NONE            0

/**
 * To flash the light at a given rate, set flashMode to LIGHT_FLASH_TIMED,
 * and then flashOnMS should be set to the number of milliseconds to turn
 * the light on, followed by the number of milliseconds to turn the light
 * off.
 */
#define LIGHT_FLASH_TIMED           1

/**
 * To flash the light using hardware assist, set flashMode to
 * the hardware mode.
 */
#define LIGHT_FLASH_HARDWARE        2

/* Registers */
#define KTD_REG_RESET_CONTROL		0x00
#define KTD_REG_FLASH_PERIOD		0x01
#define KTD_REG_FLASH_ON_PERCENT1	0x02 // Timer1: Percentage of flash period the led is lit
#define KTD_REG_FLASH_ON_PERCENT2	0x03 // Timer2: Percentage of flash period the led is lit
#define KTD_REG_LED_ID			0x04
#define KTD_REG_CURRENT_RED		0x06 // in 0.125mA steps
#define KTD_REG_CURRENT_BLUE		0x07 // in 0.125mA steps
#define KTD_REG_CURRENT_GREEN		0x08 // in 0.125mA steps
#define KTD_REG_AUTO_BLINK		0x09

/* Reg Values */
#define FLASH_PERIOD_RAMP_LINEAR	0x80

#define FLASH_PERIOD_STEP_MS		128 // in 0.128s steps
#define FLASH_PERIOD_1_MS		384 // from here it goes in FLASH_PERIOD_STEP_MS steps
#define FLASH_ON_TIMER_FULL             9960 // (99.6% multiplied by 100 in order to avoid floating point) when KTD_REG_FLASH_ON_PERCENT1 or KTD_REG_FLASH_ON_PERCENT2 have value of 255
#define FLASH_PERIOD_MAX		16380 // maximum period value the KTD chip allows

#define LED_ENABLE			0x40
#define LED_RED_OFFSET			0
#define LED_BLUE_OFFSET			2
#define LED_GREEN_OFFSET		4

#define LED_ON				0x1
#define LED_TIMER1			0x2
#define LED_TIMER2			0x3

#define ENABLE_CTRL_LOW_POWER		0x8

struct ktd2037_priv{
	struct i2c_client *client;
	int operation;
	unsigned int led_id;
	unsigned int curr;
	int flash_mode;
	unsigned int flash_on_ms;
	unsigned int flash_off_ms;
	u8  red_brightness;
	u8  green_brightness;
	u8  blue_brightness;
};

static struct i2c_client * ktd2037_client = NULL;
static struct ktd2037_priv *ktd2037_obj = NULL;

static int ktd2037_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int ktd2037_i2c_remove(struct i2c_client *client);
static void ktd2037_i2c_shutdown(struct i2c_client *);

#define I2C_WRITE(reg, val) \
	if (i2c_smbus_write_byte_data(ktd2037_client, reg, val) != 0) { \
		printk("%s: I2C_WRITE error reg(0x%x) val(0x%x)\n", __func__, reg, val); \
	}

u8 brightness_to_steps(u8 b)
{
	return (b * MAX_CURRENT) / 255;
}

void ktd2037_flash_period_calculate(struct ktd2037_priv *obj, u8 *period, u8 *on_timer)
{
	unsigned int flash_period = (obj->flash_on_ms + obj->flash_off_ms); //in ms
	unsigned int no_steps = 0;
	unsigned int flash_on_percentage = (obj->flash_on_ms * 100) / flash_period;

	if (flash_period < FLASH_PERIOD_1_MS) {
		*period = 0;
	} else {
		if (flash_period > FLASH_PERIOD_MAX)
			flash_period = FLASH_PERIOD_MAX;

		no_steps = (flash_period - FLASH_PERIOD_1_MS) / FLASH_PERIOD_STEP_MS;
		*period = (u8) (no_steps + 1);
	}

	*on_timer = (u8) ((flash_on_percentage * 255 * 100) / FLASH_ON_TIMER_FULL);

#ifdef LIGHTS_DBG_ON
	printk("%s: flash_on_ms(%d) flash_off_ms(%d)\n", __func__, obj->flash_on_ms, obj->flash_off_ms);
	printk("%s: flash_period(%d) no_steps(%d) period(%d)\n", __func__, flash_period, no_steps, *period);
	printk("%s: flash_on_percentage(%d) on_timer(%d)\n", __func__, flash_on_percentage, *on_timer);
#endif
}

void ktd2037_turn_off_all_leds(void)
{
#ifdef LIGHTS_DBG_ON
	printk("%s\n", __func__);
#endif
	I2C_WRITE(KTD_REG_LED_ID, 0x00); // turn off leds
}

void ktd2037_execute_operation(struct ktd2037_priv *obj)
{
	u8 led_id = 0;
	u8 cr = brightness_to_steps(obj->red_brightness);
	u8 cg = brightness_to_steps(obj->green_brightness);
	u8 cb = brightness_to_steps(obj->blue_brightness);
	u8 period = 0, on_timer = 0;

	if (!obj->red_brightness && !obj->green_brightness && !obj->blue_brightness) {
		ktd2037_turn_off_all_leds();
		return;
	}

	/* Set led_id */
	led_id |= LED_ENABLE;

	switch(obj->flash_mode) {
	case LIGHT_FLASH_TIMED:
	case LIGHT_FLASH_HARDWARE:
		if (obj->red_brightness) {
			led_id |= (LED_TIMER1 << LED_RED_OFFSET);
		}
		if (obj->green_brightness) {
			led_id |= (LED_TIMER1 << LED_GREEN_OFFSET);
		}
		if (obj->blue_brightness) {
			led_id |= (LED_TIMER1 << LED_BLUE_OFFSET);
		}
		break;

	default:
		if (obj->red_brightness) {
			led_id |= (LED_ON << LED_RED_OFFSET);
		}
		if (obj->green_brightness) {
			led_id |= (LED_ON << LED_GREEN_OFFSET);
		}
		if (obj->blue_brightness) {
			led_id |= (LED_ON << LED_BLUE_OFFSET);
		}
	}

	/* Set flash period and timer */
	switch(obj->flash_mode) {
	case LIGHT_FLASH_TIMED:
		ktd2037_flash_period_calculate(obj, &period, &on_timer);
		break;

	case LIGHT_FLASH_HARDWARE:
		period = 0x12;
		on_timer = 0x32;
		break;
	}

#ifdef LIGHTS_DBG_ON
	printk("%s led_id(%x) current(%x,%x,%x) period(%d) on_timer(%d)\n", __func__, led_id, cr, cg, cb, period, on_timer);
#endif

	I2C_WRITE(KTD_REG_LED_ID, 0x00);// initialization LED off
	I2C_WRITE(KTD_REG_AUTO_BLINK, 0x06);// disable auto blink

	I2C_WRITE(KTD_REG_CURRENT_RED, cr);//set current
	I2C_WRITE(KTD_REG_CURRENT_GREEN, cg);//set current
	I2C_WRITE(KTD_REG_CURRENT_BLUE, cb);//set current

	switch(obj->flash_mode) {
	case LIGHT_FLASH_TIMED:
	case LIGHT_FLASH_HARDWARE:
		I2C_WRITE(KTD_REG_FLASH_PERIOD, period);//dry flash period
		I2C_WRITE(KTD_REG_FLASH_ON_PERCENT1, 0x00);//reset internal counter
		I2C_WRITE(KTD_REG_LED_ID, led_id); //turn on led
		I2C_WRITE(KTD_REG_FLASH_ON_PERCENT1, on_timer); //led flashing(current ramp-up and down countinuously)
		break;

	default:
		/* solid color */
		I2C_WRITE(KTD_REG_LED_ID, led_id);//turn on led
	}
}

static const struct i2c_device_id ktd2037_id[] = {
	{KTD_I2C_NAME, 0},
	{ },
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
	.shutdown = ktd2037_i2c_shutdown,
	.id_table = ktd2037_id,
};

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static ssize_t ktd2037_show_flash_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client;
	struct ktd2037_priv *obj = NULL;

	client = to_i2c_client(dev);
	obj = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", obj->flash_mode);
}

static ssize_t ktd2037_store_flash_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct i2c_client *client;
	struct ktd2037_priv *obj = NULL;
	u8 value;

	client = to_i2c_client(dev);
	obj = i2c_get_clientdata(client);

	ret = kstrtou8(buf, 10, &value);
	if (ret < 0) {
		dev_err(dev, "%s: value too large!\n", __func__);
		return ret;
	}
	obj->flash_mode = value;

#ifdef LIGHTS_DBG_ON
	printk("%s: value(%d)\n", __func__, value);
#endif

	return count;
}

static ssize_t ktd2037_show_flash_period(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client;
	struct ktd2037_priv *obj = NULL;

	client = to_i2c_client(dev);
	obj = i2c_get_clientdata(client);

	return sprintf(buf, "'%d' ms on, '%d' ms off\n", obj->flash_on_ms, obj->flash_off_ms);
}

static ssize_t ktd2037_store_flash_period(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct i2c_client *client;
	struct ktd2037_priv *obj = NULL;
	unsigned int on_ms = 0, off_ms = 0;

	client = to_i2c_client(dev);
	obj = i2c_get_clientdata(client);

	ret = sscanf(buf, "%d:%d", &on_ms, &off_ms);
	if (ret != 2) {
		dev_err(dev, "%s: unable to parse values!\n", __func__);
		return -EINVAL;
	}
	obj->flash_on_ms = on_ms;
	obj->flash_off_ms = off_ms;

#ifdef LIGHTS_DBG_ON
	printk("%s: on_ms(%d) off_ms(%d)\n", __func__, on_ms, off_ms);
#endif

	return count;
}

static ssize_t ktd2037_show_brightness(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client;
	struct ktd2037_priv *obj = NULL;

	client = to_i2c_client(dev);
	obj = i2c_get_clientdata(client);

	return sprintf(buf, "R,G,B (%d, %d, %d)\n", obj->red_brightness, obj->green_brightness, obj->blue_brightness);
}

static ssize_t ktd2037_store_brightness(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct i2c_client *client;
	struct ktd2037_priv *obj = NULL;
	int red_brightness, green_brightness, blue_brightness;

	client = to_i2c_client(dev);
	obj = i2c_get_clientdata(client);
	ret = sscanf(buf, "%d:%d:%d", &red_brightness, &green_brightness, &blue_brightness);
	if (ret != 3) {
		dev_err(dev, "%s: unable to parse values!\n", __func__);
		return -EINVAL;
	}

	if (red_brightness > 255 || green_brightness > 255 || blue_brightness > 255) {
		dev_err(dev, "%s: values too high (> 255)!\n", __func__);
		return -EINVAL;
	}

	obj->red_brightness = red_brightness;
	obj->green_brightness = green_brightness;
	obj->blue_brightness = blue_brightness;

#ifdef LIGHTS_DBG_ON
	printk("%s: R,G,B (%d, %d, %d)\n", __func__, obj->red_brightness, obj->green_brightness, obj->blue_brightness);
#endif

	ktd2037_execute_operation(obj);

	return count;
}


static DEVICE_ATTR(flash_mode, S_IRUGO | S_IWUSR, ktd2037_show_flash_mode, ktd2037_store_flash_mode);
static DEVICE_ATTR(flash_period, S_IRUGO | S_IWUSR, ktd2037_show_flash_period, ktd2037_store_flash_period);
static DEVICE_ATTR(brightness, S_IRUGO | S_IWUSR, ktd2037_show_brightness, ktd2037_store_brightness);

static struct attribute *ktd2037_attributes[] = {
	&dev_attr_flash_mode.attr,
	&dev_attr_flash_period.attr,
	&dev_attr_brightness.attr,
	NULL,
};

static struct attribute_group ktd2037_attr_group = {
	.name = "ktd2037",
	.attrs = ktd2037_attributes,
};

static int ktd2037_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	struct ktd2037_priv *obj = NULL;
	int err;

#ifdef LIGHTS_DBG_ON
	printk("%s: begin\n", __func__);
#endif

	obj = devm_kzalloc(&client->dev, sizeof(*obj), GFP_KERNEL);
	if (obj == NULL) {
		err = -ENOMEM;
		goto exit;
	}
	ktd2037_obj = obj;
	obj->client = client;
	ktd2037_client = client;
	i2c_set_clientdata(client, obj);

	err = sysfs_create_group(&client->dev.kobj, &ktd2037_attr_group);
	if (err) {
		printk("%s: sysfs_create_group failed\n", __func__);
		goto exit;
	}

#ifdef LIGHTS_DBG_ON
	printk("%s: end\n", __func__);
#endif

	return err;

exit:
	ktd2037_client = NULL;
	printk("%s: failed!\n", __func__);
	return err;
}

/*----------------------------------------------------------------------------*/
static int ktd2037_i2c_remove(struct i2c_client *client)
{
#ifdef LIGHTS_DBG_ON
	printk("%s:n", __func__);
#endif

	sysfs_remove_group(&client->dev.kobj, &ktd2037_attr_group);

	ktd2037_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static void ktd2037_i2c_shutdown(struct i2c_client *client)
{
	ktd2037_turn_off_all_leds();
}

static int __init ktd2037_init(void)
{
	int err;
	err = i2c_add_driver(&ktd2037_driver);
	if (err) {
		printk("%s: ktd2037 driver failed (errno = %d)\n", __func__, err);
	} else {
		printk("%s: Successfully added driver \"%s\"\n", __func__, ktd2037_driver.driver.name);
	}
	return err;
}

static void __exit ktd2037_exit(void)
{
#ifdef LIGHTS_DBG_ON
	printk("%s\n",__func__);
#endif
	ktd2037_turn_off_all_leds();
	i2c_del_driver(&ktd2037_driver);
}

module_init(ktd2037_init);
module_exit(ktd2037_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Javi Ferrer @ SHIFTPHONES");
