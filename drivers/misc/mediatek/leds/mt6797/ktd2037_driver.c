#include <linux/types.h>
#include <linux/init.h>     /* For init/exit macros */
#include <linux/module.h>   /* For MODULE_ marcros  */
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#define LIGHTS_DBG_ON  0

#define KTD_I2C_NAME	"ktd2037"

#define MAX_CURRENT 0x9

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
#define KTD_REG_DRY_FLASH_PERIOD	0x01
#define KTD_REG_FLASH_COUNTER		0x02 // No of times the led lits?
#define KTD_REG_LED_ID				0x04
#define KTD_REG_CURRENT_RED			0x06
#define KTD_REG_CURRENT_BLUE		0x07
#define KTD_REG_CURRENT_GREEN		0x08

#define LED_BASE		0x40
#define LED_RED			LED_BASE + (1 << 0)
#define LED_RED_TIMER	LED_BASE + (1 << 1)
#define LED_BLUE		LED_BASE + (1 << 2)
#define LED_BLUE_TIMER	LED_BASE + (1 << 3)
#define LED_GREEN		LED_BASE + (1 << 4)
#define LED_GREEN_TIMER	LED_BASE + (1 << 5)

struct ktd2037_priv{
	struct i2c_client *client;
	int operation;
	unsigned int led_id;
	unsigned int curr;
	int flash_mode;
	unsigned int flash_period;
	u8  red_brightness;
	u8  green_brightness;
	u8  blue_brightness;
};

static struct i2c_client * ktd2037_client = NULL;
static struct ktd2037_priv *ktd2037_obj = NULL;

static int ktd2037_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int ktd2037_i2c_remove(struct i2c_client *client);

#define I2C_WRITE(reg, val) \
	if (i2c_smbus_write_byte_data(ktd2037_client, reg, val) != 0) { \
		printk("%s: I2C_WRITE error reg(0x%x) val(0x%x)\n", __func__, reg, val); \
	}

void ktd2037_led_off(void)
{
#ifdef LIGHTS_DBG_ON
	printk("%s\n", __func__);
#endif
	// turn on led when 0x02 is not 0x00,there is set to same as breath leds
	I2C_WRITE(KTD_REG_CURRENT_RED, 0x00); // set current is 0.125mA
	I2C_WRITE(KTD_REG_LED_ID, 0x00);      // turn off leds
	I2C_WRITE(0x00, 0x05);                // enable auto blink
}

u8 brightness_to_steps(u8 b)
{
	return (b * MAX_CURRENT) / 255;
}

void ktd2037_turn_off_all_leds(void)
{
#ifdef LIGHTS_DBG_ON
	printk("%s\n", __func__);
#endif
	I2C_WRITE(0x00, 0x05);           // initialization LED off
	I2C_WRITE(KTD_REG_LED_ID, 0x00); // initialization LED off
	I2C_WRITE(KTD_REG_LED_ID, 0x00); // turn off all LEDs
}

void ktd2037_execute_operation(struct ktd2037_priv *obj)
{
	u8 led_id = 0;
	u8 cr = brightness_to_steps(obj->red_brightness);
	u8 cg = brightness_to_steps(obj->green_brightness);
	u8 cb = brightness_to_steps(obj->blue_brightness);
	u8 period = 0;

	if (!obj->red_brightness && !obj->green_brightness && !obj->blue_brightness) {
		ktd2037_turn_off_all_leds();
		return;
	}

	switch(obj->flash_mode) {
	case LIGHT_FLASH_TIMED:
		/* atm only fixed period, until we guess how to set it up */
	case LIGHT_FLASH_HARDWARE:
		if (obj->red_brightness) {
			led_id |= LED_RED_TIMER;
		}
		if (obj->green_brightness) {
			led_id |= LED_GREEN_TIMER;
		}
		if (obj->blue_brightness) {
			led_id |= LED_BLUE_TIMER;
		}
		period = 0x12;
		break;

	default:
		if (obj->red_brightness) {
			led_id |= LED_RED;
		}
		if (obj->green_brightness) {
			led_id |= LED_GREEN;
		}
		if (obj->blue_brightness) {
			led_id |= LED_BLUE;
		}
	}

#ifdef LIGHTS_DBG_ON
	printk("%s led_id(%x) current(%x,%x,%x) period(%x)\n", __func__, led_id, cr, cg, cb, period);
#endif

	I2C_WRITE(0x00, 0x05);// initialization LED off
	I2C_WRITE(KTD_REG_LED_ID, 0x00);// initialization LED off
	I2C_WRITE(0x09, 0x06);// disable auto blink

	if (period) {
		/* flashing */
		I2C_WRITE(KTD_REG_CURRENT_RED, cr);//set current
		I2C_WRITE(KTD_REG_CURRENT_GREEN, cg);//set current
		I2C_WRITE(KTD_REG_CURRENT_BLUE, cb);//set current
		I2C_WRITE(KTD_REG_DRY_FLASH_PERIOD, period);//dry flash period
		I2C_WRITE(KTD_REG_FLASH_COUNTER, 0x00);//reset internal counter
		I2C_WRITE(KTD_REG_LED_ID, led_id);//turn on led
		I2C_WRITE(KTD_REG_FLASH_COUNTER, 0x32);//led flashing(current ramp-up and down countinuously)
	} else {
		/* solid color */
		I2C_WRITE(KTD_REG_CURRENT_RED, cr);//set current
		I2C_WRITE(KTD_REG_CURRENT_GREEN, cg);//set current
		I2C_WRITE(KTD_REG_CURRENT_BLUE, cb);//set current
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

	return sprintf(buf, "%d\n", obj->flash_period);
}

static ssize_t ktd2037_store_flash_period(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct i2c_client *client;
	struct ktd2037_priv *obj = NULL;
	unsigned int value;

	client = to_i2c_client(dev);
	obj = i2c_get_clientdata(client);

	ret = kstrtouint(buf, 10, &value);
	if (ret < 0) {
		dev_err(dev, "%s: value too large!\n", __func__);
		return ret;
	}
	obj->flash_period = value;

#ifdef LIGHTS_DBG_ON
	printk("%s: value(%d)\n", __func__, value);
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
