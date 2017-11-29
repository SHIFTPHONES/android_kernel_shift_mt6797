#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include "lcm_drv.h"
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#endif

//#define MDELAY(n)		(lcm_util.mdelay(n))

static struct pinctrl *lcm_power_gpio;
struct pinctrl_state *pin_lcm_power_18_on = NULL;
struct pinctrl_state *pin_lcm_power_33_on = NULL;
struct pinctrl_state *pin_lcm_power_18_off = NULL;
struct pinctrl_state *pin_lcm_power_33_off = NULL;
struct pinctrl_state *pin_lcm_reset_high = NULL;
struct pinctrl_state *pin_lcm_reset_low = NULL;

static const struct of_device_id lcm_power_of_ids[] = {
 {.compatible = "mediatek,lcm_power",},
 {}
};

static int lcm_power_probe(struct device *dev)
{
   int ret;
 printk("mly lcm_power_probe\n");
lcm_power_gpio = devm_pinctrl_get(dev);
	if (IS_ERR(lcm_power_gpio)) {
		ret = PTR_ERR(lcm_power_gpio);
		dev_err(dev, "fwq Cannot find lcm pinctrl1!\n");
             return -1;
	}
 pin_lcm_power_18_on = pinctrl_lookup_state(lcm_power_gpio, "pin_lcm_power_18_on");
 if (IS_ERR(pin_lcm_power_18_on)) {
  pr_debug("%s : pinctrl err, pin_lcm_power_on\n", __func__);
 }

 pin_lcm_power_18_off = pinctrl_lookup_state(lcm_power_gpio, "pin_lcm_power_18_off");
 if (IS_ERR(pin_lcm_power_18_off)) {
  pr_debug("%s : pinctrl err, pin_lcm_power_off\n", __func__);
 }

 pin_lcm_power_33_on = pinctrl_lookup_state(lcm_power_gpio, "pin_lcm_power_33_on");
 if (IS_ERR(pin_lcm_power_33_on)) {
  pr_debug("%s : pinctrl err, pin_lcm_power_on\n", __func__);
 }

 pin_lcm_power_33_off = pinctrl_lookup_state(lcm_power_gpio, "pin_lcm_power_33_off");
 if (IS_ERR(pin_lcm_power_33_off)) {
  pr_debug("%s : pinctrl err, pin_lcm_power_off\n", __func__);
 }

 pin_lcm_reset_high = pinctrl_lookup_state(lcm_power_gpio, "pin_lcm_reset_high");
 if (IS_ERR(pin_lcm_reset_high)) {
  pr_debug("%s : pinctrl err, pin_lcm_reset_high\n", __func__);
 }

 pin_lcm_reset_low = pinctrl_lookup_state(lcm_power_gpio, "pin_lcm_reset_low");
 if (IS_ERR(pin_lcm_reset_low)) {
  pr_debug("%s : pinctrl err, pin_lcm_reset_low\n", __func__);
 }
 return 0;
}


static int lcm_power_remove(struct device *dev)
{
 return 0;
 }

static struct platform_driver lcm_power_driver = {
	.driver = {
		   .name = "lcm_power_driver",
		   .probe = lcm_power_probe,
		   .remove = lcm_power_remove,
		   .of_match_table = lcm_power_of_ids,
		   },
};

static int __init lcm_power_init(void)
{
	printk("mly----Simcom LCM GPIO driver init\n");
	if (platform_driver_register(&lcm_power_driver) != 0) {
		pr_err("unable to register LCM GPIO driver.\n");
		return -1;
	}
	return 0;
}

static void __exit lcm_power_exit(void)
{
	printk("mly----Simcom LCM GPIO driver exit\n");
	platform_driver_unregister(&lcm_power_driver);

}
module_init(lcm_power_init);
module_exit(lcm_power_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Simcom LCM GPIO driver");
MODULE_AUTHOR("PAN LONG<long.pan@sim.com>");



/*****************************************************************************
User io
*****************************************************************************/
 void lcm_gpio_power_off(void)
 {
  pinctrl_select_state(lcm_power_gpio,pin_lcm_power_18_off);
  
  pinctrl_select_state(lcm_power_gpio,pin_lcm_power_33_off);
  printk("dongsheng------line is   :%d        function is   :%s\n",__LINE__,__FUNCTION__);
 }

 void lcm_gpio_power_on(void)
 {
  pinctrl_select_state(lcm_power_gpio,pin_lcm_power_18_on);

  pinctrl_select_state(lcm_power_gpio,pin_lcm_power_33_on);

  printk("dongsheng------line is   :%d        function is   :%s\n",__LINE__,__FUNCTION__);
 }

 void lcm_gpio_reset(int i)
 {
if(i==1){
  pinctrl_select_state(lcm_power_gpio,pin_lcm_reset_high);
}else if(i==0){
  pinctrl_select_state(lcm_power_gpio,pin_lcm_reset_low);
}
else{
	 printk("dongsheng--reset-error----line is   :%d        function is   :%s\n",__LINE__,__FUNCTION__);
	}
  	printk("dongsheng------line is   :%d        function is   :%s\n",__LINE__,__FUNCTION__);
 }

