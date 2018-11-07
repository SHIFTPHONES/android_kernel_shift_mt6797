#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>

#include "lcm_drv.h"
#include "ddp_irq.h"

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#endif

#define LCM_POWER_GPIO_DEVICE "lcm-power-gpio"

static struct pinctrl *lcm_power_gpio;
static struct pinctrl_state *lcm_power_18_en0,*lcm_power_18_en1,
							*lcm_power_enn0,*lcm_power_enn1, *lcm_power_enp0,*lcm_power_enp1;
//*pins_default,

static const struct of_device_id lcm_gpio_of_ids[] = {
	{.compatible = "mediatek,lcm_gpio",},
	{}
};

static int lcm_gpio_probe(struct device *dev)
{
	int ret;
	printk("[lcy] lcm_gpio_probe+++++++++++++++++\n");
	lcm_power_gpio = devm_pinctrl_get(dev);
	if (IS_ERR(lcm_power_gpio)) {
		ret = PTR_ERR(lcm_power_gpio);
		dev_err(dev, "lcy Cannot find lcm pinctrl1!\n");
             return -1;
	}
	lcm_power_18_en0 = pinctrl_lookup_state(lcm_power_gpio, "lcm_power_18_en0");
	if (IS_ERR(lcm_power_18_en0)) {
		ret = PTR_ERR(lcm_power_18_en0);
		dev_err(dev, "lcy Cannot find lcm pinctrl lcm_power_18_en0!\n");
	}
	lcm_power_18_en1 = pinctrl_lookup_state(lcm_power_gpio, "lcm_power_18_en1");
	if (IS_ERR(lcm_power_18_en1)) {
		ret = PTR_ERR(lcm_power_18_en1);
		dev_err(dev, "lcy Cannot find lcm pinctrl lcm_power_18_en1!\n");
	}

      lcm_power_enn0 = pinctrl_lookup_state(lcm_power_gpio, "lcm_power_enn0");
      if (IS_ERR(lcm_power_enn0)) {
          ret = PTR_ERR(lcm_power_enn0);
          dev_err(dev, "lcy Cannot find lcm pinctrl lcm_power_enn0!\n");
      }
      lcm_power_enn1 = pinctrl_lookup_state(lcm_power_gpio, "lcm_power_enn1");
      if (IS_ERR(lcm_power_enn1)) {
          ret = PTR_ERR(lcm_power_enn1);
          dev_err(dev, "lcy Cannot find lcm pinctrl lcm_power_enn1!\n");
      }

      lcm_power_enp0 = pinctrl_lookup_state(lcm_power_gpio, "lcm_power_enp0");
      if (IS_ERR(lcm_power_enp0)) {
          ret = PTR_ERR(lcm_power_enp0);
          dev_err(dev, "lcy Cannot find lcm pinctrl lcm_power_enp0!\n");
      }
      lcm_power_enp1 = pinctrl_lookup_state(lcm_power_gpio, "lcm_power_enp1");
      if (IS_ERR(lcm_power_enp1)) {
          ret = PTR_ERR(lcm_power_enp1);
          dev_err(dev, "lcy Cannot find lcm pinctrl lcm_power_enp1!\n");
      } 
	
	printk("lcy LCM_gpio_probe end ok!\n");
	return 0;
}

static int lcm_gpio_remove(struct device *dev)
{
	return 0;
}

static struct platform_driver lcm_gpio_driver = {
	.driver = {
		   .name = LCM_POWER_GPIO_DEVICE,
		   .probe = lcm_gpio_probe,
		   .remove = lcm_gpio_remove,
		   .of_match_table = lcm_gpio_of_ids,
		   },
};

static int __init lcm_gpio_init(void)
{
	printk("lcy----Simcom LCM GPIO driver init\n");
	if (platform_driver_register(&lcm_gpio_driver) != 0) {
		pr_err("unable to register LCM GPIO driver.\n");
		return -1;
	}
	return 0;
}

static void __exit lcm_gpio_exit(void)
{
	printk("lcy----Simcom LCM GPIO driver exit\n");
	platform_driver_unregister(&lcm_gpio_driver);

}
module_init(lcm_gpio_init);
module_exit(lcm_gpio_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Simcom LCM GPIO driver");
MODULE_AUTHOR("PAN LONG<long.pan@sim.com>");



/*****************************************************************************
User io
*****************************************************************************/

void lcm_18_pin_enable(bool on) 
{
	printk("mly---lcm_18_pin_enable:%d!\n",on);
	if (on)   
	{      
		pinctrl_select_state(lcm_power_gpio, lcm_power_18_en1);
	}
	else 
	{
		pinctrl_select_state(lcm_power_gpio, lcm_power_18_en0);
	}
}

void lcm_power_enn_enable(bool on) 
{
	printk("lcy----lcm_power_enn_enable:%d!\n",on);
	if (on)   
	{
		pinctrl_select_state(lcm_power_gpio, lcm_power_enn1);
	}
	else 
	{
		pinctrl_select_state(lcm_power_gpio, lcm_power_enn0);
	}
}

void lcm_power_enp_enable(bool on) 
{
	printk("lcy----lcm_power_enp_enable:%d!\n",on);
	if (on)   
	{
		pinctrl_select_state(lcm_power_gpio, lcm_power_enp1);
	}
	else 
	{
		pinctrl_select_state(lcm_power_gpio, lcm_power_enp0);
	}
}

