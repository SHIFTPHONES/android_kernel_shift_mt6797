#include <linux/kernel.h> //constant xx
#include <linux/module.h> 
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
//#include <linux/xlog.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif

#define TAG_NAME "speaker_pa_gpio.c"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        xlog_printk(ANDROID_LOG_WARNING, TAG_NAME, KERN_WARNING  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_NOTICE  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        xlog_printk(ANDROID_LOG_INFO   , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME,  "<%s>\n", __FUNCTION__);
#define PK_TRC_VERBOSE(fmt, arg...) xlog_printk(ANDROID_LOG_VERBOSE, TAG_NAME,  fmt, ##arg)
#define PK_ERROR(fmt, arg...)       xlog_printk(ANDROID_LOG_ERROR  , TAG_NAME, KERN_ERR "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_SPEAKER
#ifdef  DEBUG_SPEAKER
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif



#define SPEAKER_PA_GPIO_DEVICE "speakerpa-gpio"
static struct pinctrl *speakerpa_gpio;
//static struct pinctrl_state *speakerpa_gpio_mode_default;
static struct pinctrl_state *speakerpa_gpio_en_h,*speakerpa_gpio_en_l;


static const struct of_device_id _speakerpa_gpio_of_ids[] = {
	{.compatible = "mediatek,speaker_pa",},
	{}
};


static int speakerpa_gpio_probe(struct device *dev)
{
	int ret;

	printk("mly speakerpa_gpio_probe!\n");

	speakerpa_gpio = devm_pinctrl_get(dev);
	if (IS_ERR(speakerpa_gpio)) {
		ret = PTR_ERR(speakerpa_gpio);
		dev_err(dev, "[ERROR] Cannot find speakerpa_gpio %d!\n", ret);
		return ret;
	}//else
		//printk("mly speakerpa_gpio_probe devm_pinctrl_get dev ok!\n ");
	
	//speakerpa_gpio_mode_default = pinctrl_lookup_state(speakerpa_gpio, "pin_default");
	//if (IS_ERR(speakerpa_gpio_mode_default)) {
		//ret = PTR_ERR(speakerpa_gpio_mode_default);
		//dev_err(dev, "[ERROR] Cannot find speakerpa_gpio_mode_default %d!\n", ret);
	//}//else
		//printk("mly speakerpa_gpio_probe speakerpa_gpio_mode_default  ok!\n ");
	
	speakerpa_gpio_en_h = pinctrl_lookup_state(speakerpa_gpio, "speaker_pa_en_high");
	if (IS_ERR(speakerpa_gpio_en_h)) {
		ret = PTR_ERR(speakerpa_gpio_en_h);
		dev_err(dev, "[ERROR] Cannot find speakerpa_gpio_en_h:%d!\n", ret);
		return ret;
	}//else
		//printk("mly speakerpa_gpio_probe speakerpa_gpio_power_h  ok!\n ");
	
	speakerpa_gpio_en_l = pinctrl_lookup_state(speakerpa_gpio, "speaker_pa_en_low");
	if (IS_ERR(speakerpa_gpio_en_l)) {
		ret = PTR_ERR(speakerpa_gpio_en_l);
		dev_err(dev, "[ERROR] Cannot find speakerpa_gpio_en_l:%d!\n", ret);
		return ret;
	}//else
		//printk("mly speakerpa_gpio_probe speakerpa_gpio_shutdown_l  ok!\n ");

	printk("mly speakerpa_gpio_probe end ok!\n");
	return 0;
}
        

static int speakerpa_gpio_remove(struct device *dev)
{
	return 0;
}


static struct platform_driver speakerpa_gpio_driver = {
	.driver = {
		   .name =SPEAKER_PA_GPIO_DEVICE,
		   .probe = speakerpa_gpio_probe,
		   .remove = speakerpa_gpio_remove,
		   .of_match_table = _speakerpa_gpio_of_ids,
		   },
};



/* called when loaded into kernel */
static int __init speakerpa_gpio_init(void)
{
	printk("mly speakerpa_gpio_init\n");
	if (platform_driver_register(&speakerpa_gpio_driver) != 0) {
		pr_err("unable to register led GPIO driver.\n");
		return -1;
	}
	return 0;
}


/* should never be called */
static void __exit speakerpa_gpio_exit(void)
{
	printk("mly speakerpa_gpio_exit\n");
	platform_driver_unregister(&speakerpa_gpio_driver);
}


module_init(speakerpa_gpio_init);
module_exit(speakerpa_gpio_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek cmx148 GPIO driver");
MODULE_AUTHOR("Joey Pan<joey.pan@mediatek.com>");





/*****************************************************************************
User interface
*****************************************************************************/




void speaker_pa_enable(bool on)
{
     printk("mly speaker_pa_enable:%d!\n",on);

    if (on)
       {      
        pinctrl_select_state(speakerpa_gpio, speakerpa_gpio_en_h);
       }
    else
        {
//        printk("mly cmx148_shutdown off\n");
        pinctrl_select_state(speakerpa_gpio, speakerpa_gpio_en_l);
        }


}



