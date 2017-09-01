/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/

#ifdef CONFIG_USB_MTK_OTG
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/gpio.h>
#include <linux/io.h>
#ifndef CONFIG_OF
#include <mach/irqs.h>
#endif
#if defined(CONFIG_MTK_LEGACY)
#include <mt-plat/mt_gpio.h>
#include <cust_gpio_usage.h>
#endif
#include "musb_core.h"
#include <linux/platform_device.h>
#include "musbhsdma.h"
#include <linux/switch.h>
#include "usb20.h"
#ifdef CONFIG_OF
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

#include <mt-plat/mt_boot_common.h>

#ifdef CONFIG_OF
struct device_node		*usb_node;
static unsigned int iddig_pin;
static unsigned int iddig_pin_mode;
static unsigned int iddig_if_config = 1;
static unsigned int drvvbus_pin;
static unsigned int drvvbus_pin_mode;
static unsigned int drvvbus_if_config = 1;
#endif

static int usb_iddig_number;
#if !defined(CONFIG_MTK_LEGACY)
struct pinctrl *pinctrl;
struct pinctrl_state *pinctrl_iddig;
struct pinctrl_state *pinctrl_drvvbus;
struct pinctrl_state *pinctrl_drvvbus_low;
struct pinctrl_state *pinctrl_drvvbus_high;

#ifdef CONFIG_USB_MTK_OTG_SWITCH
static bool otg_switch_state;
static bool otg_iddig_isr_enable;
static struct pinctrl_state *pinctrl_iddig_init;
static struct pinctrl_state *pinctrl_iddig_enable;
static struct pinctrl_state *pinctrl_iddig_disable;
static struct mutex otg_switch_mutex;

static irqreturn_t mt_usb_ext_iddig_int(int irq, void *dev_id);
static int mtk_musb_eint_iddig_irq_en(void);

static ssize_t otg_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t otg_mode_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count);

DEVICE_ATTR(otg_mode, 0664, otg_mode_show, otg_mode_store);

static struct attribute *otg_attributes[] = {
	&dev_attr_otg_mode.attr,
	NULL
};

static const struct attribute_group otg_attr_group = {
	.attrs = otg_attributes,
};

static ssize_t otg_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (!dev) {
		dev_err(dev, "otg_mode_store no dev\n");
		return 0;
	}

	return sprintf(buf, "%d\n", otg_switch_state);
}

static ssize_t otg_mode_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned int mode;

	if (!dev) {
		dev_err(dev, "otg_mode_store no dev\n");
		return count;
	}

	mutex_lock(&otg_switch_mutex);

	if (1 == sscanf(buf, "%ud", &mode)) {
		if (mode == 0) {
			pr_info("otg_mode_disable start\n");

			if (otg_switch_state == true) {

				if (otg_iddig_isr_enable) {

					pr_info("%s - disable_irq - is_host_active %d, is_active %d\n",
						__func__, is_host_active(mtk_musb),	mtk_musb->is_active);

					disable_irq(usb_iddig_number);
					free_irq(usb_iddig_number, NULL);
					otg_iddig_isr_enable = false;
					if (is_host_active(mtk_musb))
						schedule_delayed_work(&mtk_musb->id_pin_work, HZ/1000);
				}

				if (!IS_ERR(pinctrl_iddig_disable))
					pinctrl_select_state(pinctrl, pinctrl_iddig_disable);

				otg_switch_state = false;
			}
			pr_debug("otg_mode_disable end\n");
		} else {
			pr_debug("otg_mode_enable start\n");

			if (otg_switch_state == false) {

				otg_switch_state = true;

				if (!IS_ERR(pinctrl_iddig_enable))
					pinctrl_select_state(pinctrl, pinctrl_iddig_enable);

				msleep(200);
				pr_info("%s - enable_irq\n", __func__);
				mtk_musb_eint_iddig_irq_en();
			}
			pr_debug("otg_mode_enable end\n");
		}
	}
	mutex_unlock(&otg_switch_mutex);
	return count;
}

static int mtk_musb_eint_iddig_irq_en(void)
{
	int retval = 0;

	pr_info("%s - otg_iddig_isr_enable %d\n", __func__, otg_iddig_isr_enable);
	if (!otg_iddig_isr_enable) {
		retval = request_irq(usb_iddig_number, mt_usb_ext_iddig_int, IRQF_TRIGGER_LOW, "USB_IDDIG", NULL);

		if (retval != 0) {
			pr_notice("request_irq fail, ret %d, irqnum %d!!!\n", retval,
					 usb_iddig_number);
		} else {
			otg_iddig_isr_enable = true;
			enable_irq_wake(usb_iddig_number);
		}
	} else
		switch_int_to_host(mtk_musb);  /* restore ID pin interrupt */
	return retval;
}
#endif
#endif
static ktime_t ktime_start, ktime_end;

static struct musb_fifo_cfg fifo_cfg_host[] = {
{ .hw_ep_num =  1, .style = MUSB_FIFO_TX,   .maxpacket = 512, .mode = MUSB_BUF_SINGLE},
{ .hw_ep_num =  1, .style = MUSB_FIFO_RX,   .maxpacket = 512, .mode = MUSB_BUF_SINGLE},
{ .hw_ep_num =  2, .style = MUSB_FIFO_TX,   .maxpacket = 512, .mode = MUSB_BUF_SINGLE},
{ .hw_ep_num =  2, .style = MUSB_FIFO_RX,   .maxpacket = 512, .mode = MUSB_BUF_SINGLE},
{ .hw_ep_num =  3, .style = MUSB_FIFO_TX,   .maxpacket = 512, .mode = MUSB_BUF_SINGLE},
{ .hw_ep_num =  3, .style = MUSB_FIFO_RX,   .maxpacket = 512, .mode = MUSB_BUF_SINGLE},
{ .hw_ep_num =  4, .style = MUSB_FIFO_TX,   .maxpacket = 512, .mode = MUSB_BUF_SINGLE},
{ .hw_ep_num =  4, .style = MUSB_FIFO_RX,   .maxpacket = 512, .mode = MUSB_BUF_SINGLE},
{ .hw_ep_num =  5, .style = MUSB_FIFO_TX,   .maxpacket = 512, .mode = MUSB_BUF_SINGLE},
{ .hw_ep_num =	5, .style = MUSB_FIFO_RX,   .maxpacket = 512, .mode = MUSB_BUF_SINGLE},
{ .hw_ep_num =  6, .style = MUSB_FIFO_TX,   .maxpacket = 512, .mode = MUSB_BUF_SINGLE},
{ .hw_ep_num =	6, .style = MUSB_FIFO_RX,   .maxpacket = 512, .mode = MUSB_BUF_SINGLE},
{ .hw_ep_num =	7, .style = MUSB_FIFO_TX,   .maxpacket = 512, .mode = MUSB_BUF_SINGLE},
{ .hw_ep_num =	7, .style = MUSB_FIFO_RX,   .maxpacket = 512, .mode = MUSB_BUF_SINGLE},
{ .hw_ep_num =	8, .style = MUSB_FIFO_TX,   .maxpacket = 512, .mode = MUSB_BUF_SINGLE},
{ .hw_ep_num =	8, .style = MUSB_FIFO_RX,   .maxpacket = 64,  .mode = MUSB_BUF_SINGLE},
};

u32 delay_time = 15;
module_param(delay_time, int, 0644);
u32 delay_time1 = 55;
module_param(delay_time1, int, 0644);
u32 iddig_cnt = 0;
module_param(iddig_cnt, int, 0644);

void mt_usb_set_vbus(struct musb *musb, int is_on)
{
	DBG(0, "mt65xx_usb20_vbus++,is_on=%d\r\n", is_on);
#ifndef FPGA_PLATFORM
	if (is_on) {
		/* power on VBUS, implement later... */
	#ifdef CONFIG_MTK_FAN5405_SUPPORT
		fan5405_set_opa_mode(1);
		fan5405_set_otg_pl(1);
		fan5405_set_otg_en(1);
	#elif defined(CONFIG_MTK_BQ24261_SUPPORT)
		bq24261_set_en_boost(1);
	#elif defined(CONFIG_MTK_BQ24296_SUPPORT)
		bq24296_set_otg_config(0x1); /* OTG */
		bq24296_set_boostv(0x7); /* boost voltage 4.998V */
		bq24296_set_boost_lim(0x1); /* 1.5A on VBUS */
		bq24296_set_en_hiz(0x0);
	#elif defined(CONFIG_MTK_BQ24196_SUPPORT)
		bq24196_set_otg_config(0x01);	/* OTG */
		bq24196_set_boost_lim(0x01);	/* 1.3A on VBUS */
	#elif defined(CONFIG_MTK_NCP1854_SUPPORT)
		ncp1854_set_otg_en(0);
		ncp1854_set_chg_en(0);
		ncp1854_set_otg_en(1);
	#else
		#ifdef CONFIG_OF
		#if defined(CONFIG_MTK_LEGACY)
		mt_set_gpio_mode(drvvbus_pin, drvvbus_pin_mode);
		mt_set_gpio_out(drvvbus_pin, GPIO_OUT_ONE);
		#else
		pr_debug("****%s:%d Drive VBUS HIGH KS!!!!!\n", __func__, __LINE__);
		pinctrl_select_state(pinctrl, pinctrl_drvvbus_high);
		#endif
		#else
		mt_set_gpio_mode(GPIO_OTG_DRVVBUS_PIN, GPIO_OTG_DRVVBUS_PIN_M_GPIO);
		mt_set_gpio_out(GPIO_OTG_DRVVBUS_PIN, GPIO_OUT_ONE);
		#endif
		#endif
	} else {
		/* power off VBUS, implement later... */
	#ifdef CONFIG_MTK_FAN5405_SUPPORT
		fan5405_reg_config_interface(0x01, 0x30);
		fan5405_reg_config_interface(0x02, 0x8e);
	#elif defined(CONFIG_MTK_BQ24261_SUPPORT)
		bq24261_set_en_boost(0);
	#elif defined(CONFIG_MTK_BQ24296_SUPPORT)
		bq24296_set_otg_config(0);
	#elif defined(CONFIG_MTK_BQ24196_SUPPORT)
		bq24196_set_otg_config(0x0);	/* OTG disabled */
	#elif defined(CONFIG_MTK_NCP1854_SUPPORT)
		ncp1854_set_otg_en(0x0);
	#else
		#ifdef CONFIG_OF
		#if defined(CONFIG_MTK_LEGACY)
		mt_set_gpio_mode(drvvbus_pin, drvvbus_pin_mode);
		mt_set_gpio_out(drvvbus_pin, GPIO_OUT_ZERO);
		#else
		pr_debug("****%s:%d Drive VBUS LOW KS!!!!!\n", __func__, __LINE__);
		pinctrl_select_state(pinctrl, pinctrl_drvvbus_low);
		#endif
		#else
		mt_set_gpio_mode(GPIO_OTG_DRVVBUS_PIN, GPIO_OTG_DRVVBUS_PIN_M_GPIO);
		mt_set_gpio_out(GPIO_OTG_DRVVBUS_PIN, GPIO_OUT_ZERO);
		#endif
	#endif
	}
#endif
}

int mt_usb_get_vbus_status(struct musb *musb)
{
#if 1
	return true;
#else
	int	ret = 0;

	if ((musb_readb(musb->mregs, MUSB_DEVCTL) & MUSB_DEVCTL_VBUS) != MUSB_DEVCTL_VBUS)
		ret = 1;
	else
		DBG(0, "VBUS error, devctl=%x, power=%d\n", musb_readb(musb->mregs, MUSB_DEVCTL), musb->power);
	pr_debug("vbus ready = %d\n", ret);
	return ret;
#endif
}

void mt_usb_init_drvvbus(void)
{
#if !(defined(SWITCH_CHARGER) || defined(FPGA_PLATFORM))
	#ifdef CONFIG_OF
	#if defined(CONFIG_MTK_LEGACY)
	mt_set_gpio_mode(drvvbus_pin, drvvbus_pin_mode); /* should set GPIO2 as gpio mode. */
	mt_set_gpio_dir(drvvbus_pin, GPIO_DIR_OUT);
	mt_get_gpio_pull_enable(drvvbus_pin);
	mt_set_gpio_pull_select(drvvbus_pin, GPIO_PULL_UP);
	#else
	int ret = 0;

	pr_debug("****%s:%d before Init Drive VBUS KS!!!!!\n", __func__, __LINE__);

	pinctrl_drvvbus = pinctrl_lookup_state(pinctrl, "drvvbus_init");
	if (IS_ERR(pinctrl_drvvbus)) {
		ret = PTR_ERR(pinctrl_drvvbus);
		dev_err(mtk_musb->controller, "Cannot find usb pinctrl drvvbus\n");
	}

	pinctrl_drvvbus_low = pinctrl_lookup_state(pinctrl, "drvvbus_low");
	if (IS_ERR(pinctrl_drvvbus_low)) {
		ret = PTR_ERR(pinctrl_drvvbus_low);
		dev_err(mtk_musb->controller, "Cannot find usb pinctrl drvvbus_low\n");
	}

	pinctrl_drvvbus_high = pinctrl_lookup_state(pinctrl, "drvvbus_high");
	if (IS_ERR(pinctrl_drvvbus_high)) {
		ret = PTR_ERR(pinctrl_drvvbus_high);
		dev_err(mtk_musb->controller, "Cannot find usb pinctrl drvvbus_high\n");
	}

	pinctrl_select_state(pinctrl, pinctrl_drvvbus);
	pr_debug("****%s:%d end Init Drive VBUS KS!!!!!\n", __func__, __LINE__);
	#endif
	#else
	mt_set_gpio_mode(GPIO_OTG_DRVVBUS_PIN, GPIO_OTG_DRVVBUS_PIN_M_GPIO); /* should set GPIO2 as gpio mode. */
	mt_set_gpio_dir(GPIO_OTG_DRVVBUS_PIN, GPIO_DIR_OUT);
	mt_get_gpio_pull_enable(GPIO_OTG_DRVVBUS_PIN);
	mt_set_gpio_pull_select(GPIO_OTG_DRVVBUS_PIN, GPIO_PULL_UP);
#endif
#endif
}

#if defined(CONFIG_USBIF_COMPLIANCE)
u32 sw_deboun_time = 1;
#else
u32 sw_deboun_time = 400;
#endif
module_param(sw_deboun_time, int, 0644);
struct switch_dev otg_state;

static bool musb_is_host(void)
{
	u8 devctl = 0;
	int iddig_state = 1;
	bool usb_is_host = 0;

	DBG(0, "will mask PMIC charger detection\n");
#ifndef FPGA_PLATFORM
	pmic_chrdet_int_en(0);
#endif

	musb_platform_enable(mtk_musb);

#ifdef ID_PIN_USE_EX_EINT
#ifndef CONFIG_MTK_FPGA
	#ifdef CONFIG_OF
	#if defined(CONFIG_MTK_LEGACY)
	iddig_state = mt_get_gpio_in(iddig_pin);
	#else
	iddig_state = __gpio_get_value(iddig_pin);
	#endif
	#else
	iddig_state = mt_get_gpio_in(GPIO_OTG_IDDIG_EINT_PIN);
	#endif
	DBG(0, "iddig_state = %d\n", iddig_state);
#endif
#else
	iddig_state = 0;
	devctl = musb_readb(mtk_musb->mregs, MUSB_DEVCTL);
	DBG(0, "devctl = %x before end session\n", devctl);
	devctl &= ~MUSB_DEVCTL_SESSION;	/* this will cause A-device change back to B-device after A-cable plug out */
	musb_writeb(mtk_musb->mregs, MUSB_DEVCTL, devctl);
	msleep(delay_time);

	devctl = musb_readb(mtk_musb->mregs, MUSB_DEVCTL);
	DBG(0, "devctl = %x before set session\n", devctl);

	devctl |= MUSB_DEVCTL_SESSION;
	musb_writeb(mtk_musb->mregs, MUSB_DEVCTL, devctl);
	msleep(delay_time1);
	devctl = musb_readb(mtk_musb->mregs, MUSB_DEVCTL);
	DBG(0, "devclt = %x\n", devctl);
#endif

	if (devctl & MUSB_DEVCTL_BDEVICE || iddig_state) {
		DBG(0, "will unmask PMIC charger detection\n");
#ifndef FPGA_PLATFORM
		pmic_chrdet_int_en(1);
#endif
		usb_is_host = false;
	} else {
		usb_is_host = true;
#if defined(CONFIG_USB_MTK_OTG_SWITCH)
		pr_info("%s - otg_iddig_isr_enable %d, otg_switch_state %d, usb_is_host %d\n", __func__,
				otg_iddig_isr_enable, otg_switch_state, usb_is_host);
		if (!otg_iddig_isr_enable) {
			#ifndef FPGA_PLATFORM
					DBG(0, "will unmask PMIC charger detection\n");
					pmic_chrdet_int_en(1);
			#endif
			usb_is_host = false;
		}
#endif
	}

	DBG(0, "usb_is_host = %d\n", usb_is_host);
	return usb_is_host;
}

void musb_session_restart(struct musb *musb)
{
	void __iomem	*mbase = musb->mregs;

	musb_writeb(mbase, MUSB_DEVCTL, (musb_readb(mbase, MUSB_DEVCTL) & (~MUSB_DEVCTL_SESSION)));
	DBG(0, "[MUSB] stopped session for VBUSERROR interrupt\n");
	USBPHY_SET8(0x6d, 0x3c);
	USBPHY_SET8(0x6c, 0x10);
	USBPHY_CLR8(0x6c, 0x2c);
	DBG(0, "[MUSB] force PHY to idle, 0x6d=%x, 0x6c=%x\n", USBPHY_READ8(0x6d), USBPHY_READ8(0x6c));
	mdelay(5);
	USBPHY_CLR8(0x6d, 0x3c);
	USBPHY_CLR8(0x6c, 0x3c);
	DBG(0, "[MUSB] let PHY resample VBUS, 0x6d=%x, 0x6c=%x\n", USBPHY_READ8(0x6d), USBPHY_READ8(0x6c));
	musb_writeb(mbase, MUSB_DEVCTL, (musb_readb(mbase, MUSB_DEVCTL) | MUSB_DEVCTL_SESSION));
	DBG(0, "[MUSB] restart session\n");
}

static struct delayed_work host_plug_test_work;
int host_plug_test_enable; /* default disable */
module_param(host_plug_test_enable, int, 0644);
int host_plug_in_test_period_ms = 5000;
module_param(host_plug_in_test_period_ms, int, 0644);
int host_plug_out_test_period_ms = 5000;
module_param(host_plug_out_test_period_ms, int, 0644);
int host_test_vbus_off_time_us = 3000;
module_param(host_test_vbus_off_time_us, int, 0644);
int host_test_vbus_only = 1;
module_param(host_test_vbus_only, int, 0644);
static int host_plug_test_triggered;
void switch_int_to_device(struct musb *musb)
{
	if (host_plug_test_triggered) {
		DBG(1, "directly return\n");
		return;
	}

	if (!usb_iddig_number) {
		DBG(0, "OTG not inited, directly return\n");
		return;
	}
#ifdef ID_PIN_USE_EX_EINT
	irq_set_irq_type(usb_iddig_number, IRQF_TRIGGER_HIGH);
	enable_irq(usb_iddig_number);
#else
	 musb_writel(musb->mregs, USB_L1INTP, 0);
	 musb_writel(musb->mregs, USB_L1INTM, IDDIG_INT_STATUS|musb_readl(musb->mregs, USB_L1INTM));
#endif
	 DBG(0, "switch_int_to_device is done\n");
}

void switch_int_to_host(struct musb *musb)
{
	if (host_plug_test_triggered) {
		DBG(1, "directly return\n");
		return;
	}

	if (!usb_iddig_number) {
		DBG(0, "OTG not inited, directly return\n");
		return;
	}
#ifdef ID_PIN_USE_EX_EINT
	irq_set_irq_type(usb_iddig_number, IRQF_TRIGGER_LOW);
	enable_irq(usb_iddig_number);
#else
	musb_writel(musb->mregs, USB_L1INTP, IDDIG_INT_STATUS);
	musb_writel(musb->mregs, USB_L1INTM, IDDIG_INT_STATUS|musb_readl(musb->mregs, USB_L1INTM));
#endif
	DBG(0, "switch_int_to_host is done\n");

}

void switch_int_to_host_and_mask(struct musb *musb)
{
	if (host_plug_test_triggered) {
		DBG(1, "directly return\n");
		return;
	}

	if (!usb_iddig_number) {
		DBG(0, "OTG not inited, directly return\n");
		return;
	}
#ifdef ID_PIN_USE_EX_EINT
	disable_irq(usb_iddig_number);
	irq_set_irq_type(usb_iddig_number, IRQF_TRIGGER_LOW);
#else
	musb_writel(musb->mregs, USB_L1INTM, (~IDDIG_INT_STATUS)&musb_readl(musb->mregs, USB_L1INTM));
	mb();
	musb_writel(musb->mregs, USB_L1INTP, IDDIG_INT_STATUS);
#endif
	DBG(0, "swtich_int_to_host_and_mask is done\n");
}
static void do_host_plug_test_work(struct work_struct *data)
{
	static ktime_t ktime_begin, ktime_end;
	static s64 diff_time;
	static int host_on;
	static struct wake_lock host_test_wakelock;
	static int wake_lock_inited;

	if (!wake_lock_inited) {
		DBG(0, "%s wake_lock_init\n", __func__);
		wake_lock_init(&host_test_wakelock, WAKE_LOCK_SUSPEND, "host.test.lock");
		wake_lock_inited = 1;
	}

	host_plug_test_triggered = 1;
	/* sync global status */
	mb();
	wake_lock(&host_test_wakelock);
	DBG(0, "BEGIN");
	ktime_begin = ktime_get();

	host_on  = 1;
	while (1) {
		if (!musb_is_host() && host_on) {
			DBG(0, "about to exit");
			break;
		}
		msleep(50);

		ktime_end = ktime_get();
		diff_time = ktime_to_ms(ktime_sub(ktime_end, ktime_begin));
		if (host_on && diff_time >= host_plug_in_test_period_ms) {
			host_on = 0;
			DBG(0, "OFF\n");

			ktime_begin = ktime_get();

			/* simulate plug out */
			musb_platform_set_vbus(mtk_musb, 0);
			udelay(host_test_vbus_off_time_us);

			if (!host_test_vbus_only)
				schedule_delayed_work(&mtk_musb->id_pin_work, 0);
		} else if (!host_on && diff_time >= host_plug_out_test_period_ms) {
			host_on = 1;
			DBG(0, "ON\n");

			ktime_begin = ktime_get();
			if (!host_test_vbus_only)
				schedule_delayed_work(&mtk_musb->id_pin_work, 0);

			musb_platform_set_vbus(mtk_musb, 1);
			msleep(100);

		}
	}

	/* wait host_work done */
	msleep(1000);
	host_plug_test_triggered = 0;
	wake_unlock(&host_test_wakelock);
	DBG(0, "END\n");
}

#define ID_PIN_WORK_RECHECK_TIME 30	/* 30 ms */
#define ID_PIN_WORK_BLOCK_TIMEOUT 30000 /* 30000 ms */
static void musb_id_pin_work(struct work_struct *data)
{
	u8 devctl = 0;
	unsigned long flags;
	static int inited, timeout; /* default to 0 */
	static DEFINE_RATELIMIT_STATE(ratelimit, 1 * HZ, 3);
	static s64 diff_time;

	/* kernel_init_done should be set in early-init stage through init.$platform.usb.rc */
	if (!inited && !kernel_init_done && !mtk_musb->is_ready && !timeout) {
		ktime_end = ktime_get();
		diff_time = ktime_to_ms(ktime_sub(ktime_end, ktime_start));
		if (__ratelimit(&ratelimit)) {
			DBG(0, "init_done:%d, is_ready:%d, inited:%d, TO:%d, diff:%lld\n",
					kernel_init_done, mtk_musb->is_ready, inited, timeout,
					diff_time);
		}

		if (diff_time > ID_PIN_WORK_BLOCK_TIMEOUT) {
			DBG(0, "diff_time:%lld\n", diff_time);
			timeout = 1;
		}

		queue_delayed_work(mtk_musb->st_wq, &mtk_musb->id_pin_work, msecs_to_jiffies(ID_PIN_WORK_RECHECK_TIME));
		return;
	} else if (!inited) {
		DBG(0, "PASS, init_done:%d, is_ready:%d, inited:%d, TO:%d\n",
				kernel_init_done,  mtk_musb->is_ready, inited, timeout);
	}

	inited = 1;

	spin_lock_irqsave(&mtk_musb->lock, flags);
	musb_generic_disable(mtk_musb);
	spin_unlock_irqrestore(&mtk_musb->lock, flags);

	down(&mtk_musb->musb_lock);
	DBG(0, "work start, is_host=%d\n", mtk_musb->is_host);

	if (mtk_musb->in_ipo_off) {
		DBG(0, "do nothing due to in_ipo_off\n");
		goto out;
	}

	/* flip */
	if (host_plug_test_triggered)
		mtk_musb->is_host = !mtk_musb->is_host;
	else
		mtk_musb->is_host = musb_is_host();

	DBG(0, "musb is as %s\n", mtk_musb->is_host?"host":"device");
	switch_set_state((struct switch_dev *)&otg_state, mtk_musb->is_host);

	if (mtk_musb->is_host) {
		/* setup fifo for host mode */
		ep_config_from_table_for_host(mtk_musb);
		wake_lock(&mtk_musb->usb_lock);
		musb_platform_set_vbus(mtk_musb, 1);

	/* for no VBUS sensing IP*/
	#if 1
		/* wait VBUS ready */
		msleep(100);
		/* clear session*/
		devctl = musb_readb(mtk_musb->mregs, MUSB_DEVCTL);
		musb_writeb(mtk_musb->mregs, MUSB_DEVCTL, (devctl&(~MUSB_DEVCTL_SESSION)));
		/* USB MAC OFF*/
		/* VBUSVALID=0, AVALID=0, BVALID=0, SESSEND=1, IDDIG=X, IDPULLUP=1 */
		USBPHY_SET8(0x6c, 0x11);
		USBPHY_CLR8(0x6c, 0x2e);
		USBPHY_SET8(0x6d, 0x3f);
		DBG(0, "force PHY to idle, 0x6d=%x, 0x6c=%x\n", USBPHY_READ8(0x6d), USBPHY_READ8(0x6c));
		/* wait */
		mdelay(5);
		/* restart session */
		devctl = musb_readb(mtk_musb->mregs, MUSB_DEVCTL);
		musb_writeb(mtk_musb->mregs, MUSB_DEVCTL, (devctl | MUSB_DEVCTL_SESSION));
		/* USB MAC ONand Host Mode*/
		/* VBUSVALID=1, AVALID=1, BVALID=1, SESSEND=0, IDDIG=0, IDPULLUP=1 */
		USBPHY_CLR8(0x6c, 0x10);
		USBPHY_SET8(0x6c, 0x2d);
		USBPHY_SET8(0x6d, 0x3f);
		DBG(0, "force PHY to host mode, 0x6d=%x, 0x6c=%x\n", USBPHY_READ8(0x6d), USBPHY_READ8(0x6c));

		USBPHY_SET8(0x18, 0xF0);
		DBG(0, "Set Host Disconnect Threshold to 700mV.\n");
	#endif

		musb_start(mtk_musb);
		MUSB_HST_MODE(mtk_musb);
		switch_int_to_device(mtk_musb);

		if (host_plug_test_enable && !host_plug_test_triggered)
			queue_delayed_work(mtk_musb->st_wq, &host_plug_test_work, 0);
	} else {
		/* for device no disconnect interrupt */
		spin_lock_irqsave(&mtk_musb->lock, flags);
		if (mtk_musb->is_active) {
			DBG(0, "for not receiving disconnect interrupt\n");
			usb_hcd_resume_root_hub(musb_to_hcd(mtk_musb));
			musb_root_disconnect(mtk_musb);
		}
		spin_unlock_irqrestore(&mtk_musb->lock, flags);

		DBG(0, "devctl is %x\n", musb_readb(mtk_musb->mregs, MUSB_DEVCTL));
		musb_writeb(mtk_musb->mregs, MUSB_DEVCTL, 0);
		if (wake_lock_active(&mtk_musb->usb_lock))
			wake_unlock(&mtk_musb->usb_lock);
		musb_platform_set_vbus(mtk_musb, 0);

	/* for no VBUS sensing IP */
	#if 1
	/* USB MAC OFF*/
		/* VBUSVALID=0, AVALID=0, BVALID=0, SESSEND=1, IDDIG=X, IDPULLUP=1 */
		USBPHY_SET8(0x6c, 0x11);
		USBPHY_CLR8(0x6c, 0x2e);
		USBPHY_SET8(0x6d, 0x3f);
		DBG(0, "force PHY to idle, 0x6d=%x, 0x6c=%x\n", USBPHY_READ8(0x6d), USBPHY_READ8(0x6c));

		USBPHY_SET8(0x18, 0x80);
		DBG(0, "Set Host Disconnect Threshold to 560mV.\n");
	#endif

		musb_stop(mtk_musb);
		mtk_musb->xceiv->state = OTG_STATE_B_IDLE;
		MUSB_DEV_MODE(mtk_musb);
		switch_int_to_host(mtk_musb);
	}
out:
	DBG(0, "work end, is_host=%d\n", mtk_musb->is_host);
	up(&mtk_musb->musb_lock);

}
static irqreturn_t mt_usb_ext_iddig_int(int irq, void *dev_id)
{
	iddig_cnt++;

	queue_delayed_work(mtk_musb->st_wq, &mtk_musb->id_pin_work, msecs_to_jiffies(sw_deboun_time));
	DBG(0, "id pin interrupt assert\n");
	disable_irq_nosync(usb_iddig_number);
	return IRQ_HANDLED;
}

void mt_usb_iddig_int(struct musb *musb)
{
	u32 usb_l1_ploy = musb_readl(musb->mregs, USB_L1INTP);

	DBG(0, "id pin interrupt assert,polarity=0x%x\n", usb_l1_ploy);
	if (usb_l1_ploy & IDDIG_INT_STATUS)
		usb_l1_ploy &= (~IDDIG_INT_STATUS);
	else
		usb_l1_ploy |= IDDIG_INT_STATUS;

	musb_writel(musb->mregs, USB_L1INTP, usb_l1_ploy);
	musb_writel(musb->mregs, USB_L1INTM, (~IDDIG_INT_STATUS)&musb_readl(musb->mregs, USB_L1INTM));

	queue_delayed_work(mtk_musb->st_wq, &mtk_musb->id_pin_work, msecs_to_jiffies(sw_deboun_time));
	DBG(0, "id pin interrupt assert\n");
}

static void otg_int_init(void)
{
#ifdef ID_PIN_USE_EX_EINT
	int	ret = 0;
#ifndef CONFIG_MTK_FPGA
	#ifdef CONFIG_OF
	#if defined(CONFIG_MTK_LEGACY)
	mt_set_gpio_mode(iddig_pin, iddig_pin_mode);
	mt_set_gpio_dir(iddig_pin, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(iddig_pin, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(iddig_pin, GPIO_PULL_UP);
	#else
	pr_debug("****%s:%d before Init IDDIG KS!!!!!\n", __func__, __LINE__);

#if !defined(CONFIG_USB_MTK_OTG_SWITCH)
	pinctrl_iddig = pinctrl_lookup_state(pinctrl, "iddig_irq_init");
	if (IS_ERR(pinctrl_iddig)) {
		ret = PTR_ERR(pinctrl_iddig);
		dev_err(mtk_musb->controller, "Cannot find usb pinctrl iddig_irq_init\n");
	}

	pinctrl_select_state(pinctrl, pinctrl_iddig);
#else
	pinctrl_iddig_init = pinctrl_lookup_state(pinctrl, "iddig_init");
	if (IS_ERR(pinctrl_iddig_init))
		pr_err("Cannot find usb pinctrl iddig_init\n");
	else
		pinctrl_select_state(pinctrl, pinctrl_iddig_init);

	pinctrl_iddig_enable = pinctrl_lookup_state(pinctrl, "iddig_enable");
	pinctrl_iddig_disable = pinctrl_lookup_state(pinctrl, "iddig_disable");
	if (IS_ERR(pinctrl_iddig_enable))
		pr_err("Cannot find usb pinctrl iddig_enable\n");

	if (IS_ERR(pinctrl_iddig_disable))
		pr_err("Cannot find usb pinctrl iddig_disable\n");
	else
		pinctrl_select_state(pinctrl, pinctrl_iddig_disable);
#ifdef CONFIG_SYSFS
	ret = sysfs_create_group(&mtk_musb->controller->kobj, &otg_attr_group);
	if (ret < 0) {
		pr_err("Cannot register USB bus sysfs attributes: %d\n",
			ret);
	}
#endif
	mutex_init(&otg_switch_mutex);
#endif

	pr_debug("****%s:%d end Init IDDIG KS!!!!!\n", __func__, __LINE__);
	#endif
	#else
	mt_set_gpio_mode(GPIO_OTG_IDDIG_EINT_PIN, GPIO_OTG_IDDIG_EINT_PIN_M_IDDIG);
	mt_set_gpio_dir(GPIO_OTG_IDDIG_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_OTG_IDDIG_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_OTG_IDDIG_EINT_PIN, GPIO_PULL_UP);
	#endif
#endif
	#if defined(CONFIG_MTK_LEGACY)
	mt_gpio_set_debounce(IDDIG_EINT_PIN, 64000);
	usb_iddig_number = mt_gpio_to_irq(IDDIG_EINT_PIN);
	pr_debug("USB IDDIG IRQ LINE %d, %d!!\n", IDDIG_EINT_PIN, mt_gpio_to_irq(IDDIG_EINT_PIN));
	ret = request_irq(usb_iddig_number, mt_usb_ext_iddig_int, IRQF_TRIGGER_LOW, "USB_IDDIG", NULL);
	#else
	/*gpio_request(iddig_pin, "USB_IDDIG");*/
	gpio_set_debounce(iddig_pin, 64000);
	usb_iddig_number = mt_gpio_to_irq(iddig_pin);
	#if !defined(CONFIG_USB_MTK_OTG_SWITCH)
	ret = request_irq(usb_iddig_number, mt_usb_ext_iddig_int, IRQF_TRIGGER_LOW, "USB_IDDIG", NULL);
	#endif
	#endif
	if (ret > 0)
		pr_err("USB IDDIG IRQ LINE not available!!\n");
	else
		pr_debug("USB IDDIG IRQ LINE available!!\n");
#else
	u32 phy_id_pull = 0;

	phy_id_pull = __raw_readl(U2PHYDTM1);
	phy_id_pull |= ID_PULL_UP;
	__raw_writel(phy_id_pull, U2PHYDTM1);

	musb_writel(mtk_musb->mregs, USB_L1INTM, IDDIG_INT_STATUS|musb_readl(mtk_musb->mregs, USB_L1INTM));
#endif
}

void mt_usb_otg_init(struct musb *musb)
{
	/* BYPASS OTG function in special mode */
	if (get_boot_mode() == META_BOOT
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
			|| get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT
			|| get_boot_mode() == LOW_POWER_OFF_CHARGING_BOOT
#endif
	   ) {
		DBG(0, "in special mode %d\n", get_boot_mode());
		return;
	}

	/* test */
	INIT_DELAYED_WORK(&host_plug_test_work, do_host_plug_test_work);

#ifdef CONFIG_OF
	usb_node = of_find_compatible_node(NULL, NULL, "mediatek,mt6735-usb20");
	if (usb_node == NULL) {
		pr_err("USB OTG - get USB0 node failed\n");
	} else {
		if (of_property_read_u32_index(usb_node, "iddig_gpio", 0, &iddig_pin)) {
			iddig_if_config = 0;
			pr_err("get dtsi iddig_pin fail\n");
		}
		if (of_property_read_u32_index(usb_node, "iddig_gpio", 1, &iddig_pin_mode))
			pr_err("get dtsi iddig_pin_mode fail\n");
		if (of_property_read_u32_index(usb_node, "drvvbus_gpio", 0, &drvvbus_pin)) {
			drvvbus_if_config = 0;
			pr_err("get dtsi drvvbus_pin fail\n");
		}
		if (of_property_read_u32_index(usb_node, "drvvbus_gpio", 1, &drvvbus_pin_mode))
			pr_err("get dtsi drvvbus_pin_mode fail\n");
		#if defined(CONFIG_MTK_LEGACY)
		iddig_pin |= 0x80000000;
		drvvbus_pin |= 0x80000000;
		#endif
	}

	#if !defined(CONFIG_MTK_LEGACY)
	pinctrl = devm_pinctrl_get(mtk_musb->controller);
	if (IS_ERR(pinctrl)) {
		dev_err(mtk_musb->controller, "Cannot find usb pinctrl!\n");
	}
	#endif
#endif
	/*init drrvbus*/
	mt_usb_init_drvvbus();

	/* init idpin interrupt */
	ktime_start = ktime_get();
	INIT_DELAYED_WORK(&musb->id_pin_work, musb_id_pin_work);
	otg_int_init();

	/* EP table */
	musb->fifo_cfg_host = fifo_cfg_host;
	musb->fifo_cfg_host_size = ARRAY_SIZE(fifo_cfg_host);

	otg_state.name = "otg_state";
	otg_state.index = 0;
	otg_state.state = 0;

	if (switch_dev_register(&otg_state))
		pr_err("switch_dev_register fail\n");
	else
		pr_debug("switch_dev register success\n");
}
#else
#include "musb_core.h"
/* for not define CONFIG_USB_MTK_OTG */
void mt_usb_otg_init(struct musb *musb) {}
void mt_usb_init_drvvbus(void){}
void mt_usb_set_vbus(struct musb *musb, int is_on) {}
int mt_usb_get_vbus_status(struct musb *musb) {return 1; }
void mt_usb_iddig_int(struct musb *musb) {}
void switch_int_to_device(struct musb *musb) {}
void switch_int_to_host(struct musb *musb) {}
void switch_int_to_host_and_mask(struct musb *musb) {}
void musb_session_restart(struct musb *musb) {}
#endif
