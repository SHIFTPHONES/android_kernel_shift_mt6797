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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/usb/usb_phy_generic.h>
#include <linux/switch.h>
#include <linux/i2c.h>
#include "musb_core.h"
#include "mtk_musb.h"
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include "musbhsdma.h"
#include <mt-plat/upmu_common.h>
#include <mt-plat/mt_boot_common.h>
#ifndef CONFIG_MTK_CLKMGR
#include <linux/clk.h>
struct clk *musb_clk;
#endif
#include "usb20.h"
#include <linux/delay.h>
#ifdef CONFIG_OF
#include <linux/of_irq.h>
#include <linux/of_address.h>
#ifndef CONFIG_MTK_LEGACY
#include <linux/regulator/consumer.h>
#endif
#endif

#ifdef MUSB_QMU_SUPPORT
#include "musb_qmu.h"
#endif

#ifdef CONFIG_MTK_USB2JTAG_SUPPORT
#include <mt-plat/mt_usb2jtag.h>
#endif


/* default value 0 */
static int usb_rdy;
void set_usb_rdy(void)
{
	DBG(0, "set usb_rdy, wake up bat\n");
	usb_rdy = 1;
#ifdef CONFIG_MTK_SMART_BATTERY
	wake_up_bat();
#endif
}
kal_bool is_usb_rdy(void)
{
	if (usb_rdy)
		return KAL_TRUE;
	else
		return KAL_FALSE;
}

/* static bool platform_init_first = true; */
static u32 cable_mode = CABLE_MODE_NORMAL;
#ifndef CONFIG_MTK_LEGACY
static struct regulator *reg;
#endif
/* add for linux kernel 3.10 */


#ifdef CONFIG_OF
u32 usb_irq_number1 = 0;
unsigned long usb_phy_base;
#endif

#ifdef CONFIG_MTK_UART_USB_SWITCH
u32 port_mode = PORT_MODE_USB;
u32 sw_tx = 0;
u32 sw_rx = 0;
u32 sw_uart_path = 0;
#define AP_UART0_COMPATIBLE_NAME "mediatek,mt6735-uart"
void __iomem *ap_uart0_base;
#endif

/*EP Fifo Config*/
static struct musb_fifo_cfg fifo_cfg[] __initdata = {
	{.hw_ep_num = 1, .style = MUSB_FIFO_TX, .maxpacket = 512, .ep_mode = EP_BULK, .mode =
	 MUSB_BUF_DOUBLE},
	{.hw_ep_num = 1, .style = MUSB_FIFO_RX, .maxpacket = 512, .ep_mode = EP_BULK, .mode =
	 MUSB_BUF_DOUBLE},
	{.hw_ep_num = 2, .style = MUSB_FIFO_TX, .maxpacket = 512, .ep_mode = EP_BULK, .mode =
	 MUSB_BUF_DOUBLE},
	{.hw_ep_num = 2, .style = MUSB_FIFO_RX, .maxpacket = 512, .ep_mode = EP_BULK, .mode =
	 MUSB_BUF_DOUBLE},
	{.hw_ep_num = 3, .style = MUSB_FIFO_TX, .maxpacket = 512, .ep_mode = EP_BULK, .mode =
	 MUSB_BUF_DOUBLE},
	{.hw_ep_num = 3, .style = MUSB_FIFO_RX, .maxpacket = 512, .ep_mode = EP_BULK, .mode =
	 MUSB_BUF_DOUBLE},
	{.hw_ep_num = 4, .style = MUSB_FIFO_TX, .maxpacket = 512, .ep_mode = EP_BULK, .mode =
	 MUSB_BUF_DOUBLE},
	{.hw_ep_num = 4, .style = MUSB_FIFO_RX, .maxpacket = 512, .ep_mode = EP_BULK, .mode =
	 MUSB_BUF_DOUBLE},
	{.hw_ep_num = 5, .style = MUSB_FIFO_TX, .maxpacket = 512, .ep_mode = EP_INT, .mode =
	 MUSB_BUF_SINGLE},
	{.hw_ep_num = 5, .style = MUSB_FIFO_RX, .maxpacket = 512, .ep_mode = EP_INT, .mode =
	 MUSB_BUF_SINGLE},
	{.hw_ep_num = 6, .style = MUSB_FIFO_TX, .maxpacket = 512, .ep_mode = EP_INT, .mode =
	 MUSB_BUF_SINGLE},
	{.hw_ep_num = 6, .style = MUSB_FIFO_RX, .maxpacket = 512, .ep_mode = EP_INT, .mode =
	 MUSB_BUF_SINGLE},
	{.hw_ep_num = 7, .style = MUSB_FIFO_TX, .maxpacket = 512, .ep_mode = EP_BULK, .mode =
	 MUSB_BUF_SINGLE},
	{.hw_ep_num = 7, .style = MUSB_FIFO_RX, .maxpacket = 512, .ep_mode = EP_BULK, .mode =
	 MUSB_BUF_SINGLE},
	{.hw_ep_num = 8, .style = MUSB_FIFO_TX, .maxpacket = 512, .ep_mode = EP_ISO, .mode =
	 MUSB_BUF_DOUBLE},
	{.hw_ep_num = 8, .style = MUSB_FIFO_RX, .maxpacket = 512, .ep_mode = EP_ISO, .mode =
	 MUSB_BUF_DOUBLE},
};


/*=======================================================================*/
/* MT6589 USB GADGET                                                     */
/*=======================================================================*/
#ifdef CONFIG_OF
static const struct of_device_id apusb_of_ids[] = {
	{.compatible = "mediatek,mt6735-usb20",},
	{},
};

static struct platform_device mt_usb_device = {
	.name = "mt_usb",
	.id = -1,
};

#endif
MODULE_DEVICE_TABLE(of, apusb_of_ids);

#ifndef FPGA_PLATFORM
#ifdef CONFIG_ARCH_MT6735
#include "mt_vcore_dvfs.h"

static struct workqueue_struct *vcore_wq;
static struct work_struct vcore_work;
static DEFINE_SPINLOCK(vcore_req_lock);
static int vcore_state;
static int vcore_req_num;
static int vcore_enable_cnt;
static int vcore_disable_cnt;

struct list_head vcore_req_free_pool;
struct list_head vcore_req_execute_pool;
struct vcore_req_wrapper {
	struct list_head list;
	int vcore_req;
};
static inline void put_req(struct vcore_req_wrapper *req, struct list_head *head)
{
	list_add_tail(&req->list, head);
}

static inline struct vcore_req_wrapper *get_req(struct list_head *head)
{
	struct vcore_req_wrapper *req = NULL;

	if (!list_empty(head)) {
		req = list_first_entry(head, struct vcore_req_wrapper, list);
		list_del(&req->list);
	}
	return req;
}
void vcore_hold(void)
{
	int vcore_ret;

	vcore_ret = vcorefs_request_dvfs_opp(KIR_USB, OPP_0);
	if (vcore_ret)
		DBG(0, "hold VCORE fail (%d)\n", vcore_ret);
	else
		DBG(0, "hold VCORE ok\n");

}
void vcore_release(void)
{
	int vcore_ret;

	vcore_ret = vcorefs_request_dvfs_opp(KIR_USB, OPP_OFF);
	if (vcore_ret)
		DBG(0, "release VCORE fail(%d)\n", vcore_ret);
	else
		DBG(0, "release VCORE ok\n");
}

static void do_vcore_work(struct work_struct *work)
{
	DBG(0, "%s begin\n", __func__);
	while (1) {
		unsigned long	flags;
		struct vcore_req_wrapper *req;
		int vcore_req;

		spin_lock_irqsave(&vcore_req_lock, flags);
		req = get_req(&vcore_req_execute_pool);
		if (!req) {
			spin_unlock_irqrestore(&vcore_req_lock, flags);
			break;
		}
		vcore_req = req->vcore_req;
		put_req(req, &vcore_req_free_pool);
		spin_unlock_irqrestore(&vcore_req_lock, flags);

		DBG(0, "vcore_state:%d, vcore_req:%d\n", vcore_state, vcore_req);
		if (!vcore_state && vcore_req) {
			vcore_hold();
			vcore_state = 1;
			vcore_enable_cnt++;
		} else if (vcore_state && !vcore_req) {
			vcore_release();
			vcore_state = 0;
			vcore_disable_cnt++;
		}

	}
	DBG(0, "end, state:%d, num:%d, cnt<%d, %d>\n",
			vcore_state, vcore_req_num, vcore_enable_cnt, vcore_disable_cnt);
}

void request_vcore(int on)
{
	unsigned long flags;
	struct vcore_req_wrapper *req;

	spin_lock_irqsave(&vcore_req_lock, flags);
	req = get_req(&vcore_req_free_pool);
	if (!req) {
		req = kmalloc(sizeof(struct vcore_req_wrapper), GFP_ATOMIC);
		if (!req) {
			DBG(0, "vcore_req_wrapper alloc fail, directly return !!!!\n");
			return;
		}
		vcore_req_num++;
		DBG(0, "vcore_req_wrapper alloc done<%d>\n", vcore_req_num);
	}
	req->vcore_req = on;
	put_req(req, &vcore_req_execute_pool);
	queue_work(vcore_wq, &vcore_work);
	spin_unlock_irqrestore(&vcore_req_lock, flags);
	DBG(0, "req:%d, end\n", on);

}
#endif
#endif

static struct timer_list musb_idle_timer;

static void musb_do_idle(unsigned long _musb)
{
	struct musb *musb = (void *)_musb;
	unsigned long flags;
	u8 devctl;

	spin_lock_irqsave(&musb->lock, flags);
	if (musb->is_active) {
		DBG(0, "%s active, igonre do_idle\n", otg_state_string(musb->xceiv->state));
		spin_unlock_irqrestore(&musb->lock, flags);
		return;
	}

	switch (musb->xceiv->state) {
	case OTG_STATE_B_PERIPHERAL:
	case OTG_STATE_A_WAIT_BCON:
		devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
		if (devctl & MUSB_DEVCTL_BDEVICE) {
			musb->xceiv->state = OTG_STATE_B_IDLE;
			MUSB_DEV_MODE(musb);
		} else {
			musb->xceiv->state = OTG_STATE_A_IDLE;
			MUSB_HST_MODE(musb);
		}
		break;
	case OTG_STATE_A_HOST:
		devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
		if (devctl & MUSB_DEVCTL_BDEVICE)
			musb->xceiv->state = OTG_STATE_B_IDLE;
		else
			musb->xceiv->state = OTG_STATE_A_WAIT_BCON;
		break;
	default:
		break;
	}
	spin_unlock_irqrestore(&musb->lock, flags);

	DBG(0, "otg_state %s\n", otg_state_string(musb->xceiv->state));
}

static void mt_usb_try_idle(struct musb *musb, unsigned long timeout)
{
	unsigned long default_timeout = jiffies + msecs_to_jiffies(3);
	static unsigned long last_timer;

	if (timeout == 0)
		timeout = default_timeout;

	/* Never idle if active, or when VBUS timeout is not set as host */
	if (musb->is_active || ((musb->a_wait_bcon == 0)
				&& (musb->xceiv->state == OTG_STATE_A_WAIT_BCON))) {
		DBG(2, "%s active, deleting timer\n", otg_state_string(musb->xceiv->state));
		del_timer(&musb_idle_timer);
		last_timer = jiffies;
		return;
	}

	if (time_after(last_timer, timeout)) {
		if (!timer_pending(&musb_idle_timer))
			last_timer = timeout;
		else {
			DBG(2, "Longer idle timer already pending, ignoring\n");
			return;
		}
	}
	last_timer = timeout;

	DBG(2, "%s inactive, for idle timer for %lu ms\n",
	    otg_state_string(musb->xceiv->state),
	    (unsigned long)jiffies_to_msecs(timeout - jiffies));
	mod_timer(&musb_idle_timer, timeout);
}

static int real_enable = 0, real_disable;
static int virt_enable = 0, virt_disable;
static void mt_usb_enable(struct musb *musb)
{
	unsigned long flags;

	virt_enable++;
	DBG(0, "begin <%d,%d>,<%d,%d,%d,%d>\n", mtk_usb_power, musb->power, virt_enable, virt_disable,
			real_enable, real_disable);
	if (musb->power == true)
		return;
#ifndef FPGA_PLATFORM
#ifdef CONFIG_ARCH_MT6735
	request_vcore(1);
#endif
#endif

	flags = musb_readl(musb->mregs, USB_L1INTM);

	mdelay(10);

	usb_phy_recover();

	/* update musb->power & mtk_usb_power in the same time */
	musb->power = true;
	mtk_usb_power = true;
	real_enable++;
	if (in_interrupt()) {
		DBG(0, "in interrupt !!!!!!!!!!!!!!!\n");
		DBG(0, "in interrupt !!!!!!!!!!!!!!!\n");
		DBG(0, "in interrupt !!!!!!!!!!!!!!!\n");
	}
	DBG(0, "end, <%d,%d,%d,%d>\n", virt_enable, virt_disable, real_enable, real_disable);
	musb_writel(mtk_musb->mregs, USB_L1INTM, flags);
}

static void mt_usb_disable(struct musb *musb)
{
	virt_disable++;

	DBG(0, "begin, <%d,%d>,<%d,%d,%d,%d>\n", mtk_usb_power, musb->power, virt_enable, virt_disable,
	    real_enable, real_disable);
	if (musb->power == false)
		return;

	usb_phy_savecurrent();
	real_disable++;
	DBG(0, "end, <%d,%d,%d,%d>\n", virt_enable, virt_disable, real_enable, real_disable);

	/* update musb->power & mtk_usb_power in the same time */
	musb->power = 0;
	mtk_usb_power = false;
#ifndef FPGA_PLATFORM
#ifdef CONFIG_ARCH_MT6735
	request_vcore(0);
#endif
#endif
}

/* ================================ */
/* connect and disconnect functions */
/* ================================ */
bool mt_usb_is_device(void)
{
	DBG(4, "called\n");

	if (!mtk_musb) {
		DBG(0, "mtk_musb is NULL\n");
		return false;	/* don't do charger detection when usb is not ready */
	}
	DBG(4, "is_host=%d\n", mtk_musb->is_host);

#ifdef CONFIG_MTK_UART_USB_SWITCH
	if (in_uart_mode) {
		DBG(0, "in UART Mode\n");
		return false;
	}
#endif

	return !mtk_musb->is_host;
}

static struct delayed_work disconnect_check_work;
static bool musb_hal_is_vbus_exist(void);
void do_disconnect_check_work(struct work_struct *data)
{
	bool vbus_exist = false;
	unsigned long flags = 0;
	struct musb *musb = mtk_musb;

	msleep(200);

	vbus_exist = musb_hal_is_vbus_exist();
	DBG(1, "vbus_exist:<%d>\n", vbus_exist);
	if (vbus_exist)
		return;

	spin_lock_irqsave(&mtk_musb->lock, flags);
	DBG(1, "speed <%d>\n", musb->g.speed);
	/* notify gadget driver, g.speed judge is very important */
	if (!musb->is_host && musb->g.speed != USB_SPEED_UNKNOWN) {
		DBG(0, "musb->gadget_driver:%p\n", musb->gadget_driver);
		if (musb->gadget_driver && musb->gadget_driver->disconnect) {
			DBG(0, "musb->gadget_driver->disconnect:%p\n", musb->gadget_driver->disconnect);
			/* align musb_g_disconnect */
			spin_unlock(&musb->lock);
			musb->gadget_driver->disconnect(&musb->g);
			spin_lock(&musb->lock);

		}
		musb->g.speed = USB_SPEED_UNKNOWN;
	}
	DBG(1, "speed <%d>\n", musb->g.speed);
	spin_unlock_irqrestore(&mtk_musb->lock, flags);
}
void trigger_disconnect_check_work(void)
{
	static int inited;

	if (!inited) {
		INIT_DELAYED_WORK(&disconnect_check_work, do_disconnect_check_work);
		inited = 1;
	}
	queue_delayed_work(mtk_musb->st_wq, &disconnect_check_work, 0);
}

#define CONN_WORK_DELAY 50
static struct delayed_work connection_work;
void do_connection_work(struct work_struct *data)
{
	static DEFINE_RATELIMIT_STATE(ratelimit, 1 * HZ, 3);
	unsigned long flags = 0;
	bool usb_in = false;


	if (mtk_musb) {
		if (__ratelimit(&ratelimit))
			DBG(0, "is ready %d is_host %d power %d\n", mtk_musb->is_ready, mtk_musb->is_host,
					mtk_musb->power);
	} else {
		/* re issue delay work */
		if (__ratelimit(&ratelimit))
			DBG(0, "issue work after %d ms\n", CONN_WORK_DELAY);
		queue_delayed_work(mtk_musb->st_wq, &connection_work, msecs_to_jiffies(CONN_WORK_DELAY));
		return;
	}



	if (!mtk_musb->is_ready) {
		/* re issue work */
		if (__ratelimit(&ratelimit))
			DBG(0, "issue work after %d ms\n", CONN_WORK_DELAY);
		queue_delayed_work(mtk_musb->st_wq, &connection_work, msecs_to_jiffies(CONN_WORK_DELAY));
		return;
	}

	/* be aware this could not be used in non-sleep context */
	usb_in = usb_cable_connected();

	spin_lock_irqsave(&mtk_musb->lock, flags);

	if (mtk_musb->is_host) {
		DBG(0, "is host, return\n");
		spin_unlock_irqrestore(&mtk_musb->lock, flags);
		return;
	}

#ifdef CONFIG_MTK_UART_USB_SWITCH
	if (usb_phy_check_in_uart_mode()) {
		DBG(0, "in uart mode, return\n");
		spin_unlock_irqrestore(&mtk_musb->lock, flags);
		return;
	}
#endif

	if (!mtk_musb->power && (usb_in == true)) {
		/* enable usb */
		if (!wake_lock_active(&mtk_musb->usb_lock)) {
			wake_lock(&mtk_musb->usb_lock);
			DBG(0, "lock\n");
		} else {
			DBG(0, "already lock\n");
		}

		/* note this already put SOFTCON */
		musb_start(mtk_musb);

	} else if (mtk_musb->power && (usb_in == false)) {
		/* disable usb */
		musb_stop(mtk_musb);
		if (wake_lock_active(&mtk_musb->usb_lock)) {
			DBG(0, "unlock\n");
			wake_unlock(&mtk_musb->usb_lock);
		} else {
			DBG(0, "lock not active\n");
		}

	} else
		DBG(0, "do nothing, usb_in:%d, power:%d\n",
				usb_in, mtk_musb->power);

	spin_unlock_irqrestore(&mtk_musb->lock, flags);

}

void mt_usb_connect(void)
{
	if (!mtk_musb) {
		DBG(0, "mtk_musb = NULL\n");
		return;
	}
	/* issue connection work */
	DBG(0, "issue work\n");
	queue_delayed_work(mtk_musb->st_wq, &connection_work, 0);
	DBG(0, "[MUSB] USB connect\n");

}

void mt_usb_disconnect(void)
{
	if (!mtk_musb) {
		DBG(0, "mtk_musb = NULL\n");
		return;
	}
	/* issue connection work */
	DBG(0, "issue work\n");
	queue_delayed_work(mtk_musb->st_wq, &connection_work, 0);
	DBG(0, "[MUSB] USB disconnect\n");
}

static void do_connect_rescue_work(struct work_struct *work)
{
	DBG(0, "do_connect_rescue_work, issue connection work\n");
	queue_delayed_work(mtk_musb->st_wq, &connection_work, 0);
}


/* #define BYPASS_PMIC_LINKAGE */
static CHARGER_TYPE musb_hal_get_charger_type(void)
{
	CHARGER_TYPE chg_type;
#ifdef BYPASS_PMIC_LINKAGE
	DBG(0, "force on");
	chg_type = STANDARD_HOST;
#else
	chg_type = mt_get_charger_type();
#endif

	return chg_type;
}
static bool musb_hal_is_vbus_exist(void)
{
	bool vbus_exist;

#ifdef BYPASS_PMIC_LINKAGE
	DBG(0, "force on");
	vbus_exist = true;
#else
#ifdef CONFIG_POWER_EXT
	vbus_exist = upmu_get_rgs_chrdet();
#else
	vbus_exist = upmu_is_chr_det();
#endif
#endif

	return vbus_exist;

}

static int usb20_test_connect;
static bool test_connected;
static struct delayed_work usb20_test_connect_work;
#define TEST_CONNECT_BASE_MS 3000
#define TEST_CONNECT_BIAS_MS 5000
static void do_usb20_test_connect_work(struct work_struct *work)
{
	static ktime_t ktime;
	static unsigned long int ktime_us;
	unsigned int delay_time_ms;

	if (!usb20_test_connect) {
		test_connected = false;
		DBG(0, "%s, test done, trigger connect\n", __func__);
		mt_usb_connect();
		return;
	}
	mt_usb_connect();

	ktime = ktime_get();
	ktime_us = ktime_to_us(ktime);
	delay_time_ms = TEST_CONNECT_BASE_MS + (ktime_us % TEST_CONNECT_BIAS_MS);
	DBG(0, "%s, work after %d ms\n", __func__, delay_time_ms);
	schedule_delayed_work(&usb20_test_connect_work, msecs_to_jiffies(delay_time_ms));

	test_connected = !test_connected;
}
void mt_usb_connect_test(int start)
{
	static struct wake_lock device_test_wakelock;
	static int wake_lock_inited;

	if (!wake_lock_inited) {
		DBG(0, "%s wake_lock_init\n", __func__);
		wake_lock_init(&device_test_wakelock, WAKE_LOCK_SUSPEND, "device.test.lock");
		wake_lock_inited = 1;
	}

	if (start) {
		wake_lock(&device_test_wakelock);
		usb20_test_connect = 1;
		INIT_DELAYED_WORK(&usb20_test_connect_work, do_usb20_test_connect_work);
		schedule_delayed_work(&usb20_test_connect_work, 0);
	} else {
		usb20_test_connect = 0;
		wake_unlock(&device_test_wakelock);
	}
}

DEFINE_MUTEX(cable_connected_lock);
/* be aware this could not be used in non-sleep context */
bool usb_cable_connected(void)
{
	CHARGER_TYPE chg_type = CHARGER_UNKNOWN;
	bool connected = false, vbus_exist = false;
	int delay_time;

	if (usb20_test_connect) {
		DBG(0, "%s, return test_connected<%d>\n", __func__, test_connected);
		return test_connected;
	}

	mutex_lock(&cable_connected_lock);
	/* FORCE USB ON case */
	if (musb_force_on) {
		delay_time = 0;    /* directly issue connection */
		chg_type = STANDARD_HOST;
		vbus_exist = true;
		connected = true;
		DBG(0, "%s type force to STANDARD_HOST\n", __func__);
	} else {
		/* TYPE CHECK*/
		delay_time = 2000; /* issue connection one time in case, BAT THREAD didn't come*/
		chg_type = musb_hal_get_charger_type();
		if (musb_fake_CDP && chg_type == STANDARD_HOST) {
			DBG(0, "%s, fake to type 2\n", __func__);
			chg_type = CHARGING_HOST;
		}

		if (chg_type == STANDARD_HOST || chg_type == CHARGING_HOST)
			connected = true;

		/* VBUS CHECK to avoid type miss-judge */
		vbus_exist = musb_hal_is_vbus_exist();

		DBG(0, "%s vbus_exist=%d type=%d\n", __func__, vbus_exist, chg_type);
		if (!vbus_exist)
			connected = false;
	}

	/* CMODE CHECK */
	if (cable_mode == CABLE_MODE_CHRG_ONLY || (cable_mode == CABLE_MODE_HOST_ONLY && chg_type != CHARGING_HOST))
		connected = false;

	/* one time job, set_usb_rdy, issue connect_rescue_work */
	if (is_usb_rdy() == KAL_FALSE && mtk_musb->is_ready) {
		static struct delayed_work connect_rescue_work;

		set_usb_rdy();

		INIT_DELAYED_WORK(&connect_rescue_work, do_connect_rescue_work);
		DBG(0, "issue connect_rescue_work on is_ready begin, delay_time:%d ms\n", delay_time);
		schedule_delayed_work(&connect_rescue_work, msecs_to_jiffies(delay_time));
		DBG(0, "issue connect_rescue_work on is_ready end, delay_time:%d ms\n", delay_time);
	}

	mutex_unlock(&cable_connected_lock);
	DBG(0, "%s, connected:%d, cable_mode:%d\n", __func__, connected, cable_mode);
	return connected;
}

void musb_platform_reset(struct musb *musb)
{
	u16 swrst = 0;
	void __iomem *mbase = musb->mregs;
	u8 bit;

	/* clear all DMA enable bit */
	for (bit = 0; bit < MUSB_HSDMA_CHANNELS; bit++)
		musb_writew(mbase, MUSB_HSDMA_CHANNEL_OFFSET(bit, MUSB_HSDMA_CONTROL), 0);

	/* set DMA channel 0 burst mode to boost QMU speed */
	musb_writel(musb->mregs, 0x204, musb_readl(musb->mregs, 0x204) | 0x600);

	swrst = musb_readw(mbase, MUSB_SWRST);
	swrst |= (MUSB_SWRST_DISUSBRESET | MUSB_SWRST_SWRST);
	musb_writew(mbase, MUSB_SWRST, swrst);
}

static void usb_reconnect(void)
{
	DBG(0, "issue work\n");
	queue_delayed_work(mtk_musb->st_wq, &connection_work, 0);
	DBG(0, "%s end\n", __func__);
}

bool is_switch_charger(void)
{
#ifdef SWITCH_CHARGER
	return true;
#else
	return false;
#endif
}

void pmic_chrdet_int_en(int is_on)
{
#ifndef FPGA_PLATFORM
	upmu_interrupt_chrdet_int_en(is_on);
#endif
}

void musb_sync_with_bat(struct musb *musb, int usb_state)
{
#ifndef FPGA_PLATFORM

	DBG(0, "BATTERY_SetUSBState, state=%d\n", usb_state);
#ifdef CONFIG_MTK_SMART_BATTERY
	BATTERY_SetUSBState(usb_state);
	wake_up_bat();
#endif
#endif
}

/*-------------------------------------------------------------------------*/
static irqreturn_t generic_interrupt(int irq, void *__hci)
{
	irqreturn_t retval = IRQ_NONE;
	struct musb *musb = __hci;

	/* musb_read_clear_generic_interrupt */
	musb->int_usb =
	    musb_readb(musb->mregs, MUSB_INTRUSB) & musb_readb(musb->mregs, MUSB_INTRUSBE);
	musb->int_tx = musb_readw(musb->mregs, MUSB_INTRTX) & musb_readw(musb->mregs, MUSB_INTRTXE);
	musb->int_rx = musb_readw(musb->mregs, MUSB_INTRRX) & musb_readw(musb->mregs, MUSB_INTRRXE);
#ifdef MUSB_QMU_SUPPORT
	musb->int_queue = musb_readl(musb->mregs, MUSB_QISAR);
#endif
	mb();
	musb_writew(musb->mregs, MUSB_INTRRX, musb->int_rx);
	musb_writew(musb->mregs, MUSB_INTRTX, musb->int_tx);
	musb_writeb(musb->mregs, MUSB_INTRUSB, musb->int_usb);
#ifdef MUSB_QMU_SUPPORT
	if (musb->int_queue) {
		musb_writel(musb->mregs, MUSB_QISAR, musb->int_queue);
		musb->int_queue &= ~(musb_readl(musb->mregs, MUSB_QIMR));
	}
#endif
	/* musb_read_clear_generic_interrupt */

#ifdef MUSB_QMU_SUPPORT
	if (musb->int_usb || musb->int_tx || musb->int_rx || musb->int_queue)
		retval = musb_interrupt(musb);
#else
	if (musb->int_usb || musb->int_tx || musb->int_rx)
		retval = musb_interrupt(musb);
#endif


	return retval;
}

static irqreturn_t mt_usb_interrupt(int irq, void *dev_id)
{
	irqreturn_t tmp_status;
	irqreturn_t status = IRQ_NONE;
	struct musb *musb = (struct musb *)dev_id;
	u32 usb_l1_ints;
	unsigned long flags;

	spin_lock_irqsave(&musb->lock, flags);
	usb_l1_ints = musb_readl(musb->mregs, USB_L1INTS) & musb_readl(mtk_musb->mregs, USB_L1INTM);
	DBG(1, "usb interrupt assert %x %x  %x %x %x\n", usb_l1_ints,
	    musb_readl(mtk_musb->mregs, USB_L1INTM), musb_readb(musb->mregs, MUSB_INTRUSBE),
	    musb_readw(musb->mregs, MUSB_INTRTX), musb_readw(musb->mregs, MUSB_INTRTXE));

	if ((usb_l1_ints & TX_INT_STATUS) || (usb_l1_ints & RX_INT_STATUS)
	    || (usb_l1_ints & USBCOM_INT_STATUS)
#ifdef MUSB_QMU_SUPPORT
	    || (usb_l1_ints & QINT_STATUS)
#endif
	   ) {
		tmp_status = generic_interrupt(irq, musb);
		if (tmp_status != IRQ_NONE)
			status = tmp_status;
	}
	spin_unlock_irqrestore(&musb->lock, flags);

	/* FIXME, workaround for device_qmu + host_dma */
#if 1
/* #ifndef MUSB_QMU_SUPPORT */
	if (usb_l1_ints & DMA_INT_STATUS) {
		tmp_status = dma_controller_irq(irq, musb->dma_controller);
		if (tmp_status != IRQ_NONE)
			status = tmp_status;
	}
#endif

#ifdef	CONFIG_USB_MTK_OTG
	if (usb_l1_ints & IDDIG_INT_STATUS) {
		mt_usb_iddig_int(musb);
		status = IRQ_HANDLED;
	}
#endif

	return status;

}

/*--FOR INSTANT POWER ON USAGE--------------------------------------------------*/
static ssize_t mt_usb_show_cmode(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (!dev) {
		DBG(0, "dev is null!!\n");
		return 0;
	}
	return scnprintf(buf, PAGE_SIZE, "%d\n", cable_mode);
}

static ssize_t mt_usb_store_cmode(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned int cmode;
	long tmp_val;

	if (!dev) {
		DBG(0, "dev is null!!\n");
		return count;
	/* } else if (1 == sscanf(buf, "%d", &cmode)) { */
	} else if (kstrtol(buf, 10, (long *)&tmp_val) == 0) {
		if (mtk_musb) {
			if (down_interruptible(&mtk_musb->musb_lock))
				DBG(0, "USB20: %s: busy, Couldn't get power_clock_lock\n",
						__func__);
		}
		cmode = tmp_val;
		DBG(0, "cmode=%d, cable_mode=%d\n", cmode, cable_mode);
		if (cmode >= CABLE_MODE_MAX)
			cmode = CABLE_MODE_NORMAL;

		if (cable_mode != cmode) {
			if (cmode == CABLE_MODE_CHRG_ONLY) {	/* IPO shutdown, disable USB */
				if (mtk_musb) {
					mtk_musb->in_ipo_off = true;
					DBG(0, "in_ipo_off is set\n");
				}

			} else {	/* IPO bootup, enable USB */
				if (mtk_musb) {
					mtk_musb->in_ipo_off = false;
					DBG(0, "in_ipo_off is clr\n");
				}
			}

			cable_mode = cmode;
			usb_reconnect();

			/* let conection work do its job */
			msleep(50);

#ifdef CONFIG_USB_MTK_OTG
			if (cmode == CABLE_MODE_CHRG_ONLY) {
				if (mtk_musb && mtk_musb->is_host) {	/* shut down USB host for IPO */
					if (wake_lock_active(&mtk_musb->usb_lock))
						wake_unlock(&mtk_musb->usb_lock);
					musb_platform_set_vbus(mtk_musb, 0);
					/* add sleep time to ensure vbus off and disconnect irq processed. */
					msleep(50);
					musb_stop(mtk_musb);
					MUSB_DEV_MODE(mtk_musb);
					/* Think about IPO shutdown with A-cable, then switch to B-cable and IPO bootup.
					   We need a point to clear session bit */
					musb_writeb(mtk_musb->mregs, MUSB_DEVCTL,
						    (~MUSB_DEVCTL_SESSION) &
						    musb_readb(mtk_musb->mregs, MUSB_DEVCTL));
				}
				/* mask ID pin interrupt even if A-cable is not plugged in */
				switch_int_to_host_and_mask(mtk_musb);
			} else {
				switch_int_to_host(mtk_musb);	/* resotre ID pin interrupt */
			}
#endif
		}
		if (mtk_musb)
			up(&mtk_musb->musb_lock);
	}
	return count;
}

DEVICE_ATTR(cmode, 0664, mt_usb_show_cmode, mt_usb_store_cmode);

static bool saving_mode;

static ssize_t mt_usb_show_saving_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (!dev) {
		DBG(0, "dev is null!!\n");
		return 0;
	}
	return scnprintf(buf, PAGE_SIZE, "%d\n", saving_mode);
}

static ssize_t mt_usb_store_saving_mode(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	int saving;
	long tmp_val;

	if (!dev) {
		DBG(0, "dev is null!!\n");
		return count;
	/* } else if (1 == sscanf(buf, "%d", &saving)) { */
	} else if (kstrtol(buf, 10, (long *)&tmp_val) == 0) {
		saving = tmp_val;
		DBG(0, "old=%d new=%d\n", saving, saving_mode);
		if (saving_mode == (!saving))
			saving_mode = !saving_mode;
	}
	return count;
}

bool is_saving_mode(void)
{
	DBG(0, "%d\n", saving_mode);
	return saving_mode;
}

DEVICE_ATTR(saving, 0664, mt_usb_show_saving_mode, mt_usb_store_saving_mode);

#ifdef CONFIG_MTK_UART_USB_SWITCH
static void uart_usb_switch_dump_register(void)
{
	usb_enable_clock(true);
#ifdef FPGA_PLATFORM
	DBG(0, "[MUSB]addr: 0x6B, value: %x\n", USB_PHY_Read_Register8(0x6B));
	DBG(0, "[MUSB]addr: 0x6E, value: %x\n", USB_PHY_Read_Register8(0x6E));
	DBG(0, "[MUSB]addr: 0x22, value: %x\n", USB_PHY_Read_Register8(0x22));
	DBG(0, "[MUSB]addr: 0x68, value: %x\n", USB_PHY_Read_Register8(0x68));
	DBG(0, "[MUSB]addr: 0x6A, value: %x\n", USB_PHY_Read_Register8(0x6A));
	DBG(0, "[MUSB]addr: 0x1A, value: %x\n", USB_PHY_Read_Register8(0x1A));
#else
	DBG(0, "[MUSB]addr: 0x6B, value: %x\n", USBPHY_READ8(0x6B));
	DBG(0, "[MUSB]addr: 0x6E, value: %x\n", USBPHY_READ8(0x6E));
	DBG(0, "[MUSB]addr: 0x22, value: %x\n", USBPHY_READ8(0x22));
	DBG(0, "[MUSB]addr: 0x68, value: %x\n", USBPHY_READ8(0x68));
	DBG(0, "[MUSB]addr: 0x6A, value: %x\n", USBPHY_READ8(0x6A));
	DBG(0, "[MUSB]addr: 0x1A, value: %x\n", USBPHY_READ8(0x1A));
#endif
	usb_enable_clock(false);
	DBG(0, "[MUSB]addr: 0x110020B0 (UART0), value: %x\n\n", DRV_Reg8(ap_uart0_base + 0xB0));
}

static ssize_t mt_usb_show_portmode(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (!dev) {
		DBG(0, "dev is null!!\n");
		return 0;
	}

	if (usb_phy_check_in_uart_mode())
		port_mode = PORT_MODE_UART;
	else
		port_mode = PORT_MODE_USB;

	if (port_mode == PORT_MODE_USB)
		DBG(0, "\nUSB Port mode -> USB\n");
	else if (port_mode == PORT_MODE_UART)
		DBG(0, "\nUSB Port mode -> UART\n");
	uart_usb_switch_dump_register();

	return scnprintf(buf, PAGE_SIZE, "%d\n", port_mode);
}

static ssize_t mt_usb_store_portmode(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t count)
{
	unsigned int portmode;

	if (!dev) {
		DBG(0, "dev is null!!\n");
		return count;
	/* } else if (1 == sscanf(buf, "%d", &portmode)) { */
	} else if (kstrtouint(buf, 10, &portmode) == 0) {
		DBG(0, "\nUSB Port mode: current => %d (port_mode), change to => %d (portmode)\n",
		    port_mode, portmode);
		if (portmode >= PORT_MODE_MAX)
			portmode = PORT_MODE_USB;

		if (port_mode != portmode) {
			if (portmode == PORT_MODE_USB) {	/* Changing to USB Mode */
				DBG(0, "USB Port mode -> USB\n");
				usb_phy_switch_to_usb();
			} else if (portmode == PORT_MODE_UART) {	/* Changing to UART Mode */
				DBG(0, "USB Port mode -> UART\n");
				usb_phy_switch_to_uart();
			}
			uart_usb_switch_dump_register();
			port_mode = portmode;
		}
	}
	return count;
}

DEVICE_ATTR(portmode, 0664, mt_usb_show_portmode, mt_usb_store_portmode);


static ssize_t mt_usb_show_tx(struct device *dev, struct device_attribute *attr, char *buf)
{
	UINT8 var;
	UINT8 var2;

	if (!dev) {
		DBG(0, "dev is null!!\n");
		return 0;
	}
#ifdef FPGA_PLATFORM
	var = USB_PHY_Read_Register8(0x6E);
#else
	var = USBPHY_READ8(0x6E);
#endif
	var2 = (var >> 3) & ~0xFE;
	DBG(0, "[MUSB]addr: 0x6E (TX), value: %x - %x\n", var, var2);

	sw_tx = var;

	return scnprintf(buf, PAGE_SIZE, "%x\n", var2);
}

static ssize_t mt_usb_store_tx(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	unsigned int val;
	UINT8 var;
	UINT8 var2;

	if (!dev) {
		DBG(0, "dev is null!!\n");
		return count;
	/* } else if (1 == sscanf(buf, "%d", &val)) { */
	} else if (kstrtol(buf, 10, (long *)&val) == 0) {
		DBG(0, "\n Write TX : %d\n", val);

#ifdef FPGA_PLATFORM
		var = USB_PHY_Read_Register8(0x6E);
#else
		var = USBPHY_READ8(0x6E);
#endif

		if (val == 0)
			var2 = var & ~(1 << 3);
		else
			var2 = var | (1 << 3);

#ifdef FPGA_PLATFORM
		USB_PHY_Write_Register8(var2, 0x6E);
		var = USB_PHY_Read_Register8(0x6E);
#else
		USBPHY_WRITE8(0x6E, var2);
		var = USBPHY_READ8(0x6E);
#endif

		var2 = (var >> 3) & ~0xFE;

		DBG(0, "[MUSB]addr: 0x6E TX [AFTER WRITE], value after: %x - %x\n", var, var2);
		sw_tx = var;
	}
	return count;
}

DEVICE_ATTR(tx, 0664, mt_usb_show_tx, mt_usb_store_tx);

static ssize_t mt_usb_show_rx(struct device *dev, struct device_attribute *attr, char *buf)
{
	UINT8 var;
	UINT8 var2;

	if (!dev) {
		DBG(0, "dev is null!!\n");
		return 0;
	}
#ifdef FPGA_PLATFORM
	var = USB_PHY_Read_Register8(0x77);
#else
	var = USBPHY_READ8(0x77);
#endif
	var2 = (var >> 7) & ~0xFE;
	DBG(0, "[MUSB]addr: 0x77 (RX), value: %x - %x\n", var, var2);
	sw_rx = var;

	return scnprintf(buf, PAGE_SIZE, "%x\n", var2);
}

DEVICE_ATTR(rx, 0444, mt_usb_show_rx, NULL);

static ssize_t mt_usb_show_uart_path(struct device *dev, struct device_attribute *attr, char *buf)
{
	UINT8 var;

	if (!dev) {
		DBG(0, "dev is null!!\n");
		return 0;
	}

	var = DRV_Reg8(ap_uart0_base + 0xB0);
	DBG(0, "[MUSB]addr: (UART0) 0xB0, value: %x\n\n", DRV_Reg8(ap_uart0_base + 0xB0));
	sw_uart_path = var;

	return scnprintf(buf, PAGE_SIZE, "%x\n", var);
}

DEVICE_ATTR(uartpath, 0444, mt_usb_show_uart_path, NULL);
#endif

#ifdef FPGA_PLATFORM
static struct i2c_client *usb_i2c_client;
static const struct i2c_device_id usb_i2c_id[] = { {"mtk-usb", 0}, {} };

static struct i2c_board_info usb_i2c_dev __initdata = { I2C_BOARD_INFO("mtk-usb", 0x60) };


void USB_PHY_Write_Register8(UINT8 var, UINT8 addr)
{
	char buffer[2];

	buffer[0] = addr;
	buffer[1] = var;
	i2c_master_send(usb_i2c_client, buffer, 2);
}

UINT8 USB_PHY_Read_Register8(UINT8 addr)
{
	UINT8 var;

	i2c_master_send(usb_i2c_client, &addr, 1);
	i2c_master_recv(usb_i2c_client, &var, 1);
	return var;
}

static int usb_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
#ifdef CONFIG_OF
	unsigned long base;
	/* if i2c probe before musb prob, this would cause KE */
	/* base = (unsigned long)((unsigned long)mtk_musb->xceiv->io_priv); */
	base = usb_phy_base;
	DBG(0, "[MUSB]usb_i2c_probe, start, base:%lx\n", base);
#endif
	usb_i2c_client = client;

#ifdef CONFIG_OF
	/* disable usb mac suspend */
	DRV_WriteReg8(base + 0x86a, 0x00);
#else
	DRV_WriteReg8(USB_SIF_BASE + 0x86a, 0x00);
#endif
	/* usb phy initial sequence */
	USB_PHY_Write_Register8(0x00, 0xFF);
	USB_PHY_Write_Register8(0x04, 0x61);
	USB_PHY_Write_Register8(0x00, 0x68);
	USB_PHY_Write_Register8(0x00, 0x6a);
	USB_PHY_Write_Register8(0x6e, 0x00);
	USB_PHY_Write_Register8(0x0c, 0x1b);
	USB_PHY_Write_Register8(0x44, 0x08);
	USB_PHY_Write_Register8(0x55, 0x11);
	USB_PHY_Write_Register8(0x68, 0x1a);


	DBG(0, "[MUSB]addr: 0xFF, value: %x\n", USB_PHY_Read_Register8(0xFF));
	DBG(0, "[MUSB]addr: 0x61, value: %x\n", USB_PHY_Read_Register8(0x61));
	DBG(0, "[MUSB]addr: 0x68, value: %x\n", USB_PHY_Read_Register8(0x68));
	DBG(0, "[MUSB]addr: 0x6a, value: %x\n", USB_PHY_Read_Register8(0x6a));
	DBG(0, "[MUSB]addr: 0x00, value: %x\n", USB_PHY_Read_Register8(0x00));
	DBG(0, "[MUSB]addr: 0x1b, value: %x\n", USB_PHY_Read_Register8(0x1b));
	DBG(0, "[MUSB]addr: 0x08, value: %x\n", USB_PHY_Read_Register8(0x08));
	DBG(0, "[MUSB]addr: 0x11, value: %x\n", USB_PHY_Read_Register8(0x11));
	DBG(0, "[MUSB]addr: 0x1a, value: %x\n", USB_PHY_Read_Register8(0x1a));


	DBG(0, "[MUSB]usb_i2c_probe, end\n");
	return 0;

}

/*static int usb_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {
    strcpy(info->type, "mtk-usb");
    return 0;
}*/

static int usb_i2c_remove(struct i2c_client *client)
{
	return 0;
}


struct i2c_driver usb_i2c_driver = {
	.probe = usb_i2c_probe,
	.remove = usb_i2c_remove,
	/*.detect = usb_i2c_detect, */
	.driver = {
		   .name = "mtk-usb",
		   },
	.id_table = usb_i2c_id,
};

static int add_usb_i2c_driver(void)
{
	i2c_register_board_info(2, &usb_i2c_dev, 1);

	if (i2c_add_driver(&usb_i2c_driver) != 0) {
		DBG(0, "[MUSB]usb_i2c_driver initialization failed!!\n");
		return -1;
	}
	DBG(0, "[MUSB]usb_i2c_driver initialization succeed!!\n");
	return 0;
}
#endif				/* End of FPGA_PLATFORM */

static int __init mt_usb_init(struct musb *musb)
{
#ifndef CONFIG_MTK_LEGACY
	int ret;
#endif
	DBG(0, "mt_usb_init\n");

	usb_phy_generic_register();
	musb->xceiv = usb_get_phy(USB_PHY_TYPE_USB2);

	if (IS_ERR_OR_NULL(musb->xceiv)) {
		DBG(0, "[MUSB] usb_get_phy error!!\n");
		return -EPROBE_DEFER;
	}
#ifdef CONFIG_OF
	/* musb->nIrq = usb_irq_number1; */
#else
	musb->nIrq = USB_MCU_IRQ_BIT1_ID;
#endif
	musb->dma_irq = (int)SHARE_IRQ;
	musb->fifo_cfg = fifo_cfg;
	musb->fifo_cfg_size = ARRAY_SIZE(fifo_cfg);
	musb->dyn_fifo = true;
	musb->power = false;
	musb->is_host = false;
	musb->fifo_size = 8 * 1024;

	wake_lock_init(&musb->usb_lock, WAKE_LOCK_SUSPEND, "USB suspend lock");

#ifndef FPGA_PLATFORM
#ifdef CONFIG_ARCH_MT6735
	INIT_WORK(&vcore_work, do_vcore_work);
	vcore_wq  = create_singlethread_workqueue("usb20_vcore_work");
	INIT_LIST_HEAD(&vcore_req_free_pool);
	INIT_LIST_HEAD(&vcore_req_execute_pool);
#endif
#endif

#ifndef FPGA_PLATFORM
#ifdef CONFIG_MTK_LEGACY
	hwPowerOn(MT6328_POWER_LDO_VUSB33, VOL_3300, "VUSB_LDO");
	DBG(0, "enable VBUS LDO\n");
#else
	reg = regulator_get(musb->controller, "vusb33");
	if (!IS_ERR(reg)) {
#define	VUSB33_VOL_MIN 3300000
#define	VUSB33_VOL_MAX 3300000
		ret = regulator_set_voltage(reg, VUSB33_VOL_MIN, VUSB33_VOL_MAX);
		if (ret < 0)
			DBG(0, "regulator set vol failed: %d\n", ret);
		else
			DBG(0, "regulator set vol ok, <%d,%d>\n", VUSB33_VOL_MIN, VUSB33_VOL_MAX);
		ret = regulator_enable(reg);
		if (ret < 0) {
			DBG(0, "regulator_enable failed: %d\n", ret);
			regulator_put(reg);
		} else {
			DBG(0, "enable USB regulator\n");
		}
	} else {
		DBG(0, "regulator_get failed\n");
	}
#endif
#endif

	/* mt_usb_enable(musb); */

	musb->isr = mt_usb_interrupt;
	musb_writel(musb->mregs, MUSB_HSDMA_INTR, 0xff | (0xff << DMA_INTR_UNMASK_SET_OFFSET));
	DBG(0, "musb platform init %x\n", musb_readl(musb->mregs, MUSB_HSDMA_INTR));

#ifdef MUSB_QMU_SUPPORT
	/* FIXME, workaround for device_qmu + host_dma */
	musb_writel(musb->mregs, USB_L1INTM,
		    TX_INT_STATUS | RX_INT_STATUS | USBCOM_INT_STATUS | DMA_INT_STATUS |
		    QINT_STATUS);
#else
	musb_writel(musb->mregs, USB_L1INTM,
		    TX_INT_STATUS | RX_INT_STATUS | USBCOM_INT_STATUS | DMA_INT_STATUS);
#endif

	setup_timer(&musb_idle_timer, musb_do_idle, (unsigned long)musb);

#ifdef CONFIG_USB_MTK_OTG
	mt_usb_otg_init(musb);
#endif

	return 0;
}

static int mt_usb_exit(struct musb *musb)
{
	del_timer_sync(&musb_idle_timer);
	return 0;
}

static const struct musb_platform_ops mt_usb_ops = {
	.init = mt_usb_init,
	.exit = mt_usb_exit,
	/*.set_mode     = mt_usb_set_mode, */
	.try_idle = mt_usb_try_idle,
	.enable = mt_usb_enable,
	.disable = mt_usb_disable,
	.set_vbus = mt_usb_set_vbus,
	.vbus_status = mt_usb_get_vbus_status
};

static u64 mt_usb_dmamask = DMA_BIT_MASK(32);

static int mt_usb_probe(struct platform_device *pdev)
{
	struct musb_hdrc_platform_data *pdata = pdev->dev.platform_data;
	struct platform_device *musb;
	struct mt_usb_glue *glue;
#ifdef CONFIG_OF
	struct musb_hdrc_config *config;
	struct device_node *np = pdev->dev.of_node;
#endif
#ifdef CONFIG_MTK_UART_USB_SWITCH
	struct device_node *ap_uart0_node = NULL;
#endif
	int ret = -ENOMEM;

	glue = kzalloc(sizeof(*glue), GFP_KERNEL);
	if (!glue) {
		/* dev_err(&pdev->dev, "failed to allocate glue context\n"); */
		goto err0;
	}

	musb = platform_device_alloc("musb-hdrc", PLATFORM_DEVID_AUTO);
	if (!musb) {
		dev_err(&pdev->dev, "failed to allocate musb device\n");
		goto err1;
	}
#ifdef CONFIG_OF
	dts_np = pdev->dev.of_node;

	/* usb_irq_number1 = irq_of_parse_and_map(pdev->dev.of_node, 0); */
	usb_phy_base = (unsigned long)of_iomap(pdev->dev.of_node, 1);
	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev, "failed to allocate musb platform data\n");
		goto err2;
	}

	config = devm_kzalloc(&pdev->dev, sizeof(*config), GFP_KERNEL);
	if (!config) {
		/* dev_err(&pdev->dev, "failed to allocate musb hdrc config\n"); */
		goto err2;
	}
#ifdef CONFIG_USB_MTK_OTG
	pdata->mode = MUSB_OTG;
#else
	of_property_read_u32(np, "mode", (u32 *) &pdata->mode);
#endif

#ifdef CONFIG_MTK_UART_USB_SWITCH
	ap_uart0_node = of_find_compatible_node(NULL, NULL, AP_UART0_COMPATIBLE_NAME);

	if (ap_uart0_node == NULL) {
		dev_err(&pdev->dev, "USB get ap_uart0_node failed\n");
		if (ap_uart0_base)
			iounmap(ap_uart0_base);
		ap_uart0_base = 0;
	} else {
		ap_uart0_base = of_iomap(ap_uart0_node, 0);
	}
#endif

	of_property_read_u32(np, "num_eps", (u32 *) &config->num_eps);
	config->multipoint = of_property_read_bool(np, "multipoint");

	/* deprecated on musb.h, mark it to reduce build warning */
#if 0
	of_property_read_u32(np, "dma_channels", (u32 *) &config->dma_channels);
	config->dyn_fifo = of_property_read_bool(np, "dyn_fifo");
	config->soft_con = of_property_read_bool(np, "soft_con");
	config->dma = of_property_read_bool(np, "dma");
#endif

	pdata->config = config;
#endif

	musb->dev.parent = &pdev->dev;
	musb->dev.dma_mask = &mt_usb_dmamask;
	musb->dev.coherent_dma_mask = mt_usb_dmamask;
#ifdef CONFIG_OF
	pdev->dev.dma_mask = &mt_usb_dmamask;
	pdev->dev.coherent_dma_mask = mt_usb_dmamask;
#endif

	glue->dev = &pdev->dev;
	glue->musb = musb;

	pdata->platform_ops = &mt_usb_ops;

	platform_set_drvdata(pdev, glue);

	ret = platform_device_add_resources(musb, pdev->resource, pdev->num_resources);
	if (ret) {
		dev_err(&pdev->dev, "failed to add resources\n");
		goto err2;
	}

	ret = platform_device_add_data(musb, pdata, sizeof(*pdata));
	if (ret) {
		dev_err(&pdev->dev, "failed to add platform_data\n");
		goto err2;
	}

	ret = platform_device_add(musb);

	if (ret) {
		dev_err(&pdev->dev, "failed to register musb device\n");
		goto err2;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_cmode);
	ret = device_create_file(&pdev->dev, &dev_attr_saving);
#ifdef CONFIG_MTK_UART_USB_SWITCH
	ret = device_create_file(&pdev->dev, &dev_attr_portmode);
	ret = device_create_file(&pdev->dev, &dev_attr_tx);
	ret = device_create_file(&pdev->dev, &dev_attr_rx);
	ret = device_create_file(&pdev->dev, &dev_attr_uartpath);
#endif

	if (ret) {
		dev_err(&pdev->dev, "failed to create musb device\n");
		goto err2;
	}
#ifdef CONFIG_OF
	DBG(0, "USB probe done!\n");
#endif

#if defined(FPGA_PLATFORM) || defined(FOR_BRING_UP)
	musb_force_on = 1;
#endif
	if (get_boot_mode() == META_BOOT) {
		DBG(0, "in special mode %d\n", get_boot_mode());
		musb_force_on = 1;
	}

	return 0;

err2:
	platform_device_put(musb);

err1:
	kfree(glue);

err0:
	return ret;
}

static int mt_usb_dts_probe(struct platform_device *pdev)
{
	int retval = 0;

	register_usb_hal_disconnect_check(trigger_disconnect_check_work);

	/* enable uart log */
	musb_uart_debug = 1;

	DBG(0, "first_connect, check_delay_done to 0\n");
	first_connect = 0;
	check_delay_done = 0;

	DBG(0, "set musb_connect_legacy to 0\n");
	musb_connect_legacy = 0;
	DBG(0, "init connection_work\n");
	INIT_DELAYED_WORK(&connection_work, do_connection_work);
	DBG(0, "keep musb->power & mtk_usb_power in the samae value\n");
	mtk_usb_power = false;

#ifndef CONFIG_MTK_CLKMGR
	musb_clk = devm_clk_get(&pdev->dev, "usb0");
	if (IS_ERR(musb_clk)) {
		DBG(0, KERN_WARNING "cannot get musb clock\n");
		return PTR_ERR(musb_clk);
	}
	DBG(0, KERN_WARNING "get musb clock ok, prepare it\n");
	retval = clk_prepare(musb_clk);
	if (retval == 0) {
		DBG(0, KERN_WARNING "prepare done\n");
	} else {
		DBG(0, KERN_WARNING "prepare fail\n");
		return retval;
	}
#endif

	mt_usb_device.dev.of_node = pdev->dev.of_node;
	retval = platform_device_register(&mt_usb_device);
	if (retval != 0)
		DBG(0, "register musbfsh device fail!\n");

	if (usb20_phy_init_debugfs())
		DBG(0, "usb20_phy_init_debugfs fail!\n");
	return retval;

}

static int mt_usb_remove(struct platform_device *pdev)
{
	struct mt_usb_glue *glue = platform_get_drvdata(pdev);

	platform_device_unregister(glue->musb);
	kfree(glue);

	return 0;
}

static int mt_usb_dts_remove(struct platform_device *pdev)
{
	struct mt_usb_glue *glue = platform_get_drvdata(pdev);

	platform_device_unregister(glue->musb);
	kfree(glue);

#ifndef CONFIG_MTK_CLKMGR
	clk_unprepare(musb_clk);
#endif

	return 0;
}


static struct platform_driver mt_usb_driver = {
	.remove = mt_usb_remove,
	.probe = mt_usb_probe,
	.driver = {
		   .name = "mt_usb",
		   },
};

static struct platform_driver mt_usb_dts_driver = {
	.remove = mt_usb_dts_remove,
	.probe = mt_usb_dts_probe,
	.driver = {
		   .name = "mt_dts_usb",
#ifdef CONFIG_OF
		   .of_match_table = apusb_of_ids,
#endif
		   },
};


static int __init usb20_init(void)
{
	DBG(0, "usb20 init\n");

#ifdef CONFIG_MTK_USB2JTAG_SUPPORT
	if (usb2jtag_mode()) {
		pr_err("[USB2JTAG] in usb2jtag mode, not to initialize usb driver\n");
		return 0;
	}
#endif

#ifdef FPGA_PLATFORM
	add_usb_i2c_driver();
#endif
	platform_driver_register(&mt_usb_driver);
	return platform_driver_register(&mt_usb_dts_driver);
}
fs_initcall(usb20_init);

static void __exit usb20_exit(void)
{
	platform_driver_unregister(&mt_usb_driver);
	platform_driver_unregister(&mt_usb_dts_driver);
}
module_exit(usb20_exit);

static int option;
static int set_option(const char *val, const struct kernel_param *kp)
{
	int local_option;
	int rv;

	/* update module parameter */
	rv = param_set_int(val, kp);
	if (rv)
		return rv;

	/* update local_option */
	rv = kstrtoint(val, 10, &local_option);
	if (rv != 0)
		return rv;

	DBG(0, "option:%d, local_option:%d\n", option, local_option);

	switch (local_option) {
	case 0:
		DBG(0, "case %d\n", local_option);
		mt_usb_connect_test(1);
		break;
	case 1:
		DBG(0, "case %d\n", local_option);
		mt_usb_connect_test(0);
		break;
	default:
		break;
	}
	return 0;
}
static struct kernel_param_ops option_param_ops = {
	.set = set_option,
	.get = param_get_int,
};
module_param_cb(option, &option_param_ops, &option, 0644);
