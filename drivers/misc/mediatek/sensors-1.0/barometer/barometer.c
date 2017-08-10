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

#include "inc/barometer.h"

struct baro_context *baro_context_obj/* = NULL*/;

static void initTimer(struct hrtimer *timer, enum hrtimer_restart (*callback)(struct hrtimer *))
{
	hrtimer_init(timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	timer->function = callback;
}

static void startTimer(struct hrtimer *timer, int delay_ms, bool first)
{
	struct baro_context *obj = (struct baro_context *)container_of(timer, struct baro_context, hrTimer);

	if (obj == NULL) {
		BARO_PR_ERR("NULL pointer\n");
		return;
	}

	if (first) {
		obj->target_ktime = ktime_add_ns(ktime_get(), (int64_t)delay_ms*1000000);
#if 0
		BARO_LOG("cur_ns = %lld, first_target_ns = %lld\n",
			ktime_to_ns(ktime_get()), ktime_to_ns(obj->target_ktime));
#endif
	} else {
		do {
			obj->target_ktime = ktime_add_ns(obj->target_ktime, (int64_t)delay_ms*1000000);
		} while (ktime_to_ns(obj->target_ktime) < ktime_to_ns(ktime_get()));
#if 0
		BARO_LOG("cur_ns = %lld, target_ns = %lld\n", ktime_to_ns(ktime_get()),
			ktime_to_ns(obj->target_ktime));
#endif
	}

	hrtimer_start(timer, obj->target_ktime, HRTIMER_MODE_ABS);
}

#if !defined(CONFIG_NANOHUB) || !defined(CONFIG_MTK_BAROHUB)
static void stopTimer(struct hrtimer *timer)
{
	hrtimer_cancel(timer);
}
#endif
static struct baro_init_info *barometer_init_list[MAX_CHOOSE_BARO_NUM] = { 0 };

static void baro_work_func(struct work_struct *work)
{

	struct baro_context *cxt = NULL;
	/* hwm_sensor_data sensor_data; */
	int value, status;
	int64_t pre_ns, cur_ns;
	int64_t delay_ms;
	struct timespec time;
	int err;

	cxt = baro_context_obj;
	delay_ms = atomic_read(&cxt->delay);

	if (cxt->baro_data.get_data == NULL)
		BARO_LOG("baro driver not register data path\n");

	time.tv_sec = time.tv_nsec = 0;
	get_monotonic_boottime(&time);
	cur_ns = time.tv_sec*1000000000LL+time.tv_nsec;

	/* add wake lock to make sure data can be read before system suspend */
	err = cxt->baro_data.get_data(&value, &status);

	if (err) {
		BARO_PR_ERR("get baro data fails!!\n");
		goto baro_loop;
	} else {
		{
			cxt->drv_data.baro_data.values[0] = value;
			cxt->drv_data.baro_data.status = status;
			pre_ns = cxt->drv_data.baro_data.time;
			cxt->drv_data.baro_data.time = cur_ns;
		}
	}

	if (true == cxt->is_first_data_after_enable) {
		pre_ns = cur_ns;
		cxt->is_first_data_after_enable = false;
		/* filter -1 value */
		if (cxt->drv_data.baro_data.values[0] == BARO_INVALID_VALUE) {
			BARO_LOG(" read invalid data\n");
			goto baro_loop;

		}
	}
	/* report data to input device */
	/*BARO_LOG("new baro work run....\n"); */
	/*BARO_LOG("baro data[%d].\n", cxt->drv_data.baro_data.values[0]); */

	while ((cur_ns - pre_ns) >= delay_ms*1800000LL) {
		pre_ns += delay_ms*1000000LL;
		baro_data_report(cxt->drv_data.baro_data.values[0],
			cxt->drv_data.baro_data.status, pre_ns);
	}

	baro_data_report(cxt->drv_data.baro_data.values[0],
		cxt->drv_data.baro_data.status, cxt->drv_data.baro_data.time);

baro_loop:
	if (true == cxt->is_polling_run) {
		{
			startTimer(&cxt->hrTimer, atomic_read(&cxt->delay), false);
		}
	}
}

enum hrtimer_restart baro_poll(struct hrtimer *timer)
{
	struct baro_context *obj = (struct baro_context *)container_of(timer, struct baro_context, hrTimer);

	queue_work(obj->baro_workqueue, &obj->report);

	return HRTIMER_NORESTART;
}

static struct baro_context *baro_context_alloc_object(void)
{

	struct baro_context *obj = kzalloc(sizeof(*obj), GFP_KERNEL);

	BARO_LOG("baro_context_alloc_object++++\n");
	if (!obj) {
		BARO_PR_ERR("Alloc baro object error!\n");
		return NULL;
	}
	atomic_set(&obj->delay, 200);	/*5Hz set work queue delay time 200 ms */
	atomic_set(&obj->wake, 0);
	INIT_WORK(&obj->report, baro_work_func);
	obj->baro_workqueue = NULL;
	obj->baro_workqueue = create_workqueue("baro_polling");
	if (!obj->baro_workqueue) {
		kfree(obj);
		return NULL;
	}
	initTimer(&obj->hrTimer, baro_poll);
	obj->is_first_data_after_enable = false;
	obj->is_polling_run = false;
	mutex_init(&obj->baro_op_mutex);
	obj->is_batch_enable = false;	/* for batch mode init */
	obj->power = 0;
	obj->enable = 0;
	obj->delay_ns = -1;
	obj->latency_ns = -1;

	BARO_LOG("baro_context_alloc_object----\n");
	return obj;
}
#if !defined(CONFIG_NANOHUB) || !defined(CONFIG_MTK_BAROHUB)
static int baro_enable_and_batch(void)
{
	struct baro_context *cxt = baro_context_obj;
	int err;

	/* power on -> power off */
	if (cxt->power == 1 && cxt->enable == 0) {
		BARO_LOG("BARO disable\n");
		/* stop polling firstly, if needed */
		if (cxt->baro_ctl.is_report_input_direct == false
			&& cxt->is_polling_run == true) {
			smp_mb();/* for memory barrier */
			stopTimer(&cxt->hrTimer);
			smp_mb();/* for memory barrier */
			cancel_work_sync(&cxt->report);
			cxt->drv_data.baro_data.values[0] = BARO_INVALID_VALUE;
			cxt->is_polling_run = false;
			BARO_LOG("baro stop polling done\n");
		}
		/* turn off the power */
		err = cxt->baro_ctl.enable_nodata(0);
		if (err) {
			BARO_PR_ERR("baro turn off power err = %d\n", err);
			return -1;
		}
		BARO_LOG("baro turn off power done\n");

		cxt->power = 0;
		cxt->delay_ns = -1;
		BARO_LOG("BARO disable done\n");
		return 0;
	}
	/* power off -> power on */
	if (cxt->power == 0 && cxt->enable == 1) {
		BARO_LOG("BARO power on\n");
		err = cxt->baro_ctl.enable_nodata(1);
		if (err) {
			BARO_PR_ERR("baro turn on power err = %d\n", err);
			return -1;
		}
		BARO_LOG("baro turn on power done\n");

		cxt->power = 1;
		BARO_LOG("BARO power on done\n");
	}
	/* rate change */
	if (cxt->power == 1 && cxt->delay_ns >= 0) {
		BARO_LOG("BARO set batch\n");
		/* set ODR, fifo timeout latency */
		if (cxt->baro_ctl.is_support_batch)
			err = cxt->baro_ctl.batch(0, cxt->delay_ns, cxt->latency_ns);
		else
			err = cxt->baro_ctl.batch(0, cxt->delay_ns, 0);
		if (err) {
			BARO_PR_ERR("baro set batch(ODR) err %d\n", err);
			return -1;
		}
		BARO_LOG("baro set ODR, fifo latency done\n");
		/* start polling, if needed */
		if (cxt->baro_ctl.is_report_input_direct == false) {
			int mdelay = cxt->delay_ns;

			do_div(mdelay, 1000000);
			atomic_set(&cxt->delay, mdelay);
			/* the first sensor start polling timer */
			if (cxt->is_polling_run == false) {
				startTimer(&cxt->hrTimer, atomic_read(&cxt->delay), true);
				cxt->is_polling_run = true;
				cxt->is_first_data_after_enable = true;
			}
			BARO_LOG("baro set polling delay %d ms\n", atomic_read(&cxt->delay));
		}
		BARO_LOG("BARO batch done\n");
	}
	/* just for debug, remove it when everything is ok */
	if (cxt->power == 0 && cxt->delay_ns >= 0)
		BARO_LOG("batch will call firstly in API1.3, do nothing\n");

	return 0;
}
#endif
static ssize_t baro_store_active(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct baro_context *cxt = baro_context_obj;
	int err = 0;

	BARO_LOG("baro_store_active buf=%s\n", buf);
	mutex_lock(&baro_context_obj->baro_op_mutex);
	if (!strncmp(buf, "1", 1))
		cxt->enable = 1;
	else if (!strncmp(buf, "0", 1))
		cxt->enable = 0;
	else {
		BARO_PR_ERR(" baro_store_active error !!\n");
		err = -1;
		goto err_out;
	}
#if defined(CONFIG_NANOHUB) && defined(CONFIG_MTK_BAROHUB)
	err = cxt->baro_ctl.enable_nodata(cxt->enable);
	if (err) {
		BARO_PR_ERR("baro turn on power err = %d\n", err);
		return -1;
	}
#else
	err = baro_enable_and_batch();
#endif
err_out:
	mutex_unlock(&baro_context_obj->baro_op_mutex);
	BARO_LOG(" baro_store_active done\n");
	return err;
}

/*----------------------------------------------------------------------------*/
static ssize_t baro_show_active(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct baro_context *cxt = NULL;
	int div;

	cxt = baro_context_obj;
	div = cxt->baro_data.vender_div;

	BARO_LOG("baro vender_div value: %d\n", div);
	return snprintf(buf, PAGE_SIZE, "%d\n", div);
}


static ssize_t baro_store_batch(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct baro_context *cxt = baro_context_obj;
	int handle = 0, flag = 0, err = 0;

	err = sscanf(buf, "%d,%d,%lld,%lld", &handle, &flag,
			&cxt->delay_ns, &cxt->latency_ns);
	if (err != 4)
		BARO_PR_ERR("grav_store_batch param error: err = %d\n", err);

	mutex_lock(&baro_context_obj->baro_op_mutex);
#if defined(CONFIG_NANOHUB) && defined(CONFIG_MTK_BAROHUB)
	if (cxt->baro_ctl.is_support_batch)
		err = cxt->baro_ctl.batch(0, cxt->delay_ns, cxt->latency_ns);
	else
		err = cxt->baro_ctl.batch(0, cxt->delay_ns, 0);
	if (err)
		BARO_PR_ERR("baro set batch(ODR) err %d\n", err);
#else
	err = baro_enable_and_batch();
#endif
	mutex_unlock(&baro_context_obj->baro_op_mutex);
	BARO_LOG(" baro_store_batch done: %d\n", cxt->is_batch_enable);
	return err;

}

static ssize_t baro_show_batch(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static ssize_t baro_store_flush(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct baro_context *cxt = NULL;
	int handle = 0, err = 0;

	err = kstrtoint(buf, 10, &handle);
	if (err != 0)
		BARO_PR_ERR("baro_store_flush param error: err = %d\n", err);

	BARO_LOG("baro_store_flush param: handle %d\n", handle);

	mutex_lock(&baro_context_obj->baro_op_mutex);
	cxt = baro_context_obj;
	if (cxt->baro_ctl.flush != NULL)
		err = cxt->baro_ctl.flush();
	else
		BARO_PR_ERR("BARO DRIVER OLD ARCHITECTURE DON'T SUPPORT BARO COMMON VERSION FLUSH\n");
	if (err < 0)
		BARO_PR_ERR("baro enable flush err %d\n", err);
	mutex_unlock(&baro_context_obj->baro_op_mutex);
	return err;
}

static ssize_t baro_show_flush(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static ssize_t baro_show_devnum(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static int barometer_remove(struct platform_device *pdev)
{
	BARO_LOG("barometer_remove\n");
	return 0;
}

static int barometer_probe(struct platform_device *pdev)
{
	BARO_LOG("barometer_probe\n");
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id barometer_of_match[] = {
	{.compatible = "mediatek,barometer",},
	{},
};
#endif

static struct platform_driver barometer_driver = {
	.probe = barometer_probe,
	.remove = barometer_remove,
	.driver = {
		   .name = "barometer",
#ifdef CONFIG_OF
		   .of_match_table = barometer_of_match,
#endif
		   }
};

static int baro_real_driver_init(void)
{
	int i = 0;
	int err = 0;

	BARO_LOG(" baro_real_driver_init +\n");
	for (i = 0; i < MAX_CHOOSE_BARO_NUM; i++) {
		BARO_LOG(" i=%d\n", i);
		if (barometer_init_list[i] != 0) {
			BARO_LOG(" baro try to init driver %s\n", barometer_init_list[i]->name);
			err = barometer_init_list[i]->init();
			if (err == 0) {
				BARO_LOG(" baro real driver %s probe ok\n",
					 barometer_init_list[i]->name);
				break;
			}
		}
	}

	if (i == MAX_CHOOSE_BARO_NUM) {
		BARO_LOG(" baro_real_driver_init fail\n");
		err = -1;
	}
	return err;
}

int baro_driver_add(struct baro_init_info *obj)
{
	int err = 0;
	int i = 0;

	BARO_FUN();
	if (!obj) {
		BARO_PR_ERR("BARO driver add fail, baro_init_info is NULL\n");
		return -1;
	}

	for (i = 0; i < MAX_CHOOSE_BARO_NUM; i++) {
		if (i == 0) {
			BARO_LOG("register barometer driver for the first time\n");
			if (platform_driver_register(&barometer_driver))
				BARO_PR_ERR("failed to register gensor driver already exist\n");
		}

		if (barometer_init_list[i] == NULL) {
			obj->platform_diver_addr = &barometer_driver;
			barometer_init_list[i] = obj;
			break;
		}
	}
	if (i >= MAX_CHOOSE_BARO_NUM) {
		BARO_PR_ERR("BARO driver add err\n");
		err = -1;
	}

	return err;
} EXPORT_SYMBOL_GPL(baro_driver_add);
static int pressure_open(struct inode *inode, struct file *file)
{
	nonseekable_open(inode, file);
	return 0;
}

static ssize_t pressure_read(struct file *file, char __user *buffer,
			  size_t count, loff_t *ppos)
{
	ssize_t read_cnt = 0;

	read_cnt = sensor_event_read(baro_context_obj->mdev.minor, file, buffer, count, ppos);

	return read_cnt;
}

static unsigned int pressure_poll(struct file *file, poll_table *wait)
{
	return sensor_event_poll(baro_context_obj->mdev.minor, file, wait);
}

static const struct file_operations pressure_fops = {
	.owner = THIS_MODULE,
	.open = pressure_open,
	.read = pressure_read,
	.poll = pressure_poll,
};

static int baro_misc_init(struct baro_context *cxt)
{

	int err = 0;

	cxt->mdev.minor = ID_PRESSURE;
	cxt->mdev.name = BARO_MISC_DEV_NAME;
	cxt->mdev.fops = &pressure_fops;
	err = sensor_attr_register(&cxt->mdev);
	if (err)
		BARO_PR_ERR("unable to register baro misc device!!\n");

	return err;
}

DEVICE_ATTR(baroactive, S_IWUSR | S_IRUGO, baro_show_active, baro_store_active);
DEVICE_ATTR(barobatch, S_IWUSR | S_IRUGO, baro_show_batch, baro_store_batch);
DEVICE_ATTR(baroflush, S_IWUSR | S_IRUGO, baro_show_flush, baro_store_flush);
DEVICE_ATTR(barodevnum, S_IWUSR | S_IRUGO, baro_show_devnum, NULL);


static struct attribute *baro_attributes[] = {
	&dev_attr_baroactive.attr,
	&dev_attr_barobatch.attr,
	&dev_attr_baroflush.attr,
	&dev_attr_barodevnum.attr,
	NULL
};

static struct attribute_group baro_attribute_group = {
	.attrs = baro_attributes
};

int baro_register_data_path(struct baro_data_path *data)
{
	struct baro_context *cxt = NULL;

	cxt = baro_context_obj;
	cxt->baro_data.get_data = data->get_data;
	cxt->baro_data.vender_div = data->vender_div;
	cxt->baro_data.get_raw_data = data->get_raw_data;
	BARO_LOG("baro register data path vender_div: %d\n", cxt->baro_data.vender_div);
	if (cxt->baro_data.get_data == NULL) {
		BARO_LOG("baro register data path fail\n");
		return -1;
	}
	return 0;
}

int baro_register_control_path(struct baro_control_path *ctl)
{
	struct baro_context *cxt = NULL;
	int err = 0;

	cxt = baro_context_obj;
	cxt->baro_ctl.set_delay = ctl->set_delay;
	cxt->baro_ctl.open_report_data = ctl->open_report_data;
	cxt->baro_ctl.enable_nodata = ctl->enable_nodata;
	cxt->baro_ctl.batch = ctl->batch;
	cxt->baro_ctl.flush = ctl->flush;
	cxt->baro_ctl.is_support_batch = ctl->is_support_batch;
	cxt->baro_ctl.is_report_input_direct = ctl->is_report_input_direct;
	cxt->baro_ctl.is_support_batch = ctl->is_support_batch;
	cxt->baro_ctl.is_use_common_factory = ctl->is_use_common_factory;

	if (NULL == cxt->baro_ctl.set_delay || NULL == cxt->baro_ctl.open_report_data
	    || NULL == cxt->baro_ctl.enable_nodata) {
		BARO_LOG("baro register control path fail\n");
		return -1;
	}

	/* add misc dev for sensor hal control cmd */
	err = baro_misc_init(baro_context_obj);
	if (err) {
		BARO_PR_ERR("unable to register baro misc device!!\n");
		return -2;
	}
	err = sysfs_create_group(&baro_context_obj->mdev.this_device->kobj, &baro_attribute_group);
	if (err < 0) {
		BARO_PR_ERR("unable to create baro attribute file\n");
		return -3;
	}

	kobject_uevent(&baro_context_obj->mdev.this_device->kobj, KOBJ_ADD);

	return 0;
}

int baro_data_report(int value, int status, int64_t nt)
{
	struct sensor_event event;
	int err = 0;

	memset(&event, 0, sizeof(struct sensor_event));

	event.flush_action = DATA_ACTION;
	event.time_stamp = nt;
	event.word[0] = value;
	event.status = status;

	err = sensor_input_event(baro_context_obj->mdev.minor, &event);
	if (err < 0)
		pr_err_ratelimited("failed due to event buffer full\n");
	return err;
}

int baro_flush_report(void)
{
	struct sensor_event event;
	int err = 0;

	memset(&event, 0, sizeof(struct sensor_event));

	BARO_LOG("flush\n");
	event.flush_action = FLUSH_ACTION;
	err = sensor_input_event(baro_context_obj->mdev.minor, &event);
	if (err < 0)
		pr_err_ratelimited("failed due to event buffer full\n");
	return err;
}

static int baro_probe(void)
{
	int err;

	BARO_LOG("+++++++++++++baro_probe!!\n");

	baro_context_obj = baro_context_alloc_object();
	if (!baro_context_obj) {
		err = -ENOMEM;
		BARO_PR_ERR("unable to allocate devobj!\n");
		goto exit_alloc_data_failed;
	}

	/* init real baro driver */
	err = baro_real_driver_init();
	if (err) {
		BARO_PR_ERR("baro real driver init fail\n");
		goto real_driver_init_fail;
	}

	BARO_LOG("----baro_probe OK !!\n");
	return 0;

real_driver_init_fail:
	kfree(baro_context_obj);
	baro_context_obj = NULL;
exit_alloc_data_failed:


	BARO_LOG("----baro_probe fail !!!\n");
	return err;
}



static int baro_remove(void)
{
	int err = 0;

	BARO_FUN(f);

	sysfs_remove_group(&baro_context_obj->mdev.this_device->kobj, &baro_attribute_group);

	err = sensor_attr_deregister(&baro_context_obj->mdev);
	if (err)
		BARO_PR_ERR("misc_deregister fail: %d\n", err);
	kfree(baro_context_obj);

	return 0;
}


static int __init baro_init(void)
{
	BARO_FUN();

	if (baro_probe()) {
		BARO_PR_ERR("failed to register baro driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit baro_exit(void)
{
	baro_remove();
	platform_driver_unregister(&barometer_driver);
}

late_initcall(baro_init);
/* module_init(baro_init); */
/* module_exit(baro_exit); */
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BAROMETER device driver");
MODULE_AUTHOR("Mediatek");
