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

#include <linux/vmalloc.h>
#include "inc/mag.h"
#include "sensor_performance.h"

struct mag_context *mag_context_obj/* = NULL*/;
static struct mag_init_info *msensor_init_list[MAX_CHOOSE_G_NUM] = {0};

static void initTimer(struct hrtimer *timer, enum hrtimer_restart (*callback)(struct hrtimer *))
{
	hrtimer_init(timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	timer->function = callback;
}

static void startTimer(struct hrtimer *timer, int delay_ms, bool first)
{
	struct mag_context *obj = (struct mag_context *)container_of(timer, struct mag_context, hrTimer);

	if (obj == NULL) {
		MAG_PR_ERR("NULL pointer\n");
		return;
	}

	if (first) {
		obj->target_ktime = ktime_add_ns(ktime_get(), (int64_t)delay_ms*1000000);
	} else {
		do {
			obj->target_ktime = ktime_add_ns(obj->target_ktime, (int64_t)delay_ms*1000000);
		} while (ktime_to_ns(obj->target_ktime) < ktime_to_ns(ktime_get()));
	}

	hrtimer_start(timer, obj->target_ktime, HRTIMER_MODE_ABS);
}

static void stopTimer(struct hrtimer *timer)
{
	hrtimer_cancel(timer);
}

static void mag_work_func(struct work_struct *work)
{
	struct mag_context *cxt = NULL;
	struct hwm_sensor_data sensor_data;
	int64_t m_pre_ns, cur_ns;
	int64_t delay_ms;
	struct timespec time;
	int err;
	int x, y, z, status;

	cxt  = mag_context_obj;
	delay_ms = atomic_read(&cxt->delay);
	memset(&sensor_data, 0, sizeof(sensor_data));
	time.tv_sec = time.tv_nsec = 0;
	get_monotonic_boottime(&time);
	cur_ns = time.tv_sec*1000000000LL+time.tv_nsec;

	err = cxt->mag_dev_data.get_data(&x, &y, &z, &status);
	if (err) {
		MAG_PR_ERR("get data fails!!\n");
		return;
	}
	cxt->drv_data.x = x;
	cxt->drv_data.y = y;
	cxt->drv_data.z = z;
	cxt->drv_data.status = status;
	m_pre_ns = cxt->drv_data.timestamp;
	cxt->drv_data.timestamp = cur_ns;
	if (true ==  cxt->is_first_data_after_enable) {
		m_pre_ns = cur_ns;
		cxt->is_first_data_after_enable = false;
		/* filter -1 value */
		if (cxt->drv_data.x == MAG_INVALID_VALUE ||
			cxt->drv_data.y == MAG_INVALID_VALUE ||
			cxt->drv_data.z == MAG_INVALID_VALUE) {
			MAG_LOG(" read invalid data\n");
			goto mag_loop;
		}
	}
	while ((cur_ns - m_pre_ns) >= delay_ms*1800000LL) {
		struct mag_data tmp_data = cxt->drv_data;

		m_pre_ns += delay_ms*1000000LL;
		tmp_data.timestamp = m_pre_ns;
		mag_data_report(&tmp_data);
	}

	mag_data_report(&cxt->drv_data);

mag_loop:
		/* MAG_LOG("mag_type(%d) data[%d,%d,%d]\n" ,i,cxt->drv_data[i].mag_data.values[0], */
	/* cxt->drv_data[i].mag_data.values[1],cxt->drv_data[i].mag_data.values[2]); */
	if (true == cxt->is_polling_run)
		startTimer(&cxt->hrTimer, atomic_read(&cxt->delay), false);
}

enum hrtimer_restart mag_poll(struct hrtimer *timer)
{
	struct mag_context *obj = (struct mag_context *)container_of(timer, struct mag_context, hrTimer);

	queue_work(obj->mag_workqueue, &obj->report);

	return HRTIMER_NORESTART;
}

static struct mag_context *mag_context_alloc_object(void)
{

	struct mag_context *obj = kzalloc(sizeof(*obj), GFP_KERNEL);

	MAG_LOG("mag_context_alloc_object++++\n");
	if (!obj) {
		MAG_PR_ERR("Alloc magel object error!\n");
		return NULL;
	}

	atomic_set(&obj->delay, 200); /* set work queue delay time 200ms */
	atomic_set(&obj->wake, 0);
	INIT_WORK(&obj->report, mag_work_func);
	obj->mag_workqueue = NULL;
	obj->mag_workqueue = create_workqueue("mag_polling");
	if (!obj->mag_workqueue) {
		kfree(obj);
		return NULL;
	}
	initTimer(&obj->hrTimer, mag_poll);
	obj->is_first_data_after_enable = false;
	obj->is_polling_run = false;
	obj->is_batch_enable = false;
	mutex_init(&obj->mag_op_mutex);
	obj->power = 0;
	obj->enable = 0;
	obj->delay_ns = -1;
	obj->latency_ns = -1;
	MAG_LOG("mag_context_alloc_object----\n");
	return obj;
}

static int mag_enable_and_batch(void)
{
	struct mag_context *cxt = mag_context_obj;
	int err;

	/* power on -> power off */
	if (cxt->power == 1 && cxt->enable == 0) {
		MAG_LOG("MAG disable\n");
		/* stop polling firstly, if needed */
		if (cxt->mag_ctl.is_report_input_direct == false
			&& cxt->is_polling_run == true) {
			smp_mb();/* for memory barrier */
			stopTimer(&cxt->hrTimer);
			smp_mb();/* for memory barrier */
			cancel_work_sync(&cxt->report);
			cxt->drv_data.x = MAG_INVALID_VALUE;
			cxt->drv_data.y = MAG_INVALID_VALUE;
			cxt->drv_data.z = MAG_INVALID_VALUE;
			cxt->is_polling_run = false;
			MAG_LOG("mag stop polling done\n");
		}
		/* turn off the power */
		err = cxt->mag_ctl.enable(0);
		if (err) {
			MAG_PR_ERR("mag turn off power err = %d\n", err);
			return -1;
		}
		MAG_LOG("mag turn off power done\n");

		cxt->power = 0;
		cxt->delay_ns = -1;
		MAG_LOG("MAG disable done\n");
		return 0;
	}
	/* power off -> power on */
	if (cxt->power == 0 && cxt->enable == 1) {
		MAG_LOG("MAG power on\n");
		err = cxt->mag_ctl.enable(1);
		if (err) {
			MAG_PR_ERR("mag turn on power err = %d\n", err);
			return -1;
		}
		MAG_LOG("mag turn on power done\n");

		cxt->power = 1;
		MAG_LOG("MAG power on done\n");
	}
	/* rate change */
	if (cxt->power == 1 && cxt->delay_ns >= 0) {
		MAG_LOG("MAG set batch\n");
		/* set ODR, fifo timeout latency */
		if (cxt->mag_ctl.is_support_batch)
			err = cxt->mag_ctl.batch(0, cxt->delay_ns, cxt->latency_ns);
		else
			err = cxt->mag_ctl.batch(0, cxt->delay_ns, 0);
		if (err) {
			MAG_PR_ERR("mag set batch(ODR) err %d\n", err);
			return -1;
		}
		MAG_LOG("mag set ODR, fifo latency done\n");
		/* start polling, if needed */
		if (cxt->mag_ctl.is_report_input_direct == false) {
			int mdelay = cxt->delay_ns;

			do_div(mdelay, 1000000);
			atomic_set(&cxt->delay, mdelay);
			/* the first sensor start polling timer */
			if (cxt->is_polling_run == false) {
				startTimer(&cxt->hrTimer, atomic_read(&cxt->delay), true);
				cxt->is_polling_run = true;
				cxt->is_first_data_after_enable = true;
			}
			MAG_LOG("mag set polling delay %d ms\n", atomic_read(&cxt->delay));
		}
		MAG_LOG("MAG batch done\n");
	}
	/* just for debug, remove it when everything is ok */
	if (cxt->power == 0 && cxt->delay_ns >= 0)
		MAG_INFO("batch will call firstly in API1.3, do nothing\n");

	return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t mag_show_magdev(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int len = 0;

	MAG_LOG("sensor test: mag function!\n");
	return len;
}
static ssize_t mag_store_active(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mag_context *cxt = mag_context_obj;
	int err = 0;

	MAG_LOG("mag_store_active buf=%s\n", buf);
	mutex_lock(&mag_context_obj->mag_op_mutex);

	if (!strncmp(buf, "1", 1))
		cxt->enable = 1;
	else if (!strncmp(buf, "0", 1))
		cxt->enable = 0;
	else {
		MAG_PR_ERR(" mag_store_active error !!\n");
		err = -1;
		goto err_out;
	}
	err = mag_enable_and_batch();
err_out:
	mutex_unlock(&mag_context_obj->mag_op_mutex);
	MAG_LOG(" mag_store_active done\n");
	return err;
}
/*----------------------------------------------------------------------------*/
static ssize_t mag_show_active(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct mag_context *cxt = NULL;
	int div = 0;

	cxt = mag_context_obj;
	div = cxt->mag_dev_data.div;
	MAG_LOG("mag mag_dev_data m_div value: %d\n", div);
	return snprintf(buf, PAGE_SIZE, "%d\n", div);
}

static ssize_t mag_store_batch(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mag_context *cxt = mag_context_obj;
	int handle = 0, flag = 0, err = 0;

	MAG_LOG(" acc_store_batch %s\n", buf);
	err = sscanf(buf, "%d,%d,%lld,%lld", &handle, &flag,
		&cxt->delay_ns, &cxt->latency_ns);
	if (err != 4) {
		MAG_PR_ERR("mag_store_batch param error: err = %d\n", err);
		return -1;
	}

	mutex_lock(&mag_context_obj->mag_op_mutex);
	err = mag_enable_and_batch();
	mutex_unlock(&mag_context_obj->mag_op_mutex);
	MAG_LOG(" mag_store_batch done: %d\n", cxt->is_batch_enable);
	return err;
}


static ssize_t mag_show_batch(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int len = 0;

	MAG_LOG(" not support now\n");
	return len;
}

static ssize_t mag_show_flush(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static ssize_t mag_store_flush(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mag_context *cxt = NULL;
	int handle = 0, err = 0;

	err = kstrtoint(buf, 10, &handle);
	if (err != 0)
		MAG_PR_ERR("mag_store_flush param error: err = %d\n", err);

	MAG_LOG("mag_store_flush param: handle %d\n", handle);

	mutex_lock(&mag_context_obj->mag_op_mutex);
	cxt = mag_context_obj;
	if (cxt->mag_ctl.flush != NULL)
		err = cxt->mag_ctl.flush();
	else
		MAG_INFO("MAG DRIVER OLD ARCHITECTURE DON'T SUPPORT ACC COMMON VERSION FLUSH\n");
	if (err < 0)
		MAG_PR_ERR("mag enable flush err %d\n", err);
	mutex_unlock(&mag_context_obj->mag_op_mutex);
	return count;
}

static ssize_t mag_show_cali(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static ssize_t mag_store_cali(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mag_context *cxt = NULL;
	int err = 0;
	uint8_t *cali_buf = NULL;

	cali_buf = vzalloc(count);
	if (cali_buf == NULL) {
		MAG_PR_ERR("kzalloc fail\n");
		return -EFAULT;
	}
	memcpy(cali_buf, buf, count);

	mutex_lock(&mag_context_obj->mag_op_mutex);
	cxt = mag_context_obj;
	if (cxt->mag_ctl.set_cali != NULL)
		err = cxt->mag_ctl.set_cali(cali_buf, count);
	else
		MAG_INFO("MAG DRIVER OLD ARCHITECTURE DON'T SUPPORT MAG COMMON VERSION FLUSH\n");
	if (err < 0)
		MAG_PR_ERR("mag set cali err %d\n", err);
	mutex_unlock(&mag_context_obj->mag_op_mutex);
	vfree(cali_buf);
	return count;
}

/* need work around again */
static ssize_t mag_show_sensordevnum(struct device *dev,
				 struct device_attribute *attr, char *buf)
{

	return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static ssize_t mag_show_libinfo(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct mag_context *cxt = mag_context_obj;

	if (!buf)
		return -1;
	memcpy(buf, &cxt->mag_ctl.libinfo, sizeof(struct mag_libinfo_t));
	return sizeof(struct mag_libinfo_t);
}

static int msensor_remove(struct platform_device *pdev)
{
	MAG_LOG("msensor_remove\n");
	return 0;
}

static int msensor_probe(struct platform_device *pdev)
{
	MAG_LOG("msensor_probe\n");
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id msensor_of_match[] = {
	{ .compatible = "mediatek,msensor", },
	{},
};
#endif

static struct platform_driver msensor_driver = {
	.probe	  = msensor_probe,
	.remove	 = msensor_remove,
	.driver = {

		.name  = "msensor",
		#ifdef CONFIG_OF
		.of_match_table = msensor_of_match,
		#endif
	}
};

static int mag_real_driver_init(void)
{
	int i = 0;
	int err = 0;

	MAG_LOG(" mag_real_driver_init +\n");
	for (i = 0; i < MAX_CHOOSE_G_NUM; i++) {
		MAG_LOG(" i=%d\n", i);
		if (msensor_init_list[i] != 0) {
			MAG_LOG(" mag try to init driver %s\n", msensor_init_list[i]->name);
			err = msensor_init_list[i]->init();
			if (err == 0) {
				MAG_LOG(" mag real driver %s probe ok\n", msensor_init_list[i]->name);
				break;
			}
		}
	}

	if (i == MAX_CHOOSE_G_NUM) {
		MAG_LOG(" mag_real_driver_init fail\n");
		err =  -1;
	}
	return err;
}

int mag_driver_add(struct mag_init_info *obj)
{
	int err = 0;
	int i = 0;

	MAG_FUN();
	if (!obj) {
		MAG_PR_ERR("MAG driver add fail, mag_init_info is NULL\n");
		return -1;
	}

	for (i = 0; i < MAX_CHOOSE_G_NUM; i++) {
		if ((i == 0) && (msensor_init_list[0] == NULL)) {
			MAG_LOG("register mensor driver for the first time\n");
			if (platform_driver_register(&msensor_driver))
				MAG_PR_ERR("failed to register msensor driver already exist\n");
		}
		if (msensor_init_list[i] == NULL) {
			obj->platform_diver_addr = &msensor_driver;
			msensor_init_list[i] = obj;
			break;
		}
	}

	if (i >= MAX_CHOOSE_G_NUM) {
		MAG_PR_ERR("MAG driver add err\n");
		err =  -1;
	}

	return err;
}
EXPORT_SYMBOL_GPL(mag_driver_add);
static int magnetic_open(struct inode *inode, struct file *file)
{
	nonseekable_open(inode, file);
	return 0;
}

static ssize_t magnetic_read(struct file *file, char __user *buffer,
			  size_t count, loff_t *ppos)
{
	ssize_t read_cnt = 0;

	read_cnt = sensor_event_read(mag_context_obj->mdev.minor, file, buffer, count, ppos);

	return read_cnt;
}

static unsigned int magnetic_poll(struct file *file, poll_table *wait)
{
	return sensor_event_poll(mag_context_obj->mdev.minor, file, wait);
}

static const struct file_operations mag_fops = {
	.owner = THIS_MODULE,
	.open = magnetic_open,
	.read = magnetic_read,
	.poll = magnetic_poll,
};

static int mag_misc_init(struct mag_context *cxt)
{

	int err = 0;

	cxt->mdev.minor = ID_MAGNETIC;
	cxt->mdev.name  = MAG_MISC_DEV_NAME;
	cxt->mdev.fops = &mag_fops;
	err = sensor_attr_register(&cxt->mdev);
	if (err)
		MAG_PR_ERR("unable to register mag misc device!!\n");

	return err;
}

DEVICE_ATTR(magdev,		S_IWUSR | S_IRUGO, mag_show_magdev, NULL);
DEVICE_ATTR(magactive,	 S_IWUSR | S_IRUGO, mag_show_active, mag_store_active);
DEVICE_ATTR(magbatch,	S_IWUSR | S_IRUGO, mag_show_batch,  mag_store_batch);
DEVICE_ATTR(magflush,		S_IWUSR | S_IRUGO, mag_show_flush,  mag_store_flush);
DEVICE_ATTR(magcali,		S_IWUSR | S_IRUGO, mag_show_cali,  mag_store_cali);
DEVICE_ATTR(magdevnum,	S_IWUSR | S_IRUGO, mag_show_sensordevnum,  NULL);
DEVICE_ATTR(maglibinfo,	          S_IWUSR | S_IRUGO, mag_show_libinfo,  NULL);

static struct attribute *mag_attributes[] = {
	&dev_attr_magdev.attr,
	&dev_attr_magactive.attr,
	&dev_attr_magbatch.attr,
	&dev_attr_magflush.attr,
	&dev_attr_magcali.attr,
	&dev_attr_magdevnum.attr,
	&dev_attr_maglibinfo.attr,
	NULL
};

static struct attribute_group mag_attribute_group = {
	.attrs = mag_attributes
};


int mag_register_data_path(struct mag_data_path *data)
{
	struct mag_context *cxt = NULL;

	cxt = mag_context_obj;
	cxt->mag_dev_data.div = data->div;
	cxt->mag_dev_data.get_data = data->get_data;
	cxt->mag_dev_data.get_raw_data = data->get_raw_data;
	MAG_LOG("mag register data path div: %d\n", cxt->mag_dev_data.div);

	return 0;
}

int mag_register_control_path(struct mag_control_path *ctl)
{
	struct mag_context *cxt = NULL;
	int err = 0;

	cxt = mag_context_obj;
	cxt->mag_ctl.set_delay = ctl->set_delay;
	cxt->mag_ctl.enable = ctl->enable;
	cxt->mag_ctl.open_report_data = ctl->open_report_data;
	cxt->mag_ctl.batch = ctl->batch;
	cxt->mag_ctl.flush = ctl->flush;
	cxt->mag_ctl.set_cali = ctl->set_cali;
	cxt->mag_ctl.is_report_input_direct = ctl->is_report_input_direct;
	cxt->mag_ctl.is_support_batch = ctl->is_support_batch;
	cxt->mag_ctl.is_use_common_factory = ctl->is_use_common_factory;
	memcpy(cxt->mag_ctl.libinfo.libname, ctl->libinfo.libname, sizeof(cxt->mag_ctl.libinfo.libname));
	cxt->mag_ctl.libinfo.layout = ctl->libinfo.layout;
	cxt->mag_ctl.libinfo.deviceid = ctl->libinfo.deviceid;

	if (NULL == cxt->mag_ctl.set_delay || NULL == cxt->mag_ctl.enable
		|| NULL == cxt->mag_ctl.open_report_data) {
		MAG_LOG("mag register control path fail\n");
		return -1;
	}

	/* add misc dev for sensor hal control cmd */
	err = mag_misc_init(mag_context_obj);
	if (err) {
		MAG_PR_ERR("unable to register mag misc device!!\n");
		return -2;
	}
	err = sysfs_create_group(&mag_context_obj->mdev.this_device->kobj,
			&mag_attribute_group);
	if (err < 0) {
		MAG_PR_ERR("unable to create mag attribute file\n");
		return -3;
	}

	kobject_uevent(&mag_context_obj->mdev.this_device->kobj, KOBJ_ADD);

	return 0;
}
static int x1, y1, z1;
static long pc;
static long count;

static int check_repeat_data(int x, int y, int z)
{
	if ((x1 == x) && (y1 == y) && (z1 == z))
		pc++;
	else
		pc = 0;

	x1 = x; y1 = y; z1 = z;

	if (pc > 100) {
		MAG_INFO("Mag sensor output repeat data\n");
		pc = 0;
	}

	return 0;
}

static int check_abnormal_data(int x, int y, int z, int status)
{
	long total;
	struct mag_context *cxt = mag_context_obj;

	total = (x*x + y*y + z*z)/(cxt->mag_dev_data.div * cxt->mag_dev_data.div);
	if ((total < 100) || (total > 10000)) {
		if (count % 10 == 0)
			MAG_INFO("mag sensor abnormal data: x=%d,y=%d,z=%d, status=%d\n", x, y, z, status);
		count++;
		if (count > 1000)
			count = 0;
	}

	return 0;
}

int mag_data_report(struct mag_data *data)
{
	/* MAG_LOG("update!valus: %d, %d, %d, %d\n" , x, y, z, status); */
	struct sensor_event event;
	int err = 0;
	memset(&event, 0, sizeof(struct sensor_event));

	check_repeat_data(data->x, data->y, data->z);
	check_abnormal_data(data->x, data->y, data->z, data->status);
	event.flush_action = DATA_ACTION;
	event.status = data->status;
	event.time_stamp = data->timestamp;
	event.word[0] = data->x;
	event.word[1] = data->y;
	event.word[2] = data->z;
	event.word[3] = data->reserved[0];
	event.word[4] = data->reserved[1];
	event.word[5] = data->reserved[2];
	event.reserved = data->reserved[0];

	if (event.reserved == 1)
		mark_timestamp(ID_MAGNETIC, DATA_REPORT, ktime_get_boot_ns(), event.time_stamp);
	err = sensor_input_event(mag_context_obj->mdev.minor, &event);
	if (err < 0)
		MAG_PR_ERR("failed due to event buffer full\n");
	return err;
}

int mag_bias_report(struct mag_data *data)
{
	/* MAG_LOG("update!valus: %d, %d, %d, %d\n" , x, y, z, status); */
	struct sensor_event event;
	int err = 0;
	memset(&event, 0, sizeof(struct sensor_event));

	event.flush_action = BIAS_ACTION;
	event.word[0] = data->x;
	event.word[1] = data->y;
	event.word[2] = data->z;

	err = sensor_input_event(mag_context_obj->mdev.minor, &event);
	if (err < 0)
		MAG_PR_ERR("failed due to event buffer full\n");
	return err;
}

int mag_flush_report(void)
{
	struct sensor_event event;
	int err = 0;
	memset(&event, 0, sizeof(struct sensor_event));

	MAG_LOG("flush\n");
	event.flush_action = FLUSH_ACTION;
	err = sensor_input_event(mag_context_obj->mdev.minor, &event);
	if (err < 0)
		MAG_PR_ERR("failed due to event buffer full\n");
	return err;
}
static int mag_probe(void)
{
	int err;

	MAG_LOG("+++++++++++++mag_probe!!\n");
	mag_context_obj = mag_context_alloc_object();
	if (!mag_context_obj) {
		err = -ENOMEM;
		MAG_PR_ERR("unable to allocate devobj!\n");
		goto exit_alloc_data_failed;
	}

	/* init real mageleration driver */
	err = mag_real_driver_init();
	if (err) {
		MAG_PR_ERR("mag_real_driver_init fail\n");
		goto real_driver_init_fail;
	}

	MAG_LOG("----magel_probe OK !!\n");
	return 0;

real_driver_init_fail:
	kfree(mag_context_obj);

exit_alloc_data_failed:

	MAG_PR_ERR("----magel_probe fail !!!\n");
	return err;
}

static int mag_remove(void)
{
	int err = 0;

	MAG_FUN(f);
	sysfs_remove_group(&mag_context_obj->mdev.this_device->kobj,
				&mag_attribute_group);

	err = sensor_attr_deregister(&mag_context_obj->mdev);
	if (err)
		MAG_PR_ERR("misc_deregister fail: %d\n", err);

	kfree(mag_context_obj);

	return 0;
}

static int __init mag_init(void)
{
	MAG_FUN();

	if (mag_probe()) {
		MAG_PR_ERR("failed to register mag driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit mag_exit(void)
{
	mag_remove();
	platform_driver_unregister(&msensor_driver);
}

late_initcall(mag_init);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MAGELEROMETER device driver");
MODULE_AUTHOR("Mediatek");

