/*
 * Copyright (c) 2015-2017 MICROTRUST Incorporated
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <asm/cacheflush.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <imsg_log.h>
#include "../tz_driver/include/teei_id.h"
#include "../tz_driver/include/tz_service.h"
#include "../tz_driver/include/nt_smc_call.h"
#include "../tz_driver/include/teei_keymaster.h"
#include "../tz_driver/include/teei_client_main.h"
#define TEEI_IOC_MAGIC 'T'

#define SEMA_INIT_ZERO			0
#define KEYMASTER_DRIVER_ID            101
#define KEYMASTER_MAJOR	               254
#define KEYMASTER_SIZE		           (128 * 1024)
#define DEV_NAME                       "ut_keymaster"
#define CMD_MEM_CLEAR	               _IO(TEEI_IOC_MAGIC, 0x1)
#define CMD_MEM_SEND                   _IO(TEEI_IOC_MAGIC, 0x2)
#define CMD_NOTIFY_UTD	               _IO(TEEI_IOC_MAGIC, 0x3)
#define CMD_FIRST_TIME_BOOT            _IO(TEEI_IOC_MAGIC, 0x4)


static int keymaster_major = KEYMASTER_MAJOR;
static struct class *driver_class;
static dev_t devno;
struct semaphore keymaster_api_lock;

struct keymaster_dev {
	struct cdev cdev;
	unsigned char mem[KEYMASTER_SIZE];
	struct semaphore sem;
};
struct keymaster_dev *keymaster_devp;


int keymaster_open(struct inode *inode, struct file *filp)
{
	IMSG_DEBUG("!!!!!microtrust kernel open keymaster dev operation!!!\n");

	if (keymaster_devp != NULL) {
		filp->private_data = keymaster_devp;
	} else {
		IMSG_ERROR("microtrust keymaster_devp is NULL\n");
		return -EINVAL;
	}

	return 0;
}

int keymaster_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;
	return 0;
}

static long keymaster_ioctl(struct file *filp,
				unsigned int cmd, unsigned long arg)
{
	down(&keymaster_api_lock);

	switch (cmd) {
	case CMD_MEM_CLEAR:
		IMSG_DEBUG("microtrust keymaster mem clear.\n");
		break;

	case CMD_MEM_SEND:
		if (!keymaster_buff_addr) {
			IMSG_ERROR("microtrust keymaster fp_buiff_addr is invalid!.\n");
			up(&keymaster_api_lock);
			return -EFAULT;
		}

		memset((void *)keymaster_buff_addr, 0, KEYMASTER_SIZE);

		if (copy_from_user((void *)keymaster_buff_addr,
					(void *)arg, KEYMASTER_SIZE)) {
			IMSG_ERROR(KERN_ERR "microturst keymaster copy from user failed.\n");
			up(&keymaster_api_lock);
			return -EFAULT;
		}

		Flush_Dcache_By_Area((unsigned long)keymaster_buff_addr,
				(unsigned long)keymaster_buff_addr + KEYMASTER_SIZE);

		/*this para 'KEYMASTER_DRIVER_ID' is not used*/
		send_keymaster_command(KEYMASTER_DRIVER_ID);

		if (copy_to_user((void *)arg, (void *)keymaster_buff_addr,
			KEYMASTER_SIZE)) {
			IMSG_ERROR("microtrust keymaster copy from user failed.\n");
			up(&keymaster_api_lock);
			return -EFAULT;
		}

		break;

	case CMD_NOTIFY_UTD:

		up(&(boot_decryto_lock));
		break;

	case CMD_FIRST_TIME_BOOT:

		if (!keymaster_buff_addr) {
			IMSG_ERROR("microtrust keymaster fp_buiff_addr is invalid!.\n");
			up(&keymaster_api_lock);
			return -EFAULT;
		}

		memset((void *)keymaster_buff_addr, 0, 8);

		if (copy_from_user((void *)keymaster_buff_addr,
				(void *)arg, 8)) {
			IMSG_ERROR(KERN_ERR "microturst keymaster copy from user failed.\n");
			up(&keymaster_api_lock);
			return -EFAULT;
		}

		Flush_Dcache_By_Area((unsigned long)keymaster_buff_addr,
							(unsigned long)keymaster_buff_addr + 8);

		/*this para 'KEYMASTER_DRIVER_ID' is not used*/
		send_keymaster_command(KEYMASTER_DRIVER_ID);

		if (copy_to_user((void *)arg, (void *)keymaster_buff_addr, 8)) {
			IMSG_ERROR("microtrust keymaster copy from user failed.\n");
			up(&keymaster_api_lock);
			return -EFAULT;
		}

		break;

	default:
		up(&keymaster_api_lock);
		return -EINVAL;
	}

	up(&keymaster_api_lock);
	return 0;
}
static ssize_t keymaster_read(struct file *filp, char __user *buf,
	size_t size, loff_t *ppos)
{
	int ret = 0;
	return ret;
}

static ssize_t keymaster_write(struct file *filp, const char __user *buf,
	size_t size, loff_t *ppos)
{
	return 0;
}

static loff_t keymaster_llseek(struct file *filp, loff_t offset, int orig)
{
	return 0;
}


static const struct file_operations keymaster_fops = {
	.owner = THIS_MODULE,
	.llseek = keymaster_llseek,
	.read = keymaster_read,
	.write = keymaster_write,
	.unlocked_ioctl = keymaster_ioctl,
	.compat_ioctl = keymaster_ioctl,
	.open = keymaster_open,
	.release = keymaster_release,
};

static void keymaster_setup_cdev(struct keymaster_dev *dev, int index)
{
	int err = 0;
	int devno = MKDEV(keymaster_major, index);

	cdev_init(&dev->cdev, &keymaster_fops);
	dev->cdev.owner = keymaster_fops.owner;
	err = cdev_add(&dev->cdev, devno, 1);

	if (err)
		IMSG_ERROR("Error %d adding keymaster %d.\n", err, index);
}

int keymaster_init(void)
{
	int result = 0;
	struct device *class_dev = NULL;

	devno = MKDEV(keymaster_major, 0);
	result = alloc_chrdev_region(&devno, 0, 1, DEV_NAME);
	keymaster_major = MAJOR(devno);
	sema_init(&(keymaster_api_lock), SEMA_INIT_ZERO);

	if (result < 0)
		return result;


	driver_class = NULL;
	driver_class = class_create(THIS_MODULE, DEV_NAME);

	if (IS_ERR(driver_class)) {
		result = -ENOMEM;
		IMSG_ERROR("ut_keymaster class_create failed %d.\n", result);
		goto unregister_chrdev_region;
	}

	class_dev = device_create(driver_class, NULL, devno, NULL, DEV_NAME);

	if (!class_dev) {
		result = -ENOMEM;
		IMSG_ERROR("ut_keymaster class_device_create failed %d.\n", result);
		goto class_destroy;
	}

	keymaster_devp = NULL;
	keymaster_devp = kmalloc(sizeof(struct keymaster_dev), GFP_KERNEL);

	if (keymaster_devp == NULL) {
		result = -ENOMEM;
		goto class_device_destroy;
	}

	memset(keymaster_devp, 0, sizeof(struct keymaster_dev));
	keymaster_setup_cdev(keymaster_devp, 0);
	sema_init(&keymaster_devp->sem, 1);


	IMSG_DEBUG("[%s][%d]create keymaster device node successfully!\n",
			__func__, __LINE__);
	goto return_fn;

class_device_destroy:
	device_destroy(driver_class, devno);
class_destroy:
	class_destroy(driver_class);
unregister_chrdev_region:
	unregister_chrdev_region(devno, 1);
return_fn:
	return result;
}

void keymaster_exit(void)
{
	device_destroy(driver_class, devno);
	class_destroy(driver_class);
	cdev_del(&keymaster_devp->cdev);
	kfree(keymaster_devp);
	unregister_chrdev_region(MKDEV(keymaster_major, 0), 1);
}

MODULE_AUTHOR("Microtrust");
MODULE_LICENSE("Dual BSD/GPL");
module_init(keymaster_init);
module_exit(keymaster_exit);
