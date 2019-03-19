/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#define DEBUG	1

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/proc_fs.h> 
#include <linux/regulator/consumer.h>
#include <pn548.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

#include <mt_clkbuf_ctl.h>	/*  for clock buffer */

#define PN544_DRVNAME		"pn544"


#define MAX_BUFFER_SIZE		512
#define I2C_ID_NAME		"pn544"

#define PN544_MAGIC		0xE9
#define PN544_SET_PWR		_IOW(PN544_MAGIC, 0x01, unsigned int)


//#ifndef CONFIG_MTK_I2C_EXTENSION 
//#define CONFIG_MTK_I2C_EXTENSION
//#endif

//#undef CONFIG_MTK_I2C_EXTENSION // baker add ,without DMA Mode
/******************************************************************************
 * extern functions
 *******************************************************************************/
//extern void mt_eint_mask(unsigned int eint_num);
//extern void mt_eint_unmask(unsigned int eint_num);
//extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
//extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);

struct pn544_dev	
{
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct i2c_client	*client;
	struct miscdevice	pn544_device;
	bool			irq_enabled;
	spinlock_t		irq_enabled_lock;

	int			irq_gpio;
	struct regulator	*reg;
};

//struct pn544_i2c_platform_data pn544_platform_data;

struct pinctrl *gpctrl = NULL;
struct pinctrl_state *st_ven_h = NULL;
struct pinctrl_state *st_ven_l = NULL;
struct pinctrl_state *st_dwn_h = NULL;
struct pinctrl_state *st_dwn_l = NULL;

struct pinctrl_state *st_eint_int = NULL;

/* For DMA */

#ifdef CONFIG_MTK_I2C_EXTENSION
static char *I2CDMAWriteBuf;	/*= NULL;*//* unnecessary initialise */
static unsigned int I2CDMAWriteBuf_pa;	/* = NULL; */
static char *I2CDMAReadBuf;	/*= NULL;*//* unnecessary initialise */
static unsigned int I2CDMAReadBuf_pa;	/* = NULL; */
#else
static char I2CDMAWriteBuf[255];
static char I2CDMAReadBuf[255];
#endif

/*****************************************************************************
 * Function
 *****************************************************************************/

static void pn544_enable_irq(struct pn544_dev *pn544_dev)
{
	unsigned long flags;
	printk("%s\n", __func__);

	spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
	enable_irq(pn544_dev->client->irq);
	pn544_dev->irq_enabled = true;
	spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}

static void pn544_disable_irq(struct pn544_dev *pn544_dev)
{
	unsigned long flags;

	printk("%s\n", __func__);


	spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
	if (pn544_dev->irq_enabled) 
	{
		//mt_eint_mask(EINT_NUM);
		disable_irq_nosync(pn544_dev->client->irq);
		pn544_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn544_dev_irq_handler(int irq, void *dev)
{
	struct pn544_dev *pn544_dev = dev;

	printk("pn544_dev_irq_handler()\n");		

	pn544_disable_irq(pn544_dev);

	/* Wake up waiting readers */
	wake_up(&pn544_dev->read_wq);

	return IRQ_HANDLED;
}

static int pn544_platform_pinctrl_select(struct pinctrl *p, struct pinctrl_state *s)
{
	int ret = 0;

	printk("%s\n", __func__);

	if (p != NULL && s != NULL) {
		ret = pinctrl_select_state(p, s);
	} else {
		printk("%s: pinctrl_select err\n", __func__);
		ret = -1;
	}

	return ret;
}

static ssize_t pn544_dev_read(struct file *filp, char __user *buf,
			       size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev = filp->private_data;
	int ret=0,i;//baker_mod;
        int read_retry = 5;
	printk("%s\n", __func__);

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	printk("pn544 %s : reading %zu bytes.\n", __func__, count);
	mutex_lock(&pn544_dev->read_mutex);
	if (!gpio_get_value(pn544_dev->irq_gpio)) {
		printk("pn544 read no event\n");		
		if (filp->f_flags & O_NONBLOCK) 
		{
			ret = -EAGAIN;
			goto fail;
		}
		
		printk("pn544 read wait event\n");		
		pn544_enable_irq(pn544_dev);
	
		printk("pn544 read1  pn544_dev->irq_gpio=%d\n", pn544_dev->irq_gpio);			
		printk("pn544 read2  gpio_get_value(pn544_dev->irq_gpio)=%d\n", gpio_get_value(pn544_dev->irq_gpio));		

		printk("fangzhihua pn544 read1  pn544_dev->irq_gpio=%d\n", pn544_dev->irq_gpio);			

		ret = wait_event_interruptible(pn544_dev->read_wq, gpio_get_value(pn544_dev->irq_gpio));

		printk("cjy pn544 read1  pn544_dev->irq_gpio=%d\n", pn544_dev->irq_gpio);	

		pn544_disable_irq(pn544_dev);

		if (ret) 
		{
			printk("pn544 read wait event error\n");
			goto fail;
		}
	}

#ifdef CONFIG_MTK_I2C_EXTENSION
    if (count < 8) {
#if 0
         pn544_dev->client->addr = ((pn544_dev->client->addr & I2C_MASK_FLAG) | (I2C_ENEXT_FLAG));
         pn544_dev->client->timing = 400;
#endif
          ret = i2c_master_recv(pn544_dev->client, buf, count);
          printk("[fangzhihua] %s ,defined  CONFIG_MTK_I2C_EXTENSION  R1 \n",__func__);
    }
    else
    {
#if 0
   	pn544_dev->client->addr = (((pn544_dev->client->addr & I2C_MASK_FLAG) | (I2C_DMA_FLAG)) | (I2C_ENEXT_FLAG));
   	pn544_dev->client->timing = 400;
#endif
       ret = i2c_master_recv(pn544_dev->client, (unsigned char *)(uintptr_t)I2CDMAReadBuf_pa, count);
	if (ret < 0)
           return ret;
	for (i = 0; i < count; i++)
           buf[i] = I2CDMAReadBuf[i];
    printk("[fangzhihua] %s ,defined  CONFIG_MTK_I2C_EXTENSION R2 \n",__func__);
   }

#else
#if 0
    if (count < 8)
    {
          ret = i2c_master_recv(pn544_dev->client, buf, count);
         printk("[fangzhihua] %s ,defined  CONFIG_MTK_I2C_EXTENSION R3 \n",__func__);
    }
    else {
        ret = i2c_master_recv(pn544_dev->client, (unsigned char *)(uintptr_t)I2CDMAReadBuf, count);
       if (ret < 0)
               return ret;
	   
       for (i = 0; i < count; i++)
              buf[i] = I2CDMAReadBuf[i];
      printk("[fangzhihua] %s ,defined  CONFIG_MTK_I2C_EXTENSION R4 \n",__func__);

    }
#else
	while (read_retry) {
		ret =
		    i2c_master_recv(pn544_dev->client,
				    (unsigned char *)(uintptr_t) I2CDMAReadBuf,
				    count);

		/* mutex_unlock(&mt6605_dev->read_mutex); */

		/*pr_debug("%s : i2c_master_recv returned=%d, irq status=%d\n", __func__,
			 ret, mt_nfc_get_gpio_value(mt6605_dev->irq_gpio));*/

		if (ret < 0) {
			printk("%s: i2c_master_recv failed: %d, read_retry: %d\n",
				__func__, ret, read_retry);
			read_retry--;
			usleep_range(900, 1000);
			continue;
		}
		break;
	}
        for (i = 0; i < count; i++)
           buf[i] = I2CDMAReadBuf[i];
#endif
#endif                            /* CONFIG_MTK_I2C_EXTENSION */

	mutex_unlock(&pn544_dev->read_mutex);

	if (ret < 0) 
	{
		pr_err("pn544 %s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) 
	{
		pr_err("pn544 %s: received too many bytes from i2c (%d)\n", __func__, ret);
		return -EIO;
	}
	
	printk("pn544 IFD->PC:");
	for(i = 0; i < ret; i++) 
	{
		printk(" %02X", I2CDMAReadBuf[i]);
	}
	printk("\n");

	return ret;

fail:
	mutex_unlock(&pn544_dev->read_mutex);
	return ret;
}


static ssize_t pn544_dev_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev;
	int ret=0, i,idx = 0; //baker_mod_w=0;
	char tmp[MAX_BUFFER_SIZE];
	
	printk("%s\n", __func__);

	pn544_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;
	if (copy_from_user(tmp, buf, count)) 
	{
		printk("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(I2CDMAWriteBuf, &buf[(idx*255)], count)) 
	{
		printk(KERN_DEBUG "%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}
	msleep(15);
	printk("pn544 %s : writing %zu bytes.\n", __func__, count);


#ifdef CONFIG_MTK_I2C_EXTENSION
   for (i = 0; i < count; i++){
		I2CDMAWriteBuf[i] = buf[i];
		printk("[Bert] buf[%d] = %d\n", i, buf[i]);
   	}
    if (count < 8) {
         //pn544_dev->client->addr = ((pn544_dev->client->addr & I2C_MASK_FLAG) | (I2C_ENEXT_FLAG));
	printk("[fangzhihua] pn544_dev->client->addr =%d\n", pn544_dev->client->addr);
         //pn544_dev->client->timing = 400;
        // ret = i2c_master_send(pn544_dev->client, buf, count);
        ret =  i2c_master_send(pn544_dev->client, tmp, count);
	  printk("[fangzhihua] %s ,CONFIG_MTK_I2C_EXTENSION    w1 , ret = %d \n",__func__, ret);

    }
    else
    {
#if 0
   	pn544_dev->client->addr = (((pn544_dev->client->addr & I2C_MASK_FLAG) | (I2C_DMA_FLAG)) | (I2C_ENEXT_FLAG));
   	pn544_dev->client->timing = 400;
#endif
       ret = i2c_master_send(pn544_dev->client, (unsigned char *)(uintptr_t)I2CDMAWriteBuf_pa, count);
	printk("[fangzhihua] %s ,CONFIG_MTK_I2C_EXTENSION   w2 \n",__func__);
   }
#else
   for (i = 0; i < count; i++)
		I2CDMAWriteBuf[i] = buf[i];
#if 0
    if (count < 8)
    {
          ret = i2c_master_send(pn544_dev->client, buf, count);
	   printk("[fangzhihua] %s ,CONFIG_MTK_I2C_EXTENSION   w3 \n",__func__);
    }
    else
    {
        ret = i2c_master_send(pn544_dev->client, (unsigned char *)(uintptr_t)I2CDMAWriteBuf, count);
        printk("[fangzhihua] %s ,CONFIG_MTK_I2C_EXTENSION   w4 \n",__func__);
    }
#else
    ret =i2c_master_send(pn544_dev->client,
				    (unsigned char *)(uintptr_t)I2CDMAWriteBuf, count);
#endif
#endif                            /* CONFIG_MTK_I2C_EXTENSION */

	if (ret != count) 
	{
		pr_err("pn544 %s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}
	printk("pn544 PC->IFD:");
	for(i = 0; i < count; i++) 
	{
		printk(" %02X\n", I2CDMAWriteBuf[i]);
	}
	printk("\n");

	return ret;
}

static int pn544_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct pn544_dev *pn544_dev = container_of(filp->private_data, struct pn544_dev, pn544_device);
	
	printk("%s:pn544_dev=%p\n", __func__, pn544_dev);

	filp->private_data = pn544_dev;
	
	pr_debug("pn544 %s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	return ret;
}

static long pn544_dev_unlocked_ioctl(struct file *filp, unsigned int cmd,
				      unsigned long arg)
{
	int ret=0;
	struct pn544_dev *pn544_dev = filp->private_data;

	printk("%s:cmd=%d, arg=%ld, pn544_dev=%p\n", __func__, cmd, arg, pn544_dev);

	switch (cmd) 
	{
		case PN544_SET_PWR:
			if (arg == 2) {
				/* power on with firmware download (requires hw reset) */
				printk("pn544 %s power on with firmware\n", __func__);
				ret = pn544_platform_pinctrl_select(gpctrl, st_ven_h);
				ret = pn544_platform_pinctrl_select(gpctrl, st_dwn_h);
				msleep(10);
				ret = pn544_platform_pinctrl_select(gpctrl, st_ven_l);
				msleep(50);
				ret = pn544_platform_pinctrl_select(gpctrl, st_ven_h);
				msleep(10);
			} else if (arg == 1) {
				/* power on */
				printk("pn544 %s power on\n", __func__);
				ret = pn544_platform_pinctrl_select(gpctrl, st_dwn_l);
				ret = pn544_platform_pinctrl_select(gpctrl, st_ven_h); 
				msleep(10);
			} else  if (arg == 0) {
				/* power off */
				printk("pn544 %s power off\n", __func__);
				ret = pn544_platform_pinctrl_select(gpctrl, st_dwn_l);
				ret = pn544_platform_pinctrl_select(gpctrl, st_ven_l);
				msleep(50);
			} else {
				printk("pn544 %s bad arg %lu\n", __func__, arg);
				return -EINVAL;
			}
			break;
		default:
      {
			        printk("pn544 %s default ioctl %u\n", __func__, cmd);
                        if (arg == 2) {
				/* power on with firmware download (requires hw reset) */
				printk("default pn544 %s power on with firmware\n", __func__);
				ret = pn544_platform_pinctrl_select(gpctrl, st_ven_h);
				ret = pn544_platform_pinctrl_select(gpctrl, st_dwn_h);
				msleep(10);
				ret = pn544_platform_pinctrl_select(gpctrl, st_ven_l);
				msleep(50);
				ret = pn544_platform_pinctrl_select(gpctrl, st_ven_h);
				msleep(10);
			 } else if (arg == 1) {
				/* power on */
				printk("default pn544 %s power on\n", __func__);
				ret = pn544_platform_pinctrl_select(gpctrl, st_dwn_l);
				ret = pn544_platform_pinctrl_select(gpctrl, st_ven_h); 
				msleep(10);
			} else  if (arg == 0) {
				/* power off */
				printk("default pn544 %s power off\n", __func__);
				ret = pn544_platform_pinctrl_select(gpctrl, st_dwn_l);
				ret = pn544_platform_pinctrl_select(gpctrl, st_ven_l);
				msleep(50);
			} else {
				printk("default pn544 %s bad arg %lu\n", __func__, arg);
				return -EINVAL;
			}

    }
	}

	return ret;
}


static const struct file_operations pn544_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = pn544_dev_read,
	.write = pn544_dev_write,
	.open = pn544_dev_open,
#ifdef CONFIG_COMPAT
	.compat_ioctl = pn544_dev_unlocked_ioctl,
#endif
	.unlocked_ioctl = pn544_dev_unlocked_ioctl,
};

static int pn544_remove(struct i2c_client *client)
{
	struct pn544_dev *pn544_dev;

#ifdef CONFIG_MTK_I2C_EXTENSION
	if (I2CDMAWriteBuf) {
		#if 1//def CONFIG_64BIT
		dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMAWriteBuf,
				  I2CDMAWriteBuf_pa);
		#else
		dma_free_coherent(NULL, MAX_BUFFER_SIZE, I2CDMAWriteBuf,
				  I2CDMAWriteBuf_pa);
		#endif
		I2CDMAWriteBuf = NULL;
		I2CDMAWriteBuf_pa = 0;
	}

	if (I2CDMAReadBuf) {
		#if 1//def CONFIG_64BIT
		dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMAReadBuf,
				  I2CDMAReadBuf_pa);
		#else
		dma_free_coherent(NULL, MAX_BUFFER_SIZE, I2CDMAReadBuf,
				  I2CDMAReadBuf_pa);
		#endif
		I2CDMAReadBuf = NULL;
		I2CDMAReadBuf_pa = 0;
	}
#endif

	pn544_dev = i2c_get_clientdata(client);
	misc_deregister(&pn544_dev->pn544_device);
	mutex_destroy(&pn544_dev->read_mutex);
	regulator_put(pn544_dev->reg);
	kfree(pn544_dev);
	return 0;
}

static int pn544_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{

	int ret=0;
	struct pn544_dev *pn544_dev;
	struct device_node *node;

	printk("%s: start...\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		pr_err("%s: need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}

	printk("%s: step02 is ok\n", __func__);

	pn544_dev = kzalloc(sizeof(*pn544_dev), GFP_KERNEL);
	printk("pn544_dev=%p\n", pn544_dev);

	if (pn544_dev == NULL) 
	{
		dev_err(&client->dev, "pn544 failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	memset(pn544_dev, 0, sizeof(struct pn544_dev));

	printk("%s: step03 is ok\n", __func__);

	pn544_dev->client = client;

	/* init mutex and queues */
	init_waitqueue_head(&pn544_dev->read_wq);
	mutex_init(&pn544_dev->read_mutex);
	spin_lock_init(&pn544_dev->irq_enabled_lock);
#if 1
	pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
	pn544_dev->pn544_device.name = PN544_DRVNAME;
	pn544_dev->pn544_device.fops = &pn544_dev_fops;

	ret = misc_register(&pn544_dev->pn544_device);
	if (ret) 
	{
		pr_err("%s: misc_register failed\n", __func__);
		goto err_misc_register;
	}
#endif    
	printk("%s: step04 is ok\n", __func__);
	
	/* request irq.  the irq is set whenever the chip has data available
	* for reading.  it is cleared when all data has been read.
	*/    
#ifdef CONFIG_MTK_I2C_EXTENSION
	client->dev.coherent_dma_mask = DMA_BIT_MASK(32);
#if 1//def CONFIG_64BIT
	I2CDMAWriteBuf =
	    (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE,
				       (dma_addr_t *) &I2CDMAWriteBuf_pa,
				       GFP_KERNEL);
#else
	I2CDMAWriteBuf =
	    (char *)dma_alloc_coherent(NULL, MAX_BUFFER_SIZE,
				       (dma_addr_t *) &I2CDMAWriteBuf_pa,
				       GFP_KERNEL);
#endif

	if (I2CDMAWriteBuf == NULL) {
		pr_err("%s : failed to allocate dma buffer\n", __func__);
		mutex_destroy(&pn544_dev->read_mutex);
		//gpio_free(platform_data->sysrstb_gpio);
		return ret;
	}
#if 1//def CONFIG_64BIT
	I2CDMAReadBuf =
	    (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE,
				       (dma_addr_t *) &I2CDMAReadBuf_pa,
				       GFP_KERNEL);
#else
	I2CDMAReadBuf =
	    (char *)dma_alloc_coherent(NULL, MAX_BUFFER_SIZE,
				       (dma_addr_t *) &I2CDMAReadBuf_pa,
				       GFP_KERNEL);
#endif

	if (I2CDMAReadBuf == NULL) {
		pr_err("%s : failed to allocate dma buffer\n", __func__);
		mutex_destroy(&pn544_dev->read_mutex);
		//gpio_free(platform_data->sysrstb_gpio);
		return ret;
	}
	pr_debug("%s :I2CDMAWriteBuf_pa %d, I2CDMAReadBuf_pa,%d\n", __func__,
		 I2CDMAWriteBuf_pa, I2CDMAReadBuf_pa);
#else
	memset(I2CDMAWriteBuf, 0x00, sizeof(I2CDMAWriteBuf));
	memset(I2CDMAReadBuf, 0x00, sizeof(I2CDMAReadBuf));
#endif

	printk("%s: step05 is ok\n", __func__);

	/*  NFC IRQ settings     */	
	node = of_find_compatible_node(NULL, NULL, "mediatek,nfc-gpio-v2");

	if (node) {
		of_property_read_u32_array(node, "gpio-irq",
					   &(pn544_dev->irq_gpio), 1);		
		printk("pn544_dev->irq_gpio = %d\n", pn544_dev->irq_gpio);
	} else {
		pr_debug("%s : get gpio num err.\n", __func__);
		return -1;
	}

	//node = of_find_compatible_node(NULL, NULL, "mediatek, irq_nfc-eint");
	node = of_find_compatible_node(NULL, NULL, "mediatek,nfc-gpio-v2");
	if (node) {
		client->irq = irq_of_parse_and_map(node, 0);
		printk("pn544 client->irq = %d\n", client->irq);

		ret =
		    request_irq(client->irq, pn544_dev_irq_handler,
				IRQF_TRIGGER_NONE, "nfc_eint_as_int", pn544_dev);

		if (ret) {
			pr_err("%s: EINT IRQ LINE NOT AVAILABLE, ret = %d\n", __func__, ret);
		} else {

			printk("%s: set EINT finished, client->irq=%d\n", __func__,
				 client->irq);

			pn544_dev->irq_enabled = true;
			pn544_disable_irq(pn544_dev);
		}

	} else {
		pr_err("%s: can not find NFC eint compatible node\n",
		       __func__);
	}

	//Temp soultion by Zhao Fei
	clk_buf_ctrl(CLK_BUF_NFC, 1);

	pn544_platform_pinctrl_select(gpctrl, st_ven_h);

	i2c_set_clientdata(client, pn544_dev);
       printk("%s: step06 success\n", __func__);
	return 0;


err_misc_register:
	mutex_destroy(&pn544_dev->read_mutex);
	kfree(pn544_dev);
	return ret;
}

static const struct i2c_device_id pn544_id[] = {
	{I2C_ID_NAME, 0},
	{}
};

static const struct of_device_id nfc_i2c_of_match[] = {
	{.compatible = "mediatek,nfc"},
	{},
};

static struct i2c_driver pn544_i2c_driver = 
{
	.id_table	= pn544_id,
	.probe		= pn544_probe,
	.remove		= pn544_remove,
	/* .detect	= pn544_detect, */
	.driver		= {
		.name = "pn544",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = nfc_i2c_of_match,
#endif
	},
};


static int pn544_platform_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	printk("%s\n", __func__);

	gpctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(gpctrl)) {
		dev_err(&pdev->dev, "Cannot find pinctrl!");
		ret = PTR_ERR(gpctrl);
		goto end;
	}

	st_ven_h = pinctrl_lookup_state(gpctrl, "ven_high");
	if (IS_ERR(st_ven_h)) {
		ret = PTR_ERR(st_ven_h);
		printk("%s: pinctrl err, ven_high\n", __func__);
	}

	st_ven_l = pinctrl_lookup_state(gpctrl, "ven_low");
	if (IS_ERR(st_ven_l)) {
		ret = PTR_ERR(st_ven_l);
		printk("%s: pinctrl err, ven_low\n", __func__);
	}
//modify by tzf@meitu.begin
	st_dwn_h = pinctrl_lookup_state(gpctrl, "eint_high");
    if (IS_ERR(st_dwn_h)) {
    	ret = PTR_ERR(st_dwn_h);
    	printk("%s: pinctrl err, eint_high\n", __func__);
    }
    
    
	st_dwn_l = pinctrl_lookup_state(gpctrl, "eint_low");
    if (IS_ERR(st_dwn_l)) {
    	ret = PTR_ERR(st_dwn_l);
    	printk("%s: pinctrl err, eint_low\n", __func__);
    }
    
    
	st_eint_int = pinctrl_lookup_state(gpctrl, "irq_init");
    if (IS_ERR(st_eint_int)) {
    	ret = PTR_ERR(st_eint_int);
    	printk("%s: pinctrl err, st_irq_init\n", __func__);
    }
    pn544_platform_pinctrl_select(gpctrl, st_eint_int);

end:
	return ret;
}

static int pn544_platform_probe(struct platform_device *pdev)
{
	int ret = 0;

	printk("%s: &pdev=%p\n", __func__, pdev);

	/* pinctrl init */
	ret = pn544_platform_pinctrl_init(pdev);

	return ret;
}

static int pn544_platform_remove(struct platform_device *pdev)
{
	printk("%s: &pdev=%p\n", __func__, pdev);

	return 0;
}

/*  platform driver */
static const struct of_device_id pn544_platform_of_match[] = {
	{.compatible = "mediatek,nfc-gpio-v2",},
	{},
};

static struct platform_driver pn544_platform_driver = {
	.probe		= pn544_platform_probe,
	.remove		= pn544_platform_remove,
	.driver		= {
		.name = PN544_DRVNAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = pn544_platform_of_match,
#endif
	},
};

/*
 * module load/unload record keeping
 */
static int __init pn544_dev_init(void)
{
	int ret;
	printk("pn544_dev_init\n");

	platform_driver_register(&pn544_platform_driver);

	ret = i2c_add_driver(&pn544_i2c_driver);
	printk("[bert] i2c_add_driver  ret = %d \n", ret);
	printk("pn544_dev_init success\n");

	return 0;
}
module_init(pn544_dev_init);

static void __exit pn544_dev_exit(void)
{
	printk("pn544_dev_exit\n");

	i2c_del_driver(&pn544_i2c_driver);
}
module_exit(pn544_dev_exit);

MODULE_AUTHOR("XXX");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");

