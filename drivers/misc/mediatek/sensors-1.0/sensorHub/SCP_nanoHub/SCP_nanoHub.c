/* SCP sensor hub driver
 *
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 */

#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/atomic.h>
#include <linux/types.h>
#include <linux/module.h>
#include <asm/arch_timer.h>
#include <linux/wakelock.h>
#include <linux/suspend.h>
#include <scp_ipi.h>
#include "scp_helper.h"
#include "scp_excep.h"
#include <linux/time.h>
#include "cust_sensorHub.h"
#include "hwmsensor.h"
#include "sensors_io.h"
#include "SCP_sensorHub.h"
#include "hwmsen_helper.h"
#include "comms.h"
#include "sensor_event.h"
#include "sensor_performance.h"
#include "SCP_power_monitor.h"
#include <asm/arch_timer.h>

/* ALGIN TO SCP SENSOR_DATA_SIZE AT FILE CONTEXTHUB_FW.H, ALGIN
 * TO SCP_SENSOR_HUB_DATA UNION, ALGIN TO STRUCT DATA_UNIT_T
 * SIZEOF(STRUCT DATA_UNIT_T) = SCP_SENSOR_HUB_DATA = SENSOR_DATA_SIZE
 * BUT AT THE MOMENT AP GET DATA THROUGH IPI, WE ONLY TRANSFER
 * 44 BYTES DATA_UNIT_T, THERE ARE 4 BYTES HEADER IN SCP_SENSOR_HUB_DATA
 * HEAD
 */
#define SENSOR_DATA_SIZE 48
/*
 * experience number for delay_count per DELAY_COUNT sensor input delay 10ms
 * msleep(10) system will schedule to hal process then read input node
 */
#define DELAY_COUNT			32
#define SYNC_TIME_CYCLC		10
#define SCP_sensorHub_DEV_NAME        "SCP_sensorHub"

#define CHRE_POWER_RESET_NOTIFY

static int sensor_send_timestamp_to_hub(void);
static int SCP_sensorHub_server_dispatch_data(uint32_t *currWp);
static int SCP_sensorHub_init_flag = -1;
static uint8_t rtc_compensation_suspend;
struct curr_wp_queue {
	spinlock_t buffer_lock;
	uint32_t head;
	uint32_t tail;
	uint32_t bufsize;
	uint32_t *ringbuffer;
};

struct SCP_sensorHub_data {
	struct work_struct power_up_work;

	struct sensorHub_hw *hw;
	struct work_struct direct_push_work;
	struct workqueue_struct	*direct_push_workqueue;
	struct timer_list sync_time_timer;
	struct work_struct sync_time_worker;
	struct wake_lock sync_time_wake_lock;

	volatile struct sensorFIFO *volatile SCP_sensorFIFO;
	struct curr_wp_queue wp_queue;
	phys_addr_t shub_dram_phys;
	phys_addr_t shub_dram_virt;
	SCP_sensorHub_handler dispatch_data_cb[ID_SENSOR_MAX_HANDLE + 1];
	atomic_t traces[ID_SENSOR_MAX_HANDLE];
};
static struct SensorState mSensorState[ID_SENSOR_MAX_HANDLE + 1];
static DEFINE_MUTEX(mSensorState_mtx);
static atomic_t power_status = ATOMIC_INIT(SENSOR_POWER_DOWN);
static DECLARE_WAIT_QUEUE_HEAD(chre_kthread_wait);
static uint8_t chre_kthread_wait_condition;
static struct SCP_sensorHub_data *obj_data;
#define SCP_TAG						"[sensorHub] "
#define SCP_FUN(f)					pr_err(SCP_TAG"%s\n", __func__)
#define SCP_PR_ERR(fmt, args...)	pr_err(SCP_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define SCP_LOG(fmt, args...)		pr_debug(SCP_TAG fmt, ##args)

/* arch counter is 13M, mult is 161319385, shift is 21 */
static inline uint64_t arch_counter_to_ns(uint64_t cyc)
{
#define ARCH_TIMER_MULT 161319385
#define ARCH_TIMER_SHIFT 21
	return (cyc * ARCH_TIMER_MULT) >> ARCH_TIMER_SHIFT;
}
#define FILTER_DATAPOINTS	16
#define FILTER_TIMEOUT		10000000000ULL /* 10 seconds, ~100us drift */
#define FILTER_FREQ			10000000ULL /* 10 ms */
struct moving_average {
	uint64_t last_time;
	uint64_t input[FILTER_DATAPOINTS];
	uint64_t output;
	uint8_t cnt;
	uint8_t tail;
};
static struct moving_average moving_average_algo;
static uint8_t rtc_compensation_suspend;
static void moving_average_filter(struct moving_average *filter, uint64_t ap_time, uint64_t hub_time)
{
	int i = 0;
	int32_t avg;
	uint64_t ret_avg = 0;

	if (ap_time > filter->last_time + FILTER_TIMEOUT || filter->last_time == 0) {
		filter->tail = 0;
		filter->cnt = 0;
	} else if (ap_time < filter->last_time + FILTER_FREQ) {
		return;
	}
	filter->last_time = ap_time;

	filter->input[filter->tail++] = ap_time - hub_time;
	filter->tail &= (FILTER_DATAPOINTS - 1);
	if (filter->cnt < FILTER_DATAPOINTS)
		filter->cnt++;

	/* pr_err("hongxu raw_offset=%lld\n", ap_time - hub_time); */

	for (i = 1, avg = 0; i < filter->cnt; i++)
		avg += (int32_t)(filter->input[i] - filter->input[0]);
	ret_avg = (avg / filter->cnt) + filter->input[0];
	WRITE_ONCE(filter->output, ret_avg);
}

static uint64_t get_filter_output(struct moving_average *filter)
{
	return READ_ONCE(filter->output);
}

struct SCP_sensorHub_Cmd {
	uint32_t reason;
	void (*handler)(SCP_SENSOR_HUB_DATA_P rsp, int rx_len);
};

#define SCP_SENSORHUB_CMD(_reason, _handler) \
	{.reason = _reason, .handler = _handler}

struct ipi_master {
	spinlock_t		    lock;
	struct list_head	queue;
	struct workqueue_struct	*workqueue;
	struct work_struct	work;
};

struct ipi_transfer {
	const unsigned char	*tx_buf;
	unsigned char		rx_buf[SENSOR_DATA_SIZE];
	unsigned int		len;
	struct list_head transfer_list;
};

struct ipi_message {
	struct list_head	transfers;
	struct list_head	queue;
	void			*context;
	int				status;
	void			(*complete)(void *context);
};

struct scp_send_ipi {
	struct completion	 done;
	int			 len;
	int			 count;
	/* data buffers */
	const unsigned char	*tx;
	unsigned char		*rx;
	void			*context;
};

static struct scp_send_ipi txrx_cmd;
static struct ipi_master master;

static inline void ipi_message_init(struct ipi_message *m)
{
	memset(m, 0, sizeof(*m));
	INIT_LIST_HEAD(&m->transfers);
}

static inline void ipi_message_add_tail(struct ipi_transfer *t, struct ipi_message *m)
{
	list_add_tail(&t->transfer_list, &m->transfers);
}

static int ipi_txrx_bufs(struct ipi_transfer *t)
{
	int status = 0, retry = 0;
	int timeout;
	struct scp_send_ipi *hw = &txrx_cmd;

	/* SCP_SENSOR_HUB_DATA_P req = (SCP_SENSOR_HUB_DATA_P)t->tx_buf;
	SCP_PR_ERR("sensorType:%d, action:%d\n", req->req.sensorType, req->req.action); */

	hw->tx = t->tx_buf;
	hw->rx = t->rx_buf;
	hw->len = t->len;

	init_completion(&hw->done);
	hw->context = &hw->done;
	do {
		status = scp_ipi_send(IPI_SENSOR, (unsigned char *)hw->tx, hw->len, 0, SCP_A_ID);
		if (ERROR == status) {
			SCP_PR_ERR("scp_ipi_send fail\n");
			return -1;
		}
		if (BUSY == status) {
			if (retry++ == 1000) {
				SCP_PR_ERR("retry fail\n");
				return -1;
			}
			if (retry % 100 == 0)
				udelay(10);
		}
	} while (BUSY == status);

	if (retry >= 100)
		SCP_PR_ERR("retry time:%d\n", retry);

	timeout = wait_for_completion_timeout(&hw->done, 500 * HZ / 1000);
	if (!timeout) {
		hw->context = NULL;
		SCP_PR_ERR("transfer timeout!");
		return -1;
	}
	hw->context = NULL;
	return hw->count;
}

static void ipi_complete(void *arg)
{
	complete(arg);
}

static void ipi_work(struct work_struct *work)
{
	struct ipi_message	*m, *_m;
	struct ipi_transfer	*t = NULL;
	int			status = 0;

	spin_lock(&master.lock);
	list_for_each_entry_safe(m, _m, &master.queue, queue) {
		list_del(&m->queue);
		spin_unlock(&master.lock);
		list_for_each_entry(t, &m->transfers, transfer_list) {
			if (!t->tx_buf && !t->rx_buf && t->len) {
				status = -EINVAL;
				SCP_PR_ERR("transfer param wrong :%d\n", status);
				break;
			}
			if (t->len)
				status = ipi_txrx_bufs(t);
			if (status < 0) {
				status = -EREMOTEIO;
				/* SCP_PR_ERR("transfer err :%d\n", status); */
				break;
			} else if (status != t->len) {
				status = -EREMOTEIO;
				SCP_PR_ERR("ack err :%d\n", status);
				break;
			}
			status = 0;
		}
		m->status = status;
		m->complete(m->context);
		spin_lock(&master.lock);
	}
	spin_unlock(&master.lock);
}

static int __ipi_transfer(struct ipi_message *m)
{
	m->status = -EINPROGRESS;

	spin_lock(&master.lock);
	list_add_tail(&m->queue, &master.queue);
	queue_work(master.workqueue, &master.work);
	spin_unlock(&master.lock);
	return 0;
}

static int __ipi_xfer(struct ipi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = ipi_complete;
	message->context = &done;

	status = __ipi_transfer(message);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
	}
	message->context = NULL;
	return status;
}

static int scp_ipi_txrx(const unsigned char *txbuf, unsigned int n_tx,
	unsigned char *rxbuf, unsigned int n_rx)
{
	struct ipi_transfer t;
	struct ipi_message	m;
	int status = 0;

	t.tx_buf = txbuf,
	t.len = n_tx,

	ipi_message_init(&m);
	ipi_message_add_tail(&t, &m);
	status =  __ipi_xfer(&m);
	if (status == 0)
		memcpy(rxbuf, t.rx_buf, n_rx);
	return status;
}

static int SCP_sensorHub_ipi_txrx(unsigned char *txrxbuf)
{
	return scp_ipi_txrx(txrxbuf, SENSOR_DATA_SIZE, txrxbuf, SENSOR_DATA_SIZE);
}

static int SCP_sensorHub_ipi_master_init(void)
{
	INIT_WORK(&master.work, ipi_work);
	INIT_LIST_HEAD(&master.queue);
	spin_lock_init(&master.lock);
	master.workqueue = create_singlethread_workqueue("ipi_master");
	if (master.workqueue == NULL) {
		SCP_PR_ERR("workqueue fail\n");
		return -1;
	}

	return 0;
}

int scp_sensorHub_req_send(SCP_SENSOR_HUB_DATA_P data, uint *len, unsigned int wait)
{
	int ret = 0;

	/* SCP_PR_ERR("sensorType = %d, action = %d\n", data->req.sensorType,
	 *	data->req.action);
	 */

	if (*len > SENSOR_DATA_SIZE) {
		SCP_PR_ERR("!!\n");
		return -1;
	}

	if (in_interrupt()) {
		SCP_PR_ERR("Can't do %s in interrupt context!!\n", __func__);
		return -1;
	}

	if (ID_SENSOR_MAX_HANDLE < data->rsp.sensorType) {
		SCP_PR_ERR("SCP_sensorHub_IPI_handler invalid sensor type %d\n", data->rsp.sensorType);
		return -1;
	}
	ret = SCP_sensorHub_ipi_txrx((unsigned char *)data);
	if (ret != 0 || data->rsp.errCode != 0)
		return -1;
	return 0;
}

int scp_sensorHub_data_registration(uint8_t sensor, SCP_sensorHub_handler handler)
{
	struct SCP_sensorHub_data *obj = obj_data;

	if (sensor > ID_SENSOR_MAX_HANDLE) {
		SCP_PR_ERR("SCP_sensorHub_rsp_registration invalid sensor %d\n", sensor);
		return -1;
	}

	if (NULL == handler)
		SCP_PR_ERR("SCP_sensorHub_rsp_registration null handler\n");

	obj->dispatch_data_cb[sensor] = handler;

	return 0;
}
static void SCP_sensorHub_write_wp_queue(SCP_SENSOR_HUB_DATA_P rsp)
{
	struct SCP_sensorHub_data *obj = obj_data;
	struct curr_wp_queue *wp_queue = &obj->wp_queue;

	spin_lock(&wp_queue->buffer_lock);
	wp_queue->ringbuffer[wp_queue->head++] = rsp->notify_rsp.currWp;
	wp_queue->head &= wp_queue->bufsize - 1;
	if (unlikely(wp_queue->head == wp_queue->tail))
		SCP_PR_ERR("dropped currWp due to ringbuffer is full\n");
	spin_unlock(&wp_queue->buffer_lock);
}
static int SCP_sensorHub_fetch_next_wp(uint32_t *currWp)
{
	int have_event;
	struct SCP_sensorHub_data *obj = obj_data;
	struct curr_wp_queue *wp_queue = &obj->wp_queue;

	spin_lock_irq(&wp_queue->buffer_lock);

	have_event = wp_queue->head != wp_queue->tail;
	if (have_event) {
		*currWp = wp_queue->ringbuffer[wp_queue->tail++];
		wp_queue->tail &= wp_queue->bufsize - 1;
	}
	spin_unlock_irq(&wp_queue->buffer_lock);
	/* SCP_PR_ERR("head:%d, tail:%d, currWp:%d\n", wp_queue->head, wp_queue->tail, *currWp); */
	return have_event;
}
static int SCP_sensorHub_read_wp_queue(void)
{
	uint32_t currWp = 0;

	while (SCP_sensorHub_fetch_next_wp(&currWp)) {
		if (SCP_sensorHub_server_dispatch_data(&currWp))
			return -EFAULT;
	}
	return 0;
}
static void SCP_sensorHub_sync_time_work(struct work_struct *work)

{
	struct SCP_sensorHub_data *obj = obj_data;

	sensor_send_timestamp_to_hub();
	mod_timer(&obj->sync_time_timer, jiffies +  SYNC_TIME_CYCLC * HZ);
}

static void SCP_sensorHub_sync_time_func(unsigned long data)
{
	struct SCP_sensorHub_data *obj = obj_data;

	schedule_work(&obj->sync_time_worker);
}

static int SCP_sensorHub_direct_push_work(void *data)
{
	for (;;) {
		wait_event(chre_kthread_wait, READ_ONCE(chre_kthread_wait_condition));
		WRITE_ONCE(chre_kthread_wait_condition, false);
		mark_timestamp(0, WORK_START, ktime_get_boot_ns(), 0);
		SCP_sensorHub_read_wp_queue();
	}
	return 0;
}
static void SCP_sensorHub_enable_cmd(SCP_SENSOR_HUB_DATA_P rsp, int rx_len)
{
	SCP_SENSOR_HUB_DATA_P req = (SCP_SENSOR_HUB_DATA_P)txrx_cmd.tx;

	if (req->req.sensorType != rsp->rsp.sensorType || req->req.action != rsp->rsp.action) {
		SCP_PR_ERR("sensor type %d != %d action %d != %d\n",
			req->req.sensorType, rsp->rsp.sensorType, req->req.action, rsp->rsp.action);
	} else {
		if (txrx_cmd.context == NULL)
			SCP_PR_ERR("after ipi transfer timeout ack occur then dropped this\n");
		else {
			memcpy(txrx_cmd.rx, rsp, rx_len);
			txrx_cmd.count = rx_len;
			complete(txrx_cmd.context);
		}
	}
}
static void SCP_sensorHub_set_delay_cmd(SCP_SENSOR_HUB_DATA_P rsp, int rx_len)
{
	SCP_SENSOR_HUB_DATA_P req = (SCP_SENSOR_HUB_DATA_P)txrx_cmd.tx;

	if (req->req.sensorType != rsp->rsp.sensorType || req->req.action != rsp->rsp.action) {
		SCP_PR_ERR("sensor type %d != %d action %d != %d\n",
			req->req.sensorType, rsp->rsp.sensorType, req->req.action, rsp->rsp.action);
	} else {
		if (txrx_cmd.context == NULL)
			SCP_PR_ERR("after ipi transfer timeout ack occur then dropped this\n");
		else {
			memcpy(txrx_cmd.rx, rsp, rx_len);
			txrx_cmd.count = rx_len;
			complete(txrx_cmd.context);
		}
	}
}
static void SCP_sensorHub_get_data_cmd(SCP_SENSOR_HUB_DATA_P rsp, int rx_len)
{
	SCP_SENSOR_HUB_DATA_P req = (SCP_SENSOR_HUB_DATA_P)txrx_cmd.tx;

	if (req->req.sensorType != rsp->rsp.sensorType || req->req.action != rsp->rsp.action) {
		SCP_PR_ERR("sensor type %d != %d action %d != %d\n",
			req->req.sensorType, rsp->rsp.sensorType, req->req.action, rsp->rsp.action);
	} else {
		if (txrx_cmd.context == NULL)
			SCP_PR_ERR("after ipi transfer timeout ack occur then dropped this\n");
		else {
			memcpy(txrx_cmd.rx, rsp, rx_len);
			txrx_cmd.count = rx_len;
			complete(txrx_cmd.context);
		}
	}
}
static void SCP_sensorHub_batch_cmd(SCP_SENSOR_HUB_DATA_P rsp, int rx_len)
{
	SCP_SENSOR_HUB_DATA_P req = (SCP_SENSOR_HUB_DATA_P)txrx_cmd.tx;

	if (req->req.sensorType != rsp->rsp.sensorType || req->req.action != rsp->rsp.action) {
		SCP_PR_ERR("sensor type %d != %d action %d != %d\n",
			req->req.sensorType, rsp->rsp.sensorType, req->req.action, rsp->rsp.action);
	} else {
		if (txrx_cmd.context == NULL)
			SCP_PR_ERR("after ipi transfer timeout ack occur then dropped this\n");
		else {
			memcpy(txrx_cmd.rx, rsp, rx_len);
			txrx_cmd.count = rx_len;
			complete(txrx_cmd.context);
		}
	}
}
static void SCP_sensorHub_set_cfg_cmd(SCP_SENSOR_HUB_DATA_P rsp, int rx_len)
{
	SCP_SENSOR_HUB_DATA_P req = (SCP_SENSOR_HUB_DATA_P)txrx_cmd.tx;

	if (req->req.sensorType != rsp->rsp.sensorType || req->req.action != rsp->rsp.action) {
		SCP_PR_ERR("sensor type %d != %d action %d != %d\n",
			req->req.sensorType, rsp->rsp.sensorType, req->req.action, rsp->rsp.action);
	} else {
		if (txrx_cmd.context == NULL)
			SCP_PR_ERR("after ipi transfer timeout ack occur then dropped this\n");
		else {
			memcpy(txrx_cmd.rx, rsp, rx_len);
			txrx_cmd.count = rx_len;
			complete(txrx_cmd.context);
		}
	}
}
static void SCP_sensorHub_set_cust_cmd(SCP_SENSOR_HUB_DATA_P rsp, int rx_len)
{
	SCP_SENSOR_HUB_DATA_P req = (SCP_SENSOR_HUB_DATA_P)txrx_cmd.tx;

	if (req->req.sensorType != rsp->rsp.sensorType || req->req.action != rsp->rsp.action) {
		SCP_PR_ERR("sensor type %d != %d action %d != %d\n",
			req->req.sensorType, rsp->rsp.sensorType, req->req.action, rsp->rsp.action);
	} else {
		if (txrx_cmd.context == NULL)
			SCP_PR_ERR("after ipi transfer timeout ack occur then dropped this\n");
		else {
			memcpy(txrx_cmd.rx, rsp, rx_len);
			txrx_cmd.count = rx_len;
			complete(txrx_cmd.context);
		}
	}
}
static void SCP_sensorHub_batch_timeout_cmd(SCP_SENSOR_HUB_DATA_P rsp, int rx_len)
{
	SCP_SENSOR_HUB_DATA_P req = (SCP_SENSOR_HUB_DATA_P)txrx_cmd.tx;

	if (req->req.sensorType != rsp->rsp.sensorType || req->req.action != rsp->rsp.action) {
		SCP_PR_ERR("sensor type %d != %d action %d != %d\n",
			req->req.sensorType, rsp->rsp.sensorType, req->req.action, rsp->rsp.action);
	} else {
		if (txrx_cmd.context == NULL)
			SCP_PR_ERR("after ipi transfer timeout ack occur then dropped this\n");
		else {
			memcpy(txrx_cmd.rx, rsp, rx_len);
			txrx_cmd.count = rx_len;
			complete(txrx_cmd.context);
		}
	}
}
static void SCP_sensorHub_set_timestamp_cmd(SCP_SENSOR_HUB_DATA_P rsp, int rx_len)
{
	SCP_SENSOR_HUB_DATA_P req = (SCP_SENSOR_HUB_DATA_P)txrx_cmd.tx;

	if (req->req.sensorType != rsp->rsp.sensorType || req->req.action != rsp->rsp.action) {
		SCP_PR_ERR("sensor type %d != %d action %d != %d\n",
			req->req.sensorType, rsp->rsp.sensorType, req->req.action, rsp->rsp.action);
	} else {
		if (txrx_cmd.context == NULL)
			SCP_PR_ERR("after ipi transfer timeout ack occur then dropped this\n");
		else {
			memcpy(txrx_cmd.rx, rsp, rx_len);
			txrx_cmd.count = rx_len;
			complete(txrx_cmd.context);
		}
	}
}
static void SCP_sensorHub_moving_average(SCP_SENSOR_HUB_DATA_P rsp)
{
	uint64_t ap_now_time = 0, arch_counter = 0, scp_raw_time = 0, scp_now_time = 0;
	uint64_t ipi_transfer_time = 0;

	if (READ_ONCE(rtc_compensation_suspend)) {
		SCP_PR_ERR("rtc_compensation_suspend is suspended, so drop run algo\n");
		return;
	}
	ap_now_time = ktime_get_boot_ns();
	arch_counter = arch_counter_get_cntvct();
	scp_raw_time = rsp->notify_rsp.scp_timestamp;
	ipi_transfer_time = arch_counter_to_ns(arch_counter - rsp->notify_rsp.arch_counter);
	scp_now_time = scp_raw_time + ipi_transfer_time;
	moving_average_filter(&moving_average_algo, ap_now_time, scp_now_time);
}
static void SCP_sensorHub_notify_cmd(SCP_SENSOR_HUB_DATA_P rsp, int rx_len)
{
	struct SCP_sensorHub_data *obj = obj_data;
#if 0
	struct data_unit_t *event;
	int handle = 0;
#endif

	switch (rsp->notify_rsp.event) {
	case SCP_DIRECT_PUSH:
	case SCP_FIFO_FULL:
		mark_timestamp(0, GOT_IPI, ktime_get_boot_ns(), 0);
		mark_ipi_timestamp(arch_counter_get_cntvct() - rsp->notify_rsp.arch_counter);
#ifdef DEBUG_PERFORMANCE_HW_TICK
		pr_warn("[Performance Debug] ====> AP_get_ipi, Stanley kernel report tick:%llu\n",
				arch_counter_get_cntvct());
#endif
		SCP_sensorHub_moving_average(rsp);
		SCP_sensorHub_write_wp_queue(rsp);
		/* queue_work(obj->direct_push_workqueue, &obj->direct_push_work); */
		WRITE_ONCE(chre_kthread_wait_condition, true);
		wake_up(&chre_kthread_wait);
		break;
	case SCP_NOTIFY:
#if 0
		handle = rsp->rsp.sensorType;
		if (ID_SENSOR_MAX_HANDLE < handle) {
			SCP_PR_ERR("invalid sensor %d\n", handle);
		} else {
			event = (struct data_unit_t *)rsp->notify_rsp.int8_Data;
			if (NULL != obj->dispatch_data_cb[handle])
				obj->dispatch_data_cb[handle](event, NULL);
			else
				SCP_PR_ERR("type:%d don't support this flow?\n", handle);
			if (event->flush_action == FLUSH_ACTION)
				atomic_dec(&mSensorState[handle].flushCnt);
		}
#endif
		break;
#ifdef CHRE_POWER_RESET_NOTIFY
	case SCP_INIT_DONE:
		atomic_set(&power_status, SENSOR_POWER_UP);
		scp_power_monitor_notify(SENSOR_POWER_UP, NULL);
		schedule_work(&obj->power_up_work);
		break;
#endif
	default:
		break;
	}
}

static const struct SCP_sensorHub_Cmd SCP_sensorHub_Cmds[] = {
	SCP_SENSORHUB_CMD(SENSOR_HUB_ACTIVATE,
		SCP_sensorHub_enable_cmd),
	SCP_SENSORHUB_CMD(SENSOR_HUB_SET_DELAY,
		SCP_sensorHub_set_delay_cmd),
	SCP_SENSORHUB_CMD(SENSOR_HUB_GET_DATA,
		SCP_sensorHub_get_data_cmd),
	SCP_SENSORHUB_CMD(SENSOR_HUB_BATCH,
		SCP_sensorHub_batch_cmd),
	SCP_SENSORHUB_CMD(SENSOR_HUB_SET_CONFIG,
		SCP_sensorHub_set_cfg_cmd),
	SCP_SENSORHUB_CMD(SENSOR_HUB_SET_CUST,
		SCP_sensorHub_set_cust_cmd),
	SCP_SENSORHUB_CMD(SENSOR_HUB_BATCH_TIMEOUT,
		SCP_sensorHub_batch_timeout_cmd),
	SCP_SENSORHUB_CMD(SENSOR_HUB_SET_TIMESTAMP,
		SCP_sensorHub_set_timestamp_cmd),
	SCP_SENSORHUB_CMD(SENSOR_HUB_NOTIFY,
		SCP_sensorHub_notify_cmd),
};

const struct SCP_sensorHub_Cmd *SCP_sensorHub_find_cmd(uint32_t packetReason)
{
	int i;
	const struct SCP_sensorHub_Cmd *cmd;

	for (i = 0; i < ARRAY_SIZE(SCP_sensorHub_Cmds); i++) {
		cmd = &SCP_sensorHub_Cmds[i];
		if (cmd->reason == packetReason)
			return cmd;
	}
	return NULL;
}

static void SCP_sensorHub_IPI_handler(int id, void *data, unsigned int len)
{
	SCP_SENSOR_HUB_DATA_P rsp = (SCP_SENSOR_HUB_DATA_P) data;
	const struct SCP_sensorHub_Cmd *cmd;

	if (len > SENSOR_DATA_SIZE) {
		SCP_PR_ERR("SCP_sensorHub_IPI_handler len=%d error\n", len);
		return;
	}
	/*SCP_PR_ERR("sensorType:%d, action=%d event:%d len:%d\n", rsp->rsp.sensorType,
		rsp->rsp.action, rsp->notify_rsp.event, len);*/
	cmd = SCP_sensorHub_find_cmd(rsp->rsp.action);
	if (cmd != NULL)
		cmd->handler(rsp, len);
	else
		SCP_PR_ERR("cannot find cmd!\n");
}
static void SCP_sensorHub_init_sensor_state(void)
{
	mSensorState[ID_ACCELEROMETER].sensorType = ID_ACCELEROMETER;
	mSensorState[ID_ACCELEROMETER].timestamp_filter = true;

	mSensorState[ID_GYROSCOPE].sensorType = ID_GYROSCOPE;
	mSensorState[ID_GYROSCOPE].timestamp_filter = true;
#ifdef CONFIG_MTK_UNCALI_GYROHUB
	mSensorState[ID_GYROSCOPE].alt = ID_GYROSCOPE_UNCALIBRATED;
	mSensorState[ID_GYROSCOPE_UNCALIBRATED].sensorType = ID_GYROSCOPE;
	mSensorState[ID_GYROSCOPE_UNCALIBRATED].alt = ID_GYROSCOPE;
	mSensorState[ID_GYROSCOPE_UNCALIBRATED].timestamp_filter = true;
#endif

	mSensorState[ID_MAGNETIC].sensorType = ID_MAGNETIC;
	mSensorState[ID_MAGNETIC].timestamp_filter = true;
#ifdef CONFIG_MTK_UNCALI_MAGHUB
	mSensorState[ID_MAGNETIC].alt = ID_MAGNETIC_UNCALIBRATED;
	mSensorState[ID_MAGNETIC_UNCALIBRATED].sensorType = ID_MAGNETIC;
	mSensorState[ID_MAGNETIC_UNCALIBRATED].alt = ID_MAGNETIC;
	mSensorState[ID_MAGNETIC_UNCALIBRATED].timestamp_filter = true;
#endif

	mSensorState[ID_LIGHT].sensorType = ID_LIGHT;
	mSensorState[ID_LIGHT].timestamp_filter = false;

	mSensorState[ID_PROXIMITY].sensorType = ID_PROXIMITY;
	mSensorState[ID_PROXIMITY].timestamp_filter = false;

	mSensorState[ID_PRESSURE].sensorType = ID_PRESSURE;
	mSensorState[ID_PRESSURE].timestamp_filter = false;

	mSensorState[ID_ORIENTATION].sensorType = ID_ORIENTATION;
	mSensorState[ID_ORIENTATION].timestamp_filter = true;

	mSensorState[ID_ROTATION_VECTOR].sensorType = ID_ROTATION_VECTOR;
	mSensorState[ID_ROTATION_VECTOR].timestamp_filter = true;

	mSensorState[ID_GAME_ROTATION_VECTOR].sensorType = ID_GAME_ROTATION_VECTOR;
	mSensorState[ID_GAME_ROTATION_VECTOR].timestamp_filter = true;

	mSensorState[ID_GEOMAGNETIC_ROTATION_VECTOR].sensorType = ID_GEOMAGNETIC_ROTATION_VECTOR;
	mSensorState[ID_GEOMAGNETIC_ROTATION_VECTOR].timestamp_filter = true;

	mSensorState[ID_LINEAR_ACCELERATION].sensorType = ID_LINEAR_ACCELERATION;
	mSensorState[ID_LINEAR_ACCELERATION].timestamp_filter = true;

	mSensorState[ID_GRAVITY].sensorType = ID_GRAVITY;
	mSensorState[ID_GRAVITY].timestamp_filter = true;

	mSensorState[ID_SIGNIFICANT_MOTION].sensorType = ID_SIGNIFICANT_MOTION;
	mSensorState[ID_SIGNIFICANT_MOTION].rate = SENSOR_RATE_ONESHOT;
	mSensorState[ID_SIGNIFICANT_MOTION].timestamp_filter = false;

	mSensorState[ID_STEP_COUNTER].sensorType = ID_STEP_COUNTER;
	mSensorState[ID_STEP_COUNTER].rate = SENSOR_RATE_ONCHANGE;
	mSensorState[ID_STEP_COUNTER].timestamp_filter = false;

	mSensorState[ID_STEP_DETECTOR].sensorType = ID_STEP_DETECTOR;
	mSensorState[ID_STEP_DETECTOR].rate = SENSOR_RATE_ONCHANGE;
	mSensorState[ID_STEP_DETECTOR].timestamp_filter = false;

	mSensorState[ID_TILT_DETECTOR].sensorType = ID_TILT_DETECTOR;
	mSensorState[ID_TILT_DETECTOR].rate = SENSOR_RATE_ONCHANGE;
	mSensorState[ID_TILT_DETECTOR].timestamp_filter = false;

	mSensorState[ID_IN_POCKET].sensorType = ID_IN_POCKET;
	mSensorState[ID_IN_POCKET].rate = SENSOR_RATE_ONESHOT;
	mSensorState[ID_IN_POCKET].timestamp_filter = false;

	mSensorState[ID_ACTIVITY].sensorType = ID_ACTIVITY;
	mSensorState[ID_ACTIVITY].timestamp_filter = false;

	mSensorState[ID_GLANCE_GESTURE].sensorType = ID_GLANCE_GESTURE;
	mSensorState[ID_GLANCE_GESTURE].rate = SENSOR_RATE_ONESHOT;
	mSensorState[ID_GLANCE_GESTURE].timestamp_filter = false;

	mSensorState[ID_PICK_UP_GESTURE].sensorType = ID_PICK_UP_GESTURE;
	mSensorState[ID_PICK_UP_GESTURE].rate = SENSOR_RATE_ONESHOT;
	mSensorState[ID_PICK_UP_GESTURE].timestamp_filter = false;

	mSensorState[ID_WAKE_GESTURE].sensorType = ID_WAKE_GESTURE;
	mSensorState[ID_WAKE_GESTURE].rate = SENSOR_RATE_ONESHOT;
	mSensorState[ID_WAKE_GESTURE].timestamp_filter = false;

	mSensorState[ID_ANSWER_CALL].sensorType = ID_ANSWER_CALL;
	mSensorState[ID_ANSWER_CALL].rate = SENSOR_RATE_ONESHOT;
	mSensorState[ID_ANSWER_CALL].timestamp_filter = false;

	mSensorState[ID_STATIONARY_DETECT].sensorType = ID_STATIONARY_DETECT;
	mSensorState[ID_STATIONARY_DETECT].rate = SENSOR_RATE_ONESHOT;
	mSensorState[ID_STATIONARY_DETECT].timestamp_filter = false;

	mSensorState[ID_MOTION_DETECT].sensorType = ID_MOTION_DETECT;
	mSensorState[ID_MOTION_DETECT].rate = SENSOR_RATE_ONESHOT;
	mSensorState[ID_MOTION_DETECT].timestamp_filter = false;

	mSensorState[ID_DEVICE_ORIENTATION].sensorType = ID_DEVICE_ORIENTATION;
	mSensorState[ID_DEVICE_ORIENTATION].rate = SENSOR_RATE_ONCHANGE;
	mSensorState[ID_DEVICE_ORIENTATION].timestamp_filter = false;

	mSensorState[ID_GEOFENCE].sensorType = ID_GEOFENCE;
	mSensorState[ID_GEOFENCE].rate = SENSOR_RATE_ONCHANGE;
	mSensorState[ID_GEOFENCE].timestamp_filter = false;

	mSensorState[ID_FLOOR_COUNTER].sensorType = ID_FLOOR_COUNTER;
	mSensorState[ID_FLOOR_COUNTER].rate = SENSOR_RATE_ONCHANGE;
	mSensorState[ID_FLOOR_COUNTER].timestamp_filter = false;
}

static void init_sensor_config_cmd(struct ConfigCmd *cmd, int handle)
{
	uint8_t alt = mSensorState[handle].alt;

	memset(cmd, 0x00, sizeof(*cmd));

	cmd->evtType = EVT_NO_SENSOR_CONFIG_EVENT;
	cmd->sensorType = mSensorState[handle].sensorType + ID_OFFSET;

	if (alt && mSensorState[alt].enable && mSensorState[handle].enable) {
		cmd->cmd = CONFIG_CMD_ENABLE;
		if (mSensorState[alt].rate > mSensorState[handle].rate)
			cmd->rate = mSensorState[alt].rate;
		else
			cmd->rate = mSensorState[handle].rate;
		if (mSensorState[alt].latency < mSensorState[handle].latency)
			cmd->latency = mSensorState[alt].latency;
		else
			cmd->latency = mSensorState[handle].latency;
	} else if (alt && mSensorState[alt].enable) {
		cmd->cmd = mSensorState[alt].enable ? CONFIG_CMD_ENABLE : CONFIG_CMD_DISABLE;
		cmd->rate = mSensorState[alt].rate;
		cmd->latency = mSensorState[alt].latency;
	} else { /* !alt || !mSensorState[alt].enable */
		cmd->cmd = mSensorState[handle].enable ? CONFIG_CMD_ENABLE : CONFIG_CMD_DISABLE;
		cmd->rate = mSensorState[handle].rate;
		cmd->latency = mSensorState[handle].latency;
	}
}

static int SCP_sensorHub_batch(int handle, int flag, long long samplingPeriodNs,
				  long long maxBatchReportLatencyNs)
{
	struct ConfigCmd cmd;
	int ret = 0;

	if (mSensorState[handle].sensorType || (handle == ID_ACCELEROMETER &&
				mSensorState[handle].sensorType == ID_ACCELEROMETER)) {
		if (samplingPeriodNs > 0 && mSensorState[handle].rate != SENSOR_RATE_ONCHANGE &&
			mSensorState[handle].rate != SENSOR_RATE_ONESHOT)
			mSensorState[handle].rate = 1024000000000ULL / samplingPeriodNs;
		mSensorState[handle].latency = maxBatchReportLatencyNs;
		init_sensor_config_cmd(&cmd, handle);
		if (atomic_read(&power_status) == SENSOR_POWER_UP) {
			ret = nanohub_external_write((const uint8_t *)&cmd, sizeof(struct ConfigCmd));
			if (ret < 0) {
				SCP_PR_ERR("failed enablebatch handle:%d, rate: %d, latency: %lld, cmd:%d\n",
					handle, cmd.rate, cmd.latency, cmd.cmd);
				return -1;
			}
		}
	} else {
		SCP_PR_ERR("unhandle handle=%d, is inited?\n", handle);
		return -1;
	}
	return 0;
}

static int SCP_sensorHub_flush(int handle)
{
	struct ConfigCmd cmd;
	int ret = 0;

	if (mSensorState[handle].sensorType || (handle == ID_ACCELEROMETER &&
				mSensorState[handle].sensorType == ID_ACCELEROMETER)) {
		atomic_inc(&mSensorState[handle].flushCnt);
		init_sensor_config_cmd(&cmd, handle);
		cmd.cmd = CONFIG_CMD_FLUSH;
		if (atomic_read(&power_status) == SENSOR_POWER_UP) {
			ret = nanohub_external_write((const uint8_t *)&cmd, sizeof(struct ConfigCmd));
			if (ret < 0) {
				SCP_PR_ERR("failed flush handle:%d\n", handle);
				return -1;
			}
		}
	} else {
		SCP_PR_ERR("unhandle handle=%d, is inited?\n", handle);
		return -1;
	}
	return 0;
}
static int SCP_sensorHub_report_data(struct data_unit_t *data_t)
{
	struct SCP_sensorHub_data *obj = obj_data;
	int err = 0, sensor_type = 0;
	int64_t timestamp_ms = 0;
	static int64_t last_timestamp_ms[ID_SENSOR_MAX_HANDLE + 1];
	uint8_t alt = 0;
	bool raw_enable = 0, alt_enable = 0;
	bool need_send = false;
	/* int64_t now_enter_timestamp = 0;

	now_enter_timestamp = ktime_get_boot_ns();
	SCP_PR_ERR("type:%d,now time:%lld, scp time: %lld\n",
		data_t->sensor_type, now_enter_timestamp, (data_t->time_stamp + data_t->time_stamp_gpt)); */
	sensor_type = data_t->sensor_type;
	data_t->time_stamp_gpt = get_filter_output(&moving_average_algo);
	/* pr_err("hongxu compensation_offset=%lld\n", data_t->time_stamp_gpt); */
	alt = ACCESS_ONCE(mSensorState[sensor_type].alt);
	if (!alt) {
		raw_enable = ACCESS_ONCE(mSensorState[sensor_type].enable);
	} else if (alt) {
		raw_enable = ACCESS_ONCE(mSensorState[sensor_type].enable);
		alt_enable = ACCESS_ONCE(mSensorState[alt].enable);
	}
	if (ID_SENSOR_MAX_HANDLE < sensor_type)
		SCP_PR_ERR("invalid sensor %d\n", sensor_type);
	else {
		if (NULL == obj->dispatch_data_cb[sensor_type]) {
			SCP_PR_ERR("type:%d don't support this flow?\n", sensor_type);
			return 0;
		}
		if (alt) {
			if (NULL == obj->dispatch_data_cb[alt]) {
				SCP_PR_ERR("alt:%d don't support this flow?\n", alt);
				return 0;
			}
		}
		if (data_t->flush_action != DATA_ACTION)
			need_send = true;
		else {
			/* timestamp filter, drop events which timestamp equal to each other at 1 ms */
			timestamp_ms = (int64_t)(data_t->time_stamp + data_t->time_stamp_gpt) / 1000000;
			if (last_timestamp_ms[sensor_type] != timestamp_ms) {
				last_timestamp_ms[sensor_type] = timestamp_ms;
				need_send = true;
			} else
				need_send = false;
			if (!mSensorState[sensor_type].timestamp_filter)
				need_send = true;
		}
		if (need_send == true) {
			if (!alt) {
				err = obj->dispatch_data_cb[sensor_type](data_t, NULL);
				if (data_t->flush_action == FLUSH_ACTION)
					atomic_dec(&mSensorState[sensor_type].flushCnt);
			} else if (alt) {
				if (alt_enable && data_t->flush_action == DATA_ACTION)
					err = obj->dispatch_data_cb[alt](data_t, NULL);
				else if (alt_enable && data_t->flush_action == FLUSH_ACTION) {
					if (atomic_dec_if_positive(&mSensorState[alt].flushCnt) >= 0)
						err = obj->dispatch_data_cb[alt](data_t, NULL);
				}
				if (raw_enable && data_t->flush_action == DATA_ACTION)
					err = obj->dispatch_data_cb[sensor_type](data_t, NULL);
				else if (raw_enable && data_t->flush_action == FLUSH_ACTION) {
					if (atomic_dec_if_positive(&mSensorState[sensor_type].flushCnt) >= 0)
						err = obj->dispatch_data_cb[sensor_type](data_t, NULL);
				} else if (data_t->flush_action == BIAS_ACTION || data_t->flush_action == CALI_ACTION
					|| data_t->flush_action == TEMP_ACTION)
					err = obj->dispatch_data_cb[sensor_type](data_t, NULL);
			}
		}
	}
	return err;
}
static int SCP_sensorHub_server_dispatch_data(uint32_t *currWp)
{
	struct SCP_sensorHub_data *obj = obj_data;
	char *pStart, *pEnd, *rp, *wp;
	struct data_unit_t event;
	uint32_t wp_copy;
	int err = 0;

	pStart = (char *)obj->SCP_sensorFIFO + offsetof(struct sensorFIFO, data);
	pEnd = pStart + obj->SCP_sensorFIFO->FIFOSize;
	wp_copy = *currWp;
	rp = pStart + obj->SCP_sensorFIFO->rp;
	wp = pStart + wp_copy;


	if (wp < pStart || pEnd < wp) {
		SCP_PR_ERR("FIFO wp invalid : %p, %p, %p\n", pStart, pEnd, wp);
		return -5;
	}
	if (rp == wp) {
		SCP_PR_ERR("FIFO empty\n");
		return 0;
	}
	/* opimize performance for dram, dram have no cacheable, so we should firstly memcpy data to cacheable ram */
	if (rp < wp) {
		while (rp < wp) {
			memcpy_fromio(&event, rp, SENSOR_DATA_SIZE);
			/* this is a work, we sleep here safe enough, data will save in dram and not lost */
			do {
				err = SCP_sensorHub_report_data(&event);
				if (err < 0) {
					usleep_range(2000, 4000);
					SCP_PR_ERR("event buffer full, so sleep some time\n");
				}
			} while (err < 0);
			rp += SENSOR_DATA_SIZE;
		}
	} else if (rp > wp) {
		while (rp < pEnd) {
			memcpy_fromio(&event, rp, SENSOR_DATA_SIZE);
			do {
				err = SCP_sensorHub_report_data(&event);
				if (err < 0) {
					usleep_range(2000, 4000);
					SCP_PR_ERR("event buffer full, so sleep some time\n");
				}
			} while (err < 0);
			rp += SENSOR_DATA_SIZE;
		}
		rp = pStart;
		while (rp < wp) {
			memcpy_fromio(&event, rp, SENSOR_DATA_SIZE);
			do {
				err = SCP_sensorHub_report_data(&event);
				if (err < 0) {
					usleep_range(2000, 4000);
					SCP_PR_ERR("event buffer full, so sleep some time\n");
				}
			} while (err < 0);
			rp += SENSOR_DATA_SIZE;
		}
	}
	/* must obj->SCP_sensorFIFO->rp = wp, there can not obj->SCP_sensorFIFO->rp = obj->SCP_sensorFIFO->wp */
	obj->SCP_sensorFIFO->rp = wp_copy;

	return 0;
}
static int sensor_send_dram_info_to_hub(void)
{				/* call by init done workqueue */
	struct SCP_sensorHub_data *obj = obj_data;
	SCP_SENSOR_HUB_DATA data;
	unsigned int len = 0;
	int err = 0, retry = 0, total = 3;

	obj->shub_dram_phys = scp_get_reserve_mem_phys(SENS_MEM_ID);
	obj->shub_dram_virt = scp_get_reserve_mem_virt(SENS_MEM_ID);

	data.set_config_req.sensorType = 0;
	data.set_config_req.action = SENSOR_HUB_SET_CONFIG;
	data.set_config_req.bufferBase = (unsigned int)(obj->shub_dram_phys & 0xFFFFFFFF);

	len = sizeof(data.set_config_req);
	for (retry = 0; retry < total; ++retry) {
		err = scp_sensorHub_req_send(&data, &len, 1);
		if (err < 0) {
			SCP_PR_ERR("sensor_send_dram_info_to_hub fail!\n");
			continue;
		}
		break;
	}
	if (retry < total)
		SCP_PR_ERR("sensor_send_dram_info_to_hub success!\n");
	return SCP_SENSOR_HUB_SUCCESS;
}
static int sensor_send_timestamp_wake_locked(void)
{
	SCP_SENSOR_HUB_DATA req;
	int len;
	int err = 0;
	uint64_t now_time, arch_counter;

	/* sensor_send_timestamp_to_hub is process context, we only disable irq is safe */
	local_irq_disable();
	now_time = ktime_get_boot_ns();
	arch_counter = arch_counter_get_cntvct();
	local_irq_enable();
	req.set_config_req.sensorType = 0;
	req.set_config_req.action = SENSOR_HUB_SET_TIMESTAMP;
	req.set_config_req.ap_timestamp = now_time;
	req.set_config_req.arch_counter = arch_counter;
	/* pr_err("hongxu, ns=%lld, arch_counter=%lld!\n", now_time, arch_counter); */
	len = sizeof(req.set_config_req);
	err = scp_sensorHub_req_send(&req, &len, 1);
	if (err < 0)
			SCP_PR_ERR("scp_sensorHub_req_send fail!\n");
	return err;
}

static int sensor_send_timestamp_to_hub(void)
{
	int err = 0;
	struct SCP_sensorHub_data *obj = obj_data;

	if (READ_ONCE(rtc_compensation_suspend)) {
		SCP_PR_ERR("rtc_compensation_suspend is suspended, so drop time sync\n");
		return 0;
	}

	wake_lock(&obj->sync_time_wake_lock);
	err = sensor_send_timestamp_wake_locked();
	wake_unlock(&obj->sync_time_wake_lock);
	return err;
}

int sensor_enable_to_hub(uint8_t sensorType, int enabledisable)
{
	struct ConfigCmd cmd;
	int ret = 0;

	if (enabledisable == 1)
		scp_register_feature(SENS_FEATURE_ID);
	mutex_lock(&mSensorState_mtx);
	if (ID_SENSOR_MAX_HANDLE < sensorType) {
		SCP_PR_ERR("invalid sensor %d\n", sensorType);
		ret = -1;
	} else {
		if (mSensorState[sensorType].sensorType || (sensorType == ID_ACCELEROMETER &&
				mSensorState[sensorType].sensorType == ID_ACCELEROMETER)) {
			mSensorState[sensorType].enable = enabledisable;
			init_sensor_config_cmd(&cmd, sensorType);
			if (atomic_read(&power_status) == SENSOR_POWER_UP) {
				ret = nanohub_external_write((const uint8_t *)&cmd, sizeof(struct ConfigCmd));
				if (ret < 0)
					SCP_PR_ERR("failed registerlistener sensorType:%d, cmd:%d\n",
						sensorType, cmd.cmd);
			}
			if (!enabledisable) {
				if (atomic_read(&mSensorState[sensorType].flushCnt) != 0)
					SCP_PR_ERR("handle=%d flush count not equal to 0 when disable\n",
						sensorType);
			}
		} else {
			SCP_PR_ERR("unhandle handle=%d, is inited?\n", sensorType);
			mutex_unlock(&mSensorState_mtx);
			return -1;
		}
	}
	mutex_unlock(&mSensorState_mtx);
	return ret < 0 ? ret : 0;
}

int sensor_set_delay_to_hub(uint8_t sensorType, unsigned int delayms)
{
	int ret = 0;
	long long samplingPeriodNs = delayms * 1000000ULL;

	mutex_lock(&mSensorState_mtx);
	if (ID_SENSOR_MAX_HANDLE < sensorType) {
		SCP_PR_ERR("invalid sensor %d\n", sensorType);
		ret = -1;
	} else {
		ret = SCP_sensorHub_batch(sensorType, 0, samplingPeriodNs, 0);
	}
	mutex_unlock(&mSensorState_mtx);
	return ret < 0 ? ret : 0;
}

int sensor_batch_to_hub(uint8_t sensorType, int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int ret = 0;

	mutex_lock(&mSensorState_mtx);
	if (ID_SENSOR_MAX_HANDLE < sensorType) {
		SCP_PR_ERR("invalid sensor %d\n", sensorType);
		ret = -1;
	} else
		ret = SCP_sensorHub_batch(sensorType, flag, samplingPeriodNs, maxBatchReportLatencyNs);
	mutex_unlock(&mSensorState_mtx);
	return ret;
}

int sensor_flush_to_hub(uint8_t sensorType)
{
	int ret = 0;

	mutex_lock(&mSensorState_mtx);
	if (ID_SENSOR_MAX_HANDLE < sensorType) {
		SCP_PR_ERR("invalid sensor %d\n", sensorType);
		ret = -1;
	} else
		ret = SCP_sensorHub_flush(sensorType);
	mutex_unlock(&mSensorState_mtx);
	return ret;
}

int sensor_cfg_to_hub(uint8_t sensorType, uint8_t *data, uint8_t count)
{
	struct ConfigCmd *cmd = NULL;
	int ret = 0;

	if (ID_SENSOR_MAX_HANDLE < sensorType) {
		SCP_PR_ERR("invalid sensor %d\n", sensorType);
		ret = -1;
	} else {
		cmd = vzalloc(sizeof(struct ConfigCmd) + count);
		if (cmd == NULL)
			return -1;
		cmd->evtType = EVT_NO_SENSOR_CONFIG_EVENT;
		cmd->sensorType = sensorType + ID_OFFSET;
		cmd->cmd = CONFIG_CMD_CFG_DATA;
		memcpy(cmd->data, data, count);
		ret = nanohub_external_write((const uint8_t *)cmd, sizeof(struct ConfigCmd) + count);
		if (ret < 0) {
			SCP_PR_ERR("failed cfg data handle:%d, cmd:%d\n", sensorType, cmd->cmd);
			ret =  -1;
		}
		vfree(cmd);
	}
	return ret;
}

int sensor_calibration_to_hub(uint8_t sensorType)
{
	struct ConfigCmd cmd;
	int ret = 0;

	if (mSensorState[sensorType].sensorType || (sensorType == ID_ACCELEROMETER &&
				mSensorState[sensorType].sensorType == ID_ACCELEROMETER)) {
		init_sensor_config_cmd(&cmd, sensorType);
		cmd.cmd = CONFIG_CMD_CALIBRATE;
		ret = nanohub_external_write((const uint8_t *)&cmd, sizeof(struct ConfigCmd));
		if (ret < 0) {
			SCP_PR_ERR("failed calibration handle:%d\n", sensorType);
			return -1;
		}
	} else {
		SCP_PR_ERR("unhandle handle=%d, is inited?\n", sensorType);
		return -1;
	}
	return 0;
}

int sensor_get_data_from_hub(uint8_t sensorType, struct data_unit_t *data)
{
	SCP_SENSOR_HUB_DATA req;
	struct data_unit_t *data_t;
	int len = 0, err = 0;

	if (atomic_read(&power_status) == SENSOR_POWER_DOWN) {
		SCP_PR_ERR("scp power down, we can not access scp\n");
		return -1;
	}

	req.get_data_req.sensorType = sensorType;
	req.get_data_req.action = SENSOR_HUB_GET_DATA;
	len = sizeof(req.get_data_req);
	err = scp_sensorHub_req_send(&req, &len, 1);
	if (err < 0) {
		SCP_PR_ERR("fail :%d!\n", err);
		return -1;
	}
	if (sensorType != req.get_data_rsp.sensorType ||
	    SENSOR_HUB_GET_DATA != req.get_data_rsp.action || 0 != req.get_data_rsp.errCode) {
		SCP_PR_ERR("req sensorType: %d, rsp sensorType:%d, rsp action:%d, errcode:%d\n", sensorType,
			req.get_data_rsp.sensorType, req.get_data_rsp.action, req.get_data_rsp.errCode);
		return req.get_data_rsp.errCode;
	}

	data_t = (struct data_unit_t *)req.get_data_rsp.data.int8_Data;
	switch (sensorType) {
	case ID_ACCELEROMETER:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->accelerometer_t.x = data_t->accelerometer_t.x;
		data->accelerometer_t.y = data_t->accelerometer_t.y;
		data->accelerometer_t.z = data_t->accelerometer_t.z;
		data->accelerometer_t.x_bias = data_t->accelerometer_t.x_bias;
		data->accelerometer_t.y_bias = data_t->accelerometer_t.y_bias;
		data->accelerometer_t.z_bias = data_t->accelerometer_t.z_bias;
		data->accelerometer_t.status = data_t->accelerometer_t.status;
		break;
	case ID_GRAVITY:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->accelerometer_t.x = data_t->accelerometer_t.x;
		data->accelerometer_t.y = data_t->accelerometer_t.y;
		data->accelerometer_t.z = data_t->accelerometer_t.z;
		data->accelerometer_t.status = data_t->accelerometer_t.status;
		break;
	case ID_LINEAR_ACCELERATION:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->accelerometer_t.x = data_t->accelerometer_t.x;
		data->accelerometer_t.y = data_t->accelerometer_t.y;
		data->accelerometer_t.z = data_t->accelerometer_t.z;
		data->accelerometer_t.status = data_t->accelerometer_t.status;
		break;
	case ID_LIGHT:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->light = data_t->light;
		break;
	case ID_PROXIMITY:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->proximity_t.steps = data_t->proximity_t.steps;
		data->proximity_t.oneshot = data_t->proximity_t.oneshot;
		break;
	case ID_PRESSURE:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->pressure_t.pressure = data_t->pressure_t.pressure;
		data->pressure_t.status = data_t->pressure_t.status;
		break;
	case ID_GYROSCOPE:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->gyroscope_t.x = data_t->gyroscope_t.x;
		data->gyroscope_t.y = data_t->gyroscope_t.y;
		data->gyroscope_t.z = data_t->gyroscope_t.z;
		data->gyroscope_t.x_bias = data_t->gyroscope_t.x_bias;
		data->gyroscope_t.y_bias  = data_t->gyroscope_t.y_bias;
		data->gyroscope_t.z_bias  = data_t->gyroscope_t.z_bias;
		data->gyroscope_t.status = data_t->gyroscope_t.status;
		break;
	case ID_GYROSCOPE_UNCALIBRATED:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->uncalibrated_gyro_t.x = data_t->uncalibrated_gyro_t.x;
		data->uncalibrated_gyro_t.y = data_t->uncalibrated_gyro_t.y;
		data->uncalibrated_gyro_t.z = data_t->uncalibrated_gyro_t.z;
		data->uncalibrated_gyro_t.x_bias = data_t->uncalibrated_gyro_t.x_bias;
		data->uncalibrated_gyro_t.y_bias  = data_t->uncalibrated_gyro_t.y_bias;
		data->uncalibrated_gyro_t.z_bias  = data_t->uncalibrated_gyro_t.z_bias;
		data->uncalibrated_gyro_t.status = data_t->uncalibrated_gyro_t.status;
		break;
	case ID_RELATIVE_HUMIDITY:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->relative_humidity_t.relative_humidity =
		data_t->relative_humidity_t.relative_humidity;
		data->relative_humidity_t.status = data_t->relative_humidity_t.status;
		break;
	case ID_MAGNETIC:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->magnetic_t.x = data_t->magnetic_t.x;
		data->magnetic_t.y = data_t->magnetic_t.y;
		data->magnetic_t.z = data_t->magnetic_t.z;
		data->magnetic_t.x_bias = data_t->magnetic_t.x_bias;
		data->magnetic_t.y_bias = data_t->magnetic_t.y_bias;
		data->magnetic_t.z_bias = data_t->magnetic_t.z_bias;
		data->magnetic_t.status = data_t->magnetic_t.status;
		break;
	case ID_MAGNETIC_UNCALIBRATED:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->uncalibrated_mag_t.x = data_t->uncalibrated_mag_t.x;
		data->uncalibrated_mag_t.y = data_t->uncalibrated_mag_t.y;
		data->uncalibrated_mag_t.z = data_t->uncalibrated_mag_t.z;
		data->uncalibrated_mag_t.x_bias = data_t->uncalibrated_mag_t.x_bias;
		data->uncalibrated_mag_t.y_bias = data_t->uncalibrated_mag_t.y_bias;
		data->uncalibrated_mag_t.z_bias = data_t->uncalibrated_mag_t.z_bias;
		data->uncalibrated_mag_t.status = data_t->uncalibrated_mag_t.status;
		break;
	case ID_GEOMAGNETIC_ROTATION_VECTOR:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->magnetic_t.x = data_t->magnetic_t.x;
		data->magnetic_t.y = data_t->magnetic_t.y;
		data->magnetic_t.z = data_t->magnetic_t.z;
		data->magnetic_t.scalar = data_t->magnetic_t.scalar;
		data->magnetic_t.status = data_t->magnetic_t.status;
		break;
	case ID_ORIENTATION:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->orientation_t.azimuth = data_t->orientation_t.azimuth;
		data->orientation_t.pitch = data_t->orientation_t.pitch;
		data->orientation_t.roll = data_t->orientation_t.roll;
		data->orientation_t.status = data_t->orientation_t.status;
		break;
	case ID_ROTATION_VECTOR:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->orientation_t.azimuth = data_t->orientation_t.azimuth;
		data->orientation_t.pitch = data_t->orientation_t.pitch;
		data->orientation_t.roll = data_t->orientation_t.roll;
		data->orientation_t.scalar = data_t->orientation_t.scalar;
		data->orientation_t.status = data_t->orientation_t.status;
		break;
	case ID_GAME_ROTATION_VECTOR:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->orientation_t.azimuth = data_t->orientation_t.azimuth;
		data->orientation_t.pitch = data_t->orientation_t.pitch;
		data->orientation_t.roll = data_t->orientation_t.roll;
		data->orientation_t.scalar = data_t->orientation_t.scalar;
		data->orientation_t.status = data_t->orientation_t.status;
		break;
	case ID_STEP_COUNTER:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->step_counter_t.accumulated_step_count
		    = data_t->step_counter_t.accumulated_step_count;
		break;
	case ID_STEP_DETECTOR:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->step_detector_t.step_detect = data_t->step_detector_t.step_detect;
		break;
	case ID_SIGNIFICANT_MOTION:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->smd_t.state = data_t->smd_t.state;
		break;
	case ID_HEART_RATE:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->heart_rate_t.bpm = data_t->heart_rate_t.bpm;
		data->heart_rate_t.status = data_t->heart_rate_t.status;
		break;
	case ID_PEDOMETER:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->pedometer_t.accumulated_step_count =
		    data_t->pedometer_t.accumulated_step_count;
		data->pedometer_t.accumulated_step_length =
		    data_t->pedometer_t.accumulated_step_length;
		data->pedometer_t.step_frequency = data_t->pedometer_t.step_frequency;
		data->pedometer_t.step_length = data_t->pedometer_t.step_length;
		break;
	case ID_ACTIVITY:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->activity_data_t.probability[STILL] =
		    data_t->activity_data_t.probability[STILL];
		data->activity_data_t.probability[STANDING] =
		    data_t->activity_data_t.probability[STANDING];
		data->activity_data_t.probability[SITTING] =
		    data_t->activity_data_t.probability[SITTING];
		data->activity_data_t.probability[LYING] =
		    data_t->activity_data_t.probability[LYING];
		data->activity_data_t.probability[ON_FOOT] =
		    data_t->activity_data_t.probability[ON_FOOT];
		data->activity_data_t.probability[WALKING] =
		    data_t->activity_data_t.probability[WALKING];
		data->activity_data_t.probability[RUNNING] =
		    data_t->activity_data_t.probability[RUNNING];
		data->activity_data_t.probability[CLIMBING] =
		    data_t->activity_data_t.probability[CLIMBING];
		data->activity_data_t.probability[ON_BICYCLE] =
		    data_t->activity_data_t.probability[ON_BICYCLE];
		data->activity_data_t.probability[IN_VEHICLE] =
		    data_t->activity_data_t.probability[IN_VEHICLE];
		data->activity_data_t.probability[TILTING] =
		    data_t->activity_data_t.probability[TILTING];
		data->activity_data_t.probability[UNKNOWN] =
		    data_t->activity_data_t.probability[UNKNOWN];
		break;
	case ID_IN_POCKET:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->inpocket_event.state = data_t->inpocket_event.state;
		break;
	case ID_PICK_UP_GESTURE:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->gesture_data_t.probability = data_t->gesture_data_t.probability;
		break;
	case ID_TILT_DETECTOR:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->tilt_event.state = data_t->tilt_event.state;
		break;
	case ID_WAKE_GESTURE:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->gesture_data_t.probability = data_t->gesture_data_t.probability;
		break;
	case ID_GLANCE_GESTURE:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->gesture_data_t.probability = data_t->gesture_data_t.probability;
		break;
	case ID_PDR:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->pdr_event.x = data_t->pdr_event.x;
		data->pdr_event.y = data_t->pdr_event.y;
		data->pdr_event.z = data_t->pdr_event.z;
		data->pdr_event.status = data_t->pdr_event.status;
		break;
	case ID_FLOOR_COUNTER:
		data->time_stamp = data_t->time_stamp;
		data->time_stamp_gpt = data_t->time_stamp_gpt;
		data->floor_counter_t.accumulated_floor_count
		    = data_t->floor_counter_t.accumulated_floor_count;
		break;
	default:
		err = -1;
		break;
	}
	return err;
}

int sensor_set_cmd_to_hub(uint8_t sensorType, CUST_ACTION action, void *data)
{
	SCP_SENSOR_HUB_DATA req;
	int len = 0, err = 0;

	req.get_data_req.sensorType = sensorType;
	req.get_data_req.action = SENSOR_HUB_SET_CUST;

	if (atomic_read(&power_status) == SENSOR_POWER_DOWN) {
		SCP_PR_ERR("scp power down, we can not access scp\n");
		return -1;
	}

	switch (sensorType) {
	case ID_ACCELEROMETER:
		req.set_cust_req.sensorType = ID_ACCELEROMETER;
		req.set_cust_req.action = SENSOR_HUB_SET_CUST;
		switch (action) {
		case CUST_ACTION_RESET_CALI:
			req.set_cust_req.resetCali.action = CUST_ACTION_RESET_CALI;
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.resetCali);
			break;
		case CUST_ACTION_SET_CALI:
			req.set_cust_req.setCali.action = CUST_ACTION_SET_CALI;
			req.set_cust_req.setCali.int32_data[SCP_SENSOR_HUB_X]
			    = *((int32_t *) data + SCP_SENSOR_HUB_X);
			req.set_cust_req.setCali.int32_data[SCP_SENSOR_HUB_Y]
			    = *((int32_t *) data + SCP_SENSOR_HUB_Y);
			req.set_cust_req.setCali.int32_data[SCP_SENSOR_HUB_Z]
			    = *((int32_t *) data + SCP_SENSOR_HUB_Z);
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.setCali);
			break;
		case CUST_ACTION_SET_TRACE:
			req.set_cust_req.setTrace.action = CUST_ACTION_SET_TRACE;
			req.set_cust_req.setTrace.trace = *((int32_t *) data);
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.setTrace);
			break;
		case CUST_ACTION_SET_DIRECTION:
			req.set_cust_req.setDirection.action = CUST_ACTION_SET_DIRECTION;
			req.set_cust_req.setDirection.direction = *((int32_t *) data);
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.setDirection);
			break;
		case CUST_ACTION_SET_FACTORY:
			req.set_cust_req.setFactory.action = CUST_ACTION_SET_FACTORY;
			req.set_cust_req.setFactory.factory = *((int32_t *) data);
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.setFactory);
			break;
		case CUST_ACTION_SHOW_REG:
			req.set_cust_req.showReg.action = CUST_ACTION_SHOW_REG;
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.showReg);
			break;
		default:
			return -1;
		}
		break;
	case ID_LIGHT:
		req.set_cust_req.sensorType = ID_LIGHT;
		req.set_cust_req.action = SENSOR_HUB_SET_CUST;
		switch (action) {
		case CUST_ACTION_GET_RAW_DATA:
			req.set_cust_req.getRawData.action = CUST_ACTION_GET_RAW_DATA;
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.getRawData);
			err = scp_sensorHub_req_send(&req, &len, 1);
			if (0 == err) {
				if (SENSOR_HUB_SET_CUST != req.set_cust_rsp.action
				    || 0 != req.set_cust_rsp.errCode) {
					SCP_PR_ERR("scp_sensorHub_req_send failed!\n");
					return -1;
				}
				if (CUST_ACTION_GET_RAW_DATA != req.set_cust_rsp.getRawData.action) {
					SCP_PR_ERR("scp_sensorHub_req_send failed!\n");
					return -1;
				}
				*((uint8_t *) data) = req.set_cust_rsp.getRawData.uint8_data[0];
			} else {
				SCP_PR_ERR("scp_sensorHub_req_send failed!\n");
			}
			return 0;
		case CUST_ACTION_SHOW_ALSLV:
			req.set_cust_req.showAlslv.action = CUST_ACTION_SHOW_ALSLV;
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.showAlslv);
			break;
		case CUST_ACTION_SHOW_ALSVAL:
			req.set_cust_req.showAlsval.action = CUST_ACTION_GET_RAW_DATA;
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.showAlsval);
			break;
		default:
			return -1;
		}
		break;
	case ID_PROXIMITY:
		req.set_cust_req.sensorType = ID_PROXIMITY;
		req.set_cust_req.action = SENSOR_HUB_SET_CUST;
		switch (action) {
		case CUST_ACTION_RESET_CALI:
			req.set_cust_req.resetCali.action = CUST_ACTION_RESET_CALI;
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.resetCali);
			break;
		case CUST_ACTION_SET_CALI:
			req.set_cust_req.setCali.action = CUST_ACTION_SET_CALI;
			req.set_cust_req.setCali.int32_data[0] = *((int32_t *) data);
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.setCali);
			break;
		case CUST_ACTION_SET_TRACE:
			req.set_cust_req.setTrace.action = CUST_ACTION_SET_TRACE;
			req.set_cust_req.setTrace.trace = *((int32_t *) data);
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.setTrace);
			break;
		case CUST_ACTION_SHOW_REG:
			req.set_cust_req.showReg.action = CUST_ACTION_SHOW_REG;
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.showReg);
			break;
		case CUST_ACTION_SET_PS_THRESHOLD:
			req.set_cust_req.setPSThreshold.action = CUST_ACTION_SET_PS_THRESHOLD;
			req.set_cust_req.setPSThreshold.threshold[0]
			    = *((int32_t *) data + 0);
			req.set_cust_req.setPSThreshold.threshold[1]
			    = *((int32_t *) data + 1);
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.setPSThreshold);
			break;
		case CUST_ACTION_GET_RAW_DATA:
			req.set_cust_req.getRawData.action = CUST_ACTION_GET_RAW_DATA;
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.getRawData);
			err = scp_sensorHub_req_send(&req, &len, 1);
			if (0 == err) {
				if (SENSOR_HUB_SET_CUST != req.set_cust_rsp.action
				    || 0 != req.set_cust_rsp.errCode) {
					SCP_PR_ERR("scp_sensorHub_req_send failed!\n");
					return -1;
				}
				if (CUST_ACTION_GET_RAW_DATA != req.set_cust_rsp.getRawData.action) {
					SCP_PR_ERR("scp_sensorHub_req_send failed!\n");
					return -1;
				}
				*((uint16_t *) data) = req.set_cust_rsp.getRawData.uint16_data[0];
			} else {
				SCP_PR_ERR("scp_sensorHub_req_send failed!\n");
			}
			return 0;
		default:
			return -1;
		}
		break;
	case ID_PRESSURE:
		req.set_cust_req.sensorType = ID_PRESSURE;
		req.set_cust_req.action = SENSOR_HUB_SET_CUST;
		switch (action) {
		case CUST_ACTION_SET_TRACE:
			req.set_cust_req.setTrace.action = CUST_ACTION_SET_TRACE;
			req.set_cust_req.setTrace.trace = *((int32_t *) data);
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.setTrace);
			break;
		case CUST_ACTION_SHOW_REG:
			req.set_cust_req.showReg.action = CUST_ACTION_SHOW_REG;
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.showReg);
			break;
		default:
			return -1;
		}
		break;
	case ID_GYROSCOPE:
		req.set_cust_req.sensorType = ID_GYROSCOPE;
		req.set_cust_req.action = SENSOR_HUB_SET_CUST;
		switch (action) {
		case CUST_ACTION_RESET_CALI:
			req.set_cust_req.resetCali.action = CUST_ACTION_RESET_CALI;
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.resetCali);
			break;
		case CUST_ACTION_SET_CALI:
			req.set_cust_req.setCali.action = CUST_ACTION_SET_CALI;
			req.set_cust_req.setCali.int32_data[SCP_SENSOR_HUB_X]
			    = *((int32_t *) data + SCP_SENSOR_HUB_X);
			req.set_cust_req.setCali.int32_data[SCP_SENSOR_HUB_Y]
			    = *((int32_t *) data + SCP_SENSOR_HUB_Y);
			req.set_cust_req.setCali.int32_data[SCP_SENSOR_HUB_Z]
			    = *((int32_t *) data + SCP_SENSOR_HUB_Z);
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.setCali);
			break;
		case CUST_ACTION_SET_TRACE:
			req.set_cust_req.setTrace.action = CUST_ACTION_SET_TRACE;
			req.set_cust_req.setTrace.trace = *((int32_t *) data);
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.setTrace);
			break;
		case CUST_ACTION_SET_DIRECTION:
			req.set_cust_req.setDirection.action = CUST_ACTION_SET_DIRECTION;
			req.set_cust_req.setDirection.direction = *((int32_t *) data);
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.setDirection);
			break;
		case CUST_ACTION_SET_FACTORY:
			req.set_cust_req.setFactory.action = CUST_ACTION_SET_FACTORY;
			req.set_cust_req.setFactory.factory = *((int32_t *) data);
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.setFactory);
			break;
		case CUST_ACTION_SHOW_REG:
			req.set_cust_req.showReg.action = CUST_ACTION_SHOW_REG;
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.showReg);
			break;
		default:
			return -1;
		}
		break;
	case ID_RELATIVE_HUMIDITY:
		req.set_cust_req.sensorType = ID_MAGNETIC;
		req.set_cust_req.action = SENSOR_HUB_SET_CUST;
		switch (action) {
		case CUST_ACTION_SET_TRACE:
			req.set_cust_req.setTrace.action = CUST_ACTION_SET_TRACE;
			req.set_cust_req.setTrace.trace = *((int32_t *) data);
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.setTrace);
			break;
		case CUST_ACTION_SHOW_REG:
			req.set_cust_req.showReg.action = CUST_ACTION_SHOW_REG;
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.showReg);
			break;
		default:
			return -1;
		}
		break;
	case ID_MAGNETIC:
		req.set_cust_req.sensorType = ID_MAGNETIC;
		req.set_cust_req.action = SENSOR_HUB_SET_CUST;
		switch (action) {
		case CUST_ACTION_SET_TRACE:
			req.set_cust_req.setTrace.action = CUST_ACTION_SET_TRACE;
			req.set_cust_req.setTrace.trace = *((int32_t *) data);
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.setTrace);
			break;
		case CUST_ACTION_SET_DIRECTION:
			req.set_cust_req.setDirection.action = CUST_ACTION_SET_DIRECTION;
			req.set_cust_req.setDirection.direction = *((int32_t *) data);
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.setDirection);
			break;
		case CUST_ACTION_SHOW_REG:
			req.set_cust_req.showReg.action = CUST_ACTION_SHOW_REG;
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
			    + sizeof(req.set_cust_req.showReg);
			break;
		default:
			return -1;
		}
		break;
	default:
		req.set_cust_req.sensorType = sensorType;
		req.set_cust_req.action = SENSOR_HUB_SET_CUST;
		switch (action) {
		case CUST_ACTION_SET_TRACE:
			req.set_cust_req.setTrace.action = CUST_ACTION_SET_TRACE;
			req.set_cust_req.setTrace.trace = *((int32_t *) data);
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData)
				+ sizeof(req.set_cust_req.setTrace);
			break;
		default:
			return -1;
		}
	}
	err = scp_sensorHub_req_send(&req, &len, 1);
	if (err < 0) {
		SCP_PR_ERR("scp_sensorHub_req_send fail!\n");
		return -1;
	}
	if (sensorType != req.get_data_rsp.sensorType ||
	    SENSOR_HUB_SET_CUST != req.get_data_rsp.action || 0 != req.get_data_rsp.errCode) {
		SCP_PR_ERR("error : %d\n", req.get_data_rsp.errCode);
		return req.get_data_rsp.errCode;
	}

	return err;
}

static void sensorHub_power_up_work(struct work_struct *work)
{
	int ret = 0;
	int handle = 0, flush_cnt = 0;
	struct ConfigCmd cmd;
	struct SCP_sensorHub_data *obj = obj_data;

	/* firstly we should update dram information */
	/* 1. reset wp queue head and tail */
	obj->wp_queue.head = 0;
	obj->wp_queue.tail = 0;
	/* 2. init dram information */
	obj->SCP_sensorFIFO = (struct sensorFIFO *)scp_get_reserve_mem_virt(SENS_MEM_ID);
	WARN_ON(obj->SCP_sensorFIFO == NULL);
	obj->SCP_sensorFIFO->wp = 0;
	obj->SCP_sensorFIFO->rp = 0;
	obj->SCP_sensorFIFO->FIFOSize =
		(SCP_SENSOR_HUB_FIFO_SIZE - offsetof(struct sensorFIFO, data)) / SENSOR_DATA_SIZE * SENSOR_DATA_SIZE;
	SCP_PR_ERR("obj->SCP_sensorFIFO = %p, wp = %d, rp = %d, size = %d\n", obj->SCP_sensorFIFO,
		obj->SCP_sensorFIFO->wp, obj->SCP_sensorFIFO->rp, obj->SCP_sensorFIFO->FIFOSize);
#ifndef CHRE_POWER_RESET_NOTIFY
	/* 3. wait for chre init done when don't support power reset feature */
	msleep(2000);
#endif
	/* 4. send dram information to scp */
	sensor_send_dram_info_to_hub();
	/* secondly we enable sensor which sensor is enable by framework */
	mutex_lock(&mSensorState_mtx);
	for (handle = 0; handle < ID_SENSOR_MAX_HANDLE + 1; handle++) {
		if ((mSensorState[handle].sensorType || (handle == ID_ACCELEROMETER &&
				mSensorState[handle].sensorType == ID_ACCELEROMETER)) && mSensorState[handle].enable) {
			init_sensor_config_cmd(&cmd, handle);
			SCP_PR_ERR("restoring: handle=%d, enable=%d, rate=%d, latency=%lld\n", handle,
				mSensorState[handle].enable, mSensorState[handle].rate, mSensorState[handle].latency);
			ret = nanohub_external_write((const uint8_t *)&cmd, sizeof(struct ConfigCmd));
			if (ret < 0)
				SCP_PR_ERR("failed registerlistener handle:%d, cmd:%d\n", handle, cmd.cmd);

			cmd.cmd = CONFIG_CMD_FLUSH;
			for (flush_cnt = 0; flush_cnt < atomic_read(&mSensorState[handle].flushCnt); flush_cnt++) {
				ret = nanohub_external_write((const uint8_t *)&cmd, sizeof(struct ConfigCmd));
				if (ret < 0)
					SCP_PR_ERR("failed flush handle:%d\n", handle);
			}
		}
	}
	mutex_unlock(&mSensorState_mtx);
}
static int sensorHub_ready_event(struct notifier_block *this, unsigned long event, void *ptr)
{
#ifndef CHRE_POWER_RESET_NOTIFY
	struct SCP_sensorHub_data *obj = obj_data;
#endif

	if (event == SCP_EVENT_STOP) {
		atomic_set(&power_status, SENSOR_POWER_DOWN);
		scp_power_monitor_notify(SENSOR_POWER_DOWN, ptr);
	}
#ifndef CHRE_POWER_RESET_NOTIFY
	if (event == SCP_EVENT_READY) {
		atomic_set(&power_status, SENSOR_POWER_UP);
		scp_power_monitor_notify(SENSOR_POWER_UP, ptr);
		schedule_work(&obj->power_up_work);
	}
#endif
	return NOTIFY_DONE;
}

static struct notifier_block sensorHub_ready_notifier = {
	.notifier_call = sensorHub_ready_event,
};
static int sensorHub_probe(struct platform_device *pdev)
{
	struct SCP_sensorHub_data *obj;
	int err = 0, index;
	struct task_struct *task = NULL;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };

	SCP_FUN();
	SCP_sensorHub_init_sensor_state();
	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		SCP_PR_ERR("Allocate SCP_sensorHub_data fail\n");
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(struct SCP_sensorHub_data));
	obj_data = obj;

	/* init sensor share dram write pointer event queue */
	spin_lock_init(&obj->wp_queue.buffer_lock);
	obj->wp_queue.head = 0;
	obj->wp_queue.tail = 0;
	obj->wp_queue.bufsize = 32;
	obj->wp_queue.ringbuffer =
		vzalloc(obj->wp_queue.bufsize * sizeof(uint32_t));
	if (!obj->wp_queue.ringbuffer) {
		SCP_PR_ERR("Alloc ringbuffer error!\n");
		goto exit;
	}
	/* register ipi interrupt handler */
	scp_ipi_registration(IPI_SENSOR, SCP_sensorHub_IPI_handler, "SCP_sensorHub");
	/* init receive scp dram data worker */
	/* INIT_WORK(&obj->direct_push_work, SCP_sensorHub_direct_push_work); */
	/* obj->direct_push_workqueue = alloc_workqueue("chre_work", WQ_MEM_RECLAIM |
	*			WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);
	*/
#if 0
	obj->direct_push_workqueue = create_singlethread_workqueue("chre_work");
	if (obj->direct_push_workqueue == NULL) {
		SCP_PR_ERR("direct_push_workqueue fail\n");
		return -1;
	}
#endif
	WRITE_ONCE(chre_kthread_wait_condition, false);
	task = kthread_run(SCP_sensorHub_direct_push_work, NULL, "chre_kthread");
	if (IS_ERR(task)) {
		SCP_PR_ERR("SCP_sensorHub_direct_push_work create fail!\n");
		goto exit;
	}
	sched_setscheduler(task, SCHED_FIFO, &param);
	/* init the debug trace flag */
	for (index = 0; index < ID_SENSOR_MAX_HANDLE; index++)
		atomic_set(&obj->traces[index], 0);
	/* init timestamp sync worker */
	INIT_WORK(&obj->sync_time_worker, SCP_sensorHub_sync_time_work);
	obj->sync_time_timer.expires = jiffies + 3 * HZ;
	obj->sync_time_timer.function = SCP_sensorHub_sync_time_func;
	init_timer(&obj->sync_time_timer);
	mod_timer(&obj->sync_time_timer, jiffies + 3 * HZ);
	wake_lock_init(&obj->sync_time_wake_lock, WAKE_LOCK_SUSPEND, "sync_time");
	/* this call back can get scp power down status */
	scp_A_register_notify(&sensorHub_ready_notifier);
	/* this call back can get scp power UP status */
	INIT_WORK(&obj->power_up_work, sensorHub_power_up_work);

	SCP_sensorHub_init_flag = 0;
	SCP_PR_ERR("init done, data_unit_t size: %d, SCP_SENSOR_HUB_DATA size:%d\n",
		(int)sizeof(struct data_unit_t), (int)sizeof(SCP_SENSOR_HUB_DATA));
	BUG_ON(sizeof(struct data_unit_t) != SENSOR_DATA_SIZE || sizeof(SCP_SENSOR_HUB_DATA) != SENSOR_DATA_SIZE);
	return 0;
exit:
	SCP_PR_ERR("%s: err = %d\n", __func__, err);
	SCP_sensorHub_init_flag = -1;
	return err;
}

static int sensorHub_remove(struct platform_device *pdev)
{
	return 0;
}

static int sensorHub_suspend(struct platform_device *pdev, pm_message_t msg)
{
	/* sensor_send_timestamp_to_hub(); */
	return 0;
}

static int sensorHub_resume(struct platform_device *pdev)
{
	/* sensor_send_timestamp_to_hub(); */
	return 0;
}

static ssize_t nanohub_show_trace(struct device_driver *ddri, char *buf)
{
	struct SCP_sensorHub_data *obj = obj_data;
	int i;
	ssize_t res = 0;

	for (i = 0; i < ID_SENSOR_MAX_HANDLE; i++)
		res += snprintf(&buf[res], PAGE_SIZE, "%2d:[%d]\n", i, atomic_read(&obj->traces[i]));
	return res;
}

static ssize_t nanohub_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
	struct SCP_sensorHub_data *obj = obj_data;
	int handle, trace = 0;
	int res = 0;

	SCP_PR_ERR("nanohub_store_trace buf:%s\n", buf);
	if (sscanf(buf, "%d,%d", &handle, &trace) != 2) {
		SCP_PR_ERR("invalid content: '%s', length = %zu\n", buf, count);
		goto err_out;
	}

	if (handle < 0 || handle >= ID_SENSOR_MAX_HANDLE) {
		SCP_PR_ERR("invalid handle value:%d, that should be '0<=handle<%d'\n", trace, ID_SENSOR_MAX_HANDLE);
		goto err_out;
	}

	if (trace != 0 && trace != 1) {
		SCP_PR_ERR("invalid trace value:%d, the trace value should be '0' or '1'", trace);
		goto err_out;
	}

	res = sensor_set_cmd_to_hub(handle, CUST_ACTION_SET_TRACE, &trace);
	if (res < 0)
		SCP_PR_ERR("sensor_set_cmd_to_hub fail, (ID: %d),(action: %d)err:%d\n", handle,
			CUST_ACTION_SET_TRACE, res);
	else
		atomic_set(&obj->traces[handle], trace);

err_out:
	return count;
}

static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, nanohub_show_trace, nanohub_store_trace);

static struct driver_attribute *nanohub_attr_list[] = {
	&driver_attr_trace,	/*trace log */
};

static int nanohub_create_attr(struct device_driver *driver)
{
	int idx = 0, err = 0;
	int num = (int)(ARRAY_SIZE(nanohub_attr_list));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, nanohub_attr_list[idx]);
		if (err) {
			SCP_PR_ERR("driver_create_file (%s) = %d\n", nanohub_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

static int nanohub_delete_attr(struct device_driver *driver)
{
	int idx = 0, err = 0;
	int num = (int)(ARRAY_SIZE(nanohub_attr_list));

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, nanohub_attr_list[idx]);

	return err;
}

static struct platform_device sensorHub_device = {
	.name = "sensor_hub_pl",
	.id = -1,
};

static struct platform_driver sensorHub_driver = {
	.driver = {
	   .name = "sensor_hub_pl",
	},
	.probe = sensorHub_probe,
	.remove = sensorHub_remove,
	.suspend = sensorHub_suspend,
	.resume = sensorHub_resume,
};

#ifdef CONFIG_PM
static int sensorHub_pm_event(struct notifier_block *notifier, unsigned long pm_event,
			void *unused)
{
	switch (pm_event) {
	case PM_POST_SUSPEND:
		SCP_PR_ERR("resume\n");
		WRITE_ONCE(rtc_compensation_suspend, false);
		sensor_send_timestamp_to_hub();
		return NOTIFY_DONE;
	case PM_SUSPEND_PREPARE:
		SCP_PR_ERR("suspend\n");
		WRITE_ONCE(rtc_compensation_suspend, true);
		return NOTIFY_DONE;
	default:
		return NOTIFY_OK;
	}
	return NOTIFY_OK;
}

static struct notifier_block sensorHub_pm_notifier_func = {
	.notifier_call = sensorHub_pm_event,
	.priority = 0,
};
#endif /* CONFIG_PM */

static int __init SCP_sensorHub_init(void)
{
	SCP_sensorHub_ipi_master_init();
	SCP_FUN();
	if (platform_device_register(&sensorHub_device)) {
		SCP_PR_ERR("SCP_sensorHub platform device error\n");
		return -1;
	}
	if (platform_driver_register(&sensorHub_driver)) {
		SCP_PR_ERR("SCP_sensorHub platform driver error\n");
		return -1;
	}
	if (nanohub_create_attr(&sensorHub_driver.driver)) {
		SCP_PR_ERR("create attribute err\n");
		nanohub_delete_attr(&sensorHub_driver.driver);
	}
#ifdef CONFIG_PM
	if (register_pm_notifier(&sensorHub_pm_notifier_func)) {
		SCP_PR_ERR("Failed to register PM notifier.\n");
		return -1;
	}
#endif /* CONFIG_PM */
	return 0;
}

static void __exit SCP_sensorHub_exit(void)
{
	SCP_FUN();
}

module_init(SCP_sensorHub_init);
module_exit(SCP_sensorHub_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SCP sensor hub driver");
MODULE_AUTHOR("hongxu.zhao@mediatek.com");
