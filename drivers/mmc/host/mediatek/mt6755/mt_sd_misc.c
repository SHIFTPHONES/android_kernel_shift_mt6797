/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <generated/autoconf.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/blkdev.h>
#include <linux/dma-mapping.h>

#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#include "mt_sd.h"
#include <core/core.h>
#include <mt-plat/sd_misc.h>
#include "msdc_io.h"
#include "dbg.h"

#define PARTITION_NAME_LENGTH   (64)
#define DRV_NAME_MISC           "misc-sd"

#define DEBUG_MMC_IOCTL         0
#define DEBUG_MSDC_SSC          1
/*
 * For simple_sd_ioctl
 */
#define FORCE_IN_DMA            (0x11)
#define FORCE_IN_PIO            (0x10)
#define FORCE_NOTHING           (0x0)

static int dma_force[HOST_MAX_NUM] =    /* used for sd ioctrol */
{
	FORCE_NOTHING,
	FORCE_NOTHING,
	FORCE_NOTHING,
	FORCE_NOTHING,
};

#define dma_is_forced(host_id)                  (dma_force[host_id] & 0x10)
#define get_forced_transfer_mode(host_id)       (dma_force[host_id] & 0x01)

static u32 *sg_msdc_multi_buffer;

static int simple_sd_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int sd_ioctl_reinit(struct msdc_ioctl *msdc_ctl)
{
	struct msdc_host *host = msdc_get_host(MSDC_SD,	0, 0);
	if (NULL != host)
		return msdc_reinit(host);
	else
		return -EINVAL;
}

static int sd_ioctl_cd_pin_en(struct msdc_ioctl	*msdc_ctl)
{
	struct msdc_host *host = msdc_get_host(MSDC_SD,	0, 0);

	if (NULL != host)
		return (host->mmc->caps & MMC_CAP_NONREMOVABLE)
			== MMC_CAP_NONREMOVABLE;
	else
		return -EINVAL;
}

int simple_sd_ioctl_rw(struct msdc_ioctl *msdc_ctl)
{
	struct scatterlist msdc_sg;
	struct mmc_data msdc_data;
	struct mmc_command msdc_cmd;
	struct mmc_command msdc_stop;
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	int is_cmdq_en;
#endif
	int ret = 0;
#ifdef CONFIG_MTK_EMMC_SUPPORT
	char part_id;
#endif
	int no_single_rw;
	u32 total_size;

#ifdef MTK_MSDC_USE_CMD23
	struct mmc_command msdc_sbc;
#endif

	struct mmc_request  msdc_mrq;
	struct msdc_host *host_ctl;
	struct mmc_host *mmc;

	if (!msdc_ctl)
		return -EINVAL;

	host_ctl = mtk_msdc_host[msdc_ctl->host_num];
	if (!host_ctl || !host_ctl->mmc || !host_ctl->mmc->card)
		return -EINVAL;

	mmc = host_ctl->mmc;

	if ((msdc_ctl->total_size <= 0) ||
	    (msdc_ctl->total_size > host_ctl->mmc->max_seg_size))
		return -EINVAL;
	total_size = msdc_ctl->total_size;

	if (msdc_ctl->total_size > 512)
		no_single_rw = 1;
	else
		no_single_rw = 0;

#ifdef MTK_MSDC_USE_CACHE
	if (msdc_ctl->iswrite && mmc_card_mmc(mmc->card)
	 && (mmc->card->ext_csd.cache_ctrl & 0x1))
		no_single_rw = 1;
#endif

	mmc_claim_host(mmc);

#if DEBUG_MMC_IOCTL
	pr_debug("user want access %d partition\n", msdc_ctl->partition);
#endif

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	is_cmdq_en = false;
	if (mmc->card->ext_csd.cmdq_mode_en) {
		/* cmdq enabled, turn it off first */
		pr_debug("[MSDC_DBG] cmdq enabled, turn it off\n");
		ret = mmc_blk_cmdq_switch(mmc->card, 0);
		if (ret) {
			pr_debug("[MSDC_DBG] turn off cmdq en failed\n");
			goto rw_end;
		} else
			is_cmdq_en = true;
	}
#endif

#ifdef CONFIG_MTK_EMMC_SUPPORT
	switch (msdc_ctl->partition) {
	case EMMC_PART_BOOT1:
		part_id = 1;
		break;
	case EMMC_PART_BOOT2:
		part_id = 2;
		break;
	default:
		/* make sure access partition is user data area */
		part_id = 0;
		break;
	}

	if (msdc_switch_part(host_ctl, part_id))
		goto rw_end;

#endif

	memset(&msdc_data, 0, sizeof(struct mmc_data));
	memset(&msdc_mrq, 0, sizeof(struct mmc_request));
	memset(&msdc_cmd, 0, sizeof(struct mmc_command));

	if (no_single_rw) {
		memset(&msdc_stop, 0, sizeof(struct mmc_command));

#ifdef MTK_MSDC_USE_CMD23
		memset(&msdc_sbc, 0, sizeof(struct mmc_command));
#endif
	}

	msdc_mrq.cmd = &msdc_cmd;
	msdc_mrq.data = &msdc_data;

	if (msdc_ctl->trans_type)
		dma_force[host_ctl->id] = FORCE_IN_DMA;
	else
		dma_force[host_ctl->id] = FORCE_IN_PIO;

	if (msdc_ctl->iswrite) {
		msdc_data.flags = MMC_DATA_WRITE;
		if (no_single_rw)
			msdc_cmd.opcode = MMC_WRITE_MULTIPLE_BLOCK;
		else
			msdc_cmd.opcode = MMC_WRITE_BLOCK;
		msdc_data.blocks = total_size / 512;
		if (MSDC_CARD_DUNM_FUNC != msdc_ctl->opcode) {
			if (copy_from_user(sg_msdc_multi_buffer,
				msdc_ctl->buffer, total_size)) {
				dma_force[host_ctl->id] = FORCE_NOTHING;
				ret = -EFAULT;
				goto rw_end;
			}
		} else {
			/* called from other kernel module */
			memcpy(sg_msdc_multi_buffer, msdc_ctl->buffer,
				total_size);
		}
	} else {
		msdc_data.flags = MMC_DATA_READ;
		if (no_single_rw)
			msdc_cmd.opcode = MMC_READ_MULTIPLE_BLOCK;
		else
			msdc_cmd.opcode = MMC_READ_SINGLE_BLOCK;
		msdc_data.blocks = total_size / 512;
		memset(sg_msdc_multi_buffer, 0, total_size);
	}

#ifdef MTK_MSDC_USE_CMD23
	if (no_single_rw == 0)
		goto skip_sbc_prepare;

	if ((mmc_card_mmc(mmc->card) || (mmc_card_sd(mmc->card)
	 && mmc->card->scr.cmds & SD_SCR_CMD23_SUPPORT))
	 && !(mmc->card->quirks & MMC_QUIRK_BLK_NO_CMD23)) {
		msdc_mrq.sbc = &msdc_sbc;
		msdc_mrq.sbc->opcode = MMC_SET_BLOCK_COUNT;
#ifdef MTK_MSDC_USE_CACHE
		/* if ioctl access cacheable partition data,
		   there is on flush mechanism in msdc driver
		 * so do reliable write .*/
		if (mmc_card_mmc(mmc->card)
		 && (mmc->card->ext_csd.cache_ctrl & 0x1)
		 && (msdc_cmd.opcode == MMC_WRITE_MULTIPLE_BLOCK))
			msdc_mrq.sbc->arg = msdc_data.blocks | (1 << 31);
		else
			msdc_mrq.sbc->arg = msdc_data.blocks;
#else
		msdc_mrq.sbc->arg = msdc_data.blocks;
#endif
		msdc_mrq.sbc->flags = MMC_RSP_R1 | MMC_CMD_AC;
	}
skip_sbc_prepare:
#endif

	msdc_cmd.arg = msdc_ctl->address;

	if (!mmc_card_blockaddr(mmc->card)) {
		pr_debug("this device use byte address!!\n");
		msdc_cmd.arg <<= 9;
	}
	msdc_cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	if (no_single_rw) {
		msdc_stop.opcode = MMC_STOP_TRANSMISSION;
		msdc_stop.arg = 0;
		msdc_stop.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;

		msdc_data.stop = &msdc_stop;
	} else {
		msdc_data.stop = NULL;
	}
	msdc_data.blksz = 512;
	msdc_data.sg = &msdc_sg;
	msdc_data.sg_len = 1;

#if DEBUG_MMC_IOCTL
	pr_debug("total size is %d\n", total_size);
	pr_debug("ueser buf address is 0x%p!\n", msdc_ctl->buffer);
#endif
	sg_init_one(&msdc_sg, sg_msdc_multi_buffer, total_size);
	mmc_set_data_timeout(&msdc_data, mmc->card);
	mmc_wait_for_req(mmc, &msdc_mrq);

	if (msdc_ctl->partition)
		msdc_switch_part(host_ctl, 0);

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	if (is_cmdq_en) {
		pr_debug("[MSDC_DBG] turn on cmdq\n");
		ret = mmc_blk_cmdq_switch(host_ctl->mmc->card, 1);
		if (ret)
			pr_debug("[MSDC_DBG] turn on cmdq en failed\n");
		else
			is_cmdq_en = false;
	}
#endif
	mmc_release_host(mmc);
	if (!msdc_ctl->iswrite) {
		if (MSDC_CARD_DUNM_FUNC != msdc_ctl->opcode) {
			if (copy_to_user(msdc_ctl->buffer, sg_msdc_multi_buffer,
				total_size)) {
				dma_force[host_ctl->id] = FORCE_NOTHING;
				ret = -EFAULT;
				goto rw_end_without_release;
			}
		} else {
			/* called from other kernel module */
			memcpy(msdc_ctl->buffer, sg_msdc_multi_buffer,
				total_size);
		}
	}

	/* clear the global buffer of R/W IOCTL */
	memset(sg_msdc_multi_buffer, 0 , total_size);
	goto rw_end_without_release;

rw_end:
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	if (is_cmdq_en) {
		pr_debug("[MSDC_DBG] turn on cmdq\n");
		ret = mmc_blk_cmdq_switch(mmc->card, 1);
		if (ret)
			pr_debug("[MSDC_DBG] turn on cmdq en failed\n");
		else
			is_cmdq_en = false;
	}
#endif
	mmc_release_host(mmc);

rw_end_without_release:
	if (ret)
		msdc_ctl->result = ret;

	if (msdc_cmd.error)
		msdc_ctl->result = msdc_cmd.error;

	if (msdc_data.error)
		msdc_ctl->result = msdc_data.error;
	else
		msdc_ctl->result = 0;

	dma_force[host_ctl->id] = FORCE_NOTHING;
	return msdc_ctl->result;

}

static int simple_sd_ioctl_get_cid(struct msdc_ioctl *msdc_ctl)
{
	struct msdc_host *host_ctl;

	if (!msdc_ctl)
		return -EINVAL;

	host_ctl = mtk_msdc_host[msdc_ctl->host_num];

	if (!host_ctl || !host_ctl->mmc || !host_ctl->mmc->card)
		return -EINVAL;

#if DEBUG_MMC_IOCTL
	pr_debug("user want the cid in msdc slot%d\n", msdc_ctl->host_num);
#endif

	if (copy_to_user(msdc_ctl->buffer, &host_ctl->mmc->card->raw_cid, 16))
		return -EFAULT;

#if DEBUG_MMC_IOCTL
	pr_debug("cid:0x%x,0x%x,0x%x,0x%x\n",
		host_ctl->mmc->card->raw_cid[0],
		host_ctl->mmc->card->raw_cid[1],
		host_ctl->mmc->card->raw_cid[2],
		host_ctl->mmc->card->raw_cid[3]);
#endif
	return 0;

}

static int simple_sd_ioctl_get_csd(struct msdc_ioctl *msdc_ctl)
{
	struct msdc_host *host_ctl;

	if (!msdc_ctl)
		return -EINVAL;

	host_ctl = mtk_msdc_host[msdc_ctl->host_num];

	if (!host_ctl || !host_ctl->mmc || !host_ctl->mmc->card)
		return -EINVAL;

#if DEBUG_MMC_IOCTL
	pr_debug("user want the csd in msdc slot%d\n", msdc_ctl->host_num);
#endif

	if (copy_to_user(msdc_ctl->buffer, &host_ctl->mmc->card->raw_csd, 16))
		return -EFAULT;

#if DEBUG_MMC_IOCTL
	pr_debug("csd:0x%x,0x%x,0x%x,0x%x\n",
		 host_ctl->mmc->card->raw_csd[0],
		 host_ctl->mmc->card->raw_csd[1],
		 host_ctl->mmc->card->raw_csd[2],
		 host_ctl->mmc->card->raw_csd[3]);
#endif
	return 0;

}

static int simple_sd_ioctl_get_excsd(struct msdc_ioctl *msdc_ctl)
{
	char *l_buf;
	struct msdc_host *host_ctl;
	struct mmc_host *mmc;
	int ret;

#if DEBUG_MMC_IOCTL
	int i;
#endif

	if (!msdc_ctl)
		return -EINVAL;

	host_ctl = mtk_msdc_host[msdc_ctl->host_num];

	if (!host_ctl || !host_ctl->mmc || !host_ctl->mmc->card)
		return -EINVAL;

	if (host_ctl->hw->host_function != MSDC_EMMC)
		return -EINVAL;

	mmc = host_ctl->mmc;

	l_buf = kzalloc((512), GFP_KERNEL);
	if (!l_buf)
		return -ENOMEM;

	mmc_claim_host(mmc);

#if DEBUG_MMC_IOCTL
	pr_debug("user want the extend csd in msdc slot%d\n",
		msdc_ctl->host_num);
#endif
	ret = mmc_send_ext_csd(mmc->card, l_buf);
	mmc_release_host(mmc);
	if (ret)
		goto end;

	if (copy_to_user(msdc_ctl->buffer, l_buf, 512))
		return -EFAULT;

#if DEBUG_MMC_IOCTL
	for (i = 0; i < 512; i++) {
		pr_debug("%x", l_buf[i]);
		if (0 == ((i + 1) % 16))
			pr_debug("\n");
	}
#endif

end:
	kfree(l_buf);

	return 0;

}

static int simple_sd_ioctl_get_bootpart(struct msdc_ioctl *msdc_ctl)
{
	char *l_buf;
	struct msdc_host *host_ctl;
	struct mmc_host *mmc;
	int ret = 0;
	int bootpart = 0;
	unsigned int user_buffer;

	host_ctl = mtk_msdc_host[msdc_ctl->host_num];
	if (!host_ctl || !host_ctl->mmc || !host_ctl->mmc->card)
		return -EINVAL;

	if (host_ctl->hw->host_function != MSDC_EMMC)
		return -EINVAL;

	mmc = host_ctl->mmc;

	if (get_user(user_buffer, msdc_ctl->buffer))
		return -EINVAL;

	l_buf = kzalloc((512), GFP_KERNEL);
	if (!l_buf)
		return -ENOMEM;

	mmc_claim_host(mmc);

#if DEBUG_MMC_IOCTL
	pr_debug("user want get boot partition info in msdc slot%d\n",
		msdc_ctl->host_num);
#endif
	ret = mmc_send_ext_csd(mmc->card, l_buf);
	if (ret) {
		pr_debug("mmc_send_ext_csd error, get boot part\n");
		goto end;
	}
	bootpart = (l_buf[EXT_CSD_PART_CFG] & 0x38) >> 3;

#if DEBUG_MMC_IOCTL
	pr_debug("bootpart Byte[EXT_CSD_PART_CFG] =%x, booten=%x\n",
		l_buf[EXT_CSD_PART_CFG], bootpart);
#endif

	if (MSDC_CARD_DUNM_FUNC != msdc_ctl->opcode) {
		if (copy_to_user(msdc_ctl->buffer, &bootpart, 1)) {
			ret = -EFAULT;
			goto end;
		}
	} else {
		/* called from other kernel module */
		memcpy(msdc_ctl->buffer, &bootpart, 1);
	}

end:
	msdc_ctl->result = ret;

	mmc_release_host(mmc);

	kfree(l_buf);

	return ret;
}

static int simple_sd_ioctl_set_bootpart(struct msdc_ioctl *msdc_ctl)
{
	char *l_buf;
	struct msdc_host *host_ctl;
	struct mmc_host *mmc;
	int ret = 0;
	int bootpart = 0;

	host_ctl = mtk_msdc_host[msdc_ctl->host_num];

	if (!host_ctl || !host_ctl->mmc || !host_ctl->mmc->card)
		return -EINVAL;

	if (host_ctl->hw->host_function != MSDC_EMMC)
		return -EINVAL;

	mmc = host_ctl->mmc;

	if (msdc_ctl->buffer == NULL)
		return -EINVAL;

	l_buf = kzalloc((512), GFP_KERNEL);
	if (!l_buf)
		return -ENOMEM;

	mmc_claim_host(mmc);

#if DEBUG_MMC_IOCTL
	pr_debug("user want set boot partition in msdc slot%d\n",
		msdc_ctl->host_num);
#endif
	ret = mmc_send_ext_csd(mmc->card, l_buf);
	if (ret) {
		pr_debug("mmc_send_ext_csd error, set boot partition\n");
		goto end;
	}

	if (copy_from_user(&bootpart, msdc_ctl->buffer, 1)) {
		ret = -EFAULT;
		goto end;
	}

	if ((bootpart != EMMC_BOOT1_EN)
	 && (bootpart != EMMC_BOOT2_EN)
	 && (bootpart != EMMC_BOOT_USER)) {
		pr_debug("set boot partition error, not support %d\n",
			bootpart);
		ret = -EFAULT;
		goto end;
	}

	if (((l_buf[EXT_CSD_PART_CFG] & 0x38) >> 3) != bootpart) {
		/* active boot partition */
		l_buf[EXT_CSD_PART_CFG] &= ~0x38;
		l_buf[EXT_CSD_PART_CFG] |= (bootpart << 3);
		pr_debug("mmc_switch set %x\n", l_buf[EXT_CSD_PART_CFG]);
		ret = mmc_switch(mmc->card, 0, EXT_CSD_PART_CFG,
			l_buf[EXT_CSD_PART_CFG], 1000);
		if (ret) {
			pr_debug("mmc_switch error, set boot partition\n");
		} else {
			mmc->card->ext_csd.part_config =
				l_buf[EXT_CSD_PART_CFG];
		}
	}

end:
	msdc_ctl->result = ret;

	mmc_release_host(mmc);

	kfree(l_buf);
	return ret;
}

static int simple_sd_ioctl_get_partition_size(struct msdc_ioctl *msdc_ctl)
{
	struct msdc_host *host_ctl;
	unsigned long long partitionsize = 0;
	struct mmc_host *mmc;
	int ret = 0;

	host_ctl = mtk_msdc_host[msdc_ctl->host_num];

	if (!host_ctl || !host_ctl->mmc || !host_ctl->mmc->card)
		return -EINVAL;

	if (host_ctl->hw->host_function != MSDC_EMMC)
		return -EINVAL;

	mmc = host_ctl->mmc;

	mmc_claim_host(mmc);

#if DEBUG_MMC_IOCTL
	pr_debug("user want get size of partition=%d\n", msdc_ctl->partition);
#endif

	switch (msdc_ctl->partition) {
	case EMMC_PART_BOOT1:
		partitionsize = msdc_get_other_capacity(host_ctl, "boot0");
		break;
	case EMMC_PART_BOOT2:
		partitionsize = msdc_get_other_capacity(host_ctl, "boot1");
		break;
	case EMMC_PART_RPMB:
		partitionsize = msdc_get_other_capacity(host_ctl, "rpmb");
		break;
	case EMMC_PART_USER:
		partitionsize = msdc_get_user_capacity(host_ctl);
		break;
	default:
		pr_debug("not support partition =%d\n", msdc_ctl->partition);
		partitionsize = 0;
		break;
	}
#if DEBUG_MMC_IOCTL
	pr_debug("bootpart partitionsize =%llx\n", partitionsize);
#endif
	if (copy_to_user(msdc_ctl->buffer, &partitionsize, 8))
		ret = -EFAULT;

	msdc_ctl->result = ret;

	mmc_release_host(mmc);
	return ret;
}

static int simple_sd_ioctl_set_driving(struct msdc_ioctl *msdc_ctl)
{
	void __iomem *base;
	struct msdc_host *host;

	host = mtk_msdc_host[msdc_ctl->host_num];
	if (host == NULL)
		return -EINVAL;

	base = host->base;

	/* msdc_clk_enable(host); */

#if DEBUG_MMC_IOCTL
	pr_debug("set: clk driving is 0x%x\n", msdc_ctl->clk_pu_driving);
	pr_debug("set: cmd driving is 0x%x\n", msdc_ctl->cmd_pu_driving);
	pr_debug("set: dat driving is 0x%x\n", msdc_ctl->dat_pu_driving);
	pr_debug("set: rst driving is 0x%x\n", msdc_ctl->rst_pu_driving);
	pr_debug("set: ds driving is 0x%x\n", msdc_ctl->ds_pu_driving);
#endif
	host->hw->clk_drv = msdc_ctl->clk_pu_driving;
	host->hw->cmd_drv = msdc_ctl->cmd_pu_driving;
	host->hw->dat_drv = msdc_ctl->dat_pu_driving;
	host->hw->rst_drv = msdc_ctl->rst_pu_driving;
	host->hw->ds_drv = msdc_ctl->ds_pu_driving;
	host->hw->clk_drv_sd_18 = msdc_ctl->clk_pu_driving;
	host->hw->cmd_drv_sd_18 = msdc_ctl->cmd_pu_driving;
	host->hw->dat_drv_sd_18 = msdc_ctl->dat_pu_driving;
	msdc_set_driving(host, host->hw, 0);
#if DEBUG_MMC_IOCTL
#if 0
	msdc_dump_padctl(host);
#endif
#endif

	return 0;
}

static int simple_sd_ioctl_get_driving(struct msdc_ioctl *msdc_ctl)
{
	void __iomem *base;
	struct msdc_host *host;

	host = mtk_msdc_host[msdc_ctl->host_num];
	if (host == NULL)
		return -EINVAL;

	base = host->base;

	/* (void)msdc_clk_enable(host); */

	msdc_get_driving(host, host->hw);

	if ((host->id == 1) && (g_msdc1_io == 1800000)) {
		msdc_ctl->clk_pu_driving = host->hw->clk_drv_sd_18;
		msdc_ctl->cmd_pu_driving = host->hw->cmd_drv_sd_18;
		msdc_ctl->dat_pu_driving = host->hw->dat_drv_sd_18;
	} else {
		msdc_ctl->clk_pu_driving = host->hw->clk_drv;
		msdc_ctl->cmd_pu_driving = host->hw->cmd_drv;
		msdc_ctl->dat_pu_driving = host->hw->dat_drv;
	}
	if (host->id == 0) {
		msdc_ctl->rst_pu_driving = host->hw->rst_drv;
		msdc_ctl->ds_pu_driving = host->hw->ds_drv;
	} else {
		msdc_ctl->rst_pu_driving = 0;
		msdc_ctl->ds_pu_driving = 0;
	}

#if DEBUG_MMC_IOCTL
	pr_debug("read: clk driving is 0x%x\n", msdc_ctl->clk_pu_driving);
	pr_debug("read: cmd driving is 0x%x\n", msdc_ctl->cmd_pu_driving);
	pr_debug("read: dat driving is 0x%x\n", msdc_ctl->dat_pu_driving);
	pr_debug("read: rst driving is 0x%x\n", msdc_ctl->rst_pu_driving);
	pr_debug("read: ds driving is 0x%x\n", msdc_ctl->ds_pu_driving);
#endif

	return 0;
}

/*  to ensure format operate is clean the emmc device fully(partition erase) */
struct mbr_part_info {
	unsigned int start_sector;
	unsigned int nr_sects;
	unsigned int part_no;
	unsigned char *part_name;
};

#define MBR_PART_NUM            6
#define __MMC_ERASE_ARG         0x00000000
#define __MMC_TRIM_ARG          0x00000001
#define __MMC_DISCARD_ARG       0x00000003

int msdc_get_info(STORAGE_TPYE storage_type, GET_STORAGE_INFO info_type,
	struct storage_info *info)
{
	struct msdc_host *host = NULL;
	int host_function = 0;
	bool boot = 0;

	if (!info)
		return -EINVAL;

	switch (storage_type) {
	case EMMC_CARD_BOOT:
		host_function = MSDC_EMMC;
		boot = MSDC_BOOT_EN;
		break;
	case EMMC_CARD:
		host_function = MSDC_EMMC;
		break;
	case SD_CARD_BOOT:
		host_function = MSDC_SD;
		boot = MSDC_BOOT_EN;
		break;
	case SD_CARD:
		host_function = MSDC_SD;
		break;
	default:
		pr_err("No supported storage type!");
		return 0;
	}

	host = msdc_get_host(host_function, boot, 0);
	if (!host || !host->mmc)
		return -EINVAL;

	switch (info_type) {
	/* FIX ME: check if any user space program use this EMMC_XXX */
	case CARD_INFO:
		if (host->mmc->card)
			info->card = host->mmc->card;
		else {
			pr_err("CARD was not ready<get card>!");
			return 0;
		}
		break;
	case DISK_INFO:
		if (host->mmc->card)
			info->disk = mmc_get_disk(host->mmc->card);
		else {
			pr_err("CARD was not ready<get disk>!");
			return 0;
		}
		break;
	case EMMC_USER_CAPACITY:
		info->emmc_user_capacity = msdc_get_capacity(0);
		break;
	case EMMC_CAPACITY:
		info->emmc_capacity = msdc_get_capacity(1);
		break;
	case EMMC_RESERVE:
#ifdef CONFIG_MTK_EMMC_SUPPORT
		info->emmc_reserve = 0;
#endif
		break;
	default:
		pr_err("Please check INFO_TYPE");
		return 0;
	}
	return 1;
}

#ifdef CONFIG_MTK_EMMC_SUPPORT
static int simple_mmc_get_disk_info(struct mbr_part_info *mpi,
	unsigned char *name)
{
	struct disk_part_iter piter;
	struct hd_struct *part;
	struct msdc_host *host;
	struct gendisk *disk;

	/* emmc always in slot0 */
	host = msdc_get_host(MSDC_EMMC, MSDC_BOOT_EN, 0);

	if (!host)
		return 1;

	disk = mmc_get_disk(host->mmc->card);

	/* Find partition info in this way to
	 * avoid addr translation in scatter file and 64bit address calculate */
	disk_part_iter_init(&piter, disk, 0);
	while ((part = disk_part_iter_next(&piter))) {
#if DEBUG_MMC_IOCTL
		pr_debug("part_name = %s name = %s\n", part->info->volname,
			name);
#endif
		if (!strncmp(part->info->volname, name,
			PARTITION_NAME_LENGTH)) {
			mpi->start_sector = part->start_sect;
			mpi->nr_sects = part->nr_sects;
			mpi->part_no = part->partno;
			mpi->part_name = part->info->volname;
			disk_part_iter_exit(&piter);
			return 0;
		}
	}
	disk_part_iter_exit(&piter);

	return 1;
}

/* call mmc block layer interface for userspace to do erase operate */
static int simple_mmc_erase_func(unsigned int start, unsigned int size)
{
	struct msdc_host *host;
	struct mmc_host *mmc;
	unsigned int arg;

	/* emmc always in slot0 */
	host = msdc_get_host(MSDC_EMMC, MSDC_BOOT_EN, 0);
	if (!host || !host->mmc || !host->mmc->card)
		return -EINVAL;

	mmc = host->mmc;

	mmc_claim_host(mmc);

	if (mmc_can_discard(mmc->card)) {
		arg = __MMC_DISCARD_ARG;
	} else if (mmc_can_trim(mmc->card)) {
		arg = __MMC_TRIM_ARG;
	} else if (mmc_can_erase(mmc->card)) {
		arg = __MMC_ERASE_ARG;
	} else {
		pr_err("[%s]: emmc card can't support trim / discard / erase\n",
			__func__);
		goto end;
	}

	pr_debug("[%s]: start=0x%x, size=%d, arg=0x%x, can_trim=(0x%x), EXT_CSD_SEC_GB_CL_EN=0x%lx\n",
		__func__, start, size, arg,
		mmc->card->ext_csd.sec_feature_support,
		EXT_CSD_SEC_GB_CL_EN);
	mmc_erase(mmc->card, start, size, arg);

#if DEBUG_MMC_IOCTL
	pr_debug("[%s]: erase done....arg=0x%x\n", __func__, arg);
#endif
end:
	mmc_release_host(mmc);

	return 0;
}
#endif

/* These definitiona and functions are coded by reference to
   mmc_blk_issue_discard_rq()@block.c*/
#define INAND_CMD38_ARG_EXT_CSD  113
#define INAND_CMD38_ARG_ERASE    0x00
#define INAND_CMD38_ARG_TRIM     0x01
#define INAND_CMD38_ARG_SECERASE 0x80
#define INAND_CMD38_ARG_SECTRIM1 0x81
#define INAND_CMD38_ARG_SECTRIM2 0x88
static int simple_sd_ioctl_erase_selected_area(struct msdc_ioctl *msdc_ctl)
{
	struct msdc_host *host_ctl;
	struct mmc_host *mmc;
	unsigned int from, nr, arg;
	int err = 0;

	host_ctl = mtk_msdc_host[msdc_ctl->host_num];
	if (!host_ctl || !host_ctl->mmc || !host_ctl->mmc->card)
		return -EINVAL;

	if (host_ctl->hw->host_function != MSDC_EMMC)
		return -EINVAL;

	mmc = host_ctl->mmc;

	mmc_claim_host(mmc);

	msdc_switch_part(host_ctl, 0);

	if (!mmc_can_erase(mmc->card)) {
		err = -EOPNOTSUPP;
		goto out;
	}

	from = msdc_ctl->address;
	nr = msdc_ctl->total_size;

	if (from > host_ctl->mmc->card->ext_csd.sectors
	 || ((from + nr - 1) > host_ctl->mmc->card->ext_csd.sectors)) {
		err = -EINVAL;
		goto out;
	}

	if (mmc_can_discard(mmc->card))
		arg = MMC_DISCARD_ARG;
	else if (mmc_can_trim(mmc->card))
		arg = MMC_TRIM_ARG;
	else
		arg = MMC_ERASE_ARG;

#if DEBUG_MMC_IOCTL
	pr_debug("Erase range %x~%x\n", from, from + nr - 1);
#endif

	if (mmc_card_mmc(mmc->card)) {
		if (mmc->card->quirks & MMC_QUIRK_INAND_CMD38) {
			err = mmc_switch(mmc->card, EXT_CSD_CMD_SET_NORMAL,
				 INAND_CMD38_ARG_EXT_CSD,
				 arg == MMC_TRIM_ARG ?
				 INAND_CMD38_ARG_TRIM :
				 INAND_CMD38_ARG_ERASE,
				 0);
			if (err)
				goto out;
		}
	}

	err = mmc_erase(mmc->card, from, nr, arg);
out:

	mmc_release_host(mmc);

	msdc_ctl->result = err;

	return msdc_ctl->result;

}

static int simple_mmc_erase_partition(unsigned char *name)
{
#ifdef CONFIG_MTK_EMMC_SUPPORT
	struct mbr_part_info mbr_part;
	int l_ret;

	/* just support erase cache & data partition now */
	if (name &&
	    (strncmp(name, "usrdata", 7) == 0 ||
	     strncmp(name, "cache", 5) == 0)) {
		/* find erase start address and erase size,
		just support high capacity emmc card now */
		l_ret = simple_mmc_get_disk_info(&mbr_part, name);

		if (l_ret == 0) {
			/* do erase */
			pr_debug("erase %s start sector: 0x%x size: 0x%x\n",
				mbr_part.part_name,
				mbr_part.start_sector, mbr_part.nr_sects);
			simple_mmc_erase_func(mbr_part.start_sector,
				mbr_part.nr_sects);
		}
	}
#endif
	return 0;

}

static int simple_mmc_erase_partition_wrap(struct msdc_ioctl *msdc_ctl)
{
	unsigned char name[PARTITION_NAME_LENGTH];

	if (!msdc_ctl)
		return -EINVAL;

	if (msdc_ctl->total_size > PARTITION_NAME_LENGTH)
		return -EFAULT;

	if (copy_from_user(name, (unsigned char *)msdc_ctl->buffer,
		msdc_ctl->total_size))
		return -EFAULT;

	return simple_mmc_erase_partition(name);
}

static long simple_sd_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	struct msdc_ioctl msdc_ctl;
	struct msdc_host *host;
	int ret = 0;

	if ((struct msdc_ioctl *)arg == NULL) {
		switch (cmd) {
		case MSDC_REINIT_SDCARD:
			pr_err("sd ioctl re-init!!\n");
			ret = sd_ioctl_reinit((struct msdc_ioctl *)arg);
			break;

		case MSDC_CD_PIN_EN_SDCARD:
			pr_err("sd ioctl cd pin\n");
			ret = sd_ioctl_cd_pin_en((struct msdc_ioctl *)arg);
			break;

		case MSDC_SD_POWER_OFF:
			pr_err("sd ioctl power off!!!\n");
			host = msdc_get_host(MSDC_SD, 0, 0);
			if (host) {
				mmc_claim_host(host->mmc);
				mmc_power_off(host->mmc);
				mmc_release_host(host->mmc);
			}
			break;

		case MSDC_SD_POWER_ON:
			pr_err("sd ioctl power on!!!\n");
			host = msdc_get_host(MSDC_SD, 0, 0);
			/* FIX ME: kernel 3.18 does not provide
			    mmc_resume_host  */
			/*
			ret = mmc_resume_host(host->mmc);*/
			break;

		default:
			pr_err("mt_sd_ioctl:this opcode value is illegal!!\n");
			return -EINVAL;
		}
		return ret;
	}

	if (copy_from_user(&msdc_ctl, (struct msdc_ioctl *)arg,
		sizeof(struct msdc_ioctl))) {
		return -EFAULT;
	}

	if (msdc_ctl.opcode != MSDC_ERASE_PARTITION) {
		if ((msdc_ctl.host_num < 0)
		 || (msdc_ctl.host_num >= HOST_MAX_NUM)) {
			pr_err("invalid host num: %d\n", msdc_ctl.host_num);
			return -EINVAL;
		}
	}

	switch (msdc_ctl.opcode) {
	case MSDC_SINGLE_READ_WRITE:
	case MSDC_MULTIPLE_READ_WRITE:
		/* No More Support */
		/*
		 * msdc_ctl.result = simple_sd_ioctl_rw(&msdc_ctl);
		 */
		break;
	case MSDC_GET_CID:
		msdc_ctl.result = simple_sd_ioctl_get_cid(&msdc_ctl);
		break;
	case MSDC_GET_CSD:
		msdc_ctl.result = simple_sd_ioctl_get_csd(&msdc_ctl);
		break;
	case MSDC_GET_EXCSD:
		msdc_ctl.result = simple_sd_ioctl_get_excsd(&msdc_ctl);
		break;
	case MSDC_DRIVING_SETTING:
		if (1 == msdc_ctl.iswrite) {
			msdc_ctl.result =
				simple_sd_ioctl_set_driving(&msdc_ctl);
		} else {
			msdc_ctl.result =
				simple_sd_ioctl_get_driving(&msdc_ctl);
		}
		break;
	case MSDC_ERASE_PARTITION:
		msdc_ctl.result =
			simple_mmc_erase_partition_wrap(&msdc_ctl);
		break;
	case MSDC_ERASE_SELECTED_AREA:
		msdc_ctl.result = simple_sd_ioctl_erase_selected_area(
			&msdc_ctl);
		break;
	case MSDC_SD30_MODE_SWITCH:
		/* No More support now */
		break;
	case MSDC_GET_BOOTPART:
		msdc_ctl.result =
			simple_sd_ioctl_get_bootpart(&msdc_ctl);
		break;
	case MSDC_SET_BOOTPART:
		msdc_ctl.result =
			simple_sd_ioctl_set_bootpart(&msdc_ctl);
		break;
	case MSDC_GET_PARTSIZE:
		msdc_ctl.result =
			simple_sd_ioctl_get_partition_size(&msdc_ctl);
		break;
	default:
		pr_err("simple_sd_ioctl:invlalid opcode!!\n");
		return -EINVAL;
	}

	if (copy_to_user((struct msdc_ioctl *)arg, &msdc_ctl,
		sizeof(struct msdc_ioctl))) {
		return -EFAULT;
	}

	return msdc_ctl.result;
}

#ifdef CONFIG_COMPAT

struct compat_simple_sd_ioctl {
	compat_int_t opcode;
	compat_int_t host_num;
	compat_int_t iswrite;
	compat_int_t trans_type;
	compat_uint_t total_size;
	compat_uint_t address;
	compat_uptr_t buffer;
	compat_int_t cmd_pu_driving;
	compat_int_t cmd_pd_driving;
	compat_int_t dat_pu_driving;
	compat_int_t dat_pd_driving;
	compat_int_t clk_pu_driving;
	compat_int_t clk_pd_driving;
	compat_int_t ds_pu_driving;
	compat_int_t ds_pd_driving;
	compat_int_t rst_pu_driving;
	compat_int_t rst_pd_driving;
	compat_int_t clock_freq;
	compat_int_t partition;
	compat_int_t hopping_bit;
	compat_int_t hopping_time;
	compat_int_t result;
	compat_int_t sd30_mode;
	compat_int_t sd30_max_current;
	compat_int_t sd30_drive;
	compat_int_t sd30_power_control;
};

static int compat_get_simple_ion_allocation(
	struct compat_simple_sd_ioctl __user *arg32,
	struct msdc_ioctl __user *arg64)
{
	compat_int_t i;
	compat_uint_t u;
	compat_uptr_t p;
	int err;

	err = get_user(i, &(arg32->opcode));
	err |= put_user(i, &(arg64->opcode));
	err |= get_user(i, &(arg32->host_num));
	err |= put_user(i, &(arg64->host_num));
	err |= get_user(i, &(arg32->iswrite));
	err |= put_user(i, &(arg64->iswrite));
	err |= get_user(i, &(arg32->trans_type));
	err |= put_user(i, &(arg64->trans_type));
	err |= get_user(u, &(arg32->total_size));
	err |= put_user(u, &(arg64->total_size));
	err |= get_user(u, &(arg32->address));
	err |= put_user(u, &(arg64->address));
	err |= get_user(p, &(arg32->buffer));
	err |= put_user(compat_ptr(p), &(arg64->buffer));
	err |= get_user(i, &(arg32->cmd_pu_driving));
	err |= put_user(i, &(arg64->cmd_pu_driving));
	err |= get_user(i, &(arg32->cmd_pd_driving));
	err |= put_user(i, &(arg64->cmd_pd_driving));
	err |= get_user(i, &(arg32->dat_pu_driving));
	err |= put_user(i, &(arg64->dat_pu_driving));
	err |= get_user(i, &(arg32->dat_pd_driving));
	err |= put_user(i, &(arg64->dat_pd_driving));
	err |= get_user(i, &(arg32->clk_pu_driving));
	err |= put_user(i, &(arg64->clk_pu_driving));
	err |= get_user(i, &(arg32->clk_pd_driving));
	err |= put_user(i, &(arg64->clk_pd_driving));
	err |= get_user(i, &(arg32->ds_pu_driving));
	err |= put_user(i, &(arg64->ds_pu_driving));
	err |= get_user(i, &(arg32->ds_pd_driving));
	err |= put_user(i, &(arg64->ds_pd_driving));
	err |= get_user(i, &(arg32->rst_pu_driving));
	err |= put_user(i, &(arg64->rst_pu_driving));
	err |= get_user(i, &(arg32->rst_pd_driving));
	err |= put_user(i, &(arg64->rst_pd_driving));
	err |= get_user(i, &(arg32->clock_freq));
	err |= put_user(i, &(arg64->clock_freq));
	err |= get_user(i, &(arg32->partition));
	err |= put_user(i, &(arg64->partition));
	err |= get_user(i, &(arg32->hopping_bit));
	err |= put_user(i, &(arg64->hopping_bit));
	err |= get_user(i, &(arg32->hopping_time));
	err |= put_user(i, &(arg64->hopping_time));
	err |= get_user(i, &(arg32->result));
	err |= put_user(i, &(arg64->result));
	err |= get_user(i, &(arg32->sd30_mode));
	err |= put_user(i, &(arg64->sd30_mode));
	err |= get_user(i, &(arg32->sd30_max_current));
	err |= put_user(i, &(arg64->sd30_max_current));
	err |= get_user(i, &(arg32->sd30_drive));
	err |= put_user(i, &(arg64->sd30_drive));
	err |= get_user(i, &(arg32->sd30_power_control));
	err |= put_user(i, &(arg64->sd30_power_control));

	return err;
}

static int compat_put_simple_ion_allocation(
	struct compat_simple_sd_ioctl __user *arg32,
	struct msdc_ioctl __user *arg64)
{
	compat_int_t i;
	compat_uint_t u;
	int err;

	err = get_user(i, &(arg64->opcode));
	err |= put_user(i, &(arg32->opcode));
	err |= get_user(i, &(arg64->host_num));
	err |= put_user(i, &(arg32->host_num));
	err |= get_user(i, &(arg64->iswrite));
	err |= put_user(i, &(arg32->iswrite));
	err |= get_user(i, &(arg64->trans_type));
	err |= put_user(i, &(arg32->trans_type));
	err |= get_user(u, &(arg64->total_size));
	err |= put_user(u, &(arg32->total_size));
	err |= get_user(u, &(arg64->address));
	err |= put_user(u, &(arg32->address));
	err |= get_user(i, &(arg64->cmd_pu_driving));
	err |= put_user(i, &(arg32->cmd_pu_driving));
	err |= get_user(i, &(arg64->cmd_pd_driving));
	err |= put_user(i, &(arg32->cmd_pd_driving));
	err |= get_user(i, &(arg64->dat_pu_driving));
	err |= put_user(i, &(arg32->dat_pu_driving));
	err |= get_user(i, &(arg64->dat_pd_driving));
	err |= put_user(i, &(arg32->dat_pd_driving));
	err |= get_user(i, &(arg64->clk_pu_driving));
	err |= put_user(i, &(arg32->clk_pu_driving));
	err |= get_user(i, &(arg64->clk_pd_driving));
	err |= put_user(i, &(arg32->clk_pd_driving));
	err |= get_user(i, &(arg64->ds_pu_driving));
	err |= put_user(i, &(arg32->ds_pu_driving));
	err |= get_user(i, &(arg64->ds_pd_driving));
	err |= put_user(i, &(arg32->ds_pd_driving));
	err |= get_user(i, &(arg64->rst_pu_driving));
	err |= put_user(i, &(arg32->rst_pu_driving));
	err |= get_user(i, &(arg64->rst_pd_driving));
	err |= put_user(i, &(arg32->rst_pd_driving));
	err |= get_user(i, &(arg64->clock_freq));
	err |= put_user(i, &(arg32->clock_freq));
	err |= get_user(i, &(arg64->partition));
	err |= put_user(i, &(arg32->partition));
	err |= get_user(i, &(arg64->hopping_bit));
	err |= put_user(i, &(arg32->hopping_bit));
	err |= get_user(i, &(arg64->hopping_time));
	err |= put_user(i, &(arg32->hopping_time));
	err |= get_user(i, &(arg64->result));
	err |= put_user(i, &(arg32->result));
	err |= get_user(i, &(arg64->sd30_mode));
	err |= put_user(i, &(arg32->sd30_mode));
	err |= get_user(i, &(arg64->sd30_max_current));
	err |= put_user(i, &(arg32->sd30_max_current));
	err |= get_user(i, &(arg64->sd30_drive));
	err |= put_user(i, &(arg32->sd30_drive));
	err |= get_user(i, &(arg64->sd30_power_control));
	err |= put_user(i, &(arg32->sd30_power_control));

	return err;
}


static long simple_sd_compat_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	struct compat_simple_sd_ioctl *arg32;
	struct msdc_ioctl *arg64;
	int err, ret;

	if (!file->f_op || !file->f_op->unlocked_ioctl) {
		pr_err("f_op or unlocked ioctl is NULL.\n");
		return -ENOTTY;
	}

	if ((struct msdc_ioctl *)arg == NULL) {
		ret = file->f_op->unlocked_ioctl(file, cmd, (unsigned long)arg);
		return ret;
	}

	arg32 = compat_ptr(arg);
	arg64 = compat_alloc_user_space(sizeof(*arg64));
	if (arg64 == NULL)
		return -EFAULT;

	err = compat_get_simple_ion_allocation(arg32, arg64);
	if (err)
		return err;

	ret = file->f_op->unlocked_ioctl(file, arg64->opcode,
		(unsigned long)arg64);

	err = compat_put_simple_ion_allocation(arg32, arg64);

	return ret ? ret : err;
}

#endif

static const struct file_operations simple_msdc_em_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = simple_sd_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = simple_sd_compat_ioctl,
#endif
	.open = simple_sd_open,
};

static struct miscdevice simple_msdc_em_dev[] = {
	{
	 .minor = MISC_DYNAMIC_MINOR,
	 .name = "misc-sd",
	 .fops = &simple_msdc_em_fops,
	}
};

static int simple_sd_probe(struct platform_device *pdev)
{
	int ret = 0;

	pr_debug("in misc_sd_probe function\n");

	return ret;
}

static int simple_sd_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver simple_sd_driver = {
	.probe = simple_sd_probe,
	.remove = simple_sd_remove,

	.driver = {
		.name = DRV_NAME_MISC,
		.owner = THIS_MODULE,
	},
};

static int __init simple_sd_init(void)
{
	int ret;

	sg_msdc_multi_buffer = kmalloc(64 * 1024, GFP_KERNEL);
	if (sg_msdc_multi_buffer == NULL)
		return 0;

	ret = platform_driver_register(&simple_sd_driver);
	if (ret) {
		pr_err(DRV_NAME_MISC ": Can't register driver\n");
		return ret;
	}
	pr_debug(DRV_NAME_MISC ": MediaTek simple SD/MMC Card Driver\n");

	/*msdc0 is for emmc only, just for emmc */
	/* ret = misc_register(&simple_msdc_em_dev[host->id]); */
	ret = misc_register(&simple_msdc_em_dev[0]);
	if (ret) {
		pr_err("register MSDC Slot[0] misc driver failed (%d)\n", ret);
		return ret;
	}

	return 0;
}

static void __exit simple_sd_exit(void)
{
	if (sg_msdc_multi_buffer != NULL) {
		kfree(sg_msdc_multi_buffer);
		sg_msdc_multi_buffer = NULL;
	}

	misc_deregister(&simple_msdc_em_dev[0]);

	platform_driver_unregister(&simple_sd_driver);
}

module_init(simple_sd_init);
module_exit(simple_sd_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("simple MediaTek SD/MMC Card Driver");
MODULE_AUTHOR("feifei.wang <feifei.wang@mediatek.com>");
