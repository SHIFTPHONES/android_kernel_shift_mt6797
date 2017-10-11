/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program
 * If not, see <http://www.gnu.org/licenses/>.
 */
/*******************************************************************************
 *
 * Filename:
 * ---------
 *   mt_soc_pcm_btcvsd.c
 *
 * Project:
 * --------
 *    Audio Driver Kernel Function
 *
 * Description:
 * ------------
 *   Audio btcvsd common function
 *
 *
 *------------------------------------------------------------------------------
 *
 *
 *******************************************************************************/


/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/

#include <linux/dma-mapping.h>
#include <linux/delay.h>

#include "mt_soc_pcm_btcvsd.h"

CVSD_MEMBLOCK_T BT_CVSD_Mem;
struct device *mDev_btcvsd_rx;
struct device *mDev_btcvsd_tx;
BT_SCO_T btsco;
bool isProbeDone = 0;
static kal_uint32 BTCVSD_write_wait_queue_flag;
static kal_uint32 BTCVSD_read_wait_queue_flag;
static kal_uint32 writeToBT_cnt;
static kal_uint32 readFromBT_cnt;

static bool rx_timeout;
static bool tx_timeout;

TIME_BUFFER_INFO_T time_buffer_info_rx;
TIME_BUFFER_INFO_T time_buffer_info_tx;
kal_uint64 BT_RX_timestamp;
kal_uint64 BT_TX_timestamp;
kal_uint64 BT_RX_bufdata_equivalent_time;
kal_uint64 BT_TX_bufdata_equivalent_time;


/*=============================================================================================
/     BT SCO Internal Function
/=============================================================================================*/

void AudDrv_BTCVSD_DataTransfer(BT_SCO_DIRECT uDir, kal_uint8 *pSrc,
										kal_uint8 *pDst, kal_uint32 uBlockSize,
										kal_uint32 uBlockNum, CVSD_STATE uState)
{
	kal_int32 i, j;

	if (uBlockSize == 60 || uBlockSize == 120 || uBlockSize == 20) {
		kal_uint32 *pSrc32 = (kal_uint32 *)pSrc;
		kal_uint32 *pDst32 = (kal_uint32 *)pDst;

		for (i = 0; i < (uBlockSize * uBlockNum / 4); i++)
			*pDst32++ = *pSrc32++;
	} else {
		kal_uint16 *pSrc16 = (kal_uint16 *)pSrc;
		kal_uint16 *pDst16 = (kal_uint16 *)pDst;

		for (j = 0; j < uBlockNum; j++) {
			for (i = 0; i < (uBlockSize / 2); i++)
				*pDst16++ = *pSrc16++;

			if (uDir == BT_SCO_DIRECT_BT2ARM)
				pSrc16++;
			else
				pDst16++;
		}
	}
}

void AudDrv_BTCVSD_ReadFromBT(BT_SCO_PACKET_LEN uLen,
											kal_uint32 uPacketLength,
											kal_uint32 uPacketNumber,
											kal_uint32 uBlockSize,
											kal_uint32 uControl)
{
	kal_int32 i;
	kal_uint16 pv;
	kal_uint8 *pSrc;
	kal_uint8 *pPacketBuf;
	unsigned long flags;
	unsigned long connsys_addr_rx, ap_addr_rx;

	LOGBT("%s(+) btsco.pRX->iPacket_w=%d\n", __func__, btsco.pRX->iPacket_w);
	connsys_addr_rx = *bt_hw_REG_PACKET_R;
	ap_addr_rx = (unsigned long)BTSYS_SRAM_BANK2_BASE_ADDRESS + (connsys_addr_rx & 0xFFFF);
	LOGBT("%s connsys_addr_rx=0x%lx,ap_addr_rx=0x%lx\n",
			__func__, connsys_addr_rx, ap_addr_rx);
	pSrc = (kal_uint8 *)ap_addr_rx;

	LOGBT("%s uPacketLength=%d,uPacketNumber=%d, btsco.uRXState=%d\n",
			__func__, uPacketLength, uPacketNumber, btsco.uRXState);
	AudDrv_BTCVSD_DataTransfer(BT_SCO_DIRECT_BT2ARM, pSrc, btsco.pRX->TempPacketBuf,
								uPacketLength, uPacketNumber, btsco.uRXState);
	LOGBT("%s AudDrv_BTCVSD_DataTransfer DONE!!!,uControl=0x%x,uLen=%d\n", __func__, uControl, uLen);

	spin_lock_irqsave(&auddrv_btcvsd_rx_lock, flags);
	for (i = 0; i < uBlockSize; i++) {
#ifdef TEST_PACKETLOSS
		packet_cnt++;
		if (packet_cnt == 30) {
			pr_debug("%s()Test Packet Loss\n", __func__);
			memset((void *)uSilencePattern, 0x55, SCO_RX_PLC_SIZE);
			memcpy(btsco.pRX->PacketBuf[btsco.pRX->iPacket_w & SCO_RX_PACKET_MASK],
					(void *)&uSilencePattern, SCO_RX_PLC_SIZE);
			pv = 0;
			packet_cnt = 0;
		} else {
			memcpy(btsco.pRX->PacketBuf[btsco.pRX->iPacket_w & SCO_RX_PACKET_MASK],
					btsco.pRX->TempPacketBuf + (SCO_RX_PLC_SIZE * i), SCO_RX_PLC_SIZE);
			if ((uControl & btsco_PacketValidMask[uLen][i]) == btsco_PacketValidMask[uLen][i])
				pv = 1;
			else
				pv = 0;
		}
#else
		memcpy(btsco.pRX->PacketBuf[btsco.pRX->iPacket_w & SCO_RX_PACKET_MASK],
				btsco.pRX->TempPacketBuf + (SCO_RX_PLC_SIZE * i), SCO_RX_PLC_SIZE);
		if ((uControl & btsco_PacketValidMask[uLen][i]) == btsco_PacketValidMask[uLen][i])
			pv = 1;
		else
			pv = 0;
#endif

		pPacketBuf = (kal_uint8 *)btsco.pRX->PacketBuf + (btsco.pRX->iPacket_w & SCO_RX_PACKET_MASK) *
					(SCO_RX_PLC_SIZE + BTSCO_CVSD_PACKET_VALID_SIZE) + SCO_RX_PLC_SIZE;
		memcpy((void *)pPacketBuf, (void *)&pv , BTSCO_CVSD_PACKET_VALID_SIZE);
		btsco.pRX->iPacket_w++;
	}
	spin_unlock_irqrestore(&auddrv_btcvsd_rx_lock, flags);
	LOGBT("%s(-) btsco.pRX->iPacket_w=%d\n", __func__, btsco.pRX->iPacket_w);
}

void AudDrv_BTCVSD_WriteToBT(BT_SCO_PACKET_LEN uLen,
											kal_uint32 uPacketLength,
											kal_uint32 uPacketNumber,
											kal_uint32 uBlockSize)
{
	kal_int32 i;
	unsigned long flags;
	kal_uint8 *pDst;
	unsigned long connsys_addr_tx, ap_addr_tx;
	bool new_ap_addr_tx = true;

	if (!btsco.pTX) {
		pr_err("%s(), btsco.pTX == NULL\n", __func__);
		return;
	}

	LOGBT("%s(+) btsco.pTX->iPacket_r=%d\n", __func__, btsco.pTX->iPacket_r);

	spin_lock_irqsave(&auddrv_btcvsd_tx_lock, flags);
	for (i = 0; i < uBlockSize; i++) {
		memcpy(btsco.pTX->TempPacketBuf + (SCO_TX_ENCODE_SIZE * i),
		       (btsco.pTX->PacketBuf +
			(btsco.pTX->iPacket_r & SCO_TX_PACKET_MASK) *
			SCO_TX_ENCODE_SIZE),
		       SCO_TX_ENCODE_SIZE);

		btsco.pTX->iPacket_r++;
	}
	spin_unlock_irqrestore(&auddrv_btcvsd_tx_lock, flags);

	connsys_addr_tx = *bt_hw_REG_PACKET_W;
	ap_addr_tx = (unsigned long)BTSYS_SRAM_BANK2_BASE_ADDRESS + (connsys_addr_tx & 0xFFFF);
	LOGBT("AudDrv_BTCVSD_WriteToBT connsys_addr_tx=0x%lx,ap_addr_tx=0x%lx\n", connsys_addr_tx, ap_addr_tx);
	pDst = (kal_uint8 *)ap_addr_tx;

	if (!btsco.pTX->mute) {
		AudDrv_BTCVSD_DataTransfer(BT_SCO_DIRECT_ARM2BT, btsco.pTX->TempPacketBuf,
					   pDst, uPacketLength, uPacketNumber, btsco.uTXState);
	}

	/* store bt tx buffer sram info */
	btsco.pTX->buffer_info.packet_length = uPacketLength;
	btsco.pTX->buffer_info.packet_number = uPacketNumber;
	for (i = 0; i < btsco.pTX->buffer_info.num_valid_addr; i++) {
		if (btsco.pTX->buffer_info.addr_to_clean[i] == ap_addr_tx) {
			new_ap_addr_tx = false;
			break;
		}
	}
	if (new_ap_addr_tx) {
		btsco.pTX->buffer_info.num_valid_addr++;
		pr_warn("%s(), new ap_addr_tx = 0x%lx, num_valid_addr %d\n",
			__func__, ap_addr_tx, btsco.pTX->buffer_info.num_valid_addr);
		btsco.pTX->buffer_info.addr_to_clean[btsco.pTX->buffer_info.num_valid_addr - 1] = ap_addr_tx;
	}

	if (btsco.pTX->mute)
		btcvsd_tx_clean_buffer();

	LOGBT("%s(-),btsco.pTX->iPacket_r=%d\n", __func__, btsco.pTX->iPacket_r);
}


int AudDrv_btcvsd_Allocate_Buffer(kal_uint8 isRX)
{
	pr_warn("%s(+) isRX=%d\n", __func__, isRX);

	if (isRX == 1) {
		readFromBT_cnt = 0;
		rx_timeout = false;

		BT_CVSD_Mem.u4RXBufferSize = sizeof(BT_SCO_RX_T);

		if ((BT_CVSD_Mem.pucRXVirtBufAddr == NULL)
				&& (BT_CVSD_Mem.pucRXPhysBufAddr == 0)) {
			BT_CVSD_Mem.pucRXVirtBufAddr = dma_alloc_coherent(mDev_btcvsd_rx,
											BT_CVSD_Mem.u4RXBufferSize,
											&BT_CVSD_Mem.pucRXPhysBufAddr,
											GFP_KERNEL);
			if ((0 == BT_CVSD_Mem.pucRXPhysBufAddr) || (NULL == BT_CVSD_Mem.pucRXVirtBufAddr)) {
				pr_debug("AudDrv_btcvsd_Allocate_Buffer dma_alloc_coherent RX fail\n");
				return -1;
			}

			memset((void *)BT_CVSD_Mem.pucRXVirtBufAddr, 0, BT_CVSD_Mem.u4RXBufferSize);

			LOGBT("BT_CVSD_Mem.pucRXVirtBufAddr = %p BT_CVSD_Mem.pucRXPhysBufAddr = %pad\n" ,
						  BT_CVSD_Mem.pucRXVirtBufAddr,
						  &BT_CVSD_Mem.pucRXPhysBufAddr);

			btsco.pRX = (BT_SCO_RX_T *)(BT_CVSD_Mem.pucRXVirtBufAddr);
			btsco.pRX->u4BufferSize = SCO_RX_PACKER_BUF_NUM *
							(SCO_RX_PLC_SIZE + BTSCO_CVSD_PACKET_VALID_SIZE);

			/* AudDrv_Allocate_mem_Buffer */
			BT_CVSD_Mem.RX_btcvsd_dma_buf.area = BT_CVSD_Mem.pucRXVirtBufAddr;
			BT_CVSD_Mem.RX_btcvsd_dma_buf.addr = BT_CVSD_Mem.pucRXPhysBufAddr;
			BT_CVSD_Mem.RX_btcvsd_dma_buf.bytes = BT_CVSD_Mem.u4RXBufferSize;
		}
	} else {
		writeToBT_cnt = 0;
		tx_timeout = false;
		BT_CVSD_Mem.u4TXBufferSize = sizeof(BT_SCO_TX_T);
		if ((BT_CVSD_Mem.pucTXVirtBufAddr == NULL)
				&& (BT_CVSD_Mem.pucTXPhysBufAddr == 0)) {
			BT_CVSD_Mem.pucTXVirtBufAddr = dma_alloc_coherent(mDev_btcvsd_tx,
											BT_CVSD_Mem.u4TXBufferSize,
											&BT_CVSD_Mem.pucTXPhysBufAddr,
											GFP_KERNEL);
			if ((0 == BT_CVSD_Mem.pucTXPhysBufAddr) || (NULL == BT_CVSD_Mem.pucTXVirtBufAddr)) {
				pr_debug("AudDrv_btcvsd_Allocate_Buffer dma_alloc_coherent TX fail\n");
				return -1;
			}
			memset((void *)BT_CVSD_Mem.pucTXVirtBufAddr, 0, BT_CVSD_Mem.u4TXBufferSize);

			LOGBT("BT_CVSD_Mem.pucTXVirtBufAddr = 0x%p BT_CVSD_Mem.pucTXPhysBufAddr = %pad\n" ,
						  BT_CVSD_Mem.pucTXVirtBufAddr,
						  &BT_CVSD_Mem.pucTXPhysBufAddr);

			btsco.pTX = (BT_SCO_TX_T *)(BT_CVSD_Mem.pucTXVirtBufAddr);
			btsco.pTX->u4BufferSize = SCO_TX_PACKER_BUF_NUM * SCO_TX_ENCODE_SIZE;

			/* AudDrv_Allocate_mem_Buffer */
			BT_CVSD_Mem.TX_btcvsd_dma_buf.area = BT_CVSD_Mem.pucTXVirtBufAddr;
			BT_CVSD_Mem.TX_btcvsd_dma_buf.addr = BT_CVSD_Mem.pucTXPhysBufAddr;
			BT_CVSD_Mem.TX_btcvsd_dma_buf.bytes = BT_CVSD_Mem.u4TXBufferSize;
		}
	}
	pr_debug("%s(-)\n", __func__);
	return 0;
}

int AudDrv_btcvsd_Free_Buffer(kal_uint8 isRX)
{
	pr_warn("%s(+) isRX=%d\n", __func__, isRX);

	if (isRX == 1) {
		if ((BT_CVSD_Mem.pucRXVirtBufAddr != NULL) && (BT_CVSD_Mem.pucRXPhysBufAddr != 0)) {
			LOGBT("%s dma_free_coherent pucRXVirtBufAddr = 0x%p pucRXPhysBufAddr = %pad",
					__func__, BT_CVSD_Mem.pucRXVirtBufAddr, &BT_CVSD_Mem.pucRXPhysBufAddr);
			btsco.pRX =  NULL;
			dma_free_coherent(mDev_btcvsd_rx, BT_CVSD_Mem.u4RXBufferSize,
								BT_CVSD_Mem.pucRXVirtBufAddr,
								BT_CVSD_Mem.pucRXPhysBufAddr);
			BT_CVSD_Mem.u4RXBufferSize = 0;
			BT_CVSD_Mem.pucRXVirtBufAddr = NULL;
			BT_CVSD_Mem.pucRXPhysBufAddr = 0;
		} else {
			pr_warn("%s cannot dma_free_coherent pucRXVirtBufAddr = 0x%p pucRXPhysBufAddr = %pad",
					__func__, BT_CVSD_Mem.pucRXVirtBufAddr, &BT_CVSD_Mem.pucRXPhysBufAddr);
			return -1;
		}
	} else {
		if ((BT_CVSD_Mem.pucTXVirtBufAddr != NULL) && (BT_CVSD_Mem.pucTXPhysBufAddr != 0)) {
			LOGBT("%s dma_free_coherent pucTXVirtBufAddr = 0x%p pucTXPhysBufAddr = %pad",
					__func__, BT_CVSD_Mem.pucTXVirtBufAddr, &BT_CVSD_Mem.pucTXPhysBufAddr);
			btsco.pTX =  NULL;
			dma_free_coherent(mDev_btcvsd_tx, BT_CVSD_Mem.u4TXBufferSize,
								BT_CVSD_Mem.pucTXVirtBufAddr,
								BT_CVSD_Mem.pucTXPhysBufAddr);
			BT_CVSD_Mem.u4TXBufferSize = 0;
			BT_CVSD_Mem.pucTXVirtBufAddr = NULL;
			BT_CVSD_Mem.pucTXPhysBufAddr = 0;

		} else {
			pr_warn("%s cannot dma_free_coherent pucTXVirtBufAddr = 0x%p pucTXPhysBufAddr = %pad",
					__func__, BT_CVSD_Mem.pucTXVirtBufAddr, &BT_CVSD_Mem.pucTXPhysBufAddr);
			return -1;
		}
	}

	return 0;
}

int AudDrv_BTCVSD_IRQ_handler(void)
{
	kal_uint32 uPacketType, uPacketNumber, uPacketLength, uBufferCount_TX, uBufferCount_RX, uControl;

	LOGBT("+%s, irq=%d\n", __func__, btcvsd_irq_number);

	if ((btsco.uRXState != BT_SCO_RXSTATE_RUNNING && btsco.uRXState != BT_SCO_RXSTATE_ENDING)
		&& (btsco.uTXState != BT_SCO_TXSTATE_RUNNING && btsco.uTXState != BT_SCO_TXSTATE_ENDING)
		&& (btsco.uTXState != BT_SCO_TXSTATE_DIRECT_LOOPBACK)) {
		pr_debug("%s in idle state: btsco.uRXState: %d, btsco.uTXState: %d\n",
					__func__, btsco.uRXState, btsco.uTXState);
		*bt_hw_REG_CONTROL &= ~BT_CVSD_CLEAR;
		goto AudDrv_BTCVSD_IRQ_handler_exit;
	}

	uControl = *bt_hw_REG_CONTROL;
	uPacketType = (uControl >> 18) & 0x7;
	LOGBT("%s BT uControl =0x%x, BT uPacketType=%d\n", __func__, uControl, uPacketType);

	if (((uControl >> 31) & 1) == 0) {
		*bt_hw_REG_CONTROL &= ~BT_CVSD_CLEAR;
		goto AudDrv_BTCVSD_IRQ_handler_exit;
	}

	if (uPacketType >= BT_SCO_CVSD_MAX) {
		pr_warn("%s(), invalid uPacketType %u, exit\n", __func__, uPacketType);
		*bt_hw_REG_CONTROL &= ~BT_CVSD_CLEAR;
		goto AudDrv_BTCVSD_IRQ_handler_exit;
	}

	uPacketLength = (kal_uint32)btsco_PacketInfo[uPacketType][0];
	uPacketNumber = (kal_uint32)btsco_PacketInfo[uPacketType][1];
	uBufferCount_TX = (kal_uint32)btsco_PacketInfo[uPacketType][2];
	uBufferCount_RX = (kal_uint32)btsco_PacketInfo[uPacketType][3];

	LOGBT("%s uPacketLength=%d, uPacketNumber=%d, uBufferCount_TX=%d, uBufferCount_RX=%d\n",
			__func__, uPacketLength, uPacketNumber, uBufferCount_TX, uBufferCount_RX);
	LOGBT("btsco.uTXState=0x%x,btsco.uRXState=0x%x\n", btsco.uTXState, btsco.uRXState);

	if (btsco.pTX && btsco.uTXState == BT_SCO_TXSTATE_DIRECT_LOOPBACK) {
		kal_uint8 *pSrc, *pDst;
		unsigned long connsys_addr_rx, ap_addr_rx, connsys_addr_tx, ap_addr_tx;
		kal_uint8 *pPacketBuf;
		kal_int32 i;
		unsigned long flags;

		connsys_addr_rx = *bt_hw_REG_PACKET_R;
		ap_addr_rx = (unsigned long)BTSYS_SRAM_BANK2_BASE_ADDRESS + (connsys_addr_rx & 0xFFFF);
		LOGBT("ReadFromBT connsys_addr_rx=0x%lx,ap_addr_rx=0x%lx\n", connsys_addr_rx, ap_addr_rx);
		pSrc = (kal_uint8 *)ap_addr_rx;

		connsys_addr_tx = *bt_hw_REG_PACKET_W;
		ap_addr_tx = (unsigned long)BTSYS_SRAM_BANK2_BASE_ADDRESS + (connsys_addr_tx & 0xFFFF);
		LOGBT("AudDrv_BTCVSD_WriteToBT connsys_addr_tx=0x%lx,ap_addr_tx=0x%lx\n", connsys_addr_tx, ap_addr_tx);
		pDst = (kal_uint8 *)ap_addr_tx;

		AudDrv_BTCVSD_DataTransfer(BT_SCO_DIRECT_BT2ARM, pSrc, btsco.pTX->TempPacketBuf,
									uPacketLength, uPacketNumber,
									BT_SCO_RXSTATE_RUNNING);
		AudDrv_BTCVSD_DataTransfer(BT_SCO_DIRECT_ARM2BT, btsco.pTX->TempPacketBuf, pDst,
									uPacketLength, uPacketNumber,
									BT_SCO_TXSTATE_RUNNING);

		if (btsco.pRX) {
			spin_lock_irqsave(&auddrv_btcvsd_rx_lock, flags);
			for (i = 0; i < uBufferCount_RX; i++) {
				memset(btsco.pRX->PacketBuf[btsco.pRX->iPacket_w & SCO_RX_PACKET_MASK],
				0, SCO_RX_PLC_SIZE);

				pPacketBuf = (kal_uint8 *)btsco.pRX->PacketBuf +
					(btsco.pRX->iPacket_w & SCO_RX_PACKET_MASK) *
					(SCO_RX_PLC_SIZE + BTSCO_CVSD_PACKET_VALID_SIZE) + SCO_RX_PLC_SIZE;
				memset((void *)pPacketBuf, 0 , BTSCO_CVSD_PACKET_VALID_SIZE);
				btsco.pRX->iPacket_w++;
			}
			spin_unlock_irqrestore(&auddrv_btcvsd_rx_lock, flags);

		}

		writeToBT_cnt++;
		readFromBT_cnt++;
	} else {
		if (btsco.pRX) {
			if (btsco.uRXState == BT_SCO_RXSTATE_RUNNING || btsco.uRXState == BT_SCO_RXSTATE_ENDING) {
				LOGBT("%s pRX->fOverflow=%d, iPacket_w=%d, iPacket_r=%d, uBufferCount_RX=%d\n",
						__func__, btsco.pRX->fOverflow, btsco.pRX->iPacket_w,
						btsco.pRX->iPacket_r, uBufferCount_RX);
				if (btsco.pRX->fOverflow) {
					if (btsco.pRX->iPacket_w - btsco.pRX->iPacket_r <=
						SCO_RX_PACKER_BUF_NUM - 2 * uBufferCount_RX) {
						/* free space is larger then twice interrupt rx data size */
						btsco.pRX->fOverflow = false; /*KAL_FALSE;*/
						pr_debug("AudDrv_BTCVSD_IRQ_handler pRX->fOverflow FALSE!!!\n");
					}
				}

				if (!btsco.pRX->fOverflow &&
					(btsco.pRX->iPacket_w - btsco.pRX->iPacket_r <=
					SCO_RX_PACKER_BUF_NUM - uBufferCount_RX)) {
					AudDrv_BTCVSD_ReadFromBT(uPacketType, uPacketLength,
											uPacketNumber, uBufferCount_RX,
											uControl);
					readFromBT_cnt++;
				} else {
					btsco.pRX->fOverflow = true; /*KAL_TRUE;*/
					pr_debug("%s pRX->fOverflow TRUE!!!\n", __func__);
				}
			}
		}

		if (btsco.pTX) {
			tx_timeout = false;
			if (btsco.uTXState == BT_SCO_TXSTATE_RUNNING || btsco.uTXState == BT_SCO_TXSTATE_ENDING) {
				LOGBT("%s pTX->fUnderflow=%d, iPacket_w=%d, iPacket_r=%d, uBufferCount_TX=%d\n",
						__func__, btsco.pTX->fUnderflow, btsco.pTX->iPacket_w,
						btsco.pTX->iPacket_r, uBufferCount_TX);
				if (btsco.pTX->fUnderflow) {
					/* prepared data is larger then twice interrupt tx data size */
					if (btsco.pTX->iPacket_w - btsco.pTX->iPacket_r >= 2 * uBufferCount_TX) {
						btsco.pTX->fUnderflow = false; /*KAL_FALSE;*/
						pr_debug("%s pTX->fUnderflow FALSE!!!\n", __func__);
					}
				}

				if ((!btsco.pTX->fUnderflow
						&& (btsco.pTX->iPacket_w - btsco.pTX->iPacket_r >= uBufferCount_TX))
						|| btsco.uTXState ==  BT_SCO_TXSTATE_ENDING) {
					AudDrv_BTCVSD_WriteToBT(uPacketType, uPacketLength,
											uPacketNumber, uBufferCount_TX);
					writeToBT_cnt++;
				} else {
					btsco.pTX->fUnderflow = true; /*KAL_TRUE;*/
					pr_debug("%s pTX->fUnderflow TRUE!!!\n", __func__);
				}
			}
		}
	}
	LOGBT("writeToBT_cnt=%d, readFromBT_cnt=%d\n", writeToBT_cnt, readFromBT_cnt);

	*bt_hw_REG_CONTROL &= ~BT_CVSD_CLEAR;


	if (btsco.uRXState == BT_SCO_RXSTATE_RUNNING || btsco.uRXState == BT_SCO_RXSTATE_ENDING) {
		BTCVSD_read_wait_queue_flag = 1;
		wake_up_interruptible(&BTCVSD_Read_Wait_Queue);
		snd_pcm_period_elapsed(BT_CVSD_Mem.RX_substream);
	}
	if (btsco.uTXState == BT_SCO_TXSTATE_RUNNING || btsco.uTXState == BT_SCO_TXSTATE_ENDING) {
		BTCVSD_write_wait_queue_flag = 1;
		wake_up_interruptible(&BTCVSD_Write_Wait_Queue);
		snd_pcm_period_elapsed(BT_CVSD_Mem.TX_substream);
	}


AudDrv_BTCVSD_IRQ_handler_exit:
	return IRQ_HANDLED;
}

bool Register_BTCVSD_Irq(void *dev, unsigned int irq_number)
{
	int ret;

	ret = request_irq(irq_number, (irq_handler_t) AudDrv_BTCVSD_IRQ_handler,
				  IRQF_TRIGGER_LOW /*IRQF_TRIGGER_FALLING */ , "BTCVSD_ISR_Handle", dev);

	if (ret)
		pr_debug("%s fail!!! irq_number=%d\n", __func__, irq_number);

	return ret;
}

ssize_t AudDrv_btcvsd_read(char __user *data, size_t count)
{
	char *Read_Data_Ptr = (char *)data;
	int ret;
	ssize_t read_size = 0, read_count = 0, BTSCORX_ReadIdx_tmp;
	unsigned long u4DataRemained;
	unsigned long flags;
	kal_uint64 read_timeout_limit;
	int max_timeout_trial = 2;
	unsigned int packet_size = SCO_RX_PLC_SIZE + BTSCO_CVSD_PACKET_VALID_SIZE;

	if ((btsco.pRX == NULL) || (btsco.pRX->u4BufferSize == 0)) {
		pr_debug("AudDrv_btcvsd_read btsco.pRX == NULL || btsco.pRX->u4BufferSize == 0!!!\n");
		msleep(60);
		return -1;
	}

	read_timeout_limit = ((kal_uint64)SCO_RX_PACKER_BUF_NUM * SCO_RX_PLC_SIZE * 16 * 1000000000) / 2 / 2 / 64000;

	while (count) {
		LOGBT("%s btsco.pRX->iPacket_w=%d, btsco.pRX->iPacket_r=%d,count=%zu\n",
				__func__, btsco.pRX->iPacket_w, btsco.pRX->iPacket_r, count);

		spin_lock_irqsave(&auddrv_btcvsd_rx_lock, flags);
		/* available data in RX packet buffer */
		u4DataRemained = (btsco.pRX->iPacket_w - btsco.pRX->iPacket_r)
			* (SCO_RX_PLC_SIZE + BTSCO_CVSD_PACKET_VALID_SIZE);

		BTSCORX_ReadIdx_tmp = (btsco.pRX->iPacket_r & SCO_RX_PACKET_MASK)
			* (SCO_RX_PLC_SIZE + BTSCO_CVSD_PACKET_VALID_SIZE);
		spin_unlock_irqrestore(&auddrv_btcvsd_rx_lock, flags);

		/* count must be multiple of SCO_RX_PLC_SIZE + BTSCO_CVSD_PACKET_VALID_SIZE */
		if (count % packet_size != 0 ||
		    u4DataRemained % packet_size != 0) {
			pr_err("%s(), count %zu or u4DataRemained %lu is not multiple of (SCO_RX_PLC_SIZE + BTSCO_CVSD_PACKET_VALID_SIZE)\n",
			       __func__, count, u4DataRemained);

			count -= count % packet_size;
			u4DataRemained -= u4DataRemained % packet_size;
		}

		if (count > u4DataRemained)
			read_size = u4DataRemained;
		else
			read_size = count;

		LOGBT("AudDrv_btcvsd_read read_size=%zd, BTSCORX_ReadIdx_tmp=%zd\n", read_size, BTSCORX_ReadIdx_tmp);
		LOGBT("%s finish0, read_count:%zd,read_size:%zd,DataRemained:0x%zd,iPacket_r:0x%x,iPacket_w:0x%x\r\n",
				__func__, read_count, read_size,
				u4DataRemained, btsco.pRX->iPacket_r, btsco.pRX->iPacket_w);
		if (BTSCORX_ReadIdx_tmp + read_size < btsco.pRX->u4BufferSize) /* copy once */ {
			LOGBT("%s 1 copy_to_user target=0x%p, source=0x%p, read_size=%zd\n",
					__func__, Read_Data_Ptr,
					((unsigned char *)btsco.pRX->PacketBuf + BTSCORX_ReadIdx_tmp),
					read_size);
			if (copy_to_user
					((void __user *)Read_Data_Ptr,
					(void *)((kal_uint8 *)btsco.pRX->PacketBuf + BTSCORX_ReadIdx_tmp),
					read_size)) {
				pr_debug("%s Fail 1 copy to user Read_Data_Ptr:%p,PacketBuf:%p,BTSCORX_ReadIdx_tmp:%zd,read_size:%zd",
						 __func__, Read_Data_Ptr, (kal_uint8 *)btsco.pRX->PacketBuf,
						 BTSCORX_ReadIdx_tmp, read_size);
				if (read_count == 0)
					return -1;
				else
					return read_count;
			}

			read_count += read_size;
			spin_lock_irqsave(&auddrv_btcvsd_rx_lock, flags);
			/* 2 byte is packetvalid info */
			btsco.pRX->iPacket_r += read_size / (SCO_RX_PLC_SIZE + BTSCO_CVSD_PACKET_VALID_SIZE);
			spin_unlock_irqrestore(&auddrv_btcvsd_rx_lock, flags);

			Read_Data_Ptr += read_size;
			count -= read_size;
			LOGBT("%s finish1, read_sizesize:%zd, pRX->iPacket_r:0x%x, pRX->iPacket_w:%x, count:%zu\r\n",
					__func__, read_size, btsco.pRX->iPacket_r, btsco.pRX->iPacket_w, count);
		} else /* copy twice */ {
			unsigned long size_1 = btsco.pRX->u4BufferSize - BTSCORX_ReadIdx_tmp;
			unsigned long size_2 = read_size - size_1;

			LOGBT("%s 2-2 copy_to_user target=%p, source=0x%p, size_1=%zu\n",
					__func__, Read_Data_Ptr,
					((unsigned char *)btsco.pRX->PacketBuf + BTSCORX_ReadIdx_tmp),
					size_1);
			if (copy_to_user
					((void __user *)Read_Data_Ptr,
					(void *)((kal_uint8 *)btsco.pRX->PacketBuf + BTSCORX_ReadIdx_tmp),
					size_1)) {
				LOGBT("%s Fail 2 copy to user Ptr:%p,PacketBuf:%p,ReadIdx_tmp:0x%zx,read_size:%zd",
					__func__, Read_Data_Ptr, btsco.pRX->PacketBuf, BTSCORX_ReadIdx_tmp, read_size);
				if (read_count == 0)
					return -1;
				else
					return read_count;
			}

			read_count += size_1;
			spin_lock_irqsave(&auddrv_btcvsd_rx_lock, flags);
			/* 2 byte is packetvalid info */
			btsco.pRX->iPacket_r += size_1 / (SCO_RX_PLC_SIZE + BTSCO_CVSD_PACKET_VALID_SIZE);
			spin_unlock_irqrestore(&auddrv_btcvsd_rx_lock, flags);

			LOGBT("%s 2-2 copy_to_user target=0x%p, source=0x%p,size_2=%zu\n",
					__func__,
					(Read_Data_Ptr + size_1),
					((unsigned char *)btsco.pRX->PacketBuf + BTSCORX_ReadIdx_tmp + size_1),
					size_2);
			if (copy_to_user
					((void __user *)(Read_Data_Ptr + size_1),
					(void *)((kal_uint8 *)btsco.pRX->PacketBuf),
					size_2)) {
				LOGBT("%s Fail 3 copy to user Ptr:%p,PacketBuf:%p,ReadIdx_tmp:0x%zd,read_size:%zd",
					__func__, Read_Data_Ptr, btsco.pRX->PacketBuf,
					BTSCORX_ReadIdx_tmp, read_size);
				if (read_count == 0)
					return -1;
				else
					return read_count;
			}

			read_count += size_2;
			spin_lock_irqsave(&auddrv_btcvsd_rx_lock, flags);
			/* 2 byte is packetvalid info */
			btsco.pRX->iPacket_r += size_2 / (SCO_RX_PLC_SIZE + BTSCO_CVSD_PACKET_VALID_SIZE);
			spin_unlock_irqrestore(&auddrv_btcvsd_rx_lock, flags);

			count -= read_size;
			Read_Data_Ptr += read_size;
			LOGBT("%s finish3,copy size_2:%zu,iPacket_r:0x%x,iPacket_w:0x%x u4DataRemained:%zu\r\n",
					__func__, size_2, btsco.pRX->iPacket_r, btsco.pRX->iPacket_w, u4DataRemained);
		}

		if (count != 0) {
			kal_uint64 t1, t2;

			LOGBT("%s WAITING... pRX->iPacket_r=0x%x, count=%zu\n",
					__func__, btsco.pRX->iPacket_r, count);
			t1 = sched_clock();
			BTCVSD_read_wait_queue_flag = 0;
			ret = wait_event_interruptible_timeout(BTCVSD_Read_Wait_Queue,
						BTCVSD_read_wait_queue_flag,
						nsecs_to_jiffies(read_timeout_limit));
			t2 = sched_clock();
			t2 = t2 - t1; /* in ns (10^9) */

			LOGBT("%s(), WAKEUP...wait event interrupt, ret = %d, BTCVSD_read_wait_queue_flag = %d\n",
			      __func__,
			      ret,
			      BTCVSD_read_wait_queue_flag);

			if (t2 > read_timeout_limit) {
				pr_warn("%s timeout, %llu ns, timeout_limit %llu, ret %d, flag %d\n",
					__func__,
					t2, read_timeout_limit,
					ret,
					BTCVSD_read_wait_queue_flag);
			}

			if (ret < 0) {
				/* error, -ERESTARTSYS if it was interrupted by a signal */
				pr_err("%s(), error, trial left %d, read_count %zd\n",
				       __func__,
				       max_timeout_trial,
				       read_count);

				rx_timeout = true;
				return read_count;
			} else if (ret == 0) {
				/* conidtion is false after timeout */
				max_timeout_trial--;
				pr_err("%s(), error, timeout, condition is false, trial left %d, read_count %zd\n",
				       __func__,
				       max_timeout_trial,
				       read_count);

				if (max_timeout_trial <= 0) {
					rx_timeout = true;
					return read_count;
				}
			} else if (ret == 1) {
				/* condition is true after timeout */
				LOGBT("%s(), timeout, condition is true\n", __func__);
			} else {
				LOGBT("%s(), condition is true before timeout\n", __func__);
			}
		}
	}

    /* Save current timestamp & buffer time in BT_RX_timestamp and BT_RX_bufdata_equivalent_time */
	BT_RX_timestamp = sched_clock();
	BT_RX_bufdata_equivalent_time = (btsco.pRX->iPacket_w - btsco.pRX->iPacket_r) * (SCO_RX_PLC_SIZE)
		* 16 * 1000  / 2 / 64;
	BT_RX_bufdata_equivalent_time += read_count * SCO_RX_PLC_SIZE * 16 * 1000
		/ (SCO_RX_PLC_SIZE+BTSCO_CVSD_PACKET_VALID_SIZE) / 2 / 64;
	BT_RX_bufdata_equivalent_time *= 1000;  /* return equivalent time(us) to data count */

	pr_warn("[JH] BT_RX_timestamp:%llu,BT_RX_bufdata_equivalent_time:%llu, iPacket_w:%d, iPacket_r:%d ",
		BT_RX_timestamp, BT_RX_bufdata_equivalent_time, btsco.pRX->iPacket_w, btsco.pRX->iPacket_r);

	LOGBT("AudDrv_btcvsd_read read_count = %zd,read_timeout_limit=%llu\n", read_count, read_timeout_limit);
	return read_count;
}

ssize_t AudDrv_btcvsd_write(const char __user *data, size_t count)
{
	int written_size = count , ret = 0, copy_size = 0, BTSCOTX_WriteIdx;
	unsigned long flags;
	char *data_w_ptr = (char *)data;
	kal_uint64 write_timeout_limit;
	int max_timeout_trial = 2;

	if ((btsco.pTX == NULL) || (btsco.pTX->u4BufferSize == 0)) {
		pr_debug("AudDrv_btcvsd_write btsco.pTX == NULL || (btsco.pTX->u4BufferSize == 0 !!!\n");
		msleep(60);
		return written_size;
	}

	/* ns */
	write_timeout_limit = ((kal_uint64)SCO_TX_PACKER_BUF_NUM*SCO_TX_ENCODE_SIZE*16*1000000000)/2/2/64000;

    /* Save current timestamp & buffer time in BT_TX_timestamp and BT_TX_bufdata_equivalent_time */
	BT_TX_timestamp = sched_clock();
	BT_TX_bufdata_equivalent_time = (btsco.pTX->iPacket_w - btsco.pTX->iPacket_r) * (SCO_TX_ENCODE_SIZE)
		* 16 * 1000  / 2 / 64;
	BT_TX_bufdata_equivalent_time *= 1000; /* return equivalent time(us) to data count */
	LOGBT("BT_TX_timestamp:%llu,BT_TX_bufdata_equivalent_time:%llu, iPacket_w:%d, iPacket_r:%d ",
		BT_TX_timestamp, BT_TX_bufdata_equivalent_time, btsco.pTX->iPacket_w, btsco.pTX->iPacket_r);

	while (count) {
		LOGBT("%s btsco.pTX->iPacket_w=%d, btsco.pTX->iPacket_r=%d\n",
				__func__, btsco.pTX->iPacket_w, btsco.pTX->iPacket_r);
		spin_lock_irqsave(&auddrv_btcvsd_tx_lock, flags);
		/*	free space of TX packet buffer */
		copy_size = btsco.pTX->u4BufferSize - (btsco.pTX->iPacket_w - btsco.pTX->iPacket_r)
		* SCO_TX_ENCODE_SIZE;
		spin_unlock_irqrestore(&auddrv_btcvsd_tx_lock, flags);

		/* count must be multiple of SCO_TX_ENCODE_SIZE */
		if (count % SCO_TX_ENCODE_SIZE != 0 ||
		    copy_size % SCO_TX_ENCODE_SIZE != 0) {
			pr_err("%s(), count %zu or copy_size %d is not multiple of SCO_TX_ENCODE_SIZE\n",
			       __func__, count, copy_size);

			count -= count % SCO_TX_ENCODE_SIZE;
			copy_size -= copy_size % SCO_TX_ENCODE_SIZE;
		}

		if (count <= (kal_uint32) copy_size)
			copy_size = count;

		LOGBT("AudDrv_btcvsd_write count=%zd, copy_size=%d\n", count, copy_size);

		if (copy_size != 0) {
			spin_lock_irqsave(&auddrv_btcvsd_tx_lock, flags);
			BTSCOTX_WriteIdx = (btsco.pTX->iPacket_w & SCO_TX_PACKET_MASK) * SCO_TX_ENCODE_SIZE;
			spin_unlock_irqrestore(&auddrv_btcvsd_tx_lock, flags);

			if (BTSCOTX_WriteIdx + copy_size < btsco.pTX->u4BufferSize) /* copy once */ {

				if (!access_ok(VERIFY_READ, data_w_ptr, copy_size)) {
					pr_debug("%s 0ptr invalid data_w_ptr=%lx, size=%d\n",
							__func__, (unsigned long)data_w_ptr, copy_size);
					pr_debug("%s u4BufferSize=%d, BTSCOTX_WriteIdx=%d\n",
							__func__, btsco.pTX->u4BufferSize, BTSCOTX_WriteIdx);
				} else {
					LOGBT("mcmcpy PacketBuf+BTSCOTX_WriteIdx=%lx data_w_ptr=%p copy_size=%d\n",
							(unsigned long)(btsco.pTX->PacketBuf+BTSCOTX_WriteIdx),
							data_w_ptr, copy_size);
					if (copy_from_user(btsco.pTX->PacketBuf + BTSCOTX_WriteIdx,
							   data_w_ptr,
							   copy_size)) {
						pr_debug("AudDrv_btcvsd_write Fail copy_from_user\n");
						return -1;
					}
				}

				spin_lock_irqsave(&auddrv_btcvsd_tx_lock, flags);
				btsco.pTX->iPacket_w += copy_size / SCO_TX_ENCODE_SIZE;
				spin_unlock_irqrestore(&auddrv_btcvsd_tx_lock, flags);
				data_w_ptr += copy_size;
				count -= copy_size;
				LOGBT("%s finish1, copy_size:%d, pTX->iPacket_w:%d, pTX->iPacket_r=%d, count=%zd\r\n",
						__func__, copy_size, btsco.pTX->iPacket_w,
						btsco.pTX->iPacket_r, count);
			} else  /* copy twice */ {
				kal_int32 size_1 = 0, size_2 = 0;

				size_1 = btsco.pTX->u4BufferSize - BTSCOTX_WriteIdx;
				size_2 = copy_size - size_1;
				LOGBT("%s size_1=%d, size_2=%d\n", __func__, size_1, size_2);
				if (!access_ok(VERIFY_READ, data_w_ptr, size_1)) {
					pr_debug("%s 1ptr invalid data_w_ptr=%lx, size_1=%d\n",
							__func__, (unsigned long)data_w_ptr, size_1);
					pr_debug("%s u4BufferSize=%d, BTSCOTX_WriteIdx=%d\n",
							__func__, btsco.pTX->u4BufferSize, BTSCOTX_WriteIdx);
				} else {
					LOGBT("mcmcpy PacketBuf+BTSCOTX_WriteIdx=%lx data_w_ptr=%p size_1=%d\n",
							(unsigned long)(btsco.pTX->PacketBuf+BTSCOTX_WriteIdx),
							data_w_ptr, size_1);
					if (copy_from_user(btsco.pTX->PacketBuf + BTSCOTX_WriteIdx,
							   data_w_ptr,
							   size_1)) {
						pr_debug("AudDrv_write Fail 1 copy_from_user\n");
						return -1;
					}
				}
				spin_lock_irqsave(&auddrv_btcvsd_tx_lock, flags);
				btsco.pTX->iPacket_w += size_1 / SCO_TX_ENCODE_SIZE;

				spin_unlock_irqrestore(&auddrv_btcvsd_tx_lock, flags);

				if (!access_ok(VERIFY_READ, data_w_ptr + size_1, size_2)) {
					pr_debug("AudDrv_btcvsd_write 2ptr invalid data_w_ptr=%lx, size_1=%d, size_2=%d\n",
							(unsigned long)data_w_ptr, size_1, size_2);
					pr_debug("AudDrv_btcvsd_write u4BufferSize=%d, pTX->iPacket_w=%d\n",
							btsco.pTX->u4BufferSize, btsco.pTX->iPacket_w);
				} else {
					LOGBT("PacketBuf+BTSCOTX_WriteIdx+size_1=%lx data_w_ptr+size_1=%p size_2=%x\n",
						(unsigned long)(btsco.pTX->PacketBuf+BTSCOTX_WriteIdx+size_1),
						data_w_ptr+size_1, size_2);
					if (copy_from_user(btsco.pTX->PacketBuf,
							   data_w_ptr + size_1,
							   size_2)) {
						pr_debug("AudDrv_btcvsd_write Fail 2 copy_from_user\n");
						return -1;
					}
				}

				spin_lock_irqsave(&auddrv_btcvsd_tx_lock, flags);

				btsco.pTX->iPacket_w += size_2 / SCO_TX_ENCODE_SIZE;
				spin_unlock_irqrestore(&auddrv_btcvsd_tx_lock, flags);
				count -= copy_size;
				data_w_ptr += copy_size;
				LOGBT("%s finish2, copy size:%d, pTX->iPacket_w=%d, pTX->iPacket_r=%d, count:%zd\r\n",
					__func__, copy_size, btsco.pTX->iPacket_w,
					btsco.pTX->iPacket_r, count);
			}
		}

		if (count != 0) {
			kal_uint64 t1, t2;

			LOGBT("%s WAITING...btsco.pTX->iPacket_w=%d, count=%zd\n",
				__func__, btsco.pTX->iPacket_w, count);
			t1 = sched_clock();
			BTCVSD_write_wait_queue_flag = 0;
			ret = wait_event_interruptible_timeout(BTCVSD_Write_Wait_Queue,
						BTCVSD_write_wait_queue_flag,
						nsecs_to_jiffies(write_timeout_limit));
			t2 = sched_clock();
			t2 = t2 - t1; /* in ns (10^9) */

			LOGBT("%s(), WAKEUP...wait event interrupt, ret = %d, BTCVSD_write_wait_queue_flag = %d\n",
			      __func__,
			      ret,
			      BTCVSD_write_wait_queue_flag);

			if (t2 > write_timeout_limit) {
				pr_warn("%s timeout, %llu ns, timeout_limit %llu, ret %d, flag %d\n",
					__func__,
					t2, write_timeout_limit,
					ret,
					BTCVSD_write_wait_queue_flag);
			}

			if (ret < 0) {
				/* error, -ERESTARTSYS if it was interrupted by a signal */
				pr_err("%s(), error, trial left %d, ret = %d, written_size %d\n",
				       __func__,
				       max_timeout_trial,
				       ret,
				       written_size);

				tx_timeout = true;
				return written_size;
			} else if (ret == 0) {
				/* conidtion is false after timeout */
				max_timeout_trial--;
				pr_warn("%s(), warn, timeout, condition is false, trial left %d, written_size %d\n",
					__func__,
					max_timeout_trial,
					written_size);

				if (max_timeout_trial <= 0) {
					tx_timeout = true;
					return written_size;
				}
			} else if (ret == 1) {
				/* condition is true after timeout */
				LOGBT("%s(), timeout, condition is true\n", __func__);
			} else {
				LOGBT("%s(), condition is true before timeout\n", __func__);
			}
		}
		/* here need to wait for interrupt handler */
	}
	LOGBT("AudDrv_btcvsd_write written_size = %d, write_timeout_limit=%llu\n", written_size, write_timeout_limit);
	return written_size;
}

void Set_BTCVSD_State(unsigned long arg)
{
	pr_warn("+%s, state=%ld\n", __func__, arg);

	if (arg == BT_SCO_TXSTATE_DIRECT_LOOPBACK) {
		btsco.uTXState = arg;
		btsco.uRXState = arg;
	} else if ((arg & 0x10) == 0) {	/*TX state*/
		btsco.uTXState = arg;
	} else {	/*RX state*/
		btsco.uRXState = arg;
	}

	if (btsco.uTXState == BT_SCO_TXSTATE_IDLE && btsco.uRXState == BT_SCO_RXSTATE_IDLE) {
		if (disableBTirq == 0) {
			disable_irq(btcvsd_irq_number);
			Disable_CVSD_Wakeup();
			disableBTirq = 1;
		}
	} else {
		if (disableBTirq == 1) {
			enable_irq(btcvsd_irq_number);
			Enable_CVSD_Wakeup();
			disableBTirq = 0;
		}
	}
}

bool btcvsd_rx_irq_received(void)
{
	return readFromBT_cnt;
}

bool btcvsd_rx_timeout(void)
{
	return rx_timeout;
}

void btcvsd_rx_reset_timeout(void)
{
	rx_timeout = false;
}

bool btcvsd_tx_timeout(void)
{
	return tx_timeout;
}

void btcvsd_tx_reset_timeout(void)
{
	tx_timeout = false;
}

unsigned long btcvsd_frame_to_bytes(struct snd_pcm_substream *substream,
				    unsigned long count)
{
	unsigned long bytes = count;
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (runtime->format == SNDRV_PCM_FORMAT_S32_LE ||
	    runtime->format == SNDRV_PCM_FORMAT_U32_LE)
		bytes = bytes << 2;
	else
		bytes = bytes << 1;

	if (runtime->channels == 2)
		bytes = bytes << 1;
	else if (runtime->channels == 4)
		bytes = bytes << 2;
	else if (runtime->channels != 1)
		bytes = bytes << 3;
	/* printk("%s bytes = %d count = %d\n",__func__,bytes,count); */
	return bytes;
}


unsigned long btcvsd_bytes_to_frame(struct snd_pcm_substream *substream,
				    unsigned long bytes)
{
	unsigned long count  = bytes;
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (runtime->format == SNDRV_PCM_FORMAT_S32_LE ||
	    runtime->format == SNDRV_PCM_FORMAT_U32_LE)
		count = count >> 2;
	else
		count = count >> 1;


	if (runtime->channels == 2)
		count = count >> 1;
	else if (runtime->channels == 4)
		count = count >> 2;
	else if (runtime->channels != 1)
		count = count >> 3;
	/* printk("%s bytes = %d count = %d\n",__func__,bytes,count); */
	return count;
}

void set_btcvsd_band(enum BT_SCO_BAND band)
{
	btsco.band = band;
}

enum BT_SCO_BAND get_btcvsd_band(void)
{
	return btsco.band;
}

/* write encoded mute data to bt sram */
void btcvsd_tx_clean_buffer(void)
{
	unsigned int i;
	enum BT_SCO_BAND band = get_btcvsd_band();

	pr_debug("%s(), band %d, num_valid_addr %u\n",
		 __func__,
		 band,
		 btsco.pTX->buffer_info.num_valid_addr);

	if (!btsco.pTX) {
		pr_err("%s(), btsco.pTX == NULL\n", __func__);
		return;
	}

	/* prepare encoded mute data */
	if (band == BT_SCO_NB)
		memset(btsco.pTX->TempPacketBuf, 170, BT_SCO_PACKET_180);
	else
		memset(btsco.pTX->TempPacketBuf, 65, BT_SCO_PACKET_180);


	/* write mute data to bt tx sram buffer */
	for (i = 0; i < btsco.pTX->buffer_info.num_valid_addr; i++) {
		kal_uint8 *pDst;

		pr_debug("%s(), clean addr 0x%lx\n",
			 __func__, btsco.pTX->buffer_info.addr_to_clean[i]);

		pDst = (kal_uint8 *)btsco.pTX->buffer_info.addr_to_clean[i];

		AudDrv_BTCVSD_DataTransfer(BT_SCO_DIRECT_ARM2BT,
					   btsco.pTX->TempPacketBuf,
					   pDst,
					   btsco.pTX->buffer_info.packet_length,
					   btsco.pTX->buffer_info.packet_number,
					   btsco.uTXState);
	}
}

