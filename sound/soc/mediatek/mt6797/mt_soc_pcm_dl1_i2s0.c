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
 *   mt_soc_pcm_i2s0.c
 *
 * Project:
 * --------
 *    Audio Driver Kernel Function
 *
 * Description:
 * ------------
 *   Audio i2s0 playback
 *
 * Author:
 * -------
 * Chipeng Chang
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
#include "AudDrv_Common.h"
#include "AudDrv_Def.h"
#include "AudDrv_Afe.h"
#include "AudDrv_Ana.h"
#include "AudDrv_Clk.h"
#include "AudDrv_Kernel.h"
#include "mt_soc_afe_control.h"
#include "mt_soc_digital_type.h"
#include "mt_soc_pcm_common.h"

static DEFINE_SPINLOCK(auddrv_I2S0_lock);
static AFE_MEM_CONTROL_T *pI2s0MemControl;

static struct device *mDev;

/*
 *    function implementation
 */

static int mtk_i2s0_probe(struct platform_device *pdev);
static int mtk_pcm_i2s0_close(struct snd_pcm_substream *substream);
static int mtk_asoc_pcm_i2s0_new(struct snd_soc_pcm_runtime *rtd);
static int mtk_afe_i2s0_probe(struct snd_soc_platform *platform);

int mtk_soc_always_hd = 0;
static int mi2s0_sidegen_control;
static int mi2s0_hdoutput_control;
static int mi2s0_extcodec_echoref_control;
static const char const *i2s0_SIDEGEN[] = {
	"Off", "On8000", "On16000", "On32000", "On44100", "On48000", "On96000", "On192000"};
static const char const *i2s0_HD_output[] = {"Off", "On"};
static const char const *i2s0_ExtCodec_EchoRef[] = {"Off", "MD1", "MD3"};

static const struct soc_enum Audio_i2s0_Enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(i2s0_SIDEGEN), i2s0_SIDEGEN),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(i2s0_HD_output), i2s0_HD_output),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(i2s0_ExtCodec_EchoRef), i2s0_ExtCodec_EchoRef),
};

static int Audio_i2s0_SideGen_Get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("Audio_AmpR_Get = %d\n", mi2s0_sidegen_control);
	ucontrol->value.integer.value[0] = mi2s0_sidegen_control;
	return 0;
}

static int Audio_i2s0_SideGen_Set(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	uint32 u32AudioI2S = 0;	/* REG448 = 0, REG44C = 0; */
	uint32 Audio_I2S_Dac = 0;
	static int samplerate;

	if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(i2s0_SIDEGEN)) {
		pr_err("return -EINVAL\n");
		return -EINVAL;
	}
	mi2s0_sidegen_control = ucontrol->value.integer.value[0];

	pr_debug("%s(), mi2s0_sidegen = %d, samplerate = %d, mi2s0_hdoutput = %d, mi2s0_extcodec_echoref = %d, mtk_soc_always_hd = %d\n",
		 __func__,
		 mi2s0_sidegen_control,
		 samplerate,
		 mi2s0_hdoutput_control,
		 mi2s0_extcodec_echoref_control,
		 mtk_soc_always_hd);

	AudDrv_Clk_On();

	if (mi2s0_sidegen_control) {
		/* Phone call echo ref, speaker mode connection*/
		switch (mi2s0_extcodec_echoref_control) {
		case 1:
			/* MD1 connection */
			SetConnection(Soc_Aud_InterCon_Connection,
				      Soc_Aud_InterConnectionInput_I14, Soc_Aud_InterConnectionOutput_O00);
			SetConnection(Soc_Aud_InterCon_Connection,
				      Soc_Aud_InterConnectionInput_I14, Soc_Aud_InterConnectionOutput_O01);

			SetConnection(Soc_Aud_InterCon_Connection,
				      Soc_Aud_InterConnectionInput_I01,
				      Soc_Aud_InterConnectionOutput_O24);
			break;
		case 2:
			/* MD3 connection */
			SetConnection(Soc_Aud_InterCon_Connection,
				      Soc_Aud_InterConnectionInput_I09, Soc_Aud_InterConnectionOutput_O00);
			SetConnection(Soc_Aud_InterCon_Connection,
				      Soc_Aud_InterConnectionInput_I09, Soc_Aud_InterConnectionOutput_O01);

			SetConnection(Soc_Aud_InterCon_Connection,
				      Soc_Aud_InterConnectionInput_I01,
				      Soc_Aud_InterConnectionOutput_O27);
			break;
		default:
			break;
		}

		switch (mi2s0_sidegen_control) {
		case 1:
			samplerate = 8000;
			break;
		case 2:
			samplerate = 16000;
			break;
		case 3:
			samplerate = 32000;
			break;
		case 4:
			samplerate = 44100;
			break;
		case 5:
			samplerate = 48000;
			break;
		case 6:
			samplerate = 96000;
			break;
		case 7:
			samplerate = 192000;
			break;
		default:
			pr_err("%s(), sidegen_control error, return -EINVAL\n", __func__);
			return -EINVAL;
		}

		AudDrv_Clk_On();
		if (!mtk_soc_always_hd)
			EnableALLbySampleRate(samplerate);

		if (GetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_2) == false) {
			SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_2, true);
		} else {
			pr_debug
			    ("%s(), mi2s0_sidegen_control=%d, write AFE_I2S_CON (0x%x), AFE_I2S_CON3(0x%x)\n",
			     __func__, mi2s0_sidegen_control, Audio_I2S_Dac, u32AudioI2S);
			SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_2, true);
			if (mi2s0_extcodec_echoref_control > 0)
				Afe_Set_Reg(AFE_I2S_CON, 0x0, 0x1);
		}
		Afe_Set_Reg(AFE_I2S_CON3, 0x0, 0x1);
		udelay(20);

		/* i2s0 i2s3 4pin i2s setting */
		Afe_Set_Reg(AUDIO_TOP_CON1, 0x1 << 4,  0x1 << 4); /* I2S0 clock-gated */
		Afe_Set_Reg(AUDIO_TOP_CON1, 0x1 << 7,  0x1 << 7); /* I2S3 clock-gated */
		/* I2S0 I2S3 clock-gated */

		if (mi2s0_extcodec_echoref_control > 0) {
			/* I2S0 Input Control */
			Audio_I2S_Dac = 0;
			SetCLkMclk(Soc_Aud_I2S0, samplerate);
			SetSampleRate(Soc_Aud_Digital_Block_MEM_I2S, samplerate);

			Audio_I2S_Dac |= (Soc_Aud_LR_SWAP_NO_SWAP << 31);

			if (mi2s0_hdoutput_control == true)
				Audio_I2S_Dac |= Soc_Aud_LOW_JITTER_CLOCK << 12;	/* Low jitter mode */
			else
				Audio_I2S_Dac |= Soc_Aud_NORMAL_CLOCK << 12;	/* Low jitter mode */

			Audio_I2S_Dac |= (Soc_Aud_I2S_IN_PAD_SEL_I2S_IN_FROM_IO_MUX << 28);	/* I2S in from io_mux */
			Audio_I2S_Dac |= (Soc_Aud_INV_LRCK_NO_INVERSE << 5);
			Audio_I2S_Dac |= (Soc_Aud_I2S_FORMAT_I2S << 3);
			Audio_I2S_Dac |= (Soc_Aud_I2S_WLEN_WLEN_32BITS << 1);
		}

		u32AudioI2S = SampleRateTransform(samplerate, Soc_Aud_Digital_Block_I2S_OUT_2) << 8;
		u32AudioI2S |= Soc_Aud_I2S_FORMAT_I2S << 3;	/* us3 I2s format */
		u32AudioI2S |= Soc_Aud_I2S_WLEN_WLEN_32BITS << 1;	/* 32 BITS */

		if (mi2s0_hdoutput_control == true)
			u32AudioI2S |= Soc_Aud_LOW_JITTER_CLOCK << 12;	/* Low jitter mode */
		else
			u32AudioI2S |= Soc_Aud_NORMAL_CLOCK << 12;	/* Low jitter mode */

		/* start I2S DAC out */

		if (mi2s0_extcodec_echoref_control > 0)
			Afe_Set_Reg(AFE_I2S_CON, Audio_I2S_Dac, MASK_ALL);	/* set I2S0 configuration */

		Afe_Set_Reg(AFE_I2S_CON3, u32AudioI2S, AFE_MASK_ALL);	/* set I2S3 configuration */

		Afe_Set_Reg(AUDIO_TOP_CON1, 0 << 4,  0x1 << 4); /* Clear I2S0 clock-gated */
		Afe_Set_Reg(AUDIO_TOP_CON1, 0 << 7,  0x1 << 7); /* Clear I2S3 clock-gated */
		/* Clear I2S0 I2S3 clock-gated */

		Afe_Set_Reg(AFE_I2S_CON, 0x1, 0x1);	/* Enable I2S0 */
		Afe_Set_Reg(AFE_I2S_CON3, 0x1, 0x1);	/* Enable I2S3 */
		EnableAfe(true);

	} else {
		if (mi2s0_extcodec_echoref_control > 0) {
			SetConnection(Soc_Aud_InterCon_DisConnect,
				      Soc_Aud_InterConnectionInput_I01,
				      Soc_Aud_InterConnectionOutput_O27);
			SetConnection(Soc_Aud_InterCon_DisConnect,
				      Soc_Aud_InterConnectionInput_I01,
				      Soc_Aud_InterConnectionOutput_O24);
		}

		SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_2, false);

		if (GetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_2) == false) {
			Afe_Set_Reg(AFE_I2S_CON3, 0x0, 0x1);	/* Disable I2S3 */
			if (mi2s0_extcodec_echoref_control > 0)
				Afe_Set_Reg(AFE_I2S_CON, 0x0, 0x1);	/* Disable I2S0 */

			udelay(20);
#if 0 /* avoding clock gating in FM or other case */
			Afe_Set_Reg(AUDIO_TOP_CON1, 0x1 << 4, 0x1 << 4);	/* I2S0 clock-gated */
			Afe_Set_Reg(AUDIO_TOP_CON1, 0x1 << 7, 0x1 << 7);	/* I2S3 clock-gated */
#endif
			SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I14,
				      Soc_Aud_InterConnectionOutput_O00);
			SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I14,
				      Soc_Aud_InterConnectionOutput_O01);
			SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I09,
				      Soc_Aud_InterConnectionOutput_O00);
			SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I09,
				      Soc_Aud_InterConnectionOutput_O01);
			EnableAfe(false);
		}
		if (!mtk_soc_always_hd)
			DisableALLbySampleRate(samplerate);
		AudDrv_Clk_Off();
	}
	AudDrv_Clk_Off();
	return 0;
}

static int audio_always_hd_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s(), mtk_soc_always_hd %d\n", __func__, mtk_soc_always_hd);
	ucontrol->value.integer.value[0] = mtk_soc_always_hd;
	return 0;
}

static int audio_always_hd_set(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s(), mtk_soc_always_hd %d\n", __func__, mtk_soc_always_hd);
	if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(i2s0_HD_output)) {
		pr_err("return -EINVAL\n");
		return -EINVAL;
	}

	mtk_soc_always_hd = ucontrol->value.integer.value[0];
	return 0;
}

static int Audio_i2s0_hdoutput_Get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("Audio_i2s0_hdoutput_Get = %d\n", mi2s0_hdoutput_control);
	ucontrol->value.integer.value[0] = mi2s0_hdoutput_control;
	return 0;
}

static int Audio_i2s0_hdoutput_Set(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("+%s()\n", __func__);
	if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(i2s0_HD_output)) {
		pr_err("return -EINVAL\n");
		return -EINVAL;
	}

	mi2s0_hdoutput_control = ucontrol->value.integer.value[0];
#if 0
	if (mi2s0_hdoutput_control) {
		/* set APLL clock setting */
		EnableApll1(true);
		EnableApll2(true);
		EnableI2SDivPower(AUDIO_APLL1_DIV0, true);
		EnableI2SDivPower(AUDIO_APLL2_DIV0, true);
	} else {
		/* set APLL clock setting */
		EnableApll1(false);
		EnableApll2(false);
		EnableI2SDivPower(AUDIO_APLL1_DIV0, false);
		EnableI2SDivPower(AUDIO_APLL2_DIV0, false);
	}
#endif
	return 0;
}

static int Audio_i2s0_ExtCodec_EchoRef_Get(struct snd_kcontrol *kcontrol,
					   struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("Audio_i2s0_ExtCodec_EchoRef_Get = %d\n", mi2s0_extcodec_echoref_control);
	ucontrol->value.integer.value[0] = mi2s0_extcodec_echoref_control;
	return 0;
}

static int Audio_i2s0_ExtCodec_EchoRef_Set(struct snd_kcontrol *kcontrol,
					   struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(i2s0_ExtCodec_EchoRef)) {
		pr_err("return -EINVAL\n");
		return -EINVAL;
	}
	mi2s0_extcodec_echoref_control = ucontrol->value.integer.value[0];
	return 0;
}

static const struct snd_kcontrol_new Audio_snd_i2s0_controls[] = {
	SOC_ENUM_EXT("Audio_i2s0_SideGen_Switch",
		     Audio_i2s0_Enum[0], Audio_i2s0_SideGen_Get, Audio_i2s0_SideGen_Set),
	SOC_ENUM_EXT("Audio_i2s0_hd_Switch",
		     Audio_i2s0_Enum[1], Audio_i2s0_hdoutput_Get, Audio_i2s0_hdoutput_Set),
	SOC_ENUM_EXT("Audio_always_hd_Switch",
		     Audio_i2s0_Enum[1], audio_always_hd_get, audio_always_hd_set),
	SOC_ENUM_EXT("Audio_ExtCodec_EchoRef_Switch",
		     Audio_i2s0_Enum[2], Audio_i2s0_ExtCodec_EchoRef_Get,
		     Audio_i2s0_ExtCodec_EchoRef_Set),
};

static struct snd_pcm_hardware mtk_i2s0_hardware = {
	.info = (SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_RESUME | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = Dl1_MAX_BUFFER_SIZE,
	.rates = SOC_HIGH_USE_RATE,
	.rate_min = SOC_HIGH_USE_RATE_MIN,
	.rate_max = SOC_HIGH_USE_RATE_MAX,
	.channels_min = SOC_NORMAL_USE_CHANNELS_MIN,
	.channels_max = SOC_NORMAL_USE_CHANNELS_MAX,
	.buffer_bytes_max = Dl1_MAX_BUFFER_SIZE,
	.period_bytes_max = Dl1_MAX_BUFFER_SIZE,
	.periods_min = SOC_NORMAL_USE_PERIODS_MIN,
	.periods_max = SOC_NORMAL_USE_PERIODS_MAX,
	.fifo_size = 0,
};

static int mtk_pcm_i2s0_stop(struct snd_pcm_substream *substream)
{
	AFE_BLOCK_T *Afe_Block = &(pI2s0MemControl->rBlock);

	pr_debug("mtk_pcm_i2s0_stop\n");
	irq_remove_user(substream, Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE);

	/* here start digital part */
	SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I05,
		      Soc_Aud_InterConnectionOutput_O00);
	SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I06,
		      Soc_Aud_InterConnectionOutput_O01);
	SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL1, false);

	/* stop I2S */
	Afe_Set_Reg(AFE_I2S_CON3, 0x0, 0x1);

	EnableAfe(false);

	/* clean audio hardware buffer */
	memset_io(Afe_Block->pucVirtBufAddr, 0, Afe_Block->u4BufferSize);
	RemoveMemifSubStream(Soc_Aud_Digital_Block_MEM_DL1, substream);
	AudDrv_Clk_Off();

	return 0;
}

static snd_pcm_uframes_t mtk_pcm_i2s0_pointer(struct snd_pcm_substream *substream)
{
	kal_int32 HW_memory_index = 0;
	kal_int32 HW_Cur_ReadIdx = 0;
	kal_uint32 Frameidx = 0;
	kal_int32 Afe_consumed_bytes = 0;
	AFE_BLOCK_T *Afe_Block = &pI2s0MemControl->rBlock;
	/* struct snd_pcm_runtime *runtime = substream->runtime; */
	PRINTK_AUD_DL1(" %s Afe_Block->u4DMAReadIdx = 0x%x\n", __func__, Afe_Block->u4DMAReadIdx);

	Auddrv_Dl1_Spinlock_lock();

	/* get total bytes to copy */
	/* Frameidx = audio_bytes_to_frame(substream , Afe_Block->u4DMAReadIdx); */
	/* return Frameidx; */

	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL1) == true) {
		HW_Cur_ReadIdx = Afe_Get_Reg(AFE_DL1_CUR);
		if (HW_Cur_ReadIdx == 0) {
			PRINTK_AUDDRV("[Auddrv] HW_Cur_ReadIdx ==0\n");
			HW_Cur_ReadIdx = Afe_Block->pucPhysBufAddr;
		}

		HW_memory_index = (HW_Cur_ReadIdx - Afe_Block->pucPhysBufAddr);
		if (HW_memory_index >= Afe_Block->u4DMAReadIdx)
			Afe_consumed_bytes = HW_memory_index - Afe_Block->u4DMAReadIdx;
		else
			Afe_consumed_bytes =
			    Afe_Block->u4BufferSize + HW_memory_index - Afe_Block->u4DMAReadIdx;

		Afe_consumed_bytes = Align64ByteSize(Afe_consumed_bytes);

		Afe_Block->u4DataRemained -= Afe_consumed_bytes;
		Afe_Block->u4DMAReadIdx += Afe_consumed_bytes;
		Afe_Block->u4DMAReadIdx %= Afe_Block->u4BufferSize;
		PRINTK_AUD_DL1
		    ("[Auddrv] HW_Cur_ReadIdx =0x%x HW_memory_index = 0x%x Afe_consumed_bytes  = 0x%x\n",
		     HW_Cur_ReadIdx, HW_memory_index, Afe_consumed_bytes);
		Auddrv_Dl1_Spinlock_unlock();
		Frameidx = audio_bytes_to_frame(substream, Afe_Block->u4DMAReadIdx);
	} else {
		Frameidx = audio_bytes_to_frame(substream, Afe_Block->u4DMAReadIdx);
		Auddrv_Dl1_Spinlock_unlock();
	}
	return Frameidx;
}


static int mtk_pcm_i2s0_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *hw_params)
{
	int ret = 0;

	PRINTK_AUDDRV("mtk_pcm_hw_params\n");

	/* runtime->dma_bytes has to be set manually to allow mmap */
	substream->runtime->dma_bytes = params_buffer_bytes(hw_params);

	/* here to allcoate sram to hardware --------------------------- */
	AudDrv_Allocate_mem_Buffer(mDev, Soc_Aud_Digital_Block_MEM_DL1,
				   substream->runtime->dma_bytes);
	/* substream->runtime->dma_bytes = AFE_INTERNAL_SRAM_SIZE; */
	substream->runtime->dma_area = (unsigned char *)Get_Afe_SramBase_Pointer();
	substream->runtime->dma_addr = AFE_INTERNAL_SRAM_PHY_BASE;
	SetHighAddr(Soc_Aud_Digital_Block_MEM_DL1, false);
	AudDrv_Emi_Clk_On();

	/* ------------------------------------------------------- */
	PRINTK_AUDDRV("1 dma_bytes = %zu dma_area = %p dma_addr = 0x%lx\n",
		      substream->runtime->dma_bytes, substream->runtime->dma_area,
		      (long)substream->runtime->dma_addr);

	return ret;
}

static int mtk_pcm_i2s0_hw_free(struct snd_pcm_substream *substream)
{
	AudDrv_Emi_Clk_Off();
	return 0;
}

static struct snd_pcm_hw_constraint_list constraints_sample_rates = {
	.count = ARRAY_SIZE(soc_high_supported_sample_rates),
	.list = soc_high_supported_sample_rates,
	.mask = 0,
};

static unsigned int mPlaybackDramState;
static int mtk_pcm_i2s0_open(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;

	AfeControlSramLock();
	if (GetSramState() == SRAM_STATE_FREE) {
		mtk_i2s0_hardware.buffer_bytes_max = GetPLaybackSramFullSize();
		mPlaybackDramState = SRAM_STATE_PLAYBACKFULL;
		SetSramState(mPlaybackDramState);
	} else {
		mtk_i2s0_hardware.buffer_bytes_max = GetPLaybackSramPartial();
		mPlaybackDramState = SRAM_STATE_PLAYBACKPARTIAL;
		SetSramState(mPlaybackDramState);
	}
	AfeControlSramUnLock();
	runtime->hw = mtk_i2s0_hardware;

	pr_debug("mtk_pcm_i2s0_open\n");

	AudDrv_Clk_On();
	memcpy((void *)(&(runtime->hw)), (void *)&mtk_i2s0_hardware,
	       sizeof(struct snd_pcm_hardware));
	pI2s0MemControl = Get_Mem_ControlT(Soc_Aud_Digital_Block_MEM_DL1);


	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
					 &constraints_sample_rates);
	ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);

	if (ret < 0)
		pr_warn("snd_pcm_hw_constraint_integer failed\n");

	/* print for hw pcm information */
	pr_debug("mtk_pcm_i2s0_open runtime rate = %d channels = %d substream->pcm->device = %d\n",
		 runtime->rate, runtime->channels, substream->pcm->device);

	 if (ret < 0) {
		pr_err("mtk_pcm_i2s0_close\n");
		mtk_pcm_i2s0_close(substream);
		return ret;
	}
	pr_debug("mtk_pcm_i2s0_open return\n");
	AudDrv_Clk_Off();
	return 0;
}

static int mtk_pcm_i2s0_close(struct snd_pcm_substream *substream)
{
	pr_debug("%s\n", __func__);
	AfeControlSramLock();
	ClearSramState(mPlaybackDramState);
	mPlaybackDramState = GetSramState();
	AfeControlSramUnLock();
	return 0;
}

static int mtk_pcm_i2s0_prepare(struct snd_pcm_substream *substream)
{
	return 0;
}

static int mtk_pcm_i2s0_start(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	uint32 u32AudioI2S = 0;

	AudDrv_Clk_On();
	SetMemifSubStream(Soc_Aud_Digital_Block_MEM_DL1, substream);
	if (runtime->format == SNDRV_PCM_FORMAT_S32_LE
	    || runtime->format == SNDRV_PCM_FORMAT_U32_LE) {
		SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_DL1,
					     AFE_WLEN_32_BIT_ALIGN_8BIT_0_24BIT_DATA);
	} else {
		SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_DL1, AFE_WLEN_16_BIT);
	}

	SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_16BIT, Soc_Aud_InterConnectionOutput_O00);
	SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_16BIT, Soc_Aud_InterConnectionOutput_O01);

	/* here start digital part */
	SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I05,
		      Soc_Aud_InterConnectionOutput_O00);
	SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I06,
		      Soc_Aud_InterConnectionOutput_O01);


	u32AudioI2S = SampleRateTransform(runtime->rate, Soc_Aud_Digital_Block_I2S_OUT_2) << 8;
	u32AudioI2S |= Soc_Aud_I2S_FORMAT_I2S << 3;	/* us3 I2s format */
	u32AudioI2S |= Soc_Aud_I2S_WLEN_WLEN_32BITS << 1;	/* 32 BITS */

	if (mi2s0_hdoutput_control == true)
		u32AudioI2S |= Soc_Aud_LOW_JITTER_CLOCK << 12;	/* Low jitter mode */

	pr_debug(" u32AudioI2S= 0x%x\n", u32AudioI2S);
	Afe_Set_Reg(AFE_I2S_CON3, u32AudioI2S | 1, AFE_MASK_ALL);

	SetSampleRate(Soc_Aud_Digital_Block_MEM_DL1, runtime->rate);
	SetChannels(Soc_Aud_Digital_Block_MEM_DL1, runtime->channels);
	SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL1, true);

	/* here to set interrupt */
	irq_add_user(substream,
		     Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE,
		     substream->runtime->rate,
		     substream->runtime->period_size);

	EnableAfe(true);

	return 0;
}

static int mtk_pcm_i2s0_trigger(struct snd_pcm_substream *substream, int cmd)
{
	pr_debug("mtk_pcm_i2s0_trigger cmd = %d\n", cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		return mtk_pcm_i2s0_start(substream);
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		return mtk_pcm_i2s0_stop(substream);
	}
	return -EINVAL;
}

static int mtk_pcm_i2s0_copy(struct snd_pcm_substream *substream,
			     int channel, snd_pcm_uframes_t pos,
			     void __user *dst, snd_pcm_uframes_t count)
{
	AFE_BLOCK_T *Afe_Block = NULL;
	int copy_size = 0, Afe_WriteIdx_tmp;
	unsigned long flags;
	char *data_w_ptr = (char *)dst;
	/* struct snd_pcm_runtime *runtime = substream->runtime; */
	PRINTK_AUD_DL1("%s  pos = 0x%x count = 0x%x\n ", __func__, (unsigned int)pos,
		       (unsigned int)count);

	count = audio_frame_to_bytes(substream, count);

	/* check which memif nned to be write */
	Afe_Block = &pI2s0MemControl->rBlock;

	/* handle for buffer management */
	PRINTK_AUD_DL1(" WriteIdx=0x%x, ReadIdx=0x%x, DataRemained=0x%x\n",
		       Afe_Block->u4WriteIdx, Afe_Block->u4DMAReadIdx, Afe_Block->u4DataRemained);
	if (Afe_Block->u4BufferSize == 0) {
		pr_err(" u4BufferSize=0 Error");
		return 0;
	}

	AudDrv_checkDLISRStatus();

	spin_lock_irqsave(&auddrv_I2S0_lock, flags);
	copy_size = Afe_Block->u4BufferSize - Afe_Block->u4DataRemained;	/* free space of the buffer */
	spin_unlock_irqrestore(&auddrv_I2S0_lock, flags);
	if (count <= copy_size) {
		if (copy_size < 0)
			copy_size = 0;
		else
			copy_size = count;
	}

	copy_size = Align64ByteSize(copy_size);
	PRINTK_AUD_DL1("copy_size=0x%x, count=0x%x\n", (unsigned int)copy_size,
		       (unsigned int)count);

	if (copy_size != 0) {
		spin_lock_irqsave(&auddrv_I2S0_lock, flags);
		Afe_WriteIdx_tmp = Afe_Block->u4WriteIdx;
		spin_unlock_irqrestore(&auddrv_I2S0_lock, flags);

		if (Afe_WriteIdx_tmp + copy_size < Afe_Block->u4BufferSize) {	/* copy once */
			if (!access_ok(VERIFY_READ, data_w_ptr, copy_size)) {
				PRINTK_AUDDRV("0ptr invalid data_w_ptr=%p, size=%d",
					      data_w_ptr, copy_size);
				PRINTK_AUDDRV(" u4BufferSize=%d, u4DataRemained=%d",
					      Afe_Block->u4BufferSize, Afe_Block->u4DataRemained);
			} else {

				PRINTK_AUD_DL1
				    ("memcpy WriteIdx= %p data_w_ptr = %p copy_size = 0x%x\n",
				     Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp, data_w_ptr,
				     copy_size);
				if (copy_from_user
				    ((Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp), data_w_ptr,
				     copy_size)) {
					PRINTK_AUDDRV(" Fail copy from user\n");
					return -1;
				}
			}

			spin_lock_irqsave(&auddrv_I2S0_lock, flags);
			Afe_Block->u4DataRemained += copy_size;
			Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + copy_size;
			Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
			spin_unlock_irqrestore(&auddrv_I2S0_lock, flags);
			data_w_ptr += copy_size;
			count -= copy_size;

			PRINTK_AUD_DL1
			    (" finish1, copy_size:%x, WriteIdx:%x, ReadIdx=%x, Remained:%x, count=%x \r\n",
			     copy_size, Afe_Block->u4WriteIdx, Afe_Block->u4DMAReadIdx,
			     Afe_Block->u4DataRemained, (unsigned int)count);

		} else {	/* copy twice */
			kal_uint32 size_1 = 0, size_2 = 0;

			size_1 = Align64ByteSize((Afe_Block->u4BufferSize - Afe_WriteIdx_tmp));
			size_2 = Align64ByteSize((copy_size - size_1));
			PRINTK_AUD_DL1("size_1=0x%x, size_2=0x%x\n", size_1, size_2);
			if (!access_ok(VERIFY_READ, data_w_ptr, size_1)) {
				pr_debug(" 1ptr invalid data_w_ptr=%p, size_1=%d",
					 data_w_ptr, size_1);
				pr_debug(" u4BufferSize=%d, u4DataRemained=%d",
					 Afe_Block->u4BufferSize, Afe_Block->u4DataRemained);
			} else {

				PRINTK_AUD_DL1("mcmcpy WriteIdx= %p data_w_ptr = %p size_1 = %x\n",
					       Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp,
					       data_w_ptr, size_1);
				if ((copy_from_user
				     ((Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp), data_w_ptr,
				      size_1))) {
					PRINTK_AUDDRV(" Fail 1 copy from user");
					return -1;
				}
			}
			spin_lock_irqsave(&auddrv_I2S0_lock, flags);
			Afe_Block->u4DataRemained += size_1;
			Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + size_1;
			Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
			Afe_WriteIdx_tmp = Afe_Block->u4WriteIdx;
			spin_unlock_irqrestore(&auddrv_I2S0_lock, flags);

			if (!access_ok(VERIFY_READ, data_w_ptr + size_1, size_2)) {
				PRINTK_AUDDRV("2ptr invalid data_w_ptr=%p, size_1=%d, size_2=%d",
					      data_w_ptr, size_1, size_2);
				PRINTK_AUDDRV("u4BufferSize=%d, u4DataRemained=%d",
					      Afe_Block->u4BufferSize, Afe_Block->u4DataRemained);
			} else {

				PRINTK_AUD_DL1
				    ("mcmcpy WriteIdx= %p data_w_ptr+size_1 = %p size_2 = %x\n",
				     Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp,
				     data_w_ptr + size_1, size_2);
				if ((copy_from_user
				     ((Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp),
				      (data_w_ptr + size_1), size_2))) {
					PRINTK_AUDDRV("AudDrv_write Fail 2  copy from user");
					return -1;
				}
			}
			spin_lock_irqsave(&auddrv_I2S0_lock, flags);

			Afe_Block->u4DataRemained += size_2;
			Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + size_2;
			Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
			spin_unlock_irqrestore(&auddrv_I2S0_lock, flags);
			count -= copy_size;
			data_w_ptr += copy_size;

			PRINTK_AUD_DL1
			    (" finish2, copy size:%x, WriteIdx:%x,ReadIdx=%x Remained:%x \r\n",
			     copy_size, Afe_Block->u4WriteIdx, Afe_Block->u4DMAReadIdx,
			     Afe_Block->u4DataRemained);
		}
	}
	return 0;
	PRINTK_AUD_DL1("pcm_copy return\n");

}

static int mtk_pcm_i2s0_silence(struct snd_pcm_substream *substream,
				int channel, snd_pcm_uframes_t pos, snd_pcm_uframes_t count)
{
	pr_debug("%s\n", __func__);
	return 0;		/* do nothing */
}

static void *dummy_page[2];

static struct page *mtk_i2s0_pcm_page(struct snd_pcm_substream *substream, unsigned long offset)
{
	pr_debug("%s\n", __func__);
	return virt_to_page(dummy_page[substream->stream]);	/* the same page */
}

static struct snd_pcm_ops mtk_i2s0_ops = {
	.open = mtk_pcm_i2s0_open,
	.close = mtk_pcm_i2s0_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = mtk_pcm_i2s0_hw_params,
	.hw_free = mtk_pcm_i2s0_hw_free,
	.prepare = mtk_pcm_i2s0_prepare,
	.trigger = mtk_pcm_i2s0_trigger,
	.pointer = mtk_pcm_i2s0_pointer,
	.copy = mtk_pcm_i2s0_copy,
	.silence = mtk_pcm_i2s0_silence,
	.page = mtk_i2s0_pcm_page,
};

static struct snd_soc_platform_driver mtk_i2s0_soc_platform = {
	.ops = &mtk_i2s0_ops,
	.pcm_new = mtk_asoc_pcm_i2s0_new,
	.probe = mtk_afe_i2s0_probe,
};

static int mtk_i2s0_probe(struct platform_device *pdev)
{
	pr_debug("%s\n", __func__);

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(64);
	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	if (pdev->dev.of_node)
		dev_set_name(&pdev->dev, "%s", MT_SOC_I2S0_PCM);

	pr_debug("%s: dev name %s\n", __func__, dev_name(&pdev->dev));

	mDev = &pdev->dev;

	return snd_soc_register_platform(&pdev->dev, &mtk_i2s0_soc_platform);
}

static int mtk_asoc_pcm_i2s0_new(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;

	pr_debug("%s\n", __func__);
	return ret;
}


static int mtk_afe_i2s0_probe(struct snd_soc_platform *platform)
{
	pr_debug("mtk_afe_i2s0_probe\n");
	snd_soc_add_platform_controls(platform, Audio_snd_i2s0_controls,
				      ARRAY_SIZE(Audio_snd_i2s0_controls));
	return 0;
}

static int mtk_i2s0_remove(struct platform_device *pdev)
{
	pr_debug("%s\n", __func__);
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mt_soc_pcm_dl1_i2s0_of_ids[] = {
	{.compatible = "mediatek,mt_soc_pcm_dl1_i2s0",},
	{}
};
#endif

static struct platform_driver mtk_i2s0_driver = {
	.driver = {
		   .name = MT_SOC_I2S0_PCM,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = mt_soc_pcm_dl1_i2s0_of_ids,
#endif
		   },
	.probe = mtk_i2s0_probe,
	.remove = mtk_i2s0_remove,
};

#ifndef CONFIG_OF
static struct platform_device *soc_mtki2s0_dev;
#endif

static int __init mtk_i2s0_soc_platform_init(void)
{
	int ret;

	pr_debug("%s\n", __func__);
#ifndef CONFIG_OF
	soc_mtki2s0_dev = platform_device_alloc(MT_SOC_I2S0_PCM, -1);
	if (!soc_mtki2s0_dev)
		return -ENOMEM;

	ret = platform_device_add(soc_mtki2s0_dev);
	if (ret != 0) {
		platform_device_put(soc_mtki2s0_dev);
		return ret;
	}
#endif

	ret = platform_driver_register(&mtk_i2s0_driver);
	return ret;

}
module_init(mtk_i2s0_soc_platform_init);

static void __exit mtk_i2s0_soc_platform_exit(void)
{
	pr_debug("%s\n", __func__);

	platform_driver_unregister(&mtk_i2s0_driver);
}
module_exit(mtk_i2s0_soc_platform_exit);

MODULE_DESCRIPTION("AFE PCM module platform driver");
MODULE_LICENSE("GPL");
