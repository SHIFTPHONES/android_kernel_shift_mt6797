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
 *   mtk_soc_codec_63xx
 *
 * Project:
 * --------
 *
 *
 * Description:
 * ------------
 *   Audio codec stub file
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
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#if !defined(CONFIG_MTK_LEGACY)
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#else
#include <mt-plat/mt_gpio.h>
#endif

#include "AudDrv_Common.h"
#include "AudDrv_Def.h"
#include "AudDrv_Afe.h"
#include "AudDrv_Ana.h"
#include "AudDrv_Clk.h"
#include "mt_soc_analog_type.h"
#ifdef _GIT318_READY
#include "mt_clkbuf_ctl.h"
#endif
#ifdef _GIT318_PMIC_READY
#include <mach/mt_pmic.h>
#endif
#include <mt-plat/mt_chip.h>
#ifdef _VOW_ENABLE
#include <mt-plat/vow_api.h>
#endif
#include "mt_soc_afe_control.h"
/*
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>
#include <mach/mt_pmic_wrap.h>
#include <mach/upmu_common.h>
*/
#include <mt-plat/upmu_common.h>

#ifdef CONFIG_MTK_SPEAKER
#include "mt_soc_codec_speaker_63xx.h"
/*#include <cust_pmic.h>*/
/* int PMIC_IMM_GetOneChannelValue(int dwChannel, int deCount, int trimd); */
#endif

#include "mt_soc_pcm_common.h"
#include "AudDrv_Common_func.h"
#include "AudDrv_Gpio.h"
#include "mt_soc_codec_63xx.h"


/* #define AW8736_MODE_CTRL // AW8736 PA output power mode control */

/* static function declaration */
static bool AudioPreAmp1_Sel(int Mul_Sel);
static bool GetAdcStatus(void);
static void Apply_Speaker_Gain(void);
static bool TurnOnVOWDigitalHW(bool enable);
static void TurnOffDacPower(void);
static void TurnOnDacPower(void);
static void SetDcCompenSation(void);
static void Voice_Amp_Change(bool enable);
static void Speaker_Amp_Change(bool enable);
static bool TurnOnVOWADcPowerACC(int MicType, bool enable);
static void EnableDcCompensation(bool bEnable);

static mt6331_Codec_Data_Priv *mCodec_data;
static uint32 mBlockSampleRate[AUDIO_ANALOG_DEVICE_INOUT_MAX] = { 48000, 48000, 48000 };

static DEFINE_MUTEX(Ana_Ctrl_Mutex);
static DEFINE_MUTEX(Ana_buf_Ctrl_Mutex);
static DEFINE_MUTEX(Ana_Clk_Mutex);
static DEFINE_MUTEX(Ana_Power_Mutex);
static DEFINE_MUTEX(AudAna_lock);

static int mAudio_Analog_Mic1_mode = AUDIO_ANALOGUL_MODE_ACC;
static int mAudio_Analog_Mic2_mode = AUDIO_ANALOGUL_MODE_ACC;
static int mAudio_Analog_Mic3_mode = AUDIO_ANALOGUL_MODE_ACC;
static int mAudio_Analog_Mic4_mode = AUDIO_ANALOGUL_MODE_ACC;

static int mAudio_Vow_Analog_Func_Enable;
static int mAudio_Vow_Digital_Func_Enable;

static int TrimOffset = 2048;
static const int DC1unit_in_uv = 19184;	/* in uv with 0DB */
static const int DC1devider = 8;	/* in uv */

/* Pmic Headphone Impedance varible */
static unsigned short hp_impedance;
static const unsigned short HpImpedanceAuxCable = 5000;
static const int HpImpedancePhase1Step = 100;
static const int HpImpedancePhase2Step = 100;
static const int HpImpedancePhase1AdcValue = 1200;
static const int HpImpedancePhase2AdcValue = 8800;

static unsigned int RG_AUDHPLTRIM_VAUDP15, RG_AUDHPRTRIM_VAUDP15, RG_AUDHPLFINETRIM_VAUDP15,
	RG_AUDHPRFINETRIM_VAUDP15, RG_AUDHPLTRIM_VAUDP15_SPKHP, RG_AUDHPRTRIM_VAUDP15_SPKHP,
	RG_AUDHPLFINETRIM_VAUDP15_SPKHP, RG_AUDHPRFINETRIM_VAUDP15_SPKHP;

static unsigned int pin_extspkamp, pin_extspkamp_2, pin_vowclk, pin_audmiso, pin_rcvspkswitch;
static unsigned int pin_mode_extspkamp, pin_mode_extspkamp_2, pin_mode_vowclk, pin_mode_audmiso,
	pin_mode_rcvspkswitch;


#ifdef CONFIG_MTK_SPEAKER
static int Speaker_mode = AUDIO_SPEAKER_MODE_AB;
static unsigned int Speaker_pga_gain = 1;	/* default 0Db. */
static bool mSpeaker_Ocflag;
#endif
static int mAdc_Power_Mode;
static unsigned int dAuxAdcChannel = 16;
static const int mDcOffsetTrimChannel = 9;
static bool mInitCodec;
static uint32 MicbiasRef, GetMicbias;

static int reg_AFE_VOW_CFG0 = 0x0000;	/* VOW AMPREF Setting */
static int reg_AFE_VOW_CFG1 = 0x0000;	/* VOW A,B timeout initial value (timer) */
static int reg_AFE_VOW_CFG2 = 0x2222;	/* VOW A,B value setting (BABA) */
static int reg_AFE_VOW_CFG3 = 0x8767;	/* alhpa and beta K value setting (beta_rise,fall,alpha_rise,fall) */
static int reg_AFE_VOW_CFG4 = 0x006E;	/* gamma K value setting (gamma), bit4:8 should not modify */
static int reg_AFE_VOW_CFG5 = 0x0001;	/* N mini value setting (Nmin) */
static bool mIsVOWOn;

/* VOW using */
typedef enum {
	AUDIO_VOW_MIC_TYPE_Handset_AMIC = 0,
	AUDIO_VOW_MIC_TYPE_Headset_MIC,
	AUDIO_VOW_MIC_TYPE_Handset_DMIC,	/* 1P6 */
	AUDIO_VOW_MIC_TYPE_Handset_DMIC_800K,	/* 800K */
	AUDIO_VOW_MIC_TYPE_Handset_AMIC_DCC,	/* DCC mems */
	AUDIO_VOW_MIC_TYPE_Headset_MIC_DCC,
	AUDIO_VOW_MIC_TYPE_Handset_AMIC_DCCECM,	/* DCC ECM, dual differential */
	AUDIO_VOW_MIC_TYPE_Headset_MIC_DCCECM	/* DCC ECM, signal differential */
} AUDIO_VOW_MIC_TYPE;

static int mAudio_VOW_Mic_type = AUDIO_VOW_MIC_TYPE_Handset_AMIC;
static void Audio_Amp_Change(int channels, bool enable);
static void SavePowerState(void)
{
	int i = 0;

	for (i = 0; i < AUDIO_ANALOG_DEVICE_MAX; i++) {
		mCodec_data->mAudio_BackUpAna_DevicePower[i] =
		    mCodec_data->mAudio_Ana_DevicePower[i];
	}
}

static void RestorePowerState(void)
{
	int i = 0;

	for (i = 0; i < AUDIO_ANALOG_DEVICE_MAX; i++) {
		mCodec_data->mAudio_Ana_DevicePower[i] =
		    mCodec_data->mAudio_BackUpAna_DevicePower[i];
	}
}

static bool GetDLStatus(void)
{
	int i = 0;

	for (i = 0; i < AUDIO_ANALOG_DEVICE_2IN1_SPK; i++) {
		if (mCodec_data->mAudio_Ana_DevicePower[i] == true)
			return true;
	}
	return false;
}

static bool mAnaSuspend;
void SetAnalogSuspend(bool bEnable)
{
	pr_debug("%s bEnable ==%d mAnaSuspend = %d\n", __func__, bEnable, mAnaSuspend);
	if ((bEnable == true) && (mAnaSuspend == false)) {
		/*Ana_Log_Print();*/
		SavePowerState();
		if (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETL] == true) {
			mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETL] =
			    false;
			Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_LEFT1, false);
		}
		if (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETR] == true) {
			mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETR] =
			    false;
			Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_RIGHT1, false);
		}
		if (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_EARPIECEL] == true) {
			mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_EARPIECEL] =
			    false;
			Voice_Amp_Change(false);
		}
		if (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_SPEAKERL] == true) {
			mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_SPEAKERL] =
			    false;
			Speaker_Amp_Change(false);
		}
		/*Ana_Log_Print();*/
		mAnaSuspend = true;
	} else if ((bEnable == false) && (mAnaSuspend == true)) {
		/*Ana_Log_Print();*/
		if (mCodec_data->mAudio_BackUpAna_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETL] ==
		    true) {
			Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_LEFT1, true);
			mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETL] =
			    true;
		}
		if (mCodec_data->mAudio_BackUpAna_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETR] ==
		    true) {
			Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_RIGHT1, true);
			mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETR] =
			    false;
		}
		if (mCodec_data->mAudio_BackUpAna_DevicePower[AUDIO_ANALOG_DEVICE_OUT_EARPIECEL] ==
		    true) {
			Voice_Amp_Change(true);
			mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_EARPIECEL] =
			    false;
		}
		if (mCodec_data->mAudio_BackUpAna_DevicePower[AUDIO_ANALOG_DEVICE_OUT_SPEAKERL] ==
		    true) {
			Speaker_Amp_Change(true);
			mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_SPEAKERL] =
			    false;
		}
		RestorePowerState();
		/*Ana_Log_Print();*/
		mAnaSuspend = false;
	}
}

static int audck_buf_Count;
void audckbufEnable(bool enable)
{
	/*pr_debug("audckbufEnable audck_buf_Count = %d enable = %d\n", audck_buf_Count, enable);*/
	mutex_lock(&Ana_buf_Ctrl_Mutex);
	if (enable) {
		if (audck_buf_Count == 0) {
			/*pr_debug("+clk_buf_ctrl(CLK_BUF_AUDIO,true)\n");*/
#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef _GIT318_READY
			clk_buf_ctrl(CLK_BUF_AUDIO, true);
#endif
#endif
			/*pr_debug("-clk_buf_ctrl(CLK_BUF_AUDIO,true)\n");*/
		}
		audck_buf_Count++;
	} else {
		audck_buf_Count--;
		if (audck_buf_Count == 0) {
			/*pr_debug("+clk_buf_ctrl(CLK_BUF_AUDIO,false)\n");*/
#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef _GIT318_READY
			clk_buf_ctrl(CLK_BUF_AUDIO, false);
#endif
#endif
			/*pr_debug("-clk_buf_ctrl(CLK_BUF_AUDIO,false)\n");*/
		}
		if (audck_buf_Count < 0) {
			pr_warn("audck_buf_Count count <0\n");
			audck_buf_Count = 0;
		}
	}
	mutex_unlock(&Ana_buf_Ctrl_Mutex);
}

static int ClsqCount;
static void ClsqEnable(bool enable)
{
	/*pr_debug("ClsqEnable ClsqCount = %d enable = %d\n", ClsqCount, enable);*/
	mutex_lock(&AudAna_lock);
	if (enable) {
		if (ClsqCount == 0) {
			Ana_Set_Reg(TOP_CLKSQ, 0x0001, 0x0001);	/* Enable CLKSQ 26MHz */
			Ana_Set_Reg(TOP_CLKSQ_SET, 0x0001, 0x0001);	/* Turn on 26MHz source clock */
		}
		ClsqCount++;
	} else {
		ClsqCount--;
		if (ClsqCount < 0) {
			pr_warn("ClsqEnable count <0\n");
			ClsqCount = 0;
		}
		if (ClsqCount == 0) {
			Ana_Set_Reg(TOP_CLKSQ_CLR, 0x0001, 0x0001);	/* Turn off 26MHz source clock */
			Ana_Set_Reg(TOP_CLKSQ, 0x0000, 0x0001);	/* Disable CLKSQ 26MHz */
		}
	}
	mutex_unlock(&AudAna_lock);
}

static int TopCkCount;
static void Topck_Enable(bool enable)
{
	/*pr_debug("Topck_Enable enable = %d TopCkCount = %d\n", enable, TopCkCount);*/
	mutex_lock(&Ana_Clk_Mutex);
	if (enable == true) {
		if (TopCkCount == 0)
			Ana_Set_Reg(TOP_CKPDN_CON0_CLR, 0x7000, 0x7000);
		/* MT6328 Turn on AUDNCP_CLKDIV engine clock,Turn on AUD 26M */

		TopCkCount++;
	} else {
		TopCkCount--;
		if (TopCkCount == 0)
			Ana_Set_Reg(TOP_CKPDN_CON0_SET, 0x7000, 0x7000);
		/* MT6328 Turn off AUDNCP_CLKDIV engine clock,Turn off AUD 26M */

		if (TopCkCount <= 0) {
			pr_warn("TopCkCount <0 =%d\n ", TopCkCount);
			TopCkCount = 0;
		}
	}
	mutex_unlock(&Ana_Clk_Mutex);
}

static int NvRegCount;
static void NvregEnable(bool enable)
{
	/*pr_debug("NvregEnable NvRegCount == %d enable = %d\n", NvRegCount, enable);*/
	mutex_lock(&Ana_Clk_Mutex);
	if (enable == true) {
		if (NvRegCount == 0)
			Ana_Set_Reg(AUDDEC_ANA_CON8, 0x0000, 0x0002);	/* Enable AUDGLB */

		NvRegCount++;
	} else {
		NvRegCount--;
		if (NvRegCount == 0)
			Ana_Set_Reg(AUDDEC_ANA_CON8, 0x0002, 0x0002);	/* disable AUDGLB */

		if (NvRegCount < 0) {
			pr_warn("NvRegCount <0 =%d\n ", NvRegCount);
			NvRegCount = 0;
		}
	}
	mutex_unlock(&Ana_Clk_Mutex);
}

#ifdef _VOW_ENABLE
static int VOW12MCKCount;
#endif

#ifdef _VOW_ENABLE

static void VOW12MCK_Enable(bool enable)
{

	pr_debug("VOW12MCK_Enable VOW12MCKCount == %d enable = %d\n", VOW12MCKCount, enable);
	mutex_lock(&Ana_Clk_Mutex);
	if (enable == true) {
		if (VOW12MCKCount == 0)
			Ana_Set_Reg(TOP_CKPDN_CON0_CLR, 0x8000, 0x8000);
		/* Enable  TOP_CKPDN_CON0 bit15 for enable VOW 12M clock */
		VOW12MCKCount++;
	} else {
		VOW12MCKCount--;
		if (VOW12MCKCount == 0)
			Ana_Set_Reg(TOP_CKPDN_CON0_SET, 0x8000, 0x8000);
		/* disable TOP_CKPDN_CON0 bit15 for enable VOW 12M clock */

		if (VOW12MCKCount < 0) {
			pr_warn("VOW12MCKCount <0 =%d\n ", VOW12MCKCount);
			VOW12MCKCount = 0;
		}
	}
	mutex_unlock(&Ana_Clk_Mutex);
}
#endif


void vow_irq_handler(void)
{
#ifdef _VOW_ENABLE

	pr_debug("vow_irq_handler,audio irq event....\n");
	/* TurnOnVOWADcPowerACC(AUDIO_ANALOG_DEVICE_IN_ADC1, false); */
	/* TurnOnVOWDigitalHW(false); */
#if defined(VOW_TONE_TEST)
	EnableSideGenHw(Soc_Aud_InterConnectionOutput_O03, Soc_Aud_MemIF_Direction_DIRECTION_OUTPUT,
			true);
#endif
	/* VowDrv_ChangeStatus(); */
#endif
}

/*extern kal_uint32 upmu_get_reg_value(kal_uint32 reg);*/

void Auddrv_Read_Efuse_HPOffset(void)
{
#ifndef CONFIG_FPGA_EARLY_PORTING

	unsigned int ret = 0;
	unsigned int reg_val = 0;
	int i = 0, j = 0;
	unsigned int efusevalue[3];

	pr_debug("Auddrv_Read_Efuse_HPOffset(+)\n");

	/* 1. enable efuse ctrl engine clock */
	ret = pmic_config_interface(0x027C, 0x0040, 0xFFFF, 0);
	ret = pmic_config_interface(0x0252, 0x0004, 0xFFFF, 0);

	/* 2. */
	ret = pmic_config_interface(0x0C16, 0x1, 0x1, 0);

	/*  //MT6325
	   Audio data from 746 to 770
	   0xe 746 751
	   0xf 752 767
	   0x10 768 770
	 */

	/* MT6328 681 to 705
	   Audio data from 681 to 705
	 */

	for (i = 0xa; i <= 0xc; i++) {
		/* 3. set row to read */
		ret = pmic_config_interface(0x0C00, i, 0x1F, 1);

		/* 4. Toggle */
		ret = pmic_read_interface(0xC10, &reg_val, 0x1, 0);

		if (reg_val == 0)
			ret = pmic_config_interface(0xC10, 1, 0x1, 0);
		else
			ret = pmic_config_interface(0xC10, 0, 0x1, 0);


		/* 5. polling Reg[0xC1A] */
		reg_val = 1;
		while (reg_val == 1) {
			ret = pmic_read_interface(0xC1A, &reg_val, 0x1, 0);
			pr_debug("Auddrv_Read_Efuse_HPOffset polling 0xC1A=0x%x\n", reg_val);
		}

		udelay(1000);	/* Need to delay at least 1ms for 0xC1A and than can read 0xC18 */

		/* 6. read data */
		efusevalue[j] = upmu_get_reg_value(0x0C18);
		pr_debug("HPoffset : efuse[%d]=0x%x\n", j, efusevalue[j]);
		j++;
	}

	/* 7. Disable efuse ctrl engine clock */
	ret = pmic_config_interface(0x0250, 0x0004, 0xFFFF, 0);
	ret = pmic_config_interface(0x027A, 0x0040, 0xFFFF, 0);

	RG_AUDHPLTRIM_VAUDP15 = (efusevalue[0] >> 9) & 0xf;
	RG_AUDHPRTRIM_VAUDP15 = ((efusevalue[0] >> 13) & 0x7) + ((efusevalue[1] & 0x1) << 3);
	RG_AUDHPLFINETRIM_VAUDP15 = (efusevalue[1] >> 2) & 0x3;
	RG_AUDHPRFINETRIM_VAUDP15 = (efusevalue[1] >> 4) & 0x3;
	RG_AUDHPLTRIM_VAUDP15_SPKHP = (efusevalue[1] >> 6) & 0xF;
	RG_AUDHPRTRIM_VAUDP15_SPKHP = (efusevalue[1] >> 10) & 0xF;
	RG_AUDHPLFINETRIM_VAUDP15_SPKHP = ((efusevalue[1] >> 14) & 0x3);
	RG_AUDHPRFINETRIM_VAUDP15_SPKHP = (efusevalue[2] & 0x3);

	pr_debug("RG_AUDHPLTRIM_VAUDP15 = %x\n", RG_AUDHPLTRIM_VAUDP15);
	pr_debug("RG_AUDHPRTRIM_VAUDP15 = %x\n", RG_AUDHPRTRIM_VAUDP15);
	pr_debug("RG_AUDHPLFINETRIM_VAUDP15 = %x\n", RG_AUDHPLFINETRIM_VAUDP15);
	pr_debug("RG_AUDHPRFINETRIM_VAUDP15 = %x\n", RG_AUDHPRFINETRIM_VAUDP15);
	pr_debug("RG_AUDHPLTRIM_VAUDP15_SPKHP = %x\n", RG_AUDHPLTRIM_VAUDP15_SPKHP);
	pr_debug("RG_AUDHPRTRIM_VAUDP15_SPKHP = %x\n", RG_AUDHPRTRIM_VAUDP15_SPKHP);
	pr_debug("RG_AUDHPLFINETRIM_VAUDP15_SPKHP = %x\n", RG_AUDHPLFINETRIM_VAUDP15_SPKHP);
	pr_debug("RG_AUDHPRFINETRIM_VAUDP15_SPKHP = %x\n", RG_AUDHPRFINETRIM_VAUDP15_SPKHP);

#endif
	pr_debug("Auddrv_Read_Efuse_HPOffset(-)\n");
}
EXPORT_SYMBOL(Auddrv_Read_Efuse_HPOffset);

#ifdef CONFIG_MTK_SPEAKER
static void Apply_Speaker_Gain(void)
{
	pr_debug("%s Speaker_pga_gain= %d\n", __func__, Speaker_pga_gain);

	Ana_Set_Reg(SPK_ANA_CON0, Speaker_pga_gain << 11, 0x7800);
}
#else
static void Apply_Speaker_Gain(void)
{
}
#endif

void setOffsetTrimMux(unsigned int Mux)
{
	pr_debug("%s Mux = %d\n", __func__, Mux);
	Ana_Set_Reg(AUDDEC_ANA_CON3, Mux << 1, 0xf << 1);	/* Audio offset trimming buffer mux selection */
}

void setOffsetTrimBufferGain(unsigned int gain)
{
	Ana_Set_Reg(AUDDEC_ANA_CON3, gain << 5, 0x3 << 5);	/* Audio offset trimming buffer gain selection */
}

static int mHplTrimOffset = 2048;
static int mHprTrimOffset = 2048;

void SetHplTrimOffset(int Offset)
{
	pr_debug("%s Offset = %d\n", __func__, Offset);
	mHplTrimOffset = Offset;
	if ((Offset > 2098) || (Offset < 1998)) {
		mHplTrimOffset = 2048;
		pr_warn("SetHplTrimOffset offset may be invalid offset = %d\n", Offset);
	}
}

void SetHprTrimOffset(int Offset)
{
	pr_debug("%s Offset = %d\n", __func__, Offset);
	mHprTrimOffset = Offset;
	if ((Offset > 2098) || (Offset < 1998)) {
		mHprTrimOffset = 2048;
		pr_warn("SetHprTrimOffset offset may be invalid offset = %d\n", Offset);
	}
}

void EnableTrimbuffer(bool benable)
{
	if (benable == true) {
		Ana_Set_Reg(AUDDEC_ANA_CON4, 0x0001, 0x0001);
		/* Enable HP & HS drivers bias circuit    //trim buffer enable bit MT6328 */

		Ana_Set_Reg(AUDDEC_ANA_CON3, 0x1, 0x1);	/* Audio offset trimming buffer enable */
	} else {
		Ana_Set_Reg(AUDDEC_ANA_CON4, 0x0000, 0x0001);	/* Disable drivers bias circuit */

		Ana_Set_Reg(AUDDEC_ANA_CON3, 0x0, 0x1);	/* Audio offset trimming buffer disable */
	}
}


void OpenTrimBufferHardware(bool enable)
{				/* 0804 TODO!!! */
	if (enable) {
		pr_debug("%s true\n", __func__);
		TurnOnDacPower();
		/* AUXADC large scale - AUXADC_CON2(AUXADC ADC AVG SELECTION[9]) */
		Ana_Set_Reg(0x0E9C, 0x0200, 0x0200);
		/* set analog part (HP playback) */
		Ana_Set_Reg(AUDDEC_ANA_CON8, 0x0000, 0x0002);
		/* Enable AUDGLB */
		/* upmu_set_rg_vio18_cal(1);// for MT6328 E1 VIO18 patch only */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON1, 0x0001, 0xffff);
		/* Turn on DA_600K_NCP_VA18 */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON2, 0x002B, 0xffff);
		/* Set NCP clock as 604kHz // 26MHz/43 = 604KHz */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON0, 0x0001, 0xffff);
		/* Toggle RG_DIVCKS_CHG */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON4, 0x0000, 0xffff);
		/* Set NCP soft start mode as default mode */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON3, 0x0000, 0xffff);
		/* Enable NCP */
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0A41, 0xfeeb);
		/* Enable cap-less HC LDO (1.5V) & LDO VA33REFGEN */
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC1, 0xfeeb);
		/* Enable cap-less LC LDO (1.5V) */
		Ana_Set_Reg(AUDDEC_ANA_CON7, 0x8000, 0x8000);
		/* Enable NV regulator (-1.5V) */
		Ana_Set_Reg(ZCD_CON0, 0x0000, 0xffff);
		/* Disable AUD_ZCD */
		Ana_Set_Reg(AUDDEC_ANA_CON0, 0xE080, 0xffff);
		/* Disable headphone, voice and short-ckt protection. voice MUX is set mute */

		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC0, 0xfeeb);
		/* Enable IBIST */
		/* Ana_Set_Reg(AUDDEC_ANA_CON4, 0x0700, 0xffff); //Enable HP & HS drivers bias circuit */
		/* Ana_Set_Reg(AUDDEC_ANA_CON4, 0x1f00, 0xffff); //Enable HP & HS drivers bias circuit    //trim */
		Ana_Set_Reg(AUDDEC_ANA_CON4, 0x1f01, 0xffff);
		/* Enable HP & HS drivers bias circuit    //trim buffer enable bit MT6328 */

		Ana_Set_Reg(AUDDEC_ANA_CON5, 0x5490, 0xffff);	/* HP/HS ibias & DR bias current optimization */
		udelay(50);
		Ana_Set_Reg(ZCD_CON2, 0x0489, 0xffff);	/* Set HPR/HPL gain as -1dB */
		Ana_Set_Reg(ZCD_CON3, 0x001F, 0xffff);	/* Set voice gain as minimum (~ -40dB) */
		Ana_Set_Reg(AUDDEC_ANA_CON1, 0x0480, 0xffff);	/* De_OSC of HP and enable output STBENH */
		Ana_Set_Reg(AUDDEC_ANA_CON1, 0x1480, 0xffff);	/* De_OSC of voice, enable output STBENH */
		Ana_Set_Reg(AUDDEC_ANA_CON0, 0xE090, 0xffff);	/* Enable voice driver */
		Ana_Set_Reg(AUDDEC_ANA_CON1, 0x14A0, 0xffff);	/* Enable pre-charge buffer */
		udelay(50);
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC2, 0xfeeb);	/* Enable AUD_CLK */
		Ana_Set_Reg(AUDDEC_ANA_CON0, 0x0009, 0x0009);	/* Enable Audio DACL */
		Ana_Set_Reg(AUDDEC_ANA_CON0, 0x0006, 0x0006);	/* Enable Audio DACR */

	} else {
		pr_debug("%s false\n", __func__);
		Ana_Set_Reg(AUDDEC_ANA_CON4, 0x0000, 0xffff);	/* Disable drivers bias circuit */
		Ana_Set_Reg(AUDDEC_ANA_CON0, 0x0000, 0xffff);	/* Disable Audio DAC */
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC0, 0xfeeb);	/* Disable AUD_CLK, bit2/4/8 is for ADC, do not set */
		Ana_Set_Reg(AUDDEC_ANA_CON7, 0x0000, 0x8000);	/* Disable NV regulator (-1.5V) */
		/*Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0001, 0xfeeb);*/
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x02c1, 0xfeeb);
		/* for AUX detection ,Disable cap-less LDOs (1.5V) & Disable IBIST */
		TurnOffDacPower();
	}
}

void OpenAnalogTrimHardware(bool enable)
{
	if (enable) {
		pr_debug("%s true\n", __func__);
		TurnOnDacPower();
		/* upmu_set_rg_vio18_cal(1);// for MT6328 E1 VIO18 patch only */
		/* set analog part (HP playback) */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON1, 0x0001, 0xffff);
		/* Turn on DA_600K_NCP_VA18 */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON2, 0x002B, 0xffff);
		/* Set NCP clock as 604kHz // 26MHz/43 = 604KHz */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON0, 0x0001, 0xffff);
		/* Toggle RG_DIVCKS_CHG */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON4, 0x0000, 0xffff);
		/* Set NCP soft start mode as default mode */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON3, 0x0000, 0xffff);
		/* Enable NCP */
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0A41, 0xfeeb);
		/* Enable cap-less HC LDO (1.5V) & LDO VA33REFGEN */
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC1, 0xfeeb);
		/* Enable cap-less LC LDO (1.5V) */
		Ana_Set_Reg(AUDDEC_ANA_CON7, 0x8000, 0x8000);
		/* Enable NV regulator (-1.5V) */
		Ana_Set_Reg(ZCD_CON0, 0x0000, 0xffff);
		/* Disable AUD_ZCD */
		Ana_Set_Reg(AUDDEC_ANA_CON0, 0xE080, 0xffff);
		/* Disable headphone, voice and short-ckt protection. voice MUX is set mute */

		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC0, 0xfeeb);
		/* Enable IBIST */
		/* Ana_Set_Reg(AUDDEC_ANA_CON4, 0x0700, 0xffff); //Enable HP & HS drivers bias circuit */
		/* Ana_Set_Reg(AUDDEC_ANA_CON4, 0x1f00, 0xffff); //Enable HP & HS drivers bias circuit    //trim */
		Ana_Set_Reg(AUDDEC_ANA_CON4, 0x1f01, 0xffff);
		/* Enable HP & HS drivers bias circuit    //trim */

		Ana_Set_Reg(AUDDEC_ANA_CON5, 0x5490, 0xffff);
		/* HP/HS ibias & DR bias current optimization */
		udelay(50);
		Ana_Set_Reg(ZCD_CON2, 0x0489, 0xffff);
		/* Set HPR/HPL gain as -1dB */
		Ana_Set_Reg(ZCD_CON3, 0x001F, 0xffff);
		/* Set voice gain as minimum (~ -40dB) */
		Ana_Set_Reg(AUDDEC_ANA_CON1, 0x0480, 0xffff);
		/* De_OSC of HP and enable output STBENH */
		Ana_Set_Reg(AUDDEC_ANA_CON1, 0x1480, 0xffff);
		/* De_OSC of voice, enable output STBENH */
		Ana_Set_Reg(AUDDEC_ANA_CON0, 0xE090, 0xffff);
		/* Enable voice driver */
		Ana_Set_Reg(AUDDEC_ANA_CON1, 0x14A0, 0xffff);
		/* Enable pre-charge buffer */
		udelay(50);
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC2, 0xfeeb);
		/* Enable AUD_CLK */
		Ana_Set_Reg(AUDDEC_ANA_CON0, 0x0009, 0x0009);
		/* Enable Audio DACL */
		Ana_Set_Reg(AUDDEC_ANA_CON0, 0x0006, 0x0006);
		/* Enable Audio DACR */

	} else {
		pr_debug("%s false\n", __func__);
		Ana_Set_Reg(AUDDEC_ANA_CON4, 0x0000, 0xffff);
		/* Disable drivers bias circuit */
		Ana_Set_Reg(AUDDEC_ANA_CON0, 0x0000, 0xffff);
		/* Disable Audio DAC */
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC0, 0xfeeb);
		/* Disable AUD_CLK, bit2/4/8 is for ADC, do not set */
		Ana_Set_Reg(AUDDEC_ANA_CON7, 0x0000, 0x8000);
		/* Disable NV regulator (-1.5V) */
		/*Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0001, 0xfeeb);*/
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x02c1, 0xfeeb); /* for AUX detection */
		/* Disable cap-less LDOs (1.5V) & Disable IBIST */
		TurnOffDacPower();
	}
}

void OpenAnalogHeadphone(bool bEnable)
{
	pr_debug("OpenAnalogHeadphone bEnable = %d", bEnable);
	if (bEnable) {
		SetHplTrimOffset(2048);
		SetHprTrimOffset(2048);
		mBlockSampleRate[AUDIO_ANALOG_DEVICE_OUT_DAC] = 44100;
		Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_LEFT1, true);
		mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETL] = true;
		Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_RIGHT1, true);
		mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETR] = true;
	} else {
		mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETL] = false;
		Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_LEFT1, false);
		mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETR] = false;
		Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_RIGHT1, false);
	}
}

bool OpenHeadPhoneImpedanceSetting(bool bEnable)
{
	pr_debug("%s benable = %d\n", __func__, bEnable);
	if (GetDLStatus() == true)
		return false;

	if (bEnable == true) {
		TurnOnDacPower();
		Ana_Set_Reg(0x0E9C, 0x0200, 0x0200);
		/* AUXADC large scale - AUXADC_CON2(AUXADC ADC AVG SELECTION[9]) */

		/* upmu_set_rg_vio18_cal(1);// for MT6328 E1 VIO18 patch only */
#if 0				/* test 6328 sgen */
		Ana_Set_Reg(AFE_SGEN_CFG0, 0x0080, 0xffff);
		Ana_Set_Reg(AFE_SGEN_CFG1, 0x0101, 0xffff);
		Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x0000, 0xffff);	/* power on clock */
		/* Ana_Set_Reg(PMIC_AFE_TOP_CON0, 0x0002, 0x0002);   //UL from sinetable */
		Ana_Set_Reg(PMIC_AFE_TOP_CON0, 0x0001, 0x0001);	/* DL from sinetable */
		Ana_Set_Reg(AFE_DL_DC_COMP_CFG0, 0xf000, 0xffff);
		Ana_Set_Reg(AFE_DL_DC_COMP_CFG1, 0xf000, 0xffff);
		Ana_Set_Reg(AFE_DL_DC_COMP_CFG2, 0x0001, 0xffff);
		Ana_Set_Reg(AFE_DL_DC_COMP_CFG0, 0x7f00, 0xffff);
		Ana_Set_Reg(AFE_DL_DC_COMP_CFG1, 0x7f00, 0xffff);
		Ana_Set_Reg(AFE_DL_DC_COMP_CFG2, 0x0001, 0xffff);
#endif

		/* set analog part (HP playback) */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON4, 0x0003, 0xffff);
		/* Set NCP soft start mode as default mode */

		Ana_Set_Reg(AUDNCP_CLKDIV_CON1, 0x0001, 0xffff);
		/* Turn on DA_600K_NCP_VA18 */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON2, 0x002B, 0xffff);
		/* Set NCP clock as 604kHz // 26MHz/43 = 604KHz */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON0, 0x0001, 0xffff);
		/* Toggle RG_DIVCKS_CHG */
		/* Ana_Set_Reg(AUDNCP_CLKDIV_CON4, 0x0000, 0xffff);
		   //Set NCP soft start mode as default mode */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON3, 0x0000, 0xffff);
		/* Enable NCP */

		/* Ana_Set_Reg(AUDDEC_ANA_CON4, 0x0300, 0xffff);
		   //Enable HP driver bias circuit */
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0841, 0xfeeb);
		/* Enable cap-less HC LDO (1.5V) & LDO VA33REFGEN */

		/* Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0A41, 0xfeeb);
		   //Enable cap-less HC LDO (1.5V) & LDO VA33REFGEN */
		/* Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC1, 0xfeeb);
		   //Enable cap-less LC LDO (1.5V) */
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x28C1, 0xfeeb);
		/* Enable cap-less LC LDO (1.5V) */
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC1, 0xfeeb);
		/* Enable cap-less LC LDO (1.5V) */

		Ana_Set_Reg(AUDDEC_ANA_CON7, 0x8000, 0x8000);
		/* Enable NV regulator (-1.5V) */
		Ana_Set_Reg(ZCD_CON0, 0x0000, 0xffff);	/* Disable AUD_ZCD */

		/* Ana_Set_Reg(AUDDEC_ANA_CON0, 0xE000, 0xffff);
		   Disable headphone, voice and short-ckt protection. HP and voice MUX is opened */
		Ana_Set_Reg(AUDDEC_ANA_CON0, 0xE080, 0xffff);
		/* Disable headphone, voice and short-ckt protection. HP and voice MUX is opened */
		Ana_Set_Reg(AUDDEC_ANA_CON1, 0x0000, 0x0400);
		/* HP output not short to VCM */
		Ana_Set_Reg(AUDDEC_ANA_CON1, 0x0000, 0x0080);
		/* HP driver output stability enhance option: NO */
		Ana_Set_Reg(AUDDEC_ANA_CON3, 0x0080, 0x0080);
		/* Audio headphone speaker detection enable */
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0000, 0x0001);
		/* Power down control for IbiasDistrib circuit */
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0002, 0x0002);
		/* Enable AUD_CLK */
		Ana_Set_Reg(AUDDEC_ANA_CON0, 0xE089, 0xffff);
		/* Enable LCH Audio DAC */
		Ana_Set_Reg(AUDDEC_ANA_CON3, 0x0900, 0x0f00);
		/* Select HPR as HPDET output and select DACLP as HPDET circuit input */

	} else {

		Ana_Set_Reg(AUDDEC_ANA_CON3, 0x0000, 0x0f00);
		/* Select HPR as HPDET output and select DACLP as HPDET circuit input */
		Ana_Set_Reg(AUDDEC_ANA_CON0, 0xE080, 0xffff);
		/* Disable LCH Audio DAC */
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0000, 0x0002);
		/* Disable AUD_CLK */
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0001, 0x0001);
		/* Power down control for IbiasDistrib circuit */
		Ana_Set_Reg(AUDDEC_ANA_CON3, 0x0000, 0x0080);
		/* Audio headphone speaker detection disable */

		Ana_Set_Reg(AUDDEC_ANA_CON7, 0x0000, 0x8000);
		/* Disable NV regulator (-1.5V) */
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0001, 0xfeeb);
		/* Disable cap-less LDOs (1.5V) & Disable IBIST */
		Ana_Set_Reg(AUDDEC_ANA_CON1, 0x0080, 0x0080);
		/* HP driver output stability enhance option: NO */
		Ana_Set_Reg(AUDDEC_ANA_CON0, 0xE000, 0xffff);

		TurnOffDacPower();
	}
	return true;
}

static unsigned short Calculate_HP_Impedance(unsigned short dcinit,
		unsigned short dcinput, unsigned short pcmoffset)
{
	unsigned short R_hp;
	unsigned int dcvalue;
	unsigned int R_tmp = 0;

	if (dcinput < dcinit)
		return 0;

	if (dcinput > 4080*4)					/* For AuxCable case - open path */
		return HpImpedanceAuxCable;

	dcvalue = (unsigned int)(dcinput - dcinit);		/* S32.2 = S32.2 - S32.2 */
	if (pcmoffset == HpImpedancePhase1AdcValue) {
		R_tmp = (dcvalue * 3223 + 2048) >> 12;		/* 1200 (S32.0) */
		if (R_tmp < 160) {
			pr_debug("%s Phase 1200 is smaller than 160, resistor %d, Goto Phase 8800\n",
				__func__, R_tmp);
			R_tmp = 0;
		}
	} else if (pcmoffset == HpImpedancePhase2AdcValue) {
		R_tmp = (dcvalue * 879 + 4096) >> 13;		/* 8800 (S32.0) */
	}

	R_hp = (unsigned short)R_tmp;
	pr_debug("%s pcmoffset %d dcoffset %d resistor %d\n", __func__,
			pcmoffset, dcvalue, R_hp);

	return R_hp;
}


static int detect_impedance_by_phase(void)
{
	unsigned int i;
	unsigned short dcoffset = 0, average = 0;
	unsigned short ibuffer_v[4];
	short value = 0;
	unsigned short detect_impedance = 0;

	setOffsetTrimMux(AUDIO_OFFSET_TRIM_MUX_HPR);
	setOffsetTrimBufferGain(3); /* HPDET trim. buffer gain : 18db */
	EnableTrimbuffer(true);
	setHpGainZero();
	Ana_Set_Reg(AFE_DL_DC_COMP_CFG0, 0x0, 0xffff);
	Ana_Set_Reg(AFE_DL_DC_COMP_CFG1, 0x0, 0xffff);
	EnableDcCompensation(true);

	for (value = 0; value < (HpImpedancePhase2AdcValue + HpImpedancePhase1Step);
		value += HpImpedancePhase1Step) {

		if (value > HpImpedancePhase2AdcValue)
			value = HpImpedancePhase2AdcValue;

		/* apply dc by dc compensation: 16bit and negative value */
		Ana_Set_Reg(AFE_DL_DC_COMP_CFG0, -value, 0xffff);
		Ana_Set_Reg(AFE_DL_DC_COMP_CFG1, -value, 0xffff);

		/* save for DC =0 offset */
		if (value == 0) {
			usleep_range(1*1000, 2*1000);

			/* get adc value */
			dcoffset = 0;
			for (i = 0; i < 4; i++) {
				ibuffer_v[i] = PMIC_IMM_GetOneChannelValue(MT6328_AUX_CH9, 5, 0);
				dcoffset = dcoffset + ibuffer_v[i];
			}
			pr_debug("[DCinit]offset = %d offset1 = %d offset2 = %d offset3= %d offset4 = %d\n",
				dcoffset, ibuffer_v[0], ibuffer_v[1], ibuffer_v[2], ibuffer_v[3]);
		}

		/* start checking */
		if (value == HpImpedancePhase1AdcValue || value == HpImpedancePhase2AdcValue) {
			usleep_range(1*1000, 2*1000);

			/* get adc value */
			average = 0;
			for (i = 0; i < 4; i++) {
				ibuffer_v[i] = PMIC_IMM_GetOneChannelValue(MT6328_AUX_CH9, 5, 0);
				average = average + ibuffer_v[i];
			}

			detect_impedance = Calculate_HP_Impedance(dcoffset, average, value);
			if (detect_impedance) {
				pr_debug("[phase1]average = %d offset1 = %d offset2 = %d offset3 = %d offset4 = %d\n",
					average, ibuffer_v[0], ibuffer_v[1], ibuffer_v[2], ibuffer_v[3]);
				pr_debug("[phase1]value = %d average = %d dcinit_value = %d mhp_impedance = %d\n ",
					value, average, dcoffset, detect_impedance);
				break;
			}
		}

		usleep_range(1*250, 1*500);
	}

	/* Ramp-Down */
	while (value > 0) {
		value = value - HpImpedancePhase1Step;
		/* apply dc by dc compensation: 16bit and negative value */
		Ana_Set_Reg(AFE_DL_DC_COMP_CFG0, -value, 0xffff);
		Ana_Set_Reg(AFE_DL_DC_COMP_CFG1, -value, 0xffff);

		usleep_range(1*200, 1*400);
	}
	EnableDcCompensation(false);
	setOffsetTrimMux(AUDIO_OFFSET_TRIM_MUX_GROUND);
	EnableTrimbuffer(false);

	return detect_impedance;
}

static int pmic_hp_impedance_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	if (OpenHeadPhoneImpedanceSetting(true) == true) {
		hp_impedance = detect_impedance_by_phase();
		OpenHeadPhoneImpedanceSetting(false);
	} else
		pr_warn("%s(), Pmic DL Busy, HPDET do nothing\n", __func__);

	ucontrol->value.integer.value[0] = hp_impedance;
	pr_debug("-%s(), mhp_impedance = %d\n",
		 __func__, hp_impedance);
	return 0;
}

static int pmic_hp_impedance_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s(), hp_impedance = %ld\n", __func__, ucontrol->value.integer.value[0]);
	hp_impedance = ucontrol->value.integer.value[0];
	return 0;
}

void setHpGainZero(void)
{
	Ana_Set_Reg(ZCD_CON2, 0x8 << 7, 0x0f80);
	Ana_Set_Reg(ZCD_CON2, 0x8, 0x001f);
}

void SetSdmLevel(unsigned int level)
{
	Ana_Set_Reg(AFE_DL_SDM_CON1, level, 0xffffffff);
}


static void SetHprOffset(int OffsetTrimming)
{
	short Dccompsentation = 0;
	int DCoffsetValue = 0;
	unsigned short RegValue = 0;

	/* pr_debug("%s OffsetTrimming = %d\n", __func__, OffsetTrimming);*/
	DCoffsetValue = (OffsetTrimming * 10589 + 2048) / 4096;
	/* pr_debug("%s DCoffsetValue = %d\n", __func__, DCoffsetValue); */
	Dccompsentation = DCoffsetValue;
	RegValue = Dccompsentation;
	/* pr_debug("%s RegValue = 0x%x\n", __func__, RegValue); */
	Ana_Set_Reg(AFE_DL_DC_COMP_CFG1, RegValue, 0xffff);
}

static void SetHplOffset(int OffsetTrimming)
{
	short Dccompsentation = 0;
	int DCoffsetValue = 0;
	unsigned short RegValue = 0;

	/*pr_debug("%s OffsetTrimming = %d\n", __func__, OffsetTrimming);*/
	DCoffsetValue = (OffsetTrimming * 10589 + 2048) / 4096;
	/* pr_debug("%s DCoffsetValue = %d\n", __func__, DCoffsetValue); */
	Dccompsentation = DCoffsetValue;
	RegValue = Dccompsentation;
	/* pr_debug("%s RegValue = 0x%x\n", __func__, RegValue); */
	Ana_Set_Reg(AFE_DL_DC_COMP_CFG0, RegValue, 0xffff);
}

static void EnableDcCompensation(bool bEnable)
{
#ifndef EFUSE_HP_TRIM
	Ana_Set_Reg(AFE_DL_DC_COMP_CFG2, bEnable, 0x1);
#endif
}

static void SetHprOffsetTrim(void)
{
	int OffsetTrimming = mHprTrimOffset - TrimOffset;

	pr_debug("%s mHprTrimOffset = %d TrimOffset = %d\n", __func__, mHprTrimOffset, TrimOffset);
	SetHprOffset(OffsetTrimming);
}

static void SetHpLOffsetTrim(void)
{
	int OffsetTrimming = mHplTrimOffset - TrimOffset;

	pr_debug("%s mHprTrimOffset = %d TrimOffset = %d\n", __func__, mHplTrimOffset, TrimOffset);
	SetHplOffset(OffsetTrimming);
}

static void SetDcCompenSation(void)
{
#ifndef EFUSE_HP_TRIM
	SetHprOffsetTrim();
	SetHpLOffsetTrim();
	EnableDcCompensation(true);
#else				/* use efuse trim */
	Ana_Set_Reg(AUDDEC_ANA_CON2, 0x0800, 0x0800);	/* Enable trim circuit of HP */
	Ana_Set_Reg(AUDDEC_ANA_CON2, RG_AUDHPLTRIM_VAUDP15 << 3, 0x0078);	/* Trim offset voltage of HPL */
	Ana_Set_Reg(AUDDEC_ANA_CON2, RG_AUDHPRTRIM_VAUDP15 << 7, 0x0780);	/* Trim offset voltage of HPR */
	Ana_Set_Reg(AUDDEC_ANA_CON2, RG_AUDHPLFINETRIM_VAUDP15 << 12, 0x3000);	/* Fine trim offset voltage of HPL */
	Ana_Set_Reg(AUDDEC_ANA_CON2, RG_AUDHPRFINETRIM_VAUDP15 << 14, 0xC000);	/* Fine trim offset voltage of HPR */
#endif
}

static void SetDcCompenSation_SPKHP(void)
{
#ifdef EFUSE_HP_TRIM		/* use efuse trim */
	Ana_Set_Reg(AUDDEC_ANA_CON2, 0x0800, 0x0800);	/* Enable trim circuit of HP */
	Ana_Set_Reg(AUDDEC_ANA_CON2, RG_AUDHPLTRIM_VAUDP15_SPKHP << 3, 0x0078);
	/* Trim offset voltage of HPL */
	Ana_Set_Reg(AUDDEC_ANA_CON2, RG_AUDHPRTRIM_VAUDP15_SPKHP << 7, 0x0780);
	/* Trim offset voltage of HPR */
	Ana_Set_Reg(AUDDEC_ANA_CON2, RG_AUDHPLFINETRIM_VAUDP15_SPKHP << 12, 0x3000);
	/* Fine trim offset voltage of HPL */
	Ana_Set_Reg(AUDDEC_ANA_CON2, RG_AUDHPRFINETRIM_VAUDP15_SPKHP << 14, 0xC000);
	/* Fine trim offset voltage of HPR */
#endif
}


static void SetDCcoupleNP(int MicBias, int mode)
{
	pr_debug("%s MicBias= %d mode = %d\n", __func__, MicBias, mode);
	switch (mode) {
	case AUDIO_ANALOGUL_MODE_ACC:
	case AUDIO_ANALOGUL_MODE_DCC:
	case AUDIO_ANALOGUL_MODE_DMIC:
		{
			if (MicBias == AUDIO_MIC_BIAS0)
				Ana_Set_Reg(AUDENC_ANA_CON9, 0x0000, 0x0006);
			else if (MicBias == AUDIO_MIC_BIAS1)
				Ana_Set_Reg(AUDENC_ANA_CON9, 0x0000, 0x0060);
		}
		break;
	case AUDIO_ANALOGUL_MODE_DCCECMDIFF:
		{
			if (MicBias == AUDIO_MIC_BIAS0)
				Ana_Set_Reg(AUDENC_ANA_CON9, 0x0006, 0x0006);
			else if (MicBias == AUDIO_MIC_BIAS1)
				Ana_Set_Reg(AUDENC_ANA_CON9, 0x0060, 0x0060);
		}
		break;
	case AUDIO_ANALOGUL_MODE_DCCECMSINGLE:
		{
			if (MicBias == AUDIO_MIC_BIAS0)
				Ana_Set_Reg(AUDENC_ANA_CON9, 0x0002, 0x0006);
			else if (MicBias == AUDIO_MIC_BIAS1)
				Ana_Set_Reg(AUDENC_ANA_CON9, 0x0020, 0x0060);
		}
		break;
	default:
		break;
	}
}

uint32 GetULFrequency(uint32 frequency)
{
	uint32 Reg_value = 0;

	pr_debug("%s frequency =%d\n", __func__, frequency);
	switch (frequency) {
	case 8000:
	case 16000:
	case 32000:
		Reg_value = 0x0;
		break;
	case 48000:
		Reg_value = 0x1;
	default:
		break;

	}
	return Reg_value;
}


uint32 ULSampleRateTransform(uint32 SampleRate)
{
	switch (SampleRate) {
	case 8000:
		return 0;
	case 16000:
		return 1;
	case 32000:
		return 2;
	case 48000:
		return 3;
	default:
		break;
	}
	return 0;
}


static int mt63xx_codec_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *Daiport)
{
	/* pr_debug("+mt63xx_codec_startup name = %s number = %d\n", substream->name, substream->number); */
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE && substream->runtime->rate) {
		/* pr_debug("mt63xx_codec_startup set up SNDRV_PCM_STREAM_CAPTURE rate = %d\n",
		substream->runtime->rate); */
		mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC] = substream->runtime->rate;

	} else if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK && substream->runtime->rate) {
		/* pr_debug("mt63xx_codec_startup set up SNDRV_PCM_STREAM_PLAYBACK rate = %d\n",
		substream->runtime->rate); */
		mBlockSampleRate[AUDIO_ANALOG_DEVICE_OUT_DAC] = substream->runtime->rate;
	}
	/* pr_debug("-mt63xx_codec_startup name = %s number = %d\n", substream->name, substream->number); */
	return 0;
}

static int mt63xx_codec_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *Daiport)
{
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		/*pr_debug("mt63xx_codec_prepare set up SNDRV_PCM_STREAM_CAPTURE rate = %d\n",
		       substream->runtime->rate);*/
		mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC] = substream->runtime->rate;

	} else if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/*pr_debug("mt63xx_codec_prepare set up SNDRV_PCM_STREAM_PLAYBACK rate = %d\n",
		       substream->runtime->rate);*/
		mBlockSampleRate[AUDIO_ANALOG_DEVICE_OUT_DAC] = substream->runtime->rate;
	}
	return 0;
}

static int mt6323_codec_trigger(struct snd_pcm_substream *substream, int command,
				struct snd_soc_dai *Daiport)
{
	switch (command) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		break;
	}

	/* pr_debug("mt6323_codec_trigger command = %d\n ", command); */
	return 0;
}

static const struct snd_soc_dai_ops mt6323_aif1_dai_ops = {
	.startup = mt63xx_codec_startup,
	.prepare = mt63xx_codec_prepare,
	.trigger = mt6323_codec_trigger,
};

static struct snd_soc_dai_driver mtk_6331_dai_codecs[] = {
	{
	 .name = MT_SOC_CODEC_TXDAI_NAME,
	 .ops = &mt6323_aif1_dai_ops,
	 .playback = {
		      .stream_name = MT_SOC_DL1_STREAM_NAME,
		      .channels_min = 1,
		      .channels_max = 2,
		      .rates = SNDRV_PCM_RATE_8000_192000,
		      .formats = SND_SOC_ADV_MT_FMTS,
		      },
	 },
	{
	 .name = MT_SOC_CODEC_RXDAI_NAME,
	 .ops = &mt6323_aif1_dai_ops,
	 .capture = {
		     .stream_name = MT_SOC_UL1_STREAM_NAME,
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = SNDRV_PCM_RATE_8000_192000,
		     .formats = SND_SOC_ADV_MT_FMTS,
		     },
	 },
	{
	 .name = MT_SOC_CODEC_TDMRX_DAI_NAME,
	 .ops = &mt6323_aif1_dai_ops,
	 .capture = {
		     .stream_name = MT_SOC_TDM_CAPTURE_STREAM_NAME,
		     .channels_min = 2,
		     .channels_max = 8,
		     .rates = SNDRV_PCM_RATE_8000_192000,
		     .formats = (SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S8 |
				 SNDRV_PCM_FMTBIT_U16_LE | SNDRV_PCM_FMTBIT_S16_LE |
				 SNDRV_PCM_FMTBIT_U16_BE | SNDRV_PCM_FMTBIT_S16_BE |
				 SNDRV_PCM_FMTBIT_U24_LE | SNDRV_PCM_FMTBIT_S24_LE |
				 SNDRV_PCM_FMTBIT_U24_BE | SNDRV_PCM_FMTBIT_S24_BE |
				 SNDRV_PCM_FMTBIT_U24_3LE | SNDRV_PCM_FMTBIT_S24_3LE |
				 SNDRV_PCM_FMTBIT_U24_3BE | SNDRV_PCM_FMTBIT_S24_3BE |
				 SNDRV_PCM_FMTBIT_U32_LE | SNDRV_PCM_FMTBIT_S32_LE |
				 SNDRV_PCM_FMTBIT_U32_BE | SNDRV_PCM_FMTBIT_S32_BE),
		     },
	 },
	{
	 .name = MT_SOC_CODEC_I2S0TXDAI_NAME,
	 .ops = &mt6323_aif1_dai_ops,
	 .playback = {
		      .stream_name = MT_SOC_I2SDL1_STREAM_NAME,
		      .channels_min = 1,
		      .channels_max = 2,
		      .rate_min = 8000,
		      .rate_max = 192000,
		      .rates = SNDRV_PCM_RATE_8000_192000,
		      .formats = SND_SOC_ADV_MT_FMTS,
		      }
	 },
	{
	 .name = MT_SOC_CODEC_VOICE_MD1DAI_NAME,
	 .ops = &mt6323_aif1_dai_ops,
	 .playback = {
		      .stream_name = MT_SOC_VOICE_MD1_STREAM_NAME,
		      .channels_min = 1,
		      .channels_max = 2,
		      .rates = SNDRV_PCM_RATE_8000_48000,
		      .formats = SND_SOC_ADV_MT_FMTS,
		      },
	 .capture = {
		     .stream_name = MT_SOC_VOICE_MD1_STREAM_NAME,
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = SNDRV_PCM_RATE_8000_48000,
		     .formats = SND_SOC_ADV_MT_FMTS,
		     },
	 },
	{
	 .name = MT_SOC_CODEC_VOICE_MD2DAI_NAME,
	 .ops = &mt6323_aif1_dai_ops,
	 .playback = {
		      .stream_name = MT_SOC_VOICE_MD2_STREAM_NAME,
		      .channels_min = 1,
		      .channels_max = 2,
		      .rates = SNDRV_PCM_RATE_8000_48000,
		      .formats = SND_SOC_ADV_MT_FMTS,
		      },
	 .capture = {
		     .stream_name = MT_SOC_VOICE_MD2_STREAM_NAME,
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = SNDRV_PCM_RATE_8000_48000,
		     .formats = SND_SOC_ADV_MT_FMTS,
		     },
	 },
	{
	 .name = MT_SOC_CODEC_FMI2S2RXDAI_NAME,
	 .ops = &mt6323_aif1_dai_ops,
	 .playback = {
		      .stream_name = MT_SOC_FM_I2S2_STREAM_NAME,
		      .channels_min = 1,
		      .channels_max = 2,
		      .rates = SNDRV_PCM_RATE_8000_48000,
		      .formats = SND_SOC_ADV_MT_FMTS,
		      },
	 .capture = {
		     .stream_name = MT_SOC_FM_I2S2_RECORD_STREAM_NAME,
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = SNDRV_PCM_RATE_8000_48000,
		     .formats = SND_SOC_ADV_MT_FMTS,
		     },
	 },
	{
	 .name = MT_SOC_CODEC_FMMRGTXDAI_DUMMY_DAI_NAME,
	 .ops = &mt6323_aif1_dai_ops,
	 .playback = {
		      .stream_name = MT_SOC_FM_MRGTX_STREAM_NAME,
		      .channels_min = 1,
		      .channels_max = 2,
		      .rates = SNDRV_PCM_RATE_8000_48000,
		      .formats = SND_SOC_ADV_MT_FMTS,
		      },
	 },
	{
	 .name = MT_SOC_CODEC_ULDLLOOPBACK_NAME,
	 .ops = &mt6323_aif1_dai_ops,
	 .playback = {
		      .stream_name = MT_SOC_ULDLLOOPBACK_STREAM_NAME,
		      .channels_min = 1,
		      .channels_max = 2,
		      .rates = SNDRV_PCM_RATE_8000_48000,
		      .formats = SND_SOC_ADV_MT_FMTS,
		      },
	 .capture = {
		     .stream_name = MT_SOC_ULDLLOOPBACK_STREAM_NAME,
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = SNDRV_PCM_RATE_8000_48000,
		     .formats = SND_SOC_ADV_MT_FMTS,
		     },
	 },
	{
	 .name = MT_SOC_CODEC_STUB_NAME,
	 .ops = &mt6323_aif1_dai_ops,
	 .playback = {
		      .stream_name = MT_SOC_ROUTING_STREAM_NAME,
		      .channels_min = 1,
		      .channels_max = 2,
		      .rates = SNDRV_PCM_RATE_8000_192000,
		      .formats = SND_SOC_ADV_MT_FMTS,
		      },
	 },
	{
	 .name = MT_SOC_CODEC_RXDAI2_NAME,
	 .capture = {
		     .stream_name = MT_SOC_UL1DATA2_STREAM_NAME,
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = SNDRV_PCM_RATE_8000_192000,
		     .formats = SND_SOC_ADV_MT_FMTS,
		     },
	 },
	{
	 .name = MT_SOC_CODEC_MRGRX_DAI_NAME,
	 .ops = &mt6323_aif1_dai_ops,
	 .playback = {
		      .stream_name = MT_SOC_MRGRX_STREAM_NAME,
		      .channels_min = 1,
		      .channels_max = 8,
		      .rates = SNDRV_PCM_RATE_8000_192000,
		      .formats = SND_SOC_ADV_MT_FMTS,
		      },
	 .capture = {
		     .stream_name = MT_SOC_MRGRX_STREAM_NAME,
		     .channels_min = 1,
		     .channels_max = 8,
		     .rates = SNDRV_PCM_RATE_8000_192000,
		     .formats = SND_SOC_ADV_MT_FMTS,
		     },
	 },
	{
	 .name = MT_SOC_CODEC_FM_I2S_DAI_NAME,
	 .ops = &mt6323_aif1_dai_ops,
	 .playback = {
		      .stream_name = MT_SOC_FM_I2S_PLAYBACK_STREAM_NAME,
		      .channels_min = 1,
		      .channels_max = 8,
		      .rates = SNDRV_PCM_RATE_8000_192000,
		      .formats = SND_SOC_ADV_MT_FMTS,
		      },
	 },
	{
	 .name = MT_SOC_CODEC_TXDAI2_NAME,
	 .ops = &mt6323_aif1_dai_ops,
	 .playback = {
		      .stream_name = MT_SOC_DL2_STREAM_NAME,
		      .channels_min = 1,
		      .channels_max = 2,
		      .rates = SNDRV_PCM_RATE_8000_192000,
		      .formats = SND_SOC_ADV_MT_FMTS,
		      },
	 },
};


uint32 GetDLNewIFFrequency(unsigned int frequency)
{
	uint32 Reg_value = 0;
	/* pr_debug("AudioPlatformDevice ApplyDLNewIFFrequency ApplyDLNewIFFrequency = %d", frequency); */
	switch (frequency) {
	case 8000:
		Reg_value = 0;
		break;
	case 11025:
		Reg_value = 1;
		break;
	case 12000:
		Reg_value = 2;
		break;
	case 16000:
		Reg_value = 3;
		break;
	case 22050:
		Reg_value = 4;
		break;
	case 24000:
		Reg_value = 5;
		break;
	case 32000:
		Reg_value = 6;
		break;
	case 44100:
		Reg_value = 7;
		break;
	case 48000:
		Reg_value = 8;
		break;
	default:
		pr_warn("ApplyDLNewIFFrequency with frequency = %d", frequency);
	}
	return Reg_value;
}

static void TurnOnDacPower(void)
{
	pr_debug("TurnOnDacPower\n");
	audckbufEnable(true);
	ClsqEnable(true);
	Topck_Enable(true);
	udelay(250);
	NvregEnable(true);	/* 6752 moved to 0x0CEE */

	if (GetAdcStatus() == false)
		Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x003a, 0xffff);	/* power on clock */
	else
		Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x0000, 0xffff);	/* power on clock */


	/* set digital part */
	Ana_Set_Reg(AFUNC_AUD_CON2, 0x0006, 0xffff);	/* sdm audio fifo clock power on */
	Ana_Set_Reg(AFUNC_AUD_CON0, 0xc3a1, 0xffff);	/* scrambler clock on enable */
	Ana_Set_Reg(AFUNC_AUD_CON2, 0x0003, 0xffff);	/* sdm power on */
	Ana_Set_Reg(AFUNC_AUD_CON2, 0x000b, 0xffff);	/* sdm fifo enable */
	Ana_Set_Reg(AFE_DL_SDM_CON1, 0x001e, 0xffff);	/* set attenuation gain */
	Ana_Set_Reg(AFE_UL_DL_CON0, 0x0001, 0xffff);	/* [0] afe enable */

	Ana_Set_Reg(AFE_PMIC_NEWIF_CFG0,
		    GetDLNewIFFrequency(mBlockSampleRate[AUDIO_ANALOG_DEVICE_OUT_DAC]) << 12 |
		    0x330, 0xffff);
	Ana_Set_Reg(AFE_DL_SRC2_CON0_H,
		GetDLNewIFFrequency(mBlockSampleRate[AUDIO_ANALOG_DEVICE_OUT_DAC]) << 12 | 0x300, 0xffff);

	Ana_Set_Reg(AFE_DL_SRC2_CON0_L, 0x0001, 0xffff);	/* turn on dl */
	Ana_Set_Reg(PMIC_AFE_TOP_CON0, 0x0000, 0xffff);	/* set DL in normal path, not from sine gen table */
}

static void TurnOffDacPower(void)
{
	pr_debug("TurnOffDacPower\n");

	Ana_Set_Reg(AFE_DL_SRC2_CON0_L, 0x0000, 0xffff);	/* bit0, Turn off down-link */
	if (GetAdcStatus() == false)
		Ana_Set_Reg(AFE_UL_DL_CON0, 0x0000, 0xffff);	/* turn off afe */

	udelay(250);

	Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x0040, 0x0040);	/* down-link power down */

	Ana_Set_Reg(AUDNCP_CLKDIV_CON0, 0x0000, 0xffff);	/* Toggle RG_DIVCKS_CHG */
	Ana_Set_Reg(AUDNCP_CLKDIV_CON1, 0x0000, 0xffff);	/* Turn off DA_600K_NCP_VA18 */
	Ana_Set_Reg(AUDNCP_CLKDIV_CON3, 0x0001, 0xffff);	/* Disable NCP */
	/* upmu_set_rg_vio18_cal(0);// for MT6328 E1 VIO18 patch only */
	NvregEnable(false);
	ClsqEnable(false);
	Topck_Enable(false);
	audckbufEnable(false);
}

static void HeadsetVoloumeRestore(void)
{
	int index = 0, oldindex = 0, offset = 0, count = 1;

	pr_debug("%s\n", __func__);
	index = 8;

	oldindex = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTR];
	if (index > oldindex) {
		pr_debug("index = %d oldindex = %d\n", index, oldindex);
		offset = index - oldindex;
		while (offset > 0) {
			Ana_Set_Reg(ZCD_CON2, ((oldindex + count) << 7) | (oldindex + count),
				    0xf9f);
			offset--;
			count++;
			udelay(100);
		}
	} else {
		pr_debug("index = %d oldindex = %d\n", index, oldindex);
		offset = oldindex - index;
		while (offset > 0) {
			Ana_Set_Reg(ZCD_CON2, ((oldindex - count) << 7) | (oldindex - count),
				    0xf9f);
			offset--;
			count++;
			udelay(100);
		}
	}
	Ana_Set_Reg(ZCD_CON2, 0x0489, 0xf9f);
}

static void HeadsetVoloumeSet(void)
{
	int index = 0, oldindex = 0, offset = 0, count = 1;

	index = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTR];
	oldindex = 8;
	if (index > oldindex) {
		pr_debug("%s index = %d oldindex = %d\n", __func__ , index, oldindex);
		offset = index - oldindex;
		while (offset > 0) {
			Ana_Set_Reg(ZCD_CON2, ((oldindex + count) << 7) | (oldindex + count),
				    0xf9f);
			offset--;
			count++;
			udelay(200);
		}
	} else {
		pr_debug("%s index = %d oldindex = %d\n", __func__ , index, oldindex);
		offset = oldindex - index;
		while (offset > 0) {
			Ana_Set_Reg(ZCD_CON2, ((oldindex - count) << 7) | (oldindex - count),
				    0xf9f);
			offset--;
			count++;
			udelay(200);
		}
	}
	Ana_Set_Reg(ZCD_CON2, (index << 7) | (index), 0xf9f);
}

static void Audio_Amp_Change(int channels, bool enable)
{
	if (enable) {
		if (GetDLStatus() == false)
			TurnOnDacPower();

		/* here pmic analog control */
		if (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETL] == false
		    && mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETR] ==
		    false) {
			pr_debug("%s\n", __func__);

			/* upmu_set_rg_vio18_cal(1);// for MT6328 E1 VIO18 patch only */
			/* set analog part (HP playback) */
			Ana_Set_Reg(AUDNCP_CLKDIV_CON1, 0x0001, 0xffff);
			/* Turn on DA_600K_NCP_VA18 */
			Ana_Set_Reg(AUDNCP_CLKDIV_CON2, 0x002B, 0xffff);
			/* Set NCP clock as 604kHz // 26MHz/43 = 604KHz */
			Ana_Set_Reg(AUDNCP_CLKDIV_CON0, 0x0001, 0xffff);
			/* Toggle RG_DIVCKS_CHG */
			Ana_Set_Reg(AUDNCP_CLKDIV_CON4, 0x0000, 0xffff);
			/* Set NCP soft start mode as default mode */
			Ana_Set_Reg(AUDNCP_CLKDIV_CON3, 0x0000, 0xffff);
			/* Enable NCP */
			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0A41, 0xfeeb);
			/* Enable cap-less HC LDO (1.5V) & LDO VA33REFGEN */
			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC1, 0xfeeb);
			/* Enable cap-less LC LDO (1.5V) */
			Ana_Set_Reg(AUDDEC_ANA_CON7, 0x8000, 0x8000);
			/* Enable NV regulator (-1.5V) */
			Ana_Set_Reg(ZCD_CON0, 0x0000, 0xffff);
			/* Disable AUD_ZCD */
			Ana_Set_Reg(AUDDEC_ANA_CON0, 0xE080, 0xffff);
			/* Disable headphone, voice and short-ckt protection. HP MUX is opened, voice MUX is set mute */
			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC0, 0xfeeb);
			/* Enable IBIST */
			Ana_Set_Reg(AUDDEC_ANA_CON4, 0x0700, 0xffff);
			/* Enable HP & HS drivers bias circuit */
			Ana_Set_Reg(AUDDEC_ANA_CON5, 0x5490, 0xffff);
			/* HP/HS ibias & DR bias current optimization */
			udelay(50);
			Ana_Set_Reg(ZCD_CON2, 0x0F9F, 0xffff);
			/* Set HPR/HPL gain as minimum (~ -40dB) */
			Ana_Set_Reg(ZCD_CON3, 0x001F, 0xffff);
			/* Set voice gain as minimum (~ -40dB) */
			Ana_Set_Reg(AUDDEC_ANA_CON1, 0x0480, 0xffff);
			/* De_OSC of HP and enable output STBENH */
			Ana_Set_Reg(AUDDEC_ANA_CON1, 0x1480, 0xffff);
			/* De_OSC of voice, enable output STBENH */
			Ana_Set_Reg(AUDDEC_ANA_CON0, 0xE090, 0xffff);
			/* Enable voice driver */
			Ana_Set_Reg(AUDDEC_ANA_CON1, 0x14A0, 0xffff);
			/* Enable pre-charge buffer */
			udelay(50);
			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC2, 0xfeeb);
			/* Enable AUD_CLK */
			Ana_Set_Reg(AUDDEC_ANA_CON0, 0xE09F, 0xffff);
			/* Enable Audio DAC */

			/* Apply digital DC compensation value to DAC */
			Ana_Set_Reg(ZCD_CON2, 0x0489, 0xffff);
			/* Set HPR/HPL gain to -1dB, step by step */
			SetDcCompenSation();
			udelay(100);
			Ana_Set_Reg(AUDDEC_ANA_CON0, 0xF49F, 0xffff);
			/* Switch HP MUX to audio DAC */
			/* here may cause pop */
			/* msleep(1); //6752 removed */
			Ana_Set_Reg(AUDDEC_ANA_CON0, 0xF4FF, 0xffff);
			/* Enable HPR/HPL */
			udelay(50);
			Ana_Set_Reg(AUDDEC_ANA_CON1, 0x1480, 0xffff);
			/* Disable pre-charge buffer */
			Ana_Set_Reg(AUDDEC_ANA_CON1, 0x0480, 0xffff);
			/* Disable De_OSC of voice */
			/* Ana_Set_Reg(AUDDEC_ANA_CON0, 0xF46F, 0xffff); //Disable voice buffer & Open HS input MUX */
			Ana_Set_Reg(AUDDEC_ANA_CON0, 0xF46F, 0xffff);
			/* Disable voice buffer & Open HS input MUX  //MT6328 George table error */

			Ana_Set_Reg(AUDDEC_ANA_CON4, 0x0300, 0xffff);
			/* Disable HS drivers bias circuit */

			/* apply volume setting */
			HeadsetVoloumeSet();
		}

	} else {
		if (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETL] == false
		    && mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETR] ==
		    false) {
			pr_debug("Audio_Amp_Change off amp\n");
			HeadsetVoloumeRestore();	/* Set HPR/HPL gain as -1dB, step by step */
			Ana_Set_Reg(ZCD_CON2, 0x0F9F, 0xffff); /* Set HPR/HPL gain as minimum (~ -40dB) */
			Ana_Set_Reg(AUDDEC_ANA_CON0, 0xF40F, 0xffff);	/* Disable HPR/HPL */
		}

		if (GetDLStatus() == false) {
			Ana_Set_Reg(AUDDEC_ANA_CON4, 0x0000, 0xffff);
			/* Disable drivers bias circuit */
			Ana_Set_Reg(AUDDEC_ANA_CON0, 0x0000, 0xffff);
			/* Disable Audio DAC */
			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC0, 0xfeeb);
			/* Disable AUD_CLK, bit2/4/8 is for ADC, do not set */
			Ana_Set_Reg(AUDDEC_ANA_CON7, 0x0000, 0x8000);
			/* Disable NV regulator (-1.5V) */
			/*Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0001, 0xfeeb);*/
			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x02c1, 0xfeeb); /* for AUX detection */
			/* Disable cap-less LDOs (1.5V) & Disable IBIST */
			TurnOffDacPower();
		}
		EnableDcCompensation(false);
	}
}

static int Audio_AmpL_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("Audio_AmpL_Get = %d\n",
	       mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETL]);
	ucontrol->value.integer.value[0] =
	    mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETL];
	return 0;
}

static int Audio_AmpL_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	mutex_lock(&Ana_Ctrl_Mutex);

	pr_debug("%s() gain = %ld\n ", __func__, ucontrol->value.integer.value[0]);
	if ((ucontrol->value.integer.value[0] == true)
	    && (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETL] == false)) {
		Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_LEFT1, true);
		mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETL] =
		    ucontrol->value.integer.value[0];
	} else if ((ucontrol->value.integer.value[0] == false)
		   && (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETL] ==
		       true)) {
		mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETL] =
		    ucontrol->value.integer.value[0];
		Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_LEFT1, false);
	}
	mutex_unlock(&Ana_Ctrl_Mutex);
	return 0;
}

static int Audio_AmpR_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("Audio_AmpR_Get = %d\n",
	       mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETR]);
	ucontrol->value.integer.value[0] =
	    mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETR];
	return 0;
}

static int Audio_AmpR_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	mutex_lock(&Ana_Ctrl_Mutex);

	pr_debug("%s()\n", __func__);
	if ((ucontrol->value.integer.value[0] == true)
	    && (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETR] == false)) {
		Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_RIGHT1, true);
		mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETR] =
		    ucontrol->value.integer.value[0];
	} else if ((ucontrol->value.integer.value[0] == false)
		   && (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETR] ==
		       true)) {
		mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_HEADSETR] =
		    ucontrol->value.integer.value[0];
		Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_RIGHT1, false);
	}
	mutex_unlock(&Ana_Ctrl_Mutex);
	return 0;
}


static void Voice_Amp_Change(bool enable)
{
	if (enable) {
		pr_debug("%s\n", __func__);
		if (GetDLStatus() == false) {
			TurnOnDacPower();
			pr_debug("Voice_Amp_Change on amp\n");

			/* upmu_set_rg_vio18_cal(1);// for MT6328 E1 VIO18 patch only */
			/* set analog part (voice HS playback) */
			Ana_Set_Reg(AUDNCP_CLKDIV_CON1, 0x0001, 0xffff);
			/* Turn on DA_600K_NCP_VA18 */
			Ana_Set_Reg(AUDNCP_CLKDIV_CON2, 0x002B, 0xffff);
			/* Set NCP clock as 604kHz // 26MHz/43 = 604KHz */
			Ana_Set_Reg(AUDNCP_CLKDIV_CON0, 0x0001, 0xffff);
			/* Toggle RG_DIVCKS_CHG */
			Ana_Set_Reg(AUDNCP_CLKDIV_CON4, 0x0000, 0xffff);
			/* Set NCP soft start mode as default mode */
			Ana_Set_Reg(AUDNCP_CLKDIV_CON3, 0x0000, 0xffff);
			/* Enable NCP */
			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0A40, 0xfeeb);
			/* Enable cap-less HC LDO (1.5V) */
			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC0, 0xfeeb);
			/* Enable cap-less LC LDO (1.5V) */
			Ana_Set_Reg(AUDDEC_ANA_CON7, 0x8000, 0x8000);
			/* Enable NV regulator (-1.5V) */
			Ana_Set_Reg(ZCD_CON0, 0x0000, 0xffff);
			/* Disable AUD_ZCD */
			Ana_Set_Reg(AUDDEC_ANA_CON0, 0xE080, 0xffff);
			/* Disable headphone, voice and short-ckt protection. HP MUX is opened, voice MUX is set mute */
			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC0, 0xfeeb);
			/* Enable IBIST */
			Ana_Set_Reg(AUDDEC_ANA_CON4, 0x0400, 0xffff);
			/* Enable HS drivers bias circuit */
			Ana_Set_Reg(AUDDEC_ANA_CON5, 0x5490, 0xffff);
			/* HP/HS ibias & DR bias current optimization */
			udelay(50);
			Ana_Set_Reg(ZCD_CON3, 0x001F, 0xffff);
			/* Set voice gain as minimum (~ -40dB) */
			Ana_Set_Reg(AUDDEC_ANA_CON1, 0x1000, 0xffff);
			/* De_OSC of voice, enable output STBENH */
			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC2, 0xfeeb);
			/* Enable AUD_CLK */
			Ana_Set_Reg(AUDDEC_ANA_CON0, 0xE089, 0xffff);
			/* Enable Audio DAC */

			/* Apply digital DC compensation value to DAC */
			Ana_Set_Reg(AUDDEC_ANA_CON0, 0xE109, 0xffff);
			/* Switch HS MUX to audio DAC */
			Ana_Set_Reg(AUDDEC_ANA_CON0, 0xE119, 0xffff);
			/* Enable voice driver */
			udelay(50);
			Ana_Set_Reg(ZCD_CON3, 0x0000, 0xffff);
			/* Set HS gain to +8dB(for SMT), step by step //0x0009 for 0dB Georoge */
		}
	} else {
		pr_debug("turn off ampL\n");
		Ana_Set_Reg(AUDDEC_ANA_CON0, 0xE109, 0xffff);	/* Disable voice driver */

		if (GetDLStatus() == false) {
			Ana_Set_Reg(AUDDEC_ANA_CON4, 0x0000, 0xffff);
			/* Disable drivers bias circuit */
			Ana_Set_Reg(AUDDEC_ANA_CON0, 0x0000, 0xffff);
			/* Disable Audio DAC */
			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC0, 0xfeeb);
			/* Disable AUD_CLK, bit2/4/8 is for ADC, do not set */
			Ana_Set_Reg(AUDDEC_ANA_CON7, 0x0000, 0x8000);
			/* Disable NV regulator (-1.5V) */
			/*Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0001, 0xfeeb);*/
			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x02c1, 0xfeeb); /* for AUX detection */
			/* Disable cap-less LDOs (1.5V) & Disable IBIST */

			TurnOffDacPower();
		}
	}
}

static int Voice_Amp_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("Voice_Amp_Get = %d\n",
	       mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_EARPIECEL]);
	ucontrol->value.integer.value[0] =
	    mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_EARPIECEL];
	return 0;
}

static int Voice_Amp_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	mutex_lock(&Ana_Ctrl_Mutex);
	pr_debug("%s()\n", __func__);
	if ((ucontrol->value.integer.value[0] == true)
	    && (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_EARPIECEL] == false)) {
		Voice_Amp_Change(true);
		mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_EARPIECEL] =
		    ucontrol->value.integer.value[0];
	} else if ((ucontrol->value.integer.value[0] == false)
		   && (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_EARPIECEL] ==
		       true)) {
		mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_EARPIECEL] =
		    ucontrol->value.integer.value[0];
		Voice_Amp_Change(false);
	}
	mutex_unlock(&Ana_Ctrl_Mutex);
	return 0;
}

static void Speaker_Amp_Change(bool enable)
{
	if (enable) {
		if (GetDLStatus() == false)
			TurnOnDacPower();

		pr_debug("%s\n", __func__);
		/* upmu_set_rg_vio18_cal(1);// for MT6328 E1 VIO18 patch only */
		/* ClassAB spk pmic analog control */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON1, 0x0001, 0xffff);
		/* Turn on DA_600K_NCP_VA18 */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON2, 0x002B, 0xffff);
		/* Set NCP clock as 604kHz // 26MHz/43 = 604KHz */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON0, 0x0001, 0xffff);
		/* Toggle RG_DIVCKS_CHG */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON4, 0x0000, 0xffff);
		/* Set NCP soft start mode as default mode */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON3, 0x0000, 0xffff);
		/* Enable NCP */
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0A41, 0xfeeb);
		/* Enable cap-less HC LDO (1.5V) */
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC1, 0xfeeb);
		/* Enable cap-less LC LDO (1.5V) */
		Ana_Set_Reg(AUDDEC_ANA_CON7, 0x8000, 0x8000);
		/* Enable NV regulator (-1.5V) */
		Ana_Set_Reg(ZCD_CON0, 0x0000, 0xffff);
		/* Disable AUD_ZCD */

		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC0, 0xfeeb);
		/* Enable IBIST */
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC2, 0xfeeb);
		/* Enable AUD_CLK */
		Ana_Set_Reg(AUDDEC_ANA_CON0, 0x0009, 0x0009);
		/* Enable Audio DAC Lch */

		/* SetDcCompenSation(); */

		Ana_Set_Reg(ZCD_CON0, 0x0400, 0xffff);
		/* Disable IVBUF_ZCD */
		Ana_Set_Reg(ZCD_CON4, 0x0705, 0xffff);
		/* Set 0dB IV buffer gain */
		Ana_Set_Reg(SPK_ANA_CON3, 0x0100, 0xffff);
		/* Set IV buffer MUX select */
		Ana_Set_Reg(SPK_ANA_CON3, 0x0110, 0xffff);
		/* Enable IV buffer */
		Ana_Set_Reg(TOP_CKPDN_CON2_CLR, 0x0003, 0xffff);
		/* Enable ClassAB/D clock */

#ifdef CONFIG_MTK_SPEAKER
		if (Speaker_mode == AUDIO_SPEAKER_MODE_D)
			Speaker_ClassD_Open();
		else if (Speaker_mode == AUDIO_SPEAKER_MODE_AB)
			Speaker_ClassAB_Open();
		else if (Speaker_mode == AUDIO_SPEAKER_MODE_RECEIVER)
			Speaker_ReveiverMode_Open();
#endif
		Apply_Speaker_Gain();
	} else {
		pr_debug("turn off Speaker_Amp_Change\n");
#ifdef CONFIG_MTK_SPEAKER
		if (Speaker_mode == AUDIO_SPEAKER_MODE_D)
			Speaker_ClassD_close();
		else if (Speaker_mode == AUDIO_SPEAKER_MODE_AB)
			Speaker_ClassAB_close();
		else if (Speaker_mode == AUDIO_SPEAKER_MODE_RECEIVER)
			Speaker_ReveiverMode_close();

#endif
		Ana_Set_Reg(TOP_CKPDN_CON2_SET, 0x0003, 0xffff);
		/* Disable ClassAB/D clock */
		Ana_Set_Reg(SPK_ANA_CON3, 0x0, 0xffff);
		/* Disable IV buffer, Set IV buffer MUX select open/open */
		Ana_Set_Reg(ZCD_CON4, 0x0707, 0xffff);
		/* Set min -2dB IV buffer gain */

		if (GetDLStatus() == false) {
			Ana_Set_Reg(AUDDEC_ANA_CON4, 0x0000, 0xffff);
			/* Disable drivers bias circuit */
			Ana_Set_Reg(AUDDEC_ANA_CON0, 0x0000, 0xffff);
			/* Disable Audio DAC */
			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC0, 0xfeeb);
			/* Disable AUD_CLK, bit2/4/8 is for ADC, do not set */
			Ana_Set_Reg(AUDDEC_ANA_CON7, 0x0000, 0x8000);
			/* Disable NV regulator (-1.5V) */
			/*Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0001, 0xfeeb);*/
			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x02c1, 0xfeeb); /* for AUX detection */
			/* Disable cap-less LDOs (1.5V) & Disable IBIST */

			TurnOffDacPower();
		}
	}
}

static int Speaker_Amp_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	ucontrol->value.integer.value[0] =
	    mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_SPEAKERL];
	return 0;
}

static int Speaker_Amp_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s() value = %ld\n ", __func__, ucontrol->value.integer.value[0]);
	if ((ucontrol->value.integer.value[0] == true)
	    && (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_SPEAKERL] == false)) {
		Speaker_Amp_Change(true);
		mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_SPEAKERL] =
		    ucontrol->value.integer.value[0];
	} else if ((ucontrol->value.integer.value[0] == false)
		   && (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_SPEAKERL] ==
		       true)) {
		mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_SPEAKERL] =
		    ucontrol->value.integer.value[0];
		Speaker_Amp_Change(false);
	}
	return 0;
}



#ifdef CONFIG_OF

#define GAP (2)			/* unit: us */
#if defined(CONFIG_MTK_LEGACY)
#define AW8736_MODE3 /*0.8w*/ \
do { \
	mt_set_gpio_out(pin_extspkamp, GPIO_OUT_ONE); \
	udelay(GAP); \
	mt_set_gpio_out(pin_extspkamp, GPIO_OUT_ZERO); \
	udelay(GAP); \
	mt_set_gpio_out(pin_extspkamp, GPIO_OUT_ONE); \
	udelay(GAP); \
	mt_set_gpio_out(pin_extspkamp, GPIO_OUT_ZERO); \
	udelay(GAP); \
	mt_set_gpio_out(pin_extspkamp, GPIO_OUT_ONE); \
} while (0)
#endif

#define NULL_PIN_DEFINITION    (-1)
static void Ext_Speaker_Amp_Change(bool enable)
{
#define SPK_WARM_UP_TIME        (25)	/* unit is ms */
#ifndef CONFIG_FPGA_EARLY_PORTING
#if defined(CONFIG_MTK_LEGACY)
	int ret;

	ret = GetGPIO_Info(5, &pin_extspkamp, &pin_mode_extspkamp);
	if (ret < 0) {
		pr_warn("Ext_Speaker_Amp_Change GetGPIO_Info FAIL!!!\n");
		return;
	}
#endif
	if (enable) {
		pr_debug("Ext_Speaker_Amp_Change ON+\n");
#ifndef CONFIG_MTK_SPEAKER
#if defined(CONFIG_MTK_LEGACY)

		ret = GetGPIO_Info(10, &pin_extspkamp_2, &pin_mode_extspkamp_2);
		pr_debug("Ext_Speaker_Amp_Change ON set GPIO\n");
		mt_set_gpio_mode(pin_extspkamp, GPIO_MODE_00);	/* GPIO117: DPI_D3, mode 0 */
		mt_set_gpio_pull_enable(pin_extspkamp, GPIO_PULL_ENABLE);
		mt_set_gpio_dir(pin_extspkamp, GPIO_DIR_OUT);	/* output */
		mt_set_gpio_out(pin_extspkamp, GPIO_OUT_ZERO);	/* low disable */
		if (pin_extspkamp_2 != NULL_PIN_DEFINITION) {
			mt_set_gpio_mode(pin_extspkamp_2, GPIO_MODE_00);	/* GPIO117: DPI_D3, mode 0 */
			mt_set_gpio_pull_enable(pin_extspkamp_2, GPIO_PULL_ENABLE);
			mt_set_gpio_dir(pin_extspkamp_2, GPIO_DIR_OUT);	/* output */
			mt_set_gpio_out(pin_extspkamp_2, GPIO_OUT_ZERO);	/* low disable */
		}
#else
		AudDrv_GPIO_EXTAMP_Select(false);
		AudDrv_GPIO_EXTAMP2_Select(false);
#endif /*CONFIG_MTK_LEGACY*/

		/*udelay(1000);*/
		usleep_range(1*1000, 20*1000);
#if defined(CONFIG_MTK_LEGACY)
		mt_set_gpio_dir(pin_extspkamp, GPIO_DIR_OUT);	/* output */
		if (pin_extspkamp_2 != NULL_PIN_DEFINITION)
			mt_set_gpio_dir(pin_extspkamp_2, GPIO_DIR_OUT);	/* output */

#ifdef AW8736_MODE_CTRL
		AW8736_MODE3;
#else
		mt_set_gpio_out(pin_extspkamp, GPIO_OUT_ONE);	/* high enable */
#endif /*AW8736_MODE_CTRL*/
		if (pin_extspkamp_2 != NULL_PIN_DEFINITION)
			mt_set_gpio_out(pin_extspkamp_2, GPIO_OUT_ONE);	/* high enable */
#else
		AudDrv_GPIO_EXTAMP_Select(true);
		AudDrv_GPIO_EXTAMP2_Select(true);
#endif /*CONFIG_MTK_LEGACY*/
		msleep(SPK_WARM_UP_TIME);
#endif
		pr_debug("Ext_Speaker_Amp_Change ON-\n");
	} else {
		pr_debug("Ext_Speaker_Amp_Change OFF+\n");
#ifndef CONFIG_MTK_SPEAKER
#if defined(CONFIG_MTK_LEGACY)
		ret = GetGPIO_Info(10, &pin_extspkamp_2, &pin_mode_extspkamp_2);
		/* mt_set_gpio_mode(pin_extspkamp, GPIO_MODE_00); //GPIO117: DPI_D3, mode 0 */
		mt_set_gpio_dir(pin_extspkamp, GPIO_DIR_OUT);	/* output */
		mt_set_gpio_out(pin_extspkamp, GPIO_OUT_ZERO);	/* low disbale */
		if (pin_extspkamp_2 != NULL_PIN_DEFINITION) {
			mt_set_gpio_dir(pin_extspkamp_2, GPIO_DIR_OUT);	/* output */
			mt_set_gpio_out(pin_extspkamp_2, GPIO_OUT_ZERO);	/* low disbale */
		}
#else
		AudDrv_GPIO_EXTAMP_Select(false);
		AudDrv_GPIO_EXTAMP2_Select(false);
#endif
		udelay(500);
#endif
		pr_debug("Ext_Speaker_Amp_Change OFF-\n");
	}
#endif
}

#else /*CONFIG_OF*/
#ifndef CONFIG_MTK_SPEAKER
#ifdef AW8736_MODE_CTRL
/* 0.75us<TL<10us; 0.75us<TH<10us */
#define GAP (2)			/* unit: us */
/*1.2w*/
static void AW8736_MODE1(void)
{
	mt_set_gpio_out(GPIO_EXT_SPKAMP_EN_PIN, GPIO_OUT_ONE);
}
/*1.0w*/
static void AW8736_MODE2(void)
{
	do {
		mt_set_gpio_out(GPIO_EXT_SPKAMP_EN_PIN, GPIO_OUT_ONE);
		udelay(GAP);
		mt_set_gpio_out(GPIO_EXT_SPKAMP_EN_PIN, GPIO_OUT_ZERO);
		udelay(GAP);
		mt_set_gpio_out(GPIO_EXT_SPKAMP_EN_PIN, GPIO_OUT_ONE);
	} while (0)
}
/*0.8w*/
static void AW8736_MODE3(void)
{
	do {
		mt_set_gpio_out(GPIO_EXT_SPKAMP_EN_PIN, GPIO_OUT_ONE);
		udelay(GAP);
		mt_set_gpio_out(GPIO_EXT_SPKAMP_EN_PIN, GPIO_OUT_ZERO);
		udelay(GAP);
		mt_set_gpio_out(GPIO_EXT_SPKAMP_EN_PIN, GPIO_OUT_ONE);
		udelay(GAP);
		mt_set_gpio_out(GPIO_EXT_SPKAMP_EN_PIN, GPIO_OUT_ZERO);
		udelay(GAP);
		mt_set_gpio_out(GPIO_EXT_SPKAMP_EN_PIN, GPIO_OUT_ONE);
	} while (0)
}

/*it depends on THD, range: 1.5 ~ 2.0w*/
static void AW8736_MODE4(void)
{
	do {
		mt_set_gpio_out(GPIO_EXT_SPKAMP_EN_PIN, GPIO_OUT_ONE);
		udelay(GAP);
		mt_set_gpio_out(GPIO_EXT_SPKAMP_EN_PIN, GPIO_OUT_ZERO);
		udelay(GAP);
		mt_set_gpio_out(GPIO_EXT_SPKAMP_EN_PIN, GPIO_OUT_ONE);
		udelay(GAP);
		mt_set_gpio_out(GPIO_EXT_SPKAMP_EN_PIN, GPIO_OUT_ZERO);
		udelay(GAP);
		mt_set_gpio_out(GPIO_EXT_SPKAMP_EN_PIN, GPIO_OUT_ONE);
		udelay(GAP);
		mt_set_gpio_out(GPIO_EXT_SPKAMP_EN_PIN, GPIO_OUT_ZERO);
		udelay(GAP);
		mt_set_gpio_out(GPIO_EXT_SPKAMP_EN_PIN, GPIO_OUT_ONE);
	} while (0)
}
#endif
#endif

static void Ext_Speaker_Amp_Change(bool enable)
{
#define SPK_WARM_UP_TIME        (25)	/* unit is ms */
#ifndef CONFIG_FPGA_EARLY_PORTING

	if (enable) {
		pr_debug("Ext_Speaker_Amp_Change ON+\n");
#ifndef CONFIG_MTK_SPEAKER
		pr_debug("Ext_Speaker_Amp_Change ON set GPIO\n");
		mt_set_gpio_mode(GPIO_EXT_SPKAMP_EN_PIN, GPIO_MODE_00);	/* GPIO117: DPI_D3, mode 0 */
		mt_set_gpio_pull_enable(GPIO_EXT_SPKAMP_EN_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_dir(GPIO_EXT_SPKAMP_EN_PIN, GPIO_DIR_OUT);	/* output */
		mt_set_gpio_out(GPIO_EXT_SPKAMP_EN_PIN, GPIO_OUT_ZERO);	/* low disable */
		udelay(1000);
		mt_set_gpio_dir(GPIO_EXT_SPKAMP_EN_PIN, GPIO_DIR_OUT);	/* output */

#ifdef AW8736_MODE_CTRL
		AW8736_MODE3();
#else
		mt_set_gpio_out(GPIO_EXT_SPKAMP_EN_PIN, GPIO_OUT_ONE);	/* high enable */
#endif

		msleep(SPK_WARM_UP_TIME);
#endif
		pr_debug("Ext_Speaker_Amp_Change ON-\n");
	} else {
		pr_debug("Ext_Speaker_Amp_Change OFF+\n");
#ifndef CONFIG_MTK_SPEAKER
		/* mt_set_gpio_mode(GPIO_EXT_SPKAMP_EN_PIN, GPIO_MODE_00); //GPIO117: DPI_D3, mode 0 */
		mt_set_gpio_dir(GPIO_EXT_SPKAMP_EN_PIN, GPIO_DIR_OUT);	/* output */
		mt_set_gpio_out(GPIO_EXT_SPKAMP_EN_PIN, GPIO_OUT_ZERO);	/* low disbale */
		udelay(500);
#endif
		pr_debug("Ext_Speaker_Amp_Change OFF-\n");
	}
#endif
}
#endif

static int Ext_Speaker_Amp_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	ucontrol->value.integer.value[0] =
	    mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_EXTSPKAMP];
	return 0;
}

static int Ext_Speaker_Amp_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{

	pr_debug("%s() gain = %ld\n ", __func__, ucontrol->value.integer.value[0]);
	if (ucontrol->value.integer.value[0]) {
		Ext_Speaker_Amp_Change(true);
		mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_EXTSPKAMP] =
		    ucontrol->value.integer.value[0];
	} else {
		mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_EXTSPKAMP] =
		    ucontrol->value.integer.value[0];
		Ext_Speaker_Amp_Change(false);
	}
	return 0;
}

static void Receiver_Speaker_Switch_Change(bool enable)
{
#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef CONFIG_OF
#if defined(CONFIG_MTK_LEGACY)

	int ret;

	pr_debug("%s\n", __func__);

	if (enable) {
		pr_debug("switch to receiver\n");
		ret = GetGPIO_Info(11, &pin_rcvspkswitch, &pin_mode_rcvspkswitch);
		if (ret < 0) {
			pr_warn("Receiver_Speaker_Switch_Change GetGPIO_Info FAIL!!!\n");
			return;
		}
		mt_set_gpio_mode(pin_rcvspkswitch, GPIO_MODE_00);
		mt_set_gpio_pull_enable(pin_rcvspkswitch, GPIO_PULL_ENABLE);
		mt_set_gpio_dir(pin_rcvspkswitch, GPIO_DIR_OUT);	/* output */
		mt_set_gpio_out(pin_rcvspkswitch, GPIO_OUT_ZERO);	/* switch to receiver */

	} else {
		pr_debug("switch to speaker\n");
		ret = GetGPIO_Info(11, &pin_rcvspkswitch, &pin_mode_rcvspkswitch);
		if (ret < 0) {
			pr_warn("Receiver_Speaker_Switch_Change GetGPIO_Info FAIL!!!\n");
			return;
		}
		mt_set_gpio_mode(pin_rcvspkswitch, GPIO_MODE_00);
		mt_set_gpio_pull_enable(pin_rcvspkswitch, GPIO_PULL_ENABLE);
		mt_set_gpio_dir(pin_rcvspkswitch, GPIO_DIR_OUT);	/* output */
		mt_set_gpio_out(pin_rcvspkswitch, GPIO_OUT_ONE);	/* switch to speaker */

	}
#else
	pr_debug("%s\n", __func__);

	if (enable)
		AudDrv_GPIO_RCVSPK_Select(true);
	else
		AudDrv_GPIO_RCVSPK_Select(false);
#endif
#endif
#endif
}

static int Receiver_Speaker_Switch_Get(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s() : %d\n", __func__,
	       mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_RECEIVER_SPEAKER_SWITCH]);
	ucontrol->value.integer.value[0] =
	    mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_RECEIVER_SPEAKER_SWITCH];
	return 0;
}

static int Receiver_Speaker_Switch_Set(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	if ((ucontrol->value.integer.value[0] == true)
	    && (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_RECEIVER_SPEAKER_SWITCH] ==
		false)) {
		Receiver_Speaker_Switch_Change(true);
		mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_RECEIVER_SPEAKER_SWITCH] =
		    ucontrol->value.integer.value[0];
	} else if ((ucontrol->value.integer.value[0] == false)
		   && (mCodec_data->
		       mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_RECEIVER_SPEAKER_SWITCH] ==
		       true)) {
		mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_RECEIVER_SPEAKER_SWITCH] =
		    ucontrol->value.integer.value[0];
		Receiver_Speaker_Switch_Change(false);
	}
	return 0;
}

static void Headset_Speaker_Amp_Change(bool enable)
{
	if (enable) {
		if (GetDLStatus() == false)
			TurnOnDacPower();

		pr_debug("turn on Speaker_Amp_Change\n");
		/* here pmic analog control */
		/* upmu_set_rg_vio18_cal(1);// for MT6328 E1 VIO18 patch only */
		/* set analog part (HP playback) */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON1, 0x0001, 0xffff);
		/* Turn on DA_600K_NCP_VA18 */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON2, 0x002B, 0xffff);
		/* Set NCP clock as 604kHz // 26MHz/43 = 604KHz */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON0, 0x0001, 0xffff);
		/* Toggle RG_DIVCKS_CHG */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON4, 0x0000, 0xffff);
		/* Set NCP soft start mode as default mode */
		Ana_Set_Reg(AUDNCP_CLKDIV_CON3, 0x0000, 0xffff);
		/* Enable NCP */
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0A41, 0xfeeb);
		/* Enable cap-less HC LDO (1.5V) & LDO VA33REFGEN */
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC1, 0xfeeb);
		/* Enable cap-less LC LDO (1.5V) */
		Ana_Set_Reg(AUDDEC_ANA_CON7, 0x8000, 0x8000);
		/* Enable NV regulator (-1.5V) */
		Ana_Set_Reg(ZCD_CON0, 0x0000, 0xffff);
		/* Disable AUD_ZCD */
		Ana_Set_Reg(AUDDEC_ANA_CON0, 0xE080, 0xffff);
		/* Disable headphone, voice and short-ckt protection. HP MUX is opened, voice MUX is set mute */
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC0, 0xfeeb);
		/* Enable IBIST */
		Ana_Set_Reg(AUDDEC_ANA_CON4, 0x0700, 0xffff);
		/* Enable HP & HS drivers bias circuit */
		Ana_Set_Reg(AUDDEC_ANA_CON5, 0x5490, 0xffff);
		/* HP/HS ibias & DR bias current optimization */
		udelay(50);
		Ana_Set_Reg(ZCD_CON2, 0x0F9F, 0xffff);
		/* Set HPR/HPL gain as minimum (~ -40dB) */
		Ana_Set_Reg(ZCD_CON3, 0x001F, 0xffff);
		/* Set voice gain as minimum (~ -40dB) */
		Ana_Set_Reg(AUDDEC_ANA_CON1, 0x0480, 0xffff);
		/* De_OSC of HP and enable output STBENH */
		Ana_Set_Reg(AUDDEC_ANA_CON1, 0x1480, 0xffff);
		/* De_OSC of voice, enable output STBENH */
		Ana_Set_Reg(AUDDEC_ANA_CON0, 0xE090, 0xffff);
		/* Enable voice driver */
		Ana_Set_Reg(AUDDEC_ANA_CON1, 0x14A0, 0xffff);
		/* Enable pre-charge buffer */
		udelay(50);
		Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC2, 0xfeeb);
		/* Enable AUD_CLK */
		Ana_Set_Reg(AUDDEC_ANA_CON0, 0xE09F, 0xffff);
		/* Enable Audio DAC */

		/* Apply digital DC compensation value to DAC */
		Ana_Set_Reg(ZCD_CON2, 0x0489, 0xffff);
		/* Set HPR/HPL gain to -1dB, step by step */
		SetDcCompenSation_SPKHP();
		udelay(100);
		Ana_Set_Reg(AUDDEC_ANA_CON0, 0xF29F, 0xffff);
		/* R hp input mux select "Audio playback", L hp input mux select "LoudSPK playback" */
		/* here may cause pop? */
		/* msleep(1); //6752 removed */
		Ana_Set_Reg(AUDDEC_ANA_CON0, 0xF2FF, 0xffff);
		/* Enable HPR/HPL */
		udelay(50);
		Ana_Set_Reg(AUDDEC_ANA_CON1, 0x1480, 0xffff);
		/* Disable pre-charge buffer */
		Ana_Set_Reg(AUDDEC_ANA_CON1, 0x0480, 0xffff);
		/* Disable De_OSC of voice */
		Ana_Set_Reg(AUDDEC_ANA_CON0, 0xF26F, 0xffff);
		/* Disable voice buffer & Open HS input MUX */
		Ana_Set_Reg(AUDDEC_ANA_CON4, 0x0300, 0xffff);
		/* Disable HS drivers bias circuit */

		/* ClassAB spk pmic analog control */
		Ana_Set_Reg(ZCD_CON0, 0x0400, 0xffff);
		/* Disable IVBUF_ZCD */
		Ana_Set_Reg(ZCD_CON4, 0x0705, 0xffff);
		/* Set 0dB IV buffer gain */
		Ana_Set_Reg(SPK_ANA_CON3, 0x0100, 0xffff);
		/* Set IV buffer MUX select */
		Ana_Set_Reg(SPK_ANA_CON3, 0x0110, 0xffff);
		/* Enable IV buffer */
		Ana_Set_Reg(TOP_CKPDN_CON2_CLR, 0x0003, 0xffff);
		/* Enable ClassAB/D clock */

#ifdef CONFIG_MTK_SPEAKER
		if (Speaker_mode == AUDIO_SPEAKER_MODE_D)
			Speaker_ClassD_Open();
		else if (Speaker_mode == AUDIO_SPEAKER_MODE_AB)
			Speaker_ClassAB_Open();
		else if (Speaker_mode == AUDIO_SPEAKER_MODE_RECEIVER)
			Speaker_ReveiverMode_Open();

#endif
		/* apply volume setting */
		HeadsetVoloumeSet();
		Apply_Speaker_Gain();
	} else {
#ifdef CONFIG_MTK_SPEAKER
		if (Speaker_mode == AUDIO_SPEAKER_MODE_D)
			Speaker_ClassD_close();
		else if (Speaker_mode == AUDIO_SPEAKER_MODE_AB)
			Speaker_ClassAB_close();
		else if (Speaker_mode == AUDIO_SPEAKER_MODE_RECEIVER)
			Speaker_ReveiverMode_close();

#endif
		Ana_Set_Reg(TOP_CKPDN_CON2_SET, 0x0003, 0xffff);
		/* Disable ClassAB/D clock */
		Ana_Set_Reg(SPK_ANA_CON3, 0x0, 0xffff);
		/* Disable IV buffer, Set IV buffer MUX select open/open */
		Ana_Set_Reg(ZCD_CON4, 0x0707, 0xffff);
		/* Set min -2dB IV buffer gain */

		HeadsetVoloumeRestore();/* Set HPR/HPL gain as 0dB, step by step */
		Ana_Set_Reg(ZCD_CON2, 0x0F9F, 0xffff); /* Set HPR/HPL gain as minimum (~ -40dB) */
		Ana_Set_Reg(AUDDEC_ANA_CON0, 0xF20F, 0xffff);	/* Disable HPR/HPL */
		if (GetDLStatus() == false) {
			Ana_Set_Reg(AUDDEC_ANA_CON4, 0x0000, 0xffff);
			/* Disable drivers bias circuit */
			Ana_Set_Reg(AUDDEC_ANA_CON0, 0x0000, 0xffff);
			/* Disable Audio DAC */
			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x2AC0, 0xfeeb);
			/* Disable AUD_CLK, bit2/4/8 is for ADC, do not set */
			Ana_Set_Reg(AUDDEC_ANA_CON7, 0x0000, 0x8000);
			/* Disable NV regulator (-1.5V) */
			/*Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0001, 0xfeeb);*/
			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x02c1, 0xfeeb); /* for AUX detection */
			/* Disable cap-less LDOs (1.5V) & Disable IBIST */

			TurnOffDacPower();
		}
		EnableDcCompensation(false);
	}
}


static int Headset_Speaker_Amp_Get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	ucontrol->value.integer.value[0] =
	    mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_SPEAKER_HEADSET_R];
	return 0;
}

static int Headset_Speaker_Amp_Set(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	/* struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol); */

	pr_debug("%s() gain = %lu\n ", __func__, ucontrol->value.integer.value[0]);
	if ((ucontrol->value.integer.value[0] == true)
	    && (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_SPEAKER_HEADSET_R] ==
		false)) {
		Headset_Speaker_Amp_Change(true);
		mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_SPEAKER_HEADSET_R] =
		    ucontrol->value.integer.value[0];
	} else if ((ucontrol->value.integer.value[0] == false)
		   && (mCodec_data->
		       mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_SPEAKER_HEADSET_R] == true)) {
		mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_OUT_SPEAKER_HEADSET_R] =
		    ucontrol->value.integer.value[0];
		Headset_Speaker_Amp_Change(false);
	}
	return 0;
}

#ifdef CONFIG_MTK_SPEAKER
static const char * const speaker_amp_function[] = { "CALSSD", "CLASSAB", "RECEIVER" };

static const char * const speaker_PGA_function[] = { "MUTE", "0Db", "4Db", "5Db", "6Db", "7Db", "8Db", "9Db", "10Db",
	"11Db", "12Db", "13Db", "14Db", "15Db", "16Db", "17Db"
};
static const char * const speaker_OC_function[] = { "Off", "On" };
static const char * const speaker_CS_function[] = { "Off", "On" };
static const char * const speaker_CSPeakDetecReset_function[] = { "Off", "On" };

static int Audio_Speaker_Class_Set(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	mutex_lock(&Ana_Ctrl_Mutex);
	Speaker_mode = ucontrol->value.integer.value[0];
	mutex_unlock(&Ana_Ctrl_Mutex);
	return 0;
}

static int Audio_Speaker_Class_Get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = Speaker_mode;
	return 0;
}

static int Audio_Speaker_Pga_Gain_Set(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	Speaker_pga_gain = ucontrol->value.integer.value[0];

	pr_debug("%s Speaker_pga_gain= %d\n", __func__, Speaker_pga_gain);
	Ana_Set_Reg(SPK_ANA_CON0, Speaker_pga_gain << 11, 0x7800);
	return 0;
}

static int Audio_Speaker_OcFlag_Get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	mSpeaker_Ocflag = GetSpeakerOcFlag();
	ucontrol->value.integer.value[0] = mSpeaker_Ocflag;
	return 0;
}

static int Audio_Speaker_OcFlag_Set(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	pr_warn("%s is not support setting\n", __func__);
	return 0;
}

static int Audio_Speaker_Pga_Gain_Get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = Speaker_pga_gain;
	return 0;
}

static int Audio_Speaker_Current_Sensing_Set(struct snd_kcontrol *kcontrol,
					     struct snd_ctl_elem_value *ucontrol)
{

	if (ucontrol->value.integer.value[0])
		Ana_Set_Reg(SPK_CON12, 0x9300, 0xff00);
	else
		Ana_Set_Reg(SPK_CON12, 0x1300, 0xff00);

	return 0;
}

static int Audio_Speaker_Current_Sensing_Get(struct snd_kcontrol *kcontrol,
					     struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = (Ana_Get_Reg(SPK_CON12) >> 15) & 0x01;
	return 0;
}

static int Audio_Speaker_Current_Sensing_Peak_Detector_Set(struct snd_kcontrol *kcontrol,
							   struct snd_ctl_elem_value *ucontrol)
{

	if (ucontrol->value.integer.value[0])
		Ana_Set_Reg(SPK_CON12, 1 << 14, 1 << 14);
	else
		Ana_Set_Reg(SPK_CON12, 0, 1 << 14);

	return 0;
}

static int Audio_Speaker_Current_Sensing_Peak_Detector_Get(struct snd_kcontrol *kcontrol,
							   struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = (Ana_Get_Reg(SPK_CON12) >> 14) & 0x01;
	return 0;
}


static const struct soc_enum Audio_Speaker_Enum[] = {
	/* speaker class setting */
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(speaker_amp_function), speaker_amp_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(speaker_PGA_function), speaker_PGA_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(speaker_OC_function), speaker_OC_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(speaker_CS_function), speaker_CS_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(speaker_CSPeakDetecReset_function),
			    speaker_CSPeakDetecReset_function),
};

static const struct snd_kcontrol_new mt6331_snd_Speaker_controls[] = {
	SOC_ENUM_EXT("Audio_Speaker_class_Switch", Audio_Speaker_Enum[0], Audio_Speaker_Class_Get,
		     Audio_Speaker_Class_Set),
	SOC_ENUM_EXT("Audio_Speaker_PGA_gain", Audio_Speaker_Enum[1], Audio_Speaker_Pga_Gain_Get,
		     Audio_Speaker_Pga_Gain_Set),
	SOC_ENUM_EXT("Audio_Speaker_OC_Falg", Audio_Speaker_Enum[2], Audio_Speaker_OcFlag_Get,
		     Audio_Speaker_OcFlag_Set),
	SOC_ENUM_EXT("Audio_Speaker_CurrentSensing", Audio_Speaker_Enum[3],
		     Audio_Speaker_Current_Sensing_Get, Audio_Speaker_Current_Sensing_Set),
	SOC_ENUM_EXT("Audio_Speaker_CurrentPeakDetector", Audio_Speaker_Enum[4],
		     Audio_Speaker_Current_Sensing_Peak_Detector_Get,
		     Audio_Speaker_Current_Sensing_Peak_Detector_Set),
};

int Audio_AuxAdcData_Get_ext(void)
{
#ifdef _GIT318_PMIC_READY
	int dRetValue = PMIC_IMM_GetOneChannelValue(AUX_ICLASSAB_AP, 1, 0);

	pr_debug("%s dRetValue 0x%x\n", __func__, dRetValue);
	return dRetValue;
#else
return 0;
#endif

}


#endif

static int Audio_AuxAdcData_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{

#ifdef CONFIG_MTK_SPEAKER
	ucontrol->value.integer.value[0] = Audio_AuxAdcData_Get_ext();
	/* PMIC_IMM_GetSPK_THR_IOneChannelValue(0x001B, 1, 0); */
#else
	ucontrol->value.integer.value[0] = 0;
#endif
	pr_debug("%s dMax = 0x%lx\n", __func__, ucontrol->value.integer.value[0]);
	return 0;

}

static int Audio_AuxAdcData_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	dAuxAdcChannel = ucontrol->value.integer.value[0];
	pr_debug("%s dAuxAdcChannel = 0x%x\n", __func__, dAuxAdcChannel);
	return 0;
}


static const struct snd_kcontrol_new Audio_snd_auxadc_controls[] = {
	SOC_SINGLE_EXT("Audio AUXADC Data", SND_SOC_NOPM, 0, 0x80000, 0, Audio_AuxAdcData_Get,
		       Audio_AuxAdcData_Set),
};


static const char * const amp_function[] = { "Off", "On" };
static const char * const aud_clk_buf_function[] = { "Off", "On" };

/* static const char *DAC_SampleRate_function[] = {"8000", "11025", "16000", "24000", "32000", "44100", "48000"}; */
static const char * const DAC_DL_PGA_Headset_GAIN[] = {
	"8Db", "7Db", "6Db", "5Db", "4Db", "3Db", "2Db", "1Db", "0Db", "-1Db", "-2Db", "-3Db",
	"-4Db", "-5Db", "-6Db", "-7Db", "-8Db", "-9Db", "-10Db", "-40Db"
};

static const char * const DAC_DL_PGA_Handset_GAIN[] = {
	"8Db", "7Db", "6Db", "5Db", "4Db", "3Db", "2Db", "1Db", "0Db", "-1Db", "-2Db", "-3Db",
	"-4Db", "-5Db", "-6Db", "-7Db", "-8Db", "-9Db", "-10Db", "-40Db"
};

static const char * const DAC_DL_PGA_Speaker_GAIN[] = {
	"8Db", "7Db", "6Db", "5Db", "4Db", "3Db", "2Db", "1Db", "0Db", "-1Db", "-2Db", "-3Db",
	"-4Db", "-5Db", "-6Db", "-7Db", "-8Db", "-9Db", "-10Db", "-40Db"
};

/* static const char *Voice_Mux_function[] = {"Voice", "Speaker"}; */

static int Lineout_PGAL_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("Speaker_PGA_Get = %d\n", mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_SPKL]);
	ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_SPKL];
	return 0;
}

static int Lineout_PGAL_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int index = 0;

	/*pr_debug("%s(), index = %d\n", __func__, ucontrol->value.enumerated.item[0]);*/

	if (ucontrol->value.enumerated.item[0] >= ARRAY_SIZE(DAC_DL_PGA_Speaker_GAIN)) {
		pr_warn("return -EINVAL\n");
		return -EINVAL;
	}

	index = ucontrol->value.integer.value[0];

	if (ucontrol->value.enumerated.item[0] == (ARRAY_SIZE(DAC_DL_PGA_Speaker_GAIN) - 1))
		index = 0x1f;

	Ana_Set_Reg(ZCD_CON1, index, 0x001f);

	mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_SPKL] = ucontrol->value.integer.value[0];
	return 0;
}

static int Lineout_PGAR_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s  = %d\n", __func__, mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_SPKR]);
	ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_SPKR];
	return 0;
}

static int Lineout_PGAR_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol); */
	int index = 0;

	/*pr_debug("%s(), index = %d\n", __func__, ucontrol->value.enumerated.item[0]);*/

	if (ucontrol->value.enumerated.item[0] >= ARRAY_SIZE(DAC_DL_PGA_Speaker_GAIN)) {
		pr_warn("return -EINVAL\n");
		return -EINVAL;
	}

	index = ucontrol->value.integer.value[0];

	if (ucontrol->value.enumerated.item[0] == (ARRAY_SIZE(DAC_DL_PGA_Speaker_GAIN) - 1))
		index = 0x1f;

	Ana_Set_Reg(ZCD_CON1, index << 7, 0x0f10);
	mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_SPKR] = ucontrol->value.integer.value[0];
	return 0;
}

static int Handset_PGA_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("Handset_PGA_Get = %d\n",
	       mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HSOUTL]);
	ucontrol->value.integer.value[0] =
	    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HSOUTL];
	return 0;
}

static int Handset_PGA_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol); */
	int index = 0;

	pr_debug("%s(), index = %d\n", __func__, ucontrol->value.enumerated.item[0]);

	if (ucontrol->value.enumerated.item[0] >= ARRAY_SIZE(DAC_DL_PGA_Handset_GAIN)) {
		pr_warn("return -EINVAL\n");
		return -EINVAL;
	}
	index = ucontrol->value.integer.value[0];

	if (ucontrol->value.enumerated.item[0] == (ARRAY_SIZE(DAC_DL_PGA_Handset_GAIN) - 1))
		index = 0x1f;

	Ana_Set_Reg(ZCD_CON3, index, 0x001f);

	mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HSOUTL] =
	    ucontrol->value.integer.value[0];
	return 0;
}

static int Headset_PGAL_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("Headset_PGAL_Get = %d\n",
		 mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTL]);
	ucontrol->value.integer.value[0] =
	    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTL];
	return 0;
}

static int Headset_PGAL_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol); */
	int index = 0;

	/* pr_debug("%s(), index = %d arraysize = %d\n", __func__,
	ucontrol->value.enumerated.item[0], ARRAY_SIZE(DAC_DL_PGA_Headset_GAIN)); //mark for 64bit build fail */

	if (ucontrol->value.enumerated.item[0] >= ARRAY_SIZE(DAC_DL_PGA_Headset_GAIN)) {
		pr_warn("return -EINVAL\n");
		return -EINVAL;
	}

	index = ucontrol->value.integer.value[0];

	if (ucontrol->value.enumerated.item[0] == (ARRAY_SIZE(DAC_DL_PGA_Headset_GAIN) - 1))
		index = 0x1f;

	Ana_Set_Reg(ZCD_CON2, index, 0x001f);
	mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTL] =
	    ucontrol->value.integer.value[0];
	return 0;
}

static int Headset_PGAR_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("Headset_PGAR_Get = %d\n",
	       mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTR]);
	ucontrol->value.integer.value[0] =
	    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTR];
	return 0;
}


static int Headset_PGAR_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol); */
	int index = 0;

	/*pr_debug("%s(), index = %d\n", __func__, ucontrol->value.enumerated.item[0]);*/

	if (ucontrol->value.enumerated.item[0] >= ARRAY_SIZE(DAC_DL_PGA_Headset_GAIN)) {
		pr_warn("return -EINVAL\n");
		return -EINVAL;
	}
	index = ucontrol->value.integer.value[0];

	if (ucontrol->value.enumerated.item[0] == (ARRAY_SIZE(DAC_DL_PGA_Headset_GAIN) - 1))
		index = 0x1f;

	Ana_Set_Reg(ZCD_CON2, index << 7, 0x0f80);
	mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTR] =
	    ucontrol->value.integer.value[0];
	return 0;
}

static int Aud_Clk_Buf_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/*pr_debug("\%s n", __func__);*/
	ucontrol->value.integer.value[0] = audck_buf_Count;
	return 0;
}

static int Aud_Clk_Buf_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/*pr_debug("%s(), value = %d\n", __func__, ucontrol->value.enumerated.item[0]);*/

	if (ucontrol->value.integer.value[0])
		audckbufEnable(true);
	else
		audckbufEnable(false);

	return 0;
}


static const struct soc_enum Audio_DL_Enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(amp_function), amp_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(amp_function), amp_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(amp_function), amp_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(amp_function), amp_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(amp_function), amp_function),
	/* here comes pga gain setting */
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(DAC_DL_PGA_Headset_GAIN), DAC_DL_PGA_Headset_GAIN),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(DAC_DL_PGA_Headset_GAIN), DAC_DL_PGA_Headset_GAIN),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(DAC_DL_PGA_Handset_GAIN), DAC_DL_PGA_Handset_GAIN),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(DAC_DL_PGA_Speaker_GAIN), DAC_DL_PGA_Speaker_GAIN),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(DAC_DL_PGA_Speaker_GAIN), DAC_DL_PGA_Speaker_GAIN),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(aud_clk_buf_function), aud_clk_buf_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(amp_function), amp_function),
};

static const struct snd_kcontrol_new mt6331_snd_controls[] = {
	SOC_ENUM_EXT("Audio_Amp_R_Switch", Audio_DL_Enum[0], Audio_AmpR_Get, Audio_AmpR_Set),
	SOC_ENUM_EXT("Audio_Amp_L_Switch", Audio_DL_Enum[1], Audio_AmpL_Get, Audio_AmpL_Set),
	SOC_ENUM_EXT("Voice_Amp_Switch", Audio_DL_Enum[2], Voice_Amp_Get, Voice_Amp_Set),
	SOC_ENUM_EXT("Speaker_Amp_Switch", Audio_DL_Enum[3], Speaker_Amp_Get, Speaker_Amp_Set),
	SOC_ENUM_EXT("Headset_Speaker_Amp_Switch", Audio_DL_Enum[4], Headset_Speaker_Amp_Get,
		     Headset_Speaker_Amp_Set),
	SOC_ENUM_EXT("Headset_PGAL_GAIN", Audio_DL_Enum[5], Headset_PGAL_Get, Headset_PGAL_Set),
	SOC_ENUM_EXT("Headset_PGAR_GAIN", Audio_DL_Enum[6], Headset_PGAR_Get, Headset_PGAR_Set),
	SOC_ENUM_EXT("Handset_PGA_GAIN", Audio_DL_Enum[7], Handset_PGA_Get, Handset_PGA_Set),
	SOC_ENUM_EXT("Lineout_PGAR_GAIN", Audio_DL_Enum[8], Lineout_PGAR_Get, Lineout_PGAR_Set),
	SOC_ENUM_EXT("Lineout_PGAL_GAIN", Audio_DL_Enum[9], Lineout_PGAL_Get, Lineout_PGAL_Set),
	SOC_ENUM_EXT("AUD_CLK_BUF_Switch", Audio_DL_Enum[10], Aud_Clk_Buf_Get, Aud_Clk_Buf_Set),
	SOC_ENUM_EXT("Ext_Speaker_Amp_Switch", Audio_DL_Enum[11], Ext_Speaker_Amp_Get,
		     Ext_Speaker_Amp_Set),
	SOC_ENUM_EXT("Receiver_Speaker_Switch", Audio_DL_Enum[11], Receiver_Speaker_Switch_Get,
		     Receiver_Speaker_Switch_Set),
};

static const struct snd_kcontrol_new mt6331_Voice_Switch[] = {
	/* SOC_DAPM_ENUM_EXT("Voice Mux", Audio_DL_Enum[10], Voice_Mux_Get, Voice_Mux_Set), */
};

void SetMicPGAGain(void)
{
	int index = 0;

	index = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP1];
	pr_debug("%s  AUDIO_ANALOG_VOLUME_MICAMP1 index =%d\n", __func__, index);
	/* Ana_Set_Reg(AUDENC_ANA_CON15, index , 0x0007); */
	Ana_Set_Reg(AUDENC_ANA_CON10, index, 0x0007);

	index = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP2];
	/* Ana_Set_Reg(AUDENC_ANA_CON15, index << 4, 0x0070); */
	Ana_Set_Reg(AUDENC_ANA_CON10, index << 4, 0x0070);

}

static bool GetAdcStatus(void)
{
	int i = 0;

	for (i = AUDIO_ANALOG_DEVICE_IN_ADC1; i < AUDIO_ANALOG_DEVICE_MAX; i++) {
		if ((mCodec_data->mAudio_Ana_DevicePower[i] == true)
		    && (i != AUDIO_ANALOG_DEVICE_RECEIVER_SPEAKER_SWITCH)) {
			return true;
		}
	}
	return false;
}

static bool GetDacStatus(void)
{
	int i = 0;

	for (i = AUDIO_ANALOG_DEVICE_OUT_EARPIECER; i < AUDIO_ANALOG_DEVICE_2IN1_SPK; i++) {
		if (mCodec_data->mAudio_Ana_DevicePower[i] == true)
			return true;
	}
	return false;
}


static bool TurnOnADcPowerACC(int ADCType, bool enable)
{
	bool refmic_using_ADC_L;

	refmic_using_ADC_L = false;

	pr_debug("%s ADCType = %d enable = %d, refmic_using_ADC_L=%d\n", __func__, ADCType,
		 enable, refmic_using_ADC_L);

	if (enable) {
		/* uint32 ULIndex = GetULFrequency(mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC]); */
		uint32 SampleRate_VUL1 = mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC];
		/* uint32 SampleRate_VUL2 = mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC_2]; */
		if (GetMicbias == 0) {
			MicbiasRef = Ana_Get_Reg(AUDENC_ANA_CON9) & 0x0700;
			/* save current micbias ref set by accdet */
			pr_debug("MicbiasRef=0x%x\n", MicbiasRef);
			GetMicbias = 1;
		}
		if (GetAdcStatus() == false) {
			audckbufEnable(true);
			/* Ana_Set_Reg(LDO_VCON1, 0x0301, 0xffff);
			//VA28 remote sense //removed in MT6328 */

			Ana_Set_Reg(LDO_CON2, 0x8102, 0xffff);
			/* LDO enable control by RG_VAUD28_EN, Enable AVDD28_LDO (Default on) */

			NvregEnable(true);
			/* ClsqAuxEnable(true); */
			ClsqEnable(true);

			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0004, 0x0004);
			/* Enable audio ADC CLKGEN */

			Ana_Set_Reg(AUDENC_ANA_CON3, 0x0000, 0xffff);
			/* ADC CLK from CLKGEN (13MHz) */
			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0104, 0x0104);
			/* Enable  LCLDO_ENC 1P8V */

			Ana_Set_Reg(AUDDEC_ANA_CON7, 0x0006, 0x0006);
			/* LCLDO_ENC remote sense */
			/* Ana_Set_Reg(AUDENC_ANA_CON6, 0x1515, 0xffff); //default value */
			Ana_Set_Reg(AUDENC_ANA_CON6, 0x0555, 0xffff);
			/* default value MT6328 */

			Ana_Set_Reg(AUDENC_ANA_CON4, 0x0800, 0xffff);
			/* default value */
		}

		if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC1) {	/* main and headset mic */
			pr_debug("%s  AUDIO_ANALOG_DEVICE_IN_ADC1 mux =%d\n", __func__,
				 mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1]);
			if (mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1] == 0) {
				/* "ADC1", main_mic */
				SetDCcoupleNP(AUDIO_MIC_BIAS0, mAudio_Analog_Mic1_mode);
				/* micbias0 DCCopuleNP */
				/* Ana_Set_Reg(AUDENC_ANA_CON9, 0x0201, 0xff09);
				//Enable MICBIAS0, MISBIAS0 = 1P9V */
				Ana_Set_Reg(AUDENC_ANA_CON9, 0x0711, 0xff19);
				/* Enable MICBIAS0, MISBIAS0 = 1P9V,
				also enable MICBIAS1 at the same time to avoid noise */
			} else if (mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1] == 1) {
				/* "ADC2", headset mic */
				SetDCcoupleNP(AUDIO_MIC_BIAS1, mAudio_Analog_Mic1_mode);
				/* micbias1 DCCopuleNP */
				Ana_Set_Reg(AUDENC_ANA_CON9, 0x0710, 0xff90);
				/* Enable MICBIAS1, MISBIAS1 = 2P5V ?// or 2P7V George? */
			}
			/* Ana_Set_Reg(AUDENC_ANA_CON15, 0x0003, 0x000f); //Audio L PGA 18 dB gain(SMT) */
			Ana_Set_Reg(AUDENC_ANA_CON10, 0x0003, 0x000f);	/* Audio L PGA 18 dB gain(SMT) MT6328 */

		} else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC2) {	/* ref mic */
			pr_debug("%s  AUDIO_ANALOG_DEVICE_IN_ADC2 refmic_using_ADC_L =%d\n", __func__,
				 refmic_using_ADC_L);
			SetDCcoupleNP(AUDIO_MIC_BIAS0, mAudio_Analog_Mic2_mode);
			/* micbias0 DCCopuleNP */

			if (mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1] == 1) {
				/*Enable MICBIAS0, MISBIAS0 = 2P7V */
				Ana_Set_Reg(AUDENC_ANA_CON9, 0x0710, 0xff90);

			} else {
				/* "ADC2", headset mic */
				SetDCcoupleNP(AUDIO_MIC_BIAS1, mAudio_Analog_Mic1_mode);
				/* micbias1 DCCopuleNP */
				Ana_Set_Reg(AUDENC_ANA_CON9, 0x0711, 0xff19);
				/* Enable MICBIAS1, MISBIAS1 = 1P9V// or 2P7V George? */
				/* Enable MICBIAS0, MISBIAS0 = 1P9V, also enable MICBIAS1 to avoid noise */
			}


			if (refmic_using_ADC_L == false) {
				/* Ana_Set_Reg(AUDENC_ANA_CON15, 0x0030, 0x00f0); //Audio R PGA 18 dB gain(SMT) */
				Ana_Set_Reg(AUDENC_ANA_CON10, 0x0033, 0x00ff);
				/* Audio R PGA 18 dB gain(SMT) MT6328 */
			} else {
				/* Ana_Set_Reg(AUDENC_ANA_CON15, 0x0003, 0x000f);
				//Audio L PGA 18 dB gain(SMT) */
				Ana_Set_Reg(AUDENC_ANA_CON10, 0x0003, 0x000f);
				/* Audio L PGA 18 dB gain(SMT) MT6328 */
			}
		}

		if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC1) {	/* main and headset mic */
			Ana_Set_Reg(AUDENC_ANA_CON3, 0x0800, 0xf900);	/* PGA stb enhance */

			if (mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1] == 0) {
				/* "ADC1", main_mic */
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0041, 0x00C1);
				/* Audio L preamplifier input sel : AIN0. Enable audio L PGA */
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0541, 0xffff);
				/* Audio L ADC input sel : L PGA. Enable audio L ADC */
			} else if (mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1] == 1) {
				/* "ADC2", headset mic */
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0500, 0xffff);
				/* Audio L ADC input sel : L PGA. Enable audio L ADC */
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0581, 0xffff);
				/* Audio L preamplifier input sel : AIN1. Enable audio L PGA */

			}
		} else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC2) {
			/* ref mic */
			Ana_Set_Reg(AUDENC_ANA_CON3, 0x0800, 0xf900);
			/* PGA stb enhance */
			if (refmic_using_ADC_L == false) {
				Ana_Set_Reg(AUDENC_ANA_CON1, 0x00C1, 0x00C1);
				/* Audio R preamplifier input sel : AIN2. Enable audio R PGA */
				Ana_Set_Reg(AUDENC_ANA_CON1, 0x05C1, 0xffff);
				/* Audio R ADC input sel : R PGA. Enable audio R ADC */
			} else {
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x00c1, 0x00C1);
				/* Audio L preamplifier input sel : AIN2. Enable audio L PGA */
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x05c1, 0xffff);
				/* Audio L ADC input sel : L PGA. Enable audio L ADC */
			}
		}

		SetMicPGAGain();

		if (GetAdcStatus() == false) {
			/* here to set digital part */
			Topck_Enable(true);
			/* AdcClockEnable(true); */
			if (GetDacStatus() == false) {
				Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x005a, 0xffff);
				/* power on clock */
			} else {
				Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x0000, 0xffff);
				/* power on clock */
			}
			Ana_Set_Reg(PMIC_AFE_TOP_CON0, 0x0000, 0xffff);
			/* configure ADC setting */
			/* Ana_Set_Reg(AFUNC_AUD_CON2, 0x0006, 0xffff);
			//sdm audio fifo clock power on */
			/* Ana_Set_Reg(AFUNC_AUD_CON0, 0xc3a1, 0xffff);
			//scrambler clock on enable */
			/* Ana_Set_Reg(AFUNC_AUD_CON2, 0x0003, 0xffff);
			//sdm power on */
			/* Ana_Set_Reg(AFUNC_AUD_CON2, 0x000b, 0xffff);
			//sdm fifo enable */
			/* Ana_Set_Reg(AFE_DL_SDM_CON1, 0x001e, 0xffff);
			//set attenuation gain */
			Ana_Set_Reg(AFE_UL_DL_CON0, 0x0001, 0xffff);
			/* [0] afe enable */

			Ana_Set_Reg(AFE_UL_SRC0_CON0_H,
			(ULSampleRateTransform(SampleRate_VUL1) << 3 | ULSampleRateTransform(SampleRate_VUL1) << 1),
			0x001f);
			/* UL sample rate and mode configure */
			Ana_Set_Reg(AFE_UL_SRC0_CON0_L, 0x0001, 0xffff);
			/* UL turn on */
		}
	} else {
		if (GetMicbias == 0) {
			MicbiasRef = Ana_Get_Reg(AUDENC_ANA_CON9) & 0x0700;
			/* save current micbias ref set by accdet */
			pr_debug("MicbiasRef=0x%x\n", MicbiasRef);
			GetMicbias = 1;
		}
		if (GetAdcStatus() == false) {
			Ana_Set_Reg(AFE_UL_SRC0_CON0_L, 0x0000, 0xffff);
			/* UL turn off */
			Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x0020, 0x0020);
			/* up-link power down */
		}
		if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC1) {
			/* main and headset mic */
			if (mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1] == 0) {
				/* "ADC1", main_mic */
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0041, 0xffff);
				/* Audio L ADC input sel : off, disable audio L ADC */
				Ana_Set_Reg(AUDENC_ANA_CON3, 0x0000, 0xffff);
				/* PGA stb enhance off */
				/* Ana_Set_Reg(AUDENC_ANA_CON15, 0x0000, 0x000f); //Audio L PGA 0 dB gain */
				Ana_Set_Reg(AUDENC_ANA_CON10, 0x0000, 0x000f);
				/* MT6328 Audio L PGA 0 dB gain */

				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0000, 0xffff);
				/* Audio L preamplifier input sel : off, disable audio L PGA */
			} else if (mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1] == 1) {
				/* "ADC2", headset mic */
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0500, 0xffff);
				/* Audio L preamplifier input sel : off, disable audio L PGA */
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0000, 0xffff);
				/* Audio L ADC input sel : off, disable audio L ADC */
				Ana_Set_Reg(AUDENC_ANA_CON3, 0x0000, 0xffff);
				/* PGA stb enhance off */
				/* Ana_Set_Reg(AUDENC_ANA_CON15, 0x0000, 0x000f);
				//Audio L PGA 0 dB gain */
				Ana_Set_Reg(AUDENC_ANA_CON10, 0x0000, 0x000f);
				/* MT6328 Audio L PGA 0 dB gain */
			}

			Ana_Set_Reg(AUDENC_ANA_CON4, 0x0000, 0xffff);	/*  */
			Ana_Set_Reg(AUDENC_ANA_CON6, 0x2020, 0xffff);	/*  */

			if (mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1] == 0) {
				/* "ADC1", main_mic */
				/* Ana_Set_Reg(AUDENC_ANA_CON9, (MicbiasRef|0x0000), 0xff09);
				//disable MICBIAS0, restore to micbias set by accdet */
				Ana_Set_Reg(AUDENC_ANA_CON9, (MicbiasRef | 0x0000), 0xff19);
				/* disable MICBIAS0 and MICBIAS1, restore to micbias set by accdet */
			} else if (mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1] == 1) {
				/* "ADC2", headset mic */
				Ana_Set_Reg(AUDENC_ANA_CON9, (MicbiasRef | 0x0000), 0xff90);
				/* disable MICBIAS1, restore to micbias set by accdet */
			}
		} else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC2) {	/* ref mic */
			if (refmic_using_ADC_L == false) {
				Ana_Set_Reg(AUDENC_ANA_CON1, 0x00C1, 0xffff);
				/* Audio R ADC input sel : off, disable audio R ADC */
				Ana_Set_Reg(AUDENC_ANA_CON3, 0x0000, 0xffff);
				/* PGA stb enhance off */
				Ana_Set_Reg(AUDENC_ANA_CON10, 0x0000, 0x00ff);
				/* Audio R PGA 0 dB gain //MT6328 */
				Ana_Set_Reg(AUDENC_ANA_CON1, 0x0000, 0xffff);
				/* Audio R preamplifier input sel : off, disable audio R PGA */
			} else {
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x00c1, 0xffff);
				/* Audio L ADC input sel : off, disable audio L ADC */
				Ana_Set_Reg(AUDENC_ANA_CON3, 0x0000, 0xffff);
				/* PGA stb enhance off */
				Ana_Set_Reg(AUDENC_ANA_CON10, 0x0000, 0x000f);
				/* Audio L PGA 0 dB gain */
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0000, 0xffff);
				/* Audio L preamplifier input sel : off, disable audio L PGA */
			}
			Ana_Set_Reg(AUDENC_ANA_CON4, 0x0000, 0xffff);	/*  */
			Ana_Set_Reg(AUDENC_ANA_CON6, 0x2020, 0xffff);	/*  */

			/* Ana_Set_Reg(AUDENC_ANA_CON9, (MicbiasRef|0x0000), 0xff09);
			//disable MICBIAS0, restore to micbias set by accdet */
			Ana_Set_Reg(AUDENC_ANA_CON9, (MicbiasRef | 0x0000), 0xff19);
			/* disable MICBIAS0 and MICBIAS1, restore to micbias set by accdet */
		}
		if (GetAdcStatus() == false) {
			Ana_Set_Reg(AUDDEC_ANA_CON7, 0x0000, 0x0006);
			/* LCLDO_ENC remote sense off */
			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0004, 0x0104);
			/* disable LCLDO_ENC 1P8V */

			Ana_Set_Reg(AUDENC_ANA_CON3, 0x0000, 0xffff);
			/* disable ADC CLK from CLKGEN (13MHz) */
			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0000, 0x0104);
			/* disable audio ADC CLKGEN */

			if (GetDLStatus() == false) {
				Ana_Set_Reg(AFE_UL_DL_CON0, 0x0000, 0xffff);
				/* afe disable */
				Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x0084, 0x0084);
				/* afe power down and total audio clk disable */
			}

			/* AdcClockEnable(false); */
			Topck_Enable(false);
			/* ClsqAuxEnable(false); */
			ClsqEnable(false);
			NvregEnable(false);
			audckbufEnable(false);
		}
		GetMicbias = 0;
	}
	return true;
}

static bool TurnOnADcPowerDmic(int ADCType, bool enable)
{
	pr_debug("%s ADCType = %d enable = %d\n", __func__, ADCType, enable);
	if (enable) {
		uint32 ULIndex = GetULFrequency(mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC]);
		uint32 SampleRate_VUL1 = mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC];
		/* uint32 SampleRate_VUL2 = mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC_2]; */
		if (GetMicbias == 0) {
			MicbiasRef = Ana_Get_Reg(AUDENC_ANA_CON9) & 0x0700;
			/* save current micbias ref set by accdet */
			pr_debug("MicbiasRef=0x%x\n", MicbiasRef);
			GetMicbias = 1;
		}
		if (GetAdcStatus() == false) {
			audckbufEnable(true);
			Ana_Set_Reg(LDO_CON2, 0x8102, 0xffff);
			/* LDO enable control by RG_VAUD28_EN, Enable AVDD28_LDO (Default on) */

			NvregEnable(true);
			/* ClsqAuxEnable(true); */
			ClsqEnable(true);

			SetDCcoupleNP(AUDIO_MIC_BIAS0, mAudio_Analog_Mic1_mode);
			/* micbias0 DCCopuleNP */
			/* Ana_Set_Reg(AUDENC_ANA_CON9, 0x0201, 0xff09); //Enable MICBIAS0, MISBIAS0 = 1P9V */
			Ana_Set_Reg(AUDENC_ANA_CON9, 0x0211, 0xff19);
			/* Enable MICBIAS0 and MICBIAS1, MISBIAS0 = 1P9V */
			Ana_Set_Reg(AUDENC_ANA_CON8, 0x0005, 0xffff);
			/* DMIC enable */

			/* here to set digital part */
			Topck_Enable(true);
			/* AdcClockEnable(true); */
			if (GetDacStatus() == false) {
				Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x005a, 0xffff);
				/* power on clock  // ? MT6328 George check */
				/* George  ]0x00O}A]0x40ODAC ,   SW]0x5aODADCclockAhٹq */
			} else {
				Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x0000, 0xffff);
				/* power on clock */
			}
			Ana_Set_Reg(PMIC_AFE_TOP_CON0, (ULIndex << 7) | (ULIndex << 6), 0xffff);
			/* dmic sample rate, ch1 and ch2 set to 3.25MHz 48k // ? MT6328 George check */
			Ana_Set_Reg(AFE_UL_DL_CON0, 0x0001, 0xffff);
			/* [0] afe enable */

			Ana_Set_Reg(AFE_UL_SRC0_CON0_H,
			(ULSampleRateTransform(SampleRate_VUL1) << 3 | ULSampleRateTransform(SampleRate_VUL1) << 1),
			0x001f);	/* UL sample rate and mode configure */
			Ana_Set_Reg(AFE_UL_SRC0_CON0_H, 0x00e0, 0xffe0);
			/* 2-wire dmic mode, ch1 and ch2 digital mic ON */
			Ana_Set_Reg(AFE_UL_SRC0_CON0_L, 0x0003, 0xffff);
			/* digmic input mode 3.25MHz, select SDM 3-level mode, UL turn on */
		}
	} else {
		if (GetAdcStatus() == false) {
			Ana_Set_Reg(AFE_UL_SRC0_CON0_L, 0x0000, 0xffff);
			/* UL turn off */
			Ana_Set_Reg(AFE_UL_SRC0_CON0_H, 0x0000, 0xffe0);
			/* ch1 and ch2 digital mic OFF */
			Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x0020, 0x0020);
			/* up-link power down // ? MT6328 George check */

			Ana_Set_Reg(AUDENC_ANA_CON8, 0x0000, 0xffff);
			/* DMIC disable */

			/* Ana_Set_Reg(AUDENC_ANA_CON9, (MicbiasRef|0x0000), 0xff09);
			//MICBIAS0(1.7v), powen down, restore to micbias set by accdet */
			Ana_Set_Reg(AUDENC_ANA_CON9, (MicbiasRef | 0x0000), 0xff19);
			/* MICBIAS0(1.7v), powen down, restore to micbias set by accdet, MICBIAS1 off */
			if (GetDLStatus() == false) {
				Ana_Set_Reg(AFE_UL_DL_CON0, 0x0000, 0xffff);
				/* afe disable */
				Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x0084, 0x0084);
				/* afe power down and total audio clk disable */
			}

			/* AdcClockEnable(false); */
			Topck_Enable(false);
			/* ClsqAuxEnable(false); */
			ClsqEnable(false);
			NvregEnable(false);
			audckbufEnable(false);
		}
		GetMicbias = 0;
	}
	return true;
}

static bool TurnOnADcPowerDCC(int ADCType, bool enable, int ECMmode)
{
	pr_debug("%s ADCType = %d enable = %d\n", __func__, ADCType, enable);
	if (enable) {
		/* uint32 ULIndex = GetULFrequency(mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC]); */
		uint32 SampleRate_VUL1 = mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC];
		/* uint32 SampleRate_VUL2 = mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC_2]; */
		if (GetMicbias == 0) {
			MicbiasRef = Ana_Get_Reg(AUDENC_ANA_CON9) & 0x0700;
			/* save current micbias ref set by accdet */
			pr_debug("MicbiasRef=0x%x\n", MicbiasRef);
			GetMicbias = 1;
		}
		if (GetAdcStatus() == false) {
			audckbufEnable(true);
			/* Ana_Set_Reg(LDO_VCON1, 0x0301, 0xffff); //VA28 remote sense // removed in MT6328 */
			Ana_Set_Reg(LDO_CON2, 0x8102, 0xffff);
			/* LDO enable control by RG_VAUD28_EN, Enable AVDD28_LDO (Default on) */

			NvregEnable(true);
			/* ClsqAuxEnable(true); */
			ClsqEnable(true);

			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0004, 0x0004);
			/* Enable audio ADC CLKGEN */
			Ana_Set_Reg(AUDENC_ANA_CON3, 0x0000, 0xffff);
			/* ADC CLK from CLKGEN (13MHz) */
			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0104, 0x0104);
			/* Enable LCLDO_ENC 1P8V */

			Ana_Set_Reg(AUDDEC_ANA_CON7, 0x0006, 0x0006);
			/* LCLDO_ENC remote sense */

			/* DCC 50k CLK(from 26M */
			Ana_Set_Reg(TOP_CLKSQ_SET, 0x0003, 0x0003);	/*  */
			/* Ana_Set_Reg(TOP_CKPDN_CON0, 0x0000, 0x2000);
			//bit[13] AUD_CK power down=0 */
			Ana_Set_Reg(TOP_CKPDN_CON0, 0x0000, 0x6000);
			/* bit[14]+bit[13] AUD_CK power on=0 MT6328 */

			Ana_Set_Reg(AFE_DCCLK_CFG0, 0x2062, 0xffff);
			/* dcclk_div=11'b00100000011, dcclk_ref_ck_sel=2'b00 */
			Ana_Set_Reg(AFE_DCCLK_CFG0, 0x2060, 0xffff);
			/* dcclk_pdn=1'b0 */
			Ana_Set_Reg(AFE_DCCLK_CFG0, 0x2061, 0xffff);
			/* dcclk_gen_on=1'b1 */

			Ana_Set_Reg(AFE_DCCLK_CFG1, 0x0100, 0x0100);
			/* dcclk_resync_bypass=1'b1 MT6328 requested by chen-chien(2014/10/30) */
			/*  */
			/*  */
			/* Ana_Set_Reg(AUDENC_ANA_CON6, 0x1515, 0xffff); //default value */
			Ana_Set_Reg(AUDENC_ANA_CON6, 0x0555, 0xffff);
			/* default value  MT6328 */

			Ana_Set_Reg(AUDENC_ANA_CON4, 0x0800, 0xffff);
			/* default value */
		}
		if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC1) {	/* main and headset mic */
			pr_debug("%s  AUDIO_MICSOURCE_MUX_IN_1 mux =%d\n", __func__,
				 mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1]);
			if (mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1] == 0) {	/* "ADC1", main_mic */
				SetDCcoupleNP(AUDIO_MIC_BIAS0, mAudio_Analog_Mic1_mode);
				/* micbias0 DCCopuleNP */
				/* Ana_Set_Reg(AUDENC_ANA_CON9, 0x0201, 0xff09);
				//Enable MICBIAS0, MISBIAS0 = 1P9V */
				Ana_Set_Reg(AUDENC_ANA_CON9, 0x0701, 0xff09);
				/* Enable MICBIAS0, MISBIAS0 = 2P5V */
				if (ECMmode == 1) {	/* differential */
					/* Ana_Set_Reg(AUDENC_ANA_CON9, 0x0217, 0xff1f);
					//Enable MICBIAS0 and MICBIAS1, MISBIAS0 = 1P9V, [6752] ALPS01824949 */
					Ana_Set_Reg(AUDENC_ANA_CON9, 0x0717, 0xff1f);
					/* Enable MICBIAS0 and MICBIAS1, MISBIAS0 = 2P5V, [6752] ALPS01824949 */

				} else if (ECMmode == 2) {	/* single end */
					/* Ana_Set_Reg(AUDENC_ANA_CON9, 0x0213, 0xff1f);
					//Enable MICBIAS0 and MICBIAS1, MISBIAS0 = 1P9V, [6752] ALPS01824949 */
					Ana_Set_Reg(AUDENC_ANA_CON9, 0x0713, 0xff1f);
					/* Enable MICBIAS0 and MICBIAS1, MISBIAS0 = 2P5V */

				} else {	/* MEMS */

					/* Ana_Set_Reg(AUDENC_ANA_CON9, 0x0211, 0xff1f);
					//Enable MICBIAS0 and MICBIAS1, MISBIAS0 = 1P9V , [6752] ALPS01824949 */
					Ana_Set_Reg(AUDENC_ANA_CON9, 0x0711, 0xff1f);
					/* Enable MICBIAS0 and MICBIAS1, MISBIAS0 = 2P5V */
				}
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0004, 0xffff);
				/* Audio L preamplifier DCC precharge */
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0045, 0xffff);
				/* Audio L preamplifier input sel : AIN0. Enable audio L PGA */
				/* Ana_Set_Reg(AUDENC_ANA_CON15, 0x0002, 0x000f);
				//Audio L PGA 12 dB gain */
				Ana_Set_Reg(AUDENC_ANA_CON10, 0x0003, 0x000f);
				/* Audio L PGA 18 dB gain MT6328 */

				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0047, 0xffff);
				/* Audio L preamplifier DCCEN */
			} else if (mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1] == 1) {
				/* "ADC2", headset mic */
				SetDCcoupleNP(AUDIO_MIC_BIAS1, mAudio_Analog_Mic1_mode);
				/* micbias1 DCCopuleNP */
				if (ECMmode == 1) {	/* differential */
					Ana_Set_Reg(AUDENC_ANA_CON9, 0x0770, 0xfff0);
					/* Enable MICBIAS1, SwithP/N MISBIAS1 = 2P5V */
				} else if (ECMmode == 2) {	/* single end */
					Ana_Set_Reg(AUDENC_ANA_CON9, 0x0730, 0xfff0);
					/* Enable MICBIAS1 and MICBIAS1, MISBIAS0 = 1P9V, [6752] ALPS01824949 */
				} else {	/* MEMS */

					/* Ana_Set_Reg(AUDENC_ANA_CON9, 0x0710, 0xfff0); //MICBIAS1 DCC  on //DCC */
					Ana_Set_Reg(AUDENC_ANA_CON9, 0x0710, 0xfff0);
					/* MICBIAS1 DCC  on //DCC, bit 5 to 1 to match ACCDET */

				}
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0004, 0xffff);
				/* Audio L preamplifier DCC precharge */
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0085, 0xffff);
				/* Audio L preamplifier input sel : AIN1. Enable audio L PGA */
				/* Ana_Set_Reg(AUDENC_ANA_CON15, 0x0002, 0x000f);
				//Audio L PGA 12 dB gain */
				Ana_Set_Reg(AUDENC_ANA_CON10, 0x0003, 0x000f);
				/* Audio L PGA 18 dB gain MT6328 */
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0087, 0xffff);
				/* Audio L preamplifier DCCEN */
			}
		} else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC2) {	/* ref mic */
			SetDCcoupleNP(AUDIO_MIC_BIAS0, mAudio_Analog_Mic2_mode);
			/* micbias0 DCCopuleNP */
			if (ECMmode == 1) {	/* differenital */
				/* Ana_Set_Reg(AUDENC_ANA_CON9, 0x0217, 0xff1f);
				//Enable MICBIAS0 and MICBIAS1, MISBIAS0 = 1P9V , [6752] ALPS01824949 */
				Ana_Set_Reg(AUDENC_ANA_CON9, 0x0717, 0xff1f);
				/* Enable MICBIAS0 and MICBIAS1, MISBIAS0 = 2P5V */

			} else if (ECMmode == 2) {	/* single end */
				Ana_Set_Reg(AUDENC_ANA_CON9, 0x0713, 0xff1f);
				/* Enable MICBIAS0 and MICBIAS1, MISBIAS0 = 2P5V */
			} else {	/* MEMS */

				Ana_Set_Reg(AUDENC_ANA_CON9, 0x0711, 0xff1f);
				/* Enable MICBIAS0 and MICBIAS1, MISBIAS0 = 2P5V */
			}
			/* Ana_Set_Reg(AUDENC_ANA_CON9, 0x0207, 0xff0f); //MICBIAS0 DCC SwithP/N on //DCC */
			Ana_Set_Reg(AUDENC_ANA_CON1, 0x0004, 0xffff);
			/* Audio R preamplifier DCC precharge */
			Ana_Set_Reg(AUDENC_ANA_CON1, 0x00C5, 0xffff);
			/* Audio R preamplifier input sel : AIN2. Enable audio R PGA */
			/* Ana_Set_Reg(AUDENC_ANA_CON15, 0x0020, 0x00f0); //Audio R PGA 12 dB gain */
			Ana_Set_Reg(AUDENC_ANA_CON10, 0x0033, 0x00ff);
			/* Audio R PGA 18 dB gain MT6328 */

			Ana_Set_Reg(AUDENC_ANA_CON1, 0x00C7, 0xffff);
			/* Audio R preamplifier DCCEN */
		}

		Ana_Set_Reg(AUDENC_ANA_CON3, 0x0800, 0x0900);
		/* PGA stb enhance */

		if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC1) {
			/* main and headset mic */

			if (mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1] == 0) {
				/* "ADC1", main_mic */
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0547, 0xffff);
				/* Audio L ADC input sel : L PGA. Enable audio L ADC */
				udelay(100);
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0543, 0xffff);
				/* Audio L preamplifier DCC precharge off */
			} else if (mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1] == 1) {
				/* "ADC2", headset mic */
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0587, 0xffff);
				/* Audio L ADC input sel : L PGA. Enable audio L ADC */
				udelay(100);
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0583, 0xffff);
				/* Audio L preamplifier DCC precharge off */
			}
		} else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC2) {
			/* ref mic */
			Ana_Set_Reg(AUDENC_ANA_CON1, 0x05C7, 0xffff);
			/* Audio R ADC input sel : R PGA. Enable audio R ADC */
			udelay(100);
			Ana_Set_Reg(AUDENC_ANA_CON1, 0x05C3, 0xffff);
			/* Audio R preamplifier DCC precharge off */
		}

		SetMicPGAGain();

		if (GetAdcStatus() == false) {
			/* here to set digital part */
			Topck_Enable(true);
			/* AdcClockEnable(true); */

			Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x0000, 0xffff);
			/* Audio system digital clock power down release */
			Ana_Set_Reg(PMIC_AFE_TOP_CON0, 0x0000, 0xffff);
			/* configure ADC setting */
			Ana_Set_Reg(AFE_UL_DL_CON0, 0x0001, 0xffff);
			/* [0] afe enable */

			Ana_Set_Reg(AFE_UL_SRC0_CON0_H,
			(ULSampleRateTransform(SampleRate_VUL1) << 3 | ULSampleRateTransform(SampleRate_VUL1) << 1),
			0x001f);
			/* UL sample rate and mode configure */
			Ana_Set_Reg(AFE_UL_SRC0_CON0_L, 0x0001, 0xffff);	/* UL turn on */
		}
#if 0
		Ana_Set_Reg(PMIC_AFE_TOP_CON0, 0x0002, 0x2);	/* set DL sine gen table */
		Ana_Set_Reg(AFE_SGEN_CFG0, 0x0080, 0xffff);
		Ana_Set_Reg(AFE_SGEN_CFG1, 0x0101, 0xffff);
#endif
	} else {
		if (GetAdcStatus() == false) {
			Ana_Set_Reg(AFE_UL_SRC0_CON0_L, 0x0000, 0xffff);
			/* UL turn off */
			Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x0020, 0x0020);
			/* up-link power down */
		}
		if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC1) {
			/* main and headset mic */
			if (mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1] == 0) {
				/* "ADC1", main_mic */
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0043, 0xffff);
				/* Audio L ADC input sel : off, disable audio L ADC */
				Ana_Set_Reg(AUDENC_ANA_CON3, 0x0000, 0xffff);
				/* PGA stb enhance off */
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0041, 0xffff);
				/* Audio L preamplifier DCCEN disable */
				Ana_Set_Reg(AUDENC_ANA_CON10, 0x0000, 0x000f);
				/* Audio L PGA 0 dB gain */
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0000, 0xffff);
				/* Audio L preamplifier input sel : off, disable audio L PGA */
			} else if (mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1] == 1) {
				/* "ADC2", headset mic */
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0083, 0xffff);
				/* Audio L preamplifier input sel : off, disable audio L PGA */
				Ana_Set_Reg(AUDENC_ANA_CON3, 0x0000, 0xffff);
				/* PGA stb enhance off */
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0081, 0xffff);
				/* Audio L preamplifier DCCEN disable */
				Ana_Set_Reg(AUDENC_ANA_CON10, 0x0000, 0x000f);
				/* Audio L PGA 0 dB gain */
				Ana_Set_Reg(AUDENC_ANA_CON0, 0x0000, 0xffff);
				/* Audio L ADC input sel : off, disable audio L ADC */
			}

			Ana_Set_Reg(AUDENC_ANA_CON4, 0x0000, 0xffff);	/*  */
			Ana_Set_Reg(AUDENC_ANA_CON6, 0x2020, 0xffff);	/*  */

			if (mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1] == 0) {
				/* "ADC1", main_mic */
				/* Ana_Set_Reg(AUDENC_ANA_CON9, (MicbiasRef|0x0000), 0xff09);
				//disable MICBIAS0, restore to micbias set by accdet */
				Ana_Set_Reg(AUDENC_ANA_CON9, (MicbiasRef | 0x0000), 0xff19);
				/* disable MICBIAS0 and MICBIAS1, restore to micbias set by accdet */
			} else if (mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1] == 1) {
				/* "ADC2", headset mic */
				Ana_Set_Reg(AUDENC_ANA_CON9, (MicbiasRef | 0x0000), 0xff90);
				/* disable MICBIAS1, restore to micbias set by accdet */
			}
		} else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC2) {
			/* ref mic */
			Ana_Set_Reg(AUDENC_ANA_CON1, 0x00C3, 0xffff);
			/* Audio R ADC input sel : off, disable audio R ADC */
			Ana_Set_Reg(AUDENC_ANA_CON3, 0x0000, 0xffff);
			/* PGA stb enhance off */
			Ana_Set_Reg(AUDENC_ANA_CON1, 0x00C1, 0xffff);
			/* Audio R preamplifier DCCEN disable */
			Ana_Set_Reg(AUDENC_ANA_CON10, 0x0000, 0x00f0);
			/* Audio R PGA 0 dB gain */
			Ana_Set_Reg(AUDENC_ANA_CON1, 0x0000, 0xffff);
			/* Audio R preamplifier input sel : off, disable audio R PGA */

			Ana_Set_Reg(AUDENC_ANA_CON4, 0x0000, 0xffff);
			/*  */
			Ana_Set_Reg(AUDENC_ANA_CON6, 0x2020, 0xffff);
			/*  */
			/* Ana_Set_Reg(AUDENC_ANA_CON9, (MicbiasRef|0x0000), 0xff09);
			//disable MICBIAS0, restore to micbias set by accdet */
			Ana_Set_Reg(AUDENC_ANA_CON9, (MicbiasRef | 0x0000), 0xff19);
			/* disable MICBIAS0 and MICBIAS1, restore to micbias set by accdet */
		}

		if (GetAdcStatus() == false) {
			Ana_Set_Reg(AFE_DCCLK_CFG0, 0x2060, 0xffff);
			/* dcclk_gen_on=1'b0 */
			Ana_Set_Reg(AFE_DCCLK_CFG0, 0x2062, 0x0002);
			/* dcclk_pdn=1'b0 */

			Ana_Set_Reg(AUDDEC_ANA_CON7, 0x0000, 0x0006);
			/* LCLDO_ENC remote sense off */
			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0004, 0x0104);
			/* disable LCLDO_ENC 1P8V */
			Ana_Set_Reg(AUDENC_ANA_CON3, 0x0000, 0xffff);
			/* disable ADC CLK from CLKGEN (13MHz) */
			Ana_Set_Reg(AUDDEC_ANA_CON6, 0x0000, 0x0104);
			/* disable audio ADC CLKGEN */

			if (GetDLStatus() == false) {
				Ana_Set_Reg(AFE_UL_DL_CON0, 0x0000, 0xffff);
				/* afe disable */
				Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x0084, 0x0084);
				/* afe power down and total audio clk disable */
			}

			/* AdcClockEnable(false); */
			Topck_Enable(false);
			/* ClsqAuxEnable(false); */
			ClsqEnable(false);
			NvregEnable(false);
			audckbufEnable(false);
		}
		GetMicbias = 0;
	}
	return true;
}


static bool TurnOnVOWDigitalHW(bool enable)
{
	return false;
}

static bool TurnOnVOWADcPowerACC(int MicType, bool enable)
{
	return false;
}



/* here start uplink power function */
static const char * const ADC_function[] = { "Off", "On" };
static const char * const ADC_power_mode[] = { "normal", "lowpower" };
static const char * const PreAmp_Mux_function[] = { "OPEN", "IN_ADC1", "IN_ADC2", "IN_ADC3" };
/* OPEN:0, IN_ADC1: 1, IN_ADC2:2, IN_ADC3:3 */
static const char * const ADC_UL_PGA_GAIN[] = { "0Db", "6Db", "12Db", "18Db", "24Db", "30Db" };
static const char * const Pmic_Digital_Mux[] = { "ADC1", "ADC2", "ADC3", "ADC4" };
static const char * const Adc_Input_Sel[] = { "idle", "AIN", "Preamp" };
static const char * const Audio_AnalogMic_Mode[] = {
	"ACCMODE", "DCCMODE", "DMIC", "DCCECMDIFFMODE", "DCCECMSINGLEMODE" };
static const char * const Audio_VOW_ADC_Function[] = { "Off", "On" };
static const char * const Audio_VOW_Digital_Function[] = { "Off", "On" };
static const char * const Audio_VOW_MIC_Type[] = {
	"HandsetAMIC", "HeadsetMIC", "HandsetDMIC", "HandsetDMIC_800K", "HandsetAMIC_DCC",
	"HeadsetMIC_DCC", "HandsetAMIC_DCCECM", "HeadsetMIC_DCCECM" };

/* here start uplink power function */
static const char * const Pmic_Test_function[] = { "Off", "On" };
static const char * const Pmic_LPBK_function[] = { "Off", "LPBK3_DL44_UL48", "LPBK3_DL48_UL48" };

static const struct soc_enum Audio_UL_Enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_function), ADC_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_function), ADC_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_function), ADC_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_function), ADC_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(PreAmp_Mux_function), PreAmp_Mux_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Adc_Input_Sel), Adc_Input_Sel),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Adc_Input_Sel), Adc_Input_Sel),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Adc_Input_Sel), Adc_Input_Sel),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Adc_Input_Sel), Adc_Input_Sel),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_UL_PGA_GAIN), ADC_UL_PGA_GAIN),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_UL_PGA_GAIN), ADC_UL_PGA_GAIN),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_UL_PGA_GAIN), ADC_UL_PGA_GAIN),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_UL_PGA_GAIN), ADC_UL_PGA_GAIN),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Pmic_Digital_Mux), Pmic_Digital_Mux),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Pmic_Digital_Mux), Pmic_Digital_Mux),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Pmic_Digital_Mux), Pmic_Digital_Mux),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Pmic_Digital_Mux), Pmic_Digital_Mux),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Audio_AnalogMic_Mode), Audio_AnalogMic_Mode),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Audio_AnalogMic_Mode), Audio_AnalogMic_Mode),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Audio_AnalogMic_Mode), Audio_AnalogMic_Mode),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Audio_AnalogMic_Mode), Audio_AnalogMic_Mode),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_power_mode), ADC_power_mode),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Audio_VOW_ADC_Function), Audio_VOW_ADC_Function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(PreAmp_Mux_function), PreAmp_Mux_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Audio_VOW_Digital_Function), Audio_VOW_Digital_Function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Audio_VOW_MIC_Type), Audio_VOW_MIC_Type),
};

static int Audio_ADC1_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("Audio_ADC1_Get = %d\n",
		 mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC1]);
	ucontrol->value.integer.value[0] =
	    mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC1];
	return 0;
}

static int Audio_ADC1_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	mutex_lock(&Ana_Power_Mutex);
	if (ucontrol->value.integer.value[0]) {
		if (mAudio_Analog_Mic1_mode == AUDIO_ANALOGUL_MODE_ACC)
			TurnOnADcPowerACC(AUDIO_ANALOG_DEVICE_IN_ADC1, true);
		else if (mAudio_Analog_Mic1_mode == AUDIO_ANALOGUL_MODE_DCC)
			TurnOnADcPowerDCC(AUDIO_ANALOG_DEVICE_IN_ADC1, true, 0);
		else if (mAudio_Analog_Mic1_mode == AUDIO_ANALOGUL_MODE_DMIC)
			TurnOnADcPowerDmic(AUDIO_ANALOG_DEVICE_IN_ADC1, true);
		else if (mAudio_Analog_Mic1_mode == AUDIO_ANALOGUL_MODE_DCCECMDIFF)
			TurnOnADcPowerDCC(AUDIO_ANALOG_DEVICE_IN_ADC1, true, 1);
		else if (mAudio_Analog_Mic1_mode == AUDIO_ANALOGUL_MODE_DCCECMSINGLE)
			TurnOnADcPowerDCC(AUDIO_ANALOG_DEVICE_IN_ADC1, true, 2);

		mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC1] =
		    ucontrol->value.integer.value[0];
	} else {
		mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC1] =
		    ucontrol->value.integer.value[0];
		if (mAudio_Analog_Mic1_mode == AUDIO_ANALOGUL_MODE_ACC)
			TurnOnADcPowerACC(AUDIO_ANALOG_DEVICE_IN_ADC1, false);
		else if (mAudio_Analog_Mic1_mode == AUDIO_ANALOGUL_MODE_DCC)
			TurnOnADcPowerDCC(AUDIO_ANALOG_DEVICE_IN_ADC1, false, 0);
		else if (mAudio_Analog_Mic1_mode == AUDIO_ANALOGUL_MODE_DMIC)
			TurnOnADcPowerDmic(AUDIO_ANALOG_DEVICE_IN_ADC1, false);
		else if (mAudio_Analog_Mic1_mode == AUDIO_ANALOGUL_MODE_DCCECMDIFF)
			TurnOnADcPowerDCC(AUDIO_ANALOG_DEVICE_IN_ADC1, false, 1);
		else if (mAudio_Analog_Mic1_mode == AUDIO_ANALOGUL_MODE_DCCECMSINGLE)
			TurnOnADcPowerDCC(AUDIO_ANALOG_DEVICE_IN_ADC1, false, 2);

	}
	mutex_unlock(&Ana_Power_Mutex);
	return 0;
}

static int Audio_ADC2_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("Audio_ADC2_Get = %d\n",
		 mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC2]);
	ucontrol->value.integer.value[0] =
	    mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC2];
	return 0;
}

static int Audio_ADC2_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	mutex_lock(&Ana_Power_Mutex);
	if (ucontrol->value.integer.value[0]) {
		if (mAudio_Analog_Mic2_mode == AUDIO_ANALOGUL_MODE_ACC)
			TurnOnADcPowerACC(AUDIO_ANALOG_DEVICE_IN_ADC2, true);
		else if (mAudio_Analog_Mic2_mode == AUDIO_ANALOGUL_MODE_DCC)
			TurnOnADcPowerDCC(AUDIO_ANALOG_DEVICE_IN_ADC2, true, 0);
		else if (mAudio_Analog_Mic2_mode == AUDIO_ANALOGUL_MODE_DMIC)
			TurnOnADcPowerDmic(AUDIO_ANALOG_DEVICE_IN_ADC2, true);
		else if (mAudio_Analog_Mic2_mode == AUDIO_ANALOGUL_MODE_DCCECMDIFF)
			TurnOnADcPowerDCC(AUDIO_ANALOG_DEVICE_IN_ADC2, true, 1);
		else if (mAudio_Analog_Mic2_mode == AUDIO_ANALOGUL_MODE_DCCECMSINGLE)
			TurnOnADcPowerDCC(AUDIO_ANALOG_DEVICE_IN_ADC2, true, 2);

		mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC2] =
		    ucontrol->value.integer.value[0];
	} else {
		mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC2] =
		    ucontrol->value.integer.value[0];
		if (mAudio_Analog_Mic2_mode == AUDIO_ANALOGUL_MODE_ACC)
			TurnOnADcPowerACC(AUDIO_ANALOG_DEVICE_IN_ADC2, false);
		else if (mAudio_Analog_Mic2_mode == AUDIO_ANALOGUL_MODE_DCC)
			TurnOnADcPowerDCC(AUDIO_ANALOG_DEVICE_IN_ADC2, false, 0);
		else if (mAudio_Analog_Mic2_mode == AUDIO_ANALOGUL_MODE_DMIC)
			TurnOnADcPowerDmic(AUDIO_ANALOG_DEVICE_IN_ADC2, false);
		else if (mAudio_Analog_Mic2_mode == AUDIO_ANALOGUL_MODE_DCCECMDIFF)
			TurnOnADcPowerDCC(AUDIO_ANALOG_DEVICE_IN_ADC2, false, 1);
		else if (mAudio_Analog_Mic2_mode == AUDIO_ANALOGUL_MODE_DCCECMSINGLE)
			TurnOnADcPowerDCC(AUDIO_ANALOG_DEVICE_IN_ADC2, false, 2);

	}
	mutex_unlock(&Ana_Power_Mutex);
	return 0;
}

static int Audio_ADC3_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* 6752 removed */
	return 0;
}

static int Audio_ADC3_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* 6752 removed */
	return 0;
}

static int Audio_ADC4_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* 6752 removed */
	return 0;
}

static int Audio_ADC4_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* 6752 removed */
	return 0;
}

static int Audio_ADC1_Sel_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s() = %d\n", __func__, mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC1]);
	ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC1];
	return 0;
}

static int Audio_ADC1_Sel_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);

	if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Adc_Input_Sel)) {
		pr_warn("return -EINVAL\n");
		return -EINVAL;
	}

	if (ucontrol->value.integer.value[0] == 0)
		Ana_Set_Reg(AUDENC_ANA_CON0, (0x0000 << 9), 0x0600);	/* pinumx sel */
	else if (ucontrol->value.integer.value[0] == 1)
		Ana_Set_Reg(AUDENC_ANA_CON0, (0x0001 << 9), 0x0600);	/* AIN0 */
	else if (ucontrol->value.integer.value[0] == 2)
		Ana_Set_Reg(AUDENC_ANA_CON0, (0x0002 << 9), 0x0600);	/* Left preamp */
	else
		pr_warn("%s() warning\n ", __func__);

	pr_debug("%s() done\n", __func__);
	mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC1] = ucontrol->value.integer.value[0];
	return 0;
}

static int Audio_ADC2_Sel_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s() = %d\n", __func__, mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC2]);
	ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC2];
	return 0;
}

static int Audio_ADC2_Sel_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Adc_Input_Sel)) {
		pr_warn("return -EINVAL\n");
		return -EINVAL;
	}

	if (ucontrol->value.integer.value[0] == 0)
		Ana_Set_Reg(AUDENC_ANA_CON1, (0x0000 << 9), 0x0600);	/* pinumx sel */
	else if (ucontrol->value.integer.value[0] == 1)
		Ana_Set_Reg(AUDENC_ANA_CON1, (0x0001 << 9), 0x0600);	/* AIN2 */
	else if (ucontrol->value.integer.value[0] == 2)	/* Right preamp */
		Ana_Set_Reg(AUDENC_ANA_CON1, (0x0002 << 9), 0x0600);
	else
		pr_warn("%s() warning\n ", __func__);

	pr_debug("%s() done\n", __func__);
	mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC2] = ucontrol->value.integer.value[0];
	return 0;
}

static int Audio_ADC3_Sel_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* 6752 removed */
	return 0;
}

static int Audio_ADC3_Sel_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* 6752 removed */
	return 0;
}

static int Audio_ADC4_Sel_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* 6752 removed */
	return 0;
}

static int Audio_ADC4_Sel_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* 6752 removed */
	return 0;
}


static bool AudioPreAmp1_Sel(int Mul_Sel)
{
	pr_debug("%s Mul_Sel = %d ", __func__, Mul_Sel);
	if (Mul_Sel == 0)
		Ana_Set_Reg(AUDENC_ANA_CON0, 0x0000, 0x00C0);	/* pinumx open */
	else if (Mul_Sel == 1)
		Ana_Set_Reg(AUDENC_ANA_CON0, 0x0040, 0x00C0);	/* AIN0 */
	else if (Mul_Sel == 2)
		Ana_Set_Reg(AUDENC_ANA_CON0, 0x0080, 0x00C0);	/* AIN1 */
	else if (Mul_Sel == 3)
		Ana_Set_Reg(AUDENC_ANA_CON0, 0x00C0, 0x00C0);	/* AIN2 */
	else
		pr_warn("AudioPreAmp1_Sel warning");

	return true;
}


static int Audio_PreAmp1_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s() mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_1]; = %d\n", __func__,
		 mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_1]);
	ucontrol->value.integer.value[0] =
	    mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_1];
	return 0;
}

static int Audio_PreAmp1_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);

	if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(PreAmp_Mux_function)) {
		pr_warn("return -EINVAL\n");
		return -EINVAL;
	}
	mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_1] =
	    ucontrol->value.integer.value[0];
	AudioPreAmp1_Sel(mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_1]);
	pr_debug("%s() done\n", __func__);
	return 0;
}

static bool AudioPreAmp2_Sel(int Mul_Sel)
{
	pr_debug("%s Mul_Sel = %d ", __func__, Mul_Sel);

	if (Mul_Sel == 0)
		Ana_Set_Reg(AUDENC_ANA_CON1, 0x0000, 0x00C0);	/* pinumx open */
	else if (Mul_Sel == 1)
		Ana_Set_Reg(AUDENC_ANA_CON1, 0x0040, 0x00C0);	/* AIN0 */
	else if (Mul_Sel == 2)
		Ana_Set_Reg(AUDENC_ANA_CON1, 0x0080, 0x00C0);	/* AIN1 */
	else if (Mul_Sel == 3)
		Ana_Set_Reg(AUDENC_ANA_CON1, 0x00C0, 0x00C0);	/* AIN2 */
	else
		pr_warn("AudioPreAmp1_Sel warning");

	return true;
}


static int Audio_PreAmp2_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s() mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_2]; = %d\n", __func__,
	       mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_2]);
	ucontrol->value.integer.value[0] =
	    mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_2];
	return 0;
}

static int Audio_PreAmp2_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);

	if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(PreAmp_Mux_function)) {
		pr_warn("return -EINVAL\n");
		return -EINVAL;
	}
	mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_2] =
	    ucontrol->value.integer.value[0];
	AudioPreAmp2_Sel(mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_2]);
	pr_debug("%s() done\n", __func__);
	return 0;
}

/* PGA1: PGA_L */
static int Audio_PGA1_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("Audio_AmpR_Get = %d\n",
	       mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP1]);
	ucontrol->value.integer.value[0] =
	    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP1];
	return 0;
}

static int Audio_PGA1_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int index = 0;

	pr_debug("%s()\n", __func__);
	if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(ADC_UL_PGA_GAIN)) {
		pr_warn("return -EINVAL\n");
		return -EINVAL;
	}
	index = ucontrol->value.integer.value[0];
	Ana_Set_Reg(AUDENC_ANA_CON10, index, 0x0007);
	mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP1] =
	    ucontrol->value.integer.value[0];
	return 0;
}

/* PGA2: PGA_R */
static int Audio_PGA2_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("Audio_PGA2_Get = %d\n",
	       mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP2]);
	ucontrol->value.integer.value[0] =
	    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP2];
	return 0;
}

static int Audio_PGA2_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int index = 0;

	pr_debug("%s()\n", __func__);
	if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(ADC_UL_PGA_GAIN)) {
		pr_warn("return -EINVAL\n");
		return -EINVAL;
	}
	index = ucontrol->value.integer.value[0];
	Ana_Set_Reg(AUDENC_ANA_CON10, index << 4, 0x0070);
	mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP2] =
	    ucontrol->value.integer.value[0];
	return 0;
}

static int Audio_PGA3_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* 6752 removed */
	return 0;
}

static int Audio_PGA3_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* 6752 removed */
	return 0;
}

static int Audio_PGA4_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* 6752 removed */
	return 0;
}

static int Audio_PGA4_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* 6752 removed */
	return 0;
}

static int Audio_MicSource1_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("Audio_MicSource1_Get = %d\n",
	       mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1]);
	ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1];
	return 0;
}

static int Audio_MicSource1_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* 6752 used for ADC1 Mic source selection, "ADC1" is main_mic, "ADC2" is headset_mic */
	int index = 0;

	pr_debug("%s()\n", __func__);
	if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Pmic_Digital_Mux)) {
		pr_warn("return -EINVAL\n");
		return -EINVAL;
	}
	index = ucontrol->value.integer.value[0];
	pr_debug("%s() index = %d done\n", __func__, index);
	mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1] = ucontrol->value.integer.value[0];

	return 0;
}

static int Audio_MicSource2_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* 6752 removed */
	return 0;
}

static int Audio_MicSource2_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* 6752 removed */
	return 0;
}

static int Audio_MicSource3_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* 6752 removed */
	return 0;
}

static int Audio_MicSource3_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* 6752 removed */
	return 0;
}


static int Audio_MicSource4_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* 6752 removed */
	return 0;
}

static int Audio_MicSource4_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* 6752 removed */
	return 0;
}

/* Mic ACC/DCC Mode Setting */
static int Audio_Mic1_Mode_Select_Get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s() mAudio_Analog_Mic1_mode = %d\n", __func__, mAudio_Analog_Mic1_mode);
	ucontrol->value.integer.value[0] = mAudio_Analog_Mic1_mode;
	return 0;
}

static int Audio_Mic1_Mode_Select_Set(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Audio_AnalogMic_Mode)) {
		pr_warn("return -EINVAL\n");
		return -EINVAL;
	}
	mAudio_Analog_Mic1_mode = ucontrol->value.integer.value[0];
	pr_debug("%s() mAudio_Analog_Mic1_mode = %d\n", __func__, mAudio_Analog_Mic1_mode);
	return 0;
}

static int Audio_Mic2_Mode_Select_Get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()  = %d\n", __func__, mAudio_Analog_Mic2_mode);
	ucontrol->value.integer.value[0] = mAudio_Analog_Mic2_mode;
	return 0;
}

static int Audio_Mic2_Mode_Select_Set(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Audio_AnalogMic_Mode)) {
		pr_warn("return -EINVAL\n");
		return -EINVAL;
	}
	mAudio_Analog_Mic2_mode = ucontrol->value.integer.value[0];
	pr_debug("%s() mAudio_Analog_Mic2_mode = %d\n", __func__, mAudio_Analog_Mic2_mode);
	return 0;
}


static int Audio_Mic3_Mode_Select_Get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()  = %d\n", __func__, mAudio_Analog_Mic3_mode);
	ucontrol->value.integer.value[0] = mAudio_Analog_Mic3_mode;
	return 0;
}

static int Audio_Mic3_Mode_Select_Set(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Audio_AnalogMic_Mode)) {
		pr_warn("return -EINVAL\n");
		return -EINVAL;
	}
	mAudio_Analog_Mic3_mode = ucontrol->value.integer.value[0];
	pr_debug("%s() mAudio_Analog_Mic3_mode = %d\n", __func__, mAudio_Analog_Mic3_mode);
	return 0;
}

static int Audio_Mic4_Mode_Select_Get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()  = %d\n", __func__, mAudio_Analog_Mic4_mode);
	ucontrol->value.integer.value[0] = mAudio_Analog_Mic4_mode;
	return 0;
}

static int Audio_Mic4_Mode_Select_Set(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Audio_AnalogMic_Mode)) {
		pr_warn("return -EINVAL\n");
		return -EINVAL;
	}
	mAudio_Analog_Mic4_mode = ucontrol->value.integer.value[0];
	pr_debug("%s() mAudio_Analog_Mic4_mode = %d\n", __func__, mAudio_Analog_Mic4_mode);
	return 0;
}

static int Audio_Adc_Power_Mode_Get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()  = %d\n", __func__, mAdc_Power_Mode);
	ucontrol->value.integer.value[0] = mAdc_Power_Mode;
	return 0;
}

static int Audio_Adc_Power_Mode_Set(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(ADC_power_mode)) {
		pr_warn("return -EINVAL\n");
		return -EINVAL;
	}
	mAdc_Power_Mode = ucontrol->value.integer.value[0];
	pr_debug("%s() mAdc_Power_Mode = %d\n", __func__, mAdc_Power_Mode);
	return 0;
}


static int Audio_Vow_ADC_Func_Switch_Get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()  = %d\n", __func__, mAudio_Vow_Analog_Func_Enable);
	ucontrol->value.integer.value[0] = mAudio_Vow_Analog_Func_Enable;
	return 0;
}

static int Audio_Vow_ADC_Func_Switch_Set(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Audio_VOW_ADC_Function)) {
		pr_warn("return -EINVAL\n");
		return -EINVAL;
	}

	if (ucontrol->value.integer.value[0])
		TurnOnVOWADcPowerACC(mAudio_VOW_Mic_type, true);
	else
		TurnOnVOWADcPowerACC(mAudio_VOW_Mic_type, false);


	mAudio_Vow_Analog_Func_Enable = ucontrol->value.integer.value[0];
	pr_debug("%s() mAudio_Vow_Analog_Func_Enable = %d\n", __func__,
		 mAudio_Vow_Analog_Func_Enable);
	return 0;
}

static int Audio_Vow_Digital_Func_Switch_Get(struct snd_kcontrol *kcontrol,
					     struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()  = %d\n", __func__, mAudio_Vow_Digital_Func_Enable);
	ucontrol->value.integer.value[0] = mAudio_Vow_Digital_Func_Enable;
	return 0;
}

static int Audio_Vow_Digital_Func_Switch_Set(struct snd_kcontrol *kcontrol,
					     struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Audio_VOW_Digital_Function)) {
		pr_warn("return -EINVAL\n");
		return -EINVAL;
	}

	if (ucontrol->value.integer.value[0])
		TurnOnVOWDigitalHW(true);
	else
		TurnOnVOWDigitalHW(false);


	mAudio_Vow_Digital_Func_Enable = ucontrol->value.integer.value[0];
	pr_debug("%s() mAudio_Vow_Digital_Func_Enable = %d\n", __func__,
		 mAudio_Vow_Digital_Func_Enable);
	return 0;
}


static int Audio_Vow_MIC_Type_Select_Get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()  = %d\n", __func__, mAudio_VOW_Mic_type);
	ucontrol->value.integer.value[0] = mAudio_VOW_Mic_type;
	return 0;
}

static int Audio_Vow_MIC_Type_Select_Set(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Audio_VOW_MIC_Type)) {
		pr_warn("return -EINVAL\n");
		return -EINVAL;
	}
	mAudio_VOW_Mic_type = ucontrol->value.integer.value[0];
	pr_debug("%s() mAudio_VOW_Mic_type = %d\n", __func__, mAudio_VOW_Mic_type);
	return 0;
}


static int Audio_Vow_Cfg0_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int value = /*Ana_Get_Reg(AFE_VOW_CFG0) */ reg_AFE_VOW_CFG0;

	pr_debug("%s()  = %d\n", __func__, value);
	ucontrol->value.integer.value[0] = value;
	return 0;
}

static int Audio_Vow_Cfg0_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()  = %d\n", __func__, (int)(ucontrol->value.integer.value[0]));
	/* Ana_Set_Reg(AFE_VOW_CFG0, ucontrol->value.integer.value[0], 0xffff); */
	reg_AFE_VOW_CFG0 = ucontrol->value.integer.value[0];
	return 0;
}

static int Audio_Vow_Cfg1_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int value = /*Ana_Get_Reg(AFE_VOW_CFG1) */ reg_AFE_VOW_CFG1;

	pr_debug("%s()  = %d\n", __func__, value);
	ucontrol->value.integer.value[0] = value;
	return 0;
}

static int Audio_Vow_Cfg1_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()  = %ld\n", __func__, ucontrol->value.integer.value[0]);
	/* Ana_Set_Reg(AFE_VOW_CFG1, ucontrol->value.integer.value[0], 0xffff); */
	reg_AFE_VOW_CFG1 = ucontrol->value.integer.value[0];
	return 0;
}

static int Audio_Vow_Cfg2_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int value = /*Ana_Get_Reg(AFE_VOW_CFG2) */ reg_AFE_VOW_CFG2;

	pr_debug("%s()  = %d\n", __func__, value);
	ucontrol->value.integer.value[0] = value;
	return 0;
}

static int Audio_Vow_Cfg2_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()  = %ld\n", __func__, ucontrol->value.integer.value[0]);
	/* Ana_Set_Reg(AFE_VOW_CFG2, ucontrol->value.integer.value[0], 0xffff); */
	reg_AFE_VOW_CFG2 = ucontrol->value.integer.value[0];
	return 0;
}

static int Audio_Vow_Cfg3_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int value = /*Ana_Get_Reg(AFE_VOW_CFG3) */ reg_AFE_VOW_CFG3;

	pr_debug("%s()  = %d\n", __func__, value);
	ucontrol->value.integer.value[0] = value;
	return 0;
}

static int Audio_Vow_Cfg3_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()  = %ld\n", __func__, ucontrol->value.integer.value[0]);
	/* Ana_Set_Reg(AFE_VOW_CFG3, ucontrol->value.integer.value[0], 0xffff); */
	reg_AFE_VOW_CFG3 = ucontrol->value.integer.value[0];
	return 0;
}

static int Audio_Vow_Cfg4_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int value = /*Ana_Get_Reg(AFE_VOW_CFG4) */ reg_AFE_VOW_CFG4;

	pr_debug("%s()  = %d\n", __func__, value);
	ucontrol->value.integer.value[0] = value;
	return 0;
}

static int Audio_Vow_Cfg4_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()  = %ld\n", __func__, ucontrol->value.integer.value[0]);
	/* Ana_Set_Reg(AFE_VOW_CFG4, ucontrol->value.integer.value[0], 0xffff); */
	reg_AFE_VOW_CFG4 = ucontrol->value.integer.value[0];
	return 0;
}

static int Audio_Vow_Cfg5_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int value = /*Ana_Get_Reg(AFE_VOW_CFG5) */ reg_AFE_VOW_CFG5;

	pr_debug("%s()  = %d\n", __func__, value);
	ucontrol->value.integer.value[0] = value;
	return 0;
}

static int Audio_Vow_Cfg5_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()  = %ld\n", __func__, ucontrol->value.integer.value[0]);
	/* Ana_Set_Reg(AFE_VOW_CFG5, ucontrol->value.integer.value[0], 0xffff); */
	reg_AFE_VOW_CFG5 = ucontrol->value.integer.value[0];
	return 0;
}

static int Audio_Vow_State_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int value = mIsVOWOn;

	pr_debug("%s()  = %d\n", __func__, value);
	ucontrol->value.integer.value[0] = value;
	return 0;
}

static int Audio_Vow_State_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* pr_debug("%s()  = %ld\n", __func__, ucontrol->value.integer.value[0]); */
	/* reg_AFE_VOW_CFG5 = ucontrol->value.integer.value[0]; */
	return 0;
}

static bool SineTable_DAC_HP_flag;
static bool SineTable_UL2_flag;

static int SineTable_UL2_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	if (ucontrol->value.integer.value[0]) {
		Ana_Set_Reg(PMIC_AFE_TOP_CON0, 0x0002, 0x2);	/* set DL sine gen table */
		Ana_Set_Reg(AFE_SGEN_CFG0, 0x0080, 0xffff);
		Ana_Set_Reg(AFE_SGEN_CFG1, 0x0101, 0xffff);
	} else {
		Ana_Set_Reg(PMIC_AFE_TOP_CON0, 0x0000, 0x2);	/* set DL sine gen table */
		Ana_Set_Reg(AFE_SGEN_CFG0, 0x0000, 0xffff);
		Ana_Set_Reg(AFE_SGEN_CFG1, 0x0101, 0xffff);
	}
	return 0;
}

static int32 Pmic_Loopback_Type;

static int Pmic_Loopback_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	ucontrol->value.integer.value[0] = Pmic_Loopback_Type;
	return 0;
}

static int Pmic_Loopback_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Pmic_LPBK_function)) {
		pr_warn("return -EINVAL\n");
		return -EINVAL;
	}

	if (ucontrol->value.integer.value[0] == 0) { /* disable pmic lpbk */
		Ana_Set_Reg(AFE_UL_SRC0_CON0_L, 0x0000, 0x0005); /* power off uplink */
		Ana_Set_Reg(AFE_DL_SRC2_CON0_L, 0x0000, 0x0001); /* turn off mute function and turn on dl */
		Ana_Set_Reg(AFE_UL_DL_CON0, 0x0000, 0x0001);   /* turn off afe UL & DL */
		Topck_Enable(false);
		ClsqEnable(false);
		audckbufEnable(false);
	} else if (ucontrol->value.integer.value[0] > 0) { /* enable pmic lpbk */
		audckbufEnable(true);
		ClsqEnable(true);
		Topck_Enable(true);

		Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x0000, 0xffff);	/* power on clock */

		/* Se DL Part */
		Ana_Set_Reg(AFE_DL_SDM_CON1, 0x001e, 0xffffffff);	/* set attenuation gain */
		Ana_Set_Reg(AFE_UL_DL_CON0, 0x0001, 0xffffffff);	/* [0] afe enable */

		if (ucontrol->value.integer.value[0] == 1) { /* pmic lpbk 3 DL44 UL48*/
			Ana_Set_Reg(AFE_PMIC_NEWIF_CFG0, 0x7330, 0xffffffff);	/* 44k sample rate */
			Ana_Set_Reg(AFE_DL_SRC2_CON0_H, 0x7300, 0xffffffff);	/* 44k sample rate */
		} else if (ucontrol->value.integer.value[0] == 2) { /* pmic lpbk 3 DL48 UL48*/
			Ana_Set_Reg(AFE_PMIC_NEWIF_CFG0, 0x8330, 0xffffffff);	/* 48k sample rate */
			Ana_Set_Reg(AFE_DL_SRC2_CON0_H, 0x8300, 0xffffffff);	/* 48k sample rate */
		}

		Ana_Set_Reg(AFE_DL_SRC2_CON0_L, 0x0001, 0x0001);
		/* turn off mute function and turn on dl */
		Ana_Set_Reg(PMIC_AFE_TOP_CON0, 0x0000, 0x0001);
		/* set DL in normal path, not from sine gen table */

		/* Set UL Part */
		Ana_Set_Reg(AFE_PMIC_NEWIF_CFG2, 0x0c00, 0x0c00);
		/* config UL up8x_rxif adc voice mode, 48k sample rate */
		Ana_Set_Reg(AFE_UL_SRC0_CON0_H, (0x3 << 3 | 0x3 << 1) , 0x001E);
		/* ULsampling rate, 48k sample rate */
		Ana_Set_Reg(AFE_UL_SRC0_CON0_L, 0x0005, 0xffff);
		/* power on uplink, and loopback from DL */
		Ana_Set_Reg(PMIC_AFE_TOP_CON0, 0x0000, 0x0002);
		/* configure ADC setting */
		Ana_Set_Reg(AFE_UL_DL_CON0, 0x0001, 0xffff);
		/* turn on afe */
	}

	pr_debug("%s() done\n", __func__);
	Pmic_Loopback_Type = ucontrol->value.integer.value[0];
	return 0;
}

static int SineTable_UL2_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	ucontrol->value.integer.value[0] = SineTable_UL2_flag;
	return 0;
}

static int SineTable_DAC_HP_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	ucontrol->value.integer.value[0] = SineTable_DAC_HP_flag;
	return 0;
}

static int SineTable_DAC_HP_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	/* 6752 TODO? */
	pr_debug("%s()\n", __func__);
	return 0;
}

static void ADC_LOOP_DAC_Func(int command)
{
	/* 6752 TODO? */
}

static bool DAC_LOOP_DAC_HS_flag;
static int ADC_LOOP_DAC_HS_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	ucontrol->value.integer.value[0] = DAC_LOOP_DAC_HS_flag;
	return 0;
}

static int ADC_LOOP_DAC_HS_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	if (ucontrol->value.integer.value[0]) {
		DAC_LOOP_DAC_HS_flag = ucontrol->value.integer.value[0];
		ADC_LOOP_DAC_Func(AUDIO_ANALOG_DAC_LOOP_DAC_HS_ON);
	} else {
		DAC_LOOP_DAC_HS_flag = ucontrol->value.integer.value[0];
		ADC_LOOP_DAC_Func(AUDIO_ANALOG_DAC_LOOP_DAC_HS_OFF);
	}
	return 0;
}

static bool DAC_LOOP_DAC_HP_flag;
static int ADC_LOOP_DAC_HP_Get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	ucontrol->value.integer.value[0] = DAC_LOOP_DAC_HP_flag;
	return 0;
}

static int ADC_LOOP_DAC_HP_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{

	pr_debug("%s()\n", __func__);
	if (ucontrol->value.integer.value[0]) {
		DAC_LOOP_DAC_HP_flag = ucontrol->value.integer.value[0];
		ADC_LOOP_DAC_Func(AUDIO_ANALOG_DAC_LOOP_DAC_HP_ON);
	} else {
		DAC_LOOP_DAC_HP_flag = ucontrol->value.integer.value[0];
		ADC_LOOP_DAC_Func(AUDIO_ANALOG_DAC_LOOP_DAC_HP_OFF);
	}
	return 0;
}

static bool Voice_Call_DAC_DAC_HS_flag;
static int Voice_Call_DAC_DAC_HS_Get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	ucontrol->value.integer.value[0] = Voice_Call_DAC_DAC_HS_flag;
	return 0;
}

static int Voice_Call_DAC_DAC_HS_Set(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	/* 6752 TODO */
	pr_debug("%s()\n", __func__);
	return 0;
}

static const struct soc_enum Pmic_Test_Enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Pmic_Test_function), Pmic_Test_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Pmic_Test_function), Pmic_Test_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Pmic_Test_function), Pmic_Test_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Pmic_Test_function), Pmic_Test_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Pmic_Test_function), Pmic_Test_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Pmic_LPBK_function), Pmic_LPBK_function),
};

static const struct snd_kcontrol_new mt6331_pmic_Test_controls[] = {
	SOC_ENUM_EXT("SineTable_DAC_HP", Pmic_Test_Enum[0], SineTable_DAC_HP_Get,
		     SineTable_DAC_HP_Set),
	SOC_ENUM_EXT("DAC_LOOP_DAC_HS", Pmic_Test_Enum[1], ADC_LOOP_DAC_HS_Get,
		     ADC_LOOP_DAC_HS_Set),
	SOC_ENUM_EXT("DAC_LOOP_DAC_HP", Pmic_Test_Enum[2], ADC_LOOP_DAC_HP_Get,
		     ADC_LOOP_DAC_HP_Set),
	SOC_ENUM_EXT("Voice_Call_DAC_DAC_HS", Pmic_Test_Enum[3], Voice_Call_DAC_DAC_HS_Get,
		     Voice_Call_DAC_DAC_HS_Set),
	SOC_ENUM_EXT("SineTable_UL2", Pmic_Test_Enum[4], SineTable_UL2_Get, SineTable_UL2_Set),
	SOC_ENUM_EXT("Pmic_Loopback", Pmic_Test_Enum[5], Pmic_Loopback_Get, Pmic_Loopback_Set),
	SOC_SINGLE_EXT("Audio HP ImpeDance Setting", SND_SOC_NOPM, 0, 0x10000, 0,
		       pmic_hp_impedance_get, pmic_hp_impedance_set),
};

static const struct snd_kcontrol_new mt6331_UL_Codec_controls[] = {
	SOC_ENUM_EXT("Audio_ADC_1_Switch", Audio_UL_Enum[0], Audio_ADC1_Get, Audio_ADC1_Set),
	SOC_ENUM_EXT("Audio_ADC_2_Switch", Audio_UL_Enum[1], Audio_ADC2_Get, Audio_ADC2_Set),
	SOC_ENUM_EXT("Audio_ADC_3_Switch", Audio_UL_Enum[2], Audio_ADC3_Get, Audio_ADC3_Set),
	SOC_ENUM_EXT("Audio_ADC_4_Switch", Audio_UL_Enum[3], Audio_ADC4_Get, Audio_ADC4_Set),
	SOC_ENUM_EXT("Audio_Preamp1_Switch", Audio_UL_Enum[4], Audio_PreAmp1_Get,
		     Audio_PreAmp1_Set),
	SOC_ENUM_EXT("Audio_ADC_1_Sel", Audio_UL_Enum[5], Audio_ADC1_Sel_Get, Audio_ADC1_Sel_Set),
	SOC_ENUM_EXT("Audio_ADC_2_Sel", Audio_UL_Enum[6], Audio_ADC2_Sel_Get, Audio_ADC2_Sel_Set),
	SOC_ENUM_EXT("Audio_ADC_3_Sel", Audio_UL_Enum[7], Audio_ADC3_Sel_Get, Audio_ADC3_Sel_Set),
	SOC_ENUM_EXT("Audio_ADC_4_Sel", Audio_UL_Enum[8], Audio_ADC4_Sel_Get, Audio_ADC4_Sel_Set),
	SOC_ENUM_EXT("Audio_PGA1_Setting", Audio_UL_Enum[9], Audio_PGA1_Get, Audio_PGA1_Set),
	SOC_ENUM_EXT("Audio_PGA2_Setting", Audio_UL_Enum[10], Audio_PGA2_Get, Audio_PGA2_Set),
	SOC_ENUM_EXT("Audio_PGA3_Setting", Audio_UL_Enum[11], Audio_PGA3_Get, Audio_PGA3_Set),
	SOC_ENUM_EXT("Audio_PGA4_Setting", Audio_UL_Enum[12], Audio_PGA4_Get, Audio_PGA4_Set),
	SOC_ENUM_EXT("Audio_MicSource1_Setting", Audio_UL_Enum[13], Audio_MicSource1_Get,
		     Audio_MicSource1_Set),
	SOC_ENUM_EXT("Audio_MicSource2_Setting", Audio_UL_Enum[14], Audio_MicSource2_Get,
		     Audio_MicSource2_Set),
	SOC_ENUM_EXT("Audio_MicSource3_Setting", Audio_UL_Enum[15], Audio_MicSource3_Get,
		     Audio_MicSource3_Set),
	SOC_ENUM_EXT("Audio_MicSource4_Setting", Audio_UL_Enum[16], Audio_MicSource4_Get,
		     Audio_MicSource4_Set),
	SOC_ENUM_EXT("Audio_MIC1_Mode_Select", Audio_UL_Enum[17], Audio_Mic1_Mode_Select_Get,
		     Audio_Mic1_Mode_Select_Set),
	SOC_ENUM_EXT("Audio_MIC2_Mode_Select", Audio_UL_Enum[18], Audio_Mic2_Mode_Select_Get,
		     Audio_Mic2_Mode_Select_Set),
	SOC_ENUM_EXT("Audio_MIC3_Mode_Select", Audio_UL_Enum[19], Audio_Mic3_Mode_Select_Get,
		     Audio_Mic3_Mode_Select_Set),
	SOC_ENUM_EXT("Audio_MIC4_Mode_Select", Audio_UL_Enum[20], Audio_Mic4_Mode_Select_Get,
		     Audio_Mic4_Mode_Select_Set),
	SOC_ENUM_EXT("Audio_Mic_Power_Mode", Audio_UL_Enum[21], Audio_Adc_Power_Mode_Get,
		     Audio_Adc_Power_Mode_Set),
	SOC_ENUM_EXT("Audio_Vow_ADC_Func_Switch", Audio_UL_Enum[22], Audio_Vow_ADC_Func_Switch_Get,
		     Audio_Vow_ADC_Func_Switch_Set),
	SOC_ENUM_EXT("Audio_Preamp2_Switch", Audio_UL_Enum[23], Audio_PreAmp2_Get,
		     Audio_PreAmp2_Set),
	SOC_ENUM_EXT("Audio_Vow_Digital_Func_Switch", Audio_UL_Enum[24],
		     Audio_Vow_Digital_Func_Switch_Get, Audio_Vow_Digital_Func_Switch_Set),
	SOC_ENUM_EXT("Audio_Vow_MIC_Type_Select", Audio_UL_Enum[25], Audio_Vow_MIC_Type_Select_Get,
		     Audio_Vow_MIC_Type_Select_Set),
	SOC_SINGLE_EXT("Audio VOWCFG0 Data", SND_SOC_NOPM, 0, 0x80000, 0, Audio_Vow_Cfg0_Get,
		       Audio_Vow_Cfg0_Set),
	SOC_SINGLE_EXT("Audio VOWCFG1 Data", SND_SOC_NOPM, 0, 0x80000, 0, Audio_Vow_Cfg1_Get,
		       Audio_Vow_Cfg1_Set),
	SOC_SINGLE_EXT("Audio VOWCFG2 Data", SND_SOC_NOPM, 0, 0x80000, 0, Audio_Vow_Cfg2_Get,
		       Audio_Vow_Cfg2_Set),
	SOC_SINGLE_EXT("Audio VOWCFG3 Data", SND_SOC_NOPM, 0, 0x80000, 0, Audio_Vow_Cfg3_Get,
		       Audio_Vow_Cfg3_Set),
	SOC_SINGLE_EXT("Audio VOWCFG4 Data", SND_SOC_NOPM, 0, 0x80000, 0, Audio_Vow_Cfg4_Get,
		       Audio_Vow_Cfg4_Set),
	SOC_SINGLE_EXT("Audio VOWCFG5 Data", SND_SOC_NOPM, 0, 0x80000, 0, Audio_Vow_Cfg5_Get,
		       Audio_Vow_Cfg5_Set),
	SOC_SINGLE_EXT("Audio_VOW_State", SND_SOC_NOPM, 0, 0x80000, 0, Audio_Vow_State_Get,
		       Audio_Vow_State_Set),
};

static const struct snd_soc_dapm_widget mt6331_dapm_widgets[] = {
	/* Outputs */
	SND_SOC_DAPM_OUTPUT("EARPIECE"),
	SND_SOC_DAPM_OUTPUT("HEADSET"),
	SND_SOC_DAPM_OUTPUT("SPEAKER"),
	/*
	   SND_SOC_DAPM_MUX_E("VOICE_Mux_E", SND_SOC_NOPM, 0, 0  , &mt6331_Voice_Switch, codec_enable_rx_bias,
	   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
	   SND_SOC_DAPM_PRE_REG | SND_SOC_DAPM_POST_REG),
	 */

};

static const struct snd_soc_dapm_route mtk_audio_map[] = {
	{"VOICE_Mux_E", "Voice Mux", "SPEAKER PGA"},
};

static void mt6331_codec_init_reg(struct snd_soc_codec *codec)
{
	pr_debug("%s\n", __func__);
	Ana_Set_Reg(TOP_CLKSQ, 0x0, 0x0001);
	/* Disable CLKSQ 26MHz */
	Ana_Set_Reg(AUDDEC_ANA_CON8, 0x0002, 0x0002);
	/* disable AUDGLB */
	Ana_Set_Reg(TOP_CKPDN_CON0_SET, 0x3800, 0x3800);
	/* Turn off AUDNCP_CLKDIV engine clock,Turn off AUD 26M */
	Ana_Set_Reg(AUDDEC_ANA_CON0, 0xe000, 0xe000);
	/* Disable HeadphoneL/HeadphoneR/voice short circuit protection */
	Ana_Set_Reg(AUDENC_ANA_CON9, 0x0000, 0x0010);
	/* power off mic bias1 */
	Ana_Set_Reg(AFE_PMIC_NEWIF_CFG2, 0x8000, 0x8000);
	/* Reverse the PMIC clock*/
}

void InitCodecDefault(void)
{
	pr_debug("%s\n", __func__);
	mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP1] = 3;
	mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP2] = 3;
	mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP3] = 3;
	mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP4] = 3;
	mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTR] = 8;
	mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTR] = 8;

	mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC1] =
	    AUDIO_ANALOG_AUDIOANALOG_INPUT_PREAMP;
	mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC2] =
	    AUDIO_ANALOG_AUDIOANALOG_INPUT_PREAMP;
	mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC3] =
	    AUDIO_ANALOG_AUDIOANALOG_INPUT_PREAMP;
	mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC4] =
	    AUDIO_ANALOG_AUDIOANALOG_INPUT_PREAMP;
}

static void InitGlobalVarDefault(void)
{
	mCodec_data = NULL;
	mAudio_Vow_Analog_Func_Enable = false;
	mAudio_Vow_Digital_Func_Enable = false;
	mIsVOWOn = false;
	mAdc_Power_Mode = 0;
	mInitCodec = false;
	mAnaSuspend = false;
	audck_buf_Count = 0;
	ClsqCount = 0;
	TopCkCount = 0;
	NvRegCount = 0;

#ifdef CONFIG_MTK_SPEAKER
	mSpeaker_Ocflag = false;
#endif

}

static int mt6331_codec_probe(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	pr_debug("%s()\n", __func__);

	if (mInitCodec == true)
		return 0;

	pin_extspkamp = pin_extspkamp_2 = pin_vowclk = pin_audmiso = pin_rcvspkswitch = 0;
	pin_mode_extspkamp = pin_mode_extspkamp_2 = pin_mode_vowclk = pin_mode_audmiso =
	    pin_mode_rcvspkswitch = 0;

	snd_soc_dapm_new_controls(dapm, mt6331_dapm_widgets, ARRAY_SIZE(mt6331_dapm_widgets));
	snd_soc_dapm_add_routes(dapm, mtk_audio_map, ARRAY_SIZE(mtk_audio_map));

	/* add codec controls */
	snd_soc_add_codec_controls(codec, mt6331_snd_controls, ARRAY_SIZE(mt6331_snd_controls));
	snd_soc_add_codec_controls(codec, mt6331_UL_Codec_controls,
				   ARRAY_SIZE(mt6331_UL_Codec_controls));
	snd_soc_add_codec_controls(codec, mt6331_Voice_Switch, ARRAY_SIZE(mt6331_Voice_Switch));
	snd_soc_add_codec_controls(codec, mt6331_pmic_Test_controls,
				   ARRAY_SIZE(mt6331_pmic_Test_controls));

#ifdef CONFIG_MTK_SPEAKER
	snd_soc_add_codec_controls(codec, mt6331_snd_Speaker_controls,
				   ARRAY_SIZE(mt6331_snd_Speaker_controls));
#endif

	snd_soc_add_codec_controls(codec, Audio_snd_auxadc_controls,
				   ARRAY_SIZE(Audio_snd_auxadc_controls));

	/* here to set  private data */
	mCodec_data = kzalloc(sizeof(mt6331_Codec_Data_Priv), GFP_KERNEL);
	if (!mCodec_data) {
		/*pr_warn("Failed to allocate private data\n");*/
		return -ENOMEM;
	}
	snd_soc_codec_set_drvdata(codec, mCodec_data);

	memset((void *)mCodec_data, 0, sizeof(mt6331_Codec_Data_Priv));
	mt6331_codec_init_reg(codec);
	InitCodecDefault();
	mInitCodec = true;

	return 0;
}

static int mt6331_codec_remove(struct snd_soc_codec *codec)
{
	pr_debug("%s()\n", __func__);
	return 0;
}

static unsigned int mt6331_read(struct snd_soc_codec *codec, unsigned int reg)
{
	pr_debug("mt6331_read reg = 0x%x", reg);
	Ana_Get_Reg(reg);
	return 0;
}

static int mt6331_write(struct snd_soc_codec *codec, unsigned int reg, unsigned int value)
{
	pr_debug("mt6331_write reg = 0x%x  value= 0x%x\n", reg, value);
	Ana_Set_Reg(reg, value, 0xffffffff);
	return 0;
}


static struct snd_soc_codec_driver soc_mtk_codec = {
	.probe = mt6331_codec_probe,
	.remove = mt6331_codec_remove,

	.read = mt6331_read,
	.write = mt6331_write,


	/* use add control to replace */
	/* .controls = mt6331_snd_controls, */
	/* .num_controls = ARRAY_SIZE(mt6331_snd_controls), */

	.dapm_widgets = mt6331_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mt6331_dapm_widgets),
	.dapm_routes = mtk_audio_map,
	.num_dapm_routes = ARRAY_SIZE(mtk_audio_map),

};

static int mtk_mt6331_codec_dev_probe(struct platform_device *pdev)
{
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(64);

	if (pdev->dev.dma_mask == NULL)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;


	if (pdev->dev.of_node)
		dev_set_name(&pdev->dev, "%s", MT_SOC_CODEC_NAME);


	pr_debug("%s: dev name %s\n", __func__, dev_name(&pdev->dev));
	return snd_soc_register_codec(&pdev->dev,
				      &soc_mtk_codec, mtk_6331_dai_codecs,
				      ARRAY_SIZE(mtk_6331_dai_codecs));
}

static int mtk_mt6331_codec_dev_remove(struct platform_device *pdev)
{
	pr_debug("%s:\n", __func__);

	snd_soc_unregister_codec(&pdev->dev);
	return 0;

}

#ifdef CONFIG_OF
static const struct of_device_id mt_soc_codec_63xx_of_ids[] = {
	{.compatible = "mediatek,mt_soc_codec_63xx",},
	{}
};
#endif

static struct platform_driver mtk_codec_6331_driver = {
	.driver = {
		   .name = MT_SOC_CODEC_NAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = mt_soc_codec_63xx_of_ids,
#endif
		   },
	.probe = mtk_mt6331_codec_dev_probe,
	.remove = mtk_mt6331_codec_dev_remove,
};

#ifndef CONFIG_OF
static struct platform_device *soc_mtk_codec6331_dev;
#endif

static int __init mtk_mt6331_codec_init(void)
{
	pr_debug("%s:\n", __func__);
#ifndef CONFIG_OF
	int ret = 0;

	soc_mtk_codec6331_dev = platform_device_alloc(MT_SOC_CODEC_NAME, -1);

	if (!soc_mtk_codec6331_dev)
		return -ENOMEM;


	ret = platform_device_add(soc_mtk_codec6331_dev);
	if (ret != 0) {
		platform_device_put(soc_mtk_codec6331_dev);
		return ret;
	}
#endif
	InitGlobalVarDefault();

	return platform_driver_register(&mtk_codec_6331_driver);
}

module_init(mtk_mt6331_codec_init);

static void __exit mtk_mt6331_codec_exit(void)
{
	pr_debug("%s:\n", __func__);

	platform_driver_unregister(&mtk_codec_6331_driver);
}

module_exit(mtk_mt6331_codec_exit);

/* Module information */
MODULE_DESCRIPTION("MTK  codec driver");
MODULE_LICENSE("GPL v2");
