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

/* ******************************************************************************** */
/*  */
/* << LC898122 Evaluation Soft>> */
/* Program Name    : Ois.h */
/* Explanation             : LC898122 Global Declaration & ProtType Declaration */
/* Design                  : Y.Yamada */
/* History                 : First edition                                         2009.07.30 Y.Tashita */
/* ******************************************************************************** */

/* Define According To Usage */

/****************************** Define���� ******************************/
/*	USE_3WIRE_DGYRO		Digital Gyro I/F 3��Mode�p		*/
/*	USE_INVENSENSE		Invensense Digital Gyro�g�p		*/
/*	USE_IDG2020		Inv IDG-2020�g�p				*/
/*	STANDBY_MODE		Standby����g�p(���m�F)		*/
/*	GAIN_CONT			:Gain control�@			*/
/*	(disable)	DSC	:�O�rMode�g�p		*/
/*	HALLADJ_HW			Hall Calibration LSI�@		*/
/************************************************************************/

/**************** Select Gyro Sensor **************/
/* #define       USE_3WIRE_DGYRO    //for D-Gyro SPI interface */

#define		USE_INVENSENSE	/* INVENSENSE */
#ifdef USE_INVENSENSE



#define		FS_SEL		3	/* �}32.8LSB/?/s  */


#endif

/**************** Model name *****************/
#define		MN_3BSD05P1

/**************** FW version *****************/
#ifdef	MN_3BSD05P1
#define	MDL_VER			0x06
#define	FW_VER			0x03
#endif


/**************** Select Mode **************/
#define		STANDBY_MODE	/* STANDBY Mode */
#define		GAIN_CONT	/* Gain Control Mode */
/* #define               HALLADJ_HW                      // H/W Hall adjustment */
#define		PWM_BREAK	/* PWM mode select (disable zero cross) */

#ifdef	MN_3BSD05P1
#define		ACTREG_10P2OHM	/* Use 10.2ohm */
#endif

#define		DEF_SET		/* default value re-setting */
/* #define               USE_EXTCLK_ALL          // USE Ext clk for ALL */
/* #define               USE_EXTCLK_PWM          // USE Ext clk for PWM */
/* #define               USE_VH_SYNC                     // USE V/H Sync for PWM */
/* #define               PWM_CAREER_TEST         // PWM_CAREER_TEST */
#define		NEUTRAL_CENTER	/* Upper Position Current 0mA Measurement */
#define		H1COEF_CHANGER	/* H1 coef lvl chage */
#define		MONITOR_OFF	/* default Monitor output */
/* #define               MODULE_CALIBRATION              // for module maker   use float */
/* #define               AF_PWMMODE                      // AF Driver PWM mode */
/* #define               CORRECT_1DEG                    // Correct 1deg   disable 0.5deg */



/* Command Status */
#define		EXE_END		0x02	/* Execute End (Adjust OK) */
#define		EXE_HXADJ	0x06	/* Adjust NG : X Hall NG (Gain or Offset) */
#define		EXE_HYADJ	0x0A	/* Adjust NG : Y Hall NG (Gain or Offset) */
#define		EXE_LXADJ	0x12	/* Adjust NG : X Loop NG (Gain) */
#define		EXE_LYADJ	0x22	/* Adjust NG : Y Loop NG (Gain) */
#define		EXE_GXADJ	0x42	/* Adjust NG : X Gyro NG (offset) */
#define		EXE_GYADJ	0x82	/* Adjust NG : Y Gyro NG (offset) */
#define		EXE_OCADJ	0x402	/* Adjust NG : OSC Clock NG */
#define		EXE_ERR		0x99	/* Execute Error End */

/* Common Define */
#define	SUCCESS			0x00	/* Success */
#define	FAILURE			0x01	/* Failure */

#ifndef ON
#define	ON				0x01	/* ON */
#define	OFF				0x00	/* OFF */
#endif
#define	SPC				0x02	/* Special Mode */

#define	X_DIR			0x00	/* X Direction */
#define	Y_DIR			0x01	/* Y Direction */
#define	X2_DIR			0x10	/* X Direction */
#define	Y2_DIR			0x11	/* Y Direction */

#define	NOP_TIME		0.00004166F

#ifdef STANDBY_MODE
 /* Standby mode */
#define		STB1_ON		0x00	/* Standby1 ON */
#define		STB1_OFF	0x01	/* Standby1 OFF */
#define		STB2_ON		0x02	/* Standby2 ON */
#define		STB2_OFF	0x03	/* Standby2 OFF */
#define		STB3_ON		0x04	/* Standby3 ON */
#define		STB3_OFF	0x05	/* Standby3 OFF */
#define		STB4_ON		0x06	/* Standby4 ON  for Digital Gyro Read */
#define		STB4_OFF	0x07	/* Standby4 OFF */
#define		STB2_OISON	0x08	/* Standby2 ON (only OIS) */
#define		STB2_OISOFF	0x09	/* Standby2 OFF(only OIS) */
#define		STB2_AFON	0x0A	/* Standby2 ON (only AF) */
#define		STB2_AFOFF	0x0B	/* Standby2 OFF(only AF) */
#endif


/* OIS Adjust Parameter */
#define		DAHLXO_INI		0x0000
#define		DAHLXB_INI		0xE000
#define		DAHLYO_INI		0x0000
#define		DAHLYB_INI		0xE000
#define		SXGAIN_INI		0x3000
#define		SYGAIN_INI		0x3000
#define		HXOFF0Z_INI		0x0000
#define		HYOFF1Z_INI		0x0000

#ifdef ACTREG_6P5OHM		/* MTM 9.5 Actuator *************************** */
#define		BIAS_CUR_OIS	0x33	/* 2.0mA/2.0mA */
#define		AMP_GAIN_X		0x05	/* x150 */
#define		AMP_GAIN_Y		0x05	/* x150 */

/* OSC Init */
#define		OSC_INI			0x2E	/* VDD=2.8V */

/* AF Open para */
#define		RWEXD1_L_AF		0x7FFF	/*  */
#define		RWEXD2_L_AF		0x1094	/*  */
#define		RWEXD3_L_AF		0x72BA	/*  */
#define		FSTCTIME_AF		0xED	/*  */
#define		FSTMODE_AF		0x02	/*  */

 /* (0.3750114X^3+0.5937681X)*(0.3750114X^3+0.5937681X) 6.5ohm */
#define		A3_IEXP3		0x3EC0017F
#define		A1_IEXP1		0x3F180130

#endif
#ifdef ACTREG_10P2OHM		/* MTM 10.2 Actuator *************************** */
#define		BIAS_CUR_OIS	0x33	/* 2.0mA/2.0mA */
#define		AMP_GAIN_X		0x05	/* x150 */
#define		AMP_GAIN_Y		0x05	/* x150 */

/* OSC Init */
#define		OSC_INI			0x2E	/* VDD=2.8V */

/* AF Open para */
#define		RWEXD1_L_AF		0x7FFF	/*  */
#define		RWEXD2_L_AF		0x75FE	/*  */
#define		RWEXD3_L_AF		0x7F32	/*  */
#define		FSTCTIME_AF		0xF1	/*  */
#define		FSTMODE_AF		0x00	/*  */

 /* (0.3750114X^3+0.55X)*(0.3750114X^3+0.55X) 10.2ohm */
#define		A3_IEXP3		0x3EC0017F
#define		A1_IEXP1		0x3F0CCCCD

#endif
#ifdef ACTREG_15OHM		/* TDK 10.5 Actuator *************************** */
#define		BIAS_CUR_OIS	0x22	/* 1.0mA/1.0mA */
#define		AMP_GAIN_X		0x04	/* x100 */
#define		AMP_GAIN_Y		0x04	/* x100 */

/* OSC Init */
#define		OSC_INI			0x2E	/* VDD=2.8V */

/* AF Open para */
#define		RWEXD1_L_AF		0x7FFF	/*  */
#define		RWEXD2_L_AF		0x5A00	/*  */
#define		RWEXD3_L_AF		0x7000	/*  */
#define		FSTCTIME_AF		0x5F	/*  */
#define		FSTMODE_AF		0x00	/*  */

 /* (0.4531388X^3+0.4531388X)*(0.4531388X^3+0.4531388X) 15ohm */
#define		A3_IEXP3		0x3EE801CF
#define		A1_IEXP1		0x3EE801CF

#endif

/* AF adjust parameter */
#define		DAHLZB_INI		0x9000
#define		DAHLZO_INI		0x0000
#define		BIAS_CUR_AF		0x00	/* 0.25mA */
#define		AMP_GAIN_AF		0x00	/* x6 */

/* Digital Gyro offset Initial value */
#define		DGYRO_OFST_XH	0x00
#define		DGYRO_OFST_XL	0x00
#define		DGYRO_OFST_YH	0x00
#define		DGYRO_OFST_YL	0x00

#define		SXGAIN_LOP		0x3000
#define		SYGAIN_LOP		0x3000

#define		TCODEH_ADJ		0x0000

#define		GYRLMT1H		0x3DCCCCCD	/* 0.1F */

#ifdef	CORRECT_1DEG
#define		GYRLMT3_S1		0x3F19999A	/* 0.60F */
#define		GYRLMT3_S2		0x3F19999A	/* 0.60F */

#define		GYRLMT4_S1		0x40400000	/* 3.0F */
#define		GYRLMT4_S2		0x40400000	/* 3.0F */

#define		GYRA12_HGH		0x40000000	/* 2.00F */
#define		GYRA12_MID		0x3F800000	/* 1.0F */
#define		GYRA34_HGH		0x3F000000	/* 0.5F */
#define		GYRA34_MID		0x3DCCCCCD	/* 0.1F */

#define		GYRB12_HGH		0x3E4CCCCD	/* 0.20F */
#define		GYRB12_MID		0x3CA3D70A	/* 0.02F */
#define		GYRB34_HGH		0x3CA3D70A	/* 0.02F */
#define		GYRB34_MID		0x3C23D70A	/* 0.001F */

#else
#define		GYRLMT3_S1		0x3ECCCCCD	/* 0.40F */
#define		GYRLMT3_S2		0x3ECCCCCD	/* 0.40F */

#define		GYRLMT4_S1		0x40000000	/* 2.0F */
#define		GYRLMT4_S2		0x40000000	/* 2.0F */

#define		GYRA12_HGH		0x3FC00000	/* 1.50F */
#define		GYRA12_MID		0x3F800000	/* 1.0F */
#define		GYRA34_HGH		0x3F000000	/* 0.5F */
#define		GYRA34_MID		0x3DCCCCCD	/* 0.1F */

#define		GYRB12_HGH		0x3E4CCCCD	/* 0.20F */
#define		GYRB12_MID		0x3CA3D70A	/* 0.02F */
#define		GYRB34_HGH		0x3CA3D70A	/* 0.02F */
#define		GYRB34_MID		0x3C23D70A	/* 0.001F */

#endif


/* #define               OPTCEN_X                0x0000 */
/* #define               OPTCEN_Y                0x0000 */

#ifdef USE_INVENSENSE
#ifdef	MN_3BSD05P1
#define		SXQ_INI			0x3F800000
#define		SYQ_INI			0xBF800000

#define		GXGAIN_INI		0xBF147AE1
#define		GYGAIN_INI		0xBF147AE1

#define		GYROX_INI		0x45
#define		GYROY_INI		0x43

#define		GXHY_GYHX		1
#endif
#endif


/* Optical Center & Gyro Gain for Mode */
#define	VAL_SET				0x00	/* Setting mode */
#define	VAL_FIX				0x01	/* Fix Set value */
#define	VAL_SPC				0x02	/* Special mode */


struct STFILREG {
	unsigned short UsRegAdd;
	unsigned char UcRegDat;
};				/* Register Data Table */

struct STFILRAM {
	unsigned short UsRamAdd;
	unsigned long UlRamDat;
};				/* Filter Coefficient Table */

struct STCMDTBL {
	unsigned short Cmd;
	unsigned int UiCmdStf;
	void (*UcCmdPtr)(void);
};

/*** caution [little-endian] ***/

/* Word Data Union */
union UnWrdVal {
	unsigned short UsWrdVal;
	unsigned char UcWrkVal[2];
	struct {
		unsigned char UcLowVal;
		unsigned char UcHigVal;
	} StWrdVal;
};

union UnDwdVal {
	unsigned long UlDwdVal;
	unsigned short UsDwdVal[2];
	struct {
		unsigned short UsLowVal;
		unsigned short UsHigVal;
	} StDwdVal;
	struct {
		unsigned char UcRamVa0;
		unsigned char UcRamVa1;
		unsigned char UcRamVa2;
		unsigned char UcRamVa3;
	} StCdwVal;
};

/* Float Data Union */
union UnFltVal {
	float SfFltVal;
	unsigned long UlLngVal;
	unsigned short UsDwdVal[2];
	struct {
		unsigned short UsLowVal;
		unsigned short UsHigVal;
	} StFltVal;
};


struct stAdjPar {
	struct {
		unsigned char UcAdjPhs;	/* Hall Adjust Phase */

		unsigned short UsHlxCna;	/* Hall Center Value after Hall Adjust */
		unsigned short UsHlxMax;	/* Hall Max Value */
		unsigned short UsHlxMxa;	/* Hall Max Value after Hall Adjust */
		unsigned short UsHlxMin;	/* Hall Min Value */
		unsigned short UsHlxMna;	/* Hall Min Value after Hall Adjust */
		unsigned short UsHlxGan;	/* Hall Gain Value */
		unsigned short UsHlxOff;	/* Hall Offset Value */
		unsigned short UsAdxOff;	/* Hall A/D Offset Value */
		unsigned short UsHlxCen;	/* Hall Center Value */

		unsigned short UsHlyCna;	/* Hall Center Value after Hall Adjust */
		unsigned short UsHlyMax;	/* Hall Max Value */
		unsigned short UsHlyMxa;	/* Hall Max Value after Hall Adjust */
		unsigned short UsHlyMin;	/* Hall Min Value */
		unsigned short UsHlyMna;	/* Hall Min Value after Hall Adjust */
		unsigned short UsHlyGan;	/* Hall Gain Value */
		unsigned short UsHlyOff;	/* Hall Offset Value */
		unsigned short UsAdyOff;	/* Hall A/D Offset Value */
		unsigned short UsHlyCen;	/* Hall Center Value */
	} StHalAdj;

	struct {
		unsigned short UsLxgVal;	/* Loop Gain X */
		unsigned short UsLygVal;	/* Loop Gain Y */
		unsigned short UsLxgSts;	/* Loop Gain X Status */
		unsigned short UsLygSts;	/* Loop Gain Y Status */
	} StLopGan;

	struct {
		unsigned short UsGxoVal;	/* Gyro A/D Offset X */
		unsigned short UsGyoVal;	/* Gyro A/D Offset Y */
		unsigned short UsGxoSts;	/* Gyro Offset X Status */
		unsigned short UsGyoSts;	/* Gyro Offset Y Status */
	} StGvcOff;

	unsigned char UcOscVal;	/* OSC value */

};

extern struct stAdjPar StAdjPar;	/* Execute Command Parameter */

extern unsigned char UcOscAdjFlg;	/* For Measure trigger */
#define	MEASSTR		0x01
#define	MEASCNT		0x08
#define	MEASFIX		0x80

extern unsigned short UsCntXof;	/* OPTICAL Center Xvalue */
extern unsigned short UsCntYof;	/* OPTICAL Center Yvalue */

extern unsigned char UcPwmMod;	/* PWM MODE */
#define		PWMMOD_CVL	0x00	/* CVL PWM MODE */
#define		PWMMOD_PWM	0x01	/* PWM MODE */

#define		INIT_PWMMODE	PWMMOD_CVL	/* initial output mode */

extern unsigned char UcCvrCod;	/* CverCode */
#define	CVER122		0x93	/* LC898122 */
#define	CVER122A	0xA1	/* LC898122A */


/* Prottype Declation */
extern void IniSet(void);	/* Initial Top Function */
extern void IniSetAf(void);	/* Initial Top Function */

extern void ClrGyr(unsigned short, unsigned char);	/* Clear Gyro RAM */
#define CLR_FRAM0			0x01
#define CLR_FRAM1			0x02
#define CLR_ALL_RAM		0x03
extern void BsyWit(unsigned short, unsigned char);	/* Busy Wait Function */
/* extern void WitTim_LC898122AF( unsigned short ) ;             // Wait */
extern void MemClr(unsigned char *, unsigned short);	/* Memory Clear Function */
extern void GyOutSignal(void);	/* Slect Gyro Output signal Function */
extern void GyOutSignalCont(void);	/* Slect Gyro Output Continuos Function */
#ifdef STANDBY_MODE
extern void AccWit(unsigned char);	/* Acc Wait Function */
extern void SelectGySleep(unsigned char);	/* Select Gyro Mode Function */
#endif
#ifdef	GAIN_CONT
extern void AutoGainControlSw(unsigned char);	/* Auto Gain Control Sw */
#endif
extern void DrvSw(unsigned char UcDrvSw);	/* Driver Mode setting function */
extern void AfDrvSw(unsigned char UcDrvSw);	/* AF Driver Mode setting function */
extern void RamAccFixMod(unsigned char);	/* Ram Access Fix Mode setting function */
extern void IniPtMovMod(unsigned char);	/* Pan/Tilt parameter setting by mode function */
extern void ChkCvr(void);	/* Check Function */

extern void SrvCon(unsigned char, unsigned char);	/* Servo ON/OFF */
extern unsigned short TneRun(void);	/* Hall System Auto Adjustment Function */
extern unsigned char RtnCen(unsigned char);	/* Return to Center Function */
extern void OisEna(void);	/* OIS Enable Function */
extern void OisEnaLin(void);	/* OIS Enable Function for Line adjustment */
extern void TimPro(void);	/* Timer Interrupt Process Function */
extern void S2cPro(unsigned char);	/* S2 Command Process Function */

#ifdef	MN_3BSD05P1
#define		DIFIL_S2		0x3F7FFE00
#endif
extern void SetSinWavePara(unsigned char, unsigned char);	/* Sin wave Test Function */
#define		SINEWAVE	0
#define		XHALWAVE	1
#define		YHALWAVE	2
#define		CIRCWAVE	255
extern unsigned char TneGvc(void);	/* Gyro VC Offset Adjust */

extern void SetZsp(unsigned char);	/* Set Zoom Step parameter Function */
extern void OptCen(unsigned char, unsigned short, unsigned short);	/* Set Optical Center adjusted value Function */
extern void StbOnnN(unsigned char, unsigned char);	/* Stabilizer For Servo On Function */
#ifdef	MODULE_CALIBRATION
extern unsigned char LopGan(unsigned char);	/* Loop Gain Adjust */
#endif
#ifdef STANDBY_MODE
extern void SetStandby(unsigned char);	/* Standby control      */
#endif
#ifdef	MODULE_CALIBRATION
extern unsigned short OscAdj(void);	/* OSC clock adjustment */
#endif

#ifdef	HALLADJ_HW
#ifdef	MODULE_CALIBRATION
extern unsigned char LoopGainAdj(unsigned char);
#endif
extern unsigned char BiasOffsetAdj(unsigned char, unsigned char);
#endif
extern void GyrGan(unsigned char, unsigned long, unsigned long);	/* Set Gyro Gain Function */
extern void SetPanTiltMode(unsigned char);	/* Pan_Tilt control Function */
#ifndef	HALLADJ_HW
extern unsigned long TnePtp(unsigned char, unsigned char);	/* Get Hall Peak to Peak Values */
#ifdef	MN_3BSD05P1
#define		HALL_H_VAL	0x3F800000	/* 1.0 */
#endif
extern unsigned char TneCen(unsigned char, union UnDwdVal);	/* Tuning Hall Center */
#define		PTP_BEFORE		0
#define		PTP_AFTER		1
#endif
#ifdef GAIN_CONT
extern unsigned char TriSts(void);	/* Read Status of Tripod mode Function */
#endif
extern unsigned char DrvPwmSw(unsigned char);	/* Select Driver mode Function */
#define		Mlnp		0	/* Linear PWM */
#define		Mpwm		1	/* PWM */
#ifdef	NEUTRAL_CENTER		/* Gyro VC Offset Adjust */
extern unsigned char TneHvc(void);	/* Hall VC Offset Adjust */
#endif				/* NEUTRAL_CENTER */
extern void SetGcf(unsigned char);	/* Set DI filter coefficient Function */
extern unsigned long UlH1Coefval;	/* H1 coefficient value */
#ifdef H1COEF_CHANGER
extern unsigned char UcH1LvlMod;	/* H1 level coef mode */
extern void SetH1cMod(unsigned char);	/* Set H1C coefficient Level chang Function */
#define		S2MODE		0x40
#define		ACTMODE		0x80
#define		MOVMODE		0xFF
#endif
extern unsigned short RdFwVr(void);
void RegWriteA_LC898122AF(unsigned short RegAddr, unsigned char RegData);
void RegReadA_LC898122AF(unsigned short RegAddr, unsigned char *RegData);
void RamWriteA_LC898122AF(unsigned short RamAddr, unsigned short RamData);
void RamReadA_LC898122AF(unsigned short RamAddr, void *ReadData);
void RamWrite32A_LC898122AF(unsigned short RamAddr, unsigned long RamData);
void RamRead32A_LC898122AF(unsigned short RamAddr, void *ReadData);
void WitTim_LC898122AF(unsigned short UsWitTim);
void LC898prtvalue(unsigned short value);

/* ************************** */
/* Local Function Prottype */
/* ************************** */
extern void IniClk(void);		/* Clock Setting */
extern void IniIop(void);		/* I/O Port Initial Setting */
extern void IniMon(void);		/* Monitor & Other Initial Setting */
extern void IniSrv(void);		/* Servo Register Initial Setting */
extern void IniGyr(void);		/* Gyro Filter Register Initial Setting */
extern void IniFil(void);		/* Gyro Filter Initial Parameter Setting */
extern void IniAdj(void);		/* Adjust Fix Value Setting */
extern void IniCmd(void);		/* Command Execute Process Initial */
extern void IniDgy(void);		/* Digital Gyro Initial Setting */
extern void IniAf(void);		/* Open AF Initial Setting */
extern void IniPtAve(void);		/* Average setting */


/* ************************** */
/* Local Function Prottype */
/* ************************** */
extern void MesFil(unsigned char);	/* Measure Filter Setting */
#ifdef	MODULE_CALIBRATION
#ifndef	HALLADJ_HW
extern void LopIni(unsigned char);	/* Loop Gain Initialize */
#endif
extern void LopPar(unsigned char);	/* Loop Gain Parameter initialize */
#ifndef	HALLADJ_HW
extern void LopSin(unsigned char, unsigned char);	/* Loop Gain Sin Wave Output */
extern unsigned char LopAdj(unsigned char);	/* Loop Gain Adjust */
extern void LopMes(void);		/* Loop Gain Measure */
#endif
#endif
#ifndef	HALLADJ_HW
extern unsigned long GinMes(unsigned char);	/* Measure Result Getting */
#endif
extern  void GyrCon(unsigned char);	/* Gyro Filter Control */
extern short GenMes(unsigned short, unsigned char);	/* General Measure */
#ifndef	HALLADJ_HW
/* unsigned long        TnePtp( unsigned char, unsigned char ) ;        // Get Hall Peak to Peak Values */
/* unsigned char        TneCen( unsigned char, union UnDwdVal ) ;       // Tuning Hall Center */
extern unsigned long TneOff(union UnDwdVal, unsigned char);	/* Hall Offset Tuning */
extern unsigned long TneBia(union UnDwdVal, unsigned char);	/* Hall Bias Tuning */
#endif

extern void StbOnn(void);		/* Servo ON Slope mode */

extern void SetSineWave(unsigned char, unsigned char);
extern void StartSineWave(void);
extern void StopSineWave(void);

extern void SetMeasFil(unsigned char);
extern void ClrMeasFil(void);

										/* Read Fw Version Function */
