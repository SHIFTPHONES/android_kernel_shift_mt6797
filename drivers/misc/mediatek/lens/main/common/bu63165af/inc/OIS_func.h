/*
* Copyright (C) Rohm Co.,Ltd. All rights reserved.

* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#ifndef OIS_FUNC_H
#define OIS_FUNC_H

extern unsigned short int OIS_REQUEST;	/* OIS control register. */
/* ==> RHM_HT 2013.03.04        Change type (unsigned short int -> double) */
extern double OIS_PIXEL[2];	/* Just Only use for factory adjustment. */
/* <== RHM_HT 2013.03.04 */

extern short int CROP_X;	/* x start position for cropping */
extern short int CROP_Y;		/* y start position for cropping */
extern short int CROP_WIDTH;	/* cropping width */
extern short int CROP_HEIGHT;	/* cropping height */
extern unsigned char SLICE_LEVE;	/* slice level of bitmap binalization */

extern double DISTANCE_BETWEEN_CIRCLE;
extern double DISTANCE_TO_CIRCLE;	/* distance to the circle [mm] */
extern double D_CF;		/* Correction Factor for distance to the circle */
/* ==> RHM_HT 2013/07/10        Added new user definition variables for DC gain check */
extern unsigned short int ACT_DRV;	/* [mV]: Full Scale of OUTPUT DAC. */
extern unsigned short int FOCAL_LENGTH;	/* [um]: Focal Length 3.83mm */
extern double MAX_OIS_SENSE;	/* [um/mA]: per actuator difinition (change to absolute value) */
extern double MIN_OIS_SENSE;	/* [um/mA]: per actuator difinition (change to absolute value) */
extern unsigned short int MAX_COIL_R;	/* [ohm]: Max value of coil resistance */
extern unsigned short int MIN_COIL_R;	/* [ohm]: Min value of coil resistance */
/* <== RHM_HT 2013/07/10        Added new user definition variables */

#endif
