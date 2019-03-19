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

#ifndef _LENS_LIST_H

#define _LENS_LIST_H

#define AK7371AF_SetI2Cclient AK7371AF_SetI2Cclient_Sub
#define AK7371AF_Ioctl AK7371AF_Ioctl_Sub
#define AK7371AF_Release AK7371AF_Release_Sub
extern int AK7371AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long AK7371AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param);
extern int AK7371AF_Release(struct inode *a_pstInode, struct file *a_pstFile);

#define BU6424AF_SetI2Cclient BU6424AF_SetI2Cclient_Sub
#define BU6424AF_Ioctl BU6424AF_Ioctl_Sub
#define BU6424AF_Release BU6424AF_Release_Sub
extern int BU6424AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long BU6424AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param);
extern int BU6424AF_Release(struct inode *a_pstInode, struct file *a_pstFile);

#define BU6429AF_SetI2Cclient BU6429AF_SetI2Cclient_Sub
#define BU6429AF_Ioctl BU6429AF_Ioctl_Sub
#define BU6429AF_Release BU6429AF_Release_Sub
extern int BU6429AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long BU6429AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param);
extern int BU6429AF_Release(struct inode *a_pstInode, struct file *a_pstFile);

#define DW9714AF_SetI2Cclient DW9714AF_SetI2Cclient_Sub
#define DW9714AF_Ioctl DW9714AF_Ioctl_Sub
#define DW9714AF_Release DW9714AF_Release_Sub
extern int DW9714AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9714AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param);
extern int DW9714AF_Release(struct inode *a_pstInode, struct file *a_pstFile);

#define DW9763AF_SetI2Cclient DW9763AF_SetI2Cclient_Sub
#define DW9763AF_Ioctl DW9763AF_Ioctl_Sub
#define DW9763AF_Release DW9763AF_Release_Sub
extern int DW9763AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9763AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param);
extern int DW9763AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
//pxs_add
#define DW9715AF_SetI2Cclient DW9715AF_SetI2Cclient_Sub
#define DW9715AF_Ioctl DW9715AF_Ioctl_Sub
#define DW9715AF_Release DW9715AF_Release_Sub
extern int DW9715AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9715AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param);
extern int DW9715AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
#define DW9814AF_SetI2Cclient DW9814AF_SetI2Cclient_Sub
#define DW9814AF_Ioctl DW9814AF_Ioctl_Sub
#define DW9814AF_Release DW9814AF_Release_Sub
extern int DW9814AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9814AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param);
extern int DW9814AF_Release(struct inode *a_pstInode, struct file *a_pstFile);

#define DW9718AF_SetI2Cclient DW9718AF_SetI2Cclient_Sub
#define DW9718AF_Ioctl DW9718AF_Ioctl_Sub
#define DW9718AF_Release DW9718AF_Release_Sub
extern int DW9718AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9718AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param);
extern int DW9718AF_Release(struct inode *a_pstInode, struct file *a_pstFile);

#define FM50AF_SetI2Cclient FM50AF_SetI2Cclient_Sub
#define FM50AF_Ioctl FM50AF_Ioctl_Sub
#define FM50AF_Release FM50AF_Release_Sub
extern int FM50AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long FM50AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param);
extern int FM50AF_Release(struct inode *a_pstInode, struct file *a_pstFile);

#define LC898212AF_SetI2Cclient LC898212AF_SetI2Cclient_Sub
#define LC898212AF_Ioctl LC898212AF_Ioctl_Sub
#define LC898212AF_Release LC898212AF_Release_Sub
extern int LC898212AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long LC898212AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param);
extern int LC898212AF_Release(struct inode *a_pstInode, struct file *a_pstFile);

#define LC898214AF_SetI2Cclient LC898214AF_SetI2Cclient_Sub
#define LC898214AF_Ioctl LC898214AF_Ioctl_Sub
#define LC898214AF_Release LC898214AF_Release_Sub
extern int LC898214AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long LC898214AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param);
extern int LC898214AF_Release(struct inode *a_pstInode, struct file *a_pstFile);

#define AD5820AF_SetI2Cclient AD5820AF_SetI2Cclient_Sub
#define AD5820AF_Ioctl AD5820AF_Ioctl_Sub
#define AD5820AF_Release AD5820AF_Release_Sub
extern int AD5820AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long AD5820AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param);
extern int AD5820AF_Release(struct inode *a_pstInode, struct file *a_pstFile);

#define WV511AAF_SetI2Cclient WV511AAF_SetI2Cclient_Sub
#define WV511AAF_Ioctl WV511AAF_Ioctl_Sub
#define WV511AAF_Release WV511AAF_Release_Sub
extern int WV511AAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long WV511AAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param);
extern int WV511AAF_Release(struct inode *a_pstInode, struct file *a_pstFile);

#endif
