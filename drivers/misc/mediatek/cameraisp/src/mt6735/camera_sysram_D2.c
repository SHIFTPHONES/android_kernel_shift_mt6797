/*
* Copyright (C) 2011-2014 MediaTek Inc.
*
* This program is free software: you can redistribute it and/or modify it under the terms of the
* GNU General Public License version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
/* #include <asm/io.h> */
#include <linux/proc_fs.h>	/* proc file use */
#include <linux/slab.h>		/* kmalloc/kfree in kernel 3.10 */
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
/* #include <linux/xlog.h> */
/* #include <mach/mt_reg_base.h> */
#include <mach/mt_clkmgr.h>
#include <mt-plat/sync_write.h>
#include "inc/camera_sysram_D2.h"
/* #include <mach/mt_reg_base.h> */
#include "inc/camera_sysram_imp_D2.h"

#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif


#ifdef CONFIG_OF
#include <linux/of_platform.h>	/* for device tree */
#include <linux/of_irq.h>	/* for device tree */
#include <linux/of_address.h>	/* for device tree */
#endif


#define ISP_VALID_REG_RANGE  0x10000
#define IMGSYS_BASE_ADDR     0x15000000

/* ----------------------------------------------------------------------------- */
static SYSRAM_STRUCT Sysram;
/* ------------------------------------------------------------------------------ */
static void SYSRAM_GetTime(MUINT64 *pUS64, MUINT32 *pSec, MUINT32 *pUSec)
{
	ktime_t Time;
	MUINT64 TimeSec;
	/*  */
	Time = ktime_get();	/* ns */
	TimeSec = Time.tv64;
	do_div(TimeSec, 1000);
	/*  */
	*pUS64 = TimeSec;
	*pUSec = do_div(TimeSec, 1000000);
	*pSec = (MUINT64) TimeSec;
}

/* ------------------------------------------------------------------------------ */
static void SYSRAM_CheckClock(void)
{
	/* LOG_MSG("AllocatedTbl(0x%08X),EnableClk(%d)",Sysram.AllocatedTbl,Sysram.EnableClk); */
	if (Sysram.AllocatedTbl) {
		if (!(Sysram.EnableClk)) {
			Sysram.EnableClk = true;
			/*
			   LOG_MSG("AllocatedTbl(0x%08lX),EnableClk(%d)",
			   Sysram.AllocatedTbl,
			   Sysram.EnableClk);
			 */
		}
	} else {
		if (Sysram.EnableClk) {
			Sysram.EnableClk = false;
			/*
			   LOG_MSG("AllocatedTbl(0x%08lX),EnableClk(%d)",
			   Sysram.AllocatedTbl,
			   Sysram.EnableClk);
			 */
		}
	}
}

/* ------------------------------------------------------------------------------ */
static void SYSRAM_DumpResMgr(void)
{
	unsigned int u4Idx = 0;

	LOG_MSG("TotalUserCount(%ld),AllocatedTbl(0x%lX)",
		Sysram.TotalUserCount, Sysram.AllocatedTbl);
	/*  */
	for (u4Idx = 0; u4Idx < SYSRAM_USER_AMOUNT; u4Idx++) {
		if (0 < Sysram.AllocatedSize[u4Idx]) {
			SYSRAM_USER_STRUCT * const pUserInfo = &Sysram.UserInfo[u4Idx];
			LOG_MSG("[id:%u][%s][size:0x%lX][pid:%d][tgid:%d][%s][%5lu.%06lu]",
				u4Idx,
				SysramUserName[u4Idx],
				Sysram.AllocatedSize[u4Idx],
				pUserInfo->pid,
				pUserInfo->tgid,
				pUserInfo->ProcName, pUserInfo->TimeS, pUserInfo->TimeUS);
		}
	}
	LOG_MSG("End");
}

/* ------------------------------------------------------------------------------ */
static inline MBOOL SYSRAM_IsBadOwner(SYSRAM_USER_ENUM const User)
{
	if (SYSRAM_USER_AMOUNT <= User || User < 0) {
		return MTRUE;
	} else {
		return MFALSE;
	}
}

/* ------------------------------------------------------------------------------ */
static void SYSRAM_SetUserTaskInfo(SYSRAM_USER_ENUM const User)
{
	if (!SYSRAM_IsBadOwner(User)) {
		SYSRAM_USER_STRUCT * const pUserInfo = &Sysram.UserInfo[User];
		/*  */
		pUserInfo->pid = current->pid;
		pUserInfo->tgid = current->tgid;
		memcpy(pUserInfo->ProcName, current->comm, sizeof(pUserInfo->ProcName));
		/*  */
		SYSRAM_GetTime(&(pUserInfo->Time64), &(pUserInfo->TimeS), &(pUserInfo->TimeUS));
	}
}

/* ------------------------------------------------------------------------------ */
static void SYSRAM_ResetUserTaskInfo(SYSRAM_USER_ENUM const User)
{
	if (!SYSRAM_IsBadOwner(User)) {
		SYSRAM_USER_STRUCT * const pUserInfo = &Sysram.UserInfo[User];
		memset(pUserInfo, 0, sizeof(*pUserInfo));
	}
}

/* ------------------------------------------------------------------------------ */
static inline void SYSRAM_SpinLock(void)
{
	spin_lock(&Sysram.SpinLock);
}

/* ------------------------------------------------------------------------------ */
static inline void SYSRAM_SpinUnlock(void)
{
	spin_unlock(&Sysram.SpinLock);
}

/* ------------------------------------------------------------------------------ */
static inline MBOOL SYSRAM_UserIsLocked(SYSRAM_USER_ENUM const User)
{
	if ((1 << User) & Sysram.AllocatedTbl) {
		return MTRUE;
	} else {
		return MFALSE;
	}
}

/* ------------------------------------------------------------------------------ */
static inline MBOOL SYSRAM_UserIsUnlocked(SYSRAM_USER_ENUM const User)
{
	if (SYSRAM_UserIsLocked(User) == 0) {
		return MTRUE;
	} else {
		return MFALSE;
	}
}

/* ------------------------------------------------------------------------------ */
static void SYSRAM_LockUser(SYSRAM_USER_ENUM const User, MUINT32 const Size)
{
	if (SYSRAM_UserIsLocked(User)) {
		return;
	}
	/*  */
	Sysram.TotalUserCount++;
	Sysram.AllocatedTbl |= (1 << User);
	Sysram.AllocatedSize[User] = Size;
	SYSRAM_SetUserTaskInfo(User);
	/* Debug Log. */
	if ((1 << User) & SysramLogUserMask) {
		SYSRAM_USER_STRUCT * const pUserInfo = &Sysram.UserInfo[User];
		LOG_MSG("[%s][%lu bytes]OK,Time(%lu.%06lu)",
			SysramUserName[User],
			Sysram.AllocatedSize[User], pUserInfo->TimeS, pUserInfo->TimeUS);
	}
}

/* ------------------------------------------------------------------------------ */
static void SYSRAM_UnlockUser(SYSRAM_USER_ENUM const User)
{
	if (SYSRAM_UserIsUnlocked(User)) {
		return;
	}
	/* Debug Log. */
	if ((1 << User) & SysramLogUserMask) {
		SYSRAM_USER_STRUCT * const pUserInfo = &Sysram.UserInfo[User];
		MUINT32 Sec, USec;
		MUINT64 Time64 = 0;

		SYSRAM_GetTime(&Time64, &Sec, &USec);
		/*  */
		LOG_MSG("[%s][%lu bytes]Time(%lu.%06lu - %lu.%06lu)(%lu.%06lu)",
			SysramUserName[User],
			Sysram.AllocatedSize[User],
			pUserInfo->TimeS,
			pUserInfo->TimeUS,
			Sec,
			USec,
			((MUINT32) (Time64 - pUserInfo->Time64)) / 1000,
			((MUINT32) (Time64 - pUserInfo->Time64)) % 1000);
	}
	/*  */
	if (Sysram.TotalUserCount > 0) {
		Sysram.TotalUserCount--;
	}
	Sysram.AllocatedTbl &= (~(1 << User));
	Sysram.AllocatedSize[User] = 0;
	SYSRAM_ResetUserTaskInfo(User);
}

/* ------------------------------------------------------------------------------ */
static void SYSRAM_DumpLayout(void)
{
	MUINT32 Index = 0;
	SYSRAM_MEM_NODE_STRUCT *pCurrNode = NULL;
	/*  */
	LOG_DMP("[SYSRAM_DumpLayout]\n");
	LOG_DMP("AllocatedTbl = 0x%08lX\n", Sysram.AllocatedTbl);
	LOG_DMP("=========================================\n");
	for (Index = 0; Index < SYSRAM_MEM_BANK_AMOUNT; Index++) {
		LOG_DMP("\n [Mem Pool %ld] (IndexTbl, UserCount)=(%lX, %ld)\n",
			Index,
			SysramMemPoolInfo[Index].IndexTbl, SysramMemPoolInfo[Index].UserCount);
		LOG_DMP
		    ("[Locked Time] [Owner   Offset   Size  Index pCurrent pPrevious pNext]  [pid tgid] [Proc Name / Owner Name]\n");
		pCurrNode = &SysramMemPoolInfo[Index].pMemNode[0];
		while (NULL != pCurrNode) {
			SYSRAM_USER_ENUM const User = pCurrNode->User;
			if (SYSRAM_IsBadOwner(User)) {
				LOG_DMP("------------ --------"
					" %2d\t0x%05lX 0x%05lX  %ld    %p %p\t%p\n",
					pCurrNode->User,
					pCurrNode->Offset,
					pCurrNode->Length,
					pCurrNode->Index,
					pCurrNode, pCurrNode->pPrev, pCurrNode->pNext);
			} else {
				SYSRAM_USER_STRUCT * const pUserInfo = &Sysram.UserInfo[User];
				LOG_DMP("%5lu.%06lu"
					" %2d\t0x%05lX 0x%05lX  %ld    %p %p\t%p"
					"  %-4d %-4d \"%s\" / \"%s\"\n",
					pUserInfo->TimeS,
					pUserInfo->TimeUS,
					User,
					pCurrNode->Offset,
					pCurrNode->Length,
					pCurrNode->Index,
					pCurrNode,
					pCurrNode->pPrev,
					pCurrNode->pNext,
					pUserInfo->pid,
					pUserInfo->tgid, pUserInfo->ProcName, SysramUserName[User]);
			}
			pCurrNode = pCurrNode->pNext;
		};
	}
	LOG_DMP("\n");
	SYSRAM_DumpResMgr();
}

/* ------------------------------------------------------------------------------ */
static SYSRAM_MEM_NODE_STRUCT *SYSRAM_AllocNode(SYSRAM_MEM_POOL_STRUCT * const pMemPoolInfo)
{
	SYSRAM_MEM_NODE_STRUCT *pNode = NULL;
	MUINT32 Index = 0;
	/*  */
	for (Index = 0; Index < pMemPoolInfo->UserAmount; Index += 1) {
		if ((pMemPoolInfo->IndexTbl) & (1 << Index)) {
			pMemPoolInfo->IndexTbl &= (~(1 << Index));
			/* A free node is found. */
			pNode = &pMemPoolInfo->pMemNode[Index];
			pNode->User = SYSRAM_USER_NONE;
			pNode->Offset = 0;
			pNode->Length = 0;
			pNode->pPrev = NULL;
			pNode->pNext = NULL;
			pNode->Index = Index;
			break;
		}
	}
	/* Shouldn't happen. */
	if (!pNode) {
		LOG_ERR("returns NULL - pMemPoolInfo->IndexTbl(%lX)", pMemPoolInfo->IndexTbl);
	}
	return pNode;
}

/* ------------------------------------------------------------------------------ */
static void SYSRAM_FreeNode(SYSRAM_MEM_POOL_STRUCT * const pMemPoolInfo,
			    SYSRAM_MEM_NODE_STRUCT * const pNode)
{
	pMemPoolInfo->IndexTbl |= (1 << pNode->Index);
	pNode->User = SYSRAM_USER_NONE;
	pNode->Offset = 0;
	pNode->Length = 0;
	pNode->pPrev = NULL;
	pNode->pNext = NULL;
	pNode->Index = 0;
}

/* ------------------------------------------------------------------------------ */
static MBOOL SYSRAM_IsLegalSizeToAlloc(SYSRAM_MEM_BANK_ENUM const MemBankNo,
				       SYSRAM_USER_ENUM const User, MUINT32 const Size)
{
	MUINT32 MaxSize = 0;
	/* (1) Check the memory pool. */
	switch (MemBankNo) {
	case SYSRAM_MEM_BANK_BAD:
	case SYSRAM_MEM_BANK_AMOUNT:
		{
			/* Illegal Memory Pool: return "illegal" */
			/* Shouldn't happen. */
			goto EXIT;
		}
	default:
		{
			break;
		}
	}
	/* (2) */
	/* Here we use the dynamic memory pools. */
	MaxSize = SysramUserSize[User];
	/*  */
EXIT:
	if (MaxSize < Size) {
		LOG_ERR("[User:%s]requested size(0x%lX) > max size(0x%lX)",
			SysramUserName[User], Size, MaxSize);
		SYSRAM_DumpLayout();
		return MFALSE;
	}
	return MTRUE;
}

/* ------------------------------------------------------------------------------ */
/*
Alignment should be 2^N, 4/8/2048 bytes alignment only
First fit algorithm
*/
static MUINT32 SYSRAM_AllocUserPhy(SYSRAM_USER_ENUM const User,
				   MUINT32 const Size,
				   MUINT32 const Alignment, SYSRAM_MEM_BANK_ENUM const MemBankNo)
{
	SYSRAM_MEM_NODE_STRUCT *pSplitNode = NULL;
	SYSRAM_MEM_NODE_STRUCT *pCurrNode = NULL;
	MUINT32 AlingnedAddr = 0;
	MUINT32 ActualSize = 0;
	/*  */
	SYSRAM_MEM_POOL_STRUCT * const pMemPoolInfo = SYSRAM_GetMemPoolInfo(MemBankNo);
	if (!pMemPoolInfo) {
		return 0;
	}
	/*  */
	pCurrNode = &pMemPoolInfo->pMemNode[0];
	for (; pCurrNode && pCurrNode->Offset < pMemPoolInfo->Size; pCurrNode = pCurrNode->pNext) {
		if (SYSRAM_USER_NONE == pCurrNode->User) {
			/* Free space */
			AlingnedAddr = (pCurrNode->Offset + Alignment - 1) & (~(Alignment - 1));
			ActualSize = Size + AlingnedAddr - pCurrNode->Offset;
			if (ActualSize <= pCurrNode->Length) {
				/* Hit!! Split into 2 */
				/* pSplitNode pointers to the next available (free) node. */
				pSplitNode = SYSRAM_AllocNode(pMemPoolInfo);
				pSplitNode->Offset = pCurrNode->Offset + ActualSize;
				pSplitNode->Length = pCurrNode->Length - ActualSize;
				pSplitNode->pPrev = pCurrNode;
				pSplitNode->pNext = pCurrNode->pNext;
				/*  */
				pCurrNode->User = User;
				pCurrNode->Length = ActualSize;
				pCurrNode->pNext = pSplitNode;
				/*  */
				if (NULL != pSplitNode->pNext) {
					pSplitNode->pNext->pPrev = pSplitNode;
				}
				/*  */
				pMemPoolInfo->UserCount++;
				break;
			}
			/* Not hit */
			ActualSize = 0;
		}
	};
	/*  */
	return ActualSize ? (AlingnedAddr + pMemPoolInfo->Addr) : 0;
}

/* ------------------------------------------------------------------------------ */
static MBOOL SYSRAM_FreeUserPhy(SYSRAM_USER_ENUM const User, SYSRAM_MEM_BANK_ENUM const MemBankNo)
{
	MBOOL Ret = MFALSE;
	SYSRAM_MEM_NODE_STRUCT *pPrevOrNextNode = NULL;
	SYSRAM_MEM_NODE_STRUCT *pCurrNode = NULL;
	SYSRAM_MEM_NODE_STRUCT *pTempNode = NULL;

	SYSRAM_MEM_POOL_STRUCT * const pMemPoolInfo = SYSRAM_GetMemPoolInfo(MemBankNo);
	/*  */
	if (!pMemPoolInfo) {
		LOG_ERR("pMemPoolInfo==NULL,User(%d),MemBankNo(%d)", User, MemBankNo);
		return MFALSE;
	}
	/*  */
	pCurrNode = &pMemPoolInfo->pMemNode[0];
	for (; pCurrNode; pCurrNode = pCurrNode->pNext) {
		if (User == pCurrNode->User) {
			Ret = MTRUE;	/* user is found. */
			if (pMemPoolInfo->UserCount > 0)
				pMemPoolInfo->UserCount--;

			pCurrNode->User = SYSRAM_USER_NONE;
			if (NULL != pCurrNode->pPrev) {
				pPrevOrNextNode = pCurrNode->pPrev;
				/*  */
				if (SYSRAM_USER_NONE == pPrevOrNextNode->User) {
					/* Merge previous: prev(o) + curr(x) */
					pTempNode = pCurrNode;
					pCurrNode = pPrevOrNextNode;
					pCurrNode->Length += pTempNode->Length;
					pCurrNode->pNext = pTempNode->pNext;
					if (NULL != pTempNode->pNext) {
						pTempNode->pNext->pPrev = pCurrNode;
					}
					SYSRAM_FreeNode(pMemPoolInfo, pTempNode);
				}
			}

			if (NULL != pCurrNode->pNext) {
				pPrevOrNextNode = pCurrNode->pNext;
				/*  */
				if (SYSRAM_USER_NONE == pPrevOrNextNode->User) {
					/* Merge next: curr(o) + next(x) */
					pTempNode = pPrevOrNextNode;
					pCurrNode->Length += pTempNode->Length;
					pCurrNode->pNext = pTempNode->pNext;
					if (NULL != pCurrNode->pNext) {
						pCurrNode->pNext->pPrev = pCurrNode;
					}
					SYSRAM_FreeNode(pMemPoolInfo, pTempNode);
				}
			}
			break;
		}
	}
	/*  */
	return Ret;
}

/* ------------------------------------------------------------------------------ */
static MUINT32 SYSRAM_AllocUser(SYSRAM_USER_ENUM const User, MUINT32 Size, MUINT32 const Alignment)
{
	MUINT32 Addr = 0;

	SYSRAM_MEM_BANK_ENUM const MemBankNo = SYSRAM_GetMemBankNo(User);
	/*  */
	if (SYSRAM_IsBadOwner(User)) {
		LOG_ERR("User(%d) out of range(%d)", User, SYSRAM_USER_AMOUNT);
		return 0;
	}
	/*  */
	if (!SYSRAM_IsLegalSizeToAlloc(MemBankNo, User, Size)) {
		return 0;
	}
	/*  */
	switch (MemBankNo) {
	case SYSRAM_MEM_BANK_BAD:
	case SYSRAM_MEM_BANK_AMOUNT:
		{
			/* Do nothing. */
			break;
		}
	default:
		{
			Addr = SYSRAM_AllocUserPhy(User, Size, Alignment, MemBankNo);
			break;
		}
	}
	/*  */
	if (0 < Addr) {
		SYSRAM_LockUser(User, Size);
	}
	/*  */
	return Addr;
}

/* ------------------------------------------------------------------------------ */
static void SYSRAM_FreeUser(SYSRAM_USER_ENUM const User)
{
	SYSRAM_MEM_BANK_ENUM const MemBankNo = SYSRAM_GetMemBankNo(User);
	/*  */
	switch (MemBankNo) {
	case SYSRAM_MEM_BANK_BAD:
	case SYSRAM_MEM_BANK_AMOUNT:
		{
			/* Do nothing. */
			break;
		}
	default:
		{
			if (SYSRAM_FreeUserPhy(User, MemBankNo)) {
				SYSRAM_UnlockUser(User);
			} else {
				LOG_ERR("Cannot free User(%d)", User);
				SYSRAM_DumpLayout();
			}
			break;
		}
	}
}

/* ------------------------------------------------------------------------------ */
static MUINT32 SYSRAM_MsToJiffies(MUINT32 TimeMs)
{
	return (TimeMs * HZ + 512) >> 10;
}

/* ------------------------------------------------------------------------------ */
static MUINT32 SYSRAM_TryAllocUser(SYSRAM_USER_ENUM const User,
				   MUINT32 const Size, MUINT32 const Alignment)
{
	MUINT32 Addr = 0;
	/*  */
	SYSRAM_SpinLock();
	/*  */
	if (SYSRAM_UserIsLocked(User)) {
		SYSRAM_SpinUnlock();
		LOG_ERR("[User:%s]has been already allocated!", SysramUserName[User]);
		return 0;
	}
	/*  */
	Addr = SYSRAM_AllocUser(User, Size, Alignment);
	if (Addr != 0) {
		SYSRAM_CheckClock();
	}
	SYSRAM_SpinUnlock();
	/*  */
	return Addr;
}

/* ------------------------------------------------------------------------------ */
static MUINT32 SYSRAM_IOC_Alloc(SYSRAM_USER_ENUM const User,
				MUINT32 const Size, MUINT32 Alignment, MUINT32 const TimeoutMS)
{
	MUINT32 Addr = 0;
	MINT32 TimeOut = 0;
	/*  */
	if (SYSRAM_IsBadOwner(User)) {
		LOG_ERR("User(%d) out of range(%d)", User, SYSRAM_USER_AMOUNT);
		return 0;
	}
	/*  */
	if (0 == Size) {
		LOG_ERR("[User:%s]allocates 0 size!", SysramUserName[User]);
		return 0;
	}
	/*  */
	Addr = SYSRAM_TryAllocUser(User, Size, Alignment);
	if (0 != Addr		/* success */
	    || 0 == TimeoutMS	/* failure without a timeout specified */
	    ) {
		goto EXIT;
	}
	/*  */
	TimeOut = wait_event_interruptible_timeout(Sysram.WaitQueueHead,
						   0 != (Addr =
							 SYSRAM_TryAllocUser(User, Size,
									     Alignment)),
						   SYSRAM_MsToJiffies(TimeoutMS));
	/*  */
	if (0 == TimeOut && 0 == Addr) {
		LOG_ERR("[User:%s]allocate timeout", SysramUserName[User]);
	}
	/*  */
EXIT:
	if (0 == Addr) {	/* Failure */
		LOG_ERR("[User:%s]fails to allocate.Size(%lu),Alignment(%lu),TimeoutMS(%lu)",
			SysramUserName[User], Size, Alignment, TimeoutMS);
		SYSRAM_DumpLayout();
	} else {		/* Success */
		if ((1 << User) & SysramLogUserMask) {
			LOG_MSG("[User:%s]%lu bytes OK", SysramUserName[User], Size);
		}
	}
	/*  */
	return Addr;
}

/* ------------------------------------------------------------------------------ */
static void SYSRAM_IOC_Free(SYSRAM_USER_ENUM User)
{
	if (SYSRAM_IsBadOwner(User)) {
		LOG_ERR("User(%d) out of range(%d)", User, SYSRAM_USER_AMOUNT);
		return;
	}
	/*  */
	SYSRAM_SpinLock();
	SYSRAM_FreeUser(User);
	wake_up_interruptible(&Sysram.WaitQueueHead);
	SYSRAM_CheckClock();
	SYSRAM_SpinUnlock();
	/*  */
	if ((1 << User) & SysramLogUserMask) {
		LOG_MSG("[User:%s]Done", SysramUserName[User]);
	}
}

/* ------------------------------------------------------------------------------ */
static int SYSRAM_Open(struct inode *pInode, struct file *pFile)
{
	int Ret = 0;
	MUINT32 Sec = 0, USec = 0;
	MUINT64 Time64 = 0;
	SYSRAM_PROC_STRUCT *pProc;
	/*  */
	SYSRAM_GetTime(&Time64, &Sec, &USec);
	/*  */
	LOG_MSG("Cur:Name(%s),pid(%d),tgid(%d),Time(%ld.%06ld)",
		current->comm, current->pid, current->tgid, Sec, USec);
	/*  */
	SYSRAM_SpinLock();
	/*  */
	pFile->private_data = kmalloc(sizeof(SYSRAM_PROC_STRUCT), GFP_ATOMIC);
	if (pFile->private_data == NULL) {
		Ret = -ENOMEM;
	} else {
		pProc = (SYSRAM_PROC_STRUCT *) (pFile->private_data);
		pProc->Pid = 0;
		pProc->Tgid = 0;
		strncpy(pProc->ProcName, SYSRAM_PROC_NAME, sizeof(SYSRAM_PROC_NAME));
		pProc->Table = 0;
		pProc->Time64 = Time64;
		pProc->TimeS = Sec;
		pProc->TimeUS = USec;
	}
	/*  */
	SYSRAM_SpinUnlock();
	/*  */
	if (Ret == (-ENOMEM)) {
		LOG_ERR("No enough memory");
		/*
		   LOG_ERR("Cur:Name(%s),pid(%d),tgid(%d),Time(%ld.%06ld)",
		   current->comm,
		   current->pid,
		   current->tgid,
		   Sec,
		   USec);
		 */
	}
	/*  */
	return Ret;
}

/* ------------------------------------------------------------------------------ */
static int SYSRAM_Release(struct inode *pInode, struct file *pFile)
{
	MUINT32 Index = 0;
	MUINT32 Sec = 0, USec = 0;
	MUINT64 Time64 = 0;
	SYSRAM_PROC_STRUCT *pProc;
	/*  */
	SYSRAM_GetTime(&Time64, &Sec, &USec);
	/*  */
	LOG_MSG("Cur:Name(%s),pid(%d),tgid(%d),Time(%ld.%06ld)",
		current->comm, current->pid, current->tgid, Sec, USec);
	/*  */
	if (pFile->private_data != NULL) {
		pProc = (SYSRAM_PROC_STRUCT *) (pFile->private_data);
		/*  */
		if (pProc->Pid != 0 || pProc->Tgid != 0 || pProc->Table != 0) {
			/*  */
			LOG_WRN("Proc:Name(%s),pid(%d),tgid(%d),Table(0x%08lX),Time(%ld.%06ld)",
				pProc->ProcName,
				pProc->Pid, pProc->Tgid, pProc->Table, pProc->TimeS, pProc->TimeUS);
			/*  */
			if (pProc->Table) {
				LOG_WRN("Force to release");
				/*
				   LOG_WRN("Proc:Name(%s),pid(%d),tgid(%d),Table(0x%08lX),Time(%ld.%06ld)",
				   pProc->ProcName,
				   pProc->Pid,
				   pProc->Tgid,
				   pProc->Table,
				   pProc->TimeS,
				   pProc->TimeUS);
				 */
				SYSRAM_DumpLayout();
				/*  */
				for (Index = 0; Index < SYSRAM_USER_AMOUNT; Index++) {
					if (pProc->Table & (1 << Index)) {
						SYSRAM_IOC_Free((SYSRAM_USER_ENUM) Index);
					}
				}
			}
		}
		/*  */
		kfree(pFile->private_data);
		pFile->private_data = NULL;
	} else {
		LOG_WRN("private_data is NULL");
		/*
		   LOG_WRN("Cur:Name(%s),pid(%d),tgid(%d),Time(%ld.%06ld)",
		   current->comm,
		   current->pid,
		   current->tgid,
		   Sec,
		   USec);
		 */
	}
	/*  */
	return 0;
}

/* ------------------------------------------------------------------------------ */
static int SYSRAM_Flush(struct file *pFile, fl_owner_t Id)
{
	MUINT32 Index = 0;
	MUINT32 Sec = 0, USec = 0;
	MUINT64 Time64 = 0;
	SYSRAM_PROC_STRUCT *pProc;
	/*  */
	SYSRAM_GetTime(&Time64, &Sec, &USec);
	/*  */
	LOG_MSG("Cur:Name(%s),pid(%d),tgid(%d),Time(%ld.%06ld)",
		current->comm, current->pid, current->tgid, Sec, USec);
	/*  */
	if (pFile->private_data != NULL) {
		pProc = (SYSRAM_PROC_STRUCT *) pFile->private_data;
		/*  */
		if (pProc->Pid != 0 || pProc->Tgid != 0 || pProc->Table != 0) {
			/*  */
			LOG_WRN("Proc:Name(%s),pid(%d),tgid(%d),Table(0x%08lX),Time(%ld.%06ld)",
				pProc->ProcName,
				pProc->Pid, pProc->Tgid, pProc->Table, pProc->TimeS, pProc->TimeUS);
			/*  */
			if (pProc->Tgid == 0 && pProc->Table != 0) {
				LOG_ERR("No Tgid info");
				/*
				   LOG_ERR("Cur:Name(%s),pid(%d),tgid(%d),Time(%ld.%06ld)",
				   current->comm,
				   current->pid,
				   current->tgid,
				   Sec,
				   USec);
				   LOG_ERR("Proc:Name(%s),pid(%d),tgid(%d),Table(0x%08lX),Time(%ld.%06ld)",
				   pProc->ProcName,
				   pProc->Pid,
				   pProc->Tgid,
				   pProc->Table,
				   pProc->TimeS,
				   pProc->TimeUS);
				 */
			} else
			    if ((pProc->Tgid == current->tgid) ||
				((pProc->Tgid != current->tgid)
				 && (strcmp(current->comm, "binder") == 0))) {
				if (pProc->Table) {
					LOG_WRN("Force to release");
					/*
					   LOG_WRN("Cur:Name(%s),pid(%d),tgid(%d),Time(%ld.%06ld)",
					   current->comm,
					   current->pid,
					   current->tgid,
					   Sec,
					   USec);
					 */
					SYSRAM_DumpLayout();
					/*  */
					for (Index = 0; Index < SYSRAM_USER_AMOUNT; Index++) {
					/**/	if (pProc->Table & (1 << Index)) {
							SYSRAM_IOC_Free((SYSRAM_USER_ENUM) Index);
						}
					}
					/*  */
					pProc->Table = 0;
				}
			}

		}
	} else {
		LOG_WRN("private_data is NULL");
		/*
		   LOG_WRN("Cur:Name(%s),pid(%d),tgid(%d),Time(%ld.%06ld)",
		   current->comm,
		   current->pid,
		   current->tgid,
		   Sec,
		   USec);
		 */
	}
	/*  */
	return 0;
}

/* ------------------------------------------------------------------------------ */
static int SYSRAM_mmap(struct file *pFile, struct vm_area_struct *pVma)
{
	/* LOG_MSG(""); */
	long length = 0;
	MUINT32 pfn = 0x0;

	pVma->vm_page_prot = pgprot_noncached(pVma->vm_page_prot);
	length = (long)(pVma->vm_end - pVma->vm_start);
	pfn = pVma->vm_pgoff << PAGE_SHIFT;
	LOG_WRN
	    ("pVma->vm_pgoff(0x%lx),phy(0x%lx),pVmapVma->vm_start(0x%lx),pVma->vm_end(0x%lx),length(0x%lx)",
	     pVma->vm_pgoff, pVma->vm_pgoff << PAGE_SHIFT, pVma->vm_start, pVma->vm_end, length);
	if ((length > ISP_VALID_REG_RANGE) || (pfn < IMGSYS_BASE_ADDR)
	    || (pfn > (IMGSYS_BASE_ADDR + ISP_VALID_REG_RANGE))) {
		LOG_ERR
		    ("mmap range error : vm_start(0x%lx),vm_end(0x%lx),length(0x%lx),pfn(0x%lx)!",
		     pVma->vm_start, pVma->vm_end, length, pfn);
		return -EAGAIN;
	}
	if (remap_pfn_range(pVma,
			    pVma->vm_start,
			    pVma->vm_pgoff, pVma->vm_end - pVma->vm_start, pVma->vm_page_prot)) {
		LOG_ERR("fail");
		return -EAGAIN;
	}
	return 0;
}

/* ------------------------------------------------------------------------------ */
static long SYSRAM_Ioctl(struct file *pFile, unsigned int Cmd, unsigned long Param)
{
	MINT32 Ret = 0;
	MUINT32 Sec = 0, USec = 0;
	MUINT64 Time64 = 0;
	SYSRAM_PROC_STRUCT *pProc = (SYSRAM_PROC_STRUCT *) pFile->private_data;
	SYSRAM_ALLOC_STRUCT Alloc;
	SYSRAM_USER_ENUM User;
	/*  */
	SYSRAM_GetTime(&Time64, &Sec, &USec);
	/*
	   LOG_MSG("Cur:Name(%s),pid(%d),tgid(%d),Time(%ld.%06ld)",
	   current->comm,
	   current->pid,
	   current->tgid,
	   Sec,
	   USec);
	 */
	if (pFile->private_data == NULL) {
		LOG_WRN("private_data is NULL.");
		Ret = -EFAULT;
		goto EXIT;
	}
	/*  */
	switch (Cmd) {
	case SYSRAM_ALLOC:
		{
			if (copy_from_user(&Alloc, (void *)Param, sizeof(SYSRAM_ALLOC_STRUCT)) == 0) {
				if (SYSRAM_IsBadOwner(Alloc.User)) {
					LOG_ERR("User(%d) out of range(%d)", Alloc.User,
						SYSRAM_USER_AMOUNT);
					Ret = -EFAULT;
					goto EXIT;
				}
				/*  */
				Alloc.Addr = SYSRAM_IOC_Alloc(Alloc.User,
							      Alloc.Size,
							      Alloc.Alignment, Alloc.TimeoutMS);
				if (Alloc.Addr != 0) {
					SYSRAM_SpinLock();
					pProc->Table |= (1 << Alloc.User);
					if (pProc->Tgid == 0) {
						pProc->Pid = current->pid;
						pProc->Tgid = current->tgid;
						strncpy(pProc->ProcName, current->comm, sizeof(pProc->ProcName));
						SYSRAM_SpinUnlock();
					} else {
						SYSRAM_SpinUnlock();
					/**/	if (pProc->Tgid != current->tgid) {
							LOG_ERR("Tgid is inconsistent");
							Ret = -EFAULT;
						}
					}
				} else {
					LOG_MSG("[christ test] SYSRAM_ALLOC E.6 ");
					Ret = -EFAULT;
				}
				/*  */
				if (copy_to_user
				    ((void *)Param, &Alloc, sizeof(SYSRAM_ALLOC_STRUCT))) {
					LOG_MSG("[christ test] SYSRAM_ALLOC E.7 ");
					LOG_ERR("copy to user failed");
					Ret = -EFAULT;
				}
			} else {
				LOG_ERR("copy_from_user fail");
				Ret = -EFAULT;
			}
			break;
		}
		/*  */
	case SYSRAM_FREE:
		{
			if (copy_from_user(&User, (void *)Param, sizeof(SYSRAM_USER_ENUM)) == 0) {
				if (SYSRAM_IsBadOwner(User)) {
					LOG_ERR("User(%d) out of range(%d)", User,
						SYSRAM_USER_AMOUNT);
					Ret = -EFAULT;
					goto EXIT;
				}
				/*  */
				SYSRAM_SpinLock();
				if ((pProc->Table) & (1 << User)) {
					SYSRAM_SpinUnlock();
					SYSRAM_IOC_Free(User);
					SYSRAM_SpinLock();
					/*  */
					pProc->Table &= (~(1 << User));
					if (pProc->Table == 0) {
						pProc->Pid = 0;
						pProc->Tgid = 0;
						strcpy(pProc->ProcName, SYSRAM_PROC_NAME);
					}
					SYSRAM_SpinUnlock();
				} else {
					SYSRAM_SpinUnlock();
					LOG_WRN("Freeing unallocated buffer user(%d)", User);
					Ret = -EFAULT;
				}
			} else {
				LOG_ERR("copy_from_user fail");
				Ret = -EFAULT;
			}
			break;
		}
	case SYSRAM_DUMP:
		{
			SYSRAM_DumpLayout();
			break;
		}
	default:
		{
			LOG_WRN("No such command");
			Ret = -EINVAL;
			break;
		}
	}
	/*  */
EXIT:
	if (Ret != 0) {
		LOG_ERR("Fail");
		LOG_ERR("Cur:Name(%s),pid(%d),tgid(%d),Time(%ld.%06ld)",
			current->comm, current->pid, current->tgid, Sec, USec);
		if (pFile->private_data != NULL) {
			LOG_ERR("Proc:Name(%s),pid(%d),tgid(%d),Table(0x%08lX),Time(%ld.%06ld)",
				pProc->ProcName, pProc->Pid, pProc->Tgid, pProc->Table, Sec, USec);
		}
	}
	/*  */
	return Ret;
}

/* ------------------------------------------------------------------------------ */

#ifdef CONFIG_COMPAT

static int compat_get_sysram_alloc_data(compat_SYSRAM_ALLOC_STRUCT __user *data32,
					SYSRAM_ALLOC_STRUCT __user *data)
{
	compat_uint_t tmp;
	int err;

	err = get_user(tmp, &data32->Alignment);
	err |= put_user(tmp, &data->Alignment);
	err |= get_user(tmp, &data32->Size);
	err |= put_user(tmp, &data->Size);
	err |= get_user(tmp, &data32->User);
	err |= put_user(tmp, &data->User);
	err |= get_user(tmp, &data32->Addr);
	err |= put_user(tmp, &data->Addr);
	err |= get_user(tmp, &data32->TimeoutMS);
	err |= put_user(tmp, &data->TimeoutMS);
	return err;
}


static int compat_put_sysram_alloc_data(compat_SYSRAM_ALLOC_STRUCT __user *data32,
					SYSRAM_ALLOC_STRUCT __user *data)
{
	compat_uint_t tmp;
	int err;

	err = get_user(tmp, &data->Alignment);
	err |= put_user(tmp, &data32->Alignment);
	err |= get_user(tmp, &data->Size);
	err |= put_user(tmp, &data32->Size);
	err |= get_user(tmp, &data->User);
	err |= put_user(tmp, &data32->User);
	err |= get_user(tmp, &data->Addr);
	err |= put_user(tmp, &data32->Addr);
	err |= get_user(tmp, &data->TimeoutMS);
	err |= put_user(tmp, &data32->TimeoutMS);
	return err;
}

#if 0
static int compat_get_sysram_userenum_data(compat_SYSRAM_USER_ENUM __user *data32,
					   SYSRAM_USER_ENUM __user *data)
{
	compat_uint_t tmp;
	int err;

	err = get_user(tmp, &data32);
	err |= put_user(tmp, &data);
	return err;
}


static int compat_put_sysram_userenum_data(compat_SYSRAM_USER_ENUM __user *data32,
					   SYSRAM_USER_ENUM __user *data)
{
	compat_uint_t tmp;
	int err;

	err = get_user(tmp, &data);
	err |= put_user(tmp, &data32);
	return err;
}
#endif

static long SYSRAM_ioctl_compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret;

	LOG_MSG
	    ("SYSRAM_ioctl_compat E, cmd(0x%x), SYSRAM_ALLOC/SYSRAM_FREE/SYSRAM_DUMP/compat_SYSRAM_ALLOC/compat_SYSRAM_FREE(0x%x/0x%x/0x%x/0x%x/0x%x)",
	     cmd, (unsigned int)SYSRAM_ALLOC, (unsigned int)SYSRAM_FREE, (unsigned int)SYSRAM_DUMP,
	     (unsigned int)COMPAT_SYSRAM_ALLOC, (unsigned int)COMPAT_SYSRAM_FREE);

	if (!filp->f_op || !filp->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_SYSRAM_ALLOC:	/*  */
		{
			compat_SYSRAM_ALLOC_STRUCT __user *data32;
			SYSRAM_ALLOC_STRUCT __user *data;
			int err;

			data32 = compat_ptr(arg);
			data = compat_alloc_user_space(sizeof(*data));
			if (data == NULL)
				return -EFAULT;
			err = compat_get_sysram_alloc_data(data32, data);
			if (err) {
				LOG_MSG("compat_get_sysram_alloc_data error!!!\n");
				return err;
			}
			ret = filp->f_op->unlocked_ioctl(filp, SYSRAM_ALLOC, (unsigned long)data);
			err = compat_put_sysram_alloc_data(data32, data);
			if (err) {
				LOG_MSG("compat_put_sysram_alloc_data error!!!\n");
				return err;
			}
			return ret;
		}
	case COMPAT_SYSRAM_FREE:	/*  */
	case SYSRAM_DUMP:	/* no arg */
		return filp->f_op->unlocked_ioctl(filp, cmd, arg);
	default:
		return -ENOIOCTLCMD;
	}
	LOG_MSG("SYSRAM_ioctl_compat X");
}
#endif

/* ------------------------------------------------------------------------------ */
#if 0
#ifdef CONFIG_OF
struct cam_isp_device {
	void __iomem *regs[ISP_CAM_BASEADDR_NUM];
	struct device *dev;
	int irq[ISP_CAM_IRQ_IDX_NUM];
};

static struct cam_isp_device *cam_isp_devs;
static int nr_camisp_devs;
#endif
#endif


static const struct file_operations SysramFileOper = {

	.owner = THIS_MODULE,
	.open = SYSRAM_Open,
	.release = SYSRAM_Release,
	.flush = SYSRAM_Flush,
	.unlocked_ioctl = SYSRAM_Ioctl,
	.mmap = SYSRAM_mmap,
#ifdef CONFIG_COMPAT
	.compat_ioctl = SYSRAM_ioctl_compat,
#endif
};

/* ------------------------------------------------------------------------------ */
static inline int SYSRAM_RegCharDrv(void)
{
	LOG_MSG("E");
	if (alloc_chrdev_region(&Sysram.DevNo, 0, 1, SYSRAM_DEV_NAME)) {
		LOG_ERR("allocate device no failed");
		return -EAGAIN;
	}
	/* allocate driver */
	Sysram.pCharDrv = cdev_alloc();
	if (Sysram.pCharDrv == NULL) {
		unregister_chrdev_region(Sysram.DevNo, 1);
		LOG_ERR("allocate mem for kobject failed");
		return -ENOMEM;
	}
	/* Attatch file operation. */
	cdev_init(Sysram.pCharDrv, &SysramFileOper);
	Sysram.pCharDrv->owner = THIS_MODULE;
	/* Add to system */
	if (cdev_add(Sysram.pCharDrv, Sysram.DevNo, 1)) {
		LOG_ERR("Attatch file operation failed");
		unregister_chrdev_region(Sysram.DevNo, 1);
		return -EAGAIN;
	}
	LOG_MSG("X");
	return 0;
}

/* ------------------------------------------------------------------------------ */
static inline void SYSRAM_UnregCharDrv(void)
{
	LOG_MSG("E");
	/* Release char driver */
	cdev_del(Sysram.pCharDrv);
	unregister_chrdev_region(Sysram.DevNo, 1);
	LOG_MSG("X");
}

/* ------------------------------------------------------------------------------ */
static int SYSRAM_Probe(struct platform_device *pDev)
{
	MINT32 Ret = 0;
	MUINT32 Index = 0;
	struct device *sysram_device = NULL;
	/*  */
	LOG_MSG("SYSRAM_Probe E");
	/* register char driver */
	/* allocate major no */
	if (SYSRAM_RegCharDrv()) {
		LOG_ERR("register char failed");
		return -EAGAIN;
	}

	Sysram.pClass = class_create(THIS_MODULE, "SysramDrv");
	if (IS_ERR(Sysram.pClass)) {
		Ret = PTR_ERR(Sysram.pClass);
		LOG_ERR("Unable to create class, err(%ld)", Ret);
		return Ret;
	}
	sysram_device = device_create(Sysram.pClass, NULL, Sysram.DevNo, NULL, SYSRAM_DEV_NAME);
	/* Initialize variables */
	spin_lock_init(&Sysram.SpinLock);
	Sysram.TotalUserCount = 0;
	Sysram.AllocatedTbl = 0;
	memset(Sysram.AllocatedSize, 0, sizeof(Sysram.AllocatedSize));
	memset(Sysram.UserInfo, 0, sizeof(Sysram.UserInfo));
	init_waitqueue_head(&Sysram.WaitQueueHead);
	Sysram.EnableClk = MFALSE;
	/*  */
	for (Index = 0; Index < SYSRAM_MEM_BANK_AMOUNT; Index++) {
		SysramMemPoolInfo[Index].pMemNode[0].User = SYSRAM_USER_NONE;
		SysramMemPoolInfo[Index].pMemNode[0].Offset = 0;
		SysramMemPoolInfo[Index].pMemNode[0].Length = SysramMemPoolInfo[Index].Size;
		SysramMemPoolInfo[Index].pMemNode[0].Index = 0;
		SysramMemPoolInfo[Index].pMemNode[0].pNext = NULL;
		SysramMemPoolInfo[Index].pMemNode[0].pPrev = NULL;
		SysramMemPoolInfo[Index].IndexTbl = (~0x1);
		SysramMemPoolInfo[Index].UserCount = 0;
	}
	/*  */
	for (Index = 0; Index < SYSRAM_USER_AMOUNT; Index++) {
		Sysram.AllocatedSize[Index] = 0;
	}
	Sysram.DebugFlag = SYSRAM_DEBUG_DEFAULT;
	/*  */
	LOG_MSG("SYSRAM_Probe X");
	return Ret;
}

/* ------------------------------------------------------------------------------ */
static int SYSRAM_Remove(struct platform_device *pDev)
{
	LOG_MSG("E");
	/* unregister char driver. */
	SYSRAM_UnregCharDrv();
	/*  */
	device_destroy(Sysram.pClass, Sysram.DevNo);
	class_destroy(Sysram.pClass);
	/*  */
	LOG_MSG("X");
	/*  */
	return 0;
}

/* ------------------------------------------------------------------------------ */
static int SYSRAM_Suspend(struct platform_device *pDev, pm_message_t Mesg)
{
	LOG_MSG("");
	return 0;
}

/* ------------------------------------------------------------------------------ */
static int SYSRAM_Resume(struct platform_device *pDev)
{
	LOG_MSG("");
	return 0;
}

/* ------------------------------------------------------------------------------ */
#ifdef CONFIG_OF		/* force to use define in dts file to hook device */
static const struct of_device_id isp_of_ids[] = {
	{.compatible = "mediatek,ISP_SYSR",},
	{}
};
#endif



static struct platform_driver SysramPlatformDriver = {

	.probe = SYSRAM_Probe,
	.remove = SYSRAM_Remove,
	.suspend = SYSRAM_Suspend,
	.resume = SYSRAM_Resume,
	.driver = {

		   .name = SYSRAM_DEV_NAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = isp_of_ids,
#endif
		   }
};

/* ------------------------------------------------------------------------------ */
static ssize_t SYSRAM_DumpLayoutToProc(struct file *pPage,
				char __user *pBuffer, size_t Count, loff_t *Off)
{
	/*Legacy Function, For Debug Only*/
	return 0;
}

/* ------------------------------------------------------------------------------ */
static ssize_t SYSRAM_ReadFlag(struct file *pPage,
				char __user *pBuffer, size_t Count, loff_t *Off)
{
	char tempStr[256];
	char tempStr2[256] = {'\0'};
	long length = 0;
	static int finished;

	if (finished) {
		finished = 0;
		return 0;
	}

	finished = 1;

	if (Count < 256) {
		LOG_ERR("BufferSize(%d) less than 256.", (int)Count);
		return 0;
	}

	length += sprintf(tempStr, "Sysram.DebugFlag = 0x%08lX\r\n", Sysram.DebugFlag);

	strncat(tempStr2, tempStr, length);

	if (copy_to_user(pBuffer, tempStr2, length))
		return -EFAULT;

	return length;/*end of reading*/
}

/* ------------------------------------------------------------------------------ */
static ssize_t SYSRAM_WriteFlag(struct file *pFile,
			    const char __user *pBuffer, size_t Count, loff_t *p_off)
{
	char acBuf[32];
	MUINT32 u4CopySize = 0;
	MUINT32 u4SysramDbgFlag = 0;
	/*  */
	u4CopySize = (Count < (sizeof(acBuf) - 1)) ? Count : (sizeof(acBuf) - 1);
	if (copy_from_user(acBuf, pBuffer, u4CopySize)) {
		return 0;
	}
	acBuf[u4CopySize] = '\0';
	if (3 == sscanf(acBuf, "%lx", &u4SysramDbgFlag)) {
		Sysram.DebugFlag = u4SysramDbgFlag;
	}
	return ((ssize_t)Count);
}

/*******************************************************************************
*
********************************************************************************/
static const struct file_operations fsysram_proc_fops = {
	.read = SYSRAM_DumpLayoutToProc,
	.write = NULL,
};

static const struct file_operations fsysram_flag_proc_fops = {
	.read = SYSRAM_ReadFlag,
	.write = SYSRAM_WriteFlag,
};

/* ----------------------------------------------------------------------------- */
static int __init SYSRAM_Init(void)
{
	/* struct proc_dir_entry *pEntry; */
	/*  */
	LOG_MSG("E");
	/*  */
	if (platform_driver_register(&SysramPlatformDriver)) {
		LOG_ERR("failed to register sysram driver");
		return -ENODEV;
	}
	/*  */
	/* linux-3.10 procfs API changed */
#if 1
	proc_create("sysram", 0, NULL, &fsysram_proc_fops);
	proc_create("sysram_flag", 0, NULL, &fsysram_flag_proc_fops);
#else
	pEntry = create_proc_entry("sysram", 0, NULL);
	if (pEntry) {
		pEntry->read_proc = SYSRAM_DumpLayoutToProc;
		pEntry->write_proc = NULL;
	} else {
		LOG_ERR("add /proc/sysram entry fail.");
	}
	/*  */
	pEntry = create_proc_entry("sysram_flag", 0, NULL);
	if (pEntry) {
		pEntry->read_proc = SYSRAM_ReadFlag;
		pEntry->write_proc = SYSRAM_WriteFlag;
	} else {
		LOG_ERR("add /proc/sysram_flag entry fail");
	}
#endif
	LOG_MSG("X");
	/*  */
	return 0;
}

/* ------------------------------------------------------------------------------ */
static void __exit SYSRAM_Exit(void)
{
	LOG_MSG("E");
	platform_driver_unregister(&SysramPlatformDriver);
	LOG_MSG("X");
}

/* ------------------------------------------------------------------------------ */
module_init(SYSRAM_Init);
module_exit(SYSRAM_Exit);
MODULE_DESCRIPTION("Camera sysram driver");
MODULE_AUTHOR("Marx <marx.chiu@mediatek.com>");
MODULE_LICENSE("GPL");
/* ------------------------------------------------------------------------------ */
