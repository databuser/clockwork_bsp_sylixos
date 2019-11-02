/*
 * usb_keypad_ops.c
 *
 *  Created on: Oct 25, 2019
 *      Author: databus
 */
#define  __SYLIXOS_KERNEL
#include <SylixOS.h>
#include <module.h>
#include <keyboard.h>

LW_DEV_HDR          __GdevHdr;
LW_HANDLE           keypad_queue;
LW_SEL_WAKEUPLIST   select_list;

static LONG __devOpen(PLW_DEV_HDR pdevhdrHdr,
                      PCHAR       pcName,
                      INT         iFlag,
                      INT         iMode)
{
    API_MsgQueueClear(keypad_queue);

    return 0;
}

static INT __devClose(PLW_FD_ENTRY  pFdEntry)
{
    API_MsgQueueClear(keypad_queue);

    return 0;
}


static INT __devIoctl (PLW_FD_ENTRY  pFdEntry, INT  iCmd, LONG lArg)
{
    struct stat         *pstat;
    PLW_SEL_WAKEUPNODE   pselwunNode = (PLW_SEL_WAKEUPNODE)lArg;
    ULONG msg_num = 0;


    switch (iCmd) {
    case FIOFSTATGET:
        pstat = (struct stat *)lArg;
        pstat->st_dev     = (dev_t)0;
        pstat->st_ino     = (ino_t)0;
        pstat->st_mode    = 0666 | S_IFCHR;
        pstat->st_nlink   = 1;
        pstat->st_uid     = 0;
        pstat->st_gid     = 0;
        pstat->st_rdev    = 1;
        pstat->st_size    = 0;
        pstat->st_blksize = LW_CFG_VMM_PAGE_SIZE;
        pstat->st_blocks  = (blkcnt_t)(0 >> LW_CFG_VMM_PAGE_SHIFT);
        pstat->st_atime   = API_RootFsTime(LW_NULL);
        pstat->st_mtime   = API_RootFsTime(LW_NULL);
        pstat->st_ctime   = API_RootFsTime(LW_NULL);
        return 0;
    /*
     *  Sylixos下poll接口是通过select机制实现的
     */
    case FIOSELECT:
        SEL_WAKE_NODE_ADD(&select_list, pselwunNode);
        switch (pselwunNode->SELWUN_seltypType) {
        case SELREAD:
            API_MsgQueueStatus(keypad_queue,
                               NULL,
                               &msg_num,
                               NULL,
                               NULL,
                               NULL);

            if (msg_num)
                SEL_WAKE_UP(pselwunNode);

            break;
        case SELWRITE:
            break;
        case SELEXCEPT:
            break;
        }
        return 0;

    case FIOUNSELECT:
        SEL_WAKE_NODE_DELETE(&select_list, pselwunNode);
        return 0;
    default:
        return -ENOSYS;
    }

    return 0;
}

static ssize_t __devRead (PLW_FD_ENTRY  pFdEntry, PCHAR  pcBuffer, ssize_t stMaxByte)
{
    size_t pstMsgLen = 0;

    API_MsgQueueTryReceive(keypad_queue, pcBuffer, stMaxByte, &pstMsgLen);

    return pstMsgLen;
}

struct file_operations __GdevOps = {
    .fo_open  = __devOpen,
    .fo_close = __devClose,
    .fo_ioctl = __devIoctl,
    .fo_read  = __devRead,
};

INT  __devRegister(VOID)
{
    INT iDrvNum    = iosDrvInstallEx2(&__GdevOps, LW_DRV_TYPE_NEW_1);

    keypad_queue = API_MsgQueueCreate("dpad_q", (ULONG)50,
                                      sizeof(struct keyboard_event_notify),
                                      LW_OPTION_OBJECT_GLOBAL, NULL);
    API_SelWakeupListInit(&select_list);

    return  (iosDevAddEx(&__GdevHdr, "/dev/input/kbd0", iDrvNum, DT_CHR));
}

INT  __devUnregister(VOID)
{
    PLW_DEV_HDR pDev;

    pDev = &__GdevHdr;

    if (pDev) {
        iosDevDelete(pDev);
        return  (iosDrvRemove(pDev->DEVHDR_usDrvNum, 0));
    } else {
        return  (PX_ERROR);
    }
}
