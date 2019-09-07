/*
 * 16550.c
 *
 *  Created on: Jun 7, 2019
 *      Author: databus
 */

#define  __SYLIXOS_KERNEL
#include "SylixOS.h"
#include "../SylixOS/config/driver/drv_cfg.h"
#include "driver/uart/uart.h"
/*********************************************************************************************************
  裁减控制
*********************************************************************************************************/
#if (LW_CFG_DEVICE_EN > 0) && (LW_CFG_SIO_DEVICE_EN > 0) && (LW_CFG_DRV_SIO_16C550 > 0)
#include "16550.h"
/*********************************************************************************************************
  16c550 register operate
*********************************************************************************************************/
#define SET_REG(psiochan, reg, value)   psiochan->setreg(psiochan, reg, (UINT8)(value))
#define GET_REG(psiochan, reg)          psiochan->getreg(psiochan, reg)
/*********************************************************************************************************
  16c550 switch pin operate (RS-485 MODE)
*********************************************************************************************************/
#define IS_SWITCH_EN(psiochan)          (psiochan->switch_en)
#define IS_Tx_HOLD_REG_EMPTY(psiochan)  ((GET_REG(psiochan, LSR) & LSR_THRE) != 0x00)

#define SEND_START(psiochan) do {   \
            psiochan->send_start(psiochan); \
            if (psiochan->delay) {  \
               psiochan->delay(psiochan);  \
            }   \
        } while (0)

#define SEND_END(psiochan) do {   \
            psiochan->send_end(psiochan); \
            if (psiochan->delay) {  \
               psiochan->delay(psiochan);  \
            }   \
        } while (0)
/*********************************************************************************************************
  internal function declare
*********************************************************************************************************/
static INT sio16c550SetBaud(SIO16C550_CHAN *psiochan, ULONG baud);
static INT sio16c550SetHwOption(SIO16C550_CHAN *psiochan, ULONG hw_option);
static INT sio16c550Ioctl(SIO16C550_CHAN *psiochan, INT cmd, LONG arg);
static INT sio16c550TxStartup(SIO16C550_CHAN *psiochan);
static INT sio16c550CallbackInstall(SIO_CHAN *pchan,
                                    INT       callbackType,
                                    INT     (*callback)(),
                                    VOID     *callbackArg);
static INT sio16c550PollInput(SIO16C550_CHAN *psiochan, CHAR *pc);
static INT sio16c550PollOutput(SIO16C550_CHAN *psiochan, CHAR c);
/*********************************************************************************************************
  16c550 driver functions
*********************************************************************************************************/
static SIO_DRV_FUNCS sio16c550_drv_funcs = {
        (INT (*)())sio16c550Ioctl,
        (INT (*)())sio16c550TxStartup,
        (INT (*)())sio16c550CallbackInstall,
        (INT (*)())sio16c550PollInput,
        (INT (*)(SIO_CHAN *, CHAR))sio16c550PollOutput,
};
/*********************************************************************************************************
** 函数名称: sio16c550Init
** 功能描述: 初始化 16C550 驱动程序
** 输　入  : psiochan      SIO CHAN
** 输　出  : ERROR or OK
*********************************************************************************************************/
INT  sio16c550Init (SIO16C550_CHAN *psiochan)
{
    /*
     * initialize the driver function pointers in the SIO_CHAN's
     */
    psiochan->pdrvFuncs = &sio16c550_drv_funcs;

    LW_SPIN_INIT(&psiochan->slock);

    psiochan->channel_mode = SIO_MODE_POLL;
    psiochan->switch_en    = 0;
    psiochan->hw_option    = (CLOCAL | CREAD | CS8);

    psiochan->mcr = MCR_OUT2;
    psiochan->lcr = 0;
    psiochan->ier = 0;

    psiochan->bdefer = LW_FALSE;

    psiochan->err_overrun = 0;
    psiochan->err_parity  = 0;
    psiochan->err_framing = 0;
    psiochan->err_break   = 0;

    psiochan->rx_trigger_level &= 0x3;

    /*
     * reset the chip
     */
    sio16c550SetBaud(psiochan, psiochan->baud);
    sio16c550SetHwOption(psiochan, psiochan->hw_option);

    return  (ERROR_NONE);
}
/*********************************************************************************************************
** 函数名称: sio16c550SetBaud
** 功能描述: 设置波特率
** 输　入  : psiochan      SIO CHAN
**           baud          波特率
** 输　出  : ERROR or OK
*********************************************************************************************************/
static INT sio16c550SetBaud (SIO16C550_CHAN *psiochan, ULONG  baud)
{
    INTREG  intreg;
    INT     divisor = (INT)((psiochan->xtal + (8 * baud)) / (16 * baud));

    if ((divisor < 1) || (divisor > 0xffff)) {
        _ErrorHandle(EIO);
        return  (PX_ERROR);
    }

    /*
     * disable interrupts during chip access
     */
    LW_SPIN_LOCK_QUICK(&psiochan->slock, &intreg);

    /*
     *  FIXME:等待宽口空闲，修改LCR_DLAB位必须在非忙碌时才可以，即UER的0位为0时
     */
    do{
        INT  iOverTime = 100000;

        iOverTime--;
        if(0 == iOverTime){
             _ErrorHandle(EIO);
             return  (PX_ERROR);
        }
    }while(GET_REG(psiochan, USR) & BUSY);

    /*
     * Enable access to the divisor latches by setting DLAB in LCR.
     */
    SET_REG(psiochan, LCR, (LCR_DLAB | psiochan->lcr));

    /*
     * Set divisor latches.
     */
    SET_REG(psiochan, DLL, divisor);
    SET_REG(psiochan, DLM, (divisor >> 8));

    /*
     * Restore line control register
     */
    SET_REG(psiochan, LCR, psiochan->lcr);

    psiochan->baud = baud;

    /*
     * some 16550 ip need reset hw here
     */
    SET_REG(psiochan, IER, 0);                                          /* disable interrupt            */

    SET_REG(psiochan, FCR,
            ((psiochan->rx_trigger_level << 6) |
             RxCLEAR | TxCLEAR | FIFO_ENABLE));

    if (psiochan->channel_mode == SIO_MODE_INT) {
        SET_REG(psiochan, IER, psiochan->ier);                          /* enable interrupt             */
    }

    LW_SPIN_UNLOCK_QUICK(&psiochan->slock, intreg);

    return  (ERROR_NONE);
}
/*********************************************************************************************************
** 函数名称: sio16c550SetHwOption
** 功能描述: 设置硬件功能选项
** 输　入  : psiochan      SIO CHAN
**           hw_option     硬件功能选项
** 输　出  : ERROR or OK
*********************************************************************************************************/
static INT sio16c550SetHwOption (SIO16C550_CHAN *psiochan, ULONG  hw_option)
{
    INTREG  intreg;

    if (!psiochan) {
        _ErrorHandle(EINVAL);
        return  (PX_ERROR);
    }

    hw_option |= HUPCL;                                                 /* need HUPCL option            */

    LW_SPIN_LOCK_QUICK(&psiochan->slock, &intreg);

    psiochan->lcr = 0;
    psiochan->mcr &= (~(MCR_RTS | MCR_DTR));                            /* clear RTS and DTR bits       */

    switch (hw_option & CSIZE) {                                        /* data bit set                 */

    case CS5:
        psiochan->lcr = CHAR_LEN_5;
        break;

    case CS6:
        psiochan->lcr = CHAR_LEN_6;
        break;

    case CS7:
        psiochan->lcr = CHAR_LEN_7;
        break;

    case CS8:
        psiochan->lcr = CHAR_LEN_8;
        break;

    default:
        psiochan->lcr = CHAR_LEN_8;
        break;
    }

    if (hw_option & STOPB) {                                            /* stop bit set                 */
        psiochan->lcr |= LCR_STB;

    } else {
        psiochan->lcr |= ONE_STOP;
    }

    switch (hw_option & (PARENB | PARODD)) {

    case PARENB | PARODD:
        psiochan->lcr |= LCR_PEN;
        psiochan->ier |= IER_ELSI;                                      /* Enable receiver line status  */
        break;

    case PARENB:
        psiochan->lcr |= (LCR_PEN | LCR_EPS);
        psiochan->ier |= IER_ELSI;                                      /* Enable receiver line status  */
        break;

    default:
        psiochan->lcr |= PARITY_NONE;
        psiochan->ier &= (~IER_ELSI);
        break;
    }

    SET_REG(psiochan, IER, 0);

    if (!(hw_option & CLOCAL)) {
        /*
         * !clocal enables hardware flow control(DTR/DSR)
         */
        psiochan->mcr |= (MCR_DTR | MCR_RTS);
        psiochan->ier &= (~TxFIFO_BIT);
        psiochan->ier |= IER_EMSI;                                      /* en modem status interrupt    */

    } else {
        psiochan->ier &= (~IER_EMSI);                                   /* dis modem status interrupt   */
    }

    SET_REG(psiochan, LCR, psiochan->lcr);
    SET_REG(psiochan, MCR, psiochan->mcr);

    /*
     * now reset the channel mode registers
     */
    SET_REG(psiochan, FCR,
            ((psiochan->rx_trigger_level << 6) |
             RxCLEAR | TxCLEAR | FIFO_ENABLE));

    if (hw_option & CREAD) {
        psiochan->ier |= RxFIFO_BIT;
    }

    if (psiochan->channel_mode == SIO_MODE_INT) {
        SET_REG(psiochan, IER, psiochan->ier);                          /* enable interrupt             */
    }

    psiochan->hw_option = hw_option;

    LW_SPIN_UNLOCK_QUICK(&psiochan->slock, intreg);

    return  (ERROR_NONE);
}
/*********************************************************************************************************
** 函数名称: sio16c550Hup
** 功能描述: 串口接口挂起
** 输　入  : psiochan      SIO CHAN
** 输　出  : ERROR or OK
*********************************************************************************************************/
static INT sio16c550Hup (SIO16C550_CHAN *psiochan)
{
    INTREG  intreg;

    LW_SPIN_LOCK_QUICK(&psiochan->slock, &intreg);

    psiochan->mcr &= (~(MCR_RTS | MCR_DTR));
    SET_REG(psiochan, MCR, psiochan->mcr);
    SET_REG(psiochan, FCR, (RxCLEAR | TxCLEAR));

    LW_SPIN_UNLOCK_QUICK(&psiochan->slock, intreg);

    return  (ERROR_NONE);
}
/*********************************************************************************************************
** 函数名称: sio16c550Open
** 功能描述: 串口接口打开 (Set the modem control lines)
** 输　入  : psiochan      SIO CHAN
** 输　出  : ERROR or OK
*********************************************************************************************************/
static INT sio16c550Open (SIO16C550_CHAN *psiochan)
{
    INTREG  intreg;
    UINT8   mask;

    mask = (UINT8)(GET_REG(psiochan, MCR) & (MCR_RTS | MCR_DTR));

    if (mask != (MCR_RTS | MCR_DTR)) {
        /*
         * RTS and DTR not set yet
         */
        LW_SPIN_LOCK_QUICK(&psiochan->slock, &intreg);

        /*
         * set RTS and DTR TRUE
         */
        psiochan->mcr |= (MCR_DTR | MCR_RTS);
        SET_REG(psiochan, MCR, psiochan->mcr);

        /*
         * clear Tx and receive and enable FIFO
         */
        SET_REG(psiochan, FCR,
                ((psiochan->rx_trigger_level << 6) |
                 RxCLEAR | TxCLEAR | FIFO_ENABLE));

        LW_SPIN_UNLOCK_QUICK(&psiochan->slock, intreg);
    }

    return  (ERROR_NONE);
}
/*********************************************************************************************************
** 函数名称: sio16550SetMode
** 功能描述: 串口接口设置模式
** 输　入  : psiochan      SIO CHAN
**           newmode       新的模式 SIO_MODE_INT / SIO_MODE_POLL
** 输　出  : ERROR or OK
*********************************************************************************************************/
static INT sio16c550SetMode (SIO16C550_CHAN *psiochan, INT newmode)
{
    INTREG  intreg;
    UINT8   mask;

    if ((newmode != SIO_MODE_POLL) && (newmode != SIO_MODE_INT)) {
        _ErrorHandle(EINVAL);
        return  (PX_ERROR);
    }

    if (psiochan->channel_mode == newmode) {
        return  (ERROR_NONE);
    }

    LW_SPIN_LOCK_QUICK(&psiochan->slock, &intreg);

    if (newmode == SIO_MODE_INT) {
        /*
         * Enable appropriate interrupts
         */
        if (psiochan->hw_option & CLOCAL) {
            SET_REG(psiochan, IER, (psiochan->ier | RxFIFO_BIT | TxFIFO_BIT));

        } else {
            mask = (UINT8)(GET_REG(psiochan, MSR) & MSR_CTS);
            /*
             * if the CTS is asserted enable Tx interrupt
             */
            if (mask & MSR_CTS) {
                psiochan->ier |= TxFIFO_BIT;                            /* enable Tx interrupt          */

            } else {
                psiochan->ier &= (~TxFIFO_BIT);                         /* disable Tx interrupt         */
            }

            SET_REG(psiochan, IER, psiochan->ier);
        }
    } else {
        /*
         * disable all ns16550 interrupts
         */
        SET_REG(psiochan, IER, 0);
    }

    psiochan->channel_mode = newmode;

    LW_SPIN_UNLOCK_QUICK(&psiochan->slock, intreg);

    return  (ERROR_NONE);
}
/*********************************************************************************************************
** 函数名称: sio16c550Ioctl
** 功能描述: 串口接口控制
** 输　入  : psiochan      SIO CHAN
**           cmd           命令
**           arg           参数
** 输　出  : ERROR or OK
*********************************************************************************************************/
static INT sio16c550Ioctl (SIO16C550_CHAN *psiochan, INT cmd, LONG arg)
{
    INT  error = ERROR_NONE;

    switch (cmd) {

    case SIO_BAUD_SET:
        if (arg < 50) {
            _ErrorHandle(EIO);
            error = PX_ERROR;
        } else {
            error = sio16c550SetBaud(psiochan, arg);
        }
        break;

    case SIO_BAUD_GET:
        *((LONG *)arg) = psiochan->baud;
        break;

    case SIO_MODE_SET:
        error = sio16c550SetMode(psiochan, (INT)arg);
        break;

    case SIO_MODE_GET:
        *((LONG *)arg) = psiochan->channel_mode;
        break;

    case SIO_HW_OPTS_SET:
        error = sio16c550SetHwOption(psiochan, arg);
        break;

    case SIO_HW_OPTS_GET:
        *(LONG *)arg = psiochan->hw_option;
        break;

    case SIO_HUP:
        if (psiochan->hw_option & HUPCL) {
            error = sio16c550Hup(psiochan);
        }
        break;

    case SIO_OPEN:
        if (psiochan->hw_option & HUPCL) {
            error = sio16c550Open(psiochan);
        }
        break;

    case SIO_SWITCH_PIN_EN_SET:
        if (*(INT *)arg) {
            if (!psiochan->send_start) {
                _ErrorHandle(ENOSYS);
                error = PX_ERROR;
                break;
            }
            if (psiochan->switch_en == 0) {
                SEND_END(psiochan);
            }
            psiochan->switch_en = 1;
        } else {
            psiochan->switch_en = 0;
        }
        break;

    case SIO_SWITCH_PIN_EN_GET:
        *(INT *)arg = psiochan->switch_en;
        break;

    default:
        _ErrorHandle(ENOSYS);
        error = PX_ERROR;
        break;
    }

    return  (error);
}
/*********************************************************************************************************
** 函数名称: sio16c550TxStartup
** 功能描述: 串口接口启动发送
** 输　入  : psiochan      SIO CHAN
** 输　出  : ERROR or OK
*********************************************************************************************************/
static INT sio16c550TxStartup (SIO16C550_CHAN *psiochan)
{
    INTREG  intreg;
    UINT8   mask;
    CHAR    cTx;

    if (psiochan->switch_en && (psiochan->hw_option & CLOCAL)) {
        SEND_START(psiochan);                                           /* switch direct to send mode   */

        do {
            if (psiochan->pcbGetTxChar(psiochan->getTxArg, &cTx) != ERROR_NONE) {
                break;
            }
            while (!IS_Tx_HOLD_REG_EMPTY(psiochan));                    /* wait tx holding reg empty    */

            SET_REG(psiochan, THR, cTx);
        } while (1);

        while (!IS_Tx_HOLD_REG_EMPTY(psiochan));                        /* wait tx holding reg empty    */

        SEND_END(psiochan);                                             /* switch direct to rx mode     */

        return  (ERROR_NONE);
    }

    LW_SPIN_LOCK_QUICK(&psiochan->slock, &intreg);

    if (psiochan->channel_mode == SIO_MODE_INT) {
        if (psiochan->hw_option & CLOCAL) {                             /* No modem control             */
            if (psiochan->bdefer == LW_FALSE) {
                psiochan->ier |= TxFIFO_BIT;
            }

        } else {
            mask = (UINT8)(GET_REG(psiochan, MSR) & MSR_CTS);
            if (mask & MSR_CTS) {                                       /* if the CTS is enable Tx Int  */
                if (psiochan->bdefer == LW_FALSE) {
                    psiochan->ier |= TxFIFO_BIT;                        /* enable Tx interrupt          */
                }
            } else {
                psiochan->ier &= (~TxFIFO_BIT);                         /* disable Tx interrupt         */
            }
        }

        SET_REG(psiochan, IER, psiochan->ier);

        LW_SPIN_UNLOCK_QUICK(&psiochan->slock, intreg);

        return  (ERROR_NONE);

    } else {
        LW_SPIN_UNLOCK_QUICK(&psiochan->slock, intreg);

        _ErrorHandle(ENOSYS);
        return  (ENOSYS);
    }
}
/*********************************************************************************************************
** 函数名称: sio16c550CallbackInstall
** 功能描述: 串口接口安装回调函数
** 输　入  : pchan          SIO CHAN
**           callbackType   回调类型
**           callback       回调函数
**           callbackArg    回调参数
** 输　出  : ERROR or OK
*********************************************************************************************************/
static INT sio16c550CallbackInstall (SIO_CHAN *pchan,
                                     INT       callbackType,
                                     INT     (*callback)(),
                                     VOID     *callbackArg)
{
    SIO16C550_CHAN *psiochan = (SIO16C550_CHAN *)pchan;

    switch (callbackType) {

    case SIO_CALLBACK_GET_TX_CHAR:
        psiochan->pcbGetTxChar = callback;
        psiochan->getTxArg     = callbackArg;
        return  (ERROR_NONE);

    case SIO_CALLBACK_PUT_RCV_CHAR:
        psiochan->pcbPutRcvChar = callback;
        psiochan->putRcvArg     = callbackArg;
        return  (ERROR_NONE);

    default:
        _ErrorHandle(ENOSYS);
        return  (PX_ERROR);
    }
}
/*********************************************************************************************************
** 函数名称: sio16c550PollInput
** 功能描述: 串口接口查询方式输入
** 输　入  : pchan          SIO CHAN
**           pc             读取到的数据
** 输　出  : ERROR or OK
*********************************************************************************************************/
static INT sio16c550PollInput (SIO16C550_CHAN *psiochan, CHAR *pc)
{
    UINT8 poll_status = GET_REG(psiochan, LSR);

    if ((poll_status & LSR_DR) == 0x00) {
        _ErrorHandle(EAGAIN);
        return  (PX_ERROR);
    }

    *pc = GET_REG(psiochan, RBR);                                       /* got a character              */

    return  (ERROR_NONE);
}
/*********************************************************************************************************
** 函数名称: sio16c550PollInput
** 功能描述: 串口接口查询方式输出
** 输　入  : pchan          SIO CHAN
**           c              输出的数据
** 输　出  : ERROR or OK
*********************************************************************************************************/
static INT sio16c550PollOutput (SIO16C550_CHAN *psiochan, char c)
{
    UINT8 msr = GET_REG(psiochan, MSR);

    while (!IS_Tx_HOLD_REG_EMPTY(psiochan));                            /* wait tx holding reg empty    */

    if (!(psiochan->hw_option & CLOCAL)) {                              /* modem flow control ?         */
        if (msr & MSR_CTS) {
            SET_REG(psiochan, THR, c);

        } else {
            _ErrorHandle(EAGAIN);
            return  (PX_ERROR);
        }

    } else {
        SET_REG(psiochan, THR, c);
    }

    return  (ERROR_NONE);
}
/*********************************************************************************************************
** 函数名称: sio16c550TxIsr
** 功能描述: 16C550 输出中断服务函数
** 输　入  : psiochan      SIO CHAN
** 输　出  : ERROR or OK
*********************************************************************************************************/
VOID sio16c550TxIsr (SIO16C550_CHAN *psiochan)
{
    INTREG  intreg;
    UINT8   ucTx;
    UINT8   lsr;
    INT     i;

    LW_SPIN_LOCK_QUICK(&psiochan->slock, &intreg);

    psiochan->bdefer = LW_FALSE;

    lsr = psiochan->lsr;                                                /*  first use save lsr value    */

    if (lsr & LSR_THRE) {
        for (i = 0; i < psiochan->fifo_len; i++) {
            LW_SPIN_UNLOCK_QUICK(&psiochan->slock, intreg);
            if (psiochan->pcbGetTxChar(psiochan->getTxArg, &ucTx)) {
                return;
            }
            LW_SPIN_LOCK_QUICK(&psiochan->slock, &intreg);

            SET_REG(psiochan, THR, ucTx);                               /* char to Transmit Holding Reg */
        }
    }

    psiochan->ier |= TxFIFO_BIT;                                        /*  enable Tx Int               */
    SET_REG(psiochan, IER, psiochan->ier);                              /*  update ier                  */

    LW_SPIN_UNLOCK_QUICK(&psiochan->slock, intreg);
}
/*********************************************************************************************************
** 函数名称: sio16c550RxIsr
** 功能描述: 16C550 输入中断服务函数
** 输　入  : psiochan      SIO CHAN
** 输　出  : ERROR or OK
*********************************************************************************************************/
VOID sio16c550RxIsr (SIO16C550_CHAN *psiochan)
{
    INTREG  intreg;
    UINT8   ucRd;
    UINT8   lsr;

    LW_SPIN_LOCK_QUICK(&psiochan->slock, &intreg);

    lsr = psiochan->lsr;                                                /*  first use save lsr value    */

    do {
        if (lsr & (LSR_BI | LSR_FE | LSR_PE | LSR_OE)) {
            if (lsr & RxCHAR_AVAIL) {
                ucRd = GET_REG(psiochan, RBR);
            }
            if (lsr & LSR_BI) {
                psiochan->err_break++;
            }
            if (lsr & LSR_FE) {
                psiochan->err_framing++;
            }
            if (lsr & LSR_PE) {
                psiochan->err_parity++;
            }
            if (lsr & LSR_OE) {
                psiochan->err_overrun++;
            }

        } else if (lsr & RxCHAR_AVAIL) {
            ucRd = GET_REG(psiochan, RBR);

            LW_SPIN_UNLOCK_QUICK(&psiochan->slock, intreg);
            psiochan->pcbPutRcvChar(psiochan->putRcvArg, ucRd);
            LW_SPIN_LOCK_QUICK(&psiochan->slock, &intreg);
        }

        lsr = GET_REG(psiochan, LSR);
    } while (lsr & RxCHAR_AVAIL);

    psiochan->ier |= RxFIFO_BIT;                                        /*  enable Rx Int               */
    SET_REG(psiochan, IER, psiochan->ier);                              /*  update ier                  */

    LW_SPIN_UNLOCK_QUICK(&psiochan->slock, intreg);
}
/*********************************************************************************************************
** 函数名称: sio16c550Isr
** 功能描述: 16C550 中断服务函数
** 输　入  : psiochan      SIO CHAN
** 输　出  : ERROR or OK
*********************************************************************************************************/
VOID sio16c550Isr (SIO16C550_CHAN *psiochan)
{
    INTREG  intreg;
    UINT8   iir;
    UINT8   lsr;
    UINT8   msr;

    LW_SPIN_LOCK_QUICK(&psiochan->slock, &intreg);

    iir = (UINT8)(GET_REG(psiochan, IIR) & 0x0f);

    SET_REG(psiochan, IER, 0);                                          /* disable all interrupt        */

    lsr = GET_REG(psiochan, LSR);

    if (lsr & (RxCHAR_AVAIL | LSR_BI | LSR_FE | LSR_PE | LSR_OE)) {     /* Rx Int                       */
        psiochan->ier &= (~RxFIFO_BIT);
        SET_REG(psiochan, IER, psiochan->ier);                          /* disable Rx Int               */

        psiochan->lsr = lsr;                                            /* save lsr                     */
        LW_SPIN_UNLOCK_QUICK(&psiochan->slock, intreg);

#if LW_CFG_ISR_DEFER_EN > 0
        if (psiochan->pdeferq) {
            if (API_InterDeferJobAdd(psiochan->pdeferq,
                                     sio16c550RxIsr, psiochan)) {
                sio16c550RxIsr(psiochan);                               /* queue error                  */
            }
        } else
#endif                                                                  /* LW_CFG_ISR_DEFER_EN > 0      */
        {
            sio16c550RxIsr(psiochan);
        }

        LW_SPIN_LOCK_QUICK(&psiochan->slock, &intreg);
    }

    if ((lsr & LSR_THRE) || ((iir & 0x06) == IIR_THRE)) {               /* Tx Int                       */
        psiochan->ier &= (~TxFIFO_BIT);                                 /* indicate to disable Tx Int   */
        SET_REG(psiochan, IER, psiochan->ier);                          /* disable Tx Int               */

        if (psiochan->bdefer == LW_FALSE) {                             /* not in queue                 */
            psiochan->bdefer =  LW_TRUE;

            psiochan->lsr = lsr;                                        /* save lsr                     */
            LW_SPIN_UNLOCK_QUICK(&psiochan->slock, intreg);

#if LW_CFG_ISR_DEFER_EN > 0
            if (psiochan->pdeferq) {
                if (API_InterDeferJobAdd(psiochan->pdeferq,
                                         sio16c550TxIsr, psiochan)) {
                    sio16c550TxIsr(psiochan);                           /* queue error                  */
                }
            } else
#endif                                                                  /* LW_CFG_ISR_DEFER_EN > 0      */
            {
                sio16c550TxIsr(psiochan);
            }

            LW_SPIN_LOCK_QUICK(&psiochan->slock, &intreg);
        }
    }

    switch (iir) {

    case IIR_RLS:                                                       /* overrun, parity error        */
        if (lsr & LSR_PE) {
            GET_REG(psiochan, RBR);
        }
        SET_REG(psiochan, IER, psiochan->ier);                          /*  update ier                  */
        LW_SPIN_UNLOCK_QUICK(&psiochan->slock, intreg);
        break;

    case IIR_MSTAT:                                                     /* modem status register        */
        msr = GET_REG(psiochan, MSR);
        if (msr & MSR_DCTS) {
            if (msr & MSR_CTS) {
                psiochan->ier |= TxFIFO_BIT;                            /* CTS was turned on            */

            } else {
                psiochan->ier &= (~TxFIFO_BIT);                         /* CTS was turned off           */
            }
        }
        SET_REG(psiochan, IER, psiochan->ier);                          /*  update ier                  */
        LW_SPIN_UNLOCK_QUICK(&psiochan->slock, intreg);
        break;

    default:
        SET_REG(psiochan, IER, psiochan->ier);                          /*  update ier                  */
        LW_SPIN_UNLOCK_QUICK(&psiochan->slock, intreg);
        break;
    }
}

#endif                                                                  /*  LW_CFG_DEVICE_EN            */
                                                                        /*  LW_CFG_SIO_DEVICE_EN        */
                                                                        /*  LW_CFG_DRV_SIO_16C550       */
/*********************************************************************************************************
  END
*********************************************************************************************************/
