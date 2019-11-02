/*
 * sio.c
 *
 *  Created on: Jun 7, 2019
 *      Author: databus
 */

#define  __SYLIXOS_KERNEL
#include <SylixOS.h>
#include <linux/compat.h>
#include "driver/sio/16550.h"
#include "driver/include/irq_numbers.h"

/*********************************************************************************************************
  16C550 SIO 配置定义
*********************************************************************************************************/
typedef struct {
       ULONG        CFG_ulBase;
       ULONG        CFG_ulXtal;
       ULONG        CFG_ulBaud;
       ULONG        CFG_ulVector;
       ULONG        CFG_ulRegShift;
} SIO16C550_CFG;

#define APB2_CLK                        24 * 1000 * 1000
#define BSP_CFG_16C550_SIO_NR           4
#define UART0_BASE                      (0x01c28000)
#define UART1_BASE                      (0x01c28400)
#define UART2_BASE                      (0x01c28800)
#define UART3_BASE                      (0x01c28c00)
#define UART4_BASE                      (0x01c29000)

#define BSP_CFG_16C550_0_BASE           (UART0_BASE)
#define BSP_CFG_16C550_0_SIZE           LW_CFG_VMM_PAGE_SIZE
#define BSP_CFG_16C550_0_BAUD           115200
#define BSP_CFG_16C550_0_XTAL           (APB2_CLK - 8 * BSP_CFG_16C550_0_BAUD)
#define BSP_CFG_16C550_0_VECTOR         UART0
#define BSP_CFG_16C550_0_REG_SHIFT      2

#define BSP_CFG_16C550_1_BASE           (UART1_BASE)
#define BSP_CFG_16C550_1_SIZE           LW_CFG_VMM_PAGE_SIZE
#define BSP_CFG_16C550_1_BAUD           115200
#define BSP_CFG_16C550_1_XTAL           (APB2_CLK - 8 * BSP_CFG_16C550_1_BAUD)
#define BSP_CFG_16C550_1_VECTOR         UART1
#define BSP_CFG_16C550_1_REG_SHIFT      2

#define BSP_CFG_16C550_2_BASE           (UART2_BASE)
#define BSP_CFG_16C550_2_SIZE           LW_CFG_VMM_PAGE_SIZE
#define BSP_CFG_16C550_2_BAUD           115200
#define BSP_CFG_16C550_2_XTAL           (APB2_CLK - 8 * BSP_CFG_16C550_2_BAUD)
#define BSP_CFG_16C550_2_VECTOR         UART2
#define BSP_CFG_16C550_2_REG_SHIFT      2

#define BSP_CFG_16C550_3_BASE           (UART3_BASE)
#define BSP_CFG_16C550_3_SIZE           LW_CFG_VMM_PAGE_SIZE
#define BSP_CFG_16C550_3_BAUD           115200
#define BSP_CFG_16C550_3_XTAL           (APB2_CLK - 8 * BSP_CFG_16C550_3_BAUD)
#define BSP_CFG_16C550_3_VECTOR         UART3
#define BSP_CFG_16C550_3_REG_SHIFT      2
/*********************************************************************************************************
  Qemu SylixOS 平台 16C550 SIO 配置初始化
*********************************************************************************************************/
SIO16C550_CFG      _G_sio16C550Cfgs[BSP_CFG_16C550_SIO_NR] = {
#ifdef BSP_CFG_16C550_0_BASE
    {
        BSP_CFG_16C550_0_BASE,
        BSP_CFG_16C550_0_XTAL,
        BSP_CFG_16C550_0_BAUD,
        BSP_CFG_16C550_0_VECTOR,
        BSP_CFG_16C550_0_REG_SHIFT,
    },
#endif
#ifdef BSP_CFG_16C550_1_BASE
    {
        BSP_CFG_16C550_1_BASE,
        BSP_CFG_16C550_1_XTAL,
        BSP_CFG_16C550_1_BAUD,
        BSP_CFG_16C550_1_VECTOR,
        BSP_CFG_16C550_1_REG_SHIFT,
    },
#endif
#ifdef BSP_CFG_16C550_2_BASE
    {
        BSP_CFG_16C550_2_BASE,
        BSP_CFG_16C550_2_XTAL,
        BSP_CFG_16C550_2_BAUD,
        BSP_CFG_16C550_2_VECTOR,
        BSP_CFG_16C550_2_REG_SHIFT,
    },
#endif
#ifdef BSP_CFG_16C550_3_BASE
    {
        BSP_CFG_16C550_3_BASE,
        BSP_CFG_16C550_3_XTAL,
        BSP_CFG_16C550_3_BAUD,
        BSP_CFG_16C550_3_VECTOR,
        BSP_CFG_16C550_3_REG_SHIFT,
    },
#endif
};
/*********************************************************************************************************
  16C550 SIO 通道
*********************************************************************************************************/
static SIO16C550_CHAN   _G_sio16C550Chans[BSP_CFG_16C550_SIO_NR];
/*********************************************************************************************************
** 函数名称: __sio16C550SetReg
** 功能描述: 设置 16C550 寄存器
** 输　入  : pSio16C550Chan        16C550 SIO 通道
**           iReg                  寄存器
**           ucValue               值
** 输　出  : NONE
*********************************************************************************************************/
static VOID  __sio16C550SetReg (SIO16C550_CHAN  *pSio16C550Chan, INT  iReg, UINT8  ucValue)
{
    SIO16C550_CFG   *pSio16C550Cfg = pSio16C550Chan->priv;

    write8(ucValue, pSio16C550Cfg->CFG_ulBase + (iReg << pSio16C550Cfg->CFG_ulRegShift));
}
/*********************************************************************************************************
** 函数名称: __sio16C550GetReg
** 功能描述: 获得 16C550 寄存器的值
** 输　入  : pSio16C550Chan        16C550 SIO 通道
**           iReg                  寄存器
** 输　出  : 寄存器的值
*********************************************************************************************************/
static UINT8  __sio16C550GetReg (SIO16C550_CHAN  *pSio16C550Chan, INT  iReg)
{
    SIO16C550_CFG   *pSio16C550Cfg = pSio16C550Chan->priv;

    return  (read8(pSio16C550Cfg->CFG_ulBase + (iReg << pSio16C550Cfg->CFG_ulRegShift)));
}
/*********************************************************************************************************
** 函数名称: __sio16C550Isr
** 功能描述: 16C550 中断服务程序
** 输　入  : pSio16C550Chan        16C550 SIO 通道
**           ulVector              中断向量号
** 输　出  : 中断返回值
*********************************************************************************************************/
static irqreturn_t  __sio16C550Isr (SIO16C550_CHAN  *pSio16C550Chan, ULONG  ulVector)
{
    sio16c550Isr(pSio16C550Chan);

    return  (LW_IRQ_HANDLED);
}
/*********************************************************************************************************
** 函数名称: sioChanCreate
** 功能描述: 创建一个 SIO 通道
** 输　入  : uiChannel                 硬件通道号
** 输　出  : SIO 通道
*********************************************************************************************************/
SIO_CHAN  *sioChanCreate (UINT  uiChannel)
{
    SIO16C550_CHAN          *pSio16C550Chan;
    SIO16C550_CFG           *pSio16C550Cfg;

    if (uiChannel < BSP_CFG_16C550_SIO_NR) {

        pSio16C550Chan = &_G_sio16C550Chans[uiChannel];
        pSio16C550Cfg  = &_G_sio16C550Cfgs[uiChannel];

        /*
         *  Receiver FIFO Trigger Level and Tirgger bytes table
         *  level  16 Bytes FIFO Trigger   32 Bytes FIFO Trigger  64 Bytes FIFO Trigger
         *    0              1                       8                    1
         *    1              4                      16                   16
         *    2              8                      24                   32
         *    3             14                      28                   56
         */
        pSio16C550Chan->fifo_len         = 60;
        pSio16C550Chan->rx_trigger_level = 3;

        pSio16C550Chan->baud   = pSio16C550Cfg->CFG_ulBaud;
        pSio16C550Chan->xtal   = pSio16C550Cfg->CFG_ulXtal;
        pSio16C550Chan->setreg = __sio16C550SetReg;
        pSio16C550Chan->getreg = __sio16C550GetReg;

        pSio16C550Chan->priv   = pSio16C550Cfg;

        API_InterVectorDisable(pSio16C550Cfg->CFG_ulVector);
        sio16c550Init(pSio16C550Chan);
        API_InterVectorConnect(pSio16C550Cfg->CFG_ulVector,
                               (PINT_SVR_ROUTINE)__sio16C550Isr,
                               (PVOID)pSio16C550Chan,
                               "16c550_isr");                           /*  安装操作系统中断向量表      */

        API_InterVectorEnable(pSio16C550Cfg->CFG_ulVector);

#if 0
        //bind interrupt on cpu
        LW_CLASS_CPUSET set;
        LW_CPU_ZERO(&set);
        LW_CPU_SET(3, &set);
        API_InterSetTarget(pSio16C550Cfg->CFG_ulVector, sizeof(LW_CLASS_CPUSET), &set);
#endif

        return  ((SIO_CHAN *)pSio16C550Chan);
    } else {
        return  (LW_NULL);
    }
}
/*********************************************************************************************************
  END
*********************************************************************************************************/
