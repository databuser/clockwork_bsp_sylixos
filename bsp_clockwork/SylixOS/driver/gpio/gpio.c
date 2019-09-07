/*
 * gpio.c
 *
 *  Created on: Jun 9, 2019
 *      Author: databus
 */


#define __SYLIXOS_KERNEL
#include <SylixOS.h>
#include <linux/compat.h>
#include "driver/include/irq_numbers.h"
#include "driver/pinmux/pinmux.h"
/*********************************************************************************************************
  管脚功能固定值选择宏
*********************************************************************************************************/
#define GPIO_INPUT                 ALT0                                 /*  管脚为数字输入功能          */
#define GPIO_OUTPUT                ALT1                                 /*  管脚为数字输出功能          */
#define GPIO_EINT                  ALT4                                 /*  管脚为中断触发              */
#define GPIO_DISABLE               ALT7                                 /*  管脚关闭                    */
/*********************************************************************************************************
 中断触发类型选择宏
*********************************************************************************************************/
#define EINT_MODE_POSITIVE_EDGE    (0X00)                               /*  上升沿触发                  */
#define EINT_MODE_NEGATIVE_EDGE    (0X01)                               /*  下降沿触发                  */
#define EINT_MODE_HIGH_LEVEL       (0X02)                               /*  高电平触发                  */
#define EINT_MODE_LOW_LEVEL        (0X03)                               /*  低电平触发                  */
#define EINT_MODE_DOUBLE_EDGE      (0X04)                               /*  双边沿触发                  */
/*********************************************************************************************************
  GPIO控制寄存器偏移量定义 (R16只有PB、PG引脚有中断功能)
*********************************************************************************************************/
#define GPIO_CFG                   (0x00)                               /*  GPIO 功能设置寄存器         */
#define GPIO_DAT                   (0x10)                               /*  GPIO 数据寄存器             */
#define GPIO_DRV                   (0x14)                               /*  GPIO 驱动能力设置寄存器     */
#define GPIO_PUL                   (0x1C)                               /*  GPIO 上下拉设置寄存器       */

#define GPIO_B_INT_CFG             (0x220)                              /*  GPIOB 中断触发类型设置寄存器*/
#define GPIO_B_INT_CTL             (0x230)                              /*  GPIOB 中断控制寄存器        */
#define GPIO_B_INT_STA             (0x234)                              /*  GPIOB 中断状态寄存器        */
#define GPIO_B_INT_DEB             (0x238)                              /*  GPIOB 中断频率设置寄存器    */

#define GPIO_G_INT_CFG             (0x240)                              /*  GPIOG 中断触发类型设置寄存器*/
#define GPIO_G_INT_CTL             (0x250)                              /*  GPIOG 中断控制寄存器        */
#define GPIO_G_INT_STA             (0x254)                              /*  GPIOG 中断状态寄存器        */
#define GPIO_G_INT_DEB             (0x258)                              /*  GPIOG 中断频率设置寄存器    */
/*********************************************************************************************************
** 函数名称: sunxiR16GpioRequest
** 功能描述: 实现 GPIO 管脚的 PINMUX 设置
** 输  入  : pGpioChip GPIO 芯片
**           uiNum     GPIO在系统中的编号
** 输  出  : ERROR CODE
*********************************************************************************************************/
static INT  sunxiR16GpioRequest (PLW_GPIO_CHIP  pGpioChip, UINT uiNum)
{
    return  (gpioPinmuxSet(uiNum, GPIO_DISABLE));
}
/*********************************************************************************************************
** 函数名称: sunxiR16GpioFree
** 功能描述: 释放一个正在被使用的 GPIO, 如果当前是中断模式则, 放弃中断输入功能.
** 输  入  : pGpioChip   GPIO 芯片
**           uiNum     GPIO在系统中的编号
** 输  出  : ERROR CODE
*********************************************************************************************************/
static VOID  sunxiR16GpioFree (PLW_GPIO_CHIP  pGpioChip, UINT uiNum)
{
    INT     iRet;
    addr_t  ulAddr;
    UINT32  uiValue;
    UINT32  uiINTN;
    INT32   iGpioSoc = gpioSysToSoc(uiNum);

    if (iGpioSoc == PX_ERROR) {
        return  ;
    }

    /*
     *  设置引脚IO功能关闭
     */
    iRet = gpioPinmuxSet(uiNum, GPIO_DISABLE);
    if (PX_ERROR == iRet){
        return  ;
    }


    /*
     *  获得引脚中断控制寄存器地址
     */
    if (iGpioSoc >= GPIO_NUM(GPIO_PORT_B, GPIO_PIN_00) &&
        iGpioSoc <= GPIO_NUM(GPIO_PORT_B, GPIO_PIN_07)) {
        uiINTN = iGpioSoc - GPIO_NUM(GPIO_PORT_B, GPIO_PIN_00);          /*  GPIOB中断EINT0-7           */
        ulAddr = GPIO_B_INT_CTL + GPIO_BASE;                             /*  中断控制寄存器地址         */
    } else if (iGpioSoc >= GPIO_NUM(GPIO_PORT_G, GPIO_PIN_00) &&
               iGpioSoc <= GPIO_NUM(GPIO_PORT_G, GPIO_PIN_13)) {
        uiINTN = iGpioSoc - GPIO_NUM(GPIO_PORT_G, GPIO_PIN_00);          /*  GPIOG中断EINT0-13          */
        ulAddr = GPIO_G_INT_CTL + GPIO_BASE;                             /*  中断控制寄存器地址         */
    } else {
        return;                                                          /*  该端口没有中断功能         */
    }

    /*
     *  关闭引脚中断功能
     */
    uiValue = readl(ulAddr);
    uiValue &= ~BIT(uiINTN);
    writel(uiValue, ulAddr);

    return  ;
}
/*********************************************************************************************************
** 函数名称: sunxiR16GpioGetDirection
** 功能描述: 获得指定 GPIO 方向
** 输  入  : pGpioChip   GPIO 芯片
**           uiNum       GPIO在系统中的编号
** 输  出  : 0: 输入 1:输出 -1:错误
*********************************************************************************************************/
static INT  sunxiR16GpioGetDirection (PLW_GPIO_CHIP  pGpioChip, UINT uiNum)
{
    UINT32  uiRegVal;

    uiRegVal = gpioPinmuxGet(uiNum);
    if (uiRegVal == GPIO_INPUT) {
        return  (0);
    } else if(uiRegVal == GPIO_OUTPUT) {
        return  (1);
    } else {
        return  (PX_ERROR);
    }
}
/*********************************************************************************************************
** 函数名称: sunxiR16GpioDirectionInput
** 功能描述: 设置指定 GPIO 为输入模式
** 输  入  : pGpioChip   GPIO 芯片
**           uiNum       GPIO在系统中的编号
** 输  出  : 0: 正确 -1:错误
*********************************************************************************************************/
static INT  sunxiR16GpioDirectionInput (PLW_GPIO_CHIP  pGpioChip, UINT uiNum)
{
    return  (gpioPinmuxSet(uiNum, GPIO_INPUT));
}
/*********************************************************************************************************
** 函数名称: sunxiR16GpioGet
** 功能描述: 获得指定 GPIO 电平
** 输  入  : pGpioChip   GPIO 芯片
**           uiNum       GPIO在系统中的编号
** 输  出  : 0: 低电平 1:高电平 -1:错误
*********************************************************************************************************/
static INT  sunxiR16GpioGet (PLW_GPIO_CHIP  pGpioChip, UINT  uiNum)
{
    addr_t  ulAddr;
    UINT32  uiPortNum;
    UINT32  uiPinNum;
    INT32   iGpioSoc = gpioSysToSoc(uiNum);

    if (iGpioSoc == PX_ERROR) {
        return  (PX_ERROR);
    }

    /*
     *  计算组号和引脚号
     */
    uiPortNum  = GET_SOC_PORT_NUM(iGpioSoc);
    uiPinNum   = GET_SOC_PIN_NUM(iGpioSoc);

    if (uiPinNum > GPIO_PIN_31) {
        return  (PX_ERROR);
    }

    ulAddr = GPIO_DAT + uiPortNum * 0x24 + GPIO_BASE;

    return  (readl(ulAddr) & (BIT(uiPinNum))) ? 1 : 0;
}
/*********************************************************************************************************
** 函数名称: sunxiR16GpioSet
** 功能描述: 设置指定 GPIO 电平
** 输  入  : pGpioChip   GPIO 芯片
**           uiNum       GPIO在系统中的编号
**           iValue      输出电平
** 输  出  : 0: 正确 -1:错误
*********************************************************************************************************/
static VOID  sunxiR16GpioSet (PLW_GPIO_CHIP  pGpioChip, UINT  uiNum, INT  iValue)
{
    addr_t  ulAddr;
    UINT32  uiRegister;
    UINT32  uiPortNum;
    UINT32  uiPinNum;
    INT32   iGpioSoc = gpioSysToSoc(uiNum);

    if (iGpioSoc == PX_ERROR) {
        return  ;
    }

    /*
     *  计算组号和引脚号
     */
    uiPortNum  = GET_SOC_PORT_NUM(iGpioSoc);
    uiPinNum   = GET_SOC_PIN_NUM(iGpioSoc);

    if (uiPinNum > GPIO_PIN_31) {
        return  ;
    }

    ulAddr = GPIO_DAT + uiPortNum * 0x24 + GPIO_BASE;

    uiRegister = readl(ulAddr);
    if (0 == iValue) {
        uiRegister &= ~BIT(uiPinNum);
    } else {
        uiRegister |= BIT(uiPinNum);
    }
    writel(uiRegister, ulAddr);

    return  ;
}
/*********************************************************************************************************
** 函数名称: sunxiR16GpioDirectionOutput
** 功能描述: 设置指定 GPIO 为输出模式
** 输  入  : pGpioChip   GPIO 芯片
**           uiNum       GPIO在系统中的编号
**           iValue      输出电平
** 输  出  : 0: 正确 -1:错误
*********************************************************************************************************/
static INT  sunxiR16GpioDirectionOutput (PLW_GPIO_CHIP  pGpioChip, UINT  uiNum, INT  iValue)
{
    INT32  iGpioSoc = gpioSysToSoc(uiNum);

    if (iGpioSoc == PX_ERROR) {
        return  (PX_ERROR);
    }

    gpioPinmuxSet(uiNum, GPIO_OUTPUT);
    sunxiR16GpioSet (pGpioChip, uiNum, iValue);

    return  (ERROR_NONE);
}
/*********************************************************************************************************
** 函数名称: sunxiR16GpioSetupIrq
** 功能描述: 设置指定 GPIO 为外部中断输入管脚
** 输  入  : pGpioChip   GPIO 芯片
**           uiNum       GPIO在系统中的编号
**           bIsLevel    是否为电平触发, 1 表示电平触发，0 表示边沿触发
**           uiType      如果为电平触发, 1 表示高电平触发, 0 表示低电平触发
**                       如果为边沿触发, 1 表示上升沿触发, 0 表示下降沿触发, 2 双边沿触发
** 输  出  : IRQ 向量号 -1:错误
*********************************************************************************************************/
static ULONG  sunxiR16GpioSetupIrq (PLW_GPIO_CHIP  pGpioChip, UINT  uiNum, BOOL  bIsLevel, UINT  uiType)
{
    addr_t  ulIntCfgAddr;
    addr_t  ulIntCtlAddr;
    UINT32  uiValue;
    UINT32  uiCfg;
    INT     iIRQn;
    UINT32  uiINTN;
    INT32   iGpioSoc = gpioSysToSoc(uiNum);

    if (iGpioSoc == PX_ERROR) {
        return  (PX_ERROR);
    }

    if (iGpioSoc >= GPIO_NUM(GPIO_PORT_B, GPIO_PIN_00) &&
        iGpioSoc <= GPIO_NUM(GPIO_PORT_B, GPIO_PIN_07)) {
        uiINTN = iGpioSoc - GPIO_NUM(GPIO_PORT_B, GPIO_PIN_00);          /*  GPIOB中断EINT0-7           */
        ulIntCfgAddr = GPIO_B_INT_CFG + GPIO_BASE + (uiINTN / 8 * 4);    /*  中断类型寄存器地址         */
        ulIntCtlAddr = GPIO_B_INT_CTL + GPIO_BASE;
        iIRQn  = PB_EINT;
    } else if (iGpioSoc >= GPIO_NUM(GPIO_PORT_G, GPIO_PIN_00) &&
               iGpioSoc <= GPIO_NUM(GPIO_PORT_G, GPIO_PIN_13)) {
        uiINTN = iGpioSoc - GPIO_NUM(GPIO_PORT_G, GPIO_PIN_00);          /*  GPIOG中断EINT0-13          */
        ulIntCfgAddr = GPIO_G_INT_CFG + GPIO_BASE + (uiINTN / 8 * 4);    /*  中断类型寄存器地址         */
        ulIntCtlAddr = GPIO_G_INT_CTL + GPIO_BASE;
        iIRQn  = PG_EINT;
    } else {
        return  (-1);                                                    /*  该端口没有中断功能         */
    }

    /*
     * 设置中断触发类型
     */
    if (bIsLevel) {                                                     /*  如果是电平触发中断          */
        if (uiType) {
            uiCfg = EINT_MODE_HIGH_LEVEL;                               /*  高电平触发                  */
        } else {
            uiCfg = EINT_MODE_LOW_LEVEL;                                /*  低电平触发                  */
        }
    } else {
        if (uiType == 0) {                                              /*  下降沿触发                  */
            uiCfg = EINT_MODE_NEGATIVE_EDGE;
        } else if (uiType == 1) {                                       /*  上升沿触发                  */
            uiCfg = EINT_MODE_POSITIVE_EDGE;
        } else {                                                        /*  双边沿触发                  */
            uiCfg = EINT_MODE_DOUBLE_EDGE;
        }
    }

    uiValue = readl(ulIntCfgAddr);
    uiValue &= ~(0x0f << ((uiINTN % 8) * 4));
    uiValue |= (uiCfg << ((uiINTN % 8) * 4));
    writel(uiValue, ulIntCfgAddr);

    /*
     * 设置引脚为中断输入功能
     */
    gpioPinmuxSet(uiNum, GPIO_EINT);

    /*
     * 使能引脚中断响应
     */
    uiValue = readl(ulIntCtlAddr);
    uiValue |= BIT(uiINTN);
    writel(uiValue, ulIntCtlAddr);

    return  (iIRQn);
}
/*********************************************************************************************************
** 函数名称: sunxiR16GpioClearIrq
** 功能描述: 清除指定 GPIO 中断标志
** 输  入  : pGpioChip   GPIO 芯片
**           uiNum       GPIO在系统中的编号
** 输  出  : NONE
*********************************************************************************************************/
static VOID  sunxiR16GpioClearIrq (PLW_GPIO_CHIP  pGpioChip, UINT  uiNum)
{
    addr_t  ulAddr;
    UINT32  uiINTN;
    INT32   iGpioSoc = gpioSysToSoc(uiNum);

    if (iGpioSoc == PX_ERROR) {
        return  ;
    }

    if (iGpioSoc >= GPIO_NUM(GPIO_PORT_B, GPIO_PIN_00) &&
        iGpioSoc <= GPIO_NUM(GPIO_PORT_B, GPIO_PIN_07)) {
        uiINTN = iGpioSoc - GPIO_NUM(GPIO_PORT_B, GPIO_PIN_00);          /*  GPIOB中断EINT0-7           */
        ulAddr = GPIO_B_INT_STA + GPIO_BASE;                             /*  中断状态寄存器地址         */
    } else if (iGpioSoc >= GPIO_NUM(GPIO_PORT_G, GPIO_PIN_00) &&
               iGpioSoc <= GPIO_NUM(GPIO_PORT_G, GPIO_PIN_13)) {
        uiINTN = iGpioSoc - GPIO_NUM(GPIO_PORT_G, GPIO_PIN_00);          /*  GPIOG中断EINT0-13          */
        ulAddr = GPIO_G_INT_STA + GPIO_BASE;                             /*  中断状态寄存器地址         */
    } else {
        return;                                                          /*  该端口没有中断功能         */
    }

    writel(BIT(uiINTN), ulAddr);

    return  ;
}
/*********************************************************************************************************
** 函数名称: sunxiR16GpioSvrIrq
** 功能描述: 判断 GPIO 中断标志
** 输  入  : pGpioChip   GPIO 芯片
**           uiNum       GPIO在系统中的编号
** 输  出  : 中断返回值
*********************************************************************************************************/
static irqreturn_t  sunxiR16GpioSvrIrq (PLW_GPIO_CHIP  pGpioChip, UINT  uiNum)
{
    addr_t  ulAddr;
    UINT32  uiRegister;
    UINT32  uiINTN;
    INT32   iGpioSoc = gpioSysToSoc(uiNum);

    if (iGpioSoc == PX_ERROR) {
        return  (PX_ERROR);
    }

    if (iGpioSoc >= GPIO_NUM(GPIO_PORT_B, GPIO_PIN_00) &&
        iGpioSoc <= GPIO_NUM(GPIO_PORT_B, GPIO_PIN_07)) {
        uiINTN = iGpioSoc - GPIO_NUM(GPIO_PORT_B, GPIO_PIN_00);          /*  GPIOB中断EINT0-7           */
        ulAddr = GPIO_B_INT_STA + GPIO_BASE;                             /*  中断状态寄存器地址         */
    } else if (iGpioSoc >= GPIO_NUM(GPIO_PORT_G, GPIO_PIN_00) &&
               iGpioSoc <= GPIO_NUM(GPIO_PORT_G, GPIO_PIN_13)) {
        uiINTN = iGpioSoc - GPIO_NUM(GPIO_PORT_G, GPIO_PIN_00);          /*  GPIOG中断EINT0-13          */
        ulAddr = GPIO_G_INT_STA + GPIO_BASE;                             /*  中断状态寄存器地址         */
    } else {
        return  (PX_ERROR);;                                                          /*  该端口没有中断功能         */
    }

    uiRegister = readl(ulAddr);
    if (uiRegister & (BIT(uiINTN))) {
        return  (LW_IRQ_HANDLED);
    } else {
        return  (LW_IRQ_NONE);
    }
}

/*********************************************************************************************************
** 函数名称: sunxiR16GpioGetIrq
** 功能描述: 获取GPIO中断号
** 输  入  : pgchip      GPIO 芯片
**           uiNum       GPIO在系统中的编号
**           bIsLevel    是否为电平触发, 1 表示电平触发，0 表示边沿触发
**           uiType      如果为电平触发, 1 表示高电平触发, 0 表示低电平触发
**                       如果为边沿触发, 1 表示上升沿触发, 0 表示下降沿触发, 2 双边沿触发
** 输  出  : IRQ 向量号 -1:错误
*********************************************************************************************************/
static ULONG  sunxiR16GpioGetIrq (PLW_GPIO_CHIP  pgchip, UINT  uiNum,
                            BOOL  bIsLevel, UINT  uiType)
{
    UINT32  uiPortNum;
    INT32   iGpioSoc = gpioSysToSoc(uiNum);

    if (iGpioSoc == PX_ERROR) {
        return  (-1);
    }

    /*
     *  计算组号
     */
    uiPortNum  = GET_SOC_PORT_NUM(iGpioSoc);

    if (uiPortNum == GPIO_PORT_B) {
        return  (PB_EINT);
    } else if (uiPortNum == GPIO_PORT_G) {
        return  (PG_EINT);
    } else {
        return  (-1);
    }
}

/*********************************************************************************************************
** 函数名称: gpioDrvInstall
** 功能描述: 安装 GPIO 驱动
** 输  入  : NONE
** 输  出  : ERROR_CODE
*********************************************************************************************************/
INT  gpioDrvInstall (VOID)
{
    static LW_GPIO_CHIP  t3GpioChip = {
        .GC_pcLabel              = "sunxi R16 GPIO",
        .GC_ulVerMagic           = LW_GPIO_VER_MAGIC,
        .GC_uiBase               = 0,
        .GC_uiNGpios             = GPIO_AMOUNT,
        .GC_pfuncRequest         = sunxiR16GpioRequest,
        .GC_pfuncFree            = sunxiR16GpioFree,
        .GC_pfuncGetDirection    = sunxiR16GpioGetDirection,
        .GC_pfuncDirectionInput  = sunxiR16GpioDirectionInput,
        .GC_pfuncGet             = sunxiR16GpioGet,
        .GC_pfuncDirectionOutput = sunxiR16GpioDirectionOutput,
        .GC_pfuncSetDebounce     = LW_NULL,
        .GC_pfuncSetPull         = LW_NULL,
        .GC_pfuncSet             = sunxiR16GpioSet,
        .GC_pfuncSetupIrq        = sunxiR16GpioSetupIrq,
        .GC_pfuncClearIrq        = sunxiR16GpioClearIrq,
        .GC_pfuncSvrIrq          = sunxiR16GpioSvrIrq,
        .GC_pfuncGetIrq          = sunxiR16GpioGetIrq,
    };

    return  (API_GpioChipAdd(&t3GpioChip));
}
/*********************************************************************************************************
  END
*********************************************************************************************************/
