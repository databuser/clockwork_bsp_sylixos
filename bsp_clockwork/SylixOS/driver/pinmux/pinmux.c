/*
 * pinmux.c
 *
 *  Created on: Jun 9, 2019
 *      Author: databus
 */

#define __SYLIXOS_KERNEL
#include <SylixOS.h>
#include <linux/compat.h>
#include "driver/pinmux/pinmux.h"

/*********************************************************************************************************
 SylixOS GPIO编号和R16芯片GPIO编号映射表
*********************************************************************************************************/
static const  UINT16  _G_usSunxiR16Gpios[] = {
    GPIO_NUM(GPIO_PORT_B, GPIO_PIN_00),
    GPIO_NUM(GPIO_PORT_B, GPIO_PIN_01),
    GPIO_NUM(GPIO_PORT_B, GPIO_PIN_02),
    GPIO_NUM(GPIO_PORT_B, GPIO_PIN_03),
    GPIO_NUM(GPIO_PORT_B, GPIO_PIN_04),
    GPIO_NUM(GPIO_PORT_B, GPIO_PIN_05),
    GPIO_NUM(GPIO_PORT_B, GPIO_PIN_06),
    GPIO_NUM(GPIO_PORT_B, GPIO_PIN_07),
    GPIO_NUM(GPIO_PORT_C, GPIO_PIN_00),
    GPIO_NUM(GPIO_PORT_C, GPIO_PIN_01),
    GPIO_NUM(GPIO_PORT_C, GPIO_PIN_02),
    GPIO_NUM(GPIO_PORT_C, GPIO_PIN_03),
    GPIO_NUM(GPIO_PORT_C, GPIO_PIN_04),
    GPIO_NUM(GPIO_PORT_C, GPIO_PIN_05),
    GPIO_NUM(GPIO_PORT_C, GPIO_PIN_06),
    GPIO_NUM(GPIO_PORT_C, GPIO_PIN_07),
    GPIO_NUM(GPIO_PORT_C, GPIO_PIN_08),
    GPIO_NUM(GPIO_PORT_C, GPIO_PIN_09),
    GPIO_NUM(GPIO_PORT_C, GPIO_PIN_10),
    GPIO_NUM(GPIO_PORT_C, GPIO_PIN_11),
    GPIO_NUM(GPIO_PORT_C, GPIO_PIN_12),
    GPIO_NUM(GPIO_PORT_C, GPIO_PIN_13),
    GPIO_NUM(GPIO_PORT_C, GPIO_PIN_14),
    GPIO_NUM(GPIO_PORT_C, GPIO_PIN_15),
    GPIO_NUM(GPIO_PORT_C, GPIO_PIN_16),
    GPIO_NUM(GPIO_PORT_D, GPIO_PIN_00),
    GPIO_NUM(GPIO_PORT_D, GPIO_PIN_01),
    GPIO_NUM(GPIO_PORT_D, GPIO_PIN_02),
    GPIO_NUM(GPIO_PORT_D, GPIO_PIN_03),
    GPIO_NUM(GPIO_PORT_D, GPIO_PIN_04),
    GPIO_NUM(GPIO_PORT_D, GPIO_PIN_05),
    GPIO_NUM(GPIO_PORT_D, GPIO_PIN_06),
    GPIO_NUM(GPIO_PORT_D, GPIO_PIN_07),
    GPIO_NUM(GPIO_PORT_D, GPIO_PIN_08),
    GPIO_NUM(GPIO_PORT_D, GPIO_PIN_09),
    GPIO_NUM(GPIO_PORT_D, GPIO_PIN_10),
    GPIO_NUM(GPIO_PORT_D, GPIO_PIN_11),
    GPIO_NUM(GPIO_PORT_D, GPIO_PIN_12),
    GPIO_NUM(GPIO_PORT_D, GPIO_PIN_13),
    GPIO_NUM(GPIO_PORT_D, GPIO_PIN_14),
    GPIO_NUM(GPIO_PORT_D, GPIO_PIN_15),
    GPIO_NUM(GPIO_PORT_D, GPIO_PIN_16),
    GPIO_NUM(GPIO_PORT_D, GPIO_PIN_17),
    GPIO_NUM(GPIO_PORT_D, GPIO_PIN_18),
    GPIO_NUM(GPIO_PORT_D, GPIO_PIN_19),
    GPIO_NUM(GPIO_PORT_D, GPIO_PIN_20),
    GPIO_NUM(GPIO_PORT_D, GPIO_PIN_21),
    GPIO_NUM(GPIO_PORT_E, GPIO_PIN_00),
    GPIO_NUM(GPIO_PORT_E, GPIO_PIN_01),
    GPIO_NUM(GPIO_PORT_E, GPIO_PIN_02),
    GPIO_NUM(GPIO_PORT_E, GPIO_PIN_03),
    GPIO_NUM(GPIO_PORT_E, GPIO_PIN_04),
    GPIO_NUM(GPIO_PORT_E, GPIO_PIN_05),
    GPIO_NUM(GPIO_PORT_E, GPIO_PIN_06),
    GPIO_NUM(GPIO_PORT_E, GPIO_PIN_07),
    GPIO_NUM(GPIO_PORT_E, GPIO_PIN_08),
    GPIO_NUM(GPIO_PORT_E, GPIO_PIN_09),
    GPIO_NUM(GPIO_PORT_E, GPIO_PIN_10),
    GPIO_NUM(GPIO_PORT_E, GPIO_PIN_11),
    GPIO_NUM(GPIO_PORT_E, GPIO_PIN_12),
    GPIO_NUM(GPIO_PORT_E, GPIO_PIN_13),
    GPIO_NUM(GPIO_PORT_E, GPIO_PIN_14),
    GPIO_NUM(GPIO_PORT_E, GPIO_PIN_15),
    GPIO_NUM(GPIO_PORT_E, GPIO_PIN_16),
    GPIO_NUM(GPIO_PORT_E, GPIO_PIN_17),
    GPIO_NUM(GPIO_PORT_F, GPIO_PIN_00),
    GPIO_NUM(GPIO_PORT_F, GPIO_PIN_01),
    GPIO_NUM(GPIO_PORT_F, GPIO_PIN_02),
    GPIO_NUM(GPIO_PORT_F, GPIO_PIN_03),
    GPIO_NUM(GPIO_PORT_F, GPIO_PIN_04),
    GPIO_NUM(GPIO_PORT_F, GPIO_PIN_05),
    GPIO_NUM(GPIO_PORT_G, GPIO_PIN_00),
    GPIO_NUM(GPIO_PORT_G, GPIO_PIN_01),
    GPIO_NUM(GPIO_PORT_G, GPIO_PIN_02),
    GPIO_NUM(GPIO_PORT_G, GPIO_PIN_03),
    GPIO_NUM(GPIO_PORT_G, GPIO_PIN_04),
    GPIO_NUM(GPIO_PORT_G, GPIO_PIN_05),
    GPIO_NUM(GPIO_PORT_G, GPIO_PIN_06),
    GPIO_NUM(GPIO_PORT_G, GPIO_PIN_07),
    GPIO_NUM(GPIO_PORT_G, GPIO_PIN_08),
    GPIO_NUM(GPIO_PORT_G, GPIO_PIN_09),
    GPIO_NUM(GPIO_PORT_G, GPIO_PIN_10),
    GPIO_NUM(GPIO_PORT_G, GPIO_PIN_11),
    GPIO_NUM(GPIO_PORT_G, GPIO_PIN_12),
    GPIO_NUM(GPIO_PORT_G, GPIO_PIN_13),
    GPIO_NUM(GPIO_PORT_H, GPIO_PIN_00),
    GPIO_NUM(GPIO_PORT_H, GPIO_PIN_01),
    GPIO_NUM(GPIO_PORT_H, GPIO_PIN_02),
    GPIO_NUM(GPIO_PORT_H, GPIO_PIN_03),
    GPIO_NUM(GPIO_PORT_H, GPIO_PIN_04),
    GPIO_NUM(GPIO_PORT_H, GPIO_PIN_05),
    GPIO_NUM(GPIO_PORT_H, GPIO_PIN_06),
    GPIO_NUM(GPIO_PORT_H, GPIO_PIN_07),
    GPIO_NUM(GPIO_PORT_H, GPIO_PIN_08),
    GPIO_NUM(GPIO_PORT_H, GPIO_PIN_09),
};
/*********************************************************************************************************
** 函数名称: gpioSysToSoc
** 功能描述: 将GPIO 在系统中的编号转换为SOC中的pin编号和port编号
** 输  入  : uiNum    GPIO 在系统中的编号
** 输  出  : uiOffset  低5位是pin编号，高位为port编号;不合法则为PX_ERROR
*********************************************************************************************************/
INT32  gpioSysToSoc (UINT32  uiNum)
{
   if (uiNum < (sizeof(_G_usSunxiR16Gpios) / sizeof(_G_usSunxiR16Gpios[0]))) {
       return  (_G_usSunxiR16Gpios[uiNum]);
   }

   return  (PX_ERROR);
}
/*********************************************************************************************************
** 函数名称: GpioPinmuxSet
** 功能描述: 设置 GPIO 管脚的功能选择
** 输  入  : uiNum    GPIO 在系统中的编号
**           uiCfg       管脚功能设置 0～7
** 输  出  : 0: 正确 -1:错误
** 备    注: 引脚功能选择寄存器只能按4字节对齐地址访问
*********************************************************************************************************/
INT32  gpioPinmuxSet (UINT32  uiNum, UINT32 uiCfg)
{
    addr_t  ulAddr;
    UINT32  uiValue;
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

    /*
     *  根据组号和引脚号确定要操作哪个寄存器
     */
    ulAddr = ((uiPinNum / 8) * 4) + (uiPortNum * 0x24) + GPIO_BASE;

    /*
     *  设置新值
     */
    uiCfg   &= 0x00000007;
    uiValue  = readl(ulAddr);
    uiValue &= ~(0x0f << ((uiPinNum % 8) * 4));
    uiValue |= (uiCfg << ((uiPinNum % 8) * 4));
    writel(uiValue, ulAddr);

    return  (ERROR_NONE);
}
/*********************************************************************************************************
** 函数名称: GpioPinmuxGet
** 功能描述: 读取 GPIO 管脚的功能选择
** 输  入  : uiNum    GPIO 在系统中的编号
** 输  出  : 管脚功能设置 0～7,错误时返回-1
** 备    注: 引脚功能选择寄存器只能按4字节对齐地址访问
*********************************************************************************************************/
INT32  gpioPinmuxGet (UINT32  uiNum)
{
    addr_t  ulAddr;
    UINT32  uiValue;
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

    /*
     *  根据组号和引脚号确定要操作哪个寄存器
     */
    ulAddr = ((uiPinNum / 8) * 4) + (uiPortNum * 0x24) + GPIO_BASE;

    uiValue  = readl(ulAddr);
    uiValue  = uiValue >> ((uiPinNum % 8) * 4);
    uiValue &= 0x00000007;

    return  (uiValue);
}
/*********************************************************************************************************
  END
*********************************************************************************************************/
