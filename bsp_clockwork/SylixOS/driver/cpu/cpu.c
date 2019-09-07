/*
 * cpu.c
 *
 *  Created on: Jun 6, 2019
 *      Author: databus
 */

#define __SYLIXOS_KERNEL
#include "config.h"
#include "SylixOS.h"
#include <linux/compat.h>
/*********************************************************************************************************
  基地址定义
*********************************************************************************************************/
#define R_PRCM_BASE                     (0x01f01400)
#define R_CPUCFG_BASE                   (0x01f01C00)
/*********************************************************************************************************
  控制寄存器偏移量定义
*********************************************************************************************************/
#define CPUCFG_CPU_PWR_CLAMP_STATUS_REG(cpu)    ((cpu) * 0x40 + 0x64)
#define CPUCFG_CPU_RST_CTRL_REG(cpu)            (((cpu) + 1) * 0x40)
#define CPUCFG_CPU_CTRL_REG(cpu)                (((cpu) + 1) * 0x40 + 0x04)
#define CPUCFG_CPU_STATUS_REG(cpu)              (((cpu) + 1) * 0x40 + 0x08)
#define CPUCFG_GEN_CTRL_REG                     0x184
#define CPUCFG_PRIVATE0_REG                     0x1a4
#define CPUCFG_PRIVATE1_REG                     0x1a8

#define PRCM_CPU_PWROFF_REG                     0x100
#define PRCM_CPU_PWR_CLAMP_REG(cpu)             (((cpu) * 4) + 0x140)

#define CNT64_CTRL_REG                          (0x280)
#define CNT64_LOW_REG                           (0x284)
#define CNT64_HIGH_REG                          (0x288)
#define MAX_CORE_COUNT                          (4)

/*
 *  Start up a secondary CPU core.
 */
VOID  cpuStart2nd (UINT8 ucCoreNum)
{
    UINT32  uiValue;

    /*
     * Exit if the requested core is not available.
     */
    if (ucCoreNum == 0 || ucCoreNum >= MAX_CORE_COUNT) {
        return;
    }

    writel(BSP_CFG_RAM_BASE, R_CPUCFG_BASE + CPUCFG_PRIVATE0_REG);      /*  Set CPU boot address        */

    writel(0, R_CPUCFG_BASE + CPUCFG_CPU_RST_CTRL_REG(ucCoreNum));      /*  Assert the CPU core in reset*/

    uiValue = readl(R_CPUCFG_BASE + CPUCFG_GEN_CTRL_REG);
    writel(uiValue & ~BIT(ucCoreNum), R_CPUCFG_BASE + CPUCFG_GEN_CTRL_REG);
                                                                        /*  Assert the L1 cache in reset*/

    uiValue = readl(R_PRCM_BASE + PRCM_CPU_PWROFF_REG);
    writel(uiValue & ~BIT(ucCoreNum), R_PRCM_BASE + PRCM_CPU_PWROFF_REG);
                                                                        /*  Clear CPU power-off gating  */

    udelay(10);

    writel(3, R_CPUCFG_BASE + CPUCFG_CPU_RST_CTRL_REG(ucCoreNum));      /*  Deassert the CPU core reset */

}

/*
 *  Places a secondary CPU core in reset.
 */
VOID cpuDisable (UINT8 ucCoreNum)
{

}

/*********************************************************************************************************
  END
*********************************************************************************************************/
