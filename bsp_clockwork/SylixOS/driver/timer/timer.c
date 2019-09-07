/*
 * timer.c
 *
 *  Created on: Jun 6, 2019
 *      Author: databus
 */
#define  __SYLIXOS_KERNEL
#include <SylixOS.h>
#include <linux/compat.h>
#include "timer.h"
/*********************************************************************************************************
  timer 寄存器偏移
*********************************************************************************************************/
#define TIMER_BASE     0x01c20c00
#define TIMER_IRQ_EN   0x00
#define TIMER_IRQ_STA  0x04
#define TIMER0_CTRL    0x10
#define TIMER0_INTV    0x14
#define TIMER0_CUR     0x18
#define TIMER1_CTRL    0x20
#define TIMER1_INTV    0x24
#define TIMER1_CUR     0x28

#define WDT_IRQ_EN     0xA0
#define WDT_IRQ_STA    0xA4
#define WDT_CTRL       0xB0
#define WDT_CFG        0xB4
#define WDT_MODE       0xB8

/*********************************************************************************************************
  tick 定时器配置
*********************************************************************************************************/
#define TICK_TIMER_FREQ  (24 * 1000 * 1000 / 2)

/*********************************************************************************************************
** 函数名称: timerStart
** 功能描述: 启动 timer
** 输　入  : iNum  timer定时器编号
**           uiHZ  设置定时器中断频率
** 输　出  : NONE
*********************************************************************************************************/
VOID  timerStart (INT32  iNum, UINT32  uiHZ)
{
    UINT32  uiCount;
    UINT32  uiIntvOffset;
    UINT32  uiCtrlOffset;

    if (0 == iNum) {
        uiIntvOffset = TIMER0_INTV;
        uiCtrlOffset = TIMER0_CTRL;
    } else if (1 == iNum) {
        uiIntvOffset = TIMER1_INTV;
        uiCtrlOffset = TIMER1_CTRL;
    } else {
        return ;
    }

    uiCount  = TICK_TIMER_FREQ;
    uiCount /= uiHZ;
    writel(uiCount, TIMER_BASE + uiIntvOffset);
    writel(BIT(1) | BIT(2) | BIT(4), TIMER_BASE + uiCtrlOffset);
    while ((readl(TIMER_BASE + uiCtrlOffset) >> 1) & 0x01);
    writel(readl(TIMER_BASE + uiCtrlOffset) | BIT(0), TIMER_BASE + uiCtrlOffset);
    writel(readl(TIMER_BASE + TIMER_IRQ_EN) | BIT(iNum), TIMER_BASE + TIMER_IRQ_EN);
}
/*********************************************************************************************************
** 函数名称: timerStop
** 功能描述: 关闭 timer
** 输　入  : iNum  timer定时器编号
** 输　出  : NONE
*********************************************************************************************************/
VOID  timerStop (INT32  iNum)
{
    UINT32  uiCtrlOffset;

    if (0 == iNum) {
        uiCtrlOffset = TIMER0_CTRL;
    } else if (1 == iNum) {
        uiCtrlOffset = TIMER1_CTRL;
    } else {
        return ;
    }

    writel(readl(TIMER_BASE + uiCtrlOffset) & (~BIT(0)), TIMER_BASE + uiCtrlOffset);
}
/*********************************************************************************************************
** 函数名称: timerClearirq
** 功能描述: 清除 timer 中断标志
** 输　入  : iNum  timer定时器编号
** 输　出  : NONE
*********************************************************************************************************/
VOID  timerClearirq (INT32  iNum)
{
    if ((0 != iNum) && (1 != iNum))
        return ;

    writel(readl(TIMER_BASE + TIMER_IRQ_STA) | BIT(iNum), TIMER_BASE + TIMER_IRQ_STA);
}
/*********************************************************************************************************
** 函数名称: timerIsIrqPending
** 功能描述: 检测是否有timer中断产生
** 输　入  : iNum  timer定时器编号
** 输　出  : TRUE  : 有timer中断产生
**           FALSE : 没有timer中断产生
*********************************************************************************************************/
BOOL  timerIsIrqPending (INT32  iNum)
{
    if ((0 != iNum) && (1 != iNum))
        return  FALSE;

    return  (readl(TIMER_BASE + TIMER_IRQ_STA) & BIT(iNum)) ? TRUE : FALSE;
}
/*********************************************************************************************************
** 函数名称: timerCurGet
** 功能描述: 获取timer当前计数值
** 输　入  : iNum  timer定时器编号
** 输　出  : timer当前计数值
*********************************************************************************************************/
UINT32  timerCurGet (INT32  iNum)
{
    if ((0 != iNum) && (1 != iNum))
        return 0;

    return  readl(TIMER_BASE + (TIMER0_CUR + iNum * 0x10));
}
/*********************************************************************************************************
** 函数名称: timerInputFreqGet
** 功能描述: 获取timer模块输入频率
** 输　入  : iNum  timer定时器编号
** 输　出  : timer当前计数值
*********************************************************************************************************/
UINT32  timerInputFreqGet (INT32  iNum)
{
    if ((0 != iNum) && (1 != iNum))
        return 0;

    return  TICK_TIMER_FREQ;
}
