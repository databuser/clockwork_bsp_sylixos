/*
 * uart.c
 *
 *  Created on: Jun 6, 2019
 *      Author: databus
 */
#define  __SYLIXOS_KERNEL
#include <SylixOS.h>
#include <linux/compat.h>
/*********************************************************************************************************
  基地址定义
*********************************************************************************************************/
#define UART0_BASE            (0x01c28000)
#define UART1_BASE            (0x01c28400)
#define UART2_BASE            (0x01c28800)
#define UART3_BASE            (0x01c28c00)
#define UART4_BASE            (0x01c29000)
/*********************************************************************************************************
  寄存器偏移
*********************************************************************************************************/
#define RBR                   0x0
#define THR                   0x0
#define USR                   0x7C
/*********************************************************************************************************
** 函数名称: uartPutChar
** 功能描述: 向通道 uiChann 发送一个字节
** 输　入  : uiChann        通道号
**           ucChar         数据
** 输　出  : NONE
*********************************************************************************************************/
VOID  uartPutChar (UINT32  uiChann, UINT8  ucChar)
{
    addr_t atUartBase;

    switch (uiChann) {
    case 0:
        atUartBase = UART0_BASE;
        break;

    case 1:
        atUartBase = UART1_BASE;
        break;

    case 2:
        atUartBase = UART2_BASE;
        break;

    case 3:
        atUartBase = UART3_BASE;
        break;

    default:
        return;
    }

    /*
     * 若 FIFO 不满就填入数据，否则等待
     */
    while (!(readl(atUartBase + USR) & BIT(1)));

    writel(ucChar, atUartBase + THR);
}
/*********************************************************************************************************
** 函数名称: uartGetChar
** 功能描述: 从通道 uiChann 接收一个字节
** 输　入  : uiChann         通道号
** 输　出  : 接收到的数据
*********************************************************************************************************/
UINT8  uartGetChar (UINT32  uiChann)
{
    addr_t atUartBase;
    UINT8  cChar;

    switch (uiChann) {
    case 0:
        atUartBase = UART0_BASE;
        break;

    case 1:
        atUartBase = UART1_BASE;
        break;

    case 2:
        atUartBase = UART2_BASE;
        break;

    case 3:
        atUartBase = UART3_BASE;
        break;

    default:
        atUartBase = UART0_BASE;
        break;
    }

    /*
     * 若 FIFO 非空就读取一字节数据，否则等待
     */
    while (!(readl(atUartBase + USR) & BIT(3)));

    cChar = readl(atUartBase + RBR);

    return (cChar);
}

