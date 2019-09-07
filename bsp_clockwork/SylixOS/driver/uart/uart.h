/*
 * uart.h
 *
 *  Created on: Jun 6, 2019
 *      Author: databus
 */

#ifndef SYLIXOS_DRIVER_UART_UART_H_
#define SYLIXOS_DRIVER_UART_UART_H_

VOID   uartPutChar(UINT32 uiChann, UINT8 cChar);
UINT8  uartGetChar(UINT32 uiChann);

#endif /* SYLIXOS_DRIVER_UART_UART_H_ */
