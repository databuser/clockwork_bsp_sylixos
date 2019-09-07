/*
 * timer.h
 *
 *  Created on: Jun 6, 2019
 *      Author: databus
 */

#ifndef SYLIXOS_DRIVER_TIMER_TIMER_H_
#define SYLIXOS_DRIVER_TIMER_TIMER_H_

VOID  timerStart(INT32  iNum, UINT32  uiHZ);
VOID  timerStop(INT32  iNum);
VOID  timerClearirq(INT32  iNum);
BOOL  timerIsIrqPending(INT32  iNum);
UINT32  timerCurGet(INT32  iNum);
UINT32  timerInputFreqGet(INT32  iNum);

#endif /* SYLIXOS_DRIVER_TIMER_TIMER_H_ */
