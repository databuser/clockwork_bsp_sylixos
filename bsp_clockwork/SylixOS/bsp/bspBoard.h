/*
 * bspBoard.h
 *
 *  Created on: Jun 3, 2019
 *      Author: databus
 */

#ifndef SYLIXOS_BSP_BSPBOARD_H_
#define SYLIXOS_BSP_BSPBOARD_H_

/*
 *  板级相关接口
 */
VOID  bspBoardTargetInit(VOID);
VOID  bspBoardTimeInit(VOID);
VOID  bspBoardPmInit(VOID);
VOID  bspBoardBusInit(VOID);
VOID  bspBoardDrvInit(VOID);
VOID  bspBoardDevInit(VOID);
VOID  bspBoardNandInit(VOID);
VOID  bspBoardNetifAttch(VOID);
VOID  bspBoardDebugChInit(UINT uiIndex);
VOID  bspBoardDebugMsg(CPCHAR cpcMsg);
VOID  bspBoardSymbolInit(VOID);
VOID  bspBoardkernelStartParam(VOID);

#endif /* SYLIXOS_BSP_BSPBOARD_H_ */
