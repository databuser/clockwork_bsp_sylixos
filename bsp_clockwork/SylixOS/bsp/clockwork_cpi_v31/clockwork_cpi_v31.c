/*
 * clockwork_cpi_v31.c
 *
 *  Created on: Jun 3, 2019
 *      Author: databus
 */
#define  __SYLIXOS_KERNEL
#include <SylixOS.h>
#include "clockwork_cpi_v31.h"
#include "driver/uart/uart.h"
#include "driver/sio/sio.h"
#include "driver/gpio/gpio.h"
#include "driver/pinmux/pinmux.h"
#include "driver/sd/sunxi_mmc.h"
#include "driver/display/display.h"

VOID  bspBoardTargetInit (VOID)
{
}

VOID  bspBoardTimeInit (VOID)
{
}

VOID  bspBoardPmInit (VOID)
{
}

/*
 *  初始化各种总线系统
 */
VOID  bspBoardBusInit (VOID)
{
    API_I2cLibInit();                                                   /*  初始化 i2c 组件库           */

    API_SpiLibInit();                                                   /*  初始化 spi 组件库           */
}

/*
 *  驱动初始化
 */
VOID  bspBoardDrvInit (VOID)
{
    gpioDrvInstall();
}

/*
 *  设备初始化
 */
VOID bspBoardDevInit (VOID)
{
    SIO_CHAN  *psio;

    psio = sioChanCreate(0);                                            /*  创建串口 0 通道             */
    if (psio) {
        ttyDevCreate("/dev/ttyS0", psio, 1024, 1024);                   /*  增加 tty 设备               */
    }

    lcdDevCreate();

    sdDrvInstall(0, GPIO_B_03, GPIO_NONE, TRUE);
}

/*
 *  nandflash设备初始化
 */
VOID  bspBoardNandInit (VOID)
{
}

/*
 *  网络初始化
 */
VOID  bspBoardNetifAttch (VOID)
{
}

VOID  bspBoardDebugChInit (UINT  uiIndex)
{
}

/*
 *  打印调试信息
 */
VOID  bspBoardDebugMsg (CPCHAR  cpcMsg)
{
    CHAR cChar;

    if (!cpcMsg) {                                                       /*  指针为空                    */
        return;
    }

    while ((cChar = *cpcMsg) != '\0') {                                  /*  发送字符串                  */
        uartPutChar(BSP_CFG_DEBUG_CH, cChar);
        cpcMsg++;
    }
}

VOID  bspBoardSymbolInit (VOID)
{
}

/*
 *  设置启动参数
 */
VOID  bspBoardkernelStartParam (VOID)
{
    API_KernelStartParam(NCPUS
                         " sldepcache=yes kdlog=no kderror=no kfpu=no heapchk=yes hz=1000 hhz=1000 "
                         MOUNT_PARAMETERS);                             /*  操作系统启动参数设置        */
}
