/*
 * display.c
 *
 *  Created on: Oct 5, 2019
 *      Author: Administrator
 *
 *      在R16中,DEFE没有用到,DEBE通过DMA获取显存(内存)数据,显存基址设置的是layer0基址
 *      DEBE设置的像素格式是4字节的XRGB,TCON设置的像素格式是RGB666
 *      uboot已经做好了控制器的初始化工作，这里仅仅重新设置新的显存基址
 */

#define __SYLIXOS_KERNEL
#include <SylixOS.h>
#include <linux/compat.h>
#include <driver/include/irq_numbers.h>
/*********************************************************************************************************
  全局变量
*********************************************************************************************************/
LW_GM_DEVICE            LCDD_gmdev;
LW_GM_FILEOPERATIONS    LCDD_gmfo;
PVOID                   LCDD_pvFbBase;
PVOID                   LCDD_pvFbBaseNew;

ULONG                   x_offset;
ULONG                   y_offset;

PLW_JOB_QUEUE           irq_queue;

LW_HANDLE               mutex;
LW_THREAD_COND          thread_cond;

extern const unsigned char gImage_sylixos[307200];
/*********************************************************************************************************
  屏幕相关参数
*********************************************************************************************************/
#define LCD_XSIZE               (320)
#define LCD_YSIZE               (240)
#define BYTES_PER_PIX           (4)
/*********************************************************************************************************
  寄存器地址
*********************************************************************************************************/
#define DE_BE_REG               (0x1e60000)
#define DE_BE_LAYFB_L32ADD_REG  (0x850)
#define DE_BE_LAYFB_H4ADD_REG   (0x860)

#define TCON_REG                (0x1c0c000)
#define TCON_GINT0_REG          (0x4)

static INT  __lcdOpen (PLW_GM_DEVICE  pgmdev, INT  iFlag, INT  iMode)
{
    return  (ERROR_NONE);
}

static INT  __lcdClose (PLW_GM_DEVICE  pgmdev)
{
    return  (ERROR_NONE);
}

static INT  __lcdGetVarInfo (PLW_GM_DEVICE  pgmdev, PLW_GM_VARINFO  pgmvi)
{
    if (pgmvi) {
        pgmvi->GMVI_ulXRes = LCD_XSIZE;
        pgmvi->GMVI_ulYRes = LCD_YSIZE;

        pgmvi->GMVI_ulXResVirtual = LCD_XSIZE;
        pgmvi->GMVI_ulYResVirtual = LCD_YSIZE * 2;

        pgmvi->GMVI_ulXOffset = x_offset;
        pgmvi->GMVI_ulYOffset = y_offset;

        pgmvi->GMVI_ulBitsPerPixel  = BYTES_PER_PIX * 8;
        pgmvi->GMVI_ulBytesPerPixel = BYTES_PER_PIX;
        pgmvi->GMVI_ulGrayscale     = 0;

        pgmvi->GMVI_ulRedMask   = 0xFF0000;
        pgmvi->GMVI_ulGreenMask = 0xFF00;
        pgmvi->GMVI_ulBlueMask  = 0xFF;
        pgmvi->GMVI_ulTransMask = 0;

        pgmvi->GMVI_gmbfRed.GMBF_uiLength = 8;
        pgmvi->GMVI_gmbfRed.GMBF_uiOffset = 16;
        pgmvi->GMVI_gmbfRed.GMBF_uiMsbRight = 0;

        pgmvi->GMVI_gmbfGreen.GMBF_uiLength = 8;
        pgmvi->GMVI_gmbfGreen.GMBF_uiOffset = 8;
        pgmvi->GMVI_gmbfGreen.GMBF_uiMsbRight = 0;

        pgmvi->GMVI_gmbfBlue.GMBF_uiLength = 8;
        pgmvi->GMVI_gmbfBlue.GMBF_uiOffset = 0;
        pgmvi->GMVI_gmbfBlue.GMBF_uiMsbRight = 0;

        pgmvi->GMVI_gmbfTrans.GMBF_uiLength = 0;
        pgmvi->GMVI_gmbfTrans.GMBF_uiOffset = 0;
        pgmvi->GMVI_gmbfTrans.GMBF_uiMsbRight = 0;

        pgmvi->GMVI_bHardwareAccelerate = LW_FALSE;
        pgmvi->GMVI_ulMode              = LW_GM_SET_MODE;
        pgmvi->GMVI_ulStatus            = 0;
    }

    return  (ERROR_NONE);
}

static INT  __lcdGetScrInfo (PLW_GM_DEVICE  pgmdev, PLW_GM_SCRINFO  pgmsi)
{
    if (pgmsi) {
        pgmsi->GMSI_pcName    = "/dev/fb0";
        pgmsi->GMSI_ulId      = 0;
        pgmsi->GMSI_stMemSize        = LCD_XSIZE * LCD_YSIZE * BYTES_PER_PIX * 2;
        pgmsi->GMSI_stMemSizePerLine = LCD_XSIZE * BYTES_PER_PIX;
        pgmsi->GMSI_pcMem            = (caddr_t)LCDD_pvFbBaseNew;
    }

    return  (ERROR_NONE);
}

static VOID __setNewDisplayAddress (PVOID addr)
{
    addr_t pDebeRegBase = DE_BE_REG;
    addr_t pAddr = (addr_t)(addr - bspInfoRamBase());

    writel(pAddr << 3, pDebeRegBase + DE_BE_LAYFB_L32ADD_REG);
    writel(pAddr >> 29, pDebeRegBase + DE_BE_LAYFB_H4ADD_REG);
}

static INT  __lcdSetVarInfo(PLW_GM_DEVICE  pgmdev, const PLW_GM_VARINFO  pgmvi)
{
    PVOID  newDisplayAddr;

    x_offset = pgmvi->GMVI_ulXOffset;
    y_offset = pgmvi->GMVI_ulYOffset;

    if (pgmvi) {
        newDisplayAddr = LCDD_pvFbBase + (pgmvi->GMVI_ulXOffset * pgmvi->GMVI_ulBytesPerPixel +
                                          pgmvi->GMVI_ulYOffset * LCD_XSIZE * BYTES_PER_PIX);
        __setNewDisplayAddress(newDisplayAddr);
        LCDD_pvFbBaseNew = newDisplayAddr;
    }

    //wait vblank
    API_SemaphoreMPend(mutex, LW_OPTION_WAIT_INFINITE);
    API_ThreadCondWait(&thread_cond, mutex, LW_OPTION_WAIT_INFINITE);
    API_SemaphoreMPost(mutex);

    return  (ERROR_NONE);
}



static VOID  __debeInit (VOID)
{
    __setNewDisplayAddress(LCDD_pvFbBase);
}

void lcd_irq_process_bootm (void *arg)
{
    /*
     *  TODO:wakeup threads wait for vsync
     */
    API_SemaphoreMPend(mutex, LW_OPTION_WAIT_INFINITE);
    API_ThreadCondBroadcast(&thread_cond);
    API_SemaphoreMPost(mutex);
}

static irqreturn_t  __lcdIsr (VOID)
{
    INT iReg = readl(TCON_REG + TCON_GINT0_REG);

    //clear vblank interrupt
    writel(iReg & ~(0xffff), TCON_REG + TCON_GINT0_REG);

    /*
     * 扔到中断底半部处理
     */
    API_InterDeferJobAdd(irq_queue, lcd_irq_process_bootm, NULL);

    return LW_IRQ_HANDLED;
}
static VOID  __lcdInit (VOID)
{
    ULONG ulCondAttr;

    API_ThreadCondAttrInit(&ulCondAttr);
    API_ThreadCondAttrSetPshared(&ulCondAttr, LW_FALSE);
    API_ThreadCondInit(&thread_cond, ulCondAttr);
    API_ThreadCondAttrDestroy(&ulCondAttr);

    mutex =  API_SemaphoreMCreate("count_lock",
                                     LW_PRIO_HIGH,
                                     LW_OPTION_WAIT_FIFO |
                                     LW_OPTION_OBJECT_LOCAL|
                                     LW_OPTION_INHERIT_PRIORITY,
                                     LW_NULL);

    irq_queue = API_InterDeferGet(0);

    INT iReg = readl(TCON_REG + TCON_GINT0_REG);
    writel(iReg | (1 << 31), TCON_REG + TCON_GINT0_REG);

    API_InterVectorConnect(LCD,
                           (PINT_SVR_ROUTINE)__lcdIsr,
                           LW_NULL,
                           "vblank");

    API_InterVectorEnable(LCD);
}

INT  lcdDevCreate (VOID)
{
    LCDD_pvFbBase = API_VmmDmaAlloc(LCD_XSIZE * LCD_YSIZE * BYTES_PER_PIX * 2);
    if (LCDD_pvFbBase == LW_NULL) {
        return  (PX_ERROR);
    }
    LCDD_pvFbBaseNew = LCDD_pvFbBase;

    memcpy(LCDD_pvFbBase, gImage_sylixos, sizeof(gImage_sylixos));

    LCDD_gmfo.GMFO_pfuncOpen  = __lcdOpen;
    LCDD_gmfo.GMFO_pfuncClose = __lcdClose;
    LCDD_gmfo.GMFO_pfuncGetVarInfo = (INT (*)(LONG, PLW_GM_VARINFO))__lcdGetVarInfo;
    LCDD_gmfo.GMFO_pfuncSetVarInfo = (INT (*)(LONG, PLW_GM_VARINFO))__lcdSetVarInfo;
    LCDD_gmfo.GMFO_pfuncGetScrInfo = (INT (*)(LONG, PLW_GM_SCRINFO))__lcdGetScrInfo;
    LCDD_gmdev.GMDEV_gmfileop = &LCDD_gmfo;

    //set new framebuffer base address
    __debeInit();

#if 1
    //enable vblank interrupt
    __lcdInit();
#endif

    return  (gmemDevAdd("/dev/fb0", &LCDD_gmdev));
}
