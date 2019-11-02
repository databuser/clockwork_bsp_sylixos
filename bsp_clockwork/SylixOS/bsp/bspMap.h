/*********************************************************************************************************
**
**                                    中国软件开源组织
**
**                                   嵌入式实时操作系统
**
**                                       SylixOS(TM)
**
**                               Copyright  All Rights Reserved
**
**--------------文件信息--------------------------------------------------------------------------------
**
** 文   件   名: bspMap.h
**
** 创   建   人: Han.Hui (韩辉)
**
** 文件创建日期: 2008 年 12 月 23 日
**
** 描        述: C 程序入口部分. 物理分页空间与全局映射关系表定义.
*********************************************************************************************************/

#ifndef __BSPMAP_H
#define __BSPMAP_H

/*********************************************************************************************************
   内存分配关系图

    +-----------------------+--------------------------------+
    |       通用内存区      |          VMM 管理区            |
    |         CACHE         |                                |
    +-----------------------+--------------------------------+

*********************************************************************************************************/
/*********************************************************************************************************
  physical memory
*********************************************************************************************************/
#ifdef  __BSPINIT_MAIN_FILE

LW_MMU_PHYSICAL_DESC    _G_physicalDesc[] = {
    {                                                                   /*  中断向量表                  */
        BSP_CFG_RAM_BASE,
        0,
        LW_CFG_VMM_PAGE_SIZE,
        LW_PHYSICAL_MEM_VECTOR
    },

    {                                                                   /*  内核代码段                  */
        BSP_CFG_RAM_BASE,
        BSP_CFG_RAM_BASE,
        BSP_CFG_TEXT_SIZE,
        LW_PHYSICAL_MEM_TEXT
    },

    {                                                                   /*  内核数据段                  */
        BSP_CFG_RAM_BASE + BSP_CFG_TEXT_SIZE,
        BSP_CFG_RAM_BASE + BSP_CFG_TEXT_SIZE,
        BSP_CFG_DATA_SIZE,
        LW_PHYSICAL_MEM_DATA
    },

    {                                                                   /*  DMA 缓冲区                  */
        BSP_CFG_RAM_BASE + BSP_CFG_TEXT_SIZE + BSP_CFG_DATA_SIZE,
        BSP_CFG_RAM_BASE + BSP_CFG_TEXT_SIZE + BSP_CFG_DATA_SIZE,
        BSP_CFG_DMA_SIZE,
        LW_PHYSICAL_MEM_DMA
    },

    {                                                                   /*  APP 通用内存                */
        BSP_CFG_RAM_BASE + BSP_CFG_TEXT_SIZE + BSP_CFG_DATA_SIZE + BSP_CFG_DMA_SIZE,
        BSP_CFG_RAM_BASE + BSP_CFG_TEXT_SIZE + BSP_CFG_DATA_SIZE + BSP_CFG_DMA_SIZE,
        BSP_CFG_APP_SIZE,
        LW_PHYSICAL_MEM_APP
    },

    {                                                                   /*  system control              */
        0x01c00000,
        0x01c00000,
        LW_CFG_VMM_PAGE_SIZE,
        LW_PHYSICAL_MEM_BOOTSFR
    },

    {                                                                   /*  LCD                         */
        0x1c0c000,
        0x1c0c000,
        LW_CFG_VMM_PAGE_SIZE,
        LW_PHYSICAL_MEM_BOOTSFR
    },

    {                                                                   /*  UART0 ~ 4                   */
        0x01C28000,
        0x01C28000,
        2 * LW_CFG_VMM_PAGE_SIZE,
        LW_PHYSICAL_MEM_BOOTSFR
    },

    {                                                                   /*  USB OTG                     */
        0x01c19000,
        0x01c19000,
        LW_CFG_VMM_PAGE_SIZE,
        LW_PHYSICAL_MEM_BOOTSFR
    },

    {                                                                   /*  USB OHCI0                   */
        0x01c1a000,
        0x01c1a000,
        LW_CFG_VMM_PAGE_SIZE,
        LW_PHYSICAL_MEM_BOOTSFR
    },

    {                                                                   /*  CCU,PIO,TIMER               */
        0x01c20000,
        0x01c20000,
        LW_CFG_VMM_PAGE_SIZE,
        LW_PHYSICAL_MEM_BOOTSFR
    },

    {                                                                   /*  R_PIO                       */
        0x01f02000,
        0x01f02000,
        LW_CFG_VMM_PAGE_SIZE,
        LW_PHYSICAL_MEM_BOOTSFR
    },

    {                                                                   /*  GIC                         */
        0x01C80000,
        0x01C80000,
        (LW_CFG_VMM_PAGE_SIZE * 4),
        LW_PHYSICAL_MEM_BOOTSFR
    },

    {                                                                   /*  DEBE                         */
        0x01E60000,
        0x01E60000,
        (LW_CFG_VMM_PAGE_SIZE * 16),
        LW_PHYSICAL_MEM_BOOTSFR
    },

    {                                                                   /*  R_CPUCFG                    */
        0x01f01000,
        0x01f01000,
        LW_CFG_VMM_PAGE_SIZE,
        LW_PHYSICAL_MEM_BOOTSFR
    },

    {                                                                   /*  结束                        */
        0,
        0,
        0,
        0
    }
};
/*********************************************************************************************************
  virtual memory
*********************************************************************************************************/
LW_MMU_VIRTUAL_DESC    _G_virtualDesc[] = {
    {                                                                   /*  应用程序虚拟空间            */
        BSP_CFG_VAPP_START,
        BSP_CFG_VAPP_SIZE,
        LW_VIRTUAL_MEM_APP
    },
    {
        BSP_CFG_VIO_START,                                              /*  ioremap 空间                */
        BSP_CFG_VIO_SIZE,
        LW_VIRTUAL_MEM_DEV
    },
    {                                                                   /*  结束                        */
        0,
        0,
        0
    }
};

#endif                                                                  /*  __BSPINIT_MAIN_FILE         */
#endif                                                                  /*  __BSPMAP_H                  */
/*********************************************************************************************************
  END
*********************************************************************************************************/
