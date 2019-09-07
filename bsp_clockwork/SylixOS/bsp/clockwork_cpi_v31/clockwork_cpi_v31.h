/*
 * clockwork_cpi_v31.h
 *
 *  Created on: Jun 3, 2019
 *      Author: databus
 */

#ifndef SYLIXOS_BSP_CLOCKWORK_CPI_V31_CLOCKWORK_CPI_V31_H_
#define SYLIXOS_BSP_CLOCKWORK_CPI_V31_CLOCKWORK_CPI_V31_H_

/*********************************************************************************************************
  板级包名称
*********************************************************************************************************/
#define BSP_CFG_PLATFORM_NAME               "ClockworkPI (CPI v3.1) development board"

/*********************************************************************************************************
  标准输入输出配置
*********************************************************************************************************/
#define BSP_CFG_STD_FILE                    "/dev/ttyS0"
#define BSP_CFG_DEBUG_CH                    0                           /* 调试信息输出通道             */

/*********************************************************************************************************
  CPU 相关
*********************************************************************************************************/
#define ARM_SW_INT_VECTOR(cpuid)            cpuid                       /*  软件产生中断 ID             */
#define ARM_SW_INT_PRIORITY                 0                           /*  软件产生中断优先级          */
#define ARM_TICK_INT_VECTOR                 50                          /*  TICK 中断 ID                */
#define ARM_TICK_INT_PRIORITY               1                           /*  TICK 中断优先级             */
#define ARM_DEFAULT_INT_PRIORITY            127                         /*  默认的中断优先级            */
#define BSP_CFG_CPU_NUM                     4                           /*  CPU数                       */
/*********************************************************************************************************
  ROM RAM 相关配置
*********************************************************************************************************/
#define BSP_CFG_ROM_BASE                    (0x00000000)
#define BSP_CFG_ROM_SIZE                    (4 * 1024 * 1024)           /*  保留 4M 空间                */

#define BSP_CFG_RAM_START                   0x40000000                  /*  内存基址                    */
#define BSP_CFG_RAM_BASE                    (BSP_CFG_RAM_START)
#define BSP_CFG_RAM_SIZE                    (1 * 1024 * 1024 * 1024)

#define BSP_CFG_TEXT_SIZE                   (6   * 1024 * 1024)
#define BSP_CFG_DATA_SIZE                   (121 * 1024 * 1024)

#define BSP_CFG_DMA_SIZE                    (128 * 1024 * 1024)
#define BSP_CFG_APP_SIZE                    (BSP_CFG_RAM_SIZE  - BSP_CFG_TEXT_SIZE - \
                                             BSP_CFG_DATA_SIZE - BSP_CFG_DMA_SIZE)

#define BSP_CFG_BOOT_STACK_SIZE             (4 * 128 * 1024)

/*********************************************************************************************************
  虚拟内存空间 相关配置
*********************************************************************************************************/
#define BSP_CFG_VAPP_START                  0x80000000
#define BSP_CFG_VAPP_SIZE                   (1 * LW_CFG_GB_SIZE)

#define BSP_CFG_VIO_START                   0xC0000000
#define BSP_CFG_VIO_SIZE                    (64  * LW_CFG_MB_SIZE)

/*********************************************************************************************************
  根文件系统挂载设备配置(只能选择一种)
*********************************************************************************************************/
#define BSP_CFG_NAND_ROOTFS_EN              0                           /*  用 NAND 作为根文件系统存储  */
#define BSP_CFG_SD_ROOTFS_EN                0                           /*  用 SD 卡作为根文件系统存储  */
#define BSP_CFG_HDD_ROOTFS_EN               0                           /*  用硬盘作为根文件系统存储    */
#define BSP_CFG_RAM_ROOTFS_EN               1                           /*  用内存做存储(通常为测试)    */

/*********************************************************************************************************
  当确定了以上的根文件系统存储设备后
  还需进行下面的配置:
  BSP_CFG_BOOT_DIR: 存放系统启动所需内容
  BSP_CFG_WORK_DIR: 系统主工作目录
  建议把存储设备一般分两个区, 一个区专门存放于boot有关的数据, 另一个区为工作区
  当然也可以只有一个分区, 则只需将下面对应的两个宏定义为相同即可
*********************************************************************************************************/
#if BSP_CFG_NAND_ROOTFS_EN > 0
#define BSP_CFG_FS_DIR                      "rfsmap=/boot:/yaffs2/n0,/:/yaffs2/n1"
#endif

#if BSP_CFG_SD_ROOTFS_EN > 0
#define BSP_CFG_FS_DIR                      "rfsmap=/boot:/media/sdcard0,/:/media/sdcard1"
#endif

#if BSP_CFG_HDD_ROOTFS_EN > 0
#define BSP_CFG_FS_DIR                      "rfsmap=/boot:/media/hdd0,/:/media/hdd1"
#endif

#if BSP_CFG_RAM_ROOTFS_EN > 0
#define BSP_CFG_FS_DIR                      "rfsmap=/:/dev/ram"
#endif
/*********************************************************************************************************
**  启动参数中文件系统挂载配置
*********************************************************************************************************/
#define MOUNT_PARAMETERS                    BSP_CFG_FS_DIR
/*********************************************************************************************************
  内核启动参数相关定义
*********************************************************************************************************/
#define __STR__(x)                          __STRING(x)
#if defined(BSP_CFG_CPU_NUM)
#define NCPUS                              "ncpus=" __STR__(BSP_CFG_CPU_NUM)
#else
#define NCPUS                              "ncpus=4"
#endif

#endif /* SYLIXOS_BSP_CLOCKWORK_CPI_V31_CLOCKWORK_CPI_V31_H_ */
