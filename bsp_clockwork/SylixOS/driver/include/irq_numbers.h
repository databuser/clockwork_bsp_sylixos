/*
 * irq_numbers.h
 *
 *  Created on: Jun 3, 2019
 *      Author: databus
 */

#ifndef SYLIXOS_DRIVER_INCLUDE_IRQ_NUMBERS_H_
#define SYLIXOS_DRIVER_INCLUDE_IRQ_NUMBERS_H_

/*********************************************************************************************************
  Allwinner_R16 通用中断控制器的中断编号
*********************************************************************************************************/
#define SGI_0                         0
#define SGI_1                         1
#define SGI_2                         2
#define SGI_3                         3
#define SGI_4                         4
#define SGI_5                         5
#define SGI_6                         6
#define SGI_7                         7
#define SGI_8                         8
#define SGI_9                         9
#define SGI_10                        10
#define SGI_11                        11
#define SGI_12                        12
#define SGI_13                        13
#define SGI_14                        14
#define SGI_15                        15
#define PPI_0                         16
#define PPI_1                         17
#define PPI_2                         18
#define PPI_3                         19
#define PPI_4                         20
#define PPI_5                         21
#define PPI_6                         22
#define PPI_7                         23
#define PPI_8                         24
#define PPI_9                         25
#define PPI_10                        26
#define PPI_11                        27
#define PPI_12                        28
#define PPI_13                        29
#define PPI_14                        30
#define PPI_15                        31
#define UART0                         32
#define UART1                         33
#define UART2                         34
#define UART3                         35
#define UART4                         36

#define TWI0                          38
#define TWI1                          39
#define TWI2                          40

#define PA_EINT                       43

#define DAUDIO_0                      45
#define DAUDIO_1                      46
#define PB_EINT                       47

#define PG_EINT                       49
#define TIMER0                        50
#define TIMER1                        51

#define WATCHDOG                      57

#define AUDIO_CODEC                   61
#define KEYADC                        62
#define THERMAL_SENSOR                63
#define EXTERNAL_NMI                  64
#define R_TIMER0                      65
#define R_TIMER1                      66

#define R_WATCHDOG                    68

#define R_UART                        70
#define R_RSB                         71
#define R_ALARM0                      72
#define R_ALARM1                      73

#define R_TWI                         76
#define R_PL_LINT                     77
#define HMIC                          78

#define M_BOX                         81
#define DMA                           82
#define HS_TIMER                      83

#define VE                            90

#define SD_MMC0                       92
#define SD_MMC1                       93
#define SD_MMC2                       94

#define SPI0                          97
#define SPI1                          98

#define NAND                          102
#define USB_OTG                       103
#define USB_EHCI0                     104
#define USB_OHCI0                     105

#define CE                            112

#define CSI                           116
#define CSI_CCI                       117
#define LCD                           118

#define MIPI_DSI                      121

#define DRC_01                        123

#define DE_FE                         125

#define DE_BE                         127

#define GPU_GP                        129
#define GPU_GPMMU                     130
#define GPU_PP0                       131
#define GPU_PP0MMU0                   132
#define GPU_PMU                       133
#define GPU_PP1                       134
#define GPU_PPMMU1                    135

#endif /* SYLIXOS_DRIVER_INCLUDE_IRQ_NUMBERS_H_ */
