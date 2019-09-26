/*
 * sunxi_mmc.c
 *
 *  Created on: Sep 19, 2019
 *      Author: databus
 *
 *      this driver is port from linux
 */


#define __SYLIXOS_KERNEL
#include <SylixOS.h>
#include <linux/compat.h>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <system/device/sdcard/core/sdstd.h>
#include <system/device/sd/sdBus.h>
#include "driver/include/irq_numbers.h"
#include "driver/ccu/ccu.h"

/*********************************************************************************************************
  config
*********************************************************************************************************/
#ifdef DEBUG
#define SD_DBG(fmt, arg...)             _PrintFormat("[SDDBG]"fmt, ##arg);
#define SD_ERR(fmt, arg...)             _PrintFormat("[SDERR]"fmt, ##arg);
#else
#define SD_DBG(fmt, arg...)
#define SD_ERR(fmt, arg...)
#endif

#define USE_CPU_COPY_DATA               0
#define USE_DMA_WITH_CACHE_BUFFER       1
#define USE_DMA_WITH_NOCACHE_BUFFER     2
#define DATA_TRANSFER_OPTION            USE_DMA_WITH_CACHE_BUFFER

#define MEM_TO_DEVICE                   0
#define DEVICE_TO_MEM                   1

#define DMA_DATA_BUFFER_SIZE            1 * 1024 * 1024
/*********************************************************************************************************
  register define
*********************************************************************************************************/
#define __SDHOST_NAME                   "/bus/sd/0"
#define __SDHOST_CDNAME                 "sd_cd0"
#define __SDHOST_WPNAME                 "sd_wp0"
#define __SDHOST_MAXNUM                 3
#define __SDHOST_VECTOR_CH0             SD_MMC0
#define __SDHOST_PHYADDR_CH0            0x1c0f000
#define __SDHOST_VECTOR_CH1             SD_MMC1
#define __SDHOST_PHYADDR_CH1            0x1c0f000
#define __SDHOST_VECTOR_CH2             SD_MMC2
#define __SDHOST_PHYADDR_CH2            0x1c11000

#ifndef GPIO_NONE
#define GPIO_NONE                       LW_CFG_MAX_GPIOS
#endif

/* register offset definitions */
#define SDXC_REG_GCTRL  (0x00) /* SMC Global Control Register */
#define SDXC_REG_CLKCR  (0x04) /* SMC Clock Control Register */
#define SDXC_REG_TMOUT  (0x08) /* SMC Time Out Register */
#define SDXC_REG_WIDTH  (0x0C) /* SMC Bus Width Register */
#define SDXC_REG_BLKSZ  (0x10) /* SMC Block Size Register */
#define SDXC_REG_BCNTR  (0x14) /* SMC Byte Count Register */
#define SDXC_REG_CMDR   (0x18) /* SMC Command Register */
#define SDXC_REG_CARG   (0x1C) /* SMC Argument Register */
#define SDXC_REG_RESP0  (0x20) /* SMC Response Register 0 */
#define SDXC_REG_RESP1  (0x24) /* SMC Response Register 1 */
#define SDXC_REG_RESP2  (0x28) /* SMC Response Register 2 */
#define SDXC_REG_RESP3  (0x2C) /* SMC Response Register 3 */
#define SDXC_REG_IMASK  (0x30) /* SMC Interrupt Mask Register */
#define SDXC_REG_MISTA  (0x34) /* SMC Masked Interrupt Status Register */
#define SDXC_REG_RINTR  (0x38) /* SMC Raw Interrupt Status Register */
#define SDXC_REG_STAS   (0x3C) /* SMC Status Register */
#define SDXC_REG_FTRGL  (0x40) /* SMC FIFO Threshold Watermark Registe */
#define SDXC_REG_FUNS   (0x44) /* SMC Function Select Register */
#define SDXC_REG_CBCR   (0x48) /* SMC CIU Byte Count Register */
#define SDXC_REG_BBCR   (0x4C) /* SMC BIU Byte Count Register */
#define SDXC_REG_DBGC   (0x50) /* SMC Debug Enable Register */
#define SDXC_REG_HWRST  (0x78) /* SMC Card Hardware Reset for Register */
#define SDXC_REG_DMAC   (0x80) /* SMC IDMAC Control Register */
#define SDXC_REG_DLBA   (0x84) /* SMC IDMAC Descriptor List Base Addre */
#define SDXC_REG_IDST   (0x88) /* SMC IDMAC Status Register */
#define SDXC_REG_IDIE   (0x8C) /* SMC IDMAC Interrupt Enable Register */
#define SDXC_REG_CHDA   (0x90)
#define SDXC_REG_CBDA   (0x94)
#define SDXC_REG_FIFO   (0x200) /* FIFO access address */

/* New registers introduced in A64 */
#define SDXC_REG_A12A           0x058 /* SMC Auto Command 12 Register */
#define SDXC_REG_SD_NTSR        0x05C /* SMC New Timing Set Register */
#define SDXC_REG_DRV_DL         0x140 /* Drive Delay Control Register */
#define SDXC_REG_SAMP_DL_REG    0x144 /* SMC sample delay control */
#define SDXC_REG_DS_DL_REG      0x148 /* SMC data strobe delay control */

#define mmc_readl(host, reg) \
    readl((host)->SDCH_virAddr + SDXC_##reg)
#define mmc_writel(host, reg, value) \
    writel((value), (host)->SDCH_virAddr + SDXC_##reg)

/* global control register bits */
#define SDXC_SOFT_RESET             BIT(0)
#define SDXC_FIFO_RESET             BIT(1)
#define SDXC_DMA_RESET              BIT(2)
#define SDXC_INTERRUPT_ENABLE_BIT   BIT(4)
#define SDXC_DMA_ENABLE_BIT         BIT(5)
#define SDXC_DEBOUNCE_ENABLE_BIT    BIT(8)
#define SDXC_POSEDGE_LATCH_DATA     BIT(9)
#define SDXC_DDR_MODE               BIT(10)
#define SDXC_MEMORY_ACCESS_DONE     BIT(29)
#define SDXC_ACCESS_DONE_DIRECT     BIT(30)
#define SDXC_ACCESS_BY_AHB          BIT(31)
#define SDXC_ACCESS_BY_DMA          (0 << 31)
#define SDXC_HARDWARE_RESET \
    (SDXC_SOFT_RESET | SDXC_FIFO_RESET | SDXC_DMA_RESET)

/* clock control bits */
#define SDXC_MASK_DATA0             BIT(31)
#define SDXC_CARD_CLOCK_ON          BIT(16)
#define SDXC_LOW_POWER_ON           BIT(17)

/* bus width */
#define SDXC_WIDTH1                 0
#define SDXC_WIDTH4                 1
#define SDXC_WIDTH8                 2

/* smc command bits */
#define SDXC_RESP_EXPIRE            BIT(6)
#define SDXC_LONG_RESPONSE          BIT(7)
#define SDXC_CHECK_RESPONSE_CRC     BIT(8)
#define SDXC_DATA_EXPIRE            BIT(9)
#define SDXC_WRITE                  BIT(10)
#define SDXC_SEQUENCE_MODE          BIT(11)
#define SDXC_SEND_AUTO_STOP         BIT(12)
#define SDXC_WAIT_PRE_OVER          BIT(13)
#define SDXC_STOP_ABORT_CMD         BIT(14)
#define SDXC_SEND_INIT_SEQUENCE     BIT(15)
#define SDXC_UPCLK_ONLY             BIT(21)
#define SDXC_READ_CEATA_DEV         BIT(22)
#define SDXC_CCS_EXPIRE             BIT(23)
#define SDXC_ENABLE_BIT_BOOT        BIT(24)
#define SDXC_ALT_BOOT_OPTIONS       BIT(25)
#define SDXC_BOOT_ACK_EXPIRE        BIT(26)
#define SDXC_BOOT_ABORT             BIT(27)
#define SDXC_VOLTAGE_SWITCH         BIT(28)
#define SDXC_USE_HOLD_REGISTER      BIT(29)
#define SDXC_START                  BIT(31)

/* interrupt bits */
#define SDXC_RESP_ERROR             BIT(1)
#define SDXC_COMMAND_DONE           BIT(2)
#define SDXC_DATA_OVER              BIT(3)
#define SDXC_TX_DATA_REQUEST        BIT(4)
#define SDXC_RX_DATA_REQUEST        BIT(5)
#define SDXC_RESP_CRC_ERROR         BIT(6)
#define SDXC_DATA_CRC_ERROR         BIT(7)
#define SDXC_RESP_TIMEOUT           BIT(8)
#define SDXC_DATA_TIMEOUT           BIT(9)
#define SDXC_VOLTAGE_CHANGE_DONE    BIT(10)
#define SDXC_FIFO_RUN_ERROR         BIT(11)
#define SDXC_HARD_WARE_LOCKED       BIT(12)
#define SDXC_START_BIT_ERROR        BIT(13)
#define SDXC_AUTO_COMMAND_DONE      BIT(14)
#define SDXC_END_BIT_ERROR          BIT(15)
#define SDXC_SDIO_INTERRUPT         BIT(16)
#define SDXC_CARD_INSERT            BIT(30)
#define SDXC_CARD_REMOVE            BIT(31)
#define SDXC_INTERRUPT_ERROR_BIT \
    (SDXC_RESP_ERROR | SDXC_RESP_CRC_ERROR | SDXC_DATA_CRC_ERROR | \
     SDXC_RESP_TIMEOUT | SDXC_DATA_TIMEOUT | SDXC_FIFO_RUN_ERROR | \
     SDXC_HARD_WARE_LOCKED | SDXC_START_BIT_ERROR | SDXC_END_BIT_ERROR)
#define SDXC_INTERRUPT_DONE_BIT \
    (SDXC_AUTO_COMMAND_DONE | SDXC_DATA_OVER | \
     SDXC_COMMAND_DONE | SDXC_VOLTAGE_CHANGE_DONE)

/* status */
#define SDXC_RXWL_FLAG          BIT(0)
#define SDXC_TXWL_FLAG          BIT(1)
#define SDXC_FIFO_EMPTY         BIT(2)
#define SDXC_FIFO_FULL          BIT(3)
#define SDXC_CARD_PRESENT       BIT(8)
#define SDXC_CARD_DATA_BUSY     BIT(9)
#define SDXC_DATA_FSM_BUSY      BIT(10)
#define SDXC_DMA_REQUEST        BIT(31)
#define SDXC_FIFO_SIZE          16

/* Function select */
#define SDXC_CEATA_ON               (0xceaa << 16)
#define SDXC_SEND_IRQ_RESPONSE      BIT(0)
#define SDXC_SDIO_READ_WAIT         BIT(1)
#define SDXC_ABORT_READ_DATA        BIT(2)
#define SDXC_SEND_CCSD              BIT(8)
#define SDXC_SEND_AUTO_STOPCCSD     BIT(9)
#define SDXC_CEATA_DEV_IRQ_ENABLE   BIT(10)

/* IDMA controller bus mod bit field */
#define SDXC_IDMAC_SOFT_RESET       BIT(0)
#define SDXC_IDMAC_FIX_BURST        BIT(1)
#define SDXC_IDMAC_IDMA_ON          BIT(7)
#define SDXC_IDMAC_REFETCH_DES      BIT(31)

/* IDMA status bit field */
#define SDXC_IDMAC_TRANSMIT_INTERRUPT       BIT(0)
#define SDXC_IDMAC_RECEIVE_INTERRUPT        BIT(1)
#define SDXC_IDMAC_FATAL_BUS_ERROR          BIT(2)
#define SDXC_IDMAC_DESTINATION_INVALID      BIT(4)
#define SDXC_IDMAC_CARD_ERROR_SUM           BIT(5)
#define SDXC_IDMAC_NORMAL_INTERRUPT_SUM     BIT(8)
#define SDXC_IDMAC_ABNORMAL_INTERRUPT_SUM   BIT(9)
#define SDXC_IDMAC_HOST_ABORT_INTERRUPT     BIT(10)
#define SDXC_IDMAC_IDLE                     (0 << 13)
#define SDXC_IDMAC_SUSPEND                  (1 << 13)
#define SDXC_IDMAC_DESC_READ                (2 << 13)
#define SDXC_IDMAC_DESC_CHECK               (3 << 13)
#define SDXC_IDMAC_READ_REQUEST_WAIT        (4 << 13)
#define SDXC_IDMAC_WRITE_REQUEST_WAIT       (5 << 13)
#define SDXC_IDMAC_READ                     (6 << 13)
#define SDXC_IDMAC_WRITE                    (7 << 13)
#define SDXC_IDMAC_DESC_CLOSE               (8 << 13)

/*
* If the idma-des-size-bits of property is ie 13, bufsize bits are:
*  Bits  0-12: buf1 size
*  Bits 13-25: buf2 size
*  Bits 26-31: not used
* Since we only ever set buf1 size, we can simply store it directly.
*/
#define SDXC_IDMAC_DES0_DIC BIT(1)  /* disable interrupt on completion */
#define SDXC_IDMAC_DES0_LD  BIT(2)  /* last descriptor */
#define SDXC_IDMAC_DES0_FD  BIT(3)  /* first descriptor */
#define SDXC_IDMAC_DES0_CH  BIT(4)  /* chain mode */
#define SDXC_IDMAC_DES0_ER  BIT(5)  /* end of ring */
#define SDXC_IDMAC_DES0_CES BIT(30) /* card error summary */
#define SDXC_IDMAC_DES0_OWN BIT(31) /* 1-idma owns it, 0-host owns it */

#define SDXC_CLK_400K           0
#define SDXC_CLK_25M            1
#define SDXC_CLK_50M            2
#define SDXC_CLK_50M_DDR        3
#define SDXC_CLK_50M_DDR_8BIT   4

#define SDXC_2X_TIMING_MODE     BIT(31)

#define SDXC_CAL_START          BIT(15)
#define SDXC_CAL_DONE           BIT(14)
#define SDXC_CAL_DL_SHIFT       8
#define SDXC_CAL_DL_SW_EN       BIT(7)
#define SDXC_CAL_DL_SW_SHIFT    0
#define SDXC_CAL_DL_MASK        0x3f

#define SDXC_CAL_TIMEOUT        3   /* in seconds, 3s is enough*/
/*********************************************************************************************************
  linux compat
*********************************************************************************************************/
#define msecs_to_jiffies        LW_MSECOND_TO_TICK_1

#define typecheck(type,x) \
({  type __dummy; \
    typeof(x) __dummy2; \
    (void)(&__dummy == &__dummy2); \
    1; \
})

#define time_after(a,b)     \
    (typecheck(unsigned long, a) && \
     typecheck(unsigned long, b) && \
     ((long)((b) - (a)) < 0))
#define time_before(a,b)    time_after(b,a)

struct sunxi_mmc_clk_delay {
    u32 output;
    u32 sample;
};

struct sunxi_idma_des {
    __le32 config;
    __le32 buf_size;
    __le32 buf_addr_ptr1;
    __le32 buf_addr_ptr2;
};

static const struct sunxi_mmc_clk_delay sunxi_mmc_clk_delays[] = {
    [SDXC_CLK_400K]     = { .output = 180, .sample = 180 },
    [SDXC_CLK_25M]      = { .output = 180, .sample =  75 },
    [SDXC_CLK_50M]      = { .output =  90, .sample = 120 },
    [SDXC_CLK_50M_DDR]  = { .output =  60, .sample = 120 },
    /* Value from A83T "new timing mode". Works but might not be right. */
    [SDXC_CLK_50M_DDR_8BIT] = { .output =  90, .sample = 180 },
};

struct sd_channel;
typedef struct sd_channel{
    /* sylixos sd/mmc host info */
    SD_HOST                 SDCH_sdhost;
    PVOID                   SDCH_pvSdmHost;
    LW_SD_FUNCS             SDCH_sdFuncs;
    SD_CALLBACK             SDCH_callbackChkDev;
    PVOID                   SDCH_pvCallBackArg;

    /* channel */
    CHAR                    SDCH_cName[10];
    INT                     SDCH_iChan;

    /* register and gpio */
    UINT32                  SDCH_sdOCR;                                 /*  电压及设备容量支持           */
    addr_t                  SDCH_phyAddr;                               /*  控制器物理基址               */
    addr_t                  SDCH_virAddr;                               /*  控制器虚拟基址               */
    UINT32                  SDCH_uVector;                               /*  中断号                       */
    UINT32                  SDCH_uiSdiCd;                               /*  SD 卡使用的 CD 管脚          */
    UINT32                  SDCH_uiSdiWp;                               /*  SD 卡使用的 WP 管脚          */

#if (DATA_TRANSFER_OPTION == USE_DMA_WITH_CACHE_BUFFER) || \
    (DATA_TRANSFER_OPTION == USE_DMA_WITH_NOCACHE_BUFFER)
    /* irq */
    PLW_JOB_QUEUE           SDCH_jqDefer;                               /*  SDIO 中断在中断延迟队列处理  */
    LW_OBJECT_HANDLE        SDCH_hMsgSync;                              /*  用于msg传输同步              */
    UINT32                  SDCH_uiIntSum;                              /*  控制器中断信息               */
    UINT32                  SDCH_uiSdioMmask;                           /*  SDIO中断信息                 */

    /* dma */
    dma_addr_t              SDCH_aDmaDescPhyAddr;                       /*  DMA描述符缓冲区物理地址      */
    PVOID                   SDCH_aDmaDescVirAddr;                       /*  DMA描述符缓冲区虚拟地址      */
    dma_addr_t              SDCH_aDmaDataPhyAddr;                       /*  DMA数据缓冲区物理地址        */
    BOOL                    SDCH_bWaitDma;
    VOID                    (*SDCH_pDataSync) (PVOID, PLW_SD_MESSAGE, INT);
#endif

    /* status */
    INT                     SDCH_iError;
    INT                     SDCH_iCardSta;                              /*  当前 SD 卡状态               */
    PLW_SD_MESSAGE          SDCH_pSdMsg;
    PLW_SD_MESSAGE          SDCH_pManulStopSdMsg;
} __SD_CHANNEL, *__PSD_CHANNEL;
static __SD_CHANNEL _G_sdChannel[__SDHOST_MAXNUM];

#if (DATA_TRANSFER_OPTION == USE_DMA_WITH_CACHE_BUFFER)
static VOID __sdDmaDataSyncWitchCache (VOID *pDmaAddr, PLW_SD_MESSAGE pMsg, INT iWhen)
{
    PVOID pSrcAddr, pDstAddr;
    INT   iNum = pMsg->SDMSG_psddata->SDDAT_uiBlkSize * pMsg->SDMSG_psddata->SDDAT_uiBlkNum;

    switch (iWhen) {
    case MEM_TO_DEVICE:
        pDstAddr = pDmaAddr;
        pSrcAddr = (PVOID)pMsg->SDMSG_pucWrtBuffer;
        memcpy(pDstAddr, pSrcAddr, iNum);
        API_CacheFlush(DATA_CACHE, (PVOID)pDstAddr, iNum);
        break;
    case DEVICE_TO_MEM:
        pSrcAddr = pDmaAddr;
        pDstAddr = (PVOID)pMsg->SDMSG_pucRdBuffer;
        API_CacheInvalidate(DATA_CACHE, pSrcAddr, iNum);
        memcpy(pDstAddr, pSrcAddr, iNum);
        break;
    }
}
#elif (DATA_TRANSFER_OPTION == USE_DMA_WITH_NOCACHE_BUFFER)
static VOID __sdDmaDataSyncNoCache (VOID *pDmaAddr, PLW_SD_MESSAGE pMsg, INT iWhen)
{
    PVOID pSrcAddr, pDstAddr;
    INT   iNum = pMsg->SDMSG_psddata->SDDAT_uiBlkSize * pMsg->SDMSG_psddata->SDDAT_uiBlkNum;

    switch (iWhen) {
    case MEM_TO_DEVICE:
        pDstAddr = pDmaAddr;
        pSrcAddr = (PVOID)pMsg->SDMSG_pucWrtBuffer;
        memcpy(pDstAddr, pSrcAddr, iNum);
        break;
    case DEVICE_TO_MEM:
        pSrcAddr = pDmaAddr;
        pDstAddr = (PVOID)pMsg->SDMSG_pucRdBuffer;
        memcpy(pDstAddr, pSrcAddr, iNum);
        break;
    }
}
#endif

static int sunxi_mmc_reset_host(__SD_CHANNEL *host)
{
    unsigned long expire = API_TimeGet() + msecs_to_jiffies(250);
    u32 rval;

    mmc_writel(host, REG_GCTRL, SDXC_HARDWARE_RESET);
    do {
        rval = mmc_readl(host, REG_GCTRL);
    } while (time_before(API_TimeGet(), expire) && (rval & SDXC_HARDWARE_RESET));

    if (rval & SDXC_HARDWARE_RESET) {
        SD_ERR("fatal err reset timeout\r\n");
        return -EIO;
    }

    return 0;
}

static int sunxi_mmc_init_host(__SD_CHANNEL *host)
{
    if (sunxi_mmc_reset_host(host))
        return -EIO;

#if (DATA_TRANSFER_OPTION != USE_CPU_COPY_DATA)
    u32 rval;

    /*
     * Burst 8 transfers, RX trigger level: 7, TX trigger level: 8
     *
     * TODO: sun9i has a larger FIFO and supports higher trigger values
     */
    mmc_writel(host, REG_FTRGL, 0x20070008);
    /* Maximum timeout value */
    mmc_writel(host, REG_TMOUT, 0xffffffff);
    /* Unmask SDIO interrupt if needed */
    mmc_writel(host, REG_IMASK, host->SDCH_uiSdioMmask);
    /* Clear all pending interrupts */
    mmc_writel(host, REG_RINTR, 0xffffffff);
    /* Debug register? undocumented */
    mmc_writel(host, REG_DBGC, 0xdeb);
    /* Enable CEATA support */
    mmc_writel(host, REG_FUNS, SDXC_CEATA_ON);
    /* Set DMA descriptor list base address */
    mmc_writel(host, REG_DLBA, host->SDCH_aDmaDescPhyAddr);

    rval = mmc_readl(host, REG_GCTRL);
    rval |= SDXC_INTERRUPT_ENABLE_BIT;
    /* Undocumented, but found in Allwinner code */
    rval &= ~SDXC_ACCESS_DONE_DIRECT;
    mmc_writel(host, REG_GCTRL, rval);
#else
    mmc_writel(host, REG_GCTRL, mmc_readl(host, REG_GCTRL) | SDXC_ACCESS_BY_AHB);
#endif

    return 0;
}

static int sunxi_mmc_oclk_onoff(__SD_CHANNEL *host, u32 oclk_en)
{
    unsigned long expire = API_TimeGet() + msecs_to_jiffies(750);
    u32 rval;

    SD_DBG("%sabling the clock\r\n", oclk_en ? "en" : "dis");

    rval = mmc_readl(host, REG_CLKCR);
    rval &= ~(SDXC_CARD_CLOCK_ON | SDXC_LOW_POWER_ON | SDXC_MASK_DATA0);

    if (oclk_en)
        rval |= SDXC_CARD_CLOCK_ON;

    mmc_writel(host, REG_CLKCR, rval);

    rval = SDXC_START | SDXC_UPCLK_ONLY | SDXC_WAIT_PRE_OVER;
    mmc_writel(host, REG_CMDR, rval);

    do {
        rval = mmc_readl(host, REG_CMDR);
    } while (time_before(API_TimeGet(), expire) && (rval & SDXC_START));

    /* clear irq status bits set by the command */
    mmc_writel(host, REG_RINTR, mmc_readl(host, REG_RINTR) & ~SDXC_SDIO_INTERRUPT);

    if (rval & SDXC_START) {
        SD_ERR("fatal err update clk timeout\r\n");
        return -EIO;
    }

    return 0;
}

static int sunxi_mmc_clk_set_phase(__SD_CHANNEL *host, u32 rate)
{
    int index;

    /* determine delays */
    if (rate <= 400000) {
        index = SDXC_CLK_400K;
    } else if (rate <= 25000000) {
        index = SDXC_CLK_25M;
    } else if (rate <= 52000000) {
        index = SDXC_CLK_50M;
    } else {
        SD_DBG("Invalid clock... returning\r\n");
        return -EINVAL;
    }

    ccu_mmc_sample_clk_set_phase(host->SDCH_iChan, sunxi_mmc_clk_delays[index].sample);
    ccu_mmc_output_clk_set_phase(host->SDCH_iChan, sunxi_mmc_clk_delays[index].output);

    return 0;
}

static int sunxi_mmc_clk_set_rate(__SD_CHANNEL *host, int rate)
{
    u32 rval, div = 1;
    int ret;

    ret = sunxi_mmc_oclk_onoff(host, 0);
    if (ret)
        return ret;

    /* setting clock rate */
    ret = ccu_mmc_clk_rate_set(host->SDCH_iChan, rate);
    if (ret) {
        SD_ERR("error setting clk to %d: %d\r\n", rate, ret);
        return ret;
    }

    /* set internal divider */
    rval = mmc_readl(host, REG_CLKCR);
    rval &= ~0xff;
    rval |= div - 1;
    mmc_writel(host, REG_CLKCR, rval);

    /* update card clock rate to account for internal divider */
    rate /= div;

    /* sunxi_mmc_clk_set_phase expects the actual card clock rate */
    ret = sunxi_mmc_clk_set_phase(host, rate);
    if (ret)
        return ret;

    /*
     * FIXME:
     *
     * In HS400 we'll also need to calibrate the data strobe
     * signal. This should only happen on the MMC2 controller (at
     * least on the A64).
     */

    ret = sunxi_mmc_oclk_onoff(host, 1);
    if (ret)
        return ret;

    return 0;
}

static void sunxi_mmc_set_bus_width(__SD_CHANNEL *host,
                   unsigned char width)
{
    switch (width) {
    case SDBUS_WIDTH_1:
        mmc_writel(host, REG_WIDTH, SDXC_WIDTH1);
        break;
    case SDBUS_WIDTH_4:
        mmc_writel(host, REG_WIDTH, SDXC_WIDTH4);
        break;
    case SDBUS_WIDTH_8:
        mmc_writel(host, REG_WIDTH, SDXC_WIDTH8);
        break;
    }
}

static void sunxi_mmc_set_clk(__SD_CHANNEL *host, int rate)
{
    u32 rval;

    /* set ddr mode */
    rval = mmc_readl(host, REG_GCTRL);
    rval &= ~SDXC_DDR_MODE;
    mmc_writel(host, REG_GCTRL, rval);

    host->SDCH_iError = sunxi_mmc_clk_set_rate(host, rate);
    /* Android code had a usleep_range(50000, 55000); here */
}

static void sunxi_mmc_hw_reset(__SD_CHANNEL *host)
{
    mmc_writel(host, REG_HWRST, 0);
    usleep(10);
    mmc_writel(host, REG_HWRST, 1);
    usleep(300);
}

static int sunxi_mmc_enable(__SD_CHANNEL *host)
{
    int ret;

    ret = ccu_bus_mmc_soft_reset(host->SDCH_iChan);
    if (ret) {
        SD_ERR("Couldn't reset the MMC controller (%d)\r\n", ret);
        return ret;
    }

    ret = ccu_mmc_clk_enable(host->SDCH_iChan, TRUE);
    if (ret) {
        SD_ERR("Couldn't enable the bus clocks (%d)\r\n", ret);
        return ret;
    }

    /*
     * Sometimes the controller asserts the irq on boot for some reason,
     * make sure the controller is in a sane state before enabling irqs.
     */
    ret = sunxi_mmc_reset_host(host);
    if (ret)
        return ret;

    return 0;
}

#if 0
static void sunxi_mmc_disable(__SD_CHANNEL *host)
{
    sunxi_mmc_reset_host(host);

    ccu_mmc_clk_enable(host->SDCH_iChan, FALSE);
}

static int sunxi_mmc_card_busy(__SD_CHANNEL *host)
{
    return !!(mmc_readl(host, REG_STAS) & SDXC_CARD_DATA_BUSY);
}
#endif

#if (DATA_TRANSFER_OPTION != USE_CPU_COPY_DATA)
static void sunxi_mmc_init_idma_des(__SD_CHANNEL *host, LW_SD_DATA *data)
{
    struct sunxi_idma_des *pdes = (struct sunxi_idma_des *)host->SDCH_aDmaDescVirAddr;
    dma_addr_t next_desc = host->SDCH_aDmaDescPhyAddr;
    dma_addr_t next_buffer = host->SDCH_aDmaDataPhyAddr;
    int i, max_len = (1 << 16);

    for (i = 0; i < data->SDDAT_uiBlkNum; i++) {
        pdes[i].config = cpu_to_le32(SDXC_IDMAC_DES0_CH |
                         SDXC_IDMAC_DES0_OWN |
                         SDXC_IDMAC_DES0_DIC);

        if (data->SDDAT_uiBlkSize == max_len)
            pdes[i].buf_size = 0; /* 0 == max_len */
        else
            pdes[i].buf_size = cpu_to_le32(data->SDDAT_uiBlkSize);

        next_desc += sizeof(struct sunxi_idma_des);
        pdes[i].buf_addr_ptr1 = cpu_to_le32(next_buffer);
        pdes[i].buf_addr_ptr2 = cpu_to_le32((u32)next_desc);
        next_buffer += pdes[i].buf_size;
    }

    pdes[0].config |= cpu_to_le32(SDXC_IDMAC_DES0_FD);
    pdes[i - 1].config |= cpu_to_le32(SDXC_IDMAC_DES0_LD |
                      SDXC_IDMAC_DES0_ER);
    pdes[i - 1].config &= cpu_to_le32(~SDXC_IDMAC_DES0_DIC);
    pdes[i - 1].buf_addr_ptr2 = 0;

    /*
     * Avoid the io-store starting the idmac hitting io-mem before the
     * descriptors hit the main-mem.
     */
    KN_WMB();
}

static void sunxi_mmc_start_dma(__SD_CHANNEL *host, LW_SD_DATA *data)
{
    u32 rval;

    sunxi_mmc_init_idma_des(host, data);

    rval = mmc_readl(host, REG_GCTRL);
    rval |= SDXC_DMA_ENABLE_BIT;
    mmc_writel(host, REG_GCTRL, rval);
    rval |= SDXC_DMA_RESET;
    mmc_writel(host, REG_GCTRL, rval);

    mmc_writel(host, REG_DMAC, SDXC_IDMAC_SOFT_RESET);

    if (!SD_DAT_IS_WRITE(data))
        mmc_writel(host, REG_IDIE, SDXC_IDMAC_RECEIVE_INTERRUPT);

    mmc_writel(host, REG_DMAC,
           SDXC_IDMAC_FIX_BURST | SDXC_IDMAC_IDMA_ON);
}

static VOID __sdSdioIntHandle (PVOID pvArg)
{
    __PSD_CHANNEL pChannel;

    pChannel = (__PSD_CHANNEL)pvArg;
    API_SdmEventNotify(pChannel->SDCH_pvSdmHost, SDM_EVENT_SDIO_INTERRUPT);
}

static void sunxi_mmc_dump_errinfo(__PSD_CHANNEL host)
{
    LW_SD_COMMAND *cmd = host->SDCH_pSdMsg->SDMSG_psdcmdCmd;
    LW_SD_DATA *data = host->SDCH_pSdMsg->SDMSG_psddata;

#if 0
    /* For some cmds timeout is normal with sd/mmc cards */
    if ((host->int_sum & SDXC_INTERRUPT_ERROR_BIT) ==
        SDXC_RESP_TIMEOUT && (cmd->SDCMD_uiOpcode == SD_IO_SEND_OP_COND ||
                      cmd->SDCMD_uiOpcode == SD_IO_RW_DIRECT))
        return;
#endif

    SD_ERR(
        "smc %d err, cmd %d,%s%s%s%s%s%s%s%s%s%s !!\r\n",
        host->SDCH_iChan, cmd->SDCMD_uiOpcode,
        data ? (data->SDDAT_uiFlags & SD_DAT_WRITE ? " WR" : " RD") : "",
        host->SDCH_uiIntSum & SDXC_RESP_ERROR     ? " RE"     : "",
        host->SDCH_uiIntSum & SDXC_RESP_CRC_ERROR  ? " RCE"    : "",
        host->SDCH_uiIntSum & SDXC_DATA_CRC_ERROR  ? " DCE"    : "",
        host->SDCH_uiIntSum & SDXC_RESP_TIMEOUT ? " RTO"    : "",
        host->SDCH_uiIntSum & SDXC_DATA_TIMEOUT ? " DTO"    : "",
        host->SDCH_uiIntSum & SDXC_FIFO_RUN_ERROR  ? " FE"     : "",
        host->SDCH_uiIntSum & SDXC_HARD_WARE_LOCKED ? " HL"     : "",
        host->SDCH_uiIntSum & SDXC_START_BIT_ERROR ? " SBE"    : "",
        host->SDCH_uiIntSum & SDXC_END_BIT_ERROR   ? " EBE"    : ""
        );
}

static void sunxi_mmc_send_manual_stop(__PSD_CHANNEL host, PLW_SD_MESSAGE req)
{
    u32 arg, cmd_val, ri;
    unsigned long expire = API_TimeGet() + msecs_to_jiffies(1000);

    cmd_val = SDXC_START | SDXC_RESP_EXPIRE |
          SDXC_STOP_ABORT_CMD | SDXC_CHECK_RESPONSE_CRC;

#if 0
    if (req->cmd->opcode == SD_IO_RW_EXTENDED) {
        cmd_val |= SD_IO_RW_DIRECT;
        arg = (1 << 31) | (0 << 28) | (SDIO_CCCR_ABORT << 9) |
              ((req->cmd->arg >> 28) & 0x7);
    } else {
        cmd_val |= SD_STOP_TRANSMISSION;
        arg = 0;
    }
#else
    cmd_val |= SD_STOP_TRANSMISSION;
    arg = 0;
#endif

    mmc_writel(host, REG_CARG, arg);
    mmc_writel(host, REG_CMDR, cmd_val);

    do {
        ri = mmc_readl(host, REG_RINTR);
    } while (!(ri & (SDXC_COMMAND_DONE | SDXC_INTERRUPT_ERROR_BIT)) &&
         time_before(API_TimeGet(), expire));

    if (!(ri & SDXC_COMMAND_DONE) || (ri & SDXC_INTERRUPT_ERROR_BIT)) {
        SD_ERR("send stop command failed\n");
        if (req->SDMSG_psdcmdStop)
            req->SDMSG_psdcmdStop->SDCMD_uiResp[0] = -ETIMEDOUT;
    } else {
        if (req->SDMSG_psdcmdStop)
            req->SDMSG_psdcmdStop->SDCMD_uiResp[0] = mmc_readl(host, REG_RESP0);
    }

    mmc_writel(host, REG_RINTR, 0xffff);

    host->SDCH_pManulStopSdMsg = NULL;
}


/* Called in interrupt context! */
static irqreturn_t sunxi_mmc_finalize_request(__PSD_CHANNEL host)
{
    PLW_SD_MESSAGE mrq = host->SDCH_pSdMsg;
    LW_SD_DATA *data = host->SDCH_pSdMsg->SDMSG_psddata;
    u32 rval;

    mmc_writel(host, REG_IMASK, host->SDCH_uiSdioMmask);
    mmc_writel(host, REG_IDIE, 0);

    if (host->SDCH_uiIntSum & SDXC_INTERRUPT_ERROR_BIT) {
        SD_ERR("sd/mmc controller interrupt error\r\n");
        sunxi_mmc_dump_errinfo(host);

        if (data) {
            host->SDCH_pManulStopSdMsg = mrq;
        }
    } else {
        if (SD_MSG_FLG(mrq) & SD_RSP_136) {
            mrq->SDMSG_psdcmdCmd->SDCMD_uiResp[0] = mmc_readl(host, REG_RESP3);
            mrq->SDMSG_psdcmdCmd->SDCMD_uiResp[1] = mmc_readl(host, REG_RESP2);
            mrq->SDMSG_psdcmdCmd->SDCMD_uiResp[2] = mmc_readl(host, REG_RESP1);
            mrq->SDMSG_psdcmdCmd->SDCMD_uiResp[3] = mmc_readl(host, REG_RESP0);
        } else {
            mrq->SDMSG_psdcmdCmd->SDCMD_uiResp[0] = mmc_readl(host, REG_RESP0);
        }
    }

    if (data) {
        mmc_writel(host, REG_IDST, 0x337);
        mmc_writel(host, REG_DMAC, 0);
        rval = mmc_readl(host, REG_GCTRL);
        rval |= SDXC_DMA_RESET;
        mmc_writel(host, REG_GCTRL, rval);
        rval &= ~SDXC_DMA_ENABLE_BIT;
        mmc_writel(host, REG_GCTRL, rval);
        rval |= SDXC_FIFO_RESET;
        mmc_writel(host, REG_GCTRL, rval);
    }
    mmc_writel(host, REG_RINTR, 0xffff);

    if (host->SDCH_uiIntSum & SDXC_INTERRUPT_ERROR_BIT) {
        if (data) {
            sunxi_mmc_send_manual_stop(host, host->SDCH_pManulStopSdMsg);
        }
    }

    if (host->SDCH_pSdMsg->SDMSG_psddata && SD_DAT_IS_READ(host->SDCH_pSdMsg->SDMSG_psddata)) {
        host->SDCH_pDataSync((PVOID)host->SDCH_aDmaDataPhyAddr, mrq, DEVICE_TO_MEM);
    }

    host->SDCH_pSdMsg   = NULL;
    host->SDCH_uiIntSum = 0;
    host->SDCH_bWaitDma = FALSE;

    return LW_IRQ_HANDLED;
}

/*
 *  R16上的sd中断分为sd控制器中断和sd dma传输中断
 *  当有数据传输时，先产生sd dma传输中断，紧接着产生sd控制器中断
 */
static irqreturn_t __sdIrq (PVOID pvArg, ULONG ulVector)
{
    SD_DBG("irq start\r\n");
    __PSD_CHANNEL host = (__PSD_CHANNEL)pvArg;
    u32 msk_int, idma_int;
    BOOL finalize = FALSE;
    BOOL sdio_int = FALSE;

    idma_int  = mmc_readl(host, REG_IDST);
    msk_int   = mmc_readl(host, REG_MISTA);

    SD_DBG("irq: mi %08x idi %08x\r\n", msk_int, idma_int);

    if (host->SDCH_pSdMsg) {
        if (idma_int & SDXC_IDMAC_RECEIVE_INTERRUPT)
            host->SDCH_bWaitDma = FALSE;

        host->SDCH_uiIntSum |= msk_int;

        /* Wait for COMMAND_DONE on RESPONSE_TIMEOUT before finalize */
        if ((host->SDCH_uiIntSum & SDXC_RESP_TIMEOUT) &&
                !(host->SDCH_uiIntSum & SDXC_COMMAND_DONE))
            mmc_writel(host, REG_IMASK,
                   host->SDCH_uiSdioMmask | SDXC_COMMAND_DONE);
        /* Don't wait for dma on error */
        else if (host->SDCH_uiIntSum & SDXC_INTERRUPT_ERROR_BIT)
            finalize = TRUE;
        else if ((host->SDCH_uiIntSum & SDXC_INTERRUPT_DONE_BIT) &&
                !host->SDCH_bWaitDma)
            finalize = TRUE;
    }

    if (msk_int & SDXC_SDIO_INTERRUPT)
        sdio_int = TRUE;

    mmc_writel(host, REG_RINTR, msk_int);
    mmc_writel(host, REG_IDST, idma_int);

    if (finalize)
        sunxi_mmc_finalize_request(host);

    if (sdio_int)
        API_InterDeferJobAdd(host->SDCH_jqDefer, __sdSdioIntHandle, (PVOID)host);

    if (finalize)
        API_SemaphoreBPost(host->SDCH_hMsgSync);

    SD_DBG("irq handle end\r\n");

    return LW_IRQ_HANDLED;
}
#endif

static INT __sdIoCtl (PLW_SD_ADAPTER psdadapter, INT iCmd, LONG lArg)
{
    __SD_CHANNEL   *pChannel;
    INT             iError = ERROR_NONE;
    INT             iNum;

    iNum     = psdadapter->SDADAPTER_busadapter.BUSADAPTER_cName[sizeof(__SDHOST_NAME) - 2]- '0';
    pChannel = &_G_sdChannel[iNum];                                     /* 用名称来判断是哪一个主控器    */
    switch (iCmd) {
    case SDBUS_CTRL_POWEROFF:
        break;
    case SDBUS_CTRL_POWERUP:
    case SDBUS_CTRL_POWERON:
        break;
    case SDBUS_CTRL_SETBUSWIDTH:
        sunxi_mmc_set_bus_width(pChannel, lArg);
        break;
    case SDBUS_CTRL_SETCLK:
        sunxi_mmc_set_clk(pChannel, lArg);
        break;
    case SDBUS_CTRL_DELAYCLK:
        break;
    case SDBUS_CTRL_GETOCR:
        *(UINT32 *)lArg = pChannel->SDCH_sdOCR;
        iError = ERROR_NONE;
        break;
    default:
        SD_ERR("%s error : can't support this cmd.\r\n", __func__);
        iError = PX_ERROR;
        break;
    }

    return (iError);
}

#if (DATA_TRANSFER_OPTION != USE_CPU_COPY_DATA)
static INT __sdTransferUseDma (PLW_SD_ADAPTER psdadapter,
                               PLW_SD_DEVICE  psddevice,
                               PLW_SD_MESSAGE psdmsg,
                               INT            iNum)
{
    INT             iCount = 0;
    __SD_CHANNEL   *host;
    INT             iCh;

    iCh      = psdadapter->SDADAPTER_busadapter.BUSADAPTER_cName[sizeof(__SDHOST_NAME) - 2] - '0';
    host = &_G_sdChannel[iCh];

    LW_SD_DATA *data = psdmsg->SDMSG_psddata;
    u32 imask = SDXC_INTERRUPT_ERROR_BIT;
    u32 cmd_val = SDXC_START | (SD_MSG_OPC(psdmsg) & 0x3f);
    BOOL wait_dma = host->SDCH_bWaitDma;

    /* Check for __sdIoCtl errors (should never happen) */
    if (host->SDCH_iError) {
        SD_ERR("%s error : error in __sdIoCtl.\r\n", __func__);
        return (PX_ERROR);
    }

    while (iCount < iNum && psdmsg != NULL) {
        /* prepare dma buffer to transfer */
        if (data && SD_DAT_IS_WRITE(data)) {
            host->SDCH_pDataSync((PVOID)host->SDCH_aDmaDataPhyAddr, psdmsg, MEM_TO_DEVICE);
        }

        if (SD_MSG_OPC(psdmsg) == SD_GO_IDLE_STATE) {
            cmd_val |= SDXC_SEND_INIT_SEQUENCE;
            imask |= SDXC_COMMAND_DONE;
        }

        if (SD_MSG_FLG(psdmsg) & SD_RSP_PRESENT) {
            cmd_val |= SDXC_RESP_EXPIRE;
            if (SD_MSG_FLG(psdmsg) & SD_RSP_136)
                cmd_val |= SDXC_LONG_RESPONSE;
            if (SD_MSG_FLG(psdmsg) & SD_RSP_CRC)
                cmd_val |= SDXC_CHECK_RESPONSE_CRC;

            if ((SD_MSG_FLG(psdmsg) & SD_CMD_MASK) == SD_CMD_ADTC) {
                cmd_val |= SDXC_DATA_EXPIRE | SDXC_WAIT_PRE_OVER;

                if (psdmsg->SDMSG_psdcmdStop) {
                    imask |= SDXC_AUTO_COMMAND_DONE;
                    cmd_val |= SDXC_SEND_AUTO_STOP;
                } else {
                    imask |= SDXC_DATA_OVER;
                }

                if (data && SD_DAT_IS_WRITE(data))
                    cmd_val |= SDXC_WRITE;
                else
                    wait_dma = TRUE;
            } else {
                imask |= SDXC_COMMAND_DONE;
            }
        } else {
            imask |= SDXC_COMMAND_DONE;
        }

        SD_DBG( "cmd %d(%08x) flag %x arg %x ie 0x%08x len %d\r\n",
            cmd_val & 0x3f, cmd_val, SD_MSG_FLG(psdmsg), SD_MSG_ARG(psdmsg), imask,
            data ? data->SDDAT_uiBlkSize * data->SDDAT_uiBlkNum : 0);

        if (host->SDCH_pSdMsg || host->SDCH_pManulStopSdMsg) {
            SD_ERR("request already pending\r\n");
            return (PX_ERROR);
        }

        host->SDCH_pSdMsg = psdmsg;
        if (data) {
            mmc_writel(host, REG_BLKSZ, data->SDDAT_uiBlkSize);
            mmc_writel(host, REG_BCNTR, data->SDDAT_uiBlkSize * data->SDDAT_uiBlkNum);
            sunxi_mmc_start_dma(host, data);
        }

        host->SDCH_bWaitDma = wait_dma;
        mmc_writel(host, REG_IMASK, host->SDCH_uiSdioMmask | imask);
        mmc_writel(host, REG_CARG, SD_MSG_ARG(psdmsg));
        mmc_writel(host, REG_CMDR, cmd_val);

        API_SemaphoreBPend(host->SDCH_hMsgSync, LW_OPTION_WAIT_INFINITE);

        iCount++;
        psdmsg++;
    }

    SD_DBG("data transfer end\r\n");

    return (ERROR_NONE);
}
#else
static int mmc_trans_data_by_cpu(__SD_CHANNEL *host, LW_SD_DATA *data)
{
    const int reading = !!(data->SDDAT_uiFlags & SD_DAT_READ);
    const uint32_t status_bit = (reading ? SDXC_FIFO_EMPTY : SDXC_FIFO_FULL);
    unsigned i;
    unsigned *buff = (unsigned int *)(reading ? host->SDCH_pSdMsg->SDMSG_pucRdBuffer :
                                                host->SDCH_pSdMsg->SDMSG_pucWrtBuffer);
    unsigned byte_cnt = data->SDDAT_uiBlkSize * data->SDDAT_uiBlkNum;
    unsigned timeout_msecs = byte_cnt >> 8;

    if (timeout_msecs < 2000)
        timeout_msecs = 2000;

    unsigned long  end = API_TimeGet() + LW_MSECOND_TO_TICK_1(timeout_msecs);

    for (i = 0; i < (byte_cnt >> 2); i++) {
        while (mmc_readl(host, REG_STAS) & status_bit) {
            if (API_TimeGet() > end)
                return -1;
        }
        if (reading) {
            buff[i] = mmc_readl(host, REG_FIFO);
        }
        else
            mmc_writel(host, REG_FIFO, buff[i]);
    }

    return 0;
}

static int mmc_rint_wait(__SD_CHANNEL *host, u32 timeout_msecs, u32 done_bit, const char *what)
{
    unsigned int status;
    unsigned long end = API_TimeGet() + LW_MSECOND_TO_TICK_1(timeout_msecs);

    do {
        status = mmc_readl(host, REG_RINTR);
        if ((API_TimeGet() > end) ||
            (status & SDXC_INTERRUPT_ERROR_BIT)) {
            SD_DBG("%s timeout %x\n", what, status & SDXC_INTERRUPT_ERROR_BIT);
            return -ETIMEDOUT;
        }
    } while (!(status & done_bit));

    return 0;
}

static INT __sdTransferUseCpu (PLW_SD_ADAPTER psdadapter,
                               PLW_SD_DEVICE  psddevice,
                               PLW_SD_MESSAGE psdmsg,
                               INT            iNum)
{
    INT             iCount = 0;
    __SD_CHANNEL   *host;
    INT             iCh;

    iCh  = psdadapter->SDADAPTER_busadapter.BUSADAPTER_cName[sizeof(__SDHOST_NAME) - 2] - '0';
    host = &_G_sdChannel[iCh];

    LW_SD_DATA    *data = psdmsg->SDMSG_psddata;
    LW_SD_COMMAND *cmd  = psdmsg->SDMSG_psdcmdCmd;

    unsigned int cmdval = SDXC_START;
    unsigned int timeout_msecs;
    int error = 0;
    unsigned int status = 0;

    while (iCount < iNum && psdmsg != NULL) {
        host->SDCH_pSdMsg = psdmsg;

        /* Check for __sdIoCtl errors (should never happen) */
        if (host->SDCH_iError) {
            SD_ERR("%s error : error in __sdIoCtl.\r\n", __func__);
            return (PX_ERROR);
        }
        if (SD_MSG_FLG(psdmsg) & SD_RSP_BUSY)
            SD_DBG("mmc cmd %d check rsp busy\r\n", cmd->SDCMD_uiOpcode);

        if (cmd->SDCMD_uiOpcode == SD_STOP_TRANSMISSION)
            return 0;

        if (SD_MSG_OPC(psdmsg) == SD_GO_IDLE_STATE)
            cmdval |= SDXC_SEND_INIT_SEQUENCE;

        if (SD_MSG_FLG(psdmsg) & SD_RSP_PRESENT)
            cmdval |= SDXC_RESP_EXPIRE;
        if (SD_MSG_FLG(psdmsg) & SD_RSP_136)
            cmdval |= SDXC_LONG_RESPONSE;
        if (SD_MSG_FLG(psdmsg) & SD_RSP_CRC)
            cmdval |= SDXC_CHECK_RESPONSE_CRC;

        if (data) {
            cmdval |= SDXC_DATA_EXPIRE | SDXC_WAIT_PRE_OVER;
            if (SD_DAT_IS_WRITE(data))
                cmdval |= SDXC_WRITE;
            if (data->SDDAT_uiBlkNum > 1)
                cmdval |= SDXC_SEND_AUTO_STOP;
            mmc_writel(host, REG_BLKSZ, data->SDDAT_uiBlkSize);
            mmc_writel(host, REG_BCNTR, data->SDDAT_uiBlkSize * data->SDDAT_uiBlkNum);
        }

        SD_DBG( "cmd %d(%08x) flag %x arg %x len %d\r\n",
                SD_MSG_OPC(psdmsg), cmdval, SD_MSG_FLG(psdmsg), SD_MSG_ARG(psdmsg),
                data ? data->SDDAT_uiBlkSize * data->SDDAT_uiBlkNum : 0);

        mmc_writel(host, REG_CARG, SD_MSG_ARG(psdmsg));

        if (!data)
            mmc_writel(host, REG_CMDR, cmdval | SD_MSG_OPC(psdmsg));

        /*
         * transfer data and check status
         * STATREG[2] : FIFO empty
         * STATREG[3] : FIFO full
         */
        if (data) {
            int ret = 0;

            mmc_writel(host, REG_CMDR, cmdval | SD_MSG_OPC(psdmsg));
            ret = mmc_trans_data_by_cpu(host, data);
            if (ret) {
                error = mmc_readl(host, REG_RINTR) & SDXC_INTERRUPT_ERROR_BIT;
                error = -ETIMEDOUT;
                goto out;
            }
        }

        error = mmc_rint_wait(host, 1000, SDXC_COMMAND_DONE, "cmd");
        if (error)
            goto out;

        if (data) {
            timeout_msecs = 120;
            SD_DBG("cacl timeout %x msec\r\n", timeout_msecs);
            error = mmc_rint_wait(host, timeout_msecs,
                          data->SDDAT_uiBlkNum > 1 ?
                          SDXC_AUTO_COMMAND_DONE :
                          SDXC_DATA_OVER,
                          "data");
            if (error)
                goto out;
        }

        if (SD_MSG_FLG(psdmsg) & SD_RSP_BUSY) {
            unsigned long end = API_TimeGet() + LW_MSECOND_TO_TICK_1(2000);

            do {
                status = mmc_readl(host, REG_STAS);
                if (API_TimeGet() > end) {
                    SD_DBG("busy timeout\r\n");
                    error = -ETIMEDOUT;
                    goto out;
                }
            } while (status & SDXC_CARD_DATA_BUSY);
        }

        if (SD_MSG_FLG(psdmsg) & SD_RSP_136) {
            cmd->SDCMD_uiResp[0] = mmc_readl(host, REG_RESP3);
            cmd->SDCMD_uiResp[1] = mmc_readl(host, REG_RESP1);
            cmd->SDCMD_uiResp[2] = mmc_readl(host, REG_RESP1);
            cmd->SDCMD_uiResp[3] = mmc_readl(host, REG_RESP0);
            SD_DBG("mmc resp 0x%08x 0x%08x 0x%08x 0x%08x\r\n",
                  cmd->SDCMD_uiResp[3], cmd->SDCMD_uiResp[2],
                  cmd->SDCMD_uiResp[1], cmd->SDCMD_uiResp[0]);
        } else {
            cmd->SDCMD_uiResp[0] = mmc_readl(host, REG_RESP0);
            SD_DBG("mmc resp 0x%08x\r\n", cmd->SDCMD_uiResp[0]);
        }

out:
        if (error < 0) {
            SD_ERR("transfer error\r\n");
        }
        mmc_writel(host, REG_RINTR, 0xffffffff);
        mmc_writel(host, REG_GCTRL, mmc_readl(host, REG_GCTRL) | SDXC_FIFO_RESET);
        host->SDCH_pSdMsg = NULL;

        iCount++;
        psdmsg++;
    }

    return (ERROR_NONE);
}
#endif

static INT __sdCallBackInstall (SD_HOST    *pHost,
                                INT         iCallbackType,
                                SD_CALLBACK callback,
                                PVOID       pvCallbackArg)
{
    __SD_CHANNEL *pChannel = (__SD_CHANNEL *)pHost;

    if (!pChannel) {
        return (PX_ERROR);
    }

    if (iCallbackType == SDHOST_CALLBACK_CHECK_DEV) {
        pChannel->SDCH_callbackChkDev = callback;
        pChannel->SDCH_pvCallBackArg = pvCallbackArg;
    }

    return (ERROR_NONE);
}

static INT __sdCallBackUnInstall (SD_HOST *pHost, INT iCallbackType)
{
    __SD_CHANNEL *pChannel = (__SD_CHANNEL *)pHost;

    if (!pChannel) {
        return (PX_ERROR);
    }

    if (iCallbackType == SDHOST_CALLBACK_CHECK_DEV) {
        pChannel->SDCH_callbackChkDev = NULL;
        pChannel->SDCH_pvCallBackArg  = NULL;
    }

    return (ERROR_NONE);
}

#if (DATA_TRANSFER_OPTION != USE_CPU_COPY_DATA)
static VOID __sdSdioIntEn (SD_HOST *pHost, BOOL bEnable)
{
    u32 imask;

    __SD_CHANNEL *host = (__SD_CHANNEL *)pHost;

    imask = mmc_readl(host, REG_IMASK);
    if (bEnable) {
        host->SDCH_uiSdioMmask = SDXC_SDIO_INTERRUPT;
        imask |= SDXC_SDIO_INTERRUPT;
    } else {
        host->SDCH_uiSdioMmask = 0;
        imask &= ~SDXC_SDIO_INTERRUPT;
    }
    mmc_writel(host, REG_IMASK, imask);
}
#endif

static INT __sdDataInit (INT iChannel, UINT32 uiSdCdPin, UINT32 uiSdWpPin)
{
    __PSD_CHANNEL    pChannel;
    SD_HOST         *pSdHost;
    LW_SD_FUNCS     *pSdFuncs;
    INT              iRet;
    CHAR             cSdCdName[10] = __SDHOST_CDNAME;
    CHAR             cSdWpName[10] = __SDHOST_WPNAME;

    pChannel = &_G_sdChannel[iChannel];
    pSdHost  = &pChannel->SDCH_sdhost;
    pSdFuncs = &pChannel->SDCH_sdFuncs;
    pChannel->SDCH_uiSdiCd = uiSdCdPin;
    pChannel->SDCH_uiSdiWp = uiSdWpPin;

    snprintf(pChannel->SDCH_cName, sizeof(__SDHOST_NAME), __SDHOST_NAME);
    pChannel->SDCH_iChan = iChannel;

#if (DATA_TRANSFER_OPTION != USE_CPU_COPY_DATA)
    pChannel->SDCH_hMsgSync = API_SemaphoreBCreate("sunxi_sd_sync",
                                                    LW_FALSE,
                                                    LW_OPTION_OBJECT_GLOBAL,
                                                    LW_NULL);
    if (pChannel->SDCH_hMsgSync == LW_OBJECT_HANDLE_INVALID) {
        SD_ERR("%s err:Semaphore create fail!\r\n", __func__);
        goto __erra2;
    }

    pChannel->SDCH_aDmaDescVirAddr = API_VmmDmaAlloc(PAGE_SIZE);
    if (!pChannel->SDCH_aDmaDescVirAddr) {
        SD_ERR("%s err:Dma descriptor buffer create fail!\r\n", __func__);
        goto __erra3;
    }
    pChannel->SDCH_aDmaDescPhyAddr = (dma_addr_t)pChannel->SDCH_aDmaDescVirAddr;

#if (DATA_TRANSFER_OPTION == USE_DMA_WITH_NOCACHE_BUFFER)
    pChannel->SDCH_aDmaDataPhyAddr = (addr_t)API_VmmDmaAlloc(DMA_DATA_BUFFER_SIZE);
    pChannel->SDCH_pDataSync = __sdDmaDataSyncNoCache;
#else
    pChannel->SDCH_aDmaDataPhyAddr = (addr_t)API_VmmDmaAllocAlignWithFlags(DMA_DATA_BUFFER_SIZE,
                                                                           4,
                                                                           LW_VMM_FLAG_RDWR);
    pChannel->SDCH_pDataSync = __sdDmaDataSyncWitchCache;
#endif
    if (!pChannel->SDCH_aDmaDataPhyAddr) {
        SD_ERR("%s err:Dma data buffer create fail!\r\n", __func__);
        goto __erra4;
    }
#endif

    switch (iChannel) {
    case 0:
        pChannel->SDCH_phyAddr = __SDHOST_PHYADDR_CH0;
        pChannel->SDCH_uVector = __SDHOST_VECTOR_CH0;
        break;
    case 1:
        pChannel->SDCH_phyAddr = __SDHOST_PHYADDR_CH1;
        pChannel->SDCH_uVector = __SDHOST_VECTOR_CH1;
        break;
    case 2:
        pChannel->SDCH_phyAddr = __SDHOST_PHYADDR_CH2;
        pChannel->SDCH_uVector = __SDHOST_VECTOR_CH2;
        break;
    }

#if (DATA_TRANSFER_OPTION != USE_CPU_COPY_DATA)
    iRet = API_InterVectorConnect(pChannel->SDCH_uVector,
                                  __sdIrq,
                                  pChannel,
                                  "sd_isr");
    if (iRet) {
        SD_ERR("%s err:Vector connect fail!\r\n", __func__);
        goto __erra5;
    }

    iRet = API_InterVectorEnable(pChannel->SDCH_uVector);
    if (iRet) {
        SD_ERR("%s err:Vector enable fail!\r\n", __func__);
        goto __erra6;
    }
#endif

    pChannel->SDCH_virAddr = (addr_t)API_VmmIoRemap((PVOID)pChannel->SDCH_phyAddr, LW_CFG_VMM_PAGE_SIZE);
    if (!pChannel->SDCH_virAddr) {
        SD_ERR("%s err:Ioremap fail!\r\n", __func__);
        goto __erra7;
    }
    cSdCdName[sizeof(__SDHOST_CDNAME) - 2] += iChannel;
    cSdWpName[sizeof(__SDHOST_WPNAME) - 2] += iChannel;
    if (uiSdCdPin != GPIO_NONE && uiSdCdPin != 0) {                     /*  申请热插拔检测管脚           */
        iRet = API_GpioRequestOne(uiSdCdPin, LW_GPIOF_IN, cSdCdName);
        if (iRet != ERROR_NONE) {
            SD_ERR("%s err:failed to request gpio %d!\r\n", __func__, uiSdCdPin);
            goto __erra8;
        }
    }

    if (uiSdWpPin != GPIO_NONE && uiSdWpPin != 0) {                     /*  申请写保护检测管脚           */
        iRet = API_GpioRequestOne(uiSdWpPin, LW_GPIOF_IN, cSdWpName);
        if (iRet != ERROR_NONE) {
            SD_ERR("%s err:failed to request gpio %d!\r\n", __func__, uiSdWpPin);
            goto __erra9;
        }
    }

#if (DATA_TRANSFER_OPTION != USE_CPU_COPY_DATA)
    pChannel->SDCH_jqDefer  = API_InterDeferGet(0);                     /*  获取 CPU0 中断延迟处理队列   */
#endif
    pChannel->SDCH_cName[sizeof(__SDHOST_NAME) - 2] += iChannel;
    pChannel->SDCH_iCardSta = 0;                                        /*  初始化状态为卡未插入         */
    pChannel->SDCH_sdOCR    = SD_VDD_32_33 |                            /*  OCR 中包含主控制器的电压支持 */
                              SD_VDD_33_34 ;                            /*  情况,还有设备容量支持情况    */

    /*
     * 注册 SD 总线需要使用的结构体
     */
    pSdFuncs->SDFUNC_pfuncMasterCtl  = __sdIoCtl;
#if (DATA_TRANSFER_OPTION != USE_CPU_COPY_DATA)
    pSdFuncs->SDFUNC_pfuncMasterXfer = __sdTransferUseDma;
#else
    pSdFuncs->SDFUNC_pfuncMasterXfer = __sdTransferUseCpu;
#endif

    /*
     * 注册到 SDM 层需要的结构体
     */
    pSdHost->SDHOST_cpcName                = pChannel->SDCH_cName;
    pSdHost->SDHOST_iType                  = SDHOST_TYPE_SD;
    pSdHost->SDHOST_iCapbility             = SDHOST_CAP_HIGHSPEED | SDHOST_CAP_DATA_4BIT;
    pSdHost->SDHOST_pfuncCallbackInstall   = __sdCallBackInstall;
    pSdHost->SDHOST_pfuncCallbackUnInstall = __sdCallBackUnInstall;
#if (DATA_TRANSFER_OPTION != USE_CPU_COPY_DATA)
    pSdHost->SDHOST_pfuncSdioIntEn         = __sdSdioIntEn;
#endif

    return (ERROR_NONE);

__erra9:
    API_GpioFree(uiSdCdPin);
__erra8:
    API_VmmIoUnmap((PVOID)pChannel->SDCH_virAddr);
__erra7:
    API_InterVectorDisable(pChannel->SDCH_uVector);
#if (DATA_TRANSFER_OPTION != USE_CPU_COPY_DATA)
__erra6:
    API_InterVectorDisconnect(pChannel->SDCH_uVector, __sdIrq, pChannel);
__erra5:
    API_VmmDmaFree((PVOID)pChannel->SDCH_aDmaDataPhyAddr);
__erra4:
    API_VmmDmaFree(pChannel->SDCH_aDmaDescVirAddr);
__erra3:
    API_SemaphoreBDelete(&pChannel->SDCH_hMsgSync);
__erra2:
#endif
    return (PX_ERROR);
}

static INT __sdHwInit (INT iChannel)
{
    int ret;
    __PSD_CHANNEL   host;

    host = &_G_sdChannel[iChannel];

    ret = sunxi_mmc_enable(host);
    if (ret)
        return ret;

    sunxi_mmc_hw_reset(host);

    ret = sunxi_mmc_init_host(host);
    if (ret)
        return ret;

    return (ERROR_NONE);
}

static VOID __sdDataDeinit (INT iChannel)
{
    __PSD_CHANNEL pChannel;

    pChannel = &_G_sdChannel[iChannel];
    API_GpioFree(pChannel->SDCH_uiSdiWp);
    API_GpioFree(pChannel->SDCH_uiSdiCd);
    API_VmmIoUnmap((PVOID)pChannel->SDCH_virAddr);
#if (DATA_TRANSFER_OPTION != USE_CPU_COPY_DATA)
    API_InterVectorDisable(pChannel->SDCH_uVector);
    API_InterVectorDisconnect(pChannel->SDCH_uVector, __sdIrq, pChannel);
    API_VmmDmaFree((PVOID)pChannel->SDCH_aDmaDataPhyAddr);
    API_VmmDmaFree(pChannel->SDCH_aDmaDescVirAddr);
    API_SemaphoreBDelete(&pChannel->SDCH_hMsgSync);
#endif
    memset(pChannel, 0, sizeof(__SD_CHANNEL));
}

static INT __sdStatGet (__PSD_CHANNEL pChannel)
{
    UINT32 uiValue = 0;
    if (pChannel->SDCH_uiSdiCd != GPIO_NONE && pChannel->SDCH_uiSdiCd != 0) {
        uiValue = API_GpioGetValue(pChannel->SDCH_uiSdiCd);
    } else {
        uiValue = 0;
    }

    return (uiValue == 0);
}

static VOID __sdCdHandle(PVOID pvArg)
{
    INT           iStaCurr;
    __PSD_CHANNEL pChannel = (__PSD_CHANNEL)pvArg;

    iStaCurr = __sdStatGet(pChannel);
    if (iStaCurr ^ pChannel->SDCH_iCardSta) {
        if (iStaCurr) {                                               /*  插入状态                       */
            SD_DBG("insert card\r\n");
            API_SdmEventNotify(pChannel->SDCH_pvSdmHost, SDM_EVENT_DEV_INSERT);
        } else {                                                      /*  移除状态                       */
            SD_DBG("remove card\r\n");
            if (pChannel->SDCH_callbackChkDev) {
                pChannel->SDCH_callbackChkDev(pChannel->SDCH_pvCallBackArg, SDHOST_DEVSTA_UNEXIST);
            }
            API_SdmEventNotify(pChannel->SDCH_pvSdmHost, SDM_EVENT_DEV_REMOVE);
        }

        pChannel->SDCH_iCardSta = iStaCurr;
    }
}

static VOID sdiLibInit (VOID)
{
    /*
     * SDM 系统层初始化，包含 SD 存储卡库的初始化
     */
    API_SdmLibInit();

    /*
     * SDIO 库初始化
     */
    API_SdmSdioLibInit();
    API_SdMemDrvInstall();                                              /*  安装 SD 存储卡驱动           */
    API_SdioBaseDrvInstall();                                           /*  安装 SDIO 基础驱动           */
}

INT sdDrvInstall (INT iChannel, UINT32 uiSdCdPin, UINT32 uiSdWpPin, BOOL bIsBootDev)
{
    __PSD_CHANNEL   pChannel;
    INT             iRet;

    sdiLibInit();

    /*
     * 初始化控制器软件资源
     */
    iRet = __sdDataInit(iChannel, uiSdCdPin, uiSdWpPin);
    if (iRet != ERROR_NONE) {
        return (PX_ERROR);
    }

    /*
     * 初始化控制器硬件资源
     */
    iRet = __sdHwInit(iChannel);
    if (iRet != ERROR_NONE) {
        goto __errb1;
    }

    /*
     * 创建 SD 总线适配器
     */
    pChannel = &_G_sdChannel[iChannel];
    iRet = API_SdAdapterCreate(pChannel->SDCH_cName, &pChannel->SDCH_sdFuncs);
    if (iRet != ERROR_NONE) {
        SD_ERR("%s err:fail to create sd adapter!\r\n", __func__);
        goto __errb1;
    }

    /*
     * 向 SDM 注册 HOST 信息
     */
    pChannel->SDCH_pvSdmHost = API_SdmHostRegister(&pChannel->SDCH_sdhost);
    if (!pChannel->SDCH_pvSdmHost) {
        SD_ERR("%s err:fail to register SDM host!\r\n", __func__);
        goto __errb2;
    }

    /*
     * BOOT 设备特殊处理: 在当前线程(而非热插拔线程)直接创建设备, 提高启动速度
     */
    if (bIsBootDev) {
        iRet = API_SdmEventNotify(pChannel->SDCH_pvSdmHost, SDM_EVENT_BOOT_DEV_INSERT);
        pChannel->SDCH_iCardSta = 1;
        if (iRet) {
            SD_ERR("%s err:fail to create boot device!\r\n", __func__);
            pChannel->SDCH_iCardSta = 0;
        }
    }

    /*
     * 加入到内核热插拔检测线程
     */
    hotplugPollAdd((VOIDFUNCPTR)__sdCdHandle, (PVOID)pChannel);

    return (ERROR_NONE);

__errb2:
    API_SdAdapterDelete(pChannel->SDCH_cName);
__errb1:
    __sdDataDeinit(iChannel);
    return (PX_ERROR);
}
