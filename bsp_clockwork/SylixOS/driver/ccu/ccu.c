/*
 * ccu.c
 *
 *  Created on: Sep 20, 2019
 *      Author: databus
 */
#define __SYLIXOS_KERNEL
#include "config.h"
#include "SylixOS.h"
#include <linux/compat.h>

#define CCU_REG_BASE            (0x1c20000)
#define CCU_REG_BUS_CLK_GATE0   (CCU_REG_BASE + 0x60)
#define CCU_REG_BUS_CLK_GATE1   (CCU_REG_BASE + 0x64)
#define CCU_REG_BUS_CLK_GATE2   (CCU_REG_BASE + 0x68)
#define CCU_REG_BUS_CLK_GATE3   (CCU_REG_BASE + 0x6c)

#define CCU_REG_SDMMC0_CLK      (CCU_REG_BASE + 0x88)
#define CCU_REG_SDMMC1_CLK      (CCU_REG_BASE + 0x8c)
#define CCU_REG_SDMMC2_CLK      (CCU_REG_BASE + 0x90)

#define CCU_REG_BUS_SOFT_RST0   (CCU_REG_BASE + 0x2c0)
#define CCU_REG_BUS_SOFT_RST1   (CCU_REG_BASE + 0x2c4)
#define CCU_REG_BUS_SOFT_RST2   (CCU_REG_BASE + 0x2c8)
#define CCU_REG_BUS_SOFT_RST3   (CCU_REG_BASE + 0x2d0)
#define CCU_REG_BUS_SOFT_RST4   (CCU_REG_BASE + 0x2d8)

#define DIV_ROUND_CLOSEST(x, divisor)(                \
{                                                     \
         typeof(divisor)__divisor = divisor;          \
         (((x)+ ((__divisor) / 2)) / (__divisor));    \
}                                                     \
)


static int ccu_bus_soft_reset(int channel, int shift)
{
    addr_t reg_addr;
    u32 reg;

    if (channel < 0 || channel > 4) {
        return -1;
    }

    switch (channel) {
    case 0:
        reg_addr = CCU_REG_BUS_SOFT_RST0;
        break;
    case 1:
        reg_addr = CCU_REG_BUS_SOFT_RST1;
        break;
    case 2:
        reg_addr = CCU_REG_BUS_SOFT_RST2;
        break;
    case 3:
        reg_addr = CCU_REG_BUS_SOFT_RST3;
        break;
    case 4:
        reg_addr = CCU_REG_BUS_SOFT_RST4;
        break;
    }

    //assert
    reg = readl(reg_addr);
    reg &= ~(1 << shift);
    writel(reg, reg_addr);

    usleep(10);

    //deassert
    reg = readl(reg_addr);
    reg |= (1 << shift);
    writel(reg, reg_addr);

    return 0;
}

static int ccu_bus_gate_enable(int channel, int shift, BOOL enable)
{
    u32 reg;
    addr_t reg_addr;

    if (channel < 0 || channel > 3) {
        return -1;
    }

    reg_addr = CCU_REG_BUS_CLK_GATE0 + 4 * channel;
    reg = readl(reg_addr);
    if (enable) {
        reg |= (1 << shift);
        writel(reg, reg_addr);
    } else {
        reg &= ~(1 << shift);
        writel(reg, reg_addr);
    }

    return 0;
}

static int ccu_sdmmc_clk_enable(int channel, BOOL enable)
{
    u32 reg;
    addr_t reg_addr;

    if (channel < 0 || channel > 3) {
        return -1;
    }

    reg_addr = CCU_REG_SDMMC0_CLK + 4 * channel;
    reg = readl(reg_addr);

    if (enable) {
        reg |= (1 << 31);
        writel(reg, reg_addr);
    } else {
        reg &= ~(1 << 31);
        writel(reg, reg_addr);
    }

    return 0;
}

int ccu_mmc_output_clk_set_phase(int channel, int degrees)
{
    int ratio_n, ratio_m;
    u32 reg;
    u8 delay;

    if (channel < 0 || channel > 2) {
        return -1;
    }

    ccu_sdmmc_clk_enable(channel, FALSE);

    reg = readl(CCU_REG_SDMMC0_CLK + 4 * channel);
    ratio_n = (reg >> 16) & 0x3;
    switch (ratio_n) {
    case 0:
        ratio_n = 1;
        break;
    case 1:
        ratio_n = 2;
        break;
    case 2:
        ratio_n = 4;
        break;
    case 3:
        ratio_n = 8;
        break;
    }

    ratio_m = (reg & 0xf) + 1;

    if (degrees != 180) {
        u16 step, clk_div;

        /* Get our parent divider */
        clk_div = ratio_n * ratio_m;

        /*
         * We can only outphase the clocks by multiple of the
         * PLL's period.
         *
         * Since our parent clock is only a divider, and the
         * formula to get the outphasing in degrees is deg =
         * 360 * delta / period
         *
         * If we simplify this formula, we can see that the
         * only thing that we're concerned about is the number
         * of period we want to outphase our clock from, and
         * the divider set by our parent clock.
         */
        step = DIV_ROUND_CLOSEST(360, clk_div);
        delay = DIV_ROUND_CLOSEST(degrees, step);
    } else {
        delay = 0;
    }

    reg &= ~(0x7 << 8);
    writel(reg | (delay << 8), CCU_REG_SDMMC0_CLK + 4 * channel);

    ccu_sdmmc_clk_enable(channel, TRUE);

    return 0;
}

int ccu_mmc_sample_clk_set_phase(int channel, int degrees)
{
    int ratio_n, ratio_m;
    u32 reg;
    u8 delay;

    if (channel < 0 || channel > 2) {
        return -1;
    }

    ccu_sdmmc_clk_enable(channel, FALSE);

    reg = readl(CCU_REG_SDMMC0_CLK + 4 * channel);
    ratio_n = (reg >> 16) & 0x3;
    switch (ratio_n) {
    case 0:
        ratio_n = 1;
        break;
    case 1:
        ratio_n = 2;
        break;
    case 2:
        ratio_n = 4;
        break;
    case 3:
        ratio_n = 8;
        break;
    }

    ratio_m = (reg & 0xf) + 1;

    if (degrees != 180) {
        u16 step, clk_div;

        /* Get our parent divider */
        clk_div = ratio_n * ratio_m;

        /*
         * We can only outphase the clocks by multiple of the
         * PLL's period.
         *
         * Since our parent clock is only a divider, and the
         * formula to get the outphasing in degrees is deg =
         * 360 * delta / period
         *
         * If we simplify this formula, we can see that the
         * only thing that we're concerned about is the number
         * of period we want to outphase our clock from, and
         * the divider set by our parent clock.
         */
        step = DIV_ROUND_CLOSEST(360, clk_div);
        delay = DIV_ROUND_CLOSEST(degrees, step);
    } else {
        delay = 0;
    }

    reg &= ~(0x7 << 20);
    writel(reg | (delay << 20), CCU_REG_SDMMC0_CLK + 4 * channel);

    ccu_sdmmc_clk_enable(channel, TRUE);

    return 0;
}

int ccu_bus_mmc_soft_reset(int channel)
{
    int shift;

    if (channel < 0 || channel > 2) {
        return -1;
    }

    switch (channel) {
    case 0:
        shift = 8;
        break;
    case 1:
        shift = 9;
        break;
    case 2:
        shift = 10;
        break;
    }

    ccu_bus_soft_reset(0, shift);

    return 0;
}

int ccu_mmc_clk_enable(int channel, BOOL enable)
{
    int shift;

    if (channel < 0 || channel > 2) {
        return -1;
    }

    switch (channel) {
    case 0:
        shift = 8;
        break;
    case 1:
        shift = 9;
        break;
    case 2:
        shift = 10;
        break;
    }

    ccu_bus_gate_enable(0, shift, enable);
    ccu_sdmmc_clk_enable(channel, enable);

    return 0;
}

int ccu_mmc_clk_rate_set(int channel, int rate)
{
    u32 reg;
    addr_t reg_addr;

    if (rate != 400000 && rate != 25000000 && rate != 50000000) {
        return -1;
    }

    ccu_sdmmc_clk_enable(channel, FALSE);

    reg_addr = CCU_REG_SDMMC0_CLK + 4 * channel;
    reg = 0;
    switch (rate) {
    case 400000:
        reg |= (2 << 16) | (14);
        break;
    case 25000000:
        reg |= (1 << 24) | (1 << 16) | (11);
        break;
    case 50000000:
        reg |= (1 << 24) | (0 << 16) | (11);
        break;
    }
    writel(reg, reg_addr);

    ccu_sdmmc_clk_enable(channel, TRUE);

    return 0;
}
