/*
 * Sunxi usb-phy code
 *
 * Copyright (C) 2015 Hans de Goede <hdegoede@redhat.com>
 * Copyright (C) 2014 Roman Byshko <rbyshko@gmail.com>
 *
 * Based on code from
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#define __SYLIXOS_KERNEL
#include <SylixOS.h>
#include <linux/compat.h>
#include <linux/compat2.h>
#include <linux/types.h>
#include <linux/types2.h>
#include <arm/io.h>
#include <driver/pinmux/pinmux.h>
#include <linux/unaligned.h>
#include <usb_configs.h>

#include <usb_phy.h>
#include <clock_ccm.h>

#define SUNXI_USB_PMU_IRQ_ENABLE	0x800
#ifdef CONFIG_MACH_SUN8I_A33
#define SUNXI_USB_CSR			    0x410
#else
#define SUNXI_USB_CSR			    0x404
#endif
#define SUNXI_USB_PASSBY_EN		    1

#define SUNXI_EHCI_AHB_ICHR8_EN		    (1 << 10)
#define SUNXI_EHCI_AHB_INCR4_BURST_EN	(1 << 9)
#define SUNXI_EHCI_AHB_INCRX_ALIGN_EN	(1 << 8)
#define SUNXI_EHCI_ULPI_BYPASS_EN	    (1 << 0)

static struct sunxi_usb_phy {
	int usb_rst_mask;
	int id;
	int init_count;
	int power_on_count;
} sunxi_usb_phy[] = {
	{
		.usb_rst_mask = CCM_USB_CTRL_PHY0_RST | CCM_USB_CTRL_PHY0_CLK,
		.id = 0,
	},
	{
		.usb_rst_mask = CCM_USB_CTRL_PHY1_RST | CCM_USB_CTRL_PHY1_CLK,
		.id = 1,
	},
#if CONFIG_SUNXI_USB_PHYS >= 3
	{
		.usb_rst_mask = CCM_USB_CTRL_PHY2_RST | CCM_USB_CTRL_PHY2_CLK,
		.id = 2,
	}
#endif
};

static void usb_phy_write(struct sunxi_usb_phy *phy, int addr,
			  int data, int len)
{
	int j = 0, usbc_bit = 0;
	void *dest = (void *)SUNXI_USB0_BASE + SUNXI_USB_CSR;

#ifdef CONFIG_MACH_SUN8I_A33
	/* CSR needs to be explicitly initialized to 0 on A33 */
	writel(0, dest);
#endif

	usbc_bit = 1 << (phy->id * 2);
	for (j = 0; j < len; j++) {
		/* set the bit address to be written */
		clrbits_le32(dest, 0xff << 8);
		setbits_le32(dest, (addr + j) << 8);

		clrbits_le32(dest, usbc_bit);
		/* set data bit */
		if (data & 0x1)
			setbits_le32(dest, 1 << 7);
		else
			clrbits_le32(dest, 1 << 7);

		setbits_le32(dest, usbc_bit);

		clrbits_le32(dest, usbc_bit);

		data >>= 1;
	}
}

static void sunxi_usb_phy_config(struct sunxi_usb_phy *phy)
{
	/* The following comments are machine
	 * translated from Chinese, you have been warned!
	 */

	/* Regulation 45 ohms */
	if (phy->id == 0)
		usb_phy_write(phy, 0x0c, 0x01, 1);

	/* adjust PHY's magnitude and rate */
	usb_phy_write(phy, 0x20, 0x14, 5);

	/* threshold adjustment disconnect */
#if defined CONFIG_MACH_SUN5I || defined CONFIG_MACH_SUN7I
	usb_phy_write(phy, 0x2a, 2, 2);
#else
	usb_phy_write(phy, 0x2a, 3, 2);
#endif

	return;
}

static void sunxi_usb_phy_passby(int index, int enable)
{
	unsigned long bits = 0;
	void *addr;

	if (index == 1)
		addr = (void *)SUNXI_USB1_BASE + SUNXI_USB_PMU_IRQ_ENABLE;

	bits = SUNXI_EHCI_AHB_ICHR8_EN |
		SUNXI_EHCI_AHB_INCR4_BURST_EN |
		SUNXI_EHCI_AHB_INCRX_ALIGN_EN |
		SUNXI_EHCI_ULPI_BYPASS_EN;

	if (enable)
		setbits_le32(addr, bits);
	else
		clrbits_le32(addr, bits);

	return;
}

void sunxi_usb_phy_enable_squelch_detect(int index, int enable)
{
	struct sunxi_usb_phy *phy = &sunxi_usb_phy[index];

	usb_phy_write(phy, 0x3c, enable ? 0 : 2, 2);
}

void sunxi_usb_phy_init(int index)
{
	struct sunxi_usb_phy *phy = &sunxi_usb_phy[index];
	struct sunxi_ccm_reg *ccm = (struct sunxi_ccm_reg *)SUNXI_CCM_BASE;

	phy->init_count++;
	if (phy->init_count != 1)
		return;

	setbits_le32(&ccm->usb_clk_cfg, phy->usb_rst_mask);

	sunxi_usb_phy_config(phy);

	if (phy->id != 0)
		sunxi_usb_phy_passby(index, SUNXI_USB_PASSBY_EN);
}

void sunxi_usb_phy_exit(int index)
{
	struct sunxi_usb_phy *phy = &sunxi_usb_phy[index];
	struct sunxi_ccm_reg *ccm = (struct sunxi_ccm_reg *)SUNXI_CCM_BASE;

	phy->init_count--;
	if (phy->init_count != 0)
		return;

	if (phy->id != 0)
		sunxi_usb_phy_passby(index, !SUNXI_USB_PASSBY_EN);

	clrbits_le32(&ccm->usb_clk_cfg, phy->usb_rst_mask);
}

void sunxi_usb_phy_power_on(int index)
{
	struct sunxi_usb_phy *phy = &sunxi_usb_phy[index];

	phy->power_on_count++;
	if (phy->power_on_count != 1)
		return;

	API_GpioSetValue(GPIO_L_02, 1);
}

void sunxi_usb_phy_power_off(int index)
{
	struct sunxi_usb_phy *phy = &sunxi_usb_phy[index];

	phy->power_on_count--;
	if (phy->power_on_count != 0)
		return;

	API_GpioSetValue(GPIO_L_02, 0);
}

int sunxi_usb_phy_probe(void)
{
	struct sunxi_ccm_reg *ccm = (struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	int ret = 0;

    ret = API_GpioRequestOne(GPIO_L_02, LW_GPIOF_OUT_INIT_LOW, "usb1_vbus");
    if (ret != ERROR_NONE) {
        _PrintFormat("Request usb1_vbus gpio failed!\r\n");
        return -1;
    }

	setbits_le32(&ccm->usb_clk_cfg, CCM_USB_CTRL_PHYGATE);

	return 0;
}

int sunxi_usb_phy_remove(void)
{
	struct sunxi_ccm_reg *ccm = (struct sunxi_ccm_reg *)SUNXI_CCM_BASE;

	clrbits_le32(&ccm->usb_clk_cfg, CCM_USB_CTRL_PHYGATE);

    API_GpioFree(GPIO_L_02);

	return 0;
}
