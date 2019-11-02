/*
 * Sunxi ohci glue
 *
 * Copyright (C) 2015 Hans de Goede <hdegoede@redhat.com>
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
#include <linux/unaligned.h>
#include <clock_ccm.h>
#include <usb_configs.h>

#include <usb_phy.h>
#include <usb.h>
#include "ohci.h"

int sunxi_ohci_usb_probe(void)
{
	struct sunxi_ccm_reg *ccm = (struct sunxi_ccm_reg *)SUNXI_CCM_BASE;

	setbits_le32(&ccm->ahb_gate0, 1 << AHB_GATE_OFFSET_USB_OHCI0);
	setbits_le32(&ccm->usb_clk_cfg, CCM_USB_CTRL_OHCI0_CLK);
#ifdef CONFIG_SUNXI_GEN_SUN6I
	setbits_le32(&ccm->ahb_reset0_cfg, 1 << AHB_GATE_OFFSET_USB_OHCI0);
#endif

	setbits_le32(&ccm->pll9_cfg, 1 << 31);

	sunxi_usb_phy_init(1);
	sunxi_usb_phy_power_on(1);

	return 0;
}

int sunxi_ohci_usb_remove(void)
{
	struct sunxi_ccm_reg *ccm = (struct sunxi_ccm_reg *)SUNXI_CCM_BASE;

	sunxi_usb_phy_power_off(1);
	sunxi_usb_phy_exit(1);

#ifdef CONFIG_SUNXI_GEN_SUN6I
	clrbits_le32(&ccm->ahb_reset0_cfg, 1 << AHB_GATE_OFFSET_USB_OHCI0);
#endif
	clrbits_le32(&ccm->usb_clk_cfg, CCM_USB_CTRL_OHCI0_CLK);
	clrbits_le32(&ccm->ahb_gate0, 1 << AHB_GATE_OFFSET_USB_OHCI0);

	return 0;
}
