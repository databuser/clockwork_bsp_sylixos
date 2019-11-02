/*
 * config.h
 *
 *  Created on: Oct 22, 2019
 *      Author: databus
 */

#ifndef SYLIXOS_DRIVER_KEYPAD_CONFIG_H_
#define SYLIXOS_DRIVER_KEYPAD_CONFIG_H_

#define CONFIG_SYS_USB_OHCI_MAX_ROOT_PORTS  1
#define CONFIG_SYS_USB_OHCI_REGS_BASE       (0x1c1a000 + 0x400)
#define CONFIG_SYS_USB_OHCI_SLOT_NAME       "sunxi-r16-ohci"

#define CONFIG_MACH_SUN8I_A33               1
#define CONFIG_SUNXI_GEN_SUN6I              1
#define SUNXI_USB0_BASE                     (0x01c19000)    //USB_OTG
#define SUNXI_USB1_BASE                     (0x01c1a000)    //USB_OHCI
#define CONFIG_SUNXI_USB_PHYS               2
#define SUNXI_CCM_BASE                      (0x01c20000)

//#define DEBUG                               1
//#define SHOW_INFO                           1
//#define OHCI_VERBOSE_DEBUG                  1
//#define OHCI_FILL_TRACE                     1

#endif /* SYLIXOS_DRIVER_KEYPAD_CONFIG_H_ */
