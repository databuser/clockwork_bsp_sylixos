/*
 * usb_cmd.c
 *
 *  Created on: Oct 23, 2019
 *      Author: databus
 */

#define __SYLIXOS_KERNEL
#include <SylixOS.h>
#include <module.h>
#include <linux/compat.h>
#include <linux/compat2.h>
#include <linux/types.h>
#include <linux/types2.h>
#include <arm/io.h>
#include <linux/unaligned.h>
#include <usb_configs.h>

#include <usb.h>
#include <usb_phy.h>
#include <ohci_sunxi.h>

#define debug(format, arg...) printf(format, ## arg)

/* some display routines (info command) */
static char *usb_get_class_desc(unsigned char dclass)
{
    switch (dclass) {
    case USB_CLASS_PER_INTERFACE:
        return "See Interface";
    case USB_CLASS_AUDIO:
        return "Audio";
    case USB_CLASS_COMM:
        return "Communication";
    case USB_CLASS_HID:
        return "Human Interface";
    case USB_CLASS_PRINTER:
        return "Printer";
    case USB_CLASS_MASS_STORAGE:
        return "Mass Storage";
    case USB_CLASS_HUB:
        return "Hub";
    case USB_CLASS_DATA:
        return "CDC Data";
    case USB_CLASS_VENDOR_SPEC:
        return "Vendor specific";
    default:
        return "";
    }
}

static void usb_display_class_sub(unsigned char dclass, unsigned char subclass,
                  unsigned char proto)
{
    switch (dclass) {
    case USB_CLASS_PER_INTERFACE:
        debug("See Interface");
        break;
    case USB_CLASS_HID:
        debug("Human Interface, Subclass: ");
        switch (subclass) {
        case USB_SUB_HID_NONE:
            debug("None");
            break;
        case USB_SUB_HID_BOOT:
            debug("Boot ");
            switch (proto) {
            case USB_PROT_HID_NONE:
                debug("None");
                break;
            case USB_PROT_HID_KEYBOARD:
                debug("Keyboard");
                break;
            case USB_PROT_HID_MOUSE:
                debug("Mouse");
                break;
            default:
                debug("reserved");
                break;
            }
            break;
        default:
            debug("reserved");
            break;
        }
        break;
    case USB_CLASS_MASS_STORAGE:
        debug("Mass Storage, ");
        switch (subclass) {
        case US_SC_RBC:
            debug("RBC ");
            break;
        case US_SC_8020:
            debug("SFF-8020i (ATAPI)");
            break;
        case US_SC_QIC:
            debug("QIC-157 (Tape)");
            break;
        case US_SC_UFI:
            debug("UFI");
            break;
        case US_SC_8070:
            debug("SFF-8070");
            break;
        case US_SC_SCSI:
            debug("Transp. SCSI");
            break;
        default:
            debug("reserved");
            break;
        }
        debug(", ");
        switch (proto) {
        case US_PR_CB:
            debug("Command/Bulk");
            break;
        case US_PR_CBI:
            debug("Command/Bulk/Int");
            break;
        case US_PR_BULK:
            debug("Bulk only");
            break;
        default:
            debug("reserved");
            break;
        }
        break;
    default:
        debug("%s", usb_get_class_desc(dclass));
        break;
    }
}

static void usb_display_string(struct usb_device *dev, int index)
{
    ALLOC_CACHE_ALIGN_BUFFER(char, buffer, 256);

    if (index != 0) {
        if (usb_string(dev, index, &buffer[0], 256) > 0)
            debug("String: \"%s\"", buffer);
    }
}

static void usb_display_desc(struct usb_device *dev)
{
    if (dev->descriptor.bDescriptorType == USB_DT_DEVICE) {
        debug("%d: %s,  USB Revision %x.%x\r\n", dev->devnum,
        usb_get_class_desc(dev->config.if_desc[0].desc.bInterfaceClass),
                   (dev->descriptor.bcdUSB>>8) & 0xff,
                   dev->descriptor.bcdUSB & 0xff);

        if (strlen(dev->mf) || strlen(dev->prod) ||
            strlen(dev->serial))
            debug(" - %s %s %s\r\n", dev->mf, dev->prod,
                dev->serial);
        if (dev->descriptor.bDeviceClass) {
            debug(" - Class: ");
            usb_display_class_sub(dev->descriptor.bDeviceClass,
                          dev->descriptor.bDeviceSubClass,
                          dev->descriptor.bDeviceProtocol);
            debug("\r\n");
        } else {
            debug(" - Class: (from Interface) %s\r\n",
                   usb_get_class_desc(
                dev->config.if_desc[0].desc.bInterfaceClass));
        }
        debug(" - PacketSize: %d  Configurations: %d\r\n",
            dev->descriptor.bMaxPacketSize0,
            dev->descriptor.bNumConfigurations);
        debug(" - Vendor: 0x%04x  Product 0x%04x Version %d.%d\r\n",
            dev->descriptor.idVendor, dev->descriptor.idProduct,
            (dev->descriptor.bcdDevice>>8) & 0xff,
            dev->descriptor.bcdDevice & 0xff);
    }

}

static void usb_display_conf_desc(struct usb_config_descriptor *config,
                  struct usb_device *dev)
{
    debug("   Configuration: %d\r\n", config->bConfigurationValue);
    debug("   - Interfaces: %d %s%s%dmA\r\n", config->bNumInterfaces,
           (config->bmAttributes & 0x40) ? "Self Powered " : "Bus Powered ",
           (config->bmAttributes & 0x20) ? "Remote Wakeup " : "",
        config->bMaxPower*2);
    if (config->iConfiguration) {
        debug("   - ");
        usb_display_string(dev, config->iConfiguration);
        debug("\r\n");
    }
}

static void usb_display_if_desc(struct usb_interface_descriptor *ifdesc,
                struct usb_device *dev)
{
    debug("     Interface: %d\r\n", ifdesc->bInterfaceNumber);
    debug("     - Alternate Setting %d, Endpoints: %d\r\n",
        ifdesc->bAlternateSetting, ifdesc->bNumEndpoints);
    debug("     - Class ");
    usb_display_class_sub(ifdesc->bInterfaceClass,
        ifdesc->bInterfaceSubClass, ifdesc->bInterfaceProtocol);
    debug("\r\n");
    if (ifdesc->iInterface) {
        debug("     - ");
        usb_display_string(dev, ifdesc->iInterface);
        debug("\r\n");
    }
}

static void usb_display_ep_desc(struct usb_endpoint_descriptor *epdesc)
{
    debug("     - Endpoint %d %s ", epdesc->bEndpointAddress & 0xf,
        (epdesc->bEndpointAddress & 0x80) ? "In" : "Out");
    switch ((epdesc->bmAttributes & 0x03)) {
    case 0:
        debug("Control");
        break;
    case 1:
        debug("Isochronous");
        break;
    case 2:
        debug("Bulk");
        break;
    case 3:
        debug("Interrupt");
        break;
    }
    debug(" MaxPacket %d", get_unaligned(&epdesc->wMaxPacketSize));
    if ((epdesc->bmAttributes & 0x03) == 0x3)
        debug(" Interval %dms", epdesc->bInterval);
    debug("\r\n");
}

/* main routine to diasplay the configs, interfaces and endpoints */
static void usb_display_config(struct usb_device *dev)
{
    struct usb_config *config;
    struct usb_interface *ifdesc;
    struct usb_endpoint_descriptor *epdesc;
    int i, ii;

    config = &dev->config;
    usb_display_conf_desc(&config->desc, dev);
    for (i = 0; i < config->no_of_if; i++) {
        ifdesc = &config->if_desc[i];
        usb_display_if_desc(&ifdesc->desc, dev);
        for (ii = 0; ii < ifdesc->no_of_ep; ii++) {
            epdesc = &ifdesc->ep_desc[ii];
            usb_display_ep_desc(epdesc);
        }
    }
    debug("\r\n");
}

/*
 * With driver model this isn't right since we can have multiple controllers
 * and the device numbering starts at 1 on each bus.
 * TODO(sjg@chromium.org): Add a way to specify the controller/bus.
 */
static struct usb_device *usb_find_device(int devnum)
{
    struct usb_device *udev;
    int d;

    for (d = 0; d < USB_MAX_DEVICE; d++) {
        udev = usb_get_dev_index(d);
        if (udev == NULL)
            return NULL;
        if (udev->devnum == devnum)
            return udev;
    }

    return NULL;
}

static inline char *portspeed(int speed)
{
    char *speed_str;

    switch (speed) {
    case USB_SPEED_SUPER:
        speed_str = "5 Gb/s";
        break;
    case USB_SPEED_HIGH:
        speed_str = "480 Mb/s";
        break;
    case USB_SPEED_LOW:
        speed_str = "1.5 Mb/s";
        break;
    default:
        speed_str = "12 Mb/s";
        break;
    }

    return speed_str;
}

/* shows the device tree recursively */
static void usb_show_tree_graph(struct usb_device *dev, char *pre)
{
    int index;
    int has_child, last_child;

    index = strlen(pre);
    debug(" %s", pre);

    /* check if the device has connected children */
    int i;

    has_child = 0;
    for (i = 0; i < dev->maxchild; i++) {
        if (dev->children[i] != NULL)
            has_child = 1;
    }

    /* check if we are the last one */
    last_child = (dev->parent != NULL);

    if (last_child) {
        for (i = 0; i < dev->parent->maxchild; i++) {
            /* search for children */
            if (dev->parent->children[i] == dev) {
                /* found our pointer, see if we have a
                 * little sister
                 */
                while (i++ < dev->parent->maxchild) {
                    if (dev->parent->children[i] != NULL) {
                        /* found a sister */
                        last_child = 0;
                        break;
                    } /* if */
                } /* while */
            } /* device found */
        } /* for all children of the parent */

        debug("\b+-");
        /* correct last child */
        if (last_child && index)
            pre[index-1] = ' ';
    } /* if not root hub */
    else
        debug(" ");
    debug("%d ", dev->devnum);
    pre[index++] = ' ';
    pre[index++] = has_child ? '|' : ' ';
    pre[index] = 0;
    debug(" %s (%s, %dmA)\r\n", usb_get_class_desc(
                    dev->config.if_desc[0].desc.bInterfaceClass),
                    portspeed(dev->speed),
                    dev->config.desc.bMaxPower * 2);
    if (strlen(dev->mf) || strlen(dev->prod) || strlen(dev->serial))
        debug(" %s  %s %s %s\r\n", pre, dev->mf, dev->prod, dev->serial);
    debug(" %s\r\n", pre);

    if (dev->maxchild > 0) {
        for (i = 0; i < dev->maxchild; i++) {
            if (dev->children[i] != NULL) {
                usb_show_tree_graph(dev->children[i], pre);
                pre[index] = 0;
            }
        }
    }
}

/* main routine for the tree command */
static void usb_show_tree(struct usb_device *dev)
{
    char preamble[32];

    memset(preamble, '\0', sizeof(preamble));
    usb_show_tree_graph(dev, &preamble[0]);
}

int do_usb_tree (int argc, char **argv)
{
    int i;
    struct usb_device *udev = NULL;

    debug("USB device tree:\r\n");

    for (i = 0; i < USB_MAX_DEVICE; i++) {
        udev = usb_get_dev_index(i);
        if (udev == NULL)
            break;
        if (udev->parent == NULL)
            usb_show_tree(udev);
    }

    return 0;
}

int do_usb_info (int argc, char **argv)
{
    int i;
    struct usb_device *udev = NULL;

    if (argc == 2) {
        int d;
        for (d = 0; d < USB_MAX_DEVICE; d++) {
            udev = usb_get_dev_index(d);
            if (udev == NULL)
                break;
            usb_display_desc(udev);
            usb_display_config(udev);
        }
        return 0;
    } else {
        /*
         * With driver model this isn't right since we can
         * have multiple controllers and the device numbering
         * starts at 1 on each bus.
         */
        i = strtoul(argv[2], NULL, 10);
        debug("config for device %d\r\n", i);
        udev = usb_find_device(i);
        if (udev == NULL) {
            debug("*** No device available ***\r\n");
            return 0;
        } else {
            usb_display_desc(udev);
            usb_display_config(udev);
        }
    }

    return 0;
}
