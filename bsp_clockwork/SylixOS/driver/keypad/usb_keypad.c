/*
 * (C) Copyright 2001
 * Denis Peter, MPL AG Switzerland
 *
 * Part of this source has been derived from the Linux USB
 * project.
 *
 * SPDX-License-Identifier:	GPL-2.0+
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
#include <keyboard.h>

#include <usb.h>
#include <usb_phy.h>
#include <ohci_sunxi.h>

#ifdef DEBUG
#define debug(format, arg...) _PrintFormat("KEYPAD DEBUG: " format, ## arg)
#else
#define debug(format, arg...)
#endif

/* Keyboard sampling rate */
#define REPEAT_RATE     40      /* 40msec -> 25cps */
#define REPEAT_DELAY    10      /* 10 x REPEAT_RATE = 400msec */

#define NUM_LOCK    0x53
#define CAPS_LOCK   0x39
#define SCROLL_LOCK 0x47

/* Modifier bits */
#define LEFT_CNTR   (1 << 0)
#define LEFT_SHIFT  (1 << 1)
#define LEFT_ALT    (1 << 2)
#define LEFT_GUI    (1 << 3)
#define RIGHT_CNTR  (1 << 4)
#define RIGHT_SHIFT (1 << 5)
#define RIGHT_ALT   (1 << 6)
#define RIGHT_GUI   (1 << 7)

/* Size of the keyboard buffer */
#define USB_KBD_BUFFER_LEN  0x20

struct usb_device *dpad_dev;
extern LW_HANDLE keypad_queue;
extern LW_SEL_WAKEUPLIST select_list;

/*
 * NOTE: It's important for the NUM, CAPS, SCROLL-lock bits to be in this
 *       order. See usb_kbd_setled() function!
 */
#define USB_KBD_NUMLOCK     (1 << 0)
#define USB_KBD_CAPSLOCK    (1 << 1)
#define USB_KBD_SCROLLLOCK  (1 << 2)
#define USB_KBD_CTRL        (1 << 3)

#define USB_KBD_LEDMASK     \
    (USB_KBD_NUMLOCK | USB_KBD_CAPSLOCK | USB_KBD_SCROLLLOCK)

/*
 * USB Keyboard reports are 8 bytes in boot protocol.
 * Appendix B of HID Device Class Definition 1.11
 */
#define USB_KBD_BOOT_REPORT_SIZE 8

struct usb_kbd_pdata {
    unsigned long   intpipe;
    int     intpktsize;
    int     intinterval;
    unsigned long   last_report;
    struct int_queue *intq;

    uint32_t    repeat_delay;

    uint32_t    usb_in_pointer;
    uint32_t    usb_out_pointer;
    uint8_t     usb_kbd_buffer[USB_KBD_BUFFER_LEN];

    uint8_t     *new;
    uint8_t     old[USB_KBD_BOOT_REPORT_SIZE];

    uint8_t     flags;
};

/**
 * memscan - Find a character in an area of memory.
 * @addr: The memory area
 * @c: The byte to search for
 * @size: The size of the area.
 *
 * returns the address of the first occurrence of @c, or 1 byte past
 * the area if @c is not found
 */
void * memscan(void * addr, int c, size_t size)
{
    unsigned char * p = (unsigned char *) addr;

    while (size) {
        if (*p == c)
            return (void *) p;
        p++;
        size--;
    }
    return (void *) p;
}

/* Interrupt service routine */
static int usb_kbd_irq_worker(struct usb_device *dev)
{
    int i;
    struct usb_kbd_pdata *kbd = dev->privptr;
    keyboard_event_notify kbd_event = {1,};

    usb_submit_int_msg(dev, kbd->intpipe, kbd->new, kbd->intpktsize, kbd->intinterval);

    for (i = 1; i < 8; i++) {
        //release key
        if (kbd->old[i] > 0 && memscan(kbd->new + 1, kbd->old[i], 7) == kbd->new + 8) {
            debug("key release,key = %x\n", kbd->old[i]);

            //dpad on gameshell send 0x2 keycode after usb init,but why???
            //we dont want this keycode
            if (kbd->old[i] == 0x2)
                continue;

            kbd_event.type = KE_RELEASE;
            kbd_event.keymsg[2] = kbd->old[i];
            API_MsgQueueSend(keypad_queue, &kbd_event, sizeof(keyboard_event_notify));
            SEL_WAKE_UP_ALL(&select_list, SELREAD);
        }

        //press key
        if (kbd->new[i] > 0 && memscan(kbd->old + 1, kbd->new[i], 7) == kbd->old + 8) {
            debug("key press,key = %x\n", kbd->new[i]);

            kbd_event.type = KE_PRESS;
            kbd_event.keymsg[2] = kbd->new[i];
            API_MsgQueueSend(keypad_queue, &kbd_event, sizeof(keyboard_event_notify));
            SEL_WAKE_UP_ALL(&select_list, SELREAD);
        }
    }

    //update key state
    memcpy(kbd->old, kbd->new, 8);

    return 0;
}

/* Keyboard interrupt handler */
static int usb_kbd_irq(struct usb_device *dev)
{
    return usb_kbd_irq_worker(dev);
}

/* probes the USB device dev for keyboard type. */
static int usb_kbd_probe(struct usb_device *dev, unsigned int ifnum)
{
    struct usb_interface *iface;
    struct usb_endpoint_descriptor *ep;
    struct usb_kbd_pdata *data;

    if (dev->descriptor.bNumConfigurations != 1)
        return 0;

    iface = &dev->config.if_desc[ifnum];

    if (iface->desc.bInterfaceClass != 3)
        return 0;

#if 0
    if (iface->desc.bInterfaceSubClass != 1)
        return 0;

    if (iface->desc.bInterfaceProtocol != 1)
        return 0;
#endif

    if (iface->desc.bNumEndpoints != 1)
        return 0;

    ep = &iface->ep_desc[0];

    /* Check if endpoint 1 is interrupt endpoint */
    if (!(ep->bEndpointAddress & 0x80))
        return 0;

    if ((ep->bmAttributes & 3) != 3)
        return 0;

    debug("USB KBD: found set protocol...\r\n");

    data = malloc(sizeof(struct usb_kbd_pdata));
    if (!data) {
        debug("USB KBD: Error allocating private data\r\n");
        return 0;
    }

    /* Clear private data */
    memset(data, 0, sizeof(struct usb_kbd_pdata));

    /* allocate input buffer aligned and sized to USB DMA alignment */
    data->new = API_VmmDmaAllocAlign(roundup(USB_KBD_BOOT_REPORT_SIZE, USB_DMA_MINALIGN), USB_DMA_MINALIGN);

    /* Insert private data into USB device structure */
    dev->privptr = data;

#if 1
    /* Set IRQ handler */
    dev->irq_handle = usb_kbd_irq;
#endif

    data->intpipe = usb_rcvintpipe(dev, ep->bEndpointAddress);
    data->intpktsize = min(usb_maxpacket(dev, data->intpipe),
                   USB_KBD_BOOT_REPORT_SIZE);
    data->intinterval = 1;
    data->last_report = -1;

    /* We found a USB Keyboard, install it. */
    usb_set_protocol(dev, iface->desc.bInterfaceNumber, 0);

    debug("USB KBD: found set idle...\r\n");
#if !defined(CONFIG_SYS_USB_EVENT_POLL_VIA_CONTROL_EP) && \
    !defined(CONFIG_SYS_USB_EVENT_POLL_VIA_INT_QUEUE)
    usb_set_idle(dev, iface->desc.bInterfaceNumber, REPEAT_RATE / 4, 0);
#else
    usb_set_idle(dev, iface->desc.bInterfaceNumber, 0, 0);
#endif

    /* Success. */
    return 1;
}

static int probe_usb_keyboard(struct usb_device *dev)
{
    /* Try probing the keyboard */
    if (usb_kbd_probe(dev, 0) != 1)
        return -ENOENT;

    dpad_dev = dev;
    return 0;
}

/* Search for keypad and register it if found. */
int drv_usb_kbd_init(void)
{
    int error, i;

    debug("%s: Probing for keyboard\r\n", __func__);

    /* Scan all USB Devices */
    for (i = 0; i < USB_MAX_DEVICE; i++) {
        struct usb_device *dev;

        /* Get USB device. */
        dev = usb_get_dev_index(i);
        if (!dev)
            break;

        if (dev->devnum == -1)
            continue;

        error = probe_usb_keyboard(dev);
        if (!error)
            return 1;
        if (error && error != -ENOENT)
            return error;
    }

    /* No USB Keypad found */
    return -1;
}

extern int do_usb_tree (int argc, char **argv);
extern int do_usb_info (int argc, char **argv);
extern INT  __devRegister(VOID);
int keypad_init (void)
{
    API_TShellKeywordAdd("usbtree", do_usb_tree);
    API_TShellKeywordAdd("usbinfo", do_usb_info);
    __devRegister();

    sunxi_usb_phy_probe();
    sunxi_ohci_usb_probe();
    usb_init();
    drv_usb_kbd_init();

    return 0;
}
