/*
 * Most of this source has been derived from the Linux USB
 * project:
 * (C) Copyright Linus Torvalds 1999
 * (C) Copyright Johannes Erdfelt 1999-2001
 * (C) Copyright Andreas Gal 1999
 * (C) Copyright Gregory P. Smith 1999
 * (C) Copyright Deti Fliegl 1999 (new USB architecture)
 * (C) Copyright Randy Dunlap 2000
 * (C) Copyright David Brownell 2000 (kernel hotplug, usb_device_id)
 * (C) Copyright Yggdrasil Computing, Inc. 2000
 *     (usb_device_id matching changes by Adam J. Richter)
 *
 * Adapted for U-Boot:
 * (C) Copyright 2001 Denis Peter, MPL AG Switzerland
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

/****************************************************************************
 * HUB "Driver"
 * Probes device for being a hub and configurate it
 */

#define __SYLIXOS_KERNEL
#include <SylixOS.h>
#include <linux/compat.h>
#include <linux/compat2.h>
#include <linux/types.h>
#include <linux/types2.h>
#include <arm/io.h>
#include <linux/unaligned.h>
#include <usb_configs.h>

#include <usb.h>

//#define DEBUG
#ifdef DEBUG
#define debug(format, arg...) _PrintFormat("HUB DEBUG: " format, ## arg)
#else
#define debug(format, arg...)
#endif

#define USB_BUFSIZ	512

/* TODO(sjg@chromium.org): Remove this when CONFIG_DM_USB is defined */
static struct usb_hub_device hub_dev[USB_MAX_HUB];
static int usb_hub_index;

void usb_hub_reset_devices(int port)
{
	return;
}

static int usb_get_hub_descriptor(struct usb_device *dev, void *data, int size)
{
	return usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
		USB_REQ_GET_DESCRIPTOR, USB_DIR_IN | USB_RT_HUB,
		USB_DT_HUB << 8, 0, data, size, USB_CNTL_TIMEOUT);
}

static int usb_clear_port_feature(struct usb_device *dev, int port, int feature)
{
	return usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
				USB_REQ_CLEAR_FEATURE, USB_RT_PORT, feature,
				port, NULL, 0, USB_CNTL_TIMEOUT);
}

static int usb_set_port_feature(struct usb_device *dev, int port, int feature)
{
	return usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
				USB_REQ_SET_FEATURE, USB_RT_PORT, feature,
				port, NULL, 0, USB_CNTL_TIMEOUT);
}

static int usb_get_hub_status(struct usb_device *dev, void *data)
{
	return usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
			USB_REQ_GET_STATUS, USB_DIR_IN | USB_RT_HUB, 0, 0,
			data, sizeof(struct usb_hub_status), USB_CNTL_TIMEOUT);
}

int usb_get_port_status(struct usb_device *dev, int port, void *data)
{
	return usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
			USB_REQ_GET_STATUS, USB_DIR_IN | USB_RT_PORT, 0, port,
			data, sizeof(struct usb_hub_status), USB_CNTL_TIMEOUT);
}


static void usb_hub_power_on(struct usb_hub_device *hub)
{
	int i;
	struct usb_device *dev;
	unsigned pgood_delay = hub->desc.bPwrOn2PwrGood * 2;

	dev = hub->pusb_dev;

	debug("enabling power on all ports\r\n");
	for (i = 0; i < dev->maxchild; i++) {
		usb_set_port_feature(dev, i + 1, USB_PORT_FEAT_POWER);
		debug("port %d returns %lx\r\n", i + 1, dev->status);
	}

	/*
	 * Wait for power to become stable,
	 * plus spec-defined max time for device to connect
	 * but allow this time to be increased via env variable as some
	 * devices break the spec and require longer warm-up times
	 */
//    pgood_delay = 100; //700 is good for SylixOS
	debug("pgood_delay=%dms\r\n", pgood_delay);
	mdelay(pgood_delay);
}

void usb_hub_reset(void)
{
	usb_hub_index = 0;
}

static struct usb_hub_device *usb_hub_allocate(void)
{
	if (usb_hub_index < USB_MAX_HUB)
		return &hub_dev[usb_hub_index++];

	_PrintFormat("ERROR: USB_MAX_HUB (%d) reached\r\n", USB_MAX_HUB);
	return NULL;
}

#define MAX_TRIES 10

static inline char *portspeed(int portstatus)
{
	char *speed_str;

	switch (portstatus & USB_PORT_STAT_SPEED_MASK) {
	case USB_PORT_STAT_SUPER_SPEED:
		speed_str = "5 Gb/s";
		break;
	case USB_PORT_STAT_HIGH_SPEED:
		speed_str = "480 Mb/s";
		break;
	case USB_PORT_STAT_LOW_SPEED:
		speed_str = "1.5 Mb/s";
		break;
	default:
		speed_str = "12 Mb/s";
		break;
	}

	return speed_str;
}

int legacy_hub_port_reset(struct usb_device *dev, int port,
			unsigned short *portstat)
{
	int err, tries;
	ALLOC_CACHE_ALIGN_BUFFER(struct usb_port_status, portsts, 1);
	unsigned short portstatus, portchange;

	debug("%s: resetting port %d...\r\n", __func__, port + 1);

	for (tries = 0; tries < MAX_TRIES; tries++) {
		err = usb_set_port_feature(dev, port + 1, USB_PORT_FEAT_RESET);
		if (err < 0) {
		    API_VmmDmaFree(portsts);
			return err;
		}

		mdelay(200);

		if (usb_get_port_status(dev, port + 1, portsts) < 0) {
			debug("get_port_status failed status %lx\r\n",
			      dev->status);
			API_VmmDmaFree(portsts);
			return -1;
		}
		portstatus = le16_to_cpu(portsts->wPortStatus);
		portchange = le16_to_cpu(portsts->wPortChange);

		debug("portstatus %x, change %x, %s\r\n", portstatus, portchange,
							portspeed(portstatus));

		debug("STAT_C_CONNECTION = %d STAT_CONNECTION = %d" \
		      "  USB_PORT_STAT_ENABLE %d\r\n",
		      (portchange & USB_PORT_STAT_C_CONNECTION) ? 1 : 0,
		      (portstatus & USB_PORT_STAT_CONNECTION) ? 1 : 0,
		      (portstatus & USB_PORT_STAT_ENABLE) ? 1 : 0);

		/*
		 * Perhaps we should check for the following here:
		 * - C_CONNECTION hasn't been set.
		 * - CONNECTION is still set.
		 *
		 * Doing so would ensure that the device is still connected
		 * to the bus, and hasn't been unplugged or replaced while the
		 * USB bus reset was going on.
		 *
		 * However, if we do that, then (at least) a San Disk Ultra
		 * USB 3.0 16GB device fails to reset on (at least) an NVIDIA
		 * Tegra Jetson TK1 board. For some reason, the device appears
		 * to briefly drop off the bus when this second bus reset is
		 * executed, yet if we retry this loop, it'll eventually come
		 * back after another reset or two.
		 */

		if (portstatus & USB_PORT_STAT_ENABLE)
			break;

		mdelay(200);
	}

	if (tries == MAX_TRIES) {
		debug("Cannot enable port %i after %i retries, " \
		      "disabling port.\r\n", port + 1, MAX_TRIES);
		debug("Maybe the USB cable is bad?\r\n");
		API_VmmDmaFree(portsts);
		return -1;
	}

	usb_clear_port_feature(dev, port + 1, USB_PORT_FEAT_C_RESET);
	*portstat = portstatus;
	API_VmmDmaFree(portsts);

	return 0;
}

int usb_hub_port_connect_change(struct usb_device *dev, int port)
{
	ALLOC_CACHE_ALIGN_BUFFER(struct usb_port_status, portsts, 1);
	unsigned short portstatus;
	int ret, speed;

	/* Check status */
	ret = usb_get_port_status(dev, port + 1, portsts);
	if (ret < 0) {
		debug("get_port_status failed\r\n");
		API_VmmDmaFree(portsts);
		return ret;
	}

	portstatus = le16_to_cpu(portsts->wPortStatus);
	debug("portstatus %x, change %x, %s\r\n",
	      portstatus,
	      le16_to_cpu(portsts->wPortChange),
	      portspeed(portstatus));

#if 0
	/* Clear the connection change status */
	usb_clear_port_feature(dev, port + 1, USB_PORT_FEAT_C_CONNECTION);

	/* Disconnect any existing devices under this port */
	if (((!(portstatus & USB_PORT_STAT_CONNECTION)) &&
	     (!(portstatus & USB_PORT_STAT_ENABLE))) ||
	    usb_device_has_child_on_port(dev, port)) {
		debug("usb_disconnect(&hub->children[port]);\r\n");
		/* Return now if nothing is connected */
		if (!(portstatus & USB_PORT_STAT_CONNECTION)) {
		    API_VmmDmaFree(portsts);
			return -ENOTCONN;
		}
	}
	mdelay(200);
#endif

	/* Reset the port */
	ret = legacy_hub_port_reset(dev, port, &portstatus);
	if (ret < 0) {
		if (ret != -ENXIO)
		    _PrintFormat("cannot reset port %d!?\r\n", port + 1);
		API_VmmDmaFree(portsts);
		return ret;
	}

	mdelay(200);

	switch (portstatus & USB_PORT_STAT_SPEED_MASK) {
	case USB_PORT_STAT_SUPER_SPEED:
		speed = USB_SPEED_SUPER;
		break;
	case USB_PORT_STAT_HIGH_SPEED:
		speed = USB_SPEED_HIGH;
		break;
	case USB_PORT_STAT_LOW_SPEED:
		speed = USB_SPEED_LOW;
		break;
	default:
		speed = USB_SPEED_FULL;
		break;
	}

	struct usb_device *usb;

	ret = usb_alloc_new_device(dev->controller, &usb);
	if (ret) {
	    _PrintFormat("cannot create new device: ret=%d\r\n", ret);
	    API_VmmDmaFree(portsts);
		return ret;
	}

	dev->children[port] = usb;
	usb->speed = speed;
	usb->parent = dev;
	usb->portnr = port + 1;
	/* Run it through the hoops (find a driver, etc) */
	ret = usb_new_device(usb);
	if (ret < 0) {
		/* Woops, disable the port */
		usb_free_device(dev->controller);
		dev->children[port] = NULL;
	}

	if (ret < 0) {
		debug("hub: disabling port %d\r\n", port + 1);
		usb_clear_port_feature(dev, port + 1, USB_PORT_FEAT_ENABLE);
	}

	API_VmmDmaFree(portsts);
	return ret;
}


static int usb_hub_configure(struct usb_device *dev)
{
	int i, length;
	ALLOC_CACHE_ALIGN_BUFFER(unsigned char, buffer, USB_BUFSIZ);
	unsigned char *bitmap;
	short hubCharacteristics;
	struct usb_hub_descriptor *descriptor;
	struct usb_hub_device *hub;
    struct usb_hub_status *hubsts;
	int ret;

	/* "allocate" Hub device */
	hub = usb_hub_allocate();
	if (hub == NULL) {
	    API_VmmDmaFree(buffer);
		return -ENOMEM;
	}
	hub->pusb_dev = dev;
	/* Get the the hub descriptor */
	ret = usb_get_hub_descriptor(dev, buffer, 4);
	if (ret < 0) {
		debug("usb_hub_configure: failed to get hub " \
		      "descriptor, giving up %lx\r\n", dev->status);
		API_VmmDmaFree(buffer);
		return ret;
	}
	descriptor = (struct usb_hub_descriptor *)buffer;

	length = min_t(int, descriptor->bLength,
		       sizeof(struct usb_hub_descriptor));

	ret = usb_get_hub_descriptor(dev, buffer, length);
	if (ret < 0) {
		debug("usb_hub_configure: failed to get hub " \
		      "descriptor 2nd giving up %lx\r\n", dev->status);
		API_VmmDmaFree(buffer);
		return ret;
	}
	memcpy((unsigned char *)&hub->desc, buffer, length);
	/* adjust 16bit values */
	put_unaligned(le16_to_cpu(get_unaligned(
			&descriptor->wHubCharacteristics)),
			&hub->desc.wHubCharacteristics);
	/* set the bitmap */
	bitmap = (unsigned char *)&hub->desc.DeviceRemovable[0];
	/* devices not removable by default */
	memset(bitmap, 0xff, (USB_MAXCHILDREN+1+7)/8);
	bitmap = (unsigned char *)&hub->desc.PortPowerCtrlMask[0];
	memset(bitmap, 0xff, (USB_MAXCHILDREN+1+7)/8); /* PowerMask = 1B */

	for (i = 0; i < ((hub->desc.bNbrPorts + 1 + 7)/8); i++)
		hub->desc.DeviceRemovable[i] = descriptor->DeviceRemovable[i];

	for (i = 0; i < ((hub->desc.bNbrPorts + 1 + 7)/8); i++)
		hub->desc.PortPowerCtrlMask[i] = descriptor->PortPowerCtrlMask[i];

	dev->maxchild = descriptor->bNbrPorts;
	debug("%d ports detected\r\n", dev->maxchild);

	hubCharacteristics = get_unaligned(&hub->desc.wHubCharacteristics);
	switch (hubCharacteristics & HUB_CHAR_LPSM) {
	case 0x00:
		debug("ganged power switching\r\n");
		break;
	case 0x01:
		debug("individual port power switching\r\n");
		break;
	case 0x02:
	case 0x03:
		debug("unknown reserved power switching mode\r\n");
		break;
	}

	if (hubCharacteristics & HUB_CHAR_COMPOUND)
		debug("part of a compound device\r\n");
	else
		debug("standalone hub\r\n");

	switch (hubCharacteristics & HUB_CHAR_OCPM) {
	case 0x00:
		debug("global over-current protection\r\n");
		break;
	case 0x08:
		debug("individual port over-current protection\r\n");
		break;
	case 0x10:
	case 0x18:
		debug("no over-current protection\r\n");
		break;
	}

	debug("power on to power good time: %dms\r\n",
	      descriptor->bPwrOn2PwrGood * 2);
	debug("hub controller current requirement: %dmA\r\n",
	      descriptor->bHubContrCurrent);

	for (i = 0; i < dev->maxchild; i++)
		debug("port %d is%s removable\r\n", i + 1,
		      hub->desc.DeviceRemovable[(i + 1) / 8] & \
		      (1 << ((i + 1) % 8)) ? " not" : "");

	if (sizeof(struct usb_hub_status) > USB_BUFSIZ) {
		debug("usb_hub_configure: failed to get Status - " \
		      "too long: %d\r\n", descriptor->bLength);
		API_VmmDmaFree(buffer);
		return -EFBIG;
	}

	ret = usb_get_hub_status(dev, buffer);
	if (ret < 0) {
		debug("usb_hub_configure: failed to get Status %lx\r\n",
		      dev->status);
		API_VmmDmaFree(buffer);
		return ret;
	}

#ifdef DEBUG
	hubsts = (struct usb_hub_status *)buffer;
#endif

	debug("get_hub_status returned status %x, change %x\r\n",
	      le16_to_cpu(hubsts->wHubStatus),
	      le16_to_cpu(hubsts->wHubChange));
	debug("local power source is %s\r\n",
	      (le16_to_cpu(hubsts->wHubStatus) & HUB_STATUS_LOCAL_POWER) ? \
	      "lost (inactive)" : "good");
	debug("%sover-current condition exists\r\n",
	      (le16_to_cpu(hubsts->wHubStatus) & HUB_STATUS_OVERCURRENT) ? \
	      "" : "no ");

#if 0
	usb_hub_power_on(hub);
#endif

	/*
	 * Reset any devices that may be in a bad state when applying
	 * the power.  This is a __weak function.  Resetting of the devices
	 * should occur in the board file of the device.
	 */
	for (i = 0; i < dev->maxchild; i++)
		usb_hub_reset_devices(i + 1);

	for (i = 0; i < dev->maxchild; i++) {
		ALLOC_CACHE_ALIGN_BUFFER(struct usb_port_status, portsts, 1);
		unsigned short portstatus, portchange;
		int ret;

#if 0
		ulong start = gettimeofday(0);
#endif

		debug("Scanning port %d\r\n", i + 1);

		/*
		 * Wait for (whichever finishes first)
		 *  - A maximum of 10 seconds
		 *    This is a purely observational value driven by connecting
		 *    a few broken pen drives and taking the max * 1.5 approach
		 *  - connection_change and connection state to report same
		 *    state
		 */
		do {
			ret = usb_get_port_status(dev, i + 1, portsts);
			if (ret < 0) {
				debug("get_port_status failed\r\n");
				break;
			}

			portstatus = le16_to_cpu(portsts->wPortStatus);
			portchange = le16_to_cpu(portsts->wPortChange);

			if ((portchange & USB_PORT_STAT_C_CONNECTION) ==
				(portstatus & USB_PORT_STAT_CONNECTION))
				break;

#if 0
		} while (get_timer(start) < CONFIG_SYS_HZ * 10);
#else
	    } while (1);
#endif

		if (ret < 0)
			continue;

		debug("Port %d Status %x Change %x\r\n",
		      i + 1, portstatus, portchange);
#if 0
		if (portchange & USB_PORT_STAT_C_CONNECTION) {
			debug("port %d connection change\r\n", i + 1);
			usb_hub_port_connect_change(dev, i);
		}
		if (portchange & USB_PORT_STAT_C_ENABLE) {
			debug("port %d enable change, status %x\r\n",
			      i + 1, portstatus);
			usb_clear_port_feature(dev, i + 1,
						USB_PORT_FEAT_C_ENABLE);
			/*
			 * The following hack causes a ghost device problem
			 * to Faraday EHCI
			 */
#ifndef CONFIG_USB_EHCI_FARADAY
			/* EM interference sometimes causes bad shielded USB
			 * devices to be shutdown by the hub, this hack enables
			 * them again. Works at least with mouse driver */
			if (!(portstatus & USB_PORT_STAT_ENABLE) &&
			     (portstatus & USB_PORT_STAT_CONNECTION) &&
			     usb_device_has_child_on_port(dev, i)) {
				debug("already running port %i "  \
				      "disabled by hub (EMI?), " \
				      "re-enabling...\r\n", i + 1);
				      usb_hub_port_connect_change(dev, i);
			}
#endif
		}
		if (portstatus & USB_PORT_STAT_SUSPEND) {
			debug("port %d suspend change\r\n", i + 1);
			usb_clear_port_feature(dev, i + 1,
						USB_PORT_FEAT_SUSPEND);
		}

		if (portchange & USB_PORT_STAT_C_OVERCURRENT) {
			debug("port %d over-current change\r\n", i + 1);
			usb_clear_port_feature(dev, i + 1,
						USB_PORT_FEAT_C_OVER_CURRENT);
			usb_hub_power_on(hub);
		}

		if (portchange & USB_PORT_STAT_C_RESET) {
			debug("port %d reset change\r\n", i + 1);
			usb_clear_port_feature(dev, i + 1,
						USB_PORT_FEAT_C_RESET);
		}
#else
		usb_hub_port_connect_change(dev, i);
#endif
		API_VmmDmaFree(portsts);
	} /* end for i all ports */

    API_VmmDmaFree(buffer);
	return 0;
}

static int usb_hub_check(struct usb_device *dev, int ifnum)
{
	struct usb_interface *iface;
	struct usb_endpoint_descriptor *ep = NULL;

	iface = &dev->config.if_desc[ifnum];
	/* Is it a hub? */
	if (iface->desc.bInterfaceClass != USB_CLASS_HUB)
		goto err;
	/* Some hubs have a subclass of 1, which AFAICT according to the */
	/*  specs is not defined, but it works */
	if ((iface->desc.bInterfaceSubClass != 0) &&
	    (iface->desc.bInterfaceSubClass != 1))
		goto err;
	/* Multiple endpoints? What kind of mutant ninja-hub is this? */
	if (iface->desc.bNumEndpoints != 1)
		goto err;
	ep = &iface->ep_desc[0];
	/* Output endpoint? Curiousier and curiousier.. */
	if (!(ep->bEndpointAddress & USB_DIR_IN))
		goto err;
	/* If it's not an interrupt endpoint, we'd better punt! */
	if ((ep->bmAttributes & 3) != 3)
		goto err;
	/* We found a hub */
	debug("USB hub found\r\n");
	return 0;

err:
	debug("USB hub not found: bInterfaceClass=%d, bInterfaceSubClass=%d, bNumEndpoints=%d\r\n",
	      iface->desc.bInterfaceClass, iface->desc.bInterfaceSubClass,
	      iface->desc.bNumEndpoints);
	if (ep) {
		debug("   bEndpointAddress=%x, bmAttributes=%d\r\n",
		      ep->bEndpointAddress, ep->bmAttributes);
	}

	return -ENOENT;
}

int usb_hub_probe(struct usb_device *dev, int ifnum)
{
	int ret;

	ret = usb_hub_check(dev, ifnum);
	if (ret)
		return 0;
	ret = usb_hub_configure(dev);
	return ret;
}
