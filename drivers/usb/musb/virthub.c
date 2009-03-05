/*****************************************************************
 * Copyright 2005 Mentor Graphics Corporation
 * Copyright (C) 2005-2006 by Texas Instruments
 *
 * This file is part of the Inventra Controller Driver for Linux.
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 *
 * The Inventra Controller Driver for Linux is distributed in
 * the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not,
 * write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307  USA
 *
 * ANY DOWNLOAD, USE, REPRODUCTION, MODIFICATION OR DISTRIBUTION
 * OF THIS DRIVER INDICATES YOUR COMPLETE AND UNCONDITIONAL ACCEPTANCE
 * OF THOSE TERMS.THIS DRIVER IS PROVIDED "AS IS" AND MENTOR GRAPHICS
 * MAKES NO WARRANTIES, EXPRESS OR IMPLIED, RELATED TO THIS DRIVER.
 * MENTOR GRAPHICS SPECIFICALLY DISCLAIMS ALL IMPLIED WARRANTIES
 * OF MERCHANTABILITY; FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT.  MENTOR GRAPHICS DOES NOT PROVIDE SUPPORT
 * SERVICES OR UPDATES FOR THIS DRIVER, EVEN IF YOU ARE A MENTOR
 * GRAPHICS SUPPORT CUSTOMER.
 ******************************************************************/

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/timer.h>

#include <linux/usb.h>

#include "../core/hcd.h"

#include "musbdefs.h"
#include "musb_host.h"

/* FIXME most of this file will vanish when we convert this driver
 * over to the standard 2.6 hcd framework
 */

/****************************** GLOBALS *********************************/

static void musb_port_reset_done(struct virtual_root *pHub, u8 bHubSpeed);
/* device descriptor */
static const u8 rh_dev_desc[] = {
	USB_DT_DEVICE_SIZE,
	USB_DT_DEVICE,
	0x00, 0x02,		/* bcdUSB */
	USB_CLASS_HUB,		/* bDeviceClass */
	0,			/* bDeviceSubClass */
	1,			/* bDeviceProtocol (single TT) */
	64,			/* bMaxPacketSize0 */
	0xd6, 0x4,		/* idVendor */
	0, 0,			/* idProduct */
	0, 0,			/* bcdDevice */
	0,			/* iManufacturer */
	0,			/* iProduct */
	0,			/* iSerialNumber */
	1			/* bNumConfigurations */
};

/* Configuration descriptor */
static const u8 rh_config_desc[] = {
	USB_DT_CONFIG_SIZE,
	USB_DT_CONFIG,
	USB_DT_CONFIG_SIZE + USB_DT_INTERFACE_SIZE + USB_DT_ENDPOINT_SIZE, 0,
	0x01,			/* bNumInterfaces */
	0x01,			/* bConfigurationValue */
	0x00,			/* iConfiguration */
	0xE0,			/* bmAttributes (self-powered, remote wake) */
	0x00,			/* MaxPower */

	/* interface */
	USB_DT_INTERFACE_SIZE,
	USB_DT_INTERFACE,
	0x00,			/* bInterfaceNumber */
	0x00,			/* bAlternateSetting */
	0x01,			/* bNumEndpoints */
	USB_CLASS_HUB,		/* bInterfaceClass */
	0x00,			/* bInterfaceSubClass */
	0x00,			/* bInterfaceProtocol */
	0x00,			/* iInterface */

	/* endpoint */
	USB_DT_ENDPOINT_SIZE,
	USB_DT_ENDPOINT,
	USB_DIR_IN | 1,		/* bEndpointAddress: IN Endpoint 1 */
	USB_ENDPOINT_XFER_INT,	/* bmAttributes: Interrupt */
	(MGC_VIRTUALHUB_MAX_PORTS + 8) / 8, 0,	/* wMaxPacketSize */
	12			/* bInterval: 256 ms */
};

/***************************** FUNCTIONS ********************************/

/*
 * assumes pHub to be locked!
 */
static void MGC_VirtualHubCheckIrq(struct virtual_root *pHub, struct urb *pUrb,
				   int status)
{
	int nLength, nPort;
	u8 bData, bBit;
	u8 *pData;

	/* how many bits are needed/possible */
	nLength = min(pUrb->transfer_buffer_length * 8, 1 +
		      min((u8) MGC_VIRTUALHUB_MAX_PORTS, pHub->bPortCount));
	bData = 0;
	bBit = 1;
	pData = (u8 *) pUrb->transfer_buffer;

	/* hub status bit would indicate overcurrent or power lost */

	/* count 1..N to accomodate hub status bit */
	for (nPort = 1; nPort <= nLength; nPort++) {
		if (pHub->aPortStatusChange[nPort - 1].wChange) {
			bData |= 1 << bBit;
		}
		if (++bBit > 7) {
			*pData++ = bData;
			bData = bBit = 0;
		}
	}

	if (bBit) {
		*pData++ = bData;
	}

	pUrb->actual_length = (int)pData - (int)pUrb->transfer_buffer;
	if (pUrb->actual_length && pUrb->complete) {
		DBG(4, "completing hub interrupt URB\n");

		/* REVISIT root hub completions (all of them!) ... they don't
		 * handle usb_kill_urb() correctly, and have other issues.
		 */
		pUrb->status = status;
		pUrb->hcpriv = NULL;

		spin_unlock(&pHub->Lock);
		pUrb->complete(pUrb, NULL);
		spin_lock(&pHub->Lock);
	}
}

/*
 * Timer expiration function to complete the interrupt URB on changes
 *
 * REVISIT better to not use a timer, be purely irq-driven.
 */
static void rh_timer(unsigned long ptr)
{
	struct virtual_root *pHub = (struct virtual_root *)ptr;
	struct urb *pUrb;
	unsigned long flags;

	spin_lock_irqsave(&pHub->Lock, flags);
	pUrb = pHub->pUrb;

	if (pUrb && (pUrb->hcpriv == pHub)) {
		u8 bPort;

		for (bPort = 0; bPort < pHub->bPortCount; bPort++) {
			if (pHub->aPortStatusChange[bPort].wChange) {
				MGC_VirtualHubCheckIrq(pHub, pUrb, 0);
				break;
			}
		}

		/* re-activate timer only when the urb is still mine;
		 * pUrb->hcpriv is set to NULL on port disconnect
		 */
		mod_timer(&pHub->Timer, jiffies
			  + msecs_to_jiffies(pHub->wInterval));
	} else {
		DBG(3, "pUrb=%p, for me =%d\n", pUrb,
		    (pUrb) ? ((pUrb->hcpriv) ? 1 : 0) : -1);
	}

	spin_unlock_irqrestore(&pHub->Lock, flags);
}

/*
 * Initialize the virtual hub.z
 * @param pHub
 * @param pBus
 * @param pPortServices
 */
u8 MGC_VirtualHubInit(struct virtual_root *pHub, struct usb_bus *pBus,
		      struct musb *musb)
{
	/* allocate device */
	pHub->pDevice = usb_alloc_dev(NULL, pBus, 0);
	if (!pHub->pDevice) {
		ERR("Cannot allocate root hub\n");
		return FALSE;
	}

	pHub->pBus = pBus;
	pHub->pDevice->speed = USB_SPEED_HIGH;

	spin_lock_init(&pHub->Lock);
	pHub->pUrb = NULL;
	pHub->musb = musb;
	pHub->bPortCount = 1;

	// DONE WITH ALLOCATION: init_timer(&pHub->Timer);
	pHub->Timer.function = rh_timer;
	pHub->Timer.data = (unsigned long)pHub;

	pHub->aPortStatusChange[0].wStatus = 0;
	pHub->aPortStatusChange[0].wChange = 0;

	return TRUE;
}

void MGC_VirtualHubStop(struct virtual_root *pHub)
{
	/* stop interrupt timer */
	del_timer_sync(&pHub->Timer);
}

static inline void set_active(struct musb *musb)
{
	unsigned long flags;

	spin_lock_irqsave(&musb->Lock, flags);
	musb_start(musb);
	spin_unlock_irqrestore(&musb->Lock, flags);
}

static void musb_port_suspend(struct musb *pThis, u8 bSuspend)
{
	u8 power, devctl;
	unsigned long flags;
	void __iomem *pBase = pThis->pRegs;

	DBG(2, "<==\n");

	spin_lock_irqsave(&pThis->Lock, flags);
	if (!is_host_active(pThis))
		goto done;

	power = musb_readb(pBase, MGC_O_HDRC_POWER);
	devctl = musb_readb(pBase, MGC_O_HDRC_DEVCTL);
	if (bSuspend) {
		DBG(3, "Root port suspended\n");
		musb_writeb(pBase, MGC_O_HDRC_POWER,
			    power | MGC_M_POWER_SUSPENDM);

#ifdef CONFIG_USB_MUSB_OTG

		otg_input_changed(pThis, devctl, FALSE, FALSE, TRUE);
#endif

	} else if (power & MGC_M_POWER_SUSPENDM) {
		DBG(3, "Root port resumed\n");
		power &= ~(MGC_M_POWER_SUSPENDM | MGC_M_POWER_RESUME);
		musb_writeb(pBase, MGC_O_HDRC_POWER,
			    power | MGC_M_POWER_RESUME);

#ifdef CONFIG_USB_MUSB_OTG
		otg_input_changed(pThis, devctl, FALSE, FALSE, FALSE);
#endif
		mod_timer(&pThis->Timer, jiffies + msecs_to_jiffies(10));
	}
      done:
	spin_unlock_irqrestore(&pThis->Lock, flags);
}

static void musb_port_reset(struct musb *pThis, u8 bReset)
{
	u8 power;
	unsigned long flags;
	void __iomem *pBase = pThis->pRegs;

#ifdef CONFIG_USB_MUSB_OTG
	/* REVISIT this looks wrong for HNP */
	u8 devctl = musb_readb(pBase, MGC_O_HDRC_DEVCTL);
	if (pThis->bDelayPortPowerOff || !(devctl & MGC_M_DEVCTL_HM)) {
//              return;
		DBG(1, "what?\n");
	}
#endif

	spin_lock_irqsave(&pThis->Lock, flags);
	if (!is_host_active(pThis))
		goto done;

	/* NOTE:  caller guarantees it will turn off the reset when
	 * the appropriate amount of time has passed
	 */
	power = musb_readb(pBase, MGC_O_HDRC_POWER);
	if (bReset) {
		pThis->bIgnoreDisconnect = TRUE;
		power &= 0xf1;	/* Have the ENSUSPEND bit set */
		musb_writeb(pBase, MGC_O_HDRC_POWER, power | MGC_M_POWER_RESET);
	} else {
		DBG(3, "root port reset stopped\n");
		musb_writeb(pBase, MGC_O_HDRC_POWER,
			    power & ~MGC_M_POWER_RESET);

		pThis->bIgnoreDisconnect = FALSE;

		/* check for high-speed and set in root device if so */
		power = musb_readb(pBase, MGC_O_HDRC_POWER);
		if (power & MGC_M_POWER_HSMODE) {
			DBG(4, "high-speed device connected\n");
			pThis->bRootSpeed = USB_SPEED_HIGH;
		}

		musb_port_reset_done(&(pThis->RootHub), pThis->bRootSpeed);
	}

      done:
	spin_unlock_irqrestore(&pThis->Lock, flags);
}

int MGC_VirtualHubSubmitUrb(struct virtual_root *pHub, struct urb *pUrb)
{
	u8 bRecip;		/* from standard request */
	u8 bReqType;		/* from standard request */
	u8 bType;		/* requested descriptor type */
	u16 wValue;		/* from standard request */
	u16 wIndex;		/* from standard request */
	u16 wLength;		/* from standard request */
	u8 bPort;
	const struct usb_ctrlrequest *pRequest;
	u16 wSize = 0xffff;
	u8 *pData = (u8 *) pUrb->transfer_buffer;
	unsigned int pipe = pUrb->pipe;
	unsigned long flags;

	spin_lock_irqsave(&pHub->Lock, flags);
	usb_get_urb(pUrb);

	pUrb->hcpriv = pHub;
	pUrb->status = -EINPROGRESS;
	if (usb_pipeint(pipe)) {
		DBG(6, "is periodic status/change event\n");

		/* this is the one for periodic status/change events */
		pHub->pUrb = pUrb;
		pHub->wInterval = HZ / 4;
		spin_unlock_irqrestore(&pHub->Lock, flags);
		return 0;
	}

	/* REVISIT  when we convert to using the usbcore root hub support,
	 * most of this will vanish.  At that time, review the rest of this
	 * code for correctness (against the usb 2.0 hub spec).
	 */

	/* handle hub requests/commands */
	pRequest = (const struct usb_ctrlrequest *)pUrb->setup_packet;
	bReqType = pRequest->bRequestType & USB_TYPE_MASK;
	bRecip = pRequest->bRequestType & USB_RECIP_MASK;
	wValue = le16_to_cpu(pRequest->wValue);
	wIndex = le16_to_cpu(pRequest->wIndex);
	wLength = le16_to_cpu(pRequest->wLength);

	DBG(4, "ROOT SETUP req%02x.%02x v%04x i%04x l%d\n",
	    pRequest->bRequestType, pRequest->bRequest,
	    wValue, wIndex, wLength);

	/* otg hosts mustn't change any root hub status until entering
	 * OTG_STATE_A_IDLE or OTG_STATE_B_HOST.  strictly speaking we should
	 * return errors for such requests, but let's avoid trouble...
	 */
	if (!is_host_active(pHub->musb)
	    && !(pRequest->bRequestType & USB_DIR_IN)) {
		wSize = 0;
		goto fakeit;
	}

	switch (pRequest->bRequest) {
	case USB_REQ_GET_STATUS:
		DBG(5, "GET_STATUS(), bType=%02x, bRecip=%02x, wIndex=%04x\n",
		    bReqType, bRecip, wIndex);

		if (USB_TYPE_STANDARD == bReqType) {
			/* self-powered */
			pData[0] = (USB_RECIP_DEVICE == bRecip) ? 1 : 0;
			pData[1] = 0;
			wSize = 2;
		} else if (USB_TYPE_CLASS == bReqType) {
			if ((USB_RECIP_OTHER == bRecip)
			    && wIndex && (wIndex <= pHub->bPortCount)) {
				MGC_HubPortStatusChange *p;

				p = pHub->aPortStatusChange + (wIndex - 1);

				/* this same mechanism is used by other hcds,
				 * but we should probably also track how much
				 * time has passed (in case something other
				 * than khubd issues this GET_STATUS request)
				 */
				if (p->wStatus & USB_PORT_STAT_RESET) {
					p->wStatus &= ~USB_PORT_STAT_RESET;
					musb_port_reset(pHub->musb, FALSE);
				}

				/* REVISIT assumes 2-byte alignment */

				/* port status/change report */
				*(u16 *) (pData + 0) = cpu_to_le16(p->wStatus);
				*(u16 *) (pData + 2) = cpu_to_le16(p->wChange);

				/* reset change (TODO: lock) */
				pHub->aPortStatusChange[wIndex - 1].wChange = 0;
				wSize = 4;
			} else {
				/* hub status */
				memset(pData, 0, 4);
				wSize = 4;
			}

			DBG(5, "status %04x change %04x\n",
			    pHub->aPortStatusChange[0].wStatus,
			    pHub->aPortStatusChange[0].wChange);
		}
		break;

	case USB_REQ_CLEAR_FEATURE:
		bPort = (u8) (wIndex & 0xff) - 1;
		if ((USB_TYPE_STANDARD == bReqType)
		    && (USB_RECIP_ENDPOINT == bRecip)) {
			wSize = 0;
			DBG(5, "clear END POINT feature!\n");
		} else if (USB_TYPE_CLASS == bReqType) {

			if (USB_RECIP_OTHER == bRecip) {
				bPort = (u8) (wIndex & 0xff) - 1;
				switch (wValue) {
				case USB_PORT_FEAT_CONNECTION:
				case USB_PORT_FEAT_OVER_CURRENT:
				case USB_PORT_FEAT_POWER:
				case USB_PORT_FEAT_INDICATOR:
					DBG(5, "clear feat 0x%02x port %d\n",
					    wValue, bPort);
					wSize = 0;
					break;
				case USB_PORT_FEAT_ENABLE:
					DBG(5, "enable port %d\n", bPort);
					wSize = 0;
					break;
				case USB_PORT_FEAT_SUSPEND:
					DBG(5, "suspend port %d\n", bPort);
					musb_port_suspend(pHub->musb, FALSE);
					wSize = 0;
					break;
				case USB_PORT_FEAT_RESET:
					DBG(5, "reset port %d\n", bPort);
					musb_port_reset(pHub->musb, FALSE);
					wSize = 0;
					break;

					/* acknowledge changes: */
				case USB_PORT_FEAT_C_CONNECTION:
					DBG(5, "ack connect chg port %d\n",
					    bPort);
					pHub->aPortStatusChange[bPort].wChange
					    &= ~USB_PORT_STAT_C_CONNECTION;
					wSize = 0;
					break;
				case USB_PORT_FEAT_C_ENABLE:
					DBG(5, "ack enable chg %d\n", bPort);
					pHub->aPortStatusChange[bPort].wChange
					    &= ~USB_PORT_STAT_C_ENABLE;
					wSize = 0;
					break;
				case USB_PORT_FEAT_C_SUSPEND:
					DBG(5, "ack suspend chg %d\n", bPort);
					pHub->aPortStatusChange[bPort].wChange
					    &= ~USB_PORT_STAT_C_SUSPEND;
					wSize = 0;
					break;
				case USB_PORT_FEAT_C_RESET:
					DBG(5, "ack reset chg %d\n", bPort);
					pHub->aPortStatusChange[bPort].wChange
					    &= ~USB_PORT_STAT_C_RESET;
					wSize = 0;
					break;
				case USB_PORT_FEAT_C_OVER_CURRENT:
					DBG(5, "ack overcurrent chg port %d\n",
					    bPort);
					/*
					   pHub->aPortStatusChange[bPort].wChange
					   &= ~USB_PORT_STAT_C_OVERCURRENT;
					 */
					wSize = 0;
					break;

				default:
					DBG(1, "clear feature 0x%02x on "
					    "port=%d unknown\n", wValue, bPort);
					break;
				}
			} else {
				DBG(5, "clear wValue=%d on port=%d\n", wValue,
				    bPort);
				switch (wValue) {
				case C_HUB_LOCAL_POWER:
				case C_HUB_OVER_CURRENT:
					wSize = 0;
					break;
				}
			}
		} else {
			DBG(1, "CLR_FEAT type=0x%x, wValue=0x%x, wIndex=0x%x\n",
			    bReqType, wValue, (wIndex & 0xff));
		}
		break;

	case USB_REQ_SET_FEATURE:
		if ((USB_TYPE_CLASS == bReqType)
		    && (USB_RECIP_OTHER == bRecip)) {
			bPort = (u8) (wIndex & 0xff) - 1;
			DBG(5, "SET_PORT_FEATURE(0x%02x), port %d\n", wValue,
			    bPort);
			switch (wValue) {
			case USB_PORT_FEAT_SUSPEND:
				DBG(5, "suspend port %d\n", bPort);
				musb_port_suspend(pHub->musb, TRUE);
				pHub->aPortStatusChange[bPort].wStatus |=
				    USB_PORT_STAT_SUSPEND;
				wSize = 0;
				break;

			case USB_PORT_FEAT_RESET:
				DBG(5, "reset port %d\n", bPort);
				pHub->aPortStatusChange[bPort].wStatus |=
				    USB_PORT_STAT_RESET;
				pHub->aPortStatusChange[bPort].wStatus |=
				    USB_PORT_STAT_ENABLE;
				pHub->aPortStatusChange[bPort].wChange |=
				    USB_PORT_STAT_C_RESET;
				musb_port_reset(pHub->musb, TRUE);
				wSize = 0;
				break;

			case USB_PORT_FEAT_POWER:
				DBG(5, "power port %d\n", bPort);
				/* REVISIT: doing it this way seems like an
				 * unclean fit for the hub and bus models,
				 * especially without deactivation...
				 */
				set_active(pHub->musb);
				pHub->aPortStatusChange[bPort].wStatus |=
				    USB_PORT_STAT_POWER;
				wSize = 0;
				break;

			case USB_PORT_FEAT_ENABLE:
				DBG(5, "enable port %d\n", bPort);
				pHub->aPortStatusChange[bPort].wStatus |=
				    USB_PORT_STAT_ENABLE;
				wSize = 0;
				break;
			}
		} else {
			DBG(1, "SET_FEATURE(%04x), but feature unknown\n",
			    wValue);
		}
		break;

	case USB_REQ_SET_ADDRESS:
		DBG(5, "SET_ADDRESS(%x) \n", wValue & 0x7f);
		wSize = 0;
		break;

	case USB_REQ_GET_DESCRIPTOR:
		if (USB_TYPE_CLASS == bReqType) {
			DBG(5, "GET_CLASS_DESCRIPTOR()\n");

			pData[0] = 9;
			pData[1] = 0x29;
			pData[2] = pHub->bPortCount;
			/* min characteristics */
			/* individual port power switching (given
			 * platform_data->set_vbus); no overcurrent
			 */
			pData[3] = 0x11;
			pData[4] = 0;
			/* REVISIT ... report platform_data->potpgt */
			/* PowerOn2PowerGood */
			pData[5] = 50;
			/* no current */
			pData[6] = 0;
			/* removable ports */
			pData[7] = 0;
			/* reserved */
			pData[8] = 0xff;
			wSize = pData[0];
		} else {
			bType = (u8) (wValue >> 8);
			DBG(5, "GET_DESCRIPTOR(%d)\n", bType);
			switch (bType) {
			case USB_DT_DEVICE:	/* 1 */
				wSize = min(wLength, (u16) rh_dev_desc[0]);
				memcpy(pData, rh_dev_desc, wSize);
				break;
			case USB_DT_CONFIG:	/* 2 */
				wSize = min(wLength, (u16) rh_config_desc[2]);
				memcpy(pData, rh_config_desc, wSize);
				break;
			}
		}
		break;

	case USB_REQ_GET_CONFIGURATION:
		DBG(5, "GET_CONFIG() => 1\n");
		pData[0] = 1;
		wSize = 1;
		break;

	case USB_REQ_SET_CONFIGURATION:
		DBG(5, "SET_CONFIG(%04x)\n", wValue);
		wSize = 0;
		break;

	}			/* END: switch on request type */

      fakeit:
	if (0xffff == wSize) {
		pUrb->status = -EPIPE;
	} else {
		pUrb->actual_length = wSize;
		pUrb->status = 0;
	}

	spin_unlock_irqrestore(&pHub->Lock, flags);

	DBG((pUrb->status < 0) ? 3 : 4,
	    "URB status %d, len %d\n", pUrb->status, pUrb->actual_length);
	pUrb->hcpriv = NULL;
	pUrb->complete(pUrb, NULL);
	usb_put_urb(pUrb);

	return 0;
}

/* Implementation */
int MGC_VirtualHubUnlinkUrb(struct virtual_root *pHub, struct urb *pUrb)
{
	//unsigned long flags;

	//spin_lock_irqsave(&pHub->Lock, flags);
	if (pUrb && (pHub->pUrb == pUrb) && (pUrb->hcpriv == pHub)) {
		/* NOTE:  this path should support usb_kill_urb()... */
		pUrb->status = -ECONNRESET;
		pUrb->hcpriv = NULL;
		pHub->pUrb = NULL;
		pUrb->complete(pUrb, NULL);
		usb_put_urb(pUrb);
	}
	//spin_unlock_irqrestore(&pHub->Lock, flags);
	return 0;
}

/*
 * assumes bPortIndex < MGC_VIRTUALHUB_MAX_PORTS
 * AND pHub->Lock to be... locked :)
 */
static inline void musb_port_speed(struct virtual_root *pHub, u8 bSpeed)
{
	u16 wSpeedMask = 0;

	switch (bSpeed) {
	case USB_SPEED_LOW:
		wSpeedMask = USB_PORT_STAT_LOW_SPEED;
		break;
	case USB_SPEED_HIGH:
		wSpeedMask = USB_PORT_STAT_HIGH_SPEED;
		break;
	}

	pHub->aPortStatusChange[0].wStatus &=
	    ~(USB_PORT_STAT_LOW_SPEED | USB_PORT_STAT_HIGH_SPEED);
	pHub->aPortStatusChange[0].wStatus |= wSpeedMask;
}

static void musb_port_reset_done(struct virtual_root *pHub, u8 bHubSpeed)
{
	//unsigned long flags;

	//spin_lock_irqsave(&pHub->Lock, flags);

	DBG(4, "port %d reset complete\n", 0);
	musb_port_speed(pHub, bHubSpeed);

	pHub->aPortStatusChange[0].wStatus &= ~USB_PORT_STAT_RESET;
	pHub->aPortStatusChange[0].wStatus |= USB_PORT_STAT_ENABLE;
	pHub->aPortStatusChange[0].wChange =
	    USB_PORT_STAT_C_RESET | USB_PORT_STAT_C_ENABLE;

	//spin_unlock_irqrestore(&pHub->Lock, flags);
}

/*
 * Connect a port on the virtual hub.
 *
 * @param pHub the virtual hub
 * @param bPortIndex the port that has been disconnected
 * @param bSpeed the port speed
 */
void MGC_VirtualHubPortConnected(struct virtual_root *pHub, u8 bPortIndex,
				 u8 bSpeed)
{
	struct urb *pUrb;

	DBG(2, "<== port %d connected, core reports speed=%d\n", bPortIndex,
	    bSpeed);

	if (bPortIndex < MGC_VIRTUALHUB_MAX_PORTS) {
		unsigned long flags;

		spin_lock_irqsave(&pHub->Lock, flags);

		pUrb = pHub->pUrb;
		musb_port_speed(pHub, bSpeed);
		pHub->aPortStatusChange[bPortIndex].wStatus |=
		    USB_PORT_STAT_CONNECTION;
		pHub->aPortStatusChange[bPortIndex].wChange |=
		    USB_PORT_STAT_C_CONNECTION;

		if (pUrb && ((!pUrb->hcpriv) || (pUrb->hcpriv == pHub))) {
			pUrb->hcpriv = pHub;
			mod_timer(&pHub->Timer, jiffies);
		}

		spin_unlock_irqrestore(&pHub->Lock, flags);
	}
}

/* caller irqlocked musb */
void musb_root_disconnect(struct musb *musb)
{
	unsigned long flags;

	spin_lock_irqsave(&musb->RootHub.Lock, flags);
	musb->RootHub.aPortStatusChange[0].wStatus &=
	    ~(USB_PORT_STAT_CONNECTION
	      | USB_PORT_STAT_ENABLE
	      | USB_PORT_STAT_LOW_SPEED
	      | USB_PORT_STAT_HIGH_SPEED | USB_PORT_STAT_TEST);
	musb->RootHub.aPortStatusChange[0].wChange |=
	    USB_PORT_STAT_C_CONNECTION;
	mod_timer(&musb->RootHub.Timer, jiffies);
	spin_unlock_irqrestore(&musb->RootHub.Lock, flags);

	/* HBG 21SEPT2006 Removed as part of OTG enhancement */
	//=====================================================
#if 0
	switch (musb->xceiv.state) {
	case OTG_STATE_A_HOST:
		musb->xceiv.state = OTG_STATE_A_WAIT_BCON;
		break;
	case OTG_STATE_A_WAIT_VFALL:
		musb->xceiv.state = OTG_STATE_B_IDLE;
		break;
	default:
		DBG(1, "host disconnect, state %d\n", musb->xceiv.state);
	}
#endif
	//====================================================
	if (musb->pRootDevice)
		usb_set_device_state(musb->pRootDevice, USB_STATE_NOTATTACHED);
	else
		DBG(1, "disconnect with NO DEVICE CONNECTED?\n");
	usb_put_dev(musb->pRootDevice);
	musb->pRootDevice = NULL;
}

/* caller irqlocked musb */
void MGC_VirtualHubPortResumed(struct virtual_root *pHub, u8 bPortIndex)
{
	unsigned long flags;

	DBG(3, "Resume port %d\n", bPortIndex);
	if (bPortIndex >= MGC_VIRTUALHUB_MAX_PORTS) {
		return;
	}
	spin_lock_irqsave(&pHub->Lock, flags);
	pHub->aPortStatusChange[bPortIndex].wStatus &= ~USB_PORT_STAT_SUSPEND;
	pHub->aPortStatusChange[bPortIndex].wChange |= USB_PORT_STAT_C_SUSPEND;
	spin_unlock_irqrestore(&pHub->Lock, flags);
	return;
}
