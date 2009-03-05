/******************************************************************
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

#ifndef __MUSB_LINUX_VIRTUALHUB_H__
#define __MUSB_LINUX_VIRTUALHUB_H__

#include <linux/spinlock.h>
#include <linux/timer.h>

struct urb;
struct usb_bus;

/** Maximum number of ports to accomodate */
#define MGC_VIRTUALHUB_MAX_PORTS	1

/**
 * MGC_HubPortStatusChange.
 * @field wStatus status
 * @field wChange change
 */
typedef struct {
	u16 wStatus;
	u16 wChange;
} MGC_HubPortStatusChange;

struct virtual_root {
	spinlock_t Lock;
	struct usb_bus *pBus;
	struct usb_device *pDevice;
	void *pUrb;
	struct musb *musb;
	struct timer_list Timer;
	MGC_HubPortStatusChange aPortStatusChange[MGC_VIRTUALHUB_MAX_PORTS];
	u8 bPortCount;
	u16 wInterval;
};

/****************************** FUNCTIONS ********************************/

/**
 * Initialize a virtual hub.
 * @param pHub hub struct pointer; struct filled on success
 * @param pDevice pointer to bus
 * @return TRUE on success
 * @return FALSE on failure
 */
extern u8 MGC_VirtualHubInit(struct virtual_root *pHub,
			     struct usb_bus *pBus, struct musb *musb);

/**
 * Stop a virtual hub
 */
extern void MGC_VirtualHubStop(struct virtual_root *pHub);

/**
 * Submit an URB to a virtual hub.
 * @param pHub pointer to hub initialized by successful MGC_VirtualHubInit
 * @param pUrb URB pointer
 * @return Linux status code
 * @see #MGC_VirtualHubInit
 */
extern int MGC_VirtualHubSubmitUrb(struct virtual_root *pHub, struct urb *pUrb);

/**
 * Unlink an URB from a virtual hub.
 * @param pHub pointer to hub initialized by successful MGC_VirtualHubInit
 * @param pUrb URB pointer
 * @return Linux status code
 * @see #MGC_VirtualHubInit
 */
extern int MGC_VirtualHubUnlinkUrb(struct virtual_root *pHub, struct urb *pUrb);

/**
 * A device has effectively been connected to a virtual hub port
 * @param pHub pointer to hub initialized by successful MGC_VirtualHubInit
 * @param bPortIndex 0-based index of port with connected device
 * @param bSpeed device speed (0=>low, 1=>full, 2=>high)
 * @see #MGC_VirtualHubInit
 */
extern void MGC_VirtualHubPortConnected(struct virtual_root *pHub,
					u8 bPortIndex, u8 bSpeed);

/**
 * A device has effectively resumed a virtual hub port
 * @param pHub pointer to hub initialized by successful MGC_VirtualHubInit
 * @param bPortIndex 0-based index of port of resume
 * @see #MGC_VirtualHubInit
 */
extern void MGC_VirtualHubPortResumed(struct virtual_root *pHub, u8 bPortIndex);

#endif				/* multiple inclusion protection */
