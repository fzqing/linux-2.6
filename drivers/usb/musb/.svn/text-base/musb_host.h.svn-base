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

#ifndef _MUSB_HOST_H
#define _MUSB_HOST_H

#ifdef CONFIG_MUSB_RESERVE_ISO_EP
#define ISO_EP	2 	/* Reserve EP 2 Rx, Tx for ISO */
#endif
/* host side representation of one hardware transfer channel, bound
 * during transfers to the peripheral endpoint addressed by urb->pipe.
 */
struct musb_host_ep {
	unsigned tx:1;

	unsigned claimed:1;	/* unavailable for binding? */
	unsigned busy:1;	/* unsafe to touch hw? */
	unsigned ready:1;

	/* address and type of current peripheral endpoint */
	u8 remote_addr;
	u8 remote_end;

	u8 type;
	u16 max_packet;

	/* transfer state */
	u16 wait_frame;

	unsigned int offset;
	unsigned int request_size;
	unsigned int iso_packet;	/* index into urb */

	/* FIXME just hook into a separate schedule data structure.
	 * When scheduling is this closely coupled to hardware, we
	 * waste resources in ways that are surprising to users.
	 */
	struct urb *urb;
	struct list_head urb_list;
	// struct musb_sched_node       *next;
};

/* in newer 2.6 kernels, something like this should be the schedule data
 * structure stored in "usb_host_endpoint.hcpriv"; store it there when
 * queueing the first URB, remove when usb_host_endpoint.urb_list empties.
 */
struct musb_sched_node {
	struct usb_host_endpoint *host_ep;	/* usbcore info */
	struct usb_device *dev;
	struct musb_host_ep *ep;	/* current binding */
	struct list_head ring;	/* of host_ep */
	struct musb_sched_node *next;	/* for periodic tree */
};


extern void MGC_HdrcStartTx(struct musb *, u8 bEnd);

extern void MGC_HdrcStopEnd(struct musb *, u8 bEnd);

static inline struct urb *MGC_GetCurrentUrb(struct musb_hw_ep *pEnd)
{
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	return list_empty(&(pEnd->urb_list)) ? NULL
	    : list_entry(pEnd->urb_list.next, struct urb, urb_list);
#else
	return NULL;
#endif
}

extern void musb_root_disconnect(struct musb *musb);

extern struct usb_operations musb_host_bus_ops;

extern void musb_h_fifo_check_complete (struct musb_hw_ep *ep);

static inline struct urb *next_in_urb(struct musb_hw_ep *hw_ep)
{
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	struct list_head *queue = &hw_ep->in_urb_list;

	if (list_empty(queue))
		return NULL;
	return container_of(queue->next, struct urb, urb_list);
#else
	return NULL;
#endif
}

static inline struct urb *next_out_urb(struct musb_hw_ep *hw_ep)
{
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	struct list_head *queue = &hw_ep->out_urb_list;

	if (list_empty(queue))
		return NULL;
	return container_of(queue->next, struct urb, urb_list);
#else
	return NULL;
#endif
}

#endif				/* _MUSB_HOST_H */
