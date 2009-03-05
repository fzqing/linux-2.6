/*
 * Renesas SuperH USB 1.1 device controller (found on SH7705, SH7727...)
 *
 * Copyright (C) 2003 Renesas Technology Europe Limited
 * Copyright (C) 2003 Julian Back (jback@mpc-data.co.uk), MPC Data Limited
 * Copyright (C) 2005 Takashi Kusuda(Hitachi-ULSI Systems Co., LTD)
 *               Add to support kernel 2.6.x.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
 * This is a driver for the USB Device Controller found on Renesas SH
 * processors.  This is a full-speed controller which has four
 * endpoints in a single fixed configuration.
 *
 * Limitations
 *
 * Only tested on SH7705.  Mostly tested with Mass Storage gadget
 * using Bulk-Only Transport.  It has been tested with Linux 2.4,
 * Linux 2.6, Windows 2000 and Windows XP hosts.
 *
 * DMA is not (yet) implemented.
 *
 * Handling of application stalls is tricky.  We set a bit to stall an
 * endpoint.  When the host tries to access the ep it gets a stall and
 * another stall bit is latched by the device.  The host clears the
 * stall with a clear feature but the hardware doesn't inform us, the
 * latched bit is cleared but not the bit we have set, so the next
 * time the host accesses the ep it will get another stall and the
 * latch will be set again unless we have cleared our stall bit.  The
 * solution adopted in this driver is to use a timer to clear the
 * application stall bit some time after setting the stall.  This
 * seems to work most of the time but is not 100% reliable.  Because
 * of this it is best to avoid USB protocols that require the USB
 * device to stall the host.  Unfortunately USB mass storage does
 * require the device to stall when it gets unsupported commands,
 * Linux hosts don't send any of these unsupported commands but
 * Windows hosts do.
 *
 * Another place where the hardware is too clever is in the handling
 * of setup packets.  Many setup packets including SET_INTERFACE and
 * SET_CONFIGURATION are handled by the hardware without informing the
 * driver software.  But we need to inform the gadget driver of at
 * least one of these as it uses this to kick of it's data processing.
 * The solution adopted is that after we have recieved N setup packets
 * following a bus reset a fake SET_CONFIGURATION is sent to the
 * gadget.  We also have to arrange things so that the reply to the
 * fake packet is not sent out.
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>

#include <asm/atomic.h>
#include <asm/byteorder.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>

#include <linux/usb_ch9.h>
#include <linux/usb_gadget.h>

#undef DEBUG
#undef VERY_NOISY

#define	DRIVER_DESC		"SuperH USB Peripheral Controller"
#define	DRIVER_VERSION		"alpha (11 November 2003)"

#ifdef USE_DMA
#error "DMA not supported"
#endif

static const char driver_name[] = "superh_udc";
static const char driver_desc[] = DRIVER_DESC;

static const char ep0name[] = "ep0";
static const char *ep_name[] = {
	ep0name,
	"ep1out-bulk",
	"ep2in-bulk",
	"ep3in-bulk",
};

static struct superh_udc *the_controller;

#include "superh_udc.h"

/* High priority interrupts */
#define F0_HIGH (EP1_FULL | EP2_TR | EP2_EMPTY )
#define F1_HIGH (0)

/* Low priority interrupts */
#define F0_LOW  (BRST | SETUP_TS | EP0o_TS | EP0i_TR | EP0i_TS)
#define F1_LOW  (EP3_TR | EP3_TS | VBUSF)

/* How long to leave the stall bit set - this value is quite critical
 * to making stalls work.  Unfortunately it doesn't seem possible to
 * get a value that will work reliably with both fast and slow
 * machines.
 */
#define STALL_TIME     (HZ/75)

/* Number of endpoints to check in the unstall timer.  It should not
 * be necessary to unstall bulk endpoints using the timer as long as
 * the gadget code is aware that this device cannot stall properly
 * (see the file backed storage gadget for an example).  But if the
 * UDC driver stalls ep0 due to a bad SETUP then the timer is still
 * required otherwise the stall will never get cleared.  If it is
 * necessary to unstall all endpoints using the timer then set this to
 * 4.
 */
#define EP_TO_UNSTALL  1

/* Number of packets to wait for before sending a fake
 * SET_CONFIGURATION to the gadget driver
 */
#define DEFAULT_SETUP_COUNT     7
#define RESET_SETUP_COUNT       2

/* How long to wait for the number of packets specified above */
#define SETUP_TIME              (HZ/10 )

static void superh_ep_fifo_flush(struct usb_ep *_ep);
static void stop_activity(struct superh_udc *dev,
			  struct usb_gadget_driver *driver);
static int superh_ep_set_halt(struct usb_ep *_ep, int value);
static void udc_timer(unsigned long _dev);
static struct superh_request *process_ep_req(struct superh_ep *ep,
					     struct superh_request *req);
static void done(struct superh_ep *ep, struct superh_request *req, int status);

/*
 * IO
 */

static inline void and_b(u8 mask, unsigned long addr)
{
	ctrl_outb(ctrl_inb(addr) & mask, addr);
}

static inline void or_b(u8 mask, unsigned long addr)
{
	ctrl_outb(ctrl_inb(addr) | mask, addr);
}

static inline void ep0_idle(struct superh_udc *dev)
{
	DBG(DBG_VERY_NOISY, "ep0_idle\n");
	dev->ep0state = EP0_IDLE;
}

static void init_udc_timer(struct superh_udc *dev)
{
	init_timer(&dev->timer);
	dev->timer.function = udc_timer;
	dev->timer.data = (unsigned long)dev;
	dev->timer.expires = jiffies + STALL_TIME;
	add_timer(&dev->timer);
}

/* Send a fake SET_CONFIGURATION to the gadget to start it up.
 * Needed because the hardware doesn't let us know when the real packet
 * has arrived.
 */
static void send_fake_config(struct superh_udc *dev)
{
	struct usb_ctrlrequest r;
	dev->fake_config = 1;
	dev->setup_countdown = 0;
	r.bRequestType = USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE;
	r.bRequest = USB_REQ_SET_CONFIGURATION;
	r.wValue = 1;		/* configuration to select */
	r.wIndex = 0;
	r.wLength = 0;
	if (dev->driver->setup(&dev->gadget, &r) < 0) {
		DMSG("SET_CONFIGURATION failed.\n");
	}
}

/*
 * Timer function.  Clears stall from any stalled endpoints as we
 * don't get informed when the host has sent a clear feature.
 */
static void udc_timer(unsigned long _dev)
{
	struct superh_udc *dev = (void *)_dev;
	int i;
	unsigned long flags;

	local_irq_save(flags);

	if (atomic_read(&dev->in_interrupt) == 0) {

		/* Check if a bus reset has been done and we haven't faked a SET_CONFIGURATION */
		if (dev->gadget.speed != USB_SPEED_UNKNOWN
		    && dev->setup_countdown > 0
		    && jiffies - dev->reset_time > SETUP_TIME
		    && list_empty(&dev->ep[0].queue)) {
			send_fake_config(dev);
		}

		/* Check if any end points are halted and restart them */
		for (i = 0; i < EP_TO_UNSTALL; i++) {
			struct superh_ep *ep = &dev->ep[i];
			if (ep->halted) {
				DBG(DBG_VERBOSE, "unstalling ep %d\n", i);
				superh_ep_set_halt(&ep->ep, 0);
				if (likely(!list_empty(&ep->queue))) {
					struct superh_request *req
					    = list_entry(ep->queue.next,
							 struct superh_request,
							 queue);
					process_ep_req(ep, req);
				}
			}
		}
	}

	init_udc_timer(dev);

	local_irq_restore(flags);
}

/*
 *	done - retire a request; caller blocked irqs
 */
static void done(struct superh_ep *ep, struct superh_request *req, int status)
{
	unsigned stopped = ep->stopped;

	DBG(DBG_NOISY, "done: %s %p %d\n", ep->ep.name, req, status);

	list_del_init(&req->queue);

	if (likely(req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

	if (status && status != -ESHUTDOWN)
		DBG(DBG_VERBOSE, "complete %s req %p stat %d len %u/%u\n",
		    ep->ep.name, &req->req, status,
		    req->req.actual, req->req.length);

	/* don't modify queue heads during completion callback */
	ep->stopped = 1;
	req->req.complete(&ep->ep, &req->req);
	ep->stopped = stopped;
}

/*
 *	Enable interrupts for the specified endpoint
 */
static inline void pio_irq_enable(struct superh_ep *ep)
{
	or_b(ep->interrupt_mask, ep->interrupt_reg);
}

/*
 *	Disable interrupts for the specified endpoint
 */
static inline void pio_irq_disable(struct superh_ep *ep)
{
	and_b(~ep->interrupt_mask, ep->interrupt_reg);
}

/*
 * 	nuke - dequeue ALL requests
 */
static void nuke(struct superh_ep *ep, int status)
{
	struct superh_request *req;

	DBG(DBG_NOISY, "nuke %s %d\n", ep->ep.name, status);

	/* called with irqs blocked */
#ifdef	USE_DMA
	if (ep->dma >= 0 && !ep->stopped)
		cancel_dma(ep);
#endif
	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct superh_request, queue);
		done(ep, req, status);
	}

	if (ep->desc)
		pio_irq_disable(ep);
}

static inline void clear_ep_state(struct superh_udc *dev)
{
	unsigned i;

	/* hardware SET_{CONFIGURATION,INTERFACE} automagic resets endpoint
	 * fifos, and pending transactions mustn't be continued in any case.
	 */
	for (i = 1; i < 4; i++)
		nuke(&dev->ep[i], -ECONNABORTED);
}

/*
 * write a packet to an endpoint data register
 */
static int write_packet(u32 epdr, struct superh_request *req, unsigned max)
{
	u8 *buf;
	unsigned length, count;

	buf = req->req.buf + req->req.actual;
	prefetch(buf);

	/* how big will this packet be? */
	length = min(req->req.length - req->req.actual, max);
	req->req.actual += length;

	count = length;
	while (likely(count--))
		ctrl_outb(*buf++, epdr);

	return length;
}

static int write_ep0_fifo(struct superh_ep *ep, struct superh_request *req)
{
	unsigned count;
	int is_short;

	count = write_packet(USBEPDR0I, req, EP0_FIFO_SIZE);
	ep->dev->stats.write.bytes += count;

	/* last packet "must be" short (or a zlp) */
	is_short = (count != EP0_FIFO_SIZE);

	DBG(DBG_VERY_NOISY, "ep0in %d bytes %d left %p\n", count,
	    req->req.length - req->req.actual, req);

	ctrl_outb(EP0i_PKTE, USBTRG);

	if (unlikely(is_short)) {
		ep->dev->ep0state = EP0_END_XFER;

		count = req->req.length;
		done(ep, req, 0);
		/*
		 * If we have received a specified number of setups
		 * after a bus reset or connect then fake a
		 * SET_CONFIGURATION to the driver (as we don't get
		 * them from the hardware).
		 */
		if (ep->dev->setup_countdown >= 0)
			ep->dev->setup_countdown--;
		if (ep->dev->setup_countdown == 0) {
			send_fake_config(ep->dev);
		}
	}

	return is_short;
}

/*
 * handle_ep0_setup
 *
 * Handles a SETUP request on EP0
 */
static void handle_ep0_setup(struct superh_udc *dev)
{
	int i;
	union {
		u8 raw[8];
		struct usb_ctrlrequest r;
	} u;

	for (i = 0; i < 8; i++) {
		u.raw[i] = ctrl_inb(USBEPDR0S);
	}

	/* Send ACK */
	ctrl_outb(EP0s_RDFN, USBTRG);

	le16_to_cpus(&u.r.wValue);
	le16_to_cpus(&u.r.wIndex);
	le16_to_cpus(&u.r.wLength);

	DBG(DBG_VERBOSE, "SETUP %02x.%02x v%04x i%04x l%04x\n",
	    u.r.bRequestType, u.r.bRequest,
	    u.r.wValue, u.r.wIndex, u.r.wLength);

	if (u.r.bRequestType & USB_DIR_IN) {
		DBG(DBG_VERY_NOISY, "handle_ep0_setup: EP0_IN_DATA_PHASE\n");
		dev->ep0state = EP0_IN_DATA_PHASE;
	} else {
		DBG(DBG_VERY_NOISY, "handle_ep0_setup: EP0_OUT_DATA_PHASE\n");
		dev->ep0state = EP0_OUT_DATA_PHASE;
	}

	i = dev->driver->setup(&dev->gadget, &u.r);
	if (i < 0) {
		DMSG("SETUP %02x.%02x v%04x i%04x l%04x failed\n",
		     u.r.bRequestType, u.r.bRequest,
		     u.r.wValue, u.r.wIndex, u.r.wLength);
		superh_ep_set_halt(&dev->ep[0].ep, 1);
	}
}

/*
 * write to an IN endpoint fifo, as many packets as possible.
 * irqs will use this to write the rest later.
 * caller guarantees at least one packet buffer is ready.
 */
static int write_fifo(struct superh_ep *ep, struct superh_request *req)
{
	unsigned max;

	DBG(DBG_VERY_NOISY, "write_fifo\n");

	if ((ep->bEndpointAddress & USB_DIR_IN) != USB_DIR_IN) {
		DMSG("write_fifo from invalid EP (%s)\n", ep->ep.name);
		return -EINVAL;
	}

	max = ep->desc->wMaxPacketSize;
	do {
		unsigned count;
		int is_last, is_short;

		count = write_packet(ep->fifo_reg, req, max);

		/* last packet is usually short (or a zlp) */
		if (unlikely(count != max))
			is_last = is_short = 1;
		else {
			if (likely(req->req.length != req->req.actual)
			    || req->req.zero)
				is_last = 0;
			else
				is_last = 1;
			/* interrupt/iso maxpacket may not fill the fifo */
			is_short = unlikely(max < ep->ep.maxpacket);

			/* FIXME ep.maxpacket should be the current size,
			 * modified (for periodic endpoints) when the
			 * ep is enabled.  do that, re-init as needed,
			 * and change maxpacket refs accordingly.
			 */
		}

		DBG(DBG_VERY_NOISY, "wrote %s %d bytes%s%s %d left %p\n",
		    ep->ep.name, count,
		    is_last ? "/L" : "", is_short ? "/S" : "",
		    req->req.length - req->req.actual, req);

		/* let loose that packet. maybe try writing another one,
		 * double buffering might work.
		 */
		or_b(ep->packet_enable_mask, USBTRG);

		/* requests complete when all IN data is in the FIFO */
		if (is_last) {
			done(ep, req, 0);
			if (list_empty(&ep->queue) || unlikely(ep->dma >= 0)) {
				pio_irq_disable(ep);
			}
#ifdef USE_DMA
			/* TODO */
			if (unlikely(ep->dma >= 0) && !list_empty(&ep->queue)) {
				DMSG("%s pio2dma\n", ep->ep.name);
				req = list_entry(ep->queue.next,
						 struct superh_request, queue);
				kick_dma(ep, req);
				return 0;
			}
#endif
			return 1;
		}
		/* Only loop if on EP2 as it is double buffered */
	} while (ep->bEndpointAddress == (2 | USB_DIR_IN)
		 && ctrl_inb(USBIFR0) & EP2_EMPTY);
	return 0;
}

/*
 * read_ep0_fifo - unload packets from ep0 control-out fifo.  caller
 * should have made sure there's at least one packet ready.
 *
 * returns true if the request completed because of short packet or the
 * request buffer having filled (and maybe overran till end-of-packet).
 */
static int read_ep0_fifo(struct superh_ep *ep, struct superh_request *req)
{
	u8 *buf;
	unsigned bufferspace, count;

	DBG(DBG_VERY_NOISY, "read_ep0_fifo\n");

	if (!ep) {
		DMSG("read_ep0_fifo invalid ep\n");
		return -EINVAL;
	}

	if (!req) {
		DMSG("read_ep0_fifo invalid req\n");
		return -EINVAL;
	}

	if (ep->desc != 0) {
		DMSG("read_ep0_fifo from invalid EP (%s)\n", ep->ep.name);
		return -EINVAL;
	}

	/* make sure there's a packet in the FIFO.
	 */
	if (likely((ctrl_inb(USBIFR0) & EP0o_TS) == 0)) {
		buf = req->req.buf + req->req.actual;
		bufferspace = req->req.length - req->req.actual;

		/* read all bytes from this packet */
		count = ctrl_inb(USBEPSZ0O);
		req->req.actual += min(count, bufferspace);
		DBG(DBG_VERY_NOISY, "read %s %d bytes req %p %d/%d\n",
		    ep->ep.name, count, req, req->req.actual, req->req.length);
		while (likely(count-- != 0)) {
			u8 byte = ctrl_inb(USBEPDR0O);

			if (unlikely(bufferspace == 0)) {
				/* this happens when the driver's buffer
				 * is smaller than what the host sent.
				 * discard the extra data.
				 */
				if (req->req.status != -EOVERFLOW)
					DMSG("%s overflow %d\n",
					     ep->ep.name, count);
				req->req.status = -EOVERFLOW;
			} else {
				*buf++ = byte;
				bufferspace--;
			}
		}

		/* Send ACK */
		or_b(EP0o_RDFN, USBTRG);

		/* completion */
		if (req->req.actual >= req->req.length) {
			done(ep, req, 0);
			ep0_idle(ep->dev);
			return 1;
		}
	}

	return 0;
}

/*
 * read_fifo -  unload packet(s) from the fifo we use for usb OUT
 * transfers and put them into the request.  caller should have made
 * sure there's at least one packet ready.
 *
 * returns true if the request completed because of short packet or the
 * request buffer having filled (and maybe overran till end-of-packet).
 */
static int read_fifo(struct superh_ep *ep, struct superh_request *req)
{
	DBG(DBG_VERY_NOISY, "read_fifo\n");

	if ((ep->bEndpointAddress & 0x0f) != 1) {
		DMSG("read_fifo from invalid EP (%s)\n", ep->ep.name);
		return -EINVAL;
	}

	for (;;) {
		u8 *buf;
		unsigned bufferspace, count, is_short;

		/* make sure there's a packet in the FIFO.
		 */
		if (unlikely((ctrl_inb(USBIFR0) & EP1_FULL) == 0))
			break;
		buf = req->req.buf + req->req.actual;
		bufferspace = req->req.length - req->req.actual;

		/* read all bytes from this packet */
		count = ctrl_inb(USBEPSZ1);
		req->req.actual += min(count, bufferspace);
		is_short = (count < ep->desc->wMaxPacketSize);
		DBG(DBG_VERY_NOISY, "read %s %d bytes%s req %p %d/%d\n",
		    ep->ep.name, count,
		    is_short ? "/S" : "",
		    req, req->req.actual, req->req.length);
		while (likely(count-- != 0)) {
			u8 byte = ctrl_inb(USBEPDR1);

			if (unlikely(bufferspace == 0)) {
				/* this happens when the driver's buffer
				 * is smaller than what the host sent.
				 * discard the extra data.
				 */
				if (req->req.status != -EOVERFLOW)
					DMSG("%s overflow %d\n",
					     ep->ep.name, count);
				req->req.status = -EOVERFLOW;
			} else {
				*buf++ = byte;
				bufferspace--;
			}
		}

		or_b(EP1_RDFN, USBTRG);
		/* There could now be another packet because of dual buffer */

		/* completion */
		if (is_short || req->req.actual == req->req.length) {
			done(ep, req, 0);
			if (list_empty(&ep->queue))
				pio_irq_disable(ep);
			return 1;
		}

		/* finished that packet.  the next one may be waiting... */
	}
	return 0;
}

/*--------------------------------------------------------------------------*/
/* Interrupt Handler(s)
 */

/*
 * superh_udc_irq_f0 - high priority interrupt handler
 * this deals with data to & from the bulk pipes
 */
static irqreturn_t superh_udc_irq_f0(int irq, void *_dev, struct pt_regs *regs)
{
	unsigned char f0_status;
	struct superh_udc *dev = (struct superh_udc *)_dev;
	struct superh_request *req;
	struct superh_ep *ep;

	DBG(DBG_NOISY, "superh_udc_irq_f0 %p\n", dev);

	atomic_inc(&dev->in_interrupt);

	dev->stats.irqs++;
	dev->stats.irq0s++;
	f0_status = ctrl_inb(USBIFR0);

	/* Acknowledge interrupts */
	ctrl_outb(~(f0_status & F0_HIGH), USBIFR0);

	if (f0_status & EP1_FULL) {
		DBG(DBG_NOISY, "superh_udc_irq_f0 %p: EP1 FULL\n", dev);
		ep = &dev->ep[1];

		if (likely(!list_empty(&ep->queue)))
			req = list_entry(ep->queue.next,
					 struct superh_request, queue);
		else
			req = 0;

		if (req)
			read_fifo(ep, req);
		else
			pio_irq_disable(ep);
	}

	if (f0_status & (EP2_TR | EP2_EMPTY)) {
		DBG(DBG_NOISY, "superh_udc_irq_f0 %p: EP2 TR | EP2_EMPTY\n",
		    dev);
		ep = &dev->ep[2];

		if (likely(!list_empty(&ep->queue)))
			req = list_entry(ep->queue.next,
					 struct superh_request, queue);
		else
			req = 0;

		if (req) {
			if ((f0_status & EP2_TR) && (f0_status & EP2_EMPTY))
				write_fifo(ep, req);
			else
				and_b(~EP2_EMPTY, USBIER0);

		} else {
			pio_irq_disable(ep);
		}
	}

	atomic_dec(&dev->in_interrupt);

	return IRQ_HANDLED;
}

/**
 * superh_udc_irq_f1 - low priority interrupt handler
 *
 */
static irqreturn_t superh_udc_irq_f1(int irq, void *_dev, struct pt_regs *regs)
{
	unsigned char f0_status;
	unsigned char f1_status;
	struct superh_udc *dev = (struct superh_udc *)_dev;

	atomic_inc(&dev->in_interrupt);;

	dev->stats.irqs++;
	dev->stats.irq1s++;

	f0_status = ctrl_inb(USBIFR0);
	f1_status = ctrl_inb(USBIFR1);

	/* Acknowledge interrupts */
	ctrl_outb(~(f0_status & F0_LOW), USBIFR0);
	ctrl_outb(~(f1_status & F1_LOW), USBIFR1);

	/* VBUSF indicates the USB being connected/disconnected */
	if (f1_status & VBUSF) {
		DBG(DBG_VERY_NOISY, "superh_udc_irq_f1[%lx] VBUSF\n",
		    dev->stats.irqs);
		if (!is_usb_connected) {
			/* report disconnect just once */
			if (dev->gadget.speed != USB_SPEED_UNKNOWN) {
				DMSG("disconnect %s\n",
				     dev->driver ? dev->driver->driver.
				     name : 0);
				stop_activity(dev, dev->driver);
			}
		} else if (dev->gadget.speed == USB_SPEED_UNKNOWN) {
			DMSG("connect\n");
			dev->setup_countdown = DEFAULT_SETUP_COUNT;
		}
	}

	/* Bus Reset */
	if (f0_status & BRST) {
		int i;
		DBG(DBG_VERBOSE, "superh_udc_irq_f1[%lx]: BRST bus reset\n",
		    dev->stats.irqs);
		/* kill any outstanding requests  */
		for (i = 0; i < 4; i++) {
			struct superh_ep *ep = &dev->ep[i];
			nuke(ep, -ESHUTDOWN);
			ep->halted = 0;
			ep->stopped = 0;
		}

		/* reset fifo's and stall's */
		ctrl_outb(EP3_CLEAR | EP1_CLEAR | EP2_CLEAR | EP0o_CLEAR |
			  EP0i_CLEAR, USBFCLR);
		ctrl_outb(0, USBEPSTL);
		DMSG("gadget driver '%s', address zero\n",
		     dev->driver->driver.name);
		if (dev->gadget.speed == USB_SPEED_UNKNOWN)
			init_udc_timer(dev);
		dev->gadget.speed = USB_SPEED_FULL;
		memset(&dev->stats, 0, sizeof dev->stats);
		if (dev->setup_countdown < 0)
			dev->setup_countdown = RESET_SETUP_COUNT;
		dev->reset_time = jiffies;
		dev->fake_config = 0;
		ep0_idle(dev);
	}

	/* EPOi Transmit Complete - data to host on EP0 ACKed
	 * EP0i Transfer Request - no data in FIFO to send on EP0
	 * either way we send next data if there is any and the FIFO is not busy
	 * it will interrupt again if we later if we don't send anything.
	 */
	if ((f0_status & EP0i_TR || f0_status & EP0i_TS)
	    && (ctrl_inb(USBDASTS) & EP0i_DE) == 0) {
		struct superh_ep *ep = &dev->ep[0];
		struct superh_request *req;
		DBG(DBG_VERY_NOISY, "superh_udc_irq_f1[%lx]: ep0i TR\n",
		    dev->stats.irqs);
		if (!list_empty(&ep->queue)) {
			req =
			    list_entry(ep->queue.next, struct superh_request,
				       queue);
			write_ep0_fifo(ep, req);
		}
		or_b(EP0i_PKTE, USBTRG);
	}

	/* Setup Command Receive Complete */
	if (f0_status & SETUP_TS) {
		DBG(DBG_NOISY, "superh_udc_irq_f1[%lx]: SETUP TS\n",
		    dev->stats.irqs);
		or_b(EP0o_CLEAR | EP0i_CLEAR, USBFCLR);
		handle_ep0_setup(dev);
	}

	/* EPOo Receive Complete - EP0 has received data from host */
	if (f0_status & EP0o_TS) {
		struct superh_request *req;
		struct superh_ep *ep;
		DBG(DBG_VERY_NOISY, "superh_int_hndlr_f1[%lx]: ep0o TS\n",
		    dev->stats.irqs);
		ep = &dev->ep[0];

		if (likely(!list_empty(&ep->queue)))
			req = list_entry(ep->queue.next,
					 struct superh_request, queue);
		else
			req = 0;

		if (req)
			read_ep0_fifo(ep, req);
	}

	/* EP3 Transmit Request & Transmit Complete */
	if (f1_status & (EP3_TR | EP3_TS)) {
		struct superh_request *req;
		struct superh_ep *ep;
		DBG(DBG_VERY_NOISY,
		    "superh_udc_irq_f1[%lx]: EP3 TR | EP3_TS (%x)\n",
		    dev->stats.irqs, f1_status);
		ep = &dev->ep[3];

		if (likely(!list_empty(&ep->queue)))
			req = list_entry(ep->queue.next,
					 struct superh_request, queue);
		else
			req = 0;

		if (req) {
			if ((f1_status & EP3_TR)
			    && (ctrl_inb(USBDASTS) & EP3_DE) == 0)
				write_fifo(ep, req);

		} else {
			pio_irq_disable(ep);
		}
	}

	atomic_dec(&dev->in_interrupt);;

	return IRQ_HANDLED;
}

/*--------------------------------------------------------------------------*/

/*
 * endpoint enable/disable
 *
 * we need to verify the descriptors used to enable endpoints.  since superh
 * endpoint configurations are fixed, and are pretty much always enabled,
 * there's not a lot to manage here.
 *
 */
static int superh_ep_enable(struct usb_ep *_ep,
			    const struct usb_endpoint_descriptor *desc)
{
	struct superh_ep *ep;
	struct superh_udc *dev;

	DBG(DBG_NOISY, "superh_ep_enable\n");

	ep = container_of(_ep, struct superh_ep, ep);
	if (!_ep || !desc || ep->desc || _ep->name == ep0name
	    || desc->bDescriptorType != USB_DT_ENDPOINT
	    || ep->bEndpointAddress != desc->bEndpointAddress
	    || ep->ep.maxpacket < desc->wMaxPacketSize) {
		DMSG("%s, bad ep or descriptor\n", __FUNCTION__);
		return -EINVAL;
	}

	/* xfer types must match, except that interrupt ~= bulk */
	if (ep->bmAttributes != desc->bmAttributes
	    && ep->bmAttributes != USB_ENDPOINT_XFER_BULK
	    && desc->bmAttributes != USB_ENDPOINT_XFER_INT) {
		DMSG("%s, %s type mismatch\n", __FUNCTION__, _ep->name);
		return -EINVAL;
	}
#if 0
	/* hardware _could_ do smaller, but driver doesn't */
	if ((desc->bmAttributes == USB_ENDPOINT_XFER_BULK
	     && desc->wMaxPacketSize != BULK_FIFO_SIZE)
	    || !desc->wMaxPacketSize) {
		DMSG("%s, bad %s maxpacket\n", __FUNCTION__, _ep->name);
		return -ERANGE;
	}
#endif

	dev = ep->dev;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN) {
		DMSG("%s, bogus device state\n", __FUNCTION__);
		return -ESHUTDOWN;
	}

	ep->desc = desc;
	ep->dma = -1;
	ep->stopped = 0;

	/* flush fifo (mostly for OUT buffers), enable irq */
	superh_ep_fifo_flush(_ep);

	/* ... reset halt state too, if we could ... */

#ifdef	USE_DMA

#endif

	DBG(DBG_VERBOSE, "enabled %s\n", _ep->name);
	return 0;
}

static int superh_ep_disable(struct usb_ep *_ep)
{
	struct superh_ep *ep;

	DBG(DBG_NOISY, "superh_ep_disable\n");

	ep = container_of(_ep, struct superh_ep, ep);
	if (!_ep || !ep->desc) {
		DMSG("%s, %s not enabled\n", __FUNCTION__,
		     _ep ? ep->ep.name : NULL);
		return -EINVAL;
	}
	nuke(ep, -ESHUTDOWN);

#ifdef	USE_DMA
	/* TODO */
	if (ep->dma >= 0) {
		*ep->reg_drcmr = 0;
		pxa_free_dma(ep->dma);
		ep->dma = -1;
	}
#endif

	/* flush fifo (mostly for IN buffers) */
	superh_ep_fifo_flush(_ep);

	ep->desc = 0;
	ep->stopped = 1;

	DBG(DBG_VERBOSE, "%s disabled\n", _ep->name);
	return 0;
}

/* for the superh, these can just wrap kmalloc/kfree.  gadget drivers
 * must still pass correctly initialized endpoints, since other controller
 * drivers may care about how it's currently set up (dma issues etc).
 */

/*
 * 	superh_ep_alloc_request - allocate a request data structure
 */
static struct usb_request *superh_ep_alloc_request(struct usb_ep *_ep,
						   int gfp_flags)
{
	struct superh_request *req;

	/* FIXME for bulk out-dma endpoints, preallocate a frame's worth of
	 * (aligned) dma descriptors at the end of the request
	 */

	req = kmalloc(sizeof *req, gfp_flags);
	if (!req)
		return 0;

	memset(req, 0, sizeof *req);
	INIT_LIST_HEAD(&req->queue);
	DBG(DBG_VERY_NOISY, "superh_ep_alloc_request: %p %d\n", req,
	    list_empty(&req->queue));

	return &req->req;
}

/*
 * 	superh_ep_free_request - deallocate a request data structure
 */
static void superh_ep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct superh_request *req;

	req = container_of(_req, struct superh_request, req);
	WARN_ON(!list_empty(&req->queue));
	kfree(req);
}

/* SH cache needs flushing with DMA I/O (it's dma-incoherent), but there's
 * no device-affinity and the heap works perfectly well for i/o buffers.
 * TODO: check this
 */
static void *superh_ep_alloc_buffer(struct usb_ep *_ep, unsigned bytes,
				    dma_addr_t * dma, int gfp_flags)
{
	char *retval;

	retval = kmalloc(bytes, gfp_flags);
	if (retval)
		*dma = virt_to_bus(retval);
	return retval;
}

static void
superh_ep_free_buffer(struct usb_ep *_ep, void *buf, dma_addr_t dma,
		      unsigned bytes)
{
	kfree(buf);
}

static struct superh_request *process_ep_req(struct superh_ep *ep,
					     struct superh_request *req)
{
	struct superh_udc *dev = ep->dev;

	if (ep->desc == 0 /* ep0 */ ) {
		switch (dev->ep0state) {
		case EP0_IN_DATA_PHASE:
			DBG(DBG_VERY_NOISY,
			    "superh_ep_queue: EP0_IN_DATA_PHASE\n");
			dev->stats.write.ops++;
			if (write_ep0_fifo(ep, req))
				req = 0;
			break;

		case EP0_OUT_DATA_PHASE:
			DBG(DBG_VERY_NOISY,
			    "superh_ep_queue: EP0_OUT_DATA_PHASE\n");
			dev->stats.read.ops++;
			if (read_ep0_fifo(ep, req))
				req = 0;
			break;

		default:
			DMSG("ep0 i/o, odd state %d\n", dev->ep0state);
			return 0;
		}
#ifdef	USE_DMA
		/* either start dma or prime pio pump */
	} else if (ep->dma >= 0) {
		kick_dma(ep, req);
#endif
		/* can the FIFO can satisfy the request immediately? */
	} else if ((ep->bEndpointAddress & USB_DIR_IN) != 0) {
		if ((ep->desc->bEndpointAddress & 0x0f) == 2
		    && (ctrl_inb(USBIFR0) & EP2_TR) != 0
		    && write_fifo(ep, req)) {
			req = 0;
		} else if ((ep->desc->bEndpointAddress & 0x0f) == 3
			   && (ctrl_inb(USBIFR1) & EP3_TR) != 0
			   && write_fifo(ep, req)) {
			req = 0;
		}
	}

	if (likely(((req && ep->desc) && ep->dma < 0) || ep->desc == 0))
		pio_irq_enable(ep);

	return req;
}

static int
superh_ep_queue(struct usb_ep *_ep, struct usb_request *_req, int gfp_flags)
{
	struct superh_request *req;
	struct superh_ep *ep;
	struct superh_udc *dev;
	unsigned long flags;

	req = container_of(_req, struct superh_request, req);
	ep = container_of(_ep, struct superh_ep, ep);

	DBG(DBG_VERY_NOISY, "superh_ep_queue\n");

	/* If we have just sent a fake configuration request then
	 * this is the reply.  We don't want to send it to the host
	 * so just ignore it.
	 */
	if (ep->desc == 0 /* ep0 */  && ep->dev->fake_config) {
		DBG(DBG_NOISY, "Ignoring bogus SET_CONFIGURATION response\n");
		done(ep, req, 0);
		ep->dev->fake_config = 0;
		return 1;
	}

	if (unlikely(!_req || !_req->complete || !_req->buf
		     || !list_empty(&req->queue))) {
		DMSG("%s, bad params %s, %p, %p, %p, %d\n", __FUNCTION__,
		     ep->ep.name, _req, _req->complete, _req->buf,
		     list_empty(&req->queue));
		return -EINVAL;
	}

	if (unlikely(!_ep || (!ep->desc && ep->ep.name != ep0name))) {
		DMSG("%s, bad ep\n", __FUNCTION__);
		return -EINVAL;
	}

	dev = ep->dev;
	if (unlikely(!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)) {
		DMSG("%s, bogus device state\n", __FUNCTION__);
		return -ESHUTDOWN;
	}
#ifdef	USE_DMA
	/* TODO */
	if (ep->dma >= 0) {
		unsigned long start = (unsigned long)_req->buf;

		clean_dcache_range(start, start + _req->length);
		/* or for USB_DIR_OUT, invalidate_dcache_range (...) */
	}
#endif

	DBG(DBG_NOISY, "%s queue req %p, len %d buf %p\n",
	    _ep->name, _req, _req->length, _req->buf);

	local_irq_save(flags);

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	/* kickstart this i/o queue? */
	if (list_empty(&ep->queue) && !ep->stopped && !ep->halted) {
		req = process_ep_req(ep, req);
	}

	/* pio or dma irq handler advances the queue. */
	if (likely(req != 0))
		list_add_tail(&req->queue, &ep->queue);

	local_irq_restore(flags);

	return 0;
}

static int superh_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct superh_ep *ep;
	struct superh_request *req;
	unsigned long flags;

	DBG(DBG_NOISY, "superh_ep_dequeue %s\n", _ep->name);

	ep = container_of(_ep, struct superh_ep, ep);
	req = container_of(_req, struct superh_request, req);
	if (!_ep || !_req || ep->ep.name == ep0name)
		return -EINVAL;

	local_irq_save(flags);
#ifdef	USE_DMA
	if (ep->dma >= 0 && ep->queue.next == &req->queue && !ep->stopped) {
		cancel_dma(ep);
		done(ep, req, -ECONNRESET);
		/* restart i/o */
		if (!list_empty(&ep->queue)) {
			req = list_entry(ep->queue.next,
					 struct superh_request, queue);
			kick_dma(ep, req);
		}
	} else
#endif
	if (!list_empty(&req->queue))
		done(ep, req, -ECONNRESET);
	else
		req = 0;
	local_irq_restore(flags);

	return req ? 0 : -EOPNOTSUPP;
}

/* stall/unstall an endpoint, 0 clears the stall, 1 sets it */
static int superh_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct superh_ep *ep;
	unsigned long flags;

	ep = container_of(_ep, struct superh_ep, ep);
	if (unlikely(!_ep || (!ep->desc && ep->ep.name != ep0name))
	    || ep->bmAttributes == USB_ENDPOINT_XFER_ISOC) {
		DMSG("%s, bad ep\n", __FUNCTION__);
		return -EINVAL;
	}

	if (ep->halted == value)
		return 0;

	local_irq_save(flags);

	if (value == 1 && (ep->bEndpointAddress & USB_DIR_IN) != 0
	    && ((ctrl_inb(USBDASTS) & ep->data_present_mask) != 0
		|| !list_empty(&ep->queue))) {
		local_irq_restore(flags);
		DBG(DBG_VERBOSE, "Can't %s on %s\n",
		    value ? " halt" : "clear halt", _ep->name);
		return -EAGAIN;
	}

	if (value) {
		or_b(ep->stall_mask, USBEPSTL);
		if (!ep->desc) {
			ep->dev->ep0state = EP0_STALL;
		}
		/* disable ep interrupts and set a timer to clear the stall */
		pio_irq_disable(ep);
		mod_timer(&ep->dev->timer, jiffies + STALL_TIME);
	} else {
		and_b(~ep->stall_mask, USBEPSTL);
	}

	ep->halted = value;

	local_irq_restore(flags);

	DBG(DBG_VERBOSE, "%s %s\n", _ep->name, value ? " halt" : "clear halt");

	return 0;
}

static int superh_ep_fifo_status(struct usb_ep *_ep)
{
	struct superh_ep *ep;

	DBG(DBG_NOISY, "superh_ep_fifo_status\n");

	ep = container_of(_ep, struct superh_ep, ep);
	if (!_ep) {
		DMSG("%s, bad ep\n", __FUNCTION__);
		return -ENODEV;
	}
	if ((ep->bEndpointAddress & USB_DIR_IN) != 0)
		return -EOPNOTSUPP;
	if (ep->dev->gadget.speed == USB_SPEED_UNKNOWN)
		return 0;
	else {
		switch (ep->desc->bEndpointAddress & 0x0f) {
		case 0:
			return ctrl_inb(USBEPSZ0O);
		case 1:
			return ctrl_inb(USBEPSZ1);
		}
	}

	return 0;
}

static void superh_ep_fifo_flush(struct usb_ep *_ep)
{
	struct superh_ep *ep;

	DBG(DBG_NOISY, "superh_ep_fifo_flush\n");

	ep = container_of(_ep, struct superh_ep, ep);
	if (!_ep || ep->ep.name == ep0name || !list_empty(&ep->queue)) {
		DMSG("%s, bad ep\n", __FUNCTION__);
		return;
	}

	or_b(ep->clear_mask, USBFCLR);
}

static struct usb_ep_ops superh_ep_ops = {
	.enable = superh_ep_enable,
	.disable = superh_ep_disable,

	.alloc_request = superh_ep_alloc_request,
	.free_request = superh_ep_free_request,

	.alloc_buffer = superh_ep_alloc_buffer,
	.free_buffer = superh_ep_free_buffer,

	.queue = superh_ep_queue,
	.dequeue = superh_ep_dequeue,

	.set_halt = superh_ep_set_halt,
	.fifo_status = superh_ep_fifo_status,
	.fifo_flush = superh_ep_fifo_flush,
};

/* ---------------------------------------------------------------------------
 * 	device-scoped parts of the api to the usb controller hardware
 * ---------------------------------------------------------------------------
 */

static int superh_udc_get_frame(struct usb_gadget *_gadget)
{
	DBG(DBG_VERY_NOISY, "superh_udc_get_frame\n");

	return -EOPNOTSUPP;
}

static const struct usb_gadget_ops superh_udc_ops = {
	.get_frame = superh_udc_get_frame,
	/* no remote wakeup    */
	/* always selfpowered  */
};

/* if we're trying to save space, don't bother with this proc file */

#if defined(CONFIG_PROC_FS) && !defined(CONFIG_EMBEDDED)
#  define	UDC_PROC_FILE
#endif

#ifdef UDC_PROC_FILE

static const char proc_node_name[] = "driver/udc";

static int
udc_proc_read(char *page, char **start, off_t off, int count,
	      int *eof, void *_dev)
{
	char *buf = page;
	struct superh_udc *dev = _dev;
	char *next = buf;
	unsigned size = count;
	unsigned long flags;
	int t;
	int i;

	local_irq_save(flags);

	/* basic device status */
	t = snprintf(next, size,
		     "%s\n%s version: %s\nGadget driver: %s\nHost %s\n\n",
		     driver_desc,
		     driver_name, DRIVER_VERSION,
		     dev->driver ? dev->driver->driver.name : "(none)",
		     is_usb_connected ? "full speed" : "disconnected");
	size -= t;
	next += t;

	/* device registers */
	t = snprintf(next, size,
		     "ifr0 %02X, ifr1 %02X, isr0 %02X, isr1 %02X, ier0 %02X, ier1 %02X\n",
		     ctrl_inb(USBIFR0), ctrl_inb(USBIFR1),
		     ctrl_inb(USBISR0), ctrl_inb(USBISR1),
		     ctrl_inb(USBIER0), ctrl_inb(USBIER1));
	size -= t;
	next += t;

	t = snprintf(next, size,
		     "epsz0o %02X, epsz1 %02X, dasts %02X, dmar %02X\n",
		     ctrl_inb(USBEPSZ0O), ctrl_inb(USBEPSZ1),
		     ctrl_inb(USBDASTS), ctrl_inb(USBDMA));
	size -= t;
	next += t;

#if defined(CONFIG_CPU_SUBTYPE_SH7705)
	t = snprintf(next, size,
		     "epstl %02X, xvercr %02X\n",
		     ctrl_inb(USBEPSTL), ctrl_inb(USBXVERCR));
#elif defined(CONFIG_CPU_SUBTYPE_SH7727)
	t = snprintf(next, size, "epstl %02X\n", ctrl_inb(USBEPSTL));
#endif
	size -= t;
	next += t;

	if (!is_usb_connected || !dev->driver)
		goto done;

	t = snprintf(next, size,
		     "ep0 IN %lu/%lu, OUT %lu/%lu; irq0s %lu; irq1s %lu\n\n",
		     dev->stats.write.bytes, dev->stats.write.ops,
		     dev->stats.read.bytes, dev->stats.read.ops,
		     dev->stats.irq0s, dev->stats.irq1s);
	size -= t;
	next += t;

	/* dump endpoint queues */
	for (i = 0; i < 4; i++) {
		struct superh_ep *ep = &dev->ep[i];
		struct superh_request *req;
		int t;

		if (i != 0) {
			const struct usb_endpoint_descriptor *d;

			d = ep->desc;
			if (!d)
				continue;
			t = snprintf(next, size,
				     "%s max %d %s\n",
				     ep->ep.name,
				     le16_to_cpu(d->wMaxPacketSize),
				     (ep->dma >= 0) ? "dma" : "pio");

		} else		/* ep0 should only have one transfer queued */
			t = snprintf(next, size, "ep0 max 8 pio\n");
		if (t <= 0 || t > size)
			goto done;
		size -= t;
		next += t;

		if (list_empty(&ep->queue)) {
			t = snprintf(next, size, "\t(nothing queued)\n");
			if (t <= 0 || t > size)
				goto done;
			size -= t;
			next += t;
			continue;
		}
		list_for_each_entry(req, &ep->queue, queue) {
#ifdef	USE_DMA
			if (ep->dma >= 0 && req->queue.prev == &ep->queue)
				t = snprintf(next, size,
					     "\treq %p len %d/%d "
					     "buf %p (dma%d dcmd %08x)\n",
					     &req->req, req->req.actual,
					     req->req.length, req->req.buf,
					     ep->dma, DCMD(ep->dma)
					     /* low 13 bits == bytes-to-go */
				    );
			else
#endif
				t = snprintf(next, size,
					     "\treq %p len %d/%d buf %p\n",
					     &req->req, req->req.actual,
					     req->req.length, req->req.buf);
			if (t <= 0 || t > size)
				goto done;
			size -= t;
			next += t;
		}
	}

      done:
	local_irq_restore(flags);
	return count - size;
}

#endif				/* UDC_PROC_FILE */

/*-------------------------------------------------------------------------*/

/*
 * 	udc_disable - disable USB device controller
 */
static void udc_disable(struct superh_udc *dev)
{
	/* block all irqs */
	ctrl_outb(0, USBIER0);
	ctrl_outb(0, USBIER1);

	/* Disable the USB module */
#if defined(CONFIG_CPU_SUBTYPE_SH7705)
	or_b(0x80, STBCR3);
#elif defined(CONFIG_CPU_SUBTYPE_SH7727)
	or_b(0x10, STBCR3);
#endif

	/* Disable the USB clock */
#if defined(CONFIG_CPU_SUBTYPE_SH7705)
	ctrl_outw(0xA500, UCLKCR);
#elif defined(CONFIG_CPU_SUBTYPE_SH7727)
	ctrl_outw((ctrl_inw(EXCPGCR) | USBDIVS_EL2), EXCPGCR);
#endif

	ep0_idle(dev);
	dev->gadget.speed = USB_SPEED_UNKNOWN;
}

/*
 * 	udc_reinit - initialize software state
 */
static void udc_reinit(struct superh_udc *dev)
{
	u32 i;

	/* device/ep0 records init */
	INIT_LIST_HEAD(&dev->gadget.ep_list);
	dev->gadget.ep0 = &dev->ep[0].ep;
	INIT_LIST_HEAD(&dev->gadget.ep0->ep_list);
	dev->ep0state = EP0_IDLE;

	/* basic endpoint records init */
	for (i = 0; i < 4; i++) {
		struct superh_ep *ep = &dev->ep[i];

		ep->ep.name = ep_name[i];
		ep->ep.ops = &superh_ep_ops;
		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &dev->gadget.ep_list);

		ep->dev = dev;
		ep->desc = 0;
		ep->stopped = 0;
		ep->halted = 0;
		ep->dma = -1;
		INIT_LIST_HEAD(&ep->queue);

		/* address may need USB_DIR_IN, attributes likely wrong */
		ep->bEndpointAddress = i;
		ep->bmAttributes = USB_ENDPOINT_XFER_BULK;
	}

	/* TODO at least from here on, static initialization
	 * would work just as well and would need less code space
	 */

	/* ep0 == control */
	dev->ep[0].ep.maxpacket = EP0_FIFO_SIZE;
	dev->ep[0].data_present_mask = EP0i_DE;
	dev->ep[0].stall_mask = EP0_STL;
	dev->ep[0].interrupt_mask = EP0o_TS | EP0i_TR | EP0i_TS;
	dev->ep[0].interrupt_reg = USBIER0;
	dev->ep[0].clear_mask = EP0i_CLEAR | EP0o_CLEAR;
	dev->ep[0].fifo_reg = 0;
	dev->ep[0].packet_enable_mask = 0;

	dev->ep[1].ep.maxpacket = BULK_FIFO_SIZE;
	dev->ep[1].bEndpointAddress |= USB_DIR_OUT;
	dev->ep[1].data_present_mask = 0x00;
	dev->ep[1].stall_mask = EP1_STL;
	dev->ep[1].interrupt_mask = EP1_FULL;
	dev->ep[1].interrupt_reg = USBIER0;
	dev->ep[1].clear_mask = EP1_CLEAR;
	dev->ep[1].fifo_reg = 0;
	dev->ep[1].packet_enable_mask = 0;

	dev->ep[2].ep.maxpacket = BULK_FIFO_SIZE;
	dev->ep[2].bEndpointAddress |= USB_DIR_IN;
	dev->ep[2].data_present_mask = EP2_DE;
	dev->ep[2].stall_mask = EP2_STL;
	dev->ep[2].interrupt_mask = EP2_TR | EP2_EMPTY;
	dev->ep[2].interrupt_reg = USBIER0;
	dev->ep[2].clear_mask = EP2_CLEAR;
	dev->ep[2].fifo_reg = USBEPDR2;
	dev->ep[2].packet_enable_mask = EP2_PKTE;

	dev->ep[3].ep.maxpacket = INT_FIFO_SIZE;
	dev->ep[3].bEndpointAddress |= USB_DIR_IN;
	dev->ep[3].data_present_mask = EP3_DE;
	dev->ep[3].stall_mask = EP3_STL;
	dev->ep[3].interrupt_mask = EP3_TR | EP3_TS;
	dev->ep[3].interrupt_reg = USBIER1;
	dev->ep[3].clear_mask = EP3_CLEAR;
	dev->ep[3].fifo_reg = USBEPDR3;
	dev->ep[3].packet_enable_mask = EP3_PKTE;
}

/* until it's enabled, this UDC should be completely invisible
 * to any USB host.
 */
static void udc_enable(struct superh_udc *dev)
{
#if defined(CONFIG_CPU_SUBTYPE_SH7727)
	/* Reset and then Select Function USB1_pwr_en out (USB) c.f.
	   Section 26, Table 26.1 PTE2 */
	and_w(PN_PB2_MSK, PORT_PECR);
	or_w(PN_PB2_OF, PORT_PECR);

	/* Reset and then Select Function UCLK c.f.
	   Section 26, Table 26.1, PTD6 */
	and_w(PN_PB6_MSK, PORT_PDCR);
	or_w(PN_PB6_OF, PORT_PDCR);

	/* Stop USB module prior to setting clocks c.f. Section 9.2.3 */
	and_b(~MSTP14, STBCR3);
	or_b(MSTP14, STBCR3);

	/* Select external clock, 1/1 divisor c.f. Section 11.3.1 */
	or_b(USBDIV_11 | USBCKS_EC, EXCPGCR);

	/* Start USB c.f. Section 9.2.3 */
	and_b(~MSTP14, STBCR3);

	/* Disable pullup c.f. Section 23.5.19 */
	or_b(PULLUP_E, USBDMA);

	/* Set port 1 to function, disabled c.f. Section 22.2.1
	   or_w(USB_TRANS_TRAN | USB_SEL_FUNC, EXPFC);

	   /* Enable pullup c.f. Section 23.5.19a */
	and_b(~PULLUP_E, USBDMA);
#elif defined(CONFIG_CPU_SUBTYPE_SH7705)
	/* Disable the USB module */
	or_b(MSTP37, STBCR3);

	/* Set the clock to external & enable */
	ctrl_outw(0xA5E0, UCLKCR);

	/* Enable the USB module */
	and_b(~MSTP37, STBCR3);

	/* Enable USB pins. */
	ctrl_outw(0x01FD, PORT_PMCR);	/* VBUS */
	or_b(PULLUP_E, PORT_PMDR);

#if defined(CONFIG_SH_SOLUTION_ENGINE)
	ctrl_outw(((ctrl_inw(PORT_PDCR) & 0xFFF3) | 0x0004), PORT_PDCR);	/* PTD1=output */
	/* Disable pullup */
	and_b(~PULLUP_E, PORT_PDDR);	/* PortD1 = low */
	/* Enable pullup */
	or_b(PULLUP_E, PORT_PDDR);	/* PortD1 = high */
#endif
#endif
	dev->gadget.speed = USB_SPEED_UNKNOWN;
	dev->stats.irqs = 0;
	dev->stats.irq0s = 0;
	dev->stats.irq1s = 0;

	/* reset fifo's and stall's */
	or_b(EP3_CLEAR | EP1_CLEAR | EP2_CLEAR | EP0o_CLEAR | EP0i_CLEAR,
	     USBFCLR);
	or_b(0, USBEPSTL);

	/* Setup interrupt priority by using the interrupt select registers */
	ctrl_outb(F0_LOW, USBISR0);
	ctrl_outb(F1_LOW, USBISR1);

	/* Enable some interrupts */
	or_b(BRST | SETUP_TS | EP0o_TS | EP0i_TR | EP0i_TS, USBIER0);
	or_b(VBUSF, USBIER1);
}

/* when a driver is successfully registered, it will receive
 * control requests including set_configuration(), which enables
 * non-control requests.  then usb traffic follows until a
 * disconnect is reported.  then a host may connect again, or
 * the driver might get unbound.
 */
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct superh_udc *dev = the_controller;
	int retval;

	if (!driver
	    /*|| driver->speed != USB_SPEED_FULL
	       || !driver->bind
	       || !driver->unbind
	       || !driver->disconnect
	       || !driver->setup */ )
		return -EINVAL;
	if (!dev)
		return -ENODEV;
	if (dev->driver)
		return -EBUSY;

	/* first hook up the driver ... */
	dev->driver = driver;

	retval = driver->bind(&dev->gadget);
	if (retval) {
		DMSG("bind to driver %s --> error %d\n",
		     driver->driver.name, retval);
		dev->driver = 0;
		return retval;
	}

	/* ... then enable host detection and ep0; and we're ready
	 * for set_configuration as well as eventual disconnect.
	 * NOTE:  this shouldn't power up until later.
	 */
	udc_enable(dev);

	DMSG("registered gadget driver '%s'\n", driver->driver.name);
	dump_state(dev);
	return 0;
}

EXPORT_SYMBOL(usb_gadget_register_driver);

static void
stop_activity(struct superh_udc *dev, struct usb_gadget_driver *driver)
{
	int i;

	/* don't disconnect drivers more than once */
	if (dev->gadget.speed == USB_SPEED_UNKNOWN)
		driver = 0;
	dev->gadget.speed = USB_SPEED_UNKNOWN;

	/* prevent new request submissions, kill any outstanding requests  */
	for (i = 0; i < 4; i++) {
		struct superh_ep *ep = &dev->ep[i];

		ep->stopped = 1;
		nuke(ep, -ESHUTDOWN);
	}

	del_timer_sync(&dev->timer);

	/* report disconnect; the driver is already quiesced */
	if (driver)
		driver->disconnect(&dev->gadget);

	/* re-init driver-visible data structures */
	udc_reinit(dev);
}

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct superh_udc *dev = the_controller;

	if (!dev)
		return -ENODEV;
	if (!driver || driver != dev->driver)
		return -EINVAL;

	local_irq_disable();
	udc_disable(dev);
	stop_activity(dev, driver);
	driver->unbind(&dev->gadget);
	dev->driver = 0;
	local_irq_enable();

	DMSG("unregistered gadget driver '%s'\n", driver->driver.name);
	dump_state(dev);
	return 0;
}

EXPORT_SYMBOL(usb_gadget_unregister_driver);

/*-------------------------------------------------------------------------*/

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,40)
MODULE_DESCRIPTION(driver_desc);
#endif
MODULE_AUTHOR("Julian Back");
MODULE_LICENSE("GPL");

/*
 *	cleanup - free resources allocated during init
 */
static void /*__exit and */ __init cleanup(void)
{
	struct superh_udc *dev = the_controller;

	if (!dev)
		return;

	udc_disable(dev);
#ifdef	UDC_PROC_FILE
	remove_proc_entry(proc_node_name, NULL);
#endif
	usb_gadget_unregister_driver(dev->driver);

	if (dev->got_irq0) {
		free_irq(USBFI0_IRQ, dev);
		dev->got_irq0 = 0;
	}

	if (dev->got_irq1) {
		free_irq(USBFI1_IRQ, dev);
		dev->got_irq1 = 0;
	}

	the_controller = 0;
}

module_exit(cleanup);

/*
 * 	init - allocate resources
 */
static int __init init(void)
{
	static struct superh_udc memory;

	struct superh_udc *dev;
	int retval;

	printk(KERN_DEBUG "%s: version %s\n", driver_name, DRIVER_VERSION);

	/* initialize data */
	dev = &memory;

	memset(dev, 0, sizeof *dev);
	dev->gadget.ops = &superh_udc_ops;
	dev->gadget.name = driver_name;
	dev->gadget.dev.bus_id[0] = 'g';
	dev->gadget.dev.bus_id[1] = 'a';
	dev->gadget.dev.bus_id[2] = 'd';
	dev->gadget.dev.bus_id[3] = 'g';
	dev->gadget.dev.bus_id[4] = 'e';
	dev->gadget.dev.bus_id[5] = 't';
	dev->gadget.speed = USB_SPEED_UNKNOWN;

	dev->vbusmn = 0;

	atomic_set(&dev->in_interrupt, 0);

	the_controller = dev;
	udc_disable(dev);
	udc_reinit(dev);

	/* irq setup after old hardware state is cleaned up */
	retval = request_irq(USBFI0_IRQ, superh_udc_irq_f0,
			     0 /*SA_INTERRUPT | SA_SAMPLE_RANDOM */ ,
			     driver_name, dev);
	if (retval != 0) {
		printk(KERN_ERR "%s: can't get irq %i, err %d\n",
		       driver_name, USBFI0_IRQ, retval);
		goto failed;
	}
	dev->got_irq0 = 1;

	retval = request_irq(USBFI1_IRQ, superh_udc_irq_f1,
			     0 /*SA_INTERRUPT | SA_SAMPLE_RANDOM */ ,
			     driver_name, dev);
	if (retval != 0) {
		printk(KERN_ERR "%s: can't get irq %i, err %d\n",
		       driver_name, USBFI1_IRQ, retval);
		goto failed;
	}
	dev->got_irq1 = 1;

	printk(KERN_INFO "%s, IRQs %d %d\n", driver_desc,
	       USBFI0_IRQ, USBFI1_IRQ);
	dump_state(dev);

	dev->setup_countdown = DEFAULT_SETUP_COUNT;

#ifdef	UDC_PROC_FILE
	create_proc_read_entry(proc_node_name, 0, NULL, udc_proc_read, dev);
#endif

	return 0;

      failed:
	cleanup();
	return retval;
}

module_init(init);
