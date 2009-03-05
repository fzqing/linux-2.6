/*
 * linux/drivers/usb/gadget/pxa27x_udc.c
 * Intel pxa27x on-chip full speed USB device controller
 *
 * Copyright (C) 2002 Intrinsyc, Inc. (Frank Becker)
 * Copyright (C) 2003 Robert Schwebel, Pengutronix
 * Copyright (C) 2003 Benedikt Spranger, Pengutronix
 * Copyright (C) 2003 David Brownell
 * Copyright (C) 2003 Joshua Wise
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
 *
 */

#undef       DEBUG
#undef       VERBOSE

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

#include <asm/byteorder.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <asm/unaligned.h>
#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>

#include <linux/usb_ch9.h>
#include <linux/usb_gadget.h>

#include <asm/arch/udc.h>

#define	DRIVER_VERSION	"0.9"
#define	DRIVER_DESC	"PXA 27x USB Device Controller driver"

static const char driver_name[] = "pxa27x_udc";
static char ep0name[] = "ep0";

#include "pxa27x_udc.h"

#define	DMA_ADDR_INVALID	(~(dma_addr_t)0)

#ifdef	CONFIG_USB_PXA27X_DMA
static int use_dma = 1;
module_param(use_dma, bool, 0);
MODULE_PARM_DESC(use_dma, "true to use dma");

static void dma_nodesc_handler(int dmach, void *_ep, struct pt_regs *r);
static void kick_dma(struct pxa27x_ep *ep, struct pxa27x_request *req);

#define	DMASTR " (dma support)"

#else				/* !CONFIG_USB_PXA27X_DMA */
#define	DMASTR " (pio only)"
#endif

static unsigned udc_config_default = 1;
static struct udc_config_desc udc_configs[PXA_UDC_NUM_CONFIGS];

/* ---------------------------------------------------------------------------
 * 	endpoint related parts of the api to the usb controller hardware,
 *	used by gadget driver; and the inner talker-to-hardware core.
 * ---------------------------------------------------------------------------
 */
static void pxa27x_ep_fifo_flush(struct usb_ep *ep);
static void nuke(struct pxa27x_ep *, int status);

static void pio_irq_enable(int phys_ep, u8 irq_type)
{
	irq_type &= (UDC_INT_FIFOERROR | UDC_INT_PACKETCMP);
	if (phys_ep < 16) {
		UDCICR0 |= irq_type << (phys_ep << 1);
	} else {
		UDCICR1 |= irq_type << ((phys_ep - 16) << 1);
	}
}

static void pio_irq_disable(int phys_ep, u8 irq_type)
{
	irq_type &= (UDC_INT_FIFOERROR | UDC_INT_PACKETCMP);
	if (phys_ep < 16) {
		UDCICR0 &= ~(irq_type << (phys_ep << 1));
	} else {
		UDCICR1 &= ~(irq_type << ((phys_ep - 16) << 1));
	}
}

static void pio_reset_irq_status(int phys_ep)
{
	if (phys_ep < 16) {
		UDCISR0 = (UDCISR0 & 
			   ((UDC_INT_FIFOERROR | UDC_INT_PACKETCMP) << 
			   (phys_ep << 1)));
	} else {
		UDCISR1 = (UDCISR1 & 
			   ((UDC_INT_FIFOERROR | UDC_INT_PACKETCMP) << 
			   ((phys_ep - 16) << 1)));
	}
}

/*
 * endpoint enable/disable
 */
static int pxa27x_ep_enable(struct usb_ep *_ep,
			    const struct usb_endpoint_descriptor *desc)
{
	struct pxa27x_ep *ep;
	struct pxa27x_udc *dev;

	ep = container_of(_ep, struct pxa27x_ep, ep);

	if (!_ep || !desc || ep->desc || _ep->name == ep0name
	    || desc->bDescriptorType != USB_DT_ENDPOINT
	    || ep->bEndpointAddress != desc->bEndpointAddress
	    || ep->fifo_size < le16_to_cpu(desc->wMaxPacketSize)) {
		pr_debug(PREFIX "%s, bad ep or descriptor\n", __FUNCTION__);
		return -EINVAL;
	}

	/* xfer types must match, except that interrupt ~= bulk */
	if (ep->bmAttributes != desc->bmAttributes
	    && ep->bmAttributes != USB_ENDPOINT_XFER_BULK
	    && desc->bmAttributes != USB_ENDPOINT_XFER_INT) {
		pr_debug(PREFIX "%s, %s type mismatch\n", __FUNCTION__,
			 _ep->name);
		return -EINVAL;
	}

	/* hardware _could_ do smaller, but driver doesn't */
	if ((desc->bmAttributes == USB_ENDPOINT_XFER_BULK
	     && le16_to_cpu(desc->wMaxPacketSize)
	     != BULK_FIFO_SIZE)
	    || !desc->wMaxPacketSize) {
		pr_debug(PREFIX "%s, bad %s maxpacket\n", __FUNCTION__,
			 _ep->name);
		return -ERANGE;
	}

	dev = ep->dev;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN) {
		pr_debug(PREFIX "%s, bogus device state\n", __FUNCTION__);
		return -ESHUTDOWN;
	}

	ep->desc = desc;
	ep->dma = -1;
	ep->stopped = 0;
	ep->pio_irqs = ep->dma_irqs = 0;
	ep->ep.maxpacket = le16_to_cpu(desc->wMaxPacketSize);

	/* flush fifo (mostly for OUT buffers) */
	pxa27x_ep_fifo_flush(_ep);

	/* ... reset halt state too, if we could ... */

#ifdef	CONFIG_USB_PXA27X_DMA
	/* for (some) bulk and ISO endpoints, try to get a DMA channel and
	 * bind it to the endpoint.  otherwise use PIO. 
	 */
	switch (ep->bmAttributes) {
	case USB_ENDPOINT_XFER_ISOC:
		if (le16_to_cpu(desc->wMaxPacketSize) % 32)
			break;
		/* fall through */
	case USB_ENDPOINT_XFER_BULK:
		if (!use_dma || !ep->reg_drcmr)
			break;
		ep->dma = pxa_request_dma((char *)_ep->name,
				(le16_to_cpu(desc->wMaxPacketSize) > 64)
					  ? DMA_PRIO_MEDIUM	/* some iso */
					  : DMA_PRIO_LOW,
					  dma_nodesc_handler, ep);
		if (ep->dma >= 0) {
			*ep->reg_drcmr = DRCMR_MAPVLD | ep->dma;
			DALGN |= 1 << ep->dma;
			pr_debug(PREFIX "%s using dma%d, DALGN %X\n", _ep->name,
				 ep->dma, DALGN);
		}
	}
#endif

	DBG(DBG_VERBOSE, "enabled %s\n", _ep->name);
	return 0;
}

static int pxa27x_ep_disable(struct usb_ep *_ep)
{
	struct pxa27x_ep *ep;
	unsigned long     flags;

	ep = container_of(_ep, struct pxa27x_ep, ep);
	if (!_ep || !ep->desc) {
		pr_debug(PREFIX "%s, %s not enabled\n", __FUNCTION__,
			 _ep ? ep->ep.name : NULL);
		return -EINVAL;
	}

	local_irq_save_nort(flags);
	spin_lock_rt(&ep->dev->lock);

	nuke(ep, -ESHUTDOWN);

#ifdef	CONFIG_USB_PXA27X_DMA
	if (ep->dma >= 0) {
		*ep->reg_drcmr = 0;
		DALGN &= ~(1 << ep->dma);
		pxa_free_dma(ep->dma);
		ep->dma = -1;
	}
#endif
	/* flush fifo (mostly for IN buffers) */
	pxa27x_ep_fifo_flush(_ep);

	ep->desc = 0;
	ep->stopped = 1;

	local_irq_restore_nort(flags);
	spin_unlock_rt(&ep->dev->lock);

	DBG(DBG_VERBOSE, "%s disabled\n", _ep->name);
	return 0;
}

/*-------------------------------------------------------------------------*/
/*
 * 	pxa27x_ep_alloc_request - allocate a request data structure
 */
static struct usb_request *pxa27x_ep_alloc_request(struct usb_ep *_ep,
						   int gfp_flags)
{
	struct pxa27x_request *req;

	req = kmalloc(sizeof *req, gfp_flags);
	if (!req)
		return 0;

	memset(req, 0, sizeof *req);
	req->req.dma = DMA_ADDR_INVALID;
	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}

/*
 * 	pxa27x_ep_free_request - deallocate a request data structure
 */
static void pxa27x_ep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct pxa27x_request *req;

	req = container_of(_req, struct pxa27x_request, req);
	WARN_ON(!list_empty(&req->queue));
	kfree(req);
}

/* PXA cache needs flushing with DMA I/O (it's dma-incoherent), but there's
 * no device-affinity and the heap works perfectly well for i/o buffers.
 * It wastes much less memory than dma_alloc_coherent() would, and even
 * prevents cacheline (32 bytes wide) sharing problems.
 */
static void *pxa27x_ep_alloc_buffer(struct usb_ep *_ep, unsigned bytes,
				    dma_addr_t * dma, int gfp_flags)
{
	char *retval;

	retval = kmalloc(bytes, gfp_flags & ~(__GFP_DMA | __GFP_HIGHMEM));
	if (retval)
		*dma = virt_to_bus(retval);
	return retval;
}

static void
pxa27x_ep_free_buffer(struct usb_ep *_ep, void *buf, dma_addr_t dma,
		      unsigned bytes)
{
	kfree(buf);
}

/*-------------------------------------------------------------------------*/
/*
 *	done - retire a request; caller blocked irqs
 */
static void done(struct pxa27x_ep *ep, struct pxa27x_request *req, int status)
{
	unsigned stopped = ep->stopped;
	struct pxa27x_udc *dev;

	dev = ep->dev;
	list_del_init(&req->queue);

	if (likely(req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

#ifdef CONFIG_USB_PXA27X_DMA
	if (use_dma && (ep->dma >= 0)) {
		if (req->mapped) {
			dma_unmap_single(dev->dev,
					 req->req.dma, req->req.length,
					 (ep->bEndpointAddress & USB_DIR_IN)
					 ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
			req->req.dma = DMA_ADDR_INVALID;
			req->mapped = 0;
		} else
			dma_sync_single_for_cpu(dev->dev,
						req->req.dma, req->req.length,
						(ep->
						 bEndpointAddress & USB_DIR_IN)
						? DMA_TO_DEVICE :
						DMA_FROM_DEVICE);
	}
#endif
	if (status && status != -ESHUTDOWN)
		DBG(DBG_VERBOSE, "complete %s req %p stat %d len %u/%u\n",
		    ep->ep.name, &req->req, status,
		    req->req.actual, req->req.length);

	/* don't modify queue heads during completion callback */
	ep->stopped = 1;

	spin_unlock_rt(&dev->lock);
	req->req.complete(&ep->ep, &req->req);
	spin_lock_rt(&dev->lock);

	ep->stopped = stopped;

}

static inline void ep0_idle(struct pxa27x_udc *dev)
{
	dev->ep0state = EP0_IDLE;
}

static int
write_packet(volatile u32 * udcdr, volatile u8 * pudcdr,
	     struct pxa27x_request *req, unsigned max)
{
	u32 *buf;
	u8 *rd;
	unsigned length = 0, count = 0, remain = 0;

	buf = (u32 *) (req->req.buf + req->req.actual);
	prefetch(buf);

	/* how big will this packet be? */
	length = min(req->req.length - req->req.actual, max);
	req->req.actual += length;

	remain = length & 0x03;
	count = length & ~0x03;

	while (likely(count > 0)) {
		*udcdr = *buf++;
		count -= 4;
	}

	if (remain) {
		volatile u8 *reg = pudcdr;

		rd = (u8 *) buf;
		while (remain--) {
			*reg = *rd++;
		}
	}

	return length;
}

/*
 * write to an IN endpoint fifo, as many packets as possible.
 * irqs will use this to write the rest later.
 * caller guarantees at least one packet buffer is ready (or a zlp).
 */
static int write_fifo(struct pxa27x_ep *ep, struct pxa27x_request *req)
{
	unsigned max;

	max = le16_to_cpu(ep->desc->wMaxPacketSize);
	do {
		unsigned count;
		int is_last, is_short;

		count = write_packet(ep->reg_udcdr, ep->reg_pudcdr, req, max);

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
			is_short = unlikely(max < ep->fifo_size);
		}

		DBG(DBG_NOISY, "wrote %s , phys %d, %d bytes%s%s %d left %p\n",
		    ep->ep.name, ep->phys, count,
		    is_last ? "/L" : "", is_short ? "/S" : "",
		    req->req.length - req->req.actual, req);

		/* let loose that packet. maybe try writing another one,
		 * double buffering might work.  SP, PC, and FS
		 * bit values are the same for all normal IN endpoints.
		 */
		*ep->reg_udccsr = UDCCSR_PC;
		if (is_short)
			*ep->reg_udccsr = UDCCSR_SP;

		/* requests complete when all IN data is in the FIFO */
		if (is_last) {
			done(ep, req, 0);
			if (list_empty(&ep->queue))
				pio_irq_disable(ep->phys,
						(UDC_INT_FIFOERROR |
						 UDC_INT_PACKETCMP));
			return 1;
		}
		/*
		 * TODO experiment: how robust can fifo mode tweaking be?
		 * double buffering is off in the default fifo mode, which
		 * prevents TFS from being set here.
		 */
	} while (*ep->reg_udccsr & UDCCSR_FS);
	return 0;
}

/* caller asserts req->pending (ep0 irq status nyet cleared); starts
 * ep0 data stage.  these chips want very simple state transitions.
 */
static inline void ep0start(struct pxa27x_udc *dev, u32 flags, const char *tag)
{
	UDCCSR0 = (UDCCSR0 & UDCCSR0_WR_MASK) | flags;
	dev->req_pending = 0;
	DBG(DBG_VERY_NOISY, "%s %s, %02x/%02x\n",
	    __FUNCTION__, tag, UDCCSR0, flags);
}

static int write_ep0_fifo(struct pxa27x_ep *ep, struct pxa27x_request *req)
{
	unsigned count;
	int is_short = 0;

	count = write_packet(ep->reg_udcdr, ep->reg_pudcdr, req, EP0_FIFO_SIZE);
	ep->dev->stats.write.bytes += count;

	/* last packet "must be" short (or a zlp) */
	is_short = (count != EP0_FIFO_SIZE);

	DBG(DBG_VERY_NOISY, "ep0in %d bytes %d left %p\n", count,
	    req->req.length - req->req.actual, req);

	if (unlikely(is_short)) {
		if (ep->dev->req_pending)
			ep0start(ep->dev, UDCCSR0_IPR, "short IN");
		else
			UDCCSR0 = (UDCCSR0 & UDCCSR0_WR_MASK) | UDCCSR0_IPR;

		count = req->req.length;
		done(ep, req, 0);
		ep0_idle(ep->dev);
#if 0
		/* This seems to get rid of lost status irqs in some cases:
		 * host responds quickly, or next request involves config
		 * change automagic, or should have been hidden, or ...
		 *
		 * FIXME get rid of all udelays possible...
		 */
		if (count >= EP0_FIFO_SIZE) {
			count = 100;
			do {
				if ((UDCCSR0 & UDCCSR0_OPC) != 0) {
					/* clear OPC, generate ack */
					UDCCSR0 =
					    (UDCCSR0 & UDCCSR0_WR_MASK) |
					    UDCCSR0_OPC;
					break;
				}
				count--;
				udelay(1);
			} while (count);
		}
#endif
	} else if (ep->dev->req_pending)
		ep0start(ep->dev, 0, "IN");

	return is_short;
}

/*
 * read_fifo -  unload packet(s) from the fifo we use for usb OUT
 * transfers and put them into the request.  caller should have made
 * sure there's at least one packet ready.
 *
 * returns true if the request completed because of short packet or the
 * request buffer having filled (and maybe overran till end-of-packet).
 */
static int read_fifo(struct pxa27x_ep *ep, struct pxa27x_request *req)
{
	for (;;) {
		u32 udccsr;
		u32 *buf, word;
		u8 *cbuf;
		int bufferspace, count, is_short;

		udccsr = *ep->reg_udccsr;
		if (unlikely((udccsr & UDCCSR_PC) == 0))
			break;
		buf = req->req.buf + req->req.actual;
		prefetchw(buf);
		bufferspace = req->req.length - req->req.actual;

		/* read all bytes from this packet */
		if (likely(udccsr & UDCCSR_BNE)) {
			count = *ep->reg_udcbcr;
			req->req.actual += min(count, bufferspace);
		} else		/* zlp */
			count = 0;
		is_short = (count < ep->ep.maxpacket);
		DBG(DBG_NOISY,
		    "read %s, phys %d, %02x, %d bytes%s req %p %d/%d\n",
		    ep->ep.name, ep->phys, udccsr, count, is_short ? "/S" : "",
		    req, req->req.actual, req->req.length);
		while (likely(count >= 4)) {
			word = (u32) * ep->reg_udcdr;

			if (unlikely(bufferspace < 4)) {
				/* this happens when the driver's buffer
				 * is smaller than what the host sent.
				 * discard the extra data.
				 */
				if (req->req.status != -EOVERFLOW)
					pr_debug(PREFIX "%s overflow %d\n",
						 ep->ep.name, count);
				req->req.status = -EOVERFLOW;
			} else {
				*buf++ = word;
				bufferspace -= 4;
			}
			count -= 4;
		}
		word = (u32) * ep->reg_udcdr;
		cbuf = (u8 *)buf;
		while (unlikely(count > 0)) {
			if (unlikely(bufferspace <= 0)) {
				/* this happens when the driver's buffer
				 * is smaller than what the host sent.
				 * discard the extra data.
				 */
				if (req->req.status != -EOVERFLOW)
					pr_debug(PREFIX "%s overflow %d\n",
						 ep->ep.name, count);
				req->req.status = -EOVERFLOW;
			} else {
				*cbuf++ = word & 0x000000FF;
				word = word >> 8;
				bufferspace -= 1;
			}
			count -= 1;
		}

		*ep->reg_udccsr =
		    (*ep->reg_udccsr & UDCCSR_WR_MASK) | UDCCSR_PC;

		/* iso is one request per packet */
		if (ep->bmAttributes == USB_ENDPOINT_XFER_ISOC) {
			if (udccsr & UDCCSR_TRN)
				req->req.status = -EHOSTUNREACH;
			/* more like "is_done" */
			is_short = 1;
		}

		/* completion */
		if (is_short || req->req.actual == req->req.length) {
			done(ep, req, 0);
			if (list_empty(&ep->queue))
				pio_irq_disable(ep->phys,
						(UDC_INT_FIFOERROR |
						 UDC_INT_PACKETCMP));
			return 1;
		}

		/* finished that packet.  the next one may be waiting... */
	}

	*ep->reg_udccsr = (*ep->reg_udccsr & UDCCSR_WR_MASK) | UDCCSR_PC;
	return 0;
}

/*
 * special ep0 version of the above.  no UBCR0 or double buffering; status
 * handshaking is magic.  most device protocols don't need control-OUT.
 * CDC vendor commands (and RNDIS), mass storage CB/CBI, and some other
 * protocols do use them.
 */
static int read_ep0_fifo(struct pxa27x_ep *ep, struct pxa27x_request *req)
{

	u32 *buf, word;
	u8 *cbuf;
	int bufferspace;
	int count = 0;

	buf = (u32 *) (req->req.buf + req->req.actual);
	bufferspace = req->req.length - req->req.actual;

	/* read all bytes from this packet */
	if (likely(UDCCSR0 & UDCCSR0_RNE)) {
		count = UDCBCR0;
	} else		/* zlp */
		count = 0;

	DBG(DBG_NOISY,
	    "read %s, phys %d, %02x, %d bytes%s req %p %d/%d\n",
	    ep->ep.name, ep->phys, UDCCSR0, count, "",
	    req, req->req.actual, req->req.length);
	 
	while (likely(count >= 4)) {
		word = (u32) UDCDN(0);

		if (unlikely(bufferspace < 4)) {
			/* this happens when the driver's buffer
			 * is smaller than what the host sent.
			 * discard the extra data.
			 */
			if (req->req.status != -EOVERFLOW)
				pr_debug(PREFIX "%s overflow %d\n",
					 ep->ep.name, count);
			req->req.status = -EOVERFLOW;
		} else {
			*buf++ = word;
			req->req.actual += 4;
			bufferspace -= 4;
		}
		count -= 4;
	}
	word = (u32) UDCDN(0);
	cbuf = (u8 *)buf;
	while (unlikely(count > 0)) {
		if (unlikely(bufferspace <= 0)) {
			/* this happens when the driver's buffer
			 * is smaller than what the host sent.
			 * discard the extra data.
			 */
			if (req->req.status != -EOVERFLOW)
				pr_debug(PREFIX "%s overflow %d\n",
					 ep->ep.name, count);
			req->req.status = -EOVERFLOW;
		} else {
			*cbuf++ = word & 0x000000FF;
			word = word >> 8;
			req->req.actual += 1;
			bufferspace -= 1;
		}
		count -= 1;
	}

	UDCCSR0 = (UDCCSR0 & UDCCSR0_WR_MASK) | UDCCSR0_OPC | UDCCSR0_IPR;

	/* completion */
	if (req->req.actual >= req->req.length)
		return 1;

	/* finished that packet.  the next one may be waiting... */
	return 0;
}

#ifdef	CONFIG_USB_PXA27X_DMA

static void
start_dma_nodesc(struct pxa27x_ep *ep, struct pxa27x_request *req, int is_in)
{
	u32 dcmd = 0;
	u32 max = ep->fifo_size;
	u32 buf = req->req.dma;
	u32 fifo = PHYS_UDCDN(ep->phys);;

	buf += req->req.actual;
	ep->dma_fixup = 0;

	/* no-descriptor mode can be simple for bulk-in, iso-in, iso-out */
	DCSR(ep->dma) = DCSR_NODESC;
	if (is_in) {
		DSADR(ep->dma) = buf;
		DTADR(ep->dma) = fifo;

		dcmd = min(req->req.length - req->req.actual, max);
		dcmd &= DCMD_LENGTH;

		/*unuseful when DMA-alignment is off through DALGN */
		ep->dma_fixup = (dcmd < ep->ep.maxpacket);

		dcmd |= DCMD_BURST32 | DCMD_WIDTH4
		    | DCMD_FLOWTRG | DCMD_INCSRCADDR | DCMD_ENDIRQEN;
	} else {
		DSADR(ep->dma) = fifo;
		DTADR(ep->dma) = buf;

		dcmd = ep->ep.maxpacket;
		dcmd |= DCMD_BURST32 | DCMD_WIDTH4
		    | DCMD_FLOWSRC | DCMD_INCTRGADDR | DCMD_ENDIRQEN;
	}
	DCMD(ep->dma) = dcmd;
	DBG(DBG_VERBOSE, "DCMD %08X, DCSR %08X, DSADR %08X, DTADR %08X\n",
	    DCMD(ep->dma), DCSR(ep->dma), DSADR(ep->dma), DTADR(ep->dma));
	DCSR(ep->dma) = DCSR_RUN | DCSR_NODESC | DCSR_EORIRQEN;
}

static void kick_dma(struct pxa27x_ep *ep, struct pxa27x_request *req)
{
	int is_in = ep->bEndpointAddress & USB_DIR_IN;

	if (is_in) {
		if ((req->req.length - req->req.actual == 0) && req->req.zero) {
			DBG(DBG_VERBOSE, "%s: zlp sending...\n", __FUNCTION__);
			if ((*ep->reg_udccsr & UDCCSR_FS) != 0) {
				/*drop DME (implicitly) and set zlp */
				pio_irq_enable(ep->phys,
					       (UDC_INT_FIFOERROR |
						UDC_INT_PACKETCMP));
				*ep->reg_udccsr = UDCCSR_PC | UDCCSR_SP;;
			} else
				DBG(DBG_VERBOSE, "...oops! can't send zlp\n");

		} else {
			*ep->reg_udccsr = UDCCSR_DME | 
					  (*ep->reg_udccsr & UDCCSR_WR_MASK);
			start_dma_nodesc(ep, req, USB_DIR_IN);
		}
	} else {
		*ep->reg_udccsr = UDCCSR_DME | 
				  (*ep->reg_udccsr & UDCCSR_WR_MASK);
		start_dma_nodesc(ep, req, USB_DIR_OUT);
	}
}

static void cancel_dma(struct pxa27x_ep *ep)
{
	struct pxa27x_request *req;
	u32 tmp;

	if (DCSR(ep->dma) == 0 || list_empty(&ep->queue))
		return;

	DCSR(ep->dma) = 0;
	while ((DCSR(ep->dma) & DCSR_STOPSTATE) == 0)
		cpu_relax();

	req = list_entry(ep->queue.next, struct pxa27x_request, queue);
	tmp = DCMD(ep->dma) & DCMD_LENGTH;
	req->req.actual = req->req.length - (tmp & DCMD_LENGTH);

	/* the last tx packet may be incomplete, so flush the fifo.
	 * FIXME correct req.actual if we can
	 */
	if (ep->bEndpointAddress & USB_DIR_IN)
		*ep->reg_udccsr = UDCCSR_FEF;
}

static void dma_nodesc_handler(int dmach, void *_ep, struct pt_regs *r)
{
	struct pxa27x_ep *ep = _ep;
	struct pxa27x_request *req;
	u32 tmp = 0, completed = 0, len = 0;
	u32 dcsr = 0;
	unsigned long flags;

	local_irq_save_nort(flags);
	spin_lock_rt(&ep->dev->lock);

	DBG(DBG_VERBOSE, "%s, dma irq dcsr %08x\n", ep->ep.name, DCSR(ep->dma));

	req = list_entry(ep->queue.next, struct pxa27x_request, queue);

	ep->dma_irqs++;
	ep->dev->stats.irqs++;

	dcsr = DCSR(ep->dma);
	DCSR(ep->dma) &= ~DCSR_RUN;

	if ((dcsr & (DCSR_ENDINTR | DCSR_ENRINTR)) == 0) {
		DBG(DBG_VERBOSE, "%s, skip irq, dcsr %08x\n",
		    ep->ep.name, dcsr);
		goto done;
	}

	DCSR(ep->dma) |= DCSR_ENDINTR | DCSR_ENRINTR;

	/* update transfer status */
	if (dcsr & DCSR_BUSERR) {
		completed = 1;
		req->req.status = -EIO;
	} else {

		if (ep->bEndpointAddress & USB_DIR_IN) {

			tmp = DSADR(ep->dma);

			DBG(DBG_VERY_NOISY, "%s, dsadr %08x\n", ep->ep.name,
			    tmp);

			req->req.actual = tmp - req->req.dma;

			if (ep->dma_fixup)
				req->req.actual =
				    min(req->req.actual + 3, req->req.length);

			DBG(DBG_VERBOSE, "actual %d, length %d zero %d\n",
			    req->req.actual, req->req.length, req->req.zero);

			if ((req->req.length - req->req.actual) == 0) {
				completed = 1;
				/* maybe validate final short packet ... */
				if ((req->req.actual % ep->ep.maxpacket) !=
				    0 /*|| req->req.actual == 0 */ )
					*ep->reg_udccsr =
					    UDCCSR_SP | (*ep->
							 reg_udccsr &
							 UDCCSR_WR_MASK);

				/* ... or zlp, using pio fallback */
				else if (ep->bmAttributes ==
					 USB_ENDPOINT_XFER_BULK
					 && req->req.zero) {
					pr_debug(PREFIX
						 "%s zlp terminate ...\n",
						 ep->ep.name);
					completed = 0;
				}
			}

		} else {
			/*there processing of OUT endpoints */
			tmp = DTADR(ep->dma);

			DBG(DBG_VERY_NOISY, "%s, dtadr %08x\n", ep->ep.name,
			    tmp);
			if (*ep->reg_udccsr & UDCCSR_SP)
				len =
				    ep->ep.maxpacket -
				    (DCMD(ep->dma) & DCMD_LENGTH);
			else
				len = ep->ep.maxpacket;

			req->req.actual += len;

			if ((req->req.length - req->req.actual) == 0)
				completed = 1;

			if (len < ep->ep.maxpacket) {
				tmp = UDCCSR_PC;
				if (len == 0) {
					tmp |= UDCCSR_SP;
				}
				*ep->reg_udccsr =
				    tmp | (*ep->reg_udccsr & UDCCSR_WR_MASK);
				completed = 1;
			}
		}
	}

	if (likely(completed)) {
		done(ep, req, 0);

		/* maybe re-activate after completion */
		if (ep->stopped || list_empty(&ep->queue))
			goto done;
		req = list_entry(ep->queue.next, struct pxa27x_request, queue);
	}
	kick_dma(ep, req);
      done:
	local_irq_restore_nort(flags);
	spin_unlock_rt(&ep->dev->lock);
}

#endif

/*-------------------------------------------------------------------------*/

static int
pxa27x_ep_queue(struct usb_ep *_ep, struct usb_request *_req, int gfp_flags)
{
	struct pxa27x_request *req;
	struct pxa27x_ep *ep;
	struct pxa27x_udc *dev;
	unsigned long flags;

	req = container_of(_req, struct pxa27x_request, req);

	if (unlikely(!_req || !_req->complete || !_req->buf
		     || !list_empty(&req->queue))) {
		pr_debug(PREFIX "%s, bad params\n", __FUNCTION__);
		return -EINVAL;
	}

	ep = container_of(_ep, struct pxa27x_ep, ep);
	if (unlikely(!_ep || (!ep->desc && ep->ep.name != ep0name))) {
		pr_debug(PREFIX "%s, bad ep\n", __FUNCTION__);
		return -EINVAL;
	}

	dev = ep->dev;
	if (unlikely(!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)) {
		pr_debug(PREFIX "%s, bogus device state\n", __FUNCTION__);
		return -ESHUTDOWN;
	}

	/* iso is always one packet per request, that's the only way
	 * we can report per-packet status.  that also helps with dma.
	 */
	if (unlikely(ep->bmAttributes == USB_ENDPOINT_XFER_ISOC
		     && req->req.length > le16_to_cpu
		     (ep->desc->wMaxPacketSize)))
		return -EMSGSIZE;

#ifdef	CONFIG_USB_PXA27X_DMA
	if (use_dma && (ep->dma >= 0)) {
		if (req->req.dma == DMA_ADDR_INVALID) {
			req->req.dma = dma_map_single(dev->dev,
						      req->req.buf,
						      req->req.length,
						      (ep->
						       bEndpointAddress &
						       USB_DIR_IN)
						      ? DMA_TO_DEVICE :
						      DMA_FROM_DEVICE);

			if (dma_mapping_error(req->req.dma))
				pr_debug(PREFIX
					 "%s couldn't create dma mapping for request %p\n",
					 _ep->name, _req);
			else
				req->mapped = 1;

		} else {
			dma_sync_single_for_device(dev->dev,
						   req->req.dma,
						   req->req.length,
						   (ep->
						    bEndpointAddress &
						    USB_DIR_IN)
						   ? DMA_TO_DEVICE :
						   DMA_FROM_DEVICE);
			req->mapped = 0;
		}
	}
#endif

	DBG(DBG_VERBOSE, "%s queue req %p, len %d buf %p\n",
	    _ep->name, _req, _req->length, _req->buf);

	local_irq_save_nort(flags);
	spin_lock_rt(&dev->lock);

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	/* kickstart this i/o queue? */
	if (list_empty(&ep->queue) && !ep->stopped) {
		if (ep->desc == 0 /* ep0 */ ) {
			unsigned length = _req->length;

			switch (dev->ep0state) {
			case EP0_IN_DATA_PHASE:
				dev->stats.write.ops++;
				if (write_ep0_fifo(ep, req))
					req = 0;
				break;

			case EP0_OUT_DATA_PHASE:
				dev->stats.read.ops++;
				/* messy ... */
				if (dev->req_config) {
					DBG(DBG_VERBOSE, "ep0 config ack%s\n",
					    dev->has_cfr ? "" : " raced");
					if (dev->has_cfr)
						UDCCSR0 =
						    UDCCSR0_AREN | UDCCSR0_ACM;

					done(ep, req, 0);
					dev->ep0state = EP0_END_XFER;

					local_irq_restore_nort(flags);
				        spin_unlock_rt(&dev->lock);
					return 0;
				}
				if (dev->req_pending)
					ep0start(dev, UDCCSR0_IPR, "OUT");
				if (length == 0 || ((UDCCSR0 & UDCCSR0_RNE) != 0
						    && read_ep0_fifo(ep,
								     req))) {
					ep0_idle(dev);
					done(ep, req, 0);
					req = 0;
				}
				break;

			default:
				pr_debug(PREFIX "ep0 i/o, odd state %d\n",
					 dev->ep0state);

				local_irq_restore_nort(flags);
				spin_unlock_rt(&dev->lock);
				return -EL2HLT;
			}

#ifdef	CONFIG_USB_PXA27X_DMA
			/* either start dma or prime pio pump */
		} else if (ep->dma >= 0) {
			kick_dma(ep, req);
#endif
			/* can the FIFO can satisfy the request immediately? */
		} else if ((ep->bEndpointAddress & USB_DIR_IN) != 0
			   && (*ep->reg_udccsr & UDCCSR_FS) != 0
			   && write_fifo(ep, req)) {
			req = 0;
		} else if ((ep->bEndpointAddress & USB_DIR_IN) == 0
			   && (*ep->reg_udccsr & UDCCSR_FS) != 0
			   && read_fifo(ep, req)) {
			req = 0;
		}

		if (likely(req && ep->desc) && ep->dma < 0)
			pio_irq_enable(ep->phys,
				       (UDC_INT_FIFOERROR | UDC_INT_PACKETCMP));
	}

	/* pio or dma irq handler advances the queue. */
	if (likely(req != 0))
		list_add_tail(&req->queue, &ep->queue);

	local_irq_restore_nort(flags);
	spin_unlock_rt(&dev->lock);

	return 0;
}

/*
 * 	nuke - dequeue ALL requests
 */
static void nuke(struct pxa27x_ep *ep, int status)
{
	struct pxa27x_request *req;

	/* called with irqs blocked */
#ifdef	CONFIG_USB_PXA27X_DMA
	if (ep->dma >= 0 && !ep->stopped)
		cancel_dma(ep);
#endif
	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct pxa27x_request, queue);
		done(ep, req, status);
	}
	if (ep->desc)
		pio_irq_disable(ep->phys,
				(UDC_INT_FIFOERROR | UDC_INT_PACKETCMP));
}

/* dequeue JUST ONE request */
static int pxa27x_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct pxa27x_ep *ep;
	struct pxa27x_request *req;
	unsigned long flags;

	ep = container_of(_ep, struct pxa27x_ep, ep);
	if (!_ep || ep->ep.name == ep0name)
		return -EINVAL;

	local_irq_save_nort(flags);
	spin_lock_rt(&ep->dev->lock);

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req) {
		local_irq_restore_nort(flags);
		spin_unlock_rt(&ep->dev->lock);
		return -EINVAL;
	}
#ifdef	CONFIG_USB_PXA27X_DMA
	if (ep->dma >= 0 && ep->queue.next == &req->queue && !ep->stopped) {
		cancel_dma(ep);
		done(ep, req, -ECONNRESET);
		/* restart i/o */
		if (!list_empty(&ep->queue)) {
			req = list_entry(ep->queue.next,
					 struct pxa27x_request, queue);
			kick_dma(ep, req);
		}
	} else
#endif
		done(ep, req, -ECONNRESET);

	local_irq_restore_nort(flags);
	spin_unlock_rt(&ep->dev->lock);
	return 0;
}

/*-------------------------------------------------------------------------*/

static int pxa27x_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct pxa27x_ep *ep;
	unsigned long flags;

	ep = container_of(_ep, struct pxa27x_ep, ep);
	if (unlikely(!_ep || (!ep->desc && ep->ep.name != ep0name))
	    || ep->bmAttributes == USB_ENDPOINT_XFER_ISOC) {
		pr_debug(PREFIX "%s, bad ep\n", __FUNCTION__);
		return -EINVAL;
	}
	if (value == 0) {
		/* FIXME
		 * this path (reset toggle+halt) is needed to implement
		 * SET_INTERFACE on normal hardware.  but it can't be
		 * done from software on the PXA UDC, and the hardware
		 * forgets to do it as part of SET_INTERFACE automagic.
		 */
		pr_debug(PREFIX "only host can clear %s halt\n", _ep->name);
		return -EROFS;
	}
	local_irq_save_nort(flags);
	spin_lock_rt(&ep->dev->lock);

	if ((ep->bEndpointAddress & USB_DIR_IN) != 0
	    && ((*ep->reg_udccsr & UDCCSR_FS) == 0
		|| !list_empty(&ep->queue))) {
		local_irq_restore_nort(flags);
		spin_unlock_rt(&ep->dev->lock);
		return -EAGAIN;
	}

	/* FST bit is the same for control, bulk in, bulk out, interrupt in */
	*ep->reg_udccsr = UDCCSR_FST | UDCCSR_FEF;

	while(*ep->reg_udccsr & UDCCSR_FST) 
		yield();
	if ((*ep->reg_udccsr & UDCCSR_SST) == 0) 
		DBG(DBG_NORMAL, "%s: stall error!\n", __FUNCTION__);
	*ep->reg_udccsr = UDCCSR_SST | (*ep->reg_udccsr & UDCCSR_WR_MASK);

	/* FIXME ep0 needs special care ? */
	if (!ep->desc) {
		start_watchdog(ep->dev);
		ep->dev->req_pending = 0;
		ep->dev->ep0state = EP0_STALL;

		/* and bulk/intr endpoints like dropping stalls too */
	} /*
	    FIXME this is a workaround of PXA25x hardware glitch,
	    do we observe it on PXA27x ?
	    else {
		unsigned i;
		for (i = 0; i < 1000; i += 20) {
			if (*ep->reg_udccsr & UDCCSR_SST)
				break;
			udelay(20);
		}
	}*/
	
	local_irq_restore_nort(flags);
	spin_unlock_rt(&ep->dev->lock);

	DBG(DBG_VERBOSE, "%s halt\n", _ep->name);
	return 0;
}

static int pxa27x_ep_fifo_status(struct usb_ep *_ep)
{
	struct pxa27x_ep *ep;

	ep = container_of(_ep, struct pxa27x_ep, ep);
	if (!_ep) {
		pr_debug(PREFIX "%s, bad ep\n", __FUNCTION__);
		return -ENODEV;
	}

	/*FIXME  pxa can't report unclaimed bytes from IN fifos ? */
	if ((ep->bEndpointAddress & USB_DIR_IN) != 0)
		return -EOPNOTSUPP;
	if (ep->dev->gadget.speed == USB_SPEED_UNKNOWN
	    || (*ep->reg_udccsr & UDCCSR_FS) == 0)
		return 0;
	else
		return (*ep->reg_udcbcr & 0xfff) + 1;

}

static void pxa27x_ep_fifo_flush(struct usb_ep *_ep)
{
	struct pxa27x_ep *ep;

	ep = container_of(_ep, struct pxa27x_ep, ep);
	if (!_ep || ep->ep.name == ep0name || !list_empty(&ep->queue)) {
		pr_debug(PREFIX "%s, bad ep\n", __FUNCTION__);
		return;
	}

	/* toggle and halt bits stay unchanged */

	/* for OUT, just read and discard the FIFO contents. */
	if ((ep->bEndpointAddress & USB_DIR_IN) == 0) {
		while (((*ep->reg_udccsr) & UDCCSR0_RNE) != 0)
			(void)*ep->reg_udcdr;
		return;
	}

	/* most IN status is the same, but ISO can't stall */
	*ep->reg_udccsr = UDCCSR_PC | UDCCSR_FEF | UDCCSR_TRN
	    | (ep->bmAttributes == USB_ENDPOINT_XFER_ISOC)
	    ? 0 : UDCCSR_SST;

}

static struct usb_ep_ops pxa27x_ep_ops = {
	.enable = pxa27x_ep_enable,
	.disable = pxa27x_ep_disable,

	.alloc_request = pxa27x_ep_alloc_request,
	.free_request = pxa27x_ep_free_request,

	.alloc_buffer = pxa27x_ep_alloc_buffer,
	.free_buffer = pxa27x_ep_free_buffer,

	.queue = pxa27x_ep_queue,
	.dequeue = pxa27x_ep_dequeue,

	.set_halt = pxa27x_ep_set_halt,
	.fifo_status = pxa27x_ep_fifo_status,
	.fifo_flush = pxa27x_ep_fifo_flush,
};

/* ---------------------------------------------------------------------------
 * 	device-scoped parts of the api to the usb controller hardware
 * ---------------------------------------------------------------------------
 */

static int pxa27x_udc_get_frame(struct usb_gadget *_gadget)
{
	DBG(DBG_VERBOSE, "get frame\n");
	return (UDCFNR & 0x07ff);
}

static int pxa27x_udc_wakeup(struct usb_gadget *_gadget)
{

	DBG(DBG_VERBOSE, "wakeup\n");

	/* host may not have enabled remote wakeup */
	if ((UDCCR & UDCCR_DWRE) == 0)
		return -EHOSTUNREACH;

	UDCCR |= UDCCR_UDR;
	return 0;
}

static const struct usb_gadget_ops pxa27x_udc_ops = {
	.get_frame = pxa27x_udc_get_frame,
	.wakeup = pxa27x_udc_wakeup,
	/* current versions must always be self-powered */
};

/*-------------------------------------------------------------------------*/

#ifdef CONFIG_PROC_FS

static const char proc_node_name[] = "driver/udc";

static int
udc_proc_read(char *page, char **start, off_t off, int count,
	      int *eof, void *_dev)
{
	char *buf = page;
	struct pxa27x_udc *dev = _dev;
	char *next = buf;
	unsigned size = count;
	unsigned long flags;
	int i, t;
	u32 tmp;

	if (off != 0)
		return 0;

	local_irq_save_nort(flags);
	spin_lock_rt(&dev->lock);

	/* basic device status */
	t = scnprintf(next, size, DRIVER_DESC "\n"
		      "%s version: %s\nGadget driver: %s\nHost %s\n\n",
		      driver_name, DRIVER_VERSION DMASTR,
		      dev->driver ? dev->driver->driver.name : "(none)",
		      is_usb_connected()? "full speed" : "disconnected");
	size -= t;
	next += t;

	/* registers for device and ep0 */
	t = scnprintf(next, size,
		      "udcicr %02X.%02X, udcisr %02X.%02x\n",
		      UDCICR1, UDCICR0, UDCISR1, UDCISR0);
	size -= t;
	next += t;

	tmp = UDCCR;
	t = scnprintf(next, size,
		      "udccr %02X =%s%s%s%s%s%s%s%s%s%s\n", tmp,
		      (tmp & UDCCR_OEN) ? " oen" : "",
		      (tmp & UDCCR_AALTHNP) ? " aalthnp" : "",
		      (tmp & UDCCR_AHNP) ? " ahnp" : "",
		      (tmp & UDCCR_BHNP) ? " bhnp" : "",
		      (tmp & UDCCR_DWRE) ? " dwre" : "",
		      (tmp & UDCCR_SMAC) ? " smac" : "",
		      (tmp & UDCCR_EMCE) ? " emce" : "",
		      (tmp & UDCCR_UDR) ? " udr" : "",
		      (tmp & UDCCR_UDA) ? " uda" : "",
		      (tmp & UDCCR_UDE) ? " ude" : "");
	size -= t;
	next += t;

	tmp = UDCCSR0;
	t = scnprintf(next, size,
		      "udccs0 %02X =%s%s%s%s%s%s%s%s%s%s\n", tmp,
		      (tmp & (1 << 8)) ? " aren" : "",
		      (tmp & (1 << 9)) ? " acm" : "",
		      (tmp & UDCCSR0_SA) ? " sa" : "",
		      (tmp & UDCCSR0_RNE) ? " rne" : "",
		      (tmp & UDCCSR0_FST) ? " fst" : "",
		      (tmp & UDCCSR0_SST) ? " sst" : "",
		      (tmp & UDCCSR0_DME) ? " dme" : "",
		      (tmp & UDCCSR0_FTF) ? " ftf" : "",
		      (tmp & UDCCSR0_IPR) ? " ipr" : "",
		      (tmp & UDCCSR0_OPC) ? " opc" : "");
	size -= t;
	next += t;

	if (!is_usb_connected() || !dev->driver)
		goto done;

	t = scnprintf(next, size, "ep0 IN %lu/%lu, OUT %lu/%lu\nirqs %lu\n\n",
		      dev->stats.write.bytes, dev->stats.write.ops,
		      dev->stats.read.bytes, dev->stats.read.ops,
		      dev->stats.irqs);
	size -= t;
	next += t;

	/* dump endpoint queues */
	for (i = 0; i < num_use_endpoints; i++) {
		struct pxa27x_ep *ep = &dev->ep[i];
		struct pxa27x_request *req;
		int t;

		if (i != 0) {
			const struct usb_endpoint_descriptor *d;

			d = ep->desc;
			if (!d)
				continue;
			tmp = *dev->ep[i].reg_udccsr;
			t = scnprintf(next, size,
				      "%s max %d %s udccsr %02x irqs %lu/%lu\n",
				      ep->ep.name,
				      le16_to_cpu(d->wMaxPacketSize),
				      (ep->dma >= 0) ? "dma" : "pio", tmp,
				      ep->pio_irqs, ep->dma_irqs);

			/* TODO add udccr fo this ep */
			/* TODO translate all five groups of udccs bits! */

		} else		/* ep0 should only have one transfer queued */
			t = scnprintf(next, size, "ep0 max 16 pio irqs %lu\n",
				      ep->pio_irqs);
		if (t <= 0 || t > size)
			goto done;
		size -= t;
		next += t;

		if (list_empty(&ep->queue)) {
			t = scnprintf(next, size, "\t(nothing queued)\n");
			if (t <= 0 || t > size)
				goto done;
			size -= t;
			next += t;
			continue;
		}

		list_for_each_entry(req, &ep->queue, queue) {
#ifdef	CONFIG_USB_PXA27X_DMA
			if (ep->dma >= 0 && req->queue.prev == &ep->queue)
				t = scnprintf(next, size,
					      "\treq %p len %d/%d "
					      "buf %p (dma%d dcmd %08x)\n",
					      &req->req, req->req.actual,
					      req->req.length, req->req.buf,
					      ep->dma, DCMD(ep->dma)
				    );
			else
#endif
				t = scnprintf(next, size,
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
	local_irq_restore_nort(flags);
	spin_unlock_rt(&dev->lock);

	*eof = 1;
	return count - size;
}

#define create_proc_files() \
	create_proc_read_entry(proc_node_name, 0, NULL, udc_proc_read, dev)
#define remove_proc_files() \
	remove_proc_entry(proc_node_name, NULL)

#else				/* !CONFIG_PROC_FS */
#define create_proc_files() do {} while (0)
#define remove_proc_files() do {} while (0)

#endif				/* CONFIG_PROC_FS */

/* "function" sysfs attribute */
static ssize_t show_function(struct device *_dev, char *buf)
{
	struct pxa27x_udc *dev = dev_get_drvdata(_dev);

	if (!dev->driver
	    || !dev->driver->function
	    || strlen(dev->driver->function) > PAGE_SIZE)
		return 0;
	return scnprintf(buf, PAGE_SIZE, "%s\n", dev->driver->function);
}

static DEVICE_ATTR(function, S_IRUGO, show_function, NULL);

/*-------------------------------------------------------------------------*/

/*
 * 	udc_disable - disable USB device controller
 */
static void udc_disable(struct pxa27x_udc *dev)
{

	/* if hardware supports it, disconnect from usb */
	make_usb_disappear();

	UDCCR &= ~UDCCR_UDE;

	/* Disable clock for USB device */
	pxa_set_cken(CKEN11_USB, 0);

	ep0_idle(dev);
	dev->gadget.speed = USB_SPEED_UNKNOWN;

}

/*
 * 	udc_reinit - initialize software state
 */
static void udc_reinit(struct pxa27x_udc *dev)
{

	u32 i;

	/* device/ep0 records init */
	INIT_LIST_HEAD(&dev->gadget.ep_list);
	INIT_LIST_HEAD(&dev->gadget.ep0->ep_list);
	dev->ep0state = EP0_IDLE;

	/* basic endpoint records init */
	for (i = 0; i < num_use_endpoints; i++) {
		struct pxa27x_ep *ep = &dev->ep[i];
		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &dev->gadget.ep_list);

		ep->desc = 0;
		ep->stopped = 0;
		INIT_LIST_HEAD(&ep->queue);
		ep->pio_irqs = ep->dma_irqs = 0;
	}

	/* the rest was statically initialized, and is read-only */

}

static void udc_set_ep_list(unsigned config_num)
{
	struct pxa27x_udc *dev = the_controller;
	u32 i, bgn, end;

	if (unlikely(config_num > (PXA_UDC_NUM_CONFIGS - 1)))
		return;

	INIT_LIST_HEAD(&dev->gadget.ep_list);

	bgn = udc_configs[config_num].first;
	end = bgn + udc_configs[config_num].num;

	for (i = bgn; i < end; i++) {
		struct pxa27x_ep *ep = &dev->ep[i];
		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &dev->gadget.ep_list);
	}

	return;
}

/* until it's enabled, this UDC should be completely invisible
 * to any USB host.
 */
static void udc_enable(struct pxa27x_udc *dev)
{

	/* enable UDC clock */
	pxa_set_cken(CKEN11_USB, 1);

	ep0_idle(dev);
	dev->gadget.speed = USB_SPEED_UNKNOWN;
	dev->stats.irqs = 0;

	/* enable UDC controller */
	UDCCR = UDCCR_UDE;

	/* set ACK control mode */
	if (dev->has_cfr)
		UDCCSR0 = UDCCSR0_ACM;

	/* 
	 * If UDCCR_EMCE is set, we should reduce the number of the 
	 * configuration automatically or just yelled. 
	 */
	DBG(DBG_VERBOSE, "udc_enable: UDCCR %X\n", UDCCR);
	if ((UDCCR & UDCCR_EMCE) == UDCCR_EMCE) {
		pr_debug(PREFIX "%s, Endpoint memory configure error\n",
			 __FUNCTION__);
		UDCCR = UDCCR_EMCE;
	}

	/*
	 * udc_suspended_interrupts - enable suspended interrupts
	 */
	UDCICR1 = (UDCICR1 | UDCICR1_IERS | UDCICR1_IERU |
		   UDCICR1_IECC | UDCICR1_IESU) & ~UDCICR1_IESOF;

	/*enable ep0 interrupt */
	pio_irq_enable(0, (UDC_INT_FIFOERROR | UDC_INT_PACKETCMP));

	DBG(DBG_VERBOSE,
	    "udc_enable: UDCICR0 %X, UDCICR1 %X, UDCISR0 %X, UDCISR1 %X\n",
	    UDCICR0, UDCICR1, UDCISR0, UDCISR1);
	let_usb_appear();

}

/* when a driver is successfully registered, it will receive
 * control requests including set_configuration(), which enables
 * non-control requests.  then usb traffic follows until a
 * disconnect is reported.  then a host may connect again, or
 * the driver might get unbound.
 */
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct pxa27x_udc *dev = the_controller;
	int retval;

	if (!driver
	    || driver->speed != USB_SPEED_FULL
	    || !driver->bind
	    || !driver->unbind || !driver->disconnect || !driver->setup)
		return -EINVAL;
	if (!dev)
		return -ENODEV;
	if (dev->driver)
		return -EBUSY;

	/* first hook up the driver ... */
	dev->driver = driver;
	dev->gadget.dev.driver = &driver->driver;
	device_add(&dev->gadget.dev);
	retval = driver->bind(&dev->gadget);
	if (retval) {
		pr_debug(PREFIX "bind to driver %s --> error %d\n",
			 driver->driver.name, retval);
		device_del(&dev->gadget.dev);

		dev->driver = 0;
		dev->gadget.dev.driver = 0;
		return retval;
	}
	device_create_file(dev->dev, &dev_attr_function);

	/* ... then enable host detection and ep0; and we're ready
	 * for set_configuration as well as eventual disconnect.
	 * NOTE:  this shouldn't power up until later.
	 */
	pr_debug(PREFIX "registered gadget driver '%s'\n", driver->driver.name);
	udc_enable(dev);
	dump_state(dev);
	return 0;
}

EXPORT_SYMBOL(usb_gadget_register_driver);

static void
stop_activity(struct pxa27x_udc *dev, struct usb_gadget_driver *driver)
{
	int i;

	/* don't disconnect drivers more than once */
	if (dev->gadget.speed == USB_SPEED_UNKNOWN)
		driver = 0;
	dev->gadget.speed = USB_SPEED_UNKNOWN;

	/* prevent new request submissions, kill any outstanding requests  */
	for (i = 0; i < num_use_endpoints; i++) {
		struct pxa27x_ep *ep = &dev->ep[i];

		ep->stopped = 1;
		nuke(ep, -ESHUTDOWN);
	}
	del_timer_sync(&dev->timer);

	/* report disconnect; the driver is already quiesced */
	if (driver) {
		spin_unlock_rt(&dev->lock);
		driver->disconnect(&dev->gadget);
		spin_lock_rt(&dev->lock);
	}

	/* re-init driver-visible data structures */
	udc_reinit(dev);
}

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct pxa27x_udc *dev = the_controller;

	if (!dev)
		return -ENODEV;
	if (!driver || driver != dev->driver)
		return -EINVAL;
	local_irq_disable_nort();
	spin_lock_rt(&dev->lock);

	udc_disable(dev);
	stop_activity(dev, driver);

	local_irq_enable_nort();
	spin_unlock_rt(&dev->lock);

	driver->unbind(&dev->gadget);
	dev->driver = 0;

	device_del(&dev->gadget.dev);
	device_remove_file(dev->dev, &dev_attr_function);

	pr_debug(PREFIX "unregistered gadget driver '%s'\n",
		 driver->driver.name);
	dump_state(dev);
	return 0;
}

EXPORT_SYMBOL(usb_gadget_unregister_driver);

/*-------------------------------------------------------------------------*/

static inline void clear_ep_state(struct pxa27x_udc *dev)
{
	unsigned i;

	/* hardware SET_{CONFIGURATION,INTERFACE} automagic resets endpoint
	 * fifos, and pending transactions mustn't be continued in any case.
	 */
	for (i = 1; i < num_use_endpoints; i++)
		nuke(&dev->ep[i], -ECONNABORTED);
}

static void udc_watchdog(unsigned long _dev)
{
	struct pxa27x_udc *dev = (void *)_dev;
	unsigned long flags;

	local_irq_save_nort(flags);
	spin_lock_rt(&dev->lock);

	if (dev->ep0state == EP0_STALL
	    && (UDCCSR0 & UDCCSR0_FST) == 0 && (UDCCSR0 & UDCCSR0_SST) == 0) {
		UDCCSR0 =
		    (UDCCSR0 & UDCCSR0_WR_MASK) | UDCCSR0_FST | UDCCSR0_FTF;
		DBG(DBG_VERBOSE, "ep0 re-stall\n");
		start_watchdog(dev);
	}

	local_irq_restore_nort(flags);
	spin_unlock_rt(&dev->lock);
}

static void handle_ep0(struct pxa27x_udc *dev)
{
	u32 udccsr0 = UDCCSR0;
	struct pxa27x_ep *ep = &dev->ep[0];
	struct pxa27x_request *req;
	union {
		struct usb_ctrlrequest r;
		u8 raw[8];
		u32 word[2];
	} u;

	if (list_empty(&ep->queue))
		req = 0;
	else
		req = list_entry(ep->queue.next, struct pxa27x_request, queue);

	/* clear stall status */
	if (udccsr0 & UDCCSR0_SST) {
		nuke(ep, -EPIPE);
		UDCCSR0 = (UDCCSR0 & UDCCSR0_WR_MASK) | UDCCSR0_SST;
		del_timer(&dev->timer);
		ep0_idle(dev);
	}

	/* previous request unfinished?  non-error iff back-to-back ... */
	if ((udccsr0 & UDCCSR0_SA) != 0 && dev->ep0state != EP0_IDLE) {
		nuke(ep, 0);
		del_timer(&dev->timer);
		ep0_idle(dev);
	}

	switch (dev->ep0state) {
	case EP0_IDLE:
		/* late-breaking status? */
		udccsr0 = UDCCSR0;
		/* start control request? */
		if (likely((udccsr0 & (UDCCSR0_OPC | UDCCSR0_SA | UDCCSR0_RNE))
			   == (UDCCSR0_OPC | UDCCSR0_SA | UDCCSR0_RNE))) {
			int i;

			nuke(ep, -EPROTO);

			/* read SETUP packet */
			for (i = 0; i < 2; i++) {
				if (unlikely(!(UDCCSR0 & UDCCSR0_RNE))) {
				      bad_setup:
					pr_debug(PREFIX "SETUP %d!\n", i);
					goto stall;
				}
				u.word[i] = (u32) UDCDN(0);
			}
			if (unlikely((UDCCSR0 & UDCCSR0_RNE) != 0))
				goto bad_setup;

			UDCCSR0 = ((UDCCSR0 & UDCCSR0_WR_MASK) | UDCCSR0_SA | 
				    UDCCSR0_OPC);

			le16_to_cpus(&u.r.wValue);
			le16_to_cpus(&u.r.wIndex);
			le16_to_cpus(&u.r.wLength);

			DBG(DBG_VERBOSE, "SETUP %02x.%02x v%04x i%04x l%04x\n",
			    u.r.bRequestType, u.r.bRequest,
			    u.r.wValue, u.r.wIndex, u.r.wLength);

			/* cope with automagic for some standard requests. */
			dev->req_std = (u.r.bRequestType & USB_TYPE_MASK)
			    == USB_TYPE_STANDARD;
			dev->req_config = 0;
			dev->req_pending = 1;

			if (u.r.bRequestType & USB_DIR_IN)
				dev->ep0state = EP0_IN_DATA_PHASE;
			else
				dev->ep0state = EP0_OUT_DATA_PHASE;

			spin_unlock_rt(&dev->lock);
			i = dev->driver->setup(&dev->gadget, &u.r);
			spin_lock_rt(&dev->lock);

			if (i < 0) {
				/* hardware automagic preventing STALL... */
				if (dev->req_config) {
					/* hardware sometimes neglects to tell
					 * tell us about config change events,
					 * so later ones may fail...
					 */
					WARN("config change %02x fail %d?\n",
					     u.r.bRequest, i);
					return;
					/* TODO experiment:  if has_cfr,
					 * hardware didn't ACK; maybe we
					 * could actually STALL!
					 */
				}
				DBG(DBG_VERBOSE, "protocol STALL, "
				    "%02x err %d\n", UDCCSR0, i);
			      stall:
				/* the watchdog timer helps deal with cases
				 * where udc seems to clear FST wrongly, and
				 * then NAKs instead of STALLing.
				 */
				ep0start(dev, UDCCSR0_FST | UDCCSR0_FTF,
					 "stall");
				start_watchdog(dev);
				dev->ep0state = EP0_STALL;

				/* deferred i/o == no response yet */
			} else if (dev->req_pending) {
				if (likely(dev->ep0state == EP0_IN_DATA_PHASE
					   || dev->req_std || u.r.wLength))
					ep0start(dev, 0, "defer");
				else
					ep0start(dev, UDCCSR0_IPR, "defer/IPR");
			}

			/* expect at least one data or status stage irq */
			return;

		} else {
			/* some random early IRQ:
			 * - we acked FST
			 * - IPR cleared
			 * - OPR got set, without SA (likely status stage)
			 */
			UDCCSR0 =
			    (UDCCSR0 & UDCCSR0_WR_MASK) | (udccsr0 &
							   (UDCCSR0_SA |
							    UDCCSR0_OPC));
		}
		break;
	case EP0_IN_DATA_PHASE:	/* GET_DESCRIPTOR etc */
		if (udccsr0 & UDCCSR0_OPC) {
			UDCCSR0 =
			    (UDCCSR0 & UDCCSR0_WR_MASK) | UDCCSR0_OPC |
			    UDCCSR0_FTF;
			DBG(DBG_VERBOSE, "ep0in premature status\n");
			if (req)
				done(ep, req, 0);
			ep0_idle(dev);
		} else {	/* irq was IPR clearing */

			if (req) {
				/* this IN packet might finish the request */
				(void)write_ep0_fifo(ep, req);
			}	/* else IN token before response was written */
		}
		break;
	case EP0_OUT_DATA_PHASE:	/* SET_DESCRIPTOR etc */
		if (udccsr0 & UDCCSR0_OPC) {
			if (req) {
				/* this OUT packet might finish the request */
				if (read_ep0_fifo(ep, req))
					done(ep, req, 0);
				/* else more OUT packets expected */
			}	/* else OUT token before read was issued */
		} else {	/* irq was IPR clearing */

			DBG(DBG_VERBOSE, "ep0out premature status\n");
			if (req)
				done(ep, req, 0);
			ep0_idle(dev);
		}
		break;
	case EP0_END_XFER:
		if (req)
			done(ep, req, 0);
		/* ack control-IN status (maybe in-zlp was skipped)
		 * also appears after some config change events.
		 */
		if (udccsr0 & UDCCSR0_OPC)
			UDCCSR0 = (UDCCSR0 & UDCCSR0_WR_MASK) | UDCCSR0_OPC;
		ep0_idle(dev);
		break;
	case EP0_STALL:
		UDCCSR0 = (UDCCSR0 & UDCCSR0_WR_MASK) | UDCCSR0_FST;
		break;
	}
}

static void handle_set_config(struct pxa27x_udc *dev)
{
	struct usb_ctrlrequest r;
	u8 cfg = 0, ifc = 0, alt = 0;
	int i;

	cfg = (UDCCR & UDCCR_ACN) >> UDCCR_ACN_S;
	ifc = (UDCCR & UDCCR_AIN) >> UDCCR_AIN_S;
	alt = (UDCCR & UDCCR_AAISN) >> UDCCR_AAISN_S;

	DBG(DBG_VERBOSE, "%s: cfg %x, ifc %x, alt %x\n", __FUNCTION__, cfg, ifc,
	    alt);

	if (dev->cfg_done == 0) {
		/*first config after reset */
		dev->cfg_done = 1;
		r.bRequestType = USB_RECIP_DEVICE;
	} else if (dev->num_cfg != cfg) {
		/*config changed by host */
		r.bRequestType = USB_RECIP_DEVICE;
	} else {
		/*assume set interface */
		r.bRequestType = USB_RECIP_INTERFACE;
	}

	if (r.bRequestType == USB_RECIP_DEVICE) {
		r.bRequest = USB_REQ_SET_CONFIGURATION;
		r.wValue = cfg;
		r.wIndex = 0;
		r.wLength = 0;

		/*update gadget's ep list */
		udc_set_ep_list(cfg);

	} else {
		r.bRequest = USB_REQ_SET_INTERFACE;
		r.wValue = alt;
		r.wIndex = ifc;
		r.wLength = 0;
	}

	/*is it really needed? */
	the_controller->num_cfg = cfg;
	the_controller->num_ifc = ifc;
	the_controller->num_aifc = alt;

	DBG(DBG_VERBOSE, "SETUP %02x.%02x v%04x i%04x l%04x\n",
	    r.bRequestType, r.bRequest, r.wValue, r.wIndex, r.wLength);

	dev->req_config = 1;
	clear_ep_state(dev);
	dev->ep0state = EP0_OUT_DATA_PHASE;

	spin_unlock_rt(&dev->lock);
	i = dev->driver->setup(&dev->gadget, &r);
	spin_lock_rt(&dev->lock);

	if (i < 0) {
		WARN("config change %02x fail %d?\n", r.bRequest, i);
		return;
	} else {
		/*seems we don't need to do something else here */
		;
	}
}

static void handle_ep(struct pxa27x_ep *ep)
{
	struct pxa27x_request *req;
	int is_in = ep->bEndpointAddress & USB_DIR_IN;
	int completed;
	u32 udccsr, tmp;

#ifdef CONFIG_USB_PXA27X_DMA
	if (use_dma && (ep->dma >= 0)) {
		pio_irq_disable(ep->phys,
				(UDC_INT_FIFOERROR | UDC_INT_PACKETCMP));

		if (likely(!list_empty(&ep->queue))) {

			req =
			    list_entry(ep->queue.next, struct pxa27x_request,
				       queue);
			/*check, is it really that case we wait */
			if ((req->req.length - req->req.actual == 0)
			    && req->req.zero) {
				done(ep, req, 0);

				if (unlikely(!list_empty(&ep->queue))) {
					req =
					    list_entry(ep->queue.next,
						       struct pxa27x_request,
						       queue);
					kick_dma(ep, req);
				}
			}
		}

		return;
	}
#endif

	do {
		completed = 0;
		if (likely(!list_empty(&ep->queue)))
			req = list_entry(ep->queue.next,
					 struct pxa27x_request, queue);
		else
			req = 0;

		/* TODO check FST handling */

		udccsr = *ep->reg_udccsr;
		if (unlikely(is_in)) {	/* irq from PC, SST ... */
			DBG(DBG_VERY_NOISY, "IN packet\n");
			tmp = UDCCSR_TRN;
			if (likely(ep->bmAttributes == USB_ENDPOINT_XFER_BULK))
				tmp |= UDCCSR_SST;
			tmp &= udccsr;
			if (likely(tmp))
				*ep->reg_udccsr = tmp;
			if (req && likely((udccsr & UDCCSR_FS) != 0))
				completed = write_fifo(ep, req);

		} else {	/* irq from PC ... */
			DBG(DBG_VERY_NOISY, "OUT packet\n");
			if (likely(ep->bmAttributes == USB_ENDPOINT_XFER_BULK))
				tmp = UDCCSR_SST | UDCCSR_DME;
			else
				tmp = UDCCSR_TRN | UDCCSR_DME;
			tmp &= udccsr;
			if (likely(tmp))
				*ep->reg_udccsr = tmp;

			/* fifos can hold packets, ready for reading... */
			if (likely(req))
				completed = read_fifo(ep, req);
			else
				pio_irq_disable(ep->phys,
						(UDC_INT_FIFOERROR |
						 UDC_INT_PACKETCMP));
		}
		ep->pio_irqs++;
	} while (completed);
}

/*
 *	pxa27x_udc_irq - interrupt handler
 *
 * avoid delays in ep0 processing. the control handshaking isn't always
 * under software control, and delays could cause usb protocol errors.
 */
static irqreturn_t pxa27x_udc_irq(int irq, void *_dev, struct pt_regs *r)
{
	struct pxa27x_udc *dev = _dev;
	int handled = 0;
	u32 udcisr1 = UDCISR1;
	u32 udcisr0 = UDCISR0;
	int ep_phys;

	spin_lock_rt(&dev->lock);

	dev->stats.irqs++;

	/* For Endpoint0, Endpoint A-P */
	for (ep_phys = 0; udcisr0 != 0 && ep_phys < 16; 
	     udcisr0 >>= 2, ep_phys++) {
		if (udcisr0 & UDC_INT_PACKETCMP) {
			if (ep_phys == 0) {
				/* control traffic */
				dev->ep[0].pio_irqs++;
				pio_reset_irq_status(0);
				handle_ep0(dev);
			} else {
				/* endpoint data transfers */
				pio_reset_irq_status(ep_phys);
				handle_ep(&dev->ep[ep_phys]);
			}

			handled = 1;
		}

		if (udcisr0 & UDC_INT_FIFOERROR) {
			DBG(DBG_NORMAL, "fifo error on ep %d\n", ep_phys);
			handled = 1;
		}
	}

	/*Change configuration Request */
	if (udcisr1 & UDCISR1_IECC) {
		UDCCR |= UDCCR_SMAC;

		handle_set_config(dev);

		UDCISR1 = UDCISR1_IECC;
		handled = 1;
	}

	/* SUSpend Interrupt Request */
	if (udcisr1 & UDCISR1_IESU) {
		UDCISR1 = UDCISR1_IESU;
		handled = 1;
		DBG(DBG_VERBOSE, "USB suspend%s\n", is_usb_connected()
		    ? "" : "+disconnect");

		if (!is_usb_connected())
			stop_activity(dev, dev->driver);
		else if (dev->gadget.speed != USB_SPEED_UNKNOWN
			 && dev->driver && dev->driver->suspend) {
				spin_unlock_rt(&dev->lock);
				dev->driver->suspend(&dev->gadget);
				spin_lock_rt(&dev->lock);
		}
		ep0_idle(dev);
	}

	/* RESume Interrupt Request */
	if (udcisr1 & UDCISR1_IERU) {
		UDCISR1 = UDCISR1_IERU;
		handled = 1;
		DBG(DBG_VERBOSE, "USB resume\n");

		if (dev->gadget.speed != USB_SPEED_UNKNOWN
		    && dev->driver
		    && dev->driver->resume && is_usb_connected()) {
			spin_unlock_rt(&dev->lock);
			dev->driver->resume(&dev->gadget);
			spin_lock_rt(&dev->lock);
		}
	}

	/* ReSeT Interrupt Request - USB reset */
	if (udcisr1 & UDCISR1_IERS) {
		UDCISR1 = UDCISR1_IERS;
		handled = 1;
		INFO("USB reset\n");

		/*after reset we need to set configuration again */
		dev->cfg_done = 0;
		dev->gadget.speed = USB_SPEED_FULL;
		memset(&dev->stats, 0, sizeof dev->stats);
	}

	spin_unlock_rt(&dev->lock);
	return IRQ_HANDLED;
}

/*-------------------------------------------------------------------------*/

static void nop_release(struct device *dev)
{
	pr_debug(PREFIX "%s %s\n", __FUNCTION__, dev->bus_id);
}

static int pxa27x_ep_init(char *name, int ep_phys, u8 addr, u8 type,
			  unsigned maxp)
{
	struct pxa27x_udc *dev = the_controller;
	struct pxa27x_ep *ep;

	/* FIXME!!! Add some sanity checks */

	ep = &dev->ep[ep_phys];

	ep->phys = ep_phys;
	ep->dev = dev;
	ep->ep.name = name;
	ep->ep.ops = &pxa27x_ep_ops;
	ep->dma = -1;

	ep->reg_udccsr = &UDCCSN(ep_phys);
	ep->reg_udcdr = &UDCDN(ep_phys);
	ep->reg_pudcdr = (volatile u8 *)PUDCDN(ep_phys);
	ep->ep.maxpacket = ep->fifo_size = maxp;

	if (ep_phys == 0)
		return 0;

	ep->bEndpointAddress = addr;
	ep->bmAttributes = type;

	if (!(addr & USB_DIR_IN))
		ep->reg_udcbcr = &UDCBCN(ep_phys);

#ifdef	CONFIG_USB_PXA27X_DMA
	ep->reg_drcmr = &DRCMR(DRCMR_EP0 + ep_phys);
#endif
	return 0;
}

static int pxa27x_ep_config(int ep_phys, int configuration, int interface,
			    int alternate)
{
	struct pxa27x_udc *dev = the_controller;
	struct pxa27x_ep *ep;
	u32 config_reg = 0;

	ep = &dev->ep[ep_phys];

	if (ep->dev == NULL) {
		pr_debug(PREFIX "%s, bad ep or descriptor\n", __FUNCTION__);
		return -1;

	}

	config_reg = ((configuration << UDCCONR_CN_S) & UDCCONR_CN) |
	    ((interface << UDCCONR_IN_S) & UDCCONR_IN) |
	    ((alternate << UDCCONR_AISN_S) & UDCCONR_AISN) |
	    ((ep->bEndpointAddress << UDCCONR_EN_S) & UDCCONR_EN);

	/* Double Buffering Enabled and Enable the EP */
	config_reg |= UDCCONR_EE | UDCCONR_DE;

	if (ep->bEndpointAddress & USB_DIR_IN) {
		config_reg |= UDCCONR_ED;	/* Direction: IN */
	} else {
		config_reg &= ~UDCCONR_ED;	/* Direction: OUT */
	}

	config_reg |= ((ep->bmAttributes << UDCCONR_ET_S) & UDCCONR_ET) |
	    ((ep->fifo_size << UDCCONR_MPS_S) & UDCCONR_MPS);

	UDCCN(ep_phys) = config_reg;

	DBG(DBG_VERBOSE, "%s: ep(%d) %X\n", __FUNCTION__, ep_phys, config_reg);

	return 0;
}

static void udc_ep_setup(void)
{
	int ep_phys;

	pxa27x_ep_init(ep0name, 0, 0, 0, EP0_FIFO_SIZE);

	/*drop all endpoints */
	for (ep_phys = 1; ep_phys < 24; ep_phys++)
		UDCCN(ep_phys) = 0;

	/*
	 * configuration 1 (file_storage, serial non-ACM) 
	 */
	pxa27x_ep_init("ep1in-int", 1, USB_DIR_IN | 1,
		       USB_ENDPOINT_XFER_INT, INT_FIFO_SIZE);
	pxa27x_ep_config(1, 1, 0, 0);
	pxa27x_ep_init("ep2in-bulk", 2, USB_DIR_IN | 2,
		       USB_ENDPOINT_XFER_BULK, BULK_FIFO_SIZE);
	pxa27x_ep_config(2, 1, 0, 0);
	pxa27x_ep_init("ep3out-bulk", 3, 3, USB_ENDPOINT_XFER_BULK,
		       BULK_FIFO_SIZE);
	pxa27x_ep_config(3, 1, 0, 0);
	
	udc_configs[1].first = 1;
	udc_configs[1].num = 3;
	
	/* 
	 * configuration 2 (RNDIS Ethernet, serial CDC ACM) 
	 * NOTE: it seems windows host never issues 
	 * the SET_INTERFACE request for the above devices 
	 * that means the data interface (ep2 & ep3)
	 * won't be ever enabled if we programm it to where it 
	 * originally should be. So the data interface  
	 * is programmed to the same interface as control (0)
        *
	 * NOTE2: config 2 is also gadget zero loopback, but pxa27x
	 * does not support zero.
	 */
	pxa27x_ep_init("ep1in-int", 4, USB_DIR_IN | 1,
		       USB_ENDPOINT_XFER_INT, INT_FIFO_SIZE);
	pxa27x_ep_config(4, 2, 0, 0);
	
	pxa27x_ep_init("ep2in-bulk", 5, USB_DIR_IN | 2,
		       USB_ENDPOINT_XFER_BULK, BULK_FIFO_SIZE);
	pxa27x_ep_config(5, 2, 0, 0);	

	pxa27x_ep_init("ep3out-bulk", 6, 3, USB_ENDPOINT_XFER_BULK,
		       BULK_FIFO_SIZE);
	pxa27x_ep_config(6, 2, 0, 0);	

	udc_configs[2].first = 4;
	udc_configs[2].num = 3;
	
	/* 
	 * configuration 3 (CDC Ethernet) 
	 * NOTE: config 3 is also gadget zero src/sink, but pxa27x
	 * does not support zero.
	 */
	pxa27x_ep_init("ep1in-int", 7, USB_DIR_IN | 1,
		       USB_ENDPOINT_XFER_INT, INT_FIFO_SIZE);
	pxa27x_ep_config(7, 3, 0, 0);
	
	pxa27x_ep_init("ep2in-bulk", 8, USB_DIR_IN | 2,
		       USB_ENDPOINT_XFER_BULK, BULK_FIFO_SIZE);
	pxa27x_ep_config(8, 3, 1, 1);
	
	pxa27x_ep_init("ep3out-bulk", 9, 3, USB_ENDPOINT_XFER_BULK,
		       BULK_FIFO_SIZE);
	pxa27x_ep_config(9, 3, 1, 1);
	
	udc_configs[3].first = 7;
	udc_configs[3].num = 3;
	
	num_use_endpoints = 10;
	
}

/*
 * 	probe - binds to the platform device
 */
static int __init pxa27x_udc_probe(struct device *_dev)
{
	struct pxa27x_udc *dev;
	int retval, err;

	dev = kmalloc(sizeof(struct pxa27x_udc), SLAB_KERNEL);
	if (!dev)
		return -ENOMEM;

	memset(dev, 0, sizeof(struct pxa27x_udc));

#ifdef CONFIG_PREEMPT_RT
	spin_lock_init (&dev->lock);
#endif
	dev->has_cfr = 1;

	dev->cfg_done = 0;
	dev->num_cfg = 0;
	dev->num_ifc = 0;
	dev->num_aifc = 0;

	dev->gadget.ops = &pxa27x_udc_ops;
	dev->gadget.ep0 = &dev->ep[0].ep;
	dev->gadget.name = driver_name;
	strcpy(dev->gadget.dev.bus_id, "gadget");
	dev->gadget.dev.release = nop_release;

	init_timer(&dev->timer);
	dev->timer.function = udc_watchdog;
	dev->timer.data = (unsigned long)dev;

	device_initialize(&dev->gadget.dev);

	the_controller = dev;

	dev->dev = _dev;
	dev->mach = _dev->platform_data;

	dev->gadget.dev.parent = _dev;
	dev->gadget.dev.dma_mask = _dev->dma_mask;

	dev_set_drvdata(_dev, dev);

	udc_ep_setup();
	udc_disable(dev);
	udc_reinit(dev);

	/* irq setup after old hardware state is cleaned up */
	retval = request_irq(IRQ_USB, pxa27x_udc_irq,
#ifndef CONFIG_PREEMPT_RT
			     SA_INTERRUPT, 
#else
			     0,
#endif
			     driver_name, dev);
	if (retval != 0) {
		printk(KERN_ERR "%s: can't get irq %i, err %d\n",
		       driver_name, IRQ_USB, retval);
		err = -EBUSY;
		goto cleanup;
	}
	dev->got_irq = 1;

	create_proc_files();

	return 0;

      cleanup:
	kfree(dev);

	return err;
}

static int __exit pxa27x_udc_remove(struct device *_dev)
{
	struct pxa27x_udc *dev = _dev->driver_data;

	udc_disable(dev);
	remove_proc_files();
	usb_gadget_unregister_driver(dev->driver);

	if (dev->got_irq) {
		free_irq(IRQ_USB, dev);
		dev->got_irq = 0;
	}

	dev_set_drvdata(_dev, 0);
	kfree(the_controller);
	the_controller = 0;
	return 0;
}

/*-------------------------------------------------------------------------*/

static struct device_driver udc_driver = {
	.name = "pxa2xx-udc",
	.bus = &platform_bus_type,
	.probe = pxa27x_udc_probe,
	.remove = __exit_p(pxa27x_udc_remove),

	/* FIXME power management support */
	/* .suspend = ... disable UDC */
	/* .resume = ... re-enable UDC */
};

static int __init udc_init(void)
{
	printk(KERN_INFO "%s: version %s\n", driver_name, DRIVER_VERSION);
	return driver_register(&udc_driver);
}

module_init(udc_init);

static void __exit udc_exit(void)
{
	driver_unregister(&udc_driver);
}

module_exit(udc_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Frank Becker, Robert Schwebel, David Brownell");
MODULE_LICENSE("GPL");
