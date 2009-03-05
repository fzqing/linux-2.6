/*
 * linux/drivers/usb/gadget/pxa3xx_udc.c
 * Marvell PXA3xx on-chip full speed USB device controllers
 *
 * Copyright (C) 2002 Intrinsyc, Inc. (Frank Becker)
 * Copyright (C) 2003 Robert Schwebel, Pengutronix
 * Copyright (C) 2003 Benedikt Spranger, Pengutronix
 * Copyright (C) 2003 David Brownell
 * Copyright (C) 2003 Joshua Wise
 * Copyright (C) 2004 Intel Corporation
 * Copyright (C) 2007 Marvell International Ltd.
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

#undef DEBUG
//#define      VERBOSE DBG_NOISY

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
#include <asm/uaccess.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/mhn_pmic.h>

#include <linux/usb.h>
#include <linux/usb_ch9.h>
#include <linux/usb_gadget.h>
#include <linux/usb_otg.h>

#include <asm/arch/udc.h>
#include "../otg/pxa3xx_otg.h"

/*
 * This driver handles the USB Device Controller (UDC) in Intel's PXA 3xx
 * series processors.
 * Such controller drivers work with a gadget driver.  The gadget driver
 * returns descriptors, implements configuration and data protocols used
 * by the host to interact with this device, and allocates endpoints to
 * the different protocol interfaces.  The controller driver virtualizes
 * usb hardware so that the gadget drivers will be more portable.
 *
 * This UDC hardware wants to implement a bit too much USB protocol, so
 * it constrains the sorts of USB configuration change events that work.
 * The errata for these chips are misleading; some "fixed" bugs from
 * pxa250 a0/a1 b0/b1/b2 sure act like they're still there.
 */

#ifdef CONFIG_USB_COMPOSITE
#include <linux/kernel.h>
#endif

#define	DRIVER_VERSION	"01-Jul-2005"
#define	DRIVER_DESC	"PXA 3xx USB Device Controller driver"

static const char driver_name[] = "pxa3xx_udc";

static const char ep0name[] = "ep0";

#ifdef CONFIG_PROC_FS
#define	UDC_PROC_FILE
#endif

#include "pxa3xx_udc.h"

#ifdef CONFIG_EMBEDDED
/* few strings, and little code to use them */
#undef	DEBUG
#undef	UDC_PROC_FILE
#endif

/* If we can detect the cable attach and detach */
#if defined(CONFIG_PXA3XX_ARAVA)
#define ENABLE_CABLE_DETECT
static int connected = 0;
static int d0cs = 0;
static int out_d0cs = 0;
#endif
static int is_below_624(struct mhn_fv_info *info, int attached);

static int is_cable_attached(void);

#ifdef	CONFIG_USB_PXA3XX_DMA
static int use_dma = 1;
module_param(use_dma, bool, 0);
MODULE_PARM_DESC(use_dma, "true to use dma");

static void dma_nodesc_handler(int dmach, void *_ep, struct pt_regs *r);
static void kick_dma(struct pxa3xx_ep *ep, struct pxa3xx_request *req);

#define	DMASTR " (dma support)"

#else				/* !CONFIG_USB_PXA3XX_DMA */
#define	DMASTR " (pio only)"
#endif

#define SIZE_STR	""

#define UDCISR0_IR0	 0x3
#define UDCISR_INT_MASK	 (UDC_INT_FIFOERROR | UDC_INT_PACKETCMP)
#define UDCICR_INT_MASK	 UDCISR_INT_MASK

#define UDCCSR_MASK	(UDCCSR_FST | UDCCSR_DME)

#ifdef CONFIG_USB_COMPOSITE
static int num_ep_used = 1;
#endif

static int ipm_notify(void)
{
	return 0;
}

#ifdef CONFIG_USB_OTG
extern int mhnotg_otg_interrupt(struct otg_transceiver *otg);
extern int mhnotg_host_suspend(struct otg_transceiver *otg);
static unsigned udcicr0, udcicr1;
static int irq_saved = 0;
void pxa3xx_udc_save_irq(void)
{
	unsigned long flags;

	local_irq_save(flags);
	if (!irq_saved) {
		UDCISR0 = UDCISR1 = 0xFFFFFFFF;
		udcicr0 = UDCICR0;
		udcicr1 = UDCICR1;
		UDCICR0 = UDCICR1 = 0;
		irq_saved = 1;
	}
	local_irq_restore(flags);
}

void pxa3xx_udc_restore_irq(void)
{
	unsigned long flags;

	local_irq_save(flags);
	if (irq_saved) {
		UDCICR0 = udcicr0;
		UDCICR1 = udcicr1;
		irq_saved = 0;
	}
	local_irq_restore(flags);
}

extern void mhnotg_require_bus(int require);
extern char *mhnotg_state(struct otg_transceiver *otg);
#endif

void dump_buffer(char *buf, unsigned length)
{
	char *c = buf;
	int i;

	printk(KERN_DEBUG "%s, buffer total length = %d\n", __func__, length);
	for (i = 0; i < length; i++) {
		if (0 == i % 10)
			printk("\n");
		printk(KERN_DEBUG " 0x%x", c[i]);
	}
	printk(KERN_DEBUG "\n");
}

/* ---------------------------------------------------------------------------
 * 	endpoint related parts of the api to the usb controller hardware,
 *	used by gadget driver; and the inner talker-to-hardware core.
 * ---------------------------------------------------------------------------
 */

static void pxa3xx_ep_fifo_flush(struct usb_ep *ep);
static void nuke(struct pxa3xx_ep *, int status);

static void pio_irq_enable(int ep_num)
{
	if (ep_num < 16)
		UDCICR0 |= 3 << (ep_num * 2);
	else {
		ep_num -= 16;
		UDCICR1 |= 3 << (ep_num * 2);
	}
}

static void pio_irq_disable(int ep_num)
{
	ep_num &= 0xf;
	if (ep_num < 16)
		UDCICR0 &= ~(3 << (ep_num * 2));
	else {
		ep_num -= 16;
		UDCICR1 &= ~(3 << (ep_num * 2));
	}
}

/*
 * endpoint enable/disable
 *
 * we need to verify the descriptors used to enable endpoints.  since pxa3xx
 * endpoint configurations are fixed, and are pretty much always enabled,
 * there's not a lot to manage here.
 *
 * because pxa3xx can't selectively initialize bulk (or interrupt) endpoints,
 * (resetting endpoint halt and toggle), SET_INTERFACE is unusable except
 * for a single interface (with only the default altsetting) and for gadget
 * drivers that don't halt endpoints (not reset by set_interface).  that also
 * means that if you use ISO, you must violate the USB spec rule that all
 * iso endpoints must be in non-default altsettings.
 */
static int pxa3xx_ep_enable(struct usb_ep *_ep,
			    const struct usb_endpoint_descriptor *desc)
{
	struct pxa3xx_ep *ep;
	struct pxa3xx_udc *dev;

	ep = container_of(_ep, struct pxa3xx_ep, ep);
	if (!_ep || !desc || _ep->name == ep0name
	    || desc->bDescriptorType != USB_DT_ENDPOINT
	    || ep->fifo_size < le16_to_cpu(desc->wMaxPacketSize)) {
		DMSG("%s, bad ep or descriptor\n", __FUNCTION__);
		return -EINVAL;
	}

	/* xfer types must match, except that interrupt ~= bulk */
	if (ep->ep_type != USB_ENDPOINT_XFER_BULK
	    && desc->bmAttributes != USB_ENDPOINT_XFER_INT) {
		DMSG("%s, %s type mismatch\n", __FUNCTION__, _ep->name);
		return -EINVAL;
	}

	/* hardware _could_ do smaller, but driver doesn't */
	if ((desc->bmAttributes == USB_ENDPOINT_XFER_BULK
	     && le16_to_cpu(desc->wMaxPacketSize)
	     != BULK_FIFO_SIZE)
	    || !desc->wMaxPacketSize) {
		DMSG("%s, bad %s maxpacket\n", __FUNCTION__, _ep->name);
		return -ERANGE;
	}

	dev = ep->dev;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN) {
		DMSG("%s, bogus device state\n", __FUNCTION__);
		return -ESHUTDOWN;
	}

	ep->desc = desc;
	ep->dma = -1;
	ep->stopped = 0;
	ep->pio_irqs = ep->dma_irqs = 0;
	ep->ep.maxpacket = le16_to_cpu(desc->wMaxPacketSize);

	/* flush fifo (mostly for OUT buffers) */
	pxa3xx_ep_fifo_flush(_ep);

	/* ... reset halt state too, if we could ... */

#ifdef	CONFIG_USB_PXA3XX_DMA
	/* for (some) bulk and ISO endpoints, try to get a DMA channel and
	 * bind it to the endpoint.  otherwise use PIO.
	 */
	DMSG("%s: called attributes=%d\n", __FUNCTION__, ep->ep_type);
	switch (ep->ep_type) {
	case USB_ENDPOINT_XFER_ISOC:
		if (le16_to_cpu(desc->wMaxPacketSize) % 32)
			break;
		/*  fall through */
	case USB_ENDPOINT_XFER_BULK:
		if (!use_dma || !ep->reg_drcmr)
			break;
		ep->dma = pxa_request_dma((char *)_ep->name,
					  (le16_to_cpu(desc->wMaxPacketSize) >
					   64)
					  ? DMA_PRIO_MEDIUM	/* some iso */
					  : DMA_PRIO_LOW,
					  dma_nodesc_handler, ep);
		if (ep->dma >= 0) {
			*ep->reg_drcmr = DRCMR_MAPVLD | ep->dma;
			ep->dma_buffer_size = DMA_BUFFER_SIZE;
			ep->dma_buffer_virt = dma_alloc_coherent(dev->dev,
								 DMA_BUFFER_SIZE,
								 &ep->
								 dma_buffer_phys,
								 /* GFP_KERNEL); called in irq, should not be switched out */
								 GFP_ATOMIC);
			if (!ep->dma_buffer_virt) {
				printk(KERN_ERR
				       "%s: failed to allocate dma buffer\n",
				       __FUNCTION__);
				return -ENOMEM;
			}
			DMSG("%s using dma%d\n", _ep->name, ep->dma);
		}
		break;
	default:
		break;
	}
#endif
	DBG(DBG_VERBOSE, "enabled %s\n", _ep->name);
	return 0;
}

static int pxa3xx_ep_disable(struct usb_ep *_ep)
{
	struct pxa3xx_ep *ep;
	unsigned long flags;

	ep = container_of(_ep, struct pxa3xx_ep, ep);
	if (!_ep || !ep->desc) {
		DMSG("%s, %s not enabled\n", __FUNCTION__,
		     _ep ? ep->ep.name : NULL);
		return -EINVAL;
	}
	nuke(ep, -ESHUTDOWN);

	local_irq_save_nort(flags);
	spin_lock_rt(&ep->dev->lock);

#ifdef	CONFIG_USB_PXA3XX_DMA
	if (ep->dma >= 0) {
		*ep->reg_drcmr = 0;
		pxa_free_dma(ep->dma);
		dma_free_coherent(ep->dev->dev, ep->dma_buffer_size,
				  ep->dma_buffer_virt, ep->dma_buffer_phys);
		ep->dma_buffer_virt = NULL;
		ep->dma_buffer_phys = ep->dma_buffer_size = -1;
		ep->dma = -1;
	}
#endif

	/* flush fifo (mostly for IN buffers) */
	pxa3xx_ep_fifo_flush(_ep);

	ep->desc = 0;
	ep->stopped = 1;

	local_irq_restore_nort(flags);
	spin_unlock_rt(&ep->dev->lock);

	DBG(DBG_VERBOSE, "%s disabled\n", _ep->name);
	return 0;
}

/*-------------------------------------------------------------------------*/

/* for the pxa3xx, these can just wrap kmalloc/kfree.  gadget drivers
 * must still pass correctly initialized endpoints, since other controller
 * drivers may care about how it's currently set up (dma issues etc).
 */

/*
 * 	pxa3xx_ep_alloc_request - allocate a request data structure
 */
static struct usb_request *pxa3xx_ep_alloc_request(struct usb_ep *_ep,
						   int gfp_flags)
{
	struct pxa3xx_request *req;

	req = kmalloc(sizeof *req, gfp_flags);
	if (!req)
		return 0;

	memset(req, 0, sizeof *req);
	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}

/*
 * 	pxa3xx_ep_free_request - deallocate a request data structure
 */
static void pxa3xx_ep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct pxa3xx_request *req;

	req = container_of(_req, struct pxa3xx_request, req);
	WARN_ON(!list_empty(&req->queue));
	kfree(req);
}

/* PXA cache needs flushing with DMA I/O (it's dma-incoherent), but there's
 * no device-affinity and the heap works perfectly well for i/o buffers.
 * It wastes much less memory than dma_alloc_coherent() would, and even
 * prevents cacheline (32 bytes wide) sharing problems.
 */
static void *pxa3xx_ep_alloc_buffer(struct usb_ep *_ep, unsigned bytes,
				    dma_addr_t * dma, int gfp_flags)
{
	char *retval;

	retval = kmalloc(bytes, gfp_flags & ~(__GFP_DMA | __GFP_HIGHMEM));
	if (retval)
		*dma = __virt_to_bus((unsigned long)retval);
	return retval;
}

static void
pxa3xx_ep_free_buffer(struct usb_ep *_ep, void *buf, dma_addr_t dma,
		      unsigned bytes)
{
	kfree(buf);
}

/*-------------------------------------------------------------------------*/

/*
 *	done - retire a request; caller blocked irqs
 */
static void done(struct pxa3xx_ep *ep, struct pxa3xx_request *req, int status)
{
	DMSG("%s is called\n", __FUNCTION__);
	list_del_init(&req->queue);
	if (likely(req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

	if (status && status != -ESHUTDOWN) {
		DBG(DBG_VERBOSE, "complete %s req %p stat %d len %u/%u\n",
		    ep->ep.name, &req->req, status,
		    req->req.actual, req->req.length);
	}

	/* don't modify queue heads during completion callback */
	req->req.complete(&ep->ep, &req->req);	/* FIXME, better use tasklet */
}

static inline void ep0_idle(struct pxa3xx_udc *dev)
{
	dev->ep0state = EP0_IDLE;
	LED_EP0_OFF;
}

static int
write_packet(volatile u32 * uddr, struct pxa3xx_request *req, unsigned max)
{
	u32 *buf;
	int length, count, remain;

	buf = (u32 *) (req->req.buf + req->req.actual);
	prefetch(buf);

	/* how big will this packet be? */
	length = min(req->req.length - req->req.actual, max);
	req->req.actual += length;

	remain = length & 0x3;
	count = length & ~(0x3);

	while (likely(count)) {
		*uddr = *buf++;
		count -= 4;
	}

	if (remain) {
		volatile u8 *reg = (u8 *) uddr;
		char *rd = (u8 *) buf;

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
static int write_fifo(struct pxa3xx_ep *ep, struct pxa3xx_request *req)
{
	unsigned max;

	max = le16_to_cpu(ep->desc->wMaxPacketSize);
	do {
		int count;
		int is_last, is_short;

		count = write_packet(ep->reg_udcdr, req, max);

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

		DMSG("wrote %s count:%d bytes%s%s %d left %p\n",
		     ep->ep.name, count,
		     is_last ? "/L" : "", is_short ? "/S" : "",
		     req->req.length - req->req.actual, &req->req);

		/* let loose that packet. maybe try writing another one,
		 * double buffering might work.  TSP, TPC, and TFS
		 * bit values are the same for all normal IN endpoints.
		 */
		*ep->reg_udccsr = UDCCSR_PC;
		if (is_short)
			*ep->reg_udccsr = UDCCSR_SP;

		/* requests complete when all IN data is in the FIFO */
		if (is_last) {
			if (list_empty(&ep->queue) || unlikely(ep->dma >= 0)) {
				pio_irq_disable(ep->ep_num);
#ifdef	CONFIG_USB_PXA3XX_DMA
				/* unaligned data and zlps couldn't use dma */
				if (unlikely(!list_empty(&ep->queue))) {
					req = list_entry(ep->queue.next,
							 struct pxa3xx_request,
							 queue);
					kick_dma(ep, req);
					return 0;
				}
#endif
			}
			return 1;
		}

		/*  TODO experiment: how robust can fifo mode tweaking be?
		 *  double buffering is off in the default fifo mode, which
		 *  prevents TFS from being set here.
		 */
	} while (*ep->reg_udccsr & UDCCSR_FS);
	return 0;
}

/* caller asserts req->pending (ep0 irq status nyet cleared); starts
 * ep0 data stage.  these chips want very simple state transitions.
 */
static inline void ep0start(struct pxa3xx_udc *dev, u32 flags, const char *tag)
{
	UDCCSR0 =
	    flags | UDCCSR0_SA | UDCCSR0_OPC | UDCCSR0_ACM | UDCCSR0_ODFCLR;
	dev->req_pending = 0;
	DBG(DBG_VERY_NOISY, "%s %s, %02x/%02x\n",
	    __FUNCTION__, tag, UDCCSR0, flags);
}

static int write_ep0_fifo(struct pxa3xx_ep *ep, struct pxa3xx_request *req)
{
	unsigned count;
	int is_short;

	count = write_packet(&UDCDR0, req, EP0_FIFO_SIZE);
	ep->dev->stats.write.bytes += count;

	/* last packet "must be" short (or a zlp) */
	is_short = (count != EP0_FIFO_SIZE);

	DBG(DBG_VERY_NOISY, "ep0in %d bytes %d left %p\n", count,
	    req->req.length - req->req.actual, &req->req);

	if (unlikely(is_short)) {
		if (ep->dev->req_pending)
			ep0start(ep->dev, UDCCSR0_IPR, "short IN");
		else
			UDCCSR0 = UDCCSR0_ACM | UDCCSR0_IPR | UDCCSR0_ODFCLR;

		count = req->req.length;
		done(ep, req, 0);
		ep0_idle(ep->dev);
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
static int read_fifo(struct pxa3xx_ep *ep, struct pxa3xx_request *req)
{
	for (;;) {
		u32 *buf;
		int bufferspace, count, is_short;

		/* make sure there's a packet in the FIFO. */
		if (unlikely((*ep->reg_udccsr & UDCCSR_PC) == 0))
			break;
		buf = (u32 *) (req->req.buf + req->req.actual);
		prefetchw(buf);
		bufferspace = req->req.length - req->req.actual;

		/* read all bytes from this packet */
		if (likely(*ep->reg_udccsr & UDCCSR_BNE)) {
			count = 0x3ff & *ep->reg_udcbcr;
			req->req.actual += min(count, bufferspace);
		} else		/* zlp */
			count = 0;

		is_short = (count < ep->ep.maxpacket);
		DMSG("read %s udccsr:%02x, count:%d bytes%s req %p %d/%d\n",
		     ep->ep.name, *ep->reg_udccsr, count,
		     is_short ? "/S" : "",
		     &req->req, req->req.actual, req->req.length);

		count = min(count, bufferspace);
		while (likely(count > 0)) {
			*buf++ = *ep->reg_udcdr;
			count -= 4;
		}
		DMSG("Buf:0x%p\n", req->req.buf);

		*ep->reg_udccsr = UDCCSR_PC;
		/* RPC/RSP/RNE could now reflect the other packet buffer */

		/* completion */
		if (is_short || req->req.actual == req->req.length) {
			if (list_empty(&ep->queue))
				pio_irq_disable(ep->ep_num);
			return 1;
		}

		/* finished that packet.  the next one may be waiting... */
	}
	return 0;
}

/*
 * special ep0 version of the above.  no UBCR0 or double buffering; status
 * handshaking is magic.  most device protocols don't need control-OUT.
 * CDC vendor commands (and RNDIS), mass storage CB/CBI, and some other
 * protocols do use them.
 */
static int read_ep0_fifo(struct pxa3xx_ep *ep, struct pxa3xx_request *req)
{
	u32 *buf, word;
	unsigned bufferspace;

	buf = (u32 *) (req->req.buf + req->req.actual);
	bufferspace = req->req.length - req->req.actual;

	while (UDCCSR0 & UDCCSR0_RNE) {	/* FIXME if setup data is not multiple of 4, this routing will read some extra bytes */
		word = UDCDR0;

		if (unlikely(bufferspace == 0)) {
			/* this happens when the driver's buffer
			 * is smaller than what the host sent.
			 * discard the extra data.
			 */
			if (req->req.status != -EOVERFLOW)
				DMSG("%s overflow\n", ep->ep.name);
			req->req.status = -EOVERFLOW;
		} else {
			*buf++ = word;
			req->req.actual += 4;
			bufferspace -= 4;
		}
	}
	UDCCSR0 = UDCCSR0_ACM | UDCCSR0_OPC | UDCCSR0_ODFCLR;
	/* completion */
	if (req->req.actual >= req->req.length) {
		req->req.actual = req->req.length;
		return 1;
	}

	/* finished that packet.  the next one may be waiting... */
	return 0;
}

#ifdef	CONFIG_USB_PXA3XX_DMA

#define	MAX_IN_DMA	((DCMD_LENGTH + 1) - BULK_FIFO_SIZE)

static void start_dma(struct pxa3xx_ep *ep, struct pxa3xx_request *req, int len)
{
	u32 buf = (u32) ep->dma_buffer_phys;
	u32 fifo = (u32) io_v2p((u32) (ep->reg_udcdr));

	DCSR(ep->dma) = DCSR_NODESC;

	if (ep->dir_in) {
		DSADR(ep->dma) = buf;
		DTADR(ep->dma) = fifo;
		DCMD(ep->dma) = DCMD_BURST32 | DCMD_WIDTH4 | DCMD_ENDIRQEN |
		    DCMD_FLOWTRG | DCMD_INCSRCADDR | len;
	} else {
		DSADR(ep->dma) = fifo;
		DTADR(ep->dma) = buf;
		DCMD(ep->dma) = DCMD_BURST32 | DCMD_WIDTH4 | DCMD_ENDIRQEN |
		    DCMD_FLOWSRC | DCMD_INCTRGADDR | len;

		if (ep->ep_num < 16)
			UDCICR0 |= (UDCICR_PKTCOMPL << (ep->ep_num << 1));
		else
			UDCICR1 |= (UDCICR_PKTCOMPL << (ep->ep_num << 1));
	}

	*(ep->reg_udccsr) = UDCCSR_DME;
	DCSR(ep->dma) = DCSR_RUN | DCSR_NODESC;
}

static void kick_dma(struct pxa3xx_ep *ep, struct pxa3xx_request *req)
{
	u32 len = req->req.length;
	char *buf = (char *)req->req.buf;

	buf += req->req.actual;
	len -= req->req.actual;
	ep->dma_con = 0;

	DCSR(ep->dma) &= ~DCSR_RUN;
	DMSG("%s: req:0x%p, buf:%p, length:%d, actual:%d dma:%d\n",
	     __FUNCTION__, &req->req, req->req.buf, req->req.length,
	     req->req.actual, ep->dma);

	if (len > ep->dma_buffer_size)
		ep->dma_con = 1;
	len = min(len, (u32) ep->dma_buffer_size);
	if (ep->dir_in) {
		memcpy(ep->dma_buffer_virt, buf, len);
	}

	start_dma(ep, req, len);
}

static void cancel_dma(struct pxa3xx_ep *ep)
{
	struct pxa3xx_request *req;
	u32 tmp;

	if (DCSR(ep->dma) == 0 || list_empty(&ep->queue))
		return;

	DMSG("dma:%d,dcsr:0x%x\n", ep->dma, DCSR(ep->dma));
	DCSR(ep->dma) = 0;
	while ((DCSR(ep->dma) & DCSR_STOPSTATE) == 0)
		cpu_relax();

	req = list_entry(ep->queue.next, struct pxa3xx_request, queue);
	tmp = DCMD(ep->dma) & DCMD_LENGTH;
	req->req.actual = req->req.length - tmp;

	/* the last tx packet may be incomplete, so flush the fifo.
	 * FIXME correct req.actual if we can
	 */
	*ep->reg_udccsr = UDCCSR_FEF;
}

static void dma_nodesc_handler(int dmach, void *_ep, struct pt_regs *r)
{
	struct pxa3xx_ep *ep = _ep;
	struct pxa3xx_request *req, *req_next;
	u32 dcsr, completed, remained, count;
	unsigned long flags;

	DMSG("\n");
	local_irq_save(flags);

	local_irq_save_nort(flags);
	spin_lock_rt(&ep->dev->lock);

	req = list_entry(ep->queue.next, struct pxa3xx_request, queue);

	ep->dma_irqs++;
	ep->dev->stats.irqs++;
	HEX_DISPLAY(ep->dev->stats.irqs);

	completed = 0;
	remained = req->req.length - req->req.actual;

	dcsr = DCSR(dmach);
	DCSR(ep->dma) &= ~DCSR_RUN;

	DMSG("%s, buf:0x%p, dmach:%d, dcsr:%x\n", __FUNCTION__, req->req.buf,
	     dmach, dcsr);
	if (dcsr & DCSR_BUSERR) {
		printk(KERN_ERR " Buss Error\n");
		DMSG("dcsr:%x, ddadr:%x, dsadr:%x, dtadr:%x, dcmd:%x\n",
		     DCSR(dmach), DDADR(dmach), DSADR(dmach),
		     DTADR(dmach), DCMD(dmach));
		DCSR(dmach) = DCSR_BUSERR;
		req->req.status = -EIO;
		completed = 1;
	} else if (dcsr & DCSR_ENDINTR) {
		DCSR(dmach) = DCSR_ENDINTR;
		if (ep->dir_in) {
			/* There are still packets to transfer */
			if (ep->dma_con) {
				DMSG("%s: more packets,length:%d,actual:%d\n",
				     __FUNCTION__, req->req.length,
				     req->req.actual);
				req->req.actual += ep->dma_buffer_size;
			} else if (remained % BULK_FIFO_SIZE) {
				count = 0;

				*ep->reg_udccsr = UDCCSR_SP |
				    (*ep->reg_udccsr & UDCCSR_MASK);
				/*Wait for packet out */
				while ((count++ < 10000) &&
				       !(*ep->reg_udccsr & UDCCSR_PC)) ;
				if (count >= 10000)
					DMSG("Failed to send short packet\n");
				else
					DMSG("%s: short packet sent length:"
					     "%d,actual:%d\n", __FUNCTION__,
					     req->req.length, req->req.actual);
				completed = 1;
				req->req.actual = req->req.length;
			} else {	/* It is whole package */
				/* FIXME Sent a ZLP? */
				completed = 1;
				req->req.actual = req->req.length;
				DMSG("%s: req->req.zero=%d, req->req.length=%d\n", __FUNCTION__, req->req.zero, req->req.length);
			}
		} else {
			if (ep->dma_con) {
				memcpy((char *)req->req.buf + req->req.actual,
				       ep->dma_buffer_virt,
				       ep->dma_buffer_size);
				req->req.actual += ep->dma_buffer_size;
			} else {
				completed = 1;
				memcpy((char *)req->req.buf + req->req.actual,
				       ep->dma_buffer_virt,
				       req->req.length - req->req.actual);
				req->req.actual = req->req.length;
				DMSG("%s, fully data received\n", __FUNCTION__);
			}
		}
	} else
		DMSG("%s: Others dma:%d DCSR:0x%x DCMD:0x%x\n",
		     __FUNCTION__, dmach, DCSR(dmach), DCMD(dmach));

	if (likely(completed)) {
		if (req->queue.next != &ep->queue) {
			req_next = list_entry(req->queue.next,
					      struct pxa3xx_request, queue);
			kick_dma(ep, req_next);
		}
	} else {
		kick_dma(ep, req);
	}

	local_irq_restore_nort(flags);
	spin_unlock_rt(&ep->dev->lock);
	if (likely(completed)) {
		done(ep, req, 0);
	}
}

#endif
/*-------------------------------------------------------------------------*/

static int
pxa3xx_ep_queue(struct usb_ep *_ep, struct usb_request *_req, int gfp_flags)
{
	struct pxa3xx_ep *ep;
	struct pxa3xx_request *req;
	struct pxa3xx_udc *dev;
	unsigned long flags;
	int completed = 0;

	req = container_of(_req, struct pxa3xx_request, req);
	if (unlikely(!_req || !_req->complete || !_req->buf ||
		     !list_empty(&req->queue))) {
		DMSG("%s, bad params\n", __FUNCTION__);
		return -EINVAL;
	}

	ep = container_of(_ep, struct pxa3xx_ep, ep);
	if (unlikely(!_ep || (!ep->desc && ep->ep.name != ep0name))) {
		DMSG("%s, bad ep\n", __FUNCTION__);
		return -EINVAL;
	}

	DMSG("%s, ep point %d is queue\n", __FUNCTION__, ep->ep_num);

	dev = ep->dev;
	if (unlikely(!dev->driver || ((dev->ep0state != EP0_IN_FAKE)
				      && (dev->gadget.speed ==
					  USB_SPEED_UNKNOWN)))) {
		DMSG("%s, bogus device state\n", __FUNCTION__);
		return -ESHUTDOWN;
	}

	/* iso is always one packet per request, that's the only way
	 * we can report per-packet status.  that also helps with dma.
	 */
	if (unlikely(ep->ep_type == USB_ENDPOINT_XFER_ISOC
		     && req->req.length > le16_to_cpu
		     (ep->desc->wMaxPacketSize)))
		return -EMSGSIZE;

#if VERBOSE
	/*  FIXME caller may already have done the dma mapping */
	if (ep->dma >= 0) {
		_req->dma = dma_map_single(dev->dev, _req->buf, _req->length,
					   (ep->
					    dir_in) ? DMA_TO_DEVICE :
					   DMA_FROM_DEVICE);
	}
#endif

	DBG(DBG_NOISY, "%s queue req %p, len %d buf %p\n",
	    _ep->name, _req, _req->length, _req->buf);

	local_irq_save_nort(flags);
	spin_lock_rt(&ep->dev->lock);

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	/* kickstart this i/o queue? */
	if (list_empty(&ep->queue) && !ep->stopped) {
		if (ep->desc == 0 /* ep0 */ ) {
			unsigned length = _req->length;

			switch (dev->ep0state) {
			case EP0_IN_DATA_PHASE:
				dev->stats.write.ops++;
				DMSG("%s, dev->req_config = %d\n", __FUNCTION__,
				     dev->req_config);
				if (dev->req_config) {
#ifdef CONFIG_USB_COMPOSITE
					if (dev->req_config > 1) {
						dev->req_config--;
						done(ep, req, 0);
						req = 0;
						break;
					}
#endif
					DMSG("ep0: set config finished\n");
					UDCCSR0 =
					    UDCCSR0_ACM | UDCCSR0_AREN |
					    UDCCSR0_ODFCLR;
					dev->req_config = 0;
					ep0_idle(dev);
					done(ep, req, 0);
					req = 0;
				} else if (write_ep0_fifo(ep, req))
					req = 0;
				break;

			case EP0_OUT_DATA_PHASE:
				dev->stats.read.ops++;
				if (dev->req_pending)
					ep0start(dev, UDCCSR0_IPR, "OUT");
				if (length == 0 || ((UDCCSR0 & UDCCSR0_RNE) != 0
						    && read_ep0_fifo(ep,
								     req))) {
					ep0_idle(dev);
					UDCCSR0 |= UDCCSR0_IPR | UDCCSR0_ODFCLR;
					done(ep, req, 0);
					req = 0;
				}
				break;
			case EP0_NO_ACTION:
				ep0_idle(dev);
				req = 0;
				break;
			case EP0_IN_FAKE:
				DMSG("%s: in EP0_IN_FAKE\n", __FUNCTION__);
				dev->config_length = _req->length;
#ifndef CONFIG_USB_COMPOSITE
				memcpy(dev->configs, _req->buf, _req->length);
#else
#ifndef MULTI_P3
				memcpy(dev->active_gadget->config_desc,
				       _req->buf, _req->length);
#else
				if (((__u8 *) (_req->buf))[1] == USB_DT_CONFIG) {
					memcpy(dev->active_gadget->config_desc,
					       _req->buf, _req->length);
				} else if (((__u8 *) (_req->buf))[1] ==
					   USB_DT_DEVICE) {
					memcpy(dev->active_gadget->device_desc,
					       _req->buf, _req->length);
				}
#endif				/* MULTI_P3 */
#endif
				ep0_idle(dev);
				req->req.actual = req->req.length;
				done(ep, req, 0);
				req = 0;
				break;
			default:
				DMSG("ep0 i/o, odd state %d\n", dev->ep0state);
				local_irq_restore_nort(flags);
				spin_unlock_rt(&dev->lock);
				return -EL2HLT;
			}
#ifdef	CONFIG_USB_PXA3XX_DMA
			/* either start dma or prime pio pump */
		} else if (ep->dma >= 0) {
			if ((_req->length == 0) && ep->dir_in) {	/* ZLP */
				*ep->reg_udccsr = UDCCSR_SP | UDCCSR_DME;
				done(ep, req, 0);
				req = 0;
			} else
				kick_dma(ep, req);
#else
			/* can the FIFO can satisfy the request immediately? */
		} else if (ep->dir_in && (*ep->reg_udccsr & UDCCSR_FS) != 0) {	   /*  pio mode error, FIXME */
			completed = write_fifo(ep, req);

		} else if (!(ep->dir_in)	/*  neo */
			&&(*ep->reg_udccsr & UDCCSR_FS) != 0 ) {
			completed = read_fifo(ep, req);
#endif
		}
		if (completed)   {
			local_irq_restore_nort(flags);
			spin_unlock_rt(&dev->lock);
			done(ep, req, 0);
			return 0;
		}

		DMSG("req:%p,ep->desc:%p,ep->dma:%d\n", req, ep->desc, ep->dma);
		if (likely(req && ep->desc))
			pio_irq_enable(ep->ep_num);
	}

	/* pio or dma irq handler advances the queue. */
	if (likely(req))
		list_add_tail(&req->queue, &ep->queue);
	local_irq_restore_nort(flags);
	spin_unlock_rt(&dev->lock);

	return 0;
}

/*
 * 	nuke - dequeue ALL requests
 */
static void nuke(struct pxa3xx_ep *ep, int status)
{
	struct pxa3xx_request *req;

	/* called with irqs blocked */
#ifdef	CONFIG_USB_PXA3XX_DMA
	if (ep->dma >= 0 && !ep->stopped)
		cancel_dma(ep);
#endif
	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct pxa3xx_request, queue);
		done(ep, req, status);
	}
	if (ep->desc)
		pio_irq_disable(ep->ep_num);
}

/* dequeue JUST ONE request */
static int pxa3xx_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct pxa3xx_ep *ep;
	struct pxa3xx_request *req;
	unsigned long flags;

	ep = container_of(_ep, struct pxa3xx_ep, ep);
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
#ifdef	CONFIG_USB_PXA3XX_DMA
	if (ep->dma >= 0 && ep->queue.next == &req->queue && !ep->stopped) {
		cancel_dma(ep);
		done(ep, req, -ECONNRESET);
		/* restart i/o */
		if (!list_empty(&ep->queue)) {
			req = list_entry(ep->queue.next,
					 struct pxa3xx_request, queue);
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

static int pxa3xx_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct pxa3xx_ep *ep;
	unsigned long flags;

	DMSG("%s is called\n", __FUNCTION__);
	ep = container_of(_ep, struct pxa3xx_ep, ep);
	if (unlikely(!_ep || (!ep->desc && ep->ep.name != ep0name))
	    || ep->ep_type == USB_ENDPOINT_XFER_ISOC) {
		DMSG("%s, bad ep\n", __FUNCTION__);
		return -EINVAL;
	}
	if (value == 0) {
		/* this path (reset toggle+halt) is needed to implement
		 * SET_INTERFACE on normal hardware.  but it can't be
		 * done from software on the PXA UDC, and the hardware
		 * forgets to do it as part of SET_INTERFACE automagic.
		 */
		DMSG("only host can clear %s halt\n", _ep->name);
		return -EROFS;
	}
#ifdef CONFIG_USB_COMPOSITE
	if (!list_empty(&ep->queue) && ep->dir_in) {
		DMSG("%s, -EAGAIN\n", __FUNCTION__);
		return (-EAGAIN);
	}
#endif

	local_irq_save_nort(flags);
	spin_lock_rt(&ep->dev->lock);

	*ep->reg_udccr = UDCCSR_FST | UDCCSR_FEF;

	/* ep0 needs special care */
	if (!ep->desc) {
		start_watchdog(ep->dev);
		ep->dev->req_pending = 0;
		ep->dev->ep0state = EP0_STALL;
		LED_EP0_OFF;

		/* and bulk/intr endpoints like dropping stalls too */
	} else {
		unsigned i;

		for (i = 0; i < 1000; i += 20) {
			if (*ep->reg_udccsr & UDCCSR_SST)
				break;
			udelay(20);
		}
	}
	local_irq_restore_nort(flags);
	spin_unlock_rt(&ep->dev->lock);

	DBG(DBG_VERBOSE, "%s halt\n", _ep->name);
	return 0;
}

static int pxa3xx_ep_fifo_status(struct usb_ep *_ep)
{
	struct pxa3xx_ep *ep;

	ep = container_of(_ep, struct pxa3xx_ep, ep);
	if (!_ep) {
		DMSG("%s, bad ep\n", __FUNCTION__);
		return -ENODEV;
	}
	/* pxa can't report unclaimed bytes from IN fifos */
	if (ep->dir_in)
		return -EOPNOTSUPP;
	if (ep->dev->gadget.speed == USB_SPEED_UNKNOWN
	    || (*ep->reg_udccsr & UDCCSR_FS) == 0)
		return 0;
	else
		return (*ep->reg_udcbcr & 0xfff) + 1;
}

static void pxa3xx_ep_fifo_flush(struct usb_ep *_ep)
{
	struct pxa3xx_ep *ep;

	ep = container_of(_ep, struct pxa3xx_ep, ep);
	if (!_ep || ep->ep.name == ep0name || !list_empty(&ep->queue)) {
		DMSG("%s, bad ep\n", __FUNCTION__);
		return;
	}

	/* toggle and halt bits stay unchanged */

	/* for OUT, just read and discard the FIFO contents. */
	if (!ep->dir_in) {
		while (((*ep->reg_udccsr) & UDCCSR_BNE) != 0)
			(void)*ep->reg_udcdr;
		return;
	}

	/* most IN status is the same, but ISO can't stall */
	*ep->reg_udccsr = UDCCSR_PC | UDCCSR_FST | UDCCSR_TRN
	    | (ep->ep_type == USB_ENDPOINT_XFER_ISOC)
	    ? 0 : UDCCSR_SST;
}

static struct usb_ep_ops pxa3xx_ep_ops = {
	.enable = pxa3xx_ep_enable,
	.disable = pxa3xx_ep_disable,

	.alloc_request = pxa3xx_ep_alloc_request,
	.free_request = pxa3xx_ep_free_request,

	.alloc_buffer = pxa3xx_ep_alloc_buffer,
	.free_buffer = pxa3xx_ep_free_buffer,

	.queue = pxa3xx_ep_queue,
	.dequeue = pxa3xx_ep_dequeue,

	.set_halt = pxa3xx_ep_set_halt,
	.fifo_status = pxa3xx_ep_fifo_status,
	.fifo_flush = pxa3xx_ep_fifo_flush,
};

/* ---------------------------------------------------------------------------
 * 	device-scoped parts of the api to the usb controller hardware
 * ---------------------------------------------------------------------------
 */

static int pxa3xx_udc_get_frame(struct usb_gadget *_gadget)
{
	return (UDCFNR & 0x3FF);
}

static int pxa3xx_udc_wakeup(struct usb_gadget *_gadget)
{
	struct pxa3xx_udc *dev;

	dev = container_of(_gadget, struct pxa3xx_udc, gadget);
	/* if remote wakeup is not enabled, call SRP */
	if ((UDCCR & UDCCR_DWRE) == 0) {
		if (dev->transceiver) {
			return otg_start_srp(dev->transceiver);
		}
	} else
		UDCCR = (UDCCR & (UDCCR_OEN | UDCCR_UDE)) | UDCCR_UDR;
	return 0;
}

static const struct usb_gadget_ops pxa3xx_udc_ops = {
	.get_frame = pxa3xx_udc_get_frame,
	.wakeup = pxa3xx_udc_wakeup,
	/*  current versions must always be self-powered */
};

#ifdef DEBUG
static void gadget_info_dump(void);
#endif

/*-------------------------------------------------------------------------*/

#ifdef UDC_PROC_FILE

static struct pxa3xx_udc memory;

static const char proc_node_name[] = "driver/udc";
static const char none[] = "none";

static int
udc_proc_read(char *page, char **start, off_t off, int count,
	      int *eof, void *_dev)
{
	char *buf = page;
	struct pxa3xx_udc *dev = _dev;
	char *next = buf;
	unsigned size = count;
	unsigned long flags;
	int i, t;
	u32 tmp;
#ifdef CONFIG_USB_COMPOSITE
	char *name = (char *)none;
	if (dev->driver)
		name = (char *)dev->driver->driver.name;
#endif

	if (off != 0)
		return 0;

	local_irq_save_nort(flags);
	spin_lock_rt(&dev->lock);

	/* basic device status */
	t = scnprintf(next, size, DRIVER_DESC "\n"
		      "%s version: %s\nGadget driver: %s, connected %d\n",
		      driver_name, DRIVER_VERSION SIZE_STR DMASTR,
#ifndef CONFIG_USB_COMPOSITE
		      dev->driver ? dev->driver->driver.name : "(none)");
#else
		      (dev->driver_count > 1) ? "(composite)" : name,
		      connected);
#endif
	size -= t;
	next += t;

	/* registers for device and ep0 */
	t = scnprintf(next, size,
		      "uicr %02X.%02X, usir %02X.%02x, ufnr %02X\n",
		      UDCICR1, UDCICR0, UDCISR1, UDCISR0, UDCFNR);
	size -= t;
	next += t;

#ifdef CONFIG_USB_OTG
	/* register for otg */
	t = scnprintf(next, size, "up2ocr:%02X, udcotgicr:%02X, "
		      "udcotgisr:%02X\n", UP2OCR, UDCOTGICR, UDCOTGISR);
	size -= t;
	next += t;
#endif

	tmp = UDCCR;
	t = scnprintf(next, size,
		      "udccr %02X =%s%s%s%s%s%s%s%s%s%s, con=%d,inter=%d,altinter=%d\n",
		      tmp, (tmp & UDCCR_OEN) ? " oen" : "",
		      (tmp & UDCCR_AALTHNP) ? " aalthnp" : "",
		      (tmp & UDCCR_AHNP) ? " rem" : "",
		      (tmp & UDCCR_BHNP) ? " rstir" : "",
		      (tmp & UDCCR_DWRE) ? " dwre" : "",
		      (tmp & UDCCR_SMAC) ? " smac" : "",
		      (tmp & UDCCR_EMCE) ? " emce" : "",
		      (tmp & UDCCR_UDR) ? " udr" : "",
		      (tmp & UDCCR_UDA) ? " uda" : "",
		      (tmp & UDCCR_UDE) ? " ude" : "",
		      (tmp & UDCCR_ACN) >> UDCCR_ACN_S,
		      (tmp & UDCCR_AIN) >> UDCCR_AIN_S,
		      (tmp & UDCCR_AAISN) >> UDCCR_AAISN_S);

	size -= t;
	next += t;

	tmp = UDCCSR0;
	t = scnprintf(next, size,
		      "udccsr0 %02X =%s%s%s%s%s%s%s\n", tmp,
		      (tmp & UDCCSR0_SA) ? " sa" : "",
		      (tmp & UDCCSR0_RNE) ? " rne" : "",
		      (tmp & UDCCSR0_FST) ? " fst" : "",
		      (tmp & UDCCSR0_SST) ? " sst" : "",
		      (tmp & UDCCSR0_DME) ? " dme" : "",
		      (tmp & UDCCSR0_IPR) ? " ipr" : "",
		      (tmp & UDCCSR0_OPC) ? " opc" : "");
	size -= t;
	next += t;

	if (!dev->driver)
		goto done;

	t = scnprintf(next, size, "ep0 IN %lu/%lu, OUT %lu/%lu\nirqs %lu\n\n",
		      dev->stats.write.bytes, dev->stats.write.ops,
		      dev->stats.read.bytes, dev->stats.read.ops,
		      dev->stats.irqs);
	size -= t;
	next += t;

	/* dump endpoint queues */
	for (i = 0; i < UDC_EP_NUM; i++) {
		struct pxa3xx_ep *ep = &dev->ep[i];
		struct pxa3xx_request *req;
		int t;

		if (i != 0) {
			const struct usb_endpoint_descriptor *d;

			d = ep->desc;
			if (!d)
				continue;
			tmp = *dev->ep[i].reg_udccsr;
			t = scnprintf(next, size,
#ifndef CONFIG_USB_COMPOSITE
				      "%s max %d %s udccs %02x udccr:0x%x\n",
				      ep->ep.name,
				      le16_to_cpu(d->wMaxPacketSize),
				      (ep->dma >= 0) ? "dma" : "pio", tmp,
				      *dev->ep[i].reg_udccr);
#else
				      "%s max %d %s udccs %02x udccr:0x%x, intf=%d(%d)\n",
				      ep->ep.name,
				      le16_to_cpu(d->wMaxPacketSize),
				      (ep->dma >= 0) ? "dma" : "pio", tmp,
				      *dev->ep[i].reg_udccr,
				      dev->ep[i].assigned_interface,
				      dev->ep[i].interface);
#endif
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
#ifdef	CONFIG_USB_PXA3XX_DMA
			if (ep->dma >= 0 && req->queue.prev == &ep->queue)
				t = scnprintf(next, size,
					      "\treq %p len %d/%d "
					      "buf %p (dma%d dcmd %08x)\n",
					      &req->req, req->req.actual,
					      req->req.length, req->req.buf,
					      ep->dma, DCMD(ep->dma)
					      /*  low 13 bits == bytes-to-go */
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

static int udc_proc_write(struct file *filp, const char *buffer,
			  unsigned long count, void *data)
{
	char kbuf[8];
	int index;

	if (count >= 8)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;
	index = (int)simple_strtoul(kbuf, NULL, 10);

	switch (index) {
		/* Set IDON */
	case 1:
		/* UP2OCR |= 0x400 ; */
		break;
		/* Clear IDON */
	case 2:
		/* UP2OCR &= ~0x400; */
		break;

	case 3:
		break;

	case 4:
		pxa3xx_udc_wakeup(&the_controller->gadget);
		break;

	default:
		return -EINVAL;
	}
	return count;
}

#define create_proc_files() \
	do {	struct proc_dir_entry *ent;\
		ent = create_proc_entry(proc_node_name, 0, NULL);\
		if (ent) { \
			ent->data = dev; \
			ent->read_proc = udc_proc_read; \
			ent->write_proc = udc_proc_write; \
		} \
	}while(0);
#define remove_proc_files() \
	remove_proc_entry(proc_node_name, NULL)

#else				/* !UDC_PROC_FILE */
#define create_proc_files() do {} while (0)
#define remove_proc_files() do {} while (0)

#endif				/* UDC_PROC_FILE */

/* "function" sysfs attribute */
static ssize_t show_function(struct device *_dev, char *buf)
{
	struct pxa3xx_udc *dev = dev_get_drvdata(_dev);

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
static void udc_disable(struct pxa3xx_udc *dev)
{
	UDCISR0 = UDCISR1 = 0xffffffff;
	UDCICR0 = UDCICR1 = 0x00000000;

	/* disconnect from bus */
	UP2OCR &= ~(UP2OCR_DPPUE | UP2OCR_DPPDE | UP2OCR_DMPUE | UP2OCR_DMPDE);

	/* disable the controller */
	UDCCR = 0;
	pxa_set_cken(CKEN_UDC, 0);

	ep0_idle(dev);
	dev->gadget.speed = USB_SPEED_UNKNOWN;
#ifdef CONFIG_USB_COMPOSITE
	dev->configuration = 0;
	dev->interface = 0;
	dev->alternate = 0;
#endif
	LED_CONNECTED_OFF;
}

/*
 * 	udc_reinit - initialize software state
 */
static void udc_reinit(struct pxa3xx_udc *dev)
{
	u32 i;

	dev->ep0state = EP0_IDLE;

	/* basic endpoint records init */
	for (i = 0; i < UDC_EP_NUM; i++) {
		struct pxa3xx_ep *ep = &dev->ep[i];

		ep->stopped = 0;
		ep->pio_irqs = ep->dma_irqs = 0;
	}
	dev->configuration = 0;
	dev->interface = 0;
	dev->alternate = 0;
	/* the rest was statically initialized, and is read-only */
}

/* until it's enabled, this UDC should be completely invisible
 * to any USB host.
 */
static void udc_enable(struct pxa3xx_udc *dev)
{
	pxa_set_cken(CKEN_UDC, 1);
	mdelay(1);

	ep0_idle(dev);
	dev->gadget.speed = USB_SPEED_FULL;
	dev->stats.irqs = 0;

	/* enable the controller */
	UDCCR |= UDCCR_UDE;

	/* clear all interrupt status */
	UDCISR0 = 0xffffffff;
	UDCISR1 = 0xf800ffff;

	DMSG("%s, UDCCR = 0x%x\n", __FUNCTION__, UDCCR);

	/* enable suspend/resume and reset irqs */
	UDCICR1 = UDCICR1_IECC | UDCICR1_IERU | UDCICR1_IESU | UDCICR1_IERS;

	/* enable ep0 irqs */
	UDCICR0 = UDCICR_INT(0, UDCICR_INT_MASK);

#ifndef CONFIG_USB_OTG
	/* use internal transceiver in a non-OTG way */
	UP2OCR = UP2OCR_HXOE | UP2OCR_DPPUE;
#endif

}

/*-------------------------------------------------------------------*/
/*
 * get_extra_descriptor() finds a descriptor of specific type in the
 * extra field of the interface and endpoint descriptor structs.
 */
static int get_extra_descriptor(char *buffer, unsigned size,
				unsigned char type, void **ptr)
{
	struct usb_descriptor_header *header;

	*ptr = buffer;
	while (size >= sizeof(struct usb_descriptor_header)) {
		header = (struct usb_descriptor_header *)buffer;

		if (header->bLength < 2) {
			DMSG("%s: descriptor, type %d length %d not found\n",
			     __FUNCTION__,
			     header->bDescriptorType, header->bLength);
			return -ENODATA;
		}

		if (header->bDescriptorType == type) {
			*ptr = header;
			return size;
		}

		buffer += header->bLength;
		size -= header->bLength;
	}
	return -ENODATA;
}

#ifdef CONFIG_USB_COMPOSITE
static void udc_setup_complete(struct usb_ep *ep, struct usb_request *req);
#ifdef DEBUG
/*  dump the gadget_driver_info structure
 */
static void gadget_info_dump(void)
{
	struct pxa3xx_udc *dev = the_controller;
	struct gadget_driver_info *pInfo = dev->first_gadget;
	int i = 1;

	printk(KERN_DEBUG "%s, dev->interface_count= 0x%x\n", __FUNCTION__,
	       dev->interface_count);
	while (pInfo) {

		printk(KERN_DEBUG "   i=%d, pInfo=%p\n", i, pInfo);
		printk(KERN_DEBUG "   next = 0x%x\n", (unsigned)pInfo->next);
		printk(KERN_DEBUG "   config = 0x%x\n", pInfo->config);
		printk(KERN_DEBUG "   assigned_intf_start = 0x%x\n",
		       pInfo->assigned_intf_start);
		printk(KERN_DEBUG "   num_intfs = 0x%x\n", pInfo->num_intfs);
		printk(KERN_DEBUG "   ep_start = 0x%x\n", pInfo->ep_start);
		printk(KERN_DEBUG "   config_desc = 0x%x\n",
		       (unsigned)pInfo->config_desc);
		printk(KERN_DEBUG "   driver = 0x%x\n",
		       (unsigned)pInfo->driver);
		printk(KERN_DEBUG "   driver_data = 0x%x\n",
		       (unsigned)pInfo->driver_data);

		pInfo = pInfo->next;
		i++;
	}
	printk(KERN_DEBUG "dev->first_gadget = %p\n", dev->first_gadget);
	printk(KERN_DEBUG "dev->active_gadget = %p\n", dev->active_gadget);
}
#endif
/* gadget_info_init
 * init the gadget_driver_info structure when the driver is registered
 *
 */
/* combined from several gadget driver, should be lager ?? */
#define REQ_BUFSIZ 256
static int gadget_info_init(struct usb_gadget_driver *driver)
{
	struct pxa3xx_udc *dev = the_controller;
	struct gadget_driver_info *info;
	struct gadget_driver_info *pInfo;

	/* set up the new gadget driver info */
	info = kmalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		printk(KERN_ERR "kmalloc gadget_driver_info error\n");
		return -EBUSY;
	}
	memset(info, 0, sizeof *info);

	info->driver = driver;
	info->next = NULL;

	info->ep_start = num_ep_used;
	info->num_eps = 0;
	info->num_intfs = 0;
	info->stopped = 1;

	if (dev->first_gadget) {
		/* find the last element */
		pInfo = dev->first_gadget;
		while (pInfo->next) {
			pInfo = pInfo->next;
		}
		/* set up the struct
		   the last registered driver is always the active one
		   before receive the set_interface request */
		pInfo->next = info;
		dev->active_gadget = info;
		dev->driver_count++;
	} else {
		dev->first_gadget = dev->active_gadget = info;
		dev->interface_count = 0;
		dev->driver_count = 1;

		/* init ep0 control request queueand buffer */
		memset((void *)&dev->ep0_req, 0, sizeof(dev->ep0_req));
		INIT_LIST_HEAD(&dev->ep0_req.queue);

		dev->ep0_req.req.complete = udc_setup_complete;
		dev->ep0_req.req.buf = pxa3xx_ep_alloc_buffer(dev->gadget.ep0,
							      REQ_BUFSIZ,
							      &dev->ep0_req.req.
							      dma, GFP_KERNEL);
		if (!dev->ep0_req.req.buf) {
			usb_ep_free_request(dev->gadget.ep0, &dev->ep0_req.req);
			DMSG("%s, dev->ep0_req.req.buf malloc error\n",
			     __FUNCTION__);
		}
	}

	return 0;
}

#ifdef MULTI_P3
struct usb_interface_assoc_descriptor iad_desc = {
	.bLength = sizeof iad_desc,
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,

	.bFirstInterface = 0,
	.bInterfaceCount = 0,
	.bFunctionClass = 0,
	.bFunctionSubClass = 0,
	.bFunctionProtocol = 0,
	.iFunction = 0,
};

static void set_iad_desc(struct usb_device_descriptor *device_desc,
			 __u8 first_intf, __u8 num_intfs)
{
	iad_desc.bFirstInterface = first_intf;
	iad_desc.bInterfaceCount = num_intfs;

	iad_desc.bFunctionClass = device_desc->bDeviceClass;
	iad_desc.bFunctionSubClass = device_desc->bDeviceSubClass;
	iad_desc.bFunctionProtocol = device_desc->bDeviceProtocol;
}

static void gadget_get_device_desc(void)
{
	struct pxa3xx_udc *dev = the_controller;
	struct usb_ctrlrequest req;
	int i;

	DMSG(KERN_DEBUG "%s\n", __FUNCTION__);
	req.bRequestType = USB_RECIP_DEVICE | USB_DIR_IN;
	req.bRequest = USB_REQ_GET_DESCRIPTOR;
	req.wValue = (USB_DT_DEVICE << 8);
	req.wIndex = 0;
	req.wLength = sizeof(struct usb_device_descriptor);

	dev->ep0state = EP0_IN_FAKE;
	i = dev->driver->setup(&dev->gadget, &req);

}
#endif

/* combine_configuration
 * Combine the configuration descriptors for all gadget drivers registered.
 * Add the IAD descriptor if there are more than one interfaces within one
 * function.
 */
static int combine_configuration(void)
{
	struct pxa3xx_udc *dev = the_controller;
	struct gadget_driver_info *pInfo = dev->first_gadget;
	struct usb_config_descriptor *config_desc;
#ifdef MULTI_P3
	struct usb_interface_assoc_descriptor *p_iad_desc;
#endif
	struct usb_config_descriptor *configs =
	    (struct usb_config_descriptor *)dev->configs;
	int desc_length;

	DMSG("%s\n", __FUNCTION__);
	/* config desc, may diff between gadget drivers */
	configs->bLength = USB_DT_CONFIG_SIZE;
	configs->bDescriptorType = USB_DT_CONFIG;

	configs->wTotalLength = USB_DT_CONFIG_SIZE;
	configs->bNumInterfaces = 0;
	configs->bConfigurationValue =	/* UDC_DEFAULT_CONFIG; */
	    ((struct usb_config_descriptor *)pInfo->config_desc)->
	    bConfigurationValue;
	configs->iConfiguration = 0;
	configs->bmAttributes = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER;
	configs->bMaxPower = 1;

	do {
		config_desc =
		    (struct usb_config_descriptor *)(pInfo->config_desc);
		configs->bNumInterfaces += (u8) (config_desc->bNumInterfaces);

#ifdef MULTI_P3
		/* add the IAD descriptor if there are multiple interface in one
		   gadget driver */
		if (config_desc->bNumInterfaces > 1) {

			if ((get_extra_descriptor((char *)config_desc,
						  config_desc->wTotalLength,
						  USB_DT_INTERFACE_ASSOCIATION,
						  (void **)&p_iad_desc)) >= 0) {
				p_iad_desc->bFirstInterface =
				    pInfo->assigned_intf_start;
			} else {
				/* fill the iad_desc functionClass/subclass/protocol fields */
				set_iad_desc((struct usb_device_descriptor *)
					     pInfo->device_desc,
					     pInfo->assigned_intf_start,
					     config_desc->bNumInterfaces);
				memcpy((u8 *) configs + configs->wTotalLength,
				       (u8 *) & iad_desc, sizeof iad_desc);
				configs->wTotalLength += sizeof iad_desc;
			}
		}
#endif

		/* copy all descriptors except config desc */
		memcpy((u8 *) configs + configs->wTotalLength,
		       (u8 *) config_desc + USB_DT_CONFIG_SIZE,
		       config_desc->wTotalLength - USB_DT_CONFIG_SIZE);

		/* modify the interface number to assigned interface number */
		desc_length = config_desc->wTotalLength - USB_DT_CONFIG_SIZE;

		configs->wTotalLength +=
		    (config_desc->wTotalLength - USB_DT_CONFIG_SIZE);
		pInfo = pInfo->next;
		DMSG("configs->wTotalLength = 0x%x\n", configs->wTotalLength);
	} while (pInfo != NULL);

	/* gadget_info_dump(); */

	return configs->wTotalLength;
}

/* set_eps
 * fill pxa_ep structure with their configuration, interface, alternate
 * settings, assigned interface number
 */
static int set_eps(__u8 num_eps, int ep_start_num,
		   struct usb_endpoint_descriptor *p_ep_desc, int len,
		   int config, int interface, int alt)
{
	struct pxa3xx_udc *dev = the_controller;
	struct usb_endpoint_descriptor *ep_desc = p_ep_desc;
	int ep_desc_length = len;
	int j, k, ret;

	DMSG("  ----%s----\n", __FUNCTION__);

	for (j = 0; j < num_eps; j++) {
		DMSG("  search eps: ep_start_num=%d, num_ep_used=%d\n",
		     ep_start_num, num_ep_used);
		/* find the ep */
		if ((ret = get_extra_descriptor((char *)p_ep_desc,
						ep_desc_length,
						USB_DT_ENDPOINT,
						(void **)&ep_desc)) >= 0) {
			/* compare with the ep in pxa3xx, if match, fill the config,
			   interface and asin number fields */
			for (k = ep_start_num; k < num_ep_used; k++) {
				if (dev->ep[k].ep_num ==
				    (ep_desc->bEndpointAddress & 0x0f)) {
					dev->ep[k].assigned_interface =
					    dev->interface_count;
					dev->ep[k].config = config;
					dev->ep[k].interface = interface;
					dev->ep[k].aisn = alt;
					dev->ep[k].driver_info =
					    dev->active_gadget;
					DMSG("  found ep num = %d, old interface=%d," " assigned_interface=%d\n", k, interface, dev->ep[k].assigned_interface);
					break;
				}
			}
		} else {
			DMSG("  ep desc not find, ep_desc_length=0x%x, p_ep_desc=0x%x\n", ep_desc_length, (int)p_ep_desc);
			return -EFAULT;
		}

		ep_desc_length -=
		    (int)ep_desc - (int)p_ep_desc + ep_desc->bLength;
		p_ep_desc =
		    (struct usb_endpoint_descriptor *)((unsigned)ep_desc +
						       ep_desc->bLength);
	}			/* for(j=0;j<num_eps;j++) */
	return 0;
}

/* set_cdc_desc
 * modify the cdc union descriptor which include the master/slave interface
 * number
 */
#include <linux/usb_cdc.h>

static void set_cdc_desc(void)
{
	struct pxa3xx_udc *dev = the_controller;
	struct usb_device_descriptor *device_desc =
	    (struct usb_device_descriptor *)dev->active_gadget->device_desc;
	struct usb_config_descriptor *config_desc =
	    (struct usb_config_descriptor *)dev->active_gadget->config_desc;
	struct usb_cdc_union_desc *union_desc;
	int config_desc_len = config_desc->wTotalLength, ret = 0;

	if (device_desc->bDeviceClass != USB_CLASS_COMM)
		return;

	while ((config_desc_len > 0) && (ret >= 0)) {
		if ((ret = get_extra_descriptor((char *)config_desc,
						config_desc_len,
						USB_DT_CS_INTERFACE,
						(void **)&union_desc)) >= 0) {
			if (union_desc->bDescriptorSubType ==
			    USB_CDC_UNION_TYPE) {
				union_desc->bMasterInterface0 =
				    dev->active_gadget->assigned_intf_start;
				union_desc->bSlaveInterface0 =
				    dev->active_gadget->assigned_intf_start + 1;
			}

			if (union_desc->bDescriptorSubType ==
			    USB_CDC_CALL_MANAGEMENT_TYPE) {
				((struct usb_cdc_call_mgmt_descriptor *)
				 union_desc)->bDataInterface =
dev->active_gadget->assigned_intf_start + 1;
			}
			config_desc_len -= ((unsigned)union_desc -
					    (unsigned)config_desc +
					    union_desc->bLength);
			config_desc = (struct usb_config_descriptor *)
			    ((unsigned)union_desc + union_desc->bLength);
		}
	}
}

#endif

/* After driver is bound, send a fake get configuration command to
 * gadget driver to get the configuration information */
static int gadget_get_config_desc(struct pxa3xx_udc *dev)
{
	struct usb_ctrlrequest req;
	struct usb_config_descriptor *config_desc;
	struct usb_interface_descriptor *interface_desc;
	unsigned config;
#ifndef CONFIG_USB_COMPOSITE
	unsigned interface, alternate;
#endif
	int i;

	DMSG("----------%s------------\n", __FUNCTION__);
	req.bRequestType = USB_RECIP_DEVICE | USB_DIR_IN;
	req.bRequest = USB_REQ_GET_DESCRIPTOR;
	req.wValue = (USB_DT_CONFIG << 8);
	req.wIndex = 0;
	req.wLength = MAX_CONFIG_LENGTH;

	dev->ep0state = EP0_IN_FAKE;
	i = dev->driver->setup(&dev->gadget, &req);

#ifndef CONFIG_USB_COMPOSITE
	config_desc = (struct usb_config_descriptor *)dev->configs;
#else
	config_desc =
	    (struct usb_config_descriptor *)dev->active_gadget->config_desc;
#endif
	if (config_desc->bDescriptorType == USB_DT_CONFIG) {
		config = config_desc->bConfigurationValue;
	} else {
		DMSG("wrong configuration\n");
		return -EFAULT;
	}

#ifdef CONFIG_USB_COMPOSITE
	{
		__u8 num_itfs = config_desc->bNumInterfaces;
		__u8 ep_num = dev->active_gadget->ep_start, cur_intf =
		    0, last_intf = 0;
		struct usb_config_descriptor *p_config_desc = config_desc;
		int config_desc_length = config_desc->wTotalLength, ret;

		dev->active_gadget->assigned_intf_start = dev->interface_count;
		dev->active_gadget->config = config;
		dev->active_gadget->num_intfs = num_itfs;

		/* get every interface desc, fill the gadget_driver_info structure */
		for (i = 0; i < num_itfs; i++) {

			while (config_desc_length >= 0) {
				if ((ret =
				     get_extra_descriptor((char *)p_config_desc,
							  config_desc_length,
							  USB_DT_INTERFACE,
							  (void **)
							  &interface_desc)) >=
				    0) {
					cur_intf =
					    interface_desc->bInterfaceNumber;

					config_desc_length -=
					    (u32) interface_desc -
					    (u32) p_config_desc;

					if (cur_intf != last_intf) {
						p_config_desc =
						    (struct
						     usb_config_descriptor *)
						    interface_desc;
						goto next_intf;
					}

					/* set interface number to assigned one */
					interface_desc->bInterfaceNumber =
					    dev->active_gadget->
					    assigned_intf_start + i;

#ifdef MULTI_P4
					/* set string desc id */
					{
						struct t_str_id *pStr =
						    dev->str_id;
						int j = 1;
						if (interface_desc->iInterface) {
							if (pStr) {
								j++;
								while (pStr->
								       next) {
									pStr =
									    pStr->
									    next;
									j++;
								}
								pStr->next =
								    kmalloc
								    (sizeof
								     (*pStr),
								     GFP_KERNEL);
								pStr =
								    pStr->next;
							} else {
								pStr =
								    kmalloc
								    (sizeof
								     (*pStr),
								     GFP_KERNEL);
								dev->str_id =
								    pStr;
								j = 1;
							}
							pStr->driver_info =
							    dev->active_gadget;
							pStr->str_id =
							    interface_desc->
							    iInterface;
							pStr->next = NULL;
							interface_desc->
							    iInterface = j;
						}
					}
#else
					interface_desc->iInterface = 0;
#endif

					/* search eps and fill the pxa3xx_ep_config struct */
					if (interface_desc->bNumEndpoints) {
						set_eps(interface_desc->
							bNumEndpoints, ep_num,
							(struct
							 usb_endpoint_descriptor
							 *)interface_desc,
							config_desc_length,
							config, cur_intf,
							interface_desc->
							bAlternateSetting);
					}

				} else {
					goto next_intf;
				}	/* if */

				p_config_desc = (struct usb_config_descriptor *)
				    ((struct usb_interface_descriptor *)
				     interface_desc + 1);

				config_desc_length -= interface_desc->bLength;	/* yfw */
			}	/* while */

		      next_intf:
			last_intf = cur_intf;
			dev->interface_count++;
		}

		/* set CDC union descriptors */
		set_cdc_desc();

		return 0;
	}
#else				/*  CONFIG_USB_COMPOSITE */

	if (get_extra_descriptor((char *)config_desc,
				 config_desc->wTotalLength,
				 USB_DT_INTERFACE,
				 (void **)&interface_desc) >= 0) {

		/*interface_desc = (struct usb_interface_descriptor*)(dev->configs +
		   config_desc->bLength); */
		interface = interface_desc->bInterfaceNumber;
		alternate = interface_desc->bAlternateSetting;
	} else {
		DMSG("interface config not find\n");
		return -EFAULT;
	}

	for (i = 1; i < UDC_EP_NUM; i++) {
		dev->ep[i].config = config;
		dev->ep[i].interface = interface;
		dev->ep[i].aisn = alternate;
	}

	return 0;
#endif				/*  CONFIG_USB_COMPOSITE */
}

/* eps_config
 * fill the ep config table
 */
static int eps_config(struct pxa3xx_udc *dev)
{
	struct pxa3xx_ep *ep;
	uint32_t udccr;
	int i;

	for (i = 1; i < UDC_EP_NUM; i++) {
		if (!dev->ep[i].assigned)
			continue;

		udccr = 0;

		ep = &dev->ep[i];
#ifdef CONFIG_USB_COMPOSITE
		udccr |= ((((struct usb_config_descriptor *)
			    dev->first_gadget->config_desc)->
			   bConfigurationValue)
			  << UDCCONR_CN_S) & UDCCONR_CN;
		udccr |= (ep->assigned_interface << UDCCONR_IN_S) & UDCCONR_IN;
#else
		udccr |= (ep->config << UDCCONR_CN_S) & UDCCONR_CN;
		udccr |= (ep->interface << UDCCONR_IN_S) & UDCCONR_IN;
#endif
		udccr |= (ep->aisn << UDCCONR_AISN_S) & UDCCONR_AISN;
		udccr |= (ep->ep_num << UDCCONR_EN_S) & UDCCONR_EN;
		udccr |= (ep->ep_type << UDCCONR_ET_S) & UDCCONR_ET;
		udccr |= (ep->dir_in) ? UDCCONR_ED : 0;
		udccr |= (ep->fifo_size << UDCCONR_MPS_S) & UDCCONR_MPS;
		udccr |= UDCCONR_DE | UDCCONR_EE;

		*ep->reg_udccr = udccr;
	}

	return 0;
}

#ifdef CONFIG_USB_COMPOSITE
static void stop_activity(struct pxa3xx_udc *dev,
			  struct gadget_driver_info *pInfo);
#endif

/* when a driver is successfully registered, it will receive
 * control requests including set_configuration(), which enables
 * non-control requests.  then usb traffic follows until a
 * disconnect is reported.  then a host may connect again, or
 * the driver might get unbound.
 */
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct pxa3xx_udc *dev = the_controller;
	int retval;
#if VERBOSE
	DMSG("dev=0x%x, driver=0x%x, speed=%d,"
	     "bind=0x%x, unbind=0x%x, disconnect=0x%x, setup=0x%x\n",
	     (unsigned)dev, (unsigned)driver, driver->speed,
	     (unsigned)driver->bind, (unsigned)driver->unbind,
	     (unsigned)driver->disconnect, (unsigned)driver->setup);
#endif
#ifndef CONFIG_USB_COMPOSITE
	if (!driver || driver->speed != USB_SPEED_FULL
	    || !driver->bind
	    || !driver->unbind || !driver->disconnect || !driver->setup)
		return -EINVAL;
	if (!dev)
		return -ENODEV;
	if (dev->driver)
		return -EBUSY;
#else
	struct gadget_driver_info *pInfo = dev->first_gadget;

	if (!driver || driver->speed != USB_SPEED_FULL
	    || !driver->bind
	    || !driver->unbind || !driver->disconnect || !driver->setup)
		return -EINVAL;
	if (!dev)
		return -ENODEV;

	out_d0cs = 1;

#ifndef CONFIG_MULTIPLE_GADGET_DRIVERS
	if (dev->driver)
		return -EBUSY;
#else
	/* FIXME remove all modules before insert again */
	if ((dev->rm_flag) && pInfo) {
		printk(KERN_ERR "left modules may not work!  "
		       "please remove all and insert again!!!\n");
		return -EBUSY;
	}
#endif
	local_irq_disable_nort();
	spin_lock_rt(&dev->lock);

	udc_disable(dev);
	while (pInfo) {
		set_gadget_data(&dev->gadget, pInfo->driver_data);
		stop_activity(dev, pInfo);
		pInfo = pInfo->next;
	}
	local_irq_enable_nort();
	spin_unlock_rt(&dev->lock);

	gadget_info_init(driver);
#endif

#ifdef ENABLE_CABLE_DETECT
	mhn_pmic_set_pump(1);
#endif

	pxa_set_cken(CKEN_UDC, 1);
	/* first hook up the driver ... */
	dev->driver = driver;
	dev->gadget.dev.driver = &driver->driver;

	device_add(&dev->gadget.dev);
	retval = driver->bind(&dev->gadget);
#ifdef CONFIG_USB_COMPOSITE
	dev->active_gadget->driver_data = get_gadget_data(&dev->gadget);
#endif
	if (retval) {
		DMSG("bind to driver %s --> error %d\n",
		     driver->driver.name, retval);
		device_del(&dev->gadget.dev);

		dev->driver = 0;
		dev->gadget.dev.driver = 0;
		return retval;
	}
	device_create_file(dev->dev, &dev_attr_function);

#ifdef MULTI_P3
	gadget_get_device_desc();
#endif				/* MULTI_P3 */

	gadget_get_config_desc(dev);

	eps_config(dev);

	/* ... then enable host detection and ep0; and we're ready
	 * for set_configuration as well as eventual disconnect.
	 * NOTE:  this shouldn't power up until later.
	 */
	DMSG("registered gadget driver '%s'\n", driver->driver.name);
	/* If OTG is enabled, USB client should be open all the way */
#if defined(ENABLE_CABLE_DETECT) || defined(CONFIG_USB_OTG)
	if (is_cable_attached())
		if (is_below_624(NULL, 1) || d0cs)
			ipm_notify();
	udc_enable(dev);
#else
	udc_enable(dev);
#endif

#ifdef CONFIG_USB_OTG
	otg_set_peripheral(dev->transceiver, &dev->gadget);
#endif
	out_d0cs = 0;
	dump_state(dev);
	return 0;
}

EXPORT_SYMBOL(usb_gadget_register_driver);

static void
#ifndef CONFIG_USB_COMPOSITE
stop_activity(struct pxa3xx_udc *dev, struct usb_gadget_driver *driver)
#else
stop_activity(struct pxa3xx_udc *dev, struct gadget_driver_info *pInfo)
#endif
{
	int i;

	DMSG("Trace path 1\n");
	/* don't disconnect drivers more than once */
#ifndef CONFIG_USB_COMPOSITE
	if (dev->gadget.speed == USB_SPEED_UNKNOWN)
		driver = 0;
#endif
	dev->gadget.speed = USB_SPEED_UNKNOWN;

	/* prevent new request submissions, kill any outstanding requests  */
	for (i = 0; i < UDC_EP_NUM; i++) {
		struct pxa3xx_ep *ep = &dev->ep[i];

		ep->stopped = 1;
		nuke(ep, -ESHUTDOWN);
	}
	del_timer_sync(&dev->timer);

#ifndef CONFIG_USB_COMPOSITE
	/* report disconnect; the driver is already quiesced */
	if (driver)
		driver->disconnect(&dev->gadget);
#else
	if (!pInfo->stopped)
		pInfo->driver->disconnect(&dev->gadget);
	pInfo->stopped = 1;
#endif
	/* re-init driver-visible data structures */
	udc_reinit(dev);
}

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	int i;
	struct pxa3xx_udc *dev = the_controller;
#ifdef CONFIG_USB_COMPOSITE
	struct gadget_driver_info *pInfo = dev->first_gadget;
	struct gadget_driver_info *info = NULL;
#ifdef MULTI_P4
	struct t_str_id *pStr = dev->str_id, *str;
#endif

	if (!dev)
		return -ENODEV;

	out_d0cs = 1;

	do {
		/* find the gadget driver info to pInfo */
		if (pInfo->driver == driver) {
			/* for the first driver is being removed */
			if (!info)
				info = pInfo->next;
			break;
		}
		/* save the previous one */
		info = pInfo;
		pInfo = pInfo->next;
	} while (pInfo);

	if (NULL == pInfo) {
		printk(KERN_ERR "%s, can't find driver!\n", __FUNCTION__);
		return -EINVAL;
	}
#ifdef CONFIG_USB_OTG
	if (dev->driver_count == 1) {
		otg_set_peripheral(dev->transceiver, NULL);
	}
#endif

	local_irq_disable_nort();
	spin_lock_rt(&dev->lock);

	if (dev->driver_count == 1)
		udc_disable(dev);
	set_gadget_data(&dev->gadget, pInfo->driver_data);
	stop_activity(dev, pInfo);

	local_irq_enable_nort();
	spin_unlock_rt(&dev->lock);

	driver->unbind(&dev->gadget);

	/* put the active one to the previous one */
	if (dev->first_gadget == pInfo) {
		if (info)
			dev->first_gadget = info;
		else
			dev->first_gadget = pInfo->next;
	}
	if (dev->active_gadget == pInfo) {
		if (info)
			dev->active_gadget = info;
		else
			dev->active_gadget = pInfo->next;
	}
	if ((info) && (info != pInfo->next))
		info->next = pInfo->next;

	if (dev->active_gadget)
		dev->driver = dev->active_gadget->driver;
	else
		dev->driver = 0;	/* no drivers left */

	/* del the gadget abstract device */
	if (dev->driver_count == 1) {
		device_del(&dev->gadget.dev);
		device_remove_file(dev->dev, &dev_attr_function);
	}

	for (i = pInfo->ep_start; i < (pInfo->ep_start + pInfo->num_eps); i++) {
		struct pxa3xx_ep *ep = &dev->ep[i];
		ep->assigned = 0;
		ep->desc = NULL;
		kfree(ep->ep.name);
	}

	/* free the gadget_driver_info struct */
	kfree(pInfo);

#ifdef MULTI_P4
	while (pStr) {
		str = pStr;
		pStr = str->next;
		kfree(str);
	}
	dev->str_id = NULL;
#endif

	dev->driver_count--;
	num_ep_used -= pInfo->num_eps;
	dev->interface_count -= pInfo->num_intfs;

	memset(dev->configs, 0, MAX_CONFIG_LENGTH);
	if (dev->driver_count != 0) {
		dev->rm_flag = 1;
		printk(KERN_ERR "left modules may not work!  "
		       "please remove all and insmod again!!!\n");
	} else {
		dev->rm_flag = 0;
	}

#else

	if (!dev)
		return -ENODEV;
	if (!driver || driver != dev->driver)
		return -EINVAL;

#ifdef CONFIG_USB_OTG
	otg_set_peripheral(dev->transceiver, NULL);
#endif

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

	for (i = 1; i < UDC_EP_NUM; i++) {
		struct pxa3xx_ep *ep = &dev->ep[i];
		ep->assigned = 0;
		kfree(ep->ep.name);
	}
#endif

#ifdef ENABLE_CABLE_DETECT
	connected = 0;
	mhn_pmic_set_pump(0);
#endif
	out_d0cs = 0;
	DMSG("unregistered gadget driver '%s'\n", driver->driver.name);
	dump_state(dev);
	return 0;
}

EXPORT_SYMBOL(usb_gadget_unregister_driver);

#ifndef	enable_disconnect_irq
#define	enable_disconnect_irq()		do {} while (0)
#define	disable_disconnect_irq()	do {} while (0)
#endif

/*-------------------------------------------------------------------------*/

static inline void clear_ep_state(struct pxa3xx_udc *dev)
{
	unsigned i;

	/* hardware SET_{CONFIGURATION,INTERFACE} automagic resets endpoint
	 * fifos, and pending transactions mustn't be continued in any case.
	 */
	for (i = 1; i < UDC_EP_NUM; i++)
		nuke(&dev->ep[i], -ECONNABORTED);
}

static void udc_watchdog(unsigned long _dev)
{
	struct pxa3xx_udc *dev = (void *)_dev;
	unsigned long flags;

	local_irq_save_nort(flags);
	spin_lock_rt(&dev->lock);

	if (dev->ep0state == EP0_STALL
	    && (UDCCSR0 & UDCCSR0_FST) == 0 && (UDCCSR0 & UDCCSR0_SST) == 0) {
		UDCCSR0 =
		    UDCCSR0_ACM | UDCCSR0_FST | UDCCSR0_FTF | UDCCSR0_ODFCLR;
		DBG(DBG_VERBOSE, "ep0 re-stall\n");
		start_watchdog(dev);
	}
	local_irq_restore_nort(flags);
	spin_unlock_rt(&dev->lock);
}

#ifdef CONFIG_USB_COMPOSITE
/* string desc not supported yet */
#define UDC_STRING_MANUFACTURER	0
#define UDC_STRING_PRODUCT		0

static struct usb_device_descriptor udc_device_desc = {
	.bLength = sizeof udc_device_desc,
	.bDescriptorType = USB_DT_DEVICE,

	.bcdUSB = __constant_cpu_to_le16(0x0110),
	/*  USB_CLASS_COMM, for rndis */
	.bDeviceClass = USB_CLASS_PER_INTERFACE,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 0x10,	/*  for mhn */
	.idVendor = __constant_cpu_to_le16(UDC_VENDOR_NUM),
	.idProduct = __constant_cpu_to_le16(UDC_PRODUCT_NUM),
	.iManufacturer = UDC_STRING_MANUFACTURER,
	.iProduct = UDC_STRING_PRODUCT,
	.bNumConfigurations = 1,
};

static void udc_setup_complete(struct usb_ep *ep, struct usb_request *req)
{
	if (req->status || req->actual != req->length)
		DMSG("pseudo setup complete --> %d, %d/%d\n",
		     req->status, req->actual, req->length);
}

static int udc_do_request(struct usb_ctrlrequest *ctrl, struct pxa3xx_ep *ep)
{
	struct pxa3xx_udc *dev = the_controller;
	struct usb_request *usb_req = &dev->ep0_req.req;
	int value = -EOPNOTSUPP;
	int pseudo = 0, ret = 0;

	switch (ctrl->bRequest) {

	case USB_REQ_GET_DESCRIPTOR:
		if (ctrl->bRequestType != USB_DIR_IN)
			break;
		switch (ctrl->wValue >> 8) {

		case USB_DT_DEVICE:
			DMSG("%s, get device desc\n", __FUNCTION__);
			pseudo = 1;
			/* send the pseudo device desc */
			value =
			    min(ctrl->wLength, (u16) sizeof udc_device_desc);
			memcpy(usb_req->buf, &udc_device_desc, value);
			break;

		case USB_DT_CONFIG:
			DMSG("%s, get conf desc\n", __FUNCTION__);
			pseudo = 1;
			/* send the pseudo configuration desc */
			value = combine_configuration();
			value = min((int)ctrl->wLength, value);
			memcpy(usb_req->buf, &dev->configs, value);
			break;

		default:
			break;
		}
	default:
		break;
	}

	if (pseudo) {
		usb_req->length = value;
		usb_req->no_interrupt = 0;
		usb_req->zero = value < ctrl->wLength
		    && (value % ep->ep.maxpacket) == 0;
		usb_req->complete = udc_setup_complete;

		ret = pxa3xx_ep_queue(&ep->ep, usb_req, GFP_KERNEL);
		if (!(ret == 0)) {
			DMSG("%s, ep_queue error = 0x%x", __FUNCTION__, ret);
		}
		return value;
	} else {
		return -1;
	}
}
#endif

static void handle_ep0(struct pxa3xx_udc *dev)
{
	u32 udccsr0 = UDCCSR0;
	struct pxa3xx_ep *ep = &dev->ep[0];
	struct pxa3xx_request *req;
	union {
		struct usb_ctrlrequest r;
		u8 raw[8];
		u32 word[2];
	} u;

	DMSG("%s is called, ep0 state:%d\n", __FUNCTION__, dev->ep0state);
	if (list_empty(&ep->queue))
		req = 0;
	else
		req = list_entry(ep->queue.next, struct pxa3xx_request, queue);

	/* clear stall status */
	if (udccsr0 & UDCCSR0_SST) {
		nuke(ep, -EPIPE);
		UDCCSR0 = UDCCSR0_ACM | UDCCSR0_SST | UDCCSR0_ODFCLR;
		del_timer(&dev->timer);
		ep0_idle(dev);
	}

	/* previous request unfinished?  non-error iff back-to-back ... */
	if ((udccsr0 & UDCCSR0_SA) != 0 && dev->ep0state != EP0_IDLE) {
		DMSG("handle_ep0: Setup command again\n");
		nuke(ep, 0);
		del_timer(&dev->timer);
		ep0_idle(dev);
	}

	switch (dev->ep0state) {
	case EP0_NO_ACTION:
		printk(KERN_INFO "%s: Busy\n", __FUNCTION__);
		/*Fall through */
	case EP0_IDLE:
		/* late-breaking status? */
		udccsr0 = UDCCSR0;
		DMSG("%s EP0_IDLE udccsr0:0x%08x\n", __FUNCTION__, udccsr0);
		/* start control request? */
		if (likely((udccsr0 & (UDCCSR0_OPC | UDCCSR0_SA | UDCCSR0_RNE))
			   == (UDCCSR0_OPC | UDCCSR0_SA | UDCCSR0_RNE))) {
			int i;

			nuke(ep, -EPROTO);
			/* read SETUP packet */
			for (i = 0; i < 2; i++) {
				if (unlikely(!(UDCCSR0 & UDCCSR0_RNE))) {
				      bad_setup:
					DMSG("SETUP %d!\n", i);
					goto stall;
				}
				u.word[i] = UDCDR0;
			}
			if (unlikely((UDCCSR0 & UDCCSR0_RNE) != 0))
				goto bad_setup;

			le16_to_cpus(&u.r.wValue);
			le16_to_cpus(&u.r.wIndex);
			le16_to_cpus(&u.r.wLength);

			LED_EP0_ON;

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

			if (u.r.wLength == 0)
				dev->ep0state = EP0_IN_DATA_PHASE;

#ifdef CONFIG_USB_COMPOSITE
			/* when only one driver is registered, do as original */
			if (dev->driver_count == 1) {
				i = dev->driver->setup(&dev->gadget, &u.r);
			} else if (dev->driver_count <= 0) {
				printk(KERN_ERR "%s, error: "
				       "dev->driver_count = %d\n", __FUNCTION__,
				       dev->driver_count);
				return;
			} else {
				struct gadget_driver_info *pInfo =
				    dev->active_gadget;
				struct gadget_driver_info *pCurInfo =
				    dev->active_gadget;

				i = udc_do_request(&u.r, ep);

				/* class specfic requests needed to be set up */
				if (i < 0) {
					if ((u.r.
					     bRequestType & USB_RECIP_MASK) ==
					    USB_RECIP_INTERFACE) {
						for (i = 1; i < num_ep_used;
						     i++) {
							if (u.r.wIndex ==
							    dev->ep[i].
							    assigned_interface)
							{
								u.r.wIndex =
								    dev->ep[i].
								    interface;
								if (i)
									pInfo =
									    dev->
									    ep
									    [i].
									    driver_info;
								if (pInfo ==
								    NULL) {
									printk
									    (KERN_ERR
									     "driver not found, wrong req!!!\n");
									pInfo =
									    pCurInfo;
								}
								set_gadget_data
								    (&dev->
								     gadget,
								     pInfo->
								     driver_data);
								break;
							}
						}
					}
					if ((u.r.
					     bRequestType & USB_RECIP_MASK) ==
					    USB_RECIP_ENDPOINT) {
						i = u.r.wIndex & 0xf;
						if (i)
							pInfo =
							    dev->ep[i].
							    driver_info;
						if (pInfo == NULL) {
							printk(KERN_ERR
							       "driver not found, wrong req!!!\n");
							pInfo = pCurInfo;
						}
						set_gadget_data(&dev->gadget,
								pInfo->
								driver_data);
					}
#ifdef CONFIG_MULTIPLE_GADGET_DRIVERS	/*   FIXME */
					if (((u.r.bRequestType == 0x21)
					     && (u.r.bRequest == 0x00))
					    || ((u.r.bRequestType == 0xa1)
						&& (u.r.bRequest == 0x01))) {

						pInfo = dev->first_gadget;
						do {
							if (pInfo) {
								if (strcmp
								    (pInfo->
								     driver->
								     driver.
								     name,
								     "ether") ==
								    0) {
									break;
								}
							}
							pInfo = pInfo->next;
						} while (pInfo);
						if (pInfo == NULL) {
							printk(KERN_ERR
							       "%s, eth not found????\n",
							       __func__);
							goto stall;
						} else {
							set_gadget_data(&dev->
									gadget,
									pInfo->
									driver_data);
						}
					}

					if (((u.r.bRequestType == 0xa1)
					     && (u.r.bRequest == 0xfe))
					    || ((u.r.bRequestType == 0x21)
						&& (u.r.bRequest == 0xff))) {
						pInfo = dev->first_gadget;
						while ((strcmp
							(pInfo->driver->driver.
							 name, "g_file_storage")
							&& pInfo)) {
							pInfo = pInfo->next;
						}
						if (pInfo == NULL) {
							printk(KERN_ERR
							       "%s, mass not found????\n",
							       __func__);
							goto stall;
						} else
							set_gadget_data(&dev->
									gadget,
									pInfo->
									driver_data);
					}

					if (((u.r.bRequestType == 0xa1)
					     && (u.r.bRequest == 0x21))
					    || ((u.r.bRequestType == 0x21)
						&& (u.r.bRequest == 0x20))
					    || ((u.r.bRequestType == 0x21)
						&& (u.r.bRequest == 0x22))) {
						pInfo = dev->first_gadget;
						while ((strcmp
							(pInfo->driver->driver.
							 name, "g_serial")
							&& pInfo)) {
							pInfo = pInfo->next;
						}
						if (pInfo == NULL) {
							printk(KERN_ERR
							       "%s, serial not found????\n",
							       __func__);
							goto stall;
						} else
							set_gadget_data(&dev->
									gadget,
									pInfo->
									driver_data);
					}
#endif

					/* for string desc for each interface, not supported yet */
					if ((USB_REQ_GET_DESCRIPTOR ==
					     u.r.bRequestType)
					    && (USB_DT_STRING ==
						(u.r.wValue >> 8))) {
#ifdef MULTI_P4
						struct t_str_id *pStr =
						    dev->str_id;
						int id_num =
						    u.r.wValue & 0xff, j;
						for (j = 1; j < id_num; j++) {
							if (pStr)
								pStr =
								    pStr->next;
							else {
								printk(KERN_ERR
								       " string %d not find !\n",
								       id_num);
							}
						}
						pInfo = pStr->driver_info;
						set_gadget_data(&dev->gadget,
								pInfo->
								driver_data);
						u.r.wValue = pStr->str_id;
					}
#else
						DMSG("  get string desc, %d, unsupported, " "will pass to current driver(info %p)\n", u.r.wValue, pInfo);
					}
#endif
					i = pInfo->driver->setup(&dev->gadget,
								 &u.r);

					if (i < 0) {
						pInfo = dev->first_gadget;
						do {
							set_gadget_data(&dev->
									gadget,
									pInfo->
									driver_data);
							i = pInfo->driver->
							    setup(&dev->gadget,
								  &u.r);
							pInfo = pInfo->next;
						} while ((i == -EOPNOTSUPP)
							 && (pInfo));
						if (i == -EOPNOTSUPP)
							DMSG("%s, no correct driver found to respond to the req!!!\n", __FUNCTION__);
						set_gadget_data(&dev->gadget,
								pCurInfo->
								driver_data);
					}	/* if(i) */
				}	/* if(!i) */
			}	/* if(dev->driver_count == 1)  */
#else
			i = dev->driver->setup(&dev->gadget, &u.r);
#endif

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
				LED_EP0_OFF;

				/* deferred i/o == no response yet */
			} else if (dev->req_pending) {
				if (likely(dev->ep0state == EP0_IN_DATA_PHASE
					   || dev->req_std || u.r.wLength))
					ep0start(dev, 0, "defer");
				else
					/* Wait for client to send 0 length ep0 request */
/* 					ep0start(dev, UDCCSR0_IPR, "defer/IPR"); */
					ep0start(dev, 0, "defer/IPR");
			}

			/* expect at least one data or status stage irq */
			return;

		} else {
			/* some random early IRQ:
			 * - we acked FST
			 * - IPR cleared
			 * - OPC got set, without SA (likely status stage)
			 */
			UDCCSR0 =
			    udccsr0 & (UDCCSR0_ACM | UDCCSR0_SA | UDCCSR0_OPC |
				       UDCCSR0_ODFCLR);
		}
		break;
	case EP0_IN_DATA_PHASE:	/* GET_DESCRIPTOR etc */
		if (udccsr0 & UDCCSR0_OPC) {
			UDCCSR0 =
			    UDCCSR0_ACM | UDCCSR0_OPC | UDCCSR0_FTF |
			    UDCCSR0_ODFCLR;
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
				if (read_ep0_fifo(ep, req)) {
					UDCCSR0 = UDCCSR0_IPR | UDCCSR0_ODFCLR;	/* RNDIS */
					done(ep, req, 0);
				}
				/* else more OUT packets expected */
			}	/* else OUT token before read was issued */
		} else {	/* irq was IPR clearing */

			DBG(DBG_VERBOSE, "ep0out premature status\n");
			if (req)
				done(ep, req, 0);
			ep0_idle(dev);
		}
		break;
	case EP0_STALL:
		UDCCSR0 = UDCCSR0_ACM | UDCCSR0_FST | UDCCSR0_ODFCLR;
		break;
	case EP0_IN_FAKE:
		printk(KERN_ERR "%s: impossible come here\n", __FUNCTION__);
		break;
	}
/* 	UDCISR0 = UDCISR_INT(0, UDCISR_INT_MASK); */
}

u32 status_reg_UDCISR1;
u32 status_reg_UDCISR0;
u32 status_reg_UDCOTGISR;

static void handle_ep(struct pxa3xx_ep *ep)
{
	struct pxa3xx_request *req, *req_next;
	int completed, length, remain;
	u32 udccsr = 0;

	DMSG("%s is called, ep num:%d, in:%d\n", __FUNCTION__, ep->ep_num,
	     ep->dir_in);
	do {
		completed = 0;
		if (likely(!list_empty(&ep->queue))) {
			req = list_entry(ep->queue.next,
					 struct pxa3xx_request, queue);
		} else
			req = 0;

		DMSG("%s: req:%p, udcisr0:0x%x udccsr %p:0x%x\n", __FUNCTION__,
		     req, status_reg_UDCISR0, ep->reg_udccsr, *ep->reg_udccsr);
		if (unlikely(ep->dir_in)) {
			udccsr = (UDCCSR_SST | UDCCSR_TRN) & *ep->reg_udccsr;
			if (unlikely(udccsr))
				*ep->reg_udccsr = udccsr;

			if (req && likely((*ep->reg_udccsr & UDCCSR_FS) != 0))
				completed = write_fifo(ep, req);

		} else {	/*  for out endpoints */
			udccsr = (UDCCSR_SST | UDCCSR_TRN) & *ep->reg_udccsr;
			if (unlikely(udccsr))
				*ep->reg_udccsr = udccsr;
#ifdef	CONFIG_USB_PXA3XX_DMA
			/* DMA enabled, Short packet received, transfer completed */
			if (req && (ep->dma > 0)
			    && (*ep->reg_udccsr & UDCCSR_SP)) {
				DMSG("dcsr:%x, dcmd:%x\n", DCSR(ep->dma),
				     DCMD(ep->dma));
				DCSR(ep->dma) &= ~DCSR_RUN;
				remain = DCMD(ep->dma) & DCMD_LENGTH;
				length =
				    req->req.length - req->req.actual - remain;
				DMSG("%s, buf:%p, actual:%d,length:%d\n",
				     __FUNCTION__, req->req.buf,
				     req->req.actual, length);
				memcpy((char *)req->req.buf + req->req.actual,
				       ep->dma_buffer_virt, length);
				req->req.actual += length;
				*ep->reg_udccsr = UDCCSR_PC | UDCCSR_DME;

				if (req->queue.next != &ep->queue) {
					req_next = list_entry(req->queue.next,
							      struct
							      pxa3xx_request,
							      queue);
					kick_dma(ep, req_next);
				}
				done(ep, req, 0);
				/* fifos can hold packets, ready for reading... */
			} else
#endif
			if (likely(req)) {
				completed = read_fifo(ep, req);
			} else {
				pio_irq_disable(ep->ep_num);
				*ep->reg_udccsr = UDCCSR_FEF | UDCCSR_PC;
				DMSG("%s: no req for out data\n", __FUNCTION__);
			}
			if(completed)
			    done(ep, req, 0);
		}
		ep->pio_irqs++;
	} while (completed);
}

static void pxa3xx_change_configuration(struct pxa3xx_udc *dev)
{
	struct usb_ctrlrequest req;

	req.bRequestType = 0;
	req.bRequest = USB_REQ_SET_CONFIGURATION;
	req.wValue = dev->configuration;
	req.wIndex = 0;
	req.wLength = 0;

	dev->ep0state = EP0_IN_DATA_PHASE;

#ifndef CONFIG_USB_COMPOSITE
	dev->req_config = 1;
	dev->driver->setup(&dev->gadget, &req);
#else				/* CONFIG_USB_COMPOSITE */
	{
		struct gadget_driver_info *pInfo = dev->first_gadget;

		dev->req_config = dev->driver_count;
		do {
			set_gadget_data(&dev->gadget, pInfo->driver_data);
#ifndef MULTIPLE_CONFIGURATION
			/*  switch to gadget driver's configuration */
			req.wValue = pInfo->config;
#endif
			pInfo->driver->setup(&dev->gadget, &req);
			pInfo->stopped = 0;
			pInfo = pInfo->next;
		} while (pInfo);
	}
#endif				/* CONFIG_USB_COMPOSITE */
}

static void pxa3xx_change_interface(struct pxa3xx_udc *dev)
{
	struct usb_ctrlrequest req;

	DMSG("%s\n", __FUNCTION__);

	req.bRequestType = USB_RECIP_INTERFACE;
	req.bRequest = USB_REQ_SET_INTERFACE;
	req.wValue = dev->alternate;
	req.wIndex = dev->interface;
	req.wLength = 0;
#ifdef CONFIG_USB_COMPOSITE
	{
		/* change the assigned interface to gadget interface */
		int active_interface = (UDCCR & 0x700) >> 8;
		struct gadget_driver_info *pInfo = NULL;
		int ret, i;

		for (i = 1; i < num_ep_used; i++) {
			if (dev->ep[i].assigned_interface == active_interface) {
				pInfo = dev->ep[i].driver_info;
				req.wIndex = dev->ep[i].interface;
				break;
			}
		}

		if (pInfo == NULL) {
			printk(KERN_ERR "active interface not found, error\n");
		} else {
			dev->driver = pInfo->driver;
			dev->gadget.dev.driver =
			    &((struct usb_gadget_driver *)(pInfo->driver))->
			    driver;

			dev->active_gadget = pInfo;

			set_gadget_data(&dev->gadget,
					dev->active_gadget->driver_data);

			/* req.wValue = dev->ep[i].interface; */
			/* req.wIndex = dev->ep[i]->aisn; */
			dev->interface = active_interface;

			dev->ep0state = EP0_IN_DATA_PHASE;
			ret = dev->driver->setup(&dev->gadget, &req);
			if (ret == -EOPNOTSUPP) {
				DMSG(" ret EOPNOTSUPP\n");
			}
			UDCCSR0 = UDCCSR0_ACM | UDCCSR0_AREN | UDCCSR0_ODFCLR;
		}
		return;
	}

#else				/* CONFIG_USB_COMPOSITE */

	dev->ep0state = EP0_IN_DATA_PHASE;
	dev->driver->setup(&dev->gadget, &req);

	UDCCSR0 = UDCCSR0_ACM | UDCCSR0_AREN | UDCCSR0_ODFCLR;	/* xj add */

#endif				/* CONFIG_USB_COMPOSITE */
}

/*
 *	pxa3xx_udc_irq - interrupt handler
 *
 * avoid delays in ep0 processing. the control handshaking isn't always
 * under software control (pxa250c0 and the pxa255 are better), and delays
 * could cause usb protocol errors.
 */

struct work_struct pxa3xx_udc_work;

static irqreturn_t pxa3xx_udc_irq(int irq, void *_dev, struct pt_regs *r)
{
	int status;

	status_reg_UDCISR0 = UDCISR0;
	status_reg_UDCISR1 = UDCISR1;
	status_reg_UDCOTGISR = UDCOTGISR;

	/* clear all interrupts */
	UDCISR0 = 0xffffffff;
	UDCISR1 = 0xffffffff;
	UDCOTGISR = 0xffffffff;

	status = schedule_work(&pxa3xx_udc_work);

	DMSG("IRQ_HANDLED\n");
	return IRQ_HANDLED;
}

static void pxa3xx_udc_handler(void *data)
{
#ifndef CONFIG_USB_COMPOSITE
	struct pxa3xx_udc *dev = data;
#else
	struct pxa3xx_udc *dev = the_controller;
#endif
	int handled;

	dev->stats.irqs++;
	HEX_DISPLAY(dev->stats.irqs);

	DBG(DBG_VERBOSE, "Interrupt, UDCISR0:0x%08x, UDCISR1:0x%08x, "
	    "UDCCR:0x%08x\n", UDCISR0, UDCISR1, UDCCR);
#ifdef CONFIG_USB_OTG
	DBG(DBG_VERBOSE, "UPO2CR:0x%08x, UDCOTGICR:0x%08x, UDCOTGISR:0x%08x\n",
	    UP2OCR, UDCOTGICR, status_reg_UDCOTGISR);
#endif

	{
		u32 udcir;

#ifdef 	CONFIG_USB_OTG
		if (status_reg_UDCOTGISR) {
			mhnotg_otg_interrupt(dev->transceiver);
		}
#endif
		udcir = status_reg_UDCISR1 & 0xF8000000;
		handled = 0;

		/* SUSpend Interrupt Request */
		if (unlikely(udcir & UDCISR1_IESU)) {
			handled = 1;
			DBG(DBG_VERBOSE, "USB suspend\n");
			if (dev->gadget.speed != USB_SPEED_UNKNOWN
			    && dev->driver && dev->driver->suspend)
#ifndef CONFIG_USB_COMPOSITE
				dev->driver->suspend(&dev->gadget);
#else
			{
				struct gadget_driver_info *pInfo =
				    dev->first_gadget;

				do {
					set_gadget_data(&dev->gadget,
							pInfo->driver_data);
					pInfo->driver->suspend(&dev->gadget);
					pInfo = pInfo->next;
				} while (pInfo);
			}
#endif
			ep0_idle(dev);
#ifdef CONFIG_USB_OTG
			mhnotg_host_suspend(dev->transceiver);
#endif
		}

		/* RESume Interrupt Request */
		if (unlikely(udcir & UDCISR1_IERU)) {
			handled = 1;
			DBG(DBG_VERBOSE, "USB resume\n");

			if (dev->gadget.speed != USB_SPEED_UNKNOWN
			    && dev->driver && dev->driver->resume)
#ifndef CONFIG_USB_COMPOSITE
				dev->driver->resume(&dev->gadget);
#else
			{
				struct gadget_driver_info *pInfo =
				    dev->first_gadget;

				do {
					set_gadget_data(&dev->gadget,
							pInfo->driver_data);
					pInfo->driver->resume(&dev->gadget);
					pInfo = pInfo->next;
				} while (pInfo);
			}
#endif

		}

		if (unlikely(udcir & UDCISR1_IECC)) {
			unsigned config, interface, alternate;

			handled = 1;
			DBG(DBG_VERBOSE, "USB SET_CONFIGURATION or "
			    "SET_INTERFACE command received\n");

			UDCCR |= UDCCR_SMAC;

			config = (UDCCR & UDCCR_ACN) >> UDCCR_ACN_S;
			interface = (UDCCR & UDCCR_AIN) >> UDCCR_AIN_S;
			alternate = (UDCCR & UDCCR_AAISN) >> UDCCR_AAISN_S;

			DBG(DBG_VERBOSE,
			    "    config=%d,  interface=%d, alternate=%d\n",
			    config, interface, alternate);

			if (dev->configuration != config) {
				dev->configuration = config;
				pxa3xx_change_configuration(dev);
#ifndef CONFIG_USB_COMPOSITE
			} else if ((dev->interface != interface) ||
				   (dev->alternate != alternate)) {
#else
			} else if ((dev->interface != interface) ||
				   (dev->interface == 0) ||
				   (dev->alternate != alternate)) {
#endif
				dev->interface = interface;
				dev->alternate = alternate;
				pxa3xx_change_interface(dev);
			} else {
				UDCCSR0 =
				    UDCCSR0_ACM | UDCCSR0_AREN | UDCCSR0_ODFCLR;
			}
			DMSG("%s: con:%d,inter:%d,alt:%d\n",
			     __FUNCTION__, config, interface, alternate);
		}

		/* ReSeT Interrupt Request - USB reset */
		if (unlikely(udcir & UDCISR1_IERS)) {

			handled = 1;

			if ((UDCCR & UDCCR_UDA) == 0) {
				DBG(DBG_VERBOSE, "USB reset start\n");

				/* reset driver and endpoints,
				 * in case that's not yet done
				 */
#ifndef CONFIG_USB_COMPOSITE
				stop_activity(dev, dev->driver);
#else
				{
					struct gadget_driver_info *pInfo =
					    dev->first_gadget;

					do {
						set_gadget_data(&dev->gadget,
								pInfo->
								driver_data);
						stop_activity(dev, pInfo);
						pInfo = pInfo->next;
					} while (pInfo);
				}
#endif

			}
			DMSG("USB reset\n");
			dev->gadget.speed = USB_SPEED_FULL;
			memset(&dev->stats, 0, sizeof dev->stats);
		}

		if (status_reg_UDCISR0 || (status_reg_UDCISR1 & 0xFFFF)) {
			u32 udcisr0 = status_reg_UDCISR0;
			u32 udcisr1 = status_reg_UDCISR1 & 0xFFFF;
			int i;

			DBG(DBG_VERY_NOISY, "irq %02x.%02x\n", udcisr1,
			    udcisr0);

			/* control traffic */
			if (udcisr0 & UDCISR0_IR0) {
				DMSG("handle_ep0: UDCISR0:%x, UDCCSR0:%x\n",
				     status_reg_UDCISR0, UDCCSR0);
				if (udcisr0 & UDC_INT_FIFOERROR) {
					printk(KERN_DEBUG
					       "Endpoint 0 fifo Error\n");
				}
				dev->ep[0].pio_irqs++;
				if (udcisr0 & UDC_INT_PACKETCMP) {
					handle_ep0(dev);
					handled = 1;
				}
			}

			udcisr0 >>= 2;
			/* endpoint data transfers */
			for (i = 1; udcisr0 != 0 && i < 16; udcisr0 >>= 2, i++) {

				if (udcisr0 & UDC_INT_FIFOERROR)
					printk(KERN_WARNING
					       " Endpoint %d Fifo error\n", i);
				if (udcisr0 & UDC_INT_PACKETCMP) {
					handle_ep(&dev->ep[i]);
					handled = 1;
				}
			}

			for (i = 0; udcisr1 != 0 && i < 8; udcisr1 >>= 2, i++) {

				if (udcisr1 & UDC_INT_FIFOERROR) {
					printk(KERN_WARNING
					       " Endpoint %d fifo error\n",
					       (i + 16));
				}

				if (udcisr1 & UDC_INT_PACKETCMP) {
					handle_ep(&dev->ep[i + 16]);
					handled = 1;
				}
			}
		}

		/* we could also ask for 1 msec SOF (SIR) interrupts */

	}
}

static inline void validate_fifo_size(struct pxa3xx_ep *pxa_ep, u8 bmAttributes)
{
	switch (bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) {
	case USB_ENDPOINT_XFER_CONTROL:
		pxa_ep->fifo_size = EP0_FIFO_SIZE;
		break;
	case USB_ENDPOINT_XFER_ISOC:
		pxa_ep->fifo_size = ISO_FIFO_SIZE;
		break;
	case USB_ENDPOINT_XFER_BULK:
		pxa_ep->fifo_size = BULK_FIFO_SIZE;
		break;
	case USB_ENDPOINT_XFER_INT:
		pxa_ep->fifo_size = INT_FIFO_SIZE;
		break;
	default:
		break;
	}
}

static void udc_init_ep(struct pxa3xx_udc *dev)
{
	int i;

	INIT_LIST_HEAD(&dev->gadget.ep_list);
	INIT_LIST_HEAD(&dev->gadget.ep0->ep_list);

	for (i = 0; i < UDC_EP_NUM; i++) {
		struct pxa3xx_ep *ep = &dev->ep[i];

		ep->dma = -1;
		if (i != 0) {
			memset(ep, 0, sizeof(*ep));
		}
		INIT_LIST_HEAD(&ep->queue);
	}
}

#define NAME_SIZE 120

struct usb_ep *pxa3xx_ep_config(struct usb_gadget *gadget,
				struct usb_endpoint_descriptor *desc)
{
	unsigned i;
	char *name;
	struct usb_ep *ep = NULL;
	struct pxa3xx_ep *pxa_ep = NULL;
	struct pxa3xx_udc *dev = the_controller;

	DMSG("pxa3xx_config_ep is called\n");
	DMSG(" usb endpoint descriptor is:\n"
	     "	bLength:%d\n"
	     "	bDescriptorType:%x\n"
	     "	bEndpointAddress:%x\n"
	     "	bmAttributes:%x\n"
	     "	wMaxPacketSize:%d\n",
	     desc->bLength,
	     desc->bDescriptorType, desc->bEndpointAddress,
	     desc->bmAttributes, desc->wMaxPacketSize);

	for (i = 1; i < UDC_EP_NUM; i++) {
		if (!dev->ep[i].assigned) {
			pxa_ep = &dev->ep[i];
			pxa_ep->assigned = 1;
			pxa_ep->ep_num = i;
			break;
		}
	}
	if (unlikely(i == UDC_EP_NUM)) {
		printk(KERN_ERR "%s:Failed to find a spare endpoint\n",
		       __FILE__);
		return NULL;
	}

	ep = &pxa_ep->ep;

	pxa_ep->dev = dev;
	pxa_ep->desc = desc;
	pxa_ep->pio_irqs = pxa_ep->dma_irqs = 0;
	pxa_ep->dma = -1;

	if (!(desc->bEndpointAddress & 0xF))
		desc->bEndpointAddress |= i;

	if (!(desc->wMaxPacketSize)) {
		validate_fifo_size(pxa_ep, desc->bmAttributes);
		desc->wMaxPacketSize = pxa_ep->fifo_size;
	} else
		pxa_ep->fifo_size = desc->wMaxPacketSize;

	pxa_ep->dir_in = (desc->bEndpointAddress & USB_DIR_IN) ? 1 : 0;
	pxa_ep->ep_type = desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
	pxa_ep->stopped = 1;
	pxa_ep->dma_con = 0;
	pxa_ep->reg_udccsr = &UDCCSR0 + i;
	pxa_ep->reg_udcbcr = &UDCBCR0 + i;
	pxa_ep->reg_udcdr = &UDCDR0 + i;
	pxa_ep->reg_udccr = &UDCCRA - 1 + i;
#ifdef	CONFIG_USB_PXA3XX_DMA
	pxa_ep->reg_drcmr = &DRCMR24 + i;
#endif

	/* Fill ep name */
	name = kmalloc(NAME_SIZE, GFP_KERNEL);
	if (!name) {
		printk(KERN_ERR "%s: Error\n", __FUNCTION__);
		return NULL;
	}

	switch (pxa_ep->ep_type) {
	case USB_ENDPOINT_XFER_BULK:
		sprintf(name, "%s-bulk-%s-%d", gadget->dev.driver->name,
			(pxa_ep->dir_in ? "in" : "out"), i);
		break;
	case USB_ENDPOINT_XFER_INT:
		sprintf(name, "%s-interrupt-%s-%d", gadget->dev.driver->name,
			(pxa_ep->dir_in ? "in" : "out"), i);
		break;
	default:
		sprintf(name, "endpoint-%s-%d", (pxa_ep->dir_in ?
						 "in" : "out"), i);
		break;
	}
	ep->name = name;

	ep->ops = &pxa3xx_ep_ops;
	ep->maxpacket = min((ushort) pxa_ep->fifo_size, desc->wMaxPacketSize);

	list_add_tail(&ep->ep_list, &gadget->ep_list);
#ifdef CONFIG_USB_COMPOSITE
	dev->active_gadget->num_eps++;
	num_ep_used++;
	DMSG("		ep->ep_num = 0x%x\n", pxa_ep->ep_num);
	DMSG("		num_ep_used = %d\n", num_ep_used);
#endif
	return ep;
}

EXPORT_SYMBOL(pxa3xx_ep_config);

/*-------------------------------------------------------------------------*/

static void nop_release(struct device *dev)
{
	DMSG("%s %s\n", __FUNCTION__, dev->bus_id);
}

/* this uses load-time allocation and initialization (instead of
 * doing it at run-time) to save code, eliminate fault paths, and
 * be more obviously correct.
 */
static struct pxa3xx_udc memory = {
	.gadget = {
		   .ops = &pxa3xx_udc_ops,
		   .ep0 = &memory.ep[0].ep,
		   .name = driver_name,
		   .dev = {
			   .bus_id = "gadget",
			   .release = nop_release,
			   },
		   },

	/* control endpoint */
	.ep[0] = {
		  .ep = {
			 .name = ep0name,
			 .ops = &pxa3xx_ep_ops,
			 .maxpacket = EP0_FIFO_SIZE,
			 },
		  .dev = &memory,
		  .reg_udccsr = &UDCCSR0,
		  .reg_udcdr = &UDCDR0,
		  }
};

#define CP15R0_VENDOR_MASK	0xffffe000

#define CP15R0_XSCALE_VALUE	0x69054000	/* intel/arm/xscale */

/*
 * 	probe - binds to the platform device
 */
static int __init pxa3xx_udc_probe(struct device *_dev)
{
	struct pxa3xx_udc *dev = &memory;
	int retval;

	/* other non-static parts of init */
	dev->dev = _dev;
	dev->mach = _dev->platform_data;

#ifdef CONFIG_PREEMPT_RT
	spin_lock_init(&dev->lock);
#endif
	init_timer(&dev->timer);
	dev->timer.function = udc_watchdog;
	dev->timer.data = (unsigned long)dev;

	device_initialize(&dev->gadget.dev);
	dev->gadget.dev.parent = _dev;
	dev->gadget.dev.dma_mask = _dev->dma_mask;

	the_controller = dev;
	dev_set_drvdata(_dev, dev);

	udc_disable(dev);
	udc_init_ep(dev);
	udc_reinit(dev);

#ifdef CONFIG_USB_OTG
	dev->gadget.is_otg = 1;
	dev->transceiver = otg_get_transceiver();
	if (!dev->transceiver) {
		DMSG("failed to get transceiver\n");
	}
#endif
	/* irq setup after old hardware state is cleaned up */
	retval = request_irq(IRQ_USB, pxa3xx_udc_irq,
#ifndef CONFIG_PREEMPT_RT
			     SA_INTERRUPT,
#else
			     0,
#endif
			     driver_name, dev);
	if (retval != 0) {
		printk(KERN_ERR "%s: can't get irq %i, err %d\n",
		       driver_name, IRQ_USB, retval);
		return -EBUSY;
	}
	dev->got_irq = 1;

	create_proc_files();

	INIT_WORK(&pxa3xx_udc_work, pxa3xx_udc_handler, (void *)dev);

	return 0;
}

static int __exit pxa3xx_udc_remove(struct device *_dev)
{
	struct pxa3xx_udc *dev = _dev->driver_data;

	udc_disable(dev);
	remove_proc_files();
	usb_gadget_unregister_driver(dev->driver);

	if (dev->got_irq) {
		free_irq(IRQ_USB, dev);
		dev->got_irq = 0;
	}
	if (machine_is_lubbock() && dev->got_disc) {
		free_irq(LUBBOCK_USB_DISC_IRQ, dev);
		dev->got_disc = 0;
	}
	dev_set_drvdata(_dev, 0);
	the_controller = 0;

	return 0;
}

/*
 * Interrupt comes from pmic when usb cable attached or detached
 */
#ifdef ENABLE_CABLE_DETECT
#ifndef CONFIG_USB_OTG
static void udc_stop(int state)
{
#ifdef CONFIG_USB_COMPOSITE
	struct pxa3xx_udc *dev = the_controller;
	struct gadget_driver_info *pInfo = dev->first_gadget;

	if (dev->driver) {
		if (state == 0) {
			do {
				pInfo->stopped = 0;
				pInfo = pInfo->next;
			} while (pInfo);
		} else if (state == 1) {
			do {
				set_gadget_data(&dev->gadget,
						pInfo->driver_data);
				stop_activity(dev, pInfo);
				pInfo = pInfo->next;
			} while (pInfo);
		} else
			printk(KERN_ERR "stop state %d error\n", state);

	}
#endif
}
#endif

/* detect USB cable attach and detach by PMIC
 * 1 -- cable attached; 0 -- cable detached
 */
static int is_cable_attached(void)
{
	struct pxa3xx_udc *dev = &memory;

#ifdef ENABLE_CABLE_DETECT
	if (mhn_pmic_is_vbus_assert() && dev->driver) {
		/* VBUS level is high, cable attached */
		connected = 1;
		return 1;
	} else {
		connected = 0;
		return 0;
	}
#endif
	return 1;
}

int pxa3xx_pmic_usb_status(int status)
{
	int ret = 0;

	DMSG("%s: enter\n", __func__);
	if ((status & PMIC_EVENT_VBUS) && mhn_pmic_is_vbus_assert()) {
		DMSG("%s: vbus A ssert\n", __func__);
		ret |= USBOTG_VBUS_VALID;
	}

	if ((status & PMIC_EVENT_VBUS) && mhn_pmic_is_srp_ready()) {
		DMSG("%s: srp ready\n", __func__);
		ret |= USBOTG_SRP_DETECT;
	}

	if ((status & PMIC_EVENT_VBUS) && mhn_pmic_is_avbusvld()) {
		DMSG("%s: vbus A valid\n", __func__);
		ret |= USBOTG_VBUS_VALID;
	}

	return ret;
}

int pxa3xx_usb_event_change(unsigned int events)
{
	unsigned long flags;
	int status;
	struct pxa3xx_udc *dev = the_controller;

	DMSG("%s is called, events:%d\n", __FUNCTION__, events);

	local_irq_save_nort(flags);
	spin_lock_rt(&dev->lock);

	if (dev && dev->driver && events) {
		/* bit EXTON_N indicate usb cable event */
		status = pxa3xx_pmic_usb_status(events);
		DMSG("%s: status = 0x%x\n", __func__, status);
		if (status & USBOTG_STATUS_REQUIRED) {
			connected = 1;
#ifdef CONFIG_USB_OTG
			if (d0cs || is_below_624(NULL, 1)) {
				ipm_notify();
			} else {
				mhnotg_require_bus(status);
			}
#else
			if (d0cs || is_below_624(NULL, 1)) {
				ipm_notify();
			} else {
				udc_enable(&memory);
				udc_stop(0);
			}
#endif
		} else {	/* Cable detached */
			connected = 0;
#ifdef CONFIG_USB_OTG
			mhnotg_require_bus(status);
#else
			udc_disable(&memory);
#endif
		}
	}

	local_irq_restore_nort(flags);
	spin_unlock_rt(&dev->lock);

	return 0;
}
#else
int pxa3xx_usb_event_change(unsigned int events)
{
}
#endif

#ifdef CONFIG_PM
static int pxa3xx_udc_suspend(struct device *_dev, u32 state, u32 level)
{
	int i;
	struct pxa3xx_udc *dev = (struct pxa3xx_udc *)dev_get_drvdata(_dev);

	if (dev->driver) {
		DMSG("%s is called\n", __FUNCTION__);
		if (connected) {
			printk(KERN_ERR
			       "Can't make system into susupend when USB cable is attached\n");
			return -EACCES;
		}
#ifdef	CONFIG_USB_OTG
		if (dev->transceiver->default_a) {
			printk(KERN_ERR "Can make systme into suspend"
			       "when USB cable is attached\n");
			return -EACCES;
		}
#endif
		switch (level) {
		case SUSPEND_SAVE_STATE:

			DMSG("%s will go into SUSPEND_SAVE_STATE\n",
			     __FUNCTION__);
			dev->udccsr0 = UDCCSR0;
			for (i = 1; (i < UDC_EP_NUM); i++) {
				if (dev->ep[i].assigned) {
					struct pxa3xx_ep *ep = &dev->ep[i];

					ep->udccsr_value = *ep->reg_udccsr;
					ep->udccr_value = *ep->reg_udccr;
					DMSG("EP%d, udccsr:0x%x, udccr:0x%x\n",
					     i, *ep->reg_udccsr,
					     *ep->reg_udccr);
				}
			}
			break;

		case SUSPEND_DISABLE:
			DMSG("%s will go into SUSPEND_DISABLE\n", __FUNCTION__);
#ifndef CONFIG_USB_COMPOSITE
			stop_activity(dev, dev->driver);
#else
			{
				struct gadget_driver_info *pInfo =
				    dev->first_gadget;

				do {
					set_gadget_data(&dev->gadget,
							pInfo->driver_data);
					stop_activity(dev, pInfo);
					pInfo = pInfo->next;
				} while (pInfo);
			}
#endif

#ifdef CONFIG_USB_OTG
			mhnotg_require_bus(0);
#endif

#ifdef ENABLE_CABLE_DETECT
			connected = 0;
			mhn_pmic_set_pump(0);
#endif
			break;

		case SUSPEND_POWER_DOWN:
			DMSG("%s will go into SUSPEND_POWER_DOWN\n",
			     __FUNCTION__);
			/* disable all the interrupts */
			UDCICR0 = UDCICR1 = 0;
			UDCISR0 = UDCISR1 = 0xffffffff;

			UP2OCR &= ~(UP2OCR_DPPUE | UP2OCR_DPPDE |
				    UP2OCR_DMPUE | UP2OCR_DMPDE);
#ifdef CONFIG_USB_OTG
			otg_set_peripheral(dev->transceiver, NULL);
#endif

			/* disable the controller */
			UDCCR = 0;
			pxa_set_cken(CKEN_UDC, 0);
			break;

		default:
			DMSG("%s, unsupported level\n", __FUNCTION__);
			break;
		}
	}

	return 0;
}

static int pxa3xx_udc_resume(struct device *_dev, u32 level)
{
	int i;
	struct pxa3xx_udc *dev = (struct pxa3xx_udc *)dev_get_drvdata(_dev);

	if (dev->driver) {
		DMSG("%s is called\n", __FUNCTION__);
		switch (level) {
		case RESUME_POWER_ON:
			DMSG("%s: RESUME_POWER_ON\n", __FUNCTION__);
			pxa_set_cken(CKEN_UDC, 1);
			break;

		case RESUME_RESTORE_STATE:
			DMSG("%s: RESUME_RESTORE_STATE\n", __FUNCTION__);

			UDCCSR0 =
			    dev->
			    udccsr0 & (UDCCSR0_ACM | UDCCSR0_FST | UDCCSR0_DME |
				       UDCCSR0_ODFCLR);
			for (i = 1; i < UDC_EP_NUM; i++) {
				if (dev->ep[i].assigned) {
					struct pxa3xx_ep *ep = &dev->ep[i];

					*ep->reg_udccsr = ep->udccsr_value;
					*ep->reg_udccr = ep->udccr_value;
					DMSG("EP%d, udccsr:0x%x, udccr:0x%x\n",
					     i, *ep->reg_udccsr,
					     *ep->reg_udccr);
				}
			}
			break;

		case RESUME_ENABLE:
			DMSG("%s: RESUME_ENABLE\n", __FUNCTION__);
			/* NOTES: we have assumption that the silicon
			 * response for VBUS keep registers value
			 * when doing suspend/resume. So needn't to
			 * set vbus pump here. If this behavior can not
			 * be guaranteed, we need set vbus pump here.
			 *
			 */
			udc_enable(dev);
#ifdef CONFIG_USB_OTG
			otg_set_peripheral(dev->transceiver, &dev->gadget);
#endif
			break;
		default:
			DMSG("%s, unsupported level\n", __FUNCTION__);
			break;
		}
	}

	return 0;
}

#endif

static int is_below_624(struct mhn_fv_info *info, int attached)
{
#ifdef FIX_AT_624
	if (info && is_cable_attached() && ((info->xl != 0x18)
					    || (info->xn != 0x2)
					    || (info->d0cs != 0)))
		return 1;	/* cable attached, and next op is not 624MHz */
	if (!info && attached) {
		if (((ACSR & 0x1f) != 0x18) && ((ACSR & 0x700) != 0x2))
			return 1;	/* cable attached, and cur op is not 624MHz */
	}
#endif
	return 0;
}

/*-------------------------------------------------------------------------*/

static struct device_driver udc_driver = {
	.name = "pxa2xx-udc",
	.bus = &platform_bus_type,
	.probe = pxa3xx_udc_probe,
	.remove = __exit_p(pxa3xx_udc_remove),

#ifdef CONFIG_PM
	/*  FIXME power management support */
	.suspend = pxa3xx_udc_suspend,
	.resume = pxa3xx_udc_resume
#endif
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
