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

#include <linux/config.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/usb.h>

#include "../core/hcd.h"

#include "musbdefs.h"
#include "musb_host.h"
static void
musb_advance_urb_queue(struct musb *pThis, struct urb *urb,
		       struct musb_hw_ep *pEnd, int is_in);
#define HOST_TX_DMA_SOLUTION

#ifdef HOST_TX_DMA_SOLUTION
/* Globals and function prototypes */

#define MAX_TASKLET_RESCHED_COUNT	256	/* Maximum tasklet reschedule count */
#endif				/* HOST_TX_DMA_SOLUTION */

/* MUSB HOST status 9-mar-2006
 *
 * - PIO mostly behaved when last tested.  Error paths are all over the map
 *   though, there's lots of partially duplicated code ... the cleanup code
 *   itself needs to be cleaned up!
 *
 * - DMA (CPPI) ... mostly behaves, but not as quickly as expected by
 *   comparison with other high speed hosts.
 *     + RX, sometimes reqpkt seems to misbehave so that a packet from
 *	 a second URB (e.g. mass storage status) gets read too early
 *     + TX, no known issues
 *
 * - Still no traffic scheduling code to make NAKing for bulk or control
 *   transfers unable to starve other requests; or to make efficient use
 *   of hardware with periodic transfers.  (Note that network drivers
 *   commonly post bulk reads that stay pending for a long time; these
 *   would make very visible trouble.)
 *
 * - Host side doesn't understand that endpoint hardware has two directions.
 */

/*
 * NOTE on endpoint usage:
 *
 * CONTROL transfers all go through ep0.  BULK ones go through dedicated IN
 * and OUT endpoints ... hardware is dedicated for those "async" queue(s).
 *
 * (Yes, bulk _could_ use more of the endpoints than that, and would even
 * benefit from it ... one remote device may easily be NAKing while others
 * need to perform transfers in that same direction.)
 *
 * INTERUPPT and ISOCHRONOUS transfers are scheduled to the other endpoints.
 * So far that scheduling is both dumb and optimistic:  the endpoint will be
 * "claimed" until its software queue is no longer refilled.  No multiplexing
 * of transfers between endpoints, or anything clever.
 */

/*************************** Forwards ***************************/

static void musb_ep_program(struct musb *pThis, u8 bEnd,
			    struct urb *pUrb, unsigned int nOut,
			    u8 * pBuffer, u32 dwLength);
static void musb_giveback(struct musb_hw_ep *ep, struct urb *urb, int status);

/*
 * Start transmit. Caller is responsible for locking shared resources.
 * pThis must be locked.
 *
 * @param pThis instance pointer
 * @param bEnd local endpoint
 */
void MGC_HdrcStartTx(struct musb *pThis, u8 bEnd)
{
	u16 wCsr;
	void __iomem *pBase = pThis->pRegs;

	/* NOTE: no locks here; caller should lock */
	MGC_SelectEnd(pBase, bEnd);
	if (bEnd) {
		wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd);
		wCsr |= MGC_M_TXCSR_TXPKTRDY | MGC_M_TXCSR_H_WZC_BITS;
		DBG(5, "Writing TXCSR%d = %x\n", bEnd, wCsr);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, wCsr);
	} else {
		wCsr = MGC_M_CSR0_H_SETUPPKT | MGC_M_CSR0_TXPKTRDY;
		MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, wCsr);
	}

}

/*
 *   Enable DmareqEnab in TxCSr
 *
 *   @param pThis instance pointer
 *   @param bEnd  local endpoint
 */
void MGC_HdrcEnableTXDMA(struct musb *pThis, u8 bEnd)
{
	void __iomem *pBase = pThis->pRegs;
	u16 txCsr;

	/* NOTE: no locks here; caller should lock */
	MGC_SelectEnd(pBase, bEnd);
	txCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd);
	txCsr |= MGC_M_TXCSR_DMAENAB | MGC_M_TXCSR_H_WZC_BITS;
#ifdef HOST_TX_DMA_SOLUTION
	txCsr |= MGC_M_TXCSR_DMAMODE;
#endif
	MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, txCsr);
}

/*
 * Start the URB at the front of an endpoint's queue
 * end must be claimed from the caller.
 *
 * Context: controller locked, irqs blocked
 *
 * FIXME caller should specify rx or tx
 */
static void musb_start_urb(struct musb *pThis, struct musb_hw_ep *pEnd)
{
	u16 wFrame = 0;
	u32 dwLength = 0;
	void *pBuffer;
	void __iomem *pBase = pThis->pRegs;
	struct urb *pUrb = MGC_GetCurrentUrb(pEnd);
	unsigned nPipe = pUrb->pipe;
	unsigned is_out = usb_pipeout(nPipe);
	u8 bAddress = usb_pipedevice(nPipe);
	u8 bRemoteEnd = usb_pipeendpoint(nPipe);
	u16 wPacketSize;
	int bEnd = pEnd->bLocalEnd;
	struct musb_opmode *musb_mode = pUrb->hcpriv;

	wPacketSize = usb_maxpacket(pUrb->dev, nPipe, is_out);

	/* remember software state */
	pEnd->dwOffset = 0;
	pEnd->dwRequestSize = 0;
	pEnd->dwIsoPacket = 0;
	pEnd->dwWaitFrame = 0;

	/* REVISIT distinguish tx and rx sides properly */
	pEnd->wPacketSize = wPacketSize;
	pEnd->bAddress = bAddress;
	pEnd->bEnd = bRemoteEnd;
	pEnd->bTrafficType = (u8) usb_pipetype(nPipe);

	if (is_out)
		pEnd->out_busy = 1;
	else
		pEnd->in_busy = 1;

	/* end must be claimed from my caller */

	if (usb_pipecontrol(nPipe)) {
		/* control transfers always start with an OUT */
		is_out = 1;
		pThis->bEnd0Stage = MGC_END0_START;
	}

	/* gather right source of data */
	if (usb_pipeisoc(nPipe)) {
		pBuffer = pUrb->transfer_buffer + pUrb->
				iso_frame_desc[pEnd->dwIsoPacket].offset;
		if (musb_mode->dma) {
			if (pEnd->num_iso_desc < pUrb->number_of_packets) {
				if (pEnd->iso_desc)
					kfree(pEnd->iso_desc);

				pEnd->iso_desc = kmalloc(pUrb->
						number_of_packets *
						sizeof(struct musb_iso_desc),
						GFP_KERNEL);
				if (pEnd->iso_desc == NULL) {
					DBG(1, "ISO desc. alloc failed\n");
					musb_giveback(pEnd, pUrb, -EINVAL);
					return;
				}
			}

			do {
				dwLength += pUrb->iso_frame_desc[wFrame].length;
				pEnd->iso_desc[wFrame].length =
					pUrb->iso_frame_desc[wFrame].length;
				pEnd->iso_desc[wFrame].offset =
					pUrb->iso_frame_desc [wFrame].offset;
				DBG(4, "Frame Number %d Length %d Offset %d\n",
				wFrame, pUrb->iso_frame_desc[wFrame].length,
				pUrb->iso_frame_desc[wFrame].offset);
			} while (++wFrame < pUrb->number_of_packets);

			pEnd->num_iso_desc = pUrb->number_of_packets;
		} else
			dwLength = pUrb->iso_frame_desc[pEnd->dwIsoPacket].
					length;

	} else if (usb_pipecontrol(nPipe)) {
		pBuffer = pUrb->setup_packet;
		dwLength = 8;
	} else {
		/* - */
		pBuffer = pUrb->transfer_buffer;
		dwLength = pUrb->transfer_buffer_length;
	}

	DBG(4, "urb %p: ep%d%s, type %d, max %d, addr %d, buffer %p len %u\n",
	    pUrb, bRemoteEnd, (is_out) ? "out" : "in",
	    usb_pipetype(nPipe), wPacketSize, bAddress, pBuffer, dwLength);

	/* Configure endpoint */
	musb_ep_program(pThis, bEnd, pUrb, is_out, pBuffer, dwLength);

	/* transmit may have more work: start it when it is time */
	if (!is_out)
		return;

	/* TODO: with CPPI DMA, once DMA is setup and DmaReqEnable in TxCSR
	 * is set (which is the case) transfer is initiated. For periodic
	 * transfer support, add another field in pEnd struct which will
	 * serve as a flag. If CPPI DMA is programmed for the transfer set
	 * this flag and disable DMAReqEnab while programming TxCSR in
	 * programEnd() Once we reach the appropriate time, enable DMA Req
	 * instead of calling StartTx() function
	 */

	/* determine if the time is right for a periodic transfer */
	if (usb_pipeisoc(nPipe) || usb_pipeint(nPipe)) {
		DBG(3, "check whether there's still time for periodic Tx\n");
		wFrame = musb_readw(pBase, MGC_O_HDRC_FRAME);
		/* FIXME this doesn't implement that scheduling policy ...
		 * or handle framecounter wrapping
		 */
		if (!((pUrb->transfer_flags & URB_ISO_ASAP)
		    || (wFrame >= pUrb->start_frame))) {
			pEnd->dwWaitFrame = pUrb->start_frame;
			/* enable SOF interrupt so we can count down */
			DBG(1, "SOF for %d\n", bEnd);
#if 1				// ifndef CONFIG_ARCH_DAVINCI
			musb_writeb(pBase, MGC_O_HDRC_INTRUSBE, 0xff);
#endif
			return;
		}
	} else {
		DBG(4, "Start TX%d %s\n", bEnd,
		    pEnd->pDmaChannel ? "dma" : "pio");
	}

	if (musb_mode->dma && pEnd->pDmaChannel)
		MGC_HdrcEnableTXDMA(pThis, bEnd);
	else
		MGC_HdrcStartTx(pThis, bEnd);
}

/* caller owns no controller locks, irqs are blocked */
static inline void __musb_giveback(struct urb *urb, int status)
{
	struct musb_opmode *musb_mode = urb->hcpriv;
	const struct musb_hw_ep *hw_ep = musb_mode->ep;

	if ((urb->transfer_flags & URB_SHORT_NOT_OK)
	    && (urb->actual_length < urb->transfer_buffer_length)
	    && status == 0 && usb_pipein(urb->pipe))
		status = -EREMOTEIO;

	spin_lock(&urb->lock);
	//list_del(&urb->urb_list);
	list_del_init(&urb->urb_list);
	urb->hcpriv = NULL;
	if (urb->status == -EINPROGRESS)
		urb->status = status;
	spin_unlock(&urb->lock);

	DBG(( {
	     int level; switch (urb->status) {
case 0:
	     level = 3; break;
	     /* common/boring faults */
case -EREMOTEIO:
case -ESHUTDOWN:
case -EPIPE:
level = 3; break; default:
	     level = 2; break;}; level;}
	    ),
	    "complete %p (%d), dev%d ep%d%s, %d/%d\n",
	    urb, urb->status,
	    usb_pipedevice(urb->pipe),
	    usb_pipeendpoint(urb->pipe),
	    usb_pipein(urb->pipe) ? "in" : "out",
	    urb->actual_length, urb->transfer_buffer_length) ;

	/* teardown DMA mapping, if needed (does dcache sync) */
	if (musb_mode->dma && !(urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP))
		dma_unmap_single(hw_ep->musb->controller, urb->transfer_dma,
				 urb->transfer_buffer_length,
				 usb_pipein(urb->pipe)
				 ? DMA_FROM_DEVICE : DMA_TO_DEVICE);

	/* completion handler may reenter this hcd; periodic transfers
	 * are normally resubmitted during the callback.
	 *
	 * FIXME: make this use hcd framework giveback, so we don't use
	 * the usbcore-internal wakeup queue...
	 */
	if (urb->dev) {
		usb_put_dev(urb->dev);
		spin_unlock (&hw_ep->musb->Lock);
		urb->complete(urb, hw_ep->musb->int_regs);
		spin_lock (&hw_ep->musb->Lock);
		atomic_dec(&urb->use_count);
		if (urb->reject)
			wake_up(&usb_kill_urb_queue);
		usb_put_urb(urb);
	} else {
		DBG(4, "NULL device pointer\n");
	}
}

/* for non-iso endpoints only */
static inline void musb_save_toggle(struct musb_hw_ep *ep, struct urb *urb)
{
	struct usb_device *udev = urb->dev;
	u16 csr;
	void __iomem *hw = ep->musb->pRegs;

	ep->bIsReady = FALSE;
	if (!udev)
		return;
	if (usb_pipeout(urb->pipe)) {
		csr = MGC_ReadCsr16(hw, MGC_O_HDRC_TXCSR, ep->bLocalEnd);
		usb_settoggle(udev, ep->bEnd, 1,
			      (csr & MGC_M_TXCSR_H_DATATOGGLE)
			      ? 1 : 0);
	} else {
		csr = MGC_ReadCsr16(hw, MGC_O_HDRC_RXCSR, ep->bLocalEnd);
		usb_settoggle(udev, ep->bEnd, 0,
			      (csr & MGC_M_RXCSR_H_DATATOGGLE)
			      ? 1 : 0);
	}
}

#ifdef HOST_TX_DMA_SOLUTION
void musb_h_fifo_check_complete (struct musb_hw_ep *ep)
{
	musb_advance_urb_queue(ep->musb, next_out_urb(ep), ep, USB_DIR_OUT);
}
/*
 *	Invoke tasklet and update the EP with information for tasklet to
 *	probe for fifo flush.
 */
void musb_tx_tasklet_invoke(struct musb *pThis, struct musb_hw_ep *pEnd)
{
	u16 csr;

	csr = MGC_ReadCsr16(pThis->pRegs, MGC_O_HDRC_TXCSR, pEnd->bLocalEnd);
	if ((csr & MGC_M_TXCSR_FIFONOTEMPTY) || (csr & MGC_M_TXCSR_TXPKTRDY)) {
		pEnd->fifo_flush_check = 1;
		tasklet_schedule(&pThis->fifo_check);
	} else {
		pEnd->fifo_flush_check = 0;
		musb_advance_urb_queue(pThis, next_out_urb(pEnd), pEnd,
				       USB_DIR_OUT);
	}
}
#endif				/* HOST_TX_DMA_SOLUTION */

/* caller owns controller lock, irqs are blocked */
static void musb_giveback(struct musb_hw_ep *ep, struct urb *urb, int status)
__releases(ep->musb->Lock) __acquires(ep->musb->Lock)
{
	int is_in;
	u8 type;
	struct urb *next;

	if (ep->bIsSharedFifo)
		is_in = 1;
	else
		is_in = usb_pipein(urb->pipe);
	type = is_in ? ep->in_traffic_type : ep->out_traffic_type;
	next = (is_in) ? next_in_urb(ep) : next_out_urb(ep);

	/* save toggle eagerly, for paranoia */
	switch (type) {
	case PIPE_BULK:
	case PIPE_INTERRUPT:
		if (urb == next)
			musb_save_toggle(ep, urb);
	}

	/* spin_unlock(&ep->musb->Lock); */

	__musb_giveback(urb, status);

	/* spin_lock(&ep->musb->Lock); */
	if (is_in)
		ep->in_busy = (urb == next) ? 0 : ep->in_busy;
	else
		ep->out_busy = (urb == next) ? 0 : ep->out_busy;

	/* reclaim resources (and bandwidth) ASAP */
	if (list_empty(&ep->urb_list)) {
		switch (type) {
		case PIPE_ISOCHRONOUS:
		case PIPE_INTERRUPT:
			/* this is where periodic bandwidth should be
			 * de-allocated if its tracked and allocated.
			 */
			ep->bIsClaimed = FALSE;
			ep->bIsReady = FALSE;
			break;
		default:
			ep->bIsReady = FALSE;
		}
		ep->out_busy = ep->out_busy = 0;
	}
}

/*
 * Advance this endpoint's queue, completing the URB at list head.
 * Then start the next URB, if there is one.
 *
 * @pThis: instance pointer
 * @bEnd: local endpoint
 * Context: caller owns controller lock, irqs are blocked
 */
static void
musb_advance_urb_queue(struct musb *pThis, struct urb *urb,
		       struct musb_hw_ep *pEnd, int is_in)
{
	if (urb)
		musb_giveback(pEnd, urb, 0);

	while ((!list_empty(&pEnd->urb_list)) && (!pEnd->busy)) {
		DBG(4, "... next ep%d %cX urb %p\n",
		    pEnd->bLocalEnd, is_in ? 'R' : 'T',
		    // next_in_urb() or next_out_urb()
		    MGC_GetCurrentUrb(pEnd));
		pEnd->bIsReady = 0;
		musb_start_urb(pThis, pEnd);
	}
}

/*
 * Receive a packet (or part of it).
 * @requires pThis->Lock locked
 * @return TRUE if URB is complete
 */
static u8 musb_host_packet_rx(struct musb *pThis, struct urb *pUrb,
			      u8 bEnd, u8 bIsochError)
{
	u16 wRxCount;
	u16 wLength;
	u8 *pBuffer;
	u16 wCsr;
	u8 bDone = FALSE;
	void __iomem *pBase = pThis->pRegs;
	struct musb_hw_ep *pEnd = &(pThis->aLocalEnd[bEnd * 2]);
	int nPipe = pUrb->pipe;
	void *buffer = pUrb->transfer_buffer;

	// MGC_SelectEnd(pBase, bEnd);
	wRxCount = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCOUNT, bEnd);

	DBG(3, "RX%d count %d, buffer %p len %d/%d\n", bEnd, wRxCount,
	    pUrb->transfer_buffer, pEnd->dwOffset,
	    pUrb->transfer_buffer_length);

	/* unload FIFO */
	if (usb_pipeisoc(nPipe)) {
		/* isoch case */
		pBuffer =
		    buffer + pUrb->iso_frame_desc[pEnd->dwIsoPacket].offset;
		wLength =
		    min((unsigned int)wRxCount,
			pUrb->iso_frame_desc[pEnd->dwIsoPacket].length);
		pUrb->actual_length += wLength;
		/* update actual & status */
		pUrb->iso_frame_desc[pEnd->dwIsoPacket].actual_length = wLength;
		if (bIsochError) {
			pUrb->iso_frame_desc[pEnd->dwIsoPacket].status =
			    -EILSEQ;
			pUrb->error_count++;
		} else {
			pUrb->iso_frame_desc[pEnd->dwIsoPacket].status = 0;
		}

		/* see if we are done */
		bDone = (++pEnd->dwIsoPacket >= pUrb->number_of_packets);
	} else {
		/* non-isoch */
		pBuffer = buffer + pEnd->dwOffset;
		wLength = min((unsigned int)wRxCount,
			      pUrb->transfer_buffer_length - pEnd->dwOffset);
		pUrb->actual_length += wLength;
		pEnd->dwOffset += wLength;

		/* see if we are done */
		bDone = (pEnd->dwOffset >= pUrb->transfer_buffer_length) ||
		    (wRxCount < pEnd->wPacketSize);
	}

	if (wLength)
		musb_read_fifo(pEnd, wLength, pBuffer);

	if (wRxCount <= wLength) {
		wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd);
		wCsr |= MGC_M_RXCSR_H_WZC_BITS;
		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd,
			       wCsr & ~MGC_M_RXCSR_RXPKTRDY);
	}

	return bDone;
}

/*
 * Program an HDRC endpoint as per the given URB
 * @pThis: instance pointer
 * @bEnd: local endpoint
 * @pURB: URB pointer
 * @is_out: zero for Rx; non-zero for Tx
 * @pBuffer: buffer pointer
 * @dwLength: how many bytes to transmit or expect to receive
 * Context: irqs blocked, controller lock held
 */
static void musb_ep_program(struct musb *pThis, u8 bEnd,
			    struct urb *pUrb, unsigned int is_out,
			    u8 * pBuffer, u32 dwLength)
{
	u16 wCsr, wLoadCount;
#ifndef HOST_TX_DMA_SOLUTION
	u16 wIntrTxE;
#endif
	struct usb_device *pParent;
#ifndef	CONFIG_USB_INVENTRA_FIFO
	struct dma_controller *pDmaController;
	struct dma_channel *pDmaChannel = NULL;
	u8 bDmaOk;
#endif
	void __iomem *pBase = pThis->pRegs;
	unsigned int nPipe = pUrb->pipe;
	u16 wPacketSize = usb_maxpacket(pUrb->dev, nPipe, is_out);
	u8 bIsBulk = usb_pipebulk(nPipe);
	u8 bAddress = (u8) usb_pipedevice(nPipe);
	u8 bRemoteEnd = (u8) usb_pipeendpoint(nPipe);
	u8 bSpeed = (u8) pUrb->dev->speed;
	u8 bInterval = (u8) pUrb->interval;
	struct musb_hw_ep *pEnd = &(pThis->aLocalEnd[(bEnd) ?
				(is_out) ? bEnd * 2 - 1 : bEnd * 2 : 0]);
	u8 bStdType = 0;
	u8 bHubAddr = 0;
	u8 bHubPort = 0;
	u8 reg = 0;
	u8 bIsMulti = FALSE;
#ifndef	CONFIG_USB_INVENTRA_FIFO
	u32 addr;
	struct musb_opmode *musb_mode = pUrb->hcpriv;
#endif
	pParent = pUrb->dev->parent;
	if (pParent != pThis->RootHub.pDevice)
		bHubAddr = (u8) pParent->devnum;

	/* set up tt info if needed */
	if (pUrb->dev->tt) {
		bHubAddr = (u8) pUrb->dev->tt->hub->devnum;
		bHubPort = (u8) pUrb->dev->ttport;
		bIsMulti = (u8) pUrb->dev->tt->multi;
	}

	DBG(3, "%s hw%d, urb %p, spd%d dev%d ep%d%s, "
	    "hub%d port%d%s, bytes %d\n",
	    is_out ? "-->" : "<--",
	    bEnd, pUrb, pUrb->dev->speed,
	    bAddress, bRemoteEnd, is_out ? "out" : "in",
	    bHubAddr, bHubPort + 1, bIsMulti ? " multi" : "", dwLength);

	/* prepare endpoint registers according to flags */
	if (usb_pipeisoc(nPipe)) {
		bStdType = USB_ENDPOINT_XFER_ISOC;
		/* both speeds use log encoding */
		bInterval = fls(pUrb->interval);
	} else if (usb_pipeint(nPipe)) {
		bStdType = USB_ENDPOINT_XFER_INT;
		/* only highspeed uses log encoding */
		if (USB_SPEED_HIGH == bSpeed)
			bInterval = fls(pUrb->interval);
		else if (bInterval == 0)
			bInterval = 1;
	} else if (bIsBulk) {
		bStdType = USB_ENDPOINT_XFER_BULK;
		bInterval = 0;	/* ignoring bulk NAK limits for now */
	} else {
		// is_out = 1;
		bInterval = 0;
		bStdType = USB_ENDPOINT_XFER_CONTROL;
	}

	reg = bStdType << 4;
#if 0
	/* REVISIT we actually want to use NAK limits, as a hint to the
	 * transfer scheduling logic to try some other peripheral endpoint.
	 *
	 * The downside of disabling this is that transfer scheduling gets
	 * VERY unfair for nonperiodic transfers, and a misbehaving driver
	 * could make that hurt.  Or for reads, one that acts perfectly
	 * normally:  network and other drivers keep reads posted at all
	 * times, having one pending for a week should be perfectly safe.
	 *
	 * The downside of enabling it is needing transfer scheduling that
	 * can put this aside for while ... NAKing is **NOT AN ERROR** but
	 * some device drivers will want to time out requests.  The current
	 * scheduler treats these as errors though.
	 */
	if (bInterval == 0) {
		bInterval = 16;
	}
#endif

	reg |= (bRemoteEnd & 0xf);
	/* REVISIT:  test multipoint/not at compiletime ... */
	if (pThis->bIsMultipoint) {
		switch (bSpeed) {
		case USB_SPEED_LOW:
			reg |= 0xc0;
			break;
		case USB_SPEED_FULL:
			reg |= 0x80;
			break;
		default:
			reg |= 0x40;
		}
	}

	if (bIsBulk && pThis->bBulkSplit) {
		wLoadCount = min((u32) pEnd->wMaxPacketSizeTx, dwLength);
	} else {
		wLoadCount = min((u32) wPacketSize, dwLength);
	}

	MGC_SelectEnd(pBase, bEnd);

#ifndef	CONFIG_USB_INVENTRA_FIFO
	pDmaChannel = pEnd->pDmaChannel;
	pDmaController = pThis->pDmaController;

	if (!pEnd->pDmaChannel)  {
		if (musb_mode->dma) {
			pDmaChannel = pEnd->pDmaChannel = pDmaController->
			    pfDmaAllocateChannel(pDmaController->pPrivateData,
						 bEnd, is_out ? TRUE : FALSE,
						 bStdType, wPacketSize);
			if (pDmaChannel)
				bDmaOk = 1;
			else
				bDmaOk = 0;
		} else
			bDmaOk = 0;
	} else
		if (musb_mode->dma)
			bDmaOk = 1;
		else
			bDmaOk = 0;

#ifdef CONFIG_USB_INVENTRA_DMA
	if (bDmaOk && pDmaChannel) {
		pDmaChannel->dwActualLength = 0L;
		pEnd->dwRequestSize = min(dwLength, pDmaChannel->dwMaxLength);
		bDmaOk = pDmaController->pfDmaProgramChannel(pDmaChannel,
								wPacketSize,
								pDmaChannel->
								bDesiredMode,
								pUrb->
								transfer_dma,
								pEnd->
								dwRequestSize);
		if (bDmaOk) {
			wLoadCount = 0;
		} else {
			pDmaController->pfDmaReleaseChannel(pDmaChannel);
			pDmaChannel = pEnd->pDmaChannel = NULL;
			musb_dma->dma = 0;
		}
	}
#endif
#endif

	/* even RX side may need TXCSR, for MGC_M_TXCSR_MODE */
	if (bEnd)
		wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd);
	else
		wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_CSR0, bEnd);

	/* make sure we clear DMAEnab, autoSet bits from previous run */

	/* OUT/transmit or IN/receive? */
	if (is_out) {
		/* transmit */
#ifndef HOST_TX_DMA_SOLUTION
		/* disable interrupt in case we flush */
		wIntrTxE = musb_readw(pBase, MGC_O_HDRC_INTRTXE);
		musb_writew(pBase, MGC_O_HDRC_INTRTXE, wIntrTxE & ~(1 << bEnd));
#endif
		if (bEnd) {

			// REVISIT assert(bIsReady) earlier

			/* general endpoint setup */
			if (!pEnd->bIsReady) {
				u16 csr = wCsr;

				pEnd->bIsReady = TRUE;

				/* flush all old state, set default */
				csr &= ~(MGC_M_TXCSR_H_NAKTIMEOUT
					 | MGC_M_TXCSR_DMAMODE
					 | MGC_M_TXCSR_FRCDATATOG
					 | MGC_M_TXCSR_ISO
					 | MGC_M_TXCSR_H_RXSTALL
					 | MGC_M_TXCSR_H_ERROR
					 | MGC_M_TXCSR_FIFONOTEMPTY
					 | MGC_M_TXCSR_TXPKTRDY);

				if ((wCsr & MGC_M_TXCSR_FIFONOTEMPTY) ||
					(wCsr & MGC_M_TXCSR_TXPKTRDY)) {
					WARN("tx%d, fifo full %x ?\n", bEnd,
					     wCsr);
					csr |= MGC_M_TXCSR_FLUSHFIFO;
				}

				csr |= MGC_M_TXCSR_MODE;

				if (pEnd->out_traffic_type == PIPE_ISOCHRONOUS)
					csr |= MGC_M_TXCSR_ISO;
				else if (usb_gettoggle(pUrb->dev,
						       pEnd->bEnd, 1))
					csr |= MGC_M_TXCSR_H_WR_DATATOGGLE
					    | MGC_M_TXCSR_H_DATATOGGLE;
				else
					csr |= MGC_M_TXCSR_CLRDATATOG;

				/* twice in case of double packet buffering */
				MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd,
					       csr);
				MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd,
					       csr);
				wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR,
						     bEnd);
			}
		} else {
			/* endpoint 0: just flush */
			MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, bEnd,
				       wCsr | MGC_M_CSR0_FLUSHFIFO);
			MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, bEnd,
				       wCsr | MGC_M_CSR0_FLUSHFIFO);
		}

		/* REVISIT:  test multipoint/not at compiletime ... */
		if (pThis->bIsMultipoint) {
			/* target addr & hub addr/port */
			musb_writeb(pBase,
				    MGC_BUSCTL_OFFSET(bEnd,
						      MGC_O_HDRC_TXFUNCADDR),
				    bAddress);
			musb_writeb(pBase,
				    MGC_BUSCTL_OFFSET(bEnd,
						      MGC_O_HDRC_TXHUBADDR),
				    bIsMulti ? 0x80 | bHubAddr : bHubAddr);
			musb_writeb(pBase,
				    MGC_BUSCTL_OFFSET(bEnd,
						      MGC_O_HDRC_TXHUBPORT),
				    bHubPort);
		} else {
			/* non-multipoint core */
			musb_writeb(pBase, MGC_O_HDRC_FADDR, bAddress);
		}

		/* protocol/endpoint/interval/NAKlimit */
		if (bEnd) {
			MGC_WriteCsr8(pBase, MGC_O_HDRC_TXTYPE, bEnd, reg);
#ifdef	C_MP_TX
			if (bIsBulk && pThis->bBulkSplit) {
				MGC_WriteCsr16(pBase, MGC_O_HDRC_TXMAXP, bEnd,
					       wPacketSize |
					       ((pEnd->wMaxPacketSizeTx /
						 wPacketSize) - 1) << 11);
			} else
#endif
			{
				MGC_WriteCsr16(pBase, MGC_O_HDRC_TXMAXP, bEnd,
					       wPacketSize);
			}
// XXX
			MGC_WriteCsr8(pBase, MGC_O_HDRC_TXINTERVAL, bEnd,
				      bInterval);
		} else {
			MGC_WriteCsr8(pBase, MGC_O_HDRC_NAKLIMIT0, 0,
				      bInterval);
			/* REVISIT:  test multipoint/not at compiletime ... */
			if (pThis->bIsMultipoint) {
				MGC_WriteCsr8(pBase, MGC_O_HDRC_TYPE0, 0,
					      reg & 0xc0);
			}
		}

#ifdef CONFIG_USB_INVENTRA_DMA
		if (bDmaOk) {
			wCsr |= (MGC_M_TXCSR_AUTOSET | MGC_M_TXCSR_DMAENAB |
				 (pDmaChannel->bDesiredMode
				  ? MGC_M_TXCSR_DMAMODE : 0));
			MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, wCsr);
		}
#elif defined(CONFIG_USB_TI_CPPI_DMA)

		/* candidate for DMA */
		if (bDmaOk) {
			pEnd->fifo_flush_check = 0;

			/* program endpoint CSRs first, then setup DMA.
			 * assume CPPI setup succeeds.
			 * defer enabling dma.
			 */
			wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd);
			wCsr &= ~(MGC_M_TXCSR_AUTOSET
				  | MGC_M_TXCSR_DMAMODE | MGC_M_TXCSR_DMAENAB);
			wCsr |= MGC_M_TXCSR_MODE ;
			MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, wCsr);

			pDmaChannel->dwActualLength = 0L;
			pEnd->dwRequestSize = dwLength;

			/* TX uses "rndis" mode automatically, but needs help
			 * to identify the zero-length-final-packet case.
			 */
			if (usb_pipeisoc(nPipe))
				addr = pUrb->transfer_dma + pUrb->
						iso_frame_desc[0].offset;
			else
				addr = pUrb->transfer_dma;

			bDmaOk = pDmaController->pfDmaProgramChannel(
					pDmaChannel, wPacketSize,
					usb_pipeisoc(nPipe)? 2 :
					(pUrb->transfer_flags&URB_ZERO_PACKET)
					== URB_ZERO_PACKET, addr, pEnd->
					dwRequestSize);
			if (bDmaOk) {
				wLoadCount = 0;
			} else {
				pDmaController->pfDmaReleaseChannel(
							pDmaChannel);
				pEnd->pDmaChannel = NULL;
			}
		}
#endif
		if (wLoadCount) {
			pUrb->hcpriv = &pEnd->musb_mode[0];
			/* PIO to load FIFO */
			pEnd->dwRequestSize = wLoadCount;
			musb_write_fifo(pEnd, wLoadCount, pBuffer);
			wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd);
			wCsr &= ~(MGC_M_TXCSR_DMAENAB | MGC_M_TXCSR_DMAMODE |
				  MGC_M_TXCSR_AUTOSET);
			/* write CSR */
			wCsr |= MGC_M_TXCSR_MODE;

			if (bEnd)
				MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd,
					       wCsr);

		}
#ifndef HOST_TX_DMA_SOLUTION
		/* re-enable interrupt */
		musb_writew(pBase, MGC_O_HDRC_INTRTXE, wIntrTxE);
#endif

		/* IN/receive */
	} else {
		/* First retarget this EP hardware to the correct peripheral
		 * endpoint.  Then activate the transfer (plus maybe dma).
		 */

		/* if programmed for Tx, be sure it is ready for re-use */
		if (pEnd->bIsSharedFifo && (wCsr & MGC_M_TXCSR_MODE)) {
			pEnd->bIsReady = FALSE;
			pr_debug("set endready to False \n");
			if (wCsr & MGC_M_TXCSR_FIFONOTEMPTY) {
#if 0
				/* REVISIT but setting toggle won't help as
				 * much as, oh, clearing the fifo...
				 */
				/* this shouldn't happen */
				MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd,
					       MGC_M_TXCSR_FRCDATATOG);
				MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd,
					       MGC_M_TXCSR_FRCDATATOG);
#endif
				ERR("TX FIFO%d not empty\n", bEnd);
			}
			/* clear mode (and everything else) to enable Rx */
			MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, 0);
		}

		/* REVISIT:  test multipoint/not at compiletime ... */
		/* address */
		if (pThis->bIsMultipoint) {
			/* target addr & hub addr/port */
			musb_writeb(pBase, MGC_BUSCTL_OFFSET(bEnd,
							     MGC_O_HDRC_RXFUNCADDR),
				    bAddress);
			musb_writeb(pBase, MGC_BUSCTL_OFFSET(bEnd,
							     MGC_O_HDRC_RXHUBADDR),
				    bIsMulti ? 0x80 | bHubAddr : bHubAddr);
			musb_writeb(pBase, MGC_BUSCTL_OFFSET(bEnd,
							     MGC_O_HDRC_RXHUBPORT),
				    bHubPort);
		} else {
			/* non-multipoint core */
			musb_writeb(pBase, MGC_O_HDRC_FADDR, bAddress);
		}

		/* protocol/endpoint/interval/NAKlimit */
		if (bEnd) {
			MGC_WriteCsr8(pBase, MGC_O_HDRC_RXTYPE, bEnd, reg);
#if 0
//#ifdef        C_MP_RX
			/* doesn't work reliably */
			if (bIsBulk && pThis->bBulkCombine) {
				MGC_WriteCsr16(pBase, MGC_O_HDRC_RXMAXP, bEnd,
					       wPacketSize | ((min
							       (pEnd->
								wMaxPacketSizeRx,
								dwLength) /
							       wPacketSize) -
							      1) << 11);
			} else
#endif
			{
				MGC_WriteCsr16(pBase, MGC_O_HDRC_RXMAXP, bEnd,
					       wPacketSize);
			}
			MGC_WriteCsr8(pBase, MGC_O_HDRC_RXINTERVAL, bEnd,
				      bInterval);
		} else if (pThis->bIsMultipoint) {
			/* REVISIT:  test multipoint/not at compiletime ... */
			MGC_WriteCsr8(pBase, MGC_O_HDRC_TYPE0, 0, reg & 0xc0);
		}

		/* if not  flush & init toggle */
		WARN_ON(pEnd->bIsReady);
		pEnd->bIsReady = TRUE;

		wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd);
		if (wCsr & MGC_M_RXCSR_RXPKTRDY)
			WARN("rx%d, packet/%d ready?\n", bEnd,
			     MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCOUNT, bEnd));

// SCRUB (RX)
		/* twice in case of double packet buffering */
		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd,
			       MGC_M_RXCSR_FLUSHFIFO | MGC_M_RXCSR_CLRDATATOG |
			       MGC_M_RXCSR_RXPKTRDY);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd,
			       MGC_M_RXCSR_FLUSHFIFO | MGC_M_RXCSR_CLRDATATOG
			       | MGC_M_RXCSR_RXPKTRDY);

		if (usb_gettoggle(pUrb->dev, pEnd->bEnd, 0))
			MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd,
				       MGC_M_RXCSR_H_WR_DATATOGGLE
				       | MGC_M_RXCSR_H_DATATOGGLE |
				       MGC_M_RXCSR_RXPKTRDY);

		/* kick things off */
		if (bEnd) {
			int newcsr;

			newcsr = wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR,
						      bEnd);
			newcsr |= MGC_M_RXCSR_H_REQPKT;

			/* scrub any stale state */
			newcsr &= ~(MGC_M_CSR0_H_ERROR
				    | MGC_M_CSR0_H_RXSTALL
				    | MGC_M_CSR0_H_NAKTIMEOUT
				    | MGC_M_RXCSR_RXPKTRDY);

			if (usb_pipeint(nPipe))
				newcsr |= MGC_M_RXCSR_DISNYET;

#ifdef CONFIG_USB_INVENTRA_DMA
			if (bDmaOk) {
				newcsr &= ~MGC_M_RXCSR_H_REQPKT;
				newcsr |= MGC_M_RXCSR_H_AUTOREQ;
				newcsr |= (MGC_M_RXCSR_AUTOCLEAR |
					   MGC_M_RXCSR_DMAENAB
					   | (pDmaChannel->
					      bDesiredMode ? MGC_M_RXCSR_DMAMODE
					      : 0));
			}
#elif defined(CONFIG_USB_TI_CPPI_DMA)
			/* candidate for DMA */
			if (bDmaOk) {
				pDmaChannel->dwActualLength = 0L;
				pEnd->dwRequestSize = dwLength;

				/* AUTOREQ is in a DMA register */
				newcsr &= ~(MGC_M_RXCSR_AUTOCLEAR
					    | MGC_M_RXCSR_DMAMODE
					    | MGC_M_RXCSR_H_REQPKT
					    | MGC_M_RXCSR_H_AUTOREQ);
				if (newcsr != wCsr) {
					DBG(7, "RXCSR%d %04x\n", bEnd, newcsr);
					MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR,
						       bEnd,
						       newcsr |
						       MGC_M_RXCSR_RXPKTRDY);
				}

				if (usb_pipeisoc(nPipe))
					addr = pUrb->transfer_dma + pUrb->
						iso_frame_desc[0].offset;
				else
					addr = pUrb->transfer_dma;

				/* unless caller treats short rx transfers as
				 * errors, we dare not queue multiple transfers.
				 */
				bDmaOk =
				    pDmaController->
				    pfDmaProgramChannel(pDmaChannel,
							wPacketSize,
							usb_pipeisoc(nPipe)?
							2 : !(pUrb->
							transfer_flags &
							URB_SHORT_NOT_OK),
							addr,
							pEnd->dwRequestSize);
				if (!bDmaOk) {
					pDmaController->
					    pfDmaReleaseChannel(pDmaChannel);
					pDmaChannel = pEnd->pDmaChannel = NULL;
					newcsr &= ~MGC_M_RXCSR_DMAENAB;
					wCsr = MGC_ReadCsr16(pBase,
							     MGC_O_HDRC_RXCSR,
							     bEnd);
					pUrb->hcpriv = &pEnd->musb_mode[0];
				} else
					newcsr |= MGC_M_RXCSR_DMAENAB
					    | MGC_M_RXCSR_H_REQPKT;
			}
#endif
			if (newcsr != wCsr) {
				DBG(7, "RXCSR%d := %04x\n", bEnd, newcsr);
				MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR,
					       bEnd,
					       MGC_M_RXCSR_H_WZC_BITS | newcsr);
			}
		}
	}
}

/*
 * Try to stop traffic on the given local endpoint.
 * @param pThis the controller
 * @param bEnd the endpoint number.
 * Context: irqs blocked, controller lock held
 */
void MGC_HdrcStopEnd(struct musb *pThis, u8 bEnd)
{
	u16 wCsr;
	void __iomem *pBase = pThis->pRegs;
	const u8 reg = (bEnd) ? MGC_O_HDRC_RXCSR : MGC_O_HDRC_CSR0;
	struct musb_hw_ep *hw_ep = pThis->aLocalEnd + bEnd * 2;

// REVISIT probably worth calling this in other places, once the
// urb abort stuff is removed ...

// FIXME it's not just RX endpoints that need stopping ...

// SCRUB (RX -and- TX)
	/* clear the pending request */
	MGC_SelectEnd(pBase, bEnd);
	wCsr = MGC_ReadCsr16(pBase, reg, bEnd);
	wCsr &=
	    (bEnd) ? (~MGC_M_RXCSR_H_REQPKT) | MGC_M_RXCSR_RXPKTRDY :
	    ~MGC_M_CSR0_H_REQPKT;
	MGC_WriteCsr16(pBase, reg, bEnd, wCsr);

	while (!list_empty(&hw_ep->urb_list)) {
		struct urb *urb;

		urb = list_entry(hw_ep->urb_list.next, struct urb, urb_list);
		ERR("abort urb %p, dev %d\n", urb, urb->dev->devnum);
		musb_giveback(hw_ep, urb, -ESHUTDOWN);

		// REVISIT ... usbcore will abort things for us
		// if we disable() endpoints properly, so that this
		// routine should never see urbs still queued
		// (root hub currently has a big hole)
	}
}

/*
 * Service the default endpoint (ep0) as host.
 *
 * @param pThis this
 * @param wCount current byte count in FIFO
 * @param pUrb URB pointer for EP0
 * @return TRUE if more packets are required for this transaction
 */
static u8 musb_h_ep0_continue(struct musb *pThis, u16 wCount, struct urb *pUrb)
{
	u8 bMore = FALSE;
	u8 *pFifoDest = NULL;
	u16 wFifoCount = 0;
	struct musb_hw_ep *pEnd = &(pThis->aLocalEnd[0]);
	struct usb_ctrlrequest *pRequest =
	    (struct usb_ctrlrequest *)pUrb->setup_packet;

	if (MGC_END0_IN == pThis->bEnd0Stage) {
		/* we are receiving from peripheral */
		pFifoDest = pUrb->transfer_buffer + pUrb->actual_length;
		wFifoCount = min(wCount, ((u16)
					  (pUrb->transfer_buffer_length -
					   pUrb->actual_length)));
		if (wFifoCount < wCount)
			pUrb->status = -EOVERFLOW;

		if (wFifoCount)
			musb_read_fifo(pEnd, wFifoCount, pFifoDest);

		pUrb->actual_length += wCount;
		if (wCount < pEnd->wPacketSize) {
			/* always terminate on short read; it's
			 * rarely reported as an error.
			 */
			if ((pUrb->transfer_flags & URB_SHORT_NOT_OK)
			    && (pUrb->actual_length <
				pUrb->transfer_buffer_length))
				pUrb->status = -EREMOTEIO;
		} else if (pUrb->actual_length < pUrb->transfer_buffer_length)
			bMore = TRUE;
	} else {
		/* we are sending to peripheral */
		if ((MGC_END0_START == pThis->bEnd0Stage) &&
		    (pRequest->bRequestType & USB_DIR_IN)) {
			/* this means we just did setup; switch to IN */
			DBG(4, "start IN-DATA\n");
			pThis->bEnd0Stage = MGC_END0_IN;
			bMore = TRUE;

		} else if (pRequest->wLength
			   && ((MGC_END0_START == pThis->bEnd0Stage)
			       || (pThis->bEnd0Stage == MGC_END0_OUT))
			   && (pUrb->actual_length <
			       pUrb->transfer_buffer_length)
		    ) {
			pThis->bEnd0Stage = MGC_END0_OUT;
			pFifoDest = (u8 *) (pUrb->transfer_buffer +
					    pUrb->actual_length);
			wFifoCount = min(pEnd->wPacketSize, ((u16)
							     (pUrb->
							      transfer_buffer_length
							      -
							      pUrb->
							      actual_length)));
			DBG(3, "Sending %d bytes to %p\n", wFifoCount,
			    pFifoDest);
			if (wFifoCount)
				musb_write_fifo(pEnd, wFifoCount, pFifoDest);

			pEnd->dwRequestSize = wFifoCount;
			pUrb->actual_length += wFifoCount;
			bMore = TRUE;
		}
	}

	return bMore;
}

/*
 * Handle default endpoint interrupt as host. Only called in IRQ time
 * from the LinuxIsr() interrupt service routine.
 *
 * called with controller irqlocked
 */
irqreturn_t musb_h_ep0_irq(struct musb * pThis)
{
	struct urb *pUrb;
	u16 wCsrVal, wCount;
	int status = 0;
	void __iomem *pBase = pThis->pRegs;
	struct musb_hw_ep *pEnd = &pThis->aLocalEnd[0];
	u8 bComplete = FALSE;
	irqreturn_t retval = IRQ_NONE;
	u8 bInterval = 8;

	/* ep0 only has one queue, "in" */
	pUrb = next_in_urb(pEnd);

	MGC_SelectEnd(pBase, 0);
	wCsrVal = MGC_ReadCsr16(pBase, MGC_O_HDRC_CSR0, 0);
	wCount = MGC_ReadCsr8(pBase, MGC_O_HDRC_COUNT0, 0);

	DBG(4, "<== csr0 %04x, count %d, urb %p\n", wCsrVal, wCount, pUrb);

	/* if we just did status stage, we are done */
	if (MGC_END0_STATUS == pThis->bEnd0Stage) {
		retval = IRQ_HANDLED;
		bComplete = TRUE;
	}

	/* prepare status */
	if (wCsrVal & MGC_M_CSR0_H_RXSTALL) {
		DBG(6, "STALLING ENDPOINT\n");
		status = -EPIPE;

	} else if (wCsrVal & MGC_M_CSR0_H_ERROR) {
		DBG(2, "no response, csr0 %04x\n", wCsrVal);
		status = -EPROTO;

	} else if (wCsrVal & MGC_M_CSR0_H_NAKTIMEOUT) {
		DBG(2, "control NAK timeout\n");

		/* NOTE:  this code path would be a good place to PAUSE a
		 * control transfer, if another one is queued, so that
		 * ep0 is more likely to stay busy.
		 */
		MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, 0);
		retval = IRQ_HANDLED;
	}

	if (status) {
		DBG(6, "aborting\n");
		retval = IRQ_HANDLED;
		if (pUrb)
			pUrb->status = status;
		bComplete = TRUE;

		/* use the proper sequence to abort the transfer */
		if (wCsrVal & MGC_M_CSR0_H_REQPKT) {
			wCsrVal &= ~MGC_M_CSR0_H_REQPKT;
			MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, wCsrVal);
			wCsrVal &= ~MGC_M_CSR0_H_NAKTIMEOUT;
			MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, wCsrVal);
		} else {
			wCsrVal |= MGC_M_CSR0_FLUSHFIFO;
			MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, wCsrVal);
			MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, wCsrVal);
			wCsrVal &= ~MGC_M_CSR0_H_NAKTIMEOUT;
			MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, wCsrVal);
		}

		MGC_WriteCsr8(pBase, MGC_O_HDRC_NAKLIMIT0, 0, 0);

		/* clear it */
		MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, 0);
	}

	if (!pUrb) {
		/* stop endpoint since we have no place for its data, this
		 * SHOULD NEVER HAPPEN! */
		DBG(1, "no URB for end 0\n");

		MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, MGC_M_CSR0_FLUSHFIFO);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, MGC_M_CSR0_FLUSHFIFO);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, 0);

		/* start next URB that might be queued for it
		   musb_advance_urb_queue(pThis, pUrb, pEnd,
		   USB_DIR_IN); */
		goto done;
	}

	if (!bComplete) {
		/* call common logic and prepare response */
		if (musb_h_ep0_continue(pThis, wCount, pUrb)) {
			/* more packets required */
			wCsrVal = (MGC_END0_IN == pThis->bEnd0Stage) ?
			    MGC_M_CSR0_H_REQPKT : MGC_M_CSR0_TXPKTRDY;
			DBG(5, "more ep0 DATA, csr %04x\n", wCsrVal);
		} else {
			if (usb_pipein(pUrb->pipe))
				MGC_WriteCsr8(pBase, MGC_O_HDRC_NAKLIMIT0, 0,
					      bInterval);

			/* data transfer complete; perform status phase */
			wCsrVal = MGC_M_CSR0_H_STATUSPKT |
			    ((usb_pipeout(pUrb->pipe)) ? MGC_M_CSR0_H_REQPKT :
			     MGC_M_CSR0_TXPKTRDY);
			/* flag status stage */
			pThis->bEnd0Stage = MGC_END0_STATUS;

			DBG(5, "ep0 STATUS, csr %04x\n", wCsrVal);

		}
		MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, wCsrVal);
		retval = IRQ_HANDLED;
	}

	/* call completion handler if done */
	if (bComplete)
		musb_advance_urb_queue(pThis, pUrb, pEnd,
				       usb_pipein(pUrb->pipe));
      done:
	return retval;
}

/* Service a Tx-Available or dma completion irq for the endpoint */
void musb_host_tx(struct musb *pThis, u8 bEnd)
{
	int nPipe;
	u8 bDone = FALSE;
	u16 wTxCsrVal;
	size_t wLength = 0;
	u8 *pBuffer = NULL;
	struct urb *pUrb;
	struct musb_hw_ep *pEnd = pThis->aLocalEnd + bEnd * 2 - 1;
	u32 status = 0;
	void __iomem *pBase = pThis->pRegs;
	struct dma_channel *dma;
	struct musb_opmode *musb_mode;

	pUrb = next_out_urb(pEnd);

	MGC_SelectEnd(pBase, bEnd);
	wTxCsrVal = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd);

	// FIXME we seem to get these sometimes, not clear why...
	// ... usually right after a cppi completion irq, dma enabled
	// ... ideally, it's when the fifo empties after dma ...
	if (!pUrb) {
		DBG(1, "null urb, TXCSR%d = %04x\n", bEnd, wTxCsrVal);
		goto finish;
	}

	nPipe = pUrb->pipe;
	musb_mode = pUrb->hcpriv;
	dma = is_dma_capable()? pEnd->pDmaChannel : NULL;

	if (pUrb->status == 0)
		DBG(1, "OUT/TX%d end, csr %04x%s\n", bEnd, wTxCsrVal,
		    dma ? ", dma" : "");

	/* check for errors */
	if (wTxCsrVal & MGC_M_TXCSR_H_RXSTALL) {
		DBG(1, "TX end %d stall\n", bEnd);

		/* stall; record URB status */
		status = -EPIPE;

	} else if (wTxCsrVal & MGC_M_TXCSR_H_ERROR) {
		DBG(1, "TX data error on ep=%d\n", bEnd);

		status = -ETIMEDOUT;

	} else if (wTxCsrVal & MGC_M_TXCSR_H_NAKTIMEOUT) {
		DBG(1, "TX end=%d device not responding\n", bEnd);

		/* NOTE:  this code path would be a good place to PAUSE a
		 * transfer, if there's some other (nonperiodic) tx urb
		 * that could use this fifo.  (dma complicates it...)
		 */
		MGC_SelectEnd(pBase, bEnd);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0,
			       MGC_M_TXCSR_H_WZC_BITS | MGC_M_TXCSR_TXPKTRDY);
		goto finish;
	}

	if (status) {
		if (dma_channel_status(dma) == MGC_DMA_STATUS_BUSY) {
			dma->bStatus = MGC_DMA_STATUS_CORE_ABORT;
			(void)pThis->pDmaController->pfDmaAbortChannel(dma);
		}

		pEnd->fifo_flush_check = 0;
		/* do the proper sequence to abort the transfer in the
		 * usb core; the dma engine should already be stopped.
		 */
// SCRUB (TX)
		wTxCsrVal &= ~(MGC_M_TXCSR_FIFONOTEMPTY
			       | MGC_M_TXCSR_AUTOSET
			       | MGC_M_TXCSR_H_ERROR
			       | MGC_M_TXCSR_H_RXSTALL
			       | MGC_M_TXCSR_H_NAKTIMEOUT);
		wTxCsrVal |= MGC_M_TXCSR_FLUSHFIFO;

		MGC_SelectEnd(pBase, bEnd);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, wTxCsrVal);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, wTxCsrVal);
		MGC_WriteCsr8(pBase, MGC_O_HDRC_TXINTERVAL, bEnd, 0);

		bDone = TRUE;
	}

	if (dma_channel_status(dma) == MGC_DMA_STATUS_BUSY) {
		/* SHOULD NOT HAPPEN */
		DBG(3, "bogus TX%d busy dma, csr %04x\n", bEnd, wTxCsrVal);
		goto finish;

	}

	/* REVISIT this looks wrong... */
	if (!status || musb_mode->dma || usb_pipeisoc(nPipe)) {

		if (musb_mode->dma)
			wLength = pEnd->pDmaChannel->dwActualLength;
		else
			wLength = pEnd->dwRequestSize;
		pEnd->dwOffset += wLength;

		if (usb_pipeisoc(nPipe)) {
			struct usb_iso_packet_descriptor *d;

			if (musb_mode->dma) {
				do {
					d = &pUrb->iso_frame_desc[
						pEnd->dwIsoPacket];
					d->status = 0;
				} while (++pEnd->dwIsoPacket <
						pUrb->number_of_packets);
				bDone = TRUE;
				pEnd->dwOffset = pEnd->pDmaChannel->
							dwActualLength;
				DBG(3, "TX%d dma, csr %04x %d\n",
					bEnd, wTxCsrVal, pEnd->pDmaChannel->
					dwActualLength);
				pEnd->pDmaChannel->dwActualLength = 0;

			} else {
				d = pUrb->iso_frame_desc + pEnd->dwIsoPacket;
				d->actual_length += pEnd->dwRequestSize;
				if (++pEnd->dwIsoPacket >=
					pUrb->number_of_packets) {
					bDone = TRUE;
				} else {
					d++;
					pBuffer = pUrb->transfer_buffer +
								d->offset;
					wLength = d->length;
				}
			}
		} else if (musb_mode->dma) {
			bDone = TRUE;
			pEnd->pDmaChannel->dwActualLength = 0L;
		} else {
			/* see if we need to send more data, or ZLP */
			if (pEnd->dwRequestSize < pEnd->wPacketSize)
				bDone = TRUE;
			else if (pEnd->dwOffset == pUrb->transfer_buffer_length
				 && !(pUrb->transfer_flags & URB_ZERO_PACKET))
				bDone = TRUE;
			if (!bDone) {
				pBuffer = pUrb->transfer_buffer
				    + pEnd->dwOffset;
				wLength = pUrb->transfer_buffer_length
				    - pEnd->dwOffset;
			}
		}
	}

	/* urb->status != -EINPROGRESS means request has been faulted,
	 * so we must abort this transfer after cleanup
	 */
	if (pUrb->status != -EINPROGRESS) {
		bDone = TRUE;
		if (status == 0) {
			status = pUrb->status;
		}
	}

	if (bDone) {
		pUrb->status = status;
		pUrb->actual_length = pEnd->dwOffset;
		musb_tx_tasklet_invoke(pThis, pEnd);
	} else if (!musb_mode->dma) {
		/* PIO:  start next packet in this URB */
		wLength = min(pEnd->wPacketSize, (u16) wLength);
		if (wLength)
			musb_write_fifo(pEnd, wLength, pBuffer);
		pEnd->dwRequestSize = wLength;

		MGC_SelectEnd(pBase, bEnd);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd,
			       MGC_M_TXCSR_H_WZC_BITS | MGC_M_TXCSR_TXPKTRDY);
	} else
		DBG(1, "not complete, but dma enabled?\n");

      finish:
	return;
}

/*
 * Service an Rx-Ready interrupt for the given endpoint; see section 18.2.1
 * of the manual for details.
 *
 * @param pThis instance pointer
 * @param bEnd local endpoint
 */
void musb_host_rx(struct musb *pThis, u8 bEnd)
{
	struct urb *pUrb;
	struct musb_hw_ep *pEnd = &(pThis->aLocalEnd[bEnd * 2]);
	size_t xfer_len = 0;
	void __iomem *pBase = pThis->pRegs;
	int nPipe;
	u16 wRxCsrVal, wVal;
	u8 bIsochError = FALSE;
	u8 bDone = FALSE;
	u32 status;
	struct dma_channel *dma;
	struct usb_iso_packet_descriptor *piso_desc;
	struct musb_opmode *musb_mode;

	MGC_SelectEnd(pBase, bEnd);

	pUrb = next_in_urb(pEnd);
	dma = is_dma_capable()? pEnd->pDmaChannel : NULL;
	status = 0;

	wVal = wRxCsrVal = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd);

	if (!pUrb) {
		/* SHOULD NOT HAPPEN */
		DBG(1, "bogus RX%d ready, csr %04x\n", bEnd, wVal);
		goto finish;
	}

	nPipe = pUrb->pipe;
	musb_mode = pUrb->hcpriv;

	DBG(4, "<== hw %d rxcsr %04x, urb actual %d (+dma %d)\n", bEnd,
	    wRxCsrVal, pUrb->actual_length, dma ? dma->dwActualLength : 0);

	/* check for errors, concurrent stall & unlink is not really
	 * handled yet! */
	if (wRxCsrVal & MGC_M_RXCSR_H_RXSTALL) {
		DBG(3, "RX end %d STALL\n", bEnd);

		/* stall; record URB status */
		status = -EPIPE;

	} else if (wRxCsrVal & MGC_M_RXCSR_H_ERROR) {
		DBG(3, "end %d RX proto error\n", bEnd);

		status = -EPROTO;
		MGC_WriteCsr8(pBase, MGC_O_HDRC_RXINTERVAL, bEnd, 0);

	} else if (wRxCsrVal & MGC_M_RXCSR_DATAERROR) {

		if (PIPE_ISOCHRONOUS != pEnd->out_traffic_type) {
			/* NOTE this code path would be a good place to PAUSE a
			 * transfer, if there's some other (nonperiodic) rx urb
			 * that could use this fifo.  (dma complicates it...)
			 */
			DBG(6, "RX end=%d device not responding\n", bEnd);
			MGC_SelectEnd(pBase, bEnd);
			MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd,
				       MGC_M_RXCSR_H_WZC_BITS
				       | MGC_M_RXCSR_H_REQPKT);

			goto finish;
		} else {
			DBG(3, "bEnd=%d Isochronous error\n", bEnd);
			// packet[i].status = -EILSEQ;
			bIsochError = TRUE;
		}
	}

	if (status) {
		/* clean up dma and collect transfer count */
		if (dma_channel_status(dma) == MGC_DMA_STATUS_BUSY) {
			dma->bStatus = MGC_DMA_STATUS_CORE_ABORT;
			(void)pThis->pDmaController->pfDmaAbortChannel(dma);
			xfer_len = dma->dwActualLength;
			dma->dwActualLength = 0L;
		}
	}

	if (status) {
		wVal &= ~(MGC_M_RXCSR_H_ERROR
			  | MGC_M_RXCSR_DATAERROR
			  | MGC_M_RXCSR_H_RXSTALL
			  | MGC_M_RXCSR_RXPKTRDY | MGC_M_RXCSR_H_REQPKT);
		MGC_SelectEnd(pBase, bEnd);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd,
			       wVal | MGC_M_RXCSR_RXPKTRDY);
		MGC_WriteCsr8(pBase, MGC_O_HDRC_RXINTERVAL, bEnd, 0);

		bDone = TRUE;
	}

	if (dma_channel_status(dma) == MGC_DMA_STATUS_BUSY) {
		/* SHOULD NOT HAPPEN */
		DBG(3, "RX%d busy\n", bEnd);
		dma->bStatus = MGC_DMA_STATUS_CORE_ABORT;
		(void)pThis->pDmaController->pfDmaAbortChannel(dma);
		xfer_len = dma->dwActualLength;
		dma->dwActualLength = 0L;
		bDone = TRUE;
		/* goto finish; */
	}

	/* thorough shutdown for now ... given more precise fault handling
	 * and better queueing support, we might keep a DMA pipeline going
	 * while processing this irq for earlier completions.
	 */

	if (wRxCsrVal & MGC_M_RXCSR_H_REQPKT) {
		/* REVISIT this happened for a while on some short reads...
		 * the cleanup still needs investigation... looks bad...
		 * and also duplicates dma cleanup code above ...
		 */
		if (dma_channel_status(dma) == MGC_DMA_STATUS_BUSY) {
			dma->bStatus = MGC_DMA_STATUS_CORE_ABORT;
			(void)pThis->pDmaController->pfDmaAbortChannel(dma);
			xfer_len = dma->dwActualLength;
			dma->dwActualLength = 0L;
		}

		DBG(3, "RXCSR%d %04x, reqpkt, len %d%s\n", bEnd, wRxCsrVal,
		    xfer_len, dma ? ", dma" : "");
		wRxCsrVal &= ~MGC_M_RXCSR_H_REQPKT;

		MGC_SelectEnd(pBase, bEnd);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd,
			       MGC_M_RXCSR_H_WZC_BITS | wRxCsrVal);
	}

	if (musb_mode->dma) {
#if 0
		wRxCsrVal &= ~MGC_M_RXCSR_DMAENAB;
		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd,
			       MGC_M_RXCSR_H_WZC_BITS | wRxCsrVal);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd,
			       MGC_M_RXCSR_H_WZC_BITS | wRxCsrVal);
		DBG(4, "RXCSR%d %04x, dma off, %04x\n", bEnd, wRxCsrVal,
		    MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd));
#endif
		xfer_len = dma->dwActualLength;
		DBG(3, "RXCSR%d %04x, iso len %d%s\n", bEnd, wRxCsrVal,
		    xfer_len, dma ? ", dma" : "");
		dma->dwActualLength = 0L;
		while ((pEnd->dwIsoPacket < pUrb->number_of_packets) &&
			 (usb_pipeisoc(nPipe))) {
			piso_desc = &pUrb->iso_frame_desc[pEnd->dwIsoPacket];
			pUrb->actual_length +=
				pEnd->iso_desc[pEnd->dwIsoPacket].length;
			piso_desc->actual_length =
				pEnd->iso_desc[pEnd->dwIsoPacket].length;
			if (bIsochError) {
				piso_desc->status = -EILSEQ;
				pUrb->error_count++;
			} else
				piso_desc->status = 0;

			++pEnd->dwIsoPacket;
		}

		if (usb_pipeisoc(nPipe))
			goto advance;
		else
			bDone = TRUE;

	} else if (!bDone && pUrb->status == -EINPROGRESS) {

		/* if no errors, be sure a packet is ready for unloading */
		if (!(wRxCsrVal & MGC_M_RXCSR_RXPKTRDY)) {
			status = -EPROTO;
			DBG(1, "Rx interrupt with no errors or packet!\n");

			// FIXME this is another "SHOULD NEVER HAPPEN"
			// like the "no URB" case below

// SCRUB (RX)
			/* do the proper sequence to abort the transfer */
			MGC_SelectEnd(pBase, bEnd);
			wVal &= ~MGC_M_RXCSR_H_REQPKT;
			MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd,
				       wVal | MGC_M_RXCSR_RXPKTRDY);
		}

		/* we are expecting IN packets */
		if (pUrb) {
			if (!bDone) {
				bDone = musb_host_packet_rx(pThis, pUrb,
							    bEnd, bIsochError);
				DBG(6, "read %spacket\n", bDone ? "last " : "");
			}
		} else {
			/* THIS SHOULD NEVER HAPPEN */
			/* stop endpoint since we have no place for its data */
			DBG(1, "no URB on end %d Rx!\n", bEnd);

// SCRUB (RX)
			MGC_SelectEnd(pBase, bEnd);
			wVal |= MGC_M_RXCSR_FLUSHFIFO;
			wVal &= ~MGC_M_RXCSR_H_REQPKT;
			MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd,
				       wVal | MGC_M_RXCSR_RXPKTRDY);
			MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd,
				       wVal | MGC_M_RXCSR_RXPKTRDY);

			wVal &= ~(MGC_M_RXCSR_FLUSHFIFO | MGC_M_RXCSR_RXPKTRDY);
			MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, wVal);
		}
	} else if (!bDone) {
		bDone = TRUE;
		status = pUrb->status;
	}

	pUrb->actual_length += xfer_len;
	pEnd->dwOffset += xfer_len;

	if (bDone) {
		pUrb->status = status;
		goto advance;
	} else {
		DBG(5, "not done yet, setup next in transaction\n");

		/* continue by clearing RxPktRdy and setting ReqPkt */
		MGC_SelectEnd(pBase, bEnd);
		wVal &= ~MGC_M_RXCSR_RXPKTRDY;
		wVal |= MGC_M_RXCSR_H_REQPKT | MGC_M_RXCSR_H_WZC_BITS;
		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, wVal);
	}

      finish:
	return;

      advance:
	musb_advance_urb_queue(pThis, pUrb, pEnd, USB_DIR_IN);
	return;
}

/*
 * Find an endpoint for the given pipe
 */
static struct musb_hw_ep *musb_find_ep(struct musb *pThis, struct urb *pUrb)
{
	unsigned int pipe = pUrb->pipe;
	unsigned int nOut;
	struct musb_hw_ep *pEnd;
	unsigned long flags;
	int nEnd;
	s32 dwDiff;
	u16 wBestDiff;
	int nBestEnd;
	u16 wPacketSize;
	u8 bEnd;
	u8 bAddress, level;

	/* control is always EP0 */
	if (usb_pipecontrol(pipe))
		return pThis->aLocalEnd;

	nOut = usb_pipeout(pipe);
#ifdef CONFIG_MUSB_RESERVE_ISO_EP
	/* If we reserve EP for ISO bulk Reservation is for sure to be done*/
	if (usb_pipebulk(pipe)) {
		if (nOut)
			return pThis->bulk_tx_end;
		else
			return pThis->bulk_rx_end;
	}
#endif
	wBestDiff = 0xffff;
	nBestEnd = -1;
	wPacketSize = usb_maxpacket(pUrb->dev, pipe, nOut);
	bEnd = usb_pipeendpoint(pipe);
	bAddress = usb_pipedevice(pipe);

	/* for periodic, use exact match or something ok but unclaimed */
	spin_lock_irqsave(&pThis->Lock, flags);
	for (nEnd = (nOut) ? 1 : 2; nEnd < pThis->bEndCount; nEnd += 2) {
		u8 type;

		pEnd = &pThis->aLocalEnd[nEnd];
		if (nOut) {
			if ((pEnd == pThis->bulk_tx_end) &&
				(!usb_pipebulk(pipe)))
				continue;
#ifdef CONFIG_MUSB_RESERVE_ISO_EP
			if ((nEnd == (ISO_EP * 2 - 1)) && (!usb_pipeisoc(pipe)))
				continue;
#endif

			if (pEnd->wMaxPacketSizeTx < wPacketSize) {
				DBG (4, "Asked for %d having %d\n", wPacketSize,
						pEnd->wMaxPacketSizeTx);
				continue;
			}
			dwDiff = pEnd->wMaxPacketSizeTx - wPacketSize;
		} else {
			if ((pEnd == pThis->bulk_rx_end) &&
			    (!usb_pipebulk(pipe)))
				continue;
#ifdef CONFIG_MUSB_RESERVE_ISO_EP
			if ((nEnd == (ISO_EP * 2)) && (!usb_pipeisoc(pipe)))
				continue;
#endif

			if (pEnd->wMaxPacketSizeRx < wPacketSize) {
				DBG (4, "Asked for %d having %d\n", wPacketSize,
						pEnd->wMaxPacketSizeRx);
				continue;
			}
			dwDiff = pEnd->wMaxPacketSizeRx - wPacketSize;
		}

		type = usb_pipein(pipe)
		    ? pEnd->in_traffic_type : pEnd->out_traffic_type;

		/* exact match */
		if ((usb_pipetype(pipe) == type)
		    && (pEnd->bEnd == bEnd)
		    && (pEnd->bAddress == bAddress)) {
			nBestEnd = nEnd;
			break;
		}

		/* unclaimed, and a closer match? */
		if (!pEnd->bIsClaimed && (wBestDiff > dwDiff)) {
			wBestDiff = (u16) dwDiff;
			nBestEnd = nEnd;
		}
	}
	spin_unlock_irqrestore(&pThis->Lock, flags);

	if (nBestEnd == -1)
		level = 1;
	else
		level = 4;
	DBG(level, "(out=%d, size=%d, proto=%d, addr=%d, end=%d, urb=%lx) = %d",
	    nOut, wPacketSize, usb_pipetype(pipe),
	    bAddress, bEnd, (unsigned long)pUrb, nBestEnd);

	if (nBestEnd >= 0) {
		pEnd = pThis->aLocalEnd + nBestEnd;
		if ((pEnd != pThis->bulk_tx_end)
		    && (pEnd != pThis->bulk_rx_end)) {
			if (nOut)
				pEnd->out_traffic_type = usb_pipetype(pipe);
			else
				pEnd->in_traffic_type = usb_pipetype(pipe);
		} else {
			if (!usb_pipebulk(pipe))
				return NULL;
		}

		return pEnd;
	}
#ifndef CONFIG_MUSB_RESERVE_ISO_EP
	else {
		if (usb_pipebulk(pipe)) {
			if (nOut)
				return pThis->bulk_tx_end;
			else
				return pThis->bulk_rx_end;
		}
	}
#endif
	return NULL;
}

/*
 * Submit an URB, either to the virtual root hut or to a real device;
 * This is called by the Linux USB core. TSubmit Urb lock pThis
 * and the End to use, so make sure the caller releases its locks.
 *
 * also set the hcpriv member to the localEnd
 *
 * @param pUrb URB pointer (urb = USB request block data structure)
 * @return status code (0 succes)
 */
static int musb_submit_urb(struct urb *pUrb, gfp_t iMemFlags)
{
	unsigned long flags;
	unsigned int pipe = pUrb->pipe;
	struct musb_hw_ep *pEnd;
	struct musb *pThis;
	int nEnd, idle = 0;
	int status;
	enum dma_data_direction maptype = DMA_NONE;

	pThis = pUrb->dev->bus->hcpriv;

	/* root hub requests are OK except in peripheral-only mode */
	if (pThis->board_mode == MUSB_PERIPHERAL)
		return -ENODEV;
	if (!pUrb->dev->parent)
		return MGC_VirtualHubSubmitUrb(&(pThis->RootHub), pUrb);

	/* for other devices, host role must be active */
	if (!is_host_active(pThis))
		return -ENODEV;

	/* find appropriate local endpoint to do it */
	pEnd = musb_find_ep(pThis, pUrb);
	if (!pEnd)
		return -EBUSY;
	nEnd = pEnd->bLocalEnd;

	DBG(6, "pUrb=%p, end=%d, bufsize=%d\n",
	    pUrb, nEnd, pUrb->transfer_buffer_length);

	/* setup DMA mapping, if needed (does dcache sync)
	 * insist on either a PIO or DMA buffer
	 */
	if (is_dma_capable()
	    && nEnd && pThis->controller->dma_mask
	    /*&& pUrb->transfer_buffer_length >= MIN_DMA_REQUEST) { */
	    /* To be removed later on completion of restructuring */
	    && (pUrb->transfer_buffer_length >= 32)) {
		if (!(pUrb->transfer_flags & URB_NO_TRANSFER_DMA_MAP)) {
			DBG(4, "end %d No DMA Map Length %d\n", nEnd,
				pUrb->transfer_buffer_length);
			maptype = usb_pipein(pipe)
			    ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
			pUrb->transfer_dma = dma_map_single(pThis->controller,
							    pUrb->
							    transfer_buffer,
							    pUrb->
							    transfer_buffer_length,
							    maptype);
		}

		pUrb->hcpriv = &pEnd->musb_mode[1];
	} else if (pUrb->transfer_buffer == NULL
		   && pUrb->transfer_buffer_length != 0) {
		/* FIXME release pEnd claim, for periodic endpoint */
		return -EINVAL;
	} else
		pUrb->hcpriv = &pEnd->musb_mode[0];

	/* if no root device, assume this must be it */
	if (!pThis->pRootDevice)
		pThis->pRootDevice = usb_get_dev(pUrb->dev);

	/* reserve new periodic bandwidth, unless this endpoint has
	 * already had its bandwidth reserved.
	 *
	 * FIXME but do it right.  For now we can just rely on the
	 * fact that we don't make enough endpoints available to
	 * overcommit bandwidth except maybe for fullspeed ISO.
	 * Two low-rate interrupt transfers and we're full...
	 */
	if (usb_pipeisoc(pipe) || usb_pipeint(pipe)) {
		pUrb->start_frame = musb_readw(pThis->pRegs, MGC_O_HDRC_FRAME)
		    + pUrb->interval;
	}

	/* FIXME for reserved bulk endpoints, stick it in the relevant
	 * ring of endpoints ...
	 */

	DBG(6, "end %d claimed for type=%d, addr=%d, end=%d\n", nEnd,
	    usb_pipetype(pipe), usb_pipedevice(pipe), usb_pipeendpoint(pipe));

	/*pEnd = &(pThis->aLocalEnd[nEnd]); */

	/* increment reference counts, neither urb nor device may vanish yet */
	pUrb = usb_get_urb(pUrb);
	usb_get_dev(pUrb->dev);

	/* queue & start */
	spin_lock_irqsave(&pThis->Lock, flags);
	if (usb_pipein(pipe) || pEnd->bIsSharedFifo)
		idle = !pEnd->in_busy && list_empty(&pEnd->in_urb_list);
	else
		idle = !pEnd->out_busy && list_empty(&pEnd->out_urb_list);

	pEnd->bIsClaimed = 1;

	/* assign the URB to the endpoint */
	spin_lock(&pUrb->lock);
	if (unlikely(pUrb->reject)) {
		INIT_LIST_HEAD(&pUrb->urb_list);
		status = -EPERM;
	} else {
		status = 0;
		pUrb->error_count = 0;
		list_add_tail(&pUrb->urb_list, &pEnd->urb_list);
		atomic_inc(&pUrb->use_count);
	}
	spin_unlock(&pUrb->lock);
	if (status) {
		usb_put_dev(pUrb->dev);
		usb_put_urb(pUrb);
		goto unmap;
	}

	DBG(4, "submit %p hw%d, dev%d ep%d%s, len %d\n",
	    pUrb, nEnd,
	    usb_pipedevice(pipe),
	    usb_pipeendpoint(pipe),
	    usb_pipein(pipe) ? "in" : "out", pUrb->transfer_buffer_length);

	if (idle)
		musb_start_urb(pThis, pEnd);
	spin_unlock_irqrestore(&pThis->Lock, flags);

	return status;
      unmap:
	spin_unlock_irqrestore(&pThis->Lock, flags);
	pUrb->hcpriv = NULL;
	if (is_dma_capable() && maptype != DMA_NONE)
		dma_unmap_single(pThis->controller, pUrb->transfer_dma,
				 pUrb->transfer_buffer_length, maptype);
	return status;
}

/*
 * abort a transfer that's at the head of a hardware queue.
 * called with controller locked, irqs blocked
 */
static int musb_cleanup_urb(struct urb *urb, struct musb_hw_ep *ep, int is_in)
{
	unsigned hw_end = ep->bLocalEnd;
	void __iomem *regs = ep->musb->pRegs;
	u16 csr;
	int status = 0;
	struct musb_opmode *musb_mode = urb->hcpriv;

	MGC_SelectEnd(ep->musb->pRegs, hw_end);
	/* turn off DMA requests, discard state, stop polling ... */
	if (is_in) {

/* SCRUB (RX) */
		csr = MGC_ReadCsr16(regs, MGC_O_HDRC_RXCSR, hw_end);
		csr &= ~(MGC_M_RXCSR_AUTOCLEAR
			 | MGC_M_RXCSR_H_AUTOREQ
			 | MGC_M_RXCSR_H_REQPKT
			 | MGC_M_RXCSR_H_RXSTALL
			 | MGC_M_RXCSR_DATAERROR
			 | MGC_M_RXCSR_H_ERROR | MGC_M_RXCSR_RXPKTRDY);
		csr |= MGC_M_RXCSR_FLUSHFIFO;
		MGC_WriteCsr16(regs, MGC_O_HDRC_RXCSR, hw_end, csr);
		MGC_WriteCsr16(regs, MGC_O_HDRC_RXCSR, hw_end, csr);
	} else {
/* SCRUB (TX) */
		csr = MGC_ReadCsr16(regs, MGC_O_HDRC_TXCSR, hw_end);
		csr &= ~(MGC_M_TXCSR_AUTOSET
			 | MGC_M_TXCSR_H_RXSTALL
			 | MGC_M_TXCSR_H_NAKTIMEOUT
			 | MGC_M_TXCSR_H_ERROR
			 | MGC_M_TXCSR_FIFONOTEMPTY | MGC_M_TXCSR_TXPKTRDY);
		csr |= MGC_M_TXCSR_FLUSHFIFO;
		MGC_WriteCsr16(regs, MGC_O_HDRC_TXCSR, hw_end, csr);
		MGC_WriteCsr16(regs, MGC_O_HDRC_TXCSR, hw_end, csr);
		ep->fifo_flush_check = 0;
	}
	if (musb_mode->dma) {
		status =
		    ep->musb->pDmaController->pfDmaAbortChannel(ep->
								pDmaChannel);
		DBG(status ? 1 : 3, "abort %cX%d DMA for urb %p --> %d\n",
		    is_in ? 'R' : 'T', ep->bLocalEnd, urb, status);
	}
	if (ep->pDmaChannel)
	WARN_ON(dma_channel_status(ep->pDmaChannel) == MGC_DMA_STATUS_BUSY);
	if (status == 0)
		musb_giveback(ep, urb, 0);

	ep->bIsReady = 0;

	return status;
}

/*
 * Cancel URB.
 * @param pUrb URB pointer
 */
static int musb_unlink_urb(struct urb *pUrb, int status)
{
	struct musb_opmode *musb_mode;
	struct musb *musb;
	struct musb_hw_ep *ep;
	unsigned i;
	unsigned long flags;
	u8 busy = 0, is_in = pUrb->pipe & USB_DIR_IN;

	DBG(1, "urb=%p, dev%d ep%d%s\n", pUrb,
	    usb_pipedevice(pUrb->pipe),
	    usb_pipeendpoint(pUrb->pipe),
	    usb_pipein(pUrb->pipe) ? "in" : "out");

	/* sanity */
	if (!pUrb || !pUrb->hcpriv)
		return -EINVAL;
	if (!pUrb->dev || !pUrb->dev->bus)
		return -ENODEV;
	musb_mode = pUrb->hcpriv;
	musb = pUrb->dev->bus->hcpriv;
	if (!musb)
		return -ENODEV;

	spin_lock_irqsave(&musb->Lock, flags);

	for (i = 0, ep = musb->aLocalEnd; i < MUSB_C_NUM_EPS; i++, ep++) {
		struct urb *urb;

		list_for_each_entry(urb, &ep->urb_list, urb_list) {
			if (urb == pUrb)
				goto found;
		}
	}
	status = -ENOENT;
	goto done;

      found:
	spin_lock(&pUrb->lock);
	if (musb_mode->ep != ep)
		status = -ENOENT;
	else if (pUrb->status != -EINPROGRESS)
		status = -EBUSY;
	else {
		pUrb->status = status;
		status = 0;
	}
	spin_unlock(&pUrb->lock);

	if (status)
		goto done;

	busy = (is_in) ? ep->in_busy : ep->out_busy;
	busy = (busy) ? ((is_in) ? (pUrb == next_in_urb(ep)) :
			 (pUrb == next_out_urb(ep))) : 0;
	/* if it is a request to the root hub, delegate */
	if (pUrb->dev == musb->RootHub.pDevice) {
		spin_unlock_irqrestore(&musb->Lock, flags);
		/* screwey locking */
		return MGC_VirtualHubUnlinkUrb(&(musb->RootHub), pUrb);
	}

	/* anything not at the head of the queue can just be given back,
	 * else cleanup pending dma etc
	 */
	if (busy) {
		status = musb_cleanup_urb(pUrb, ep, pUrb->pipe & USB_DIR_IN);
		if (!list_empty(&ep->urb_list))
			musb_start_urb(musb, ep);
	} else {
		musb_giveback(ep, pUrb, 0);
		status = 0;
	}

      done:
	spin_unlock_irqrestore(&musb->Lock, flags);
	return status;
}

// 2.6.current can't work without a new version of this routine
// #error rewrite host endpoint disable routine

/* disable an endpoint */
static void musb_h_disable(struct usb_device *udev, int epnum)
{
	unsigned long flags;
	struct musb *musb = udev->bus->hcpriv;
	struct musb_hw_ep *ep;
	unsigned do_wait = 0;
	u8 is_in = epnum & USB_DIR_IN;
	u8 epn = epnum & 0x0f;
	struct urb *urb, *next = (struct urb *)0xDEADBEEF, *prev = NULL;
	unsigned pipe;
	u16 csr = 0;

	if (((is_in) && (epn >= MUSB_C_NUM_EPR)) ||
		((!is_in) && (epn >= MUSB_C_NUM_EPT)))
		return;

	/* FIXME 2.6.current passes "struct usb_host_endpoint *hep" as the
	 * parameter, not "epnum" ... and the endpoint's URBs are provided
	 * in that structure, so no searching is needed.  (Only a check to
	 * see if it's at the front of a hardware endpoint's queue ...)
	 */
	spin_lock_irqsave(&musb->Lock, flags);
	csr = MGC_ReadCsr16(musb->pRegs, MGC_O_HDRC_RXCSR, epn);

	if (is_in)
		ep = &musb->aLocalEnd[epn ? epn*2: 0];
	else
		ep = &musb->aLocalEnd[epn ? epn*2 -1 : 0];

	next = (is_in) ? next_in_urb(ep) : next_out_urb(ep);
	while ((urb = (is_in) ? next_in_urb(ep) : next_out_urb(ep))
		&& (prev != urb)) {

		prev = urb;
		if (urb->dev != udev)
			continue;
		pipe = urb->pipe;
		if ((pipe & USB_DIR_IN) != is_in)
			continue;
		if (usb_pipeendpoint(pipe) != epn)
			continue;
		/* easy case: the hardware's not touching it */
		if (next != urb) {
			musb_giveback(ep, urb, -ESHUTDOWN);
		} else {

			/* make software (then hardware) stop ASAP */
			spin_lock(&urb->lock);
			if (urb->status == -EINPROGRESS)
				urb->status = -ESHUTDOWN;
			spin_unlock(&urb->lock);

			/* cleanup */
			musb_cleanup_urb(urb, ep, urb->pipe & USB_DIR_IN);
			next = NULL;
		}
		do_wait++;
	}

	if (list_empty(&ep->urb_list)) {
		/* Free up any local ISO messaging buffers */
		if (ep->iso_desc)
			kfree (ep->iso_desc);
		ep->iso_desc = NULL;
		ep->num_iso_desc = 0;
		if (is_in) {
			csr &= ~MGC_M_RXCSR_RXPKTRDY;
			MGC_WriteCsr16(musb->pRegs, MGC_O_HDRC_RXCSR, epn,
				csr|MGC_M_RXCSR_FLUSHFIFO);
		}
	} else if (next == NULL)
		musb_start_urb (musb, ep);

	spin_unlock_irqrestore(&musb->Lock, flags);

	/* REVISIT gives (most) irqs a chance to trigger */
	if (do_wait)
		msleep(10);
}

static void *musb_host_alloc_buffer(struct usb_bus *pBus, size_t nSize,
				    gfp_t iMemFlags, dma_addr_t * pDmaAddress)
{
	return musb_alloc_buffer(pBus->hcpriv, nSize, iMemFlags, pDmaAddress);
}

static void musb_host_free_buffer(struct usb_bus *pBus, size_t nSize,
				  void *address, dma_addr_t dma)
{
	musb_free_buffer(pBus->hcpriv, nSize, address, dma);
}

/*
 * Get the current frame number
 * @param usb_dev pointer to USB device
 * @return frame number
 */
static int musb_h_get_frame_number(struct usb_device *pDevice)
{
	struct musb *pThis = pDevice->bus->hcpriv;

	return musb_readw(pThis->pRegs, MGC_O_HDRC_FRAME);
}

/* FIXME -- switchover to use the hcd glue layer;
 * define root hub support with hub suspend/resume calls
 *
 * latest mentor code has some of that, plus ULPI calls
 *
 * also, kernel.org interfaces now pass a usb_host_endpoint
 * handle around; "struct hcd_dev" is gone.
 */
struct usb_operations musb_host_bus_ops = {
	.get_frame_number = musb_h_get_frame_number,
	.submit_urb = musb_submit_urb,
	.unlink_urb = musb_unlink_urb,
	.buffer_alloc = musb_host_alloc_buffer,
	.buffer_free = musb_host_free_buffer,
	.disable = musb_h_disable,
};
