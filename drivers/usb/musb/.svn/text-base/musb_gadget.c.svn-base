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

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/stat.h>
#include <linux/dma-mapping.h>

#include "musbdefs.h"

#ifdef CONFIG_USB_MUSB_OTG
#include "otg.h"
#endif

static void musb_gadget_fifo_flush(struct usb_ep *ep);
/* MUSB PERIPHERAL status 3-mar:
 *
 * - EP0 seems solid.  It passes both USBCV and usbtest control cases.
 *   (Minor glitches running the remote wakeup tests, which seem to live
 *   in USBCV; wakeup of Linux hosts works.)
 *
 * - Mass storage behaved ok when last tested.  Network traffic patterns
 *   (with lots of short transfers etc) aren't yet reliable
 *
 * - TEMPORARY STATUS:
 *     + RXDMA broke recently
 *     + still didn't test DMA with double buffering
 *
 * - TX/IN
 *     + both pio and dma behave in with network and g_zero tests
 *     + no dma throughput issues other than no-hw-queueing
 *     + failed with FLAT_REG
 *     + seems to behave with double buffering
 *     + with gadgetfs + AIO, requests got lost?
 *
 * - RX/OUT
 *     + both pio and dma behave in with network and g_zero tests
 *     + dma is slow in typical case (short_not_ok is clear)
 *     + double buffering ok with PIO
 *     + double buffering *FAILS* with DMA, wrong data bytes sometimes
 *     + request lossage observed with gadgetfs
 *
 * - ISO not tested ... might work, but only weakly isochronous
 *
 * - Gadget driver disabling of softconnect during bind() is ignored; so
 *   drivers can't hold off host requests until userspace is ready.
 *   (Workaround:  they can turn it off later.)
 */

// #define  DISABLE_RXDMA

/**************************************************************************
Handling completion
**************************************************************************/

/*
 * Immediately complete a request.
 *
 * @param pRequest the request to complete
 * @param status the status to complete the request with
 * Context: controller locked, IRQs blocked.
 */
void musb_g_giveback(struct musb_ep *ep,
		     struct usb_request *pRequest,
		     int status)
__releases(ep->musb->Lock) __acquires(ep->musb->Lock)
{
	struct musb_request *req;
	struct musb *musb;

	req = to_musb_request(pRequest);

	list_del(&pRequest->list);
	if (req->request.status == -EINPROGRESS)
		req->request.status = status;
	musb = req->musb;

	spin_unlock(&musb->Lock);
	if (is_dma_capable() && req->mapped) {
		dma_unmap_single(musb->controller,
				 req->request.dma,
				 req->request.length,
				 req->bTx ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
		req->request.dma = DMA_ADDR_INVALID;
		req->mapped = 0;
	}
	if (pRequest->status == 0)
		DBG(5, "%s done request %p,  %d/%d\n",
		    ep->end_point.name, pRequest,
		    req->request.actual, req->request.length);
	else
		DBG(2, "%s request %p, %d/%d fault %d\n",
		    ep->end_point.name, pRequest,
		    req->request.actual, req->request.length, pRequest->status);
	ep->hw_ep->fifo_flush_check = 0;
	req->request.complete(&req->ep->end_point, &req->request);
	ep->busy = 0;
	spin_lock(&musb->Lock);
}

/* ----------------------------------------------------------------------- */

/*
 * Abort requests queued to an endpoint using the status. Synchronous.
 * caller locked controller and blocked irqs, and selected this ep.
 */
static void nuke(struct musb_ep *ep, const int status)
{
	struct musb_request *req = NULL;

	ep->busy = 1;

	if (is_dma_capable() && ep->dma) {
		struct dma_controller *c = ep->pThis->pDmaController;
		int value;

		value = c->pfDmaAbortChannel(ep->dma);
		DBG(value ? 1 : 6, "%s: abort DMA --> %d\n", ep->name, value);
		c->pfDmaReleaseChannel(ep->dma);
		ep->dma = NULL;
	}

	while (!list_empty(&(ep->req_list))) {
		req = container_of(ep->req_list.next, struct musb_request,
				   request.list);
		musb_g_giveback(ep, &req->request, status);
		ep->busy = 1;
	}
	ep->busy = 0;
}

/**************************************************************************
 * TX/IN and RX/OUT Data transfers
 **************************************************************************/

/*
 * This assumes the separate CPPI engine is responding to DMA requests
 * from the usb core ... sequenced a bit differently from mentor dma.
 */

static inline int max_ep_writesize(struct musb *pThis, struct musb_ep *ep)
{
#ifdef  C_MP_TX
	if ((USB_ENDPOINT_XFER_BULK == ep->type) && pThis->bBulkSplit)
		return pThis->aLocalEnd[bEnd].wMaxPacketSizeTx;
	else
#endif
		return ep->wPacketSize;
}

/*
 * An endpoint is transmitting data. This can be called either from
 * the IRQ routine or from GadgetQueue to kickstart a request on an
 * endpoint.
 *
 * Context: controller locked, IRQs blocked, endpoint selected
 */
static void txstate(struct musb *pThis, struct musb_request *req)
{
	u8 bEnd;
	struct musb_ep *pEnd;
	struct usb_request *pRequest;
	void __iomem *pBase = pThis->pRegs;
	u16 wFifoCount = 0, wCsrVal;
	int use_dma = 0;

	bEnd = req->bEnd;
	pEnd = req->ep;

	/* we shouldn't get here while DMA is active ... but we do ... */
	if (dma_channel_status(pEnd->dma) == MGC_DMA_STATUS_BUSY) {
		DBG(4, "dma pending...\n");
		return;
	}

	/* read TXCSR before */
	wCsrVal = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd);

	pRequest = &req->request;
	wFifoCount = min(max_ep_writesize(pThis, pEnd),
			 (int)(pRequest->length - pRequest->actual));

	if (wCsrVal & MGC_M_TXCSR_TXPKTRDY) {
		DBG(5, "%s old packet still ready , txcsr %03x\n",
		    pEnd->end_point.name, wCsrVal);
		return;
	}

	DBG(4, "hw_ep%d, maxpacket %d, fifo count %d, txcsr %03x\n",
	    bEnd, pEnd->wPacketSize, wFifoCount, wCsrVal);

	/* stalled?? */
	if (wCsrVal & MGC_M_TXCSR_P_SENTSTALL) {
		musb_g_giveback(pEnd, pRequest, -EPIPE);
		return;
	}

	/* update the endpoint busy/free status */
	pEnd->busy = 1;

	if (req->dma) {
		struct dma_controller *c;

		c = pThis->pDmaController;
		use_dma = (pRequest->dma != DMA_ADDR_INVALID);

		/* MGC_M_TXCSR_ISO is still set correctly */

#ifdef CONFIG_USB_INVENTRA_DMA
		/* setup DMA, then program endpoint CSR */
		dma->dwActualLength = 0L;
		pEnd->dwRequestSize = min(pRequest->length, dma->dwMaxLength);
		use_dma = use_dma
		    && c->pfDmaProgramChannel(pEnd->dma, pEnd->wPacketSize,
					      pDmaChannel->bDesiredMode,
					      pRequest->dma,
					      pEnd->dwRequestSize);
		if (use_dma) {
			wCsrVal |= MGC_M_TXCSR_AUTOSET | MGC_M_TXCSR_DMAENAB
			    | (pDmaChannel->bDesiredMode
			       ? MGC_M_TXCSR_DMAMODE : 0);
			MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, wCsrVal);
		}
#elif defined(CONFIG_USB_TI_CPPI_DMA)

// REVISIT host side sets DMAENAB later than this ...

		/* prefer 'rndis' mode whenever the last
		 * tx packet will be short
		 */
		use_dma = use_dma
		    && c->pfDmaProgramChannel(pEnd->dma, pEnd->wPacketSize,
					      pRequest->zero, pRequest->dma,
					      pRequest->length);
		if (!use_dma) {
			c->pfDmaReleaseChannel(pEnd->dma);
			pEnd->dma = NULL;
			wCsrVal &= ~(MGC_M_TXCSR_DMAMODE | MGC_M_TXCSR_MODE);
			/* invariant: prequest->buf is non-null */
		} else {
			wCsrVal &= ~(MGC_M_TXCSR_AUTOSET
				     | MGC_M_TXCSR_DMAMODE
				     | MGC_M_TXCSR_TXPKTRDY);
			wCsrVal |= MGC_M_TXCSR_MODE | MGC_M_TXCSR_DMAENAB |
			    MGC_M_TXCSR_DMAMODE;
			MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd,
				       MGC_M_TXCSR_P_WZC_BITS | wCsrVal);
		}
#endif
	}

	if (!use_dma) {
		if ((pRequest->length != 0) && wFifoCount) {
			musb_write_fifo(pEnd->hw_ep, wFifoCount,
					(u8 *) (pRequest->buf +
						pRequest->actual));
			pRequest->actual += wFifoCount;
		}
		wCsrVal |= (MGC_M_TXCSR_TXPKTRDY | MGC_M_TXCSR_MODE);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, wCsrVal);
	}

	/* host may already have the data when this message shows... */
	DBG(3, "%s TX/IN %s len %d/%d, txcsr %04x, fifo %d/%d\n",
	    pEnd->end_point.name, use_dma ? "dma" : "pio",
	    pRequest->actual, pRequest->length,
	    MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd),
	    wFifoCount, MGC_ReadCsr16(pBase, MGC_O_HDRC_TXMAXP, bEnd));
}

/*
 * FIFO state update (e.g. data ready).
 * Called from IRQ,  with controller locked.
 */
void musb_g_tx(struct musb *pThis, u8 bEnd, u8 is_ep_intr)
{
	u16 wCsrVal;
	struct usb_request *pRequest;
	u8 __iomem *pBase = pThis->pRegs;
	struct musb_ep *pEnd;
	struct dma_channel *dma;

	MGC_SelectEnd(pBase, bEnd);
	pEnd = &pThis->aLocalEnd[bEnd * 2 - 1].ep_in;
	pRequest = next_request(pEnd);

	wCsrVal = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd);
	DBG(4, "<== %s, txcsr %04x\n", pEnd->end_point.name, wCsrVal);

	dma = is_dma_capable()? pEnd->dma : NULL;
	do {
		/* REVISIT for high bandwidth, MGC_M_TXCSR_P_INCOMPTX
		 * probably rates reporting as a host error
		 */
		if (wCsrVal & MGC_M_TXCSR_P_SENTSTALL) {
			wCsrVal &= ~MGC_M_TXCSR_P_SENTSTALL;
			MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, wCsrVal);
			if (dma_channel_status(dma) == MGC_DMA_STATUS_BUSY) {
				dma->bStatus = MGC_DMA_STATUS_CORE_ABORT;
				pThis->pDmaController->pfDmaAbortChannel(dma);
			}

			if ((pRequest) && (pEnd->busy)) {
				musb_g_giveback(pEnd, pRequest, -EPIPE);

			}
			break;
		}

		if (wCsrVal & MGC_M_TXCSR_P_UNDERRUN) {
			/* we NAKed, no big deal ... little reason to care */
			wCsrVal &= ~(MGC_M_TXCSR_P_UNDERRUN
				     | MGC_M_TXCSR_TXPKTRDY);
			MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, wCsrVal);
			DBG(7, "underrun on ep%d, req %p\n", bEnd, pRequest);
		}

		if (dma_channel_status(dma) == MGC_DMA_STATUS_BUSY) {
			/* SHOULD NOT HAPPEN (but does) */
			DBG(3, "%s busy\n", pEnd->end_point.name);
			break;
		}

		if (pRequest) {
			u8 is_dma = 0;

			if (dma && to_musb_request(pRequest)->dma &&
			    !is_ep_intr) {
				is_dma = 1;
				if (wCsrVal & MGC_M_TXCSR_DMAENAB) {
					wCsrVal &= ~(MGC_M_TXCSR_DMAENAB
						     | MGC_M_TXCSR_TXPKTRDY);
					MGC_WriteCsr16(pBase,
						       MGC_O_HDRC_TXCSR, bEnd,
						       MGC_M_TXCSR_P_WZC_BITS |
						       wCsrVal);
				}

				DBG(4,
				    "TXCSR%d %04x, dma off, %04x, "
				    "len %Zd, req %p\n", bEnd, wCsrVal,
				    MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR,
						  bEnd),
				    pEnd->dma->dwActualLength, pRequest);
				pRequest->actual += pEnd->dma->dwActualLength;
				wCsrVal = MGC_ReadCsr16(pBase,
							MGC_O_HDRC_TXCSR, bEnd);
				/*if (wCsrVal & MGC_M_TXCSR_FIFONOTEMPTY)
				   break; */
			}

			if (pRequest->actual == pRequest->length) {
				if (pRequest->zero && (!is_dma)
				    && pRequest->length
				    && (pRequest->length % pEnd->wPacketSize)
				    == 0) {
					wCsrVal |= (MGC_M_TXCSR_MODE |
					     MGC_M_TXCSR_TXPKTRDY);
						DBG(3, "sending zero pkt\n");
					MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR,
							       bEnd, wCsrVal);
					}

				MGC_SelectEnd(pBase, bEnd);
				wCsrVal = MGC_ReadCsr16(pBase,
							MGC_O_HDRC_TXCSR, bEnd);
				if ((wCsrVal & MGC_M_TXCSR_FIFONOTEMPTY) ||
					(wCsrVal & MGC_M_TXCSR_TXPKTRDY)) {
					pEnd->hw_ep->fifo_flush_check = 1;
					tasklet_schedule(&pThis->fifo_check);
					break;
				} else
					musb_g_giveback(pEnd, pRequest, 0);

				pRequest = pEnd->desc ? next_request(pEnd)
				    : NULL;
				if (!pRequest) {
					DBG(4, "bEnd=0x%x idle now\n", bEnd);
					break;
				}
			}

			txstate(pThis, to_musb_request(pRequest));
		}

	} while (0);
}

/* check for FIFO empty before scheduling the next request */
void musb_g_fifo_check_complete (struct musb_hw_ep *hw_ep)
{
	struct musb_ep *ep = &hw_ep->ep_in;
	struct usb_request *req = next_request (ep);

	DBG (5, "TX Fifo flush complete on %d\n", hw_ep->bLocalEnd);
	musb_g_giveback(ep, req, 0);
	req = ep->desc ? next_request(ep) : NULL;
	if (req)
		txstate(hw_ep->musb, to_musb_request(req));
}
/* ------------------------------------------------------------ */

/*
 * Context: controller locked, IRQs blocked, endpoint selected
 */
static void rxstate(struct musb *pThis, struct musb_request *req)
{
	u16 wCsrVal = 0;
	const u8 bEnd = req->bEnd;
	struct usb_request *pRequest = &req->request;
	void __iomem *pBase = pThis->pRegs;
	struct musb_ep *pEnd = req->ep;
	u16 wFifoCount = 0;
	u16 wCount = pEnd->wPacketSize;

	wCsrVal = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd);

	/* update the endpoint busy/free status */
	pEnd->busy = 1;
	if (wCsrVal & MGC_M_RXCSR_RXPKTRDY)
		wCount = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCOUNT, bEnd);

#ifdef  CONFIG_USB_TI_CPPI_DMA
	if (req->dma) {
		struct dma_controller *c = pThis->pDmaController;
		struct dma_channel *channel = pEnd->dma;

#ifdef CONFIG_ARCH_DAVINCI
		if ((wCsrVal & MGC_M_RXCSR_RXPKTRDY) && (wCount == 0)) {
			wCsrVal |= MGC_M_RXCSR_P_WZC_BITS;
			wCsrVal &= ~(MGC_M_RXCSR_RXPKTRDY |
					MGC_M_RXCSR_P_OVERRUN);
			MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, wCsrVal);
			pRequest->actual = 0;
			goto done;
		}
#endif

		if (c->pfDmaProgramChannel(channel,
					   pEnd->wPacketSize,
					   !pRequest->short_not_ok,
					   pRequest->dma + pRequest->actual,
					   pRequest->length -
					   pRequest->actual)) {

			/* make sure that if an rxpkt arrived after the irq,
			 * the cppi engine will be ready to take it as soon
			 * as DMA is enabled
			 */
#ifdef CONFIG_ARCH_DAVINCI
			wCsrVal &=
			    ~(MGC_M_RXCSR_AUTOCLEAR | MGC_M_RXCSR_DMAMODE
				| MGC_M_RXCSR_P_OVERRUN);
#else
			wCsrVal &=
			    ~(MGC_M_RXCSR_AUTOCLEAR | MGC_M_RXCSR_DMAMODE);
#endif
			wCsrVal |= MGC_M_RXCSR_DMAENAB | MGC_M_RXCSR_P_WZC_BITS;
			MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, wCsrVal);
			return;
		}
	}
#endif

	if (wCsrVal & MGC_M_RXCSR_RXPKTRDY) {
		if (pRequest->actual < pRequest->length) {
#ifdef CONFIG_USB_INVENTRA_DMA
			if (is_dma_capable() && pEnd->dma) {
				struct dma_controller *c;
				struct dma_channel *channel;
				int use_dma = 0;

				c = pThis->pDmaController;
				channel = pEnd->dma;
				/* see if we need to continue transfer */
				switch (c->pfDmaGetChannelStatus(channel)) {
				case MGC_DMA_STATUS_FREE:
					pRequest->actual =
					    channel->dwActualLength;
					if (pRequest->actual < pRequest->length) {
						channel->dwActualLength = 0L;
						pEnd->dwRequestSize =
						    min(pRequest->length,
							channel->dwMaxLength);
						use_dma = c->pfDmaProgramChannel
						    (channel,
						     pEnd->wPacketSize,
						     channel->bDesiredMode,
						     channel->dma +
						     channel->actual,
						     pEnd->dwRequestSize);
					}
					break;
				case MGC_DMA_STATUS_BUSY:
					return;
				default:
					/* TODO: say what? */
					pRequest->status = -ECONNRESET;
				}
				if (use_dma && !pRequest->actual) {
					wCsrVal |= MGC_M_RXCSR_AUTOCLEAR
					    | MGC_M_RXCSR_DMAENAB
					    | MGC_M_RXCSR_P_WZC_BITS;

					/* this special sequence is required to
					 * get DMAReq to activate
					 * */
					MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR,
						       bEnd, wCsrVal |
						       MGC_M_RXCSR_DMAMODE);
					MGC_WriteCsr16(pBase,
						       MGC_O_HDRC_RXCSR,
						       bEnd, wCsrVal);
				}
				if (use_dma)
					return;
			}
#endif

			wFifoCount = pRequest->length - pRequest->actual;
			DBG(3, "%s OUT/RX pio fifo %d/%d, maxpacket %d\n",
			    pEnd->end_point.name,
			    wCount, wFifoCount, pEnd->wPacketSize);

			wFifoCount = min(wCount, wFifoCount);
			if (wFifoCount)
				musb_read_fifo(pEnd->hw_ep, wFifoCount,
				       (u8 *) (pRequest->buf +
					       pRequest->actual));
			pRequest->actual += wFifoCount;

			/* REVISIT if we left anything in the fifo, flush
			 * it and report -EOVERFLOW
			 */

			/* ack the read! */
			wCsrVal |= MGC_M_RXCSR_P_WZC_BITS;
			wCsrVal &= ~MGC_M_RXCSR_RXPKTRDY;
			MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, wCsrVal);
		}
	}
done:
	/* reach the end or short packet detected */
	if (pRequest->actual == pRequest->length || wCount < pEnd->wPacketSize)
		musb_g_giveback(pEnd, pRequest, 0);
}

/*
 * Data ready for a request; called from IRQ
 * @param pThis the controller
 * @param req the request
 */
void musb_g_rx(struct musb *pThis, u8 bEnd)
{
	u16 wCsrVal;
	struct usb_request *pRequest;
	void __iomem *pBase = pThis->pRegs;
	struct musb_ep *pEnd;
	struct dma_channel *dma;

	MGC_SelectEnd(pBase, bEnd);

	pEnd = &pThis->aLocalEnd[bEnd * 2].ep_out;
	pRequest = next_request(pEnd);
	if (pRequest == NULL) {
		return;
	}

	wCsrVal = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd);
	DBG(4, "<== %s, rxcsr %04x\n", pEnd->end_point.name, wCsrVal);

	dma = is_dma_capable()? pEnd->dma : NULL;

	if (wCsrVal & MGC_M_RXCSR_P_SENTSTALL) {
		if (dma_channel_status(dma) == MGC_DMA_STATUS_BUSY) {
			dma->bStatus = MGC_DMA_STATUS_CORE_ABORT;
			(void)pThis->pDmaController->pfDmaAbortChannel(dma);
			pRequest->actual += pEnd->dma->dwActualLength;
		}

		wCsrVal |= MGC_M_RXCSR_P_WZC_BITS;
		wCsrVal &= ~MGC_M_RXCSR_P_SENTSTALL;
		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, wCsrVal);

		if ((pRequest) && (pEnd->busy))
			musb_g_giveback(pEnd, pRequest, -EPIPE);
		goto done;
	}

	if (wCsrVal & MGC_M_RXCSR_P_OVERRUN) {
		// wCsrVal |= MGC_M_RXCSR_P_WZC_BITS;
		wCsrVal &= ~MGC_M_RXCSR_P_OVERRUN;
		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, wCsrVal);

		DBG(3, "%s iso overrun on %p\n", pEnd->name, pRequest);
		if (pRequest && pRequest->status == -EINPROGRESS)
			pRequest->status = -EOVERFLOW;
	}
	if (wCsrVal & MGC_M_RXCSR_INCOMPRX) {
		/* REVISIT not necessarily an error */
		DBG(4, "%s, incomprx\n", pEnd->end_point.name);
	}

	if (dma_channel_status(dma) == MGC_DMA_STATUS_BUSY) {
#ifdef CONFIG_ARCH_DAVINCI
		u32 count;

		/* Resolve DMA race in handling zero byte packet */
		count = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCOUNT, bEnd);
		if ((wCsrVal & MGC_M_RXCSR_RXPKTRDY) && (count == 0)) {
			dma->bStatus = MGC_DMA_STATUS_CORE_ABORT;
			pThis->pDmaController->pfDmaAbortChannel(dma);
			pRequest->actual = 0;
			musb_g_giveback(pEnd, pRequest, 0);
			pRequest = next_request(pEnd);
			if (!pRequest)
				goto done;

			/* don't start more i/o till the stall clears */
			MGC_SelectEnd(pBase, bEnd);
			wCsrVal = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd);
			if (!(wCsrVal & MGC_M_RXCSR_P_SENDSTALL))
				rxstate(pThis, to_musb_request(pRequest));
		}
#endif
		/* "should not happen"; likely RXPKTRDY pending for DMA */
		DBG(4, "%s busy, csr %04x\n", pEnd->end_point.name, wCsrVal);
		goto done;
	}

	if (dma && to_musb_request(pRequest)->dma) {

		DBG(4, "RXCSR%d %04x, dma off, %04x, len %Zd, req %p\n",
		    bEnd, wCsrVal,
		    MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd),
		    pEnd->dma->dwActualLength, pRequest);
		pRequest->actual += pEnd->dma->dwActualLength;
		musb_g_giveback(pEnd, pRequest, 0);

		pRequest = next_request(pEnd);
		if (!pRequest)
			goto done;

		/* don't start more i/o till the stall clears */
		MGC_SelectEnd(pBase, bEnd);
		wCsrVal = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd);
		if (wCsrVal & MGC_M_RXCSR_P_SENDSTALL)
			goto done;
	}

	/* analyze request if the ep is hot */
	if (pRequest)
		rxstate(pThis, to_musb_request(pRequest));
	else
		DBG(3, "Rx: bytes waiting on %sep=0x%x\n",
		    pEnd->desc ? "" : "inactive ", bEnd);

      done:
	return;
}

/* ------------------------------------------------------------ */

static int musb_gadget_enable(struct usb_ep *ep,
			      const struct usb_endpoint_descriptor *desc)
{
	unsigned long flags;
	struct musb_ep *pEnd;
	struct musb *pThis;
	void __iomem *pBase;
	u8 bEnd;
	u16 csr;
	unsigned tmp;
	int status = -EINVAL;

	if (!ep || !desc)
		return -EINVAL;

	pEnd = to_musb_ep(ep);
	pThis = pEnd->pThis;
	pBase = pThis->pRegs;
	bEnd = pEnd->bEndNumber;

	spin_lock_irqsave(&pThis->Lock, flags);

	if (pEnd->desc) {
		status = -EBUSY;
		goto fail;
	}
	pEnd->type = desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;

	/* check direction and (later) maxpacket size against endpoint */
	if ((desc->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK) != bEnd)
		goto fail;

	/* REVISIT this rules out high bandwidth periodic transfers */
	tmp = le16_to_cpu(desc->wMaxPacketSize);
	if (tmp & ~0x07ff)
		goto fail;
	pEnd->wPacketSize = tmp;

	/* enable the interrupts for the endpoint, set the endpoint
	 * packet size (or fail), set the mode, clear the fifo
	 */
	MGC_SelectEnd(pBase, bEnd);
	if (desc->bEndpointAddress & USB_DIR_IN) {
		u16 wIntrTxE = musb_readw(pBase, MGC_O_HDRC_INTRTXE);

		if (pEnd->hw_ep->bIsSharedFifo)
			pEnd->is_in = 1;
		if (!pEnd->is_in)
			goto fail;
		if (tmp > pEnd->hw_ep->wMaxPacketSizeTx)
			goto fail;

		wIntrTxE |= (1 << bEnd);
		musb_writew(pBase, MGC_O_HDRC_INTRTXE, wIntrTxE);
		csr = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd);
		/* REVISIT if pThis->bBulkSplit, use by updating "tmp";
		 * likewise high bandwidth periodic tx
		 */
		MGC_WriteCsr16(pBase, MGC_O_HDRC_TXMAXP, bEnd, tmp);
		if (pEnd->type == USB_ENDPOINT_XFER_ISOC)
			csr |= MGC_M_TXCSR_ISO;

		/* set twice in case of double buffering */
		MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, csr);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, csr);
	} else {
		u16 wIntrRxE = musb_readw(pBase, MGC_O_HDRC_INTRRXE);

		if (pEnd->hw_ep->bIsSharedFifo)
			pEnd->is_in = 0;
		if (pEnd->is_in)
			goto fail;
		if (tmp > pEnd->hw_ep->wMaxPacketSizeRx)
			goto fail;

		wIntrRxE |= (1 << bEnd);
		musb_writew(pBase, MGC_O_HDRC_INTRRXE, wIntrRxE);

		/* REVISIT if pThis->bBulkCombine, use by updating "tmp"
		 * likewise high bandwidth periodic rx
		 */
		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXMAXP, bEnd, tmp);

		/* force shared fifo to OUT-only mode */
		if (pEnd->hw_ep->bIsSharedFifo) {
			csr = musb_readw(pBase, MGC_O_HDRC_TXCSR);
			csr &= ~(MGC_M_TXCSR_MODE | MGC_M_TXCSR_TXPKTRDY);
			MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, csr);
		}

		csr = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd);
		if (pEnd->type == USB_ENDPOINT_XFER_ISOC)
			csr |= MGC_M_RXCSR_P_ISO;
		else if (pEnd->type == USB_ENDPOINT_XFER_INT)
			csr |= MGC_M_RXCSR_DISNYET;

		/* set twice in case of double buffering */
		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, csr);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, csr);
	}

	/* NOTE:  all the I/O code _should_ work fine without DMA, in case
	 * for some reason you run out of channels here.
	 */
	if (is_dma_capable() && pThis->pDmaController) {
		struct dma_controller *c = pThis->pDmaController;

#ifdef  DISABLE_RXDMA
		if (desc->bEndpointAddress & USB_DIR_IN)
			pEnd->dma = NULL;
		else
#endif
			pEnd->dma = c->pfDmaAllocateChannel(c->pPrivateData,
							    bEnd,
							    (desc->
							     bEndpointAddress &
							     USB_DIR_IN),
							    pEnd->type,
							    pEnd->wPacketSize);
	} else
		pEnd->dma = NULL;

	pEnd->desc = desc;
	pEnd->busy = 0;
	status = 0;

	pr_debug("%s periph: enabled %s for %s %s, %smaxpacket %d\n",
		 musb_driver_name, pEnd->end_point.name, ( {
							  char *s;
							  switch (pEnd->type) {
case USB_ENDPOINT_XFER_BULK:
s = "bulk"; break; case USB_ENDPOINT_XFER_INT:
s = "int"; break; default:
							  s = "iso"; break;};
							  s;}
		 ),
		 pEnd->is_in ? "IN" : "OUT",
		 pEnd->dma ? "dma, " : "", pEnd->wPacketSize) ;

      fail:
	spin_unlock_irqrestore(&pThis->Lock, flags);
	return status;
}

/*
 * Disable an endpoint flushing all requests queued.
 */
static int musb_gadget_disable(struct usb_ep *ep)
{
	unsigned long flags;
	struct musb *pThis;
	u8 bEnd;
	struct musb_ep *pEnd;
	int status = 0;
	u16 wCsr;

	pEnd = to_musb_ep(ep);
	pThis = pEnd->pThis;
	bEnd = pEnd->bEndNumber;

	spin_lock_irqsave(&pThis->Lock, flags);
	MGC_SelectEnd(pThis->pRegs, bEnd);

	/* zero the endpoint sizes */
	if (pEnd->is_in) {
		u16 wIntrTxE = musb_readw(pThis->pRegs, MGC_O_HDRC_INTRTXE);
		wIntrTxE &= ~(1 << bEnd);
		musb_writew(pThis->pRegs, MGC_O_HDRC_INTRTXE, wIntrTxE);
		MGC_WriteCsr16(pThis->pRegs, MGC_O_HDRC_TXMAXP, bEnd, 0);
		wCsr = MGC_ReadCsr16(pThis->pRegs, MGC_O_HDRC_TXCSR, bEnd);
		wCsr |= MGC_M_TXCSR_CLRDATATOG;
		MGC_WriteCsr16(pThis->pRegs, MGC_O_HDRC_TXCSR, bEnd, wCsr);
		MGC_WriteCsr16(pThis->pRegs, MGC_O_HDRC_TXCSR, bEnd, wCsr);
	} else {
		u16 wIntrRxE = musb_readw(pThis->pRegs, MGC_O_HDRC_INTRRXE);
		wIntrRxE &= ~(1 << bEnd);
		musb_writew(pThis->pRegs, MGC_O_HDRC_INTRRXE, wIntrRxE);
		MGC_WriteCsr16(pThis->pRegs, MGC_O_HDRC_RXMAXP, bEnd, 0);
		wCsr = MGC_ReadCsr16(pThis->pRegs, MGC_O_HDRC_RXCSR, bEnd);
		wCsr |= MGC_M_RXCSR_CLRDATATOG;
		MGC_WriteCsr16(pThis->pRegs, MGC_O_HDRC_RXCSR, bEnd, wCsr);
		MGC_WriteCsr16(pThis->pRegs, MGC_O_HDRC_RXCSR, bEnd, wCsr);
	}

	pEnd->desc = NULL;
	pEnd->hw_ep->fifo_flush_check = 0;

	/* abort all pending DMA and requests */
	nuke(pEnd, -ESHUTDOWN);
	spin_unlock (&(pThis->Lock));
	musb_gadget_fifo_flush(ep);
	spin_lock (&(pThis->Lock));

	spin_unlock_irqrestore(&(pThis->Lock), flags);

	DBG(2, "%s\n", pEnd->end_point.name);

	return status;
}

/*
 * Allocate a request for an endpoint.
 * Reused by ep0 code.
 */
struct usb_request *musb_alloc_request(struct usb_ep *ep, gfp_t gfp_flags)
{
	struct musb_ep *musb_ep = to_musb_ep(ep);
	struct musb_request *pRequest = NULL;

	pRequest = kzalloc(sizeof *pRequest, gfp_flags);
	if (pRequest) {
		INIT_LIST_HEAD(&pRequest->request.list);
		pRequest->request.dma = DMA_ADDR_INVALID;
		pRequest->bEnd = musb_ep->bEndNumber;
		pRequest->ep = musb_ep;
	}

	return &pRequest->request;
}

/*
 * Free a request
 * Reused by ep0 code.
 */
void musb_free_request(struct usb_ep *ep, struct usb_request *req)
{
	kfree(to_musb_request(req));
}

/*
 * dma-coherent memory allocation (for dma-capable endpoints)
 */
static void *musb_gadget_alloc_buffer(struct usb_ep *ep, unsigned bytes,
				      dma_addr_t * dma, gfp_t gfp_flags)
{
	struct musb_ep *musb_ep = to_musb_ep(ep);

	return musb_alloc_buffer(musb_ep->pThis, bytes, gfp_flags, dma);
}

static void musb_gadget_free_buffer(struct usb_ep *ep,
				    void *address, dma_addr_t dma,
				    unsigned bytes)
{
	struct musb_ep *musb_ep = to_musb_ep(ep);

	musb_free_buffer(musb_ep->pThis, bytes, address, dma);
}

/*
 * Context: controller locked, IRQs blocked.
 */
static void musb_ep_restart(struct musb *pThis, struct musb_request *req)
{
	DBG(3, "<== %s request %p on hw_ep%d\n",
	    req->bTx ? "TX/IN" : "RX/OUT", &req->request, req->bEnd);

	MGC_SelectEnd(pThis->pRegs, req->bEnd);
	if (req->bTx) {
		txstate(pThis, req);
	} else {
		rxstate(pThis, req);
	}
}

static int musb_gadget_queue(struct usb_ep *ep, struct usb_request *req,
			     gfp_t gfp_flags)
{
	struct musb_ep *pEnd;
	struct musb_request *pRequest;
	struct musb *pThis;
	int status = 0;
	unsigned long lockflags;
	u16 csr;
	u8 stall = 0;

	if (!ep || !req)
		return -EINVAL;

	pEnd = to_musb_ep(ep);
	pThis = pEnd->pThis;

	pRequest = to_musb_request(req);
	pRequest->musb = pThis;

	if (pRequest->ep != pEnd)
		return -EINVAL;

	DBG(4, "<== to %s request=%p\n", ep->name, req);

	/* request is mine now... */
	pRequest->request.actual = 0;
	pRequest->request.status = -EINPROGRESS;
	pRequest->bEnd = pEnd->bEndNumber;
	pRequest->bTx = pEnd->is_in;
	pRequest->dma = 0;

	if (is_dma_capable() && pEnd->dma
	    && pRequest->request.length >= 32) {
		if (pRequest->request.dma == DMA_ADDR_INVALID) {
			pRequest->request.dma =
			    dma_map_single(pThis->controller,
					   pRequest->request.buf,
					   pRequest->request.length,
					   pRequest->bTx
					   ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
			pRequest->mapped = 1;
		} else
			pRequest->mapped = 0;

		pRequest->dma = 1;
	} else if (!req->buf) {
		return -ENODATA;
	} else
		pRequest->mapped = 0;

	spin_lock_irqsave(&pThis->Lock, lockflags);

	/* don't queue if the ep is down */
	if (!pEnd->desc) {
		DBG(4, "req %p queued to %s while ep %s\n",
		    req, ep->name, "disabled");
		status = -ESHUTDOWN;
		goto cleanup;
	}

	/* add pRequest to the list */
	list_add_tail(&(pRequest->request.list), &(pEnd->req_list));

	/* it this is the head of the queue, start i/o ... */
	if (!pEnd->busy && &pRequest->request.list == pEnd->req_list.next) {
		if (pEnd->is_in) {
			csr = MGC_ReadCsr16(pThis->pRegs,
					    MGC_O_HDRC_TXCSR, pEnd->bEndNumber);
			stall = csr & MGC_M_TXCSR_P_SENDSTALL ? 1 : 0;
		} else {
			csr = MGC_ReadCsr16(pThis->pRegs,
					    MGC_O_HDRC_RXCSR, pEnd->bEndNumber);
			stall = csr & MGC_M_RXCSR_P_SENDSTALL ? 1 : 0;
		}

		/* Endpoint Stalled do not start a new request until the STALL
		 * is cleared
		 */
		if (!stall)
			musb_ep_restart(pThis, pRequest);
	}

      cleanup:
	spin_unlock_irqrestore(&pThis->Lock, lockflags);
	return status;
}

static int musb_gadget_dequeue(struct usb_ep *ep, struct usb_request *pRequest)
{
	struct musb_ep *pEnd = to_musb_ep(ep);
	struct usb_request *r;
	unsigned long flags;
	int status = 0;

	if (!ep || !pRequest || to_musb_request(pRequest)->ep != pEnd)
		return -EINVAL;

	spin_lock_irqsave(&pEnd->pThis->Lock, flags);

	list_for_each_entry(r, &pEnd->req_list, list) {
		if (r == pRequest)
			break;
	}
	if (r != pRequest) {
		DBG(3, "request %p not queued to %s\n", pRequest, ep->name);
		status = -EINVAL;
		goto done;
	}

	/* if the hardware doesn't have the request, easy ... */
	if (pEnd->req_list.next != &pRequest->list)
		musb_g_giveback(pEnd, pRequest, -ECONNRESET);

	/* ... else abort the dma transfer ... */
	else if (is_dma_capable() && pEnd->dma
		 && to_musb_request(pRequest)->dma) {
		struct dma_controller *c = pEnd->pThis->pDmaController;

		MGC_SelectEnd(pEnd->pThis->pRegs, pEnd->bEndNumber);
		if (c->pfDmaAbortChannel)
			status = c->pfDmaAbortChannel(pEnd->dma);
		else
			status = -EBUSY;
		pEnd->hw_ep->fifo_flush_check = 0;
		if (status == 0)
			musb_g_giveback(pEnd, pRequest, -ECONNRESET);
	} else {
		/* NOTE: by sticking to easily tested hardware/driver states,
		 * we leave counting of in-flight packets imprecise.
		 */
		pEnd->hw_ep->fifo_flush_check = 0;
		musb_g_giveback(pEnd, pRequest, -ECONNRESET);
	}

      done:
	spin_unlock_irqrestore(&pEnd->pThis->Lock, flags);
	return status;
}

/*
 * Set or clear the halt bit of an endpoint. A halted enpoint won't tx/rx any
 * data but will queue requests.
 *
 * exported to ep0 code
 */
int musb_gadget_set_halt(struct usb_ep *ep, int value)
{
	struct musb_ep *pEnd;
	u8 bEnd;
	struct musb *pThis;
	void __iomem *pBase;
	unsigned long flags;
	u16 wCsr;
	struct musb_request *pRequest = NULL;
	int status = 0;

	if (!ep)
		return -EINVAL;

	pEnd = to_musb_ep(ep);
	bEnd = pEnd->bEndNumber;
	pThis = pEnd->pThis;
	pBase = pThis->pRegs;

	spin_lock_irqsave(&pThis->Lock, flags);

	if ((USB_ENDPOINT_XFER_ISOC == pEnd->type)) {
		status = -EINVAL;
		goto done;
	}

	MGC_SelectEnd(pBase, bEnd);

	/* cannot portably stall with non-empty FIFO */
	pRequest = to_musb_request(next_request(pEnd));
#if 0
	if (value && pEnd->is_in) {
		wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd);
		if (wCsr & MGC_M_TXCSR_FIFONOTEMPTY) {
			DBG(3, "%s fifo busy, cannot halt\n", ep->name);
			spin_unlock_irqrestore(&pThis->Lock, flags);
			return -EAGAIN;
		}
	}
#endif
	/* set/clear the stall bit */
	DBG(2, "%s: %s stall\n", ep->name, value ? "set" : "clear");
	if (pEnd->is_in) {
		wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd);
		if (value) {
			wCsr |= MGC_M_TXCSR_P_SENDSTALL;
		} else {
			wCsr &=
			    ~(MGC_M_TXCSR_P_SENDSTALL | MGC_M_TXCSR_P_UNDERRUN);
			wCsr |= MGC_M_TXCSR_CLRDATATOG;
		}
		wCsr &= ~MGC_M_TXCSR_TXPKTRDY;
		MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, wCsr);
	} else {
		wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd);
		if (value) {
			wCsr |= MGC_M_RXCSR_P_SENDSTALL;
		} else {
			wCsr &= ~MGC_M_RXCSR_P_SENDSTALL;
			wCsr |= MGC_M_RXCSR_CLRDATATOG;
		}

		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, wCsr);
	}

      done:

	/* maybe start the first request in the queue */
	if (!pEnd->busy && !value && pRequest) {
		DBG(3, "restarting the request\n");
		musb_ep_restart(pThis, pRequest);
	}

	spin_unlock_irqrestore(&pThis->Lock, flags);
	return status;
}

static int musb_gadget_fifo_status(struct usb_ep *ep)
{
	struct musb_ep *musb_ep = to_musb_ep(ep);
	int retval = -EINVAL;

	if (musb_ep->desc && !musb_ep->is_in) {
		struct musb *musb = musb_ep->pThis;
		int bEnd = musb_ep->bEndNumber;
		void __iomem *mbase = musb->pRegs;
		unsigned long flags;

		spin_lock_irqsave(&musb->Lock, flags);

		MGC_SelectEnd(mbase, bEnd);
		/* FIXME return zero unless RXPKTRDY is set */
		retval = MGC_ReadCsr16(mbase, MGC_O_HDRC_RXCOUNT, bEnd);

		spin_unlock_irqrestore(&musb->Lock, flags);
	}
	return retval;
}

static void musb_gadget_fifo_flush(struct usb_ep *ep)
{
	struct musb_ep *musb_ep = to_musb_ep(ep);
	struct musb *musb;
	void __iomem *mbase;
	u8 nEnd;
	unsigned long flags;
	u16 wCsr, wIntrTxE;

	musb = musb_ep->pThis;
	mbase = musb->pRegs;
	nEnd = musb_ep->bEndNumber;

	spin_lock_irqsave(&musb->Lock, flags);
	MGC_SelectEnd(mbase, (u8) nEnd);

	/* disable interrupts */
	wIntrTxE = musb_readw(mbase, MGC_O_HDRC_INTRTXE);
	musb_writew(mbase, MGC_O_HDRC_INTRTXE, wIntrTxE & ~(1 << nEnd));

	if (musb_ep->is_in) {
		wCsr = MGC_ReadCsr16(mbase, MGC_O_HDRC_TXCSR, nEnd);
		wCsr |= MGC_M_TXCSR_FLUSHFIFO | MGC_M_TXCSR_P_WZC_BITS;
		MGC_WriteCsr16(mbase, MGC_O_HDRC_TXCSR, nEnd, wCsr);
		MGC_WriteCsr16(mbase, MGC_O_HDRC_TXCSR, nEnd, wCsr);
	} else {
		wCsr = MGC_ReadCsr16(mbase, MGC_O_HDRC_RXCSR, nEnd);
		wCsr |= MGC_M_RXCSR_FLUSHFIFO | MGC_M_RXCSR_P_WZC_BITS;
		MGC_WriteCsr16(mbase, MGC_O_HDRC_RXCSR, nEnd, wCsr);
		MGC_WriteCsr16(mbase, MGC_O_HDRC_RXCSR, nEnd, wCsr);
	}

	/* re-enable interrupt */
	musb_writew(mbase, MGC_O_HDRC_INTRTXE, wIntrTxE);
	spin_unlock_irqrestore(&musb->Lock, flags);
}

static struct usb_ep_ops musb_ep_ops = {
	.enable = musb_gadget_enable,
	.disable = musb_gadget_disable,
	.alloc_request = musb_alloc_request,
	.free_request = musb_free_request,
	.alloc_buffer = musb_gadget_alloc_buffer,
	.free_buffer = musb_gadget_free_buffer,
	.queue = musb_gadget_queue,
	.dequeue = musb_gadget_dequeue,
	.set_halt = musb_gadget_set_halt,
	.fifo_status = musb_gadget_fifo_status,
	.fifo_flush = musb_gadget_fifo_flush
};

/***********************************************************************/

static int musb_gadget_get_frame(struct usb_gadget *gadget)
{
	struct musb *pThis = gadget_to_musb(gadget);

	return (int)musb_readw(pThis->pRegs, MGC_O_HDRC_FRAME);
}

static int musb_gadget_wakeup(struct usb_gadget *gadget)
{
	struct musb *musb = gadget_to_musb(gadget);
	unsigned long flags;
	int status = 0;
	u8 power;

	spin_lock_irqsave(&musb->Lock, flags);

#ifdef CONFIG_USB_MUSB_OTG
	switch (musb->OtgMachine.xceiv.state) {
	case OTG_STATE_B_PERIPHERAL:
		/* FIXME if not suspended, fail */
		if (musb->bMayWakeup) {
			power = musb_readb(musb->pRegs, MGC_O_HDRC_POWER);
			power |= MGC_M_POWER_RESUME;
			musb_writeb(musb->pRegs, MGC_O_HDRC_POWER, power);

			mod_timer(&musb->Timer, jiffies + msecs_to_jiffies(10));
			break;
		}
		goto fail;
	case OTG_STATE_B_IDLE:
		/* REVISIT we might be able to do SRP even without OTG,
		 * though Linux doesn't yet expose that capability
		 */
		if (is_otg_enabled(musb)) {
			musb->OtgMachine.bRequest = MGC_OTG_REQUEST_START_BUS;
			otg_input_changed_X(musb, FALSE, FALSE);
			break;
		}
		/* FALLTHROUGH */
	default:
	      fail:
		status = -EINVAL;
		goto done;
	}
#else
	power = musb_readb(musb->pRegs, MGC_O_HDRC_POWER);
	power |= MGC_M_POWER_RESUME;
	musb_writeb(musb->pRegs, MGC_O_HDRC_POWER, power);
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	mod_timer(&musb->Timer, jiffies + msecs_to_jiffies(10));
#else
	mdelay(10);

	power = musb_readb(musb->pRegs, MGC_O_HDRC_POWER);
	power &= ~MGC_M_POWER_RESUME;
	musb_writeb(musb->pRegs, MGC_O_HDRC_POWER, power);
#endif

#endif
#ifdef CONFIG_USB_MUSB_OTG
      done:
#endif
	spin_unlock_irqrestore(&musb->Lock, flags);
	return status;
}

static int
musb_gadget_set_self_powered(struct usb_gadget *gadget, int is_selfpowered)
{
	struct musb *pThis = gadget_to_musb(gadget);

	pThis->bIsSelfPowered = !!is_selfpowered;
	return 0;
}

void musb_pullup(struct musb *musb, int is_on)
{
	u8 power;

	power = musb_readb(musb->pRegs, MGC_O_HDRC_POWER);
	if (is_on)
		power |= MGC_M_POWER_SOFTCONN;
	else
		power &= ~MGC_M_POWER_SOFTCONN;

	/* FIXME if on, HdrcStart; if off, HdrcStop */

	DBG(3, "gadget %s D+ pullup %s\n",
	    musb->pGadgetDriver->function, is_on ? "on" : "off");
	musb_writeb(musb->pRegs, MGC_O_HDRC_POWER, power);
}

#if 0
static int musb_gadget_vbus_session(struct usb_gadget *gadget, int is_active)
{
	DBG(2, "<= %s =>\n", __FUNCTION__);

	// FIXME iff driver's softconnect flag is set (as it is during probe,
	// though that can clear it), just musb_pullup().

	return -EINVAL;
}

static int musb_gadget_vbus_draw(struct usb_gadget *gadget, unsigned mA)
{
	/* FIXME -- delegate to otg_transciever logic */

	DBG(2, "<= vbus_draw %u =>\n", mA);
	return 0;
}
#endif

static int musb_gadget_pullup(struct usb_gadget *gadget, int is_on)
{
	struct musb *musb = gadget_to_musb(gadget);
	unsigned long flags;

	is_on = !!is_on;

	/* NOTE: this assumes we are sensing vbus; we'd rather
	 * not pullup unless the B-session is active.
	 */
	spin_lock_irqsave(&musb->Lock, flags);
	if (is_on != musb->softconnect) {
		musb->softconnect = is_on;
		musb_pullup(musb, is_on);
	}
	spin_unlock_irqrestore(&musb->Lock, flags);
	return 0;
}

static struct usb_gadget_ops musb_gadget_operations = {
	.get_frame = musb_gadget_get_frame,
	.wakeup = musb_gadget_wakeup,
	.set_selfpowered = musb_gadget_set_self_powered,
	//.vbus_session     = musb_gadget_vbus_session,
	//.vbus_draw        = musb_gadget_vbus_draw,
	.pullup = musb_gadget_pullup,
};

/****************************************************************
 * Registration operations
 ****************************************************************/

/* Only this registration code "knows" the rule (from USB standards)
 * about there being only one external upstream port.  It assumes
 * all peripheral ports are external...
 */
static struct musb *the_gadget;

static void musb_gadget_release(struct device *dev)
{
	// kref_put(WHAT)
	dev_dbg(dev, "%s\n", __FUNCTION__);
}

static void __init
init_peripheral_ep(struct musb *musb, struct musb_ep *ep, u8 bEnd, int is_in)
{
	struct musb_hw_ep *hw_ep = musb->aLocalEnd + bEnd;

	memset(ep, 0, sizeof *ep);

	ep->bEndNumber = hw_ep->bLocalEnd;
	ep->pThis = musb;
	ep->hw_ep = hw_ep;
	ep->is_in = is_in;

	INIT_LIST_HEAD(&ep->req_list);

	sprintf(ep->name, "ep%d%s", hw_ep->bLocalEnd,
		(!bEnd || hw_ep->bIsSharedFifo) ? "" : (is_in ? "in" : "out"));
	ep->end_point.name = ep->name;
	INIT_LIST_HEAD(&ep->end_point.ep_list);
	if (!bEnd) {
		ep->end_point.maxpacket = 64;
		ep->end_point.ops = &musb_g_ep0_ops;
		musb->g.ep0 = &ep->end_point;
	} else {
		if (is_in)
			ep->end_point.maxpacket = hw_ep->wMaxPacketSizeTx;
		else
			ep->end_point.maxpacket = hw_ep->wMaxPacketSizeRx;
		ep->end_point.ops = &musb_ep_ops;
		list_add_tail(&ep->end_point.ep_list, &musb->g.ep_list);
	}
	DBG(4, "periph: %s, maxpacket %d\n", ep->end_point.name,
	    ep->end_point.maxpacket);
}

/*
 * Initialize the endpoints exposed to peripheral drivers, with backlinks
 * to the rest of the driver state.
 */
static inline void __init musb_g_init_endpoints(struct musb *pThis)
{
	u8 bEnd;
	struct musb_hw_ep *hw_ep;
	unsigned count = 0;

	/* intialize endpoint list just once */
	INIT_LIST_HEAD(&(pThis->g.ep_list));

	for (bEnd = 0, hw_ep = pThis->aLocalEnd; bEnd < pThis->bEndCount;
	     bEnd++, hw_ep++) {
		if (hw_ep->bIsSharedFifo /* || !bEnd */ ) {
			init_peripheral_ep(pThis, &hw_ep->ep_in, bEnd, 0);
			count++;
		} else {
			if (hw_ep->wMaxPacketSizeTx) {
				init_peripheral_ep(pThis, &hw_ep->ep_in, bEnd,
						   1);
				count++;
			}
			if (hw_ep->wMaxPacketSizeRx) {
				init_peripheral_ep(pThis, &hw_ep->ep_out, bEnd,
						   0);
				count++;
			}
		}
	}
	DBG(2, "initialized %d (max %d) endpoints\n", count,
	    pThis->bEndCount * 2 - 1);
}

/* called once during driver setup to initialize and link into
 * the driver model; memory is zeroed.
 */
int __init musb_gadget_setup(struct musb *pThis)
{
	int status;

	/* REVISIT minor race:  if (erroneously) setting up two
	 * musb peripherals at the same time, only the bus lock
	 * is probably held.
	 */
	if (the_gadget)
		return -EBUSY;
	the_gadget = pThis;

	pThis->g.ops = &musb_gadget_operations;
	pThis->g.is_dualspeed = 1;
	pThis->g.speed = USB_SPEED_UNKNOWN;
#ifdef CONFIG_USB_MUSB_OTG
	if (pThis->board_mode == MUSB_OTG)
		pThis->g.is_otg = 1;
#endif

	/* this "gadget" abstracts/virtualizes the controller */
	strcpy(pThis->g.dev.bus_id, "gadget");
	pThis->g.dev.parent = pThis->controller;
	pThis->g.dev.dma_mask = pThis->controller->dma_mask;
	pThis->g.dev.release = musb_gadget_release;
	pThis->g.name = musb_driver_name;

	musb_g_init_endpoints(pThis);

	status = device_register(&pThis->g.dev);
	if (status != 0)
		the_gadget = NULL;

	return status;
}

void musb_gadget_cleanup(struct musb *pThis)
{
	if (pThis != the_gadget)
		return;

	device_unregister(&pThis->g.dev);
	the_gadget = NULL;
}

/*
 * Register the gadget driver. Used by gadget drivers when
 * registering themselves with the controller.
 *
 * -EINVAL something went wrong (not driver)
 * -EBUSY another gadget is already using the controller
 * -ENOMEM no memeory to perform the operation
 *
 * @param driver the gadget driver
 * @return <0 if error, 0 if everything is fine
 */
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	int retval;
	unsigned long flags;
	struct musb *pThis = the_gadget;

	if (!driver || driver->speed != USB_SPEED_HIGH || !driver->bind
	    || !driver->unbind || !driver->setup)
		return -EINVAL;

	/* driver must be initialized to support peripheral mode */
	if (!pThis || !(pThis->board_mode == MUSB_OTG
			|| pThis->board_mode != MUSB_OTG)) {
		DBG(1, "%s, no dev??\n", __FUNCTION__);
		return -ENODEV;
	}

	DBG(3, "registering driver %s\n", driver->function);
	spin_lock_irqsave(&pThis->Lock, flags);

	if (pThis->pGadgetDriver) {
		DBG(1, "%s is already bound to %s\n",
		    musb_driver_name, pThis->pGadgetDriver->driver.name);
		retval = -EBUSY;
	} else {
		pThis->pGadgetDriver = driver;
		pThis->g.dev.driver = &driver->driver;
		driver->driver.bus = NULL;
		pThis->softconnect = 1;
		retval = 0;
	}

	spin_unlock_irqrestore(&pThis->Lock, flags);

	if (retval == 0)
		retval = driver->bind(&pThis->g);
	if (retval != 0) {
		DBG(3, "bind to driver %s failed --> %d\n",
		    driver->driver.name, retval);
		pThis->pGadgetDriver = NULL;
		pThis->g.dev.driver = NULL;
	}

	/* start peripheral and/or OTG engines */
	if (retval == 0) {
		spin_lock_irqsave(&pThis->Lock, flags);

		/* REVISIT always use otg_set_peripheral(), handling
		 * issues including the root hub one below ...
		 */
		/* HBG 21SEPT2006 OTG implementation */
#ifdef CONFIG_USB_MUSB_OTG
		otg_set_peripheral(&(pThis->OtgMachine.xceiv), &(pThis->g));
#endif
		/* FIXME this ignores the softconnect flag.  Drivers are
		 * allowed hold the peripheral inactive until for example
		 * userspace hooks up printer hardware or DSP codecs, so
		 * hosts only see fully functional devices.
		 */

		musb_start(pThis);
		spin_unlock_irqrestore(&pThis->Lock, flags);
	}

	return retval;
}

EXPORT_SYMBOL(usb_gadget_register_driver);

static void
stop_activity(struct musb *musb, struct usb_gadget_driver *driver,
	      u8 fifo_flush)
{
	int i;
	struct musb_hw_ep *hw_ep;

	if (driver && (fifo_flush == 0)) {
		/* don't disconnect if it's not connected */
		if (musb->g.speed == USB_SPEED_UNKNOWN)
			driver = NULL;
		else
			musb->g.speed = USB_SPEED_UNKNOWN;

		/* deactivate the hardware */
		if (musb->softconnect) {
			musb->softconnect = 0;
			musb_pullup(musb, 0);
		}

		musb_stop(musb);
	}
	/* killing any outstanding requests will quiesce the driver;
	 * then report disconnect
	 */
	{
		for (i = 0, hw_ep = musb->aLocalEnd;
		     i < musb->bEndCount; i++, hw_ep++) {
			MGC_SelectEnd(musb->pRegs, hw_ep->bLocalEnd);
			if (hw_ep->bIsSharedFifo /* || !bEnd */ ) {
				spin_unlock(&musb->Lock);
				musb_gadget_disable(&hw_ep->ep_in.end_point);
				spin_lock(&musb->Lock);
			} else {
				if (hw_ep->wMaxPacketSizeTx) {
					spin_unlock(&musb->Lock);
					musb_gadget_disable(&hw_ep->ep_in.
							    end_point);
					spin_lock(&musb->Lock);
				}
				if (hw_ep->wMaxPacketSizeRx) {
					spin_unlock(&musb->Lock);
					musb_gadget_disable(&hw_ep->ep_out.
							    end_point);
					spin_lock(&musb->Lock);
				}
			}
		}

		spin_unlock(&musb->Lock);

		if (driver && (fifo_flush == 0))
			driver->disconnect(&musb->g);

		spin_lock(&musb->Lock);
	}
}

/*
 * Unregister the gadget driver. Used by gadget drivers when
 * unregistering themselves from the controller.
 *
 * @param driver the gadget driver to unregister
 */
int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	unsigned long flags;
	int retval = 0;
	struct musb *pThis = the_gadget;

	if (!driver || !pThis)
		return -EINVAL;

	/* REVISIT always use otg_set_peripheral() here too;
	 * this needs to shut down the OTG engine.
	 */

	spin_lock_irqsave(&pThis->Lock, flags);
	if (pThis->pGadgetDriver == driver) {
		stop_activity(pThis, driver, 0);

		DBG(3, "unregistering driver %s\n", driver->function);
		spin_unlock_irqrestore(&pThis->Lock, flags);
		driver->unbind(&pThis->g);
		spin_lock_irqsave(&pThis->Lock, flags);

		pThis->pGadgetDriver = NULL;
		pThis->g.dev.driver = NULL;
	} else
		retval = -EINVAL;

	spin_unlock_irqrestore(&pThis->Lock, flags);
	return retval;
}

EXPORT_SYMBOL(usb_gadget_unregister_driver);

/***********************************************************************/

/* lifecycle operations called through plat_uds.c */

void musb_g_resume(struct musb *pThis)
{
	DBG(4, "<==\n");
	if (pThis->pGadgetDriver && pThis->pGadgetDriver->resume) {
		spin_unlock(&pThis->Lock);
		pThis->pGadgetDriver->resume(&pThis->g);
		spin_lock(&pThis->Lock);
	}
}

/* called when SOF packets stop for 3+ msec */
void musb_g_suspend(struct musb *pThis)
{
	u8 devctl;

	devctl = musb_readb(pThis->pRegs, MGC_O_HDRC_DEVCTL);
	DBG(3, "devctl %02x\n", devctl);

	if (pThis->pGadgetDriver && pThis->pGadgetDriver->suspend) {
		spin_unlock(&pThis->Lock);
		pThis->pGadgetDriver->suspend(&pThis->g);
		spin_lock(&pThis->Lock);
	}
}

/* called when VBUS drops below session threshold, and in other cases */
void musb_g_disconnect(struct musb *pThis)
{
	DBG(3, "devctl %02x\n", musb_readb(pThis->pRegs, MGC_O_HDRC_DEVCTL));

	pThis->g.speed = USB_SPEED_UNKNOWN;
	stop_activity(pThis, NULL, 1);
	if (pThis->pGadgetDriver && pThis->pGadgetDriver->disconnect) {
		spin_unlock(&pThis->Lock);
		pThis->pGadgetDriver->disconnect(&pThis->g);
		spin_lock(&pThis->Lock);
	}
}

void musb_g_reset(struct musb *pThis)
__releases(pThis->Lock) __acquires(pThis->Lock)
{
	void __iomem *pBase = pThis->pRegs;
	u8 devctl = musb_readb(pBase, MGC_O_HDRC_DEVCTL);
	u8 power;

	DBG(3, "<== %s addr=%x driver '%s'\n", (devctl & MGC_M_DEVCTL_BDEVICE)
	    ? "B-Device" : "A-Device",
	    musb_readb(pBase, MGC_O_HDRC_FADDR),
	    pThis->pGadgetDriver ? pThis->pGadgetDriver->driver.name : NULL);

	/* HR does NOT clear itself */
	if (devctl & MGC_M_DEVCTL_HR)
		musb_writeb(pBase, MGC_O_HDRC_DEVCTL, MGC_M_DEVCTL_SESSION);

	/* report disconnect, if we didn't already (flushing EP state) */
	if (pThis->g.speed != USB_SPEED_UNKNOWN)
		musb_g_disconnect(pThis);

	/* what speed did we negotiate? */
	power = musb_readb(pBase, MGC_O_HDRC_POWER);
	pThis->g.speed = (power & MGC_M_POWER_HSMODE)
	    ? USB_SPEED_HIGH : USB_SPEED_FULL;

	/* start in USB_STATE_DEFAULT */
	MUSB_DEV_MODE(pThis);
	pThis->bAddress = 0;
	pThis->ep0_state = MGC_END0_STAGE_SETUP;

	pThis->bMayWakeup = 0;
	pThis->g.b_hnp_enable = 0;
	pThis->g.a_alt_hnp_support = 0;
	pThis->g.a_hnp_support = 0;

}
