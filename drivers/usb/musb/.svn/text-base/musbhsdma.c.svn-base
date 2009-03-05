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

/*
 * DMA implementation for high-speed controllers.
 */

#ifndef CONFIG_USB_INVENTRA_DMA
#error misconfigured, why are you trying to build this
#endif

#define MUSB_HSDMA

#include "musbdefs.h"

/****************************** CONSTANTS ********************************/

#define MGC_O_HSDMA_BASE    0x200
#define MGC_O_HSDMA_INTR    0x200

#define MGC_O_HSDMA_CONTROL 4
#define MGC_O_HSDMA_ADDRESS 8
#define MGC_O_HSDMA_COUNT   0xc

#define MGC_HSDMA_CHANNEL_OFFSET(_bChannel, _bOffset) (MGC_O_HSDMA_BASE + (_bChannel << 4) + _bOffset)

/* control register (16-bit): */
#define MGC_S_HSDMA_ENABLE	0
#define MGC_S_HSDMA_TRANSMIT	1
#define MGC_S_HSDMA_MODE1	2
#define MGC_S_HSDMA_IRQENABLE	3
#define MGC_S_HSDMA_ENDPOINT	4
#define MGC_S_HSDMA_BUSERROR	8
#define MGC_S_HSDMA_BURSTMODE	9
#define MGC_M_HSDMA_BURSTMODE	(3 << MGC_S_HSDMA_BURSTMODE)
#define MGC_HSDMA_BURSTMODE_UNSPEC  0
#define MGC_HSDMA_BURSTMODE_INCR4   1
#define MGC_HSDMA_BURSTMODE_INCR8   2
#define MGC_HSDMA_BURSTMODE_INCR16  3

/******************************* Types ********************************/

struct _MGC_HsDmaController;

typedef struct {
	struct dma_channel Channel;
	struct _MGC_HsDmaController *pController;
	u32 dwStartAddress;
	u32 dwCount;
	u16 wMaxPacketSize;
	u8 bIndex;
	u8 bEnd;
	u8 bTransmit;
} MGC_HsDmaChannel;

typedef struct _MGC_HsDmaController {
	struct dma_controller Controller;
	MGC_HsDmaChannel aChannel[MGC_HSDMA_CHANNELS];
	MGC_pfDmaChannelStatusChanged pfDmaChannelStatusChanged;
	void *pDmaPrivate;
	u8 *pCoreBase;
	u8 bChannelCount;
	u8 bmUsedChannels;
} MGC_HsDmaController;

/******************************* FORWARDS ********************************/

static u8 MGC_HsDmaStartController(void *pPrivateData);

static u8 MGC_HsDmaStopController(void *pPrivateData);

static struct dma_channel *MGC_HsDmaAllocateChannel(void *pPrivateData,
						    u8 bLocalEnd, u8 bTransmit,
						    u8 bProtocol,
						    u16 wMaxPacketSize);

static void MGC_HsDmaReleaseChannel(struct dma_channel *pChannel);

static u8 MGC_HsDmaProgramChannel(struct dma_channel *pChannel,
				  u16 wPacketSize, u8 bMode,
				  dma_addr_t dma_addr, u32 dwLength);

static u8 MGC_HsDmaControllerIsr(void *pPrivateData);

static struct dma_controller
    *MGC_HsNewDmaController(MGC_pfDmaChannelStatusChanged
			    pfDmaChannelStatusChanged, void *pDmaPrivate,
			    u8 * pCoreBase);

static void MGC_HsDestroyDmaController(struct dma_controller *pController);

/******************************* GLOBALS *********************************/

struct dma_controller_factory dma_controller_factory = {
	.pfNewDmaController = MGC_HsNewDmaController,
	.pfDestroyDmaController = MGC_HsDestroyDmaController,
};

/****************************** FUNCTIONS ********************************/

#ifdef MUSB_HSDMA

static u8 MGC_HsDmaStartController(void *pPrivateData)
{
	/* nothing to do */
	return TRUE;
}

static u8 MGC_HsDmaStopController(void *pPrivateData)
{
	/* nothing to do */
	return TRUE;
}

static struct dma_channel *MGC_HsDmaAllocateChannel(void *pPrivateData,
						    u8 bLocalEnd, u8 bTransmit,
						    u8 bProtocol,
						    u16 wMaxPacketSize)
{
	u8 bBit;
	struct dma_channel *pChannel = NULL;
	MGC_HsDmaChannel *pImplChannel = NULL;
	MGC_HsDmaController *pController = (MGC_HsDmaController *) pPrivateData;

	for (bBit = 0; bBit < MGC_HSDMA_CHANNELS; bBit++) {
		if (!(pController->bmUsedChannels & (1 << bBit))) {
			pController->bmUsedChannels |= (1 << bBit);
			pImplChannel = &(pController->aChannel[bBit]);
			pImplChannel->pController = pController;
			pImplChannel->wMaxPacketSize = wMaxPacketSize;
			pImplChannel->bIndex = bBit;
			pImplChannel->bEnd = bLocalEnd;
			pImplChannel->bTransmit = bTransmit;
			pChannel = &(pImplChannel->Channel);
			pChannel->pPrivateData = pImplChannel;
			pChannel->bStatus = MGC_DMA_STATUS_FREE;
			pChannel->dwMaxLength = 0x10000;
			/* Tx => mode 1; Rx => mode 0 */
			pChannel->bDesiredMode = bTransmit;
			break;
		}
	}
	return pChannel;
}

static void MGC_HsDmaReleaseChannel(struct dma_channel *pChannel)
{
	MGC_HsDmaChannel *pImplChannel =
	    (MGC_HsDmaChannel *) pChannel->pPrivateData;

	pImplChannel->pController->bmUsedChannels &=
	    ~(1 << pImplChannel->bIndex);
	pImplChannel->Channel.bStatus = MGC_DMA_STATUS_FREE;
}

static u8 MGC_HsDmaProgramChannel(struct dma_channel *pChannel,
				  u16 wPacketSize, u8 bMode,
				  dma_addr_t dma_addr, u32 dwLength)
{
	MGC_HsDmaChannel *pImplChannel =
	    (MGC_HsDmaChannel *) pChannel->pPrivateData;
	MGC_HsDmaController *pController = pImplChannel->pController;
	u8 *pBase = pController->pCoreBase;
	u16 wCsr =
	    (pImplChannel->
	     bEnd << MGC_S_HSDMA_ENDPOINT) | (1 << MGC_S_HSDMA_ENABLE);
	u8 bChannel = pImplChannel->bIndex;

	if (bMode) {
		wCsr |= 1 << MGC_S_HSDMA_MODE1;
		if (dwLength < wPacketSize) {
			return FALSE;
		}
		if (pImplChannel->wMaxPacketSize >= 64) {
			wCsr |=
			    MGC_HSDMA_BURSTMODE_INCR16 << MGC_S_HSDMA_BURSTMODE;
		} else if (pImplChannel->wMaxPacketSize >= 32) {
			wCsr |=
			    MGC_HSDMA_BURSTMODE_INCR8 << MGC_S_HSDMA_BURSTMODE;
		} else if (pImplChannel->wMaxPacketSize >= 16) {
			wCsr |=
			    MGC_HSDMA_BURSTMODE_INCR4 << MGC_S_HSDMA_BURSTMODE;
		}
	}

	if (pImplChannel->bTransmit) {
		wCsr |= 1 << MGC_S_HSDMA_TRANSMIT;
	}
	wCsr |= 1 << MGC_S_HSDMA_IRQENABLE;

	/* address/count */
	musb_writel(pBase,
		    MGC_HSDMA_CHANNEL_OFFSET(bChannel, MGC_O_HSDMA_ADDRESS),
		    dma_addr);
	musb_writel(pBase,
		    MGC_HSDMA_CHANNEL_OFFSET(bChannel, MGC_O_HSDMA_COUNT),
		    dwLength);

	/* control (this should start things) */
	pChannel->dwActualLength = 0L;
	pImplChannel->dwStartAddress = dma_addr;
	pImplChannel->dwCount = dwLength;
	musb_writew(pBase,
		    MGC_HSDMA_CHANNEL_OFFSET(bChannel, MGC_O_HSDMA_CONTROL),
		    wCsr);

	return TRUE;
}

/* FIXME just update the status when it changes ... */
static enum dma_channel_status MGC_HsDmaGetChannelStatus(struct dma_channel
							 *pChannel)
{
	u32 dwAddress;
	MGC_HsDmaChannel *pImplChannel =
	    (MGC_HsDmaChannel *) pChannel->pPrivateData;
	MGC_HsDmaController *pController = pImplChannel->pController;
	u8 *pBase = pController->pCoreBase;
	u8 bChannel = pImplChannel->bIndex;
	u16 wCsr = musb_readw(pBase,
			      MGC_HSDMA_CHANNEL_OFFSET(bChannel,
						       MGC_O_HSDMA_CONTROL));

	if (wCsr & (1 << MGC_S_HSDMA_BUSERROR)) {
		return MGC_DMA_STATUS_BUS_ABORT;
	}
	/* most DMA controllers would update the count register for simplicity... */
	dwAddress =
	    musb_readl(pBase,
		       MGC_HSDMA_CHANNEL_OFFSET(bChannel, MGC_O_HSDMA_ADDRESS));
	if (dwAddress < (pImplChannel->dwStartAddress + pImplChannel->dwCount)) {
		return MGC_DMA_STATUS_BUSY;
	}
	return MGC_DMA_STATUS_FREE;
}

static u8 MGC_HsDmaControllerIsr(void *pPrivateData)
{
	u8 bChannel;
	u16 wCsr;
	u32 dwAddress;
	MGC_HsDmaChannel *pImplChannel;
	MGC_HsDmaController *pController = (MGC_HsDmaController *) pPrivateData;
	u8 *pBase = pController->pCoreBase;
	u8 bIntr = musb_readb(pBase, MGC_O_HSDMA_INTR);

	if (!bIntr) {
		return FALSE;
	}
	for (bChannel = 0; bChannel < MGC_HSDMA_CHANNELS; bChannel++) {
		if (bIntr & (1 << bChannel)) {
			pImplChannel =
			    (MGC_HsDmaChannel *) & (pController->
						    aChannel[bChannel]);
			wCsr =
			    musb_readw(pBase,
				       MGC_HSDMA_CHANNEL_OFFSET(bChannel,
								MGC_O_HSDMA_CONTROL));
			if (wCsr & (1 << MGC_S_HSDMA_BUSERROR)) {
				pImplChannel->Channel.bStatus =
				    MGC_DMA_STATUS_BUS_ABORT;
			} else {
				/* most DMA controllers would update the count register for simplicity... */
				dwAddress = musb_readl(pBase,
						       MGC_HSDMA_CHANNEL_OFFSET
						       (bChannel,
							MGC_O_HSDMA_ADDRESS));
				pImplChannel->Channel.bStatus =
				    MGC_DMA_STATUS_FREE;
				pImplChannel->Channel.dwActualLength =
				    dwAddress - pImplChannel->dwStartAddress;
				pController->
				    pfDmaChannelStatusChanged(pController->
							      pDmaPrivate,
							      pImplChannel->
							      bEnd,
							      pImplChannel->
							      bTransmit);
			}
		}
	}
	return TRUE;
}

#endif				/* MUSB_HSDMA */

static struct dma_controller
    *MGC_HsNewDmaController(MGC_pfDmaChannelStatusChanged
			    pfDmaChannelStatusChanged, void *pDmaPrivate,
			    u8 * pCoreBase)
{
	struct dma_controller *pResult = NULL;
#ifdef MUSB_HSDMA
	MGC_HsDmaController *pController =
	    (MGC_HsDmaController *) kmalloc(sizeof(MGC_HsDmaController),
					    GFP_KERNEL);
	if (pController) {
		memset(pController, 0, sizeof(MGC_HsDmaController));
		pController->bChannelCount = MGC_HSDMA_CHANNELS;
		pController->pfDmaChannelStatusChanged =
		    pfDmaChannelStatusChanged;
		pController->pDmaPrivate = pDmaPrivate;
		pController->pCoreBase = pCoreBase;
		pController->Controller.pPrivateData = pController;
		pController->Controller.pfDmaStartController =
		    MGC_HsDmaStartController;
		pController->Controller.pfDmaStopController =
		    MGC_HsDmaStopController;
		pController->Controller.pfDmaAllocateChannel =
		    MGC_HsDmaAllocateChannel;
		pController->Controller.pfDmaReleaseChannel =
		    MGC_HsDmaReleaseChannel;
		pController->Controller.pfDmaAllocateBuffer =
		    MGC_HsDmaAllocateBuffer;
		pController->Controller.pfDmaReleaseBuffer =
		    MGC_HsDmaReleaseBuffer;
		pController->Controller.pfDmaProgramChannel =
		    MGC_HsDmaProgramChannel;
		pController->Controller.pfDmaControllerIsr =
		    MGC_HsDmaControllerIsr;
		pResult = &(pController->Controller);
	}
#endif
	return pResult;
}

static void MGC_HsDestroyDmaController(struct dma_controller *pController)
{
#ifdef MUSB_HSDMA
	MGC_HsDmaController *pHsController =
	    (MGC_HsDmaController *) pController->pPrivateData;

	if (pHsController) {
		pHsController->Controller.pPrivateData = NULL;
		kfree(pHsController);
	}
#endif
}
