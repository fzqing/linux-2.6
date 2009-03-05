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
 * DMA Controller Abstraction (DCA) for the Inventra Controller Driver (ICD).
 */

#ifndef __MUSB_DMA_H__
#define __MUSB_DMA_H__

/**
 * Introduction.
 * The purpose of the DMA Controller Abstraction (DCA) is to allow the ICD
 * to use any DMA controller,
 * since this is an option in the Inventra USB cores.
 * The assumptions are:
 *
 * <li>A DMA controller will be tied to an Inventra USB core in the
 * way specified in the Inventra core product specification.
 * <li>A DMA controller's base address in the memory map correlates
 * somehow to the Inventra USB core it serves.
 * <li>Linux sets up the DMA mappings and flushes caches, so the
 * controller doesn't need to.
 *
 * The responsibilities of an implementation include:
 *
 * <li>Handling the details of moving multiple USB packets
 * in cooperation with the Inventra USB core.
 * <li>Knowing the correlation between channels and the
 * Inventra core's local endpoint resources and data direction,
 * and maintaining a list of allocated/available channels.
 * <li>Updating channel status on interrupts,
 * whether shared with the Inventra core or separate.
 * <li>If the DMA interrupt is shared with the Inventra core,
 * handling it when called, and reporting whether it was the
 * source of interrupt.
 *
 */

#define	DMA_ADDR_INVALID	(~(dma_addr_t)0)

#ifndef CONFIG_USB_INVENTRA_FIFO
#define	is_dma_capable()	(1)
#else
#define	is_dma_capable()	(0)
#endif

/*************************** CONSTANTS ****************************/

/**
 * DMA channel status ... updated by the dma controller driver whenever that
 * status changes, and protected by the overall controller spinlock.
 */
enum dma_channel_status {
		/** A channel's status is unknown */
	MGC_DMA_STATUS_UNKNOWN,
		/** A channel is available (not busy and no errors) */
	MGC_DMA_STATUS_FREE,
		/** A channel is busy (not finished attempting its transactions) */
	MGC_DMA_STATUS_BUSY,
		/** A channel aborted its transactions due to a local bus error */
	MGC_DMA_STATUS_BUS_ABORT,
		/** A channel aborted its transactions due to a core error or USB fault */
	MGC_DMA_STATUS_CORE_ABORT
};

/***************************** TYPES ******************************/

/**
 * struct dma_channel - A DMA channel.
 * @field pPrivateData channel-private data; not to be interpreted by the ICD
 * @field wMaxLength the maximum number of bytes the channel can move
 * in one transaction (typically representing many USB maximum-sized packets)
 * @field dwActualLength how many bytes have been transferred
 * @field bStatus current channel status (updated e.g. on interrupt)
 * @field bDesiredMode TRUE if mode 1 is desired; FALSE if mode 0 is desired
 */
struct dma_channel {
	void *pPrivateData;
	size_t dwMaxLength;
	size_t dwActualLength;
	enum dma_channel_status bStatus;
	u8 bDesiredMode;
};

/**
 * Start a DMA controller.
 * @param pPrivateData private data pointer from MGC_DmaController
 * @return TRUE on success
 * @return FALSE on failure (e.g. no DMAC appears present)
 */
typedef u8(*MGC_pfDmaStartController) (void *pPrivateData);

/**
 * Stop a DMA controller.
 * @param pPrivateData the controller's private data pointer
 * @return TRUE on success
 * @return FALSE on failure; the ICD may try again
 */
typedef u8(*MGC_pfDmaStopController) (void *pPrivateData);

/**
 * Allocate a DMA channel.
 * Allocate a DMA channel suitable for the given conditions.
 * @param pPrivateData the controller's private data pointer
 * @param bLocalEnd the local endpoint index (1-15)
 * @param bTransmit TRUE for transmit; FALSE for receive
 * @param bProtocol the USB protocol, as per USB 2.0 chapter 9
 * (0 => control, 1 => isochronous, 2 => bulk, 3 => interrupt)
 * @param wMaxPacketSize maximum packet size
 * @return a non-NULL pointer on success
 * @return NULL on failure (no channel available)
 */
typedef struct dma_channel *(*MGC_pfDmaAllocateChannel) (void *pPrivateData,
							 u8 bLocalEnd,
							 u8 bTransmit,
							 u8 bProtocol,
							 u16 wMaxPacketSize);

/**
 * Release a DMA channel.
 * Release a previously-allocated DMA channel.
 * The ICD guarantess to no longer reference this channel.
 * @param pChannel pointer to a channel obtained by
 * a successful call to pController->pfDmaAllocateChannel
 */
typedef void (*MGC_pfDmaReleaseChannel) (struct dma_channel * pChannel);

/**
 * Program a DMA channel.
 * Program a DMA channel to move data at the core's request.
 * The local core endpoint and direction should already be known,
 * since they are specified in the pfDmaAllocateChannel call.
 * @param pChannel pointer to a channel obtained by
 * a successful call to pController->pfDmaAllocateChannel
 * @param wPacketSize the packet size
 * @param bMode TRUE if mode 1; FALSE if mode 0
 * @param dma_addr base address of data (in DMA space)
 * @param dwLength the number of bytes to transfer;
 * guaranteed by the ICD to be no larger than the channel's reported dwMaxLength
 * @return TRUE on success
 * @return FALSE on error
 */
typedef u8(*MGC_pfDmaProgramChannel) (struct dma_channel * pChannel,
				      u16 wPacketSize, u8 bMode,
				      // const u8* pBuffer,
				      dma_addr_t dma_addr, u32 dwLength);

/**
 * dma_channel_status - return status of dma channel
 * @c: the channel
 *
 * Returns the software's view of the channel status.  If that status is BUSY
 * then it's possible that the hardware has completed (or aborted) a transfer,
 * so the driver needs to update that status.
 */
static inline enum dma_channel_status dma_channel_status(struct dma_channel *c)
{
	return (is_dma_capable() && c) ? c->bStatus : MGC_DMA_STATUS_UNKNOWN;
}

/**
 * DMA ISR.
 * If present, this function is called by the ICD on every interrupt.
 * This is necessary because with the built-in DMA controller
 * (and probably some other configurations),
 * the DMA interrupt is shared with other core interrupts.
 * Therefore, this function should return quickly
 * when there is no DMA interrupt.
 * When there is a DMA interrupt, this function should
 * perform any implementations-specific operations,
 * and update the status of all appropriate channels.
 * If the DMA controller has its own dedicated interrupt,
 * this function should do nothing.
 * This function is called BEFORE the ICD handles other interrupts.
 * @param pPrivateData the controller's private data pointer
 * @return TRUE if an interrupt was serviced
 * @return FALSE if no interrupt required servicing
 */
typedef u8(*MGC_pfDmaControllerIsr) (void *pPrivateData);

/**
 * struct dma_controller - A DMA Controller.
 * This is in a struct to allow the ICD to support
 * multiple cores of different types,
 * since each may use a different type of DMA controller.
 * @field pPrivateData controller-private data;
 * not to be interpreted by the ICD
 * @field pfDmaStartController ICD calls this to start a DMA controller
 * @field pfDmaStopController ICD calls this to stop a DMA controller
 * @field pfDmaAllocateChannel ICD calls this to allocate a DMA channel
 * @field pfDmaReleaseChannel ICD calls this to release a DMA channel
 * @field pfDmaControllerIsr ICD calls this (if non-NULL) from its ISR
 */
struct dma_controller {
	void *pPrivateData;
	MGC_pfDmaStartController pfDmaStartController;
	MGC_pfDmaStopController pfDmaStopController;
	MGC_pfDmaAllocateChannel pfDmaAllocateChannel;
	MGC_pfDmaReleaseChannel pfDmaReleaseChannel;
	MGC_pfDmaProgramChannel pfDmaProgramChannel;
	int (*pfDmaAbortChannel) (struct dma_channel *);
	MGC_pfDmaControllerIsr pfDmaControllerIsr;
};

/*
 * A DMA channel has new status.
 * This may be used to notify the ICD of channel status changes asynchronously.
 * This is useful if the DMA interrupt is different from the USB controller's
 * interrupt, so on some systems there may be no control over the order of
 * USB controller and DMA controller assertion.
 * @param pPrivateData the controller's private data pointer
 * @param bLocalEnd the local endpoint index (1-15)
 * @param bTransmit TRUE for transmit; FALSE for receive
 * @return zero for success, else a negative errno fault code
 */
typedef int (*MGC_pfDmaChannelStatusChanged) (void *pPrivateData,
					      u8 bLocalEnd, u8 bTransmit);

/**
 * Instantiate a DMA controller.
 * Instantiate a software object representing a DMA controller.
 * @param pfDmaChannelStatusChanged channel status change notification function.
 * Normally, the ICD requests status in its interrupt handler.
 * For some DMA controllers, this may not be the correct time.
 * @param pDmaPrivate parameter for pfDmaChannelStatusChanged
 * @param pCoreBase the base address (in kernel space) of the core
 * It is assumed the DMA controller's registers' base address will be related
 * to this in some way.
 * @return non-NULL pointer on success
 * @return NULL on failure (out of memory or exhausted
 * a fixed number of controllers)
 */
typedef struct dma_controller
*(*MGC_pfNewDmaController) (MGC_pfDmaChannelStatusChanged
			    pfDmaChannelStatusChanged, void *pDmaPrivate,
			    u8 * pCoreBase);

/**
 * Destroy DMA controller.
 * Destroy a previously-instantiated DMA controller.
 */
typedef void (*MGC_pfDestroyDmaController) (struct dma_controller *
					    pController);

/**
 * struct dma_controller_factory: DMA controller factory
 * @pfNewDmaController: create a DMA controller
 * @pfDestroyDmaController: destroy a DMA controller
 *
 * To allow for multi-core implementations and different
 * types of cores and DMA controllers to co-exist,
 * (only at the source level; no runtime coexistence supported)
 * it is necessary to create them from factories.
 */
struct dma_controller_factory {
	MGC_pfNewDmaController pfNewDmaController;
	MGC_pfDestroyDmaController pfDestroyDmaController;
};

extern struct dma_controller_factory dma_controller_factory;

#endif				/* __MUSB_DMA_H__ */
