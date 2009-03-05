/******************************************************************************
*
*     Author: Xilinx, Inc.
*
*
*     This program is free software; you can redistribute it and/or modify it
*     under the terms of the GNU General Public License as published by the
*     Free Software Foundation; either version 2 of the License, or (at your
*     option) any later version.
*
*
*     XILINX IS PROVIDING THIS DESIGN, CODE, OR INFORMATION "AS IS" AS A
*     COURTESY TO YOU. BY PROVIDING THIS DESIGN, CODE, OR INFORMATION AS
*     ONE POSSIBLE IMPLEMENTATION OF THIS FEATURE, APPLICATION OR STANDARD,
*     XILINX IS MAKING NO REPRESENTATION THAT THIS IMPLEMENTATION IS FREE
*     FROM ANY CLAIMS OF INFRINGEMENT, AND YOU ARE RESPONSIBLE FOR OBTAINING
*     ANY THIRD PARTY RIGHTS YOU MAY REQUIRE FOR YOUR IMPLEMENTATION.
*     XILINX EXPRESSLY DISCLAIMS ANY WARRANTY WHATSOEVER WITH RESPECT TO
*     THE ADEQUACY OF THE IMPLEMENTATION, INCLUDING BUT NOT LIMITED TO ANY
*     WARRANTIES OR REPRESENTATIONS THAT THIS IMPLEMENTATION IS FREE FROM
*     CLAIMS OF INFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY AND
*     FITNESS FOR A PARTICULAR PURPOSE.
*
*
*     Xilinx hardware products are not intended for use in life support
*     appliances, devices, or systems. Use in such applications is
*     expressly prohibited.
*
*
*     (c) Copyright 2005 Xilinx Inc.
*     All rights reserved.
*
*
*     You should have received a copy of the GNU General Public License along
*     with this program; if not, write to the Free Software Foundation, Inc.,
*     675 Mass Ave, Cambridge, MA 02139, USA.
*
******************************************************************************/
/*****************************************************************************/
/**
*
* @file xtemac_intr_sgdma.c
*
* Functions in this file implement interrupt related operations for
* scatter gather DMA packet transfer mode. See xtemac.h for a detailed
* description of the driver.
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date     Changes
* ----- ---- -------- -------------------------------------------------------
* 1.00a rmm  06/01/05 First release
* </pre>
******************************************************************************/

/***************************** Include Files *********************************/

#include "xtemac.h"
#include "xtemac_i.h"

/************************** Constant Definitions *****************************/

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/

/* shortcut macros */
#define ERR_HANDLER(Class, Word1, Word2)  \
    InstancePtr->ErrorHandler(InstancePtr->ErrorRef, Class, Word1, Word2)

#define SGSEND_HANDLER() \
    InstancePtr->SgSendHandler(InstancePtr->SgSendRef)

#define SGRECV_HANDLER() \
    InstancePtr->SgRecvHandler(InstancePtr->SgRecvRef)

/************************** Function Prototypes ******************************/

/************************** Variable Definitions *****************************/

/*****************************************************************************/
/**
*
* Enable DMA related interrupts for SG DMA frame transfer mode.
*
* @param InstancePtr is a pointer to the instance to be worked on.
* @param Direction specifies whether the transmit related (XTE_SEND) or
*        receive related (XTE_RECV) interrupts should be affected, or
*        both (XTE_SEND | XTE_RECV).
*
* @note
* The state of the transmitter and receiver are not modified by this function.
*
******************************************************************************/
void XTemac_IntrSgEnable(XTemac * InstancePtr, u32 Direction)
{
	u32 RegIPIER;
	u32 RegDmaIER;

	XASSERT_VOID(InstancePtr != NULL);
	XASSERT_VOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);
	XASSERT_VOID(!(Direction & ~(XTE_SEND | XTE_RECV)));

	/* Default DMA mask */
	RegDmaIER = XDMAV2_IPXR_PCTR_MASK |
	    XDMAV2_IPXR_PWBR_MASK | XDMAV2_IPXR_DE_MASK;

	/* Add to dma mask if SGEND option is enabled */
	if (InstancePtr->Options & XTE_SGEND_INT_OPTION) {
		RegDmaIER |= XDMAV2_IPXR_SGEND_MASK;
	}

	/* Get current contents of core's IER. Depending on direction(s)
	 * specified, status/length FIFO error interrupt enables will be enabled
	 */
	RegIPIER = XTemac_mGetIpifReg(XTE_IPIER_OFFSET);

	/* Set interrupts for transmit DMA channel */
	if (Direction & XTE_SEND) {
		/* DMA interrupt enable */
		XDmaV2_SetInterruptEnable(&InstancePtr->SendDma, RegDmaIER);
		InstancePtr->Flags |= XTE_FLAGS_SEND_SGDMA_INT_ENABLE;

		/* Mask in core's transmit interrupt enables */
		RegIPIER |= XTE_IPXR_XMIT_ERROR_MASK;

		/* Don't allow Tx status overrun interrupt if option is cleared */
		if (!
		    (InstancePtr->
		     Options & XTE_REPORT_TXSTATUS_OVERRUN_OPTION)) {
			RegIPIER &= ~XTE_IPXR_XMIT_SFIFO_OVER_MASK;;
		}
	}

	/* Set interrupts for receive DMA channel */
	if (Direction & XTE_RECV) {
		/* DMA interrupt enable */
		XDmaV2_SetInterruptEnable(&InstancePtr->RecvDma, RegDmaIER);
		InstancePtr->Flags |= XTE_FLAGS_RECV_SGDMA_INT_ENABLE;

		/* Mask in core's receive interrupt enables */
		RegIPIER |= XTE_IPXR_RECV_ERROR_MASK;

		/* Don't enable recv reject errors if option is cleared */
		if (!(InstancePtr->Options & XTE_REPORT_RXERR_OPTION)) {
			RegIPIER &= ~XTE_IPXR_RECV_DROPPED_MASK;
		}
	}

	/* Update core interrupt enables */
	XTemac_mSetIpifReg(XTE_IPIER_OFFSET, RegIPIER);
}

/*****************************************************************************/
/**
*
* Disable DMA related interrupts for SG DMA frame transfer mode.
*
* @param InstancePtr is a pointer to the instance to be worked on.
* @param Direction specifies whether the transmit related (XTE_SEND) or
*        receive related (XTE_RECV) interrupts should be affected, or
*        both (XTE_SEND | XTE_RECV).
*
* @note
* The state of the transmitter and receiver are not modified by this function.
*
******************************************************************************/
void XTemac_IntrSgDisable(XTemac * InstancePtr, u32 Direction)
{
	u32 RegIPIER;

	XASSERT_VOID(InstancePtr != NULL);
	XASSERT_VOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);
	XASSERT_VOID(!(Direction & ~(XTE_SEND | XTE_RECV)));

	/* Get contents of IPIER register */
	RegIPIER = XTemac_mGetIpifReg(XTE_IPIER_OFFSET);

	if (Direction & XTE_SEND) {
		/* Disable DMA channel interrupt */
		XDmaV2_SetInterruptEnable(&InstancePtr->SendDma, 0);
		InstancePtr->Flags &= ~XTE_FLAGS_SEND_SGDMA_INT_ENABLE;

		/* Mask out length/status FIFO errors on transmit side */
		RegIPIER &= ~XTE_IPXR_XMIT_ERROR_MASK;
	}

	if (Direction & XTE_RECV) {
		/* Disable DMA channel interrupt */
		XDmaV2_SetInterruptEnable(&InstancePtr->RecvDma, 0);
		InstancePtr->Flags &= ~XTE_FLAGS_RECV_SGDMA_INT_ENABLE;

		/* Mask out length/status FIFO errors on receive side */
		RegIPIER &= ~XTE_IPXR_RECV_ERROR_MASK;
	}

	/* Update IPIER with new setting */
	XTemac_mSetIpifReg(XTE_IPIER_OFFSET, RegIPIER);
}

/*****************************************************************************/
/**
*
* Set the SGDMA interrupt coalescing parameters. The device must be stopped
* before setting these parameters. See xtemac.h for a complete discussion of
* the interrupt coalescing features of this device.
*
* @param InstancePtr is a pointer to the instance to be worked on.
* @param Direction indicates the channel, XTE_SEND or XTE_RECV, to set.
* @param Threshold is the value of the packet threshold count used during
*        interrupt coalescing. Valid range is 0 - 1023. A value of 0 disables
*        the use of packet threshold by the hardware.
* @param Timer is the waitbound timer value in units of approximately
*        milliseconds. Valid range is 0 - 1023. A value of 0 disables the use
*        of the waitbound timer by the hardware.
*
* @return
* - XST_SUCCESS if the threshold was successfully set
* - XST_NO_FEATURE if the MAC is not configured for scatter-gather DMA
* - XST_DEVICE_IS_STARTED if the device has not been stopped
* - XST_INVALID_PARAM if Direction does not indicate a valid channel
*
******************************************************************************/
XStatus XTemac_IntrSgCoalSet(XTemac * InstancePtr, u32 Direction,
			     u16 Threshold, u16 Timer)
{
	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	/* Must be SGDMA */
	if (!XTemac_mIsSgDma(InstancePtr)) {
		return (XST_NO_FEATURE);
	}

	/* Device must be stopped before changing these settings */
	if (InstancePtr->IsStarted == XCOMPONENT_IS_STARTED) {
		return (XST_DEVICE_IS_STARTED);
	}

	/* Set HW */
	if (Direction == XTE_SEND) {
		(void)XDmaV2_SgSetPktThreshold(&InstancePtr->SendDma,
					       Threshold);
		XDmaV2_SgSetPktWaitbound(&InstancePtr->SendDma, Timer);
	} else if (Direction == XTE_RECV) {
		(void)XDmaV2_SgSetPktThreshold(&InstancePtr->RecvDma,
					       Threshold);
		XDmaV2_SgSetPktWaitbound(&InstancePtr->RecvDma, Timer);
	} else {
		return (XST_INVALID_PARAM);
	}

	return (XST_SUCCESS);
}

/*****************************************************************************/
/**
*
* Get the current interrupt coalescing settings. See xtemac.h for more
* discussion of interrupt coalescing features.
*
* @param InstancePtr is a pointer to the instance to be worked on.
* @param Direction indicates the channel, XTE_SEND or XTE_RECV, to get.
* @param ThresholdPtr is a pointer to the word into which the current value of
*        the packet threshold will be copied.
* @param TimerPtr is a pointer to the word into which the current value of the
*        waitbound timer will be copied.
*
* @return
* - XST_SUCCESS if the packet threshold was retrieved successfully
* - XST_NO_FEATURE if the MAC is not configured for scatter-gather DMA
* - XST_INVALID_PARAM if Direction does not indicate a valid channel
*
******************************************************************************/
XStatus XTemac_IntrSgCoalGet(XTemac * InstancePtr, u32 Direction,
			     u16 * ThresholdPtr, u16 * TimerPtr)
{
	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(ThresholdPtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	/* Must be SGDMA */
	if (!XTemac_mIsSgDma(InstancePtr)) {
		return (XST_NO_FEATURE);
	}

	/* Get data from HW */
	if (Direction == XTE_SEND) {
		*ThresholdPtr = XDmaV2_SgGetPktThreshold(&InstancePtr->SendDma);
		*TimerPtr = XDmaV2_SgGetPktWaitbound(&InstancePtr->SendDma);
	} else if (Direction == XTE_RECV) {
		*ThresholdPtr = XDmaV2_SgGetPktThreshold(&InstancePtr->RecvDma);
		*TimerPtr = XDmaV2_SgGetPktWaitbound(&InstancePtr->RecvDma);
	} else {
		return (XST_INVALID_PARAM);
	}

	return (XST_SUCCESS);
}

/*****************************************************************************/
/**
* Master interrupt handler for SGDMA frame transfer mode. This routine will
* query the status of the device, bump statistics, and invoke user callbacks
* in the following priority:
*
*  - XTE_HANDLER_ERROR for error class XST_PFIFO_DEADLOCK.
*  - XTE_HANDLER_ERROR for error class XST_FIFO_ERROR.
*  - XTE_HANDLER_ERROR for error class XST_DMA_ERROR for the recv DMA channel
*  - XTE_HANDLER_SGRECV for newly received frames
*  - XTE_HANDLER_ERROR for error class XST_DMA_ERROR for the xmit DMA channel
*  - XTE_HANDLER_SGSEND for completed transmits
*  - XTE_HANDLER_ERROR for error class XST_RECV_ERROR.
*
* This routine must be connected to an interrupt controller using OS/BSP
* specific methods.
*
* @param InstancePtr is a pointer to the TEMAC instance that has caused the
*        interrupt.
*
******************************************************************************/
void XTemac_IntrSgHandler(void *TemacPtr)
{
	u32 RegDISR;
	u32 CorePending;
	u32 RegDmaPending;
	XTemac *InstancePtr = (XTemac *) TemacPtr;

	XASSERT_VOID(InstancePtr != NULL);

	/* This ISR will try to handle as many interrupts as it can in a single
	 * call. However, in most of the places where the user's error handler is
	 * called, this ISR exits because it is expected that the user will reset
	 * the device in nearly all instances.
	 */

	/* Log interrupt */
	XTemac_mBumpStats(Interrupts, 1);

	/* Get top level interrupt status */
	RegDISR = XTemac_mGetIpifReg(XTE_DISR_OFFSET);

	/* Handle IPIF and packet FIFO errors */
	if (RegDISR & (XTE_DXR_DPTO_MASK | XTE_DXR_TERR_MASK |
		       XTE_DXR_RECV_FIFO_MASK | XTE_DXR_SEND_FIFO_MASK)) {
		/* IPIF transaction or data phase error */
		if (RegDISR & (XTE_DXR_DPTO_MASK | XTE_DXR_TERR_MASK)) {
			XTemac_mBumpStats(IpifErrors, 1);
			ERR_HANDLER(XST_IPIF_ERROR, RegDISR, 0);
			return;
		}

		/* Receive packet FIFO is deadlocked */
		if (RegDISR & XTE_DXR_RECV_FIFO_MASK) {
			XTemac_mBumpStats(RxPktFifoErrors, 1);
			ERR_HANDLER(XST_PFIFO_DEADLOCK, XTE_RECV, 0);
			return;
		}

		/* Transmit packet FIFO is deadlocked */
		if (RegDISR & XTE_DXR_SEND_FIFO_MASK) {
			XTemac_mBumpStats(TxPktFifoErrors, 1);
			ERR_HANDLER(XST_PFIFO_DEADLOCK, XTE_SEND, 0);
			return;
		}
	}

	/* Get currently pending core interrupts */
	CorePending = XTemac_mGetIpifReg(XTE_IPIER_OFFSET) &
	    XTemac_mGetIpifReg(XTE_IPISR_OFFSET);

	/* Check for fatal status/length FIFO errors. These errors can't be
	 * cleared
	 */
	if (CorePending & XTE_IPXR_FIFO_FATAL_ERROR_MASK) {
		XTemac_mBumpStats(FifoErrors, 1);
		ERR_HANDLER(XST_FIFO_ERROR,
			    CorePending & XTE_IPXR_FIFO_FATAL_ERROR_MASK, 0);
		return;
	}

	/* Check for SGDMA receive interrupts */
	if (RegDISR & XTE_DXR_RECV_DMA_MASK) {
		/* Get interrupts pending */
		RegDmaPending =
		    XDmaV2_GetInterruptStatus(&InstancePtr->RecvDma);

		/* Check for errors */
		if (RegDmaPending & XDMAV2_IPXR_DE_MASK) {
			XTemac_mBumpStats(RxDmaErrors, 1);
			ERR_HANDLER(XST_DMA_ERROR, XTE_RECV,
				    XDmaV2_mGetStatus(&InstancePtr->RecvDma));
			return;
		}

		/* Check for packets processed */
		if (RegDmaPending & (XDMAV2_IPXR_PCTR_MASK |
				     XDMAV2_IPXR_PWBR_MASK |
				     XDMAV2_IPXR_SGEND_MASK)) {
			/* Invoke the user's receive handler. The handler may remove the
			 * ready BDs from the list right away or defer until later
			 */
			SGRECV_HANDLER();
		}

		/* Ack interrupts. IPXR_PCTR is a special case. If the handler does not
		 * yank BDs, then the unserviced packet count (UPC) will be unchanged.
		 * So if UPC stays >= packet threshold, the interrupt remains pending
		 * even if it is acked here. The ack must occur in XTemac_SgGet-
		 * Processed() where packets are yanked.
		 */
		XDmaV2_SetInterruptStatus(&InstancePtr->RecvDma,
					  RegDmaPending &
					  ~XDMAV2_IPXR_PCTR_MASK);
	}

	/* Check for SGDMA transmit interrupts */
	if (RegDISR & XTE_DXR_SEND_DMA_MASK) {
		/* Get interrupts pending */
		RegDmaPending =
		    XDmaV2_GetInterruptStatus(&InstancePtr->SendDma);

		/* Check for errors */
		if (RegDmaPending & XDMAV2_IPXR_DE_MASK) {
			XTemac_mBumpStats(TxDmaErrors, 1);
			ERR_HANDLER(XST_DMA_ERROR, XTE_SEND,
				    XDmaV2_mGetStatus(&InstancePtr->SendDma));
			return;
		}

		/* Check for packets processed */
		if (RegDmaPending & (XDMAV2_IPXR_PCTR_MASK |
				     XDMAV2_IPXR_PWBR_MASK |
				     XDMAV2_IPXR_SGEND_MASK)) {
			/* Invoke the user's send handler. The handler may remove the
			 * ready BDs from the list right away or defer until later
			 */
			SGSEND_HANDLER();
		}

		/* Ack interrupts. IPXR_PCTR is a special case. If the handler does not
		 * yank BDs, then the unserviced packet count (UPC) will be unchanged.
		 * So if UPC stays >= packet threshold, the interrupt remains pending
		 * even if it is acked here. The ack must occur in XTemac_SgGet-
		 * Processed() where packets are yanked.
		 */
		XDmaV2_SetInterruptStatus(&InstancePtr->SendDma,
					  RegDmaPending &
					  ~XDMAV2_IPXR_PCTR_MASK);
	}

	/* Check for dropped receive frame. Ack the interupt then call the
	 * error handler
	 */
	if (CorePending & XTE_IPXR_RECV_DROPPED_MASK) {
		XTemac_mSetIpifReg(XTE_IPISR_OFFSET,
				   CorePending & XTE_IPXR_RECV_DROPPED_MASK);
		XTemac_mBumpStats(RxRejectErrors, 1);
		ERR_HANDLER(XST_RECV_ERROR,
			    CorePending & XTE_IPXR_RECV_DROPPED_MASK, 0);

		/* no return here, nonfatal error */
	}

	/* Ack top level interrupt status */
	XTemac_mSetIpifReg(XTE_DISR_OFFSET, RegDISR);
}
