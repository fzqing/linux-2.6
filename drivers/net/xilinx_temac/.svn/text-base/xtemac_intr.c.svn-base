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
* @file xtemac_intr.c
*
* Functions in this file implement general purpose interrupt processing related
* functionality. See xtemac.h for a detailed description of the driver.
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

/************************** Constant Definitions *****************************/

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/

/************************** Function Prototypes ******************************/

/************************** Variable Definitions *****************************/

/*****************************************************************************/
/**
 * Install an asynchronious handler function for the given HandlerType:
 *
 * <pre>
 * HandlerType               Function Type
 * ------------------------  ---------------------------
 * XTE_HANDLER_FIFOSEND      XTemac_FifoSendHandler
 * XTE_HANDLER_FIFORECV      XTemac_FifoRecvHandler
 * XTE_HANDLER_FIFODMAWRITE  XTemac_FifoDmaWriteHandler
 * XTE_HANDLER_FIFODMAREAD   XTemac_FifoDmaReadHandler
 * XTE_HANDLER_SGSEND        XTemac_SgHandler
 * XTE_HANDLER_SGRECV        XTemac_SgHandler
 * XTE_HANDLER_ERROR         XTemac_ErrorHandler
 *
 * HandlerType               Invoked by this driver when:
 * ------------------------  --------------------------------------------------
 * XTE_HANDLER_FIFOSEND      A packet transmitted by a call to
 *                           XTemac_FifoSend() has been sent successfully.
 * XTE_HANDLER_FIFORECV      When a packet has been received and is sitting in
 *                           the packet FIFO.
 * XTE_HANDLER_FIFODMAWRITE  A DMA transfer from a user buffer to the packet
 *                           FIFO has completed. See XTemac_FifoDmaWrite().
 * XTE_HANDLER_FIFODMAREAD   A DMA transfer from the packet FIFO to a use
 *                           buffer has completed. See XTemac_FifoDmaRead().
 * XTE_HANDLER_SGSEND        SG DMA has completed an operation on the transmit
 *                           side. Transmitted buffer descriptors require post
 *                           processing.
 * XTE_HANDLER_SGRECV        SG DMA has completed an operation on the receive
 *                           side. Buffer descriptors contain received packets.
 * XTE_HANDLER_ERROR         Any type of error has been detected.
 * </pre>
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 * @param HandlerType specifies which handler is to be attached.
 * @param CallbackFunc is the address of the callback function
 * @param CallbackRef is a user data item that will be passed to the callback
 *        when it is invoked.
 *
 * @return
 * - XST_SUCCESS when handler is installed.
 * - XST_INVALID_PARAM when HandlerType is invalid
 *
 * @note
 * Invoking this function for a handler that already has been installed replaces
 * it with the new handler.
 *
 ******************************************************************************/
XStatus XTemac_SetHandler(XTemac * InstancePtr, u32 HandlerType,
			  void *CallbackFunc, void *CallbackRef)
{
	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);
	XASSERT_NONVOID(CallbackFunc != NULL);

	switch (HandlerType) {
	case XTE_HANDLER_FIFOSEND:
		InstancePtr->FifoSendHandler =
		    (XTemac_FifoSendHandler) CallbackFunc;
		InstancePtr->FifoSendRef = CallbackRef;
		break;

	case XTE_HANDLER_FIFORECV:
		InstancePtr->FifoRecvHandler =
		    (XTemac_FifoRecvHandler) CallbackFunc;
		InstancePtr->FifoRecvRef = CallbackRef;
		break;

	case XTE_HANDLER_FIFODMAWRITE:
		InstancePtr->FifoDmaWriteHandler =
		    (XTemac_FifoDmaWriteHandler) CallbackFunc;
		InstancePtr->FifoDmaWriteRef = CallbackRef;
		break;

	case XTE_HANDLER_FIFODMAREAD:
		InstancePtr->FifoDmaReadHandler =
		    (XTemac_FifoDmaReadHandler) CallbackFunc;
		InstancePtr->FifoDmaReadRef = CallbackRef;
		break;

	case XTE_HANDLER_SGSEND:
		InstancePtr->SgSendHandler = (XTemac_SgHandler) CallbackFunc;
		InstancePtr->SgSendRef = CallbackRef;
		break;

	case XTE_HANDLER_SGRECV:
		InstancePtr->SgRecvHandler = (XTemac_SgHandler) CallbackFunc;
		InstancePtr->SgRecvRef = CallbackRef;
		break;

	case XTE_HANDLER_ERROR:
		InstancePtr->ErrorHandler = (XTemac_ErrorHandler) CallbackFunc;
		InstancePtr->ErrorRef = CallbackRef;
		break;

	default:
		return (XST_INVALID_PARAM);

	}
	return (XST_SUCCESS);
}
