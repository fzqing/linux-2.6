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
 * @file xdmav2.c
 *
 * This file implements initialization and control related functions. For more
 * information on this driver, see xdmav2.h.
 *
 * <pre>
 * MODIFICATION HISTORY:
 *
 * Ver   Who  Date     Changes
 * ----- ---- -------- -------------------------------------------------------
 * 2.00a rmm  06/01/05 First release
 * </pre>
 ******************************************************************************/

/***************************** Include Files *********************************/

#include "xdmav2.h"
#include <asm/delay.h>

/************************** Constant Definitions *****************************/

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/

/************************** Function Prototypes ******************************/

/************************** Variable Definitions *****************************/

/*****************************************************************************/
/**
 * This function initializes a DMA channel.  This function must be called
 * prior to using a DMA channel. Initialization of a channel includes setting
 * up the register base address, setting up the instance data, and ensuring the
 * HW is in a quiescent state.
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 * @param BaseAddress is where the registers for this channel can be found.
 *        If address translation is being used, then this parameter must
 *        reflect the virtual base address.
 *
 * @return
 * - XST_SUCCESS
 *
 ******************************************************************************/
XStatus XDmaV2_Initialize(XDmaV2 * InstancePtr, u32 BaseAddress)
{
	u32 Dmasr;

	/* Setup the instance */
	memset(InstancePtr, 0, sizeof(XDmaV2));
	InstancePtr->RegBase = BaseAddress;
	InstancePtr->IsReady = XCOMPONENT_IS_READY;
	InstancePtr->BdRing.RunState = XST_DMA_SG_IS_STOPPED;

	/* If this is SGDMA channel, then make sure it is stopped */
	Dmasr = XDmaV2_mReadReg(InstancePtr->RegBase, XDMAV2_DMASR_OFFSET);
	Dmasr &= XDMAV2_MIR_CHAN_TYPE_MASK;
	if ((Dmasr == XDMAV2_MIR_CHAN_TYPE_SSGDMA) ||
	    (Dmasr == XDMAV2_MIR_CHAN_TYPE_SGDMATX) ||
	    (Dmasr == XDMAV2_MIR_CHAN_TYPE_SGDMARX)) {
		XDmaV2_SgStop(InstancePtr);
	}

	return (XST_SUCCESS);
}

/*****************************************************************************/
/**
 * Initiate a HW reset for the given DMA channel.
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 *
 ******************************************************************************/
void XDmaV2_Reset(XDmaV2 * InstancePtr)
{
	XDmaV2_mWriteReg(InstancePtr->RegBase, XDMAV2_RST_OFFSET,
			 XDMAV2_RST_MASK);
}
