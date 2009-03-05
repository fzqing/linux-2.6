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
 * @file xdmav2_intr.c
 *
 * This file implements interrupt control related functions. For more
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

/************************** Constant Definitions *****************************/

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/

/************************** Function Prototypes ******************************/

/************************** Variable Definitions *****************************/

/*****************************************************************************/
/**
 * Set the interrupt status register for this channel. Use this function
 * to ack pending interrupts.
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 * @param Mask is a logical OR of XDMAV2_IPXR_*_MASK constants found in
 *        xdmav2_l.h.
 *
 ******************************************************************************/
void XDmaV2_SetInterruptStatus(XDmaV2 * InstancePtr, u32 Mask)
{
	XDmaV2_mWriteReg(InstancePtr->RegBase, XDMAV2_ISR_OFFSET, Mask);
}

/*****************************************************************************/
/**
 * Retrieve the interrupt status for this channel. OR the results of this
 * function with results from XDmaV2_GetInterruptEnable() to determine which
 * interrupts are currently pending to the processor.
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 *
 * @return Mask of interrupt bits made up of XDMAV2_IPXR_*_MASK constants found
 *         in xdmav2_l.h.
 *
 ******************************************************************************/
u32 XDmaV2_GetInterruptStatus(XDmaV2 * InstancePtr)
{
	return (XDmaV2_mReadReg(InstancePtr->RegBase, XDMAV2_ISR_OFFSET));
}

/*****************************************************************************/
/**
 * Enable specific DMA interrupts.
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 * @param Mask is a logical OR of of XDMAV2_IPXR_*_MASK constants found in
 *        xdmav2_l.h.
 *
 ******************************************************************************/
void XDmaV2_SetInterruptEnable(XDmaV2 * InstancePtr, u32 Mask)
{
	XDmaV2_mWriteReg(InstancePtr->RegBase, XDMAV2_IER_OFFSET, Mask);
}

/*****************************************************************************/
/**
 * Retrieve the interrupt enable register for this channel. Use this function to
 * determine which interrupts are currently enabled to the processor.
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 *
 * @return Mask of interrupt bits made up of XDMAV2_IPXR_*_MASK constants found
 *         int xdmav2_l.h.
 *
 ******************************************************************************/
u32 XDmaV2_GetInterruptEnable(XDmaV2 * InstancePtr)
{
	return (XDmaV2_mReadReg(InstancePtr->RegBase, XDMAV2_IER_OFFSET));
}
