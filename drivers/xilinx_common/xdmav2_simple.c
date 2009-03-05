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
* @file xdmav2_simple.c
*
* This file implements Simple DMA related functions. For more
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
 * Initiate a simple DMA transfer. The BD argument sets the parameters of the
 * transfer. Since the BD is also used for SG DMA transfers, some fields of the
 * BD will be ignored. The following BD macros will have no effect on the
 * transfer:
 *
 * - XDmaBdV2_mSetLast()
 * - XDmaBdV2_mClearLast()
 *
 * To determine when the transfer has completed, the user can poll the device
 * with XDmaV2_mGetStatus() and test the XDMAV2_DMASR_DMABSY_MASK bit, or wait
 * for an interrupt. When the DMA operation has completed, the outcome of the
 * transfer can be retrieved by calling XDmaV2_mGetStatus() and testing for DMA
 * bus errors bits.
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 * @param BdPtr sets the parameters of the transfer.
 *
 * @return
 * - XST_SUCCESS if the transfer was initated
 * - XST_DEVICE_BUSY if a transfer is already in progress
 *
 ******************************************************************************/
XStatus XDmaV2_SimpleTransfer(XDmaV2 * InstancePtr, XDmaBdV2 * BdPtr)
{
	u32 Dmasr;

	/* Is the channel busy */
	Dmasr = XDmaV2_mReadReg(InstancePtr->RegBase, XDMAV2_DMASR_OFFSET);
	if (Dmasr & (XDMAV2_DMASR_DMABSY_MASK | XDMAV2_DMASR_SGBSY_MASK)) {
		return (XST_DEVICE_BUSY);
	}

	/* Copy BdPtr fields into the appropriate HW registers */

	/* DMACR: SGS bit is set always. This is done in case the transfer
	 * occurs on a SGDMA channel and will prevent the HW from fetching the
	 * next BD.
	 */
	XDmaV2_mWriteReg(InstancePtr->RegBase, XDMAV2_DMACR_OFFSET,
			 XDmaV2_mReadBd(BdPtr, XDMAV2_BD_DMACR_OFFSET)
			 | XDMAV2_DMACR_SGS_MASK);

	/* SA */
	XDmaV2_mWriteReg(InstancePtr->RegBase, XDMAV2_SA_OFFSET,
			 XDmaV2_mReadBd(BdPtr, XDMAV2_BD_SA_OFFSET));

	/* DA */
	XDmaV2_mWriteReg(InstancePtr->RegBase, XDMAV2_DA_OFFSET,
			 XDmaV2_mReadBd(BdPtr, XDMAV2_BD_DA_OFFSET));

	/* LENGTH: Writing this register starts HW */
	XDmaV2_mWriteReg(InstancePtr->RegBase, XDMAV2_LENGTH_OFFSET,
			 XDmaV2_mReadBd(BdPtr, XDMAV2_BD_LENGTH_OFFSET));

	return (XST_SUCCESS);
}
