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
 * @file xdmav2_sg.c
 *
 * This file implements Scatter-Gather DMA (SGDMA) related functions. For more
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

/****************************************************************************
 * These cache macros are used throughout this source code file to show
 * users where cache operations should occur if BDs were to be placed in
 * a cached memory region. Caching BD regions, however, is not common.
 *
 * The macros are implemented as NULL operations, but may be hooked into
 * XENV macros in future revisions of this driver.
 ****************************************************************************/
#define XDMAV2_CACHE_FLUSH(BdPtr)
#define XDMAV2_CACHE_INVALIDATE(BdPtr)

/****************************************************************************
 * Compute the physical address of a descriptor from its virtual address
 *
 * @param Ring is the ring BdPtr appears in
 * @param BdPtr is the virtual address of the BD
 *
 * @returns Physical address of BdPtr
 *
 * @note Assume BdPtr is always a valid BD in the ring
 ****************************************************************************/
#define XDMAV2_VIRT_TO_PHYS(Ring, BdPtr) ((u32)BdPtr - Ring->TO)

/****************************************************************************
 * Compute the virtual address of a descriptor from its physical address
 *
 * @param Ring is the ring BdPtr appears in
 * @param BdPtr is the virtual address of the BD
 *
 * @returns Physical address of BdPtr
 *
 * @note Assume BdPtr is always a valid BD in the ring
 ****************************************************************************/
#define XDMAV2_PHYS_TO_VIRT(Ring, BdPtr) ((u32)BdPtr + Ring->TO)

/****************************************************************************
 * Clear or set the SGS bit of the DMACR register
 ****************************************************************************/
#define XDMAV2_HW_SGS_CLEAR                                             \
    XDmaV2_mWriteReg(InstancePtr->RegBase, XDMAV2_DMACR_OFFSET,         \
                     XDmaV2_mReadReg(InstancePtr->RegBase, XDMAV2_DMACR_OFFSET) \
                     & ~XDMAV2_DMACR_SGS_MASK)

#define XDMAV2_HW_SGS_SET                                               \
    XDmaV2_mWriteReg(InstancePtr->RegBase, XDMAV2_DMACR_OFFSET,         \
                     XDmaV2_mReadReg(InstancePtr->RegBase, XDMAV2_DMACR_OFFSET) \
                     |XDMAV2_DMACR_SGS_MASK)

/****************************************************************************
 * Move the BdPtr argument ahead an arbitrary number of BDs wrapping around
 * to the beginning of the ring if needed.
 *
 * We know if a wrapaound should occur if the new BdPtr is greater than
 * the high address in the ring OR if the new BdPtr crosses over the
 * 0xFFFFFFFF to 0 boundary. The latter test is a valid one since we do not
 * allow a BD space to span this boundary.
 *
 * @param Ring is the ring BdPtr appears in
 * @param BdPtr on input is the starting BD position and on output is the
 *        final BD position
 * @param NumBd is the number of BD spaces to increment
 *
 ****************************************************************************/
#define XDMAV2_RING_SEEKAHEAD(Ring, BdPtr, NumBd)                       \
    {                                                                   \
        u32 Addr = (u32)BdPtr;                                  \
                                                                        \
        Addr += (Ring->Separation * NumBd);                             \
        if ((Addr > Ring->HighAddr) || ((u32)BdPtr > Addr))         \
        {                                                               \
            Addr -= Ring->Length;                                       \
        }                                                               \
                                                                        \
        BdPtr = (XDmaBdV2*)Addr;                                        \
    }

/************************** Function Prototypes ******************************/

static int IsSgDmaChannel(XDmaV2 * InstancePtr);

/************************** Variable Definitions *****************************/

/******************************************************************************/
/**
 * Start the SGDMA channel.
 *
 * @param InstancePtr is a pointer to the instance to be started.
 *
 * @return
 * - XST_SUCCESS if channel was started.
 * - XST_DMA_SG_NO_LIST if buffer descriptor space has not been assigned to
 *   the channel. See XDmaV2_SgListCreate().
 *
 ******************************************************************************/
XStatus XDmaV2_SgStart(XDmaV2 * InstancePtr)
{
	XDmaV2_BdRing *Ring = &InstancePtr->BdRing;
	u32 BdaV;
	int i;

	/* BD list has yet to be created for this channel */
	if (Ring->AllCnt == 0) {
		return (XST_DMA_SG_NO_LIST);
	}

	/* Do nothing if already started */
	if (Ring->RunState == XST_DMA_SG_IS_STARTED) {
		return (XST_SUCCESS);
	}

	/* Note as started */
	Ring->RunState = XST_DMA_SG_IS_STARTED;

	/* Sync HW.BDA with the driver and start the engine if unprocessed BDs
	 * are present. This process is quite complex since we have to assume
	 * that HW may have been reset since it was stopped. Additionally, we
	 * have to account for calls made to XDmaV2_SgBdToHw() and
	 * XDmaV2_SgBdFromHw() while stopped. Several cases are handled below.
	 *
	 * Wherever HW.BDA is set, that will be the 1st BD processed once
	 * the engine starts.
	 */

	/* Case 1: Virgin BD ring that hasn't changed state since it was
	 * created. No BDs have ever been enqueued.
	 */
	if (!(XDmaV2_mReadBd(Ring->HwTail, XDMAV2_BD_DMACR_OFFSET) &
	      XDMAV2_DMACR_SGS_MASK)) {
		XDmaV2_mWriteReg(InstancePtr->RegBase, XDMAV2_BDA_OFFSET,
				 Ring->PhysBaseAddr);
	}

	/* Case 2: There are no active BDs. In this case, HwHead has overtaken
	 * HwTail and points one BD past the last one HW has processed. This is
	 * the BD to set HW to start from. Any new BDs will be enqueued at this
	 * point.
	 */
	else if (Ring->HwCnt == 0) {
		XDmaV2_mWriteReg(InstancePtr->RegBase, XDMAV2_BDA_OFFSET,
				 XDMAV2_VIRT_TO_PHYS(Ring, Ring->HwHead));
	}

	/* Case 3: There are 1 or more BDs between HwHead and HwTail.
	 * To find the restart point, look for the first BD between HwHead and
	 * HwTail where the DMABSY bit is set. This will be the 1st unprocessed
	 * BD. Set HW here then tell the engine to begin processing straight away.
	 *
	 * If the end of the list is reached before a DMABSY set bit is found, then
	 * there are no BDs unprocessed by HW. In this case set HW to HwTail.BDA.
	 * Any new BDs will be enqueued at this point.
	 */
	else {
		BdaV = (u32) Ring->HwHead;

		for (i = 0; i < Ring->HwCnt; i++) {
			/* Found a BD with DMABSY set? */
			if (XDmaV2_mReadBd(BdaV, XDMAV2_BD_DMASR_OFFSET) &
			    XDMAV2_DMASR_DMABSY_MASK) {
				/* Yes, this is where to point HW */
				XDmaV2_mWriteReg(InstancePtr->RegBase,
						 XDMAV2_BDA_OFFSET,
						 XDMAV2_VIRT_TO_PHYS(Ring,
								     BdaV));

				/* Since this BD is unprocessed by HW, enable processing */
				XDMAV2_HW_SGS_CLEAR;
				break;
			}

			/* Onto next BD */
			BdaV = (u32) XDmaV2_mSgBdNext(InstancePtr, BdaV);
		}

		/* Made it through loop without finding a DMABSY? */
		if (i == Ring->HwCnt) {
			/* Point HW to the next BD location that will be read once
			 * new BDs are enqueued. This position is at HwTail.BDA which
			 * is where BdaV should be after completing the loop above.
			 */
			XDmaV2_mWriteReg(InstancePtr->RegBase,
					 XDMAV2_BDA_OFFSET,
					 XDMAV2_VIRT_TO_PHYS(Ring, BdaV));
		}
	}

	/* Enable the engine */
	XDmaV2_mWriteReg(InstancePtr->RegBase, XDMAV2_SWCR_OFFSET,
			 XDMAV2_SWCR_SGE_MASK);

	/* Note: If while the channel was XDmaV2_SgStop'd and new BDs were enqueued
	 * to HW, XDmaV2_SgBdToHw() will have cleared DMACR.SGS. Once we set
	 * SWCR.SGE, then processing will begin.
	 */
	return (XST_SUCCESS);
}

/******************************************************************************/
/**
 * Stop the SGDMA or Simple SGDMA channel gracefully. Any DMA operation
 * currently in progress is allowed to finish.
 *
 * An interrupt may be generated as the DMA engine finishes the packet in
 * process. To prevent this (if desired) then disabled DMA interrupts prior to
 * invoking this function.
 *
 * If after stopping the channel, new BDs are enqueued with XDmaV2_SgBdToHw(),
 * then those BDs will not be processed until after XDmaV2_SgStart() is called.
 *
 * @param InstancePtr is a pointer to the instance to be stopped.
 *
 * @note This function will block until the HW indicates that DMA has stopped.
 *
 ******************************************************************************/
void XDmaV2_SgStop(XDmaV2 * InstancePtr)
{
	volatile u32 Swcr;
	u32 Ier;

	/* Save the contents of the interrupt enable register then disable
	 * interrupts. This register will be restored at the end of the function
	 */
	Ier = XDmaV2_mReadReg(InstancePtr->RegBase, XDMAV2_IER_OFFSET);
	XDmaV2_mWriteReg(InstancePtr->RegBase, XDMAV2_IER_OFFSET, 0);

	/* Clear the SGE bit of the SWCR register and wait for SGE to clear */
	XDmaV2_mWriteReg(InstancePtr->RegBase, XDMAV2_SWCR_OFFSET, 0);

	/* Wait for SWCR.SGE = 0 */
	while (Swcr & XDMAV2_SWCR_SGE_MASK) {
		Swcr =
		    XDmaV2_mReadReg(InstancePtr->RegBase, XDMAV2_SWCR_OFFSET);
	}

	/* Note as stopped */
	InstancePtr->BdRing.RunState = XST_DMA_SG_IS_STOPPED;

	/* Restore interrupt enables. If an interrupt occurs due to this function
	 * stopping the channel then it will happen right here
	 */
	XDmaV2_mWriteReg(InstancePtr->RegBase, XDMAV2_IER_OFFSET, Ier);
}

/******************************************************************************/
/**
 * Set the packet threshold for this SGDMA channel. This has the effect of
 * delaying processor interrupts until the given number of packets (not BDs)
 * have been processed.
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 * @param Threshold is the packet threshold to set. If 0 is specified, then
 *        this feature is disabled. Maximum threshold is 2^10 - 1.
 *
 * @return
 * - XST_SUCCESS if threshold set properly.
 * - XST_NO_FEATURE if the provided instance is a non SGDMA type of DMA
 *   channel.
 ******************************************************************************/
XStatus XDmaV2_SgSetPktThreshold(XDmaV2 * InstancePtr, u16 Threshold)
{
	if (!IsSgDmaChannel(InstancePtr)) {
		return (XST_NO_FEATURE);
	}

	XDmaV2_mWriteReg(InstancePtr->RegBase, XDMAV2_PCT_OFFSET,
			 (u32) (Threshold & XDMAV2_PCT_MASK));

	return (XST_SUCCESS);
}

/******************************************************************************/
/**
 * Set the packet waitbound timer for this SGDMA channel. See xdmav2.h for more
 * information on interrupt coalescing and the effects of the waitbound timer.
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 * @param TimerVal is the waitbound period to set. If 0 is specified, then
 *        this feature is disabled. Maximum waitbound is 2^12 - 1. LSB is
 *        1 millisecond (approx).
 *
 * @return
 * - XST_SUCCESS if waitbound set properly.
 * - XST_NO_FEATURE if the provided instance is a non SGDMA type of DMA
 *   channel.
 ******************************************************************************/
XStatus XDmaV2_SgSetPktWaitbound(XDmaV2 * InstancePtr, u16 TimerVal)
{
	if (!IsSgDmaChannel(InstancePtr)) {
		return (XST_NO_FEATURE);
	}

	XDmaV2_mWriteReg(InstancePtr->RegBase, XDMAV2_PWB_OFFSET,
			 (u32) (TimerVal & XDMAV2_PWB_MASK));

	return (XST_SUCCESS);
}

/******************************************************************************/
/**
 * Get the packet threshold for this channel that was set with
 * XDmaV2_SgSetPktThreshold().
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 *
 * @return Current packet threshold as reported by HW. If the channel is not of
 *         SGDMA type then the return value is 0.
 ******************************************************************************/
u16 XDmaV2_SgGetPktThreshold(XDmaV2 * InstancePtr)
{
	u32 Reg;

	if (!IsSgDmaChannel(InstancePtr)) {
		return (0);
	}

	Reg = XDmaV2_mReadReg(InstancePtr->RegBase, XDMAV2_PCT_OFFSET);
	Reg &= XDMAV2_PCT_MASK;
	return ((u16) Reg);
}

/******************************************************************************/
/**
 * Get the waitbound timer for this channel that was set with
 * XDmaV2_SgSetPktWaitbound().
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 *
 * @return Current waitbound timer as reported by HW. If the channel is not of
 *         SGDMA type then the return value is 0.
 ******************************************************************************/
u16 XDmaV2_SgGetPktWaitbound(XDmaV2 * InstancePtr)
{
	u32 Reg;

	if (!IsSgDmaChannel(InstancePtr)) {
		return (0);
	}

	Reg = XDmaV2_mReadReg(InstancePtr->RegBase, XDMAV2_PWB_OFFSET);
	Reg &= XDMAV2_PWB_MASK;
	return ((u16) Reg);
}

/******************************************************************************/
/**
 * Using a memory segment allocated by the caller, create and setup the BD list
 * for the given SGDMA channel.
 *
 * @param InstancePtr is the instance to be worked on.
 * @param PhysAddr is the physical base address of user memory region.
 * @param VirtAddr is the virtual base address of the user memory region. If
 *        address translation is not being utilized, then VirtAddr should be
 *        equivalent to PhysAddr.
 * @param Alignment governs the byte alignment of individual BDs. This function
 *        will enforce a minimum alignment of 4 bytes with no maximum as long as
 *        it is specified as a power of 2.
 * @param BdCount is the number of BDs to setup in the user memory region. It is
 *        assumed the region is large enough to contain the BDs. Refer to the
 *        "SGDMA List Creation" section  in xdmav2.h for more information on list
 *        creation.
 *
 * @return
 *
 * - XST_SUCCESS if initialization was successful
 * - XST_NO_FEATURE if the provided instance is a non SGDMA type of DMA
 *   channel.
 * - XST_INVALID_PARAM under any of the following conditions: 1) PhysAddr and/or
 *   VirtAddr are not aligned to the given Alignment parameter; 2) Alignment
 *   parameter does not meet minimum requirements or is not a power of 2 value;
 *   3) BdCount is 0.
 * - XST_DMA_SG_LIST_ERROR if the memory segment containing the list spans
 *   over address 0x00000000 in virtual address space.
 *
 * @note
 *
 * Some DMA HW requires 8 or more byte alignments of BDs. Make sure the correct
 * value is passed into the Alignment parameter to meet individual DMA HW
 * requirements.
 *
 ******************************************************************************/
XStatus XDmaV2_SgListCreate(XDmaV2 * InstancePtr, u32 PhysAddr, u32 VirtAddr,
			    u32 Alignment, unsigned BdCount)
{
	int i;
	u32 BdV;
	u32 BdP;
	XDmaV2_BdRing *Ring = &InstancePtr->BdRing;
	u32 Upc;

	/* In case there is a failure prior to creating list, make sure the following
	 * attributes are 0 to prevent calls to other SG functions from doing anything
	 */
	Ring->AllCnt = 0;
	Ring->FreeCnt = 0;
	Ring->HwCnt = 0;
	Ring->PreCnt = 0;
	Ring->PostCnt = 0;

	/* Is this a SGDMA channel */
	if (!IsSgDmaChannel(InstancePtr)) {
		return (XST_NO_FEATURE);
	}

	/* Make sure Alignment parameter meets minimum requirements */
	if (Alignment < XDMABD_MINIMUM_ALIGNMENT) {
		return (XST_INVALID_PARAM);
	}

	/* Make sure Alignment is a power of 2 */
	if ((Alignment - 1) & Alignment) {
		return (XST_INVALID_PARAM);
	}

	/* Make sure PhysAddr and VirtAddr are on same Alignment */
	if ((PhysAddr % Alignment) || (VirtAddr % Alignment)) {
		return (XST_INVALID_PARAM);
	}

	/* Is BdCount is reasonable? */
	if (BdCount == 0) {
		return (XST_INVALID_PARAM);
	}

	/* Calculate the number of bytes between the start of adjacent BDs */
	Ring->Separation =
	    (sizeof(XDmaBdV2) + (Alignment - 1)) & ~(Alignment - 1);

	/* Must make sure the ring doesn't span across address 0x00000000.
	 * The design will fail if this occurs.
	 */
	if (VirtAddr > (VirtAddr + (Ring->Separation * BdCount))) {
		return (XST_DMA_SG_LIST_ERROR);
	}

	/* Initial ring setup:
	 *  - Clear the entire space
	 *  - Lay out the BDA fields the HW follows as it processes BDs
	 */
	memset((void *)VirtAddr, 0, (Ring->Separation * BdCount));

	BdV = VirtAddr;
	BdP = PhysAddr + Ring->Separation;
	for (i = 1; i < BdCount; i++) {
		XDmaV2_mWriteBd(BdV, XDMAV2_BD_BDA_OFFSET, BdP);
		BdV += Ring->Separation;
		BdP += Ring->Separation;
	}

	/* At the end of the ring, link the last BD back to the top */
	XDmaV2_mWriteBd(BdV, XDMAV2_BD_BDA_OFFSET, PhysAddr);

	/* Setup and initialize pointers and counters */
	InstancePtr->BdRing.RunState = XST_DMA_SG_IS_STOPPED;
	Ring->BaseAddr = VirtAddr;
	Ring->PhysBaseAddr = PhysAddr;
	Ring->TO = VirtAddr - PhysAddr;
	Ring->HighAddr = BdV;
	Ring->Length = Ring->HighAddr - Ring->BaseAddr + Ring->Separation;
	Ring->AllCnt = BdCount;
	Ring->FreeCnt = BdCount;
	Ring->FreeHead = (XDmaBdV2 *) VirtAddr;
	Ring->PreHead = (XDmaBdV2 *) VirtAddr;
	Ring->HwHead = (XDmaBdV2 *) VirtAddr;
	Ring->HwTail = (XDmaBdV2 *) VirtAddr;
	Ring->PostHead = (XDmaBdV2 *) VirtAddr;

	/* Sync HW with the beginning of the ring */

	XDmaV2_mWriteReg(InstancePtr->RegBase, XDMAV2_BDA_OFFSET, PhysAddr);
	/* Make sure the DMACR.SGS is 1 so that no DMA operations proceed until
	 * the start function is called.
	 */
	XDMAV2_HW_SGS_SET;

	/* As a final purging of any previous BD ring, make sure the UPC register
	 * has been cleared to 0.
	 */
	Upc = XDmaV2_mReadReg(InstancePtr->RegBase, XDMAV2_UPC_OFFSET);
	for (i = 0; i < Upc; i++) {
		XDmaV2_mWriteReg(InstancePtr->RegBase, XDMAV2_UPC_OFFSET, 1);
	}

	return (XST_SUCCESS);
}

/******************************************************************************/
/**
 * Clone the given BD into every BD in the list. Except for XDMAV2_BD_BDA_OFFSET,
 * every field of the source BD is replicated in every BD of the list.
 *
 * This function can be called only when all BDs are in the free group such as
 * they are immediately after initialization with XDmaV2_SgListCreate(). This
 * prevents modification of BDs while they are in use by HW or the user.
 *
 * @param InstancePtr is the instance to be worked on.
 * @param SrcBdPtr is the source BD to be cloned into the list.
 *
 * @return
 *   - XST_SUCCESS if the list was modified.
 *   - XST_DMA_SG_NO_LIST if a list has not been created.
 *   - XST_DMA_SG_LIST_ERROR if some of the BDs in this channel are under HW
 *     or user control.
 *   - XST_DEVICE_IS_STARTED if the DMA channel has not been stopped.
 *
 ******************************************************************************/
XStatus XDmaV2_SgListClone(XDmaV2 * InstancePtr, XDmaBdV2 * SrcBdPtr)
{
	u32 CurBd;
	u32 BdaSave;
	XDmaV2_BdRing *Ring = &InstancePtr->BdRing;

	/* Can't do this function if there isn't a ring */
	if (Ring->AllCnt == 0) {
		return (XST_DMA_SG_NO_LIST);
	}

	/* Can't do this function with the channel running */
	if (Ring->RunState == XST_DMA_SG_IS_STARTED) {
		return (XST_DEVICE_IS_STARTED);
	}

	/* Can't do this function with some of the BDs in use */
	if (Ring->FreeCnt != Ring->AllCnt) {
		return (XST_DMA_SG_LIST_ERROR);
	}

	/* Starting from the top, save BD.Next, overwrite the entire BD with the
	 * template, then restore BD.Next
	 */
	for (CurBd = Ring->BaseAddr; CurBd <= Ring->HighAddr;
	     CurBd += Ring->Separation) {
		BdaSave = XDmaV2_mReadBd(CurBd, XDMAV2_BD_BDA_OFFSET);
		memcpy((void *)CurBd, SrcBdPtr, sizeof(XDmaBdV2));
		XDmaV2_mWriteBd(CurBd, XDMAV2_BD_BDA_OFFSET, BdaSave);
	}

	return (XST_SUCCESS);
}

/******************************************************************************/
/**
 * Reserve locations in the BD list. The set of returned BDs may be modified in
 * preparation for future DMA transaction(s). Once the BDs are ready to be
 * submitted to HW, the user must call XDmaV2_SgBdToHw() in the same order which
 * they were allocated here. Example:
 *
 * <pre>
 *        NumBd = 2;
 *        Status = XDmaV2_SgBdAlloc(MyDmaInstPtr, NumBd, &MyBdSet);
 *
 *        if (Status != XST_SUCCESS)
 *        {
 *            // All BDs are in use
 *        }
 *
 *        CurBd = MyBdSet;
 *        for (i=0; i<NumBd; i++)
 *        {
 *            // Prepare CurBd.....
 *
 *            // Onto next BD
 *            CurBd = XDmaV2_mSgBdNext(MyDmaInstPtr, CurBd);
 *        }
 *
 *        // Give list to HW
 *        Status = XDmaV2_SgBdToHw(MyDmaInstPtr, NumBd, MyBdSet);
 * </pre>
 *
 * A more advanced use of this function may allocate multiple sets of BDs.
 * They must be allocated and given to HW in the correct sequence:
 * <pre>
 *        // Legal
 *        XDmaV2_SgBdAlloc(MyDmaInstPtr, NumBd1, &MySet1);
 *        XDmaV2_SgBdToHw(MyDmaInstPtr, NumBd1, MySet1);
 *
 *        // Legal
 *        XDmaV2_SgBdAlloc(MyDmaInstPtr, NumBd1, &MySet1);
 *        XDmaV2_SgBdAlloc(MyDmaInstPtr, NumBd2, &MySet2);
 *        XDmaV2_SgBdToHw(MyDmaInstPtr, NumBd1, MySet1);
 *        XDmaV2_SgBdToHw(MyDmaInstPtr, NumBd2, MySet2);
 *
 *        // Not legal
 *        XDmaV2_SgBdAlloc(MyDmaInstPtr, NumBd1, &MySet1);
 *        XDmaV2_SgBdAlloc(MyDmaInstPtr, NumBd2, &MySet2);
 *        XDmaV2_SgBdToHw(MyDmaInstPtr, NumBd2, MySet2);
 *        XDmaV2_SgBdToHw(MyDmaInstPtr, NumBd1, MySet1);
 * </pre>
 *
 * Use the API defined in xdmabd.h to modify individual BDs. Traversal of the
 * BD set can be done using XDmaV2_mSgBdNext() and XDmaV2_mSgBdPrev().
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 * @param NumBd is the number of BDs to allocate
 * @param BdSetPtr is an output parameter, it points to the first BD available
 *        for modification.
 *
 * @return
 *   - XST_SUCCESS if the requested number of BDs was returned in the BdSetPtr
 *     parameter.
 *   - XST_FAILURE if there were not enough free BDs to satisfy the request.
 *
 * @note This function should not be preempted by another XDmaV2 function call
 *       that modifies the BD space. It is the caller's responsibility to
 *       provide a mutual exclusion mechanism.
 *
 * @note Do not modify more BDs than the number requested with the NumBd
 *       parameter. Doing so will lead to data corruption and system
 *       instability.
 *
 ******************************************************************************/
XStatus XDmaV2_SgBdAlloc(XDmaV2 * InstancePtr, unsigned NumBd,
			 XDmaBdV2 ** BdSetPtr)
{
	XDmaV2_BdRing *Ring = &InstancePtr->BdRing;

	/* Enough free BDs available for the request? */
	if (Ring->FreeCnt < NumBd) {
		*BdSetPtr = NULL;
		return (XST_FAILURE);
	}

	/* Set the return argument and move FreeHead forward */
	*BdSetPtr = Ring->FreeHead;
	XDMAV2_RING_SEEKAHEAD(Ring, Ring->FreeHead, NumBd);
	Ring->FreeCnt -= NumBd;
	Ring->PreCnt += NumBd;
	return (XST_SUCCESS);
}

/******************************************************************************/
/**
 * Enqueue a set of BDs to HW that were previously allocated by
 * XDmaV2_SgBdAlloc(). Once this function returns, the argument BD set goes
 * under HW control. Any changes made to these BDs after this point will corrupt
 * the BD list leading to data corruption and system instability.
 *
 * The set will be rejected if the last BD of the set does not mark the end of
 * a packet (see XDmaBdV2_mSetLast()).
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 * @param NumBd is the number of BDs in the set.
 * @param BdSetPtr is the first BD of the set to commit to HW.
 *
 * @return
 *   - XST_SUCCESS if the set of BDs was accepted and enqueued to HW.
 *   - XST_FAILURE if the set of BDs was rejected because the last BD of the set
 *     did not have its "last" bit set.
 *   - XST_DMA_SG_LIST_ERROR if this function was called out of sequence with
 *     XDmaV2_SgBdAlloc().
 *
 * @note This function should not be preempted by another XDmaV2 function call
 *       that modifies the BD space. It is the caller's responsibility to
 *       provide a mutual exclusion mechanism.
 *
 ******************************************************************************/
XStatus XDmaV2_SgBdToHw(XDmaV2 * InstancePtr, unsigned NumBd,
			XDmaBdV2 * BdSetPtr)
{
	XDmaV2_BdRing *Ring = &InstancePtr->BdRing;
	XDmaBdV2 *LastBdPtr;
	int i;
	u32 Dmacr;
	u32 Swcr;

	/* Make sure we are in sync with XDmaV2_SgBdAlloc() */
	if ((Ring->PreCnt < NumBd) || (Ring->PreHead != BdSetPtr)) {
		return (XST_DMA_SG_LIST_ERROR);
	}

	/* For all BDs in this set (except the last one)
	 *   - Clear DMASR except for DMASR.DMABSY
	 *   - Clear DMACR.SGS
	 *
	 * For the last BD in this set
	 *   - Clear DMASR except for DMASR.DMABSY
	 *   - Set DMACR.SGS
	 */
	LastBdPtr = BdSetPtr;
	for (i = 1; i < NumBd; i++) {
		XDmaV2_mWriteBd(LastBdPtr, XDMAV2_BD_DMASR_OFFSET,
				XDMAV2_DMASR_DMABSY_MASK);

		Dmacr = XDmaV2_mReadBd(LastBdPtr, XDMAV2_BD_DMACR_OFFSET);
		XDmaV2_mWriteBd(LastBdPtr, XDMAV2_BD_DMACR_OFFSET,	/* DMACR.SGS = 0 */
				Dmacr & ~XDMAV2_DMACR_SGS_MASK);
		XDMAV2_CACHE_FLUSH(LastBdPtr);

		LastBdPtr = XDmaV2_mSgBdNext(InstancePtr, LastBdPtr);
	}

	/* Last BD */
	XDmaV2_mWriteBd(LastBdPtr, XDMAV2_BD_DMASR_OFFSET,
			XDMAV2_DMASR_DMABSY_MASK);

	Dmacr = XDmaV2_mReadBd(LastBdPtr, XDMAV2_BD_DMACR_OFFSET);
	XDmaV2_mWriteBd(LastBdPtr, XDMAV2_BD_DMACR_OFFSET,	/* DMACR.SGS = 1 */
			Dmacr | XDMAV2_DMACR_SGS_MASK);
	XDMAV2_CACHE_FLUSH(LastBdPtr);

	/* The last BD should have DMACR.LAST set */
	if (!(Dmacr & XDMAV2_DMACR_L_MASK)) {
		return (XST_FAILURE);
	}

	/* This set has completed pre-processing, adjust ring pointers & counters */
	XDMAV2_RING_SEEKAHEAD(Ring, Ring->PreHead, NumBd);
	Ring->PreCnt -= NumBd;

	/* This set is now ready to be added to the work group.
	 *
	 * Case 1: If there are no BDs in the list, then we know HW is idle, simply
	 * reset the list to begin and end on the current BD set
	 */
	if (Ring->HwCnt == 0) {
		/* Update pointers and counters. XDMAV2_RING_SEEKAHEAD could be used to
		 * advance HwTail, but it will always evaluate to LastBdPtr
		 */
		Ring->HwTail = LastBdPtr;
		Ring->HwCnt += NumBd;

		/* HW DMACR.SGS = 0 */
		XDMAV2_HW_SGS_CLEAR;
	}

	/* Case 2: There are BDs in the work group so we extend it in such a way
	 * that the channel doesn't need to be stopped
	 */
	else {
		/* Extend the work list: HwTail->DMACR.SGS = 0 */
		Dmacr = XDmaV2_mReadBd(Ring->HwTail, XDMAV2_BD_DMACR_OFFSET);
		XDmaV2_mWriteBd(Ring->HwTail, XDMAV2_BD_DMACR_OFFSET,
				Dmacr & ~XDMAV2_DMACR_SGS_MASK);
		XDMAV2_CACHE_FLUSH(Ring->HwTail);

		/* Update pointers and counters. HwTail now points to a new BD. */
		Ring->HwTail = LastBdPtr;
		Ring->HwCnt += NumBd;

		/* HW DMACR.SGS = 0 (with conditions)
		 *
		 * Must be careful here, the HW SGS may be set because it encountered
		 * the end of the work list before we extended it. If that is the
		 * situation, then clear it as we did in Case 1 so HW will see the
		 * BDs just added.
		 *
		 * The HW SGS bit may also be set because it has finished processing
		 * the set of BDs we just added! If that is the case then do nothing.
		 *
		 * If the HW SGS is clear, then HW is actively processing BDs so it will
		 * continue on getting to the list we just added.
		 */
		Dmacr =
		    XDmaV2_mReadReg(InstancePtr->RegBase, XDMAV2_DMACR_OFFSET);
		if (Dmacr & XDMAV2_DMACR_SGS_MASK) {
			/* Did HW just finish processing what we added? This is checked by
			 * comparing the BDA register with HwTail. If they are the same
			 * then the channel has loaded the HwTail BD so we shouldn't enable
			 * the engine by setting SGS = 0.
			 */
			XDMAV2_CACHE_INVALIDATE(Ring->HwTail);
			if (XDMAV2_VIRT_TO_PHYS(Ring, Ring->HwTail) !=
			    XDmaV2_mReadReg(InstancePtr->RegBase,
					    XDMAV2_BDA_OFFSET)) {
				/* No, SGS = 0 */
				XDmaV2_mWriteReg(InstancePtr->RegBase,
						 XDMAV2_DMACR_OFFSET,
						 Dmacr &
						 ~XDMAV2_DMACR_SGS_MASK);
			}
		}
	}

	/* If the channel was in a running state, then keep it that way. It may have
	 * stopped because DMACR.SGS got set for any reason.
	 */
	if (Ring->RunState == XST_DMA_SG_IS_STARTED) {
		/* If SWCR.SGE was 0, then set it to 1 */
		Swcr =
		    XDmaV2_mReadReg(InstancePtr->RegBase, XDMAV2_SWCR_OFFSET);
		if (!(Swcr & XDMAV2_SWCR_SGE_MASK)) {
			XDmaV2_mWriteReg(InstancePtr->RegBase,
					 XDMAV2_SWCR_OFFSET,
					 Swcr | XDMAV2_SWCR_SGE_MASK);
		}
	}

	return (XST_SUCCESS);
}

/******************************************************************************/
/**
 * Returns a set of BD(s) that have been processed by HW. The returned BDs may
 * be examined to determine the outcome of the DMA transaction(s). Once the BDs
 * have been examined, the user must call XDmaV2_SgBdFree() in the same order
 * which they were retrieved here. Example:
 *
 * <pre>
 *        NumBd = 0xFFFFFFFF;   // Ensure we get all that are ready
 *
 *        Status = XDmaV2_SgBdFromHw(MyDmaInstPtr, &NumBd, &MyBdSet);
 *
 *        if (Status != XST_SUCCESS)
 *        {
 *           // HW has nothing ready for us yet
 *        }
 *
 *        CurBd = MyBdSet;
 *        for (i=0; i<NumBd; i++)
 *        {
 *           // Examine CurBd for post processing.....
 *
 *           // Onto next BD
 *           CurBd = XDmaV2_mSgBdNext(MyDmaInstPtr, CurBd);
 *           }
 *
 *           XDmaV2_SgBdFree(MyDmaInstPtr, NumBd, MyBdSet); // Return the list
 *        }
 * </pre>
 *
 * A more advanced use of this function may allocate multiple sets of BDs.
 * They must be retrieved from HW and freed in the correct sequence:
 * <pre>
 *        // Legal
 *        XDmaV2_SgBdFromHw(MyDmaInstPtr, NumBd1, &MySet1);
 *        XDmaV2_SgBdFree(MyDmaInstPtr, NumBd1, MySet1);
 *
 *        // Legal
 *        XDmaV2_SgBdFromHw(MyDmaInstPtr, NumBd1, &MySet1);
 *        XDmaV2_SgBdFromHw(MyDmaInstPtr, NumBd2, &MySet2);
 *        XDmaV2_SgBdFree(MyDmaInstPtr, NumBd1, MySet1);
 *        XDmaV2_SgBdFree(MyDmaInstPtr, NumBd2, MySet2);
 *
 *        // Not legal
 *        XDmaV2_SgBdFromHw(MyDmaInstPtr, NumBd1, &MySet1);
 *        XDmaV2_SgBdFromHw(MyDmaInstPtr, NumBd2, &MySet2);
 *        XDmaV2_SgBdFree(MyDmaInstPtr, NumBd2, MySet2);
 *        XDmaV2_SgBdFree(MyDmaInstPtr, NumBd1, MySet1);
 * </pre>
 *
 * If HW has only partially completed a packet spanning multiple BDs, then none
 * of the BDs for that packet will be included in the results.
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 * @param NumBd is the maximum number of BDs to return in the set.
 * @param BdSetPtr is an output parameter, it points to the first BD available
 *        for examination.
 *
 * @return
 *   The number of BDs processed by HW. A value of 0 indicates that no data
 *   is available.
 *
 * @note Treat BDs returned by this function as read-only.
 *
 * @note This function should not be preempted by another XDmaV2 function call
 *       that modifies the BD space. It is the caller's responsibility to
 *       provide a mutual exclusion mechanism.
 *
 ******************************************************************************/
unsigned XDmaV2_SgBdFromHw(XDmaV2 * InstancePtr, unsigned NumBd,
			   XDmaBdV2 ** BdSetPtr)
{
	XDmaV2_BdRing *Ring = &InstancePtr->BdRing;
	XDmaBdV2 *CurBd;
	unsigned BdCount;
	unsigned PktCount;
	unsigned BdPartialCount;
	unsigned BdLimit = NumBd;
	u32 Dmasr;
	u32 Upc;

	CurBd = Ring->HwHead;
	PktCount = 0;
	BdCount = 0;
	BdPartialCount = 0;

	/* If no BDs in work group, then there's nothing to search */
	if (Ring->HwCnt == 0) {
		*BdSetPtr = NULL;
		return (0);
	}

	/* Get the number of packets HW is reporting completed */
	Upc = XDmaV2_mReadReg(InstancePtr->RegBase, XDMAV2_UPC_OFFSET);

	/* Starting at HwHead, keep moving forward in the list until:
	 *  - A BD is encountered with its DMASR.DMABSY bit set which means HW has
	 *    not completed processing of that BD.
	 *  - We've processed the number of packets HW has reported completed
	 *  - Ring->HwTail is reached
	 *  - The number of requested BDs has been processed
	 */
	while (BdCount < BdLimit) {
		/* Read the status */
		XDMAV2_CACHE_INVALIDATE(CurBd);
		Dmasr = XDmaV2_mReadBd(CurBd, XDMAV2_BD_DMASR_OFFSET);

		/* If the HW still hasn't processed this BD then we are done */
		if (Dmasr & XDMAV2_DMASR_DMABSY_MASK) {
			break;
		}

		BdCount++;

		/* HW has processed this BD so check the "last" bit. If it is clear,
		 * then there are more BDs for the current packet. Keep a count of
		 * these partial packet BDs.
		 */
		if (Dmasr & XDMAV2_DMASR_L_MASK) {
			/* Tell HW a packet has been serviced */
			XDmaV2_mWriteReg(InstancePtr->RegBase,
					 XDMAV2_UPC_OFFSET, 1);
			BdPartialCount = 0;

			/* We are done if we have serviced the number of packets HW has
			 * reported completed.
			 */
			if (++PktCount == Upc) {
				break;
			}
		} else {
			BdPartialCount++;
		}

		/* Reached the end of the work group */
		if (CurBd == Ring->HwTail) {
			break;
		}

		/* Move on to next BD in work group */
		CurBd = XDmaV2_mSgBdNext(InstancePtr, CurBd);
	}

	/* Subtract off any partial packet BDs found */
	BdCount -= BdPartialCount;

	/* If BdCount is non-zero then BDs were found to return. Set return
	 * parameters, update pointers and counters, return success
	 */
	if (BdCount) {
		*BdSetPtr = Ring->HwHead;
		Ring->HwCnt -= BdCount;
		Ring->PostCnt += BdCount;
		XDMAV2_RING_SEEKAHEAD(Ring, Ring->HwHead, BdCount);
		return (BdCount);
	} else {
		*BdSetPtr = NULL;
		return (0);
	}
}

/******************************************************************************/
/**
 * Frees a set of BDs that had been previously retrieved with XDmaV2_SgBdFromHw().
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 * @param NumBd is the number of BDs to free.
 * @param BdSetPtr is the head of a list of BDs returned by XDmaV2_SgBdFromHw().
 *
 * @return
 *   - XST_SUCCESS if the set of BDs was freed.
 *   - XST_DMA_SG_LIST_ERROR if this function was called out of sequence with
 *     XDmaV2_SgBdFromHw().
 *
 * @note This function should not be preempted by another XDmaV2 function call
 *       that modifies the BD space. It is the caller's responsibility to
 *       provide a mutual exclusion mechanism.
 *
 ******************************************************************************/
XStatus XDmaV2_SgBdFree(XDmaV2 * InstancePtr, unsigned NumBd,
			XDmaBdV2 * BdSetPtr)
{
	XDmaV2_BdRing *Ring = &InstancePtr->BdRing;

	/* Make sure we are in sync with XDmaV2_SgBdFromHw() */
	if ((Ring->PostCnt < NumBd) || (Ring->PostHead != BdSetPtr)) {
		return (XST_DMA_SG_LIST_ERROR);
	}

	/* Update pointers and counters */
	Ring->FreeCnt += NumBd;
	Ring->PostCnt -= NumBd;
	XDMAV2_RING_SEEKAHEAD(Ring, Ring->PostHead, NumBd);
	return (XST_SUCCESS);
}

/******************************************************************************/
/**
 * Check the internal data structures of the BD list for the provided channel.
 * The following checks are made:
 *
 *   - Is the BD list linked correctly in physical address space.
 *   - Do the internal pointers point to BDs in the list.
 *   - Do the internal counters add up.
 *
 * The channel should be stopped prior to calling this function.
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 *
 * @return
 *   - XST_SUCCESS if the set of BDs was freed.
 *   - XST_DMA_SG_NO_LIST if the list has not been created.
 *   - XST_IS_STARTED if the channel is not stopped.
 *   - XST_DMA_SG_LIST_ERROR if a problem is found with the internal data
 *     structures. If this value is returned, the channel should be reset and
 *     the list recreated to avoid data corruption or system instability.
 *
 * @note This function should not be preempted by another XDmaV2 function call
 *       that modifies the BD space. It is the caller's responsibility to
 *       provide a mutual exclusion mechanism.
 *
 ******************************************************************************/
XStatus XDmaV2_SgCheck(XDmaV2 * InstancePtr)
{
	XDmaV2_BdRing *RingPtr = &InstancePtr->BdRing;
	u32 AddrV, AddrP;
	int i;

	/* Is the list created */
	if (RingPtr->AllCnt == 0) {
		return (XST_DMA_SG_NO_LIST);
	}

	/* Can't check if channel is running */
	if (RingPtr->RunState == XST_DMA_SG_IS_STARTED) {
		return (XST_IS_STARTED);
	}

	/* RunState doesn't make sense */
	else if (RingPtr->RunState != XST_DMA_SG_IS_STOPPED) {
		return (XST_DMA_SG_LIST_ERROR);
	}

	/* Verify internal pointers point to correct memory space */
	AddrV = (u32) RingPtr->FreeHead;
	if ((AddrV < RingPtr->BaseAddr) || (AddrV > RingPtr->HighAddr)) {
		return (XST_DMA_SG_LIST_ERROR);
	}

	AddrV = (u32) RingPtr->PreHead;
	if ((AddrV < RingPtr->BaseAddr) || (AddrV > RingPtr->HighAddr)) {
		return (XST_DMA_SG_LIST_ERROR);
	}

	AddrV = (u32) RingPtr->HwHead;
	if ((AddrV < RingPtr->BaseAddr) || (AddrV > RingPtr->HighAddr)) {
		return (XST_DMA_SG_LIST_ERROR);
	}

	AddrV = (u32) RingPtr->HwTail;
	if ((AddrV < RingPtr->BaseAddr) || (AddrV > RingPtr->HighAddr)) {
		return (XST_DMA_SG_LIST_ERROR);
	}

	AddrV = (u32) RingPtr->PostHead;
	if ((AddrV < RingPtr->BaseAddr) || (AddrV > RingPtr->HighAddr)) {
		return (XST_DMA_SG_LIST_ERROR);
	}

	/* Verify internal counters add up */
	if ((RingPtr->HwCnt + RingPtr->PreCnt + RingPtr->FreeCnt +
	     RingPtr->PostCnt) != RingPtr->AllCnt) {
		return (XST_DMA_SG_LIST_ERROR);
	}

	/* Verify BDs are linked correctly */
	AddrV = RingPtr->BaseAddr;
	AddrP = RingPtr->PhysBaseAddr + RingPtr->Separation;
	for (i = 1; i < RingPtr->AllCnt; i++) {
		/* Check BDA for this BD. It should point to next physical addr */
		if (XDmaV2_mReadBd(AddrV, XDMAV2_BD_BDA_OFFSET) != AddrP) {
			return (XST_DMA_SG_LIST_ERROR);
		}

		/* Move on to next BD */
		AddrV += RingPtr->Separation;
		AddrP += RingPtr->Separation;
	}

	/* Last BD should point back to the beginning of ring */
	if (XDmaV2_mReadBd(AddrV, XDMAV2_BD_BDA_OFFSET) !=
	    RingPtr->PhysBaseAddr) {
		return (XST_DMA_SG_LIST_ERROR);
	}

	/* No problems found */
	return (XST_SUCCESS);
}

/******************************************************************************
 * Verify given channel is of the SGDMA variety.
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 *
 * @return
 *   - 1 if channel is of type SGDMA
 *   - 0 if channel is not of type SGDMA
 ******************************************************************************/
static int IsSgDmaChannel(XDmaV2 * InstancePtr)
{
	u32 Reg;

	Reg = XDmaV2_mReadReg(InstancePtr->RegBase, XDMAV2_MIR_OFFSET);
	Reg &= XDMAV2_MIR_CHAN_TYPE_MASK;
	if ((Reg != XDMAV2_MIR_CHAN_TYPE_SSGDMA) &&
	    (Reg != XDMAV2_MIR_CHAN_TYPE_SGDMATX) &&
	    (Reg != XDMAV2_MIR_CHAN_TYPE_SGDMARX)) {
		return (0);
	} else {
		return (1);
	}
}
