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
* @file xdmav2.h
*
* The Xilinx Simple and Scatter Gather DMA driver.  This component supports a
* distributed DMA design in which each device can have it's own dedicated DMA
* channel, as opposed to a centralized DMA design. A device which uses DMA
* typically contains two DMA channels, one for sending data and the other for
* receiving data.
*
* This component is designed to be used as a basic building block for
* designing a device driver. It provides registers accesses such that all
* DMA processing can be maintained easier, but the device driver designer
* must still understand all the details of the DMA channel.
*
* For a full description of DMA features, please see the HW spec. This driver
* supports the following features:
*   - Simple DMA
*   - Scatter-Gather DMA (SGDMA)
*   - Interrupts
*   - Programmable interrupt coalescing for SGDMA
*   - APIs to manage Buffer Descriptors (BD) movement to and from the SGDMA
*     engine
*   - Virtual memory support
*
* <b>Transactions</b>
*
* To describe a DMA transaction in its simplest form, you need a source address,
* destination address, and the number of bytes to transfer. When using a DMA
* receive channel, the source address is within some piece of IP HW and doesn't
* require the user explicitly set it. Likewise with a transmit channel and the
* destination address. So this leaves a user buffer address and the number
* bytes to transfer as the primary transaction attributes. There are more
* obscure attributes such as:
*
*   - Is the user buffer a fixed address FIFO or a range of memory
*   - Which direction does the transfer occur.
*   - If SGDMA, does this transaction represent the end of a packet.
*
* The object used to describe a transaction is referred to as a Buffer
* Descriptor (BD). The format of a BD closely matches that of the DMA HW.
* Fields within the BD correspond directly with the same fields within the HW
* registers. See xdmabdv2.h for a detailed description of and the API for
* manipulation of these objects.
*
* <b>Simple DMA</b>
*
* Simple DMA is a single transaction type of operation. The user uses this
* driver to setup a transaction, initiate the transaction, then either wait for
* an interrupt or poll the HW for completion of the transaction. A new
* transaction may not be initiated until the current one completes.
*
* <b>Scatter-Gather DMA</b>
*
* SGDMA is more sophisticated in that it allows the user to define a list of
* transactions in memory which the HW will process without further user
* intervention. During this time, the user is free to continue adding more work
* to keep the HW busy.
*
* Notification of completed transactions can be done either by polling the HW,
* or using interrupts that signal a transaction has completed or a series of
* transactions have been processed.
*
* SGDMA processes in units of packets. A packet is defined as a series of
* data bytes that represent a message. SGDMA allows a packet of data to be
* broken up into one or more transactions. For example, take an Ethernet IP
* packet which consists of a 14 byte header followed by a 1 or more byte
* payload. With SGDMA, the user may point a BD to the header and another BD to
* the payload, then transfer them as a single message. This strategy can make a
* TCP/IP stack more efficient by allowing it to keep packet headers and data in
* different memory regions instead of assembling packets into contiguous blocks
* of memory.
*
* <b>Interrupt Coalescing</b>
*
* SGDMA provides control over the frequency of interrupts. On a high speed link
* significant processor overhead may be used servicing interrupts. Interrupt
* coalescing provides two mechanisms that help control interrupt frequency.
*
* The packet threshold will hold off interrupting the CPU until a programmable
* number of packets have been processed by the engine. The packet waitbound
* timer is used to interrupt the CPU if after a programmable amount of time
* after processing the last packet, no new packets were processed.
*
* <b>Interrupts</b>
*
* This driver does not service interrupts. This is done typically within
* a higher level driver that uses DMA. This driver does provide an API to
* enable or disable specific interrupts.
*
* <b>SGDMA List Management</b>
*
* The HW expectes BDs to be setup as a singly linked list. As BDs are completed,
* the DMA engine will dereference BD.Next and load the next BD to process.
* This driver uses a fixed buffer ring where all BDs are linked to the next
* adjacent BD in memory. The last BD in the ring is linked to the first.
*
* Within the BD ring, the driver maintains four groups of BDs. Each group
* consists of 0 or more adjacent BDs:
*
*   - Free group: Those BDs that can be allocated by the user with
*     XDmaV2_SgBdAlloc(). These BDs are under driver control.
*
*   - Pre-work group: Those BDs that have been allocated with
*     XDmaV2_SgBdAlloc(). These BDs are under user control. The user modifies
*     these BDs in preparation for future DMA transactions.
*
*   - Work group: Those BDs that have been enqueued to HW with
*     XDmaV2_SgBdToHw(). These BDs are under HW control and may be in a
*     state of awaiting HW processing, in process, or processed by HW.
*
*   - Post-work group: Those BDs that have been processed by HW and have been
*     extracted from the work group with XDmaV2_SgBdFromHw(). These BDs are under
*     user control. The user may access these BDs to determine the result
*     of DMA transactions. When the user is finished, XDmaV2_SgBdFree() should
*     be called to place them back into the Free group.
*
* It is considered an error for the user to change BDs while they are in the
* Work group. Doing so can cause data corruption and lead to system instability.
*
* The API provides macros that allow BD list traversal. These macros should be
* used with care as they do not understand where one group ends and another
* begins.
*
* The driver does not cache or keep copies of any BD. When the user modifies
* BDs returned by XDmaV2_SgBdAlloc() or XDmaV2_SgBdFromHw(), they are modifying
* the same BDs that HW sees.
*
* Certain pairs of list modification functions have usage restrictions. See
* the function headers for XDmaV2_SgBdAlloc() and XDmaV2_SgBdFromHw() for
* more information.
*
* <b>SGDMA List Creation</b>
*
* During initialization, the function XDmaV2_SgListCreate() is used to setup
* a user supplied memory block to contain all BDs for the DMA channel. This
* function takes as an argument the number of BDs to place in the list. To
* arrive at this number, the user is given two methods of calculating it.
*
* The first method assumes the user has a block of memory and they just
* want to fit as many BDs as possible into it. This is how version 1.00a of
* this driver was implemented. Here, the user must calculate the number of BDs
* that will fit with XDmaV2_mSgListCntCalc(), then supply that number into the
* list creation function.
*
* A second method allows the user to just supply the number directly. The
* driver assumes the memory block is large enough to contain them all. To
* double-check, the user should invoke XDmaV2_mSgListMemCalc() to verify the
* memory block is adequate.
*
* Once the list has been created, it can be used right away to perform DMA
* transactions. However, there are optional steps that can be done to increase
* throughput and decrease user code complexity by the use of XDmaV2_SgListClone().
*
* BDs have many user accessible attributes that affect how DMA transactions are
* carried out. Many of these attributes (such as a FIFO address) will probably
* be constant at run-time. The cloning function can be used to copy a template
* BD to every BD in the list relieving the user of having to setup transactions
* from scratch every time a BD is submitted to HW.
*
* Ideally, the only transaction parameters that need to be set at run-time
* should be: buffer address, bytes to transfer, and whether the BD is the
* "Last" BD of a packet.
*
* <b>Adding / Removing BDs from the SGDMA Engine</b>
*
* BDs may be enqueued (see XDmaV2_SgBdToHw()) to the engine any time after
* the SGDMA list is created. If the channel is running (see XDmaV2_SgStart()),
* then newly added BDs will be processed as soon as the engine reaches them.
* If the channel is stopped (see XDmaV2_SgStop()), the newly added BDs will
* be accepted but not processed by the engine until it is restarted.
*
* Processed BDs may be removed (see XDmaV2_SgBdFromHw()) at any time
* after the SGDMA list is created provided the engine has processed any.
*
* <b>Address Translation</b>
*
* When the BD list is setup with XDmaV2_SgListCreate(), all BDs in the list
* are linked to one another as described in "SGDMA List Management" using
* the physical address provided. After the initial setup, only the HW sees
* BD.Next fields. If address translation is used, the driver will reference
* BDs with the provided virtual address. If address translation is not used,
* then the provided physical and virtual addresses should be equivalent.
*
* Irregardless of the state of address translation, the user view BDs in the
* same way without any need to convert between virtual and physical address
* spaces.
*
* <b>Alignment</b>
*
* Except for 4 byte alignment of BDs there are no other alignment restrictions
* imposed by this driver. Individual DMA channels may, based on their
* capabilities or which bus they are a master of, have more stringent alignment
* requirements. It is up to the user to match the requirements of the DMA
* channel being used.
*
* Aside from the initial creation of BD list (see XDmaV2_SgListCreate()),
* there are no other run-time checks for proper alignment. Misaligned user
* buffers or BDs may result in corrupted data.
*
* <b>Cache Coherency</b>
*
* This driver expects all user buffers attached to BDs to be in cache coherent
* memory. Buffers for transmit should be flushed from the cache before passing
* the associated BD to this driver. Buffers for receive should be invalidated
* before being accessed.
*
* If the user wishes that the BD space itself be in cached memory, then
* modification of this driver is required. The driver helps the user in
* this area by: 1) Allowing the user to specify what alignment BDs should
* use (ie. aligned to the cache line size); 2) Identify areas in the code
* where cache flush or invalidate operations should be placed.
*
* <b>Reset After Stopping</b>
*
* This driver is designed to allow for stop-reset-start cycles of the DMA
* HW while keeping the BD list intact. When restarted after a reset, this
* driver will point the DMA engine to where it left off after stopping it.
*
* <b>Limitations</b>
*
* This driver requires exclusive use of the hardware DMACR.SGS bit. This
* applies to the actual HW register and BDs submitted through this driver to
* be processed. If a BD is encountered with this bit set, then it will be
* cleared within the driver.
*
* This driver does not have any mechanism for mutual exclusion. It is up to the
* user to provide this protection.
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date     Changes
* ----- ---- -------- -------------------------------------------------------
* 2.00a rmm  06/01/05 First release
* </pre>
*
******************************************************************************/

#ifndef XDMAV2_H		/* prevent circular inclusions */
#define XDMAV2_H		/* by using protection macros */

/***************************** Include Files *********************************/

#include "xdmabdv2.h"
#include "xstatus.h"

/************************** Constant Definitions *****************************/

/* Minimum alignment */
#define XDMABD_MINIMUM_ALIGNMENT  4

/**************************** Type Definitions *******************************/

/* This is an internal structure used to maintain the SGDMA list */
typedef struct {
	u32 PhysBaseAddr;
		       /**< Physical address of 1st BD in list */
	u32 BaseAddr;  /**< Virtual address of 1st BD in list */
	u32 TO;	       /**< Translation offset between virt and phys addr */
	u32 HighAddr;  /**< Virtual address of last BD in the list */
	u32 Length;    /**< Total size of ring in bytes */
	u32 RunState;  /**< Flag to indicate SGDMA is started */
	u32 Separation;/**< Number of bytes between the starting address
                                of adjacent BDs */
	XDmaBdV2 *FreeHead;/**< First BD in the free group */
	XDmaBdV2 *PreHead; /**< First BD in the pre-work group */
	XDmaBdV2 *HwHead;  /**< First BD in the work group */
	XDmaBdV2 *HwTail;  /**< Last BD in the work group */
	XDmaBdV2 *PostHead;/**< First BD in the post-work group */
	unsigned HwCnt;	   /**< Number of BDs in work group */
	unsigned PreCnt;   /**< Number of BDs in pre-work group */
	unsigned FreeCnt;  /**< Number of allocatable BDs in the free group */
	unsigned PostCnt;  /**< Number of BDs in post-work group */
	unsigned AllCnt;   /**< Total Number of BDs for channel */
} XDmaV2_BdRing;

/**
 * The XDmaV2 driver instance data. An instance must be allocated for each DMA
 * channel in use. If address translation is enabled, then all addresses and
 * pointers excluding PhysBase are expressed in terms of the virtual address.
 */
typedef struct XDmaV2 {
	u32 RegBase;	   /**< Base address of channel registers */
	u32 IsReady;	   /**< Flag to indicate device is ready to use */
	XDmaV2_BdRing BdRing;  /**< BD storage for SGDMA */
} XDmaV2;

/***************** Macros (Inline Functions) Definitions *********************/

/*****************************************************************************/
/**
* Use this macro at initialization time to determine how many BDs will fit
* in a BD list within the given memory constraints.
*
* The results of this macro can be provided to XDmaV2_SgListCreate().
*
* @param Alignment specifies what byte alignment the BDs must fall on and
*        must be a power of 2 to get an accurate calculation.
* @param Bytes is the number of bytes to be used to store BDs.
*
* @return Number of BDs that can fit in the given memory area
*
* @note
* C-style signature:
*    u32 XDmaV2_mSgListCntCalc(u32 Alignment, u32 Bytes)
*
******************************************************************************/
#define XDmaV2_mSgListCntCalc(Alignment, Bytes)                         \
    (u32)((Bytes) / ((sizeof(XDmaBdV2) + ((Alignment)-1)) & ~((Alignment)-1)))

/*****************************************************************************/
/**
* Use this macro at initialization time to determine how many bytes of memory
* is required to contain a given number of BDs at a given alignment.
*
* @param Alignment specifies what byte alignment the BDs must fall on. This
*        parameter must be a power of 2 to get an accurate calculation.
* @param NumBd is the number of BDs to calculate memory size requirements for
*
* @return The number of bytes of memory required to create a BD list with the
*         given memory constraints.
*
* @note
* C-style signature:
*    u32 XDmaV2_mSgListMemCalc(u32 Alignment, u32 NumBd)
*
******************************************************************************/
#define XDmaV2_mSgListMemCalc(Alignment, NumBd)                           \
    (u32)((sizeof(XDmaBdV2) + ((Alignment)-1)) & ~((Alignment)-1)) * (NumBd)

/****************************************************************************/
/**
* Return the total number of BDs allocated by this channel with
* XDmaV2_SgListCreate().
*
* @param  DmaPtr is the DMA channel to operate on.
*
* @return The total number of BDs allocated for this channel.
*
* @note
* C-style signature:
*    u32 XDmaV2_mSgGetCnt(XDmaV2 *DmaPtr)
*
*****************************************************************************/
#define XDmaV2_mSgGetCnt(InstancePtr)       ((InstancePtr)->BdRing.AllCnt)

/****************************************************************************/
/**
* Return the number of BDs allocatable with XDmaV2_SgAlloc() for pre-
* processing.
*
* @param  DmaPtr is the DMA channel to operate on.
*
* @return The number of BDs currently allocatable.
*
* @note
* C-style signature:
*    u32 XDmaV2_mSgGetFreeCnt(XDmaV2 *DmaPtr)
*
*****************************************************************************/
#define XDmaV2_mSgGetFreeCnt(InstancePtr)   ((InstancePtr)->BdRing.FreeCnt)

/****************************************************************************/
/**
* Return the next BD in a list.
*
* @param  DmaPtr is the DMA channel to operate on.
* @param  BdPtr is the BD to operate on.
*
* @return The next BD in the list relative to the BdPtr parameter.
*
* @note
* C-style signature:
*    XDmaBdV2 *XDmaV2_mSgBdNext(XDmaV2 *DmaPtr, XDmaBdV2 *BdPtr)
*
*****************************************************************************/
#define XDmaV2_mSgBdNext(InstancePtr, BdPtr)                            \
    (((u32)(BdPtr) >= (InstancePtr)->BdRing.HighAddr) ?             \
     (XDmaBdV2*)(InstancePtr)->BdRing.BaseAddr :                        \
     (XDmaBdV2*)((u32)(BdPtr) + (InstancePtr)->BdRing.Separation))

/****************************************************************************/
/**
* Return the previous BD in the list.
*
* @param  DmaPtr is the DMA channel to operate on.
* @param  BdPtr is the BD to operate on
*
* @return The previous BD in the list relative to the BdPtr parameter.
*
* @note
* C-style signature:
*    XDmaBdV2 *XDmaV2_mSgBdPrev(XDmaV2 *DmaPtr, XDmaBdV2 *BdPtr)
*
*****************************************************************************/
#define XDmaV2_mSgBdPrev(InstancePtr, BdPtr)                            \
    (((u32)(BdPtr) <= (InstancePtr)->BdRing.BaseAddr) ?             \
     (XDmaBdV2*)(InstancePtr)->BdRing.HighAddr :                        \
     (XDmaBdV2*)((u32)(BdPtr) - (InstancePtr)->BdRing.Separation))

/****************************************************************************/
/**
* Retrieve the current contents of the DMASR register. This macro can be
* used to poll the DMA HW for completion of a transaction.
*
* @param  DmaPtr is the DMA channel to operate on.
*
* @return The current contents of the DMASR register.
*
* @note
* C-style signature:
*    u32 XDmaV2_mGetStatus(XDmaV2 *DmaPtr)
*
*****************************************************************************/
#define XDmaV2_mGetStatus(InstancePtr)                         \
    XDmaV2_mReadReg((InstancePtr)->RegBase, XDMAV2_DMASR_OFFSET)

/************************** Function Prototypes ******************************/

/*
 * Initialization and control functions in xdma.c
 */
XStatus XDmaV2_Initialize(XDmaV2 * InstancePtr, u32 BaseAddress);
void XDmaV2_Reset(XDmaV2 * InstancePtr);

/*
 * Interrupt related functions in xdmav2_intr.c
 */
void XDmaV2_SetInterruptStatus(XDmaV2 * InstancePtr, u32 Mask);
u32 XDmaV2_GetInterruptStatus(XDmaV2 * InstancePtr);
void XDmaV2_SetInterruptEnable(XDmaV2 * InstancePtr, u32 Mask);
u32 XDmaV2_GetInterruptEnable(XDmaV2 * InstancePtr);

/*
 * Simple DMA related functions in xdmav2_simple.c
 */
XStatus XDmaV2_SimpleTransfer(XDmaV2 * InstancePtr, XDmaBdV2 * Bdptr);

/*
 * Scatter gather DMA related functions in xdmav2_sg.c
 */
XStatus XDmaV2_SgStart(XDmaV2 * InstancePtr);
void XDmaV2_SgStop(XDmaV2 * InstancePtr);
XStatus XDmaV2_SgSetPktThreshold(XDmaV2 * InstancePtr, u16 Threshold);
XStatus XDmaV2_SgSetPktWaitbound(XDmaV2 * InstancePtr, u16 TimerVal);
u16 XDmaV2_SgGetPktThreshold(XDmaV2 * InstancePtr);
u16 XDmaV2_SgGetPktWaitbound(XDmaV2 * InstancePtr);

XStatus XDmaV2_SgListCreate(XDmaV2 * InstancePtr, u32 PhysAddr, u32 VirtAddr,
			    u32 Alignment, unsigned BdCount);
XStatus XDmaV2_SgListClone(XDmaV2 * InstancePtr, XDmaBdV2 * SrcBdPtr);
XStatus XDmaV2_SgCheck(XDmaV2 * InstancePtr);
XStatus XDmaV2_SgBdAlloc(XDmaV2 * InstancePtr, unsigned NumBd,
			 XDmaBdV2 ** BdSetPtr);
XStatus XDmaV2_SgBdToHw(XDmaV2 * InstancePtr, unsigned NumBd,
			XDmaBdV2 * BdSetPtr);
XStatus XDmaV2_SgBdFree(XDmaV2 * InstancePtr, unsigned NumBd,
			XDmaBdV2 * BdSetPtr);
unsigned XDmaV2_SgBdFromHw(XDmaV2 * InstancePtr, unsigned NumBd,
			   XDmaBdV2 ** BdSetPtr);

/*
 * Selftest functions in xdmav2_selftest.c
 */
XStatus XDmaV2_SelfTest(XDmaV2 * InstancePtr);

#endif				/* end of protection macro */
