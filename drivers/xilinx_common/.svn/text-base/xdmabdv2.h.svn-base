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
 * @file xdmabdv2.h
 *
 * This header provides operations to manage buffer descriptors in support
 * of simple and scatter-gather DMA (see xdmav2.h).
 *
 * The API exported by this header define abstracted macros that allow the
 * user to read/write specific BD fields.
 *
 * <b>Buffer Descriptors</b>
 *
 * A buffer descriptor (BD) defines a DMA transaction (see "Transaction"
 * section in xdmav2.h). The macros defined by this header file allow access
 * to most fields within a BD to tailor a DMA transaction according to user
 * and HW requirements.  See the HW IP DMA spec for more information on BD
 * fields and how they affect transfers.
 *
 * The XDmaBdV2 structure defines a BD. The organization of this structure is
 * driven mainly by the hardware for use in scatter-gather DMA transfers.
 *
 * <b>Accessor Macros</b>
 *
 * Most of the BD attributes can be accessed through macro functions defined
 * here in this API.
 *
 * <b>Performance</b>
 *
 * Most of the time BDs are in a non-cached memory space. Limiting I/O to BDs
 * with the accessor macros can improve overall performance of the DMA channel.
 *
 * <pre>
 * MODIFICATION HISTORY:
 *
 * Ver   Who  Date     Changes
 * ----- ---- -------- -------------------------------------------------------
 * 2.00a rmm  06/01/05 First release
 * </pre>
 *
 * ***************************************************************************
 */

#ifndef XDMABD_H		/* prevent circular inclusions */
#define XDMABD_H		/* by using protection macros */

/***************************** Include Files *********************************/

#include "xbasic_types.h"
#include "xdmav2_l.h"
#include <asm/delay.h>

/************************** Constant Definitions *****************************/

/**************************** Type Definitions *******************************/

/**
 * The XDmaBdV2 is the type for buffer descriptors (BDs).
 */
typedef u32 XDmaBdV2[XDMAV2_BD_NUM_WORDS];

/***************** Macros (Inline Functions) Definitions *********************/

/*****************************************************************************/
/**
 * Zero out BD fields
 *
 * @param  BdPtr is the BD to operate on
 *
 * @return Nothing
 *
 * @note
 * C-style signature:
 *    void XDmaBdV2_mClear(XDmaBdV2* BdPtr)
 *
 ******************************************************************************/
#define XDmaBdV2_mClear(BdPtr)                  \
    memset((BdPtr), 0, sizeof(XDmaBdV2))

/*****************************************************************************/
/**
 * Retrieve the BD's Packet DMA transfer status word.
 *
 * @param  BdPtr is the BD to operate on
 *
 * @return Word at offset XDMAV2_BD_SR_OFFSET
 *
 * @note
 * C-style signature:
 *    u32 XDmaBdV2_mGetStatus(XDmaBdV2* BdPtr)
 *
 ******************************************************************************/
#define XDmaBdV2_mGetStatus(BdPtr)                      \
    XDmaV2_mReadBd((BdPtr), XDMAV2_BD_DMASR_OFFSET)

/*****************************************************************************/
/**
 * Retrieve the BD's Packet device status word. This differs from
 * XDmaBdV2_mGetStatus() because the status returned originates from the IP
 * driving the DMA channel. The status word will be specific to the IP.
 *
 * @param  BdPtr is the BD to operate on
 *
 * @return Word at offset XDMAV2_BD_SR_OFFSET
 *
 * @note
 * C-style signature:
 *    u32 XDmaBdV2_mGetDeviceStatus(XDmaBdV2* BdPtr)
 *
 ******************************************************************************/
#define XDmaBdV2_mGetDeviceStatus(BdPtr)                \
    XDmaV2_mReadBd((BdPtr), XDMAV2_BD_SR_OFFSET)

/*****************************************************************************/
/**
 * Retrieve number of bytes processed by the HW for the given BD. For receive
 * channels, the returned value is the number of bytes received. For transmit
 * channels, the returned value is the number of bytes transmitted.
 *
 * @param  BdPtr is the BD to operate on
 *
 * @return Bytes processed by HW
 *
 * @note
 * C-style signature:
 *    u32 XDmaBdV2_mGetProcessedLength(XDmaBdV2* BdPtr)
 *
 *****************************************************************************/
#define XDmaBdV2_mGetProcessedLength(BdPtr)             \
    (XDmaV2_mReadBd((BdPtr), XDMAV2_BD_LENCPY_OFFSET) - \
     XDmaV2_mReadBd((BdPtr), XDMAV2_BD_LENGTH_OFFSET))

/*****************************************************************************/
/**
 * Test whether the given BD has been marked as the last BD of a packet.
 *
 * @param  BdPtr is the BD to operate on
 *
 * @return TRUE if BD represents the "Last" BD of a packet, FALSE otherwise
 *
 * @note
 * C-style signature:
 *    u32 XDmaBdV2_mIsLast(XDmaBdV2* BdPtr)
 *
 ******************************************************************************/
#define XDmaBdV2_mIsLast(BdPtr)                                         \
    ((XDmaV2_mReadBd((BdPtr), XDMAV2_BD_DMACR_OFFSET) & XDMAV2_DMACR_L_MASK) ? \
     TRUE : FALSE)

/*****************************************************************************/
/**
 * Set the ID field of the given BD. The ID is an arbitrary piece of data the
 * user can associate with a specific BD.
 *
 * @param  BdPtr is the BD to operate on
 * @param  Id is a 32 bit quantity to set in the BD
 *
 * @note
 * C-style signature:
 *    void XDmaBdV2_mSetId(XDmaBdV2* BdPtr, void Id)
 *
 ******************************************************************************/
#define XDmaBdV2_mSetId(BdPtr, Id)                                      \
    (XDmaV2_mWriteBd((BdPtr), XDMAV2_BD_ID_OFFSET, (u32)Id))

/*****************************************************************************/
/**
 * Retrieve the ID field of the given BD previously set with XDmaBdV2_mSetId.
 *
 * @param  BdPtr is the BD to operate on
 *
 * @note
 * C-style signature:
 *    u32 XDmaBdV2_mGetId(XDmaBdV2* BdPtr)
 *
 ******************************************************************************/
#define XDmaBdV2_mGetId(BdPtr) (XDmaV2_mReadBd((BdPtr), XDMAV2_BD_ID_OFFSET))

/*****************************************************************************/
/**
 * Set the data realignment engine (DRE) control bit. Has no effect if channel
 * this BD belongs to does not have DRE capabilities.
 *
 * @param  BdPtr is the BD to operate on
 *
 * @note
 * C-style signature:
 *    void XDmaBdV2_mSetDre(XDmaBdV2* BdPtr)
 *
 ******************************************************************************/
#define XDmaBdV2_mSetDre(BdPtr)                                         \
    (XDmaV2_mWriteBd((BdPtr), XDMAV2_BD_DMACR_OFFSET,                   \
        XDmaV2_mReadBd((BdPtr), XDMAV2_BD_DMACR_OFFSET) | XDMAV2_DMACR_DRE_MASK))

/*****************************************************************************/
/**
 * Causes the DMA engine to increment the source buffer address during the DMA
 * transfer for this BD. This is the desirable setting when the buffer data
 * occupies a memory range.
 *
 * @param  BdPtr is the BD to operate on
 *
 * @note
 * C-style signature:
 *    void XDmaBdV2_mSetSrcBufIncrement(XDmaBdV2* BdPtr)
 *
 ******************************************************************************/
#define XDmaBdV2_mSetSrcBufIncrement(BdPtr)                             \
    (XDmaV2_mWriteBd((BdPtr), XDMAV2_BD_DMACR_OFFSET,                   \
        XDmaV2_mReadBd((BdPtr), XDMAV2_BD_DMACR_OFFSET) | XDMAV2_DMACR_SINC_MASK))

/*****************************************************************************/
/**
 * Cause the DMA engine to use the same source memory buffer address during the
 * DMA transfer for this BD. This is the desirable setting when the buffer data
 * occupies a single address as may be the case if transferring to/from FOFO.
 *
 * @param  BdPtr is the BD to operate on
 *
 * @note
 * C-style signature:
 *    void XDmaBdV2_mSetSrcBufNoIncrement(XDmaBdV2* BdPtr)
 *
 ******************************************************************************/
#define XDmaBdV2_mSetSrcBufNoIncrement(BdPtr)                           \
    (XDmaV2_mWriteBd((BdPtr), XDMAV2_BD_DMACR_OFFSET,                   \
        XDmaV2_mReadBd((BdPtr), XDMAV2_BD_DMACR_OFFSET) & ~XDMAV2_DMACR_SINC_MASK))

/*****************************************************************************/
/**
 * Causes the DMA engine to increment the destination buffer address during the
 * DMA transfer for this BD. This is the desirable setting when the buffer data
 * occupies a memory range.
 *
 * @param  BdPtr is the BD to operate on
 *
 * @note
 * C-style signature:
 *    void XDmaBdV2_mSetDestBufIncrement(XDmaBdV2* BdPtr)
 *
 ******************************************************************************/
#define XDmaBdV2_mSetDestBufIncrement(BdPtr)                            \
    (XDmaV2_mWriteBd((BdPtr), XDMAV2_BD_DMACR_OFFSET,                   \
        XDmaV2_mReadBd((BdPtr), XDMAV2_BD_DMACR_OFFSET) | XDMAV2_DMACR_DINC_MASK))

/*****************************************************************************/
/**
 * Cause the DMA engine to use the same destination memory buffer address during
 * the DMA transfer for this BD. This is the desirable setting when the buffer
 * data occupies a single address as may be the case if transferring to/from
 * FIFO.
 *
 * @param  BdPtr is the BD to operate on
 *
 * @note
 * C-style signature:
 *    void XDmaBdV2_mSetDestBufNoIncrement(XDmaBdV2* BdPtr)
 *
 ******************************************************************************/
#define XDmaBdV2_mSetDestBufNoIncrement(BdPtr)                          \
    (XDmaV2_mWriteBd((BdPtr), XDMAV2_BD_DMACR_OFFSET,                   \
        XDmaV2_mReadBd((BdPtr), XDMAV2_BD_DMACR_OFFSET) & ~XDMAV2_DMACR_DINC_MASK))

/*****************************************************************************/
/**
 * Set the BD to indicate that the direction of transfer is from a memory
 * range of user space to a fixed packet FIFO address in IP space.
 *
 * @param  BdPtr is the BD to operate on
 *
 * @note
 * C-style signature:
 *    void XDmaBdV2_mSetTxDir(XDmaBdV2* BdPtr)
 *
 ******************************************************************************/
#define XDmaBdV2_mSetTxDir(BdPtr)                                       \
    {                                                                   \
        u32 Cr = XDmaV2_mReadBd((BdPtr), XDMAV2_BD_DMACR_OFFSET);   \
        Cr |= (XDMAV2_DMACR_DLOCAL_MASK | XDMAV2_DMACR_SINC_MASK);      \
        Cr &= ~(XDMAV2_DMACR_SLOCAL_MASK | XDMAV2_DMACR_DINC_MASK);     \
        XDmaV2_mWriteBd((BdPtr),XDMAV2_BD_DMACR_OFFSET, Cr);            \
    }

/*****************************************************************************/
/**
 * Set the BD to indicate that the direction of transfer is from a fixed packet
 * FIFO address in IP space to memory range in user space.
 *
 * @param  BdPtr is the BD to operate on
 *
 * @note
 * C-style signature:
 *    void XDmaBdV2_mSetRxDir(XDmaBdV2* BdPtr)
 *
 ******************************************************************************/
#define XDmaBdV2_mSetRxDir(BdPtr)                                       \
    {                                                                   \
        u32 Cr = XDmaV2_mReadBd((BdPtr), XDMAV2_BD_DMACR_OFFSET);   \
        Cr |= (XDMAV2_DMACR_SLOCAL_MASK | XDMAV2_DMACR_DINC_MASK);      \
        Cr &= ~(XDMAV2_DMACR_DLOCAL_MASK | XDMAV2_DMACR_SINC_MASK);     \
        XDmaV2_mWriteBd((BdPtr),XDMAV2_BD_DMACR_OFFSET, Cr);            \
    }

/*****************************************************************************/
/**
 * Tell the SG DMA engine that the given BD marks the end of the current packet
 * to be processed.
 *
 * @param  BdPtr is the BD to operate on
 *
 * @note
 * C-style signature:
 *    void XDmaBdV2_mSetLast(XDmaBdV2* BdPtr)
 *
 ******************************************************************************/
#define XDmaBdV2_mSetLast(BdPtr)                                        \
    (XDmaV2_mWriteBd((BdPtr), XDMAV2_BD_DMACR_OFFSET,                   \
        XDmaV2_mReadBd((BdPtr), XDMAV2_BD_DMACR_OFFSET) | XDMAV2_DMACR_L_MASK))

/*****************************************************************************/
/**
 * Tell the SG DMA engine that the current packet does not end with the given
 * BD.
 *
 * @param  BdPtr is the BD to operate on
 *
 * @note
 * C-style signature:
 *    void XDmaBdV2_mClearLast(XDmaBdV2* BdPtr)
 *
 ******************************************************************************/
#define XDmaBdV2_mClearLast(BdPtr)                                      \
    (XDmaV2_mWriteBd((BdPtr), XDMAV2_BD_DMACR_OFFSET,                   \
        XDmaV2_mReadBd((BdPtr), XDMAV2_BD_DMACR_OFFSET) & ~XDMAV2_DMACR_L_MASK))

/*****************************************************************************/
/**
 * Set transfer length in bytes for the given BD. The length must be set each
 * time a BD is submitted to HW.
 *
 * @param  BdPtr is the BD to operate on
 * @param  LenBytes is the number of bytes to transfer.
 *
 * @note
 * C-style signature:
 *    void XDmaBdV2_mSetLength(XDmaBdV2* BdPtr, u32 LenBytes)
 *
 ******************************************************************************/
#define XDmaBdV2_mSetLength(BdPtr, LenBytes)                            \
    {                                                                   \
        XDmaV2_mWriteBd((BdPtr), XDMAV2_BD_LENGTH_OFFSET, (LenBytes));  \
        XDmaV2_mWriteBd((BdPtr), XDMAV2_BD_LENCPY_OFFSET, (LenBytes));  \
    }

/*****************************************************************************/
/**
 * Set the address of the BD's source buffer address.
 *
 * @param  BdPtr is the BD to operate on
 * @param  Addr is the address bits to set, LSB = 1.
 *
 * @note
 * C-style signature:
 *    void XDmaBdV2_mSetSrcAddr(XDmaBdV2* BdPtr, u32 LowAddr)
 *
 ******************************************************************************/
#define XDmaBdV2_mSetSrcAddr(BdPtr, Addr)                       \
    (XDmaV2_mWriteBd((BdPtr), XDMAV2_BD_SA_OFFSET, (u32)(Addr)))

/*****************************************************************************/
/**
 * Get the address of the BD's source buffer address.
 *
 * @param  BdPtr is the BD to operate on
 *
 * @note
 * C-style signature:
 *    u32 XDmaBdV2_mGetSrcAddr(XDmaBdV2* BdPtr)
 *
 ******************************************************************************/
#define XDmaBdV2_mGetSrcAddr(BdPtr)                                     \
    (XDmaV2_mReadBd((BdPtr), XDMAV2_BD_SA_OFFSET))

/*****************************************************************************/
/**
 * Set the address of the BD's destination buffer address.
 *
 * @param  BdPtr is the BD to operate on
 * @param  Addr is the address bits to set, LSB = 1.
 *
 * @note
 * C-style signature:
 *    void XDmaBdV2_mSetDestAddr(XDmaBdV2* BdPtr, u32 LowAddr)
 *
 ******************************************************************************/
#define XDmaBdV2_mSetDestAddr(BdPtr, Addr)                      \
    (XDmaV2_mWriteBd((BdPtr), XDMAV2_BD_DA_OFFSET, (u32)(Addr)))

/*****************************************************************************/
/**
 * Get the address of the BD's destination buffer address.
 *
 * @param  BdPtr is the BD to operate on
 *
 * @note
 * C-style signature:
 *    u32 XDmaBdV2_mGetDestAddr(XDmaBdV2* BdPtr)
 *
 ******************************************************************************/
#define XDmaBdV2_mGetDestAddr(BdPtr)                    \
    (XDmaV2_mReadBd((BdPtr), XDMAV2_BD_DA_OFFSET))

/************************** Function Prototypes ******************************/

#endif				/* end of protection macro */
