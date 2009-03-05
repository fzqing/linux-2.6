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
* @file xdmav2_l.h
*
* This header file contains identifiers and low-level driver functions (or
* macros) that can be used to access the Direct Memory Access and Scatter
* Gather (SG DMA) device.
*
* For more information about the operation of this device, see the hardware
* specification and documentation in the higher level driver xdmav2.h
* source code file.
*
* <pre>
* MODIFICATION HISTORY:
*
* Who  Date     Changes
* ---- -------- -----------------------------------------------
* rmm  06/01/05 First release
* </pre>
*
******************************************************************************/

#ifndef XDMAV2_L_H		/* prevent circular inclusions */
#define XDMAV2_L_H		/* by using protection macros */

/***************************** Include Files *********************************/

#include "xbasic_types.h"
#include "xio.h"

/************************** Constant Definitions *****************************/

/* Register offset definitions. Unless otherwise noted, register access is
 * 32 bit.
 */

/** @name DMA channel registers
 *  @{
 */
#define XDMAV2_RST_OFFSET    0x00000000	 /**< Reset (write) */
#define XDMAV2_MIR_OFFSET    0x00000000	 /**< MIR register (read) */
#define XDMAV2_DMACR_OFFSET  0x00000004	 /**< DMA Control Register */
#define XDMAV2_SA_OFFSET     0x00000008	 /**< Source address */
#define XDMAV2_DA_OFFSET     0x0000000C	 /**< Destination address */
#define XDMAV2_LENGTH_OFFSET 0x00000010	 /**< DMA Length */
#define XDMAV2_DMASR_OFFSET  0x00000014	 /**< DMA Status Register */
#define XDMAV2_BDA_OFFSET    0x00000018	 /**< Buffer Descriptor Address */
#define XDMAV2_SWCR_OFFSET   0x0000001C	 /**< Software Control Register */
#define XDMAV2_UPC_OFFSET    0x00000020	 /**< Unserviced packet count */
#define XDMAV2_PCT_OFFSET    0x00000024	 /**< Packet count threshold */
#define XDMAV2_PWB_OFFSET    0x00000028	 /**< Packet waitbound */
#define XDMAV2_ISR_OFFSET    0x0000002C	 /**< Interrupt Status Register */
#define XDMAV2_IER_OFFSET    0x00000030	 /**< Interrupt Enable Register */

/*@}*/

/** @name Buffer Descriptor register offsets
 *  @{
 */
#define XDMAV2_BD_SR_OFFSET        0x00	 /**< Packet Status */
#define XDMAV2_BD_DMACR_OFFSET     0x04	 /**< Channel DMACR register contents */
#define XDMAV2_BD_SA_OFFSET        0x08	 /**< Source address */
#define XDMAV2_BD_DA_OFFSET        0x0C	 /**< Destination address */
#define XDMAV2_BD_LENGTH_OFFSET    0x10	 /**< DMA Length */
#define XDMAV2_BD_DMASR_OFFSET     0x14	 /**< Channel DMASR register contents */
#define XDMAV2_BD_BDA_OFFSET       0x18	 /**< Next buffer descriptor pointer */
#define XDMAV2_BD_ID_OFFSET        0x1C	 /**< SW Driver usage */
#define XDMAV2_BD_FLAGS_OFFSET     0x20	 /**< SW Driver usage */
#define XDMAV2_BD_LENCPY_OFFSET    0x24	 /**< SW Driver usage */

#define XDMAV2_BD_NUM_WORDS        10	 /**< Number of 32-bit words that make
                                              up a BD */
/*@}*/

/* Register masks. The following constants define bit locations of various
 * control bits in the registers. Constants are not defined for those registers
 * that have a single bit field representing all 32 bits. For further
 * information on the meaning of the various bit masks, refer to the HW spec.
 */

/** @name Reset Register (RST) bitmasks
 *  @{
 */
#define XDMAV2_RST_MASK           0x0000000A  /**< Write this value to RST to
                                                   reset the channel */
/*@}*/

/** @name Module ID Register (MIR) bitmasks
 *  @{
 */
#define XDMAV2_MIR_MAJOR_MASK       0xF0000000
#define XDMAV2_MIR_MINOR_MASK       0x0FE00000
#define XDMAV2_MIR_CHAN_BLOCK_MASK  0x001F0000
#define XDMAV2_MIR_CHAN_TYPE_MASK   0x000000FF

/* Subfields within XDMAV2_MIR_CHAN_TYPE_MASK */
#define XDMAV2_MIR_CHAN_TYPE_SDMA    0x00000004
#define XDMAV2_MIR_CHAN_TYPE_SSGDMA  0x00000005
#define XDMAV2_MIR_CHAN_TYPE_SGDMATX 0x00000006
#define XDMAV2_MIR_CHAN_TYPE_SGDMARX 0x00000007

/*@}*/

/** @name DMA Control Register (DMACR) bitmasks
 *  @note These bitmasks are identical between XDMAV2_DMACR_OFFSET and
 *  XDMAV2_BD_DMACR_OFFSET
 * @{
 */
#define XDMAV2_DMACR_SINC_MASK    0x80000000  /**< Source address increment */
#define XDMAV2_DMACR_DINC_MASK    0x40000000  /**< Destination address increment */
#define XDMAV2_DMACR_SLOCAL_MASK  0x20000000  /**< Source address is local */
#define XDMAV2_DMACR_DLOCAL_MASK  0x10000000  /**< Destination address is local */
#define XDMAV2_DMACR_SGS_MASK     0x08000000  /**< Scatter gather stop */
#define XDMAV2_DMACR_L_MASK       0x02000000  /**< Last BD of packet */
#define XDMAV2_DMACR_DRE_MASK     0x01000000  /**< Use DRE transfer capabilities */
#define XDMAV2_DMACR_RESET_MASK   0x98000000  /**< DMACR contents at reset */
/*@}*/

/** @name DMA Status Register (DMASR) bitmasks
 *  @note These bitmasks are identical between XDMAV2_DMASR_OFFSET and
 *  XDMAV2_BD_DMASR_OFFSET
 * @{
 */
#define XDMAV2_DMASR_DMABSY_MASK  0x80000000  /**< DMA busy */
#define XDMAV2_DMASR_DBE_MASK     0x40000000  /**< Bus error */
#define XDMAV2_DMASR_DBT_MASK     0x20000000  /**< Bus timeout */
#define XDMAV2_DMASR_L_MASK       0x10000000  /**< Last BD of packet */
#define XDMAV2_DMASR_SGBSY_MASK   0x08000000  /**< SG channel busy */

/*@}*/

/** @name Software control register (SWCR) bitmasks
 *  @{
 */
#define XDMAV2_SWCR_SGE_MASK     0x80000000  /**< SG Enable */

/*@}*/

/**< @name Packet Threshold count (PCT) bitmasks
 *  @{
 */
#define XDMAV2_PCT_MASK          0x0000003FF /**< Value mask */

/*@}*/

/**< @name Packet Waitbound (PWB) bitmasks
 *  @{
 */
#define XDMAV2_PWB_MASK          0x0000003FF /**< Value mask */

/*@}*/

/** @name Interrupt status bits for MAC interrupts
 *  These bits are associated with XDMAV2_ISR_OFFSET and
 *  XDMAV2_IER_OFFSET registers.
 *  @{
 */
#define XDMAV2_IPXR_SGEND_MASK   0x00000040  /**< SG End */
#define XDMAV2_IPXR_SGDA_MASK    0x00000020  /**< SG Disable ack */
#define XDMAV2_IPXR_PWBR_MASK    0x00000010  /**< Pkt waitbound reached */
#define XDMAV2_IPXR_PCTR_MASK    0x00000008  /**< Pkt count threshold reached */
#define XDMAV2_IPXR_PD_MASK      0x00000004  /**< Pkt done */
#define XDMAV2_IPXR_DE_MASK      0x00000002  /**< DMA error */
#define XDMAV2_IPXR_DD_MASK      0x00000001  /**< DMA complete */
/*@}*/

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/

/****************************************************************************/
/**
 *
 * Read the given IPIF register.
 *
 * @param    BaseAddress is the IPIF base address of the device
 * @param    RegOffset is the register offset to be read
 *
 * @return   The 32-bit value of the register
 *
 * @note
 * C-style signature:
 *    u32 XDmaV2_mReadReg(u32 BaseAddress, u32 RegOffset)
 *
 *****************************************************************************/
#define XDmaV2_mReadReg(BaseAddress, RegOffset) \
    XIo_In32((BaseAddress) + (RegOffset))

/****************************************************************************/
/**
 *
 * Write the given IPIF register.
 *
 * @param    BaseAddress is the IPIF base address of the device
 * @param    RegOffset is the register offset to be written
 * @param    Data is the 32-bit value to write to the register
 *
 * @return   None.
 *
 * @note
 * C-style signature:
 *    void XDmaV2_mWriteReg(u32 BaseAddress, u32 RegOffset,
 *                          u32 Data)
 *
 *****************************************************************************/
#define XDmaV2_mWriteReg(BaseAddress, RegOffset, Data)       \
    XIo_Out32((BaseAddress) + (RegOffset), (Data))

/****************************************************************************/
/**
 *
 * Read the given Buffer Descriptor word.
 *
 * @param    BaseAddress is the base address of the BD to read
 * @param    Offset is the word offset to be read
 *
 * @return   The 32-bit value of the field
 *
 * @note
 * C-style signature:
 *    u32 XDmaV2_mReadBd(u32 BaseAddress, u32 Offset)
 *
 *****************************************************************************/
#define XDmaV2_mReadBd(BaseAddress, Offset)                       \
    (*(u32*)((u32)(BaseAddress) + (Offset)))

/****************************************************************************/
/**
 *
 * Write the given Buffer Descriptor word.
 *
 * @param    BaseAddress is the base address of the BD to write
 * @param    Offset is the word offset to be written
 * @param    Data is the 32-bit value to write to the field
 *
 * @return   None.
 *
 * @note
 * C-style signature:
 *    void XDmaV2_mWriteReg(u32 BaseAddress, u32 RegOffset, u32 Data)
 *
 *****************************************************************************/
#define XDmaV2_mWriteBd(BaseAddress, Offset, Data)                \
    (*(u32*)((u32)(BaseAddress) + (Offset)) = (Data))

/************************** Function Prototypes ******************************/

#endif				/* end of protection macro */
