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
 * @file xtemac_control.c
 *
 * Functions in this file implement general purpose command and control related
 * functionality. See xtemac.h for a detailed description of the driver.
 *
 * <pre>
 * MODIFICATION HISTORY:
 *
 * Ver   Who  Date     Changes
 * ----- ---- -------- -------------------------------------------------------
 * 1.00a rmm  06/01/05 First release
 * </pre>
 *****************************************************************************/

/***************************** Include Files *********************************/

#include "xtemac.h"
#include "xtemac_i.h"

/************************** Constant Definitions *****************************/

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/

/************************** Function Prototypes ******************************/

/************************** Variable Definitions *****************************/

/*****************************************************************************/
/**
 * Set the MAC address for this driver/device.  The address is a 48-bit value.
 * The device must be stopped before calling this function.
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 * @param AddressPtr is a pointer to a 6-byte MAC address.
 *
 * @return
 * - XST_SUCCESS if the MAC address was set successfully
 * - XST_DEVICE_IS_STARTED if the device has not yet been stopped
 *
 ******************************************************************************/
XStatus XTemac_SetMacAddress(XTemac * InstancePtr, void *AddressPtr)
{
	u32 MacAddr;
	u8 *Aptr = AddressPtr;

	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	/* Be sure device has been stopped */
	if (InstancePtr->IsStarted == XCOMPONENT_IS_STARTED) {
		return (XST_DEVICE_IS_STARTED);
	}

	/* Set the MAC bits [31:0] in EUAW0 */
	MacAddr = Aptr[0] & 0x000000FF;
	MacAddr |= Aptr[1] << 8;
	MacAddr |= Aptr[2] << 16;
	MacAddr |= Aptr[3] << 24;
	XTemac_mSetHostReg(XTE_EUAW0_OFFSET, MacAddr);

	/* There are reserved bits in EUAW1 so don't affect them */
	MacAddr = XTemac_mGetHostReg(XTE_EUAW1_OFFSET);
	MacAddr &= ~XTE_EUAW1_MASK;

	/* Set MAC bits [47:32] in EUAW1 */
	MacAddr |= Aptr[4] & 0x000000FF;
	MacAddr |= Aptr[5] << 8;
	XTemac_mSetHostReg(XTE_EUAW1_OFFSET, MacAddr);

	return (XST_SUCCESS);
}

/*****************************************************************************/
/**
 * Get the MAC address for this driver/device.
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 * @param AddressPtr is an output parameter, and is a pointer to a buffer into
 *        which the current MAC address will be copied. The buffer must be at
 *        least 6 bytes in length.
 *
 ******************************************************************************/
void XTemac_GetMacAddress(XTemac * InstancePtr, void *AddressPtr)
{
	u32 MacAddr;
	u8 *Aptr = AddressPtr;

	XASSERT_VOID(InstancePtr != NULL);
	XASSERT_VOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	/* Read MAC bits [31:0] in EUAW0 */
	MacAddr = XTemac_mGetHostReg(XTE_EUAW0_OFFSET);
	Aptr[0] = (u8) MacAddr;
	Aptr[1] = (u8) (MacAddr >> 8);
	Aptr[2] = (u8) (MacAddr >> 16);
	Aptr[3] = (u8) (MacAddr >> 24);

	/* Read MAC bits [47:32] in EUAW1 */
	MacAddr = XTemac_mGetHostReg(XTE_EUAW1_OFFSET);
	Aptr[4] = (u8) MacAddr;
	Aptr[5] = (u8) (MacAddr >> 8);
}

/*****************************************************************************/
/**
 * Add an Ethernet address to the list that will be accepted by the receiver.
 * The address may be any unicast, multicast, or the broadcast address form.
 * Up to XTE_MULTI_CAM_ENTRIES addresses may be filtered in this way. The
 * device must be stopped to use this function.
 *
 * Once an address is programmed, it will be received by the device. There is
 * no control bit to disable multicast filtering. The only way to prevent a
 * CAM address from being received is to clear it with XTemac_MulticastClear().
 *
 * @param InstancePtr is a pointer to the XTemac instance to be worked on.
 * @param AddressPtr is a pointer to a 6-byte Ethernet address. The previous
 *        address at this entry location (if any) is overwritten with the new
 *        one.
 * @param Entry is the storage location the HW uses to program this address.
 *        It must be between 0..XTE_MULTI_CAM_ENTRIES-1.
 *
 * @return
 *
 * - XST_SUCCESS if the address was added successfully
 * - XST_DEVICE_IS_STARTED if the device has not yet been stopped
 ******************************************************************************/
#if 0				/* No HW support */
XStatus XTemac_MulticastAdd(XTemac * InstancePtr, void *AddressPtr, int Entry)
{
	u32 Emaw0Reg;
	u32 Emaw1Reg;
	u8 *Aptr = (u8 *) AddressPtr;

	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);
	XASSERT_NONVOID(Entry < XTE_MULTI_CAM_ENTRIES);

	/* The device must be stopped before clearing the multicast hash table */
	if (InstancePtr->IsStarted == XCOMPONENT_IS_STARTED) {
		return (XST_DEVICE_IS_STARTED);
	}

	/* Set MAC bits [31:0] */
	Emaw0Reg = Aptr[0] & 0x000000FF;
	Emaw0Reg |= Aptr[1] << 8;
	Emaw0Reg |= Aptr[2] << 16;
	Emaw0Reg |= Aptr[3] << 24;

	/* Set MAC bits [47:32] */
	Emaw1Reg = Aptr[4] & 0x000000FF;
	Emaw1Reg |= Aptr[5] << 8;

	/* Add in CAM address */
	Emaw1Reg |= (Entry << XTE_EMAW1_CAMMADDR_SHIFT_MASK);

	/* Program HW */
	XTemac_mSetHostReg(XTE_EMAW0_OFFSET, Emaw0Reg);
	XTemac_mSetHostReg(XTE_EMAW1_OFFSET, Emaw1Reg);

	return (XST_SUCCESS);
}
#endif

/*****************************************************************************/
/**
 * Retrieve an Ethernet address set by XTemac_MulticastAdd().
 *
 * @param InstancePtr is a pointer to the XTemac instance to be worked on.
 * @param AddressPtr is an output parameter, and is a pointer to a buffer into
 *        which the current MAC address will be copied. The buffer must be at
 *        least 6 bytes in length.
 * @param Entry is the storage location in the HW. It must be between
 *        0..XTE_MULTI_CAM_ENTRIES-1.
 *
 * @return
 *
 * - XST_SUCCESS if the address was added successfully
 * - XST_DEVICE_IS_STARTED if the device has not yet been stopped
 ******************************************************************************/
#if 0				/* No HW support */
void XTemac_MulticastGet(XTemac * InstancePtr, void *AddressPtr, int Entry)
{
	u32 Emaw0Reg;
	u32 Emaw1Reg;
	u8 *Aptr = AddressPtr;

	XASSERT_VOID(InstancePtr != NULL);
	XASSERT_VOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);
	XASSERT_VOID(Entry < XTE_MULTI_CAM_ENTRIES);

	/* Tell HW to provide address stored in given entry */
	XTemac_mSetHostReg(XTE_EMAW1_OFFSET, XTE_EMAW1_CAMRNW_MASK |
			   (Entry << XTE_EMAW1_CAMMADDR_SHIFT_MASK));

	/* The HW should now have provided the CAM entry */
	Emaw0Reg = XTemac_mGetHostReg(XTE_EMAW0_OFFSET);
	Emaw1Reg = XTemac_mGetHostReg(XTE_EMAW1_OFFSET);

	/* Copy the address to the user buffer */
	Aptr[0] = (u8) Emaw0Reg;
	Aptr[1] = (u8) (Emaw0Reg >> 8);
	Aptr[2] = (u8) (Emaw0Reg >> 16);
	Aptr[3] = (u8) (Emaw0Reg >> 24);
	Aptr[4] = (u8) Emaw1Reg;
	Aptr[5] = (u8) (Emaw1Reg >> 8);
}
#endif

/*****************************************************************************/
/**
* Clear an address set by XTemac_MulticastAdd(). The device must be stopped
* before calling this function.
*
* @param InstancePtr is a pointer to the XTemac instance to be worked on.
* @param Entry is the HW storage location used when this address was added.
*        It must be between 0..XTE_MULTI_CAM_ENTRIES-1.
*
* @return
*
* - XST_SUCCESS if the address was cleared
* - XST_DEVICE_IS_STARTED if the device has not yet been stopped
*
******************************************************************************/
#if 0				/* No HW support */
XStatus XTemac_MulticastClear(XTemac * InstancePtr, int Entry)
{
	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);
	XASSERT_NONVOID(Entry < XTE_MULTI_CAM_ENTRIES);

	/* The device must be stopped before clearing the multicast hash table */
	if (InstancePtr->IsStarted == XCOMPONENT_IS_STARTED) {
		return (XST_DEVICE_IS_STARTED);
	}

	/* Clear the entry by writing 0:0:0:0:0:0 to it */
	XTemac_mSetHostReg(XTE_EMAW0_OFFSET, 0);
	XTemac_mSetHostReg(XTE_EMAW1_OFFSET,
			   Entry << XTE_EMAW1_CAMMADDR_SHIFT_MASK);

	return (XST_SUCCESS);
}
#endif

/*****************************************************************************/
/**
 * Set the MAC address for pause frames. This is the address the device will
 * recognize as pause frames. Pause frames transmitted with
 * XTemac_SendPausePacket() will also use this address.
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 * @param AddressPtr is a pointer to a 6-byte MAC address.
 *
 * @return
 * - XST_SUCCESS if the MAC address was set successfully
 * - XST_DEVICE_IS_STARTED if the device has not yet been stopped
 *
 ******************************************************************************/
XStatus XTemac_SetMacPauseAddress(XTemac * InstancePtr, void *AddressPtr)
{
	u32 MacAddr;
	u8 *Aptr = AddressPtr;

	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	/* Be sure device has been stopped */
	if (InstancePtr->IsStarted == XCOMPONENT_IS_STARTED) {
		return (XST_DEVICE_IS_STARTED);
	}

	/* Set the MAC bits [31:0] in ERXC0 */
	MacAddr = Aptr[0] & 0x000000FF;
	MacAddr |= Aptr[1] << 8;
	MacAddr |= Aptr[2] << 16;
	MacAddr |= Aptr[3] << 24;
	XTemac_mSetHostReg(XTE_ERXC0_OFFSET, MacAddr);

	/* ERXC1 contains other info that must be preserved */
	MacAddr = XTemac_mGetHostReg(XTE_ERXC1_OFFSET);
	MacAddr &= ~XTE_ERXC1_ERXC1_MASK;;

	/* Set MAC bits [47:32] */
	MacAddr |= Aptr[4] & 0x000000FF;
	MacAddr |= Aptr[5] << 8;
	XTemac_mSetHostReg(XTE_ERXC1_OFFSET, MacAddr);

	return (XST_SUCCESS);
}

/*****************************************************************************/
/**
 * Get the MAC address for pause frames for this driver/device.
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 * @param AddressPtr is an output parameter, and is a pointer to a buffer into
 *        which the current MAC address will be copied. The buffer must be at
 *        least 6 bytes in length.
 *
 ******************************************************************************/
void XTemac_GetMacPauseAddress(XTemac * InstancePtr, void *AddressPtr)
{
	u32 MacAddr;
	u8 *Aptr = AddressPtr;

	XASSERT_VOID(InstancePtr != NULL);
	XASSERT_VOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	/* Read MAC bits [31:0] in ERXC0 */
	MacAddr = XTemac_mGetHostReg(XTE_ERXC0_OFFSET);
	Aptr[0] = (u8) MacAddr;
	Aptr[1] = (u8) (MacAddr >> 8);
	Aptr[2] = (u8) (MacAddr >> 16);
	Aptr[3] = (u8) (MacAddr >> 24);

	/* Read MAC bits [47:32] in ERXC1 */
	MacAddr = XTemac_mGetHostReg(XTE_ERXC1_OFFSET);
	Aptr[4] = (u8) MacAddr;
	Aptr[5] = (u8) (MacAddr >> 8);
}

/*****************************************************************************/
/**
 * Set options for the driver/device. The driver should be stopped with
 * XTemac_Stop() before changing options.
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 * @param Options are the options to set. Multiple options can be set by OR'ing
 *        XTE_*_OPTIONS constants together. Options not specified are not
 *        affected.
 *
 * @return
 * - XST_SUCCESS if the options were set successfully
 * - XST_DEVICE_IS_STARTED if the device has not yet been stopped
 *
 * @note
 * See xtemac.h for a description of the available options.
 *
 ******************************************************************************/
XStatus XTemac_SetOptions(XTemac * InstancePtr, u32 Options)
{
	u32 Reg;		/* Generic register contents */
	u32 RegErxc1;		/* Reflects original contents of ERXC1 */
	u32 RegEtxc;		/* Reflects original contents of ETXC  */
	u32 RegNewErxc1;	/* Reflects new contents of ERXC1 */
	u32 RegNewEtxc;		/* Reflects new contents of ETXC  */

	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	/* Be sure device has been stopped */
	if (InstancePtr->IsStarted == XCOMPONENT_IS_STARTED) {
		return (XST_DEVICE_IS_STARTED);
	}

	/* Many of these options will change the ERXC1 or ETXC registers.
	 * To reduce the amount of IO to the device, group these options here
	 * and change them all at once.
	 */

	/* Grab current register contents */
	RegErxc1 = XTemac_mGetHostReg(XTE_ERXC1_OFFSET);
	RegEtxc = XTemac_mGetHostReg(XTE_ETXC_OFFSET);
	RegNewErxc1 = RegErxc1;
	RegNewEtxc = RegEtxc;

	/* Turn on jumbo packet support for both Rx and Tx */
	if (Options & XTE_JUMBO_OPTION) {
		RegNewEtxc |= XTE_ETXC_TXJMBO_MASK;
		RegNewErxc1 |= XTE_ERXC1_RXJMBO_MASK;
	}

	/* Turn on VLAN packet support for both Rx and Tx */
	if (Options & XTE_VLAN_OPTION) {
		RegNewEtxc |= XTE_ETXC_TXVLAN_MASK;
		RegNewErxc1 |= XTE_ERXC1_RXVLAN_MASK;
	}

	/* Turn on half duplex connectivity for both Rx and Tx */
	if (Options & XTE_HALF_DUPLEX_OPTION) {
		RegNewEtxc |= XTE_ETXC_TXHD_MASK;
		RegNewErxc1 |= XTE_ERXC1_RXHD_MASK;
	}

	/* Turn on FCS stripping on receive packets */
	if (Options & XTE_FCS_STRIP_OPTION) {
		RegNewErxc1 &= ~XTE_ERXC1_RXFCS_MASK;
	}

	/* Turn on FCS insertion on transmit packets */
	if (Options & XTE_FCS_INSERT_OPTION) {
		RegNewEtxc &= ~XTE_ETXC_TXFCS_MASK;
	}

	/* Turn on length/type field checking on receive packets */
	if (Options & XTE_LENTYPE_ERR_OPTION) {
		RegNewErxc1 &= ~XTE_ERXC1_RXLT_MASK;
	}

	/* Officially change the ETXC or ERXC1 registers if they need to be
	 * modified
	 */
	if (RegEtxc != RegNewEtxc) {
		XTemac_mSetHostReg(XTE_ETXC_OFFSET, RegNewEtxc);
	}

	if (RegErxc1 != RegNewErxc1) {
		XTemac_mSetHostReg(XTE_ERXC1_OFFSET, RegNewErxc1);
	}

	/* Rest of options twiddle bits of other registers. Handle them one at
	 * a time
	 */

	/* Turn on flow control */
	if (Options & XTE_FLOW_CONTROL_OPTION) {
		Reg = XTemac_mGetHostReg(XTE_EFCC_OFFSET);
		Reg |= XTE_EFCC_RXFLO_MASK;
		XTemac_mSetHostReg(XTE_EFCC_OFFSET, Reg);
	}

	/* Turn on promiscuous frame filtering (all frames are received ) */
	if (Options & XTE_PROMISC_OPTION) {
		Reg = XTemac_mGetHostReg(XTE_EAFM_OFFSET);
		Reg |= XTE_EAFM_EPPRM_MASK;
		XTemac_mSetHostReg(XTE_EAFM_OFFSET, Reg);
	}

	/* Allow broadcast address filtering */
	if (Options & XTE_BROADCAST_OPTION) {
		Reg = XTemac_mGetIpifReg(XTE_CR_OFFSET);
		Reg &= ~XTE_CR_BCREJ_MASK;
		XTemac_mSetIpifReg(XTE_CR_OFFSET, Reg);
	}

	/* Allow multicast address filtering */
	if (Options & XTE_MULTICAST_CAM_OPTION) {
		Reg = XTemac_mGetIpifReg(XTE_CR_OFFSET);
		Reg &= ~XTE_CR_MCREJ_MASK;
		XTemac_mSetIpifReg(XTE_CR_OFFSET, Reg);
	}

	/* Enable interrupts related to rejection of bad frames */
	if (Options & XTE_REPORT_RXERR_OPTION) {
		/* Clear out any previous error conditions that may have existed
		 * prior to enabling the reporting of these types of errors
		 */
		Reg = XTemac_mGetIpifReg(XTE_IPISR_OFFSET);
		XTemac_mSetIpifReg(XTE_IPISR_OFFSET,
				   Reg & XTE_IPXR_RECV_DROPPED_MASK);

		/* Whether these are enabled here are based on the last call to
		 * XTemac_IntrFifoEnable/Disable() and XTemac_IntrSgDmaEnable/Disable()
		 * for the receive channel.
		 *
		 * If receive interrupts are enabled, then enable these interrupts. This
		 * way, when XTemac_Start() is called, these interrupt enables take
		 * effect right away.
		 *
		 * If receive interrupts are disabled, then don't do anything here. The
		 * XTemac_IntrFifoEnable() and XTemac_IntrSgDmaEnable() functions when
		 * called will check this option and enable these interrupts if needed.
		 */
		if (InstancePtr->Flags &
		    (XTE_FLAGS_RECV_FIFO_INT_ENABLE |
		     XTE_FLAGS_RECV_SGDMA_INT_ENABLE)) {
			Reg = XTemac_mGetIpifReg(XTE_IPIER_OFFSET);
			Reg |= XTE_IPXR_RECV_DROPPED_MASK;
			XTemac_mSetIpifReg(XTE_IPIER_OFFSET, Reg);
		}
	}

	/* The remaining options not handled here are managed elsewhere in the
	 * driver. No register modifications are needed at this time. Reflecting the
	 * option in InstancePtr->Options is good enough for now.
	 */

	/* Set options word to its new value */
	InstancePtr->Options |= Options;

	return (XST_SUCCESS);
}

/*****************************************************************************/
/**
 * Clear options for the driver/device
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 * @param Options are the options to clear. Multiple options can be cleared by
 *        OR'ing XTE_*_OPTIONS constants together. Options not specified are not
 *        affected.
 *
 * @return
 * - XST_SUCCESS if the options were set successfully
 * - XST_DEVICE_IS_STARTED if the device has not yet been stopped
 *
 * @note
 * See xtemac.h for a description of the available options.
 *
 ******************************************************************************/
XStatus XTemac_ClearOptions(XTemac * InstancePtr, u32 Options)
{
	volatile u32 Reg;	/* Generic */
	u32 RegErxc1;		/* Reflects original contents of ERXC1 */
	u32 RegEtxc;		/* Reflects original contents of ETXC  */
	u32 RegNewErxc1;	/* Reflects new contents of ERXC1 */
	u32 RegNewEtxc;		/* Reflects new contents of ETXC  */

	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	/* Be sure device has been stopped */
	if (InstancePtr->IsStarted == XCOMPONENT_IS_STARTED) {
		return (XST_DEVICE_IS_STARTED);
	}

	/* Many of these options will change the ERXC1 or ETXC registers.
	 * Group these options here and change them all at once. What we are
	 * trying to accomplish is to reduce the amount of IO to the device
	 */

	/* Grab current register contents */
	RegErxc1 = XTemac_mGetHostReg(XTE_ERXC1_OFFSET);
	RegEtxc = XTemac_mGetHostReg(XTE_ETXC_OFFSET);
	RegNewErxc1 = RegErxc1;
	RegNewEtxc = RegEtxc;

	/* Turn off jumbo packet support for both Rx and Tx */
	if (Options & XTE_JUMBO_OPTION) {
		RegNewEtxc &= ~XTE_ETXC_TXJMBO_MASK;
		RegNewErxc1 &= ~XTE_ERXC1_RXJMBO_MASK;
	}

	/* Turn off VLAN packet support for both Rx and Tx */
	if (Options & XTE_VLAN_OPTION) {
		RegNewEtxc &= ~XTE_ETXC_TXVLAN_MASK;
		RegNewErxc1 &= ~XTE_ERXC1_RXVLAN_MASK;
	}

	/* Turn off half duplex connectivity for both Rx and Tx */
	if (Options & XTE_HALF_DUPLEX_OPTION) {
		RegNewEtxc &= ~XTE_ETXC_TXHD_MASK;
		RegNewErxc1 &= ~XTE_ERXC1_RXHD_MASK;
	}

	/* Turn off FCS stripping on receive packets */
	if (Options & XTE_FCS_STRIP_OPTION) {
		RegNewErxc1 |= XTE_ERXC1_RXFCS_MASK;
	}

	/* Turn off FCS insertion on transmit packets */
	if (Options & XTE_FCS_INSERT_OPTION) {
		RegNewEtxc |= XTE_ETXC_TXFCS_MASK;
	}

	/* Turn off length/type field checking on receive packets */
	if (Options & XTE_LENTYPE_ERR_OPTION) {
		RegNewErxc1 |= XTE_ERXC1_RXLT_MASK;
	}

	/* Disable transmitter */
	if (Options & XTE_TRANSMITTER_ENABLE_OPTION) {
		RegNewEtxc &= ~XTE_ETXC_TXEN_MASK;
	}

	/* Disable receiver */
	if (Options & XTE_RECEIVER_ENABLE_OPTION) {
		RegNewErxc1 &= ~XTE_ERXC1_RXEN_MASK;
	}

	/* Officially change the ETXC or ERXC1 registers if they need to be
	 * modified
	 */
	if (RegEtxc != RegNewEtxc) {
		XTemac_mSetHostReg(XTE_ETXC_OFFSET, RegNewEtxc);
	}

	if (RegErxc1 != RegNewErxc1) {
		XTemac_mSetHostReg(XTE_ERXC1_OFFSET, RegNewErxc1);
	}

	/* Rest of options twiddle bits of other registers. Handle them one at
	 * a time
	 */

	/* Turn off flow control */
	if (Options & XTE_FLOW_CONTROL_OPTION) {
		Reg = XTemac_mGetHostReg(XTE_EFCC_OFFSET);
		Reg &= ~XTE_EFCC_RXFLO_MASK;
		XTemac_mSetHostReg(XTE_EFCC_OFFSET, Reg);
	}

	/* Turn off promiscuous frame filtering */
	if (Options & XTE_PROMISC_OPTION) {
		Reg = XTemac_mGetHostReg(XTE_EAFM_OFFSET);
		Reg &= ~XTE_EAFM_EPPRM_MASK;
		XTemac_mSetHostReg(XTE_EAFM_OFFSET, Reg);
	}

	/* Disable broadcast address filtering */
	if (Options & XTE_BROADCAST_OPTION) {
		Reg = XTemac_mGetIpifReg(XTE_CR_OFFSET);
		Reg |= XTE_CR_BCREJ_MASK;
		XTemac_mSetIpifReg(XTE_CR_OFFSET, Reg);
	}

	/* Disable multicast address filtering */
	if (Options & XTE_MULTICAST_CAM_OPTION) {
		Reg = XTemac_mGetIpifReg(XTE_CR_OFFSET);
		Reg |= XTE_CR_MCREJ_MASK;
		XTemac_mSetIpifReg(XTE_CR_OFFSET, Reg);
	}

	/* Disable interrupts related to rejection of bad frames */
	if (Options & XTE_REPORT_RXERR_OPTION) {
		Reg = XTemac_mGetIpifReg(XTE_IPIER_OFFSET);
		Reg &= ~XTE_IPXR_RECV_DROPPED_MASK;
		XTemac_mSetIpifReg(XTE_IPIER_OFFSET, Reg);
	}

	/* The remaining options not handled here are managed elsewhere in the
	 * driver. No register modifications are needed at this time. Reflecting the
	 * option in InstancePtr->Options is good enough for now.
	 */

	/* Set options word to its new value */
	InstancePtr->Options &= ~Options;

	return (XST_SUCCESS);
}

/*****************************************************************************/
/**
 * Get current option settings
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 *
 * @return
 * A bitmask of XTE_*_OPTION constants. Any bit set to 1 is to be interpreted
 * as a set opion.
 *
 * @note
 * See xtemac.h for a description of the available options.
 *
 ******************************************************************************/
u32 XTemac_GetOptions(XTemac * InstancePtr)
{
	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	return (InstancePtr->Options);
}

/*****************************************************************************/
/**
 * Send a pause packet
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 * @param PauseValue is the pause value in units of 512 bit times.
 *
 * @return
 * - XST_SUCCESS if pause frame transmission was initiated
 * - XST_DEVICE_IS_STOPPED if the device has not been started.
 *
 ******************************************************************************/
XStatus XTemac_SendPausePacket(XTemac * InstancePtr, u16 PauseValue)
{
	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	/* Make sure device is ready for this operation */
	if (InstancePtr->IsStarted != XCOMPONENT_IS_STARTED) {
		return (XST_DEVICE_IS_STOPPED);
	}

	/* Send flow control frame */
	XTemac_mSetIpifReg(XTE_TPPR_OFFSET, (u32) PauseValue);
	return (XST_SUCCESS);
}

/*****************************************************************************/
/**
 * Adjust the interframe gap. The MAC has a minimum IFG of 96 bit times. Using
 * this function increases that IFG by the specified amount in the given
 * parameter.
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 * @param AdditionalIfg is the new IFG extension value with a range of 0..255.
 *        The LSB is 8 bit times. For example, if this parameter is 6, then the
 *        total IFG will become 96 + (6 * 8) = 144 bit times.
 *
 ******************************************************************************/
void XTemac_SetIfg(XTemac * InstancePtr, u8 AdditionalIfg)
{
	XASSERT_VOID(InstancePtr != NULL);
	XASSERT_VOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	XTemac_mSetIpifReg(XTE_IFGP_OFFSET, (u32) AdditionalIfg);
}

/*****************************************************************************/
/**
 * Get the current interframe gap as set by XTemac_SetIfg().
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 *
 * @return the current addtional interframe gap.
 ******************************************************************************/
u8 XTemac_GetIfg(XTemac * InstancePtr)
{
	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	return ((u8) XTemac_mGetIpifReg(XTE_IFGP_OFFSET));
}

/*****************************************************************************/
/**
 * Get information about the physical link interface
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 * @param MiiTypePtr (output) is set to either XTE_MII_TYPE_RGMII for RGMII mode,
 *        XTE_MII_TYPE_SGMII for SGMII mode, XTE_MII_TYPE_MII for MII or GMII
 *        modes.
 * @param Is1000BaseX (output) is set to TRUE when the device is operating in
 *        1000BaseX mode, FALSE otherwise.
 *
 * @note At this time there is no HW interface available to tell whether MII or
 * GMII is being utilized.
 *
 ******************************************************************************/
void XTemac_GetPhysicalInterface(XTemac * InstancePtr, u8 * MiiTypePtr,
				 u32 * Is1000BaseX)
{
	u32 EcfgReg;

	XASSERT_VOID(InstancePtr != NULL);
	XASSERT_VOID(MiiTypePtr != NULL);
	XASSERT_VOID(Is1000BaseX != NULL);
	XASSERT_VOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	/* Get the current contents of the relevant register */
	EcfgReg = XTemac_mGetHostReg(XTE_ECFG_OFFSET);

	/* Decode the register and return the correct interface types */
	if (EcfgReg & XTE_ECFG_RGMII_MASK) {
		*MiiTypePtr = XTE_MII_TYPE_RGMII;
	} else if (EcfgReg & XTE_ECFG_SGMII_MASK) {
		*MiiTypePtr = XTE_MII_TYPE_SGMII;
	} else {
		*MiiTypePtr = XTE_MII_TYPE_MII;
	}

	if (EcfgReg & XTE_ECFG_1000BASEX_MASK) {
		*Is1000BaseX = TRUE;
	} else {
		*Is1000BaseX = FALSE;
	}
}

/*****************************************************************************/
/**
 * Get the current link speed. When configured for SGMII or RGMII this function
 * returns what the device thinks it is running at. This should be an accurate
 * value.
 *
 * When configured for MII or GMII, this function returns whatever the speed
 * set by XTemac_SetMiiLinkSpeed() is, or if that function has never been called
 * then the HW default is returned. The speed returned may not reflect the
 * actual speed the link is operating at. For better accuracy, the user should
 * query the PHY.
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 *
 * @return Link speed in units of megabits per second
 *
 ******************************************************************************/
u16 XTemac_GetLinkSpeed(XTemac * InstancePtr)
{
	u32 EcfgReg;
	u32 EgmicReg;
	u16 Speed = 10;

	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	/* Get the current contents of the relevant registers */
	EcfgReg = XTemac_mGetHostReg(XTE_ECFG_OFFSET);
	EgmicReg = XTemac_mGetHostReg(XTE_EGMIC_OFFSET);

	/* Decode registers and return the correct speed */
	if (EcfgReg & XTE_ECFG_RGMII_MASK) {
		if (EgmicReg & XTE_EGMIC_RGLINKSPD_1000) {
			Speed = 1000;
		} else if (EgmicReg & XTE_EGMIC_RGLINKSPD_100) {
			Speed = 100;
		}
	} else if (EcfgReg & XTE_ECFG_SGMII_MASK) {
		if (EgmicReg & XTE_EGMIC_SGLINKSPD_1000) {
			Speed = 1000;
		} else if (EgmicReg & XTE_EGMIC_SGLINKSPD_100) {
			Speed = 100;
		}
	} else {
		if (EcfgReg & XTE_ECFG_LINKSPD_1000) {
			Speed = 1000;
		} else if (EcfgReg & XTE_ECFG_LINKSPD_100) {
			Speed = 100;
		}
	}

	return (Speed);
}

/*****************************************************************************/
/**
 * Set the speed of the MAC when operating in MII or GMII modes. The speed
 * must match what the PHY is reporting as the link speed.
 *
 * If the device is configured for SGMII or RGMII then the speed is set
 * implicitly by the device and cannot be changed.
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 * @param Speed is the speed to set in units of Mbps. Valid values are 10, 100,
 *        or 1000
 *
 * @return
 *   - XST_SUCCESS if the speed was set.
 *   - XST_FAULURE if the speed was not set because the device is configured
 *     for SGMII or RGMII.
 *
 ******************************************************************************/
XStatus XTemac_SetMiiLinkSpeed(XTemac * InstancePtr, u16 Speed)
{
	u32 EcfgReg;

	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);
	XASSERT_NONVOID((Speed == 10) || (Speed == 100) || (Speed == 1000));

	/* Get the current contents of the EMAC config register  */
	EcfgReg = XTemac_mGetHostReg(XTE_ECFG_OFFSET);

	/* RGMII or SGMII? */
	if (EcfgReg & (XTE_ECFG_RGMII_MASK | XTE_ECFG_SGMII_MASK)) {
		return (XST_FAILURE);
	}

	/* Clear out link speed bits (resets speed to 10mbps) */
	EcfgReg &= ~XTE_ECFG_LINKSPD_MASK;

	switch (Speed) {
	case 10:
		break;

	case 100:
		EcfgReg |= XTE_ECFG_LINKSPD_100;
		break;

	case 1000:
		EcfgReg |= XTE_ECFG_LINKSPD_1000;
		break;
	}

	/* Set register and return */
	XTemac_mSetHostReg(XTE_ECFG_OFFSET, EcfgReg);
	return (XST_SUCCESS);
}

/*****************************************************************************/
/**
 * Get the current state of the link when media interface is of the RGMII type
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 * @param LinkPtr is a return value set to either XTE_RGMII_LINK_DOWN or
 *        XTE_RGMII_LINK_UP.
 * @param DuplexPtr is a returne value set to either XTE_RGMII_DUPLEX_HALF or
 *        XTE_RGMII_DUPLEX_FULL.
 *
 * @return
 *   - XST_SUCCESS if the RGMII status was read and return values set.
 *   - XST_FAILURE if the device is not using RGMII.
 *
 ******************************************************************************/
XStatus XTemac_GetRgmiiStatus(XTemac * InstancePtr, u8 * LinkPtr,
			      u8 * DuplexPtr)
{
	u32 EcfgReg;
	u32 EgmicReg;

	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(LinkPtr != NULL);
	XASSERT_NONVOID(DuplexPtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	/* Get the current contents of the relevant registers */
	EcfgReg = XTemac_mGetHostReg(XTE_ECFG_OFFSET);
	EgmicReg = XTemac_mGetHostReg(XTE_EGMIC_OFFSET);

	/* Decode registers and return the correct link attributes */
	if (EcfgReg & XTE_ECFG_RGMII_MASK) {
		if (EgmicReg & XTE_EGMIC_RGSTATUS_MASK) {
			*LinkPtr = XTE_RGMII_LINK_UP;
		} else {
			*LinkPtr = XTE_RGMII_LINK_DOWN;
		}

		if (EgmicReg & XTE_EGMIC_RGHALFDUPLEX_MASK) {
			*DuplexPtr = XTE_RGMII_DUPLEX_HALF;
		} else {
			*DuplexPtr = XTE_RGMII_DUPLEX_FULL;
		}

		return (XST_SUCCESS);
	}

	/* Not RGMII */
	return (XST_FAILURE);
}

/*****************************************************************************/
/**
 * Set the MDIO clock divisor. This function must be called once after each
 * reset prior to accessing MII PHY registers.
 *
 * Calculating the divisor:
 *
 * From the Virtex-4 Embedded Tri-Mode Ethernet MAC User's Guide, the
 * following equation governs the MDIO clock to the PHY:
 *
 * <pre>
 *              f[HOSTCLK]
 *   f[MDC] = -----------------
 *            (1 + Divisor) * 2
 * </pre>
 *
 * where f[HOSTCLK] is the bus clock frequency in MHz, and f[MDC] is the
 * MDIO clock frequency in MHz to the PHY. Typically, f[MDC] should not
 * exceed 2.5 MHz. Some PHYs can tolerate faster speeds which means faster
 * access.
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 * @param Divisor is the divisor to set. Range is 0 to XTE_EMC_CLK_DVD_MAX.
 *
 ******************************************************************************/
void XTemac_PhySetMdioDivisor(XTemac * InstancePtr, u8 Divisor)
{
	XASSERT_VOID(InstancePtr != NULL);
	XASSERT_VOID(InstancePtr->IsReady == XCOMPONENT_IS_READY)
	    XASSERT_VOID(Divisor <= XTE_EMC_CLK_DVD_MAX);

	XTemac_mSetHostReg(XTE_EMC_OFFSET, (u32) Divisor | XTE_EMC_MDIO_MASK);
}

/*****************************************************************************/
/*
*
* Read the current value of the PHY register indicated by the PhyAddress and
* the RegisterNum parameters. The MAC provides the driver with the ability to
* talk to a PHY that adheres to the Media Independent Interface (MII) as
* defined in the IEEE 802.3 standard.
*
* Prior to PHY access with this function, the user should have setup the MDIO
* clock with XTemac_PhyEnableAccess().
*
* @param InstancePtr is a pointer to the XTemac instance to be worked on.
* @param PhyAddress is the address of the PHY to be read (supports multiple
*        PHYs)
* @param RegisterNum is the register number, 0-31, of the specific PHY register
*        to read
* @param PhyDataPtr is an output parameter, and points to a 16-bit buffer into
*        which the current value of the register will be copied.
*
* @return
*
* - XST_SUCCESS if the PHY was read from successfully
* - XST_NO_FEATURE if the device is not configured with MII support
* - XST_EMAC_MII_BUSY if there is another PHY operation in progress
* - XST_EMAC_MII_READ_ERROR if a read error occurred between the MAC and the PHY
*
* @note
*
* This function is not thread-safe. The user must provide mutually exclusive
* access to this function if there are to be multiple threads that can call it.
* <br><br>
* There is the possibility that this function will not return if the hardware
* is broken (i.e., it never sets the status bit indicating that the read is
* done). If this is of concern to the user, the user should provide protection
* from this problem - perhaps by using a different timer thread to monitor the
* PhyRead thread.
*
******************************************************************************/
XStatus XTemac_PhyRead(XTemac * InstancePtr, u32 PhyAddress,
		       u32 RegisterNum, u16 * PhyDataPtr)
{
	XASSERT_NONVOID(InstancePtr != NULL);

	/* HW doesn't support PHY access yet */
	return (XST_NO_FEATURE);
}

/*****************************************************************************/
/*
* Write data to the specified PHY register. The Ethernet driver does not
* require the device to be stopped before writing to the PHY.  Although it is
* probably a good idea to stop the device, it is the responsibility of the
* application to deem this necessary. The MAC provides the driver with the
* ability to talk to a PHY that adheres to the Media Independent Interface
* (MII) as defined in the IEEE 802.3 standard.
*
* Prior to PHY access with this function, the user should have setup the MDIO
* clock with XTemac_PhyEnableAccess().
*
* @param InstancePtr is a pointer to the XTemac instance to be worked on.
* @param PhyAddress is the address of the PHY to be written (supports multiple
*        PHYs)
* @param RegisterNum is the register number, 0-31, of the specific PHY register
*        to write
* @param PhyData is the 16-bit value that will be written to the register
*
* @return
*
* - XST_SUCCESS if the PHY was written to successfully. Since there is no error
*   status from the MAC on a write, the user should read the PHY to verify the
*   write was successful.
* - XST_NO_FEATURE if the device is not configured with MII support
* - XST_EMAC_MII_BUSY if there is another PHY operation in progress
*
* @note
*
* This function is not thread-safe. The user must provide mutually exclusive
* access to this function if there are to be multiple threads that can call it.
* <br><br>
* There is the possibility that this function will not return if the hardware
* is broken (i.e., it never sets the status bit indicating that the write is
* done). If this is of concern to the user, the user should provide protection
* from this problem - perhaps by using a different timer thread to monitor the
* PhyWrite thread.
*
******************************************************************************/
XStatus XTemac_PhyWrite(XTemac * InstancePtr, u32 PhyAddress,
			u32 RegisterNum, u16 PhyData)
{
	XASSERT_NONVOID(InstancePtr != NULL);

	/* HW doesn't support PHY access yet */
	return (XST_NO_FEATURE);
}
