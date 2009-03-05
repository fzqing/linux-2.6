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
* @file xtemac.c
*
* The XTemac driver. Functions in this file are the minimum required functions
* for this driver. See xtemac.h for a detailed description of the driver.
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

/************************** Function Prototypes ******************************/

void XTemac_StubHandler(void);	/* Default handler routine */
static void InitHw(XTemac * InstancePtr);	/* HW reset */
static XStatus Initialize(XTemac * InstancePtr);

/************************** Variable Definitions *****************************/

/*****************************************************************************/
/**
* Initialize a specific XTemac instance/driver for systems that use address
* translation. This function performs the same initialization steps as does
* XTemac_Initialize() except for how the base address of the device derived.
* Instead of using the physical address found in the configuration table for
* the given device ID, the supplied virtual address is used.
*
* For systems that do not use address translation, use XTemac_Initialize()
* instead of this one to initialize a device instance.
*
* See XTemac_Initialize() for further information on what is done during
* initialization.
*
* @param InstancePtr is a pointer to the instance to be worked on.
* @param DeviceId is the unique id of the device controlled by this XTemac
*        instance.  Passing in a device id associates the generic XTemac
*        instance to a specific device, as chosen by the caller or application
*        developer.
* @param VirtualAddress is the virtual base address of the device if address
*        translation is being utilized. This address must translate to the
*        physical address as defined in the XTemac_ConfigTable array for the
*        given DeviceId. The translation must be setup by the user prior to
*        calling this function.
*
* @return
* - XST_SUCCESS if initialization was successful
* - XST_DEVICE_NOT_FOUND if device configuration information was not found for
*   a device with the supplied device ID.
* - XST_FAILURE if initialization of packet FIFOs or DMA channels failed.
*
******************************************************************************/
XStatus XTemac_VmInitialize(XTemac * InstancePtr, u16 DeviceId,
			    u32 VirtualAddress)
{
	/* Verify arguments */
	XASSERT_NONVOID(InstancePtr != NULL);

	/* Clear instance memory */
	memset(InstancePtr, 0, sizeof(XTemac));

	/* Lookup the configuration information for this device */
	InstancePtr->Config = XTemac_LookupConfig(DeviceId);
	if (InstancePtr->Config == NULL) {
		return (XST_DEVICE_NOT_FOUND);
	}

	/* Setup the device's base address */
	InstancePtr->BaseAddress = VirtualAddress;

	/* Call local function to perform rest of init */
	return (Initialize(InstancePtr));
}

/*****************************************************************************/
/**
* Initialize a specific XTemac instance/driver. The initialization entails:
* - Initialize fields of the XTemac structure
* - Initialize the IPIF component with its register base address
* - Configure the FIFO components with their register base addresses.
* - If the device is configured with DMA, configure the DMA channel components
*   with their register base addresses. At some later time, memory pools for
*   the scatter-gather descriptor lists may be passed to the driver.
* - Reset the MAC hardware
*
* The PHY is setup independently from the TEMAC. Use the MII or whatever other
* interface may be present for setup.
*
* For systems that do use address translation, use XTemac_VmInitialize()
* instead of this one to initialize a device instance.
*
* @param InstancePtr is a pointer to the instance to be worked on.
* @param DeviceId is the unique id of the device controlled by this XTemac
*        instance.  Passing in a device id associates the generic XTemac
*        instance to a specific device, as chosen by the caller or application
*        developer.
*
* @return
* - XST_SUCCESS if initialization was successful
* - XST_DEVICE_NOT_FOUND if device configuration information was not found for
*   a device with the supplied device ID.
* - XST_FAILURE if initialization of packet FIFOs or DMA channels failed.
*
******************************************************************************/
XStatus XTemac_Initialize(XTemac * InstancePtr, u16 DeviceId)
{
	/* Verify arguments */
	XASSERT_NONVOID(InstancePtr != NULL);

	/* Clear instance memory */
	memset(InstancePtr, 0, sizeof(XTemac));

	/* Lookup the device configuration */
	InstancePtr->Config = XTemac_LookupConfig(DeviceId);
	if (InstancePtr->Config == NULL) {
		return (XST_DEVICE_NOT_FOUND);
	}

	/* Initialize the device register base addresses */
	InstancePtr->BaseAddress = InstancePtr->Config->BaseAddress;

	/* Call local function to perform rest of init */
	return (Initialize(InstancePtr));
}

/*****************************************************************************/
/**
* Start the Ethernet controller as follows:
*   - Enable transmitter if XTE_TRANSMIT_ENABLE_OPTION is set
*   - Enable receiver if XTE_RECEIVER_ENABLE_OPTION is set
*   - If not polled mode, then start the SG DMA send and receive channels (if
*     configured) and enable the global device interrupt.
*
* If starting for the first time after calling XTemac_Initialize() or
* XTemac_Reset(), interrupts will not be generated until one or more of
* XTemac_IntrFifoEnable(), XTemac_IntrFifoDmaEnable(), or
* XTemac_IntrSgEnable() are called. Otherwise, interrupt settings made by
* these functions will be restored.
*
* @param InstancePtr is a pointer to the instance to be worked on.
*
* @return
* - XST_SUCCESS if the device was started successfully
* - XST_DMA_SG_NO_LIST if configured for scatter-gather DMA and a descriptor
*   list has not yet been created for the send or receive channel
* - XST_DMA_SG_LIST_EMPTY if configured for scatter-gather DMA but no
*   receive buffer descriptors have been initialized.
*
* @note
* The driver tries to match the hardware configuration. So if the hardware
* is configured with scatter-gather DMA, the driver expects to start the
* scatter-gather channels and expects that the user has previously set up
* the buffer descriptor lists.
*
* This function makes use of internal resources that are shared between the
* Start, Stop, and Set/ClearOptions functions. So if one task might be setting
* device options while another is trying to start the device, the user is
* required to provide protection of this shared data (typically using a
* semaphore).
*
* This function must not be preempted by an interrupt that may service the
* device.
*
******************************************************************************/
XStatus XTemac_Start(XTemac * InstancePtr)
{
	u32 Reg;
	XStatus Result;

	/* Assert bad arguments and conditions */
	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	/* If already started, then there is nothing to do */
	if (InstancePtr->IsStarted == XCOMPONENT_IS_STARTED) {
		return (XST_SUCCESS);
	}

	/* Start SG DMA (only if not in polled mode) */
	if (XTemac_mIsSgDma(InstancePtr) &&
	    ((InstancePtr->Options & XTE_POLLED_OPTION) == 0)) {
		/* When starting the DMA channels, both transmit and receive sides
		 * need an initialized BD list.
		 */
		Result = XDmaV2_SgStart(&InstancePtr->RecvDma);
		if (Result == XST_DMA_SG_NO_LIST) {
			return (Result);
		}

		Result = XDmaV2_SgStart(&InstancePtr->SendDma);
		if (Result == XST_DMA_SG_NO_LIST) {
			return (Result);
		}
	}

	/* Enable transmitter if not already enabled */
	if (InstancePtr->Options & XTE_TRANSMITTER_ENABLE_OPTION) {
		Reg = XTemac_mGetHostReg(XTE_ETXC_OFFSET);
		if (!(Reg & XTE_ETXC_TXEN_MASK)) {
			XTemac_mSetHostReg(XTE_ETXC_OFFSET,
					   Reg | XTE_ETXC_TXEN_MASK);
		}
	}

	/* Enable receiver? */
	if (InstancePtr->Options & XTE_RECEIVER_ENABLE_OPTION) {
		Reg =
		    XTemac_mGetHostReg(XTE_ERXC1_OFFSET) | XTE_ERXC1_RXEN_MASK;
		XTemac_mSetHostReg(XTE_ERXC1_OFFSET, Reg);
	}

	/* Mark as started */
	InstancePtr->IsStarted = XCOMPONENT_IS_STARTED;

	/* Allow interrupts (if not in polled mode) and exit */
	if ((InstancePtr->Options & XTE_POLLED_OPTION) == 0) {
		XTemac_mSetIpifReg(XTE_DGIE_OFFSET, XTE_DGIE_ENABLE_MASK);
	}

	return (XST_SUCCESS);
}

/*****************************************************************************/
/**
* Gracefully stop the Ethernet MAC as follows:
*   - Disable all interrupts from this device
*   - Stop DMA channels (if configured)
*   - Disable the receiver
*
* Device options currently in effect are not changed.
*
* This function will disable all interrupts by clearing the global interrupt
* enable bit. Any interrupts settings that had been enabled through
* XTemac_IntrFifoEnable(), XTemac_IntrFifoDmaEnable(), or
* XTemac_IntrSgEnable() will be restored when XTemac_Start() is called.
*
* Since the transmitter is not disabled, frames currently in the packet FIFO
* or in process by the SGDMA engine are allowed to be transmitted. XTemac API
* functions that place new data in the packet FIFOs will not be allowed to do
* so until XTemac_Start() is called.
*
* @param InstancePtr is a pointer to the instance to be worked on.
*
* @note
* This function makes use of internal resources that are shared between the
* Start, Stop, SetOptions, and ClearOptions functions. So if one task might be
* setting device options while another is trying to start the device, the user
* is required to provide protection of this shared data (typically using a
* semaphore).
*
* Stopping the DMA channels may cause this function to block until the DMA
* operation is complete. This function will not block waiting for frame data to
* to exit the packet FIFO to the transmitter.
*
******************************************************************************/
void XTemac_Stop(XTemac * InstancePtr)
{
	volatile u32 Reg;

	XASSERT_VOID(InstancePtr != NULL);
	XASSERT_VOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	/* If already stopped, then there is nothing to do */
	if (InstancePtr->IsStarted == 0) {
		return;
	}

	/* Disable interrupts */
	XTemac_mSetIpifReg(XTE_DGIE_OFFSET, 0);

	/* Stop Simple/SG DMA (unless in polled mode) */
	if (!(InstancePtr->Options & XTE_POLLED_OPTION)) {
		/* For simple DMA, wait for the DMA channel's BUSY bit to clear */
		if (XTemac_mIsSimpleDma(InstancePtr)) {
			do {
				Reg = XDmaV2_mGetStatus(&InstancePtr->SendDma);
			} while (Reg & XDMAV2_DMASR_DMABSY_MASK);

			do {
				Reg = XDmaV2_mGetStatus(&InstancePtr->RecvDma);
			} while (Reg & XDMAV2_DMASR_DMABSY_MASK);

		}

		/* For SGDMA, use the DMA driver function to stop the channels */
		else if (XTemac_mIsSgDma(InstancePtr)) {
			(void)XDmaV2_SgStop(&InstancePtr->SendDma);
			(void)XDmaV2_SgStop(&InstancePtr->RecvDma);
		}
	}

	/* Disable the receiver */
	Reg = XTemac_mGetHostReg(XTE_ERXC1_OFFSET);
	Reg &= ~XTE_ERXC1_RXEN_MASK;
	XTemac_mSetHostReg(XTE_ERXC1_OFFSET, Reg);

	/* Stopping the receiver in mid-packet causes a dropped packet indication
	 * from HW. Clear it.
	 */
	Reg = XTemac_mGetIpifReg(XTE_IPISR_OFFSET);
	if (Reg & XTE_IPXR_RECV_REJECT_MASK) {
		XTemac_mSetIpifReg(XTE_IPISR_OFFSET, XTE_IPXR_RECV_REJECT_MASK);
	}

	/* Mark as stopped */
	InstancePtr->IsStarted = 0;
}

/*****************************************************************************/
/**
* Perform a graceful reset of the Ethernet MAC. Resets the DMA channels, the
* FIFOs, the transmitter, and the receiver.
*
* All options are placed in their default state. Any frames in the scatter-
* gather descriptor lists will remain in the lists. The side effect of doing
* this is that after a reset and following a restart of the device, frames that
* were in the list before the reset may be transmitted or received.
*
* The upper layer software is responsible for re-configuring (if necessary)
* and restarting the MAC after the reset. Note also that driver statistics
* are not cleared on reset. It is up to the upper layer software to clear the
* statistics if needed.
*
* When a reset is required due to an internal error, the driver notifies the
* upper layer software of this need through the ErrorHandler callback and
* specific status codes.  The upper layer software is responsible for calling
* this Reset function and then re-configuring the device.
*
* @param InstancePtr is a pointer to the instance to be worked on.
*
* @internal
*
* The reset is accomplished by setting the IPIF reset register.  This takes
* care of resetting all hardware blocks, including the MAC.
*
******************************************************************************/
void XTemac_Reset(XTemac * InstancePtr)
{
	XASSERT_VOID(InstancePtr != NULL);
	XASSERT_VOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	/* Stop the device and reset HW */
	XTemac_Stop(InstancePtr);
	InitHw(InstancePtr);

	/* Set default options */
	InstancePtr->Options = XTE_DEFAULT_OPTIONS;
	XTemac_SetOptions(InstancePtr, InstancePtr->Options);
	XTemac_ClearOptions(InstancePtr, ~InstancePtr->Options);
}

/*****************************************************************************
* Finish off instance initialization started by XTemac_Initialize or
* XTemac_VmInitialize().
*
* The two functions mentioned should have setup the following instance data
* InstancePtr->Config and InstancePtr->BaseAddress.
*
* @param InstancePtr is a pointer to the instance to be worked on.
*
* @return
* - XST_SUCCESS if initialization was successful
* - XST_FAILURE if initialization of packet FIFOs or DMA channels failed.
*
******************************************************************************/
static XStatus Initialize(XTemac * InstancePtr)
{
	XStatus Result;

	/* Set callbacks to an initial stub routine */
	InstancePtr->FifoRecvHandler =
	    (XTemac_FifoRecvHandler) XTemac_StubHandler;
	InstancePtr->FifoSendHandler =
	    (XTemac_FifoSendHandler) XTemac_StubHandler;
	InstancePtr->ErrorHandler = (XTemac_ErrorHandler) XTemac_StubHandler;

	InstancePtr->FifoDmaReadHandler =
	    (XTemac_FifoDmaReadHandler) XTemac_StubHandler;
	InstancePtr->FifoDmaWriteHandler =
	    (XTemac_FifoDmaWriteHandler) XTemac_StubHandler;

	InstancePtr->SgRecvHandler = (XTemac_SgHandler) XTemac_StubHandler;
	InstancePtr->SgSendHandler = (XTemac_SgHandler) XTemac_StubHandler;

	/* Setup and select best processor based transfer method to/from FIFOs */
	Result = XTemac_ConfigureFifoAccess(InstancePtr);
	if (Result != XST_SUCCESS) {
		return (XST_FAILURE);
	}

	/* Setup DMA channels */
	if (XTemac_mIsDma(InstancePtr)) {
		Result = XDmaV2_Initialize(&InstancePtr->RecvDma,
					   InstancePtr->BaseAddress +
					   XTE_DMA_RECV_OFFSET);
		if (Result != XST_SUCCESS) {
			return (XST_FAILURE);
		}

		Result = XDmaV2_Initialize(&InstancePtr->SendDma,
					   InstancePtr->BaseAddress +
					   XTE_DMA_SEND_OFFSET);
		if (Result != XST_SUCCESS) {
			return (XST_FAILURE);
		}
	}

	/* Reset the hardware and set default options */
	InstancePtr->IsReady = XCOMPONENT_IS_READY;
	InstancePtr->Options = XTE_DEFAULT_OPTIONS;
	InitHw(InstancePtr);

	return (XST_SUCCESS);
}

/******************************************************************************
 * Perform one-time setup of HW. The setups performed here only need to occur
 * once after any reset.
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 *
 ******************************************************************************/
static void InitHw(XTemac * InstancePtr)
{
	u32 RegDXR;
	u32 RegETXC;

	/* Reset HW */
	XTemac_mSetIpifReg(XTE_DSR_OFFSET, XTE_DSR_RESET_MASK);

	/* Default IPIF interrupt block enable mask */
	RegDXR = XTE_DXR_CORE_MASK |
	    XTE_DXR_RECV_FIFO_MASK |
	    XTE_DXR_SEND_FIFO_MASK | XTE_DXR_DPTO_MASK | XTE_DXR_TERR_MASK;

	/* Add to default IPIF interrupt block enable mask if DMA is present */
	if (XTemac_mIsDma(InstancePtr)) {
		RegDXR |= (XTE_DXR_RECV_DMA_MASK | XTE_DXR_SEND_DMA_MASK);
	}

	XTemac_mSetIpifReg(XTE_DIER_OFFSET, RegDXR);

	/* Setup SGDMA interupt coalescing defaults */
	if (XTemac_mIsSgDma(InstancePtr)) {
		(void)XTemac_IntrSgCoalSet(InstancePtr, XTE_SEND,
					   XTE_SGDMA_DFT_THRESHOLD,
					   XTE_SGDMA_DFT_WAITBOUND);
		(void)XTemac_IntrSgCoalSet(InstancePtr, XTE_RECV,
					   XTE_SGDMA_DFT_THRESHOLD,
					   XTE_SGDMA_DFT_WAITBOUND);
	}

	/* Interframe gap: The HW has an enable bit to change the IFG through the
	 * XTE_IFGP_OFFSET register. Rather than make the user set this bit then
	 * change the register, simplifiy the process by always setting the
	 * enable bit. All the user needs to do is use XTemac_SetIfg() thereafter.
	 */

	/* The default IFG is 96 bit times. Whatever is in the register adds to that.
	 * By default leave this register at 0 so we have 96 bit times.
	 */
	XTemac_mSetIpifReg(XTE_IFGP_OFFSET, 0);

	/* Now set IFG adjust enable */
	RegETXC = XTemac_mGetHostReg(XTE_ETXC_OFFSET) | XTE_ETXC_TXIFG_MASK;
	XTemac_mSetHostReg(XTE_ETXC_OFFSET, RegETXC);

	/* Set default MDIO divisor */
	XTemac_PhySetMdioDivisor(InstancePtr, XTE_MDIO_DIV_DFT);

	/* Sync default options with HW but leave receiver and transmitter
	 * disabled. They get enabled with XTemac_Start() if XTE_TRANSMITTER_ENABLE-
	 * _OPTION and XTE_RECEIVER_ENABLE_OPTION are set
	 */
	XTemac_SetOptions(InstancePtr, InstancePtr->Options &
			  ~(XTE_TRANSMITTER_ENABLE_OPTION |
			    XTE_RECEIVER_ENABLE_OPTION));

	XTemac_ClearOptions(InstancePtr, ~InstancePtr->Options);
}

/******************************************************************************/
/**
 * This is a stub for the asynchronous callbacks. The stub is here in case the
 * upper layer forgot to set the handler(s). On initialization, all handlers are
 * set to this callback. It is considered an error for this handler to be
 * invoked.
 *
 ******************************************************************************/
void XTemac_StubHandler(void)
{
	XASSERT_VOID_ALWAYS();
}
