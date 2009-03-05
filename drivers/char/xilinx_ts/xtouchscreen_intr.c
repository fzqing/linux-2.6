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
*     (c) Copyright 2004 Xilinx Inc.
*     All rights reserved.
*
*
*     You should have received a copy of the GNU General Public License along
*     with this program; if not, write to the Free Software Foundation, Inc.,
*     675 Mass Ave, Cambridge, MA 02139, USA.
*
******************************************************************************/
/****************************************************************************/
/**
*
* @file xtouchscreen_intr.c
*
* This file contains the functions that are related to interrupt processing
* for the touchscreen driver.
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date     Changes
* ----- ---- -------- -----------------------------------------------
* 1.00a ch   08/15/02 First release
* </pre>
*
*****************************************************************************/

/***************************** Include Files ********************************/

#include "xio.h"
#include "xtouchscreen.h"
#include "xtouchscreen_l.h"

/************************** Constant Definitions ****************************/

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/

/************************** Variable Definitions ****************************/

typedef void (*Handler) (XTouchscreen * InstancePtr);

/************************** Function Prototypes *****************************/

/****************************************************************************/
/**
*
* This function sets the handler that will be called when an event (interrupt)
* occurs in the driver. The purpose of the handler is to allow application
* specific processing to be performed.
*
* @param    InstancePtr is a pointer to the XTouchscreen instance to be
*           worked on.
* @param    FuncPtr is the pointer to the callback function.
* @param    CallBackRef is the upper layer callback reference passed back when
*           the callback function is invoked.
*
* @return
*
* None.
*
* @notes
*
* There is no assert on the CallBackRef since the driver doesn't know what it
* is (nor should it)
*
*****************************************************************************/
void XTouchscreen_SetHandler(XTouchscreen * InstancePtr,
			     XTouchscreen_Handler FuncPtr, void *CallBackRef)
{
	/*
	 * Assert validates the input arguments
	 * CallBackRef not checked, no way to know what is valid
	 */
	XASSERT_VOID(InstancePtr != NULL);
	XASSERT_VOID(FuncPtr != NULL);
	XASSERT_VOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	InstancePtr->Handler = FuncPtr;
	InstancePtr->CallBackRef = CallBackRef;
}

/****************************************************************************/
/**
*
* This function is the interrupt handler for the Touchscreen driver.
* It must be connected to an interrupt system by the user such that it is
* called when an interrupt for any PS/2 port occurs. This function does
* not save or restore the processor context such that the user must
* ensure this occurs.
*
* @param    InstancePtr contains a pointer to the Touchscreen instance.
*
* @return
*
* None.
*
* @note
*
* None.
*
******************************************************************************/
void XTouchscreen_InterruptHandler(XTouchscreen * InstancePtr)
{
	u8 IntrStatus;
	//   u32 x,y,z;

	XASSERT_VOID(InstancePtr != NULL);

	/*
	 * Read the interrupt status register to determine which
	 * interrupt is active
	 */
	IntrStatus = XTouchscreen_mGetIntrStatus(InstancePtr->BaseAddress);

	if (IntrStatus & XTOUCHSCREEN_INT_PEN_DOWN) {
		/*
		 * Check if the pen is alreay down. This is done for debouncing
		 */
		if (!(InstancePtr->CurrentState == XTOUCHSCREEN_STATE_PEN_DOWN)) {
			/*
			 * Call the application handler
			 */
			InstancePtr->Handler(InstancePtr->CallBackRef,
					     XTOUCHSCREEN_EVENT_PEN_DOWN, 0);
		}
		/*
		 * Save the current state
		 */
		InstancePtr->CurrentState = XTOUCHSCREEN_STATE_PEN_DOWN;

		XTouchscreen_mClearIntr(InstancePtr->BaseAddress,
					XTOUCHSCREEN_INT_PEN_DOWN);
	}

	if (IntrStatus & XTOUCHSCREEN_INT_PEN_UP) {
		/*
		 * Check if the pen is alreay up. This is done for debouncing
		 */
		if (!(InstancePtr->CurrentState == XTOUCHSCREEN_STATE_PEN_UP)) {
			/*
			 * Call the application handler
			 */
			InstancePtr->Handler(InstancePtr->CallBackRef,
					     XTOUCHSCREEN_EVENT_PEN_UP, 0);
		}
		/*
		 * Save the current pen state
		 */
		InstancePtr->CurrentState = XTOUCHSCREEN_STATE_PEN_UP;

		XTouchscreen_mClearIntr(InstancePtr->BaseAddress,
					XTOUCHSCREEN_INT_PEN_UP);
	}
}
