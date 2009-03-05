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
* @file xtouchscreen.c
*
* This file contains the required functions for the touchscreen driver.
* Refer to the header file xtouchscreen.h for more detailed information.
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

#include <asm/delay.h>
#include "xstatus.h"
#include "xtouchscreen.h"
#include "xtouchscreen_i.h"
#include "xtouchscreen_l.h"
#include "xio.h"

/************************** Constant Definitions ****************************/

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/

/************************** Variable Definitions ****************************/

/************************** Function Prototypes *****************************/

static void XTouchscreen_StubHandler(void *CallBackRef, u32 Event,
				     unsigned int ByteCount);

/****************************************************************************/
/**
*
* Initializes a specific touchscreen instance such that it is ready to be used.
* The default operating mode of the driver is polled mode.
*
* @param    InstancePtr is a pointer to the XTouchscreen instance to be
*           worked on.
* @param    DeviceId is the unique id of the device controlled by this
*           XTouchscreen instance. Passing in a device id associates the
*           generic XTouchscreen instance to a specific device, as chosen
*           by the caller or application developer.
*
* @return
*
* - XST_SUCCESS if initialization was successful
* - XST_DEVICE_NOT_FOUND if the device ID could not be found in the
*           configuration table
*
* @note
*
* None.
*
*****************************************************************************/
XStatus XTouchscreen_Initialize(XTouchscreen * InstancePtr, u16 DeviceId)
{
	XTouchscreen_Config *TouchscreenConfigPtr;

	/*
	 * Assert validates the input arguments
	 */
	XASSERT_NONVOID(InstancePtr != NULL);

	/*
	 * Lookup the device configuration in the temporary CROM table. Use this
	 * configuration info down below when initializing this component.
	 */
	TouchscreenConfigPtr = XTouchscreen_LookupConfig(DeviceId);

	if (TouchscreenConfigPtr == (XTouchscreen_Config *) NULL) {
		return XST_DEVICE_NOT_FOUND;
	}

	/*
	 * Setup the data that is from the configuration information
	 */
	InstancePtr->BaseAddress = TouchscreenConfigPtr->BaseAddress;

	/*
	 * Initialize the instance data to some default values and setup a default
	 * handler
	 */
	InstancePtr->Handler = XTouchscreen_StubHandler;

	InstancePtr->CurrentState = XTOUCHSCREEN_STATE_PEN_UP;

	/*
	 * Enable Touchscreen
	 */
	XTouchscreen_mWriteCtrlReg(InstancePtr->BaseAddress, 0xd3);
	udelay(100);
	XTouchscreen_mWriteCtrlReg(InstancePtr->BaseAddress, 0xd0);
	udelay(100);

	/*
	 * Clear all interrupts
	 */
	XTouchscreen_mClearIntr(InstancePtr->BaseAddress,
				XTOUCHSCREEN_INT_PEN_DOWN |
				XTOUCHSCREEN_INT_PEN_UP);

	/*
	 * Indicate the instance is now ready to use, initialized without error
	 */
	InstancePtr->IsReady = XCOMPONENT_IS_READY;

	return XST_SUCCESS;
}

/****************************************************************************/
/**
*
* This function reads 2D (X & Y) coordinates from the touchscreen.
*
* @param    InstancePtr is a pointer to the XTouchscreen instance to be
*           worked on.
* @param    *x pointer to store the x coordinate.
*           *y pointer to store the y coordinate.
*
* @return   None.
*
* @note     None.
*
*****************************************************************************/
void XTouchscreen_GetPosition_2D(XTouchscreen * InstancePtr, u32 * x, u32 * y)
{
	*x = XTouchscreen_GetValue(InstancePtr->BaseAddress,
				   XTOUCHSCREEN_CTRL_CHSEL_X);
	*y = XTouchscreen_GetValue(InstancePtr->BaseAddress,
				   XTOUCHSCREEN_CTRL_CHSEL_Y);
}

/****************************************************************************/
/**
*
* This function reads 3D (X, Y & Z) coordinates from the touchscreen. x and
* y are the actual positions, the pressure has to be calculated from z1 and
* z2
*
* @param    InstancePtr is a pointer to the XTouchscreen instance to be
*           worked on.
* @param    *x pointer to store the x coordinate.
*           *y pointer to store the y coordinate.
*           *z pointer to store the z coordinate (pressure)
*
* @return   None.
*
* @note     None.
*
*****************************************************************************/
void XTouchscreen_GetPosition_3D(XTouchscreen * InstancePtr, u32 * x,
				 u32 * y, u32 * z)
{
	u32 z1;
	u32 z2;

   /**x = NORM_FACTOR - XTouchscreen_GetValue(InstancePtr->BaseAddress,
      XTOUCHSCREEN_CTRL_CHSEL_X);*/
	*x = XTouchscreen_GetValue(InstancePtr->BaseAddress,
				   XTOUCHSCREEN_CTRL_CHSEL_X);
	*y = XTouchscreen_GetValue(InstancePtr->BaseAddress,
				   XTOUCHSCREEN_CTRL_CHSEL_Y);

	z1 = XTouchscreen_GetValue(InstancePtr->BaseAddress,
				   XTOUCHSCREEN_CTRL_CHSEL_Z1);
	z2 = XTouchscreen_GetValue(InstancePtr->BaseAddress,
				   XTOUCHSCREEN_CTRL_CHSEL_Z2);

	if (z1 == 0) {
		*z = XTOUCHSCREEN_SAMPLE_ERROR;
		return;
	} else if (z1 > z2) {
		*z = XTOUCHSCREEN_SAMPLE_ERROR;
		return;
	} else {
		*z = ((*x) * (z2 / z1 - 1));
	}
}

/****************************************************************************/
/**
*
* This function is a stub handler that is the default handler such that if the
* application has not set the handler when interrupts are enabled, this
* function will be called. The function interface has to match the interface
* specified for a handler even though none of the arguments are used.
*
* @param    CallBackRef is unused by this function.
* @param    Event is unused by this function.
* @param    ByteCount is unused by this function.
*
* @return
*
* None.
*
* @note
*
* None.
*
*****************************************************************************/
static void XTouchscreen_StubHandler(void *CallBackRef, u32 Event,
				     unsigned int ByteCount)
{
	XASSERT_VOID(FALSE);
}
