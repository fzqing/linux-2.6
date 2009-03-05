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
/*****************************************************************************/
/**
*
* @file xtouchscreen.h
*
* This driver supports the following features:
*
* - Polled mode
* - Interrupt driven mode
*
* <b>Interrupts</b>
*
* In order to use interrupts, it is necessary for the user to connect the
* driver interrupt handler, XTouchscreen_InterruptHandler(), to the interrupt
* system of the application. This function does not save and restore the
* processor context such that the user must provide it. A handler must be set
* for the driver such that the handler is called when interrupt events occur.
* The handler is called from interrupt context and is designed to allow
* application specific processing to be performed.
*
* @note
*
* None.
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date     Changes
* ----- ---- -------- -----------------------------------------------
* 1.00a ch   08/15/02 First release
* </pre>
*
******************************************************************************/

#ifndef XTOUCHSCREEN_H		/* prevent circular inclusions */
#define XTOUCHSCREEN_H		/* by using protection macros */

/***************************** Include Files ********************************/

#include "xbasic_types.h"
#include "xstatus.h"
#include "xtouchscreen_l.h"

/************************** Constant Definitions ****************************/

/*
 * These constants specify the handler events that are passed to
 * a handler from the driver. These constants are not bit masks suuch that
 * only one will be passed at a time to the handler
 */
#define XTOUCHSCREEN_EVENT_PEN_UP       1
#define XTOUCHSCREEN_EVENT_PEN_DOWN     2

/*
 * These constants are used to specify the current pen state.
 * They are only used internally.
 */
#define XTOUCHSCREEN_STATE_PEN_UP       1
#define XTOUCHSCREEN_STATE_PEN_DOWN     2

/**************************** Type Definitions ******************************/

/*
 * This typedef contains configuration information for the device
 */
typedef struct {
	u16 DeviceId;
	u32 BaseAddress;
} XTouchscreen_Config;

/*
 * This data type defines a handler which the application must define
 * when using interrupt mode.  The handler will be called from the driver in an
 * interrupt context to handle application specific processing.
 *
 * @param CallBackRef is a callback reference passed in by the upper layer
 *        when setting the handler, and is passed back to the upper layer when
 *        the handler is called.
 * @param Event contains one of the event constants indicating why the handler
 *        is being called.
 * @param EventData contains the number of bytes sent or received at the time
*         of the call.
*/
typedef void (*XTouchscreen_Handler) (void *CallBackRef, u32 Event,
				      unsigned int EventData);
/*
 * The Touchscreen driver instance data. The user is required to allocate a
 * variable of this type for every touchscreen device in the system.
 * A pointer to a variable of this type is then passed to the driver API
 * functions
 */
typedef struct {
	u32 BaseAddress;
	u32 IsReady;
	u32 CurrentState;
	XTouchscreen_Handler Handler;
	void *CallBackRef;
} XTouchscreen;

/***************** Macros (Inline Functions) Definitions ********************/

/************************** Function Prototypes *****************************/

/*
 * required functions is xtouchscreen.c
 */
XStatus XTouchscreen_Initialize(XTouchscreen * InstancePtr, u16 DeviceId);
XTouchscreen_Config *XTouchscreen_LookupConfig(u16 DeviceId);
void XTouchscreen_GetPosition_2D(XTouchscreen * InstancePtr, u32 * x, u32 * y);
void XTouchscreen_GetPosition_3D(XTouchscreen * InstancePtr, u32 * x,
				 u32 * y, u32 * z);
/*
 * interrupt functions in xtouchscreen_intr.c
 */
void XTouchscreen_SetHandler(XTouchscreen * InstancePtr,
			     XTouchscreen_Handler FuncPtr, void *CallBackRef);
void XTouchscreen_InterruptHandler(XTouchscreen * InstancePtr);

#endif
