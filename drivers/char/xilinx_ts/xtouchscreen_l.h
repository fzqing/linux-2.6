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
* @file xtouchscreen_l.h
*
* This header file contains identifiers and low-level driver functions (or
* macros) that can be used to access the device.  The user should refer to the
* hardware device specification for more details of the device operation.
* High-level driver functions are defined in xtouchscreen.h.
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date     Changes
* ----- ---- -------- -----------------------------------------------
* 1.00a ch   08/08/02 First release
* </pre>
*
*****************************************************************************/

#ifndef XTOUCHSCREEN_L_H	/* prevent circular inclusions */
#define XTOUCHSCREEN_L_H	/* by using protection macros  */

/***************************** Include Files ********************************/

#include "xbasic_types.h"
#include "xio.h"

/************************** Constant Definitions ****************************/

/* Register offsets */
#define XTOUCHSCREEN_CTRL_REG_OFFSET     0
#define XTOUCHSCREEN_INTR_REG_OFFSET     4

/* control register channel selectors */
#define XTOUCHSCREEN_CTRL_CHSEL_X    0xd3
#define XTOUCHSCREEN_CTRL_CHSEL_Y    0x93
#define XTOUCHSCREEN_CTRL_CHSEL_Z1   0xb3
#define XTOUCHSCREEN_CTRL_CHSEL_Z2   0xc3

/* interrupt register positions */
#define XTOUCHSCREEN_INT_PEN_UP      0x01
#define XTOUCHSCREEN_INT_PEN_DOWN    0x02

/* error reading touchscreen */
#define XTOUCHSCREEN_SAMPLE_ERROR     0xffffffff

/* normalization factor */
#define NORM_FACTOR                   1000000

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/

/*****************************************************************************
*
* Low-level driver macros.  The list below provides signatures to help the
* user use the macros.
*
* u32 XTouchscreen_mReadCtrlReg(u32 BaseAddress)
* void XTouchscreen_mWriteCtrlReg(u32 BaseAddress, u8 Value)
*
* u32 XTouchscreen_mGetIntrStatus(u32 BaseAddress)
* void XTouchscreen_mClearIntr(u32 BaseAddress, u32 ClearMask)
*
*****************************************************************************/

/****************************************************************************/
/**
* Read the control register.
*
* @param    BaseAddress contains the base address of the device.
*
* @return   The value read from the register.
*
* @note     None.
*
*****************************************************************************/
#define XTouchscreen_mReadCtrlReg(BaseAddress) \
            XIo_In32((BaseAddress) + XTOUCHSCREEN_CTRL_REG_OFFSET)

/****************************************************************************/
/**
* Write to the control register
*
* @param    BaseAddress contains the base address of the device.
            Value to be written
*
* @return   None.
*
* @note     None.
*
*****************************************************************************/
#define XTouchscreen_mWriteCtrlReg(BaseAddress, Value) \
            XIo_Out8((BaseAddress) + XTOUCHSCREEN_CTRL_REG_OFFSET, Value)

/****************************************************************************/
/**
* Read the interrupt status register.
*
* @param    BaseAddress contains the base address of the device.
*
* @return   The value read from the register.
*
* @note     None.
*
*****************************************************************************/
#define XTouchscreen_mGetIntrStatus(BaseAddress) \
            XIo_In32((BaseAddress) + XTOUCHSCREEN_INTR_REG_OFFSET)

/****************************************************************************/
/**
* Clear pending interrupts.
*
* @param    BaseAddress contains the base address of the device.
*           Bitmask for interrupts to be cleared. A "1" clears the interrupt.
*
* @return   None.
*
* @note     None.
*
*****************************************************************************/
#define XTouchscreen_mClearIntr(BaseAddress, ClearMask) \
            XIo_Out32((BaseAddress) + XTOUCHSCREEN_INTR_REG_OFFSET, ClearMask)

/************************** Variable Definitions ****************************/

/************************** Function Prototypes *****************************/

u32 XTouchscreen_GetValue(u32 BaseAddress, u8 Channel);

/****************************************************************************/

#endif
