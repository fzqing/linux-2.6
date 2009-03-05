/*
   Copyright (C) 2005, Intel Corporation.
   Copyright (C) 2006, Marvell International Ltd.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * Abstract:
 *	contains all OV2630 function prototypes.
 *	Declares no storage.
 *
 * Notes:
 *	Only valid for processor code named Monahans.
 */

#ifndef __MONAHANS_CAM_OV2630_HEADER__
#define __MONAHANS_CAM_OV2630_HEADER__

#include "camera.h"

/*
 * Prototypes
 */

int ov2630_init(p_camera_context_t);

int ov2630_deinit(p_camera_context_t);

int ov2630_sleep(p_camera_context_t camera_context);

int ov2630_wake(p_camera_context_t camera_context);

int ov2630_set_capture_format(p_camera_context_t);

int ov2630_start_capture(p_camera_context_t, unsigned int frames);

int ov2630_stop_capture(p_camera_context_t);

int ov2630_read_8bit(p_camera_context_t camera_context,
		u8 reg_addr,  u8 *reg_val);

int ov2630_write_8bit(p_camera_context_t camera_context,
		u8 reg_addr,  u8 reg_val);

int ov2630_set_power_mode(p_camera_context_t camera_context, u8 mode);

int ov2630_set_contrast(p_camera_context_t camera_context, u8 mode, u32 value);

int ov2630_set_exposure(p_camera_context_t camera_context, u8 mode, u32 value);

int ov2630_set_white_balance(p_camera_context_t camera_context,
		u8 mode, u32 value);
#endif

