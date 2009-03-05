/*
 * Copyright (C) 2007 Texas Instruments Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <media/davinci/davinci_enc.h>
#include <media/davinci/davinci_platform.h>

/* Function: davinci_enc_set_mode_platform
 * @channel: channel number, starting index 0.
 * @mgr: encoder device structure
 * Returns: None
 *
 * Description:
 *   This function does platform specific settings for the current mode.
 *
 */
struct enc_config davinci_enc_default[DAVINCI_ENC_MAX_CHANNELS] = {
	{VID_ENC_OUTPUT_COMPOSITE,
	 VID_ENC_STD_NTSC}
};
EXPORT_SYMBOL(davinci_enc_default);

char *davinci_outputs[] = {
	VID_ENC_OUTPUT_COMPOSITE,
	VID_ENC_OUTPUT_COMPOSITE1,
	VID_ENC_OUTPUT_SVIDEO,
	VID_ENC_OUTPUT_SVIDEO1,
	VID_ENC_OUTPUT_COMPONENT,
	VID_ENC_OUTPUT_COMPONENT1,
	VID_ENC_OUTPUT_LCD,
	VID_ENC_OUTPUT_LCD1,
	""
};
EXPORT_SYMBOL(davinci_outputs);

char *davinci_modes[] = {
	VID_ENC_STD_NTSC,
	VID_ENC_STD_NTSC_RGB,
	VID_ENC_STD_PAL,
	VID_ENC_STD_PAL_RGB,
	VID_ENC_STD_720P_25,
	VID_ENC_STD_720P_30,
	VID_ENC_STD_720P_50,
	VID_ENC_STD_720P_60,
	VID_ENC_STD_1080I_25,
	VID_ENC_STD_1080I_30,
	VID_ENC_STD_1080P_24,
	VID_ENC_STD_1080P_25,
	VID_ENC_STD_1080P_30,
	VID_ENC_STD_1080P_50,
	VID_ENC_STD_1080P_60,
	VID_ENC_STD_480P_60,
	VID_ENC_STD_576P_50,
	VID_ENC_STD_640x480,
	VID_ENC_STD_640x400,
	VID_ENC_STD_640x350,
	""
};
EXPORT_SYMBOL(davinci_modes);

void davinci_enc_set_mode_platform(int channel, struct vid_enc_device_mgr *mgr)
{
}
EXPORT_SYMBOL(davinci_enc_set_mode_platform);

/*
 * davinci_enc_set_display_timing
 */
/* This function sets the display timing from the fb_info structure*/
void davinci_enc_set_display_timing(struct vid_enc_mode_info *mode)
{
};
EXPORT_SYMBOL(davinci_enc_set_display_timing);

