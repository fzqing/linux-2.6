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
/* vpif.c */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <media/davinci/vpif.h>

#define VPIF_CH0_MAX_MODES	(22)
#define VPIF_CH1_MAX_MODES	2
#define VPIF_CH2_MAX_MODES	15
#define VPIF_CH3_MAX_MODES	2

/* This structure is used to keep track of VPIF size register's offsets */
struct vpif_registers {
	u32 h_cfg, v_cfg_00, v_cfg_01, v_cfg_02, v_cfg, ch_ctrl;
	u32 line_offset, vanc0_strt, vanc0_size, vanc1_strt;
	u32 vanc1_size, width_mask, len_mask;
	u8 max_modes;
};

static struct vpif_registers vpifregs[VPIF_NUM_CHANNELS] = {
	/* Channel0 registers offsets */
	{VPIF_CH0_H_CFG, VPIF_CH0_V_CFG_00, VPIF_CH0_V_CFG_01,
	 VPIF_CH0_V_CFG_02, VPIF_CH0_V_CFG_03, VPIF_CH0_CTRL,
	 VPIF_CH0_IMG_ADD_OFST, 0, 0, 0, 0, 0x1FFF, 0xFFF, VPIF_CH0_MAX_MODES},
	/* Channel1 registers offsets */
	{VPIF_CH1_H_CFG, VPIF_CH1_V_CFG_00, VPIF_CH1_V_CFG_01,
	 VPIF_CH1_V_CFG_02, VPIF_CH1_V_CFG_03, VPIF_CH1_CTRL,
	 VPIF_CH1_IMG_ADD_OFST, 0, 0, 0, 0, 0x1FFF, 0xFFF, VPIF_CH1_MAX_MODES},
	/* Channel2 registers offsets */
	{VPIF_CH2_H_CFG, VPIF_CH2_V_CFG_00, VPIF_CH2_V_CFG_01,
	 VPIF_CH2_V_CFG_02, VPIF_CH2_V_CFG_03, VPIF_CH2_CTRL,
	 VPIF_CH2_IMG_ADD_OFST, VPIF_CH2_VANC0_STRT, VPIF_CH2_VANC0_SIZE,
	 VPIF_CH2_VANC1_STRT, VPIF_CH2_VANC1_SIZE, 0x7FF, 0x7FF,
	 VPIF_CH2_MAX_MODES},
	/* Channel3 registers offsets */
	{VPIF_CH3_H_CFG, VPIF_CH3_V_CFG_00, VPIF_CH3_V_CFG_01,
	 VPIF_CH3_V_CFG_02, VPIF_CH3_V_CFG_03, VPIF_CH3_CTRL,
	 VPIF_CH3_IMG_ADD_OFST, VPIF_CH3_VANC0_STRT, VPIF_CH3_VANC0_SIZE,
	 VPIF_CH3_VANC1_STRT, VPIF_CH3_VANC1_SIZE, 0x7FF, 0x7FF,
	 VPIF_CH3_MAX_MODES}
};

int vpif_channel_getfid(u8 channel_id)
{
	return (((regr(vpifregs[channel_id].ch_ctrl) & VPIF_CH_FID_MASK) >>
		 VPIF_CH_FID_SHIFT));
}

EXPORT_SYMBOL(vpif_channel_getfid);

#define VPIF_SD_PARAMS \
	{"NTSC", 720, 480, 30, 0, 1, 268, 1440, 1, 23, 263, 266, \
	 286, 525, 525, 0, 1, 0}, \
	{"PAL", 720, 576, 25, 0, 1, 280, 1440, 1, 23, 311, 313, \
	 336, 624, 625, 0, 1, 0}

#define VPIF_SD_16BIT_OUTPUT_PARAMS \
        {"NTSC-16Bit", 720, 480, 30, 0, 1, 268, 1440, 1, 23, 263, 266, \
	 286, 525, 525, 0, 0, 0}, \
	{"PAL-16Bit", 720, 576, 25, 0, 1, 280, 1440, 1, 23, 311, 313, \
	 336, 624, 625, 0, 0, 0}

#define VPIF_HD_PARAMS	\
	{"720P-60", 1280, 720, 60, 1, 0, 362, 1280, 1, 26, 746, 0, \
	 0, 0, 750, 0, 0, 1}, \
	{"1080I-30", 1920, 1080, 30, 0, 0, 272, 1920, 1, 21, 561, 564, \
	 584, 1124, 1125, 0, 0, 1}, \
	{"1080I-25", 1920, 1080, 25, 0, 0, 712, 1920, 1, 21, 561, 564, \
	 584, 1124, 1125, 0, 0, 1}

#define VPIF_720P_PARAMS \
	{"720P-25", 1280, 720, 25, 1, 0, 2672, 1280, 1, 26, 746, 0, \
	 0, 0, 750, 0, 0, 1}, \
	{"720P-30", 1280, 720, 30, 1, 0, 2012, 1280, 1, 26, 746, 0, \
	 0, 0, 750, 0, 0, 1}, \
	{"720P-50", 1280, 720, 50, 1, 0, 692, 1280, 1, 26, 746, 0, \
	 0, 0, 750, 0, 0, 1}

#define VPIF_1080P_PARAMS  \
	{"1080P-24", 1920, 1080, 24, 1, 0, 822, 1920, 1, 42, 1122, 0, \
	 0, 0, 1125, 0, 0, 1}, \
	{"1080P-25", 1920, 1080, 25, 1, 0, 712, 1920, 1, 42, 1122, 0, \
	 0, 0, 1125, 0, 0, 1}, \
	{"1080P-30", 1920, 1080, 30, 1, 0, 272, 1920, 1, 42, 1122, 0, \
	 0, 0, 1125, 0, 0, 1},

#define VPIF_ED_PARAMS	\
	{"480P-60", 720, 480, 60, 1, 0, 130, 720, 1, 43, 525, \
	 0, 0, 0, 525, 0, 0, 0}, \
	{"576P-50", 720, 576, 50, 1, 0, 136, 720, 1, 45, 621, \
	 0, 0, 0, 625, 0, 0, 0}

#define VPIF_MT9T001_PARAMS \
	{"VGA-30", 640, 480, 30, 1, 0, 2068, 1320, 1, 31, 522, 0, \
	 0, 0, 490, 1, 0, 0}, \
	{"VGA-60", 640, 480, 60, 1, 0, 1432, 1320, 1, 31, 522, 0, \
	 0, 0, 490, 1, 0, 0}, \
	{"SVGA-30", 800, 600, 30, 1, 0, 1432, 1640, 1, 31, 652, 0, \
	 0, 0, 620, 1, 0, 0}, \
	{"SVGA-60", 800, 600, 60, 1, 0, 780, 1640, 1, 31, 652, 0, \
	 0, 0, 620, 1, 0, 0}, \
	{"XGA", 1024, 768, 30, 1, 0, 938, 2080, 1, 283, 806, 0, \
	 0, 0, 776, 1, 0, 0}, \
	{"480P-MT-30", 720, 480, 30, 1, 0, 2090, 1472, 1, 350, 839, 0, \
	 0, 0, 488, 1, 0, 0}, \
	{"480P-MT-60", 720, 480, 60, 1, 0, 1494, 1472, 1, 50, 539, 0, \
	 0, 0, 488, 1, 0, 0}, \
	{"576P-MT-25", 720, 576, 25, 1, 0, 2238, 1472, 1, 450, 1035, 0, \
	 0, 0, 584, 1, 0, 0}, \
	{"576P-MT-50", 720, 576, 50, 1, 0, 1558, 1472, 1, 48, 663, 0, \
	 0, 0, 584, 1, 0, 0}, \
	{"720P-MT-24", 1280, 720, 24, 1, 0, 1338, 2600, 1, 282, 1013, 0, \
	 0, 0, 730, 1, 0, 0}, \
	{"720P-MT-30", 1280, 720, 30, 1, 0, 782, 2600, 1, 220, 951, 0, \
	 0, 0, 730, 1, 0, 0}, \
	{"1080P-MT-18", 1920, 1080, 18, 1, 0, 954, 3840, 1, 31, 1112, 0, \
	 0, 0, 1080, 1, 0, 0}

static struct vpif_channel_config_params ch0_params[VPIF_CH0_MAX_MODES] = {
	VPIF_SD_PARAMS, VPIF_HD_PARAMS, VPIF_MT9T001_PARAMS, VPIF_720P_PARAMS,
	VPIF_ED_PARAMS
};

static struct vpif_channel_config_params ch1_params[VPIF_CH1_MAX_MODES] = {
	VPIF_SD_PARAMS
};
static struct vpif_channel_config_params ch2_params[VPIF_CH2_MAX_MODES] = {
	VPIF_SD_PARAMS, VPIF_HD_PARAMS, VPIF_ED_PARAMS,
	VPIF_SD_16BIT_OUTPUT_PARAMS, VPIF_720P_PARAMS, VPIF_1080P_PARAMS
};

static struct vpif_channel_config_params *vpif_config_params
    [VPIF_NUM_CHANNELS] = {
	ch0_params,
	ch1_params,
	ch2_params,
	ch1_params
};
static u8 irq_vpif_channel[VPIF_NUM_CHANNELS] = { 0, 1, 2, 3 };
int vpif_get_irq_number(int ch_id)
{
	return irq_vpif_channel[ch_id];
}

EXPORT_SYMBOL(vpif_get_irq_number);

/* Function to set vpif parameters */
static void vpif_set_mode_info(u8 index, u8 channel_id, u8 config_channel_id);

/* vpif_set_mode_info: This function is used to set horizontal and
 * vertical config parameters in VPIF registers */
static void vpif_set_mode_info(u8 index, u8 channel_id, u8 config_channel_id)
{
	u32 value;

	/* As per the standard in the channel, configure the values of L1, L3,
	   L5, L7  L9, L11 in VPIF Register */
	/* Write width of the image */
	value = (vpif_config_params[config_channel_id][index].eav2sav &
		 vpifregs[config_channel_id].width_mask);
	value <<= VPIF_CH_LEN_SHIFT;
	value |= (vpif_config_params[config_channel_id][index].sav2eav &
		  vpifregs[config_channel_id].width_mask);

	regw(value, vpifregs[channel_id].h_cfg);

	/* Write the L1 and L3 parameters in VPIF register */
	value = (vpif_config_params[config_channel_id][index].l1 &
		 vpifregs[config_channel_id].len_mask);
	value <<= VPIF_CH_LEN_SHIFT;
	value |= (vpif_config_params[config_channel_id][index].l3 &
		  vpifregs[config_channel_id].len_mask);

	regw(value, vpifregs[channel_id].v_cfg_00);

	/* Write the L5 and L6 parameters in VPIF register */
	value = (vpif_config_params[config_channel_id][index].l5 &
		 vpifregs[config_channel_id].len_mask);
	value <<= VPIF_CH_LEN_SHIFT;
	value |= (vpif_config_params[config_channel_id][index].l7 &
		  vpifregs[config_channel_id].len_mask);

	regw(value, vpifregs[channel_id].v_cfg_01);

	/* Write the L9 and L11 parameters in VPIF register */
	value = (vpif_config_params[config_channel_id][index].l9 &
		 vpifregs[config_channel_id].len_mask);
	value <<= VPIF_CH_LEN_SHIFT;
	value |= (vpif_config_params[config_channel_id][index].l11 &
		  vpifregs[config_channel_id].len_mask);

	regw(value, vpifregs[channel_id].v_cfg_02);

	/* Write the image height in VPIF register */
	value = (vpif_config_params[config_channel_id][index].vsize &
		 vpifregs[config_channel_id].len_mask);
	regw(value, vpifregs[channel_id].v_cfg);
}

/* vpif_set_video_capture_params: This function is used to set video
 * parameters in VPIF register. It sets size parameters, frame format
 * and yc mux mode */
int vpif_set_video_params(struct vpif_params *vpifparams, u8 channel_id)
{
	int index, found = -1, i;
	u8 max_modes = vpifregs[channel_id].max_modes;
	u32 value, ch_nip;
	u8 start, end;

	/* loop on the number of mode supported per channel */
	for (index = 0; index < max_modes; index++) {

		/* If the mode is found, set the parameter in VPIF register */
		if (0 == strcmp(vpif_config_params[channel_id][index].name,
				vpifparams->video_params.name)) {

			found = 1;
			/* Set the size parameteres in the VPIF registers */
			vpif_set_mode_info(index, channel_id, channel_id);

			if (!(vpif_config_params[channel_id][index].ycmux_mode)) {
				vpif_set_mode_info(index,
						   channel_id + 1, channel_id);
				found = 2;
			}
			start = channel_id;
			end = channel_id + found;

			for (i = start; i < end; i++) {
				value = regr(vpifregs[i].ch_ctrl);

				if (channel_id < 2)
					ch_nip = VPIF_CAPTURE_CH_NIP;
				else
					ch_nip = VPIF_DISPLAY_CH_NIP;

				/* Set the frame format in the 
				 * control register */
				if (vpif_config_params[channel_id][index].
				    frm_fmt) {
					/* Progressive Frame Format */
					SETBIT(value, ch_nip);
				} else {
					/* Interlaced Frame Format */
					RESETBIT(value, ch_nip);
				}

				/* Set YC mux mode in the control register */
				if (vpif_config_params[channel_id][index].
				    ycmux_mode) {
					/* YC Mux mode */
					SETBIT(value, VPIF_CH_YC_MUX_BIT);
				} else {
					/* YC Non Mux mode */
					RESETBIT(value, VPIF_CH_YC_MUX_BIT);
				}
				/* Set field/frame mode in control register */
				if (vpifparams->video_params.storage_mode) {
					SETBIT(value,
					       VPIF_CH_INPUT_FIELD_FRAME_BIT);
				} else {
					RESETBIT(value,
						 VPIF_CH_INPUT_FIELD_FRAME_BIT);
				}

				/* Set raster scanning SDR Format */
				RESETBIT(value, VPIF_CH_SDR_FMT_BIT);

				/* Set the capture format */
				if (vpif_config_params[channel_id][index].
				    capture_format) {
					SETBIT(value, VPIF_CH_DATA_MODE_BIT);
				} else {
					RESETBIT(value, VPIF_CH_DATA_MODE_BIT);
				}

				if (channel_id > 1) {
					/* Set the Pixel enable bit */
					SETBIT(value, VPIF_DISPLAY_PIX_EN_BIT);
				} else {
					/* Set the polarity of various pins */
					if (vpif_config_params[channel_id]
					    [index].capture_format) {
						if (vpifparams->params.
						    raw_params.fid_pol) {
							SETBIT(value,
							       VPIF_CH_FID_POLARITY_BIT);
						} else {
							RESETBIT(value,
								 VPIF_CH_FID_POLARITY_BIT);
						}
						if (vpifparams->params.
						    raw_params.vd_pol) {
							SETBIT(value,
							       VPIF_CH_V_VALID_POLARITY_BIT);
						} else {
							RESETBIT(value,
								 VPIF_CH_V_VALID_POLARITY_BIT);
						}
						if (vpifparams->params.
						    raw_params.hd_pol) {
							SETBIT(value,
							       VPIF_CH_H_VALID_POLARITY_BIT);
						} else {
							RESETBIT(value,
								 VPIF_CH_H_VALID_POLARITY_BIT);
						}
						/* Set data width */
						value &=
						    ((~(unsigned int)(0x3)) <<
						     VPIF_CH_DATA_WIDTH_BIT);
						value |=
						    ((vpifparams->params.
						      raw_params.
						      data_sz) <<
						     VPIF_CH_DATA_WIDTH_BIT);
					}
				}
				regw(value, vpifregs[i].ch_ctrl);

				/* Write the pitch in the driver */
				regw((vpifparams->video_params.hpitch),
				     vpifregs[i].line_offset);
			}
			break;
		}
	}
	regw(0x80, VPIF_REQ_SIZE);
	regw(0x01, VPIF_EMULATION_CTRL);
	return found;
}

EXPORT_SYMBOL(vpif_set_video_params);

int vpif_set_vbi_display_params(struct vpif_vbi_params *vbiparams,
				u8 channel_id)
{
	u32 value;

	value = 0x3F8 & (vbiparams->hstart0);
	value |= 0x3FFFFFF & ((vbiparams->vstart0) << 16);
	regw(value, vpifregs[channel_id].vanc0_strt);

	value = 0x3F8 & (vbiparams->hstart1);
	value |= 0x3FFFFFF & ((vbiparams->vstart1) << 16);
	regw(value, vpifregs[channel_id].vanc1_strt);

	value = 0x3F8 & (vbiparams->hsize0);
	value |= 0x3FFFFFF & ((vbiparams->vsize0) << 16);
	regw(value, vpifregs[channel_id].vanc0_size);

	value = 0x3F8 & (vbiparams->hsize1);
	value |= 0x3FFFFFF & ((vbiparams->vsize1) << 16);
	regw(value, vpifregs[channel_id].vanc1_size);

	return 0;
}

EXPORT_SYMBOL(vpif_set_vbi_display_params);

int vpif_get_mode_info(struct vpif_stdinfo *std_info)
{
	int index, found = -1;
	u8 channel_id;
	u8 max_modes;

	if (!std_info)
		return found;

	channel_id = std_info->channel_id;

	if (channel_id != 0 && channel_id != 1 && channel_id != 2 &&
	    channel_id != 3) {
		return found;
	}

	max_modes = vpifregs[channel_id].max_modes;

	/* loop on the number of mode supported per channel */
	for (index = 0; index < max_modes; index++) {

		/* If the mode is found, set the parameter in VPIF register */
		if (0 == strcmp(vpif_config_params[channel_id][index].name,
				std_info->name)) {
			std_info->activelines =
			    vpif_config_params[channel_id][index].height;
			std_info->activepixels =
			    vpif_config_params[channel_id][index].width;
			std_info->fps =
			    vpif_config_params[channel_id][index].fps;
			std_info->frame_format =
			    vpif_config_params[channel_id][index].frm_fmt;
			std_info->ycmux_mode =
			    vpif_config_params[channel_id][index].ycmux_mode;
			std_info->vbi_supported = vpif_config_params
			    [channel_id][index].vbi_supported;
			std_info->hd_sd =
			    vpif_config_params[channel_id][index].hd_sd;
			found = 1;
			break;
		}
	}
	return found;
}

EXPORT_SYMBOL(vpif_get_mode_info);

MODULE_LICENSE("GPL");
