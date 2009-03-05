/*
 * drivers/video/smi/smi_hw.c
 *
 * LynxEM+/EM4+(Silicon Motion Inc.) fb driver for NEC Electronics Corporation VR5701 SolutionGearII
 *
 * Author: Sergey Podstavin <spodstavin@ru.mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include "smifb.h"
#include "smi_hw.h"
#include "smi_params.h"

/*
 * set mode registers
 */
void
smi_set_moderegs(struct smifb_info *sinfo,
		 int bpp, int width, int height,
		 int hDisplaySize,
		 int hDisplay, int hStart, int hEnd, int hTotal,
		 int vDisplay, int vStart, int vEnd, int vTotal,
		 int dotClock, int sync)
{
	int i;
	int tmp_mode = SMI_DEFAULT_MODE;
	int lineLength;
	struct smi_mode_regs curMode;

	pr_debug("smi_set_moderegs");
	pr_debug("bpp = %d, width = %d, height = %d\n", bpp, width, height);
	pr_debug("hDisplaySize = %d\n", hDisplaySize);
	pr_debug("hDisplay = %d, hStart = %d, hEnd = %d, hTotal = %d\n",
		 hDisplay, hStart, hEnd, hTotal);
	pr_debug("vDisplay = %d, vStart = %d, vEnd = %d, vTotal = %d\n",
		 vDisplay, vStart, vEnd, vTotal);
	pr_debug("dotClock = %d\n", dotClock);

	lineLength = width * bpp / 8;

	switch (bpp) {
#ifdef FBCON_HAS_CFB8
	case 8:
		if (hDisplaySize <= 640)
			tmp_mode = DISPLAY_640x480x8;
		else if (width <= 800)
			tmp_mode = DISPLAY_800x600x8;
		else if (width <= 1024)
			tmp_mode = DISPLAY_1024x768x8;
		else if (width <= 1280)
			tmp_mode = DISPLAY_1280x1024x8;
		reg_DPR10(sinfo) = (lineLength << 16) | lineLength;	/* RowPitch */
		reg_DPR1E(sinfo) = 0x0005;
		reg_DPR3C(sinfo) = (lineLength << 16) | lineLength;	/* Dst & Src Window Width */
		reg_VPR00(sinfo) = 0x0 << 16;
		break;
#endif
#ifdef FBCON_HAS_CFB16
	case 16:
		if (hDisplaySize <= 400)
			tmp_mode = DISPLAY_LCD_400x232x16;
		if (hDisplaySize <= 640)
			tmp_mode = DISPLAY_640x480x16;
		else if (width <= 800)
			tmp_mode = DISPLAY_800x600x16;
		else if (width <= 1024)
			tmp_mode = DISPLAY_1024x768x16;
		reg_DPR10(sinfo) = (lineLength / 2 << 16) | lineLength / 2;	/* RowPitch */
		reg_DPR1E(sinfo) = 0x0015;
		reg_DPR3C(sinfo) = (lineLength / 2 << 16) | lineLength / 2;	/* Dst & Src Window Width */
		reg_VPR00(sinfo) = 0x2 << 16;
		break;
#endif
#ifdef FBCON_HAS_CFB24
	case 24:
		if (hDisplaySize <= 640)
			tmp_mode = DISPLAY_640x480x24;
		else if (width <= 800)
			tmp_mode = DISPLAY_800x600x24;
		reg_DPR10(sinfo) = (lineLength / 3 << 16) | lineLength / 3;	/* RowPitch */
		reg_DPR1E(sinfo) = 0x0035;
		reg_DPR3C(sinfo) = (lineLength / 3 << 16) | lineLength / 3;	/* Dst & Src Window Width */
		reg_VPR00(sinfo) = 0x4 << 16;
		break;
#endif
	};

	for (i = 0; i < modeNums; i++) {
		if (ModeInitParams[i].mode == tmp_mode)
			break;
	}
	if (i == modeNums)
		tmp_mode = SMI_DEFAULT_MODE;

	memcpy(&curMode, &ModeInitParams[tmp_mode],
	       sizeof(struct smi_mode_regs));

	/*
	 * Override some Mode Params
	 */
	/* MISC Reg */
	curMode.reg_MISC = 0x30 | (hDisplay == 640) ? 0x03 : 0x0b;
	if (sync & FB_SYNC_HOR_HIGH_ACT)
		curMode.reg_MISC |= 0x40;
	if (sync & FB_SYNC_VERT_HIGH_ACT)
		curMode.reg_MISC |= 0x80;

	/* CRTC */
	curMode.reg_CR00_CR18[0x00] = (u8) (hTotal - 4);
	curMode.reg_CR00_CR18[0x01] = (u8) hDisplay;
	curMode.reg_CR00_CR18[0x02] = (u8) hDisplay;
	curMode.reg_CR00_CR18[0x03] = 0x00;
	curMode.reg_CR00_CR18[0x04] = (u8) hStart;
	curMode.reg_CR00_CR18[0x05] = (hEnd & 0x1f);
	curMode.reg_CR00_CR18[0x06] = (u8) (vTotal & 0xff);
	curMode.reg_CR00_CR18[0x07] = (u8) (((vStart >> 9) & 0x01) << 7)
	    | (u8) (((vDisplay >> 9) & 0x01) << 6)
	    | (u8) (((vTotal >> 9) & 0x01) << 5)
	    | 1 << 4		/* D (LC) */
	    | (u8) (((vStart >> 8) & 0x01) << 2)
	    | (u8) (((vDisplay >> 8) & 0x01) << 1)
	    | (u8) ((vTotal >> 8) & 0x01);

	curMode.reg_CR00_CR18[0x09] = (u8) (vDisplay >> 9) << 5 | 1 << 6;	/* D (LC bit9) */
	curMode.reg_CR00_CR18[0x10] = (u8) (vStart & 0xff);
	curMode.reg_CR00_CR18[0x11] = (u8) (vEnd & 0xf);
	curMode.reg_CR00_CR18[0x12] = (u8) (vDisplay & 0xff);
	curMode.reg_CR00_CR18[0x13] = ((width / 8) * ((bpp + 1) / 8)) & 0xFF;
	curMode.reg_CR00_CR18[0x15] = (u8) (vDisplay & 0xff);
	curMode.reg_CR00_CR18[0x16] = 0x00;
	curMode.reg_CR00_CR18[0x14] = (hDisplaySize > 1024) ? 0x00 : 0x40;	/* D *//* Underline Location */

	/* Extended CRTC */
	curMode.reg_CR30_CR4D[0x30 - 0x30] = (u8) (((vTotal >> 10) & 0x01) << 3)
	    | (u8) (((vDisplay >> 10) & 0x01) << 1)
	    | (u8) ((vStart >> 10) & 0x1);	/* D (CRTD) (CVDER) */

	curMode.reg_SR30_SR75[0x32] = 0xff;	/* (Memory Type and Timig Control Reg) */

	for (i = 0; i <= SIZE_SR00_SR04; i++)
		regSR_write(sinfo->mmio, 0x00 + i, curMode.reg_SR00_SR04[i]);
	for (i = 0; i <= SIZE_SR10_SR24; i++)
		regSR_write(sinfo->mmio, 0x10 + i, curMode.reg_SR10_SR24[i]);
	for (i = 0; i <= SIZE_SR30_SR75; i++) {
		regSR_write(sinfo->mmio, 0x30 + i, curMode.reg_SR30_SR75[i]);
	}
	for (i = 0; i <= SIZE_SR80_SR93; i++)
		regSR_write(sinfo->mmio, 0x80 + i, curMode.reg_SR80_SR93[i]);
	for (i = 0; i <= SIZE_SRA0_SRAF; i++)
		regSR_write(sinfo->mmio, 0xA0 + i, curMode.reg_SRA0_SRAF[i]);
	for (i = 0; i <= SIZE_GR00_GR08; i++)
		regGR_write(sinfo->mmio, 0x00 + i, curMode.reg_GR00_GR08[i]);
	for (i = 0; i <= SIZE_AR00_AR14; i++)
		regAR_write(sinfo->mmio, 0x00 + i, curMode.reg_AR00_AR14[i]);
	for (i = 0; i <= SIZE_CR00_CR18; i++)
		regCR_write(sinfo->mmio, 0x00 + i, curMode.reg_CR00_CR18[i]);
	for (i = 0; i <= SIZE_CR30_CR4D; i++)
		regCR_write(sinfo->mmio, 0x30 + i, curMode.reg_CR30_CR4D[i]);
	for (i = 0; i <= SIZE_CR90_CRA7; i++)
		regCR_write(sinfo->mmio, 0x90 + i, curMode.reg_CR90_CRA7[i]);

	/* SetMemoryMapRegisters */
	reg_DPR14(sinfo) = 0xffffffff;	/* FG color */
	reg_DPR18(sinfo) = 0x00000000;	/* BG color */
	reg_DPR24(sinfo) = 0xffffffff;	/* Color Mask */
	reg_DPR28(sinfo) = 0xffff;	/* Masks */
	reg_DPR2C(sinfo) = 0;
	reg_DPR30(sinfo) = 0;
	reg_DPR34(sinfo) = 0xffffffff;
	reg_DPR38(sinfo) = 0xffffffff;
	reg_DPR40(sinfo) = 0;
	reg_DPR44(sinfo) = 0;
	reg_VPR0C(sinfo) = 0;
	reg_VPR10(sinfo) = ((lineLength / 8 + 2) << 16) | (lineLength / 8);
	reg_VPR40(sinfo) = 0;
	reg_VPR28(sinfo) = 0x00000000;
	reg_VPR2C(sinfo) = ((hDisplaySize - 1) << 16) | (vDisplay);
	reg_VPR30(sinfo) = 0x00000000;
	reg_VPR34(sinfo) = (lineLength << 16) | lineLength;
}
