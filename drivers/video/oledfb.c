/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/cpufreq.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>

#include <asm-arm/arch-pxa/hardware.h>
#include <asm-arm/arch-pxa/io.h>
#include <asm-arm/arch-pxa/irq.h>
#include <asm-arm/uaccess.h>
#include <asm-arm/arch-pxa/bitfield.h>
#include <asm-arm/arch-pxa/pxafb.h>
#include <asm-arm/arch-pxa/pxa-regs.h>

#include "pxafb.h"
#include "oledfb.h"

struct lcd_smart_info *smart_fbi = NULL;

#ifdef CONFIG_REGULAR_UPDATE
static DECLARE_WAIT_QUEUE_HEAD(screen_update_wait_q);
#endif

extern void pxa_set_cken(int clock, int enable);

static int lcd_smart_check_var(struct fb_var_screeninfo *var,
			       struct fb_info *info)
{
	struct lcd_smart_info *smart_fbi = (struct lcd_smart_info *)info;

	pr_debug("lcd_smart_check_var\n");

	if (var->xres < MIN_XRES) {
		var->xres = MIN_XRES;
	}
	if (var->yres < MIN_YRES) {
		var->yres = MIN_YRES;
	}
	if (var->xres > smart_fbi->max_xres) {
		var->xres = smart_fbi->max_xres;
	}
	if (var->yres > smart_fbi->max_yres) {
		var->yres = smart_fbi->max_yres;
	}
	var->xres_virtual = max(var->xres_virtual, var->xres);
	var->yres_virtual = max(var->yres_virtual, var->yres);

	/*
	 * Setup the RGB parameters for this display.
	 *
	 * The pixel packing format is described on page 7-11 of the
	 * PXA2XX Developer's Manual.
	 */
	switch (var->bits_per_pixel) {
	case 16:
		/* 2 pixels per line */
		var->red.offset = 11;
		var->red.length = 5;
		var->green.offset = 5;
		var->green.length = 6;
		var->blue.offset = 0;
		var->blue.length = 5;
		var->transp.offset = var->transp.length = 0;
		break;
	case 18:
	case 19:
		if (smart_fbi->fb.fix.type == FB_TYPE_PACKED_PIXELS) {
			var->red.offset = 12;
			var->red.length = 6;
			var->green.offset = 6;
			var->green.length = 6;
			var->blue.offset = 0;
			var->blue.length = 6;
			var->transp.offset = var->transp.length = 0;
		} else {
			return -1;
		}
		break;
	case 24:
	case 25:
		var->red.offset = 16;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->transp.offset = var->transp.length = 0;
		break;
	default:
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 0;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->transp.offset = var->transp.length = 0;
		break;
	}

	return 0;
}

static int lcd_smart_set_par(struct fb_info *info)
{

	struct lcd_smart_info *smart_fbi = (struct lcd_smart_info *)info;
	struct fb_var_screeninfo *var = &info->var;

	pr_debug("lcd_smart_set_par\n");

	if (var->bits_per_pixel == 16 || var->bits_per_pixel == 18
	    || var->bits_per_pixel == 19 || var->bits_per_pixel == 24
	    || var->bits_per_pixel == 25)
		smart_fbi->fb.fix.visual = FB_VISUAL_TRUECOLOR;
	else {
		smart_fbi->fb.fix.visual = FB_VISUAL_PSEUDOCOLOR;
	}

	switch (var->bits_per_pixel) {
	case 16:
		smart_fbi->fb.fix.line_length = var->xres_virtual * 2;
		break;
	case 18:
	case 19:
		if (smart_fbi->fb.fix.type == FB_TYPE_PACKED_PIXELS)
			smart_fbi->fb.fix.line_length = var->xres_virtual * 3;
		else
			smart_fbi->fb.fix.line_length = var->xres_virtual * 4;
		break;
	case 24:
	case 25:
		smart_fbi->fb.fix.line_length = var->xres_virtual * 4;
		break;
	default:
		smart_fbi->fb.fix.line_length =
		    var->xres_virtual * var->bits_per_pixel / 8;
	}

	smart_fbi->_FRAME_BUFFER_BASE_PHYSICAL = smart_fbi->map_dma + PAGE_SIZE;
	smart_fbi->_COMMAND_BUFFER_BASE_PHYSICAL = smart_fbi->cmd_buf_dma;
	smart_fbi->_DMA_CHANNEL_0_FRAME_DESCRIPTOR_BASE_PHYSICAL =
	    smart_fbi->map_dma + PAGE_SIZE - 2 * 16;
	smart_fbi->_DMA_CHANNEL_6_COMMAND_DESCRIPTOR_BASE_PHYSICAL =
	    smart_fbi->map_dma + PAGE_SIZE - 4 * 16;
	smart_fbi->frameDescriptorCh0fd1 =
	    (struct LCD_FRAME_DESCRIPTOR *)(smart_fbi->map_cpu + PAGE_SIZE -
					    2 * 16);
	smart_fbi->frameDescriptorCh6_command =
	    (struct LCD_FRAME_DESCRIPTOR *)(smart_fbi->map_cpu + PAGE_SIZE -
					    4 * 16);

	return 0;
}

static void lcd_smart_init_pins(void)
{
	extern void zylonite_enable_lcd_smart_pins(void);

	zylonite_enable_lcd_smart_pins();

	return;
}

static void lcd_smart_init_dma(struct lcd_smart_info *smart_fbi)
{
	smart_fbi->FrameBufferSize =
	    smart_fbi->panel->width * smart_fbi->panel->height;

	switch (smart_fbi->fb.var.bits_per_pixel) {
	case 1:
		smart_fbi->FrameBufferSize >>= 3;
		break;
	case 2:
		smart_fbi->FrameBufferSize >>= 2;
		break;
	case 4:
		smart_fbi->FrameBufferSize >>= 1;
		break;
	case 8:
		break;
	case 16:
		smart_fbi->FrameBufferSize <<= 1;
		break;
	case 18:		/* Fall through */
	case 19:
	case 24:
	case 25:
		smart_fbi->FrameBufferSize <<= 2;
		break;
	default:
		break;
	}

	smart_fbi->frameDescriptorCh0fd1->FDADR =
	    XLLP_FDADR_DESCADDR(smart_fbi->
				_DMA_CHANNEL_0_FRAME_DESCRIPTOR_BASE_PHYSICAL);
	smart_fbi->frameDescriptorCh0fd1->FSADR =
	    XLLP_FSADR_SRCADDR(smart_fbi->_FRAME_BUFFER_BASE_PHYSICAL);
	smart_fbi->frameDescriptorCh0fd1->FIDR = XLLP_FIDR_FRAMEID(0);
	smart_fbi->frameDescriptorCh0fd1->LDCMD =
	    XLLP_LDCMD_LEN(smart_fbi->FrameBufferSize);

	return;
}

static void lcd_smart_init_controller_common(struct lcd_smart_info *fbi)
{
	int BPP = 0;

	if (16 == fbi->fb.var.bits_per_pixel) {
		BPP = 4;
	} else {
		pr_debug("invalid bpp parameter BPP = %d\n",
			fbi->fb.var.bits_per_pixel);
		return;
	}

	LCCR0 = LCCR0_OUC | LCCR0_PAS;
	LCCR1 = XLLP_LCCR1_PPL(fbi->panel->width - 1);

	LCCR2 = XLLP_LCCR2_LPP(fbi->panel->height - 1);
	LCCR3 = XLLP_LCCR3_BPP(BPP) | XLLP_LCCR3_PDFOR(fbi->PixelDataFormat);
	LCCR4 = XLLP_LCCR4_PAL_FOR(0);

	/* Mask all interrupt */

	LCCR0 = LCCR0 | LCCR0_LDM | LCCR0_SFM | LCCR0_IUM |
	    LCCR0_EFM | LCCR0_QDM | LCCR0_BM |
	    LCCR0_OUM | LCCR0_RDSTM | LCCR0_CMDIM;
	LCCR4 = LCCR4_REOFM0 | LCCR4_REOFM1 | LCCR4_REOFM2 |
	    LCCR4_REOFM3 | LCCR4_REOFM4 | LCCR4_REOFM5 | LCCR4_REOFM6;
	LCCR5 = LCCR5_SOFM1 | LCCR5_SOFM2 | LCCR5_SOFM3 |
	    LCCR5_SOFM4 | LCCR5_SOFM5 | LCCR5_SOFM6 |
	    LCCR5_EOFM1 | LCCR5_EOFM2 | LCCR5_EOFM3 |
	    LCCR5_EOFM4 | LCCR5_EOFM5 | LCCR5_EOFM6 |
	    LCCR5_BSM1 | LCCR5_BSM2 | LCCR5_BSM3 |
	    LCCR5_BSM4 | LCCR5_BSM5 | LCCR5_BSM6 |
	    LCCR5_IUM1 | LCCR5_IUM2 | LCCR5_IUM3 |
	    LCCR5_IUM4 | LCCR5_IUM5 | LCCR5_IUM6;

	/* Clear status register */
	LCSR = LCSR0_LDD | LCSR0_SOF0 | LCSR0_BER |
	    LCSR0_ABC | LCSR0_IU0 | LCSR0_IU1 |
	    LCSR0_OU | LCSR0_QD | LCSR0_EOF0 |
	    LCSR0_BS0 | LCSR0_SINT | LCSR0_RD_ST |
	    LCSR_CMD_INT | LCSR0_REOF0 | LCSR0_REOF1 |
	    LCSR0_REOF2 | LCSR0_REOF3 | LCSR0_REOF4 | LCSR0_REOF5 | LCSR0_REOF6;

	LCSR1 = LCSR1_SOF1 | LCSR1_SOF2 | LCSR1_SOF3 |
	    LCSR1_SOF4 | LCSR1_SOF5 | LCSR1_SOF6 |
	    LCSR1_EOF1 | LCSR1_EOF2 | LCSR1_EOF3 |
	    LCSR1_EOF4 | LCSR1_EOF5 | LCSR1_EOF6 |
	    LCSR1_BS1 | LCSR1_BS2 | LCSR1_BS3 |
	    LCSR1_BS4 | LCSR1_BS5 | LCSR1_BS6 |
	    LCSR1_IU2 | LCSR1_IU3 | LCSR1_IU4 | LCSR1_IU5 | LCSR1_IU6;
	return;

}

static void lcd_smart_init_timing(struct lcd_smart_info *smart_fbi)
{
	struct smart_timing *timing =
	    (struct smart_timing *)&smart_fbi->panel->timing;

	LCCR1 |=
	    XLLP_LCCR1_HSW(timing->HSW) | XLLP_LCCR1_ELW(timing->
							 ELW) |
	    XLLP_LCCR1_BLW(timing->BLW);

	LCCR3 |= XLLP_LCCR3_PCD(timing->PCD);

	CMDCR = CMDCR_SYNC_CNT(timing->SYNC_CNT);
	return;
}

static void lcd_smart_init_controller(struct lcd_smart_info *smart_fbi)
{
	lcd_smart_init_controller_common(smart_fbi);

	LCCR0 |= LCCR0_LCDT;
	LCCR6 =
	    XLLP_LCCR6_B_BLUE(0) | XLLP_LCCR6_B_GREEN(0) |
	    XLLP_LCCR6_B_RED(0xFF);
	FDADR6 = XLLP_FDADR_DESCADDR(0);
	PRSR &= ~(PRSR_ST_OK | PRSR_CON_ST);
	LCCR3 |= XLLP_LCCR3_ACB(0xff);	/* in active mode this bit is ignored */
	FDADR0 = XLLP_FDADR_DESCADDR(smart_fbi->frameDescriptorCh0fd1->FDADR);

	lcd_smart_init_timing(smart_fbi);

	return;
}

static int lcd_smart_init(struct lcd_smart_info *smart_fbi)
{
	lcd_smart_init_pins();
	lcd_smart_init_dma(smart_fbi);
	lcd_smart_init_controller(smart_fbi);
	smart_fbi->cmd_current = smart_fbi->cmd_buf;
	return 0;
}

static void pxa_smartfb_disable_controller(struct lcd_smart_info *fbi)
{
	LCCR0 &= ~LCCR0_LCDT;

	LCSR = 0xffffffff;	/* Clear LCD Status Register */
	LCCR0 &= ~LCCR0_LDM;
	LCCR0 |= LCCR0_DIS;	/* Disable LCD Controller */
}

static unsigned char rotate_8b(unsigned char c)
{
	char c1 = c << 2;

	return c1 | (c >> 6);
}

static int lcd_smart_load_command(struct lcd_smart_info *smart_fbi,
			   unsigned short *cmdBuf, int cmdNum)
{

	while (smart_fbi->cmd_current < smart_fbi->cmd_buf + smart_fbi->cmd_max) {
		*smart_fbi->cmd_current++ = *cmdBuf++;
		if (!(--cmdNum)) {
			return 0;
		}
	}

	return -1;
}

static void lcd_smart_dma_setup(struct lcd_smart_info *smart_fbi)
{

	unsigned int size;
	unsigned short int_proc_cmd = LCD_CMD_INT_PROC | LCD_CMD_A0_COMMAND;
	unsigned short wait_vsync_cmd =
	    LCD_CMD_WAIT_FOR_VSYNC | LCD_CMD_A0_COMMAND;

	/* Insert the "Interrupt Processor" and "Wait for Vsync" command */
	lcd_smart_load_command(smart_fbi, &int_proc_cmd, 1);
	lcd_smart_load_command(smart_fbi, &wait_vsync_cmd, 1);

	size = smart_fbi->cmd_current - smart_fbi->cmd_buf;
	if (size & 0x1) {
		size++;
	}

	smart_fbi->frameDescriptorCh6_command->LDCMD =
	    XLLP_LDCMD_LEN(size << 3);
	smart_fbi->frameDescriptorCh6_command->FSADR =
	    XLLP_FSADR_SRCADDR(smart_fbi->_COMMAND_BUFFER_BASE_PHYSICAL);
	smart_fbi->frameDescriptorCh6_command->FIDR = XLLP_FIDR_FRAMEID(0);
	smart_fbi->frameDescriptorCh6_command->FDADR =
	    XLLP_FDADR_DESCADDR(smart_fbi->
				_DMA_CHANNEL_6_COMMAND_DESCRIPTOR_BASE_PHYSICAL);

#ifdef CONFIG_PXA3xx
	pxa_set_cken(CKEN_LCD, 0);
#else
	pxa_set_cken(CKEN16_LCD, 0);
#endif

	/* Need to reset the dma control registers */
	LCCR0 &= ~(LCCR0_ENB);
	PRSR |= PRSR_ST_OK | PRSR_CON_ST;
	FDADR6 =
	    XLLP_FDADR_DESCADDR(smart_fbi->
				_DMA_CHANNEL_6_COMMAND_DESCRIPTOR_BASE_PHYSICAL);

	LCCR0 |= LCCR0_ENB;

#ifdef CONFIG_PXA3xx
	pxa_set_cken(CKEN_LCD, 1);
#else
	pxa_set_cken(CKEN16_LCD, 1);
#endif

	return;
}

static void lcd_smart_dma_done(struct lcd_smart_info *smart_fbi)
{
	/* Quick disable */
	PRSR &= ~(PRSR_ST_OK | PRSR_CON_ST);
	LCCR0 &= ~(LCCR0_ENB);
	LCSR |= LCSR_CMD_INT;	/* Clear CMD_INT */
	FDADR6 = XLLP_FDADR_DESCADDR(0);	/* Disable DMA 6 */
	smart_fbi->cmd_current = smart_fbi->cmd_buf;

#ifdef CONFIG_PXA3xx
	pxa_set_cken(CKEN_LCD, 0);
#else
	pxa_set_cken(CKEN16_LCD, 0);
#endif

	return;
}

/*
 * Wait a max # milliseconds for a register to match a given state
 * (any or all bits)
 *
 * Returns number of milliseconds of max_ms remaining
 * (ie. >= 0 = got match, < 0 = no match)
 */
static int lcd_smart_wait_cmd_done(unsigned int mask, unsigned int max_ms)
{
	int matched = 0;
	int sleep_quantum = (max_ms / 100);	/* wait 1/100 and see */
	int sleep_time_left = max_ms;

	if (sleep_quantum == 0) {
		sleep_quantum = 1;
	}
	while (1) {
		matched = ((LCSR & mask) == mask);
		if (matched) {
			if (sleep_time_left < 0) {
				sleep_time_left = 0;
			}
			break;
		}
		if (sleep_time_left < 0) {
			break;
		}

		mdelay(sleep_quantum);
		sleep_time_left -= sleep_quantum;
	}

	return sleep_time_left;
}

static void lcd_smart_send_command(struct lcd_smart_info *smart_fbi)
{
	int left = 0;

	lcd_smart_dma_setup(smart_fbi);

	left = lcd_smart_wait_cmd_done(LCSR_CMD_INT, 1000);

	lcd_smart_dma_done(smart_fbi);

	if (left < 0) {
		pr_debug("lcd_smart_send_command end: left = %d\n", left);
	}

	return;
}

static void cmd_out(struct lcd_smart_info *fbi, unsigned char cmd)
{
	unsigned short lcd_cmd = (unsigned short)MAKEUP_CMD(rotate_8b(cmd));
	lcd_smart_load_command(fbi, &lcd_cmd, 1);
	return;
}

static void data_out(struct lcd_smart_info *fbi, unsigned char data)
{
	unsigned short lcd_cmd = (unsigned short)MAKEUP_DATA(rotate_8b(data));
	lcd_smart_load_command(fbi, &lcd_cmd, 1);
	return;
}

static void lcd_smart_init_oled(struct lcd_smart_info *fbi)
{
	cmd_out(fbi, 0xae);	/* display off */

	cmd_out(fbi, 0xca);	/* set mux ratio */
	data_out(fbi, 0x7f);	/* 1/128 duty */

	cmd_out(fbi, 0xa1);	/* display start line */
	data_out(fbi, 0x00);	/* start on 0 */

	cmd_out(fbi, 0xa2);	/* display offset */
	data_out(fbi, 0x80);	/* offset 0 */

	cmd_out(fbi, 0xa0);	/* set remap and color depth */
	data_out(fbi, 0xa0);	/* 262k color, 8-bit MCU interface / Scan from COM 0 to COM [N .1] */

	cmd_out(fbi, 0xc7);	/* master current control */
	data_out(fbi, 0x09);	/* brightness: 0x3 low, 0x6 typical, 0x9 high */

	cmd_out(fbi, 0xc1);	/* set contrast level for R, G, B */
	data_out(fbi, 0x6f);	/* red */
	data_out(fbi, 0x58);	/* green */
	data_out(fbi, 0x80);	/* blue */

	cmd_out(fbi, 0xb1);	/* set reset and precharge period */
	data_out(fbi, 0x1f);

	cmd_out(fbi, 0xb3);	/* frame rate */
	data_out(fbi, 0x10);	/* 85Hz */

	cmd_out(fbi, 0xbb);	/* precharge level for R, G, B */
	data_out(fbi, 0x00);	/* red */
	data_out(fbi, 0x00);	/* green */
	data_out(fbi, 0x00);	/* blue */

	cmd_out(fbi, 0xad);	/* master configuration */
	data_out(fbi, 0x8e);

	cmd_out(fbi, 0xb0);	/* current saving */
	data_out(fbi, 0x05);

	cmd_out(fbi, 0xa6);	/* normal display */

	cmd_out(fbi, 0xaf);	/* display on */

	lcd_smart_send_command(fbi);

	return;
}

/*
 * Better if we can use a general set_ctrlr_state() interface to change
 * LCD controller state, but the Smart mode is quite different
 */
static void pxa_smartfb_set_ctrlr_state(struct lcd_smart_info *fbi, u_int state)
{
	u_int old_state;

	/* Need to sync with set_ctrlr_state() */
	down_interruptible(&fbi->ctrlr_sem);

	old_state = fbi->state;

	switch (state) {
	case C_DISABLE:
		/*
		 * Disable controller
		 */
		if (old_state != C_DISABLE) {
			if (old_state == C_BLANK) {
				lcd_smart_init_pins();
			}
			if (old_state != C_ENABLE) {
				lcd_smart_init(fbi);
			}
			fbi->state = state;
			pxa_smartfb_disable_controller(fbi);

			/* disable LCD controller clock */
#ifdef CONFIG_PXA3xx
			pxa_set_cken(CKEN_LCD, 0);
#else
			pxa_set_cken(CKEN16_LCD, 0);
#endif
		}
		break;

	case C_ENABLE:
		/*
		 * Power up the LCD screen, enable controller.
		 */
		if (old_state != C_ENABLE) {
			fbi->state = C_ENABLE;
			lcd_smart_init(fbi);
			lcd_smart_init_oled(fbi);
#ifdef CONFIG_REGULAR_UPDATE
			wake_up_interruptible(&screen_update_wait_q);
#endif
		}
		break;
	case C_REENABLE:
		if (old_state == C_ENABLE) {
			pxa_smartfb_disable_controller(fbi);
			lcd_smart_init(fbi);
		}
		break;
	case C_BLANK:
		/*
		 * Disable controller, blank overlays if exist.
		 */
		if (old_state == C_ENABLE) {
			fbi->state = state;
			if (old_state != C_ENABLE) {
				lcd_smart_init(fbi);
			}

			pxa_smartfb_disable_controller(fbi);
			/* disable LCD controller clock */
#ifdef CONFIG_PXA3xx
			pxa_set_cken(CKEN_LCD, 0);
#else
			pxa_set_cken(CKEN16_LCD, 0);
#endif
		}
		break;

	case C_UNBLANK:
		/*
		 * Power up the LCD screen, enable controller, and
		 * turn on the backlight, unblank overlays if exist.
		 */
		if (old_state == C_BLANK) {
			fbi->state = C_ENABLE;
			lcd_smart_init(fbi);
			if (old_state == C_BLANK) {
				lcd_smart_init_oled(fbi);
			}
#ifdef CONFIG_REGULAR_UPDATE
			wake_up_interruptible(&screen_update_wait_q);
#endif
		}
		break;
	}
	up(&fbi->ctrlr_sem);
}

#ifdef CONFIG_REGULAR_UPDATE
static int pxafb_smart_screenupdate_fn(void *arg);
#endif

static int lcd_smart_open(struct fb_info *info, int user)
{
	int ret = 0;
	struct lcd_smart_info *fbi = (struct lcd_smart_info *)info;

	pr_debug("lcd_smart_open\n");

#ifdef CONFIG_REGULAR_UPDATE
	if (kernel_thread(pxafb_smart_screenupdate_fn,
			  fbi, CLONE_FS | CLONE_FILES | CLONE_SIGHAND) < 0) {
		pr_debug("kernel thread creat fail\n");
		return -1;
	}
#endif
	/*
	 * This makes sure that our colour bitfield
	 * descriptors are correctly initialised.
	 */
	lcd_smart_check_var(&fbi->fb.var, &fbi->fb);
	lcd_smart_set_par(&fbi->fb);

	/*
	 * Ok, now enable the LCD controller
	 */
	pxa_smartfb_set_ctrlr_state(fbi, C_ENABLE);
	return ret;
}

static int lcd_smart_release(struct fb_info *info, int user)
{
	int ret = 0;
	struct lcd_smart_info *fbi = (struct lcd_smart_info *)info;

	pr_debug("lcd_smart_release\n");

	pxa_smartfb_set_ctrlr_state(fbi, C_DISABLE);

	return ret;
}

static inline void smartpanel_schedule_work(struct lcd_smart_info *fbi,
					    u_int state)
{
	unsigned long flags;

	local_irq_save(flags);
	/*
	 * We need to handle two requests being made at the same time.
	 * There are two important cases:
	 *  1. When we are changing VT (C_REENABLE) while unblanking (C_ENABLE)
	 *     We must perform the unblanking, which will do our REENABLE for us.
	 *  2. When we are blanking, but immediately unblank before we have
	 *     blanked.  We do the "REENABLE" thing here as well, just to be sure.
	 */
	if (fbi->task_state == C_ENABLE && state == C_REENABLE)
		state = (u_int) - 1;
	if (fbi->task_state == C_DISABLE && state == C_ENABLE)
		state = C_REENABLE;

	if (state != (u_int) - 1) {
		fbi->task_state = state;
		schedule_work(&fbi->task);
	}
	local_irq_restore(flags);
}

static int lcd_smart_blank(int blank, struct fb_info *info)
{
	struct lcd_smart_info *fbi = (struct lcd_smart_info *)info;

	pr_debug("pxafb: blank=%d\n", blank);

	switch (blank) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		smartpanel_schedule_work(fbi, C_BLANK);
		break;

	case FB_BLANK_UNBLANK:
		smartpanel_schedule_work(fbi, C_UNBLANK);
	}
	return 0;
}

static void oled_set_update_area(struct lcd_smart_info *fbi, unsigned int x1,
				 unsigned int x2, unsigned int y1,
				 unsigned int y2)
{
	/* column address */
	cmd_out(fbi, 0x15);
	data_out(fbi, (unsigned char)x1);	/* start */
	data_out(fbi, (unsigned char)x2);	/* end */

	/* row address */
	cmd_out(fbi, 0x75);
	data_out(fbi, (unsigned char)y1);	/* start */
	data_out(fbi, (unsigned char)y2);	/* end */
}

static void oled_write_ram_cmd(struct lcd_smart_info *fbi)
{
	cmd_out(fbi, 0x5c);
}

static void lcd_smart_update_setup(struct lcd_smart_info *smart_fbi, unsigned short x,
			    unsigned short y)
{
	unsigned int x2;
	unsigned int y2;

	unsigned short FrameWriteCmd =
	    LCD_CMD_FRAME_DATA_WRITE | LCD_CMD_A0_DATA;

	x2 = x + smart_fbi->panel->width - 1;
	y2 = y + smart_fbi->panel->height - 1;

	oled_set_update_area(smart_fbi, x, x2, y, y2);
	oled_write_ram_cmd(smart_fbi);

	/* The DMA setting looks strange, but it works.. */
	FDADR0 = XLLP_FDADR_DESCADDR(smart_fbi->
				_DMA_CHANNEL_0_FRAME_DESCRIPTOR_BASE_PHYSICAL);

	lcd_smart_load_command(smart_fbi, &FrameWriteCmd, 1);
	lcd_smart_dma_setup(smart_fbi);

	return;
}

static void lcd_smart_update_done(struct lcd_smart_info *smart_fbi)
{
	lcd_smart_dma_done(smart_fbi);

	FDADR0 = XLLP_FDADR_DESCADDR(0);

	return;
}

static void lcd_smart_update_framedata(struct lcd_smart_info *smart_fbi,
				unsigned short x, unsigned short y)
{
	int left = 0;

	lcd_smart_update_setup(smart_fbi, x, y);
	left = lcd_smart_wait_cmd_done(LCSR_CMD_INT, 1000);
	lcd_smart_update_done(smart_fbi);
	if (left < 0) {
		pr_debug("lcd_smart_update_framedata end: left = %d\n", left);
	}

	return;
}

static int lcd_smartfb_ioctl(struct inode *inode,
			     struct file *file, unsigned int cmd,
			     unsigned long arg, struct fb_info *info)
{
	struct lcd_smart_info *smart_fbi = (struct lcd_smart_info *)info;
	struct pxa_smartpanel_cmdbuf cmd_buf;

	switch (cmd) {
	case FBIOSCREENUPDATE:
		down_interruptible(&smart_fbi->ctrlr_sem);
		if (smart_fbi->state != C_ENABLE) {
			printk(KERN_WARNING
			       "CLI not enabled. Cannot Update Smart Panel\n");
			up(&smart_fbi->ctrlr_sem);
			return -1;
		}
		lcd_smart_update_framedata(smart_fbi, 0, 0);
		up(&smart_fbi->ctrlr_sem);
		return 0;
	case FBIOSENDCMD:
		if (copy_from_user(&cmd_buf, (void *)arg, sizeof(cmd_buf)))
			return -EFAULT;
		down_interruptible(&smart_fbi->ctrlr_sem);
		if (smart_fbi->state != C_ENABLE) {
			printk(KERN_WARNING
			       "CLI not enabled. Cannot send command to smart panel\n");
			up(&smart_fbi->ctrlr_sem);
			return -EACCES;
		}
		lcd_smart_load_command(smart_fbi, cmd_buf.cmds, cmd_buf.cmd_num);
		lcd_smart_send_command(smart_fbi);
		up(&smart_fbi->ctrlr_sem);
		return 0;
	}
	return -EINVAL;
}

static struct smart_panel oled_ssd1339 = {
	.width = OLED_WIDTH,
	.height = OLED_HEIGHT,
	.bpp = 16,
	.timing = {
		.BLW = 30,	/* WR, RD pulse width */
		.ELW = 8,	/* A0, CS, setup/hold */
		.HSW = 10,	/* output hold */
		.PCD = 42,	/* command inhibit */
		.SYNC_CNT = 2,	/* synchronous count */
	}
};

/*
 * Our LCD controller task (which is called when we blank or unblank)
 * via keventd.
 */
static void pxafb_smart_task(void *dummy)
{
	struct lcd_smart_info *smart_fbi = dummy;
	u_int state = xchg(&smart_fbi->task_state, -1);

	pxa_smartfb_set_ctrlr_state(smart_fbi, state);
}

#ifdef CONFIG_REGULAR_UPDATE
#define PXA_SMART_UPDATE_INTERVAL_MS 30
static int pxafb_smart_screenupdate_fn(void *arg)
{
	struct lcd_smart_info *fbi = (struct lcd_smart_info *)arg;
	int update_interval = PXA_SMART_UPDATE_INTERVAL_MS * HZ / 1000;

	if (!fbi)
		return -1;
	pr_debug("THREAD created\n");
	while (1) {
		down_interruptible(&fbi->ctrlr_sem);

		if (fbi->state == C_DISABLE) {
			up(&fbi->ctrlr_sem);
			break;
		}

		while (fbi->state != C_ENABLE) {
			up(&fbi->ctrlr_sem);
			wait_event_interruptible(screen_update_wait_q,
						 (fbi->state == C_ENABLE));
			down(&fbi->ctrlr_sem);
		}

		lcd_smart_update_framedata(fbi, 0, 0);
		up(&fbi->ctrlr_sem);
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(update_interval);
	}
	pr_debug("THREAD close\n");
	return 0;
}
#endif

static struct fb_ops lcd_smart_ops = {
	.owner = THIS_MODULE,
	.fb_open = lcd_smart_open,
	.fb_release = lcd_smart_release,
	.fb_check_var = lcd_smart_check_var,
	.fb_set_par = lcd_smart_set_par,
	.fb_blank = lcd_smart_blank,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_cursor = soft_cursor,
	.fb_ioctl = lcd_smartfb_ioctl,
};

static struct lcd_smart_info *__init pxafb_init_fbinfo(void)
{
	struct lcd_smart_info *smart_fbi;
	struct smart_panel *panel = (struct smart_panel *)&oled_ssd1339;

	/* Alloc the lcd_smart_info and pseudo_palette in one step */
	smart_fbi = kmalloc(sizeof(struct lcd_smart_info), GFP_KERNEL);
	if (!smart_fbi) {
		return NULL;
	}

	memset(smart_fbi, 0, sizeof(struct lcd_smart_info));
	smart_fbi->panel = panel;

	strcpy(smart_fbi->fb.fix.id, "OLED");

	smart_fbi->fb.fix.type = FB_TYPE_PACKED_PIXELS;
	smart_fbi->fb.fix.type_aux = 0;
	smart_fbi->fb.fix.xpanstep = 0;
	smart_fbi->fb.fix.ypanstep = 0;
	smart_fbi->fb.fix.ywrapstep = 0;
	smart_fbi->fb.fix.accel = FB_ACCEL_NONE;

	smart_fbi->fb.var.nonstd = 0;
	smart_fbi->fb.var.activate = FB_ACTIVATE_NOW;
	smart_fbi->fb.var.height = -1;
	smart_fbi->fb.var.width = -1;
	smart_fbi->fb.var.accel_flags = 0;
	smart_fbi->fb.var.vmode = FB_VMODE_NONINTERLACED;

	smart_fbi->fb.fbops = &lcd_smart_ops;
	smart_fbi->fb.flags = FBINFO_DEFAULT;
	smart_fbi->fb.node = -1;

	smart_fbi->max_xres = panel->width;
	smart_fbi->fb.var.xres = panel->width;
	smart_fbi->fb.var.xres_virtual = panel->width;
	smart_fbi->max_yres = panel->height;
	smart_fbi->fb.var.yres = panel->height;
	smart_fbi->fb.var.yres_virtual = panel->height;
	smart_fbi->max_bpp = panel->bpp;
	smart_fbi->fb.var.bits_per_pixel = panel->bpp;

	if (smart_fbi->max_bpp <= 16) {
		smart_fbi->fb.fix.smem_len =
		    smart_fbi->max_xres * smart_fbi->max_yres *
		    smart_fbi->max_bpp / 8;
		smart_fbi->fb.fix.line_length = smart_fbi->fb.var.xres * 2;
	} else if (smart_fbi->max_bpp > 19) {
		smart_fbi->fb.fix.smem_len =
		    smart_fbi->max_xres * smart_fbi->max_yres * 4;
		smart_fbi->fb.fix.line_length = smart_fbi->fb.var.xres * 4;
	} else {		/* 18, 19 bpp */
		if (smart_fbi->fb.fix.type == FB_TYPE_PACKED_PIXELS) {
			/* packed format */
			smart_fbi->fb.fix.smem_len =
			    smart_fbi->max_xres * smart_fbi->max_yres * 3;
			smart_fbi->fb.fix.line_length =
			    smart_fbi->fb.var.xres * 3;
		} else {
			/* unpacked format */
			smart_fbi->fb.fix.smem_len =
			    smart_fbi->max_xres * smart_fbi->max_yres * 4;
			smart_fbi->fb.fix.line_length =
			    smart_fbi->fb.var.xres * 4;
		}
	}

	INIT_WORK(&smart_fbi->task, pxafb_smart_task, smart_fbi);
	init_MUTEX(&smart_fbi->ctrlr_sem);

	return smart_fbi;
}

static int __init pxafb_map_video_memory(struct lcd_smart_info *smart_fbi)
{
	unsigned short *cmd_ptr;
	unsigned int cmd_cnt;

	/*
	 * we reserve 3 pages after the framebuffer for occasionally out
	 * of boundary access
	 */
	smart_fbi->map_size =
	    PAGE_ALIGN(smart_fbi->fb.fix.smem_len + 4 * PAGE_SIZE);
	smart_fbi->map_cpu =
	    dma_alloc_writecombine(NULL, smart_fbi->map_size,
				   &smart_fbi->map_dma, GFP_KERNEL);

	if (smart_fbi->map_cpu) {
		/* prevent initial garbage on screen */
		memset(smart_fbi->map_cpu, 0, smart_fbi->map_size);
		smart_fbi->fb.screen_base = smart_fbi->map_cpu + PAGE_SIZE;
		smart_fbi->fb.fix.smem_start = smart_fbi->map_dma + PAGE_SIZE;
	} else {
		return -ENOMEM;
	}

	smart_fbi->cmd_max = sizeof(unsigned short) * PXA_SMART_CMD_BUF_MAX_NUM;
	smart_fbi->cmd_buf = dma_alloc_writecombine(NULL, smart_fbi->cmd_max,
						    &smart_fbi->cmd_buf_dma,
						    GFP_KERNEL);

	if (!smart_fbi->cmd_buf)
		return -ENOMEM;

	memset(smart_fbi->cmd_buf, 0, smart_fbi->cmd_max);

	/* Fill the command buffer with NOP */
	cmd_ptr = (unsigned short *)smart_fbi->cmd_buf;
	cmd_cnt = smart_fbi->cmd_max / sizeof(unsigned short);
	while (cmd_cnt-- > 0) {
		*cmd_ptr++ = LCD_CMD_NOP;
	}

	return 0;
}

struct callback_data_t {
	char *name;
	struct device *dev;
};

static int lcd_smart_find_dev(struct device *dev, void *callback_data)
{
	int found = 0;
	struct callback_data_t *data = (struct callback_data_t *)callback_data;

	found = (strncmp(dev->kobj.name, data->name, KOBJ_NAME_LEN) == 0);
	if (found == 1) {
		data->dev = dev;
	}

	return found;
}

struct device *lcd_smart_find_bus_device(struct bus_type *bus, char *name)
{
	struct callback_data_t callback_data;

	callback_data.name = name;
	callback_data.dev = NULL;
	bus_for_each_dev(bus, NULL, &callback_data, lcd_smart_find_dev);

	return callback_data.dev;
}

static int __devinit pxafb_smart_init(void)
{
	int ret = 0;

	ret = -1;
	smart_fbi = NULL;

	pr_debug("PXA smart lcd driver initializing\n");

	smart_fbi = pxafb_init_fbinfo();

	if (!smart_fbi) {
		ret = -ENOMEM;
		pr_debug("pxafb_init_fbinfo failed\n");
		goto failed;
	}

	ret = pxafb_map_video_memory(smart_fbi);
	if (ret) {
		pr_debug("pxafb_map_video_memory failed\n");
		goto failed;
	}

	ret = register_framebuffer(&smart_fbi->fb);
	if (ret < 0) {
		pr_debug("register_framebuffer failed\n");
		goto failed;
	}

	printk(KERN_INFO "PXA smart lcd driver\n");
	return 0;

failed:
	if (smart_fbi) {
		kfree(smart_fbi);
	}
	printk(KERN_INFO "PXA smart lcd driver failed\n");
	return ret;

}

static void __exit pxafb_smart_exit(void)
{
	if (smart_fbi) {
		unregister_framebuffer(&(smart_fbi->fb));
		if (smart_fbi->cmd_buf)
			dma_free_writecombine(NULL, smart_fbi->cmd_max,
					      (void *)(smart_fbi->cmd_buf),
					      smart_fbi->cmd_buf_dma);
		if (smart_fbi->map_cpu) {
			dma_free_writecombine(NULL, smart_fbi->map_size,
					      (void *)(smart_fbi->map_cpu),
					      smart_fbi->map_dma);
		}
		kfree(smart_fbi);
	}

	printk(KERN_INFO "PXA smart lcd driver unloaded\n");
	return;
}

module_init(pxafb_smart_init);
module_exit(pxafb_smart_exit);

MODULE_DESCRIPTION("Loadable smart panel framebuffer for PXA");
MODULE_LICENSE("GPL");
