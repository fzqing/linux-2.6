/*
 * s1d13806fb.c,v 0.9 2002/09/05
 *  S1D13806 video device driver for Super-H
 *
 *   Copyright (C) 2002/08/20 Mitsuharu Takei(takei-mitsuharu@hitachi-ul.co.jp)
 *   Copyright(C) 2004/10/13 Mitsuharu Takei(takei-mitsuharu@hitachi-ul.co.jp)
 *    Modified to support kernel version 2.6
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>

#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/io.h>
#include <asm/irq.h>

#include "s1d13806.h"

struct s1d13806fb_par {

	/* struct about register */
	u8 hdisp;
	u8 hblank;
	u8 hstart;
	u8 hsw;

	u16 vdisp;
	u8 vblank;
	u8 vstart;
	u8 vsw;

	u16 x_vir;
	u16 y_vir;

} s1d13806_par;

static int pseudo_palette[16];

/*
 * Here we define the default structs fb_fix_screeninfo and fb_var_screeninfo
 * if we don't use modedb. If we do use modedb see xxxfb_init how to use it
 * to get a fb_var_screeninfo. Otherwise define a default var as well.
 */
static struct fb_fix_screeninfo s1d13806fb_fix __initdata = {
	.id = "s1d13806",
	.smem_start = S1D13806_FB_PHYS,	/* Top addr of frame Buffer */
	.smem_len = S1D13806_FB_PHYS_LEN,	/* Length of frame buffer */
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_TRUECOLOR,
	.xpanstep = 1,
	.ypanstep = 1,
	.ywrapstep = 0,
	.line_length = 240 * 2,	/* xres_virtual*(16bit/8) */
	.accel = FB_ACCEL_NONE,
};

#ifdef CONFIG_S1D13806_NTSC
static struct fb_var_screeninfo s1d13806fb_default = {
	/* 16 bpp */
	.xres = 640,
	.yres = 480,
	.xres_virtual = 640,
	.yres_virtual = 480,
	.bits_per_pixel = 16,
	.activate = 0,
	.height = -1,
	.width = -1,
	.pixclock = S1D13806_PIXCLOCK_14_318,
	.left_margin = 189,
	.right_margin = 73,
	.upper_margin = 28,
	.lower_margin = 1,
	.hsync_len = 1,
	.vsync_len = 1,
	.vmode = FB_VMODE_INTERLACED,
};
#elif defined(CONFIG_S1D13806_LCD)
#if defined(CONFIG_SH_SOLUTION_ENGINE_2ND)
static struct fb_var_screeninfo s1d13806fb_default = {
	/* 16 bpp */
	.xres = 240,
	.yres = 320,
	.xres_virtual = 240,
	.yres_virtual = 320,
	.bits_per_pixel = 16,
	.activate = 0,
	.height = -1,
	.width = -1,
	.pixclock = S1D13806_PIXCLOCK_5_6448,
	.left_margin = 10,
	.right_margin = 6,
	.upper_margin = 5,
	.lower_margin = 1,
	.hsync_len = 32,
	.vsync_len = 6,
	.vmode = FB_VMODE_NONINTERLACED,
};
#else
static struct fb_var_screeninfo s1d13806fb_default = {
	/* 16 bpp */
	.xres = 640,
	.yres = 480,
	.xres_virtual = 640,
	.yres_virtual = 480,
	.bits_per_pixel = 16,
	.activate = 0,
	.height = -1,
	.width = -1,
	.pixclock = S1D13806_PIXCLOCK_25_175,
	.left_margin = 50,
	.right_margin = 14,
	.upper_margin = 31,
	.lower_margin = 12,
	.hsync_len = 96,
	.vsync_len = 2,
	.vmode = FB_VMODE_NONINTERLACED,
};
#endif
#else
static struct fb_var_screeninfo s1d13806fb_default = {
	/* 16 bpp */
	.xres = 640,
	.yres = 480,
	.xres_virtual = 640,
	.yres_virtual = 480,
	.bits_per_pixel = 16,
	.activate = 0,
	.height = -1,
	.width = -1,
	.pixclock = S1D13806_PIXCLOCK_25_175,
	.left_margin = 24,
	.right_margin = 40,
	.upper_margin = 18,
	.lower_margin = 25,
	.hsync_len = 96,
	.vsync_len = 2,
	.vmode = FB_VMODE_NONINTERLACED,
};
#endif

static struct fb_info info;

int s1d13806fb_init(void);
int s1d13806fb_setup(char *);

/**
 *      xxxxfb_open - Optional function. Called when the framebuffer is
 *                   first accessed.
 *      @info: frame buffer structure that represents a single frame buffer
 *      @user: tell us if the userland (value=1) or the console is accessing
 *             the framebuffer.
 *
 *      This function is the first function called in the framebuffer api.
 *      Usually you don't need to provide this function. The case where it
 *      is used is to change from a text mode hardware state to a graphics
 *      mode state.
 */
static int s1d13806fb_open(const struct fb_info *info, int user)
{
	return 0;
}

/**
 *      xxxxfb_release - Optional function. Called when the framebuffer
 *                      device is closed.
 *      @info: frame buffer structure that represents a single frame buffer
 *      @user: tell us if the userland (value=1) or the console is accessing
 *             the framebuffer.
 *
 *      Thus function is called when we close /dev/fb or the framebuffer
 *      console system is released. Usually you don't need this function.
 *      The case where it is usually used is to go from a graphics state
 *      to a text mode state.
 */
static int s1d13806fb_release(const struct fb_info *info, int user)
{
	return 0;
}

/**
 *      xxxfb_check_var - Optional function. Validates a var passed in.
 *      @var: frame buffer variable screen structure
 *      @info: frame buffer structure that represents a single frame buffer
 *
 */
static int s1d13806fb_check_var(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	u32 xres, right, hslen, left;
	u32 yres, lower, vslen, upper;
	u32 vxres, xoffset, vyres, yoffset;

	xres = (var->xres + 7) & ~7;
	vxres = (var->xres_virtual + 0xF) & ~0xF;
	xoffset = (var->xoffset + 7) & ~7;
	left = (var->left_margin + 7) & ~7;
	right = (var->right_margin + 7) & ~7;
	hslen = (var->hsync_len + 7) & ~7;

	if (vxres < xres)
		vxres = xres;
	if (xres + xoffset > vxres)
		xoffset = vxres - xres;

	var->xres = xres;
	var->right_margin = right;
	var->hsync_len = hslen;
	var->left_margin = left;
	var->xres_virtual = vxres;
	var->xoffset = xoffset;

	yres = var->yres;
	lower = var->lower_margin;
	vslen = var->vsync_len;
	upper = var->upper_margin;
	vyres = var->yres_virtual;
	yoffset = var->yoffset;

	if (yres > vyres)
		vyres = yres;
	if (yoffset + yres > vyres)
		yoffset = vyres - yres;
	var->yres = yres;
	var->lower_margin = lower;
	var->vsync_len = vslen;
	var->upper_margin = upper;
	var->yres_virtual = vyres;
	var->yoffset = yoffset;

	var->red.msb_right = 0;
	var->green.msb_right = 0;
	var->blue.msb_right = 0;

	switch (var->bits_per_pixel) {
	case 8:		/* PSEUDOCOLOUR, 256 */
		var->transp.offset = 0;
		var->transp.length = 0;
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 0;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		break;

	case 16:		/* TRUECOLOR(?), 64k */
		var->transp.offset = 0;
		var->transp.length = 0;
		var->red.offset = 11;
		var->red.length = 5;
		var->green.offset = 5;
		var->green.length = 6;
		var->blue.offset = 0;
		var->blue.length = 5;
		break;

	case 24:		/* TRUECOLOUR, 16m */
		var->transp.offset = 0;
		var->transp.length = 0;
		var->red.offset = 16;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 0;
		break;

	default:
		printk(KERN_WARNING "s1d13806fb: no support for %dbpp\n",
		       var->bits_per_pixel);
		return -EINVAL;
	}

	var->activate = FB_ACTIVATE_NOW;
	var->height = -1;
	var->width = -1;
	var->accel_flags = FB_ACCELF_TEXT;
	return 0;
}

/**
 *      xxxfb_setcolreg - Optional function. Sets a color register.
 *      @regno: Which register in the CLUT we are programming
 *      @red: The red value which can be up to 16 bits wide
 *      @green: The green value which can be up to 16 bits wide
 *      @blue:  The blue value which can be up to 16 bits wide.
 *      @transp: If supported the alpha value which can be up to 16 bits wide.

 *      @info: frame buffer info structure
 *
 *      Set a single color register. The values supplied have a 16 bit
 *      magnitude which needs to be scaled in this function for the hardware.
 *      Things to take into consideration are how many color registers, if
 *      any, are supported with the current color visual. With truecolor mode
 *      no color palettes are supported. Here a psuedo palette is created
 *      which we store the value in pseudo_palette in struct fb_info. For
 *      pseudocolor mode we have a limited color palette. To deal with this
 *      we can program what color is displayed for a particular pixel value.
 *      DirectColor is similar in that we can program each color field. If
 *      we have a static colormap we don't need to implement this function.
 *
 *      Returns negative errno on error, or zero on success.
 */
static int s1d13806fb_setcolreg(unsigned regno, unsigned red, unsigned green,
				unsigned blue, unsigned transp,
				const struct fb_info *info)
{
	if (regno > 255)
		return 1;

	/* grayscale works only partially under directcolor */
	if (info->var.grayscale) {
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue = (red * 77 + green * 151 + blue * 28) >> 8;
	}

	/* Directcolor:
	 *   var->{color}.offset contains start of bitfield
	 *   var->{color}.length contains length of bitfield
	 *   {hardwarespecific} contains width of DAC
	 *   cmap[X] is programmed to (X << red.offset) | (X << green.offset) | (X <<blue.offset)
	 *   RAMDAC[X] is programmed to (red, green, blue)
	 *
	 * Pseudocolor:
	 *    uses offset = 0 && length = DAC register width.
	 *    var->{color}.offset is 0
	 *    var->{color}.length contains widht of DAC
	 *    cmap is not used
	 *    DAC[X] is programmed to (red, green, blue)
	 * Truecolor:
	 *    does not use RAMDAC (usually has 3 of them).
	 *    var->{color}.offset contains start of bitfield
	 *    var->{color}.length contains length of bitfield
	 *    cmap is programmed to (red << red.offset) | (green << green.offset) |
	 *                      (blue << blue.offset) | (transp << transp.offset)
	 *    RAMDAC does not exist
	 */
#define CNVT_TOHW(val,width) ((((val)<<(width))+0x7FFF-(val))>>16)
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		red = CNVT_TOHW(red, info->var.red.length);
		green = CNVT_TOHW(green, info->var.green.length);
		blue = CNVT_TOHW(blue, info->var.blue.length);
		transp = CNVT_TOHW(transp, info->var.transp.length);
		break;
	case FB_VISUAL_DIRECTCOLOR:
		/* example here assumes 8 bit DAC. Might be different
		 * for your hardware */
		red = CNVT_TOHW(red, 8);
		green = CNVT_TOHW(green, 8);
		blue = CNVT_TOHW(blue, 8);
		/* hey, there is bug in transp handling... */
		transp = CNVT_TOHW(transp, 8);
		break;
	}
#undef CNVT_TOHW
	/* Truecolor has hardware independent palette */
	if (info->fix.visual == FB_VISUAL_TRUECOLOR) {
		u32 v;

		if (regno >= 16)
			return 1;

		v = (red << info->var.red.offset) |
		    (green << info->var.green.offset) |
		    (blue << info->var.blue.offset) |
		    (transp << info->var.transp.offset);

		switch (info->var.bits_per_pixel) {
		case 8:
			/* Yes some hand held devices have this. */
			((u8 *) (info->pseudo_palette))[regno] = v;
			break;
		case 16:
			((u16 *) (info->pseudo_palette))[regno] = v;
			break;
		case 24:
		case 32:
			((u32 *) (info->pseudo_palette))[regno] = v;
			break;
		}
		return 0;
	}

	if (info->fix.visual == FB_VISUAL_PSEUDOCOLOR) {

		if (regno >= 256)
			return 1;

		writeb(0x00, S1D13806_R(0x1e0));
		writeb(regno, S1D13806_R(0x1e2));

		writeb(red << 4, S1D13806_R(0x1e4));
		writeb(green << 4, S1D13806_R(0x1e4));
		writeb(blue << 4, S1D13806_R(0x1e4));
	}
	return 0;
}

static int s1d13806fb_blank(int blank, struct fb_info *info)
{
	if (blank) {
#ifdef CONFIG_S1D13806_LCD
		writeb((readb(S1D13806_R(0x040)) | 0x80), S1D13806_R(0x040));	/* LCD */
#else
		writeb((readb(S1D13806_R(0x060)) | 0x80), S1D13806_R(0x060));
#endif
	} else {
#ifdef CONFIG_S1D13806_LCD
		writeb((readb(S1D13806_R(0x040)) & ~0x80), S1D13806_R(0x040));	/* LCD */
#else
		writeb((readb(S1D13806_R(0x060)) & ~0x80), S1D13806_R(0x060));
#endif
	}
	return 0;
}

void s1d13806fb_ini_reg(struct s1d13806fb_par *par)
{
	/* initializing */
	writeb(0x00, S1D13806_R(0x001));
	writeb(0x00, S1D13806_R(0x1fc));	/* display disable */

#if defined(CONFIG_SH_SOLUTION_ENGINE_2ND)
	writeb(0x07, S1D13806_R(0x004));
#else
	writeb(0x03, S1D13806_R(0x004));
#endif
	writeb(0x00, S1D13806_R(0x005));
#ifdef CONFIG_S1D13806_NTSC
	writeb(0x02, S1D13806_R(0x008));
#else
#if defined(CONFIG_SH_SOLUTION_ENGINE_2ND)
	writeb(0x00, S1D13806_R(0x008));	/* GPIO0(DON) OutPut low */
#else
	writeb(0x00, S1D13806_R(0x008));	/* LCD/CRT=0x00 NTSC/PAL=0x02 */
#endif
#endif
	writeb(0x00, S1D13806_R(0x009));
#if defined(CONFIG_SH_SOLUTION_ENGINE_2ND)
	writeb(0x00, S1D13806_R(0x010));	/* memory clock=CLKI(48Mhz) */
	writeb(0x12, S1D13806_R(0x014));	/* LCD clock=CLKI2/2(5.6448Mhz) */
#else
	writeb(0x02, S1D13806_R(0x010));	/* memory clock=CLKI3 */
	writeb(0x02, S1D13806_R(0x014));	/* LCD clock=CLKI2(25.175Mhz) */
#endif
#if defined(CONFIG_S1D13806_NTSC) || defined(CONFIG_S1D13806_LCD)
	writeb(0x82, S1D13806_R(0x018));	/* LCD/TV:0x82(with ff) */
#else
	writeb(0x02, S1D13806_R(0x018));	/* CRT:0x02(CLKI2(1:1)) */
#endif
#if defined(CONFIG_SH_SOLUTION_ENGINE_2ND)
	writeb(0x02, S1D13806_R(0x01e));	/* CPU wait MCLK-4ns > BCLK */
#else
	writeb(0x01, S1D13806_R(0x01e));	/* CPU wait 2xMCLK-4ns > BCLK */
#endif

	writeb(0x80, S1D13806_R(0x020));	/* SDRAM initalize */
	writeb(0x03, S1D13806_R(0x021));	/* SDRAM reflesh rate */
	writeb(0x00, S1D13806_R(0x02a));	/* SDRAM timing0 */
#if defined(CONFIG_SH_SOLUTION_ENGINE_2ND)
	writeb(0x01, S1D13806_R(0x02b));	/* SDRAM timing1 */
#else
	writeb(0x12, S1D13806_R(0x02b));	/* SDRAM timing1 */
#endif

	writeb(0x21, S1D13806_R(0x030));	/* LCD type (TFT color 16bitbus) */
	writeb(0x00, S1D13806_R(0x031));	/* MOD rate */

	/* LCD setting */
	writeb(par->hdisp, S1D13806_R(0x032));	/* LCD disp width */
	writeb(par->hblank, S1D13806_R(0x034));	/* LCD blank width */
	writeb(par->hstart, S1D13806_R(0x035));	/* LCD FPLINE start */
	writeb(par->hsw, S1D13806_R(0x036));	/* FPLINE width */
	writeb(par->vdisp & 0xff, S1D13806_R(0x038));	/* LCD disp height0 */
	writeb(par->vdisp >> 8, S1D13806_R(0x039));	/* LCD disp height1 */
	writeb(par->vblank, S1D13806_R(0x03a));	/* LCD blank height */
	writeb(par->vstart, S1D13806_R(0x03b));	/* LCD FPFRAME start */
	writeb(par->vsw, S1D13806_R(0x03c));	/* FPFRAME height */

	writeb(0x05, S1D13806_R(0x040));	/* LCD mode 16bpp */
	writeb(0x00, S1D13806_R(0x041));	/* 64k color mode */
	writeb(0x00, S1D13806_R(0x042));	/* LCD display memory start address0 */
	writeb(0x00, S1D13806_R(0x043));	/* LCD display memory start address1 */
	writeb(0x00, S1D13806_R(0x044));	/* LCD display memory start address2 */
	writeb(0xf0 & 0xff, S1D13806_R(0x046));	/* memory address offset0(virtual width) */
	writeb(0x00, S1D13806_R(0x047));	/* memory address offset1(virtual width) */

	writeb(0x00, S1D13806_R(0x048));	/* pan controll */
	writeb(0x00, S1D13806_R(0x04a));
	writeb(0x00, S1D13806_R(0x04b));

	/* CRT setting */
	writeb(par->hdisp, S1D13806_R(0x050));	/* CRT disp width */
	writeb(par->hblank, S1D13806_R(0x052));	/* CRT blank width */
	writeb(par->hstart, S1D13806_R(0x053));	/* CRT HRTC start */
	writeb(par->hsw, S1D13806_R(0x054));	/* HRTC width */
	writeb(par->vdisp & 0xff, S1D13806_R(0x056));	/* CRT disp width0 */
	writeb(par->vdisp >> 8, S1D13806_R(0x057));	/* CRT disp width1 */
	writeb(par->vblank, S1D13806_R(0x058));	/* CRT blank height */
	writeb(par->vstart, S1D13806_R(0x059));	/* CRT VRTC start */
	writeb(par->vsw, S1D13806_R(0x05a));	/* VRTC height */

#ifdef CONFIG_S1D13806_NTSC
	writeb(0x30, S1D13806_R(0x05b));
#else
	writeb(0x00, S1D13806_R(0x05b));
#endif
	writeb(0x05, S1D13806_R(0x060));	/* 16bpp */
	writeb(0x00, S1D13806_R(0x062));	/* CRT/TV display memory start address0 */
	writeb(0x00, S1D13806_R(0x063));	/* CRT/TV display memory start address1 */
	writeb(0x00, S1D13806_R(0x064));	/* CRT/TV display memory start address2 */
	writeb(par->x_vir & 0xff, S1D13806_R(0x066));	/* memory address offset0(virtual width) */
	writeb(par->x_vir >> 8, S1D13806_R(0x067));	/* memory address offset1(virtual width) */
	writeb(0x00, S1D13806_R(0x068));	/* pan controll */
	writeb(0x00, S1D13806_R(0x06a));
	writeb(0x00, S1D13806_R(0x06b));

	writeb(0x10, S1D13806_R(0x1f0));	/* power save disable */
#if defined(CONFIG_SH_SOLUTION_ENGINE_2ND)
	writeb(0x00, S1D13806_R(0x1f4));	/* WatchDogTimer Disable */
#endif

#ifdef CONFIG_S1D13806_NTSC
	writeb(0x06, S1D13806_R(0x1fc));	/* display enable(TV Flicker on) */
#elif defined(CONFIG_S1D13806_LCD)
#if defined(CONFIG_SH_SOLUTION_ENGINE_2ND)
	writeb((readb(S1D13806_R(0x008))) | 0x07, S1D13806_R(0x008));	/* GPIO0(DON) OutPut High */
#endif
	writeb(0x01, S1D13806_R(0x1fc));	/* display enable(LCD only) */
#else
	writeb(0x02, S1D13806_R(0x1fc));	/* display enable(CRT only) */
#endif
}

static int s1d13806fb_decode_var(const struct fb_var_screeninfo *var,
				 struct s1d13806fb_par *par,
				 const struct fb_info *info)
{
	/* data convert from var to par */

	par->hdisp = (var->xres) / 8 ? ((var->xres) / 8 - 1) : 0;
#ifdef CONFIG_S1D13806_NTSC
	par->hblank =
	    (var->hsync_len + var->left_margin + var->right_margin - 6) / 8 >
	    0 ? (var->hsync_len + var->left_margin + var->right_margin -
		 6) / 8 : 0;
	if (8 == var->bits_per_pixel)
		par->hstart =
		    ((var->right_margin) + 7) / 8 >
		    1 ? ((var->right_margin) + 7) / 8 - 1 : 0;
	else
		par->hstart =
		    ((var->right_margin) + 5) / 8 >
		    1 ? ((var->right_margin) + 5) / 8 - 1 : 0;
#else
	par->hblank =
	    (var->hsync_len + var->left_margin + var->right_margin) / 8 >
	    1 ? ((var->hsync_len + var->left_margin + var->right_margin) / 8) -
	    1 : 0;
#ifdef CONFIG_S1D13806_LCD
	if (8 == var->bits_per_pixel)
		par->hstart =
		    (var->right_margin - 5) / 8 >
		    0 ? ((var->right_margin - 5) / 8) : 0;
	else
		par->hstart =
		    (var->right_margin - 6) / 8 >
		    0 ? ((var->right_margin - 6) / 8) : 0;
#else
	if (8 == var->bits_per_pixel)
		par->hstart =
		    (var->right_margin - 4) / 8 >
		    0 ? ((var->right_margin - 4) / 8) : 0;
	else
		par->hstart =
		    (var->right_margin - 5) / 8 >
		    0 ? ((var->right_margin - 5) / 8) : 0;
#endif
#endif
	par->hsw = (var->hsync_len) / 8 > 1 ? ((var->hsync_len) / 8) - 1 : 0;

	par->vdisp = var->yres - 1;
	par->vblank =
	    var->vsync_len + var->upper_margin + var->lower_margin - 1;
	par->vstart = var->lower_margin - 1;
	par->vsw = var->vsync_len - 1;

	/* virtual display */
	par->x_vir = var->xres_virtual;
	par->y_vir = var->yres_virtual;

	return 0;
}

static int s1d13806fb_encode_var(struct fb_var_screeninfo *var,
				 struct s1d13806fb_par *par)
{
	struct fb_info info;

	*var = s1d13806fb_default;

	s1d13806fb_check_var(var, &info);

#ifdef CONFIG_S1D13806_NTSC
	var->pixclock = S1D13806_PIXCLOCK_14_318;
#elif defined(CONFIG_S1D13806_LCD)
#if defined(CONFIG_SH_SOLUTION_ENGINE_2ND)
	var->pixclock = S1D13806_PIXCLOCK_5_6448;
#else
	var->pixclock = S1D13806_PIXCLOCK_25_175;
#endif
#else
	var->pixclock = S1D13806_PIXCLOCK_25_175;
#endif

	var->activate = 0;
	return 0;

}

static struct fb_ops s1d13806fb_ops = {
	.owner = THIS_MODULE,
	.fb_open = s1d13806fb_open,
	.fb_release = s1d13806fb_release,
	.fb_check_var = s1d13806fb_check_var,
	.fb_setcolreg = s1d13806fb_setcolreg,
	.fb_blank = s1d13806fb_blank,
	.fb_fillrect = cfb_fillrect,	/* Needed !!! */
	.fb_copyarea = cfb_copyarea,	/* Needed !!! */
	.fb_imageblit = cfb_imageblit,	/* Needed !!! */
	.fb_cursor = soft_cursor,	/* Needed !!! */
};

int __init s1d13806fb_init(void)
{
	int cmap_len = 16;
	int retval;

	/*
	 * Here we set the screen_base to the vitrual memory address
	 * for the framebuffer. Usually we obtain the resource address
	 * from the bus layer and then translate it to virtual memory
	 * space via ioremap. Consult ioport.h.
	 */
	 /**/ info.node = -1;
	info.screen_base = (char *)S1D13806_FB_PHYS;
	info.fbops = &s1d13806fb_ops;
	info.fix = s1d13806fb_fix;
	info.pseudo_palette = pseudo_palette;

	info.par = &s1d13806_par;

	s1d13806fb_decode_var(&s1d13806fb_default, &s1d13806_par, &info);
	s1d13806fb_encode_var(&s1d13806fb_default, &s1d13806_par);

	s1d13806fb_ini_reg(&s1d13806_par);

	/*
	 * Set up flags to indicate what sort of acceleration your
	 * driver can provide (pan/wrap/copyarea/etc.) and whether it
	 * is a module -- see FBINFO_* in include/linux/fb.h
	 */
	info.flags = FBINFO_DEFAULT;

	/*
	 * This should give a reasonable default video mode. The following is
	 * done when we can set a video mode.
	 */

	retval = fb_find_mode(&info.var, &info, "240x320@60", NULL, 0, NULL, 8);

	if (!retval || retval == 4)
		return -EINVAL;

	/* This has to been done !!! */
	fb_alloc_cmap(&info.cmap, cmap_len, 0);

	/*
	 * The following is done in the case of having hardware with a static
	 * mode. If we are setting the mode ourselves we don't call this.
	 */
	info.var = s1d13806fb_default;

	if (register_framebuffer(&info) < 0)
		return -EINVAL;
	printk(KERN_INFO "fb%d: %s frame buffer device\n", info.node,
	       info.fix.id);
	return 0;
}

/*
 *  Cleanup
 */

static void __exit s1d13806fb_cleanup(void)
{
	/*
	 *  If your driver supports multiple boards, you should unregister and
	 *  clean up all instances.
	 */

	unregister_framebuffer(&info);
	/* ... */
}

/*
 *  Setup
 */

/*
 * Only necessary if your driver takes special options,
 * otherwise we fall back on the generic fb_setup().
 */
int __init s1d13806fb_setup(char *options)
{

	char *this_opt;

	if (!options || !*options)
		return 0;

	while ((this_opt = strsep(&options, ",")) != NULL) {
		if (!*this_opt)
			continue;
	}
	return 0;
}

/* ------------------------------------------------------------------------- */

/*
 *  Frame buffer operations
 */

/* ------------------------------------------------------------------------- */

module_init(s1d13806fb_init);
module_exit(s1d13806fb_cleanup);

MODULE_LICENSE("GPL");
