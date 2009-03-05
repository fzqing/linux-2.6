/*
 * ygv619fb.c,v 0.8 2001/02/16
 *  YGV619 video device driver for Super-H
 *
 *   Copyright (C) 2001/02/16 Mitsuharu Takei(takei-mitsuharu@hitachi-ul.co.jp)
 *   Copyright (C) Takashi Kusuda (Dec 1, 2004)
 *    Modified to support kernel 2.6.x
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/init.h>

#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/io.h>

#include "ygv619.h"

static struct fb_info ygv619fb_info;	/* use fb_info.fontname in ygv619fb_setup */
static int pseudo_palette[16];

struct ygv619fb_par {
	u32 htl;
	u32 dsc;
	u32 dec;
	u32 hsw;

	u32 vtl;
	u32 dsr;
	u32 der;
	u32 vsw;

	u32 x_vir;
	u32 y_vir;
} ygv619_par;

static struct fb_fix_screeninfo ygv619fb_fix __initdata = {
	.id = "Yamaha YGV619",
	.smem_start = YGV619_FB_PHYS,	/* Top addr of frame Buffer */
	.smem_len = YGV619_FB_PHYS_LEN,	/* Length of frame buffer */
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_TRUECOLOR,
	.xpanstep = 1,
	.ypanstep = 1,
	.ywrapstep = 0,
	.line_length = 640 * 1,	/* xres_virtual*(8bit/8) */
	.accel = FB_ACCEL_NONE,
};

#ifdef CONFIG_YGV619_NTSC
static struct fb_var_screeninfo ygv619fb_default = {
	.xres = 576,
	.yres = 432,
	.xres_virtual = 576,
	.yres_virtual = 432,
	.bits_per_pixel = 8,
	.activate = 0,
	.height = -1,
	.width = -1,
	.pixclock = YGV619_PIXCLOCK_12_272,
	.left_margin = 92,
	.right_margin = 49,
	.upper_margin = 55,
	.lower_margin = 36,
	.hsync_len = 63,
	.vsync_len = 3,
	.vmode = FB_VMODE_INTERLACED,
};
#else
static struct fb_var_screeninfo ygv619fb_default = {
	.xres = 640,
	.yres = 480,
	.xres_virtual = 640,
	.yres_virtual = 480,
	.bits_per_pixel = 8,
	.activate = 0,
	.height = -1,
	.width = -1,
	.pixclock = YGV619_PIXCLOCK_25_175,
	.left_margin = 24,
	.right_margin = 40,
	.upper_margin = 18,
	.lower_margin = 25,
	.hsync_len = 96,
	.vsync_len = 2,
	.vmode = FB_VMODE_NONINTERLACED,
};
#endif

int ygv619fb_init(void);
int ygv619fb_setup(char *);

static int ygv619fb_open(struct fb_info *info, int user)
{
	return 0;
}

static int ygv619fb_release(struct fb_info *info, int user)
{
	return 0;
}

static int ygv619fb_check_var(struct fb_var_screeninfo *var,
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
		printk(KERN_WARNING "ygv619fb: no support for %dbpp\n",
		       var->bits_per_pixel);
		return -EINVAL;
	}

	var->activate = FB_ACTIVATE_NOW;
	var->height = -1;
	var->width = -1;
	var->accel_flags = FB_ACCELF_TEXT;

	return 0;
}

static int ygv619fb_setcolreg(u_int regno, u_int red, u_int green,
			      u_int blue, u_int transp, struct fb_info *info)
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
	}

	return 0;
}

static int ygv619fb_blank(int blank, struct fb_info *info)
{
	if (blank)
		writel((readl(YGV619_R(0x04)) | 0x80000000), YGV619_R(0x04));
	else
		writel((readl(YGV619_R(0x04)) & ~0x80000000), YGV619_R(0x04));
	return 0;
}

static int ygv619fb_decode_var(const struct fb_var_screeninfo *var,
			       struct ygv619fb_par *par,
			       const struct fb_info *info)
{
	par->vtl =
	    var->vsync_len + var->upper_margin + var->yres + var->lower_margin -
	    5;
	par->htl =
	    var->hsync_len + var->left_margin + var->xres + var->right_margin -
	    3;

	par->dsr = var->vsync_len + var->upper_margin - 1;
	par->dsc = var->hsync_len + var->left_margin - 12;

	par->der = var->vsync_len + var->upper_margin + var->yres - 1 - 1;
	par->dec = var->hsync_len + var->left_margin + var->xres - 12 - 1;

	par->vsw = var->vsync_len - 1;
	par->hsw = var->hsync_len - 1;

	par->x_vir = var->xres_virtual;
	par->y_vir = var->yres_virtual;

	return 0;
}

static int ygv619fb_encode_var(struct fb_var_screeninfo *var,
			       struct ygv619fb_par *par)
{
	var->xres = par->dec - par->dsc + 1;
	var->yres = par->der - par->dsr + 1;
	var->xres_virtual = par->x_vir;
	var->yres_virtual = par->y_vir;
	var->xoffset = 0;
	var->yoffset = 0;

	var->grayscale = 0;
	var->bits_per_pixel = 8;

	var->red.offset = 0;
	var->red.length = 8;
	var->red.msb_right = 0;
	var->blue = var->green = var->red;

#ifdef CONFIG_YGV619_NTSC
	var->pixclock = YGV619_PIXCLOCK_12_272;	/* type NTSC */
#else
	var->pixclock = YGV619_PIXCLOCK_25_175;	/* type 640x480 */
#endif

	var->activate = 0;
	return 0;

}

static struct fb_ops ygv619fb_ops = {
	.owner = THIS_MODULE,
	.fb_open = ygv619fb_open,
	.fb_release = ygv619fb_release,
	.fb_check_var = ygv619fb_check_var,
	.fb_setcolreg = ygv619fb_setcolreg,
	.fb_blank = ygv619fb_blank,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_cursor = soft_cursor,
};

int ygv619fb_setup(char *options)
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

void delay(void)
{
	volatile unsigned short trash;
	trash = *(volatile unsigned short *)0xa0000000;
}

static void delay10000(void)
{
	int i;
	for (i = 0; i < 10000; i++)
		delay();
}

void ygv619fb_ini_reg(struct ygv619fb_par *par)
{
	/* set PLL */
	do {
#ifdef CONFIG_YGV619_NTSC
		writel(0x90000100, YGV619_R(0x30));
	} while (readl(YGV619_R(0x30)) != 0x90000100);	/* type NTSC */
#else
		writel(0x90000112, YGV619_R(0x30));
	} while (readl(YGV619_R(0x30)) != 0x90000112);	/* type 640x480 */
#endif

#ifdef CONFIG_YGV619_NTSC
	writel(0x06430760, YGV619_R(0x34));	/* type NTSC */
#else
	writel(0x06430fd3, YGV619_R(0x34));	/* type 640x480 */
#endif
	writel(0x00000000, YGV619_R(0x38));

	/* delay for SDRAM reset(3/15) */
	delay10000();
	delay10000();
	delay10000();

	/* VRAM Control */
	writel(0x000000dc, YGV619_R(0x00));	/* 3/14 change MPRF RFRSH */
	do {
		delay10000();
	} while (readl(YGV619_R(0x00)) & 0x8);

	/* Header Start Address */
	writel((YGV619_PA1_OFF) & ~0xff000000, YGV619_R(0x04));

	/* Data Format Control */
#ifdef CONFIG_YGV619_NTSC
	writel(0x09800000, YGV619_R(0x14));	/* type NTSC */
#else
	/*(3/15 GCKOH PCKH CVT4H=ALL 1 critical!! */
	writel(0x7c800000, YGV619_R(0x14));	/* type 640x480 */
#endif

	/* CPU I/F: Interrupt Enable & Status (3/14) */

	writel(0x00000003, YGV619_R(0x5c));
	do {
		delay10000();
	} while (readl(YGV619_R(0x5c)) & 0x8);

	/* set display size */
#ifdef CONFIG_YGV619_NTSC
	writel((((par->vtl << 15 & 0xfff0000) - 0x20000) | (par->htl -
							    1)) | 0x80004000,
	       YGV619_R(0x3c));
	writel(((par->der + 1) << 15) | (par->dec + 1), YGV619_R(0x40));	/* blank start =disp end + 1 */
	writel(((par->dsr - 1) << 15) | (par->dsc), YGV619_R(0x44));	/* blank end = disp start - 1 */
	writel( /* (par->dsr <<15 ) | (par->dsc +8) */ 0x0, YGV619_R(0x48));
	writel( /*(par->der <<15 ) | (par->dec +8) */ 0x0, YGV619_R(0x4c));
	writel(0x00001000 | (par->vsw << 8) | par->hsw, YGV619_R(0x50));	/* change 3/14 TRES */
#else
	writel(((par->vtl << 16) | par->htl) & ~0x80000000, YGV619_R(0x3c));
	writel(((par->der + 1) << 16) | (par->dec + 1), YGV619_R(0x40));	/* blank start =disp end + 1 */
	writel(((par->dsr - 1) << 16) | (par->dsc - 1), YGV619_R(0x44));	/* blank end = disp start - 1 */
	writel((par->dsr << 16) | (par->dsc + 8), YGV619_R(0x48));
	writel((par->der << 16) | (par->dec + 8), YGV619_R(0x4c));
	writel(0x80001000 | (par->vsw << 8) | par->hsw, YGV619_R(0x50));	/* change 3/14 TRES */
#endif

	/* Interrupt Enable & Status */
	writel(0x0, YGV619_R(0x5c));

	/* Drawing Processor Unit */
	writel(0x00000800, YGV619_R(0x88));

	/* Header Data Format & Description */
#ifdef CONFIG_YGV619_NTSC
	writel((par->dsr << 16) | par->dsc, YGV619_PA1_R(0x00));
	writel(((par->der + 1) << 16) | (par->dec + 1), YGV619_PA1_R(0x04));
	writel(par->x_vir, YGV619_PA1_R(0x08));
	writel(0x62000000, YGV619_PA1_R(0x0c));
	writel(YGV619_REG & 0xffffff, YGV619_PA1_R(0x10));
	writel(0x0, YGV619_PA1_R(0x14));
	writel(0x80000000 | YGV619_PAL_OFF, YGV619_PA1_R(0x18));
	writel(0x80000000, YGV619_PA1_R(0x1c));
#else
	writel((par->dsr << 16) | par->dsc, YGV619_PA1_R(0x00));
	writel(((par->der + 1) << 16) | (par->dec + 1), YGV619_PA1_R(0x04));
	writel(par->x_vir, YGV619_PA1_R(0x08));
	writel(0x20000000, YGV619_PA1_R(0x0c));
	writel(0x0, YGV619_PA1_R(0x10));
	writel(0x0, YGV619_PA1_R(0x14));
	writel(0x80000000 | YGV619_PAL_OFF, YGV619_PA1_R(0x18));
	writel(0x80000000, YGV619_PA1_R(0x1c));
#endif

	/* Drawing Processor Unit (add 3/14) */
	writel(0x00000000, YGV619_R(0x88));

}

int __init ygv619fb_init(void)
{
	int retval;
	int cmap_len = 16;

	/*
	 * Here we set the screen_base to the vitrual memory address
	 * for the framebuffer. Usually we obtain the resource address
	 * from the bus layer and then translate it to virtual memory
	 * space via ioremap. Consult ioport.h.
	 */
	/* set fb_info */
	ygv619fb_info.node = -1;
	ygv619fb_info.screen_base = (char *)YGV619_FB_PHYS;
	ygv619fb_info.fbops = &ygv619fb_ops;
	ygv619fb_info.fix = ygv619fb_fix;
	ygv619fb_info.pseudo_palette = pseudo_palette;

	ygv619fb_info.par = &ygv619_par;

	ygv619fb_decode_var(&ygv619fb_default, &ygv619_par, &ygv619fb_info);
	ygv619fb_encode_var(&ygv619fb_default, &ygv619_par);
	ygv619fb_ini_reg(&ygv619_par);

	/*
	 * Set up flags to indicate what sort of acceleration your
	 * driver can provide (pan/wrap/copyarea/etc.) and whether it
	 * is a module -- see FBINFO_* in include/linux/fb.h
	 */
	ygv619fb_info.flags = FBINFO_DEFAULT;

	/*
	 * This should give a reasonable default video mode. The following is
	 * done when we can set a video mode.
	 */
	retval =
	    fb_find_mode(&ygv619fb_info.var, &ygv619fb_info, "640x480@60", NULL,
			 0, NULL, 8);

	if (!retval || retval == 4)
		return -EINVAL;

	/* This has to been done !!! */
	fb_alloc_cmap(&ygv619fb_info.cmap, cmap_len, 0);

	/*
	 * The following is done in the case of having hardware with a static
	 * mode. If we are setting the mode ourselves we don't call this.
	 */
	ygv619fb_info.var = ygv619fb_default;

	if (register_framebuffer(&ygv619fb_info) < 0) {
		printk(KERN_ERR "ygv619fb.c: register_framebuffer failed\n");
		return -EINVAL;
	}

	printk(KERN_INFO "fb%d: %s frame buffer device\n", ygv619fb_info.node,
	       ygv619fb_info.fix.id);

	return 0;
}

static void __exit ygv619fb_cleanup(void)
{
	/*
	 *  If your driver supports multiple boards, you should unregister and
	 *  clean up all instances.
	 */

	unregister_framebuffer(&ygv619fb_info);
}

module_init(ygv619fb_init);
module_exit(ygv619fb_cleanup);
