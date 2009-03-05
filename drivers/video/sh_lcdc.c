/*
 *  SH LCD control device driver (16bit color only)
 *
 *   Copyright(C) 2002/06/05 Mitsuharu Takei(takei-mitsuharu@hitachi-ul.co.jp)
 *	Modified to support SH7727 LCDC
 *   Copyright(C) 2004/01/22 Takashi Kusuda
 *	Modified to support SH7720 LCDC
 *   Copyright(C) 2004/11/25 Takashi Kusuda
 *	Modified to support kernel 2.6
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
#include <asm/irq.h>

#include "sh_lcdc.h"

struct sh_lcdcfb_par {
	u16 htcn;
	u16 hdcn;
	u16 hsynp;
	u16 hsynw;

	u16 vtln;
	u16 vdln;
	u16 vsynp;
	u16 vsynw;

	u32 vxres;
	u32 vyres;
} sh_lcdc_par;

static int pseudo_palette[16];

#if defined(CONFIG_SH_SOLUTION_ENGINE_2ND) || defined(CONFIG_SH_SOLUTION_ENGINE_LIGHT)
#if defined(CONFIG_MSTLCD01)	/* MSTLCD01 version */

#if defined(CONFIG_MSTLCD01_QVGA)	/* QVGA */
/* size=240x320, color=16bit */
static struct fb_fix_screeninfo sh_lcdcfb_fix __initdata = {
	.id = "SuperH LCDC",
	.smem_start = SH_LCDC_FB_PHYS,	/* Top addr of frame Buffer */
	.smem_len = SH_LCDC_FB_PHYS_LEN,	/* Length of frame buffer */
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_TRUECOLOR,
	.xpanstep = 1,
	.ypanstep = 1,
	.ywrapstep = 0,
	.line_length = 240 * 2,	/* xres_virtual*(16bit/8) */
	.accel = FB_ACCEL_NONE,
};
static struct fb_var_screeninfo sh_lcdcfb_default = {
	.xres = 240,
	.yres = 320,
	.xres_virtual = 240,
	.yres_virtual = 320,
	.bits_per_pixel = 16,
	.activate = 0,
	.height = -1,
	.width = -1,
	.pixclock = SH_LCDC_PIXCLOCK_25_175,
	.left_margin = 32,
	.right_margin = 40,
	.upper_margin = 1,
	.lower_margin = 1,
	.hsync_len = 8,
	.vsync_len = 2,
	.vmode = FB_VMODE_NONINTERLACED,
};
#else				/* VGA */
/* size=480x640, color=16bit */
static struct fb_fix_screeninfo sh_lcdcfb_fix __initdata = {
	.id = "SuperH LCDC",
	.smem_start = SH_LCDC_FB_PHYS,	/* Top addr of frame Buffer */
	.smem_len = SH_LCDC_FB_PHYS_LEN,	/* Length of frame buffer */
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_TRUECOLOR,
	.xpanstep = 1,
	.ypanstep = 1,
	.ywrapstep = 0,
	.line_length = 480 * 2,	/* xres_virtual*(16bit/8) */
	.accel = FB_ACCEL_NONE,
};
static struct fb_var_screeninfo sh_lcdcfb_default = {
	.xres = 480,
	.yres = 640,
	.xres_virtual = 480,
	.yres_virtual = 640,
	.bits_per_pixel = 16,
	.activate = 0,
	.height = -1,
	.width = -1,
	.pixclock = SH_LCDC_PIXCLOCK_25_175,
	.left_margin = 64,
	.right_margin = 88,
	.upper_margin = 0,
	.lower_margin = 5,
	.hsync_len = 16,
	.vsync_len = 3,
	.vmode = FB_VMODE_NONINTERLACED,
};
#endif

#else				/* !CONFIG_MSTLCD01 */
/* size=240x320, color=16bit */
static struct fb_fix_screeninfo sh_lcdcfb_fix __initdata = {
	.id = "SuperH LCDC",
	.smem_start = SH_LCDC_FB_PHYS,	/* Top addr of frame Buffer */
	.smem_len = SH_LCDC_FB_PHYS_LEN,	/* Length of frame buffer */
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_TRUECOLOR,
	.xpanstep = 1,
	.ypanstep = 1,
	.ywrapstep = 0,
	.line_length = 240 * 2,	/* xres_virtual*(16bit/8) */
	.accel = FB_ACCEL_NONE,
};

static struct fb_var_screeninfo sh_lcdcfb_default = {
	.xres = 240,
	.yres = 320,
	.xres_virtual = 240,
	.yres_virtual = 320,
	.bits_per_pixel = 16,
	.activate = 0,
	.height = -1,
	.width = -1,
	.pixclock = SH_LCDC_PIXCLOCK_25_175,
	.left_margin = 8,
	.right_margin = 8,
	.upper_margin = 8,
	.lower_margin = 2,
	.hsync_len = 8,
	.vsync_len = 2,
	.vmode = FB_VMODE_NONINTERLACED,
};
#endif

#else				/* !CONFIG_SH_SOLUTION_ENGINE_2ND && !CONFIG_SH_SOLUTION_ENGINE_LIGHT */
/* Default size=640x480, color=16bit */
static struct fb_fix_screeninfo sh_lcdcfb_fix __initdata = {
	.id = "SuperH LCDC",
	.smem_start = SH_LCDC_FB_PHYS,	/* Top addr of frame Buffer */
	.smem_len = SH_LCDC_FB_PHYS_LEN,	/* Length of frame buffer */
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_TRUECOLOR,
	.xpanstep = 1,
	.ypanstep = 1,
	.ywrapstep = 0,
	.line_length = 640 * 2,	/* xres_virtual*(16bit/8) */
	.accel = FB_ACCEL_NONE,
};
static struct fb_var_screeninfo sh_lcdcfb_default = {
	/* 16 bpp */
	.xres = 640,
	.yres = 480,
	.xres_virtual = 640,
	.yres_virtual = 480,
	.bits_per_pixel = 16,
	.activate = 0,
	.height = -1,
	.width = -1,
	.pixclock = SH_LCDC_PIXCLOCK_25_175,
	.left_margin = 144,
	.right_margin = 8,
	.upper_margin = 23,
	.lower_margin = 16,
	.hsync_len = 8,
	.vsync_len = 6,
	.vmode = FB_VMODE_NONINTERLACED,
};
#endif

static struct fb_info sh_lcdcfb_info;

int sh_lcdcfb_init(void);
int sh_lcdcfb_setup(char *);

static int sh_lcdcfb_open(const struct fb_info *info, int user)
{
	return 0;
}

static int sh_lcdcfb_release(const struct fb_info *info, int user)
{
	return 0;
}

static int sh_lcdcfb_check_var(struct fb_var_screeninfo *var,
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
		printk(KERN_WARNING "sh_lcdcfb: no support for %dbpp\n",
		       var->bits_per_pixel);
		return -EINVAL;
	}

	var->activate = FB_ACTIVATE_NOW;
	var->height = -1;
	var->width = -1;
	var->accel_flags = FB_ACCELF_TEXT;

	return 0;
}

static int sh_lcdcfb_setcolreg(unsigned regno, unsigned red, unsigned green,
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
	}

	return 0;
}

static int sh_lcdcfb_blank(int blank, struct fb_info *info)
{
	if (blank)
		writew(0x0, SH_LCDC_LDCNTR);
	else
		writew(0x11, SH_LCDC_LDCNTR);
	return 0;
}

void sh_lcdcfb_regs_init(struct sh_lcdcfb_par *par)
{
	/* multiplex initialize */
#if defined(CONFIG_CPU_SUBTYPE_SH7720)
	writew(0x0000, PORT_PCCR);
	writew(0x0000, PORT_PDCR);
	writew(0xA800, PORT_PECR);
	writew(0x0000, PORT_PVCR);
#elif defined(CONFIG_CPU_SUBTYPE_SH7727)
	writew(0x0000, PORT_PCCR);
	writew((readw(PORT_PDCR)) & 0x0300, PORT_PDCR);
	writew((readw(PORT_PECR)) & 0xcf3f, PORT_PECR);
	writew((readw(PORT_PHCR)) & 0x3fff, PORT_PHCR);
	writew((readw(PORT_PMCR)) & 0xff00, PORT_PMCR);
#endif

#if defined(CONFIG_MSTLCD01)	/* MSTLCD01 version */
	writew(0xc02b, SH_LCDC_LDMTR);
	writew(0x012d, SH_LCDC_LDDFR);	/* little endian, 16bit-color(5-6-5) */
	writew(0x0200, SH_LCDC_LDSMR);	/* rotate disable, 16burst */
	writew(0x0, SH_LCDC_LDPALCR);	/* palette control display mode(default) */
#if defined(CONFIG_MSTLCD01_QVGA)	/* QVGA */
	writew(0x0108, SH_LCDC_LDICKR);	/* use CKIO, CKIO/8 */
#else				/* VGA */
	writew(0x0102, SH_LCDC_LDICKR);	/* use CKIO, CKIO/2 */
#endif
	writew(par->vxres * 2, SH_LCDC_LDLAOR);
	writew((par->hdcn << 8) | (par->htcn & 0xff), SH_LCDC_LDHCNR);
	writew((par->hsynw << 12) | (par->hsynp & 0xff), SH_LCDC_LDHSYNR);
	writew(par->vdln, SH_LCDC_LDVDLNR);
	writew(par->vtln, SH_LCDC_LDVTLNR);
	writew((par->vsynw << 12) | (par->vsynp & 0x7ff), SH_LCDC_LDVSYNR);

	writel(SH_LCDC_FB_PHYS & 0x3ffffff, SH_LCDC_LDSARU);
	writel(SH_LCDC_FB_PHYS & 0x3ffffff, SH_LCDC_LDSARL);	/* if TFT or STN, not use */
	writew(0x000c, SH_LCDC_LDACLNR);
	writew(0x0000, SH_LCDC_LDINTR);
	writew(0xff70, SH_LCDC_LDPMMR);
	writew(0x0500, SH_LCDC_LDPSPR);

	writew(0x0011, SH_LCDC_LDCNTR);
#else				/* old type LCD */
	/* LCD CONTROL registers */
	writew(0x0108, SH_LCDC_LDICKR);	/* 33Mhz/8? */
	writew(0xc02b, SH_LCDC_LDMTR);
	writew(0x012d, SH_LCDC_LDDFR);
	writew(0x0200, SH_LCDC_LDSMR);
	writel(SH_LCDC_FB_PHYS & 0x3ffffff, SH_LCDC_LDSARU);
	writel(SH_LCDC_FB_PHYS & 0x3ffffff, SH_LCDC_LDSARL);
	writew(par->vxres * 2, SH_LCDC_LDLAOR);
	writew(0x0, SH_LCDC_LDPALCR);
	writew((par->hdcn << 8) | (par->htcn & 0xff), SH_LCDC_LDHCNR);
	writew((par->hsynw << 12) | (par->hsynp & 0xff), SH_LCDC_LDHSYNR);
	writew(par->vdln, SH_LCDC_LDVDLNR);
	writew(par->vtln, SH_LCDC_LDVTLNR);
	writew((par->vsynw << 12) | (par->vsynp & 0x7ff), SH_LCDC_LDVSYNR);
	writew(0x000c, SH_LCDC_LDACLNR);
	writew(0x0000, SH_LCDC_LDINTR);
	writew(0xff70, SH_LCDC_LDPMMR);
	writew(0x0500, SH_LCDC_LDPSPR);
	writew(0x0011, SH_LCDC_LDCNTR);
#endif
}

static int sh_lcdcfb_decode_var(const struct fb_var_screeninfo *var,
				struct sh_lcdcfb_par *par,
				const struct fb_info *info)
{
	par->hdcn = var->xres > 7 ? var->xres / 8 - 1 : 0;
	par->hsynp =
	    var->xres + var->right_margin >
	    7 ? (var->xres + var->right_margin) / 8 - 1 : 0;
	par->hsynw = var->hsync_len > 7 ? var->hsync_len / 8 - 1 : 0;
	par->htcn =
	    var->xres + var->right_margin + var->hsync_len + var->left_margin >
	    7 ? (var->xres + var->right_margin + var->hsync_len +
		 var->left_margin) / 8 - 1 : 0;

	par->vdln = var->yres - 1;
	par->vsynp = var->yres + var->lower_margin - 1;
	par->vsynw = var->vsync_len - 1;
	par->vtln =
	    var->yres + var->lower_margin + var->vsync_len + var->upper_margin -
	    2;

	par->vxres = var->xres_virtual;
	par->vyres = var->yres_virtual;

	return 0;
}

static int sh_lcdcfb_encode_var(struct fb_var_screeninfo *var,
				struct sh_lcdcfb_par *par)
{
	struct fb_info info;

	*var = sh_lcdcfb_default;

	sh_lcdcfb_check_var(var, &info);

	var->pixclock = SH_LCDC_PIXCLOCK_25_175;
	var->activate = 0;

	return 0;
}

static struct fb_ops sh_lcdcfb_ops = {
	.owner = THIS_MODULE,
	.fb_open = sh_lcdcfb_open,
	.fb_release = sh_lcdcfb_release,
	.fb_check_var = sh_lcdcfb_check_var,
	.fb_setcolreg = sh_lcdcfb_setcolreg,
	.fb_blank = sh_lcdcfb_blank,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_cursor = soft_cursor,
};

int __init sh_lcdcfb_init(void)
{
	int cmap_len = 16;
	int retval;

	/*
	 * Here we set the screen_base to the vitrual memory address
	 * for the framebuffer. Usually we obtain the resource address
	 * from the bus layer and then translate it to virtual memory
	 * space via ioremap. Consult ioport.h.
	 */
	 /**/ sh_lcdcfb_info.node = -1;
	sh_lcdcfb_info.screen_base = (char *)SH_LCDC_FB_PHYS;
	sh_lcdcfb_info.fbops = &sh_lcdcfb_ops;
	sh_lcdcfb_info.fix = sh_lcdcfb_fix;
	sh_lcdcfb_info.pseudo_palette = pseudo_palette;

	sh_lcdcfb_info.par = &sh_lcdc_par;

	sh_lcdcfb_decode_var(&sh_lcdcfb_default, &sh_lcdc_par, &sh_lcdcfb_info);
	sh_lcdcfb_encode_var(&sh_lcdcfb_default, &sh_lcdc_par);
	sh_lcdcfb_regs_init(&sh_lcdc_par);

	/*
	 * Set up flags to indicate what sort of acceleration your
	 * driver can provide (pan/wrap/copyarea/etc.) and whether it
	 * is a module -- see FBINFO_* in include/linux/fb.h
	 */
	sh_lcdcfb_info.flags = FBINFO_DEFAULT;

	/*
	 * This should give a reasonable default video mode. The following is
	 * done when we can set a video mode.
	 */
#if defined(CONFIG_SH_SOLUTION_ENGINE_LIGHT) || defined(CONFIG_SH_SOLUTION_ENGINE_2ND)
	retval =
	    fb_find_mode(&sh_lcdcfb_info.var, &sh_lcdcfb_info, "240x320@60",
			 NULL, 0, NULL, 8);
#else
	retval =
	    fb_find_mode(&sh_lcdcfb_info.var, &sh_lcdcfb_info, "640x480@60",
			 NULL, 0, NULL, 8);
#endif

	if (!retval || retval == 4)
		return -EINVAL;

	/* This has to been done !!! */
	fb_alloc_cmap(&sh_lcdcfb_info.cmap, cmap_len, 0);

	/*
	 * The following is done in the case of having hardware with a static
	 * mode. If we are setting the mode ourselves we don't call this.
	 */
	sh_lcdcfb_info.var = sh_lcdcfb_default;

	if (register_framebuffer(&sh_lcdcfb_info) < 0)
		return -EINVAL;
	printk(KERN_INFO "fb%d: %s frame buffer device\n", sh_lcdcfb_info.node,
	       sh_lcdcfb_info.fix.id);
	return 0;
}

static void __exit sh_lcdcfb_cleanup(void)
{
	/*
	 *  If your driver supports multiple boards, you should unregister and
	 *  clean up all instances.
	 */

	unregister_framebuffer(&sh_lcdcfb_info);
}

int __init sh_lcdcfb_setup(char *options)
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

module_init(sh_lcdcfb_init);
module_exit(sh_lcdcfb_cleanup);
