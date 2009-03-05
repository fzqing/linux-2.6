/*
 *  linux/drivers/video/pxafb.c
 *
 *  Copyright (C) 1999 Eric A. Thomas.
 *  Copyright (C) 2004 Jean-Frederic Clere.
 *  Copyright (C) 2004 Ian Campbell.
 *  Copyright (C) 2004 Jeff Lackey.
 *   Based on sa1100fb.c Copyright (C) 1999 Eric A. Thomas
 *  which in turn is
 *   Based on acornfb.c Copyright (C) Russell King.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	Intel PXA250/210/27X/3XX LCD Controller Frame Buffer Driver
 *
 * Please direct your questions and comments on this driver to the following
 * email address:
 *
 *	linux-arm-kernel@lists.arm.linux.org.uk
 *
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

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/bitfield.h>
#include <asm/arch/pxafb.h>

#include "console/fbcon.h"
/*
 * Complain if VAR is out of range.
 */
#define DEBUG_VAR 1

#include "pxafb.h"

/* Bits which should not be set in machine configuration structures */
#define LCCR0_INVALID_CONFIG_MASK (LCCR0_OUM|LCCR0_BM|LCCR0_QDM|LCCR0_DIS|LCCR0_EFM|LCCR0_IUM|LCCR0_SFM|LCCR0_LDM|LCCR0_ENB)
#define LCCR3_INVALID_CONFIG_MASK (LCCR3_HSP|LCCR3_VSP|LCCR3_PCD|LCCR3_BPP)

static void (*pxafb_backlight_power)(int);
static void (*pxafb_lcd_power)(int);

static int pxafb_activate_var(struct fb_var_screeninfo *var, struct pxafb_info *);
static void set_ctrlr_state(struct pxafb_info *fbi, u_int state);

#if defined(CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
static int overlay1fb_disable(struct overlayfb_info *fbi);
static int overlay1fb_enable(struct overlayfb_info *fbi);
static int overlay1fb_release(struct fb_info *info, int user);
static int overlay1fb_open(struct fb_info *info, int user);
static int overlay2fb_enable(struct overlayfb_info *fbi);
static int overlay2fb_open(struct fb_info *info, int user);
static int overlay2fb_release(struct fb_info *info, int user);
#ifdef CONFIG_PXA27x
static int overlay2fb_enable_RGB(struct fb_info *info);
static int overlay2fb_disable_RGB(struct fb_info *info);
static int overlay2fb_YUV420_workaround(struct fb_info *info);
static int overlay2fb_enable_YUV420(struct overlayfb_info *fbi);
static int overlay2fb_disable_YUV420(struct fb_info *info);
#endif
static int overlay2fb_activate_var(struct fb_var_screeninfo *var,
				   struct fb_info *info);
static int cursorfb_enable(struct fb_info *info);
static void cursorfb_disable(struct fb_info *info);
static int cursorfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			      u_int trans, struct fb_info *info);
static int cursorfb_activate_var(struct fb_var_screeninfo *var,
				 struct fb_info *info);

static void overlay1fb_blank(int blank, struct fb_info *info);
static void overlay2fb_blank(int blank, struct fb_info *info);
#endif /*CONFIG_PXA27x, CONFIG_PXA3xx */

int pxafb_probe(struct device *dev);
#ifdef CONFIG_PM
static int pxafb_suspend(struct device *dev, u32 state, u32 level);
static int pxafb_resume(struct device *dev, u32 level);
#endif

#ifdef CONFIG_FB_PXA_PARAMETERS
#define PXAFB_OPTIONS_SIZE 256
static char g_options[PXAFB_OPTIONS_SIZE] __initdata = "";
#endif

#if defined(CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
static int overlay1fb_activate_var(struct fb_var_screeninfo *var,
				   struct fb_info *info);
static int overlay2fb_disable(struct fb_info *info);
#endif /* CONFIG_PXA27x CONFIG_PXA3xx */
static void pxafb_init_unchangeable_vars(struct fb_var_screeninfo *var,
				      struct device *dev);

#if defined(CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
static int directfb = 0;
#endif
#if defined(CONFIG_PXA27x)
static int yuv420_enabled = 0;
#endif

static struct device *my_device;	/* We preserve reference to the device here */
static struct device_driver pxafb_driver;
static struct pxafb_info *pxafbi;

static struct device_driver pxafb_driver = {
	.name = "pxa2xx-fb",	/* the name same as in generic.c ! */
	.bus = &platform_bus_type,
	.probe = pxafb_probe,

#ifdef CONFIG_PM
	.suspend = pxafb_suspend,
	.resume = pxafb_resume,
#endif
};

/*
 * IMHO this looks wrong.  In 8BPP, length should be 8.
 */
/*
static struct pxafb_rgb rgb_8 = {
      .red = {.offset = 0, .length = 4,},
      .green = {.offset = 0, .length = 4,},
      .blue = {.offset = 0, .length = 4,},
      .transp = {.offset = 0, .length = 0,},
};

static struct pxafb_rgb def_rgb_16 = {
      .red = {.offset = 11, .length = 5,},
      .green = {.offset = 5, .length = 6,},
      .blue = {.offset = 0, .length = 5,},
      .transp = {.offset = 0, .length = 0,},
};
*/

#if defined(CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
/* 16bpp, format 4 */
static struct pxafb_rgb def_rgbt_16 = {
      .red = {.offset = 10, .length = 5,},
      .green = {.offset = 5, .length = 5,},
      .blue = {.offset = 0, .length=5,},
      .transp = {.offset =15, .length = 1,},
};

static struct pxafb_rgb def_rgb_18 = {
      .red = {.offset = 12, .length = 6,},
      .green = {.offset = 6, .length = 6,},
      .blue = {.offset = 0, .length = 6,},
      .transp = {.offset = 0, .length = 0,},
};

static struct pxafb_rgb def_rgbt_18 = {
      .red = {.offset = 12, .length = 6,},
      .green = {.offset = 6, .length = 6,},
      .blue = {.offset = 0, .length = 6,},
      .transp = {.offset = 0, .length = 0,},
};

static struct pxafb_rgb def_rgbt_19 = {
      .red = {.offset = 12, .length = 6,},
      .green = {.offset = 6, .length = 6,},
      .blue = {.offset = 0, .length = 6,},
      .transp = {.offset = 18, .length = 1,},
};

static struct pxafb_rgb def_rgbt_24 = {
      .red = {.offset = 16, .length = 8,},
      .green = {.offset = 8, .length = 8,},
      .blue = {.offset = 0, .length = 8,},
      .transp = {.offset = 0, .length = 0,},
};

static struct pxafb_rgb def_rgbt_25 = {
      .red = {.offset = 16, .length = 8,},
      .green = {.offset = 8, .length = 8,},
      .blue = {.offset = 0, .length = 8,},
      .transp = {.offset = 24, .length = 1,},
};

/*
 * Hardware cursor support
 */
/* Bulverde Cursor Modes */
struct cursor_mode {
	int xres;
	int yres;
	int bpp;
};

static struct cursor_mode cursor_modes[] = {
	{32, 32, 2},
	{32, 32, 2},
	{32, 32, 2},
	{64, 64, 2},
	{64, 64, 2},
	{64, 64, 2},
	{128, 128, 1},
	{128, 128, 1}
};
#endif				/* CONFIG_PXA27x */

#if 0
static void printk_state(int state)
{
	if (state == C_DISABLE) {
		printk("C_DISABLE");
	} else if (state == C_ENABLE) {
		printk("C_ENABLE");
	}

}
#endif

#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
static void disable_overlays(struct pxafb_info *fbi)
{
	if (directfb)
		return;

	if (fbi->overlay1fb && fbi->overlay1fb->state == C_ENABLE) {
		overlay1fb_disable(fbi->overlay1fb);
		fbi->overlay1fb->state = C_ENABLE;
	}

	if (fbi->overlay2fb && fbi->overlay2fb->state == C_ENABLE) {
		overlay2fb_disable((struct fb_info *)fbi->overlay2fb);
		fbi->overlay2fb->state = C_ENABLE;
	}
	if (fbi->cursorfb && fbi->cursorfb->state == C_ENABLE) {
		cursorfb_disable((struct fb_info *)fbi->cursorfb);
		fbi->cursorfb->state = C_ENABLE;
	}

}

static void enable_overlays(struct pxafb_info *fbi)
{
	if (directfb)
		return;

	if (fbi->overlay1fb && fbi->overlay1fb->state == C_ENABLE) {
		fbi->overlay1fb->state = C_DISABLE;
		overlay1fb_enable(fbi->overlay1fb);
	}
	if (fbi->overlay2fb && fbi->overlay2fb->state == C_ENABLE) {
		fbi->overlay2fb->state = C_DISABLE;
		overlay2fb_enable(fbi->overlay2fb);
	}
	if (fbi->cursorfb && fbi->cursorfb->state == C_ENABLE) {
		fbi->cursorfb->state = C_DISABLE;
		cursorfb_enable((struct fb_info *)fbi->cursorfb);
	}
}
#endif  /*CONFIG_PXA27x*/

#define CLEAR_LCD_INTR(reg, intr) do {  \
        reg = (intr);                   \
}while(0)

#define WAIT_FOR_LCD_INTR(reg,intr,timeout) ({  \
        int __done =0;                          \
        int __t = timeout;                      \
        while (__t) {                           \
                __done = (reg) & (intr);        \
                if (__done) break;              \
                msleep(10);                     \
                __t--;                          \
        }                                       \
        __done;                                 \
})

static inline void pxafb_schedule_work(struct pxafb_info *fbi, u_int state)
{
	unsigned long flags;

	/*
	 * We need to handle two requests being made at the same time.
	 * There are two important cases:
	 *  1. When we are changing VT (C_REENABLE) while unblanking (C_ENABLE)
	 *     We must perform the unblanking, which will do our REENABLE for us.
	 *  2. When we are blanking, but immediately unblank before we have
	 *     blanked.  We do the "REENABLE" thing here as well, just to be sure.
	 */

	/*use 1 to enable Kernel turn off the LCD or 0 to dsiable it*/
	spin_lock_irqsave(&fbi->rt_lock, flags);
	if (fbi->task_state == C_ENABLE && state == C_REENABLE)
		state = (u_int) -1;
	if (fbi->task_state == C_DISABLE && state == C_ENABLE)
		state = C_REENABLE;

	if (state != (u_int)-1) {
		fbi->task_state = state;
		schedule_work(&fbi->task);
	}
	spin_unlock_irqrestore(&fbi->rt_lock, flags);
}

static inline u_int chan_to_field(u_int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}


static int pxafb_wait_for_eof(struct pxafb_info *fbi, u_int ch_mask)
{
	int ret;

#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
	/* must be channels 0-6 */
	if (ch_mask & ~0x7f)
		return -EINVAL;
#else
	/* must be channel 0 */
	if (ch_mask & ~0x1)
		return -EINVAL;
#endif

	down(&fbi->ctrlr_sem);

	fbi->eof_mask = 0;

	if (ch_mask & 1) {
		/* channel 0 */
		CLEAR_LCD_INTR(LCSR0, LCSR_EOF); /* clear */
		LCCR0 &= ~LCCR0_EFM; /* unmask */
	}

#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
	if (ch_mask & 0x7e) {
		/* channels 1-6 */
		CLEAR_LCD_INTR(LCSR1, (0x3f << 8)); /* clear 1-6 */
		LCCR5 &= ~((ch_mask >> 1) << 8); /* unmask */
	}
#endif

	ret = wait_event_interruptible_timeout(
		fbi->eof_wait, (fbi->eof_mask & ch_mask) == ch_mask, HZ/10);

	if (ret == 0)
		ret = -ETIMEDOUT;
	else if (ret > 0)
		ret = 0;

	up(&fbi->ctrlr_sem);
	return ret;
}

static int
pxafb_setpalettereg(u_int regno, u_int red, u_int green, u_int blue,
		       u_int trans, struct fb_info *info)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	u_int val, ret = 1;

	if (regno < fbi->palette_size) {
		if (fbi->fb.var.grayscale) {
			val = ((blue >> 8) & 0x00ff);
		} else {
			val  = ((red   >>  0) & 0xf800);
			val |= ((green >>  5) & 0x07e0);
			val |= ((blue  >> 11) & 0x001f);
		}
		fbi->palette_cpu[regno] = val;
		ret = 0;
	}
	return ret;
}

static int
pxafb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
		   u_int trans, struct fb_info *info)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	unsigned int val;
	int ret = 1;

	/*
	 * If inverse mode was selected, invert all the colours
	 * rather than the register number.  The register number
	 * is what you poke into the framebuffer to produce the
	 * colour you requested.
	 */
	if (fbi->cmap_inverse) {
		red   = 0xffff - red;
		green = 0xffff - green;
		blue  = 0xffff - blue;
	}

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no matter what visual we are using.
	 */
	if (fbi->fb.var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
					7471 * blue) >> 16;
	switch (fbi->fb.fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/*
		 * 16-bit True Colour.  We encode the RGB value
		 * according to the RGB bitfield information.
		 */
		if (regno < 16) {
			u32 *pal = fbi->fb.pseudo_palette;

			val  = chan_to_field(red, &fbi->fb.var.red);
			val |= chan_to_field(green, &fbi->fb.var.green);
			val |= chan_to_field(blue, &fbi->fb.var.blue);

			pal[regno] = val;
			ret = 0;
		}
		break;

	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		ret = pxafb_setpalettereg(regno, red, green, blue, trans, info);
		break;
	}

	return ret;
}

/*
 *  pxafb_bpp_to_lccr3():
 *    Convert a bits per pixel value to the correct bit pattern for LCCR3
 */
static int pxafb_bpp_to_lccr3(struct fb_var_screeninfo *var)
{
        int ret = 0;
        switch (var->bits_per_pixel) {
        case 1:  ret = LCCR3_1BPP; break;
        case 2:  ret = LCCR3_2BPP; break;
        case 4:  ret = LCCR3_4BPP; break;
        case 8:  ret = LCCR3_8BPP; break;
        case 16: ret = LCCR3_16BPP; break;
        }
        return ret;
}

#ifdef CONFIG_CPU_FREQ
/*
 *  pxafb_display_dma_period()
 *    Calculate the minimum period (in picoseconds) between two DMA
 *    requests for the LCD controller.  If we hit this, it means we're
 *    doing nothing but LCD DMA.
 */
static unsigned int pxafb_display_dma_period(struct fb_var_screeninfo *var)
{
       /*
        * Period = pixclock * bits_per_byte * bytes_per_transfer
        *              / memory_bits_per_pixel;
        */
       return var->pixclock * 8 * 16 / var->bits_per_pixel;
}

extern unsigned int get_clk_frequency_khz(int info);
#endif

/*
 *  pxafb_check_var():
 *    Get the video params out of 'var'. If a value doesn't fit, round it up,
 *    if it's too big, return -EINVAL.
 *
 *    Round up in the following order: bits_per_pixel, xres,
 *    yres, xres_virtual, yres_virtual, xoffset, yoffset, grayscale,
 *    bitfields, horizontal timing, vertical timing.
 */
static int pxafb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;

	pxafb_init_unchangeable_vars(&fbi->fb.var, my_device);

	pxafb_init_unchangeable_vars(var, my_device);

	if (var->xres < MIN_XRES)
		var->xres = MIN_XRES;
	if (var->yres < MIN_YRES)
		var->yres = MIN_YRES;
	if (var->xres > MAX_XRES)
		var->xres = MAX_XRES;
	if (var->yres > MAX_YRES)
		var->yres = MAX_YRES;

	var->xres_virtual = max(var->xres_virtual, var->xres);
	var->yres_virtual = max(var->yres_virtual, var->yres);
	if (var->xres_virtual > MAX_XRES)
		var->xres_virtual = MAX_XRES;
	if (var->xres_virtual * var->yres_virtual * var->bits_per_pixel/8 >
	    info->fix.smem_len)
		var->yres_virtual = MAX_YRES;

        /*
	 * Setup the RGB parameters for this display.
	 *
	 * The pixel packing format is described on page 7-11 of the
	 * PXA2XX Developer's Manual.
	 *
	 *Overlays and cursor are disabled with base plane bpp=1,2
	 *(see 4.7.6 paperback Intel's Bulvrde secret manual)
         */

	if (var->bits_per_pixel == 16) {
		var->red.offset   = 11; var->red.length   = 5;
		var->green.offset = 5;  var->green.length = 6;
		var->blue.offset  = 0;  var->blue.length  = 5;
		var->transp.offset = var->transp.length = 0;
	} else {
		var->red.offset = var->green.offset = var->blue.offset = var->transp.offset = 0;
		var->red.length   = 8;
		var->green.length = 8;
		var->blue.length  = 8;
		var->transp.length = 0;
	}

#ifdef CONFIG_CPU_FREQ
	DPRINTK("dma period = %d ps, clock = %d kHz\n",
		pxafb_display_dma_period(var),
		get_clk_frequency_khz(0));
#endif
	return 0;
}

/*
 *  overlay1fb_check_var():
 *    Get the video params out of 'var'. If a value doesn't fit, round it up,
 *    if it's too big, return -EINVAL.
 *
 */
#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
static int overlay1fb_check_var(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info *)info;
	int xpos = 0, ypos = 0;
	int base_xres, base_yres;

	base_xres = fbi->basefb->fb.var.xres;
	base_yres = fbi->basefb->fb.var.yres;

	xpos = (var->nonstd & 0x3ff);
	ypos = ((var->nonstd >> 10) & 0x3ff);

	if (var->xres > base_xres)
		var->xres = base_xres;
	if (var->yres > base_yres)
		var->yres = base_yres;

	if ((xpos + var->xres) > base_xres) {
		xpos = base_xres - var->xres;
		if (xpos < 0)
			xpos = 0;
	}

	if ((ypos + var->yres) > base_yres) {
		ypos = base_yres - var->yres;
		if (ypos < 0)
			ypos = 0;
	}

	/*set xpos and ypos back to variable params */
	var->nonstd = (ypos << 10) | xpos;

	switch (var->bits_per_pixel) {
	case 16:
		if (var->xres & 0x1) {
			var->xres = var->xres & ~((int)0x1);
			printk("xres should be a multiple of 2 pixels! setting it to %d\n", var->xres);
		}
		break;
	case 18:
	case 19:
		if (var->xres & 0x7) {
			var->xres = var->xres & ~((int)0x7);
			printk("xres should be a multiple of 8 pixels! setting it to %d\n", var->xres);
		}
		break;
	case 24:
	case 25:
		break;
	default:
		if (var->bits_per_pixel < 16) {
			var->bits_per_pixel = 16;
		} else if (var->bits_per_pixel > 25) {
			var->bits_per_pixel = 25;
		}
	}			/*switch */

	return 0;
}

/*
 *  overlay2fb_check_var():
 *    Get the video params out of 'var'. If a value doesn't fit, round it up,
 *    if it's too big, return -EINVAL.
 *
 */
static int overlay2fb_check_var(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info *)info;
	int format, xpos, ypos, xres = 0, yres = 0;
	int base_xres, base_yres;

	base_xres = fbi->basefb->fb.var.xres;
	base_yres = fbi->basefb->fb.var.yres;

	xpos = (var->nonstd & 0x3ff);
	ypos = (var->nonstd >> 10) & 0x3ff;
	format = (var->nonstd >> 20) & 0x7;

	if (var->xres > base_xres)
		var->xres = base_xres;
	if (var->yres > base_yres)
		var->yres = base_yres;

	/* Planar YCbCr444, YCbCr422, YCbCr420, RGB */
	if ((format != 0x4) && (format != 0x3) && (format != 0x2)
	    && (format != 0x0)) {
		format = 0x0;
	}

	/* dummy pixels */
	switch (format) {
	case 0x0:		/* RGB */
		xres = var->xres;
		break;
	case 0x2:		/* 444 */
		/* xres should be a multiple of 4 pixels! */
		xres = var->xres & ~(0x3);
		printk("xres should be a multiple of 4 pixels! "
		       "setting it to %d\n", xres);
		break;

	case 0x3:		/* 422 */
		/* xres should be a multiple of 8 pixels! */
		xres = var->xres & ~(0x7);
		printk("xres should be a multiple of 8 pixels! "
		       "setting it to %d\n", xres);
		break;
	case 0x4:		/* 420 */
		/* xres should be a multiple of 16 pixels! */
		xres = var->xres & ~(0xf);
		printk("xres should be a multiple of 16 pixels! "
		       "setting it to %d\n", xres);
		break;
	}
	yres = var->yres;

	if ((xpos + xres) > base_xres) {
		xpos = base_xres - xres;
		if (xpos < 0)
			xpos = 0;
	}
	if ((ypos + yres) > base_yres) {
		ypos = base_yres - yres;
		if (ypos < 0)
			ypos = 0;
	}

	/*set corrected values back */
	var->nonstd = (format << 20) | (ypos << 10) | xpos;
	var->xres = xres;
	var->yres = yres;

	return 0;
}

static int cursorfb_check_var(struct fb_var_screeninfo *var,
			      struct fb_info *info)
{
	struct cursor_mode *cursor;
	int mode, xpos, ypos;

	mode = var->nonstd & 0x7;
	xpos = (var->nonstd >> 5) & 0x3ff;
	ypos = (var->nonstd >> 15) & 0x3ff;

	/* check that mode number fit into "cursor_modes" array */
	if (mode < 0) {
		mode = 0;
		printk("%s:%d  %s incorrect cursor mode %d\n", __FILE__,
		       __LINE__, __FUNCTION__, mode);
	}
	if (mode > 7) {
		mode = 7;
		printk("%s:%d  %s incorrect cursor mode %d\n", __FILE__,
		       __LINE__, __FUNCTION__, mode);
	}

	cursor = &cursor_modes[mode];

	/*TODO: perform check of xpos/ypos based on base frame resolution 
	 * and cursor size for the mode, note cursor may be partiarly
	 * out of screen, accordingly the docs
	 * (do it at this point)
	 */

	/* update "var" info (resolution and bpp depend on the mode only) */
	var->xres = cursor->xres;
	var->yres = cursor->yres;
	var->bits_per_pixel = cursor->bpp;
	var->nonstd = (ypos << 15) | (xpos << 5) | mode;

	return 0;
}

#endif				/* CONFIG_PXA27x */

static inline void pxafb_set_truecolor(u_int is_true_color)
{
	DPRINTK("true_color = %d\n", is_true_color);
	// do your machine-specific setup if needed
}

/*
 * pxafb_set_par():
 *	Set the user defined part of the display for the specified console
 */
static int pxafb_set_par(struct fb_info *info)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	struct fb_var_screeninfo *var = &info->var;

	DPRINTK("set_par\n");

	return  pxafb_activate_var(var, fbi);
}

#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
static int overlay1fb_set_par(struct fb_info *info)
{
	return  overlay1fb_activate_var(&info->var, info);
}

static int overlay2fb_set_par(struct fb_info *info)
{
	return overlay2fb_activate_var(&info->var, info);
}

static int cursorfb_set_par(struct fb_info *info)
{
	return cursorfb_activate_var(&info->var, info);
}

#endif

/*
 * Formal definition of the VESA spec:
 *  On
 *  	This refers to the state of the display when it is in full operation
 *  Stand-By
 *  	This defines an optional operating state of minimal power reduction with
 *  	the shortest recovery time
 *  Suspend
 *  	This refers to a level of power management in which substantial power
 *  	reduction is achieved by the display.  The display can have a longer
 *  	recovery time from this state than from the Stand-by state
 *  Off
 *  	This indicates that the display is consuming the lowest level of power
 *  	and is non-operational. Recovery from this state may optionally require
 *  	the user to manually power on the monitor
 *
 *  Now, the fbdev driver adds an additional state, (blank), where they
 *  turn off the video (maybe by colormap tricks), but don't mess with the
 *  video itself: think of it semantically between on and Stand-By.
 *
 *  So here's what we should do in our fbdev blank routine:
 *
 *  	VESA_NO_BLANKING (mode 0)	Video on,  front/back light on
 *  	VESA_VSYNC_SUSPEND (mode 1)  	Video on,  front/back light off
 *  	VESA_HSYNC_SUSPEND (mode 2)  	Video on,  front/back light off
 *  	VESA_POWERDOWN (mode 3)		Video off, front/back light off
 *
 *  This will match the matrox implementation.
 */

/*
 * pxafb_blank():
 *	Blank the display by setting all palette values to zero.  Note, the
 * 	16 bpp mode does not really use the palette, so this will not
 *      blank the display in all modes.
 */
static int pxafb_blank(int blank, struct fb_info *info)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	int i;

	DPRINTK("pxafb_blank: blank=%d\n", blank);

	switch (blank) {
	case VESA_POWERDOWN:
	case VESA_VSYNC_SUSPEND:
	case VESA_HSYNC_SUSPEND:
		if (fbi->fb.fix.visual == FB_VISUAL_PSEUDOCOLOR ||
		    fbi->fb.fix.visual == FB_VISUAL_STATIC_PSEUDOCOLOR)
			for (i = 0; i < fbi->palette_size; i++)
				pxafb_setpalettereg(i, 0, 0, 0, 0, info);

		pxafb_schedule_work(fbi, C_DISABLE);
		//TODO if (pxafb_blank_helper) pxafb_blank_helper(blank);
		break;

	case VESA_NO_BLANKING:
		//TODO if (pxafb_blank_helper) pxafb_blank_helper(blank);
		if (fbi->fb.fix.visual == FB_VISUAL_PSEUDOCOLOR ||
		    fbi->fb.fix.visual == FB_VISUAL_STATIC_PSEUDOCOLOR)
			fb_set_cmap(&fbi->fb.cmap, info);
		pxafb_schedule_work(fbi, C_ENABLE);
	}
	return 0;
}

static int pxafb_ioctl(struct inode *inode, struct file *file, u_int cmd,
		       u_long arg, struct fb_info *info)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;

	switch (cmd) {
	case FBIO_WAITFORVSYNC:
		return pxafb_wait_for_eof(fbi, 0x1);
	default:
#ifdef CONFIG_FB_PXA_MINILCD
		return pxafb_minilcd_ioctl(inode, file, cmd, arg, info);
#else
		return -EINVAL;
#endif
	}

	return 0;
}

static int pxafb_pan_display(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	return pxafb_activate_var(var, fbi);
}

static struct fb_ops pxafb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= pxafb_check_var,
	.fb_set_par	= pxafb_set_par,
	.fb_setcolreg	= pxafb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_blank	= pxafb_blank,
	.fb_ioctl       = pxafb_ioctl,
	.fb_pan_display = pxafb_pan_display,
	.fb_cursor	= soft_cursor,
};


#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)

static struct fb_ops overlay1fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = overlay1fb_check_var,
	.fb_set_par = overlay1fb_set_par,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
#if 0
	.fb_blank       = overlay1fb_blank,
#endif
	.fb_cursor = soft_cursor,
	.fb_release = overlay1fb_release,
	.fb_open = overlay1fb_open,
};

static struct fb_ops overlay2fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = overlay2fb_check_var,
	.fb_set_par = overlay2fb_set_par,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
#if 0
	.fb_blank        = overlay2fb_blank,
#endif
	.fb_cursor = soft_cursor,
	.fb_release = overlay2fb_release,
	.fb_open = overlay2fb_open,
};

static struct fb_ops cursorfb_ops = {
	.owner THIS_MODULE,
	.fb_setcolreg = cursorfb_setcolreg,
	.fb_check_var = cursorfb_check_var,
	.fb_set_par = cursorfb_set_par,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
#if 0
	/*.fb_blank       = cursorfb_blank, */
#endif
	.fb_cursor = soft_cursor,
};


static int cursorfb_enable(struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info *)info;

	if (!fbi->map_cpu)
		return -EINVAL;

	CCR &= ~(1 << 31);

	/* set palette format 
	 *
	 * FIXME: if only cursor uses palette
	 */
	LCCR4 = (LCCR4 & (~(0x3 << 15))) | (0x1 << 15);

	/* disable branch/start/end of frame interrupt */
	LCCR5 |= (LCCR5_IUM5 | LCCR5_BSM5 | LCCR5_EOFM5 | LCCR5_SOFM5);

	/*load palette and frame data */
	if (fbi->state == C_DISABLE) {
		FDADR5 = fbi->dma5_pal->fdadr;
		udelay(1);
		FDADR5 = fbi->dma5_frame->fdadr;
		udelay(1);
	} else {
		FBR5 = fbi->dma5_pal->fdadr | 0x1;
		udelay(1);
		FBR5 = fbi->dma5_frame->fdadr | 0x1;
		udelay(1);
	}

	CCR = (1 << 31) | (fbi->ypos << 15) | (fbi->xpos << 5) | (fbi->format);

	fbi->state = C_ENABLE;

	return 0;
}

static void cursorfb_disable(struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info *)info;

	fbi->state = C_DISABLE;
	CCR &= ~(1 << 31);

}

/* used by fb_set_cmap*/
static int
cursorfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
		   u_int trans, struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info *)info;
	u_int val, ret = 1;
	u_int *pal = (u_int *) fbi->palette_cpu;

	/* 25bit with Transparcy for 16bpp format */
	if (regno < fbi->palette_size && regno >= 0) {
		val = ((trans << 24) & 0x1000000);
		val |= ((red << 16) & 0x0ff0000);
		val |= ((green << 8) & 0x000ff00);
		val |= ((blue << 0) & 0x00000ff);

		pal[regno] = val;
		ret = 0;
	} else {
		printk("%s:%d  %s\n   incorrect regno=%d\n", __FILE__, __LINE__,
		       __FUNCTION__, regno);
	}
	return ret;
}

/*
static void cursorfb_blank(int blank, struct fb_info *info)
{
	switch (blank) {
	case 0:
		cursorfb_enable(info);
		break;
	case 1:
		cursorfb_disable(info);
		break;
	default:	        
		break;
	}
}
*/
#endif /*CONFIG_PXA27x*/


/*
 * Calculate the PCD value from the clock rate (in picoseconds).
 * We take account of the PPCR clock setting.
 * From PXA Developer's Manual:
 *
 *   PixelClock =      LCLK
 *                -------------
 *                2 ( PCD + 1 )
 *
 *   PCD =      LCLK
 *         ------------- - 1
 *         2(PixelClock)
 *
 * Where:
 *   LCLK = LCD/Memory Clock
 *   PCD = LCCR3[7:0]
 *
 * PixelClock here is in Hz while the pixclock argument given is the
 * period in picoseconds. Hence PixelClock = 1 / ( pixclock * 10^-12 )
 *
 * The function get_lclk_frequency_10khz returns LCLK in units of
 * 10khz. Calling the result of this function lclk gives us the
 * following
 *
 *    PCD = (lclk * 10^4 ) * ( pixclock * 10^-12 )
 *          -------------------------------------- - 1
 *                          2
 *
 * Factoring the 10^4 and 10^-12 out gives 10^-8 == 1 / 100000000 as used below.
 */
static inline unsigned int get_pcd(unsigned int pixclock)
{
	unsigned long long pcd;

	/* FIXME: Need to take into account Double Pixel Clock mode
         * (DPC) bit? or perhaps set it based on the various clock
         * speeds */

	pcd = (unsigned long long)get_lcdclk_frequency_10khz() * pixclock;
	do_div(pcd, 100000000 * 2);
	/* no need for this, since we should subtract 1 anyway. they cancel */
	/* pcd += 1; */ /* make up for integer math truncations */
	return (unsigned int)pcd;
}

/*
 * pxafb_activate_var():
 *	Configures LCD Controller based on entries in var parameter.  Settings are
 *	only written to the controller if changes were made.
 */
static int pxafb_activate_var(struct fb_var_screeninfo *var, struct pxafb_info *fbi)
{
	struct pxafb_lcd_reg new_regs;
	unsigned long palette_mem_size;
	u_long flags, fsaddr_offset;
	u_int lines_per_panel, pcd;

	DPRINTK("Configuring PXA LCD\n");
	pxafb_check_var(var, (struct fb_info *)fbi);
	pxafb_init_unchangeable_vars(&fbi->fb.var, my_device);

	DPRINTK("var: xres=%d hslen=%d lm=%d rm=%d\n",
		var->xres, var->hsync_len,
		var->left_margin, var->right_margin);
	DPRINTK("var: yres=%d vslen=%d um=%d bm=%d\n",
		var->yres, var->vsync_len,
		var->upper_margin, var->lower_margin);
	pcd = get_pcd(var->pixclock);
	DPRINTK("var: pixclock=%d pcd=%d\n", var->pixclock, pcd);

	if (var->bits_per_pixel == 16)
		fbi->fb.fix.visual = FB_VISUAL_TRUECOLOR;
	else if (!fbi->cmap_static)
		fbi->fb.fix.visual = FB_VISUAL_PSEUDOCOLOR;
	else {
		/*
		 * Some people have weird ideas about wanting static
		 * pseudocolor maps.  I suspect their user space
		 * applications are broken.
		 */
		fbi->fb.fix.visual = FB_VISUAL_STATIC_PSEUDOCOLOR;
	}

	fbi->fb.fix.line_length = var->xres_virtual * var->bits_per_pixel / 8;

	if (var->bits_per_pixel == 16)
		fbi->palette_size = 0;
	else
		fbi->palette_size =
		    var->bits_per_pixel == 1 ? 4 : 1 << var->bits_per_pixel;

	palette_mem_size = fbi->palette_size * sizeof(u16);

	DPRINTK("palette_mem_size = 0x%08lx\n", (u_long) palette_mem_size);

	fbi->palette_cpu = (u16 *) (fbi->screen_cpu - palette_mem_size);
	fbi->palette_dma = fbi->screen_dma - palette_mem_size;

	/*
	 * Set (any) board control register to handle new color depth
	 */
	pxafb_set_truecolor(fbi->fb.fix.visual == FB_VISUAL_TRUECOLOR);

	DPRINTK("Configuring PXA LCD\n");

	DPRINTK("var: xres=%d hslen=%d lm=%d rm=%d\n",
		var->xres, var->hsync_len, var->left_margin, var->right_margin);
	DPRINTK("var: yres=%d vslen=%d um=%d bm=%d\n",
		var->yres, var->vsync_len,
		var->upper_margin, var->lower_margin);
	DPRINTK("var: pixclock=%d pcd=%d\n", var->pixclock, pcd);

#if DEBUG_VAR
	if (var->xres < MIN_XRES || var->xres > MAX_XRES)
		printk(KERN_ERR "%s: invalid xres %d\n",
		       fbi->fb.fix.id, var->xres);
	switch(var->bits_per_pixel) {
	case 1:
	case 2:
	case 4:
	case 8:
	case 16:
		break;
	default:
		printk(KERN_ERR "%s: invalid bit depth %d\n",
		       fbi->fb.fix.id, var->bits_per_pixel);
		break;
	}
	if (var->hsync_len < 1    || var->hsync_len > 64)
		printk(KERN_ERR "%s: invalid hsync_len %d\n",
			fbi->fb.fix.id, var->hsync_len);
	if (var->left_margin < 1  || var->left_margin > 255)
		printk(KERN_ERR "%s: invalid left_margin %d\n",
			fbi->fb.fix.id, var->left_margin);
	if (var->right_margin < 1 || var->right_margin > 255)
		printk(KERN_ERR "%s: invalid right_margin %d\n",
			fbi->fb.fix.id, var->right_margin);
	if (var->yres < MIN_YRES || var->yres > MAX_YRES)
		printk(KERN_ERR "%s: invalid yres %d\n",
			fbi->fb.fix.id, var->yres);
	if (var->vsync_len < 1    || var->vsync_len > 64)
		printk(KERN_ERR "%s: invalid vsync_len %d\n",
			fbi->fb.fix.id, var->vsync_len);
	if (var->upper_margin < 0 || var->upper_margin > 255)
		printk(KERN_ERR "%s: invalid upper_margin %d\n",
			fbi->fb.fix.id, var->upper_margin);
	if (var->lower_margin < 0 || var->lower_margin > 255)
		printk(KERN_ERR "%s: invalid lower_margin %d\n",
			fbi->fb.fix.id, var->lower_margin);
#endif

	new_regs.lccr0 = fbi->lccr0 |
		(LCCR0_LDM | LCCR0_SFM | LCCR0_IUM | LCCR0_EFM |
#if defined(CONFIG_PXA27x) || defined(CONFIG_PXA3xx) /* Enable overlay for PXA27x */
		 LCCR0_OUC | LCCR0_CMDIM | LCCR0_RDSTM |
#endif
                 LCCR0_QDM | LCCR0_BM );

	new_regs.lccr1 =
		LCCR1_DisWdth(var->xres_virtual) +
		LCCR1_HorSnchWdth(var->hsync_len) +
		LCCR1_BegLnDel(var->left_margin) +
		LCCR1_EndLnDel(var->right_margin);

	/*
	 * If we have a dual scan LCD, we need to halve
	 * the YRES parameter.
	 */
	lines_per_panel = var->yres;
	if ((fbi->lccr0 & LCCR0_SDS) == LCCR0_Dual)
		lines_per_panel /= 2;

	new_regs.lccr2 =
		LCCR2_DisHght(lines_per_panel) +
		LCCR2_VrtSnchWdth(var->vsync_len) +
		LCCR2_BegFrmDel(var->upper_margin) +
		LCCR2_EndFrmDel(var->lower_margin);

	new_regs.lccr3 = fbi->lccr3 |
		pxafb_bpp_to_lccr3(var) |
		(var->sync & FB_SYNC_HOR_HIGH_ACT ? LCCR3_HorSnchH : LCCR3_HorSnchL) |
		(var->sync & FB_SYNC_VERT_HIGH_ACT ? LCCR3_VrtSnchH : LCCR3_VrtSnchL);

	if (pcd)
		new_regs.lccr3 |= LCCR3_PixClkDiv(pcd);

	DPRINTK("nlccr0 = 0x%08x\n", new_regs.lccr0);
	DPRINTK("nlccr1 = 0x%08x\n", new_regs.lccr1);
	DPRINTK("nlccr2 = 0x%08x\n", new_regs.lccr2);
	DPRINTK("nlccr3 = 0x%08x\n", new_regs.lccr3);

	/* Update shadow copy atomically */
	local_irq_save(flags);

	/* setup dma descriptors */

	fbi->dmadesc_palette_cpu = (struct pxafb_dma_descriptor *)(fbi->map_cpu);
	fbi->dmadesc_fbhigh_cpu  = (struct pxafb_dma_descriptor *)(fbi->map_cpu) + 1;
	fbi->dmadesc_fblow_cpu   = (struct pxafb_dma_descriptor *)(fbi->map_cpu) + 2;

	fbi->dmadesc_palette_dma = fbi->map_dma;
	fbi->dmadesc_fbhigh_dma  = fbi->map_dma + 1 * sizeof(struct pxafb_dma_descriptor);
	fbi->dmadesc_fblow_dma   = fbi->map_dma + 2 * sizeof(struct pxafb_dma_descriptor);

#define BYTES_PER_PANEL (lines_per_panel * fbi->fb.fix.line_length)

	fsaddr_offset = var->yoffset * var->xres_virtual + var->xoffset;
	fsaddr_offset *= (var->bits_per_pixel / 8);

	/* populate descriptors */
	fbi->dmadesc_fblow_cpu->fdadr = fbi->dmadesc_fblow_dma;
	fbi->dmadesc_fblow_cpu->fsadr =
		fbi->screen_dma + BYTES_PER_PANEL + fsaddr_offset;
	fbi->dmadesc_fblow_cpu->fidr  = 0;
	fbi->dmadesc_fblow_cpu->ldcmd = BYTES_PER_PANEL | LDCMD_EOFINT;
	fbi->fdadr1 = fbi->dmadesc_fblow_dma; /* only used in dual-panel mode */

	fbi->dmadesc_fbhigh_cpu->fsadr = fbi->screen_dma + fsaddr_offset;
	fbi->dmadesc_fbhigh_cpu->fidr = 0;
	fbi->dmadesc_fbhigh_cpu->ldcmd = BYTES_PER_PANEL | LDCMD_EOFINT;

	fbi->dmadesc_palette_cpu->fsadr = fbi->palette_dma;
	fbi->dmadesc_palette_cpu->fidr  = 0;
	/* TODO: why "palette_size*2" but not "palette_size<<2" according
	 * to docs ???, compare to the same for cursor.
	 */
	fbi->dmadesc_palette_cpu->ldcmd = (fbi->palette_size * 2) | LDCMD_PAL;

	if (var->bits_per_pixel == 16) {
		/* palette shouldn't be loaded in true-color mode */
		fbi->dmadesc_fbhigh_cpu->fdadr = fbi->dmadesc_fbhigh_dma;
		fbi->fdadr0 = fbi->dmadesc_fbhigh_dma; /* no pal just fbhigh */
		/* init it to something, even though we won't be using it */
		fbi->dmadesc_palette_cpu->fdadr = fbi->dmadesc_palette_dma;
	} else {
		fbi->dmadesc_palette_cpu->fdadr = fbi->dmadesc_fbhigh_dma;
		fbi->dmadesc_fbhigh_cpu->fdadr = fbi->dmadesc_palette_dma;
		fbi->fdadr0 = fbi->dmadesc_palette_dma; /* flips back and forth between pal and fbhigh */
	}

#if 0
	DPRINTK("fbi->dmadesc_fblow_cpu = 0x%p\n", fbi->dmadesc_fblow_cpu);
	DPRINTK("fbi->dmadesc_fbhigh_cpu = 0x%p\n", fbi->dmadesc_fbhigh_cpu);
	DPRINTK("fbi->dmadesc_palette_cpu = 0x%p\n", fbi->dmadesc_palette_cpu);
	DPRINTK("fbi->dmadesc_fblow_dma = 0x%x\n", fbi->dmadesc_fblow_dma);
	DPRINTK("fbi->dmadesc_fbhigh_dma = 0x%x\n", fbi->dmadesc_fbhigh_dma);
	DPRINTK("fbi->dmadesc_palette_dma = 0x%x\n", fbi->dmadesc_palette_dma);

	DPRINTK("fbi->dmadesc_fblow_cpu->fdadr = 0x%x\n", fbi->dmadesc_fblow_cpu->fdadr);
	DPRINTK("fbi->dmadesc_fbhigh_cpu->fdadr = 0x%x\n", fbi->dmadesc_fbhigh_cpu->fdadr);
	DPRINTK("fbi->dmadesc_palette_cpu->fdadr = 0x%x\n", fbi->dmadesc_palette_cpu->fdadr);

	DPRINTK("fbi->dmadesc_fblow_cpu->fsadr = 0x%x\n", fbi->dmadesc_fblow_cpu->fsadr);
	DPRINTK("fbi->dmadesc_fbhigh_cpu->fsadr = 0x%x\n", fbi->dmadesc_fbhigh_cpu->fsadr);
	DPRINTK("fbi->dmadesc_palette_cpu->fsadr = 0x%x\n", fbi->dmadesc_palette_cpu->fsadr);

	DPRINTK("fbi->dmadesc_fblow_cpu->ldcmd = 0x%x\n", fbi->dmadesc_fblow_cpu->ldcmd);
	DPRINTK("fbi->dmadesc_fbhigh_cpu->ldcmd = 0x%x\n", fbi->dmadesc_fbhigh_cpu->ldcmd);
	DPRINTK("fbi->dmadesc_palette_cpu->ldcmd = 0x%x\n", fbi->dmadesc_palette_cpu->ldcmd);
#endif

	fbi->reg_lccr0 = new_regs.lccr0;
	fbi->reg_lccr1 = new_regs.lccr1;
	fbi->reg_lccr2 = new_regs.lccr2;
	fbi->reg_lccr3 = new_regs.lccr3;
	local_irq_restore(flags);

	/*
	 * Only update the registers if the controller is enabled
	 * and something has changed.
	 */
	if ((LCCR0  != fbi->reg_lccr0) || (LCCR1  != fbi->reg_lccr1) ||
	    (LCCR2  != fbi->reg_lccr2) || (LCCR3  != fbi->reg_lccr3) ||
	    (FDADR0 != fbi->fdadr0)    || (FDADR1 != fbi->fdadr1))
		pxafb_schedule_work(fbi, C_REENABLE);

	return 0;
}

/*
 * NOTE!  The following functions are purely helpers for set_ctrlr_state.
 * Do not call them directly; set_ctrlr_state does the correct serialisation
 * to ensure that things happen in the right way 100% of time time.
 *	-- rmk
 */
static inline void __pxafb_backlight_power(struct pxafb_info *fbi, int on)
{
	DPRINTK("backlight o%s\n", on ? "n" : "ff");

 	if (pxafb_backlight_power)
 		pxafb_backlight_power(on);
}

static inline void __pxafb_lcd_power(struct pxafb_info *fbi, int on)
{
	DPRINTK("LCD power o%s\n", on ? "n" : "ff");

	if (pxafb_lcd_power)
		pxafb_lcd_power(on);
}

static void pxafb_setup_gpio(struct pxafb_info *fbi)
{
#ifdef CONFIG_PXA3xx
	extern void zylonite_enable_lcd_pins(void);
	zylonite_enable_lcd_pins();
#else
	int gpio, ldd_bits;
        unsigned int lccr0 = fbi->lccr0;

	/*
	 * setup is based on type of panel supported
        */

	/* 4 bit interface */
	if ((lccr0 & LCCR0_CMS) == LCCR0_Mono &&
	    (lccr0 & LCCR0_SDS) == LCCR0_Sngl &&
	    (lccr0 & LCCR0_DPD) == LCCR0_4PixMono)
		ldd_bits = 4;

	/* 8 bit interface */
        else if (((lccr0 & LCCR0_CMS) == LCCR0_Mono &&
		  ((lccr0 & LCCR0_SDS) == LCCR0_Dual || (lccr0 & LCCR0_DPD) == LCCR0_8PixMono)) ||
                 ((lccr0 & LCCR0_CMS) == LCCR0_Color &&
		  (lccr0 & LCCR0_PAS) == LCCR0_Pas && (lccr0 & LCCR0_SDS) == LCCR0_Sngl))
		ldd_bits = 8;

	/* 16 bit interface */
	else if ((lccr0 & LCCR0_CMS) == LCCR0_Color &&
		 ((lccr0 & LCCR0_SDS) == LCCR0_Dual || (lccr0 & LCCR0_PAS) == LCCR0_Act))
		ldd_bits = 16;

	else {
	        printk(KERN_ERR "pxafb_setup_gpio: unable to determine bits per pixel\n");
		return;
        }

	for (gpio = 58; ldd_bits; gpio++, ldd_bits--)
		pxa_gpio_mode(gpio | GPIO_ALT_FN_2_OUT);
	pxa_gpio_mode(GPIO74_LCD_FCLK_MD);
	pxa_gpio_mode(GPIO75_LCD_LCLK_MD);
	pxa_gpio_mode(GPIO76_LCD_PCLK_MD);
	pxa_gpio_mode(GPIO77_LCD_ACBIAS_MD);
#endif
}

static void pxafb_enable_controller(struct pxafb_info *fbi)
{
	DPRINTK("Enabling LCD controller\n");
	DPRINTK("fdadr0 0x%08x\n", (unsigned int) fbi->fdadr0);
	DPRINTK("fdadr1 0x%08x\n", (unsigned int) fbi->fdadr1);
	DPRINTK("reg_lccr0 0x%08x\n", (unsigned int) fbi->reg_lccr0);
	DPRINTK("reg_lccr1 0x%08x\n", (unsigned int) fbi->reg_lccr1);
	DPRINTK("reg_lccr2 0x%08x\n", (unsigned int) fbi->reg_lccr2);
	DPRINTK("reg_lccr3 0x%08x\n", (unsigned int) fbi->reg_lccr3);

	/* disable LCD controller clock */
	pxa_set_cken(CKEN16_LCD, 0);

#ifdef CONFIG_PXA27x
	/* workaround for insight 41187 */
	OVL1C2 = 0;
	OVL1C1 = 0;
	OVL2C2 = 0;
	OVL2C1 = 0;
	CCR = 0;

	LCCR3 = fbi->reg_lccr3;
	LCCR2 = fbi->reg_lccr2;
	LCCR1 = fbi->reg_lccr1;
	LCCR0 = fbi->reg_lccr0 & ~LCCR0_ENB;
	LCCR0 |= LCCR0_ENB;
	LCCR0 |= LCCR0_DIS;
	LCCR3 = fbi->reg_lccr3;
	LCCR2 = fbi->reg_lccr2;
	LCCR1 = fbi->reg_lccr1;
	LCCR0 = fbi->reg_lccr0 & ~LCCR0_ENB;
	LCCR0 |= LCCR0_ENB;
	FDADR0 = fbi->fdadr0;
	FDADR1 = fbi->fdadr1;
	LCCR0 |= LCCR0_ENB;
#else
	/* Sequence from 11.7.10 */
	LCCR3 = fbi->reg_lccr3;
	LCCR2 = fbi->reg_lccr2;
	LCCR1 = fbi->reg_lccr1;
	LCCR0 = fbi->reg_lccr0 & ~LCCR0_ENB;

	FDADR0 = fbi->fdadr0;
	FDADR1 = fbi->fdadr1;
#endif

	LCCR4 |= (1 << 31);
#ifdef CONFIG_PXA27x
	LCCR4 |= (5 << 17);
#endif

#ifdef CONFIG_PXA3xx
	LCCR4 = LCCR4 | LCCR4_REOFM0 | LCCR4_REOFM1 | LCCR4_REOFM2 |
		LCCR4_REOFM3 | LCCR4_REOFM4 | LCCR4_REOFM5 |
		LCCR4_REOFM6;
#endif
	LCCR0 |= LCCR0_ENB;
	/* enable LCD controller clock */
	pxa_set_cken(CKEN16_LCD, 1);

	DPRINTK("FDADR0 0x%08x\n", (unsigned int) FDADR0);
	DPRINTK("FDADR1 0x%08x\n", (unsigned int) FDADR1);
	DPRINTK("LCCR0 0x%08x\n", (unsigned int) LCCR0);
	DPRINTK("LCCR1 0x%08x\n", (unsigned int) LCCR1);
	DPRINTK("LCCR2 0x%08x\n", (unsigned int) LCCR2);
	DPRINTK("LCCR3 0x%08x\n", (unsigned int) LCCR3);
}

static void pxafb_disable_controller(struct pxafb_info *fbi)
{
	DECLARE_WAITQUEUE(wait, current);

	DPRINTK("Disabling LCD controller\n");

	add_wait_queue(&fbi->ctrlr_wait, &wait);
	set_current_state(TASK_UNINTERRUPTIBLE);

	LCSR = 0xffffffff;	/* Clear LCD Status Register */
	LCCR0 &= ~LCCR0_LDM;	/* Enable LCD Disable Done Interrupt */
	LCCR0 |= LCCR0_DIS;	/* Disable LCD Controller */

	schedule_timeout(20 * HZ / 1000);
	remove_wait_queue(&fbi->ctrlr_wait, &wait);
}

/*
 *  pxafb_handle_irq: Handle 'LCD DONE' interrupts.
 */
static irqreturn_t pxafb_handle_irq(int irq, void *dev_id,
				    struct pt_regs *regs)
{
	struct pxafb_info *fbi = dev_id;
	unsigned int lcsr0 = LCSR0;
#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
	unsigned int lcsr1 = LCSR1;
	unsigned long channels;
#endif
	unsigned long flags;

	local_irq_save(flags);
	LCSR0 = lcsr0;
#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
	LCSR1 = lcsr1;
#endif
	if (lcsr0 & LCSR_LDD) {
		LCCR0 |= LCCR0_LDM;
		local_irq_restore(flags);
		wake_up(&fbi->ctrlr_wait);
	} else local_irq_restore(flags);

	if (lcsr0 & LCSR_EOF) {
		local_irq_save(flags);
		LCCR0 |= LCCR0_EFM; /* mask */
		CLEAR_LCD_INTR(LCSR0, LCSR_EOF); /* clear */
		local_irq_restore(flags);
		fbi->eof_mask |= (1<<0);
	}

#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
	channels = (lcsr1 & (0x3f << 8)) >> 8;
	if (channels) {
		/* mask and clear */
		local_irq_save(flags);
		LCCR5 |= (channels << 8);
		CLEAR_LCD_INTR(LCSR1, (channels << 8));
		local_irq_restore(flags);
		fbi->eof_mask |= (channels << 1);
	}
#endif

	wake_up(&fbi->eof_wait);

	return IRQ_HANDLED;
}

/*
 * This function must be called from task context only, since it will
 * sleep when disabling the LCD controller, or if we get two contending
 * processes trying to alter state.
 */
static void set_ctrlr_state(struct pxafb_info *fbi, u_int state)
{
	u_int old_state;

	down(&fbi->ctrlr_sem);

	old_state = fbi->state;

	/*
	 * Hack around fbcon initialisation.
	 */
	if (old_state == C_STARTUP && state == C_REENABLE)
		state = C_ENABLE;

	switch (state) {
	case C_DISABLE_CLKCHANGE:
		/*
		 * Disable controller for clock change.  If the
		 * controller is already disabled, then do nothing.
		 */
		if (old_state != C_DISABLE && old_state != C_DISABLE_PM) {
			fbi->state = state;
			//TODO __pxafb_lcd_power(fbi, 0);
			pxafb_disable_controller(fbi);
#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
			disable_overlays(fbi);
#endif
		}
		break;

	case C_DISABLE_PM:
	case C_DISABLE:
		/*
		 * Disable controller
		 */
		if (old_state != C_DISABLE) {
			fbi->state = state;
			__pxafb_backlight_power(fbi, 0);
			__pxafb_lcd_power(fbi, 0);
			if (old_state != C_DISABLE_CLKCHANGE)
				pxafb_disable_controller(fbi);
#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
			disable_overlays(fbi);
#endif
		}
		break;

	case C_ENABLE_CLKCHANGE:
		/*
		 * Enable the controller after clock change.  Only
		 * do this if we were disabled for the clock change.
		 */
		if (old_state == C_DISABLE_CLKCHANGE) {
			fbi->state = C_ENABLE;
			pxafb_enable_controller(fbi);
			//TODO __pxafb_lcd_power(fbi, 1);
#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
			enable_overlays(fbi);
#endif
		}
		break;

	case C_REENABLE:
		/*
		 * Re-enable the controller only if it was already
		 * enabled.  This is so we reprogram the control
		 * registers.
		 */
		if (old_state == C_ENABLE) {
#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
			disable_overlays(fbi);
#endif
			pxafb_disable_controller(fbi);
			pxafb_setup_gpio(fbi);
			pxafb_enable_controller(fbi);
#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
			enable_overlays(fbi);
#endif
		}
		break;

#ifdef CONFIG_PM
	case C_ENABLE_PM:
		/*
		 * Re-enable the controller after PM.  This is not
		 * perfect - think about the case where we were doing
		 * a clock change, and we suspended half-way through.
		 */
		if (old_state != C_DISABLE_PM)
			break;
		/* fall through */
#endif

	case C_ENABLE:
		/*
		 * Power up the LCD screen, enable controller, and
		 * turn on the backlight.
		 */
		if (old_state != C_ENABLE) {
			fbi->state = C_ENABLE;
			pxafb_setup_gpio(fbi);
			pxafb_enable_controller(fbi);
			__pxafb_lcd_power(fbi, 1);
			__pxafb_backlight_power(fbi, 1);
		}
#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
		enable_overlays(fbi);
#endif
		break;
	}
	up(&fbi->ctrlr_sem);
}

/*
 * Our LCD controller task (which is called when we blank or unblank)
 * via keventd.
 */
static void pxafb_task(void *dummy)
{
	struct pxafb_info *fbi = dummy;
	u_int state = xchg(&fbi->task_state, -1);

	set_ctrlr_state(fbi, state);
}

#ifdef CONFIG_CPU_FREQ
/*
 * CPU clock speed change handler.  We need to adjust the LCD timing
 * parameters when the CPU clock is adjusted by the power management
 * subsystem.
 *
 * TODO: Determine why f->new != 10*get_lclk_frequency_10khz()
 */
static int
pxafb_freq_transition(struct notifier_block *nb, unsigned long val, void *data)
{
	struct pxafb_info *fbi = TO_INF(nb, freq_transition);
	//TODO struct cpufreq_freqs *f = data;
	u_int pcd;

	switch (val) {
	case CPUFREQ_PRECHANGE:
		set_ctrlr_state(fbi, C_DISABLE_CLKCHANGE);
		break;

	case CPUFREQ_POSTCHANGE:
		pcd = get_pcd(fbi->fb.var.pixclock);
		fbi->reg_lccr3 = (fbi->reg_lccr3 & ~0xff) | LCCR3_PixClkDiv(pcd);
		set_ctrlr_state(fbi, C_ENABLE_CLKCHANGE);
		break;
	}
	return 0;
}

static int
pxafb_freq_policy(struct notifier_block *nb, unsigned long val, void *data)
{
	struct pxafb_info *fbi = TO_INF(nb, freq_policy);
	struct fb_var_screeninfo *var = &fbi->fb.var;
	struct cpufreq_policy *policy = data;

	switch (val) {
	case CPUFREQ_ADJUST:
	case CPUFREQ_INCOMPATIBLE:
		printk(KERN_DEBUG "min dma period: %d ps, "
			"new clock %d kHz\n", pxafb_display_dma_period(var),
			policy->max);
		// TODO: fill in min/max values
		break;
#if 0
	case CPUFREQ_NOTIFY:
		printk(KERN_ERR "%s: got CPUFREQ_NOTIFY\n", __FUNCTION__);
		do {} while(0);
		/* todo: panic if min/max values aren't fulfilled
		 * [can't really happen unless there's a bug in the
		 * CPU policy verification process *
		 */
		break;
#endif
	}
	return 0;
}
#endif

#ifdef CONFIG_PM
/*
 * Power management hooks.  Note that we won't be called from IRQ context,
 * unlike the blank functions above, so we may sleep.
 */
static int pxafb_suspend(struct device *dev, u32 state, u32 level)
{
	struct pxafb_info *fbi = pxafbi;

	if (level == SUSPEND_DISABLE || level == SUSPEND_POWER_DOWN)
		set_ctrlr_state(fbi, C_DISABLE_PM);
	return 0;
}

static int pxafb_resume(struct device *dev, u32 level)
{
	struct pxafb_info *fbi = pxafbi;

	if (level == RESUME_ENABLE)
		set_ctrlr_state(fbi, C_ENABLE_PM);
	return 0;
}
#else
#define pxafb_suspend	NULL
#define pxafb_resume	NULL
#endif

/*
 * pxafb_map_video_memory():
 *      Allocates the DRAM memory for the frame buffer.  This buffer is
 *	remapped into a non-cached, non-buffered, memory region to
 *      allow palette and pixel writes to occur without flushing the
 *      cache.  Once this area is remapped, all virtual memory
 *      access to the video memory should occur at the new region.
 */
static int __init pxafb_map_video_memory(struct pxafb_info *fbi)
{
	u_long palette_mem_size;

	/*
	 * We reserve one page for the palette and DMA descriptors,
	 * plus the size of the framebuffer.
	 */
	fbi->map_size = PAGE_ALIGN(fbi->fb.fix.smem_len + PAGE_SIZE);
	fbi->map_cpu = dma_alloc_coherent(fbi->dev, fbi->map_size,
					  (dma_addr_t *) & fbi->map_dma,
					  GFP_KERNEL);


	if (fbi->map_cpu) {
		/* prevent initial garbage on screen */
		memset(fbi->map_cpu, 0, fbi->map_size);
		fbi->screen_cpu = fbi->fb.screen_base =
			fbi->map_cpu + PAGE_SIZE;
		fbi->screen_dma = fbi->map_dma + PAGE_SIZE;
		/*
		 * FIXME: this is actually the wrong thing to place in
		 * smem_start.  But fbdev suffers from the problem that
		 * it needs an API which doesn't exist (in this case,
		 * dma_writecombine_mmap)
		 */
		fbi->fb.fix.smem_start = fbi->screen_dma;

		fbi->palette_size = fbi->fb.var.bits_per_pixel == 8 ? 256 : 16;

		palette_mem_size = fbi->palette_size * sizeof(u16);
		DPRINTK("palette_mem_size = 0x%08lx\n", (u_long) palette_mem_size);

		fbi->palette_cpu = (u16 *)(fbi->screen_cpu - palette_mem_size);
		fbi->palette_dma = fbi->screen_dma - palette_mem_size;
	}
	return fbi->map_cpu ? 0 : -ENOMEM;
}

#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)

/*
 * Overlay 1: 16 bpp, 24 bpp (no palette)
 */
static int overlay1fb_map_video_memory(struct overlayfb_info *fbi)
{

	if (fbi->map_cpu) {
		dma_free_coherent(fbi->dev, fbi->map_size, fbi->map_cpu,
				  fbi->map_dma);
		fbi->map_cpu = NULL;
	}
	fbi->map_size = PAGE_ALIGN(fbi->fb.fix.smem_len + PAGE_SIZE);
	fbi->map_cpu = dma_alloc_coherent(fbi->dev, fbi->map_size,
					  (dma_addr_t *) & fbi->map_dma,
					  GFP_KERNEL);

	if (!fbi->map_cpu)
		return -ENOMEM;

	memset(fbi->map_cpu, 0, fbi->map_size);

	fbi->screen_cpu = fbi->map_cpu + PAGE_SIZE;
	fbi->screen_dma = fbi->map_dma + PAGE_SIZE;
	fbi->fb.screen_base = fbi->map_cpu + PAGE_SIZE;
	fbi->fb.fix.smem_start = fbi->screen_dma;

	/* setup dma descriptor */
	fbi->dma1 = (struct pxafb_dma_descriptor *)
	    (fbi->screen_cpu - sizeof(struct pxafb_dma_descriptor));

	return 0;

}

static int overlay2fb_map_YUV_memory(struct overlayfb_info *fbi)
{
	unsigned int ylen, cblen, crlen, aylen, acblen, acrlen;
	unsigned int yoff, cboff, croff;
	unsigned int xres, yres;
	unsigned int nbytes;

	ylen = cblen = crlen = aylen = acblen = acrlen = 0;
	yoff = cboff = croff = 0;

	if (fbi->map_cpu) {
		dma_free_coherent(fbi->dev, fbi->map_size, fbi->map_cpu,
				  fbi->map_dma);
		fbi->map_cpu = NULL;
	}

	yres = fbi->fb.var.yres;

	switch (fbi->format) {
	case 0x4:		/* YCbCr 4:2:0 planar */
		DPRINTK("420 planar\n");
		/* 16 pixels per line */
		xres = (fbi->fb.var.xres + 0xf) & (~0xf);
		fbi->fb.fix.line_length = xres;

		nbytes = xres * yres;
		ylen = nbytes;
		cblen = crlen = (nbytes / 4);

		break;
	case 0x3:		/* YCbCr 4:2:2 planar */
		/* 8 pixles per line */
		DPRINTK("422 planar\n");
		xres = (fbi->fb.var.xres + 0x7) & (~0x7);
		fbi->fb.fix.line_length = xres;

		nbytes = xres * yres;
		ylen = nbytes;
		cblen = crlen = (nbytes / 2);

		break;
	case 0x2:		/* YCbCr 4:4:4 planar */
		/* 4 pixels per line */
		DPRINTK("444 planar\n");
		xres = (fbi->fb.var.xres + 0x3) & (~0x3);
		fbi->fb.fix.line_length = xres;

		nbytes = xres * yres;
		ylen = cblen = crlen = nbytes;
		break;
	}
	/* 16-bytes alignment for DMA */
	aylen = (ylen + 0xf) & (~0xf);
	acblen = (cblen + 0xf) & (~0xf);
	acrlen = (crlen + 0xf) & (~0xf);

	fbi->fb.fix.smem_len = aylen + acblen + acrlen;

	/* alloc memory */

	fbi->map_size = PAGE_ALIGN(fbi->fb.fix.smem_len + PAGE_SIZE);
	fbi->map_cpu = dma_alloc_coherent(fbi->dev,
					  fbi->map_size,
					  (dma_addr_t *) & fbi->map_dma,
					  GFP_KERNEL);


	if (!fbi->map_cpu)
		return -ENOMEM;

	fbi->screen_cpu = fbi->map_cpu + PAGE_SIZE;
	fbi->screen_dma = fbi->map_dma + PAGE_SIZE;

	fbi->fb.fix.smem_start = fbi->screen_dma;

	/* setup dma for Planar format */
	fbi->dma2 = (struct pxafb_dma_descriptor *)
	    (fbi->screen_cpu - sizeof(struct pxafb_dma_descriptor));
	fbi->dma3 = fbi->dma2 - 1;
	fbi->dma4 = fbi->dma3 - 1;

	/* offset */
	yoff = 0;
	cboff = aylen;
	croff = cboff + acblen;

	/* Y vector */
	fbi->dma2->fdadr =
	    (fbi->screen_dma - sizeof(struct pxafb_dma_descriptor));
	fbi->dma2->fsadr = fbi->screen_dma + yoff;
	fbi->dma2->fidr = 0;
	fbi->dma2->ldcmd = ylen;

	/* Cb vector */
	fbi->dma3->fdadr =
	    (fbi->dma2->fdadr - sizeof(struct pxafb_dma_descriptor));
	fbi->dma3->fsadr = (fbi->screen_dma + cboff);
	fbi->dma3->fidr = 0;
	fbi->dma3->ldcmd = cblen;

	/* Cr vector */

	fbi->dma4->fdadr =
	    (fbi->dma3->fdadr - sizeof(struct pxafb_dma_descriptor));
	fbi->dma4->fsadr = (fbi->screen_dma + croff);
	fbi->dma4->fidr = 0;
	fbi->dma4->ldcmd = crlen;

	/* adjust for user */
	fbi->fb.var.red.length = ylen;
	fbi->fb.var.red.offset = yoff;
	fbi->fb.var.green.length = cblen;
	fbi->fb.var.green.offset = cboff;
	fbi->fb.var.blue.length = crlen;
	fbi->fb.var.blue.offset = croff;

	return 0;
};

static int overlay2fb_map_RGB_memory(struct overlayfb_info *fbi)
{
	struct fb_var_screeninfo *var = &fbi->fb.var;
	int pixels_per_line = 0, nbytes = 0;

	if (fbi->map_cpu) {
		dma_free_coherent(fbi->dev, fbi->map_size, fbi->map_cpu,
				  fbi->map_dma);
		fbi->map_cpu = NULL;
	}

	switch (var->bits_per_pixel) {
	case 16:
		/* 2 pixels per line */
		pixels_per_line = (fbi->fb.var.xres + 0x1) & (~0x1);
		nbytes = 2;

		var->red = def_rgbt_16.red;
		var->green = def_rgbt_16.green;
		var->blue = def_rgbt_16.blue;
		var->transp = def_rgbt_16.transp;
		break;

	case 18:
		/* 8 pixels per line */
		pixels_per_line = (fbi->fb.var.xres + 0x7) & (~0x7);
		nbytes = 3;

		var->red = def_rgb_18.red;
		var->green = def_rgb_18.green;
		var->blue = def_rgb_18.blue;
		var->transp = def_rgb_18.transp;
		break;

	case 19:
		/* 8 pixels per line */
		pixels_per_line = (fbi->fb.var.xres + 0x7) & (~0x7);
		nbytes = 3;

		var->red = def_rgbt_19.red;
		var->green = def_rgbt_19.green;
		var->blue = def_rgbt_19.blue;
		var->transp = def_rgbt_19.transp;
		break;

	case 24:
		pixels_per_line = fbi->fb.var.xres;
		nbytes = 4;

		var->red = def_rgbt_24.red;
		var->green = def_rgbt_24.green;
		var->blue = def_rgbt_24.blue;
		var->transp = def_rgbt_24.transp;
		break;

	case 25:
		pixels_per_line = fbi->fb.var.xres;
		nbytes = 4;

		var->red = def_rgbt_25.red;
		var->green = def_rgbt_25.green;
		var->blue = def_rgbt_25.blue;
		var->transp = def_rgbt_25.transp;
		break;
	}

	fbi->fb.fix.line_length = nbytes * pixels_per_line;
	fbi->fb.fix.smem_len = fbi->fb.fix.line_length * fbi->fb.var.yres;

	DPRINTK("FB2, RGB: map %d bytes (%dx%d), nbytes %d\n",
		fbi->fb.fix.smem_len,
		fbi->fb.fix.line_length, fbi->fb.var.yres, nbytes);

	fbi->map_size = PAGE_ALIGN(fbi->fb.fix.smem_len + PAGE_SIZE);
	fbi->map_cpu = dma_alloc_coherent(fbi->dev,
					  fbi->map_size,
					  (dma_addr_t *) & fbi->map_dma,
					  GFP_KERNEL);


	if (!fbi->map_cpu)
		return -ENOMEM;
	memset(fbi->map_cpu, 0, fbi->map_size);

	fbi->screen_cpu = fbi->map_cpu + PAGE_SIZE;
	fbi->screen_dma = fbi->map_dma + PAGE_SIZE;

	fbi->fb.fix.smem_start = fbi->screen_dma;

	/* setup dma descriptor */
	fbi->dma2 = (struct pxafb_dma_descriptor *)
	    (fbi->screen_cpu - sizeof(struct pxafb_dma_descriptor));

	fbi->dma2->fdadr =
	    (fbi->screen_dma - sizeof(struct pxafb_dma_descriptor));
	fbi->dma2->fsadr = fbi->screen_dma;
	fbi->dma2->fidr = 0;
	fbi->dma2->ldcmd = fbi->fb.fix.smem_len;

	return 0;
}

static int overlay1_activated_once;

static int
overlay1fb_activate_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info *)info;

	struct fb_var_screeninfo *dvar = &fbi->fb.var;

	int nbytes = 0, err = 0, pixels_per_line = 0;
	int xpos = 0, ypos = 0;
	int changed = 0;

	if (!fbi->map_cpu)
		return -EINVAL;

	xpos = (var->nonstd & 0x3ff);
	ypos = ((var->nonstd >> 10) & 0x3ff);

	if ((fbi->xpos != xpos) || (fbi->ypos != ypos)) {
		fbi->xpos = xpos;
		fbi->ypos = ypos;
		changed = 1;
	}

	changed = 1;

	switch (var->bits_per_pixel) {
	case 16:
		/* 2 pixels per line */
		pixels_per_line = (fbi->fb.var.xres + 0x1) & (~0x1);
		nbytes = 2;

		dvar->red = def_rgbt_16.red;
		dvar->green = def_rgbt_16.green;
		dvar->blue = def_rgbt_16.blue;
		dvar->transp = def_rgbt_16.transp;
		dvar->bits_per_pixel = 16;
		break;
#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)

	case 18:
	case 19:
		pixels_per_line = (fbi->fb.var.xres + 0x3) & (~0x3);
		nbytes = 3;

		dvar->red = def_rgbt_18.red;
		dvar->green = def_rgbt_18.green;
		dvar->blue = def_rgbt_18.blue;
		dvar->transp = def_rgbt_18.transp;
		dvar->bits_per_pixel = 18;
		break;
#endif
	case 24:
#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
	case 25:
#endif
		pixels_per_line = fbi->fb.var.xres;
		nbytes = 4;

		dvar->red = def_rgbt_24.red;
		dvar->green = def_rgbt_24.green;
		dvar->blue = def_rgbt_24.blue;
		dvar->transp = def_rgbt_24.transp;
		dvar->bits_per_pixel = 24;
		break;
	}			/*switch */

	/* update fix_screeninfo fields */

	fbi->fb.fix.line_length = nbytes * pixels_per_line;
	fbi->fb.fix.smem_len = fbi->fb.fix.line_length * fbi->fb.var.yres;

	err = overlay1fb_map_video_memory(fbi);

	if (err)
		return err;

	/*do the real work with hardware now overlay1fb_enable(...) */
	overlay1fb_enable(fbi);

	overlay1_activated_once = 1;

	return 0;
}

static int overlay2_activated_once;

static int
overlay2fb_activate_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info *)info;
	struct fb_var_screeninfo *dvar = &fbi->fb.var;
	int format, err;

	overlay2fb_disable(info);

	format = (var->nonstd >> 20) & 0x7;

	if (fbi->state == C_DISABLE) {
		goto out1;
	}

	/* resolution */
	if ((var->xres == dvar->xres) &&
	    (var->yres == dvar->yres) &&
	    (var->bits_per_pixel == dvar->bits_per_pixel) &&
	    (format == fbi->format)) {
		goto out2;
	}

      out1:
#ifdef CONFIG_PXA27x
	/* FIXME : sighting #49219, #56573
	 * 
	 * Enable RGB mode before entering YUV420
	 */
	if ((format == 0x4) &&
	    ((fbi->state == C_DISABLE) || (fbi->format != format)))
		overlay2fb_YUV420_workaround(info);
#endif

	/* update var_screeninfo fields */
	dvar->nonstd = var->nonstd;
	dvar->xres = var->xres;
	dvar->yres = var->yres;
	dvar->bits_per_pixel = var->bits_per_pixel;
	dvar->nonstd = var->nonstd;

	fbi->format = format;
	if (fbi->format == 0)
		err = overlay2fb_map_RGB_memory(fbi);
	else
		err = overlay2fb_map_YUV_memory(fbi);

	if (err)
		return err;

      out2:
	/* position */
	fbi->xpos = var->nonstd & 0x3ff;
	fbi->ypos = (var->nonstd >> 10) & 0x3ff;

	overlay2fb_enable((struct overlay_fb_info *)info);

	overlay2_activated_once = 1;

	return 0;
}

static int  overlay1fb_disable(struct overlayfb_info *fbi)
{
	int done;

	fbi->state = C_DISABLE;

	/* clear O1EN */
	OVL1C1 &= ~OVL1C1_O1EN;

	CLEAR_LCD_INTR(LCSR1, LCSR1_BS1);
	FBR1 = FDADR1 | 0x03;
	done = WAIT_FOR_LCD_INTR(LCSR1, LCSR1_BS1, 100);

	if (!done) {
		DPRINTK(KERN_INFO "%s: timeout\n", __FUNCTION__);
		return -1;
	}
	return 0;
}

static int overlay1fb_open(struct fb_info *info, int user)
{

	if(overlay1_activated_once != 0){
		overlay1fb_blank(0, info);
	}

	return 0;
}

static int overlay1fb_release(struct fb_info *info, int user)
{

	/* disable overlay when released */
	if(overlay1_activated_once != 0){
		overlay1fb_blank(1, info);
	}

	return 0;
}

static int overlay2fb_open(struct fb_info *info, int user)
{

       if(overlay2_activated_once != 0){
	 	overlay2fb_blank(0, info);
       }

	return 0;
}

static int overlay2fb_release(struct fb_info *info, int user)
{

	/* disable overlay when released */
       if(overlay2_activated_once != 0){
		overlay2fb_blank(1, info);
       }

	return 0;
}


static void overlay1fb_blank(int blank, struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info *)info;
	int res=0;

	switch (blank) {
	case 0:
		res=overlay1fb_enable(fbi);
		if(res!=0){
		  fbi->state = C_DISABLE;
		  set_ctrlr_state(fbi->basefb, C_REENABLE);
		}
		break;
	case 1:
		res=overlay1fb_disable(fbi);
		if(res!=0){
		  fbi->state = C_DISABLE;
		  set_ctrlr_state(fbi->basefb, C_REENABLE);
		}
		break;
	default:
		break;
	}
}

static int overlay1fb_enable(struct overlayfb_info *fbi)
{

	unsigned long bpp1;
	int xpos = 0, ypos = 0;

	xpos = (fbi->fb.var.nonstd & 0x3ff);
	ypos = ((fbi->fb.var.nonstd >> 10) & 0x3ff);

	/*do the real work with hardware now */
	switch (fbi->fb.var.bits_per_pixel) {
	case 16:
		bpp1 = 0x4;
		break;
	case 18:
		bpp1 = 0x6;
		break;
	case 19:
		bpp1 = 0x8;
		break;
	case 24:
		bpp1 = 0x0;
		break;
	case 25:
		bpp1 = 0xa;
		break;
	default:
		return -EINVAL;
	}

	/* disable branch/start/end of frame interrupt */
	LCCR5 |= (LCCR5_IUM1 | LCCR5_BSM1 | LCCR5_EOFM1 | LCCR5_SOFM1);

	/* disable overlay 1 window */
	OVL1C1 &= ~(1 << 31);

	OVL1C2 = (fbi->ypos << 10) | fbi->xpos;
	OVL1C1 =
	    (bpp1 << 20) | ((fbi->fb.var.yres - 1) << 10) | (fbi->fb.var.xres -
							     1);

	/*prepare DMA-descriptor */
	fbi->dma1 = (struct pxafb_dma_descriptor *)
	    (fbi->screen_cpu - sizeof(struct pxafb_dma_descriptor));
	fbi->dma1->fdadr =
	    (fbi->screen_dma - sizeof(struct pxafb_dma_descriptor));
	fbi->dma1->fsadr = fbi->screen_dma;
	fbi->dma1->fidr = 0;
	fbi->dma1->ldcmd = fbi->fb.fix.smem_len;

	/*give the DMA-descriptor prepared to our controller Overlay1
	 *  (we must handle it differently, depending on waither DMA is
	 *  running now or not and possibly some more correct method exists,
	 *  which uses examining of some DMA-status register)
	 */
	if (fbi->state == C_DISABLE) {
		FDADR1 = (fbi->dma1->fdadr);
	} else {
		FBR1 = fbi->dma1->fdadr | 0x1;
	}

	/* enable overlay 1 window */

	OVL1C1 |= OVL1C1_O1EN;

	fbi->state = C_ENABLE;

	return 0;
}

static int
cursorfb_activate_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info *)info;
	struct cursor_mode *cursor;
	int mode, xpos, ypos;
	int changed, err;

	mode = var->nonstd & 0x7;
	xpos = (var->nonstd >> 5) & 0x3ff;
	ypos = (var->nonstd >> 15) & 0x3ff;

	changed = 0;

	cursor = cursor_modes + mode;

	/* update "var" info */
	fbi->fb.var.xres = cursor->xres;
	fbi->fb.var.yres = cursor->yres;
	fbi->fb.var.bits_per_pixel = cursor->bpp;

	/*
	 * alloc video memory 
	 * 
	 *  4k is engouh for 128x128x1 cursor, 
	 *  - 2k for cursor pixels, 
	 *  - 2k for palette data, plus 2 dma descriptor
	 */
	if (!fbi->map_cpu) {
		fbi->map_size = PAGE_SIZE;
		fbi->map_cpu = dma_alloc_coherent(fbi->dev, fbi->map_size,
						  (dma_addr_t *) & fbi->map_dma,
						  GFP_KERNEL);

		if (!fbi->map_cpu) {
			return -ENOMEM;
		}
		memset(fbi->map_cpu, 0, fbi->map_size);
	}

	cursor = cursor_modes + mode;

	/* update cursor overlay & fix "info" */
	fbi->screen_cpu = fbi->map_cpu;
	fbi->palette_cpu = fbi->map_cpu + (PAGE_SIZE / 2);
	fbi->screen_dma = fbi->map_dma;
	fbi->palette_dma = fbi->map_dma + (PAGE_SIZE / 2);

	fbi->format = mode;
	fbi->palette_size = (1 << cursor->bpp);
	fbi->fb.fix.smem_start = fbi->screen_dma;
	fbi->fb.fix.smem_len = cursor->xres * cursor->yres * cursor->bpp / 8;
	fbi->fb.fix.line_length = cursor->xres * cursor->bpp / 8;

	fbi->dma5_pal =
	    (struct pxafb_dma_descriptor *)(fbi->map_cpu + PAGE_SIZE - 16);
	fbi->dma5_pal->fdadr = (fbi->map_dma + PAGE_SIZE - 16);
	fbi->dma5_pal->fsadr = fbi->palette_dma;
	fbi->dma5_pal->fidr = 0;

	fbi->dma5_pal->ldcmd = (fbi->palette_size << 2) | LDCMD_PAL;

	fbi->dma5_frame =
	    (struct pxafb_dma_descriptor *)(fbi->map_cpu + PAGE_SIZE - 32);
	fbi->dma5_frame->fdadr = (fbi->map_dma + PAGE_SIZE - 32);
	fbi->dma5_frame->fsadr = fbi->screen_dma;
	fbi->dma5_frame->fidr = 0;
	fbi->dma5_frame->ldcmd = fbi->fb.fix.smem_len;

	/* alloc & set default cmap */
	err = fb_alloc_cmap(&fbi->fb.cmap, fbi->palette_size, 0);
	if (err)
		return err;
	err = fb_set_cmap(&fbi->fb.cmap, info);	/* cursorfb_setcolreg should be set in ..._ops, as fb_set_cmap uses it from there */
	if (err)
		return err;

	changed = 1;

	/* update overlay info */
	if ((xpos != fbi->xpos) || (ypos != fbi->ypos)) {
		fbi->xpos = xpos;
		fbi->ypos = ypos;
		changed = 1;
	}

	cursorfb_enable(info);

	set_ctrlr_state(fbi->basefb, C_REENABLE);

	return 0;
}

#endif				/*CONFIG_PXA27x */

static void pxafb_init_unchangeable_vars(struct fb_var_screeninfo *var,
				      struct device *dev)
{

	struct pxafb_mach_info *inf = dev->platform_data;

	var->xres = inf->xres;
	var->yres = inf->yres;
	var->bits_per_pixel = inf->bpp;
	var->pixclock = inf->pixclock;
	var->hsync_len = inf->hsync_len;
	var->left_margin = inf->left_margin;
	var->right_margin = inf->right_margin;
	var->vsync_len = inf->vsync_len;
	var->upper_margin = inf->upper_margin;
	var->lower_margin = inf->lower_margin;
	var->sync = inf->sync;
	var->grayscale = inf->cmap_greyscale;
}

static struct pxafb_info *__init pxafb_init_fbinfo(struct device *dev)
{
	struct pxafb_info *fbi;
	void *addr;
	struct pxafb_mach_info *inf = dev->platform_data;

	/* Alloc the pxafb_info and pseudo_palette in one step */
	fbi = kmalloc(sizeof(struct pxafb_info) + sizeof(u32) * 16, GFP_KERNEL);
	if (!fbi)
		return NULL;

	memset(fbi, 0, sizeof(struct pxafb_info));
	fbi->dev = dev;

	strcpy(fbi->fb.fix.id, PXA_NAME);

	fbi->fb.fix.type	= FB_TYPE_PACKED_PIXELS;
	fbi->fb.fix.type_aux	= 0;
	fbi->fb.fix.xpanstep	= 1;
	fbi->fb.fix.ypanstep	= 1;
	fbi->fb.fix.ywrapstep	= 0;
#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
	fbi->fb.fix.accel	= directfb ? FB_ACCEL_PXA27X : FB_ACCEL_NONE;
#else
	fbi->fb.fix.accel	= FB_ACCEL_NONE;
#endif
	fbi->fb.fix.mmio_start  = 0x44000000;
	fbi->fb.fix.mmio_len    = 0x270;

	fbi->fb.var.nonstd	= 0;
	fbi->fb.var.activate	= FB_ACTIVATE_NOW;
	fbi->fb.var.height	= -1;
	fbi->fb.var.width	= -1;
	fbi->fb.var.accel_flags	= 0;
	fbi->fb.var.vmode	= FB_VMODE_NONINTERLACED;
	fbi->fb.var.xres_virtual= inf->xres;
	fbi->fb.var.yres_virtual= inf->yres;

	fbi->fb.fbops		= &pxafb_ops;
	fbi->fb.flags		= FBINFO_DEFAULT;
	fbi->fb.node		= -1;

	addr = fbi;
	addr = addr + sizeof(struct pxafb_info);
	fbi->fb.pseudo_palette	= addr;

	pxafb_init_unchangeable_vars(&fbi->fb.var, my_device);

	fb_alloc_cmap(&fbi->fb.cmap, 256, 0);

	fbi->cmap_inverse		= inf->cmap_inverse;
	fbi->cmap_static		= inf->cmap_static;
	fbi->lccr0			= inf->lccr0;
	fbi->lccr3			= inf->lccr3;
	fbi->state			= C_STARTUP;
	fbi->task_state			= (u_char)-1;
	fbi->fb.fix.smem_len = PAGE_ALIGN(MAX_XRES * MAX_YRES * inf->bpp / 8);

#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
	if (directfb) {
		/*
		 * include space for the two overlay's frame data, where
		 * each overlay's max resolution is the base layer
		 * screen size.
		 */
		fbi->fb.fix.smem_len += 2 * PAGE_ALIGN(inf->xres * inf->yres *
						       MAX_BPP / 8);
		/*
		 * and add a couple pages that DirectFB can use for
		 * the overlay DMA descriptors.
		 */
		fbi->fb.fix.smem_len += 2*PAGE_SIZE;
	}
#endif

	init_waitqueue_head(&fbi->ctrlr_wait);
	init_waitqueue_head(&fbi->eof_wait);
	INIT_WORK(&fbi->task, pxafb_task, fbi);
	init_MUTEX(&fbi->ctrlr_sem);
	spin_lock_init(&fbi->rt_lock);

	return fbi;
}


#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
static struct overlayfb_info *__init overlay1fb_init_fbinfo(struct device *dev)
{
	struct overlayfb_info *fbi;

	fbi =
	    kmalloc(sizeof(struct overlayfb_info) + sizeof(u16) * 16,
		    GFP_KERNEL);
	if (!fbi) {
		return NULL;
	}

	memset(fbi, 0, sizeof(struct overlayfb_info));
	fbi->dev = dev;

	strcpy(fbi->fb.fix.id, "overlay1");

	fbi->fb.fix.type = FB_TYPE_PACKED_PIXELS;
	fbi->fb.fix.type_aux = 0;
	fbi->fb.fix.xpanstep = 0;
	fbi->fb.fix.ypanstep = 0;
	fbi->fb.fix.ywrapstep = 0;
	fbi->fb.fix.accel = FB_ACCEL_NONE;

	fbi->fb.var.nonstd = 0;
	fbi->fb.var.activate = FB_ACTIVATE_NOW;
	fbi->fb.var.height = -1;
	fbi->fb.var.width = -1;
	fbi->fb.var.accel_flags = 0;
	fbi->fb.var.vmode = FB_VMODE_NONINTERLACED;

	fbi->fb.fbops = &overlay1fb_ops;

	fbi->fb.flags = FBINFO_FLAG_DEFAULT;

	fbi->xpos = 0;
	fbi->ypos = 0;
	fbi->format = -1;
	fbi->enabled = 0;
	fbi->state = C_DISABLE;

	return fbi;
}

static struct overlayfb_info *__init overlay2fb_init_fbinfo(struct device *dev)
{
	struct overlayfb_info *fbi;

	fbi =
	    kmalloc(sizeof(struct overlayfb_info) + sizeof(u16) * 16,
		    GFP_KERNEL);
	if (!fbi) {
		return NULL;
	}

	memset(fbi, 0, sizeof(struct overlayfb_info));
	fbi->dev = dev;

	strcpy(fbi->fb.fix.id, "overlay2");

	fbi->fb.fix.type = FB_TYPE_PACKED_PIXELS;
	fbi->fb.fix.type_aux = 0;
	fbi->fb.fix.xpanstep = 0;
	fbi->fb.fix.ypanstep = 0;
	fbi->fb.fix.ywrapstep = 0;
	fbi->fb.fix.accel = FB_ACCEL_NONE;

	fbi->fb.var.nonstd = 0;
	fbi->fb.var.activate = FB_ACTIVATE_NOW;
	fbi->fb.var.height = -1;
	fbi->fb.var.width = -1;
	fbi->fb.var.accel_flags = 0;
	fbi->fb.var.vmode = FB_VMODE_NONINTERLACED;

	fbi->fb.fbops = &overlay2fb_ops;

	fbi->fb.flags = FBINFO_FLAG_DEFAULT;

	fbi->xpos = 0;
	fbi->ypos = 0;
	fbi->format = -1;
	fbi->enabled = 0;
	fbi->state = C_DISABLE;

	return fbi;
}

static struct overlayfb_info *__init cursorfb_init_fbinfo(struct device *dev)
{
	struct overlayfb_info *fbi;

	fbi =
	    kmalloc(sizeof(struct overlayfb_info) + sizeof(u16) * 16,
		    GFP_KERNEL);
	if (!fbi)
		return NULL;

	memset(fbi, 0, sizeof(struct overlayfb_info));

	strcpy(fbi->fb.fix.id, "cursor");
	fbi->dev = dev;

	fbi->fb.fix.type = FB_TYPE_PACKED_PIXELS;
	fbi->fb.fix.type_aux = 0;
	fbi->fb.fix.xpanstep = 0;
	fbi->fb.fix.ypanstep = 0;
	fbi->fb.fix.ywrapstep = 0;
	fbi->fb.fix.accel = FB_ACCEL_NONE;

	fbi->fb.var.nonstd = 0;
	fbi->fb.var.activate = FB_ACTIVATE_NOW;
	fbi->fb.var.height = -1;
	fbi->fb.var.width = -1;
	fbi->fb.var.accel_flags = 0;
	fbi->fb.var.vmode = FB_VMODE_NONINTERLACED;

	fbi->fb.fbops = &cursorfb_ops;

	fbi->fb.flags = FBINFO_FLAG_DEFAULT;

	fbi->xpos = 0;
	fbi->ypos = 0;
	fbi->format = -1;
	fbi->enabled = 0;
	fbi->state = C_DISABLE;

	return fbi;
}

static int overlay2fb_enable(struct overlayfb_info *fbi)
{
	unsigned long bpp2;
	unsigned int xres, yres;

	if (!fbi->map_cpu) {
		return -EINVAL;
	}

	switch (fbi->fb.var.bits_per_pixel) {
	case 16:
		bpp2 = 0x4;
		break;
	case 18:
		bpp2 = 0x6;
		break;
	case 19:
		bpp2 = 0x8;
		break;
	case 24:
		bpp2 = 0x9;
		break;
	case 25:
		bpp2 = 0xa;
		break;
	default:
		return -EINVAL;
	}

	DPRINTK("FB2, enable: %dx%d, bpp %x, format %d, xpos %d, ypos %d\n",
		xres, yres, bpp2, fbi->format, fbi->xpos, fbi->ypos);

	/*disable branch/start/end of frame interrupt */
	LCCR5 |= (LCCR5_IUM4 | LCCR5_IUM3 | LCCR5_IUM2 |
		  LCCR5_BSM4 | LCCR5_BSM3 | LCCR5_BSM2 |
		  LCCR5_EOFM4 | LCCR5_EOFM3 | LCCR5_EOFM2 |
		  LCCR5_SOFM4 | LCCR5_SOFM3 | LCCR5_SOFM2);

	if (fbi->format == 0) {
		/* overlay2 RGB resolution, RGB and YUV have different xres value*/
		xres = fbi->fb.var.xres;
		yres = fbi->fb.var.yres;

		OVL2C2 = (fbi->format << 20) | (fbi->ypos << 10) | fbi->xpos;
		OVL2C1 = (bpp2 << 20) | ((yres-1)<<10) | (xres-1);

		DPRINTK("Setup RGB DMA...\n");
		/* setup RGB DMA */
		if (fbi->state == C_DISABLE)
			FDADR2 = fbi->dma2->fdadr;
		else
			FBR2 = fbi->dma2->fdadr | 0x1;
	} else {
		/* overlay2 YUV resolution */
		xres = fbi->fb.fix.line_length;
		yres = fbi->fb.var.yres;

		OVL2C2 = (fbi->format << 20) | (fbi->ypos << 10) | fbi->xpos;
		OVL2C1 = (bpp2 << 20) | ((yres-1)<<10) | (xres-1);
#ifdef CONFIG_PXA27x
		/* FIXME PXA27x E25 */
		if (fbi->format == 4) {
			overlay2fb_enable_RGB((struct fb_info *)fbi);
			overlay2fb_disable_RGB((struct fb_info *)fbi);
			overlay2fb_enable_YUV420(fbi);
			yuv420_enabled = 1;
		}
#endif
		if (fbi->state == C_DISABLE){
			FDADR2 = fbi->dma2->fdadr;
			FDADR3 = fbi->dma3->fdadr;
			FDADR4 = fbi->dma4->fdadr;
		} else {
			FBR2 = fbi->dma2->fdadr | 0x01;
			FBR3 = fbi->dma3->fdadr | 0x01;
			FBR4 = fbi->dma4->fdadr | 0x01;
		}

	}

	OVL2C1 |= OVL2C1_O2EN;

	fbi->state = C_ENABLE;
	return 0;
}

static int overlay2fb_disable(struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info *)info;
	int done;

	if (fbi->state == C_DISABLE) {
		DPRINTK("FB2 disabled already\n");
		return 0;
	}
	fbi->state = C_DISABLE;

	/* clear O2EN - disable overlay 2 */
	OVL2C1 &= ~OVL2C1_O2EN;

	LCSR1 = LCSR1_BS2;

	FBR2 = 0x3;
	if (fbi->format) {
		FBR3 = 0x3;
		FBR4 = 0x3;
	}

#ifdef CONFIG_PXA27x
	if (yuv420_enabled && fbi->format == 0x4)
		overlay2fb_disable_YUV420(info);
#endif

	done = WAIT_FOR_LCD_INTR(LCSR1, LCSR1_BS2, 100);

	if (!done) {
		DPRINTK(KERN_INFO "%s: timeout\n", __FUNCTION__);
		return -1;
	}
	return 0;
}

static void overlay2fb_blank(int blank, struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info *)info;
	int res=0;

	switch (blank) {
	case 0:
	        res=overlay2fb_enable(fbi);
		if(res != 0){
		  fbi->state = C_DISABLE;
		  set_ctrlr_state(fbi->basefb, C_REENABLE);
		}
		break;
	case 1:
		res=overlay2fb_disable((struct fb_info *)fbi);
		if(res != 0){
		  fbi->state = C_DISABLE;
		  set_ctrlr_state(fbi->basefb, C_REENABLE);
		}
		break;
	default:		
		break;
	}
}

#ifdef CONFIG_PXA27x
/* set xpos, ypos, PPL and LP to 0 */
static int overlay2fb_YUV420_workaround(struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info *)info;
	struct pxafb_dma_descriptor *dma;
	u32 map_dma;
	void *map_cpu;
	int done, ret = 0;

	map_cpu = dma_alloc_coherent(fbi->dev, PAGE_SIZE,
				     (dma_addr_t *) & map_dma,
				     GFP_KERNEL);


	if (!map_cpu)
		return -1;

	dma =
	    (struct pxafb_dma_descriptor *)((map_cpu + PAGE_SIZE) -
					    sizeof(struct
						   pxafb_dma_descriptor));
	dma->fdadr = map_dma + PAGE_SIZE - sizeof(struct pxafb_dma_descriptor);
	dma->fsadr = map_dma;
	dma->fidr = 0;
	dma->ldcmd = LDCMD_EOFINT | 128;

	/* step 2.a - enable overlay 2 with RGB mode
	 *
	 * - (xpos,ypos) = (0,0); 
	 * - 64 pixels, 16bpp 
	 */
	LCCR5 |= (LCCR5_IUM4 | LCCR5_IUM3 | LCCR5_IUM2 |
		  LCCR5_BSM4 | LCCR5_BSM3 | LCCR5_BSM2 |
		  LCCR5_EOFM4 | LCCR5_EOFM3 | LCCR5_EOFM2 |
		  LCCR5_SOFM4 | LCCR5_SOFM3 | LCCR5_SOFM2);
	OVL2C2 = 0;
	OVL2C1 = OVL2C1_O2EN | (63);

	CLEAR_LCD_INTR(LCSR1, LCSR1_EOF2);
	if (fbi->state == C_DISABLE)
		FDADR2 = dma->fdadr;
	else
		FBR2 = dma->fdadr | 0x1;

	/* step 2.b - run at least 1 frame with overlay 2 */
	done = WAIT_FOR_LCD_INTR(LCSR1, LCSR1_EOF2, 100);
	if (!done) {
		goto err;
	}

	/* step 2.c - disable overlay 2 */
	OVL2C1 &= ~OVL2C1_O2EN;

	CLEAR_LCD_INTR(LCSR1, LCSR1_BS2);
	FBR2 = 0x3;

	done = WAIT_FOR_LCD_INTR(LCSR1, LCSR1_BS2, 100);
	if (!done) {
		goto err;
	}

	/* step 2.d - Wait for base EOF interrupts */
	CLEAR_LCD_INTR(LCSR0, LCSR_EOF);
	done = WAIT_FOR_LCD_INTR(LCSR0, LCSR_EOF, 100);

	goto out;
      err:
	ret = -1;
      out:
	/* free buffer allocated */
	if (map_cpu != NULL) {
		dma_free_coherent(fbi->dev, fbi->map_size, map_cpu, map_dma);
		map_cpu = NULL;
	}

	fbi->state = C_DISABLE;

	return ret;
}

static int overlay2fb_enable_RGB(struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info *)info;
	struct pxafb_dma_descriptor *dma =
	    (struct pxafb_dma_descriptor *)fbi->screen_cpu;

	/* xpos, ypos = 0, 0 */
	OVL2C2 = 0;

	/* 64 pixels, 16bpp */
	OVL2C1 = OVL2C1_O2EN | (63);
	dma->fdadr = fbi->screen_dma;
	dma->fsadr = PHYS_OFFSET;	/*DRAM offset */
	dma->fidr = 0;
	dma->ldcmd = LDCMD_EOFINT | 128;

	FDADR2 = fbi->screen_dma;

	LCSR1 = LCSR1_EOF2;

	if (!WAIT_FOR_LCD_INTR(LCSR1, LCSR1_EOF2, 100)) {
		DPRINTK(KERN_INFO "%s: timeout\n", __FUNCTION__);
		return -1;
	}
	return 0;
}

static int overlay2fb_disable_RGB(struct fb_info *info)
{
	OVL2C1 &= ~OVL2C1_O2EN;

	LCSR1 = LCSR1_BS2;
	FBR2 = 0x3;
	if (!WAIT_FOR_LCD_INTR(LCSR1, LCSR1_BS2, 100)) {
		DPRINTK(KERN_WARNING "%s: timeout\n", __FUNCTION__);
		return -1;
	}
	return 0;
}


static int overlay2fb_enable_YUV420(struct overlayfb_info *fbi)
{
	unsigned int xres, yres;

	xres = fbi->fb.fix.line_length;
	yres = fbi->fb.var.yres;

	/* disable overlay 2 window */
	OVL2C1 &= ~OVL2C1_O2EN;

	/* enable overlay 2 window */
	if (fbi->format) {
		OVL2C2 = (fbi->format << 20) | (fbi->ypos << 10) | fbi->xpos;
		OVL2C1 = ((yres - 1) << 10) | (xres - 1);
	} else {
		DPRINTK(KERN_INFO "%s: fbi->format == 0 ???\n", __FUNCTION__);
	}

	OVL2C1 |= OVL2C1_O2EN;

	LCSR1 = LCSR1_IU2;
	if (!WAIT_FOR_LCD_INTR(LCSR1, LCSR1_IU2, 100))
		DPRINTK(KERN_WARNING "%s: timeout\n", __FUNCTION__);

	if (fbi->state == C_DISABLE) {
		FDADR2 = fbi->dma2->fdadr;
		FDADR3 = fbi->dma3->fdadr;
		FDADR4 = fbi->dma4->fdadr;
	} else {
		FBR2 = fbi->dma2->fdadr | 0x1;
		FBR3 = fbi->dma3->fdadr | 0x1;
		FBR4 = fbi->dma4->fdadr | 0x1;
	}

	return 0;
}

static int overlay2fb_disable_YUV420(struct fb_info *info)
{
	/* Workaround for sightings ? */

	LCSR1 = LCSR1_BS2;
	FBR2 = 0x3;
	FBR3 = 0x3;
	FBR4 = 0x3;
	if (!WAIT_FOR_LCD_INTR(LCSR1, LCSR1_BS2, 100)) {
		DPRINTK(KERN_WARNING "%s: timeout.\n", __FUNCTION__);
		return -1;
	}

	return 0;
}
#endif
#endif

#ifdef CONFIG_FB_PXA_PARAMETERS
static int __init pxafb_parse_options(struct device *dev, char *options)
{
	struct pxafb_mach_info *inf = dev->platform_data;
	char *this_opt;

        if (!options || !*options)
                return 0;

	dev_dbg(dev, "options are \"%s\"\n", options ? options : "null");

	/* could be made table driven or similar?... */
        while ((this_opt = strsep(&options, ",")) != NULL) {
                if (!strncmp(this_opt, "mode:", 5)) {
			const char *name = this_opt+5;
			unsigned int namelen = strlen(name);
			int res_specified = 0, bpp_specified = 0;
			unsigned int xres = 0, yres = 0, bpp = 0;
			int yres_specified = 0;
			int i;
			for (i = namelen-1; i >= 0; i--) {
				switch (name[i]) {
				case '-':
					namelen = i;
					if (!bpp_specified && !yres_specified) {
						bpp = simple_strtoul(&name[i+1], NULL, 0);
						bpp_specified = 1;
					} else
						goto done;
					break;
				case 'x':
					if (!yres_specified) {
						yres = simple_strtoul(&name[i+1], NULL, 0);
						yres_specified = 1;
					} else
						goto done;
					break;
				case '0'...'9':
					break;
				default:
					goto done;
				}
			}
			if (i < 0 && yres_specified) {
				xres = simple_strtoul(name, NULL, 0);
				res_specified = 1;
			}
		done:
			if (res_specified) {
				dev_info(dev, "overriding resolution: %dx%d\n", xres, yres);
				inf->xres = xres; inf->yres = yres;
			}
			if (bpp_specified)
				switch (bpp) {
				case 1:
				case 2:
				case 4:
				case 8:
				case 16:
					inf->bpp = bpp;
					dev_info(dev, "overriding bit depth: %d\n", bpp);
					break;
				default:
					dev_err(dev, "Depth %d is not valid\n", bpp);
				}
                } else if (!strncmp(this_opt, "pixclock:", 9)) {
                        inf->pixclock = simple_strtoul(this_opt+9, NULL, 0);
			dev_info(dev, "override pixclock: %ld\n", inf->pixclock);
                } else if (!strncmp(this_opt, "left:", 5)) {
                        inf->left_margin = simple_strtoul(this_opt+5, NULL, 0);
			dev_info(dev, "override left: %u\n", inf->left_margin);
                } else if (!strncmp(this_opt, "right:", 6)) {
                        inf->right_margin = simple_strtoul(this_opt+6, NULL, 0);
			dev_info(dev, "override right: %u\n", inf->right_margin);
                } else if (!strncmp(this_opt, "upper:", 6)) {
                        inf->upper_margin = simple_strtoul(this_opt+6, NULL, 0);
			dev_info(dev, "override upper: %u\n", inf->upper_margin);
                } else if (!strncmp(this_opt, "lower:", 6)) {
                        inf->lower_margin = simple_strtoul(this_opt+6, NULL, 0);
			dev_info(dev, "override lower: %u\n", inf->lower_margin);
                } else if (!strncmp(this_opt, "hsynclen:", 9)) {
                        inf->hsync_len = simple_strtoul(this_opt+9, NULL, 0);
			dev_info(dev, "override hsynclen: %u\n", inf->hsync_len);
                } else if (!strncmp(this_opt, "vsynclen:", 9)) {
                        inf->vsync_len = simple_strtoul(this_opt+9, NULL, 0);
			dev_info(dev, "override vsynclen: %u\n", inf->vsync_len);
                } else if (!strncmp(this_opt, "hsync:", 6)) {
                        if (simple_strtoul(this_opt+6, NULL, 0) == 0) {
				dev_info(dev, "override hsync: Active Low\n");
				inf->sync &= ~FB_SYNC_HOR_HIGH_ACT;
			} else {
				dev_info(dev, "override hsync: Active High\n");
				inf->sync |= FB_SYNC_HOR_HIGH_ACT;
			}
                } else if (!strncmp(this_opt, "vsync:", 6)) {
                        if (simple_strtoul(this_opt+6, NULL, 0) == 0) {
				dev_info(dev, "override vsync: Active Low\n");
				inf->sync &= ~FB_SYNC_VERT_HIGH_ACT;
			} else {
				dev_info(dev, "override vsync: Active High\n");
				inf->sync |= FB_SYNC_VERT_HIGH_ACT;
			}
                } else if (!strncmp(this_opt, "dpc:", 4)) {
                        if (simple_strtoul(this_opt+4, NULL, 0) == 0) {
				dev_info(dev, "override double pixel clock: false\n");
				inf->lccr3 &= ~LCCR3_DPC;
			} else {
				dev_info(dev, "override double pixel clock: true\n");
				inf->lccr3 |= LCCR3_DPC;
			}
                } else if (!strncmp(this_opt, "outputen:", 9)) {
                        if (simple_strtoul(this_opt+9, NULL, 0) == 0) {
				dev_info(dev, "override output enable: active low\n");
				inf->lccr3 = (inf->lccr3 & ~LCCR3_OEP) | LCCR3_OutEnL;
			} else {
				dev_info(dev, "override output enable: active high\n");
				inf->lccr3 = (inf->lccr3 & ~LCCR3_OEP) | LCCR3_OutEnH;
			}
                } else if (!strncmp(this_opt, "pixclockpol:", 12)) {
                        if (simple_strtoul(this_opt+12, NULL, 0) == 0) {
				dev_info(dev, "override pixel clock polarity: falling edge\n");
				inf->lccr3 = (inf->lccr3 & ~LCCR3_PCP) | LCCR3_PixFlEdg;
			} else {
				dev_info(dev, "override pixel clock polarity: rising edge\n");
				inf->lccr3 = (inf->lccr3 & ~LCCR3_PCP) | LCCR3_PixRsEdg;
			}
                } else if (!strncmp(this_opt, "color", 5)) {
			inf->lccr0 = (inf->lccr0 & ~LCCR0_CMS) | LCCR0_Color;
                } else if (!strncmp(this_opt, "mono", 4)) {
			inf->lccr0 = (inf->lccr0 & ~LCCR0_CMS) | LCCR0_Mono;
                } else if (!strncmp(this_opt, "active", 6)) {
			inf->lccr0 = (inf->lccr0 & ~LCCR0_PAS) | LCCR0_Act;
                } else if (!strncmp(this_opt, "passive", 7)) {
			inf->lccr0 = (inf->lccr0 & ~LCCR0_PAS) | LCCR0_Pas;
                } else if (!strncmp(this_opt, "single", 6)) {
			inf->lccr0 = (inf->lccr0 & ~LCCR0_SDS) | LCCR0_Sngl;
                } else if (!strncmp(this_opt, "dual", 4)) {
			inf->lccr0 = (inf->lccr0 & ~LCCR0_SDS) | LCCR0_Dual;
                } else if (!strncmp(this_opt, "4pix", 4)) {
			inf->lccr0 = (inf->lccr0 & ~LCCR0_DPD) | LCCR0_4PixMono;
                } else if (!strncmp(this_opt, "8pix", 4)) {
			inf->lccr0 = (inf->lccr0 & ~LCCR0_DPD) | LCCR0_8PixMono;
		} else {
			dev_err(dev, "unknown option: %s\n", this_opt);
			return -EINVAL;
		}
        }
        return 0;

}
#endif

int __init pxafb_probe(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pxafb_mach_info *inf;
	int ret;

#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
	struct overlayfb_info *overlay1fb, *overlay2fb, *cursorfb;

	overlay1fb = overlay2fb = cursorfb = NULL;
#endif				/* CONFIG_PXA27x */

	my_device = dev;

	dev_dbg(dev, "pxafb_probe\n");

	inf = dev->platform_data;
	ret = -ENOMEM;
	pxafbi = NULL;
	if (!inf)
		goto failed;

#ifdef CONFIG_FB_PXA_PARAMETERS
	ret = pxafb_parse_options(dev, g_options);
	if (ret < 0)
		goto failed;
#endif

#ifdef DEBUG_VAR
        /* Check for various illegal bit-combinations. Currently only
	 * a warning is given. */

        if (inf->lccr0 & LCCR0_INVALID_CONFIG_MASK)
                dev_warn(dev, "machine LCCR0 setting contains illegal bits: %08x\n",
                        inf->lccr0 & LCCR0_INVALID_CONFIG_MASK);
        if (inf->lccr3 & LCCR3_INVALID_CONFIG_MASK)
                dev_warn(dev, "machine LCCR3 setting contains illegal bits: %08x\n",
                        inf->lccr3 & LCCR3_INVALID_CONFIG_MASK);
        if (inf->lccr0 & LCCR0_DPD &&
	    ((inf->lccr0 & LCCR0_PAS) != LCCR0_Pas ||
	     (inf->lccr0 & LCCR0_SDS) != LCCR0_Sngl ||
	     (inf->lccr0 & LCCR0_CMS) != LCCR0_Mono))
                dev_warn(dev, "Double Pixel Data (DPD) mode is only valid in passive mono"
			 " single panel mode\n");
        if ((inf->lccr0 & LCCR0_PAS) == LCCR0_Act &&
	    (inf->lccr0 & LCCR0_SDS) == LCCR0_Dual)
                dev_warn(dev, "Dual panel only valid in passive mode\n");
        if ((inf->lccr0 & LCCR0_PAS) == LCCR0_Pas &&
             (inf->upper_margin || inf->lower_margin))
                dev_warn(dev, "Upper and lower margins must be 0 in passive mode\n");
#endif

	dev_dbg(dev, "got a %dx%dx%d LCD\n",inf->xres, inf->yres, inf->bpp);
	if (inf->xres == 0 || inf->yres == 0 || inf->bpp == 0) {
		dev_err(dev, "Invalid resolution or bit depth\n");
		ret = -EINVAL;
		goto failed;
	}
	pxafb_backlight_power = inf->pxafb_backlight_power;
	pxafb_lcd_power = inf->pxafb_lcd_power;
	pxafbi = pxafb_init_fbinfo(dev);
	if (!pxafbi) {
		dev_err(dev, "Failed to initialize framebuffer device\n");
		ret = -ENOMEM; // only reason for pxafb_init_fbinfo to fail is kmalloc
		goto failed;
	}

	/* Initialize video memory */
	ret = pxafb_map_video_memory(pxafbi);
	if (ret) {
		dev_err(dev, "Failed to allocate video RAM: %d\n", ret);
		ret = -ENOMEM;
		goto failed;
	}
	/* enable LCD controller clock */
#ifdef CONFIG_PXA3xx
        pxa_set_cken(CKEN_LCD, 1);
#else
	pxa_set_cken(CKEN16_LCD, 1);
#endif

	ret = request_irq(IRQ_LCD, pxafb_handle_irq, 0, "LCD", pxafbi);
	if (ret) {
		dev_err(dev, "request_irq failed: %d\n", ret);
		ret = -EBUSY;
		goto failed;
	}

	/*
	 * This makes sure that our colour bitfield
	 * descriptors are correctly initialised.
	 */
	pxafb_check_var(&pxafbi->fb.var, &pxafbi->fb);
	pxafb_set_par(&pxafbi->fb);

	dev_set_drvdata(dev, pxafbi);

	ret = register_framebuffer(&pxafbi->fb);
	if (ret < 0) {
		dev_err(dev, "Failed to register framebuffer device: %d\n", ret);
		goto failed;
	}
#ifdef CONFIG_PM
	// TODO
#endif

#ifdef CONFIG_CPU_FREQ
	pxafbi->freq_transition.notifier_call = pxafb_freq_transition;
	pxafbi->freq_policy.notifier_call = pxafb_freq_policy;
	cpufreq_register_notifier(&pxafbi->freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_register_notifier(&pxafbi->freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif

#if defined(CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
	if (!directfb) {
		/* Overlay 1 window */
		overlay1fb = overlay1fb_init_fbinfo(dev);
		if (!overlay1fb) {
			ret = -ENOMEM;
			printk(KERN_ERR "PXA FB: overlay1fb_init_fbinfo failed\n");
			goto failed;
		}

		/* Overlay 2 window */
		overlay2fb = overlay2fb_init_fbinfo(dev);
		if (!overlay2fb) {
			ret = -ENOMEM;
			printk(KERN_ERR "PXA FB: overlay2fb_init_fbinfo failed\n");
			goto failed;
		}
	}

	/* Hardware cursor window */
	cursorfb = cursorfb_init_fbinfo(dev);
	if (!cursorfb) {
		ret = -ENOMEM;
		DPRINTK(KERN_ERR "PXA FB: cursorfb_init_fbinfo failed\n");
		goto failed;
	}

	/* set refernce to Overlays  */
	pxafbi->overlay1fb = overlay1fb;
	pxafbi->overlay2fb = overlay2fb;
	pxafbi->cursorfb = cursorfb;

	if (!directfb) {
		/* set refernce to BaseFrame */
		overlay1fb->basefb = pxafbi;
		overlay2fb->basefb = pxafbi;
	}
	cursorfb->basefb = pxafbi;

	if (!directfb) {
		/* Initialize video memory for overlay1,
		 * we can't do it here for overlay2
		 * (because  of mode for overlay2 is unknown)
		 */
		ret = overlay1fb_map_video_memory(overlay1fb);
		if (ret) {
			dev_err(dev,
				"Failed to allocate overlay1 video RAM: %d\n",
				ret);
			ret = -ENOMEM;
			goto failed;
		}

		/*
		 * This makes sure that our colour bitfield
		 * descriptors are correctly initialised.
		 */

		/* Overlay 1 */
		overlay1fb_check_var(&pxafbi->overlay1fb->fb.var,
				     &pxafbi->overlay1fb->fb);

		ret = register_framebuffer(&overlay1fb->fb);
		if (ret < 0)
			goto failed;

		/*Overlay 2 */
		overlay2fb_check_var(&pxafbi->overlay2fb->fb.var,
				     &pxafbi->overlay2fb->fb);

		ret = register_framebuffer(&overlay2fb->fb);
		if (ret < 0)
			goto failed;
	}

	/* Cursor */
	cursorfb_check_var(&pxafbi->cursorfb->fb.var, &pxafbi->cursorfb->fb);

	ret = register_framebuffer(&cursorfb->fb);
	if (ret < 0)
		goto failed;

#endif				/* CONFIG_PXA27x */

#ifdef CONFIG_FB_PXA_MINILCD
	pxafb_minilcd_register(&pxafbi->fb);
#endif

	/*
	 * Ok, now enable the LCD controller
	 */
	set_ctrlr_state(pxafbi, C_ENABLE);
	pdev->dev.driver = &pxafb_driver;

	return 0;

failed:
	dev_set_drvdata(dev, NULL);
	if (pxafbi) {
		fb_dealloc_cmap(&pxafbi->fb.cmap);
		kfree(pxafbi);
	}
#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
	if (overlay1fb)
		kfree(overlay1fb);
	if (overlay2fb)
		kfree(overlay2fb);
	if (cursorfb)
		kfree(cursorfb);
#endif

	return ret;
}

static void pxafb_ldm_register(void)
{
	driver_register(&pxafb_driver);
}


#ifndef MODULE
int __devinit pxafb_setup(char *options)
{
# ifdef CONFIG_FB_PXA_PARAMETERS
	if (options)
		strlcpy(g_options, options, sizeof(g_options));
# endif
	return 0;
}
#else
# ifdef CONFIG_FB_PXA_PARAMETERS
module_param_string(options, g_options, sizeof(g_options), 0);
MODULE_PARM_DESC(options, "LCD parameters (see Documentation/fb/pxafb.txt)");
# endif
#ifdef CONFIG_PXA27x
module_param(directfb, int, 0);
MODULE_PARM_DESC(directfb,
		 "\nSet this to work with the DirectFB PXA27x gfxdriver.\n"
		 "The gfxdriver takes over control of the overlays, so they\n"
		 "are not initialized or registered as seperate framebuffer\n"
		 "devices in this driver. Also, the framebuffer memory must\n"
		 "be allocated as one contiguous buffer big enough to hold\n"
		 "the primary base layer and all overlays at the largest\n"
		 "allowed resolutions in each layer\n");
#endif /* CONFIG_PXA27x */
#endif

int __devinit pxafb_init(void)
{
#ifndef MODULE
	char *option = NULL;

	if (fb_get_options("pxafb", &option))
		return -ENODEV;
	pxafb_setup(option);
#endif

	pxafb_ldm_register();
	return 0;
}

module_init(pxafb_init);

MODULE_DESCRIPTION("loadable framebuffer driver for PXA");
MODULE_LICENSE("GPL");
