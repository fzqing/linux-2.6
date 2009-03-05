/*
 * xilinxfb.c
 *
 * Xilinx TFT LCD frame buffer driver
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * 2002-2005 (c) MontaVista Software, Inc.  This file is licensed under the
 * terms of the GNU General Public License version 2.  This program is licensed
 * "as is" without any warranty of any kind, whether express or implied.
 */

/*
 * This driver was based off of au1100fb.c by MontaVista rewritten for 2.6
 * by Embedded Alley Solutions <source@embeddedalley.com>, which in turn
 * was based off of skeletonfb.c, Skeleton for a frame buffer device by
 * Geert Uytterhoeven.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
#include <linux/device.h>
#else
#include <linux/platform_device.h>
#endif

#include <asm/io.h>

#define DRIVER_NAME		"xilinx_fb"
#define DRIVER_DESCRIPTION	"Xilinx TFT LCD frame buffer driver"

/*
 * The interface to the framebuffer is nice and simple.  There are two
 * control registers.  The first tells the LCD interface where in memory
 * the frame buffer is (only the 11 most significant bits are used, so
 * don't start thinking about scrolling).  The second allows the LCD to
 * be turned on or off as well as rotated 180 degrees.
 */
#define NUM_REGS	2
#define REG_FB_ADDR	0
#define REG_CTRL	1
#define REG_CTRL_ENABLE	 0x0001
#define REG_CTRL_ROTATE	 0x0002
#if defined(CONFIG_FB_XILINX_ROTATE)
#define REG_CTRL_DEFAULT (REG_CTRL_ENABLE | REG_CTRL_ROTATE)
#else
#define REG_CTRL_DEFAULT (REG_CTRL_ENABLE)
#endif				/* CONFIG_FB_XILINX_ROTATE */

/*
 * The hardware only handles a single mode: 640x480 24 bit true
 * color. Each pixel gets a word (32 bits) of memory.  Within each word,
 * the 8 most significant bits are ignored, the next 8 bits are the red
 * level, the next 8 bits are the green level and the 8 least
 * significant bits are the blue level.  Each row of the LCD uses 1024
 * words, but only the first 640 pixels are displayed with the other 384
 * words being ignored.  There are 480 rows.
 */
#define BYTES_PER_PIXEL	4
#define BITS_PER_PIXEL	(BYTES_PER_PIXEL * 8)
#define XRES		640
#define YRES		480
#define XRES_VIRTUAL	1024
#define YRES_VIRTUAL	YRES
#define LINE_LENGTH	(XRES_VIRTUAL * BYTES_PER_PIXEL)
#define FB_SIZE		(YRES_VIRTUAL * LINE_LENGTH)

#define PALETTE_ENTRIES_NO	16	/* passed to fb_alloc_cmap() */

/*
 * Here are the default fb_fix_screeninfo and fb_var_screeninfo structures
 */
static struct fb_fix_screeninfo xilinx_fb_fix __initdata = {
	.id =		"Xilinx",
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_TRUECOLOR,
	.smem_len =	FB_SIZE,
	.line_length =	LINE_LENGTH,
	.accel =	FB_ACCEL_NONE
};

static struct fb_var_screeninfo xilinx_fb_var __initdata = {
	.xres =			XRES,
	.yres =			YRES,
	.xres_virtual =		XRES_VIRTUAL,
	.yres_virtual =		YRES_VIRTUAL,

	.bits_per_pixel =	BITS_PER_PIXEL,

	.red =		{ 16, 8, 0 },
	.green =	{ 8, 8, 0 },
	.blue =		{ 0, 8, 0 },
	.transp =	{ 0, 0, 0 },

	.activate =	FB_ACTIVATE_NOW,
	.height = 	99,	/* in mm of NEC NL6448BC20-08 on ML300 */
	.width =	132	/* in mm of NEC NL6448BC20-08 on ML300 */
};

struct xilinxfb_drvdata {

	struct fb_info	info;		/* FB driver info record */

	unsigned long	regs_phys;	/* phys. address of the control registers */
	u32 		*regs;		/* virt. address of the control registers */

	unsigned char	*fb_virt;	/* virt. address of the frame buffer */
	dma_addr_t	fb_phys;	/* phys. address of the frame buffer */

	u32		pseudo_palette[16];	/* Fake palette of 16 colors */
};

#define to_xilinxfb_drvdata(_info) \
	container_of(_info, struct xilinxfb_drvdata, info)

static int
xilinx_fb_setcolreg(unsigned regno, unsigned red, unsigned green, unsigned blue,
	unsigned transp, struct fb_info *fbi)
{
	u32 *palette = fbi->pseudo_palette;

	if (regno >= PALETTE_ENTRIES_NO)
		return -EINVAL;

	if (fbi->var.grayscale) {
		/* Convert color to grayscale.
		 * grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue =
			(red * 77 + green * 151 + blue * 28 + 127) >> 8;
	}

	/* fbi->fix.visual is always FB_VISUAL_TRUECOLOR */

	/* We only handle 8 bits of each color. */
	red >>= 8;
	green >>= 8;
	blue >>= 8;
	palette[regno] = (red << 16) | (green << 8) | blue;

	return 0;
}

static int
xilinx_fb_blank(int blank_mode, struct fb_info *fbi)
{
	struct xilinxfb_drvdata *drvdata = to_xilinxfb_drvdata(fbi);

	switch (blank_mode) {
	case VESA_NO_BLANKING:
		/* turn on panel */
		out_be32(drvdata->regs + REG_CTRL, REG_CTRL_DEFAULT);
		break;

	case VESA_VSYNC_SUSPEND:
	case VESA_HSYNC_SUSPEND:
	case VESA_POWERDOWN:
		/* turn off panel */
		out_be32(drvdata->regs + REG_CTRL, 0);
	default:
		break;

	}
	return 0; /* success */
}

static int
xilinx_fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *fbi)
{
	if (var->xoffset != 0 || var->yoffset != 0)
		return -EINVAL;

	return 0;
}

static struct fb_ops xilinxfb_ops =
{
	.owner			= THIS_MODULE,
	.fb_setcolreg		= xilinx_fb_setcolreg,
	.fb_blank		= xilinx_fb_blank,
	.fb_pan_display		= xilinx_fb_pan_display,
	.fb_fillrect		= cfb_fillrect,
	.fb_copyarea		= cfb_copyarea,
	.fb_imageblit		= cfb_imageblit,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
	.fb_cursor		= soft_cursor,
#endif
};

/* === The device driver === */

static int
xilinxfb_drv_probe(struct device *dev)
{
	struct xilinxfb_drvdata *drvdata;
	struct resource *regs_res;
	int retval;

	if (!dev)
		return -EINVAL;

	drvdata = kmalloc(sizeof(struct xilinxfb_drvdata), GFP_KERNEL);
	if (!drvdata) {
		printk(KERN_ERR "Couldn't allocate device private record\n");
		return -ENOMEM;
	}
	memset((void*)drvdata, 0, sizeof(struct xilinxfb_drvdata));
	dev_set_drvdata(dev, (void *)drvdata);

	/* Map the control registers in */
	regs_res = platform_get_resource(to_platform_device(dev),
			IORESOURCE_MEM, 0);
	if (!regs_res || (regs_res->end - regs_res->start + 1 < 8)) {
		printk(KERN_ERR "Couldn't get registers resource\n");
		retval = -EFAULT;
		goto failed1;
	}

	if (!request_mem_region(regs_res->start, 8, DRIVER_NAME)) {
		printk(KERN_ERR "Couldn't lock memory region at 0x%08lX\n",
			regs_res->start);
		retval = -EBUSY;
		goto failed1;
	}

	drvdata->regs_phys = regs_res->start;
	drvdata->regs = (u32 *) ioremap(regs_res->start, 8);

	/* Allocate the framebuffer memory */
	drvdata->fb_virt = dma_alloc_coherent(dev, PAGE_ALIGN(FB_SIZE),
				&drvdata->fb_phys, GFP_KERNEL);
	if (!drvdata->fb_virt) {
		printk(KERN_ERR "Could not allocate frame buffer memory\n");
		retval = -ENOMEM;
		goto failed2;
	}

	/* Clear (turn to black) the framebuffer */
	memset((void *) drvdata->fb_virt, 0, FB_SIZE);

	/* Tell the hardware where the frame buffer is */
	out_be32(drvdata->regs + REG_FB_ADDR, drvdata->fb_phys);

	/* Turn on the display */
	out_be32(drvdata->regs + REG_CTRL, REG_CTRL_DEFAULT);

	/* Fill struct fb_info */
	drvdata->info.screen_base = drvdata->fb_virt;
	drvdata->info.fbops = &xilinxfb_ops;
	drvdata->info.fix = xilinx_fb_fix;
	drvdata->info.fix.smem_start = drvdata->fb_phys;
	drvdata->info.pseudo_palette = drvdata->pseudo_palette;

	if (fb_alloc_cmap(&drvdata->info.cmap, PALETTE_ENTRIES_NO, 0) < 0) {
		printk(KERN_ERR "Fail to allocate colormap (%d entries)\n",
			PALETTE_ENTRIES_NO);
		retval = -EFAULT;
		goto failed3;
	}

	drvdata->info.flags = FBINFO_DEFAULT;
	drvdata->info.var = xilinx_fb_var;

	/* Register new frame buffer */
	if (register_framebuffer(&drvdata->info) < 0) {
		printk(KERN_ERR "Could not register frame buffer\n");
		retval = -EINVAL;
		goto failed4;
	}

	return 0;	/* success */

failed4:
	fb_dealloc_cmap(&drvdata->info.cmap);

failed3:
	dma_free_coherent(dev, PAGE_ALIGN(FB_SIZE), drvdata->fb_virt,
		drvdata->fb_phys);

	/* Turn off the display */
	out_be32(drvdata->regs + REG_CTRL, 0);
	iounmap(drvdata->regs);

failed2:
	release_mem_region(regs_res->start, 8);

failed1:
	kfree(drvdata);
	dev_set_drvdata(dev, NULL);

	return retval;
}

static int
xilinxfb_drv_remove(struct device *dev)
{
	struct xilinxfb_drvdata *drvdata;

	if (!dev)
		return -ENODEV;

	drvdata = (struct xilinxfb_drvdata *) dev_get_drvdata(dev);

#if !defined(CONFIG_FRAMEBUFFER_CONSOLE) && defined(CONFIG_LOGO)
	xilinx_fb_blank(VESA_POWERDOWN, &drvdata->info);
#endif

	unregister_framebuffer(&drvdata->info);

	fb_dealloc_cmap(&drvdata->info.cmap);

	dma_free_coherent(dev, PAGE_ALIGN(FB_SIZE), drvdata->fb_virt,
		drvdata->fb_phys);

	/* Turn off the display */
	out_be32(drvdata->regs + REG_CTRL, 0);
	iounmap(drvdata->regs);

	release_mem_region(drvdata->regs_phys, 8);

	kfree(drvdata);
	dev_set_drvdata(dev, NULL);

	return 0;
}


static struct device_driver xilinxfb_driver = {
	.name		= DRIVER_NAME,
	.bus		= &platform_bus_type,

	.probe		= xilinxfb_drv_probe,
	.remove		= xilinxfb_drv_remove
};

static int __init
xilinxfb_init(void)
{
	/*
	 * No kernel boot options used,
	 * so we just need to register the driver
	 */
	return driver_register(&xilinxfb_driver);
}

static void __exit
xilinxfb_cleanup(void)
{
	driver_unregister(&xilinxfb_driver);
}

module_init(xilinxfb_init);
module_exit(xilinxfb_cleanup);

MODULE_AUTHOR("MontaVista Software, Inc. <source@mvista.com>");
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_LICENSE("GPL");
