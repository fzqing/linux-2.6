/*
 * drivers/video/smi/smi_base.c
 *
 * LynxEM+/EM4+(Silicon Motion Inc.) fb driver	for NEC Electronics Corporation VR5701 SolutionGearII
 *
 * Author: Sergey Podstavin <spodstavin@ru.mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/selection.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/console.h>
#include "../console/fbcon.h"
#include "smifb.h"
#include "smi_hw.h"

/*
 * Card Identification
 *
 */
static struct pci_device_id smifb_pci_tbl[] __devinitdata = {
	{PCI_VENDOR_ID_SMI, PCI_DEVICE_ID_SMI_LYNX_EM_PLUS,
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},	/* Lynx EM+/EM4+ */
	{PCI_VENDOR_ID_SMI, PCI_DEVICE_ID_SMI_LYNX_3DM,
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},	/* Lynx 3DM/3DM+/3DM4+ */
	{0,}			/* terminate list */
};

MODULE_DEVICE_TABLE(pci, smifb_pci_tbl);

/*
 *
 * global variables
 *
 */

#ifdef CONFIG_DISPLAY_1024x768
/* 1024x768, 16bpp, 60Hz */
static struct fb_var_screeninfo smifb_default_var = {
      xres:1024,
      yres:768,
      xres_virtual:1024,
      yres_virtual:768,
      xoffset:0,
      yoffset:0,
      bits_per_pixel:16,
      grayscale:0,
      red:{11, 5, 0},
      green:{5, 6, 0},
      blue:{0, 5, 0},
      transp:{0, 0, 0},
      nonstd:0,
      activate:0,
      height:-1,
      width:-1,
      accel_flags:0,
      pixclock:39721,		/* D */
      left_margin:138,
      right_margin:24,
      upper_margin:24,
      lower_margin:4,
      hsync_len:160,
      vsync_len:6,
      sync:0,
      vmode:FB_VMODE_NONINTERLACED
};
#else
/* 640x480, 16bpp, 60Hz */
static struct fb_var_screeninfo smifb_default_var = {
      xres:640,
      yres:480,
      xres_virtual:640,
      yres_virtual:480,
      xoffset:0,
      yoffset:0,
      bits_per_pixel:16,
      grayscale:0,
      red:{11, 5, 0},
      green:{5, 6, 0},
      blue:{0, 5, 0},
      transp:{0, 0, 0},
      nonstd:0,
      activate:0,
      height:-1,
      width:-1,
      accel_flags:0,
      pixclock:39721,		/* D */
      left_margin:82,
      right_margin:16,
      upper_margin:19,
      lower_margin:1,
      hsync_len:152,
      vsync_len:4,
      sync:0,
      vmode:FB_VMODE_NONINTERLACED
};
#endif

static char drvrname[] = "NEC video driver for SMI LynxEM+";

/*
 *
 * general utility functions
 *
 */

static void
smi_load_video_mode(struct smifb_info *sinfo,
		    struct fb_var_screeninfo *video_mode)
{
	int bpp, width, height;
	int hDisplaySize, hDisplay, hStart, hEnd, hTotal;
	int vDisplay, vStart, vEnd, vTotal;
	int dotClock;

	pr_debug("smi_load_video_mode: video_mode->xres = %d\n",
		 video_mode->xres);
	pr_debug("                   :             yres = %d\n",
		 video_mode->yres);
	pr_debug("                   :             xres_virtual = %d\n",
		 video_mode->xres_virtual);
	pr_debug("                   :             yres_virtual = %d\n",
		 video_mode->yres_virtual);
	pr_debug("                   :             xoffset = %d\n",
		 video_mode->xoffset);
	pr_debug("                   :             yoffset = %d\n",
		 video_mode->yoffset);
	pr_debug("                   :             bits_per_pixel = %d\n",
		 video_mode->bits_per_pixel);

	/* smifb_blank(1, (struct fb_info*)sinfo); */
	bpp = video_mode->bits_per_pixel;
	if (bpp == 16 && video_mode->green.length == 5)
		bpp = 15;

	/* horizontal params */
	width = video_mode->xres_virtual;
	hDisplaySize = video_mode->xres;	/* number of pixels for one horizontal line */
	hDisplay = (hDisplaySize / 8) - 1;	/* number of character clocks */
	hStart = (hDisplaySize + video_mode->right_margin) / 8 + 2;	/* h-blank start character clocks */
	hEnd = (hDisplaySize + video_mode->right_margin + video_mode->hsync_len) / 8 - 1;	/* h-sync end */
	hTotal = (hDisplaySize + video_mode->right_margin + video_mode->hsync_len + video_mode->left_margin) / 8 - 1;	/* character clock from h-sync to next h-sync */

	/* vertical params */
	height = video_mode->yres_virtual;
	vDisplay = video_mode->yres - 1;	/* number of lines */
	vStart = video_mode->yres + video_mode->lower_margin - 1;	/* v-sync pulse start */
	vEnd = video_mode->yres + video_mode->lower_margin + video_mode->vsync_len - 1;	/* v-sync end */
	vTotal = video_mode->yres + video_mode->lower_margin + video_mode->vsync_len + video_mode->upper_margin + 2;	/* number of scanlines (v-blank end) */

	dotClock = 1000000000 / video_mode->pixclock;

	smi_set_moderegs(sinfo, bpp, width, height,
			 hDisplaySize,
			 hDisplay, hStart, hEnd, hTotal,
			 vDisplay, vStart, vEnd, vTotal,
			 dotClock, video_mode->sync);
}

/*
 *
 * framebuffer operations
 *
 */
static int
smifb_get_fix(struct fb_fix_screeninfo *fix, int con, struct fb_info *info)
{
	struct smifb_info *sinfo = (struct smifb_info *)info;

	pr_debug("smifb_get_fix");
	fix->smem_start = sinfo->fb_base_phys;
	fix->smem_len = sinfo->fbsize;
	fix->mmio_start = sinfo->dpr_base_phys;
	fix->mmio_len = sinfo->dpport_size;

	fix->xpanstep = 0;	/* FIXME: no xpanstep for now */
	fix->ypanstep = 1;	/* FIXME: no ypanstep for now */
	fix->ywrapstep = 0;	/* FIXME: no ywrap for now */

	return 0;
}

static int vgxfb_setcolreg(unsigned regno, unsigned red, unsigned green,
			   unsigned blue, unsigned transp, struct fb_info *info)
{
	if (regno > 15)
		return 1;

	((u16 *) (info->pseudo_palette))[regno] =
	    (red & 0xf800) | (green & 0xfc00 >> 5) | (blue & 0xf800 >> 11);
	return 0;
}

/*
 * Initialization helper functions
 *
 */
/* kernel interface */
static struct fb_ops smifb_ops = {
	.owner = THIS_MODULE,
	.fb_setcolreg = vgxfb_setcolreg,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_cursor = soft_cursor,
};

/*
 * VGA registers
 *
 */
static void Unlock(struct smifb_info *sinfo)
{
	pr_debug("Unlock");
	regSR_write(sinfo->mmio, 0x33, regSR_read(sinfo->mmio, 0x33) & 0x20);
}

static void UnlockVGA(struct smifb_info *sinfo)
{
	pr_debug("UnlockVGA");
	regCR_write(sinfo->mmio, 0x11, regCR_read(sinfo->mmio, 0x11) & 0x7f);
}

static struct fb_fix_screeninfo vgxfb_fix = {
	.id = "vgxFB",
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_TRUECOLOR,
#ifdef CONFIG_DISPLAY_1024x768
	.line_length = 1024 * 2,
#else
	.line_length = 640 * 2,
#endif
	.accel = FB_ACCEL_NONE,
};

static u32 colreg[17];

/*
 * PCI bus
 *
 */
static int __devinit
smifb_probe(struct pci_dev *pd, const struct pci_device_id *ent)
{
	int len;
	int res;
	u16 cmd;
	struct smifb_info *sinfo;
	struct fb_info *info;

	pr_debug("smifb_probe");

	pr_debug("vendor id        %04x\n", pd->vendor);
	pr_debug("device id        %04x\n", pd->device);
	pr_debug("sub vendor id    %04x\n", pd->subsystem_vendor);
	pr_debug("sub device id    %04x\n", pd->subsystem_device);

	pr_debug("base0 start addr %08x\n",
		 (unsigned int)pci_resource_start(pd, 0));
	pr_debug("base0 end   addr %08x\n",
		 (unsigned int)pci_resource_end(pd, 0));
	pr_debug("base0 region len %08x\n",
		 (unsigned int)pci_resource_len(pd, 0));
	pr_debug("base0 flags      %08x\n",
		 (unsigned int)pci_resource_flags(pd, 0));

	pci_read_config_word(pd, PCI_STATUS, &cmd);
	pr_debug("PCI status      %04x\n", cmd);

	pci_read_config_word(pd, PCI_COMMAND, &cmd);
	pr_debug("PCI command      %04x\n", cmd);

	cmd |= PCI_COMMAND_MEMORY | PCI_COMMAND_IO;
	pci_write_config_word(pd, PCI_COMMAND, cmd);

	pci_read_config_word(pd, PCI_STATUS, &cmd);
	pr_debug("PCI status      %04x\n", cmd);
	pci_read_config_word(pd, PCI_COMMAND, &cmd);
	pr_debug("PCI command      %04x\n", cmd);

	/* allocate memory resources */
	sinfo = kmalloc(sizeof(struct smifb_info), GFP_KERNEL);
	if (!sinfo) {
		goto err_out;
	}
	memset(sinfo, 0, sizeof(struct smifb_info));

	/* driver name */
	sinfo->drvr_name = drvrname;

	sinfo->pd = pd;
	sinfo->base_phys = pci_resource_start(sinfo->pd, 0);	/* Frame Buffer base address */
	len = pci_resource_len(sinfo->pd, 0);
	pr_debug("len = %lX\n", len);
	if (!request_mem_region(sinfo->base_phys, len, "smifb")) {
		printk(KERN_ERR "cannot reserve FrameBuffer and MMIO region\n");
		goto err_out_kfree;
	}

	if ((res = pci_enable_device(sinfo->pd)) < 0) {
		printk(KERN_ERR "smifb: failed to enable -- err %d\n", res);
		goto err_out_free_base;
	}

	pci_read_config_word(pd, PCI_COMMAND, &cmd);
	pr_debug(KERN_INFO "PCI command      %04x\n", cmd);

	{
		unsigned int pseudo_io, pseudo_io_len;
		unsigned char *pseudo_io_p;

		*(unsigned long *)0xbe000610 = 0x10000012;	/* CHANGE to PCI IO ACCESS */
		asm("sync");
		pseudo_io = pci_resource_start(sinfo->pd, 0);
		pseudo_io_len = pci_resource_len(sinfo->pd, 0);
		pseudo_io_p = ioremap(pseudo_io, pseudo_io_len);

		VGA_WRITE8(pseudo_io_p, 0x3c3, 0x40);
		regSR_write(pseudo_io_p, 0x00, 0x00);
		regSR_write(pseudo_io_p, 0x17, 0xe2);
		regSR_write(pseudo_io_p, 0x18, 0xff);

		iounmap(pseudo_io_p);
		*(unsigned long *)0xbe000610 = 0x10000016;	/* PCI MEM ACCESS */
		asm("sync");
	}
	sinfo->base = ioremap(sinfo->base_phys, len);	/* FB+DPD+DPR+VPR+CPR+MMIO */
	if (!sinfo->base) {
		goto err_out_free_base;
	}
	switch ((sinfo->pd)->device) {
	case PCI_DEVICE_ID_SMI_LYNX_EM_PLUS:
		sinfo->dpport = (caddr_t) (sinfo->base + DPPORT_BASE_OFFSET);
		sinfo->dpr = (caddr_t) (sinfo->base + DP_BASE_OFFSET);
		sinfo->vpr = (caddr_t) (sinfo->base + VP_BASE_OFFSET);
		sinfo->cpr = (caddr_t) (sinfo->base + CP_BASE_OFFSET);
		sinfo->mmio = (caddr_t) (sinfo->base + IO_BASE_OFFSET);
		sinfo->fb_base = (caddr_t) (sinfo->base + 0);
		break;
	case PCI_DEVICE_ID_SMI_LYNX_3DM:
		sinfo->dpport =
		    (caddr_t) (sinfo->base + LYNX3DM_DPPORT_BASE_OFFSET);
		sinfo->dpr = (caddr_t) (sinfo->base + LYNX3DM_DP_BASE_OFFSET);
		sinfo->vpr = (caddr_t) (sinfo->base + LYNX3DM_VP_BASE_OFFSET);
		sinfo->cpr = (caddr_t) (sinfo->base + LYNX3DM_CP_BASE_OFFSET);
		sinfo->mmio = (caddr_t) (sinfo->base + LYNX3DM_IO_BASE_OFFSET);
		sinfo->fb_base =
		    (caddr_t) (sinfo->base + LYNX3DM_FB_BASE_OFFSET);
		break;
	}
	regSR_write(sinfo->mmio, 0x18, 0x11);

	pr_debug("sinfo->dpport = 0x%08x\n", (u_int32_t) sinfo->dpport);
	pr_debug("sinfo->dpr  = 0x%08x, sinfo->vpr   = 0x%08x\n",
		 (unsigned int)sinfo->dpr, (unsigned int)sinfo->vpr);
	pr_debug("sinfo->cpr  = 0x%08x, sinfo->mmio  = 0x%08x\n",
		 (unsigned int)sinfo->cpr, (unsigned int)sinfo->mmio);

	/* Set the chip in color mode and unlock the registers */
	VGA_WRITE8(sinfo->mmio, 0x3c2, 0x2b);	/* Miscellaneous Output Register ( write 0x3c2, read 0x3cc ) */

	Unlock(sinfo);
	UnlockVGA(sinfo);

	/* save the current chip status */
	switch ((sinfo->pd)->device) {
	case PCI_DEVICE_ID_SMI_LYNX_EM_PLUS:
		regSR_write(sinfo->mmio, 0x62, 0xff);
		regSR_write(sinfo->mmio, 0x6a, 0x0c);
		regSR_write(sinfo->mmio, 0x6b, 0x02);

		*(u32 *) (sinfo->fb_base + 4) = 0xaa551133;
		pr_debug("       *(u32 *)(sinfo->fb_base +4) = 0x%08x\n",
			 *(u32 *) (sinfo->fb_base + 4));
		if (*(u32 *) (sinfo->fb_base + 4) != 0xaa551133) {
			/* Program the MCLK to 130MHz */
			regSR_write(sinfo->mmio, 0x6a, 0x10);
			regSR_write(sinfo->mmio, 0x6b, 0x02);
			regSR_write(sinfo->mmio, 0x62, 0x3e);
			sinfo->fbsize = 2 * 1024 * 1024;	/* LynxEM+ */
			pr_debug
			    ("ChipID = LynxEM+. Force the MCLK to 85MHz and the memory size to 2MiB\n");
		} else {
			sinfo->fbsize = 4 * 1024 * 1024;	/* LynxEM4+ */
			pr_debug
			    ("ChipID = LynxEM4+. Force the MCLK to 85MHz and the memory size to 4MiB\n");
		}
		sinfo->fb_base_phys = sinfo->base_phys;
		break;
	case PCI_DEVICE_ID_SMI_LYNX_3DM:
		{
			int tmp;
			int mem_table[4] = { 8, 16, 0, 4 };
			tmp = (regSR_read(sinfo->mmio, 0x76) & 0xff);
			pr_debug("%02x\n", tmp);
			sinfo->fbsize = mem_table[(tmp >> 6)] * 1024 * 1024;

			regSR_write(sinfo->mmio, 0x62, 0xff);
			regSR_write(sinfo->mmio, 0x6a, 0x0c);
			regSR_write(sinfo->mmio, 0x6b, 0x02);

			sinfo->fb_base_phys =
			    sinfo->base_phys + LYNX3DM_FB_BASE_OFFSET;
		}
		break;
	default:
		/* this driver supports only LynxEM+/EM4+ */
		goto err_out_free_base;
	};

	info = &(sinfo->info);
	smifb_get_fix(&vgxfb_fix, -1, info);

	info->flags = FBINFO_FLAG_DEFAULT;
	info->fbops = &smifb_ops;
	info->var = smifb_default_var;
	info->fix = vgxfb_fix;
	info->pseudo_palette = colreg;
	info->screen_base = sinfo->fb_base;

	smi_load_video_mode(sinfo, &smifb_default_var);

	if (register_framebuffer(&sinfo->info) < 0) {
		goto err_out_free_base;
	}
	pci_set_drvdata(pd, sinfo);

	printk(KERN_INFO "smifb: " "framebuffer (%s)\n", sinfo->drvr_name);

	return 0;

      err_out_free_base:
	release_mem_region(sinfo->base_phys, len);
      err_out_kfree:
	kfree(sinfo);
      err_out:
	return -ENODEV;
}

static void __devexit smifb_remove(struct pci_dev *pd)
{
	struct smifb_info *sinfo = pci_get_drvdata(pd);
	pr_debug("smifb_remove");

	if (!sinfo)
		return;

	unregister_framebuffer(&sinfo->info);

	/* stop the lynx chip */
	release_mem_region(sinfo->base_phys, pci_resource_len(sinfo->pd, 0));
	kfree(sinfo);
	pci_set_drvdata(pd, NULL);
}

/*
 * Initialization
 *
 */
#ifndef MODULE
int __init smifb_setup(char *options)
{
	pr_debug("smifb_setup");

	if (!options || options)
		return 0;
	return 0;
}
#endif				/* not MODULE */

static struct pci_driver smifb_driver = {
	.name = "smifb",
	.id_table = smifb_pci_tbl,
	.probe = smifb_probe,
	.remove = __devexit_p(smifb_remove),
};

/*
 * Driver initialization
 */
int __init smifb_init(void)
{
	pr_debug("smifb_init");
	return pci_module_init(&smifb_driver);
}

void __exit smifb_exit(void)
{
	pci_unregister_driver(&smifb_driver);
}

module_init(smifb_init);
module_exit(smifb_exit);

MODULE_AUTHOR("Sergey Podstavin");
MODULE_DESCRIPTION("Framebuffer driver for NEC Electronics Corporation VR5701 SolutionGearII");
MODULE_LICENSE("GPL");
