#ifndef __PXAFB_H__
#define __PXAFB_H__

/*
 * linux/drivers/video/pxafb.h
 *    -- Intel PXA250/210 LCD Controller Frame Buffer Device
 *
 *  Copyright (C) 1999 Eric A. Thomas.
 *  Copyright (C) 2004 Jean-Frederic Clere.
 *  Copyright (C) 2004 Ian Campbell.
 *  Copyright (C) 2004 Jeff Lackey.
 *   Based on sa1100fb.c Copyright (C) 1999 Eric A. Thomas
 *  which in turn is
 *   Based on acornfb.c Copyright (C) Russell King.
 *
 *  2001-08-03: Cliff Brake <cbrake@acclent.com>
 *	 - ported SA1100 code to PXA
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */

#ifdef CONFIG_FB_PXA_MINILCD
#include "pxafb_minilcd.h"
#endif

/*
 * These are the bitfields for each
 * display depth that we support.
 */
struct pxafb_rgb {
	struct fb_bitfield red;
	struct fb_bitfield green;
	struct fb_bitfield blue;
	struct fb_bitfield transp;
};

/* Shadows for LCD controller registers */
struct pxafb_lcd_reg {
	unsigned int lccr0;
	unsigned int lccr1;
	unsigned int lccr2;
	unsigned int lccr3;
};

/* PXA LCD DMA descriptor */
struct pxafb_dma_descriptor {
	unsigned int fdadr;
	unsigned int fsadr;
	unsigned int fidr;
	unsigned int ldcmd;
};

struct pxafb_info {
	struct fb_info		fb;
	struct device		*dev;

	/*
	 * These are the addresses we mapped
	 * the framebuffer memory region to.
	 */
	/* raw memory addresses */
	dma_addr_t		map_dma;	/* physical */
	u_char *		map_cpu;	/* virtual */
	u_int			map_size;

	/* addresses of pieces placed in raw buffer */
	u_char *		screen_cpu;	/* virtual address of frame buffer */
	dma_addr_t		screen_dma;	/* physical address of frame buffer */
	u16 *			palette_cpu;	/* virtual address of palette memory */
	dma_addr_t		palette_dma;	/* physical address of palette memory */
	u_int			palette_size;

	/* DMA descriptors */
	struct pxafb_dma_descriptor * 	dmadesc_fblow_cpu;
	dma_addr_t		dmadesc_fblow_dma;
	struct pxafb_dma_descriptor * 	dmadesc_fbhigh_cpu;
	dma_addr_t		dmadesc_fbhigh_dma;
	struct pxafb_dma_descriptor *	dmadesc_palette_cpu;
	dma_addr_t		dmadesc_palette_dma;

	dma_addr_t		fdadr0;
	dma_addr_t		fdadr1;

	u_int			lccr0;
	u_int			lccr3;
	u_int			cmap_inverse:1,
				cmap_static:1,
				unused:30;

	u_int			reg_lccr0;
	u_int			reg_lccr1;
	u_int			reg_lccr2;
	u_int			reg_lccr3;

	volatile u_char		state;
	volatile u_char		task_state;
	struct semaphore	ctrlr_sem;
	wait_queue_head_t	ctrlr_wait;
	wait_queue_head_t       eof_wait;
	struct work_struct	task;
	spinlock_t		rt_lock;

	u_int                   eof_mask; /* end-of-frame flags,
					     mask of DMA channels */

#ifdef CONFIG_CPU_FREQ
	struct notifier_block	freq_transition;
	struct notifier_block	freq_policy;
#endif

#if defined(CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
	struct overlayfb_info *overlay1fb;
	struct overlayfb_info *overlay2fb;
	struct overlayfb_info *cursorfb;
#endif				/* CONFIG_PXA27x */

#ifdef CONFIG_FB_PXA_MINILCD
	struct pxafb_minilcd_info minilcd_info;
#endif

};

#if defined(CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
struct overlayfb_info {
	struct fb_info fb;
	struct device *dev;

	struct pxafb_info *basefb;

	void *map_cpu;
	void *screen_cpu;
	void *palette_cpu;
	unsigned long map_size;
	unsigned long palette_size;

	dma_addr_t screen_dma;
	dma_addr_t map_dma;
	dma_addr_t palette_dma;

	unsigned long enabled;
	volatile u_char state;

	/* overlay specific info */
	unsigned long xpos;	/* screen position (x, y) */
	unsigned long ypos;
	unsigned long format;

	/* additional */
	union {
		struct pxafb_dma_descriptor *dma0;
		struct pxafb_dma_descriptor *dma1;
		struct {
			struct pxafb_dma_descriptor *dma2;
			struct pxafb_dma_descriptor *dma3;
			struct pxafb_dma_descriptor *dma4;
		};
		struct {
			struct pxafb_dma_descriptor *dma5_pal;
			struct pxafb_dma_descriptor *dma5_frame;
		};
	};

};
#endif				/* CONFIG_PXA27x */

#define TO_INF(ptr,member) container_of(ptr,struct pxafb_info,member)

/*
 * These are the actions for set_ctrlr_state
 */
#define C_DISABLE		(0)
#define C_ENABLE		(1)
#define C_DISABLE_CLKCHANGE	(2)
#define C_ENABLE_CLKCHANGE	(3)
#define C_REENABLE		(4)
#define C_DISABLE_PM		(5)
#define C_ENABLE_PM		(6)
#define C_STARTUP		(7)
#define C_BLANK                 (8)
#define C_UNBLANK               (9)

#define PXA_NAME	"PXA"

#define FBIO_WAITFORVSYNC       _IOW('F', 0x20, u_int32_t)

/*
 *  Debug macros
 */
#if DEBUG
#  define DPRINTK(fmt, args...)	printk("%s: " fmt, __FUNCTION__ , ## args)
#else
#  define DPRINTK(fmt, args...)
#endif

/*
 * Minimum and maximum X and Y resolutions
 */
#define MIN_XRES	64
#define MIN_YRES	64
#define MAX_XRES       800
#define MAX_YRES       600
#define MAX_BPP         32

#endif /* __PXAFB_H__ */
