#ifndef __PXAFB_MINILCD_H__
#define __PXAFB_MINILCD_H__

/*
 * linux/drivers/video/pxafb_minilcd.h
 *    -- Intel PXA3xx mini-LCD Controller Frame Buffer Device
 *
 * (C) Copyright 2006 Marvell International Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */

#include <asm/ioctl.h>

/* commands for pxafb_minilcd_ioctl() */

#define PXAFB_MINILCD_ENABLE		_IOW('F', 0x80, unsigned int)
#define PXAFB_MINILCD_BACKLIGHT		_IOW('F', 0x81, unsigned int)
#define PXAFB_MINILCD_WAKEUP		_IOW('F', 0x82, unsigned int)
#define PXAFB_MINILCD_FWAKEUP		_IOW('F', 0x83, unsigned int)
#define PXAFB_MINILCD_FRAMEDATA		_IOW('F', 0x84, void *)

/* Mini-LCD register definitions */

#define MLCCR0			__REG_2(0x46000000)
#define MLCCR1			__REG_2(0x46000004)
#define MLCCR2			__REG_2(0x46000008)
#define MLSADD			__REG_2(0x4600000C)
#define MLFRMCNT		__REG_2(0x46000010)

#define MLCCR0_OEP		(1 << 11)
#define MLCCR0_PCP		(1 << 10)
#define MLCCR0_VSP		(1 << 9)
#define MLCCR0_HSP		(1 << 8)
#define MLCCR0_PCD(d)		((d) & 0xff)

#define MLCCR1_BLW(n)		(((n) & 0xff) << 24)
#define MLCCR1_ELW(n)		(((n) & 0xff) << 16)
#define MLCCR1_HSW(n)		(((n) & 0x3f) << 10)
#define MLCCR1_PPL(n)		(((n) & 0x3ff)

#define MLCCR2_BFW(n)		(((n) & 0xff) << 24)
#define MLCCR2_EFW(n)		(((n) & 0xff) << 16)
#define MLCCR2_VSW(n)		(((n) & 0x3f) << 10)
#define MLCCR2_LPP(n)		(((n) & 0x3ff)

#define MLFRMCNT_WKUP		(1U << 31)
#define MLFRMCNT_FWKUP		(1U << 30)
#define MLFRMCNT_FRCOUNT(n)	((n) & 0x3ff)
#define MLFRMCNT_FRCOUNT_MASK	(0x3ff)

/* Shadows for Mini-LCD controller registers */
struct pxafb_minilcd_reg {
	uint32_t mlccr0;
	uint32_t mlccr1;
	uint32_t mlccr2;
	uint32_t mlsadd;
	uint32_t mlfrmcnt;
};

/*
 * pxafb_minilcd_info - run-time information to enable mini-lcd
 * enable     - enable in low power mode (S0/D1/C2)
 * framecount - shadow of register MLFRMCNT
 * frameaddr  - shadow of register MLSADR
 * framedata  - points to the encoded data from user specified buffer,
 *              or NULL if the base frame buffer is going to be used.
 * framesize  - size of the encoded frame data if 'framedata' is not NULL
 */
struct pxafb_minilcd_info {
	unsigned int	enable;
	unsigned int	backlight;
	uint32_t	framecount;
	void *		framedata;
	size_t		framesize;

	uint32_t	sram_addr_phys; /* Physical address of the SRAM */
	void *		sram_addr_virt; /* Virtual address of the SRAM */
	void *		sram_save_to;	/* address to backup SRAM into */
	size_t		sram_save_size; /* size of saved SRAM */
};

extern int pxafb_minilcd_register(struct fb_info *);
extern int pxafb_minilcd_ioctl(struct inode *inode, struct file *file,
				unsigned int cmd, unsigned long arg,
				struct fb_info *info);

extern int pxafb_minilcd_enter(void);
extern int pxafb_minilcd_exit(void);

#endif /* __PXAFB_MINILCD_H__ */
