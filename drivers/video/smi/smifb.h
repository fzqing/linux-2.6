/*
 * drivers/video/smi/smifb.h
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

#ifndef __SMIFB_H__
#define __SMIFB_H__

#define FBCON_HAS_CFB16

enum ScreenModes {
	DISPLAY_640x480x16,
	DISPLAY_800x600x16,
	DISPLAY_1024x768x16,
	DISPLAY_640x480x24,
	DISPLAY_800x600x24,
	DISPLAY_LCD_400x232x16,
	modeNums,
};

#define SMI_DEFAULT_MODE	DISPLAY_640x480x16

#define SIZE_SR00_SR04		(0x04 - 0x00 + 1)
#define SIZE_SR10_SR24		(0x24 - 0x10 + 1)
#define SIZE_SR30_SR75		(0x75 - 0x30 + 1)
#define SIZE_SR80_SR93		(0x93 - 0x80 + 1)
#define SIZE_SRA0_SRAF		(0xAF - 0xA0 + 1)
#define SIZE_GR00_GR08		(0x08 - 0x00 + 1)
#define SIZE_AR00_AR14		(0x14 - 0x00 + 1)
#define SIZE_CR00_CR18		(0x18 - 0x00 + 1)
#define SIZE_CR30_CR4D		(0x4D - 0x30 + 1)
#define SIZE_CR90_CRA7		(0xA7 - 0x90 + 1)

struct smi_mode_regs {
	int mode;
	u8 reg_MISC;
	u8 reg_SR00_SR04[SIZE_SR00_SR04];	/* SEQ00--04 (SEQ) */
	u8 reg_SR10_SR24[SIZE_SR10_SR24];	/* SCR10--1F, PDR20--24 (SYS),(PWR) */
	u8 reg_SR30_SR75[SIZE_SR30_SR75];	/* FPR30--5A, MCR60--62, CCR65--6F  GPR70--75 (LCD),(MEM),(CLK),(GP) */
	u8 reg_SR80_SR93[SIZE_SR80_SR93];	/* PHR80-81, POP82--86, HCR88-8D, POP90--93 (CURS),(ICON),(CURS),(ICON) */
	u8 reg_SRA0_SRAF[SIZE_SRA0_SRAF];	/* FPRA0--AF (LCD) */
	u8 reg_GR00_GR08[SIZE_GR00_GR08];	/* GR00--08 (GC) */
	u8 reg_AR00_AR14[SIZE_AR00_AR14];	/* ATR00--14 (ATTR) */
	u8 reg_CR00_CR18[SIZE_CR00_CR18];	/* CRT00--18 (CRTC) */
	u8 reg_CR30_CR4D[SIZE_CR30_CR4D];	/* CRT30--3F, SVR40--4D (ECRTC),(SHADOW) */
	u8 reg_CR90_CRA7[SIZE_CR90_CRA7];	/* CRT90--9B,9E,9F,A0--A5,A6,A7, (DDA),(EC2),(EC1)(VCLUT),(VC),(HC) */
};

typedef struct {
	unsigned char red, green, blue, transp;
} smi_cfb8_cmap_t;

struct smifb_info;
struct smifb_info {
	struct fb_info info;	/* kernel framebuffer info */
	const char *drvr_name;	/* Silicon Motion hardware board type */
	struct pci_dev *pd;	/* descripbe the PCI device */
	unsigned long base_phys;	/* physical base address                  */

	/* PCI base physical addresses */
	unsigned long fb_base_phys;	/* physical Frame Buffer base address                  */
	unsigned long dpr_base_phys;	/* physical Drawing Processor base address             */
	unsigned long vpr_base_phys;	/* physical Video Processor base address               */
	unsigned long cpr_base_phys;	/* physical Capture Processor base address             */
	unsigned long mmio_base_phys;	/* physical MMIO spase (VGA + SMI regs ?) base address */
	unsigned long dpport_base_phys;	/* physical Drawing Processor Data Port base address   */
	int dpport_size;	/* size of Drawin Processor Data Port memory space     */

	/* PCI base virtual addresses */
	caddr_t base;		/* address of base */
	caddr_t fb_base;	/* address of frame buffer base */
	caddr_t dpr;		/* Drawing Processor Registers  */
	caddr_t vpr;		/* Video Processor Registers    */
	caddr_t cpr;		/* Capture Processor Registers  */
	caddr_t mmio;		/* Memory Mapped I/O Port       */
	caddr_t dpport;		/* Drawing Processor Data       */

	int fbsize;		/* Frame-Buffer memory size */
};

#endif				/* __SMIFB_H__ */
