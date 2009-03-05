/*
 * (C) Copyright 2005 Tundra Semiconductor Corp.
 * Alex Bounine, <alexandreb@tundra.com).
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 *  arch/ppc/syslib/tsi108_pic.h - Tsi108 Interrupt Controller definitions
 */

#ifndef _LINUX_TSI108_PIC_H
#define _LINUX_TSI108_PIC_H

#include <asm/tsi108_irq.h>

#ifdef __KERNEL__

/*
 *  Tsi108 PIC supports up to 24 interrupt sources and up to 4 processors
 */

#define TSI108PIC_MAX_SOURCES		24
#define TSI108PIC_MAX_PROCESSORS	4

#define TSI108PIC_NUM_TIMERS	4
#define TSI108PIC_NUM_DBELLS	4
#define TSI108PIC_NUM_PROC   	4
#define TSI108PIC_NUM_PRI	16
#define TSI108PIC_NUM_VECTORS	256

/*
 * Tsi108 PIC Register offsets within block.
 */

/* Registers controlling sources */
#define TSI108_INT_FRR		(0x000)
#define TSI108_INT_GCR		(0x004)
#define TSI108_INT_SVR		(0x010)
#define TSI108_INT_GTVPR(n)	(0x38 + 0x10*(n))
#define TSI108_INT_GTDR(n)	(0x3C + 0x10*(n))
#define TSI108_INT_IVPR(n)	(0x100 + 0x8*(n))
#define TSI108_INT_IDR(n)	(0x104 + 0x8*(n))
#define TSI108_INT_DVPR(n)	(0x204 + 0xC*(n))
#define TSI108_INT_DDR(n)	(0x208 + 0xC*(n))
#define TSI108_INT_MBVPR(n)	(0x284 + 0x10*(n))
#define TSI108_INT_MBDR(n)	(0x288 + 0x10*(n))

/* Registers controlling destinations */
#define TSI108_INT_TASKP(n)	(0x300 + 0x40*(n))
#define TSI108_INT_VECTOR(n)	(0x304 + 0x40*(n))
#define TSI108_INT_EOI(n)	(0x308 + 0x40*(n))
#define TSI108_INT_CSR(n)	(0x30C + 0x40*(n))

/*
 * Generic definitions common for different types of interrupt
 * sources.
 */

#define TSI108PIC_MASK				(0x80000000)
#define TSI108PIC_ACTIVITY			(0x40000000)
#define TSI108PIC_PRIORITY_MASK			(0x000f0000)
#define TSI108PIC_VECTOR_MASK			(0x000000ff)

/**********************************************************
 * Register Bit Masks definitions for every register
 */

/* TSI108PIC_INT_FRR : Register Bits Masks Definitions */
#define TSI108PIC_INT_FRR_VID			(0x000000ff)
#define TSI108PIC_INT_FRR_NCPU			(0x00001f00)
#define TSI108PIC_INT_FRR_NITM			(0x0000e000)
#define TSI108PIC_INT_FRR_NIRQ			(0x07ff0000)
#define TSI108PIC_INT_FRR_NIDOOR		(0xe0000000)
#define TSI108PIC_INT_FRR_RESERVED		(0x18000000)

/* TSI108PIC_INT_GCR : Register Bits Masks Definitions */
#define TSI108PIC_INT_GCR_R			(0x80000000)
#define TSI108PIC_INT_GCR_RESERVED		(0x7fffffff)

/* TSI108PIC_INT_ICR : Register Bits Masks Definitions */
#define TSI108PIC_INT_ICR_R			(0x0000000f)
#define TSI108PIC_INT_ICR_RESERVED		(0xfffffff0)

/* TSI108PIC_INT_MVI : Register Bits Masks Definitions */
#define TSI108PIC_INT_MVI_VID			(0x000000ff)
#define TSI108PIC_INT_MVI_DID			(0x0000ff00)
#define TSI108PIC_INT_MVI_STEP			(0x00ff0000)
#define TSI108PIC_INT_MVI_RESERVED		(0xff000000)

/* TSI108PIC_INT_SVR : Register Bits Masks Definitions */
#define TSI108PIC_INT_SVR_VECTOR		(0x000000ff)
#define TSI108PIC_INT_SVR_RESERVED		(0xffffff00)

/* TSI108PIC_INT_TFRR : Register Bits Masks Definitions */
#define TSI108PIC_INT_TFRR_TIME_FREQ		(0xffffffff)

/* TSI108PIC_INT_SOFT_SET : Register Bits Masks Definitions */
#define TSI108PIC_INT_SOFT_SET_S		(0x00ffffff)
#define TSI108PIC_INT_SOFT_SET_RESERVED		(0xff000000)

/* TSI108PIC_INT_SOFT_ENABLE : Register Bits Masks Definitions */
#define TSI108PIC_INT_SOFT_ENABLE_EN		(0x00ffffff)
#define TSI108PIC_INT_SOFT_ENABLE_RESERVED	(0xff000000)

/* TSI108PIC_INT_GTCCR(X) : Register Bits Masks Definitions */
#define TSI108PIC_INT_GTCCR_COUNT		(0x7fffffff)
#define TSI108PIC_INT_GTCCR_T			(0x80000000)

/* TSI108PIC_INT_GTBCR(X) : Register Bits Masks Definitions */
#define TSI108PIC_INT_GTBCR_B_COUNT		(0x7fffffff)
#define TSI108PIC_INT_GTBCR_CI			(0x80000000)

/* TSI108PIC_INT_GTVPR(X) : Register Bits Masks Definitions */
#define TSI108PIC_INT_GTVPR_VECTOR		(0x000000ff)
#define TSI108PIC_INT_GTVPR_PRIORITY		(0x000f0000)
#define TSI108PIC_INT_GTVPR_PRESCALE		(0x00f00000)
#define TSI108PIC_INT_GTVPR_A			(0x40000000)
#define TSI108PIC_INT_GTVPR_M			(0x80000000)
#define TSI108PIC_INT_GTVPR_RESERVED		(0x3f00ff00)

/* TSI108PIC_INT_GTDR(X) : Register Bits Masks Definitions */
#define TSI108PIC_INT_GTDR_SEL_OUT		(0x0000000f)
#define TSI108PIC_INT_GTDR_RESERVED		(0xfffffff0)

/* TSI108PIC_INT_IVPR(X) : Register Bits Masks Definitions */
#define TSI108PIC_INT_IVPR_VECTOR		(0x000000ff)
#define TSI108PIC_INT_IVPR_PRIORITY		(0x000f0000)

#define TSI108PIC_INT_IVPR_P			(0x01000000)
#define TSI108PIC_INT_IVPR_P_LOW		(0 << 24)
#define TSI108PIC_INT_IVPR_P_HIGH		(1 << 24)

#define TSI108PIC_INT_IVPR_S			(0x02000000)
#define TSI108PIC_INT_IVPR_S_EDGE		(0 << 25)
#define TSI108PIC_INT_IVPR_S_LEVEL		(1 << 25)

#define TSI108PIC_INT_IVPR_MODE			(0x20000000)
#define TSI108PIC_INT_IVPR_A			(0x40000000)
#define TSI108PIC_INT_IVPR_M			(0x80000000)
#define TSI108PIC_INT_IVPR_RESERVED		(0x1cf0ff00)

/* TSI108PIC_INT_IDR(X) : Register Bits Masks Definitions */
#define TSI108PIC_INT_IDR_SEL_OUT		(0x0000000f)
#define TSI108PIC_INT_IDR_RESERVED		(0xfffffff0)

/* TSI108PIC_INT_DAR : Register Bits Masks Definitions */
#define TSI108PIC_INT_DAR_A			(0x0000000f)
#define TSI108PIC_INT_DAR_RESERVED		(0xfffffff0)

/* TSI108PIC_INT_DVPR(X) : Register Bits Masks Definitions */
#define TSI108PIC_INT_DVPR_VECTOR		(0x000000ff)
#define TSI108PIC_INT_DVPR_PRIORITY		(0x000f0000)
#define TSI108PIC_INT_DVPR_A			(0x40000000)
#define TSI108PIC_INT_DVPR_M			(0x80000000)
#define TSI108PIC_INT_DVPR_RESERVED		(0x3ff0ff00)

/* TSI108PIC_INT_DDR(X) : Register Bits Masks Definitions */
#define TSI108PIC_INT_DDR_SEL_OUT		(0x0000000f)
#define TSI108PIC_INT_DDR_RESERVED		(0xfffffff0)

/* TSI108PIC_INT_DMR(X) : Register Bits Masks Definitions */
#define TSI108PIC_INT_DMR_M			(0xffffffff)

/* TSI108PIC_INT_MBR(X) : Register Bits Masks Definitions */
#define TSI108PIC_INT_MBR_M			(0xffffffff)

/* TSI108PIC_INT_MBVPR(X) : Register Bits Masks Definitions */
#define TSI108PIC_INT_MBVPR_VECTOR		(0x000000ff)
#define TSI108PIC_INT_MBVPR_PRIORITY		(0x000f0000)
#define TSI108PIC_INT_MBVPR_A			(0x40000000)
#define TSI108PIC_INT_MBVPR_M			(0x80000000)
#define TSI108PIC_INT_MBVPR_RESERVED		(0x3ff0ff00)

/* TSI108PIC_INT_MBDR(X) : Register Bits Masks Definitions */
#define TSI108PIC_INT_MBDR_SEL_OUT		(0x0000000f)
#define TSI108PIC_INT_MBDR_RESERVED		(0xfffffff0)

/* TSI108PIC_INT_TASKP(X) : Register Bits Masks Definitions */
#define TSI108PIC_INT_TASKP_TASKP		(0x0000000f)
#define TSI108PIC_INT_TASKP_RESERVED		(0xfffffff0)

/* TSI108PIC_INT_VECTOR(X) : Register Bits Masks Definitions */
#define TSI108PIC_INT_VECTOR_VECTOR		(0x000000ff)
#define TSI108PIC_INT_VECTOR_LS_VECTOR		(0xff000000)
#define TSI108PIC_INT_VECTOR_RESERVED		(0x00ffff00)

/* TSI108PIC_INT_EOI(X) : Register Bits Masks Definitions */
#define TSI108PIC_INT_EOI_EOI			(0x000000ff)
#define TSI108PIC_INT_EOI_RESERVED		(0xffffff00)

/* TSI108PIC_INT_CSR(X) : Register Bits Masks Definitions */
#define TSI108PIC_INT_CSR_RESERVED		(0xfffffffc)

#define TSI108PIC_INT_CSR_P			(1 << 0)
#define TSI108PIC_INT_CSR_P_LOW			(0 << 0)
#define TSI108PIC_INT_CSR_P_HIGH		(1 << 0)

#define TSI108PIC_INT_CSR_S			(1 << 1)
#define TSI108PIC_INT_CSR_S_EDGE		(0 << 1)
#define TSI108PIC_INT_CSR_S_LEVEL		(1 << 1)

extern void tsi108_pic_init(u_char * board_init_senses);
extern void tsi108_pic_reset(void);
extern void tsi108_pic_set_output(int dest_num, u32 sense, u32 polarity);
extern int tsi108_pic_source_cfg(int src_num, u32 sense,
				 u32 polarity, TSI108_IRQ_MODE mode);
extern int tsi108_pic_set_vector(int src_num, int vect, int prio);

extern void tsi108_pic_init_nmi_irq(u_int irq);
extern void tsi108_pic_hookup_cascade(u_int irq, char *name,
				      int (*cascade_fn) (struct pt_regs *));
extern int tsi108_pic_get_irq(struct pt_regs *regs);

#endif				/* __KERNEL__ */

#endif				/* _LINUX_TSI108_PIC_H */
