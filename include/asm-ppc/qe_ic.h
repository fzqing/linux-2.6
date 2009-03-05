/*
 * include/asm-ppc/qe_ic.h
 *
 * QE IC external definitions and structure.
 *
 * Author: Shlomi Gridih <gridish@freescale.com>
 *
 * Copyright 2005 Freescale Semiconductor, Inc
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#ifdef __KERNEL__
#ifndef __ASM_QE_IC_H__
#define __ASM_QE_IC_H__

#include <linux/irq.h>

#define _IO_BASE        isa_io_base
#define _ISA_MEM_BASE   isa_mem_base
#ifdef CONFIG_PCI
#define PCI_DRAM_OFFSET pci_dram_offset
#else
#define PCI_DRAM_OFFSET 0
#endif

#define NUM_OF_QE_IC_GROUPS    6

/* Flags when we init the QE IC */
#define QE_IC_SPREADMODE_GRP_W                   0x00000001
#define QE_IC_SPREADMODE_GRP_X                   0x00000002
#define QE_IC_SPREADMODE_GRP_Y                   0x00000004
#define QE_IC_SPREADMODE_GRP_Z                   0x00000008
#define QE_IC_SPREADMODE_GRP_RISCA               0x00000010
#define QE_IC_SPREADMODE_GRP_RISCB               0x00000020

#define QE_IC_LOW_SIGNAL                         0x00000100
#define QE_IC_HIGH_SIGNAL                        0x00000200

#define QE_IC_GRP_W_PRI0_DEST_SIGNAL_HIGH        0x00001000
#define QE_IC_GRP_W_PRI1_DEST_SIGNAL_HIGH        0x00002000
#define QE_IC_GRP_X_PRI0_DEST_SIGNAL_HIGH        0x00004000
#define QE_IC_GRP_X_PRI1_DEST_SIGNAL_HIGH        0x00008000
#define QE_IC_GRP_Y_PRI0_DEST_SIGNAL_HIGH        0x00010000
#define QE_IC_GRP_Y_PRI1_DEST_SIGNAL_HIGH        0x00020000
#define QE_IC_GRP_Z_PRI0_DEST_SIGNAL_HIGH        0x00040000
#define QE_IC_GRP_Z_PRI1_DEST_SIGNAL_HIGH        0x00080000
#define QE_IC_GRP_RISCA_PRI0_DEST_SIGNAL_HIGH    0x00100000
#define QE_IC_GRP_RISCA_PRI1_DEST_SIGNAL_HIGH    0x00200000
#define QE_IC_GRP_RISCB_PRI0_DEST_SIGNAL_HIGH    0x00400000
#define QE_IC_GRP_RISCB_PRI1_DEST_SIGNAL_HIGH    0x00800000
#define QE_IC_GRP_W_DEST_SIGNAL_SHIFT            (12)

/*
 * QE interrupt sources groups
 */
enum qe_ic_grp_id {
	QE_IC_GRP_W = 0,	/* QE interrupt controller group W      */
	QE_IC_GRP_X,		/* QE interrupt controller group X      */
	QE_IC_GRP_Y,		/* QE interrupt controller group Y      */
	QE_IC_GRP_Z,		/* QE interrupt controller group Z      */
	QE_IC_GRP_RISCA,	/* QE interrupt controller RISC group A */
	QE_IC_GRP_RISCB		/* QE interrupt controller RISC group B */
};

/*
 * QE interrupt controller internal structure
 */
struct qe_ic_info {
	u32 mask;		/* locaion of this source at the QIMR register.    */
	int qimr;		/* TRUE is this source is mappd to QIMR,           */
	/* otherwise - QRIMR (risc).                       */
	u8 pri_code;		/* for grouped interrupts sources - the interrupt  */
	/* code as appears at the group priority register. */
};

/*********************************************/
/******   QE IC API routines             *****/
/*********************************************/
int qe_ic_init(phys_addr_t phys_addr,
	       unsigned int flags, unsigned int irq_offset);
void qe_ic_free(void);
void qe_ic_enable_irq(unsigned int qeIntrSrc);
void qe_ic_disable_irq(unsigned int qeIntrSrc);
void qe_ic_disable_irq_and_ack(unsigned int irq);
void qe_ic_end_irq(unsigned int irq);

/* qe_ic_modify_highest_priority
 * Optional, used to change default. This routine defines a single interrupt
 * source to be highest priority. It may be called at any stage, thus enabling
 * dynamic change of the highest priority interrupt.
 * In default, Highest priority is XCC1 highest priority interrupt source.
 *
 * @Param    irq (In) -  Interrupt source Id.
 */
void qe_ic_modify_highest_priority(unsigned int irq);

/* qe_ic_modify_priority
 * Optional, used to change default. May be called at run time to manipulate
 * priorities. This routine is called to reorganize a specific group.
 *
 * @Param    qeIcGroupId (In)         - One of:
 *                                          e_QE_IC_GRP_W
 *                                          e_QE_IC_GRP_X
 *                                          e_QE_IC_GRP_Y
 *                                          e_QE_IC_GRP_Z
 *                                          e_QE_IC_GRP_RISCA
 *                                          e_QE_IC_GRP_RISCB
 * @Param    pri0, pr1,..., pri7 (In) - A list of interrupt sources (of type
 *                                      unsigned int) in order of priority. The
 *                                      list must include all and only sources
 *                                      of the specified group.
 */
void qe_ic_modify_priority(enum qe_ic_grp_id qeIcGroupId,
			   unsigned int pri0,
			   unsigned int pri1,
			   unsigned int pri2,
			   unsigned int pri3,
			   unsigned int pri4,
			   unsigned int pri5,
			   unsigned int pri6, unsigned int pri7);

#endif				/* __ASM_QE_IC_H__ */
#endif				/* __KERNEL__ */
