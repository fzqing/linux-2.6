/*
 * arch/ppc/syslib/qe_ic.h
 *
 * QE IC private definitions and structure.
 *
 * Author: Shlomi Gridish <gridish@freescale.com>
 *
 * Copyright 2005 Freescale Semiconductor, Inc
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#ifndef __QE_IC_H__
#define __QE_IC_H__

typedef struct qe_ic_map {
	volatile u32 qicr;
	volatile u32 qivec;
	volatile u32 qripnr;
	volatile u32 qipnr;
	volatile u32 qipxcc;
	volatile u32 qipycc;
	volatile u32 qipwcc;
	volatile u32 qipzcc;
	volatile u32 qimr;
	volatile u32 qrimr;
	volatile u32 qicnr;
	volatile u8 res0[0x4];
	volatile u32 qiprta;
	volatile u32 qiprtb;
	volatile u8 res1[0x4];
	volatile u32 qricr;
	volatile u8 res2[0x20];
	volatile u32 qhivec;
	volatile u8 res3[0x1C];
} __attribute__ ((packed)) qe_ic_map_t;

#define QE_IC_SIZE sizeof(struct qe_ic_map)

/* Interrupt priority registers */
#define QIPCC_SHIFT_PRI0        29
#define QIPCC_SHIFT_PRI1        26
#define QIPCC_SHIFT_PRI2        23
#define QIPCC_SHIFT_PRI3        20
#define QIPCC_SHIFT_PRI4        13
#define QIPCC_SHIFT_PRI5        10
#define QIPCC_SHIFT_PRI6        7
#define QIPCC_SHIFT_PRI7        4

/* QICR priority modes */
#define QICR_GWCC               0x00040000
#define QICR_GXCC               0x00020000
#define QICR_GYCC               0x00010000
#define QICR_GZCC               0x00080000
#define QICR_GRTA               0x00200000
#define QICR_GRTB               0x00400000
#define QICR_HPIT_SHIFT         8

/* QICNR */
#define QICNR_WCC1T_SHIFT       20
#define QICNR_ZCC1T_SHIFT       28
#define QICNR_YCC1T_SHIFT       12
#define QICNR_XCC1T_SHIFT       4

/* QRICR */
#define QRICR_RTA1T_SHIFT       20
#define QRICR_RTB1T_SHIFT       28

struct qe_ic_private {
	struct qe_ic_map *regs;
	unsigned int irq_offset;
} qe_ic_private_t;

extern struct hw_interrupt_type qe_ic;
extern int qe_ic_get_irq(struct pt_regs *regs);

#endif				/* __QE_IC_H__ */
