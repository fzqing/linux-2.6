#ifndef __ASM_SH_SE7780_IRQ_H
#define __ASM_SH_SE7780_IRQ_H

/*
 * linux/include/asm-sh/se7780/irq.h
 *
 * Copyright (C) 2004  Takashi Kusuda
 *
 *  Hitachi SolutionEngine support
 *
 *  Modified for 7780 Solution Engine
 */

#include <asm/mach/map.h>
#include <asm/irq.h>

/* interrupt numer */
#define IRQ_SM501		SH7780_IRQ7_INTNUM	/* 0 */
#define IRQ_PCCPW		SH7780_IRQ7_INTNUM	/* 0 */
#define IRQ_EXTINT1		SH7780_IRQ0_INTNUM	/* 2 */
#define IRQ_EXTINT2		SH7780_IRQ1_INTNUM	/* 4 */
#define IRQ_EXTINT3		SH7780_IRQ2_INTNUM	/* 6 */
#define IRQ_SMC91CX		SH7780_IRQ3_INTNUM	/* 8 */
#define IRQ_EXTINT4		SH7780_IRQ4_INTNUM	/* 10 */
#define IRQ_PCC0		SH7780_IRQ5_INTNUM	/* 12 */
#define IRQ_PCC2		SH7780_IRQ6_INTNUM	/* 14 */

/* interrupt pin */
#define IRQPIN_EXTINT1		0 /* IRQ0 pin */
#define IRQPIN_EXTINT2		1 /* IRQ1 pin */
#define IRQPIN_EXTINT3		2 /* IRQ2 pin */
#define IRQPIN_SMC91CX		3 /* IRQ3 pin */
#define IRQPIN_EXTINT4		4 /* IRQ4 pin */
#define IRQPIN_PCC0		5 /* IRQ5 pin */
#define IRQPIN_PCC2		6 /* IRQ6 pin */
#define IRQPIN_SM501		7 /* IRQ7 pin */
#define IRQPIN_PCCPW		7 /* IRQ7 pin */

/* FPGA INTSEL position */
/* INTSEL1 */
#define IRQPOS_SMC91CX		(0 * 4)
#define IRQPOS_SM501		(1 * 4)
#define IRQPOS_PCC0		(2 * 4)
#define IRQPOS_PCC2		(3 * 4)
/* INTSEL2 */
#define IRQPOS_EXTINT1		(0 * 4)
#define IRQPOS_EXTINT2		(1 * 4)
#define IRQPOS_EXTINT3		(2 * 4)
#define IRQPOS_EXTINT4		(3 * 4)
/* INTSEL3 */
#define IRQPOS_PCCPW		(0 * 4)

/* IDE interrupt */
#define IRQ_IDE0		67 /* iVDR */
#define IRQ_IDE1		65 /* PCI slot */

/* SMC interrupt */
#define SMC_IRQ			IRQ_SMC91CX /* 8 */

#endif  /* __ASM_SH_SE7780_IRQ_H */
