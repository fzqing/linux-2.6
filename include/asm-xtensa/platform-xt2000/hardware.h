/*
 * include/asm-xtensa/platform-xt2000/hardware.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 */

/*
 * This file contains the hardware configuration of the XT2000 board.
 */

#ifndef _XTENSA_XT2000_HARDWARE_H
#define _XTENSA_XT2000_HARDWARE_H

/* 
 * Memory configuration.
 */

#define PLATFORM_DEFAULT_MEM_START 0x00000000
#define PLATFORM_DEFAULT_MEM_SIZE 0x08000000

/*
 * Interrupt configuration.
 */

/* The XT2000 uses the V3 as a cascaded interrupt controller for the PCI bus */
#define PLATFORM_NR_IRQS 3
#define IRQ_PCI_A (XCHAL_NUM_INTERRUPTS + 0)
#define IRQ_PCI_B (XCHAL_NUM_INTERRUPTS + 1)
#define IRQ_PCI_C (XCHAL_NUM_INTERRUPTS + 2)

#endif /* _XTENSA_XT2000_HARDWARE_H */
