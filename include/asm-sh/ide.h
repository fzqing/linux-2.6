/*
 *  linux/include/asm-sh/ide.h
 *
 *  Copyright (C) 1994-1996  Linus Torvalds & authors
 */

/*
 *  This file contains the i386 architecture specific IDE code.
 *  In future, SuperH code.
 */

#ifndef __ASM_SH_IDE_H
#define __ASM_SH_IDE_H

#ifdef __KERNEL__

#include <linux/config.h>

#ifndef MAX_HWIFS
#define MAX_HWIFS	CONFIG_IDE_MAX_HWIFS
#endif

#define IDE_ARCH_OBSOLETE_DEFAULTS

/*
 * IRQ_IDEx must be defined at asm/mach/irq.h
 * (Because this interrupt numbers depend on platform.)
 */
#ifndef IRQ_IDE0
#define IRQ_IDE0       0
#endif
#ifndef IRQ_IDE1
#define IRQ_IDE1       0
#endif
#ifndef IRQ_IDE2
#define IRQ_IDE2       0
#endif
#ifndef IRQ_IDE3
#define IRQ_IDE3       0
#endif
#ifndef IRQ_IDE4
#define IRQ_IDE4       0
#endif
#ifndef IRQ_IDE5
#define IRQ_IDE5        0
#endif
static __inline__ int ide_default_irq(unsigned long base)
{
       switch (base) {
               case 0x1f0: return IRQ_IDE0;
               case 0x170: return IRQ_IDE1;
               case 0x1e8: return IRQ_IDE2;
               case 0x168: return IRQ_IDE3;
               case 0x1e0: return IRQ_IDE4;
               case 0x160: return IRQ_IDE5;
               default:
                       return 0;
       }
}

static __inline__ unsigned long ide_default_io_base(int index)
{
       switch (index) {
               case 0: return 0x1f0;
               case 1: return 0x170;
               case 2: return 0x1e8;
               case 3: return 0x168;
               case 4: return 0x1e0;
               case 5: return 0x160;
               default:
                       return 0;
       }
}

#define IDE_ARCH_OBSOLETE_INIT

#define ide_default_io_ctl(base)	(0)
#ifdef CONFIG_BLK_DEV_IDEPCI
#define ide_init_default_irq(base)     (0)
#else
#define ide_init_default_irq(base)     ide_default_irq(base)
#endif

#include <asm-generic/ide_iops.h>

#endif /* __KERNEL__ */

#endif /* __ASM_SH_IDE_H */
