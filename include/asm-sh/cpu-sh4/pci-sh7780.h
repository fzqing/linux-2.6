/*
 *	Low-Level PCI Support for SH7780 targets
 *
 *  Copyright (C) 2004  Takashi Yoshii, Takashi Kusuda
 *
 *  May be copied or modified under the terms of the GNU General Public
 *  License.  See linux/COPYING for more information.
 *
 */

#ifndef ASM_SH_CPU_SH4__PCI_SH7780_H
#define ASM_SH_CPU_SH4__PCI_SH7780_H

#include <linux/pci.h>

/* set debug level 4=verbose...1=terse */
#undef DEBUG_PCI

#ifdef DEBUG_PCI
#define PCIDBG(n, x...) { if(DEBUG_PCI>=n) printk(x); }
#else
#define PCIDBG(n, x...)
#endif

/* startup values */
#define PCI_PROBE_BIOS		1
#define PCI_PROBE_CONF1		2
#define PCI_PROBE_CONF2		4
#define PCI_NO_SORT		0x100
#define PCI_BIOS_SORT		0x200
#define PCI_NO_CHECKS		0x400
#define PCI_ASSIGN_ROMS		0x1000
#define PCI_BIOS_IRQ_SCAN	0x2000

/* Platform Specific Values */
#define SH7780_VENDOR_ID	0x1912
#define SH7780_DEVICE_ID	0x0002
#define SH7781_DEVICE_ID	0x0001

/* control register */
#define SH7780_PCI_ECR		0xFE000008

/* SH7780 Specific Values */
#define SH7780_PCI_CONFIG_BASE	0xFD000000  /* Config space base addr */
#define SH7780_PCI_CONFIG_SIZE	0x01000000  /* Config space size */
#define SH7780_PCI_MEMORY_BASE	0xFD000000  /* Memory space base addr */
#define SH7780_PCI_MEM_SIZE	0x01000000  /* Size of Memory window */
#define SH7780_PCI_IO_BASE	0xFE200000  /* IO space base address */
#define SH7780_PCI_IO_SIZE	0x00200000  /* Size of IO window */

#define SH7780_PCIREG_BASE	0xFE040000  /* PCI regs base address */
#define PCI_REG(n)		(SH7780_PCIREG_BASE + n)

#define SH7780_PCI_VID		0x000
#define SH7780_PCI_DID		0x002
#define SH7780_PCI_CMD		0x004
#define SH7780_PCI_STATUS	0x006
#define SH7780_PCI_RID		0x008
#define SH7780_PCI_PIF		0x009
#define SH7780_PCI_SUB		0x00a
#define SH7780_PCI_BCC		0x00b
#define SH7780_PCI_CLS		0x00c
#define SH7780_PCI_LTM		0x00d
#define SH7780_PCI_HDR		0x00e
#define SH7780_PCI_BIST		0x00f
#define SH7780_PCI_IBAR		0x010
#define SH7780_PCI_MBAR0	0x014
#define SH7780_PCI_MBAR1	0x018
#define SH7780_PCI_SVID		0x02c
#define SH7780_PCI_SID		0x02e
#define SH7780_PCI_CP		0x034
#define SH7780_PCI_INTLINE	0x03c
#define SH7780_PCI_INTPIN	0x03d
#define SH7780_PCI_MINGNT	0x03e
#define SH7780_PCI_MAXLAT	0x03f
#define SH7780_PCI_CID		0x040
#define SH7780_PCI_NIP		0x041
#define SH7780_PCI_PMC		0x042
#define SH7780_PCI_PMCSR	0x044
#define SH7780_PCI_PMCSR_BSE	0x046
#define SH7780_PCI_PCDD		0x047


/* SH7780 Internal PCI Registers */
#define SH7780_PCI_CR		0x100		/* PCI Control Register */
  #define SH7780_PCICR_PREFIX	0xA5000000	/* CR prefix for write */
  #define SH7780_PCICR_TRSB	0x00000200	/* Target Read Single */
  #define SH7780_PCICR_BSWP	0x00000100	/* Target Byte Swap */
  #define SH7780_PCICR_PLUP	0x00000080	/* Enable PCI Pullup */
  #define SH7780_PCICR_ARBM	0x00000040	/* PCI Arbitration Mode */
  #define SH7780_PCICR_MD	0x00000030	/* MD9 and MD10 status */
  #define SH7780_PCICR_SERR	0x00000008	/* SERR output assert */
  #define SH7780_PCICR_INTA	0x00000004	/* INTA output assert */
  #define SH7780_PCICR_PRST	0x00000002	/* PCI Reset Assert */
  #define SH7780_PCICR_CFIN	0x00000001	/* Central Fun. Init Done */

#define SH7780_PCI_LSR0		0x104	/* PCI Local Space Register0 */
#define SH7780_PCI_LSR1		0x108	/* PCI Local Space Register1 */
#define SH7780_PCI_LAR0		0x10C	/* PCI Local Address Register1 */
#define SH7780_PCI_LAR1		0x110	/* PCI Local Address Register1 */
#define SH7780_PCI_IR		0x114	/* PCI Interrupt Register */
#define SH7780_PCI_IMR		0x118	/* PCI Interrupt Mask Register */
#define SH7780_PCI_AIR		0x11C	/* Error Address Register */
#define SH7780_PCI_CIR		0x120	/* Error Command/Data Register */
#define SH7780_PCI_AINT		0x130	/* Arbiter Interrupt Register */
#define SH7780_PCI_AINTM	0x134	/* Arbiter Int. Mask Register */
#define SH7780_PCIBMIR		0x138	/* Error Bus Master Register */
#define SH7780_PCI_PAR		0x1C0	/* PIO Address Register */
#define SH7780_PCI_PINT		0x1CC	/* Power Mgmnt Int. Register */
#define SH7780_PCI_PINTM	0x1D0	/* Power Mgmnt Mask Register */
#define SH7780_PCI_MBR0		0x1E0
#define SH7780_PCI_MBMR0	0x1E4
#define SH7780_PCI_MBR1		0x1E8
#define SH7780_PCI_MBMR1	0x1EC
#define SH7780_PCI_MBR2		0x1F0
#define SH7780_PCI_MBMR2	0x1F4
#define SH7780_PCI_IOBR		0x1F8
#define SH7780_PCI_IOBMR	0x1FC
#define SH7780_PCI_CSCR0	0x210
#define SH7780_PCI_CSCR1	0x214
#define SH7780_PCI_CSAR0	0x218
#define SH7780_PCI_CSAR1	0x21c
#define SH7780_PCI_PDR		0x220	/* Port IO Data Register */


extern int pcibios_init_platform(void);
extern int pcibios_map_platform_irq(u8 slot, u8 pin);

#endif /* ASM_SH_CPU_SH4__PCI_SH7780_H */

