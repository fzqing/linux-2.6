/*
 * include/asm-ppc/mpc83xx.h
 *
 * MPC83xx definitions
 *
 * Maintainer: Kumar Gala <kumar.gala@freescale.com>
 *
 * Copyright 2005 Freescale Semiconductor, Inc
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifdef __KERNEL__
#ifndef __ASM_MPC83xx_H__
#define __ASM_MPC83xx_H__

#include <linux/config.h>
#include <asm/mmu.h>
#ifdef CONFIG_QE
#include <asm/qe.h>
#endif /* CONFIG_QE */

#ifdef CONFIG_83xx

#ifdef CONFIG_MPC834x_SYS
#include <platforms/83xx/mpc834x_sys.h>
#endif
#ifdef CONFIG_MPC834x_ITX
#include <platforms/83xx/mpc834x_itx.h>
#endif
#ifdef CONFIG_MPC8360E_PB
#include <platforms/83xx/mpc8360e_pb.h>
#endif
#ifdef CONFIG_MPC832XE_MDS
#include <platforms/83xx/mpc832xe_mds.h>
#endif

#define _IO_BASE        isa_io_base
#define _ISA_MEM_BASE   isa_mem_base
#ifdef CONFIG_PCI
#define PCI_DRAM_OFFSET pci_dram_offset
#else
#define PCI_DRAM_OFFSET 0
#endif

/*
 * The "residual" board information structure the boot loader passes
 * into the kernel.
 */
extern unsigned char __res[];

/* Internal IRQs on MPC83xx OpenPIC */
/* Not all of these exist on all MPC83xx implementations */

#ifndef MPC83xx_IPIC_IRQ_OFFSET
#define MPC83xx_IPIC_IRQ_OFFSET	0
#endif

#define NR_IPIC_INTS 128

#define MPC83xx_IRQ_UART1	( 9 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_UART2	(10 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_SEC2	(11 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_IIC1	(14 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_IIC2	(15 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_SPI		(16 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_EXT1	(17 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_EXT2	(18 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_EXT3	(19 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_EXT4	(20 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_EXT5	(21 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_EXT6	(22 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_EXT7	(23 + MPC83xx_IPIC_IRQ_OFFSET)
#ifdef CONFIG_QE
#define MPC83xx_IRQ_QE_HIGH             (32 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_QE_LOW              (33 + MPC83xx_IPIC_IRQ_OFFSET)
#endif /* CONFIG_QE */
#define MPC83xx_IRQ_TSEC1_TX	(32 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_TSEC1_RX	(33 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_TSEC1_ERROR	(34 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_TSEC2_TX	(35 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_TSEC2_RX	(36 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_TSEC2_ERROR	(37 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_USB2_DR	(38 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_USB2_MPH	(39 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_EXT0	(48 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_RTC_SEC	(64 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_PIT		(65 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_PCI1	(66 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_PCI2	(67 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_RTC_ALR	(68 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_MU		(69 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_SBA		(70 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_DMA		(71 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_GTM4	(72 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_GTM8	(73 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_GPIO1	(74 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_GPIO2	(75 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_DDR		(76 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_LBC		(77 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_GTM2	(78 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_GTM6	(79 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_PMC		(80 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_GTM3	(84 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_GTM7	(85 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_GTM1	(90 + MPC83xx_IPIC_IRQ_OFFSET)
#define MPC83xx_IRQ_GTM5	(91 + MPC83xx_IPIC_IRQ_OFFSET)

#define MPC83xx_PCI1_LOWER_IO	0x00000000
#define MPC83xx_PCI1_UPPER_IO	0x00ffffff
#define MPC83xx_PCI1_LOWER_MEM	0x80000000
#define MPC83xx_PCI1_UPPER_MEM	0x9fffffff
#define MPC83xx_PCI1_IO_BASE	0xe2000000
#define MPC83xx_PCI1_MEM_OFFSET	0x00000000
#define MPC83xx_PCI1_IO_SIZE	0x01000000

#define MPC83xx_PCI2_LOWER_IO	0x00000000
#define MPC83xx_PCI2_UPPER_IO	0x00ffffff
#define MPC83xx_PCI2_LOWER_MEM	0xa0000000
#define MPC83xx_PCI2_UPPER_MEM	0xbfffffff
#define MPC83xx_PCI2_IO_BASE	0xe3000000
#define MPC83xx_PCI2_MEM_OFFSET	0x00000000
#define MPC83xx_PCI2_IO_SIZE	0x01000000

/* Internal IRQs on MPC83xx QE IC */
/* Not all of these exist on all MPC83xx QE implementations */

#ifndef MPC83xx_QE_IRQ_OFFSET
#define MPC83xx_QE_IRQ_OFFSET	NR_IPIC_INTS
#endif /* MPC83xx_QE_IRQ_OFFSET */

#define MPC83xx_NR_QE_IC_INTS 64

#define MPC83xx_QE_IRQ_SPI2		(1 + MPC83xx_QE_IRQ_OFFSET)
#define MPC83xx_QE_IRQ_SPI1		(2 + MPC83xx_QE_IRQ_OFFSET)
#define MPC83xx_QE_IRQ_USB		(11 + MPC83xx_QE_IRQ_OFFSET)
#define MPC83xx_QE_IRQ_UCC1		(32 + MPC83xx_QE_IRQ_OFFSET)
#define MPC83xx_QE_IRQ_UCC2		(33 + MPC83xx_QE_IRQ_OFFSET)
#define MPC83xx_QE_IRQ_UCC3		(34 + MPC83xx_QE_IRQ_OFFSET)
#define MPC83xx_QE_IRQ_UCC4		(35 + MPC83xx_QE_IRQ_OFFSET)
#define MPC83xx_QE_IRQ_UCC5		(40 + MPC83xx_QE_IRQ_OFFSET)
#define MPC83xx_QE_IRQ_UCC6		(41 + MPC83xx_QE_IRQ_OFFSET)
#define MPC83xx_QE_IRQ_UCC7		(42 + MPC83xx_QE_IRQ_OFFSET)
#define MPC83xx_QE_IRQ_UCC8		(43 + MPC83xx_QE_IRQ_OFFSET)

#ifdef CONFIG_QE
#define MPC83xx_IMMRBAR_SIZE    (2*1024*1024)
#else
#define MPC83xx_IMMRBAR_SIZE    (1024*1024)
#endif /* CONFIG_QE */

#define MPC83xx_CCSRBAR_SIZE	(1024*1024)

/* system priority and configuration register */
#define MPC83xx_SPCR            0x00000110
#define MPC83xx_SPCR_TBEN       0x00400000

/* reset protection register */
#define MPC83xx_RPR		0x00000918
#define MPC83xx_RPR_RSTE	0x52535445      /* "RSTE" in ASCII */

/* reset control register*/
#define MPC83xx_RCR		0x0000091c
#define MPC83xx_RCR_SWHR	0x00000002      /* sw hard reset */
#define MPC83xx_RCR_SWSR	0x00000001      /* sw soft reset */

#define MPC83XX_SCCR_OFFS          0xA08
#define MPC83XX_SCCR_USB_MPHCM_11  0x00c00000
#define MPC83XX_SCCR_USB_MPHCM_01  0x00400000
#define MPC83XX_SCCR_USB_MPHCM_10  0x00800000
#define MPC83XX_SCCR_USB_DRCM_11   0x00300000
#define MPC83XX_SCCR_USB_DRCM_01   0x00100000
#define MPC83XX_SCCR_USB_DRCM_10   0x00200000

/* system i/o configuration register low */
#define MPC83XX_SICRL_OFFS         0x114
#define MPC83XX_SICRL_USB1         0x40000000
#define MPC83XX_SICRL_USB0         0x20000000

/* system i/o configuration register high */
#define MPC83XX_SICRH_OFFS         0x118
#define MPC83XX_SICRH_USB_UTMI     0x00020000

/* Let modules/drivers get at immrbar (physical) */
extern phys_addr_t immrbar;

enum ppc_sys_devices {
	MPC83xx_TSEC1,
	MPC83xx_TSEC2,
	MPC83xx_IIC1,
	MPC83xx_IIC2,
	MPC83xx_DUART,
	MPC83xx_SEC2,
	MPC83xx_USB2_DR,
	MPC83xx_USB2_MPH,
	MPC83xx_MDIO,
	MPC83xx_MDIO_TSEC1,
	MPC83xx_CFIDE,
	MPC83xx_QE_UCC1,
	MPC83xx_QE_UCC2,
	MPC83xx_QE_UCC3,
	MPC83xx_QE_UCC4,
	MPC83xx_QE_UCC5,
	MPC83xx_QE_UCC6,
	MPC83xx_QE_UCC7,
	MPC83xx_QE_UCC8,
	MPC83xx_QE_SPI1,
	MPC83xx_QE_SPI2,
	MPC83xx_QE_USB,
	NUM_PPC_SYS_DEVS,
};

static inline unsigned long immrbar_virt_to_phys(volatile void * address)
{
    if ( ((uint)address >= VIRT_IMMRBAR) &&
         ((uint)address < (VIRT_IMMRBAR + MPC83xx_IMMRBAR_SIZE)) )
        return (unsigned long)(address - VIRT_IMMRBAR + immrbar);
    return (unsigned long)address;
}

static inline void * immrbar_phys_to_virt(unsigned long address)
{
    if ( (address >= immrbar) &&
         (address < (immrbar + MPC83xx_IMMRBAR_SIZE)) )
        return (void *)(address - immrbar + VIRT_IMMRBAR);
    return (void *)address;
}

#endif /* CONFIG_83xx */
#endif /* __ASM_MPC83xx_H__ */
#endif /* __KERNEL__ */
