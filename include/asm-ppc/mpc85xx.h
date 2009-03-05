/*
 * include/asm-ppc/mpc85xx.h
 *
 * MPC85xx definitions
 *
 * Maintainer: Kumar Gala <kumar.gala@freescale.com>
 *
 * Copyright 2004 Freescale Semiconductor, Inc
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifdef __KERNEL__
#ifndef __ASM_MPC85xx_H__
#define __ASM_MPC85xx_H__

#include <linux/config.h>
#include <asm/mmu.h>

#ifdef CONFIG_85xx

#ifdef CONFIG_MPC8540_ADS
#include <platforms/85xx/mpc8540_ads.h>
#endif
#if defined(CONFIG_MPC8555_CDS) || defined(CONFIG_MPC8548_CDS)
#include <platforms/85xx/mpc8555_cds.h>
#endif
#ifdef CONFIG_MPC8560_ADS
#include <platforms/85xx/mpc8560_ads.h>
#endif
#ifdef CONFIG_SBC8560
#include <platforms/85xx/sbc8560.h>
#endif
#ifdef CONFIG_STX_GP3
#include <platforms/85xx/stx_gp3.h>
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

/* Offset from CCSRBAR */
#define MPC85xx_CPM_OFFSET	(0x80000)
#define MPC85xx_CPM_SIZE	(0x40000)
#define MPC85xx_DMA_OFFSET	(0x21000)
#define MPC85xx_DMA_SIZE	(0x01000)
#define MPC85xx_DMA0_OFFSET	(0x21100)
#define MPC85xx_DMA0_SIZE	(0x00080)
#define MPC85xx_DMA1_OFFSET	(0x21180)
#define MPC85xx_DMA1_SIZE	(0x00080)
#define MPC85xx_DMA2_OFFSET	(0x21200)
#define MPC85xx_DMA2_SIZE	(0x00080)
#define MPC85xx_DMA3_OFFSET	(0x21280)
#define MPC85xx_DMA3_SIZE	(0x00080)
#define MPC85xx_ENET1_OFFSET	(0x24000)
#define MPC85xx_ENET1_SIZE	(0x01000)
#define MPC85xx_MIIM_OFFSET	(0x24520)
#define MPC85xx_MIIM_SIZE	(0x00018)
#define MPC85xx_ENET2_OFFSET	(0x25000)
#define MPC85xx_ENET2_SIZE	(0x01000)
#define MPC85xx_ENET3_OFFSET	(0x26000)
#define MPC85xx_ENET3_SIZE	(0x01000)
#define MPC85xx_GUTS_OFFSET	(0xe0000)
#define MPC85xx_GUTS_SIZE	(0x01000)
#define MPC85xx_IIC1_OFFSET	(0x03000)
#define MPC85xx_IIC1_SIZE	(0x00100)
#define MPC85xx_OPENPIC_OFFSET	(0x40000)
#define MPC85xx_OPENPIC_SIZE	(0x40000)
#define MPC85xx_PCI1_OFFSET	(0x08000)
#define MPC85xx_PCI1_SIZE	(0x01000)
#define MPC85xx_PCI2_OFFSET	(0x09000)
#define MPC85xx_PCI2_SIZE	(0x01000)
#define MPC85xx_PEX_OFFSET	(0x0a000)
#define MPC85xx_PEX_SIZE	(0x01000)
#define MPC85xx_PERFMON_OFFSET	(0xe1000)
#define MPC85xx_PERFMON_SIZE	(0x01000)
#define MPC85xx_SEC2_OFFSET	(0x30000)
#define MPC85xx_SEC2_SIZE	(0x10000)
#define MPC85xx_UART0_OFFSET	(0x04500)
#define MPC85xx_UART0_SIZE	(0x00100)
#define MPC85xx_UART1_OFFSET	(0x04600)
#define MPC85xx_UART1_SIZE	(0x00100)

#define MPC85xx_CCSRBAR_SIZE	(1024*1024)

/* Let modules/drivers get at CCSRBAR */
extern phys_addr_t get_ccsrbar(void);

#ifdef MODULE
#define CCSRBAR get_ccsrbar()
#else
#define CCSRBAR BOARD_CCSRBAR
#endif

enum ppc_sys_devices {
	MPC85xx_TSEC1,
	MPC85xx_TSEC2,
	MPC85xx_FEC,
	MPC85xx_IIC1,
	MPC85xx_DMA0,
	MPC85xx_DMA1,
	MPC85xx_DMA2,
	MPC85xx_DMA3,
	MPC85xx_DUART,
	MPC85xx_PERFMON,
	MPC85xx_SEC2,
	MPC85xx_CPM_SPI,
	MPC85xx_CPM_I2C,
	MPC85xx_CPM_USB,
	MPC85xx_CPM_SCC1,
	MPC85xx_CPM_SCC2,
	MPC85xx_CPM_SCC3,
	MPC85xx_CPM_SCC4,
	MPC85xx_CPM_FCC1,
	MPC85xx_CPM_FCC2,
	MPC85xx_CPM_FCC3,
	MPC85xx_CPM_MCC1,
	MPC85xx_CPM_MCC2,
	MPC85xx_CPM_SMC1,
	MPC85xx_CPM_SMC2,
	MPC85xx_eTSEC1,
	MPC85xx_eTSEC2,
	MPC85xx_eTSEC3,
	MPC85xx_eTSEC4,
	MPC85xx_IIC2,
	MPC85xx_MDIO,
	MPC85xx_MDIO_BB,
	NUM_PPC_SYS_DEVS,
};

/* Internal interrupts are all Level Sensitive, and Positive Polarity */
#define MPC85XX_INTERNAL_IRQ_SENSES \
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  0 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  1 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  2 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  3 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  4 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  5 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  6 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  7 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  8 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  9 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 10 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 11 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 12 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 13 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 14 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 15 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 16 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 17 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 18 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 19 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 20 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 21 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 22 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 23 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 24 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 25 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 26 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 27 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 28 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 29 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 30 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 31 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 32 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 33 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 34 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 35 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 36 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 37 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 38 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 39 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 40 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 41 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 42 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 43 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 44 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 45 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 46 */	\
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE)	/* Internal 47 */

/* FCC1 Clock Source Configuration.  These can be 
 * redefined in the board specific file. 
 *    Can only choose from CLK9-12 */ 

#define F1_RXCLK       12 
#define F1_TXCLK       11 

/* FCC2 Clock Source Configuration.  These can be 
 * redefined in the board specific file. 
 *    Can only choose from CLK13-16 */ 
#define F2_RXCLK       13 
#define F2_TXCLK       14 

/* FCC3 Clock Source Configuration.  These can be 
 * redefined in the board specific file. 
 *    Can only choose from CLK13-16 */ 
#define F3_RXCLK       15 
#define F3_TXCLK       16 

/* MDIO and MDCK settings. These can be redefined in the 
 * board specific file.*/ 
#define PC_MDIO         0x00400000U 
#define PC_MDCK         0x00200000U 

/* Automatically generates register configurations */ 
#define PC_CLK(x)      ((uint)(1<<(x-1)))      /* FCC CLK I/O ports */ 

#define CMXFCR_RF1CS(x)        ((uint)((x-5)<<27))     /* FCC1 Receive Clock Source */ 
#define CMXFCR_TF1CS(x)        ((uint)((x-5)<<24))     /* FCC1 Transmit Clock Source */ 
#define CMXFCR_RF2CS(x)        ((uint)((x-9)<<19))     /* FCC2 Receive Clock Source */ 
#define CMXFCR_TF2CS(x) ((uint)((x-9)<<16))    /* FCC2 Transmit Clock Source */ 
#define CMXFCR_RF3CS(x)        ((uint)((x-9)<<11))     /* FCC3 Receive Clock Source */ 
#define CMXFCR_TF3CS(x) ((uint)((x-9)<<8))     /* FCC3 Transmit Clock Source */ 

#define PC_F1RXCLK     PC_CLK(F1_RXCLK) 
#define PC_F1TXCLK     PC_CLK(F1_TXCLK) 
#define CMX1_CLK_ROUTE (CMXFCR_RF1CS(F1_RXCLK) | CMXFCR_TF1CS(F1_TXCLK)) 
#define CMX1_CLK_MASK  ((uint)0xff000000) 

#define PC_F2RXCLK     PC_CLK(F2_RXCLK) 
#define PC_F2TXCLK     PC_CLK(F2_TXCLK) 
#define CMX2_CLK_ROUTE (CMXFCR_RF2CS(F2_RXCLK) | CMXFCR_TF2CS(F2_TXCLK)) 
#define CMX2_CLK_MASK  ((uint)0x00ff0000) 

#define PC_F3RXCLK     PC_CLK(F3_RXCLK) 
#define PC_F3TXCLK     PC_CLK(F3_TXCLK) 
#define CMX3_CLK_ROUTE (CMXFCR_RF3CS(F3_RXCLK) | CMXFCR_TF3CS(F3_TXCLK)) 
#define CMX3_CLK_MASK  ((uint)0x0000ff00) 

/* Some board-specific defines here... Temporary I hope 
 * -vb*/ 
#define CPMUX_CLK_MASK (CMX3_CLK_MASK | CMX2_CLK_MASK) 
#define CPMUX_CLK_ROUTE (CMX3_CLK_ROUTE | CMX2_CLK_ROUTE) 
 
#define CLK_TRX (PC_F3TXCLK | PC_F3RXCLK | PC_F2TXCLK | PC_F2RXCLK) 

 
/* I/O Pin assignment for FCC1.  I don't yet know the 
 * best way to do this, 
 * but there is little variation among the choices. */ 
#define PA1_COL                0x00000001U 
#define PA1_CRS                0x00000002U 
#define PA1_TXER       0x00000004U 
#define PA1_TXEN       0x00000008U 
#define PA1_RXDV       0x00000010U 
#define PA1_RXER       0x00000020U 
#define PA1_TXDAT      0x00003c00U 
#define PA1_RXDAT      0x0003c000U 
#define PA1_PSORA0     (PA1_RXDAT | PA1_TXDAT) 
#define PA1_PSORA1     (PA1_COL | PA1_CRS | PA1_TXER | PA1_TXEN | \
			PA1_RXDV | PA1_RXER) 
#define PA1_DIRA0      (PA1_RXDAT | PA1_CRS | PA1_COL | PA1_RXER | PA1_RXDV) 
#define PA1_DIRA1      (PA1_TXDAT | PA1_TXEN | PA1_TXER) 

/* I/O Pin assignment for FCC2.  I don't yet know the best way to 
 * do this, 
 *  * but there is little variation among the choices. 
 *   */ 
#define PB2_TXER	0x00000001U 
#define PB2_RXDV       0x00000002U 
#define PB2_TXEN       0x00000004U 
#define PB2_RXER       0x00000008U 
#define PB2_COL		0x00000010U 
#define PB2_CRS                0x00000020U 
#define PB2_TXDAT      0x000003c0U 
#define PB2_RXDAT      0x00003c00U 
#define PB2_PSORB0     (PB2_RXDAT | PB2_TXDAT | PB2_CRS | PB2_COL | \
			PB2_RXER | PB2_RXDV | PB2_TXER) 
#define PB2_PSORB1     (PB2_TXEN) 
#define PB2_DIRB0      (PB2_RXDAT | PB2_CRS | PB2_COL | PB2_RXER | PB2_RXDV) 
#define PB2_DIRB1      (PB2_TXDAT | PB2_TXEN | PB2_TXER) 

/* I/O Pin assignment for FCC3.  I don't yet know the best way to 
 * do this, 
 * but there is little variation among the choices. 
 */ 
#define PB3_RXDV       0x00004000U 
#define PB3_RXER       0x00008000U 
#define PB3_TXER       0x00010000U 
#define PB3_TXEN       0x00020000U 
#define PB3_COL                0x00040000U 
#define PB3_CRS                0x00080000U 
#define PB3_TXDAT      0x0e000000U 
#define PC3_TXDAT      0x00000010U 
#define PB3_RXDAT      0x00f00000U 
#define PB3_PSORB0     (PB3_RXDAT | PB3_TXDAT | PB3_CRS | PB3_COL | \
			PB3_RXER | PB3_RXDV | PB3_TXER | PB3_TXEN) 
#define PB3_PSORB1     0 
#define PB3_DIRB0      (PB3_RXDAT | PB3_CRS | PB3_COL | PB3_RXER | PB3_RXDV) 
#define PB3_DIRB1      (PB3_TXDAT | PB3_TXEN | PB3_TXER) 
#define PC3_DIRC1      (PC3_TXDAT) 

#define FCC_MEM_OFFSET(x) (CPM_FCC_SPECIAL_BASE + (x*128)) 
#define FCC1_MEM_OFFSET FCC_MEM_OFFSET(0) 
#define FCC2_MEM_OFFSET FCC_MEM_OFFSET(1) 
#define FCC3_MEM_OFFSET FCC_MEM_OFFSET(2) 

#endif /* CONFIG_85xx */
#endif /* __ASM_MPC85xx_H__ */
#endif /* __KERNEL__ */
