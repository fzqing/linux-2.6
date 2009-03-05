/*
 * include/asm-ppc/mpc86xx.h
 *
 * MPC86xx definitions
 *
 * Maintainer: Jeff Brown <jeffrey@freescale.com>
 *
 * Copyright 2004 Freescale Semiconductor, Inc
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifdef __KERNEL__
#ifndef __ASM_MPC86xx_H__
#define __ASM_MPC86xx_H__

#include <linux/config.h>
#include <asm/mmu.h>

#ifdef CONFIG_86xx

#ifdef CONFIG_MPC8641_HPCN
#include <platforms/86xx/mpc8641_hpcn.h>
#endif


#define _IO_BASE	isa_io_base
#define _ISA_MEM_BASE	isa_mem_base
#ifdef CONFIG_PCI
#define PCI_DRAM_OFFSET pci_dram_offset
#else
#define PCI_DRAM_OFFSET 0
#endif

#define CPU0_BOOT_RELEASE 0x01000000
#define CPU1_BOOT_RELEASE 0x02000000
#define CPU_ALL_RELEASED (CPU0_BOOT_RELEASE | CPU1_BOOT_RELEASE)
#define MCM_PORT_CONFIG_OFFSET 0x1010
/*
 * The "residual" board information structure the boot loader passes
 * into the kernel.
 */
extern unsigned char __res[];

/* Offset from CCSRBAR */
#define MPC86xx_DMA_OFFSET	(0x21000)
#define MPC86xx_DMA_SIZE	(0x01000)
#define MPC86xx_DMA0_OFFSET	(0x21100)
#define MPC86xx_DMA0_SIZE	(0x00080)
#define MPC86xx_DMA1_OFFSET	(0x21180)
#define MPC86xx_DMA1_SIZE	(0x00080)
#define MPC86xx_DMA2_OFFSET	(0x21200)
#define MPC86xx_DMA2_SIZE	(0x00080)
#define MPC86xx_DMA3_OFFSET	(0x21280)
#define MPC86xx_DMA3_SIZE	(0x00080)
#define MPC86xx_ENET1_OFFSET	(0x24000)
#define MPC86xx_ENET1_SIZE	(0x01000)
#define MPC86xx_MIIM_OFFSET	(0x24520)
#define MPC86xx_MIIM_SIZE	(0x00018)
#define MPC86xx_ENET2_OFFSET	(0x25000)
#define MPC86xx_ENET2_SIZE	(0x01000)
#define MPC86xx_ENET3_OFFSET	(0x26000)
#define MPC86xx_ENET3_SIZE	(0x01000)
#define MPC86xx_ENET4_OFFSET	(0x27000)
#define MPC86xx_ENET4_SIZE	(0x01000)
#define MPC86xx_GUTS_OFFSET	(0xe0000)
#define MPC86xx_GUTS_SIZE	(0x01000)
#define MPC86XX_RSTCR_OFFSET	(0xe00b0)
#define MPC86xx_IIC1_OFFSET	(0x03000)
#define MPC86xx_IIC1_SIZE	(0x00100)
#define MPC86xx_IIC2_OFFSET	(0x03100)
#define MPC86xx_IIC2_SIZE	(0x00100)
#define MPC86xx_OPENPIC_OFFSET	(0x40000)
#define MPC86xx_OPENPIC_SIZE	(0x40000)
#define MPC86xx_PCIE1_OFFSET	(0x08000)
#define MPC86xx_PCIE1_SIZE	(0x01000)
#define MPC86xx_PCIE2_OFFSET	(0x09000)
#define MPC86xx_PCIE2_SIZE	(0x01000)
#define MPC86xx_PERFMON_OFFSET	(0xe1000)
#define MPC86xx_PERFMON_SIZE	(0x01000)
#define MPC86xx_UART0_OFFSET	(0x04500)
#define MPC86xx_UART0_SIZE	(0x00100)
#define MPC86xx_UART1_OFFSET	(0x04600)
#define MPC86xx_UART1_SIZE	(0x00100)
#define MPC86xx_MCM_OFFSET	(0x00000)
#define MPC86xx_MCM_SIZE	(0x02000)

#define MPC86xx_CCSRBAR_SIZE	(1024*1024)

/* Let modules/drivers get at CCSRBAR */
extern phys_addr_t get_ccsrbar(void);

#ifdef MODULE
#define CCSRBAR get_ccsrbar()
#else
#define CCSRBAR BOARD_CCSRBAR
#endif

enum ppc_sys_devices {
	MPC86xx_TSEC1,
	MPC86xx_TSEC2,
	MPC86xx_TSEC3,
	MPC86xx_TSEC4,
	MPC86xx_DUART,
	MPC86xx_MDIO,
	MPC86xx_IIC1,
	MPC86xx_IIC2,
	NUM_PPC_SYS_DEVS,
};

#endif /* CONFIG_86xx */
#endif /* __ASM_MPC86xx_H__ */
#endif /* __KERNEL__ */
