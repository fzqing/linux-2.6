/*
 * taiga.h
 *
 * Definitions for Freescale TAIGA platform
 *
 * Author: Jacob Pan
 *         jacob.pan@freescale.com
 * Author: Xianghua Xiao
 *         updated to taiga platform from emulation platform. 2005.5
 *         x.xiao@freescale.com
 * Maintainer: Roy Zang <roy.zang@freescale.com>
 *
 * 2006 (c) Freescale Semiconductor, Inc.  This file is licensed under
 * the terms of the GNU General Public License version 2.  This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef __PPC_PLATFORMS_TAIGA_H
#define __PPC_PLATFORMS_TAIGA_H

#include <asm/ppcboot.h>

/* Ethernet defines
 */
#define TSI108_ETH
#define TSI108_ETH_MAX_PORTS	(2)	/* TSI108 Ethernet block has 2 ports */
#define TSI108_PHY0_ADDR	(8)	/* PHY address for GIGE port 0 */
#define TSI108_PHY1_ADDR	(9)	/* PHY address for GIGE port 1 */

/*
 * Serial defines.
 */
#define GEN550_CONSOLE
#define TAIGA_SERIAL_0		(TSI108_CSR_ADDR_PHYS + 0x7808)
#define TAIGA_SERIAL_1		(TSI108_CSR_ADDR_PHYS + 0x7C08)

#define RS_TABLE_SIZE  2	/* two COM ports */

/*
 * Rate recalculated for the internal UART clock @ 133MHz
 * and set to be compatible with standard UART divider by 16.
 * Tsi108 UART divides internal clock by 2 not 16.
 */
#define BASE_BAUD	(133000000/2)
#define UART_CLK	(133000000*8)

#ifdef CONFIG_SERIAL_DETECT_IRQ
#define STD_COM_FLAGS (ASYNC_BOOT_AUTOCONF|ASYNC_AUTO_IRQ)
#else
#define STD_COM_FLAGS (ASYNC_BOOT_AUTOCONF)
#endif

#define STD_SERIAL_PORT_DFNS \
        { 0, BASE_BAUD, TAIGA_SERIAL_0, 4, STD_COM_FLAGS, /* ttyS0 */ \
		iomem_base: (u8 *)TAIGA_SERIAL_0,			  \
		io_type: SERIAL_IO_MEM },				  \
        { 0, BASE_BAUD, TAIGA_SERIAL_1, 3, STD_COM_FLAGS, /* ttyS1 */ \
		iomem_base: (u8 *)TAIGA_SERIAL_1,			  \
		io_type: SERIAL_IO_MEM },

#define SERIAL_PORT_DFNS \
        STD_SERIAL_PORT_DFNS

/* Base Addresses for the PCI bus
 * HOST_PCI initiator (outbound) window to PCI bus
 */
#define TAIGA_PCI_MEM_OFFSET	(0x00000000)
#define TAIGA_PCI_IO_BASE_PHYS	(0xfa000000)
#define TAIGA_PCI_IO_BASE_VIRT	(TAIGA_PCI_IO_BASE_PHYS)
#define TAIGA_PCI_MEM_BASE	(0xe0000000)
#define TAIGA_ISA_IO_BASE	(0x00000000)
#define TAIGA_ISA_MEM_BASE	(0x00000000)

#define	TSI10X_PCI_CNFG_BASE	(0xfb000000)
#define TAIGA_PCI_CFG_BASE_PHYS	(0xfb000000)
#define TAIGA_PCI_CFG_BASE_VIRT	(TAIGA_PCI_CFG_BASE_PHYS)
#define TAIGA_PCI_CFG_SIZE	(0x01000000)

#define TAIGA_PCI_MEM_START	(0xE0000000)
#define TAIGA_PCI_MEM_END	(0xF9FFFFFF)

#define TAIGA_PCI_IO_START	(0xFA000000)
#define TAIGA_PCI_IO_END	(0xFA00FFFF)

#define TAIGA_PCI_CFG_OFFSET	(0xFB000000)
#define TAIGA_PCI_IO_OFFSET	(0xFA000000)

#define TAIGA_PCI_MEM32_OFFSET	0x00000000	/* PCI MEM32 space offset within
						   the PCI window */
#define TAIGA_PCI_PFM_OFFSET	0x10000000	/* PCI PFM1 space offset within
						   the PCI window */

/* Memory-mapped CIU resources (CPU view)
 * The memory map is set by initialization code in monitor
 */

#define TSI108_CSR_ADDR_PHYS	(0xC0000000)	/* Physical Tsi108 CSR Base Address */
#define TSI108_CSR_ADDR_VIRT	(0xF0000000)	/* Virtual Tsi108 CSR Base Address */

#define FLASH_BASE_ADDR		(0xFF000000)	/* Boot FLASH Base Address */

#define SDRAM_BASE_ADDR		0x00000000	/* SDRAM base address */
#define BOARD_SDRAM_SIZE	0x20000000	/* Default value: 512MB */

/* Size of SDRAM space reserved for user space. This limits space available
 * for dynamic allocation.
 */
#define USER_RESERVED_MEM	(BOARD_SDRAM_SIZE - 0x02000000)

#define USER_RESERVED_BASE  \
  (SDRAM_BASE_ADDR + (BOARD_SDRAM_SIZE - USER_RESERVED_MEM)

/* Memory mapped NVRAM/RTC */
#define TAIGA_NVRAM_BASE_ADDR	(0xFC000000)
#define TAIGA_NVRAM_SIZE	(0x8000)

#endif				/* __PPC_PLATFORMS_TAIGA_H */
