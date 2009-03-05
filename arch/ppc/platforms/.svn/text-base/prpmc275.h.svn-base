/*
 * arch/ppc/platforms/prpmc275.h
 * 
 * Definitions for Force PPMC275 development board.
 *
 * Vladimir A. Barinov <vbrinov@ru.mvista.com>
 *
 * Based on code done by Rabeeh Khoury - rabeeh@galileo.co.il
 * Based on code done by Mark A. Greer <mgreer@mvista.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

/*
 * The MV64360 has 2 PCI buses each with 1 window from the CPU bus to
 * PCI I/O space and 4 windows from the CPU bus to PCI MEM space.
 * We'll only use one PCI MEM window on each PCI bus.
 *
 * This is the CPU physical memory map (windows must be at least 1MB and start
 * on a boundary that is a multiple of the window size):
 *	
 *	PPMC275 MEM MAP			
 *
 *	0xff800000-0xffffffff		- Boot flash (BootCS#)
 *	0xf2040000-0xff7fffff		- Unused
 * 	0xf2000000-0xf203ffff		- Integrated SRAM
 * 	0xf1010000-0xf1ffffff		- Unused
 * 	0xf1000000-0xf100ffff		- MV64360 Registers
 * 	0xa8000000-0xf0ffffff		- Unused
 * 	0xa4000000-0xa7ffffff		- User flash expansion
 * 	0xa2000000-0xa3ffffff		- User flash 1 ( CS1)
 * 	0xa0000000-0xa1ffffff           - User flash 0 ( CS0)
 * 	0x99000000-0x9fffffff		- <hole>
 * 	0x98000000-0x98ffffff		- PCI 1 I/O (defined in mv64360.h)
 * 	0x90000000-0x97ffffff		- PCI 1 MEM (defined in mv64360.h)
 * 	0x88000000-0x88ffffff		- PCI 0 I/O (defined in mv64360.h)
 * 	0x80000000-0x87ffffff		- PCI 0 MEM (defined in mv64360.h)
 *	0x20000000-0x7fffffff		- Reserved for SDRAM expansion
 * 	0x00000000-0x1fffffff		- On Board SDRAM ( CS0)
 */

#ifndef __PPC_PLATFORMS_PPMC275_H
#define __PPC_PLATFORMS_PPMC275_H

/*
 * CPU Physical Memory Map setup.
 */

#define PPMC275_BOOT_FLASH_BASE			0xff800000
#define PPMC275_USER_FLASH_BASE			0xa0000000
#define PPMC275_INTERNAL_SRAM_BASE		0xf2000000

#define PPMC275_BOOT_FLASH_SIZE			0x00100000	/* 1MB of Embed FLASH */
#define PPMC275_USER_FLASH_SIZE			0x08000000	/* <= 128 MB Extern FLASH */

#define PPMC275_BUS_FREQ			133333333

#define PPMC275_DEFAULT_BAUD			115200
#define PPMC275_MPSC_CLK_SRC			8	/* TCLK */

#define PPMC275_PCI0_MEM_START_PROC_ADDR        0x80000000
#define PPMC275_PCI0_MEM_START_PCI_HI_ADDR      0x00000000
#define PPMC275_PCI0_MEM_START_PCI_LO_ADDR      0x80000000
#define PPMC275_PCI0_MEM_SIZE                   0x08000000
#define PPMC275_PCI0_IO_START_PCI_ADDR          0x00000000
#define PPMC275_PCI0_IO_SIZE                    0x01000000

#define PPMC275_PCI1_MEM_START_PROC_ADDR        0x90000000
#define PPMC275_PCI1_MEM_START_PCI_HI_ADDR      0x00000000
#define PPMC275_PCI1_MEM_START_PCI_LO_ADDR      0x90000000
#define PPMC275_PCI1_MEM_SIZE                   0x08000000
#define PPMC275_PCI1_IO_START_PCI_ADDR          0x01000000
#define PPMC275_PCI1_IO_SIZE                    0x01000000

#define PPMC275_PCI0_IO_START_PROC_ADDR         (PPMC275_PCI0_MEM_START_PROC_ADDR + PPMC275_PCI0_MEM_SIZE)
#define PPMC275_PCI1_IO_START_PROC_ADDR         (PPMC275_PCI1_MEM_START_PROC_ADDR + PPMC275_PCI1_MEM_SIZE)

/*
 * Board-specific IRQ info
 */
#define PPMC275_PCI_0_IRQ               64+27
#define PPMC275_PCI_0_IRQ_B             64+29
#define PPMC275_PCI_0_IRQ_C             64+16
#define PPMC275_PCI_0_IRQ_D             64+17

#define PPMC275_PCI_1_IRQ               64+30

#endif				/* __PPC_PLATFORMS_PPMC275_H */
