/*
 * arch/ppc/platforms/86xx/mpc8641_cts.h
 *
 * MPC8641 HPCN board definitions
 *
 * Maintainer: Xianghua Xiao <x.xiao@freescale.com>
 *
 * Copyright 2006 Freescale Semiconductor Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#ifndef __MACH_MPC8641HPCN_H__
#define __MACH_MPC8641HPCN_H__

#include <linux/config.h>
#include <linux/initrd.h>
#include <syslib/ppc86xx_setup.h>
#include <linux/init.h>
#include <linux/seq_file.h>
#include <asm/ppcboot.h>

#define BOARD_CCSRBAR		((uint)0xf8000000)

struct seq_file;

extern int mpc86xx_hpcn_show_cpuinfo(struct seq_file *m);
extern void mpc86xx_hpcn_init_irq(void) __init;
extern void mpc86xx_hpcn_map_io(void) __init;

/* PCI interrupt controller */
#define PIRQA		3
#define PIRQB		4
#define PIRQC		5
#define PIRQD		6
#define PIRQ7		7
#define PIRQE		9
#define PIRQF		10
#define PIRQG		11
#undef PIRQH				/* disabled */

/* PCIE memory map */
#define MPC86XX_PCIE_LOWER_IO		0x00000000
#define MPC86XX_PCIE_UPPER_IO		0x00ffffff

#define MPC86XX_PCIE1_LOWER_MEM		0x80000000
#define MPC86XX_PCIE1_UPPER_MEM		0x9fffffff

#define MPC86XX_PCIE2_LOWER_MEM		0xa0000000
#define MPC86XX_PCIE2_UPPER_MEM		0xafffffff

#define MPC86XX_PCIE1_IO_BASE		0xe2000000
#define MPC86XX_PCIE2_IO_BASE		0xe3000000
#define MPC86XX_PCIE_MEM_OFFSET		0x00000000

#define MPC86XX_PCIE_IO_SIZE		0x01000000

#define MPC8641_SUPERIO_GPIO_OFFSET	0x0400

#define NR_8259_INTS			16

/*
 * The top 16 bits of the I/O BAR's for the ULI 5455 Audio Device and the
 * ULI 5457 Modem Device are hardwired to zero, causing pciauto_bus_scan() to
 * fail.  We will set up the BAR's manually using these values.
 */
#define MPC8641HPCN_ULI5455_IO_BASE	0x0000ee00
#define MPC8641HPCN_ULI5457_IO_BASE	0x0000ed00


#endif /* __MACH_MPC8641HPCN_H__ */
