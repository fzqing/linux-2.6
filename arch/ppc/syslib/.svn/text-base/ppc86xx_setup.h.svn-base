/*
 * arch/ppc/syslib/ppc86xx_setup.h
 *
 * MPC86XX common board definitions
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

#ifndef __PPC_SYSLIB_PPC86XX_SETUP_H
#define __PPC_SYSLIB_PPC86XX_SETUP_H

#include <linux/config.h>
#include <linux/init.h>
#include <asm/ppcboot.h>

extern unsigned long mpc86xx_find_end_of_memory(void) __init;
extern void mpc86xx_calibrate_decr(void) __init;
extern void mpc86xx_early_serial_map(void) __init;
extern void mpc86xx_restart(char *cmd);
extern void mpc86xx_power_off(void);
extern void mpc86xx_halt(void);
extern void mpc86xx_setup_hose(void) __init;

#define PCIE1_CFG_ADDR_OFFSET	(0x8000)
#define PCIE1_CFG_DATA_OFFSET	(0x8004)

#define PCIE2_CFG_ADDR_OFFSET	(0x9000)
#define PCIE2_CFG_DATA_OFFSET	(0x9004)

#define MPC86xx_PCIE_OFFSET PCIE1_CFG_ADDR_OFFSET
#define MPC86xx_PCIE_SIZE (0x1000)

/* Serial Config */
#ifdef CONFIG_SERIAL_MANY_PORTS
#define RS_TABLE_SIZE  64
#else
#define RS_TABLE_SIZE  2
#endif

#ifndef BASE_BAUD
#define BASE_BAUD 115200
#endif

#ifndef SERIAL_PORT_DFNS
#define SERIAL_PORT_DFNS
#endif

#endif /* __PPC_SYSLIB_PPC86XX_SETUP_H */

