/*
 * arch/ppc/platforms/83xx/mpc832xe_mds.h
 *
 * Description: MPC832XE MDS common board definitions
 *
 * Author: Shlomi Gridish <gridish@freescale.com>
 *
 * Copyright (C) Freescale Semiconductor, Inc. 2005. All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#ifndef __MPC832XE_MDS_H__
#define __MPC832XE_MDS_H__

#include <linux/config.h>
#include <linux/init.h>
#include <linux/seq_file.h>
#include <syslib/ppc83xx_setup.h>
#include <asm/ppcboot.h>

#define VIRT_IMMRBAR		((uint)0xfe000000)

#define BCSR_PHYS_ADDR		((uint)0xf8000000)
#define BCSR_VIRT_ADDR		((uint)0xfe200000)
#define BCSR_SIZE			((uint)(128 * 1024))

#define QE_MAP_ADDR			((uint)immrbar + 0x100000)

#define PIRQA	MPC83xx_IRQ_EXT4
#define PIRQB	MPC83xx_IRQ_EXT5
#define PIRQC	MPC83xx_IRQ_EXT6
#define PIRQD	MPC83xx_IRQ_EXT7

#endif /* __MPC832XE_MDS_H__ */
