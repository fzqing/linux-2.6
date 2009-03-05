/*
 * arch/ppc/platforms/86xx/mpc86xx_sys.c
 *
 * MPC86xx System descriptions
 *
 * Maintainer: Jeff Brown <jeffrey@freescale.com>
 *
 * Copyright 2005 Freescale Semiconductor Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <asm/ppc_sys.h>

struct ppc_sys_spec *cur_ppc_sys_spec;
struct ppc_sys_spec ppc_sys_specs[] = {
	{
		.ppc_sys_name	= "MPC8641D",
		.mask		= 0xFFFF0F00,
		.value		= 0x80900100,
		.num_devices	= 8,
		.device_list	= (enum ppc_sys_devices[])
		{
			MPC86xx_TSEC1, MPC86xx_TSEC2, MPC86xx_TSEC3, MPC86xx_TSEC4,
			MPC86xx_DUART, MPC86xx_MDIO,MPC86xx_IIC1, MPC86xx_IIC2,
		},
	},
	{
		.ppc_sys_name	= "MPC8641",
		.mask		= 0xFFFF0F00,
		.value		= 0x80900000,
		.num_devices	= 8,
		.device_list	= (enum ppc_sys_devices[])
		{
			MPC86xx_TSEC1, MPC86xx_TSEC2, MPC86xx_TSEC3, MPC86xx_TSEC4,
			MPC86xx_DUART, MPC86xx_MDIO,MPC86xx_IIC1, MPC86xx_IIC2,
		},
	},
};
