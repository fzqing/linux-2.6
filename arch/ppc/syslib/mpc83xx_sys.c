/*
 * arch/ppc/platforms/83xx/mpc83xx_sys.c
 *
 * MPC83xx System descriptions
 *
 * Maintainer: Kumar Gala <kumar.gala@freescale.com>
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
		.ppc_sys_name	= "8349E",
		.mask 		= 0xFFFF0000,
		.value 		= 0x80500000,
		.num_devices	= 9,
		.device_list	= (enum ppc_sys_devices[])
		{
			MPC83xx_TSEC1, MPC83xx_TSEC2, MPC83xx_IIC1,
			MPC83xx_IIC2, MPC83xx_DUART, MPC83xx_SEC2,
			MPC83xx_USB2_DR, MPC83xx_USB2_MPH, MPC83xx_MDIO
		},
	},
	{
		.ppc_sys_name	= "8349E_ITX",
		.mask 		= 0xFFFF0000,
		.value 		= 0x80500000,
		.num_devices	= 10,
		.device_list	= (enum ppc_sys_devices[])
		{
			MPC83xx_TSEC1, MPC83xx_TSEC2, MPC83xx_IIC1,
			MPC83xx_IIC2, MPC83xx_DUART, MPC83xx_SEC2,
			MPC83xx_USB2_DR, MPC83xx_USB2_MPH, MPC83xx_MDIO_TSEC1,
			MPC83xx_CFIDE,
		},
	},
	{
		.ppc_sys_name	= "8349",
		.mask 		= 0xFFFF0000,
		.value 		= 0x80510000,
		.num_devices	= 8,
		.device_list	= (enum ppc_sys_devices[])
		{
			MPC83xx_TSEC1, MPC83xx_TSEC2, MPC83xx_IIC1,
			MPC83xx_IIC2, MPC83xx_DUART,
			MPC83xx_USB2_DR, MPC83xx_USB2_MPH, MPC83xx_MDIO
		},
	},
	{
		.ppc_sys_name	= "8347E",
		.mask 		= 0xFFFF0000,
		.value 		= 0x80520000,
		.num_devices	= 9,
		.device_list	= (enum ppc_sys_devices[])
		{
			MPC83xx_TSEC1, MPC83xx_TSEC2, MPC83xx_IIC1,
			MPC83xx_IIC2, MPC83xx_DUART, MPC83xx_SEC2,
			MPC83xx_USB2_DR, MPC83xx_USB2_MPH, MPC83xx_MDIO
		},
	},
	{
		.ppc_sys_name	= "8347",
		.mask 		= 0xFFFF0000,
		.value 		= 0x80530000,
		.num_devices	= 8,
		.device_list	= (enum ppc_sys_devices[])
		{
			MPC83xx_TSEC1, MPC83xx_TSEC2, MPC83xx_IIC1,
			MPC83xx_IIC2, MPC83xx_DUART,
			MPC83xx_USB2_DR, MPC83xx_USB2_MPH, MPC83xx_MDIO
		},
	},
	{
		.ppc_sys_name	= "8343E",
		.mask 		= 0xFFFF0000,
		.value 		= 0x80540000,
		.num_devices	= 8,
		.device_list	= (enum ppc_sys_devices[])
		{
			MPC83xx_TSEC1, MPC83xx_TSEC2, MPC83xx_IIC1,
			MPC83xx_IIC2, MPC83xx_DUART, MPC83xx_SEC2,
			MPC83xx_USB2_DR, MPC83xx_MDIO
		},
	},
	{
		.ppc_sys_name	= "8343",
		.mask 		= 0xFFFF0000,
		.value 		= 0x80550000,
		.num_devices	= 7,
		.device_list	= (enum ppc_sys_devices[])
		{
			MPC83xx_TSEC1, MPC83xx_TSEC2, MPC83xx_IIC1,
			MPC83xx_IIC2, MPC83xx_DUART,
			MPC83xx_USB2_DR, MPC83xx_MDIO
		},
	},
	{
		.ppc_sys_name	= "8360E",
		.mask		= 0xFFFF0000,
		.value		= 0x80480000,
		.num_devices	= 5,
		.device_list	= (enum ppc_sys_devices[])
		{
			MPC83xx_QE_UCC1,MPC83xx_QE_UCC2,
			MPC83xx_IIC1, MPC83xx_IIC2, MPC83xx_DUART,

		},
	},
	{
		.ppc_sys_name	= "8325E",
		.mask		= 0xFFFF0000,
		.value		= 0x80680000,
		.num_devices	= 7,
		.device_list	= (enum ppc_sys_devices[])
		{
			MPC83xx_IIC1, MPC83xx_DUART,
			MPC83xx_QE_UCC3, MPC83xx_QE_UCC4,
			MPC83xx_QE_SPI1, MPC83xx_QE_SPI2, MPC83xx_QE_USB,
		},
	},
	{
		.ppc_sys_name	= "8325",
		.mask		= 0xFFFF0000,
		.value		= 0x80690000,
		.num_devices	= 7,
		.device_list	= (enum ppc_sys_devices[])
		{
			MPC83xx_IIC1, MPC83xx_DUART,
			MPC83xx_QE_UCC3, MPC83xx_QE_UCC4,
			MPC83xx_QE_SPI1, MPC83xx_QE_SPI2, MPC83xx_QE_USB,
		},
	},
	{
		.ppc_sys_name	= "8323E",
		.mask		= 0xFFFF0000,
		.value		= 0x80620000,
		.num_devices	= 7,
		.device_list	= (enum ppc_sys_devices[])
		{
			MPC83xx_IIC1, MPC83xx_DUART,
			MPC83xx_QE_UCC3, MPC83xx_QE_UCC4,
			MPC83xx_QE_SPI1, MPC83xx_QE_SPI2, MPC83xx_QE_USB,
		},
	},
	{
		.ppc_sys_name	= "8323",
		.mask		= 0xFFFF0000,
		.value		= 0x80630000,
		.num_devices	= 7,
		.device_list	= (enum ppc_sys_devices[])
		{
			MPC83xx_IIC1, MPC83xx_DUART,
			MPC83xx_QE_UCC3, MPC83xx_QE_UCC4,
			MPC83xx_QE_SPI1, MPC83xx_QE_SPI2, MPC83xx_QE_USB,
		},
	},
	{
		.ppc_sys_name	= "8321E",
		.mask		= 0xFFFF0000,
		.value		= 0x80660000,
		.num_devices	= 4,
		.device_list	= (enum ppc_sys_devices[])
		{
			MPC83xx_IIC1, MPC83xx_DUART,
			MPC83xx_QE_UCC3, MPC83xx_QE_UCC4,
		},
	},
	{
		.ppc_sys_name	= "8321",
		.mask		= 0xFFFF0000,
		.value		= 0x80670000,
		.num_devices	= 4,
		.device_list	= (enum ppc_sys_devices[])
		{
			MPC83xx_IIC1, MPC83xx_DUART,
			MPC83xx_QE_UCC3, MPC83xx_QE_UCC4,
		},
	},
	{	/* default match */
		.ppc_sys_name	= "",
		.mask 		= 0x00000000,
		.value 		= 0x00000000,
	},
};
