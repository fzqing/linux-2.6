/*
 * arch/ppc/platforms/86xx/mpc86xx_devices.c
 *
 * MPC86xx Device descriptions
 *
 * Maintainer: Xianghua Xiao <x.xiao@freescale.com>
 *
 * Copyright 2006 Freescale Semiconductor Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/serial_8250.h>
#include <linux/fsl_devices.h>
#include <asm/mpc86xx.h>
#include <asm/irq.h>
#include <asm/ppc_sys.h>

/* We use offsets for IORESOURCE_MEM since we do not know at compile time
 * what CCSRBAR is, will get fixed up by mach_mpc86xx_fixup
 */
struct gianfar_mdio_data mpc86xx_mdio_pdata = {
};

static struct gianfar_platform_data mpc86xx_tsec1_pdata = {
	.device_flags = FSL_GIANFAR_DEV_HAS_GIGABIT |
	    FSL_GIANFAR_DEV_HAS_COALESCE | FSL_GIANFAR_DEV_HAS_RMON |
	    FSL_GIANFAR_DEV_HAS_MULTI_INTR,
};

static struct gianfar_platform_data mpc86xx_tsec2_pdata = {
	.device_flags = FSL_GIANFAR_DEV_HAS_GIGABIT |
	    FSL_GIANFAR_DEV_HAS_COALESCE | FSL_GIANFAR_DEV_HAS_RMON |
	    FSL_GIANFAR_DEV_HAS_MULTI_INTR,
};

static struct gianfar_platform_data mpc86xx_tsec3_pdata = {
	.device_flags = FSL_GIANFAR_DEV_HAS_GIGABIT |
	    FSL_GIANFAR_DEV_HAS_COALESCE | FSL_GIANFAR_DEV_HAS_RMON |
	    FSL_GIANFAR_DEV_HAS_MULTI_INTR,
};

static struct gianfar_platform_data mpc86xx_tsec4_pdata = {
	.device_flags = FSL_GIANFAR_DEV_HAS_GIGABIT |
	    FSL_GIANFAR_DEV_HAS_COALESCE | FSL_GIANFAR_DEV_HAS_RMON |
	    FSL_GIANFAR_DEV_HAS_MULTI_INTR,
};

static struct fsl_i2c_platform_data mpc86xx_fsl_i2c_pdata = {
	.device_flags = FSL_I2C_DEV_SEPARATE_DFSRR,
};

static struct fsl_i2c_platform_data mpc86xx_fsl_i2c2_pdata = {
	.device_flags = FSL_I2C_DEV_SEPARATE_DFSRR,
};

static struct plat_serial8250_port serial_platform_data[] = {
	[0] = {
		.mapbase	= 0x4500,
		.irq		= MPC86xx_IRQ_UART1,
		.iotype		= UPIO_MEM,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.line		= 0,
	},
	[1] = {
		.mapbase	= 0x4600,
		.irq		= MPC86xx_IRQ_UART2,
		.iotype		= UPIO_MEM,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.line		= 1,
	},
	{},
};

struct platform_device ppc_sys_platform_devices[] = {
	[MPC86xx_TSEC1] = {
		.name = "fsl-gianfar",
		.id	= 1,
		.dev.platform_data = &mpc86xx_tsec1_pdata,
		.num_resources	 = 4,
		.resource = (struct resource[]) {
			{
				.start	= MPC86xx_ENET1_OFFSET,
				.end	= MPC86xx_ENET1_OFFSET +
						MPC86xx_ENET1_SIZE - 1,
				.flags	= IORESOURCE_MEM,
			},
			{
				.name	= "tx",
				.start	= MPC86xx_IRQ_TSEC1_TX,
				.end	= MPC86xx_IRQ_TSEC1_TX,
				.flags	= IORESOURCE_IRQ,
			},
			{
				.name	= "rx",
				.start	= MPC86xx_IRQ_TSEC1_RX,
				.end	= MPC86xx_IRQ_TSEC1_RX,
				.flags	= IORESOURCE_IRQ,
			},
			{
				.name	= "error",
				.start	= MPC86xx_IRQ_TSEC1_ERROR,
				.end	= MPC86xx_IRQ_TSEC1_ERROR,
				.flags	= IORESOURCE_IRQ,
			},
		},
	},
	[MPC86xx_TSEC2] = {
		.name = "fsl-gianfar",
		.id	= 2,
		.dev.platform_data = &mpc86xx_tsec2_pdata,
		.num_resources	 = 4,
		.resource = (struct resource[]) {
			{
				.start	= MPC86xx_ENET2_OFFSET,
				.end	= MPC86xx_ENET2_OFFSET +
						MPC86xx_ENET2_SIZE - 1,
				.flags	= IORESOURCE_MEM,
			},
			{
				.name	= "tx",
				.start	= MPC86xx_IRQ_TSEC2_TX,
				.end	= MPC86xx_IRQ_TSEC2_TX,
				.flags	= IORESOURCE_IRQ,
			},
			{
				.name	= "rx",
				.start	= MPC86xx_IRQ_TSEC2_RX,
				.end	= MPC86xx_IRQ_TSEC2_RX,
				.flags	= IORESOURCE_IRQ,
			},
			{
				.name	= "error",
				.start	= MPC86xx_IRQ_TSEC2_ERROR,
				.end	= MPC86xx_IRQ_TSEC2_ERROR,
				.flags	= IORESOURCE_IRQ,
			},
		},
	},
	[MPC86xx_TSEC3] = {
		.name = "fsl-gianfar",
		.id	= 3,
		.dev.platform_data = &mpc86xx_tsec3_pdata,
		.num_resources	 = 4,
		.resource = (struct resource[]) {
			{
				.start	= MPC86xx_ENET3_OFFSET,
				.end	= MPC86xx_ENET3_OFFSET +
						MPC86xx_ENET3_SIZE - 1,
				.flags	= IORESOURCE_MEM,
			},
			{
				.name	= "tx",
				.start	= MPC86xx_IRQ_TSEC3_TX,
				.end	= MPC86xx_IRQ_TSEC3_TX,
				.flags	= IORESOURCE_IRQ,
			},
			{
				.name	= "rx",
				.start	= MPC86xx_IRQ_TSEC3_RX,
				.end	= MPC86xx_IRQ_TSEC3_RX,
				.flags	= IORESOURCE_IRQ,
			},
			{
				.name	= "error",
				.start	= MPC86xx_IRQ_TSEC3_ERROR,
				.end	= MPC86xx_IRQ_TSEC3_ERROR,
				.flags	= IORESOURCE_IRQ,
			},
		},
	},
	[MPC86xx_TSEC4] = {
		.name = "fsl-gianfar",
		.id	= 4,
		.dev.platform_data = &mpc86xx_tsec4_pdata,
		.num_resources	 = 4,
		.resource = (struct resource[]) {
			{
				.start	= MPC86xx_ENET4_OFFSET,
				.end	= MPC86xx_ENET4_OFFSET +
						MPC86xx_ENET4_SIZE - 1,
				.flags	= IORESOURCE_MEM,
			},
			{
				.name	= "tx",
				.start	= MPC86xx_IRQ_TSEC4_TX,
				.end	= MPC86xx_IRQ_TSEC4_TX,
				.flags	= IORESOURCE_IRQ,
			},
			{
				.name	= "rx",
				.start	= MPC86xx_IRQ_TSEC4_RX,
				.end	= MPC86xx_IRQ_TSEC4_RX,
				.flags	= IORESOURCE_IRQ,
			},
			{
				.name	= "error",
				.start	= MPC86xx_IRQ_TSEC4_ERROR,
				.end	= MPC86xx_IRQ_TSEC4_ERROR,
				.flags	= IORESOURCE_IRQ,
			},
		},
	},
	[MPC86xx_MDIO] = {
		.name = "fsl-gianfar_mdio",
		.id = 0,
		.dev.platform_data = &mpc86xx_mdio_pdata,
		.num_resources = 1,
		.resource = (struct resource[]) {
			{
				.start  = MPC86xx_MIIM_OFFSET,
				.end	= MPC86xx_MIIM_OFFSET +
						MPC86xx_MIIM_SIZE - 1,
				.flags  = IORESOURCE_MEM,
			},
		},

	},
	[MPC86xx_DUART] = {
		.name = "serial8250",
		.id = 0,
		.dev.platform_data = serial_platform_data,
		.num_resources = 0,
	},

	[MPC86xx_IIC1] = {
		.name = "fsl-i2c",
		.id = 1,
		.dev.platform_data = &mpc86xx_fsl_i2c_pdata,
		.num_resources	= 2,
		.resource = (struct resource[]) {
			{
				.start	= MPC86xx_IIC1_OFFSET,
				.end	= MPC86xx_IIC1_OFFSET +
						MPC86xx_IIC1_SIZE - 1,
				.flags	= IORESOURCE_MEM,
			},
			{
				.start	= MPC86xx_IRQ_IIC,
				.end	= MPC86xx_IRQ_IIC,
				.flags  = IORESOURCE_IRQ,
			}
		}
	},
	[MPC86xx_IIC2] = {
		.name = "fsl-i2c",
		.id  = 2,
		.dev.platform_data = &mpc86xx_fsl_i2c2_pdata,
		.num_resources  = 2,
		.resource = (struct resource[]) {
			{
				.start  = MPC86xx_IIC2_OFFSET,
				.end	= MPC86xx_IIC2_OFFSET +
						MPC86xx_IIC2_SIZE - 1,
				.flags  = IORESOURCE_MEM,
			},
			{
				.start  = MPC86xx_IRQ_IIC,
				.end	= MPC86xx_IRQ_IIC,
				.flags  = IORESOURCE_IRQ,
			}
		}
	}

};

static int __init mach_mpc86xx_fixup(struct platform_device *pdev)
{
	ppc_sys_fixup_mem_resource(pdev, CCSRBAR);
	return 0;
}

static int __init mach_mpc86xx_init(void)
{
	ppc_sys_device_fixup = mach_mpc86xx_fixup;
	return 0;
}

postcore_initcall(mach_mpc86xx_init);
