/*
 * arch/ppc/platforms/83xx/mpc83xx_devices.c
 *
 * MPC83xx Device descriptions
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
#include <linux/serial_8250.h>
#include <linux/fsl_devices.h>
#include <linux/cfide.h>
#include <asm/mpc83xx.h>
#include <asm/irq.h>
#include <asm/ppc_sys.h>

static u64 mpc83xx_dma_mask = 0xffffffffULL;

/* We use offsets for IORESOURCE_MEM since we do not know at compile time
 * what IMMRBAR is, will get fixed up by mach_mpc83xx_fixup
 */

struct gianfar_mdio_data mpc83xx_mdio_pdata = {
};

static struct gianfar_platform_data mpc83xx_tsec1_pdata = {
	.device_flags = FSL_GIANFAR_DEV_HAS_GIGABIT |
	    FSL_GIANFAR_DEV_HAS_COALESCE | FSL_GIANFAR_DEV_HAS_RMON |
	    FSL_GIANFAR_DEV_HAS_MULTI_INTR,
};

static struct gianfar_platform_data mpc83xx_tsec2_pdata = {
	.device_flags = FSL_GIANFAR_DEV_HAS_GIGABIT |
	    FSL_GIANFAR_DEV_HAS_COALESCE | FSL_GIANFAR_DEV_HAS_RMON |
	    FSL_GIANFAR_DEV_HAS_MULTI_INTR,
};

static struct fsl_i2c_platform_data mpc83xx_fsl_i2c1_pdata = {
	.device_flags = FSL_I2C_DEV_SEPARATE_DFSRR,
};

static struct fsl_i2c_platform_data mpc83xx_fsl_i2c2_pdata = {
	.device_flags = FSL_I2C_DEV_SEPARATE_DFSRR,
};

/* Placeholder to be filled in by board code */
static struct fsl_usb2_platform_data mpc83xx_fsl_dr_pdata = {
};

/* Placeholder to be filled in by board code */
static struct fsl_usb2_platform_data mpc83xx_fsl_mph_pdata = {
};

static struct plat_serial8250_port serial_platform_data[] = {
	[0] = {
		.mapbase	= 0x4500,
		.irq		= MPC83xx_IRQ_UART1,
		.iotype		= UPIO_MEM,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.line		= 0,
	},
	[1] = {
		.mapbase	= 0x4600,
		.irq		= MPC83xx_IRQ_UART2,
		.iotype		= UPIO_MEM,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.line		= 1,
	},
	{ },
};

static struct ucc_geth_platform_data mpc83xx_ugeth1_pdata = {
	.device_flags = FSL_GIANFAR_DEV_HAS_GIGABIT,
	.phy_reg_addr = 0x2000,
};

static struct ucc_geth_platform_data mpc83xx_ugeth2_pdata = {
	.device_flags = FSL_UGETH_DEV_HAS_GIGABIT,
	.phy_reg_addr = 0x3000,
};

static struct ucc_geth_platform_data mpc83xx_ugeth3_pdata = {
	.device_flags = FSL_UGETH_DEV_HAS_GIGABIT,
	.phy_reg_addr = 0x2200,
};

static struct ucc_geth_platform_data mpc83xx_ugeth4_pdata = {
	.device_flags = FSL_UGETH_DEV_HAS_GIGABIT,
	.phy_reg_addr = 0x3200,
};

static struct cfide_platform_data cfide_pdata = {
	.byte_lanes_swapping	= 0,
	.regaddr_step		= 2,
};

struct platform_device ppc_sys_platform_devices[] = {
	[MPC83xx_TSEC1] = {
		.name = "fsl-gianfar",
		.id	= 1,
		.dev.platform_data = &mpc83xx_tsec1_pdata,
		.num_resources	 = 4,
		.resource = (struct resource[]) {
			{
				.start	= 0x24000,
				.end	= 0x24fff,
				.flags	= IORESOURCE_MEM,
			},
			{
				.name	= "tx",
				.start	= MPC83xx_IRQ_TSEC1_TX,
				.end	= MPC83xx_IRQ_TSEC1_TX,
				.flags	= IORESOURCE_IRQ,
			},
			{
				.name	= "rx",
				.start	= MPC83xx_IRQ_TSEC1_RX,
				.end	= MPC83xx_IRQ_TSEC1_RX,
				.flags	= IORESOURCE_IRQ,
			},
			{
				.name	= "error",
				.start	= MPC83xx_IRQ_TSEC1_ERROR,
				.end	= MPC83xx_IRQ_TSEC1_ERROR,
				.flags	= IORESOURCE_IRQ,
			},
		},
	},
	[MPC83xx_TSEC2] = {
		.name = "fsl-gianfar",
		.id	= 2,
		.dev.platform_data = &mpc83xx_tsec2_pdata,
		.num_resources	 = 4,
		.resource = (struct resource[]) {
			{
				.start	= 0x25000,
				.end	= 0x25fff,
				.flags	= IORESOURCE_MEM,
			},
			{
				.name	= "tx",
				.start	= MPC83xx_IRQ_TSEC2_TX,
				.end	= MPC83xx_IRQ_TSEC2_TX,
				.flags	= IORESOURCE_IRQ,
			},
			{
				.name	= "rx",
				.start	= MPC83xx_IRQ_TSEC2_RX,
				.end	= MPC83xx_IRQ_TSEC2_RX,
				.flags	= IORESOURCE_IRQ,
			},
			{
				.name	= "error",
				.start	= MPC83xx_IRQ_TSEC2_ERROR,
				.end	= MPC83xx_IRQ_TSEC2_ERROR,
				.flags	= IORESOURCE_IRQ,
			},
		},
	},
	[MPC83xx_IIC1] = {
		.name = "fsl-i2c",
		.id	= 1,
		.dev.platform_data = &mpc83xx_fsl_i2c1_pdata,
		.num_resources	 = 2,
		.resource = (struct resource[]) {
			{
				.start	= 0x3000,
				.end	= 0x30ff,
				.flags	= IORESOURCE_MEM,
			},
			{
				.start	= MPC83xx_IRQ_IIC1,
				.end	= MPC83xx_IRQ_IIC1,
				.flags	= IORESOURCE_IRQ,
			},
		},
	},
	[MPC83xx_IIC2] = {
		.name = "fsl-i2c",
		.id	= 2,
		.dev.platform_data = &mpc83xx_fsl_i2c2_pdata,
		.num_resources	 = 2,
		.resource = (struct resource[]) {
			{
				.start	= 0x3100,
				.end	= 0x31ff,
				.flags	= IORESOURCE_MEM,
			},
			{
				.start	= MPC83xx_IRQ_IIC2,
				.end	= MPC83xx_IRQ_IIC2,
				.flags	= IORESOURCE_IRQ,
			},
		},
	},
	[MPC83xx_DUART] = {
		.name = "serial8250",
		.id	= 0,
		.dev.platform_data = serial_platform_data,
	},
	[MPC83xx_SEC2] = {
		.name = "fsl-sec2",
		.id	= 1,
		.num_resources	 = 2,
		.resource = (struct resource[]) {
			{
				.start	= 0x30000,
				.end	= 0x3ffff,
				.flags	= IORESOURCE_MEM,
			},
			{
				.start	= MPC83xx_IRQ_SEC2,
				.end	= MPC83xx_IRQ_SEC2,
				.flags	= IORESOURCE_IRQ,
			},
		},
	},
	[MPC83xx_USB2_DR] = {
		.name = "fsl-usb2-dr",
		.id	= 1,
		.dev.platform_data = &mpc83xx_fsl_dr_pdata,
		.num_resources	 = 2,
		.dev.dma_mask	= &mpc83xx_dma_mask,
		.dev.coherent_dma_mask = 0xffffffffULL,
		.resource = (struct resource[]) {
			{
				.start	= 0x23000,
				.end	= 0x23fff,
				.flags	= IORESOURCE_MEM,
			},
			{
				.start	= MPC83xx_IRQ_USB2_DR,
				.end	= MPC83xx_IRQ_USB2_DR,
				.flags	= IORESOURCE_IRQ,
			},
		},
	},
	[MPC83xx_USB2_MPH] = {
		.name = "fsl-usb2-mph",
		.id	= 1,
		.num_resources	 = 2,
		.dev.platform_data = &mpc83xx_fsl_mph_pdata,
		.dev.dma_mask	= &mpc83xx_dma_mask,
		.dev.coherent_dma_mask = 0xffffffffULL,
		.resource = (struct resource[]) {
			{
				.start	= 0x22000,
				.end	= 0x22fff,
				.flags	= IORESOURCE_MEM,
			},
			{
				.start	= MPC83xx_IRQ_USB2_MPH,
				.end	= MPC83xx_IRQ_USB2_MPH,
				.flags	= IORESOURCE_IRQ,
			},
		},
	},
	[MPC83xx_QE_UCC1] = {
		.name = "fsl-ucc-geth",
		.id     = 0,
		.dev.platform_data = &mpc83xx_ugeth1_pdata,
		.num_resources   = 2,
		.resource = (struct resource[]) {
			{
				.start  = 0x12000,
				.end    = 0x121ff,
				.flags  = IORESOURCE_MEM,
			},
			{
				.name   = "ucc1-irq",
				.start  = MPC83xx_QE_IRQ_UCC1,
				.end    = MPC83xx_QE_IRQ_UCC1,
				.flags  = IORESOURCE_IRQ,
			},
		},
	},
	[MPC83xx_QE_UCC2] = {
		.name = "fsl-ucc-geth",
		.id	= 1,
		.dev.platform_data = &mpc83xx_ugeth2_pdata,
		.num_resources	 = 2,
		.resource = (struct resource[]) {
			{
				.start	= 0x13000,
				.end	= 0x131ff,
				.flags	= IORESOURCE_MEM,
			},
			{
				.name	= "ucc2-irq",
				.start	= MPC83xx_QE_IRQ_UCC2,
				.end	= MPC83xx_QE_IRQ_UCC2,
				.flags	= IORESOURCE_IRQ,
			},
		},
	},
	[MPC83xx_QE_UCC3] = {
		.name = "fsl-ucc-geth",
		.id	= 2,
		.dev.platform_data = &mpc83xx_ugeth3_pdata,
		.num_resources = 2,
		.resource = (struct resource[]) {
			{
				.start	= 0x102200,
				.end	= 0x1023ff,
				.flags	= IORESOURCE_MEM,
			},
			{
				.name	= "ucc3-irq",
				.start	= MPC83xx_QE_IRQ_UCC3,
				.end	= MPC83xx_QE_IRQ_UCC3,
				.flags	= IORESOURCE_IRQ,
			},
		},
	},
	[MPC83xx_QE_UCC4] = {
		.name = "fsl-ucc-geth",
		.id	= 3,
		.dev.platform_data = &mpc83xx_ugeth4_pdata,
		.num_resources = 2,
		.resource = (struct resource[]) {
			{
				.start	= 0x103200,
				.end	= 0x1033ff,
				.flags	= IORESOURCE_MEM,
			},
			{
				.name	= "ucc4-irq",
				.start	= MPC83xx_QE_IRQ_UCC4,
				.end	= MPC83xx_QE_IRQ_UCC4,
				.flags	= IORESOURCE_IRQ,
			},
		},
	},
	[MPC83xx_MDIO] = {
		.name = "fsl-gianfar_mdio",
		.id = 0,
		.dev.platform_data = &mpc83xx_mdio_pdata,
		.num_resources = 1,
		.resource = (struct resource[]) {
			{
				.start	= 0x24520,
				.end	= 0x2453f,
				.flags	= IORESOURCE_MEM,
			},
		},
	},
	[MPC83xx_MDIO_TSEC1] = {
		.name = "fsl-gianfar_mdio",
		.id = 1,
		.dev.platform_data = &mpc83xx_mdio_pdata,
		.num_resources = 2,
		.resource = (struct resource[]) {
			{
				.start	= 0x24520,
				.end	= 0x2453f,
				.flags	= IORESOURCE_MEM,
			},
			{
				.start	= MPC83xx_IRQ_EXT1,
				.end	= MPC83xx_IRQ_EXT1,
				.flags	= IORESOURCE_IRQ,
			},
		},
	},
	[MPC83xx_CFIDE] = {
		.name			= "mmio-cfide",
		.id			= 1,
		.dev.platform_data	= &cfide_pdata,
		.num_resources		= 3,
		.resource = (struct resource[]) {
			{
				.start  = 0x10000000,
				.end	= 0x1000000f,
				.flags	= IORESOURCE_MEM,
			},
			{
				.start  = 0x10000200, /* there is a self-contradiction
							in board description: either
							0x10000100 or 0x10000200 */
				.end	= 0x1000020f,
				.flags	= IORESOURCE_MEM,
			},
			{
				.start	= MPC83xx_IRQ_EXT7,
				.end	= MPC83xx_IRQ_EXT7,
				.flags	= IORESOURCE_IRQ,
			},
		},
	},
	[MPC83xx_QE_SPI1] = {
		.name = "fsl-qe-spi1",
		.id	= 1,
		.num_resources	 = 2,
		.resource = (struct resource[]) {
			{
				.start	= 0x1004c0,
				.end	= 0x1004ff,
				.flags	= IORESOURCE_MEM,
			},
			{
				.name	= "qe-spi1-irq",
				.start	= MPC83xx_QE_IRQ_SPI1,
				.end	= MPC83xx_QE_IRQ_SPI1,
				.flags	= IORESOURCE_IRQ,
			},
		},
	},
	[MPC83xx_QE_SPI2] = {
		.name = "fsl-qe-spi2",
		.id	= 1,
		.num_resources	 = 2,
		.resource = (struct resource[]) {
			{
				.start	= 0x100500,
				.end	= 0x10053f,
				.flags	= IORESOURCE_MEM,
			},
			{
				.name	= "qe-spi2-irq",
				.start	= MPC83xx_QE_IRQ_SPI2,
				.end	= MPC83xx_QE_IRQ_SPI2,
				.flags	= IORESOURCE_IRQ,
			},
		},
	},
	[MPC83xx_QE_USB] = {
		.name = "fsl-qe-usb",
		.id     = 0,
		.num_resources   = 2,
		.resource = (struct resource[]) {
			{
				.start  = 0x1006c0,
				.end    = 0x1006ff,
				.flags  = IORESOURCE_MEM,
			},
			{
				.name   = "usb-irq",
				.start  = MPC83xx_QE_IRQ_USB,
				.end    = MPC83xx_QE_IRQ_USB,
				.flags  = IORESOURCE_IRQ,
			},
		},
	},
};

static int __init mach_mpc83xx_fixup(struct platform_device *pdev)
{
	ppc_sys_fixup_mem_resource(pdev, immrbar);
	return 0;
}

static int __init mach_mpc83xx_init(void)
{
	if (ppc_md.progress)
		ppc_md.progress("mach_mpc83xx_init:enter", 0);
	ppc_sys_device_fixup = mach_mpc83xx_fixup;
	return 0;
}

postcore_initcall(mach_mpc83xx_init);
