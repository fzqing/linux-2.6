/*
 * TI DaVinci DM6467 EVM board
 *
 * Derived from: arch/arm/mach-davinci/board-evm.c
 * Copyright (C) 2006 Texas Instruments.
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 */

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/major.h>
#include <linux/root_dev.h>
#include <linux/dma-mapping.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/serial.h>
#include <linux/mtd/nand.h>
#include <linux/serial_8250.h>
#include <linux/usb_musb.h>
#include <linux/nand_davinci.h>

#include <asm/setup.h>
#include <asm/io.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/arch/irqs.h>
#include <asm/arch/hardware.h>
#include <asm/arch/edma.h>
#include <linux/kgdb.h>
#include <asm/arch/cpu.h>
#include <asm/arch/mux.h>
#include "clock.h"

/**************************************************************************
 * Definitions
 **************************************************************************/
#define DAVINCI_DM646X_UART_CLK		24000000

static struct plat_serial8250_port serial_platform_data[] = {
	{
		.membase	= (char *)IO_ADDRESS(DAVINCI_UART0_BASE),
		.mapbase	= (unsigned long)DAVINCI_UART0_BASE,
		.irq		= IRQ_UARTINT0,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM32,
		.regshift	= 2,
		.uartclk	= DAVINCI_DM646X_UART_CLK,
	},
	{
		.membase	= (char *)IO_ADDRESS(DAVINCI_UART1_BASE),
		.mapbase	= (unsigned long)DAVINCI_UART1_BASE,
		.irq		= IRQ_UARTINT1,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM32,
		.regshift	= 2,
		.uartclk	= DAVINCI_DM646X_UART_CLK,
	},
	{
		.membase	= (char *)IO_ADDRESS(DM644X_UART2_BASE),
		.mapbase	= (unsigned long)DM644X_UART2_BASE,
		.irq		= IRQ_UARTINT2,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM32,
		.regshift	= 2,
		.uartclk	= DAVINCI_DM646X_UART_CLK,
	},
	{
		.flags	= 0,
	},
};

static struct platform_device serial_device	= {
	.name			= "serial8250",
	.id			= 0,
	.dev			= {
		.platform_data	= serial_platform_data,
	},
};

static struct musb_hdrc_platform_data usb_data[] = {
	{
#if defined(CONFIG_USB_MUSB_OTG)
		/* OTG requires a Mini-AB connector */
		.mode		= MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_PERIPHERAL)
		.mode		= MUSB_PERIPHERAL,
#elif defined(CONFIG_USB_MUSB_HOST)
		.mode		= MUSB_HOST,
#endif
		.set_vbus	= NULL,
		/* irlml6401 switches 5V */
		.power		= 255,           /* sustains 3.0+ Amps (!) */
		.potpgt		= 4,            /* ~8 msec */
		.multipoint	= 1,
	},		/* Need Multipoint support */
};

static struct resource usb_resources[] = {
	{
		/* physical address */
		.start		= DAVINCI_USB_OTG_BASE,
		.end		= DAVINCI_USB_OTG_BASE + 0x5ff,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= IRQ_DM646X_USBINT,
		.flags		= IORESOURCE_IRQ,
	},
	{
		.start		= IRQ_DM646X_USBDMAINT,
		.flags		= IORESOURCE_IRQ,
	},
};

static u64 usb_dmamask = DMA_32BIT_MASK;

static struct platform_device usb_dev = {
	.name		= "musb_hdrc",
	.id		= -1,
	.dev		= {
		.platform_data = usb_data,
		.dma_mask = &usb_dmamask,
		.coherent_dma_mask = DMA_32BIT_MASK,
	},
	.resource	= usb_resources,
	.num_resources	= ARRAY_SIZE(usb_resources),
};

/**************************************************************************
 * Public Functions
 **************************************************************************/
extern void davinci_serial_init(struct platform_device *pdev);

#if defined (CONFIG_MTD_NAND_DAVINCI) || defined(CONFIG_MTD_NAND_DAVINCI_MODULE)
static struct mtd_partition dm646x_nand_partitions[] = {
	/* bootloader (U-Boot, etc) in first sector */
	{
		.name		= "bootloader",
		.offset		= 0,
		.size		= SZ_512K,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	/* bootloader params in the next sector */
	{
		.name		= "params",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_128K,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	/* kernel */
	{
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_4M,
		.mask_flags		= 0,
	},
	/* file system */
	{
		.name		= "filesystem",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags		= 0,
	}
};

static struct nand_davinci_platform_data dm646x_nand_data = {
	.options		= 0,
	.eccmode		= NAND_ECC_HW3_512,
	.cle_mask		= 0x80000,
	.ale_mask		= 0x40000,
	.parts			= dm646x_nand_partitions,
	.nr_parts		= ARRAY_SIZE(dm646x_nand_partitions),
};

static struct resource dm646x_nand_resources[] = {
	[0] = {		/* First memory resource is AEMIF control registers */
		.start		= DAVINCI_DM646X_ASYNC_EMIF_CNTRL_BASE,
		.end		= DAVINCI_DM646X_ASYNC_EMIF_CNTRL_BASE +
					SZ_16K - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {		/* Second memory resource is NAND I/O window */
		.start		= DAVINCI_DM646X_ASYNC_EMIF_DATA_CE0_BASE,
		.end		= DAVINCI_DM646X_ASYNC_EMIF_DATA_CE0_BASE +
					SZ_512K + 2 * SZ_1K - 1,
		.flags		= IORESOURCE_MEM,
	},
};

static struct platform_device nand_device = {
	.name			= "nand_davinci",
	.id			= 0,
	.dev			= {
		.platform_data	= &dm646x_nand_data,
	},

	.num_resources		= ARRAY_SIZE(dm646x_nand_resources),
	.resource		= dm646x_nand_resources,
};
#endif

#if defined (CONFIG_MTD_CFI_INTELEXT) || defined(CONFIG_MTD_AMDSTD)
static struct mtd_partition davinci_evm_nor_partitions[] = {
	/* bootloader (U-Boot, etc) in first 4 sectors */
	{
		.name		= "bootloader",
		.offset		= 0,
		.size		= 2 * SZ_64K,
		.mask_flags	= 0,
	},
	/* bootloader params in the next 1 sector */
	{
		.name		= "params",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 2 * SZ_64K,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	/* kernel */
	{
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_4M,
		.mask_flags	= 0,
	},
	/* file system */
	{
		.name		= "filesystem",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	}
};

static struct flash_platform_data davinci_evm_flash_data = {
	.parts			= davinci_evm_nor_partitions,
	.nr_parts		= ARRAY_SIZE(davinci_evm_nor_partitions),
};

static struct resource davinci_evm_flash_resource = {
	.start			= DAVINCI_DM646X_ASYNC_EMIF_DATA_CE0_BASE,
	.end			= DAVINCI_DM646X_ASYNC_EMIF_DATA_CE0_BASE +
					SZ_64K - 1,
	.flags			= IORESOURCE_MEM,
};

static struct platform_device davinci_evm_flash_device = {
	.name			= "nor_davinci",
	.id			= 0,
	.dev			= {
		.platform_data	= &davinci_evm_flash_data,
	},

	.num_resources		= 1,
	.resource		= &davinci_evm_flash_resource,
};
#endif

static struct platform_device *davinci_evm_devices[] __initdata = {
	&serial_device,
	&usb_dev,
#if defined (CONFIG_MTD_NAND_DAVINCI) || defined(CONFIG_MTD_NAND_DAVINCI_MODULE)
	&nand_device,
#endif

#if defined (CONFIG_MTD_CFI_INTELEXT) || defined(CONFIG_MTD_AMDSTD)
	&davinci_evm_flash_device,
#endif
};

/* FIQ are pri 0-1; otherwise 2-7, with 7 lowest priority */
static const u8 dm646x_default_priorities[DAVINCI_N_AINTC_IRQ] = {
	[IRQ_DM646X_VP_VERTINT0]	= 2,
	[IRQ_DM646X_VP_VERTINT1]	= 6,
	[IRQ_DM646X_VP_VERTINT2]	= 6,
	[IRQ_DM646X_VP_VERTINT3]	= 6,
	[IRQ_DM646X_VP_ERRINT]		= 6,
	[IRQ_DM646X_RESERVED_1]		= 7,
	[IRQ_DM646X_RESERVED_2]		= 7,
	[IRQ_DM646X_WDINT]		= 7,
	[IRQ_DM646X_CRGENINT0]		= 6,
	[IRQ_DM646X_CRGENINT1]		= 6,
	[IRQ_DM646X_TSIFINT0]		= 6,
	[IRQ_DM646X_TSIFINT1]		= 6,
	[IRQ_DM646X_VDCEINT]		= 4,
	[IRQ_DM646X_USBINT]		= 4,
	[IRQ_DM646X_USBDMAINT]		= 7,
	[IRQ_DM646X_PCIINT]		= 7,
	[IRQ_CCINT0]			= 5,	/* dma */
	[IRQ_CCERRINT]			= 5,	/* dma */
	[IRQ_TCERRINT0]			= 5,	/* dma */
	[IRQ_TCERRINT]			= 5,	/* dma */
	[IRQ_DM646X_TCERRINT2]		= 7,
	[IRQ_DM646X_TCERRINT3]		= 7,
	[IRQ_DM646X_IDE]		= 4,
	[IRQ_DM646X_HPIINT]		= 7,
	[IRQ_DM646X_EMACRXTHINT]	= 7,
	[IRQ_DM646X_EMACRXINT]		= 7,
	[IRQ_DM646X_EMACTXINT]		= 7,
	[IRQ_DM646X_EMACMISCINT]	= 7,
	[IRQ_DM646X_MCASP0TXINT]	= 7,
	[IRQ_DM646X_MCASP0RXINT]	= 7,
	[IRQ_AEMIFINT]			= 7,
	[IRQ_DM646X_RESERVED_3]		= 4,
	[IRQ_DM646X_MCASP1TXINT]	= 2,	/* clockevent */
	[IRQ_TINT0_TINT34]		= 2,	/* clocksource */
	[IRQ_TINT1_TINT12]		= 7,	/* DSP timer */
	[IRQ_TINT1_TINT34]		= 7,	/* system tick */
	[IRQ_PWMINT0]			= 7,
	[IRQ_PWMINT1]			= 7,
	[IRQ_DM646X_VLQINT]		= 7,
	[IRQ_I2C]			= 3,
	[IRQ_UARTINT0]			= 3,
	[IRQ_UARTINT1]			= 3,
	[IRQ_DM646X_UARTINT2]		= 3,
	[IRQ_DM646X_SPINT0]		= 3,
	[IRQ_DM646X_SPINT1]		= 3,
	[IRQ_DM646X_DSP2ARMINT]		= 7,
	[IRQ_DM646X_RESERVED_4]		= 7,
	[IRQ_DM646X_PSCINT]		= 4,
	[IRQ_DM646X_GPIO0]		= 7,
	[IRQ_DM646X_GPIO1]		= 7,
	[IRQ_DM646X_GPIO2]		= 7,
	[IRQ_DM646X_GPIO3]		= 7,
	[IRQ_DM646X_GPIO4]		= 7,
	[IRQ_DM646X_GPIO5]		= 7,
	[IRQ_DM646X_GPIO6]		= 7,
	[IRQ_DM646X_GPIO7]		= 7,
	[IRQ_DM646X_GPIOBNK0]		= 7,
	[IRQ_DM646X_GPIOBNK1]		= 7,
	[IRQ_DM646X_GPIOBNK2]		= 7,
	[IRQ_DM646X_DDRINT]		= 7,
	[IRQ_DM646X_AEMIFINT]		= 7,
	[IRQ_COMMTX]			= 7,
	[IRQ_COMMRX]			= 7,
	[IRQ_EMUINT]			= 7,
};
static void board_init(void)
{
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_VLYNQ, 1);
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_DM646X_LPSC_HDVICP0, 1);
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_DM646X_LPSC_HDVICP1, 1);
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_DM646X_LPSC_SPI, 1);
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_DM646X_LPSC_TPCC, 1);
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_DM646X_LPSC_TPTC0, 1);
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_DM646X_LPSC_TPTC1, 1);
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_DM646X_LPSC_TPTC2, 1);
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_DM646X_LPSC_TPTC3, 1);
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_DM646X_LPSC_AEMIF, 1);
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_DM646X_LPSC_GPIO, 1);
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_DM646X_LPSC_TSIF0, 1);
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_DM646X_LPSC_TSIF1, 1);

	davinci_serial_init(&serial_device);
}

static void dm646x_setup_pinmux(unsigned int id)
{
	switch (id) {
	case DAVINCI_LPSC_ATA:
		davinci_cfg_reg(DM646X_ATAEN);
		break;
	case DAVINCI_LPSC_USB:
		davinci_cfg_reg(DM646X_VBUSDIS);
		DAVINCI_VDD3P3V_PWDN &= 0xEFFFFFFF;
		break;
	case DAVINCI_DM646X_LPSC_I2C:
		break;
	case DAVINCI_DM646X_LPSC_PWM0:
		break;
	case DAVINCI_DM646X_LPSC_PWM1:
		break;
	default:
		break;
	}
}

extern const u8 *davinci_def_priorities;

static void __init davinci_map_io(void)
{
	davinci_pinmux_setup = dm646x_setup_pinmux;
	davinci_def_priorities = dm646x_default_priorities;
	davinci_map_common_io();

#ifdef CONFIG_KGDB_8250
	early_serial_setup((struct uart_port *)
			   &serial_platform_data[kgdb8250_ttyS]);
	kgdb8250_add_platform_port(kgdb8250_ttyS,
				   &serial_platform_data[kgdb8250_ttyS]);
#endif
	/* Initialize the DaVinci EVM board settigs */
	board_init();
}

int __init davinci_gpio_irq_setup(void);

static __init void evm_init(void)
{
	davinci_gpio_irq_setup();

	platform_add_devices(davinci_evm_devices,
		ARRAY_SIZE(davinci_evm_devices));
}

extern void davinci_irq_init(void);
extern struct sys_timer davinci_timer;

MACHINE_START(DAVINCI_EVM, "DaVinci DM6467 EVM")
    MAINTAINER("Texas Instruments, PSP Team")
    BOOT_MEM(DAVINCI_DDR_BASE, IO_PHYS, IO_VIRT)
    BOOT_PARAMS(0x80000100)
    MAPIO(davinci_map_io)
    INITIRQ(davinci_irq_init)
    .timer = &davinci_timer,
    INIT_MACHINE(evm_init)
    MACHINE_END
