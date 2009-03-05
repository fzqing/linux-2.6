/*
 * linux/arch/arm/mach-davinci/board-evm.c
 *
 * TI DaVinci EVM board
 *
 * Copyright (C) 2006 Texas Instruments.
 *
 * ----------------------------------------------------------------------------
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
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
#include <linux/usb_musb.h>
#include <linux/mtd/nand.h>
#include <linux/serial_8250.h>
#include <linux/davinci_mmc.h>
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
#define DAVINCI_UART_CLK		27000000

static struct plat_serial8250_port serial_platform_data[] = {
	{
		.membase	= (char *)IO_ADDRESS(DAVINCI_UART0_BASE),
		.mapbase	= (unsigned long)DAVINCI_UART0_BASE,
		.irq		= IRQ_UARTINT0,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM32,
		.regshift	= 2,
		.uartclk	= DAVINCI_UART_CLK,
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
		.start		= IRQ_USBINT,
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
int cpu_type(void)
{
	return MACH_TYPE_DAVINCI_EVM;
}

extern void davinci_serial_init(struct platform_device *pdev);

#if defined (CONFIG_MTD_NAND_DAVINCI) || defined(CONFIG_MTD_NAND_DAVINCI_MODULE)
static struct mtd_partition nand_partitions[] = {
	/* bootloader (U-Boot, etc) in first sector */
	{
		.name		= "bootloader",
		.offset		= 0,
		.size		= SZ_256K,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	/* bootloader params in the next sector */
	{
		.name		= "params",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_128K,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	/* bootloader logo */
	{
		.name		= "logo",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 0x80000,//512K
		.mask_flags	= 0,
	},
	/* kernel */
	{
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_4M,
		.mask_flags	= 0,
	},
	/* progma */
	{
		.name		= "progma",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 0x01E00000,//30M
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

//NAND_ECC_SOFT
static struct nand_davinci_platform_data nand_data = {
	.options		= 0,
	.eccmode		= NAND_ECC_HW3_512,
	.cle_mask		= 0x10,
	.ale_mask		= 0x08,
	.bbt_td			= NULL,
	.bbt_md			= NULL,
	.parts			= nand_partitions,
	.nr_parts		= ARRAY_SIZE(nand_partitions),
};

static struct resource nand_resources[]		= {
	[0] = {		/* First memory resource is AEMIF control registers */
		.start		= DM644X_ASYNC_EMIF_CNTRL_BASE,
		.end		= DM644X_ASYNC_EMIF_CNTRL_BASE + SZ_4K - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {		/* Second memory resource is NAND I/O window */
		.start		= DAVINCI_ASYNC_EMIF_DATA_CE0_BASE,
		.end		= DAVINCI_ASYNC_EMIF_DATA_CE0_BASE + SZ_16K - 1,
		.flags		= IORESOURCE_MEM,
	},
};

static struct platform_device nand_device = {
	.name			= "nand_davinci",
	.id			= 0,
	.dev			= {
		.platform_data	= &nand_data,
	},

	.num_resources		= ARRAY_SIZE(nand_resources),
	.resource		= nand_resources,
};
#endif

#if defined (CONFIG_MTD_DAVINCI_NOR) || defined(CONFIG_MTD_DAVINCI_NOR_MODULE)
static struct mtd_partition davinci_evm_nor_partitions[] = {
	/* bootloader (U-Boot, etc) in first 4 sectors */
	{
		.name		= "bootloader",
		.offset		= 0,
		.size		= SZ_128K,
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
	},
	/* bootloader params in the next 1 sector */
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
	.map_name		= "cfi_probe",
	.width			= 2,
	.parts			= davinci_evm_nor_partitions,
	.nr_parts		= ARRAY_SIZE(davinci_evm_nor_partitions),
};

/* NOTE: CFI probe will correctly detect flash part as 32M, but EMIF
 * limits addresses to 16M, so using addresses past 16M will wrap */
static struct resource davinci_evm_flash_resource = {
	.start			= DAVINCI_ASYNC_EMIF_DATA_CE0_BASE,
	.end			= DAVINCI_ASYNC_EMIF_DATA_CE0_BASE + SZ_16M - 1,
	.flags			= IORESOURCE_MEM,
};

static struct platform_device davinci_evm_flash_device = {
	.name			= "davinciflash",
	.id			= 0,
	.dev			= {
		.platform_data	= &davinci_evm_flash_data,
	},

	.num_resources		= 1,
	.resource		= &davinci_evm_flash_resource,
};
#endif

#if defined(CONFIG_MMC_DAVINCI) || defined(CONFIG_MMC_DAVINCI_MODULE)
static struct resource mmc0_resources[] = {
	[0] = {			/* registers */
		.start		= DAVINCI_MMC_SD_BASE,
		.end		= DAVINCI_MMC_SD_BASE + SZ_1K - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {			/* interrupt */
		.start		= IRQ_MMCINT,
		.end		= IRQ_MMCINT,
		.flags		= IORESOURCE_IRQ,
	},
	[2] = {			/* dma rx */
		.start		= DAVINCI_DMA_MMCRXEVT,
		.end		= DAVINCI_DMA_MMCRXEVT,
		.flags		= IORESOURCE_DMA,
	},
	[3] = {			/* dma tx */
		.start		= DAVINCI_DMA_MMCTXEVT,
		.end		= DAVINCI_DMA_MMCTXEVT,
		.flags		= IORESOURCE_DMA,
	},
};

static struct davinci_mmc_platform_data mmc0_platform_data = {
	.mmc_clk		= "MMCSDCLK0",
	.rw_threshold		= 32,
	.use_4bit_mode		= 1,
};

static struct platform_device mmc0_device = {
	.name			= "mmc",
	.id			= 0,
	.dev			= {
		.platform_data	= &mmc0_platform_data,
	},

	.num_resources		= ARRAY_SIZE(mmc0_resources),
	.resource		= mmc0_resources,
};

static void setup_mmc(void)
{
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_MMC_SD0, 1);
}
#else
#define setup_mmc()
#endif

static struct platform_device *davinci_evm_devices[] __initdata = {
	&serial_device,
	&usb_dev,
#if defined (CONFIG_MTD_NAND_DAVINCI) || defined(CONFIG_MTD_NAND_DAVINCI_MODULE)
	&nand_device,
#endif

#if defined (CONFIG_MTD_DAVINCI_NOR) || defined(CONFIG_MTD_DAVINCI_NOR_MODULE)
	&davinci_evm_flash_device,
#endif

#if defined(CONFIG_MMC_DAVINCI) || defined(CONFIG_MMC_DAVINCI_MODULE)
	&mmc0_device,
#endif
};

/* FIQ are pri 0-1; otherwise 2-7, with 7 lowest priority */
static const u8 dm644x_default_priorities[DAVINCI_N_AINTC_IRQ] = {
	[IRQ_VDINT0]		= 2,
	[IRQ_VDINT1]		= 6,
	[IRQ_VDINT2]		= 6,
	[IRQ_HISTINT]		= 6,
	[IRQ_H3AINT]		= 6,
	[IRQ_PRVUINT]		= 6,
	[IRQ_RSZINT]		= 6,
	[IRQ_VFOCINT]		= 7,
	[IRQ_VENCINT]		= 6,
	[IRQ_ASQINT]		= 6,
	[IRQ_IMXINT]		= 6,
	[IRQ_VLCDINT]		= 6,
	[IRQ_USBINT]		= 4,
	[IRQ_EMACINT]		= 4,
	[IRQ_IEEE1394INT]	= 7,
	[IRQ_IEEE1394WK]	= 7,
	[IRQ_CCINT0]		= 5,	/* dma */
	[IRQ_CCERRINT]		= 5,	/* dma */
	[IRQ_TCERRINT0]		= 5,	/* dma */
	[IRQ_TCERRINT]		= 5,	/* dma */
	[IRQ_PSCINT]		= 7,
	[21]			= 7,
	[IRQ_IDE]		= 4,
	[IRQ_HPIINT]		= 7,
	[IRQ_MBXINT]		= 7,
	[IRQ_MBRINT]		= 7,
	[IRQ_MMCINT]		= 7,
	[IRQ_SDIOINT]		= 7,
	[IRQ_HPIINT]		= 7,
	[IRQ_DDRINT]		= 7,
	[IRQ_AEMIFINT]		= 7,
	[IRQ_VLQINT]		= 4,
	[IRQ_TINT0_TINT12]	= 2,	/* clockevent */
	[IRQ_TINT0_TINT34]	= 2,	/* clocksource */
	[IRQ_TINT1_TINT12]	= 7,	/* DSP timer */
	[IRQ_TINT1_TINT34]	= 7,	/* system tick */
	[IRQ_PWMINT0]		= 7,
	[IRQ_PWMINT1]		= 7,
	[IRQ_PWMINT2]		= 7,
	[IRQ_I2C]		= 3,
	[IRQ_UARTINT0]		= 3,
	[IRQ_UARTINT1]		= 3,
	[IRQ_UARTINT2]		= 3,
	[IRQ_SPINT0]		= 3,
	[IRQ_SPINT1]		= 3,
	[45]			= 7,
	[IRQ_DSP2ARM0]		= 4,
	[IRQ_DSP2ARM1]		= 4,
	[IRQ_GPIO0]		= 7,
	[IRQ_GPIO1]		= 7,
	[IRQ_GPIO2]		= 7,
	[IRQ_GPIO3]		= 7,
	[IRQ_GPIO4]		= 7,
	[IRQ_GPIO5]		= 7,
	[IRQ_GPIO6]		= 7,
	[IRQ_GPIO7]		= 7,
	[IRQ_GPIOBNK0]		= 7,
	[IRQ_GPIOBNK1]		= 7,
	[IRQ_GPIOBNK2]		= 7,
	[IRQ_GPIOBNK3]		= 7,
	[IRQ_GPIOBNK4]		= 7,
	[IRQ_COMMTX]		= 7,
	[IRQ_COMMRX]		= 7,
	[IRQ_EMUINT]		= 7,
};
static void board_init(void)
{
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_VLYNQ, 1);

	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_VPSSMSTR, 1);
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_VPSSSLV, 1);
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_TPCC, 1);
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_TPTC0, 1);
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_TPTC1, 1);
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_GPIO, 1);

	/* Turn on WatchDog timer LPSC.  Needed for RESET to work */
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_TIMER2, 1);

	davinci_serial_init(&serial_device);
}

static void dm644x_setup_pinmux(unsigned int id)
{
	switch (id) {
	case DAVINCI_LPSC_ATA:
		davinci_cfg_reg(DM644X_HDIREN);
		davinci_cfg_reg(DM644X_ATAEN);
		break;
	case DAVINCI_LPSC_MMC_SD0:
		/* VDD power manipulations are done in U-Boot for CPMAC
		* so applies to MMC as well
		*/
		/*Set up the pull register for MMC */
		DAVINCI_VDD3P3V_PWDN = 0x0;
		davinci_cfg_reg(DM644X_MSTK);
		break;
	case DAVINCI_LPSC_I2C:
		davinci_cfg_reg(DM644X_I2C);
		break;
	case DAVINCI_LPSC_McBSP0:
		davinci_cfg_reg(DM644X_MCBSP0);
		break;
	case DAVINCI_LPSC_PWM0:
		davinci_cfg_reg(DM644X_PWM0);
		break;
	case DAVINCI_LPSC_PWM1:
		davinci_cfg_reg(DM644X_PWM1);
		break;
	case DAVINCI_LPSC_PWM2:
		davinci_cfg_reg(DM644X_PWM2);
		break;
	case DAVINCI_LPSC_VLYNQ:
		davinci_cfg_reg(DM644X_VLINQEN);
		davinci_cfg_reg(DM644X_VLINQWD);
		break;
	default:
		break;

	}
}

extern const u8 *davinci_def_priorities;

static void __init davinci_map_io(void)
{
	davinci_pinmux_setup = dm644x_setup_pinmux;
	davinci_def_priorities = dm644x_default_priorities;
	davinci_map_common_io();

#ifdef CONFIG_KGDB_8250
	early_serial_setup((struct uart_port *)
			   &serial_platform_data[kgdb8250_ttyS]);
	kgdb8250_add_platform_port(kgdb8250_ttyS,
				   &serial_platform_data[kgdb8250_ttyS]);
#endif
	/* Initialize the DaVinci EVM board settigs */
	board_init ();
}

int __init davinci_gpio_irq_setup(void);

void davinci_msp430_deep_sleep(void)
{
	unsigned int icstr = DAVINCI_I2C_BASE + 0x08;
	unsigned int icsar = DAVINCI_I2C_BASE + 0x1C;
	unsigned int iccnt = DAVINCI_I2C_BASE + 0x14;
	unsigned int icdxr = DAVINCI_I2C_BASE + 0x20;
	unsigned int icmdr = DAVINCI_I2C_BASE + 0x24;
	u32 cnt = 0, buflen = 2;
	char rtcdata[2] = { 2, 8 };
	char *buffer = rtcdata;

	/* check for bus busy */
	while (readl(icstr) & 0x1000) ;

	/* configure the count register */
	writel(2, iccnt);

	/* set the salve address */
	writel(0x23, icsar);

	/* Take I2C out of reset, configure it as master,
	 * set the start bit, stop bit and enable the
	 * transmitter */
	writel(0x2e20, icmdr);

	while (cnt < buflen) {
		if ((readl(icstr) & 0x0010) != 0) {
			writel(*buffer, icdxr);
			++buffer;
			++cnt;
		}
	}
}


static __init void evm_init(void)
{
#if defined (CONFIG_MTD_DAVINCI_NOR) || defined(CONFIG_MTD_DAVINCI_NOR_MODULE)
#if defined(CONFIG_BLK_DEV_DAVINCI) || defined(CONFIG_BLK_DEV_DAVINCI_MODULE)
#warning IDE and NOR flash are are pin-muxed. Disable IDE or NOR.
	printk(KERN_WARNING "WARNING: both IDE and NOR flash are enabled, "
		      "but are pin-muxed.\n\t Disable IDE or NOR support.\n");
#endif
#if defined (CONFIG_MTD_NAND_DAVINCI) || defined(CONFIG_MTD_NAND_DAVINCI_MODULE)
#warning NAND and NOR flash are are pin-muxed. Disable NAND or NOR.
	printk(KERN_WARNING "WARNING: both NAND and NOR flash are enabled, "
		      "but are pin-muxed.\n\t Disable NAND or NOR support.\n");
#endif
#endif

#if defined(CONFIG_BLK_DEV_DAVINCI) || defined(CONFIG_BLK_DEV_DAVINCI_MODULE)
#if defined (CONFIG_MTD_NAND_DAVINCI) || defined(CONFIG_MTD_NAND_DAVINCI_MODULE)
#warning IDE and NAND flash are are pin-muxed. Disable IDE or NAND.
	printk(KERN_WARNING "WARNING: both IDE and NAND flash are enabled, "
		      "but are pin-muxed.\n\t Disable IDE or NAND support.\n");
#endif
#endif
	pm_power_off = NULL;//davinci_msp430_deep_sleep;
	setup_mmc();
	davinci_gpio_irq_setup();
	platform_add_devices(davinci_evm_devices,
		ARRAY_SIZE(davinci_evm_devices));
}


extern void davinci_irq_init(void);
extern struct sys_timer davinci_timer;

MACHINE_START(DAVINCI_EVM, "DaVinci EVM")
    MAINTAINER("Texas Instruments, PSP Team")
    BOOT_MEM(DAVINCI_DDR_BASE, IO_PHYS, IO_VIRT)
    BOOT_PARAMS(0x80000100)
    MAPIO(davinci_map_io)
    INITIRQ(davinci_irq_init)
    .timer = &davinci_timer,
    INIT_MACHINE(evm_init)
    MACHINE_END
EXPORT_SYMBOL(cpu_type);
