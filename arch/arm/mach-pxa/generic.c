/*
 *  linux/arch/arm/mach-pxa/generic.c
 *
 *  Author:	Nicolas Pitre
 *  Created:	Jun 15, 2001
 *  Copyright:	MontaVista Software Inc.
 *
 * Code common to all PXA machines.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Since this file should be linked before any other machine specific file,
 * the __initcall() here will be executed first.  This serves as default
 * initialization stuff for PXA machines which can be overridden later if
 * need be.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/pm.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/mach/map.h>

#include <asm/arch/pxa-regs.h>
#include <asm/arch/udc.h>
#include <asm/arch/pxafb.h>
#include <asm/arch/mmc.h>
#include <asm/arch/irda.h>

#include "generic.h"

/*
 * Handy function to set GPIO alternate functions
 */

void pxa_gpio_mode(int gpio_mode)
{
	unsigned long flags;
	int gpio = gpio_mode & GPIO_MD_MASK_NR;
	int fn = (gpio_mode & GPIO_MD_MASK_FN) >> 8;
	int gafr;

	local_irq_save(flags);
#ifndef CONFIG_PXA3xx
	if (gpio_mode & GPIO_DFLT_LOW)
		GPCR(gpio) = GPIO_bit(gpio);
	else if (gpio_mode & GPIO_DFLT_HIGH)
		GPSR(gpio) = GPIO_bit(gpio);
	if (gpio_mode & GPIO_MD_MASK_DIR)
		GPDR(gpio) |= GPIO_bit(gpio);
	else
		GPDR(gpio) &= ~GPIO_bit(gpio);
	gafr = GAFR(gpio) & ~(0x3 << (((gpio) & 0xf)*2));
	GAFR(gpio) = gafr |  (fn  << (((gpio) & 0xf)*2));
#else
        GCDR(gpio) &= (1 << (gpio & 0x1f));
#endif
	local_irq_restore(flags);
}

EXPORT_SYMBOL(pxa_gpio_mode);

/*
 * Routine to safely enable or disable a clock in the CKEN
 */
void pxa_set_cken(int clock, int enable)
{
	unsigned long flags;
#if defined(CONFIG_PXA3xx)
	int hsio2_enable = 0;
#endif

	local_irq_save(flags);

#if defined(CONFIG_PXA3xx)
	switch(clock) {
	/*special case1:*/
		case CKEN_AC97:
			if (enable) {
				CKENA |= (0x1u << clock);
				if (CKEN_AC97 == clock) {
				/*
				 * FIXME: Make out-clock for AC97 24.576MHz
				 * This is workaround for that AC97 clock is
				 * not correct after reset.
				 */
					AC97_DIV = 1625<<12 | 128;
				}
			} else {
				CKENA &= ~(0x1u << clock);
			}
			break;

	/*special case2:*/
		case CKEN_USB2:
			if (enable) {
				CKENA |= (0x1u << clock);
			} else {
				CKENA &= ~(0x1u << clock);
			}
			/* down */
		case CKEN_GRAPHICS:
			if (enable) {
				CKENB |= (0x1u << (clock - 32));
			} else {
				CKENB &= ~(0x1u << (clock - 32));
			}
			/* down */
#ifdef CONFIG_PXA310
		case CKEN_MVED:
			if (enable) {
				CKENB |= (0x1u << (clock - 32));
			} else {
				CKENB &= ~(0x1u << (clock - 32));
			}
			/* down */
#endif
		case CKEN_HSIO2:
			if (enable)
				hsio2_enable++;
			if(CKENA & (0x1u << CKEN_USB2))
				hsio2_enable++;
			if (CKENB & (0x1u << (CKEN_GRAPHICS - 32)))
				hsio2_enable++;
#ifdef CONFIG_PXA310
			if(CKENB & (0x1u << (CKEN_MVED - 32)))
				hsio2_enable++;
#endif

			if(hsio2_enable)
				CKENB |= (0x1u << (CKEN_HSIO2-32) );
			else
				CKENB &= ~(0x1u << (CKEN_HSIO2-32) );
			break;

		default:	/*normal case:*/
			if(clock < 32) {
				if (enable) {
					CKENA |= (0x1u << clock);
				} else {
					CKENA &= ~(0x1u << clock);
				}
			}else {
				if (enable) {
					CKENB |= (0x1u << (clock - 32) );
				} else {
					CKENB &= ~(0x1u << (clock - 32));
				}
			}
			break;
	}
#else
	if (enable)
		CKEN |= clock;
	else
		CKEN &= ~clock;

#endif
	local_irq_restore(flags);
	return;
}

EXPORT_SYMBOL(pxa_set_cken);

#ifndef CONFIG_PXA3xx
/*
 * Intel PXA2xx internal register mapping.
 *
 * Note 1: not all PXA2xx variants implement all those addresses.
 *
 * Note 2: virtual 0xfffe0000-0xffffffff is reserved for the vector table
 *         and cache flush area.
 */
static struct map_desc standard_io_desc[] __initdata = {
 /* virtual     physical    length      type */
  { 0xf2000000, 0x40000000, 0x02000000, MT_DEVICE }, /* Devs */
  { 0xf4000000, 0x44000000, 0x00100000, MT_DEVICE }, /* LCD */
  { 0xf6000000, 0x48000000, 0x00100000, MT_DEVICE }, /* Mem Ctl */
  { 0xf8000000, 0x4c000000, 0x00100000, MT_DEVICE }, /* USB host */
  { 0xfa000000, 0x50000000, 0x00100000, MT_DEVICE }, /* Camera */
  { 0xfe000000, 0x58000000, 0x00100000, MT_DEVICE }, /* IMem ctl */
  { 0xff000000, 0x00000000, 0x00100000, MT_DEVICE }  /* UNCACHED_PHYS_0 */
};
#else
/*
 * Intel PXA3xx internal register mapping.
 *
 * Note 1: not all variants implement all those addresses.
 *
 * Note 2: virtual 0xfffe0000-0xffffffff is reserved for the vector table
 *         and cache flush area.
 */
static struct map_desc standard_io_desc[] __initdata = {
  { 0xf5000000, 0x14000000, 0x01000000, MT_DEVICE },  /* VLIO IO            */
  { 0xf6000000, 0x40000000, 0x02000000, MT_DEVICE },  /* devices            */
  { 0xf8000000, 0x42000000, 0x00200000, MT_DEVICE },  /* MMC2 & USIM2       */
  { 0xf8300000, 0x43100000, 0x00100000, MT_DEVICE },  /* nand               */
  { 0xf8400000, 0x44000000, 0x00100000, MT_DEVICE },  /* lcd                */
  { 0xf8800000, 0x46000000, 0x00100000, MT_DEVICE },  /* mini-lcd           */
  { 0xf8d00000, 0x48100000, 0x00100000, MT_DEVICE },  /* dynamic mem ctl    */
  { 0xf9000000, 0x4a000000, 0x00100000, MT_DEVICE },  /* static memory ctl  */
  { 0xf9400000, 0x4c000000, 0x00100000, MT_DEVICE },  /* usb host           */
  { 0xfa000000, 0x50000000, 0x00100000, MT_DEVICE },  /* camera             */
  { 0xfa400000, 0x54000000, 0x00200000, MT_DEVICE },  /* 2d-graphics & usb2 */
  { 0xfa800000, 0x58000000, 0x00100000, MT_DEVICE },  /* internal SRAM ctl  */
#ifdef CONFIG_PXA310
  { 0xfa900000, 0x42500000, 0x00100000, MT_DEVICE },  /* MMC3 */
#endif
};
#endif

void __init pxa_map_io(void)
{
	iotable_init(standard_io_desc, ARRAY_SIZE(standard_io_desc));
	get_clk_frequency_khz(1);
}

static u64 ohci_hcd_pxa_dmamask = 0xffffffffUL;

static struct resource pxa_ohci_resources[] = {
	{
		.start= 0x4C000000,
		.end= 0x4C000fff,
		.flags= IORESOURCE_MEM,
	}, {
		.start= IRQ_USBH1,
		.end= IRQ_USBH1,
		.flags= IORESOURCE_IRQ,
	},
};

static void ohci_hcd_pxa_device_release(struct device *dev)
{
	/* Keep this function empty. */
}

static struct platform_device ohci_hcd_pxa_device = {
	.name = "pxa27x-ohci",
	.id = -1,
	.dev		= {
		.dma_mask = &ohci_hcd_pxa_dmamask,
		.coherent_dma_mask = 0xffffffff,
		.release = ohci_hcd_pxa_device_release,
	},

	.num_resources = ARRAY_SIZE(pxa_ohci_resources),
	.resource      = pxa_ohci_resources,
};

#ifndef CONFIG_PXA3xx
static struct resource pxamci_resources[] = {
	[0] = {
		.start	= 0x41100000,
		.end	= 0x41100fff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_MMC,
		.end	= IRQ_MMC,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 pxamci_dmamask = 0xffffffffUL;

static struct platform_device pxamci_device = {
	.name		= "pxa2xx-mci",
	.id		= -1,
	.dev		= {
		.dma_mask = &pxamci_dmamask,
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(pxamci_resources),
	.resource	= pxamci_resources,
};

#else

static u64 pxa_mmc_controller_dmamask = 0xffffffffUL;

static void pxa_mmc_controller_release(struct device *dev)
{
}

static struct resource mmc0_resources[] = {
	{
		.start	= 0x41100000,
		.end	= 0x41100fff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_MMC,
		.end	= IRQ_MMC,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device mmc0_device = {
	.name		=	"pxa2xx-mci",
	.id		=	0,
	.resource	= mmc0_resources,
	.num_resources	= ARRAY_SIZE(mmc0_resources),
	.dev		= {
		.dma_mask	   =	&pxa_mmc_controller_dmamask,
		.release	   =	pxa_mmc_controller_release,
		.coherent_dma_mask =	0xffffffff,
	},
};

#ifdef CONFIG_PXA310_MMC3
static struct resource mmc2_resources[] = {
	{
		.start	= 0x42500000,
		.end	= 0x42500fff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_MMC3,
		.end	= IRQ_MMC3,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device mmc2_device = {
	.name		=	"pxa2xx-mci",
	.id		=	1,
	.resource	= mmc2_resources,
	.num_resources	= ARRAY_SIZE(mmc2_resources),
	.dev		= {
		.dma_mask	   =	&pxa_mmc_controller_dmamask,
		.release	   =	pxa_mmc_controller_release,
		.coherent_dma_mask =	0xffffffff,
	},
};
#endif /* CONFIG_PXA310_MMC3 */

#endif /* CPU_PXA_3xx */

void __init pxa_set_mci_info(struct pxamci_platform_data *info)
{
#ifndef CONFIG_PXA3xx
	pxamci_device.dev.platform_data = info;
#else
	mmc0_device.dev.platform_data = info;
#ifdef CONFIG_PXA310_MMC3
	mmc2_device.dev.platform_data = info;
#endif
#endif
}

EXPORT_SYMBOL_GPL(pxa_set_mci_info);

static struct pxa2xx_udc_mach_info pxa_udc_info;

void __init pxa_set_udc_info(struct pxa2xx_udc_mach_info *info)
{
	memcpy(&pxa_udc_info, info, sizeof *info);
}

EXPORT_SYMBOL_GPL(pxa_set_udc_info);

static struct resource pxa2xx_udc_resources[] = {
	[0] = {
		.start	= 0x40600000,
		.end	= 0x4060ffff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_USB,
		.end	= IRQ_USB,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 udc_dma_mask = ~(u32)0;

static struct platform_device udc_device = {
	.name		= "pxa2xx-udc",
	.id		= -1,
	.resource	= pxa2xx_udc_resources,
	.num_resources	= ARRAY_SIZE(pxa2xx_udc_resources),
	.dev		=  {
		.platform_data	= &pxa_udc_info,
		.dma_mask	= &udc_dma_mask,
#if defined(CONFIG_PXA3xx)
		.coherent_dma_mask = 0xffffffff,
#endif
	}
};

static struct pxafb_mach_info pxa_fb_info;

void __init set_pxa_fb_info(struct pxafb_mach_info *hard_pxa_fb_info)
{
	memcpy(&pxa_fb_info,hard_pxa_fb_info,sizeof(struct pxafb_mach_info));
}

EXPORT_SYMBOL_GPL(set_pxa_fb_info);

static struct resource pxafb_resources[] = {
	[0] = {
		.start	= 0x44000000,
		.end	= 0x4400ffff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_LCD,
		.end	= IRQ_LCD,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 fb_dma_mask = ~(u64)0;

static struct platform_device pxafb_device = {
	.name		= "pxa2xx-fb",
	.id		= -1,
	.dev		= {
 		.platform_data	= &pxa_fb_info,
		.dma_mask	= &fb_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(pxafb_resources),
	.resource	= pxafb_resources,
};

static struct platform_device ffuart_device = {
	.name		= "pxa2xx-uart",
	.id		= 0,
};
static struct platform_device btuart_device = {
	.name		= "pxa2xx-uart",
	.id		= 1,
};
static struct platform_device stuart_device = {
	.name		= "pxa2xx-uart",
	.id		= 2,
};

#if defined(CONFIG_PXA3xx)

static struct platform_device nand_device = {
	.name		= "PXA3xx-nand-flash",
	.id		= -1,
};

static struct platform_device camera_device = {
	.name		= "pxa2xx-camera",
	.id		= -1,
};

static struct platform_device sibley_device = {
	.name		= "PXA3xx-sibley-flash",
	.id		= -1,
};

static struct resource pxaac97_resources[] = {
        {
                .start = 0x40500000,
                .end = 0x40500fff,
                .flags = IORESOURCE_MEM,
        }, {
                .start  = IRQ_AC97,
                .end = IRQ_AC97,
                .flags = IORESOURCE_IRQ,
        },
};

static u64 pxa_ac97_dmamask = 0xffffffffUL;

static struct platform_device pxaac97_device = {
	.name           = "pxa2xx-ac97",
	.id             = -1,
	.dev            = {
                .dma_mask = &pxa_ac97_dmamask,
                .coherent_dma_mask = 0xffffffffUL,
	},
	.num_resources  = ARRAY_SIZE(pxaac97_resources),
	.resource       = pxaac97_resources,
};

static struct platform_device pxatouch_device = {
        .name           = "pxa2xx-touch",
        .id             = -1,
};

#ifdef CONFIG_PXA3xx_GPIOEX
#include <asm/arch/mhn_gpio.h>
static struct resource gpio_exp0_resources[] = {
	[0] = {
		.start = IRQ_GPIO(MFP2GPIO(MFP_GPIO_EXP_0_N)),
		.end = IRQ_GPIO(MFP2GPIO(MFP_GPIO_EXP_0_N)),
		.flags = IORESOURCE_IRQ,
	},
	[1] = {
		.start  = GPIO_EXP0_ADDRESS,
		.end = GPIO_EXP0_ADDRESS,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.start  = GPIO_EXP0_START,
		.end = GPIO_EXP0_END,
		.flags = IORESOURCE_IO,
	},
};

static struct platform_device gpio_exp0_device = {
	.name           = "gpio-exp",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(gpio_exp0_resources),
	.resource       = gpio_exp0_resources,
};

static struct resource gpio_exp1_resources[] = {
	[0] = {
		.start = IRQ_GPIO(MFP2GPIO(MFP_GPIO_EXP_1_N)),
		.end = IRQ_GPIO(MFP2GPIO(MFP_GPIO_EXP_1_N)),
		.flags = IORESOURCE_IRQ,
	},
	[1] = {
		.start  = GPIO_EXP1_ADDRESS,
		.end = GPIO_EXP1_ADDRESS,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.start  = GPIO_EXP1_START,
		.end = GPIO_EXP1_END,
		.flags = IORESOURCE_IO,
	},
};

static struct platform_device gpio_exp1_device = {
	.name           = "gpio-exp",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(gpio_exp1_resources),
	.resource       = gpio_exp1_resources,
};

#ifdef CONFIG_PXA_MWB_12
static struct resource gpio_exp2_resources[] = {
	[0] = {
		.start  = GPIO_EXP2_ADDRESS,
		.end = GPIO_EXP2_ADDRESS,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start  = GPIO_EXP2_START,
		.end = GPIO_EXP2_END,
		.flags = IORESOURCE_IO,
	},
};

static struct platform_device gpio_exp2_device = {
	.name           = "gpio-exp",
	.id             = 2,
	.num_resources  = ARRAY_SIZE(gpio_exp2_resources),
	.resource       = gpio_exp2_resources,
};
#endif
#endif

static struct platform_device mhn_pmic_device = {
        .name 		= "mhn_pmic",
        .id 		= -1,
};

static struct platform_device mhn_fv_device = {
        .name 		= "mhn_fv",
        .id 		= -1,
};

static struct resource w1_resources[] = {
	{
		.start	= 0x41B00000,
		.end	= 0x41B00010,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device w1_device = {
	.name		= "1-wire-mp",
	.id		= 0,
	.resource	= w1_resources,
	.num_resources	= ARRAY_SIZE(w1_resources),
};

static struct platform_device rtc_device = {
	.name		= "pxa-rtc",
	.id		= -1,
};

static struct resource i2c_resources[] = {
	{
		.start	= 0x40301680,
		.end	= 0x403016a3,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_I2C,
		.end	= IRQ_I2C,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 pxaficp_dmamask = ~(u32)0;
#endif

static struct platform_device i2c_device = {
	.name 	= "pxa2xx-i2c",
	.id 	= 0,
#if defined(CONFIG_PXA3xx)
	.resource	= i2c_resources,
	.num_resources	= ARRAY_SIZE(i2c_resources),
#endif
};

static struct platform_device pxaficp_device = {
	.name		= "pxaxxx-ir",
	.id		= -1,
#if defined(CONFIG_PXA3xx)
        .dev            = {
                .dma_mask = &pxaficp_dmamask,
                .coherent_dma_mask = 0xffffffff,
        },
#endif
};

#if defined(CONFIG_PXA3xx)
static struct platform_device PXA3xx_otg_device = {
	.name	=	"PXA3xx-otg",
	.id	=	-1,
};

static void pxa_u2d_release(struct device *dev)
{
	/* Keep this function empty. */
}

static struct pxa27x_u2d_mach_info pxa_u2d_info;

static u64 u2d_dma_mask = ~(u32)0;

static struct resource pxa3xx_u2d_resources[] = {
	{
		.start	= 0x54100000,
		.end	= 0x5410ffff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_USB2,
		.end	= IRQ_USB2,
		.flags	= IORESOURCE_IRQ,
	},
};

#include <asm/arch/u2d.h>

static struct platform_device u2d_device = {
	.name		= "pxa3xx-u2d",
	.id		= -1,
	.resource	= pxa3xx_u2d_resources,
	.num_resources	= ARRAY_SIZE(pxa3xx_u2d_resources),
	.dev		=  {
		.platform_data	= &pxa_u2d_info,
		.dma_mask	= &u2d_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.release = pxa_u2d_release,
	}
};

void pxa_set_u2d_info(struct pxa27x_u2d_mach_info *info)
{
        u2d_device.dev.platform_data = info;
}

EXPORT_SYMBOL_GPL(pxa_set_u2d_info);

static struct platform_device zyt_pcmcia_device = {
	.name		= "pxa2xx-pcmcia",
	.id		= 0,
	.resource	= 0,
	.num_resources	= 0,
	.dev		=  {
		.platform_data	= NULL,
		.dma_mask	= 0,
		.coherent_dma_mask = 0,
		.release = 0,
	}
};

void pxa_set_pcmcia_info(void *info)
{
        zyt_pcmcia_device.dev.platform_data = info;
}

EXPORT_SYMBOL_GPL(pxa_set_pcmcia_info);

static struct platform_device zb_ts_device = {
	.name		= "zb_ts",
	.id		= -1,
};

static struct platform_device zb_kp_device = {
	.name		= "zb_kp",
	.id		= -1,
};

static struct resource m2d_resources[2] = {
	{
		.start = 0x54000000,
		.end   = 0x540fffff,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_GRPHICS,
		.end   = IRQ_GRPHICS,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device gcu_device = {
	.name		= "m2d",
	.id		= -1,
	.resource       = m2d_resources,
	.num_resources  = ARRAY_SIZE(m2d_resources),
};

#endif

void __init pxa_set_ficp_info(struct pxaficp_platform_data *info)
{
	pxaficp_device.dev.platform_data = info;
}
EXPORT_SYMBOL_GPL(pxa_set_ficp_info);

static struct platform_device *devices[] __initdata = {
#if !defined(CONFIG_PXA3xx)
	&pxamci_device,
#endif
	&udc_device,
	&pxafb_device,
	&ffuart_device,
	&btuart_device,
	&stuart_device,
	&i2c_device,
#if defined(CONFIG_PXA3xx)
	&mhn_pmic_device,
#ifdef CONFIG_PXA3xx_GPIOEX
	&gpio_exp0_device,
	&gpio_exp1_device,
#ifdef CONFIG_PXA_MWB_12
	&gpio_exp2_device,
#endif
#endif
	&mhn_fv_device,
 	&nand_device,
	&pxaac97_device,
	&pxatouch_device,
	&camera_device,
	&sibley_device,
	&w1_device,
	&rtc_device,
#ifdef  CONFIG_PXA_FICP
	&pxaficp_device,
#endif
	&mmc0_device,
#ifdef CONFIG_PXA310_MMC3
	&mmc2_device,
#endif
	&PXA3xx_otg_device,
#ifdef CONFIG_USB_GADGET_PXA3XX_U2D
	&u2d_device,
#endif
	&ohci_hcd_pxa_device,
	&zyt_pcmcia_device,
	&zb_ts_device,
	&zb_kp_device,
	&gcu_device,
#else	/* defined(CONFIG_PXA3xx) */
	&pxaficp_device,
#endif	/* defined(CONFIG_PXA3xx) */
};

static int __init pxa_init(void)
{
#if defined(CONFIG_PXA3xx)
	/* clear RDH */
	ASCR &= 0x7fffffff;
#endif
  	return platform_add_devices(devices, ARRAY_SIZE(devices));
}
subsys_initcall(pxa_init);
