/*
 * arch/mips/tx4939/toshiba_rbtx4939/setup.c
 *
 * Setup pointers to hardware-dependent routines.
 *
 * (C) Copyright TOSHIBA CORPORATION SEMICONDUCTOR COMPANY 2000-2001,2005
 *
 * Author: source@mvista.com
 *
 * 2001-2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Support for TX4939 in 2.6 - Hiroshi DOYU <Hiroshi_DOYU@montavista.co.jp>
 */

#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/console.h>
#include <linux/string.h>
#include <linux/device.h>
#include <asm/io.h>
#include <asm/types.h>
#include <asm/tx4939/tx4939.h>
#include <asm/tx4939/rbtx4939.h>
#if defined(CONFIG_PCI)
#include <linux/pci.h>
#endif
#if defined(CONFIG_MTD_TX4939)
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#endif

void rbtx4939_machine_restart(char *command)
{
	local_irq_disable();
	reg_wr08(rbtx4939_sreset_enable_ptr, 1);
	reg_wr08(rbtx4939_soft_reset_ptr, 1);
	wbflush();
	while (1) ;
}

#if defined(CONFIG_SMC91X)
static struct resource smc91x_resources[] = {
	[0] = {
		.start = CPHYSADDR(RBTX4939_DEBUG_ETHER_BASE),
		.end = CPHYSADDR(RBTX4939_DEBUG_ETHER_BASE) + 0x10 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = RBTX4939_IRQ_DEBUG_ETHER,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = 1 /* io_mask: 8bit only */ ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name = "smc91x",
	.id = 1,
	.num_resources = ARRAY_SIZE(smc91x_resources),
	.resource = smc91x_resources,
};

static struct platform_device *rbtx4939_devs[] __initdata = {
	&smc91x_device,
};

static int __init rbtx4939_device_init(void)
{
	platform_add_devices(rbtx4939_devs, ARRAY_SIZE(rbtx4939_devs));
	return 0;
}

__initcall(rbtx4939_device_init);

#endif

void __init rbtx4939_pin_enable_setup(void)
{
	u64 pcfg = reg_rd64s(&tx4939_ccfgptr->pcfg);
	u8 pe1, pe2, pe3;

	pe1 = 0xf0;

	if (pcfg & TX4939_PCFG_ET1MODE_ETHER)
		pe1 |= RBTX4939_PE1_RMII1;

	if (pcfg & TX4939_PCFG_ET0MODE_ETHER)
		pe1 |= RBTX4939_PE1_RMII0;

	if (pcfg & TX4939_PCFG_ATA1MODE_ATA)
		pe1 |= RBTX4939_PE1_ATA1;

	if (pcfg & TX4939_PCFG_ATA0MODE_ATA)
		pe1 |= RBTX4939_PE1_ATA0;

	reg_wr08(rbtx4939_pe1_ptr, pe1);


	pe2 = 0xc0;

	if (pcfg & TX4939_PCFG_I2SMODE_GPIO)
		pe2 |= RBTX4939_PE2_GPIO;

	if (pcfg & TX4939_PCFG_SPIMODE_SIO_GPIO) {
#if defined(CONFIG_TOSHIBA_RBTX4939_PIN_CIR)
		pe2 |= RBTX4939_PE2_CIR;
#endif
#if defined(CONFIG_TOSHIBA_RBTX4939_PIN_SIO3)
		pe2 |= RBTX4939_PE2_SIO3
#endif
	} else
		pe2 |= RBTX4939_PE2_SPI;

	if (pcfg & TX4939_PCFG_SIO2MODE_SIO2)
		pe2 |= RBTX4939_PE2_SIO2;

	pe2 |= RBTX4939_PE2_SIO1;
	reg_wr08(rbtx4939_pe2_ptr, pe2);


	pe3 = 0xf0;

	if (pcfg & TX4939_PCFG_VPMODE_3S)
		pe3 |= RBTX4939_PE3_VP_S;

	if (pcfg & (TX4939_PCFG_VPMODE_1P | TX4939_PCFG_VPMODE_1P1S))
		pe3 |= RBTX4939_PE3_VP_P;

	if (pcfg & (TX4939_PCFG_VPMODE_3S | TX4939_PCFG_VPMODE_1P1S))
		pe3 |= RBTX4939_PE3_VP;

	reg_wr08(rbtx4939_pe3_ptr, pe3);
}

static void __init rbtx4939_request_resource(void)
{
	/* RBTX4939 I/O Controller */
	static struct resource rbtx4939_ioc_resource = {
		.name = "RBTX4939 I/O controller",
		.start = CPHYSADDR(RBTX4939_IOC_REG),
		.end = CPHYSADDR(RBTX4939_IOC_REG + RBTX4939_IOC_SIZE),
		.flags = IORESOURCE_MEM,
	};

	if (request_resource(&iomem_resource, &rbtx4939_ioc_resource))
		printk(KERN_ERR "request resource for I/O controller failed\n");
}

static void __init rbtx4939_bootlog(void)
{
	printk
	    (KERN_INFO "RBTX4939 --- Board(0x%02x) IOC(0x%02x) DIPSW:(User:0x%02x,Boot:0x%02x)\n",
	     reg_rd08(rbtx4939_board_rev_ptr),
	     reg_rd08(rbtx4939_ioc_rev_ptr),
	     reg_rd08(rbtx4939_usersw_ptr),
	     reg_rd08(rbtx4939_bootsw_ptr));
}

#if defined(CONFIG_MTD_TX4939)

static struct mtd_partition tx4939_userrom1_parts[] = {
	{
		.name = "USER ROM1_1",
		.size = 0x00800000,  /* 8MB */
		.offset = 0,
	},
	{
		.name = "USER ROM1_2",
		.size = 0x007c0000,  /* 8MB *//* don't expose yamon data area */
		.offset = 0x00800000,
	},
};

static struct mtd_partition tx4939_userrom2_parts[] = {
	{
		.name = "USER ROM2_1",
		.size = 0x00800000,  /* 8MB */
		.offset = 0,
	},
	{
		.name = "USER ROM2_2",
		.size = 0x00800000,  /* 8MB */
		.offset = 0x00800000,
	},
};
static void __init rbtx4939_nor_setup(void)
{
	int n = 0;
	u8 sw = reg_rd08(rbtx4939_bootsw_ptr);

	switch (sw) {
	case 0x0:
	case 0x1:
		early_txmtd_setup(n++, "USER ROM1",
				  0x1d000000, 0x1000000,
				  2, 2, tx4939_userrom1_parts, NULL);
		early_txmtd_setup(n++, "USER ROM2",
				  0x1c000000, 0x1000000,
				  2, 2, tx4939_userrom2_parts, NULL);
		break;
	case 0x8:
	case 0x9:
	case 0xa:
	case 0xb:
		early_txmtd_setup(n++, "USER ROM1",
				  0x1f000000, 0x1000000,
				  2, 2, tx4939_userrom1_parts, NULL);
		early_txmtd_setup(n++, "USER ROM2",
				  0x1e000000, 0x1000000,
				  2, 2, tx4939_userrom2_parts, NULL);
		break;
	case 0xc:
	case 0xd:
	case 0xe:
	case 0xf:
		early_txmtd_setup(n++, "USER ROM2",
				  0x1f000000, 0x1000000,
				  2, 2, tx4939_userrom2_parts, NULL);
		early_txmtd_setup(n++, "USER ROM1",
				  0x1e000000, 0x1000000,
				  2, 2, tx4939_userrom1_parts, NULL);
		break;
	}
}

/**
 * rbtx4939_get_tx4939_nandc_parameter - get rbtx4939 nandc parameter
 * @hold: hold time
 * @spw: strobe pulse width
 *
 */
#if defined(CONFIG_MTD_NAND)
void rbtx4939_get_tx4939_nandc_parameter(int *hold, int *spw)
{
	*hold = 2;
	*spw = 8;               /*  GBUSCLK = 5ns (@ GBUSCLK 200MHz) */
}
#endif                         /* CONFIG_MTD_NAND */
#endif				/* CONFIG_MTD_TX4938 */

/**
 * rbtx4939_setup - TX4939 reference board setup routine
 *
 * This function runs RBTX4939(TX4939 reference board) setup routines.
 */

void __init rbtx4939_setup(void)
{
	rbtx4939_pin_enable_setup();
	rbtx4939_request_resource();
	rbtx4939_bootlog();
#if defined(CONFIG_MTD_TX4939)
	rbtx4939_nor_setup();
#endif
	rbtx4939_led(0, 0x0);
	rbtx4939_led(1, 0x0);
	rbtx4939_led(2, 0x2);
	rbtx4939_led(3, 0x6);
	/* mips_io_port_base will be overwritten by tx4939_pci_setup() */
	set_io_port_base(TX4939_PCI0_MEM_RESOURCE_START);
}

#if defined(CONFIG_PCI)
/**
 * rbtx4939_get_tx4939_ethaddr - get rbtx4939 debug ether mac address
 * @dev:
 * @addr: character pointer for address string
 *
 */

int rbtx4939_get_tx4939_ethaddr(struct pci_dev *dev, unsigned char *addr)
{
	int ch;
	static int read_dat;
	struct pci_controller *channel =
		(struct pci_controller *)dev->bus->sysdata;
	static unsigned char dat[0x17];

	if (channel != &tx4939_pci_controller[1])
		return -ENODEV;
	/* TX4939 PCIC1 */
	switch (PCI_SLOT(dev->devfn)) {
	case TX4939_PCIC_IDSEL_AD_TO_SLOT(TX4939_ETHER_IDSEL(0)):
		ch = 0;
		break;
	case TX4939_PCIC_IDSEL_AD_TO_SLOT(TX4939_ETHER_IDSEL(1)):
		ch = 1;
		break;
	default:
		return -ENODEV;
	}

	if (!read_dat) {
		unsigned char sum;
		int i;
		read_dat = 1;
		memcpy(dat, (void *)(KSEG1 + RBTX4939_ETHER_MAC_ADDR_BASE),
		       sizeof(dat));

		for (i = 0, sum = 0; i < sizeof(dat); i++)
			sum += dat[i];

		if (sum)
			printk(KERN_WARNING
			       "Ethernet MAC Address: bad checksum.\n");
	}

	memcpy(addr, &dat[0x10 * ch], 6);

	return 0;
}

EXPORT_SYMBOL(rbtx4939_get_tx4939_ethaddr);

#endif				/* CONFIG_PCI */
