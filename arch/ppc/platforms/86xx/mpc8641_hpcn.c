/*
 * arch/ppc/platforms/86xx/mpc8641_hpcn.c
 *
 * MPC8641 HPCN  board specific routines
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

#include <linux/config.h>
#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/reboot.h>
#include <linux/pci.h>
#include <linux/kdev_t.h>
#include <linux/major.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/seq_file.h>
#include <linux/root_dev.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/module.h>
#include <linux/fsl_devices.h>
#include <linux/initrd.h>

#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/atomic.h>
#include <asm/time.h>
#include <asm/io.h>
#include <asm/machdep.h>
#include <asm/prom.h>
#include <asm/open_pic.h>
#include <asm/bootinfo.h>
#include <asm/pci-bridge.h>
#include <asm/mpc86xx.h>
#include <asm/irq.h>
#include <asm/immap_86xx.h>
#include <asm/kgdb.h>
#include <asm/ppc_sys.h>
#include <asm/todc.h>
#include <asm/i8259.h>
#include <mm/mmu_decl.h>

#include <syslib/ppc86xx_common.h>
#include <syslib/ppc86xx_setup.h>

extern void gen550_progress(char *, unsigned short);
extern void gen550_init(int, struct uart_port *);
extern void __init mpc86xx_smp_init(void);

#ifndef CONFIG_PCI
unsigned long isa_io_base = 0;
unsigned long isa_mem_base = 0;
unsigned long pci_dram_offset = 0;
#endif
extern unsigned long total_memory;	/* in mm/init */

unsigned char __res[sizeof (bd_t)];

/* Internal interrupts are all Level Sensitive, and Positive Polarity */

static u_char mpc86xx_hpcn_openpic_initsenses[] __initdata = {
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  0: Reserved */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  1: MCM */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  2: DDR DRAM */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  3: LBIU */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  4: DMA 0 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  5: DMA 1 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  6: DMA 2 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  7: DMA 3 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  8: PCIE1 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  9: PCIE2 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 10: Reserved */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 11: Reserved */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 12: DUART2 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 13: TSEC 1 Transmit */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 14: TSEC 1 Receive */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 15: TSEC 3 transmit */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 16: TSEC 3 receive */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 17: TSEC 3 error */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 18: TSEC 1 Receive/Transmit Error */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 19: TSEC 2 Transmit */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 20: TSEC 2 Receive */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 21: TSEC 4 transmit */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 22: TSEC 4 receive */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 23: TSEC 4 error */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 24: TSEC 2 Receive/Transmit Error */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 25: Unused */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 26: DUART1 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 27: I2C */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 28: Performance Monitor */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 29: Unused */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 30: Unused */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 31: Unused */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 32: SRIO error/write-port unit */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 33: SRIO outbound doorbell */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 34: SRIO inbound doorbell */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 35: Unused */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 36: Unused */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 37: SRIO outbound message unit 1 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 38: SRIO inbound message unit 1 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 39: SRIO outbound message unit 2 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 40: SRIO inbound message unit 2 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 41: Unused */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 42: Unused */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 43: Unused */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 44: Unused */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 45: Unused */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 46: Unused */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 47: Unused */
	0x0,						/* External  0: */
	0x0,						/* External  1: */
	0x0,						/* External  2: */
	0x0,						/* External  3: */
	0x0,						/* External  4: */
	0x0,						/* External  5: */
	0x0,						/* External  6: */
	0x0,						/* External  7: */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* External  8: Pixis FPGA */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* External  9: ULI 8259 INTR Cascade */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* External 10: Quad ETH PHY */
	0x0,						/* External 11: */
	0x0,
	0x0,
	0x0,
	0x0,
};

/* ************************************************************************ */
int
mpc86xx_hpcn_show_cpuinfo(struct seq_file *m)
{
	uint pvid, svid, phid1;
	uint memsize = total_memory;
	bd_t *binfo = (bd_t *) __res;
	unsigned int freq;

	/* get the core frequency */
	freq = binfo->bi_intfreq;

	pvid = mfspr(SPRN_PVR);
	svid = mfspr(SPRN_SVR);

	seq_printf(m, "Vendor\t\t: Freescale Semiconductor\n");

	switch (pvid & 0xffff0000) {
	case PVR_8641:
		seq_printf(m, "Machine\t\t: HPCN 8641 Board\n");
		break;
	default:
		seq_printf(m, "Machine\t\t: unknown\n");
		break;
	}
	seq_printf(m, "bus freq\t: %u.%.6u MHz\n", freq / 1000000,
		   freq % 1000000);
	seq_printf(m, "PVR\t\t: 0x%x\n", pvid);
	seq_printf(m, "SVR\t\t: 0x%x\n", svid);

	/* Display cpu Pll setting */
	phid1 = mfspr(SPRN_HID1);
	seq_printf(m, "PLL setting\t: 0x%x\n", ((phid1 >> 24) & 0x3f));

	/* Display the amount of memory */
	seq_printf(m, "Memory\t\t: %d MB\n", memsize / (1024 * 1024));

	return 0;
}

void __init
mpc86xx_hpcn_init_irq(void)
{
	bd_t *binfo = (bd_t *) __res;
	/* Determine the Physical Address of the OpenPIC regs */
	phys_addr_t OpenPIC_PAddr =
		binfo->bi_immr_base + MPC86xx_OPENPIC_OFFSET;
	OpenPIC_Addr = ioremap(OpenPIC_PAddr, MPC86xx_OPENPIC_SIZE);
	OpenPIC_InitSenses = mpc86xx_hpcn_openpic_initsenses;
	OpenPIC_NumInitSenses = sizeof (mpc86xx_hpcn_openpic_initsenses);

	/* Map Internal Interrupts */
	openpic_set_sources(0, 48, OpenPIC_Addr + 0x10200);

	/* Map External Interrupts */
	openpic_set_sources(48, 16, OpenPIC_Addr + 0x10000);

	/* we let openpic interrupts start from an offset, to
	 * leave space for cascading interrupts underneath.
	 */
	openpic_init(MPC86xx_OPENPIC_IRQ_OFFSET);

#ifdef CONFIG_PCIE
	{
		int i;

		openpic_hookup_cascade(MPC86xx_IRQ_EXT9, "ULI 8259 cascade", i8259_irq);

		for (i = 0; i < NUM_8259_INTERRUPTS; i++)
			irq_desc[i].handler = &i8259_pic;

		i8259_init(0);
	}
#endif
}

#ifdef CONFIG_PCI
/*
 * interrupt routing
 */

int
mpc86xx_map_irq(struct pci_dev *dev, unsigned char idsel, unsigned char pin)
{
	struct pci_controller *hose = pci_bus_to_hose(dev->bus->number);

	if (hose->index == 0) {
		static char pci_irq_table[][4] =
 		/*
		 *      PCI IDSEL/INTPIN->INTLINE
		 *       A      B      C      D
		 */
		{
			{PIRQA, PIRQB, PIRQC, PIRQD},	/* 17 - PCI Slot 1 */
			{PIRQB, PIRQC, PIRQD, PIRQA},	/* 18 - PCI Slot 2 */
			{PIRQC, PIRQD, PIRQA, PIRQB},	/* 19 */
			{PIRQD, PIRQA, PIRQB, PIRQC},	/* 20 */
			{0, 0, 0, 0},			/* 21 */
			{0, 0, 0, 0},			/* 22 */
			{0, 0, 0, 0},			/* 23 */
			{0, 0, 0, 0},			/* 24 */
			{0, 0, 0, 0},			/* 25 */
			{PIRQD, PIRQA, PIRQB, PIRQC},	/* 26 - PCI Bridge*/
			{PIRQC, 0, 0, 0},		/* 27 - LAN */
			{PIRQE, PIRQF, PIRQG, PIRQ7},	/* 28 - USB 1.1 */
			{PIRQE, PIRQF, PIRQG, 0},	/* 29 - Audio & Modem */
			{PIRQG, 0, 0, 0},		/* 30 - LPC & PMU*/
			{PIRQD, PIRQD, PIRQD, PIRQD},	/* 31 - ATA */
		};
		const long min_idsel = 17, max_idsel = 31, irqs_per_slot = 4;

		/*
		 * On rev 1.02 h/w, PCI Bridge is on bus 1 devfn 0.  Other PCI
		 * devices on the m1575 are behind the bridge.  Fix up the
		 * idsel's for these devices.
		 */
		if (!idsel) {
			if (PCI_SLOT(dev->devfn) >= 27)
				idsel = PCI_SLOT(dev->devfn);
			else
				idsel = 26;
		}

		return PCI_IRQ_TABLE_LOOKUP + I8259_OFFSET;
	} else if (hose->index == 1) {
		/*
		 *      PCI IDSEL/INTPIN->INTLINE
		 *       A   B   C   D
		 */
		static char pci_irq_table[][4] =
		{
			{44, 45, 46, 47},		/* 0 */
		};
		const long min_idsel = 0, max_idsel = 0, irqs_per_slot = 4;
		return PCI_IRQ_TABLE_LOOKUP;
	} else
		return -1;
}

static void __devinit quirk_uli1575(struct pci_dev *dev)
{
	unsigned short temp;

	/*
	 * ULI1575 interrupts route table setup:
	 *
	 * IRQ pin   IRQ#
	 * PIRQA ---- 3
	 * PIRQB ---- 4
	 * PIRQC ---- 5
	 * PIRQD ---- 6
	 * PIRQE ---- 9
	 * PIRQF ---- 10
	 * PIRQG ---- 11
	 * PIRQH ---- disabled
	 *
	 * interrupts for PCI slot0 -- PIRQA / PIRQB / PIRQC / PIRQD
	 *		  PCI slot1 -- PIRQB / PIRQC / PIRQD / PIRQA
	 */
	pci_write_config_dword(dev, 0x48, 0x09317542);

	/* USB 1.1 OHCI controller 1, interrupt: PIRQE */
	pci_write_config_byte(dev, 0x86, 0x0c);

	/* USB 1.1 OHCI controller 2, interrupt: PIRQF */
	pci_write_config_byte(dev, 0x87, 0x0d);

	/* USB 1.1 OHCI controller 3, interrupt: PIRQH */
	pci_write_config_byte(dev, 0x88, 0x0f);

	/* USB 2.0 controller, interrupt: PIRQ7 */
	pci_write_config_byte(dev, 0x74, 0x06);

	/* Audio controller, interrupt: PIRQE */
	pci_write_config_byte(dev, 0x8a, 0x0c);

	/* Modem controller, interrupt: PIRQF */
	pci_write_config_byte(dev, 0x8b, 0x0d);

	/* HD audio controller, interrupt: PIRQG */
	pci_write_config_byte(dev, 0x8c, 0x0e);

	/* Serial ATA interrupt: PIRQD */
	pci_write_config_byte(dev, 0x8d, 0x0b);

	/* SMB interrupt: PIRQG */
	pci_write_config_byte(dev, 0x8e, 0x0e);

	/* PMU ACPI SCI interrupt: PIRQG */
	pci_write_config_byte(dev, 0x8f, 0x0e);

	/* Primary PATA IDE IRQ: 14
	 * Secondary PATA IDE IRQ: 15
	 */
	pci_write_config_byte(dev, 0x44, 0x3d);
	pci_write_config_byte(dev, 0x75, 0x0f);

	/* Set IRQ1, IRQ12, IRQ14 and IRQ15 to legacy IRQs */
	pci_read_config_word(dev, 0x46, &temp);
	temp |= 0xd002;
	pci_write_config_word(dev, 0x46, temp);

	/* Set i8259 interrupt trigger
	 * IRQ 3:  Level
	 * IRQ 4:  Level
	 * IRQ 5:  Level
	 * IRQ 6:  Level
	 * IRQ 7:  Level
	 * IRQ 9:  Level
	 * IRQ 10: Level
	 * IRQ 11: Level
	 * IRQ 12: Edge
	 * IRQ 14: Edge
	 * IRQ 15: Edge
	 */
	outb(0xf8, 0x4d0);
	outb(0x0e, 0x4d1);

	/* enable superio @ 0x4e and keyboard/mouse address decoding */
	pci_write_config_byte(dev, 0x63, 0x90);

	/* LPC47M192 Super I/O configuration */
	outb(0x55, 0x4e);	/* enter superio config mode */

	/* Enable keyboard and mouse */
	outb(0x07, 0x4e);	/* device selector register */
	outb(0x07, 0x4f);	/* select keyboard registers (device 7) */
	outb(0x30, 0x4e);	/* keyboard activation register */
	outb(0x01, 0x4f);	/* activate keyboard */
	outb(0x70, 0x4e);	/* keyboard IRQ register */
	outb(0x01, 0x4f);	/* IRQ1 for keyboard */
	outb(0x72, 0x4e);	/* mouse IRQ register */
	outb(0x0c, 0x4f);	/* IRQ12 for mouse */

	/* Enable superio runtime registers for gpio in pci i/o space */
	outb(0x20, 0x4e);	/* device id register */
	outb(0x07, 0x4e);	/* device selector register */
	outb(0x0a, 0x4f);	/* select runtime registers (device A) */
	outb(0x60, 0x4e);	/* select runtime register address high byte */
	outb(MPC8641_SUPERIO_GPIO_OFFSET >> 8, 0x4f);
	outb(0x61, 0x4e);	/* select runtime register address low byte */
	outb((MPC8641_SUPERIO_GPIO_OFFSET & 0xff) | 1, 0x4f);
	outb(0x30, 0x4e);	/* runtime registers activation register */
	outb(0x01, 0x4f);	/* activate runtime registers */

	outb(0xaa, 0x4e);	/* exit superio config mode */
}

static void __devinit quirk_uli5288(struct pci_dev *dev)
{
	unsigned char c;

	pci_read_config_byte(dev,0x83,&c);
	c |= 0x80;
	pci_write_config_byte(dev, 0x83, c);

	pci_write_config_byte(dev, 0x09, 0x01);
	pci_write_config_byte(dev, 0x0a, 0x06);

	pci_read_config_byte(dev,0x83,&c);
	c &= 0x7f;
	pci_write_config_byte(dev, 0x83, c);

	pci_read_config_byte(dev,0x84,&c);
	c |= 0x01;
	pci_write_config_byte(dev, 0x84, c);
}

static void __devinit quirk_uli5229(struct pci_dev *dev)
{
	unsigned short temp;
	pci_write_config_word(dev, 0x04, 0x0405);
	pci_read_config_word(dev, 0x4a, &temp);
	temp |= 0x1000;
	pci_write_config_word(dev, 0x4a, temp);
}

static void __devinit early_uli5249(struct pci_dev *dev)
{
	unsigned char temp;
	pci_write_config_word(dev, 0x04, 0x0007);
	pci_read_config_byte(dev, 0x7c, &temp);
	pci_write_config_byte(dev, 0x7c, 0x80);
	pci_write_config_byte(dev, 0x09, 0x01);
	pci_write_config_byte(dev, 0x7c, temp);
	dev->class |= 0x1;
}

DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_AL, 0x1575, quirk_uli1575);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_AL, 0x5288, quirk_uli5288);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_AL, 0x5229, quirk_uli5229);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_AL, 0x5249, early_uli5249);

int
mpc86xx_exclude_device(u_char bus, u_char devfn)
{
	if ((mfspr(SPRN_SVR) & 0xf0) == 0x10)
		if (bus == 0 && PCI_SLOT(devfn) == 0)
			return PCIBIOS_DEVICE_NOT_FOUND;

 	return PCIBIOS_SUCCESSFUL;
}

void mpc86xx_postscan_fixups(struct pci_controller *hose)
{
	/*
	 * The I/O base addresses for the Audio & Modem devices need to be valid
	 * or the RTC on the m1575 may hang.  This needs to happen before
	 * time_init() is called.
	 */
	unsigned short device_id;
	int bus;

	/* Rev 1.0 has devices on bus 0.  Rev 1.02 has devices on bus 2 */
	early_read_config_word(hose, 0, PCI_DEVFN(29, 0), PCI_DEVICE_ID,
			       &device_id);
	if (device_id == 0x5455)
		bus = 0;
	else
		bus = 2;

	early_write_config_dword(hose, bus, PCI_DEVFN(29, 0), PCI_BASE_ADDRESS_0,
				 MPC8641HPCN_ULI5455_IO_BASE);
	early_write_config_dword(hose, bus, PCI_DEVFN(29, 1), PCI_BASE_ADDRESS_1,
				 MPC8641HPCN_ULI5457_IO_BASE);
  }

#endif /* CONFIG_PCI */

void mpc8641hpcn_restart(char *cmd)
{
	bd_t *binfo = (bd_t *) __res;
	void __iomem *rstcr;

	rstcr = ioremap(binfo->bi_immr_base + MPC86XX_RSTCR_OFFSET, 0x100);

	local_irq_disable();

	/* Assert reset request to Reset Control Register */
	out_be32(rstcr, 0x2);

	/* not reached */
}

/* ************************************************************************
 *
 * Setup the architecture
 *
 */
static void __init
mpc8641hpcn_setup_arch(void)
{
	bd_t *binfo = (bd_t *) __res;
	unsigned int freq;
	struct gianfar_platform_data *pdata;
	struct gianfar_mdio_data *mdata;

	/* get the core frequency */
	freq = binfo->bi_intfreq;

	if (ppc_md.progress)
		ppc_md.progress("mpc8641hpcn_setup_arch()", 0);

	/* Set loops_per_jiffy to a half-way reasonable value,
	   for use until calibrate_delay gets called. */
	loops_per_jiffy = freq / HZ;

#ifdef CONFIG_PCI
	/* setup PCI host bridges */
	mpc86xx_setup_hose();
#endif

#ifdef CONFIG_SERIAL_8250
	mpc86xx_early_serial_map();
#endif

	printk(KERN_INFO "HPCN board with 8641D from DSD at Freescale Semiconductor. 2006\n");

	/* setup the board related info for the MDIO bus */
	mdata = (struct gianfar_mdio_data *) ppc_sys_get_pdata(MPC86xx_MDIO);

	mdata->irq[0] = MPC86xx_IRQ_EXT10;
	mdata->irq[1] = MPC86xx_IRQ_EXT10;
	mdata->irq[2] = MPC86xx_IRQ_EXT10;
	mdata->irq[3] = MPC86xx_IRQ_EXT10;
	mdata->irq[31] = -1;

	/* setup the board related information for the enet controllers */
	pdata = (struct gianfar_platform_data *) ppc_sys_get_pdata(MPC86xx_TSEC1);
	if (pdata) {
		pdata->board_flags = FSL_GIANFAR_BRD_HAS_PHY_INTR;
		pdata->bus_id = 0;
		pdata->phy_id = 0;
		memcpy(pdata->mac_addr, binfo->bi_enetaddr, 6);
	}

	pdata = (struct gianfar_platform_data *) ppc_sys_get_pdata(MPC86xx_TSEC2);
	if (pdata) {
		pdata->board_flags = FSL_GIANFAR_BRD_HAS_PHY_INTR;
		pdata->bus_id = 0;
		pdata->phy_id = 1;
		memcpy(pdata->mac_addr, binfo->bi_enet1addr, 6);
	}

	pdata = (struct gianfar_platform_data *) ppc_sys_get_pdata(MPC86xx_TSEC3);
	if (pdata) {
		pdata->board_flags = 0;
		pdata->bus_id = 0;
		pdata->phy_id = 2;
		memcpy(pdata->mac_addr, binfo->bi_enet2addr, 6);
	}

	pdata = (struct gianfar_platform_data *) ppc_sys_get_pdata(MPC86xx_TSEC4);
	if (pdata) {
		pdata->board_flags = 0;
		pdata->bus_id = 0;
		pdata->phy_id = 3;
		memcpy(pdata->mac_addr, binfo->bi_enet3addr, 6);
	}

#ifdef CONFIG_BLK_DEV_INITRD
	if (initrd_start)
		ROOT_DEV = Root_RAM0;
	else
#endif
#ifdef  CONFIG_ROOT_NFS
		ROOT_DEV = Root_NFS;
#else
		ROOT_DEV = Root_HDA1;
#endif
}

static void __init
mpc8641hpcn_map_io(void)
{
	uint addr;
	bd_t *binfo = (bd_t *) __res;

	/* Map IMMR region to a 1MB BAT */
	addr = binfo->bi_immr_base;
	io_block_mapping(addr, addr, (1024*1024), _PAGE_IO);
}

/*
 * Set BAT 1 to map CCSRBAR
 */
#ifdef CONFIG_SERIAL_TEXT_DEBUG
static __inline__ void
mpc8641_early_ccsrbar_map(void)
{
	bd_t *binfo = (bd_t *) __res;

	mb();
	mtspr(SPRN_DBAT1U, (binfo->bi_immr_base | BL_1M | 2)); /* Vs=1, Vp=0 */
	mtspr(SPRN_DBAT1L, (binfo->bi_immr_base | _PAGE_NO_CACHE | _PAGE_GUARDED | BPP_RW));
	mb();
}
#endif

TODC_ALLOC();

long mpc8641hpcn_time_init(void)
{
	u32 *p;
	/*
	 * This is a workaround for the RTC in the ULI m1575.
	 * The RTC is locked until we do a memory read through the chip.
	 * Reading the first location in PCI memory space seems to unlock it.
	 */
 	p = ioremap(MPC86XX_PCIE1_LOWER_MEM, sizeof(*p));
	(void) *(volatile u32 *)p;
	iounmap(p);

	TODC_INIT(TODC_TYPE_MC146818, 0x70, 0x00, 0x71, 8);
	return todc_time_init();
}

/* ************************************************************************ */
void __init
platform_init(unsigned long r3, unsigned long r4, unsigned long r5,
	      unsigned long r6, unsigned long r7)
{

	/* parse_bootinfo must always be called first */
	parse_bootinfo(find_bootinfo());

	/*
	 * If we were passed in a board information, copy it into the
	 * residual data area.
	 */
	if (r3) {
		memcpy((void *) __res, (void *) (r3 + KERNELBASE),
		       sizeof (bd_t));
	}


#ifdef CONFIG_SERIAL_TEXT_DEBUG
	{
		bd_t *binfo = (bd_t *) __res;
		struct uart_port p;
		mpc8641_early_ccsrbar_map();

		memset(&p, 0, sizeof (p));
		p.iotype = SERIAL_IO_MEM;
		p.membase = (void *) binfo->bi_immr_base + MPC86xx_UART0_OFFSET;
		p.uartclk = binfo->bi_busfreq; //BASE_BAUD*16;

		gen550_init(0, &p);

		memset(&p, 0, sizeof (p));
		p.iotype = SERIAL_IO_MEM;
		p.membase = (void *) binfo->bi_immr_base + MPC86xx_UART1_OFFSET;
		p.uartclk = binfo->bi_busfreq; //BASE_BAUD*16;

		gen550_init(1, &p);

	}
#endif

#if defined(CONFIG_BLK_DEV_INITRD)
	/*
	 * If the init RAM disk has been configured in, and there's a valid
	 * starting address for it, set it up.
	 */
	if (r4) {
		initrd_start = r4 + KERNELBASE;
		initrd_end = r5 + KERNELBASE;
	}
#endif	/* CONFIG_BLK_DEV_INITRD */

	/* Copy the kernel command line arguments to a safe place. */

	if (r6) {
		*(char *) (r7 + KERNELBASE) = 0;
		strcpy(cmd_line, (char *) (r6 + KERNELBASE));
	}

	identify_ppc_sys_by_id(mfspr(SPRN_SVR));

	/* setup the PowerPC module struct */
	ppc_md.setup_arch = mpc8641hpcn_setup_arch;
	ppc_md.show_cpuinfo = mpc86xx_hpcn_show_cpuinfo;

	ppc_md.init_IRQ = mpc86xx_hpcn_init_irq;
	ppc_md.get_irq = openpic_get_irq;

	ppc_md.restart = mpc8641hpcn_restart;
	ppc_md.power_off = mpc86xx_power_off;
	ppc_md.halt = mpc86xx_halt;

	ppc_md.find_end_of_memory = mpc86xx_find_end_of_memory;

	ppc_md.time_init = mpc8641hpcn_time_init;
	ppc_md.set_rtc_time = todc_set_rtc_time;
	ppc_md.get_rtc_time = todc_get_rtc_time;
	ppc_md.nvram_read_val = todc_mc146818_read_val;
	ppc_md.nvram_write_val = todc_mc146818_write_val;

	ppc_md.calibrate_decr = mpc86xx_calibrate_decr;
	ppc_md.setup_io_mappings = mpc8641hpcn_map_io;
#if defined(CONFIG_SERIAL_8250) && defined(CONFIG_SERIAL_TEXT_DEBUG)
	ppc_md.progress = gen550_progress;
#endif	/* CONFIG_SERIAL_8250 && CONFIG_SERIAL_TEXT_DEBUG */

#ifdef CONFIG_SMP
	mpc86xx_smp_init();
#endif

	if (ppc_md.progress)
		ppc_md.progress("mpc8641hpcn_init(): exit", 0);
}
