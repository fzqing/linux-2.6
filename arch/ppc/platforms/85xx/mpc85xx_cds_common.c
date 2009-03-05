/*
 * arch/ppc/platform/85xx/mpc85xx_cds_common.c
 *
 * MPC85xx CDS board specific routines
 *
 * Maintainer: Kumar Gala <kumar.gala@freescale.com>
 *
 * Copyright 2004 Freescale Semiconductor, Inc
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
#include <linux/ide.h>
#include <linux/irq.h>
#include <linux/seq_file.h>
#include <linux/serial.h>
#include <linux/module.h>
#include <linux/root_dev.h>
#include <linux/initrd.h>
#include <linux/tty.h>
#include <linux/serial_core.h>
#include <linux/fsl_devices.h>

#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/atomic.h>
#include <asm/time.h>
#include <asm/todc.h>
#include <asm/io.h>
#include <asm/machdep.h>
#include <asm/open_pic.h>
#include <asm/i8259.h>
#include <asm/bootinfo.h>
#include <asm/pci-bridge.h>
#include <asm/mpc85xx.h>
#include <asm/irq.h>
#include <asm/immap_85xx.h>
#include <asm/immap_cpm2.h>
#include <asm/ppc_sys.h>
#include <asm/kgdb.h>

#ifdef CONFIG_MTD
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#endif

#include <mm/mmu_decl.h>
#include <syslib/cpm2_pic.h>
#include <syslib/ppc85xx_common.h>
#include <syslib/gen550.h>
#include <syslib/ppc85xx_setup.h>


#ifndef CONFIG_PCI
unsigned long isa_io_base = 0;
unsigned long isa_mem_base = 0;
#endif

#ifdef CONFIG_MTD
static struct mtd_partition mpc85xx_cds_partitions[] = {
	{
		.name =   "flash bank 2",
		.offset = 0,
		.size =   0x800000,
	},
	{
		.name =   "flash boot bank - Kernel",
		.offset = MTDPART_OFS_APPEND,
		.size =   0x200000,
		.mask_flags = MTD_WRITEABLE,  /* force read-only */
	},
	{
		.name =   "flash boot bank - Ram Disk",
		.offset = MTDPART_OFS_APPEND,
		.size =   0x580000,
		.mask_flags = MTD_WRITEABLE,  /* force read-only */
	},
	{
		.name =   "flash boot bank - firmware",
		.offset = MTDPART_OFS_APPEND,
		.size =   0x080000,
		.mask_flags = MTD_WRITEABLE,  /* force read-only */
	}
};

#define number_partitions       (sizeof(mpc85xx_cds_partitions)/sizeof(struct mtd_partition))
#endif

extern void cpm2_reset(void);

extern unsigned long total_memory;      /* in mm/init */

extern int pci_remove_device_safe(struct pci_dev *dev);

extern void abort(void);

unsigned char __res[sizeof (bd_t)];

static int cds_pci_slot = 2;
static volatile u8 * cadmus;

#if defined(CONFIG_PCI) && !defined(CONFIG_PEX)
static int Tundra_found = 0;
static int Tundra_bus = 0;
static int Tundra_devfn = 0;

/* We can determine which Rev of the Arcadia we are on by looking at the
 * type of PCI-to-PCI bridge we have and the rev of the ARC chip. However,
 * the PCI-to-PCI bridge can be disabled and the ARC chip is behind the bridge.
 * So, when the PCI-to-PCI bridge is disabled, we need a little help.
 */
#if defined(CONFIG_ARCADIA_X2)
static int arcadia_rev = 0x20; /* Assume 2 */
#elif defined(CONFIG_ARCADIA_X30)
static int arcadia_rev = 0x30; /* Assume 3.0 */
#elif defined(CONFIG_ARCADIA_X31)
static int arcadia_rev = 0x31; /* Assume 3.1 */
#else
#error Missing Arcadia revision definition
#endif
#endif /* CONFIG_PCI */

/* Internal interrupts are all Level Sensitive, and Positive Polarity */
static u_char mpc85xx_cds_openpic_initsenses[] __initdata = {
	MPC85XX_INTERNAL_IRQ_SENSES,
#if defined(CONFIG_PCI)
#if defined(CONFIG_PEX)
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* External 0: PEX INTA  */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* External 1: PEX INTB  */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* External 2: PEX INTC  */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* External 3: PEX INTD  */
#else
        (IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),      /* External 0: PCI1 slot */
        (IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),      /* External 1: PCI1 slot */
        (IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),      /* External 2: PCI1 slot */
        (IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),      /* External 3: PCI1 slot */
#endif
#else
        0x0,                            /* External  0: */
        0x0,                            /* External  1: */
        0x0,                            /* External  2: */
        0x0,                            /* External  3: */
#endif
        0x0,                            /* External  4: */
        (IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),      /* External 5: PHY */
        0x0,                            /* External  6: */
        0x0,                            /* External  7: */
        0x0,                            /* External  8: */
        0x0,                            /* External  9: */
        0x0,                            /* External 10: */
#if defined(CONFIG_85xx_PCI2) && defined(CONFIG_PCI)
        (IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),      /* External 11: PCI2 slot 0 */
#else
        0x0,                            /* External 11: */
#endif
};

/* ************************************************************************ */
int
mpc85xx_cds_show_cpuinfo(struct seq_file *m)
{
        uint pvid, svid, phid1;
        uint memsize = total_memory;
        bd_t *binfo = (bd_t *) __res;
        unsigned int freq;

        /* get the core frequency */
        freq = binfo->bi_intfreq;

        pvid = mfspr(PVR);
        svid = mfspr(SVR);

        seq_printf(m, "Vendor\t\t: Freescale Semiconductor\n");
	seq_printf(m, "Machine\t\t: CDS - MPC%s (%x)\n", cur_ppc_sys_spec->ppc_sys_name, cadmus[CM_VER]);
	seq_printf(m, "clock\t\t: %dMHz\n", freq / 1000000);
        seq_printf(m, "PVR\t\t: 0x%x\n", pvid);
        seq_printf(m, "SVR\t\t: 0x%x\n", svid);

        /* Display cpu Pll setting */
        phid1 = mfspr(HID1);
        seq_printf(m, "PLL setting\t: 0x%x\n", ((phid1 >> 24) & 0x3f));

        /* Display the amount of memory */
        seq_printf(m, "Memory\t\t: %d MB\n", memsize / (1024 * 1024));

        return 0;
}

#ifdef CONFIG_CPM2
static irqreturn_t cpm2_cascade(int irq, void *dev_id, struct pt_regs *regs)
{
	while((irq = cpm2_get_irq(regs)) >= 0)
		__do_IRQ(irq, regs);
	return IRQ_HANDLED;
}

static struct irqaction cpm2_irqaction = {
	.handler = cpm2_cascade,
	.flags = SA_INTERRUPT|SA_SHIRQ,
	.mask = CPU_MASK_NONE,
	.name = "cpm2_cascade",
};
#endif /* CONFIG_CPM2 */

#if defined(CONFIG_PCI) && !defined(CONFIG_PEX)
static irqreturn_t i8259_cascade(int irq, void *dev_id, struct pt_regs *regs)
{
	while((irq = i8259_irq(regs)) >= 0)
		__do_IRQ(irq, regs);
	return IRQ_HANDLED;
}

static struct irqaction i8259_irqaction = {
	.handler = i8259_cascade,
	.flags = SA_INTERRUPT|SA_SHIRQ,
	.mask = CPU_MASK_NONE,
	.name = "i8259_cascade",
};

static __init void
mpc85xx_cds_config_i8259(void)
{
 	int i;

 	for (i = 0; i < NUM_8259_INTERRUPTS; i++)
 		irq_desc[i].handler = &i8259_pic;

 	i8259_init(0);

	setup_irq(PIRQ0A, &i8259_irqaction);
}
#endif /* CONFIG_PCI && !CONFIG_PEX */

void __init
mpc85xx_cds_init_IRQ(void)
{
	bd_t *binfo = (bd_t *) __res;

        /* Determine the Physical Address of the OpenPIC regs */
        phys_addr_t OpenPIC_PAddr = binfo->bi_immr_base + MPC85xx_OPENPIC_OFFSET;
        OpenPIC_Addr = ioremap(OpenPIC_PAddr, MPC85xx_OPENPIC_SIZE);
        OpenPIC_InitSenses = mpc85xx_cds_openpic_initsenses;
        OpenPIC_NumInitSenses = sizeof (mpc85xx_cds_openpic_initsenses);

        /* Skip reserved space and internal sources */
#ifdef CONFIG_MPC8548
	openpic_set_sources(0, 48, OpenPIC_Addr + 0x10200);
#else
        openpic_set_sources(0, 32, OpenPIC_Addr + 0x10200);
#endif
        /* Map PIC IRQs 0-11 */
	openpic_set_sources(48, 12, OpenPIC_Addr + 0x10000);

        /* we let openpic interrupts starting from an offset, to
         * leave space for cascading interrupts underneath.
         */
        openpic_init(MPC85xx_OPENPIC_IRQ_OFFSET);

#ifdef CONFIG_CPM2
	/* Setup CPM2 PIC */
	cpm2_init_IRQ();

	setup_irq(MPC85xx_IRQ_CPM, &cpm2_irqaction);
#endif

        return;
}

#ifdef CONFIG_RAPIDIO
void
platform_rio_init(void)
{
	mpc85xx_rio_setup(MPC85XX_RIO_MEM, MPC85XX_RIO_MEM_SIZE);
}
#endif /* CONFIG_RAPIDIO */

#ifdef CONFIG_PCI
/*
 * interrupt routing
 */
int
mpc85xx_map_irq(struct pci_dev *dev, unsigned char idsel, unsigned char pin)
{
#ifdef CONFIG_PEX
	char pci_irq_table[][4] =
	{
		{ MPC85xx_IRQ_EXT0, MPC85xx_IRQ_EXT1, MPC85xx_IRQ_EXT2, MPC85xx_IRQ_EXT3 },
	};
	const long min_idsel = 0, max_idsel = 0, irqs_per_slot = 4;

	return PCI_IRQ_TABLE_LOOKUP;
#else
	struct pci_controller *hose = pci_bus_to_hose(dev->bus->number);
	if (!hose->index)
	{
               /* Handle PCI1 interrupts */
                       /*
                        *      PCI IDSEL/INTPIN->INTLINE
                        *        A      B      C      D
                        */

                       /* Note IRQ assignment for slots is based on which slotthe elysium is
                        * in -- in this setup elysium is in slot #2 (this PIRQA as first
                        * interrupt on slot */

		char (*pci_irq_table)[4];

		char pci_irq_table_x31 [][4] = {
			{ 0, 0, 0, 0 }, /* 16 - */
			{ 0, 0, 0, 0 }, /* 17 - */
			{ 0, 0, 0, 0 }, /* 18 - */
			{ 0, 0, 0, 0 }, /* 19 - */
			{ 0, 1, 2, 3 }, /* 20 - Slot 1 */
			{ 1, 2, 3, 0 }, /* 21 - Slot 2 */
			{ 2, 3, 0, 1 }, /* 22 - slot 3 */
			{ 0, 0, 0, 0 }, /* 23-- */
			{ 0, 1, 2, 3 }, /* 24 - slot 4 */
			{ 0, 0, 0, 0 }, /* 25-- */
			{ 0, 0, 0, 0 }, /* 26-- */
			{ 0, 0, 0, 0 }, /* 27-- */
			{ 0, 1, 2, 3 }, /* 28 - Tsi310 bridge */
		};

		char pci_irq_table_x30[][4] = {
			{ 0, 0, 0, 0 }, /* 16 - */
			{ 0, 0, 0, 0 }, /* 17 - */
			{ 0, 1, 2, 3 }, /* 18 - Tsi310 Bridge */
			{ 0, 1, 2, 3 }, /* 19 - Slot 2 */
			{ 0, 1, 2, 3 }, /* 20 - Slot 3 */
			{ 0, 1, 2, 3 }, /* 21 - Slot 4 */
			{ 0, 1, 2, 3 }, /* 22 - slot 5 */
			{ 0, 0, 0, 0 }, /* 23 - */
			{ 0, 0, 0, 0 }, /* 24 - */
			{ 0, 0, 0, 0 }, /* 25 - */
			{ 0, 0, 0, 0 }, /* 26 - */
			{ 0, 0, 0, 0 }, /* 27 - */
			{ 0, 0, 0, 0 }, /* 28 - */
		};

		char pci_irq_table_x2[][4] = {
			{ 0, 1, 2, 3 }, /* 16 - PMC */
			{ 0, 1, 2, 3 }, /* 17 P2P (Tsi320) */
			{ 0, 1, 2, 3 }, /* 18 - Slot 1 */
			{ 1, 2, 3, 0 }, /* 19 - Slot 2 */
			{ 2, 3, 0, 1 }, /* 20 - Slot 3 */
			{ 3, 0, 1, 2 }, /* 21 - Slot 4 */
			{ 0, 0, 0, 0 }, /* 22 - */
			{ 0, 0, 0, 0 }, /* 23 - */
			{ 0, 0, 0, 0 }, /* 24 - */
			{ 0, 0, 0, 0 }, /* 25 - */
			{ 0, 0, 0, 0 }, /* 26 - */
			{ 0, 0, 0, 0 }, /* 27 - */
			{ 0, 0, 0, 0 }, /* 28 - */
		};

		const long min_idsel = 16 , max_idsel = 28 , irqs_per_slot = 4;
		int i, j;

		if (arcadia_rev == 0x20) {
			pci_irq_table = pci_irq_table_x2;
			for (i = 0; i <= (max_idsel - min_idsel); i++)
				for (j = 0; j < 4; j++)
					pci_irq_table[i][j] =
						((pci_irq_table[i][j] + 5 -
						  cds_pci_slot) & 0x3) + PIRQ0A;

		} else {
			pci_irq_table = (arcadia_rev == 0x30) ?
				pci_irq_table_x30 : pci_irq_table_x31;

			for (i = 0; i <= (max_idsel - min_idsel); i++)
				for (j = 0; j < 4; j++)
					pci_irq_table[i][j] =
						pci_irq_table[i][j] + PIRQ0A;
		}
		return PCI_IRQ_TABLE_LOOKUP;
	} else {
		/* Handle PCI2 interrupts (if we have one) */
		char pci_irq_table[][4] =
		{
			/*
			 * We only have one slot */
			{ PIRQ1A, PIRQ0B, PIRQ0C, PIRQ0D }, /* 21 - slot 0 */
		};

		const long min_idsel = 21, max_idsel = 21, irqs_per_slot = 4;

		return PCI_IRQ_TABLE_LOOKUP;
	}
#endif
}

#define ARCADIA_X2_BRIDGE_IDSEL		17
#define ARCADIA_X3_BRIDGE_IDSEL		18
#define ARCADIA_X31_BRIDGE_IDSEL	28
#define ARCADIA_X2_2ND_BRIDGE_IDSEL	3
#define ARCADIA_X3_2ND_BRIDGE_IDSEL	1

#ifndef CONFIG_PEX
#ifdef CONFIG_PPC_INDIRECT_PCI_BE
#define PCI_CFG_OUT out_be32
#else
#define PCI_CFG_OUT out_le32
#endif

static void
mpc85xx_private_read_config_word(struct pci_controller *hose, u8 bus,
			     u8 devfn, int offset, u16 *val)
{
	u8 cfg_type = 0;
	volatile unsigned char *cfg_data;

	if (bus == hose->first_busno)
		bus = 0;

	PCI_CFG_OUT(hose->cfg_addr,
		 (0x80000000 | (bus << 16)
		  | (devfn << 8) | ((offset & 0xfc) | cfg_type)));

	cfg_data = hose->cfg_data + (offset & 3);
	*val = in_le16((u16 *)cfg_data);
}
#endif /* CONFIG_PEX */

int
mpc85xx_exclude_device(u_char bus, u_char devfn)
{
#ifdef CONFIG_PEX
	return PCIBIOS_SUCCESSFUL;
#else
	struct pci_controller *hose;

	hose = pci_bus_to_hose(bus);
	if (hose == NULL) {
		printk("Unable to locate hose\n");
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	/* Skip the host bridge on both hoses. */
	if (hose->first_busno == bus && PCI_SLOT(devfn) == 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	/* Skip the seconday IDSEL for the Tundra P2P bridge. */
	if (Tundra_found && hose->index == 0 && bus != hose->first_busno &&
	    (PCI_SLOT(devfn) == ARCADIA_X2_2ND_BRIDGE_IDSEL ||
	     PCI_SLOT(devfn) == ARCADIA_X3_2ND_BRIDGE_IDSEL)) {
		u16 bus2;

		mpc85xx_private_read_config_word(hose, Tundra_bus,
				Tundra_devfn, PCI_PRIMARY_BUS,
				&bus2);
		bus2 = (bus2 >> 8) & 0xff;
		if (bus == bus2)
			return PCIBIOS_DEVICE_NOT_FOUND;
	}

	/*
	 * If the Tundra Bridge has not been found, see if it is this device.
	 * We know it is on the root bus of hose A, but it can be at any of
	 * 3 different IDSEL values and some of those IDSELs are valid slots
	 * on some of the Arcadia Revs.
	 */
	if (!Tundra_found && hose->index == 0 && hose->first_busno == bus &&
	    (PCI_SLOT(devfn) == ARCADIA_X2_BRIDGE_IDSEL ||
	     PCI_SLOT(devfn) == ARCADIA_X3_BRIDGE_IDSEL ||
	     PCI_SLOT(devfn) == ARCADIA_X31_BRIDGE_IDSEL)) {
		u16 venid, devid;

		mpc85xx_private_read_config_word(hose, bus, devfn,
				PCI_VENDOR_ID, &venid);
		mpc85xx_private_read_config_word(hose, bus, devfn,
				PCI_DEVICE_ID, &devid);
		/*
		 * To confuse everything, the Tsi320 is a Tundra design,
		 * but the Tsi310 is an IBM design, hence the odd check.
		 */
		if ((venid == PCI_VENDOR_ID_IBM &&
			devid == PCI_DEVICE_ID_IBM_TSI310) ||
		    (venid == PCI_VENDOR_ID_TUNDRA &&
		     	devid == PCI_DEVICE_ID_TUNDRA_TSI320)) {
			Tundra_bus = bus;
			Tundra_devfn = devfn;
			Tundra_found = 1;
		}
	}
	return PCIBIOS_SUCCESSFUL;
#endif /* CONFIG_PEX */
}

#ifndef CONFIG_PEX
static void __init
mpc85xx_cds_fixup_arc(struct pci_dev *dev)
{
	u32 rev;

	pci_read_config_dword(dev, PCI_CLASS_REVISION, &rev);
	rev &= 0xff;
	arcadia_rev = (rev == 0x04) ? 0x30 : rev;
	printk(KERN_DEBUG "Found Arcadia Rev %01d.%01d\n",
		(arcadia_rev >> 4) & 0xf, (arcadia_rev & 0xf));

}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_FREESCALE, PCI_DEVICE_ID_ARCADIA_ARC,
		mpc85xx_cds_fixup_arc);

static void __init
mpc85xx_cds_fixup_tundra(struct pci_dev *dev)
{
	struct pci_controller *hose;
	u16 cmd;
	unsigned long start, end;

	hose = dev->sysdata;

	if (!hose) {
		printk("%s: No Hose?\n", __FUNCTION__);
		return;
	}

	start = hose->io_resource.start;
	end = start + 0x10000 - 1;

	pci_read_config_word(dev, PCI_COMMAND, &cmd);
	pci_write_config_word(dev, PCI_COMMAND,
			      cmd & ~(PCI_COMMAND_IO | PCI_COMMAND_MEMORY));

	pci_write_config_byte(dev, PCI_IO_LIMIT,
			(end >> 8) & PCI_IO_RANGE_MASK);
	pci_write_config_word(dev, PCI_IO_LIMIT_UPPER16,
			end >> 16);

	pci_write_config_byte(dev, PCI_IO_BASE,
			(start >> 8) & PCI_IO_RANGE_MASK);
	pci_write_config_word(dev, PCI_IO_BASE_UPPER16,
			start >> 16);
	pci_write_config_word(dev, PCI_COMMAND, cmd);

	/* If we found a Tundra chip, we're on a rev 2.x board. */
	if (dev->vendor == PCI_VENDOR_ID_TUNDRA) {
		arcadia_rev = 0x20;
		printk(KERN_DEBUG "Found Arcadia Rev 2\n");
	}

}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_TUNDRA, PCI_DEVICE_ID_TUNDRA_TSI320,
			mpc85xx_cds_fixup_tundra);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_IBM, PCI_DEVICE_ID_IBM_TSI310,
			mpc85xx_cds_fixup_tundra);

#define IDE_BASE_0 0xfff8
#define IDE_BASE_1 0xfff4
#define IDE_BASE_2 0xffe8
#define IDE_BASE_3 0xffe4
#define IDE_BASE_4 0xffd0

#if defined(CONFIG_BLK_DEV_IDE) || defined(CONFIG_BLK_DEV_IDE_MODULE)
/*
 * IDE stuff.
 */
static int mpc85xx_cds_ide_default_irq(unsigned long base)
{
	switch (base) {
	case IDE_BASE_0:
		return 14;
	case IDE_BASE_1:
		return 15;
	default:
		return 0;
	}
}

static unsigned long mpc85xx_cds_ide_default_io_base(int index)
{
	switch (index) {
	case 0:
		return IDE_BASE_0;
	case 1:
		return IDE_BASE_1;
	default:
		return 0;
	}
}

static void __init
mpc85xx_cds_ide_init_hwif_ports(hw_regs_t * hw, unsigned long data_port,
			  unsigned long ctrl_port, int *irq)
{
	unsigned long reg = data_port;
	int i;

	for (i = IDE_DATA_OFFSET; i <= IDE_STATUS_OFFSET; i++) {
		hw->io_ports[i] = reg;
		reg += 1;
	}

	if (ctrl_port)
		hw->io_ports[IDE_CONTROL_OFFSET] = ctrl_port;
	else
		switch (data_port) {
			case IDE_BASE_0:
				hw->io_ports[IDE_CONTROL_OFFSET] =
					IDE_BASE_2 + 2;
				break;
			case IDE_BASE_2:
				hw->io_ports[IDE_CONTROL_OFFSET] =
					IDE_BASE_3 + 2;
				break;
			default:
				break;
		}
	if (irq != NULL)
		*irq = mpc85xx_cds_ide_default_irq(data_port);
}
#endif /* defined(CONFIG_BLK_DEV_IDE) || defined(CONFIG_BLK_DEV_IDE_MODULE) */

static void __init
mpc85xx_cds_enable_via(struct pci_dev *dev)
{
	/* Enable USB and IDE functions */
	pci_write_config_byte(dev, 0x48, 0x08);

	/* Connect up the i8259 IRQ handler */
	mpc85xx_cds_config_i8259();
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA_82C686,
			mpc85xx_cds_enable_via);

static void __init
mpc85xx_cds_fixup_via_ide(struct pci_dev *dev)
{
	u_char		c;

	/*
	 * U-Boot does not set the enable bits
	 * for the IDE device. Force them on here.
	 */
	pci_read_config_byte(dev, 0x40, &c);
	c |= 0x03; /* IDE: Chip Enable Bits */
	pci_write_config_byte(dev, 0x40, c);

	pci_write_config_byte(dev, PCI_INTERRUPT_LINE, 14);
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_0, IDE_BASE_0);
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_1, IDE_BASE_1);
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_2, IDE_BASE_2);
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_3, IDE_BASE_3);
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_4, IDE_BASE_4);

}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA_82C586_1,
			mpc85xx_cds_fixup_via_ide);

static void __init
mpc85xx_cds_fixup_via_usb(struct pci_dev *dev)
{
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_4,
			(PCI_FUNC(dev->devfn) == 2) ? 0xffa0 : 0xff80);
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA_82C586_2,
			mpc85xx_cds_fixup_via_usb);

static void __init
mpc85xx_cds_fixup_via_pwr(struct pci_dev *dev)
{
	/* Function 5, Power Management  (actually AC97 sound */
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_0, 0xfe00);
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_1, 0xfdfc);
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_2, 0xfdf8);

}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA_82C686_5,
			mpc85xx_cds_fixup_via_pwr);

static void __init
mpc85xx_cds_fixup_via_ac97(struct pci_dev *dev)
{
	/* Function 6, AC97 Interface (actually modem) */
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_0, 0xfc00);

}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA_82C686_6,
			mpc85xx_cds_fixup_via_ac97);
#endif /* CONFIG_PEX */

void __init
mpc85xx_cds_pcibios_fixup(void)
{
        struct pci_dev *dev = NULL;

	if ((dev = pci_find_device(PCI_VENDOR_ID_FREESCALE,
				   PCI_DEVICE_ID_ARCADIA_ARC, NULL))) {
		printk(KERN_DEBUG "%s: Found ARC\n", __FUNCTION__);
		if (pci_remove_device_safe(dev) == 0)
			printk(KERN_DEBUG "%s: Removed ARC\n", __FUNCTION__);
	}

        if ((dev = pci_find_device(PCI_VENDOR_ID_VIA,
                                        PCI_DEVICE_ID_VIA_82C586_1, NULL))) {
		/*
		 * Since only primary interface works, force the
		 * IDE function to standard primary IDE interrupt
		 * w/ 8259 offset
		 */
                dev->irq = 14;
                pci_write_config_byte(dev, PCI_INTERRUPT_LINE, dev->irq);
        }

	/*
	 * Force legacy USB interrupt routing
	 */
        if ((dev = pci_find_device(PCI_VENDOR_ID_VIA,
                                        PCI_DEVICE_ID_VIA_82C586_2, NULL))) {
                dev->irq = 10;
                pci_write_config_byte(dev, PCI_INTERRUPT_LINE, 10);
        }

        if ((dev = pci_find_device(PCI_VENDOR_ID_VIA,
                                        PCI_DEVICE_ID_VIA_82C586_2, dev))) {
                dev->irq = 11;
                pci_write_config_byte(dev, PCI_INTERRUPT_LINE, 11);
        }
}

static void __init
mpc85xx_pci_law_fixup(void)
{
	int i, number, empty = 0;
	u32 *lawbase, lawar, target;
	int map[3] = {0, 0, 0};
	struct local_window {
		int number;
		int target;
		u32 base;
		u32 size; } lw[3][2] = {
			{{
				 .number = -1,
				 .target = 0,
				 .base = MPC85XX_PCI1_LOWER_MEM,
				 .size = (__ilog2(MPC85XX_PCI1_UPPER_MEM - MPC85XX_PCI1_LOWER_MEM + 1) - 1),
			 },
			{
				.number = -1,
				.target = 0,
				.base = MPC85XX_PCI1_IO_BASE,
				.size = (__ilog2(MPC85XX_PCI1_IO_SIZE) - 1),
			},},
			{ {
				  .number = -1,
				  .target = 1,
				  .base = MPC85XX_PCI2_LOWER_MEM,
				  .size = (__ilog2(MPC85XX_PCI2_UPPER_MEM - MPC85XX_PCI2_LOWER_MEM + 1) - 1),
			  },
			{
				.number = -1,
				.target = 1,
				.base = MPC85XX_PCI2_IO_BASE,
				.size = (__ilog2(MPC85XX_PCI2_IO_SIZE) - 1),
			},},
			{ {
				  .number = -1,
				  .target = 2,
				  .base = MPC85XX_PEX_LOWER_MEM,
				  .size = (__ilog2(MPC85XX_PEX_UPPER_MEM - MPC85XX_PEX_LOWER_MEM + 1) - 1),
			  },
			{
				.number = -1,
				.target = 2,
				.base = MPC85XX_PEX_IO_BASE,
				.size = (__ilog2(MPC85XX_PEX_IO_SIZE) - 1),
			} }
		};
	struct local_window *lwp;

	/* Search each LAW, detemine whether the bootloader
	 * open memory window for PCI1, PCI2, PEX. If not, define
	 * it. If yes, fix window size.
	 */
	lawbase = (uint32_t *)ioremap((get_ccsrbar() + 0xc00), 0x1000);

	for(i=1; i<=7; i++) {
		lawar = in_be32(lawbase + 0x4 + (i * 0x8));
		target = (lawar & 0x00f00000) >> 20;

		if ((lawar & 0x80000000) == 0) {
			empty = i;
			break;
		}
		if (target > 2)
			continue;
		lw[target][map[target]].number = i;
		map[target]++;
	}

	lwp = &lw[0][0];
	for (i=0; i<6; i++, lwp++) {
		number = lwp->number;
		if (number == -1)
			number = empty++;

		out_be32(lawbase + 0x2 + (number * 0x8), (lwp->base)>>12);
		out_be32(lawbase + 0x4 + (number * 0x8),
			 0x80000000 | (lwp->target)<<20 | (lwp->size));
	}

	iounmap(lawbase);
	return;
}
#endif /* CONFIG_PCI */

void
mpc85xx_cds_restart(char *cmd)
{
	struct pci_dev *dev;
	unsigned long i = 0x2000000;
	u_char tmp;
	u32 __iomem *rst;

	local_irq_disable();

	if ((dev = pci_find_device(PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA_82C686,
					NULL))) {

		/* make sure bit 0 (reset) is a 0 */
		outb(inb(0x92) & ~1L, 0x92);

		/* Set port 92 reset enable bit in VIA header */
		pci_read_config_byte(dev, 0x41, &tmp);
		pci_write_config_byte(dev, 0x41, tmp | (1<<5));

		/* signal a reset to system control port A - soft reset */
		outb(inb(0x92) | 1, 0x92);

		while (i != 0)
			i--;
		printk("Port A restart failed. Trying PCI restart.\n");

		/* Hmmm, Port A reset failed, try a PCI reset */
		pci_read_config_byte(dev, 0x47, &tmp);
		pci_write_config_byte(dev, 0x47, tmp | 1);

		i = 0x2000000;
		while (i != 0)
			i--;
		/* fall through into 85xx processor-based reset code. */
	}

#ifdef CONFIG_PCI
	printk("Unable to perform PCI reset. Using processor reset.\n");
	printk("NOTE: Processor reset does not reset PCI devices.\n");
#endif

	rst = ioremap((BOARD_CCSRBAR + 0xe00b0),0x100);
	out_be32(rst, 0x2); /* Set HRESET_REQ flag */

	abort();
}

TODC_ALLOC();

/* ************************************************************************
 *
 * Setup the architecture
 *
 */
static void __init
mpc85xx_cds_setup_arch(void)
{
        bd_t *binfo = (bd_t *) __res;
        unsigned int freq;
	struct gianfar_platform_data *pdata;
	struct gianfar_mdio_data *mdata;

        /* get the core frequency */
        freq = binfo->bi_intfreq;

	if (ppc_md.progress)
		ppc_md.progress("mpc85xx_cds_setup_arch()", 0);

#if !defined(CONFIG_BDI_SWITCH)
	/*
	 * The Abatron BDI JTAG debugger does not tolerate others
	 * mucking with the debug registers.
	 */
	mtspr(SPRN_DBCR0, (DBCR0_IDM));
	mtspr(SPRN_DBSR, 0xffffffff);
#endif

#ifdef CONFIG_CPM2
	cpm2_reset();
#endif

	cadmus = ioremap(CADMUS_BASE, CADMUS_SIZE);
	cds_pci_slot = ((cadmus[CM_CSR] >> 6) & 0x3) + 1;
	printk("CDS Version = %x in PCI slot %d\n", cadmus[CM_VER], cds_pci_slot);

	/* Setup TODC access */
	TODC_INIT(TODC_TYPE_DS1743,
			0,
			0,
			ioremap(CDS_RTC_ADDR, CDS_RTC_SIZE),
			8);

        /* Set loops_per_jiffy to a half-way reasonable value,
           for use until calibrate_delay gets called. */
        loops_per_jiffy = freq / HZ;

#ifdef CONFIG_PCI
 	/* VIA IDE configuration */
         ppc_md.pcibios_fixup = mpc85xx_cds_pcibios_fixup;

	/* fixup local access windows */
	 mpc85xx_pci_law_fixup();

        /* setup PCI host bridges */
        mpc85xx_setup_hose();
#endif

#ifdef CONFIG_SERIAL_8250
        mpc85xx_early_serial_map();
#endif

#ifdef CONFIG_SERIAL_TEXT_DEBUG
	/* Invalidate the entry we stole earlier the serial ports
	 * should be properly mapped */
	invalidate_tlbcam_entry(num_tlbcam_entries - 1);
#endif

	/* setup the board related info for the MDIO bus */
	mdata = (struct gianfar_mdio_data *) ppc_sys_get_pdata(MPC85xx_MDIO);

	mdata->irq[0] = MPC85xx_IRQ_EXT5;
	mdata->irq[1] = MPC85xx_IRQ_EXT5;
	mdata->irq[2] = -1;
	mdata->irq[3] = -1;
	mdata->irq[31] = -1;

	/* setup the board related information for the enet controllers */
	pdata = (struct gianfar_platform_data *) ppc_sys_get_pdata(MPC85xx_TSEC1);
	if (pdata) {
		pdata->board_flags = FSL_GIANFAR_BRD_HAS_PHY_INTR;
		pdata->bus_id = 0;
		pdata->phy_id = 0;
		memcpy(pdata->mac_addr, binfo->bi_enetaddr, 6);
	}

	pdata = (struct gianfar_platform_data *) ppc_sys_get_pdata(MPC85xx_TSEC2);
	if (pdata) {
		pdata->board_flags = FSL_GIANFAR_BRD_HAS_PHY_INTR;
		pdata->bus_id = 0;
		pdata->phy_id = 1;
		memcpy(pdata->mac_addr, binfo->bi_enet1addr, 6);
	}

	pdata = (struct gianfar_platform_data *) ppc_sys_get_pdata(MPC85xx_eTSEC1);
	if (pdata) {
		pdata->board_flags = FSL_GIANFAR_BRD_HAS_PHY_INTR;
		pdata->bus_id = 0;
		pdata->phy_id = 0;
		memcpy(pdata->mac_addr, binfo->bi_enetaddr, 6);
	}

	pdata = (struct gianfar_platform_data *) ppc_sys_get_pdata(MPC85xx_eTSEC2);
	if (pdata) {
		pdata->board_flags = FSL_GIANFAR_BRD_HAS_PHY_INTR;
		pdata->bus_id = 0;
		pdata->phy_id = 1;
		memcpy(pdata->mac_addr, binfo->bi_enet1addr, 6);
	}

	ppc_sys_device_remove(MPC85xx_eTSEC3);
	ppc_sys_device_remove(MPC85xx_eTSEC4);


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

#ifdef CONFIG_MTD
	/*
	 * Support for MTD on MPC85xx CDS. Use the generic physmap driver
	 */
	physmap_configure(0xff000000, 0x1000000, 2, NULL);
	physmap_set_partitions(mpc85xx_cds_partitions, number_partitions);
#endif
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

		/* Use the last TLB entry to map CCSRBAR to allow access to DUART regs */
		settlbcam(num_tlbcam_entries - 1, binfo->bi_immr_base,
			  binfo->bi_immr_base, MPC85xx_CCSRBAR_SIZE, _PAGE_IO, 0);

		memset(&p, 0, sizeof (p));
		p.iotype = SERIAL_IO_MEM;
		p.membase = (void *) binfo->bi_immr_base + MPC85xx_UART0_OFFSET;
		p.uartclk = binfo->bi_busfreq;

		gen550_init(0, &p);

		memset(&p, 0, sizeof (p));
		p.iotype = SERIAL_IO_MEM;
		p.membase = (void *) binfo->bi_immr_base + MPC85xx_UART1_OFFSET;
		p.uartclk = binfo->bi_busfreq;

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
#endif                          /* CONFIG_BLK_DEV_INITRD */

        /* Copy the kernel command line arguments to a safe place. */

        if (r6) {
                *(char *) (r7 + KERNELBASE) = 0;
                strcpy(cmd_line, (char *) (r6 + KERNELBASE));
        }

	identify_ppc_sys_by_id(mfspr(SVR));

        /* setup the PowerPC module struct */
        ppc_md.setup_arch = mpc85xx_cds_setup_arch;
        ppc_md.show_cpuinfo = mpc85xx_cds_show_cpuinfo;

        ppc_md.init_IRQ = mpc85xx_cds_init_IRQ;
        ppc_md.get_irq = openpic_get_irq;

        ppc_md.restart = mpc85xx_cds_restart;
        ppc_md.power_off = mpc85xx_power_off;
        ppc_md.halt = mpc85xx_halt;

        ppc_md.find_end_of_memory = mpc85xx_find_end_of_memory;

        ppc_md.calibrate_decr = mpc85xx_calibrate_decr;

	ppc_md.time_init = todc_time_init;
	ppc_md.set_rtc_time = todc_set_rtc_time;
	ppc_md.get_rtc_time = todc_get_rtc_time;

	ppc_md.nvram_read_val = todc_direct_read_val;
	ppc_md.nvram_write_val = todc_direct_write_val;

#ifdef CONFIG_PCI
#if defined(CONFIG_BLK_DEV_IDE) || defined(CONFIG_BLK_DEV_IDE_MODULE)
	ppc_ide_md.default_irq = mpc85xx_cds_ide_default_irq;
	ppc_ide_md.default_io_base = mpc85xx_cds_ide_default_io_base;
	ppc_ide_md.ide_init_hwif = mpc85xx_cds_ide_init_hwif_ports;
#endif
#endif

#if defined(CONFIG_SERIAL_8250) && defined(CONFIG_SERIAL_TEXT_DEBUG)
        ppc_md.progress = gen550_progress;
#endif /* CONFIG_SERIAL_8250 && CONFIG_SERIAL_TEXT_DEBUG */

        if (ppc_md.progress)
                ppc_md.progress("mpc85xx_cds_init(): exit", 0);

        return;
}
