/*
 * arch/ppc/syslib/ppc85xx_setup.c
 *
 * MPC85XX common board code
 *
 * Maintainer: Kumar Gala <kumar.gala@freescale.com>
 *
 * Copyright 2004 Freescale Semiconductor Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/config.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/serial.h>
#include <linux/tty.h>	/* for linux/serial_core.h */
#include <linux/serial_core.h>
#include <linux/serial_8250.h>

#include <asm/time.h>
#include <asm/mpc85xx.h>
#include <asm/immap_85xx.h>
#include <asm/mmu.h>
#include <asm/ppc_sys.h>
#include <asm/kgdb.h>

#include <syslib/ppc85xx_setup.h>

extern void kgdb8250_add_port(int i, struct uart_port *serial_req);

extern void abort(void);

/* Return the amount of memory */
unsigned long __init
mpc85xx_find_end_of_memory(void)
{
        bd_t *binfo;

        binfo = (bd_t *) __res;

        return binfo->bi_memsize;
}

/* The decrementer counts at the system (internal) clock freq divided by 8 */
void __init
mpc85xx_calibrate_decr(void)
{
        bd_t *binfo = (bd_t *) __res;
        unsigned int freq, divisor;

        /* get the core frequency */
        freq = binfo->bi_busfreq;

        /* The timebase is updated every 8 bus clocks, HID0[SEL_TBCLK] = 0 */
        divisor = 8;
        tb_ticks_per_jiffy = freq / divisor / HZ;
        tb_to_us = mulhwu_scale_factor(freq / divisor, 1000000);
        us_to_tb = (freq / divisor) / 1000000;

	/* Set the time base to zero */
	mtspr(SPRN_TBWL, 0);
	mtspr(SPRN_TBWU, 0);

	/* Clear any pending timer interrupts */
	mtspr(SPRN_TSR, TSR_ENW | TSR_WIS | TSR_DIS | TSR_FIS);

	/* Enable decrementer interrupt */
	mtspr(SPRN_TCR, TCR_DIE);
}

void __init
mpc85xx_early_serial_map(void)
{
#if defined(CONFIG_SERIAL_TEXT_DEBUG) || defined(CONFIG_KGDB_8250)
	struct uart_port serial_req;
#endif
	struct plat_serial8250_port *pdata;
	bd_t *binfo = (bd_t *) __res;
	pdata = (struct plat_serial8250_port *) ppc_sys_get_pdata(MPC85xx_DUART);

	/* Setup serial port access */
	pdata[0].uartclk = binfo->bi_busfreq;
	pdata[0].mapbase += binfo->bi_immr_base;
	pdata[0].membase = ioremap(pdata[0].mapbase, MPC85xx_UART0_SIZE);

#if defined(CONFIG_SERIAL_TEXT_DEBUG) || defined(CONFIG_KGDB_8250)
	memset(&serial_req, 0, sizeof (serial_req));
	serial_req.iotype = SERIAL_IO_MEM;
	serial_req.mapbase = pdata[0].mapbase;
	serial_req.membase = pdata[0].membase;
	serial_req.regshift = 0;
	serial_req.uartclk = pdata[0].uartclk;
	serial_req.irq = pdata[0].irq;
	serial_req.line = pdata[0].line;
	serial_req.flags = pdata[0].flags;

	if (early_serial_setup(&serial_req) != 0)
		printk("Early serial init of port 0 failed\n");

#if defined(CONFIG_SERIAL_TEXT_DEBUG)
	gen550_init(0, &serial_req);
#endif
#ifdef CONFIG_KGDB_8250
	kgdb8250_add_port(0, &serial_req);
#endif
#endif

	pdata[1].uartclk = binfo->bi_busfreq;
	pdata[1].mapbase += binfo->bi_immr_base;
	pdata[1].membase = ioremap(pdata[1].mapbase, MPC85xx_UART1_SIZE);

#if defined(CONFIG_SERIAL_TEXT_DEBUG) || defined(CONFIG_KGDB_8250)
	/* Assume gen550_init() doesn't modify serial_req */
	serial_req.mapbase = pdata[1].mapbase;
	serial_req.membase = pdata[1].membase;
	serial_req.line = pdata[1].line;

	if (early_serial_setup(&serial_req) != 0)
		printk("Early serial init of port 1 failed\n");

#if defined(CONFIG_SERIAL_TEXT_DEBUG)
	gen550_init(1, &serial_req);
#endif
#ifdef CONFIG_KGDB_8250
	kgdb8250_add_port(1, &serial_req);
#endif
#endif
}

void
mpc85xx_restart(char *cmd)
{
#ifdef CONFIG_MPC8548_CDS
	u32 *pMem = (u32*) ioremap((BOARD_CCSRBAR + 0xe00b0),0x100);
	*pMem = 0x2; /* Set HRESET_REQ flag */
#endif
	local_irq_disable();
	abort();
}

void
mpc85xx_power_off(void)
{
	local_irq_disable();
	for(;;);
}

void
mpc85xx_halt(void)
{
	local_irq_disable();
	for(;;);
}

#ifdef CONFIG_PCI

#if defined(CONFIG_MPC8555_CDS) || defined(CONFIG_MPC8548_CDS)
extern void mpc85xx_cds_enable_via(struct pci_controller *hose);
extern void mpc85xx_cds_fixup_via(struct pci_controller *hose);
#endif

extern int mpc85xx_map_irq(struct pci_dev *dev, unsigned char idsel, unsigned char pin);
extern int mpc85xx_exclude_device(u_char bus, u_char devfn);

#ifndef CONFIG_PEX
static void __init
mpc85xx_setup_pci1(struct pci_controller *hose)
{
	struct ccsr_pci *pci;
	struct ccsr_guts *guts;
	unsigned short temps;
	bd_t *binfo = (bd_t *) __res;

	pci = ioremap(binfo->bi_immr_base + MPC85xx_PCI1_OFFSET,
		    MPC85xx_PCI1_SIZE);

	guts = ioremap(binfo->bi_immr_base + MPC85xx_GUTS_OFFSET,
		    MPC85xx_GUTS_SIZE);

	early_read_config_word(hose, 0, 0, PCI_COMMAND, &temps);
	temps |= PCI_COMMAND_SERR | PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY;
	early_write_config_word(hose, 0, 0, PCI_COMMAND, temps);

#define PORDEVSR_PCI	(0x00800000)	/* PCI Mode */
	if (guts->pordevsr & PORDEVSR_PCI) {
 		early_write_config_byte(hose, 0, 0, PCI_LATENCY_TIMER, 0x80);
 	} else {
		/* PCI-X init */
		temps = PCI_X_CMD_MAX_SPLIT | PCI_X_CMD_MAX_READ
			| PCI_X_CMD_ERO | PCI_X_CMD_DPERR_E;
		early_write_config_word(hose, 0, 0, PCIX_COMMAND, temps);
	}

	/* Disable all windows (except powar0 since its ignored) */
	pci->powar1 = 0;
	pci->powar2 = 0;
	pci->powar3 = 0;
	pci->powar4 = 0;
	pci->piwar1 = 0;
	pci->piwar2 = 0;
	pci->piwar3 = 0;

	/* Setup Phys:PCI 1:1 outbound mem window @ MPC85XX_PCI1_LOWER_MEM */
	pci->potar1 = (MPC85XX_PCI1_LOWER_MEM >> 12) & 0x000fffff;
	pci->potear1 = 0x00000000;
	pci->powbar1 = (MPC85XX_PCI1_LOWER_MEM >> 12) & 0x000fffff;
	/* Enable, Mem R/W */
	pci->powar1 = 0x80044000 |
	   (__ilog2(MPC85XX_PCI1_UPPER_MEM - MPC85XX_PCI1_LOWER_MEM + 1) - 1);

	/* Setup outbound IO windows @ MPC85XX_PCI1_IO_BASE */
	pci->potar2 = (MPC85XX_PCI1_LOWER_IO >> 12) & 0x000fffff;
	pci->potear2 = 0x00000000;
	pci->powbar2 = (MPC85XX_PCI1_IO_BASE >> 12) & 0x000fffff;
	/* Enable, IO R/W */
	pci->powar2 = 0x80088000 | (__ilog2(MPC85XX_PCI1_IO_SIZE) - 1);

	/* Setup 2G inbound Memory Window @ 0 */
	pci->pitar1 = 0x00000000;
	pci->piwbar1 = 0x00000000;
	pci->piwar1 = 0xa0f5501e;	/* Enable, Prefetch, Local
					   Mem, Snoop R/W, 2G */
}

#ifdef CONFIG_85xx_PCI2
static void __init
mpc85xx_setup_pci2(struct pci_controller *hose)
{
	struct ccsr_pci *pci;
	unsigned short temps;
	bd_t *binfo = (bd_t *) __res;

	pci = ioremap(binfo->bi_immr_base + MPC85xx_PCI2_OFFSET,
		    MPC85xx_PCI2_SIZE);

	early_read_config_word(hose, hose->first_busno, 0, PCI_COMMAND, &temps);
	temps |= PCI_COMMAND_SERR | PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY;
	early_write_config_word(hose, hose->first_busno, 0, PCI_COMMAND, temps);
	early_write_config_byte(hose, hose->first_busno, 0, PCI_LATENCY_TIMER, 0x80);

	/* Disable all windows (except powar0 since its ignored) */
	pci->powar1 = 0;
	pci->powar2 = 0;
	pci->powar3 = 0;
	pci->powar4 = 0;
	pci->piwar1 = 0;
	pci->piwar2 = 0;
	pci->piwar3 = 0;

	/* Setup Phys:PCI 1:1 outbound mem window @ MPC85XX_PCI2_LOWER_MEM */
	pci->potar1 = (MPC85XX_PCI2_LOWER_MEM >> 12) & 0x000fffff;
	pci->potear1 = 0x00000000;
	pci->powbar1 = (MPC85XX_PCI2_LOWER_MEM >> 12) & 0x000fffff;
	/* Enable, Mem R/W */
	pci->powar1 = 0x80044000 |
	   (__ilog2(MPC85XX_PCI2_UPPER_MEM - MPC85XX_PCI2_LOWER_MEM + 1) - 1);

	/* Setup outbound IO windows @ MPC85XX_PCI2_IO_BASE */
	pci->potar2 = (MPC85XX_PCI2_LOWER_IO >> 12) & 0x000fffff;;
	pci->potear2 = 0x00000000;
	pci->powbar2 = (MPC85XX_PCI2_IO_BASE >> 12) & 0x000fffff;
	/* Enable, IO R/W */
	pci->powar2 = 0x80088000 | (__ilog2(MPC85XX_PCI2_IO_SIZE) - 1);

	/* Setup 2G inbound Memory Window @ 0 */
	pci->pitar1 = 0x00000000;
	pci->piwbar1 = 0x00000000;
	pci->piwar1 = 0xa0f5501e;	/* Enable, Prefetch, Local
					   Mem, Snoop R/W, 2G */
}
#endif /* CONFIG_85xx_PCI2 */
#endif

#ifdef CONFIG_PEX
static void __init
mpc85xx_setup_pex(struct pci_controller *hose)
{
	struct ccsr_pex *pex;
	unsigned short temps;
	bd_t *binfo = (bd_t *) __res;

	pex = ioremap(binfo->bi_immr_base + MPC85xx_PEX_OFFSET,
		      MPC85xx_PEX_SIZE);

	early_read_config_word(hose, 0, 0, PCI_COMMAND, &temps);
	temps |= PCI_COMMAND_SERR | PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY;
	early_write_config_word(hose, 0, 0, PCI_COMMAND, temps);
	early_write_config_byte(hose, 0, 0, PCI_LATENCY_TIMER, 0x80);

	/* Disable all windows (except powar0 since its ignored) */
	pex->pexowar1 = 0;
	pex->pexowar2 = 0;
	pex->pexowar3 = 0;
	pex->pexowar4 = 0;
	pex->pexiwar1 = 0;
	pex->pexiwar1 = 0;
	pex->pexiwar2 = 0;
	pex->pexiwar3 = 0;

	/* Setup Phys:PEX 1:1 outbound mem window @ MPC85XX_PEX_LOWER_MEM */
	pex->pexotar1 = (MPC85XX_PEX_LOWER_MEM >> 12) & 0x000fffff;
	pex->pexotear1 = 0x00000000;
	pex->pexowbar1 = (MPC85XX_PEX_LOWER_MEM >> 12) & 0x000fffff;
	/* Enable, Mem R/W */
	pex->pexowar1 = 0x80044000 |
			(__ilog2(MPC85XX_PEX_UPPER_MEM - MPC85XX_PEX_LOWER_MEM + 1) - 1);

	/* Setup outboud IO windows @ MPC85XX_PEX_IO_BASE */
	pex->pexotar2 = 0x00000000;
	pex->pexotear2 = 0x00000000;
	pex->pexowbar2 = (MPC85XX_PEX_IO_BASE >> 12) & 0x000fffff;
	/* Enable, IO R/W */
	pex->pexowar2 = 0x80088000 | (__ilog2(MPC85XX_PEX_IO_SIZE) - 1);

	/* Setup 2G inbound Memory Window @ 0 */
	pex->pexitar1 = 0x00000000;
	pex->pexiwbar1 = 0x00000000;
	pex->pexiwar1 = 0xa0f5501e;	/* Enable, Prefetch, Local
	Mem, Snoop R/W, 2G */
}

#else /* ifdef CONFIG_PEX */

#ifdef CONFIG_PPC_INDIRECT_PCI_BE
#define PCI_CFG_OUT out_be32
#else
#define PCI_CFG_OUT out_le32
#endif

static int
mpc85xx_cds_indirect_read_config(struct pci_bus *bus, unsigned int devfn,
				int offset, int len, u32 *val)
{
	struct pci_controller *hose = bus->sysdata;
	volatile unsigned char *cfg_data;
	u8 cfg_type = 0;
	u8 bus_num;

	if (ppc_md.pci_exclude_device)
		if (ppc_md.pci_exclude_device(bus->number, devfn))
			return PCIBIOS_DEVICE_NOT_FOUND;

	if (bus->number == hose->first_busno)
		bus_num = 0;
	else
		bus_num = bus->number;

	PCI_CFG_OUT(hose->cfg_addr,
		 (0x80000000 | (bus_num << 16)
		  | (devfn << 8) | ((offset & 0xfc) | cfg_type)));

	/*
	 * Note: the caller has already checked that offset is
	 * suitably aligned and that len is 1, 2 or 4.
	 */
	cfg_data = hose->cfg_data + (offset & 3);
	switch (len) {
	case 1:
		*val = in_8((u8 *)cfg_data);
		break;
	case 2:
		*val = in_le16((u16 *)cfg_data);
		break;
	default:
		*val = in_le32((u32 *)cfg_data);
		break;
	}
	return PCIBIOS_SUCCESSFUL;
}

static int
mpc85xx_cds_indirect_write_config(struct pci_bus *bus, unsigned int devfn,
				  int offset, int len, u32 val)
{
	struct pci_controller *hose = bus->sysdata;
	volatile unsigned char *cfg_data;
	u8 cfg_type = 0;
	u8 bus_num;

	if (ppc_md.pci_exclude_device)
		if (ppc_md.pci_exclude_device(bus->number, devfn))
			return PCIBIOS_DEVICE_NOT_FOUND;

	if (bus->number == hose->first_busno)
		bus_num = 0;
	else
		bus_num = bus->number;

	PCI_CFG_OUT(hose->cfg_addr,
		 (0x80000000 | (bus_num << 16)
		  | (devfn << 8) | ((offset & 0xfc) | cfg_type)));

	/*
	 * Note: the caller has already checked that offset is
	 * suitably aligned and that len is 1, 2 or 4.
	 */
	cfg_data = hose->cfg_data + (offset & 3);
	switch (len) {
	case 1:
		out_8((u8 *)cfg_data, val);
		break;
	case 2:
		out_le16((u16 *)cfg_data, val);
		break;
	default:
		out_le32((u32 *)cfg_data, val);
		break;
	}
	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops mpc85xx_cds_indirect_pci_ops =
{
	mpc85xx_cds_indirect_read_config,
	mpc85xx_cds_indirect_write_config
};
#endif /* CONFIG_PEX */

void __init
mpc85xx_setup_hose(void)
{
	struct pci_controller *hose_a;
#ifdef CONFIG_PEX
	extern void setup_indirect_pex(struct pci_controller* hose, u32 cfg_addr, u32 cfg_data);

	bd_t *binfo = (bd_t *) __res;

	hose_a = pcibios_alloc_controller();

	if (!hose_a)
		return;

	ppc_md.pci_swizzle = common_swizzle;
	ppc_md.pci_map_irq = mpc85xx_map_irq;

	hose_a->first_busno = 0;
	hose_a->bus_offset = 0;
	hose_a->last_busno = 0xff;

	setup_indirect_pex(hose_a, binfo->bi_immr_base + PEX_CFG_ADDR_OFFSET,
			   binfo->bi_immr_base + PEX_CFG_DATA_OFFSET);

	mpc85xx_setup_pex(hose_a);

	hose_a->pci_mem_offset = MPC85XX_PEX_MEM_OFFSET;
	hose_a->mem_space.start = MPC85XX_PEX_LOWER_MEM;
	hose_a->mem_space.end = MPC85XX_PEX_UPPER_MEM;

	hose_a->io_space.start = MPC85XX_PEX_LOWER_IO;
	hose_a->io_space.end = MPC85XX_PEX_UPPER_IO;
	hose_a->io_base_phys = MPC85XX_PEX_IO_BASE;
	isa_io_base =
			(unsigned long) ioremap(MPC85XX_PEX_IO_BASE,
	MPC85XX_PEX_IO_SIZE);
	hose_a->io_base_virt = (void *) isa_io_base;

	/* setup resources */
	pci_init_resource(&hose_a->mem_resources[0],
			   MPC85XX_PEX_LOWER_MEM,
			   MPC85XX_PEX_UPPER_MEM,
			   IORESOURCE_MEM, "PCI Express host bridge");

	pci_init_resource(&hose_a->io_resource,
			   MPC85XX_PEX_LOWER_IO,
			   MPC85XX_PEX_UPPER_IO,
			   IORESOURCE_IO, "PCI Express host bridge");

	ppc_md.pci_exclude_device = mpc85xx_exclude_device;

	hose_a->last_busno = pciauto_bus_scan(hose_a, hose_a->first_busno);

#else
#ifdef CONFIG_85xx_PCI2
	struct pci_controller *hose_b;
#endif
	bd_t *binfo = (bd_t *) __res;

	hose_a = pcibios_alloc_controller();

	if (!hose_a)
		return;

	ppc_md.pci_swizzle = common_swizzle;
	ppc_md.pci_map_irq = mpc85xx_map_irq;

	hose_a->first_busno = 0;
	hose_a->last_busno = 0xff;

	setup_indirect_pci(hose_a, binfo->bi_immr_base + PCI1_CFG_ADDR_OFFSET,
			   binfo->bi_immr_base + PCI1_CFG_DATA_OFFSET);
	hose_a->ops = &mpc85xx_cds_indirect_pci_ops;

	mpc85xx_setup_pci1(hose_a);

	hose_a->pci_mem_offset = MPC85XX_PCI1_MEM_OFFSET;
	hose_a->mem_space.start = MPC85XX_PCI1_LOWER_MEM;
	hose_a->mem_space.end = MPC85XX_PCI1_UPPER_MEM;

	hose_a->io_space.start = MPC85XX_PCI1_LOWER_IO;
	hose_a->io_space.end = MPC85XX_PCI1_UPPER_IO;
	hose_a->io_base_phys = MPC85XX_PCI1_IO_BASE;
#ifdef CONFIG_85xx_PCI2
	isa_io_base =
		(unsigned long) ioremap(MPC85XX_PCI1_IO_BASE,
					MPC85XX_PCI1_IO_SIZE +
					MPC85XX_PCI2_IO_SIZE);
#else
	isa_io_base =
		(unsigned long) ioremap(MPC85XX_PCI1_IO_BASE,
					MPC85XX_PCI1_IO_SIZE);
#endif
	hose_a->io_base_virt = (void *) isa_io_base;

	/* setup resources */
	pci_init_resource(&hose_a->mem_resources[0],
			MPC85XX_PCI1_LOWER_MEM,
			MPC85XX_PCI1_UPPER_MEM,
			IORESOURCE_MEM, "PCI1 host bridge");

	pci_init_resource(&hose_a->io_resource,
			MPC85XX_PCI1_LOWER_IO,
			MPC85XX_PCI1_UPPER_IO,
			IORESOURCE_IO, "PCI1 host bridge");

	ppc_md.pci_exclude_device = mpc85xx_exclude_device;
	hose_a->last_busno = pciauto_bus_scan(hose_a, hose_a->first_busno);

#ifdef CONFIG_85xx_PCI2
	hose_b = pcibios_alloc_controller();

	if (!hose_b)
		return;

	hose_b->first_busno = hose_a->last_busno + 1;
	hose_b->last_busno = 0xff;

	setup_indirect_pci(hose_b, binfo->bi_immr_base + PCI2_CFG_ADDR_OFFSET,
			   binfo->bi_immr_base + PCI2_CFG_DATA_OFFSET);
	hose_b->ops = &mpc85xx_cds_indirect_pci_ops;

	/* allow access to the host bridge again. */
	ppc_md.pci_exclude_device = NULL;

	mpc85xx_setup_pci2(hose_b);

	hose_b->pci_mem_offset = MPC85XX_PCI2_MEM_OFFSET;
	hose_b->mem_space.start = MPC85XX_PCI2_LOWER_MEM;
	hose_b->mem_space.end = MPC85XX_PCI2_UPPER_MEM;

	hose_b->io_space.start = MPC85XX_PCI2_LOWER_IO;
	hose_b->io_space.end = MPC85XX_PCI2_UPPER_IO;
	hose_b->io_base_phys = MPC85XX_PCI2_IO_BASE;
	hose_b->io_base_virt = (void *) isa_io_base + MPC85XX_PCI1_IO_SIZE;

	/* setup resources */
	pci_init_resource(&hose_b->mem_resources[0],
			MPC85XX_PCI2_LOWER_MEM,
			MPC85XX_PCI2_UPPER_MEM,
			IORESOURCE_MEM, "PCI2 host bridge");

	pci_init_resource(&hose_b->io_resource,
			MPC85XX_PCI2_LOWER_IO,
			MPC85XX_PCI2_UPPER_IO,
			IORESOURCE_IO, "PCI2 host bridge");

	ppc_md.pci_exclude_device = mpc85xx_exclude_device;

	hose_b->last_busno = pciauto_bus_scan(hose_b, hose_b->first_busno);
#endif
#endif
	return;
}
#endif /* CONFIG_PCI */
