/*
 * arch/ppc/syslib/ppc86xx_setup.c
 *
 * MPC86XX common board code
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
#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>

#include <asm/prom.h>
#include <asm/time.h>
#include <asm/mpc86xx.h>
#include <asm/immap_86xx.h>
#include <asm/mmu.h>
#include <asm/ppc_sys.h>
#include <asm/kgdb.h>

#include <syslib/ppc86xx_setup.h>

#undef DEBUG

#ifdef DEBUG
#define DBG(fmt, args...) printk(KERN_ERR "%s: " fmt, __FUNCTION__, ## args)
#else
#define DBG(fmt, args...)
#endif

extern void kgdb8250_add_port(int i, struct uart_port *serial_req);

#define CONFIG_86xx_PCI2	1

/* Return the amount of memory */
unsigned long __init
mpc86xx_find_end_of_memory(void)
{
	bd_t *binfo;

	binfo = (bd_t *) __res;

	return binfo->bi_memsize;
}

/* The decrementer counts at the system (internal) clock freq divided by 8 */
void __init
mpc86xx_calibrate_decr(void)
{
	bd_t *binfo = (bd_t *) __res;
	unsigned int freq, divisor, temp;

	/* get the core frequency */
	freq = binfo->bi_busfreq;

	/* The timebase is updated every 4 bus clocks */
	divisor = 4;
	tb_ticks_per_jiffy = freq / divisor / HZ;
	tb_to_us = mulhwu_scale_factor(freq / divisor, 1000000);

	/* Set the time base to zero */
	mtspr(SPRN_TBWL, 0);
	mtspr(SPRN_TBWU, 0);

	temp = mfspr(SPRN_HID0);
	temp |= HID0_TBEN;
	mtspr(SPRN_HID0, temp);
}

#ifdef CONFIG_SERIAL_8250
void __init
mpc86xx_early_serial_map(void)
{
#if defined(CONFIG_SERIAL_TEXT_DEBUG) || defined(CONFIG_KGDB)
	struct uart_port serial_req;
#endif
	struct plat_serial8250_port *pdata;
	bd_t *binfo = (bd_t *) __res;
	pdata = (struct plat_serial8250_port *) ppc_sys_get_pdata(MPC86xx_DUART);

	/* Setup serial port access */
	pdata[0].uartclk = binfo->bi_busfreq;
	pdata[0].mapbase += binfo->bi_immr_base;
	pdata[0].membase = ioremap(pdata[0].mapbase, MPC86xx_UART0_SIZE);

#if defined(CONFIG_SERIAL_TEXT_DEBUG) || defined(CONFIG_KGDB)
	memset(&serial_req, 0, sizeof (serial_req));
	serial_req.iotype = UPIO_MEM; /*SERIAL_IO_MEM;*/
	serial_req.mapbase = pdata[0].mapbase;
	serial_req.membase = pdata[0].membase;
	serial_req.regshift = 0;
	serial_req.uartclk = pdata[0].uartclk;
	serial_req.irq = pdata[0].irq;
	serial_req.line = pdata[0].line;
	serial_req.flags = pdata[0].flags;

	if (early_serial_setup(&serial_req) != 0)
		printk(KERN_INFO "Early serial init of port 0 failed\n");
#endif

#if defined(CONFIG_SERIAL_TEXT_DEBUG)
	gen550_init(0, &serial_req);
#endif
#ifdef CONFIG_KGDB_8250
	kgdb8250_add_port(0, &serial_req);
#endif

	pdata[1].mapbase += binfo->bi_immr_base;
	pdata[1].membase = ioremap(pdata[1].mapbase, MPC86xx_UART1_SIZE);

#if defined(CONFIG_SERIAL_TEXT_DEBUG) || defined(CONFIG_KGDB)
	/*
	 * Assume gen550_init() doesn't modify serial_req
	 * and that port 0 and 1 have the same iotype, flags and uartclk
	 */

	serial_req.mapbase = pdata[1].mapbase;
	serial_req.membase = pdata[1].membase;
	serial_req.irq = pdata[0].irq;
	serial_req.line = pdata[1].line;

	if (early_serial_setup(&serial_req) != 0)
		printk(KERN_INFO "Early serial init of port 1 failed\n");
#endif

#if defined(CONFIG_SERIAL_TEXT_DEBUG)
	gen550_init(1, &serial_req);
#endif
#ifdef CONFIG_KGDB_8250
	kgdb8250_add_port(1, &serial_req);
#endif
}
#endif

void
mpc86xx_restart(char *cmd)
{
	local_irq_disable();
	for(;;);
}

void
mpc86xx_power_off(void)
{
	local_irq_disable();
	for(;;);
}

void
mpc86xx_halt(void)
{
	local_irq_disable();
	for(;;);
}

extern int mpc86xx_map_irq(struct pci_dev *dev, unsigned char idsel, unsigned char pin);
extern int mpc86xx_exclude_device(u_char bus, u_char devfn);

#ifdef CONFIG_PCIE

static void __init
mpc86xx_setup_pcie(struct pci_controller *hose, u32 pcie_offset, u32 lower_mem,
		   u32 upper_mem, u32 io_base)
{
	volatile struct ccsr_pcie *pcie;
	u16 cmd;
	bd_t *binfo = (bd_t *) __res;

	pcie = ioremap(binfo->bi_immr_base + pcie_offset,
		MPC86xx_PCIE_SIZE);

	early_read_config_word(hose, 0, 0, PCI_COMMAND, &cmd);
	cmd |=  PCI_COMMAND_SERR   | PCI_COMMAND_MASTER |
		PCI_COMMAND_MEMORY | PCI_COMMAND_IO;
	early_write_config_word(hose, 0, 0, PCI_COMMAND, cmd);
	early_write_config_byte(hose, 0, 0, PCI_LATENCY_TIMER, 0x80);

	/* Disable all windows (except pcieowar0 since its ignored) */
	pcie->pcieowar1 = 0;
	pcie->pcieowar2 = 0;
	pcie->pcieowar3 = 0;
	pcie->pcieowar4 = 0;
	pcie->pcieiwar1 = 0;
	pcie->pcieiwar1 = 0;
	pcie->pcieiwar2 = 0;
	pcie->pcieiwar3 = 0;

	/* Setup Phys:PCIE 1:1 outbound mem window @ MPC86XX_PCIEn_LOWER_MEM */
	pcie->pcieotar1 = (lower_mem >> 12) & 0x000fffff;
	pcie->pcieotear1 = 0x00000000;
	pcie->pcieowbar1 = (lower_mem >> 12) & 0x000fffff;
	/* Enable, Mem R/W */
	pcie->pcieowar1 = 0x80044000 |
		(__ilog2(upper_mem - lower_mem + 1) - 1);

	/* Setup outbound IO windows @ MPC86XX_PCIEn_IO_BASE */
	pcie->pcieotar2 = (MPC86XX_PCIE_LOWER_IO >> 12) & 0x000fffff;
	pcie->pcieotear2 = 0x00000000;
	pcie->pcieowbar2 = (io_base >> 12) & 0x000fffff;
	/* Enable, IO R/W */
	pcie->pcieowar2 = 0x80088000 | (__ilog2(MPC86XX_PCIE_IO_SIZE) - 1);

	/* Setup 2G inbound Memory Window @ 0 */
	pcie->pcieitar1 = 0x00000000;
	pcie->pcieiwbar1 = 0x00000000;
	/* Enable, Prefetch, Local Mem, Snoop R/W, 2G */
	pcie->pcieiwar1 = 0xa0f5501e;
}

void __init
mpc86xx_setup_one_hose(struct pci_controller * hose, u32 cfg_addr_offset, u32 cfg_data_offset,
		       u32 lower_mem, u32 upper_mem, u32 io_base)
{
	extern void setup_indirect_pcie(struct pci_controller* hose, u32 cfg_addr, u32 cfg_data);

	bd_t *binfo = (bd_t *) __res;
	setup_indirect_pcie(hose, binfo->bi_immr_base + cfg_addr_offset,
			    binfo->bi_immr_base + cfg_data_offset);
	hose->set_cfg_type = 1;

	mpc86xx_setup_pcie(hose, cfg_addr_offset, lower_mem, upper_mem, io_base);

	hose->pci_mem_offset = MPC86XX_PCIE_MEM_OFFSET;
	hose->mem_space.start = lower_mem;
	hose->mem_space.end = upper_mem;

	hose->io_space.start = MPC86XX_PCIE_LOWER_IO;
	hose->io_space.end = MPC86XX_PCIE_UPPER_IO;
	hose->io_base_phys = io_base;
	hose->io_base_virt =  ioremap(io_base,
					MPC86XX_PCIE_IO_SIZE);

	/* setup resources */
	pci_init_resource(&hose->mem_resources[0], lower_mem, upper_mem,
			  IORESOURCE_MEM, "PCI Express host bridge");

	pci_init_resource(&hose->io_resource, MPC86XX_PCIE_LOWER_IO,
			  MPC86XX_PCIE_UPPER_IO, IORESOURCE_IO,
			  "PCI Express host bridge");
}

static void __devinit early_host_bridge(struct pci_dev *dev)
{
	/* Fix the device class to match header type 1. */
	dev->class = 0x060401;
}
DECLARE_PCI_FIXUP_EARLY(0x1957, 0x7011, early_host_bridge);
#endif /* CONFIG_PCIE */

void __init
mpc86xx_setup_hose(void)
{
#ifdef CONFIG_PCIE
	struct pci_controller *hose_a, *hose_b;
	extern void mpc86xx_postscan_fixups(struct pci_controller *hose);

	DBG("Adding PCIE host bridge\n");

	hose_a = pcibios_alloc_controller();
	if (!hose_a)
		return;

	hose_a->bus_offset = 0;
	hose_a->first_busno = 0x0;
	hose_a->index = 0;

	mpc86xx_setup_one_hose(hose_a, PCIE1_CFG_ADDR_OFFSET,
			       PCIE1_CFG_DATA_OFFSET, MPC86XX_PCIE1_LOWER_MEM,
			       MPC86XX_PCIE1_UPPER_MEM, MPC86XX_PCIE1_IO_BASE);

	isa_io_base = (unsigned long) hose_a->io_base_virt;
	if ((mfspr(SPRN_SVR) & 0xf0) == 0x10) {
		/*
 		 * PCIE Bus, Fix the MPC8641D host bridge's location to bus 0xFF.
		 * Silicon Rev 1.0 only.
		 */
		early_write_config_byte(hose_a, 0, 0x0, PCI_PRIMARY_BUS, 0xff);
		early_write_config_byte(hose_a, 0xff, 0x0, PCI_SECONDARY_BUS,
					0x0);
		early_write_config_byte(hose_a, 0xff, 0x0, PCI_SUBORDINATE_BUS,
					0xfe);

		hose_a->last_busno = 0xfe;
		hose_a->scan_dev_0 = 0;
	}
	else {
		hose_a->last_busno = 0xff;
		hose_a->scan_dev_0 = 1;
	}

	hose_a->last_busno = pciauto_bus_scan(hose_a, hose_a->first_busno);

	mpc86xx_postscan_fixups(hose_a);

	/* Second host bridge broken on Rev 1.0 8641D */
	if ((mfspr(SPRN_SVR) & 0xf0) != 0x10) {
		hose_b = pcibios_alloc_controller();
		if (!hose_b)
			return;
		hose_b->first_busno = hose_a->last_busno + 1;
		hose_b->last_busno = 0xff;
		hose_b->index = 1;
		hose_b->scan_dev_0 = 1;

		mpc86xx_setup_one_hose(hose_b, PCIE2_CFG_ADDR_OFFSET,
				       PCIE2_CFG_DATA_OFFSET,
				       MPC86XX_PCIE2_LOWER_MEM,
				       MPC86XX_PCIE2_UPPER_MEM,
				       MPC86XX_PCIE2_IO_BASE);

		hose_b->last_busno = pciauto_bus_scan(hose_b,
						      hose_b->first_busno);
	}

	ppc_md.pci_swizzle = common_swizzle;
	ppc_md.pci_map_irq = mpc86xx_map_irq;
	ppc_md.pci_exclude_device = mpc86xx_exclude_device;

#endif /* CONFIG_PCIE */
}
