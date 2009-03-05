/*
 * arch/ppc/platforms/4xx/sequoia.c
 *
 * Sequoia board specific routines
 *
 * Based on Bamboo.c from Wade Farnsworth <wfarnsworth@mvista.com>
 * Copyright 2004 MontaVista Software Inc.
 * Copyright 2006 AMCC
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/config.h>
#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/reboot.h>
#include <linux/pci.h>
#include <linux/kdev_t.h>
#include <linux/types.h>
#include <linux/major.h>
#include <linux/blkdev.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/initrd.h>
#include <linux/seq_file.h>
#include <linux/root_dev.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/ethtool.h>

#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/machdep.h>
#include <asm/ocp.h>
#include <asm/pci-bridge.h>
#include <asm/time.h>
#include <asm/todc.h>
#include <asm/bootinfo.h>
#include <asm/ppc4xx_pic.h>
#include <asm/ppcboot.h>

#include <syslib/gen550.h>
#include <syslib/ibm440gx_common.h>


bd_t __res;

static struct ibm44x_clocks clocks __initdata;

/*
 * Sequoia external IRQ triggering/polarity settings
 */
unsigned char ppc4xx_uic_ext_irq_cfg[] __initdata = {
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index0 - IRQ4: */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index1 - IRQ7: */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index2 - IRQ8: */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index3 - IRQ9: */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index4 - IRQ0: Ethernet 0 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index5 - IRQ1: Ethernet 1 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index6 - IRQ5: */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index7 - IRQ6: */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index8 - IRQ2: PCI slots */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE), /* Index9 - IRQ3: STTM alert */
};

/*
* get size of system memory from Board Info .
*/
unsigned long __init sequoia_find_end_of_memory(void)
{
	/* board info structure defined in /include/asm-ppc/ppcboot.h */
	return  __res.bi_memsize;
}


static void __init sequoia_calibrate_decr(void)
{
	unsigned int freq;

	if (mfspr(SPRN_CCR1) & CCR1_TCS)
		freq = SEQUOIA_TMRCLK;
	else
		freq = clocks.cpu;

	ibm44x_calibrate_decr(freq);

}


static int sequoia_show_cpuinfo(struct seq_file *m)
{
	seq_printf(m, "vendor\t\t: AMCC\n");
	seq_printf(m, "machine\t\t: PPC440EPx EVB (Sequoia)\n");

	return 0;
}


static inline int
sequoia_map_irq(struct pci_dev *dev, unsigned char idsel, unsigned char pin)
{
	static char pci_irq_table[][4] =
	/*
	 *	PCI IDSEL/INTPIN->INTLINE
	 * 	   A   B   C   D
	 */
	{
		{ 67, 67, 67, 67 },	/* IDSEL [0] bit address 14 - PCI Slot 1 associated to ext. IRQ 2*/
		{ 67, 67, 67, 67 },	/* IDSEL [0] bit address 14 - PCI Slot x associated to ext. IRQ 2*/
		{ 67, 67, 67, 67 },	/* IDSEL [0] bit address 14 - PCI Slot 0 associated to ext. IRQ 2*/
	};

	const long min_idsel = 10, max_idsel = 12, irqs_per_slot = 4;
	return PCI_IRQ_TABLE_LOOKUP;
}


static void __init sequoia_set_emacdata(void)
{
	struct ocp_def *def;
	struct ocp_func_emac_data *emacdata;

	/* Set mac_addr, phy mode and unsupported phy features for each EMAC */

	def = ocp_get_one_device(OCP_VENDOR_IBM, OCP_FUNC_EMAC, 0);
	emacdata = def->additions;

	memcpy(emacdata->mac_addr, __res.bi_enetaddr, 6);

	emacdata->phy_mode = PHY_MODE_RGMII;
	emacdata->phy_feat_exc = SUPPORTED_Autoneg;

	emacdata->rgmii_idx = 0;
	emacdata->rgmii_mux = 0;

	def = ocp_get_one_device(OCP_VENDOR_IBM, OCP_FUNC_EMAC, 1);
	emacdata = def->additions;

	emacdata->rgmii_idx = 0;
	emacdata->rgmii_mux = 1;

	memcpy(emacdata->mac_addr, __res.bi_enet1addr, 6);

	emacdata->phy_mode = PHY_MODE_RGMII;
	emacdata->phy_feat_exc = SUPPORTED_Autoneg;
	emacdata->mdio_idx = 0;
}


static int
sequoia_exclude_device(unsigned char bus, unsigned char devfn)
{
	return (bus == 0 && devfn == 0);
}

#define PCI_READW(offset) \
        (readw((void *)((u32)pci_reg_base+offset)))

#define PCI_WRITEW(value, offset) \
	(writew(value, (void *)((u32)pci_reg_base+offset)))

#define PCI_WRITEL(value, offset) \
	(writel(value, (void *)((u32)pci_reg_base+offset)))

#define PCI_CFG_OUT(offset, value)\
	(out_le32 (pci_cfg_base+offset, value))

#define PCI_CFG_IN(offset)\
	 (in_le32(pci_cfg_base+offset))


static void __init sequoia_setup_pci(void)
{
	void *pci_reg_base;
	void *pci_cfg_base;
	unsigned long memory_size;

	memory_size = ppc_md.find_end_of_memory();

	pci_reg_base = ioremap64(SEQUOIA_PCIL0_BASE, SEQUOIA_PCIL0_SIZE);
	pci_cfg_base = ioremap64(SEQUOIA_PCI_CFGREGS_BASE, 64);

	PCI_CFG_OUT(SEQUOIA_PCI_CFGA_OFFSET,
		    0x80000000 | (PCI_COMMAND & 0xfc));
	PCI_CFG_OUT(SEQUOIA_PCI_CFGD_OFFSET,
		    ( PCI_CFG_IN(SEQUOIA_PCI_CFGD_OFFSET) |
		    PCI_COMMAND_MEMORY |
		    PCI_COMMAND_MASTER));

	/* Disable region first */
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM0MA);

	/* PLB starting addr: 0x0000000180000000 */
	PCI_WRITEL(SEQUOIA_PCI_PHY_MEM_BASE, SEQUOIA_PCIL0_PMM0LA);

	/* PCI start addr, 0x80000000 (PCI Address) */
	PCI_WRITEL(SEQUOIA_PCI_MEM_BASE, SEQUOIA_PCIL0_PMM0PCILA);
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM0PCIHA);

	/* Enable no pre-fetch, enable region */
	PCI_WRITEL(((0xffffffff -
		     (SEQUOIA_PCI_UPPER_MEM - SEQUOIA_PCI_MEM_BASE)) | 0x01),
		      SEQUOIA_PCIL0_PMM0MA);

	/* Disable region one */
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM1MA);
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM1LA);
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM1PCILA);
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM1PCIHA);
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM1MA);

	/* Disable region two */
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM2MA);
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM2LA);
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM2PCILA);
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM2PCIHA);
	PCI_WRITEL(0, SEQUOIA_PCIL0_PMM2MA);

	/* Now configure the PCI->PLB windows, we only use PTM1
	 *
	 * For Inbound flow, set the window size to all available memory
	 * This is required because if size is smaller,
	 * then Eth/PCI DD would fail as PCI card not able to access
	 * the memory allocated by DD.
	 */

	PCI_WRITEL(0, SEQUOIA_PCIL0_PTM1MS);	/* disabled region 1 */
	PCI_WRITEL(0, SEQUOIA_PCIL0_PTM1LA);	/* begin of address map */

	memory_size = 1 << fls(memory_size - 1);

	/* Size low + Enabled */
	PCI_WRITEL((0xffffffff - (memory_size - 1)) | 0x1, SEQUOIA_PCIL0_PTM1MS);

	eieio();
	iounmap(pci_reg_base);
}


static void __init sequoia_setup_hose(void)
{
	unsigned int bar_response, bar;
	struct pci_controller *hose;

	sequoia_setup_pci();

	hose = pcibios_alloc_controller();

	if (!hose)
		return;

	hose->first_busno = 0;
	hose->last_busno = 0xff;

	hose->pci_mem_offset = SEQUOIA_PCI_MEM_OFFSET;

	pci_init_resource(&hose->io_resource,
			SEQUOIA_PCI_LOWER_IO,
			SEQUOIA_PCI_UPPER_IO,
			IORESOURCE_IO,
			"PCI host bridge");

	pci_init_resource(&hose->mem_resources[0],
			SEQUOIA_PCI_LOWER_MEM,
			SEQUOIA_PCI_UPPER_MEM,
			IORESOURCE_MEM,
			"PCI host bridge");

	ppc_md.pci_exclude_device = sequoia_exclude_device;

	hose->io_space.start = SEQUOIA_PCI_LOWER_IO;
	hose->io_space.end = SEQUOIA_PCI_UPPER_IO;
	hose->mem_space.start = SEQUOIA_PCI_LOWER_MEM;
	hose->mem_space.end = SEQUOIA_PCI_UPPER_MEM;
	isa_io_base =
		(unsigned long)ioremap64(SEQUOIA_PCI_IO_BASE, SEQUOIA_PCI_IO_SIZE);
	hose->io_base_virt = (void *)isa_io_base;

	setup_indirect_pci(hose,
			SEQUOIA_PCI_CFGA_PLB32,
			SEQUOIA_PCI_CFGD_PLB32);
	hose->set_cfg_type = 1;

	/* Zero config bars */
	for (bar = PCI_BASE_ADDRESS_1; bar <= PCI_BASE_ADDRESS_2; bar += 4) {
		early_write_config_dword(hose, hose->first_busno,
					 PCI_FUNC(hose->first_busno), bar,
					 0x00000000);
		early_read_config_dword(hose, hose->first_busno,
					PCI_FUNC(hose->first_busno), bar,
					&bar_response);
	}

	hose->last_busno = pciauto_bus_scan(hose, hose->first_busno);

	ppc_md.pci_swizzle = common_swizzle;

	ppc_md.pci_map_irq = sequoia_map_irq;
}

TODC_ALLOC();

static void __init sequoia_early_serial_map(void)
{
	struct uart_port port;

	/* Setup ioremapped serial port access */
	memset(&port, 0, sizeof(port));
	port.mapbase = (unsigned long)PPC440EPX_UART0_ADDR;
	port.membase = ioremap64(PPC440EPX_UART0_ADDR, 8);
	port.irq = UART0_INT;
	port.uartclk = clocks.uart0;
	port.regshift = 0;
	port.iotype = SERIAL_IO_MEM;
	port.flags = ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST;
	port.line = 0;

	if (early_serial_setup(&port) != 0) {
		printk("Early serial init of port 0 failed\n");
	}

#if defined(CONFIG_SERIAL_TEXT_DEBUG)
	/* Configure debug serial access */
	gen550_init(0, &port);
#endif
#ifdef CONFIG_KGDB_8250
	kgdb8250_add_port(0, &port);
#endif

	port.mapbase = (unsigned long)PPC440EPX_UART1_ADDR;
	port.membase = ioremap64(PPC440EPX_UART1_ADDR, 8);
	port.irq = UART1_INT;
	port.uartclk = clocks.uart1;
	port.line = 1;

	if (early_serial_setup(&port) != 0) {
		printk("Early serial init of port 1 failed\n");
	}

#if defined(CONFIG_SERIAL_TEXT_DEBUG)
	/* Configure debug serial access */
	gen550_init(1, &port);
#endif
#ifdef CONFIG_KGDB_8250
	kgdb8250_add_port(1, &port);
#endif

	port.mapbase = (unsigned long)PPC440EPX_UART2_ADDR;
	port.membase = ioremap64(PPC440EPX_UART2_ADDR, 8);
	port.irq = UART2_INT;
	port.uartclk = clocks.uart2;
	port.line = 2;

	if (early_serial_setup(&port) != 0) {
		printk("Early serial init of port 2 failed\n");
	}

#if defined(CONFIG_SERIAL_TEXT_DEBUG)
	/* Configure debug serial access */
	gen550_init(2, &port);
#endif
#ifdef CONFIG_KGDB_8250
	kgdb8250_add_port(2, &port);
#endif

	port.mapbase = (unsigned long)PPC440EPX_UART3_ADDR;
	port.membase = ioremap64(PPC440EPX_UART3_ADDR, 8);
	port.irq = UART3_INT;
	port.uartclk = clocks.uart3;
	port.line = 3;

	if (early_serial_setup(&port) != 0) {
		printk("Early serial init of port 3 failed\n");
	}

#if defined(CONFIG_SERIAL_TEXT_DEBUG)
	/* Configure debug serial access */
	gen550_init(3, &port);
#endif
#ifdef CONFIG_KGDB_8250
	kgdb8250_add_port(3, &port);
#endif
}


static void __init sequoia_setup_arch(void)
{
	sequoia_set_emacdata();

	/* parm1 = sys clock is OK , parm 2 ser_clock to be checked */
	ibm440gx_get_clocks(&clocks, 33333333, 6 * 1843200);
	ocp_sys_info.opb_bus_freq = clocks.opb;

	/* init to some ~sane value until calibrate_delay() runs */
        loops_per_jiffy = 50000000/HZ;

	/* Setup PCI host bridge */
	sequoia_setup_hose();

#ifdef CONFIG_BLK_DEV_INITRD
	if (initrd_start)
		ROOT_DEV = Root_RAM0;
	else {
#ifdef CONFIG_ROOT_NFS
		ROOT_DEV = Root_NFS;
#else
		ROOT_DEV = Root_HDA1;
#endif
	}
#endif

	sequoia_early_serial_map();

	printk( "AMCC PowerPC Sequoia platform\n" );
}


static void __init sequoia_init_irq(void)
{
	ppc4xx_pic_init();
}


void __init platform_init(unsigned long r3, unsigned long r4,
			  unsigned long r5, unsigned long r6, unsigned long r7)
{
	parse_bootinfo(find_bootinfo());

	/*
	 * If we were passed in a board information, copy it into the
	 * residual data area.
	 */
	if (r3)
		__res = *(bd_t *)(r3 + KERNELBASE);

#if defined(CONFIG_BLK_DEV_INITRD)
	/*
	 * If the init RAM disk has been configured in, and there's a valid
	 * starting address for it, set it up.
	 */
	if (r4) {
		initrd_start = r4 + KERNELBASE;
		initrd_end = r5 + KERNELBASE;
	}
#endif  /* CONFIG_BLK_DEV_INITRD */

	/* Copy the kernel command line arguments to a safe place. */

	if (r6) {
		*(char *) (r7 + KERNELBASE) = 0;
		strcpy(cmd_line, (char *) (r6 + KERNELBASE));
	}
	ibm44x_platform_init();

	ppc_md.setup_arch = sequoia_setup_arch;
	ppc_md.show_cpuinfo = sequoia_show_cpuinfo;
	ppc_md.find_end_of_memory = sequoia_find_end_of_memory;
	ppc_md.get_irq = NULL;		/* Set in ppc4xx_pic_init() */

	ppc_md.calibrate_decr = sequoia_calibrate_decr;
	ppc_md.time_init = NULL;
	ppc_md.set_rtc_time = NULL;
	ppc_md.get_rtc_time = NULL;

	ppc_md.init_IRQ = sequoia_init_irq;
}
