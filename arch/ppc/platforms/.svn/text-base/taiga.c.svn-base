/*
 * taiga.c
 *
 * Board setup routines for the Freescale Taiga platform
 *
 * Author: Jacob Pan
 *	 jacob.pan@freescale.com
 * Author: Xianghua Xiao
 *       updated for taiga board from emulation platform 2005.5
 *       x.xiao@freescale.com
 * Maintainer: Roy Zang <tie-fei.zang@freescale.com>
 *
 * Copyright 2004-2006 Freescale Semiconductor, Inc.
 *
 * This file is licensed under
 * the terms of the GNU General Public License version 2.  This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
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
#include <linux/initrd.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/ide.h>
#include <linux/seq_file.h>
#include <linux/root_dev.h>
#include <linux/serial.h>
#include <linux/tty.h>		/* for linux/serial_core.h */
#include <linux/serial_core.h>
#include <linux/kgdb.h>

#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/time.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/machdep.h>
#include <asm/prom.h>
#include <asm/smp.h>
#include <asm/vga.h>
#include <asm/todc.h>
#include <asm/bootinfo.h>
#include <asm/tsi108.h>
#include <syslib/tsi108_pic.h>
#include <asm/pci-bridge.h>
#include <asm/kgdb.h>
#include <asm/reg.h>

#include "taiga.h"

#ifdef TSI108_ETH
#include "../../../drivers/net/tsi108_eth.h"
#endif

extern void gen550_init(int, struct uart_port *);

extern int tsi108_direct_write_config(struct pci_bus *bus, unsigned int devfn,
				      int offset, int len, u32 val);
extern int tsi108_direct_read_config(struct pci_bus *bus, unsigned int devfn,
				     int offset, int len, u32 * val);

unsigned char __res[sizeof(bd_t)];
#ifdef GEN550_CONSOLE
extern void gen550_progress(char *, unsigned short);
#else
static void tsi108_uart_progress(char *s, unsigned short hex);
#endif

static void taiga_halt(void);

#ifdef TSI108_ETH
hw_info hw_info_table[TSI108_ETH_MAX_PORTS + 1] = {
	{TSI108_CSR_ADDR_VIRT + TSI108_ETH_OFFSET,
	 TSI108_CSR_ADDR_VIRT + TSI108_ETH_OFFSET,
	 TSI108_PHY0_ADDR, IRQ_TSI108_GIGE0},
	{TSI108_CSR_ADDR_VIRT + TSI108_ETH_OFFSET + 0x400,
	 TSI108_CSR_ADDR_VIRT + TSI108_ETH_OFFSET,
	 TSI108_PHY1_ADDR, IRQ_TSI108_GIGE1},
	{TBL_END, TBL_END, TBL_END, TBL_END}
};
#endif

/*
 * Define all of the IRQ senses and polarities.  Taken from the
 * Taiga  manual.
 */
static u_char taiga_pic_initsenses[] __initdata = {
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* INT[0] XINT0 from FPGA */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* INT[1] XINT1 from FPGA */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* INT[2] PHY_INT from both GIGE */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* INT[3] RESERVED */
};

static struct pci_ops direct_pci_ops = {
	tsi108_direct_read_config,
	tsi108_direct_write_config
};

/*
 * Taiga PCI interrupt routing. all PCI interrupt comes from
 * external PCI source at 23. need to program pci interrupt control registers
 * to route per slot IRQs.
 */

static inline int
taiga_map_irq(struct pci_dev *dev, unsigned char idsel, unsigned char pin)
{
	static char pci_irq_table[][4] =
	    /*
	     *      PCI IDSEL/INTPIN->INTLINE
	     *         A     B     C     D
	     */
	{
		{IRQ_PCI_INTA, IRQ_PCI_INTB, IRQ_PCI_INTC, IRQ_PCI_INTD},	/* A SLOT 1 IDSEL 17 */
		{IRQ_PCI_INTB, IRQ_PCI_INTC, IRQ_PCI_INTD, IRQ_PCI_INTA},	/* B SLOT 2 IDSEL 18 */
		{IRQ_PCI_INTC, IRQ_PCI_INTD, IRQ_PCI_INTA, IRQ_PCI_INTB},	/* C SATA IDSEL 19 */
		{IRQ_PCI_INTD, IRQ_PCI_INTA, IRQ_PCI_INTB, IRQ_PCI_INTC},	/* D USB IDSEL 20 */
	};

	const long min_idsel = 1, max_idsel = 4, irqs_per_slot = 4;
	return PCI_IRQ_TABLE_LOOKUP;
}

static void __init taiga_setup_bridge(void)
{
	struct pci_controller *hose;

	hose = pcibios_alloc_controller();

	if (hose){
		hose->first_busno = 0;
		hose->last_busno = 0xff;
		hose->pci_mem_offset = TAIGA_PCI_MEM_OFFSET;

		/* Setup resources to match map
		 * PCI memory and IO space are set in PFAB_BARs by boot code
		 * 64KB take one 16MB LUT entry, PFAB_IO
		 */
		pci_init_resource(&hose->io_resource, TAIGA_PCI_IO_START,
				  TAIGA_PCI_IO_END, IORESOURCE_IO,
				  "PCI host bridge");

		/* OCN to PCI/X transaction is unchanged,
		 * bar1 first 8 LUTs, 128MB
		 */
		pci_init_resource(&hose->mem_resources[0],
				  TAIGA_PCI_MEM_START,
				  TAIGA_PCI_MEM_END,
				  IORESOURCE_MEM, "PCI host bridge");

		(hose)->io_space.start = TAIGA_PCI_IO_START;
		(hose)->io_space.end = TAIGA_PCI_IO_END;
		(hose)->mem_space.start = TAIGA_PCI_MEM_START;
		(hose)->mem_space.end = TAIGA_PCI_MEM_END;
		(hose)->io_base_virt = (void *)TAIGA_ISA_IO_BASE;
		(hose)->ops = &direct_pci_ops;

		hose->last_busno = pciauto_bus_scan(hose, hose->first_busno);

		ppc_md.pcibios_fixup = NULL;
		ppc_md.pcibios_fixup_bus = NULL;
		ppc_md.pci_swizzle = common_swizzle;
		ppc_md.pci_map_irq = taiga_map_irq;

		if (ppc_md.progress)
			ppc_md.progress("tsi108: resources set", 0x100);

		tsi108_bridge_init(hose, TSI108_CSR_ADDR_PHYS);

	} else {
		printk("PCI Host bridge init failed\n");
	}
}

#ifdef	CONFIG_SERIAL_8250
static void __init taiga_early_serial_map(void)
{
	struct uart_port serial_req;

	/* Setup serial port access */
	memset(&serial_req, 0, sizeof(serial_req));
	serial_req.uartclk = UART_CLK;
	serial_req.irq = IRQ_TSI108_UART0;
	serial_req.flags = STD_COM_FLAGS;
	serial_req.iotype = SERIAL_IO_MEM;
	/* CONFIG_TAIGA: remapped */
	serial_req.membase = (u_char *) (TAIGA_SERIAL_0 | TSI108_CSR_ADDR_VIRT);

#ifdef CONFIG_SERIAL_TEXT_DEBUG
	gen550_init(0, &serial_req);
#endif
#ifdef CONFIG_KGDB_8250
	kgdb8250_add_port(0, &serial_req);
#endif

	if (early_serial_setup(&serial_req) != 0)
		printk(KERN_ERR "Early serial init of port 0 failed\n");

	/* Assume early_serial_setup() doesn't modify serial_req */
	serial_req.line = 1;
	serial_req.irq = IRQ_TSI108_UART1;
	/* CONFIG_TAIGA: remapped */
	serial_req.membase = (u_char *) (TAIGA_SERIAL_1 | TSI108_CSR_ADDR_VIRT);

#ifdef CONFIG_SERIAL_TEXT_DEBUG
	gen550_init(1, &serial_req);
#endif
#ifdef CONFIG_KGDB_8250
	kgdb8250_add_port(1, &serial_req);
#endif

	if (early_serial_setup(&serial_req) != 0)
		printk(KERN_ERR "Early serial init of port 1 failed\n");
}
#endif

static __inline__ void taiga_l2cr_prefetch_enable(void)
{
	unsigned long msscr0;
	__asm__ __volatile__("mfspr %0, 0x3f6\n \
		ori %0,%0,0x3\n	         \
		sync \n                  \
		mtspr 0x3f6,%0\n   \
		sync\n			 \
		isync ":"=r"(msscr0));
}

TODC_ALLOC();
static void __init taiga_setup_arch(void)
{
	loops_per_jiffy = 50000000 / HZ;

#ifdef CONFIG_BLK_DEV_INITRD
	if (initrd_start)
		ROOT_DEV = Root_RAM0;
	else
#endif
#ifdef	CONFIG_ROOT_NFS
		ROOT_DEV = Root_NFS;
#else
		ROOT_DEV = Root_HDA1;
#endif

	if (ppc_md.progress)
		ppc_md.progress("taiga_setup_arch: setup_bridge", 0);

	/* setup PCI host bridge */
	taiga_setup_bridge();

#ifdef CONFIG_DUMMY_CONSOLE
	conswitchp = &dummy_con;
#endif

	printk(KERN_INFO "Taiga Platform\n");
	printk(KERN_INFO
	       "Jointly ported by Freescale and Tundra Semiconductor\n");
	printk(KERN_INFO
	       "Enabling L2 cache then enabling the HID0 prefetch engine.\n");
	_set_L2CR(L2CR_L2E);
	taiga_l2cr_prefetch_enable();
	TODC_INIT(TODC_TYPE_MK48T35, 0, 0, TAIGA_NVRAM_BASE_ADDR, 8);
}

/*
 * Interrupt setup and service.  Interrrupts on the taiga come
 * from the four external INT pins, PCI interrupts are routed via
 * PCI interrupt control registers, it generates internal IRQ23
 *
 * Interrupt routing on the Taiga Board:
 * TSI108:PB_INT[0] -> CPU0:INT#
 * TSI108:PB_INT[1] -> CPU0:MCP#
 * TSI108:PB_INT[2] -> N/C
 * TSI108:PB_INT[3] -> N/C
 */
static void __init taiga_init_IRQ(void)
{

	tsi108_pic_init(taiga_pic_initsenses);

	/* Configure MPIC outputs to CPU0 */
	tsi108_pic_set_output(0, IRQ_SENSE_EDGE, IRQ_POLARITY_NEGATIVE);
}

static unsigned long __init taiga_find_end_of_memory(void)
{
	bd_t *bp = (bd_t *) __res;

	if (bp->bi_memsize)
		return bp->bi_memsize;
	/* read memory controller to determine memory size */
	return tsi108_get_mem_size();
}

static void __init taiga_map_io(void)
{
	/* PCI IO  mapping */
	io_block_mapping(TAIGA_PCI_IO_BASE_VIRT, TAIGA_PCI_IO_BASE_PHYS,
			 0x00800000, _PAGE_IO);
	/* Tsi108 CSR mapping */
	io_block_mapping(TSI108_CSR_ADDR_VIRT, TSI108_CSR_ADDR_PHYS,
			 0x100000, _PAGE_IO);
	tsi108_csr_base = TSI108_CSR_ADDR_VIRT;
	/* PCI Config mapping */
	io_block_mapping(TAIGA_PCI_CFG_BASE_VIRT, TAIGA_PCI_CFG_BASE_PHYS,
			 TAIGA_PCI_CFG_SIZE, _PAGE_IO);
	tsi108_pci_cfg_base = TAIGA_PCI_CFG_BASE_VIRT;
	/* NVRAM mapping */
	io_block_mapping(TAIGA_NVRAM_BASE_ADDR, TAIGA_NVRAM_BASE_ADDR,
			 TAIGA_NVRAM_SIZE, _PAGE_IO);
}

static void taiga_restart(char *cmd)
{
	local_irq_disable();

	/* Set exception prefix high - to the firmware */
	_nmask_and_or_msr(0, MSR_IP);

	for (;;) ;		/* Spin until reset happens */
}

static void taiga_power_off(void)
{
	local_irq_disable();
	for (;;) ;		/* No way to shut power off with software */
}

static void taiga_halt(void)
{
	taiga_power_off();
}

static int taiga_show_cpuinfo(struct seq_file *m)
{
	seq_printf(m, "vendor\t\t: Freescale Semiconductor\n");
	seq_printf(m, "machine\t\t: Taiga\n");
	seq_printf(m, "PB freq\t\t: %ldMhz\n", tsi108_get_cpu_clk() / 1000000);
	seq_printf(m, "SDC freq\t: %ldMhz\n", tsi108_get_sdc_clk() / 1000000);

	return 0;
}

void tsi108_clear_pci_cfg_error(void)
{
	tsi108_clear_pci_error(TAIGA_PCI_CFG_BASE_PHYS);
}

void __init
platform_init(unsigned long r3, unsigned long r4, unsigned long r5,
	      unsigned long r6, unsigned long r7)
{
	parse_bootinfo(find_bootinfo());

	/* ASSUMPTION:  If both r3 (bd_t pointer) and r6 (cmdline pointer)
	 * are non-zero, then we should use the board info from the bd_t
	 * structure and the cmdline pointed to by r6 instead of the
	 * information from birecs, if any.  Otherwise, use the information
	 * from birecs as discovered by the preceeding call to
	 * parse_bootinfo().  This rule should work with both PPCBoot, which
	 * uses a bd_t board info structure, and the kernel boot wrapper,
	 * which uses birecs.
	 */
	if (r3 && r6) {
		/* copy board info structure */
		memcpy((void *)__res, (void *)(r3 + KERNELBASE), sizeof(bd_t));
		/* copy command line */
		*(char *)(r7 + KERNELBASE) = 0;
		strcpy(cmd_line, (char *)(r6 + KERNELBASE));
	}
#ifdef CONFIG_BLK_DEV_INITRD
	/* take care of initrd if we have one */
	if (r4) {
		initrd_start = r4 + KERNELBASE;
		initrd_end = r5 + KERNELBASE;
	}
#endif				/* CONFIG_BLK_DEV_INITRD */

	/* Allow access to Tsi108 CSR registers before IO space is mapped */
	tsi108_csr_base = TSI108_CSR_ADDR_VIRT;

	/* set MSSCR0[EIDIS] bit here. */
	mtspr(SPRN_MSSCR0, mfspr(SPRN_MSSCR0) | 0x01000000);
	mb();
	isa_io_base = TAIGA_ISA_IO_BASE;
	isa_mem_base = TAIGA_ISA_MEM_BASE;
	pci_dram_offset = TAIGA_PCI_MEM_OFFSET;

	DMA_MODE_READ = 0x44;
	DMA_MODE_WRITE = 0x48;

	ppc_md.setup_arch = taiga_setup_arch;
	ppc_md.show_cpuinfo = taiga_show_cpuinfo;
	/* taiga_irq_canonicalize */
	ppc_md.irq_canonicalize = NULL;
	ppc_md.init_IRQ = taiga_init_IRQ;
	ppc_md.get_irq = tsi108_pic_get_irq;

	ppc_md.restart = taiga_restart;
	ppc_md.power_off = taiga_power_off;
	ppc_md.halt = taiga_halt;

	ppc_md.calibrate_decr = todc_calibrate_decr;
	ppc_md.time_init = todc_time_init;
	ppc_md.set_rtc_time = todc_set_rtc_time;
	ppc_md.get_rtc_time = todc_get_rtc_time;
	ppc_md.nvram_read_val = todc_direct_read_val;
	ppc_md.nvram_write_val = todc_direct_write_val;
	ppc_md.find_end_of_memory = taiga_find_end_of_memory;
	ppc_md.setup_io_mappings = taiga_map_io;

#ifdef	CONFIG_SERIAL_8250
	taiga_early_serial_map();
#ifdef CONFIG_SERIAL_TEXT_DEBUG
#ifdef GEN550_CONSOLE
	ppc_md.progress = gen550_progress;
#  else
	ppc_md.progress = tsi108_uart_progress;
#endif
#endif
#endif

#if defined(CONFIG_BLK_DEV_IDE) || defined(CONFIG_BLK_DEV_IDE_MODULE)
	ppc_ide_md.default_irq = NULL;
	ppc_ide_md.default_io_base = NULL;
	ppc_ide_md.ide_init_hwif = NULL;
#endif

}

#ifndef GEN550_CONSOLE
#if defined(CONFIG_SERIAL_TEXT_DEBUG)
#include <linux/serialP.h>
#include <linux/serial_reg.h>
#include <asm/serial.h>

static struct serial_state rs_table[RS_TABLE_SIZE] = {
	SERIAL_PORT_DFNS	/* Defined in <asm/serial.h> */
};

static void tsi108_uart_progress(char *s, unsigned short hex)
{
	volatile char c;
	volatile unsigned long com_port;
	u16 shift;
	com_port = rs_table[0].port;
	shift = rs_table[0].iomem_reg_shift;
	while ((c = *s++) != 0) {
		while ((*((volatile unsigned char *)com_port +
			  (UART_LSR << shift)) & UART_LSR_THRE) == 0) ;
		*(volatile unsigned char *)com_port = c;

		if (c == '\n') {
			while ((*((volatile unsigned char *)com_port +
				  (UART_LSR << shift)) & UART_LSR_THRE) == 0) ;
			*(volatile unsigned char *)com_port = '\r';
		}
	}

	/* Move to next line on */
	while ((*((volatile unsigned char *)com_port +
		  (UART_LSR << shift)) & UART_LSR_THRE) == 0) ;

	*(volatile unsigned char *)com_port = '\n';
	while ((*((volatile unsigned char *)com_port +
		  (UART_LSR << shift)) & UART_LSR_THRE) == 0) ;
	*(volatile unsigned char *)com_port = '\r';
	return;
}
#endif				/* CONFIG_SERIAL_TEXT_DEBUG */
#endif				/* GEN550_CONSOLE */
