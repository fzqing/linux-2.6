/*
 * arch/ppc/platforms/prpmc275.c
 *
 * Board setup routines for the Force PPMC275 Development Board.
 *
 * Athor: Vladimir A. Barinov <vbarinov@ru.mvista.com>
 * 
 * Based on code done by Rabeeh Khoury - rabeeh@galileo.co.il
 * Based on code done by - Mark A. Greer <mgreer@mvista.com>
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
#include <linux/blkdev.h>
#include <linux/console.h>
#include <linux/root_dev.h>
#include <linux/initrd.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/ide.h>
#include <linux/seq_file.h>
#include <linux/mtd/physmap.h>
#include <linux/mv643xx.h>

#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/hardirq.h>
#include <asm/time.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/machdep.h>
#include <asm/prom.h>
#include <asm/bootinfo.h>
#include <asm/mv64x60.h>

#include "prpmc275.h"

static mv64x60_handle_t bh;

static void __iomem *sram_base;	/* Virtual addr of Internal SRAM */

static u32 prpmc275_flash_size_0;
static u32 prpmc275_flash_size_1;

#ifdef CONFIG_NONMONARCH_SUPPORT
static int monarch = 0;

static u8
read_monarch_status(void)
{
	u32 temp;

	/* Make sure that MV64360_MPP[4] is made MV64360_GPP[4] */

	temp = mv64x60_read(&bh, MV64x60_MPP_CNTL_0);
	temp = temp & 0xfff0ffff;
	mv64x60_write(&bh, MV64x60_MPP_CNTL_0, temp);

	/* Make sure that MV64360_GPP[4] is an Input pin */

	temp = mv64x60_read(&bh, MV64x60_GPP_IO_CNTL);
	temp = temp & 0xffffffef;
	mv64x60_write(&bh, MV64x60_GPP_IO_CNTL, temp);

	/* Make sure MV64360_GPP[4] is active high */
	temp = mv64x60_read(&bh, MV64x60_GPP_LEVEL_CNTL);
	temp = temp & 0xffffffef;
	mv64x60_write(&bh, MV64x60_GPP_LEVEL_CNTL, temp);

	/* Check wether MV is in monarch mode using MV64360_GPP[4] */

	temp = mv64x60_read(&bh, MV64x60_GPP_VALUE);
	if ((temp & 0x00000010) == 0x00000000)
		return 1;
	else
		return 0;
}

static int
prpmc275_pci_exclude_device(u8 bus, u8 devfn)
{
	struct pci_controller	*hose;

	/*
	 * Monarch is allowed to access all PCI devices. Non-monarch is
	 * only allowed to access its own Marvell-64x60 chip.
	 */
	if (monarch)
		return PCIBIOS_SUCCESSFUL;

	hose = pci_bus_to_hose(bus);

	if ((PCI_SLOT(devfn) == 0) && (hose->first_busno == bus))
		return PCIBIOS_SUCCESSFUL;
	else
		return PCIBIOS_DEVICE_NOT_FOUND;
}
#endif

/*
 * DESCRIPTION: ppc_md memory size callback
 */
unsigned long __init
ppmc275_find_end_of_memory(void)
{
	return mv64x60_get_mem_size(CONFIG_MV64X60_NEW_BASE,
				    MV64x60_TYPE_MV64360);
}

/*
 * Force PPMC275 Board PCI interrupt routing.
 */
static int __init
ppmc275_map_irq(struct pci_dev *dev, unsigned char idsel, unsigned char pin)
{
	struct pci_controller *hose = pci_bus_to_hose(dev->bus->number);

	if (hose->index == 0) {
		static char pci_irq_table[][4] =
		    /*
		     *      PCI IDSEL/INTPIN->INTLINE 
		     *         A   B   C   D
		     */
		{
			/* IDSEL 1 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 2 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 3 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 4 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 5 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 6 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 7 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 8 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 9 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 10 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 11 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 12 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 13 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 14 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 15 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 16 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 17 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 18 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 19 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 20 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 21 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 22 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 23 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 24 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 25 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 26 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 27 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 28 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 29 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 30 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D},
			/* IDSEL 31 - PCI bus 0 */
			{PPMC275_PCI_0_IRQ, PPMC275_PCI_0_IRQ_B,
			 PPMC275_PCI_0_IRQ_C, PPMC275_PCI_0_IRQ_D}
		};

		const long min_idsel = 1, max_idsel = 32, irqs_per_slot = 4;
		return PCI_IRQ_TABLE_LOOKUP;
	} else {
		static char pci_irq_table[][4] =
		    /*
		     *      PCI IDSEL/INTPIN->INTLINE 
		     *         A   B   C   D
		     */
		{
			{PPMC275_PCI_1_IRQ, 0, 0, 0},	/* IDSEL 7 - PCI bus 1 */
			{PPMC275_PCI_1_IRQ, 0, 0, 0},	/* IDSEL 8 - PCI bus 1 */
			{PPMC275_PCI_1_IRQ, 0, 0, 0},	/* IDSEL 9 - PCI bus 1 */
		};

		const long min_idsel = 7, max_idsel = 9, irqs_per_slot = 4;
		return PCI_IRQ_TABLE_LOOKUP;
	}
}

static void __init
ppmc275_setup_bridge(void)
{
	struct mv64x60_setup_info si;
	int i;

	if (ppc_md.progress)
		ppc_md.progress("ppmc275_setup_bridge: enter", 0);

	memset(&si, 0, sizeof (si));

	si.phys_reg_base = CONFIG_MV64X60_NEW_BASE;

	si.pci_0.enable_bus = 1;
	si.pci_0.pci_io.cpu_base = PPMC275_PCI0_IO_START_PROC_ADDR;
	si.pci_0.pci_io.pci_base_hi = 0;
	si.pci_0.pci_io.pci_base_lo = PPMC275_PCI0_IO_START_PCI_ADDR;
	si.pci_0.pci_io.size = PPMC275_PCI0_IO_SIZE;
	si.pci_0.pci_io.swap = MV64x60_CPU2PCI_SWAP_NONE;
	si.pci_0.pci_mem[0].cpu_base = PPMC275_PCI0_MEM_START_PROC_ADDR;
	si.pci_0.pci_mem[0].pci_base_hi = PPMC275_PCI0_MEM_START_PCI_HI_ADDR;
	si.pci_0.pci_mem[0].pci_base_lo = PPMC275_PCI0_MEM_START_PCI_LO_ADDR;
	si.pci_0.pci_mem[0].size = PPMC275_PCI0_MEM_SIZE;
	si.pci_0.pci_mem[0].swap = MV64x60_CPU2PCI_SWAP_NONE;
	si.pci_0.pci_cmd_bits = 0;
	si.pci_0.latency_timer = 0x80;

	si.pci_1.enable_bus = 1;
	si.pci_1.pci_io.cpu_base = PPMC275_PCI1_IO_START_PROC_ADDR;
	si.pci_1.pci_io.pci_base_hi = 0;
	si.pci_1.pci_io.pci_base_lo = PPMC275_PCI1_IO_START_PCI_ADDR;
	si.pci_1.pci_io.size = PPMC275_PCI1_IO_SIZE;
	si.pci_1.pci_io.swap = MV64x60_CPU2PCI_SWAP_NONE;
	si.pci_1.pci_mem[0].cpu_base = PPMC275_PCI1_MEM_START_PROC_ADDR;
	si.pci_1.pci_mem[0].pci_base_hi = PPMC275_PCI1_MEM_START_PCI_HI_ADDR;
	si.pci_1.pci_mem[0].pci_base_lo = PPMC275_PCI1_MEM_START_PCI_LO_ADDR;
	si.pci_1.pci_mem[0].size = PPMC275_PCI1_MEM_SIZE;
	si.pci_1.pci_mem[0].swap = MV64x60_CPU2PCI_SWAP_NONE;
	si.pci_1.pci_cmd_bits = 0;
	si.pci_1.latency_timer = 0x80;

	for (i = 0; i < MV64x60_CPU2MEM_WINDOWS; i++) {
#if defined(CONFIG_NOT_COHERENT_CACHE)
		si.cpu_prot_options[i] = 0;
		si.enet_options[i] = MV64360_ENET2MEM_SNOOP_NONE;
		si.mpsc_options[i] = MV64360_MPSC2MEM_SNOOP_NONE;
		si.idma_options[i] = MV64360_IDMA2MEM_SNOOP_NONE;

		si.pci_1.acc_cntl_options[i] =
		    MV64360_PCI_ACC_CNTL_SNOOP_NONE |
		    MV64360_PCI_ACC_CNTL_SWAP_NONE |
		    MV64360_PCI_ACC_CNTL_MBURST_128_BYTES |
		    MV64360_PCI_ACC_CNTL_RDSIZE_256_BYTES;
#else
		si.cpu_prot_options[i] = 0;
		si.enet_options[i] = MV64360_ENET2MEM_SNOOP_NONE;	/* errata */
		si.mpsc_options[i] = MV64360_MPSC2MEM_SNOOP_NONE;	/* errata */
		si.idma_options[i] = MV64360_IDMA2MEM_SNOOP_NONE;	/* errata */

		si.pci_1.acc_cntl_options[i] =
		    MV64360_PCI_ACC_CNTL_SNOOP_WB |
		    MV64360_PCI_ACC_CNTL_SWAP_NONE |
		    MV64360_PCI_ACC_CNTL_MBURST_32_BYTES |
		    MV64360_PCI_ACC_CNTL_RDSIZE_32_BYTES;
#endif
	}

	/* Lookup PCI host bridges */
	if (mv64x60_init(&bh, &si))
		printk("Bridge initialization failed.\n");

#ifdef CONFIG_NONMONARCH_SUPPORT
	monarch = read_monarch_status();
	printk(KERN_INFO "Running as %sMonarch\n", monarch ? "" : "Non-");
	ppc_md.pci_exclude_device = prpmc275_pci_exclude_device;
#endif

	pci_dram_offset = 0;	/* sys mem at same addr on PCI & cpu bus */
	ppc_md.pci_swizzle = common_swizzle;
	ppc_md.pci_map_irq = ppmc275_map_irq;

	mv64x60_set_bus(&bh, 0, 0);

	bh.hose_a->first_busno = 0;
	bh.hose_a->last_busno = 0xff;
	bh.hose_a->last_busno = pciauto_bus_scan(bh.hose_a, 0);

	bh.hose_b->first_busno = bh.hose_a->last_busno + 1;
	mv64x60_set_bus(&bh, 1, bh.hose_b->first_busno);
	bh.hose_b->last_busno = 0xff;
	bh.hose_b->last_busno = pciauto_bus_scan(bh.hose_b,
						 bh.hose_b->first_busno);
}

void __init
ppmc275_setup_peripherals(void)
{
	u32 base, data;

	mv64x60_set_32bit_window(&bh, MV64x60_CPU2BOOT_WIN,
				 PPMC275_BOOT_FLASH_BASE,
				 PPMC275_BOOT_FLASH_SIZE, 0);
	bh.ci->enable_window_32bit(&bh, MV64x60_CPU2BOOT_WIN);

	/* Assume firmware set up window sizes correctly for dev 0 & 1 */
	mv64x60_get_32bit_window(&bh, MV64x60_CPU2DEV_0_WIN, &base,
				 &prpmc275_flash_size_0);

	if (prpmc275_flash_size_0 > 0) {
		mv64x60_set_32bit_window(&bh, MV64x60_CPU2DEV_0_WIN,
					 PPMC275_USER_FLASH_BASE,
					 prpmc275_flash_size_0, 0);
		bh.ci->enable_window_32bit(&bh, MV64x60_CPU2DEV_0_WIN);
	}

	mv64x60_get_32bit_window(&bh, MV64x60_CPU2DEV_1_WIN, &base,
				 &prpmc275_flash_size_1);

	if (prpmc275_flash_size_1 > 0) {
		mv64x60_set_32bit_window(&bh, MV64x60_CPU2DEV_1_WIN,
					 PPMC275_USER_FLASH_BASE +
					 prpmc275_flash_size_0,
					 prpmc275_flash_size_1, 0);
		bh.ci->enable_window_32bit(&bh, MV64x60_CPU2DEV_1_WIN);
	}

	mv64x60_set_32bit_window(&bh, MV64x60_CPU2SRAM_WIN,
				 PPMC275_INTERNAL_SRAM_BASE, MV64360_SRAM_SIZE,
				 0);
	bh.ci->enable_window_32bit(&bh, MV64x60_CPU2SRAM_WIN);

#ifdef CONFIG_NOT_COHERENT_CACHE
	mv64x60_write(&bh, MV64360_SRAM_CONFIG, 0x001600b0);
#else
	mv64x60_write(&bh, MV64360_SRAM_CONFIG, 0x001600b2);
#endif

	sram_base = ioremap(PPMC275_INTERNAL_SRAM_BASE, MV64360_SRAM_SIZE);
	memset(sram_base, 0, MV64360_SRAM_SIZE);

	/*
	 * Enabling of PCI internal-vs-external arbitration
	 * is a platform- and errata-dependent decision.
	 */

	mv64x60_set_bits(&bh, MV64x60_CPU_MASTER_CNTL, (1 << 9));

	/* MPP 27,29 */
	mv64x60_clr_bits(&bh, MV64x60_MPP_CNTL_3,
			 (1 << 12) | (1 << 13) | (1 << 14) | (1 << 15) |
			 (1 << 20) | (1 << 21) | (1 << 22) | (1 << 23));

	/* MPP 16, MPP 17 */
	mv64x60_clr_bits(&bh, MV64x60_MPP_CNTL_2,
			 (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) |
			 (1 << 5) | (1 << 6) | (1 << 7));

	/* MPP 12, MPP 13 */
	mv64x60_clr_bits(&bh, MV64x60_MPP_CNTL_1,
			 (1 << 16) | (1 << 17) | (1 << 18) | (1 << 19) |
			 (1 << 20) | (1 << 21) | (1 << 22) | (1 << 23));

	/*
	 * Define GPP 27 interrupt polarity as active low
	 * input signal and level triggered
	 */
	mv64x60_set_bits(&bh, MV64x60_GPP_LEVEL_CNTL,
			 (1 << 27) | (1 << 29) | (1 << 16) | (1 << 17) |
			 (1 << 12) | (1 << 13));

#ifdef CONFIG_NONMONARCH_SUPPORT
	if (monarch) {
		mv64x60_clr_bits(&bh, MV64x60_GPP_IO_CNTL,
				 (1 << 27) | (1 << 29) | (1 << 16) | (1 << 17) |
				 (1 << 12) | (1 << 13));
	} else {
		mv64x60_clr_bits(&bh, MV64x60_GPP_IO_CNTL,
				 (1 << 12) | (1 << 13));

		mv64x60_set_bits(&bh, MV64x60_GPP_IO_CNTL,
				 (1 << 27) | (1 << 29) | (1 << 16) | (1 << 17));
	}
#else
	mv64x60_clr_bits(&bh, MV64x60_GPP_IO_CNTL,
			 (1 << 27) | (1 << 29) | (1 << 16) | (1 << 17) |
			 (1 << 12) | (1 << 13));
#endif

	/* Config GPP interrupt controller to respond to level trigger */
	mv64x60_set_bits(&bh, MV64x60_COMM_ARBITER_CNTL, (1 << 10));

	/*
	 * Dismiss and then enable interrupt on GPP interrupt cause for CPU #0
	 */
	mv64x60_write(&bh, MV64x60_GPP_INTR_CAUSE,
		      ~((1 << 27) | (1 << 26) | (1 << 25)));
	mv64x60_set_bits(&bh, MV64x60_GPP_INTR_MASK,
			 (1 << 27) | (1 << 26) | (1 << 25));

	/*
	 * Dismiss and then enable interrupt on CPU #0 high cause register
	 * BIT25 summarizes GPP interrupts 8-15 (Need MPP 12,13)
	 * BIT26 summarizes GPP interrupts 16-23 (Need MPP 16,17)
	 * BIT27 summarizes GPP interrupts 24-31 (Need MPP 27,29)
	 */
	mv64x60_set_bits(&bh, MV64360_IC_CPU0_INTR_MASK_HI,
			 (1 << 27) | (1 << 26) | (1 << 25));

	/*
	 * Change DRAM read buffer assignment.
	 * Assign read buffer 0 dedicated only for CPU, and the rest read buffer 1.
	 */
	data = mv64x60_read(&bh, MV64360_SDRAM_CONFIG);
	data = data & 0x03ffffff;
	data = data | 0xf8000000;
	mv64x60_write(&bh, MV64360_SDRAM_CONFIG, data);
}

/* Second phase board init, called after other (architecture common)
 * low-level services have been initialized.
  */
static void
prpmc275_init2(void)
{
	u32 data;

#ifdef CONFIG_MV64X60_WDT
	/* Configure MPP19 as watchdog WDE */
	data = mv64x60_read(&bh, MV64x60_MPP_CNTL_2);
	data &= ~(0xf << 12);
	data |= (0x4 << 12);
	mv64x60_write(&bh, MV64x60_MPP_CNTL_2, data);

	/* Configure, MPP24 as watchdog NMI */
	data = mv64x60_read(&bh, MV64x60_MPP_CNTL_3);
	data &= ~(0xf << 0);
	data |= (0x4 << 0);
	mv64x60_write(&bh, MV64x60_MPP_CNTL_3, data);

	/* Make sure WatchDog is disabled */
	data = mv64x60_read(&bh, MV64x60_WDT_WDC);
	if (data & (1 << 31)) {
		data &= ~(0x3 << 24);
		data |= (0x1 << 24);
		mv64x60_write(&bh, MV64x60_WDT_WDC, data);
		data &= ~(0x3 << 24);
		data |= (0x2 << 24);
		mv64x60_write(&bh, MV64x60_WDT_WDC, data);
	}
#endif
}

#ifdef CONFIG_MTD_PHYSMAP
static struct mtd_partition prpmc275_partitions[] = {
	{
	 .name = "Kernel",
	 .offset = 0,
	 .size = 0x300000,
	 },
	{
	 .name = "Filesystem",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 0x1600000,
	 },
	{
	 .name = "JFFS2",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,
	 }
};

static int __init
prpmc275_setup_mtd(void)
{
	u32 size;

	size = prpmc275_flash_size_0 + prpmc275_flash_size_1;
	if (!size)
		return -ENOMEM;

	if (size > PPMC275_USER_FLASH_SIZE)
		size = PPMC275_USER_FLASH_SIZE;

	physmap_configure(PPMC275_USER_FLASH_BASE, size, 4, NULL);
	physmap_set_partitions(prpmc275_partitions,
			       sizeof (prpmc275_partitions) /
			       sizeof (struct mtd_partition));

	return 0;
}

arch_initcall(prpmc275_setup_mtd);
#endif

static void __init
ppmc275_setup_arch(void)
{
	if (ppc_md.progress)
		ppc_md.progress("ppmc275_setup_arch: enter", 0);

	loops_per_jiffy = 50000000 / HZ;

#ifdef CONFIG_BLK_DEV_INITRD
	if (initrd_start)
		ROOT_DEV = Root_RAM0;
	else
#endif
#ifdef	CONFIG_ROOT_NFS
		ROOT_DEV = Root_NFS;
#else
		ROOT_DEV = Root_SDA2;
#endif

	/* Enable L2 cache */
	_set_L2CR(_get_L2CR() | L2CR_L2E);

#ifdef	CONFIG_DUMMY_CONSOLE
	conswitchp = &dummy_con;
#endif

	ppmc275_setup_bridge();
	ppmc275_setup_peripherals();

	if (ppc_md.progress)
		ppc_md.progress("ppmc275_setup_arch: exit", 0);

	return;
}

/* Platform device data fixup routines. */
#if defined(CONFIG_SERIAL_MPSC)
static void __init
ppmc275_fixup_mpsc_pdata(struct platform_device *pdev)
{
	struct mpsc_pdata *pdata;
	pdata = (struct mpsc_pdata *) pdev->dev.platform_data;

	pdata->max_idle = 40;
	pdata->default_baud = PPMC275_DEFAULT_BAUD;
	pdata->brg_clk_src = PPMC275_MPSC_CLK_SRC;
	pdata->brg_clk_freq = PPMC275_BUS_FREQ;
}
#endif

#if defined(CONFIG_SENSORS_EEPROM)
static struct {
	char *bus_id;
	u16 eeprom_size;
} eeprom_map[] = {
	{"0-0052", 8192},
	{"0-0053", 8192},
	{"0-0054", 8192},
};

static void __init
ppmc275_fixup_eeprom_pdata(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(eeprom_map); i++)
		if (!strncmp(pdev->dev.bus_id, eeprom_map[i].bus_id,
			     BUS_ID_SIZE)) {
			pdev->dev.platform_data = &eeprom_map[i].eeprom_size;
		}
}
#endif

static int __init
ppmc275_platform_notify(struct device *dev)
{
	static struct {
		char *bus_id;
		void ((*rtn) (struct platform_device * pdev));
	} dev_map[] = {
#if defined(CONFIG_SERIAL_MPSC)
		{
		MPSC_CTLR_NAME ".0", ppmc275_fixup_mpsc_pdata}, {
		MPSC_CTLR_NAME ".1", ppmc275_fixup_mpsc_pdata},
#endif
#if defined(CONFIG_SENSORS_EEPROM)
		{"0-0052", ppmc275_fixup_eeprom_pdata},
		{"0-0053", ppmc275_fixup_eeprom_pdata},
		{"0-0054", ppmc275_fixup_eeprom_pdata},
#endif
	};

	struct platform_device *pdev;
	int i;
	if (dev && dev->bus_id)
		for (i = 0; i < ARRAY_SIZE(dev_map); i++)
			if (!strncmp(dev->bus_id, dev_map[i].bus_id,
				     BUS_ID_SIZE)) {
				pdev = container_of(dev,
						    struct platform_device,
						    dev);
				dev_map[i].rtn(pdev);
			}

	return 0;
}

static void
ppmc275_restart(char *cmd)
{
	volatile ulong i = 10000000;
	unsigned int gpio_io_ctrl;
	unsigned int mpp_ctrl_02, save;

	local_irq_disable();

	/* Set exception prefix high - to the firmware */
	_nmask_and_or_msr(0, MSR_IP);

	printk("mv64360_reset_board: begin\n");

	mpp_ctrl_02 = mv64x60_read(&bh, MV64x60_MPP_CNTL_2);
	mpp_ctrl_02 &= 0xffff0fff;
	mv64x60_write(&bh, MV64x60_MPP_CNTL_2, mpp_ctrl_02);

	gpio_io_ctrl = mv64x60_read(&bh, MV64x60_GPP_IO_CNTL);
	gpio_io_ctrl |= 0x00080000;
	mv64x60_write(&bh, MV64x60_GPP_IO_CNTL, gpio_io_ctrl);

	save = gpio_io_ctrl = mv64x60_read(&bh, MV64x60_GPP_LEVEL_CNTL);
	gpio_io_ctrl |= 0x00080000;
	mv64x60_write(&bh, MV64x60_GPP_LEVEL_CNTL, gpio_io_ctrl);

	while (i-- > 0) ;
	mv64x60_write(&bh, MV64x60_GPP_LEVEL_CNTL, save);
	panic("restart failed\n");
}

static void
ppmc275_halt(void)
{
	local_irq_disable();
	while (1) ;
}

static void
ppmc275_power_off(void)
{
	ppmc275_halt();
}

static int
ppmc275_show_cpuinfo(struct seq_file *m)
{
	uint pvid;

	pvid = mfspr(PVR);
	seq_printf(m, "vendor\t\t: Marvell/Galileo\n");
	seq_printf(m, "machine\t\t: PPMC-275\n");
	seq_printf(m, "PVID\t\t: 0x%x, vendor: %s\n",
		   pvid, (pvid & (1 << 15) ? "IBM" : "Motorola"));

	return 0;
}

static void __init
ppmc275_calibrate_decr(void)
{
	ulong freq;

	freq = PPMC275_BUS_FREQ / 4;

	printk("time_init: decrementer frequency = %lu.%.6lu MHz\n",
	       freq / 1000000, freq % 1000000);

	tb_ticks_per_jiffy = freq / HZ;
	tb_to_us = mulhwu_scale_factor(freq, 1000000);
	us_to_tb = freq / 1000000;

	return;
}

/*
 * Configure fixed memory-mapped IO
 */
static void __init
ppmc275_map_io(void)
{
#if defined(CONFIG_SERIAL_TEXT_DEBUG) || defined(CONFIG_KGDB_MPSC)
	io_block_mapping(CONFIG_MV64X60_NEW_BASE, CONFIG_MV64X60_NEW_BASE,
			 0x100000, _PAGE_IO);
#endif
}

#if defined(CONFIG_I2C_MV64XXX) && defined(CONFIG_SENSORS_MAX6900)
extern ulong max6900_get_rtc_time(void);
extern int max6900_set_rtc_time(ulong);

static int __init
prpmc275_rtc_hookup(void)
{
	struct timespec tv;

	ppc_md.get_rtc_time = max6900_get_rtc_time;
	ppc_md.set_rtc_time = max6900_set_rtc_time;

	tv.tv_nsec = 0;
	tv.tv_sec = (ppc_md.get_rtc_time) ();

	do_settimeofday(&tv);

	return 0;
}

late_initcall(prpmc275_rtc_hookup);
#endif

/*
 * Set BAT 3 to map 0xf0000000 to end of physical memory space.
 * This configures a (temporary) bat mapping for early access to
 * device I/O
 */
static __inline__ void
ppmc275_set_bat(void)
{
	mb();
	mtspr(DBAT3U, 0xf0001ffe);
	mtspr(DBAT3L, 0xf000002a);
	mb();
}

void __init
platform_init(unsigned long r3, unsigned long r4, unsigned long r5,
	      unsigned long r6, unsigned long r7)
{
	parse_bootinfo(find_bootinfo());

	isa_mem_base = 0;

	ppc_md.setup_arch = ppmc275_setup_arch;
	ppc_md.show_cpuinfo = ppmc275_show_cpuinfo;
	ppc_md.irq_canonicalize = NULL;
	ppc_md.init_IRQ = mv64360_init_irq;
	ppc_md.get_irq = mv64360_get_irq;
	ppc_md.init = prpmc275_init2;
	ppc_md.restart = ppmc275_restart;
	ppc_md.power_off = ppmc275_power_off;
	ppc_md.halt = ppmc275_halt;
	ppc_md.find_end_of_memory = ppmc275_find_end_of_memory;
	ppc_md.calibrate_decr = ppmc275_calibrate_decr;
	ppc_md.setup_io_mappings = ppmc275_map_io;

	ppmc275_set_bat();

#if defined(CONFIG_SERIAL_TEXT_DEBUG)
	ppc_md.progress = mv64x60_mpsc_progress;
	mv64x60_progress_init(CONFIG_MV64X60_NEW_BASE);
#endif

#if defined(CONFIG_SERIAL_MPSC)
	platform_notify = ppmc275_platform_notify;
#endif

#ifdef CONFIG_BLK_DEV_INITRD
	if (r4) {
		initrd_start = r4 + KERNELBASE;
		initrd_end = r5 + KERNELBASE;
	}
#endif				/* CONFIG_BLK_DEV_INITRD */

	return;
}
