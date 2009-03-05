/*
 * arch/xtensa/platform-xt2000/pci.c
 *
 * PCI functions for V320USC host PCI bridge
 *
 * Copyright (C) 2001 - 2004 Tensilica Inc.
 *
 * Chris Zankel <chris@zankel.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */


#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/init.h>
#include <linux/irq.h>

#include <asm/irq.h>
#include <asm/system.h>
#include <asm/platform/pci.h>

#include <asm/processor.h>
#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/pci-bridge.h>
#include <asm/platform/pci.h>

#ifdef __XTENSA_EL__
#define _V3USCREG_H_CL_
#elif defined (__XTENSA_EB__)
#define _V3USCREG_H_CB_
#else
#error endianess not defined
#endif
#include <asm/platform/pci_v320usc.h>


extern struct pci_ops v320usc_pci_ops;
extern void v320usc_pci_init(void);

char *_v3uscp = (char *)PCI_V320_BASE;

static spinlock_t v320usc_lock = SPIN_LOCK_UNLOCKED;


/* ------------------------------------------------------------------------- */

/* V320 interrupt device. */

static void v320usc_enable_irq(unsigned int irq)
{
	int lirq = irq - XTENSA_NR_IRQS;

	/* INTA..INTC */
	if (lirq >=0 && lirq <= 2)
		V3USC_INT_CFG3 |= (INT_CFGX_INT0 << lirq);
	/* INTD */
	/* if (lirq == 3) */

}

static unsigned int v320usc_startup_irq(unsigned int irq)
{
	v320usc_enable_irq(irq);

	return 0;
}

static void v320usc_disable_irq(unsigned int irq)
{
	int lirq = irq + XTENSA_NR_IRQS;

	if (lirq >=0 && lirq <= 3)
		V3USC_INT_CFG3 &= ~(INT_CFGX_INT0 << lirq);
}

static void v320usc_end_irq(unsigned int irq)
{

}

struct hw_interrupt_type v320usc_irq_type = {
	"V320USC-IRQ",
	v320usc_startup_irq,
	v320usc_disable_irq,		/* shutdown_irq */
	v320usc_enable_irq,
	v320usc_disable_irq,
	v320usc_disable_irq,		/* mask_and_ack */
	v320usc_end_irq
};

extern unsigned int do_IRQ(int irq, struct pt_regs *regs);

irqreturn_t v320usc_action(int irq, void *dev_id, struct pt_regs *regs)
{
	int cause;
	int ret = IRQ_NONE;

	cause = V3USC_INT_STAT & V3USC_INT_CFG3;

	if (cause & INT_CFGX_INT0)
		ret = do_IRQ(XTENSA_NR_IRQS, regs);
	if (cause & INT_CFGX_INT1)
		ret = do_IRQ(XTENSA_NR_IRQS+1, regs);
	if (cause & INT_CFGX_INT2)
		ret = do_IRQ(XTENSA_NR_IRQS+2, regs);

	V3USC_INT_STAT = 0xFFFFFFFF;
	return ret;
}

void __init v320_init_irq(void)
{
	int i;

	/* Enable V3_int and intd interrupt */

	writel(readl(XT2000_IMASK_PADDR)|(1<<4)|(1<<5), XT2000_IMASK_PADDR);

	for (i = XTENSA_NR_IRQS; i < NR_IRQS; i++)
		irq_desc[i].handler = &v320usc_irq_type;

	if(request_irq(XCHAL_EXTINT6_NUM, v320usc_action, 0, "v320usc", 0)) {

		printk(KERN_ERR "Unable to get V320USC IRQ %d for cascade\n",
		       XCHAL_EXTINT6_NUM);
	}
}


/* -------------------------------------------------------------------------- */

/*
 * Map slot and pin number to an interrupt number. Since we don't do
 * swizzling, this is fairly easy.
 */

static int __init map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	if (pin == 0)
		return -1;
	return XTENSA_NR_IRQS + pin - 1;
}


/*
 * This function initializes the V3 bridge and enumerates the PCI bus.
 */

void __init platform_pcibios_init(void)
{
	struct pci_controller *pci_ctrl;

	/* Setup pci_controller */
	pci_ctrl = pcibios_alloc_controller();

	if (!pci_ctrl) {
		printk("PCI: cannot allocate space for the pci controller\n");
		return;
	}

 	/* Move V3USC to 0xFD000000 */
	V3USC_LB_REG_BASE_W = PCI_V320_BASE >> 16;
	*(unsigned int*)(XT2000_V3CFG_PADDR) = 0;
	
	/* Initialize the V3 */
	v320usc_pci_init();

	/* Setup the controller */
	pci_ctrl->first_busno = 0;
	pci_ctrl->last_busno = 0xff;
	pci_ctrl->ops = &v320usc_pci_ops;
	pci_ctrl->map_irq = map_irq;

	pci_ctrl->io_space.start = PCI_IO_SPACE_PCI_BASE;
	pci_ctrl->io_space.end = PCI_IO_SPACE_PCI_BASE + PCI_IO_SPACE_SIZE;
	pci_ctrl->io_space.base = PCI_IO_SPACE_CPU_BASE;
	pci_ctrl->mem_space.start = PCI_MEM_SPACE_PCI_BASE;
	pci_ctrl->mem_space.end = PCI_MEM_SPACE_PCI_BASE + PCI_MEM_SPACE_SIZE;
	pci_ctrl->mem_space.base = 0;

	pcibios_init_resource(&pci_ctrl->io_resource,
			PCI_IO_SPACE_PCI_BASE, 
			PCI_IO_SPACE_PCI_BASE + PCI_IO_SPACE_SIZE,
			IORESOURCE_IO, "PCI host bridge");
	pcibios_init_resource(&pci_ctrl->mem_resources[0],
			PCI_MEM_SPACE_CPU_BASE, 
			PCI_MEM_SPACE_CPU_BASE + PCI_MEM_SPACE_SIZE,
			IORESOURCE_MEM, "PCI host bridge");

	/* Enumerate the PCI bus */

	pci_ctrl->last_busno = pciauto_bus_scan(pci_ctrl,pci_ctrl->first_busno);
}

/*
 * The V320USC PCI interface chip provides two windows from the local
 * bus memory into the PCI 'spaces'.
 * 
 * Address                    Usage
 * 
 * V3_PCI_LOWER_IO ...        PCI I/O space		- Base0
 * ... V3_PCI_UPPER_IO
 * V3_PCI_LOWER_MEM	      PCI MEM space		- Base1
 * ... V3_PCI_UPPER_MEM
 * V3_PCI_LOWER_PREFMEM       PCI MEM Prefetch space	- Not Used
 * ... V3_PCI_UPPER_PREFMEM
 * V3_PCI_LOWER_CONFIG        PCI Configuration		- Shared with I/O space
 * ... V3_PCI_UPPER_CONFIG
 * 
 * There are only two V3 windows. We use Base0 for PCI I/O and PCI CONFIG space,
 * and Base1 for the PCI MEMORY space (only in non-prefetch mode).
 */


/* 
 * Open a configuration window. This temporarily uses the IO aperture window.
 */

unsigned long v320usc_pci_open_config_window(struct pci_bus *bus, int devfn,
					     int offset)
{
	unsigned long addr, conf;
	int busnr;
	
	busnr = bus->number;
	conf = PCI_CFG_SPACE_BASE_REG_DEFAULT;
	
	/* Trap out illegal values */
	
	if (offset > 255 || busnr > 255 || devfn > 255)
		BUG();
	
	if (busnr == 0) {
		/* Type 0 config cycle: Local busnr segment */
		unsigned int idsel = 1 << PCI_SLOT(devfn);
		conf |= (idsel >> 8) & ~0xffff;
		addr = (idsel & 0x00fff800) | ((devfn & 0x7) << 8);
	} else {
		/* Type 1 config cycle: Not the local bus segment */
		/* a23..a16 = bus nr, a15..a8: device_function nr, a7..a0 = offset */
		addr = (busnr << 16) | (devfn << 8);
		conf |= 1;	/* set type 1 config cycle */
	}
	/* Set aperture window to config space */
	V3USC_LB_PCI_BASE0 = conf;
	V3USC_PCI_STAT_W |= PCI_STAT_W_M_ABORT | PCI_STAT_W_T_ABORT;

	return (unsigned long)PCI_CFG_SPACE_CPU_BASE + addr + offset;
}


/* v320usc_pci_close_config_window()
 * 
 * Close the aperture window into the config space and restore
 * the IO window.
 */

static inline void v320usc_pci_close_config_window(void)
{
	/* Reassign base0 for use by PCI IO space */
	V3USC_LB_PCI_BASE0 = PCI_IO_SPACE_BASE_REG_DEFAULT;
}

int v320usc_read_config(struct pci_bus *bus, unsigned int devfn, int offset, 
			int len, u32 *val)
{
	unsigned long addr;
	unsigned long flags;

        if ((bus->number == 0) && (PCI_SLOT(devfn) < 11))
		return PCIBIOS_DEVICE_NOT_FOUND;

	spin_lock_irqsave(&v320usc_lock, flags);
	addr = v320usc_pci_open_config_window(bus, devfn, offset);

	/* Note: the offset (off) is already aligned */

	if (len == 1)
		*val = (u32) *(volatile u8*)  (addr ^ 3);
	else if (len == 2) 
		*val = (u32) *(volatile u16*) (addr ^ 2);
	else
		*val = *(volatile u32*) addr;

	v320usc_pci_close_config_window();
	spin_unlock_irqrestore(&v320usc_lock, flags);

	return PCIBIOS_SUCCESSFUL;
}

int v320usc_write_config(struct pci_bus *bus, unsigned int devfn, int offset, 
			 int len, u32 val)
{
	unsigned long addr;
	unsigned long flags;

        if ((bus->number == 0) && (PCI_SLOT(devfn) < 11))
		return PCIBIOS_DEVICE_NOT_FOUND;

	spin_lock_irqsave(&v320usc_lock, flags);
	addr = v320usc_pci_open_config_window(bus, devfn, offset);

	if (len == 1)
		*(volatile u8*)  (addr ^ 3) = val;
	else if (len == 2) 
		*(volatile u16*) (addr ^ 2) = val;
	else
		*(volatile u32*) addr       = val;

	v320usc_pci_close_config_window();
	spin_unlock_irqrestore(&v320usc_lock, flags);

	return PCIBIOS_SUCCESSFUL;
}
struct pci_ops v320usc_pci_ops = {
	v320usc_read_config,
	v320usc_write_config
};


/*
 * v320usc_pci_init() - Initialize the V320USC device.
 */

void __init v320usc_pci_init(void)
{
	unsigned long flags;

	spin_lock_irqsave(&v320usc_lock, flags);

	/* stop the V3 chip from servicing any further PCI requests */
	V3USC_PCI_CMD_W = 0x0;
	
	/* reset the PCI bus */
	V3USC_SYSTEM_B = 0xa5;
	V3USC_SYSTEM_B &= ~SYSTEM_B_RST_OUT;
 
	/* enable bridge to PCI, plus error handling */
	V3USC_PCI_CMD_W = PCI_CMD_W_MASTER_EN
		| PCI_CMD_W_MEM_EN;
	//	| PCI_CMD_W_MWI_EN;
	//      | PCI_CMD_W_SERR_EN
	//      | PCI_CMD_W_PAR_EN;

	/* clear errors and say we do fast back-to-back transfers */
	V3USC_PCI_STAT_W = PCI_STAT_W_PAR_ERR
		| PCI_STAT_W_SYS_ERR
		| PCI_STAT_W_M_ABORT
		| PCI_STAT_W_T_ABORT
		| PCI_STAT_W_PAR_REP
		| PCI_STAT_W_FAST_BACK;

	/* reset PCI Bus Configuration Register */
	V3USC_PCI_BUS_CFG = 0x00200000;	

	/* Disable I2O */
	V3USC_PCI_I2O_MAP = 0;
	V3USC_PCI_I2O_BASE = 0;

	/* Setup PCI -> Local Memory */
	V3USC_PCI_MEM_MAP = PCI_LOCAL_MEM_MAP_REG;
	V3USC_PCI_MEM_BASE = 0;

	/* CPU -> PCI translations. */
	V3USC_LB_PCI_BASE0 = PCI_IO_SPACE_BASE_REG_DEFAULT;	/* 0: PCI-IO  */
	V3USC_LB_PCI_BASE1 = PCI_MEM_SPACE_BASE_REG_DEFAULT;	/* 1: PCI-MEM */

	/* Disable PCI access to V3 */
	V3USC_PCI_REG_BASE = 0;
  	V3USC_PCI_ROM_BASE = 0;
	V3USC_PCI_PCU_BASE = 0x0F << PCI_PCU_BASE_SIZE_SHIFT;
	
	/* disable CPU -> SDRAM */
	V3USC_LB_SDRAM_BASE = 0;

	/* Setup SDRAM controller */
	V3USC_SDRAM_CFG = (0x7 << SDRAM_CFG_REF_NDIV_SHIFT)|
		SDRAM_CFG_RP_2 |
		SDRAM_CFG_RCD_2 |
		SDRAM_CFG_TCAS_RD_2 ;

	V3USC_SDRAM_BANK0 = SDRAM_BANKX_ENABLE |
		SDRAM_BANKX_SIZE_256M |
		SDRAM_BANKX_ROW_MUX_MODE_9 |
		SDRAM_BANKX_COL_MUX_MODE_5;
 
	V3USC_SDRAM_BANK1 = 0x0;
	V3USC_SDRAM_BANK2 = 0x0;
	V3USC_SDRAM_BANK3 = 0x0;

	//V3USC_LB_BUS_CFG = 0x80000022;
	//V3USC_HS_CSR_B = 0x080; /* clear INS bit after reset */
	V3USC_LB_PCI_CTL_W = PCI_CNT_WR_PCI_FIFO_10
		| PCI_CNT_RD_PCI_FIFO_00
		| PCI_CNT_WFLUSH1_11
		| PCI_CNT_WFLUSH0_11;

	/* clear all interrupts */
	V3USC_INT_CFG0 = 0x0;
	V3USC_INT_CFG1 = 0x0;
	V3USC_INT_CFG2 = 0x0;
	V3USC_INT_CFG3 = 0x0;
	V3USC_INT_STAT = 0xFFFFFFFF;
 
	/* finally unreset the PCI bus */
	V3USC_SYSTEM_B |= SYSTEM_B_RST_OUT;
 
	spin_unlock_irqrestore(&v320usc_lock, flags);
}

/* -------------------------------------------------------------------------- */

/*
 * Initialize v320.
 */

static int __init xt2000_v320_init(void)
{
	v320_init_irq();
	return 0;
}

postcore_initcall(xt2000_v320_init);

