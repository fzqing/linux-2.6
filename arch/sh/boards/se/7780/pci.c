/*
 * linux/arch/sh/boards/se/7780/pci.c
 *
 * Copyright (C)  Takashi Kusuda  (Nov, 2004)
 *  modified to support MS7780SE01(SH7780 SolutionEngine).
 *
 * Highly leveraged from pci-bigsur.c, written by Dustin McIntire.
 *
 * May be copied or modified under the terms of the GNU General Public
 * License.  See linux/COPYING for more information.
 *
 * PCI initialization for Hitachi MS7780SE01
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pci.h>

#include <asm/io.h>
#include <asm/cpu/pci-sh7780.h>
#include <asm/mach/map.h>

#define PCIMCR_MRSET_OFF	0xBFFFFFFF
#define PCIMCR_RFSH_OFF		0xFFFFFFFB

#define PCIPAR PCI_REG(SH7780_PCI_PAR)
#define PCIPDR PCI_REG(SH7780_PCI_PDR)

static void __init sh7780se_pci_write_config(unsigned long busNo,
                                             unsigned long devNo,
                                             unsigned long fncNo,
                                             unsigned long cnfAdd,
                                             unsigned long cnfData)
{
	ctrl_outl((0x80000000
		+ ((busNo & 0xff)<<16)
		+ ((devNo & 0x1f)<<11)
		+ ((fncNo & 0x7)<<8)
		+ (cnfAdd & 0xfc)), PCIPAR);

	ctrl_outl(cnfData, PCIPDR);
}

static long __init sh7780se_pci_read_config(unsigned long busNo,
                                            unsigned long devNo,
                                            unsigned long fncNo,
                                            unsigned long cnfAdd)
{
	ctrl_outl((0x80000000
		+ ((busNo & 0xff)<<16)
		+ ((devNo & 0x1f)<<11)
		+ ((fncNo & 0x7)<<8)
		+ (cnfAdd & 0xfc)), PCIPAR);

	return (ctrl_inl(PCIPDR));
}

#define PCI_SLOT1_DEVNO	0
#define PCI_SLOT2_DEVNO	1
#define S_ATA_DEVNO	2
#define USB_H_DEVNO	3

#if defined(CONFIG_USB)
/*
 * Configure the NEC USB Host Controller chip
 */
static void __init init_7780se_usbc(void)
{
	unsigned long tmp, value;

	/* 4 port setting */
	tmp = sh7780se_pci_read_config(0, USB_H_DEVNO, 0, 0xe0);
	value = ((tmp & ~7) | 0x5);
	sh7780se_pci_write_config(0, USB_H_DEVNO, 0, 0xe0, value);

	/* ehci setting */
	tmp = sh7780se_pci_read_config(0, USB_H_DEVNO, 0, 0xe4);
	value = tmp & 0xfffffffe;
	sh7780se_pci_write_config(0, USB_H_DEVNO, 0, 0xe4, value);
	printk("PCIBIOS: USB Host port num set 4\n");
}
#endif

#if defined(CONFIG_CARDBUS)
/*
 * Configure the T.I PCI1420 PC Card Controller
 */
static void __init init_7780se_pci1420(void)
{
	unsigned long tmp, value;

	tmp = sh7780se_pci_read_config(0, PCCARD_DEVNO, 0, 0x80);
	value = (tmp & 0x1fffffff)|0x20000000; /* tie */
	sh7780se_pci_write_config(0, PCCARD_DEVNO, 0, 0x80, value); /* set sys ctl reg(0x80) */

	sh7780se_pci_write_config(0, PCCARD_DEVNO, 0, 0x8c, 0x0ac2cd22); /* set mfunc reg(0x8c) */

	printk("PCIBIOS: PC Card multifunc reg set.\n");
}
#endif

#if defined(CONFIG_BLK_DEV_SIIMAGE)
/*
 * Configure the Silicon Image Serial ATA Controller
 */
static void __init init_7780se_sata(void)
{
	/* Nothing to do */
}
#endif

/*
 * Only long word accesses of the PCIC's internal local registers and the
 * configuration registers from the CPU is supported.
 */
#define PCIC_WRITE(x,v) writel((v), PCI_REG(x))
#define PCIC_READ(x) readl(PCI_REG(x))

/*
 * Description:  This function sets up and initializes the pcic, sets
 * up the BARS, maps the DRAM into the address space etc, etc.
 */
int __init pcibios_init_platform(void)
{
   ctrl_outl(0x00000001, SH7780_PCI_ECR);

   /* Enable all interrupts, so we know what to fix */
   PCIC_WRITE(SH7780_PCI_IMR, 0x0000C3FF);
   PCIC_WRITE(SH7780_PCI_AINTM, 0x0000380F);

   /* Set up standard PCI config registers */
   ctrl_outw(0xFB00, PCI_REG(SH7780_PCI_STATUS));
   ctrl_outw(0x0047, PCI_REG(SH7780_PCI_CMD));
   ctrl_outb(  0x00, PCI_REG(SH7780_PCI_PIF));
   ctrl_outb(  0x00, PCI_REG(SH7780_PCI_SUB));
   ctrl_outb(  0x06, PCI_REG(SH7780_PCI_BCC));
   ctrl_outw(0x1912, PCI_REG(SH7780_PCI_SVID));
   ctrl_outw(0x0001, PCI_REG(SH7780_PCI_SID));

   ctrl_outl(0x08000000, PCI_REG(SH7780_PCI_MBAR0));  /* PCI               */
   ctrl_outl(0x08000000, PCI_REG(SH7780_PCI_LAR0));   /* SHwy              */
   ctrl_outl(0x07F00001, PCI_REG(SH7780_PCI_LSR0));   /* size 128M w/ MBAR */

   ctrl_outl(0x00000000, PCI_REG(SH7780_PCI_MBAR1));  /* unused            */
   ctrl_outl(0x00000000, PCI_REG(SH7780_PCI_LAR1));
   ctrl_outl(0x00000000, PCI_REG(SH7780_PCI_LSR1));

   ctrl_outl(0x00000000, PCI_REG(SH7780_PCI_CSCR0));
   ctrl_outl(0x00000000, PCI_REG(SH7780_PCI_CSAR0));
   ctrl_outl(0x00000000, PCI_REG(SH7780_PCI_CSCR1));
   ctrl_outl(0x00000000, PCI_REG(SH7780_PCI_CSAR1));
   ctrl_outl(0xAB000801, PCI_REG(SH7780_PCI_IBAR));   /* who? me?         */

   /*
    * Set the MBR so PCI address is one-to-one with window,
    * meaning all calls go straight through... use ifdef to
    * catch erroneous assumption.
    */
   PCIC_WRITE(SH7780_PCI_MBR0,  0xFD000000);
   PCIC_WRITE(SH7780_PCI_MBMR0, 0x00FC0000);	/* 16M */

   /* Set IOBR for window containing area specified in pci.h */
   PCIC_WRITE(SH7780_PCI_IOBR, PCIBIOS_MIN_IO & ~(SH7780_PCI_IO_SIZE-1));
   PCIC_WRITE(SH7780_PCI_IOBMR, (SH7780_PCI_IO_SIZE-1)&(7 << 18));

   /* Now turn it on... */
   PCIC_WRITE(SH7780_PCI_CR, 0xA5000C01);

   /* All done, may as well say so... */
   printk("SH7780 PCI: Finished initialization of the PCI controller\n");

   /*
    * FPGA PCISEL register initialize
    *
    *  CPU  || SLOT1 | SLOT2 | S-ATA | USB
    *  -------------------------------------
    *  INTA || INTA  | INTD  |  --   | INTB
    *  -------------------------------------
    *  INTB || INTB  | INTA  |  --   | INTC
    *  -------------------------------------
    *  INTC || INTC  | INTB  | INTA  |  --
    *  -------------------------------------
    *  INTD || INTD  | INTC  |  --   | INTA
    *  -------------------------------------
    */
   ctrl_outw(0x0013, FPGA_PCI_INTSEL1);
   ctrl_outw(0xE402, FPGA_PCI_INTSEL2);

#if defined(CONFIG_USB)
   init_7780se_usbc();
#endif
#if defined(CONFIG_CARDBUS)
   init_7780se_pci1420();
#endif
#if defined(CONFIG_BLK_DEV_SIIMAGE)
   init_7780se_sata();
#endif

   return 1;
}

int __init pcibios_map_platform_irq(u8 slot, u8 pin)
{
	/*
	 * IDSEL = AD16  PCI slot
	 * IDSEL = AD17  PCI slot
	 * IDSEL = AD18  Serial ATA Controller (Silicon Image SiL3512A)
	 * IDSEL = AD19  USB Host Controller (NEC uPD7210100A)
	 */

	int irq[4][16] = {
/* IDSEL       [16][17][18][19][20][21][22][23][24][25][26][27][28][29][30][31] */
/* INTA */     { 65, 68, 67, 68, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
/* INTB */     { 66, 65, -1, 65, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
/* INTC */     { 67, 66, -1, 66, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
/* INTD */     { 68, 67, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	};

	return irq[pin-1][slot];
}

static struct resource sh7780_io_resource = {
	.name   = "SH7780 IO",
	.start  = 0x1000,
	.end    = 0x1000 + SH7780_PCI_IO_SIZE - 1,
	.flags  = IORESOURCE_IO
};

static struct resource sh7780_mem_resource = {
	.name   = "SH7780 mem",
	.start  = SH7780_PCI_MEMORY_BASE,
	.end    = SH7780_PCI_MEMORY_BASE + SH7780_PCI_MEM_SIZE - 1,
	.flags  = IORESOURCE_MEM
};

extern struct pci_ops sh7780_pci_ops;

struct pci_channel board_pci_channels[] = {
	{ &sh7780_pci_ops, &sh7780_io_resource, &sh7780_mem_resource, 0, 0xff },
	{ NULL, NULL, NULL, 0, 0 },
};

