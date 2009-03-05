/*
 * arch/ppc/platforms/83xx/mpc832xe_mds.c
 *
 * Description: MPC832XE MDS board specific routines
 *
 * Author: Shlomi Gridish <gridish@freescale.com>
 *
 * Copyright (C) Freescale Semiconductor, Inc. 2005. All rights reserved.
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
#include <linux/tty.h>	/* for linux/serial_core.h */
#include <linux/serial_core.h>
#include <linux/mtd/physmap.h>
#include <linux/initrd.h>
#include <linux/module.h>
#include <linux/fsl_devices.h>

#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/atomic.h>
#include <asm/time.h>
#include <asm/io.h>
#include <asm/machdep.h>
#include <asm/prom.h>
#include <asm/ipic.h>
#ifdef CONFIG_QE
#include <asm/qe_ic.h>
#endif /* CONFIG_QE */
#include <asm/bootinfo.h>
#include <asm/pci-bridge.h>
#include <asm/mpc83xx.h>
#include <asm/irq.h>
#include <asm/kgdb.h>
#include <asm/ppc_sys.h>
#include <mm/mmu_decl.h>

#include <syslib/ppc83xx_setup.h>

#ifdef CONFIG_QE
extern void qe_reset(void);
#endif /* CONFIG_QE */

#ifndef CONFIG_PCI
unsigned long isa_io_base = 0;
unsigned long isa_mem_base = 0;
#endif /* CONFIG_PCI */

extern unsigned long total_memory;	/* in mm/init */

unsigned char __res[sizeof (bd_t)];

#define NUM_OF_PINS     32
#define NUM_OF_PAR_IOS  4

typedef struct par_io {
    struct
    {
        u32 cpodr;  /* Open drain register */
        u32 cpdata; /* Data register */
        u32 cpdir1; /* Direction register */
        u32 cpdir2; /* Direction register */
        u32 cppar1; /* Pin assignment register */
        u32 cppar2; /* Pin assignment register */
    } io_regs[NUM_OF_PAR_IOS];
} par_io_t;

typedef struct qe_par_io {
    u8  res[0xc];
    u32 cepier;  /* QE ports interrupt event register */
    u32 cepimr;  /* QE ports mask event register */
    u32 cepicr;  /* QE ports control event register */
} qe_par_io_t;

static int		qe_irq_ports[NUM_OF_PAR_IOS][NUM_OF_PINS] = {
    /* 0-7 */           /* 8-15 */          /* 16 - 23 */       /* 24 - 31 */
    {0,0,0,0,0,0,0,0,   0,0,0,0,0,0,0,1,    1,0,0,0,0,0,0,0,    0,0,0,0,0,1,1,0},
    {0,0,0,1,0,1,0,0,   0,0,0,0,1,1,0,0,    0,0,0,0,0,1,0,1,    0,0,0,0,0,0,1,1},
    {0,0,0,0,0,0,0,0,   0,0,0,0,0,0,0,0,    0,0,0,0,0,0,0,0,    0,0,0,1,1,1,0,0},
    {0,0,0,0,0,0,0,0,   0,0,0,0,1,1,0,0,    1,1,0,0,0,0,0,0,    0,0,1,1,0,0,0,0}
};

static u8 get_irq_num(u8 port, u8 pin)
{
    int     i, j;
    u8      num = 0;

    for (j=0;j<=port;j++)
        for (i=0;i<pin;i++)
            if(qe_irq_ports[j][i])
                num++;
    return num;
}

static par_io_t    *par_io = NULL;
static qe_par_io_t *qe_par_io = NULL;

int par_io_config_pin(u8  port, u8  pin, int dir, int open_drain,
			int assignment, int has_irq)
{
	u32 pinMask1bit, pinMask2bits, newMask2bits, tmp_val;

	if(!par_io) {
		par_io = (par_io_t *)ioremap(immrbar + 0x1400, sizeof(par_io_t));
		qe_par_io = (qe_par_io_t *)ioremap(immrbar + 0xC00,
				sizeof(qe_par_io_t));

		/* clear event bits in the event register of the QE ports */
		out_be32(&qe_par_io->cepier, 0xFFFFFFFF);
	}

	/* calculate pin location for single and 2 bits  information */
	pinMask1bit = (u32)(1 << (NUM_OF_PINS - (pin+1)));

	/* Set open drain, if required */
	tmp_val = in_be32(&par_io->io_regs[port].cpodr);
	if (open_drain)
		out_be32(&par_io->io_regs[port].cpodr, pinMask1bit | tmp_val);
	else
		out_be32(&par_io->io_regs[port].cpodr, ~pinMask1bit & tmp_val);

	/* define direction */
	tmp_val = (pin >  (NUM_OF_PINS/2) - 1) ?
		in_be32(&par_io->io_regs[port].cpdir2):
		in_be32(&par_io->io_regs[port].cpdir1);

	/* get all bits mask for 2 bit per port */
	pinMask2bits = (u32)(0x3 << (NUM_OF_PINS - (pin % (NUM_OF_PINS/2) +1)*2));

	/* Get the final mask we need for the right definition */
	newMask2bits = (u32)(dir << (NUM_OF_PINS - (pin % (NUM_OF_PINS/2)  +1)*2));

	/* clear and set 2 bits mask */
	if (pin >  (NUM_OF_PINS/2) - 1) {
		out_be32(&par_io->io_regs[port].cpdir2, ~pinMask2bits & tmp_val);
		tmp_val &= ~pinMask2bits;
		out_be32(&par_io->io_regs[port].cpdir2, newMask2bits | tmp_val);
	} else {
		out_be32(&par_io->io_regs[port].cpdir1, ~pinMask2bits & tmp_val);
		tmp_val &= ~pinMask2bits;
		out_be32(&par_io->io_regs[port].cpdir1, newMask2bits | tmp_val);
	}
	/* define pin assignment */
	tmp_val = (pin >  (NUM_OF_PINS/2) - 1) ?
				in_be32(&par_io->io_regs[port].cppar2):
				in_be32(&par_io->io_regs[port].cppar1);

	newMask2bits = (u32)(assignment << (NUM_OF_PINS - (pin % (NUM_OF_PINS/2)  +1)*2));
	/* clear and set 2 bits mask */
	if (pin >  (NUM_OF_PINS/2) - 1) {
		out_be32(&par_io->io_regs[port].cppar2, ~pinMask2bits & tmp_val);
		tmp_val &= ~pinMask2bits;
		out_be32(&par_io->io_regs[port].cppar2, newMask2bits | tmp_val);
	} else {
		out_be32(&par_io->io_regs[port].cppar1, ~pinMask2bits & tmp_val);
		tmp_val &= ~pinMask2bits;
		out_be32(&par_io->io_regs[port].cppar1, newMask2bits | tmp_val);
	}

	/* If this pin should not generate interrupt clear event mask bit */
	if(has_irq) {
		int i, j, k=0;
		int irq = get_irq_num(port, pin);
		u32 mask=0;

		for (j=0;j<NUM_OF_PAR_IOS;j++)
            for (i=0;i<NUM_OF_PINS;i++) {
                if(qe_irq_ports[j][i]) {
                    if (k == irq) {
                        mask = 0x80000000 >> k;
                        break;
                    }
                    k++;
                }
            }

        if (!mask)
	        return -EINVAL;

     	tmp_val = in_be32(&qe_par_io->cepimr);
	    out_be32(&qe_par_io->cepimr, ~mask & tmp_val);
    }

	return 0;
}
EXPORT_SYMBOL(par_io_config_pin);

int par_io_data_set(u8 port, u8 pin, u8 val)
{
	u32 pin_mask, tmp_val;

	if (port >= NUM_OF_PAR_IOS)
		return -EINVAL;
	if (pin >= NUM_OF_PINS)
		return -EINVAL;

	/* calculate pin location */
	pin_mask = (u32)(1 << (NUM_OF_PINS - 1 - pin));

	tmp_val = in_be32(&par_io->io_regs[port].cpdata);

	if (val == 0) /* clear  */
		out_be32(&par_io->io_regs[port].cpdata, ~pin_mask & tmp_val);
	else	/* set */
		out_be32(&par_io->io_regs[port].cpdata, pin_mask | tmp_val);
	return 0;
}
EXPORT_SYMBOL(par_io_data_set);

u8 par_io_data_get(u8 port, u8 pin)
{
	u32 pin_mask;

	if (port >= NUM_OF_PAR_IOS)
		return -EINVAL;
	if (pin >= NUM_OF_PINS)
		return -EINVAL;

	/*calculate pin location*/
	pin_mask = (u32)(1 << (NUM_OF_PINS - 1 - pin));

	return((u8)!!(in_be32(&par_io->io_regs[port].cpdata) & pin_mask));
}
EXPORT_SYMBOL(par_io_data_get);

static void dump_par_io(void)
{
	int i;

	printk(KERN_INFO "PAR IO registars:\n");
	printk(KERN_INFO "Base address: 0x%08x\n", (u32)par_io);
	for (i=0;i<NUM_OF_PAR_IOS;i++) {
		printk(KERN_INFO "cpodr[%d] : addr - 0x%08x, val - 0x%08x\n", i,
			(u32)&par_io->io_regs[i].cpodr,
			in_be32(&par_io->io_regs[i].cpodr));
		printk(KERN_INFO "cpdata[%d]: addr - 0x%08x, val - 0x%08x\n", i,
			(u32)&par_io->io_regs[i].cpdata,
			in_be32(&par_io->io_regs[i].cpdata));
		printk(KERN_INFO "cpdir1[%d]: addr - 0x%08x, val - 0x%08x\n", i,
			(u32)&par_io->io_regs[i].cpdir1,
			in_be32(&par_io->io_regs[i].cpdir1));
		printk(KERN_INFO "cpdir2[%d]: addr - 0x%08x, val - 0x%08x\n", i,
			(u32)&par_io->io_regs[i].cpdir2,
			in_be32(&par_io->io_regs[i].cpdir2));
		printk(KERN_INFO "cppar1[%d]: addr - 0x%08x, val - 0x%08x\n", i,
			(u32)&par_io->io_regs[i].cppar1,
			in_be32(&par_io->io_regs[i].cppar1));
		printk(KERN_INFO "cppar2[%d]: addr - 0x%08x, val - 0x%08x\n", i,
			(u32)&par_io->io_regs[i].cppar2,
			in_be32(&par_io->io_regs[i].cppar2));
	}

	printk(KERN_INFO "QE PAR IO registars:\n");
	printk(KERN_INFO "Base address: 0x%08x\n", (u32)qe_par_io);
	printk(KERN_INFO "cepier : addr - 0x%08x, val - 0x%08x\n",
		(u32)&qe_par_io->cepier, in_be32(&qe_par_io->cepier));
	printk(KERN_INFO "cepimr : addr - 0x%08x, val - 0x%08x\n",
		(u32)&qe_par_io->cepimr, in_be32(&qe_par_io->cepimr));
	printk(KERN_INFO "cepicr : addr - 0x%08x, val - 0x%08x\n",
		(u32)&qe_par_io->cepicr, in_be32(&qe_par_io->cepicr));
}

#ifdef CONFIG_PCI
int mpc83xx_map_irq(struct pci_dev *dev, unsigned char idsel, unsigned char pin)
{
	static char pci_irq_table[][4] =
	    /*
	     *      PCI IDSEL/INTPIN->INTLINE
	     *       A      B      C      D
	     */
	{
		{PIRQA, PIRQB, PIRQC, PIRQD},	/* idsel 0x11 */
		{PIRQC, PIRQD, PIRQA, PIRQB},	/* idsel 0x12 */
		{PIRQD, PIRQA, PIRQB, PIRQC}	/* idsel 0x13 */
	};

	const long min_idsel = 0x11, max_idsel = 0x13, irqs_per_slot = 4;
	return PCI_IRQ_TABLE_LOOKUP;
}

int mpc83xx_exclude_device(u_char bus, u_char devfn)
{
	if (bus == 0 && PCI_SLOT(devfn) == 0)
		return PCIBIOS_DEVICE_NOT_FOUND;
	return PCIBIOS_SUCCESSFUL;
}
#endif				/* CONFIG_PCI */

/* ************************************************************************
 *
 * Setup the architecture
 *
 */
static void __init
mpc832xe_mds_setup_arch(void)
{
	bd_t                            *binfo = (bd_t *) __res;
	unsigned int                    freq;
	struct ucc_geth_platform_data   *pdata;
	u8                              *bcsr_regs;

	/* get the core frequency */
	freq = binfo->bi_intfreq;

	/* Set loops_per_jiffy to a half-way reasonable value,
	   for use until calibrate_delay gets called. */
	loops_per_jiffy = freq / HZ;

#ifdef CONFIG_PCI
	/* setup PCI host bridges */
	mpc83xx_setup_hose();
#endif /* CONFIG_PCI */
	mpc83xx_early_serial_map();

#ifdef CONFIG_QE
	qe_reset();

	/* setup the board related information for the enet3 controllers */
	pdata = (struct ucc_geth_platform_data *) ppc_sys_get_pdata(MPC83xx_QE_UCC3);
	pdata->device_flags &= ~FSL_UGETH_DEV_HAS_GIGABIT;
	pdata->rx_clock      = QE_CLK9;
	pdata->tx_clock      = QE_CLK10;
	pdata->board_flags   = FSL_UGETH_BRD_HAS_PHY_INTR;
	pdata->phy_id        = 3;
	pdata->phy_interface = ENET_100_MII;
	pdata->phy_interrupt = MPC83xx_IRQ_EXT1;
	/* fixup phy address */
	pdata->phy_reg_addr += QE_MAP_ADDR;
	memcpy(pdata->mac_addr, binfo->bi_enetaddr, 6);

	/* setup the board related information for the enet4 controllers */
	pdata = (struct ucc_geth_platform_data *) ppc_sys_get_pdata(MPC83xx_QE_UCC4);
	pdata->device_flags &= ~FSL_UGETH_DEV_HAS_GIGABIT;
	pdata->rx_clock      = QE_CLK7;
	pdata->tx_clock      = QE_CLK8;
	pdata->board_flags   = FSL_UGETH_BRD_HAS_PHY_INTR;
	pdata->phy_id        = 4;
	pdata->phy_interface = ENET_100_MII;
	pdata->phy_interrupt = MPC83xx_IRQ_EXT2;
	/* fixup phy address */
	pdata->phy_reg_addr += QE_MAP_ADDR;
	memcpy(pdata->mac_addr, binfo->bi_enet1addr, 6);

	par_io_config_pin(3,  4, 3, 0, 2, 0); /* MDIO */
	par_io_config_pin(3,  5, 1, 0, 2, 0); /* MDC */

	/* UCC3 */
	par_io_config_pin(0, 13, 2, 0, 1, 0); /* RX_CLK (CLK9) */
	par_io_config_pin(3, 24, 2, 0, 1, 0); /* TX_CLK (CLK10) */

	par_io_config_pin(1,  0, 1, 0, 1, 0); /* TxD0 */
	par_io_config_pin(1,  1, 1, 0, 1, 0); /* TxD1 */
	par_io_config_pin(1,  2, 1, 0, 1, 0); /* TxD2 */
	par_io_config_pin(1,  3, 1, 0, 1, 0); /* TxD3 */
	par_io_config_pin(1,  4, 2, 0, 1, 0); /* RxD0 */
	par_io_config_pin(1,  5, 2, 0, 1, 0); /* RxD1 */
	par_io_config_pin(1,  6, 2, 0, 1, 0); /* RxD2 */
	par_io_config_pin(1,  7, 2, 0, 1, 0); /* RxD3 */
	par_io_config_pin(1,  8, 2, 0, 1, 0); /* RX_ER */
	par_io_config_pin(1,  9, 1, 0, 1, 0); /* TX_ER */
	par_io_config_pin(1, 10, 2, 0, 1, 0); /* RX_DV */
	par_io_config_pin(1, 11, 2, 0, 1, 0); /* COL */
	par_io_config_pin(1, 12, 1, 0, 1, 0); /* TX_EN */
	par_io_config_pin(1, 13, 2, 0, 1, 0); /* CRS */

	/* UCC4 */
	par_io_config_pin(3, 31, 2, 0, 1, 0); /* RX_CLK (CLK7) */
	par_io_config_pin(3,  6, 2, 0, 1, 0); /* TX_CLK (CLK8) */

	par_io_config_pin(1, 18, 1, 0, 1, 0); /* TxD0 */
	par_io_config_pin(1, 19, 1, 0, 1, 0); /* TxD1 */
	par_io_config_pin(1, 20, 1, 0, 1, 0); /* TxD2 */
	par_io_config_pin(1, 21, 1, 0, 1, 0); /* TxD3 */
	par_io_config_pin(1, 22, 2, 0, 1, 0); /* RxD0 */
	par_io_config_pin(1, 23, 2, 0, 1, 0); /* RxD1 */
	par_io_config_pin(1, 24, 2, 0, 1, 0); /* RxD2 */
	par_io_config_pin(1, 25, 2, 0, 1, 0); /* RxD3 */
	par_io_config_pin(1, 26, 2, 0, 1, 0); /* RX_ER */
	par_io_config_pin(1, 27, 1, 0, 1, 0); /* TX_ER */
	par_io_config_pin(1, 28, 2, 0, 1, 0); /* RX_DV */
	par_io_config_pin(1, 29, 2, 0, 1, 0); /* COL */
	par_io_config_pin(1, 30, 1, 0, 1, 0); /* TX_EN */
	par_io_config_pin(1, 31, 2, 0, 1, 0); /* CRS */

	/* Reset the Ethernet PHY */
	bcsr_regs = (u8 *)BCSR_VIRT_ADDR;
	bcsr_regs[8] &= ~0x50;
	udelay(1000);
	bcsr_regs[8] |=  0x50;
#endif /* CONFIG_QE */

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

#if defined(CONFIG_MTD_PHYSMAP) && defined(CONFIG_MTD_PARTITIONS)

/*
 * The firmware doesn't seem to provide the correct FLASH values,
 * hence these defines. They will most likely need to be updated for
 * later boards.
 */
#define MPC832XMDS_FLASH_ADDR	0xfe000000
#define MPC832XMDS_FLASH_SIZE	0x01000000
#define MPC832XMDS_HRCW_SIZE	0x00020000
#define MPC832XMDS_RAMDISK_SIZE	0x00400000
#define MPC832XMDS_KRNL_SIZE	0x00200000
#define MPC832XMDS_UBOOT_SIZE	0x00100000
#define MPC832XMDS_FS_SIZE	(MPC832XMDS_FLASH_SIZE - \
				 MPC832XMDS_HRCW_SIZE - \
				 MPC832XMDS_KRNL_SIZE - \
				 MPC832XMDS_RAMDISK_SIZE - \
				 MPC832XMDS_UBOOT_SIZE)

static struct mtd_partition ptbl[] = {
	{
	 .name = "Reset Configuration Word",
	 .size = MPC832XMDS_HRCW_SIZE,
	 .offset = 0,
	 .mask_flags = MTD_WRITEABLE,
	},
	{
	 .name = "User FS",
	 .size = MPC832XMDS_FS_SIZE,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = 0,
	},
	{
	 .name = "Ramdisk",
	 .size = MPC832XMDS_RAMDISK_SIZE,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = 0,
	 },
	{
	 .name = "Kernel",
	 .size = MPC832XMDS_KRNL_SIZE,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = 0,
	},
	{
	 .name = "ROM Monitor",
	 .size = MTDPART_SIZ_FULL,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = MTD_WRITEABLE,
	}
};

static int __init
mpc832x_setup_mtd(void)
{

	physmap_configure(MPC832XMDS_FLASH_ADDR, MPC832XMDS_FLASH_SIZE, 2,
			  NULL);

	physmap_set_partitions(ptbl, sizeof(ptbl)/sizeof(ptbl[0]));
	return 0;
}

arch_initcall(mpc832x_setup_mtd);
#endif

static void __init
mpc832xe_mds_map_io(void)
{
	/* we steal the lowest ioremap addr for virt space */
	io_block_mapping(VIRT_IMMRBAR, immrbar, MPC83xx_IMMRBAR_SIZE, _PAGE_IO);
	io_block_mapping(BCSR_VIRT_ADDR, BCSR_PHYS_ADDR, BCSR_SIZE, _PAGE_IO);
}

int
mpc832xe_mds_show_cpuinfo(struct seq_file *m)
{
	uint pvid, svid, phid1;
	bd_t *binfo = (bd_t *) __res;
	unsigned int freq;

	/* get the core frequency */
	freq = binfo->bi_intfreq;

	pvid = mfspr(PVR);
	svid = mfspr(SVR);

	seq_printf(m, "chip\t\t: MPC%s\n", cur_ppc_sys_spec->ppc_sys_name);
	seq_printf(m, "Vendor\t\t: Freescale Inc.\n");
	seq_printf(m, "Machine\t\t: mpc%s sys\n", cur_ppc_sys_spec->ppc_sys_name);
	seq_printf(m, "core clock\t: %d MHz\n"
			"bus  clock\t: %d MHz\n",
			(int)(binfo->bi_intfreq / 1000000),
			(int)(binfo->bi_busfreq / 1000000));
	seq_printf(m, "PVR\t\t: 0x%x\n", pvid);
	seq_printf(m, "SVR\t\t: 0x%x\n", svid);

	/* Display cpu Pll setting */
	phid1 = mfspr(HID1);
	seq_printf(m, "PLL setting\t: 0x%x\n", ((phid1 >> 24) & 0x3f));

	/* Display the amount of memory */
	seq_printf(m, "Memory\t\t: %d MB\n", (int)(binfo->bi_memsize / (1024 * 1024)));

	return 0;
}

void __init
mpc832xe_mds_init_IRQ(void)
{
	bd_t *binfo = (bd_t *) __res;

	u8 senses[8] = {
		0,			/* EXT 0 */
		IRQ_SENSE_LEVEL,	/* EXT 1 */
		IRQ_SENSE_LEVEL,	/* EXT 2 */
		0,			/* EXT 3 */
		IRQ_SENSE_LEVEL,	/* EXT 4 */
		IRQ_SENSE_LEVEL,	/* EXT 5 */
		IRQ_SENSE_LEVEL,	/* EXT 6 */
		IRQ_SENSE_LEVEL,	/* EXT 7 */
	};

	ipic_init(binfo->bi_immr_base + 0x00700, 0, MPC83xx_IPIC_IRQ_OFFSET, senses, 8);

	/* Initialize the default interrupt mapping priorities,
	 * in case the boot rom changed something on us.
	 */
	ipic_set_default_priority();

#ifdef CONFIG_QE
	qe_ic_init(QE_MAP_ADDR + 0x00000080,
			(QE_IC_LOW_SIGNAL | QE_IC_HIGH_SIGNAL), QE_IRQ_OFFSET);
#endif /* CONFIG_QE */
}

#if defined(CONFIG_I2C_MPC) && defined(CONFIG_SENSORS_DS1374)
extern ulong	ds1374_get_rtc_time(void);
extern int	ds1374_set_rtc_time(ulong);

static int __init
mpc832x_rtc_hookup(void)
{
	struct timespec	tv;

	ppc_md.get_rtc_time = ds1374_get_rtc_time;
	ppc_md.set_rtc_time = ds1374_set_rtc_time;

	tv.tv_nsec = 0;
	tv.tv_sec = (ppc_md.get_rtc_time)();
	do_settimeofday(&tv);

	return 0;
}
late_initcall(mpc832x_rtc_hookup);
#endif

static __inline__ void
mpc832xe_mds_set_bat(void)
{
	/* we steal the lowest ioremap addr for virt space */
	mb();
	mtspr(DBAT1U, VIRT_IMMRBAR | 0x3e);
	mtspr(DBAT1L, immrbar | 0x2a);
	mb();
}

void __init
platform_init(unsigned long r3, unsigned long r4, unsigned long r5,
	      unsigned long r6, unsigned long r7)
{
	bd_t *binfo = (bd_t *) __res;

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

#if defined(CONFIG_BLK_DEV_INITRD)
	/*
	 * If the init RAM disk has been configured in, and there's a valid
	 * starting address for it, set it up.
	 */
	if (r4) {
		initrd_start = r4 + KERNELBASE;
		initrd_end = r5 + KERNELBASE;
	}
#endif /* CONFIG_BLK_DEV_INITRD */

	/* Copy the kernel command line arguments to a safe place. */
	if (r6) {
		*(char *) (r7 + KERNELBASE) = 0;
		strcpy(cmd_line, (char *) (r6 + KERNELBASE));
	}

	immrbar = binfo->bi_immr_base;

	mpc832xe_mds_set_bat();

#if defined(CONFIG_SERIAL_8250) && defined(CONFIG_SERIAL_TEXT_DEBUG)
	{
		struct uart_port p;

		memset(&p, 0, sizeof (p));
		p.iotype = SERIAL_IO_MEM;
		p.membase = (unsigned char __iomem *)(VIRT_IMMRBAR + 0x4500);
		p.uartclk = binfo->bi_busfreq;

		gen550_init(0, &p);

		memset(&p, 0, sizeof (p));
		p.iotype = SERIAL_IO_MEM;
		p.membase = (unsigned char __iomem *)(VIRT_IMMRBAR + 0x4600);
		p.uartclk = binfo->bi_busfreq;

		gen550_init(1, &p);
	}
#endif

	identify_ppc_sys_by_id(mfspr(SVR));

	/* setup the PowerPC module struct */
	ppc_md.setup_arch = mpc832xe_mds_setup_arch;
	ppc_md.show_cpuinfo = mpc832xe_mds_show_cpuinfo;

	ppc_md.init_IRQ = mpc832xe_mds_init_IRQ;
	ppc_md.get_irq = ipic_get_irq;

	ppc_md.restart = mpc83xx_restart;
	ppc_md.power_off = mpc83xx_power_off;
	ppc_md.halt = mpc83xx_halt;

	ppc_md.find_end_of_memory = mpc83xx_find_end_of_memory;
	ppc_md.setup_io_mappings  = mpc832xe_mds_map_io;

	ppc_md.time_init = mpc83xx_time_init;
	ppc_md.set_rtc_time = NULL;
	ppc_md.get_rtc_time = NULL;
	ppc_md.calibrate_decr = mpc83xx_calibrate_decr;

#if defined(CONFIG_SERIAL_8250) && defined(CONFIG_SERIAL_TEXT_DEBUG)
	ppc_md.progress = gen550_progress;
#endif	/* CONFIG_SERIAL_8250 && CONFIG_SERIAL_TEXT_DEBUG */

	if (ppc_md.progress)
		ppc_md.progress("mpc832xe_mds_init(): exit", 0);

	return;
}
