/*
 * arch/ppc/platforms/83xx/mpc834x_sys.c
 *
 * MPC834x SYS board specific routines
 *
 * Maintainer: Kumar Gala <kumar.gala@freescale.com>
 *
 * Copyright 2005 Freescale Semiconductor Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * USB setup added by Randy Vinson <rvinson@mvista.com> based on code from
 * Hunter Wu.
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
#include <asm/ipic.h>
#include <asm/bootinfo.h>
#include <asm/pci-bridge.h>
#include <asm/mpc83xx.h>
#include <asm/irq.h>
#include <asm/kgdb.h>
#include <asm/ppc_sys.h>
#include <mm/mmu_decl.h>

#include <syslib/ppc83xx_setup.h>

#ifndef CONFIG_PCI
unsigned long isa_io_base = 0;
unsigned long isa_mem_base = 0;
#endif

extern unsigned long total_memory;	/* in mm/init */
extern void gen550_progress(char *, unsigned short);
extern void gen550_init(int, struct uart_port *);

unsigned char __res[sizeof (bd_t)];

#ifdef CONFIG_PCI
int
mpc83xx_map_irq(struct pci_dev *dev, unsigned char idsel, unsigned char pin)
{
	static char pci_irq_table[][4] =
	    /*
	     *      PCI IDSEL/INTPIN->INTLINE
	     *       A      B      C      D
	     */
	{
		{PIRQA, PIRQB,  PIRQC,  PIRQD}, /* idsel 0x11 */
		{PIRQC, PIRQD,  PIRQA,  PIRQB}, /* idsel 0x12 */
		{PIRQD, PIRQA,  PIRQB,  PIRQC}  /* idsel 0x13 */
	};

	const long min_idsel = 0x11, max_idsel = 0x13, irqs_per_slot = 4;
	return PCI_IRQ_TABLE_LOOKUP;
}

int
mpc83xx_exclude_device(u_char bus, u_char devfn)
{
	return PCIBIOS_SUCCESSFUL;
}
#endif /* CONFIG_PCI */

#ifdef CONFIG_834x_USB_SUPPORT
/*
 * Configure the on-chip USB controller. The MPC834xMDS only supports the
 * second USB interface (port 1). This code sets up the hardware and then
 * lets the platform driver take over device setup.
 */

void mpc834x_board_init(void)
{
	void __iomem *bcsr;
	unsigned char bcsr5;

	/*
	 * force to use the PHY on SYS board
	 * */
	bcsr = ioremap(BCSR_PHYS_ADDR, BCSR_SIZE);
	bcsr5 = in_8(bcsr + BCSR5_OFF);
	bcsr5 |= BCSR5_INT_USB;
	out_8(bcsr + BCSR5_OFF, bcsr5);
	iounmap(bcsr);
}

void mpc834x_usb_clk_cfg(void)
{
	unsigned long sccr;

	sccr = in_be32((void*)VIRT_IMMRBAR + MPC83XX_SCCR_OFFS);

	sccr |= MPC83XX_SCCR_USB_MPHCM_11 | MPC83XX_SCCR_USB_DRCM_11;

	out_be32((void*)VIRT_IMMRBAR + MPC83XX_SCCR_OFFS, sccr);
}

void mpc834x_usb_pin_cfg(struct fsl_usb2_platform_data *pdata)
{
	unsigned long sicrl;

	sicrl = in_be32((void*)VIRT_IMMRBAR + MPC83XX_SICRL_OFFS);

	/* set both ports to MPH mode */
	sicrl &= ~(MPC83XX_SICRL_USB0 | MPC83XX_SICRL_USB1);

	if (pdata->operating_mode == FSL_USB2_DR_HOST) {
		if (pdata->phy_mode == FSL_USB2_PHY_UTMI_WIDE) {
			/* UTMI WIDE combines both ports into a single 16-bit port */
			sicrl |= MPC83XX_SICRL_USB0 | MPC83XX_SICRL_USB1;
		}
		else {
			if (pdata->port_enables & FSL_USB2_PORT1_ENABLED)
				sicrl |= MPC83XX_SICRL_USB1;
		}
	}
	out_be32((void*)VIRT_IMMRBAR + MPC83XX_SICRL_OFFS, sicrl);
}

static void __init
mpc834x_usb_init(void)
{
	struct fsl_usb2_platform_data *pdata;

#ifdef CONFIG_834x_DR_USB_SUPPORT
	ppc_sys_device_remove(MPC83xx_USB2_MPH);
	pdata = (struct fsl_usb2_platform_data *) ppc_sys_get_pdata(MPC83xx_USB2_DR);

	if (pdata) {
		pdata->phy_mode = FSL_USB2_PHY_ULPI;
		pdata->operating_mode = FSL_USB2_DR_HOST;
		pdata->port_enables = FSL_USB2_PORT1_ENABLED;
	}

#elif defined(CONFIG_834x_MPH_USB_SUPPORT)
	ppc_sys_device_remove(MPC83xx_USB2_DR);
	pdata = (struct fsl_usb2_platform_data *) ppc_sys_get_pdata(MPC83xx_USB2_MPH);

	if (pdata) {
		pdata->phy_mode = FSL_USB2_PHY_ULPI;
		pdata->operating_mode = FSL_USB2_MPH_HOST;
		pdata->port_enables = FSL_USB2_PORT1_ENABLED;
	}

#endif
	mpc834x_board_init();
	mpc834x_usb_pin_cfg(pdata);
	mpc834x_usb_clk_cfg();
}
#endif /* CONFIG_834x_USB_SUPPORT */

/* ************************************************************************
 *
 * Setup the architecture
 *
 */
static void __init
mpc834x_sys_setup_arch(void)
{
	bd_t *binfo = (bd_t *) __res;
	unsigned int freq;
	struct gianfar_platform_data *pdata;
	struct gianfar_mdio_data *mdata;

	/* get the core frequency */
	freq = binfo->bi_intfreq;

	/* Set loops_per_jiffy to a half-way reasonable value,
	   for use until calibrate_delay gets called. */
	loops_per_jiffy = freq / HZ;

#ifdef CONFIG_PCI
	/* setup PCI host bridges */
	mpc83xx_setup_hose();
#endif
#ifdef CONFIG_SERIAL_8250
        mpc83xx_early_serial_map();
#endif

	/* setup the board related info for the MDIO bus */
	mdata = (struct gianfar_mdio_data *) ppc_sys_get_pdata(MPC83xx_MDIO);

	mdata->irq[0] = MPC83xx_IRQ_EXT1;
	mdata->irq[1] = MPC83xx_IRQ_EXT2;
	mdata->irq[2] = -1;
	mdata->irq[31] = -1;

	/* setup the board related information for the enet controllers */
	pdata = (struct gianfar_platform_data *) ppc_sys_get_pdata(MPC83xx_TSEC1);
	if (pdata) {
		pdata->board_flags = FSL_GIANFAR_BRD_HAS_PHY_INTR;
		pdata->bus_id = 0;
		pdata->phy_id = 0;
		memcpy(pdata->mac_addr, binfo->bi_enetaddr, 6);
	}

	pdata = (struct gianfar_platform_data *) ppc_sys_get_pdata(MPC83xx_TSEC2);
	if (pdata) {
		pdata->board_flags = FSL_GIANFAR_BRD_HAS_PHY_INTR;
		pdata->bus_id = 0;
		pdata->phy_id = 1;
		memcpy(pdata->mac_addr, binfo->bi_enet1addr, 6);
	}

#ifdef CONFIG_834x_USB_SUPPORT
	mpc834x_usb_init();
#else
	ppc_sys_device_remove(MPC83xx_USB2_MPH);
	ppc_sys_device_remove(MPC83xx_USB2_DR);
#endif

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
#define MPC8349MDS_FLASH_ADDR	0xfe000000
#define MPC8349MDS_FLASH_SIZE	0x00800000
#define MPC8349MDS_HRCW_SIZE	0x00020000
#define MPC8349MDS_KRNL_SIZE	0x00100000
#define MPC8349MDS_UBOOT_SIZE	0x00100000
#define MPC8349MDS_FS_SIZE	(MPC8349MDS_FLASH_SIZE - \
				 MPC8349MDS_HRCW_SIZE - \
				 MPC8349MDS_KRNL_SIZE - \
				 MPC8349MDS_UBOOT_SIZE)

static struct mtd_partition ptbl[] = {
	{
	 .name = "Reset Configuration Word",
	 .size = MPC8349MDS_HRCW_SIZE,
	 .offset = 0,
	 .mask_flags = MTD_WRITEABLE,
	},
	{
	 .name = "User FS",
	 .size = MPC8349MDS_FS_SIZE,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = 0,
	},
	{
	 .name = "Kernel",
	 .size = MPC8349MDS_KRNL_SIZE,
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
mpc834x_setup_mtd(void)
{

	physmap_configure(MPC8349MDS_FLASH_ADDR, MPC8349MDS_FLASH_SIZE, 2,
			  NULL);

	physmap_set_partitions(ptbl, sizeof(ptbl)/sizeof(ptbl[0]));
	return 0;
}

arch_initcall(mpc834x_setup_mtd);
#endif

static void __init
mpc834x_sys_map_io(void)
{
	/* we steal the lowest ioremap addr for virt space */
	io_block_mapping(VIRT_IMMRBAR, immrbar, 1024*1024, _PAGE_IO);
}

int
mpc834x_sys_show_cpuinfo(struct seq_file *m)
{
	uint pvid, svid, phid1;
	bd_t *binfo = (bd_t *) __res;
	unsigned int freq;

	/* get the core frequency */
	freq = binfo->bi_intfreq;

	pvid = mfspr(PVR);
	svid = mfspr(SVR);

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
mpc834x_sys_init_IRQ(void)
{
	bd_t *binfo = (bd_t *) __res;

	u8 senses[8] = {
		0,			/* EXT 0 */
		IRQ_SENSE_LEVEL,	/* EXT 1 */
		IRQ_SENSE_LEVEL,	/* EXT 2 */
		0,			/* EXT 3 */
#ifdef CONFIG_PCI
		IRQ_SENSE_LEVEL,	/* EXT 4 */
		IRQ_SENSE_LEVEL,	/* EXT 5 */
		IRQ_SENSE_LEVEL,	/* EXT 6 */
		IRQ_SENSE_LEVEL,	/* EXT 7 */
#else
		0,			/* EXT 4 */
		0,			/* EXT 5 */
		0,			/* EXT 6 */
		0,			/* EXT 7 */
#endif
	};

	ipic_init(binfo->bi_immr_base + 0x00700, 0, MPC83xx_IPIC_IRQ_OFFSET, senses, 8);

	/* Initialize the default interrupt mapping priorities,
	 * in case the boot rom changed something on us.
	 */
	ipic_set_default_priority();
}

#if defined(CONFIG_I2C_MPC) && defined(CONFIG_SENSORS_DS1374)
extern ulong	ds1374_get_rtc_time(void);
extern int	ds1374_set_rtc_time(ulong);

static int __init
mpc834x_rtc_hookup(void)
{
	struct timespec	tv;

	ppc_md.get_rtc_time = ds1374_get_rtc_time;
	ppc_md.set_rtc_time = ds1374_set_rtc_time;

	tv.tv_nsec = 0;
	tv.tv_sec = (ppc_md.get_rtc_time)();
	do_settimeofday(&tv);

	return 0;
}
late_initcall(mpc834x_rtc_hookup);
#endif
static __inline__ void
mpc834x_sys_set_bat(void)
{
	/* we steal the lowest ioremap addr for virt space */
	mb();
	mtspr(DBAT1U, VIRT_IMMRBAR | 0x1e);
	mtspr(DBAT1L, immrbar | 0x2a);
	mb();
}

void
mpc83xx_sys_restart(char *cmd)
{
	volatile unsigned char __iomem *reg;
	unsigned char tmp;

	reg = ioremap(BCSR_PHYS_ADDR, BCSR_SIZE);

	local_irq_disable();

	/*
	 * Unlock the BCSR bits so a PRST will update the contents.
	 * Otherwise the reset asserts but doesn't clear.
	 */
	tmp = in_8(reg + BCSR_MISC_REG3_OFF);
	tmp |= BCSR_MISC_REG3_CNFLOCK; /* low true, high false */
	out_8(reg + BCSR_MISC_REG3_OFF, tmp);

	/*
	 * Trigger a reset via a low->high transition of the
	 * PORESET bit.
	 */
	tmp = in_8(reg + BCSR_MISC_REG2_OFF);
	tmp &= ~BCSR_MISC_REG2_PORESET;
	out_8(reg + BCSR_MISC_REG2_OFF, tmp);

	udelay(1);

	tmp |= BCSR_MISC_REG2_PORESET;
	out_8(reg + BCSR_MISC_REG2_OFF, tmp);

	for(;;);
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

	mpc834x_sys_set_bat();

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
	ppc_md.setup_arch = mpc834x_sys_setup_arch;
	ppc_md.show_cpuinfo = mpc834x_sys_show_cpuinfo;

	ppc_md.init_IRQ = mpc834x_sys_init_IRQ;
	ppc_md.get_irq = ipic_get_irq;

	ppc_md.restart = mpc83xx_sys_restart;
	ppc_md.power_off = mpc83xx_power_off;
	ppc_md.halt = mpc83xx_halt;

	ppc_md.find_end_of_memory = mpc83xx_find_end_of_memory;
	ppc_md.setup_io_mappings  = mpc834x_sys_map_io;

	ppc_md.time_init = mpc83xx_time_init;
	ppc_md.set_rtc_time = NULL;
	ppc_md.get_rtc_time = NULL;
	ppc_md.calibrate_decr = mpc83xx_calibrate_decr;

#if defined(CONFIG_SERIAL_8250) && defined(CONFIG_SERIAL_TEXT_DEBUG)
	ppc_md.progress = gen550_progress;
#endif	/* CONFIG_SERIAL_8250 && CONFIG_SERIAL_TEXT_DEBUG */

	if (ppc_md.progress)
		ppc_md.progress("mpc834x_sys_init(): exit", 0);

	return;
}
