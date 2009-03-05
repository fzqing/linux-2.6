/*
 * arch/ppc/platforms/83xx/mpc834x_itx.c
 *
 * MPC834x ITX board specific routines
 *
 * Maintainer: Kumar Gala <kumar.gala@freescale.com>
 *
 * Copyright 2005 Freescale Semiconductor Inc.
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
#include <linux/module.h>
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
#include <linux/tty.h>		/* for linux/serial_core.h */
#include <linux/serial_core.h>
#include <linux/mtd/physmap.h>
#include <linux/initrd.h>
#include <linux/module.h>
#include <linux/fsl_devices.h>
#include <linux/kgdb.h>

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
#include <linux/ide.h>

#include <syslib/ppc83xx_setup.h>

#ifndef CONFIG_PCI
unsigned long isa_io_base;
unsigned long isa_mem_base;
#endif

extern unsigned long total_memory;	/* in mm/init */
extern void gen550_progress(char *, unsigned short);
extern void gen550_init(int, struct uart_port *);

unsigned char __res[sizeof(bd_t)];

#ifdef CONFIG_PCI
int mpc83xx_map_irq(struct pci_dev *dev, unsigned char idsel, unsigned char pin)
{
	static char pci_irq_table[][1] =
	    /*
	     *      PCI IDSEL/INTPIN->INTLINE
	     *       A      B      C      D
	     */
	{
		{PIRQB},	/* idsel 0x0e */
		{PIRQC},	/* idsel 0x0f */
		{PIRQA},	/* idsel 0x10 */
	};

	const long min_idsel = 0x0e, max_idsel = 0x10, irqs_per_slot = 1;
	if ((idsel < min_idsel) || (idsel > max_idsel) || (!dev) || (pin > 1))
		return -EINVAL;
	return PCI_IRQ_TABLE_LOOKUP;
}

int mpc83xx_exclude_device(u_char bus, u_char devfn)
{
	return PCIBIOS_SUCCESSFUL;
}
#endif				/* CONFIG_PCI */

#ifdef CONFIG_834x_USB_SUPPORT
/*
 * Configure the on-chip USB controller. The MPC834xMDS only supports the
 * second USB interface (port 1). This code sets up the hardware and then
 * lets the platform driver take over device setup.
 */

void mpc834x_board_init(void)
{
}

void mpc834x_usb_clk_cfg(void)
{
	unsigned long sccr;

	sccr = in_be32((void *)VIRT_IMMRBAR + MPC83XX_SCCR_OFFS);

	sccr |= MPC83XX_SCCR_USB_MPHCM_11 | MPC83XX_SCCR_USB_DRCM_11;

	out_be32((void *)VIRT_IMMRBAR + MPC83XX_SCCR_OFFS, sccr);
}

void mpc834x_usb_pin_cfg(struct fsl_usb2_platform_data *pdata)
{
	unsigned long sicrl;

	sicrl = in_be32((void *)VIRT_IMMRBAR + MPC83XX_SICRL_OFFS);

	/* set both ports to MPH mode */
	sicrl &= ~(MPC83XX_SICRL_USB0 | MPC83XX_SICRL_USB1);

	if (pdata->operating_mode == FSL_USB2_DR_HOST) {
		if (pdata->phy_mode == FSL_USB2_PHY_UTMI_WIDE) {
			/* UTMI WIDE combines both ports into a single 16-bit port */
			sicrl |= MPC83XX_SICRL_USB0 | MPC83XX_SICRL_USB1;
		} else {
			if (pdata->port_enables & FSL_USB2_PORT1_ENABLED)
				sicrl |= MPC83XX_SICRL_USB1;
		}
	}
	out_be32((void *)VIRT_IMMRBAR + MPC83XX_SICRL_OFFS, sicrl);
}

static void __init mpc834x_usb_init(void)
{
	struct fsl_usb2_platform_data *pdata;

#ifdef CONFIG_834x_DR_USB_SUPPORT
	ppc_sys_device_remove(MPC83xx_USB2_MPH);
	pdata =
	    (struct fsl_usb2_platform_data *)ppc_sys_get_pdata(MPC83xx_USB2_DR);

	if (pdata) {
		pdata->phy_mode = FSL_USB2_PHY_ULPI;
		pdata->operating_mode = FSL_USB2_DR_HOST;
		pdata->port_enables = FSL_USB2_PORT1_ENABLED;
	}
#elif defined(CONFIG_834x_MPH_USB_SUPPORT)
	ppc_sys_device_remove(MPC83xx_USB2_DR);
	pdata =
	    (struct fsl_usb2_platform_data *)
	    ppc_sys_get_pdata(MPC83xx_USB2_MPH);

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
#endif				/* CONFIG_834x_USB_SUPPORT */

static void cfide_outsw(unsigned long port, void *addr, u32 count)
{
	_outsw_ns((void __iomem *)port, addr, count);
}

static void cfide_insw(unsigned long port, void *addr, u32 count)
{
	_insw_ns((void __iomem *)port, addr, count);
}

void cfide_platform_mmiops (ide_hwif_t *hwif)
{
	default_hwif_mmiops(hwif);
	hwif->OUTL = NULL;
	hwif->OUTSW = cfide_outsw;
	hwif->OUTSL = NULL;
	hwif->INL = NULL;
	hwif->INSW = cfide_insw;
	hwif->INSL = NULL;
}

EXPORT_SYMBOL(cfide_platform_mmiops);

void cfide_selectproc (ide_drive_t *drive)
{
	u8 stat;

	stat = drive->hwif->INB(IDE_STATUS_REG);
	if ((stat & READY_STAT) && (stat & BUSY_STAT))
		drive->present = 0;
	else
		drive->present = 1;
}

EXPORT_SYMBOL(cfide_selectproc);

/* ************************************************************************
 *
 * Setup the architecture
 *
 */
static void __init mpc834x_itx_setup_arch(void)
{
	bd_t *binfo = (bd_t *) __res;
	unsigned int freq;
	struct gianfar_platform_data *pdata;

	/* get the core frequency */
	freq = binfo->bi_intfreq;

	/* Set loops_per_jiffy to a half-way reasonable value,
	   for use until calibrate_delay gets called. */
	loops_per_jiffy = freq / HZ;

#ifdef CONFIG_PCI
	/* setup PCI host bridges */
	mpc83xx_setup_hose();
#endif
	mpc83xx_early_serial_map();

	/* setup the board related information for the enet controllers */
	pdata =
	    (struct gianfar_platform_data *)ppc_sys_get_pdata(MPC83xx_TSEC1);
	if (pdata) {
		pdata->board_flags = FSL_GIANFAR_BRD_HAS_PHY_INTR;
		pdata->bus_id = 1;
		pdata->phy_id = 0x1c;
		/* fixup phy address */
		memcpy(pdata->mac_addr, binfo->bi_enetaddr, 6);
	}

	pdata =
	    (struct gianfar_platform_data *)ppc_sys_get_pdata(MPC83xx_TSEC2);
	if (pdata) {
		pdata->board_flags =
		    FSL_GIANFAR_BRD_HAS_PHY_INTR | FSL_GIANFAR_BRD_PHY_ANEG;
		pdata->bus_id = 1;
		pdata->phy_id = 0x1f;
		/* fixup phy address */
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
#if (CONFIG_MTD_PHYSMAP_START!=0)
#define MPC8349MDS_FLASH_ADDR CONFIG_MTD_PHYSMAP_START
#else
#define MPC8349MDS_FLASH_ADDR 0xFE000000
#endif
#if (CONFIG_MTD_PHYSMAP_LEN!=0)
#define MPC8349MDS_FLASH_SIZE CONFIG_MTD_PHYSMAP_LEN
#else
#define MPC8349MDS_FLASH_SIZE 0x00800000
#endif
#define MPC8349MDS_TP_SIZE	0x00200000
#define MPC8349MDS_UBOOT_SIZE	0x00400000
#define MPC8349MDS_FS_SIZE	(MPC8349MDS_FLASH_SIZE - \
				 MPC8349MDS_TP_SIZE - \
				 MPC8349MDS_UBOOT_SIZE)

static struct mtd_partition ptbl[] = {

	{
	 .name = "ROM Monitor",
	 .size = MPC8349MDS_UBOOT_SIZE,
	 .offset = 0,
	 .mask_flags = MTD_CAP_ROM,
	 },
	{
	 .name = "User FS",
	 .size = MPC8349MDS_FS_SIZE,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = MTD_ERASEABLE,
	 },
	{
	 .name = "Third_party",
	 .size = MPC8349MDS_TP_SIZE,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = MTD_CAP_ROM,
	 },
};

static int __init mpc834x_setup_mtd(void)
{
	physmap_configure(MPC8349MDS_FLASH_ADDR, MPC8349MDS_FLASH_SIZE, 2,
			  NULL);
	physmap_set_partitions(ptbl, sizeof(ptbl) / sizeof(ptbl[0]));
	return 0;
}

arch_initcall(mpc834x_setup_mtd);
#endif

static void __init mpc834x_sys_map_io(void)
{
	/* we steal the lowest ioremap addr for virt space */
	io_block_mapping(VIRT_IMMRBAR, immrbar, 1024 * 1024, _PAGE_IO);
}

int mpc834x_itx_show_cpuinfo(struct seq_file *m)
{
	uint pvid, svid, phid1;
	bd_t *binfo = (bd_t *) __res;
	unsigned int freq;

	/* get the core frequency */
	freq = binfo->bi_intfreq;

	pvid = mfspr(PVR);
	svid = mfspr(SVR);

	seq_printf(m, "Vendor\t\t: Freescale Inc.\n");
	seq_printf(m, "Machine\t\t: mpc%s itx\n",
		   cur_ppc_sys_spec->ppc_sys_name);
	seq_printf(m, "core clock\t: %d MHz\n" "bus  clock\t: %d MHz\n",
		   (int)(binfo->bi_intfreq / 1000000),
		   (int)(binfo->bi_busfreq / 1000000));
	seq_printf(m, "PVR\t\t: 0x%x\n", pvid);
	seq_printf(m, "SVR\t\t: 0x%x\n", svid);

	/* Display cpu Pll setting */
	phid1 = mfspr(HID1);
	seq_printf(m, "PLL setting\t: 0x%x\n", ((phid1 >> 24) & 0x3f));

	/* Display the amount of memory */
	seq_printf(m, "Memory\t\t: %d MB\n",
		   (int)(binfo->bi_memsize / (1024 * 1024)));

	return 0;
}

void __init mpc834x_itx_init_IRQ(void)
{
	bd_t *binfo = (bd_t *) __res;

	u8 senses[8] = {
		0,		/* EXT 0 */
		IRQ_SENSE_LEVEL,	/* EXT 1 */
		IRQ_SENSE_LEVEL,	/* EXT 2 */
		0,		/* EXT 3 */
#ifdef CONFIG_PCI
		IRQ_SENSE_LEVEL,	/* EXT 4 */
		IRQ_SENSE_LEVEL,	/* EXT 5 */
		IRQ_SENSE_LEVEL,	/* EXT 6 */
		IRQ_SENSE_LEVEL,	/* EXT 7 */
#else
		0,		/* EXT 4 */
		0,		/* EXT 5 */
		0,		/* EXT 6 */
		0,		/* EXT 7 */
#endif
	};

	ipic_init(binfo->bi_immr_base + 0x00700, 0, MPC83xx_IPIC_IRQ_OFFSET,
		  senses, 8);

	/* Initialize the default interrupt mapping priorities,
	 * in case the boot rom changed something on us.
	 */
	ipic_set_default_priority();
}

#if defined(CONFIG_I2C_MPC) && defined(CONFIG_SENSORS_DS1374)
extern ulong ds1374_get_rtc_time(void);
extern int ds1374_set_rtc_time(ulong);

static int __init mpc834x_rtc_hookup(void)
{
	struct timespec tv;

	ppc_md.get_rtc_time = ds1374_get_rtc_time;
	ppc_md.set_rtc_time = ds1374_set_rtc_time;

	tv.tv_nsec = 0;
	tv.tv_sec = (ppc_md.get_rtc_time) ();
	do_settimeofday(&tv);

	return 0;
}

late_initcall(mpc834x_rtc_hookup);
#endif

#if defined(CONFIG_I2C_MPC) && defined(CONFIG_SENSORS_DS1337)
#define DS1337_GET_DATE		0
#define DS1337_SET_DATE		1

extern int ds1337_do_command(int id, int cmd, void *arg);

static unsigned long ds1337_get_rtc_time(void)
{
	struct rtc_time date;

	if(ds1337_do_command(1, DS1337_GET_DATE, &date) == -ENODEV)
		return mktime(2000, 1, 1, 0, 0, 0);

	return mktime(date.tm_year + 1900, date.tm_mon + 1, date.tm_mday, date.tm_hour, date.tm_min, date.tm_sec);
}

static int ds1337_set_rtc_time(unsigned long t)
{
	struct rtc_time tm;

	to_tm(t, &tm);
	tm.tm_year -= 1900;
	tm.tm_mon -= 1;

	if(ds1337_do_command(1, DS1337_SET_DATE, &tm) == -ENODEV)
		return -ENODEV;

	return 0;
}

static int __init mpc834x_rtc_hookup(void)
{
	struct timespec tv;

	ppc_md.get_rtc_time = ds1337_get_rtc_time;
	ppc_md.set_rtc_time = ds1337_set_rtc_time;

	tv.tv_nsec = 0;
	tv.tv_sec = (ppc_md.get_rtc_time) ();

	return 0;
}

late_initcall(mpc834x_rtc_hookup);
#endif

static __inline__ void mpc834x_itx_set_bat(void)
{
	/* we steal the lowest ioremap addr for virt space */
	mb();
	mtspr(DBAT1U, VIRT_IMMRBAR | 0x1e);
	mtspr(DBAT1L, immrbar | 0x2a);
	mb();
}

void mpc83xx_itx_restart(char *cmd)
{
#define RST_OFFSET      0x00000900
#define RST_PROT_REG    0x00000018
#define RST_CTRL_REG    0x0000001c
	__be32 __iomem *reg;

	/* map reset register space */
	reg = ioremap(immrbar + RST_OFFSET, 0xff);

	local_irq_disable();

	/* enable software reset "RSTE" */
	out_be32(reg + (RST_PROT_REG >> 2), 0x52535445);

	/* set software hard reset */
	out_be32(reg + (RST_CTRL_REG >> 2), 0x2);
	for (;;) ;
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
		memcpy((void *)__res, (void *)(r3 + KERNELBASE), sizeof(bd_t));
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
#endif				/* CONFIG_BLK_DEV_INITRD */

	/* Copy the kernel command line arguments to a safe place. */
	if (r6) {
		*(char *)(r7 + KERNELBASE) = 0;
		strcpy(cmd_line, (char *)(r6 + KERNELBASE));
	}

	immrbar = binfo->bi_immr_base;

	mpc834x_itx_set_bat();

#if defined(CONFIG_SERIAL_8250) && defined(CONFIG_SERIAL_TEXT_DEBUG)
	{
		struct uart_port p;

		memset(&p, 0, sizeof(p));
		p.iotype = SERIAL_IO_MEM;
		p.membase = (unsigned char __iomem *)(VIRT_IMMRBAR + 0x4500);
		p.uartclk = binfo->bi_busfreq;

		gen550_init(0, &p);
#if (defined CONFIG_KGDB_8250) && (defined CONFIG_KGDB_TTYS0)
		kgdb8250_add_port(0, &p);
#endif
		memset(&p, 0, sizeof(p));
		p.iotype = SERIAL_IO_MEM;
		p.membase = (unsigned char __iomem *)(VIRT_IMMRBAR + 0x4600);
		p.uartclk = binfo->bi_busfreq;

		gen550_init(1, &p);
#if (defined CONFIG_KGDB_8250) && (defined CONFIG_KGDB_TTYS1)
		kgdb8250_add_port(1, &p);
#endif
	}
#endif

	identify_ppc_sys_by_name_and_id("8349E_ITX", mfspr(SVR));

	/* setup the PowerPC module struct */
	ppc_md.setup_arch = mpc834x_itx_setup_arch;
	ppc_md.show_cpuinfo = mpc834x_itx_show_cpuinfo;

	ppc_md.init_IRQ = mpc834x_itx_init_IRQ;
	ppc_md.get_irq = ipic_get_irq;

	ppc_md.restart = mpc83xx_itx_restart;
	ppc_md.power_off = mpc83xx_power_off;
	ppc_md.halt = mpc83xx_halt;

	ppc_md.find_end_of_memory = mpc83xx_find_end_of_memory;
	ppc_md.setup_io_mappings = mpc834x_sys_map_io;

	ppc_md.time_init = mpc83xx_time_init;
	ppc_md.set_rtc_time = NULL;
	ppc_md.get_rtc_time = NULL;
	ppc_md.calibrate_decr = mpc83xx_calibrate_decr;

#if defined(CONFIG_SERIAL_8250) && defined(CONFIG_SERIAL_TEXT_DEBUG)
	ppc_md.progress = gen550_progress;
#endif				/* CONFIG_SERIAL_8250 && CONFIG_SERIAL_TEXT_DEBUG */

	if (ppc_md.progress)
		ppc_md.progress("mpc834x_itx_init(): exit", 0);

	return;
}

#ifdef CONFIG_RTC_CLASS
late_initcall(rtc_class_hookup);
#endif
