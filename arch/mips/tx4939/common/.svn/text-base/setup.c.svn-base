/*
 * arch/mips/tx4939/common/setup.c
 *
 * common tx4939 setup routines
 *
 * (C) Copyright TOSHIBA CORPORATION SEMICONDUCTOR COMPANY 2000-2001,2005
 *
 * Author: source@mvista.com
 *
 * 2001-2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Support for TX4939 in 2.6 - Hiroshi DOYU <Hiroshi_DOYU@montavista.co.jp>
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <asm/reboot.h>
#include <asm/io.h>
#include <asm/time.h>
#include <asm/delay.h>
#include <asm/types.h>
#include <asm/tx4939/tx4939.h>
#ifdef CONFIG_IDE
#include <linux/ide.h>
#endif
#if defined(CONFIG_SERIAL_TXX9) || defined(CONFIG_KGDB_TXX9)
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#endif

void (*__wbflush) (void);

extern void fputs(unsigned char *cp);
extern void rbtx4939_heartbeat(void);
extern void rbtx4939_machine_restart(char *command);
extern void __init tx4939_pcibios_init(void);
extern void __init tx4939_ide_setup(int ch);
extern void __init rbtx4939_setup(void);
extern void __init tx4939_rtc_init(void);
extern char *__init prom_getcmdline(void);
extern int early_serial_txx9_setup(struct uart_port *port);
extern int kgdb_serial_txx9_setup(struct uart_port *port, int num);

/* clocks */
unsigned int txx9_cpu_clock;
unsigned int txx9_gbus_clock;
unsigned int txx9_sys_clock;

/**
 * tx4939_write_buffer_flush - __wbflush function
 *
 * This function runs sync command to flush TX49 core write buffer.
 */

static void tx4939_write_buffer_flush(void)
{
	__asm__ __volatile__(".set	push\n\t"
			     ".set	noreorder\n\t"
			     ".set	mips2\n\t"
			     "sync\n\t"
			     ".set	pop"
			     :	/* no output */
			     :	/* no input */
			     :"memory");

	__asm__ __volatile__(".set	push\n\t"
			     ".set	noreorder\n\t"
			     "lw	$0,%0\n\t"
			     "nop\n\t"
			     ".set	pop"
			     :	/* no output */
			     :"m"(*(int *)KSEG1)
			     :"memory");
}

/**
 * tx4939_machine_halt - system halt
 *
 * This function calls asm_wait function instead of power-off.
 */

static void tx4939_machine_halt(void)
{
	printk(KERN_NOTICE "System Halted\n");
	local_irq_disable();
	while (1)
		asm_wait();
	/* no return */
}

/**
 * tx4939_machine_power_off - system power off
 *
 * This function calls _machine_halt function.
 */

static void tx4939_machine_power_off(void)
{
	tx4939_machine_halt();
	/* no return */
}

/**
 * tx4939_machine_restart - system reset
 * @command: unknown (not use)
 *
 * This function sets ISORST bit of TX4939.CCFG.CLTCTL
 */

static void tx4939_machine_restart(char *command)
{
	u64 q;
	local_irq_disable();
	printk(KERN_INFO "Rebooting...");
	q = reg_rd64s(&tx4939_ccfgptr->clkctr);
	reg_wr64s(&tx4939_ccfgptr->clkctr, q | TX4939_CLKCTR_IOSRST);
	wbflush();
#if defined(CONFIG_TOSHIBA_RBTX4939)
	rbtx4939_machine_restart(command);
#endif
	while (1) ;
}

/**
 * tx4939_check_clock - check clkctl register and set clock valuable
 *
 * This function reads TX4939.CCFG.clkctl register to set clock
 * variable.
 */

static void tx4939_check_clock(void)
{
	{
		unsigned int mulclk =
		    TX4939_CCFG_MULCLK_GET(reg_rd64s(&tx4939_ccfgptr->ccfg));
		unsigned int nd = (0x8 + ((mulclk + 1) & 0x7)) - 0x1;
		unsigned pll2 = (nd + 1) * 200 / 3;
		txx9_cpu_clock = pll2 / 2 * 1000000;
	}
	{
		unsigned int ydivmode =
		    TX4939_CCFG_YDIVMODE_GET(reg_rd64s
					     (&tx4939_ccfgptr->ccfg));
		unsigned int gbusdiv =
		    ((ydivmode & 0x3) + 2) + ((ydivmode & 0x2) >> 1);
		txx9_gbus_clock = txx9_cpu_clock / gbusdiv;
	}
	{
		unsigned int syssp =
		    TX4939_CCFG_SYSSP_GET(reg_rd64s(&tx4939_ccfgptr->ccfg));
		unsigned int sysdiv =
		    (2 + (((syssp + 1) & 0x1) << 1) +
		     (((syssp + 1) & 0x2) >> 1) + ((syssp + 1) & 0x4));
		txx9_sys_clock = txx9_gbus_clock / sysdiv;
	}
}

static void __init tx4939_ccfg_bootlog(void)
{
	printk(KERN_INFO "TX4939 -- CPU:%dMHz(GBUS:%dMHz,SYS:%dMHz) REVID:%08lx\n",
	       txx9_cpu_clock / 1000000,
	       txx9_gbus_clock / 1000000,
	       txx9_sys_clock / 1000000,
	       (unsigned long)reg_rd64s(&tx4939_ccfgptr->revid));
	printk(KERN_INFO "          CCFG:%016Lx PCFG:%016Lx\n",
	       reg_rd64s(&tx4939_ccfgptr->ccfg),
	       reg_rd64s(&tx4939_ccfgptr->pcfg));
}

/** tx4939_ccfg_setup - initialize CCFG
 *
 * This function sets ccfg and pcfg register. This function sets pin
 * multiplexing by pcfg
 */

static void __init tx4939_ccfg_setup(void)
{
	u64 q;

	/* clear WatchDogReset,BusErrorOnWrite flag (W1C) */
	q = reg_rd64s(&tx4939_ccfgptr->ccfg);
	reg_wr64s(&tx4939_ccfgptr->ccfg,
		    q | TX4939_CCFG_WDRST | TX4939_CCFG_BEOW);
	/* clear PCIC1 reset */
	q = reg_rd64s(&tx4939_ccfgptr->clkctr);
	if (q & TX4939_CLKCTR_PCI1RST)
		reg_wr64s(&tx4939_ccfgptr->clkctr, q & ~TX4939_CLKCTR_PCI1RST);

	/* CCFG setting */
	reg_wr64s(&tx4939_ccfgptr->ccfg, CONFIG_TOSHIBA_TX4939_CCFG);

	/* PCFG setting */
	reg_wr64s(&tx4939_ccfgptr->pcfg, CONFIG_TOSHIBA_TX4939_PCFG);

	/* MCLKCTL setting (baud clock enable) */
	q = reg_rd64s(&tx4939_ccfgptr->mclkctl);
	reg_wr64s(&tx4939_ccfgptr->mclkctl, q | TX4939_MCLKCTL_BDE);

        /* MCLKOSC setting */
        reg_wr64s(&tx4939_ccfgptr->mclkosc, 0x00030420);

	tx4939_ccfg_bootlog();
}

void tx4939_set_pcfg_rmii_speed(struct net_device *dev, int speed)
{
	u64 speed_bit, q;
	switch (dev->irq) {
	case TX4939_IRQ_ETHER(0):
		speed_bit = TX4939_PCFG_SPEED0_100MBPS;
		break;
	case TX4939_IRQ_ETHER(1):
		speed_bit = TX4939_PCFG_SPEED1_100MBPS;
		break;
	default:
		return;
	}

	q = reg_rd64s(&tx4939_ccfgptr->pcfg);
	if (speed)
		reg_wr64s(&tx4939_ccfgptr->pcfg, q | speed_bit);
	else
		reg_wr64s(&tx4939_ccfgptr->pcfg, q & ~speed_bit);
}

/**
 * tx4939_ebusc_setup - initialize EBUSC
 *
 */

static void __init tx4939_ebusc_setup(void)
{
	/* EBUSC */
}

/**
 * tx4939_ddrc_setup - initialize DDRC
 *
 */

static void __init tx4939_ddrc_setup(void)
{
	u16 s;
	/* DDRC */
	s = reg_rd16(&tx4939_ddrcptr->ddr[1].ctl);
	reg_wr16(&tx4939_ddrcptr->ddr[1].ctl, s | TX4939_DDR01_INIT_CLEAR);
}

/**
 * tx4939_sram_setup - initialize SRAM
 *
 */

static void __init tx4939_sram_setup(void)
{
	/* SRAM */
}

/**
 * tx4939_irc_setup - initialize IRC
 *
 * This function sets up TX4939 Interrupt Controller.
 */

static void __init tx4939_irc_setup(void)
{
	/* IRC */
	/* disable interrupt control */
	reg_wr32(&tx4939_ircptr->den, 0);
	/* clear irq trigger */
	reg_wr32(&tx4939_ircptr->dm0, 0x00000000);
	reg_wr32(&tx4939_ircptr->dm1, 0x00000000);
	reg_wr32(&tx4939_ircptr->dm2, 0x00000000);
	reg_wr32(&tx4939_ircptr->dm3, 0x00000000);
}

/**
 * tx4939_tmr_setup - initialize TMR
 *
 * This function sets up TX4939 Timer/Counter.
 * This function sets Timer/Counter to disable.
 */

static void __init tx4939_tmr_setup(void)
{
	unsigned int i;
	/* TMR */
	/* disable all timers */
	for (i = 0; i < TX4939_NR_TMR; i++) {
		reg_wr32(&tx4939_tmrptr(i)->tcr, TX4939_TMTCR_CRE);
		reg_wr32(&tx4939_tmrptr(i)->tisr, 0);
		reg_wr32(&tx4939_tmrptr(i)->cpra, 0xffffffff);
		reg_wr32(&tx4939_tmrptr(i)->itmr, 0);
		reg_wr32(&tx4939_tmrptr(i)->ccdr, 0);
		reg_wr32(&tx4939_tmrptr(i)->pgmr, 0);
	}
}

/**
 * tx4939_sio_setup - initialize SIO
 *
 * This function sets up Serial I/O.  This function sets their serial
 * control attributes. (uses SCLK, clock frequency, has cts line)
 */

static void __init tx4939_sio_setup(void)
{
	char *argptr;
	unsigned int i;
	int tx4939_sio_flag[4] =
	    { UPF_BUGGY_UART | UPF_MAGIC_MULTIPLIER,
	      UPF_MAGIC_MULTIPLIER,
	      UPF_MAGIC_MULTIPLIER,
	      UPF_MAGIC_MULTIPLIER };

	/* SIO h/w flow control off */
	for (i = 0; i < TX4939_NR_SIO; i++) {
		reg_wr32(&tx4939_sioptr(i)->flcr, 0);
	}

#if defined(CONFIG_SERIAL_TXX9) || defined(CONFIG_KGDB_TXX9)
	{
		int i;
		struct uart_port req;

		for (i = 0; i <= 3; i++) {
			memset(&req, 0, sizeof(req));
			req.line = i;
			req.iotype = UPIO_MEM;
			req.membase = (char *)TX4939_SIO_REG(i);
			req.mapbase = TX4939_SIO_REG(i);
			req.irq = TX4939_IRQ_SIO(i);
			req.flags = tx4939_sio_flag[i];
			req.uartclk = 14745600;
#ifdef CONFIG_SERIAL_TXX9
			early_serial_txx9_setup(&req);
#endif
#ifdef CONFIG_KGDB_TXX9
			kgdb_serial_txx9_setup(&req, i);
#endif
		}
	}
#endif /* defined(CONFIG_SERIAL_TXX9) || defined(CONFIG_KGDB_TXX9) */
#ifdef CONFIG_SERIAL_TXX9_CONSOLE
	argptr = prom_getcmdline();
	if (strstr(argptr, "console=") == NULL) {
		strcat(argptr, " console=ttyS0,38400");
	}
#endif
}

/**
 * tx4939_dmac_setup - initialize DMAC
 *
 *
 */

static void __init tx4939_dmac_setup(void)
{
	unsigned int i;
	/* enable DMA */
	for (i = 0; i < TX4939_NR_DMA; i++)
		reg_wr32(&tx4939_dmacptr(i)->mcr, TX4939_DMMCR_MSTEN);
}

#if defined(CONFIG_SPI_TX4939)
static int tx4939_spi_cs_func(int chipid, int on)
{
	/* dummy function */
	static status;
	status = chipid & on;
}

/**
 * tx4939_spi_setup - initialize SPI
 *
 *
 */

static void __init tx4939_spi_setup(void)
{
	u32 l;

	l = reg_rd32(&tx4939_ccfgptr->pcfg);
	reg_wr32(&tx4939_ccfgptr->pcfg, l & TX4939_PCFG_SPIMODE_MASK);
	l = reg_rd32(&tx4939_ccfgptr->pcfg);
	reg_wr32(&tx4939_ccfgptr->pcfg, l | TX4939_PCFG_SPIMODE_SPI);

	txx9_spi_setup(TX4939_SPIC_REG, tx4939_spi_cs_func);
}
#endif				/* CONFIG_SPI_TX4939 */

/**
 * tx4939_request_resource - request resource for TX49 internal
 * registers
 *
 *
 */

static void __init tx4939_request_resource(void)
{
	/* TX4939 internal registers */
	static struct resource tx4939_reg_resource = {
		.name = "TX4939 internal registers",
		.start = TX4939_REG_BASE,
		.end = TX4939_REG_BASE + TX4939_REG_SIZE,
		.flags = IORESOURCE_MEM,
	};
	if (request_resource(&iomem_resource, &tx4939_reg_resource))
		printk(KERN_ERR "request resource for internal registers failed\n");
}

/**
 * tx4939_time_init - time setup routine
 *
 * This function runs TX4939 time setup routine.
 *
 */

static void __init tx4939_time_init(void)
{
	tx4939_rtc_init();
	mips_hpt_frequency = txx9_cpu_clock / 2;
}

#if defined(CONFIG_HEARTBEAT)
void (*org_timer_handler) (int irq, void *dev_id, struct pt_regs * regs);

static void tx4939_timer_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	org_timer_handler(irq, dev_id, regs);
#if defined(CONFIG_TOSHIBA_RBTX4939)
	{
		static int i = 0;
		if (i < 0) {
			rbtx4939_heartbeat();
			i = 50;
		}
		i--;
	}
#endif
}
#endif

/**
 * tx4939_timer_setup - generate the first timer interrupt
 * @irq: The irqaction descripter for tick timer interrupt
 *
 * This function set to generate the first timer interrupt.
 *
 * This reads CP0.count register. And calculates a timing for first
 * timer interrupt. And writes it to CP0.compare register.
 */

static void __init tx4939_timer_setup(struct irqaction *irq)
{
	u32 count;
	u32 c1;
	u32 c2;
#if defined(CONFIG_HEARTBEAT)
	org_timer_handler = irq->handler;
	irq->handler = tx4939_timer_handler;
#endif

	if (reg_rd64s(&tx4939_ccfgptr->ccfg) & TX4939_CCFG_TINTDIS) {	/* TINTDIS is off */
		fputs
		    ("***** Please set TINTDIS to be able to TX49 core timer interrupt *****\r\n");
		while (1) ;
	}

	setup_irq(TX4939_IRQ_CPU_TIMER, irq);

	/* to generate the first timer interrupt */
	c1 = read_c0_count();
	count = c1 + (mips_hpt_frequency / HZ);
	write_c0_compare(count);
	c2 = read_c0_count();
}

/**
 * tx4939_setup - common setup function
 *
 * This function runs setup routine related to TX4939/RBTX4939.
 * This sets some function pointers, board_time_init, board_timer_setup,
 * __wbflush.
 *
 */

static int __init tx4939_setup(void)
{
	board_time_init = tx4939_time_init;
	board_timer_setup = tx4939_timer_setup;
	__wbflush = tx4939_write_buffer_flush;

	ioport_resource.start = 0x1000;
	ioport_resource.end = 0xffffffff;
	iomem_resource.start = 0x1000;
	iomem_resource.end = 0xffffffff;	/* expand to 4GB */

	tx4939_check_clock();

	/* change default value to udelay/mdelay take reasonable time */
	loops_per_jiffy = txx9_cpu_clock / HZ / 2;

	tx4939_ccfg_setup();
	tx4939_ebusc_setup();
	tx4939_ddrc_setup();
	tx4939_sram_setup();
	tx4939_irc_setup();
	tx4939_tmr_setup();

#if defined(CONFIG_SERIAL_TXX9)
	tx4939_sio_setup();
#endif
	tx4939_dmac_setup();

#if defined(CONFIG_TOSHIBA_RBTX4939)
	rbtx4939_setup();
#endif

#if defined(CONFIG_SPI_TX4939)
	tx4939_spi_setup();
#endif

#if defined(CONFIG_BLK_DEV_IDE_TX4939)
	tx4939_ide_setup(0);
#ifdef CONFIG_TOSHIBA_TX4939_MPLEX_ATA1
	tx4939_ide_setup(1);
#endif
#endif
	tx4939_request_resource();
	_machine_restart = tx4939_machine_restart;
	_machine_halt = tx4939_machine_halt;
	_machine_power_off = tx4939_machine_power_off;

	return 0;
}

early_initcall(tx4939_setup);

EXPORT_SYMBOL(__wbflush);
EXPORT_SYMBOL(tx4939_set_pcfg_rmii_speed);
