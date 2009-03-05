/*
 * linux/arch/sh/boards/se/7780/setup.c
 *
 * Copyright (C) 2005  Takashi Kusuda
 *
 * Hitachi SolutionEngine 7780 Support.
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/mtd/physmap.h>

#include <linux/hdreg.h>
#include <linux/ide.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/mach/map.h>
#include <asm/cpu/pci-sh7780.h>

#ifdef CONFIG_SH_KGDB
#include <asm/kgdb.h>
#endif

#ifdef CONFIG_MTD_PARTITIONS
static struct mtd_partition sh_se_partitions[] = {
	{
	 .name = "bootloader",
	 .size = 0x20000,	/* 64KB *//* x16 */
	 .offset = 0,
	 },
	{
	 .name = "kernel",
	 .size = 0x120000,	/* 1088KB *//* x16 */
	 .offset = MTDPART_OFS_APPEND,
	 },
	{
	 .name = "userland",
	 .size = MTDPART_SIZ_FULL,
	 .offset = MTDPART_OFS_APPEND,
	 }
};
#endif

const char *get_system_type(void)
{
	return "7780 SolutionEngine";
}

#ifdef CONFIG_SH_KGDB
static int kgdb_uart_setup(void);
static struct kgdb_sermap kgdb_uart_sermap =
{ "ttyS", 0, kgdb_uart_setup, NULL };
#endif

/*
 * Initialize the board
 */
void __init platform_setup(void)
{
	/* "SH-Linux" on LED Display */
	*(volatile unsigned short *)(PA_LED_DISP + (DISP_SEL0_ADDR << 1)) = 'S';
	*(volatile unsigned short *)(PA_LED_DISP + (DISP_SEL1_ADDR << 1)) = 'H';
	*(volatile unsigned short *)(PA_LED_DISP + (DISP_SEL2_ADDR << 1)) = '-';
	*(volatile unsigned short *)(PA_LED_DISP + (DISP_SEL3_ADDR << 1)) = 'L';
	*(volatile unsigned short *)(PA_LED_DISP + (DISP_SEL4_ADDR << 1)) = 'i';
	*(volatile unsigned short *)(PA_LED_DISP + (DISP_SEL5_ADDR << 1)) = 'n';
	*(volatile unsigned short *)(PA_LED_DISP + (DISP_SEL6_ADDR << 1)) = 'u';
	*(volatile unsigned short *)(PA_LED_DISP + (DISP_SEL7_ADDR << 1)) = 'x';

	/*
	 * PCI REQ/GNT setting
	 *   REQ0/GNT0 -> USB
	 *   REQ1/GNT1 -> PC Card
	 *   REQ2/GNT2 -> Serial ATA
	 *   REQ3/GNT3 -> PCI slot
	 */
	ctrl_outw(0x0213, FPGA_REQSEL); /* test */

	/* GPIO setting */
	ctrl_outw(0x0000, GPIO_PECR);
	ctrl_outw(ctrl_inw(GPIO_PHCR)&0xfff3, GPIO_PHCR);
	ctrl_outw(0x0c00, GPIO_PMSELR);

	/* iVDR Power ON */
	ctrl_outw(0x0001, FPGA_IVDRPW);

#ifdef CONFIG_SH_KGDB
	kgdb_register_sermap(&kgdb_uart_sermap);
#endif

#ifdef CONFIG_MTD_PARTITIONS
	physmap_set_partitions(sh_se_partitions,
		sizeof(sh_se_partitions)/sizeof(sh_se_partitions[0]));
#endif
}

/*********************************************************************
 * Currently a hack (e.g. does not interact well w/serial.c, lots of *
 * hardcoded stuff) but may be useful if SCI/F needs debugging.      *
 * Mostly copied from x86 code (see files asm-i386/kgdb_local.h and  *
 * arch/i386/lib/kgdb_serial.c).                                     *
 *********************************************************************/

#ifdef CONFIG_SH_KGDB
#include <linux/types.h>
#include <linux/serial.h>
#include <linux/serialP.h>
#include <linux/serial_reg.h>

#define COM1_PORT 0x3f8  /* Base I/O address */
#define COM1_IRQ  4      /* IRQ not used yet */
#define COM2_PORT 0x2f8  /* Base I/O address */
#define COM2_IRQ  3      /* IRQ not used yet */

#define SB_CLOCK 1843200 /* Serial baud clock */
#define SB_BASE (SB_CLOCK/16)
#define SB_MCR UART_MCR_OUT2 | UART_MCR_DTR | UART_MCR_RTS

struct uart_port {
	int base;
};
#define UART_NPORTS 2
struct uart_port uart_ports[] = {
	{ COM1_PORT },
	{ COM2_PORT },
};
struct uart_port *kgdb_uart_port;

#define UART_IN(reg)	inb_p(kgdb_uart_port->base + reg)
#define UART_OUT(reg,v)	outb_p((v), kgdb_uart_port->base + reg)

/* Basic read/write functions for the UART */
#define UART_LSR_RXCERR    (UART_LSR_BI | UART_LSR_FE | UART_LSR_PE)
static int kgdb_uart_getchar(void)
{
	int lsr;
	int c = -1;

	while (c == -1) {
		lsr = UART_IN(UART_LSR);
		if (lsr & UART_LSR_DR)
			c = UART_IN(UART_RX);
		if ((lsr & UART_LSR_RXCERR))
			c = -1;
	}
	return c;
}

static void kgdb_uart_putchar(int c)
{
	while ((UART_IN(UART_LSR) & UART_LSR_THRE) == 0)
		;
	UART_OUT(UART_TX, c);
}

/*
 * Initialize UART to configured/requested values.
 * (But we don't interrupts yet, or interact w/serial.c)
 */
static int kgdb_uart_setup(void)
{
	int port;
	int lcr = 0;
	int bdiv = 0;

	if (kgdb_portnum >= UART_NPORTS) {
		KGDB_PRINTK("uart port %d invalid.\n", kgdb_portnum);
		return -1;
	}

	kgdb_uart_port = &uart_ports[kgdb_portnum];

	/* Init sequence from gdb_hook_interrupt */
	UART_IN(UART_RX);
	UART_OUT(UART_IER, 0);

	UART_IN(UART_RX);	/* Serial driver comments say */
	UART_IN(UART_IIR);	/* this clears interrupt regs */
	UART_IN(UART_MSR);

	/* Figure basic LCR values */
	switch (kgdb_bits) {
	case '7':
		lcr |= UART_LCR_WLEN7;
		break;
	default: case '8':
		lcr |= UART_LCR_WLEN8;
		break;
	}
	switch (kgdb_parity) {
	case 'O':
		lcr |= UART_LCR_PARITY;
		break;
	case 'E':
		lcr |= (UART_LCR_PARITY | UART_LCR_EPAR);
		break;
	default: break;
	}

	/* Figure the baud rate divisor */
	bdiv = (SB_BASE/kgdb_baud);

	/* Set the baud rate and LCR values */
	UART_OUT(UART_LCR, (lcr | UART_LCR_DLAB));
	UART_OUT(UART_DLL, (bdiv & 0xff));
	UART_OUT(UART_DLM, ((bdiv >> 8) & 0xff));
	UART_OUT(UART_LCR, lcr);

	/* Set the MCR */
	UART_OUT(UART_MCR, SB_MCR);

	/* Turn off FIFOs for now */
	UART_OUT(UART_FCR, 0);

	/* Setup complete: initialize function pointers */
	kgdb_getchar = kgdb_uart_getchar;
	kgdb_putchar = kgdb_uart_putchar;

	return 0;
}
#endif /* CONFIG_SH_KGDB */
