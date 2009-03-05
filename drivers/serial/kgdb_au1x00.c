/*
 * drivers/serial/kgdb_au1x00.c
 * Based on the arch/mips/au1000/common/dbg_io.c
 *
 * kgdb interface for gdb
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * Copyright 2005-2006 MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or (at your
 *  option) any later version.
 */

#include <linux/config.h>
#include <linux/kgdb.h>
#include <asm/mach-au1x00/au1000.h>

/* memory-mapped read/write of the port */
#define UART_READ(reg)		(au_readl(UART_DEBUG_BASE + reg))
#define UART_WRITE(reg,val)	(au_writel(val, UART_DEBUG_BASE + reg))

/* Speed of the UART. */
#if defined(CONFIG_KGDB_9600BAUD)
static unsigned int kgdb_au1x00_baud = 9600;
#elif defined(CONFIG_KGDB_19200BAUD)
static unsigned int kgdb_au1x00_baud = 19200;
#elif defined(CONFIG_KGDB_38400BAUD)
static unsigned int kgdb_au1x00_baud = 38400;
#elif defined(CONFIG_KGDB_57600BAUD)
static unsigned int kgdb_au1x00_baud = 57600;
#else
static unsigned int kgdb_au1x00_baud = 115200;	/* Start with this if not given */
#endif

extern unsigned long get_au1x00_uart_baud_base(void);
extern unsigned long cal_r4koff(void);

static int au1x00_kgdb_init(void)
{
	if (UART_READ(UART_MOD_CNTRL)  != (UART_MCR_DTR | UART_MCR_RTS))
		UART_WRITE(UART_MOD_CNTRL, UART_MCR_DTR | UART_MCR_RTS);

	/* disable interrupts */
	UART_WRITE(UART_IER, 0);

	if (!get_au1x00_uart_baud_base())
		cal_r4koff();

	/* set up baud rate */
	{
		u32 divisor;

		/* set divisor */
		divisor = get_au1x00_uart_baud_base() / kgdb_au1x00_baud;
		UART_WRITE(UART_CLK, divisor & 0xffff);
	}

	/* set data format */
	UART_WRITE(UART_LCR, UART_LCR_WLEN8);

	return 0;
}

static void au1x00_kgdb_late_init(void)
{
	request_mem_region(CPHYSADDR(UART_DEBUG_BASE), 0x100000,
			   "Au1x00 UART(debug)");
}

static int au1x00_kgdb_read_char(void)
{
	while (!(UART_READ(UART_LSR) & UART_LSR_DR));

	return UART_READ(UART_RX) & 0xff;
}


static void au1x00_kgdb_write_char(int byte)
{
	while (!(UART_READ(UART_LSR) & UART_LSR_TEMT));

	UART_WRITE(UART_TX, byte & 0xff);
}

struct kgdb_io kgdb_io_ops = {
	.read_char  = au1x00_kgdb_read_char,
	.write_char = au1x00_kgdb_write_char,
	.init	    = au1x00_kgdb_init,
	.late_init  = au1x00_kgdb_late_init
};
