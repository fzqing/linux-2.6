/*
 * drivers/serial/kgdb_serial_txx9.c
 *
 * kgdb interface for gdb
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * Copyright 2005 MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or (at your
 *  option) any later version.
 */

#include <linux/config.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kgdb.h>
#include <asm/io.h>

/* Speed of the UART. */
#if defined(CONFIG_KGDB_9600BAUD)
static unsigned int kgdb_txx9_baud = 9600;
#elif defined(CONFIG_KGDB_19200BAUD)
static unsigned int kgdb_txx9_baud = 19200;
#elif defined(CONFIG_KGDB_38400BAUD)
static unsigned int kgdb_txx9_baud = 38400;
#elif defined(CONFIG_KGDB_57600BAUD)
static unsigned int kgdb_txx9_baud = 57600;
#else
static unsigned int kgdb_txx9_baud = 115200; /* Start with this if not given */
#endif

int kgdb_txx9_ttyS = 1;

#if defined(CONFIG_TOSHIBA_RBTX4939)
#define TXX9_NPORT 4
#else
#define TXX9_NPORT 2
#endif

static struct uart_port uart_txx9_port[TXX9_NPORT];

/* TXX9 Serial Registers */
#define TXX9_SILCR	0x00
#define TXX9_SIDISR	0x08
#define TXX9_SISCISR	0x0c
#define TXX9_SIFCR	0x10
#define TXX9_SIFLCR	0x14
#define TXX9_SIBGR	0x18
#define TXX9_SITFIFO	0x1c
#define TXX9_SIRFIFO	0x20

/* SILCR : Line Control */
#define TXX9_SILCR_SCS_IMCLK_BG	0x00000020
#define TXX9_SILCR_SCS_SCLK_BG	0x00000060
#define TXX9_SILCR_USBL_1BIT	0x00000000
#define TXX9_SILCR_UMODE_8BIT	0x00000000

/* SIDISR : DMA/Int. Status */
#define TXX9_SIDISR_RFDN_MASK	0x0000001f

/* SISCISR : Status Change Int. Status */
#define TXX9_SICISR_TRDY	0x00000004

/* SIFCR : FIFO Control */
#define TXX9_SIFCR_SWRST	0x00008000

/* SIBGR : Baud Rate Control */
#define TXX9_SIBGR_BCLK_T0	0x00000000
#define TXX9_SIBGR_BCLK_T2	0x00000100
#define TXX9_SIBGR_BCLK_T4	0x00000200
#define TXX9_SIBGR_BCLK_T6	0x00000300

static inline unsigned int sio_in(struct uart_port *port, int offset)
{
	return *(volatile u32 *)(port->membase + offset);
}

static inline void sio_out(struct uart_port *port, int offset, unsigned int value)
{
	*(volatile u32 *)(port->membase + offset) = value;
}

void kgdb_serial_txx9_setup(struct uart_port *port, int num)
{
	memcpy(&uart_txx9_port[num], port, sizeof(struct uart_port));
}

static int txx9_sio_kgdb_init(void)
{
	struct uart_port *port = &uart_txx9_port[kgdb_txx9_ttyS];
	unsigned int quot, sibgr;

	if (port->iotype != UPIO_MEM && port->iotype != UPIO_MEM32)
		return -1;

	/* Reset the UART. */
	sio_out(port, TXX9_SIFCR, TXX9_SIFCR_SWRST);
#ifdef CONFIG_CPU_TX49XX
	/*
	 * TX4925 BUG WORKAROUND.  Accessing SIOC register
	 * immediately after soft reset causes bus error.
	 */
	iob();
	udelay(1);
#endif
	/* Wait until reset is complete. */
	while (sio_in(port, TXX9_SIFCR) & TXX9_SIFCR_SWRST);

	/* Select the frame format and input clock. */
	sio_out(port, TXX9_SILCR,
		TXX9_SILCR_UMODE_8BIT | TXX9_SILCR_USBL_1BIT |
		((port->flags & UPF_MAGIC_MULTIPLIER) ?
		TXX9_SILCR_SCS_SCLK_BG : TXX9_SILCR_SCS_IMCLK_BG));

	/* Select the input clock prescaler that fits the baud rate. */
	quot = (port->uartclk + 8 * kgdb_txx9_baud) / (16 * kgdb_txx9_baud);
	if (quot < (256 << 1))
		sibgr = (quot >> 1) | TXX9_SIBGR_BCLK_T0;
	else if (quot < ( 256 << 3))
		sibgr = (quot >> 3) | TXX9_SIBGR_BCLK_T2;
	else if (quot < ( 256 << 5))
		sibgr = (quot >> 5) | TXX9_SIBGR_BCLK_T4;
	else if (quot < ( 256 << 7))
		sibgr = (quot >> 7) | TXX9_SIBGR_BCLK_T6;
	else
		sibgr = 0xff | TXX9_SIBGR_BCLK_T6;

	sio_out(port, TXX9_SIBGR, sibgr);

	/* Enable receiver and transmitter. */
	sio_out(port, TXX9_SIFLCR, 0);

	return 0;
}

static void txx9_sio_kgdb_late_init(void)
{
	request_mem_region(uart_txx9_port[kgdb_txx9_ttyS].mapbase, 0x40,
			   "serial_txx9(debug)");
}

static int txx9_sio_kgdb_read(void)
{
	struct uart_port *port = &uart_txx9_port[kgdb_txx9_ttyS];

	while ((sio_in(port, TXX9_SIDISR) & TXX9_SIDISR_RFDN_MASK) == 0);

	return sio_in(port, TXX9_SIRFIFO);
}

static void txx9_sio_kgdb_write(int ch)
{
	struct uart_port *port = &uart_txx9_port[kgdb_txx9_ttyS];

	while (!(sio_in(port, TXX9_SISCISR) & TXX9_SICISR_TRDY));

	sio_out(port, TXX9_SITFIFO, ch);
}

struct kgdb_io kgdb_io_ops = {
	.read_char	= txx9_sio_kgdb_read,
	.write_char	= txx9_sio_kgdb_write,
	.init		= txx9_sio_kgdb_init,
	.late_init	= txx9_sio_kgdb_late_init
};
