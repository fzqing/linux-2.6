/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2004, 2005 Cavium Networks
 */
#include <linux/console.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <asm/time.h>
#include <linux/serial_core.h>
#include <asm/gdb-stub.h>
#include <hal.h>

extern int serial8250_register_port(struct uart_port *);

static int octeon_serial_init(void)
{
	int port;

	const int max_port = 1;

	struct uart_port octeon_port;
	memset(&octeon_port, 0, sizeof(octeon_port));
	octeon_port.flags = ASYNC_SKIP_TEST | UPF_SHARE_IRQ;
	octeon_port.iotype = UPIO_MEM;
	octeon_port.regshift = 3;	/* I/O addresses are every 8 bytes */
	octeon_port.uartclk = mips_hpt_frequency;	/* Clock rate of the chip */
	octeon_port.fifosize = 64;

	for (port = 0; port < max_port; port++) {
		octeon_port.line = port;
		octeon_port.mapbase = 0x8001180000000800ull + (1024 * port);
		octeon_port.membase = (unsigned char *)octeon_port.mapbase;
		octeon_port.irq = 42+port;
		serial8250_register_port(&octeon_port);

	}

	return 0;
}

late_initcall(octeon_serial_init);
