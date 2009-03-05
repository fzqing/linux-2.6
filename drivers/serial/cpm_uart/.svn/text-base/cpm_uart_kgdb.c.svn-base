/*
 * drivers/serial/cpm_uart/cpm_uart_kgdb.c
 *
 * CPM UART interface for kgdb.
 *
 * Author: Vitaly Bordug <vbordug@ru.mvista.com>
 *
 * Used some bits from drivers/serial/kgdb_8250.c as a template
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/kgdb.h>
#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>
#include <linux/touch_watchdogs.h>

#include <asm/io.h>
#include <asm/serial.h>		/* For BASE_BAUD and SERIAL_PORT_DFNS */

#include "cpm_uart.h"

#define GDB_BUF_SIZE	512	/* power of 2, please */


static char kgdb_buf[GDB_BUF_SIZE], *kgdbp;
static int kgdb_chars;

/* Forward declarations. */

/*
 * Receive character from the serial port.  This only works well
 * before the port is initialize for real use.
 */
static int kgdb_wait_key(char *obuf)
{
	struct uart_cpm_port *pinfo;

	u_char				c, *cp;
	volatile	cbd_t		*bdp;
	int				i;

	pinfo = &cpm_uart_ports[KGDB_PINFO_INDEX];

	/* Get the address of the host memory buffer.
	 */
	bdp = pinfo->rx_cur;

	do {
		touch_watchdogs();
	} while (bdp->cbd_sc & BD_SC_EMPTY);

	/* If the buffer address is in the CPM DPRAM, don't
	 * convert it.
	 */
	cp = cpm2cpu_addr(bdp->cbd_bufaddr);

	if (obuf) {
		i = c = bdp->cbd_datlen;
		while (i-- > 0)
		{
			*obuf++ = *cp++;
		}
	} else {
		c = *cp;
	}
	bdp->cbd_sc |= BD_SC_EMPTY;

	if (bdp->cbd_sc & BD_SC_WRAP) {
		bdp = pinfo->rx_bd_base;
	} else {
		bdp++;
	}
	pinfo->rx_cur = (cbd_t *)bdp;

	return((int)c);
}


/*
 * Wait until the interface can accept a char, then write it.
 */
static void
kgdb_put_debug_char(int chr)
{
	static char ch[2];
	ch[0]=(char)chr;
	cpm_uart_early_write(KGDB_PINFO_INDEX, ch, 1);
}


/*
 * Get a char if available, return -1 if nothing available.
 * Empty the receive buffer first, then look at the interface hardware.
 */
static int
kgdb_get_debug_char(void)
{
	if (kgdb_chars<=0) {
		kgdb_chars = kgdb_wait_key(kgdb_buf);
		kgdbp = kgdb_buf;
	}
	kgdb_chars--;

	return (*kgdbp++);
}

static void termios_set_options(int index,
		 int baud, int parity, int bits, int flow)
{
	struct termios termios;
	struct uart_port *port;
	struct uart_cpm_port *pinfo;

	BUG_ON(index>UART_NR);

	port =
	    (struct uart_port *)&cpm_uart_ports[index];
	pinfo = (struct uart_cpm_port *)port;

	/*
	 * Ensure that the serial console lock is initialised
	 * early.
	 */
	spin_lock_init(&port->lock);

	memset(&termios, 0, sizeof(struct termios));

	termios.c_cflag = CREAD | HUPCL | CLOCAL;

	termios.c_cflag |= baud;

	if (bits == 7)
		termios.c_cflag |= CS7;
	else
		termios.c_cflag |= CS8;

	switch (parity) {
	case 'o': case 'O':
		termios.c_cflag |= PARODD;
		/*fall through*/
	case 'e': case 'E':
		termios.c_cflag |= PARENB;
		break;
	}

	if (flow == 'r')
		termios.c_cflag |= CRTSCTS;

	port->ops->set_termios(port, &termios, NULL);
}

/*
 *  Returns:
 *	0 on success, 1 on failure.
 */
static int kgdb_init(void)
{
	struct uart_port *port;
	struct uart_cpm_port *pinfo;

	int use_bootmem = 0; /* use dma by default */

	if(!cpm_uart_nr)
	{
		use_bootmem = 1;
		cpm_uart_init_portdesc();
	}
	port = (struct uart_port *)&cpm_uart_ports[KGDB_PINFO_INDEX];
	pinfo = (struct uart_cpm_port *)port;

	if (cpm_uart_early_setup(KGDB_PINFO_INDEX, use_bootmem))
		return 1;

	termios_set_options(KGDB_PINFO_INDEX, KGDB_BAUD,'n',8,'n');
        if (IS_SMC(pinfo))
                pinfo->smcp->smc_smcm |= SMCM_TX;
        else
                pinfo->sccp->scc_sccm |= UART_SCCM_TX;

	return 0;
}


struct kgdb_io kgdb_io_ops = {
	.read_char = kgdb_get_debug_char,
	.write_char = kgdb_put_debug_char,
	.init = kgdb_init,
};

