/*
 * drivers/serial/sh-sci.c
 *
 * SuperH on-chip serial module support.  (SCI with no FIFO / with FIFO)
 *
 *  Copyright (C) 2002, 2003, 2004  Paul Mundt
 *
 * based off of the old drivers/char/sh-sci.c by:
 *
 *   Copyright (C) 1999, 2000  Niibe Yutaka
 *   Copyright (C) 2000  Sugioka Toshinobu
 *   Modified to support multiple serial ports. Stuart Menefy (May 2000).
 *   Modified to support SecureEdge. David McCullough (2002)
 *   Modified to support SH7300 SCIF. Takashi Kusuda (Jun 2003).
 *   Modified to support SH7705/SH7706/SH7710/SH7720/SH7727 SCIF. Takashi Kusuda (Sep 2004).
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#undef DEBUG

#include <linux/config.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/sysrq.h>
#include <linux/fcntl.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/console.h>
#include <linux/bitops.h>
#include <linux/kgdb.h>

#ifdef CONFIG_CPU_FREQ
#include <linux/notifier.h>
#include <linux/cpufreq.h>
#endif

#include <asm/system.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include <linux/generic_serial.h>

#ifdef CONFIG_SH_STANDARD_BIOS
#include <asm/sh_bios.h>
#endif

#if defined(CONFIG_SERIAL_SH_SCI_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include "sh-sci.h"

#ifdef CONFIG_KGDB_SH_SCI
/* Speed of the UART. */
#if defined(CONFIG_KGDB_9600BAUD)
static int kgdbsci_baud = 9600;
#elif defined(CONFIG_KGDB_19200BAUD)
static int kgdbsci_baud = 19200;
#elif defined(CONFIG_KGDB_38400BAUD)
static int kgdbsci_baud = 38400;
#elif defined(CONFIG_KGDB_57600BAUD)
static int kgdbsci_baud = 57600;
#else
static int kgdbsci_baud = 115200;	/* Start with this if not given */
#endif

/* Index of the UART, matches ttySX naming. */
#if defined(CONFIG_KGDB_TTYS1)
static int kgdbsci_ttyS = 1;
#elif defined(CONFIG_KGDB_TTYS2)
static int kgdbsci_ttyS = 2;
#elif defined(CONFIG_KGDB_TTYS3)
static int kgdbsci_ttyS = 3;
#else
static int kgdbsci_ttyS = 0;		/* Start with this if not given */
#endif

/* Make life easier on us. */
#define KGDBPORT	sci_ports[kgdbsci_ttyS]
#endif /* CONFIG_KGDB_SH_SCI */

#ifdef CONFIG_SERIAL_SH_SCI_CONSOLE
static struct sci_port *serial_console_port = 0;
#endif /* CONFIG_SERIAL_SH_SCI_CONSOLE */

/* Function prototypes */
static void sci_stop_tx(struct uart_port *port, unsigned int tty_stop);
static void sci_start_tx(struct uart_port *port, unsigned int tty_start);
static void sci_start_rx(struct uart_port *port, unsigned int tty_start);
static void sci_stop_rx(struct uart_port *port);
static int sci_request_irq(struct sci_port *port);
static void sci_free_irq(struct sci_port *port);
static void sci_set_termios(struct uart_port *port, struct termios *termios,
			    struct termios *old);

static struct sci_port sci_ports[SCI_NPORTS];
static struct uart_driver sci_uart_driver;

#if defined(CONFIG_SH_STANDARD_BIOS) || defined(CONFIG_KGDB_SH_SCI)
static int get_char_for_gdb(struct uart_port *port)
{
	unsigned long flags;
	unsigned short status;
	int c;

	local_irq_save(flags);
        do {
		status = sci_in(port, SCxSR);
		if (status & SCxSR_ERRORS(port)) {
			/* Clear error flags. */
			sci_out(port, SCxSR, SCxSR_ERROR_CLEAR(port));
			continue;
		}
	} while (!(status & SCxSR_RDxF(port)));
	c = sci_in(port, SCxRDR);
	sci_in(port, SCxSR);            /* Dummy read */
	sci_out(port, SCxSR, SCxSR_RDxF_CLEAR(port));
	local_irq_restore(flags);

	return c;
}
#endif /* CONFIG_SH_STANDARD_BIOS || CONFIG_KGDB_SH_SCI */

/*
 * Send the packet in buffer.  The host gets one chance to read it.
 * This routine does not wait for a positive acknowledge.
 */

#ifdef CONFIG_SERIAL_SH_SCI_CONSOLE
static void put_char(struct uart_port *port, char c)
{
	unsigned long flags;
	unsigned short status;

	local_irq_save(flags);

	do {
		status = sci_in(port, SCxSR);
	} while (!(status & SCxSR_TDxE(port)));

	sci_out(port, SCxTDR, c);
	sci_in(port, SCxSR);            /* Dummy read */
	sci_out(port, SCxSR, SCxSR_TDxE_CLEAR(port));

	local_irq_restore(flags);
}

static void put_string(struct sci_port *sci_port, const char *buffer, int count)
{
	struct uart_port *port = &sci_port->port;
	const unsigned char *p = buffer;
	int i;

#ifdef CONFIG_SH_STANDARD_BIOS
	int checksum;
	const char hexchars[] = "0123456789abcdef";

    	/* This call only does a trap the first time it is
	 * called, and so is safe to do here unconditionally
	 */
	if (sh_bios_in_gdb_mode()) {
	    /*  $<packet info>#<checksum>. */
	    do {
		unsigned char c;
		put_char(port, '$');
		put_char(port, 'O'); /* 'O'utput to console */
		checksum = 'O';

		for (i=0; i<count; i++) { /* Don't use run length encoding */
			int h, l;

			c = *p++;
			h = hexchars[c >> 4];
			l = hexchars[c % 16];
			put_char(port, h);
			put_char(port, l);
			checksum += h + l;
		}
		put_char(port, '#');
		put_char(port, hexchars[checksum >> 4]);
		put_char(port, hexchars[checksum & 16]);
	    } while  (get_char(port) != '+');
	} else
#endif /* CONFIG_SH_STANDARD_BIOS */
	for (i=0; i<count; i++) {
		if (*p == 10)
			put_char(port, '\r');
		put_char(port, *p++);
	}
}
#endif /* CONFIG_SERIAL_SH_SCI_CONSOLE */


#ifdef CONFIG_KGDB_SH_SCI
static int kgdbsci_read_char(void)
{
	return get_char_for_gdb(&KGDBPORT.port);
}

/* Called from kgdbstub.c to put a character, just a wrapper */
static void kgdbsci_write_char(int c)
{
	unsigned short status;

	do
		status = sci_in(&KGDBPORT.port, SCxSR);
	while (!(status & SCxSR_TDxE(&KGDBPORT.port)));

	sci_out(&KGDBPORT.port, SCxTDR, c);
	sci_in(&KGDBPORT.port, SCxSR);	/* Dummy read */
	sci_out(&KGDBPORT.port, SCxSR, SCxSR_TDxE_CLEAR(&KGDBPORT.port));
}

#ifndef CONFIG_SERIAL_SH_SCI_CONSOLE
/* If we don't have console, we never hookup IRQs.  But we need to
 * hookup one so that we can interrupt the system.
 */
static irqreturn_t kgdbsci_rx_interrupt(int irq, void *ptr,
		struct pt_regs *regs)
{
	struct uart_port *port = ptr;

	if (!(sci_in(port, SCxSR) & SCxSR_RDxF(port)))
		return IRQ_NONE;

	/* We've got an interrupt, so go ahead and call breakpoint() */
	breakpoint();

	sci_in(port, SCxSR); /* dummy read */
	sci_out(port, SCxSR, SCxSR_RDxF_CLEAR(port));

	return IRQ_HANDLED;
}

static irqreturn_t kgdbsci_mpxed_interrupt(int irq, void *ptr,
		struct pt_regs *regs)
{
        unsigned short ssr_status, scr_status;
        struct uart_port *port = ptr;

        ssr_status = sci_in(port,SCxSR);
        scr_status = sci_in(port,SCSCR);

	/* Rx Interrupt */
        if ((ssr_status&0x0002) && (scr_status&0x0040))
		kgdbsci_rx_interrupt(irq, ptr, regs);

	return IRQ_HANDLED;
}

static void __init kgdbsci_lateinit(void)
{
	if (KGDBPORT.irqs[0] == KGDBPORT.irqs[1]) {
		if (!KGDBPORT.irqs[0]) {
			printk(KERN_ERR "kgdbsci: Cannot allocate irq.\n");
			return;
		}
		if (request_irq(KGDBPORT.irqs[0], kgdbsci_mpxed_interrupt,
					SA_INTERRUPT, "kgdbsci",
					&KGDBPORT.port)) {
			printk(KERN_ERR "kgdbsci: Cannot allocate irq.\n");
			return;
		}
	} else {
		if (KGDBPORT.irqs[1])
			request_irq(KGDBPORT.irqs[1],
					kgdbsci_rx_interrupt, SA_INTERRUPT,
					"kgdbsci", &KGDBPORT.port);
	}
}
#endif

/*
 * We use the normal init routine to setup the port, so we can't be
 * in here too early.
 */
static int kgdbsci_init(void)
{
	struct termios termios;

	memset(&termios, 0, sizeof(struct termios));

	termios.c_cflag = CREAD | HUPCL | CLOCAL | CS8;
	switch (kgdbsci_baud) {
	case 9600:
		termios.c_cflag |= B9600;
		break;
	case 19200:
		termios.c_cflag |= B19200;
		break;
	case 38400:
		termios.c_cflag |= B38400;
		break;
	case 57600:
		termios.c_cflag |= B57600;
		break;
	case 115200:
		termios.c_cflag |= B115200;
		break;
	}
	sci_set_termios(&KGDBPORT.port, &termios, NULL);

	return 0;
}

struct kgdb_io kgdb_io_ops = {
	.read_char = kgdbsci_read_char,
	.write_char = kgdbsci_write_char,
	.init = kgdbsci_init,
#ifndef CONFIG_SERIAL_SH_SCI_CONSOLE
	.late_init = kgdbsci_lateinit,
#endif
};

/*
 * Syntax for this cmdline option is "kgdbsci=ttyno,baudrate".
 */
static int __init
kgdbsci_opt(char *str)
{
	/* We might have anywhere from 1 to 3 ports. */
	if (*str < '0' || *str > SCI_NPORTS + '0')
		 goto errout;
	kgdbsci_ttyS = *str - '0';
	str++;
	if (*str != ',')
		 goto errout;
	str++;
	kgdbsci_baud = simple_strtoul(str, &str, 10);
	if (kgdbsci_baud != 9600 && kgdbsci_baud != 19200 &&
	    kgdbsci_baud != 38400 && kgdbsci_baud != 57600 &&
	    kgdbsci_baud != 115200)
		 goto errout;

	return 0;

errout:
	printk(KERN_ERR "Invalid syntax for option kgdbsci=\n");
	return 1;
}
__setup("kgdbsci", kgdbsci_opt);
#endif /* CONFIG_KGDB_SH_SCI */

#if defined(__H8300S__)
enum { sci_disable, sci_enable };

static void h8300_sci_enable(struct uart_port* port, unsigned int ctrl)
{
	volatile unsigned char *mstpcrl=(volatile unsigned char *)MSTPCRL;
	int ch = (port->mapbase  - SMR0) >> 3;
	unsigned char mask = 1 << (ch+1);

	if (ctrl == sci_disable) {
		*mstpcrl |= mask;
	} else {
		*mstpcrl &= ~mask;
	}
}
#endif

#if defined(SCI_ONLY) || defined(SCI_AND_SCIF)
#if defined(__H8300H__) || defined(__H8300S__)
static void sci_init_pins_sci(struct uart_port* port, unsigned int cflag)
{
	int ch = (port->mapbase - SMR0) >> 3;

	/* set DDR regs */
	H8300_GPIO_DDR(h8300_sci_pins[ch].port,h8300_sci_pins[ch].rx,H8300_GPIO_INPUT);
	H8300_GPIO_DDR(h8300_sci_pins[ch].port,h8300_sci_pins[ch].tx,H8300_GPIO_OUTPUT);
	/* tx mark output*/
	H8300_SCI_DR(ch) |= h8300_sci_pins[ch].tx;
}
#else
static void sci_init_pins_sci(struct uart_port *port, unsigned int cflag)
{
}
#endif
#endif

#if defined(SCIF_ONLY) || defined(SCI_AND_SCIF)
#if defined(CONFIG_CPU_SH3)
/* For SH7300, SH7705, SH7706, SH7707, SH7709, SH7709A(S), SH7710, SH7720, SH7727, SH7729 */
static void sci_init_pins_scif(struct uart_port *port, unsigned int cflag)
{
	unsigned int fcr_val = 0;
#if !defined(CONFIG_CPU_SUBTYPE_SH7300) /* SH7300 doesn't use RTS/CTS */
	{
		unsigned short data;

		if (cflag & CRTSCTS) {
	                /* We need to set SCPCR to enable RTS/CTS */
#if defined(CONFIG_CPU_SUBTYPE_SH7705) || defined(CONFIG_CPU_SUBTYPE_SH7706)
			/* Clear out SCP7MD1,0, SCP6MD1,0, SCP4MD1,0*/
			data = ctrl_inw(SCPCR);
			ctrl_outw((data&0xf0cf), SCPCR);
#elif defined(CONFIG_CPU_SUBTYPE_SH7710)
			if(port->mapbase == 0xa4400000) { /* SCIF0 */
				/* Clear out PTA1MD[1:0], PTA0MD[1:0] */
					data = ctrl_inw(PORT_PACR);
					ctrl_outw((data&0xfff0), PORT_PACR);

					/* Clear out PTB7MD[1:0], PTB6MD[1:0] */
					data = ctrl_inw(PORT_PBCR);
					ctrl_outw((data&0x0fff), PORT_PBCR);
			} else if(port->mapbase == 0xa4410000){ /* SCIF1 */
				/* Clear out PTB4MD[1:0], PTB3MD[1:0], PTB2MD[1:0], PTB1MD[1:0] */
				data = ctrl_inw(PORT_PBCR);
				ctrl_outw((data&0xfc03), PORT_PBCR);
			}
#elif defined(CONFIG_CPU_SUBTYPE_SH7720)
			if(port->mapbase == 0xa4430000) { /* SCIF0 */
				/* Clear out PTT4MD[1:0], PTT3MD[1:0], PTT2MD[1:0], PTT1MD[1:0] */
				data = ctrl_inw(PORT_PTCR);
				ctrl_outw((data&0xfc03), PORT_PTCR);
			} else if(port->mapbase == 0xa4438000){ /* SCIF1 */
				/* SCIF1 is multiplexed other function. Select SCIF1 */
				data = ctrl_inw(PORT_PSELB);
				ctrl_outw((data&0xc3ff), PORT_PSELB); /* RTS1/CTS1 */
				data = ctrl_inw(PORT_PSELC);
				ctrl_outw((data&0xf0ff), PORT_PSELC); /* RxD1/TxD1 */
				/* Clear out PTV4MD[1:0], PTV3MD[1:0], PTV2MD[1:0], PTV1MD[1:0] */
				data = ctrl_inw(PORT_PVCR);
				ctrl_outw((data&0xfc03), PORT_PVCR);
			}
#elif defined(CONFIG_CPU_SUBTYPE_SH7727)
			/* Clear out SCP7MD[1:0], SCP4MD[1:0] */
			data = ctrl_inw(SCPCR);
			ctrl_outw(data&0x3cff, SCPCR);
#else
			/* Clear out SCP7MD[1:0], SCP6MD[1:0], SCP4MD[1:0] */
			data = ctrl_inw(SCPCR);
			ctrl_outw(data&0x0cff, SCPCR);
#endif
			fcr_val |= SCFCR_MCE;
		} else {
#if defined(CONFIG_CPU_SUBTYPE_SH7705) || defined(CONFIG_CPU_SUBTYPE_SH7706)
		/* Clear out SCP2MD[1:0], Set SCP4MD[1:0]={01}(output) */
		data = ctrl_inw(SCPCR);
		ctrl_outw(((data&0xfccf)|0x0100), SCPCR);
#elif defined(CONFIG_CPU_SUBTYPE_SH7710)
		if(port->mapbase == 0xa4400000) { /* SCIF0 */
			/* Clear out PTA1MD[1:0], PTA0MD[1:0] */
			data = ctrl_inw(PORT_PACR);
			ctrl_outw((data&0xfff0), PORT_PACR);

			/* Set PTB7MD[1:0]={01}(output) */
			data = ctrl_inw(PORT_PBCR);
			ctrl_outw(((data&0x3fff)|0x4000), PORT_PBCR);
		} else if(port->mapbase == 0xa4410000){ /* SCIF1 */
			/* Clear out PTB4MD[1:0], PTB3MD[1:0],
			Set PTB2MD[1:0]={01}(output) */
			data = ctrl_inw(PORT_PBCR);
			ctrl_outw(((data&0xfc0f)|0x0010), PORT_PBCR);
		}
#elif defined(CONFIG_CPU_SUBTYPE_SH7720)
		if(port->mapbase == 0xa4430000) { /* SCIF0 */
		/* Clear out PTT2MD[1:0], PTT1MD[1:0], Set PTT3MD[1:0]={01}(output) */
			data = ctrl_inw(PORT_PTCR);
			ctrl_outw(((data&0xff03)|0x0040), PORT_PTCR);
		} else if(port->mapbase == 0xa4438000){ /* SCIF1 */
			/* SCIF1 is multiplexed other function. Select SCIF1 */
			data = ctrl_inw(PORT_PSELB);
			ctrl_outw((data&0xcfff), PORT_PSELB); /* RTS1 */
			data = ctrl_inw(PORT_PSELC);
			ctrl_outw((data&0xf0ff), PORT_PSELC); /* RxD1/TxD1 */

			/* Clear out PTV2MD[1:0], PTV1MD[1:0], Set PTV3MD[1:0]={01}(output) */
			data = ctrl_inw(PORT_PVCR);
			ctrl_outw(((data&0xff03)|0x0040), PORT_PVCR);
		}
#elif defined(CONFIG_CPU_SUBTYPE_SH7727)
		/* Clear out SCP4MD[1:0] */
		data = ctrl_inw(SCPCR);
		ctrl_outw(data&0xfcff, SCPCR);
#else
		/* Clear out SCP4MD[1:0], Set SCP6MD[1:0]={01}(output)  */
		data = ctrl_inw(SCPCR);
		ctrl_outw((data&0xccff)|0x1000, SCPCR);
#endif
#if defined(CONFIG_CPU_SUBTYPE_SH7705) || defined(CONFIG_CPU_SUBTYPE_SH7706)
		data = ctrl_inb(SCPDR);
		/* Set /RTS2 (bit4) = 0 */
		ctrl_outb((unsigned char)(data&0x00ef), SCPDR);
#elif defined(CONFIG_CPU_SUBTYPE_SH7710)
		data = ctrl_inb(PORT_PBDR);
		if(port->mapbase == 0xa4400000) { /* SCIF0 */
			/* Set /RTS (PBDR:bit7) = 0 */
			ctrl_outb((unsigned char)(data&0x007f), PORT_PBDR);
		} else if(port->mapbase == 0xa4410000){ /* SCIF1 */
			/* Set /RTS (PBDR:bit2) = 0 */
			ctrl_outb((unsigned char)(data&0x00fb), PORT_PBDR);
		}
#elif defined(CONFIG_CPU_SUBTYPE_SH7720)
		/* Set /RTS (bit3) = 0 */
		if(port->mapbase == 0xa4430000) { /* SCIF0 */
			data = ctrl_inb(PORT_PTDR);
			ctrl_outb((unsigned char)(data&0x00f7), PORT_PTDR);
		} else if(port->mapbase == 0xa4438000){ /* SCIF1 */
			data = ctrl_inb(PORT_PVDR);
			ctrl_outb((unsigned char)(data&0x00f7), PORT_PVDR);
		}
#elif defined(CONFIG_CPU_SUBTYPE_SH7727)
		/* Nothing to do */
#else
		data = ctrl_inb(SCPDR);
		/* Set /RTS2 (bit6) = 0 */
		ctrl_outb((unsigned char)(data&0x00bf), SCPDR);
#endif
	}
#endif
	sci_out(port, SCFCR, fcr_val);
}

static void sci_init_pins_irda(struct uart_port *port, unsigned int cflag)
{
	unsigned int fcr_val = 0;

	if (cflag & CRTSCTS)
		fcr_val |= SCFCR_MCE;

	sci_out(port, SCFCR, fcr_val);
}

#else

/* For SH7750,SH7750S,SH7750R,SH7751,SH7751R,SH7760,SH7780 */
static void sci_init_pins_scif(struct uart_port *port, unsigned int cflag)
{
	unsigned int fcr_val = 0;

	if (cflag & CRTSCTS) {
		fcr_val |= SCFCR_MCE;
#if defined(CONFIG_CPU_SUBTYPE_SH7760)
		if (port->mapbase == 0xfe600000) { /* SCIF0 */
			ctrl_outw(ctrl_inl(PORT_PGCR)&0xc3ff, PORT_PGCR); /* PortG Control reg */
		} else if (port->mapbase == 0xfe610000) { /* SCIF1 */
			ctrl_outw(ctrl_inl(PORT_PGCR)&0xff00, PORT_PGCR); /* PortG Control reg */
		} else if (port->mapbase == 0xfe620000) { /* SCIF2 */
			ctrl_outw(ctrl_inl(PORT_PHCR)&0xc03f, PORT_PHCR); /* PortH Control reg */
		}
#elif defined(CONFIG_CPU_SUBTYPE_SH7780)
		if (port->mapbase == 0xffe00000) { /* SCIF0 */
			ctrl_outw(ctrl_inl(GPIO_PHCR)&0xff80, GPIO_PHCR); /* portH Control reg */
		} else if (port->mapbase == 0xffe10000) { /* SCIF1 */
			ctrl_outw(ctrl_inl(GPIO_PHCR)&0xe3ff, GPIO_PHCR); /* PortH Control reg */
	}
#endif
	} else {
#if defined(CONFIG_CPU_SUBTYPE_SH7760)
		if (port->mapbase == 0xfe600000) {
			ctrl_outw(ctrl_inl(PORT_PGCR)&0xc3ff, PORT_PGCR); /* PortG Control reg */
			ctrl_outw(0x0080, SCSPTR0);
		} else if (port->mapbase == 0xfe610000) {
			ctrl_outw(ctrl_inl(PORT_PGCR)&0xfff0, PORT_PGCR); /* PortG Control reg */
			ctrl_outw(0x0080, SCSPTR1);
		} else if (port->mapbase == 0xfe620000) {
			ctrl_outw(ctrl_inl(PORT_PHCR)&0xfc3f, PORT_PHCR); /* PortH Control reg */
			ctrl_outw(0x0080, SCSPTR2);
		}
#elif defined(CONFIG_CPU_SUBTYPE_SH7780)
		if (port->mapbase == 0xffe00000) {
			ctrl_outw(ctrl_inl(GPIO_PHCR)&0xff8f, GPIO_PHCR); /* PortH Control reg */
			ctrl_outw(0x0080, SCSPTR0);
		} else if (port->mapbase == 0xffe10000) {
			ctrl_outw(ctrl_inl(GPIO_PHCR)&0xe3ff, GPIO_PHCR); /* PortH Control reg */
			ctrl_outw(0x0080, SCSPTR1);
		}
#else
		ctrl_outw(0x0080, SCSPTR2); /* Set RTS = 1 */
#endif
	}
	sci_out(port, SCFCR, fcr_val);
}

#endif
#endif /* SCIF_ONLY || SCI_AND_SCIF */

/* ********************************************************************** *
 *                   the interrupt related routines                       *
 * ********************************************************************** */

static void sci_transmit_chars(struct uart_port *port)
{
	struct circ_buf *xmit = &port->info->xmit;
	unsigned int stopped = uart_tx_stopped(port);
	unsigned long flags;
	unsigned short status;
	unsigned short ctrl;
	int count, txroom;

	status = sci_in(port, SCxSR);
	if (!(status & SCxSR_TDxE(port))) {
		local_irq_save(flags);
		ctrl = sci_in(port, SCSCR);
		if (uart_circ_empty(xmit)) {
			ctrl &= ~SCI_CTRL_FLAGS_TIE;
		} else {
			ctrl |= SCI_CTRL_FLAGS_TIE;
		}
		sci_out(port, SCSCR, ctrl);
		local_irq_restore(flags);
		return;
	}

#if !defined(SCI_ONLY)
	if (port->type == PORT_SCIF) {
#if defined(CONFIG_CPU_SUBTYPE_SH7760) || defined(CONFIG_CPU_SUBTYPE_SH7780)
		txroom = 64 - sci_in(port, SCTFDR);
#else
		txroom = SCIF_TXROOM_MAX - (sci_in(port, SCFDR)>>8);
#endif
	} else {
		txroom = (sci_in(port, SCxSR) & SCI_TDRE)?1:0;
	}
#else
	txroom = (sci_in(port, SCxSR) & SCI_TDRE)?1:0;
#endif

	count = txroom;

	do {
		unsigned char c;

		if (port->x_char) {
			c = port->x_char;
			port->x_char = 0;
		} else if (!uart_circ_empty(xmit) && !stopped) {
			c = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		} else {
			break;
		}

		sci_out(port, SCxTDR, c);

		port->icount.tx++;
	} while (--count > 0);

	sci_out(port, SCxSR, SCxSR_TDxE_CLEAR(port));

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
	if (uart_circ_empty(xmit)) {
		sci_stop_tx(port, 0);
	} else {
		local_irq_save(flags);
		ctrl = sci_in(port, SCSCR);

#if !defined(SCI_ONLY)
		if (port->type == PORT_SCIF) {
			sci_in(port, SCxSR); /* Dummy read */
			sci_out(port, SCxSR, SCxSR_TDxE_CLEAR(port));
		}
#endif

		ctrl |= SCI_CTRL_FLAGS_TIE;
		sci_out(port, SCSCR, ctrl);
		local_irq_restore(flags);
	}
}

/* On SH3, SCIF may read end-of-break as a space->mark char */
#define STEPFN(c)  ({int __c=(c); (((__c-1)|(__c)) == -1); })

static inline void sci_receive_chars(struct uart_port *port,
				     struct pt_regs *regs)
{
	struct tty_struct *tty = port->info->tty;
	int i, count, copied = 0;
	unsigned short status;

	status = sci_in(port, SCxSR);
	if (!(status & SCxSR_RDxF(port)))
		return;

	while (1) {
#if !defined(SCI_ONLY)
		if (port->type == PORT_SCIF) {
#if defined(CONFIG_CPU_SUBTYPE_SH7760) || defined(CONFIG_CPU_SUBTYPE_SH7780)
			count = sci_in(port, SCRFDR);
#else
			count = sci_in(port, SCFDR)&SCIF_RFDC_MASK ;
#endif
		} else {
			count = (sci_in(port, SCxSR)&SCxSR_RDxF(port))?1:0;
		}
#else
		count = (sci_in(port, SCxSR)&SCxSR_RDxF(port))?1:0;
#endif

		/* Don't copy more bytes than there is room for in the buffer */
		if (tty->flip.count + count > TTY_FLIPBUF_SIZE)
			count = TTY_FLIPBUF_SIZE - tty->flip.count;

		/* If for any reason we can't copy more data, we're done! */
		if (count == 0)
			break;

		if (port->type == PORT_SCI) {
			char c = sci_in(port, SCxRDR);
                       if(((struct sci_port *)port)->break_flag
			    || uart_handle_sysrq_char(port, c, regs)) {
				count = 0;
			} else {
			    tty->flip.char_buf_ptr[0] = c;
			    tty->flip.flag_buf_ptr[0] = TTY_NORMAL;
			}
		} else {
			for (i=0; i<count; i++) {
				char c = sci_in(port, SCxRDR);
				status = sci_in(port, SCxSR);
#if defined(CONFIG_CPU_SH3)
				/* Skip "chars" during break */
				if (((struct sci_port *)port)->break_flag) {
					if ((c == 0) &&
					    (status & SCxSR_FER(port))) {
						count--; i--;
						continue;
					}
					/* Nonzero => end-of-break */
					pr_debug("scif: debounce<%02x>\n", c);
					((struct sci_port *)port)->break_flag = 0;
					if (STEPFN(c)) {
						count--; i--;
						continue;
					}
				}
#endif /* CONFIG_CPU_SH3 */
				if (uart_handle_sysrq_char(port, c, regs)) {
					count--; i--;
					continue;
				}

#ifdef CONFIG_KGDB_SH_SCI
				/* We assume that a ^C on the port KGDB
				 * is using means that KGDB wants to
				 * interrupt the running system.
				 */
				if (port->line == KGDBPORT.port.line &&
						c == 3)
					breakpoint();
#endif

				/* Store data and status */
				tty->flip.char_buf_ptr[i] = c;
				if (status&SCxSR_FER(port)) {
					tty->flip.flag_buf_ptr[i] = TTY_FRAME;
					pr_debug("sci: frame error\n");
				} else if (status&SCxSR_PER(port)) {
					tty->flip.flag_buf_ptr[i] = TTY_PARITY;
					pr_debug("sci: parity error\n");
				} else {
					tty->flip.flag_buf_ptr[i] = TTY_NORMAL;
				}
			}
		}

		sci_in(port, SCxSR); /* dummy read */
		sci_out(port, SCxSR, SCxSR_RDxF_CLEAR(port));

		/* Update the kernel buffer end */
		tty->flip.count += count;
		tty->flip.char_buf_ptr += count;
		tty->flip.flag_buf_ptr += count;
		copied += count;
		port->icount.rx += count;
	}

	if (copied) {
		/* Tell the rest of the system the news. New characters! */
		tty_flip_buffer_push(tty);
	} else {
		sci_in(port, SCxSR); /* dummy read */
		sci_out(port, SCxSR, SCxSR_RDxF_CLEAR(port));
	}
}

#define SCI_BREAK_JIFFIES (HZ/20)
/* The sci generates interrupts during the break,
 * 1 per millisecond or so during the break period, for 9600 baud.
 * So dont bother disabling interrupts.
 * But dont want more than 1 break event.
 * Use a kernel timer to periodically poll the rx line until
 * the break is finished.
 */
static void sci_schedule_break_timer(struct sci_port *port)
{
	port->break_timer.expires = jiffies + SCI_BREAK_JIFFIES;
	add_timer(&port->break_timer);
}
/* Ensure that two consecutive samples find the break over. */
static void sci_break_timer(unsigned long data)
{
    struct sci_port * port = (struct sci_port *)data;
	if(sci_rxd_in(&port->port) == 0) {
		port->break_flag = 1;
	    sci_schedule_break_timer(port);
	} else if(port->break_flag == 1){
		/* break is over. */
		port->break_flag = 2;
	    sci_schedule_break_timer(port);
	} else port->break_flag = 0;
}

static inline int sci_handle_errors(struct uart_port *port)
{
	int copied = 0;
	unsigned short status = sci_in(port, SCxSR);
	struct tty_struct *tty = port->info->tty;

	if (status&SCxSR_ORER(port) && tty->flip.count<TTY_FLIPBUF_SIZE) {
		/* overrun error */
		copied++;
		*tty->flip.flag_buf_ptr++ = TTY_OVERRUN;
		pr_debug("sci: overrun error\n");
	}

	if (status&SCxSR_FER(port) && tty->flip.count<TTY_FLIPBUF_SIZE) {
		if (sci_rxd_in(port) == 0) {
			/* Notify of BREAK */
			struct sci_port * sci_port = (struct sci_port *)port;
                       if(!sci_port->break_flag) {
	                        sci_port->break_flag = 1;
                               sci_schedule_break_timer((struct sci_port *)port);
				/* Do sysrq handling. */
				if(uart_handle_break(port)) {
					return 0;
				}
			        pr_debug("sci: BREAK detected\n");
			        copied++;
			        *tty->flip.flag_buf_ptr++ = TTY_BREAK;
                       }
		}
		else {
			/* frame error */
			copied++;
			*tty->flip.flag_buf_ptr++ = TTY_FRAME;
			pr_debug("sci: frame error\n");
		}
	}

	if (status&SCxSR_PER(port) && tty->flip.count<TTY_FLIPBUF_SIZE) {
		/* parity error */
		copied++;
		*tty->flip.flag_buf_ptr++ = TTY_PARITY;
		pr_debug("sci: parity error\n");
	}

	if (copied) {
		tty->flip.count += copied;
		tty_flip_buffer_push(tty);
	}

	return copied;
}

static inline int sci_handle_breaks(struct uart_port *port)
{
	int copied = 0;
	unsigned short status = sci_in(port, SCxSR);
	struct tty_struct *tty = port->info->tty;
	struct sci_port *s = &sci_ports[port->line];

	if (!s->break_flag && status & SCxSR_BRK(port) &&
	    tty->flip.count < TTY_FLIPBUF_SIZE) {
#if defined(CONFIG_CPU_SH3)
		/* Debounce break */
		s->break_flag = 1;
#endif
		/* Notify of BREAK */
		copied++;
		*tty->flip.flag_buf_ptr++ = TTY_BREAK;
		pr_debug("sci: BREAK detected\n");
	}

#if defined(SCIF_ORER)
	/* XXX: Handle SCIF overrun error */
	if (port->type == PORT_SCIF && (sci_in(port, SCLSR) & SCIF_ORER) != 0) {
		sci_out(port, SCLSR, 0);
		if(tty->flip.count<TTY_FLIPBUF_SIZE) {
			copied++;
			*tty->flip.flag_buf_ptr++ = TTY_OVERRUN;
			pr_debug("sci: overrun error\n");
		}
	}
#endif

	if (copied) {
		tty->flip.count += copied;
		tty_flip_buffer_push(tty);
	}

	return copied;
}

static irqreturn_t sci_rx_interrupt(int irq, void *ptr, struct pt_regs *regs)
{
	struct uart_port *port = ptr;

	/* I think sci_receive_chars has to be called irrespective
	 * of whether the I_IXOFF is set, otherwise, how is the interrupt
	 * to be disabled?
	 */
	sci_receive_chars(port, regs);

	return IRQ_HANDLED;
}

static irqreturn_t sci_tx_interrupt(int irq, void *ptr, struct pt_regs *regs)
{
	struct uart_port *port = ptr;

	sci_transmit_chars(port);

	return IRQ_HANDLED;
}

static irqreturn_t sci_er_interrupt(int irq, void *ptr, struct pt_regs *regs)
{
	struct uart_port *port = ptr;

	/* Handle errors */
	if (port->type == PORT_SCI) {
		if (sci_handle_errors(port)) {
			/* discard character in rx buffer */
			sci_in(port, SCxSR);
			sci_out(port, SCxSR, SCxSR_RDxF_CLEAR(port));
		}
	} else {
#if defined(SCIF_ORER)
		if((sci_in(port, SCLSR) & SCIF_ORER) != 0) {
			struct tty_struct *tty = port->info->tty;

			sci_out(port, SCLSR, 0);
			if(tty->flip.count<TTY_FLIPBUF_SIZE) {
				*tty->flip.flag_buf_ptr++ = TTY_OVERRUN;
				tty->flip.count++;
				tty_flip_buffer_push(tty);
				pr_debug("scif: overrun error\n");
			}
		}
#endif
		sci_rx_interrupt(irq, ptr, regs);
	}

	sci_out(port, SCxSR, SCxSR_ERROR_CLEAR(port));

	/* Kick the transmission */
	sci_tx_interrupt(irq, ptr, regs);

	return IRQ_HANDLED;
}

static irqreturn_t sci_br_interrupt(int irq, void *ptr, struct pt_regs *regs)
{
	struct uart_port *port = ptr;

	/* Handle BREAKs */
	sci_handle_breaks(port);
	sci_out(port, SCxSR, SCxSR_BREAK_CLEAR(port));

	return IRQ_HANDLED;
}

static irqreturn_t sci_mpxed_interrupt(int irq, void *ptr, struct pt_regs *regs)
{
        unsigned short ssr_status, scr_status;
        struct uart_port *port = ptr;

        ssr_status = sci_in(port,SCxSR);
        scr_status = sci_in(port,SCSCR);

	/* Tx Interrupt */
        if ((ssr_status&0x0020) && (scr_status&0x0080))
                sci_tx_interrupt(irq, ptr, regs);
	/* Rx Interrupt */
        if ((ssr_status&0x0002) && (scr_status&0x0040))
                sci_rx_interrupt(irq, ptr, regs);
	/* Error Interrupt */
        if ((ssr_status&0x0080) && (scr_status&0x0400))
                sci_er_interrupt(irq, ptr, regs);
	/* Break Interrupt */
        if ((ssr_status&0x0010) && (scr_status&0x0200))
                sci_br_interrupt(irq, ptr, regs);

	return IRQ_HANDLED;
}

#ifdef CONFIG_CPU_FREQ
/*
 * Here we define a transistion notifier so that we can update all of our
 * ports' baud rate when the peripheral clock changes.
 */
static int sci_notifier(struct notifier_block *self, unsigned long phase, void *p)
{
	struct cpufreq_freqs *freqs = p;
	int i;

	if ((phase == CPUFREQ_POSTCHANGE) ||
	    (phase == CPUFREQ_RESUMECHANGE)){
		for (i = 0; i < SCI_NPORTS; i++) {
			struct uart_port *port = &sci_ports[i].port;

			/*
			 * Update the uartclk per-port if frequency has
			 * changed, since it will no longer necessarily be
			 * consistent with the old frequency.
			 *
			 * Really we want to be able to do something like
			 * uart_change_speed() or something along those lines
			 * here to implicitly reset the per-port baud rate..
			 *
			 * Clean this up later..
			 */
			port->uartclk = current_cpu_data.module_clock * 16;
		}

		printk("%s: got a postchange notification for cpu %d (old %d, new %d)\n",
				__FUNCTION__, freqs->cpu, freqs->old, freqs->new);
	}

	return NOTIFY_OK;
}

static struct notifier_block sci_nb = { &sci_notifier, NULL, 0 };
#endif /* CONFIG_CPU_FREQ */

static int sci_request_irq(struct sci_port *port)
{
	int i;
	irqreturn_t (*handlers[4])(int irq, void *ptr, struct pt_regs *regs) = {
		sci_er_interrupt, sci_rx_interrupt, sci_tx_interrupt,
		sci_br_interrupt,
	};
	const char *desc[] = { "SCI Receive Error", "SCI Receive Data Full",
			       "SCI Transmit Data Empty", "SCI Break" };

	if (port->irqs[0] == port->irqs[1]) {
		if (!port->irqs[0]) {
			printk(KERN_ERR "sci: Cannot allocate irq.(IRQ=0)\n");
			return -ENODEV;
		}
		if (request_irq(port->irqs[0], sci_mpxed_interrupt,
#ifdef CONFIG_PREEMPT_HARDIRQS
				0,
#else
				SA_INTERRUPT,
#endif
				"sci", port)) {
			printk(KERN_ERR "sci: Cannot allocate irq.\n");
			return -ENODEV;
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(handlers); i++) {
			if (!port->irqs[i])
				continue;
			if (request_irq(port->irqs[i], handlers[i],
#ifdef CONFIG_PREEMPT_HARDIRQS
					0,
#else
					SA_INTERRUPT,
#endif
					desc[i], port)) {
				printk(KERN_ERR "sci: Cannot allocate irq.\n");
				return -ENODEV;
			}
		}
	}

	return 0;
}

static void sci_free_irq(struct sci_port *port)
{
	int i;

        if (port->irqs[0] == port->irqs[1]) {
                if (!port->irqs[0])
                        printk("sci: sci_free_irq error\n");
		else
                        free_irq(port->irqs[0], port);
        } else {
		for (i = 0; i < ARRAY_SIZE(port->irqs); i++) {
			if (!port->irqs[i])
				continue;

			free_irq(port->irqs[i], port);
		}
	}
}

static unsigned int sci_tx_empty(struct uart_port *port)
{
	/* Can't detect */
	return TIOCSER_TEMT;
}

static void sci_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* This routine is used for seting signals of: DTR, DCD, CTS/RTS */
	/* We use SCIF's hardware for CTS/RTS, so don't need any for that. */
	/* If you have signals for DTR and DCD, please implement here. */
}

static unsigned int sci_get_mctrl(struct uart_port *port)
{
	/* This routine is used for geting signals of: DTR, DCD, DSR, RI,
	   and CTS/RTS */

	return TIOCM_DTR | TIOCM_RTS | TIOCM_DSR;
}

static void sci_start_tx(struct uart_port *port, unsigned int tty_start)
{
	struct sci_port *s = &sci_ports[port->line];

	disable_irq(s->irqs[SCIx_TXI_IRQ]);
	sci_transmit_chars(port);
	enable_irq(s->irqs[SCIx_TXI_IRQ]);
}

static void sci_stop_tx(struct uart_port *port, unsigned int tty_stop)
{
	unsigned long flags;
	unsigned short ctrl;

	/* Clear TIE (Transmit Interrupt Enable) bit in SCSCR */
	local_irq_save(flags);
	ctrl = sci_in(port, SCSCR);
	ctrl &= ~SCI_CTRL_FLAGS_TIE;
	sci_out(port, SCSCR, ctrl);
	local_irq_restore(flags);
}

static void sci_start_rx(struct uart_port *port, unsigned int tty_start)
{
	unsigned long flags;
	unsigned short ctrl;

	/* Set RIE (Receive Interrupt Enable) bit in SCSCR */
	local_irq_save(flags);
	ctrl = sci_in(port, SCSCR);
	ctrl |= SCI_CTRL_FLAGS_RIE | SCI_CTRL_FLAGS_REIE;
	sci_out(port, SCSCR, ctrl);
	local_irq_restore(flags);
}

static void sci_stop_rx(struct uart_port *port)
{
	unsigned long flags;
	unsigned short ctrl;

	/* Clear RIE (Receive Interrupt Enable) bit in SCSCR */
	local_irq_save(flags);
	ctrl = sci_in(port, SCSCR);
	ctrl &= ~(SCI_CTRL_FLAGS_RIE | SCI_CTRL_FLAGS_REIE);
	sci_out(port, SCSCR, ctrl);
	local_irq_restore(flags);
}

static void sci_enable_ms(struct uart_port *port)
{
	/* Nothing here yet .. */
}

static void sci_break_ctl(struct uart_port *port, int break_state)
{
	/* Nothing here yet .. */
}

static int sci_startup(struct uart_port *port)
{
	struct sci_port *s = &sci_ports[port->line];

#if defined(__H8300S__)
	h8300_sci_enable(port, sci_enable);
#endif

	sci_request_irq(s);
	sci_start_tx(port, 1);
	sci_start_rx(port, 1);

	return 0;
}

static void sci_shutdown(struct uart_port *port)
{
	struct sci_port *s = &sci_ports[port->line];

	sci_stop_rx(port);
	sci_stop_tx(port, 1);
	sci_free_irq(s);

#if defined(__H8300S__)
	h8300_sci_enable(port, sci_disable);
#endif
}

static void sci_set_termios(struct uart_port *port, struct termios *termios,
			    struct termios *old)
{
	struct sci_port *s = &sci_ports[port->line];
	unsigned int status, baud, smr_val;
	unsigned long flags;
	int t;

	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);

	spin_lock_irqsave(&port->lock, flags);

	do {
		status = sci_in(port, SCxSR);
	} while (!(status & SCxSR_TEND(port)));

	sci_out(port, SCSCR, 0x00);	/* TE=0, RE=0, CKE1=0 */

#if !defined(SCI_ONLY)
	if (port->type == PORT_SCIF) {
#if defined(CONFIG_CPU_SUBTYPE_SH7300) || defined(CONFIG_CPU_SUBTYPE_SH7705)
	sci_out(port, SCFCR, SCFCR_RFRST | SCFCR_TFRST | SCFCR_TCRST);
#else
		sci_out(port, SCFCR, SCFCR_RFRST | SCFCR_TFRST);
#endif
	}
#endif

	smr_val = sci_in(port, SCSMR) & 3;
	if ((termios->c_cflag & CSIZE) == CS7)
		smr_val |= 0x40;
	if (termios->c_cflag & PARENB)
		smr_val |= 0x20;
	if (termios->c_cflag & PARODD)
		smr_val |= 0x30;
	if (termios->c_cflag & CSTOPB)
		smr_val |= 0x08;

	uart_update_timeout(port, termios->c_cflag, baud);

	sci_out(port, SCSMR, smr_val);

	switch (baud) {
		case 0:		t = -1;		break;
		case 2400:	t = BPS_2400;	break;
		case 4800:	t = BPS_4800;	break;
		case 9600:	t = BPS_9600;	break;
		case 19200:	t = BPS_19200;	break;
		case 38400:	t = BPS_38400;	break;
		case 57600:	t = BPS_57600;	break;
		case 115200:	t = BPS_115200;	break;
		default:	t = SCBRR_VALUE(baud); break;
	}

	if (t > 0) {
		if(t >= 256) {
			sci_out(port, SCSMR, (sci_in(port, SCSMR) & ~3) | 1);
			t >>= 2;
		} else {
			sci_out(port, SCSMR, sci_in(port, SCSMR) & ~3);
		}
		sci_out(port, SCBRR, t);
		udelay((1000000+(baud-1)) / baud); /* Wait one bit interval */
	}

	s->init_pins(port, termios->c_cflag);
	sci_out(port, SCSCR, SCSCR_INIT(port));

	if ((termios->c_cflag & CREAD) != 0)
              sci_start_rx(port,0);

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *sci_type(struct uart_port *port)
{
	switch (port->type) {
		case PORT_SCI:	return "sci";
		case PORT_SCIF:	return "scif";
		case PORT_IRDA: return "irda";
	}

	return 0;
}

static void sci_release_port(struct uart_port *port)
{
	/* Nothing here yet .. */
}

static int sci_request_port(struct uart_port *port)
{
	/* Nothing here yet .. */
	return 0;
}

static void sci_config_port(struct uart_port *port, int flags)
{
	struct sci_port *s = &sci_ports[port->line];

	port->type = s->type;

#if defined(CONFIG_CPU_SUBTYPE_SH5_101) || defined(CONFIG_CPU_SUBTYPE_SH5_103)
	if (port->mapbase == 0)
		port->mapbase = onchip_remap(SCIF_ADDR_SH5, 1024, "SCIF");

	port->membase = (void *)port->mapbase;
#endif
}

static int sci_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	struct sci_port *s = &sci_ports[port->line];

	if (ser->irq != s->irqs[SCIx_TXI_IRQ] || ser->irq > NR_IRQS)
		return -EINVAL;
	if (ser->baud_base < 2400)
		/* No paper tape reader for Mitch.. */
		return -EINVAL;

	return 0;
}

static struct uart_ops sci_uart_ops = {
	.tx_empty	= sci_tx_empty,
	.set_mctrl	= sci_set_mctrl,
	.get_mctrl	= sci_get_mctrl,
	.start_tx	= sci_start_tx,
	.stop_tx	= sci_stop_tx,
	.stop_rx	= sci_stop_rx,
	.enable_ms	= sci_enable_ms,
	.break_ctl	= sci_break_ctl,
	.startup	= sci_startup,
	.shutdown	= sci_shutdown,
	.set_termios	= sci_set_termios,
	.type		= sci_type,
	.release_port	= sci_release_port,
	.request_port	= sci_request_port,
	.config_port	= sci_config_port,
	.verify_port	= sci_verify_port,
};

static struct sci_port sci_ports[SCI_NPORTS] = {
#if defined(CONFIG_CPU_SUBTYPE_SH7705)
	{
		.port   = {
			.membase        = (void *)0xa4400000,
			.mapbase        = 0xa4400000,
			.iotype         = SERIAL_IO_MEM,
			.irq            = 55,
			.ops            = &sci_uart_ops,
			.flags          = ASYNC_BOOT_AUTOCONF,
			.line           = 0,
		},
		.type           = PORT_SCIF,
		.irqs           = SH7705_IRDA_IRQS,
		.init_pins      = sci_init_pins_irda,
	},
		.port   = {
			.membase        = (void *)0xa4410000,
			.mapbase        = 0xa4410000,
			.iotype         = SERIAL_IO_MEM,
			.irq            = 59,
			.ops            = &sci_uart_ops,
			.flags          = ASYNC_BOOT_AUTOCONF,
			.line           = 1,
		},
                .type           = PORT_SCIF,
		.irqs           = SH7705_SCIF_IRQS,
		.init_pins      = sci_init_pins_irda,
	},
#elif defined(CONFIG_CPU_SUBTYPE_SH7708)
	{
		.port   = {
			.membase        = (void *)0xfffffe80,
			.mapbase        = 0xfffffe80,
			.iotype         = SERIAL_IO_MEM,
			.irq            = 25,
			.ops            = &sci_uart_ops,
			.flags          = ASYNC_BOOT_AUTOCONF,
			.line           = 0,
		},
		.type           = PORT_SCI,
		.irqs           = SCI_IRQS,
		.init_pins      = sci_init_pins_sci,
	},
#elif defined(CONFIG_CPU_SUBTYPE_SH7706) || defined(CONFIG_CPU_SUBTYPE_SH7707) || \
      defined(CONFIG_CPU_SUBTYPE_SH7709) || defined(CONFIG_CPU_SUBTYPE_SH7727)
	{
		.port	= {
			.membase	= (void *)0xfffffe80,
			.mapbase	= 0xfffffe80,
			.iotype		= SERIAL_IO_MEM,
			.irq		= 25,
			.ops		= &sci_uart_ops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 0,
		},
		.type		= PORT_SCI,
		.irqs		= SCI_IRQS,
		.init_pins	= sci_init_pins_sci,
	},
	{
		.port	= {
			.membase	= (void *)0xa4000150,
			.mapbase	= 0xa4000150,
			.iotype		= SERIAL_IO_MEM,
			.irq		= 59,
			.ops		= &sci_uart_ops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 1,
		},
		.type		= PORT_SCIF,
		.irqs		= SH3_SCIF_IRQS,
		.init_pins	= sci_init_pins_scif,
	},
#if !defined(CONFIG_CPU_SUBTYPE_SH7706) && !defined(CONFIG_CPU_SUBTYPE_SH7727)
	/* SH7706/SH7727 have only 1 SCIF */

	{
		.port	= {
			.membase	= (void *)0xa4000140,
			.mapbase	= 0xa4000140,
			.iotype		= SERIAL_IO_MEM,
			.irq		= 55,
			.ops		= &sci_uart_ops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 2,
		},
		.type		= PORT_IRDA,
		.irqs		= SH3_IRDA_IRQS,
		.init_pins	= sci_init_pins_irda,
	}
#endif /* !CONFIG_CPU_SUBTYPE_SH7706 && !CONFIG_CPU_SUBTYPE_SH7727 */
#elif defined(CONFIG_CPU_SUBTYPE_SH7710)
	{
		.port   = {
			.membase        = (void *)0xa4400000,
			.mapbase        = 0xa4400000,
			.iotype         = SERIAL_IO_MEM,
			.irq            = 55,
			.ops            = &sci_uart_ops,
			.flags          = ASYNC_BOOT_AUTOCONF,
			.line           = 0,
		},
		.type           = PORT_SCIF,
		.irqs           = SH7710_SCIF0_IRQS,
		.init_pins      = sci_init_pins_irda,
	},
	{
		.port   = {
			.membase        = (void *)0xa4410000,
			.mapbase        = 0xa4410000,
			.iotype         = SERIAL_IO_MEM,
			.irq            = 59,
			.ops            = &sci_uart_ops,
			.flags          = ASYNC_BOOT_AUTOCONF,
			.line           = 1,
		},
		.type           = PORT_SCIF,
		.irqs           = SH7710_SCIF1_IRQS,
		.init_pins      = sci_init_pins_irda,
	},
#elif defined(CONFIG_CPU_SUBTYPE_SH7720)
	{
		.port   = {
			.membase        = (void *)0xa4430000,
			.mapbase        = 0xa4430000,
			.iotype         = SERIAL_IO_MEM,
			.irq            = 80,
			.ops            = &sci_uart_ops,
			.flags          = ASYNC_BOOT_AUTOCONF,
			.line           = 0,
		},
		.type           = PORT_SCIF,
		.irqs           = SH7720_SCIF0_IRQS,
		.init_pins      = sci_init_pins_irda,
	},
	{
		.port   = {
			.membase        = (void *)0xa4438000,
			.mapbase        = 0xa4438000,
			.iotype         = SERIAL_IO_MEM,
			.irq            = 81,
			.ops            = &sci_uart_ops,
			.flags          = ASYNC_BOOT_AUTOCONF,
			.line           = 1,
		},
		.type           = PORT_SCIF,
		.irqs           = SH7720_SCIF1_IRQS,
		.init_pins      = sci_init_pins_irda,
	},

#elif defined(CONFIG_CPU_SUBTYPE_SH7300)
	{
		.port	= {
			.membase	= (void *)0xA4430000,
			.mapbase	= 0xA4430000,
			.iotype		= SERIAL_IO_MEM,
			.irq		= 80,
			.ops		= &sci_uart_ops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 0,
		},
		.type		= PORT_SCIF,
		.irqs		= SH7300_SCIF0_IRQS,
		.init_pins	= sci_init_pins_scif,
	},
#elif defined(CONFIG_CPU_SUBTYPE_SH73180)
	{
		.port	= {
			.membase	= (void *)0xffe00000,
			.mapbase	= 0xffe00000,
			.iotype		= SERIAL_IO_MEM,
			.irq		= 25,
			.ops		= &sci_uart_ops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 0,
		},
		.type		= PORT_SCIF,
		.irqs		= SH73180_SCIF_IRQS,
		.init_pins	= sci_init_pins_scif,
	},
#elif defined(CONFIG_SH_RTS7751R2D)
	{
		.port	= {
			.membase	= (void *)0xffe80000,
			.mapbase	= 0xffe80000,
			.iotype		= SERIAL_IO_MEM,
			.irq		= 43,
			.ops		= &sci_uart_ops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 0,
		},
		.type		= PORT_SCIF,
		.irqs		= SH4_SCIF_IRQS,
		.init_pins	= sci_init_pins_scif,
	},
#elif defined(CONFIG_CPU_SUBTYPE_SH7750) || defined(CONFIG_CPU_SUBTYPE_SH7751)
	{
		.port	= {
			.membase	= (void *)0xffe00000,
			.mapbase	= 0xffe00000,
			.iotype		= SERIAL_IO_MEM,
			.irq		= 25,
			.ops		= &sci_uart_ops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 0,
		},
		.type		= PORT_SCI,
		.irqs		= SCI_IRQS,
		.init_pins	= sci_init_pins_sci,
	},
	{
		.port	= {
			.membase	= (void *)0xffe80000,
			.mapbase	= 0xffe80000,
			.iotype		= SERIAL_IO_MEM,
			.irq		= 43,
			.ops		= &sci_uart_ops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 1,
		},
		.type		= PORT_SCIF,
		.irqs		= SH4_SCIF_IRQS,
		.init_pins	= sci_init_pins_scif,
	},
#elif defined(CONFIG_CPU_SUBTYPE_SH7760)
	{
		.port	= {
			.membase	= (void *)0xfe600000,
			.mapbase	= 0xfe600000,
			.iotype		= SERIAL_IO_MEM,
			.irq		= 55,
			.ops		= &sci_uart_ops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 0,
		},
		.type		= PORT_SCIF,
		.irqs		= SH7760_SCIF0_IRQS,
		.init_pins	= sci_init_pins_scif,
	},
	{
		.port	= {
			.membase	= (void *)0xfe610000,
			.mapbase	= 0xfe610000,
			.iotype		= SERIAL_IO_MEM,
			.irq		= 75,
			.ops		= &sci_uart_ops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 1,
		},
		.type		= PORT_SCIF,
		.irqs		= SH7760_SCIF1_IRQS,
		.init_pins	= sci_init_pins_scif,
	},
	{
		.port	= {
			.membase	= (void *)0xfe620000,
			.mapbase	= 0xfe620000,
			.iotype		= SERIAL_IO_MEM,
			.irq		= 79,
			.ops		= &sci_uart_ops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 2,
		},
		.type		= PORT_SCIF,
		.irqs		= SH7760_SCIF2_IRQS,
		.init_pins	= sci_init_pins_scif,
	},
#elif defined(CONFIG_CPU_SUBTYPE_SH7780)
	{
		.port   = {
			.membase        = (void *)0xffe00000,
			.mapbase        = 0xffe00000,
			.iotype         = SERIAL_IO_MEM,
			.irq            = 43,
			.ops            = &sci_uart_ops,
			.flags          = ASYNC_BOOT_AUTOCONF,
			.line           = 0,
		},
		.type           = PORT_SCIF,
		.irqs           = SH7780_SCIF0_IRQS,
		.init_pins      = sci_init_pins_scif,
	},
	{
		.port   = {
			.membase        = (void *)0xffe10000,
			.mapbase        = 0xffe10000,
			.iotype         = SERIAL_IO_MEM,
			.irq            = 79,
			.ops            = &sci_uart_ops,
			.flags          = ASYNC_BOOT_AUTOCONF,
			.line           = 1,
		},
		.type           = PORT_SCIF,
		.irqs           = SH7780_SCIF1_IRQS,
		.init_pins      = sci_init_pins_scif,
	},
#elif defined(CONFIG_CPU_SUBTYPE_SH4_202)
	{
		.port	= {
			.membase	= (void *)0xffe80000,
			.mapbase	= 0xffe80000,
			.iotype		= SERIAL_IO_MEM,
			.irq		= 43,
			.ops		= &sci_uart_ops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 0,
		},
		.type		= PORT_SCIF,
		.irqs		= SH4_SCIF_IRQS,
		.init_pins	= sci_init_pins_scif,
	},
#elif defined(CONFIG_CPU_SUBTYPE_ST40STB1)
	{
		.port	= {
			.membase	= (void *)0xffe00000,
			.mapbase	= 0xffe00000,
			.iotype		= SERIAL_IO_MEM,
			.irq		= 26,
			.ops		= &sci_uart_ops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 0,
		},
		.type		= PORT_SCIF,
		.irqs		= STB1_SCIF1_IRQS,
		.init_pins	= sci_init_pins_scif,
	},
	{
		.port	= {
			.membase	= (void *)0xffe80000,
			.mapbase	= 0xffe80000,
			.iotype		= SERIAL_IO_MEM,
			.irq		= 43,
			.ops		= &sci_uart_ops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 1,
		},
		.type		= PORT_SCIF,
		.irqs		= SH4_SCIF_IRQS,
		.init_pins	= sci_init_pins_scif,
	},
#elif defined(CONFIG_CPU_SUBTYPE_SH5_101) || defined(CONFIG_CPU_SUBTYPE_SH5_103)
	{
		.port	= {
			.iotype		= SERIAL_IO_MEM,
			.irq		= 42,
			.ops		= &sci_uart_ops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 0,
		},
		.type		= PORT_SCIF,
		.irqs		= SH5_SCIF_IRQS,
		.init_pins	= sci_init_pins_scif,
	},
#elif defined(CONFIG_H83007) || defined(CONFIG_H83068)
	{
		.port	= {
			.membase	= (void *)0x00ffffb0,
			.mapbase	= 0x00ffffb0,
			.iotype		= SERIAL_IO_MEM,
			.irq		= 54,
			.ops		= &sci_uart_ops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 0,
		},
		.type		= PORT_SCI,
		.irqs		= H8300H_SCI_IRQS0,
		.init_pins	= sci_init_pins_sci,
	},
	{
		.port	= {
			.membase	= (void *)0x00ffffb8,
			.mapbase	= 0x00ffffb8,
			.iotype		= SERIAL_IO_MEM,
			.irq		= 58,
			.ops		= &sci_uart_ops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 1,
		},
		.type		= PORT_SCI,
		.irqs		= H8300H_SCI_IRQS1,
		.init_pins	= sci_init_pins_sci,
	},
	{
		.port	= {
			.membase	= (void *)0x00ffffc0,
			.mapbase	= 0x00ffffc0,
			.iotype		= SERIAL_IO_MEM,
			.irq		= 62,
			.ops		= &sci_uart_ops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 2,
		},
		.type		= PORT_SCI,
		.irqs		= H8300H_SCI_IRQS2,
		.init_pins	= sci_init_pins_sci,
	},
#elif defined(CONFIG_H8S2678)
	{
		.port	= {
			.membase	= (void *)0x00ffff78,
			.mapbase	= 0x00ffff78,
			.iotype		= SERIAL_IO_MEM,
			.irq		= 90,
			.ops		= &sci_uart_ops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 0,
		},
		.type		= PORT_SCI,
		.irqs		= H8S_SCI_IRQS0,
		.init_pins	= sci_init_pins_sci,
	},
	{
		.port	= {
			.membase	= (void *)0x00ffff80,
			.mapbase	= 0x00ffff80,
			.iotype		= SERIAL_IO_MEM,
			.irq		= 94,
			.ops		= &sci_uart_ops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 1,
		},
		.type		= PORT_SCI,
		.irqs		= H8S_SCI_IRQS1,
		.init_pins	= sci_init_pins_sci,
	},
	{
		.port	= {
			.membase	= (void *)0x00ffff88,
			.mapbase	= 0x00ffff88,
			.iotype		= SERIAL_IO_MEM,
			.irq		= 98,
			.ops		= &sci_uart_ops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 2,
		},
		.type		= PORT_SCI,
		.irqs		= H8S_SCI_IRQS2,
		.init_pins	= sci_init_pins_sci,
	},
#else
#error "CPU subtype not defined"
#endif
};

#ifdef CONFIG_SERIAL_SH_SCI_CONSOLE
/*
 *	Print a string to the serial port trying not to disturb
 *	any possible real use of the port...
 */
static void serial_console_write(struct console *co, const char *s,
				 unsigned count)
{
	put_string(serial_console_port, s, count);
}

static int __init serial_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	int ret;

	if (co->index >= SCI_NPORTS)
		co->index = 0;

	serial_console_port = &sci_ports[co->index];
	port = &serial_console_port->port;
	port->type = serial_console_port->type;

#ifdef CONFIG_SUPERH64
	/* This is especially needed on sh64 to remap the SCIF */
	sci_config_port(port, 0);
#endif

	/*
	 * We need to set the initial uartclk here, since otherwise it will
	 * only ever be setup at sci_init() time.
	 */
#if !defined(__H8300H__) && !defined(__H8300S__)
	port->uartclk = current_cpu_data.module_clock * 16;
#else
	port->uartclk = CONFIG_CPU_CLOCK;
#endif
#if defined(__H8300S__)
	h8300_sci_enable(port, sci_enable);
#endif
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	ret = uart_set_options(port, co, baud, parity, bits, flow);
#if defined(__H8300H__) || defined(__H8300S__)
	/* disable rx interrupt */
	if (ret == 0)
		sci_stop_rx(port);
#endif
	return ret;
}

static struct console serial_console = {
	.name		= "ttySC",
	.device		= uart_console_device,
	.write		= serial_console_write,
	.setup		= serial_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &sci_uart_driver,
};

static int __init sci_console_init(void)
{
	register_console(&serial_console);
	return 0;
}

console_initcall(sci_console_init);
#endif /* CONFIG_SERIAL_SH_SCI_CONSOLE */

#if 0
#ifdef CONFIG_SH_KGDB
/*
 * FIXME: Most of this can go away.. at the moment, we rely on
 * arch/sh/kernel/setup.c to do the command line parsing for kgdb, though
 * most of that can easily be done here instead.
 *
 * For the time being, just accept the values that were parsed earlier..
 */
static void __init kgdb_console_get_options(struct uart_port *port, int *baud,
					    int *parity, int *bits)
{
	*baud = kgdb_baud;
	*parity = tolower(kgdb_parity);
	*bits = kgdb_bits - '0';
}

/*
 * The naming here is somewhat misleading, since kgdb_console_setup() takes
 * care of the early-on initialization for kgdb, regardless of whether we
 * actually use kgdb as a console or not.
 *
 * On the plus side, this lets us kill off the old kgdb_sci_setup() nonsense.
 */
int __init kgdb_console_setup(struct console *co, char *options)
{
	struct uart_port *port = &sci_ports[kgdb_portnum].port;
	int baud = 38400;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index >= SCI_NPORTS || co->index != kgdb_portnum)
		co->index = kgdb_portnum;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		kgdb_console_get_options(port, &baud, &parity, &bits);

	kgdb_getchar = kgdb_sci_getchar;
	kgdb_putchar = kgdb_sci_putchar;

	return uart_set_options(port, co, baud, parity, bits, flow);
}
#endif /* CONFIG_SH_KGDB */
#endif /* 0 */

#ifdef CONFIG_SERIAL_SH_SCI_CONSOLE
#define SCI_CONSOLE	&serial_console
#else
#define SCI_CONSOLE 	0
#endif

static char banner[] __initdata =
	KERN_INFO "SuperH SCI(F) driver initialized\n";

static struct uart_driver sci_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "sci",
#ifdef CONFIG_DEVFS_FS
	.devfs_name	= "ttsc/",
#endif
	.dev_name	= "ttySC",
	.major		= SCI_MAJOR,
	.minor		= SCI_MINOR_START,
	.nr		= SCI_NPORTS,
	.cons		= SCI_CONSOLE,
};

static int __init sci_init(void)
{
	int chan, ret;

	printk("%s", banner);

	ret = uart_register_driver(&sci_uart_driver);
	if (ret == 0) {
		for (chan = 0; chan < SCI_NPORTS; chan++) {
			struct sci_port *sciport = &sci_ports[chan];

#if !defined(__H8300H__) && !defined(__H8300S__)
			sciport->port.uartclk = (current_cpu_data.module_clock * 16);
#else
			sciport->port.uartclk = CONFIG_CPU_CLOCK;
#endif
			uart_add_one_port(&sci_uart_driver, &sciport->port);
			sciport->break_timer.data = (unsigned long)sciport;
			sciport->break_timer.function = sci_break_timer;
			init_timer(&sciport->break_timer);
		}
	}

#ifdef CONFIG_CPU_FREQ
	cpufreq_register_notifier(&sci_nb, CPUFREQ_TRANSITION_NOTIFIER);
	printk("sci: CPU frequency notifier registered\n");
#endif

#ifdef CONFIG_SH_STANDARD_BIOS
	sh_bios_gdb_detach();
#endif

	return ret;
}

static void __exit sci_exit(void)
{
	int chan;

	for (chan = 0; chan < SCI_NPORTS; chan++)
		uart_remove_one_port(&sci_uart_driver, &sci_ports[chan].port);

	uart_unregister_driver(&sci_uart_driver);
}

module_init(sci_init);
module_exit(sci_exit);
