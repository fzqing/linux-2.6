/*
 * drivers/char/xilinx_uartlite/xuartlite_serial.c
 *
 * Xilinx UART Lite driver
 *
 * Author: MontaVista Software, Inc.
 *         <source@mvista.com>
 *
 * Based on the serial_tx3912.c, (C) 2001 Steven J. Hill
 *         (sjhill@realitydiluted.com)
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/mm.h>
#include <linux/serialP.h>
#include <linux/generic_serial.h>
#include <linux/tty_flip.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/console.h>

#include "xuartlite.h"
#include "xuartlite_i.h"
#include "xuartlite_serial.h"

MODULE_AUTHOR("MontaVista Software, Inc. <source@mvista.com>");
MODULE_DESCRIPTION("Xilinx System UART Lite driver");
MODULE_LICENSE("GPL");

/*
 * Forward declarations for serial routines
 */
static void xuli_disable_tx_interrupts(void *ptr);
static void xuli_enable_tx_interrupts(void *ptr);
static void xuli_disable_rx_interrupts(void *ptr);
static void xuli_enable_rx_interrupts(void *ptr);
static int xuli_get_CD(void *ptr);
static void xuli_shutdown_port(void *ptr);
static int xuli_set_real_termios(void *ptr);
static int xuli_chars_in_buffer(void *ptr);
static void xuli_hungup(void *ptr);
static void xuli_close(void *ptr);
static void xuli_getserial(void *, struct serial_struct *sp);

/*
 * Used by generic serial driver to access hardware
 */
static struct real_driver xuli_real_driver = {
	.disable_tx_interrupts = xuli_disable_tx_interrupts,
	.enable_tx_interrupts = xuli_enable_tx_interrupts,
	.disable_rx_interrupts = xuli_disable_rx_interrupts,
	.enable_rx_interrupts = xuli_enable_rx_interrupts,
	.get_CD = xuli_get_CD,
	.shutdown_port = xuli_shutdown_port,
	.set_real_termios = xuli_set_real_termios,
	.chars_in_buffer = xuli_chars_in_buffer,
	.close = xuli_close,
	.hungup = xuli_hungup,
	.getserial = xuli_getserial,
};

/*
 * Forward declarations for console routines
 */

#ifdef CONFIG_XILINX_UARTLITE_CONSOLE

static void serial_console_write(struct console *co, const char *s,
				 unsigned count);
static struct tty_driver *serial_console_device(struct console *c, int *index);
static int serial_console_setup(struct console *co, char *options);

static struct console sercons = {
	.name = UARTLITE_TTY_NAME,
	.write = serial_console_write,
	.device = serial_console_device,
	.setup = serial_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1
};
#endif				/* #ifdef CONFIG_XILINX_UARTLITE_CONSOLE */

/*
 * Structures and usage counts
 */
static struct xs_port {
	/* generic_serial data */
	struct gs_port gs_data;
	/* Xilinx low level stuff */
	XUartLite x_uart_lite;
	u32 saved_baseaddr;
	/* receiption stuff */
	int rx_enabled;
	u8 rx_buf;		/* having more than 1 byte makes little sense */
	/* transmission stuff */
	int tx_started;
	struct timer_list tx_timer;
	unsigned long tx_timeout;
	int stop_tx_timer;
	/* used by throttle/unthrottle */
	u8 thr_chars[2];
	/* initialization related */
	int init_level;		/* 0 means "not initialized at all" */
	/* some counters for statistics */
	long rx_poll_cnt;
	long rx_int_cnt;
	long rx_lost_cnt;
	long tx_int_cnt;
	long tx_tmr_cnt;
	long tx_thr_cnt;	/* the number of XON/XOFFs sent */
	long tx_thr_rqs;	/* how many times ??? was called */
} xs_data[XPAR_XUARTLITE_NUM_INSTANCES];

static struct tty_driver xuli_driver;
static struct termios *xuli_termios[XPAR_XUARTLITE_NUM_INSTANCES];
static struct termios *xuli_termios_locked[XPAR_XUARTLITE_NUM_INSTANCES];

static int xuli_ports;

#define TX_BLOCK_SIZE    1

/*
 * Xilinx configuration stuff
 */

/* SAATODO: This function will be moved into the Xilinx code. */
XUartLite_Config *XUartLite_GetConfig(int Instance)
{
	if (Instance < 0 || Instance >= XPAR_XUARTLITE_NUM_INSTANCES) {
		return NULL;
	}

	return &XUartLite_ConfigTable[Instance];
}

/*
 * Constructs termios.c_cflag value based on the UART Lite configuration.
 * Most of the code taken from serial.c.
 */
static int xulite_get_cflag(int Instance)
{
	int cflag = CREAD | HUPCL | CLOCAL;
	XUartLite_Config *cfg;

	cfg = XUartLite_GetConfig(Instance);
	if (cfg == NULL) {
		return (B9600 | CS8 | CREAD | HUPCL | CLOCAL);
	} else {
		switch (cfg->BaudRate) {
		case 1200:
			cflag |= B1200;
			break;
		case 2400:
			cflag |= B2400;
			break;
		case 4800:
			cflag |= B4800;
			break;
		case 19200:
			cflag |= B19200;
			break;
		case 38400:
			cflag |= B38400;
			break;
		case 57600:
			cflag |= B57600;
			break;
		case 115200:
			cflag |= B115200;
			break;
		case 9600:
		default:
			cflag |= B9600;
			break;
		}
		switch (cfg->DataBits) {
		case 7:
			cflag |= CS7;
			break;
		case 8:
		default:
			cflag |= CS8;
			break;
		}
		if (cfg->UseParity) {
			if (cfg->ParityOdd) {
				cflag |= PARODD;
			} else {
				cflag |= PARENB;
			}
		}
	}

	return cflag;
}

/* As XUartLite_GetConfig() can't tell us what IRQ is used by
 * each UART Lite instance, we have to refer xparameters.h directly.*/
static const int xuli_irq[XPAR_XUARTLITE_NUM_INSTANCES] = {
	XPAR_INTC_0_UARTLITE_0_VEC_ID,
#ifdef XPAR_UARTLITE_1_BASEADDR
	XPAR_INTC_0_UARTLITE_1_VEC_ID,
#ifdef XPAR_UARTLITE_2_BASEADDR
	XPAR_INTC_0_UARTLITE_2_VEC_ID,
#ifdef XPAR_UARTLITE_3_BASEADDR
	XPAR_INTC_0_UARTLITE_3_VEC_ID,
#ifdef XPAR_UARTLITE_4_BASEADDR
#error Edit this file to add more devices.
#endif				/* 4 */
#endif				/* 3 */
#endif				/* 2 */
#endif				/* 1 */
};

/*
 * Once cleared in the ISR, the Tx interrupt will not become active
 * until some data is transmitted again. So when the transmitter
 * becomes idle, we must always send some data from outside ISR to
 * start the transmission.
 */
static void xulite_start_tx(int pnum)
{
	struct xs_port *pxs = &xs_data[pnum];
	struct gs_port *pgs = &pxs->gs_data;
	int tx_size;

	if (pxs->tx_started != 0)
		return;

	if (pxs->thr_chars[1] != 0) {
		//if(pxs->thr_chars[0] == 0) { /* this must always be the case */
		pxs->thr_chars[0] = pxs->thr_chars[1];
		pxs->thr_chars[1] = 0;
		//}
		pxs->tx_started = 1;	/* Tx interrupt handler will be invoked */
		XUartLite_Send(&pxs->x_uart_lite, pxs->thr_chars, 1);
		mod_timer(&pxs->tx_timer, jiffies + pxs->tx_timeout);
		return;
	}

	if (pgs->xmit_cnt == 0 || pgs->tty->stopped || pgs->tty->hw_stopped) {
		/* nothing to send */
		pgs->flags &= ~GS_TX_INTEN;	/* this will make gs_write()
						   to call enable_tx_interrupts() */
		/* stop transmission */
		XUartLite_Send(&pxs->x_uart_lite, pgs->xmit_buf, 0);
	} else {
		tx_size = pgs->xmit_cnt;
		if (tx_size > TX_BLOCK_SIZE)
			tx_size = TX_BLOCK_SIZE;
		if (pgs->xmit_tail + tx_size > SERIAL_XMIT_SIZE) {
			tx_size = SERIAL_XMIT_SIZE - pgs->xmit_tail;
		}
		XUartLite_Send(&pxs->x_uart_lite,
			       pgs->xmit_buf + pgs->xmit_tail, tx_size);
		mod_timer(&pxs->tx_timer, jiffies + pxs->tx_timeout);
		pxs->tx_started = 1;	/* Tx interrupt handler will be invoked */
	}

	if (pgs->xmit_cnt <= (pgs->wakeup_chars + TX_BLOCK_SIZE)) {
		if ((pgs->tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
		    pgs->tty->ldisc.write_wakeup) {
			(pgs->tty->ldisc.write_wakeup) (pgs->tty);
		}
		wake_up_interruptible(&pgs->tty->write_wait);
	}
}

static void xulite_recv_bytes(int pnum)
{
	struct xs_port *pxs = &xs_data[pnum];
	struct gs_port *pgs = &pxs->gs_data;
	struct tty_struct *tty = pgs->tty;

	if (pxs->rx_enabled == 0) {
		/* empty the Rx FIFO to clear the Rx interrupt */
		while (XUartLite_Recv(&pxs->x_uart_lite, &pxs->rx_buf, 1) ==
		       1) ;
		return;
	}

	if (XUartLite_Recv(&pxs->x_uart_lite, &pxs->rx_buf, 1) == 0)
		return;

	do {
		/* check if there is space in the buffer available */
		if (tty->flip.count >= TTY_FLIPBUF_SIZE) {
			tty_flip_buffer_push(tty);
		}

		++pxs->rx_poll_cnt;
		tty_insert_flip_char(tty, pxs->rx_buf, TTY_NORMAL);
	} while (XUartLite_Recv(&pxs->x_uart_lite, &pxs->rx_buf, 1) == 1);
	tty_flip_buffer_push(tty);
}

/*
 * Here are the routines that actually interface with the generic driver
 */

static void xuli_getserial(void *ptr, struct serial_struct *sp)
{
	/* some applications (busybox, dbootstrap, etc.) look this */
	sp->line = FIND_PNUM(ptr);
}

static void xuli_disable_tx_interrupts(void *ptr)
{
	func_enter();

	/*
	 * Xilinx UART Lite Tx interrupt must not be disabled as it is active
	 * only for a moment when the last byte goes out of the TxFIFO.
	 */
	return;
}

static void xuli_enable_tx_interrupts(void *ptr)
{
	int pnum = FIND_PNUM(ptr);

	func_enter();
	disable_irq(xuli_irq[pnum]);	/* dimka --- added by me */

	xulite_recv_bytes(pnum);	/* to make sure Rx interrupt is clear */
	xulite_start_tx(pnum);	/* this is necessary to start transmission */

	enable_irq(xuli_irq[pnum]);

	func_exit();
}

static void xuli_disable_rx_interrupts(void *ptr)
{
	int pnum = FIND_PNUM(ptr);
	struct xs_port *pxs = &xs_data[pnum];

	func_enter();

	disable_irq(xuli_irq[pnum]);	/* dimka --- added by me */
	pxs->rx_enabled = 0;

	/* empty the Rx FIFO to clear the Rx interrupt */
	while (XUartLite_Recv(&pxs->x_uart_lite, &pxs->rx_buf, 1) == 1) ;
	xulite_start_tx(pnum);	/* not to loose the Tx interrupt */

	enable_irq(xuli_irq[pnum]);

	func_exit();
}

static void xuli_enable_rx_interrupts(void *ptr)
{
	int pnum = FIND_PNUM(ptr);
	func_enter();

	disable_irq(xuli_irq[pnum]);
	xs_data[pnum].rx_enabled = 1;
	xulite_recv_bytes(pnum);
	xulite_start_tx(pnum);	/* not to loose the Tx interrupt */
	enable_irq(xuli_irq[pnum]);
	func_exit();
}

/*
 * This function is used to send a high-priority XON/XOFF character to
 * the device
 */
static void send_xchar(int pnum, char ch)
{
	disable_irq(xuli_irq[pnum]);
	xs_data[pnum].thr_chars[1] = ch;
	if (ch) {
		++xs_data[pnum].tx_thr_rqs;
		xulite_start_tx(pnum);
	}
	enable_irq(xuli_irq[pnum]);
}

/*
 * This function is called by the upper-layer tty layer to signal that
 * incoming characters should be throttled.
 */
static void xuli_throttle(struct tty_struct *tty)
{
	int pnum = FIND_PNUM((tty->driver_data));

	if (I_IXOFF(tty))
		send_xchar(pnum, STOP_CHAR(tty));
}

/*
 * The counterpart of the previous one.
 */
static void xuli_unthrottle(struct tty_struct *tty)
{
	int pnum = FIND_PNUM((tty->driver_data));

	if (I_IXOFF(tty))
		send_xchar(pnum, START_CHAR(tty));
}

/*
 * We have no CD
 */
static int xuli_get_CD(void *ptr)
{
	return 1;
}

/*
 * Shut down the port
 */
static void xuli_shutdown_port(void *ptr)
{
	func_enter();
	((struct gs_port *)ptr)->flags &= ~GS_ACTIVE;
	func_exit();
}

static int xuli_set_real_termios(void *ptr)
{
	return 0;
}

/*
 * Anyone in the buffer?
 */
static int xuli_chars_in_buffer(void *ptr)
{
	int pnum = FIND_PNUM(ptr);
	return (XUartLite_IsSending(&xs_data[pnum].x_uart_lite) ? 1 : 0);
}

/*
 * === Interrupt handling stuff ===
 */

/*
 * Simple function that hands an interrupt to the Xilinx code.
 * dev_id contains a pointer to the XUartLite instance.
 */
static irqreturn_t xulite_isr(int irq, void *dev_id, struct pt_regs *regs)
{
	XUartLite_InterruptHandler((XUartLite *) dev_id);
	return IRQ_HANDLED;
}

/*
 * Transmission timeout handler (called in case Tx interrupt is
 * lost).
 */
static void xulite_timer_proc(unsigned long pnum)
{
	struct xs_port *pxs = &xs_data[pnum];

	if (pxs->stop_tx_timer)
		return;

	disable_irq(xuli_irq[pnum]);
	if (XUartLite_mIsIntrEnabled(pxs->x_uart_lite.RegBaseAddress)) {
		++pxs->tx_tmr_cnt;
		XUartLite_InterruptHandler(&pxs->x_uart_lite);
	}
	enable_irq(xuli_irq[pnum]);
}

/*
 * Send callback function (called from xulite_isr).
 * Reads the data to transmit right from the gs_port buffer.
 */
static void xulite_send_handler(void *CallBackRef, unsigned int ByteCount)
{
	//int pnum = (int) CallBackRef; /* CallBackRef contains the 0-based index */
	struct xs_port *pxs = CallBackRef;
	struct gs_port *pgs = &pxs->gs_data;
	int tx_size;
	struct tty_struct *tty = pxs->gs_data.tty;
	int pnum = FIND_PNUM(tty->driver_data);

	if (ByteCount == 0)
		return;

	disable_irq(xuli_irq[pnum]);

	/* ByteCount contains the number of bytes sent from the previous
	   XUartLite_Send() call */

	/*
	 * if(pxs->thr_chars[0] != 0) then
	 *     throttle/unthrottle character was just sent;
	 *     do not advance pgs->xmit_tail by ByteCount
	 *     in this case.
	 */
	if (pxs->thr_chars[0] == 0) {
		pxs->tx_int_cnt += ByteCount;

		if (pgs->xmit_cnt >= ByteCount) {
			pgs->xmit_cnt -= ByteCount;
			pgs->xmit_tail += ByteCount;
		} else {
			pgs->xmit_tail += pgs->xmit_cnt;
			pgs->xmit_cnt = 0;
		}
		if (pgs->xmit_tail >= SERIAL_XMIT_SIZE)
			pgs->xmit_tail -= SERIAL_XMIT_SIZE;
	} else {
		pxs->tx_thr_cnt += ByteCount;
	}
	/* throttle/unthrottle stuff: queue a byte for transmission (if any),
	 * indicate that there is a room for the next XON/XOFF */
	pxs->thr_chars[0] = pxs->thr_chars[1];
	pxs->thr_chars[1] = 0;	/* thr_chars[1] is empty */

	/* send next XON/XOFF byte (if any) */
	if (pxs->thr_chars[0] != 0) {
		XUartLite_Send(&pxs->x_uart_lite, pxs->thr_chars, 1);
		mod_timer(&pxs->tx_timer, jiffies + pxs->tx_timeout);
		enable_irq(xuli_irq[pnum]);
		return;
	}

	if (pgs->xmit_cnt == 0 || pgs->tty->stopped || pgs->tty->hw_stopped) {
		/* nothing to send */
		pgs->flags &= ~GS_TX_INTEN;	/* this will make gs_write() to
						   call enable_tx_interrupts() */
		/* stop transmission */
		XUartLite_Send(&pxs->x_uart_lite, pgs->xmit_buf, 0);

		pxs->tx_started = 0;
	} else {
		tx_size = pgs->xmit_cnt;
		if (tx_size > TX_BLOCK_SIZE)
			tx_size = TX_BLOCK_SIZE;
		if (pgs->xmit_tail + tx_size > SERIAL_XMIT_SIZE) {
			tx_size = SERIAL_XMIT_SIZE - pgs->xmit_tail;
		}
		XUartLite_Send(&pxs->x_uart_lite,
			       pgs->xmit_buf + pgs->xmit_tail, tx_size);
		mod_timer(&pxs->tx_timer, jiffies + pxs->tx_timeout);
	}

	if (pgs->xmit_cnt <= (pgs->wakeup_chars + TX_BLOCK_SIZE)) {
		if ((pgs->tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
		    pgs->tty->ldisc.write_wakeup) {
			(pgs->tty->ldisc.write_wakeup) (pgs->tty);
		}
		wake_up_interruptible(&pgs->tty->write_wait);
	}

	enable_irq(xuli_irq[pnum]);
}

/* Receive callback function (called from xulite_isr) */
static void xulite_recv_handler(void *CallBackRef, unsigned int ByteCount)
{
	struct xs_port *pxs = CallBackRef;
	struct tty_struct *tty = pxs->gs_data.tty;
	int pnum = FIND_PNUM(tty->driver_data);

	disable_irq(xuli_irq[pnum]);

	if (pxs->rx_enabled == 0) {
		while (XUartLite_Recv(&pxs->x_uart_lite, &pxs->rx_buf, 1) ==
		       1) ;
	} else {
		do {

			/* check if there is space in the buffer available */
			if (tty->flip.count >= TTY_FLIPBUF_SIZE) {
				tty_flip_buffer_push(tty);
			}

			++pxs->rx_int_cnt;
			tty_insert_flip_char(tty, pxs->rx_buf, TTY_NORMAL);
		} while (XUartLite_Recv(&pxs->x_uart_lite, &pxs->rx_buf, 1) ==
			 1);

		tty_flip_buffer_push(tty);
	}

	enable_irq(xuli_irq[pnum]);
}

/*
 * Open the serial port
 */
static int xuli_open(struct tty_struct *tty, struct file *filp)
{
	int retval;
	int pnum;
	struct xs_port *pxs;
	struct gs_port *pgs;

	func_enter();

	pnum = tty->index;
	if (pnum < 0 || pnum >= xuli_ports) {
		return -ENODEV;
	}
	pxs = &xs_data[pnum];

	if (pxs->init_level == 0) {
		return -EIO;
	}

	pgs = &pxs->gs_data;
	tty->driver_data = pgs;
	tty->disc_data = pxs;
	pgs->tty = tty;
	pgs->count++;

	if (pgs->count == 1) {
		pxs->rx_poll_cnt = 0;
		pxs->rx_int_cnt = 0;
		pxs->rx_lost_cnt = 0;
		pxs->tx_int_cnt = 0;
		pxs->tx_tmr_cnt = 0;
		pxs->tx_thr_cnt = 0;
		pxs->tx_thr_rqs = 0;
	}

	/*
	 * Start up serial port
	 */
	retval = gs_init_port(pgs);
	if (retval) {
		pgs->count--;
		return retval;
	}

	pgs->flags |= GS_ACTIVE;

	xuli_set_real_termios(pgs);

	retval = gs_block_til_ready(pgs, filp);
	if (retval) {
		pgs->count--;
		return retval;
	}

	if (pgs->count == 1) {
		xuli_set_real_termios(pgs);
	}
#ifdef CONFIG_XILINX_UARTLITE_CONSOLE
	if (sercons.cflag != 0 && sercons.index == pnum) {
		tty->termios->c_cflag = sercons.cflag;
		sercons.cflag = 0;
		xuli_set_real_termios(pgs);
	}
#endif

	if (pgs->count == 1) {
		/* initialize timer */
		init_timer(&pxs->tx_timer);
		pxs->tx_timer.data = pnum;
		pxs->tx_timer.function = xulite_timer_proc;
		pxs->stop_tx_timer = 0;

		/* enable the interrupt (Rx/Tx combined interrupt) */
		XUartLite_ResetFifos(&pxs->x_uart_lite);
		XUartLite_Send(&pxs->x_uart_lite, pgs->xmit_buf, 0);	/* stop transmission */
		pxs->rx_enabled = 1;
		pxs->tx_started = 0;
		XUartLite_EnableInterrupt(&pxs->x_uart_lite);
	}

	func_exit();

	return 0;
}

/*
 * Close the serial port
 */
static void xuli_close(void *ptr)
{
	int pnum;
	struct xs_port *pxs;

	func_enter();

	pnum = FIND_PNUM(ptr);
	if (pnum < 0 || pnum >= xuli_ports) {
		return;
	}
	pxs = &xs_data[pnum];
#if ((DEBUG_MASK & DEBUG_STAT) != 0)
	printk("close: rx_poll_cnt=%ld\n", pxs->rx_poll_cnt);
	printk("close: rx_int_cnt=%ld\n", pxs->rx_int_cnt);
	printk("close: rx_lost_cnt=%ld\n", pxs->rx_lost_cnt);
	printk("close: tx_int_cnt=%ld\n", pxs->tx_int_cnt);
	printk("close: tx_tmr_cnt=%ld\n", pxs->tx_tmr_cnt);
	printk("close: tx_thr_cnt=%ld\n", pxs->tx_thr_cnt);
	printk("close: tx_thr_rqs=%ld\n", pxs->tx_thr_rqs);
#endif

	pxs->stop_tx_timer = 1;
	del_timer_sync(&pxs->tx_timer);
	XUartLite_DisableInterrupt(&pxs->x_uart_lite);
	func_exit();
}

static void xuli_flush_buffer(struct tty_struct *tty)
{
	struct gs_port *port;
	struct xs_port *pxs;

	func_enter();

	if (!tty)
		return;

	port = tty->driver_data;
	if (!port)
		return;

	pxs = tty->disc_data;
	if (!pxs)
		return;
	del_timer_sync(&pxs->tx_timer);
	XUartLite_DisableInterrupt(&pxs->x_uart_lite);
	gs_flush_buffer(tty);
	XUartLite_ResetFifos(&pxs->x_uart_lite);
	XUartLite_Send(&pxs->x_uart_lite, port->xmit_buf, 0);	/* stop transmission */
	pxs->rx_enabled = 1;
	pxs->tx_started = 0;
	XUartLite_EnableInterrupt(&pxs->x_uart_lite);
}

/*
 * Hang up the serial port
 */
static void xuli_hungup(void *ptr)
{
	int pnum;
	struct xs_port *pxs;

	func_enter();

	pnum = FIND_PNUM(ptr);
	if (pnum < 0 || pnum >= xuli_ports) {
		return;
	}
	pxs = &xs_data[pnum];
#if ((DEBUG_MASK & DEBUG_STAT) != 0)
	printk("hungup: rx_poll_cnt=%ld\n", pxs->rx_poll_cnt);
	printk("hungup: rx_int_cnt=%ld\n", pxs->rx_int_cnt);
	printk("hungup: rx_lost_cnt=%ld\n", pxs->rx_lost_cnt);
	printk("hungup: tx_int_cnt=%ld\n", pxs->tx_int_cnt);
	printk("hungup: tx_tmr_cnt=%ld\n", pxs->tx_tmr_cnt);
	printk("hungup: tx_thr_cnt=%ld\n", pxs->tx_thr_cnt);
	printk("hungup: tx_thr_rqs=%ld\n", pxs->tx_thr_rqs);
#endif

	pxs->stop_tx_timer = 1;
	del_timer_sync(&pxs->tx_timer);
	XUartLite_DisableInterrupt(&pxs->x_uart_lite);
	func_exit();
}

/*
 * Serial ioctl call
 */
static int
xuli_ioctl(struct tty_struct *tty, struct file *filp,
	   unsigned int cmd, unsigned long arg)
{
	int ival, rc;

	rc = 0;
	switch (cmd) {
	case TIOCGSOFTCAR:
		rc = put_user((tty->termios->c_cflag & CLOCAL) ? 1 : 0,
			      (unsigned int *)arg);
		break;
	case TIOCSSOFTCAR:
		rc = verify_area(VERIFY_READ, (void *)arg, sizeof(int));
		if (rc == 0) {
			get_user(ival, (unsigned int *)arg);
			tty->termios->c_cflag =
			    (tty->termios->c_cflag & ~CLOCAL) |
			    (ival ? CLOCAL : 0);
		}
		break;
	case TIOCGSERIAL:
		rc = verify_area(VERIFY_READ, (void *)arg,
				 sizeof(struct serial_struct));
		if (rc == 0) {
			rc = gs_getserial((struct gs_port *)(tty->driver_data),
					  (struct serial_struct *)arg);
		}
		break;
	case TIOCSSERIAL:
		rc = verify_area(VERIFY_READ, (void *)arg,
				 sizeof(struct serial_struct));
		if (rc == 0) {
			rc = gs_setserial((struct gs_port *)(tty->driver_data),
					  (struct serial_struct *)arg);
		}
		break;
	default:
		rc = -ENOIOCTLCMD;
		break;
	}

	return rc;
}

static void cleanup(int instance)
{
	XUartLite_Config *cfg;

	if (instance < 0 || instance >= XPAR_XUARTLITE_NUM_INSTANCES)
		return;

	cfg = XUartLite_GetConfig(instance);

	disable_irq(xuli_irq[instance]);

	switch (xs_data[instance].init_level) {
	case 3:
		if (instance == 0) {
			tty_unregister_driver(&xuli_driver);
		}
		/* ... fall through intended */
	case 2:		/* free IRQ */
		XUartLite_DisableInterrupt(&xs_data[instance].x_uart_lite);
		free_irq(xuli_irq[instance], &xs_data[instance].x_uart_lite);
		/* ... fall through intended */
	case 1:		/* Put the base address back to the physical address. */
		iounmap((void *)cfg->RegBaseAddr);
		cfg->RegBaseAddr = xs_data[instance].saved_baseaddr;
		/* ... fall through intended */
	default:
		;
	}

	enable_irq(xuli_irq[instance]);

	xs_data[instance].init_level = 0;
}

/*
 * Initialize the serial port.
 * Set up the driver, register the UART Lite interrupt, register the driver.
 * This is called from tty_init(), or as a part of the module init.
 */
static int __init xilinx_uartlite_init(void)
{
	int i;
	int retval;

	func_enter();

	for (i = 0; i < XPAR_XUARTLITE_NUM_INSTANCES; i++) {
		XUartLite_Config *cfg;
		struct xs_port *pxs;

		/* Find the config for our device. */
		cfg = XUartLite_GetConfig(i);
		if (!cfg) {
			if (i == 0)
				return -ENODEV;
			else
				break;
		}
		pxs = &xs_data[i];

		/* calculate the time enough to transmit TX_BLOCK_SIZE bytes */
		pxs->tx_timeout =
		    2 * (10 * TX_BLOCK_SIZE * HZ) / cfg->BaudRate + 25;

		if (pxs->init_level != 2) {	/* see serial_console_setup() below */
			/*
			 *  Change the addresses to be virtual;
			 *  save the old ones to restore.
			 */
			pxs->saved_baseaddr = cfg->RegBaseAddr;
			cfg->RegBaseAddr =
			    (u32) ioremap(pxs->saved_baseaddr, 256);
			++pxs->init_level;	/* init_level = 1 */

			/* Initialize the UART */
			if (XUartLite_CfgInitialize
			    (&pxs->x_uart_lite, cfg,
			     cfg->RegBaseAddr) != XST_SUCCESS) {
				printk(KERN_ERR
				       "%s%d: Could not initialize device.\n",
				       DEVICE_NAME, i);
				cleanup(i);
				if (i == 0)
					return -ENODEV;
				else
					break;
			}
		}

		/* Assign the IRQ */
		retval =
		    request_irq(xuli_irq[i], xulite_isr, SA_INTERRUPT,
				DEVICE_NAME, &pxs->x_uart_lite);
		if (retval != 0) {
			printk(KERN_ERR
			       "%s%d: Could not allocate interrupt %d.\n",
			       DEVICE_NAME, i, xuli_irq[i]);
			cleanup(i);
			if (i == 0)
				return -ENODEV;
			else
				break;
		}
		++pxs->init_level;	/* init_level = 2 */
		/* Set interrupt callbacks */
		XUartLite_SetRecvHandler(&pxs->x_uart_lite,
					 &xulite_recv_handler, (void *)pxs);
		XUartLite_SetSendHandler(&pxs->x_uart_lite,
					 &xulite_send_handler, (void *)pxs);

		/* Fill in hardware specific port structure */
		spin_lock_init(&pxs->gs_data.driver_lock);
		pxs->gs_data.magic = SERIAL_MAGIC;
		pxs->gs_data.close_delay = HZ / 2;
		pxs->gs_data.closing_wait = 30 * HZ;
		pxs->gs_data.rd = &xuli_real_driver;
#ifdef NEW_WRITE_LOCKING
		pxs->gs_data.port_write_sem = MUTEX;
#endif
#ifdef DECLARE_WAITQUEUE
		init_waitqueue_head(&pxs->gs_data.open_wait);
		init_waitqueue_head(&pxs->gs_data.close_wait);
#endif
	}

	/* Fill in generic serial driver structures */
	xuli_driver.magic = TTY_DRIVER_MAGIC;
	xuli_driver.driver_name = "serial";
	xuli_driver.name = UARTLITE_TTY_NAME;
	xuli_driver.major = XULITE_MAJOR;
	xuli_driver.minor_start = XULITE_MINOR_START;
	xuli_driver.num = 1;
	xuli_driver.type = TTY_DRIVER_TYPE_SERIAL;
	xuli_driver.subtype = SERIAL_TYPE_NORMAL;
	/* initial termios is the same for all ports: */
	xuli_driver.init_termios = tty_std_termios;
	xuli_driver.init_termios.c_cflag = xulite_get_cflag(0);
	xuli_driver.termios = xuli_termios;
	xuli_driver.termios_locked = xuli_termios_locked;
	xuli_driver.open = xuli_open;
	xuli_driver.close = gs_close;
	xuli_driver.write = gs_write;
	xuli_driver.put_char = gs_put_char;
	xuli_driver.flush_chars = gs_flush_chars;
	xuli_driver.write_room = gs_write_room;
	xuli_driver.chars_in_buffer = gs_chars_in_buffer;
	xuli_driver.flush_buffer = xuli_flush_buffer;
	xuli_driver.ioctl = xuli_ioctl;
	xuli_driver.throttle = xuli_throttle;
	xuli_driver.unthrottle = xuli_unthrottle;
	xuli_driver.set_termios = NULL;
	xuli_driver.stop = NULL;
	xuli_driver.start = NULL;
	xuli_driver.hangup = NULL;

	/* Register serial and callout drivers */
	if (tty_register_driver(&xuli_driver) < 0) {
		int k;

		printk(KERN_ERR "Unable to register serial driver\n");
		for (k = 0; k < i; k++)
			cleanup(k);
		return -ENOMEM;
	}
	++xs_data[0].init_level;	/* init_level = 3 */

	xuli_ports = i;

	func_exit();
	return 0;
}

static void __exit xilinx_uartlite_cleanup(void)
{
	int i;
	for (i = 0; i < xuli_ports; i++) {
		cleanup(i);
	}
}

/*
 * Begin serial console routines
 */

#ifdef CONFIG_XILINX_UARTLITE_CONSOLE

static void serial_outc(int pnum, unsigned char c)
{
	XUartLite *InstancePtr = &xs_data[pnum].x_uart_lite;
	u32 was_enabled;

	/* Disable UART Lite interrupts */
	was_enabled = XUartLite_mIsIntrEnabled(InstancePtr->RegBaseAddress);
	XUartLite_mDisableIntr(InstancePtr->RegBaseAddress);
	/* Wait for Tx hold register to empty */
	while (XUartLite_IsSending(InstancePtr)) ;
	/* Send a character; let interrupt request be generated if
	 * UART Lite interrupt was enabled */
	disable_irq(xuli_irq[pnum]);
	if (was_enabled)
		XUartLite_mEnableIntr(InstancePtr->RegBaseAddress);
	XUartLite_SendByte(InstancePtr->RegBaseAddress, c);
	enable_irq(xuli_irq[pnum]);
}

static void
serial_console_write(struct console *co, const char *s, unsigned count)
{
	unsigned i;
	int pnum = co->index;

	for (i = 0; i < count; i++) {
		if (*s == '\n') {
			serial_outc(pnum, '\r');
		}
		serial_outc(pnum, *s++);
	}
}

static struct tty_driver *serial_console_device(struct console *c, int *index)
{
	*index = c->index;
	return &xuli_driver;
}

static __init int serial_console_setup(struct console *co, char *options)
{
	int pnum = co->index;
	struct xs_port *pxs;

	func_enter();

	if (pnum < 0 || pnum >= XPAR_XUARTLITE_NUM_INSTANCES) {
		return -1;
	}
	pxs = &xs_data[pnum];
	if (pxs->init_level == 0) {
		XUartLite_Config *cfg;

		cfg = XUartLite_GetConfig(pnum);
		if (!cfg)
			return -1;
		pxs->saved_baseaddr = cfg->RegBaseAddr;
		cfg->RegBaseAddr = (u32) ioremap(pxs->saved_baseaddr, 256);
		if (cfg->RegBaseAddr == 0) {
			cfg->RegBaseAddr = pxs->saved_baseaddr;
			return -1;
		}
		if (XUartLite_CfgInitialize
		    (&pxs->x_uart_lite, cfg, cfg->RegBaseAddr) != XST_SUCCESS) {
			iounmap((void *)cfg->RegBaseAddr);
			cfg->RegBaseAddr = pxs->saved_baseaddr;
			pxs->init_level = 0;
			return -1;
		}
		pxs->init_level = 2;
	}

	co->cflag = xulite_get_cflag(pnum);

	return 0;
}

static int __init uartlite_serial_console_init(void)
{
	register_console(&sercons);
	return 0;
}

console_initcall(uartlite_serial_console_init);

#endif				/* #ifdef CONFIG_XILINX_UARTLITE_CONSOLE */

/*
 * End serial console routines
 */

module_init(xilinx_uartlite_init);
module_exit(xilinx_uartlite_cleanup);
