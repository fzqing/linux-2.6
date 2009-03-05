/*
 * arch/mips/sibyte/sb1250/kgdb_sibyte.c
 *
 * Author: Manish Lachwani, mlachwani@mvista.com or manish@koffee-break.com
 *
 * 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

/*
 * Support for KGDB on the Broadcom Sibyte. The SWARM board
 * for example does not have a 8250/16550 compatible serial
 * port. Hence, we need to have a driver for the serial
 * ports to handle KGDB.  This board needs nothing in addition
 * to what is normally provided by the gdb portion of the stub.
 */

#include <linux/delay.h>
#include <linux/kernel_stat.h>
#include <linux/init.h>
#include <linux/kgdb.h>

#include <asm/io.h>
#include <asm/sibyte/sb1250.h>
#include <asm/sibyte/sb1250_regs.h>
#include <asm/sibyte/sb1250_uart.h>
#include <asm/sibyte/sb1250_int.h>
#include <asm/addrspace.h>

#if defined(CONFIG_KGDB_TTYS0)
int kgdb_port = 0;
#elif defined(CONFIG_KGDB_TTYS1)
int kgdb_port = 1;
#elif defined(CONFIG_KGDB_TTYS2)
int kgdb_port = 2;
#elif defined(CONFIG_KGDB_TTYS3)
int kgdb_port = 3;
#else
int kgdb_port = 1;         /* Start with this if not given */
#endif

static int kgdb_irq;

extern char sb1250_duart_present[];
extern int sb1250_steal_irq(int irq);

/* Forward declarations. */
static void kgdbsibyte_init_duart(void);

#define IMR_IP6_VAL	K_INT_MAP_I4
#define	duart_out(reg, val)	csr_out32(val, IOADDR(A_DUART_CHANREG(kgdb_port,reg)))
#define duart_in(reg)		csr_in32(IOADDR(A_DUART_CHANREG(kgdb_port,reg)))

static void kgdb_swarm_write_char(int c)
{
	while ((duart_in(R_DUART_STATUS) & M_DUART_TX_RDY) == 0);
	duart_out(R_DUART_TX_HOLD, c);
}

static int kgdb_swarm_read_char(void)
{
	int ret_char;
	unsigned int status;

	status = duart_in(R_DUART_STATUS);
	while ((status & M_DUART_RX_RDY) == 0) {
		status = duart_in(R_DUART_STATUS);
	}

	/*
	 * Check for framing error
	 */
	if (status & M_DUART_FRM_ERR) {
		kgdbsibyte_init_duart();
		kgdb_swarm_write_char('-');
		return '-';
	}

	ret_char = duart_in(R_DUART_RX_HOLD);

	return ret_char;
}

void sb1250_kgdb_interrupt(struct pt_regs *regs)
{
	int kgdb_irq = K_INT_UART_0 + kgdb_port;
	/*
	 * Clear break-change status (allow some time for the remote
	 * host to stop the break, since we would see another
	 * interrupt on the end-of-break too)
	 */
	kstat_this_cpu.irqs[kgdb_irq]++;
	mdelay(500);
	duart_out(R_DUART_CMD, V_DUART_MISC_CMD_RESET_BREAK_INT |
				M_DUART_RX_EN | M_DUART_TX_EN);
	breakpoint();
}

/*
 * We use port #1 and we set it for 115200 BAUD, 8n1.
 */
static void
kgdbsibyte_init_duart(void)
{
	/* Set 8n1. */
	duart_out(R_DUART_MODE_REG_1,
			V_DUART_BITS_PER_CHAR_8 | V_DUART_PARITY_MODE_NONE);
	duart_out(R_DUART_MODE_REG_2, M_DUART_STOP_BIT_LEN_1);
	/* Set baud rate of 115200. */
	duart_out(R_DUART_CLK_SEL, V_DUART_BAUD_RATE(115200));
	/* Enable rx and tx */
	duart_out(R_DUART_CMD, M_DUART_RX_EN | M_DUART_TX_EN);
}

static int
kgdb_init_io(void)
{
#ifdef CONFIG_SIBYTE_SB1250_DUART
	sb1250_duart_present[kgdb_port] = 0;
#endif

	kgdbsibyte_init_duart();

	return 0;
}

/*
 * Hookup our IRQ line.  We will already have been initialized a
 * this point.
 */
static void __init kgdbsibyte_hookup_irq(void)
{
	/* Steal the IRQ. */
	kgdb_irq = K_INT_UART_0 + kgdb_port;

	/* Setup uart 1 settings, mapper */
	__raw_writeq(M_DUART_IMR_BRK, IOADDR(A_DUART_IMRREG(kgdb_port)));

	sb1250_steal_irq(kgdb_irq);

	__raw_writeq(IMR_IP6_VAL,
       	             IOADDR(A_IMR_REGISTER(0, R_IMR_INTERRUPT_MAP_BASE) +
			(kgdb_irq<<3)));

	sb1250_unmask_irq(0, kgdb_irq);
}

struct kgdb_io kgdb_io_ops = {
	.read_char = kgdb_swarm_read_char,
	.write_char = kgdb_swarm_write_char,
	.init = kgdb_init_io,
	.late_init = kgdbsibyte_hookup_irq,
};
