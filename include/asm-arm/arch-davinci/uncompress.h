/*
 * linux/include/asm-arm/arch-davinci/uncompress.h
 *
 * Serial port stubs for kernel decompress status messages
 *
 *  Author:     Anant Gole
 * (C) Copyright (C) 2006, Texas Instruments, Inc
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/config.h>
#include <asm/arch/hardware.h>

#ifdef	CONFIG_DAVINCI_LL_DEBUG_UART1
#define DAVINCI_UART_BASE DAVINCI_UART1_BASE
#else
#define DAVINCI_UART_BASE DAVINCI_UART0_BASE
#endif

typedef struct uart_registers_t {
	unsigned int rbr_thr;
	unsigned int ier;
	unsigned int iir_fcr;
	unsigned int lcr;
	unsigned int mcr;
	unsigned int lsr;
	unsigned int msr;
	unsigned int scr;
	unsigned int dll;
	unsigned int dlh;
	unsigned int pid1;
	unsigned int pid2;
	unsigned int pwremu;
} uart_registers;

/* Initialize Serial port */
static void do_nothing(void)
{
	unsigned int counter;
	for (counter = 0; counter < 0x200; counter++) {
		/* Do nothing */
	}
}

/* Wait (busy loop) untill TX FIFO is empty and a character
 * can be transmitted */
static void serial_waitfortxcharcomplete(void)
{
	volatile uart_registers *uartregs;

	uartregs = (volatile uart_registers *) DAVINCI_UART_BASE;
	do_nothing();

	while (!uartregs->lsr & 0x20) {
		/* Do Nothing */
	}
}

static void serial_putc(const char c)
{
	volatile uart_registers *uartregs;

	uartregs = (volatile uart_registers *) DAVINCI_UART_BASE;
	if (c == '\n')
		serial_putc('\r');
	uartregs->rbr_thr = c;

	serial_waitfortxcharcomplete();
}

/* Send string on UART */
static void putstr(const char *str)
{
	volatile uart_registers *uartregs;

	uartregs = (volatile uart_registers *) DAVINCI_UART_BASE;
	while (*str != '\0') {
		serial_putc(*str);
		str++;
	}
}

#define arch_decomp_setup()
#define arch_decomp_wdog()
