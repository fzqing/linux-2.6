/*
 * arch/mips/tx4939/common/puts.c
 *
 * BRIEF MODULE DESCRIPTION
 *	Low level uart routines to directly access a 16550 uart.
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

#include <linux/types.h>
#include <asm/tx4939/tx4939.h>

#define SERIAL_BASE   TX4939_SIO_REG(0)
#define SER_CMD       0x7
#define SER_DATA      0x1
#define TX_BUSY       0x20

#define TIMEOUT       0xffffff
#define SLOW_DOWN

static const char digits[16] = "0123456789abcdef";
static volatile unsigned long *const com1 = (unsigned long *)SERIAL_BASE;

#if defined(SLOW_DOWN)
static inline void slow_down(void)
{
	int k;
	for (k = 0; k < 10000; k++) ;
}
#else
#define slow_down()
#endif

#define TXX9_SILCR	0x00
#define TXX9_SIDICR	0x04
#define TXX9_SIDISR	0x08
#define TXX9_SICISR	0x0c
#define TXX9_SIFCR	0x10
#define TXX9_SIFLCR	0x14
#define TXX9_SIBGR	0x18
#define TXX9_SITFIFO	0x1c
#define TXX9_SIRFIFO	0x20

/* SICISR : Change Int. Status */
#define TXX9_SICISR_OERS	0x00000020
#define TXX9_SICISR_CTSS	0x00000010
#define TXX9_SICISR_RBRKD	0x00000008
#define TXX9_SICISR_TRDY	0x00000004
#define TXX9_SICISR_TXALS	0x00000002
#define TXX9_SICISR_UBRKD	0x00000001

/* SIFLCR : Flow Control */
#define TXX9_SIFLCR_RTSTL_MAX	0x0000001e

static unsigned int sio_in(unsigned long base, int offset)
{
	return (*(volatile unsigned long *)(base + offset));
}

static void sio_out(unsigned long base, int offset, unsigned int value)
{
	(*(volatile unsigned long *)(base + offset)) = (value);
}

static void putch(const unsigned char c)
{
	unsigned int status;
	int i = 0;

	do {
		status = sio_in(SERIAL_BASE, TXX9_SICISR);
		slow_down();
		i++;
		if (i > TIMEOUT) {
			break;
		}
	} while (0 == (status & TXX9_SICISR_TRDY));
	sio_out(SERIAL_BASE, TXX9_SITFIFO, (u32) c);
	if (c == '\n') {
		sio_out(SERIAL_BASE, TXX9_SITFIFO, (u32) '\n');
		sio_out(SERIAL_BASE, TXX9_SITFIFO, (u32) '\r');
	}

}

static void puts(unsigned char *cp)
{
	unsigned int status;
	int i = 0;

	while (*cp) {
		do {
			status = sio_in(SERIAL_BASE, TXX9_SICISR);
			slow_down();
			i++;
			if (i > TIMEOUT) {
				break;
			}
		} while (0 == (status & TXX9_SICISR_TRDY));
		sio_out(SERIAL_BASE, TXX9_SITFIFO, (u32) * cp++);
	}
	putch('\r');
	putch('\n');
}

void fputs(unsigned char *cp)
{
	unsigned int status;
	int i = 0;

	while (*cp) {

		do {
			status = sio_in(SERIAL_BASE, TXX9_SICISR);
			slow_down();
			i++;
			if (i > TIMEOUT) {
				break;
			}
		} while (0 == (status & TXX9_SICISR_TRDY));
		sio_out(SERIAL_BASE, TXX9_SITFIFO, (u32) * cp++);
	}
}

static void put64(uint64_t ul)
{
	int cnt;
	unsigned ch;

	cnt = 16;		/* 16 nibbles in a 64 bit long */
	putch('0');
	putch('x');
	do {
		cnt--;
		ch = (unsigned char)(ul >> cnt * 4) & 0x0F;
		putch(digits[ch]);
	} while (cnt > 0);
}

static void put32(unsigned u)
{
	int cnt;
	unsigned ch;

	cnt = 8;		/* 8 nibbles in a 32 bit long */
	putch('0');
	putch('x');
	do {
		cnt--;
		ch = (unsigned char)(u >> cnt * 4) & 0x0F;
		putch(digits[ch]);
	} while (cnt > 0);
}

void sio_setup(void)
{
	/* Enable RX/TX */
	/* console port should not use RTC/CTS. */
	sio_out(SERIAL_BASE, TXX9_SIFLCR, TXX9_SIFLCR_RTSTL_MAX);
}
