/*
 * arch/xtensa/platform-xt2000/time.c
 *
 * System clock and time.
 *
 * Copyright (C) 2001 - 2004 Tensilica Inc.
 *
 * Kevin Chea <kchea@tensilica.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/time.h>
#include <asm/delay.h>
#include <asm/processor.h>
#include <asm/platform/xt2000-uart.h>

extern volatile unsigned long wall_jiffies;

#define USECS_PER_JIFFY (1000000/HZ)
#define TICK_SIZE tick


#if CONFIG_XTENSA_CALIBRATE_CCOUNT
/*
 *  Before we initialize the UART, use it to measure system clock speed.
 *  The XT2000's UART crystal should always be 18.432 MHz, but the
 *  system clock can vary widely.  So we use the UART crystal as
 *  a reference point to measure system clock rate.
 *
 *  The 16552 DUART is put temporarily in loopback mode so we can measure
 *  character transmission delay without outputting junk on the console.
 */

void platform_calibrate_ccount(void)
{
	uart_dev_t *u = &DUART_1_BASE;
	unsigned int tstart, tend, tperiod;
	unsigned char baud_hi, baud_lo, lcr, mcr, fcr;

	/*  Divisor for 100bps loopback speed (turns out to be 11520):  */
	#define DIV100BPS	DUART_DIVISOR(DUART16552_XTAL_FREQ, 100)

	lcr = _LCR(u);

	_LCR(u) = DLAB_ENABLE;      	/* DLAB=1 on UART1 at least */

	baud_hi = _DLM(u);
	baud_lo = _DLL(u);
	mcr = _MCR(u);
	fcr = _FCR(u);

	_DLL(u) = (DIV100BPS & 0xFF);	/* set baudrate gen. divider LSB */
	_DLM(u) = (DIV100BPS >> 8);	/* set baudrate gen. divider MSB */
	_AFR(u) = AFR_CONC_WRITE;	/* enable writes to both ports */
	_MCR(u) = LOOP_BACK;		/* enable loopback mode */
	_LCR(u) = 0;			/* DLAB=0 on both ports */
	_IER(u) = 0;			/* disable interrupts */
	_FCR(u) = (RCVR_FIFO_RESET | XMIT_FIFO_RESET);
	_FCR(u) = _FIFO_ENABLE;
	_LCR(u) = DLAB_ENABLE;		/* DLAB=1 on both ports */
	_AFR(u) = 0;			/* turn off concurrent writes */
	_LCR(u) = WORD_LENGTH(8);	/* DLAB=0, 8N1 (10 bits tot per char) */

	/*
	 *  Send two characters at 10 chars/sec.  The first lets us sync up with
	 *  the UART clock, and we then measure transmission time of the second
	 *  character (time from receipt of the 1st to that of the 2nd character).
	 */
	_TXB(u) = '@';
	_TXB(u) = '@';
	while (!RCVR_READY(u));
	tstart = get_ccount();
	_RXB(u);		/* clear Rx ready status */
	while (!RCVR_READY(u));
	tend = get_ccount();
	_RXB(u);		/* clear Rx ready status */

	_LCR(u) = DLAB_ENABLE;
	_DLM(u) = baud_hi;
	_DLL(u) = baud_lo;
	_MCR(u) = mcr;
	_FCR(u) = fcr;
	_LCR(u) = lcr;
	tperiod = (tend - tstart)*10;
	tperiod += 5000;		/* Round to nearest .01 MHZ */
	tperiod = tperiod / 10000;
	tperiod = tperiod * 10000;
	ccount_per_jiffy = tperiod/HZ;
	nsec_per_ccount = 1000000000UL / tperiod;
}
#endif
