/*
 * include/asm-ppc/frd.h
 *
 * Timing support for RT fast domain kernel preemption measurements
 *
 * Author: Yi Yang <yyang@ch.mvista.com>, Tom Rini <trini@mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef _ASM_FRD_H
#define _ASM_FRD_H

#include <linux/config.h>
#include <asm/time.h>

#define FRD_64BIT_TIME	1

/* frd default clock function using the timebase. */
static inline unsigned long long frd_clock(void)
{
	unsigned long lo, hi, hi2;

	do {
		hi = get_tbu();
		lo = get_tbl();
		hi2 = get_tbu();
	} while (hi2 != hi);
	return ((unsigned long long) hi << 32) | lo;
}
#endif
