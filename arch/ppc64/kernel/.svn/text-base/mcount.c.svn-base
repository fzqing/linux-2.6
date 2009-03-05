/*
 * arch/ppc64/kernel/mcount.c
 *
 * Instrumentation functions for latency tracing.
 *
 * Author: Frank Rowand <frowand@mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/linkage.h>
#include <linux/sched.h>

void notrace mcount(void)
{
	if (mcount_enabled == 1)
		__mcount();
}

/*
 * The -pg compiler flag causes a call to _mcount() to be added to the
 * prologue of all functions not tagged with notrace.
 */
void notrace _mcount(void)
{
	if (mcount_enabled == 1)
		__mcount();
}
