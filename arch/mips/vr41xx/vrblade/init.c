/*
 * arch/mips/vr41xx/vrblade/init.c
 *
 * PROM library initialisation code for NEC VRBlade VR4133A (Wildcat) board.
 *
 * Author: Yoichi Yuasa <yyuasa@mvista.com, or source@mvista.com> and
 *         Jun Sun <jsun@mvista.com, or source@mvista.com> and
 *         Alex Sapkov <asapkov@ru.mvista.com>
 *		   Edmond dela Cruz <edmondd@ntsp.nec.co.jp>
 *
 * 2001-2004 (c) MontaVista, Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/config.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include <asm/bootinfo.h>

const char *get_system_type(void)
{
	return "NEC VRBlade";
}

void __init bus_error_init(void)
{
}
