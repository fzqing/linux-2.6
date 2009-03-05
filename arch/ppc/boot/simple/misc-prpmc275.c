/*
 * arch/ppc/boot/simple/misc-prpmc275.c
 *
 * Set up MPSC values to bootwrapper can prompt user.
 *
 * Author: Vladimir A. Barinov <vbarinov@ru.mvista.com>
 *
 * Based on code done by Mark A. Greer <mgreer@mvista.com>
 *
 * 2006 (c) MontaVista Software, Inc.  This file is licensed under
 * the terms of the GNU General Public License version 2.  This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/types.h>
#include <platforms/prpmc275.h>

#ifdef CONFIG_SERIAL_MPSC_CONSOLE
extern u32 mv64x60_console_baud;
extern u32 mv64x60_mpsc_clk_src;
extern u32 mv64x60_mpsc_clk_freq;
#endif

void
mv64x60_board_init(void __iomem *old_base, void __iomem *new_base)
{
#ifdef CONFIG_SERIAL_MPSC_CONSOLE
	mv64x60_console_baud = PPMC275_DEFAULT_BAUD;
	mv64x60_mpsc_clk_src = PPMC275_MPSC_CLK_SRC;
	mv64x60_mpsc_clk_freq = PPMC275_BUS_FREQ;
#endif
}
