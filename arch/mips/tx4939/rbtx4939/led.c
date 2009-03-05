/*
 * arch/mips/tx4939/toshiba_rbtx4939/led.c
 *
 * LED driver on RBTX4939
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

#include <asm/io.h>
#include <asm/tx4939/rbtx4939.h>

static int segment[0x10] = {
	0x7e, 0x30, 0x6d, 0x79, 0x33, 0x5b, 0x5f, 0x70, 0x7f, 0x7b,	/* 0-9 */
	0x77, 0x1f, 0x0d, 0x3d, 0x4f, 0x47	/* a-f */
};

void rbtx4939_led(unsigned int led, unsigned int hex)
{
	if (hex < 0x0 || 0xf < hex)
		reg_wr08(rbtx4939_7segled_ptr + (led << 1), 0x00);
	else
		reg_wr08(rbtx4939_7segled_ptr + (led << 1), segment[hex]);
}

#if defined(CONFIG_HEARTBEAT)
void rbtx4939_heartbeat(void)
{
	static int sw;
	switch (sw) {
	case 1:
		reg_wr08(rbtx4939_7segled_ptr + 0, 0x1d);
		reg_wr08(rbtx4939_7segled_ptr + 2, 0x00);
		break;
	case 2:
		reg_wr08(rbtx4939_7segled_ptr + 0, 0x63);
		reg_wr08(rbtx4939_7segled_ptr + 2, 0x00);
		break;
	case 3:
		reg_wr08(rbtx4939_7segled_ptr + 0, 0x00);
		reg_wr08(rbtx4939_7segled_ptr + 2, 0x63);
		break;
	default:
		reg_wr08(rbtx4939_7segled_ptr + 0, 0x00);
		reg_wr08(rbtx4939_7segled_ptr + 2, 0x1d);
		sw = 0;
		break;
	}
	sw++;
}
#endif
