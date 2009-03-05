/*
 * linux/arch/sh/boards/se/7780/led.c
 *
 * Copyright (C) 2000 Stuart Menefy <stuart.menefy@st.com>
 * Copyright (C) 2004 Takashi Kusuda
 *
 * May be copied or modified under the terms of the GNU General Public
 * License.  See linux/COPYING for more information.
 *
 * This file contains SH7780 Solution Engine specific LED code.
 */

#include <linux/config.h>
#include <asm/mach/map.h>

static void mach_led(int position, int value)
{
	volatile unsigned short* p = (volatile unsigned short*)FPGA_DBG_LED;

	if (value) {
		*p &= ~1;
	} else {
		*p |= 1;
	}
}

#ifdef CONFIG_HEARTBEAT

#define LED_CHAR_NUM	24
static char disp_chars[LED_CHAR_NUM] = " SH-Linux on MS7780SE03 ";
static int disp_chars_start_pos = 0;

#include <linux/sched.h>

/* Cycle the LED's in the clasic Knightrider/Sun pattern */
void heartbeat_7780se(void)
{
	static unsigned int cnt = 0, period = 0;
	int i, tmp;
	unsigned short offset;

	cnt += 1;
	if (cnt < period) {
		return;
	}

	cnt = 0;

	/* Go through the points (roughly!):
	 * f(0)=10, f(1)=16, f(2)=20, f(5)=35,f(inf)->110
	 */
	period = 165 - ( (300<<FSHIFT)/
			 ((avenrun[0]/5) + (3<<FSHIFT)) );


	for (i=0; i < 8; i++){
		if((disp_chars_start_pos + i) >= LED_CHAR_NUM)
			tmp = disp_chars_start_pos + i - LED_CHAR_NUM;
		else
			tmp = disp_chars_start_pos + i;
		offset = ((7<<3) + i)<<1;
		*(volatile unsigned short *)(PA_LED_DISP + offset) = disp_chars[tmp];
	}

	if(disp_chars_start_pos < (LED_CHAR_NUM-1))
		disp_chars_start_pos++;
	else
		disp_chars_start_pos = 0;
}
#endif /* CONFIG_HEARTBEAT */
