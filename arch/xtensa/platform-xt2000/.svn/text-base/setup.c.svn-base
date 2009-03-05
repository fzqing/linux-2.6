// TODO
/*
 *
 * arch/xtensa/platform-xt2000/setup.c
 *
 * ...
 *
 * Authors:	Chris Zankel <chris@zankel.net>
 *		Joe Taylor <joe@tensilica.com>
 *
 * Copyright 2001 - 2004 Tensilica Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <linux/config.h>
#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/reboot.h>
#include <linux/kdev_t.h>
#include <linux/types.h>
#include <linux/major.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/stringify.h>

#include <asm/processor.h>
#include <asm/platform.h>
#include <asm/bootparam.h>
#include <asm/platform/xt2000.h>


/* Assumes s points to an 8-chr string.  No checking for NULL. */

static void led_print (int f, char *s)
{
	unsigned long* led_addr = (unsigned long*) (XTBOARD_LED_VADDR+0xE0) + f;
	int i;
	for (i = f; i < 8; i++)
		*led_addr++ = *s++;
}

void platform_halt(void)
{
	led_print (0, "  HALT  ");
	local_irq_disable();
	while (1);
}

void platform_power_off(void)
{
	led_print (0, "POWEROFF");
	local_irq_disable();
	while (1);
}

void platform_restart(void)
{
	/* Flush and reset the mmu, simulate a processor reset, and
	 * jump to the reset vector. */

	__asm__ __volatile__ ("movi	a2, 15\n\t"
			      "wsr	a2, " __stringify(ICOUNTLEVEL) "\n\t"
			      "movi	a2, 0\n\t"
			      "wsr	a2, " __stringify(ICOUNT) "\n\t"
			      "wsr	a2, " __stringify(IBREAKENABLE) "\n\t"
			      "wsr	a2, " __stringify(LCOUNT) "\n\t"
			      "movi	a2, 0x1f\n\t"
			      "wsr	a2, " __stringify(PS) "\n\t"
			      "isync\n\t"
			      "jx	%0\n\t"
			      :
			      : "a" (XCHAL_RESET_VECTOR_VADDR)
			      : "a2"
			      );

	/* control never gets here */
}

void __init platform_setup(char** cmdline)
{
	led_print (0, "LINUX   ");
}

/* early initialization */

void platform_init(bp_tag_t* first)
{
	/* Nothing to be done here. */
}

/* Heartbeat. Let the LED blink. */

void platform_heartbeat(void)
{
	static int i=0, t = 0;

	if (--t < 0)
	{
		t = 59;
		led_print(7, i ? ".": " ");
		i ^= 1;
	}
}



