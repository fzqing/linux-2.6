/*
 * linux/arch/arm/mach-pxa/leds-zylonite.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/config.h>
#include <linux/init.h>

#include <asm/hardware.h>
#include <asm/leds.h>
#include <asm/system.h>

#include <asm/arch/pxa-regs.h>
#include <asm/arch/zylonite.h>

#include "leds.h"

#define LED_STATE_ENABLED	1
#define LED_STATE_CLAIMED	2

/* Zylonite has two debug LEDs on GPIOs 1 and 4. We use the LED on GPIO 1 as
   the timer tick LED and the LED on GPIO 4 as the CPU usage LED. */

#define LED_TIMER 1
#define LED_CPU 4

static unsigned int led_state;
static unsigned int led_timer_state;
static unsigned int led_cpu_state;

static inline void set_led(int led, int state)
{
	if (state) {
		GPSR0 = (1 << led);
	} else {
		GPCR0 = (1 << led);
	}
}

void zylonite_leds_event(led_event_t evt)
{
	unsigned long flags;

	local_irq_save(flags);

	switch (evt) {

	case led_start:

		/* make sure the GPIOs for the LEDS are set up as outputs */
		GSDR0 = (1 << LED_TIMER);
		GSDR0 = (1 << LED_CPU);

		led_timer_state = 1;
		led_cpu_state = 0;
		led_state = LED_STATE_ENABLED;

		break;

	case led_stop:
		led_state &= ~LED_STATE_ENABLED;
		break;

	case led_claim:
		led_state |= LED_STATE_CLAIMED;
		led_timer_state = led_cpu_state = 0;
		break;

	case led_release:
		led_state &= ~LED_STATE_CLAIMED;
		led_timer_state = led_cpu_state = 0;
		break;

#ifdef CONFIG_LEDS_TIMER
	case led_timer:
		led_timer_state = (led_timer_state ? 0 : 1);
		break;
#endif

#ifdef CONFIG_LEDS_CPU
	case led_idle_start:
		led_cpu_state = 0;
		break;

	case led_idle_end:
		led_cpu_state = 1;
		break;
#endif

	default:
		break;
	}

	local_irq_restore(flags);
}
