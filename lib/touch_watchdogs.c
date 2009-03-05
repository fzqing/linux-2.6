/*
 * lib/touch_watchdogs.c
 *
 * The code is used to touch all the watchdogs.
 * This is needed when the system goes into a poll state
 * for too long (polling the network in kgdb or saving a crash dump)
 * to prevent the watchdogs from rebooting it.
 * The watchdog_list should be used by the watchdog drivers
 * to register a touch-me callback function.
 *
 * Author: Valentine Barshak <vbarshak@ru.mvista.com>
 *
 * 2006 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/module.h>
#include <linux/touch_watchdogs.h>

#ifdef CONFIG_X86_LOCAL_APIC
#include <linux/nmi_watchdog.h>
#endif

struct notifier_block *watchdog_list;

void touch_watchdogs(void)
{
#ifdef CONFIG_X86_LOCAL_APIC
	touch_nmi_watchdog();
#endif
	notifier_call_chain(&watchdog_list, 0, NULL);
}

EXPORT_SYMBOL(watchdog_list);
EXPORT_SYMBOL(touch_watchdogs);

