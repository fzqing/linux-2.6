/*
 * include/linux/touch_watchdogs.h
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

#ifndef _LINUX_TOUCH_WATCHDOGS_H
#define _LINUX_TOUCH_WATCHDOGS_H

#ifdef CONFIG_TOUCH_WATCHDOGS
#include <linux/notifier.h>

extern struct notifier_block *watchdog_list;

extern void touch_watchdogs(void);
#else
#define touch_watchdogs()
#endif

#endif

