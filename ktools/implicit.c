/*
 * ktools/implicit.c
 *
 * Author: David Singleton <dsingleton@mvista.com>
 *
 * 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/ilatency.h>

/*
 * We branch-link here from ret_from_exception to
 * catch interrupts coming back on.  We also just log the fact that
 * interrupts are coming back on, we don't actually turn them on, thus
 * the third parameter, just_logging.
 */
                                                                                
void latency_check(void) {
#ifdef CONFIG_ILATENCY
        ilat_irq_enable(__BASE_FILE__, __LINE__, 1);
#endif
}
EXPORT_SYMBOL(latency_check);
