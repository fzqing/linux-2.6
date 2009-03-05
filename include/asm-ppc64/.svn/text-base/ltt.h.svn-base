/*
 * linux/include/asm-ppc64/ltt.h
 *
 * Author:  MontaVista Software, Inc.  <source@mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc.
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * PowerPC 64 definitions for tracing system
 */

#include <linux/config.h>
#include <linux/ltt-events.h>

/* Arch variants are common to PPC and PPC64 */
#include <asm-ppc/ltt.h>

/* Current arch type */
#undef LTT_ARCH_TYPE
#define LTT_ARCH_TYPE LTT_ARCH_TYPE_PPC64

/* Current variant type */
#undef LTT_ARCH_VARIANT
#ifdef CONFIG_PPC_ISERIES
#define LTT_ARCH_VARIANT LTT_ARCH_VARIANT_PPC_ISERIES
#else
#define LTT_ARCH_VARIANT LTT_ARCH_VARIANT_NONE
#endif
