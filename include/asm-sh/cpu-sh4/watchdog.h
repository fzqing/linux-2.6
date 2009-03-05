/*
 * include/asm-sh/cpu-sh4/watchdog.h
 *
 * Copyright (C) 2002, 2003 Paul Mundt
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#ifndef __ASM_CPU_SH4_WATCHDOG_H
#define __ASM_CPU_SH4_WATCHDOG_H

/* Register definitions */
#if defined(CONFIG_CPU_SUBTYPE_SH7780)
#define SH_WDT_BASE_ADDR       0xFFCC0000
#define WTST                   (SH_WDT_BASE_ADDR + 0)
#define WTCSR                  (SH_WDT_BASE_ADDR + 4)
#define WTBST                  (SH_WDT_BASE_ADDR + 8)
#define WTCNT                  (SH_WDT_BASE_ADDR + 16)
#define WTBCST                 (SH_WDT_BASE_ADDR + 24)
#else
#define SH_WDT_BASE_ADDR       0xFFC00000
#define WTCNT                  (SH_WDT_BASE_ADDR + 8)
#define WTCSR                  (SH_WDT_BASE_ADDR + 12)
#endif

/* Bit definitions */
#define WTCSR_TME	0x80
#define WTCSR_WT	0x40
#define WTCSR_RSTS	0x20
#define WTCSR_WOVF	0x10
#define WTCSR_IOVF	0x08

#endif /* __ASM_CPU_SH4_WATCHDOG_H */

