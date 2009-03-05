/*
 * linux/include/asm-ppc/ltt.h
 *
 * Copyright (C) 2002, Karim Yaghmour
 *
 * PowerPC definitions for tracing system
 */

#include <linux/config.h>
#include <linux/ltt-events.h>

/* Current arch type */
#define LTT_ARCH_TYPE LTT_ARCH_TYPE_PPC

/* PowerPC variants */
#define LTT_ARCH_VARIANT_PPC_4xx          1   /* 4xx systems (IBM embedded series) */
#define LTT_ARCH_VARIANT_PPC_6xx          2   /* 6xx/7xx/74xx/8260/POWER3 systems (desktop flavor) */
#define LTT_ARCH_VARIANT_PPC_8xx          3   /* 8xx system (Motoral embedded series) */
#define LTT_ARCH_VARIANT_PPC_ISERIES      4   /* 8xx system (iSeries) */
#define LTT_ARCH_VARIANT_PPC_44x          5   /* 44x systems (IBM embedded series) */
#define LTT_ARCH_VARIANT_PPC_BOOKE        6   /* Freescale E500 */

/* Current variant type */
#ifdef CONFIG_40x
#define LTT_ARCH_VARIANT LTT_ARCH_VARIANT_PPC_4xx
#elif defined(CONFIG_44x)
#define LTT_ARCH_VARIANT LTT_ARCH_VARIANT_PPC_44x
#elif defined(CONFIG_BOOKE)
#define LTT_ARCH_VARIANT LTT_ARCH_VARIANT_PPC_BOOKE
#elif defined(CONFIG_6xx)
#define LTT_ARCH_VARIANT LTT_ARCH_VARIANT_PPC_6xx
#elif defined(CONFIG_8xx)
#define LTT_ARCH_VARIANT LTT_ARCH_VARIANT_PPC_8xx
#elif defined(CONFIG_PPC_ISERIES)
#define LTT_ARCH_VARIANT LTT_ARCH_VARIANT_PPC_ISERIES
#endif
