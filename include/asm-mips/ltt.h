/*
 * linux/include/asm-mips/ltt.h
 *
 * Copyright (C) 2002, Karim Yaghmour
 *
 * MIPS definitions for tracing system
 */

#include <linux/config.h>
#include <linux/ltt-events.h>

#ifdef CONFIG_MIPS64
/* Current arch type */
#define LTT_ARCH_TYPE LTT_ARCH_TYPE_MIPS64

/* Current variant type */
#define LTT_ARCH_VARIANT LTT_ARCH_VARIANT_NONE
#else
/* Current arch type */
#define LTT_ARCH_TYPE LTT_ARCH_TYPE_MIPS

/* Current variant type */
#define LTT_ARCH_VARIANT LTT_ARCH_VARIANT_NONE
#endif /* CONFIG_MIPS64 */
