/*
 * sh_lcdc.h: SH LCD control device driver (16bit color only)
 *
 *   Copyright(C) 2002/05/20 Mitsuharu Takei(takei-mitsuharu@hitachi-ul.co.jp)
 *   Copyright(C) 2004/11/26 Takashi Kusuda
 *        Modified to supoort SH7720 and kernel 2.6
 */

#ifndef __LINUX_VIDEO_SH_LCDC_H
#define __LINUX_VIDEO_SH_LCDC_H

#include <linux/config.h>

#define SH_LCDC_PIXCLOCK_25_175 39722 /* 25.175Mhz=39722ps */

#define SH_LCDC_FB_PHYS P2SEGADDR(CONFIG_MEMORY_START+CONFIG_MEMORY_SIZE-CONFIG_SH_LCDC_FB_SIZE) /* start add. of FB */
#define SH_LCDC_FB_PHYS_LEN	CONFIG_SH_LCDC_FB_SIZE


/* Regiseters */
#if defined(CONFIG_CPU_SUBTYPE_SH7720)
#define SH_LCDC_REG_BASE 0xA4400400
#define SH_LCDC_LDPR(x)  (0xA4400000 + (x)*4) /* Color Palette Reg(256col) */
#elif defined(CONFIG_CPU_SUBTYPE_SH7727)
#define SH_LCDC_REG_BASE 0xA4000C00
#define SH_LCDC_LDPR(x)  (0xA4000800 + (x)*4) /* Color Palette Reg(256col) */
#else
#error "What CPU is this ?"
#endif


#define SH_LCDC_LDICKR		(SH_LCDC_REG_BASE + 0x0)
#define SH_LCDC_LDMTR		(SH_LCDC_REG_BASE + 0x2)
#define SH_LCDC_LDDFR		(SH_LCDC_REG_BASE + 0x4)
#define SH_LCDC_LDSMR		(SH_LCDC_REG_BASE + 0x6)
#define SH_LCDC_LDSARU		(SH_LCDC_REG_BASE + 0x8)
#define SH_LCDC_LDSARL		(SH_LCDC_REG_BASE + 0xC)
#define SH_LCDC_LDLAOR		(SH_LCDC_REG_BASE + 0x10)
#define SH_LCDC_LDPALCR		(SH_LCDC_REG_BASE + 0x12)
#define SH_LCDC_LDHCNR		(SH_LCDC_REG_BASE + 0x14)
#define SH_LCDC_LDHSYNR		(SH_LCDC_REG_BASE + 0x16)
#define SH_LCDC_LDVDLNR		(SH_LCDC_REG_BASE + 0x18)
#define SH_LCDC_LDVTLNR		(SH_LCDC_REG_BASE + 0x1A)
#define SH_LCDC_LDVSYNR		(SH_LCDC_REG_BASE + 0x1C)
#define SH_LCDC_LDACLNR		(SH_LCDC_REG_BASE + 0x1E)
#define SH_LCDC_LDINTR		(SH_LCDC_REG_BASE + 0x20)
#define SH_LCDC_LDPMMR		(SH_LCDC_REG_BASE + 0x24)
#define SH_LCDC_LDPSPR		(SH_LCDC_REG_BASE + 0x26)
#define SH_LCDC_LDCNTR		(SH_LCDC_REG_BASE + 0x28)

#if defined(CONFIG_CPU_SUBTYPE_SH7720)
#define SH_LCDC_LDUINTR		(SH_LCDC_REG_BASE + 0x34)
#define SH_LCDC_LDUINTLNR	(SH_LCDC_REG_BASE + 0x36)
#define SH_LCDC_LDLIRNR		(SH_LCDC_REG_BASE + 0x40)
#endif

#endif /* __LINUX_VIDEO_SH_LCDC_H */
