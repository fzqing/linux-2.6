/*
 * ygv619.h,v 0.8 2001/02/16
 *  YGV619 video device driver for Super-H
 *
 *   Copyright (C) 2001/02/16 Mitsuharu Takei(takei-mitsuharu@hitachi-ul.co.jp)
 */

#ifndef __LINUX_VIDEO_YGV619_H
#define __LINUX_VIDEO_YGV619_H

#include <linux/config.h>

#define YGV619_PIXCLOCK_78_8 12690	/* 78.8Mhz=12690ps */
#define YGV619_PIXCLOCK_25_175 39722	/* 25.175Mhz=39722ps */
#define YGV619_PIXCLOCK_12_272 81486	/* 12.272Mhz=81486ps */

#define YGV619_FB_PHYS (CONFIG_YGV619_FB_BASE + 0x00000000UL)	/* FB base address */
#define YGV619_FB_PHYS_LEN 0x00200000UL	/* FB length */

#define YGV619_REG (CONFIG_YGV619_FB_BASE + 0x01000000UL)
#define YGV619_R(x) (YGV619_REG + (x))

#define YGV619_PA1_OFF 0x00200000UL
#define YGV619_PA1_R(x) (CONFIG_YGV619_FB_BASE + YGV619_PA1_OFF + (x))

#define YGV619_PAL_OFF 0x00200020UL
#define YGV619_PAL(x) (CONFIG_YGV619_FB_BASE + YGV619_PAL_OFF + (x))

#endif				/* __LINUX_VIDEO_YGV619_H */
