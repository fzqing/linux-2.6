/*
 * arch/arm/mach-pxa/sleepwkr.h
 *
 * Copyright (C) 2006, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 *(C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef __SLEEPWA_H__INCLUDED

#define __SLEEPWA_H__INCLUDED

#define SIG_ENABLE	0x80000000
#define SIG_TCK		0x00000040
#define SIG_TMS		0x00000020
#define SIG_TDI		0x00000010
#define SIG_TDO		0x00000001


extern int sleep_wreg(unsigned int portal, char *bitfield);
extern int sleep_wkr_start(unsigned long reg);
extern int sleep_wkr_end(unsigned long reg);

#endif
