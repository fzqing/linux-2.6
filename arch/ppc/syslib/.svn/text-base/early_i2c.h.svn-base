/* arch/ppc/syslib/early_i2c.h
 *
 * Copyright (c) Freescale Semiconductor, Inc. 2006. All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#ifndef _EARLY_I2C_H
#define _EARLY_I2C_H

#include <asm/mpc83xx.h>

/**********************************************************************/
/*
 * for i2c operation
 */
#define CFG_I2C_SPD	400000
/* used when i2c controller as a a slave */
#define CFG_I2C_SLV	0x7E

#define TIMEOUT 1000
#define CFG_M83xx_I2C1	1
#define CFG_M83xx_I2C2	2

#ifdef CONFIG_MPC832X
/* for mpc832x board */
#define CFG_I2C CFG_M83xx_I2C1
#else
/* For mpc8349 pilot board */
#define CFG_I2C	CFG_M83xx_I2C2
#endif

//#define CFG_IMMR	IMMRBAR
/* should do ioremap() in fact */
#define CFG_IMMR	VIRT_IMMRBAR

#if (CFG_I2C == CFG_M83xx_I2C1)
#define I2C_Addr ((unsigned *)(CFG_IMMR + 0x3000))
#elif (CFG_I2C == CFG_M83xx_I2C2)
#define I2C_Addr ((unsigned *)(CFG_IMMR + 0x3100))
#endif

#define I2CADR  &I2C_Addr[0]
#define I2CFDR  &I2C_Addr[1]
#define I2CCCR  &I2C_Addr[2]
#define I2CCSR  &I2C_Addr[3]
#define I2CCDR  &I2C_Addr[4]
#define I2CDFSRR &I2C_Addr[5]

/* bits in I2CnCR */
/* i2c module is enable */
#define M83xx_CCR_MEN	0x80
/* i2c interrupt is enable */
#define M83xx_CCR_MIEN	0x40
/*0->1 generate a start, 1->0 generate a stop */
#define M83xx_CCR_MSTA	0x20
/*1 receive mode, 0 transmit mode */
#define M83xx_CCR_MTX	0x10
/*1 generate an acknowledge signal */
#define M83xx_CCR_TXAK	0x08
/* 1 generate repeat start */
#define M83xx_CCR_RSTA	0x04

/* bits in I2CnSR */
/*1 - transfer completed */
#define M83xx_CSR_MCF	0x80
/*1 -addressed as a slave */
#define M83xx_CSR_MAAS	0x40
/* 1- i2c bus is busy */
#define M83xx_CSR_MBB	0x20
/* 1- arbitration is lost */
#define M83xx_CSR_MAL	0x10
/* 1 -master read from slave 0- master write to slave */
#define M83xx_CSR_SRW	0x04
/* 1 - interrupt is pending */
#define M83xx_CSR_MIF	0x02
/* 1- acknowledge received */
#define M83xx_CSR_RXAK	0x01

#define I2C_READ  1
#define I2C_WRITE 0

extern void early_i2c_init (int speed, int slaveadd);
extern int early_i2c_write (u8 dev, uint addr, int alen, u8 * data, int length);
extern int early_i2c_read (u8 dev, uint addr, int alen, u8 * data, int length);


#if defined EARLY_I2C_DBG
#	define EARLY_I2C_PRINTK(fmt, args...) 	printk("\n[%s]:[%s]:[line%d]----"fmt, __FILE__,__FUNCTION__, __LINE__, ## args)
#	define EARLY_I2C_LOC			printk(KERN_ERR"\nCurrent Location [%s]:[%d]\n", __FILE__, __LINE__)
#	define EARLY_I2C_FUNC_START		printk(KERN_ERR"\n[%s]:start!\n", __FUNCTION__)
#	define EARLY_I2C_FUNC_END			printk(KERN_ERR"\n[%s]:end!\n", __FUNCTION__)
#	define EAELY_I2C_CLUE(arg)		printk(" %s\n",arg)

#else
#	define EARLY_I2C_PRINTK(fmt, args...)
#	define EARLY_I2C_LOC
#	define EARLY_I2C_FUNC_START
#	define EARLY_I2c_FUNC_END
#	define EARLY_I2C_CLUE(arg)
#endif
#endif /* _EARLY_I2C_H */
