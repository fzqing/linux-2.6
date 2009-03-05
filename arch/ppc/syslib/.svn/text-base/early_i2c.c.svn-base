/*******************************************************************
 * early_i2c.c
 *
 * i2c operation while system booting
 *
 * Author: Tony Li (r64360@freescale.com)
 * based on mpc83xx_i2c.c by Dave Liu
 * based on Hardware I2C driver for mpc107 PCI bridge
 * 	by Gleb Natapov <gnatapov@mrv.com>
 * Some bits are taken from linux driver writen by adrian@humboldt.co.uk
 *
 * Copyright (C) Freescale Semiconductor, Inc. 2006. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 ********************************************************************/

#include <asm/mpc83xx.h> /* for IMMRBAR */
#include <asm/io.h>  /* for in_le32() and out_len32 */
#include <linux/delay.h> /* for mdelay */
#include <linux/module.h>
#include "./early_i2c.h"

void early_i2c_init (int speed, int slaveadd)
{
	/* stop I2C controller */
	writel (0x0, I2CCCR);
	/* set clock */
	writel (0x3f, I2CFDR);
	/* set default filter */
	writel (0x10,I2CDFSRR);
	/* write slave address */
	writel (slaveadd, I2CADR);
	/* clear status register */
	writel (0x0, I2CCSR);
	/* start I2C controller */
	writel (M83xx_CCR_MEN, I2CCCR);

	return;
}

/* return
 *	 0 sucess      -1 failed
 */
static __inline__ int i2c_wait4bus (void)
{
	ulong current_time;

	current_time = 0;
	while (readl (I2CCSR) & M83xx_CSR_MBB)
	{
		mdelay(10);
		current_time++;
		if ( current_time > TIMEOUT )
			return -1;
	}
	return 0;
}

/*
 * i2c_write(write/read flag)
 * this function is waiting for the transmite completed
 * in time limits and without problem
 * return
 * 0 success   1 fail
 */
static __inline__ int i2c_wait (int write)
{
	u32 csr;
	ulong start_time ;

	start_time = 0;

	do {
		csr = readl (I2CCSR);
		/* if no interrupt pending occurs */
		if (!(csr & M83xx_CSR_MIF))
		{
			mdelay(10);
			start_time++;
			continue;
		}

		writel (0x0, I2CCSR);

		/* if arbitration lost*/
		if (csr & M83xx_CSR_MAL)
			return -1;
		/* if transfer isnot completed */
		if (!(csr & M83xx_CSR_MCF))
			return -1;

		/* i2c write operation and acknowedge received */
		if (write == I2C_WRITE && (csr & M83xx_CSR_RXAK))
			return -1;

		return 0;

	} while (start_time < TIMEOUT);

	return -1;
}

/* i2c_write_addr(device,  WRITE/READ,  repeat start flag)
 *  1 success  0 failed
 */
static __inline__ int i2c_write_addr (u8 dev, u8 dir, int rsta)
{
	/* i2c module enable| generate START | transmit mode | repeat start flag*/
	writel (M83xx_CCR_MEN | M83xx_CCR_MSTA | M83xx_CCR_MTX |
			(rsta ? M83xx_CCR_RSTA : 0), I2CCCR);

	writel ((dev << 1) | dir, I2CCDR);

	if (i2c_wait (I2C_WRITE) < 0)
		return 0;

	return 1;
}

static __inline__ int __i2c_write (u8 * data, int length)
{
	int i;

	/* i2c module enable | generate START| transmit mode */
	writel (M83xx_CCR_MEN | M83xx_CCR_MSTA | M83xx_CCR_MTX, I2CCCR);

	for (i = 0; i < length; i++) {
		writel (data[i], I2CCDR);

		if (i2c_wait (I2C_WRITE) < 0)
			break;
	}

	return i;
}

static __inline__ int __i2c_read (u8 * data, int length)
{
	int i;

	writel (M83xx_CCR_MEN | M83xx_CCR_MSTA |
			((length == 1) ? M83xx_CCR_TXAK : 0), I2CCCR);

	/* dummy read */
	readl (I2CCDR);

	for (i = 0; i < length; i++) {
		if (i2c_wait (I2C_READ) < 0)
			break;

		/* Generate ack on last next to last byte */
		if (i == length - 2)
			writel (M83xx_CCR_MEN | M83xx_CCR_MSTA |
					M83xx_CCR_TXAK, I2CCCR);

		/* Generate stop on last byte */
		if (i == length - 1)
			writel (M83xx_CCR_MEN | M83xx_CCR_TXAK, I2CCCR);

		data[i] = readl (I2CCDR);
	}

	return i;
}

int early_i2c_read (u8 dev, uint addr, int alen, u8 * data, int length)
{
	int i = 0;
	u8 *a = (u8 *) & addr;

	/* wait while i2c bus is busy */
	if (i2c_wait4bus () < 0)
		goto exit;

	/* transmite the slave_addr+read/write phase */
	if (i2c_write_addr (dev, I2C_WRITE, 0) == 0)
		goto exit;

	/* select the i2c slave internal regs phase */
	/* because u32 -> u8, and big endian, so if alen is 1, write a[3] */
	if (__i2c_write (&a[4 - alen], alen) != alen)
		goto exit;
	/* dummy read phase */
	if (i2c_write_addr (dev, I2C_READ, 1) == 0)
		goto exit;

	/* transmite data phase */
	i = __i2c_read (data, length);

exit:
	writel (M83xx_CCR_MEN, I2CCCR);

	return !(i == length);
}

/* i2c_write(slave addr,
 *            reg No of this slave,
 *            reg size, 1 for 8bits, 2 for 16bits..4 for 32bits reg
 *            date to be write,
 *            date len as bytes)
 * 0 success        1 failed
 */
int early_i2c_write (u8 dev, uint addr, int alen, u8 * data, int length)
{
	int i = 0;
	u8 *a = (u8 *) & addr;

	/* wait while i2c bus is busy */
	if (i2c_wait4bus () < 0)
		goto exit;
	/* transmite the slave_addr+read/write phase */
	if (i2c_write_addr (dev, I2C_WRITE, 0) == 0)
		goto exit;
	/* select the i2c slave internal regs phase */
	/* because u32 -> u8, and big endian, so if alen is 1, write a[3] */
	if (__i2c_write (&a[4 - alen], alen) != alen)
		goto exit;

	/* transmite data phase */
	i = __i2c_write (data, length);

exit:
	writel (M83xx_CCR_MEN, I2CCCR);

	return !(i == length);
}
EXPORT_SYMBOL(early_i2c_write);
