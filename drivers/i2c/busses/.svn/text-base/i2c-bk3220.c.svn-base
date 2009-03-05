/*
 * Copyright © 2005 Raza Microelectronics, Inc. (.RMI.)
 *
 * This program is free software.  You may use it, redistribute it 
 * and/or modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation; either version two of the 
 * License or (at your option) any later version.
 *
 * This program is distributed in the hope that you will find it useful.  
 * Notwithstanding the foregoing, you understand and agree that this program 
 * is provided by RMI .as is,. and without any warranties, whether express, 
 * implied or statutory, including without limitation any implied warranty of 
 * non-infringement, merchantability or fitness for a particular purpose.  
 * In no event will RMI be liable for any loss of data, lost profits, cost 
 * of procurement of substitute technology or services or for any direct, 
 * indirect, incidental, consequential or special damages arising from the 
 * use of this program, however caused.  Your unconditional agreement to 
 * these terms and conditions is an express condition to, and shall be deemed 
 * to occur upon, your use, redistribution and/or modification of this program.
 *
 * See the GNU General Public License for more details.  
 *  i2c-palm-bk3220.c driver for the BK-3220 Host Adapter on the
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/wait.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-palm.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/rmi/iomap.h>
#include <asm/rmi/sim.h>

#include "../algos/i2c-algo-palm.h"

#undef 	DEBUG

#define PHOENIX_CPLD_PHYS_ADDR	0xbd850000


static wait_queue_head_t palm_wait;

static void	
palm_bk3220_write(int reg, int val)
{
	phoenix_reg_t *mmio;
	/* Code to access the low-level
	 * I2C Block on the RMI Phoenix 
	 */		
	if(xlr_revision_a0())
		*(__u16 *)(PHOENIX_CPLD_PHYS_ADDR+reg) = (__u16)(val);
	else {
		/* Only Bus 1  is supported - RTC is on this */
		mmio = phoenix_io_mmio(PHOENIX_IO_I2C_1_OFFSET);
		(*(uint32_t *)((ulong)mmio + reg)) = (__u32)val;
	}
}

static unsigned int
palm_bk3220_read(int reg)
{
	phoenix_reg_t *mmio;
	/* Code to access the low-level
	 * I2C Block on the RMI Phoenix 
	 */		
	if(xlr_revision_a0()) {
		__u32 retVal=
		be16_to_cpu(((__u16)*(__u16 *)(PHOENIX_CPLD_PHYS_ADDR+reg)));
		return retVal;
	} else {
		/* Only Bus 1  is supported - RTC is on this */
		mmio = phoenix_io_mmio(PHOENIX_IO_I2C_1_OFFSET);
		return (uint32_t)(*(uint32_t *)((ulong)mmio + reg));
	}
}


/* Only Bus 1  is supported - RTC is on this */
static struct i2c_algo_palm_data palm_bk3220_data = {
	.write		= palm_bk3220_write,
	.read		= palm_bk3220_read,
};

/* This is our i2c_adapter structure */
static struct i2c_adapter palm_bk3220_ops = {
	.owner          = THIS_MODULE,
	.id		= I2C_HW_PALM_BK3220,			
	.algo_data	= &palm_bk3220_data,
	.name		= "Palm Chip BK3220 Adapter",
};

static int __init palm_bk3220_init(void)
{
	init_waitqueue_head(&palm_wait);

	if (i2c_palm_add_bus(&palm_bk3220_ops) < 0) {
		printk(KERN_ERR "i2c-palm-bk3220: Failed to add i2c bus\n");
		goto out;
	}
	printk("i2c-palm-bk3220: Added I2C Bus.\n");
	return 0;
out:
	return -ENODEV;
}

static void palm_bk3220_exit(void)
{
	i2c_palm_del_bus(&palm_bk3220_ops);
}

MODULE_DESCRIPTION("BK3220 I2C Host adapter driver");
MODULE_LICENSE("GPL");

module_init(palm_bk3220_init);
module_exit(palm_bk3220_exit);
