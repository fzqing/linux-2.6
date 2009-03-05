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
 *
 *  i2c-algo-palm.c i2c driver algorithms for the BK3220 I2C Host 
 *  adapter on the RMI Phoenix System.
 *  Derived from the PCA-ISA I2C-Algo/Bus files.
 *    
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-palm.h>
#include "i2c-algo-palm.h"

#define DRIVER "i2c-algo-palm"

#define DEB1(fmt, args...) do {if(i2c_debug>=1) printk(fmt, ## args);} while(0)
#define DEB2(fmt, args...) do {if(i2c_debug>=2) printk(fmt, ## args);} while(0)
#define DEB3(fmt, args...) do {if(i2c_debug>=3) printk(fmt, ## args);} while(0)

static int i2c_debug=0;
spinlock_t palm_lock;
					
#define palm_write(algo_data, reg, val) 	algo_data->write(reg, val)
#define palm_read(algo_data, reg) 		algo_data->read(reg)

#define palm_clock(adap) 		adap->get_clock(adap)
#define palm_status(adap) 		palm_inw(adap, I2C_PCA_STA)
#define palm_set_con(adap, val) 	palm_outw(adap, I2C_PCA_CON, val)
#define palm_get_con(adap) 		palm_inw(adap, I2C_PCA_CON)

/*
 * Check if the I2C Bus is idle or busy
 */
static int wait_for_idle(struct i2c_algo_palm_data *algo_data)
{
	int timeOut=0x10;
	volatile __u32 regVal=0x00;
	regVal = palm_read(algo_data, I2C_PALM_STATUS) & 0x0001;
	while (regVal && timeOut--) {
		regVal = palm_read(algo_data, I2C_PALM_STATUS) & 0x0001;
	}
	if (timeOut == 0x00)
		return -1;	/* Timed Out */
	else
		return 0;
}

/*
 * Transmit Routine
 */
static int palm_tx(struct i2c_algo_palm_data *algo_data,  __u16 len, 
		__u8 *buf, __u16 addr, __u8 reg)
{
	volatile __u32 regVal=0x00;
	int timeOut, ctr=0x00, numBytes=len;

	for (ctr=0x00; ctr<len; ctr++) {
		palm_write(algo_data, I2C_PALM_CFG, 0xF8);			
		palm_write(algo_data, I2C_PALM_BYTECNT, 0);
		palm_write(algo_data, I2C_PALM_DEVADDR, addr);			
		palm_write(algo_data, I2C_PALM_ADDR, reg+ctr);
		palm_write(algo_data, I2C_PALM_DATAOUT, buf[ctr]);
		palm_write(algo_data, I2C_PALM_STARTXFR, I2C_PALM_STARTXFR_WR );
		spin_lock_irq(&palm_lock);
		mdelay(0x01);
		spin_unlock_irq(&palm_lock);
		
		regVal = palm_read(algo_data, I2C_PALM_STATUS);
		spin_lock_irq(&palm_lock);
		mdelay(0x01);
		spin_unlock_irq(&palm_lock);
		if (regVal & 0x0008) {
			printk("palm_tx: ACKERR. Aborting...\n");
			return -1;
		}
		timeOut= 0x10;
		while (!(regVal & 0x0002) && timeOut--) {
			regVal = palm_read(algo_data, I2C_PALM_STATUS);
		}
		if (timeOut==0x00) {
			printk("palm_tx: [TimeOut] SDOEMPTY Not Set\n");
			return -1;
		}
		timeOut=10;
		while ((regVal & 0x0030) && timeOut--) {
			palm_write(algo_data, I2C_PALM_STARTXFR, 
					I2C_PALM_STARTXFR_WR);
			regVal = palm_read(algo_data, I2C_PALM_STATUS);
		}
		if (timeOut==0x00) {
			printk("palm_rx: TimedOut on Valid" 
					"STARTXFR/Arbitration\n");
			return -1;
		}
		numBytes--;
	}
	return 0;
}
static int palm_addr_only(struct i2c_algo_palm_data *algo_data, __u8 *buf,
		__u16 addr, __u8 reg)
{
	volatile __u32 regVal=0x00;

	palm_write(algo_data, I2C_PALM_ADDR,    reg);
	palm_write(algo_data, I2C_PALM_DEVADDR, addr);
	//palm_write(algo_data, I2C_PALM_BYTECNT,  0);
	palm_write(algo_data, I2C_PALM_CFG,     0xF8);
	palm_write(algo_data, I2C_PALM_STARTXFR,0x02);
	regVal = palm_read(algo_data, I2C_PALM_STATUS);
	if (regVal & 0x0008) {
		printk("palm_rx: ACKERR. Aborting...\n");
		return -1;
	}
	return 0;
}


/*
 * Receive Routine
 * Read 'len' bytes from device @ 'addr'
 */
static int palm_rx(struct i2c_algo_palm_data *algo_data, __u16 len,
		__u8 *buf, __u16 addr, __u8 reg)
{	
	volatile __u32 regVal=0x00, ctr=0x00;
	int timeOut, numBytes=0x00;

	palm_write(algo_data, I2C_PALM_CFG, 0xFA);				
	palm_write(algo_data, I2C_PALM_BYTECNT, (len - 1));
	palm_write(algo_data, I2C_PALM_DEVADDR, addr);
	spin_lock_irq(&palm_lock);
	mdelay(0x01);
	spin_unlock_irq(&palm_lock);

	for (numBytes=0x00; numBytes < len; numBytes++) {
		spin_lock_irq(&palm_lock);
		mdelay(0x01);
		spin_unlock_irq(&palm_lock);
		if (!ctr) {
			/* Trigger a READ Transaction */
			palm_write(algo_data, I2C_PALM_STARTXFR, 
						I2C_PALM_STARTXFR_RD);
			ctr++;
		}

		/* Error Conditions [Begin] */
		regVal = palm_read(algo_data, I2C_PALM_STATUS);
		spin_lock_irq(&palm_lock);
		mdelay(0x01);
		spin_unlock_irq(&palm_lock);
		if (regVal & 0x0008) {
			printk("palm_rx: ACKERR. Aborting...\n");
			return -1;
		}
		timeOut=10;
		while ((regVal & 0x0030) && timeOut--) {
			palm_write(algo_data, I2C_PALM_STARTXFR, 
						I2C_PALM_STARTXFR_RD);
			regVal = palm_read(algo_data, I2C_PALM_STATUS);
		}
		if (timeOut==0x00) {
			printk("palm_rx: TimedOut on Valid" 
					"STARTXFR/Arbitration\n");
			return -1;
		}
		timeOut=10;
		/* Do we have valid data from the device yet..? */
		regVal &= 0x0004;
		while (!regVal && timeOut--) {
			regVal = palm_read(algo_data, I2C_PALM_STATUS) & 0x0004;
		}
		if (timeOut==0x00) {
			printk("palm_rx: TimedOut Waiting for Valid Data\n");
			return -1;
		}
		/* Error Conditions [End] */
		/* Read the data */
		buf[numBytes] = (__u8)palm_read(algo_data, I2C_PALM_DATAIN);
	}
	return 0;
}

static int palm_xfer(struct i2c_adapter *i2c_adap,
		struct i2c_msg msgs[],
		int num)
{
	struct 	i2c_algo_palm_data *algo_data = i2c_adap->algo_data;
	struct 	i2c_msg *msg = NULL;
	int 	curmsg;
	uint8_t reg = msgs[0].buf[0]; /* register offset */

	for (curmsg = 0; curmsg < num; curmsg++) {

		int addr;
		msg = &msgs[curmsg];

		addr = (0x7f & msg->addr);

		/*
		 * Check if I2C State Machine is idle
		 * 'wait_for_idle' returns < 0 => timedOut
		 * 'BUSY' bit cleared => BUS is IDLE
		 */
		if (wait_for_idle(algo_data) < 0) {
			printk("TimedOut on Waiting for I2C Bus Idle.\n");
			return -EIO;
		}
		if (msg->flags & I2C_M_RD ) {
			if (palm_addr_only(algo_data, &msg->buf[0], 
						addr, reg) == -1) {
				printk("I2C ADDRONLY Phase Fail.\n");
				return -EIO;
			}
			if (palm_rx(algo_data, msg->len, 
					&msg->buf[0], addr, reg) == -1) {
				printk("I2C Read Fail.\n");
				return -EIO;
			}
		}
		else {
			if(msg->len <= 1) continue; /* not a write req */

			if (palm_tx(algo_data, (msg->len - 1),
					&msg->buf[1], addr, reg) == -1) {
				printk("I2C Write Fail.\n");
				return -EIO;
			}
		}
	}
	return num;
}

static u32 palm_func(struct i2c_adapter *adap)
{
	/* We emulate SMBUS over I2C */
	return I2C_FUNC_SMBUS_EMUL;
}

static int palm_init(struct i2c_algo_palm_data *algo_data)
{
	printk("Intializing BK-3220 I2C Host Adapter...");
	spin_lock_init(&palm_lock);
#ifndef CONFIG_RMI_PHOENIX 
	/* RMI Phoenix has a hardcoded value for CLKDIV now... */
	palm_write(algo_data, I2C_PALM_CLKDIV, I2C_PALM_CLKDIV_DEF);
	/* Needed only for Multi-master environments */
	palm_write(algo_data, I2C_PALM_HDSTATIM, I2C_PALM_HDSTATIM_DEF);
#endif
	printk("done.\n");
	return 0;
}

static struct i2c_algorithm palm_algo = {
	.master_xfer	= palm_xfer,
	.functionality	= palm_func,
	.smbus_xfer	= NULL,
};

/* 
 * registering functions to load algorithms at runtime 
 */
int i2c_palm_add_bus(struct i2c_adapter *adap)
{
	struct i2c_algo_palm_data *palm_adap = adap->algo_data;
	int rval;

	adap->id |= I2C_HW_PALM_BK3220;
	adap->algo = &palm_algo;

	adap->timeout = 100;		
	adap->retries = 3;		

	rval = palm_init(palm_adap);

	/* register new adapter to i2c module... */
	if (!rval)
		i2c_add_adapter(adap);

	return rval;
}

int i2c_palm_del_bus(struct i2c_adapter *adap)
{
	return i2c_del_adapter(adap);
}

EXPORT_SYMBOL(i2c_palm_add_bus);
EXPORT_SYMBOL(i2c_palm_del_bus);

MODULE_DESCRIPTION("I2C-Bus PalmChip's Host Adapter algorithm");
MODULE_LICENSE("GPL");

module_param(i2c_debug, int, 0);
