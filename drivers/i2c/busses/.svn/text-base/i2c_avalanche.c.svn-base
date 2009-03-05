/*
 * Copyright: (C) 2005 Texas Instruments, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/version.h>
#include <linux/i2c.h>
#include <asm/irq.h>
#include <asm/io.h>
	
#include <asm/uaccess.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/moduleparam.h>
	
#include <pal.h>
#include <ssp_hal.h>
#include <asm/semaphore.h>
	
#define MODULE_NAME	"AVALANCHE I2C"
#define MAX_MESSAGES	65536   /* max number of messages */
	
static int clock = 100; /* Default: Fast Mode = 400 KHz, Standard Mode = 100 KHz */
	
typedef struct 
{
	ssp_hal_i2c_info_t   *id;      
	u8 * buf;
	size_t buf_len;
} avalanche_i2c_dev_t;
	
static avalanche_i2c_dev_t *avalanche_i2c_dev;
	
static int avalanche_i2c_setup(avalanche_i2c_dev_t *avalanche_i2c_dev)
{
	ssp_drv_desc_t ret;
	
	if((ret = ssp_i2c_open()) == NULL) {
		return -EFAULT;            
	}
	
	avalanche_i2c_dev->id = (ssp_hal_i2c_info_t *)ret;
	
	return 0;
}
	
static irqreturn_t avalanche_i2c_isr(int this_irq, void *dev_id, struct pt_regs *regs)
{
	ssp_hal_intr_ctrl(SSP_HAL_INTR_DISABLE);
	ssp_hal_intr_ctrl(SSP_HAL_INTR_ACK);
	
	ssp_isr();
	ssp_hal_intr_ctrl(SSP_HAL_INTR_ENABLE);
	
	return IRQ_HANDLED;
}      
	
static int avalanche_i2c_xfer_msg(struct i2c_adapter * adap, struct i2c_msg * msg, int stop)
{
	avalanche_i2c_dev_t *dev = i2c_get_adapdata(adap);
	u8 zero_byte = 0;
	int ret = 0;
	int count = 0;
	
	if (!msg->len) {
		return 0;
	}
	
	avalanche_i2c_setup(dev);
	dev->id->addr = msg->addr;
	
	if (dev->id->mode == SSP_HAL_MODE_INTR){
		request_irq(LNXINTNUM(AVALANCHE_SSP_INT), avalanche_i2c_isr, 0, "i2c", dev);
		avalanche_intr_type_set(LNXINTNUM(AVALANCHE_SSP_INT),0);  
	}
	
	if (msg->len == 0) {
		dev->buf = &zero_byte;
		dev->buf_len = 1;
	}
	
	if (msg->flags != I2C_M_RD){
		if((count = ssp_i2c_write((ssp_hal_i2c_info_t*)dev->id, msg->buf, msg->len)) != msg->len ) {
			printk("SSP_I2C: Failed to transmit for  addr: 0x%04x, len: %d, flags: 0x%x, stop: %d \n",
						msg->addr, msg->len, msg->flags, stop);
			ret = count;
		}
	
	}
	else {
		if((count = ssp_i2c_read((ssp_hal_i2c_info_t *)dev->id, msg->buf, msg->len)) != msg->len ) {
			printk("SSP_I2C: Failed to transmit for  addr: 0x%04x, len: %d, flags: 0x%x, stop: %d \n",
						msg->addr, msg->len, msg->flags, stop);

			ret = count;
		}
	}
	
	if (dev->id->mode == SSP_HAL_MODE_INTR) {
		free_irq(LNXINTNUM(AVALANCHE_SSP_INT), dev);
	}
	
	ssp_i2c_close(dev->id);    
	
	return count;
}

static int avalanche_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	int i;
	int r = 0;
	
	if (num < 1 || num > MAX_MESSAGES)
		return -EINVAL;
	
	for (i = 0; i < num; i++)
		if (msgs[i].buf == NULL)
			return -EINVAL;
	
	for (i = 0; i < num; i++) {
		r = avalanche_i2c_xfer_msg(adap, &msgs[i], (i == (num - 1)));
		if (r != msgs[i].len)
			break;
	}
	
	if (r >= 0 && num > 1)
		r = num;
	
	return r;
}
	
static u32 avalanche_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}
	
static struct i2c_algorithm avalanche_i2c_algo = { 
	name              : "Avalanche I2C algorithm", 
	id                : I2C_ALGO_EXP, 
	master_xfer       : avalanche_i2c_xfer, 
	smbus_xfer        : NULL, 
	slave_send        : NULL, 
	slave_recv        : NULL, 
	algo_control      : NULL, 
	functionality     : avalanche_i2c_func,
};
	
static struct i2c_adapter avalanche_i2c_adap = { 
	owner             : THIS_MODULE, 
	name              : "AVALANCHE I2C adapter", 
	id                : I2C_ALGO_EXP, 
	algo              : &avalanche_i2c_algo, 
	algo_data         : NULL, 
	client_register   : NULL, 
	client_unregister : NULL,
};
	
static int __init avalanche_i2c_init(void)
{
	int       r;
	
	clock = 400;    /* Fast mode */
	
	avalanche_i2c_dev = kmalloc(sizeof(avalanche_i2c_dev_t), GFP_KERNEL); 
	
	if(avalanche_i2c_dev == NULL) {
		printk( "SSP_I2C: Unable to allocate memory for I2C.\n" );
		return -1;
	}
	
	avalanche_i2c_dev->id = NULL;
	i2c_set_adapdata(&avalanche_i2c_adap, &avalanche_i2c_dev);
	r = i2c_add_adapter(&avalanche_i2c_adap);
	
	if (r) {
		printk("failed to add adapter");
		return r;
	}
	
	return 0;
}
	
static void __exit avalanche_i2c_exit(void)
{
	i2c_del_adapter(&avalanche_i2c_adap);
	
	if (avalanche_i2c_dev)
		kfree(avalanche_i2c_dev);
} 

MODULE_AUTHOR("Texas Instruments");
	
MODULE_DESCRIPTION("AVALANCHE I2C bus adapter");
MODULE_LICENSE("GPL");
MODULE_PARM_DESC(clock, "Set I2C clock in KHz: 100 (Standard Mode) or 400 (Fast Mode)");
	
module_init(avalanche_i2c_init);
module_exit(avalanche_i2c_exit); 
