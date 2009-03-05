/*
 * cir.c - Driver for Consumer Infrared (CIR) (on Davinci-HD EVM)
 *
 * Copyright (C) 2007  Texas Instruments, India
 * Author: Suresh Rajashekara <suresh.r@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/jiffies.h>
#include <asm/arch/cir.h>
#include <asm/arch/i2c-client.h>

#define MODULE_NAME   "Consumer IR"

/* Debug levels */
#define LOW      25
#define MEDIUM   50
#define HIGH     75
#define CRITICAL 100

/* Undefine/define this to disable/enable debugging */
/* #define CIR_DEBUG */

#ifdef CIR_DEBUG
static s8 cir_debug_level = LOW;
#define CIR_DEBUG0(format, arg...)		\
		printk(KERN_ALERT MODULE_NAME " DEBUG: " format "\n",  ## arg )
#define CIR_DEBUG1(level, format, arg...)	do {\
	if (level <= cir_debug_level) {	\
		printk(KERN_ALERT MODULE_NAME " DEBUG: " format "\n",  \
		## arg ); \
	} } while (0)
#else  /* CIR_DEBUG */
#define CIR_DEBUG0(fmt, args...)
#define CIR_DEBUG1(level, fmt, args...)
#endif /* CIR_DEBUG */

/* Undefine/define this to disable/enable tracing */
/* #define CIR_TRACE */

#ifdef CIR_TRACE

#ifndef CIR_DEBUG
#define CIR_DEBUG
#endif /* CIR_DEBUG */

#define CIR_FN_IN   \
	printk(KERN_ALERT MODULE_NAME \
		" DEBUG: Entering Function %s\n", __FUNCTION__ )
#define CIR_FN_OUT(retval)  \
	printk(KERN_ALERT MODULE_NAME \
		" DEBUG: Leaving Function %s (Ret = %d)\n", \
		__FUNCTION__, retval )
#else  /* CIR_TRACE */
#define CIR_FN_IN
#define CIR_FN_OUT(retval)
#endif /* CIR_TRACE */

#define cir_info(format, arg...) \
	printk(KERN_INFO MODULE_NAME " INFO: " format "\n",  ## arg )
#define cir_warn(format, arg...) \
	printk(KERN_WARNING MODULE_NAME " WARNING: " format "\n",  ## arg )
#define cir_err(format, arg...) \
	printk(KERN_ERR MODULE_NAME " ERROR: " format "\n",  ## arg )

/* Globals */
static dev_t cir_dev;
static u32 cir_major_number;
static u32 cir_minor_number;
static struct cdev cir_cdev;
static atomic_t reference_count = ATOMIC_INIT(0);
static DECLARE_MUTEX(mutex);

#define RAW_KEY_CODES 4
u8 temp[RAW_KEY_CODES] = {0};
u8 cir_key_idx = 0;

/* If another key press is detected within this jiffies duration, then the
 * driver discards the key press. */
#define KEY_REPEAT_DURATION 20	/* Value in Jiffies */

#define MAX_KEYS_IN_BUFFER  4
static u8 key_buf[MAX_KEYS_IN_BUFFER][RAW_KEY_CODES];

static int key_read;
static int key_write;
static unsigned long last_key_time;

DECLARE_COMPLETION(cir_keys);

static u32 cir_reg_get ( u32 reg );
static void cir_reg_set ( u32 reg, u8 reg_value );

static int cir_remove(struct device *device)
{
	return 0;
}

static void cir_platform_release(struct device *device)
{
	/* this function does nothing */
}


static struct class_simple *cir_class;

static struct platform_device cir_device = {
	.name = "cir",
	.id = 0,
	.dev = {
		.release = cir_platform_release,
	}
};

static struct device_driver cir_driver = {
	.name = "cir",
	.bus = &platform_bus_type,
	.remove = cir_remove
};

static u16 decode_cir_value ( u8 *raw_cir_data )
{
	u16 i;
	u32 acc;
	u16 lowbit;
	u16 highbit;
	u16 cir_value = 0;

	acc = ( raw_cir_data[0] << 0 )
		| ( raw_cir_data[1] << 8 )
		| ( raw_cir_data[2] << 16 )
		| ( raw_cir_data[3] << 24 );

	acc = acc >> 1;

	for ( i = 0 ; i < 16 ; i++ ) {
		cir_value <<= 1;

		/* Low & High bits */
		lowbit  = ( acc & 0x0002 );
		highbit = ( acc & 0x0001 );

		/* One bit */
		if ( ( highbit == 0 ) && ( lowbit != 0 ) )
			cir_value |= 1;

		acc = acc >> 2;
	}

	return ( cir_value >> 3 ) & 0x0fff;
}

static inline s8 get_stored_key (u16 *key)
{
	int ret = 0;

	down_interruptible(&mutex);

	if (key_read == key_write) {
		ret = -1;
	} else {
		*key = decode_cir_value(key_buf[key_read]);

		if (++key_read >= MAX_KEYS_IN_BUFFER) {
			key_read = 0;
		}
	}

	up(&mutex);

	return ret;
}

static inline s8 store_received_key ( void )
{
	s8 ret_val = -1;

	down_interruptible(&mutex);

	if ( (jiffies - last_key_time) > KEY_REPEAT_DURATION ) {
		memcpy (&key_buf[key_write], temp, RAW_KEY_CODES);

		if (++key_write >= MAX_KEYS_IN_BUFFER) {
			key_write = 0;
		}

		last_key_time = jiffies;
		ret_val = 0;
	}

	cir_key_idx = 0;
	memset(temp, 0, RAW_KEY_CODES);

	up(&mutex);
	return ret_val;
}

ssize_t cir_read (struct file *file, char __user *buff, size_t size, loff_t
		  *loff)
{
	u16 key;

	INIT_COMPLETION(cir_keys);

	if (size != sizeof(u16)) {
		cir_err ("Invalid size requested for read\n");
		return -EFAULT;
	}

	if (file->f_flags & O_NONBLOCK) {
		if (get_stored_key(&key) < 0) {
			return -EAGAIN;
		}
	} else {
		if (get_stored_key(&key) < 0) {
			wait_for_completion_interruptible(&cir_keys);
			get_stored_key(&key);
		}
	}

	if (copy_to_user(buff, &key, sizeof(u16)) != 0 ) {
		return -EFAULT;
	}

	return size;
}

static u32 cir_reg_get ( u32 reg )
{
	u32 reg_value;
	u32 lcr = 0;
	u32 preg32 = IO_ADDRESS((CIR_BASE) + (reg & ~(USE_LCR_80|USE_LCR_BF)));
	u32 set_lcr_80 = ((u32)reg) & USE_LCR_80;
	u32 set_lcr_bf = ((u32)reg) & USE_LCR_BF;

	/* Set LCR if needed */
	if ( set_lcr_80 ) {
		lcr = cir_reg_get( LCR );
		cir_reg_set( LCR, 0x80 );
	}

	if ( set_lcr_bf ) {
		lcr = cir_reg_get( LCR );
		cir_reg_set( LCR, 0xbf );
	}

	/* Get UART register */
	reg_value = *( volatile u32 * )( preg32 );

	/* Return LCR reg if necessary */
	if ( set_lcr_80 || set_lcr_bf ) {
		cir_reg_set( LCR, lcr );
	}

	return reg_value;
}

static void cir_reg_set ( u32 reg, u8 reg_value )
{
	u32 lcr = 0;
	u32 preg32 = IO_ADDRESS((CIR_BASE) + (reg & ~(USE_LCR_80|USE_LCR_BF)));
	u32 set_lcr_80 = ((u32)reg) & USE_LCR_80;
	u32 set_lcr_bf = ((u32)reg) & USE_LCR_BF;

	/* Set LCR if needed */
	if ( set_lcr_80 ) {
		lcr = cir_reg_get( LCR );
		cir_reg_set( LCR, 0x80 );
	}

	if ( set_lcr_bf ) {
		lcr = cir_reg_get( LCR );
		cir_reg_set( LCR, 0xbf );
	}

	/* Set UART register */
	*( volatile u32 * )( preg32 ) = reg_value;

	/* Return LCR reg if necessary */
	if ( set_lcr_80 || set_lcr_bf ) {
		cir_reg_set( LCR, lcr );
	}
}

static void configure_cir_registers (void)
{
	davinci_i2c_expander_op(0x3a, CIR_MOD_DM646X, 0);

	cir_reg_set( EFR, 0x10 );
	cir_reg_set( IER, 0 );
	cir_reg_set( MCR, 0 );
	cir_reg_set( EFR, 0 );
	cir_reg_set( LCR, 0 );
	cir_reg_set( MDR1, 0x07 );
	cir_reg_set( LCR, 0xbf );
	cir_reg_set( IIR, 0x10 );
	cir_reg_set( LCR, 0x87 );
	cir_reg_set( IER, 0x05 );
	cir_reg_set( RHR, 0x35 );
	cir_reg_set( LCR, 0x06 );
	cir_reg_set( MCR, 0x01 );
	cir_reg_set( IER, 0x01 );
	cir_reg_set( EBLR, 0x05 );
	cir_reg_set( SFLSR, 0x01 );
	cir_reg_set( RESUME, 0x00 );
	cir_reg_set( SFREGL, 0x04 );
	cir_reg_set( SFREGH, 0x00 );
	cir_reg_set( LCR, 0x07 );
	cir_reg_set( CFPS, 56 );
	cir_reg_set( MDR2, 0x58 );
	cir_reg_set( MDR1, 0x06 );
}

static void cir_reset( void )
{
	cir_reg_set( EFR, 0x10 );
	cir_reg_set( IER, 0 );
	cir_reg_set( MCR, 0 );
	cir_reg_set( EFR, 0 );
	cir_reg_set( LCR, 0 );
	cir_reg_set( MDR1, 0x07 );
	cir_reg_get ( RESUME );
}

int cir_open (struct inode *inode, struct file *file)
{
	if (file->f_mode == FMODE_WRITE) {
		cir_err ("TX Not supported\n");
		return -EACCES;
	}

	if (atomic_inc_return(&reference_count) > 1) {
		atomic_dec(&reference_count);
		return -EACCES;
	}

	cir_reset();
	configure_cir_registers();
	return 0;
}

int cir_release (struct inode *inode, struct file *file)
{
	complete_all(&cir_keys);
	udelay(5);
	cir_reset();

	atomic_dec(&reference_count);
	return 0;
}

static irqreturn_t cir_irq_handler (int irq, void *dev_id, struct pt_regs *regs)
{
	temp[cir_key_idx] = cir_reg_get(RHR);

	if ( cir_key_idx == RAW_KEY_CODES-1 ) {
		if ( store_received_key() == 0 ) {
			complete (&cir_keys);
		}
	} else {
		cir_key_idx++;
	}

	while ((cir_reg_get(IIR) & 0x01) == 1) {
		cir_reg_get(RHR);
	}

	cir_reg_get ( RESUME );

	return IRQ_HANDLED;
}

int cir_ioctl ( struct inode *inode, struct file *filp, unsigned int cmd,
		unsigned long arg )
{
	switch ( cmd ) {
	default:
		cir_err ("Unknow IOCTL command\n");
		break;
	}

	return -ENOTTY;
}

static struct file_operations cir_fops = {
	.owner   = THIS_MODULE,
	.read    = cir_read,
	.open    = cir_open,
	.release = cir_release,
	.ioctl   = cir_ioctl,
};

static int __init cir_init ( void )
{
	s8 retval = 0;
	struct clk *clkp;
	last_key_time = 0;

	cir_major_number = 0;
	cir_minor_number = 0;
	cir_class = NULL;

	CIR_FN_IN;

	cir_key_idx = 0;
	memset(temp, 0, sizeof(u8)*RAW_KEY_CODES);

	clkp = clk_get (NULL, "UART2");

	if ( IS_ERR ( clkp ) ) {
		cir_err ("Unable to get the clock for CIR\n");
		goto failure;
	} else {
		clk_use (clkp);
		clk_enable (clkp);
	}

	if (cir_major_number) {	/* If major number is specified */
		cir_dev = MKDEV(cir_major_number, 0);
		retval = register_chrdev_region (cir_dev,
						 CIR_DEV_COUNT,
						 "/dev/cir");
	} else {			/* If major number is not specified */
		retval = alloc_chrdev_region (&cir_dev,
					      cir_minor_number,
					      CIR_DEV_COUNT,
					      "/dev/cir");
		cir_major_number = MAJOR(cir_dev);
	}

	if (retval < 0) {
		cir_err ("Unable to register the CIR device\n");
		retval = -ENODEV;
		goto failure;
	} else {
		cir_info ("CIR device registered successfully \
			  (Major = %d, Minor = %d)",
			  MAJOR(cir_dev), MINOR(cir_dev) );
	}

	cdev_init (&cir_cdev, &cir_fops);
	cir_cdev.owner = THIS_MODULE;
	cir_cdev.ops = &cir_fops;

	/* You should not call cdev_add until your driver is completely ready to
	   handle operations on the device*/
	retval = cdev_add (&cir_cdev, cir_dev, CIR_DEV_COUNT);

	if (retval) {
		unregister_chrdev_region (cir_dev, CIR_DEV_COUNT);
		cir_err ("Error %d adding CIR device\n", retval);
		goto failure;
	}

	retval = request_irq (CIR_IRQ, cir_irq_handler,
			      SA_INTERRUPT|SA_SAMPLE_RANDOM, "Consumer IR",
			      NULL);
	if (retval) {
		unregister_chrdev_region (cir_dev, CIR_DEV_COUNT);
		cdev_del (&cir_cdev);
		cir_err ("Unable to register CIR IRQ %d\n", CIR_IRQ );
		goto failure;
	}

	cir_class = class_simple_create(THIS_MODULE, "cir");

	if (!cir_class) {
		unregister_chrdev_region (cir_dev, CIR_DEV_COUNT);
		cdev_del(&cir_cdev);
		goto failure;
	}

	if (driver_register(&cir_driver) != 0) {
		unregister_chrdev_region (cir_dev, CIR_DEV_COUNT);
		cdev_del(&cir_cdev);
		class_simple_destroy(cir_class);
		goto failure;
	}
	/* Register the drive as a platform device */
	if (platform_device_register(&cir_device) != 0) {
		driver_unregister(&cir_driver);
		unregister_chrdev_region (cir_dev, CIR_DEV_COUNT);
		cdev_del(&cir_cdev);
		class_simple_destroy(cir_class);
		goto failure;
	}

	cir_dev = MKDEV(cir_major_number, 0);

	class_simple_device_add(cir_class, cir_dev, NULL, "cir");

	cir_info ("CIR IRQ %d registered successfully", CIR_IRQ);

	key_read = 0;
	key_write = 0;

 failure:
	CIR_FN_OUT(retval);
	return retval;
}

static void __exit cir_exit ( void )
{
	CIR_FN_IN;

	cir_info ("Unregistering the CIR device");
	free_irq (CIR_IRQ, NULL);
	unregister_chrdev_region (cir_dev, CIR_DEV_COUNT);
	/* You should not access the cdev structure after
	 * passing it to cdev_del. */
	cdev_del (&cir_cdev);

	driver_unregister(&cir_driver);
	class_simple_destroy(cir_class);
	platform_device_unregister(&cir_device);
	cir_dev = MKDEV(cir_major_number, 0);
	class_simple_device_remove(cir_dev);

	CIR_FN_OUT(0);
	return;
}

module_init (cir_init);
module_exit (cir_exit);

MODULE_AUTHOR ( "Texas Instruments" );
MODULE_DESCRIPTION ( "Consumer Infrared (CIR) Driver" );
MODULE_LICENSE ( "GPL" );
