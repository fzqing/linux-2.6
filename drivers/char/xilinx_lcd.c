/*
 * xilinx_lcd.c
 *
 * Virtex2Pro 2x16 character LCD driver.
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * 2005 (c) MontaVista, Software, Inc.  This file is licensed under the terms
 * of the GNU General Public License version 2.  This program is licensed
 * "as is" without any warranty of any kind, whether express or implied.
 */

/*
 * Based on:
 *   Simple driver for a memory-mapped 44780-style LCD display.
 *   Configured for CMB-VR7701/Rockhopper
 *   2003 (c) MontaVista Software, Inc.
 * Which is in turn based on:
 *   Copyright 2001 Bradley D. LaRonde <brad@ltc.com>
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/xilinx_devices.h>
#include <asm/io.h>

#include "xilinx_lcd.h"

static u32 phys_addr;
static u32 remap_size;
static void __iomem * base_addr;
static int lcd_position;	/* LCD cursor position, * '-1' means "invalid" */
static DECLARE_MUTEX(sem);

static inline void lcd_command(unsigned char c)
{
	/*
	 * "Clear Display" and "Return Home" commands
	 * take more time to complete than others.
	 */
	int i = ((c == HD44780_CLR_DISPLAY || c == HD44780_RET_HOME) ? 30 : 1);

	writew(c | HD44780_INSTR, base_addr);
	do lcd_wait(); while(--i);
}

static inline void lcd_writechar(unsigned char c)
{
	writew(c | HD44780_WR_DATA, base_addr);
	lcd_wait();
}

static void lcd_reset(void)
{
	lcd_command(HD44780_8BIT_2LINE);
	lcd_command(HD44780_MODE_INC);
	lcd_command(HD44780_CLR_DISPLAY);
	lcd_command(HD44780_DISPLAY_OFF);
	lcd_command(HD44780_CURSOR_LINE);
}

static void lcd_seek(int pos)
{
	int i, j;

	if (lcd_position == -1)
		lcd_reset();

	if (pos == lcd_position) {
		return;
	}

	lcd_command(HD44780_RET_HOME);
	for (i = 0; i < pos; i++) {
		lcd_command(HD44780_RIGHT);
		if (i == XILINX_LCD_MAX_POS / 2) {
			for (j = 0; j < (40 - XILINX_LCD_MAX_POS / 2); j++)
				lcd_command(HD44780_RIGHT);
		}
	}
	lcd_position = pos;
}

static loff_t lcd_llseek(struct file *file, loff_t pos, int orig)
{
	if (orig == 1)		/* SEEK_CUR */
		pos = file->f_pos + pos;
	else if (orig != 0)	/* SEEK_SET */
		return -EINVAL;

	if (pos < XILINX_LCD_MAX_POS && pos >= 0)
		return pos;
	else
		return -EINVAL;
}

static ssize_t lcd_write(struct file *file, const char *s,
			 size_t size, loff_t * ppos)
{
	int i;

	/* Position the cursor */
	if (*ppos >= XILINX_LCD_MAX_POS || *ppos < 0)
		return -EFAULT;

	if(down_interruptible(&sem))
		return -ERESTARTSYS;

	lcd_seek(*ppos);

	/* Print characters */
	for (i = 0; i < size; i++) {
		lcd_writechar(*s++);
		lcd_position++;
		if (lcd_position == XILINX_LCD_MAX_POS / 2) {
			/* skip unvisible positions by writing spaces to them */
			int j;

			for (j = 0; j < (40 - XILINX_LCD_MAX_POS / 2); j++)
				lcd_writechar(' ');
		}
		if (lcd_position == XILINX_LCD_MAX_POS) {
			/* do not go beyond the visible area */
			++i;
			break;
		}
	}
	*ppos = lcd_position;

	up(&sem);
	return i;
}

static int lcd_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int lcd_release(struct inode *inode, struct file *file)
{
	if(down_interruptible(&sem))
		return -ERESTARTSYS;

	lcd_seek(XILINX_LCD_MAX_POS);

	up(&sem);
	return 0;
}

/* Misc device structures */
static struct file_operations lcd_fops = {
	.llseek	 = 	lcd_llseek,
	.write   =	lcd_write,
	.open    =	lcd_open,
	.release =	lcd_release,
};

static struct miscdevice lcd_dev = {
	XILINX_LCD_MINOR,
	XILINX_LCD_NAME,
	&lcd_fops
};

/******************************
 * The platform device driver *
 ******************************/

static int lcd_probe(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct resource *regs_res;
	loff_t start = 0;
	int err;

	if (!dev)
		return -EINVAL;

	regs_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs_res) {
		printk(KERN_ERR "%s: IO resource(s) not found\n",
		       XILINX_LCD_NAME);
		return -EFAULT;
	}

	phys_addr = regs_res->start;
	remap_size = regs_res->end - regs_res->start + 1;
	if (!request_mem_region((u32)base_addr, remap_size, XILINX_LCD_NAME)) {
		printk(KERN_ERR
		       "%s: Couldn't lock memory region at 0x%08lX\n",
		       XILINX_LCD_NAME, regs_res->start);
		return -EBUSY;
	}

	base_addr  = ioremap(phys_addr, remap_size);
	if(!base_addr) {
		release_mem_region(phys_addr, remap_size);
		return -EFAULT;
	}
	printk(KERN_INFO "%s: at 0x%08X (%d bytes) mapped to %p\n",
	       XILINX_LCD_NAME, phys_addr, remap_size, base_addr);

	lcd_reset();
	lcd_write(NULL, LCD_GREETING, sizeof (LCD_GREETING) - 1, &start);

	/* Register the lcd driver */
	err = misc_register(&lcd_dev);
	if (err != 0) {
		iounmap(base_addr);
		release_mem_region(phys_addr, remap_size);
		printk(KERN_ERR "%s: Could not register driver (error %d).\n",
		       lcd_dev.name, err);
	}

	return err;
}

static int lcd_remove(struct device *dev)
{
	misc_deregister(&lcd_dev);
	lcd_command(HD44780_DISPLAY_OFF);
	iounmap(base_addr);
	release_mem_region(phys_addr, remap_size);
	return 0;	/* success */
}

static struct device_driver lcd_driver = {
	.name		= XILINX_LCD_NAME,
	.bus		= &platform_bus_type,
	.probe		= lcd_probe,
	.remove		= lcd_remove
};

static int __init lcd_init(void)
{
	return driver_register(&lcd_driver);
}

static void __exit lcd_cleanup(void)
{
	driver_unregister(&lcd_driver);
}

module_init(lcd_init);
module_exit(lcd_cleanup);

MODULE_AUTHOR("MontaVista Software, Inc. <source@mvista.com>");
MODULE_DESCRIPTION("Xilinx character LCD driver");
MODULE_LICENSE("GPL");
