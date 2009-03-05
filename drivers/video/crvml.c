/*  
 * Copyright (c) Intel Corp. 2007.
 * All Rights Reserved.
 *
 * Intel funded Tungsten Graphics (http://www.tungstengraphics.com) to
 * develop this driver.
 *
 * This file is part of the Carillo Ranch video subsystem driver.
 * The Carillo Ranch video subsystem driver is free software; 
 * you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * The Carillo Ranch video subsystem driver is distributed 
 * in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this driver; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * Authors: 
 *   Thomas Hellstrom <thomas-at-tungstengraphics-dot-com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include "vermilion.h"

/* The LVDS- and panel power controls sits on the 
 * GPIO port of the ISA bridge. 
 */

#define CRVML_DEVICE_LPC    0x27B8
#define CRVML_REG_GPIOBAR   0x48
#define CRVML_REG_GPIOEN    0x4C
#define CRVML_GPIOEN_BIT    (1 << 4)
#define CRVML_PANEL_PORT    0x38
#define CRVML_LVDS_ON       0x00000001
#define CRVML_PANEL_ON      0x00000002
#define CRVML_BACKLIGHT_OFF 0x00000004

/* The PLL Clock register sits on Host bridge */
#define CRVML_DEVICE_MCH   0x5001
#define CRVML_REG_MCHBAR   0x44
#define CRVML_REG_MCHEN    0x54
#define CRVML_MCHEN_BIT    (1 << 28)
#define CRVML_MCHMAP_SIZE  4096
#define CRVML_REG_CLOCK    0xc3c
#define CRVML_CLOCK_SHIFT  8
#define CRVML_CLOCK_MASK   0x00000f00

#define MODULE_NAME "crvml"

struct cr_sys {
	struct vml_sys sys;
	struct pci_dev *mch_dev;
	struct pci_dev *lpc_dev;
	__u32 mch_bar;
	__u8 *mch_regs_base;
	__u32 gpio_bar;
	__u32 saved_panel_state;
	__u32 saved_clock;
};

static struct vml_sys *my_sys = NULL;

static const unsigned crvml_clocks[] = {
	6750,
	13500,
	27000,
	29700,
	37125,
	54000,
	59400,
	74250,
	120000
	    /*
	     * There are more clocks, but they are disabled on the CR board.
	     */
};

static const __u32 crvml_clock_bits[] = {
	0x0a,
	0x09,
	0x08,
	0x07,
	0x06,
	0x05,
	0x04,
	0x03,
	0x0b
};

static const unsigned crvml_num_clocks = sizeof(crvml_clocks) / sizeof(__u32);

static int crvml_sysinit(struct cr_sys *crsys)
{
	__u32 dev_en;
	__u8 dev_en8;

	if (!crsys)
		return -ENODEV;

	crsys->mch_dev = pci_get_device(PCI_VENDOR_ID_INTEL,
					CRVML_DEVICE_MCH, NULL);
	if (!crsys->mch_dev) {
		printk(KERN_ERR MODULE_NAME
		       ": Could not find Carillo Ranch MCH device.\n");
		return -ENODEV;
	}

	pci_read_config_dword(crsys->mch_dev, CRVML_REG_MCHEN, &dev_en);
	if (!(dev_en & CRVML_MCHEN_BIT)) {
		printk(KERN_ERR MODULE_NAME
		       ": Carillo Ranch MCH device was not enabled.\n");
		goto out_err_0;
	}

	pci_read_config_dword(crsys->mch_dev, CRVML_REG_MCHBAR,
			      &crsys->mch_bar);
	crsys->mch_regs_base =
	    ioremap_nocache(crsys->mch_bar, CRVML_MCHMAP_SIZE);
	if (!crsys->mch_regs_base) {
		printk(KERN_ERR MODULE_NAME
		       ": Carillo Ranch MCH device was not enabled.\n");
		goto out_err_0;
	}

	/*
	 * Get the gpio bar.
	 */

	crsys->lpc_dev = pci_get_device(PCI_VENDOR_ID_INTEL,
					CRVML_DEVICE_LPC, NULL);
	if (!crsys->lpc_dev) {
		printk(KERN_ERR MODULE_NAME
		       ": Could not find Carillo Ranch LPC device.\n");
		goto out_err_1;
	}

	pci_read_config_byte(crsys->lpc_dev, CRVML_REG_GPIOEN, &dev_en8);
	if (!(dev_en8 & CRVML_GPIOEN_BIT)) {
		printk(KERN_ERR MODULE_NAME
		       ": Carillo Ranch GPIO device was not enabled.\n");
		goto out_err_2;
	}

	pci_read_config_dword(crsys->lpc_dev, CRVML_REG_GPIOBAR,
			      &crsys->gpio_bar);
	crsys->gpio_bar &= ~0x3F;

	return 0;

      out_err_2:
	pci_dev_put(crsys->lpc_dev);
      out_err_1:
	iounmap(crsys->mch_regs_base);
      out_err_0:
	pci_dev_put(crsys->mch_dev);
	return -ENODEV;
}

static void crvml_sys_destroy(struct vml_sys *sys)
{
	struct cr_sys *crsys = container_of(sys, struct cr_sys, sys);

	pci_dev_put(crsys->lpc_dev);
	iounmap(crsys->mch_regs_base);
	pci_dev_put(crsys->mch_dev);

	if (sys == my_sys)
		my_sys = NULL;

	kfree(crsys);
}

static void crvml_panel_on(const struct vml_sys *sys)
{
	const struct cr_sys *crsys = container_of(sys, struct cr_sys, sys);
	__u32 addr = crsys->gpio_bar + CRVML_PANEL_PORT;
	__u32 cur = inl(addr);

	if (!(cur & CRVML_PANEL_ON)) {
		/* Make sure LVDS controller is down. */
		if (cur & 0x00000001) {
			cur &= ~CRVML_LVDS_ON;
			outl(cur, addr);
		}
		/* Power up Panel */
		schedule_timeout(HZ / 10);
		cur |= CRVML_PANEL_ON;
		outl(cur, addr);
	}

	/* Power up LVDS controller */

	if (!(cur & CRVML_LVDS_ON)) {
		schedule_timeout(HZ / 10);
		outl(cur | CRVML_LVDS_ON, addr);
	}
}

static void crvml_panel_off(const struct vml_sys *sys)
{
	const struct cr_sys *crsys = container_of(sys, struct cr_sys, sys);

	__u32 addr = crsys->gpio_bar + CRVML_PANEL_PORT;
	__u32 cur = inl(addr);

	/* Power down LVDS controller first to avoid high currents */
	if (cur & CRVML_LVDS_ON) {
		cur &= ~CRVML_LVDS_ON;
		outl(cur, addr);
	}
	if (cur & CRVML_PANEL_ON) {
		schedule_timeout(HZ / 10);
		outl(cur & ~CRVML_PANEL_ON, addr);
	}
}

static void crvml_backlight_on(const struct vml_sys *sys)
{
	const struct cr_sys *crsys = container_of(sys, struct cr_sys, sys);
	__u32 addr = crsys->gpio_bar + CRVML_PANEL_PORT;
	__u32 cur = inl(addr);

	if (cur & CRVML_BACKLIGHT_OFF) {
		cur &= ~CRVML_BACKLIGHT_OFF;
		outl(cur, addr);
	}
}

static void crvml_backlight_off(const struct vml_sys *sys)
{
	const struct cr_sys *crsys = container_of(sys, struct cr_sys, sys);
	__u32 addr = crsys->gpio_bar + CRVML_PANEL_PORT;
	__u32 cur = inl(addr);

	if (!(cur & CRVML_BACKLIGHT_OFF)) {
		cur |= CRVML_BACKLIGHT_OFF;
		outl(cur, addr);
	}
}

static int crvml_sys_restore(struct vml_sys *sys)
{
	struct cr_sys *crsys = container_of(sys, struct cr_sys, sys);
	__u32 *clock_reg = (__u32 *) (crsys->mch_regs_base + CRVML_REG_CLOCK);
	__u32 cur = crsys->saved_panel_state;

	if (cur & CRVML_BACKLIGHT_OFF) {
		crvml_backlight_off(sys);
	} else {
		crvml_backlight_on(sys);
	}

	if (cur & CRVML_PANEL_ON) {
		crvml_panel_on(sys);
	} else {
		crvml_panel_off(sys);
		if (cur & CRVML_LVDS_ON) {
			;
			/* Will not power up LVDS controller while panel is off */
		}
	}
	iowrite32(crsys->saved_clock, clock_reg);
	ioread32(clock_reg);

	return 0;
}

static int crvml_sys_save(struct vml_sys *sys)
{
	struct cr_sys *crsys = container_of(sys, struct cr_sys, sys);
	__u32 *clock_reg = (__u32 *) (crsys->mch_regs_base + CRVML_REG_CLOCK);

	crsys->saved_panel_state = inl(crsys->gpio_bar + CRVML_PANEL_PORT);
	crsys->saved_clock = ioread32(clock_reg);

	return 0;
}

static int crvml_nearest_index(const struct vml_sys *sys, int clock)
{

	int i;
	int cur_index;
	int cur_diff;
	int diff;

	cur_index = 0;
	cur_diff = clock - crvml_clocks[0];
	cur_diff = (cur_diff < 0) ? -cur_diff : cur_diff;
	for (i = 1; i < crvml_num_clocks; ++i) {
		diff = clock - crvml_clocks[i];
		diff = (diff < 0) ? -diff : diff;
		if (diff < cur_diff) {
			cur_index = i;
			cur_diff = diff;
		}
	}
	return cur_index;
}

static int crvml_nearest_clock(const struct vml_sys *sys, int clock)
{
	return crvml_clocks[crvml_nearest_index(sys, clock)];
}

static int crvml_set_clock(struct vml_sys *sys, int clock)
{
	struct cr_sys *crsys = container_of(sys, struct cr_sys, sys);
	__u32 *clock_reg = (__u32 *) (crsys->mch_regs_base + CRVML_REG_CLOCK);
	int index;
	__u32 clock_val;

	index = crvml_nearest_index(sys, clock);

	if (crvml_clocks[index] != clock)
		return -EINVAL;

	clock_val = ioread32(clock_reg) & ~CRVML_CLOCK_MASK;
	clock_val = crvml_clock_bits[index] << CRVML_CLOCK_SHIFT;
	iowrite32(clock_val, clock_reg);
	ioread32(clock_reg);

	return 0;
}

static void crvml_subsys(const struct vml_sys *sys, const char **name)
{
	*name = "Carillo Ranch";
}

static int crvml_false(const struct vml_sys *sys)
{
	return 0;
}

static int crvml_true(const struct vml_sys *sys)
{
	return 1;
}

static int __init crvml_init(void)
{
	int err = 0;
	struct vml_sys *sys;
	struct cr_sys *crsys;

	crsys = (struct cr_sys *)kmalloc(sizeof(*crsys), GFP_KERNEL);

	if (!crsys)
		return -ENOMEM;

	sys = &crsys->sys;
	err = crvml_sysinit(crsys);
	if (err) {
		kfree(crsys);
		return err;
	}

	sys->destroy = crvml_sys_destroy;
	sys->subsys = crvml_subsys;
	sys->save = crvml_sys_save;
	sys->restore = crvml_sys_restore;
	sys->prog_clock = crvml_false;
	sys->set_clock = crvml_set_clock;
	sys->has_panel = crvml_true;
	sys->panel_on = crvml_panel_on;
	sys->panel_off = crvml_panel_off;
	sys->backlight_on = crvml_backlight_on;
	sys->backlight_off = crvml_backlight_off;
	sys->nearest_clock = crvml_nearest_clock;

	err = vmlfb_register_subsys(sys);
	if (err) {
		crvml_sys_destroy(sys);
		return err;
	}
	my_sys = sys;
	return 0;
}

static void __exit crvml_exit(void)
{
	if (my_sys) {
		vmlfb_unregister_subsys(my_sys);
		crvml_sys_destroy(my_sys);
	}
}

module_init(crvml_init);
module_exit(crvml_exit);

MODULE_AUTHOR("Tungsten Graphics Inc.");
MODULE_DESCRIPTION("Carillo Ranch Video Subsystem Driver");
MODULE_LICENSE("GPL");
