/*
 * linux/arch/mips/tx4927/common/smsc_fdc37m81x.c
 *
 * Interface for smsc fdc48m81x Super IO chip
 *
 * Author: MontaVista Software, Inc. source@mvista.com
 *
 * 2001-2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/config.h>
#include <asm/io.h>
#include <asm/tx4927/smsc_fdc37m81x.h>

static unsigned long g_smsc_fdc37m81x_base = 0;

static inline unsigned char
smsc_fdc37m81x_rd(unsigned char index)
{
	outb(index, g_smsc_fdc37m81x_base + SMSC_FDC37M81X_CONFIG_INDEX);
	return inb(g_smsc_fdc37m81x_base + SMSC_FDC37M81X_CONFIG_DATA);
}
                                                                                                                              
static inline void
smsc_dc37m81x_wr(unsigned char index, unsigned char data)
{
	outb(index, g_smsc_fdc37m81x_base + SMSC_FDC37M81X_CONFIG_INDEX);
	outb(data, g_smsc_fdc37m81x_base + SMSC_FDC37M81X_CONFIG_DATA);
	return;
}

void
smsc_fdc37m81x_config_beg(void)
{
	if (g_smsc_fdc37m81x_base) {
		outb(SMSC_FDC37M81X_CONFIG_ENTER,
		     g_smsc_fdc37m81x_base + SMSC_FDC37M81X_CONFIG_INDEX);
	}
	return;
}

void
smsc_fdc37m81x_config_end(void)
{
	if (g_smsc_fdc37m81x_base) {
		outb(SMSC_FDC37M81X_CONFIG_EXIT,
		     g_smsc_fdc37m81x_base + SMSC_FDC37M81X_CONFIG_INDEX);
	}
	return;
}

u8
smsc_fdc37m81x_config_get(u8 reg)
{
	u8 val = 0;
	if (g_smsc_fdc37m81x_base) {
		val = smsc_fdc37m81x_rd(reg);
	}
	return (val);
}

void
smsc_fdc37m81x_config_set(u8 reg, u8 val)
{
	if (g_smsc_fdc37m81x_base) {
		smsc_dc37m81x_wr(reg, val);
	}
	return;
}

unsigned long __init
smsc_fdc37m81x_init(unsigned long port)
{
	u8 chip_id;

	if (g_smsc_fdc37m81x_base) {
		printk("smsc_fdc37m81x_init() stepping on old base=0x%08lx\n",
		       g_smsc_fdc37m81x_base);
	}

	g_smsc_fdc37m81x_base = port;

	smsc_fdc37m81x_config_beg();

	chip_id = smsc_fdc37m81x_rd(SMSC_FDC37M81X_DID);
	if (chip_id == SMSC_FDC37M81X_CHIP_ID) {
		smsc_fdc37m81x_config_end();
	} else {
		printk("smsc_fdc37m81x_init() unknow chip id 0x%02x\n",
		       chip_id);
		g_smsc_fdc37m81x_base = 0;
	}

	return (g_smsc_fdc37m81x_base);
}

#ifdef DEBUG

void
smsc_fdc37m81x_config_dump_one(char *key, u8 dev, u8 reg)
{
	printk("%s: dev=0x%02x reg=0x%02x val=0x%02x\n", key, dev, reg,
	       smsc_fdc37m81x_rd(reg));
	return;
}

void
smsc_fdc37m81x_config_dump(void)
{
	u8 orig;
	char *fname = "smsc_fdc37m81x_config_dump()";

	smsc_fdc37m81x_config_beg();

	orig = smsc_fdc37m81x_rd(SMSC_FDC37M81X_DNUM);

	printk("%s: common\n", fname);
	smsc_fdc37m81x_config_dump_one(fname, SMSC_FDC37M81X_NONE,
				       SMSC_FDC37M81X_DNUM);
	smsc_fdc37m81x_config_dump_one(fname, SMSC_FDC37M81X_NONE,
				       SMSC_FDC37M81X_DID);
	smsc_fdc37m81x_config_dump_one(fname, SMSC_FDC37M81X_NONE,
				       SMSC_FDC37M81X_DREV);
	smsc_fdc37m81x_config_dump_one(fname, SMSC_FDC37M81X_NONE,
				       SMSC_FDC37M81X_PCNT);
	smsc_fdc37m81x_config_dump_one(fname, SMSC_FDC37M81X_NONE,
				       SMSC_FDC37M81X_PMGT);

	printk("%s: keyboard\n", fname);
	smsc_dc37m81x_wr(SMSC_FDC37M81X_DNUM, SMSC_FDC37M81X_KBD);
	smsc_fdc37m81x_config_dump_one(fname, SMSC_FDC37M81X_KBD,
				       SMSC_FDC37M81X_ACTIVE);
	smsc_fdc37m81x_config_dump_one(fname, SMSC_FDC37M81X_KBD,
				       SMSC_FDC37M81X_INT);
	smsc_fdc37m81x_config_dump_one(fname, SMSC_FDC37M81X_KBD,
				       SMSC_FDC37M81X_INT2);
	smsc_fdc37m81x_config_dump_one(fname, SMSC_FDC37M81X_KBD,
				       SMSC_FDC37M81X_LDCR_F0);

	smsc_dc37m81x_wr(SMSC_FDC37M81X_DNUM, orig);

	smsc_fdc37m81x_config_end();

	return;
}

#endif
