/*
 * arch/mips/tx4939/common/proc.c
 *
 * Setup proc-fs for tx4939
 *
 * Author: source@mvista.com
 *
 * 2001-2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Support for TX4939 in 2.6 - Hiroshi DOYU <Hiroshi_DOYU@montavista.co.jp>
 */

#include <linux/config.h>
#include <linux/module.h>	/* Definitions needed for kernel modules */
#include <linux/kmod.h>
#include <linux/init.h>
#include <linux/kernel.h>	/* We run in the kernel so we need this */
#include <linux/proc_fs.h>	/* The /proc definitions are in this one */

#include <asm/io.h>
#include <asm/mipsregs.h>
#include <asm/tx4939/rbtx4939.h>

static struct proc_dir_entry *rbtx4939_proc_entry;

static int rbtx4939_proc_show_ioc(char *sysbuf, char **start, off_t off,
				  int count, int *eof, void *data)
{
	int len = 0;
	len += sprintf(sysbuf + len, "rbtx4939_board_rev_ptr        :0x%02x\n",
		       reg_rd08(rbtx4939_board_rev_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_ioc_rev_ptr          :0x%02x\n",
		       reg_rd08(rbtx4939_ioc_rev_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_config1_ptr          :0x%02x\n",
		       reg_rd08(rbtx4939_config1_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_config2_ptr          :0x%02x\n",
		       reg_rd08(rbtx4939_config2_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_config3_ptr          :0x%02x\n",
		       reg_rd08(rbtx4939_config3_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_config4_ptr          :0x%02x\n",
		       reg_rd08(rbtx4939_config4_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_usersw_ptr           :0x%02x\n",
		       reg_rd08(rbtx4939_usersw_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_bootsw_ptr           :0x%02x\n",
		       reg_rd08(rbtx4939_bootsw_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_inte_ptr             :0x%02x\n",
		       reg_rd08(rbtx4939_inte_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_intp_ptr             :0x%02x\n",
		       reg_rd08(rbtx4939_intp_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_intf1_ptr            :0x%02x\n",
		       reg_rd08(rbtx4939_intf1_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_intf2_ptr            :0x%02x\n",
		       reg_rd08(rbtx4939_intf2_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_soft_int_ptr         :0x%02x\n",
		       reg_rd08(rbtx4939_soft_int_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_isa_status_ptr       :0x%02x\n",
		       reg_rd08(rbtx4939_isa_status_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_pci66_ptr            :0x%02x\n",
		       reg_rd08(rbtx4939_pci66_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_romemu_ptr           :0x%02x\n",
		       reg_rd08(rbtx4939_romemu_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_spics_ptr            :0x%02x\n",
		       reg_rd08(rbtx4939_spics_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_audio_mode_ptr       :0x%02x\n",
		       reg_rd08(rbtx4939_audio_mode_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_isa_gpio_ptr         :0x%02x\n",
		       reg_rd08(rbtx4939_isa_gpio_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_pe1_ptr              :0x%02x\n",
		       reg_rd08(rbtx4939_pe1_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_pe2_ptr              :0x%02x\n",
		       reg_rd08(rbtx4939_pe2_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_pe3_ptr              :0x%02x\n",
		       reg_rd08(rbtx4939_pe3_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_vport_mode_ptr       :0x%02x\n",
		       reg_rd08(rbtx4939_vport_mode_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_vport_reset_ptr      :0x%02x\n",
		       reg_rd08(rbtx4939_vport_reset_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_vport_sout_ptr       :0x%02x\n",
		       reg_rd08(rbtx4939_vport_sout_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_vport_sin_ptr        :0x%02x\n",
		       reg_rd08(rbtx4939_vport_sin_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_7segled_ptr          :0x%02x\n",
		       reg_rd08(rbtx4939_7segled_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_soft_reset_ptr       :0x%02x\n",
		       reg_rd08(rbtx4939_soft_reset_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_sreset_enable_ptr    :0x%02x\n",
		       reg_rd08(rbtx4939_sreset_enable_ptr));
	len += sprintf(sysbuf + len, "rbtx4939_reset_status_ptr     :0x%02x\n",
		       reg_rd08(rbtx4939_reset_status_ptr));
	*eof = 1;
	return len;
}

/**
 * rbtx4939_proc_setup - setup proc interface for RBTX4939 registers
 *
 * This function makes proc entry to show RBTX4939 registers.
 */

static int __init rbtx4939_proc_setup(void)
{
	struct proc_dir_entry *entry;

	rbtx4939_proc_entry = proc_mkdir("rbtx4939", NULL);

	if (!rbtx4939_proc_entry) {
		printk(KERN_ERR "rbtx4939 cannot be registered in /proc\n");
		return -1;
	}

	entry = create_proc_entry("ioc_register", 0, rbtx4939_proc_entry);
	if (entry) {
		entry->read_proc = rbtx4939_proc_show_ioc;
		entry->data = 0;
	}

	return 0;
}

static int __init rbtx4939_proc_init(void)
{
	return rbtx4939_proc_setup();
}

static void __exit rbtx4939_proc_exit(void)
{
	/* need to do something */
}

module_init(rbtx4939_proc_init);
module_exit(rbtx4939_proc_exit);

MODULE_DESCRIPTION("rbtx4939 proc interface");
MODULE_LICENSE("GPL");
