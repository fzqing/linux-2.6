/*
 * arch/mips/tx4939/common/tx49wtoe.c
 *
 * Interrupt handler for TX49 Write timeout error
 *
 * (C) Copyright TOSHIBA CORPORATION SEMICONDUCTOR COMPANY 2000-2001,2005
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
#include <linux/slab.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <asm/io.h>

#include <asm/tx4939/tx4939.h>

#define IRQ_WTOERR TX4939_IRQ_TX49WTOE
#define CCFG_BEOW  TX4939_CCFG_BEOW
#define WDRST      TX4939_CCFG_WDRST
#define TOEA       reg_rd64s(&tx4939_ccfgptr->toea)
#define GBUS_WIDTH 36

/**
 * handler_tx49wtoe - Interrupt handler for TX49 write timeout error
 * @ird:
 * @dev_id:
 * @regs:
 *
 * This is a interrupt handler for TX49 write timeout error. This
 * shows message that it occurs, and show timeout error address
 * register.
 * TX49 write timeout error is related to write buffer.
 * You should notice that the timing differs between the interrupt and
 * the wrong access
 */

static irqreturn_t handler_tx49wtoe(int irq, void *dev_id, struct pt_regs *regs)
{
	u64 q;

	printk(KERN_ALERT "TX49 write timeout error\n");
	if (regs)
		printk(KERN_ALERT
		       "toea = 0x%0*Lx, epc == %08lx, ra == %08lx (Bad address in epc)\n",
		       GBUS_WIDTH / 4, (u64) TOEA, regs->cp0_epc, regs->regs[31]);
	printk(KERN_ALERT
	       "(This timeout error occurs after few instructions from a wrong access.)\n");

	q = reg_rd64s(&tx4939_ccfgptr->ccfg);
	reg_wr64s(&tx4939_ccfgptr->ccfg, (q & ~WDRST) | CCFG_BEOW);

	return IRQ_HANDLED;
}

static int __init tx49wtoe_init(void)
{
	int ret;

	printk(KERN_INFO "TX49 Write TimeOut Error interrupt module\n");

	ret = request_irq(IRQ_WTOERR, handler_tx49wtoe, SA_INTERRUPT,
			  "TX49 write timeout error", NULL);
	if (ret) {
		printk(KERN_ERR "unable to get wtoe irq%d: [%d]", IRQ_WTOERR,  ret);
		return ret;
	}
	return 0;
}

static void __exit tx49wtoe_exit(void)
{
	free_irq(IRQ_WTOERR, NULL);
}

module_init(tx49wtoe_init);
module_exit(tx49wtoe_exit);

MODULE_DESCRIPTION("tx49 write timeout error");
MODULE_LICENSE("GPL");
