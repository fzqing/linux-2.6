/*
 * arch/mips/tx4939/rbtx4939/spi_txx9.c
 *
 * Author: source@mvista.com
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

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <asm/tx4939/tx4939.h>
#include <asm/tx4939/spi.h>

static int (*txx9_spi_cs_func) (int chipid, int on);
static raw_spinlock_t txx9_spi_lock = RAW_SPIN_LOCK_UNLOCKED;

extern unsigned int txx9_gbus_clock;

#define SPI_FIFO_SIZE	4

void __init
txx9_spi_init(unsigned long base, int (*cs_func) (int chipid, int on))
{
	txx9_spi_cs_func = cs_func;
	/* enter config mode */
	reg_wr32(&tx4939_spiptr->mcr, TXx9_SPMCR_CONFIG | TXx9_SPMCR_BCLR);
}

static DECLARE_WAIT_QUEUE_HEAD(txx9_spi_wait);
static irqreturn_t txx9_spi_interrupt(int irq, void *dev_id,
				      struct pt_regs *regs)
{
	u32 v;

	/* disable rx intr */
	v = reg_rd32(&tx4939_spiptr->cr0);
	reg_wr32(&tx4939_spiptr->cr0, v & ~TXx9_SPCR0_RBSIE);
	wake_up(&txx9_spi_wait);

	return IRQ_HANDLED;
}
static struct irqaction txx9_spi_action = {
	.handler = txx9_spi_interrupt,
	.name = "spi",
};

void __init txx9_spi_irqinit(int irc_irq)
{
	setup_irq(irc_irq, &txx9_spi_action);
}

int
txx9_spi_io(int chipid, struct spi_dev_desc *desc,
	    unsigned char **inbufs, unsigned int *incounts,
	    unsigned char **outbufs, unsigned int *outcounts, int cansleep)
{
	unsigned int incount, outcount;
	unsigned char *inp, *outp;
	int ret;
	unsigned long flags;
	u32 v;

	spin_lock_irqsave(&txx9_spi_lock, flags);
	if (((v =
	      reg_rd32(&tx4939_spiptr->mcr)) & TXx9_SPMCR_OPMODE) ==
	    TXx9_SPMCR_ACTIVE) {
		spin_unlock_irqrestore(&txx9_spi_lock, flags);
		return -EBUSY;
	}
	/* enter config mode */
	reg_wr32(&tx4939_spiptr->mcr, TXx9_SPMCR_CONFIG | TXx9_SPMCR_BCLR);
	reg_wr32(&tx4939_spiptr->cr0,
		  (desc->byteorder ? TXx9_SPCR0_SBOS : 0) |
		  (desc->polarity ? TXx9_SPCR0_SPOL : 0) |
		  (desc->phase ? TXx9_SPCR0_SPHA : 0) | 0x08);
	reg_wr32(&tx4939_spiptr->cr1,
		  (((TXX9_IMCLK + desc->baud) / (2 * desc->baud) - 1) << 8) |
		  0x08) /* 8 bit only */ ;
	/* enter active mode */
	reg_wr32(&tx4939_spiptr->mcr, TXx9_SPMCR_ACTIVE);
	spin_unlock_irqrestore(&txx9_spi_lock, flags);

	/* CS ON */
	if ((ret = txx9_spi_cs_func(chipid, 1)) < 0) {
		spin_unlock_irqrestore(&txx9_spi_lock, flags);
		return ret;
	}
	udelay(desc->tcss);

	/* do scatter IO */
	inp = inbufs ? *inbufs : NULL;
	outp = outbufs ? *outbufs : NULL;
	incount = 0;
	outcount = 0;
	while (1) {
		unsigned char data;
		unsigned int count;
		int i;
		if (!incount) {
			incount = incounts ? *incounts++ : 0;
			inp = (incount && inbufs) ? *inbufs++ : NULL;
		}
		if (!outcount) {
			outcount = outcounts ? *outcounts++ : 0;
			outp = (outcount && outbufs) ? *outbufs++ : NULL;
		}
		if (!inp && !outp)
			break;
		count = SPI_FIFO_SIZE;
		if (incount)
			count = min(count, incount);
		if (outcount)
			count = min(count, outcount);

		/* now tx must be idle... */
		while (!(reg_rd32(&tx4939_spiptr->sr) & TXx9_SPSR_SIDLE)) ;

		reg_wr32(&tx4939_spiptr->cr0,
			 (reg_rd32(&tx4939_spiptr->cr0) &
			  ~TXx9_SPCR0_RXIFL_MASK) | ((count - 1) << 12));
		if (cansleep) {
			/* enable rx intr */
			reg_wr32(&tx4939_spiptr->cr0,
				 reg_rd32(&tx4939_spiptr->cr0) | TXx9_SPCR0_RBSIE);
		}
		/* send */
		for (i = 0; i < count; i++)
			reg_wr32(&tx4939_spiptr->dr, inp ? *inp++ : 0);
		/* wait all rx data */
		if (cansleep) {
			wait_event(txx9_spi_wait,
				   reg_rd32(&tx4939_spiptr->
					     sr) & TXx9_SPSR_SRRDY);
		} else {
			while (!
			       (reg_rd32(&tx4939_spiptr->sr) &
				TXx9_SPSR_RBSI)) ;
		}
		/* receive */
		for (i = 0; i < count; i++) {
			data = reg_rd32(&tx4939_spiptr->dr);
			if (outp)
				*outp++ = data;
		}
		if (incount)
			incount -= count;
		if (outcount)
			outcount -= count;
	}

	/* CS OFF */
	udelay(desc->tcsh);
	txx9_spi_cs_func(chipid, 0);
	udelay(desc->tcsr);

	spin_lock_irqsave(&txx9_spi_lock, flags);
	/* enter config mode */
	reg_wr32(&tx4939_spiptr->mcr, TXx9_SPMCR_CONFIG | TXx9_SPMCR_BCLR);
	spin_unlock_irqrestore(&txx9_spi_lock, flags);

	return 0;
}
