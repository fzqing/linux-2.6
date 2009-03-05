/*
 * i2c-algo-8xx.c i2x driver algorithms for MPC8XX CPM
 * Copyright (c) 1999 Dan Malek (dmalek@jlc.net).
 *
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * moved into proper i2c interface; separated out platform specific 
 * parts into i2c-rpx.c
 * Brad Parker (brad@heeltoe.com)
 */

// XXX todo
// timeout sleep?

/* $Id: i2c-algo-8xx.c,v 1.15 2004/11/20 08:02:24 khali Exp $ */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-8xx.h>
#include <asm/io.h>
#include <asm/mpc8xx.h>
#include <asm/commproc.h>

#define CPM_MAX_READ	513

/* #define I2C_CHIP_ERRATA *//* Try uncomment this if you have an older CPU(earlier than rev D4) */

static wait_queue_head_t iic_wait;
static ushort r_tbase, r_rbase;

int cpm_debug = 0;

static void cpm_iic_interrupt(void *dev_id, struct pt_regs *regs)
{
	i2c8xx_t *i2c = (i2c8xx_t *) dev_id;
	if (cpm_debug > 1)
		printk("cpm_iic_interrupt(dev_id=%p)\n", dev_id);
#if 0
	/* Chip errata, clear enable. This is not needed on rev D4 CPUs */
	/* This should probably be removed and replaced by I2C_CHIP_ERRATA stuff */
	/* Someone with a buggy CPU needs to confirm that */
	out_8(&i2c->i2c_i2mod, in_8(&i2c->i2c_i2mod) | ~1);
#endif
	/* Clear interrupt.
	 */
	out_8(&i2c->i2c_i2cer, 0xff);

	/* Get 'me going again.
	 */
	wake_up_interruptible(&iic_wait);
}

static void cpm_iic_init(struct i2c_algo_8xx_data *cpm)
{
	iic_t *iip = cpm->iip;
	i2c8xx_t *i2c = cpm->i2c;
	unsigned char brg;
	bd_t *bd = (bd_t *) __res;

	if (cpm_debug)
		printk(KERN_DEBUG "cpm_iic_init()\n");

	/* Initialize the parameter ram.
	 * We need to make sure many things are initialized to zero,
	 * especially in the case of a microcode patch.
	 */
	iip->iic_rstate = 0;
	iip->iic_rdp = 0;
	iip->iic_rbptr = 0;
	iip->iic_rbc = 0;
	iip->iic_rxtmp = 0;
	iip->iic_tstate = 0;
	iip->iic_tdp = 0;
	iip->iic_tbptr = 0;
	iip->iic_tbc = 0;
	iip->iic_txtmp = 0;

	/* Set up the IIC parameters in the parameter ram.
	 */
	iip->iic_tbase = r_tbase = cpm->dp_addr;
	iip->iic_rbase = r_rbase = cpm->dp_addr + sizeof(cbd_t) * 2;

	iip->iic_tfcr = SMC_EB;
	iip->iic_rfcr = SMC_EB;

	/* Set maximum receive size.
	 */
	iip->iic_mrblr = CPM_MAX_READ;

	/* Initialize Tx/Rx parameters.
	 */
	if (cpm->reloc == 0) {
		cpm8xx_t *cp = cpm->cp;
		u16 v = mk_cr_cmd(CPM_CR_CH_I2C, CPM_CR_INIT_TRX) | CPM_CR_FLG;

		out_be16(&cp->cp_cpcr, v);
		while (in_be16(&cp->cp_cpcr) & CPM_CR_FLG) ;
	} else {
		iip->iic_rbptr = iip->iic_rbase;
		iip->iic_tbptr = iip->iic_tbase;
		iip->iic_rstate = 0;
		iip->iic_tstate = 0;
	}

	/* Select an arbitrary address.  Just make sure it is unique.
	 */
	out_8(&i2c->i2c_i2add, 0xfe);

	/* Make clock run at 60 KHz.
	 */
	brg = (unsigned char)(bd->bi_intfreq / (32 * 2 * 60000) - 3);
	out_8(&i2c->i2c_i2brg, brg);

	out_8(&i2c->i2c_i2mod, 0x00);
	out_8(&i2c->i2c_i2com, 0x01);	/* Master mode */

	/* Disable interrupts.
	 */
	out_8(&i2c->i2c_i2cmr, 0);
	out_8(&i2c->i2c_i2cer, 0xff);

	init_waitqueue_head(&iic_wait);

	/* Install interrupt handler.
	 */
	if (cpm_debug) {
		printk("%s[%d] Install ISR for IRQ %d\n",
		       __func__, __LINE__, CPMVEC_I2C);
	}
	cpm_install_handler(CPMVEC_I2C, cpm_iic_interrupt, (void *)i2c);
}

static int cpm_iic_shutdown(struct i2c_algo_8xx_data *cpm)
{
	i2c8xx_t *i2c = cpm->i2c;

	/* Shut down IIC.
	 */
	out_8(&i2c->i2c_i2mod, in_8(&i2c->i2c_i2mod) | ~1);
	out_8(&i2c->i2c_i2cmr, 0);
	out_8(&i2c->i2c_i2cer, 0xff);

	return (0);
}

static void cpm_reset_iic_params(iic_t * iip)
{
	iip->iic_tbase = r_tbase;
	iip->iic_rbase = r_rbase;

	iip->iic_tfcr = SMC_EB;
	iip->iic_rfcr = SMC_EB;

	iip->iic_mrblr = CPM_MAX_READ;

	iip->iic_rstate = 0;
	iip->iic_rdp = 0;
	iip->iic_rbptr = iip->iic_rbase;
	iip->iic_rbc = 0;
	iip->iic_rxtmp = 0;
	iip->iic_tstate = 0;
	iip->iic_tdp = 0;
	iip->iic_tbptr = iip->iic_tbase;
	iip->iic_tbc = 0;
	iip->iic_txtmp = 0;
}

#define BD_SC_NAK		((ushort)0x0004)	/* NAK - did not respond */
#define BD_SC_OV		((ushort)0x0002)	/* OV - receive overrun */
#define CPM_CR_CLOSE_RXBD	((ushort)0x0007)

static void force_close(struct i2c_algo_8xx_data *cpm)
{
	i2c8xx_t *i2c = cpm->i2c;
	if (cpm->reloc == 0) {	/* micro code disabled */
		cpm8xx_t *cp = cpm->cp;
		u16 v = mk_cr_cmd(CPM_CR_CH_I2C, CPM_CR_CLOSE_RXBD) | CPM_CR_FLG;

		if (cpm_debug)
			printk("force_close()\n");

		out_be16(&cp->cp_cpcr, v);
		while (in_be16(&cp->cp_cpcr) & CPM_CR_FLG) ;
	}
	out_8(&i2c->i2c_i2cmr, 0x00);	/* Disable all interrupts */
	out_8(&i2c->i2c_i2cer, 0xff);
}

/* Read from IIC...
 * abyte = address byte, with r/w flag already set
 */
static int
cpm_iic_read(struct i2c_algo_8xx_data *cpm, u_char abyte, char *buf, int count)
{
	iic_t *iip = cpm->iip;
	i2c8xx_t *i2c = cpm->i2c;
	cpm8xx_t *cp = cpm->cp;
	cbd_t *tbdf, *rbdf;
	u_char *tb;
	unsigned long flags, tmo;

	if (count >= CPM_MAX_READ)
		return -EINVAL;

	/* check for and use a microcode relocation patch */
	if (cpm->reloc) {
		cpm_reset_iic_params(iip);
	}

	tbdf = (cbd_t *) & cp->cp_dpmem[iip->iic_tbase];
	rbdf = (cbd_t *) & cp->cp_dpmem[iip->iic_rbase];

	/* To read, we need an empty buffer of the proper length.
	 * All that is used is the first byte for address, the remainder
	 * is just used for timing (and doesn't really have to exist).
	 */
	tb = cpm->temp;
	tb = (u_char *) (((uint) tb + 15) & ~15);
	tb[0] = abyte;		/* Device address byte w/rw flag */

	flush_dcache_range((unsigned long)tb, (unsigned long)(tb + 1));

	if (cpm_debug)
		printk("cpm_iic_read(abyte=0x%x)\n", abyte);

	tbdf->cbd_bufaddr = __pa(tb);
	tbdf->cbd_datlen = count + 1;
	tbdf->cbd_sc = BD_SC_READY | BD_SC_LAST | BD_SC_WRAP | BD_IIC_START;

	iip->iic_mrblr = count + 1;	/* prevent excessive read, +1
					   is needed otherwise will the
					   RXB interrupt come too early */

	/* flush will invalidate too. */
	flush_dcache_range((unsigned long)buf, (unsigned long)(buf + count));

	rbdf->cbd_datlen = 0;
	rbdf->cbd_bufaddr = __pa(buf);
	rbdf->cbd_sc = BD_SC_EMPTY | BD_SC_WRAP | BD_SC_INTRPT;
	if (count > 16) {
		/* Chip bug, set enable here */
		local_irq_save(flags);
		out_8(&i2c->i2c_i2cmr, 0x13);	/* Enable some interupts */
		out_8(&i2c->i2c_i2cer, 0xff);
		out_8(&i2c->i2c_i2mod, in_8(&i2c->i2c_i2mod) | 1);	/* Enable */
		out_8(&i2c->i2c_i2com, in_8(&i2c->i2c_i2com) | 0x80);	/* Begin transmission */

		/* Wait for IIC transfer */
		tmo = interruptible_sleep_on_timeout(&iic_wait, 1 * HZ);
		local_irq_restore(flags);
	} else {		/* busy wait for small transfers, its faster */
		out_8(&i2c->i2c_i2cmr, 0x00);	/* Disable I2C interupts */
		out_8(&i2c->i2c_i2cer, 0xff);
		out_8(&i2c->i2c_i2mod, in_8(&i2c->i2c_i2mod) | 1);	/* Enable */
		out_8(&i2c->i2c_i2com, in_8(&i2c->i2c_i2com) | 0x80);	/* Begin transmission */
		
		tmo = jiffies + 1 * HZ;
		while (!(in_8(&i2c->i2c_i2cer) & 0x11 || time_after(jiffies, tmo))) ;/* Busy wait, with a timeout */
	}

	if (signal_pending(current) || !tmo) {
		force_close(cpm);
		if (cpm_debug)
			printk("IIC read: timeout!\n");
		return -EIO;
	}
#ifdef I2C_CHIP_ERRATA
	/* Chip errata, clear enable. This is not needed on rev D4 CPUs.
	   Disabling I2C too early may cause too short stop condition */
	udelay(4);
	out_8(&i2c->i2c_i2mod, in_8(&i2c->i2c_i2mod) | ~1);
#endif
	if (cpm_debug) {
		printk("tx sc %04x, rx sc %04x\n", tbdf->cbd_sc, rbdf->cbd_sc);
	}

	if (tbdf->cbd_sc & BD_SC_READY) {
		printk("IIC read; complete but tbuf ready\n");
		force_close(cpm);
		printk("tx sc %04x, rx sc %04x\n", tbdf->cbd_sc, rbdf->cbd_sc);
	}

	if (tbdf->cbd_sc & BD_SC_NAK) {
		if (cpm_debug)
			printk("IIC read; no ack\n");
		return -EREMOTEIO;
	}

	if (rbdf->cbd_sc & BD_SC_EMPTY) {
		/* force_close(cpm); */
		if (cpm_debug) {
			printk("IIC read; complete but rbuf empty\n");
			printk("tx sc %04x, rx sc %04x\n",
			       tbdf->cbd_sc, rbdf->cbd_sc);
		}
		return -EREMOTEIO;
	}

	if (rbdf->cbd_sc & BD_SC_OV) {
		if (cpm_debug)
			printk("IIC read; Overrun\n");
		return -EREMOTEIO;;
	}

	if (cpm_debug)
		printk("read %d bytes\n", rbdf->cbd_datlen);

	if (rbdf->cbd_datlen < count) {
		if (cpm_debug)
			printk("IIC read; short, wanted %d got %d\n",
			       count, rbdf->cbd_datlen);
		return 0;
	}

	return count;
}

/* Write to IIC...
 * addr = address byte, with r/w flag already set
 */
static int
cpm_iic_write(struct i2c_algo_8xx_data *cpm, u_char abyte, char *buf, int count)
{
	iic_t *iip = cpm->iip;
	i2c8xx_t *i2c = cpm->i2c;
	cpm8xx_t *cp = cpm->cp;
	cbd_t *tbdf;
	u_char *tb;
	unsigned long flags, tmo;

	/* check for and use a microcode relocation patch */
	if (cpm->reloc) {
		cpm_reset_iic_params(iip);
	}
	tb = cpm->temp;
	tb = (u_char *) (((uint) tb + 15) & ~15);
	*tb = abyte;		/* Device address byte w/rw flag */

	flush_dcache_range((unsigned long)tb, (unsigned long)(tb + 1));
	flush_dcache_range((unsigned long)buf, (unsigned long)(buf + count));

	if (cpm_debug)
		printk("cpm_iic_write(abyte=0x%x)\n", abyte);

	/* set up 2 descriptors */
	tbdf = (cbd_t *) & cp->cp_dpmem[iip->iic_tbase];

	tbdf[0].cbd_bufaddr = __pa(tb);
	tbdf[0].cbd_datlen = 1;
	tbdf[0].cbd_sc = BD_SC_READY | BD_IIC_START;

	tbdf[1].cbd_bufaddr = __pa(buf);
	tbdf[1].cbd_datlen = count;
	tbdf[1].cbd_sc = BD_SC_READY | BD_SC_INTRPT | BD_SC_LAST | BD_SC_WRAP;

	if (count > 16) {
		/* Chip bug, set enable here */
		local_irq_save(flags);
		out_8(&i2c->i2c_i2cmr, 0x13);	/* Enable some interupts */
		out_8(&i2c->i2c_i2cer, 0xff);
		out_8(&i2c->i2c_i2mod, in_8(&i2c->i2c_i2mod) | 1);	/* Enable */
		out_8(&i2c->i2c_i2com, in_8(&i2c->i2c_i2com) | 0x80);	/* Begin transmission */

		/* Wait for IIC transfer */
		tmo = interruptible_sleep_on_timeout(&iic_wait, 1 * HZ);
		local_irq_restore(flags);
	} else {		/* busy wait for small transfers, its faster */
		out_8(&i2c->i2c_i2cmr, 0x00);	/* Disable I2C interupts */
		out_8(&i2c->i2c_i2cer, 0xff);
		out_8(&i2c->i2c_i2mod, in_8(&i2c->i2c_i2mod) | 1);	/* Enable */
		out_8(&i2c->i2c_i2com, in_8(&i2c->i2c_i2com) | 0x80);	/* Begin transmission */
		tmo = jiffies + 1 * HZ;
		while (!(in_8(&i2c->i2c_i2cer) & 0x12 || time_after(jiffies, tmo))) ;/* Busy wait, with a timeout */
	}

	if (signal_pending(current) || !tmo) {
		force_close(cpm);
		if (cpm_debug && !tmo)
			printk("IIC write: timeout!\n");
		return -EIO;
	}
#if I2C_CHIP_ERRATA
	/* Chip errata, clear enable. This is not needed on rev D4 CPUs.
	   Disabling I2C too early may cause too short stop condition */
	udelay(4);
	out_8(&i2c->i2c_i2mod, in_8(&i2c->i2c_i2mod) | ~1);
#endif
	if (cpm_debug) {
		printk("tx0 sc %04x, tx1 sc %04x\n",
		       tbdf[0].cbd_sc, tbdf[1].cbd_sc);
	}

	if (tbdf->cbd_sc & BD_SC_NAK) {
		if (cpm_debug)
			printk("IIC write; no ack\n");
		return 0;
	}

	if (tbdf->cbd_sc & BD_SC_READY) {
		if (cpm_debug)
			printk("IIC write; complete but tbuf ready\n");
		return 0;
	}

	return count;
}

/* See if an IIC address exists..
 * addr = 7 bit address, unshifted
 */
static int cpm_iic_tryaddress(struct i2c_algo_8xx_data *cpm, int addr)
{
	iic_t *iip = cpm->iip;
	i2c8xx_t *i2c = cpm->i2c;
	cpm8xx_t *cp = cpm->cp;
	cbd_t *tbdf, *rbdf;
	u_char *tb;
	unsigned long flags, len, tmo;

	if (cpm_debug > 1)
		printk("cpm_iic_tryaddress(cpm=%p,addr=%d)\n", cpm, addr);

	/* check for and use a microcode relocation patch */
	if (cpm->reloc) {
		cpm_reset_iic_params(iip);
	}

	if (cpm_debug && addr == 0) {
		printk("iip %p, dp_addr 0x%x\n", cpm->iip, cpm->dp_addr);
		printk("iic_tbase %d, r_tbase %d\n", iip->iic_tbase, r_tbase);
	}

	tbdf = (cbd_t *) & cp->cp_dpmem[iip->iic_tbase];
	rbdf = (cbd_t *) & cp->cp_dpmem[iip->iic_rbase];

	tb = cpm->temp;
	tb = (u_char *) (((uint) tb + 15) & ~15);

	/* do a simple read */
	tb[0] = (addr << 1) | 1;	/* device address (+ read) */
	len = 2;

	flush_dcache_range((unsigned long)tb, (unsigned long)(tb + 2));

	tbdf->cbd_bufaddr = __pa(tb);
	tbdf->cbd_datlen = len;
	tbdf->cbd_sc = BD_SC_READY | BD_SC_LAST | BD_SC_WRAP | BD_IIC_START;

	rbdf->cbd_datlen = 0;
	rbdf->cbd_bufaddr = __pa(tb + 2);
	rbdf->cbd_sc = BD_SC_EMPTY | BD_SC_WRAP | BD_SC_INTRPT;

	local_irq_save(flags);
	out_8(&i2c->i2c_i2cmr, 0x13);	/* Enable some interupts */
	out_8(&i2c->i2c_i2cer, 0xff);
	out_8(&i2c->i2c_i2mod, in_8(&i2c->i2c_i2mod) | 1);	/* Enable */
	out_8(&i2c->i2c_i2com, in_8(&i2c->i2c_i2com) | 0x80);	/* Begin transmission */

	if (cpm_debug > 1)
		printk("about to sleep\n");

	/* wait for IIC transfer */
	tmo = interruptible_sleep_on_timeout(&iic_wait, 1 * HZ);
	local_irq_restore(flags);

#ifdef I2C_CHIP_ERRATA
	/* Chip errata, clear enable. This is not needed on rev D4 CPUs.
	   Disabling I2C too early may cause too short stop condition */
	udelay(4);
	out_8(&i2c->i2c_i2mod, in_8(&i2c->i2c_i2mod) | ~1);
#endif

	if (signal_pending(current) || !tmo) {
		force_close(cpm);
		if (cpm_debug && !tmo)
			printk("IIC tryaddress: timeout!\n");
		return -EIO;
	}

	if (cpm_debug > 1)
		printk("back from sleep\n");

	if (tbdf->cbd_sc & BD_SC_NAK) {
		if (cpm_debug > 1)
			printk("IIC try; no ack\n");
		return 0;
	}

	if (tbdf->cbd_sc & BD_SC_READY) {
		printk("IIC try; complete but tbuf ready\n");
	}

	return 1;
}

static int cpm_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	struct i2c_algo_8xx_data *cpm = adap->algo_data;
	struct i2c_msg *pmsg;
	int i, ret;
	u_char addr;

	for (i = 0; i < num; i++) {
		pmsg = &msgs[i];

		if (cpm_debug)
			printk("i2c-algo-8xx.o: "
			       "#%d addr=0x%x flags=0x%x len=%d\n buf=%lx\n",
			       i, pmsg->addr, pmsg->flags, pmsg->len,
			       (unsigned long)pmsg->buf);

		addr = pmsg->addr << 1;
		if (pmsg->flags & I2C_M_RD)
			addr |= 1;
		if (pmsg->flags & I2C_M_REV_DIR_ADDR)
			addr ^= 1;

		if (!(pmsg->flags & I2C_M_NOSTART)) {
		}
		if (pmsg->flags & I2C_M_RD) {
			/* read bytes into buffer */
			ret = cpm_iic_read(cpm, addr, pmsg->buf, pmsg->len);
			if (cpm_debug)
				printk("i2c-algo-8xx.o: read %d bytes\n", ret);
			if (ret < pmsg->len) {
				return (ret < 0) ? ret : -EREMOTEIO;
			}
		} else {
			/* write bytes from buffer */
			ret = cpm_iic_write(cpm, addr, pmsg->buf, pmsg->len);
			if (cpm_debug)
				printk("i2c-algo-8xx.o: wrote %d\n", ret);
			if (ret < pmsg->len) {
				return (ret < 0) ? ret : -EREMOTEIO;
			}
		}
	}
	return (num);
}

static u32 cpm_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_SMBUS_EMUL | I2C_FUNC_10BIT_ADDR |
	    I2C_FUNC_PROTOCOL_MANGLING;
}

/* -----exported algorithm data: -------------------------------------	*/

static struct i2c_algorithm cpm_algo = {
	.name = "MPC8xx CPM algorithm",
	.id = I2C_ALGO_MPC8XX,
	.master_xfer = cpm_xfer,
	.functionality = cpm_func,
};

/* 
 * registering functions to load algorithms at runtime 
 */
int i2c_8xx_add_bus(struct i2c_adapter *adap)
{
	struct i2c_algo_8xx_data *cpm = adap->algo_data;

	if (cpm_debug)
		printk("i2c-algo-8xx.o: hw routines for %s registered.\n",
		       adap->name);

	/* register new adapter to i2c module... */

	adap->id |= cpm_algo.id;
	adap->algo = &cpm_algo;

	i2c_add_adapter(adap);
	cpm_iic_init(cpm);

	return 0;
}

int i2c_8xx_del_bus(struct i2c_adapter *adap)
{
	struct i2c_algo_8xx_data *cpm = adap->algo_data;

	cpm_iic_shutdown(cpm);

	return i2c_del_adapter(adap);
}

EXPORT_SYMBOL(i2c_8xx_add_bus);
EXPORT_SYMBOL(i2c_8xx_del_bus);

MODULE_AUTHOR("Brad Parker <brad@heeltoe.com>");
MODULE_DESCRIPTION("I2C-Bus MPC8XX algorithm");
MODULE_LICENSE("GPL");
