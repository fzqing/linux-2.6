/*
 *  i2c_adap_pxa.c
 *
 *  I2C adapter for the PXA I2C bus access.
 *
 *  Copyright (C) 2002 Intrinsyc Software Inc.
 *  Copyright (C) 2004 Deep Blue Solutions Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  History:
 *    Apr 2002: Initial version [CS]
 *    Jun 2002: Properly seperated algo/adap [FB]
 *    Jan 2003: Fixed several bugs concerning interrupt handling [Kai-Uwe Bloem]
 *    Jan 2003: added limited signal handling [Kai-Uwe Bloem]
 *    Sep 2004: Major rework to ensure efficient bus handling [RMK]
 *    Dec 2004: Added support for PXA27x and slave device probing [Liam Girdwood]
 */

#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/i2c-algo-pxa.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/arch/i2c.h>
#include <asm/arch/pxa-regs.h>

#if 0
#define DBGLVL KERN_INFO
#else
#define DBGLVL KERN_DEBUG
#endif

#ifdef DEBUG
static unsigned int i2c_debug = DEBUG;
#else
#define i2c_debug	0
#endif

#undef VERBOSE

#if defined(DEBUG) || defined(VERBOSE)
static void i2c_pxa_show_state(int lno, const char *fname)
{
	printk(DBGLVL "state:%s:%d: ISR=%08x, ICR=%08x, IBMR=%02x\n", fname, lno, ISR, ICR, IBMR);
}
#endif

#ifdef DEBUG
#define show_state() i2c_pxa_show_state(__LINE__, __FUNCTION__)
#else
#define show_state() do { } while(0)
#endif

#if defined(VERBOSE)
#define show_verbose() i2c_pxa_show_state(__LINE__, __FUNCTION__)
#else
#define show_verbose() do { } while(0)
#endif

#define eedbg(lvl, x...) do { if ((lvl) < 1) { printk(DBGLVL "" x); } } while(0)

#define do_bit(val,bit,set,unset) do { printk("%s", ((val) & (bit)) ? set : unset); } while(0)

static void i2c_pxa_start(void);
static int i2c_pxa_controller_suspend(struct device *dev, u32 state, u32 level);
static int i2c_pxa_controller_resume(struct device *dev, u32 level);
static void i2c_pxa_stop(void);
static void i2c_pxa_start(void);
static int i2c_pxa_wait_bus_not_busy(void);
static void i2c_pxa_reset(void);

static struct device_driver i2c_pxa_driver;

static void decode_ISR(unsigned int val)
{
	printk(DBGLVL "ISR %08x: ", val);
	do_bit(val, ISR_RWM,    "RX ", "TX ");
	do_bit(val, ISR_ACKNAK, "NAK ", "ACK ");
	do_bit(val, ISR_UB,     "Bsy ", "Rdy ");
	do_bit(val, ISR_IBB,    "BusBsy ", "BusRdy ");
	do_bit(val, ISR_SSD,    "SlaveStop ", "");
	do_bit(val, ISR_ALD,    "ALD ", "");
	do_bit(val, ISR_ITE,    "TxEmpty ", "");
	do_bit(val, ISR_IRF,    "RxFull ", "");
	do_bit(val, ISR_GCAD,   "GenCall ", "");
	do_bit(val, ISR_SAD,    "SlaveAddr ", "");
	do_bit(val, ISR_BED,    "BusErr ", "");
	printk(DBGLVL "\n");
}

static void decode_ICR(unsigned int val)
{
	printk(DBGLVL "ICR %08x: ", val);

	do_bit(val, ICR_START,  "START ", "");
	do_bit(val, ICR_STOP,   "STOP ", "");
	do_bit(val, ICR_ACKNAK, "ACKNAK ", "");
	do_bit(val, ICR_TB,     "TB ", "");
	do_bit(val, ICR_MA,     "MA ", "");
	do_bit(val, ICR_SCLE,   "SCLE ", "scle ");
	do_bit(val, ICR_IUE,    "IUE ", "iue ");
	do_bit(val, ICR_GCD,    "GCD ", "");
	do_bit(val, ICR_ITEIE,  "ITEIE ", "");
	do_bit(val, ICR_IRFIE,  "IRFIE ", "");
	do_bit(val, ICR_BEIE,   "BEIE ", "");
	do_bit(val, ICR_SSDIE,  "SSDIE ", "");
	do_bit(val, ICR_ALDIE,  "ALDIE ", "");
	do_bit(val, ICR_SADIE,  "SADIE ", "");
	do_bit(val, ICR_UR,     "UR", "ur");
	printk("\n");
}

/*
 * New stuff...
 */
struct pxa_i2c {
	spinlock_t		lock;
	wait_queue_head_t	wait;
	struct i2c_msg		*msg;
	unsigned int		msg_num;
	unsigned int		msg_idx;
	unsigned int		msg_ptr;

	struct i2c_adapter	adap;

	int			irqlogidx;
	u32			isrlog[32];
	u32			icrlog[32];
};

static int i2c_pxa_controller_suspend(struct device *dev, u32 state, u32 level)
{
#ifdef CONFIG_PM
	i2c_pxa_wait_bus_not_busy();
	i2c_pxa_stop();
#endif
	return 0;
}

static int i2c_pxa_controller_resume(struct device *dev, u32 level)
{
#ifdef CONFIG_PXA27x
	pxa_gpio_mode(GPIO117_I2CSCL_MD);
	pxa_gpio_mode(GPIO118_I2CSDA_MD);
	udelay(100);
#endif
#ifdef CONFIG_PXA3xx
	pxa_set_cken(CKEN_I2C, 1);
#else
	pxa_set_cken(CKEN14_I2C, 1);
#endif
	i2c_pxa_reset();

#ifdef CONFIG_PM
	i2c_pxa_start();
#endif /* CONFIG_PM */
	return 0;
}

static inline int i2c_pxa_is_slavemode(void)
{
	return !(ICR & ICR_SCLE);
}


static void i2c_pxa_stop(void)
{
	unsigned long icr = ICR;

	show_state();

	icr |= ICR_STOP;
	icr &= ~(ICR_START);
	ICR = icr;

	show_state();
}

static void i2c_pxa_start(void)
{
	unsigned long icr = ICR;
	icr |= ICR_START;
	icr &= ~(ICR_STOP | ICR_ALDIE | ICR_ACKNAK);
	ICR = icr;
}

static void i2c_pxa_midbyte(void)
{
	unsigned long icr = ICR;
	icr &= ~(ICR_START | ICR_STOP);
	ICR = icr;
}

static void i2c_pxa_transfer(int lastbyte, int receive, int midbyte)
{
	if (i2c_pxa_is_slavemode()) {
		printk(DBGLVL "i2c_pxa_transfer: called in slave mode\n");
		return;
	}

	if (lastbyte)
	{
		if (receive==I2C_RECEIVE)
			ICR |= ICR_ACKNAK;
		i2c_pxa_stop();
	}
	else if (midbyte)
	{
		i2c_pxa_midbyte();
	}

	ICR |= ICR_TB;
	show_state();
}

static void i2c_pxa_abort(void)
{
	unsigned long timeout = jiffies + HZ/4;

#ifdef PXA_ABORT_MA
	while ((long)(timeout - jiffies) > 0 && (ICR & ICR_TB)) {
		msleep(1);
	}

	ICR |= ICR_MA;
	udelay(100);
#else
	while ((long)(timeout - jiffies) > 0 && (IBMR & 0x1) == 0) {
		i2c_pxa_transfer(1, I2C_RECEIVE, 1);
		msleep(1);
	}
#endif
	ICR &= ~(ICR_MA | ICR_START | ICR_STOP);
}

static int i2c_pxa_wait_bus_not_busy(void)
{
	int timeout = DEF_TIMEOUT;

	while (timeout-- && (ISR & (ISR_IBB | ISR_UB))) {
		if ((ISR & ISR_SAD) != 0) {
			timeout += 4;
		}
		msleep(2);
		show_state();
	}

	if (timeout <= 0) {
		show_verbose();
		show_state();
	}

	return (timeout<=0);
}

static int i2c_pxa_wait_master(void)
{
	unsigned long timeout = jiffies + HZ*4;

	while ((long)(timeout - jiffies) > 0) {
		if (i2c_debug > 1)
			printk(DBGLVL "i2c_pxa_wait_master: %ld: ISR=%08x, ICR=%08x, IBMR=%02x\n",
			       (long)jiffies, ISR, ICR, IBMR);

		if (ISR & ISR_SAD) {
			if (i2c_debug > 0)
				printk(DBGLVL "i2c_pxa_wait_master: Slave detected\n");
			return 0;
		}

		/* wait for unit and bus being not busy, and we also do a
		 * quick check of the i2c lines themselves to ensure they've
		 * gone high...
		 */
		if ((ISR & (ISR_UB | ISR_IBB)) == 0 &&
		    (IBMR == 3)) {
			if (i2c_debug > 0)
				printk(DBGLVL "i2c_pxa_wait_master: done\n");
			return 1;
		}

		msleep(1);
	}

	if (i2c_debug > 0)
		printk(DBGLVL "i2c_pxa_wait_master: did not free\n");
	return 0;
}

static int i2c_pxa_set_master(void)
{
	if (i2c_debug)
		printk(DBGLVL "I2C: setting to bus master\n");

	if ((ISR & (ISR_UB | ISR_IBB)) != 0) {
		printk(DBGLVL "set_master: unit is busy\n");
		if (!i2c_pxa_wait_master()) {
			show_verbose();
			printk(DBGLVL "set_master: error: unit busy\n");
			return -EBUSY;
		}
	}

	ICR |= ICR_SCLE;
	return 0;
}

static int i2c_pxa_wait_slave(void)
{
	unsigned long timeout = jiffies + HZ*1;

	/* wait for stop */

	show_state();

	while ((long)(timeout - jiffies) > 0) {
		if (i2c_debug > 1)
			printk(DBGLVL "i2c_pxa_wait_slave: %ld: ISR=%08x, ICR=%08x, IBMR=%02x\n",
			       (long)jiffies, ISR, ICR, IBMR);

		if ((ISR & (ISR_UB|ISR_IBB)) == 0 ||
		    (ISR & ISR_SAD) != 0 ||
		    (ICR & ICR_SCLE) == 0) {
			if (i2c_debug > 1)
				printk(DBGLVL "i2c_pxa_wait_slave: done\n");
			return 1;
		}

		msleep(1);
	}

	if (i2c_debug > 0)
		printk(DBGLVL "i2c_pxa_wait_slave: did not free\n");
	return 0;
}

static int i2c_pxa_set_slave(int errcode)
{
	/* clear the hold on the bus, and take of anything else
	 * that has been configured */

	show_state();

	if (errcode == SLAVE_STARTED) {
		// just fall through

	} else if (errcode < 0) {
		udelay(100);   /* simple delay */
	} else {
		/* we need to wait for the stop condition to end */

		/* if we where in stop, then clear... */
		if (ICR & ICR_STOP) {
			udelay(100);
			ICR &= ~ICR_STOP;
		}

		if (!i2c_pxa_wait_slave()) {
			show_verbose();
			printk(KERN_ERR "i2c_pxa_set_slave: wait timedout\n");
			return -EBUSY;
		}
	}

	ICR &= ~(ICR_STOP|ICR_ACKNAK|ICR_MA);
	ICR &= ~ICR_SCLE;

	if (i2c_debug) {
		printk(DBGLVL "ICR now %08x, ISR %08x\n", ICR, ISR);
		decode_ICR(ICR);
	}

	return 0;
}

static void i2c_pxa_reset(void)
{
	unsigned int slave;

#ifdef DEBUG
	printk("Resetting I2C Controller Unit\n");
#endif

	/* abort any transfer currently under way */
	i2c_pxa_abort();

	/* reset according to 9.8 */
	ICR = ICR_UR;
	ISR = I2C_ISR_INIT;
	ICR &= ~ICR_UR;

	slave = I2C_PXA_SLAVE_ADDR;

	printk(KERN_INFO "I2C: Slave address %d\n", slave);
	ISAR = slave;

	/* set control register values */
	ICR = I2C_ICR_INIT;

#ifdef CONFIG_I2C_PXA_SLAVE
	printk(KERN_INFO "I2C: Enabling slave mode\n");
	ICR |= ICR_SADIE | ICR_ALDIE | ICR_SSDIE;

	i2c_pxa_set_slave(0);
#endif

	/* enable unit */
	ICR |= ICR_IUE;
	udelay(100);
}


/*
 * I2C EEPROM emulation.
 */
static struct i2c_eeprom_emu eeprom = {
	.size = I2C_EEPROM_EMU_SIZE,
};

struct i2c_eeprom_emu *i2c_pxa_get_eeprom(void)
{
	return &eeprom;
}

int
i2c_eeprom_emu_addwatcher(struct i2c_eeprom_emu *emu,
			  int addr, int size,
			  struct i2c_eeprom_emu_watcher *watcher)
{
	struct i2c_eeprom_emu_byte *ptr;
	unsigned long flags;

	local_irq_save(flags);

	ptr = &emu->bytes[addr];

	for (; size > 0; addr++, ptr++, size--) {
		if (ptr->watcher != NULL) {
			printk(KERN_INFO "i2c_eeprom: replacing eeprom watch on %02x\n",
			       addr);
		}

		printk(DBGLVL "i2c_eeprom: adding watcher %p to 0x%02x\n", watcher, addr);

		ptr->watcher = watcher;
	}

	local_irq_restore(flags);

	return 0;
}

static int
i2c_eeprom_emu_event(void *pw, i2c_slave_event_t event, int val)
{
	eedbg(3, "i2c_eeprom_emu_event: %d, %d\n", event, val);

	switch (event) {
	case I2C_SLAVE_EVENT_START:
		if (val == I2C_SLAVE_START_WRITE) {
			eeprom.seen_start = 1;
			eedbg(2, "i2c_eeprom: write initiated\n");
		} else {
			eeprom.seen_start = 0;
			eedbg(2, "i2c_eeprom: read initiated\n");
		}

		break;

	case I2C_SLAVE_EVENT_STOP:
		eeprom.seen_start = 0;
		eedbg(2, "i2c_eeprom: received stop\n");
		break;

	case I2C_SLAVE_EVENT_NONE:
		break;

	default:
		eedbg(0, "i2c_eeprom: unhandled event\n");
		return -EINVAL;
	}

	return 0;
}

static int i2c_eeprom_emu_read(void *pw)
{
	int ret;

	ret        =  eeprom.bytes[eeprom.ptr].val;
	eeprom.ptr = (eeprom.ptr + 1 ) % eeprom.size;

	return ret;
}

static int i2c_eeprom_emu_write(void *pw, int val)
{
	struct i2c_eeprom_emu_byte *byte;

	eedbg(1, "i2c_eeprom_emu_write: ptr=0x%02x, val=0x%02x\n",
	       eeprom.ptr, val);

	if (eeprom.seen_start != 0) {
		eedbg(2, "i2c_eeprom_emu_write: setting ptr %02x\n", val);
		eeprom.ptr = val;
		eeprom.seen_start = 0;
		return 0;
	}

	byte = &eeprom.bytes[eeprom.ptr];

	byte->val = val;
	byte->last_modified = jiffies;

	eedbg(1, "i2c_eeprom_emu_write: ptr=0x%02x, val=0x%02x\n",
	      eeprom.ptr, val);

	if (byte->watcher != NULL) {
		if (byte->watcher->write != NULL)
			(byte->watcher->write)(&eeprom, eeprom.ptr, val);
	}

	eeprom.ptr = (eeprom.ptr + 1 ) % eeprom.size;
	return 0;
}

/* temporary client for testing purposes */
struct i2c_slave_client tmp_client = {
	.event	= i2c_eeprom_emu_event,
	.read	= i2c_eeprom_emu_read,
	.write	= i2c_eeprom_emu_write
};

struct i2c_slave_client *pxa_slave = &tmp_client;

/*
 * PXA I2C adapter handler
 */
static void i2c_pxa_scream_blue_murder(struct pxa_i2c *i2c, const char *why)
{
	int i;
	printk("i2c: error: %s\n", why);
	printk("i2c: msg_num: %d msg_idx: %d msg_ptr: %d\n",
		i2c->msg_num, i2c->msg_idx, i2c->msg_ptr);
	printk("i2c: ICR: %08x ISR: %08x\ni2c: log: ", ICR, ISR);
	for (i = 0; i < i2c->irqlogidx; i++)
		printk("[%08x:%08x] ", i2c->isrlog[i], i2c->icrlog[i]);
	printk("\n");
}

static inline unsigned int i2c_pxa_addr_byte(struct i2c_msg *msg)
{
	unsigned int addr = (msg->addr & 0x7f) << 1;

	if (msg->flags & I2C_M_RD)
		addr |= 1;

	return addr;
}

static inline void i2c_pxa_start_message(struct pxa_i2c *i2c)
{
	u32 icr;

	/*
	 * Step 1: target slave address into IDBR
	 */
	IDBR = i2c_pxa_addr_byte(i2c->msg);

	/*
	 * Step 2: initiate the write.
	 */
	icr = ICR & ~(ICR_STOP | ICR_ALDIE);
	ICR = icr | ICR_START | ICR_TB;
}

/*
 * We are protected by the adapter bus semaphore.
 */
static int i2c_pxa_do_xfer(struct pxa_i2c *i2c, struct i2c_msg *msg, int num)
{
	long timeout;
	int ret;

	/*
	 * Wait for the bus to become free.
	 */
	ret = i2c_pxa_wait_bus_not_busy();
	if (ret) {
		printk(KERN_INFO "i2c_pxa: timeout waiting for bus free\n");
		ret = I2C_RETRY;
		goto out;
	}

	/*
	 * Set master mode.
	 */
	ret = i2c_pxa_set_master();
	if (ret) {
		printk(KERN_INFO "i2c_pxa_set_master: error %d\n", ret);
		ret = I2C_RETRY;
		goto out;
	}

	spin_lock_irq(&i2c->lock);

	i2c->msg = msg;
	i2c->msg_num = num;
	i2c->msg_idx = 0;
	i2c->msg_ptr = 0;
	i2c->irqlogidx = 0;

	i2c_pxa_start_message(i2c);

	spin_unlock_irq(&i2c->lock);

	/*
	 * The rest of the processing occurs in the interrupt handler.
	 */
	timeout = wait_event_timeout(i2c->wait, i2c->msg_num == 0, HZ * 5);

	/*
	 * We place the return code in i2c->msg_idx.
	 */
	ret = i2c->msg_idx;

	if (timeout == 0)
		i2c_pxa_scream_blue_murder(i2c, "timeout");

 out:
	return ret;
}

/*
 * i2c_pxa_master_complete - complete the message and wake up.
 */
static inline void i2c_pxa_master_complete(struct pxa_i2c *i2c, int ret)
{
	i2c->msg_ptr = 0;
	i2c->msg = NULL;
	i2c->msg_idx ++;
	i2c->msg_num = 0;
	if (ret)
		i2c->msg_idx = ret;
	wake_up(&i2c->wait);
}

static void i2c_pxa_irq_txempty(struct pxa_i2c *i2c, u32 isr)
{
	u32 icr = ICR & ~(ICR_START|ICR_STOP|ICR_ACKNAK|ICR_TB);

 again:
	/*
	 * If ISR_ALD is set, we lost arbitration.
	 */
	if (isr & ISR_ALD) {
		/*
		 * Do we need to do anything here?  The PXA docs
		 * are vague about what happens.
		 */
		i2c_pxa_scream_blue_murder(i2c, "ALD set");

		/*
		 * We ignore this error.  We seem to see spurious ALDs
		 * for seemingly no reason.  If handle them as I think
		 * they should, we end up causing an I2C error, which
		 * is painful for some systems.
		 */
		return; /* ignore */
	}

	if (isr & ISR_BED) {
		int ret = BUS_ERROR;

		/*
		 * I2C bus error - either the device NAK'd us, or
		 * something more serious happened.  If we were NAK'd
		 * on the initial address phase, we can retry.
		 */
		if (isr & ISR_ACKNAK) {
			if (i2c->msg_ptr == 0 && i2c->msg_idx == 0)
				ret = I2C_RETRY;
			else
				ret = XFER_NAKED;
		}
		i2c_pxa_master_complete(i2c, ret);
	} else if (isr & ISR_RWM) {
		/*
		 * Read mode.  We have just sent the address byte, and
		 * now we must initiate the transfer.
		 */
		if (i2c->msg_ptr == i2c->msg->len - 1 &&
		    i2c->msg_idx == i2c->msg_num - 1)
			icr |= ICR_STOP | ICR_ACKNAK;

		icr |= ICR_ALDIE | ICR_TB;
	} else if (i2c->msg_ptr < i2c->msg->len) {
		/*
		 * Write mode.  Write the next data byte.
		 */
		IDBR = i2c->msg->buf[i2c->msg_ptr++];

		icr |= ICR_ALDIE | ICR_TB;

		/*
		 * If this is the last byte of the last message, send
		 * a STOP.
		 */

		if (i2c->msg_ptr == i2c->msg->len &&
		    i2c->msg_idx == i2c->msg_num - 1)
			icr |= ICR_STOP;
	} else if (i2c->msg_idx < i2c->msg_num - 1) {
		/*
		 * Next segment of the message.
		 */
		i2c->msg_ptr = 0;
		i2c->msg_idx ++;
		i2c->msg++;

		/*
		 * If we aren't doing a repeated start and address,
		 * go back and try to send the next byte.  Note that
		 * we do not support switching the R/W direction here.
		 */
		if (i2c->msg->flags & I2C_M_NOSTART)
			goto again;

		/*
		 * Write the next address.
		 */
		IDBR = i2c_pxa_addr_byte(i2c->msg);

		/*
		 * And trigger a repeated start, and send the byte.
		 */
		icr &= ~ICR_ALDIE;
		icr |= ICR_START | ICR_TB;
	} else {
		if (i2c->msg->len == 0) {
			/*
			 * Device probe's have a meesage length of zero and need
			 * the bus to be reset before it can be used again.
			 */
			i2c_pxa_reset();
		}
		i2c_pxa_master_complete(i2c, 0);
	}

	i2c->icrlog[i2c->irqlogidx-1] = icr;

	ICR = icr;
	show_state();
}

static void i2c_pxa_irq_rxfull(struct pxa_i2c *i2c, u32 isr)
{
	u32 icr = ICR & ~(ICR_START|ICR_STOP|ICR_ACKNAK|ICR_TB);

	/*
	 * Read the byte.
	 */
	i2c->msg->buf[i2c->msg_ptr++] = IDBR;

	if (i2c->msg_ptr < i2c->msg->len) {
		/*
		 * If this is the last byte of the last
		 * message, send a STOP.
		 */
		if (i2c->msg_ptr == i2c->msg->len - 1)
			icr |= ICR_STOP | ICR_ACKNAK;

		icr |= ICR_ALDIE | ICR_TB;
	} else {
		i2c_pxa_master_complete(i2c, 0);
	}

	i2c->icrlog[i2c->irqlogidx-1] = icr;

	ICR = icr;
}

static irqreturn_t i2c_pxa_handler(int this_irq, void *dev_id, struct pt_regs *regs)
{
	struct pxa_i2c *i2c = dev_id;
	int status/*, wakeup = 0*/;
	unsigned int flags;

	spin_lock_irqsave(&i2c->lock, flags);

	status = (ISR);

	if (i2c_debug > 2 && 0) {
		printk(DBGLVL "i2c_pxa_handler: status=%08x, ICR=%08x, IBMR=%02x\n",
		       status, ICR, IBMR);
		decode_ISR(status);
	}

	if (i2c->irqlogidx < sizeof(i2c->isrlog)/sizeof(u32))
		i2c->isrlog[i2c->irqlogidx++] = status;

	show_state();

	/*
	 * Always clear all pending IRQs.
	 */
	ISR = status & (ISR_SSD|ISR_ALD|ISR_ITE|ISR_IRF|ISR_SAD|ISR_BED);

	if (status & ISR_SAD) {
		if (i2c_debug > 0) {
			printk(DBGLVL "ISR: SAD (Slave Addr Detect) mode is %s\n",
			       (status & ISR_RWM) ? "slave-rx" : "slave-tx");
		}

		if (pxa_slave != NULL) {
			(pxa_slave->event)(NULL,
					   I2C_SLAVE_EVENT_START,
					   (status & ISR_RWM) ? I2C_SLAVE_START_READ : I2C_SLAVE_START_WRITE);
		}

		show_verbose();

		/* slave could interrupt in the middle of us
		 * generating a start condition... if this happens, we'd
		 * better back off and stop holding the poor thing up
		 */

		ICR   &= ~(ICR_START|ICR_STOP);
		ICR   |= ICR_TB;

		{
			int timeout = 0x10000;

			while (1) {
				if ((IBMR & 2) == 2)
					break;

				timeout--;

				if (timeout <= 0) {
					printk(KERN_ERR "timeout waiting for SCL high\n");
					break;
				}
			}
		}

		ICR   &= ~ICR_SCLE;
		show_verbose();
	}

	if (status & ISR_SSD) {
		if (i2c_debug > 2)
			printk(DBGLVL "ISR: SSD (Slave Stop)\n");

		if (pxa_slave != NULL) {
			(pxa_slave->event)(NULL, I2C_SLAVE_EVENT_STOP, 0);
		}

		if (i2c_debug > 2)
			printk(DBGLVL "ISR: SSD (Slave Stop) acked\n");

		if (0)
			show_verbose();

		/*
		 * If we have a master-mode message waiting,
		 * kick it off now that the slave has completed.
		 */
		if (i2c->msg)
			i2c_pxa_master_complete(i2c, I2C_RETRY);
	}

	if (i2c_pxa_is_slavemode()) {
		if (status & ISR_BED){
			show_verbose();
		}

		if (status & ISR_ITE){
			int ret = (pxa_slave->read)(NULL);

			IDBR = ret;
			ICR |= ICR_TB;   /* allow next byte */
		}

		if (status & ISR_IRF){
			if (i2c_debug > 0)
				printk(DBGLVL "ISR_IRF (slave)\n");

			if (pxa_slave != NULL) {
				(pxa_slave->write)(NULL, IDBR);
			}

			ICR |= ICR_TB;
		}
	} else if (i2c->msg) {
		if (status & ISR_ITE)
			i2c_pxa_irq_txempty(i2c, status);
		if (status & ISR_IRF)
			i2c_pxa_irq_rxfull(i2c, status);
	} else {
		i2c_pxa_scream_blue_murder(i2c, "spurious irq");
	}

	spin_unlock_irqrestore(&i2c->lock, flags);

	return IRQ_HANDLED;
}


static int i2c_pxa_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	struct pxa_i2c *i2c = adap->algo_data;
	int ret, i;

	for (i = adap->retries; i >= 0; i--) {
		ret = i2c_pxa_do_xfer(i2c, msgs, num);
		if (ret != I2C_RETRY)
			goto out;

		if (i2c_debug)
			printk(KERN_INFO "Retrying transmission\n");
		udelay(100);
	}
	i2c_pxa_scream_blue_murder(i2c, "exhausted retries");
	ret = -EREMOTEIO;
 out:
	i2c_pxa_set_slave(ret);
	return ret;
}

static struct i2c_algorithm i2c_pxa_algorithm = {
	.name			= "PXA-I2C-Algorithm",
	.id			= I2C_ALGO_PXA,
	.master_xfer		= i2c_pxa_xfer,
};

static struct pxa_i2c i2c_pxa = {
	.lock	= SPIN_LOCK_UNLOCKED,
	.wait	= __WAIT_QUEUE_HEAD_INITIALIZER(i2c_pxa.wait),
	.adap	= {
		.name			= "pxa2xx-i2c",
		.id			= I2C_ALGO_PXA,
		.algo			= &i2c_pxa_algorithm,
		.retries		= 5,
	},
};

static int i2c_pxa_probe(struct device *dev)
{
	struct pxa_i2c *i2c = &i2c_pxa;
	int ret;

#ifdef CONFIG_PXA27x
	pxa_gpio_mode(GPIO117_I2CSCL_MD);
	pxa_gpio_mode(GPIO118_I2CSDA_MD);
	udelay(100);
#endif

#ifdef CONFIG_PXA3xx
	pxa_set_cken(CKEN_I2C, 1);
#else
	pxa_set_cken(CKEN14_I2C, 1);
#endif
	ret = request_irq(IRQ_I2C, i2c_pxa_handler, 0, "pxa2xx-i2c", i2c);
	if (ret)
		goto out;

	i2c_pxa_reset();

	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = dev;

	ret = i2c_add_adapter(&i2c->adap);
	if (ret < 0) {
		printk(KERN_INFO "I2C: Failed to add bus\n");
		free_irq(IRQ_I2C, i2c);
		goto out;
	}

	dev_set_drvdata(dev, i2c);

	printk(KERN_INFO "I2C: %s: PXA I2C adapter\n", i2c->adap.dev.bus_id);

 out:
	return ret;
}

static int i2c_pxa_remove(struct device *dev)
{
	struct pxa_i2c *i2c = dev_get_drvdata(dev);

	dev_set_drvdata(dev, NULL);

	i2c_del_adapter(&i2c->adap);
	free_irq(IRQ_I2C, i2c);
#ifdef CONFIG_PXA3xx
	pxa_set_cken(CKEN_I2C, 0);
#else
	pxa_set_cken(CKEN14_I2C, 0);
#endif

	return 0;
}

static struct device_driver i2c_pxa_driver = {
	.name		= "pxa2xx-i2c",
	.bus		= &platform_bus_type,
	.probe		= i2c_pxa_probe,
	.remove		= i2c_pxa_remove,
	.suspend        = i2c_pxa_controller_suspend,
	.resume         = i2c_pxa_controller_resume,
};

static int __init i2c_adap_pxa_init(void)
{
	return driver_register(&i2c_pxa_driver);
}

static void i2c_adap_pxa_exit(void)
{
	return driver_unregister(&i2c_pxa_driver);
}

module_init(i2c_adap_pxa_init);
module_exit(i2c_adap_pxa_exit);
