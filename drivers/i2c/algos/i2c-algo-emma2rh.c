/*
   -------------------------------------------------------------------------
   i2c-algo-emma2rh.c i2c driver algorithms for NEC EMMA2RH I2C adapters
   -------------------------------------------------------------------------

    Copyright (C) NEC Electronics Corporation 2005-2006

    Changes made to support the I2C peripheral on the NEC EMMA2RH

   -------------------------------------------------------------------------
    This file was highly leveraged from i2c-algo-pcf.c, which was created
    by Simon G. Vogl and Hans Berglund:

     Copyright (C) 1995-1997 Simon G. Vogl
                   1998-2000 Hans Berglund

    With some changes from Kyösti Mälkki <kmalkki@cc.hut.fi> and
    Frodo Looijaard <frodol@dds.nl> ,and also from Martin Bailey
    <mbailey@littlefeet-inc.com>

    Partially rewriten by Oleg I. Vdovikin <vdovikin@jscc.ru> to handle multiple
    messages, proper stop/repstart signaling during receive,
    added detect code

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
   -------------------------------------------------------------------------
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>

#include <linux/i2c.h>
#include <linux/i2c-algo-emma2rh.h>

#include <asm/emma2rh/emma2rh.h>

#ifdef DEBUG
#define i2c_emma2rh_debug(level,op) do { if (i2c_debug>=(level)) { op; } } while (0)
static int i2c_debug;
module_param(i2c_debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(i2c_debug,
		 "debug level - 0 off; 1 normal; 2,3 more verbose; 9 i2c-protocol");
#else
#define i2c_emma2rh_debug(level,op) /* nothing */
#endif

#define DEB(x) i2c_emma2rh_debug(1, x)
#define DEB2(x) i2c_emma2rh_debug(2, x)
#define DEB3(x) i2c_emma2rh_debug(3, x)
#define DEBPROTO(x) i2c_emma2rh_debug(9,x)

#define EMMA2RH_I2C_RETRIES 3
#define EMMA2RH_I2C_TIMEOUT 100

/* --- setting states on the bus with the right timing: ---------------	*/
#define set_emma(adap, ctl, val) adap->setemma(adap->data, ctl, val)
#define get_emma(adap, ctl) adap->getemma(adap->data, ctl)
#define get_own(adap) adap->getown(adap->data)
#define get_clock(adap) adap->getclock(adap->data)

/* --- other auxiliary functions --------------------------------------	*/

static void i2c_start(struct i2c_algo_emma_data *adap)
{
	DEBPROTO(printk("S "));
	set_emma(adap, I2C_EMMA_CNT, I2C_EMMA_START);
}

static void i2c_repstart(struct i2c_algo_emma_data *adap)
{
	DEBPROTO(printk(" Sr "));
	set_emma(adap, I2C_EMMA_CNT, I2C_EMMA_REPSTART);
}

static void i2c_stop(struct i2c_algo_emma_data *adap)
{
	DEBPROTO(printk("P\n"));
	set_emma(adap, I2C_EMMA_CNT, I2C_EMMA_STOP);
}

static inline void emma_sleep(unsigned long timeout)
{
	schedule_timeout(timeout * HZ);
}

static int wait_for_pin(struct i2c_algo_emma_data *adap, int *status)
{
	adap->waitforpin(adap->data);
	*status = get_emma(adap, I2C_EMMA_STA);
	return 0;
}

static int emma_init(struct i2c_algo_emma_data *adap)
{
	unsigned char temp;

	/* serial interface off */
	set_emma(adap, I2C_EMMA_CNT, 0);

#if CONFIG_SLAVE_MODE
	/* load own address in SVA, effective address is (own & 0xfe) */
	set_emma(adap, I2C_EMMA_SVA, get_own(adap));
	udelay(20);
	/* check it's realy written */
	if ((temp = get_emma(adap, I2C_EMMA_SVA)) != get_own(adap)) {
		DEB2(printk
		     (KERN_ERR
		      "%s: EMMA detection failed -- can't set I2C_EMMA_SVA (0x%02x).\n",
		      __FUNCTION__,temp));
		return -ENXIO;
	}
#endif
	/* load clock register CS */
	set_emma(adap, I2C_EMMA_CSEL, get_clock(adap));
	udelay(20);		/* wait awhile */
	/* check it's realy written, the only 4 lowest bits does matter */
	if (((temp = get_emma(adap, I2C_EMMA_CSEL)) & 0x8f) != get_clock(adap)) {
		DEB2(printk
		     (KERN_ERR
		      "%s: EMMA detection failed -- can't set I2C_EMMA_CSEL (0x%02x).\n",
		      __FUNCTION__, temp));
		return -ENXIO;
	}

	/* initialize interrupt mask */
	set_emma(adap, I2C_EMMA_INT, 0);
	set_emma(adap, I2C_EMMA_INTM, INTE0);

	/* Enable serial interface */
	set_emma(adap, I2C_EMMA_CNT, IICE);

	/* generate a STOP condition, first of all */
	i2c_stop(adap);

	return 0;
}

static int emma_exit(struct i2c_algo_emma_data *adap)
{
	set_emma(adap, I2C_EMMA_INTM, 0);
}

/* --- Utility functions ---------------------------------------------- */

static inline int try_address(struct i2c_algo_emma_data *adap,
			      unsigned char addr, int retries)
{
	int i, status, ret = -1;

	for (i = 0; i < retries; i++) {
		i2c_start(adap);
		set_emma(adap, I2C_EMMA_SHR, addr);
		if (wait_for_pin(adap, &status) >= 0)
			if (status & ACKD) {
				i2c_stop(adap);
				ret = 1;
				break;	/* success! */
			}
		i2c_stop(adap);
		udelay(adap->udelay);
	}
	DEB2(if (i)
	     printk(KERN_DEBUG "%s: needed %d retries for %d\n", __FUNCTION__, i, addr)) ;
	return ret;
}

static int emma_sendbytes(struct i2c_adapter *i2c_adap, const char *buf,
			  int count, int last)
{
	struct i2c_algo_emma_data *adap = i2c_adap->algo_data;
	int wrcount, status, timeout;

	set_emma(adap, I2C_EMMA_CNT, IICE | WTIM);
	for (wrcount = 0; wrcount < count; wrcount++) {
		set_emma(adap, I2C_EMMA_INT, 0);
		set_emma(adap, I2C_EMMA_SHR, buf[wrcount] & SR);
		timeout = wait_for_pin(adap, &status);
		if (timeout) {
			i2c_stop(adap);
			printk(KERN_ERR "%s i2c_write: "
			       "error - timeout.\n", i2c_adap->name);
			return -EREMOTEIO;	/* got a better one ?? */
		}
		if (!(status & ACKD)) {
			i2c_stop(adap);
			printk(KERN_ERR "%s i2c_write: "
			       "error - no ack.\n", i2c_adap->name);
			return -EREMOTEIO;	/* got a better one ?? */
		}
	}

	if (last)
		i2c_stop(adap);
	else
		i2c_repstart(adap);

	return (wrcount);
}

static int emma_readbytes(struct i2c_adapter *i2c_adap, char *buf,
			  int count, int last)
{
	struct i2c_algo_emma_data *adap = i2c_adap->algo_data;
	int rdcount, status, timeout;

	for (rdcount = 0; rdcount < count; rdcount++) {

		/* we will suffer from unexpected interrupts if we
		 * use 8-clock-wait.
		 */

		set_emma(adap, I2C_EMMA_INT, 0);
		if (rdcount == count - 1)
			/* last byte */
			set_emma(adap, I2C_EMMA_CNT, IICE | WREL | WTIM);
		else
			set_emma(adap, I2C_EMMA_CNT, IICE | WREL | WTIM | ACKE);

		timeout = wait_for_pin(adap, &status);
		if (timeout) {
			i2c_stop(adap);
			printk(KERN_ERR
			       "i2c-algo-emma.o: emma_readbytes timed out.\n");
			return (-1);
		}
		if (!(status & ACKD) && (rdcount != count - 1)) {
			i2c_stop(adap);
			printk(KERN_ERR
			       "i2c-algo-emma.o: i2c_read: i2c_inb, No ack.\n");
			return (-1);
		}
		buf[rdcount] = get_emma(adap, I2C_EMMA_SHR);
	}

	if (last)
		i2c_stop(adap);
	else
		i2c_repstart(adap);

	return (rdcount);
}

static inline int emma_doAddress(struct i2c_algo_emma_data *adap,
				 struct i2c_msg *msg, int retries)
{
	unsigned short flags = msg->flags;
	unsigned int addr;
	int ret;

	set_emma(adap, I2C_EMMA_INT, 0);

	if (flags & I2C_M_TEN) {
		/* a ten bit address */
		addr = 0xf0 | ((msg->addr >> 7) & 0x03);
		DEB2(printk(KERN_DEBUG "addr0: %d\n", addr));

		/* try extended address code... */
		ret = try_address(adap, addr, retries);
		if (ret != 1) {
			printk(KERN_ERR "died at extended address code.\n");
			return -EREMOTEIO;
		}
		/* the remaining 8 bit address */
		/* ...TBD */
		printk(KERN_ERR"10 bit addresses are not supported in this driver.\n");
		return -EREMOTEIO;
	} else {		/* normal 7bit address  */
		addr = (msg->addr << 1);
		if (flags & I2C_M_RD)
			addr |= 1;
		if (flags & I2C_M_REV_DIR_ADDR)
			addr ^= 1;
		set_emma(adap, I2C_EMMA_SHR, addr);
	}
	return 0;
}

static int emma_xfer(struct i2c_adapter *i2c_adap,
		     struct i2c_msg msgs[], int num)
{
	struct i2c_algo_emma_data *adap = i2c_adap->algo_data;
	struct i2c_msg *pmsg;
	int i;
	int ret = 0, timeout, status;

	for (i = 0; ret >= 0 && i < num; i++) {
		pmsg = &msgs[i];

		DEB2(printk
		     (KERN_DEBUG
		      "Doing %s %d bytes to 0x%02x - %d of %d messages\n",
		      pmsg->flags & I2C_M_RD ? "read" : "write", pmsg->len,
		      pmsg->addr, i + 1, num));

		/* Send START */
		if (i == 0)
			i2c_start(adap);

		ret = emma_doAddress(adap, pmsg, i2c_adap->retries);

		/* Wait for PIN (pending interrupt NOT) */
		timeout = wait_for_pin(adap, &status);
		if (timeout) {
			i2c_stop(adap);
			DEB2(printk(KERN_ERR "Timeout waiting "
				    "for PIN(1) in emma_xfer\n"));
			return (-EREMOTEIO);
		}

		/* Check LRB (last rcvd bit - slave ack) */
		if (!(status & ACKD)) {
			i2c_stop(adap);
			DEB2(printk
			     (KERN_ERR
			      "No LRB(1) in emma_xfer\n"));
			return (-EREMOTEIO);
		}

		DEB3(printk
		     (KERN_DEBUG
		      "i2c-algo-emma.o: Msg %d, addr=0x%x, flags=0x%x, len=%d\n",
		      i, msgs[i].addr, msgs[i].flags, msgs[i].len));

		if (pmsg->flags & I2C_M_RD)
			/* read bytes into buffer */
			ret = emma_readbytes(i2c_adap, pmsg->buf, pmsg->len,
					     (i + 1 == num));
		 else 	/* Write */
			ret = emma_sendbytes(i2c_adap, pmsg->buf, pmsg->len,
					     (i + 1 == num));

		DEB2( printk( KERN_DEBUG "transferred %d bytes of %d -- %s\n",
					ret, pmsg->len, ret != pmsg->len ? "FAIL" : "succeeded"));
	}

	return (i);
}

static int algo_control(struct i2c_adapter *adapter,
			unsigned int cmd, unsigned long arg)
{
	return 0;
}

static u32 emma_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_SMBUS_EMUL | I2C_FUNC_PROTOCOL_MANGLING;
}

/* --- exported algorithm data ---------------------------------------- */

static struct i2c_algorithm emma_algo = {
	.name = "NEC EMMA2RH I2C algorithm",
	.id = I2C_ALGO_EXP,
	.master_xfer = emma_xfer,
	.smbus_xfer = NULL,
	.slave_send = NULL,	/* slave_send */
	.slave_recv = NULL,	/* slave_recv */
	.algo_control = algo_control,	/* ioctl */
	.functionality = emma_func,	/* functionality */
};

/*
 * registering functions to load algorithms at routine
 */
int i2c_emma_add_bus(struct i2c_adapter *adap)
{
	int i;
	struct i2c_algo_emma_data *emma_adap = adap->algo_data;

	DEB2(printk
	     (KERN_DEBUG "hw routines for %s registered.\n",
	      adap->name));

	/* register new adapter to i2c module... */

	adap->id |= emma_algo.id;
	adap->algo = &emma_algo;

	adap->timeout = EMMA2RH_I2C_TIMEOUT;	/* default values, should */
	adap->retries = EMMA2RH_I2C_RETRIES;	/* be replaced by defines */

	if ((i = emma_init(emma_adap)))
		return i;

	i2c_add_adapter(adap);

	return 0;
}

int i2c_emma_del_bus(struct i2c_adapter *adap)
{
	int res;
	struct i2c_algo_emma_data *emma_adap = adap->algo_data;

	emma_exit(emma_adap);

	if ((res = i2c_del_adapter(adap)) < 0)
		return res;
	DEB2(printk(KERN_DEBUG "adapter unregistered: %s\n",
		    adap->name));

	return 0;
}

int __init i2c_algo_emma_init(void)
{
	return 0;
}

void i2c_algo_emma_exit(void)
{
	return;
}

EXPORT_SYMBOL_GPL(i2c_emma_add_bus);
EXPORT_SYMBOL_GPL(i2c_emma_del_bus);

MODULE_AUTHOR("NEC Electronics Corporation <www.necel.com>");
MODULE_DESCRIPTION("NEC EMMA2RH I2C-Bus algorithm");
MODULE_LICENSE("GPL");

module_init(i2c_algo_emma_init);
module_exit(i2c_algo_emma_exit);
