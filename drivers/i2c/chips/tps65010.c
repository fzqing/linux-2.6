/*
 * Copyright (C) 2004 Texas Instruments
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#undef	DEBUG

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <linux/workqueue.h>

#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>

/*-------------------------------------------------------------------------*/

#define	DRIVER_VERSION	"20 Dec 2004"
#define	DRIVER_NAME	(tps65010_driver.name)

MODULE_DESCRIPTION("TPS65010 Power Managment Driver");
MODULE_LICENSE("GPL");

/* only two addresses possible */
#define	TPS_BASE	0x48
static unsigned short normal_i2c[] = {
	TPS_BASE,
	I2C_CLIENT_END };
static unsigned short normal_i2c_range[] = { I2C_CLIENT_END };

I2C_CLIENT_INSMOD;

static struct i2c_driver tps65010_driver;

/*-------------------------------------------------------------------------*/

/* registers, all 8 bits */

#define	TPS_CHGSTATUS		0x01
#	define	TPS_CHG_USB		(1 << 7)
#	define	TPS_CHG_AC		(1 << 6)
#	define	TPS_CHG_THERM		(1 << 5)
#	define	TPS_CHG_TERM		(1 << 4)
#	define	TPS_CHG_TAPER_TMO	(1 << 3)
#	define	TPS_CHG_CHG_TMO		(1 << 2)
#	define	TPS_CHG_PRECHG_TMO	(1 << 1)
#	define	TPS_CHG_TEMP_ERR	(1 << 0)
#define	TPS_REGSTATUS		0x02
#	define	TPS_REG_ONOFF		(1 << 7)
#	define	TPS_REG_COVER		(1 << 6)
#	define	TPS_REG_UVLO		(1 << 5)
#	define	TPS_REG_PG_LD02		(1 << 3)
#	define	TPS_REG_PG_LD01		(1 << 2)
#	define	TPS_REG_PG_MAIN		(1 << 1)
#	define	TPS_REG_PG_CORE		(1 << 0)
#define	TPS_MASK1		0x03
#define	TPS_MASK2		0x04
#define	TPS_ACKINT1		0x05
#define	TPS_ACKINT2		0x06
#define	TPS_CHGCONFIG		0x07
#	define	TPS_CHARGE_POR		(1 << 7)
#	define	TPS65013_AUA		(1 << 7)
#	define	TPS_CHARGE_RESET	(1 << 6)
#	define	TPS_CHARGE_FAST		(1 << 5)
#	define	TPS_CHARGE_CURRENT	(3 << 3)
#	define	TPS_VBUS_500MA		(1 << 2)
#	define	TPS_VBUS_CHARGING	(1 << 1)
#	define	TPS_CHARGE_ENABLE	(1 << 0)
#define	TPS_LED1_ON		0x08
#define	TPS_LED1_PER		0x09
#define	TPS_LED2_ON		0x0a
#define	TPS_LED2_PER		0x0b
#define	TPS_VDCDC1		0x0c
#	define	TPS_ENABLE_LP		(1 << 3)
#define	TPS_VDCDC2		0x0d
#define	TPS_VREGS1		0x0e
#define	TPS_MASK3		0x0f
#define	TPS_DEFGPIO		0x10

#define LED1  1
#define LED2  2
#define OFF   0
#define ON    1
#define BLINK 2
#define GPIO1 1
#define GPIO2 2
#define GPIO3 3
#define GPIO4 4
#define LOW   0
#define HIGH  1

/* operational registers */

/*-------------------------------------------------------------------------*/

#ifdef CONFIG_PREEMPT_RT
static spinlock_t sched_lock;
#endif

struct tps65010 {
	struct i2c_client	client;
	struct semaphore	lock;
	int			irq;
	struct work_struct	work;
	u16			vbus;
	unsigned long		flags;
#define	FLAG_VBUS_CHANGED	0

	/* copies of last register state */
	u8			chgstatus, regstatus, chgconf;
	u8			nmask1, nmask2;

	/* plus four GPIOs, probably used to switch power */
};

#ifdef	DEBUG

static void show_chgstatus(const char *label, u8 chgstatus)
{
	pr_debug("%s: %s %02x%s%s%s%s%s%s%s%s\n",
		DRIVER_NAME, label, chgstatus,
		(chgstatus & TPS_CHG_USB) ? " USB" : "",
		(chgstatus & TPS_CHG_AC) ? " AC" : "",
		(chgstatus & TPS_CHG_THERM) ? " therm" : "",
		(chgstatus & TPS_CHG_TERM) ? " done" : " (charging)",
		(chgstatus & TPS_CHG_TAPER_TMO) ? " taper_tmo" : "",
		(chgstatus & TPS_CHG_CHG_TMO) ? " charge_tmo" : "",
		(chgstatus & TPS_CHG_PRECHG_TMO) ? " prechg_tmo" : "",
		(chgstatus & TPS_CHG_TEMP_ERR) ? " temp_err" : "");
}

static void show_regstatus(const char *label, u8 regstatus)
{
	pr_debug("%s: %s %02x %s%s%s%s%s%s%s\n",
		DRIVER_NAME, label, regstatus,
		(regstatus & TPS_REG_ONOFF) ? "off" : "(on)",
		(regstatus & TPS_REG_COVER) ? " uncover" : "",
		(regstatus & TPS_REG_UVLO) ? " UVLO" : "",
		(regstatus & TPS_REG_PG_LD02) ? " ld01_bad" : "",
		(regstatus & TPS_REG_PG_LD01) ? " ld01_bad" : "",
		(regstatus & TPS_REG_PG_MAIN) ? " main_bad" : "",
		(regstatus & TPS_REG_PG_CORE) ? " core_bad" : "");
}

static void show_chgconfig(const char *label, u8 chgconfig)
{
	pr_debug("%s: %s 0x%02x %dms%s%s AC=%d%% USB=%dmA %sCharge\n",
		DRIVER_NAME, label, chgconfig,
		(chgconfig & TPS_CHARGE_POR) ? 69: 1000,
		(chgconfig & TPS_CHARGE_RESET) ? " reset" : "",
		(chgconfig & TPS_CHARGE_FAST) ? " fast" : "",
		({int p; switch ((chgconfig >> 3) & 3) {
		case 3:		p = 100; break;
		case 2:		p = 75; break;
		case 1:		p = 50; break;
		default:	p = 25; break;
		}; p; }),
		(chgconfig & TPS_VBUS_CHARGING)
			? ((chgconfig & TPS_VBUS_500MA) ? 500 : 100)
			: 0,
		(chgconfig & TPS_CHARGE_ENABLE) ? "" : "No");
}

#else

static inline void show_chgstatus(const char *label, u8 chgstatus) { }
static inline void show_regstatus(const char *label, u8 chgstatus) { }
static inline void show_chgconfig(const char *label, u8 chgconfig) { }

#endif

/*-------------------------------------------------------------------------*/

/* handle IRQs using keventd for now */
static void tps65010_interrupt(struct tps65010 *tps)
{
	u8 tmp = 0, mask;

	/* regstatus irqs */
	if (tps->nmask2) {
		tmp = i2c_smbus_read_byte_data(&tps->client, TPS_REGSTATUS);
		mask = tmp ^ tps->regstatus;
		tps->regstatus = tmp;
		mask &= tps->nmask2;
	} else
		mask = 0;
	if (mask) {
		show_regstatus("reg/irq", tmp);
		tps->regstatus =  tmp;
		/* may need to shut something down ... */
	}

	/* chgstatus irqs */
	if (tps->nmask1) {
		tmp = i2c_smbus_read_byte_data(&tps->client, TPS_CHGSTATUS);
		mask = tmp ^ tps->chgstatus;
		tps->chgstatus = tmp;
		mask &= tps->nmask1;
	} else
		mask = 0;
	if (mask) {
		show_chgstatus("chg/irq", tmp);
		if (tmp & (TPS_CHG_USB|TPS_CHG_AC))
			show_chgconfig("conf", tps->chgconf);
		/* charging starts or stops */
	}

	/* also potentially gpio-in rise or fall */
}

/* handle IRQs using keventd for now */
static void tps65010_work(void *_tps)
{
	struct tps65010		*tps = _tps;

	down(&tps->lock);

	tps65010_interrupt(tps);

	if (test_and_clear_bit(FLAG_VBUS_CHANGED, &tps->flags)) {
		int	status;
		u8	chgconfig, tmp;

		chgconfig = i2c_smbus_read_byte_data(&tps->client,
					TPS_CHGCONFIG);
		chgconfig &= ~(TPS_VBUS_500MA | TPS_VBUS_CHARGING);
		if (tps->vbus == 500)
			chgconfig |= TPS_VBUS_500MA | TPS_VBUS_CHARGING;
		else if (tps->vbus >= 100)
			chgconfig |= TPS_VBUS_CHARGING;

		status = i2c_smbus_write_byte_data(&tps->client,
				TPS_CHGCONFIG, chgconfig);

		/* fails unless a battery is connected! */
		tmp = i2c_smbus_read_byte_data(&tps->client, TPS_CHGCONFIG);
		pr_debug("%s: %d mA from vbus %d: 0x%02x%s, expect %02x\n",
			DRIVER_NAME, tps->vbus, status, tmp,
			(tmp != chgconfig) ? " (BAD)" : "",
			chgconfig);
	}

	up(&tps->lock);
}

static irqreturn_t tps65010_irq(int irq, void *_tps, struct pt_regs *regs)
{
	struct tps65010		*tps = _tps;

	spin_lock_rt(&sched_lock);
	(void) schedule_work(&tps->work);
	spin_unlock_rt(&sched_lock);
	return IRQ_HANDLED;
}

/*-------------------------------------------------------------------------*/

static struct tps65010 *the_tps;

static int tps65010_detach_client(struct i2c_client *client)
{
	struct tps65010		*tps;

	tps = container_of(client, struct tps65010, client);
#ifdef	CONFIG_ARCH_OMAP
	if (machine_is_omap_h2())
		omap_free_gpio(58);
#endif
	free_irq(tps->irq, tps);
	if (i2c_detach_client(client) == 0)
		kfree(tps);
	the_tps = 0;
	return 0;
}

/* no error returns, they'd just make bus scanning stop */
static int tps65010_probe(struct i2c_adapter *bus, int address, int kind)
{
	static int		tps65010_id;
	struct tps65010		*tps;
	int			status;

	if (the_tps) {
		dev_dbg(&bus->dev, "only one %s for now\n", DRIVER_NAME);
		return 0;
	}

	tps = kmalloc(sizeof *tps, GFP_KERNEL);
	if (!tps)
		return 0;

	memset(tps, 0, sizeof *tps);
	init_MUTEX(&tps->lock);
	INIT_WORK(&tps->work, tps65010_work, tps);
	tps->irq = -1;
	tps->client.addr = address;
	i2c_set_clientdata(&tps->client, tps);
	tps->client.adapter = bus;
	tps->client.id = tps65010_id++;
	tps->client.driver = &tps65010_driver;
	strlcpy(tps->client.name, DRIVER_NAME, I2C_NAME_SIZE);

	status = i2c_attach_client(&tps->client);
	if (status < 0) {
		dev_dbg(&bus->dev, "can't attach %s to device %d, err %d\n",
				DRIVER_NAME, address, status);
fail1:
		kfree(tps);
		return 0;
	}

#ifdef	CONFIG_ARCH_OMAP
	/* FIXME: Interrupt breaks USB on OSK */
	if (machine_is_omap_h2() /*|| machine_is_omap_osk()*/) {
		omap_cfg_reg(W4_GPIO58);
		tps->irq = OMAP_GPIO_IRQ(58);
		omap_request_gpio(58);
		omap_set_gpio_direction(58, 1);
		omap_set_gpio_edge_ctrl(58, OMAP_GPIO_FALLING_EDGE);

		status = request_irq(tps->irq, tps65010_irq,
			/*	SA_SAMPLE_RANDOM */ 0, DRIVER_NAME, tps);
		if (status < 0) {
			dev_dbg(&tps->client.dev, "can't get IRQ %d, err %d\n",
					tps->irq, status);
			i2c_detach_client(&tps->client);
			goto fail1;
		}
	}
#endif

	/* FIXME use IRQs to report "interesting" PM events
	 * to the current PM policy agent
	 */
	tps->nmask1 = ~0;
	tps->nmask2 = TPS_REG_ONOFF|TPS_REG_COVER|TPS_REG_UVLO;

	/* unmask the irqs */
	tps->chgconf = i2c_smbus_read_byte_data(&tps->client, TPS_CHGCONFIG);
	show_chgconfig("conf/init", tps->chgconf);

	tps->chgstatus = i2c_smbus_read_byte_data(&tps->client, TPS_CHGSTATUS);
	(void) i2c_smbus_write_byte_data(&tps->client, TPS_MASK1, ~tps->nmask1);
	show_chgstatus("chg/init", tps->chgstatus);
	show_chgstatus("mask1", tps->nmask1);

	tps->regstatus = i2c_smbus_read_byte_data(&tps->client, TPS_REGSTATUS);
	(void) i2c_smbus_write_byte_data(&tps->client, TPS_MASK2, ~tps->nmask2);
	show_regstatus("reg/init", tps->regstatus);
	show_regstatus("mask2", tps->nmask2);

	pr_debug("%s: vdcdc1 0x%02x, vdcdc1 %02x, vregs1 %02x\n", DRIVER_NAME,
		i2c_smbus_read_byte_data(&tps->client, TPS_VDCDC1),
		i2c_smbus_read_byte_data(&tps->client, TPS_VDCDC2),
		i2c_smbus_read_byte_data(&tps->client, TPS_VREGS1));
	pr_debug("%s: defgpio 0x%02x, mask3 0x%02x\n", DRIVER_NAME,
		i2c_smbus_read_byte_data(&tps->client, TPS_DEFGPIO),
		i2c_smbus_read_byte_data(&tps->client, TPS_MASK3));

	the_tps = tps;
	return 0;
}

static int tps65010_scan_bus(struct i2c_adapter *bus)
{
	if (!i2c_check_functionality(bus, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EINVAL;
	return i2c_probe(bus, &addr_data, tps65010_probe);
}

static struct i2c_driver tps65010_driver = {
	.owner		= THIS_MODULE,
	.name		= "tps65010",
	.id		= 888,		/* FIXME assign "official" value */
	.flags		= I2C_DF_NOTIFY,
	.attach_adapter	= tps65010_scan_bus,
	.detach_client	= tps65010_detach_client,
};

/*-------------------------------------------------------------------------*/

/* Draw from VBUS:
 *   0 mA -- DON'T DRAW (might supply power instead)
 * 100 mA -- usb unit load (slowest charge rate)
 * 500 mA -- usb high power (fast battery charge)
 */
int tps65010_set_vbus_draw(unsigned mA)
{
	unsigned long	flags;

	if (!the_tps)
		return -ENODEV;

	/* assumes non-SMP */
	local_irq_save_nort(flags);

	if (mA >= 500)
		mA = 500;
	else if (mA >= 100)
		mA = 100;
	else
		mA = 0;
	if (the_tps->vbus != mA && test_and_set_bit(
				FLAG_VBUS_CHANGED, &the_tps->flags)) {
		/* gadget drivers call this in_irq() */
		the_tps->vbus = mA;
		spin_lock_rt(&sched_lock);
		(void) schedule_work(&the_tps->work);
		spin_unlock_rt(&sched_lock);
	}
	local_irq_restore_nort(flags);

	return 0;
}
EXPORT_SYMBOL(tps65010_set_vbus_draw);

/*-------------------------------------------------------------------------*/
/* tps65010_set_gpio_out_value parameter:
 * gpio:  GPIO1, GPIO2, GPIO3 or GPIO4
 * value: LOW or HIGH
 */
int tps65010_set_gpio_out_value(unsigned gpio, unsigned value)
{
	int	 status;
	unsigned defgpio;

	if (!the_tps)
		return -ENODEV;
	if ((gpio < GPIO1) || (gpio > GPIO4))
		return -EINVAL;
	
	down(&the_tps->lock);

	defgpio = i2c_smbus_read_byte_data(&the_tps->client, TPS_DEFGPIO);

	/* Configure GPIO for output */
	defgpio |= 1 << (gpio + 3);
	
	/* Writing 1 forces a logic 0 on that GPIO and vice versa */
	switch (value) {
	case LOW:
		defgpio |= 1 << (gpio - 1);    /* set GPIO low by writing 1 */
		break;
	/* case HIGH: */
	default:
		defgpio &= ~(1 << (gpio - 1)); /* set GPIO high by writing 0 */
		break;
	}
	
	status = i2c_smbus_write_byte_data(&the_tps->client,
		TPS_DEFGPIO, defgpio);

	pr_debug("%s: gpio%dout = %s, defgpio 0x%02x\n", DRIVER_NAME,
		gpio, value ? "high" : "low",
		i2c_smbus_read_byte_data(&the_tps->client, TPS_DEFGPIO));
	
	up(&the_tps->lock);
	return status;
}
EXPORT_SYMBOL(tps65010_set_gpio_out_value);

/*-------------------------------------------------------------------------*/
/* tps65010_set_led parameter:
 * led:  LED1 or LED2
 * mode: ON, OFF or BLINK
 */
int tps65010_set_led(unsigned led, unsigned mode)
{
	int	 status;
	unsigned led_on, led_per, offs;

	if (!the_tps)
		return -ENODEV;

	if(led == LED1)
		offs = 0;
	else {
		offs = 2;
		led = LED2;
	}

	down(&the_tps->lock);

	dev_dbg (&the_tps->client.dev, "led%i_on   0x%02x\n", led,
		i2c_smbus_read_byte_data(&the_tps->client, TPS_LED1_ON + offs));

	dev_dbg (&the_tps->client.dev, "led%i_per  0x%02x\n", led,
		i2c_smbus_read_byte_data(&the_tps->client, TPS_LED1_PER + offs));
	
	switch (mode) {
	case OFF:
		led_on  = 1 << 7;
		led_per = 0 << 7;
		break;
	case ON:
		led_on  = 1 << 7;
		led_per = 1 << 7;
		break;
	case BLINK:
		led_on  = 0x30 | (0 << 7);
		led_per = 0x08 | (1 << 7);
		break;
	default:
		printk(KERN_ERR "%s: Wrong mode parameter for tps65010_set_led()\n", 
		       DRIVER_NAME);
		up(&the_tps->lock);
		return -EINVAL;
	}

	status = i2c_smbus_write_byte_data(&the_tps->client,
			TPS_LED1_ON + offs, led_on);

	if (status != 0) {
		printk(KERN_ERR "%s: Failed to write led%i_on register\n", 
		       DRIVER_NAME, led);
		up(&the_tps->lock);
		return status;
	} 

	dev_dbg (&the_tps->client.dev, "led%i_on   0x%02x\n", led,
		i2c_smbus_read_byte_data(&the_tps->client, TPS_LED1_ON + offs));

	status = i2c_smbus_write_byte_data(&the_tps->client,
			TPS_LED1_PER + offs, led_per);

	if (status != 0) {
		printk(KERN_ERR "%s: Failed to write led%i_per register\n", 
		       DRIVER_NAME, led);
		up(&the_tps->lock);
		return status;
	}

	dev_dbg (&the_tps->client.dev, "led%i_per  0x%02x\n", led,
		i2c_smbus_read_byte_data(&the_tps->client, TPS_LED1_PER + offs));

	up(&the_tps->lock);

	return status;
}
EXPORT_SYMBOL(tps65010_set_led);

/*-------------------------------------------------------------------------*/
/* tps65010_set_low_pwr parameter:
 * mode: ON or OFF
 */
int tps65010_set_low_pwr(unsigned mode)
{
	int	 status;
	unsigned vdcdc1;

	if (!the_tps)
		return -ENODEV;

	down(&the_tps->lock);

	pr_debug("%s: %s low_pwr, vdcdc1 0x%02x\n", DRIVER_NAME,
		mode ? "enable" : "disable",
		i2c_smbus_read_byte_data(&the_tps->client, TPS_VDCDC1));

	vdcdc1 = i2c_smbus_read_byte_data(&the_tps->client, TPS_VDCDC1);
	
	switch (mode) {
	case OFF:
		vdcdc1 &= ~TPS_ENABLE_LP; /* disable ENABLE_LP bit */
		break;
	/* case ON: */
	default:
		vdcdc1 |= TPS_ENABLE_LP;  /* enable ENABLE_LP bit */
		break;
	}

	status = i2c_smbus_write_byte_data(&the_tps->client,
			TPS_VDCDC1, vdcdc1);

	if (status != 0)
		printk(KERN_ERR "%s: Failed to write vdcdc1 register\n", 
		       DRIVER_NAME);
	else
		pr_debug("%s: vdcdc1 0x%02x\n", DRIVER_NAME,
			i2c_smbus_read_byte_data(&the_tps->client, TPS_VDCDC1));

	up(&the_tps->lock);

	return status;
}
EXPORT_SYMBOL(tps65010_set_low_pwr);

/*-------------------------------------------------------------------------*/
/* tps65013_set_low_pwr parameter:
 * mode: ON or OFF
 */

/* FIXME: Assumes AC or USB power is present. Setting AUA bit is not
	required if power supply is through a battery */

int tps65013_set_low_pwr(unsigned mode)
{
	int	 status;
	unsigned vdcdc1, chgconfig;

	if (!the_tps)
		return -ENODEV;

	down(&the_tps->lock);

	pr_debug("%s: %s low_pwr, chgconfig 0x%02x vdcdc1 0x%02x\n", DRIVER_NAME,
		mode ? "enable" : "disable",
		i2c_smbus_read_byte_data(&the_tps->client, TPS_CHGCONFIG),
		i2c_smbus_read_byte_data(&the_tps->client, TPS_VDCDC1));

	chgconfig = i2c_smbus_read_byte_data(&the_tps->client, TPS_CHGCONFIG);
	vdcdc1 = i2c_smbus_read_byte_data(&the_tps->client, TPS_VDCDC1);

	switch (mode) {
	case OFF:
		chgconfig &= ~TPS65013_AUA; /* disable AUA bit */
		vdcdc1 &= ~TPS_ENABLE_LP; /* disable ENABLE_LP bit */
		break;
	/* case ON: */
	default:
		chgconfig |= TPS65013_AUA;  /* enable AUA bit */
		vdcdc1 |= TPS_ENABLE_LP;  /* enable ENABLE_LP bit */
		break;
	}

	status = i2c_smbus_write_byte_data(&the_tps->client,
			TPS_CHGCONFIG, chgconfig);
	if (status != 0) {
		printk(KERN_ERR "%s: Failed to write chconfig register\n",
	 DRIVER_NAME);
		up(&the_tps->lock);
		return status;
	}

	pr_debug("%s: chgconfig 0x%02x\n", DRIVER_NAME,
			i2c_smbus_read_byte_data(&the_tps->client, TPS_CHGCONFIG));

	status = i2c_smbus_write_byte_data(&the_tps->client,
			TPS_VDCDC1, vdcdc1);

	if (status != 0)
		printk(KERN_ERR "%s: Failed to write vdcdc1 register\n",
	 DRIVER_NAME);
	else
		pr_debug("%s: vdcdc1 0x%02x\n", DRIVER_NAME,
			i2c_smbus_read_byte_data(&the_tps->client, TPS_VDCDC1));

	up(&the_tps->lock);

	return status;
}
EXPORT_SYMBOL(tps65013_set_low_pwr);

/*-------------------------------------------------------------------------*/

static int __init tps_init(void)
{
	u32	tries = 3;
	int	status;

	printk(KERN_INFO "%s: version %s\n", DRIVER_NAME, DRIVER_VERSION);

	/* some boards have startup glitches */
	while (tries--) {
		status = i2c_add_driver(&tps65010_driver);
		if (the_tps)
			break;
		i2c_del_driver(&tps65010_driver);
		if (!tries) {
			printk(KERN_ERR "%s: no chip?\n", DRIVER_NAME);
			return -ENODEV;
		}
		pr_debug("%s: re-probe ...\n", DRIVER_NAME);
		msleep(10);
	}

#if defined(CONFIG_ARM)
	if (machine_is_omap_osk()) {

		// FIXME: This should be placed in the initialization code
		//        of the submodules (USB, ethernet, power management, 
		//        board-osk.c). But because I2C is initialized so
		//        late in boot process no I2C access is available where
		//        this code really belongs to.

		/* Let LED1 (D9) blink */
		tps65010_set_led(LED1, BLINK);

		/* Disable LED 2 (D2) */
		tps65010_set_led(LED2, OFF);

		/* Set GPIO 1 high to enable USB */
		tps65010_set_gpio_out_value(GPIO1, LOW);

		/* Disable LED on GPIO2 (D3) */
		tps65010_set_gpio_out_value(GPIO2, HIGH);

		/* Set GPIO 3 low to enable ethernet */
		tps65010_set_gpio_out_value(GPIO3, LOW);

		/* Enable LOW_PWR */
		tps65010_set_low_pwr(ON);

	} else if (machine_is_omap_h2()) {
		/* gpio3 for SD, gpio4 for VDD_DSP */

		/* Enable LOW_PWR */
		tps65010_set_low_pwr(ON);
	} else if (machine_is_omap_h3()) {
		/* gpio4 for SD, gpio3 for VDD_DSP */
#ifdef CONFIG_PM
		/* Enable LOW_PWR */
		tps65013_set_low_pwr(ON);
#endif
	}
#endif
#ifdef CONFIG_PREEMPT_RT
	spin_lock_init(&sched_lock);
#endif

	return status;
}
subsys_initcall(tps_init);

static void __exit tps_exit(void)
{
	i2c_del_driver(&tps65010_driver);
}
module_exit(tps_exit);

