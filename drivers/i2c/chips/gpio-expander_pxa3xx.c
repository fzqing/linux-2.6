/*
 * drivers/i2c/chips/gpio-expander.c
 *
 * Author:	yfw
 * Copyright:	Intel Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/config.h>
#include <linux/kernel_stat.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/signal.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/kthread.h>

#include <asm/system.h>
#include <asm/hardware.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/leds.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/mfp.h>
#include <asm/arch/mhn_gpio.h>

#define TO_idx(x)	((x) >> 4)
#define TO_bit(x)	((x) & 0xf)

struct pxa_gpio_exp {
	struct i2c_client i2c_client;
	struct work_struct work;
	unsigned long rising_edge;
	unsigned long falling_edge;
	unsigned long input_level;
	int addr;
	int irq;
	int gpio_start;
	int gpio_end;
	int id;
	uint8_t regs[8];
};

static int nr;
static struct pxa_gpio_exp *g_gpio_exp[GPIO_EXP_NUM];

static DECLARE_MUTEX (gpio_exp_lock);

static int gpio_exp_read(struct pxa_gpio_exp *gpio_exp, uint8_t reg,
			 uint8_t * val);
static int gpio_exp_write(struct pxa_gpio_exp *gpio_exp, uint8_t reg,
			  uint8_t val);

static void do_gpio_exp_irq(unsigned int irq, struct irqdesc *desc,
			    struct pt_regs *regs)
{
	const unsigned int cpu = smp_processor_id();

	/*
	 * Earlier this was desc->triggered = 1;
	 */

	/*
	 * Acknowledge, clear _AND_ disable the interrupt.
	 */
	desc->chip->ack(irq);
	desc->chip->mask(irq);

	kstat_cpu(cpu).irqs[irq]++;

	if (desc->thread && desc->thread->state != TASK_RUNNING)
		wake_up_process(desc->thread);
	desc->chip->unmask(irq);
}

static int gpio_exp_thread(void *data)
{

	struct pxa_gpio_exp *gpio_exp = (struct pxa_gpio_exp *)data;

	current->flags |= PF_NOFREEZE;

	while (!kthread_should_stop()) {
		unsigned int i;
		struct irqdesc *desc;
		u16 cur_gpio_ext_input;
		u8 tmp0, tmp1;
		int bit;
		int gpio_ext_irq_pending = 0;
		const unsigned int cpu = smp_processor_id();

		disable_irq(gpio_exp->irq);
		up(&gpio_exp_lock);
		gpio_exp_read(gpio_exp, 0, &tmp0);
		gpio_exp_read(gpio_exp, 1, &tmp1);
		down(&gpio_exp_lock);
		cur_gpio_ext_input = tmp0 | (tmp1 << 8);

		for (i = gpio_exp->gpio_start; i < gpio_exp->gpio_end; i++) {

			gpio_ext_irq_pending = 0;
			bit = TO_bit(i - gpio_exp->gpio_start);
			if (gpio_exp->rising_edge & (1 << bit)) {
				if ((!(gpio_exp->input_level & (1 << bit))) &&
				    (cur_gpio_ext_input & (1 << bit))) {
					gpio_ext_irq_pending = 1;
				}
			}

			if (gpio_exp->falling_edge & (1 << bit)) {
				if ((gpio_exp->input_level & (1 << bit)) &&
				    (!(cur_gpio_ext_input & (1 << bit)))) {
					gpio_ext_irq_pending = 1;
				}
			}
			if (gpio_ext_irq_pending) {
				desc = irq_desc + GPIO_EXT_TO_IRQ(i);
				irq_enter();

				kstat_cpu(cpu).irqs[GPIO_EXT_TO_IRQ(i)]++;
				desc->handle(GPIO_EXT_TO_IRQ(i), desc, NULL);

				irq_exit();
			}
		}

		enable_irq(gpio_exp->irq);
		gpio_exp->input_level = cur_gpio_ext_input;

		local_irq_disable();
		set_current_state(TASK_INTERRUPTIBLE);
		local_irq_enable();

		schedule();

	}
	set_current_state(TASK_RUNNING);
	return 0;
}

struct task_struct *start_gpio_exp_thread(struct pxa_gpio_exp *gpio_exp)
{
	struct irqdesc *desc = irq_desc + gpio_exp->irq;

	desc->thread = kthread_create(gpio_exp_thread, (void *)gpio_exp,
				      "gpio_expander_irq %d", gpio_exp->irq);

	if (!desc->thread)
		printk(KERN_ERR
		       "%s: could not create gpio exp irq %d thread!\n",
		       __FUNCTION__, gpio_exp->irq);

	return desc->thread;
}

static int pxa_gpio_expander_irq_type(unsigned
				      int irq, unsigned
				      int type)
{

	int gpio_ext, gpio_ext_bit;
	int i = 0;
	struct pxa_gpio_exp *gpio_exp = NULL;

	gpio_ext = IRQ_TO_GPIO_EXT(irq);

	while (i < GPIO_EXP_NUM) {
		if (g_gpio_exp[i] && g_gpio_exp[i]->gpio_start <= gpio_ext
		    && g_gpio_exp[i]->gpio_end >= gpio_ext) {
			gpio_exp = g_gpio_exp[i];
			break;
		}
		i++;
	}
	if (!gpio_exp)
		return -EINVAL;

	gpio_ext_bit = (gpio_ext - gpio_exp->gpio_start) & 0x0f;
	if (type == IRQT_PROBE) {
		if ((gpio_exp->rising_edge | gpio_exp->falling_edge)
		    & (1 << gpio_ext_bit))
			return 0;

		type = __IRQT_RISEDGE | __IRQT_FALEDGE;
	}

	if (type & __IRQT_RISEDGE) {
		__set_bit(gpio_ext_bit, &gpio_exp->rising_edge);
	} else
		__clear_bit(gpio_ext_bit, &gpio_exp->rising_edge);

	if (type & __IRQT_FALEDGE) {
		__set_bit(gpio_ext_bit, &gpio_exp->falling_edge);
	} else
		__clear_bit(gpio_ext_bit, &gpio_exp->falling_edge);

	return 0;
}

static struct irqchip pxa_gpio_expander_chip = {
	.type = pxa_gpio_expander_irq_type,
	.ack = dummy_mask_unmask_irq,
	.mask = dummy_mask_unmask_irq,
	.unmask = dummy_mask_unmask_irq,
};

int gpio_exp_set_direction(int gpio_exp_id, int dir)
{
	uint8_t tmp, bit, reg;
	int i = 0;
	struct pxa_gpio_exp *gpio_exp = NULL;

	while (i < GPIO_EXP_NUM) {
		if (g_gpio_exp[i] && gpio_exp_id <= g_gpio_exp[i]->gpio_end
		    && gpio_exp_id >= g_gpio_exp[i]->gpio_start) {
			gpio_exp = g_gpio_exp[i];
			break;
		}
		i++;
	}
	if (!gpio_exp)
		return -EINVAL;
	bit = gpio_exp_id - gpio_exp->gpio_start;

	if (bit < (gpio_exp->gpio_end - gpio_exp->gpio_start + 1) / 2) {
		reg = 6;
	} else {
		reg = 7;
		bit -= (gpio_exp->gpio_end - gpio_exp->gpio_start + 1) / 2;
	}
	up(&gpio_exp_lock);
	gpio_exp_read(gpio_exp, reg, &tmp);
	if (dir == GPIO_DIR_IN)
		tmp |= (0x01u << bit);
	else
		tmp &= ~(0x01u << bit);

	gpio_exp_write(gpio_exp, reg, tmp);
	down(&gpio_exp_lock);

	return 0;
}

int gpio_exp_get_direction(int gpio_exp_id)
{
	uint8_t tmp, bit, reg;
	int i = 0;
	struct pxa_gpio_exp *gpio_exp = NULL;

	while (i < GPIO_EXP_NUM) {
		if (g_gpio_exp[i] && gpio_exp_id <= g_gpio_exp[i]->gpio_end
		    && gpio_exp_id >= g_gpio_exp[i]->gpio_start) {
			gpio_exp = g_gpio_exp[i];
			break;
		}
		i++;
	}
	if (!gpio_exp)
		return -EINVAL;
	bit = gpio_exp_id - gpio_exp->gpio_start;

	if (bit < (gpio_exp->gpio_end - gpio_exp->gpio_start + 1) / 2) {
		reg = 6;
	} else {
		reg = 7;
		bit -= (gpio_exp->gpio_end - gpio_exp->gpio_start + 1) / 2;
	}
	up(&gpio_exp_lock);
	gpio_exp_read(gpio_exp, reg, &tmp);
	down(&gpio_exp_lock);

	if (tmp & (1u << bit))
		return GPIO_DIR_IN;
	else
		return GPIO_DIR_OUT;

}

int gpio_exp_set_level(int gpio_exp_id, int level)
{
	uint8_t tmp, bit, reg;
	int i = 0;
	struct pxa_gpio_exp *gpio_exp = NULL;

	while (i < GPIO_EXP_NUM) {
		if (g_gpio_exp[i] && gpio_exp_id <= g_gpio_exp[i]->gpio_end
		    && gpio_exp_id >= g_gpio_exp[i]->gpio_start) {
			gpio_exp = g_gpio_exp[i];
			break;
		}
		i++;
	}
	if (!gpio_exp)
		return -EINVAL;
	bit = gpio_exp_id - gpio_exp->gpio_start;

	if (bit < (gpio_exp->gpio_end - gpio_exp->gpio_start + 1) / 2) {
		reg = 2;
	} else {
		reg = 3;
		bit -= (gpio_exp->gpio_end - gpio_exp->gpio_start + 1) / 2;
	}
	up(&gpio_exp_lock);
	gpio_exp_read(gpio_exp, reg, &tmp);

	if (level == GPIO_LEVEL_LOW)
		tmp &= ~(0x01 << bit);
	else
		tmp |= (0x01 << bit);

	gpio_exp_write(gpio_exp, reg, tmp);
	down(&gpio_exp_lock);

	return 0;
}

int gpio_exp_get_level(int gpio_exp_id)
{
	uint8_t tmp, bit, reg;
	int i = 0;
	struct pxa_gpio_exp *gpio_exp = NULL;

	while (i < GPIO_EXP_NUM) {
		if (g_gpio_exp[i] && gpio_exp_id <= g_gpio_exp[i]->gpio_end
		    && gpio_exp_id >= g_gpio_exp[i]->gpio_start) {
			gpio_exp = g_gpio_exp[i];
			break;
		}
		i++;
	}
	if (!gpio_exp)
		return -EINVAL;
	bit = gpio_exp_id - gpio_exp->gpio_start;

	if (bit < (gpio_exp->gpio_end - gpio_exp->gpio_start + 1) / 2) {
		reg = 0;
	} else {
		reg = 1;
		bit -= (gpio_exp->gpio_end - gpio_exp->gpio_start + 1) / 2;
	}
	up(&gpio_exp_lock);
	gpio_exp_read(gpio_exp, reg, &tmp);
	down(&gpio_exp_lock);
	if (tmp & (1u << bit))
		return GPIO_LEVEL_HIGH;
	else
		return GPIO_LEVEL_LOW;
}

EXPORT_SYMBOL(gpio_exp_set_direction);
EXPORT_SYMBOL(gpio_exp_get_direction);
EXPORT_SYMBOL(gpio_exp_set_level);
EXPORT_SYMBOL(gpio_exp_get_level);

#ifdef CONFIG_PM

/*
 * GPIO Expander will be reset during suspend/resume, registers should
 * be saved/restored for proper operation
 */

static int gpio_exp_suspend(struct device *dev, u32 state, u32 level)
{
	int i;
	struct pxa_gpio_exp *gpio_exp = dev_get_drvdata(dev);

	if (level == SUSPEND_POWER_DOWN) {
		/* skip saving input level register 0, 1 */
		up(&gpio_exp_lock);
		for (i = 2; i < 8; i++)
			gpio_exp_read(gpio_exp, i, &gpio_exp->regs[i]);
		down(&gpio_exp_lock);
	}
	return 0;
}

static int gpio_exp_resume(struct device *dev, u32 level)
{
	int i;
	uint8_t tmp0, tmp1;
	struct pxa_gpio_exp *gpio_exp = dev_get_drvdata(dev);

	if (level == RESUME_ENABLE) {
		/* skip restoring input level register 0, 1 */
		for (i = 2; i < 8; i++)
			gpio_exp_write(gpio_exp, i, gpio_exp->regs[i]);

		/* read the input level registers to
		 * 1) clock the input to internal registers
		 * 2) clear interrupt if any
		 * 3) record the current level
		 */
		up(&gpio_exp_lock);
		gpio_exp_read(gpio_exp, 0, &tmp0);
		gpio_exp_read(gpio_exp, 1, &tmp1);
		down(&gpio_exp_lock);
		gpio_exp->input_level = tmp0 | (tmp1 << 8);

	}
	return 0;
}
#endif

static int __init gpio_exp_probe(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	uint8_t tmp0, tmp1;
	struct resource *r;
	struct pxa_gpio_exp *gpio_exp = NULL;
	int i = 0, gpio, addr;

	printk(KERN_INFO "GPIO-expander driver for Monahans\n");

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r)
		return -ENXIO;

	addr = r->start;
	while (i < GPIO_EXP_NUM) {
		if (g_gpio_exp[i] && g_gpio_exp[i]->addr == addr) {
			gpio_exp = g_gpio_exp[i];
			break;
		}
		i++;
	}
	if (!gpio_exp)
		return -ENXIO;

	/* Initialize the MFPR and GPIO settings of the two pins
	 * connected to the GPIO expander
	 */
	gpio_exp->irq = platform_get_irq(pdev, 0);
	/* Some GPIO expander never trigger interrupt */
	if (gpio_exp->irq < 0)
		gpio_exp->irq = 0;

	r = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (!r)
		return -ENXIO;
	gpio_exp->gpio_start = r->start;
	gpio_exp->gpio_end = r->end;

	if (gpio_exp->irq == 0)
		goto exit;

	/* read in the initial pin levels */
	tmp0 = tmp1 = 0;
	up(&gpio_exp_lock);
	gpio_exp_read(gpio_exp, 0, &tmp0);
	gpio_exp_read(gpio_exp, 1, &tmp1);
	down(&gpio_exp_lock);
	gpio_exp->input_level = tmp0 | (tmp1 << 8);

	for (gpio = gpio_exp->gpio_start; gpio < gpio_exp->gpio_end; gpio++) {
		set_irq_chip(GPIO_EXT_TO_IRQ(gpio), &pxa_gpio_expander_chip);
		set_irq_handler(GPIO_EXT_TO_IRQ(gpio), do_simple_IRQ);
		set_irq_flags(GPIO_EXT_TO_IRQ(gpio), IRQF_VALID | IRQF_PROBE);
	}

	set_irq_data(gpio_exp->irq, start_gpio_exp_thread(gpio_exp));
	set_irq_type(gpio_exp->irq, IRQT_FALLING);
	set_irq_chained_handler(gpio_exp->irq, do_gpio_exp_irq);
      exit:
	dev_set_drvdata(dev, gpio_exp);

	return 0;
}

static struct device_driver gpio_exp_driver = {
	.name = "gpio-exp",
	.bus = &platform_bus_type,
	.probe = gpio_exp_probe,
#ifdef CONFIG_PM
	.suspend = gpio_exp_suspend,
	.resume = gpio_exp_resume,
#endif
};

/*********************************************************************************/
/*                                                                               */
/*                              GPIO Expander I2C Client Driver                  */
/*                                                                               */
/*********************************************************************************/
#include <linux/i2c.h>
static int i2c_gpio_exp_attach_adapter(struct i2c_adapter *adapter);
static int i2c_gpio_exp_detect_client(struct i2c_adapter *, int, int);
static int i2c_gpio_exp_detach_client(struct i2c_client *client);
#define I2C_DRIVERID_GPIOEXP   I2C_DRIVERID_EXP1

static struct i2c_driver i2c_gpio_exp_driver = {
	.owner = THIS_MODULE,
	.name = "gpio expander i2c client driver",
	.id = I2C_DRIVERID_GPIOEXP,
	.flags = I2C_DF_NOTIFY,
	.attach_adapter = &i2c_gpio_exp_attach_adapter,
	.detach_client = &i2c_gpio_exp_detach_client,
};

/* Unique ID allocation */

static unsigned short normal_i2c[] =
    { GPIO_EXP0_ADDRESS, GPIO_EXP1_ADDRESS, I2C_CLIENT_END };
static unsigned short normal_i2c_range[] = { I2C_CLIENT_END };

I2C_CLIENT_INSMOD;

static int gpio_exp_write(struct pxa_gpio_exp *gpio_exp, uint8_t reg,
			  uint8_t val)
{
	return i2c_smbus_write_byte_data(&gpio_exp->i2c_client, reg, val);
}

static int gpio_exp_read(struct pxa_gpio_exp *gpio_exp, uint8_t reg,
			 uint8_t * val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(&gpio_exp->i2c_client, reg);
	if (ret >= 0) {
		*val = ret;
		return 0;
	} else
		return -EIO;
}

static int i2c_gpio_exp_read(struct i2c_client *client, u8 reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

static int i2c_gpio_exp_attach_adapter(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, &i2c_gpio_exp_detect_client);
}

static int i2c_gpio_exp_detect_client(struct i2c_adapter *adapter, int address,
				      int kind)
{
	struct pxa_gpio_exp *gpio_exp;
	int err = 0;

	/* Let's see whether this adapter can support what we need.
	   Please substitute the things you need here!  */
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk(KERN_INFO "byte op is not permited.\n");
		goto ERROR0;
	}

	/* OK. For now, we presume we have a valid client. We now create the
	   client structure, even though we cannot fill it completely yet.
	   But it allows us to access several i2c functions safely */

	if (nr >= GPIO_EXP_NUM) {
		err = -ENOMEM;
		goto ERROR0;
	}
	gpio_exp = kzalloc(sizeof(struct pxa_gpio_exp), GFP_KERNEL);

	if (!gpio_exp) {
		err = -ENOMEM;
		goto ERROR0;
	}

	gpio_exp->i2c_client.addr = address;
	gpio_exp->i2c_client.adapter = adapter;
	gpio_exp->i2c_client.driver = &i2c_gpio_exp_driver;
	gpio_exp->i2c_client.flags = 0;

	if (i2c_gpio_exp_read(&gpio_exp->i2c_client, 0) < 0) {
		goto ERROR1;
	} else {
		printk(KERN_INFO "GPIO expander %d detected!\n", nr);
	}

	sprintf(gpio_exp->i2c_client.name, "GPIO Expander %d", nr);

	/* Tell the i2c layer a new client has arrived */
	if ((err = i2c_attach_client(&gpio_exp->i2c_client)))
		goto ERROR1;

	gpio_exp->id = nr;
	gpio_exp->addr = address;
	g_gpio_exp[nr++] = gpio_exp;
	return 0;

      ERROR1:
	kfree(gpio_exp);
      ERROR0:
	return err;
}

static int i2c_gpio_exp_detach_client(struct i2c_client *client)
{
	int err;
	int i;
	struct pxa_gpio_exp *gpio_exp
	    = container_of(client, struct pxa_gpio_exp, i2c_client);

	/* Try to detach the client from i2c space */
	if ((err = i2c_detach_client(client))) {
		printk(KERN_INFO
		       "gpio_expander.o: Client deregistration failed, client not detached.\n");
		return err;
	}

	for (i = 0; i < GPIO_EXP_NUM; i++) {
		if (g_gpio_exp[i] == gpio_exp)
			g_gpio_exp[i] = NULL;
	}

	kfree(gpio_exp);	/* Frees client data too, if allocated at the same time */
	return 0;
}

static int __init pxa_gpio_exp_init(void)
{
	int ret;

	if ((ret = i2c_add_driver(&i2c_gpio_exp_driver))) {
		printk(KERN_INFO
		       "gpio_expander: Driver registration failed, module not inserted.\n");
		return ret;
	}

	return driver_register(&gpio_exp_driver);
}

static void __exit pxa_gpio_exp_exit(void)
{
	driver_unregister(&gpio_exp_driver);
	if (i2c_del_driver(&i2c_gpio_exp_driver)) {
		printk(KERN_INFO
		       "gpio_expander: Driver registration failed, module not removed.\n");
	}

}

MODULE_DESCRIPTION("GPIO Expander Driver");
MODULE_LICENSE("GPL");

module_init(pxa_gpio_exp_init);
module_exit(pxa_gpio_exp_exit);
