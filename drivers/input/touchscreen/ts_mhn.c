/*
 *  drivers/input/touchscreen/ts_mhn.c.
 *
 *  Author:	amakarov@ru.mvista.com
 *  Created:	Jul 16, 2007
 *  Copyright:	MontaVista
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>

#include <asm/arch/zylonite.h>
#include <asm/arch/mhn_gpio.h>

#include <linux/input.h>
#include <asm/arch/pxa3xx-wm9713l.h>

static struct input_dev ts_input_dev;

#define TS_IRQ (IRQ_GPIO(MFP2GPIO(MFP_AC97_INT_N_GPIO)))

#define TS_PERIOD (HZ/100) /* 10 ms */

static struct work_struct ts_workqueue;

static void ts_workqueue_func(void * arg)
{

	unsigned short x;
	unsigned short y;

	if (pxa3xx_wm9713l_ts_get_sample(&x, &y)) {

		/* pen up */

		input_report_abs(&ts_input_dev, BTN_TOUCH, 0);
		input_report_abs(&ts_input_dev, ABS_PRESSURE, 0);
		input_sync(&ts_input_dev);

		enable_irq(TS_IRQ);
		pxa3xx_wm9713l_ts_irq_reset();

	} else {

		/* pen down */

		input_report_abs(&ts_input_dev, BTN_TOUCH, 1);
		input_report_abs(&ts_input_dev, ABS_X, x);
		input_report_abs(&ts_input_dev, ABS_Y, y);
		input_report_abs(&ts_input_dev, ABS_PRESSURE, 100);
		input_sync(&ts_input_dev);

		schedule_delayed_work(&ts_workqueue, TS_PERIOD);
	}
}

static irqreturn_t ts_irq(int irq, void *dev, struct pt_regs *regs)
{
	disable_irq(TS_IRQ);
	schedule_work(&ts_workqueue);

	return IRQ_HANDLED;
}

static int __devinit ts_probe(struct device *dev)
{
	int err;

	init_input_dev(&ts_input_dev);

	ts_input_dev.evbit[0] = BIT(EV_ABS);
	ts_input_dev.absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);
	ts_input_dev.keybit[LONG(BTN_TOUCH)] = BIT(BTN_TOUCH);

	ts_input_dev.absmin[ABS_X] = 0;
	ts_input_dev.absmin[ABS_Y] = 0;
	ts_input_dev.absmin[ABS_PRESSURE] = 0;
	ts_input_dev.absmax[ABS_X] = 0xfff;
	ts_input_dev.absmax[ABS_Y] = 0xfff;
	ts_input_dev.absmax[ABS_PRESSURE] = 100;

	ts_input_dev.name = "zylonite touchscreen";
	ts_input_dev.phys = "input0";

	input_register_device(&ts_input_dev);

	mhn_mfp_set_afds(MFP_AC97_INT_N_GPIO,0,0);
	mhn_gpio_set_direction(MFP_AC97_INT_N_GPIO, GPIO_DIR_IN);

	mhn_gpio_clear_edge_detect_status(MFP_AC97_INT_N_GPIO);
	set_irq_type(TS_IRQ, IRQT_RISING);

	err = pxa3xx_wm9713l_ts_init();
	if (err) {
		input_unregister_device(&ts_input_dev);
		return err;
	}

	pxa3xx_wm9713l_ts_irq_reset();

	err = request_irq(TS_IRQ, ts_irq, 0, "touchscreen", NULL);
	if (err) {
		pxa3xx_wm9713l_ts_exit();
		input_unregister_device(&ts_input_dev);
		return err;
	}

	input_report_abs(&ts_input_dev, BTN_TOUCH, 0);
	input_sync(&ts_input_dev);

	return 0;
}

static int __devexit ts_remove(struct device *dev)
{
	free_irq(IRQ_GPIO(MFP2GPIO(MFP_AC97_INT_N_GPIO)), NULL);
	pxa3xx_wm9713l_ts_exit();
        cancel_delayed_work(&ts_workqueue);
        flush_scheduled_work();
	input_unregister_device(&ts_input_dev);

	return 0;
}

#ifdef CONFIG_PM
static int ts_suspend(struct device *_dev, u32 state, u32 level)
{
	if (level == SUSPEND_DISABLE)
		pxa3xx_wm9713l_ts_suspend();

	return 0;
}

static int ts_resume(struct device *_dev, u32 level)
{
	if (level == RESUME_ENABLE)
		pxa3xx_wm9713l_ts_resume();

	return 0;
}
#else
#define ts_suspend	NULL
#define ts_resume	NULL
#endif

static struct device_driver ts_driver = {
	.name	=	"pxa2xx-touch",
	.bus	=	&platform_bus_type,
	.probe	=	ts_probe,
	.remove =	__devexit_p(ts_remove),
	.suspend=	ts_suspend,
	.resume =	ts_resume,
};

static int __init ts_init(void)
{
	INIT_WORK(&ts_workqueue, ts_workqueue_func, NULL);
	return driver_register(&ts_driver);
}

static void __exit ts_exit(void)
{
	driver_unregister(&ts_driver);
}

module_init(ts_init);
module_exit(ts_exit);

MODULE_AUTHOR("amakarov@ru.mvista.com");
MODULE_DESCRIPTION("Driver for touchscreen Zylonite Dev Board (wm9713l)");
MODULE_LICENSE("GPL");

