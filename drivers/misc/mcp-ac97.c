/*
 * linux/drivers/misc/mcp-ac97.c
 *
 * Author:	Nicolas Pitre
 * Created:	Jan 14, 2005
 * Copyright:	(C) MontaVista Software Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This module provides the minimum replacement for mcp-core.c allowing for
 * the UCB1400 chip to be driven by the ucb1x00 driver over an AC97 link.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/device.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/ac97_codec.h>

#include "mcp.h"


unsigned int mcp_reg_read(struct mcp *mcp, unsigned int reg)
{
	ac97_t *ac97 = mcp->me->platform_data;
	return ac97->bus->ops->read(ac97, reg);
}

void mcp_reg_write(struct mcp *mcp, unsigned int reg, unsigned int val)
{
	ac97_t *ac97 = mcp->me->platform_data;
	ac97->bus->ops->write(ac97, reg, val);
}

void mcp_enable(struct mcp *mcp)
{
}

void mcp_disable(struct mcp *mcp)
{
}

#define to_mcp_driver(d)	container_of(d, struct mcp_driver, drv)

static int mcp_probe(struct device *dev)
{
	struct mcp_driver *drv = to_mcp_driver(dev->driver);
	struct mcp *mcp;
	int ret;

	ret = -ENOMEM;
	mcp = kmalloc(sizeof(*mcp), GFP_KERNEL);
	if (mcp) {
		memset(mcp, 0, sizeof(*mcp));
		mcp->owner = THIS_MODULE;
		mcp->reg_read = mcp_reg_read;
		mcp->reg_write = mcp_reg_write;
		mcp->enable = mcp_enable;
		mcp->disable = mcp_disable;
		mcp->me = dev;
		mcp->attached_device = dev;

		ret = drv->probe(mcp);
		if (ret)
			kfree(mcp);
	}
	return ret;
}

static int mcp_remove(struct device *dev)
{
	struct mcp_driver *drv = to_mcp_driver(dev->driver);
	struct mcp *mcp = *(struct mcp **)dev_get_drvdata(dev);

	drv->remove(mcp);
	kfree(mcp);
	return 0;
}

static int mcp_suspend(struct device *dev, u32 state, u32 level)
{
	struct mcp_driver *drv = to_mcp_driver(dev->driver);
	struct mcp *mcp = dev_get_drvdata(dev);
	int ret = 0;

	if (drv->suspend && level == SUSPEND_DISABLE)
		ret = drv->suspend(mcp, state);
	return ret;
}

static int mcp_resume(struct device *dev, u32 level)
{
	struct mcp_driver *drv = to_mcp_driver(dev->driver);
	struct mcp *mcp = dev_get_drvdata(dev);
	int ret = 0;

	if (drv->resume && level == RESUME_ENABLE)
		ret = drv->resume(mcp);
	return ret;
}

int mcp_driver_register(struct mcp_driver *mcpdrv)
{
	mcpdrv->drv.name = "UCB1400";
	mcpdrv->drv.bus = &ac97_bus_type;
	mcpdrv->drv.probe = mcp_probe;
	mcpdrv->drv.remove = mcp_remove;
	mcpdrv->drv.suspend = mcp_suspend;
	mcpdrv->drv.resume = mcp_resume;
	return driver_register(&mcpdrv->drv);
}

void mcp_driver_unregister(struct mcp_driver *mcpdrv)
{
	driver_unregister(&mcpdrv->drv);
}

EXPORT_SYMBOL(mcp_reg_read);
EXPORT_SYMBOL(mcp_reg_write);
EXPORT_SYMBOL(mcp_enable);
EXPORT_SYMBOL(mcp_disable);
EXPORT_SYMBOL(mcp_driver_register);
EXPORT_SYMBOL(mcp_driver_unregister);

MODULE_LICENSE("GPL");
