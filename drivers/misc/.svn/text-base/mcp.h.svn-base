/*
 *  linux/drivers/misc/mcp.h
 *
 *  Copyright (C) 2001 Russell King, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 */
#ifndef MCP_H
#define MCP_H

#ifdef CONFIG_ARCH_SA1100
#include <asm/dma.h>
#endif

struct mcp {
	struct module	*owner;
	struct device	*me;
	spinlock_t	lock;
	int		use_count;
	unsigned int	sclk_rate;
#ifdef CONFIG_ARCH_SA1100
	unsigned int	rw_timeout;
	dma_device_t	dma_audio_rd;
	dma_device_t	dma_audio_wr;
	dma_device_t	dma_telco_rd;
	dma_device_t	dma_telco_wr;
	void		(*set_telecom_divisor)(struct mcp *, unsigned int);
	void		(*set_audio_divisor)(struct mcp *, unsigned int);
#endif
	void		(*reg_write)(struct mcp *, unsigned int, unsigned int);
	unsigned int	(*reg_read)(struct mcp *, unsigned int);
	void		(*enable)(struct mcp *);
	void		(*disable)(struct mcp *);
	struct device	*attached_device;
};

void mcp_set_telecom_divisor(struct mcp *, unsigned int);
void mcp_set_audio_divisor(struct mcp *, unsigned int);
void mcp_reg_write(struct mcp *, unsigned int, unsigned int);
unsigned int mcp_reg_read(struct mcp *, unsigned int);
void mcp_enable(struct mcp *);
void mcp_disable(struct mcp *);
#define mcp_get_sclk_rate(mcp)	((mcp)->sclk_rate)

int mcp_host_register(struct mcp *, struct device *);
void mcp_host_unregister(struct mcp *);

struct mcp_driver {
	struct device_driver drv;
	int (*probe)(struct mcp *);
	void (*remove)(struct mcp *);
	int (*suspend)(struct mcp *, u32);
	int (*resume)(struct mcp *);
};

int mcp_driver_register(struct mcp_driver *);
void mcp_driver_unregister(struct mcp_driver *);

#define mcp_get_drvdata(mcp)	dev_get_drvdata((mcp)->attached_device)
#define mcp_set_drvdata(mcp,d)	dev_set_drvdata((mcp)->attached_device, d)

#endif
