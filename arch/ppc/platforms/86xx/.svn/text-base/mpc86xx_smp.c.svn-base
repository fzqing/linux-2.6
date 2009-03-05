/*
 * Author: Xianghua Xiao <x.xiao@freescale.com>
 *         Zhang Wei <wei.zhang@freescale.com>
 *
 * Copyright 2006 Freescale Semiconductor Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>

#include <asm/pci-bridge.h>
#include <asm/open_pic.h>
#include <asm/mpc86xx.h>

static void __init
smp_86xx_kick_cpu(int nr)
{
	bd_t *binfo = (bd_t *) __res;
	__be32 __iomem *mcm_vaddr;
	unsigned long pcr;

	if (nr < 0 || nr >= NR_CPUS)
		return;

	pr_debug("smp_86xx_kick_cpu: kick CPU #%d\n", nr);

	/*
	 * Startup Core #nr.
	 */
	mcm_vaddr = ioremap(binfo->bi_immr_base + MPC86xx_MCM_OFFSET,
			    MPC86xx_MCM_SIZE);
	pcr = in_be32(mcm_vaddr + (MCM_PORT_CONFIG_OFFSET >> 2));
	pcr |= 1 << (nr + 24);
	out_be32(mcm_vaddr + (MCM_PORT_CONFIG_OFFSET >> 2), pcr);
}


static void __init
smp_86xx_setup_cpu(int cpu_nr)
{
	if (OpenPIC_Addr)
		do_openpic_setup_cpu();
}


static int __init
smp_86xx_probe(void)
{
	openpic_request_IPIs();
	smp_hw_index[1] = 1;
	return 2;
}


static void __init
smp_86xx_space_timers(int nr)
{
	(void)nr;
	return;
}


struct smp_ops_t smp_86xx_ops = {
	.message_pass = smp_openpic_message_pass,
	.probe = smp_86xx_probe,
	.kick_cpu = smp_86xx_kick_cpu,
	.space_timers = smp_86xx_space_timers,
	.setup_cpu = smp_86xx_setup_cpu,
	.take_timebase = smp_generic_take_timebase,
	.give_timebase = smp_generic_give_timebase,
};


void __init
mpc86xx_smp_init(void)
{
	ppc_md.smp_ops = &smp_86xx_ops;
}
