/*
 * linux/arch/arm/mach-davinci/io.c
 *
 * DaVinci I/O mapping code
 *
 * Copyright (C) 2005-2006 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <asm/tlb.h>
#include <asm/io.h>

#include <asm/mach/map.h>
#include <asm/arch/memory.h>
#include <asm/arch/cpu.h>
#include <asm/arch/mux.h>

extern int davinci_clk_init(void);
extern void davinci_check_revision(void);
unsigned int davinci_cpu_index = DM644X_CPU_IDX;

/*
 * The machine specific code may provide the extra mapping besides the
 * default mapping provided here.
 */
static struct map_desc davinci_io_desc[] __initdata = {
	{
		.virtual	= IO_VIRT,
		.physical	= IO_PHYS,
		.length		= IO_SIZE,
		.type		= MT_DEVICE,
	},
};
static struct map_desc dm644x_io_desc[] __initdata = {
	{
		.virtual	= DAVINCI_IRAM_VIRT,
		.physical	= DAVINCI_IRAM_BASE,
		.length		= DAVINCI_IRAM_SIZE,
		.type		= MT_DEVICE,
	},
};
static struct map_desc dm646x_io_desc[] __initdata = {
	{
		.virtual	= DM646X_IO_VIRT,
		.physical	= DM646X_IO_PHYS,
		.length		= DM646X_IO_SIZE,
		.type		= MT_DEVICE,
	},
	{
		.virtual	= DAVINCI_IRAM_VIRT,
		.physical	= DM646X_IRAM_BASE,
		.length		= DAVINCI_IRAM_SIZE,
		.type		= MT_DEVICE,
	},
	{
		.virtual	= EMIF_CNTRL_VIRT,
		.physical	= DAVINCI_DM646X_ASYNC_EMIF_CNTRL_BASE,
		.length		= SZ_16K,
		.type		= MT_DEVICE,
	},
};

void __init davinci_map_common_io(void)
{
	iotable_init(davinci_io_desc, ARRAY_SIZE(davinci_io_desc));

	/* We want to check CPU revision early for cpu_is_davinci_xxxx() macros.
	 * IO space mapping must be initialized before we can do that.
	 */
	davinci_check_revision();

	if (cpu_is_davinci_dm644x()) {
		iotable_init(dm644x_io_desc, ARRAY_SIZE(dm644x_io_desc));
	} else if (cpu_is_davinci_dm6467()) {
		davinci_cpu_index = DM6467_CPU_IDX;
		iotable_init(dm646x_io_desc, ARRAY_SIZE(dm646x_io_desc));
	} else if (cpu_is_davinci_dm355()) {
		davinci_cpu_index = DM355_CPU_IDX;
	}

	/* Normally devicemaps_init() would flush caches and tlb after
	 * mdesc->map_io(), but we must also do it here because of the CPU
	 * revision check below.
	 */
	flush_tlb_all();
	flush_cache_all();


	davinci_mux_init();
	davinci_clk_init();
}
