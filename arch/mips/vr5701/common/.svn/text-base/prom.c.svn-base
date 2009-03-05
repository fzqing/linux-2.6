/*
 * arch/mips/vr5701/common/prom.c
 *
 * A code for prom routines on NEC Electronics Corporation VR5701 SolutionGearII 
 *
 * Author: Sergey Podstavin <spodstavin@ru.mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/config.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/bootmem.h>
#include <asm/addrspace.h>
#include <asm/bootinfo.h>
#include <asm/debug.h>
#include <asm/vr5701/vr5701_sg2.h>

const char *get_system_type(void)
{
	return "NEC Electronics Corporation VR5701 SolutionGearII";
}

void __init prom_init(void)
{
	int i;
	int argc = fw_arg0;
	char **arg = (char **)fw_arg1;

	/* if user passes kernel args, ignore the default one */
	if (argc > 1)
		arcs_cmdline[0] = '\0';

	/* arg[0] is "g", the rest is boot parameters */
	for (i = 1; i < argc; i++) {
		if (strlen(arcs_cmdline) + strlen(arg[i] + 1)
		    >= sizeof(arcs_cmdline))
			break;
		strcat(arcs_cmdline, arg[i]);
		strcat(arcs_cmdline, " ");
	}
	mips_machgroup = MACH_GROUP_VR5701;
	mips_machtype = MACH_VR5701_SG2;
	add_memory_region(0, VR5701_SG2_SDRAM_SIZE, BOOT_MEM_RAM);
}

void __init prom_free_prom_memory(void)
{
}
