/*
 * arch/mips/tx4939/common/prom.c
 *
 * common tx4939 memory interface
 *
 * (C) Copyright TOSHIBA CORPORATION SEMICONDUCTOR COMPANY 2000-2001,2005
 *
 * Author: source@mvista.com
 *
 * 2001-2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Support for TX4939 in 2.6 - Hiroshi DOYU <Hiroshi_DOYU@montavista.co.jp>
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/bootmem.h>

#include <asm/addrspace.h>
#include <asm/bootinfo.h>
#include <asm/tx4939/tx4939.h>

#ifndef COMMAND_LINE_SIZE
#define COMMAND_LINE_SIZE CL_SIZE
#endif

extern void sio_setup(void);
extern void fputs(unsigned char *cp);

static int *_prom_envp;

/*
 * YAMON (32-bit PROM) pass arguments and environment as 32-bit pointer.
 * This macro take care of sign extension, if running in 64-bit mode.
 */
#define prom_envp(index) ((char *)(((int *)(int)_prom_envp)[(index)]))

char *prom_getenv(char *envname)
{
	/*
	 * Return a pointer to the given environment variable.
	 * In 64-bit mode: we're using 64-bit pointers, but all pointers
	 * in the PROM structures are only 32-bit, so we need some
	 * workarounds, if we are running in 64-bit mode.
	 */
	int i, index = 0;

	i = strlen(envname);

	while (prom_envp(index)) {
		if (strncmp(envname, prom_envp(index), i) == 0)
			if (strlen(prom_envp(index)) == i)
				return (prom_envp(index + 1));
		index += 2;
	}

	return NULL;
}

void __init prom_init_cmdline(void)
{
	int argc = (int)fw_arg0;
	char **argv = (char **)fw_arg1;
	int i;			/* Always ignore the "-c" at argv[0] */

	/* ignore all built-in args if any f/w args given */
	if (argc > 1)
		*arcs_cmdline = '\0';

	for (i = 1; i < argc; i++) {
		if (i != 1)
			strcat(arcs_cmdline, " ");
		strcat(arcs_cmdline, argv[i]);
	}
}

/**
 * tx4939_get_mem_size - get memory size on tx4939 board
 *
 * This function gives memory size which can use by kernel.
 * This size defines in include/asm-mips/tx4939/config.h
 */

unsigned int __init tx4939_get_mem_size(void)
{
	unsigned int total;

	total = CONFIG_TOSHIBA_TX4939_MEMSIZE;

	return total;
}

/**
 * prom_init - the initialize routine related to PROM
 * @argc: The count of arguments
 * @argv: The pointer list of arguments
 * @envp: The pointer list of pmon environment value
 * *pvec: The pointer list of pmon vector
 *
 * This function sets mipsmarchgroup/type. and sets memory region
 */

void __init prom_init(void)
{
	int msize;

#ifndef CONFIG_TX4939_NAND_BOOT
	prom_init_cmdline();
#endif
	mips_machgroup = MACH_GROUP_TOSHIBA;
	mips_machtype = MACH_TOSHIBA_RBTX4939;

	sio_setup();
	fputs("\nLINUX started...\r\n");

	msize = tx4939_get_mem_size();
	add_memory_region(0, msize << 20, BOOT_MEM_RAM);
}

void __init prom_free_prom_memory(void)
{
	return;
}

void __init prom_fixup_mem_map(unsigned long start, unsigned long end)
{
	return;
}

const char *get_system_type(void)
{
	return "Toshiba RBTX4939";
}

char *__init prom_getcmdline(void)
{
	return &(arcs_cmdline[0]);
}

