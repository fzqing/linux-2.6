/*
 * Carsten Langgaard, carstenl@mips.com
 * Copyright (C) 1999,2000 MIPS Technologies, Inc.  All rights reserved.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 * PROM library initialisation code.
 */
#include <linux/config.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/kernel.h>

#include <asm/io.h>
#include <asm/bootinfo.h>
#include <asm/mips-boards/prom.h>
#include <asm/mips-boards/generic.h>
#ifdef CONFIG_MIPS_GT64120
#include <asm/gt64120.h>
#endif
#include <asm/mips-boards/msc01_pci.h>
#include <asm/mips-boards/bonito64.h>
#ifdef CONFIG_MIPS_MALTA
#include <asm/mips-boards/malta.h>
#endif

#ifdef CONFIG_MIPS_AVALANCHE_PSPBOOT
#include <asm/mach-avalanche/env.h>
#endif

int prom_argc;
int *_prom_argv, *_prom_envp;

/*
 * YAMON (32-bit PROM) pass arguments and environment as 32-bit pointer.
 * This macro take care of sign extension, if running in 64-bit mode.
 */
#define prom_envp(index) ((char *)(long)_prom_envp[(index)])

int init_debug = 0;

unsigned int mips_revision_corid;

/* Bonito64 system controller register base. */
unsigned long _pcictrl_bonito;
unsigned long _pcictrl_bonito_pcicfg;

/* GT64120 system controller register base */
unsigned long _pcictrl_gt64120;

/* MIPS System controller register base */
unsigned long _pcictrl_msc;

char *prom_getenv(char *envname)
{
#if defined (CONFIG_MIPS_AVALANCHE_PSPBOOT) /* PSP Boot related code. */
	int index;
        for (index = 0; index < sizeof(env_adam2_alias)/sizeof(env_adam2_alias[0]); index++) {
                if (strcmp(env_adam2_alias[index].nm, envname) == 0) {
                        return sys_getenv(env_adam2_alias[index].new_nm);
                }
        }
	return sys_getenv(envname);
#else		
	/*
	 * Return a pointer to the given environment variable.
	 * In 64-bit mode: we're using 64-bit pointers, but all pointers
	 * in the PROM structures are only 32-bit, so we need some
	 * workarounds, if we are running in 64-bit mode.
	 */
	int i, index=0;

	i = strlen(envname);

	while (prom_envp(index)) {
		if(strncmp(envname, prom_envp(index), i) == 0) {
			return(prom_envp(index+1));
		}
		index += 2;
	}

	return NULL;
#endif
}

static inline unsigned char str2hexnum(unsigned char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	return 0; /* foo */
}

static inline void str2eaddr(unsigned char *ea, unsigned char *str)
{
	int i;

	for (i = 0; i < 6; i++) {
		unsigned char num;

		if((*str == '.') || (*str == ':'))
			str++;
		num = str2hexnum(*str++) << 4;
		num |= (str2hexnum(*str++));
		ea[i] = num;
	}
}

int get_ethernet_addr(char *ethernet_addr)
{
        char *ethaddr_str;

        ethaddr_str = prom_getenv("ethaddr");
	if (!ethaddr_str) {
	        printk("ethaddr not set in boot prom\n");
		return -1;
	}
	str2eaddr(ethernet_addr, ethaddr_str);

	if (init_debug > 1) {
	        int i;
		printk("get_ethernet_addr: ");
	        for (i=0; i<5; i++)
		        printk("%02x:", (unsigned char)*(ethernet_addr+i));
		printk("%02x\n", *(ethernet_addr+i));
	}

	return 0;
}

#ifdef CONFIG_SERIAL_8250_CONSOLE
static void __init console_config(void)
{
	char console_string[40];
	int baud = 0;
	char parity = '\0', bits = '\0', flow = '\0';
	char *s;

	if ((strstr(prom_getcmdline(), "console=ttyS")) == NULL) {
		s = prom_getenv("modetty0");
		if (s) {
			while (*s >= '0' && *s <= '9')
				baud = baud*10 + *s++ - '0';
			if (*s == ',') s++;
			if (*s) parity = *s++;
			if (*s == ',') s++;
			if (*s) bits = *s++;
			if (*s == ',') s++;
			if (*s == 'h') flow = 'r';
		}
		if (baud == 0)
			baud = 38400;
		if (parity != 'n' && parity != 'o' && parity != 'e')
			parity = 'n';
		if (bits != '7' && bits != '8')
			bits = '8';
		if (flow == '\0')
			flow = 'r';
		sprintf (console_string, " console=ttyS0,%d%c%c%c", baud, parity, bits, flow);
		strcat (prom_getcmdline(), console_string);
		prom_printf("Config serial console:%s\n", console_string);
	}
}
#endif

void __init prom_init(void)
{
	prom_argc = fw_arg0;
	_prom_argv = (int *) fw_arg1;
	_prom_envp = (int *) fw_arg2;

#if defined(CONFIG_MIPS_AVALANCHE_PSPBOOT)
    sys_initenv();
#endif

#if !defined(CONFIG_MIPS_AVALANCHE_SOC)
	mips_display_message("LINUX");

#ifdef CONFIG_MIPS_SEAD
	set_io_port_base(KSEG1);
#else
	/*
	 * early setup of _pcictrl_bonito so that we can determine
	 * the system controller on a CORE_EMUL board
	 */
	_pcictrl_bonito = (unsigned long)ioremap(BONITO_REG_BASE, BONITO_REG_SIZE);

	mips_revision_corid = MIPS_REVISION_CORID;

	if (mips_revision_corid == MIPS_REVISION_CORID_CORE_EMUL) {
		if (BONITO_PCIDID == 0x0001df53 || 
		    BONITO_PCIDID == 0x0003df53)
			mips_revision_corid = MIPS_REVISION_CORID_CORE_EMUL_BON;
		else
			mips_revision_corid = MIPS_REVISION_CORID_CORE_EMUL_MSC;
	}
	switch(mips_revision_corid) {
	case MIPS_REVISION_CORID_QED_RM5261:
	case MIPS_REVISION_CORID_CORE_LV:
	case MIPS_REVISION_CORID_CORE_FPGA:
	case MIPS_REVISION_CORID_CORE_FPGAR2:
		/*
		 * Setup the North bridge to do Master byte-lane swapping
		 * when running in bigendian.
		 */
		_pcictrl_gt64120 = (unsigned long)ioremap(MIPS_GT_BASE, 0x2000);

#ifdef CONFIG_CPU_LITTLE_ENDIAN
		GT_WRITE(GT_PCI0_CMD_OFS, GT_PCI0_CMD_MBYTESWAP_BIT |
			 GT_PCI0_CMD_SBYTESWAP_BIT);
#else
		GT_WRITE(GT_PCI0_CMD_OFS, 0);
#endif

#ifdef CONFIG_MIPS_MALTA
		set_io_port_base(MALTA_GT_PORT_BASE);
#else
		set_io_port_base((unsigned long)ioremap(0, 0x20000000));
#endif
		break;

	case MIPS_REVISION_CORID_CORE_EMUL_BON:
	case MIPS_REVISION_CORID_BONITO64:
	case MIPS_REVISION_CORID_CORE_20K:
		_pcictrl_bonito_pcicfg = (unsigned long)ioremap(BONITO_PCICFG_BASE, BONITO_PCICFG_SIZE);

		/*
		 * Disable Bonito IOBC.
		 */
		BONITO_PCIMEMBASECFG = BONITO_PCIMEMBASECFG &
			~(BONITO_PCIMEMBASECFG_MEMBASE0_CACHED |
			  BONITO_PCIMEMBASECFG_MEMBASE1_CACHED);

		/*
		 * Setup the North bridge to do Master byte-lane swapping
		 * when running in bigendian.
		 */
#ifdef CONFIG_CPU_LITTLE_ENDIAN
		BONITO_BONGENCFG = BONITO_BONGENCFG &
			~(BONITO_BONGENCFG_MSTRBYTESWAP |
			  BONITO_BONGENCFG_BYTESWAP);
#else
		BONITO_BONGENCFG = BONITO_BONGENCFG |
			BONITO_BONGENCFG_MSTRBYTESWAP |
			BONITO_BONGENCFG_BYTESWAP;
#endif

#ifdef CONFIG_MIPS_MALTA
		set_io_port_base(MALTA_BONITO_PORT_BASE);
#else
		set_io_port_base((unsigned long)ioremap(0, 0x20000000));
#endif
		break;

	case MIPS_REVISION_CORID_CORE_MSC:
	case MIPS_REVISION_CORID_CORE_FPGA2:
	case MIPS_REVISION_CORID_CORE_EMUL_MSC:
		_pcictrl_msc = (unsigned long)ioremap(MIPS_MSC01_PCI_REG_BASE, 0x2000); 

#ifdef CONFIG_CPU_LITTLE_ENDIAN
		MSC_WRITE(MSC01_PCI_SWAP, MSC01_PCI_SWAP_NOSWAP);
#else
		MSC_WRITE(MSC01_PCI_SWAP,
			  MSC01_PCI_SWAP_BYTESWAP << MSC01_PCI_SWAP_IO_SHF |
			  MSC01_PCI_SWAP_BYTESWAP << MSC01_PCI_SWAP_MEM_SHF |
			  MSC01_PCI_SWAP_BYTESWAP << MSC01_PCI_SWAP_BAR0_SHF);
#endif

#ifdef CONFIG_MIPS_MALTA
		set_io_port_base(MALTA_MSC_PORT_BASE);
#else
		set_io_port_base((unsigned long)ioremap(0, 0x20000000));
#endif
		break;

	default:
		/* Unknown Core card */
		mips_display_message("CC Error");
		while(1);   /* We die here... */
	}
#endif
#endif

#if defined(CONFIG_MIPS_AVALANCHE_SOC)
    set_io_port_base(0);
	setup_prom_printf(0);
#endif /* CONFIG_MIPS_AVALANCHE_SOC */
	prom_printf("\nLINUX started...\n");
	prom_init_cmdline();
	prom_meminit();
#ifdef CONFIG_SERIAL_8250_CONSOLE
	console_config();
#endif
}
