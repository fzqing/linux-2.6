/*
 *
 * Copyright © 2005 Raza Microelectronics, Inc. (.RMI.)
 *
 * This program is free software.  You may use it, redistribute it 
 * and/or modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation; either version two of the 
 * License or (at your option) any later version.
 *
 * This program is distributed in the hope that you will find it useful.  
 * Notwithstanding the foregoing, you understand and agree that this program 
 * is provided by RMI .as is,. and without any warranties, whether express, 
 * implied or statutory, including without limitation any implied warranty of 
 * non-infringement, merchantability or fitness for a particular purpose.  
 * In no event will RMI be liable for any loss of data, lost profits, cost 
 * of procurement of substitute technology or services or for any direct, 
 * indirect, incidental, consequential or special damages arising from the 
 * use of this program, however caused.  Your unconditional agreement to 
 * these terms and conditions is an express condition to, and shall be deemed 
 * to occur upon, your use, redistribution and/or modification of this program.
 *
 * See the GNU General Public License for more details.  
 */
/*
 * Setup code for the RMI's PTR board 
 */

#include <linux/config.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/bootmem.h>
#include <linux/init.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/bootinfo.h>
#include <asm/addrspace.h>
#include <asm/reboot.h>
#include <asm/time.h>
#include <linux/interrupt.h>
#include <asm/atomic.h>

#include <asm/bootinfo.h>
#include <asm/rmi/sim.h>
#include <asm/rmi/mips-exts.h>
#include <asm/rmi/iomap.h>
#include <asm/rmi/debug.h>
#include <asm/rmi/phnx_user_mac.h>
#include <asm/rmi/phnx_mmap.h>
#include <linux/serial.h>
#include <asm/serial.h>
#include <linux/serial_core.h>

__u8 phoenix_base_mac_addr[6];

void *phoenix_psb_shm = 0;
unsigned int phoenix_psb_shm_size = 0;

extern void phnx_msgring_config(void);
	
#ifndef CONFIG_PHOENIX_PSB
volatile struct sim_control_struct   *siminfo;
volatile struct smp_boot_info_struct *smp_boot_info;
#endif

#ifdef CONFIG_PHOENIX_PSB
struct psb_info psb_prom_info; /* Bootloader prom_info is saved here */
struct psb_info *prom_info = 0;
static struct psb_info default_prom_info = {
	.boot_level              = 2,
	.io_base                 = DEFAULT_PHOENIX_IO_BASE,
	.output_device           = 2,
	.cpu_online_map          = 0x01,
	.magic_dword             = (((__u64)0x900dbeef << 32)|PSB_INFO_VERSION),
	.size                    = sizeof(struct psb_info),
	.mac_addr                = 0x000102030405,
	.cpu_frequency           = 1200000000  
};
static int sanity_check_prom_info(struct psb_info *info)
{
	if (!prom_info) return 0;
  
	if ((prom_info->magic_dword & 0xffffffffULL) != 0x900dbeef) return 0;
	if ((prom_info->magic_dword >> 32) != PSB_INFO_VERSION) return 0;
	if (prom_info->size < sizeof(struct psb_info)) return 0;

	return 1;
}
#endif

const char *DEFAULT_BOOT_PARAMS = 

#ifdef CONFIG_PHOENIX_RMIOS
"mem=176m@16m "
#else
//"mem=380m@16m "
#endif
"console=ttyS0,38400";
#ifdef CONFIG_PHOENIX_PSB

#ifdef CONFIG_PHOENIX_RMIOS
#define DEFAULT_RAM_START ((unsigned long)4<<20)
#define DEFAULT_RAM_SIZE  ((unsigned long)316<<20)
#define PSB_START ((unsigned long)320<<20)
#define PSB_SIZE ((unsigned long)64<<20)
#else
#define DEFAULT_RAM_SIZE  ((unsigned long)192<<20)
#define DEFAULT_RAM_START ((unsigned long)1<<20)
#endif

#else

#define DEFAULT_RAM_SIZE  ((unsigned long)128<<20)
#define DEFAULT_RAM_START ((unsigned long)1<<20)

#endif

const char *get_system_type(void)
{
#ifdef CONFIG_PHOENIX_RTL
	return "RMI Phoenix RTL";
#else
	return "RMI Phoenix Test Rig";
#endif
}

extern void prom_reenter(unsigned long sp, unsigned long gp, unsigned long fn);

#ifdef CONFIG_SMP
atomic_t cpus_rebooted = ATOMIC_INIT(0);
#endif

#define GPIO_SWRESET_REG 8
static void ptr_linux_exit(void)
{
	phoenix_reg_t *mmio = phoenix_io_mmio(PHOENIX_IO_GPIO_OFFSET);

	/* trigger a chip reset */
	phoenix_write_reg(mmio, GPIO_SWRESET_REG, 1);
	for(;;) cpu_wait();
}

void __init bus_error_init(void)
{
}

int xlr_hybrid;

static void xlr_initialize_setups(void)
{
	xlr_hybrid = XLR_HYBRID_NONE;
	xlr_user_mac.rmios = 1;
	xlr_user_mac.xgmac = 0;
	xlr_user_mac.mgmt_port = -1;
	xlr_user_mac.l4_extract = 0;
	xlr_user_mac.flow_balance = 0;
}

static int __init xlr_hybrid_setup(char *str)
{
	if (strcmp(str, "=user_mac") == 0) {
#ifdef CONFIG_PHOENIX_USER_MAC 
		xlr_hybrid = XLR_HYBRID_USER_MAC;
		printk("Configured for Hybrid mode with USER_MAC\n");
		phnx_msgring_config();
#endif
	}
	else if (strcmp(str, "=rmios_ipsec") == 0) {
		xlr_hybrid = XLR_HYBRID_RMIOS_IPSEC;
		printk("Configured for Hybrid mode with RMIOS IPSEC\n");
	}
	else if (strcmp(str, "=user_space_kernel") == 0) {
		xlr_hybrid = XLR_HYBRID_USER_SPACE_KERNEL;
		printk("Configured for Hybrid mode with USER_SPACE_KERNEL\n");
	}
	else {		
		xlr_hybrid = XLR_HYBRID_NONE;
		printk("Configured for Hybrid mode with None\n");
	}

	return 1;
}

__setup("xlr_hybrid", xlr_hybrid_setup);



void ptr_time_init(void)
{
#ifdef CONFIG_PHOENIX_PSB
	/* only needed for use cpu counter timer interrupt source */
	mips_hpt_frequency = prom_info->cpu_frequency;
#else
	mips_hpt_frequency = 1.2 * 1000 * 1000;
#endif

  
	printk("mips_hpt_frequency = %u\n", mips_hpt_frequency);
}

extern void phoenix_timer_setup(void);
void __init ptr_timer_setup(struct irqaction *irq)
{
	/*
	 *	a) (optional) over-write any choices made above by time_init().
	 *	b) machine specific code should setup the timer irqaction.
	 *	c) enable the timer interrupt
   
	 * Even if a machine chooses to use a low-level timer interrupt,
	 * it still needs to setup the timer_irqaction.
	 * In that case, it might be better to set timer_irqaction.handler
	 * to be NULL function so that we are sure the high-level code
	 * is not invoked accidentally.
	 */
#ifndef CONFIG_CPU_TIMER
	phoenix_timer_setup();
#endif
}

static int __init ptr_console_setup(void) 
{
	struct uart_port up;

	memset(&up, 0, sizeof(up));
	up.membase	= (u8*)(DEFAULT_PHOENIX_IO_BASE + 
				PHOENIX_IO_UART_0_OFFSET);
	up.irq		= PIC_UART_0_IRQ;
	up.regshift	= 2;
	up.iotype	= UPIO_MEM;
	up.uartclk	= (66000000);
	up.flags	= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST;
	up.line		= 0;

	if (early_serial_setup(&up))
		printk(KERN_ERR "Early serial init of UART port 0 failed\n");

	return 0;
}

//static int __init ptr_setup(void)
static int __init rmi_setup(void)
{
	extern int panic_timeout;
  
	panic_timeout = 5;  
  
	board_time_init = ptr_time_init;
	board_timer_setup = ptr_timer_setup;
  
	_machine_restart   = (void (*)(char *))ptr_linux_exit;
	_machine_halt      = ptr_linux_exit;
	_machine_power_off = ptr_linux_exit;

	return;
}
early_initcall(rmi_setup);

struct boot_mem_map_exclude_region {
	unsigned long long start;
	unsigned long long end;
};

/* The below regions should be in ascending order of the starting physical addresses */
static struct boot_mem_map_exclude_region boot_mem_map_exclude_regions[] = {
	[0] = { PHNX_USER_MAC_MMAP_PHYS_START, PHNX_USER_MAC_MMAP_PHYS_END},
	[1] = { PHNX_MMAP_PHYS_START, PHNX_MMAP_PHYS_START+PHNX_MMAP_PHYS_SIZE}
};

#ifdef CONFIG_PHOENIX_USER_MAC
#define BOOT_MEM_MAP_NUM_EXCLUDE_REGIONS (sizeof(boot_mem_map_exclude_regions)/sizeof(struct boot_mem_map_exclude_region))
#else
#define BOOT_MEM_MAP_NUM_EXCLUDE_REGIONS 0
#endif

void read_boot_memp_map(void)
{
	struct psb_mem_map *map;
	int i,j;
	int use_default=1;	
        __u64 start=0, end=0, exc_start=0, exc_end=0;
		
	map = (struct psb_mem_map *)((unsigned long)(prom_info->psb_mem_map));

  if(map->map[0].size==0x0c000000) {
    map->map[0].size=0x0ff00000;
  }

	for(i=0; i < map->nr_map; i++) {
		start = map->map[i].addr;
		end = map->map[i].addr + map->map[i].size;

		for(j=0; (j<BOOT_MEM_MAP_NUM_EXCLUDE_REGIONS) ; j++){
			exc_start = boot_mem_map_exclude_regions[j].start;
			exc_end = boot_mem_map_exclude_regions[j].end;

			if(exc_start>=start && exc_start<end){
				if(map->map[i].type == BOOT_MEM_RAM){
					add_memory_region(start,exc_start-start, map->map[i].type);
					prom_dbg_msg("Adding %#llx @ %#llx\n",exc_start-start,start);
					use_default = 0;
					start = exc_end;
				}
			}
		}
		if(start!=end){
			if(map->map[i].type == BOOT_MEM_RAM){
				add_memory_region(start, end-start, map->map[i].type);
				use_default = 0;
			}
		}
		prom_dbg_msg("Adding %#llx @ %#llx\n", end-start, start);
	}
	
	if(use_default) {
		prom_dbg_msg("\nUsing default memory map\n");
		add_memory_region(DEFAULT_RAM_START, DEFAULT_RAM_SIZE,
					BOOT_MEM_RAM);
	}
		
}

extern void prom_boot_cpus_secondary(void *);
/*
 * prom_init is called just after the cpu type is determined, from setup_arch()
 */
void __init prom_init(void)
{
	mips_machgroup = MACH_GROUP_RMI;

	xlr_initialize_setups();
	strcat(arcs_cmdline, DEFAULT_BOOT_PARAMS);
	strcat(arcs_cmdline, " ");

	{
	/* default mac addr */
	printk("Using default Mac address\n");
	phoenix_base_mac_addr[0] = 0x00;
	phoenix_base_mac_addr[1] = 0x01;
	phoenix_base_mac_addr[2] = 0x02;
	phoenix_base_mac_addr[3] = 0x03;
	phoenix_base_mac_addr[4] = 0x04;
	phoenix_base_mac_addr[5] = 0x05;

	}

#if !defined(CONFIG_PHOENIX_PSB)
	/* Initialize the sim control structure */
	siminfo = (volatile struct sim_control_struct *)(SIMINFO_ADDR);
	smp_boot_info = 
		(volatile struct smp_boot_info_struct *)(BOOT_CPU_INFO_ADDR);
#else
	{
	int i=0;
	int argc = (int)fw_arg0;
	long temp;
	char **argv;
	char **envp;
	int t_argc = argc;
	char *n_argv[64];
	struct psb_info *p_info;

	temp = (int)fw_arg1;
	argv = (char **)temp;
	temp = (int)fw_arg2;
	envp = (char **)temp;

	temp = (int)fw_arg3;
	p_info = (struct psb_info *)temp;
	prom_info = &psb_prom_info;

	memcpy((void *)prom_info, (void *)p_info, sizeof(struct psb_info));

	prom_dbg_msg("argv = %llx envp = %llx prom_info=%llx\n", argv, envp,
			prom_info);



	if (!sanity_check_prom_info(prom_info)) {
		printk("Sanity Check failed on prom_info @ %p\n", prom_info);
		if (prom_info) {
			printk("sizeof(psb_info) = %d, psb_info_version = %x, "
	       			"magic_dword = %llx, prom_info->size = %llx\n", 
	       			(unsigned int)sizeof(struct psb_info), 
				(unsigned int)PSB_INFO_VERSION, 
	       			(unsigned long long)prom_info->magic_dword, 
				(unsigned long long)prom_info->size);
		}
		prom_info = &default_prom_info;
		goto parse_args;
	}
	/* Get the right 64bit pointers from bootloader args */
	prom_dbg_msg("Passed argv is %lx envp is %lx\n", argv, envp);
	{
		int32_t *t_argv;

		t_argv = (int32_t *)argv;
		for(i=0; i < t_argc; i++, t_argv++) {
			n_argv[i] = (char *)(unsigned long)(*t_argv);
			//prom_dbg_msg("Passed argv[%d] is %p\n", i, argv[i]);
			//prom_dbg_msg("New argv[%d] is %p\n", i, n_argv[i]);
		}
	}
	for(i=0; i < t_argc; i++)
		argv[i] = n_argv[i];
	argc = t_argc;

	/* update the phoenix mac addr */
	{
	phoenix_base_mac_addr[0] = (prom_info->mac_addr >> 40) & 0xff;
	phoenix_base_mac_addr[1] = (prom_info->mac_addr >> 32) & 0xff;
	phoenix_base_mac_addr[2] = (prom_info->mac_addr >> 24) & 0xff;
	phoenix_base_mac_addr[3] = (prom_info->mac_addr >> 16) & 0xff;
	phoenix_base_mac_addr[4] = (prom_info->mac_addr >> 8) & 0xff;
	phoenix_base_mac_addr[5] = (prom_info->mac_addr >> 0) & 0xff;
	}    
	{
	void (*wakeup)(void *, void *, __u32) = 
		((void (*)(void *, void *, __u32))
		 (unsigned long)(prom_info->wakeup));
	smp_boot.online_map = (1 << hard_smp_processor_id());
	wakeup(prom_boot_cpus_secondary, 0, 
			(__u32)prom_info->cpu_online_map);
	}
	phoenix_psb_shm = (void *)phys_to_virt((unsigned long)PHNX_USER_MAC_MMAP_PHYS_START);
	phoenix_psb_shm_size = PHNX_USER_MAC_MMAP_PHYS_END - PHNX_USER_MAC_MMAP_PHYS_START;

	parse_args:
		prom_dbg_msg("argc=%d, argv=%lx, envp=%lx, prom_info=%lx\n", 
		 argc, (unsigned long)argv, (unsigned long)envp, 
		 (unsigned long)prom_info);

	for(i=1;i<argc;i++) {
		if (argv && argv[i]) {
			strcat(arcs_cmdline, argv[i]);
			strcat(arcs_cmdline, " ");
		}
		else {
			prom_dbg_msg("bad args, i=%d\n", i);
		}
	}
	  
	prom_dbg_msg("arcs_cmdline=[%s]\n", arcs_cmdline);
	}
#endif

	printk("MAC ADDR BASE: %02x:%02x:%02x:%02x:%02x:%02x\n",
	 phoenix_base_mac_addr[0], phoenix_base_mac_addr[1], 
	 phoenix_base_mac_addr[2], phoenix_base_mac_addr[3], 
	 phoenix_base_mac_addr[4], phoenix_base_mac_addr[5]);

	prom_dbg_msg("Master CPU Thread: %d of %d running on Phoenix %d\n",
	       phoenix_thr_id(), phoenix_cpu_id(), phoenix_id());

	read_boot_memp_map();

	on_chip_init();
	prom_dbg_msg("on_chip init done\n");
}

void prom_free_prom_memory(void)
{
	/* nothing to free */
}

void read_cp0_regs(void)
{
	printk("[%s]: count = 0x%x, compare = 0x%x\n"
	 "status = 0x%x, cause = 0x%x\n"
	 "eimr = 0x%llx, eirr = 0x%llx\n",
	 __FUNCTION__, 
	 read_c0_count(),
	 read_c0_compare(),
	 read_c0_status(),
	 read_c0_cause(),
	 (unsigned long long)read_64bit_cp0_eimr(),
	 (unsigned long long)read_64bit_cp0_eirr()
	 );
}

void prom_putchar(char ch)
{
#ifdef CONFIG_PHOENIX_PSB
	if (prom_info) 
		((void (*)(char))(unsigned long)prom_info->uart_putchar)(ch);
#else
	if (siminfo) siminfo->putchar = ch;
#endif
}
