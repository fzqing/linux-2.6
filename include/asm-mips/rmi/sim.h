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


#ifndef _ASM_SIM_H
#define _ASM_SIM_H

#include <linux/types.h>
#include <asm/cpu.h>


#ifdef CONFIG_PHOENIX_PSB

#define RMI_PHOENIX_BOARD_ARIZONA_I   1
#define RMI_PHOENIX_BOARD_ARIZONA_II  2
#define RMI_PHOENIX_BOARD_ARIZONA_III 3


#define PSB_INFO_VERSION 0x0001

struct psb_info {
  uint64_t boot_level;
  uint64_t io_base;
  uint64_t output_device;
  uint64_t uart_print;
  uint64_t led_output;
  uint64_t init;
  uint64_t exit;
  uint64_t warm_reset;
  uint64_t wakeup;
  uint64_t cpu_online_map;
  uint64_t master_reentry_sp;
  uint64_t master_reentry_gp;
  uint64_t master_reentry_fn;
  uint64_t slave_reentry_fn;
  uint64_t magic_dword;
  uint64_t uart_putchar;
  uint64_t size;
  uint64_t uart_getchar;
  uint64_t nmi_handler;
  uint64_t psb_version;
  uint64_t mac_addr;
  uint64_t cpu_frequency;
  uint64_t board_version;
  uint64_t malloc;
  uint64_t free;
  uint64_t alloc_pbuf;
  uint64_t free_pbuf;
  uint64_t psb_os_cpu_map;
  uint64_t userapp_cpu_map;
  uint64_t wakeup_os;
  uint64_t psb_mem_map;
  uint64_t board_major_version;
  uint64_t board_minor_version;
  uint64_t board_manf_revision;
  uint64_t board_serial_number;
};

struct boot1_info {
  uint64_t boot_level;
  uint64_t io_base;
  uint64_t output_device;
  uint64_t uart_print;
  uint64_t led_output;
  uint64_t init;
  uint64_t exit;
  uint64_t warm_reset;
  uint64_t wakeup;
  uint64_t cpu_online_map;
  uint64_t master_reentry_sp;
  uint64_t master_reentry_gp;
  uint64_t master_reentry_fn;
  uint64_t slave_reentry_fn;
  uint64_t magic_dword;
  uint64_t uart_putchar;  
  uint64_t size;
  uint64_t uart_getchar;
  uint64_t nmi_handler;
  uint64_t psb_version;
  uint64_t mac_addr;
  uint64_t cpu_frequency;
  uint64_t board_version;
  uint64_t malloc;
  uint64_t free;
  uint64_t alloc_pbuf;
  uint64_t free_pbuf;
  uint64_t psb_os_cpu_map;
  uint64_t userapp_cpu_map;
  uint64_t wakeup_os;
  uint64_t psb_mem_map;
};

extern struct psb_info *prom_info;

struct smp_boot_info_percpu {
  volatile unsigned long ready;
  volatile unsigned long sp;
  volatile unsigned long gp;
  volatile unsigned long fn;
};

struct smp_boot_info {
  struct smp_boot_info_percpu boot_info[32];
  __u32 online_map;
};

#define PSB_MEM_MAP_MAX 32
struct psb_mem_map {
	int nr_map;
	struct psb_mem_map_entry {
		uint64_t addr;    /* start of memory segment */
		uint64_t size;    /* size of memory segment */
		uint32_t type;      /* type of memory segment */
	} map[PSB_MEM_MAP_MAX];
};

extern struct smp_boot_info smp_boot;
extern void prom_boot_cpus_secondary(void *);

static __inline__ int xlr_revision_a0(void)
{
	return (read_c0_prid() & 0xffffff) == PRID_COMP_RMI;
}

static __inline__ int xlr_board_atx_i(void)
{
	return prom_info->board_major_version == RMI_PHOENIX_BOARD_ARIZONA_I;
}

static __inline__ int xlr_board_atx_ii(void)
{
	return prom_info->board_major_version == RMI_PHOENIX_BOARD_ARIZONA_II;
}

static __inline__ int xlr_board_atx_iii(void)
{
	return prom_info->board_major_version == RMI_PHOENIX_BOARD_ARIZONA_III;
}

static __inline__ int xlr_board_atx_iii_256(void)
{
	return (prom_info->board_major_version == RMI_PHOENIX_BOARD_ARIZONA_III)
		&& (prom_info->board_minor_version == 0);
}

static __inline__ int xlr_board_atx_iii_512(void)
{
	return (prom_info->board_major_version == RMI_PHOENIX_BOARD_ARIZONA_III)
		&& (prom_info->board_minor_version == 1);
}
#define XLR_HYBRID_NONE              0
#define XLR_HYBRID_USER_MAC          1
#define XLR_HYBRID_RMIOS_IPSEC       2
#define XLR_HYBRID_USER_SPACE_KERNEL 3

extern int xlr_hybrid;

static __inline__ int xlr_hybrid_user_mac(void)
{
	return xlr_hybrid == XLR_HYBRID_USER_MAC;
}

static __inline__ int xlr_hybrid_none(void)
{
	return xlr_hybrid == XLR_HYBRID_NONE;
}


#else

/* structures and addresses shared with the RMI's 
 * Architectural Simulator
 */

/* Address of the simulator control structure */
#define SIMINFO_ADDR         0xFFFFFFFF9F400000

struct sim_control_struct {
  volatile char               putchar;
  volatile char               getchar;
  volatile char               uart_status;
  volatile char               getchar_cpunum;
  volatile unsigned long      time;
  volatile unsigned long long ipi;
  volatile char               putsocket;
  volatile char               getsocket;
  volatile char               socket_status;
  volatile char               socket_cpunum;
  volatile char               cache_disable;
  volatile char               shutdown;
};

extern volatile struct sim_control_struct   *siminfo;

#define BOOT_CPU_MAP_ADDR    0x9f501000
#define UART_RX_RDY 0x1
#define UART_ENABLE 0x80

static __inline__ void shutdown(void)
{
  siminfo->shutdown = 1;
}


struct smp_boot_info_struct {
  volatile __u32 ready;
  volatile __u32 sp;
  volatile __u32 gp;
  volatile __u32 fn;
};

struct smp_boot_info {
  struct smp_boot_info_struct start[64];
  unsigned long online_map;
};

extern volatile struct smp_boot_info_struct *smp_boot_info;

/* Addresses at which the Slave cpus read their sp, gp and 
 * start function
 */
#define BOOT_CPU_INFO_ADDR   0xFFFFFFFF9F500000

#endif



#endif
