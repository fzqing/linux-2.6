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
#ifndef _ASM_RMI_DEBUG_H
#define _ASM_RMI_DEBUG_H

extern void prom_printf(char *fmt, ...);
#include <linux/threads.h>
#include <asm/atomic.h>
enum {
  //cacheline 0
  MSGRNG_INT,
  MSGRNG_PIC_INT,
  MSGRNG_MSG,
  MSGRNG_EXIT_STATUS,
  //cacheline 1
  NETIF_TX = 8,
  NETIF_RX,
  NETIF_TX_COMPLETE,
  NETIF_TX_COMPLETE_TX,
  NETIF_RX_CYCLES,
  NETIF_TX_COMPLETE_CYCLES,
  NETIF_TX_CYCLES,
  NETIF_TIMER_START_Q,
  //NETIF_REG_FRIN,
  //NETIF_INT_REG,
  //cacheline 2
  REPLENISH_ENTER = 16,
  REPLENISH_ENTER_COUNT,
  REPLENISH_CPU,
  REPLENISH_FRIN,
  REPLENISH_CYCLES,
  NETIF_STACK_TX,
  NETIF_START_Q,
  NETIF_STOP_Q,
  //cacheline 3
  USER_MAC_START = 24,
  USER_MAC_INT   = 24,
  USER_MAC_TX_COMPLETE,
  USER_MAC_RX,
  USER_MAC_POLL,
  USER_MAC_TX,
  USER_MAC_TX_FAIL,
  USER_MAC_TX_COUNT,
  USER_MAC_FRIN,
  //cacheline 4
  USER_MAC_TX_FAIL_GMAC_CREDITS,
  USER_MAC_DO_PAGE_FAULT,
  USER_MAC_UPDATE_TLB,
  USER_MAC_UPDATE_BIGTLB,
  USER_MAC_UPDATE_TLB_PFN0,
  USER_MAC_UPDATE_TLB_PFN1,

  PHNX_MAX_COUNTERS = 40
};
extern atomic_t phnx_counters[NR_CPUS][PHNX_MAX_COUNTERS];
extern __u32 msgrng_msg_cycles;

#define phnx_inc_counter(x) atomic_inc(&phnx_counters[0][(x)])
#define phnx_dec_counter(x) atomic_dec(&phnx_counters[0][(x)])
#define phnx_set_counter(x, value) atomic_set(&phnx_counters[0][(x)], (value))
#define phnx_get_counter(x) atomic_read(&phnx_counters[0][(x)])

#define rmi_dbg_msg(fmt, args...) printk(fmt, ##args)

#define rmi_dbg_panic(fmt, args...) panic(fmt, ##args)

#define prom_dbg_msg(fmt, args...) prom_printf(fmt, ##args)

#endif
