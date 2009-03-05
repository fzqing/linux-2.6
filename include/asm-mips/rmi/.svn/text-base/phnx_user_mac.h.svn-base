#ifndef _ASM_RMI_PHNX_USER_MAC_H
#define _ASM_RMI_PHNX_USER_MAC_H

#define PHNX_USER_MAC_MMAP_VIRT_START 0x60000000
#define PHNX_USER_MAC_MMAP_PHYS_START 0x0b800000
#define PHNX_USER_MAC_MMAP_PHYS_END   0x0c000000

#ifndef __ASSEMBLY__
#include <asm/rmi/sim.h>

extern void phoenix_user_mac_update_time(void);

struct xlr_user_mac_config {
	int rmios;
	int xgmac;
	int mgmt_port;
	int l4_extract;
	int flow_balance;
};

extern struct xlr_user_mac_config xlr_user_mac;

static __inline__ int xlr_user_mac_rmios(void)
{
	return xlr_hybrid_user_mac() && (xlr_user_mac.rmios == 1);
}

static __inline__ int xlr_user_mac_xgmac(void)
{
	return xlr_hybrid_user_mac() && (xlr_user_mac.xgmac == 1);
}

static __inline__ int xlr_user_mac_mgmt_port(void)
{
	return xlr_user_mac.mgmt_port;
}

static __inline__ int xlr_user_mac_l4_extract(void)
{
	return xlr_hybrid_user_mac() ? xlr_user_mac.l4_extract  : 0;
}

static __inline__ int xlr_user_mac_flow_balance(void)
{
	return xlr_user_mac.flow_balance == 1;
}

#endif

#endif
