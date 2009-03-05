#ifndef _ASM_RMI_RMIOS_USER_MAC_H
#define _ASM_RMI_RMIOS_USER_MAC_H

#define RMIOS_USER_MAC_BUCKET 4

#define RMIOS_USER_MAC_PERF_EVT_0            0
#define RMIOS_USER_MAC_PERF_EVT_1            1
#define RMIOS_USER_MAC_PERF_TOTAL_PKTS       2
#define RMIOS_USER_MAC_PERF_DROPPED_PKTS     3
#define RMIOS_USER_MAC_PERF_RX_TIMEOUT       4
#define RMIOS_USER_MAC_PERF_NUM_CTRS         5

#ifndef __ASSEMBLY__

#include <linux/types.h>

extern __u64 rmios_user_mac_perf_ctrs[32][RMIOS_USER_MAC_PERF_NUM_CTRS];

extern int rmios_user_mac_balance_num_flows;

#define RMIOS_USER_MAC_RX_TIMEOUT 672

extern __u32 rmios_user_mac_rx_timeout[32];

extern int rmios_user_mac_perf;
extern int rmios_user_mac_perf_q_thr;
extern int rmios_user_mac_perf_global;
extern int rmios_user_mac_perf_evt_0;
extern int rmios_user_mac_perf_evt_1;

extern void rmios_user_mac_init(void);
#endif

#endif /* _ASM_RMI_RMIOS_USER_MAC_H */
