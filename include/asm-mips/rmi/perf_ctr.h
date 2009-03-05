#ifndef __ASM_RMI_PERF_CTR_H
#define __ASM_RMI_PERF_CTR_H

#include <asm/mipsregs.h>

#define CP0_PERF_CTR  $25

/* Subset of perf ctr events */

#define PERF_CTR_INSTR_FETCHED           0
#define PERF_CTR_ICACHE_MISSES           1
#define PERF_CTR_SLEEP_CYCLES           12
#define PERF_CTR_INSTR_RETIRED          17
#define PERF_CTR_BRJMP_INSTR            20
#define PERF_CTR_BRJMP_FLUSH            21
#define PERF_CTR_REPLAYFLUSH            27
#define PERF_CTR_REPLAYFLUSH_LDUSE      28
#define PERF_CTR_L1_HIT                 38
#define PERF_CTR_L1_REF                 39
#define PERF_CTR_SNOOP_UPGRADE_FAIL     47
#define PERF_CTR_SNOOP_TRANSFERS        48
#define PERF_CTR_SNOOP_HITS             49
#define PERF_CTR_SNOOP_OPS              50
#define PERF_CTR_CYCLES                 63

/* 2 sets of counters are supported across all threads of a core */
#define PERF_CTR_EVENT0        0
#define PERF_CTR_EVENT0_VALUE  1
#define PERF_CTR_EVENT1        2
#define PERF_CTR_EVENT1_VALUE  3

#define PERF_CTR_DEFAULT 0x0f /* disable int, enable counting in all modes */

#define perf_ctr_start(ctr, event, global, thr) __write_32bit_c0_register($25, ctr, ((PERF_CTR_DEFAULT)|((global)<<13)|((thr)<<11)|((event)<<5)) ) 

#define perf_ctr_stop(ctr) __write_32bit_c0_register($25, ctr, 0)

#define perf_ctr_reset(ctr) __write_32bit_c0_register($25, ctr, 0)

#define perf_ctr_read(ctr) __read_32bit_c0_register($25, ctr)

#endif /* __ASM_RMI_PERF_CTR_H */
