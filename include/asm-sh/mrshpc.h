#ifndef __ASM_SH_MRSHPC_H
#define __ASM_SH_MRSHPC_H

/*
 * linux/include/asm-sh/mrshpc.h
 *
 * Copyright (C) 2004  Takashi Kusuda
 *
 *  MRSHPC Register list.
 *  Mostly copy from asm/se/se.h
 */

#if defined(CONFIG_CF_BASE_ADDR)
#define PA_MRSHPC_BASE	CONFIG_CF_BASE_ADDR
#else
#define PA_MRSHPC_BASE	P2SEGADDR(0x18000000)
#endif

#define PA_MRSHPC	(PA_MRSHPC_BASE + 0x003fffe0) /* MR-SHPC-01 PCMCIA controller */
#if defined(CONFIG_SH_SOLUTION_ENGINE_PLUS)
#define PA_MRSHPC_MW1	(PA_MRSHPC_BASE	+ 0x00000000) /* MR-SHPC-01 memory window base */
#define PA_MRSHPC_MW2	(PA_MRSHPC_BASE + 0x00100000) /* MR-SHPC-01 attribute window base */
#define PA_MRSHPC_IO	(PA_MRSHPC_BASE + 0x00200000) /* MR-SHPC-01 I/O window base */
#else
#define PA_MRSHPC_MW1	(PA_MRSHPC_BASE + 0x00400000) /* MR-SHPC-01 memory window base */
#define PA_MRSHPC_MW2	(PA_MRSHPC_BASE + 0x00500000) /* MR-SHPC-01 attribute window base */
#define PA_MRSHPC_IO	(PA_MRSHPC_BASE + 0x00600000) /* MR-SHPC-01 I/O window base */
#endif
#define MRSHPC_MODE	(PA_MRSHPC + 4)
#define MRSHPC_OPTION   (PA_MRSHPC + 6)
#define MRSHPC_CSR      (PA_MRSHPC + 8)
#define MRSHPC_ISR      (PA_MRSHPC + 10)
#define MRSHPC_ICR      (PA_MRSHPC + 12)
#define MRSHPC_CPWCR    (PA_MRSHPC + 14)
#define MRSHPC_MW0CR1   (PA_MRSHPC + 16)
#define MRSHPC_MW1CR1   (PA_MRSHPC + 18)
#define MRSHPC_IOWCR1   (PA_MRSHPC + 20)
#define MRSHPC_MW0CR2   (PA_MRSHPC + 22)
#define MRSHPC_MW1CR2   (PA_MRSHPC + 24)
#define MRSHPC_IOWCR2   (PA_MRSHPC + 26)
#define MRSHPC_CDCR     (PA_MRSHPC + 28)
#define MRSHPC_PCIC_INFO (PA_MRSHPC + 30)

#endif  /* __ASM_SH_MRSHPC_H */
