#ifndef __ASM_CPU_SH4_RTC_H
#define __ASM_CPU_SH4_RTC_H

/* SH-4 RTC */
#if defined(CONFIG_CPU_SUBTYPE_SH7780)
#define SH_RTC_REG_BASE                0xFFE80000
#else
#define SH_RTC_REG_BASE                0xFFC80000
#endif

#define R64CNT         (SH_RTC_REG_BASE + 0)
#define RSECCNT                (SH_RTC_REG_BASE + 4)
#define RMINCNT                (SH_RTC_REG_BASE + 8)
#define RHRCNT         (SH_RTC_REG_BASE + 12)
#define RWKCNT         (SH_RTC_REG_BASE + 16)
#define RDAYCNT                (SH_RTC_REG_BASE + 20)
#define RMONCNT                (SH_RTC_REG_BASE + 24)
#define RYRCNT         (SH_RTC_REG_BASE + 28) /* 16bit */
#define RSECAR         (SH_RTC_REG_BASE + 32)
#define RMINAR         (SH_RTC_REG_BASE + 36)
#define RHRAR          (SH_RTC_REG_BASE + 40)
#define RWKAR          (SH_RTC_REG_BASE + 44)
#define RDAYAR         (SH_RTC_REG_BASE + 48)
#define RMONAR         (SH_RTC_REG_BASE + 52)
#define RCR1           (SH_RTC_REG_BASE + 56)
#define RCR2           (SH_RTC_REG_BASE + 60)

#define RTC_BIT_INVERTED	0x40	/* bug on SH7750, SH7750S */

#endif /* __ASM_CPU_SH4_RTC_H */

