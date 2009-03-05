#ifndef _AVALANCHE_INTC_H
#define _AVALANCHE_INTC_H

#define MIPS_EXCEPTION_OFFSET 8

#include <asm/mach-avalanche/pal.h>

#define AVINTNUM(x) ((x) - MIPS_EXCEPTION_OFFSET)
#define LNXINTNUM(x)((x) + MIPS_EXCEPTION_OFFSET)

#define AVALANCHE_INT_END_PRIMARY      (40 + MIPS_EXCEPTION_OFFSET)
#define AVALANCHE_INT_END_SECONDARY    (32 + AVALANCHE_INT_END_PRIMARY)

#define AVALANCHE_INT_END_PRIMARY_REG1 32
#define AVALANCHE_INT_END_PRIMARY_REG2 40

#define AVALANCHE_INTC_END AVALANCHE_INT_END_SECONDARY
#define AVALANCHE_INT_END             AVALANCHE_INTC_END
#define AVALANCHE_ICTRL_REGS_BASE  AVALANCHE_INTC_BASE
#define AVALANCHE_ECTRL_REGS_BASE  (AVALANCHE_ICTRL_REGS_BASE + 0x80)
#define AVALANCHE_IPACE_REGS_BASE  (AVALANCHE_ICTRL_REGS_BASE + 0xA0)
#define AVALANCHE_CHCTRL_REGS_BASE (AVALANCHE_ICTRL_REGS_BASE + 0x200)

struct avalanche_ictrl_regs 
{
  volatile unsigned long intsr1;    /* Interrupt Status/Set Register 1   0x00 */
  volatile unsigned long intsr2;    /* Interrupt Status/Set Register 2   0x04 */
  volatile unsigned long unused1;                                      /*0x08 */
  volatile unsigned long unused2;                                      /*0x0C */
  volatile unsigned long intcr1;    /* Interrupt Clear Register 1        0x10 */
  volatile unsigned long intcr2;    /* Interrupt Clear Register 2        0x14 */
  volatile unsigned long unused3;                                      /*0x18 */
  volatile unsigned long unused4;                                      /*0x1C */
  volatile unsigned long intesr1;   /* Interrupt Enable (Set) Register 1 0x20 */
  volatile unsigned long intesr2;   /* Interrupt Enable (Set) Register 2 0x24 */
  volatile unsigned long unused5;                                      /*0x28 */
  volatile unsigned long unused6;                                      /*0x2C */
  volatile unsigned long intecr1;   /* Interrupt Enable Clear Register 1 0x30 */
  volatile unsigned long intecr2;   /* Interrupt Enable Clear Register 2 0x34 */
  volatile unsigned long unused7;                                     /* 0x38 */
  volatile unsigned long unused8;                                     /* 0x3c */
  volatile unsigned long pintir;    /* Priority Interrupt Index Register 0x40 */
  volatile unsigned long intmsr;    /* Priority Interrupt Mask Index Reg 0x44 */
  volatile unsigned long unused9;                                     /* 0x48 */
  volatile unsigned long unused10;                                    /* 0x4C */
  volatile unsigned long intpolr1;  /* Interrupt Polarity Mask register 10x50 */
  volatile unsigned long intpolr2;  /* Interrupt Polarity Mask register 20x54 */
  volatile unsigned long unused11;                                    /* 0x58 */
  volatile unsigned long unused12;                                   /*0x5C */
  volatile unsigned long inttypr1;  /* Interrupt Type     Mask register 10x60 */
  volatile unsigned long inttypr2;  /* Interrupt Type     Mask register 20x64 */
};

struct avalanche_exctrl_regs   /* Avalanche Exception control registers */
{
  volatile unsigned long exsr;      /* Exceptions Status/Set register    0x80 */
  volatile unsigned long reserved;                                     /*0x84 */
  volatile unsigned long excr;      /* Exceptions Clear Register         0x88 */
  volatile unsigned long reserved1;                                    /*0x8c */
  volatile unsigned long exiesr;    /* Exceptions Interrupt Enable (set) 0x90 */
  volatile unsigned long reserved2;                                    /*0x94 */
  volatile unsigned long exiecr;    /* Exceptions Interrupt Enable(clear)0x98 */
};
struct avalanche_ipace_regs
{

  volatile unsigned long ipacep;    /* Interrupt pacing register         0xa0 */
  volatile unsigned long ipacemap;  /*Interrupt Pacing Map Register      0xa4 */
  volatile unsigned long ipacemax;  /*Interrupt Pacing Max Register      0xa8 */
};
struct avalanche_channel_int_number
{
  volatile unsigned long cintnr[40];   /* Channel Interrupt Number Registers */
};

struct avalanche_interrupt_line_to_channel
{
  unsigned long int_line[40];    /* Start of primary interrupts */
};

extern void avalanche_int_set(int channel, int line);
int avalanche_intr_polarity_set(unsigned int irq_nr, unsigned long type_val);
int avalanche_intr_polarity_get(unsigned int irq_nr);

#endif /* _AVALANCHE_INTC_H */
