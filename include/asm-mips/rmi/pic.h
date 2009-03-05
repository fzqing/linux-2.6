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

#ifndef _ASM_RMI_PIC_H
#define _ASM_RMI_PIC_H

#include <asm/rmi/iomap.h>

/**************************************************************/
#define PIC_IRT_WD_INDEX     0
#define PIC_IRT_TIMER_0_INDEX      1
#define PIC_IRT_TIMER_1_INDEX      2
#define PIC_IRT_TIMER_2_INDEX      3
#define PIC_IRT_TIMER_3_INDEX      4
#define PIC_IRT_TIMER_4_INDEX      5
#define PIC_IRT_TIMER_5_INDEX      6
#define PIC_IRT_TIMER_6_INDEX      7
#define PIC_IRT_TIMER_7_INDEX      8
#define PIC_IRT_CLOCK_INDEX        PIC_IRT_TIMER_7_INDEX
#define PIC_IRT_UART_0_INDEX       9
#define PIC_IRT_UART_1_INDEX       10
#define PIC_IRT_I2C_0_INDEX       11
#define PIC_IRT_I2C_1_INDEX       12
#define PIC_IRT_PCMCIA_INDEX           13
#define PIC_IRT_GPIO_INDEX             14
#define PIC_IRT_HYPER_INDEX            15
#define PIC_IRT_PCIX_INDEX             16
#define PIC_IRT_GMAC0_INDEX            17
#define PIC_IRT_GMAC1_INDEX            18
#define PIC_IRT_GMAC2_INDEX            19
#define PIC_IRT_GMAC3_INDEX            20
#define PIC_IRT_XGS0_INDEX             21
#define PIC_IRT_XGS1_INDEX             22
#define PIC_IRT_HYPER_FATAL_INDEX      23
#define PIC_IRT_PCIX_FATAL_INDEX       24
#define PIC_IRT_BRIDGE_AERR_INDEX      25
#define PIC_IRT_BRIDGE_BERR_INDEX     26
#define PIC_IRT_BRIDGE_TB_INDEX        27
#define PIC_IRT_BRIDGE_AERR_NMI_INDEX  28
#define PIC_NUM_IRTS                   32

#define PIC_SYS_TIMER_MAXVAL_0_BASE 0x100
#define PIC_SYS_TIMER_MAXVAL_1_BASE 0x110

#define PIC_SYS_TIMER_0_BASE 0x120
#define PIC_SYS_TIMER_1_BASE 0x130

#define PIC_CLOCK_TIMER 7

#define PIC_CTRL    0x00
#define PIC_IPI     0x04
#define PIC_INT_ACK 0x06

#define WD_MAX_VAL_0 0x08
#define WD_MAX_VAL_1 0x09
#define WD_MASK_0    0x0a
#define WD_MASK_1    0x0b
#define WD_HEARBEAT_0 0x0c
#define WD_HEARBEAT_1 0x0d

#define PIC_IRT_0_BASE 0x40
#define PIC_IRT_1_BASE 0x80

#define PIC_IRT_0_WD     (PIC_IRT_0_BASE   + PIC_IRT_WD_INDEX)
#define PIC_IRT_1_WD     (PIC_IRT_1_BASE   + PIC_IRT_WD_INDEX)
#define PIC_IRT_0_TIMER_0     (PIC_IRT_0_BASE   + PIC_IRT_TIMER_0_INDEX)
#define PIC_IRT_1_TIMER_0     (PIC_IRT_1_BASE   + PIC_IRT_TIMER_0_INDEX)
#define PIC_IRT_0_TIMER_1     (PIC_IRT_0_BASE   + PIC_IRT_TIMER_1_INDEX)
#define PIC_IRT_1_TIMER_1     (PIC_IRT_1_BASE   + PIC_IRT_TIMER_1_INDEX)
#define PIC_IRT_0_TIMER_2     (PIC_IRT_0_BASE   + PIC_IRT_TIMER_2_INDEX)
#define PIC_IRT_1_TIMER_2     (PIC_IRT_1_BASE   + PIC_IRT_TIMER_2_INDEX)
#define PIC_IRT_0_TIMER_3     (PIC_IRT_0_BASE   + PIC_IRT_TIMER_3_INDEX)
#define PIC_IRT_1_TIMER_3     (PIC_IRT_1_BASE   + PIC_IRT_TIMER_3_INDEX)
#define PIC_IRT_0_TIMER_4     (PIC_IRT_0_BASE   + PIC_IRT_TIMER_4_INDEX)
#define PIC_IRT_1_TIMER_4     (PIC_IRT_1_BASE   + PIC_IRT_TIMER_4_INDEX)
#define PIC_IRT_0_TIMER_5     (PIC_IRT_0_BASE   + PIC_IRT_TIMER_5_INDEX)
#define PIC_IRT_1_TIMER_5     (PIC_IRT_1_BASE   + PIC_IRT_TIMER_5_INDEX)
#define PIC_IRT_0_TIMER_6     (PIC_IRT_0_BASE   + PIC_IRT_TIMER_6_INDEX)
#define PIC_IRT_1_TIMER_6     (PIC_IRT_1_BASE   + PIC_IRT_TIMER_6_INDEX)
#define PIC_IRT_0_TIMER_7     (PIC_IRT_0_BASE   + PIC_IRT_TIMER_7_INDEX)
#define PIC_IRT_1_TIMER_7     (PIC_IRT_1_BASE   + PIC_IRT_TIMER_7_INDEX)
#define PIC_IRT_0_CLOCK       (PIC_IRT_0_TIMER_7)
#define PIC_IRT_1_CLOCK       (PIC_IRT_1_TIMER_7)
#define PIC_IRT_0_UART_0 (PIC_IRT_0_BASE + PIC_IRT_UART_0_INDEX)
#define PIC_IRT_1_UART_0 (PIC_IRT_1_BASE + PIC_IRT_UART_0_INDEX)
#define PIC_IRT_0_UART_1 (PIC_IRT_0_BASE + PIC_IRT_UART_1_INDEX)
#define PIC_IRT_1_UART_1 (PIC_IRT_1_BASE + PIC_IRT_UART_1_INDEX)
#define PIC_IRT_0_I2C_0 (PIC_IRT_0_BASE + PIC_IRT_I2C_0_INDEX)
#define PIC_IRT_1_I2C_0 (PIC_IRT_1_BASE + PIC_IRT_I2C_0_INDEX)
#define PIC_IRT_0_I2C_1 (PIC_IRT_0_BASE + PIC_IRT_I2C_1_INDEX)
#define PIC_IRT_1_I2C_1 (PIC_IRT_1_BASE + PIC_IRT_I2C_1_INDEX)

#define PIC_TIMER_0_MAXVAL_0   (PIC_SYS_TIMER_MAXVAL_0_BASE + 0)
#define PIC_TIMER_0_MAXVAL_1   (PIC_SYS_TIMER_MAXVAL_1_BASE + 0)
#define PIC_TIMER_0_COUNTER_0  (PIC_SYS_TIMER_0_BASE + 0)
#define PIC_TIMER_0_COUNTER_1  (PIC_SYS_TIMER_1_BASE + 0)
#define PIC_TIMER_7_MAXVAL_0   (PIC_SYS_TIMER_MAXVAL_0_BASE + 7)
#define PIC_TIMER_7_MAXVAL_1   (PIC_SYS_TIMER_MAXVAL_1_BASE + 7)
#define PIC_TIMER_7_COUNTER_0  (PIC_SYS_TIMER_0_BASE + 7)
#define PIC_TIMER_7_COUNTER_1  (PIC_SYS_TIMER_1_BASE + 7)

#define PIC_IRQ_BASE      8
#define PIC_IRT_FIRST_IRQ PIC_IRQ_BASE
#define PIC_WD_IRQ      (PIC_IRQ_BASE + PIC_IRT_WD_INDEX)
#define PIC_TIMER_0_IRQ (PIC_IRQ_BASE + PIC_IRT_TIMER_0_INDEX)
#define PIC_TIMER_1_IRQ (PIC_IRQ_BASE + PIC_IRT_TIMER_1_INDEX)
#define PIC_TIMER_2_IRQ (PIC_IRQ_BASE + PIC_IRT_TIMER_2_INDEX)
#define PIC_TIMER_3_IRQ (PIC_IRQ_BASE + PIC_IRT_TIMER_3_INDEX)
#define PIC_TIMER_4_IRQ (PIC_IRQ_BASE + PIC_IRT_TIMER_4_INDEX)
#define PIC_TIMER_5_IRQ (PIC_IRQ_BASE + PIC_IRT_TIMER_5_INDEX)
#define PIC_TIMER_6_IRQ (PIC_IRQ_BASE + PIC_IRT_TIMER_6_INDEX)
#define PIC_TIMER_7_IRQ (PIC_IRQ_BASE + PIC_IRT_TIMER_7_INDEX)
#define PIC_CLOCK_IRQ   (PIC_TIMER_7_IRQ)
#define PIC_UART_0_IRQ  (PIC_IRQ_BASE + PIC_IRT_UART_0_INDEX)
#define PIC_UART_1_IRQ  (PIC_IRQ_BASE + PIC_IRT_UART_1_INDEX)
#define PIC_I2C_0_IRQ   (PIC_IRQ_BASE + PIC_IRT_I2C_0_INDEX)
#define PIC_I2C_1_IRQ   (PIC_IRQ_BASE + PIC_IRT_I2C_1_INDEX)
#define PIC_PCMCIA_IRQ           (PIC_IRQ_BASE + PIC_IRT_PCMCIA_INDEX)
#define PIC_GPIO_IRQ             (PIC_IRQ_BASE + PIC_IRT_GPIO_INDEX)
#define PIC_HYPER_IRQ            (PIC_IRQ_BASE + PIC_IRT_HYPER_INDEX)
#define PIC_PCIX_IRQ             (PIC_IRQ_BASE + PIC_IRT_PCIX_INDEX)
#define PIC_GMAC_0_IRQ            (PIC_IRQ_BASE + PIC_IRT_GMAC0_INDEX)
#define PIC_GMAC_1_IRQ            (PIC_IRQ_BASE + PIC_IRT_GMAC1_INDEX)
#define PIC_GMAC_2_IRQ            (PIC_IRQ_BASE + PIC_IRT_GMAC2_INDEX)
#define PIC_GMAC_3_IRQ            (PIC_IRQ_BASE + PIC_IRT_GMAC3_INDEX)
#define PIC_XGS_0_IRQ             (PIC_IRQ_BASE + PIC_IRT_XGS0_INDEX)
#define PIC_XGS_1_IRQ             (PIC_IRQ_BASE + PIC_IRT_XGS1_INDEX)
#define PIC_HYPER_FATAL_IRQ      (PIC_IRQ_BASE + PIC_IRT_HYPER_FATAL_INDEX)
#define PIC_PCIX_FATAL_IRQ       (PIC_IRQ_BASE + PIC_IRT_PCIX_FATAL_INDEX)
#define PIC_BRIDGE_AERR_IRQ      (PIC_IRQ_BASE + PIC_IRT_BRIDGE_AERR_INDEX)
#define PIC_BRIDGE_BERR_IRQ      (PIC_IRQ_BASE + PIC_IRT_BRIDGE_BERR_INDEX)
#define PIC_BRIDGE_TB_IRQ        (PIC_IRQ_BASE + PIC_IRT_BRIDGE_TB_INDEX)
#define PIC_BRIDGE_AERR_NMI_IRQ  (PIC_IRQ_BASE + PIC_IRT_BRIDGE_AERR_NMI_INDEX)
#define PIC_IRT_LAST_IRQ  PIC_BRIDGE_AERR_NMI_IRQ

/***************************************************************/

/**************************************************************/
/***************************************************************/
#ifndef __ASSEMBLY__
static __inline__ void pic_send_ipi(__u32 ipi)
{
  phoenix_reg_t *mmio = phoenix_io_mmio(PHOENIX_IO_PIC_OFFSET);
  phoenix_write_reg(mmio, PIC_IPI, ipi);
}

static __inline__ __u32 pic_read_control(void)
{
  phoenix_reg_t *mmio = phoenix_io_mmio(PHOENIX_IO_PIC_OFFSET);
  return phoenix_read_reg(mmio, PIC_CTRL);
}

static __inline__ void pic_write_control(__u32 control)
{
  phoenix_reg_t *mmio = phoenix_io_mmio(PHOENIX_IO_PIC_OFFSET);
  phoenix_write_reg(mmio, PIC_CTRL, control);
}
static __inline__ void pic_update_control(__u32 control)
{
  phoenix_reg_t *mmio = phoenix_io_mmio(PHOENIX_IO_PIC_OFFSET);
  phoenix_write_reg(mmio, PIC_CTRL, (control | phoenix_read_reg(mmio, PIC_CTRL)));
}

#define PIC_IRQ_IS_EDGE_TRIGGERED(irq) \
		( ((irq)>=PIC_TIMER_0_IRQ) && ((irq)<=PIC_TIMER_7_IRQ) )

#define PIC_IRQ_IS_IRT(irq) \
		( ((irq)>=PIC_IRT_FIRST_IRQ) && ((irq)<=PIC_IRT_LAST_IRQ) )

#define PIC_PCMCIA_STATUS_1	0xffffffffBD0001f7
#define PIC_PCMCIA_STATUS_2	0xffffffffBEF19180

struct pt_regs;
extern void phoenix_ipi_handler(int irq, struct pt_regs *regs);
extern void phnx_msgring_int_handler(unsigned int irq, struct pt_regs *regs);

#endif

#endif
