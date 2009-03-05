/*
 * include/asm-mips/tx4939/rbtx4939.h
 *
 * Definitions for RBTX4939
 *
 * (C) Copyright TOSHIBA CORPORATION SEMICONDUCTOR COMPANY 2000-2001,2005
 *
 * Author: source@mvista.com
 *
 * 2001-2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Support for TX4939 in 2.6 - Hiroshi DOYU <Hiroshi_DOYU@montavista.co.jp>
 */

#ifndef __ASM_TX_BOARDS_RBTX4939_H
#define __ASM_TX_BOARDS_RBTX4939_H

#include <asm/tx4939/tx4939.h>
#include <asm/addrspace.h>

#ifndef __ASSEMBLY__
extern int rbtx4939_irq_nested(void);
extern void rbtx4939_irq_init(void);
extern void tx4939_pci_setup_irq(void);

extern void rbtx4939_led(unsigned int led, unsigned int hex);
#endif                         /* __ASSEMBLY__ */

/* CS */
#define RBTX4939_CS0                 0x1c000000
#define RBTX4939_CS1                 0x17f00000	/* I/O Controller etc. */
#define RBTX4939_CS2                 0x18000000
#define RBTX4939_CS3                 0x16000000	/* LOCAL or ISA */

/* RBTX4939 REGISTER ADDRESS MAP */
#define RBTX4939_IOC_REG             (KSEG1 + RBTX4939_CS1)
#define RBTX4939_IOC_SIZE            0x10000
#define RBTX4939_IR_REG              (RBTX4939_IOC_REG + 0x00000)
#define RBTX4939_ETHER_REG           (RBTX4939_IOC_REG + 0x20000)
#define RBTX4939_DBOARD_REG          (RBTX4939_IOC_REG + 0x30000)

/* Debug Ethernet port address */
#define RBTX4939_DEBUG_ETHER_BASE       (0x300+RBTX4939_ETHER_REG)
#define RBTX4939_ETHER_MAC_ADDR_BASE    0x1fff0000

/* IRQ NUMBUR */
#define RBTX4939_IRQ_IOC             TX4939_IRQ_INT(0)
#define RBTX4939_IRQ_DEBUG_ETHER     TX4939_IRQ_INT(1)

#define RBTX4939_IRQ_IOC_BEG         (1 + TX4939_IRQ_IRC_END)	/* 59 - 66 */
#define RBTX4939_IRQ_IOC_END         (7 + RBTX4939_IRQ_IOC_BEG)

#define RBTX4939_IRQ_ISA0            (0 + RBTX4939_IRQ_IOC_BEG)
#define RBTX4939_IRQ_ISA11           (1 + RBTX4939_IRQ_IOC_BEG)
#define RBTX4939_IRQ_ISA12           (2 + RBTX4939_IRQ_IOC_BEG)
#define RBTX4939_IRQ_ISA15           (3 + RBTX4939_IRQ_IOC_BEG)
#define RBTX4939_IRQ_I2S             (4 + RBTX4939_IRQ_IOC_BEG)

/* I/O Controller REGISTER ADDRESS MAP */
#define RBTX4939_BOARD_REV_REG       (0x0000 + RBTX4939_IR_REG)
#define RBTX4939_IOC_REV_REG         (0x0002 + RBTX4939_IR_REG)
#define RBTX4939_CONFIG1_REG         (0x0004 + RBTX4939_IR_REG)
#define RBTX4939_CONFIG2_REG         (0x0006 + RBTX4939_IR_REG)
#define RBTX4939_CONFIG3_REG         (0x0008 + RBTX4939_IR_REG)
#define RBTX4939_CONFIG4_REG         (0x000a + RBTX4939_IR_REG)
#define RBTX4939_USERSW_REG          (0x1002 + RBTX4939_IR_REG)
#define RBTX4939_BOOTSW_REG          (0x1004 + RBTX4939_IR_REG)
#define RBTX4939_INTE_REG            (0x2000 + RBTX4939_IR_REG)
#define RBTX4939_INTP_REG            (0x2002 + RBTX4939_IR_REG)
#define RBTX4939_INTF1_REG           (0x2004 + RBTX4939_IR_REG)
#define RBTX4939_INTF2_REG           (0x2006 + RBTX4939_IR_REG)
#define RBTX4939_SOFT_INT_REG        (0x3000 + RBTX4939_IR_REG)
#define RBTX4939_ISA_STATUS_REG      (0x4000 + RBTX4939_IR_REG)
#define RBTX4939_PCI66_REG           (0x4002 + RBTX4939_IR_REG)
#define RBTX4939_ROMEMU_REG          (0x4004 + RBTX4939_IR_REG)
#define RBTX4939_SPICS_REG           (0x4006 + RBTX4939_IR_REG)
#define RBTX4939_AUDIO_MODE_REG      (0x4008 + RBTX4939_IR_REG)
#define RBTX4939_ISA_GPIO_REG        (0x400a + RBTX4939_IR_REG)
#define RBTX4939_PE1_REG             (0x5000 + RBTX4939_IR_REG)
#define RBTX4939_PE2_REG             (0x5002 + RBTX4939_IR_REG)
#define RBTX4939_PE3_REG             (0x5004 + RBTX4939_IR_REG)
#define RBTX4939_VPORT_MODE_REG      (0x5006 + RBTX4939_IR_REG)
#define RBTX4939_VPORT_RESET_REG     (0x5008 + RBTX4939_IR_REG)
#define RBTX4939_VPORT_SOUT_REG      (0x500A + RBTX4939_IR_REG)
#define RBTX4939_VPORT_SIN_REG       (0x500B + RBTX4939_IR_REG)
#define RBTX4939_7SEGLED_REG         (0x6000 + RBTX4939_IR_REG)
#define RBTX4939_SOFT_RESET_REG      (0x7000 + RBTX4939_IR_REG)
#define RBTX4939_SRESET_ENABLE_REG   (0x7002 + RBTX4939_IR_REG)
#define RBTX4939_RESET_STATUS_REG    (0x7004 + RBTX4939_IR_REG)

#ifndef __ASSEMBLY__
#define rbtx4939_board_rev_ptr       \
        (RBTX4939_BOARD_REV_REG)
#define rbtx4939_ioc_rev_ptr         \
        (RBTX4939_IOC_REV_REG)
#define rbtx4939_config1_ptr         \
        (RBTX4939_CONFIG1_REG)
#define rbtx4939_config2_ptr         \
        (RBTX4939_CONFIG2_REG)
#define rbtx4939_config3_ptr         \
        (RBTX4939_CONFIG3_REG)
#define rbtx4939_config4_ptr         \
        (RBTX4939_CONFIG4_REG)
#define rbtx4939_usersw_ptr          \
        (RBTX4939_USERSW_REG)
#define rbtx4939_bootsw_ptr          \
        (RBTX4939_BOOTSW_REG)
#define rbtx4939_inte_ptr            \
        (RBTX4939_INTE_REG)
#define rbtx4939_intp_ptr            \
        (RBTX4939_INTP_REG)
#define rbtx4939_intf1_ptr           \
        (RBTX4939_INTF1_REG)
#define rbtx4939_intf2_ptr           \
        (RBTX4939_INTF2_REG)
#define rbtx4939_soft_int_ptr        \
        (RBTX4939_SOFT_INT_REG)
#define rbtx4939_isa_status_ptr      \
        (RBTX4939_ISA_STATUS_REG)
#define rbtx4939_pci66_ptr           \
        (RBTX4939_PCI66_REG)
#define rbtx4939_romemu_ptr          \
        (RBTX4939_ROMEMU_REG)
#define rbtx4939_spics_ptr           \
        (RBTX4939_SPICS_REG)
#define rbtx4939_audio_mode_ptr      \
        (RBTX4939_AUDIO_MODE_REG)
#define rbtx4939_isa_gpio_ptr        \
        (RBTX4939_ISA_GPIO_REG)
#define rbtx4939_pe1_ptr             \
        (RBTX4939_PE1_REG)
#define rbtx4939_pe2_ptr             \
        (RBTX4939_PE2_REG)
#define rbtx4939_pe3_ptr             \
        (RBTX4939_PE3_REG)
#define rbtx4939_vport_mode_ptr      \
        (RBTX4939_VPORT_MODE_REG)
#define rbtx4939_vport_reset_ptr     \
        (RBTX4939_VPORT_RESET_REG)
#define rbtx4939_vport_sout_ptr      \
        (RBTX4939_VPORT_SOUT_REG)
#define rbtx4939_vport_sin_ptr       \
        (RBTX4939_VPORT_SIN_REG)
#define rbtx4939_7segled_ptr         \
        (RBTX4939_7SEGLED_REG)
#define rbtx4939_soft_reset_ptr      \
        (RBTX4939_SOFT_RESET_REG)
#define rbtx4939_sreset_enable_ptr   \
        (RBTX4939_SRESET_ENABLE_REG)
#define rbtx4939_reset_status_ptr    \
        (RBTX4939_RESET_STATUS_REG)
#endif				/* __ASSEMBLY__ */

#define RBTX4939_PE1_RMII1   0x08
#define RBTX4939_PE1_RMII0   0x04
#define RBTX4939_PE1_ATA1    0x02
#define RBTX4939_PE1_ATA0    0x01

#define RBTX4939_PE2_GPIO    0x20
#define RBTX4939_PE2_SPI     0x10
#define RBTX4939_PE2_CIR     0x08
#define RBTX4939_PE2_SIO3    0x04
#define RBTX4939_PE2_SIO2    0x02
#define RBTX4939_PE2_SIO1    0x01

#define RBTX4939_PE3_VP_S    0x04
#define RBTX4939_PE3_VP_P    0x02
#define RBTX4939_PE3_VP      0x01

#endif				/* __ASM_TX_BOARDS_RBTX4939_H */
