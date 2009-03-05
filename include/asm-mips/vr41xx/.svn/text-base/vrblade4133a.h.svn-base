/*
 * include/asm-mips/vr41xx/vrblade4133a.h
 *
 * Include file for NCOS VRBlade VR4133 Wildcat.
 *
 * Author: Edmond dela Cruz <edmondd@ntsp.nec.co.jp>
 *
 * 2002-2004 (c) MontaVista, Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef __NEC_VRBLADE4133A_H
#define __NEC_VRBLADE4133A_H

#include <linux/config.h>

#include <asm/addrspace.h>
#include <asm/vr41xx/vr41xx.h>

/*
 * Board specific address mapping
 */
#define VR4133_PCI_MEM_BASE		VR4133_PCI_BASE
#define VR4133_PCI_MEM_SIZE		0x06000000

#define VR4133_PCI_IO_BASE		0x16000000
#define VR4133_PCI_IO_SIZE		0x02000000

#define VR4133_PCI_IO_START		0x01000000
#define VR4133_PCI_IO_END		0x01ffffff

#define VR4133_PCI_MEM_START		0x12000000
#define VR4133_PCI_MEM_END		0x15ffffff

#define VR4133_ISA_IO_BASE		KSEG1ADDR(VR4133_EXTERNAL_IO_BASE)

#define VR4133_IO_PORT_BASE		KSEG1ADDR(VR4133_PCI_IO_BASE)

/*
 * IRQ block assignment
 */
#define VR4133_CPU_IRQ_BASE     0
#define VR4133_SYSINT1_IRQ_BASE 8
#define VR4133_SYSINT2_IRQ_BASE 24
#define VR4133_GIUINTL_IRQ_BASE 40
#define VR4133_GIUINTH_IRQ_BASE 56
#define VR4133_ETHERMAC_IRQ_BASE 72

/*
 * Interrupt Number
 */
#define VR4133_IRQ_PCI_INTA			GIU_IRQ(2)  /* IDE HPT371 */
#define VR4133_IRQ_PCI_INTB			GIU_IRQ(3)  /* USB chip, Mini-PCI slot2 */
#define VR4133_IRQ_PCI_INTC			GIU_IRQ(1)  /* Mini-PCI slot1 */
#define VR4133_IRQ_ETHER_MAC0		VR4133_ETHERMAC_IRQ_BASE /* EtherMAC0 */
#define VR4133_IRQ_ETHER_MAC1		VR4133_ETHERMAC_IRQ_BASE+1 /* EtherMAC1 */

#define VR4133_IRQ_LAST      VR4133_IRQ_ETHER_MAC1

/*
 * Function pins
 */
#define EB4133_SCL_DATAREG  VR4133_GIUPODATL
#define EB4133_SDA_DATAREG  VR4133_GIUPIODL
#define NEC_VR4133_SCL          VR4133_GIUPIODL_GPIO10 	/* I2C SCL (For WildCat Version) */
#define NEC_VR4133_SDA          VR4133_GIUPIODL_GPIO11  /* I2C SDA (For WildCat Version) */
#define NEC_ROMREADY            VR4133_GIUPIODL_GPIO0   /* Flash ROM ready */

/*
 * I2C device's slave address
 */
#define RICOH_RTC_ADDR		0x32	/* Ricoh RTC device */
#define ATMEL_EEPROM_ADDR	0x50	/* ATMEL EEPROM device */


/*
 * GPIO
*/

#define GIUIOSELL_TYPE1	KSEG1ADDR(0x0b000100)
#define GIUIOSELL_TYPE2	KSEG1ADDR(0x0f000140)

#define GIUIOSELL	0x00
#define GIUIOSELH	0x02
#define GIUPIODL 	0x04
#define GIUINTSTATL	0x08
#define GIUINTSTATH	0x0a
#define GIUINTENL	0x0c
#define GIUINTENH	0x0e
#define GIUINTTYPL	0x10
#define GIUINTTYPH	0x12
#define GIUINTALSELL	0x14
#define GIUINTALSELH	0x16
#define GIUINTHTSELL	0x18
#define GIUINTHTSELH	0x1a
#define GIUFEDGEINHL	0x20
#define GIUFEDGEINHH	0x22
#define GIUREDGEINHL	0x24
#define GIUREDGEINHH	0x26

#define CMBVR4133_SDA_PIN		11
#define CMBVR4133_SCL_PIN		10

#endif /* __NEC_VRBLADE4133A_H */

