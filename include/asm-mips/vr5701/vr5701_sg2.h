/*
 * include/asm-mips/vr5701/vr5701_sg2.h
 *
 * Flash memory access on NEC Electronics Corporation VR5701 SolutionGearII board.
 *
 * Author: Sergey Podstavin <spodstavin@ru.mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#ifndef __VR5701_SG2_H
#define __VR5701_SG2_H

#include <asm/vr5701/vr5701.h>

#define VR5701_SG2_SDRAM_SIZE 	0x10000000

#ifndef __ASSEMBLY__
#include <asm/delay.h>

/*
 *  PCI Master Registers
 */

#define DDB_PCICMD_IACK		0	/* PCI Interrupt Acknowledge */
#define DDB_PCICMD_IO		1	/* PCI I/O Space */
#define DDB_PCICMD_MEM		3	/* PCI Memory Space */
#define DDB_PCICMD_CFG		5	/* PCI Configuration Space */

/*
 * additional options for pci init reg (no shifting needed)
 */
#define DDB_PCI_CFGTYPE1     	0x200	/* for pci init0/1 regs */
#define DDB_PCI_ACCESS_32    	0x10	/* for pci init0/1 regs */
#define NUM_5701_IRQS 	  	32
#define NUM_5701_EPCI_IRQ 	4

/* A Real Time Clock interface for Linux on NEC Electronics Corporation VR5701 SolutionGearII */
#define SET_32_BIT		0xffffffff
#define CLR_32_BIT		0x00000000

#define GPIO_3_INTR		(0x1 <<  3)
#define GPIO_4_CE		(0x1 <<  4)
#define GPIO_25_S1CLK		(0x1 << 25)
#define GPIO_26_S1DO		(0x1 << 26)
#define GPIO_27_S1DI		(0x1 << 27)
#define GPIO_CSI1_PIN	(GPIO_25_S1CLK | GPIO_26_S1DO | GPIO_27_S1DI)

#define CSIn_MODE_CKP		(0x1 << 12)
#define CSIn_MODE_DAP		(0x1 << 11)
#define CSIn_MODE_CKS_MASK	(0x7 <<  8)
#define CSIn_MODE_CKS_833333MHZ	(0x1 <<  8)
#define CSIn_MODE_CKS_416667MHZ	(0x2 <<  8)
#define CSIn_MODE_CKS_208333MHZ	(0x3 <<  8)
#define CSIn_MODE_CKS_104167MHZ	(0x4 <<  8)
#define CSIn_MODE_CKS_052083MHZ	(0x5 <<  8)
#define CSIn_MODE_CKS_0260417HZ	(0x6 <<  8)	/* Default */
#define CSIn_MODE_CSIE		(0x1 <<  7)
#define CSIn_MODE_TRMD		(0x1 <<  6)
#define CSIn_MODE_CCL_16	(0x1 <<  5)
#define CSIn_MODE_DIR_LSB	(0x1 <<  4)
#define CSIn_MODE_AUTO		(0x1 <<  2)
#define CSIn_MODE_CSOT		(0x1 <<  0)

#define CSIn_INT_CSIEND		(0x1 << 15)
#define CSIn_INT_T_EMP		(0x1 <<  8)
#define CSIn_INT_R_OVER		(0x1 <<  0)

/* IRQs */
#define ACTIVE_LOW		1
#define ACTIVE_HIGH		0

#define LEVEL_SENSE		2
#define EDGE_TRIGGER		0

#define NUM_5701_IRQS 		32
#define NUM_5701_EPCI_IRQS 	4
#define NUM_5701_IPCI_IRQS 	8
#define NUM_5701_IRQ  		32
#define NUM_EPCI_IRQ  		4
#define NUM_IPCI_IRQ  		8

#define INTA			0
#define INTB			1
#define INTC			2
#define INTD			3
#define INTE			4

/* Timers */
#define	CPU_COUNTER_FREQUENCY		166666666

#define ddb_sync       		io_sync
#define ddb_out32(x,y) 		io_out32(x,y)
#define ddb_out16(x,y) 		io_out16(x,y)
#define ddb_out8(x,y)  		io_out8(x,y)
#define ddb_in32(x)    		io_in32(x)
#define ddb_in16(x)    		io_in16(x)
#define ddb_in8(x)     		io_in8(x)

static inline void io_sync(void)
{
	asm("sync");
}

static inline void io_out32(u32 offset, u32 val)
{
	*(volatile u32 *)(VR5701_IO_BASE + offset) = val;
	io_sync();
}

static inline u32 io_in32(u32 offset)
{
	u32 val = *(volatile u32 *)(VR5701_IO_BASE + offset);
	io_sync();
	return val;
}

static inline void io_out16(u32 offset, u16 val)
{
	*(volatile u16 *)(VR5701_IO_BASE + offset) = val;
	io_sync();
}

static inline u16 io_in16(u32 offset)
{
	u16 val = *(volatile u16 *)(VR5701_IO_BASE + offset);
	io_sync();
	return val;
}

static inline void io_reset16(unsigned long adr,
			      unsigned short val1,
			      unsigned delay, unsigned short val2)
{
	io_out16(adr, val1);
	__delay(delay);
	io_out16(adr, val2);
}

static inline void io_out8(u32 offset, u8 val)
{
	*(volatile u8 *)(VR5701_IO_BASE + offset) = val;
	io_sync();
}

static inline u8 io_in8(u32 offset)
{
	u8 val = *(volatile u8 *)(VR5701_IO_BASE + offset);
	io_sync();
	return val;
}

static inline void io_set16(u32 offset, u16 mask, u16 val)
{
	u16 val0 = io_in16(offset);
	io_out16(offset, (val & mask) | (val0 & ~mask));
}

static inline void reg_set32(u32 offset, u32 mask, u32 val)
{
	u32 val0 = io_in32(offset);
	io_out32(offset, (val & mask) | (val0 & ~mask));
}

static inline void csi1_reset(void)
{
	/* CSI1 reset */
	reg_set32(CSI1_CNT, 0x00008000, SET_32_BIT);	/* set CSIRST bit */
	__delay(100000);
	reg_set32(CSI1_CNT, 0x00008000, CLR_32_BIT);	/* clear CSIRST bit */
	/* set clock phase  */
	while (io_in32(CSI1_MODE) & 1) ;
	reg_set32(CSI1_MODE, CSIn_MODE_CSIE, CLR_32_BIT);
	reg_set32(CSI1_MODE, CSIn_MODE_CKP, SET_32_BIT);
	reg_set32(CSI1_MODE, CSIn_MODE_CKS_104167MHZ, SET_32_BIT);
	reg_set32(CSI1_MODE, CSIn_MODE_CSIE, SET_32_BIT);
	while (io_in32(CSI1_MODE) & CSIn_MODE_CSOT) ;
}

extern void ll_vr5701_irq_route(int vr5701_irq, int ip);
extern void ll_vr5701_irq_enable(int vr5701_irq);
extern void ddb_set_pdar(u32, u32, u32, int, int, int);
extern void ddb_set_pmr(u32 pmr, u32 type, u32 addr, u32 options);
extern void ddb_set_bar(u32 bar, u32 phys, int prefetchable);
extern void ddb_pci_reset_bus(void);
extern struct pci_ops VR5701_ext_pci_ops;
extern struct pci_ops VR5701_io_pci_ops;
extern struct pci_controller VR5701_ext_controller;
extern struct pci_controller VR5701_io_controller;
extern int vr5701_sg2_setup(void);
extern void vr5701_sg2_irq_init(u32 base);
extern void mips_cpu_irq_init(u32 base);
extern asmlinkage void vr5701_sg2_handle_int(void);
extern void vr5701_irq_init(u32 irq_base);
extern int panic_timeout;

#endif
#endif
