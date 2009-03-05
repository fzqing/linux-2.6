/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2004-2005 Cavium Networks
 */
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>

#include <asm/irq_cpu.h>
#include <asm/mipsregs.h>
#include <asm/system.h>

#include <hal.h>

#ifndef CONFIG_SMP
#define cpu_logical_map(cpu) 0
#endif

static inline void octeon_unmask_irq(unsigned int irq)
{
    unsigned long flags;
    local_irq_save(flags);
    if (irq < 8)
    {
	clear_c0_cause(0x100 << irq);
	set_c0_status(0x100 << irq);
    }
    else
    {
	const uint64_t coreid = octeon_get_core_num();
	uint64_t bit = (irq - 8) & 0x3f;    /* Bit 0-63 of EN0 */
	uint64_t index = (irq - 8) >> 6;    /* Route to irq 2 or 3 */

	uint64_t en0 = octeon_read_csr(OCTEON_CIU_INTX_EN0(coreid*2 + index));
	en0 |= 1ull<<bit;
	octeon_write_csr(OCTEON_CIU_INTX_EN0(coreid*2 + index), en0);
    }
    local_irq_restore(flags);
}

static inline void octeon_mask_irq(unsigned int irq)
{
    unsigned long flags;
    local_irq_save(flags);
    if (irq < 8)
    {
	clear_c0_status(0x100 << irq);
    }
    else
    {
	const uint64_t coreid = octeon_get_core_num();
	uint64_t bit = (irq - 8) & 0x3f;    /* Bit 0-63 of EN0 */
	uint64_t index = (irq - 8) >> 6;    /* Route to irq 2 or 3 */

	uint64_t en0 = octeon_read_csr(OCTEON_CIU_INTX_EN0(coreid*2 + index));
	en0 &= ~(1ull<<bit);
	octeon_write_csr(OCTEON_CIU_INTX_EN0(coreid*2 + index), en0);
	octeon_read_csr(OCTEON_CIU_INTX_EN0(coreid*2 + index));
    }
    local_irq_restore(flags);
}

static inline void octeon_mask_irq_all(unsigned int irq)
{
    unsigned long flags;
    local_irq_save(flags);
    if (irq < 8)
    {
	clear_c0_status(0x100 << irq);
    }
    else
    {
	uint64_t bit = (irq - 8) & 0x3f;    /* Bit 0-63 of EN0 */
	uint64_t index = (irq - 8) >> 6;    /* Route to irq 2 or 3 */

	int cpu;
	for (cpu=0; cpu<NR_CPUS; cpu++)
	{
	    if (cpu_present(cpu))
	    {
		uint64_t coreid = cpu_logical_map(cpu);
		uint64_t en0 = octeon_read_csr(OCTEON_CIU_INTX_EN0(coreid*2 + index));
		en0 &= ~(1ull<<bit);
		octeon_write_csr(OCTEON_CIU_INTX_EN0(coreid*2 + index), en0);
		octeon_read_csr(OCTEON_CIU_INTX_EN0(coreid*2 + index));
	    }
	}
    }
    local_irq_restore(flags);
}



static inline void octeon_irq_enable(unsigned int irq)
{
	octeon_unmask_irq(irq);
}

static void octeon_irq_disable(unsigned int irq)
{
	octeon_mask_irq_all(irq);
}

static unsigned int octeon_irq_startup(unsigned int irq)
{
	octeon_unmask_irq(irq);
	return 0;
}

static void octeon_irq_shutdown(unsigned int irq)
{
	octeon_mask_irq_all(irq);
}

static void octeon_irq_ack(unsigned int irq)
{
	octeon_mask_irq(irq);
}

static void octeon_irq_end(unsigned int irq)
{
    octeon_unmask_irq(irq);
}

static void octeon_irq_set_affinity(unsigned int irq, cpumask_t dest)
{
}

static hw_irq_controller octeon_irq_controller = {
	"Octeon",
	octeon_irq_startup,
	octeon_irq_shutdown,
	octeon_irq_enable,
	octeon_irq_disable,
	octeon_irq_ack,
	octeon_irq_end,
    octeon_irq_set_affinity
};

void __init octeon_irq_init(void)
{
	int i;

    if (NR_IRQS < 8 + 64*1)
	printk("octeon_irq_init: NR_IRQS is set too low\n");

	for (i = 0; i < 8 + 64*1; i++) {
		irq_desc[i].status = IRQ_DISABLED;
		if (i == CAVIUM_TIMER_IRQ ||
		    i == CAVIUM_IPC_IRQ ||
		    i == CAVIUM_ECC_IRQ ||
		    i == CAVIUM_ETHERNET_IRQ)
			irq_desc[i].status |= IRQ_PER_CPU;
		irq_desc[i].action = NULL;
		irq_desc[i].depth = 1;
		irq_desc[i].handler = &octeon_irq_controller;
	}
    set_c0_status(0x100 << 2);
}
