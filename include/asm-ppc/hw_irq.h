/*
 * Copyright (C) 1999 Cort Dougan <cort@cs.nmt.edu>
 */
#ifdef __KERNEL__
#ifndef _PPC_HW_IRQ_H
#define _PPC_HW_IRQ_H

#include <linux/ilatency.h>
#include <asm/ptrace.h>
#include <asm/reg.h>

extern void timer_interrupt(struct pt_regs *);

#define INLINE_IRQS

#define irqs_disabled()	((mfmsr() & MSR_EE) == 0)
#define irqs_disabled_flags(flags)	((flags & MSR_EE) == 0)

#ifdef INLINE_IRQS

#ifdef CONFIG_ILATENCY
#define local_irq_enable()      ilat_irq_enable(__BASE_FILE__,__LINE__,0)
#define local_irq_disable()     ilat_irq_disable(__BASE_FILE__,__LINE__)
#define local_irq_save(x)       do {local_save_flags(x);local_irq_disable();} while (0)
#define local_irq_restore(x)    ilat_restore_flags(__BASE_FILE__,__LINE__,x)
#define __local_irq_restore(flags)	mtmsr(flags)
static inline void __local_irq_disable(void)
{
	unsigned long msr;
	msr = mfmsr();
	mtmsr(msr & ~MSR_EE);
	__asm__ __volatile__("": : :"memory");
}

static inline void __local_irq_enable(void)
{
	unsigned long msr;
	__asm__ __volatile__("": : :"memory");
	msr = mfmsr();
	mtmsr(msr | MSR_EE);
}
#else
static inline void local_irq_disable(void)
{
	unsigned long msr;
	msr = mfmsr();
	mtmsr(msr & ~MSR_EE);
	__asm__ __volatile__("": : :"memory");
}

static inline void local_irq_enable(void)
{
	unsigned long msr;
	__asm__ __volatile__("": : :"memory");
	msr = mfmsr();
	mtmsr(msr | MSR_EE);
}

static inline void local_irq_save_ptr(unsigned long *flags)
{
	unsigned long msr;
	msr = mfmsr();
	*flags = msr;
	mtmsr(msr & ~MSR_EE);
	__asm__ __volatile__("": : :"memory");
}

#define local_irq_save(flags)		local_irq_save_ptr(&flags)
#define local_irq_restore(flags)	mtmsr(flags)
#endif
#define local_save_flags(flags)		((flags) = mfmsr())

#else

extern void local_irq_enable(void);
extern void local_irq_disable(void);
extern void local_irq_restore(unsigned long);
extern void local_save_flags_ptr(unsigned long *);

#define local_save_flags(flags) local_save_flags_ptr(&flags)
#define local_irq_save(flags) ({local_save_flags(flags);local_irq_disable();})

#endif

extern void do_lost_interrupts(unsigned long);

#define mask_irq(irq) ({if (irq_desc[irq].handler && irq_desc[irq].handler->disable) irq_desc[irq].handler->disable(irq);})
#define unmask_irq(irq) ({if (irq_desc[irq].handler && irq_desc[irq].handler->enable) irq_desc[irq].handler->enable(irq);})
#define ack_irq(irq) ({if (irq_desc[irq].handler && irq_desc[irq].handler->ack) irq_desc[irq].handler->ack(irq);})

/* Should we handle this via lost interrupts and IPIs or should we don't care like
 * we do now ? --BenH.
 */
struct hw_interrupt_type;
static inline void hw_resend_irq(struct hw_interrupt_type *h, unsigned int i) {}


#endif /* _PPC_HW_IRQ_H */
#endif /* __KERNEL__ */
