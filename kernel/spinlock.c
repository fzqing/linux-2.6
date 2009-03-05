/*
 * Copyright (2004) Linus Torvalds
 *
 * Author: Zwane Mwaikambo <zwane@fsmlabs.com>
 *
 * Copyright (2004) Ingo Molnar
 */

#include <linux/config.h>
#include <linux/linkage.h>
#include <linux/preempt.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/module.h>

/*
 * Generic declaration of the raw read_trylock() function,
 * architectures are supposed to optimize this:
 */
int __lockfunc generic_raw_read_trylock(raw_rwlock_t *lock)
{
	__raw_read_lock(lock);
	return 1;
}
EXPORT_SYMBOL(generic_raw_read_trylock);

int __lockfunc _raw_spin_trylock(raw_spinlock_t *lock)
{
	preempt_disable();
	if (__raw_spin_trylock(lock))
		return 1;
	
	preempt_enable();
	return 0;
}
EXPORT_SYMBOL(_raw_spin_trylock);

int __lockfunc _raw_read_trylock(raw_rwlock_t *lock)
{
	preempt_disable();
	if (__raw_read_trylock(lock))
		return 1;

	preempt_enable();
	return 0;
}
EXPORT_SYMBOL(_raw_read_trylock);

int __lockfunc _raw_write_trylock(raw_rwlock_t *lock)
{
	preempt_disable();
	if (__raw_write_trylock(lock))
		return 1;

	preempt_enable();
	return 0;
}
EXPORT_SYMBOL(_raw_write_trylock);

#ifndef CONFIG_PREEMPT

void __lockfunc _raw_read_lock(raw_rwlock_t *lock)
{
	preempt_disable();
	__raw_read_lock(lock);
}
EXPORT_SYMBOL(_raw_read_lock);

unsigned long __lockfunc _raw_spin_lock_irqsave(raw_spinlock_t *lock)
{
	unsigned long flags;

	local_irq_save(flags);
	preempt_disable();
	__raw_spin_lock_flags(lock, flags);
	return flags;
}
EXPORT_SYMBOL(_raw_spin_lock_irqsave);

void __lockfunc _raw_spin_lock_irq(raw_spinlock_t *lock)
{
	local_irq_disable();
	preempt_disable();
	__raw_spin_lock(lock);
}
EXPORT_SYMBOL(_raw_spin_lock_irq);

void __lockfunc _raw_spin_lock_bh(raw_spinlock_t *lock)
{
	local_bh_disable();
	preempt_disable();
	__raw_spin_lock(lock);
}
EXPORT_SYMBOL(_raw_spin_lock_bh);

unsigned long __lockfunc _raw_read_lock_irqsave(raw_rwlock_t *lock)
{
	unsigned long flags;

	local_irq_save(flags);
	preempt_disable();
	__raw_read_lock(lock);
	return flags;
}
EXPORT_SYMBOL(_raw_read_lock_irqsave);

void __lockfunc _raw_read_lock_irq(raw_rwlock_t *lock)
{
	local_irq_disable();
	preempt_disable();
	__raw_read_lock(lock);
}
EXPORT_SYMBOL(_raw_read_lock_irq);

void __lockfunc _raw_read_lock_bh(raw_rwlock_t *lock)
{
	local_bh_disable();
	preempt_disable();
	__raw_read_lock(lock);
}
EXPORT_SYMBOL(_raw_read_lock_bh);

unsigned long __lockfunc _raw_write_lock_irqsave(raw_rwlock_t *lock)
{
	unsigned long flags;

	local_irq_save(flags);
	preempt_disable();
	__raw_write_lock(lock);
	return flags;
}
EXPORT_SYMBOL(_raw_write_lock_irqsave);

void __lockfunc _raw_write_lock_irq(raw_rwlock_t *lock)
{
	local_irq_disable();
	preempt_disable();
	__raw_write_lock(lock);
}
EXPORT_SYMBOL(_raw_write_lock_irq);

void __lockfunc _raw_write_lock_bh(raw_rwlock_t *lock)
{
	local_bh_disable();
	preempt_disable();
	__raw_write_lock(lock);
}
EXPORT_SYMBOL(_raw_write_lock_bh);

void __lockfunc _raw_spin_lock(raw_spinlock_t *lock)
{
	preempt_disable();
	__raw_spin_lock(lock);
}
EXPORT_SYMBOL(_raw_spin_lock);

void __lockfunc _raw_write_lock(raw_rwlock_t *lock)
{
	preempt_disable();
	__raw_write_lock(lock);
}
EXPORT_SYMBOL(_raw_write_lock);

#else /* CONFIG_PREEMPT: */

/*
 * This could be a long-held lock. We both prepare to spin for a long
 * time (making _this_ CPU preemptable if possible), and we also signal
 * towards that other CPU that it should break the lock ASAP.
 *
 * (We do this in a function because inlining it would be excessive.)
 */

#define BUILD_LOCK_OPS(op, locktype)					\
void __lockfunc _raw_##op##_lock(locktype##_t *lock)			\
{									\
	preempt_disable();						\
	for (;;) {							\
		if (likely(__raw_##op##_trylock(lock)))			\
			break;						\
		preempt_enable();					\
		if (!(lock)->break_lock)				\
			(lock)->break_lock = 1;				\
		while (!op##_can_lock(lock) && (lock)->break_lock)	\
			cpu_relax();					\
		preempt_disable();					\
	}								\
}									\
									\
EXPORT_SYMBOL(_raw_##op##_lock);					\
									\
unsigned long __lockfunc _raw_##op##_lock_irqsave(locktype##_t *lock)	\
{									\
	unsigned long flags;						\
									\
	preempt_disable();						\
	for (;;) {							\
		local_irq_save(flags);					\
		if (likely(__raw_##op##_trylock(lock)))			\
			break;						\
		local_irq_restore(flags);				\
									\
		preempt_enable();					\
		if (!(lock)->break_lock)				\
			(lock)->break_lock = 1;				\
		while (!op##_can_lock(lock) && (lock)->break_lock)	\
			cpu_relax();					\
		preempt_disable();					\
	}								\
	return flags;							\
}									\
									\
EXPORT_SYMBOL(_raw_##op##_lock_irqsave);				\
									\
void __lockfunc _raw_##op##_lock_irq(locktype##_t *lock)		\
{									\
	_raw_##op##_lock_irqsave(lock);					\
}									\
									\
EXPORT_SYMBOL(_raw_##op##_lock_irq);					\
									\
void __lockfunc _raw_##op##_lock_bh(locktype##_t *lock)			\
{									\
	unsigned long flags;						\
									\
	/*							*/	\
	/* Careful: we must exclude softirqs too, hence the	*/	\
	/* irq-disabling. We use the generic preemption-aware	*/	\
	/* function:						*/	\
	/**/								\
	flags = _raw_##op##_lock_irqsave(lock);				\
	local_bh_disable();						\
	local_irq_restore(flags);					\
}									\
									\
EXPORT_SYMBOL(_raw_##op##_lock_bh)

/*
 * Build preemption-friendly versions of the following
 * lock-spinning functions:
 *
 *         _[spin|read|write]_lock()
 *         _[spin|read|write]_lock_irq()
 *         _[spin|read|write]_lock_irqsave()
 *         _[spin|read|write]_lock_bh()
 */
BUILD_LOCK_OPS(spin, raw_spinlock);
BUILD_LOCK_OPS(read, raw_rwlock);
BUILD_LOCK_OPS(write, raw_rwlock);

#endif /* CONFIG_PREEMPT */

void __lockfunc _raw_spin_unlock(raw_spinlock_t *lock)
{
	__raw_spin_unlock(lock);
	preempt_enable();
}
EXPORT_SYMBOL(_raw_spin_unlock);

void __lockfunc _raw_write_unlock(raw_rwlock_t *lock)
{
	__raw_write_unlock(lock);
	preempt_enable();
}
EXPORT_SYMBOL(_raw_write_unlock);

void __lockfunc _raw_read_unlock(raw_rwlock_t *lock)
{
	__raw_read_unlock(lock);
	preempt_enable();
}
EXPORT_SYMBOL(_raw_read_unlock);

void __lockfunc _raw_spin_unlock_irqrestore(raw_spinlock_t *lock, unsigned long flags)
{
	__raw_spin_unlock(lock);
	preempt_enable_no_resched();
	local_irq_restore(flags);
	preempt_check_resched();
}
EXPORT_SYMBOL(_raw_spin_unlock_irqrestore);

void __lockfunc _raw_spin_unlock_irq(raw_spinlock_t *lock)
{
	__raw_spin_unlock(lock);
	preempt_enable_no_resched();
	local_irq_enable();
	preempt_check_resched();
}
EXPORT_SYMBOL(_raw_spin_unlock_irq);

void __lockfunc _raw_spin_unlock_bh(raw_spinlock_t *lock)
{
	__raw_spin_unlock(lock);
	preempt_enable_no_resched();
	local_bh_enable();
}
EXPORT_SYMBOL(_raw_spin_unlock_bh);

void __lockfunc _raw_read_unlock_irqrestore(raw_rwlock_t *lock, unsigned long flags)
{
	__raw_read_unlock(lock);
	preempt_enable_no_resched();
	local_irq_restore(flags);
	preempt_check_resched();
}
EXPORT_SYMBOL(_raw_read_unlock_irqrestore);

void __lockfunc _raw_read_unlock_irq(raw_rwlock_t *lock)
{
	__raw_read_unlock(lock);
	preempt_enable_no_resched();
	local_irq_enable();
	preempt_check_resched();
}
EXPORT_SYMBOL(_raw_read_unlock_irq);

void __lockfunc _raw_read_unlock_bh(raw_rwlock_t *lock)
{
	__raw_read_unlock(lock);
	preempt_enable_no_resched();
	local_bh_enable();
}
EXPORT_SYMBOL(_raw_read_unlock_bh);

void __lockfunc _raw_write_unlock_irqrestore(raw_rwlock_t *lock, unsigned long flags)
{
	__raw_write_unlock(lock);
	preempt_enable_no_resched();
	local_irq_restore(flags);
	preempt_check_resched();
}
EXPORT_SYMBOL(_raw_write_unlock_irqrestore);

void __lockfunc _raw_write_unlock_irq(raw_rwlock_t *lock)
{
	__raw_write_unlock(lock);
	preempt_enable_no_resched();
	local_irq_enable();
	preempt_check_resched();
}
EXPORT_SYMBOL(_raw_write_unlock_irq);

void __lockfunc _raw_write_unlock_bh(raw_rwlock_t *lock)
{
	__raw_write_unlock(lock);
	preempt_enable_no_resched();
	local_bh_enable();
}
EXPORT_SYMBOL(_raw_write_unlock_bh);

int __lockfunc _raw_spin_trylock_bh(raw_spinlock_t *lock)
{
	local_bh_disable();
	preempt_disable();
	if (__raw_spin_trylock(lock))
		return 1;

	preempt_enable_no_resched();
	local_bh_enable();
	return 0;
}
EXPORT_SYMBOL(_raw_spin_trylock_bh);

int __lockfunc _raw_spin_trylock_irq(raw_spinlock_t *lock)
{
	local_irq_disable();
	preempt_disable();
	if (__raw_spin_trylock(lock))
		return 1;

	preempt_enable_no_resched();
	local_irq_enable();
	preempt_check_resched();

	return 0;
}
EXPORT_SYMBOL(_raw_spin_trylock_irq);

int __lockfunc _raw_spin_trylock_irqsave(raw_spinlock_t *lock,
					 unsigned long *flags)
{
	local_irq_save(*flags);
	preempt_disable();
	if (__raw_spin_trylock(lock))
		return 1;

	preempt_enable_no_resched();
	local_irq_restore(*flags);
	preempt_check_resched();

	return 0;
}
EXPORT_SYMBOL(_raw_spin_trylock_irqsave);

int notrace in_lock_functions(unsigned long addr)
{
	/* Linker adds these: start and end of __lockfunc functions */
	extern char __lock_text_start[], __lock_text_end[];

	return addr >= (unsigned long)__lock_text_start
	&& addr < (unsigned long)__lock_text_end;
}
EXPORT_SYMBOL(in_lock_functions);
