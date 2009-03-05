#ifndef __ASM_SPINLOCK_H
#define __ASM_SPINLOCK_H

#include <asm/atomic.h>
#include <asm/rwlock.h>
#include <asm/page.h>
#include <linux/config.h>
#include <linux/list.h>

#define SPINLOCK_MAGIC	0xdead4ead

#ifdef CONFIG_DEBUG_SPINLOCK
#define SPINLOCK_MAGIC_INIT	, SPINLOCK_MAGIC
#else
#define SPINLOCK_MAGIC_INIT	/* */
#endif

#define __RAW_SPIN_LOCK_UNLOCKED { 1 SPINLOCK_MAGIC_INIT }
#define RAW_SPIN_LOCK_UNLOCKED (raw_spinlock_t) { 1 SPINLOCK_MAGIC_INIT }

#define __raw_spin_lock_init(x) do { *(x) = RAW_SPIN_LOCK_UNLOCKED; } while(0)

/*
 * Simple spin lock operations.  There are two variants, one clears IRQ's
 * on the local processor, one does not.
 *
 * We make no fairness assumptions. They have a cost.
 */

#define __raw_spin_is_locked(x) (*(volatile signed char *)(&(x)->lock) <= 0)
#define __raw_spin_unlock_wait(x) \
		do { barrier(); } while (__raw_spin_is_locked(x))
#define __raw_spin_lock_flags(lock, flags) _raw_spin_lock(lock)
	
 #define spin_lock_string \
	"\n1:\t" \
	"lock ; decb %0\n\t" \
	"js 2f\n" \
	LOCK_SECTION_START("") \
	"2:\t" \
	"rep;nop\n\t" \
	"cmpb $0,%0\n\t" \
	"jle 2b\n\t" \
	"jmp 1b\n" \
	LOCK_SECTION_END

#define spin_lock_string_flags \
	"\n1:\t" \
	"lock ; decb %0\n\t" \
	"js 2f\n\t" \
	LOCK_SECTION_START("") \
	"2:\t" \
	"test $0x200, %1\n\t" \
	"jz 3f\n\t" \
	"sti\n\t" \
	"3:\t" \
	"rep;nop\n\t" \
	"cmpb $0, %0\n\t" \
	"jle 3b\n\t" \
	"cli\n\t" \
	"jmp 1b\n" \
	LOCK_SECTION_END

/*
 * This works. Despite all the confusion.
 * (except on PPro SMP or if we are using OOSTORE)
 * (PPro errata 66, 92)
 */
 
#if !defined(CONFIG_X86_OOSTORE) && !defined(CONFIG_X86_PPRO_FENCE)

#define spin_unlock_string \
	"movb $1,%0" \
		:"=m" (lock->lock) : : "memory"


static inline void __raw_spin_unlock(raw_spinlock_t *lock)
{
#ifdef CONFIG_DEBUG_SPINLOCK
	BUG_ON(lock->magic != SPINLOCK_MAGIC);
	BUG_ON(!__raw_spin_is_locked(lock));
#endif
	__asm__ __volatile__(
		spin_unlock_string
	);
}

#else

#define spin_unlock_string \
	"xchgb %b0, %1" \
		:"=q" (oldval), "=m" (lock->lock) \
		:"0" (oldval) : "memory"

static inline void __raw_spin_unlock(raw_spinlock_t *lock)
{
	char oldval = 1;
#ifdef CONFIG_DEBUG_SPINLOCK
	BUG_ON(lock->magic != SPINLOCK_MAGIC);
	BUG_ON(!__raw_spin_is_locked(lock));
#endif
	__asm__ __volatile__(
		spin_unlock_string
	);
}

#endif

static inline int __raw_spin_trylock(raw_spinlock_t *lock)
{
	char oldval;
	__asm__ __volatile__(
		"xchgb %b0,%1"
		:"=q" (oldval), "=m" (lock->lock)
		:"0" (0) : "memory");
	return oldval > 0;
}

static inline void __raw_spin_lock(raw_spinlock_t *lock)
{
#ifdef CONFIG_DEBUG_SPINLOCK
	if (lock->magic != SPINLOCK_MAGIC) {
		printk("eip: %p\n", __builtin_return_address(0));
		BUG();
	}
#endif
	__asm__ __volatile__(
		spin_lock_string
		:"=m" (lock->lock) : : "memory");
}

static inline void _raw_spin_lock_flags (spinlock_t *lock, unsigned long flags)
{
#ifdef CONFIG_DEBUG_SPINLOCK
	__label__ here;
here:
	if (unlikely(lock->magic != SPINLOCK_MAGIC)) {
		printk("eip: %p\n", &&here);
		BUG();
	}
#endif
	__asm__ __volatile__(spin_lock_string_flags
		:"=m" (lock->lock) : "r" (flags) : "memory");
}


#define RWLOCK_MAGIC	0xdeaf1eed

#ifdef CONFIG_DEBUG_SPINLOCK
#define RWLOCK_MAGIC_INIT	, RWLOCK_MAGIC
#else
#define RWLOCK_MAGIC_INIT	/* */
#endif

#define __RAW_RW_LOCK_UNLOCKED { RW_LOCK_BIAS RWLOCK_MAGIC_INIT }
#define RAW_RW_LOCK_UNLOCKED (raw_rwlock_t) { RW_LOCK_BIAS RWLOCK_MAGIC_INIT }
#define __raw_rwlock_init(x)  do { *(x) = RAW_RW_LOCK_UNLOCKED; } while(0)
#define __raw_rwlock_is_locked(x) ((x)->lock != RW_LOCK_BIAS)
  
#define __raw_read_can_lock(x)	((int)(x)->lock > 0)
#define __raw_write_can_lock(x)	((x)->lock == RW_LOCK_BIAS)

 /*
 * On x86, we implement read-write locks as a 32-bit counter
 * with the high bit (sign) being the "contended" bit.
 *
 * The inline assembly is non-obvious. Think about it.
 *
 * Changed to use the same technique as rw semaphores.  See
 * semaphore.h for details.  -ben
 */
/* the spinlock helpers are in arch/i386/kernel/semaphore.c */

static inline void __raw_read_lock(raw_rwlock_t *rw)
{
#ifdef CONFIG_DEBUG_SPINLOCK
	BUG_ON(rw->magic != RWLOCK_MAGIC);
#endif
	__build_read_lock(rw, "__read_lock_failed");
}

static inline void __raw_write_lock(raw_rwlock_t *rw)
{
#ifdef CONFIG_DEBUG_SPINLOCK
	BUG_ON(rw->magic != RWLOCK_MAGIC);
#endif
	__build_write_lock(rw, "__write_lock_failed");
}

#define __raw_read_unlock(rw)		asm volatile("lock ; incl %0" :"=m" ((rw)->lock) : : "memory")
#define __raw_write_unlock(rw)	asm volatile("lock ; addl $" RW_LOCK_BIAS_STR ",%0":"=m" ((rw)->lock) : : "memory")

static inline int __raw_read_trylock(raw_rwlock_t *lock)
{
	atomic_t *count = (atomic_t *)lock;
	atomic_dec(count);
	if (atomic_read(count) >= 0)
		return 1;
	atomic_inc(count);
	return 0;
}

static inline int __raw_write_trylock(raw_rwlock_t *lock)
{
	atomic_t *count = (atomic_t *)lock;
	if (atomic_sub_and_test(RW_LOCK_BIAS, count))
		return 1;
	atomic_add(RW_LOCK_BIAS, count);
	return 0;
}

#endif /* __ASM_SPINLOCK_H */
