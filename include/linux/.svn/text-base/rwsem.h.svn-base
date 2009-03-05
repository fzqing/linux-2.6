/* rwsem.h: R/W semaphores, public interface
 *
 * Written by David Howells (dhowells@redhat.com).
 * Derived from asm-i386/semaphore.h
 */

#ifndef _LINUX_RWSEM_H
#define _LINUX_RWSEM_H

#include <linux/linkage.h>

#ifdef CONFIG_PREEMPT_RT
# include <linux/rt_lock.h>
#endif

#define RWSEM_DEBUG 0

#ifdef __KERNEL__

#include <linux/config.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <asm/system.h>
#include <asm/atomic.h>

#ifndef CONFIG_PREEMPT_RT
/*
 * On !PREEMPT_RT all rw-semaphores are compat:
 */
#define compat_rw_semaphore rw_semaphore
#endif

struct compat_rw_semaphore;

#ifdef CONFIG_RWSEM_GENERIC_SPINLOCK
#include <linux/rwsem-spinlock.h> /* use a generic implementation */
#else
#include <asm/rwsem.h> /* use an arch-specific implementation */
#endif

#ifndef rwsemtrace
#if RWSEM_DEBUG
extern void FASTCALL(rwsemtrace(struct compat_rw_semaphore *sem, const char *str));
#else
#define rwsemtrace(SEM,FMT)
#endif
#endif

/*
 * lock for reading
 */
static inline void compat_down_read(struct compat_rw_semaphore *sem)
{
	might_sleep();
	rwsemtrace(sem,"Entering down_read");
	__down_read(sem);
	rwsemtrace(sem,"Leaving down_read");
}

/*
 * trylock for reading -- returns 1 if successful, 0 if contention
 */
static inline int compat_down_read_trylock(struct compat_rw_semaphore *sem)
{
	int ret;
	rwsemtrace(sem,"Entering down_read_trylock");
	ret = __down_read_trylock(sem);
	rwsemtrace(sem,"Leaving down_read_trylock");
	return ret;
}

/*
 * lock for writing
 */
static inline void compat_down_write(struct compat_rw_semaphore *sem)
{
	might_sleep();
	rwsemtrace(sem,"Entering down_write");
	__down_write(sem);
	rwsemtrace(sem,"Leaving down_write");
}

/*
 * trylock for writing -- returns 1 if successful, 0 if contention
 */
static inline int compat_down_write_trylock(struct compat_rw_semaphore *sem)
{
	int ret;
	rwsemtrace(sem,"Entering down_write_trylock");
	ret = __down_write_trylock(sem);
	rwsemtrace(sem,"Leaving down_write_trylock");
	return ret;
}

/*
 * release a read lock
 */
static inline void compat_up_read(struct compat_rw_semaphore *sem)
{
	rwsemtrace(sem,"Entering up_read");
	__up_read(sem);
	rwsemtrace(sem,"Leaving up_read");
}

/*
 * release a write lock
 */
static inline void compat_up_write(struct compat_rw_semaphore *sem)
{
	rwsemtrace(sem,"Entering up_write");
	__up_write(sem);
	rwsemtrace(sem,"Leaving up_write");
}

/*
 * downgrade write lock to read lock
 */
static inline void compat_downgrade_write(struct compat_rw_semaphore *sem)
{
	rwsemtrace(sem,"Entering downgrade_write");
	__downgrade_write(sem);
	rwsemtrace(sem,"Leaving downgrade_write");
}

#ifndef CONFIG_PREEMPT_RT

#define DECLARE_RWSEM COMPAT_DECLARE_RWSEM
#define __RWSEM_INITIALIZER __COMPAT_RWSEM_INITIALIZER

static inline void init_rwsem(struct compat_rw_semaphore *rwsem)
{
	compat_init_rwsem(rwsem);
}
static inline void down_read(struct compat_rw_semaphore *rwsem)
{
	compat_down_read(rwsem);
}
static inline int down_read_trylock(struct compat_rw_semaphore *rwsem)
{
	return compat_down_read_trylock(rwsem);
}
static inline void down_write(struct compat_rw_semaphore *rwsem)
{
	compat_down_write(rwsem);
}
static inline int down_write_trylock(struct compat_rw_semaphore *rwsem)
{
	return compat_down_write_trylock(rwsem);
}
static inline void up_read(struct compat_rw_semaphore *rwsem)
{
	compat_up_read(rwsem);
}
static inline void up_write(struct compat_rw_semaphore *rwsem)
{
	compat_up_write(rwsem);
}
static inline void downgrade_write(struct compat_rw_semaphore *rwsem)
{
	compat_downgrade_write(rwsem);
}
#endif /* CONFIG_PREEMPT_RT */

#endif /* __KERNEL__ */
#endif /* _LINUX_RWSEM_H */
