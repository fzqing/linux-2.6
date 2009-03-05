/*
 * include/asm-xtensa/semaphore.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 - 2005 Tensilica Inc.
 */

#ifndef _XTENSA_SEMAPHORE_H
#define _XTENSA_SEMAPHORE_H

#include <asm/atomic.h>
#include <asm/system.h>
#include <linux/wait.h>
#include <linux/rwsem.h>

# define compat_semaphore semaphore

struct compat_semaphore {
	atomic_t count;
	int sleepers;
	wait_queue_head_t wait;
#if WAITQUEUE_DEBUG
	long __magic;
#endif
};

#if WAITQUEUE_DEBUG
# define __SEM_DEBUG_INIT(name) \
		, (int)&(name).__magic
#else
# define __SEM_DEBUG_INIT(name)
#endif

#define __COMPAT_SEMAPHORE_INITIALIZER(name,count)		\
	{ ATOMIC_INIT(count), 					\
	  0,							\
	  __WAIT_QUEUE_HEAD_INITIALIZER((name).wait)		\
	__SEM_DEBUG_INIT(name) }

#define __COMPAT_MUTEX_INITIALIZER(name) \
	__COMPAT_SEMAPHORE_INITIALIZER(name, 1)

#define __COMPAT_DECLARE_SEMAPHORE_GENERIC(name,count) \
	struct compat_semaphore name = __COMPAT_SEMAPHORE_INITIALIZER(name,count)
   
#define COMPAT_DECLARE_MUTEX(name) __COMPAT_DECLARE_SEMAPHORE_GENERIC(name,1)
#define COMPAT_DECLARE_MUTEX_LOCKED(name) __COMPAT_DECLARE_SEMAPHORE_GENERIC(name,0)

static inline void compat_sema_init (struct compat_semaphore *sem, int val)
{
/*
 *    *sem = (struct compat_semaphore)__COMPAT_SEMAPHORE_INITIALIZER((*sem),val);
 *
 * i'd rather use the more flexible initialization above, but sadly
 * GCC 2.7.2.3 emits a bogus warning. EGCS doesnt. Oh well.
 */
	atomic_set(&sem->count, val);
	init_waitqueue_head(&sem->wait);
#if WAITQUEUE_DEBUG
	sem->__magic = (int)&sem->__magic;
#endif
}

static inline void compat_init_MUTEX (struct compat_semaphore *sem)
{
	compat_sema_init(sem, 1);
}

static inline void compat_init_MUTEX_LOCKED (struct compat_semaphore *sem)
{
	compat_sema_init(sem, 0);
}

asmlinkage void __compat_down(struct compat_semaphore * sem);
asmlinkage int  __compat_down_interruptible(struct compat_semaphore * sem);
asmlinkage int  __compat_down_trylock(struct compat_semaphore * sem);
asmlinkage void __compat_up(struct compat_semaphore * sem);

extern spinlock_t semaphore_wake_lock;

static inline void compat_down(struct semaphore * sem)
{
#if WAITQUEUE_DEBUG
	CHECK_MAGIC(sem->__magic);
#endif

	if (atomic_sub_return(1, &sem->count) < 0)
		__compat_down(sem);
}

static inline int compat_down_interruptible(struct semaphore * sem)
{
	int ret = 0;
#if WAITQUEUE_DEBUG
	CHECK_MAGIC(sem->__magic);
#endif

	if (atomic_sub_return(1, &sem->count) < 0)
		ret = __compat_down_interruptible(sem);
	return ret;
}

static inline int compat_down_trylock(struct semaphore * sem)
{
	int ret = 0;
#if WAITQUEUE_DEBUG
	CHECK_MAGIC(sem->__magic);
#endif

	if (atomic_sub_return(1, &sem->count) < 0)
		ret = __compat_down_trylock(sem);
	return ret;
}

/*
 * Note! This is subtle. We jump to wake people up only if
 * the semaphore was negative (== somebody was waiting on it).
 */
static inline void compat_up(struct semaphore * sem)
{
#if WAITQUEUE_DEBUG
	CHECK_MAGIC(sem->__magic);
#endif
	if (atomic_add_return(1, &sem->count) <= 0)
		__compat_up(sem);
}

extern int compat_sem_is_locked(struct compat_semaphore *sem);
#define compat_sema_count(sem) atomic_read(&(sem)->count)

#endif /* _XTENSA_SEMAPHORE_H */
