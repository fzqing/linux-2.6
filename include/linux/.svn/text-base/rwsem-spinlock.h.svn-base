/* rwsem-spinlock.h: fallback C implementation
 *
 * Copyright (c) 2001   David Howells (dhowells@redhat.com).
 * - Derived partially from ideas by Andrea Arcangeli <andrea@suse.de>
 * - Derived also from comments by Linus
 */

#ifndef _LINUX_RWSEM_SPINLOCK_H
#define _LINUX_RWSEM_SPINLOCK_H

#ifndef _LINUX_RWSEM_H
#error "please don't include linux/rwsem-spinlock.h directly, use linux/rwsem.h instead"
#endif

#include <linux/spinlock.h>
#include <linux/list.h>

#ifdef __KERNEL__

#include <linux/types.h>

struct rwsem_waiter;

/*
 * the rw-semaphore definition
 * - if activity is 0 then there are no active readers or writers
 * - if activity is +ve then that is the number of active readers
 * - if activity is -1 then there is one active writer
 * - if wait_list is not empty, then there are processes waiting for the semaphore
 */
struct compat_rw_semaphore {
	__s32			activity;
	spinlock_t		wait_lock;
	struct list_head	wait_list;
#if RWSEM_DEBUG
	int			debug;
#endif
};

/*
 * initialisation
 */
#if RWSEM_DEBUG
#define __RWSEM_DEBUG_INIT      , 0
#else
#define __RWSEM_DEBUG_INIT	/* */
#endif

#define __COMPAT_RWSEM_INITIALIZER(name) \
{ 0, SPIN_LOCK_UNLOCKED, LIST_HEAD_INIT((name).wait_list) __RWSEM_DEBUG_INIT }

#define COMPAT_DECLARE_RWSEM(name) \
	struct rw_semaphore name = __COMPAT_RWSEM_INITIALIZER(name)

extern void FASTCALL(compat_init_rwsem(struct compat_rw_semaphore *sem));
extern void FASTCALL(__down_read(struct compat_rw_semaphore *sem));
extern int FASTCALL(__down_read_trylock(struct compat_rw_semaphore *sem));
extern void FASTCALL(__down_write(struct compat_rw_semaphore *sem));
extern int FASTCALL(__down_write_trylock(struct compat_rw_semaphore *sem));
extern void FASTCALL(__up_read(struct compat_rw_semaphore *sem));
extern void FASTCALL(__up_write(struct compat_rw_semaphore *sem));
extern void FASTCALL(__downgrade_write(struct compat_rw_semaphore *sem));

#endif /* __KERNEL__ */
#endif /* _LINUX_RWSEM_SPINLOCK_H */
