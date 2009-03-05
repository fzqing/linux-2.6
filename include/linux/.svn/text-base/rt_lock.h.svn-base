#ifndef __LINUX_RT_LOCK_H
#define __LINUX_RT_LOCK_H

#include <linux/config.h>
#include <linux/list.h>

/*
 * These are the basic SMP spinlocks, allowing only a single CPU anywhere.
 * We use a generic definition on all architectures.
 */

#ifdef CONFIG_SMP
typedef struct {
	volatile unsigned long lock;
# ifdef CONFIG_DEBUG_SPINLOCK
# ifdef CONFIG_PPC32
	volatile unsigned long owner_pc;
	volatile unsigned long owner_cpu;
# else
	unsigned int magic;
# endif
# endif
# ifdef CONFIG_PREEMPT
	unsigned int break_lock;
# endif
} raw_spinlock_t;
#else
  typedef struct { } raw_spinlock_t;
# define __RAW_SPIN_LOCK_UNLOCKED { }
# define RAW_SPIN_LOCK_UNLOCKED (raw_spinlock_t) __RAW_SPIN_LOCK_UNLOCKED
#endif

#include <asm/atomic.h>

/*
 * Read-write spinlocks, allowing multiple readers
 * but only one writer.
 */
#ifdef CONFIG_SMP
typedef struct {
#if defined(CONFIG_PPC32) || defined(CONFIG_PPC64)
	volatile signed int lock;
#else
	volatile unsigned long lock;
#endif
# ifdef CONFIG_DEBUG_SPINLOCK
	unsigned magic;
# endif
# ifdef CONFIG_PREEMPT
	unsigned int break_lock;
# endif
} raw_rwlock_t;
#else
  typedef struct { } raw_rwlock_t;
# define __RAW_RW_LOCK_UNLOCKED { }
# define RAW_RW_LOCK_UNLOCKED (raw_rwlock_t) __RAW_RW_LOCK_UNLOCKED
#endif

#ifdef CONFIG_PREEMPT_RT

/*
 * This is the core locking object used by PREEMPT_RT.
 * This one handles all the logic necessary, the other locking
 * objects (spinlocks, rwlocks, semaphores and rw-semaphores)
 * all use this synchronization object internally:
 */
struct rt_mutex {
	raw_spinlock_t		wait_lock;
	struct list_head	wait_list;
	struct task_struct	*owner;
	int			owner_prio;
# ifdef CONFIG_RT_DEADLOCK_DETECT
	int			save_state;
	struct list_head	held_list;
	unsigned long		acquire_eip;
	char 			*name, *file;
	int			line;
# endif
};

/*
 * This is the control structure for tasks blocked on an
 * RT mutex:
 */
struct rt_mutex_waiter {
	struct rt_mutex *lock;
	struct list_head list;
	struct list_head pi_list;
	struct task_struct *task;

	unsigned long eip;	// for debugging
};

#ifdef CONFIG_RT_DEADLOCK_DETECT
# define __RT_MUTEX_INITIALIZER(lockname) \
	{ .wait_lock = RAW_SPIN_LOCK_UNLOCKED, \
	.wait_list = LIST_HEAD_INIT((lockname).wait_list), \
	.name = #lockname, .file = __FILE__, .line = __LINE__ }
#else
# define __RT_MUTEX_INITIALIZER(lockname) \
	{ .wait_lock = RAW_SPIN_LOCK_UNLOCKED, \
	   LIST_HEAD_INIT((lockname).wait_list) }
#endif
/*
 * RW-semaphores are an RT mutex plus a reader-depth count.
 *
 * Note that the semantics are different from the usual
 * Linux rw-sems, in PREEMPT_RT mode we do not allow
 * multiple readers to hold the lock at once, we only allow
 * a read-lock owner to read-lock recursively. This is
 * better for latency, makes the implementation inherently
 * fair and makes it simpler as well:
 */
struct rw_semaphore {
	struct rt_mutex		lock;
	int			read_depth;
};

/*
 * rwlocks - an RW semaphore plus lock-break field:
 */
typedef struct {
	struct rw_semaphore	lock;
	unsigned int		break_lock;
} rwlock_t;

# ifdef CONFIG_RT_DEADLOCK_DETECT
#  define __RW_LOCK_UNLOCKED \
	.wait_lock = __RAW_SPIN_LOCK_UNLOCKED, .save_state = 1, \
	.file = __FILE__, .line = __LINE__
#  define _RW_LOCK_UNLOCKED(lock) \
	(rwlock_t) { { { __RW_LOCK_UNLOCKED, .name = #lock } } }
#  define RW_LOCK_UNLOCKED \
	(rwlock_t) { { { __RW_LOCK_UNLOCKED } } }
# else
#  define RW_LOCK_UNLOCKED (rwlock_t) \
	{ { { .wait_lock = __RAW_SPIN_LOCK_UNLOCKED } } }
#  define _RW_LOCK_UNLOCKED(lock) RW_LOCK_UNLOCKED
# endif
#else /* !PREEMPT_RT */
  typedef raw_rwlock_t rwlock_t;
# define _RW_LOCK_UNLOCKED(lock)	RAW_RW_LOCK_UNLOCKED
# define RW_LOCK_UNLOCKED		RAW_RW_LOCK_UNLOCKED
#endif

#ifdef CONFIG_PREEMPT_RT

/*
 * spinlocks - an RT mutex plus lock-break field:
 */
typedef struct {
	struct rt_mutex lock;
	unsigned int break_lock;
} spinlock_t;

#ifdef CONFIG_RT_DEADLOCK_DETECT
# define __SPIN_LOCK_UNLOCKED \
	.wait_lock = __RAW_SPIN_LOCK_UNLOCKED, \
	.save_state = 1, .file = __FILE__, .line = __LINE__
# define _SPIN_LOCK_UNLOCKED(lock) \
	(spinlock_t) { { __SPIN_LOCK_UNLOCKED, .name = #lock } }
# define SPIN_LOCK_UNLOCKED \
	(spinlock_t) { { __SPIN_LOCK_UNLOCKED } }
#else
# define SPIN_LOCK_UNLOCKED \
	(spinlock_t) { { .wait_lock = __RAW_SPIN_LOCK_UNLOCKED } }
# define _SPIN_LOCK_UNLOCKED(lock) SPIN_LOCK_UNLOCKED
#endif
#else /* !PREEMPT_RT */
  typedef raw_spinlock_t spinlock_t;
# define _SPIN_LOCK_UNLOCKED(lock)	RAW_SPIN_LOCK_UNLOCKED
# define SPIN_LOCK_UNLOCKED		RAW_SPIN_LOCK_UNLOCKED
#endif


#ifdef CONFIG_PREEMPT_RT

/*
 * Semaphores - an RT-mutex plus the semaphore count:
 */
struct semaphore {
	atomic_t count;
	struct rt_mutex lock;
};

#define __MUTEX_INITIALIZER(name) \
        { .count = { 1 }, .lock = __RT_MUTEX_INITIALIZER(name.lock) }

#define DECLARE_MUTEX(name) \
struct semaphore name = \
	{ .count = { 1 }, .lock = __RT_MUTEX_INITIALIZER(name.lock) }

/*
 * DECLARE_MUTEX_LOCKED() is deprecated: very hard to initialize properly
 * and it also often signals abuse of semaphores. So we redirect it to
 * compat semaphores:
 */
#define DECLARE_MUTEX_LOCKED COMPAT_DECLARE_MUTEX_LOCKED

extern void FASTCALL(__sema_init(struct semaphore *sem, int val, char *name, char *file, int line));

#define rt_sema_init(sem, val) \
		__sema_init(sem, val, #sem, __FILE__, __LINE__)
	
extern void FASTCALL(__init_MUTEX(struct semaphore *sem, char *name, char *file, int line));
extern void FASTCALL(__init_MUTEX_LOCKED(struct semaphore *sem, char *name, char *file, int line));
#define rt_init_MUTEX(sem) \
		__init_MUTEX(sem, #sem, __FILE__, __LINE__)
#define rt_init_MUTEX_LOCKED(sem) \
		__init_MUTEX_LOCKED(sem, #sem, __FILE__, __LINE__)
extern void FASTCALL(rt_down(struct semaphore * sem));
extern int FASTCALL(rt_down_interruptible(struct semaphore * sem));
extern int FASTCALL(rt_down_trylock(struct semaphore * sem));
extern void FASTCALL(rt_up(struct semaphore * sem));
extern int FASTCALL(rt_sem_is_locked(struct semaphore *sem));
extern int FASTCALL(rt_sema_count(struct semaphore * sem));

extern int __bad_func_type(void);

#undef TYPE_EQUAL
#define TYPE_EQUAL(var, type) \
		__builtin_types_compatible_p(typeof(var), type *)

#define PICK_FUNC_1ARG(type1, type2, func1, func2, arg)			\
do {									\
	if (TYPE_EQUAL((arg), type1))					\
		func1((type1 *)(arg));					\
	else if (TYPE_EQUAL((arg), type2))				\
		func2((type2 *)(arg));					\
	else __bad_func_type();						\
} while (0)

#define PICK_FUNC_1ARG_RET(type1, type2, func1, func2, arg)		\
({									\
	int __ret;							\
									\
	if (TYPE_EQUAL((arg), type1))					\
		__ret = func1((type1 *)(arg));				\
	else if (TYPE_EQUAL((arg), type2))				\
		__ret = func2((type2 *)(arg));				\
	else __ret = __bad_func_type();					\
									\
	__ret;								\
})

#define PICK_FUNC_2ARG(type1, type2, func1, func2, arg0, arg1)		\
do {									\
	if (TYPE_EQUAL((arg0), type1))					\
		func1((type1 *)(arg0), arg1);				\
	else if (TYPE_EQUAL((arg0), type2))				\
		func2((type2 *)(arg0), arg1);				\
	else __bad_func_type();						\
} while (0)
  
#define sema_init(sem, val) \
	PICK_FUNC_2ARG(struct compat_semaphore, struct semaphore, \
		compat_sema_init, rt_sema_init, sem, val)
  
#define init_MUTEX(sem) \
	PICK_FUNC_1ARG(struct compat_semaphore, struct semaphore, \
		compat_init_MUTEX, rt_init_MUTEX, sem)
  
#define down(sem) \
	PICK_FUNC_1ARG(struct compat_semaphore, struct semaphore, \
		compat_down, rt_down, sem)
  
#define down_interruptible(sem) \
	PICK_FUNC_1ARG_RET(struct compat_semaphore, struct semaphore, \
		compat_down_interruptible, rt_down_interruptible, sem)
  
#define down_trylock(sem) \
	PICK_FUNC_1ARG_RET(struct compat_semaphore, struct semaphore, \
		compat_down_trylock, rt_down_trylock, sem)
  
#define up(sem) \
	PICK_FUNC_1ARG(struct compat_semaphore, struct semaphore, \
		compat_up, rt_up, sem)
  
#define sem_is_locked(sem) \
	PICK_FUNC_1ARG_RET(struct compat_semaphore, struct semaphore, \
		compat_sem_is_locked, rt_sem_is_locked, sem)

#define init_MUTEX_LOCKED(sem) \
	PICK_FUNC_1ARG(struct compat_semaphore, struct semaphore, \
		compat_init_MUTEX_LOCKED, rt_init_MUTEX_LOCKED, sem)

 /*
  * rwsems:
  */

#define __RWSEM_INITIALIZER(lockname) \
	{ .lock = __RT_MUTEX_INITIALIZER(lockname.lock) }

#define DECLARE_RWSEM(lockname) \
	struct rw_semaphore lockname = __RWSEM_INITIALIZER(lockname)

extern void FASTCALL(__init_rwsem(struct rw_semaphore *rwsem, int save_state,
					char *name, char *file, int line));

#define rt_init_rwsem(sem) \
	__init_rwsem(sem, 0, #sem, __FILE__, __LINE__)

extern void FASTCALL(rt_down_read(struct rw_semaphore *rwsem));
extern int FASTCALL(rt_down_read_trylock(struct rw_semaphore *rwsem));
extern void FASTCALL(rt_down_write(struct rw_semaphore *rwsem));
extern int FASTCALL(rt_down_write_interruptible(struct rw_semaphore *rwsem));
extern int FASTCALL(rt_down_write_trylock(struct rw_semaphore *rwsem));
extern void FASTCALL(rt_up_read(struct rw_semaphore *rwsem));
extern void FASTCALL(rt_up_write(struct rw_semaphore *rwsem));
extern void FASTCALL(rt_downgrade_write(struct rw_semaphore *rwsem));
extern int FASTCALL(rt_rwsem_is_locked(struct rw_semaphore *rwsem));

#define init_rwsem(rwsem) \
	PICK_FUNC_1ARG(struct compat_rw_semaphore, struct rw_semaphore, \
		compat_init_rwsem, rt_init_rwsem, rwsem)

#define down_read(rwsem) \
	PICK_FUNC_1ARG(struct compat_rw_semaphore, struct rw_semaphore, \
		compat_down_read, rt_down_read, rwsem)

#define down_read_trylock(rwsem) \
	PICK_FUNC_1ARG_RET(struct compat_rw_semaphore, struct rw_semaphore, \
		compat_down_read_trylock, rt_down_read_trylock, rwsem)

#define down_write(rwsem) \
	PICK_FUNC_1ARG(struct compat_rw_semaphore, struct rw_semaphore, \
		compat_down_write, rt_down_write, rwsem)

#define down_write_trylock(rwsem) \
	PICK_FUNC_1ARG_RET(struct compat_rw_semaphore, struct rw_semaphore, \
		compat_down_write_trylock, rt_down_write_trylock, rwsem)

#define up_read(rwsem) \
	PICK_FUNC_1ARG(struct compat_rw_semaphore, struct rw_semaphore, \
		compat_up_read, rt_up_read, rwsem)

#define up_write(rwsem) \
	PICK_FUNC_1ARG(struct compat_rw_semaphore, struct rw_semaphore, \
		compat_up_write, rt_up_write, rwsem)

#define downgrade_write(rwsem) \
	PICK_FUNC_1ARG(struct compat_rw_semaphore, struct rw_semaphore, \
		compat_downgrade_write, rt_downgrade_write, rwsem)

#define rwsem_is_locked(rwsem) \
	PICK_FUNC_1ARG_RET(struct compat_rw_semaphore, struct rw_semaphore, \
		compat_rwsem_is_locked, rt_rwsem_is_locked, rwsem)

#endif /* CONFIG_PREEMPT_RT */

#endif

