/*
 * kernel/rt.c
 *
 * Real-Time Preemption Support
 *
 * started by Ingo Molnar:
 *
 *  Copyright (C) 2004 Red Hat, Inc., Ingo Molnar <mingo@redhat.com>
 *
 * lock debugging, locking tree, deadlock detection:
 *
 *  Copyright (C) 2004, LynuxWorks, Inc., Igor Manyilov, Bill Huey
 *  Released under the General Public License (GPL).
 *
 * Includes portions of the generic R/W semaphore implementation from:
 *
 *  Copyright (c) 2001   David Howells (dhowells@redhat.com).
 *  - Derived partially from idea by Andrea Arcangeli <andrea@suse.de>
 *  - Derived also from comments by Linus
 */
#include <linux/config.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/kallsyms.h>
#include <linux/syscalls.h>
#include <linux/interrupt.h>

/*
 * This flag is good for debugging the PI code - it makes all tasks
 * in the system fall under PI handling. Normally only SCHED_FIFO/RR
 * tasks are PI-handled:
 */
//#define ALL_TASKS_PI

/*
 * We need a global lock for priority inheritance handling.
 * This is only for the slow path, but still, we might want
 * to optimize it later to be more scalable.
 */
static __cacheline_aligned_in_smp raw_spinlock_t pi_lock =
						RAW_SPIN_LOCK_UNLOCKED;

#ifdef CONFIG_RT_DEADLOCK_DETECT
/*
 * We need a global lock when we walk through the multi-process
 * lock tree...
 */
static raw_spinlock_t trace_lock = RAW_SPIN_LOCK_UNLOCKED;

static LIST_HEAD(held_locks);

/*
 * deadlock detection flag. We turn it off when we detect
 * the first problem because we dont want to recurse back
 * into the tracing code when doing error printk or
 * executing a BUG():
 */
static int trace_on = 1;

void deadlock_trace_off(void)
{
	trace_on = 0;
}

#define trace_lock_irq(lock)			\
	do {					\
		local_irq_disable();		\
		if (trace_on)			\
			spin_lock(lock);	\
	} while (0)

#define trace_unlock(lock)			\
	do {					\
		if (trace_on)			\
			spin_unlock(lock);	\
	} while (0)

#define trace_unlock_irq(lock)			\
	do {					\
		if (trace_on)			\
			spin_unlock(lock);	\
		local_irq_enable();		\
		preempt_check_resched();	\
	} while (0)

#define trace_lock_irqsave(lock, flags)		\
	do {					\
		local_irq_save(flags);		\
		if (trace_on)			\
			spin_lock(lock);	\
	} while (0)

#define trace_unlock_irqrestore(lock, flags)	\
	do {					\
		if (trace_on)			\
			spin_unlock(lock);	\
		local_irq_restore(flags);	\
		preempt_check_resched();	\
	} while (0)

#define TRACE_OFF()				\
do {						\
	if (trace_on) {				\
		trace_on = 0;			\
		console_verbose();		\
		spin_unlock(&trace_lock);	\
	}					\
} while (0)

#define TRACE_BUG()				\
do {						\
	TRACE_OFF();				\
	BUG();					\
} while (0)

#define TRACE_WARN_ON(c)			\
do {						\
	if (c) {				\
		TRACE_OFF();			\
		WARN_ON(1);			\
	}					\
} while (0)

/*
 * Should trigger when you don't initialize your mutex
 * before you use it.
 */
#define TRACE_WARN_UNINITIALIZED(c)	WARN_ON(c)

#else
# define trace_lock_irq(lock)			local_irq_disable()
# define trace_lock_irqsave(lock, flags)	local_irq_save(flags)
# define trace_unlock(lock)			do { } while (0)

# define trace_unlock_irq(lock) \
	do { local_irq_enable(); preempt_check_resched(); } while (0)

# define trace_unlock_irqrestore(lock, flags) \
	do { local_irq_restore(flags); preempt_check_resched(); } while (0)

# define TRACE_BUG()				do { } while (0)
# define TRACE_WARN_ON(c)			do { } while (0)
#define TRACE_WARN_UNINITIALIZED(c)		do { } while (0)
# define TRACE_OFF()				do { } while (0)
#endif /* CONFIG_RT_DEADLOCK_DETECT */

#define TRACE_BUG_ON(c) do { if (c) TRACE_BUG(); } while (0)

/*
 * Unlock these on crash:
 */
void zap_rt_locks(void)
{
	spin_lock_init(&pi_lock);
#ifdef CONFIG_RT_DEADLOCK_DETECT
	spin_lock_init(&trace_lock);
#endif
}

#ifdef CONFIG_RT_DEADLOCK_DETECT

static void printk_task(struct task_struct *p)
{
	if (p)
		printk("%16s:%5d [%p, %3d]", p->comm, p->pid, p, p->prio);
	else
		printk("<none>");
}

static void printk_task_short(struct task_struct *p)
{
	if (p)
		printk("%s/%d [%p, %3d]", p->comm, p->pid, p, p->prio);
	else
		printk("<none>");
}

static void printk_lock(struct rt_mutex *lock, int print_owner)
{
	if (lock->name)
		printk(" [%p] {%s}\n",
			lock, lock->name);
	else
		printk(" [%p] {%s:%d}\n",
			lock, lock->file, lock->line);

	if (print_owner && lock->owner) {
		printk(".. held by:  ");
		printk_task(lock->owner);
		printk("\n");
	}
	if (lock->owner) {
		printk("... acquired at:  ");
		print_symbol("%s\n", lock->acquire_eip);
	}
}

static void printk_waiter(struct rt_mutex_waiter *w)
{
	printk("-------------------------\n");
	printk("| waiter struct %p:\n", w);
	printk("| w->task:\n");
	printk_task(w->task);
	printk("\n| lock:\n");
	printk_lock(w->lock, 1);
	printk("| blocked at:  ");
	print_symbol("%s\n", w->eip);
	printk("-------------------------\n");
}

static void show_task_locks(struct task_struct *p)
{
	switch (p->state) {
	case TASK_RUNNING:		printk("R"); break;
	case TASK_RUNNING_MUTEX:	printk("M"); break;
	case TASK_INTERRUPTIBLE:	printk("S"); break;
	case TASK_UNINTERRUPTIBLE:	printk("D"); break;
	case TASK_STOPPED:		printk("T"); break;
	case EXIT_ZOMBIE:		printk("Z"); break;
	case EXIT_DEAD:			printk("X"); break;
	default:			printk("?"); break;
	}
	printk_task(p);
	if (p->blocked_on) {
		struct rt_mutex *lock = p->blocked_on->lock;

		printk(" blocked on:");
		printk_lock(lock, 1);
	} else
		printk(" (not blocked)\n");
}

static void show_held_locks(struct task_struct *filter)
{
	struct list_head *curr, *cursor = NULL;
	struct rt_mutex *lock;
	struct task_struct *p;
	unsigned long flags;
	int count = 0;

	printk("\n");
	if (filter) {
		printk("------------------------------\n");
		printk("| showing all locks held by: |  (");
		printk_task_short(filter);
		printk("):\n");
		printk("------------------------------\n");
	} else {
		printk("---------------------------\n");
		printk("| showing all locks held: |\n");
		printk("---------------------------\n");
	}

	/*
	 * Play safe and acquire the global trace lock. We
	 * cannot printk with that lock held so we iterate
	 * very carefully:
	 */
next:
	trace_lock_irqsave(&trace_lock, flags);
	list_for_each(curr, &held_locks) {
		if (cursor && curr != cursor)
			continue;
		lock = list_entry(curr, struct rt_mutex, held_list);
		p = lock->owner;
		if (filter && (p != filter))
			continue;
		count++;
		cursor = curr->next;
		trace_unlock_irqrestore(&trace_lock, flags);

		printk("\n#%03d:            ", count);
		printk_lock(lock, filter ? 0 : 1);
		goto next;
	}
	trace_unlock_irqrestore(&trace_lock, flags);
}

void show_all_locks(void)
{
	struct task_struct *g, *p;
	int count = 10;
	int unlock = 1;

	printk("\nshowing all tasks:\n");

	/*
	 * Here we try to get the tasklist_lock as hard as possible,
	 * if not successful after 2 seconds we ignore it (but keep
	 * trying). This is to enable a debug printout even if a
	 * tasklist_lock-holding task deadlocks or crashes.
	 */
retry:
	if (!read_trylock(&tasklist_lock)) {
		if (count == 10)
			printk("hm, tasklist_lock locked, retrying... ");
		if (count) {
			count--;
			printk(" #%d", 10-count);
			mdelay(200);
			goto retry;
		}
		printk(" ignoring it.\n");
		unlock = 0;
	}
	if (count != 10)
		printk(" locked it.\n");

	do_each_thread(g, p) {
		show_task_locks(p);
		if (!unlock)
			if (read_trylock(&tasklist_lock))
				unlock = 1;
	} while_each_thread(g, p);

	show_held_locks(NULL);
	printk("=============================================\n\n");

	if (unlock)
		read_unlock(&tasklist_lock);
}

static int check_deadlock(struct rt_mutex *lock, int depth,
			  unsigned long eip)
{
	struct rt_mutex *lockblk;
	struct task_struct *task;

	if (!trace_on)
		return 0;
	/*
	 * Special-case: the BKL self-releases at schedule()
	 * time so it can never deadlock:
	 */
	if (lock == &kernel_sem.lock)
		return 0;
	task = lock->owner;
	if (!task)
		return 0;
	lockblk = NULL;
	if (task->blocked_on)
		lockblk = task->blocked_on->lock;
	if (current == task) {
		TRACE_OFF();
		if (depth)
			return 1;
		printk("\n==========================================\n");
		printk(  "[ BUG: lock recursion deadlock detected! |\n");
		printk(  "------------------------------------------\n");
		printk("already locked: ");
		printk_lock(lock, 1);
		show_held_locks(task);
		printk("\n-{current task's backtrace}----------------->\n");
		dump_stack();
		show_all_locks();
		printk("[ turning off deadlock detection. Please report this trace. ]\n\n");
		local_irq_disable();
		return 0;
	}
	/*
	 * Skip the BKL:
	 */
	if (lockblk == &kernel_sem.lock)
		return 0;
	/*
	 * Ugh, something corrupted the lock data structure?
	 */
	if (depth > 30) {
		TRACE_OFF();
		printk("\n===========================================\n");
		printk(  "[ BUG: infinite lock dependency detected!? |\n");
		printk(  "-------------------------------------------\n");
		goto print_it;
	}
	if (lockblk && check_deadlock(lockblk, depth+1, eip)) {
		printk("\n============================================\n");
		printk(  "[ BUG: circular locking deadlock detected! ]\n");
		printk(  "--------------------------------------------\n");
print_it:
		printk("%s/%d is deadlocking current task %s/%d\n\n",
			task->comm, task->pid, current->comm, current->pid);
		printk("\n1) %s/%d is trying to acquire this lock:\n",
			current->comm, current->pid);
		printk_lock(lock, 1);

		printk("... trying at:   ");
		print_symbol("%s\n", eip);

		printk("\n2) %s/%d is blocked on this lock:\n",
			task->comm, task->pid);
		printk_lock(lockblk, 1);

		show_held_locks(current);
		show_held_locks(task);

		printk("\n%s/%d's [blocked] stackdump:\n\n", task->comm, task->pid);
		show_stack(task, NULL);
		printk("\n%s/%d's [current] stackdump:\n\n",
			current->comm, current->pid);
		dump_stack();
		show_all_locks();
		printk("[ turning off deadlock detection. Please report this trace. ]\n\n");
		local_irq_disable();
		return 0;
	}
	return 0;
}

void check_no_held_locks(struct task_struct *task)
{
	struct list_head *curr, *next, *cursor = NULL;
	struct rt_mutex *lock;
	struct rt_mutex_waiter *w;
	struct task_struct *p;
	unsigned long flags;

	if (!trace_on)
		return;
restart:
	trace_lock_irqsave(&trace_lock, flags);
	list_for_each_safe(curr, next, &held_locks) {
		if (cursor && curr != cursor)
			continue;
		lock = list_entry(curr, struct rt_mutex, held_list);
		p = lock->owner;
		if (p != task)
			continue;
		cursor = next;
		list_del_init(curr);
		trace_unlock_irqrestore(&trace_lock, flags);

		if (lock == &kernel_sem.lock) {
			printk("BUG: %s/%d, BKL held at task exit time!\n",
				current->comm, current->pid);
			printk("BKL acquired at: ");
			print_symbol("%s\n",
				(unsigned long) current->last_kernel_lock);
		} else
			printk("BUG: %s/%d, lock held at task exit time!\n",
				current->comm, current->pid);
		printk_lock(lock, 1);
		if (lock->owner != task)
			printk("exiting task is not even the owner??\n");
		goto restart;
	}
	spin_lock(&pi_lock);
	list_for_each(curr, &task->pi_waiters) {
		w = list_entry(curr, struct rt_mutex_waiter, pi_list);
		TRACE_OFF();
		spin_unlock(&pi_lock);
		trace_unlock_irqrestore(&trace_lock, flags);

		printk("hm, PI interest held at exit time? Task:\n");
		printk_task(task);
		printk_waiter(w);
		return;
	}
	spin_unlock(&pi_lock);
	trace_unlock_irqrestore(&trace_lock, flags);
}

#endif

#if defined(ALL_TASKS_PI) && defined(CONFIG_RT_DEADLOCK_DETECT)

static void
check_pi_list_present(struct rt_mutex *lock, struct rt_mutex_waiter *waiter,
		      struct task_struct *old_owner)
{
	struct rt_mutex_waiter *w;
	struct list_head *curr;

	TRACE_WARN_ON(list_empty(&waiter->pi_list));
	TRACE_WARN_ON(lock->owner);

	list_for_each(curr, &old_owner->pi_waiters) {
		w = list_entry(curr, struct rt_mutex_waiter, pi_list);
		if (w == waiter)
			goto ok;
	}
	TRACE_WARN_ON(1);
ok:
}

static void
check_pi_list_empty(struct rt_mutex *lock, struct task_struct *old_owner)
{
	struct rt_mutex_waiter *w;
	struct list_head *curr;

	list_for_each(curr, &old_owner->pi_waiters) {
		w = list_entry(curr, struct rt_mutex_waiter, pi_list);
		if (w->lock == lock) {
			TRACE_OFF();
			printk("hm, PI interest but no waiter? Old owner:\n");
			printk_waiter(w);
			printk("\n");
			TRACE_WARN_ON(1);
			return;
		}
	}
}

#else

static inline void
check_pi_list_present(struct rt_mutex *lock, struct rt_mutex_waiter *waiter,
		      struct task_struct *old_owner)
{
}

static inline void
check_pi_list_empty(struct rt_mutex *lock, struct task_struct *old_owner)
{
}

#endif

/*
 * Move PI waiters of this lock to the new owner:
 */
static void
change_owner(struct rt_mutex *lock, struct task_struct *old_owner,
		   struct task_struct *new_owner)
{
	struct list_head *curr, *next;
	struct rt_mutex_waiter *w;
	int requeued = 0, sum = 0;

	if (old_owner == new_owner)
		return;
	list_for_each_safe(curr, next, &old_owner->pi_waiters) {
		w = list_entry(curr, struct rt_mutex_waiter, pi_list);
		if (w->lock == lock) {
			list_del_init(curr);
			list_add_tail(curr, &new_owner->pi_waiters);
			requeued++;
		}
		sum++;
	}
	trace_special(sum, requeued, 0);
}

int pi_walk, pi_null, pi_prio;

static void pi_setprio(struct rt_mutex *lock, struct task_struct *p, int prio)
{
	if (unlikely(!p->pid)) {
		pi_null++;
		return;
	}

#ifdef CONFIG_RT_DEADLOCK_DETECT
	pi_prio++;
	if (p->policy != SCHED_NORMAL && prio > mutex_getprio(p)) {
		TRACE_OFF();

		printk("huh? (%d->%d??)\n", p->prio, prio);
		printk("owner:\n");
		printk_task(p);
		printk("\ncurrent:\n");
		printk_task(current);
		printk("\nlock:\n");
		printk_lock(lock, 1);
		dump_stack();
		local_irq_disable();
	}
#endif
	/*
	 * If the task is blocked on some other task then boost that
	 * other task (or tasks) too:
	 */
	for (;;) {
		struct rt_mutex_waiter *w = p->blocked_on;
		int was_rt = rt_task(p);

		mutex_setprio(p, prio);
		if (!w)
			break;
		/*
		 * If the task is blocked on a lock, and we just made
		 * it RT, then register the task in the PI list and
		 * requeue it to the head of the wait list:
		 */
		lock = w->lock;
		TRACE_BUG_ON(!lock);
		TRACE_BUG_ON(!lock->owner);
		if (rt_task(p) && list_empty(&w->pi_list)) {
			TRACE_BUG_ON(was_rt);
			list_add_tail(&w->pi_list, &lock->owner->pi_waiters);
			list_del(&w->list);
			list_add(&w->list, &lock->wait_list);
		}
		/*
		 * If the task is blocked on a lock, and we just restored
		 * it from RT to non-RT then unregister the task from
		 * the PI list and requeue it to the tail of the wait
		 * list:
		 *
		 * (TODO: this can be unfair to SCHED_NORMAL tasks if they
		 *        get PI handled.)
		 */
		if (!rt_task(p) && !list_empty(&w->pi_list)) {
			TRACE_BUG_ON(!was_rt);
			list_del_init(&w->pi_list);
			list_del(&w->list);
			list_add_tail(&w->list, &lock->wait_list);
		}

		pi_walk++;

		p = lock->owner;
		TRACE_BUG_ON(!p);
		/*
		 * If the dependee is already higher-prio then
		 * no need to boost it, and all further tasks down
		 * the dependency chain are already boosted:
		 */
		if (p->prio <= prio)
			break;
	}
}

static void
task_blocks_on_lock(struct rt_mutex_waiter *waiter, struct task_struct *task,
		   struct rt_mutex *lock, unsigned long eip)
{
#ifdef CONFIG_RT_DEADLOCK_DETECT
	check_deadlock(lock, 0, eip);
	/* mark the current thread as blocked on the lock */
	waiter->eip = eip;
#endif
	task->blocked_on = waiter;
	waiter->lock = lock;
	waiter->task = task;
	INIT_LIST_HEAD(&waiter->pi_list);
	/*
	 * Add SCHED_NORMAL tasks to the end of the waitqueue (FIFO):
	 */
#ifndef ALL_TASKS_PI
	if (!rt_task(task)) {
		list_add_tail(&waiter->list, &lock->wait_list);
		return;
	}
#endif
	spin_lock(&pi_lock);
	list_add_tail(&waiter->pi_list, &lock->owner->pi_waiters);
	/*
	 * Add RT tasks to the head:
	 */
	list_add(&waiter->list, &lock->wait_list);
	/*
	 * If the waiter has higher priority than the owner
	 * then temporarily boost the owner:
	 */
	if (task->prio < lock->owner->prio)
		pi_setprio(lock, lock->owner, task->prio);
	spin_unlock(&pi_lock);
}

/*
 * initialise the lock:
 */
static void __init_rt_mutex(struct rt_mutex *lock, int save_state,
				char *name, char *file, int line)
{
	lock->owner = NULL;
	spin_lock_init(&lock->wait_lock);
	INIT_LIST_HEAD(&lock->wait_list);
#ifdef CONFIG_RT_DEADLOCK_DETECT
	lock->save_state = save_state;
	INIT_LIST_HEAD(&lock->held_list);
	lock->name = name;
	lock->file = file;
	lock->line = line;
#endif
}

void fastcall __init_rwsem(struct rw_semaphore *rwsem, int save_state,
			char *name, char *file, int line)
{
	__init_rt_mutex(&rwsem->lock, save_state, name, file, line);
	rwsem->read_depth = 0;
}
EXPORT_SYMBOL(__init_rwsem);

static void set_new_owner(struct rt_mutex *lock, struct task_struct *old_owner,
			struct task_struct *new_owner, unsigned long eip)
{
	if (new_owner)
		trace_special_pid(new_owner->pid, new_owner->prio, 0);
	if (old_owner)
		change_owner(lock, old_owner, new_owner);
	lock->owner = new_owner;
	lock->owner_prio = new_owner->prio;
#ifdef CONFIG_RT_DEADLOCK_DETECT
	TRACE_WARN_ON(!list_empty(&lock->held_list));
	list_add_tail(&lock->held_list, &held_locks);
	lock->acquire_eip = eip;
#endif
}

/*
 * handle the lock release when processes blocked on it that can now run
 * - the spinlock must be held by the caller
 */
static inline struct task_struct * pick_new_owner(struct rt_mutex *lock,
		struct task_struct *old_owner, int save_state,
		unsigned long eip)
{
	struct rt_mutex_waiter *w, *waiter = NULL;
	struct task_struct *new_owner;
	struct list_head *curr;

	/*
	 * Get the highest prio one:
	 *
	 * (same-prio RT tasks go FIFO)
	 */
	list_for_each(curr, &lock->wait_list) {
		w = list_entry(curr, struct rt_mutex_waiter, list);
		trace_special_pid(w->task->pid, w->task->prio, 0);
		/*
		 * Break out upon meeting the first non-RT-prio
		 * task - we inserted them to the tail, so if we
	 	 * see the first one the rest is SCHED_NORMAL too:
	 	 */
		if (!rt_task(w->task))
			break;
		if (!waiter || w->task->prio <= waiter->task->prio)
			waiter = w;
	}

	/*
	 * If no RT waiter then pick the first one:
	 */
	if (!waiter)
		waiter = list_entry(lock->wait_list.next,
					struct rt_mutex_waiter, list);
	trace_special_pid(waiter->task->pid, waiter->task->prio, 0);

#ifdef ALL_TASKS_PI
	check_pi_list_present(lock, waiter, old_owner);
#endif
	new_owner = waiter->task;
	list_del_init(&waiter->list);

	list_del_init(&waiter->pi_list);

	set_new_owner(lock, old_owner, new_owner, waiter->eip);
	/* Don't touch waiter after ->task has been NULLed */
	mb();
	waiter->task = NULL;
	new_owner->blocked_on = NULL;
	TRACE_WARN_ON(save_state != lock->save_state);

	return new_owner;
}

static inline void init_lists(struct rt_mutex *lock)
{
	// we have to do this until the static initializers get fixed:
	if (!lock->wait_list.prev && !lock->wait_list.next)
		INIT_LIST_HEAD(&lock->wait_list);
#ifdef CONFIG_RT_DEADLOCK_DETECT
	if (!lock->held_list.prev && !lock->held_list.next)
		INIT_LIST_HEAD(&lock->held_list);
#endif
}

/*
 * lock it semaphore-style: no worries about missed wakeups.
 */
static void __sched __down(struct rt_mutex *lock, unsigned long eip)
{
	struct task_struct *task = current;
	unsigned long flags, nosched_flag;
	struct rt_mutex_waiter waiter;

	trace_lock_irqsave(&trace_lock, flags);
	TRACE_BUG_ON(!irqs_disabled());
	spin_lock(&lock->wait_lock);

	init_lists(lock);

	if (!lock->owner) {
		/* granted */
		TRACE_WARN_ON(!list_empty(&lock->wait_list));
		spin_lock(&pi_lock);
		set_new_owner(lock, NULL, task, eip);
		spin_unlock(&pi_lock);
		spin_unlock(&lock->wait_lock);
		trace_unlock_irqrestore(&trace_lock, flags);

		return;
	}

	set_task_state(task, TASK_UNINTERRUPTIBLE);

	task_blocks_on_lock(&waiter, task, lock, eip);

	TRACE_BUG_ON(!irqs_disabled());
	/* we don't need to touch the lock struct anymore */
	spin_unlock(&lock->wait_lock);
	trace_unlock_irqrestore(&trace_lock, flags);

	might_sleep();

	nosched_flag = current->flags & PF_NOSCHED;

	current->flags &= ~PF_NOSCHED;

	/* wait to be given the lock */
	for (;;) {
		if (!waiter.task)
			break;
		schedule();
		set_task_state(task, TASK_UNINTERRUPTIBLE);
	}
	current->flags |= nosched_flag;
	task->state = TASK_RUNNING;
}

/*
 * get a write lock on the rw-semaphore
 */
void fastcall __sched rt_down_write(struct rw_semaphore *rwsem)
{
	__down(&rwsem->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(rt_down_write);

/*
 * get a read lock on the rw-semaphore
 */
void fastcall __sched rt_down_read(struct rw_semaphore *rwsem)
{
	/*
	 * Read locks within the write lock succeed.
	 */
	if (rwsem->lock.owner == current) {
		rwsem->read_depth++;
		return;
	}
	return __down(&rwsem->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(rt_down_read);

/*
 * lock it mutex-style: this variant is very careful not to
 * miss any non-mutex wakeups.
 *
 * The wakeup side uses wake_up_process_mutex, which, combined with
 * the xchg code of this function is a transparent sleep/wakeup
 * mechanism nested within any existing sleep/wakeup mechanism. This
 * enables the seemless use of arbitrary (blocking) spinlocks within
 * sleep/wakeup event loops.
 */
static void __sched __down_mutex(struct rt_mutex *lock, unsigned long eip)
{
	unsigned long state, saved_state, nosched_flag;
	struct task_struct *task = current;
	struct rt_mutex_waiter waiter;
	unsigned long flags;
	int got_wakeup = 0;

	trace_lock_irqsave(&trace_lock, flags);
	TRACE_BUG_ON(!irqs_disabled());
	spin_lock(&lock->wait_lock);

	init_lists(lock);

	if (!lock->owner) {
		/* granted */
		TRACE_WARN_ON(!list_empty(&lock->wait_list));
		spin_lock(&pi_lock);
		set_new_owner(lock, NULL, task, eip);
		spin_unlock(&pi_lock);
		spin_unlock(&lock->wait_lock);
		trace_unlock_irqrestore(&trace_lock, flags);

		return;
	}

	task_blocks_on_lock(&waiter, task, lock, eip);

	TRACE_BUG_ON(!irqs_disabled());
	/*
	 * Here we save whatever state the task was in originally,
	 * we'll restore it at the end of the function and we'll
	 * take any intermediate wakeup into account as well,
	 * independently of the mutex sleep/wakeup mechanism:
	 */
	saved_state = xchg(&task->state, TASK_UNINTERRUPTIBLE);

	/* we don't need to touch the lock struct anymore */
	spin_unlock(&lock->wait_lock);
	trace_unlock(&trace_lock);

	/*
	 * TODO: check 'flags' for the IRQ bit here - it is illegal to
	 * call down() from an IRQs-off section that results in
	 * an actual reschedule.
	 */

	nosched_flag = current->flags & PF_NOSCHED;

	current->flags &= ~PF_NOSCHED;

	/* wait to be given the lock */
	for (;;) {
		unsigned long saved_flags = current->flags & PF_NOSCHED;

		if (!waiter.task)
			break;
		local_irq_enable();
		current->flags &= ~PF_NOSCHED;
		schedule();
		current->flags |= saved_flags;
		local_irq_disable();
		state = xchg(&task->state, TASK_UNINTERRUPTIBLE);
		if (state == TASK_RUNNING)
			got_wakeup = 1;
	}
	/*
	 * Only set the task's state to TASK_RUNNING if it got
	 * a non-mutex wakeup. We keep the original state otherwise.
	 * A mutex wakeup changes the task's state to TASK_RUNNING_MUTEX,
	 * not TASK_RUNNING - hence we can differenciate between the two
	 * cases:
	 */
	state = xchg(&task->state, saved_state);
	if (state == TASK_RUNNING)
		got_wakeup = 1;
	if (got_wakeup)
		task->state = TASK_RUNNING;
	local_irq_enable();
	preempt_check_resched();

	current->flags |= nosched_flag;
}

/*
 * TODO: push this into __down_mutex()
 *
 * BKL users expect the BKL to be held across spinlock/rwlock-acquire.
 * Save and clear it, this will cause the scheduler to not drop the
 * BKL semaphore if we end up scheduling:
 */
#define SAVE_BKL(ACTION)					\
{								\
	struct task_struct *task = current;			\
	unsigned int saved_lock_depth;				\
								\
	saved_lock_depth = task->lock_depth;			\
	task->lock_depth = -1;					\
								\
	might_sleep();						\
	ACTION;							\
								\
	task->lock_depth = saved_lock_depth;			\
}


static void __sched down_write_mutex(struct rw_semaphore *rwsem,
					unsigned long eip)
{
	SAVE_BKL(__down_mutex(&rwsem->lock, eip));
}

static void __sched down_read_mutex(struct rw_semaphore *rwsem,
					unsigned long eip)
{
	/*
	 * Read locks within the write lock succeed.
	 */
	if (rwsem->lock.owner == current) {
		rwsem->read_depth++;
		return;
	}
	SAVE_BKL(__down_mutex(&rwsem->lock, eip));
}

/*
 * get a lock - interruptible
 */
static int __sched __down_interruptible(struct rt_mutex *lock,
					unsigned long eip)
{
	struct task_struct *task = current;
	unsigned long flags, nosched_flag;
	struct rt_mutex_waiter waiter;
	int ret;

	trace_lock_irqsave(&trace_lock, flags);
	TRACE_BUG_ON(!irqs_disabled());
	spin_lock(&lock->wait_lock);

	init_lists(lock);

	if (!lock->owner) {
		/* granted */
		TRACE_WARN_ON(!list_empty(&lock->wait_list));
		spin_lock(&pi_lock);
		set_new_owner(lock, NULL, task, eip);
		spin_unlock(&pi_lock);
		spin_unlock(&lock->wait_lock);
		trace_unlock_irqrestore(&trace_lock, flags);

		return 0;
	}

	set_task_state(task, TASK_INTERRUPTIBLE);

	task_blocks_on_lock(&waiter, task, lock, eip);

	TRACE_BUG_ON(!irqs_disabled());
	/* we don't need to touch the lock struct anymore */
	spin_unlock(&lock->wait_lock);
	trace_unlock_irqrestore(&trace_lock, flags);

	might_sleep();

	nosched_flag = current->flags & PF_NOSCHED;

	current->flags &= ~PF_NOSCHED;

	ret = 0;
	/* wait to be given the lock */
	for (;;) {
		if (signal_pending(current)) {
			/*
			 * Remove ourselves from the wait list if we
			 * didnt get the lock - else return success:
			 */
			trace_lock_irq(&trace_lock);
			spin_lock(&lock->wait_lock);
			if (waiter.task) {
				list_del_init(&waiter.list);
				/*
				 * Just remove ourselves from the PI list.
				 * (No big problem if our PI effect lingers
				 *  a bit - owner will restore prio.)
				 */
				spin_lock(&pi_lock);
				list_del_init(&waiter.pi_list);
				spin_unlock(&pi_lock);
				ret = -EINTR;
			}
			spin_unlock(&lock->wait_lock);
			trace_unlock_irq(&trace_lock);
			break;
		}
		if (!waiter.task)
			break;
		schedule();
		set_task_state(task, TASK_INTERRUPTIBLE);
	}

	task->state = TASK_RUNNING;
	current->flags |= nosched_flag;

	return ret;
}

/*
 * trylock for writing -- returns 1 if successful, 0 if contention
 */
static int __down_trylock(struct rt_mutex *lock, unsigned long eip)
{
	struct task_struct *task = current;
	unsigned long flags;
	int ret = 0;

	trace_lock_irqsave(&trace_lock, flags);
	TRACE_BUG_ON(!irqs_disabled());
	spin_lock(&lock->wait_lock);

	init_lists(lock);

	if (!lock->owner) {
		/* granted */
		TRACE_WARN_ON(!list_empty(&lock->wait_list));
		spin_lock(&pi_lock);
		set_new_owner(lock, NULL, task, eip);
		spin_unlock(&pi_lock);
		ret = 1;
	}

	spin_unlock(&lock->wait_lock);
	trace_unlock_irqrestore(&trace_lock, flags);

	return ret;
}

int fastcall rt_down_write_trylock(struct rw_semaphore *rwsem)
{
	return __down_trylock(&rwsem->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(rt_down_write_trylock);

/*
 * trylock for reading -- returns 1 if successful, 0 if contention
 */
int fastcall rt_down_read_trylock(struct rw_semaphore *rwsem)
{
	/*
	 * Read locks within the self-held write lock succeed.
	 */
	if (rwsem->lock.owner == current) {
		rwsem->read_depth++;
		return 1;
	}
	return __down_trylock(&rwsem->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(rt_down_read_trylock);

static int down_write_trylock_mutex(struct rw_semaphore *rwsem)
{
	return __down_trylock(&rwsem->lock, CALLER_ADDR0);
}

static int down_read_trylock_mutex(struct rw_semaphore *rwsem)
{
	/*
	 * Read locks within the self-held write lock succeed.
	 */
	if (rwsem->lock.owner == current) {
		rwsem->read_depth++;
		return 1;
	}
	return __down_trylock(&rwsem->lock, CALLER_ADDR0);
}

/*
 * release the lock:
 */
static void __up_mutex(struct rt_mutex *lock, int save_state, unsigned long eip)
{
	struct task_struct *old_owner, *new_owner;
	struct rt_mutex_waiter *w;
	struct list_head *curr;
	unsigned long flags;
	int prio;

	TRACE_WARN_UNINITIALIZED(save_state != lock->save_state);

	trace_lock_irqsave(&trace_lock, flags);
	TRACE_BUG_ON(!irqs_disabled());
	spin_lock(&lock->wait_lock);
	TRACE_BUG_ON(!lock->wait_list.prev && !lock->wait_list.next);

#ifdef CONFIG_RT_DEADLOCK_DETECT
	TRACE_WARN_ON(list_empty(&lock->held_list));
	list_del_init(&lock->held_list);
#endif
	spin_lock(&pi_lock);

	old_owner = lock->owner;
#ifdef ALL_TASKS_PI
	if (list_empty(&lock->wait_list))
		check_pi_list_empty(lock, old_owner);
#endif
	lock->owner = NULL;
	new_owner = NULL;
	if (!list_empty(&lock->wait_list))
		new_owner = pick_new_owner(lock, old_owner, save_state, eip);

	/*
	 * If the owner got priority-boosted then restore it
	 * to the previous priority (or to the next highest prio
	 * waiter's priority):
	 */
	prio = mutex_getprio(old_owner);
	list_for_each(curr, &old_owner->pi_waiters) {
		w = list_entry(curr, struct rt_mutex_waiter, pi_list);
		if (w->task->prio < prio)
			prio = w->task->prio;
		trace_special_pid(w->task->pid, w->task->prio, 0);
	}
	if (prio != old_owner->prio)
		pi_setprio(lock, old_owner, prio);
	if (new_owner) {
		if (save_state)
			wake_up_process_mutex(new_owner);
		else
			wake_up_process(new_owner);
	}
	spin_unlock(&pi_lock);
	spin_unlock(&lock->wait_lock);

#ifdef PREEMPT_DIRECT
	trace_unlock(&trace_lock);
	/*
	 * Common place where preemption is requested - if we can
	 * reschedule then do it here without enabling interrupts
	 * again (and lengthening latency):
	 */
	if (need_resched() && !irqs_disabled_flags(flags) && !preempt_count())
		preempt_schedule_irq();
	local_irq_restore(flags);
#else
	trace_unlock_irqrestore(&trace_lock, flags);
#endif
	/* no need to check for preempt here - we just handled it */
}

/*
 * Do owner check too:
 */
void fastcall rt_up_write(struct rw_semaphore *rwsem)
{
	WARN_ON(rwsem->lock.owner != current);
	BUG_ON(rwsem->read_depth);
	__up_mutex(&rwsem->lock, 0, CALLER_ADDR0);
}
EXPORT_SYMBOL(rt_up_write);

static void _up_write(struct rw_semaphore *rwsem, unsigned long eip)
{
	WARN_ON(rwsem->lock.owner != current);
	BUG_ON(rwsem->read_depth);
	__up_mutex(&rwsem->lock, 0, eip);
}

void fastcall up_write_mutex(struct rw_semaphore *rwsem, unsigned long eip)
{
	TRACE_WARN_UNINITIALIZED(rwsem->lock.save_state != 1);
	WARN_ON(rwsem->lock.owner != current);
	BUG_ON(rwsem->read_depth);
	__up_mutex(&rwsem->lock, 1, eip);
}

/*
 * release a read lock on the semaphore
 */
void fastcall rt_up_read(struct rw_semaphore *rwsem)
{
	/*
	 * Read locks within the self-held write lock succeed.
	 */
	if (rwsem->lock.owner == current && rwsem->read_depth) {
		rwsem->read_depth--;
		return;
	}
	return _up_write(rwsem, CALLER_ADDR0);
}
EXPORT_SYMBOL(rt_up_read);

void fastcall up_read_mutex(struct rw_semaphore *rwsem, unsigned long eip)
{
	TRACE_WARN_UNINITIALIZED(rwsem->lock.save_state != 1);
	/*
	 * Read locks within the self-held write lock succeed.
	 */
	if (rwsem->lock.owner == current && rwsem->read_depth) {
		rwsem->read_depth--;
		return;
	}
	return up_write_mutex(rwsem, eip);
}

/*
 * downgrade a write lock into a read lock
 * - just wake up any readers at the front of the queue
 */
void fastcall rt_downgrade_write(struct rw_semaphore *rwsem)
{
	/*
	 * This function is essentially a no-op in RT. This is
	 * because rwlocks are converted into plain mutexes. This is
	 * done by making a down_read() the same as a down_write() .
	 * In doing that this function is no longer needed. If you
	 * have a write lock , then you have a read lock, so you
	 * don't need to do anything to downgrade ..
	 */
	return;

}
EXPORT_SYMBOL(rt_downgrade_write);

static int rt_mutex_is_locked(struct rt_mutex *lock)
{
	int ret;

	mb();
	ret = lock->owner != NULL;

	return ret;
}

int fastcall rt_rwsem_is_locked(struct rw_semaphore *rwsem)
{
	return rt_mutex_is_locked(&rwsem->lock);
}
EXPORT_SYMBOL(rt_rwsem_is_locked);

static void _down_mutex(struct rt_mutex *lock, unsigned long eip)
{
	TRACE_WARN_UNINITIALIZED(lock->save_state != 1);
	__down_mutex(lock, eip);
}

void fastcall __sema_init(struct semaphore *sem, int val,
			  char *name, char *file, int line)
{
	atomic_set(&sem->count, val);
	switch (val) {
	case 0:
		__init_rt_mutex(&sem->lock, 0, name, file, line);
		__down(&sem->lock, CALLER_ADDR0);
		break;
	default:
		__init_rt_mutex(&sem->lock, 0, name, file, line);
		break;
	}
}
EXPORT_SYMBOL(__sema_init);

void fastcall __init_MUTEX(struct semaphore *sem, char *name, char *file,
			   int line)
{
	__sema_init(sem, 1, name, file, line);
}
EXPORT_SYMBOL(__init_MUTEX);

static int down_trylock_mutex(struct rt_mutex *lock, unsigned long eip)
{
	TRACE_WARN_UNINITIALIZED(lock->save_state != 1);
	return __down_trylock(lock, eip);
}

void fastcall up_mutex(struct rt_mutex *lock, unsigned long eip)
{
	TRACE_WARN_UNINITIALIZED(lock->save_state != 1);
	WARN_ON(lock->owner != current);
	__up_mutex(lock, 1, eip);
}

/*
 * Linux Semaphores implemented via RT-mutexes.
 *
 * In the down() variants we use the mutex as the semaphore blocking
 * object: we always acquire it, decrease the counter and keep the lock
 * locked if we did the 1->0 transition. The next down() will then block.
 *
 * In the up() path we atomically increase the counter and do the
 * unlock if we were the one doing the 0->1 transition.
 */

static inline void __down_complete(struct semaphore *sem, unsigned long eip)
{
	int count = atomic_dec_return(&sem->count);

	TRACE_WARN_UNINITIALIZED(sem->lock.save_state != 0);
	WARN_ON(count < 0);

	if (count > 0)
		__up_mutex(&sem->lock, 0, eip);
}

void fastcall rt_down(struct semaphore *sem)
{
	TRACE_WARN_UNINITIALIZED(sem->lock.save_state != 0);
	__down(&sem->lock, CALLER_ADDR0);
	__down_complete(sem, CALLER_ADDR0);
}
EXPORT_SYMBOL(rt_down);

int fastcall rt_down_interruptible(struct semaphore *sem)
{
	int ret;

	TRACE_WARN_UNINITIALIZED(sem->lock.save_state != 0);
	ret = __down_interruptible(&sem->lock, CALLER_ADDR0);
	if (ret)
		return ret;
	__down_complete(sem, CALLER_ADDR0);
	return 0;
}
EXPORT_SYMBOL(rt_down_interruptible);

/*
 * try to down the semaphore, 0 on success and 1 on failure. (inverted)
 */
int fastcall rt_down_trylock(struct semaphore *sem)
{
	TRACE_WARN_UNINITIALIZED(sem->lock.save_state != 0);
	/*
	 * Here we are a tiny bit different from ordinary Linux semaphores,
	 * because we can get 'transient' locking-failures when say a
	 * process decreases the count from 9 to 8 and locks/releases the
	 * embedded mutex internally. It would be quite complex to remove
	 * these transient failures so lets try it the simple way first:
	 */
	if (__down_trylock(&sem->lock, CALLER_ADDR0)) {
		__down_complete(sem, CALLER_ADDR0);
		return 0;
	}
	return 1;
}
EXPORT_SYMBOL(rt_down_trylock);

void fastcall rt_up(struct semaphore *sem)
{
	int count;

	TRACE_WARN_UNINITIALIZED(sem->lock.save_state != 0);
	/*
	 * Disable preemption to make sure a highprio trylock-er cannot
	 * preempt us here and get into an infinite loop:
	 */
	preempt_disable();
	count = atomic_inc_return(&sem->count);
	/*
	 * If we did the 0 -> 1 transition then we are the ones to unlock it:
	 */
	if (count == 1)
		__up_mutex(&sem->lock, 0, CALLER_ADDR0);
	preempt_enable();
}
EXPORT_SYMBOL(rt_up);

int fastcall rt_sem_is_locked(struct semaphore *sem)
{
	TRACE_WARN_UNINITIALIZED(sem->lock.save_state != 0);
	return rt_mutex_is_locked(&sem->lock);
}
EXPORT_SYMBOL(rt_sem_is_locked);

int fastcall rt_sema_count(struct semaphore *sem)
{
	TRACE_WARN_UNINITIALIZED(sem->lock.save_state != 0);
	return atomic_read(&sem->count);
}
EXPORT_SYMBOL(rt_sema_count);

/*
 * Spinlock wrappers:
 */

static void __spin_lock(spinlock_t *lock, unsigned long eip)
{
	SAVE_BKL(_down_mutex(&lock->lock, eip));
}

void _spin_lock(spinlock_t *spin)
{
	__spin_lock(spin, CALLER_ADDR0);
}
EXPORT_SYMBOL(_spin_lock);

void _spin_lock_bh(spinlock_t *spin)
{
	__spin_lock(spin, CALLER_ADDR0);
}
EXPORT_SYMBOL(_spin_lock_bh);

void _spin_lock_irq(spinlock_t *spin)
{
	__spin_lock(spin, CALLER_ADDR0);
}
EXPORT_SYMBOL(_spin_lock_irq);

unsigned long _spin_lock_irqsave(spinlock_t *spin)
{
	unsigned long flags;

	__spin_lock(spin, CALLER_ADDR0);
	local_save_flags(flags);

	return flags;
}
EXPORT_SYMBOL(_spin_lock_irqsave);

void _spin_unlock(spinlock_t *lock)
{
	up_mutex(&lock->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(_spin_unlock);

void _spin_unlock_wait(spinlock_t *lock)
{
	do {
		barrier();
	} while (_spin_is_locked(lock));
}
EXPORT_SYMBOL(_spin_unlock_wait);

void _spin_unlock_bh(spinlock_t *lock)
{
	up_mutex(&lock->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(_spin_unlock_bh);

void _spin_unlock_irq(spinlock_t *lock)
{
	up_mutex(&lock->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(_spin_unlock_irq);

void _spin_unlock_irqrestore(spinlock_t *lock, unsigned long flags)
{
	up_mutex(&lock->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(_spin_unlock_irqrestore);

int _spin_trylock(spinlock_t *lock)
{
	return down_trylock_mutex(&lock->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(_spin_trylock);

int _spin_trylock_bh(spinlock_t *lock)
{
	return down_trylock_mutex(&lock->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(_spin_trylock_bh);

int _spin_trylock_irq(spinlock_t *lock)
{
	return down_trylock_mutex(&lock->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(_spin_trylock_irq);

int _spin_trylock_irqsave(spinlock_t *lock, unsigned long *flags)
{
	local_save_flags(*flags);
	return down_trylock_mutex(&lock->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(_spin_trylock_irqsave);

int _spin_is_locked(spinlock_t *lock)
{
	return rt_mutex_is_locked(&lock->lock);
}
EXPORT_SYMBOL(_spin_is_locked);

int _spin_can_lock(spinlock_t *lock)
{
	return !rt_mutex_is_locked(&lock->lock);
}
EXPORT_SYMBOL(_spin_can_lock);

int atomic_dec_and_spin_lock(atomic_t *atomic, spinlock_t *lock)
{
	__spin_lock(lock, CALLER_ADDR0);
	if (atomic_dec_and_test(atomic))
		return 1;
	_spin_unlock(lock);

	return 0;
}
EXPORT_SYMBOL(atomic_dec_and_spin_lock);

void _spin_lock_init(spinlock_t *lock, char *name, char *file, int line)
{
	__init_rt_mutex(&lock->lock, 1, name, file, line);
}
EXPORT_SYMBOL(_spin_lock_init);


/*
 * RW-lock wrappers:
 */
int _read_trylock(rwlock_t *rwlock)
{
	return down_read_trylock_mutex(&rwlock->lock);
}
EXPORT_SYMBOL(_read_trylock);

int _write_trylock(rwlock_t *rwlock)
{
	return down_write_trylock_mutex(&rwlock->lock);
}
EXPORT_SYMBOL(_write_trylock);

void _write_lock(rwlock_t *rwlock)
{
	down_write_mutex(&rwlock->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(_write_lock);

void _read_lock(rwlock_t *rwlock)
{
	down_read_mutex(&rwlock->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(_read_lock);

void _write_unlock(rwlock_t *rwlock)
{
	up_write_mutex(&rwlock->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(_write_unlock);

void _read_unlock(rwlock_t *rwlock)
{
	up_read_mutex(&rwlock->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(_read_unlock);

unsigned long _write_lock_irqsave(rwlock_t *rwlock)
{
	unsigned long flags;

	down_write_mutex(&rwlock->lock, CALLER_ADDR0);

	local_save_flags(flags);
	return flags;
}
EXPORT_SYMBOL(_write_lock_irqsave);

unsigned long _read_lock_irqsave(rwlock_t *rwlock)
{
	unsigned long flags;

	down_read_mutex(&rwlock->lock, CALLER_ADDR0);

	local_save_flags(flags);
	return flags;
}
EXPORT_SYMBOL(_read_lock_irqsave);

void _write_lock_irq(rwlock_t *rwlock)
{
	down_write_mutex(&rwlock->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(_write_lock_irq);

void _read_lock_irq(rwlock_t *rwlock)
{
	down_read_mutex(&rwlock->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(_read_lock_irq);

void _write_lock_bh(rwlock_t *rwlock)
{
	down_write_mutex(&rwlock->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(_write_lock_bh);

void _read_lock_bh(rwlock_t *rwlock)
{
	down_read_mutex(&rwlock->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(_read_lock_bh);

void _write_unlock_irq(rwlock_t *rwlock)
{
	up_write_mutex(&rwlock->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(_write_unlock_irq);

void _read_unlock_irq(rwlock_t *rwlock)
{
	up_read_mutex(&rwlock->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(_read_unlock_irq);

void _write_unlock_bh(rwlock_t *rwlock)
{
	up_write_mutex(&rwlock->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(_write_unlock_bh);

void _read_unlock_bh(rwlock_t *rwlock)
{
	up_read_mutex(&rwlock->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(_read_unlock_bh);

void _write_unlock_irqrestore(rwlock_t *rwlock,
				       unsigned long flags)
{
	up_write_mutex(&rwlock->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(_write_unlock_irqrestore);

void _read_unlock_irqrestore(rwlock_t *rwlock, unsigned long flags)
{
	up_read_mutex(&rwlock->lock, CALLER_ADDR0);
}
EXPORT_SYMBOL(_read_unlock_irqrestore);

void _rwlock_init(rwlock_t *rwlock, char *name, char *file, int line)
{
	__init_rwsem(&rwlock->lock, 1, name, file, line);
}
EXPORT_SYMBOL(_rwlock_init);

int _rwlock_is_locked(rwlock_t *rwlock)
{
	return rt_rwsem_is_locked(&rwlock->lock);
}
EXPORT_SYMBOL(_rwlock_is_locked);

/*
 * TODO: is it ok if _read_can_lock() and _write_can_lock() does the same?
 */
int _read_can_lock(rwlock_t *rwlock)
{
	return !rt_rwsem_is_locked(&rwlock->lock);
}
EXPORT_SYMBOL(_read_can_lock);

int _write_can_lock(rwlock_t *rwlock)
{
	return !rt_rwsem_is_locked(&rwlock->lock);
}
EXPORT_SYMBOL(_write_can_lock);

