/*
 * linux/kernel/hrtimers.c
 *
 *                           by George Anzinger george@mvista.com
 *
 *			     Copyright (C) 2005 by MontaVista Software.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * MontaVista Software | 1237 East Arques Avenue | Sunnyvale | CA 94085 | USA
 */
/*
 * This module contains the code to process the arch_cycle timers.  When
 * the timer list (in kernel/timer.c) finds a timer that expires with a
 * non-zero arch_cycle_expire value it passes it to
 * do_arch_cycle_timer() which does one of two things:
 *
 * 1) if the timer has expired, passes it on to the call back function, or 
 * 2) puts it in an ordered list of such timers which will expire in the
 *    current jiffie.  

 * There are various ways to call this code depending on if the timer
 * can be delivered immeadiately.  Some timers, such as those from
 * add_timer, can not be delivered immeadiately as such delivery might
 * cause recursion possibly to the end of the kernels days
 * uh.. nanoseconds.
 *
 * Timers that are pending delivery are put into a pending list.  This
 * is used both for the above case and for cases where we detect that
 * the jiffie has expired and need to clear the list to make way for
 * arch_cycle timers in the now current jiffie.  Since these can come in
 * as a result of expiring a timer, we need to clean the list up first.
 * We just move all the timers to the expired list.
 *
 * Speaking of lists, we handle either one list set (pending and
 * expired) per cpu or one global list for all cpus.  The choice is to
 * be made by the arch code and should depend on if a per cpu timer is
 * being used to do the arch_cycle timing.  The i386 arch uses per cpu
 * hardware, other archs may not have such.
 */

#include <linux/percpu.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/cpu.h>
#include <linux/list.h>
#include <linux/hrtimers.h>
#include <linux/interrupt.h>
#include <linux/notifier.h>

/*
 * waiting, in the following structure, is the address of the timer we
 * have set the hardware up for, if any.  It allows us to touch the
 * hardware only when we need to.
 */

struct hrtimer_lists_s {
	spinlock_t lock;           /* timer_del depends on this being first */
	struct list_head pending;
	struct list_head expired;
	unsigned long jiffies;
	struct timer_list *waiting;
} ____cacheline_aligned_in_smp;

typedef struct  hrtimer_lists_s hrtimer_lists_t;

/*
 * Some archs (x86 for example) have a timer per cpu.  Others don't.
 * The arch needs to define 'HRTIME_PER_CPU' if it has per cpu timers
 * and it wants to have per cpu HR queues.  Otherwise, we combine them.
 * Also, for the combined queue we use tasklets, for the per cpu case we
 * use a softirq.
 *
 * This code is designed to run under the RT convention of preemptability.
 * Since we depend on a consistant cpu (in the HRTIME_PER_CPU case) we
 * get and put the cpu in that case.  This means we need to pass the 
 * list address from the primary entries to the helper funcitons rather
 * than re resolving them.  All helper functions are marked as static...
 */
#ifdef HRTIME_PER_CPU
static DEFINE_PER_CPU(hrtimer_lists_t, hrtimer_lists) = { 
	.lock = SPIN_LOCK_UNLOCKED
};
#define get_hrtimer_lists_cpu(cpu) per_cpu(hrtimer_lists, cpu)
#define get_hrtimer_lists() per_cpu(hrtimer_lists, smp_processor_id())
#define lock_cpu() 						\
	cpumask_t cpum, save_cpumask = current->cpus_allowed;	\
								\
	cpus_clear(cpum);					\
	cpu_set(get_cpu(),cpum);				\
	current->cpus_allowed = cpum;				\
	put_cpu()
#define unlock_cpu() current->cpus_allowed = save_cpumask
#else
static hrtimer_lists_t hrtimer_lists = {.lock = SPIN_LOCK_UNLOCKED};
#define get_hrtimer_lists_cpu(cpu) hrtimer_lists
#define get_hrtimer_lists() hrtimer_lists
#define lock_cpu()
#define unlock_cpu()
#endif

/*
 * get a timer given its list entry address.
 */
#define get_timer(pos) list_entry(pos, struct timer_list, entry)

static void hrtimer_insert(struct timer_list *timer, hrtimer_lists_t *hrtimers)
{
	/*
	 * For now we are doing a simple O(N/2) insert.  Later we may
	 * want to adjust this, but please do remember this list is ONLY
	 * of timers that expire in the current jiffie.  
	 */
	struct list_head *pos;

	list_for_each(pos, &hrtimers->pending) {
		struct timer_list *ltimer = get_timer(pos);
		if (ltimer->arch_cycle_expires > timer->arch_cycle_expires)
			break;
	}
	list_add_tail(&timer->entry, pos);
	timer->base = (void *)hrtimers;
}

static int _do_arch_cycle_timer(struct timer_list *timer, 
				hrtimer_lists_t *hrtimers)
{
	unsigned long flags;
	int rtn;

	spin_lock_irqsave(&hrtimers->lock, flags);
	if (hrtimers->jiffies != jiffies ) {
		list_splice_init(&hrtimers->pending, hrtimers->expired.prev);
		hrtimers->jiffies = jiffies;
		hrtimers->waiting = NULL;
	}
	hrtimer_insert(timer, hrtimers);
	rtn = (get_arch_cycles(hrtimers->jiffies) > 
		get_timer(hrtimers->pending.next)->arch_cycle_expires);
	spin_unlock_irqrestore(&hrtimers->lock, flags);
	return rtn;
	
}
/*
 * This is our tasklet or softirq function.  If we are late,
 * i.e. jiffies is not what we have recorded in out list, we move the
 * pending list to the expired list.  Then we deliver all timers in the
 * expired list and those in the pending list that have expired.  If
 * there are any left, the first one is the next so we set up a timer to
 * expire it.  Alwasy run from soft irq, i.e. cpu lock state.
 */

static void do_hrtimers_expire_timers(hrtimer_lists_t *hrtimers)
{
	struct timer_list *timer;

	spin_lock_irq(&hrtimers->lock);
	/*
	 * if we are late, move the whole list...
	 */
	if (hrtimers->jiffies != jiffies ) {
		list_splice_init(&hrtimers->pending, hrtimers->expired.prev);
		hrtimers->jiffies = jiffies;
		hrtimers->waiting = NULL;
	}
	while (!list_empty(&hrtimers->expired)) {
		void (*fn)(unsigned long);
		unsigned long data;

		timer = get_timer(hrtimers->expired.next);
		fn = timer->function;
		data = timer->data;

		list_del(&timer->entry);
		smp_wmb();
		timer->base = NULL;
		spin_unlock_irq(&hrtimers->lock);
		fn(data);
		spin_lock_irq(&hrtimers->lock);
	}
		
again:
	while (!list_empty(&hrtimers->pending)) {
		timer = get_timer(hrtimers->pending.next);
		
		if (timer->arch_cycle_expires <= 
		    get_arch_cycles(hrtimers->jiffies)) {

			void (*fn)(unsigned long);
			unsigned long data;

			fn = timer->function;
			data = timer->data;

			list_del(&timer->entry);
			smp_wmb();
			timer->base = NULL;
			hrtimers->waiting = NULL;
			spin_unlock_irq(&hrtimers->lock);
			fn(data);
			spin_lock_irq(&hrtimers->lock);
		} else {
			if (hrtimers->waiting != timer) {
				if (schedule_hr_timer_int(
					    hrtimers->jiffies, 
					    timer->arch_cycle_expires)) {
					goto again;
				}
				hrtimers->waiting = timer;
			}
			break;
		}
	}
	spin_unlock_irq(&hrtimers->lock);
}
/*
 * front end code for above to resolve the list and cpu lock thing.  This
 * is the real handle we give to the soft irq code.
 */
void hrtimers_expire_timers(void)
{
	hrtimer_lists_t *hrtimers = &get_hrtimer_lists();
	do_hrtimers_expire_timers(hrtimers);
}

/* 
 * We get here when the add_timer code finds a timer that expiers either
 * in the current jiffie or in a past one.  If it is a past one, we put
 * it in the expired list, else in the pending list.  Even if it is not
 * a high res timer.  If it isn't it will just go to the front of that
 * list and be handled in due course.
 *
 * Since this is from the add_timer code we can not expire a timer right
 * now but must wait for our tasklet.
 *
 * We use a home grown cpu lock to insure the list and hardware are on 
 * the same cpu.
 */
void do_handle_expired_timer(struct timer_list *timer)
{
	hrtimer_lists_t * hrtimers;

	lock_cpu();
	hrtimers = &get_hrtimer_lists();
	if (likely(timer->expires == jiffies)) {
		_do_arch_cycle_timer(timer, hrtimers);
		if (timer == get_timer(hrtimers->pending.next)) {
			do_hr_timer_int();
		}
	} else {
		/*
		 * It is a stale timer...
		 */
		spin_lock(&hrtimers->lock);
		list_add_tail(&timer->entry, &hrtimers->expired);
		timer->base = (void *)hrtimers;
		spin_unlock(&hrtimers->lock);
		do_hr_timer_int();
	}
	unlock_cpu();
}
/*
 * This code is called by the run_timers code in kernel/timer.c for each
 * timer.  If it is a HR timer we grab it and return 1, it is not we do
 * nothing and return 0.  We are already in a cpu lock state.
 */
void do_high_res_timer(struct timer_list *timer, spinlock_t *lock)
{
	int timers_expired;
	hrtimer_lists_t *hrtimers = &get_hrtimer_lists();
	/*
	 * We have a high res timer.  We are currenly under the timer
	 * list spinlock.  We need to move the timer to the new list and
	 * update its base under that lock.
	 */
	list_del(&timer->entry);
	timers_expired = _do_arch_cycle_timer(timer, hrtimers);
	spin_unlock_irq(lock);
	if (timers_expired)
		do_hrtimers_expire_timers(hrtimers);

	spin_lock_irq(lock);
}
/*

 * We need to provide an indication to VST that we have a timer runing,
 * least it use the same hardware and mess everything up.  Also, we
 * should not do a VST sleep if we have a timer running...
 */
#ifdef CONFIG_VST
int hrtimer_is_pending(void)
{
	hrtimer_lists_t *hrtimers = &get_hrtimer_lists();
	return 	!list_empty(&hrtimers->pending);
}
#endif
static void __devinit init_hrtimers_cpu(long cpu)
{
	hrtimer_lists_t *hrtimers = &get_hrtimer_lists_cpu(cpu);
	spin_lock_init(&hrtimers->lock);
	INIT_LIST_HEAD(&hrtimers->pending);
	INIT_LIST_HEAD(&hrtimers->expired);
	hrtimers->jiffies = 0;
	hrtimers->waiting = NULL;
}

/*
 * To do... the hot plug stuff...
 */
#ifdef HRTIME_PER_CPU
static int __devinit hrtimer_cpu_notify(struct notifier_block *self, 
				unsigned long action, void *hcpu)
{
	long cpu = (long)hcpu;
	switch(action) {
	case CPU_UP_PREPARE:
		init_hrtimers_cpu(cpu);
		break;
#ifdef CONFIG_HOTPLUG_CPUx
	case CPU_DEAD:
		migrate_timers(cpu);
		break;
#endif
	default:
		break;
	}
	return NOTIFY_OK;
}
static struct notifier_block __devinitdata hrtimers_nb = {
	.notifier_call	= hrtimer_cpu_notify,
};
#else
DECLARE_TASKLET(hrt_tasklet, 
		(void (*)(unsigned long))hrtimers_expire_timers,
		0);
#define register_cpu_notifier(a)
#define open_softirq(a, b, c)
#endif
__init void init_hrtimers(void)
{
	init_hrtimers_cpu(smp_processor_id());
	register_cpu_notifier(&hrtimers_nb);
	open_softirq(HRTIMER_SOFTIRQ, 
		     (void (*)(struct softirq_action*))hrtimers_expire_timers, 
		     NULL);
}
