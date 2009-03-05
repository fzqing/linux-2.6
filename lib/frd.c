/*
 * FILE NAME lib/frd.c
 *
 * Author: Sven Dietrich sdietrich@mvista.com MontaVista Software, Inc.
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/blkdev.h>
#include <linux/kernel_stat.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/frd.h>
#include <linux/delay.h>
#include <linux/seq_file.h>
#include <linux/irq.h>

static char *procnames[] = {
	"preempt_latency",
	"switch_latency",
	"sleep_time",
	"exec_time",
	"abs_time",
	"irq_time"
};

static char *cur_procnames[] = {
	"cur_preempt_latency",
	"cur_switch_latency",
	"cur_sleep_time",
	"cur_exec_time",
	"cur_abs_time",
	"cur_irq_time"
};

static char *max_procnames[] = {
	"max_preempt_latency",
	"max_switch_latency",
	"max_sleep_time",
	"max_exec_time",
	"max_abs_time",
	"max_irq_time"
};

void frd_latency_init(void);
void frd_latency_log(int frd_id, int type, unsigned long latency);

static void frd_get_cpu_utils(frd_cpu_utils_t * frd_cpu_utils);

static int frd_run;    /* Kill Switch */

/* RT Task Pointers and Parameters */
static task_t *frd_task[FRD_THREADS];
static long frd_tid[FRD_THREADS];
static long frd_thread_param[FRD_THREADS];

/* RT task wait queues. */
static wait_queue_head_t frd_thread_wait[FRD_THREADS];
static wait_queue_head_t timer_dev_wait;

/* Pacing is useful for debugging; we can slow down to printk speed */
#if CONFIG_FRD_PACING
static int frd_pacing = CONFIG_FRD_PACING;
#endif

static struct timeval frd_start_time;		/* wall time at start */
static struct timeval frd_now_time;		/* current wall time */

/* Statistics */
static unsigned long frd_interrupts;		   /* count handler calls  */
static unsigned long frd_test_iterations = CONFIG_FRD_TEST_ITERATIONS;
static unsigned long frd_start_delay = CONFIG_FRD_DELAY;
static unsigned long frd_start_delay_ticks;	   /* delay for kernel boot */

static unsigned long frd_scheds[FRD_THREADS];	   /* count wake-ups 	    */
static unsigned long frd_exec_count[FRD_THREADS];  /* count execute cycles  */
static unsigned long deadline_miss_count[FRD_THREADS];	/* deadlines missed */
static unsigned long dns_miss_count[FRD_THREADS];  /* did-not-start misses  */
static unsigned long max_p_latency_iteration[FRD_THREADS];  /* which cycle  */
static unsigned long frd_exec_count_last[FRD_THREADS];

static unsigned long buckets[FRD_THREADS][LATENCY_TYPES][PLAT_BUCKETS + 1];
static unsigned long max_latency[FRD_THREADS][LATENCY_TYPES];
static unsigned long cur_latency[FRD_THREADS][LATENCY_TYPES];
static unsigned long total_samples[FRD_THREADS][LATENCY_TYPES];
static unsigned long avg_latency[FRD_THREADS][LATENCY_TYPES];
static unsigned long long total_latency_time[FRD_THREADS][LATENCY_TYPES];

/* Holds Latched Hardware Counter */
static unsigned long ticks_at_irq[FRD_THREADS];

/* Time Stamps */
static unsigned long long post_time[FRD_THREADS];  /* time: thread woken    */
static unsigned long long irq_time[FRD_THREADS];
static unsigned long long last_sched[FRD_THREADS]; /* time: thread starts   */
static unsigned long long last_time[FRD_THREADS];  /* time: going to sleep  */

/* Export latency data to userspace via blocking I/O */
static atomic_t preempt_dev_busy;
static int frd_latcnt;
static int frd_latdat[FRD_LATSIZE];

/* Proc File System Entry and Data */
static struct proc_dir_entry *frd_root_dir;
static frd_param_t frd_params[FRD_THREADS][LATENCY_TYPES];

/* CPU Utilization */
static frd_cpu_utils_t frd_start_cpu_utils;
static frd_cpu_utils_t frd_cur_cpu_utils;
static cpu_util_percent_t cpu_utils_percent;

/* This is the FRD Interrupt Handler. It should run as an S/A IRQ.
 *
 * Check whether threads have completed their cycles.
 * Wake them up if they have completed, and record wakeup-time.
 * Otherwise, charge them either Did-Not-Start or Did-Not-Finish */

static void check_frd(void)
{
	int i, task_ready;
	unsigned long long wake_up;
	unsigned long ticks;

	frd_interrupts++;

	if (!frd_run)
		return;
	/* DELAY STARTUP */
	if (frd_interrupts < frd_start_delay_ticks)
		return;
	/* PACING skips interrupts */
#if CONFIG_FRD_PACING
	if (frd_pacing-- > 0)
		return;
	frd_pacing = CONFIG_FRD_PACING;
#endif

	ticks = FRD_TIMER_LATCH;
	wake_up = frd_clock();
	for (i = 0; i < FRD_THREADS; i++) {
		/* check exec count to make sure we got out of
		 * the blocks. mark deadline if  thread did not start,
		 * and wait for it, since it was awakened */
		task_ready = waitqueue_active(&frd_thread_wait[i]);
		if (!task_ready) {
			deadline_miss_count[i]++;
			continue;
		}
		if (frd_exec_count[i] <= frd_exec_count_last[i]) {
			if (frd_exec_count[i]) {
				/* ignore the first loop */
				dns_miss_count[i]++;
				continue;
			}
		}

		/* The thread has started and completed its
		 * cycle since the last IRQ. The thread is eligible to
		 * start again */
		frd_exec_count_last[i] = frd_exec_count[i];

		/* Save IRQ time stamp for this thread at wake up.
		 * Threads can miss multiple deadlines without
		 * losing the original interrupt time stamp */
		irq_time[i] = wake_up;
		ticks_at_irq[i] = ticks;
#ifndef CONFIG_FRD_WAKE_ALL
		if (i == 0)
#endif
		{
			post_time[i] = wake_up;
			/* assuming we are waking up a thread
			 * in the RT kernel from an SA_NODELAY handler,
			 * we can't use wake_up, must use wake_up_process */
			wake_up_process(frd_task[i]);
			frd_scheds[i]++;
		}
	}
}


void calc_frd_thread_prio(int id, struct sched_param *param)
{

#ifdef CONFIG_FRD_ASCENDING_PRIO
	param->sched_priority = MAX_USER_RT_PRIO - FRD_THREADS + id;
#elif defined CONFIG_FRD_DESCENDING_PRIO
	param->sched_priority = MAX_USER_RT_PRIO - 1 - id;	/* 0=highest */
#else
	param->sched_priority = MAX_USER_RT_PRIO - 1;	/* same prio */
#endif
}

#if !defined CONFIG_FRD_USE_TIMER_IRQ
static irqreturn_t
frd_timer_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	FRD_CHECK_SHARED_INTERRUPT;
	check_frd();
	FRD_TIMER_IRQ_ACK;
	return IRQ_HANDLED;
}

static struct irqaction frd_timer_irq = {
	.name = "FRD timer ",
	.flags = SA_INTERRUPT | SA_NODELAY | FRD_IRQ_FLAGS,
	.handler = frd_timer_interrupt
};
#endif


static inline unsigned long frd_elapsed_time(void) 
{
	return (frd_now_time.tv_sec - frd_start_time.tv_sec);
}


/* this function accepts two time stamps at native timer frequencies 
 * and returns the difference between the two. */
unsigned long diff_time(unsigned long long t_first, unsigned long long t_second)
{
#ifndef FRD_64BIT_TIME
	unsigned long l;
	unsigned long t1, t2;
	t1 = (unsigned long)t_first;
	t2 = (unsigned long)t_second;
#else
	unsigned long long l;
	unsigned long long t1, t2;
	t1 = t_first;
	t2 = t_second;
#endif
	l = t2 - t1;

#ifdef FRD_DEBUG_TIMERS
	if (t_first > t_second)
		printk("Pid %d: Rollover detected from %llu to %llu = %lld\n",
		       current->pid, (unsigned long long)t_first,
		       (unsigned long long)t_second, (unsigned long long)l);
#endif
	return (unsigned long)l;
}

static inline long
update_frd_stats(int id,
		 unsigned long long start_t,
		 unsigned long long end_t,
		 int m_type)
{
	long remainder;
	unsigned long latency;

#ifndef FRD_TIMER_COUNTS_DOWN
	unsigned long long swap_t;
#ifndef FRD_ABS_TIMER_COUNTS_DOWN
	if (m_type == ABS_LATENCY_TYPE) {
		swap_t = start_t;
		start_t = end_t;
		end_t = swap_t;
	}
#endif
#ifndef FRD_IRQ_TIMER_COUNTS_DOWN
	if (m_type == IRQ_LATENCY_TYPE) {
		swap_t = start_t;
		start_t = end_t;
		end_t = swap_t;
	}
#endif
#endif

	latency = diff_time(start_t, end_t);

	/* we do not want to rescale the a priori measurments,
	 * since we asssume that LATCH returns microseconds */
#ifndef FRD_TIMER_LATCH_ABS
	if (m_type != ABS_LATENCY_TYPE && m_type != IRQ_LATENCY_TYPE)
#endif
		latency = clock_to_usecs(latency);

# ifdef FRD_SCALE_ABS_TICKS
	else /* we DO want to scale latency if it is the absolute type,
		and we assume the default scaling function. */
		latency = clock_to_usecs(latency);
# endif

	/* check <= to be sure we do not roll to 0 */
	if (MAXULL - total_latency_time[id][m_type] <= latency) {
		printk("FRD[%2d]:%8lu Overflow: %s %llu %lu\n",
		       id, frd_exec_count[id], procnames[m_type],
			total_latency_time[id][m_type], (long) latency);
		/* either exit - end of test, or reset all stats... */
		return -1;
	}

	total_latency_time[id][m_type] += latency;
	/*
	 * alternative average calculation scheme:
	 * ((a_1 + ... + a_N)/N * N + a_(N+1))/N+1
	 */
	avg_latency[id][m_type] = (unsigned long)
			div_long_long_rem(total_latency_time[id][m_type],
					     frd_exec_count[id] - 1,
					     &remainder);

	frd_latency_log(id, m_type, latency);

	return latency;
}

/* This is the Fast realtime simulation.
   The threads are periodically woken up, run, wake up other threads,
   and then go back to sleep - threads not currently SMP affine,
   the threads should be so, and maintain per cpu stats */

int frd_thread(void *tid)
{

	long id;
	long last_latency;
	unsigned long tick_count, tick_count2;
	unsigned long long sched_time = 0;
	struct sched_param param;

	id = (long)tid;
	frd_task[id] = current;
	current->flags |= PF_NOFREEZE;

	/* Pin this thread to the first cpu. */
	sched_setaffinity(frd_tid[id],cpumask_of_cpu(0));
	calc_frd_thread_prio(id, &param);

	if (sched_setscheduler(current, SCHED_FIFO, &param))
		printk(KERN_WARNING "setscheduler: failure on frd tid %lu \n",
			frd_tid[id]);

	daemonize("frd[%d]", id);

	recalc_sigpending();

	while (frd_run) {
		local_irq_disable();	/* no interruptions now */
		last_sched[id] = sched_time;	/* save for execution calc */
		last_time[id] = frd_clock();	/* finished */
		local_irq_enable();

		interruptible_sleep_on(&frd_thread_wait[id]);
		local_irq_disable();	/* no interruptions now */
		sched_time = frd_clock();	/* start time */
#ifdef FRD_TIMER_LATCH_ABS
		tick_count = FRD_TIMER_LATCH_ABS(irq_time[id],
						 ticks_at_irq[id]);
#else
		tick_count = FRD_TIMER_LATCH;
#endif

		/* each thread increments its own counter
		 * if they are not the same, threads have
		 * missed execution cycles */
		frd_exec_count[id]++;

#ifndef CONFIG_FRD_WAKE_ALL
		if (id < (FRD_THREADS - 1)) {
			if (waitqueue_active(&frd_thread_wait[id + 1])) {
				/* wake up time stamp */
				post_time[id + 1] = frd_clock();
				wake_up_process(frd_task[id+1]);
				frd_scheds[id + 1]++;
			}
		}
#endif
		local_irq_enable();

#ifdef CONFIG_FRD_ASCENDING_PRIO
		/* All the time stamps are taken and threads are woken.
		 * Lower priority and let the rest of the threads go
		 * through their time critical operations.
		 * Then raise the priority again and finish the statistics
		 * calculation */
		param.sched_priority = MAX_USER_RT_PRIO - FRD_THREADS - 1;
		/* what is the response time of sched_setscheduler */
		if (sched_setscheduler(frd_task[id], SCHED_FIFO, &param))
			printk("setscheduler frd tid %d prio %d\n",
			       frd_tid[id], param.sched_priority);

		calc_frd_thread_prio(id, &param);
		if (sched_setscheduler(frd_task[id], SCHED_FIFO, &param))
			printk("setscheduler frd tid %d prio %d\n",
			       frd_tid[id], param.sched_priority);
#endif

		if (frd_exec_count[id] == 1) {
			/* initial cycle to sync up with the tick */
			last_time[id] = sched_time;
			last_sched[id] = sched_time;
			if (id == 0) 
			{
				frd_get_cpu_utils(&frd_start_cpu_utils);
			}
			continue;
		}

		if (frd_test_iterations == frd_exec_count[id]) {
			printk("FRD[%lu] exec count %lu test ends at %lu\n", id,
			frd_exec_count[id], frd_test_iterations);
			break;
		}

		/* save the value so we can check and update the max iteration
		 */
		last_latency = max_latency[id][PREEMPT_LATENCY_TYPE];

		tick_count2 = FRD_TIMER_INIT;

		/* calculate time between timer expire and task running
		   == absolute latency  */
		if (update_frd_stats(id, tick_count,tick_count2,
					ABS_LATENCY_TYPE) < 0)
			break;

		tick_count = ticks_at_irq[id];
		tick_count2 = FRD_TIMER_INIT;

		if (update_frd_stats(id, tick_count,tick_count2,
					IRQ_LATENCY_TYPE) < 0)
			break;

		/* calculate time between interrupt and task running
		   == preemption latency  */
		if (update_frd_stats(id, irq_time[id], sched_time,
					PREEMPT_LATENCY_TYPE) < 0)
			break;

		/* calculate time between wakeup and scheduling this task */
		if (update_frd_stats(id, post_time[id], sched_time,
					SWITCH_LATENCY_TYPE) < 0)
			break;

		/* log execution time for the last iteration */
		if (update_frd_stats(id, last_sched[id], last_time[id],
					EXEC_LATENCY_TYPE) < 0)
			break;

		/* log time between going to sleep and scheduling this task */
		if (update_frd_stats(id, last_time[id], sched_time,
					SLEEP_LATENCY_TYPE) < 0)
			break;

		if (max_latency[id][PREEMPT_LATENCY_TYPE] > last_latency) {
			max_p_latency_iteration[id] = frd_exec_count[id];
		}

		if (id == (FRD_THREADS - 1)) 
		{
			do_gettimeofday(&frd_now_time); /* for elapsed time */
			frd_get_cpu_utils(&frd_cur_cpu_utils);
			if ( (frd_latcnt >= FRD_LATSIZE) &&
				(waitqueue_active(&timer_dev_wait)) )
                                wake_up(&timer_dev_wait);
		}
	}

	/* Reducing the thread-ready count suspends wake_ups. */
	frd_run = 0;
	printk(KERN_WARNING "\n\nFRD[%2lu] pid %d exited at priority: %lu\n",
	       id, (int)current->pid, current->rt_priority);
	return 0;
}

static int preempt_dev_open(struct inode *inode, struct file *file)
{
	if (atomic_read(&preempt_dev_busy) == 1) {
		return -EBUSY;
	}
	atomic_inc(&preempt_dev_busy);
	return 0;
}

static int preempt_dev_close(struct inode *inode, struct file *file)
{
	atomic_dec(&preempt_dev_busy);
	return 0;
}

static ssize_t
preempt_dev_read(struct file *file, char __user * buf,
		 size_t count, loff_t * ppos)
{
	ssize_t retval;

	interruptible_sleep_on(&timer_dev_wait);
	retval =
	    copy_to_user(buf, (void *)&frd_latdat, FRD_LATSIZE * sizeof(int));
	if (!retval)
		retval = count;

	return retval;
}

static struct file_operations preempt_dev_fops = {
	.owner = THIS_MODULE,
	.open = preempt_dev_open,
	.release = preempt_dev_close,
	.read = preempt_dev_read,
};

/* create threads and activate IRQ hook */

static int __init frd_init(void)
{
	int i;

	printk("Fast Real Time Domain (c) 2005 Montavista Software, Inc.\n");

	if (register_chrdev(FRD_DEV_MAJOR, FRD_DEV_NAME, &preempt_dev_fops)) {
		printk ("FRD: register_chrdev error '%s'\n", "preempt_dev");
		return -EAGAIN;
	}

	atomic_set(&preempt_dev_busy, 0);

	frd_latency_init();
	frd_start_delay_ticks = frd_start_delay * HZ;
	frd_run = 1;

	init_waitqueue_head(&timer_dev_wait);

	for (i = 0; i < FRD_THREADS; i++) {
		init_waitqueue_head(&frd_thread_wait[i]);
		frd_thread_param[i] = i;
		frd_tid[i] = kernel_thread(frd_thread,
					   (void *)frd_thread_param[i],
					   CLONE_FS | CLONE_FILES | SIGCHLD);
		while (!waitqueue_active(&frd_thread_wait[i]))
			schedule();
	}

	do_gettimeofday(&frd_start_time);
	frd_start_time.tv_sec += frd_start_delay;

	check_frd_func_ptr = check_frd;
#if !defined CONFIG_FRD_USE_TIMER_IRQ
	if (setup_irq(FRD_TIMER_IRQ, &frd_timer_irq)) {
		printk("FRD: Setup IRQ fails\n");
		return -1;
	}
	FRD_TIMER_START;
#endif
	return 0;
}

int
frd_read_procmem(char *buf, char **start, off_t offset,
		 int count, int *eof, void *data)
{
	int i;
	int len = 0;

	len += sprintf(buf,
		       "FRD Irqs %lu T0 wakeups %lu max@exec T0:%lu T1:%lu T2:%lu\n",
		       frd_interrupts - frd_start_delay_ticks + 1,
		       frd_scheds[0],
		       max_p_latency_iteration[0], max_p_latency_iteration[1],
		       max_p_latency_iteration[2]);

	len += sprintf(buf + len, "IRQ latency:%lu/%lu\n",
                       avg_latency[0][IRQ_LATENCY_TYPE],
                       max_latency[0][IRQ_LATENCY_TYPE]);

	len += sprintf(buf + len,
		       "Id  Kcyc  absolute    preempt     schedule     exec-time    sleep-time  DNF/DNS\n");
	len += sprintf(buf + len,
		       "          avg / max   avg / max   avg / max    avg / max    avg / max\n");

	for (i = 0; i < FRD_THREADS; i++) {
		len += sprintf(buf + len,
			       "%2d %6lu  %3lu/%6lu  %3lu/%6lu  %3lu/%6lu  %4lu/%5lu  %5lu/%6lu %3lu/%3lu\n",
			       i,
			       (frd_exec_count[i] / 1000),
			       avg_latency[i][ABS_LATENCY_TYPE],
			       max_latency[i][ABS_LATENCY_TYPE],
			       avg_latency[i][PREEMPT_LATENCY_TYPE],
			       max_latency[i][PREEMPT_LATENCY_TYPE],
			       avg_latency[i][SWITCH_LATENCY_TYPE],
			       max_latency[i][SWITCH_LATENCY_TYPE],
			       avg_latency[i][EXEC_LATENCY_TYPE],
			       max_latency[i][EXEC_LATENCY_TYPE],
			       avg_latency[i][SLEEP_LATENCY_TYPE],
			       max_latency[i][SLEEP_LATENCY_TYPE],
			       deadline_miss_count[i], dns_miss_count[i]);
	}

	return len;
}

static inline u64 u64_div(u64 x, u64 y)
{
	do_div(x, y);
	return x;
}

static void frd_get_cpu_utils(frd_cpu_utils_t * frd_cpu_utils)
{
	int i;
	frd_cpu_utils->user = 0;
	frd_cpu_utils->nice = 0;
	frd_cpu_utils->system = 0;
	frd_cpu_utils->softirq = 0;
	frd_cpu_utils->irq = 0;
	frd_cpu_utils->idle = 0;
	frd_cpu_utils->iowait = 0;
	for_each_online_cpu(i) {
		frd_cpu_utils->user += kstat_cpu(i).cpustat.user;
		frd_cpu_utils->nice += kstat_cpu(i).cpustat.nice;
		frd_cpu_utils->system += kstat_cpu(i).cpustat.system;
		frd_cpu_utils->softirq += kstat_cpu(i).cpustat.softirq;
		frd_cpu_utils->irq += kstat_cpu(i).cpustat.irq;
		frd_cpu_utils->idle += kstat_cpu(i).cpustat.idle;
		frd_cpu_utils->iowait += kstat_cpu(i).cpustat.iowait;
	}
}

static inline percent_t *calc_percent(u64 part, u64 sum, percent_t * percent)
{
        percent->high = u64_div((part * 10000), sum);
        percent->low = percent->high - u64_div(percent->high, 100) * 100;
        percent->high = u64_div(percent->high, 100);

        return percent;
}

static inline cpu_util_percent_t *calc_cpu_utils(frd_cpu_utils_t *
                                                 start_cpu_utils,
                                                 frd_cpu_utils_t *
                                                 end_cpu_utils,
                                                 cpu_util_percent_t *
                                                 util_percent)
{
        u64 user, nice, system, softirq, irq, idle, iowait, sum;

        user = end_cpu_utils->user - start_cpu_utils->user;
        nice = end_cpu_utils->nice - start_cpu_utils->nice;
        system = end_cpu_utils->system - start_cpu_utils->system;
        softirq = end_cpu_utils->softirq - start_cpu_utils->softirq;
        irq = end_cpu_utils->irq - start_cpu_utils->irq;
        idle = end_cpu_utils->idle - start_cpu_utils->idle;
        iowait = end_cpu_utils->iowait - start_cpu_utils->iowait;
        sum = user + nice + system + softirq + irq + idle + iowait;

        calc_percent(user, sum, &util_percent->user);
        calc_percent(nice, sum, &util_percent->nice);
        calc_percent(system, sum, &util_percent->system);
        calc_percent(softirq, sum, &util_percent->softirq);
        calc_percent(irq, sum, &util_percent->irq);
	calc_percent(idle, sum, &util_percent->idle);
        calc_percent(iowait, sum, &util_percent->iowait);

        return util_percent;
}

void frd_latency_log(int frd_id, int latency_type, unsigned long latency)
{
	unsigned long sampletime;

	if (latency_type == PREEMPT_LATENCY_TYPE && frd_id == 0) {
		if (frd_latcnt >= FRD_LATSIZE) {
			frd_latcnt = 0;
		}
		frd_latdat[frd_latcnt++] = latency;
	}

	sampletime = latency / PLAT_BUCKET_SIZE;
	if (sampletime >= PLAT_LAST_BUCKET) {
		buckets[frd_id][latency_type][PLAT_LAST_BUCKET]++;
	} else {
		buckets[frd_id][latency_type][sampletime]++;
	}
	total_samples[frd_id][latency_type]++;
	cur_latency[frd_id][latency_type] = latency;
	if (latency > max_latency[frd_id][latency_type]) {
		max_latency[frd_id][latency_type] = latency;
	}
}

static int
cur_latency_read_proc(char *page_buffer, char **my_first_byte,
		      off_t offset, int length, int *eof, void *data)
{
	int len = 0;
	char *const my_base = page_buffer;
	frd_param_t *frd_param = (frd_param_t *) data;
	int frd_id = frd_param->frd_id;
	int m_type = frd_param->latency_type;

	if (offset == 0) {
		len +=
		    sprintf(my_base+len, "%lu\n", cur_latency[frd_id][m_type]);
		*my_first_byte = page_buffer;
		return len;
	} else {
		*eof = -1;
		return 0;
	}
}

static int
max_latency_read_proc(char *page_buffer, char **my_first_byte,
		      off_t offset, int length, int *eof, void *data)
{
	int len = 0;
	char *const my_base = page_buffer;
	frd_param_t *frd_param = (frd_param_t *) data;
	int frd_id = frd_param->frd_id;
	int m_type = frd_param->latency_type;

	if (offset == 0) {
		len +=
		    sprintf(my_base+len, "%lu\n", max_latency[frd_id][m_type]);
		*my_first_byte = page_buffer;
		return len;
	} else {
		*eof = -1;
		return 0;
	}
}

static int total_sum[FRD_THREADS][LATENCY_TYPES];
static struct proc_dir_entry *entry[FRD_THREADS][LATENCY_TYPES];

static void *l_start(struct seq_file *m, loff_t * pos)
{
	loff_t *index_ptr = kmalloc(sizeof(loff_t), GFP_KERNEL);
	loff_t index = *pos;
	frd_param_t *frd_param = (frd_param_t *) m->private;
	int frd_id = frd_param->frd_id;
	int m_type = frd_param->latency_type;

	if (index_ptr == NULL) {
		return NULL;
	}

	if (index == 0) {
		calc_cpu_utils(&frd_start_cpu_utils, &frd_cur_cpu_utils,
			       &cpu_utils_percent);

		seq_printf(m, "#Calculations done at priority %d.\n"
			   "#elapsed time was %d seconds\n"
			   "#cpu utilization during the test: user %llu.%02llu%%, "
			   "nice %llu.%02llu%%, system %llu.%02llu%%, \n# softirq %llu.%02llu%%, "
			   "irq %llu.%02llu%%, idle %llu.%02llu%%, iowait %llu.%02llu%%\n"
			   "#Maximum preemption latency was %lu microseconds.\n"
			   "#%lu preemptions greater than %lu microseconds. (%lu total samples)\n"
			   "#usecs     preempts\n", MAX_USER_RT_PRIO - 1,
			   frd_elapsed_time(),
			   (unsigned long long)cpu_utils_percent.user.high, (unsigned long long)cpu_utils_percent.user.low,
			   (unsigned long long)cpu_utils_percent.nice.high, (unsigned long long)cpu_utils_percent.nice.low,
			   (unsigned long long)cpu_utils_percent.system.high, (unsigned long long)cpu_utils_percent.system.low,
			   (unsigned long long)cpu_utils_percent.softirq.high, (unsigned long long)cpu_utils_percent.softirq.low,
			   (unsigned long long)cpu_utils_percent.irq.high, (unsigned long long)cpu_utils_percent.irq.low,
			   (unsigned long long)cpu_utils_percent.idle.high, (unsigned long long)cpu_utils_percent.idle.low,
			   (unsigned long long)cpu_utils_percent.iowait.high, (unsigned long long)cpu_utils_percent.iowait.low,
			   (unsigned long)max_latency[frd_id][m_type],
			   (unsigned long)buckets[frd_id][m_type]
			   [PLAT_LAST_BUCKET], (unsigned long) PLAT_UPPER_LIMIT,
			   (unsigned long)total_samples[frd_id][m_type]);
	}
	if (index >= PLAT_BUCKETS) {
		return NULL;
	}

	*index_ptr = index;
	return index_ptr;
}

static void *l_next(struct seq_file *m, void *p, loff_t * pos)
{
	loff_t *index_ptr = p;
	frd_param_t *frd_param = (frd_param_t *) m->private;
	int frd_id = frd_param->frd_id;
	int m_type = frd_param->latency_type;

	if (++*pos >= PLAT_BUCKETS) {
		if (*pos == PLAT_BUCKETS) {
			seq_printf(m,
				   "#total samples tagged as %lu samples logged %d\n",
				   total_samples[frd_id][m_type],
				   total_sum[frd_id][m_type]);
			total_sum[frd_id][m_type] = 0;
		}
		return NULL;
	}
	*index_ptr = *pos;
	return index_ptr;
}

static void l_stop(struct seq_file *m, void *p)
{
	kfree(p);
}

static int l_show(struct seq_file *m, void *p)
{
	int index = *(loff_t *) p;
	frd_param_t *frd_param = (frd_param_t *) m->private;
	int frd_id = frd_param->frd_id;
	int m_type = frd_param->latency_type;

	total_sum[frd_id][m_type] +=
	    buckets[frd_id][m_type][index];
	seq_printf(m, "%5d\t%8lu\n", (index + 1) * PLAT_BUCKET_SIZE,
		   buckets[frd_id][m_type][index]);
	return 0;
}

static struct seq_operations frd_seq_op = {
	.start = l_start,
	.next = l_next,
	.stop = l_stop,
	.show = l_show
};

static int frd_seq_open(struct inode *inode, struct file *file)
{
	int ret;
	struct proc_dir_entry *entry_ptr = NULL;
	int i, j;
	struct seq_file *seq;
	int break_flags = 0;

	entry_ptr = PDE(file->f_dentry->d_inode);
	for (i = 0; i < FRD_THREADS; i++) {
		for (j = 0; j < LATENCY_TYPES; j++) {
			if (entry_ptr->low_ino == entry[i][j]->low_ino) {
				break_flags = 1;
				break;
			}
		}
		if (break_flags == 1) {
			break;
		}
	}
	ret = seq_open(file, &frd_seq_op);
	if (break_flags == 1) {
		seq = (struct seq_file *)file->private_data;
		seq->private = entry[i][j]->data;
	}
	return ret;
}
static struct file_operations frd_seq_fops = {
	.open = frd_seq_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

void frd_latency_init(void)
{
	struct proc_dir_entry *tmp_entry;
	struct proc_dir_entry *frd_dir;
	char procname[64];
	int len = 0;
	int i = 0;
	int frd_id;

	frd_root_dir = proc_mkdir("frd", NULL);
	create_proc_read_entry("summary", 0 /* default mode */ ,
			       frd_root_dir /* parent dir */ , frd_read_procmem,
			       NULL /* client data */ );

	for (frd_id = 0; frd_id < FRD_THREADS; frd_id++) {
		len = sprintf(procname, "%d", frd_id);
		procname[len] = '\0';
		frd_dir = proc_mkdir(procname, frd_root_dir);

		for (i = 0; i < LATENCY_TYPES; i++) {
			frd_params[frd_id][i].frd_id = frd_id;
			frd_params[frd_id][i].latency_type = i;
			tmp_entry = create_proc_read_entry(cur_procnames[i], 0,
							   frd_dir,
							   cur_latency_read_proc,
							   0);
			tmp_entry->data = (void *)&frd_params[frd_id][i];
			tmp_entry = create_proc_read_entry(max_procnames[i], 0,
							   frd_dir,
							   max_latency_read_proc,
							   0);
			tmp_entry->data = (void *)&frd_params[frd_id][i];
			entry[frd_id][i] =
			    create_proc_entry(procnames[i], 0, frd_dir);
			entry[frd_id][i]->data = (void *)&frd_params[frd_id][i];
			entry[frd_id][i]->proc_fops = &frd_seq_fops;
		}
	}
}

static void __exit frd_exit(void)
{
	struct proc_dir_entry *frd_dir;
	char procname[64];
	int frd_id, len, latency_type;

	printk("FRD Module Cleanup\n");

	check_frd_func_ptr = NULL;
	frd_run = 0;
	unregister_chrdev(FRD_DEV_MAJOR, FRD_DEV_NAME);

	for (frd_id = 0; frd_id < FRD_THREADS; frd_id++) {
		if (waitqueue_active(&frd_thread_wait[frd_id]))
			wake_up_process(frd_task[frd_id]);

		frd_dir = entry[frd_id][0]->parent;
		for (latency_type = 0; latency_type < LATENCY_TYPES;
		     latency_type++) {
			remove_proc_entry(procnames[latency_type], frd_dir);
			remove_proc_entry(cur_procnames[latency_type], frd_dir);
			remove_proc_entry(max_procnames[latency_type], frd_dir);
		}

		len = sprintf(procname, "%d", frd_id);
		procname[len] = '\0';
		remove_proc_entry(procname, frd_root_dir);
	}
	remove_proc_entry("summary", frd_root_dir);
	remove_proc_entry("frd", &proc_root);
}

module_param(frd_test_iterations, ulong, S_IRUGO);
MODULE_PARM_DESC(frd_test_iterations, " Number of test iterations to execute (0 for infinite)");

module_param(frd_start_delay, ulong, S_IRUGO);
MODULE_PARM_DESC(frd_start_delay, " Number of seconds before FRD startup");

module_init(frd_init);
module_exit(frd_exit);

MODULE_AUTHOR("Sven-Thosten Dietrich, sdietrich@mvista.com");
MODULE_DESCRIPTION("Fast Real-Time Domain Scheduler");
MODULE_LICENSE("GPL");
