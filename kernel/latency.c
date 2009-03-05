/*
 *  kernel/latency.c
 *
 *  Copyright (C) 2004 Ingo Molnar
 *  Copyright (C) 2004 William Lee Irwin III
 */

#include <linux/mm.h>
#include <linux/nmi.h>
#include <linux/sched.h>
#include <linux/percpu.h>
#include <linux/config.h>
#include <linux/module.h>
#include <linux/profile.h>
#include <linux/bootmem.h>
#include <linux/version.h>
#include <linux/notifier.h>
#include <linux/kallsyms.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/rtc.h>
#include <asm/uaccess.h>
#include <asm/unistd.h>
#include <asm/preempt.h>

#ifdef __i386__
static inline cycles_t cycles(void)
{
        unsigned long long ret;

        rdtscll(ret);

        return ret;
}
#else
# define cycles() get_cycles()
#endif

#ifdef CONFIG_WAKEUP_TIMING
struct sch_struct {
	raw_spinlock_t trace_lock;
	struct task_struct *task;
	int cpu;
	struct cpu_trace *tr;
} ____cacheline_aligned_in_smp;

static __cacheline_aligned_in_smp struct sch_struct sch =
		{ trace_lock: RAW_SPIN_LOCK_UNLOCKED };

int wakeup_timing = 1;
#endif

#ifdef CONFIG_LATENCY_TIMING

/*
 * Maximum preemption latency measured. Initialize to maximum,
 * we clear it after bootup.
 */
static cycles_t preempt_max_latency = (cycles_t)ULONG_MAX;
static cycles_t preempt_thresh;

/*
 * Should this new latency be reported/recorded?
 */
static int report_latency(cycles_t delta)
{
	if (preempt_thresh) {
		if (delta < preempt_thresh)
			return 0;
	} else {
		if (delta <= preempt_max_latency)
			return 0;
	}
	return 1;
}

/*
 * Track maximum latencies and save the trace:
 */
static __cacheline_aligned_in_smp DECLARE_MUTEX(max_mutex);
/*
 * Sequence count - we record it when starting a measurement and
 * skip the latency if the sequence has changed - some other section
 * did a maximum and could disturb our measurement with serial console
 * printouts, etc. Truly coinciding maximum latencies should be rare
 * and what happens together happens separately as well, so this doesnt
 * decrease the validity of the maximum found:
 */
static __cacheline_aligned_in_smp int max_sequence;

enum trace_type
{
	__TRACE_FIRST_TYPE = 0,

	TRACE_FN,
	TRACE_SPECIAL,
	TRACE_SPECIAL_PID,
	TRACE_CMDLINE,
	TRACE_SYSCALL,
	TRACE_SYSRET,

	__TRACE_LAST_TYPE
};

enum trace_flag_type
{
	TRACE_FLAG_IRQS_OFF		= 0x01,
	TRACE_FLAG_NEED_RESCHED		= 0x02,
	TRACE_FLAG_HARDIRQ		= 0x04,
	TRACE_FLAG_SOFTIRQ		= 0x08,
};


#ifdef CONFIG_LATENCY_TRACE

#define MAX_TRACE (unsigned long)(4096-1)

#define CMDLINE_BYTES 16

/*
 * 32 bytes on 32-bit platforms:
 */
struct trace_entry {
	char type;
	char cpu;
	char flags;
	char preempt_count; // assumes PREEMPT_MASK is 8 bits or less
	int pid;
	cycles_t timestamp;
	union {
		struct {
			unsigned long eip;
			unsigned long parent_eip;
		} fn;
		struct {
			unsigned long eip;
			unsigned long v1, v2, v3;
		} special;
		struct {
			unsigned char str[CMDLINE_BYTES];
		} cmdline;
		struct {
			unsigned int nr;
			unsigned long p1, p2, p3;
		} syscall;
		struct {
			unsigned int ret;
		} sysret;
		struct {
			int __pad3[4];
		} pad;
	} u;
} __attribute__((packed));

#endif

struct cpu_trace {
	atomic_t disabled;
	unsigned long trace_idx;
	cycles_t preempt_timestamp;
	unsigned long critical_start, critical_end;
	int critical_sequence;
	int early_warning;

#ifdef CONFIG_LATENCY_TRACE
	struct trace_entry trace[MAX_TRACE];
	char comm[CMDLINE_BYTES];
	pid_t pid;
	unsigned long uid;
	unsigned long nice;
	unsigned long policy;
	unsigned long rt_priority;
	unsigned long saved_latency;
#endif

} ____cacheline_aligned_in_smp;

static struct cpu_trace cpu_traces[NR_CPUS] ____cacheline_aligned_in_smp;

static unsigned long notrace cycles_to_usecs(cycles_t delta)
{
#if defined(CONFIG_X86) || defined(CONFIG_PPC64)
	do_div(delta, cpu_khz/1000+1);
#elif defined(CONFIG_PPC32)
	delta = mulhwu(tb_to_us, delta);
#elif defined(CONFIG_ARM)
	delta /= TICKS_PER_USEC;
#else
	#error Implement cycles_to_usecs.
#endif

	return (unsigned long) delta;
}

static cycles_t notrace usecs_to_cycles(unsigned long delta)
{
#if defined(CONFIG_X86) || defined(CONFIG_PPC64)
	return (cycles_t) delta * (cycles_t) (cpu_khz/1000+1);
#elif defined(CONFIG_PPC32)
	return delta * us_to_tb;
#elif defined(CONFIG_ARM)
	return delta *= TICKS_PER_USEC;
#else
	#error implement usecs_to_cycles
#endif
}

#ifdef CONFIG_LATENCY_TRACE

int trace_enabled = 1;
int mcount_enabled = 1;
int trace_freerunning = 0;
int trace_print_at_crash = 0;
int trace_verbose = 0;
int trace_all_cpus = 0;

/*
 * user-triggered via gettimeofday(0,1)/gettimeofday(0,0)
 */
int trace_user_triggered = 0;

struct saved_trace_struct {
	int cpu;
	cycles_t first_timestamp, last_timestamp;
	struct cpu_trace traces[NR_CPUS];
} ____cacheline_aligned_in_smp;

/*
 * The current worst-case trace:
 */
static struct saved_trace_struct max_tr;

/*
 * /proc/latency_trace atomicity:
 */
static DECLARE_MUTEX(out_mutex);

static struct saved_trace_struct out_tr;


static void notrace
____trace(int cpu, enum trace_type type, struct cpu_trace *tr,
	  unsigned long eip, unsigned long parent_eip,
	  unsigned long v1, unsigned long v2, unsigned long v3)
{
	struct trace_entry *entry;

#ifdef CONFIG_DEBUG_PREEMPT
//	WARN_ON(!atomic_read(&tr->disabled));
#endif
#if defined(CONFIG_DEBUG_STACKOVERFLOW) && defined(__i386__) 
	/* Debugging check for stack overflow: is there less than 1KB free? */
	{
		long esp;

		__asm__ __volatile__("andl %%esp,%0" :
					"=r" (esp) : "0" (THREAD_SIZE - 1));
		if (unlikely(esp < (sizeof(struct thread_info) + STACK_WARN))) {
			printk("BUG: mcount: stack overflow: %ld [%08lx...%08lx...%08lx]\n",
				esp - sizeof(struct thread_info), (long)&esp, (long)current_thread_info(), (long)current_thread_info() + THREAD_SIZE);
			dump_stack();
		}
	}
#endif

	if (likely(tr->critical_start) || unlikely(trace_user_triggered || trace_all_cpus))
	if (tr->trace_idx < MAX_TRACE) {
		u32 pc = preempt_count();

		entry = tr->trace + tr->trace_idx;
		entry->type = type;
#ifdef CONFIG_SMP
		entry->cpu = cpu;
#endif
		entry->flags = (irqs_disabled() ? TRACE_FLAG_IRQS_OFF : 0) |
			((pc & HARDIRQ_MASK) ? TRACE_FLAG_HARDIRQ : 0) |
			((pc & SOFTIRQ_MASK) ? TRACE_FLAG_SOFTIRQ : 0) |
			(_need_resched() ? TRACE_FLAG_NEED_RESCHED : 0);
		entry->preempt_count = pc & 0xff;
		entry->pid = current->pid;
		entry->timestamp = cycles();

		switch (type) {
		case TRACE_FN:
			entry->u.fn.eip = eip;
			entry->u.fn.parent_eip = parent_eip;
			break;
		case TRACE_SPECIAL:
		case TRACE_SPECIAL_PID:
			entry->u.special.eip = eip;
			entry->u.special.v1 = v1;
			entry->u.special.v2 = v2;
			entry->u.special.v3 = v3;
			break;
		case TRACE_SYSCALL:
			entry->u.syscall.nr = eip;
			entry->u.syscall.p1 = v1;
			entry->u.syscall.p2 = v2;
			entry->u.syscall.p3 = v3;
			break;
		case TRACE_SYSRET:
			entry->u.sysret.ret = eip;
			break;
		case TRACE_CMDLINE:
			memcpy(entry->u.cmdline.str, current->comm, CMDLINE_BYTES);
			break;
		default:
			break;
		}
	}
	tr->trace_idx++;
	if (unlikely(trace_freerunning && (tr->trace_idx >= MAX_TRACE)))
		tr->trace_idx = 0;
}

static inline void notrace
___trace(enum trace_type type, unsigned long eip, unsigned long parent_eip,
		unsigned long v1, unsigned long v2,
			unsigned long v3)
{
	int cpu = _smp_processor_id();
	struct cpu_trace *tr;

	if (unlikely(trace_enabled <= 0))
		return;

	/*
	 * Trace on the CPU where the current highest-prio task
	 * is waiting to become runnable:
	 */
#ifdef CONFIG_WAKEUP_TIMING 
	if (wakeup_timing && !trace_all_cpus) {
		if (!sch.tr || cpu != sch.cpu)
			return;
		tr = sch.tr;
	} else
		tr = cpu_traces + cpu;
#else
	tr = cpu_traces + cpu;
#endif
	if (likely(!atomic_read(&tr->disabled))) {
		atomic_inc(&tr->disabled);
		____trace(cpu, type, tr, eip, parent_eip, v1, v2, v3);
		atomic_dec(&tr->disabled);
	}
}

/*
 * Special, ad-hoc tracepoints:
 */
void notrace trace_special(unsigned long v1, unsigned long v2, unsigned long v3)
{
	___trace(TRACE_SPECIAL, CALLER_ADDR0, 0, v1, v2, v3);
}

EXPORT_SYMBOL(trace_special);

void notrace trace_special_pid(int pid, unsigned long v1, unsigned long v2)
{
	___trace(TRACE_SPECIAL_PID, CALLER_ADDR0, 0, pid, v1, v2);
}

EXPORT_SYMBOL(trace_special_pid);

/*
 * Non-inlined function:
 */
void notrace __trace(unsigned long eip, unsigned long parent_eip)
{
	___trace(TRACE_FN, eip, parent_eip, 0, 0, 0);
}

extern void mcount(void);

EXPORT_SYMBOL(mcount);

void notrace __mcount(void)
{
	___trace(TRACE_FN, CALLER_ADDR1, CALLER_ADDR2, 0, 0, 0);
}

void notrace
sys_call(int nr, unsigned long p1, unsigned long p2, unsigned long p3)
{
	___trace(TRACE_SYSCALL, nr, 0, p1, p2, p3);
}

void notrace sys_ret(int ret)
{
	___trace(TRACE_SYSRET, ret, 0, 0, 0, 0);
}

static void notrace print_name(struct seq_file *m, unsigned long eip)
{
	char namebuf[KSYM_NAME_LEN+1];
	unsigned long size, offset;
	const char *sym_name;
	char *modname;

	/*
	 * Special trace values:
	 */
	if (((long)eip < 10000L) && ((long)eip > -10000L)) {
		seq_printf(m, "(%ld)", eip);
		return;
	}
	sym_name = kallsyms_lookup(eip, &size, &offset, &modname, namebuf);
	if (sym_name)
		seq_puts(m, sym_name);
	else
		seq_printf(m, "<%08lx>", eip);
}

static void notrace print_name_offset(struct seq_file *m, unsigned long eip)
{
	char namebuf[KSYM_NAME_LEN+1];
	unsigned long size, offset;
	const char *sym_name;
	char *modname;

	sym_name = kallsyms_lookup(eip, &size, &offset, &modname, namebuf);
	if (sym_name)
		seq_printf(m, "%s+%#lx/%#lx <%08lx>",
					sym_name, offset, size, eip);
	else
		seq_printf(m, "<%08lx>", eip);
}

static unsigned int out_sequence = -1;
static int pid_to_cmdline_array[PID_MAX_DEFAULT+1];

static void notrace _trace_cmdline(int cpu, struct cpu_trace *tr)
{
	____trace(cpu, TRACE_CMDLINE, tr, 0, 0, 0, 0, 0);
}

void notrace trace_cmdline(void)
{
	___trace(TRACE_CMDLINE, 0, 0, 0, 0, 0);
}

static void construct_pid_to_cmdline(void)
{
	struct cpu_trace *tr = out_tr.traces;
	unsigned int i, j, entries, pid;

	if (tr->critical_sequence == out_sequence)
		return;
	out_sequence = tr->critical_sequence;

	memset(pid_to_cmdline_array, -1, sizeof(int) * (PID_MAX_DEFAULT + 1));

	entries = min(tr->trace_idx, MAX_TRACE-1);

	for (i = 0; i < entries; i++) {
		struct trace_entry *entry = tr->trace + i;

		if (entry->type != TRACE_CMDLINE)
			continue;
		pid = entry->pid;
		if (pid < PID_MAX_DEFAULT) {
			pid_to_cmdline_array[pid] = i;
			/*
			 * Replace space with underline - makes it easier
			 * to process for tools:
			 */
			for (j = 0; j < CMDLINE_BYTES; j++)
				if (entry->u.cmdline.str[j] == ' ')
					entry->u.cmdline.str[j] = '_';
		}
	}
}

char *pid_to_cmdline(unsigned long pid)
{
	struct cpu_trace *tr = out_tr.traces;
	char *cmdline = "<...>";
	int idx;

	pid = min(pid, (unsigned long)PID_MAX_DEFAULT);
	if (!pid)
		return "<idle>";

	if (pid_to_cmdline_array[pid] != -1) {
		idx = pid_to_cmdline_array[pid];
		if (tr->trace[idx].type == TRACE_CMDLINE)
			cmdline = tr->trace[idx].u.cmdline.str;
	}
	return cmdline;
}

struct block_idx {
	int idx[NR_CPUS];
};

/*
 * return the trace entry (position) of the smallest-timestamp
 * one (that is still in the valid idx range):
 */
static int min_idx(struct block_idx *bidx)
{
	cycles_t min_stamp = (cycles_t) -1;
	struct trace_entry *entry;
	int cpu, min_cpu = -1, idx;

	for_each_online_cpu(cpu) {
		idx = bidx->idx[cpu];
		entry = max_tr.traces[cpu].trace + bidx->idx[cpu];
		if (idx > max_tr.traces[cpu].trace_idx)
			continue;
		if (entry->timestamp < min_stamp) {
			min_cpu = cpu;
			min_stamp = entry->timestamp;
		}
	}

	return min_cpu;
}

/*
 * This code is called to construct an output trace from
 * the maximum trace. Having separate traces serves both
 * atomicity (a new max might be saved while we are busy
 * accessing /proc/latency_trace) and it is also used to
 * delay the (expensive) sorting of the output trace by
 * timestamps, in the trace_all_cpus case.
 */
static void update_out_trace(void)
{
	int cpu, sum, entries;
	struct cpu_trace *tmp_max, *tmp_out;
	struct trace_entry *out_entry, *entry;
	struct block_idx bidx = { { 0, } };
	cycles_t stamp, first_stamp = 0, last_stamp = (cycles_t)-1;

	/*
	 * Nasty trick. We might overflow the first array but
	 * there are NR_CPUS of them so we use it as a 'big'
	 * trace buffer.
	 */
	tmp_out = out_tr.traces + 0;
	*tmp_out = max_tr.traces[max_tr.cpu];
	out_tr.cpu = max_tr.cpu;
	out_entry = tmp_out->trace + 0;

	if (!trace_all_cpus) {
		entries = min(tmp_out->trace_idx, MAX_TRACE-1);
		if (!entries)
			return;
		out_tr.first_timestamp = tmp_out->trace[0].timestamp;
		out_tr.last_timestamp = tmp_out->trace[entries-1].timestamp;
		return;
	}
	/*
	 * Find the range of timestamps that are fully traced in
	 * all CPU traces. (since CPU traces can cover a variable
	 * range of time, we have to find the best range.)
	 */
	for_each_online_cpu(cpu) {
		tmp_max = max_tr.traces + cpu;
		stamp = tmp_max->trace[0].timestamp;
		if (stamp > first_stamp)
			first_stamp = stamp;
	}
	/*
	 * Save the timestamp range:
	 */

	tmp_max = max_tr.traces + max_tr.cpu;
	entries = min(tmp_max->trace_idx, MAX_TRACE-1);
	/*
	 * No saved trace yet?
	 */
	if (!entries) {
		out_tr.traces[0].trace_idx = 0;
		return;
	}

	last_stamp = tmp_max->trace[entries-1].timestamp;

	WARN_ON(last_stamp < first_stamp);

	out_tr.first_timestamp = first_stamp;
	out_tr.last_timestamp = last_stamp;


	/*
	 * Fetch trace entries one by one, in increasing timestamp
	 * order. Start at first_stamp, stop at last_stamp:
	 */
	sum = 0;
	for (;;) {
		cpu = min_idx(&bidx);
		if (cpu == -1)
			break;
		entry = max_tr.traces[cpu].trace + bidx.idx[cpu];
		if (entry->timestamp > last_stamp) {
			break;
		}

		bidx.idx[cpu]++;
		if (entry->timestamp < first_stamp)
			continue;
		*out_entry = *entry;
		out_entry++;
		sum++;
	}
	
	WARN_ON(sum > MAX_TRACE*NR_CPUS);
	tmp_out->trace_idx = sum;
}

static void * notrace l_start(struct seq_file *m, loff_t *pos)
{
	loff_t n = *pos;
	unsigned long entries;
	struct cpu_trace *tr;

	down(&out_mutex);
	/*
	 * if the file is being read newly, update the output trace:
	 */
	if (!n) {
		// TODO: use the sequence counter here to optimize
		down(&max_mutex);
		update_out_trace();
		up(&max_mutex);
		if (!out_tr.traces[0].trace_idx) {
			up(&out_mutex);
			return NULL;
		}
		construct_pid_to_cmdline();
	}
	tr = out_tr.traces;
	entries = min(tr->trace_idx, MAX_TRACE);

	if (!n) {
		seq_printf(m, "preemption latency trace v1.1.4 on %s\n", UTS_RELEASE);
		seq_puts(m, "--------------------------------------------------------------------\n");
		seq_printf(m, " latency: %lu 탎, #%lu/%lu, CPU#%d | (M:%s VP:%d, KP:%d, SP:%d HP:%d #P:%d)\n",
			cycles_to_usecs(tr->saved_latency),
			entries, tr->trace_idx, out_tr.cpu,
#if defined(CONFIG_PREEMPT_NONE)
			"server",
#elif defined(CONFIG_PREEMPT_VOLUNTARY)
			"desktop",
#elif defined(CONFIG_PREEMPT_DESKTOP)
			"preempt",
#else
			"rt",
#endif
			voluntary_preemption, kernel_preemption,
			softirq_preemption, hardirq_preemption,
			num_online_cpus());
		seq_puts(m, "    -----------------\n");
		seq_printf(m, "    | task: %.16s-%d (uid:%ld nice:%ld policy:%ld rt_prio:%ld)\n",
			tr->comm, tr->pid, tr->uid, tr->nice,
			tr->policy, tr->rt_priority);
		seq_puts(m, "    -----------------\n");
		if (trace_user_triggered) {
			seq_puts(m, " => started at: ");
			print_name_offset(m, tr->critical_start);
			seq_puts(m, "\n => ended at:   ");
			print_name_offset(m, tr->critical_end);
			seq_puts(m, "\n");
		}
		seq_puts(m, "\n");

		seq_puts(m, "                 _------=> CPU#            \n");
		seq_puts(m, "                / _-----=> irqs-off        \n");
		seq_puts(m, "               | / _----=> need-resched    \n");
		seq_puts(m, "               || / _---=> hardirq/softirq \n");
		seq_puts(m, "               ||| / _--=> preempt-depth   \n");
		seq_puts(m, "               |||| /                      \n");
		seq_puts(m, "               |||||     delay             \n");
		seq_puts(m, "   cmd     pid ||||| time  |   caller      \n");
		seq_puts(m, "      \\   /    |||||   \\   |   /           \n");

	}
	if (n >= entries)
		return NULL;

	return tr->trace + n;
}

static void * notrace l_next(struct seq_file *m, void *p, loff_t *pos)
{
	struct cpu_trace *tr = out_tr.traces;
	unsigned long entries = min(tr->trace_idx, MAX_TRACE);

	if (++*pos >= entries) {
		if (*pos == entries)
			seq_puts(m, "\n\nvim:ft=help\n");
		return NULL;
	}
	return tr->trace + *pos;
}

static void notrace l_stop(struct seq_file *m, void *p)
{
	up(&out_mutex);
}

static void print_timestamp(struct seq_file *m, unsigned long abs_usecs,
						unsigned long rel_usecs)
{
	seq_printf(m, " %4ld탎", abs_usecs);
	if (rel_usecs > 100)
		seq_puts(m, "!: ");
	else if (rel_usecs > 1)
		seq_puts(m, "+: ");
	else
		seq_puts(m, " : ");
}

static void
print_timestamp_short(struct seq_file *m, unsigned long abs_usecs,
			unsigned long rel_usecs)
{
	seq_printf(m, " %4ld탎", abs_usecs);
	if (rel_usecs > 100)
		seq_putc(m, '!');
	else if (rel_usecs > 1)
		seq_putc(m, '+');
	else
		seq_putc(m, ' ');
}

static void
print_generic(struct seq_file *m, struct trace_entry *entry)
{
	int hardirq, softirq;

	seq_printf(m, "%8.8s-%-5d ", pid_to_cmdline(entry->pid), entry->pid);
	seq_printf(m, "%d", entry->cpu);
	seq_printf(m, "%c%c",
		(entry->flags & TRACE_FLAG_IRQS_OFF) ? 'd' : '.',
		(entry->flags & TRACE_FLAG_NEED_RESCHED) ? 'n' : '.');

	hardirq = entry->flags & TRACE_FLAG_HARDIRQ;
	softirq = entry->flags & TRACE_FLAG_SOFTIRQ;
	if (hardirq && softirq)
		seq_putc(m, 'H');
	else {
		if (hardirq)
			seq_putc(m, 'h');
		else {
			if (softirq)
				seq_putc(m, 's');
			else
				seq_putc(m, '.');
		}
	}

	if (entry->preempt_count)
		seq_printf(m, "%x", entry->preempt_count);
	else
		seq_puts(m, ".");
}


static int notrace l_show_fn(struct seq_file *m, unsigned long trace_idx,
		struct trace_entry *entry, struct trace_entry *entry0,
		struct trace_entry *next_entry)
{
	unsigned long abs_usecs, rel_usecs;

	abs_usecs = cycles_to_usecs(entry->timestamp - entry0->timestamp);
	rel_usecs = cycles_to_usecs(next_entry->timestamp - entry->timestamp);

	if (trace_verbose) {
		seq_printf(m, "%16s %5d %d %d %08x %08lx [%016Lu] %ld.%03ldms (+%ld.%03ldms): ",
			pid_to_cmdline(entry->pid),
			entry->pid, entry->cpu, entry->flags,
			entry->preempt_count, trace_idx,
			(unsigned long long)entry->timestamp, abs_usecs/1000,
			abs_usecs % 1000, rel_usecs/1000, rel_usecs % 1000);
		print_name_offset(m, entry->u.fn.eip);
		seq_puts(m, " (");
		print_name_offset(m, entry->u.fn.parent_eip);
		seq_puts(m, ")\n");
	} else {
		print_generic(m, entry);
		print_timestamp(m, abs_usecs, rel_usecs);
		print_name(m, entry->u.fn.eip);
		seq_puts(m, " (");
		print_name(m, entry->u.fn.parent_eip);
		seq_puts(m, ")\n");
	}
	return 0;
}

static int notrace l_show_special(struct seq_file *m, unsigned long trace_idx,
		struct trace_entry *entry, struct trace_entry *entry0,
		struct trace_entry *next_entry)
{
	unsigned long abs_usecs, rel_usecs;

	abs_usecs = cycles_to_usecs(entry->timestamp - entry0->timestamp);
	rel_usecs = cycles_to_usecs(next_entry->timestamp - entry->timestamp);

	print_generic(m, entry);
	print_timestamp(m, abs_usecs, rel_usecs);
	if (trace_verbose)
		print_name_offset(m, entry->u.special.eip);
	else
		print_name(m, entry->u.special.eip);
	seq_printf(m, " (%lx %lx %lx)\n",
		entry->u.special.v1, entry->u.special.v2, entry->u.special.v3);

	return 0;
}

static int notrace
l_show_special_pid(struct seq_file *m, unsigned long trace_idx,
		struct trace_entry *entry, struct trace_entry *entry0,
		struct trace_entry *next_entry)
{
	unsigned long abs_usecs, rel_usecs;
	unsigned int pid;

	pid = entry->u.special.v1;

	abs_usecs = cycles_to_usecs(entry->timestamp - entry0->timestamp);
	rel_usecs = cycles_to_usecs(next_entry->timestamp - entry->timestamp);

	print_generic(m, entry);
	print_timestamp(m, abs_usecs, rel_usecs);
	if (trace_verbose)
		print_name_offset(m, entry->u.special.eip);
	else
		print_name(m, entry->u.special.eip);
	seq_printf(m, " <%.8s-%d> (%lx %lx): ",
		pid_to_cmdline(pid), pid,
		entry->u.special.v2, entry->u.special.v3);

	seq_puts(m, "\n");

	return 0;
}


static int notrace l_show_cmdline(struct seq_file *m, unsigned long trace_idx,
		struct trace_entry *entry, struct trace_entry *entry0,
		struct trace_entry *next_entry)
{
	unsigned long abs_usecs, rel_usecs;

	if (!trace_verbose)
		return 0;

	abs_usecs = cycles_to_usecs(entry->timestamp - entry0->timestamp);
	rel_usecs = cycles_to_usecs(next_entry->timestamp - entry->timestamp);

	seq_printf(m,
		"[ => %16s ] %ld.%03ldms (+%ld.%03ldms)\n",
			entry->u.cmdline.str,
			abs_usecs/1000, abs_usecs % 1000,
			rel_usecs/1000, rel_usecs % 1000);

	return 0;
}

extern unsigned long sys_call_table[NR_syscalls];

static int notrace l_show_syscall(struct seq_file *m, unsigned long trace_idx,
		struct trace_entry *entry, struct trace_entry *entry0,
		struct trace_entry *next_entry)
{
	unsigned long abs_usecs, rel_usecs;
	unsigned int nr;

	abs_usecs = cycles_to_usecs(entry->timestamp - entry0->timestamp);
	rel_usecs = cycles_to_usecs(next_entry->timestamp - entry->timestamp);

	print_generic(m, entry);
	print_timestamp_short(m, abs_usecs, rel_usecs);

	seq_puts(m, "> ");
	nr = entry->u.syscall.nr;
	if (nr < NR_syscalls)
		print_name(m, sys_call_table[entry->u.syscall.nr]);
	else
		seq_puts(m, "<badsys>");

	seq_printf(m, " (%08lx %08lx %08lx)\n",
		entry->u.syscall.p1, entry->u.syscall.p2, entry->u.syscall.p3);

	return 0;
}

static int notrace l_show_sysret(struct seq_file *m, unsigned long trace_idx,
		struct trace_entry *entry, struct trace_entry *entry0,
		struct trace_entry *next_entry)
{
	unsigned long abs_usecs, rel_usecs;

	abs_usecs = cycles_to_usecs(entry->timestamp - entry0->timestamp);
	rel_usecs = cycles_to_usecs(next_entry->timestamp - entry->timestamp);

	print_generic(m, entry);
	print_timestamp_short(m, abs_usecs, rel_usecs);

	seq_printf(m, "< (%d)\n", entry->u.sysret.ret);

	return 0;
}


static int notrace l_show(struct seq_file *m, void *p)
{
	struct cpu_trace *tr = out_tr.traces;
	struct trace_entry *entry, *entry0, *next_entry;
	unsigned long trace_idx;

	entry = p;
	if (entry->timestamp < out_tr.first_timestamp)
		return 0;
	if (entry->timestamp > out_tr.last_timestamp)
		return 0;

	entry0 = tr->trace;
	trace_idx = entry - entry0;

	if (trace_idx + 1 < tr->trace_idx)
		next_entry = entry + 1;
	else
		next_entry = entry;

	if (trace_verbose)
		seq_printf(m, "(T%d/#%ld) ", entry->type, trace_idx);

	switch (entry->type) {
		case TRACE_FN:
			l_show_fn(m, trace_idx, entry, entry0, next_entry);
			break;
		case TRACE_SPECIAL:
			l_show_special(m, trace_idx, entry, entry0, next_entry);
			break;
		case TRACE_SPECIAL_PID:
			l_show_special_pid(m, trace_idx, entry, entry0, next_entry);
			break;
		case TRACE_CMDLINE:
			l_show_cmdline(m, trace_idx, entry, entry0, next_entry);
			break;
		case TRACE_SYSCALL:
			l_show_syscall(m, trace_idx, entry, entry0, next_entry);
			break;
		case TRACE_SYSRET:
			l_show_sysret(m, trace_idx, entry, entry0, next_entry);
			break;
		default:
			seq_printf(m, "unknown trace type %d\n", entry->type);
	}
	return 0;
}

struct seq_operations latency_trace_op = {
	.start	= l_start,
	.next	= l_next,
	.stop	= l_stop,
	.show	= l_show
};

static void copy_trace(struct cpu_trace *save, struct cpu_trace *tr)
{
	/* free-running needs reordering */
	if (trace_freerunning) {
		int i, idx, idx0 = tr->trace_idx;

		for (i = 0; i < MAX_TRACE; i++) {
			idx = (idx0 + i) % MAX_TRACE;
			save->trace[i] = tr->trace[idx];
		}
		save->trace_idx = MAX_TRACE-1;
	} else {
		save->trace_idx = tr->trace_idx;

		memcpy(save->trace, tr->trace,
			min(save->trace_idx + 1, MAX_TRACE) *
					sizeof(struct trace_entry));
	}
}

static void update_max_tr(struct cpu_trace *tr)
{
	struct cpu_trace *save;
	int this_cpu = smp_processor_id(), cpu, all_cpus = 0;

	WARN_ON(!preempt_count() && !irqs_disabled());

	max_tr.cpu = this_cpu;
	save = max_tr.traces + this_cpu;

	if ((wakeup_timing || trace_user_triggered) && trace_all_cpus) {
		all_cpus = 1;
		for_each_online_cpu(cpu)
			atomic_inc(&cpu_traces[cpu].disabled);
	}

	save->saved_latency = preempt_max_latency;
	save->preempt_timestamp = tr->preempt_timestamp;
	save->critical_start = tr->critical_start;
	save->critical_end = tr->critical_end;
	save->critical_sequence = tr->critical_sequence;

	memcpy(save->comm, current->comm, CMDLINE_BYTES);
	save->pid = current->pid;
	save->uid = current->uid;
	save->nice = current->static_prio - 20 - MAX_RT_PRIO;
	save->policy = current->policy;
	save->rt_priority = current->rt_priority;

	if (all_cpus) {
		for_each_online_cpu(cpu) {
			copy_trace(max_tr.traces + cpu, cpu_traces + cpu);
			atomic_dec(&cpu_traces[cpu].disabled);
		}
	} else
		copy_trace(save, tr);
}

#else /* !LATENCY_TRACE */

static inline void notrace
____trace(int cpu, enum trace_type type, struct cpu_trace *tr,
	  unsigned long eip, unsigned long parent_eip,
	  unsigned long v1, unsigned long v2, unsigned long v3)
{
}

static inline void notrace
___trace(enum trace_type type, unsigned long eip, unsigned long parent_eip,
		unsigned long v1, unsigned long v2,
			unsigned long v3)
{
}

static inline void notrace __trace(unsigned long eip, unsigned long parent_eip)
{
}

static inline void update_max_tr(struct cpu_trace *tr)
{
}

static inline void notrace _trace_cmdline(int cpu, struct cpu_trace *tr)
{
}

#endif

static int setup_preempt_thresh(char *s)
{
	int thresh;

	get_option(&s, &thresh);
	if (thresh > 0) {
		preempt_thresh = usecs_to_cycles(thresh);
		printk("Preemption threshold = %u 탎\n", thresh);
	}
	return 1;
}
__setup("preempt_thresh=", setup_preempt_thresh);

#ifdef CONFIG_CRITICAL_TIMING

static void notrace
check_critical_timing(int cpu, struct cpu_trace *tr, unsigned long parent_eip)
{
	unsigned long latency, t0, t1;
	cycles_t T1, T0, delta;

	if (trace_user_triggered)
		return;
	/*
	 * usecs conversion is slow so we try to delay the conversion
	 * as long as possible:
	 */
	T0 = tr->preempt_timestamp;
	T1 = cycles();
	delta = T1-T0;

	if (!report_latency(delta))
		goto out;
	____trace(cpu, TRACE_FN, tr, CALLER_ADDR0, parent_eip, 0, 0, 0);
	/*
	 * Update the timestamp, because the trace entry above
	 * might change it (it can only get larger so the latency
	 * is fair to be reported):
	 */
	T1 = cycles();
	delta = T1-T0;

	if (tr->critical_sequence != max_sequence || down_trylock(&max_mutex))
		goto out;

	preempt_max_latency = delta;
	t0 = cycles_to_usecs(T0);
	t1 = cycles_to_usecs(T1);
	latency = cycles_to_usecs(delta);

	tr->critical_end = parent_eip;

	update_max_tr(tr);

	if (preempt_thresh)
		printk("(%16s-%-5d|#%d): %lu 탎 critical section "
			"violates %lu 탎 threshold.\n"
			" => started at timestamp %lu: ",
				current->comm, current->pid,
				_smp_processor_id(),
				latency, cycles_to_usecs(preempt_thresh), t0);
	else
		printk("(%16s-%-5d|#%d): new %lu 탎 maximum-latency "
			"critical section.\n => started at timestamp %lu: ",
				current->comm, current->pid,
				_smp_processor_id(),
				latency, t0);

	print_symbol("<%s>\n", tr->critical_start);
	printk(" =>   ended at timestamp %lu: ", t1);
	print_symbol("<%s>\n", tr->critical_end);
	dump_stack();
	t1 = cycles_to_usecs(cycles());
	printk(" =>   dump-end timestamp %lu\n\n", t1);

	max_sequence++;

	up(&max_mutex);
out:
	tr->critical_sequence = max_sequence;
	tr->preempt_timestamp = cycles();
	tr->early_warning = 0;
	tr->trace_idx = 0;
	_trace_cmdline(cpu, tr);
	____trace(cpu, TRACE_FN, tr, CALLER_ADDR0, parent_eip, 0, 0, 0);
}

void notrace touch_critical_timing(void)
{
	int cpu = _smp_processor_id();
	struct cpu_trace *tr = cpu_traces + cpu;

	if (!tr->critical_start || atomic_read(&tr->disabled) ||
			trace_user_triggered || wakeup_timing)
		return;

	if (preempt_count() > 0 && tr->critical_start) {
		atomic_inc(&tr->disabled);
		check_critical_timing(cpu, tr, CALLER_ADDR0);
		tr->critical_start = CALLER_ADDR0;
		tr->critical_sequence = max_sequence;
		atomic_dec(&tr->disabled);
	}
}
EXPORT_SYMBOL(touch_critical_timing);

void notrace stop_critical_timing(void)
{
	struct cpu_trace *tr = cpu_traces + _smp_processor_id();

	tr->critical_start = 0;
}
EXPORT_SYMBOL(stop_critical_timing);

static inline void notrace
__start_critical_timing(unsigned long eip, unsigned long parent_eip)
{
	int cpu = _smp_processor_id();
	struct cpu_trace *tr = cpu_traces + cpu;

	if (tr->critical_start || atomic_read(&tr->disabled) ||
			trace_user_triggered || wakeup_timing)
		return;

	atomic_inc(&tr->disabled);

	tr->critical_sequence = max_sequence;
	tr->preempt_timestamp = cycles();
	tr->critical_start = eip;
	tr->trace_idx = 0;
	_trace_cmdline(cpu, tr);
	____trace(cpu, TRACE_FN, tr, eip, parent_eip, 0, 0, 0);

	atomic_dec(&tr->disabled);
}

static inline void notrace
__stop_critical_timing(unsigned long eip, unsigned long parent_eip)
{
	int cpu = _smp_processor_id();
	struct cpu_trace *tr = cpu_traces + cpu;

	if (!tr->critical_start || atomic_read(&tr->disabled) ||
			trace_user_triggered || wakeup_timing)
		return;

	atomic_inc(&tr->disabled);
	____trace(cpu, TRACE_FN, tr, eip, parent_eip, 0, 0, 0);
	check_critical_timing(cpu, tr, eip);
	tr->critical_start = 0;
	atomic_dec(&tr->disabled);
}

#endif

#ifdef CONFIG_CRITICAL_IRQSOFF_TIMING

#ifdef CONFIG_CRITICAL_PREEMPT_TIMING
# define irqs_off_preempt_count() preempt_count()
#else
# define irqs_off_preempt_count() 0
#endif

void notrace trace_irqs_off_lowlevel(void)
{
	unsigned long flags;

	local_save_flags(flags);

	if (system_state != SYSTEM_RUNNING) return;

	if (!irqs_off_preempt_count() && irqs_disabled_flags(flags))
		__start_critical_timing(CALLER_ADDR0, 0);
}

void notrace trace_irqs_off(void)
{
	unsigned long flags;

	local_save_flags(flags);

	if (system_state != SYSTEM_RUNNING) return;

	if (!irqs_off_preempt_count() && irqs_disabled_flags(flags))
		__start_critical_timing(CALLER_ADDR0, CALLER_ADDR1);
}

EXPORT_SYMBOL(trace_irqs_off);

void notrace trace_irqs_on(void)
{
	unsigned long flags;

	local_save_flags(flags);

	if (!irqs_off_preempt_count() && irqs_disabled_flags(flags))
		__stop_critical_timing(CALLER_ADDR0, CALLER_ADDR1);
}

EXPORT_SYMBOL(trace_irqs_on);

#endif

#endif /* LATENCY_TIMING */

#if defined(CONFIG_DEBUG_PREEMPT) || defined(CONFIG_CRITICAL_TIMING)

void notrace add_preempt_count(int val)
{
	unsigned long eip = CALLER_ADDR0;
	unsigned long parent_eip = CALLER_ADDR1;

#ifdef CONFIG_DEBUG_PREEMPT
	/*
	 * Underflow?
	 */
	BUG_ON(((int)preempt_count() < 0));
	/*
	 * Spinlock count overflowing soon?
	 */
	BUG_ON((preempt_count() & PREEMPT_MASK) >= PREEMPT_MASK-10);
#endif

	preempt_count() += val;
#ifdef CONFIG_PREEMPT_TRACE
	if (val <= 10) {
		unsigned int idx = preempt_count() & PREEMPT_MASK;
		if (idx < MAX_PREEMPT_TRACE) {
			current->preempt_trace_eip[idx] = eip;
			current->preempt_trace_parent_eip[idx] = parent_eip;
		}
	}
#endif
#ifdef CONFIG_CRITICAL_PREEMPT_TIMING
	{
#ifdef CONFIG_CRITICAL_IRQSOFF_TIMING
		unsigned long flags;

		local_save_flags(flags);

		if (!irqs_disabled_flags(flags))
#endif
			if (preempt_count() == val)
				__start_critical_timing(eip, parent_eip);
	}
#endif
	(void)eip, (void)parent_eip;
}
EXPORT_SYMBOL(add_preempt_count);

void notrace sub_preempt_count(int val)
{
#ifdef CONFIG_DEBUG_PREEMPT
	/*
	 * Underflow?
	 */
	BUG_ON(unlikely(val > preempt_count()));

	/*
	 * Is the spinlock portion underflowing?
	 */
	BUG_ON((val < PREEMPT_MASK) && !(preempt_count() & PREEMPT_MASK));
#endif

#ifdef CONFIG_CRITICAL_PREEMPT_TIMING
	{
#ifdef CONFIG_CRITICAL_IRQSOFF_TIMING
		unsigned long flags;

		local_save_flags(flags);

		if (!irqs_disabled_flags(flags))
#endif
			if (preempt_count() == val)
				__stop_critical_timing(CALLER_ADDR0, CALLER_ADDR1);
	}
#endif
	preempt_count() -= val;
}

EXPORT_SYMBOL(sub_preempt_count);

#endif

/*
 * Wakeup latency timing/tracing. We get upcalls from the scheduler
 * when a task is being woken up and we time/trace it until it gets
 * to a CPU - or an even-higher-prio task supercedes it. (in that
 * case we throw away the currently traced task - we dont try to
 * handle nesting, that simplifies things significantly)
 */
#ifdef CONFIG_WAKEUP_TIMING

static void notrace
check_wakeup_timing(struct cpu_trace *tr, unsigned long parent_eip)
{
	unsigned long latency;
	unsigned long t0, t1;
	cycles_t T0, T1, delta;

	if (trace_user_triggered)
		return;

	atomic_inc(&tr->disabled);
	if (atomic_read(&tr->disabled) != 1)
		goto out;

	T0 = tr->preempt_timestamp;
	T1 = cycles();
	delta = T1-T0;

	if (!report_latency(delta))
		goto out;

	____trace(smp_processor_id(), TRACE_FN, tr, CALLER_ADDR0, parent_eip, 0, 0, 0);
	T1 = cycles();
	delta = T1-T0;

	if (tr->critical_sequence != max_sequence || down_trylock(&max_mutex))
		goto out;

	preempt_max_latency = delta;
	t0 = cycles_to_usecs(T0);
	t1 = cycles_to_usecs(T1);
	latency = cycles_to_usecs(delta);

	tr->critical_end = parent_eip;

	update_max_tr(tr);

	if (preempt_thresh)
		printk("(%16s-%-5d|#%d): %lu 탎 wakeup latency "
			"violates %lu 탎 threshold.\n",
				current->comm, current->pid,
				_smp_processor_id(), latency,
				cycles_to_usecs(preempt_thresh));
	else
		printk("(%16s-%-5d|#%d): new %lu 탎 maximum-latency "
			"wakeup.\n", current->comm, current->pid,
				_smp_processor_id(), latency);

	max_sequence++;

	up(&max_mutex);
out:
	atomic_dec(&tr->disabled);
}

/*
 * Start wakeup latency tracing - called with the runqueue held
 * and interrupts disabled:
 */
void __trace_start_sched_wakeup(struct task_struct *p)
{
	struct cpu_trace *tr;
	int cpu;

	if (trace_user_triggered || !wakeup_timing)
		return;

	spin_lock(&sch.trace_lock);
	if (sch.task && (sch.task->prio >= p->prio))
		goto out_unlock;
	/*
	 * New highest-prio task just woke up - start tracing:
	 */
	sch.task = p;
	sch.cpu = task_cpu(p);
	/*
	 * We keep using this CPU's trace buffer even if the task
	 * gets migrated to another CPU. Tracing only happens on
	 * the CPU that 'owns' the highest-prio task so it's
	 * fundamentally single-threaded.
	 */
	sch.tr = tr = cpu_traces + sch.cpu;
	if (trace_all_cpus)
		for_each_online_cpu(cpu)
			cpu_traces[cpu].trace_idx = 0;
	else
		tr->trace_idx = 0;

	tr->critical_sequence = max_sequence;
	tr->preempt_timestamp = cycles();
	tr->critical_start = CALLER_ADDR0;
	_trace_cmdline(_smp_processor_id(), tr);
	mcount();
out_unlock:
	spin_unlock(&sch.trace_lock);
}

void trace_stop_sched_switched(struct task_struct *p)
{
	struct cpu_trace *tr;
	unsigned long flags;

	if (trace_user_triggered || !wakeup_timing)
		return;

	trace_special_pid(p->pid, p->prio, 0);

	spin_lock_irqsave(&sch.trace_lock, flags);
	if (p == sch.task) {
		sch.task = NULL;
		tr = sch.tr;
		sch.tr = NULL;
		WARN_ON(!tr);
		/*
		 * Somewhat racy but safer - the printks within
		 * check_wakeup_timing() can call back into the
		 * wakup-timing code and deadlock:
		 */
		spin_unlock(&sch.trace_lock);
		check_wakeup_timing(tr, CALLER_ADDR0);
		local_irq_restore(flags);
	} else {
		if (sch.task)
			trace_special_pid(sch.task->pid, sch.task->prio, p->prio);
		if (sch.task && (sch.task->prio >= p->prio))
			sch.task = NULL;
		spin_unlock_irqrestore(&sch.trace_lock, flags);
	}
}

void trace_change_sched_cpu(struct task_struct *p, int new_cpu)
{
	unsigned long flags;

	if (!wakeup_timing)
		return;

	trace_special(task_cpu(p), task_cpu(p), new_cpu);
	spin_lock_irqsave(&sch.trace_lock, flags);
	if (p == sch.task && task_cpu(p) != new_cpu) {
		sch.cpu = new_cpu;
		trace_special(task_cpu(p), new_cpu, 0);
	}
	spin_unlock_irqrestore(&sch.trace_lock, flags);
}

#endif

#ifdef CONFIG_LATENCY_TRACE

long user_trace_start(void)
{
	struct cpu_trace *tr;
	int cpu;

	if (!trace_user_triggered || trace_print_at_crash)
		return -EINVAL;

	if (down_trylock(&max_mutex))
		return -EAGAIN;

	preempt_disable();
	tr = cpu_traces + smp_processor_id();

#ifdef CONFIG_WAKEUP_TIMING
	if (wakeup_timing) {
		unsigned long flags;
		spin_lock_irqsave(&sch.trace_lock, flags);
		sch.task = current;
		sch.cpu = smp_processor_id();
		sch.tr = tr;
		spin_unlock_irqrestore(&sch.trace_lock, flags);
	}
#endif

	if (trace_all_cpus)
		for_each_online_cpu(cpu)
			cpu_traces[cpu].trace_idx = 0;
	else
		tr->trace_idx = 0;
	tr->critical_sequence = max_sequence;
	tr->preempt_timestamp = cycles();
	_trace_cmdline(_smp_processor_id(), tr);
	mcount();
	preempt_enable();

	up(&max_mutex);

	return 0;
}

long user_trace_stop(void)
{
	unsigned long latency;
	struct cpu_trace *tr;
	cycles_t delta;

	if (!trace_user_triggered || trace_print_at_crash)
		return -EINVAL;

	preempt_disable();
	mcount();

#ifdef CONFIG_WAKEUP_TIMING
	if (wakeup_timing) {
		unsigned long flags;
		spin_lock_irqsave(&sch.trace_lock, flags);
		if (current != sch.task) {
			spin_unlock_irqrestore(&sch.trace_lock, flags);
			preempt_enable();
			return -EINVAL;
		}
		sch.task = NULL;
		tr = sch.tr;
		sch.tr = NULL;
		spin_unlock_irqrestore(&sch.trace_lock, flags);
	} else
#endif
		tr = cpu_traces + smp_processor_id();

	atomic_inc(&tr->disabled);
	if (tr->preempt_timestamp) {
		delta = cycles() - tr->preempt_timestamp;
		if (!report_latency(delta))
			goto out;
		if (tr->critical_sequence != max_sequence ||
						down_trylock(&max_mutex))
			goto out;

		preempt_max_latency = delta;
		update_max_tr(tr);

		latency = cycles_to_usecs(delta);

		if (preempt_thresh)
			printk("(%16s-%-5d|#%d): %lu 탎 user-latency "
				"violates %lu 탎 threshold.\n",
					current->comm, current->pid,
					_smp_processor_id(), latency,
					cycles_to_usecs(preempt_thresh));
		else
			printk("(%16s-%-5d|#%d): new %lu 탎 user-latency.\n",
				current->comm, current->pid,
					_smp_processor_id(), latency);

		max_sequence++;
		up(&max_mutex);
out:
		tr->preempt_timestamp = 0;
	}
	atomic_dec(&tr->disabled);
	preempt_enable();

	return 0;
}

EXPORT_SYMBOL(user_trace_stop);

void stop_trace(void)
{
	if (trace_print_at_crash)
		trace_enabled = -1;
}

static void notrace printk_name(unsigned long eip)
{
	char namebuf[KSYM_NAME_LEN+1];
	unsigned long size, offset;
	const char *sym_name;
	char *modname;

	sym_name = kallsyms_lookup(eip, &size, &offset, &modname, namebuf);
	if (sym_name)
		printk("%s+%#lx/%#lx", sym_name, offset, size);
	else
		printk("<%08lx>", eip);
}

static void print_entry(struct trace_entry *entry, struct trace_entry *entry0,
			struct trace_entry *next_entry)
{
	unsigned long abs_usecs, rel_usecs;
	int hardirq, softirq;

	abs_usecs = cycles_to_usecs(entry->timestamp - entry0->timestamp);
	rel_usecs = cycles_to_usecs(next_entry->timestamp - entry->timestamp);

	printk("%-5d ", entry->pid);

	printk("%c%c",
		(entry->flags & TRACE_FLAG_IRQS_OFF) ? 'd' : '.',
		(entry->flags & TRACE_FLAG_NEED_RESCHED) ? 'n' : '.');

	hardirq = entry->flags & TRACE_FLAG_HARDIRQ;
	softirq = entry->flags & TRACE_FLAG_SOFTIRQ;
	if (hardirq && softirq)
		printk("H");
	else {
		if (hardirq)
			printk("h");
		else {
			if (softirq)
				printk("s");
			else
				printk(".");
		}
	}

	printk(":%d %ld.%03ldms: ",
		entry->preempt_count, abs_usecs/1000, abs_usecs % 1000);

	printk_name(entry->u.fn.eip);
	printk("  <= (");
	printk_name(entry->u.fn.parent_eip);
	printk(")\n");
}

void print_last_trace(void)
{
	unsigned int idx0, idx, i;
	struct cpu_trace *tr;
	struct trace_entry *entry0, *entry, *next_entry;

	if (trace_enabled != -1)
		return;

	preempt_disable();
	tr = cpu_traces + smp_processor_id();

	printk("Last %ld trace entries:\n", MAX_TRACE);
	idx0 = tr->trace_idx;
	printk("curr idx: %d\n", idx0);
	if (idx0 >= MAX_TRACE)
		idx0 = MAX_TRACE-1;
	idx = idx0;
	entry0 = tr->trace + idx0;

	for (i = 0; i < MAX_TRACE; i++) {
		entry = tr->trace + idx;
		idx++;
		if (idx == MAX_TRACE)
			idx = 0;
		next_entry = tr->trace + idx;
		if (entry->type == TRACE_FN)
			print_entry(entry, entry0, next_entry);
	}
	trace_print_at_crash = 0;

	preempt_enable();
}

#ifdef CONFIG_SMP
/*
 * On SMP, try to 'peek' on other CPU's traces and record them
 * in this CPU's trace. This way we get a rough idea about what's
 * going on there, without the overhead of global tracing.
 *
 * (no need to make this PER_CPU, we bounce it around anyway.)
 */
unsigned long nmi_eips[NR_CPUS];
unsigned long nmi_flags[NR_CPUS];

void notrace nmi_trace(unsigned long eip, unsigned long parent_eip,
			unsigned long flags)
{
	int cpu, this_cpu = smp_processor_id();

	__trace(eip, parent_eip);

	nmi_eips[this_cpu] = parent_eip;
	nmi_flags[this_cpu] = flags;
	for (cpu = 0; cpu < NR_CPUS; cpu++)
		if (cpu_online(cpu) && cpu != this_cpu) {
			__trace(eip, nmi_eips[cpu]);
			__trace(eip, nmi_flags[cpu]);
		}
}
#else
/*
 * On UP, NMI tracing is quite simple:
 */
void notrace nmi_trace(unsigned long eip, unsigned long parent_eip,
			unsigned long flags)
{
	__trace(eip, parent_eip);
}
#endif

#endif

#ifdef CONFIG_PREEMPT_TRACE

static void print_preempt_trace(struct task_struct *task)
{
	unsigned int count = task->thread_info->preempt_count;
	unsigned int i, lim = count & PREEMPT_MASK;
	if (lim >= MAX_PREEMPT_TRACE)
		lim = MAX_PREEMPT_TRACE-1;
	printk("---------------------------\n");
	printk("| preempt count: %08x ]\n", count);
	printk("| %d-level deep critical section nesting:\n", lim);
	printk("----------------------------------------\n");
	for (i = 1; i <= lim; i++) {
		printk(".. [<%08lx>] .... ", task->preempt_trace_eip[i]);
		print_symbol("%s\n", task->preempt_trace_eip[i]);
		printk(".....[<%08lx>] ..   ( <= ",
				task->preempt_trace_parent_eip[i]);
		print_symbol("%s)\n", task->preempt_trace_parent_eip[i]);
	}
	printk("\n");
}

#endif

#if defined(CONFIG_PREEMPT_TRACE) || defined(CONFIG_LATENCY_TRACE)
void print_traces(struct task_struct *task)
{
	preempt_disable();
#ifdef CONFIG_PREEMPT_TRACE
	print_preempt_trace(task);
#endif
#ifdef CONFIG_LATENCY_TRACE
	print_last_trace();
#endif
	preempt_enable();
}
#endif

#ifdef CONFIG_LATENCY_TIMING

static int preempt_read_proc(char *page, char **start, off_t off,
			      int count, int *eof, void *data)
{
	cycles_t *max = data;

	return sprintf(page, "%ld\n", cycles_to_usecs(*max));
}

static int preempt_write_proc(struct file *file, const char __user *buffer,
			       unsigned long count, void *data)
{
	unsigned int c, done = 0, val, sum = 0;
	cycles_t *max = data;

	while (count) {
		if (get_user(c, buffer))
			return -EFAULT;
		val = c - '0';
		buffer++;
		done++;
		count--;
		if (c == 0 || c == '\n')
			break;
		if (val > 9)
			return -EINVAL;
		sum *= 10;
		sum += val;
	}
	*max = usecs_to_cycles(sum);
	return done;
}

static __init int latency_init(void)
{
	struct proc_dir_entry *entry;

	entry = create_proc_entry("sys/kernel/preempt_max_latency", 0600, NULL);

	entry->nlink = 1;
	entry->data = &preempt_max_latency;
	entry->read_proc = preempt_read_proc;
	entry->write_proc = preempt_write_proc;

	entry = create_proc_entry("sys/kernel/preempt_thresh", 0600, NULL);

	entry->nlink = 1;
	entry->data = &preempt_thresh;
	entry->read_proc = preempt_read_proc;
	entry->write_proc = preempt_write_proc;

	return 0;
}
__initcall(latency_init);

#endif

