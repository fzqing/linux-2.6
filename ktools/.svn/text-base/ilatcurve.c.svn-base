/*
 * kernel/ilatcurve.c
 *
 * This file implements a sample based interrupt latency curve for Linux.
 *
 * Author: David Singleton dsingleton@mvista.com
 *
 * 2001-2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <asm/system.h>
#include <asm/current.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/init.h>
#include <linux/ilatency.h>
#include <linux/preempt.h>
#include <linux/smp.h>
#include <linux/module.h>
#include <asm/preempt.h>

static DEFINE_PER_CPU(struct ilatency_bucket, ilat_data);

void  inline interrupt_overhead_start()
{
	struct ilatency_bucket *l;

	l = &get_cpu_var(ilat_data);
	l->overhead_start = readclock();
	put_cpu_var(ilat_data);
}

void  inline interrupt_overhead_stop()
{
	long delta;
	struct ilatency_bucket *l;

	l = &get_cpu_var(ilat_data);

	l->overhead_stop = readclock();
	delta = clock_to_usecs(clock_diff(l->overhead_start, 
	    l->overhead_stop));
	if (delta > l->ilatency_delta) {
		l->ilatency_delta = delta;
	}
	put_cpu_var(ilat_data);
}

void inthoff_logentry(unsigned long diff)
{
	unsigned long sampletime = diff;
	struct ilatency_bucket *l;

	l = &get_cpu_var(ilat_data);
	sampletime /= BUCKET_SIZE;
	if (l->maximum_off < diff) {
		l->maximum_off = diff;
	}

	if (sampletime < 0) {
		l->bucket[0]++;
	} else if (sampletime > LAST_BUCKET) {
		l->bucket[LAST_BUCKET]++;
	} else {
		l->bucket[sampletime]++;
	}
	l->total_samples++;
	put_cpu_var(ilat_data);
	return;
}

/*
 * Writing to /proc/maxoff resets the counters
 */
static ssize_t 
ilatcurve_write_proc(struct file * file, const char * buf, size_t count,
loff_t *ppos)
{
	struct ilatency_bucket *l;

	l = &get_cpu_var(ilat_data);

	l->logging = OFF;
        memset(l, 0, sizeof(struct ilatency_bucket));
	l->logging = ON;
	put_cpu_var(ilat_data);
	return count;
}

static int 
ilatcurve_read_proc(char *page_buffer, char **my_first_byte, off_t offset,
	int length, int *eof, void *data)
{
	int len = 0;
	char * const my_base = page_buffer;
	struct ilatency_bucket *l;

	l = &get_cpu_var(ilat_data);

	if (offset == 0) {
		l->logging = OFF;
		l->which = 0;
		/* 
		 * Just been opened so display the header information 
		 */
		len += sprintf(my_base + len,
		    "#%lu samples logged\n#timer measured %lu ticks per usec.\n"
		    "#worst case interrupt overhead %lu microseconds\n"
		    "#maximum interrupt off time %lu microseconds\n"
		    "#usecs  samples", l->total_samples, ticks_per_usec,
		     l->ilatency_delta, l->maximum_off);
	} else if (l->which == -1) {
		*eof = 1;
		l->logging = ON;
		put_cpu_var(ilat_data);
		return 0;
	}

	/* dump the sample log on the screen */
	for (; l->which < BUCKETS && len + ILAT_LINE_SIZE < length; l->which++) {
		/* 
		 * if we can't return thei whole line then don't
		 * return any of it and get it in the next read call.
		 */
		len += sprintf(my_base + len, "\n%5u\t%8lu",
		    (l->which + 1) * BUCKET_SIZE, l->bucket[l->which]);
	}
	if (l->which >= BUCKETS) {
		len += sprintf(my_base + len, " (greater than this time)\n");
		l->which = -1;
	}
	*my_first_byte = page_buffer;
	put_cpu_var(ilat_data);
	return  len;
}

unsigned long ticks_per_usec;
unsigned long tpu;
static int __init tpu_setup(char *str)
{
        tpu = simple_strtoul(str,NULL,0);
        return 1;
}
__setup("tpu=", tpu_setup);
EXPORT_SYMBOL(tpu);

int  __init
ilatcurve_init(void)
{
	struct proc_dir_entry *entry;
	struct ilatency_bucket *l;

	l = &get_cpu_var(ilat_data);

#ifdef ARCH_PREDEFINES_TICKS_PER_USEC
	ticks_per_usec = TICKS_PER_USEC;
#endif
	if (tpu) {
		ticks_per_usec = tpu;
	} else if (ticks_per_usec == 0) {
		printk("%s %s %s\n",
		    "You must pass tpu=XXX on the command line for",
		    "interrupt latency instrumentation to work.\n",
		    "Please see the help section on Interrupt Latency.\n");
	}
	if (ticks_per_usec)
		l->logging = ON;
	else {
		put_cpu_var(ilat_data);
		return -1;
	}
	printk("Interrupt measurement enabled at %lu ticks per usec.\n",
	   	    ticks_per_usec);
#ifdef CONFIG_PROC_FS
	entry = create_proc_read_entry("ilatcurve", S_IWUSR | S_IRUGO, NULL,
	    ilatcurve_read_proc, 0);
	if (entry) {
		entry->write_proc = (write_proc_t *)ilatcurve_write_proc;
	}
#endif
	put_cpu_var(ilat_data);
	return 0;
}

__initcall(ilatcurve_init);



