/*
 * kernel/imaxoff.c
 *
 * This file implements the tracking of the places where interrupts
 * are turned off.
 *
 * Author: David Singleton dsingleton@mvista.com 
 *
 * 2001-2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/module.h>
#include <linux/time.h>
#include <linux/proc_fs.h>
#include <linux/init.h>
#include <linux/ilatency.h>
#include <asm/system.h>
#include <asm/preempt.h>

static DEFINE_PER_CPU(struct ilatency_log, ilat_log);

/* 
 * If this is the outer most irq_disable we log it.  If it's a nested 
 * irq_disable trying to turn interupts off and they are already off
 * we don't note the time or call to irq_disable.
 * To keep our accounting straight, we set latency.sync to 1 in
 * this routine and to zero in local_irq_enable().  This helps us keep
 * track that we have noted the irq_disable for this irq_enable.  We want to
 * keep irq_disable()/irq_enable() pairs matched.  If you have any doubt, 
 * read the code at the file and line numbers reported in /proc/imaxoff.  
 * If they are the right irq_disable/irq_enable pairs then there is yet 
 * another hole in the instrumentation where the kernel is turning interrupts 
 * on and off and we've missed it.
 */

void
ilat_irq_disable(char *cli_fname, unsigned cli_lineno)
{
	unsigned long flag;
	struct ilatency_log *l;

	l = &get_cpu_var(ilat_log);
	local_save_flags(flag); 
	__local_irq_disable();
	/* if we are not logging or we have an error, do nothing */
	if (l->logging == OFF) {
		goto out;
	}
	/* 
 	 * If interrupts were already disabled then we don't
 	 * need to log it . . this is a nested interrupt. We
	 * want the outer pair of irq_disable/irq_enable.
	 */
	if (!INTERRUPTS_ENABLED(flag)) {
		l->skip_cli++;
		goto out;
	}
	if (l->sync == 1) {
		l->cli_error++;
	}
	l->sync = 1;
	/* Read the Time Stamp Counter */
	l->cli_file = cli_fname;
	l->cli_line_no = cli_lineno;
	l->cli_ticks = readclock();
out:
	put_cpu_var(ilat_log);
	return;
}

/*
 * We want to make sure we have the irq_enable that corresponds to the 
 * irq_disable caught above.  To do this we make sure the sync count is 1, 
 * as set * by local_irq_disable().  
 * The just_logging parameter is for those situations where we know
 * we are going to be turning interrupts back on in return from
 * exception. We grab the information for logging only but don't 
 * do the __irq_enable we let the original code path do it.  An example is the
 * do_IRQ path for ARM.  We know the exception handler will turn it
 * back on, but it's such a nest of handlers that it's easier to just
 * log it in do_IRQ and leave the exception handlers alone.  We do
 * instrument the exception handlers for PPC and MIPs, they have simple
 * branch-link instructions so we can make the call just before
 * interrupts are turned back on.
 */

#define MAXINT	0x7fffffff

void
ilat_irq_enable(char *fname, unsigned lineno, int just_logging)
{
	unsigned long flag, usecs, old_lowest;
	int i, index;
	struct ilatency_log *l;

	l = &get_cpu_var(ilat_log);
	local_save_flags(flag);
	/* 
	 * logging is turned off in the boot path until we have registered
	 * with /proc.
	 */
	if (l->logging == OFF) {
		goto out;
	}
	/* 
	 * If interrupts are already on we don't have to log this call.
	 */
	if (INTERRUPTS_ENABLED(flag)) {
		l->skip_sti++;
		goto out;
	}
	/*
	 * Sync gets set to 1 by local_irq_disable.  If it's not one then 
	 * we don't have the right irq_disable/irq_enable pair.
	 */
	if (l->sync != 1) {
		l->sti_error++;
		goto out;
	}
	l->sti_ticks = readclock();
	l->sync = 0;
	/*
	 * reading the clock is platform dependent and clock_to_usecs
	 * is setup by each platform as well usually in asm/preempt.h.  
	 * get the time and log all the time for this irq_disable/irq_enable pair.
	 */
	usecs = clock_to_usecs(clock_diff(l->cli_ticks, l->sti_ticks));
	inthoff_logentry(usecs);
	/*
	 * find the lowest hold off time and substitute the lowest with 
	 * the new entry, iff the new entry is lower.
	 * We don't keep multiple entries for the same offending
	 * file and line pair.  Only one entry for printk, for example.
	 * But we want the highest hold off time shown for all
	 * occurances of printk . . . 
	 */
	old_lowest = MAXINT;
	for (index = i = 0; i < NUM_LOG_ENTRIES; i++) {
		if (old_lowest > l->log[i].interrupts_off) {
			old_lowest = l->log[i].interrupts_off;
			index = i;	
		}
		if ((lineno == l->log[i].sti_line) &&
		    (l->cli_line_no == l->log[i].cli_line) &&
		    (fname[0] == l->log[i].sti_file[0]) &&      
		    (l->cli_file[0] == l->log[i].cli_file[0])) {    

			if (usecs > l->log[i].interrupts_off) {
				l->log[i].interrupts_off = usecs;
				l->log[index].sti_ticks = l->sti_ticks;
				l->log[index].cli_ticks = l->cli_ticks;
			}
			l->log[i].multiples++;
			goto out;
		}
	}
	if ((usecs <= old_lowest) && (old_lowest != MAXINT) && (usecs != 0)) {
		goto out;
	}
	l->log[index].interrupts_off = usecs;
	l->log[index].sti_file = fname;
	l->log[index].sti_line = lineno;
	l->log[index].sti_ticks = l->sti_ticks;
	l->log[index].cli_file = l->cli_file;
	l->log[index].cli_line = l->cli_line_no;
	l->log[index].cli_ticks = l->cli_ticks;
	/*
	 * We have a couple of places where we instrument
	 * the code to inform us that interrupt are coming
	 * back on, without actually turning them one,
	 * like in ret_from_exception/intr.  We just log it and
	 * let the original code turn the interrupts back on.
	 */
out:
	put_cpu_var(ilat_log);
	if (!just_logging)
		__local_irq_enable();
}

void
ilat_restore_flags(char *fname, unsigned lineno, unsigned long x)
{
	unsigned long flag;
	struct ilatency_log *l;

	l = &get_cpu_var(ilat_log);
	local_save_flags(flag);
	if (l->logging == OFF) {
		__local_irq_restore(x);
		put_cpu_var(ilat_log);
		return;
	}
	if (!INTERRUPTS_ENABLED(flag) && INTERRUPTS_ENABLED(x)) {
		l->restore_sti++;
		ilat_irq_enable(fname, lineno, 0);
	}
	if (INTERRUPTS_ENABLED(flag) && !INTERRUPTS_ENABLED(x)) {
		l->restore_cli++;
		ilat_irq_disable(fname, lineno);
	}
	__local_irq_restore(x);
	put_cpu_var(ilat_log);
}

static ssize_t
imaxoff_write_proc(struct file * file, char * buf, size_t count,
     loff_t *ppos)
{
	struct ilatency_log *l;

	l = &get_cpu_var(ilat_log);
	l->logging = OFF;
	memset(l, 0, sizeof(struct ilatency_log));
	l->logging = ON;
	put_cpu_var(ilat_log);
	return count;
}   

static int 
imaxoff_read_proc( char *page_buffer, char **my_first_byte,
	off_t offset, int length, int *eof, void *data)
 {
	int len = 0;
	int tmp_len = 0;
	char *const my_base = page_buffer;
	struct ilatency_log *l;
	char tmp_buf[2048];

	l = &get_cpu_var(ilat_log);
	if (offset == 0) {
		l->logging = OFF;
		l->which = 0;
		/* 
		 * Just been opened so display the header information 
		 */
		len += sprintf(my_base + len,
		   "Maximum Interrupt Hold Off Times and Callers\n");
	} else if (l->which == -1) {
		*eof = 1;
		l->logging = ON;
		put_cpu_var(ilat_log);
		return 0;
	}
	for (; l->which < NUM_LOG_ENTRIES; l->which++) {
		tmp_buf[0] = '\0';
		tmp_len += sprintf(tmp_buf,
		    "%d) %lu us interrupts off\n\tirq_disable file %s"
		    " (%d times)\n"
		    "\tirq_disable line %5u\t\n\tirq_enable file %s\n"
		    "\tirq_enable line  %5u\n"
		    "\tdisable timer count %10lu\n\tenable timer count %10lu\n", 
		    l->which, l->log[l->which].interrupts_off,
		    l->log[l->which].cli_file, l->log[l->which].multiples,
		    l->log[l->which].cli_line, l->log[l->which].sti_file, 
		    l->log[l->which].sti_line, l->log[l->which].cli_ticks,
		    l->log[l->which].sti_ticks);
		if (tmp_len >= length) {
			break;
		}	
		len += sprintf(my_base + len,
		    "%d) %lu us interrupts off\n\tirq_disable file %s"
		    " (%d times)\n"
		    "\tirq_disable line %5u\t\n\tirq_enable file %s\n"
		    "\tirq_enable line %5u\n"
		    "\tdisable timer count %10lu\n\tenable timer count  %10lu\n", 
		    l->which, l->log[l->which].interrupts_off,
		    l->log[l->which].cli_file, l->log[l->which].multiples,
		    l->log[l->which].cli_line, l->log[l->which].sti_file, 
		    l->log[l->which].sti_line, l->log[l->which].cli_ticks,
		    l->log[l->which].sti_ticks);
	}
	if (l->which >= NUM_LOG_ENTRIES) {
		l->which = -1;
	}
	*my_first_byte = page_buffer;
	put_cpu_var(ilat_log);
	return  len;
}

int  __init
holdoffs_init(void)
{
        struct proc_dir_entry *entry;
	struct ilatency_log *l;

	l = &get_cpu_var(ilat_log);
	if (ticks_per_usec) {
		printk("Interrupt holdoff maximums being captured.\n");
		l->logging = ON;
	} else {
		printk("Interrupt Latency timer is NOT on! No data will be collected\n");
		put_cpu_var(ilat_log);
		return -1;
	}
#ifdef CONFIG_PROC_FS
	entry = create_proc_read_entry("imaxoff", 0, 0, imaxoff_read_proc, 0);
	if (entry)
		entry->write_proc = (write_proc_t *)imaxoff_write_proc;
#endif
	put_cpu_var(ilat_log);
	return 0;
}

__initcall(holdoffs_init);

EXPORT_SYMBOL(ilat_irq_disable);
EXPORT_SYMBOL(ilat_irq_enable);
EXPORT_SYMBOL(ilat_restore_flags);
