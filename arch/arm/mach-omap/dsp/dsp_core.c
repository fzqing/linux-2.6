/*
 * linux/arch/arm/mach-omap/dsp/dsp_core.c
 *
 * OMAP DSP driver
 *
 * Copyright (C) 2002-2004 Nokia Corporation
 *
 * Written by Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * 2004/11/22:  DSP Gateway version 3.1
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/devfs_fs_kernel.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/signal.h>
#include <asm/delay.h>
#include <asm/irq.h>
#include <asm/arch/dsp.h>
#include "hardware_dsp.h"
#include "dsp.h"
#include "ipbuf.h"
#include "proclist.h"


MODULE_AUTHOR("Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>");
MODULE_DESCRIPTION("OMAP DSP driver module");
MODULE_LICENSE("GPL");

enum mbseq_check_level {
	MBSEQ_CHECK_NONE,	/* no check */
	MBSEQ_CHECK_VERBOSE,	/* discard the illegal command and
				   error report */
	MBSEQ_CHECK_SILENT,	/* discard the illegal command */
};

static enum mbseq_check_level mbseq_check_level = MBSEQ_CHECK_VERBOSE;
static unsigned short mbseq_send;
static unsigned short mbseq_expect;

struct sync_seq {
	unsigned short da_dsp;
	unsigned short da_arm;
	unsigned short ad_dsp;
	unsigned short ad_arm;
};

static struct sync_seq *sync_seq;

/*
 * DSP module user process list
 */
static LIST_HEAD(cur_users_list);

void dsp_cur_users_add(struct task_struct *tsk)
{
	preempt_disable();
	proc_list_add(&cur_users_list, tsk);
	preempt_enable();
}

void dsp_cur_users_del(struct task_struct *tsk)
{
	preempt_disable();
	proc_list_del(&cur_users_list, tsk);
	preempt_enable();
}

void dsp_cur_users_map_update(void)
{
	struct list_head *ptr;
	struct proc_list *pl;
	struct task_struct *tsk;

	preempt_disable();
	list_for_each(ptr, &cur_users_list) {
		pl = list_entry(ptr, struct proc_list, list_head);
		tsk = pl->tsk;

		/*
		 * excluding ZOMBIE, STOPPED processes
		 */
		if ((tsk->state == TASK_RUNNING) ||
		    (tsk->state == TASK_INTERRUPTIBLE) ||
		    (tsk->state == TASK_UNINTERRUPTIBLE)) {
			dsp_map_update(tsk);
		}
	}
	preempt_enable();
}

#ifdef CONFIG_PROC_FS
static int cur_users_read_proc(char *page, char **start, off_t off, int count,
			       int *eof, void *data)
{
	char *out;
	int len;
	struct list_head *ptr;
	struct proc_list *pl;

	out = page;
	preempt_disable();
	list_for_each(ptr, &cur_users_list) {
		pl = list_entry(ptr, struct proc_list, list_head);
		out += sprintf(out, "%d\n", pl->tsk->pid);
	}
	preempt_enable();

	len = out - (page + off);
	if (len < count) {
		*eof = 1;
		if (len <= 0)
			return 0;
	} else
		len = count;
	*start = page + off;

	return len;
}

static void dsp_create_cur_users_proc(void)
{
	struct proc_dir_entry *ent;

	ent = create_proc_read_entry("cur_users", 0, procdir_dsp,
				     cur_users_read_proc, NULL);
	if (ent == NULL) {
		printk(KERN_ERR
		       "omapdsp: failed to register proc device: cur_users\n");
	}
}

static void dsp_remove_cur_users_proc(void)
{
	remove_proc_entry("cur_users", procdir_dsp);
}
#endif /* CONFIG_PROC_FS */

static void __init dsp_cur_users_init(void)
{
#ifdef CONFIG_PROC_FS
	dsp_create_cur_users_proc();
#endif
}

static void dsp_cur_users_exit(void)
{
	preempt_disable();
	proc_list_flush(&cur_users_list);
	preempt_enable();
#ifdef CONFIG_PROC_FS
	dsp_remove_cur_users_proc();
#endif
}

/*
 * mailbox commands
 */
extern void mbx1_wdsnd(struct mbcmd *mb);
extern void mbx1_wdreq(struct mbcmd *mb);
extern void mbx1_bksnd(struct mbcmd *mb);
extern void mbx1_bkreq(struct mbcmd *mb);
extern void mbx1_bkyld(struct mbcmd *mb);
extern void mbx1_bksndp(struct mbcmd *mb);
extern void mbx1_bkreqp(struct mbcmd *mb);
extern void mbx1_tctl(struct mbcmd *mb);
extern void mbx1_wdt(struct mbcmd *mb);
extern void mbx1_suspend(struct mbcmd *mb);
extern void mbx1_tcfg(struct mbcmd *mb);
extern void mbx1_tadd(struct mbcmd *mb);
extern void mbx1_tdel(struct mbcmd *mb);
extern void mbx1_dspcfg(struct mbcmd *mb);
extern void mbx1_regrw(struct mbcmd *mb);
extern void mbx1_getvar(struct mbcmd *mb);
extern void mbx1_err(struct mbcmd *mb);
extern void mbx1_dbg(struct mbcmd *mb);

static const struct cmdinfo
	cif_null     = { "Unknown",  CMD_L_TYPE_NULL,   NULL         },
	cif_wdsnd    = { "WDSND",    CMD_L_TYPE_TID,    mbx1_wdsnd   },
	cif_wdreq    = { "WDREQ",    CMD_L_TYPE_TID,    mbx1_wdreq   },
	cif_bksnd    = { "BKSND",    CMD_L_TYPE_TID,    mbx1_bksnd   },
	cif_bkreq    = { "BKREQ",    CMD_L_TYPE_TID,    mbx1_bkreq   },
	cif_bkyld    = { "BKYLD",    CMD_L_TYPE_NULL,   mbx1_bkyld   },
	cif_bksndp   = { "BKSNDP",   CMD_L_TYPE_TID,    mbx1_bksndp  },
	cif_bkreqp   = { "BKREQP",   CMD_L_TYPE_TID,    mbx1_bkreqp  },
	cif_tctl     = { "TCTL",     CMD_L_TYPE_TID,    mbx1_tctl    },
	cif_wdt      = { "WDT",      CMD_L_TYPE_NULL,   mbx1_wdt     },
	cif_runlevel = { "RUNLEVEL", CMD_L_TYPE_SUBCMD, NULL         },
	cif_pm       = { "PM",       CMD_L_TYPE_SUBCMD, NULL         },
	cif_suspend  = { "SUSPEND",  CMD_L_TYPE_NULL,   mbx1_suspend },
	cif_tcfg     = { "TCFG",     CMD_L_TYPE_TID,    mbx1_tcfg    },
	cif_tadd     = { "TADD",     CMD_L_TYPE_TID,    mbx1_tadd    },
	cif_tdel     = { "TDEL",     CMD_L_TYPE_TID,    mbx1_tdel    },
	cif_tstop    = { "TSTOP",    CMD_L_TYPE_TID,    NULL         },
	cif_dspcfg   = { "DSPCFG",   CMD_L_TYPE_SUBCMD, mbx1_dspcfg  },
	cif_regrw    = { "REGRW",    CMD_L_TYPE_SUBCMD, mbx1_regrw   },
	cif_getvar   = { "GETVAR",   CMD_L_TYPE_SUBCMD, mbx1_getvar  },
	cif_setvar   = { "SETVAR",   CMD_L_TYPE_SUBCMD, NULL         },
	cif_err      = { "ERR",      CMD_L_TYPE_SUBCMD, mbx1_err     },
	cif_dbg      = { "DBG",      CMD_L_TYPE_NULL,   mbx1_dbg     };

const struct cmdinfo *cmdinfo[128] = {
/*00*/	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
/*10*/	&cif_wdsnd, &cif_wdreq, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
/*20*/	&cif_bksnd, &cif_bkreq, &cif_null, &cif_bkyld,
	&cif_bksndp, &cif_bkreqp, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
/*30*/	&cif_tctl, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
/*40*/	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
/*50*/	&cif_wdt, &cif_runlevel, &cif_pm, &cif_suspend,
	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
/*60*/	&cif_tcfg, &cif_null, &cif_tadd, &cif_tdel,
	&cif_null, &cif_tstop, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
/*70*/	&cif_dspcfg, &cif_null, &cif_regrw, &cif_null,
	&cif_getvar, &cif_setvar, &cif_null, &cif_null,
	&cif_err, &cif_dbg, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null
};

int sync_with_dsp(unsigned short *syncwd, unsigned short tid, int try_cnt)
{
	int try;

	if (*(volatile unsigned short *)syncwd == tid)
		return 0;

	for (try = 0; try < try_cnt; try++) {
		udelay(1);
		if (*(volatile unsigned short *)syncwd == tid) {
			/* success! */
			printk(KERN_INFO
			       "omapdsp: sync_with_dsp(): try = %d\n", try);
			return 0;
		}
	}

	/* fail! */
	return -1;
}

static __inline__ int mbsync_irq_save(unsigned long *flags, int try_cnt)
{
	int cnt;

	local_irq_save(*flags);
	if (omap_readw(MAILBOX_ARM2DSP1_Flag) == 0)
		return 0;
	/*
	 * mailbox is busy. wait for some usecs...
	 */
	local_irq_restore(*flags);
	for (cnt = 0; cnt < try_cnt; cnt++) {
		udelay(1);
		local_irq_save(*flags);
		if (omap_readw(MAILBOX_ARM2DSP1_Flag) == 0)	/* success! */
			return 0;
		local_irq_restore(*flags);
	}

	/* fail! */
	return -1;
}

#ifdef CONFIG_OMAP_DSP_MBCMD_VERBOSE
#define print_mb_busy_abort(mb) \
	printk(KERN_DEBUG \
	       "mbx: mailbox is busy. %s is aborting.\n", cmd_name(*mb))
#define print_mb_lock_abort(mb) \
	printk(KERN_DEBUG \
	       "mbx: mailbox is locked. %s is aborting.\n", cmd_name(*mb))
#else /* CONFIG_OMAP_DSP_MBCMD_VERBOSE */
#define print_mb_busy_abort(mb)	do {} while(0)
#define print_mb_lock_abort(mb)	do {} while(0)
#endif /* !CONFIG_OMAP_DSP_MBCMD_VERBOSE */

static int __mbsend(struct mbcmd *mb)
{
	struct mbcmd_hw *mb_hw = (struct mbcmd_hw *)mb;
	unsigned long flags;

	/*
	 * DSP mailbox interrupt latency must be less than 1ms.
	 *
	 * FIXME: use task queue!
	 */
	if (mbsync_irq_save(&flags, 1000) < 0) {
		print_mb_busy_abort(mb);
		return -1;
	}

	mb->seq = mbseq_send & 1;
	mbseq_send++;
	if (sync_seq)
		sync_seq->ad_arm = mbseq_send;
	mblog_add(mb, MBLOG_DIR_AD);
	mblog_printcmd(mb, MBLOG_DIR_AD);

	omap_writew(mb_hw->data, MAILBOX_ARM2DSP1);
	omap_writew(mb_hw->cmd, MAILBOX_ARM2DSP1b);

	local_irq_restore(flags);
	return 0;
}

/*
 * __dsp_mbsend(): mailbox dispatcher
 * interrupt routine should call this function with
 * send_type = MBSEND_TYPE_NOWAIT
 */
int __dsp_mbsend(struct mbcmd *mb, struct mb_exarg *arg,
		 enum mbsend_type send_type)
{
	static DECLARE_MUTEX(mbsend_sem);
	int ret = 0;

	/*
	 * while MMU fault is set,
	 * only recovery command can be executed
	 */
	if (dsp_err_mmu_isset() && (send_type != MBSEND_TYPE_RECOVERY)) {
		print_mb_busy_abort(mb);
		return -1;
	}

	/*
	 * fast lane for interrupt context:
	 * no need to get semaphore
	 */
	if (send_type == MBSEND_TYPE_NOWAIT)
		return __mbsend(mb);

	/*
	 * process context
	 */
	if (down_interruptible(&mbsend_sem) < 0)
		return -1;

	if (arg) {	/* we have extra argument */
		struct mbcmd mb_pm;
		int dspmem_access;
		int i;

		if ((dspmem_access = is_dsp_internal_mem(ipbuf_sys_ad)) != 0) {
			mbcmd_set(mb_pm, MBCMD(PM), OMAP_DSP_MBCMD_PM_ENABLE,
				  DSPREG_ICR_DMA_IDLE_DOMAIN);
			if (__mbsend(&mb_pm) < 0)
				goto out;
		}
		if (sync_with_dsp(&ipbuf_sys_ad->s, OMAP_DSP_TID_FREE, 10) < 0) {
			printk(KERN_ERR "omapdsp: ipbuf_sys_ad is busy.\n");
			ret = -EBUSY;
			goto out;
		}
		for (i = 0; i < arg->argc; i++) {
			ipbuf_sys_ad->d[i] = arg->argv[i];
		}
		ipbuf_sys_ad->s = arg->tid;
		if (dspmem_access)
			mbcmd_set(mb_pm, MBCMD(PM), OMAP_DSP_MBCMD_PM_DISABLE,
				  DSPREG_ICR_DMA_IDLE_DOMAIN);
			if (__mbsend(&mb_pm) < 0)
				goto out;
	}

	ret = __mbsend(mb);

out:
	up(&mbsend_sem);
	return ret;
}

int __dsp_mbsend_and_wait(struct mbcmd *mb, struct mb_exarg *arg,
			  wait_queue_head_t *q)
{
	long current_state;
	DECLARE_WAITQUEUE(wait, current);

	add_wait_queue(q, &wait);
	current_state = current->state;
	set_current_state(TASK_INTERRUPTIBLE);
	if (dsp_mbsend_exarg(mb, arg) < 0) {
		set_current_state(current_state);
		remove_wait_queue(q, &wait);
		return -1;
	}
	schedule();
	set_current_state(current_state);
	remove_wait_queue(q, &wait);

	return 0;
}

void dsp_mb_start(void)
{
	mbseq_send = 0;
	mbseq_expect = 0;
}

void dsp_mb_stop(void)
{
	sync_seq = NULL;
}

/*
 * dsp_mb_config() is called in interrupt context.
 */
void dsp_mb_config(void *sync_seq_adr)
{
	sync_seq = sync_seq_adr;
	sync_seq->da_arm = mbseq_expect;
}

/*
 * mailbox interrupt handler
 */
static irqreturn_t mbx1_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	union {
		struct mbcmd sw;
		struct mbcmd_hw hw;
	} mb;

#if (INT_D2A_MB1 == INT_DSP_MAILBOX1)
	mb.hw.data = omap_readw(MAILBOX_DSP2ARM1);
	mb.hw.cmd  = omap_readw(MAILBOX_DSP2ARM1b);
#elif (INT_D2A_MB1 == INT_DSP_MAILBOX2)
	mb.hw.data = omap_readw(MAILBOX_DSP2ARM2);
	mb.hw.cmd  = omap_readw(MAILBOX_DSP2ARM2b);
#endif

	if (mb.sw.seq != (mbseq_expect & 1)) {
		switch (mbseq_check_level) {
		case MBSEQ_CHECK_NONE:
			break;
		case MBSEQ_CHECK_VERBOSE:
			printk(KERN_INFO
			       "mbx: illegal seq bit!!!  ignoring this command."
			       " (%04x:%04x)\n", mb.hw.cmd, mb.hw.data);
			return IRQ_HANDLED;
		case MBSEQ_CHECK_SILENT:
			return IRQ_HANDLED;
		}
	}

	mblog_add(&mb.sw, MBLOG_DIR_DA);
	mblog_printcmd(&mb.sw, MBLOG_DIR_DA);

	/*
	 * call handler for each command
	 */
	if (cmdinfo[mb.sw.cmd_h]->handler)
		cmdinfo[mb.sw.cmd_h]->handler(&mb.sw);
	else if (cmdinfo[mb.sw.cmd_h] != &cif_null)
		printk(KERN_ERR "mbx: %s is not allowed from DSP.\n",
		       cmd_name(mb.sw));
	else
		printk(KERN_ERR
		       "mbx: Unrecognized command: cmd=0x%04x, data=0x%04x\n",
		       mb.hw.cmd & 0x7fff, mb.hw.data);

	mbseq_expect++;
	if (sync_seq)
		sync_seq->da_arm = mbseq_expect;
	return IRQ_HANDLED;
}

static irqreturn_t mbx2_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	unsigned short cmd, data;

#if (INT_D2A_MB1 == INT_DSP_MAILBOX1)
	data = omap_readw(MAILBOX_DSP2ARM2);
	cmd  = omap_readw(MAILBOX_DSP2ARM2b);
#elif (INT_D2A_MB1 == INT_DSP_MAILBOX2)
	data = omap_readw(MAILBOX_DSP2ARM1);
	cmd  = omap_readw(MAILBOX_DSP2ARM1b);
#endif
	printk(KERN_DEBUG
	       "mailbox2 interrupt!  cmd=%04x, data=%04x\n", cmd, data);

	return IRQ_HANDLED;
}

#if 0
static void mpuio_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	printk(KERN_INFO "MPUIO interrupt!\n");
}
#endif

#ifdef CONFIG_PROC_FS
struct proc_dir_entry *procdir_dsp = NULL;

static void dsp_create_procdir_dsp(void)
{
	procdir_dsp = proc_mkdir("dsp", 0);
	if (procdir_dsp == NULL) {
		printk(KERN_ERR
		       "omapdsp: failed to register proc directory: dsp\n");
	}
}

static void dsp_remove_procdir_dsp(void)
{
	procdir_dsp = NULL;
	remove_proc_entry("dsp", 0);
}
#endif /* CONFIG_PROC_FS */

extern irqreturn_t dsp_mmu_interrupt(int irq, void *dev_id,
				     struct pt_regs *regs);

extern int  dsp_ctl_core_init(void);
extern void dsp_ctl_core_exit(void);
extern void dsp_ctl_init(void);
extern void dsp_ctl_exit(void);
extern int  dsp_mem_init(void);
extern void dsp_mem_exit(void);
extern void mblog_init(void);
extern void mblog_exit(void);
extern int  dsp_taskmod_init(void);
extern void dsp_taskmod_exit(void);

/*
 * device functions
 */
static void dsp_dev_release(struct device *dev)
{
}

/*
 * driver functions
 */
#if (INT_D2A_MB1 == INT_DSP_MAILBOX1)
#	define INT_D2A_MB2 INT_DSP_MAILBOX2
#elif(INT_D2A_MB1 == INT_DSP_MAILBOX2)	/* swap MB1 and MB2 */
#	define INT_D2A_MB2 INT_DSP_MAILBOX1
#endif

static int __init dsp_drv_probe(struct device *dev)
{
	int ret;

	printk(KERN_INFO "OMAP DSP driver initialization\n");

	//__dsp_enable(); // XXX

#ifdef CONFIG_PROC_FS
	dsp_create_procdir_dsp();
#endif /* CONFIG_PROC_FS */

	if ((ret = dsp_ctl_core_init()) < 0)
		goto fail1;
	if ((ret = dsp_mem_init()) < 0)
		goto fail2;
	dsp_cur_users_init();
	dsp_ctl_init();
	mblog_init();
	if ((ret = dsp_taskmod_init()) < 0)
		goto fail3;

	/*
	 * mailbox interrupt handlers registration
	 */
	ret = request_irq(INT_D2A_MB1, mbx1_interrupt, SA_INTERRUPT, "dsp",
			  dev);
	if (ret) {
		printk(KERN_ERR
		       "failed to register mailbox1 interrupt: %d\n", ret);
		goto fail4;
	}

	ret = request_irq(INT_D2A_MB2, mbx2_interrupt, SA_INTERRUPT, "dsp",
			  dev);
	if (ret) {
		printk(KERN_ERR
		       "failed to register mailbox2 interrupt: %d\n", ret);
		goto fail5;
	}

	ret = request_irq(INT_DSP_MMU, dsp_mmu_interrupt, SA_INTERRUPT, "dsp",
			  dev);
	if (ret) {
		printk(KERN_ERR
		       "failed to register DSP MMU interrupt: %d\n", ret);
		goto fail6;
	}

#if 0
	ret = request_irq(INT_MPUIO, mpuio_interrupt, SA_INTERRUPT, "dsp", dev);
	if (ret) {
		printk(KERN_ERR
		       "failed to register MPUIO interrupt: %d\n", ret);
		goto fail7;
	}
#endif

	return 0;

fail6:
	free_irq(INT_D2A_MB2, dev);
fail5:
	free_irq(INT_D2A_MB1, dev);
fail4:
	dsp_taskmod_exit();
fail3:
	mblog_exit();
	dsp_ctl_exit();
	dsp_cur_users_exit();
	dsp_mem_exit();
fail2:
	dsp_ctl_core_exit();
fail1:
#ifdef CONFIG_PROC_FS
	dsp_remove_procdir_dsp();
#endif /* CONFIG_PROC_FS */

	//__dsp_disable(); // XXX
	return ret;
}

static int dsp_drv_remove(struct device *dev)
{
	__dsp_reset();

#if 0
	free_irq(INT_MPUIO, dev);
#endif
	free_irq(INT_DSP_MMU, dev);
	free_irq(INT_D2A_MB2, dev);
	free_irq(INT_D2A_MB1, dev);

	dspuncfg();
	dsp_taskmod_exit();
	mblog_exit();
	dsp_ctl_exit();
	dsp_cur_users_exit();
	dsp_mem_exit();

	dsp_ctl_core_exit();
#ifdef CONFIG_PROC_FS
	dsp_remove_procdir_dsp();
#endif /* CONFIG_PROC_FS */

	//__dsp_disable(); // XXX

	return 0;
}

#ifdef CONFIG_PM
static int dsp_drv_suspend(struct device *dev, u32 state, u32 level)
{
	switch(level) {
	case SUSPEND_NOTIFY:
	case SUSPEND_DISABLE:
	case SUSPEND_SAVE_STATE:
		break;
	case SUSPEND_POWER_DOWN:
		dsp_suspend();
		break;
	}

	return 0;
}

static int dsp_drv_resume(struct device *dev, u32 level)
{
	switch(level) {
	case RESUME_POWER_ON:
		dsp_resume();
		break;
	case RESUME_RESTORE_STATE:
	case RESUME_ENABLE:
		break;
	}

	return 0;
}
#endif /* CONFIG_PM */

static struct resource dsp_resources[] = {
	{
		.start = INT_DSP_MAILBOX1,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = INT_DSP_MAILBOX2,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = INT_DSP_MMU,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device dsp_device = {
	.name		= "dsp",
	.id		= 0,
	.dev = {
		.release	= dsp_dev_release,
	},
	.num_resources	= ARRAY_SIZE(&dsp_resources),
	.resource	= dsp_resources,
};

static struct device_driver dsp_driver = {
	.name		= "dsp",
	.bus		= &platform_bus_type,
	.probe		= dsp_drv_probe,
	.remove		= dsp_drv_remove,
#ifdef CONFIG_PM
	.suspend	= dsp_drv_suspend,
	.resume		= dsp_drv_resume,
#endif
};

static int __init omap_dsp_mod_init(void)
{
	int ret;

	ret = platform_device_register(&dsp_device);
	if (ret) {
		printk(KERN_ERR "failed to register the DSP device: %d\n", ret);
		goto fail1;
	}

	ret = driver_register(&dsp_driver);
	if (ret) {
		printk(KERN_ERR "failed to register the DSP driver: %d\n", ret);
		goto fail2;
	}

	return 0;

fail2:
	platform_device_unregister(&dsp_device);
fail1:
	return -ENODEV;
}

static void __exit omap_dsp_mod_exit(void)
{
	driver_unregister(&dsp_driver);
	platform_device_unregister(&dsp_device);
}

module_init(omap_dsp_mod_init);
module_exit(omap_dsp_mod_exit);
