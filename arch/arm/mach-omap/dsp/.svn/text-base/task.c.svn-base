/*
 * linux/arch/arm/mach-omap/dsp/task.c
 *
 * OMAP DSP task device driver
 *
 * Copyright (C) 2002-2004 Nokia Corporation
 *
 * Written by Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
 * mmap function by Hiroo Ishikawa <ext-hiroo.ishikawa@nokia.com>
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/major.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/devfs_fs_kernel.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/signal.h>
#include <asm/irq.h>
#include <asm/ioctls.h>
#include <asm/arch/dsp.h>
#include "uaccess_dsp.h"
#include "dsp.h"
#include "ipbuf.h"
#include "fifo.h"
#include "proclist.h"

/*
 * device state machine
 * NOTASK:	task is not attached.
 * ATTACHED:	task is attached.
 * GARBAGE:	task is detached. waiting for all processes to close this device.
 * ADDREQ:	requesting for tadd
 * DELREQ:	requesting for tdel. no process is opening this device.
 * KILLREQ:	requesting for tkill.
 * ADDFAIL:	tadd failed.
 */

struct taskdev {
	long state;
	spinlock_t state_lock;
	wait_queue_head_t state_wait_q;
	unsigned int usecount;
	char name[OMAP_DSP_TNM_LEN];
	struct file_operations fops;
	struct list_head proc_list;
	struct dsptask *task;
	struct proc_dir_entry *procent_dir;

	/* read stuff */
	wait_queue_head_t read_wait_q;
	struct semaphore read_sem;

	/* write stuff */
	wait_queue_head_t write_wait_q;
	struct semaphore write_sem;

	/* ioctl stuff */
	wait_queue_head_t ioctl_wait_q;
	struct semaphore ioctl_sem;

	/* device lock */
	struct semaphore lock_sem;
	pid_t lock_pid;
};

struct rcvdt_bk_struct {
	struct ipblink link;
	unsigned int rp;
	struct ipbuf_p *ipbuf_pvt_r;
};

struct dsptask {
	enum {
		TASK_STATE_ERR = 0,
		TASK_STATE_READY,
		TASK_STATE_CFGREQ
	} state;
	unsigned char tid;
	char name[OMAP_DSP_TNM_LEN];
	unsigned short ttyp;
	struct taskdev *dev;

	/* read stuff */
	union {
		struct fifo_struct fifo;	/* for active word */
		struct rcvdt_bk_struct bk;
	} rcvdt;

	/* write stuff */
	size_t wsz;
	struct ipbuf_p *ipbuf_pvt_w;	/* for private block */

	/* tctl stuff */
	int tctl_stat;

	/* mmap stuff */
	void *map_base;
	size_t map_length;
};

#define sndtyp_acv(ttyp)	((ttyp) & OMAP_DSP_TTYP_ASND)
#define sndtyp_psv(ttyp)	(!((ttyp) & OMAP_DSP_TTYP_ASND))
#define sndtyp_bk(ttyp)		((ttyp) & OMAP_DSP_TTYP_BKDM)
#define sndtyp_wd(ttyp)		(!((ttyp) & OMAP_DSP_TTYP_BKDM))
#define sndtyp_pvt(ttyp)	((ttyp) & OMAP_DSP_TTYP_PVDM)
#define sndtyp_gbl(ttyp)	(!((ttyp) & OMAP_DSP_TTYP_PVDM))
#define rcvtyp_acv(ttyp)	((ttyp) & OMAP_DSP_TTYP_ARCV)
#define rcvtyp_psv(ttyp)	(!((ttyp) & OMAP_DSP_TTYP_ARCV))
#define rcvtyp_bk(ttyp)		((ttyp) & OMAP_DSP_TTYP_BKMD)
#define rcvtyp_wd(ttyp)		(!((ttyp) & OMAP_DSP_TTYP_BKMD))
#define rcvtyp_pvt(ttyp)	((ttyp) & OMAP_DSP_TTYP_PVMD)
#define rcvtyp_gbl(ttyp)	(!((ttyp) & OMAP_DSP_TTYP_PVMD))

static int dsp_rmdev_minor(unsigned char minor);
static int taskdev_init(struct taskdev *dev, char *name, unsigned char minor);
static void taskdev_delete(unsigned char minor);
static void taskdev_attach_task(struct taskdev *dev, struct dsptask *task);
static void taskdev_detach_task(struct taskdev *dev);

#ifdef CONFIG_PROC_FS
static void taskdev_create_proc(struct taskdev *dev);
static void taskdev_remove_proc(struct taskdev *dev);
#endif

static struct taskdev *taskdev[TASKDEV_MAX];
static struct dsptask *dsptask[TASKDEV_MAX];
static DECLARE_MUTEX(cfg_sem);
static unsigned short cfg_cmd;
static unsigned char cfg_tid;
static DECLARE_WAIT_QUEUE_HEAD(cfg_wait_q);
static unsigned char n_task;
static void *heap;
#ifdef CONFIG_PROC_FS
struct proc_dir_entry *procdir_dsp_task;
#endif

#define devstate_lock(dev, devstate)	devstate_lock_timeout(dev, devstate, 0)

/*
 * devstate_lock_timeout():
 * when called with timeout > 0, dev->state can be diffeent from what you want.
 */
static int devstate_lock_timeout(struct taskdev *dev, long devstate,
				 int timeout)
{
	DECLARE_WAITQUEUE(wait, current);
	long current_state = current->state;
	int ret = 0;

	spin_lock(&dev->state_lock);
	add_wait_queue(&dev->state_wait_q, &wait);
	while (!(dev->state & devstate)) {
		set_current_state(TASK_INTERRUPTIBLE);
		spin_unlock(&dev->state_lock);
		if (timeout) {
			if ((timeout = schedule_timeout(timeout)) == 0) {
				/* timeout */
				spin_lock(&dev->state_lock);
				break;
			}
		}
		else
			schedule();
		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}
		spin_lock(&dev->state_lock);
	}
	remove_wait_queue(&dev->state_wait_q, &wait);
	set_current_state(current_state);
	return ret;
}

static __inline__ void devstate_unlock(struct taskdev *dev)
{
	spin_unlock(&dev->state_lock);
}

static __inline__ int down_tasksem_interruptible(struct taskdev *dev,
						 struct semaphore *sem)
{
	int ret;

	if (dev->lock_pid == current->pid) {
		/* this process has lock */
		ret = down_interruptible(sem);
	} else {
		if ((ret = down_interruptible(&dev->lock_sem)) != 0)
			return ret;
		ret = down_interruptible(sem);
		up(&dev->lock_sem);
	}
	return ret;
}

static int dsp_task_flush_buf(struct dsptask *task)
{
	unsigned short ttyp = task->ttyp;

	preempt_disable();
	if (sndtyp_wd(ttyp)) {
		/* word receiving */
		flush_fifo(&task->rcvdt.fifo);
	} else {
		/* block receiving */
		struct rcvdt_bk_struct *rcvdt = &task->rcvdt.bk;

		if (sndtyp_gbl(ttyp)) {
			/* global IPBUF */
			while (!ipblink_empty(&rcvdt->link)) {
				unsigned short bid = rcvdt->link.top;
				disable_irq(INT_D2A_MB1);
				ipblink_del_top(&rcvdt->link, ipbuf);
				enable_irq(INT_D2A_MB1);
				unuse_ipbuf(bid);
			}
		} else {
			/* private IPBUF */
			if (!ipblink_empty(&rcvdt->link)) {
				ipblink_del_pvt(&rcvdt->link);
				release_ipbuf_pvt(rcvdt->ipbuf_pvt_r);
			}
		}
	}
	preempt_enable();

	return 0;
}

static int dsp_task_set_fifosz(struct dsptask *task, unsigned long sz)
{
	unsigned short ttyp = task->ttyp;
	struct fifo_struct *fifo = &task->rcvdt.fifo;
	int stat;

	if (!(sndtyp_wd(ttyp) && sndtyp_acv(ttyp))) {
		printk(KERN_ERR
		       "omapdsp: buffer size can be changed only for "
		       "active word sending task.\n");
		return -EINVAL;
	}
	if (sz == 0) {
		printk(KERN_ERR "omapdsp: buffer size shouldn't be zero!\n");
		return -EINVAL;
	}
	preempt_disable();
	if (!fifo_empty(fifo)) {
		printk(KERN_ERR "omapdsp: buffer is not empty!\n");
		preempt_enable();
		return -EINVAL;
	}
	if (sz & 0x1) {
		/* force even value */
		sz++;
	}

	free_fifo(fifo);
	stat = alloc_fifo(fifo, sz);
	preempt_enable();
	if (stat < 0) {
		printk(KERN_ERR
		       "omapdsp: unable to change receive buffer size. "
		       "(%ld bytes for %s)\n", sz, task->name);
		return -ENOMEM;
	}

	return 0;
}

static int taskdev_lock(struct taskdev *dev)
{
	if (down_interruptible(&dev->lock_sem))
		return -ERESTARTSYS;
	dev->lock_pid = current->pid;
	return 0;
}

static int taskdev_unlock(struct taskdev *dev)
{
	if (dev->lock_pid != current->pid) {
		printk(KERN_ERR
		       "omapdsp: an illegal process attempted to "
		       "unlock the dsptask lock!\n");
		return -EINVAL;
	}
	dev->lock_pid = 0;
	up(&dev->lock_sem);
	return 0;
}

static int dsp_task_config(struct dsptask *task, unsigned char tid)
{
	unsigned short ttyp;
	struct mbcmd mb;

	dsptask[tid] = task;
	task->tid = tid;

	/* TCFG request */
	task->state = TASK_STATE_CFGREQ;
	if (down_interruptible(&cfg_sem))
		return -ERESTARTSYS;
	cfg_cmd = MBCMD(TCFG);
	mbcmd_set(mb, MBCMD(TCFG), tid, 0);
	dsp_mbsend_and_wait(&mb, &cfg_wait_q);
	cfg_cmd = 0;
	up(&cfg_sem);

	if (task->state != TASK_STATE_READY) {
		printk(KERN_ERR "omapdsp: task %d configuration error!\n", tid);
		return -EINVAL;
	}

	if (strlen(task->name) <= 1)
		sprintf(task->name, "%d", tid);
	printk(KERN_INFO "omapdsp: task %d: name %s\n", tid, task->name);

	ttyp = task->ttyp;

	/* task type check */
	if (rcvtyp_psv(ttyp) && rcvtyp_pvt(ttyp)) {
		printk(KERN_ERR "mbx: illegal task type(0x%04x), tid=%d\n",
		       tid, ttyp);
	}

	/* private buffer address check */
	if (sndtyp_pvt(ttyp)) {
		void *p = task->rcvdt.bk.ipbuf_pvt_r;

		if ((unsigned long)p & 0x1) {
			printk(KERN_ERR
			       "mbx: private ipbuf (DSP->ARM) address (0x%p) "
			       "is odd number!\n", p);
			return -EINVAL;
		}
	}

	if (rcvtyp_pvt(ttyp)) {
		void *p = task->ipbuf_pvt_w;

		if ((unsigned long)p & 0x1) {
			printk(KERN_ERR
			       "mbx: private ipbuf (ARM->DSP) address (0x%p) "
			       "is odd number!\n", p);
			return -EINVAL;
		}
	}

	/* read initialization */
	if (sndtyp_wd(ttyp)) {
		/* word */
		size_t fifosz;

		fifosz = sndtyp_psv(ttyp) ? 2 :	/* passive */
					    32;	/* active */
		if (alloc_fifo(&task->rcvdt.fifo, fifosz) < 0) {
			printk(KERN_ERR
			       "omapdsp: unable to allocate receive buffer. "
			       "(%d bytes for %s)\n", fifosz, task->name);
			return -ENOMEM;
		}
	} else {
		/* block */
		INIT_IPBLINK(&task->rcvdt.bk.link);
		task->rcvdt.bk.rp = 0;
	}

	/* write initialization */
	task->wsz = rcvtyp_acv(ttyp) ? 0 :		/* active */
		    rcvtyp_wd(ttyp)  ? 2 :		/* passive word */
		    		       ipbcfg.lsz*2;	/* passive block */

	return 0;
}

static void dsp_task_init(struct dsptask *task)
{
	struct mbcmd mb;

	mbcmd_set(mb, MBCMD(TCTL), task->tid, OMAP_DSP_MBCMD_TCTL_TINIT);
	dsp_mbsend(&mb);
}

int dsp_task_config_all(unsigned char n)
{
	int i, ret;
	struct taskdev *devheap;
	struct dsptask *taskheap;
	size_t devheapsz, taskheapsz;

	memset(taskdev, 0, sizeof(void *) * TASKDEV_MAX);
	memset(dsptask, 0, sizeof(void *) * TASKDEV_MAX);

	n_task = n;
	printk(KERN_INFO "omapdsp: found %d task(s)\n", n_task);
	if (n_task == 0)
		return 0;

	/*
	 * reducing kmalloc!
	 */
	devheapsz  = sizeof(struct taskdev) * n_task;
	taskheapsz = sizeof(struct dsptask) * n_task;
	heap = kmalloc(devheapsz + taskheapsz, GFP_KERNEL);
	if (heap == NULL) {
		n_task = 0;
		return -ENOMEM;
	}
	memset(heap, 0, devheapsz + taskheapsz);
	devheap  = heap;
	taskheap = heap + devheapsz;

	for (i = 0; i < n_task; i++) {
		struct taskdev *dev  = &devheap[i];
		struct dsptask *task = &taskheap[i];

		if ((ret = dsp_task_config(task, i)) < 0)
			return ret;
		if ((ret = taskdev_init(dev, task->name, i)) < 0)
			return ret;
		taskdev_attach_task(dev, task);
		dsp_task_init(task);
		printk(KERN_INFO "omapdsp: taskdev %s enabled.\n", dev->name);
	}

	return 0;
}

static void dsp_task_unconfig(struct dsptask *task)
{
	unsigned char tid = task->tid;

	preempt_disable();
	dsp_task_flush_buf(task);
	if (sndtyp_wd(task->ttyp) && (task->state == TASK_STATE_READY))
		free_fifo(&task->rcvdt.fifo);
	dsptask[tid] = NULL;
	preempt_enable();
}

void dsp_task_unconfig_all(void)
{
	unsigned char minor;
	unsigned char tid;
	struct dsptask *task;

	for (minor = 0; minor < n_task; minor++) {
		/*
		 * taskdev[minor] can be NULL in case of
		 * configuration failure
		 */
		if (taskdev[minor])
			taskdev_delete(minor);
	}
	for (; minor < TASKDEV_MAX; minor++) {
		if (taskdev[minor])
			dsp_rmdev_minor(minor);
	}

	for (tid = 0; tid < n_task; tid++) {
		/*
		 * dsptask[tid] can be NULL in case of
		 * configuration failure
		 */
		task = dsptask[tid];
		if (task)
			dsp_task_unconfig(task);
	}
	for (; tid < TASKDEV_MAX; tid++) {
		task = dsptask[tid];
		if (task) {
			/*
			 * on-demand tasks should be deleted in
			 * rmdev_minor(), but just in case.
			 */
			dsp_task_unconfig(task);
			kfree(task);
		}
	}

	if (heap) {
		kfree(heap);
		heap = NULL;
	}

	n_task = 0;
}

unsigned char dsp_task_count(void)
{
	return n_task;
}

int dsp_taskmod_busy(void)
{
	struct taskdev *dev;
	unsigned char minor;

	for (minor = 0; minor < TASKDEV_MAX; minor++) {
		dev = taskdev[minor];
		if (dev &&
		    ((dev->usecount > 0) ||
		     (dev->state == OMAP_DSP_DEVSTATE_ADDREQ) ||
		     (dev->state == OMAP_DSP_DEVSTATE_DELREQ)))
			return 1;
	}
	return 0;
}

/*
 * DSP task device file operations
 */
static ssize_t dsp_task_read_wd_acv(struct file *file, char *buf, size_t count,
				    loff_t *ppos)
{
	unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);
	struct taskdev *dev = taskdev[minor];
	int ret = 0;

	if (count == 0) {
		return 0;
	} else if (count == 1) {
		printk(KERN_ERR
		       "omapdsp: count=1 is illegal for dsp_task_read().\n");
		return -EINVAL;
	} else if (count & 0x1) {
		/* force even value */
		count--;
	}

	if (down_tasksem_interruptible(dev, &dev->read_sem))
		return -ERESTARTSYS;
	if (devstate_lock(dev, OMAP_DSP_DEVSTATE_ATTACHED) < 0) {
		ret = -ERESTARTSYS;
		goto up_out;
	}

	if (fifo_empty(&dev->task->rcvdt.fifo)) {
		long current_state = current->state;
		DECLARE_WAITQUEUE(wait, current);

		set_current_state(TASK_INTERRUPTIBLE);
		add_wait_queue(&dev->read_wait_q, &wait);
		if (fifo_empty(&dev->task->rcvdt.fifo)) {	/* last check */
			devstate_unlock(dev);
			schedule();
		}
		set_current_state(current_state);
		remove_wait_queue(&dev->read_wait_q, &wait);
		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			goto up_out;
		}
		if (devstate_lock(dev, OMAP_DSP_DEVSTATE_ATTACHED) < 0) {
			ret = -ERESTARTSYS;
			goto up_out;
		}
		if (fifo_empty(&dev->task->rcvdt.fifo))	/* should not occur */
			goto unlock_out;
	}

	/* data protection while copying */
	disable_irq(INT_D2A_MB1);
	ret = copy_to_user_fm_fifo(buf, &dev->task->rcvdt.fifo, count);
	enable_irq(INT_D2A_MB1);

unlock_out:
	devstate_unlock(dev);
up_out:
	up(&dev->read_sem);
	return ret;
}

static ssize_t dsp_task_read_bk_acv(struct file *file, char *buf, size_t count,
				    loff_t *ppos)
{
	unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);
	struct taskdev *dev = taskdev[minor];
	struct rcvdt_bk_struct *rcvdt;
	int dspmem_access = 0;
	ssize_t ret = 0;

	if (count == 0) {
		return 0;
	} else if (count == 1) {
		printk(KERN_ERR
		       "omapdsp: count=1 is illegal for dsp_task_read().\n");
		return -EINVAL;
	} else if ((int)buf & 0x1) {
		printk(KERN_ERR
		       "omapdsp: buf should be word aligned for "
		       "dsp_task_read().\n");
		return -EINVAL;
	} else if (count & 0x1) {
		/* force even value */
		count--;
	}

	if (down_tasksem_interruptible(dev, &dev->read_sem))
		return -ERESTARTSYS;
	if (devstate_lock(dev, OMAP_DSP_DEVSTATE_ATTACHED) < 0) {
		ret = -ERESTARTSYS;
		goto up_out;
	}

	if (ipblink_empty(&dev->task->rcvdt.bk.link)) {
		long current_state;
		DECLARE_WAITQUEUE(wait, current);

		add_wait_queue(&dev->read_wait_q, &wait);
		current_state = current->state;
		set_current_state(TASK_INTERRUPTIBLE);
		if (ipblink_empty(&dev->task->rcvdt.bk.link)) {	/* last check */
			devstate_unlock(dev);
			schedule();
		}
		set_current_state(current_state);
		remove_wait_queue(&dev->read_wait_q, &wait);
		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			goto up_out;
		}
		if (devstate_lock(dev, OMAP_DSP_DEVSTATE_ATTACHED) < 0) {
			ret = -ERESTARTSYS;
			goto up_out;
		}
		/* signal or 0-byte send from DSP */
		if (ipblink_empty(&dev->task->rcvdt.bk.link))
			goto unlock_out;
	}

	/* data protection while copying */
	disable_irq(INT_D2A_MB1);

	rcvdt = &dev->task->rcvdt.bk;
	/* copy from delayed IPBUF */
	if (sndtyp_pvt(dev->task->ttyp)) {
		/* private */
		if (!ipblink_empty(&rcvdt->link)) {
			struct ipbuf_p *ipbp = rcvdt->ipbuf_pvt_r;
			unsigned long dspadr;
			unsigned char *src;
			size_t bkcnt;

			if ((dspmem_access = is_dsp_internal_mem(ipbp)) != 0)
				dsp_mem_enable();
			dspadr = MKLONG(ipbp->ah, ipbp->al);
			if (!dspmem_access &&
			    ((dspmem_access = is_dspword_internal_mem(dspadr)) != 0))
				dsp_mem_enable();
			src = dspword_to_virt(dspadr) + rcvdt->rp;
			bkcnt = ((unsigned long)ipbp->c) * 2 - rcvdt->rp;
			if (bkcnt > count) {
				if (copy_to_user_dsp(buf, src, count)) {
					ret = -EFAULT;
					goto enirq_out;
				}
				ret = count;
				rcvdt->rp += count;
			} else {
				if (copy_to_user_dsp(buf, src, bkcnt)) {
					ret = -EFAULT;
					goto enirq_out;
				}
				ret = bkcnt;
				ipblink_del_pvt(&rcvdt->link);
				release_ipbuf_pvt(ipbp);
				rcvdt->rp = 0;
			}
		}
	} else {
		/* global */
		if ((dspmem_access = is_ipbuf_internal_mem()) != 0)
			dsp_mem_enable();
		while (!ipblink_empty(&rcvdt->link)) {
			unsigned char *src;
			size_t bkcnt;
			unsigned short bid = rcvdt->link.top;
			struct ipbuf *ipbp = ipbuf[bid];

			src = ipbp->d + rcvdt->rp;
			bkcnt = ((unsigned long)ipbp->c) * 2 - rcvdt->rp;
			if (bkcnt > count) {
				if (copy_to_user_dsp(buf, src, count)) {
					ret = -EFAULT;
					goto enirq_out;
				}
				ret += count;
				rcvdt->rp += count;
				break;
			} else {
				if (copy_to_user_dsp(buf, src, bkcnt)) {
					ret = -EFAULT;
					goto enirq_out;
				}
				ret += bkcnt;
				buf += bkcnt;
				count -= bkcnt;
				ipblink_del_top(&rcvdt->link, ipbuf);
				unuse_ipbuf(bid);
				rcvdt->rp = 0;
			}
		}
	}

enirq_out:
	if (dspmem_access)
		dsp_mem_disable();
	enable_irq(INT_D2A_MB1);
unlock_out:
	devstate_unlock(dev);
up_out:
	up(&dev->read_sem);
	return ret;
}

static ssize_t dsp_task_read_wd_psv(struct file *file, char *buf, size_t count,
				    loff_t *ppos)
{
	unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);
	struct taskdev *dev = taskdev[minor];
	struct mbcmd mb;
	unsigned char tid;
	int ret = 0;

	if (count == 0) {
		return 0;
	} else if (count == 1) {
		printk(KERN_ERR
		       "omapdsp: count=1 is illegal for dsp_task_read().\n");
		return -EINVAL;
	} else {
		/* force! */
		count = 2;
	}

	if (down_tasksem_interruptible(dev, &dev->read_sem))
		return -ERESTARTSYS;
	if (devstate_lock(dev, OMAP_DSP_DEVSTATE_ATTACHED) < 0) {
		ret = -ERESTARTSYS;
		goto up_out;
	}
	tid = dev->task->tid;
	devstate_unlock(dev);

	mbcmd_set(mb, MBCMD(WDREQ), tid, 0);
	dsp_mbsend_and_wait(&mb, &dev->read_wait_q);

	if (signal_pending(current)) {
		ret = -ERESTARTSYS;
		goto up_out;
	}
	if (devstate_lock(dev, OMAP_DSP_DEVSTATE_ATTACHED) < 0) {
		ret = -ERESTARTSYS;
		goto up_out;
	}
	if (fifo_empty(&dev->task->rcvdt.fifo))	/* should not occur */
		goto unlock_out;
	ret = copy_to_user_fm_fifo(buf, &dev->task->rcvdt.fifo, count);

unlock_out:
	devstate_unlock(dev);
up_out:
	up(&dev->read_sem);
	return ret;
}

static ssize_t dsp_task_read_bk_psv(struct file *file, char *buf, size_t count,
				    loff_t *ppos)
{
	unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);
	struct taskdev *dev = taskdev[minor];
	struct rcvdt_bk_struct *rcvdt;
	struct mbcmd mb;
	unsigned char tid;
	int dspmem_access = 0;
	int ret = 0;

	if (count == 0) {
		return 0;
	} else if (count == 1) {
		printk(KERN_ERR
		       "omapdsp: count=1 is illegal for dsp_task_read().\n");
		return -EINVAL;
	} else if ((int)buf & 0x1) {
		printk(KERN_ERR
		       "omapdsp: buf should be word aligned for dsp_task_read().\n");
		return -EINVAL;
	} else if (count & 0x1) {
		/* force even value */
		count--;
	}

	if (down_tasksem_interruptible(dev, &dev->read_sem))
		return -ERESTARTSYS;
	if (devstate_lock(dev, OMAP_DSP_DEVSTATE_ATTACHED) < 0) {
		ret = -ERESTARTSYS;
		goto up_out;
	}
	tid = dev->task->tid;
	devstate_unlock(dev);

	mbcmd_set(mb, MBCMD(BKREQ), tid, count/2);
	dsp_mbsend_and_wait(&mb, &dev->read_wait_q);

	if (signal_pending(current)) {
		ret = -ERESTARTSYS;
		goto up_out;
	}
	if (devstate_lock(dev, OMAP_DSP_DEVSTATE_ATTACHED) < 0) {
		ret = -ERESTARTSYS;
		goto up_out;
	}
	rcvdt = &dev->task->rcvdt.bk;
	/* signal or 0-byte send from DSP */
	if (ipblink_empty(&rcvdt->link))
		goto unlock_out;

	/*
	 * We will not receive more than requested count.
	 */
	if (sndtyp_pvt(dev->task->ttyp)) {
		/* private */
		struct ipbuf_p *ipbp = rcvdt->ipbuf_pvt_r;
		size_t rcvcnt;
		unsigned long dspadr;
		void *src;

		if ((dspmem_access = is_dsp_internal_mem(ipbp)) != 0)
			dsp_mem_enable();
		dspadr = MKLONG(ipbp->ah, ipbp->al);
		if (!dspmem_access &&
		    ((dspmem_access = is_dspword_internal_mem(dspadr)) != 0))
			dsp_mem_enable();
		rcvcnt = ((unsigned long)ipbp->c) * 2;
		if (count > rcvcnt)
			count = rcvcnt;
		src = dspword_to_virt(dspadr);
		if (copy_to_user_dsp(buf, src, count)) {
			ret = -EFAULT;
			goto unlock_out;
		}
		ipblink_del_pvt(&rcvdt->link);
		release_ipbuf_pvt(ipbp);
		ret = count;
	} else {
		/* global */
		unsigned short bid = rcvdt->link.top;
		struct ipbuf *ipbp = ipbuf[bid];
		size_t rcvcnt;

		if ((dspmem_access = is_ipbuf_internal_mem()) != 0)
			dsp_mem_enable();
		rcvcnt = ((unsigned long)ipbp->c) * 2;
		if (count > rcvcnt)
			count = rcvcnt;
		if (copy_to_user_dsp(buf, ipbp->d, count)) {
			ret = -EFAULT;
			goto unlock_out;
		}
		ipblink_del_top(&rcvdt->link, ipbuf);
		unuse_ipbuf(bid);
		ret = count;
	}

unlock_out:
	if (dspmem_access)
		dsp_mem_disable();
	devstate_unlock(dev);
up_out:
	up(&dev->read_sem);
	return ret;
}

static ssize_t dsp_task_write_wd(struct file *file, const char *buf,
				 size_t count, loff_t *ppos)
{
	unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);
	struct taskdev *dev = taskdev[minor];
	struct mbcmd mb;
	unsigned short wd;
	int ret = 0;

	if (count == 0) {
		return 0;
	} else if (count & 0x1) {
		printk(KERN_ERR
		       "omapdsp: odd count is illegal for dsp_task_write().\n");
		return -EINVAL;
	} else {
		/* force! */
		count = 2;
	}

	if (down_tasksem_interruptible(dev, &dev->write_sem))
		return -ERESTARTSYS;
	if (devstate_lock(dev, OMAP_DSP_DEVSTATE_ATTACHED) < 0) {
		ret = -ERESTARTSYS;
		goto up_out;
	}

	if (dev->task->wsz == 0) {
		long current_state;
		DECLARE_WAITQUEUE(wait, current);

		add_wait_queue(&dev->write_wait_q, &wait);
		current_state = current->state;
		set_current_state(TASK_INTERRUPTIBLE);
		if (dev->task->wsz == 0) {	/* last check */
			devstate_unlock(dev);
			schedule();
		}
		set_current_state(current_state);
		remove_wait_queue(&dev->write_wait_q, &wait);
		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			goto up_out;
		}
		if (devstate_lock(dev, OMAP_DSP_DEVSTATE_ATTACHED) < 0) {
			ret = -ERESTARTSYS;
			goto up_out;
		}
		if (dev->task->wsz == 0)	/* should not occur */
			goto up_out;
	}

	if (copy_from_user(&wd, buf, count)) {
		ret = -EFAULT;
		goto unlock_out;
	}

	/* wsz protection */
	disable_irq(INT_D2A_MB1);
	mbcmd_set(mb, MBCMD(WDSND), dev->task->tid, wd);
	if (dsp_mbsend(&mb) < 0)
		goto enirq_out;
	ret = count;
	if (rcvtyp_acv(dev->task->ttyp))
		dev->task->wsz = 0;

enirq_out:
	enable_irq(INT_D2A_MB1);
unlock_out:
	devstate_unlock(dev);
up_out:
	up(&dev->write_sem);
	return ret;
}

static ssize_t dsp_task_write_bk(struct file *file, const char *buf,
				 size_t count, loff_t *ppos)
{
	unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);
	struct taskdev *dev = taskdev[minor];
	struct mbcmd mb;
	int dspmem_access = 0;
	int ret = 0;

	if (count == 0) {
		return 0;
	} else if ((int)buf & 0x1) {
		printk(KERN_ERR
		       "omapdsp: buf should be word aligned for "
		       "dsp_task_write().\n");
		return -EINVAL;
	} else if (count & 0x1) {
		printk(KERN_ERR
		       "omapdsp: odd count is illegal for dsp_task_write().\n");
		return -EINVAL;
	}

	if (down_tasksem_interruptible(dev, &dev->write_sem))
		return -ERESTARTSYS;
	if (devstate_lock(dev, OMAP_DSP_DEVSTATE_ATTACHED) < 0) {
		ret = -ERESTARTSYS;
		goto up_out;
	}

	if (dev->task->wsz == 0) {
		long current_state;
		DECLARE_WAITQUEUE(wait, current);

		add_wait_queue(&dev->write_wait_q, &wait);
		current_state = current->state;
		set_current_state(TASK_INTERRUPTIBLE);
		if (dev->task->wsz == 0) {	/* last check */
			devstate_unlock(dev);
			schedule();
		}
		set_current_state(current_state);
		remove_wait_queue(&dev->write_wait_q, &wait);
		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			goto up_out;
		}
		if (devstate_lock(dev, OMAP_DSP_DEVSTATE_ATTACHED) < 0) {
			ret = -ERESTARTSYS;
			goto up_out;
		}
		if (dev->task->wsz == 0)	/* should not occur */
			goto unlock_out;
	}

	/* wsz protection */
	disable_irq(INT_D2A_MB1);

	if (count > dev->task->wsz)
		count = dev->task->wsz;

	if (rcvtyp_pvt(dev->task->ttyp)) {
		/* private */
		struct ipbuf_p *ipbp = dev->task->ipbuf_pvt_w;
		unsigned long dspadr;
		unsigned char *dst;

		if ((dspmem_access = is_dsp_internal_mem(ipbp)) != 0)
			dsp_mem_enable();
		if (ipbp->s != OMAP_DSP_TID_FREE)
			goto enirq_out;
		dspadr = MKLONG(ipbp->ah, ipbp->al);
		if (!dspmem_access &&
		    ((dspmem_access = is_dspword_internal_mem(dspadr)) != 0))
			dsp_mem_enable();
		dst = dspword_to_virt(dspadr);
		if (copy_from_user_dsp(dst, buf, count)) {
			ret = -EFAULT;
			goto enirq_out;
		}
		ipbp->c = count/2;
		ipbp->s = dev->task->tid;
		mbcmd_set(mb, MBCMD(BKSNDP), dev->task->tid, 0);
		if (dsp_mbsend(&mb) < 0)
			goto enirq_out;
	} else {
		/* global */
		struct ipbuf *ipbp;
		unsigned short bid;

		if ((dspmem_access = is_ipbuf_internal_mem()) != 0)
			dsp_mem_enable();
		bid = get_free_ipbuf(dev->task->tid);
		if (bid == OMAP_DSP_BID_NULL)
			goto enirq_out;
		ipbp = ipbuf[bid];
		if (copy_from_user_dsp(ipbp->d, buf, count)) {
			release_ipbuf(bid);
			ret = -EFAULT;
			goto enirq_out;
		}
		ipbp->c  = count/2;
		ipbp->sa = dev->task->tid;
		mbcmd_set(mb, MBCMD(BKSND), dev->task->tid, bid);
		if (dsp_mbsend(&mb) < 0) {
			release_ipbuf(bid);
			goto enirq_out;
		}
		ipb_bsycnt_inc(&ipbcfg);
	}
	ret = count;

	if (rcvtyp_acv(dev->task->ttyp))
		dev->task->wsz = 0;
enirq_out:
	if (dspmem_access)
		dsp_mem_disable();
	enable_irq(INT_D2A_MB1);
unlock_out:
	devstate_unlock(dev);
up_out:
	up(&dev->write_sem);
	return ret;
}

static unsigned int dsp_task_poll(struct file * file, poll_table * wait)
{
	unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);
	struct taskdev *dev = taskdev[minor];
	struct dsptask *task = dev->task;
	unsigned int mask = 0;

	poll_wait(file, &dev->read_wait_q, wait);
	poll_wait(file, &dev->write_wait_q, wait);
	if (devstate_lock(dev, OMAP_DSP_DEVSTATE_ATTACHED) < 0)
		return 0;
	if (sndtyp_psv(task->ttyp) ||
	    (sndtyp_wd(task->ttyp) && !fifo_empty(&task->rcvdt.fifo)) ||
	    (sndtyp_bk(task->ttyp) && !ipblink_empty(&task->rcvdt.bk.link)))
		mask |= POLLIN | POLLRDNORM;
	if (task->wsz)
		mask |= POLLOUT | POLLWRNORM;
	devstate_unlock(dev);

	return mask;
}

static int dsp_task_ioctl(struct inode *inode, struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	unsigned int minor = MINOR(inode->i_rdev);
	struct taskdev *dev = taskdev[minor];
	struct mbcmd mb;
	unsigned char tid;
	struct mb_exarg mbarg, *mbargp;
	int mbargc;
	unsigned short mbargv[1];
	int interactive;
	int ret;

	/* LOCK / UNLOCK operations */
	switch (cmd) {
	case OMAP_DSP_TASK_IOCTL_LOCK:
		return taskdev_lock(dev);
	case OMAP_DSP_TASK_IOCTL_UNLOCK:
		return taskdev_unlock(dev);
	}

	/*
	 * actually only interractive commands need to lock
	 * the semaphore, but here all commands do it for simplicity.
	 */
	if (down_tasksem_interruptible(dev, &dev->ioctl_sem))
		return -ERESTARTSYS;
	if (devstate_lock(dev, OMAP_DSP_DEVSTATE_ATTACHED) < 0) {
		ret = -ERESTARTSYS;
		goto up_out;
	}

	if ((cmd >= 0x0080) && (cmd < 0x0100)) {
		/*
		 * 0x0080 - 0x00ff
		 * reserved for backward compatibility
		 * user-defined TCTL commands: no arg, non-interactive
		 */
		mbargc = 0;
		interactive = 0;
	} else if (cmd < 0x8000) {
		/*
		 * 0x0000 - 0x7fff (except 0x0080 - 0x00ff)
		 * system reserved TCTL commands
		 */
		switch (cmd) {
		case OMAP_DSP_MBCMD_TCTL_TEN:
		case OMAP_DSP_MBCMD_TCTL_TDIS:
			mbargc = 0;
			interactive = 0;
			break;
		default:
			ret = -ENOIOCTLCMD;
			goto unlock_out;
		}
	}
	/*
	 * 0x8000 - 0xffff
	 * user-defined TCTL commands
	 */
	else if (cmd < 0x8100) {
		/* 0x8000-0x80ff: no arg, non-interactive */
		mbargc = 0;
		interactive = 0;
	} else if (cmd < 0x8200) {
		/* 0x8100-0x81ff: 1 arg, non-interactive */
		mbargc = 1;
		mbargv[0] = arg & 0xffff;
		interactive = 0;
	} else if (cmd < 0x9000) {
		/* 0x8200-0x8fff: reserved */
		ret = -ENOIOCTLCMD;
		goto up_out;
	} else if (cmd < 0x9100) {
		/* 0x9000-0x90ff: no arg, interactive */
		mbargc = 0;
		interactive = 1;
	} else if (cmd < 0x9200) {
		/* 0x9100-0x91ff: 1 arg, interactive */
		mbargc = 1;
		mbargv[0] = arg & 0xffff;
		interactive = 1;
	} else if (cmd < 0x10000) {
		/* 0x9200-0xffff: reserved */
		ret =  -ENOIOCTLCMD;
		goto up_out;
	} else {
		/*
		 * 0x10000 -
		 * non TCTL ioctls
		 */
		switch (cmd) {
		case OMAP_DSP_TASK_IOCTL_BFLSH:
			ret = dsp_task_flush_buf(dev->task);
			break;
		case OMAP_DSP_TASK_IOCTL_SETBSZ:
			ret = dsp_task_set_fifosz(dev->task, arg);
			break;
		case OMAP_DSP_TASK_IOCTL_GETNAME:
			ret = 0;
			if (copy_to_user((void *)arg, dev->name,
					 strlen(dev->name) + 1))
				ret = -EFAULT;
			break;
		default:
			ret = -ENOIOCTLCMD;
		}
		goto unlock_out;
	}

	/*
	 * issue TCTL
	 */
	tid = dev->task->tid;
	mbcmd_set(mb, MBCMD(TCTL), tid, cmd);
	if (mbargc > 0) {
		mbarg.argc = mbargc;
		mbarg.tid  = tid;
		mbarg.argv = mbargv;
		mbargp = &mbarg;
	} else
		mbargp = NULL;

	if (interactive) {
		dev->task->tctl_stat = -ERESTARTSYS;
		dsp_mbsend_and_wait_exarg(&mb, mbargp, &dev->ioctl_wait_q);
		ret = dev->task->tctl_stat;
		if (ret < 0) {
			printk(KERN_ERR "omapdsp: TCTL not responding.\n");
			goto unlock_out;
		}
	} else {
		dsp_mbsend_exarg(&mb, mbargp);
		ret = 0;
	}

unlock_out:
	devstate_unlock(dev);
up_out:
	up(&dev->ioctl_sem);
	return ret;
}

/**
 * On demand page allocation is not allowed. The mapping area is defined by
 * corresponding DSP tasks.
 */
static struct page *dsp_task_nopage_mmap(struct vm_area_struct *vma,
					 unsigned long address, int *type)
{
	return NOPAGE_SIGBUS;
}

static struct vm_operations_struct dsp_task_vm_ops = {
	.nopage = dsp_task_nopage_mmap,
};

static int dsp_task_mmap(struct file *filp, struct vm_area_struct *vma)
{
	void *tmp_vadr;
	unsigned long tmp_padr, tmp_vmadr, off;
	size_t req_len, tmp_len;
	unsigned int minor = MINOR(filp->f_dentry->d_inode->i_rdev);
	struct taskdev *dev = taskdev[minor];
	struct dsptask *task;
	int ret = 0;

	if (devstate_lock(dev, OMAP_DSP_DEVSTATE_ATTACHED) < 0)
		return -ERESTARTSYS;
	task = dev->task;
	if (task->map_length == 0) {
		printk(KERN_ERR
		       "omapdsp: task %s doesn't have mmap buffer.\n",
		       task->name);
		ret = -EINVAL;
		goto unlock_out;
	}
	if (is_dsp_internal_mem(task->map_base)) {
		printk(KERN_ERR
		       "omapdsp: task %s: map_base = %p\n"
		       "    DARAM/SARAM can't be used as mmap buffer.\n",
		       task->name, task->map_base);
		ret = -EINVAL;
		goto unlock_out;
	}

	/*
	 * Don't swap this area out
	 * Don't dump this area to a core file
	 */
	vma->vm_flags |= VM_RESERVED | VM_IO;

	/* Do not cache this area */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	req_len = vma->vm_end - vma->vm_start;
	off = vma->vm_pgoff << PAGE_SHIFT;
	tmp_vmadr = vma->vm_start;
	tmp_vadr = task->map_base + off;
	do {
		tmp_padr = dsp_virt_to_phys(tmp_vadr, &tmp_len);
		if (tmp_padr == 0) {
			printk(KERN_ERR
			       "omapdsp: task %s: illegal address "
			       "for mmap: %p", task->name, tmp_vadr);
			/* partial mapping will be cleared in upper layer */
			ret = -EINVAL;
			goto unlock_out;
		}
		if (tmp_len > req_len)
			tmp_len = req_len;

		printk(KERN_DEBUG
		       "omapdsp: mmap info: "
		       "vmadr = %08lx, padr = %08lx, len = %x\n",
		       tmp_vmadr, tmp_padr, tmp_len);
		if (remap_pfn_range(vma, tmp_vmadr, tmp_padr >> PAGE_SHIFT,
				    tmp_len, vma->vm_page_prot) != 0) {
			printk(KERN_ERR
			       "omapdsp: task %s: remap_page_range() failed.\n",
			       task->name);
			/* partial mapping will be cleared in upper layer */
			ret = -EINVAL;
			goto unlock_out;
		}

		req_len   -= tmp_len;
		tmp_vmadr += tmp_len;
		tmp_vadr  += tmp_len;
	} while (req_len);

	vma->vm_ops = &dsp_task_vm_ops;
unlock_out:
	devstate_unlock(dev);
	return ret;
}

static int dsp_task_open(struct inode *inode, struct file *file)
{
	unsigned int minor = MINOR(inode->i_rdev);
	struct taskdev *dev = taskdev[minor];
	int ret = 0;

	if (devstate_lock(dev, OMAP_DSP_DEVSTATE_NOTASK |
			       OMAP_DSP_DEVSTATE_ATTACHED) < 0)
		return -ERESTARTSYS;
#ifndef CONFIG_OMAP_DSP_TASK_MULTIOPEN
	if (dev->usecount > 0) {
		ret = -EBUSY;
		goto unlock_out;
	}
#endif

	if (dev->state == OMAP_DSP_DEVSTATE_NOTASK) {
		dev->state = OMAP_DSP_DEVSTATE_ADDREQ;
		/* wake up twch daemon for tadd */
		dsp_twch_touch();
		devstate_unlock(dev);
		if (devstate_lock(dev, OMAP_DSP_DEVSTATE_ATTACHED |
				       OMAP_DSP_DEVSTATE_ADDFAIL) < 0)
			return -ERESTARTSYS;
		if (dev->state == OMAP_DSP_DEVSTATE_ADDFAIL) {
			printk(KERN_ERR "omapdsp: task attach failed for %s!\n",
			       dev->name);
			ret = -EBUSY;
			dev->state = OMAP_DSP_DEVSTATE_NOTASK;
			wake_up_interruptible_all(&dev->state_wait_q);
			goto unlock_out;
		}
	}

	/* state_lock covers usecount, proc_list as well. */
	dev->usecount++;
	proc_list_add(&dev->proc_list, current);
	file->f_op = &dev->fops;
	devstate_unlock(dev);

	dsp_map_update(current);
	dsp_cur_users_add(current);
	return 0;

unlock_out:
	devstate_unlock(dev);
	return ret;
}

static int dsp_task_release(struct inode *inode, struct file *file)
{
	unsigned int minor = MINOR(inode->i_rdev);
	struct taskdev *dev = taskdev[minor];

	dsp_cur_users_del(current);

	/* state_lock covers usecount, proc_list as well. */
	spin_lock(&dev->state_lock);

	/* state can be ATTACHED, KILLREQ or GARBAGE here. */
	switch (dev->state) {

	case OMAP_DSP_DEVSTATE_KILLREQ:
		dev->usecount--;
		break;

	case OMAP_DSP_DEVSTATE_GARBAGE:
		if(--dev->usecount == 0) {
			dev->state = OMAP_DSP_DEVSTATE_NOTASK;
			wake_up_interruptible_all(&dev->state_wait_q);
		}
		break;

	case OMAP_DSP_DEVSTATE_ATTACHED:
		if (dev->lock_pid == current->pid)
			taskdev_unlock(dev);
		proc_list_del(&dev->proc_list, current);
		if (--dev->usecount == 0) {
			if (minor >= n_task) {	/* dynamic task */
				dev->state = OMAP_DSP_DEVSTATE_DELREQ;
				/* wake up twch daemon for tdel */
				dsp_twch_touch();
			}
		}
		break;

	}

	spin_unlock(&dev->state_lock);
	return 0;
}

/*
 * mkdev / rmdev
 */
int dsp_mkdev(char *name)
{
	struct taskdev *dev;
	int status;
	unsigned char minor;

	if (!dsp_is_ready()) {
		printk(KERN_ERR "omapdsp: dsp has not been configured.\n");
		return -EINVAL;
	}
	for (minor = n_task; minor < TASKDEV_MAX; minor++) {
		if (taskdev[minor] == NULL)
			goto do_make;
	}
	printk(KERN_ERR "omapdsp: Too many task devices.\n");
	return -EBUSY;

do_make:
	if ((dev = kmalloc(sizeof(struct taskdev), GFP_KERNEL)) == NULL)
		return -ENOMEM;
	memset(dev, 0, sizeof(struct taskdev));
	if ((status = taskdev_init(dev, name, minor)) < 0) {
		kfree(dev);
		return status;
	}
	return minor;
}

int dsp_rmdev(char *name)
{
	unsigned char minor;
	int ret;

	if (!dsp_is_ready()) {
		printk(KERN_ERR "omapdsp: dsp has not been configured.\n");
		return -EINVAL;
	}
	for (minor = n_task; minor < TASKDEV_MAX; minor++) {
		if (taskdev[minor] && !strcmp(taskdev[minor]->name, name)) {
			if ((ret = dsp_rmdev_minor(minor)) < 0)
				return ret;
			return minor;
		}
	}
	return -EINVAL;
}

static int dsp_rmdev_minor(unsigned char minor)
{
	struct taskdev *dev = taskdev[minor];
	struct dsptask *task = dev->task;

	spin_lock(&dev->state_lock);

	switch (dev->state) {

	case OMAP_DSP_DEVSTATE_NOTASK:
		/* fine */
		break;

	case OMAP_DSP_DEVSTATE_ATTACHED:
		/* task is working. kill it. */
		{
			siginfo_t info;
			struct list_head *ptr;
			struct proc_list *pl;

			info.si_signo = SIGBUS;
			info.si_errno = 0;
			info.si_code = SI_KERNEL;
			info._sifields._sigfault._addr = NULL;
			list_for_each(ptr, &dev->proc_list) {
				pl = list_entry(ptr, struct proc_list, list_head);
				send_sig_info(SIGBUS, &info, pl->tsk);
			}
			taskdev_detach_task(dev);
			dsp_task_unconfig(task);
			kfree(task);
			dev->state = OMAP_DSP_DEVSTATE_GARBAGE;
		}
		break;

	case OMAP_DSP_DEVSTATE_ADDREQ:
		/* open() is waiting. drain it. */
		dev->state = OMAP_DSP_DEVSTATE_ADDFAIL;
		wake_up_interruptible_all(&dev->state_wait_q);
		break;

	case OMAP_DSP_DEVSTATE_DELREQ:
		/* nobody is waiting. */
		dev->state = OMAP_DSP_DEVSTATE_NOTASK;
		wake_up_interruptible_all(&dev->state_wait_q);
		break;

	case OMAP_DSP_DEVSTATE_KILLREQ:
	case OMAP_DSP_DEVSTATE_GARBAGE:
	case OMAP_DSP_DEVSTATE_ADDFAIL:
		/* transient state. wait for a moment. */
		break;

	}

	spin_unlock(&dev->state_lock);

	/* wait for some time and hope the state is settled */
	devstate_lock_timeout(dev, OMAP_DSP_DEVSTATE_NOTASK, HZ);
	if (dev->state != OMAP_DSP_DEVSTATE_NOTASK) {
		printk(KERN_WARNING
		       "omapdsp: illegal device state on rmdev %s.\n",
		       dev->name);
	}

	taskdev_delete(minor);
	kfree(dev);

	return 0;
}

struct file_operations dsp_task_fops = {
	.owner   = THIS_MODULE,
	.poll    = dsp_task_poll,
	.ioctl   = dsp_task_ioctl,
	.open    = dsp_task_open,
	.release = dsp_task_release,
	.mmap    = dsp_task_mmap,
};

static int taskdev_init(struct taskdev *dev, char *name, unsigned char minor)
{
	taskdev[minor] = dev;

	INIT_LIST_HEAD(&dev->proc_list);
	init_waitqueue_head(&dev->read_wait_q);
	init_waitqueue_head(&dev->write_wait_q);
	init_waitqueue_head(&dev->ioctl_wait_q);
	init_MUTEX(&dev->read_sem);
	init_MUTEX(&dev->write_sem);
	init_MUTEX(&dev->ioctl_sem);
	init_MUTEX(&dev->lock_sem);
	dev->lock_pid = 0;

	strncpy(dev->name, name, OMAP_DSP_TNM_LEN);
	dev->name[OMAP_DSP_TNM_LEN-1] = '\0';
	dev->state = (minor < n_task) ? OMAP_DSP_DEVSTATE_ATTACHED :
					OMAP_DSP_DEVSTATE_NOTASK;
	dev->usecount = 0;
	memcpy(&dev->fops, &dsp_task_fops, sizeof(struct file_operations));
	devfs_mk_cdev(MKDEV(OMAP_DSP_TASK_MAJOR, minor),
		      S_IFCHR | S_IRUGO | S_IWUGO, "dsptask/%s", dev->name);
#ifdef CONFIG_PROC_FS
	taskdev_create_proc(dev);
#endif

	init_waitqueue_head(&dev->state_wait_q);
	spin_lock_init(&dev->state_lock);

	return 0;
}

static void taskdev_delete(unsigned char minor)
{
	struct taskdev *dev = taskdev[minor];

	if (!dev)
		return;
#ifdef CONFIG_PROC_FS
	taskdev_remove_proc(dev);
#endif
	devfs_remove("dsptask/%s", dev->name);
	preempt_disable();
	proc_list_flush(&dev->proc_list);
	preempt_enable();
	taskdev[minor] = NULL;
}

static void taskdev_attach_task(struct taskdev *dev, struct dsptask *task)
{
	unsigned short ttyp = task->ttyp;

	dev->task = task;
	task->dev = dev;
	dev->fops.read =
		sndtyp_acv(ttyp) ?
			sndtyp_wd(ttyp) ? dsp_task_read_wd_acv:
			/* sndtyp_bk */   dsp_task_read_bk_acv:
		/* sndtyp_psv */
			sndtyp_wd(ttyp) ? dsp_task_read_wd_psv:
			/* sndtyp_bk */   dsp_task_read_bk_psv;
	dev->fops.write =
		rcvtyp_wd(ttyp) ? dsp_task_write_wd:
		/* rcvbyp_bk */	  dsp_task_write_bk;
}

static void taskdev_detach_task(struct taskdev *dev)
{
	if (dev->task) {
		dev->task = NULL;
		dev->fops.read = NULL;
		dev->fops.write = NULL;
		printk(KERN_INFO "omapdsp: taskdev %s disabled.\n", dev->name);
	}
}

/*
 * tadd / tdel / tkill
 */
int dsp_tadd(unsigned char minor, unsigned long adr)
{
	struct taskdev *dev;
	struct dsptask *task;
	struct mbcmd mb;
	struct mb_exarg arg;
	unsigned char tid;
	unsigned short argv[2];
	int ret = minor;

	if ((minor >= TASKDEV_MAX) || ((dev = taskdev[minor]) == NULL)) {
		printk(KERN_ERR
		       "omapdsp: no task device with minor %d\n", minor);
		return -EINVAL;
	}
	/*
	 * we don't need to lock state_lock because
	 * only tadd is allowed when devstate is ADDREQ.
	 */
	if (dev->state != OMAP_DSP_DEVSTATE_ADDREQ) {
		printk(KERN_ERR
		       "omapdsp: taskdev %s is not requesting for tadd.\n",
		       dev->name);
		return -EINVAL;
	}

	if (adr == OMAP_DSP_TADD_ABORTADR) {
		/* aborting tadd intentionally */
		printk(KERN_INFO "omapdsp: tadd address is ABORTADR.\n");
		goto fail_out;
	}
	if (adr >= DSPSPACE_SIZE) {
		printk(KERN_ERR
		       "omapdsp: illegal address 0x%08lx for tadd\n", adr);
		ret = -EINVAL;
		goto fail_out;
	}

	adr >>= 1;	/* word address */
	argv[0] = adr >> 16;	/* addrh */
	argv[1] = adr & 0xffff;	/* addrl */

	if (down_interruptible(&cfg_sem)) {
		ret = -ERESTARTSYS;
		goto fail_out;
	}
	cfg_tid = OMAP_DSP_TID_ANON;
	cfg_cmd = MBCMD(TADD);
	mbcmd_set(mb, MBCMD(TADD), 0, 0);
	arg.tid  = OMAP_DSP_TID_ANON;
	arg.argc = 2;
	arg.argv = argv;

	dsp_mbsend_and_wait_exarg(&mb, &arg, &cfg_wait_q);

	tid = cfg_tid;
	cfg_tid = OMAP_DSP_TID_ANON;
	cfg_cmd = 0;
	up(&cfg_sem);

	if (tid == OMAP_DSP_TID_ANON) {
		printk(KERN_ERR "omapdsp: tadd failed!\n");
		ret = -EINVAL;
		goto fail_out;
	}
	if ((tid < n_task) || dsptask[tid]) {
		printk(KERN_ERR "omapdsp: illegal tid (%d)!\n", tid);
		ret = -EINVAL;
		goto fail_out;
	}
	if ((task = kmalloc(sizeof(struct dsptask), GFP_KERNEL)) == NULL) {
		ret = -ENOMEM;
		goto fail_out;
	}
	memset(task, 0, sizeof(struct dsptask));

	if ((ret = dsp_task_config(task, tid)) < 0)
		goto free_out;
	taskdev_attach_task(dev, task);

	if (strcmp(dev->name, task->name)) {
		printk(KERN_ERR
		       "omapdsp: task name (%s) doesn't match with "
		       "device name (%s).\n", task->name, dev->name);
		ret = -EINVAL;
		dev->state = OMAP_DSP_DEVSTATE_DELREQ;
		dsp_twch_touch();
		return -EINVAL;
	}

	dsp_task_init(task);
	printk(KERN_INFO "omapdsp: taskdev %s enabled.\n", dev->name);
	dev->state = OMAP_DSP_DEVSTATE_ATTACHED;
	wake_up_interruptible_all(&dev->state_wait_q);
	return minor;

free_out:
	kfree(task);
fail_out:
	dev->state = OMAP_DSP_DEVSTATE_ADDFAIL;
	wake_up_interruptible_all(&dev->state_wait_q);
	return ret;
}

int dsp_tdel(unsigned char minor)
{
	struct taskdev *dev;
	struct dsptask *task;
	struct mbcmd mb;
	unsigned char tid, tid_response;
	int ret = minor;

	if ((minor >= TASKDEV_MAX) || ((dev = taskdev[minor]) == NULL)) {
		printk(KERN_ERR
		       "omapdsp: no task device with minor %d\n", minor);
		return -EINVAL;
	}
	/*
	 * we don't need to lock state_lock because
	 * only tdel is allowed when devstate is DELREQ.
	 */
	if (dev->state != OMAP_DSP_DEVSTATE_DELREQ) {
		printk(KERN_ERR
		       "omapdsp: taskdev %s is not requesting for tdel.\n",
		       dev->name);
		return -EINVAL;
	}

	task = dev->task;
	tid = task->tid;
	if (down_interruptible(&cfg_sem)) {
		return -ERESTARTSYS;
	}
	cfg_tid = OMAP_DSP_TID_ANON;
	cfg_cmd = MBCMD(TDEL);
	mbcmd_set(mb, MBCMD(TDEL), tid, OMAP_DSP_MBCMD_TDEL_SAFE);
	dsp_mbsend_and_wait(&mb, &cfg_wait_q);
	tid_response = cfg_tid;
	cfg_tid = OMAP_DSP_TID_ANON;
	cfg_cmd = 0;
	up(&cfg_sem);

	taskdev_detach_task(dev);
	dsp_task_unconfig(task);
	kfree(task);
	dev->state = OMAP_DSP_DEVSTATE_NOTASK;
	wake_up_interruptible_all(&dev->state_wait_q);

	if (tid_response != tid) {
		printk(KERN_ERR "omapdsp: tdel failed!\n");
		ret = -EINVAL;
	}

	return ret;
}

int dsp_tkill(unsigned char minor)
{
	struct taskdev *dev;
	struct dsptask *task;
	struct mbcmd mb;
	unsigned char tid, tid_response;
	siginfo_t info;
	struct list_head *ptr;
	struct proc_list *pl;
	int ret = minor;

	if ((minor >= TASKDEV_MAX) || ((dev = taskdev[minor]) == NULL)) {
		printk(KERN_ERR
		       "omapdsp: no task device with minor %d\n", minor);
		return -EINVAL;
	}
	spin_lock(&dev->state_lock);
	if (dev->state != OMAP_DSP_DEVSTATE_ATTACHED) {
		printk(KERN_ERR
		       "omapdsp: task has not been attached for taskdev %s\n",
		       dev->name);
		spin_unlock(&dev->state_lock);
		return -EINVAL;
	}
	dev->state = OMAP_DSP_DEVSTATE_KILLREQ;
	info.si_signo = SIGBUS;
	info.si_errno = 0;
	info.si_code = SI_KERNEL;
	info._sifields._sigfault._addr = NULL;
	list_for_each(ptr, &dev->proc_list) {
		pl = list_entry(ptr, struct proc_list, list_head);
		send_sig_info(SIGBUS, &info, pl->tsk);
	}
	spin_unlock(&dev->state_lock);

	task = dev->task;
	tid = task->tid;
	if (down_interruptible(&cfg_sem)) {
		tid_response = OMAP_DSP_TID_ANON;
		ret = -ERESTARTSYS;
		goto detach_out;
	}
	cfg_tid = OMAP_DSP_TID_ANON;
	cfg_cmd = MBCMD(TDEL);
	mbcmd_set(mb, MBCMD(TDEL), tid, OMAP_DSP_MBCMD_TDEL_KILL);
	dsp_mbsend_and_wait(&mb, &cfg_wait_q);
	tid_response = cfg_tid;
	cfg_tid = OMAP_DSP_TID_ANON;
	cfg_cmd = 0;
	up(&cfg_sem);

detach_out:
	taskdev_detach_task(dev);
	dsp_task_unconfig(task);
	kfree(task);

	if (tid_response != tid)
		printk(KERN_ERR "omapdsp: tkill failed!\n");

	spin_lock(&dev->state_lock);
	dev->state = (dev->usecount > 0) ? OMAP_DSP_DEVSTATE_GARBAGE :
					   OMAP_DSP_DEVSTATE_NOTASK;
	wake_up_interruptible_all(&dev->state_wait_q);
	spin_unlock(&dev->state_lock);

	return ret;
}

/*
 * state inquiry
 */
long taskdev_state(unsigned char minor)
{
	return taskdev[minor] ? taskdev[minor]->state :
				OMAP_DSP_DEVSTATE_NOTASK;
}

/*
 * functions called from mailbox1 interrupt routine
 */
void mbx1_wdsnd(struct mbcmd *mb)
{
	unsigned char tid = mb->cmd_l;
	struct dsptask *task = dsptask[tid];

	if ((tid >= TASKDEV_MAX) || (task == NULL)) {
		printk(KERN_ERR "mbx: WDSND with illegal tid! %d\n", tid);
		return;
	}
	if (sndtyp_bk(task->ttyp)) {
		printk(KERN_ERR
		       "mbx: WDSND from block sending task! (task%d)\n", tid);
		return;
	}
	if (sndtyp_psv(task->ttyp) &&
	    !waitqueue_active(&task->dev->read_wait_q)) {
		printk(KERN_WARNING
		       "mbx: WDSND from passive sending task (task%d) "
		       "without request!\n", tid);
		return;
	}

	write_word_to_fifo(&task->rcvdt.fifo, mb->data);
	wake_up_interruptible(&task->dev->read_wait_q);
}

void mbx1_wdreq(struct mbcmd *mb)
{
	unsigned char tid = mb->cmd_l;
	struct dsptask *task = dsptask[tid];

	if ((tid >= TASKDEV_MAX) || (task == NULL)) {
		printk(KERN_ERR "mbx: WDREQ with illegal tid! %d\n", tid);
		return;
	}
	if (rcvtyp_psv(task->ttyp)) {
		printk(KERN_ERR
		       "mbx: WDREQ from passive receiving task! (task%d)\n",
		       tid);
		return;
	}

	task->wsz = 2;
	wake_up_interruptible(&task->dev->write_wait_q);
}

void mbx1_bksnd(struct mbcmd *mb)
{
	unsigned char tid = mb->cmd_l;
	unsigned short bid = mb->data;
	struct dsptask *task = dsptask[tid];
	unsigned short cnt;

	if (bid >= ipbcfg.ln) {
		printk(KERN_ERR "mbx: BKSND with illegal bid! %d\n", bid);
		return;
	}
	ipb_bsycnt_dec(&ipbcfg);
	if ((tid >= TASKDEV_MAX) || (task == NULL)) {
		printk(KERN_ERR "mbx: BKSND with illegal tid! %d\n", tid);
		goto unuse_ipbuf_out;
	}
	if (sndtyp_wd(task->ttyp)) {
		printk(KERN_ERR
		       "mbx: BKSND from word sending task! (task%d)\n", tid);
		goto unuse_ipbuf_out;
	}
	if (sndtyp_pvt(task->ttyp)) {
		printk(KERN_ERR
		       "mbx: BKSND from private sending task! (task%d)\n", tid);
		goto unuse_ipbuf_out;
	}
	if (sync_with_dsp(&ipbuf[bid]->sd, tid, 10) < 0) {
		printk(KERN_ERR "mbx: BKSND - IPBUF sync failed!\n");
		return;
	}

	/* should be done in DSP, but just in case. */
	ipbuf[bid]->next = OMAP_DSP_BID_NULL;

	cnt = ipbuf[bid]->c;
	if (cnt > ipbcfg.lsz) {
		printk(KERN_ERR "mbx: BKSND cnt(%d) > ipbuf line size(%d)!\n",
		       cnt, ipbcfg.lsz);
		unuse_ipbuf_nowait(bid);
		return;
	}

	if (cnt == 0) {
		/* 0-byte send from DSP */
		unuse_ipbuf_nowait(bid);
		goto done;
	}
	ipblink_add_tail(&task->rcvdt.bk.link, bid, ipbuf);
	/* we keep coming bid and return alternative line to DSP. */
	balance_ipbuf(MBSEND_TYPE_NOWAIT);

done:
	wake_up_interruptible(&task->dev->read_wait_q);
	return;

unuse_ipbuf_out:
	unuse_ipbuf_nowait(bid);
	return;
}

void mbx1_bkreq(struct mbcmd *mb)
{
	unsigned char tid = mb->cmd_l;
	unsigned short cnt = mb->data;
	struct dsptask *task = dsptask[tid];

	if ((tid >= TASKDEV_MAX) || (task == NULL)) {
		printk(KERN_ERR "mbx: BKREQ with illegal tid! %d\n", tid);
		return;
	}
	if (rcvtyp_wd(task->ttyp)) {
		printk(KERN_ERR
		       "mbx: BKREQ from word receiving task! (task%d)\n", tid);
		return;
	}
	if (rcvtyp_pvt(task->ttyp)) {
		printk(KERN_ERR
		       "mbx: BKREQ from private receiving task! (task%d)\n",
		       tid);
		return;
	}
	if (rcvtyp_psv(task->ttyp)) {
		printk(KERN_ERR
		       "mbx: BKREQ from passive receiving task! (task%d)\n",
		       tid);
		return;
	}

	task->wsz = cnt*2;
	wake_up_interruptible(&task->dev->write_wait_q);
}

void mbx1_bkyld(struct mbcmd *mb)
{
	unsigned short bid = mb->data;

	if (bid >= ipbcfg.ln) {
		printk(KERN_ERR "mbx: BKYLD with illegal bid! %d\n", bid);
		return;
	}

	/* should be done in DSP, but just in case. */
	ipbuf[bid]->next = OMAP_DSP_BID_NULL;

	/* we don't need to sync with DSP */
	ipb_bsycnt_dec(&ipbcfg);
	release_ipbuf(bid);
}

void mbx1_bksndp(struct mbcmd *mb)
{
	unsigned char tid = mb->cmd_l;
	struct dsptask *task = dsptask[tid];
	struct rcvdt_bk_struct *rcvdt = &task->rcvdt.bk;
	struct ipbuf_p *ipbp = rcvdt->ipbuf_pvt_r;

	if ((tid >= TASKDEV_MAX) || (task == NULL)) {
		printk(KERN_ERR "mbx: BKSNDP with illegal tid! %d\n", tid);
		return;
	}
	if (sndtyp_wd(task->ttyp)) {
		printk(KERN_ERR
		       "mbx: BKSNDP from word sending task! (task%d)\n", tid);
		return;
	}
	if (sndtyp_gbl(task->ttyp)) {
		printk(KERN_ERR
		       "mbx: BKSNDP from non-private sending task! (task%d)\n",
		       tid);
		return;
	}

	/*
	 * we should not have delayed block at this point
	 * because read() routine releases the lock of the buffer and
	 * until then DSP can't send next data.
	 */

	if (sync_with_dsp(&ipbp->s, tid, 10) < 0) {
		printk(KERN_ERR "mbx: BKSNDP - IPBUF sync failed!\n");
		return;
	}
	printk(KERN_DEBUG "mbx: ipbuf_pvt_r->a = 0x%08lx\n",
	       MKLONG(ipbp->ah, ipbp->al));
	ipblink_add_pvt(&rcvdt->link);
	wake_up_interruptible(&task->dev->read_wait_q);
}

void mbx1_bkreqp(struct mbcmd *mb)
{
	unsigned char tid = mb->cmd_l;
	struct dsptask *task = dsptask[tid];
	struct ipbuf_p *ipbp = task->ipbuf_pvt_w;

	if ((tid >= TASKDEV_MAX) || (task == NULL)) {
		printk(KERN_ERR "mbx: BKREQP with illegal tid! %d\n", tid);
		return;
	}
	if (rcvtyp_wd(task->ttyp)) {
		printk(KERN_ERR
		       "mbx: BKREQP from word receiving task! (task%d)\n", tid);
		return;
	}
	if (rcvtyp_gbl(task->ttyp)) {
		printk(KERN_ERR
		       "mbx: BKREQP from non-private receiving task! (task%d)\n", tid);
		return;
	}
	if (rcvtyp_psv(task->ttyp)) {
		printk(KERN_ERR
		       "mbx: BKREQP from passive receiving task! (task%d)\n", tid);
		return;
	}

	if (sync_with_dsp(&ipbp->s, OMAP_DSP_TID_FREE, 10) < 0) {
		printk(KERN_ERR "mbx: BKREQP - IPBUF sync failed!\n");
		return;
	}
	printk(KERN_DEBUG "mbx: ipbuf_pvt_w->a = 0x%08lx\n",
	       MKLONG(ipbp->ah, ipbp->al));
	task->wsz = ipbp->c*2;
	wake_up_interruptible(&task->dev->write_wait_q);
}

void mbx1_tctl(struct mbcmd *mb)
{
	unsigned char tid = mb->cmd_l;
	struct dsptask *task = dsptask[tid];

	if ((tid >= TASKDEV_MAX) || (task == NULL)) {
		printk(KERN_ERR "mbx: TCTL with illegal tid! %d\n", tid);
		return;
	}

	if (!waitqueue_active(&task->dev->ioctl_wait_q)) {
		printk(KERN_WARNING "mbx: unexpected TCTL from DSP!\n");
		return;
	}

	task->tctl_stat = mb->data;
	wake_up_interruptible(&task->dev->ioctl_wait_q);
}

void mbx1_tcfg(struct mbcmd *mb)
{
	unsigned char tid = mb->cmd_l;
	struct dsptask *task = dsptask[tid];
	unsigned long tmp_ipbp_r, tmp_ipbp_w;
	unsigned long tmp_mapstart, tmp_maplen;
	unsigned long tmp_tnm;
	unsigned short *tnm;
	volatile unsigned short *buf;
	int i;

	if ((tid >= TASKDEV_MAX) || (task == NULL)) {
		printk(KERN_ERR "mbx: TCFG with illegal tid! %d\n", tid);
		return;
	}
	if ((task->state != TASK_STATE_CFGREQ) || (cfg_cmd != MBCMD(TCFG))) {
		printk(KERN_WARNING "mbx: unexpected TCFG from DSP!\n");
		return;
	}

	if (sync_with_dsp(&ipbuf_sys_da->s, tid, 10) < 0) {
		printk(KERN_ERR "mbx: TCFG - IPBUF sync failed!\n");
		return;
	}

	/*
	 * read configuration data on system IPBUF
	 */
	buf = ipbuf_sys_da->d;
	task->ttyp   = buf[0];
	tmp_ipbp_r   = MKLONG(buf[1], buf[2]);
	tmp_ipbp_w   = MKLONG(buf[3], buf[4]);
	tmp_mapstart = MKLONG(buf[5], buf[6]);
	tmp_maplen   = MKLONG(buf[7], buf[8]);
	tmp_tnm      = MKLONG(buf[9], buf[10]);

	task->rcvdt.bk.ipbuf_pvt_r = dspword_to_virt(tmp_ipbp_r);
	task->ipbuf_pvt_w          = dspword_to_virt(tmp_ipbp_w);
	task->map_base   = dspword_to_virt(tmp_mapstart);
	task->map_length = tmp_maplen << 1;	/* word -> byte */
	tnm = dspword_to_virt(tmp_tnm);
	for (i = 0; i < OMAP_DSP_TNM_LEN-1; i++) {
		/* avoiding byte access */
		unsigned short tmp = tnm[i];
		task->name[i] = tmp & 0x00ff;
		if (!tmp)
			break;
	}
	task->name[OMAP_DSP_TNM_LEN-1] = '\0';

	release_ipbuf_pvt(ipbuf_sys_da);
	task->state = TASK_STATE_READY;
	wake_up_interruptible(&cfg_wait_q);
}

void mbx1_tadd(struct mbcmd *mb)
{
	unsigned char tid = mb->cmd_l;

	if ((!waitqueue_active(&cfg_wait_q)) || (cfg_cmd != MBCMD(TADD))) {
		printk(KERN_WARNING "mbx: unexpected TADD from DSP!\n");
		return;
	}
	cfg_tid = tid;
	wake_up_interruptible(&cfg_wait_q);
}

void mbx1_tdel(struct mbcmd *mb)
{
	unsigned char tid = mb->cmd_l;

	if ((!waitqueue_active(&cfg_wait_q)) || (cfg_cmd != MBCMD(TDEL))) {
		printk(KERN_WARNING "mbx: unexpected TDEL from DSP!\n");
		return;
	}
	cfg_tid = tid;
	wake_up_interruptible(&cfg_wait_q);
}

void mbx1_err_fatal(unsigned char tid)
{
	struct dsptask *task = dsptask[tid];
	struct list_head *ptr;
	struct proc_list *pl;
	siginfo_t info;

	if ((tid >= TASKDEV_MAX) || (task == NULL)) {
		printk(KERN_ERR "mbx: FATAL ERR with illegal tid! %d\n", tid);
		return;
	}

	info.si_signo = SIGBUS;
	info.si_errno = 0;
	info.si_code = SI_KERNEL;
	info._sifields._sigfault._addr = NULL;
	list_for_each(ptr, &task->dev->proc_list) {
		pl = list_entry(ptr, struct proc_list, list_head);
		send_sig_info(SIGBUS, &info, pl->tsk);
	}
}

void mbx1_dbg(struct mbcmd *mb)
{
	unsigned char tid = mb->cmd_l;
	char s[80], *s_end = &s[79], *p;
	unsigned short *src;
	volatile unsigned short *buf;
	int cnt;
	int i;

	if (((tid >= TASKDEV_MAX) || (dsptask[tid] == NULL)) &&
	    (tid != OMAP_DSP_TID_ANON)) {
		printk(KERN_ERR "mbx: DBG with illegal tid! %d\n", tid);
		return;
	}
	if (sync_with_dsp(&ipbuf_sys_da->s, tid, 10) < 0) {
		printk(KERN_ERR "mbx: DBG - IPBUF sync failed!\n");
		return;
	}
	buf = ipbuf_sys_da->d;
	cnt = buf[0];
	src = dspword_to_virt(MKLONG(buf[1], buf[2]));
	p = s;
	for (i = 0; i < cnt; i++) {
		unsigned short tmp;
		/*
		 * Be carefull that ipbuf should not be read with
		 * 1-byte access since it might be placed in DARAM/SARAM
		 * and it can cause unexpected byteswap.
		 * For example,
		 *   *(p++) = *(src++) & 0xff;
		 * causes 1-byte access!
		 */
		tmp = *src++;
		*(p++) = tmp & 0xff;
		if (*(p-1) == '\n') {
			*p = '\0';
			printk(KERN_INFO "%s", s);
			p = s;
			continue;
		}
		if (p == s_end) {
			*p = '\0';
			printk(KERN_INFO "%s\n", s);
			p = s;
			continue;
		}
	}
	if (p > s) {
		*p = '\0';
		printk(KERN_INFO "%s\n", s);
	}

	release_ipbuf_pvt(ipbuf_sys_da);
}


#ifdef CONFIG_PROC_FS
/*
 * proc file entry
 */

#define devstate_name(stat) (\
	((stat) == OMAP_DSP_DEVSTATE_NOTASK)   ? "NOTASK" :\
	((stat) == OMAP_DSP_DEVSTATE_ATTACHED) ? "ATTACHED" :\
	((stat) == OMAP_DSP_DEVSTATE_GARBAGE)  ? "GARBAGE" :\
	((stat) == OMAP_DSP_DEVSTATE_ADDREQ)   ? "ADDREQ" :\
	((stat) == OMAP_DSP_DEVSTATE_DELREQ)   ? "DELREQ" :\
	((stat) == OMAP_DSP_DEVSTATE_KILLREQ)  ? "KILLREQ" :\
	((stat) == OMAP_DSP_DEVSTATE_ADDFAIL)  ? "ADDFAIL" :\
						 "unknown")

#define taskstate_name(state) (\
	((state) == TASK_STATE_ERR)    ? "ERR" :\
	((state) == TASK_STATE_CFGREQ) ? "CFGREQ" :\
	((state) == TASK_STATE_READY)  ? "READY" :\
					 "unknown")

static __inline__ char *bid_name(unsigned short bid)
{
	static char s[6];

	switch (bid) {
	case OMAP_DSP_BID_NULL:
		return "NULL";
	case OMAP_DSP_BID_PVT:
		return "PRIVATE";
	default:
		sprintf(s, "%d", bid);
		return s;
	}
}

static __inline__ char *sndtyp_name(unsigned short ttyp)
{
	static char s[32];

	sprintf(s, "%s %s send",
		(sndtyp_acv(ttyp)) ? "active" :
				     "passive",
		(sndtyp_wd(ttyp))  ? "word" :
		(sndtyp_pvt(ttyp)) ? "private block" :
				     "global block");
	return s;
}

static __inline__ char *rcvtyp_name(unsigned short ttyp)
{
	static char s[32];

	sprintf(s, "%s %s receive",
		(rcvtyp_acv(ttyp)) ? "active" :
				     "passive",
		(rcvtyp_wd(ttyp))  ? "word" :
		(rcvtyp_pvt(ttyp)) ? "private block" :
				     "global block");
	return s;
}

static int taskdev_stat_read_proc(char *page, char **start, off_t off,
				  int count, int *eof, void *data)
{
	struct taskdev *dev;
	struct dsptask *task;
	struct list_head *ptr;
	struct proc_list *pl;
	char *out;
	int len;

	preempt_disable();

	dev = (struct taskdev *)data;
	task = dev->task;

	out = page;
	out += sprintf(out,
		       "device name: %s\n"
		       "state        %s\n",
		       dev->name, devstate_name(dev->state));

	if (task) {
		unsigned short ttyp = task->ttyp;

		out += sprintf(out,
			       "  attached task: %s\n"
			       "    stat    %s\n"
			       "    tid     %d\n"
			       "    ttyp    0x%04x (%s, %s)\n",
			       task->name,
			       taskstate_name(task->state),
			       task->tid,
			       ttyp, sndtyp_name(ttyp), rcvtyp_name(ttyp));

		if (sndtyp_wd(ttyp)) {
			struct fifo_struct *fifo = &task->rcvdt.fifo;
			out += sprintf(out,
				       "    fifo->buf     0x%p\n"
				       "    fifo->sz      %d\n"
				       "    fifo->cnt     %d\n"
				       "    fifo->wp      %d\n",
				       fifo->buf, fifo->sz,
				       fifo->cnt, fifo->wp);
		} else {	/* sndtyp_bk */
			struct rcvdt_bk_struct *rcvdt = &task->rcvdt.bk;
			out += sprintf(out,
				       "    delayed list top   %s\n"
				       "                 tail  %s\n",
				       bid_name(rcvdt->link.top),
				       bid_name(rcvdt->link.tail));
			if (sndtyp_pvt(ttyp)) {
				out += sprintf(out,
					       "    private IPBUF for read   0x%p\n",
					       rcvdt->ipbuf_pvt_r);
			}
		}

		out += sprintf(out, "    wsz        %d\n", task->wsz);
		if (rcvtyp_pvt(ttyp)) {
			out += sprintf(out,
				       "    private IPBUF for write  0x%p\n",
				       task->ipbuf_pvt_w);
		}

		if (task->map_length) {
			out += sprintf(out,
				       "    map_base   %p\n"
				       "    map_length %d\n",
				       task->map_base, task->map_length);
		}

		out += sprintf(out, "    user pid ");
		list_for_each(ptr, &dev->proc_list) {
			pl = list_entry(ptr, struct proc_list, list_head);
			out += sprintf(out, "  %d", pl->tsk->pid);
		}
		out += sprintf(out, "\n");
	}

	preempt_enable();

	len = out - page - off;
	if (len < count) {
		*eof = 1;
		if (len <= 0)
			return 0;
	} else
		len = count;
	*start = page + off;

	return len;
}

static int taskdev_fifosz_read_proc(char *page, char **start, off_t off,
				    int count, int *eof, void *data)
{
	struct taskdev *dev;
	struct dsptask *task;
	size_t fifosz;
	int len;

	dev = (struct taskdev *)data;
	task = dev->task;
	fifosz = (task && sndtyp_wd(task->ttyp)) ? task->rcvdt.fifo.sz : 0;
	len = sprintf(page, "%d\n", fifosz);

	return len;
}

static int taskdev_fifosz_write_proc(struct file *file, const char *buffer,
				     unsigned long count, void *data)
{
	struct taskdev *dev;
	struct dsptask *task;
	int len;
	char tmp[16];
	unsigned long fifosz;
	int ret;

	dev = (struct taskdev *)data;
	task = dev->task;

	if (task == NULL) {
		printk(KERN_ERR "task not attached.\n");
		return -EINVAL;
	}

	len = (count > 15) ? 15 : count;
	if (copy_from_user(tmp, buffer, len))
		return -EFAULT;
	tmp[len] = '\0';

	fifosz = simple_strtol(tmp, NULL, 10);
	if ((ret = dsp_task_set_fifosz(task, fifosz)) < 0)
		return ret;

	return len;
}

/*
 * called from ipbuf_read_proc()
 */
int ipbuf_is_held(unsigned char tid, unsigned short bid)
{
	struct dsptask *task = dsptask[tid];
	unsigned short b;
	int ret = 0;

	if (task == NULL)
		return 0;

	disable_irq(INT_D2A_MB1);
	ipblink_for_each(b, &task->rcvdt.bk.link, ipbuf) {
		if (b == bid) {	/* found */
			ret = 1;
			break;
		}
	}
	enable_irq(INT_D2A_MB1);

	return ret;
}

static void taskdev_create_proc(struct taskdev *dev)
{
	struct proc_dir_entry *ent;

	if (procdir_dsp_task == NULL)
		return;

	dev->procent_dir = proc_mkdir(dev->name, procdir_dsp_task);
	if (dev->procent_dir == NULL) {
		printk(KERN_ERR
		       "omapdsp: failed to register proc directory: %s\n",
		       dev->name);
		return;
	}

	ent = create_proc_read_entry("status", 0, dev->procent_dir,
				     taskdev_stat_read_proc, dev);
	if (ent == NULL) {
		printk(KERN_ERR
		       "omapdsp: failed to register "
		       "status proc device for: %s\n", dev->name);
		return;
	}

	ent = create_proc_entry("fifosz", S_IFREG | S_IWUGO | S_IRUGO,
				dev->procent_dir);
	if (ent == NULL) {
		printk(KERN_ERR
		       "omapdsp: failed to register fifosz "
		       "proc device for: %s\n", dev->name);
		return;
	}
	ent->read_proc  = taskdev_fifosz_read_proc;
	ent->write_proc = taskdev_fifosz_write_proc;
	ent->data = dev;
}

static void taskdev_remove_proc(struct taskdev *dev)
{
	if (dev->procent_dir == NULL)
		return;

	remove_proc_entry("status", dev->procent_dir);
	remove_proc_entry("fifosz", dev->procent_dir);
	remove_proc_entry(dev->name, procdir_dsp_task);
	dev->procent_dir = NULL;
}

static void __init taskdev_create_procdir(void)
{
	procdir_dsp_task = proc_mkdir("task", procdir_dsp);
	if (procdir_dsp_task == NULL) {
		printk(KERN_ERR
		       "omapdsp: failed to register proc device: dsp/task\n");
	}
}

static void taskdev_remove_procdir(void)
{
	if (procdir_dsp_task == NULL)
		return;

	remove_proc_entry("task", procdir_dsp);
	procdir_dsp_task = NULL;
}

#endif /* CONFIG_PROC_FS */

int __init dsp_taskmod_init(void)
{
	int retval;

	memset(taskdev, 0, sizeof(void *) * TASKDEV_MAX);
	memset(dsptask, 0, sizeof(void *) * TASKDEV_MAX);

	retval = register_chrdev(OMAP_DSP_TASK_MAJOR, "dsptask",
				 &dsp_task_fops);
	if (retval < 0) {
		printk(KERN_ERR
		       "omapdsp: failed to register task device: %d\n", retval);
		return retval;
	}
	devfs_mk_dir("dsptask");
#ifdef CONFIG_PROC_FS
	taskdev_create_procdir();
#endif

	return 0;
}

void dsp_taskmod_exit(void)
{
	devfs_remove("dsptask");
	unregister_chrdev(OMAP_DSP_TASK_MAJOR, "dsptask");
#ifdef CONFIG_PROC_FS
	taskdev_remove_procdir();
#endif
}
