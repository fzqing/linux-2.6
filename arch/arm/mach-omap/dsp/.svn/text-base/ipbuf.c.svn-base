/*
 * linux/arch/arm/mach-omap/dsp/ipbuf.c
 *
 * IPBUF handler
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
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <asm/signal.h>
#include <asm/arch/dsp.h>
#include "dsp.h"
#include "ipbuf.h"

struct ipbuf **ipbuf;
struct ipbcfg ipbcfg;
struct ipbuf_sys *ipbuf_sys_da, *ipbuf_sys_ad;
static struct ipblink ipb_free;

void ipbuf_stop(void)
{
	ipbcfg.ln = 0;
	if (ipbuf) {
		kfree(ipbuf);
		ipbuf = NULL;
	}
}

/*
 * ipbuf_config() is called in interrupt context
 */
int ipbuf_config(unsigned short ln, unsigned short lsz, unsigned long adr)
{
	void *base;
	unsigned long lsz_byte = ((unsigned long)lsz) << 1;
	size_t size;
	int ret = 0;
	int i;

	INIT_IPBLINK(&ipb_free);

	/*
	 * global IPBUF
	 */
	if (adr & 0x1) {
		printk(KERN_ERR
		       "mbx: global ipbuf address (0x%08lx) is odd number!\n",
		       adr);
		return -EINVAL;
	}
	size = lsz_byte * ln;
	if (adr + size > DSPSPACE_SIZE) {
		printk(KERN_ERR
		       "mbx: ipbuf address (0x%08lx) and size (0x%08x) is "
		       "illegal!\n", adr, size);
		return -EINVAL;
	}
	base = dspword_to_virt(adr);
	ipbuf = kmalloc(sizeof(void *) * ln, GFP_KERNEL);
	if (ipbuf == NULL) {
		printk(KERN_ERR "mbx: memory allocation for ipbuf failed.\n");
		return -ENOMEM;
	}
	for (i = 0; i < ln; i++) {
		void *top, *btm;

		top = base + (sizeof(struct ipbuf) + lsz_byte) * i;
		btm = base + (sizeof(struct ipbuf) + lsz_byte) * (i+1) - 1;
		ipbuf[i] = (struct ipbuf *)top;
		if (((unsigned long)top & 0xfffe0000) !=
		    ((unsigned long)btm & 0xfffe0000)) {
			/*
			 * an ipbuf line should not cross
			 * 64k-word boundary.
			 */
			printk(KERN_ERR
			       "omapdsp: ipbuf[%d] crosses 64k-word boundary!\n"
			       "  @0x%p, size=0x%08lx\n", i, top, lsz_byte);
			ret = -EINVAL;
		}
	}
	ipbcfg.ln       = ln;
	ipbcfg.lsz      = lsz;
	ipbcfg.adr      = adr;
	ipbcfg.bsycnt   = ln;	/* DSP holds all ipbufs initially. */
	ipbcfg.cnt_full = 0;

	printk(KERN_INFO
	       "omapdsp: IPBUF configuration\n"
	       "           %d words * %d lines at 0x%p.\n",
	       ipbcfg.lsz, ipbcfg.ln, dspword_to_virt(ipbcfg.adr));

	return ret;
}

int is_ipbuf_internal_mem(void)
{
	return is_dspword_internal_mem(ipbcfg.adr);
}

/*
 * Global IPBUF operations
 */
unsigned short get_free_ipbuf(unsigned char tid)
{
	unsigned short bid;

	if (ipblink_empty(&ipb_free)) {
		/*
		 * FIXME: wait on queue when not available,
		 * but note that balance_ipbuf() also uses this routine
		 * and in that case we should return immediately.
		 */
		return OMAP_DSP_BID_NULL;
	}

	/*
	 * FIXME: dsp_enable_dspmem!
	 */
	preempt_disable();
	disable_irq(INT_D2A_MB1);
	bid = ipb_free.top;
	ipbuf[bid]->la = tid;	/* lock */
	ipblink_del_top(&ipb_free, ipbuf);
	enable_irq(INT_D2A_MB1);
	preempt_enable();

	return bid;
}

void release_ipbuf(unsigned short bid)
{
	if (ipbuf[bid]->la == OMAP_DSP_TID_FREE) {
		printk(KERN_WARNING
		       "omapdsp: attempt to release unlocked IPBUF[%d].\n",
		       bid);
		/*
		 * FIXME: re-calc bsycnt
		 */
		return;
	}
	ipbuf[bid]->la = OMAP_DSP_TID_FREE;
	ipbuf[bid]->sa = OMAP_DSP_TID_FREE;
	preempt_disable();
	disable_irq(INT_D2A_MB1);
	ipblink_add_tail(&ipb_free, bid, ipbuf);
	enable_irq(INT_D2A_MB1);
	preempt_enable();
}

static int try_yld(unsigned short bid, enum mbsend_type send_type)
{
	struct mbcmd mb;
	int status;

	ipbuf[bid]->sa = OMAP_DSP_TID_ANON;
	mbcmd_set(mb, MBCMD(BKYLD), 0, bid);
	status = __dsp_mbsend(&mb, NULL, send_type);
	if (status < 0) {
		/* DSP is busy and ARM keeps this line. */
		release_ipbuf(bid);
		return status;
	}

	ipb_bsycnt_inc(&ipbcfg);
	return 0;
}

/*
 * balancing ipbuf lines with DSP
 */
void balance_ipbuf(enum mbsend_type send_type)
{
	while (ipbcfg.bsycnt <= ipbcfg.ln / 4) {
		unsigned short bid;

		bid = get_free_ipbuf(OMAP_DSP_TID_ANON);
		if (bid == OMAP_DSP_BID_NULL)
			return;
		if (try_yld(bid, send_type) < 0)
			return;
	}
}

/*
 * interrupt routine should call this function with
 * send_type = MBSEND_TYPE_NOWAIT
 */
void __unuse_ipbuf(unsigned short bid, enum mbsend_type send_type)
{
	if (ipbcfg.bsycnt > ipbcfg.ln / 4) {
		release_ipbuf(bid);
		return;
	}

	/* try to return this line */
	ipbuf[bid]->la = OMAP_DSP_TID_ANON;
	if (try_yld(bid, send_type) < 0)
		return;

	if (send_type != MBSEND_TYPE_NOWAIT)
		balance_ipbuf(send_type);
}

/*
 * functions called from mailbox1 interrupt routine
 */

void mbx1_err_ipbfull(void)
{
	ipbcfg.cnt_full++;
}

#ifdef CONFIG_PROC_FS
/*
 * proc file entry
 */

static int ipbuf_read_proc(char *page, char **start, off_t off, int count,
			   int *eof, void *data)
{
	char *out;
	int len;
	unsigned short bid;

	/*
	 * disable process's old pmd entry
	 */
	dsp_map_update(current);
	dsp_cur_users_add(current);

	out = page;
	for (bid = 0; bid < ipbcfg.ln; bid++) {
		unsigned short la = ipbuf[bid]->la;
		unsigned short ld = ipbuf[bid]->ld;
		unsigned short c  = ipbuf[bid]->c;

		out += sprintf(out, "ipbuf[%d]: adr = 0x%p\n", bid, ipbuf[bid]);
		if (la == OMAP_DSP_TID_FREE) {
			out += sprintf(out,
				       "  DSPtask[%d]->Linux "
				       "(already read and now free for Linux)\n",
				       ld);
		} else if (ld == OMAP_DSP_TID_FREE) {
			out += sprintf(out,
				       "  Linux->DSPtask[%d] "
				       "(already read and now free for DSP)\n",
				       la);
		} else if (ipbuf_is_held(ld, bid)) {
			out += sprintf(out,
				       "  DSPtask[%d]->Linux "
				       "(waiting to be read)\n"
				       "  count = %d\n", ld, c);
		} else {
			out += sprintf(out,
				       "  Linux->DSPtask[%d] "
				       "(waiting to be read)\n"
				       "  count = %d\n", la, c);
		}
	}

	out += sprintf(out, "\nFree IPBUF link: ");
	preempt_disable();
	ipblink_for_each(bid, &ipb_free, ipbuf) {
		out += sprintf(out, "%d ", bid);
	}
	preempt_enable();
	out += sprintf(out, "\n");
	out += sprintf(out, "IPBFULL error count: %ld\n", ipbcfg.cnt_full);

	len = out - page - off;
	if (len < count) {
		*eof = 1;
		if (len <= 0) {
			len = 0;
			goto finish;
		}
	} else
		len = count;
	*start = page + off;

finish:
	dsp_cur_users_del(current);
	return len;
}

void dsp_create_ipbuf_proc(void)
{
	struct proc_dir_entry *ent;

	ent = create_proc_read_entry("ipbuf", 0, procdir_dsp, ipbuf_read_proc,
				     NULL);
	if (ent == NULL) {
		printk(KERN_ERR
		       "omapdsp: failed to register proc device: ipbuf\n");
	}
}

void dsp_remove_ipbuf_proc(void)
{
	remove_proc_entry("ipbuf", procdir_dsp);
}
#endif /* CONFIG_PROC_FS */
