/*
 * linux/arch/arm/mach-omap/dsp/mblog.c
 *
 * OMAP DSP driver Mailbox log module
 *
 * Copyright (C) 2003,2004 Nokia Corporation
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
 * 2004/11/18:  DSP Gateway version 3.1
 */

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/init.h>
#include <asm/irq.h>
#include <asm/arch/dsp.h>
#include "dsp.h"

#define RLCMD(nm)	OMAP_DSP_MBCMD_RUNLEVEL_##nm
#define PMCMD(nm)	OMAP_DSP_MBCMD_PM_##nm
#define CFGCMD(nm)	OMAP_DSP_MBCMD_DSPCFG_##nm
#define REGCMD(nm)	OMAP_DSP_MBCMD_REGRW_##nm
#define VICMD(nm)	OMAP_DSP_MBCMD_VARID_##nm
#define EID(nm)		OMAP_DSP_EID_##nm

char *subcmd_name(struct mbcmd *mb)
{
	unsigned char cmd_h = mb->cmd_h;
	unsigned char cmd_l = mb->cmd_l;
	char *s;

	switch (cmd_h) {
	case MBCMD(RUNLEVEL):
		s = (cmd_l == RLCMD(USER))     ? "USER":
		    (cmd_l == RLCMD(SUPER))    ? "SUPER":
		    (cmd_l == RLCMD(RECOVERY)) ? "RECOVERY":
		    NULL;
		break;
	case MBCMD(PM):
		s = (cmd_l == PMCMD(DISABLE)) ? "DISABLE":
		    (cmd_l == PMCMD(ENABLE))  ? "ENABLE":
		    NULL;
		break;
	case MBCMD(DSPCFG):
		{
			unsigned char cfgc = cmd_l & 0x7f;
			s = (cfgc == CFGCMD(REQ))     ? "REQ":
			    (cfgc == CFGCMD(SYSADRH)) ? "SYSADRH":
			    (cfgc == CFGCMD(SYSADRL)) ? "SYSADRL":
			    (cfgc == CFGCMD(ABORT))   ? "ABORT":
			    (cfgc == CFGCMD(PROTREV)) ? "PROTREV":
			    NULL;
			break;
		}
	case MBCMD(REGRW):
		s = (cmd_l == REGCMD(MEMR)) ? "MEMR":
		    (cmd_l == REGCMD(MEMW)) ? "MEMW":
		    (cmd_l == REGCMD(IOR))  ? "IOR":
		    (cmd_l == REGCMD(IOW))  ? "IOW":
		    (cmd_l == REGCMD(DATA)) ? "DATA":
		    NULL;
		break;
	case MBCMD(GETVAR):
	case MBCMD(SETVAR):
		s = (cmd_l == VICMD(ICRMASK))  ? "ICRMASK":
		    (cmd_l == VICMD(LOADINFO)) ? "LOADINFO":
		    NULL;
		break;
	case MBCMD(ERR):
		s = (cmd_l == EID(BADTID))     ? "BADTID":
		    (cmd_l == EID(BADTCN))     ? "BADTCN":
		    (cmd_l == EID(BADBID))     ? "BADBID":
		    (cmd_l == EID(BADCNT))     ? "BADCNT":
		    (cmd_l == EID(NOTLOCKED))  ? "NOTLOCKED":
		    (cmd_l == EID(STVBUF))     ? "STVBUF":
		    (cmd_l == EID(BADADR))     ? "BADADR":
		    (cmd_l == EID(BADTCTL))    ? "BADTCTL":
		    (cmd_l == EID(BADPARAM))   ? "BADPARAM":
		    (cmd_l == EID(FATAL))      ? "FATAL":
		    (cmd_l == EID(NOMEM))      ? "NOMEM":
		    (cmd_l == EID(NORES))      ? "NORES":
		    (cmd_l == EID(IPBFULL))    ? "IPBFULL":
		    (cmd_l == EID(TASKNOTRDY)) ? "TASKNOTRDY":
		    (cmd_l == EID(TASKBSY))    ? "TASKBSY":
		    (cmd_l == EID(TASKERR))    ? "TASKERR":
		    (cmd_l == EID(BADCFGTYP))  ? "BADCFGTYP":
		    (cmd_l == EID(DEBUG))      ? "DEBUG":
		    (cmd_l == EID(BADSEQ))     ? "BADSEQ":
		    (cmd_l == EID(BADCMD))     ? "BADCMD":
		    NULL;
		break;
	default:
		s = NULL;
	}

	return s;
}

#define MBLOG_DEPTH	256

struct mblogent {
	unsigned long jiffies;
	unsigned short cmd;
	unsigned short data;
	enum mblog_dir dir;
};

static struct {
	int wp;
	unsigned long cnt, cnt_ad, cnt_da;
	struct mblogent ent[MBLOG_DEPTH];
} mblog;

void mblog_add(struct mbcmd *mb, enum mblog_dir dir)
{
	struct mbcmd_hw *mb_hw = (struct mbcmd_hw *)mb;
	struct mblogent *ent;

	disable_irq(INT_D2A_MB1);
	ent = &mblog.ent[mblog.wp];
	ent->jiffies = jiffies;
	ent->cmd     = mb_hw->cmd;
	ent->data    = mb_hw->data;
	ent->dir     = dir;
	if (mblog.cnt < 0xffffffff)
		mblog.cnt++;
	switch (dir) {
	case MBLOG_DIR_AD:
		if (mblog.cnt_ad < 0xffffffff)
			mblog.cnt_ad++;
		break;
	case MBLOG_DIR_DA:
		if (mblog.cnt_da < 0xffffffff)
			mblog.cnt_da++;
		break;
	}
	if (++mblog.wp == MBLOG_DEPTH)
		mblog.wp = 0;
	enable_irq(INT_D2A_MB1);
}

#ifdef CONFIG_PROC_FS
static int mblog_read_proc(char *page, char **start, off_t off, int count,
			   int *eof, void *data)
{
	int len = 0;
	off_t begin = 0;
	int wp;
	int i;

	disable_irq(INT_D2A_MB1);

	wp = mblog.wp;
	len += sprintf(page,
		       "log count:%ld / ARM->DSP:%ld, DSP->ARM:%ld\n",
		       mblog.cnt, mblog.cnt_ad, mblog.cnt_da);
	if (mblog.cnt == 0)
		goto done;

	len += sprintf(page + len, "         ARM->DSP  ARM<-DSP\n");
	len += sprintf(page + len, "jiffies  cmd  data cmd  data\n");
	i = (mblog.cnt >= MBLOG_DEPTH) ? wp : 0;
	do {
		struct mblogent *ent = &mblog.ent[i];
		union {
			struct mbcmd sw;
			struct mbcmd_hw hw;
		} mb = {
			.hw.cmd  = ent->cmd,
			.hw.data = ent->data
		};
		char *subname;
		const struct cmdinfo *ci = cmdinfo[mb.sw.cmd_h];

		len += sprintf(page + len,
			       (ent->dir == MBLOG_DIR_AD) ?
				"%08lx %04x %04x           seq=%d":
				"%08lx           %04x %04x seq=%d",
			       ent->jiffies, ent->cmd, ent->data, mb.sw.seq);
		switch (ci->cmd_l_type) {
		case CMD_L_TYPE_SUBCMD:
			if ((subname = subcmd_name(&mb.sw)) == NULL)
				subname = "Unknown";
			len += sprintf(page + len, " %s:%s\n",
				       ci->name, subname);
			break;
		case CMD_L_TYPE_TID:
			len += sprintf(page + len, " %s:task %d\n",
				       ci->name, mb.sw.cmd_l);
			break;
		case CMD_L_TYPE_NULL:
			len += sprintf(page + len, " %s\n", ci->name);
			break;
		}

		if (len + begin > off + count)
			break;
		if (len + begin < off) {
			begin += len;
			len = 0;
		}

		if (++i == MBLOG_DEPTH)
			i = 0;
	} while (i != wp);

done:
	enable_irq(INT_D2A_MB1);
	len -= off - begin;
	if (len < count) {
		*eof = 1;
		if (len <= 0)
			return 0;
	} else
		len = count;
	*start = page + (off - begin);

	return len;
}

static void __init mblog_create_proc(void)
{
	struct proc_dir_entry *ent;

	ent = create_proc_read_entry("mblog", 0, procdir_dsp, mblog_read_proc,
				     NULL);
	if (ent == NULL) {
		printk(KERN_ERR
		       "omapdsp: failed to register proc device: mblog\n");
	}
}

static void mblog_remove_proc(void)
{
	remove_proc_entry("mblog", procdir_dsp);
}
#endif /* CONFIG_PROC_FS */

#ifdef CONFIG_OMAP_DSP_MBCMD_VERBOSE
void mblog_printcmd(struct mbcmd *mb, enum mblog_dir dir)
{
	const struct cmdinfo *ci = cmdinfo[mb->cmd_h];
	char *dir_str;
	char *subname;

	dir_str = (dir == MBLOG_DIR_AD) ? "sending" : "receiving";
	switch (ci->cmd_l_type) {
	case CMD_L_TYPE_SUBCMD:
		if ((subname = subcmd_name(mb)) == NULL)
			subname = "Unknown";
		printk(KERN_DEBUG
		       "mbx: %s seq=%d, cmd=%02x:%02x(%s:%s), data=%04x\n",
		       dir_str, mb->seq, mb->cmd_h, mb->cmd_l,
		       ci->name, subname, mb->data);
		break;
	case CMD_L_TYPE_TID:
		printk(KERN_DEBUG
		       "mbx: %s seq=%d, cmd=%02x:%02x(%s:task %d), data=%04x\n",
		       dir_str, mb->seq, mb->cmd_h, mb->cmd_l,
		       ci->name, mb->cmd_l, mb->data);
		break;
	case CMD_L_TYPE_NULL:
		printk(KERN_DEBUG
		       "mbx: %s seq=%d, cmd=%02x:%02x(%s), data=%04x\n",
		       dir_str, mb->seq, mb->cmd_h, mb->cmd_l,
		       ci->name, mb->data);
		break;
	}
}
#endif /* CONFIG_OMAP_DSP_MBCMD_VERBOSE */

void __init mblog_init(void)
{
#ifdef CONFIG_PROC_FS
	mblog_create_proc();
#endif
}

void mblog_exit(void)
{
#ifdef CONFIG_PROC_FS
	mblog_remove_proc();
#endif
}
