/*
 * arch/mips/tx4939/common/proc.c
 *
 * Setup proc-fs for tx4939
 *
 * (C) Copyright TOSHIBA CORPORATION SEMICONDUCTOR COMPANY 2005
 *
 * 2001-2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Support for TX4939 in 2.6 - Hiroshi DOYU <Hiroshi_DOYU@montavista.co.jp>
 */

#include <linux/config.h>
#include <linux/module.h>	/* Definitions needed for kernel modules */
#include <linux/kmod.h>
#include <linux/init.h>
#include <linux/kernel.h>	/* We run in the kernel so we need this */
#include <linux/proc_fs.h>	/* The /proc definitions are in this one */

#include <asm/io.h>
#include <asm/mipsregs.h>
#include <asm/tx4939/tx4939.h>


struct proc_dir_entry *tx4939_proc_entry;

static int
tx4939_proc_show_cp0(char *sysbuf, char **start, off_t off,
		     int count, int *eof, void *data)
{
	int len = 0;
	len += sprintf(sysbuf + len, "INDEX   :0x%08x\n", read_c0_index());
	len += sprintf(sysbuf + len, "ENTRYLO0:0x%08lx\n", read_c0_entrylo0());
	len += sprintf(sysbuf + len, "ENTRYLO1:0x%08lx\n", read_c0_entrylo1());
	len += sprintf(sysbuf + len, "CONTEXT :0x%08lx\n", read_c0_context());
	len += sprintf(sysbuf + len, "PAGEMASK:0x%08x\n", read_c0_pagemask());
	len += sprintf(sysbuf + len, "WIRED   :0x%08x\n", read_c0_wired());
	len += sprintf(sysbuf + len, "COUNT   :0x%08x\n", read_c0_count());
	len += sprintf(sysbuf + len, "ENTRYHI :0x%08lx\n", read_c0_entryhi());
	len += sprintf(sysbuf + len, "COMPARE :0x%08x\n", read_c0_compare());
	len += sprintf(sysbuf + len, "STATUS  :0x%08x\n", read_c0_status());
	len += sprintf(sysbuf + len, "CAUSE   :0x%08x\n", read_c0_cause());
	len += sprintf(sysbuf + len, "PRId    :0x%08x\n", read_c0_prid());
	len += sprintf(sysbuf + len, "CONFIG  :0x%08x\n", read_c0_config());
	len += sprintf(sysbuf + len, "XCONTEXT:0x%08lx\n", read_c0_xcontext());
	len += sprintf(sysbuf + len, "TagLo   :0x%08x\n", read_c0_taglo());
	len += sprintf(sysbuf + len, "TagHi   :0x%08x\n", read_c0_taghi());
	len += sprintf(sysbuf + len, "ErrorEPC:0x%08lx\n", read_c0_errorepc());
	*eof = 1;
	return len;
}

static int
tx4939_proc_show_mips_io_port_base(char *sysbuf, char **start, off_t off,
				   int count, int *eof, void *data)
{
	int len = 0;
	len += sprintf(sysbuf + len, "mips_io_port_base = 0x%08lx\n",
		       mips_io_port_base);
	*eof = 1;
	return len;
}

static int
tx4939_proc_show_ccfg(char *sysbuf, char **start, off_t off,
		      int count, int *eof, void *data)
{
	int len = 0;
	len += sprintf(sysbuf + len, "***** tx4939_ccfg *****\n");
	len +=
	    sprintf(sysbuf + len, "tx4939_ccfgptr  ->ccfg        : 0x%016Lx\n",
		    reg_rd64s(&tx4939_ccfgptr->ccfg));
	len +=
	    sprintf(sysbuf + len, "tx4939_ccfgptr  ->revid       : 0x%016Lx\n",
		    reg_rd64s(&tx4939_ccfgptr->revid));
	len +=
	    sprintf(sysbuf + len, "tx4939_ccfgptr  ->pcfg        : 0x%016Lx\n",
		    reg_rd64s(&tx4939_ccfgptr->pcfg));
	len +=
	    sprintf(sysbuf + len, "tx4939_ccfgptr  ->toea        : 0x%016Lx\n",
		    reg_rd64s(&tx4939_ccfgptr->toea));
	len +=
	    sprintf(sysbuf + len, "tx4939_ccfgptr  ->clkctr      : 0x%016Lx\n",
		    reg_rd64s(&tx4939_ccfgptr->clkctr));
	len +=
	    sprintf(sysbuf + len, "tx4939_ccfgptr  ->garbc       : 0x%016Lx\n",
		    reg_rd64s(&tx4939_ccfgptr->garbc));
	len +=
	    sprintf(sysbuf + len, "tx4939_ccfgptr  ->ramp        : 0x%016Lx\n",
		    reg_rd64s(&tx4939_ccfgptr->ramp));
	len +=
	    sprintf(sysbuf + len, "tx4939_ccfgptr  ->dskwctrl    : 0x%016Lx\n",
		    reg_rd64s(&tx4939_ccfgptr->dskwctrl));
	len +=
	    sprintf(sysbuf + len, "tx4939_ccfgptr  ->mclkosc     : 0x%016Lx\n",
		    reg_rd64s(&tx4939_ccfgptr->mclkosc));
	len +=
	    sprintf(sysbuf + len, "tx4939_ccfgptr  ->mclkctl     : 0x%016Lx\n",
		    reg_rd64s(&tx4939_ccfgptr->mclkctl));
	len +=
	    sprintf(sysbuf + len, "tx4939_ccfgptr  ->gpiomr1     : 0x%016Lx\n",
		    reg_rd64s(&tx4939_ccfgptr->gpio[0].mr));
	len +=
	    sprintf(sysbuf + len, "tx4939_ccfgptr  ->gpiodr1     : 0x%016Lx\n",
		    reg_rd64s(&tx4939_ccfgptr->gpio[0].dr));
	len +=
	    sprintf(sysbuf + len, "tx4939_ccfgptr  ->gpiomr2     : 0x%016Lx\n",
		    reg_rd64s(&tx4939_ccfgptr->gpio[1].mr));
	len +=
	    sprintf(sysbuf + len, "tx4939_ccfgptr  ->gpiodr2     : 0x%016Lx\n",
		    reg_rd64s(&tx4939_ccfgptr->gpio[1].dr));
	*eof = 1;
	return len;
}

static int
tx4939_proc_show_ddrc(char *sysbuf, char **start, off_t off,
		      int count, int *eof, void *data)
{
	int len = 0;
	len += sprintf(sysbuf + len, "***** tx4939_ddrc *****\n");
	len +=
	    sprintf(sysbuf + len, "tx4939_ddrcptr  ->drwinen     : 0x%016Lx\n",
		    reg_rd64s(&tx4939_ddrcptr->drwinen));
	len +=
	    sprintf(sysbuf + len, "tx4939_ddrcptr  ->drwin[0]    : 0x%016Lx\n",
		    reg_rd64s(&tx4939_ddrcptr->drwin[0]));
	len +=
	    sprintf(sysbuf + len, "tx4939_ddrcptr  ->drwin[1]    : 0x%016Lx\n",
		    reg_rd64s(&tx4939_ddrcptr->drwin[1]));
	len +=
	    sprintf(sysbuf + len, "tx4939_ddrcptr  ->drwin[2]    : 0x%016Lx\n",
		    reg_rd64s(&tx4939_ddrcptr->drwin[2]));
	len +=
	    sprintf(sysbuf + len, "tx4939_ddrcptr  ->drwin[3]    : 0x%016Lx\n",
		    reg_rd64s(&tx4939_ddrcptr->drwin[3]));
	*eof = 1;
	return len;
}

static int
tx4939_proc_show_ebusc(char *sysbuf, char **start, off_t off,
		       int count, int *eof, void *data)
{
	int len = 0;
	len += sprintf(sysbuf + len, "***** tx4939_ebusc *****\n");
	len +=
	    sprintf(sysbuf + len, "tx4939_ebuscptr ->cr[0]       : 0x%016Lx\n",
		    reg_rd64(&tx4939_ebuscptr->cr[0]));
	len +=
	    sprintf(sysbuf + len, "tx4939_ebuscptr ->cr[1]       : 0x%016Lx\n",
		    reg_rd64(&tx4939_ebuscptr->cr[1]));
	len +=
	    sprintf(sysbuf + len, "tx4939_ebuscptr ->cr[2]       : 0x%016Lx\n",
		    reg_rd64(&tx4939_ebuscptr->cr[2]));
	len +=
	    sprintf(sysbuf + len, "tx4939_ebuscptr ->cr[3]       : 0x%016Lx\n",
		    reg_rd64(&tx4939_ebuscptr->cr[3]));
	*eof = 1;
	return len;
}

static int
tx4939_proc_show_pcic(char *sysbuf, char **start, off_t off,
		      int count, int *eof, void *data)
{
	int len = 0;
	len += sprintf(sysbuf + len, "***** tx4939_pcic *****\n");
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pciid       : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->pciid));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pcistatus   : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->pcistatus));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pciccrev    : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->pciccrev));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pcicfg1     : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->pcicfg1));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->p2gm0plbase : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->p2gm0plbase));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->p2gm0pubase : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->p2gm0pubase));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->p2gm1plbase : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->p2gm1plbase));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->p2gm1pubase : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->p2gm1pubase));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->p2gm2pbase  : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->p2gm2pbase));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->p2giopbase  : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->p2giopbase));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pcisid      : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->pcisid));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pcicapptr   : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->pcicapptr));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pcicfg2     : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->pcicfg2));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->g2ptocnt    : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->g2ptocnt));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->g2pstatus   : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->g2pstatus));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->g2pmask     : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->g2pmask));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pcisstatus  : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->pcisstatus));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pcimask     : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->pcimask));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->p2gcfg      : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->p2gcfg));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->p2gstatus   : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->p2gstatus));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->p2gmask     : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->p2gmask));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->p2gccmd     : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->p2gccmd));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pbareqport  : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->pbareqport));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pbacfg      : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->pbacfg));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pbastatus   : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->pbastatus));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pbamask     : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->pbamask));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pbabm       : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->pbabm));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pbacreq     : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->pbacreq));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pbacgnt     : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->pbacgnt));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pbacstate   : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->pbacstate));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->g2pmgbase[0]: 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(0)->g2pmgbase[0]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->g2pmgbase[1]: 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(0)->g2pmgbase[1]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->g2pmgbase[2]: 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(0)->g2pmgbase[2]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->g2piogbase  : 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(0)->g2piogbase));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->g2pmmask[0] : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->g2pmmask[0]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->g2pmmask[1] : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->g2pmmask[1]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->g2pmmask[2] : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->g2pmmask[2]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->g2piomask   : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->g2piomask));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->g2pmpbase[0]: 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(0)->g2pmpbase[0]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->g2pmpbase[1]: 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(0)->g2pmpbase[1]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->g2pmpbase[2]: 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(0)->g2pmpbase[2]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->g2piopbase  : 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(0)->g2piopbase));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pciccfg     : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->pciccfg));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pcicstatus  : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->pcicstatus));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pcicmask    : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->pcicmask));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->p2gmgbase[0]: 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(0)->p2gmgbase[0]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->p2gmgbase[1]: 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(0)->p2gmgbase[1]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->p2gmgbase[2]: 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(0)->p2gmgbase[2]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->p2giogbase  : 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(0)->p2giogbase));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->g2pcfgadrs  : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->g2pcfgadrs));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->p2gm0cfg    : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->p2gm0cfg));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->p2gm1cfg    : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->p2gm1cfg));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->p2gm2cfg    : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(0)->p2gm2cfg));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pdmca       : 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(0)->pdmca));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pdmga       : 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(0)->pdmga));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pdmpa       : 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(0)->pdmpa));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pdmctr      : 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(0)->pdmctr));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pdmcfg      : 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(0)->pdmcfg));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(0)  ->pdmsts      : 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(0)->pdmsts));
	*eof = 1;
	return len;
}

static int
tx4939_proc_show_pcic1(char *sysbuf, char **start, off_t off,
		       int count, int *eof, void *data)
{
	int len = 0;
	len += sprintf(sysbuf + len, "***** tx4939_pcic *****\n");
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pciid       : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->pciid));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pcistatus   : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->pcistatus));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pciccrev    : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->pciccrev));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pcicfg1     : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->pcicfg1));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->p2gm0plbase : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->p2gm0plbase));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->p2gm0pubase : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->p2gm0pubase));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->p2gm1plbase : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->p2gm1plbase));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->p2gm1pubase : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->p2gm1pubase));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->p2gm2pbase  : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->p2gm2pbase));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->p2giopbase  : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->p2giopbase));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pcisid      : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->pcisid));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pcicapptr   : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->pcicapptr));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pcicfg2     : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->pcicfg2));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->g2ptocnt    : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->g2ptocnt));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->g2pstatus   : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->g2pstatus));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->g2pmask     : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->g2pmask));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pcisstatus  : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->pcisstatus));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pcimask     : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->pcimask));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->p2gcfg      : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->p2gcfg));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->p2gstatus   : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->p2gstatus));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->p2gmask     : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->p2gmask));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->p2gccmd     : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->p2gccmd));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pbareqport  : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->pbareqport));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pbacfg      : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->pbacfg));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pbastatus   : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->pbastatus));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pbamask     : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->pbamask));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pbabm       : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->pbabm));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pbacreq     : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->pbacreq));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pbacgnt     : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->pbacgnt));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pbacstate   : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->pbacstate));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->g2pmgbase[0]: 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(1)->g2pmgbase[0]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->g2pmgbase[1]: 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(1)->g2pmgbase[1]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->g2pmgbase[2]: 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(1)->g2pmgbase[2]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->g2piogbase  : 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(1)->g2piogbase));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->g2pmmask[0] : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->g2pmmask[0]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->g2pmmask[1] : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->g2pmmask[1]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->g2pmmask[2] : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->g2pmmask[2]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->g2piomask   : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->g2piomask));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->g2pmpbase[0]: 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(1)->g2pmpbase[0]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->g2pmpbase[1]: 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(1)->g2pmpbase[1]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->g2pmpbase[2]: 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(1)->g2pmpbase[2]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->g2piopbase  : 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(1)->g2piopbase));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pciccfg     : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->pciccfg));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pcicstatus  : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->pcicstatus));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pcicmask    : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->pcicmask));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->p2gmgbase[0]: 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(1)->p2gmgbase[0]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->p2gmgbase[1]: 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(1)->p2gmgbase[1]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->p2gmgbase[2]: 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(1)->p2gmgbase[2]));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->p2giogbase  : 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(1)->p2giogbase));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->g2pcfgadrs  : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->g2pcfgadrs));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->p2gm0cfg    : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->p2gm0cfg));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->p2gm1cfg    : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->p2gm1cfg));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->p2gm2cfg    : 0x%08x\n",
		    reg_rd32(&tx4939_pcicptr(1)->p2gm2cfg));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pdmca       : 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(1)->pdmca));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pdmga       : 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(1)->pdmga));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pdmpa       : 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(1)->pdmpa));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pdmctr      : 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(1)->pdmctr));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pdmcfg      : 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(1)->pdmcfg));
	len +=
	    sprintf(sysbuf + len,
		    "tx4939_pcicptr(1)  ->pdmsts      : 0x%016Lx\n",
		    reg_rd64s(&tx4939_pcicptr(1)->pdmsts));
	*eof = 1;
	return len;
}

static int
tx4939_proc_show_irc(char *sysbuf, char **start, off_t off,
		     int count, int *eof, void *data)
{
	int len = 0;
	len += sprintf(sysbuf + len, "***** tx4939_irc *****\n");
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->den         : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->den));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->iscipb      : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->iscipb));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->dm0         : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->dm0));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->dm1         : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->dm1));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->irlvl[0]    : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->irlvl[0].reg));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->irlvl[1]    : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->irlvl[1].reg));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->irlvl[2]    : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->irlvl[2].reg));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->irlvl[3]    : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->irlvl[3].reg));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->irlvl[4]    : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->irlvl[4].reg));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->irlvl[5]    : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->irlvl[5].reg));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->irlvl[6]    : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->irlvl[6].reg));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->irlvl[7]    : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->irlvl[7].reg));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->irlvl[8]    : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->irlvl[8].reg));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->irlvl[9]    : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->irlvl[9].reg));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->irlvl[10]   : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->irlvl[10].reg));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->irlvl[11]   : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->irlvl[11].reg));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->irlvl[12]   : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->irlvl[12].reg));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->irlvl[13]   : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->irlvl[13].reg));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->irlvl[14]   : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->irlvl[14].reg));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->irlvl[15]   : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->irlvl[15].reg));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->msk         : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->msk));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->edc         : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->edc));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->pnd0        : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->pnd0));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->cs          : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->cs));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->pnd1        : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->pnd1));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->dm2         : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->dm2));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->dm3         : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->dm3));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->dbr0        : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->dbr0));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->dbr1        : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->dbr1));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->dben        : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->dben));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->flag0       : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->flag0));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->flag1       : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->flag1));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->pol         : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->pol));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->cnt         : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->cnt));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->maskint     : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->maskint));
	len +=
	    sprintf(sysbuf + len, "tx4939_ircptr   ->maskext     : 0x%08x\n",
		    reg_rd32(&tx4939_ircptr->maskext));
	*eof = 1;
	return len;
}

static int
tx4939_proc_show_ide0(char *sysbuf, char **start, off_t off,
		      int count, int *eof, void *data)
{
	int len = 0;
	len += sprintf(sysbuf + len, "tx4939_ataptr(0) ->sysctl     : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(0)->sysctl));
	len += sprintf(sysbuf + len, "tx4939_ataptr(0) ->xfer_cnt1  : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(0)->xfer_cnt1));
	len += sprintf(sysbuf + len, "tx4939_ataptr(0) ->xfer_cnt2  : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(0)->xfer_cnt2));
	len += sprintf(sysbuf + len, "tx4939_ataptr(0) ->sec_cnt    : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(0)->sec_cnt));
	len += sprintf(sysbuf + len, "tx4939_ataptr(0) ->strt_addl  : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(0)->strt_addl));
	len += sprintf(sysbuf + len, "tx4939_ataptr(0) ->strt_addu  : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(0)->strt_addu));
	len += sprintf(sysbuf + len, "tx4939_ataptr(0) ->add_ctrl   : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(0)->add_ctrl));
	len += sprintf(sysbuf + len, "tx4939_ataptr(0) ->hi_bcnt    : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(0)->hi_bcnt));
	len += sprintf(sysbuf + len, "tx4939_ataptr(0) ->lo_bcnt    : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(0)->lo_bcnt));
	len += sprintf(sysbuf + len, "tx4939_ataptr(0) ->pio_acc    : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(0)->pio_acc));
	len += sprintf(sysbuf + len, "tx4939_ataptr(0) ->h_rst_tim  : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(0)->h_rst_tim));
	len += sprintf(sysbuf + len, "tx4939_ataptr(0) ->int_ctl    : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(0)->int_ctl));
	len += sprintf(sysbuf + len, "tx4939_ataptr(0) ->dma_cmd    : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(0)->dma_cmd));
	len += sprintf(sysbuf + len, "tx4939_ataptr(0) ->dma_stat   : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(0)->dma_stat));
	*eof = 1;
	return len;
}

static int
tx4939_proc_show_ide1(char *sysbuf, char **start, off_t off,
		      int count, int *eof, void *data)
{
	int len = 0;
	len += sprintf(sysbuf + len, "tx4939_ataptr(1) ->sysctl     : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(1)->sysctl));
	len += sprintf(sysbuf + len, "tx4939_ataptr(1) ->xfer_cnt1  : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(1)->xfer_cnt1));
	len += sprintf(sysbuf + len, "tx4939_ataptr(1) ->xfer_cnt2  : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(1)->xfer_cnt2));
	len += sprintf(sysbuf + len, "tx4939_ataptr(1) ->sec_cnt    : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(1)->sec_cnt));
	len += sprintf(sysbuf + len, "tx4939_ataptr(1) ->strt_addl  : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(1)->strt_addl));
	len += sprintf(sysbuf + len, "tx4939_ataptr(1) ->strt_addu  : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(1)->strt_addu));
	len += sprintf(sysbuf + len, "tx4939_ataptr(1) ->add_ctrl   : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(1)->add_ctrl));
	len += sprintf(sysbuf + len, "tx4939_ataptr(1) ->hi_bcnt    : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(1)->hi_bcnt));
	len += sprintf(sysbuf + len, "tx4939_ataptr(1) ->lo_bcnt    : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(1)->lo_bcnt));
	len += sprintf(sysbuf + len, "tx4939_ataptr(1) ->pio_acc    : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(1)->pio_acc));
	len += sprintf(sysbuf + len, "tx4939_ataptr(1) ->h_rst_tim  : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(1)->h_rst_tim));
	len += sprintf(sysbuf + len, "tx4939_ataptr(1) ->int_ctl    : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(1)->int_ctl));
	len += sprintf(sysbuf + len, "tx4939_ataptr(1) ->dma_cmd    : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(1)->dma_cmd));
	len += sprintf(sysbuf + len, "tx4939_ataptr(1) ->dma_stat   : 0x%04x\n",
		       reg_rd16(&tx4939_ataptr(1)->dma_stat));
	*eof = 1;
	return len;
}

/**
 * tx4939_proc_setup - setup proc interface for TX4939 registers
 *
 * This function makes proc entry to show TX4939 registers, like as
 * "tx4939", "cp0", "mips_io_port_base", "ccfg".
 */

static int __init tx4939_proc_setup(void)
{
	struct proc_dir_entry *entry;

	if (tx4939_proc_entry == NULL)
		tx4939_proc_entry = proc_mkdir("tx4939", NULL);

	if (!tx4939_proc_entry) {
		printk(KERN_ERR "tx4939 cannot be registered in /proc\n");
		return -1;
	}

	entry = create_proc_entry("cp0", 0, tx4939_proc_entry);
	if (entry) {
		entry->read_proc = tx4939_proc_show_cp0;
		entry->data = 0;
	}

	entry = create_proc_entry("mips_io_port_base", 0, tx4939_proc_entry);
	if (entry) {
		entry->read_proc = tx4939_proc_show_mips_io_port_base;
		entry->data = 0;
	}

	entry = create_proc_entry("ccfg", 0, tx4939_proc_entry);
	if (entry) {
		entry->read_proc = tx4939_proc_show_ccfg;
		entry->data = 0;
	}

	entry = create_proc_entry("ebusc", 0, tx4939_proc_entry);
	if (entry) {
		entry->read_proc = tx4939_proc_show_ebusc;
		entry->data = 0;
	}

	entry = create_proc_entry("ddrc", 0, tx4939_proc_entry);
	if (entry) {
		entry->read_proc = tx4939_proc_show_ddrc;
		entry->data = 0;
	}

	entry = create_proc_entry("pcic", 0, tx4939_proc_entry);
	if (entry) {
		entry->read_proc = tx4939_proc_show_pcic;
		entry->data = 0;
	}

	entry = create_proc_entry("pcic1", 0, tx4939_proc_entry);
	if (entry) {
		entry->read_proc = tx4939_proc_show_pcic1;
		entry->data = 0;
	}

	entry = create_proc_entry("irc", 0, tx4939_proc_entry);
	if (entry) {
		entry->read_proc = tx4939_proc_show_irc;
		entry->data = 0;
	}

	entry = create_proc_entry("ide0", 0, tx4939_proc_entry);
	if (entry) {
		entry->read_proc = tx4939_proc_show_ide0;
		entry->data = 0;
	}

	entry = create_proc_entry("ide1", 0, tx4939_proc_entry);
	if (entry) {
		entry->read_proc = tx4939_proc_show_ide1;
		entry->data = 0;
	}

	return 0;
}

static int __init tx4939_proc_init(void)
{
	return tx4939_proc_setup();
}

static void __exit tx4939_proc_exit(void)
{
	/* need to do something */
}

module_init(tx4939_proc_init);
module_exit(tx4939_proc_exit);

MODULE_DESCRIPTION("tx4939 proc interface");
MODULE_LICENSE("GPL");

EXPORT_SYMBOL(tx4939_proc_entry);
