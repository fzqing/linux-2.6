/*
 * Monahans Power Management Routines
 *
 * Copyright (C) 2004, Intel Corporation(chao.xie@intel.com).
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 *(C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#undef	DEBUG
#define DEBUG
#include <linux/config.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/io.h>
#include <asm/mach/time.h>
#include <asm/cacheflush.h>
#include <asm/arch/mfp.h>
#include <asm/arch/mhn_gpio.h>
#include <asm/arch/mhn_pm.h>

#include "sleepwkr.h"

/* mtd.h declares another DEBUG macro definition */
#undef DEBUG
#include <linux/mtd/mtd.h>

/* The first 32KB is reserved and can't be accessed by kernel.
 * This restrict is only valid on BootROM V2.
 */
#define ISRAM_START	0x5c000000

/* MOBM_START should be larger than SRAM_START */
/* MOBM_START is used on MOBM V2.
 * The address is 0x5c014000. It means MOBM will be copied on the address.
 * On MOBM V3, it will be copied on 0x5c013000.
 */
#define MOBM_START	0x5c014000
#define MOBM_SIZE	(32 * 1024)
#define MOBM_OFFSET	8

#define ISRAM_SIZE	(128 * 2 * 1024)

/* MOBM V2 is used on MhnP B0/B1/B2, MhnPL B1 and MhnL A0
 * MOBM V3 is used on MhnLV A0
 */
enum mhn_obm {
	MHN_OBM_NULL,
	MHN_OBM_V2,
	MHN_OBM_V3,
	MHN_OBM_INVAL,
};

enum mhn_pm_mode {
	MHN_PM_RUN = 0,
	MHN_PM_IDLE = 1,
	MHN_PM_STANDBY = 3,
	MHN_PM_D0CS = 5,
	MHN_PM_SLEEP = 6,
};

extern struct subsystem power_subsys;

pm_wakeup_src_t wakeup_src;
EXPORT_SYMBOL(wakeup_src);

/* How long we will in sleep mode if duty cycle. */
unsigned int pm_sleeptime = 58;	/* In seconds. */
EXPORT_SYMBOL(pm_sleeptime);
unsigned int pm_msleeptime = 0;	/* In miliseconds. */

extern void mhn_cpu_sleep(unsigned int, unsigned int);
extern void mhn_cpu_resume(void);
extern void mhn_cpu_standby(unsigned int);

void (*event_notify) (int, int, void *, unsigned int) = NULL;
EXPORT_SYMBOL(event_notify);

static struct mhn_pm_regs mhn_pm_regs;

/*************************************************************************/
/* workaround for bug 2140448 */
static int is_wkr_2140448(void)
{
	return 0;
}

/*
 * MOBM V2 is applied on chips taped out before MhnLV A0.
 * MOBM V3 is applied on chips taped out after MhnLV A0.
 * MOBM V3 is also applied on MhnLV A0.
 */
static int calc_obm_ver(void)
{
	unsigned int cpuid;
	/* read CPU ID */
      __asm__("mrc p15, 0, %0, c0, c0, 0\n":"=r"(cpuid)
	    );
	if ((cpuid & 0xFFFF0000) != 0x69050000) {
		/* It's not xscale chip. */
		return MHN_OBM_INVAL;
	}
	if ((cpuid & 0x0000FFF0) == 0x00006420) {
		/* It's MhnP Ax */
		return MHN_OBM_V2;
	}
	if ((cpuid & 0x0000FFF0) == 0x00006820) {
		/* It's MhnP Bx */
		if ((cpuid & 0x0F) <= 6)
			return MHN_OBM_V2;
		else
			return MHN_OBM_V3;
	}
	if ((cpuid & 0x0000FFF0) == 0x00006880) {
		/* It's MhnL Ax */
		if ((cpuid & 0x0F) == 0)
			return MHN_OBM_V2;
		else
			return MHN_OBM_V3;
	}
	if ((cpuid & 0x0000FFF0) == 0x00006890) {
		/* It's MhnLV Ax */
		return MHN_OBM_V3;
	}
	return MHN_OBM_INVAL;
}

/* Return the address of OBM in RAM if successful.
 * Otherwise, return negative value.
 */
static void* load_obm(void)
{
	void *addr = NULL;
	struct mtd_info *mtd = NULL;
	int obm_ver, retlen;

	mtd = get_mtd_device(NULL, 0);
	if (mtd == NULL)
		return NULL;
	addr = kmalloc(MOBM_SIZE, GFP_KERNEL);
	if (!addr)
		return NULL;

	obm_ver = calc_obm_ver();
	if (obm_ver == MHN_OBM_V2) {
		/* MOBM begins from 0x0000 */
		if (mtd->oobblock == 2048)
			mtd->read(mtd, 0x0, MOBM_SIZE, &retlen, addr);
		else {
#if (MOBM_SIZE > 16 * 1024)
			mtd->read(mtd, 0, 0x4000, &retlen, addr);
			mtd->read(mtd, 0x4000, MOBM_SIZE - 0x4000, &retlen,
				  addr + 0x3e00);
#else
			mtd->read(mtd, 0, MOBM_SIZE, &retlen, addr);
#endif
		}
		addr += MOBM_OFFSET;
	} else if (obm_ver == MHN_OBM_V3) {
		/* MOBM begins from 0x20000 */
		if (mtd->oobblock == 2048)
			mtd->read(mtd, 0x20000, MOBM_SIZE, &retlen, addr);

	}
	pr_debug("load mobm into address: 0x%x\n", (unsigned int)addr);
	return addr;
}

static void mhn_intc_save(struct intc_regs *context)
{
	unsigned char __iomem *base = context->membase;
	unsigned int temp, i;

	context->iccr = readl(base + ICCR);
	for (i = 0; i < 32; i++) {
		context->ipr[i] = readl(base + IPR0_OFF + (i << 2));
	}
	for (i = 0; i < 21; i++) {
		context->ipr2[i] = readl(base + IPR32_OFF + (i << 2));
	}

	/* load registers by accessing co-processor */
      __asm__("mrc\tp6, 0, %0, c1, c0, 0":"=r"(temp));
	context->icmr = temp;
      __asm__("mrc\tp6, 0, %0, c7, c0, 0":"=r"(temp));
	context->icmr2 = temp;
      __asm__("mrc\tp6, 0, %0, c2, c0, 0":"=r"(temp));
	context->iclr = temp;
      __asm__("mrc\tp6, 0, %0, c8, c0, 0":"=r"(temp));
	context->iclr2 = temp;
}

static void mhn_intc_restore(struct intc_regs *context)
{
	unsigned char __iomem *base = context->membase;
	unsigned int temp, i;

	writel(context->iccr, base + ICCR_OFF);
	for (i = 0; i < 32; i++) {
		writel(context->ipr[i], base + IPR0_OFF + (i << 2));
	}
	for (i = 0; i < 21; i++) {
		writel(context->ipr2[i], base + IPR32_OFF + (i << 2));
	}

	temp = context->icmr;
      __asm__("mcr\tp6, 0, %0, c1, c0, 0": :"r"(temp));
	temp = context->icmr2;
      __asm__("mcr\tp6, 0, %0, c7, c0, 0": :"r"(temp));
	temp = context->iclr;
      __asm__("mcr\tp6, 0, %0, c2, c0, 0": :"r"(temp));
	temp = context->iclr2;
      __asm__("mcr\tp6, 0, %0, c8, c0, 0": :"r"(temp));
}

static void mhn_clk_save(struct clock_regs *context)
{
	unsigned char __iomem *base = context->membase;
	unsigned int tmp;
	context->aicsr = readl(base + AICSR_OFF);
	context->ckena = readl(base + CKENA_OFF);
	context->ckenb = readl(base + CKENB_OFF);
	context->oscc = readl(base + OSCC_OFF);
	/* Disable the processor to use the ring oscillator output clock
	 * as a clock source when transitioning from any low-power mode
	 * to D0 mode.
	 */
	tmp = readl(base + ACCR_OFF);
	tmp &= ~ACCR_PCCE;
	writel(tmp, base + ACCR_OFF);
}

static void mhn_clk_restore(struct clock_regs *context)
{
	unsigned char __iomem *base = context->membase;
	context->aicsr &= (AICSR_PCIE | AICSR_TCIE | AICSR_FCIE);
	writel(context->aicsr, base + AICSR_OFF);
	writel(context->ckena, base + CKENA_OFF);
	writel(context->ckenb, base + CKENB_OFF);
	writel(context->oscc, base + OSCC_OFF);
}

static void mhn_ost_save(struct ost_regs *context)
{
	unsigned char __iomem *base = context->membase;
	context->oscr4 = readl(base + OSCR4_OFF);
	context->omcr4 = readl(base + OMCR4_OFF);
	context->oier = readl(base + OIER_OFF);
}

static void mhn_ost_restore(struct ost_regs *context)
{
	unsigned char __iomem *base = context->membase;
	writel(context->oscr4, base + OSCR4_OFF);
	writel(context->omcr4, base + OMCR4_OFF);
	writel(context->oier, base + OIER_OFF);
}

static void mhn_sysbus_init(struct mhn_pm_regs *context)
{
	context->clock.membase = (unsigned char *)KSEG0(CLKBASE);
	context->intc.membase = (unsigned char *)KSEG0(INTCBASE);
	context->rtc.membase = (unsigned char *)KSEG0(RTCBASE);
	context->ost.membase = (unsigned char *)KSEG0(OSTBASE);
	context->pmu.membase = (unsigned char *)KSEG0(PMUBASE);
	context->smc.membase = (unsigned char *)KSEG1(SMCBASE);
	context->arb.membase = (unsigned char *)KSEG1(ARBBASE);

	context->sram_map = ioremap(ISRAM_START, ISRAM_SIZE);
	context->sram = vmalloc(ISRAM_SIZE);
	context->obm = (void *)load_obm();
	/* Two words begun from 0xC0000000 are used to store key information.
	 */
	context->data_pool = (unsigned char *)0xC0000000;
}

static void mhn_sysbus_save(struct mhn_pm_regs *context)
{
	unsigned char __iomem *base = NULL;
	unsigned int tmp;

	/* static memory controller */
	base = context->smc.membase;
	context->smc.msc0 = readl(base + MSC0_OFF);
	context->smc.msc1 = readl(base + MSC1_OFF);
	context->smc.sxcnfg = readl(base + SXCNFG_OFF);
	context->smc.memclkcfg = readl(base + MEMCLKCFG_OFF);
	context->smc.cscfg0 = readl(base + CSADRCFG0_OFF);
	context->smc.cscfg1 = readl(base + CSADRCFG1_OFF);
	context->smc.cscfg2 = readl(base + CSADRCFG2_OFF);
	context->smc.cscfg3 = readl(base + CSADRCFG3_OFF);

	/* system bus arbiters */
	base = context->arb.membase;
	context->arb.ctl1 = readl(base + ARBCTL1_OFF);
	context->arb.ctl2 = readl(base + ARBCTL2_OFF);

	/* pmu controller */
	base = context->pmu.membase;
	context->pmu.pecr = readl(base + PECR_OFF);
	context->pmu.pvcr = readl(base + PVCR_OFF);
	/* clear PSR */
	tmp = readl(base + PSR_OFF);
	tmp &= 0x07;
	writel(tmp, base + PSR_OFF);

	mhn_intc_save(&(context->intc));
	mhn_clk_save(&(context->clock));
	mhn_ost_save(&(context->ost));
	mhn_mfp_save();
	mhn_gpio_save();
}

static void mhn_sysbus_restore(struct mhn_pm_regs *context)
{
	unsigned char __iomem *base = NULL;

	mhn_mfp_restore();
	mhn_gpio_restore();
	mhn_ost_restore(&(context->ost));
	mhn_intc_restore(&(context->intc));
	mhn_clk_restore(&(context->clock));

	/* PMU controller */
	base = context->pmu.membase;
	/* status information will be lost in PECR */
	writel(0xA0000000, base + PECR_OFF);
	writel((context->pmu.pecr | PECR_E1IS | PECR_E0IS), base + PECR_OFF);
	writel(context->pmu.pvcr, base + PVCR_OFF);

	/* system bus arbiters */
	base = context->arb.membase;
	writel(context->arb.ctl1, base + ARBCTL1_OFF);
	writel(context->arb.ctl2, base + ARBCTL2_OFF);

	/* static memory controller */
	base = context->smc.membase;
	writel(context->smc.msc0, base + MSC0_OFF);
	writel(context->smc.msc1, base + MSC1_OFF);
	writel(context->smc.sxcnfg, base + SXCNFG_OFF);
	writel(context->smc.memclkcfg, base + MEMCLKCFG_OFF);
	writel(context->smc.cscfg0, base + CSADRCFG0_OFF);
	writel(context->smc.cscfg1, base + CSADRCFG1_OFF);
	writel(context->smc.cscfg2, base + CSADRCFG2_OFF);
	writel(context->smc.cscfg3, base + CSADRCFG3_OFF);

}

/* This function is used to set unit clock before system enters sleep.
 */
static void mhn_pm_set_cken(void)
{
	/*
	 * turn off SMC, GPIO,INTC clocks to save power in sleep mode.
	 * they will be turn on by BLOB during wakeup
	 */
	pxa_set_cken(CKEN_SMC, 0);
	pxa_set_cken(CKEN_GPIO, 0);
	pxa_set_cken(CKEN_INTC, 0);

	/*
	 * turn on clocks used by bootrom during wakeup
	 * they will be turn off by BLOB during wakeup
	 * D0CKEN_A clocks: bootrom, No.19
	 */
	pxa_set_cken(CKEN_BOOT, 1);
	pxa_set_cken(CKEN_TPM, 1);
	/* This bit must be enabled before entering low power mode. */
	pxa_set_cken(CKEN_HSIO2, 1);
}

/* This function is used to restore unit clock after system resumes.
 */
static void mhn_pm_restore_cken(void)
{
	pxa_set_cken(CKEN_SMC, 1);
	pxa_set_cken(CKEN_GPIO, 1);
	pxa_set_cken(CKEN_INTC, 1);
	pxa_set_cken(CKEN_BOOT, 0);
	pxa_set_cken(CKEN_TPM, 0);
}

/* This function is used to clear power manager status.
 */
static void mhn_clear_pm_status(unsigned char __iomem * base, int sys_level)
{
	unsigned int tmp;

	if (sys_level) {
		/* clear power manager status */
		tmp = readl(base + PSR_OFF);
		tmp &= PSR_MASK;
		writel(tmp, base + PSR_OFF);
	}
	/* clear application system status */
	tmp = readl(base + ASCR_OFF);
	tmp &= ASCR_MASK;
	writel(tmp, base + ASCR_OFF);
	/* clear all application subsystem reset status */
	tmp = readl(base + ARSR_OFF);
	writel(tmp, base + ARSR_OFF);
}

/* This function is used to set RTC time.
 * When it timeouts, it will wakeup system from low power mode.
 * There's limitation that only 65 seconds sleep time can be set by this way.
 * And user should avoid to use PIAR because it will be used as wakeup timer.
 *
 * Notice:
 * User can also choice use another RTC register to trigger wakeup event.
 * If so, keep pm_sleeptime as 0. Otherwise, those RTC registers event
 * will make user confused. System will only serve the first RTC event.
 */
static void mhn_set_wakeup_sec(int sleeptime, struct rtc_regs *context)
{
	unsigned char __iomem *base = context->membase;
	unsigned int tmp;
	if (sleeptime) {
		/* PIAR can not more than 65535 */
		if (sleeptime > 65)
			sleeptime = 65;
		pr_debug("Set RTC to wakeup system after %d sec\n", sleeptime);
		tmp = readl(base + RTSR_OFF);
		tmp &= ~(RTSR_PICE | RTSR_PIALE);
		writel(tmp, base + RTSR_OFF);
		/* set PIAR to sleep time, in ms */
		writel(sleeptime * 1000, base + PIAR_OFF);

		tmp = readl(base + RTSR_OFF);
		tmp |= RTSR_PICE;
		writel(tmp, base + RTSR_OFF);
	} else {
		/* Disable PIAR */
		tmp = readl(base + RTSR_OFF);
		tmp &= ~(RTSR_PICE | RTSR_PIALE);
		writel(tmp, base + RTSR_OFF);
	}
}

/* This function is used to set OS Timer4 time.
 * The time interval may not be accurate. Because it's derived from 32.768kHz
 * oscillator.
 */
static void mhn_set_wakeup_msec(int msleeptime, struct ost_regs *context)
{
	unsigned char __iomem *base = context->membase;
	unsigned int tmp;
	if (msleeptime) {
		pr_debug("Set OS Timer4 to wakeup system after %d msec\n",
			 msleeptime);
		tmp = readl(base + OIER_OFF);
		tmp &= ~0x0010;
		writel(tmp, base + OIER_OFF);
		/* set the time interval of sleep */
		writel(msleeptime, base + OSMR4_OFF);
		/* use 32.768kHz oscillator when cpu is in low power mode */
		writel(0x0082, base + OMCR4_OFF);
		tmp = readl(base + OIER_OFF);
		tmp |= 0x0010;
		writel(tmp, base + OIER_OFF);
		/* kick off the OS Timer4 */
		writel(0, base + OSCR4_OFF);
	} else {
		/* Disable OS Timer 4 */
		tmp = readl(base + OIER_OFF);
		tmp &= ~0x0010;
		writel(tmp, base + OIER_OFF);
	}
}

/*
 * Clear the wakeup source event.
 */
static void pm_clear_wakeup_src(pm_wakeup_src_t src)
{
	/* set MFPR */
	if (src.value & MHN_PM_WE_MKEY) {
		mhn_mfp_set_edge(MFP_KP_MKIN_0, MFP_EDGE_NONE);
		mhn_mfp_set_edge(MFP_KP_MKIN_1, MFP_EDGE_NONE);
		mhn_mfp_set_edge(MFP_KP_MKIN_2, MFP_EDGE_NONE);
		mhn_mfp_set_edge(MFP_KP_MKIN_3, MFP_EDGE_NONE);
		mhn_mfp_set_edge(MFP_KP_MKIN_4, MFP_EDGE_NONE);
		mhn_mfp_set_edge(MFP_KP_MKIN_5, MFP_EDGE_NONE);
#ifdef	CONFIG_MACH_ZYLONITE
		mhn_mfp_set_edge(MFP_KP_MKIN_6, MFP_EDGE_NONE);
		mhn_mfp_set_edge(MFP_KP_MKIN_7, MFP_EDGE_NONE);
#endif
	}
	if (src.value & MHN_PM_WE_DKEY) {
		mhn_mfp_set_edge(MFP_KP_DKIN_0, MFP_EDGE_NONE);
		mhn_mfp_set_edge(MFP_KP_DKIN_1, MFP_EDGE_NONE);
	}
	if (src.value & MHN_PM_WE_UART1) {
		mhn_mfp_set_edge(MFP_FFRXD, MFP_EDGE_NONE);
	}
	if (src.value & MHN_PM_WE_UART2) {
		mhn_mfp_set_edge(MFP_RSVD_BT_RXD, MFP_EDGE_NONE);
		mhn_mfp_set_edge(MFP_RSVD_BT_CTS, MFP_EDGE_NONE);
	}
	if (src.value & MHN_PM_WE_UART3) {
		mhn_mfp_set_edge(MFP_STD_RXD, MFP_EDGE_NONE);
	}
	if (src.value & MHN_PM_WE_OST) {
		unsigned char __iomem *base = mhn_pm_regs.ost.membase;
		unsigned int tmp;
		writel(0x10, base + OSSR_OFF);
		tmp = readl(base + OIER_OFF);
		tmp &= ~0x10;
		writel(tmp, base + OIER_OFF);
	}
}

static void pm_select_wakeup_src(enum mhn_pm_mode lp_mode,
				 pm_wakeup_src_t src, struct pmu_regs *context)
{
	unsigned char __iomem *base = context->membase;
	unsigned int tmp;
	struct mhn_pm_regs *p = NULL;

	/* set MFPR */
	if (src.value & MHN_PM_WE_MKEY) {
		mhn_mfp_set_edge(MFP_KP_MKIN_0, MFP_EDGE_BOTH);
		mhn_mfp_set_edge(MFP_KP_MKIN_1, MFP_EDGE_BOTH);
		mhn_mfp_set_edge(MFP_KP_MKIN_2, MFP_EDGE_BOTH);
		mhn_mfp_set_edge(MFP_KP_MKIN_3, MFP_EDGE_BOTH);
		mhn_mfp_set_edge(MFP_KP_MKIN_4, MFP_EDGE_BOTH);
		mhn_mfp_set_edge(MFP_KP_MKIN_5, MFP_EDGE_BOTH);
#ifdef	CONFIG_MACH_ZYLONITE
		mhn_mfp_set_edge(MFP_KP_MKIN_6, MFP_EDGE_BOTH);
		mhn_mfp_set_edge(MFP_KP_MKIN_7, MFP_EDGE_BOTH);
#endif
	}
	if (src.value & MHN_PM_WE_DKEY) {
		mhn_mfp_set_edge(MFP_KP_DKIN_0, MFP_EDGE_BOTH);
		mhn_mfp_set_edge(MFP_KP_DKIN_1, MFP_EDGE_BOTH);
	}
	if (src.value & MHN_PM_WE_UART1)
		mhn_mfp_set_edge(MFP_FFRXD, MFP_EDGE_FALL);
	if (src.value & MHN_PM_WE_UART2) {
		mhn_mfp_set_edge(MFP_RSVD_BT_RXD, MFP_EDGE_BOTH);
		mhn_mfp_set_edge(MFP_RSVD_BT_CTS, MFP_EDGE_BOTH);
	}
	if (src.value & MHN_PM_WE_UART3)
		mhn_mfp_set_edge(MFP_STD_RXD, MFP_EDGE_BOTH);
	if (src.value & MHN_PM_WE_RTC) {
		p = container_of(context, struct mhn_pm_regs, pmu);
		mhn_set_wakeup_sec(pm_sleeptime, &(p->rtc));
	}
	if (src.value & MHN_PM_WE_OST) {
		p = container_of(context, struct mhn_pm_regs, pmu);
		mhn_set_wakeup_msec(pm_msleeptime, &(p->ost));
	}

	/* set wakeup register */
	if (lp_mode == MHN_PM_SLEEP) {
		writel(0xFFFFFFFF, base + PWSR_OFF);
		writel(0, base + PWER_OFF);
		writel(0xFFFFFFFF, base + AD3SR_OFF);
		writel(0, base + AD3ER_OFF);

		tmp = readl(base + PWER_OFF);
		if (src.value & MHN_PM_WE_RTC)
			tmp |= PWER_WERTC;
		if (src.value & MHN_PM_WE_EXTERNAL0)
			tmp |= (PWER_WER0 | PWER_WEF0);
		if (src.value & MHN_PM_WE_EXTERNAL1)
			tmp |= (PWER_WER1 | PWER_WEF1);
		writel(tmp, base + PWER_OFF);

		writel(src.value & AD3ER_MASK, base + AD3ER_OFF);
	}
	if (lp_mode == MHN_PM_STANDBY) {
		writel(0xFFFFFFFF, base + AD2D0SR_OFF);
		writel(0, base + AD2D0ER_OFF);
		writel(src.value & AD2D0ER_MASK, base + AD2D0ER_OFF);
	}
}

/* set the default wakeup source */
static void pm_init_wakeup_src(pm_wakeup_src_t * src, struct pmu_regs *context)
{
	src->value = (MHN_PM_WE_EXTERNAL0 | MHN_PM_WE_RTC |
		      MHN_PM_WE_MKEY | MHN_PM_WE_DKEY |
		      MHN_PM_WE_TSI | MHN_PM_WE_UART1);

	/* clear the related wakeup source */
	pm_select_wakeup_src(MHN_PM_SLEEP, *src, context);
	pm_clear_wakeup_src(*src);
}

/*************************************************************************/

static void flush_cpu_cache(void)
{
	__cpuc_flush_kern_all();
}

struct os_header {
	int version;
	int identifier;
	int address;
	int size;
	int reserved;
};

static int mhn_pm_enter_sleep(struct mhn_pm_regs *pm_regs)
{
	unsigned char __iomem *base = pm_regs->pmu.membase;
	unsigned int tmp;

	mhn_sysbus_save(pm_regs);

	if (is_wkr_2140448())
		sleep_wkr_start(0xf6f50084);

	pm_select_wakeup_src(MHN_PM_SLEEP, wakeup_src, &(pm_regs->pmu));

	mhn_pm_set_cken();

	/* should set:modeSaveFlags, areaAddress, flushFunc, psprAddress,
	 * extendedChecksumByteCount */
	pm_regs->pm_data.modeSaveFlags = 0x3f;	/* PM_MODE_SAVE_FLAG_SVC; */
	pm_regs->pm_data.flushFunc = flush_cpu_cache;
	pm_regs->pm_data.areaAddress = (unsigned int)&(pm_regs->pm_data);
	pm_regs->pm_data.psprAddress = (unsigned int)base + PSPR_OFF;
	pm_regs->pm_data.extendedChecksumByteCount =
	    sizeof(struct mhn_pm_regs) - sizeof(struct pm_save_data);
	pr_debug("ext size:%d, save size%d\n",
		 pm_regs->pm_data.extendedChecksumByteCount,
		 sizeof(struct pm_save_data));

	/* save the resume back address into SDRAM */
	pm_regs->word0 = readl(pm_regs->data_pool);
	pm_regs->word1 = readl(pm_regs->data_pool + 4);
	writel(virt_to_phys(mhn_cpu_resume), pm_regs->data_pool);
	writel(virt_to_phys(&(pm_regs->pm_data)), pm_regs->data_pool + 4);

	mhn_clear_pm_status(base, 1);

	/* make sure that sram bank 0 is not off */
	tmp = readl(base + AD3R_OFF);
	tmp |= 0x101;
	writel(tmp, base + AD3R_OFF);

	pr_debug("ready to sleep:0x%lx\n", virt_to_phys(&(pm_regs->pm_data)));

	/* go to Zzzz */
	mhn_cpu_sleep((unsigned int)&(pm_regs->pm_data),
		      virt_to_phys(&(pm_regs->pm_data)));

	/* come back */
	if (is_wkr_2140448())
		sleep_wkr_end(0xf6f50084);

	writel(pm_regs->word0, pm_regs->data_pool);
	writel(pm_regs->word1, pm_regs->data_pool + 4);

	mhn_pm_restore_cken();
	mhn_sysbus_restore(pm_regs);

	mhn_clear_pm_status(base, 1);
	/* clear RDH */
	tmp = readl(base + ASCR_OFF);
	tmp &= ~ASCR_RDH;
	writel(tmp, base + ASCR_OFF);

	pm_clear_wakeup_src(wakeup_src);

	/* Clear this bit after returns from low power mode.
	 * Clear this bit can save power.
	 */
	pxa_set_cken(CKEN_HSIO2, 0);

	pr_debug("Resume Back\n");

	return 0;
}

static int mhn_pm_enter_standby(struct mhn_pm_regs *pm_regs)
{
	unsigned char __iomem *base = pm_regs->pmu.membase;

	/* This bit must be enabled before entering low power mode. */
	pxa_set_cken(CKEN_HSIO2, 1);

	mhn_clear_pm_status(base, 0);
	/* make sure that sram bank 0 is not off */

	writel(0x109, base + AD2R_OFF);

	if (is_wkr_2140448())
		sleep_wkr_start(0xf6f50084);

	pm_select_wakeup_src(MHN_PM_STANDBY, wakeup_src, &(pm_regs->pmu));

	mhn_cpu_standby((unsigned int)pm_regs->sram_map + 0x8000);

	if (is_wkr_2140448())
		sleep_wkr_end(0xf6f50084);

	mhn_clear_pm_status(base, 0);
	pm_clear_wakeup_src(wakeup_src);
	/* This bit must be disabled after entering low power mode. */
	pxa_set_cken(CKEN_HSIO2, 0);

	pr_debug("*** made it back from standby\n");

	return 0;
}

static int mhn_pm_enter(suspend_state_t state)
{
#ifdef CONFIG_FB_PXA
	/* unless we are entering "lcdrefresh", turn off backlight */
	mhn_gpio_set_level(14, 0);
#endif
	if (state == PM_SUSPEND_MEM)
		return mhn_pm_enter_sleep(&mhn_pm_regs);
	else if (state == PM_SUSPEND_STANDBY)
		return mhn_pm_enter_standby(&mhn_pm_regs);
	else
		return -EINVAL;
}

/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int mhn_pm_prepare(suspend_state_t state)
{
	struct os_header header;
	int obm_ver;

	if (state == PM_SUSPEND_MEM) {
		/* backup data in ISRAM */
		memcpy(mhn_pm_regs.sram, mhn_pm_regs.sram_map, ISRAM_SIZE);
		obm_ver = calc_obm_ver();
		if (obm_ver == MHN_OBM_V2) {
			/* load OBM into ISRAM
			 * The target address is 0x5c014000
			 */
			if (mhn_pm_regs.obm)
				memcpy(mhn_pm_regs.sram_map + 0x14000,
				       mhn_pm_regs.obm, MOBM_SIZE);
		} else if (obm_ver == MHN_OBM_V3) {
			/* load OBM into ISRAM
			 * The target address is 0x5c013000
			 * The main purpose to load obm is to initialize DDR.
			 * When OBM found it's a resume process, it will jump
			 * to resume routine what resides in DDR.
			 */
			memset(&header, 0, sizeof(struct os_header));
			header.version = 3;
			header.identifier = 0x5265736D;	/* RESM */
			header.address = 0x5c013000;
			header.size = MOBM_SIZE;
			/* 0x5c008000 */
			memcpy(mhn_pm_regs.sram_map + 0x8000, &header,
			       sizeof(struct os_header));
			/* 0x5c013000 */
			if (mhn_pm_regs.obm)
				memcpy(mhn_pm_regs.sram_map + 0x13000,
				       mhn_pm_regs.obm, MOBM_SIZE);
		}
	} else if ((state == PM_SUSPEND_STANDBY)) {
		/* FIXME: allocat SRAM to execute D1/D2 entry/exit code.
		 * Try not to use it in the future.
		 */
		/* backup data in ISRAM */
		memcpy(mhn_pm_regs.sram, mhn_pm_regs.sram_map, 1024);
	}

	return 0;
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static int mhn_pm_finish(suspend_state_t state)
{
	if (state == PM_SUSPEND_MEM) {
		/* restore data in ISRAM */
		memcpy(mhn_pm_regs.sram_map, mhn_pm_regs.sram, ISRAM_SIZE);
	} else if ((state == PM_SUSPEND_STANDBY)) {
		/* restore data in ISRAM */
		memcpy(mhn_pm_regs.sram_map, mhn_pm_regs.sram, 1024);
	}

	return 0;
}

/*
 * Set to PM_DISK_FIRMWARE so we can quickly veto suspend-to-disk.
 */
static struct pm_ops mhn_pm_ops = {
	.pm_disk_mode = PM_DISK_FIRMWARE,
	.prepare = mhn_pm_prepare,
	.enter = mhn_pm_enter,
	.finish = mhn_pm_finish,
};

#define pm_attr(_name, object) \
static ssize_t _name##_store(struct subsystem * subsys,			\
			const char * buf, size_t n)			\
{									\
	sscanf(buf, "%u", &object);					\
	return n;							\
}									\
static ssize_t _name##_show(struct subsystem * subsys, char * buf)	\
{									\
	return sprintf(buf, "%u\n", object);				\
}									\
static struct subsys_attribute _name##_attr = { 			\
	.attr   = {                             			\
		.name = __stringify(_name),     			\
		.mode = 0644,                   			\
	},                                      			\
	.show   = _name##_show,                 			\
	.store  = _name##_store,                			\
}

pm_attr(sleeptime, pm_sleeptime);
pm_attr(msleeptime, pm_msleeptime);

static int __init mhn_pm_init(void)
{
	pm_set_ops(&mhn_pm_ops);

	sysfs_create_file(&power_subsys.kset.kobj, &sleeptime_attr.attr);
	sysfs_create_file(&power_subsys.kset.kobj, &msleeptime_attr.attr);

	/* set memory base */
	mhn_sysbus_init(&mhn_pm_regs);
	/* set default wakeup src */
	pm_init_wakeup_src(&wakeup_src, &(mhn_pm_regs.pmu));

	return 0;
}

late_initcall(mhn_pm_init);
