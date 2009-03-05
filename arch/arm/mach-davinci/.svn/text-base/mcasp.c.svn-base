/*
 * mcasp.c - Interface to Multichannel Audio Serial Port (McASP)
 *
 * Copyright (C) 2007  Texas Instruments, India
 * Author:Nirmal Pandey <n-pandey@ti.com>,
 *        Suresh Rajashekara <suresh.r@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>

#include <asm/io.h>
#include <asm/arch/mcasp.h>
#include <asm/hardware/clock.h>
#include <asm/atomic.h>

#include <asm/arch/evm_audio_info.h>
#include <asm/arch/davinci-audio-config.h>

#define MODULE_NAME   "McASP"

/* Debug levels */
#define LOW      25
#define MEDIUM   50
#define HIGH     75
#define CRITICAL 100

/* Undefine/define this to disable/enable debugging */
/* #define McASP_DEBUG */

#ifdef McASP_DEBUG
static s8 mcasp_debug_level = LOW;
#define McASP_DEBUG0(format, arg...)	printk(KERN_ALERT MODULE_NAME \
					       " DEBUG: " format "\n",  ## arg )
#define McASP_DEBUG1(level, format, arg...)	 do {\
	if (level <= mcasp_debug_level) {	\
		printk(KERN_ALERT MODULE_NAME " DEBUG: " \
		       format "\n",  ## arg );\
	}} while (0)

#define mcasp_info(format, arg...) printk(KERN_INFO MODULE_NAME " INFO: "\
					  format "\n",  ## arg )
#define mcasp_warn(format, arg...) printk(KERN_WARNING MODULE_NAME  \
					  " WARNING: " format "\n",  ## arg )
#define mcasp_err(format, arg...) printk(KERN_ERR MODULE_NAME " ERROR: " \
					 format "\n",  ## arg )

#else				/* McASP_DEBUG */
#define McASP_DEBUG0(fmt, args...)
#define McASP_DEBUG1(level, fmt, args...)
#define mcasp_info(format, arg...)
#define mcasp_warn(format, arg...)
#define mcasp_err(format, arg...)
#endif				/* McASP_DEBUG */

/* Undefine/define this to disable/enable tracing */
/* #define McASP_TRACE */

#ifdef McASP_TRACE

#define McASP_FN_IN   \
	printk(KERN_ALERT MODULE_NAME " DEBUG: Entering Function %s\n", \
		 __FUNCTION__ )
#define McASP_FN_OUT(retval)  \
	printk(KERN_ALERT MODULE_NAME \
		 " DEBUG: Leaving Function %s (Ret = %d)\n", \
		 __FUNCTION__, retval )
#define McASP_FN_OUT_ADDR(retval)  \
	printk(KERN_ALERT MODULE_NAME \
		 " DEBUG: Leaving Function %s (Ret = %lx)\n", \
		 __FUNCTION__, (unsigned long)retval )
#else				/* McASP_TRACE */
#define McASP_FN_IN
#define McASP_FN_OUT(retval)
#define McASP_FN_OUT_ADDR(retval)
#endif				/* McASP_TRACE */

/* If the McASP unit is in use, just display a warning. The warns the user if
 * the user calls a function (accidently) which requires the mcasp
 * to be stopped, which might result in unexpected behaviour of McASP.
 */
#define DISPLAY_MCASP_IN_USE_WARNING(id)      do {\
   if (mcasp_get(id) < 0) { \
     mcasp_warn ("McASP%d is in use. Hope you know what you are doing!\n", id);\
   }} while (0)

/* Since the internal clock of is 24MHz on Davinci-HD EVM, only the following
 * MAX_SAMPLE_RATES sample rates are supported. */
#define MAX_SAMPLE_RATES 2
static u32 sample_rates_supported[MAX_SAMPLE_RATES] = { 22050, 44100 };

struct asp_file_operations mcasp_fops = {
	.init = mcasp_dev_init,
	.de_init = mcasp_dev_deinit,
	.configure = mcasp_configure,
	.unconfigure = mcasp_unconfigure,
	.configure_reg = mcasp_reg_configure,
	.start = mcasp_start,
	.stop = mcasp_stop,
	.start_tx = mcasp_start_tx,
	.stop_tx = mcasp_stop_tx,
	.start_rx = mcasp_start_rx,
	.stop_rx = mcasp_stop_rx,
	.get_info = mcasp_get_info,
	.get_config = mcasp_get_config,
	.get_clock = mcasp_get_clock,
	.set_sample_rate = mcasp_set_sample_rate,
	.get_sample_rate = mcasp_get_sample_rate,
};

mcasp_device_t *mcasp;

static inline mcasp_unit_t *find_mcasp(u8 mcasp_id)
{
	struct list_head *temp;
	mcasp_unit_t *unit = NULL;

	McASP_FN_IN;
	if (mcasp == NULL) {
		mcasp_err("McASP is not initialized\n");
		return NULL;
	}

	/* look at the first element */
	unit = list_entry(mcasp->mcasp_list, struct __mcasp_unit, list);
	if (unit->info.id == mcasp_id) {
		return unit;
	}

	/* go throu the rest of the list */
	list_for_each(temp, mcasp->mcasp_list) {
		unit = list_entry(temp, struct __mcasp_unit, list);
		if (unit->info.id == mcasp_id) {
			return unit;
		}
		unit = NULL;
	}

	mcasp_err("McASP with ID=%d was not initialized\n", mcasp_id);
	return unit;
}

struct clk *mcasp_get_clock(u8 mcasp_id)
{				/* Change */
	mcasp_unit_t *mcasp_unit;

	mcasp_unit = find_mcasp(mcasp_id);

	if (mcasp_unit == NULL) {
		McASP_FN_OUT(0);
		return NULL;	/* Unlikely */
	}

	return mcasp_unit->clock;
}
EXPORT_SYMBOL(mcasp_get_clock);

static inline u8 __mcasp_unit_count(void)
{
	struct list_head *temp;
	u8 unit_count = 0;

	McASP_FN_IN;
	list_for_each(temp, mcasp->mcasp_list) {
		unit_count++;
	}

	McASP_FN_OUT(unit_count);
	return unit_count;
}

static inline void __mcasp_set_pinmux(void)
{
	McASP_FN_IN;
#define AUXCLK_NK 	__REG(0x01C4005C)
	/* NK : This bit must be set to enable AUXCLK */
	AUXCLK_NK &= 0xFEFFFFFF;
	/* Ideally the base port should take care of setting the proper pinmux
	 * settings. If there are any other pinmuxing settings to be done, then
	 * please do it here and call the function at the required place. */
	McASP_FN_OUT(0);
}

static inline s8 __mcasp_set_lpsc(u8 mcasp_id)
{
	struct clk *clkp;
	mcasp_unit_t *mcasp_unit;
	u8 ret_val = 0;

	mcasp_unit = find_mcasp(mcasp_id);

	if (mcasp_unit == NULL) {
		McASP_FN_OUT(0);
		ret_val = -1;	/* Unlikely */
	} else if (mcasp_unit->info.lpsc != NULL) {
		clkp = clk_get(NULL, mcasp_unit->info.lpsc);

		if (IS_ERR(clkp)) {
			mcasp_err("Unable to get the clock for McASP%d\n",
				  mcasp_id);
			McASP_FN_OUT(-1);
			ret_val = 1;
		} else {
			mcasp_unit->clock = clkp;
			clk_use(mcasp_unit->clock);
			clk_enable(mcasp_unit->clock);
		}
	}

	McASP_FN_OUT(0);
	return ret_val;
}

static inline void mcasp_set_reg(volatile u32 reg, u32 val)
{
	outl(inl(reg) | val, reg);
}

static inline u32 mcasp_get_reg(volatile u32 reg)
{
	return inl(reg);
}

s8 mcasp_get_free(void)
{
	struct list_head *temp;
	mcasp_unit_t *unit = NULL;

	McASP_FN_IN;
	if (mcasp == NULL) {
		mcasp_err("McASP is not initialized\n");
		McASP_FN_OUT(-1);
		return -1;
	}

	list_for_each(temp, mcasp->mcasp_list) {
		unit = list_entry(temp, struct __mcasp_unit, list);
		if ((atomic_read(&unit->tx_in_use) == 0) &&
		    (atomic_read(&unit->rx_in_use) == 0)) {
			McASP_FN_OUT(unit->info.id);
			return unit->info.id;
		}
	}

	McASP_FN_OUT(-1);
	return -1;
}
EXPORT_SYMBOL(mcasp_get_free);

s8 mcasp_get(u8 mcasp_id)
{
	mcasp_unit_t *mcasp_unit;

	McASP_FN_IN;
	mcasp_unit = find_mcasp(mcasp_id);

	if (unlikely(mcasp_unit == NULL)) {
		McASP_FN_OUT(-1);
		return -1;
	}

	if ((atomic_read(&mcasp_unit->tx_in_use) == 1) ||
	    (atomic_read(&mcasp_unit->rx_in_use) == 1)) {
		McASP_FN_OUT(-1);
		return -1;
	}

	McASP_FN_OUT(0);
	return 0;
}
EXPORT_SYMBOL(mcasp_get);

s8 mcasp_put(u8 mcasp_id)
{
	mcasp_unit_t *mcasp_unit;

	McASP_FN_IN;
	mcasp_unit = find_mcasp(mcasp_id);

	if (unlikely(mcasp_unit == NULL)) {
		McASP_FN_OUT(-1);
		return -1;
	}

	DISPLAY_MCASP_IN_USE_WARNING(mcasp_id);

	if (atomic_read(&mcasp_unit->tx_in_use) != 0) {
		atomic_set(&mcasp_unit->tx_in_use, 0);
	}

	if (atomic_read(&mcasp_unit->rx_in_use) != 0) {
		atomic_set(&mcasp_unit->rx_in_use, 0);
	}
	McASP_FN_OUT(0);
	return 0;
}
EXPORT_SYMBOL(mcasp_put);

static inline void mcasp_set_gblctl(mcasp_registers_t *regs, u32 val)
{
	McASP_FN_IN;
	mcasp_set_reg((u32) & regs->gblctl, val);
	while ((mcasp_get_reg((u32) & regs->gblctl) & val) != val) ;
	McASP_FN_OUT(0);
}

static inline void mcasp_set_gblctlr(mcasp_registers_t *regs, u32 val)
{
	McASP_FN_IN;
	mcasp_set_reg((u32) & regs->gblctlr, val);
	while ((mcasp_get_reg((u32) & regs->gblctl) & val) != val) ;
	McASP_FN_OUT(0);
}

static inline void mcasp_set_gblctlx(mcasp_registers_t *regs, u32 val)
{
	McASP_FN_IN;
	mcasp_set_reg((u32) & regs->gblctlx, val);
	while ((mcasp_get_reg((u32) & regs->gblctl) & val) != val) ;
	McASP_FN_OUT(0);
}

s8 mcasp_start_rx(u8 mcasp_id)
{
	mcasp_unit_t *mcasp_unit;
	mcasp_registers_t *regs;

	McASP_FN_IN;
	mcasp_unit = find_mcasp(mcasp_id);

	if (unlikely(mcasp_unit == NULL)) {
		McASP_FN_OUT(-1);
		return -1;
	}

	if (atomic_read(&mcasp_unit->rx_in_use) == 1) {
		McASP_FN_OUT(-1);
		return -1;
	}

	regs = (mcasp_registers_t *) mcasp_unit->info.reg_base;

	mcasp_set_gblctlr(regs, RXHCLKRST);
	mcasp_set_gblctlr(regs, RXCLKRST);
	mcasp_set_gblctlr(regs, RXSERCLR);

	mcasp_set_reg((u32) & regs->rxbuf[0], 0);

	mcasp_set_gblctlr(regs, RXSMRST);
	mcasp_set_gblctlr(regs, RXFSRST);

	mcasp_set_reg((u32) & regs->rxbuf[0], 0);

	mcasp_set_gblctlr(regs, RXSMRST);
	mcasp_set_gblctlr(regs, RXFSRST);

	atomic_set(&mcasp_unit->rx_in_use, 1);

	McASP_FN_OUT(-1);
	return 0;
}

s8 mcasp_start_tx(u8 mcasp_id)
{
	mcasp_unit_t *mcasp_unit;
	mcasp_registers_t *regs;

	McASP_FN_IN;
	mcasp_unit = find_mcasp(mcasp_id);

	if (unlikely(mcasp_unit == NULL)) {
		McASP_FN_OUT(-1);
		return -1;
	}

	if (atomic_read(&mcasp_unit->tx_in_use) == 1) {
		McASP_FN_OUT(-1);
		return -1;
	}

	regs = (mcasp_registers_t *) mcasp_unit->info.reg_base;

	mcasp_set_gblctlx(regs, TXHCLKRST);
	mcasp_set_gblctlx(regs, TXCLKRST);
	mcasp_set_gblctlx(regs, TXSERCLR);

	mcasp_set_reg((u32) & regs->txbuf[0], 0);

	mcasp_set_gblctlx(regs, TXSMRST);
	mcasp_set_gblctlx(regs, TXFSRST);

	mcasp_set_reg((u32) & regs->txbuf[0], 0);
	do {
	} while (!(regs->xrsrctl[0] & 0x10));	/* Check for Tx ready */
	regs->txbuf[0] = 0;

	atomic_set(&mcasp_unit->tx_in_use, 1);

	McASP_FN_OUT(0);
	return 0;
}

s8 mcasp_stop_rx(u8 mcasp_id)
{
	mcasp_unit_t *mcasp_unit;
	mcasp_registers_t *regs;

	McASP_FN_IN;
	mcasp_unit = find_mcasp(mcasp_id);

	if (unlikely(mcasp_unit == NULL)) {
		McASP_FN_OUT(-1);
		return -1;
	}

	if (atomic_read(&mcasp_unit->rx_in_use) != 1) {
		mcasp_info("McASP%d is not started\n", mcasp_id);
		McASP_FN_OUT(-1);
		return 1;
	}

	regs = (mcasp_registers_t *) mcasp_unit->info.reg_base;

	regs->gblctlr = 0;
	regs->rxstat = 0xFFFFFFFF;

	atomic_set(&mcasp_unit->rx_in_use, 0);

	McASP_FN_OUT(0);
	return 0;
}

s8 mcasp_stop_tx(u8 mcasp_id)
{
	mcasp_unit_t *mcasp_unit;
	mcasp_registers_t *regs;

	McASP_FN_IN;
	mcasp_unit = find_mcasp(mcasp_id);

	if (unlikely(mcasp_unit == NULL)) {
		McASP_FN_OUT(-1);
		return -1;
	}

	if (atomic_read(&mcasp_unit->tx_in_use) != 1) {
		mcasp_info("McASP%d is not started\n", mcasp_id);
		McASP_FN_OUT(1);
		return 1;
	}
	regs = (mcasp_registers_t *) mcasp_unit->info.reg_base;

	regs->gblctlx = 0;
	regs->txstat = 0xFFFFFFFF;
	atomic_set(&mcasp_unit->tx_in_use, 0);

	McASP_FN_OUT(0);
	return 0;
}

s8 mcasp_stop(u8 mcasp_id)
{
	mcasp_unit_t *mcasp_unit;

	McASP_FN_IN;
	mcasp_unit = find_mcasp(mcasp_id);

	if (unlikely(mcasp_unit == NULL)) {
		McASP_FN_OUT(-1);
		return -1;
	}

	if (mcasp_unit->info.flags & TX_SUPPORT) {
		if (atomic_read(&mcasp_unit->tx_in_use) != 1) {
			mcasp_info("McASP%d TX is not started\n", mcasp_id);
		} else {
			mcasp_stop_tx(mcasp_id);
		}
	}

	if (mcasp_unit->info.flags & RX_SUPPORT) {
		if (atomic_read(&mcasp_unit->rx_in_use) != 1) {
			mcasp_info("McASP%d RX is not started\n", mcasp_id);
		} else {
			mcasp_stop_rx(mcasp_id);
		}
	}

	McASP_FN_OUT(0);
	return 0;
}
EXPORT_SYMBOL(mcasp_stop);

s8 mcasp_start(u8 mcasp_id)
{
	mcasp_unit_t *mcasp_unit;

	McASP_FN_IN;
	mcasp_unit = find_mcasp(mcasp_id);

	if (unlikely(mcasp_unit == NULL)) {
		McASP_FN_OUT(-1);
		return -1;
	}

	if (mcasp_unit->info.flags & TX_SUPPORT) {
		if (atomic_read(&mcasp_unit->tx_in_use) == 1) {
			mcasp_err
			    ("McASP%d TX is in use. Stop it before starting"
			     " it again\n", mcasp_id);
		} else {
			mcasp_start_tx(mcasp_id);
		}
	}

	if (mcasp_unit->info.flags & RX_SUPPORT) {
		if (atomic_read(&mcasp_unit->rx_in_use) == 1) {
			mcasp_err
			    ("McASP%d RX is in use. Stop it before starting"
			     " it again\n", mcasp_id);
		} else {
			mcasp_start_rx(mcasp_id);
		}
	}

	McASP_FN_OUT(0);
	return 0;
}
EXPORT_SYMBOL(mcasp_start);

#ifdef McASP_DEBUG
static void dump_registers(u8 mcasp_id)
{
	/* Nirmal: Need to dump registers of the intended McASP for Debug */
	mcasp_registers_t *mcasp_regs, *config;
	mcasp_unit_t *mcasp_unit;

	mcasp_unit = find_mcasp(mcasp_id);

	if (mcasp_unit == NULL) {
		McASP_FN_OUT(0);
		return;		/* Unlikely */
	}

	config = &mcasp_unit->config;

	mcasp_regs = (mcasp_registers_t *) mcasp_unit->info.reg_base;

	printk(KERN_DEBUG "\nGBL CTL 	: %08x", mcasp_regs->gblctl);

	printk(KERN_DEBUG "\nPFUNC 	: %08x", mcasp_regs->pfunc);
	printk(KERN_DEBUG "\nPDIR 		: %08x", mcasp_regs->pdir);
	printk(KERN_DEBUG "\nAMUTE 	: %08x", mcasp_regs->amute);
	printk(KERN_DEBUG "\nDLB CTL 	: %08x", mcasp_regs->lbctl);
	printk(KERN_DEBUG "\nDIT CTL 	: %08x", mcasp_regs->txditctl);
	printk(KERN_DEBUG "\nRXMASK 	: %08x", mcasp_regs->rxmask);
	printk(KERN_DEBUG "\nRX FMT 	: %08x", mcasp_regs->rxfmt);
	printk(KERN_DEBUG "\nRX FMCTL 	: %08x", mcasp_regs->rxfmctl);
	printk(KERN_DEBUG "\nACLKR CTL 	: %08x", mcasp_regs->aclkrctl);
	printk(KERN_DEBUG "\nAHCLKR CTL 	: %08x", mcasp_regs->ahclkrctl);
	printk(KERN_DEBUG "\nRX TDM 	: %08x", mcasp_regs->rxtdm);
	printk(KERN_DEBUG "\nEVTR CTL	: %08x", mcasp_regs->evtctlr);
	printk(KERN_DEBUG "\nRX STAT 	: %08x", mcasp_regs->rxstat);
	printk(KERN_DEBUG "\nRXTDM SLOT 	: %08x", mcasp_regs->rxtdmslot);
	printk(KERN_DEBUG "\nRX CLKCHK 	: %08x", mcasp_regs->rxclkchk);
	printk(KERN_DEBUG "\nREVTCTL 	: %08x", mcasp_regs->revtctl);

	printk(KERN_DEBUG "\nTX MASK 	: %08x", mcasp_regs->txmask);
	printk(KERN_DEBUG "\nTX FMT 	: %08x", mcasp_regs->txfmt);
	printk(KERN_DEBUG "\nTX FMCTL 	: %08x", mcasp_regs->txfmctl);
	printk(KERN_DEBUG "\nACLKXCTL 	: %08x", mcasp_regs->aclkxctl);
	printk(KERN_DEBUG "\nAHCLKXTL 	: %08x", mcasp_regs->ahclkxctl);
	printk(KERN_DEBUG "\nTXTDM 	: %08x", mcasp_regs->txtdm);
	printk(KERN_DEBUG "\nEVTX CTL	: %08x", mcasp_regs->evtctlx);
	printk(KERN_DEBUG "\nTX STATL 	: %08x", mcasp_regs->txstat);
	printk(KERN_DEBUG "\nTX TDMSLOT 	: %08x", mcasp_regs->txtdmslot);
	printk(KERN_DEBUG "\nTX CLKCHK 	: %08x", mcasp_regs->txclkchk);
}
#endif				/* MCASP_DEBUG */

static inline void __mcasp_configure_regs(u8 mcasp_id)
{
	mcasp_registers_t *mcasp_regs, *config;
	mcasp_unit_t *mcasp_unit;
	u32 val = 0;

	McASP_FN_IN;
	mcasp_unit = find_mcasp(mcasp_id);

	if (mcasp_unit == NULL) {
		McASP_FN_OUT(0);
		return;		/* Unlikely */
	}

	config = &mcasp_unit->config;

	__mcasp_set_pinmux();

	if (__mcasp_set_lpsc(mcasp_id) < 0) {
		mcasp_err("Error setting LPSC domains\n");
	}

	mcasp_regs = (mcasp_registers_t *) mcasp_unit->info.reg_base;

	mcasp_set_reg((u32) & mcasp_regs->gblctl, 0x00000000);

	/* Reset */
	val = ~(RXCLKRST | RXHCLKRST | RXSERCLR | RXSMRST | RXFSRST | TXCLKRST |
		TXHCLKRST | TXSERCLR | TXSMRST | TXFSRST);
	mcasp_regs->gblctl = val;

	mcasp_regs->pwremumgt = config->pwremumgt;
	mcasp_regs->pfunc = config->pfunc;
	mcasp_regs->pdir = config->pdir;
	mcasp_regs->amute = config->amute;
	mcasp_regs->lbctl = config->lbctl;
	mcasp_regs->txditctl = config->txditctl;
	mcasp_regs->rxmask = config->rxmask;
	mcasp_regs->rxfmt = config->rxfmt;
	mcasp_regs->rxfmctl = config->rxfmctl;
	mcasp_regs->aclkrctl = config->aclkrctl;
	mcasp_regs->ahclkrctl = config->ahclkrctl;
	mcasp_regs->rxtdm = config->rxtdm;
	mcasp_regs->evtctlr = config->evtctlr;
	mcasp_regs->rxstat = config->rxstat;
	mcasp_regs->rxtdmslot = config->rxtdmslot;
	mcasp_regs->rxclkchk = config->rxclkchk;
	mcasp_regs->revtctl = config->revtctl;

	mcasp_regs->txmask = config->txmask;
	mcasp_regs->txfmt = config->txfmt;
	mcasp_regs->txfmctl = config->txfmctl;
	mcasp_regs->aclkxctl = config->aclkxctl;
	mcasp_regs->ahclkxctl = config->ahclkxctl;
	mcasp_regs->txtdm = config->txtdm;
	mcasp_regs->evtctlx = config->evtctlx;
	mcasp_regs->txstat = config->txstat;
	mcasp_regs->txtdmslot = config->txtdmslot;
	mcasp_regs->txclkchk = config->txclkchk;
	mcasp_regs->xevtctl = config->xevtctl;

	for (val = 0; val < mcasp_unit->info.serializer_count; val++) {
		mcasp_regs->xrsrctl[val] = config->xrsrctl[val];
	}

	/* Is there a need to clear the tx and rx buf */
	for (val = 0; val < mcasp_unit->info.serializer_count; val++) {
		mcasp_regs->txbuf[val] = 0x00000000;
	}

	for (val = 0; val < mcasp_unit->info.serializer_count; val++) {
		mcasp_regs->rxbuf[val] = 0x00000000;
	}
#ifdef McASP_DEBUG
	/* Nirmal: dump registers */
	dump_registers(mcasp_id);
#endif
	McASP_FN_OUT(0);
}

s32 mcasp_get_sample_rate(u8 mcasp_id)
{
	mcasp_unit_t *mcasp_unit;

	mcasp_unit = find_mcasp(mcasp_id);

	if (mcasp_unit == NULL) {
		McASP_FN_OUT(0);
		return -1;	/* Unlikely */
	}

	return mcasp_unit->sample_rate;
}

s32 mcasp_set_sample_rate(u8 mcasp_id, u32 sample_rate)
{
	u32 ret_val = 0, i = 0;
	mcasp_unit_t *mcasp_unit;
	mcasp_registers_t *config;

	mcasp_unit = find_mcasp(mcasp_id);

	if (mcasp_unit == NULL) {
		McASP_FN_OUT(0);
		return -1;	/* Unlikely */
	}

	config = &mcasp_unit->config;

	for (i = 0; i < MAX_SAMPLE_RATES; i++) {
		if (sample_rate == sample_rates_supported[i]) {
			switch (sample_rate) {
			case 22050:
				config->aclkxctl = 0x000000A0;
				config->ahclkxctl = 0x0008021;
				config->txfmctl = 0x00000002;

				config->aclkrctl = 0x000000A0;
				config->ahclkrctl = 0x0008010;
				config->rxfmctl = 0x00000002;

				config->pdir |= (0x7 << 26);
				break;

			case 44100:
				config->aclkxctl = 0x000000A0;
				config->ahclkxctl = 0x0008010;
				config->txfmctl = 0x00000002;

				config->aclkrctl = 0x000000A0;
				config->ahclkrctl = 0x0008010;
				config->rxfmctl = 0x00000002;

				config->pdir |= (0x7 << 26);
				break;
			}
			__mcasp_configure_regs(mcasp_id);
			ret_val = sample_rates_supported[i];
			break;
		}
		ret_val = mcasp_unit->sample_rate;
	}

	mcasp_unit->sample_rate = ret_val;
	return ret_val;
}

s8 mcasp_configure(u8 mcasp_id, void *cfg)
{
	mcasp_registers_t mcasp_config;
	audio_config_t *audio_cfg = (audio_config_t *) cfg;
	u8 cnt = 0;
	mcasp_unit_t *mcasp_unit;

	mcasp_unit = find_mcasp(mcasp_id);

	if (mcasp_unit == NULL) {
		McASP_FN_OUT(0);
		return -1;	/* Unlikely */
	}

	memset(&mcasp_config, 0, sizeof(mcasp_registers_t));

	/* Default configuration */
	mcasp_config.pwremumgt = 0x00000001;
	mcasp_config.pfunc = 0x00000000;
	mcasp_config.pdir = 0x00000001;
	mcasp_config.amute = 0x00000000;
	mcasp_config.lbctl = 0x00000000;
	mcasp_config.txditctl = 0x00000000;
	mcasp_config.rxmask = 0xFFFFFFFF;
	mcasp_config.rxfmt = 0x000080f0;
	mcasp_config.rxfmctl = 0x00000000;
	mcasp_config.aclkrctl = 0x00000000;
	mcasp_config.ahclkrctl = 0x00008000;
	mcasp_config.rxtdm = 0x00000000;
	mcasp_config.evtctlr = 0x00000000;
	mcasp_config.rxstat = 0xFFFFFFFF;
	mcasp_config.rxtdmslot = 0x00000000;
	mcasp_config.rxclkchk = 0x00000000;
	mcasp_config.revtctl = 0x00000000;
	mcasp_config.txmask = 0xFFFFFFFF;
	mcasp_config.txfmt = 0x000080f0;
	mcasp_config.txfmctl = 0x00000000;
	mcasp_config.aclkxctl = 0x00000000;
	mcasp_config.ahclkxctl = 0x00008000;
	mcasp_config.txtdm = 0x00000000;
	mcasp_config.evtctlx = 0x00000000;
	mcasp_config.txstat = 0xFFFFFFFF;
	mcasp_config.txtdmslot = 0x00000000;
	mcasp_config.txclkchk = 0x00000000;
	mcasp_config.xevtctl = 0x00000000;
	mcasp_config.xrsrctl[0] = 0x1;
	mcasp_config.xrsrctl[1] = 0x2;

	if (audio_cfg->mode == CODEC_IS_MASTER) {
		mcasp_config.aclkrctl = 0x00000000;
		mcasp_config.ahclkrctl = 0x00008000;
		mcasp_config.rxfmctl = 0x00000000;

		mcasp_config.aclkxctl = 0x00000000;
		mcasp_config.ahclkxctl = 0x00008000;
		mcasp_config.txfmctl = 0x00000000;

		mcasp_config.pdir &= ~(0x3F << 26);
	} else {
		/* Codec is Slave */
		printk(KERN_INFO "Configuring the McASP as Master\n");

		mcasp_config.aclkxctl = 0x000000A0;
		mcasp_config.ahclkxctl = 0x0008010;
		mcasp_config.txfmctl = 0x00000002;

		mcasp_config.aclkrctl = 0x000000A0;
		mcasp_config.ahclkrctl = 0x0008010;
		mcasp_config.rxfmctl = 0x00000002;

		mcasp_config.pdir |= (0x7 << 26);
	}

	if (audio_cfg->channel_size < 8 || audio_cfg->channel_size > 32) {
		printk(KERN_EMERG "Invalid channel size\n");
		return -1;
	}

	switch (audio_cfg->channel_size) {
	case 8:
		mcasp_config.rxfmt |= (0x03 << 4);
		mcasp_config.txfmt |= (0x03 << 4);
		break;

	case 12:
		mcasp_config.rxfmt |= (0x05 << 4);
		mcasp_config.txfmt |= (0x05 << 4);
		break;

	case 16:
		mcasp_config.rxfmt |= (0x07 << 4);
		mcasp_config.txfmt |= (0x07 << 4);
		break;

	case 20:
		mcasp_config.rxfmt |= (0x09 << 4);
		mcasp_config.txfmt |= (0x09 << 4);
		break;

	case 24:
		mcasp_config.rxfmt |= (0x0B << 4);
		mcasp_config.txfmt |= (0x0B << 4);
		break;

	case 28:
		mcasp_config.rxfmt |= (0x0D << 4);
		mcasp_config.txfmt |= (0x0D << 4);
		break;

	case 32:
		mcasp_config.rxfmt |= (0x0F << 4);
		mcasp_config.txfmt |= (0x0F << 4);
		break;

	default:
		return -1;
	}

	if (audio_cfg->loopback == 1) {
		/* Enable Loopback */
		mcasp_config.lbctl |= audio_cfg->loopback;
	}

	if (audio_cfg->amute == 1) {
		/* Enable AMUTE. Drive high if error detected. */
		mcasp_config.amute |= audio_cfg->amute;
		mcasp_config.pdir |= (0x1 << 25);	/* ???? */
	}

	/* NK: slots should not be MORE than 384 */
	if (audio_cfg->tdm_slots < 0 || audio_cfg->tdm_slots > 384) {
		printk(KERN_EMERG "Invalid number of slots\n");
		return -1;
	}

	mcasp_config.rxfmctl |= (audio_cfg->tdm_slots << 7);
	mcasp_config.txfmctl |= (audio_cfg->tdm_slots << 7);

	/* NK: This should not be done for the DIT */
	if (audio_cfg->tdm_slots < 384) {
		for (cnt = 0; cnt < audio_cfg->tdm_slots; cnt++) {
			mcasp_config.txtdm |= (1 << cnt);
			mcasp_config.rxtdm |= (1 << cnt);
		}
	}
	if (audio_cfg->tdm_slots == 0) {
		/* Receive frame sync duration is 1 bit
		 * for burst mode*/
		mcasp_config.rxfmctl &= 0xFFFFFFEF;
		mcasp_config.txfmctl &= 0xFFFFFFEF;
	} else {
		mcasp_config.rxfmctl |= 0x10;
		mcasp_config.txfmctl |= 0x10;
	}

	for (cnt = 0; cnt < audio_cfg->serializer_count; cnt++) {
		switch (audio_cfg->serializer_mode[cnt]) {
		case 0:
			mcasp_config.xrsrctl[cnt] |= SERIALIZER_IS_INACTIVE;
			break;

		case 1:
			mcasp_config.xrsrctl[cnt] |= SERIALIZER_IS_TX;
			mcasp_config.pdir |= (1 << cnt);
			break;

		case 2:
			mcasp_config.xrsrctl[cnt] |= SERIALIZER_IS_RX;
			mcasp_config.pdir &= ~(1 << cnt);
			break;

		default:
			printk(KERN_EMERG "Invalid mode for the serializers\n");
			return -1;
		}
	}
	/* Nirmal: Configure the McASP for the DIT */
	/* For want of any other suitable param,I am using tdm_slots */
	if (audio_cfg->tdm_slots == 384) {

		printk(KERN_INFO "Configuring the McASP %d for DIT \n",
		       mcasp_id);

		mcasp_config.xrsrctl[0] = 0x1;
		mcasp_config.xrsrctl[1] = 0x0;
		mcasp_config.xrsrctl[2] = 0x0;
		mcasp_config.xrsrctl[3] = 0x0;

		/* Set the PDIR for Serialiser as output */
		mcasp_config.pdir = 0x10000001;

		/* All PINS as McASP */
		mcasp_config.pfunc = 0x00000000;

		/* TXMASK for 24 bits */
		mcasp_config.txmask = 0x00FFFFFF;

		/* Set the TX format : 24 bit right rotation, 32 bit slot, Pad 0
		   and LSB first */
		mcasp_config.txfmt = 0x000000F6;

		/* Set TX frame synch : DIT Mode, 1 bit width, internal, rising
		   edge */
		mcasp_config.txfmctl = 0x0000C002;

		/* Set the TX tdm : for all the slots */
		mcasp_config.txtdm = 0xFFFFFFFF;

		/* Set the TX clock controls : div = 1 and internal */
		mcasp_config.aclkxctl = 0x00000060;

		/* Set the TX high clock with approriate divisor */
		/* Set as per the required rate */
		switch (audio_cfg->sample_rate) {

			/* Both 44100 and 48000 have the same clock setting. */
		case 44100:
			mcasp_config.ahclkxctl = 0x00008003;
			break;

		case 48000:
			mcasp_config.ahclkxctl = 0x00008003;
			break;

		default:
			printk(KERN_ERR "\n Sampling rate %d not supported\n",
			       audio_cfg->sample_rate);
			printk(KERN_ERR "\n Switching to default Sampling rate \
						48000\n");
			mcasp_config.ahclkxctl = 0x00008003;
			audio_cfg->sample_rate = 48000;
			break;
		}

		mcasp_unit->sample_rate = audio_cfg->sample_rate;
		/* Enable the DIT */
		mcasp_config.txditctl = 0x00000001;	/* 1 */

	} else {
		/* Receive frame sync duration is 1 bit
		 * for burst mode*/
		mcasp_config.rxfmctl &= 0xFFFFFFEF;
		mcasp_config.txfmctl &= 0xFFFFFFEF;
	}
	/* NK : up to this */
	if (mcasp_reg_configure(mcasp_id, &mcasp_config) < 0) {
		mcasp_err("Unable to configure McASP\n");
		return -1;
	}

	return 0;
}

s8 mcasp_reg_configure(u8 mcasp_id, void *cfg)
{
	mcasp_unit_t *mcasp_unit = NULL;
	mcasp_registers_t *mcasp_config = (mcasp_registers_t *) cfg;

	McASP_FN_IN;
	mcasp_unit = find_mcasp(mcasp_id);

	if (mcasp_unit == NULL) {
		McASP_FN_OUT(-1);
		return -1;
	}

	if (mcasp_get(mcasp_id) < 0) {
		mcasp_err("McASP%d is in use. Stop it before configuring\n",
			  mcasp_id);
		McASP_FN_OUT(-1);
		return -1;
	}

	memcpy(&mcasp_unit->config, mcasp_config, sizeof(mcasp_registers_t));
	__mcasp_configure_regs(mcasp_id);
	atomic_set(&mcasp_unit->init, 1);
	McASP_FN_OUT(0);
	return 0;
}
EXPORT_SYMBOL(mcasp_reg_configure);

void *mcasp_get_info(u8 mcasp_id)
{
	mcasp_unit_t *mcasp_unit = NULL;

	McASP_FN_IN;
	mcasp_unit = find_mcasp(mcasp_id);

	if (mcasp_unit == NULL) {
		McASP_FN_OUT(0);
		return NULL;
	}

	McASP_FN_OUT_ADDR(&mcasp_unit->info);
	return (void *)&mcasp_unit->info;
}
EXPORT_SYMBOL(mcasp_get_info);

void *mcasp_get_config(u8 mcasp_id)
{
	mcasp_unit_t *mcasp_unit = NULL;

	McASP_FN_IN;
	mcasp_unit = find_mcasp(mcasp_id);

	if (mcasp_unit == NULL) {
		McASP_FN_OUT(0);
		return NULL;
	}

	if (atomic_read(&mcasp_unit->init) != 1) {
		mcasp_info("McASP%d is not configured\n", mcasp_id);
		McASP_FN_OUT(0);
		return NULL;
	}

	McASP_FN_OUT_ADDR(&mcasp_unit->config);
	return (void *)&mcasp_unit->config;
}
EXPORT_SYMBOL(mcasp_get_config);


s8 mcasp_unconfigure(u8 mcasp_id)
{
	mcasp_unit_t *mcasp_unit = NULL;

	McASP_FN_IN;
	mcasp_unit = find_mcasp(mcasp_id);

	if (mcasp_unit == NULL) {
		McASP_FN_OUT(-1);
		return -1;
	}

	if (atomic_read(&mcasp_unit->init) != 1) {
		mcasp_info("McASP%d was not configured\n", mcasp_id);
		McASP_FN_OUT(-1);
		return 1;
	}

	DISPLAY_MCASP_IN_USE_WARNING(mcasp_id);

	if (mcasp_stop(mcasp_id) < 0) {
		McASP_FN_OUT(-1);
		return -1;
	}

	memset(&mcasp_unit->config, 0, sizeof(mcasp_registers_t));
	atomic_set(&mcasp_unit->init, 0);

	McASP_FN_OUT(0);
	return 0;
}
EXPORT_SYMBOL(mcasp_unconfigure);

static inline u8 check_for_duplicate_id(u8 mcasp_count, mcasp_info_t
					* mcasp_info)
{
	u8 *arr = NULL;
	u8 i = 0, j = 0;
	mcasp_info_t *info = mcasp_info;

	McASP_FN_IN;
	arr = (u8 *) kmalloc(mcasp_count, GFP_KERNEL);

	if (arr == NULL) {
		mcasp_err("Unable to allocate memory\n");
		McASP_FN_OUT(1);
		return 1;
	}

	for (i = 0; i < mcasp_count; i++) {
		arr[i] = info->id;
		info++;
	}

	for (i = 0; i < mcasp_count - 1; i++) {
		for (j = i; j < mcasp_count - 1; j++) {
			if (arr[i] == arr[j + 1]) {
				kfree(arr);
				McASP_FN_OUT(1);
				return 1;
			}
		}
	}

	kfree(arr);
	McASP_FN_OUT(0);
	return 0;
}

s8 mcasp_dev_init(u8 mcasp_count, void *info)
{
	u8 i = 0;
	mcasp_info_t *mcasp_info = (mcasp_info_t *) info;

	McASP_FN_IN;
	if (unlikely(mcasp_count <= 0)) {
		mcasp_err("Invalid count of McASP units\n");
		McASP_FN_OUT(-EINVAL);
		return -EINVAL;
	}

	if (check_for_duplicate_id(mcasp_count, mcasp_info)) {
		mcasp_err("McASP ID's have to be unique\n");
		McASP_FN_OUT(-1);
		return -1;
	}

	if (mcasp != NULL) {
		mcasp_err("\n\nMcASP Device has already been initialized\n\n");
		McASP_FN_OUT(-1);
		return -1;
	}

	mcasp = (mcasp_device_t *) kmalloc(sizeof(mcasp_device_t), GFP_KERNEL);

	if (unlikely(mcasp == NULL)) {
		mcasp_err("Unable to allocate memory.\n");
		McASP_FN_OUT(-1);
		return -1;
	}

	mcasp->count = mcasp_count;
	mcasp->mcasp_list = NULL;

	do {
		mcasp_unit_t *mcasp_unit = NULL;

		mcasp_unit = (mcasp_unit_t *) kmalloc(sizeof(mcasp_unit_t),
						      GFP_KERNEL);

		if (unlikely(mcasp_unit == NULL)) {
			mcasp_err("Unable to allocate memory.\n");
			McASP_FN_OUT(-1);
			return -1;
		}

		memset(mcasp_unit, 0, sizeof(mcasp_unit_t));

		INIT_LIST_HEAD(&mcasp_unit->list);
		memcpy(&(mcasp_unit->info), &mcasp_info[i],
		       sizeof(mcasp_unit_t));
		memset(&mcasp_unit->config, 0, sizeof(mcasp_registers_t));
		mcasp_unit->clock = NULL;
		mcasp_unit->sample_rate =
		    sample_rates_supported[MAX_SAMPLE_RATES - 1];
		atomic_set(&mcasp_unit->init, 0);
		atomic_set(&mcasp_unit->tx_in_use, 0);
		atomic_set(&mcasp_unit->rx_in_use, 0);

		if (unlikely(mcasp->mcasp_list == NULL)) {
			mcasp->mcasp_list = &mcasp_unit->list;
		} else {
			list_add(&mcasp_unit->list, mcasp->mcasp_list);
		}
	} while (++i < mcasp_count);

	McASP_FN_OUT(0);
	return 0;
}
EXPORT_SYMBOL(mcasp_dev_init);

s8 mcasp_dev_deinit(u8 mcasp_id)
{
	mcasp_unit_t *mcasp_unit = NULL;

	McASP_FN_IN;
	if (mcasp == NULL) {
		mcasp_err("McASP Device has not been initialized\n");
		McASP_FN_OUT(-1);
		return -1;
	}

	mcasp_unit = find_mcasp(mcasp_id);

	if (mcasp_unit == NULL) {
		McASP_FN_OUT(-1);
		return -1;
	}

	DISPLAY_MCASP_IN_USE_WARNING(mcasp_id);

	if (mcasp_stop(mcasp_id) < 0) {
		McASP_FN_OUT(-1);
		return -1;
	}

	if (mcasp_unconfigure(mcasp_id) < 0) {
		McASP_FN_OUT(-1);
		return -1;
	}

	if (__mcasp_unit_count() == 1) {
		mcasp->mcasp_list = NULL;
		kfree(mcasp);
		mcasp = NULL;
	} else {
		if (mcasp->mcasp_list == &mcasp_unit->list) {
			mcasp->mcasp_list = mcasp_unit->list.next;
		}
	}

	list_del(&mcasp_unit->list);
	clk_unuse(mcasp_unit->clock);
	clk_disable(mcasp_unit->clock);
	kfree(mcasp_unit);

	McASP_FN_OUT(0);
	return 0;
}
EXPORT_SYMBOL(mcasp_dev_deinit);

static int __init mcasp_init(void)
{
	McASP_FN_IN;
	McASP_FN_OUT(0);
	return 0;
}

static void __exit mcasp_exit(void)
{
	McASP_FN_IN;
	if (mcasp != NULL) {
		u8 cnt = __mcasp_unit_count();
		u8 *arr = NULL, i = 0;
		struct list_head *temp = NULL;
		mcasp_unit_t *mcasp_unit = NULL;

		arr = (u8 *) kmalloc(cnt, GFP_KERNEL);

		if (arr == NULL) {
			mcasp_err("Unable to allocate memory\n");
			goto failure;
		}

		list_for_each(temp, mcasp->mcasp_list) {
			mcasp_unit =
			    list_entry(temp, struct __mcasp_unit, list);
			arr[i] = mcasp_unit->info.id;
			i++;
		}

		for (i = 0; i < cnt; i++) {
			mcasp_dev_deinit(arr[i]);
		}

		kfree(arr);
	}

failure:
	McASP_FN_OUT(0);
}

module_init(mcasp_init);
module_exit(mcasp_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("Multichannel Audio Serial Port (McASP) Driver");
MODULE_LICENSE("GPL");
