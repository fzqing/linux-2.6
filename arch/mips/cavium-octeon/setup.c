/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2004, 2005 Cavium Networks
 */
#include <linux/config.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/serial.h>
#include <linux/types.h>
#include <linux/string.h>	/* for memset */
#include <linux/console.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <asm/time.h>
#include <linux/serial_core.h>
#include <linux/string.h>
#include <linux/hrtime.h>
#include <linux/kgdb.h>
#include <linux/mtd/physmap.h>

#include <asm/reboot.h>
#include <asm/io.h>
#include <asm/time.h>
#include <asm/processor.h>
#include <asm/reboot.h>
#include <asm/system.h>
#include <asm/irq_cpu.h>
#include <asm/mipsregs.h>
#include <asm/bootinfo.h>
#include <asm/gdb-stub.h>
#include <octeon-app-init.h>
#include <hal.h>

static uint64_t  MAX_MEMORY = ((512ull) << 20);

//#define ECC_REPORT_SINGLE_BIT_ERRORS
#define EARLY_BOOT_UART     (0)	/* 0 or 1 */
#define CAVIUM_UART_IRQ     42

#ifdef ECC_REPORT_SINGLE_BIT_ERRORS
#warning: **************************************************************
#warning: **************************************************************
#warning: ************* Single bit ECC errors are reported *************
#warning: **************************************************************
#warning: **************************************************************
#endif

extern void octeon_user_io_init(void);
extern asmlinkage void octeon_handle_irq(void);
static uint64_t CYCLES_PER_JIFFY;

/**
 * Return the version string. Generated from CVS tags. Turns
 * NAME_V_V_V_build_B into "V.V.V, build B". If the tag isn't of
 * this format then the tag is returned. If there isn't a tag
 * then the build date is returned as "Internal __DATE__".
 *
 * @return Version string
 */
static inline const char *get_version(void)
{
	static char version[80];
	const char *cvs_tag = "$Name:  $";

	if (cvs_tag[7] == ' ') {
		snprintf(version, sizeof(version), "Internal %s", __DATE__);
	} else {
		char *major = NULL;
		char *minor1 = NULL;
		char *minor2 = NULL;
		char *build = NULL;
		char *buildnum = NULL;
		char *end = NULL;
		char buf[80];

		strncpy(buf, cvs_tag, sizeof(buf));
		buf[sizeof(buf) - 1] = 0;

		major = strchr(buf, '_');
		if (major) {
			major++;
			minor1 = strchr(major, '_');
			if (minor1) {
				*minor1 = 0;
				minor1++;
				minor2 = strchr(minor1, '_');
				if (minor2) {
					*minor2 = 0;
					minor2++;
					build = strchr(minor2, '_');
					if (build) {
						*build = 0;
						build++;
						buildnum = strchr(build, '_');
						if (buildnum) {
							*buildnum = 0;
							buildnum++;
							end =
							    strchr(buildnum,
								   ' ');
							if (end)
								*end = 0;
						}
					}
				}
			}
		}

		if (major && minor1 && minor2 && build && buildnum
		    && (strcmp(build, "build") == 0))
			snprintf(version, sizeof(version), "%s.%s.%s, build %s",
				 major, minor1, minor2, buildnum);
		else
			snprintf(version, sizeof(version), "%s", cvs_tag);
	}

	return version;
}

/**
 * Reboot Octeon
 *
 * @param command Command to pass to the bootloader. Currently ignored.
 */
static void octeon_restart(char *command)
{
	mb();
	while (1)
		octeon_write_csr(OCTEON_CIU_SOFT_RST, 1);
}

/**
 * Permanently stop a core.
 *
 * @param arg
 */
static void octeon_kill_core(void *arg)
{
	mb();
}

/**
 * Halt the system
 */
static void octeon_halt(void)
{
	smp_call_function(octeon_kill_core, NULL, 0, 0);
	octeon_kill_core(NULL);
}

/**
 * Turn power off. Currently just halts since we can't shut
 * off power.
 */
static void octeon_power_off(void)
{
	octeon_halt();
}

#ifndef CONFIG_CPU_TIMER

/**
 * This variable is used to correct for a offset on the clock
 * when Linux boots.
 */
static int64_t octeon_hpt_correction = 0;

/**
 * Read the Octeon high performance counter
 *
 * @return The counter value. For some brain dead reason, the kernel
 *         uses a 32bit number here.
 */
static unsigned int octeon_hpt_read(void)
{
	if (octeon_is_pass1()) {
		/* Pass 1 doesn't have a global cycle count */
		return 0;
	} else {
		int64_t cycles = octeon_read_csr(OCTEON_IPD_CLK_COUNT);
		return cycles - octeon_hpt_correction;
	}
}

/**
 * Initialize the high performance counter.
 *
 * @param count  Offset to apply to the counter
 */
static void octeon_hpt_init(unsigned int count)
{
	octeon_hpt_correction += count;
}

#endif

/**
 * Acknowledge a timer tick. We don't use the standard Mips
 * one because it confuses the timer ticks and the HPT clock.
 */
static void octeon_timer_ack(void)
{
	uint32_t next_compare, count;
	next_compare = read_c0_compare() + CYCLES_PER_JIFFY;
	write_c0_compare(next_compare);
	count = read_c0_count();
	if ((count - next_compare) < 0x7fffffff) {
		next_compare = count + CYCLES_PER_JIFFY;
		write_c0_compare(next_compare);
	}
}

/**
 * Interrupt entry point for timer ticks
 *
 * @param irq
 * @param dev_id
 * @param regs
 * @return
 */
static irqreturn_t octeon_main_timer_interrupt(int irq, void *dev_id,
					       struct pt_regs *regs)
{
	if (read_c0_count() - read_c0_compare() >= 0) {
		if (smp_processor_id() == 0)
			timer_interrupt(irq, NULL, regs);
		else {
			octeon_timer_ack();
			local_timer_interrupt(irq, NULL, regs);
#ifdef CONFIG_HIGH_RES_TIMERS
			timer_interrupt(irq, NULL, regs);
#endif
		}
	}
	return IRQ_HANDLED;
}

/**
 * Setup the first cores timer interrupt
 *
 * @param irq
 * @return
 */
static void __init octeon_timer_setup(struct irqaction *irq)
{
	irq->handler = octeon_main_timer_interrupt;
	irq->flags |= SA_SHIRQ;
	setup_irq(7, irq);

	write_c0_compare(read_c0_count() + CYCLES_PER_JIFFY);
}

/**
 * Handle all the error condition interrupts that might occur.
 *
 * @param cpl
 * @param dev_id
 * @param regs
 * @return
 */
static irqreturn_t octeon_ecc_interrupt(int cpl, void *dev_id,
					struct pt_regs *regs)
{
	static uint64_t single_bit_errors = 0;
	static uint64_t double_bit_errors = 0;
	static uint64_t l2t_single_bit_errors = 0;
	static uint64_t l2t_double_bit_errors = 0;
	static uint64_t l2d_single_bit_errors = 0;
	static uint64_t l2d_double_bit_errors = 0;
	static uint64_t pow_single_bit_errors = 0;
	static uint64_t pow_double_bit_errors = 0;
	irqreturn_t result = IRQ_NONE;
	uint64_t l2t_err, l2d_err, mem_cfg0, pow_err, iob_err, ipd_err, zip_err,
	    pko_err, tim_err, fpa_err;

	/* Get the ECC status from LMC config zero */
	mem_cfg0 = octeon_read_csr(OCTEON_LMC_MEM_CFG0);
	/* Write out the same value to clear the ECC error bits */
	octeon_write_csr(OCTEON_LMC_MEM_CFG0, mem_cfg0);

	mem_cfg0 = (mem_cfg0 >> 21) & 0xff;
	if (mem_cfg0 & 0x1)
		single_bit_errors++;
	if (mem_cfg0 & 0x2)
		single_bit_errors++;
	if (mem_cfg0 & 0x4)
		single_bit_errors++;
	if (mem_cfg0 & 0x8)
		single_bit_errors++;
	if (mem_cfg0 & 0x10)
		double_bit_errors++;
	if (mem_cfg0 & 0x20)
		double_bit_errors++;
	if (mem_cfg0 & 0x40)
		double_bit_errors++;
	if (mem_cfg0 & 0x80)
		double_bit_errors++;
	if (mem_cfg0)
		result = IRQ_HANDLED;

#ifdef ECC_REPORT_SINGLE_BIT_ERRORS
	if (mem_cfg0)
#else
	if (mem_cfg0 & 0xf0)
#endif
	{
		octeon_lmc_fadr_t fadr;
		fadr.u64 = octeon_read_csr(OCTEON_LMC_FADR);
		printk("\nECC: %lu Single bit errors, %lu Double bit errors\n"
		       "ECC:\tFailing dimm:   %u\n"
		       "ECC:\tFailing rank:   %u\n"
		       "ECC:\tFailing bank:   %u\n"
		       "ECC:\tFailing row:    0x%x\n"
		       "ECC:\tFailing column: 0x%x\n",
		       single_bit_errors, double_bit_errors, fadr.s.fdimm,
		       fadr.s.fbunk, fadr.s.fbank, fadr.s.frow, fadr.s.fcol);
	}

	/* Get the ECC status from L2T_ERR */
	l2t_err = octeon_read_csr(OCTEON_L2T_ERR);
	/* Write out the same value to clear the ECC error bits */
	octeon_write_csr(OCTEON_L2T_ERR, l2t_err);

	l2t_err = (l2t_err >> 3) & 0x3;
	if (l2t_err & 0x1)
		l2t_single_bit_errors++;
	if (l2t_err & 0x2)
		l2t_double_bit_errors++;
	if (l2t_err)
		result = IRQ_HANDLED;

#ifdef ECC_REPORT_SINGLE_BIT_ERRORS
	if (l2t_err)
#else
	if (l2t_err & 0x2)
#endif
	{
		printk
		    ("\nECC: L2T %lu Single bit errors, %lu Double bit errors\n",
		     l2t_single_bit_errors, l2t_double_bit_errors);
	}

	/* Get the ECC status from L2D_ERR */
	l2d_err = octeon_read_csr(OCTEON_L2D_ERR);
	/* Write out the same value to clear the ECC error bits */
	octeon_write_csr(OCTEON_L2D_ERR, l2d_err);

	l2d_err = (l2d_err >> 3) & 0x3;
	if (l2d_err & 0x1)
		l2d_single_bit_errors++;
	if (l2d_err & 0x2)
		l2d_double_bit_errors++;
	if (l2d_err)
		result = IRQ_HANDLED;

#ifdef ECC_REPORT_SINGLE_BIT_ERRORS
	if (l2d_err)
#else
	if (l2d_err & 0x2)
#endif
	{
		printk
		    ("\nECC: L2D %lu Single bit errors, %lu Double bit errors\n",
		     l2d_single_bit_errors, l2d_double_bit_errors);
	}

	/* Get the ECC status from POW */
	pow_err = octeon_read_csr(OCTEON_POW_ECC_ERR);
	/* Write out the same value to clear the ECC error bits */
	octeon_write_csr(OCTEON_POW_ECC_ERR, pow_err);

	pow_err = pow_err & 0x3;
	if (pow_err & 0x1)
		pow_single_bit_errors++;
	if (pow_err & 0x2)
		pow_double_bit_errors++;
	if (pow_err)
		result = IRQ_HANDLED;

#ifdef ECC_REPORT_SINGLE_BIT_ERRORS
	if (pow_err)
#else
	if (pow_err & 0x2)
#endif
	{
		printk
		    ("\nECC: POW %lu Single bit errors, %lu Double bit errors\n",
		     pow_single_bit_errors, pow_double_bit_errors);
	}

	if (!octeon_is_pass1()) {
		/* Check for IOB SOP and EOP errors */
		iob_err = octeon_read_csr(OCTEON_IOB_INT_SUM);
		if (iob_err) {
			uint64_t port;
			octeon_write_csr(OCTEON_IOB_INT_SUM, iob_err);
			port = octeon_read_csr(OCTEON_IOB_PKT_ERR);
			if (iob_err & 0x1)
				printk("\nIOB: Port %lu SOP error\n", port);
			if (iob_err & 0x2)
				printk("\nIOB: Port %lu EOP error\n", port);
			if (iob_err & 0x4)
				printk
				    ("\nIOB: Passthrough Port %lu SOP error\n",
				     port);
			if (iob_err & 0x8)
				printk
				    ("\nIOB: Passthrough Port %lu EOP error\n",
				     port);
			result = IRQ_HANDLED;
		}

		ipd_err = octeon_read_csr(OCTEON_IPD_INT_SUM);
		if (ipd_err) {
			octeon_write_csr(OCTEON_IPD_INT_SUM, ipd_err);
			if (ipd_err & 0x1)
				printk
				    ("\nIPD: PBM memory parity error [31:0]\n");
			if (ipd_err & 0x2)
				printk
				    ("\nIPD: PBM memory parity error [63:32]\n");
			if (ipd_err & 0x4)
				printk
				    ("\nIPD: PBM memory parity error [95:64]\n");
			if (ipd_err & 0x8)
				printk
				    ("\nIPD: PBM memory parity error [127:96]\n");
			if (ipd_err & 0x10)
				printk
				    ("\nIPD: Backpressure subtract with an illegal value\n");
			result = IRQ_HANDLED;
		}

		zip_err = octeon_read_csr(OCTEON_ZIP_ERROR);
		if (zip_err) {
			octeon_write_csr(OCTEON_ZIP_ERROR, zip_err);
			printk("\nZIP: Doorbell overflow\n");
			result = IRQ_HANDLED;
		}

		pko_err = octeon_read_csr(OCTEON_PKO_REG_DEBUG0);
		if (pko_err) {
			octeon_write_csr(OCTEON_PKO_REG_DEBUG0, 0);
			printk("PKO: Hardware error\n");
			result = IRQ_HANDLED;
		}

		tim_err = octeon_read_csr(OCTEON_TIM_REG_ERROR);
		if (tim_err) {
			int i;
			octeon_write_csr(OCTEON_TIM_REG_ERROR, tim_err);
			for (i = 0; i < 16; i++)
				if (tim_err & (1 << i))
					printk("TIM: Timer wheel %d error\n",
					       i);
			result = IRQ_HANDLED;
		}

		fpa_err = octeon_read_csr(OCTEON_FPA_INT_SUM);
		if (fpa_err) {
			octeon_write_csr(OCTEON_FPA_INT_SUM, fpa_err);
			if (fpa_err & 0x8000000)
				printk
				    ("FPA: Set when a Queue 7 pointer read from the stack in the L2C does not have the FPA owner ship bit set.\n");
			if (fpa_err & 0x4000000)
				printk
				    ("FPA: Set when a Queue 7 stack end tag is present and the count available is greater than than pointers present in the FPA.\n");
			if (fpa_err & 0x2000000)
				printk
				    ("FPA: Set when a Queue 7 page count available goes negative.\n");
			if (fpa_err & 0x1000000)
				printk
				    ("FPA: Set when a Queue 6 pointer read from the stack in the L2C does not have the FPA owner ship bit set.\n");
			if (fpa_err & 0x800000)
				printk
				    ("FPA: Set when a Queue 6 stack end tag is present and the count available is greater than than pointers present in the FPA.\n");
			if (fpa_err & 0x400000)
				printk
				    ("FPA: Set when a Queue 6 page count available goes negative.\n");
			if (fpa_err & 0x200000)
				printk
				    ("FPA: Set when a Queue 5 pointer read from the stack in the L2C does not have the FPA owner ship bit set.\n");
			if (fpa_err & 0x100000)
				printk
				    ("FPA: Set when a Queue 5 stack end tag is present and the count available is greater than than pointers present in the FPA.\n");
			if (fpa_err & 0x80000)
				printk
				    ("FPA: Set when a Queue 5 page count available goes negative.\n");
			if (fpa_err & 0x40000)
				printk
				    ("FPA: Set when a Queue 4 pointer read from the stack in the L2C does not have the FPA owner ship bit set.\n");
			if (fpa_err & 0x20000)
				printk
				    ("FPA: Set when a Queue 4 stack end tag is present and the count available is greater than than pointers present in the FPA.\n");
			if (fpa_err & 0x10000)
				printk
				    ("FPA: Set when a Queue 4 page count available goes negative.\n");
			if (fpa_err & 0x8000)
				printk
				    ("FPA: Set when a Queue 3 pointer read from the stack in the L2C does not have the FPA owner ship bit set.\n");
			if (fpa_err & 0x4000)
				printk
				    ("FPA: Set when a Queue 3 stack end tag is present and the count available is greater than than pointers present in the FPA.\n");
			if (fpa_err & 0x2000)
				printk
				    ("FPA: Set when a Queue 3 page count available goes negative.\n");
			if (fpa_err & 0x1000)
				printk
				    ("FPA: Set when a Queue 2 pointer read from the stack in the L2C does not have the FPA owner ship bit set.\n");
			if (fpa_err & 0x800)
				printk
				    ("FPA: Set when a Queue 2 stack end tag is present and the count available is greater than than pointers present in the FPA.\n");
			if (fpa_err & 0x400)
				printk
				    ("FPA: Set when a Queue 2 page count available goes negative.\n");
			if (fpa_err & 0x200)
				printk
				    ("FPA: Set when a Queue 1 pointer read from the stack in the L2C does not have the FPA owner ship bit set.\n");
			if (fpa_err & 0x100)
				printk
				    ("FPA: Set when a Queue 1 stack end tag is present and the count available is greater than than pointers present in the FPA.\n");
			if (fpa_err & 0x80)
				printk
				    ("FPA: Set when a Queue 1 page count available goes negative.\n");
			if (fpa_err & 0x40)
				printk
				    ("FPA: Set when a Queue 0 pointer read from the stack in the L2C does not have the FPA owner ship bit set.\n");
			if (fpa_err & 0x20)
				printk
				    ("FPA: Set when a Queue 0 stack end tag is present and the count available is greater than than pointers present in the FPA.\n");
			if (fpa_err & 0x10)
				printk
				    ("FPA: Set when a Queue 0 page count available goes negative.\n");
			if (fpa_err & 0x8)
				printk
				    ("FPA: Set when a Double Bit Error is detected in FPF1.\n");
			if (fpa_err & 0x4)
				printk
				    ("FPA: Set when a Single Bit Error is detected in FPF1.\n");
			if (fpa_err & 0x2)
				printk
				    ("FPA: Set when a Double Bit Error is detected in FPF0.\n");
			if (fpa_err & 0x1)
				printk
				    ("FPA: Set when a Single Bit Error is detected in FPF0.\n");
			result = IRQ_HANDLED;
		}
	}

	return result;
}

/**
 * Return a string representing the system type
 *
 * @return
 */
const char *get_system_type(void)
{
	return octeon_board_type_string();
}

/**
 * Initialize the interrupt sub system
 */
void arch_init_irq(void)
{
	extern void octeon_irq_init(void);

	set_except_vector(0, octeon_handle_irq);
	octeon_irq_init();
}

/**
 * Early entry point for arch setup
 */
extern void __init plat_setup();

void prom_init(void)
{
	uint64_t mem_alloc_size = 4 << 20;
	uint64_t total = 0;
	uint64_t coreid;
	int count = 0;
	int i;

	octeon_hal_init();
	octeon_check_cpu_bist();

	coreid = octeon_get_core_num();

	/* Disable All CIU Interrupts. The ones we need will be enabled
	   later. Read the SUM register so we know the write completed. */
	octeon_write_csr(OCTEON_CIU_INTX_EN0((coreid * 2)), 0);
	octeon_write_csr(OCTEON_CIU_INTX_EN0((coreid * 2 + 1)), 0);
	octeon_read_csr(OCTEON_CIU_INTX_SUM0((coreid * 2)));

	printk("Cavium Networks Version: %s\n", get_version());

		octeon_write_lcd("LinuxSMP");

	for (i = 0; i < octeon_get_boot_num_arguments(); i++) {
		const char *arg = octeon_get_boot_argument(i);
		if ((strnicmp(arg, "mem=", 4) == 0)) {
		    MAX_MEMORY = memparse(arg + 4, &arg);
		    if (MAX_MEMORY == 0)
			MAX_MEMORY = 32ull<<30;
		} else if (strlen(arcs_cmdline) + strlen(arg) + 1 <
		    sizeof(arcs_cmdline) - 1) {
			strcat(arcs_cmdline, " ");
			strcat(arcs_cmdline, arg);
		}
	}

	/* you should these macros defined in include/asm/bootinfo.h */
	mips_machgroup = MACH_GROUP_CAVIUM;
	mips_machtype = MACH_CAVIUM_OCTEON;

	board_timer_setup = octeon_timer_setup;
	mips_hpt_frequency = octeon_get_clock_rate();
#ifndef CONFIG_CPU_TIMER
	mips_hpt_read = octeon_hpt_read;
	mips_hpt_init = octeon_hpt_init;
	mips_timer_ack = octeon_timer_ack;
#endif
	CYCLES_PER_JIFFY = ((mips_hpt_frequency + HZ / 2) / HZ);

	_machine_restart = octeon_restart;
	_machine_halt = octeon_halt;
	_machine_power_off = octeon_power_off;

	{
		struct uart_port octeon_port;

		memset(&octeon_port, 0, sizeof(octeon_port));
		octeon_port.flags =
		    ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST | UPF_SHARE_IRQ;
		octeon_port.iotype = UPIO_MEM;
		octeon_port.regshift = 3;	/* I/O addresses are every 8 bytes */
		octeon_port.uartclk = mips_hpt_frequency;	/* Clock rate of the chip */
		octeon_port.fifosize = 64;

		octeon_port.line = 0;
		octeon_port.mapbase = 0x8001180000000800ull + (1024 * 0);
		octeon_port.membase = (unsigned char *)octeon_port.mapbase;
		early_serial_setup(&octeon_port);

#ifdef CONFIG_KGDB_TTYS0
		octeon_port.irq = CAVIUM_UART_IRQ;
		kgdb8250_add_port(0, &octeon_port);
#endif
		octeon_port.line = 1;
		octeon_port.mapbase = 0x8001180000000800ull + (1024 * 1);
		octeon_port.membase = (unsigned char *)octeon_port.mapbase;
		early_serial_setup(&octeon_port);

#ifdef CONFIG_KGDB_TTYS1
		octeon_port.irq = CAVIUM_UART_IRQ + 1;
		kgdb8250_add_port(1, &octeon_port);
#endif

	}

	if (mem_alloc_size > MAX_MEMORY)
		mem_alloc_size = MAX_MEMORY;

	while ((count < BOOT_MEM_MAP_MAX) && (mem_alloc_size >= (1 << 20)) &&
	       (total < MAX_MEMORY)) {
		uint64_t memory;

		memory = octeon_ptr_to_phys(octeon_bootmem_alloc(mem_alloc_size, 0x10000));
		if (memory) {
			add_memory_region(memory, mem_alloc_size, BOOT_MEM_RAM);
			total += mem_alloc_size;
		} else
			break;

	}

	if (total == 0)
		panic("Unable to allocate memory from octeon_bootmem_alloc\n");

	octeon_hal_setup_reserved32();
	octeon_user_io_init();

	set_c0_status(0xff << 8);	/* Enable core interrupt processing */

	plat_setup();
}

unsigned long prom_free_prom_memory(void)
{
	uint64_t mem_cfg0, l2t_err, l2d_err, pow_ecc;

	/* Add an interrupt handler for ECC failures. Will also check ECC
	   status on any interrupt on this line */
	request_irq(8 + 46, octeon_ecc_interrupt, SA_SHIRQ, "ECC",
		    octeon_ecc_interrupt);

	/* Enable ECC Interrupts for double bit errors from main memory */
	mem_cfg0 = octeon_read_csr(OCTEON_LMC_MEM_CFG0);
#ifdef ECC_REPORT_SINGLE_BIT_ERRORS
	mem_cfg0 |= 0x3 << 19;
#else
	mem_cfg0 |= 0x2 << 19;
#endif
	octeon_write_csr(OCTEON_LMC_MEM_CFG0, mem_cfg0);

	/* Enable ECC Interrupts for double bit errors from L2C Tags */
	l2t_err = octeon_read_csr(OCTEON_L2T_ERR);
#ifdef ECC_REPORT_SINGLE_BIT_ERRORS
	l2t_err |= 0x7;
#else
	l2t_err |= 0x5;
#endif
	octeon_write_csr(OCTEON_L2T_ERR, l2t_err);

	/* Enable ECC Interrupts for double bit errors from L2D Errors */
	l2d_err = octeon_read_csr(OCTEON_L2D_ERR);
#ifdef ECC_REPORT_SINGLE_BIT_ERRORS
	l2d_err |= 0x7;
#else
	l2d_err |= 0x5;
#endif
	octeon_write_csr(OCTEON_L2D_ERR, l2d_err);

	/* Enable ECC Interrupts for double bit errors from the POW */
	pow_ecc = octeon_read_csr(OCTEON_POW_ECC_ERR);
#ifdef ECC_REPORT_SINGLE_BIT_ERRORS
	pow_ecc |= 0x3 << 2;
#else
	pow_ecc |= 0x2 << 2;
#endif
	octeon_write_csr(OCTEON_POW_ECC_ERR, pow_ecc);

	if (!octeon_is_pass1()) {
		/* Enable interrupt on IOB port SOP and EOP errors */
		octeon_write_csr(OCTEON_IOB_INT_ENB, 0xf);

		/* Enable interrupts on IPD errors */
		octeon_write_csr(OCTEON_IPD_INT_ENB, 0x1f);

		/* Enable zip interrupt on errors */
		octeon_write_csr(OCTEON_ZIP_INT_MASK, 1);

		/* Enable PKO interrupt on errors */
		octeon_write_csr(OCTEON_PKO_REG_INT_MASK, 0x2);

		/* Enable Timer interrupt on errors */
		octeon_write_csr(OCTEON_TIM_REG_INT_MASK, 0xff);

		/* Enable FPA interrupt on errors */
		octeon_write_csr(OCTEON_FPA_INT_ENB, 0xfffffff);
	}

	return 0;
}

#ifdef CONFIG_MTD_PHYSMAP
static struct mtd_partition octeon_parts[] = {
	[0] = {
		.name = "bootloader",
		.offset = 0x00000000,
		.size = 8*0x10000,
	},
	[1] = {
		.name = "data",
		.offset = MTDPART_OFS_APPEND,
		.size = 116*0x10000,
	},
	[2] = {
		.name = "reserved",
		.offset = MTDPART_OFS_APPEND,
		.size = MTDPART_SIZ_FULL,
	},
};
#endif

/**
 * This is called from arch/mips/kernel/setup.c early
 * during startup.
 *
 * @return
 */
void __init plat_setup(void)
{
#ifdef  CONFIG_MTD_PHYSMAP
        physmap_set_partitions(octeon_parts, ARRAY_SIZE(octeon_parts));
#endif
	/* Currently nothing to do here... */
}

