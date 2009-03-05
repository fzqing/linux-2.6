/*
 * 8250 interface for kgdb.
 *
 * This is a merging of many different drivers, and all of the people have
 * had an impact in some form or another:
 *
 * Amit Kale <amitkale@emsyssoft.com>, David Grothe <dave@gcom.com>,
 * Scott Foehner <sfoehner@engr.sgi.com>, George Anzinger <george@mvista.com>,
 * Robert Walsh <rjwalsh@durables.org>, wangdi <wangdi@clusterfs.com>,
 * San Mehat, Tom Rini <trini@mvista.com>
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kgdb.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>
#include <linux/serialP.h>
#include <linux/ioport.h>
#include <linux/touch_watchdogs.h>

#include <asm/io.h>
#include <asm/serial.h>		/* For BASE_BAUD and SERIAL_PORT_DFNS */

#ifdef CONFIG_RMI_PHOENIX
#include <asm/rmi/iomap.h>
#endif


#define GDB_BUF_SIZE	512	/* power of 2, please */

/* Speed of the UART. */
#if defined(CONFIG_KGDB_9600BAUD)
static int kgdb8250_baud = 9600;
#elif defined(CONFIG_KGDB_19200BAUD)
static int kgdb8250_baud = 19200;
#elif defined(CONFIG_KGDB_38400BAUD)
static int kgdb8250_baud = 38400;
#elif defined(CONFIG_KGDB_57600BAUD)
static int kgdb8250_baud = 57600;
#else
static int kgdb8250_baud = 115200;	/* Start with this if not given */
#endif

/* Index of the UART, matches ttySX naming. */
#if defined(CONFIG_KGDB_TTYS1)
int kgdb8250_ttyS = 1;
#elif defined(CONFIG_KGDB_TTYS2)
int kgdb8250_ttyS = 2;
#elif defined(CONFIG_KGDB_TTYS3)
int kgdb8250_ttyS = 3;
#else
int kgdb8250_ttyS = 0;		/* Start with this if not given */
#endif

/* Flag for if we need to call request_mem_region */
static int kgdb8250_needs_request_mem_region;

static char kgdb8250_buf[GDB_BUF_SIZE];
static atomic_t kgdb8250_buf_in_cnt;
static int kgdb8250_buf_out_inx;

/* Old-style serial definitions, if existant, and a counter. */
static int old_rs_table_copied;
static struct serial_state old_rs_table[] = {
#ifdef SERIAL_PORT_DFNS
	SERIAL_PORT_DFNS
#endif
};

/* Our internal table of UARTS. */
#define UART_NR	(ARRAY_SIZE(old_rs_table) + CONFIG_SERIAL_8250_NR_UARTS)
static struct uart_port kgdb8250_ports[UART_NR] = {
#ifndef CONFIG_KGDB_SIMPLE_SERIAL
	{
#if defined(CONFIG_KGDB_IRQ) && defined(CONFIG_KGDB_PORT)
		.irq		= CONFIG_KGDB_IRQ,
		.iobase		= CONFIG_KGDB_PORT,
		.iotype		= UPIO_PORT,
#endif
#ifdef CONFIG_KGDB_IOMEMBASE
		.membase	= (unsigned char *)CONFIG_KGDB_IOMEMBASE,
		.iotype		= UPIO_MEM,
#endif
	},
#endif
};

/* Macros to easily get what we want from kgdb8250_ports[kgdb8250_ttyS] */
#define CURRENTPORT		kgdb8250_ports[kgdb8250_ttyS]
#define KGDB8250_IRQ		CURRENTPORT.irq
#define KGDB8250_REG_SHIFT	CURRENTPORT.regshift

/* Base of the UART. */
static unsigned long kgdb8250_addr;
/* Pointers for I/O. */
static void (*serial_outb) (unsigned char val, unsigned long addr);
static unsigned long (*serial_inb) (unsigned long addr);

/* Forward declarations. */
static int kgdb8250_init(void);
extern int serial8250_release_irq(int irq);

static unsigned long
direct_inb(unsigned long addr)
{
#ifdef CONFIG_RMI_PHOENIX
	phoenix_reg_t *mmio =
		(phoenix_reg_t *)(addr);

	return *mmio;
#endif

#ifndef CONFIG_CPU_CAVIUM_OCTEON
	return readb((void *)addr);
#else
	return readq((void *)addr);
#endif
}

static void
direct_outb(unsigned char val, unsigned long addr)
{
#ifdef CONFIG_RMI_PHOENIX
	phoenix_reg_t *mmio =
		(phoenix_reg_t *)(addr);
	*mmio = val;

	return;
#endif

#ifndef CONFIG_CPU_CAVIUM_OCTEON
	writeb(val, (void *)addr);
#else
	writeq(val, (void *)addr);
#endif
}

static unsigned long
io_inb(unsigned long port)
{
	return inb(port);
}

static void
io_outb(unsigned char val, unsigned long port)
{
	outb(val, port);
}

/*
 * Wait until the interface can accept a char, then write it.
 */
static void
kgdb_put_debug_char(int chr)
{
	while (!(serial_inb(kgdb8250_addr + (UART_LSR << KGDB8250_REG_SHIFT)) &
		UART_LSR_THRE))
		;

	serial_outb(chr, kgdb8250_addr + (UART_TX << KGDB8250_REG_SHIFT));
}

/*
 * Get a byte from the hardware data buffer and return it
 */
static int
read_data_bfr(void)
{
	char it = serial_inb(kgdb8250_addr + (UART_LSR << KGDB8250_REG_SHIFT));

	if (it & UART_LSR_DR)
		return serial_inb(kgdb8250_addr +
				(UART_RX << KGDB8250_REG_SHIFT));

	/*
	 * If we have a framing error assume somebody messed with
	 * our uart.  Reprogram it and send '-' both ways...
	 */
	if (it & 0xc) {
		kgdb8250_init();
		kgdb_put_debug_char('-');
		return '-';
	}

	return -1;
}

/*
 * Get a char if available, return -1 if nothing available.
 * Empty the receive buffer first, then look at the interface hardware.
 */

static int
kgdb_get_debug_char(void)
{
	int retchr;

	/* intr routine has q'd chars */
	if (atomic_read(&kgdb8250_buf_in_cnt) != 0) {
		retchr = kgdb8250_buf[kgdb8250_buf_out_inx++];
		kgdb8250_buf_out_inx &= (GDB_BUF_SIZE - 1);
		atomic_dec(&kgdb8250_buf_in_cnt);
		return retchr;
	}

	do {
		touch_watchdogs();
		retchr = read_data_bfr();
	} while (retchr < 0);

	return retchr;
}

/*
 * This is the receiver interrupt routine for the GDB stub.
 * All that we need to do is verify that the interrupt happened on the
 * line we're in charge of.  If this is true, schedule a breakpoint and
 * return.
 */
static irqreturn_t
kgdb8250_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	char iir;

	if (irq != KGDB8250_IRQ)
		return IRQ_NONE;
        /*
         * If  there is some other CPU in KGDB then this is a
         * spurious interrupt. so return without even checking a byte
         */
        if (atomic_read(&debugger_active))
               return IRQ_NONE;

	iir = serial_inb(kgdb8250_addr + (UART_IIR << KGDB8250_REG_SHIFT));
	if (iir & UART_IIR_RDI)
		breakpoint();

	return IRQ_HANDLED;
}

/*
 *  Returns:
 *	0 on success, 1 on failure.
 */
static int
kgdb8250_init(void)
{
	unsigned cval;
	int bits = 8;
	int parity = 'n';
	int cflag = CREAD | HUPCL | CLOCAL;
	char ier = UART_IER_RDI;
	unsigned int base_baud;

	base_baud = CURRENTPORT.uartclk ? CURRENTPORT.uartclk / 16 : BASE_BAUD;

	/*
	 *      Now construct a cflag setting.
	 */
	switch (kgdb8250_baud) {
	case 1200:
		cflag |= B1200;
		break;
	case 2400:
		cflag |= B2400;
		break;
	case 4800:
		cflag |= B4800;
		break;
	case 19200:
		cflag |= B19200;
		break;
	case 38400:
		cflag |= B38400;
		break;
	case 57600:
		cflag |= B57600;
		break;
	case 115200:
		cflag |= B115200;
		break;
	default:
		kgdb8250_baud = 9600;
		/* Fall through */
	case 9600:
		cflag |= B9600;
		break;
	}
	switch (bits) {
	case 7:
		cflag |= CS7;
		break;
	default:
	case 8:
		cflag |= CS8;
		break;
	}
	switch (parity) {
	case 'o':
	case 'O':
		cflag |= PARODD;
		break;
	case 'e':
	case 'E':
		cflag |= PARENB;
		break;
	}

	/*
	 *      Divisor, bytesize and parity
	 *
	 */

	cval = cflag & (CSIZE | CSTOPB);
	cval >>= 4;
	if (cflag & PARENB)
		cval |= UART_LCR_PARITY;
	if (!(cflag & PARODD))
		cval |= UART_LCR_EPAR;

	/* Disable UART interrupts, set DTR and RTS high and set speed. */
	cval = 0x3;
#if	defined(CONFIG_ARCH_OMAP1510)
	/* Workaround to enable 115200 baud on OMAP1510 internal ports */
	if (cpu_is_omap1510() && is_omap_port((void *)kgdb8250_addr)) {
		if (kgdb8250_baud == 115200) {
			base_baud = 1;
			kgdb8250_baud = 1;
			serial_outb(1, kgdb8250_addr +
				(UART_OMAP_OSC_12M_SEL << KGDB8250_REG_SHIFT));
		} else {
			serial_outb(0, kgdb8250_addr +
				(UART_OMAP_OSC_12M_SEL << KGDB8250_REG_SHIFT));
		}
	}
#endif
	/* set DLAB */
	serial_outb(cval | UART_LCR_DLAB, kgdb8250_addr +
			(UART_LCR << KGDB8250_REG_SHIFT));
	/* LS */
	serial_outb(base_baud / kgdb8250_baud & 0xff, kgdb8250_addr +
			(UART_DLL << KGDB8250_REG_SHIFT));
	/* MS  */
	serial_outb(base_baud / kgdb8250_baud >> 8, kgdb8250_addr +
			(UART_DLM << KGDB8250_REG_SHIFT));
	/* reset DLAB */
	serial_outb(cval, kgdb8250_addr + (UART_LCR << KGDB8250_REG_SHIFT));

	/*
	 * XScale-specific bits that need to be set
	 */
	if (CURRENTPORT.type == PORT_XSCALE)
		ier |= UART_IER_UUE | UART_IER_RTOIE;

	/* turn on interrupts */
	serial_outb(ier, kgdb8250_addr + (UART_IER << KGDB8250_REG_SHIFT));
	serial_outb(UART_MCR_OUT2 | UART_MCR_DTR | UART_MCR_RTS,
		    kgdb8250_addr + (UART_MCR << KGDB8250_REG_SHIFT));

	/*
	 *      If we read 0xff from the LSR, there is no UART here.
	 */
	if (serial_inb(kgdb8250_addr + (UART_LSR << KGDB8250_REG_SHIFT)) ==
			0xff)
		return -1;
	return 0;
}

/*
 * Copy the old serial_state table to our uart_port table if we haven't
 * had values specifically configured in.  We need to make sure this only
 * happens once.
 */
static void
kgdb8250_copy_rs_table(void)
{
#ifdef CONFIG_KGDB_SIMPLE_SERIAL
	int i;

	for (i = 0; i < ARRAY_SIZE(old_rs_table); i++) {
		kgdb8250_ports[i].iobase = old_rs_table[i].port;
		kgdb8250_ports[i].irq = irq_canonicalize(old_rs_table[i].irq);
		kgdb8250_ports[i].uartclk = old_rs_table[i].baud_base * 16;
		kgdb8250_ports[i].membase = old_rs_table[i].iomem_base;
		kgdb8250_ports[i].iotype = old_rs_table[i].io_type;
		kgdb8250_ports[i].regshift = old_rs_table[i].iomem_reg_shift;
	}
#endif

	old_rs_table_copied = 1;
}

/*
 * Perform static initalization tasks which we need to always do,
 * even if KGDB isn't going to be invoked immediately.
 */
static int
kgdb8250_local_init(void)
{
	if (old_rs_table_copied == 0)
		kgdb8250_copy_rs_table();

	switch (CURRENTPORT.iotype) {
	case UPIO_MEM:
		if (CURRENTPORT.mapbase)
			kgdb8250_needs_request_mem_region = 1;
		if (CURRENTPORT.flags & UPF_IOREMAP) {
			CURRENTPORT.membase = ioremap(CURRENTPORT.mapbase,
					8 << KGDB8250_REG_SHIFT);
			if (!CURRENTPORT.membase)
				return 1;	/* Failed. */
		}
		kgdb8250_addr = (unsigned long)CURRENTPORT.membase;
		serial_outb = direct_outb;
		serial_inb = direct_inb;
		break;
	case UPIO_PORT:
	default:
		kgdb8250_addr = CURRENTPORT.iobase;
		serial_outb = io_outb;
		serial_inb = io_inb;
	}

	return 0;
}

static int
kgdb_init_io(void)
{
	if (kgdb8250_local_init())
		return -1;

	if (kgdb8250_init() == -1)
		return -1;

	return 0;
}


/*
 * Hookup our IRQ line now that it is safe to do so, after we grab any
 * memory regions we might need to.  If we haven't been initialized yet,
 * go ahead and copy the old_rs_table in.
 */
static void __init kgdb8250_late_init(void) {

	if (old_rs_table_copied == 0)
		kgdb8250_copy_rs_table();

#if defined(CONFIG_SERIAL_8250) || defined (CONFIG_SERIAL_8250_MODULE)
	/* Take the port away from the main driver. */
	serial8250_unregister_by_port(&CURRENTPORT);

	/* Now reinit the port as the above has disabled things. */
	kgdb8250_init();
#endif
	/* We may need to call request_mem_region() first. */
	if (kgdb8250_needs_request_mem_region)
		request_mem_region(CURRENTPORT.mapbase,
				8 << KGDB8250_REG_SHIFT, "kgdb");
	request_irq(KGDB8250_IRQ, kgdb8250_interrupt, SA_SHIRQ,
			"GDB-stub", &CURRENTPORT);
}

struct kgdb_io kgdb_io_ops = {
	.read_char = kgdb_get_debug_char,
	.write_char = kgdb_put_debug_char,
	.init = kgdb_init_io,
	.late_init = kgdb8250_late_init,
};

/**
 * 	kgdb8250_add_port - Define a serial port for use with KGDB
 * 	@i: The index of the port being added
 * 	@serial_req: The &struct uart_port describing the port
 *
 * 	On platforms where we must register the serial device
 * 	dynamically, this is the best option if a platform also normally
 * 	calls early_serial_setup().
 */
void
kgdb8250_add_port(int i, struct uart_port *serial_req)
{
	/* Copy the old table in if needed. */
	if (old_rs_table_copied == 0)
		kgdb8250_copy_rs_table();

	/* Copy the whole thing over. */
	memcpy(&kgdb8250_ports[i], serial_req, sizeof(struct uart_port));
}

/**
 * 	kgdb8250_add_platform_port - Define a serial port for use with KGDB
 * 	@i: The index of the port being added
 * 	@p: The &struct plat_serial8250_port describing the port
 *
 * 	On platforms where we must register the serial device
 * 	dynamically, this is the best option if a platform normally
 * 	handles uart setup with an array of &struct plat_serial8250_port.
 */
void 
kgdb8250_add_platform_port(int i, struct plat_serial8250_port *p)
{
	/* Copy the old table in if needed. */
	if (old_rs_table_copied == 0)
		kgdb8250_copy_rs_table();

	kgdb8250_ports[i].iobase = p->iobase;
	kgdb8250_ports[i].membase = p->membase;
	kgdb8250_ports[i].irq = p->irq;
	kgdb8250_ports[i].uartclk = p->uartclk;
	kgdb8250_ports[i].regshift = p->regshift;
	kgdb8250_ports[i].iotype = p->iotype;
	kgdb8250_ports[i].flags = p->flags;
	kgdb8250_ports[i].mapbase = p->mapbase;
}

/*
 * Syntax for this cmdline option is "kgdb8250=ttyno,baudrate"
 * with ",irq,iomembase" tacked on the end on IA64.
 */
static int __init
kgdb8250_opt(char *str)
{
	if (*str < '0' || *str > '3')
		goto errout;
	kgdb8250_ttyS = *str - '0';
	str++;
	if (*str != ',')
		goto errout;
	str++;
	kgdb8250_baud = simple_strtoul(str, &str, 10);
	if (kgdb8250_baud != 9600 && kgdb8250_baud != 19200 &&
	    kgdb8250_baud != 38400 && kgdb8250_baud != 57600 &&
	    kgdb8250_baud != 115200)
		goto errout;

#ifdef CONFIG_IA64
	if (*str == ',') {
		str++;
		KGDB8250_IRQ = simple_strtoul(str, &str, 10);
		if (*str == ',') {
			str++;
			CURRENTPORT.iotype = SERIAL_IO_MEM;
			CURRENTPORT.membase =
				(unsigned char *)simple_strtoul(str, &str, 0);
		}
	}
#endif

	return 0;

errout:
	printk(KERN_ERR "Invalid syntax for option kgdb8250=\n");
	return 1;
}
early_param("kgdb8250", kgdb8250_opt);
