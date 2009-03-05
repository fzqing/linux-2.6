/*
 * drivers/net/kgdb_eth.c
 *
 * A network interface for GDB.
 * Based upon 'gdbserial' by David Grothe <dave@gcom.com>
 * and Scott Foehner <sfoehner@engr.sgi.com>
 *
 * Maintainers: Amit S. Kale <amitkale@linsyssoft.com> and
 * 		Tom Rini <trini@kernel.crashing.org>
 *
 * 2004 (c) Amit S. Kale <amitkale@linsyssoft.com>
 * 2004 (c) MontaVista Software, Inc.
 *
 * Other folks:
 * San Mehat <nettwerk@biodome.org>
 * Robert Walsh <rjwalsh@durables.org>
 * wangdi <wangdi@clusterfs.com>.
 * Matt Mackall <mpm@selenic.com>
 * Pavel Machek <pavel@suse.cz>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/kgdb.h>
#include <linux/netpoll.h>
#include <linux/init.h>
#include <linux/touch_watchdogs.h>

#include <asm/atomic.h>

#define IN_BUF_SIZE 512		/* power of 2, please */
#define OUT_BUF_SIZE 30		/* We don't want to send too big of a packet. */

static char in_buf[IN_BUF_SIZE], out_buf[OUT_BUF_SIZE];
static int in_head, in_tail, out_count;
static atomic_t in_count;
static int configured;

static void rx_hook(struct netpoll *np, int port, char *msg, int len);

static struct netpoll np = {
	.name = "kgdboe",
	.dev_name = "eth0",
	.rx_hook = rx_hook,
	.local_port = 6443,
	.remote_port = 6442,
	.remote_mac = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
};

static int eth_getDebugChar(void)
{
	int chr;

#ifdef CONFIG_PREEMPT_RT
	/*
	 * A bit hairy. Netpoll API users uses mutexes (indirectly) and
	 * thus must have interrupts enabled:
	 */
	local_irq_enable();
#endif

	while (atomic_read(&in_count) == 0) {
		touch_watchdogs();
		WARN_ON_RT(irqs_disabled());
		netpoll_poll(&np);
		WARN_ON_RT(irqs_disabled());
	}

	chr = in_buf[in_tail++];
	in_tail &= (IN_BUF_SIZE - 1);
	atomic_dec(&in_count);
#ifdef CONFIG_PREEMPT_RT
	local_irq_disable();
#endif
	return chr;
}

static void eth_flushDebugChar(void)
{
#ifdef CONFIG_PREEMPT_RT
	/*
	 * A bit hairy. Netpoll API users uses mutexes (indirectly) and
	 * thus must have interrupts enabled:
	 */
	local_irq_enable();
#endif

	if (out_count && np.dev) {
		WARN_ON_RT(irqs_disabled());
		netpoll_send_udp(&np, out_buf, out_count);
		WARN_ON_RT(irqs_disabled());
		memset(out_buf, 0, sizeof(out_buf));
		out_count = 0;
	}
#ifdef CONFIG_PREEMPT_RT
	local_irq_disable();
#endif
}

static void eth_putDebugChar(int chr)
{
	out_buf[out_count++] = chr;
	if (out_count == OUT_BUF_SIZE)
		eth_flushDebugChar();
}

static void rx_hook(struct netpoll *np, int port, char *msg, int len)
{
	int i;

	np->remote_port = port;

	/* Is this gdb trying to attach? */
	if (!kgdb_connected && !atomic_read(&kgdb_setting_breakpoint))
		tasklet_schedule(&kgdb_tasklet_breakpoint);

	for (i = 0; i < len; i++) {
		if (msg[i] == 3)
			tasklet_schedule(&kgdb_tasklet_breakpoint);

		if (atomic_read(&in_count) >= IN_BUF_SIZE) {
			/* buffer overflow, clear it */
			in_head = in_tail = 0;
			atomic_set(&in_count, 0);
			break;
		}
		in_buf[in_head++] = msg[i];
		in_head &= (IN_BUF_SIZE - 1);
		atomic_inc(&in_count);
	}
}

/* We must be passed configuration options. */
static int option_setup(char *opt)
{
	configured = !netpoll_parse_options(&np, opt);
	return 0;
}
__setup("kgdboe=", option_setup);

int init_kgdboe(void)
{
	if (!configured || netpoll_setup(&np))
		return 1;

	printk(KERN_INFO "kgdb: debugging over ethernet enabled\n");

	return 0;
}

struct kgdb_io kgdb_io_ops = {
	.read_char = eth_getDebugChar,
	.write_char = eth_putDebugChar,
	.init = init_kgdboe,
	.flush = eth_flushDebugChar,
};
