/*************************************************************************
* Author: Cavium Networks info@caviumnetworks.com
*
* 2006 (c) Cavium Networks. This file is licensed under
* the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation. This program
* is licensed "as is" without any warranty of any kind, whether
* express or implied.
* This file may also be available under a different license from
* Cavium Networks.  Contact Cavium Networks for more details.
*************************************************************************/

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/init.h>
#include <linux/etherdevice.h>
#include <linux/ip.h>
#include <linux/string.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <octeon-app-init.h>

#define printf printk
#include "cvmx.h"
#include "cvmx-atomic.h"
#include "cvmx-ciu.h"
#include "cvmx-pip.h"
#include "cvmx-ipd.h"
#include "cvmx-pko.h"
#include "cvmx-gmx.h"
#include "cvmx-spi.h"
#include "cvmx-bootmem.h"
#include "cvmx-app-init.h"

/*
 * A few defines are used to control the operation of this driver:
 *  CONFIG_CAVIUM_RESERVE32
 *      This kernel config options controls the amount of memory configured
 *      in a wired TLB entry for all processes to share. If this is set, the
 *      driver may use this memory instead of kernel memory for pools. See
 *      the next option for when this happens.
 *  CONFIG_CAVIUM_OCTEON_NUM_PACKET_BUFFERS
 *      This kernel config option allows the user to control the number of
 *      packet and work queue buffers allocated by the driver. If this is zero,
 *      the driver uses the default from below. If this is set and the 32bit
 *      reserved memory region is also set, then buffers are allocated from the
 *      32bit region instead of kernel memory. This allows 32bit userspace
 *      application to access the buffers, but also requires all received
 *      packets to be copied.
 *  NUM_PACKET_BUFFERS
 *      This define is set based on CONFIG_CAVIUM_OCTEON_NUM_PACKET_BUFFERS. It
 *      is used by the driver to determine the number of packet and work queue
 *      buffers.
 *  USE_SKBUFFS_IN_HW
 *      Tells the driver to populate the packet buffers with kernel skbuffs.
 *      This allows the driver to receive packets without copying them. It also
 *      means that 32bit userspace can't access the packet buffers.
 *  USE_32BIT_SHARED
 *      This define tells the driver to allocate memory for buffers from the
 *      32bit sahred region instead of the kernel memory space.
 *  USE_OPTIMIZED_PKO_BUFFER_SIZE
 *      Set the size of the PKO command buffers to an odd number of 64bit
 *      words. This allows the normal two word send to stay aligned and never
 *      span a comamnd word buffer. It is faster but incompatable with the
 *      three word variant. This MUST be used with the linux32-packet-io
 *      example.
 *  USE_HW_TCPUDP_CHECKSUM
 *      Controls if the Octeon TCP/UDP checksum engine is used for packet
 *      output. If this is zero, the kernel will perform the checksum in
 *      software.
 *  USE_MULTICORE_RECEIVE
 *      Process receive interrupts on multiple cores. This spreads the network
 *      load across the first 8 processors. If ths is zero, only one core
 *      processes incomming packets.
 */
#if CONFIG_CAVIUM_OCTEON_NUM_PACKET_BUFFERS
#define NUM_PACKET_BUFFERS      CONFIG_CAVIUM_OCTEON_NUM_PACKET_BUFFERS
#define USE_SKBUFFS_IN_HW       0
#if CONFIG_CAVIUM_RESERVE32
#define USE_32BIT_SHARED    1
#else
#define USE_32BIT_SHARED    0
#endif
#else
#define NUM_PACKET_BUFFERS      1024
#define USE_SKBUFFS_IN_HW       1
#define USE_32BIT_SHARED        0
#endif

#define INTERRUPT_LIMIT             10000	/* Max interrupts per second per core */
#define USE_OPTIMIZED_PKO_BUFFER_SIZE 1
#define USE_HW_TCPUDP_CHECKSUM      1
#define USE_MULTICORE_RECEIVE       1
#define IP_PROTOCOL_TCP             6
#define IP_PROTOCOL_UDP             0x11

#define INTERFACE(port) (port >> 4)	/* Ports 0-15 are interface 0, 16-31 are interface 1 */
#define INDEX(port) (port & 0xf)
#define DEBUGPRINT(format, ...) do{if (__printk_ratelimit(HZ, 10)) printk(format, ##__VA_ARGS__);} while (0)

#ifndef CONFIG_SMP
#undef USE_MULTICORE_RECEIVE
#define USE_MULTICORE_RECEIVE 0
#endif

/**
 * The maximum number of ethernet ports to setup for Linux.
 * Set to zero to ignore all SPI and RGMII ethernet ports.
 * Ports not configured for Linux may be used with the
 * simple executive. The purpose of this paramter is to allow
 * setups where Linux configures some ports, but leaves others
 * for alternative setups for other applications. Normally
 * you want the kernel to do all teh config, and have the
 * application tweak it afterwards. In this case, don't specify
 * this paramter.
 */
int num_ethernet_devices = 1;
module_param(num_ethernet_devices, int, 0444);

/**
 * POW group to receive packets from. All ethernet hardware
 * will be configured to send incomming packets to this POW
 * group. Also any other software can submit packets to this
 * group for the kernel to process.
 */
int pow_receive_group = 15;
module_param(pow_receive_group, int, 0444);

/**
 * POW group to send packets to other software on. This
 * controls the creation of the virtual device pow0.
 * always_use_pow (below) also depends on this value.
 */
int pow_send_group = -1;
module_param(pow_send_group, int, 0644);

/**
 * When set, always send to the pow group. This will cause
 * packets sent to real ethernet devices to be sent to the
 * POW group instead of the hardware. Unless some other
 * application changes the config, packets will still be
 * received from the low level hardware. Use this option
 * to allow a CVMX app to intercept all packets from the
 * linux kernel. You must specify pow_send_group along with
 * this option.
 */
int always_use_pow = 0;
module_param(always_use_pow, int, 0444);

/**
 * Comma separated list of ethernet devices that should use the
 * POW for transmit instead of the actual ethernet hardware. This
 * is a per port version of always_use_pow. always_use_pow takes
 * precedence over this list. For example, setting this to
 * "eth2,spi3,spi7" would cause these three devices to transmit
 * using the pow_send_group.
 */
char pow_send_list[128] = "";
module_param_string(pow_send_list, pow_send_list, sizeof(pow_send_list), 0444);

/**
 * Exported from the kernel so we can determine board information. It is
 * passed by the bootloader to the kernel.
 */
extern cvmx_bootinfo_t *octeon_bootinfo;

/**
 * Private driver state stored in dev->priv
 */
typedef struct {
	int port;		/* PKO hardware output port */
	int queue;		/* PKO hardware queue for the port */
	int up;			/* Is this device up */
	int fau;		/* Hardware fetch and add to count outstanding tx buffers */
	int isRGMII;		/* Is this port an RGMII port? */
	struct sk_buff_head tx_free_list;	/* List of outstanding tx buffers */
	struct net_device_stats stats;	/* Device statistics */
	uint64_t link_status;	/* Status of the link we've configured for */
	struct mii_if_info mii_info;	/* Generic MII info structure */
} cvm_oct_private_t;

static struct net_device *cvm_oct_device[CVMX_PIP_NUM_INPUT_PORTS + 1];
static struct timer_list cvm_oct_poll_timer;	/* Periodic timer to check RGMII auto negotiation */
static struct tasklet_struct cvm_oct_tasklet[NR_CPUS];

static void cvm_oct_configure_rgmii_speed(struct net_device *dev);

/**
 * Return the version string. Generated from CVS tags. Turns
 * NAME_V_V_V_build_B into "V.V.V, build B". If the tag isn't of
 * this format then the tag is returned. If there isn't a tag
 * then the build date is returned as "Internal __DATE__".
 *
 * @return Version string
 */
static inline const char *cvm_oct_get_version(void)
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

#if USE_SKBUFFS_IN_HW

/**
 * Put an skbuff into a hardware pool
 *
 * @param pool   Pool to populate
 * @param size   Size of the buffer needed for the pool
 * @return Zero on success
 */
static inline int cvm_oct_free_hw_buffer(uint64_t pool, uint64_t size)
{
	struct sk_buff *skb = dev_alloc_skb(size + 128);
	if (unlikely(skb == NULL))
		return -1;

	skb_reserve(skb, 128 - (((unsigned long)skb->data) & 0x7f));
	*(struct sk_buff **)(skb->data - sizeof(void *)) = skb;
	/* Do a syncw to make sure the updates to the skb header are visiblle when
	   another core reads this packet in */
	CVMX_SYNCW;
	cvmx_fpa_free(skb->data, pool, 0);
	return 0;
}

/**
 * Fill the supplied hardware pool with skbuffs
 *
 * @param pool     Pool to allocate an skbuff for
 * @param size     Size of the buffer needed for the pool
 * @param elements Number of buffers to allocate
 */
static void cvm_oct_fill_hw_skbuff(uint64_t pool, uint64_t size,
				   uint64_t elements)
{
	while (elements--)
		if (cvm_oct_free_hw_buffer(pool, size))
			panic("Failed to allocate skb for hardware pool %lu\n",
			      pool);
}

/**
 * Free the supplied hardware pool of skbuffs
 *
 * @param pool     Pool to allocate an skbuff for
 * @param size     Size of the buffer needed for the pool
 * @param elements Number of buffers to allocate
 */
static void cvm_oct_free_hw_skbuff(uint64_t pool, uint64_t size, int elements)
{
	char *memory;

	do {
		memory = cvmx_fpa_alloc(pool);
		if (memory) {
			struct sk_buff *skb;
			elements--;
			skb = *(struct sk_buff **)(memory - sizeof(void *));
			dev_kfree_skb(skb);
		}
	} while (memory);

	if (elements < 0)
		printk
		    ("Warning: Freeing of pool %lu had too many skbuffs (%d)\n",
		     pool, elements);
	else if (elements > 0)
		printk("Warning: Freeing of pool %lu is missing %d skbuffs\n",
		       pool, elements);
}

#endif

/**
 * Given a packet data address, return a pointer to the
 * beginning of the packet buffer.
 *
 * @param packet_ptr Packet data hardware address
 * @return Packet buffer pointer
 */
static inline void *cvm_oct_get_buffer_ptr(cvmx_buf_ptr_t packet_ptr)
{
	return cvmx_phys_to_ptr(((packet_ptr.s.addr >> 7) - packet_ptr.s.back)
				<< 7);
}

/**
 * Enqueue a packet into the PKO output queue
 *
 * @param dev    Linux device to enqueue for
 * @param pko_command
 *               PKO command encoding of the packet
 * @param packet The packet data
 * @return Zero on success
 */
static inline int cvm_oct_low_level_send(struct net_device *dev,
					 cvmx_pko_command_word0_t pko_command,
					 cvmx_buf_ptr_t packet)
{
	uint64_t tx_command_state;
	uint64_t *tx_command_ptr;
	uint64_t tx_command_index;
	cvm_oct_private_t *priv = (cvm_oct_private_t *) dev->priv;

	/* Get the queue command ptr location from the FAU */
	CVMX_SYNCIOBDMA;
	tx_command_state = cvmx_scratch_read64(CVMX_SCR_SCRATCH);
	tx_command_ptr =
	    cvmx_phys_to_ptr(tx_command_state >> CVMX_PKO_INDEX_BITS);
	tx_command_index = tx_command_state & CVMX_PKO_INDEX_MASK;
	tx_command_ptr += tx_command_index;

	/* Check if we are at the end of the buffer and need to chain the next one */
#if USE_OPTIMIZED_PKO_BUFFER_SIZE
	if (likely(tx_command_index < CVMX_FPA_OUTPUT_BUFFER_POOL_SIZE / 8 - 4))
#else
	if (likely(tx_command_index < CVMX_FPA_OUTPUT_BUFFER_POOL_SIZE / 8 - 3))
#endif
	{
		/* No buffer needed. Output the command and go */
		tx_command_ptr[0] = pko_command.u64;
		tx_command_ptr[1] = packet.u64;
	} else {
		/* We need a new output buffer. At this point we have either 2 or 3
		   words left. The first word will always be the pko command */
		uint64_t *newbuf;
		tx_command_ptr[0] = pko_command.u64;

		/* Get a buffer for the new command buffer. We don't use scratch in
		   case some user app is doing something. It's possible the user app
		   is using the scrach when the interrupt happens. Reading scratch and
		   fetching the next buffer isn't atomic */
		newbuf = cvmx_fpa_alloc(CVMX_FPA_OUTPUT_BUFFER_POOL);
		if (unlikely(!newbuf))
			return -1;

#if USE_OPTIMIZED_PKO_BUFFER_SIZE == 0
		/* If we only have two words left we have to split the command over the
		   command buffers */
		if (CVMX_FPA_OUTPUT_BUFFER_POOL_SIZE / 8 - tx_command_index ==
		    2) {
			tx_command_ptr[1] = cvmx_ptr_to_phys(newbuf);
			*newbuf = packet.u64;
			cvmx_fau_atomic_write64(CVMX_FAU_REG_OQ_ADDR_INDEX +
						priv->queue * 8,
						(tx_command_ptr[1] <<
						 CVMX_PKO_INDEX_BITS) + 1);
		} else
#endif
		{
			tx_command_ptr[1] = packet.u64;
			tx_command_ptr[2] = cvmx_ptr_to_phys(newbuf);
			cvmx_fau_atomic_write64(CVMX_FAU_REG_OQ_ADDR_INDEX +
						priv->queue * 8,
						tx_command_ptr[2] <<
						CVMX_PKO_INDEX_BITS);
		}
	}

	cvmx_pko_doorbell(priv->port, priv->queue, 2);
	return 0;
}

/**
 * Packet transmit
 *
 * @param skb    Packet to send
 * @param dev    Device info structure
 * @return Always returns zero
 */
static int cvm_oct_xmit(struct sk_buff *skb, struct net_device *dev)
{
	cvmx_pko_command_word0_t pko_command;
	cvmx_buf_ptr_t hw_buffer;
	uint64_t old_scratch;
	int dropped;
	unsigned long flags;
	int32_t in_use;
	cvm_oct_private_t *priv = (cvm_oct_private_t *) dev->priv;

	/* Prefetch the private data structure. It is larger that one cache line */
	CVMX_PREFETCH(priv, 0);
	CVMX_PREFETCH(priv, 128);

	/* Start off assuming no drop */
	dropped = 0;

	/* Interrupts need to be disabled since we use scratch. We can't have
	   someone else modifying scratch while we are */
	spin_lock_irqsave(&priv->tx_free_list.lock, flags);

	preempt_disable();

	/* The sync must be inside the disabled interrupts to make sure the scratch
	   we save is correct. It will be restores before interrupts are enabled
	   again */
	CVMX_SYNCIOBDMA;
	old_scratch = cvmx_scratch_read64(CVMX_SCR_SCRATCH);

	/* Assume we're going to be able t osend this packet. Fetch and increment
	   the number of pending packets for output */
	cvmx_fau_async_fetch_and_add32(CVMX_SCR_SCRATCH, priv->fau, 1);

	/* Build the PKO buffer pointer */
	hw_buffer.u64 = 0;
	hw_buffer.s.addr = cvmx_ptr_to_phys(skb->data);
	hw_buffer.s.pool = 0;
	hw_buffer.s.size = (uint64_t) skb->end - (uint64_t) skb->head;

	/* Build the PKO command */
	pko_command.u64 = 0;
	pko_command.s.reg0 = priv->fau;
	pko_command.s.size0 = CVMX_FAU_OP_SIZE_32;
	pko_command.s.subone0 = 1;
	pko_command.s.dontfree = 1;
	pko_command.s.segs = 1;
	pko_command.s.total_bytes = skb->len;

	/* Check if we can use the hardware checksumming */
	if (USE_HW_TCPUDP_CHECKSUM && (skb->protocol == htons(ETH_P_IP)) &&
	    (skb->nh.iph->version == 4) && (skb->nh.iph->ihl == 5) &&
	    ((skb->nh.iph->frag_off == 0) || (skb->nh.iph->frag_off == 1 << 14))
	    && ((skb->nh.iph->protocol == IP_PROTOCOL_TCP)
		|| (skb->nh.iph->protocol == IP_PROTOCOL_UDP))) {
		/* Use hardware checksum calc */
		pko_command.s.ipoffp1 = sizeof(struct ethhdr) + 1;
	}

	/* Get the number of skbuffs in use by the hardware */
	CVMX_SYNCIOBDMA;
	in_use = cvmx_scratch_read64(CVMX_SCR_SCRATCH);

	/* Get the position for writing the PKO command. This will be used when we
	   call cvm_oct_low_level_send. If we drop, this must be decremented */
	cvmx_fau_async_fetch_and_add64(CVMX_SCR_SCRATCH,
				       CVMX_FAU_REG_OQ_ADDR_INDEX +
				       priv->queue * 8, 2);

	/* Free skbuffs not in use by the hardware, two at a time. Have 3 packets
	   left between the delete and insert to allow for timing slop */
	if (skb_queue_len(&priv->tx_free_list) - in_use > 4) {
		/* Call the IRQ version of dev_kfree_skb since interrupts are disabled */
		dev_kfree_skb_irq(__skb_dequeue(&priv->tx_free_list));
		dev_kfree_skb_irq(__skb_dequeue(&priv->tx_free_list));
	}

	/* Drop this packet if we have too many already queued to the HW */
	if (unlikely(skb_queue_len(&priv->tx_free_list) >= dev->tx_queue_len)) {
		/*DEBUGPRINT("%s: Tx dropped. Too many queued\n", dev->name); */
		/* We need to make sure the fetch and add for the PKO position
		   finished. Normally the low level send does this, but in this
		   case we don't call it */
		CVMX_SYNCIOBDMA;
		dropped = 1;
	}
	/* Send the packet to the output queue */
	else if (unlikely(cvm_oct_low_level_send(dev, pko_command, hw_buffer))) {
		DEBUGPRINT("%s: Failed to send the packet\n", dev->name);
		dropped = 1;
	}

	/* Restore the scratch area and enable interrupts */
	cvmx_scratch_write64(CVMX_SCR_SCRATCH, old_scratch);

	if (unlikely(dropped)) {
		dev_kfree_skb_any(skb);
		cvmx_fau_atomic_add64(CVMX_FAU_REG_OQ_ADDR_INDEX +
				      priv->queue * 8, -2);
		cvmx_fau_atomic_add32(priv->fau, -1);
		priv->stats.tx_dropped++;
	} else {
		/* Put this packet on the queue to be freed later */
		__skb_queue_tail(&priv->tx_free_list, skb);
	}

	preempt_enable();
	spin_unlock_irqrestore(&priv->tx_free_list.lock, flags);

	return 0;
}

/**
 * Packet transmit to the POW
 *
 * @param skb    Packet to send
 * @param dev    Device info structure
 * @return Always returns zero
 */
static int cvm_oct_xmit_pow(struct sk_buff *skb, struct net_device *dev)
{
	void *packet_buffer;
	void *copy_location;
	cvm_oct_private_t *priv = (cvm_oct_private_t *) dev->priv;

	/* Get a work queue entry */
	cvmx_wqe_t *work = cvmx_fpa_alloc(CVMX_FPA_WQE_POOL);
	if (unlikely(work == NULL)) {
		DEBUGPRINT("%s: Failed to allocate a work queue entry\n",
			   dev->name);
		priv->stats.tx_dropped++;
		dev_kfree_skb(skb);
		return 0;
	}

	/* Get a packet buffer */
	packet_buffer = cvmx_fpa_alloc(CVMX_FPA_PACKET_POOL);
	if (unlikely(packet_buffer == NULL)) {
		DEBUGPRINT("%s: Failed to allocate a packet buffer\n",
			   dev->name);
		cvmx_fpa_free(packet_buffer, CVMX_FPA_WQE_POOL, 0);
		priv->stats.tx_dropped++;
		dev_kfree_skb(skb);
		return 0;
	}

	/* Calculate where we need to copy the data to. We need to leave 8 bytes
	   for a next pointer (unused). We also need to include any configure
	   skip. Then we need to align the IP packet src and dest into the same
	   64bit word. The below calculation may add a little extra, but that
	   doesn't hurt */
	copy_location = packet_buffer + sizeof(uint64_t);
	copy_location += ((CVMX_HELPER_FIRST_MBUFF_SKIP + 7) & 0xfff8) + 6;

	/* We have to copy the packet since whoever processes this packet
	   will free it to a hardware pool. We can't use the trick of
	   counting outstanding packets like in cvm_oct_xmit */
	memcpy(copy_location, skb->data, skb->len);

	/* Fill in some of the work queue fields. We may need to add more
	   if the software at the other end needs them */
	work->hw_chksum = skb->csum;
	work->len = skb->len;
	work->ipprt = priv->port;
	work->qos = priv->port & 0x7;
	work->grp = pow_send_group;
	work->tag_type = CVMX_HELPER_INPUT_TAG_TYPE;
	work->tag = pow_send_group;	/* FIXME */
	work->word2.u64 = 0;	/* Default to zero. Sets of zero later are commented out */
	work->word2.s.bufs = 1;
	work->packet_ptr.u64 = 0;
	work->packet_ptr.s.addr = cvmx_ptr_to_phys(copy_location);
	work->packet_ptr.s.pool = CVMX_FPA_PACKET_POOL;
	work->packet_ptr.s.size = CVMX_FPA_PACKET_POOL_SIZE;
	work->packet_ptr.s.back = (copy_location - packet_buffer) >> 7;

	if (skb->protocol == htons(ETH_P_IP)) {
		work->word2.s.ip_offset = 14;
		/*work->word2.s.vlan_valid  = 0; */ /* FIXME */
		/*work->word2.s.vlan_cfi    = 0; */ /* FIXME */
		/*work->word2.s.vlan_id     = 0; */ /* FIXME */
		/*work->word2.s.dec_ipcomp  = 0; */ /* FIXME */
		work->word2.s.tcp_or_udp =
		    (skb->nh.iph->protocol == IP_PROTOCOL_TCP)
		    || (skb->nh.iph->protocol == IP_PROTOCOL_UDP);
		/*work->word2.s.dec_ipsec   = 0; */ /* FIXME */
		/*work->word2.s.is_v6       = 0; */ /* We only support IPv4 right now */
		/*work->word2.s.software    = 0; */ /* Hardware would set to zero */
		/*work->word2.s.L4_error    = 0; */ /* No error, packet is internal */
		work->word2.s.is_frag = !((skb->nh.iph->frag_off == 0)
					  || (skb->nh.iph->frag_off ==
					      1 << 14));
		/*work->word2.s.IP_exc      = 0; */ /* Assume Linux is sending a good packet */
		work->word2.s.is_bcast = (skb->pkt_type == PACKET_BROADCAST);
		work->word2.s.is_mcast = (skb->pkt_type == PACKET_MULTICAST);
		/*work->word2.s.not_IP      = 0; */ /* This is an IP packet */
		/*work->word2.s.rcv_error   = 0; */ /* No error, packet is internal */
		/*work->word2.s.err_code    = 0; */ /* No error, packet is internal */

		/* When copying the data, include 4 bytes of the ethernet header to
		   align the same way hardware does */
		memcpy(work->packet_data, skb->data + 10,
		       sizeof(work->packet_data));
	} else {
		/*work->word2.snoip.vlan_valid  = 0; */ /* FIXME */
		/*work->word2.snoip.vlan_cfi    = 0; */ /* FIXME */
		/*work->word2.snoip.vlan_id     = 0; */ /* FIXME */
		/*work->word2.snoip.software    = 0; */ /* Hardware would set to zero */
		work->word2.snoip.is_rarp = skb->protocol == htons(ETH_P_RARP);
		work->word2.snoip.is_arp = skb->protocol == htons(ETH_P_ARP);
		work->word2.snoip.is_bcast =
		    (skb->pkt_type == PACKET_BROADCAST);
		work->word2.snoip.is_mcast =
		    (skb->pkt_type == PACKET_MULTICAST);
		work->word2.snoip.not_IP = 1;	/* IP was done up above */
		/*work->word2.snoip.rcv_error   = 0; */ /* No error, packet is internal */
		/*work->word2.snoip.err_code    = 0; */ /* No error, packet is internal */
		memcpy(work->packet_data, skb->data, sizeof(work->packet_data));
	}

	/* Submit the packet to the POW */
	cvmx_pow_work_submit(work, work->tag, work->tag_type, work->qos,
			     work->grp);
	priv->stats.tx_packets++;
	priv->stats.tx_bytes += skb->len;
	dev_kfree_skb(skb);
	return 0;
}

/**
 * Interrupt handler. The interrupt occurs whenever the POW
 * transitions from 0->1 packets in our group.
 *
 * @param cpl
 * @param dev_id
 * @param regs
 * @return
 */
static irqreturn_t cvm_oct_do_interrupt(int cpl, void *dev_id,
					struct pt_regs *regs)
{
	/* Acknowledge the interrupt */
	cvmx_write_csr(CVMX_POW_WQ_INT, 1 << pow_receive_group);
	tasklet_schedule(cvm_oct_tasklet + smp_processor_id());
	return IRQ_HANDLED;
}

/**
 * Tasklet function that is scheduled on a core when an interrupt occurs.
 *
 * @param unused
 */
static void cvm_oct_tasklet_rx(unsigned long unused)
{
	const uint64_t coreid = cvmx_get_core_num();
	uint64_t old_group_mask;
	uint64_t old_scratch;
	uint64_t rx_count = 0;
	void *start_of_buffer;
	/* Prefetch cvm_oct_device since we know we need it soon */
	CVMX_PREFETCH(cvm_oct_device, 0);

	CVMX_SYNCIOBDMA;
	old_scratch = cvmx_scratch_read64(CVMX_SCR_SCRATCH);

	/* Only allow work for our group */
	old_group_mask = cvmx_read_csr(CVMX_POW_PP_GRP_MSKX(coreid));
	cvmx_write_csr(CVMX_POW_PP_GRP_MSKX(coreid), 1 << pow_receive_group);

	cvmx_pow_work_request_async(CVMX_SCR_SCRATCH, CVMX_POW_NO_WAIT);

	while (1) {
		struct sk_buff *skb;
		cvmx_wqe_t *work =
		    cvmx_pow_work_response_async(CVMX_SCR_SCRATCH);
		if (work == NULL)
			break;

		/* Limit each core to processing 30 packets without a break. This way
		   the RX can't starve the TX task. */
		if (likely(rx_count < 30))
			cvmx_pow_work_request_async_nocheck(CVMX_SCR_SCRATCH,
							    CVMX_POW_NO_WAIT);
		else
			cvmx_scratch_write64(CVMX_SCR_SCRATCH,
					     0x8000000000000000ull);
		rx_count++;

		CVMX_PREFETCH(cvm_oct_device[work->ipprt], 0);
		start_of_buffer = cvm_oct_get_buffer_ptr(work->packet_ptr);
		CVMX_PREFETCH(start_of_buffer, -8);

		/* Immediately throw away all packets with receive errors except for
		   CVMX_PIP_OVER_ERR. CVMX_PIP_OVER_ERR means the packet was good,
		   just oversized */
		if (unlikely
		    (work->word2.snoip.rcv_error
		     && (work->word2.snoip.err_code != CVMX_PIP_OVER_ERR))) {
			int segments;
			cvmx_buf_ptr_t segment_ptr;
			DEBUGPRINT("Port %d receive error, packet dropped\n",
				   work->ipprt);
			segments = work->word2.s.bufs;
			segment_ptr = work->packet_ptr;
			while (segments--) {
				cvmx_buf_ptr_t next_ptr =
				    *(cvmx_buf_ptr_t *)
				    cvmx_phys_to_ptr(segment_ptr.s.addr - 8);
				if (unlikely(!segment_ptr.s.i))
					cvmx_fpa_free(cvm_oct_get_buffer_ptr
						      (segment_ptr),
						      segment_ptr.s.pool, 0);
				segment_ptr = next_ptr;
			}
			cvmx_fpa_free(work, CVMX_FPA_WQE_POOL, 0);
			continue;
		}

		/* We can only use the zero copy path if skbuffs are in the FPA pool
		   and the packet fit in a single buffer */
#if USE_SKBUFFS_IN_HW
#warning USE_SKBUFFS_IN_HWUSE_SKBUFFS_IN_HWUSE_SKBUFFS_IN_HWUSE_SKBUFFS_IN_HWUSE_SKBUFFS_IN_HWUSE_SKBUFFS_IN_HW
		if (likely(work->word2.s.bufs == 1)) {
			skb =
			    *(struct sk_buff **)(start_of_buffer -
						 sizeof(void *));
			/* This calculation was changed in case the skb header is using a
			   different address aliasing type than the buffer. It doesn't make
			   any differnece now, but the new one is more correct */
			skb->data =
			    skb->head + work->packet_ptr.s.addr -
			    cvmx_ptr_to_phys(skb->head);
			CVMX_PREFETCH(skb->data, 0);
			skb->len = work->len;
			skb->tail = skb->data + skb->len;
			/* Free a buffer to replace the one we just took */
			cvm_oct_free_hw_buffer(CVMX_FPA_PACKET_POOL,
					       CVMX_FPA_PACKET_POOL_SIZE);
		} else
#endif
		{
			/* We have to copy the packet. First allocate an skbuff for it */
			skb = dev_alloc_skb(work->len);
			if (!skb) {
				cvmx_buf_ptr_t segment_ptr;
				int segments = work->word2.s.bufs;
				DEBUGPRINT
				    ("Port %d failed to allocate skbuff, packet dropped\n",
				     work->ipprt);
				segment_ptr = work->packet_ptr;
				while (segments--) {
					cvmx_buf_ptr_t next_ptr =
					    *(cvmx_buf_ptr_t *)
					    cvmx_phys_to_ptr(segment_ptr.s.
							     addr - 8);
					if (unlikely(!segment_ptr.s.i))
						cvmx_fpa_free
						    (cvm_oct_get_buffer_ptr
						     (segment_ptr),
						     segment_ptr.s.pool, 0);
					segment_ptr = next_ptr;
				}
				cvmx_fpa_free(work, CVMX_FPA_WQE_POOL, 0);
				continue;
			}

			/* Check if we've received a packet that was entirely stored the
			   work entry. This is untested */
			if (unlikely(work->word2.s.bufs == 0)) {
				DEBUGPRINT
				    ("Port %d received a work with work->word2.s.bufs=0, untested\n",
				     work->ipprt);
				memcpy(skb_put(skb, work->len),
				       work->packet_data, work->len);
				/* No packet buffers to free */
			} else {
				int segments = work->word2.s.bufs;
				cvmx_buf_ptr_t segment_ptr = work->packet_ptr;
				int len = work->len;
				while (segments--) {
					cvmx_buf_ptr_t next_ptr =
					    *(cvmx_buf_ptr_t *)
					    cvmx_phys_to_ptr(segment_ptr.s.
							     addr - 8);
					/* Octeon Errata PKI-100: The segment size is wrong. Until it
					   is fixed, calculate the segment size based on the packet
					   pool buffer size. When it is fixed, the following line should
					   be replaced with this one:
					   int segment_size = segment_ptr.s.size; */
					int segment_size =
					    CVMX_FPA_PACKET_POOL_SIZE -
					    (segment_ptr.s.addr -
					     (((segment_ptr.s.addr >> 7) -
					       segment_ptr.s.back) << 7));
					/* Don't copy more than what is left in the packet */
					if (segment_size > len)
						segment_size = len;
					/* Copy the data into the packet */
					memcpy(skb_put(skb, segment_size),
					       cvmx_phys_to_ptr(segment_ptr.s.
								addr),
					       segment_size);
					/* Free this segment if the don't free bit isn't set */
					if (unlikely(!segment_ptr.s.i))
						cvmx_fpa_free
						    (cvm_oct_get_buffer_ptr
						     (segment_ptr),
						     segment_ptr.s.pool, 0);
					/* Reduce the amount of bytes left to copy */
					len -= segment_size;
					segment_ptr = next_ptr;
				}
			}
		}

		if (likely
		    ((work->ipprt <= CVMX_PIP_NUM_INPUT_PORTS)
		     && cvm_oct_device[work->ipprt])) {
			struct net_device *dev = cvm_oct_device[work->ipprt];
			cvm_oct_private_t *priv =
			    (cvm_oct_private_t *) dev->priv;

			/* Only accept packets for devices that are currently up */
			if (likely(priv->up)) {
				skb->protocol = eth_type_trans(skb, dev);
				skb->dev = dev;

				if (unlikely
				    (work->word2.s.not_IP
				     || work->word2.s.IP_exc))
					skb->ip_summed = CHECKSUM_NONE;
				else
					skb->ip_summed = CHECKSUM_UNNECESSARY;

				netif_rx(skb);
			} else {
				/* Drop any packet received for a device that isn't up */
				/*DEBUGPRINT("%s: Device not up, packet dropped\n", dev->name); */
				cvmx_atomic_add64_nosync(&priv->stats.
							 rx_dropped, 1);
				dev_kfree_skb_irq(skb);
			}
		} else {
			/* Drop any packet received for a device that doesn't exist */
			DEBUGPRINT
			    ("Port %d not controlled by Linux, packet dropped\n",
			     work->ipprt);
			dev_kfree_skb_irq(skb);
		}
		cvmx_fpa_free(work, CVMX_FPA_WQE_POOL, 0);
	}

	/* Restore the original POW group mask */
	cvmx_write_csr(CVMX_POW_PP_GRP_MSKX(coreid), old_group_mask);
	/* Restore the scratch area */
	cvmx_scratch_write64(CVMX_SCR_SCRATCH, old_scratch);
}

#ifdef CONFIG_NET_POLL_CONTROLLER
static void cvm_oct_poll(struct net_device *dev)
{
	cvm_oct_tasklet_rx(0);
}
#endif

/**
 * Open a device for use. Device should be able to send and
 * receive packets after this is called.
 *
 * @param dev    Device to bring up
 * @return Zero on success
 */
static int cvm_oct_open(struct net_device *dev)
{
	cvm_oct_private_t *priv = (cvm_oct_private_t *) dev->priv;
	priv->up = 1;

	/* For RGMII ports, we enable them when they come up */
	if (priv->isRGMII) {
		cvmx_gmxx_prtx_cfg_t gmx_cfg;
		gmx_cfg.u64 =
		    cvmx_read_csr(CVMX_GMXX_PRTX_CFG
				  (INDEX(priv->port), INTERFACE(priv->port)));
		gmx_cfg.s.en = 1;
		cvmx_write_csr(CVMX_GMXX_PRTX_CFG
			       (INDEX(priv->port), INTERFACE(priv->port)),
			       gmx_cfg.u64);
	}

	return 0;
}

/**
 * Stop an ethernet device. No more packets should be
 * received from this device.
 *
 * @param dev    Device to bring down
 * @return Zero on success
 */
static int cvm_oct_stop(struct net_device *dev)
{
	cvm_oct_private_t *priv = (cvm_oct_private_t *) dev->priv;

	/* For RGMII ports, Disable the port when it goes down */
	if (priv->isRGMII) {
		cvmx_gmxx_prtx_cfg_t gmx_cfg;
		gmx_cfg.u64 =
		    cvmx_read_csr(CVMX_GMXX_PRTX_CFG
				  (INDEX(priv->port), INTERFACE(priv->port)));
		gmx_cfg.s.en = 0;
		cvmx_write_csr(CVMX_GMXX_PRTX_CFG
			       (INDEX(priv->port), INTERFACE(priv->port)),
			       gmx_cfg.u64);
	}

	priv->up = 0;
	return 0;
}

/**
 * Get the low level ethernet statistics
 *
 * @param dev    Device to get the statistics from
 * @return Pointer to the statistics
 */
static struct net_device_stats *cvm_oct_get_stats(struct net_device *dev)
{
	cvmx_pip_port_status_t rx_status;
	cvmx_pko_port_status_t tx_status;
	cvm_oct_private_t *priv = (cvm_oct_private_t *) dev->priv;

	if (priv->port < CVMX_PIP_NUM_INPUT_PORTS) {
		cvmx_pip_get_port_status(priv->port, 1, &rx_status);
		cvmx_pko_get_port_status(priv->port, 1, &tx_status);

		priv->stats.rx_packets += rx_status.inb_packets;
		priv->stats.tx_packets += tx_status.packets;
		priv->stats.rx_bytes += rx_status.inb_octets;
		priv->stats.tx_bytes += tx_status.octets;
		priv->stats.multicast += rx_status.multicast_packets;
		priv->stats.rx_crc_errors += rx_status.inb_errors;
		priv->stats.rx_frame_errors += rx_status.fcs_align_err_packets;
	}

	return &priv->stats;
}

/**
 * Periodic timer tick for slow management operations
 *
 * @param arg    Device to check
 */
static void cvm_do_timer(unsigned long arg)
{
	int port;

	/* Loop through all the ports and check the RGMII link status */
	for (port = 0; port < CVMX_PIP_NUM_INPUT_PORTS; port++) {
		if (cvm_oct_device[port]) {
			cvm_oct_private_t *priv =
			    (cvm_oct_private_t *) cvm_oct_device[port]->priv;
			/* Check link negotiation */
			if (priv->isRGMII)
				cvm_oct_configure_rgmii_speed(cvm_oct_device
							      [port]);

			cvm_oct_get_stats(cvm_oct_device[port]);
		}

	}

	/* Repeat every two seconds */
	mod_timer(&cvm_oct_poll_timer, jiffies + HZ * 2);
}

/**
 * Set the multicast list. Currently unimplemented.
 *
 * @param dev    Device to work on
 */
static void cvm_oct_set_multicast_list(struct net_device *dev)
{
	cvm_oct_private_t *priv = (cvm_oct_private_t *) dev->priv;

	if (priv->isRGMII) {
		uint64_t control = 1;	/* Allow broadcast MAC addresses */
		cvmx_gmxx_prtx_cfg_t gmx_cfg;
		int interface = INTERFACE(priv->port);
		int index = INDEX(priv->port);

		if (dev->mc_list || (dev->flags & IFF_ALLMULTI)
		    || (dev->flags & IFF_PROMISC))
			control |= 2 << 1;	/* Force accept multicast packets */
		else
			control |= 1 << 1;	/* Force reject multicat packets */

		if (dev->flags & IFF_PROMISC)
			control |= 0 << 3;	/* Reject matches if promisc. Since CAM is shut off, should accept everything */
		else
			control |= 1 << 3;	/* Filter packets based on the CAM */

		gmx_cfg.u64 =
		    cvmx_read_csr(CVMX_GMXX_PRTX_CFG(index, interface));
		cvmx_write_csr(CVMX_GMXX_PRTX_CFG(index, interface),
			       gmx_cfg.u64 & ~1ull);

		cvmx_write_csr(CVMX_GMXX_RXX_ADR_CTL(index, interface),
			       control);
		if (dev->flags & IFF_PROMISC)
			cvmx_write_csr(CVMX_GMXX_RXX_ADR_CAM_EN
				       (index, interface), 0);
		else
			cvmx_write_csr(CVMX_GMXX_RXX_ADR_CAM_EN
				       (index, interface), 1);

		cvmx_write_csr(CVMX_GMXX_PRTX_CFG(index, interface),
			       gmx_cfg.u64);
	}
}

/**
 * Set the hardware MAC address for a device
 *
 * @param dev    Device to change the MAC address for
 * @param addr   Address structure to change it too. MAC address is addr + 2.
 * @return Zero on success
 */
static int cvm_oct_set_mac_address(struct net_device *dev, void *addr)
{
	cvm_oct_private_t *priv = (cvm_oct_private_t *) dev->priv;

	memcpy(dev->dev_addr, addr + 2, 6);

	if (priv->isRGMII) {
		int i;
		uint8_t *ptr = addr;
		uint64_t mac = 0;
		cvmx_gmxx_prtx_cfg_t gmx_cfg;
		int interface = INTERFACE(priv->port);
		int index = INDEX(priv->port);

		for (i = 0; i < 6; i++)
			mac = (mac << 8) | (uint64_t) (ptr[i + 2]);

		gmx_cfg.u64 =
		    cvmx_read_csr(CVMX_GMXX_PRTX_CFG(index, interface));
		cvmx_write_csr(CVMX_GMXX_PRTX_CFG(index, interface),
			       gmx_cfg.u64 & ~1ull);

		cvmx_write_csr(CVMX_GMXX_SMACX(index, interface), mac);
		cvmx_write_csr(CVMX_GMXX_RXX_ADR_CAM0(index, interface),
			       ptr[2]);
		cvmx_write_csr(CVMX_GMXX_RXX_ADR_CAM1(index, interface),
			       ptr[3]);
		cvmx_write_csr(CVMX_GMXX_RXX_ADR_CAM2(index, interface),
			       ptr[4]);
		cvmx_write_csr(CVMX_GMXX_RXX_ADR_CAM3(index, interface),
			       ptr[5]);
		cvmx_write_csr(CVMX_GMXX_RXX_ADR_CAM4(index, interface),
			       ptr[6]);
		cvmx_write_csr(CVMX_GMXX_RXX_ADR_CAM5(index, interface),
			       ptr[7]);
		cvm_oct_set_multicast_list(dev);
		cvmx_write_csr(CVMX_GMXX_PRTX_CFG(index, interface),
			       gmx_cfg.u64);
	}
	return 0;
}

/**
 * Change the link MTU. Unimplemented
 *
 * @param dev     Device to change
 * @param new_mtu The new MTU
 * @return Zero on success
 */
static int cvm_oct_change_mtu(struct net_device *dev, int new_mtu)
{
	cvm_oct_private_t *priv = (cvm_oct_private_t *) dev->priv;

	/* Limit the MTU to make sure the ethernet packets are between 64 bytes
	   and 65535 bytes */
	if ((new_mtu + 14 + 4 < 64) || (new_mtu + 14 + 4 > 65392)) {
		printk("MTU must be between %d and %d.\n", 64 - 14 - 4,
		       65392 - 14 - 4);
		return -EINVAL;
	}
	dev->mtu = new_mtu;

	if (priv->isRGMII) {
		/* Set the hardware to truncate pacekts larger than the MTU */
		int interface = INTERFACE(priv->port);
		int index = INDEX(priv->port);
		cvmx_write_csr(CVMX_GMXX_RXX_JABBER(index, interface),
			       new_mtu + 14 + 4);
	}

	if (octeon_is_pass1()
	    && (new_mtu >
		CVMX_FPA_PACKET_POOL_SIZE - CVMX_HELPER_FIRST_MBUFF_SKIP - 32))
		printk
		    ("Warning: Octeon Pass 1 has problems with chained buffers. You may lose buffers with jumbo frames\n");

	return 0;
}

/**
 * Perform an MII read. Called by the generic MII routines
 *
 * @param dev      Device to perform read for
 * @param phy_id   The MII phy id
 * @param location Register location to read
 * @return Result from the read or zero on failure
 */
static int cvm_oct_mdio_read(struct net_device *dev, int phy_id, int location)
{
	cvmx_smi_cmd_t smi_cmd;
	cvmx_smi_rd_dat_t smi_rd;

	smi_cmd.u64 = 0;
	smi_cmd.s.phy_op = 1;
	smi_cmd.s.phy_adr = phy_id;
	smi_cmd.s.reg_adr = location;
	cvmx_write_csr(CVMX_SMI_CMD, smi_cmd.u64);

	do {
		yield();
		smi_rd.u64 = cvmx_read_csr(CVMX_SMI_RD_DAT);
	} while (smi_rd.s.pending);

	if (smi_rd.s.val)
		return smi_rd.s.dat;
	else
		return 0;
}

/**
 * Perform an MII write. Called by the generic MII routines
 *
 * @param dev      Device to perform write for
 * @param phy_id   The MII phy id
 * @param location Register location to write
 * @param val      Value to write
 */
static void cvm_oct_mdio_write(struct net_device *dev, int phy_id, int location,
			       int val)
{
	cvmx_smi_cmd_t smi_cmd;
	cvmx_smi_wr_dat_t smi_wr;

	smi_wr.u64 = 0;
	smi_wr.s.dat = val;
	cvmx_write_csr(CVMX_SMI_WR_DAT, smi_wr.u64);

	smi_cmd.u64 = 0;
	smi_cmd.s.phy_op = 0;
	smi_cmd.s.phy_adr = phy_id;
	smi_cmd.s.reg_adr = location;
	cvmx_write_csr(CVMX_SMI_CMD, smi_cmd.u64);

	do {
		yield();
		smi_wr.u64 = cvmx_read_csr(CVMX_SMI_WR_DAT);
	} while (smi_wr.s.pending);
}

/**
 * IOCTL support for PHY control
 *
 * @param dev    Device to change
 * @param rq     the request
 * @param cmd    the command
 * @return Zero on success
 */
static int cvm_oct_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	cvm_oct_private_t *priv = (cvm_oct_private_t *) dev->priv;
	struct mii_ioctl_data *data = if_mii(rq);
	unsigned int duplex_chg;

	if (priv->isRGMII)
		return generic_mii_ioctl(&priv->mii_info, data, cmd,
					 &duplex_chg);
	else
		return -EOPNOTSUPP;
}

static void cvm_oct_get_drvinfo(struct net_device *dev,
				struct ethtool_drvinfo *info)
{
	strcpy(info->driver, "cavium-ethernet");
	strcpy(info->version, cvm_oct_get_version());
	strcpy(info->bus_info, "Builtin");
}

static int cvm_oct_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	cvm_oct_private_t *priv = (cvm_oct_private_t *) dev->priv;
	return mii_ethtool_gset(&priv->mii_info, cmd);
}

static int cvm_oct_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	cvm_oct_private_t *priv = (cvm_oct_private_t *) dev->priv;
	return mii_ethtool_sset(&priv->mii_info, cmd);
}

static int cvm_oct_nway_reset(struct net_device *dev)
{
	cvm_oct_private_t *priv = (cvm_oct_private_t *) dev->priv;
	return mii_nway_restart(&priv->mii_info);
}

static u32 cvm_oct_get_link(struct net_device *dev)
{
	cvm_oct_private_t *priv = (cvm_oct_private_t *) dev->priv;
	return mii_link_ok(&priv->mii_info);
}

static struct ethtool_ops cvm_oct_ethtool_ops = {
	.get_drvinfo = cvm_oct_get_drvinfo,
	.get_settings = cvm_oct_get_settings,
	.set_settings = cvm_oct_set_settings,
	.nway_reset = cvm_oct_nway_reset,
	.get_link = cvm_oct_get_link,
	.get_sg = ethtool_op_get_sg,
	.get_tx_csum = ethtool_op_get_tx_csum,
};

/**
 * Per network device initialization
 *
 * @param dev    Device to initialize
 * @return Zero on success
 */
static int __init cvm_oct_init(struct net_device *dev)
{
	static int count = 0;
	cvm_oct_private_t *priv = (cvm_oct_private_t *) dev->priv;
	char mac[8] = { 0x00, 0x00,
		octeon_bootinfo->mac_addr_base[0],
		octeon_bootinfo->mac_addr_base[1],
		octeon_bootinfo->mac_addr_base[2],
		octeon_bootinfo->mac_addr_base[3],
		octeon_bootinfo->mac_addr_base[4],
		octeon_bootinfo->mac_addr_base[5] + count
	};

	if (priv->queue != -1)
		dev->hard_start_xmit = cvm_oct_xmit;
	else
		dev->hard_start_xmit = cvm_oct_xmit_pow;

	dev->get_stats = cvm_oct_get_stats;
	dev->open = cvm_oct_open;
	dev->stop = cvm_oct_stop;
	dev->weight = 16;
	dev->set_mac_address = cvm_oct_set_mac_address;
	dev->set_multicast_list = cvm_oct_set_multicast_list;
	dev->change_mtu = cvm_oct_change_mtu;
	dev->do_ioctl = cvm_oct_ioctl;
	SET_ETHTOOL_OPS(dev, &cvm_oct_ethtool_ops);
#ifdef CONFIG_NET_POLL_CONTROLLER
	dev->poll_controller = cvm_oct_poll;
#endif
	if (USE_HW_TCPUDP_CHECKSUM)
		dev->features = NETIF_F_IP_CSUM;

	cvm_oct_set_mac_address(dev, mac);
	count++;
	if (count > octeon_bootinfo->mac_addr_count)
		printk
		    ("ERROR: Board was allocated %d mac addresses, currently using %d\n",
		     octeon_bootinfo->mac_addr_count, count);
	return 0;
}

/**
 * This function fills a hardware pool with memory. Depending
 * on the config defines, this memory might come from the
 * kernel or global 32bit memory allocated with
 * cvmx_bootmem_alloc.
 *
 * @param pool     Pool to populate
 * @param size     Size of each buffer in the pool
 * @param elements Number of buffers to allocate
 */
static void cvm_oct_fill_hw_memory(uint64_t pool, uint64_t size,
				   uint64_t elements)
{
	char *memory;

#if USE_32BIT_SHARED

	memory =
	    cvmx_bootmem_alloc_range(elements * size, 128,
				     0x80000000 -
				     (CONFIG_CAVIUM_RESERVE32 << 20),
				     0x7fffffff);
	if (memory == NULL)
		panic("Unable to allocate %lu bytes for FPA pool %ld\n",
		      elements * size, pool);
	printk("Memory range %p - %p reserved for hardware\n", memory,
	       memory + elements * size - 1);
	while (elements--) {
		cvmx_fpa_free(memory, pool, 0);
		memory += size;
	}

#else
	while (elements--) {
		memory = kmalloc(size, SLAB_DMA);
		if (memory == NULL)
			panic("Unable to allocate %lu bytes for FPA pool %ld\n",
			      elements * size, pool);
		cvmx_fpa_free(memory, pool, 0);
	}
#endif
}

/**
 * Free memory previously allocated with cvm_oct_fill_hw_memory
 *
 * @param pool     FPA pool to free
 * @param size     Size of each buffer in the pool
 * @param elements Number of buffers that should be in the pool
 */
static void cvm_oct_free_hw_memory(uint64_t pool, uint64_t size, int elements)
{

#if USE_32BIT_SHARED
	printk("Warning: 32 shared memory is not freeable\n");
#else
	uint64_t top_bits;
	char *memory = kmalloc(size, SLAB_DMA);
	if (memory == NULL)
		panic("Unable to get memory for top bits reference\n");
	top_bits = ((uint64_t) memory >> 36) << 36;
	kfree(memory);
	do {
		memory = cvmx_fpa_alloc(pool);
		if (memory) {
			elements--;
			memory =
			    (char *)(top_bits |
				     ((uint64_t) memory & 0xfffffffffull));
			kfree(memory);
		}
	} while (memory);

	if (elements < 0)
		printk("Freeing of pool %lu had too many buffers (%d)\n", pool,
		       elements);
	else if (elements > 0)
		printk("Warning: Freeing of pool %lu is missing %d buffers\n",
		       pool, elements);
#endif
}

/**
 * Configure common hardware for all interfaces
 */
static void cvm_oct_configure_common_hw(void)
{
	/* Setup the FPA */
	cvmx_fpa_enable();
	cvm_oct_fill_hw_memory(CVMX_FPA_WQE_POOL, CVMX_FPA_WQE_POOL_SIZE,
			       NUM_PACKET_BUFFERS);
#if CVMX_FPA_OUTPUT_BUFFER_POOL != CVMX_FPA_PACKET_POOL
	cvm_oct_fill_hw_memory(CVMX_FPA_OUTPUT_BUFFER_POOL,
			       CVMX_FPA_OUTPUT_BUFFER_POOL_SIZE, 128);
#endif

#if USE_SKBUFFS_IN_HW
	cvm_oct_fill_hw_skbuff(CVMX_FPA_PACKET_POOL, CVMX_FPA_PACKET_POOL_SIZE,
			       NUM_PACKET_BUFFERS);
#else
	cvm_oct_fill_hw_memory(CVMX_FPA_PACKET_POOL, CVMX_FPA_PACKET_POOL_SIZE,
			       NUM_PACKET_BUFFERS);
#endif

	if (num_ethernet_devices) {
		cvmx_pko_pool_cfg_t config;
		cvmx_pko_reg_gmx_port_mode_t pko_mode;
		cvmx_ipd_config(CVMX_FPA_PACKET_POOL_SIZE / 8,
				CVMX_HELPER_FIRST_MBUFF_SKIP / 8,
				CVMX_HELPER_NOT_FIRST_MBUFF_SKIP / 8,
				CVMX_HELPER_FIRST_MBUFF_SKIP / 128,
				CVMX_HELPER_NOT_FIRST_MBUFF_SKIP / 128,
				CVMX_FPA_WQE_POOL,
				CVMX_IPD_OPC_MODE_STF,
				CVMX_HELPER_ENABLE_BACK_PRESSURE);

		/* Set the PKO to think command buffers are an odd length. This makes
		   it so we never have to divide a comamnd across two buffers */
		config.u64 = 0;
		config.s.pool = CVMX_FPA_OUTPUT_BUFFER_POOL;
#if USE_OPTIMIZED_PKO_BUFFER_SIZE
		config.s.size = CVMX_FPA_OUTPUT_BUFFER_POOL_SIZE / 8 - 1;
#else
		config.s.size = CVMX_FPA_OUTPUT_BUFFER_POOL_SIZE / 8;
#endif
		cvmx_write_csr(CVMX_PKO_REG_CMD_BUF, config.u64);

		/* PKO registers */
		pko_mode.u64 = 0;
		pko_mode.s.mode0 = 2;	/* 16 >> 2 == 4 ports */
		pko_mode.s.mode1 = 2;	/* 16 >> 2 == 4 ports */
		cvmx_write_csr(CVMX_PKO_REG_GMX_PORT_MODE, pko_mode.u64);
		cvmx_pko_enable();
	}

	/* Pass 1 PKI-12 Errata: Ignore ipv4 header checksum violations */
	if (octeon_is_pass1()) {
		cvmx_pip_gbl_ctl_t global_control;
		global_control.u64 = cvmx_read_csr(CVMX_PIP_GBL_CTL);
		global_control.s.ip_chk = 0;
		cvmx_write_csr(CVMX_PIP_GBL_CTL, global_control.u64);
	}

	/* Enable the MII interface */
	cvmx_write_csr(CVMX_SMI_EN, 1);

	/* Register an IRQ hander for irq 2 to receive POW interrupts */
	request_irq(8 + pow_receive_group, cvm_oct_do_interrupt, SA_SHIRQ,
		    "Ethernet", cvm_oct_device);

#if USE_MULTICORE_RECEIVE
	/* This code enables interrupt processing by all other cores as well. It is
	   disabled due to stability issues */
	{
		int cpu;
		for (cpu = 0; cpu < NR_CPUS; cpu++) {
			if (cpu_online(cpu) && (cpu != smp_processor_id())) {
				cvmx_ciu_intx0_t en;
				en.u64 =
				    cvmx_read_csr(CVMX_CIU_INTX_EN0
						  (cpu_logical_map(cpu) * 2));
				en.s.workq |= (1 << pow_receive_group);
				cvmx_write_csr(CVMX_CIU_INTX_EN0
					       (cpu_logical_map(cpu) * 2),
					       en.u64);
			}
		}
	}
#endif
}

/**
 * Configure per interface settings
 *
 * @param interface Interface to configure (0 or 1)
 */
static int cvm_oct_configure_interface(int interface, int isRGMII)
{
	if (isRGMII) {
		cvmx_write_csr(CVMX_ASXX_TX_PRT_EN(interface), 0xf);
		cvmx_write_csr(CVMX_ASXX_RX_PRT_EN(interface), 0xf);
		cvmx_write_csr(CVMX_GMXX_TX_PRTS(interface), 4);
		return 0;
	} else {
		int result = cvmx_spi4000_initialize(interface);
		if (result)
			printk("\t\tSPI4000 not found\n");
		return result;
	}
}

/**
 * Configure the RGMII port for the negotiated speed
 *
 * @param dev    Linux device for the RGMII port
 */
static void cvm_oct_configure_rgmii_speed(struct net_device *dev)
{
	cvm_oct_private_t *priv = (cvm_oct_private_t *) dev->priv;
	int interface = INTERFACE(priv->port);
	int index = INDEX(priv->port);
	const char *link_duplex;
	const char *link_speed;
	const char *link_status;
	cvmx_gmxx_prtx_cfg_t gmx_cfg;

	cvmx_gmxx_rxx_rx_inbnd_t link;
	link.u64 = cvmx_read_csr(CVMX_GMXX_RXX_RX_INBND(index, interface));

	/* Return if we've already setup properly */
	if (link.u64 == priv->link_status)
		return;

	priv->link_status = link.u64;

	gmx_cfg.u64 = cvmx_read_csr(CVMX_GMXX_PRTX_CFG(index, interface));

	if (octeon_is_pass1())
		gmx_cfg.s.duplex = 1;	/* Half duplex is broken for Pass 1 */
	else
		gmx_cfg.s.duplex = link.s.duplex;

	if (link.s.duplex)
		link_duplex = "Full";
	else
		link_duplex = "Half";

	if (link.s.speed == 0) {
		link_speed = " 10Mbs";
		gmx_cfg.s.slottime = 0;
		gmx_cfg.s.speed = 0;
		cvmx_write_csr(CVMX_GMXX_TXX_CLK(index, interface), 50);
		cvmx_write_csr(CVMX_GMXX_TXX_SLOT(index, interface), 0x40);
		cvmx_write_csr(CVMX_GMXX_TXX_BURST(index, interface), 0);
	} else if (link.s.speed == 1) {
		link_speed = "100Mbs";
		gmx_cfg.s.slottime = 0;
		gmx_cfg.s.speed = 0;
		cvmx_write_csr(CVMX_GMXX_TXX_CLK(index, interface), 5);
		cvmx_write_csr(CVMX_GMXX_TXX_SLOT(index, interface), 0x40);
		cvmx_write_csr(CVMX_GMXX_TXX_BURST(index, interface), 0);
	} else if (link.s.speed == 2) {
		link_speed = "  1Gbs";
		gmx_cfg.s.slottime = 1;
		gmx_cfg.s.speed = 1;
		cvmx_write_csr(CVMX_GMXX_TXX_CLK(index, interface), 1);
		cvmx_write_csr(CVMX_GMXX_TXX_SLOT(index, interface), 0x200);
		cvmx_write_csr(CVMX_GMXX_TXX_BURST(index, interface), 0x2000);
	} else {
		link_speed = " Rsrvd";
		gmx_cfg.s.slottime = 1;
		gmx_cfg.s.speed = 1;
		cvmx_write_csr(CVMX_GMXX_TXX_CLK(index, interface), 1);
		cvmx_write_csr(CVMX_GMXX_TXX_SLOT(index, interface), 0x200);
		cvmx_write_csr(CVMX_GMXX_TXX_BURST(index, interface), 0x2000);
	}

	if (link.s.status)
		link_status = "Up  ";
	else
		link_status = "Down";

	cvmx_write_csr(CVMX_GMXX_PRTX_CFG(index, interface), gmx_cfg.u64);

	if (priv->queue != -1)
		DEBUGPRINT("\t\t%s: %s %s %s duplex, port %2d, queue %2d\n",
			   dev->name, link_status, link_speed, link_duplex,
			   priv->port, priv->queue);
	else
		DEBUGPRINT("\t\t%s: %s %s %s duplex, port %2d, POW\n",
			   dev->name, link_status, link_speed, link_duplex,
			   priv->port);
}

/**
 * Configure an individual RGMII interface
 *
 * @param dev    Linux ethernet device for the RGMII
 */
static void cvm_oct_configure_rgmii_port(struct net_device *dev)
{
	cvm_oct_private_t *priv = (cvm_oct_private_t *) dev->priv;
	int interface = INTERFACE(priv->port);
	int index = INDEX(priv->port);

	cvm_oct_configure_rgmii_speed(dev);

	cvmx_write_csr(CVMX_GMXX_TXX_THRESH(index, interface), 32);

#ifndef CONFIG_CAVIUM_OCTEON_SIMULATOR
	if (octeon_is_pass1()) {
		/* Set hi water mark as per errata GMX-4 */
		if (octeon_bootinfo->eclock_hz >= 325000000
		    && octeon_bootinfo->eclock_hz < 375000000)
			cvmx_write_csr(CVMX_ASXX_TX_HI_WATERX(index, interface),
				       12);
		else if (octeon_bootinfo->eclock_hz >= 375000000
			 && octeon_bootinfo->eclock_hz < 437000000)
			cvmx_write_csr(CVMX_ASXX_TX_HI_WATERX(index, interface),
				       11);
		else if (octeon_bootinfo->eclock_hz >= 437000000
			 && octeon_bootinfo->eclock_hz < 550000000)
			cvmx_write_csr(CVMX_ASXX_TX_HI_WATERX(index, interface),
				       10);
		else if (octeon_bootinfo->eclock_hz >= 550000000
			 && octeon_bootinfo->eclock_hz < 687000000)
			cvmx_write_csr(CVMX_ASXX_TX_HI_WATERX(index, interface),
				       9);
		else
			printk
			    ("Unable to set high water marks. Unsupported frequency %u\n",
			     octeon_bootinfo->eclock_hz);
	}
#endif

	cvmx_write_csr(CVMX_ASXX_TX_CLK_SETX(index, interface), 24);
	cvmx_write_csr(CVMX_ASXX_RX_CLK_SETX(index, interface), 24);
}

/**
 * Perform any SPI per port config. Currently there isn't any.
 *
 * @param dev    Device to configure
 */
static void cvm_oct_configure_spi_port(struct net_device *dev)
{
	cvm_oct_private_t *priv = (cvm_oct_private_t *) dev->priv;
	if (priv->queue != -1)
		printk("\t\t%s: SPI4000 port %2d, queue %2d\n", dev->name,
		       priv->port, priv->queue);
	else
		printk("\t\t%s: SPI4000 port %2d, POW\n", dev->name,
		       priv->port);
}

/**
 * Perform per port configuration.
 *
 * @param dev    Device to configure
 */
static void cvm_oct_configure_port(struct net_device *dev)
{
	cvm_oct_private_t *priv = (cvm_oct_private_t *) dev->priv;
	const uint64_t priorities[8] = { 8, 8, 8, 8, 8, 8, 8, 8 };
	cvmx_pip_port_cfg_t port_config;
	cvmx_pip_port_tag_cfg_t tag_config;

	if (priv->isRGMII)
		cvm_oct_configure_rgmii_port(dev);
	else
		cvm_oct_configure_spi_port(dev);

	/* Packet output configures Queue and Ports */
	cvmx_pko_config_port(priv->port, cvmx_pko_get_base_queue(priv->port),
			     cvmx_pko_get_num_queues(priv->port), priorities);

	/* Setup Port input tagging */
	port_config.u64 = 0;
	port_config.s.mode = CVMX_PIP_PORT_CFG_MODE_SKIPL2;	/* Process the headers and place the IP header in the work queue */
	port_config.s.qos = priv->port & 0x7;
	port_config.s.crc_en = 1;

	tag_config.u64 = 0;
	tag_config.s.ip6_src_flag = CVMX_HELPER_INPUT_TAG_IPV6_SRC_IP;
	tag_config.s.ip6_dst_flag = CVMX_HELPER_INPUT_TAG_IPV6_DST_IP;
	tag_config.s.ip6_sprt_flag = CVMX_HELPER_INPUT_TAG_IPV6_SRC_PORT;
	tag_config.s.ip6_dprt_flag = CVMX_HELPER_INPUT_TAG_IPV6_DST_PORT;
	tag_config.s.ip6_nxth_flag = CVMX_HELPER_INPUT_TAG_IPV6_NEXT_HEADER;
	tag_config.s.ip4_src_flag = CVMX_HELPER_INPUT_TAG_IPV4_SRC_IP;
	tag_config.s.ip4_dst_flag = CVMX_HELPER_INPUT_TAG_IPV4_DST_IP;
	tag_config.s.ip4_sprt_flag = CVMX_HELPER_INPUT_TAG_IPV4_SRC_PORT;
	tag_config.s.ip4_dprt_flag = CVMX_HELPER_INPUT_TAG_IPV4_DST_PORT;
	tag_config.s.ip4_pctl_flag = CVMX_HELPER_INPUT_TAG_IPV4_PROTOCOL;
	tag_config.s.inc_prt_flag = CVMX_HELPER_INPUT_TAG_INPUT_PORT;
	tag_config.s.tcp6_tag_type = CVMX_HELPER_INPUT_TAG_TYPE;
	tag_config.s.tcp4_tag_type = CVMX_HELPER_INPUT_TAG_TYPE;
	tag_config.s.ip6_tag_type = CVMX_HELPER_INPUT_TAG_TYPE;
	tag_config.s.ip4_tag_type = CVMX_HELPER_INPUT_TAG_TYPE;
	tag_config.s.non_tag_type = CVMX_HELPER_INPUT_TAG_TYPE;
	tag_config.s.grp = pow_receive_group;

	cvmx_pip_config_port(priv->port, port_config, tag_config);
}

/**
 * Module/ driver initialization. Creates the linux network
 * devices.
 *
 * @return Zero on success
 */
static int __init cvm_oct_init_module(void)
{
	int max_devices = num_ethernet_devices;
	int interface;
	int fau = CVMX_FAU_REG_END - sizeof(uint32_t);
	int phy_id = 0;
	int i;

	printk("Octeon ethernet driver version: %s\n", cvm_oct_get_version());

	/* Initialize all of the tasklets */
	for (i = 0; i < NR_CPUS; i++)
		tasklet_init(cvm_oct_tasklet + i, cvm_oct_tasklet_rx, 0);

	memset(cvm_oct_device, 0, sizeof(cvm_oct_device));

	cvm_oct_configure_common_hw();

	if ((pow_send_group != -1)) {
		struct net_device *dev;
		printk("\tConfiguring device for POW only access\n");
		dev = alloc_etherdev(sizeof(cvm_oct_private_t));
		if (dev) {
			int err;
			cvm_oct_private_t *priv =
			    (cvm_oct_private_t *) dev->priv;

			SET_MODULE_OWNER(dev);
			dev->init = cvm_oct_init;

			/* Initialize the device private structure. */
			memset(priv, 0, sizeof(cvm_oct_private_t));
			priv->isRGMII = 0;
			priv->port = CVMX_PIP_NUM_INPUT_PORTS;
			priv->queue = -1;
			strcpy(dev->name, "pow%d");

			err = register_netdev(dev);
			if (err < 0) {
				printk
				    ("\t\tFailed to register ethernet device for POW\n");
				kfree(dev);
			} else {
				cvm_oct_device[CVMX_PIP_NUM_INPUT_PORTS] = dev;
				skb_queue_head_init(&priv->tx_free_list);
				printk
				    ("\t\t%s: POW send group %d, receive group %d\n",
				     dev->name, pow_send_group,
				     pow_receive_group);
			}
		} else {
			printk
			    ("\t\tFailed to allocate ethernet device for POW\n");
		}
	}

	if (num_ethernet_devices) {
		for (interface = 0; interface < 2; interface++) {
			cvmx_gmxx_inf_mode_t mode;
			mode.u64 = cvmx_read_csr(CVMX_GMXX_INF_MODE(interface));
			if (mode.s.en) {
				const char *name;
				int num_ports;
				int port;
				int isRGMII;

				if (mode.s.type) {
					printk("\tInterface %d is SPI4\n",
					       interface);
					name = "spi%d";
					num_ports = 10;
					isRGMII = 0;
				} else {
					printk("\tInterface %d is RGMII\n",
					       interface);
					name = "eth%d";
					num_ports = 4;
					isRGMII = 1;
				}

				if (num_ports) {
					if (cvm_oct_configure_interface
					    (interface, isRGMII))
						num_ports = 0;
				}
				for (port = interface * 16;
				     port < interface * 16 + num_ports;
				     port++) {
					struct net_device *dev;
					if (max_devices <= 0)
						continue;

					dev =
					    alloc_etherdev(sizeof
							   (cvm_oct_private_t));
					if (dev) {
						int err;
						cvm_oct_private_t *priv =
						    (cvm_oct_private_t *) dev->
						    priv;

						SET_MODULE_OWNER(dev);
						dev->init = cvm_oct_init;

						/* Initialize the device private structure. */
						memset(priv, 0,
						       sizeof
						       (cvm_oct_private_t));
						priv->isRGMII = isRGMII;
						priv->port = port;
						priv->mii_info.dev = dev;
						priv->mii_info.phy_id =
						    phy_id++;
						priv->mii_info.phy_id_mask =
						    0xff;
						priv->mii_info.reg_num_mask =
						    0x1f;
						priv->mii_info.mdio_read =
						    cvm_oct_mdio_read;
						priv->mii_info.mdio_write =
						    cvm_oct_mdio_write;
						strcpy(dev->name, name);

						err = register_netdev(dev);
						if (err < 0) {
							printk
							    ("\t\tFailed to register ethernet device for interface %d, port %d\n",
							     interface,
							     priv->port);
							kfree(dev);
						} else {
							cvm_oct_device[priv->
								       port] =
							    dev;
							/* Force the interface to use the POW send if always_use_pow was
							   specified or it is in the pow send list */
							if ((pow_send_group !=
							     -1)
							    && (always_use_pow
								||
								strstr
								(pow_send_list,
								 dev->name)))
								priv->queue =
								    -1;
							else
								priv->queue =
								    cvmx_pko_get_base_queue
								    (port);
							skb_queue_head_init
							    (&priv->
							     tx_free_list);
							priv->fau = fau;
							cvmx_fau_atomic_write32
							    (priv->fau, 0);
							cvm_oct_configure_port
							    (dev);
							fau -= sizeof(uint32_t);
							max_devices--;
						}
					} else {
						printk
						    ("\t\tFailed to allocate ethernet device for port %d\n",
						     port);
					}
				}
			} else {
				printk("\tInterface %d is disabled\n",
				       interface);
			}
		}
	}

	/* Set the POW timer rate to give an interrupt at most INTERRUPT_LIMIT times per second */
	cvmx_write_csr(CVMX_POW_WQ_INT_PC,
		       octeon_bootinfo->eclock_hz / (INTERRUPT_LIMIT * 16 *
						     256) << 8);

	/* Enable POW interrupt when our port has at least one packet */
	cvmx_write_csr(CVMX_POW_WQ_INT_THRX(pow_receive_group), 0x1ful << 24);

	/* Enable the poll timer for checking RGMII status */
	init_timer(&cvm_oct_poll_timer);
	cvm_oct_poll_timer.data = 0;
	cvm_oct_poll_timer.function = cvm_do_timer;
	mod_timer(&cvm_oct_poll_timer, jiffies + HZ * 2);

	cvmx_ipd_enable();
	return 0;
}

/**
 * Module / driver shutdown
 *
 * @return Zero on success
 */
static void __exit cvm_oct_cleanup_module(void)
{
	cvm_oct_private_t *priv;
	int port;
	int i;

	cvmx_ipd_disable();

	del_timer(&cvm_oct_poll_timer);

	/* Disable POW interrupt */
	cvmx_write_csr(CVMX_POW_WQ_INT_THRX(pow_receive_group), 0);

	/* Shutdown all of the tasklets */
	for (i = 0; i < NR_CPUS; i++)
		tasklet_kill(cvm_oct_tasklet + i);

	/* Free the interrupt handler */
	free_irq(8 + pow_receive_group, cvm_oct_device);

	/* Free the ethernet devices */
	for (port = 0; port <= CVMX_PIP_NUM_INPUT_PORTS; port++) {
		if (cvm_oct_device[port]) {
			priv = (cvm_oct_private_t *) cvm_oct_device[port]->priv;

			/* Free buffers */
			while (skb_queue_len(&priv->tx_free_list))
				dev_kfree_skb(skb_dequeue(&priv->tx_free_list));

			unregister_netdev(cvm_oct_device[port]);
			kfree(cvm_oct_device[port]);
			cvm_oct_device[port] = NULL;
		}
	}

	cvmx_pko_shutdown();

	/* Free the HW pools */
#if USE_SKBUFFS_IN_HW
	cvm_oct_free_hw_skbuff(CVMX_FPA_PACKET_POOL, CVMX_FPA_PACKET_POOL_SIZE,
			       NUM_PACKET_BUFFERS);
#else
	cvm_oct_free_hw_memory(CVMX_FPA_PACKET_POOL, CVMX_FPA_PACKET_POOL_SIZE,
			       NUM_PACKET_BUFFERS);
#endif

	cvm_oct_free_hw_memory(CVMX_FPA_WQE_POOL, CVMX_FPA_WQE_POOL_SIZE,
			       NUM_PACKET_BUFFERS);
#if CVMX_FPA_OUTPUT_BUFFER_POOL != CVMX_FPA_PACKET_POOL
	cvm_oct_free_hw_memory(CVMX_FPA_OUTPUT_BUFFER_POOL,
			       CVMX_FPA_OUTPUT_BUFFER_POOL_SIZE, 128);
#endif
}

MODULE_LICENSE("Proprietary");
MODULE_AUTHOR("Cavium Networks <support@caviumnetworks.com>");
MODULE_DESCRIPTION("Cavium Networks Octeon ethernet driver.");
module_init(cvm_oct_init_module);
module_exit(cvm_oct_cleanup_module);
