/*
 * Copyright (C)2003 Helsinki University of Technology
 * Copyright (C)2003 USAGI/WIDE Project
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
/*
 * Authors:
 *	Noriaki TAKAMIYA @USAGI
 *	Masahide NAKAMURA @USAGI
 *	YOSHIFUJI Hideaki @USAGI
 */
#ifndef _MIP6_H
#define _MIP6_H

#ifdef CONFIG_IPV6_MIP6_DEBUG
#define MIP6_DEBUG 3
#else
#define MIP6_DEBUG 2
#endif

#if MIP6_DEBUG >= 3
#define MIP6_DBG(x...) do { printk(KERN_DEBUG x); } while (0)
#else
#define MIP6_DBG(x...) do { ; } while (0)
#endif

#include <linux/skbuff.h>
#include <linux/in6.h>
#include <net/sock.h>

extern int mip6_init(void);
extern void mip6_fini(void);
extern int mip6_mh_filter(struct sock *sk, struct sk_buff *skb);
extern int mip6_destopt_place_find(struct sk_buff *skb, u8 **nexthdr);
extern int mip6_rthdr_place_find(struct sk_buff *skb, u8 **nexthdr);

/* XXX: Home Address Option in Destination Option Header */
struct destopt_hao
{
	__u8			type;
	__u8			length;
	struct in6_addr		addr;	/* Home Address */
} __attribute__ ((__packed__));


/*
 * Mobility Header
 */
struct ip6_mh {
	__u8	ip6mh_proto;
	__u8	ip6mh_hdrlen;
	__u8	ip6mh_type;
	__u8	ip6mh_reserved;
	__u16	ip6mh_cksum;
	/* Followed by type specific messages */
	__u8	data[0];
} __attribute__ ((__packed__));

struct ip6_mh_binding_update {
	struct ip6_mh ip6mhbu_hdr;
	__u16    ip6mhbu_seqno;      /* Sequence Number */
	__u16	 ip6mhbu_flags;
	__u16    ip6mhbu_lifetime; /* Time in unit of 4 sec */
	/* Followed by optional Mobility Options */
} __attribute__((packed));

#define	MIP6_OPT_PAD_1		0
#define	MIP6_OPT_PAD_N		1

#define IP6_MH_TYPE_BRR		0   /* Binding Refresh Request */
#define IP6_MH_TYPE_HOTI	1   /* HOTI Message   */
#define IP6_MH_TYPE_COTI	2   /* COTI Message  */
#define IP6_MH_TYPE_HOT		3   /* HOT Message   */
#define IP6_MH_TYPE_COT		4   /* COT Message  */
#define IP6_MH_TYPE_BU		5   /* Binding Update */
#define IP6_MH_TYPE_BACK	6   /* Binding ACK */
#define IP6_MH_TYPE_BERROR	7   /* Binding Error */
#define IP6_MH_TYPE_MAX		IP6_MH_TYPE_BERROR

/*
 *    Status values for the Binding Error mobility messages
 */
#define IP6_MH_BES_UNKNOWN_HAO	1 /* Unknown binding for HOA */
#define IP6_MH_BES_UNKNOWN_MH	2 /* Unknown MH Type */

#endif
