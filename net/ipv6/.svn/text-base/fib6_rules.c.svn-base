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
 *
 * Linux INET6 implementation 
 * Forwarding Information Database: policy rules
 *
 * Authors
 *	Ville Nuorvala		<vnuorval@tcs.hut.fi>	
 *
 * This file is derived from net/ipv4/fib_rules.c
 */

#include <linux/config.h>
#include <linux/netdevice.h>
#include <linux/netlink.h>

#include <net/ipv6.h>
#include <net/ip6_route.h>
#include <net/ip6_fib.h>


struct fib6_rule
{
	struct fib6_rule *r_next;
	atomic_t	r_clntref;
	u32		r_preference;
	unsigned char	r_table;
	unsigned char	r_action;
	unsigned char	r_dst_len;
	unsigned char	r_src_len;
	struct in6_addr r_src;
	struct in6_addr r_dst;
	u8		r_flags;
	u8		r_tclass;
	int		r_ifindex;
	char		r_ifname[IFNAMSIZ];
	int		r_dead;
};

static struct fib6_rule main_rule = {
	.r_clntref =	ATOMIC_INIT(2),
	.r_preference =	0x7FFE,
	.r_table =	RT6_TABLE_MAIN,
	.r_action =	RTN_UNICAST,
};

static struct fib6_rule local_rule = {
	.r_next =	&main_rule,
	.r_clntref =	ATOMIC_INIT(2),
	.r_table =	RT6_TABLE_LOCAL,
	.r_action =	RTN_UNICAST,
};

static struct fib6_rule *fib6_rules = &local_rule;
static rwlock_t fib6_rules_lock = RW_LOCK_UNLOCKED;

void fib6_rule_put(struct fib6_rule *r)
{
	if (atomic_dec_and_test(&r->r_clntref)) {
		if (r->r_dead)
			kfree(r);
		else
			printk("Freeing alive rule %p\n", r);
	}
}

int inet6_rtm_delrule(struct sk_buff *skb, struct nlmsghdr* nlh, void *arg)
{
	struct rtattr **rta = arg;
	struct rtmsg *rtm = NLMSG_DATA(nlh);
	struct fib6_rule *r, **rp;
	int err = -ESRCH;

	write_lock_bh(&fib6_rules_lock);
	for (rp = &fib6_rules; (r = *rp) != NULL; rp = &r->r_next) {
		if ((!rta[RTA_SRC-1] || 
		     !memcmp(RTA_DATA(rta[RTA_SRC-1]),
			     &r->r_src, sizeof(struct in6_addr))) &&
		    rtm->rtm_src_len == r->r_src_len &&
		    (!rta[RTA_DST-1] || 
		     !memcmp(RTA_DATA(rta[RTA_DST-1]), 
			     &r->r_dst, sizeof(struct in6_addr))) &&
		    rtm->rtm_dst_len == r->r_dst_len &&
		    rtm->rtm_tos == r->r_tclass &&
		    (!rtm->rtm_type || rtm->rtm_type == r->r_action) &&
		    (!rta[RTA_PRIORITY-1] || 
		     !memcmp(RTA_DATA(rta[RTA_PRIORITY-1]), 
			     &r->r_preference, 4)) &&
		    (!rta[RTA_IIF-1] || 
		     !strcmp(RTA_DATA(rta[RTA_IIF-1]), r->r_ifname)) &&
		    (!rtm->rtm_table || (r && rtm->rtm_table == r->r_table))) {
			err = -EPERM;
			if (r == &local_rule)
				break;

			*rp = r->r_next;
			r->r_dead = 1;
			fib6_rule_put(r);
			err = 0;
			break;
		}
	}
	write_unlock_bh(&fib6_rules_lock);
	return err;
}

/* Allocate new unique table id */

static inline int rt6_empty_table(void)
{
	int id;

	write_lock_bh(&rt6_lock);
	for (id = 1; id <= RT6_TABLE_MAX; id++)
		if (rt6_tables[id] == NULL && __rt6_new_table(id) != NULL)
			break;
	write_unlock_bh(&rt6_lock);

	return id <= RT6_TABLE_MAX ? id : 0;
}

int inet6_rtm_newrule(struct sk_buff *skb, struct nlmsghdr* nlh, void *arg)
{
	struct rtattr **rta = arg;
	struct rtmsg *rtm = NLMSG_DATA(nlh);
	struct fib6_rule *r, *new_r, **rp;
	unsigned char table_id;

	if (rtm->rtm_src_len > 128 || rtm->rtm_dst_len > 128 ||
	    (rtm->rtm_tos & ~IPV6_FLOWINFO_MASK))
		return -EINVAL;

	if (rta[RTA_IIF-1] && RTA_PAYLOAD(rta[RTA_IIF-1]) > IFNAMSIZ)
		return -EINVAL;
	
	table_id = rtm->rtm_table;
	
	if (rtm->rtm_type == RTN_UNICAST) {
		if (table_id) {
			if (!rt6_new_table(table_id))
				return -ENOBUFS;
		} else if (!(table_id = rt6_empty_table()))
			return -ENOBUFS;
	}
	new_r = kmalloc(sizeof(*new_r), GFP_KERNEL);
	if (!new_r)
		return -ENOMEM;
	memset(new_r, 0, sizeof(*new_r));
	if (rta[RTA_SRC-1])
		memcpy(&new_r->r_src, RTA_DATA(rta[RTA_SRC-1]), 
		       sizeof(struct in6_addr));
	if (rta[RTA_DST-1])
		memcpy(&new_r->r_dst, RTA_DATA(rta[RTA_DST-1]), 
		       sizeof(struct in6_addr));
	new_r->r_src_len = rtm->rtm_src_len;
	new_r->r_dst_len = rtm->rtm_dst_len;
	new_r->r_tclass = rtm->rtm_tos;
	new_r->r_action = rtm->rtm_type;
	new_r->r_flags = rtm->rtm_flags;
	if (rta[RTA_PRIORITY-1])
		memcpy(&new_r->r_preference, RTA_DATA(rta[RTA_PRIORITY-1]), 4);
	new_r->r_table = table_id;
	if (rta[RTA_IIF-1]) {
		struct net_device *dev;
		memcpy(new_r->r_ifname, RTA_DATA(rta[RTA_IIF-1]), IFNAMSIZ);
		new_r->r_ifname[IFNAMSIZ-1] = 0;
		new_r->r_ifindex = -1;
		dev = __dev_get_by_name(new_r->r_ifname);
		if (dev)
			new_r->r_ifindex = dev->ifindex;
	}

	write_lock_bh(&fib6_rules_lock);
	rp = &fib6_rules;
	if (!new_r->r_preference) {
		r = fib6_rules;
		if (r && (r = r->r_next) != NULL) {
			rp = &fib6_rules->r_next;
			if (r->r_preference)
				new_r->r_preference = r->r_preference - 1;
		}
	}

	while ( (r = *rp) != NULL ) {
		if (r->r_preference > new_r->r_preference)
			break;
		rp = &r->r_next;
	}

	new_r->r_next = r;
	atomic_inc(&new_r->r_clntref);
	*rp = new_r;

	write_unlock_bh(&fib6_rules_lock);

	return 0;
}

static void fib6_rules_detach(struct net_device *dev)
{
	struct fib6_rule *r;

	write_lock_bh(&fib6_rules_lock);

	for (r = fib6_rules; r; r = r->r_next)
		if (r->r_ifindex == dev->ifindex)
			r->r_ifindex = -1;

	write_unlock_bh(&fib6_rules_lock);
}

static void fib6_rules_attach(struct net_device *dev)
{
	struct fib6_rule *r;

	write_lock_bh(&fib6_rules_lock);

	for (r = fib6_rules; r; r = r->r_next)
		if (r->r_ifindex == -1 && strcmp(dev->name, r->r_ifname) == 0)
			r->r_ifindex = dev->ifindex;

	write_unlock_bh(&fib6_rules_lock);
}


struct dst_entry *
rt6_rule_lookup(struct flowi *fl, 
		struct rt6_info *(* tb_lookup)(struct rt6_table *table, 
					       struct flowi *fl, int flags),
		int flags)
{
	struct fib6_rule *r;
	struct rt6_info *rt = NULL;
	int src_any = ipv6_addr_any(&fl->fl6_src);

	read_lock_bh(&fib6_rules_lock);
	read_lock(&rt6_lock);
	for (r = fib6_rules; r; r = r->r_next) {
		if ((!src_any && 
		     ipv6_prefix_cmp(&fl->fl6_src, &r->r_src, r->r_src_len)) ||
		    ipv6_prefix_cmp(&fl->fl6_dst, &r->r_dst, r->r_dst_len) ||
		    (r->r_ifindex && fl->iif && r->r_ifindex != fl->iif))
			continue;

		switch (r->r_action) {
		case RTN_UNICAST:
			break;
		case RTN_UNREACHABLE:
			rt = &ip6_null_entry;
			goto discard_pkt;
		case RTN_BLACKHOLE:
			rt = &ip6_blk_hole_entry;
			goto discard_pkt;
		case RTN_PROHIBIT:
			rt = &ip6_prohibit_entry;
			goto discard_pkt;
		}

		if (!(rt = tb_lookup(rt6_get_table(r->r_table), fl, flags)))
		    continue;

		if (src_any && ipv6_prefix_cmp(&fl->fl6_src, &r->r_src, r->r_src_len)) {
			fl->fl6_src = in6addr_any;
			dst_release(&rt->u.dst);
		} else if (rt == &ip6_null_entry) {
			dst_release(&rt->u.dst);
		} else {
			read_unlock(&rt6_lock);
			read_unlock_bh(&fib6_rules_lock);
			goto out;
		}
	}
	rt = &ip6_null_entry;
discard_pkt:
	read_unlock(&rt6_lock);
	read_unlock_bh(&fib6_rules_lock);
	dst_hold(&rt->u.dst);
out:
	return &rt->u.dst;
}

static int fib6_rules_event(struct notifier_block *this, unsigned long event, void *ptr)
{
	struct net_device *dev = ptr;

	if (event == NETDEV_UNREGISTER)
		fib6_rules_detach(dev);
	else if (event == NETDEV_REGISTER)
		fib6_rules_attach(dev);
	return NOTIFY_DONE;
}


struct notifier_block fib6_rules_notifier = {
	.notifier_call = fib6_rules_event,
};

static __inline__ int inet6_fill_rule(struct sk_buff *skb,
				     struct fib6_rule *r,
				     struct netlink_callback *cb)
{
	struct rtmsg *rtm;
	struct nlmsghdr  *nlh;
	unsigned char	 *b = skb->tail;

	nlh = NLMSG_PUT(skb, NETLINK_CREDS(cb->skb)->pid, cb->nlh->nlmsg_seq, RTM_NEWRULE, sizeof(*rtm));
	rtm = NLMSG_DATA(nlh);
	rtm->rtm_family = AF_INET6;
	rtm->rtm_dst_len = r->r_dst_len;
	rtm->rtm_src_len = r->r_src_len;

	rtm->rtm_table = r->r_table;
	rtm->rtm_protocol = 0;
	rtm->rtm_scope = 0;
	rtm->rtm_type = r->r_action;
	rtm->rtm_flags = r->r_flags;
	rtm->rtm_tos = r->r_tclass;

	if (r->r_dst_len)
		RTA_PUT(skb, RTA_DST, sizeof(struct in6_addr), &r->r_dst);
	if (r->r_src_len)
		RTA_PUT(skb, RTA_SRC, sizeof(struct in6_addr), &r->r_src);
	if (r->r_ifname[0])
		RTA_PUT(skb, RTA_IIF, IFNAMSIZ, &r->r_ifname);
	if (r->r_preference)
		RTA_PUT(skb, RTA_PRIORITY, 4, &r->r_preference);
	nlh->nlmsg_len = skb->tail - b;
	return skb->len;

nlmsg_failure:
rtattr_failure:
	skb_put(skb, b - skb->tail);
	return -1;
}

int inet6_dump_rules(struct sk_buff *skb, struct netlink_callback *cb)
{
	int idx;
	int s_idx = cb->args[0];
	struct fib6_rule *r;

	read_lock_bh(&fib6_rules_lock);
	for (r = fib6_rules, idx = 0; r; r = r->r_next, idx++) {
		if (idx < s_idx)
			continue;
		if (inet6_fill_rule(skb, r, cb) < 0)
			break;
	}
	read_unlock_bh(&fib6_rules_lock);
	cb->args[0] = idx;

	return skb->len;
}

void __init fib6_rules_init(void)
{
	register_netdevice_notifier(&fib6_rules_notifier);
}

void fib6_rules_exit(void)
{
	struct fib6_rule *r, **rp;

	unregister_netdevice_notifier(&fib6_rules_notifier);

	write_lock_bh(&fib6_rules_lock);
	for (rp = &fib6_rules; (r = *rp) != NULL; rp = &r->r_next) {
		*rp = r->r_next;
		r->r_dead = 1;
		fib6_rule_put(r);
	}
	write_unlock_bh(&fib6_rules_lock);
}

