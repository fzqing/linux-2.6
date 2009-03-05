#ifndef _NET_IP6_ROUTE_H
#define _NET_IP6_ROUTE_H

#define IP6_RT_PRIO_FW		16
#define IP6_RT_PRIO_ADDRCONF	256
#define IP6_RT_PRIO_KERN	512
#define IP6_RT_PRIO_USER 	1024
#define IP6_RT_FLOW_MASK	0x00ff

#ifdef __KERNEL__

#include <net/flow.h>
#include <net/ip6_fib.h>
#include <net/sock.h>
#include <linux/tcp.h>
#include <linux/ip.h>
#include <linux/ipv6.h>

#define RT6_TABLE_UNSPEC RT_TABLE_UNSPEC
#define RT6_TABLE_MAIN RT_TABLE_MAIN

#ifdef CONFIG_IPV6_MULTIPLE_TABLES
#define RT6_TABLE_MIN 1
#define RT6_TABLE_LOCAL RT_TABLE_LOCAL
#define RT6_TABLE_MAX RT_TABLE_MAX
#else
#define RT6_TABLE_MIN RT6_TABLE_MAIN
#define RT6_TABLE_LOCAL RT6_TABLE_MIN
#define RT6_TABLE_MAX RT6_TABLE_MIN
#endif


struct rt6_table {
	struct fib6_node root;
	rwlock_t lock;
};

#ifdef CONFIG_IPV6_SUBTREES

#define FIB6_SUBTREE(fn) ((fn)->subtree)

extern struct fib6_node		*fib6_subtree_lookup(struct fib6_node *root,
					struct in6_addr *saddr);

#else

#define FIB6_SUBTREE(fn) NULL

static inline struct fib6_node         *
fib6_subtree_lookup(struct fib6_node *root, struct in6_addr *saddr)
{
	return root;
}
#endif /* CONFIG_IPV6_SUBTREES */

extern rwlock_t rt6_lock;

extern struct rt6_info	ip6_null_entry;

#ifdef CONFIG_IPV6_MULTIPLE_TABLES

extern struct rt6_info ip6_prohibit_entry;
extern struct rt6_info ip6_blk_hole_entry;

extern int inet6_rtm_delrule(struct sk_buff *skb, struct nlmsghdr* nlh, void *arg);

extern int inet6_rtm_newrule(struct sk_buff *skb, struct nlmsghdr* nlh, void *arg);

extern int inet6_dump_rules(struct sk_buff *skb, struct netlink_callback *cb);

extern void __init fib6_rules_init(void);

extern void fib6_rules_exit(void);

extern struct rt6_table *rt6_tables[RT6_TABLE_MAX + 1];

#else

extern struct rt6_table        ip6_routing_table;

#endif /* CONFIG_IPV6_MULTIPLE_TABLES */

extern int ip6_rt_gc_interval;

extern struct rt6_info *rt6_fl_tree_input(struct rt6_table *table,
					  struct flowi *fl, int);

extern struct rt6_info *rt6_fl_tree_output(struct rt6_table *table,
					   struct flowi *fl, int);

extern struct rt6_info *rt6_fl_tree_lookup(struct rt6_table *table,
					   struct flowi *fl, int);

#ifdef CONFIG_IPV6_MULTIPLE_TABLES
extern struct dst_entry *
rt6_rule_lookup(struct flowi *fl,
		struct rt6_info *( *tb_fl_lookup)(struct rt6_table *table,
						  struct flowi *fl, int),
		int flags);
#else
static inline struct dst_entry *
rt6_rule_lookup(struct flowi *fl,
		struct rt6_info *( *tb_fl_lookup)(struct rt6_table *table,
						  struct flowi *fl, int),
		int flags)
{
	struct rt6_info *rt;
	read_lock_bh(&rt6_lock);
	rt = tb_fl_lookup(&ip6_routing_table, fl, flags);
	read_unlock_bh(&rt6_lock);
	return (struct dst_entry *) rt;
}
#endif /*  CONFIG_IPV6_MULTIPLE_TABLES */

static inline void ip6_route_input(struct sk_buff *skb)
{
	struct ipv6hdr *iph = skb->nh.ipv6h;
	int strict = ipv6_addr_type(&iph->saddr) & (IPV6_ADDR_MULTICAST|IPV6_ADDR_LINKLOCAL);

	struct flowi fl = {
		.iif = ((struct inet6_skb_parm *)skb->cb)->iif,
		.oif = ((struct inet6_skb_parm *)skb->cb)->iif,
		.nl_u =
		{ .ip6_u =
		  { .daddr = iph->daddr,
		    .saddr = iph->saddr,
		    .flowlabel = (* (__u32 *) iph)&IPV6_FLOWINFO_MASK, } },
		.proto = iph->nexthdr,
	};
	skb->dst = rt6_rule_lookup(&fl, rt6_fl_tree_input, strict);
}

static inline struct dst_entry *ip6_route_output(struct sock *sk, struct flowi *fl)
{
	int strict = ipv6_addr_type(&fl->fl6_dst) & (IPV6_ADDR_MULTICAST|IPV6_ADDR_LINKLOCAL);

	return rt6_rule_lookup(fl, rt6_fl_tree_output, strict);
}

extern int			ip6_route_me_harder(struct sk_buff *skb);

extern int			ip6_route_init(void);
extern void			ip6_route_cleanup(void);

extern int			ipv6_route_ioctl(unsigned int cmd, void __user *arg);

extern int			ip6_tb_route_add(int table_id,
						 struct in6_rtmsg *rtmsg,
						 struct nlmsghdr *,
						 void *rtattr);
int				rt6_ins(struct rt6_info *,
					   struct nlmsghdr *,
					   void *rtattr);
extern int			ip6_del_rt(struct rt6_info *,
					   struct nlmsghdr *,
					   void *rtattr);

extern int			ip6_rt_addr_add(struct in6_addr *addr,
						struct net_device *dev,
						int anycast);

extern int			ip6_rt_addr_del(struct in6_addr *addr,
						struct net_device *dev);

extern void			rt6_sndmsg(int type, struct in6_addr *dst,
					   struct in6_addr *src,
					   struct in6_addr *gw,
					   struct net_device *dev, 
					   int dstlen, int srclen,
					   int metric, __u32 flags);

extern struct dst_entry *ndisc_dst_alloc(struct net_device *dev,
					 struct neighbour *neigh,
					 struct in6_addr *addr,
					 int (*output)(struct sk_buff *));

int ip6_dontroute_dst_alloc(struct dst_entry **dst, struct flowi *fl);

extern int ndisc_dst_gc(int *more);
extern void fib6_force_start_gc(void);

extern struct rt6_info *addrconf_dst_alloc(const struct in6_addr *addr,
					   int anycast);

/*
 *	support functions for ND
 *
 */
extern struct rt6_info *	rt6_get_dflt_router(struct in6_addr *addr,
						    struct net_device *dev);
extern struct rt6_info *	rt6_add_dflt_router(struct in6_addr *gwaddr,
						    struct net_device *dev,
						    int pref);

extern void			rt6_purge_dflt_routers(void);

extern void			rt6_reset_dflt_pointer(struct rt6_info *rt);

extern void			rt6_redirect(struct in6_addr *dest,
					     struct in6_addr *saddr,
					     struct neighbour *neigh,
					     u8 *lladdr,
					     int on_link);

extern void			rt6_pmtu_discovery(struct in6_addr *daddr,
						   struct in6_addr *saddr,
						   struct net_device *dev,
						   u32 pmtu);

struct nlmsghdr;
struct netlink_callback;
extern int inet6_dump_fib(struct sk_buff *skb, struct netlink_callback *cb);
extern int inet6_rtm_newroute(struct sk_buff *skb, struct nlmsghdr* nlh, void *arg);
extern int inet6_rtm_delroute(struct sk_buff *skb, struct nlmsghdr* nlh, void *arg);
extern int inet6_rtm_getroute(struct sk_buff *skb, struct nlmsghdr* nlh, void *arg);

extern void rt6_ifdown(struct net_device *dev);
extern void rt6_mtu_change(struct net_device *dev, unsigned mtu);

/*
 *	Store a destination cache entry in a socket
 */
static inline void ip6_dst_store(struct sock *sk, struct dst_entry *dst,
				 struct in6_addr *daddr,
				 struct in6_addr *saddr)
{
	struct ipv6_pinfo *np = inet6_sk(sk);

	write_lock(&sk->sk_dst_lock);
	__sk_dst_set(sk, dst);
	np->daddr_cache = daddr;
#ifdef CONFIG_IPV6_SUBTREES
	np->saddr_cache = saddr;
#endif
	np->dst_cookie = atomic_read(&flow_cache_genid);
	write_unlock(&sk->sk_dst_lock);
}

static inline int ipv6_unicast_destination(struct sk_buff *skb)
{
	struct rt6_info *rt = (struct rt6_info *) skb->dst;

	return rt->rt6i_flags & RTF_LOCAL;
}

#ifdef CONFIG_IPV6_MULTIPLE_TABLES

extern struct rt6_table *__rt6_new_table(int id);

static inline struct rt6_table *rt6_get_table(int id)
{
	if (id == 0)
		id = RT6_TABLE_MAIN;

	return rt6_tables[id];
}

static inline struct rt6_table *rt6_new_table(int id)
{
	if (id == 0)
		id = RT6_TABLE_MAIN;

	return rt6_tables[id] ? : __rt6_new_table(id);
}

#else

static inline struct rt6_table *rt6_get_table(int id)
{
	return &ip6_routing_table;
}

static inline struct rt6_table *rt6_new_table(int id)
{
	return &ip6_routing_table;
}

#endif /*CONFIG_IPV6_MULTIPLE_TABLES */

static inline int ip6_route_add(struct in6_rtmsg *rtmsg, 
				struct nlmsghdr *nlh, void *rtattr)
{
	return ip6_tb_route_add(0, rtmsg, nlh, rtattr);
}

static inline struct rt6_info *rt6_tb_lookup(struct rt6_table *table,
					     struct in6_addr *daddr,
					     struct in6_addr *saddr,
					     int oif, int flags)
{
	struct flowi fl = {
		.oif = oif,
		.nl_u =
		{ .ip6_u =
		  { .daddr = *daddr,
		    .saddr = saddr ? *saddr : in6addr_any, } },
	};
	return rt6_fl_tree_lookup(table, &fl, flags);
}

static inline struct rt6_info *rt6_fl_lookup(struct flowi *fl, int flags)
{
	struct dst_entry *dst;
	if ((dst = rt6_rule_lookup(fl, rt6_fl_tree_lookup, flags))) {
		if (dst->error == 0)
			return (struct rt6_info *) dst;
		dst_release(dst);
	}
	return NULL;
}

static inline struct rt6_info *rt6_lookup(struct in6_addr *daddr,
					  struct in6_addr *saddr,
					  int oif, int flags)
{
	struct flowi fl = {
		.oif = oif,
		.nl_u =
		{ .ip6_u =
		  { .daddr = *daddr,
		    .saddr = saddr ? *saddr : in6addr_any, } },
	};
	return rt6_fl_lookup(&fl, flags);
}
#endif
#endif
