#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/sock_diag.h>
#include <linux/unix_diag.h>
#include <linux/skbuff.h>
#include <net/netlink.h>
#include <net/af_unix.h>
#include <net/tcp_states.h>

#define UNIX_DIAG_PUT(skb, attrtype, attrlen) \
	RTA_DATA(__RTA_PUT(skb, attrtype, attrlen))

static int sk_diag_fill(struct sock *sk, struct sk_buff *skb, struct unix_diag_req *req,
		u32 pid, u32 seq, u32 flags, int sk_ino)
{
	unsigned char *b = skb_tail_pointer(skb);
	struct nlmsghdr *nlh;
	struct unix_diag_msg *rep;

	nlh = NLMSG_PUT(skb, pid, seq, SOCK_DIAG_BY_FAMILY, sizeof(*rep));
	nlh->nlmsg_flags = flags;

	rep = NLMSG_DATA(nlh);

	rep->udiag_family = AF_UNIX;
	rep->udiag_type = sk->sk_type;
	rep->udiag_state = sk->sk_state;
	rep->udiag_ino = sk_ino;
	sock_diag_save_cookie(sk, rep->udiag_cookie);

	nlh->nlmsg_len = skb_tail_pointer(skb) - b;
	return skb->len;

nlmsg_failure:
	nlmsg_trim(skb, b);
	return -EMSGSIZE;
}

static int sk_diag_dump(struct sock *sk, struct sk_buff *skb, struct unix_diag_req *req,
		u32 pid, u32 seq, u32 flags)
{
	int sk_ino;

	unix_state_lock(sk);
	sk_ino = sock_i_ino(sk);
	unix_state_unlock(sk);

	if (!sk_ino)
		return 0;

	return sk_diag_fill(sk, skb, req, pid, seq, flags, sk_ino);
}

static int unix_diag_dump(struct sk_buff *skb, struct netlink_callback *cb)
{
	struct unix_diag_req *req;
	int num, s_num, slot, s_slot;

	req = NLMSG_DATA(cb->nlh);

	s_slot = cb->args[0];
	num = s_num = cb->args[1];

	spin_lock(&unix_table_lock);
	for (slot = s_slot; slot <= UNIX_HASH_SIZE; s_num = 0, slot++) {
		struct sock *sk;
		struct hlist_node *node;

		num = 0;
		sk_for_each(sk, node, &unix_socket_table[slot]) {
			if (num < s_num)
				goto next;
			if (!(req->udiag_states & (1 << sk->sk_state)))
				goto next;
			if (sk_diag_dump(sk, skb, req,
						NETLINK_CB(cb->skb).pid,
						cb->nlh->nlmsg_seq,
						NLM_F_MULTI) < 0)
				goto done;
next:
			num++;
		}
	}
done:
	spin_unlock(&unix_table_lock);
	cb->args[0] = slot;
	cb->args[1] = num;

	return skb->len;
}

static int unix_diag_get_exact(struct sk_buff *in_skb,
			       const struct nlmsghdr *nlh,
			       struct unix_diag_req *req)
{
	return -EAFNOSUPPORT;
}

static int unix_diag_handler_dump(struct sk_buff *skb, struct nlmsghdr *h)
{
	int hdrlen = sizeof(struct unix_diag_req);

	if (nlmsg_len(h) < hdrlen)
		return -EINVAL;

	if (h->nlmsg_flags & NLM_F_DUMP)
		return netlink_dump_start(sock_diag_nlsk, skb, h,
					  unix_diag_dump, NULL, 0);
	else
		return unix_diag_get_exact(skb, h, (struct unix_diag_req *)NLMSG_DATA(h));
}

static struct sock_diag_handler unix_diag_handler = {
	.family = AF_UNIX,
	.dump = unix_diag_handler_dump,
};

static int __init unix_diag_init(void)
{
	return sock_diag_register(&unix_diag_handler);
}

static void __exit unix_diag_exit(void)
{
	sock_diag_unregister(&unix_diag_handler);
}

module_init(unix_diag_init);
module_exit(unix_diag_exit);
MODULE_LICENSE("GPL");
MODULE_ALIAS_NET_PF_PROTO_TYPE(PF_NETLINK, NETLINK_SOCK_DIAG, 1 /* AF_LOCAL */);
