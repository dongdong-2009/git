/*
 * INET		An implementation of the TCP/IP protocol suite for the LINUX
 *		operating system.  INET is implemented using the  BSD Socket
 *		interface as the means of communication with the user level.
 *
 *		Pseudo-driver for the loopback interface.
 *
 * Version:	@(#)loopback.c	1.0.4b	08/16/93
 *
 * Authors:	Ross Biro
 *		Fred N. van Kempen, <waltje@uWalt.NL.Mugnet.ORG>
 *		Donald Becker, <becker@scyld.com>
 *
 *		Alan Cox	:	Fixed oddments for NET3.014
 *		Alan Cox	:	Rejig for NET3.029 snap #3
 *		Alan Cox	: 	Fixed NET3.029 bugs and sped up
 *		Larry McVoy	:	Tiny tweak to double performance
 *		Alan Cox	:	Backed out LMV's tweak - the linux mm
 *					can't take it...
 *              Michael Griffith:       Don't bother computing the checksums
 *                                      on packets received on the loopback
 *                                      interface.
 *		Alexey Kuznetsov:	Potential hang under some extreme
 *					cases removed.
 *
 *		This program is free software; you can redistribute it and/or
 *		modify it under the terms of the GNU General Public License
 *		as published by the Free Software Foundation; either version
 *		2 of the License, or (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/socket.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/in.h>
#include <linux/init.h>

#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include <linux/inet.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <net/sock.h>
#include <net/checksum.h>
#include <linux/if_ether.h>	/* For the statistics structure. */
#include <linux/if_arp.h>	/* For ARPHRD_ETHER */
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/percpu.h>
#include <net/net_namespace.h>

unsigned long bytes   = 0;
unsigned long packets = 0;

#if 0
struct pcpu_lstats {
	unsigned long packets;
	unsigned long bytes;
};

/*
 * The higher levels take care of making this non-reentrant (it's
 * called with bh's disabled).
 */
static int loopback_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct pcpu_lstats *pcpu_lstats, *lb_stats;

	skb_orphan(skb);

	skb->protocol = eth_type_trans(skb,dev);

	/* it's OK to use per_cpu_ptr() because BHs are off */
	pcpu_lstats = dev->ml_priv;
	lb_stats = per_cpu_ptr(pcpu_lstats, smp_processor_id());
	lb_stats->bytes += skb->len;
	lb_stats->packets++;

	netif_rx(skb);

	return 0;
}

static struct net_device_stats *loopback_get_stats(struct net_device *dev)
{
	const struct pcpu_lstats *pcpu_lstats;
	struct net_device_stats *stats = &dev->stats;
	unsigned long bytes = 0;
	unsigned long packets = 0;
	int i;

	pcpu_lstats = dev->ml_priv;
	for_each_possible_cpu(i) {
		const struct pcpu_lstats *lb_stats;

		lb_stats = per_cpu_ptr(pcpu_lstats, i);
		bytes   += lb_stats->bytes;
		packets += lb_stats->packets;
	}
	stats->rx_packets = packets;
	stats->tx_packets = packets;
	stats->rx_bytes = bytes;
	stats->tx_bytes = bytes;
	return stats;
}

static u32 always_on(struct net_device *dev)
{
	return 1;
}

static const struct ethtool_ops loopback_ethtool_ops = {
	.get_link		= always_on,
	.set_tso		= ethtool_op_set_tso,
	.get_tx_csum		= always_on,
	.get_sg			= always_on,
	.get_rx_csum		= always_on,
};

static int loopback_dev_init(struct net_device *dev)
{
	struct pcpu_lstats *lstats;

	lstats = alloc_percpu(struct pcpu_lstats);
	if (!lstats)
		return -ENOMEM;

	dev->ml_priv = lstats;
	return 0;
}

static void loopback_dev_free(struct net_device *dev)
{
	struct pcpu_lstats *lstats = dev->ml_priv;

	free_percpu(lstats);
	free_netdev(dev);
}

static const struct net_device_ops loopback_ops = {
	.ndo_init      = loopback_dev_init,
	.ndo_start_xmit= loopback_xmit,
	.ndo_get_stats = loopback_get_stats,
};

/*
 * The loopback device is special. There is only one instance
 * per network namespace.
 */
static void loopback_setup(struct net_device *dev)
{
	dev->mtu		= (16 * 1024) + 20 + 20 + 12;
	dev->hard_header_len	= ETH_HLEN;	/* 14	*/
	dev->addr_len		= ETH_ALEN;	/* 6	*/
	dev->tx_queue_len	= 0;
	dev->type		= ARPHRD_LOOPBACK;	/* 0x0001*/
	dev->flags		= IFF_LOOPBACK;
	dev->features 		= NETIF_F_SG | NETIF_F_FRAGLIST
		| NETIF_F_TSO
		| NETIF_F_NO_CSUM
		| NETIF_F_HIGHDMA
		| NETIF_F_LLTX
		| NETIF_F_NETNS_LOCAL;
	dev->ethtool_ops	= &loopback_ethtool_ops;
	dev->header_ops		= &eth_header_ops;
	dev->netdev_ops		= &loopback_ops;
	dev->destructor		= loopback_dev_free;
}
#endif

static int loopback_xmit(struct sk_buff *skb, struct net_device *dev)
{
        //数据发送，对于回环网卡来说，发送数据的时候不需要通知上层停止数据的发送
        //回环网卡也不是实实在在的硬件，故不需要将数据写入寄存器

	skb->protocol = eth_type_trans(skb,dev);                //指明数据包的协议
        bytes += skb->len;                                      //统计发送的字节数
        packets++;                                              //统计包的数目
        
        netif_rx(skb);                                          //将收到的数据再送回给协议栈，既将要发送的数据直接接收

	return 0;
}

static struct net_device_stats *loopback_get_stats(struct net_device *dev)
{
        struct net_device_stats *stats = &dev->stats;     //专门用于保存网卡的状态信息

        stats->rx_packets = packets;                      //接收到的字节和包的个数
        stats->tx_packets = packets;
        stats->rx_bytes   = bytes;
        stats->tx_bytes   = bytes;

        return stats;
}


static const struct net_device_ops loopback_ops = {
	.ndo_start_xmit= loopback_xmit,                 //发送函数，关于接收函数，对于回环网卡直接在接收函数里面
	.ndo_get_stats = loopback_get_stats,            //获取网卡状态
};

void static loopback_setup(struct net_device *dev)
{
        //2.在该函数中执行初始化dev的操作,主要是分配netdev_ops
        dev->mtu	= (16 * 1024) + 20 + 20 + 12;   //网卡所能接收的数据包的最大尺寸，定义为数据为16k，然后还要加上各种协议的头
        dev->flags	= IFF_LOOPBACK;                 //回环网卡专有标志
        dev->header_ops	= &eth_header_ops;              //当构造一个数据包时，需要构造一个头，就是用这个函数集来构造头的，这个函数已经由linux内核提供了，直接调用即可
        dev->netdev_ops = &loopback_ops;                //决定网卡所能支持的操作的集合
}





/* Setup and register the loopback device. */
static __net_init int loopback_net_init(struct net *net)
{
        struct net_device *dev;                         //申请回环网卡的结构

        //1.分配空间
        dev = alloc_netdev(0, "lo", loopback_setup);    //分配空间，loopback_setup是一个函数，当执行alloc_netdev分配空间时，会调用该函数
                                                        //该函数的功能，通常为所分配的device进行相应的初始化
                                                        //这里的lo名字就是在开发板上用ifconfig的时候显示的名称

        //2.

        //4.                                                
        register_netdev(dev);                           //将分配好的dev驱动注册给linux内核

        net->loopback_dev = dev;                        //将分配好的回环网卡网络告诉给网络
        return 0;

#if 0    
	struct net_device *dev;
	int err;

	err = -ENOMEM;
	dev = alloc_netdev(0, "lo", loopback_setup);
	if (!dev)
		goto out;

	dev_net_set(dev, net);
	err = register_netdev(dev);
	if (err)
		goto out_free_netdev;

	net->loopback_dev = dev;
	return 0;


out_free_netdev:
	free_netdev(dev);
out:
	if (net == &init_net)
		panic("loopback: Failed to register netdevice: %d\n", err);
	return err;
#endif    
}

static __net_exit void loopback_net_exit(struct net *net)
{
        struct net_device *dev = net->loopback_dev;
        unregister_netdev(dev);
        
#if 0    
	struct net_device *dev = net->loopback_dev;

	unregister_netdev(dev);
#endif
}

/* Registered in net/core/dev.c */
struct pernet_operations __net_initdata loopback_net_ops = {
       .init = loopback_net_init,
       .exit = loopback_net_exit,
};
