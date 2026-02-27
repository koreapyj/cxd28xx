// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ATSC 3.0 ALP (ATSC Link-layer Protocol) parser and network interface.
 *
 * Extracts IPv4 packets from ALP-encapsulated TS streams produced by
 * the CXD2878 demodulator (ALP_DIV_TS mode) and delivers them to the
 * Linux networking stack via a raw-IP point-to-point interface.
 *
 * Copyright (c) 2026 Yoonji Park <koreapyj@dcmys.kr>
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/ip.h>
#include <linux/skbuff.h>
#include "atsc3_alp.h"

/* ALP constants (ATSC A/330) */
#define ALP_HDR_LEN	2	/* single-header mode */
#define ALP_MAX_PAYLOAD	2047	/* 11-bit length field */
#define ALP_TYPE_IPV4	0

/*
 * Parse complete ALP packets from the reassembly buffer.
 *
 * ALP header (ATSC A/330, Packet_Type != 7):
 *   Byte 0: [Packet_Type:3][Header_Mode:1][Length_hi:4]
 *   Byte 1: [Length_lo:8]
 *   Length = bits [2:0] of byte 0 ++ byte 1 = 11 bits (max 2047)
 */
static void atsc3_alp_parse(struct atsc3_alp *alp)
{
	struct net_device *net = alp->net;
	unsigned int pos = 0;

	while (pos + ALP_HDR_LEN <= alp->pos) {
		u8 pkt_type = (alp->buf[pos] >> 5) & 0x07;
		u8 hdr_mode = (alp->buf[pos] >> 4) & 0x01;
		u16 pkt_len;
		unsigned int total;
		struct sk_buff *skb;

		/* 11-bit payload length */
		pkt_len = ((u16)(alp->buf[pos] & 0x07) << 8) |
			  alp->buf[pos + 1];

		if (pkt_len == 0 || pkt_len > ALP_MAX_PAYLOAD) {
			pos++;
			net->stats.rx_errors++;
			continue;
		}

		if (hdr_mode != 0) {
			/* Extended header -- skip this ALP packet.
			 * Parse SIF/HEF byte to determine additional header
			 * size so we advance correctly. */
			unsigned int ahdr = 1; /* SIF+HEF flags byte */

			if (pos + 3 > alp->pos)
				break;
			if (alp->buf[pos + 2] & 0x80)
				ahdr++; /* SIF: sub_stream_id */
			if (alp->buf[pos + 2] & 0x40) {
				/* HEF: extension_type + extension_length + data */
				unsigned int hef_off = pos + 2 + ahdr;

				if (hef_off + 2 > alp->pos)
					break;
				ahdr += 2 + alp->buf[hef_off + 1];
			}

			total = 2 + ahdr + pkt_len;
			if (pos + total > alp->pos)
				break;
			pos += total;
			continue;
		}

		/* Single-header mode: 2-byte header + payload */
		total = ALP_HDR_LEN + pkt_len;
		if (pos + total > alp->pos)
			break; /* incomplete packet -- wait for more data */

		if (pkt_type == ALP_TYPE_IPV4 && pkt_len >= 20) {
			const u8 *ip = &alp->buf[pos + ALP_HDR_LEN];

			/* Validate IPv4 version nibble */
			if ((ip[0] >> 4) != 4)
				goto skip;

			skb = dev_alloc_skb(pkt_len);
			if (skb) {
				skb_put_data(skb, ip, pkt_len);
				skb->dev = net;
				skb->protocol = htons(ETH_P_IP);
				skb->pkt_type = PACKET_HOST;
				skb_reset_mac_header(skb);
				skb_reset_network_header(skb);

				net->stats.rx_packets++;
				net->stats.rx_bytes += pkt_len;
				netif_rx(skb);
			} else {
				net->stats.rx_dropped++;
			}
		}
skip:
		pos += total;
	}

	/* Compact remaining data to front of buffer */
	if (pos > 0) {
		alp->pos -= pos;
		if (alp->pos)
			memmove(alp->buf, alp->buf + pos, alp->pos);
	}
}

/*
 * Extract ALP payloads from TS-encapsulated ALP (ALP_DIV_TS mode).
 *
 * CXD2878 ALP_DIV_TS uses a 3-byte TS header (no TSC/AFC/CC byte):
 *   Byte 0:   0x47 sync
 *   Byte 1-2: TEI(1) + PUSI(1) + Priority(1) + PID(13)
 *
 * When PUSI=0: bytes 3-187 are 185 bytes of ALP continuation data.
 * When PUSI=1: byte 3 is a pointer_field (P), bytes 4..4+P-1 are the
 *   tail of the previous ALP packet, bytes 4+P..187 start a new ALP packet.
 */
void atsc3_alp_feed(struct atsc3_alp *alp, const u8 *data, int len)
{
	struct net_device *net = alp->net;
	int pos;

	if (!net || !netif_running(net))
		return;

	for (pos = 0; pos + 188 <= len; pos += 188) {
		const u8 *pkt = data + pos;

		if (pkt[0] != 0x47)
			continue;

		/* Skip packets with transport error */
		if (pkt[1] & 0x80)
			continue;

		if (pkt[1] & 0x40) {
			/* PUSI=1: pointer_field at byte 3 */
			u8 ptr = pkt[3];

			if (ptr > 184) {
				net->stats.rx_errors++;
				continue;
			}

			/* Feed tail of previous ALP packet (ptr bytes) */
			if (ptr > 0 && alp->pos > 0) {
				if (alp->pos + ptr <= ALP_BUF_SIZE) {
					memcpy(alp->buf + alp->pos,
					       pkt + 4, ptr);
					alp->pos += ptr;
				}
			}

			/* Parse to deliver any completed ALP packet */
			atsc3_alp_parse(alp);

			/* Discard leftover (incomplete/initial sync garbage) */
			alp->pos = 0;

			/* New ALP data starts at pkt[4 + ptr] */
			if (ptr < 184) {
				unsigned int new_len = 184 - ptr;

				memcpy(alp->buf, pkt + 4 + ptr, new_len);
				alp->pos = new_len;
			}
		} else {
			/* PUSI=0: 185 bytes of continuation data */
			if (alp->pos + 185 > ALP_BUF_SIZE) {
				net->stats.rx_errors++;
				alp->pos = 0;
				continue;
			}

			memcpy(alp->buf + alp->pos, pkt + 3, 185);
			alp->pos += 185;
		}
	}

	atsc3_alp_parse(alp);
}
EXPORT_SYMBOL_GPL(atsc3_alp_feed);

/* -------- Network device -------- */

static int atsc3_net_open(struct net_device *net)
{
	struct atsc3_alp *alp = net->ml_priv;
	struct dtv_frontend_properties *c;
	int ret;

	if (!alp->fe)
		return -ENODEV;
	c = &alp->fe->dtv_property_cache;
	if (c->delivery_system != SYS_ATSC3)
		return -ENOLINK;

	alp->pos = 0;

	if (alp->start_streaming) {
		ret = alp->start_streaming(alp->priv);
		if (ret)
			return ret;
	}

	netif_carrier_on(net);
	netif_start_queue(net);
	return 0;
}

static int atsc3_net_stop(struct net_device *net)
{
	struct atsc3_alp *alp = net->ml_priv;

	netif_stop_queue(net);
	netif_carrier_off(net);

	if (alp->stop_streaming)
		alp->stop_streaming(alp->priv);

	alp->pos = 0;
	return 0;
}

static netdev_tx_t atsc3_net_xmit(struct sk_buff *skb, struct net_device *net)
{
	/* Receive-only device */
	dev_kfree_skb_any(skb);
	net->stats.tx_dropped++;
	return NETDEV_TX_OK;
}

static const struct net_device_ops atsc3_netdev_ops = {
	.ndo_open	= atsc3_net_open,
	.ndo_stop	= atsc3_net_stop,
	.ndo_start_xmit	= atsc3_net_xmit,
};

static void atsc3_net_setup(struct net_device *net)
{
	/* Raw IP point-to-point device -- no Ethernet framing */
	net->netdev_ops = &atsc3_netdev_ops;
	net->type = ARPHRD_NONE;
	net->hard_header_len = 0;
	net->addr_len = 0;
	net->flags = IFF_POINTOPOINT | IFF_NOARP;
	net->mtu = 1500;
	net->min_mtu = 68;
	net->max_mtu = 65535;
	net->tx_queue_len = 0;
}

int atsc3_alp_register_netdev(struct atsc3_alp *alp)
{
	int i;

	if (!alp->fe)
		return 0;

	for (i = 0; i < MAX_DELSYS && alp->fe->ops.delsys[i]; i++)
		if (alp->fe->ops.delsys[i] == SYS_ATSC3)
			goto found;
	return 0;
found:
	alp->buf = kmalloc(ALP_BUF_SIZE, GFP_KERNEL);
	if (!alp->buf)
		return -ENOMEM;

	alp->pos = 0;
	alp->net = alloc_netdev(0, "atsc%d", NET_NAME_ENUM,
				atsc3_net_setup);
	if (!alp->net) {
		kfree(alp->buf);
		alp->buf = NULL;
		return -ENOMEM;
	}

	if (alp->dev)
		SET_NETDEV_DEV(alp->net, alp->dev);
	alp->net->ml_priv = alp;
	if (register_netdev(alp->net)) {
		free_netdev(alp->net);
		alp->net = NULL;
		kfree(alp->buf);
		alp->buf = NULL;
		return -ENODEV;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(atsc3_alp_register_netdev);

void atsc3_alp_unregister_netdev(struct atsc3_alp *alp)
{
	if (alp->net) {
		unregister_netdev(alp->net);
		free_netdev(alp->net);
		alp->net = NULL;
	}
	kfree(alp->buf);
	alp->buf = NULL;
}
EXPORT_SYMBOL_GPL(atsc3_alp_unregister_netdev);

MODULE_AUTHOR("koreapyj");
MODULE_DESCRIPTION("ATSC 3.0 ALP parser and network interface");
MODULE_LICENSE("GPL");
