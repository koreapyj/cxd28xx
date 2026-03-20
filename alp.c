// SPDX-License-Identifier: GPL-2.0-only
/*
 * ALP (ATSC Link-layer Protocol) virtual network adapter
 *
 * Generic A/330 ALP packet parser and IPv4 delivery layer.
 * Parses ALP headers, handles segmentation/concatenation, and injects
 * IPv4 payloads into the kernel network stack via a raw-ip net_device.
 *
 * Reference: ATSC A/330 Link-Layer Protocol
 */

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/slab.h>

#include "alp.h"

/* ── Internal structures ───────────────────────────────────────────── */

struct alp_dev {
	struct net_device   *netdev;

	/* Transport-layer callbacks */
	const struct alp_ops *ops;
	void                *ops_priv;

	/* Segmentation reassembly (A/330 §5.1.2.2) */
	u8                 *seg_buf;
	u32                 seg_len;
	u8                  seg_type;
	u8                  seg_next_sn;
	bool                seg_active;

	/* Protocol-level stats */
	struct alp_stats    stats;
};

struct alp_priv {
	struct alp_dev *alp;
};

/* ── Ethtool statistics ────────────────────────────────────────────── */

static const char alp_stat_names[][ETH_GSTRING_LEN] = {
	"alp_ip_packets",
	"alp_ip_bytes",
	"alp_seg_completed",
	"alp_concat_delivered",
	"alp_type_compressed_ip",
	"alp_type_lls",
	"alp_type_ext",
	"alp_type_ts",
	"alp_type_unknown",
	"alp_ext_hdr_compressed_ip",
	"alp_ext_hdr_lls",
	"alp_ext_hdr_ext",
	"alp_ext_hdr_ts",
	"alp_ext_hdr_unknown",
	"alp_seg_errors",
	"alp_frame_err_single",
	"alp_frame_err_seg",
	"alp_frame_err_concat",
	"alp_short_packets",
	"alp_skb_alloc_fail",
	"alp_netif_rx_fail",
};

#define ALP_STAT_COUNT ARRAY_SIZE(alp_stat_names)

/* ── ALP packet delivery ──────────────────────────────────────────── */

static void alp_deliver_ipv4(struct alp_dev *alp, const u8 *data, u32 len)
{
	struct sk_buff *skb;

	if (!alp->netdev || !netif_running(alp->netdev))
		return;
	if (len < 20) {
		alp->stats.short_packets++;
		return;
	}

	skb = dev_alloc_skb(len + NET_IP_ALIGN);
	if (!skb) {
		alp->stats.skb_alloc_fail++;
		return;
	}
	skb_reserve(skb, NET_IP_ALIGN);
	skb_put_data(skb, data, len);
	skb->dev = alp->netdev;
	skb->protocol = htons(ETH_P_IP);
	skb_reset_network_header(skb);
	if (netif_rx(skb) == NET_RX_SUCCESS) {
		alp->netdev->stats.rx_packets++;
		alp->netdev->stats.rx_bytes += len;
		alp->stats.ip_packets++;
		alp->stats.ip_bytes += len;
	} else {
		alp->stats.netif_rx_fail++;
		alp->netdev->stats.rx_dropped++;
	}
}

/* Dispatch a single complete ALP payload by type */
static void alp_dispatch(struct alp_dev *alp, u8 packet_type,
			 const u8 *payload, u32 payload_len)
{
	switch (packet_type) {
	case ALP_TYPE_IPV4:
		alp_deliver_ipv4(alp, payload, payload_len);
		break;
	case ALP_TYPE_COMPRESSED_IP:
		alp->stats.type_compressed_ip++;
		break;
	case ALP_TYPE_LLS:
		alp->stats.type_lls++;
		break;
	case ALP_TYPE_TYPE_EXT:
		alp->stats.type_ext++;
		break;
	case ALP_TYPE_MPEG2_TS:
		alp->stats.type_ts++;
		break;
	default:
		alp->stats.type_unknown++;
		break;
	}
}

/* ── ALP segmentation reassembly (A/330 §5.1.2.2) ─────────────────── */

static void alp_seg_reset(struct alp_dev *alp)
{
	alp->seg_len = 0;
	alp->seg_next_sn = 0;
	alp->seg_active = false;
}

static void alp_handle_segmented(struct alp_dev *alp, u8 packet_type,
				 const u8 *buf, u32 len)
{
	u16 alp_length;
	u8 addhdr, seg_sn, lsi, sif, hef;
	u32 hdr_size;
	const u8 *payload;
	u16 payload_len;

	if (len < 3) {
		alp->stats.short_packets++;
		return;
	}

	alp_length = ((buf[0] & 0x07) << 8) | buf[1];
	addhdr = buf[2];
	seg_sn = (addhdr >> 3) & 0x1F;
	lsi    = (addhdr >> 2) & 1;
	sif    = (addhdr >> 1) & 1;
	hef    =  addhdr       & 1;

	/* Header: 2 (base) + 1 (seg additional) + SIF + HEF(skip) */
	hdr_size = 3;
	if (sif) hdr_size += 1;
	if (hef) hdr_size += 1;

	if (alp_length + 2 > len) {
		alp->stats.frame_err_seg++;
		alp_seg_reset(alp);
		return;
	}

	payload = buf + hdr_size;
	payload_len = alp_length - (hdr_size - 2);

	if (seg_sn == 0) {
		alp_seg_reset(alp);
		alp->seg_active = true;
		alp->seg_type = packet_type;
		alp->seg_next_sn = 1;
	} else if (!alp->seg_active ||
		   seg_sn != alp->seg_next_sn ||
		   packet_type != alp->seg_type) {
		alp->stats.seg_errors++;
		alp_seg_reset(alp);
		return;
	} else {
		alp->seg_next_sn = seg_sn + 1;
	}

	if (!alp->seg_buf ||
	    alp->seg_len + payload_len > ALP_SEG_BUF_SIZE) {
		alp->stats.seg_errors++;
		alp_seg_reset(alp);
		return;
	}
	memcpy(alp->seg_buf + alp->seg_len, payload, payload_len);
	alp->seg_len += payload_len;

	if (lsi) {
		alp->stats.seg_completed++;
		alp_dispatch(alp, alp->seg_type,
			     alp->seg_buf, alp->seg_len);
		alp_seg_reset(alp);
	}
}

/* ── ALP concatenation (A/330 §5.1.2.3) ───────────────────────────── */

static void alp_handle_concatenated(struct alp_dev *alp, u8 packet_type,
				    const u8 *buf, u32 len)
{
	u16 alp_length, total_payload;
	u8 length_msb, count, sif;
	u32 hdr_bits_offset, hdr_size;
	u16 comp_len[9];
	u32 i, num_packets, offset;
	u32 sum_comp;

	if (len < 3) {
		alp->stats.short_packets++;
		return;
	}

	alp_length = ((buf[0] & 0x07) << 8) | buf[1];

	length_msb = (buf[2] >> 4) & 0x0F;
	count      = (buf[2] >> 1) & 0x07;
	sif        =  buf[2]       & 1;

	total_payload = (length_msb << 11) | alp_length;
	num_packets = count + 2;

	hdr_bits_offset = 24;

	for (i = 0; i < count + 1; i++) {
		u32 byte_off = hdr_bits_offset / 8;
		u32 bit_off  = hdr_bits_offset % 8;

		if (byte_off + 2 > len) {
			alp->stats.frame_err_concat++;
			return;
		}

		if (bit_off <= 4) {
			comp_len[i] = ((buf[byte_off] << 8) | buf[byte_off + 1])
				>> (4 - bit_off);
		} else {
			comp_len[i] = ((buf[byte_off] << 16) |
				       (buf[byte_off + 1] << 8) |
				       buf[byte_off + 2])
				>> (12 - (bit_off - 4));
		}
		comp_len[i] &= 0x0FFF;
		hdr_bits_offset += 12;
	}

	if ((count & 1) == 0)
		hdr_bits_offset += 4;

	hdr_size = (hdr_bits_offset + 7) / 8;
	if (sif) hdr_size += 1;

	u32 addl_hdr_size = hdr_size - 2;
	u32 data_len;

	if (total_payload < addl_hdr_size) {
		alp->stats.frame_err_concat++;
		return;
	}
	data_len = total_payload - addl_hdr_size;

	if (hdr_size + data_len > len) {
		alp->stats.frame_err_concat++;
		return;
	}

	offset = hdr_size;
	sum_comp = 0;

	for (i = 0; i < num_packets; i++) {
		u16 pkt_len;

		if (i < count + 1) {
			pkt_len = comp_len[i];
		} else {
			if (data_len < sum_comp) {
				alp->stats.frame_err_concat++;
				return;
			}
			pkt_len = data_len - sum_comp;
		}

		if (offset + pkt_len > len) {
			alp->stats.frame_err_concat++;
			return;
		}

		alp->stats.concat_delivered++;
		alp_dispatch(alp, packet_type, buf + offset, pkt_len);
		offset += pkt_len;
		sum_comp += pkt_len;
	}
}

/* ── ALP packet processing ─────────────────────────────────────────── */

void alp_process(struct alp_dev *alp, const u8 *buf, u32 len)
{
	u8 packet_type;
	u8 pc, header_mode;
	u16 alp_length;
	u32 hdr_size;
	const u8 *payload;
	u16 payload_len;

	if (!alp || !alp->netdev)
		return;

	if (len < 2) {
		alp->stats.short_packets++;
		return;
	}

	packet_type = (buf[0] >> 5) & 0x07;
	pc = (buf[0] >> 4) & 1;

	if (pc) {
		u8 sc = (buf[0] >> 3) & 1;

		if (sc == 0)
			alp_handle_segmented(alp, packet_type, buf, len);
		else
			alp_handle_concatenated(alp, packet_type, buf, len);
		return;
	}

	/* payload_configuration=0: single packet */
	header_mode = (buf[0] >> 3) & 1;
	alp_length = ((buf[0] & 0x07) << 8) | buf[1];

	if (header_mode) {
		switch (packet_type) {
		case ALP_TYPE_IPV4:
			hdr_size = 3;
			break;
		case ALP_TYPE_COMPRESSED_IP:
			alp->stats.ext_hdr_compressed_ip++; return;
		case ALP_TYPE_LLS:
			alp->stats.ext_hdr_lls++; return;
		case ALP_TYPE_TYPE_EXT:
			alp->stats.ext_hdr_ext++; return;
		case ALP_TYPE_MPEG2_TS:
			alp->stats.ext_hdr_ts++; return;
		default:
			alp->stats.ext_hdr_unknown++; return;
		}
	} else {
		hdr_size = 2;
	}

	if (alp_length + 2 > len) {
		alp->stats.frame_err_single++;
		return;
	}

	payload = buf + hdr_size;
	payload_len = alp_length - (hdr_size - 2);

	alp_dispatch(alp, packet_type, payload, payload_len);
}
EXPORT_SYMBOL_GPL(alp_process);

/* ── Net device operations ─────────────────────────────────────────── */

static int alp_open(struct net_device *netdev)
{
	struct alp_priv *priv = netdev_priv(netdev);
	struct alp_dev *alp = priv->alp;
	int ret = 0;

	if (alp->ops && alp->ops->open)
		ret = alp->ops->open(alp->ops_priv);
	if (ret)
		return ret;

	alp_seg_reset(alp);
	memset(&alp->stats, 0, sizeof(alp->stats));
	return 0;
}

static int alp_stop(struct net_device *netdev)
{
	struct alp_priv *priv = netdev_priv(netdev);
	struct alp_dev *alp = priv->alp;

	if (alp->ops && alp->ops->stop)
		alp->ops->stop(alp->ops_priv);

	alp_seg_reset(alp);
	return 0;
}

static netdev_tx_t alp_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	netdev->stats.tx_dropped++;
	kfree_skb(skb);
	return NETDEV_TX_OK;
}

/* ── Ethtool operations ───────────────────────────────────────────── */

static void alp_get_drvinfo(struct net_device *netdev,
			    struct ethtool_drvinfo *info)
{
	strscpy(info->driver, "alp", sizeof(info->driver));
	strscpy(info->version, "1.0", sizeof(info->version));
}

static int alp_get_sset_count(struct net_device *netdev, int sset)
{
	if (sset == ETH_SS_STATS)
		return ALP_STAT_COUNT;
	return -EOPNOTSUPP;
}

static void alp_get_strings(struct net_device *netdev, u32 sset, u8 *data)
{
	if (sset == ETH_SS_STATS)
		memcpy(data, alp_stat_names, sizeof(alp_stat_names));
}

static void alp_get_ethtool_stats(struct net_device *netdev,
				  struct ethtool_stats *stats, u64 *data)
{
	struct alp_priv *priv = netdev_priv(netdev);
	struct alp_stats *s = &priv->alp->stats;

	*data++ = s->ip_packets;
	*data++ = s->ip_bytes;
	*data++ = s->seg_completed;
	*data++ = s->concat_delivered;
	*data++ = s->type_compressed_ip;
	*data++ = s->type_lls;
	*data++ = s->type_ext;
	*data++ = s->type_ts;
	*data++ = s->type_unknown;
	*data++ = s->ext_hdr_compressed_ip;
	*data++ = s->ext_hdr_lls;
	*data++ = s->ext_hdr_ext;
	*data++ = s->ext_hdr_ts;
	*data++ = s->ext_hdr_unknown;
	*data++ = s->seg_errors;
	*data++ = s->frame_err_single;
	*data++ = s->frame_err_seg;
	*data++ = s->frame_err_concat;
	*data++ = s->short_packets;
	*data++ = s->skb_alloc_fail;
	*data++ = s->netif_rx_fail;
}

static const struct ethtool_ops alp_ethtool_ops = {
	.get_drvinfo		= alp_get_drvinfo,
	.get_sset_count		= alp_get_sset_count,
	.get_strings		= alp_get_strings,
	.get_ethtool_stats	= alp_get_ethtool_stats,
};

/* ── Net device setup ──────────────────────────────────────────────── */

static const struct net_device_ops alp_netdev_ops = {
	.ndo_open	= alp_open,
	.ndo_stop	= alp_stop,
	.ndo_start_xmit	= alp_xmit,
};

static void alp_setup(struct net_device *netdev)
{
	netdev->type		= ARPHRD_NONE;
	netdev->hard_header_len	= 0;
	netdev->addr_len	= 0;
	netdev->mtu		= 1500;
	netdev->min_mtu		= 68;
	netdev->max_mtu		= 9000;
	netdev->flags		= IFF_POINTOPOINT | IFF_NOARP | IFF_MULTICAST;
	netdev->netdev_ops	= &alp_netdev_ops;
	netdev->ethtool_ops	= &alp_ethtool_ops;
	netdev->needs_free_netdev = true;
}

/* ── Attach / detach ───────────────────────────────────────────────── */

struct alp_dev *alp_attach(struct device *parent, const struct alp_ops *ops,
			   void *ops_priv)
{
	struct net_device *netdev;
	struct alp_priv *priv;
	struct alp_dev *alp;
	int ret;

	alp = kzalloc(sizeof(*alp), GFP_KERNEL);
	if (!alp)
		return ERR_PTR(-ENOMEM);

	alp->ops = ops;
	alp->ops_priv = ops_priv;

	alp->seg_buf = kmalloc(ALP_SEG_BUF_SIZE, GFP_KERNEL);
	if (!alp->seg_buf) {
		kfree(alp);
		return ERR_PTR(-ENOMEM);
	}
	alp_seg_reset(alp);

	netdev = alloc_netdev(sizeof(*priv), "alp%d",
			      NET_NAME_ENUM, alp_setup);
	if (!netdev) {
		kfree(alp->seg_buf);
		kfree(alp);
		return ERR_PTR(-ENOMEM);
	}

	SET_NETDEV_DEV(netdev, parent);
	netif_carrier_off(netdev);

	priv = netdev_priv(netdev);
	priv->alp = alp;
	alp->netdev = netdev;

	ret = register_netdev(netdev);
	if (ret) {
		free_netdev(netdev);
		kfree(alp->seg_buf);
		kfree(alp);
		return ERR_PTR(ret);
	}

	dev_info(parent, "alp: net device %s registered\n", netdev->name);
	return alp;
}
EXPORT_SYMBOL_GPL(alp_attach);

void alp_detach(struct alp_dev *alp)
{
	if (!alp)
		return;

	if (alp->netdev)
		unregister_netdev(alp->netdev);
	/* netdev freed by needs_free_netdev */

	kfree(alp->seg_buf);
	kfree(alp);
}
EXPORT_SYMBOL_GPL(alp_detach);

struct net_device *alp_get_netdev(struct alp_dev *alp)
{
	return alp ? alp->netdev : NULL;
}
EXPORT_SYMBOL_GPL(alp_get_netdev);


/* ── Module ────────────────────────────────────────────────────────── */

static int __init alp_module_init(void)
{
	return 0;
}

static void __exit alp_module_exit(void)
{
}

module_init(alp_module_init);
module_exit(alp_module_exit);

MODULE_AUTHOR("Yoonji Park <koreapyj@dcmys.kr>");
MODULE_DESCRIPTION("ATSC A/330 ALP protocol");
MODULE_LICENSE("GPL");
