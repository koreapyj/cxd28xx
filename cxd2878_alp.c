// SPDX-License-Identifier: GPL-2.0-only
/*
 * CXD2878 ALP (ATSC Link-layer Protocol) virtual network adapter
 *
 * Receives ALP-div-TS from the DVB demux, reassembles ALP packets, and:
 *   - Type 0 (IPv4): injects into kernel network stack via netif_rx()
 *   - Type 7 (MPEG-2 TS): routes to dvb_dmx_swfilter() for DVB processing
 *
 * Reference: ATSC A/330 Link-Layer Protocol
 */

#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/skbuff.h>
#include <media/dvb_demux.h>

#include "cxd2878.h"
#include "cxd2878_priv.h"
#include "cxd2878_alp.h"

/* ── Private data stored in net_device ─────────────────────────────── */
struct cxd2878_alp_priv {
	struct cxd2878_dev *dev;
};

/* ── ALP reassembly helpers ────────────────────────────────────────── */

static void alp_ctx_reset(struct cxd2878_dev *dev)
{
	dev->alp_buf_len = 0;
	dev->alp_expected_len = 0;
	dev->alp_active = false;
}

static int alp_ctx_append(struct cxd2878_dev *dev, const u8 *data, u32 len)
{
	if (dev->alp_buf_len + len > CXD2878_ALP_BUF_SIZE) {
		if (dev->alpdev)
			dev->alpdev->stats.rx_over_errors++;
		alp_ctx_reset(dev);
		return -1;
	}
	memcpy(dev->alp_buf + dev->alp_buf_len, data, len);
	dev->alp_buf_len += len;
	return 0;
}

/* ── ALP packet delivery helpers ───────────────────────────────────── */

static void alp_deliver_ipv4(struct cxd2878_dev *dev,
			     const u8 *data, u32 len)
{
	struct sk_buff *skb;

	if (!dev->alpdev || !netif_running(dev->alpdev))
		return;
	if (len < 20) {
		dev->alpdev->stats.rx_length_errors++;
		return;
	}

	skb = dev_alloc_skb(len + NET_IP_ALIGN);
	if (!skb) {
		dev->alpdev->stats.rx_dropped++;
		return;
	}
	skb_reserve(skb, NET_IP_ALIGN);
	skb_put_data(skb, data, len);
	skb->dev = dev->alpdev;
	skb->protocol = htons(ETH_P_IP);
	skb_reset_network_header(skb);
	if (netif_rx(skb) == NET_RX_SUCCESS) {
		dev->alpdev->stats.rx_packets++;
		dev->alpdev->stats.rx_bytes += len;
	} else {
		dev->alpdev->stats.rx_dropped++;
	}
}

/* Dispatch a single complete ALP payload by type */
static void alp_dispatch(struct cxd2878_dev *dev, u8 packet_type,
			 const u8 *payload, u32 payload_len)
{
	switch (packet_type) {
	case ALP_TYPE_IPV4:
		alp_deliver_ipv4(dev, payload, payload_len);
		break;
	case ALP_TYPE_MPEG2_TS:
		if (dev->alp_dvb_demux && payload_len >= 188)
			dvb_dmx_swfilter(dev->alp_dvb_demux, payload,
					 payload_len);
		break;
	default:
		if (dev->alpdev)
			dev->alpdev->stats.rx_dropped++;
		break;
	}
}

/* ── ALP segmentation reassembly (A/330 §5.1.2.2) ─────────────────── */

static void alp_seg_reset(struct cxd2878_dev *dev)
{
	dev->alp_seg_len = 0;
	dev->alp_seg_next_sn = 0;
	dev->alp_seg_active = false;
}

static void alp_handle_segmented(struct cxd2878_dev *dev, u8 packet_type,
				 const u8 *buf, u32 len)
{
	u16 alp_length;
	u8 addhdr, seg_sn, lsi, sif, hef;
	u32 hdr_size;
	const u8 *payload;
	u16 payload_len;

	if (len < 3) {
		dev->alpdev->stats.rx_length_errors++;
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
	if (sif) hdr_size += 1;  /* sub_stream_id */
	if (hef) hdr_size += 1;  /* header_extension: skip 1 byte (min) */
	/* Note: HEF can be variable length per §5.1.3.2; for now skip 1 byte */

	if (alp_length + 2 > len) {
		dev->alpdev->stats.rx_frame_errors++;
		alp_seg_reset(dev);
		return;
	}

	payload = buf + hdr_size;
	payload_len = alp_length - (hdr_size - 2);

	if (seg_sn == 0) {
		/* First segment — start new reassembly */
		alp_seg_reset(dev);
		dev->alp_seg_active = true;
		dev->alp_seg_type = packet_type;
		dev->alp_seg_next_sn = 1;
	} else if (!dev->alp_seg_active ||
		   seg_sn != dev->alp_seg_next_sn ||
		   packet_type != dev->alp_seg_type) {
		/* Out-of-order or type mismatch — discard */
		dev->alpdev->stats.rx_frame_errors++;
		alp_seg_reset(dev);
		return;
	} else {
		dev->alp_seg_next_sn = seg_sn + 1;
	}

	/* Append segment payload */
	if (!dev->alp_seg_buf ||
	    dev->alp_seg_len + payload_len > CXD2878_ALP_SEG_BUF_SIZE) {
		dev->alpdev->stats.rx_over_errors++;
		alp_seg_reset(dev);
		return;
	}
	memcpy(dev->alp_seg_buf + dev->alp_seg_len, payload, payload_len);
	dev->alp_seg_len += payload_len;

	if (lsi) {
		/* Last segment — deliver reassembled packet */
		alp_dispatch(dev, dev->alp_seg_type,
			     dev->alp_seg_buf, dev->alp_seg_len);
		alp_seg_reset(dev);
	}
}

/* ── ALP concatenation (A/330 §5.1.2.3) ───────────────────────────── */

static void alp_handle_concatenated(struct cxd2878_dev *dev, u8 packet_type,
				    const u8 *buf, u32 len)
{
	u16 alp_length, total_payload;
	u8 length_msb, count, sif;
	u32 hdr_bits_offset, hdr_size;
	u16 comp_len[9];  /* max 9 component_length fields (count=7 → 8 packets) */
	u32 i, num_packets, offset;
	u32 sum_comp;

	if (len < 3) {
		dev->alpdev->stats.rx_length_errors++;
		return;
	}

	alp_length = ((buf[0] & 0x07) << 8) | buf[1];

	/* Additional header byte 2: length_MSB(4) | count(3) | SIF(1) */
	length_msb = (buf[2] >> 4) & 0x0F;
	count      = (buf[2] >> 1) & 0x07;
	sif        =  buf[2]       & 1;

	total_payload = (length_msb << 11) | alp_length;
	num_packets = count + 2;  /* count = num_packets - 2 */

	/*
	 * Component lengths: (count+1) × 12-bit fields starting at byte 3.
	 * If (count+1) is odd, 4 stuffing bits follow.
	 * Then optionally 1 byte SID if sif=1.
	 *
	 * Parse component_length fields from the bit stream.
	 */
	hdr_bits_offset = 24;  /* 3 bytes of base+additional so far */

	for (i = 0; i < count + 1; i++) {
		u32 byte_off = hdr_bits_offset / 8;
		u32 bit_off  = hdr_bits_offset % 8;

		if (byte_off + 2 > len) {
			dev->alpdev->stats.rx_frame_errors++;
			return;
		}

		/* Extract 12 bits starting at bit_off within buf[byte_off] */
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

	/* Stuffing bits if (count+1) is odd (i.e. count is even) */
	if ((count & 1) == 0)
		hdr_bits_offset += 4;

	hdr_size = (hdr_bits_offset + 7) / 8;
	if (sif) hdr_size += 1;

	/*
	 * total_payload includes the additional header (everything after
	 * the 2-byte base header). Subtract additional header size to get
	 * the actual data length.
	 */
	u32 addl_hdr_size = hdr_size - 2;
	u32 data_len;

	if (total_payload < addl_hdr_size) {
		dev->alpdev->stats.rx_frame_errors++;
		return;
	}
	data_len = total_payload - addl_hdr_size;

	if (hdr_size + data_len > len) {
		dev->alpdev->stats.rx_frame_errors++;
		return;
	}

	/* Deliver each concatenated component */
	offset = hdr_size;
	sum_comp = 0;

	for (i = 0; i < num_packets; i++) {
		u16 pkt_len;

		if (i < count + 1) {
			pkt_len = comp_len[i];
		} else {
			/* Last packet: remaining bytes */
			if (data_len < sum_comp) {
				dev->alpdev->stats.rx_frame_errors++;
				return;
			}
			pkt_len = data_len - sum_comp;
		}

		if (offset + pkt_len > len) {
			dev->alpdev->stats.rx_frame_errors++;
			return;
		}

		alp_dispatch(dev, packet_type, buf + offset, pkt_len);
		offset += pkt_len;
		sum_comp += pkt_len;
	}
}

/* ── ALP packet processing ─────────────────────────────────────────── */

static void cxd2878_alp_process(struct cxd2878_dev *dev, const u8 *buf, u32 len)
{
	u8 packet_type;
	u8 pc, header_mode;
	u16 alp_length;
	u32 hdr_size;
	const u8 *payload;
	u16 payload_len;

	if (!dev->alpdev)
		return;

	if (len < 2) {
		dev->alpdev->stats.rx_length_errors++;
		return;
	}

	packet_type = (buf[0] >> 5) & 0x07;
	pc = (buf[0] >> 4) & 1;

	if (pc) {
		/* payload_configuration=1: segmented or concatenated */
		u8 sc = (buf[0] >> 3) & 1;

		if (sc == 0)
			alp_handle_segmented(dev, packet_type, buf, len);
		else
			alp_handle_concatenated(dev, packet_type, buf, len);
		return;
	}

	/* payload_configuration=0: single packet */
	header_mode = (buf[0] >> 3) & 1;
	alp_length = ((buf[0] & 0x07) << 8) | buf[1];

	if (header_mode) {
		switch (packet_type) {
		case ALP_TYPE_IPV4:
			hdr_size = 3;	/* 2-byte base + 1-byte additional */
			break;
		default:
			/* Signalling/control with extended headers -- skip */
			dev->alpdev->stats.rx_missed_errors++;
			return;
		}
	} else {
		hdr_size = 2;
	}

	if (alp_length + 2 > len) {
		dev->alpdev->stats.rx_frame_errors++;
		return;
	}

	payload = buf + hdr_size;
	payload_len = alp_length - (hdr_size - 2);

	alp_dispatch(dev, packet_type, payload, payload_len);
}

/* ── ALP reassembly: check for complete packets in buffer ──────────── */

static void alp_check_complete(struct cxd2878_dev *dev)
{
	while (dev->alp_active) {
		if (dev->alp_expected_len == 0 && dev->alp_buf_len >= 2) {
			u8 pc = (dev->alp_buf[0] >> 4) & 1;
			u16 length = ((dev->alp_buf[0] & 0x07) << 8) |
				     dev->alp_buf[1];

			if (pc && (dev->alp_buf[0] & 0x08)) {
				/* PC=1, S/C=1 (concatenation): need byte 2
				 * for length_MSB */
				if (dev->alp_buf_len < 3)
					break;
				u8 length_msb = (dev->alp_buf[2] >> 4) & 0x0F;
				dev->alp_expected_len =
					((length_msb << 11) | length) + 2;
			} else {
				/* PC=0 or PC=1,S/C=0: length fits in 11 bits */
				dev->alp_expected_len = length + 2;
			}
		}

		if (dev->alp_expected_len == 0 ||
		    dev->alp_buf_len < dev->alp_expected_len)
			break;

		cxd2878_alp_process(dev, dev->alp_buf, dev->alp_expected_len);

		u32 leftover = dev->alp_buf_len - dev->alp_expected_len;
		if (leftover > 0) {
			memmove(dev->alp_buf,
				dev->alp_buf + dev->alp_expected_len, leftover);
			dev->alp_buf_len = leftover;
			dev->alp_expected_len = 0;
		} else {
			alp_ctx_reset(dev);
		}
	}
}

/* ── TS packet processing (ALP-div-TS) ─────────────────────────────── */

static void cxd2878_alp_process_ts(struct cxd2878_dev *dev, const u8 *pkt)
{
	u8 pusi;
	u8 pointer;
	const u8 *payload;
	u16 pid;
	u32 remaining;

	if (pkt[0] != 0x47)
		return;

	/* Skip null packets (padding) */
	pid = ((pkt[1] & 0x1F) << 8) | pkt[2];
	if (pid == 0x1FFF)
		return;

	pusi = (pkt[1] >> 6) & 1;
	payload = pkt + CXD2878_ALP_TS_HDR_SIZE;

	if (pusi) {
		pointer = payload[0];
		if (pointer > CXD2878_ALP_TS_PAYLOAD - 1) {
			alp_ctx_reset(dev);
			return;
		}

		/* Complete previous ALP packet with tail data */
		if (dev->alp_active && pointer > 0) {
			alp_ctx_append(dev, payload + 1, pointer);
			if (dev->alp_buf_len > 0)
				cxd2878_alp_process(dev, dev->alp_buf,
						    dev->alp_buf_len);
		}

		/* Always reset before starting new ALP data */
		alp_ctx_reset(dev);

		/* Start new ALP from after pointer */
		remaining = CXD2878_ALP_TS_PAYLOAD - 1 - pointer;
		if (remaining > 0) {
			dev->alp_active = true;
			alp_ctx_append(dev, payload + 1 + pointer, remaining);
			alp_check_complete(dev);
		}
	} else {
		if (dev->alp_active) {
			alp_ctx_append(dev, payload, CXD2878_ALP_TS_PAYLOAD);
			alp_check_complete(dev);
		}
	}
}

/* ── DVB demux TS feed callback ────────────────────────────────────── */

static int cxd2878_alp_ts_cb(const u8 *buf1, size_t len1,
			     const u8 *buf2, size_t len2,
			     struct dmx_ts_feed *feed, u32 *buffer_flags)
{
	struct cxd2878_dev *dev = feed->priv;
	const u8 *buf;
	size_t len, pos;
	int i;

	for (i = 0; i < 2; i++) {
		buf = (i == 0) ? buf1 : buf2;
		len = (i == 0) ? len1 : len2;
		if (!buf || !len)
			continue;
		for (pos = 0; pos + 188 <= len; pos += 188)
			cxd2878_alp_process_ts(dev, buf + pos);
	}
	return 0;
}

/* ── Net device operations ─────────────────────────────────────────── */

static int cxd2878_alp_open(struct net_device *netdev)
{
	struct cxd2878_alp_priv *priv = netdev_priv(netdev);
	struct cxd2878_dev *dev = priv->dev;
	struct dmx_ts_feed *feed;
	int ret;

	if (!dev->alp_demux)
		return -ENODEV;

	ret = dev->alp_demux->allocate_ts_feed(dev->alp_demux, &feed,
					       cxd2878_alp_ts_cb);
	if (ret)
		return ret;

	feed->priv = dev;

	ret = feed->set(feed, CXD2878_ALP_DEFAULT_PID, TS_PACKET,
			DMX_PES_OTHER, ktime_set(0, 0));
	if (ret) {
		dev->alp_demux->release_ts_feed(dev->alp_demux, feed);
		return ret;
	}

	ret = feed->start_filtering(feed);
	if (ret) {
		dev->alp_demux->release_ts_feed(dev->alp_demux, feed);
		return ret;
	}

	dev->alp_feed = feed;
	alp_ctx_reset(dev);
	alp_seg_reset(dev);

	return 0;
}

static int cxd2878_alp_stop(struct net_device *netdev)
{
	struct cxd2878_alp_priv *priv = netdev_priv(netdev);
	struct cxd2878_dev *dev = priv->dev;

	if (dev->alp_feed) {
		dev->alp_feed->stop_filtering(dev->alp_feed);
		dev->alp_demux->release_ts_feed(dev->alp_demux,
						dev->alp_feed);
		dev->alp_feed = NULL;
	}

	alp_ctx_reset(dev);
	alp_seg_reset(dev);
	return 0;
}

static netdev_tx_t cxd2878_alp_xmit(struct sk_buff *skb,
				     struct net_device *netdev)
{
	netdev->stats.tx_dropped++;
	kfree_skb(skb);
	return NETDEV_TX_OK;
}

static const struct net_device_ops cxd2878_alp_netdev_ops = {
	.ndo_open	= cxd2878_alp_open,
	.ndo_stop	= cxd2878_alp_stop,
	.ndo_start_xmit	= cxd2878_alp_xmit,
};

static void cxd2878_alp_setup(struct net_device *netdev)
{
	netdev->type		= ARPHRD_NONE;
	netdev->hard_header_len	= 0;
	netdev->addr_len	= 0;
	netdev->mtu		= 1500;
	netdev->min_mtu		= 68;
	netdev->max_mtu		= 9000;
	netdev->flags		= IFF_POINTOPOINT | IFF_NOARP | IFF_MULTICAST;
	netdev->netdev_ops	= &cxd2878_alp_netdev_ops;
	netdev->needs_free_netdev = true;
}

/* ── Attach / detach ───────────────────────────────────────────────── */

int cxd2878_alp_attach(struct cxd2878_dev *dev, struct dmx_demux *demux,
		       struct dvb_demux *dvb_demux, struct device *parent)
{
	struct net_device *netdev;
	struct cxd2878_alp_priv *priv;
	int ret;

	netdev = alloc_netdev(sizeof(*priv), "alp%d",
			      NET_NAME_ENUM, cxd2878_alp_setup);
	if (!netdev)
		return -ENOMEM;

	SET_NETDEV_DEV(netdev, parent);
	netif_carrier_off(netdev);

	priv = netdev_priv(netdev);
	priv->dev = dev;

	dev->alpdev = netdev;
	dev->alp_demux = demux;
	dev->alp_dvb_demux = dvb_demux;
	dev->alp_feed = NULL;
	dev->alp_carrier = false;
	alp_ctx_reset(dev);

	dev->alp_seg_buf = kmalloc(CXD2878_ALP_SEG_BUF_SIZE, GFP_KERNEL);
	if (!dev->alp_seg_buf) {
		free_netdev(netdev);
		dev->alpdev = NULL;
		dev->alp_demux = NULL;
		dev->alp_dvb_demux = NULL;
		return -ENOMEM;
	}
	alp_seg_reset(dev);

	ret = register_netdev(netdev);
	if (ret) {
		kfree(dev->alp_seg_buf);
		dev->alp_seg_buf = NULL;
		free_netdev(netdev);
		dev->alpdev = NULL;
		dev->alp_demux = NULL;
		dev->alp_dvb_demux = NULL;
		return ret;
	}

	dev_info(parent, "cxd2878: ALP net device %s registered\n",
		 netdev->name);
	return 0;
}
EXPORT_SYMBOL_GPL(cxd2878_alp_attach);

void cxd2878_alp_detach(struct cxd2878_dev *dev)
{
	if (!dev->alpdev)
		return;

	if (dev->alp_feed) {
		dev->alp_feed->stop_filtering(dev->alp_feed);
		dev->alp_demux->release_ts_feed(dev->alp_demux,
						dev->alp_feed);
		dev->alp_feed = NULL;
	}

	unregister_netdev(dev->alpdev);
	/* netdev freed by needs_free_netdev */
	kfree(dev->alp_seg_buf);
	dev->alp_seg_buf = NULL;
	dev->alpdev = NULL;
	dev->alp_demux = NULL;
	dev->alp_dvb_demux = NULL;
}
EXPORT_SYMBOL_GPL(cxd2878_alp_detach);
