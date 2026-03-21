/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * ALP (ATSC Link-layer Protocol) virtual network adapter
 *
 * Generic A/330 ALP packet parser and IPv4 delivery layer.
 * Receives complete ALP packets from a transport-specific decapsulator
 * (e.g. ALP-div-TS) and delivers IPv4 payloads to the kernel network stack.
 *
 * Reference: ATSC A/330 Link-Layer Protocol
 */
#ifndef _ALP_H_
#define _ALP_H_

#include <linux/types.h>

struct alp_dev;
struct device;

/* ALP packet types (A/330 Table 5.2) */
#define ALP_TYPE_IPV4		0
#define ALP_TYPE_COMPRESSED_IP	2
#define ALP_TYPE_LLS		4
#define ALP_TYPE_TYPE_EXT	6
#define ALP_TYPE_MPEG2_TS	7

/* Segmentation buffer size: max 32 segments × 2047 bytes */
#define ALP_SEG_BUF_SIZE	65536

/* Per-device ALP protocol statistics exposed via ethtool -S */
struct alp_stats {
	u64 ip_packets;		/* IPv4 packets delivered to stack */
	u64 ip_bytes;		/* IPv4 bytes delivered */
	u64 seg_completed;	/* segmented packets reassembled */
	u64 concat_delivered;	/* concatenated components delivered */
	u64 type_compressed_ip;	/* type 2: compressed IP */
	u64 type_lls;		/* type 4: link-layer signalling */
	u64 type_ext;		/* type 6: type extension */
	u64 type_ts;		/* type 7: MPEG-2 TS */
	u64 type_unknown;	/* types 1, 3, 5: undefined */
	u64 ext_hdr_compressed_ip; /* HM=1 type 2: compressed IP */
	u64 ext_hdr_lls;	/* HM=1 type 4: link-layer signalling */
	u64 ext_hdr_ext;	/* HM=1 type 6: type extension */
	u64 ext_hdr_ts;		/* HM=1 type 7: MPEG-2 TS */
	u64 ext_hdr_unknown;	/* HM=1 types 1, 3, 5: undefined */
	u64 seg_errors;		/* segmentation sequence/overflow errors */
	u64 frame_err_single;	/* PC=0 single packet length mismatch */
	u64 frame_err_seg;	/* segmented packet length mismatch */
	u64 frame_err_concat;	/* concatenated packet length/parse error */
	u64 short_packets;	/* packets too short to parse */
	u64 skb_alloc_fail;	/* skb allocation failures */
	u64 netif_rx_fail;	/* netif_rx delivery failures */
};

/**
 * struct alp_ops - transport-layer callbacks
 * @open:  called when the net device is brought up (start feed)
 * @stop:  called when the net device is brought down (stop feed)
 *
 * The transport layer (e.g. ALP-div-TS decap) registers these
 * to start/stop its data feed in response to netif up/down.
 */
struct alp_ops {
	int  (*open)(void *priv);
	void (*stop)(void *priv);
};

/**
 * alp_attach - Create and register an ALP net device
 * @parent:    parent device for SET_NETDEV_DEV
 * @ops:       transport-layer callbacks (may be NULL)
 * @ops_priv:  opaque pointer passed to ops callbacks
 *
 * Returns pointer to alp_dev on success, ERR_PTR on failure.
 */
struct alp_dev *alp_attach(struct device *parent, const struct alp_ops *ops,
			   void *ops_priv);

/**
 * alp_detach - Unregister and free an ALP net device
 * @alp: ALP device to tear down (NULL-safe)
 */
void alp_detach(struct alp_dev *alp);

/**
 * alp_process - Feed a complete ALP packet for processing
 * @alp: ALP device context
 * @buf: pointer to the complete ALP packet (header + payload)
 * @len: total length in bytes
 *
 * Parses the ALP header and dispatches the payload by type:
 *   - Type 0 (IPv4): delivers to the kernel network stack
 *   - Segmented (PC=1,S/C=0): reassembles across multiple calls
 *   - Concatenated (PC=1,S/C=1): splits into components
 */
void alp_process(struct alp_dev *alp, const u8 *buf, u32 len);

/**
 * alp_feed - Feed a buffer that may contain multiple ALP packets
 * @alp: ALP device context
 * @buf: pointer to ALP data (may contain multiple back-to-back packets)
 * @len: total length in bytes
 *
 * Extracts and processes complete ALP packets from the buffer using
 * their length fields.  Returns the number of bytes consumed; any
 * unconsumed trailing bytes are the caller's responsibility to carry
 * over for the next call.
 */
u32 alp_feed(struct alp_dev *alp, const u8 *buf, u32 len);

/**
 * alp_get_netdev - Get the net_device associated with this ALP context
 * @alp: ALP device context
 *
 * Returns the net_device pointer, or NULL if alp is NULL.
 * Used by demod drivers for carrier on/off control.
 */
struct net_device *alp_get_netdev(struct alp_dev *alp);

#endif /* _ALP_H_ */
