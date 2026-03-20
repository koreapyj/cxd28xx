/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * CXD2878 ALP-div-TS decapsulation layer
 *
 * Device-specific constants and API for extracting ALP packets
 * from the CXD2878's proprietary 3-byte-header TS output.
 */
#ifndef _CXD2878_ALP_H_
#define _CXD2878_ALP_H_

/* ALP-div-TS uses a 3-byte TS header (no CC/adaptation byte) */
#define CXD2878_ALP_TS_HDR_SIZE	3
#define CXD2878_ALP_TS_PAYLOAD	(188 - CXD2878_ALP_TS_HDR_SIZE)

/* PID 0x2000 = wildcard (all PIDs) for ALP-div-TS */
#define CXD2878_ALP_DEFAULT_PID	0x2000

/* TS reassembly buffer: must hold the largest ALP packet */
#define CXD2878_ALP_BUF_SIZE	16384

/* TS-level statistics (device-specific, exposed via sysfs) */
struct cxd2878_alp_ts_stats {
	u64 ts_reassembled;	/* TS packets processed */
	u64 ts_tei;		/* TS packets with TEI set */
	u64 ts_sync_miss;	/* TS packets with bad sync byte */
	u64 ts_null_skip;	/* null TS packets skipped */
	u64 ts_overflow;	/* reassembly buffer overflows */
	u64 frame_err_pusi;	/* PUSI completion found short buffer */
	u64 raw_bytes_in;	/* total bytes into feed_raw */
};

struct cxd2878_dev;
struct dmx_demux;
struct dvb_demux;

/* Sysfs attribute group for TS-level stats */
extern const struct attribute_group cxd2878_alp_ts_stat_group;

/* Store the sysfs parent device for TS stats */
void cxd2878_alp_register_sysfs(struct cxd2878_dev *dev, struct device *parent);
void cxd2878_alp_unregister_sysfs(struct cxd2878_dev *dev);

int  cxd2878_alp_attach(struct cxd2878_dev *dev, struct dmx_demux *demux,
			struct dvb_demux *dvb_demux, struct device *parent);
void cxd2878_alp_detach(struct cxd2878_dev *dev);
void cxd2878_alp_feed_raw(struct cxd2878_dev *dev, const u8 *buf, u32 len);

#endif /* _CXD2878_ALP_H_ */
