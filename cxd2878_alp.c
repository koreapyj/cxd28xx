// SPDX-License-Identifier: GPL-2.0-only
/*
 * CXD2878 ALP-div-TS decapsulation
 *
 * Extracts complete ALP packets from the CXD2878's proprietary
 * ALP-div-TS output format (3-byte TS header, no CC) and feeds
 * them to the generic ALP module for protocol-level processing.
 *
 * This layer handles:
 *   - TS sync (0x47), null PID filtering, TEI counting
 *   - PUSI / pointer-based ALP packet boundary detection
 *   - Streaming buffer reassembly (alp_check_complete)
 *   - DVB demux TS feed allocation
 */

#include <linux/netdevice.h>
#include <media/dvb_demux.h>

#include "alp.h"
#include "cxd2878.h"
#include "cxd2878_priv.h"
#include "cxd2878_alp.h"

/* ── TS reassembly helpers ─────────────────────────────────────────── */

static void alp_ctx_reset(struct cxd2878_dev *dev)
{
	dev->alp_buf_len = 0;
	dev->alp_expected_len = 0;
	dev->alp_active = false;
}

static int alp_ctx_append(struct cxd2878_dev *dev, const u8 *data, u32 len)
{
	if (dev->alp_buf_len + len > CXD2878_ALP_BUF_SIZE) {
		dev->alp_ts_stats.ts_overflow++;
		alp_ctx_reset(dev);
		return -1;
	}
	memcpy(dev->alp_buf + dev->alp_buf_len, data, len);
	dev->alp_buf_len += len;
	return 0;
}

/* ── ALP reassembly: check for complete packets in buffer ──────────── */

static void alp_check_complete(struct cxd2878_dev *dev)
{
	while (dev->alp_active) {
		if (dev->alp_expected_len == 0 && dev->alp_buf_len >= 2) {
			u8 b0 = dev->alp_buf[0];
			u8 pc = (b0 >> 4) & 1;
			u16 length = ((b0 & 0x07) << 8) | dev->alp_buf[1];

			if (pc && (b0 & 0x08)) {
				/* PC=1, S/C=1 (concatenation): 4-bit MSB */
				if (dev->alp_buf_len < 3)
					break;
				u8 msb = (dev->alp_buf[2] >> 4) & 0x0F;
				dev->alp_expected_len =
					((msb << 11) | length) + 2;
			} else if (!pc && (b0 & 0x08)) {
				/* PC=0, HM=1: additional header + optional SIF/HEF */
				if (dev->alp_buf_len < 3)
					break;
				u8 addl = dev->alp_buf[2];
				u8 msb = (addl >> 3) & 0x1F;
				u8 sif = (addl >> 1) & 1;
				u8 hef = addl & 1;
				u32 full_length = ((u32)msb << 11) | length;
				u32 hdr = 3 + (sif ? 1 : 0);
				if (hef) {
					if (dev->alp_buf_len < hdr + 2)
						break;
					u8 ext_len_m1 = dev->alp_buf[hdr + 1];
					hdr += 2 + ext_len_m1 + 1;
				}
				dev->alp_expected_len = hdr + full_length;
			} else {
				/* PC=0/HM=0 or PC=1/S/C=0 */
				dev->alp_expected_len = length + 2;
			}
		}

		if (dev->alp_expected_len == 0 ||
		    dev->alp_buf_len < dev->alp_expected_len)
			break;

		/* Hand complete ALP packet to the generic module */
		alp_process(dev->alp, dev->alp_buf, dev->alp_expected_len);

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

	/* TEI — count but process normally (ALP-div-TS TEI is not reliable) */
	if (pkt[1] & 0x80)
		dev->alp_ts_stats.ts_tei++;

	/* Skip null packets (padding) */
	pid = ((pkt[1] & 0x1F) << 8) | pkt[2];
	if (pid == 0x1FFF) {
		dev->alp_ts_stats.ts_null_skip++;
		return;
	}

	dev->alp_ts_stats.ts_reassembled++;

	pusi = (pkt[1] >> 6) & 1;
	payload = pkt + CXD2878_ALP_TS_HDR_SIZE;

	if (pusi) {
		pointer = payload[0];
		if (pointer > CXD2878_ALP_TS_PAYLOAD - 1)
			return;

		/* Complete previous ALP packet with tail data */
		if (dev->alp_active && pointer > 0) {
			alp_ctx_append(dev, payload + 1, pointer);
			if (dev->alp_buf_len > 0) {
				dev->alp_expected_len = 0;
				alp_check_complete(dev);
				if (dev->alp_active)
					dev->alp_ts_stats.frame_err_pusi++;
			}
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
			/*
			 * Skip inline ALP parsing on TEI-flagged packets.
			 * Corrupt data could produce wrong ALP headers,
			 * misaligning the buffer. Let the next PUSI handle
			 * completion via pointer-based length check.
			 */
			if (!(pkt[1] & 0x80))
				alp_check_complete(dev);
		}
	}
}

/* ── Direct raw buffer feed ────────────────────────────────────────── */

void cxd2878_alp_feed_raw(struct cxd2878_dev *dev, const u8 *buf, u32 len)
{
	u32 pos;

	if (!dev->alp || !dev->alp_feed)
		return;

	dev->alp_ts_stats.raw_bytes_in += len;

	for (pos = 0; pos + 188 <= len; pos += 188) {
		if (buf[pos] == 0x47)
			cxd2878_alp_process_ts(dev, buf + pos);
		else
			dev->alp_ts_stats.ts_sync_miss++;
	}
}
EXPORT_SYMBOL_GPL(cxd2878_alp_feed_raw);

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

/* ── Sysfs TS-level statistics (on DVB frontend device) ────────────── */

#define TS_STAT_ATTR(_name) \
static ssize_t alp_ts_##_name##_show(struct device *d, \
				     struct device_attribute *attr, \
				     char *buf) \
{ \
	struct cxd2878_dev *dev = dev_get_drvdata(d); \
	if (!dev) return -ENODEV; \
	return sysfs_emit(buf, "%llu\n", dev->alp_ts_stats._name); \
} \
static DEVICE_ATTR_RO(alp_ts_##_name)

TS_STAT_ATTR(ts_reassembled);
TS_STAT_ATTR(ts_tei);
TS_STAT_ATTR(ts_sync_miss);
TS_STAT_ATTR(ts_null_skip);
TS_STAT_ATTR(ts_overflow);
TS_STAT_ATTR(frame_err_pusi);
TS_STAT_ATTR(raw_bytes_in);

static struct attribute *alp_ts_stat_attrs[] = {
	&dev_attr_alp_ts_ts_reassembled.attr,
	&dev_attr_alp_ts_ts_tei.attr,
	&dev_attr_alp_ts_ts_sync_miss.attr,
	&dev_attr_alp_ts_ts_null_skip.attr,
	&dev_attr_alp_ts_ts_overflow.attr,
	&dev_attr_alp_ts_frame_err_pusi.attr,
	&dev_attr_alp_ts_raw_bytes_in.attr,
	NULL,
};

const struct attribute_group cxd2878_alp_ts_stat_group = {
	.name = "alp_ts_stats",
	.attrs = alp_ts_stat_attrs,
};

void cxd2878_alp_register_sysfs(struct cxd2878_dev *dev, struct device *parent)
{
	dev_set_drvdata(parent, dev);
	sysfs_create_group(&parent->kobj, &cxd2878_alp_ts_stat_group);
	dev->alp_sysfs_dev = parent;
}
EXPORT_SYMBOL_GPL(cxd2878_alp_register_sysfs);

void cxd2878_alp_unregister_sysfs(struct cxd2878_dev *dev)
{
	if (dev->alp_sysfs_dev) {
		sysfs_remove_group(&dev->alp_sysfs_dev->kobj,
				   &cxd2878_alp_ts_stat_group);
		dev->alp_sysfs_dev = NULL;
	}
}
EXPORT_SYMBOL_GPL(cxd2878_alp_unregister_sysfs);

/* ── Transport ops (called by alp.ko on net device open/stop) ──────── */

static int cxd2878_alp_start_feed(struct cxd2878_dev *dev);
static void cxd2878_alp_stop_feed(struct cxd2878_dev *dev);

static int cxd2878_alp_op_open(void *priv)
{
	return cxd2878_alp_start_feed(priv);
}

static void cxd2878_alp_op_stop(void *priv)
{
	cxd2878_alp_stop_feed(priv);
}

static const struct alp_ops cxd2878_alp_ops = {
	.open = cxd2878_alp_op_open,
	.stop = cxd2878_alp_op_stop,
};

/* ── Attach / detach ───────────────────────────────────────────────── */

int cxd2878_alp_attach(struct cxd2878_dev *dev, struct dmx_demux *demux,
		       struct dvb_demux *dvb_demux, struct device *parent)
{
	struct alp_dev *alp;

	dev->alp_demux = demux;
	dev->alp_dvb_demux = dvb_demux;
	dev->alp_feed = NULL;
	dev->alp_carrier = false;
	alp_ctx_reset(dev);
	memset(&dev->alp_ts_stats, 0, sizeof(dev->alp_ts_stats));

	alp = alp_attach(parent, &cxd2878_alp_ops, dev);
	if (IS_ERR(alp)) {
		dev->alp_demux = NULL;
		dev->alp_dvb_demux = NULL;
		return PTR_ERR(alp);
	}

	dev->alp = alp;
	return 0;
}
EXPORT_SYMBOL_GPL(cxd2878_alp_attach);

void cxd2878_alp_detach(struct cxd2878_dev *dev)
{
	if (!dev->alp)
		return;

	if (dev->alp_feed) {
		dev->alp_feed->stop_filtering(dev->alp_feed);
		dev->alp_demux->release_ts_feed(dev->alp_demux,
						dev->alp_feed);
		dev->alp_feed = NULL;
	}

	alp_detach(dev->alp);
	dev->alp = NULL;
	dev->alp_demux = NULL;
	dev->alp_dvb_demux = NULL;
}
EXPORT_SYMBOL_GPL(cxd2878_alp_detach);

/* ── DVB feed start/stop (called from net device open/stop) ────────── */

static int cxd2878_alp_start_feed(struct cxd2878_dev *dev)
{
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
	memset(&dev->alp_ts_stats, 0, sizeof(dev->alp_ts_stats));
	return 0;
}

static void cxd2878_alp_stop_feed(struct cxd2878_dev *dev)
{
	if (dev->alp_feed) {
		dev->alp_feed->stop_filtering(dev->alp_feed);
		dev->alp_demux->release_ts_feed(dev->alp_demux,
						dev->alp_feed);
		dev->alp_feed = NULL;
	}

	alp_ctx_reset(dev);
}
