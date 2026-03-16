/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * CXD2878 ALP (ATSC Link-layer Protocol) virtual network adapter
 *
 * Extracts IP packets from ALP-div-TS and injects them into the
 * kernel network stack via a raw-ip net_device.
 */
#ifndef _CXD2878_ALP_H_
#define _CXD2878_ALP_H_

#define CXD2878_ALP_DEFAULT_PID	0x2000	/* PID 0x2000 = all PIDs (wildcard) */

#define CXD2878_ALP_TS_HDR_SIZE	3
#define CXD2878_ALP_TS_PAYLOAD	(188 - CXD2878_ALP_TS_HDR_SIZE)

/* ALP packet types (A/330 Table 5.2) */
#define ALP_TYPE_IPV4		0
#define ALP_TYPE_COMPRESSED_IP	2
#define ALP_TYPE_LLS		4
#define ALP_TYPE_TYPE_EXT	6
#define ALP_TYPE_MPEG2_TS	7

struct cxd2878_dev;
struct dmx_demux;
struct dvb_demux;

int  cxd2878_alp_attach(struct cxd2878_dev *dev, struct dmx_demux *demux,
			struct dvb_demux *dvb_demux, struct device *parent);
void cxd2878_alp_detach(struct cxd2878_dev *dev);

#endif /* _CXD2878_ALP_H_ */
