/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ATSC 3.0 ALP (ATSC Link-layer Protocol) parser and network interface.
 *
 * Shared between USB bridge drivers (IT930x, TBS5530, etc.) that use
 * the CXD2878 demodulator for ATSC 3.0 reception.
 *
 * Copyright (c) 2026 Yoonji Park <koreapyj@dcmys.kr>
 */
#ifndef _ATSC3_ALP_H_
#define _ATSC3_ALP_H_

#include <linux/netdevice.h>
#include <media/dvb_frontend.h>

#ifndef SYS_ATSC3
#define SYS_ATSC3 35
#endif

#define ALP_BUF_SIZE 65536

/**
 * struct atsc3_alp - ATSC 3.0 ALP state for one adapter
 * @net:              network device (atscN)
 * @buf:              ALP reassembly buffer (ALP_BUF_SIZE bytes)
 * @pos:              current write position in buf
 * @fe:               DVB frontend (for delivery_system check in net_open)
 * @start_streaming:  optional callback, called on net_open
 * @stop_streaming:   optional callback, called on net_stop
 * @priv:             driver private data passed to callbacks
 */
struct atsc3_alp {
	struct net_device *net;
	u8 *buf;
	unsigned int pos;
	struct dvb_frontend *fe;
	struct device *dev;		/* parent device for sysfs association */
	int (*start_streaming)(void *priv);
	void (*stop_streaming)(void *priv);
	void *priv;
};

int atsc3_alp_register_netdev(struct atsc3_alp *alp);
void atsc3_alp_unregister_netdev(struct atsc3_alp *alp);
void atsc3_alp_feed(struct atsc3_alp *alp, const u8 *data, int len);

#endif /* _ATSC3_ALP_H_ */
