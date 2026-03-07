/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _TBS5530_H_
#define _TBS5530_H_

#include <linux/usb.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <media/dvb_frontend.h>
#include <media/dvb_demux.h>
#include <media/dvb_net.h>
#include <media/dmxdev.h>

#include "compat.h"

#define NUM_URBS	8
#define URB_BUF_SIZE	(188 * 348)

struct tbs5530_dev {
	struct usb_device *udev;
	struct usb_interface *intf;

	/* I2C */
	struct i2c_adapter i2c_adap;
	struct mutex i2c_mutex;

	/* DVB */
	struct dvb_adapter dvb_adap;
	struct dvb_frontend *fe_ter;
	struct dvb_frontend *fe_sat;
	struct dvb_frontend_ops fe_ter_ops_orig;
	struct dvb_frontend_ops fe_sat_ops_orig;
	struct dmxdev dmxdev;
	struct dvb_demux demux;
	struct dvb_net dvb_net;
	struct i2c_client *i2c_client_sat;

	/* Streaming */
	int active_fe;
	int feed_count;
	struct urb *urbs[NUM_URBS];
	u8 *urb_bufs[NUM_URBS];
	bool streaming;
};

#endif
