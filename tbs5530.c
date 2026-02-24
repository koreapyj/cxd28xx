/*
 * TurboSight TBS 5530 USB DVB driver
 *
 * Standalone driver with direct USB/DVB management.
 * Supports dual frontends: CXD2878 (DVB-T/T2/C/C2, ISDB-T/C, ATSC/3.0, J83B)
 * and M88RS6060 (DVB-S/S2/S2X).
 *
 * Copyright (c) 2022 Davin <Davin@tbsdtv.com>
 * Copyright (c) 2026 Yoonji Park <koreapyj@dcmys.kr>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, version 2.
 */

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/firmware.h>
#include <linux/delay.h>

#include <media/dvb_frontend.h>
#include <media/dmxdev.h>
#include <media/dvb_demux.h>
#include <media/dvb_net.h>

#include "cxd2878.h"
#include "m88rs6060.h"
#include "atsc3_alp.h"

/* USB constants */
#define TBS5530_URB_COUNT	8
#define TBS5530_URB_BUF_SIZE	4096
#define TBS5530_EP_TS		0x82
#define TBS5530_FIRMWARE	"dvb-usb-id5530.fw"

/* USB control message direction */
#define TBS5530_READ_MSG	0
#define TBS5530_WRITE_MSG	1

/* DVB adapter number */
static short adapter_nr[] = { -1 };

struct tbs5530_dev {
	struct usb_device *udev;
	struct usb_interface *intf;
	struct mutex io_lock;

	/* I2C */
	struct i2c_adapter i2c;

	/* DVB */
	struct dvb_adapter dvb_adapter;
	struct dvb_frontend *fe_ter;	/* CXD2878 */
	struct dvb_frontend *fe_sat;	/* M88RS6060 */
	struct dvb_demux demux;
	struct dmxdev dmxdev;
	struct dvb_net dvbnet;

	/* M88RS6060 I2C client */
	struct i2c_client *i2c_client_sat;

	/* Streaming */
	int feeding;
	int active_fe;			/* 0=ter/cable, 1=sat */
	bool streaming;
	struct urb *urbs[TBS5530_URB_COUNT];
	u8 *urb_bufs[TBS5530_URB_COUNT];
	dma_addr_t urb_dma[TBS5530_URB_COUNT];

	/* Saved original frontend init ops */
	int (*orig_ter_init)(struct dvb_frontend *);
	int (*orig_sat_init)(struct dvb_frontend *);

	/* ATSC 3.0 */
	struct atsc3_alp alp;
};

/* -------- USB vendor control I/O -------- */

static int tbs5530_op_rw(struct usb_device *dev, u8 request, u16 value,
			  u16 index, u8 *data, u16 len, int flags)
{
	int ret;
	void *u8buf;

	unsigned int pipe = (flags == TBS5530_READ_MSG) ?
		usb_rcvctrlpipe(dev, 0) : usb_sndctrlpipe(dev, 0);
	u8 request_type = (flags == TBS5530_READ_MSG) ?
		USB_DIR_IN : USB_DIR_OUT;

	u8buf = kmalloc(len, GFP_KERNEL);
	if (!u8buf)
		return -ENOMEM;

	if (flags == TBS5530_WRITE_MSG)
		memcpy(u8buf, data, len);

	ret = usb_control_msg(dev, pipe, request,
			      request_type | USB_TYPE_VENDOR,
			      value, index, u8buf, len, 2000);

	if (flags == TBS5530_READ_MSG)
		memcpy(data, u8buf, len);

	kfree(u8buf);
	return ret;
}

/* -------- I2C adapter -------- */

static int tbs5530_i2c_transfer(struct i2c_adapter *adap,
				struct i2c_msg msg[], int num)
{
	struct tbs5530_dev *dev = i2c_get_adapdata(adap);
	int i;
	u8 buf6[50];
	u8 inbuf[50];

	if (!dev)
		return -ENODEV;
	if (mutex_lock_interruptible(&dev->io_lock) < 0)
		return -EAGAIN;

	switch (num) {
	case 2:
		buf6[0] = msg[1].len;
		buf6[1] = msg[0].addr << 1;
		buf6[2] = msg[0].buf[0];

		tbs5530_op_rw(dev->udev, 0x90, 0, 0,
			      buf6, 3, TBS5530_WRITE_MSG);
		tbs5530_op_rw(dev->udev, 0x91, 0, 0,
			      inbuf, buf6[0], TBS5530_READ_MSG);
		memcpy(msg[1].buf, inbuf, msg[1].len);
		break;
	case 1:
		buf6[0] = msg[0].len + 1;
		buf6[1] = msg[0].addr << 1;
		for (i = 0; i < msg[0].len; i++)
			buf6[2 + i] = msg[0].buf[i];
		tbs5530_op_rw(dev->udev, 0x80, 0, 0,
			      buf6, msg[0].len + 2, TBS5530_WRITE_MSG);
		break;
	}

	mutex_unlock(&dev->io_lock);
	return num;
}

static u32 tbs5530_i2c_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C;
}

static struct i2c_algorithm tbs5530_i2c_algo = {
	.master_xfer = tbs5530_i2c_transfer,
	.functionality = tbs5530_i2c_func,
};

/* -------- Firmware loading -------- */

static int tbs5530_load_firmware(struct tbs5530_dev *dev)
{
	const struct firmware *fw;
	u8 *p;
	int ret = 0, i;
	u8 reset;

	ret = request_firmware(&fw, TBS5530_FIRMWARE, &dev->intf->dev);
	if (ret) {
		dev_err(&dev->intf->dev,
			"firmware %s not found (%d)\n", TBS5530_FIRMWARE, ret);
		return ret;
	}

	p = kmalloc(fw->size, GFP_KERNEL);
	if (!p) {
		release_firmware(fw);
		return -ENOMEM;
	}

	dev_info(&dev->intf->dev, "downloading TBS5530 firmware\n");

	/* Stop CPU */
	reset = 1;
	tbs5530_op_rw(dev->udev, 0xa0, 0x7f92, 0, &reset, 1,
		      TBS5530_WRITE_MSG);
	tbs5530_op_rw(dev->udev, 0xa0, 0xe600, 0, &reset, 1,
		      TBS5530_WRITE_MSG);

	memcpy(p, fw->data, fw->size);
	for (i = 0; i < fw->size; i += 0x40) {
		if (tbs5530_op_rw(dev->udev, 0xa0, i, 0, p + i, 0x40,
				  TBS5530_WRITE_MSG) != 0x40) {
			dev_err(&dev->intf->dev,
				"firmware transfer error at offset %d\n", i);
			ret = -EINVAL;
			break;
		}
	}

	/* Restart CPU */
	reset = 0;
	if (!ret) {
		if (tbs5530_op_rw(dev->udev, 0xa0, 0x7f92, 0, &reset, 1,
				  TBS5530_WRITE_MSG) != 1) {
			dev_err(&dev->intf->dev,
				"could not restart USB controller CPU\n");
			ret = -EINVAL;
		}
	}
	if (!ret) {
		if (tbs5530_op_rw(dev->udev, 0xa0, 0xe600, 0, &reset, 1,
				  TBS5530_WRITE_MSG) != 1) {
			dev_err(&dev->intf->dev,
				"could not restart USB controller CPU\n");
			ret = -EINVAL;
		}
	}

	msleep(100);
	kfree(p);
	release_firmware(fw);
	return ret;
}

/* -------- MAC address (informational) -------- */

static void tbs5530_read_mac_address(struct tbs5530_dev *dev)
{
	int i, ret;
	u8 ibuf[3];
	u8 mac[6];

	for (i = 16; i < 22; i++) {
		ibuf[0] = 1;
		ibuf[1] = 0xa0;
		ibuf[2] = i;
		ret = tbs5530_op_rw(dev->udev, 0x90, 0, 0,
				    ibuf, 3, TBS5530_WRITE_MSG);
		if (ret < 0)
			return;
		ret = tbs5530_op_rw(dev->udev, 0x91, 0, 0,
				    ibuf, 1, TBS5530_READ_MSG);
		if (ret < 0)
			return;
		mac[i - 16] = ibuf[0];
	}

	dev_info(&dev->intf->dev, "MAC address: %pM\n", mac);
}

/* -------- URB management -------- */

static void tbs5530_urb_complete(struct urb *urb)
{
	struct tbs5530_dev *dev = urb->context;

	if (!dev->streaming)
		return;

	switch (urb->status) {
	case 0:
		if (urb->actual_length > 0) {
			dvb_dmx_swfilter(&dev->demux,
					 urb->transfer_buffer,
					 urb->actual_length);
			atsc3_alp_feed(&dev->alp,
				       urb->transfer_buffer,
				       urb->actual_length);
		}
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		return;
	default:
		dev_dbg(&dev->intf->dev, "URB error %d\n", urb->status);
		break;
	}

	usb_submit_urb(urb, GFP_ATOMIC);
}

static int tbs5530_alloc_urbs(struct tbs5530_dev *dev)
{
	int i;

	for (i = 0; i < TBS5530_URB_COUNT; i++) {
		dev->urbs[i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!dev->urbs[i])
			goto err;

		dev->urb_bufs[i] = usb_alloc_coherent(dev->udev,
						      TBS5530_URB_BUF_SIZE,
						      GFP_KERNEL,
						      &dev->urb_dma[i]);
		if (!dev->urb_bufs[i])
			goto err;
	}
	return 0;

err:
	while (--i >= 0) {
		usb_free_coherent(dev->udev, TBS5530_URB_BUF_SIZE,
				  dev->urb_bufs[i], dev->urb_dma[i]);
		dev->urb_bufs[i] = NULL;
		usb_free_urb(dev->urbs[i]);
		dev->urbs[i] = NULL;
	}
	return -ENOMEM;
}

static void tbs5530_free_urbs(struct tbs5530_dev *dev)
{
	int i;

	for (i = 0; i < TBS5530_URB_COUNT; i++) {
		if (dev->urb_bufs[i])
			usb_free_coherent(dev->udev, TBS5530_URB_BUF_SIZE,
					  dev->urb_bufs[i], dev->urb_dma[i]);
		usb_free_urb(dev->urbs[i]);
	}
}

/* -------- Streaming control -------- */

static int tbs5530_start_streaming(struct tbs5530_dev *dev)
{
	int i, ret;
	u8 buf[2];

	if (dev->streaming)
		return 0;

	/* Select active frontend TS routing */
	buf[0] = 10;
	buf[1] = dev->active_fe;
	mutex_lock(&dev->io_lock);
	tbs5530_op_rw(dev->udev, 0x8a, 0, 0, buf, 2, TBS5530_WRITE_MSG);
	mutex_unlock(&dev->io_lock);

	/* Fill and submit URBs */
	for (i = 0; i < TBS5530_URB_COUNT; i++) {
		usb_fill_bulk_urb(dev->urbs[i], dev->udev,
				  usb_rcvbulkpipe(dev->udev, TBS5530_EP_TS),
				  dev->urb_bufs[i], TBS5530_URB_BUF_SIZE,
				  tbs5530_urb_complete, dev);
		dev->urbs[i]->transfer_dma = dev->urb_dma[i];
		dev->urbs[i]->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	}

	dev->streaming = true;

	for (i = 0; i < TBS5530_URB_COUNT; i++) {
		ret = usb_submit_urb(dev->urbs[i], GFP_KERNEL);
		if (ret) {
			dev_err(&dev->intf->dev,
				"URB submit failed (%d)\n", ret);
			dev->streaming = false;
			while (--i >= 0)
				usb_kill_urb(dev->urbs[i]);
			return ret;
		}
	}

	return 0;
}

static void tbs5530_stop_streaming(struct tbs5530_dev *dev)
{
	int i;

	if (dev->feeding > 0)
		return;

	if (!dev->streaming)
		return;

	dev->streaming = false;

	for (i = 0; i < TBS5530_URB_COUNT; i++)
		usb_kill_urb(dev->urbs[i]);
}

/* -------- DVB demux feed -------- */

static int tbs5530_start_feed(struct dvb_demux_feed *feed)
{
	struct tbs5530_dev *dev = feed->demux->priv;
	int ret;

	dev->feeding++;

	if (dev->feeding == 1) {
		ret = tbs5530_start_streaming(dev);
		if (ret) {
			dev->feeding--;
			return ret;
		}
	}

	return 0;
}

static int tbs5530_stop_feed(struct dvb_demux_feed *feed)
{
	struct tbs5530_dev *dev = feed->demux->priv;

	if (--dev->feeding == 0)
		tbs5530_stop_streaming(dev);

	return 0;
}

/* -------- Frontend init wrappers (active_fe tracking) -------- */

static int tbs5530_fe_ter_init(struct dvb_frontend *fe)
{
	struct tbs5530_dev *dev = fe->dvb->priv;

	dev->active_fe = 0;
	if (dev->orig_ter_init)
		return dev->orig_ter_init(fe);
	return 0;
}

static int tbs5530_fe_sat_init(struct dvb_frontend *fe)
{
	struct tbs5530_dev *dev = fe->dvb->priv;

	dev->active_fe = 1;
	if (dev->orig_sat_init)
		return dev->orig_sat_init(fe);
	return 0;
}

/* -------- ALP streaming callbacks -------- */

static int tbs5530_alp_start(void *priv)
{
	return tbs5530_start_streaming(priv);
}

static void tbs5530_alp_stop(void *priv)
{
	tbs5530_stop_streaming(priv);
}

/* -------- Frontend attachment -------- */

static struct cxd2878_config tbs5530_cfg = {
	.addr_slvt = 0x64,
	.xtal = SONY_DEMOD_XTAL_24000KHz,
	.tuner_addr = 0x60,
	.tuner_xtal = SONY_ASCOT3_XTAL_24000KHz,
	.ts_mode = 1,
	.ts_ser_data = 0,
	.ts_clk = 1,
	.ts_clk_mask = 1,
	.ts_valid = 0,
	.atscCoreDisable = 0,
	.lock_flag = 1,
	.write_properties = NULL,
	.read_properties = NULL,
};

static int tbs5530_frontend_attach_ter(struct tbs5530_dev *dev)
{
	dev->fe_ter = dvb_attach(cxd2878_attach, &tbs5530_cfg, &dev->i2c);
	if (!dev->fe_ter) {
		dev_err(&dev->intf->dev, "CXD2878 attach failed\n");
		return -ENODEV;
	}

	strscpy(dev->fe_ter->ops.info.name,
		"TBS 5530 DVB-T/T2/C/C2,ISDB-T/C,ATSC/3.0,J83B",
		sizeof(dev->fe_ter->ops.info.name));

	/* Hook init for active_fe tracking */
	dev->orig_ter_init = dev->fe_ter->ops.init;
	dev->fe_ter->ops.init = tbs5530_fe_ter_init;

	return 0;
}

static int tbs5530_frontend_attach_sat(struct tbs5530_dev *dev)
{
	struct i2c_client *client;
	struct i2c_board_info info;
	struct m88rs6060_cfg m88rs6060_config;
	u8 buf[2];

	memset(&m88rs6060_config, 0, sizeof(m88rs6060_config));
	m88rs6060_config.fe = &dev->fe_sat;
	m88rs6060_config.clk = 27000000;
	m88rs6060_config.i2c_wr_max = 33;
	m88rs6060_config.ts_mode = MtFeTsOutMode_Common;
	m88rs6060_config.ts_pinswitch = 0;
	m88rs6060_config.envelope_mode = 0;
	m88rs6060_config.demod_adr = 0x69;
	m88rs6060_config.tuner_adr = 0x2c;
	m88rs6060_config.repeater_value = 0x11;
	m88rs6060_config.read_properties = NULL;
	m88rs6060_config.write_properties = NULL;

	memset(&info, 0, sizeof(info));
	strscpy(info.type, "m88rs6060", I2C_NAME_SIZE);
	info.addr = 0x69;
	info.platform_data = &m88rs6060_config;

	request_module(info.type);
	client = i2c_new_client_device(&dev->i2c, &info);
	if (!i2c_client_has_driver(client)) {
		dev_err(&dev->intf->dev, "M88RS6060 attach failed\n");
		return -ENODEV;
	}
	if (!try_module_get(client->dev.driver->owner)) {
		i2c_unregister_device(client);
		return -ENODEV;
	}

	dev->i2c_client_sat = client;

	/* Tuner power and TS init */
	buf[0] = 0;
	buf[1] = 0;
	tbs5530_op_rw(dev->udev, 0xb7, 0, 0, buf, 2, TBS5530_WRITE_MSG);
	buf[0] = 8;
	buf[1] = 1;
	tbs5530_op_rw(dev->udev, 0x8a, 0, 0, buf, 2, TBS5530_WRITE_MSG);
	msleep(10);

	strscpy(dev->fe_sat->ops.info.name,
		"TBS 5530 DVB-S/S2/S2X",
		sizeof(dev->fe_sat->ops.info.name));

	/* Hook init for active_fe tracking */
	dev->orig_sat_init = dev->fe_sat->ops.init;
	dev->fe_sat->ops.init = tbs5530_fe_sat_init;

	return 0;
}

/* -------- DVB registration -------- */

static int tbs5530_dvb_init(struct tbs5530_dev *dev)
{
	int ret;

	ret = dvb_register_adapter(&dev->dvb_adapter, "TBS 5530",
				   THIS_MODULE, &dev->intf->dev, adapter_nr);
	if (ret < 0) {
		dev_err(&dev->intf->dev,
			"dvb_register_adapter failed (%d)\n", ret);
		return ret;
	}

	dev->dvb_adapter.priv = dev;

	/* Terrestrial/cable frontend (CXD2878) */
	ret = tbs5530_frontend_attach_ter(dev);
	if (ret)
		goto err_adapter;

	ret = dvb_register_frontend(&dev->dvb_adapter, dev->fe_ter);
	if (ret) {
		dev_err(&dev->intf->dev,
			"dvb_register_frontend (ter) failed (%d)\n", ret);
		dvb_frontend_detach(dev->fe_ter);
		goto err_adapter;
	}

	/* Satellite frontend (M88RS6060) */
	ret = tbs5530_frontend_attach_sat(dev);
	if (ret)
		goto err_fe_ter;

	ret = dvb_register_frontend(&dev->dvb_adapter, dev->fe_sat);
	if (ret) {
		dev_err(&dev->intf->dev,
			"dvb_register_frontend (sat) failed (%d)\n", ret);
		dvb_frontend_detach(dev->fe_sat);
		goto err_sat_client;
	}

	/* Demux */
	dev->demux.dmx.capabilities = DMX_TS_FILTERING | DMX_SECTION_FILTERING;
	dev->demux.priv = dev;
	dev->demux.filternum = 256;
	dev->demux.feednum = 256;
	dev->demux.start_feed = tbs5530_start_feed;
	dev->demux.stop_feed = tbs5530_stop_feed;
	ret = dvb_dmx_init(&dev->demux);
	if (ret)
		goto err_fe_sat;

	dev->dmxdev.filternum = 256;
	dev->dmxdev.demux = &dev->demux.dmx;
	dev->dmxdev.capabilities = 0;
	ret = dvb_dmxdev_init(&dev->dmxdev, &dev->dvb_adapter);
	if (ret)
		goto err_dmx;

	ret = dvb_net_init(&dev->dvb_adapter, &dev->dvbnet, &dev->demux.dmx);
	if (ret)
		goto err_dmxdev;

	ret = tbs5530_alloc_urbs(dev);
	if (ret)
		goto err_net;

	return 0;

err_net:
	dvb_net_release(&dev->dvbnet);
err_dmxdev:
	dvb_dmxdev_release(&dev->dmxdev);
err_dmx:
	dvb_dmx_release(&dev->demux);
err_fe_sat:
	dvb_unregister_frontend(dev->fe_sat);
err_sat_client:
	if (dev->i2c_client_sat) {
		module_put(dev->i2c_client_sat->dev.driver->owner);
		i2c_unregister_device(dev->i2c_client_sat);
		dev->i2c_client_sat = NULL;
	}
err_fe_ter:
	dvb_unregister_frontend(dev->fe_ter);
err_adapter:
	dvb_unregister_adapter(&dev->dvb_adapter);
	return ret;
}

static void tbs5530_dvb_exit(struct tbs5530_dev *dev)
{
	tbs5530_stop_streaming(dev);
	tbs5530_free_urbs(dev);
	dvb_net_release(&dev->dvbnet);
	dvb_dmxdev_release(&dev->dmxdev);
	dvb_dmx_release(&dev->demux);

	if (dev->fe_sat) {
		dvb_unregister_frontend(dev->fe_sat);
		dvb_frontend_detach(dev->fe_sat);
	}
	if (dev->fe_ter) {
		dvb_unregister_frontend(dev->fe_ter);
		dvb_frontend_detach(dev->fe_ter);
	}

	dvb_unregister_adapter(&dev->dvb_adapter);
}

/* -------- USB probe/disconnect -------- */

static const struct usb_device_id tbs5530_table[] = {
	{ USB_DEVICE(0x734c, 0x5530) },
	{ }
};
MODULE_DEVICE_TABLE(usb, tbs5530_table);

static int tbs5530_probe(struct usb_interface *intf,
			 const struct usb_device_id *id)
{
	struct tbs5530_dev *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->udev = usb_get_dev(interface_to_usbdev(intf));
	dev->intf = intf;
	mutex_init(&dev->io_lock);

	/* Load firmware */
	ret = tbs5530_load_firmware(dev);
	if (ret)
		goto err_free;

	/* Read MAC address (informational, non-fatal) */
	tbs5530_read_mac_address(dev);

	/* Register I2C adapter */
	dev->i2c.owner = THIS_MODULE;
	dev->i2c.algo = &tbs5530_i2c_algo;
	dev->i2c.dev.parent = &intf->dev;
	strscpy(dev->i2c.name, "tbs5530-i2c", sizeof(dev->i2c.name));
	i2c_set_adapdata(&dev->i2c, dev);

	ret = i2c_add_adapter(&dev->i2c);
	if (ret) {
		dev_err(&intf->dev, "i2c_add_adapter failed (%d)\n", ret);
		goto err_free;
	}

	/* Initialize DVB subsystem */
	ret = tbs5530_dvb_init(dev);
	if (ret)
		goto err_i2c;

	/* ATSC 3.0 ALP network interface (non-fatal) */
	dev->alp.fe = dev->fe_ter;
	dev->alp.start_streaming = tbs5530_alp_start;
	dev->alp.stop_streaming = tbs5530_alp_stop;
	dev->alp.priv = dev;
	atsc3_alp_register_netdev(&dev->alp);

	usb_set_intfdata(intf, dev);

	dev_info(&intf->dev, "TBS 5530 attached\n");
	return 0;

err_i2c:
	i2c_del_adapter(&dev->i2c);
err_free:
	usb_put_dev(dev->udev);
	kfree(dev);
	return ret;
}

static void tbs5530_disconnect(struct usb_interface *intf)
{
	struct tbs5530_dev *dev = usb_get_intfdata(intf);

	if (!dev)
		return;

	atsc3_alp_unregister_netdev(&dev->alp);

	tbs5530_dvb_exit(dev);

	/* M88RS6060 I2C client cleanup */
	if (dev->i2c_client_sat) {
		module_put(dev->i2c_client_sat->dev.driver->owner);
		i2c_unregister_device(dev->i2c_client_sat);
	}

	i2c_del_adapter(&dev->i2c);
	usb_put_dev(dev->udev);
	kfree(dev);

	dev_info(&intf->dev, "TBS 5530 detached\n");
}

static struct usb_driver tbs5530_driver = {
	.name = "tbs5530",
	.probe = tbs5530_probe,
	.disconnect = tbs5530_disconnect,
	.id_table = tbs5530_table,
};

module_param_array(adapter_nr, short, NULL, 0444);
MODULE_PARM_DESC(adapter_nr, "DVB adapter number(s)");

module_usb_driver(tbs5530_driver);

MODULE_FIRMWARE(TBS5530_FIRMWARE);
MODULE_AUTHOR("koreapyj");
MODULE_DESCRIPTION("TBS 5530 USB DVB driver");
MODULE_LICENSE("GPL");
