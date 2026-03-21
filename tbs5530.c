/* SPX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Davin <Davin@tbsdtv.com>
 * Copyright (c) 2026 Yoonji Park <koreapyj@dcmys.kr>
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/firmware.h>

#include "tbs5530.h"
#include "cxd2878.h"
#include "cxd2878_priv.h"
#include "alp.h"
#include "m88rs6060.h"

#define tbs5530_READ_MSG 0
#define tbs5530_WRITE_MSG 1

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

#define err(format, arg...)  printk(KERN_ERR     "tbs5530: " format "\n" , ## arg)
#define info(format, arg...) printk(KERN_INFO    "tbs5530: " format "\n" , ## arg)

static int tbs5530_op_rw(struct usb_device *dev, u8 request, u16 value,
				u16 index, u8 * data, u16 len, int flags)
{
	int ret;
	void *u8buf;

	unsigned int pipe = (flags == tbs5530_READ_MSG) ?
			usb_rcvctrlpipe(dev, 0) : usb_sndctrlpipe(dev, 0);
	u8 request_type = (flags == tbs5530_READ_MSG) ? USB_DIR_IN : USB_DIR_OUT;
	u8buf = kmalloc(len, GFP_KERNEL);
	if (!u8buf)
		return -ENOMEM;

	if (flags == tbs5530_WRITE_MSG)
		memcpy(u8buf, data, len);
	ret = usb_control_msg(dev, pipe, request, request_type | USB_TYPE_VENDOR,
				value, index , u8buf, len, 2000);

	if (flags == tbs5530_READ_MSG)
		memcpy(data, u8buf, len);
	kfree(u8buf);
	return ret;
}

/* I2C */
static int tbs5530_i2c_transfer(struct i2c_adapter *adap,
					struct i2c_msg msg[], int num)
{
	struct tbs5530_dev *dev = i2c_get_adapdata(adap);
	int i = 0;
	u8 buf6[50];
	u8 inbuf[50];

	if (!dev)
		return -ENODEV;
	if (mutex_lock_interruptible(&dev->i2c_mutex) < 0)
		return -EAGAIN;

	switch (num) {
	case 2:
		buf6[0]=msg[1].len;//lenth
		buf6[1]=msg[0].addr<<1;//demod addr
		//register
		buf6[2] = msg[0].buf[0];

		tbs5530_op_rw(dev->udev, 0x90, 0, 0,
					buf6, 3, tbs5530_WRITE_MSG);
		//msleep(5);
		tbs5530_op_rw(dev->udev, 0x91, 0, 0,
					inbuf, buf6[0], tbs5530_READ_MSG);
		memcpy(msg[1].buf, inbuf, msg[1].len);

		break;
	case 1:
		buf6[0] = msg[0].len+1;//lenth
		buf6[1] = msg[0].addr<<1;//addr
		for(i=0;i<msg[0].len;i++) {
			buf6[2+i] = msg[0].buf[i];//register
		}
		tbs5530_op_rw(dev->udev, 0x80, 0, 0,
			buf6, msg[0].len+2, tbs5530_WRITE_MSG);
		break;

	}

	mutex_unlock(&dev->i2c_mutex);
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


/* USB bulk streaming (EP 0x82) */

static void tbs5530_urb_complete(struct urb *urb)
{
	struct tbs5530_dev *dev = urb->context;

	if (!dev->streaming)
		return;

	switch (urb->status) {
	case 0:
		if (urb->actual_length > 0) {
			dvb_dmx_swfilter(&dev->demux, urb->transfer_buffer,
					 urb->actual_length);
			dev->urb_complete_ok++;
			dev->urb_bytes_total += urb->actual_length;
		} else {
			dev->urb_complete_empty++;
		}
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		return;
	default:
		dev->urb_complete_err++;
		goto resubmit;
	}

resubmit:
	usb_submit_urb(urb, GFP_ATOMIC);
}

static int tbs5530_start_streaming(struct tbs5530_dev *dev)
{
	u8 buf[2];
	int i, ret;

	if (dev->streaming)
		return 0;

	buf[0] = 10;
	buf[1] = dev->active_fe;
	tbs5530_op_rw(dev->udev, 0x8a, 0, 0, buf, 2, tbs5530_WRITE_MSG);

	dev->streaming = true;
	dev->urb_complete_ok = 0;
	dev->urb_complete_err = 0;
	dev->urb_complete_empty = 0;
	dev->urb_bytes_total = 0;

	dev_info(&dev->udev->dev, "TS streaming started, active_fe=%d\n",
		 dev->active_fe);

	for (i = 0; i < NUM_URBS; i++) {
		usb_fill_bulk_urb(dev->urbs[i], dev->udev,
				  usb_rcvbulkpipe(dev->udev, 0x82),
				  dev->urb_bufs[i], URB_BUF_SIZE,
				  tbs5530_urb_complete, dev);
		ret = usb_submit_urb(dev->urbs[i], GFP_KERNEL);
		if (ret) {
			while (--i >= 0)
				usb_kill_urb(dev->urbs[i]);
			dev->streaming = false;
			return ret;
		}
	}

	return 0;
}

static void tbs5530_stop_streaming(struct tbs5530_dev *dev)
{
	int i;

	if (!dev->streaming)
		return;

	dev->streaming = false;

	for (i = 0; i < NUM_URBS; i++)
		usb_kill_urb(dev->urbs[i]);

	dev_info(&dev->udev->dev,
		 "TS streaming stopped: ok=%lu err=%lu empty=%lu bytes=%llu\n",
		 dev->urb_complete_ok, dev->urb_complete_err,
		 dev->urb_complete_empty, dev->urb_bytes_total);
}

/* DVB demux feed callbacks */

static int tbs5530_start_feed(struct dvb_demux_feed *feed)
{
	struct tbs5530_dev *dev = feed->demux->priv;

	if (++dev->feed_count == 1)
		return tbs5530_start_streaming(dev);
	return 0;
}

static int tbs5530_stop_feed(struct dvb_demux_feed *feed)
{
	struct tbs5530_dev *dev = feed->demux->priv;

	if (--dev->feed_count == 0)
		tbs5530_stop_streaming(dev);
	return 0;
}

/* Frontend TS routing */

static int tbs5530_fe_ter_init(struct dvb_frontend *fe)
{
	struct tbs5530_dev *dev = fe->dvb->priv;

	dev->active_fe = 0;
	if (dev->fe_ter_ops_orig.init)
		return dev->fe_ter_ops_orig.init(fe);
	return 0;
}

static int tbs5530_fe_sat_init(struct dvb_frontend *fe)
{
	struct tbs5530_dev *dev = fe->dvb->priv;

	dev->active_fe = 1;
	if (dev->fe_sat_ops_orig.init)
		return dev->fe_sat_ops_orig.init(fe);
	return 0;
}

static struct cxd2878_config tbs5530_cfg = {
		.addr_slvt = 0x64,
		.xtal      = SONY_DEMOD_XTAL_24000KHz,
		.tuner_addr = 0x60,
		.tuner_xtal = SONY_ASCOT3_XTAL_24000KHz,
		.ts_mode	= 1,
		.ts_ser_data = 0,
		.ts_clk = 1,
		.ts_clk_mask= 1,
		.ts_valid = 0,
		.atscCoreDisable = 0,
		.atsc3_output = SONY_DEMOD_OUTPUT_ATSC3_ALP_DIV_TS,
		.lock_flag = 1,
		.write_properties = NULL,
		.read_properties = NULL,
	};

/* -------- ALP TS-level sysfs stats -------- */

struct tbs5530_alp_ts_sysfs {
	struct kobject kobj;
	struct cxd2878_dev *cxd;
};

#define to_tbs_alp_ts(obj) container_of(obj, struct tbs5530_alp_ts_sysfs, kobj)

struct tbs5530_alp_ts_attr {
	struct attribute attr;
	ssize_t (*show)(struct cxd2878_dev *dev, char *buf);
};

#define TBS_ALP_TS_ATTR(_name) \
static ssize_t tbs_alp_ts_##_name##_show(struct cxd2878_dev *dev, char *buf) \
{ \
	return sysfs_emit(buf, "%llu\n", dev->alp_ts_stats._name); \
} \
static struct tbs5530_alp_ts_attr tbs_alp_ts_attr_##_name = \
	{ .attr = { .name = #_name, .mode = 0444 }, \
	  .show = tbs_alp_ts_##_name##_show }

TBS_ALP_TS_ATTR(ts_reassembled);
TBS_ALP_TS_ATTR(ts_tei);
TBS_ALP_TS_ATTR(ts_sync_miss);
TBS_ALP_TS_ATTR(ts_null_skip);
TBS_ALP_TS_ATTR(ts_overflow);
TBS_ALP_TS_ATTR(frame_err_pusi);
TBS_ALP_TS_ATTR(raw_bytes_in);

static struct attribute *tbs_alp_ts_attrs[] = {
	&tbs_alp_ts_attr_ts_reassembled.attr,
	&tbs_alp_ts_attr_ts_tei.attr,
	&tbs_alp_ts_attr_ts_sync_miss.attr,
	&tbs_alp_ts_attr_ts_null_skip.attr,
	&tbs_alp_ts_attr_ts_overflow.attr,
	&tbs_alp_ts_attr_frame_err_pusi.attr,
	&tbs_alp_ts_attr_raw_bytes_in.attr,
	NULL,
};
ATTRIBUTE_GROUPS(tbs_alp_ts);

static ssize_t tbs_alp_ts_sysfs_show(struct kobject *kobj,
				     struct attribute *attr, char *buf)
{
	struct tbs5530_alp_ts_sysfs *s = to_tbs_alp_ts(kobj);
	struct tbs5530_alp_ts_attr *a =
		container_of(attr, struct tbs5530_alp_ts_attr, attr);

	return a->show(s->cxd, buf);
}

static const struct sysfs_ops tbs_alp_ts_sysfs_ops = {
	.show = tbs_alp_ts_sysfs_show,
};

static void tbs_alp_ts_sysfs_release(struct kobject *kobj)
{
	kfree(to_tbs_alp_ts(kobj));
}

static const struct kobj_type tbs_alp_ts_ktype = {
	.sysfs_ops	= &tbs_alp_ts_sysfs_ops,
	.release	= tbs_alp_ts_sysfs_release,
	.default_groups	= tbs_alp_ts_groups,
};

/* -------- ALP net device integration -------- */

static int tbs5530_alp_ts_cb(const u8 *buf1, size_t len1,
			     const u8 *buf2, size_t len2,
			     struct dmx_ts_feed *feed, u32 *buffer_flags)
{
	struct tbs5530_dev *dev = feed->priv;
	struct cxd2878_dev *cxd;
	const u8 *buf;
	size_t len;
	int i;

	if (!dev->fe_ter || !dev->alp)
		return 0;
	cxd = dev->fe_ter->demodulator_priv;

	for (i = 0; i < 2; i++) {
		buf = (i == 0) ? buf1 : buf2;
		len = (i == 0) ? len1 : len2;
		if (!buf || !len)
			continue;
		cxd2878_alp_feed_raw(cxd, dev->alp, buf, len);
	}
	return 0;
}

static int tbs5530_alp_open(void *priv)
{
	struct tbs5530_dev *dev = priv;
	struct dmx_ts_feed *feed;
	int ret;

	ret = dev->demux.dmx.allocate_ts_feed(&dev->demux.dmx, &feed,
					       tbs5530_alp_ts_cb);
	if (ret)
		return ret;

	feed->priv = dev;

	ret = feed->set(feed, 0x2000, TS_PACKET, DMX_PES_OTHER,
			ktime_set(0, 0));
	if (ret) {
		dev->demux.dmx.release_ts_feed(&dev->demux.dmx, feed);
		return ret;
	}

	ret = feed->start_filtering(feed);
	if (ret) {
		dev->demux.dmx.release_ts_feed(&dev->demux.dmx, feed);
		return ret;
	}

	dev->alp_feed = feed;

	if (dev->fe_ter) {
		struct cxd2878_dev *cxd = dev->fe_ter->demodulator_priv;

		cxd->alp_buf_len = 0;
		cxd->alp_active = false;
		memset(&cxd->alp_ts_stats, 0, sizeof(cxd->alp_ts_stats));
	}

	return 0;
}

static void tbs5530_alp_stop(void *priv)
{
	struct tbs5530_dev *dev = priv;

	if (dev->alp_feed) {
		dev->alp_feed->stop_filtering(dev->alp_feed);
		dev->demux.dmx.release_ts_feed(&dev->demux.dmx,
					       dev->alp_feed);
		dev->alp_feed = NULL;
	}

	if (dev->fe_ter) {
		struct cxd2878_dev *cxd = dev->fe_ter->demodulator_priv;

		cxd->alp_buf_len = 0;
		cxd->alp_active = false;
	}
}

static const struct alp_ops tbs5530_alp_ops = {
	.open = tbs5530_alp_open,
	.stop = tbs5530_alp_stop,
};

/* DVB stack setup */

static int tbs5530_dvb_init(struct tbs5530_dev *dev)
{
	struct dvb_adapter *adap = &dev->dvb_adap;
	int ret;

	ret = dvb_register_adapter(adap, "TurboSight TBS 5530",
				   THIS_MODULE, &dev->udev->dev,
				   adapter_nr);
	if (ret < 0)
		return ret;

	adap->priv = dev;

	/* attach CXD2878 frontend */
	dev->fe_ter = dvb_attach(cxd2878_attach, &tbs5530_cfg,
				 &dev->i2c_adap);
	if (!dev->fe_ter) {
		ret = -ENODEV;
		goto err_adapter;
	}

	strscpy(dev->fe_ter->ops.info.name,
		"TurboSight TBS 5530 DVB-T/T2/C/C2,ISDB-T/C,ATSC,J83B,ATSC3",
		sizeof(dev->fe_ter->ops.info.name));

	dev->fe_ter_ops_orig = dev->fe_ter->ops;
	dev->fe_ter->ops.init = tbs5530_fe_ter_init;

	ret = dvb_register_frontend(adap, dev->fe_ter);
	if (ret)
		goto err_fe_ter;

	/* attach M88RS6060 frontend */
	{
		struct m88rs6060_cfg m88rs6060_config;
		struct i2c_board_info info;
		u8 buf[20];

		memset(&m88rs6060_config,0,sizeof(m88rs6060_config));
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
		m88rs6060_config.write_properties  = NULL;

		memset(&info, 0, sizeof(struct i2c_board_info));
		strscpy(info.type, "m88rs6060", I2C_NAME_SIZE);
		info.addr = 0x69;
		info.platform_data = &m88rs6060_config;
		request_module(info.type);
		dev->i2c_client_sat = i2c_new_client_device(&dev->i2c_adap,&info);
		if(!i2c_client_has_driver(dev->i2c_client_sat)) {
			ret = -ENODEV;
			goto err_fe_ter_unreg;
		}
		if (!try_module_get(dev->i2c_client_sat->dev.driver->owner)) {
			i2c_unregister_device(dev->i2c_client_sat);
			dev->i2c_client_sat = NULL;
			ret = -ENODEV;
			goto err_fe_ter_unreg;
		}

		buf[0] = 0;
		buf[1] = 0;
		tbs5530_op_rw(dev->udev, 0xb7, 0, 0,
				buf, 2, tbs5530_WRITE_MSG);
		buf[0] = 8;
		buf[1] = 1;
		tbs5530_op_rw(dev->udev, 0x8a, 0, 0,
				buf, 2, tbs5530_WRITE_MSG);
		msleep(10);

		strscpy(dev->fe_sat->ops.info.name,
			"TurboSight TBS 5530 DVB-S/S2/S2X",
			sizeof(dev->fe_sat->ops.info.name));

		dev->fe_sat_ops_orig = dev->fe_sat->ops;
		dev->fe_sat->ops.init = tbs5530_fe_sat_init;

		ret = dvb_register_frontend(adap, dev->fe_sat);
		if (ret)
			goto err_i2c_client;
	}

	/* DVB demux */
	dev->demux.dmx.capabilities = DMX_TS_FILTERING |
				      DMX_SECTION_FILTERING;
	dev->demux.priv = dev;
	dev->demux.filternum = 256;
	dev->demux.feednum = 256;
	dev->demux.start_feed = tbs5530_start_feed;
	dev->demux.stop_feed = tbs5530_stop_feed;

	ret = dvb_dmx_init(&dev->demux);
	if (ret)
		goto err_fe_sat_unreg;

	dev->dmxdev.filternum = 256;
	dev->dmxdev.demux = &dev->demux.dmx;
	dev->dmxdev.capabilities = 0;

	ret = dvb_dmxdev_init(&dev->dmxdev, adap);
	if (ret)
		goto err_dmx;

	ret = dvb_net_init(adap, &dev->dvb_net, &dev->demux.dmx);
	if (ret)
		goto err_dmxdev;

	/* ALP virtual network adapter for ATSC 3.0 */
	if (dev->fe_ter) {
		struct cxd2878_dev *cxd = dev->fe_ter->demodulator_priv;

		dev->alp = alp_attach(&dev->udev->dev,
				      &tbs5530_alp_ops, dev);
		if (IS_ERR(dev->alp)) {
			dev_warn(&dev->udev->dev,
				 "ALP net attach failed: %ld\n",
				 PTR_ERR(dev->alp));
			dev->alp = NULL;
		} else {
			struct tbs5530_alp_ts_sysfs *s;

			cxd->alpdev = alp_get_netdev(dev->alp);
			s = kzalloc(sizeof(*s), GFP_KERNEL);
			if (s) {
				s->cxd = cxd;
				if (kobject_init_and_add(&s->kobj,
							&tbs_alp_ts_ktype,
							&dev->udev->dev.kobj,
							"alp_ts_stats"))
					kobject_put(&s->kobj);
				else
					dev->alp_ts_kobj = &s->kobj;
			}
		}
	}

	return 0;

err_dmxdev:
	dvb_dmxdev_release(&dev->dmxdev);
err_dmx:
	dvb_dmx_release(&dev->demux);
err_fe_sat_unreg:
	dvb_unregister_frontend(dev->fe_sat);
err_i2c_client:
	module_put(dev->i2c_client_sat->dev.driver->owner);
	i2c_unregister_device(dev->i2c_client_sat);
	dev->i2c_client_sat = NULL;
err_fe_ter_unreg:
	dvb_unregister_frontend(dev->fe_ter);
err_fe_ter:
	dvb_frontend_detach(dev->fe_ter);
	dev->fe_ter = NULL;
err_adapter:
	dvb_unregister_adapter(adap);
	return ret;
}

static void tbs5530_dvb_exit(struct tbs5530_dev *dev)
{
	if (dev->alp) {
		if (dev->alp_ts_kobj) {
			kobject_put(dev->alp_ts_kobj);
			dev->alp_ts_kobj = NULL;
		}
		tbs5530_alp_stop(dev);
		alp_detach(dev->alp);
		dev->alp = NULL;
	}

	dvb_net_release(&dev->dvb_net);
	dvb_dmxdev_release(&dev->dmxdev);
	dvb_dmx_release(&dev->demux);

	if (dev->fe_sat) {
		dvb_unregister_frontend(dev->fe_sat);
		dvb_frontend_detach(dev->fe_sat);
	}

	if (dev->i2c_client_sat) {
		module_put(dev->i2c_client_sat->dev.driver->owner);
		i2c_unregister_device(dev->i2c_client_sat);
	}

	if (dev->fe_ter) {
		dvb_unregister_frontend(dev->fe_ter);
		dvb_frontend_detach(dev->fe_ter);
	}

	dvb_unregister_adapter(&dev->dvb_adap);
}

/* USB probe / disconnect */

static struct usb_device_id tbs5530_table[] = {
	{USB_DEVICE(0x734c, 0x5530)},
	{ }
};

MODULE_DEVICE_TABLE(usb, tbs5530_table);

/* Firmware */

static int tbs5530_load_firmware(struct tbs5530_dev *dev)
{
	u8 *b, *p;
	int ret = 0, i;
	u8 reset;
	const struct firmware *fw;

	ret = request_firmware(&fw, "dvb-usb-id5530.fw", &dev->udev->dev);
	if (ret != 0) {
		err("did not find the firmware file. (dvb-usb-id5530.fw) "
		"Please see linux/Documentation/dvb/ for more details "
		"on firmware-problems.");
		return ret;
	}
	info("start downloading tbs5530 firmware");
	p = kmalloc(fw->size, GFP_KERNEL);
	reset = 1;
	/*stop the CPU*/
	tbs5530_op_rw(dev->udev, 0xa0, 0x7f92, 0, &reset, 1, tbs5530_WRITE_MSG);
	tbs5530_op_rw(dev->udev, 0xa0, 0xe600, 0, &reset, 1, tbs5530_WRITE_MSG);

	if (p != NULL) {
		memcpy(p, fw->data, fw->size);
		for (i = 0; i < fw->size; i += 0x40) {
			b = (u8 *) p + i;
			if (tbs5530_op_rw(dev->udev, 0xa0, i, 0, b , 0x40,
					tbs5530_WRITE_MSG) != 0x40) {
				err("error while transferring firmware");
				ret = -EINVAL;
				break;
			}
		}
		/* restart the CPU */
		reset = 0;
		if (ret || tbs5530_op_rw(dev->udev, 0xa0, 0x7f92, 0, &reset, 1,
					tbs5530_WRITE_MSG) != 1) {
			err("could not restart the USB controller CPU.");
			ret = -EINVAL;
		}
		if (ret || tbs5530_op_rw(dev->udev, 0xa0, 0xe600, 0, &reset, 1,
					tbs5530_WRITE_MSG) != 1) {
			err("could not restart the USB controller CPU.");
			ret = -EINVAL;
		}

		msleep(100);
		kfree(p);
	}
	release_firmware(fw);
	return ret;
}

static int tbs5530_probe(struct usb_interface *intf,
		const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct tbs5530_dev *dev;
	int ret, i;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->udev = usb_get_dev(udev);
	dev->intf = intf;
	mutex_init(&dev->i2c_mutex);
	usb_set_intfdata(intf, dev);

	ret = tbs5530_load_firmware(dev);
	if (ret)
		goto err_free;

	/* I2C adapter */
	dev->i2c_adap.owner = THIS_MODULE;
	dev->i2c_adap.algo = &tbs5530_i2c_algo;
	dev->i2c_adap.dev.parent = &intf->dev;
	strscpy(dev->i2c_adap.name, "TBS5530 I2C",
		sizeof(dev->i2c_adap.name));
	i2c_set_adapdata(&dev->i2c_adap, dev);

	ret = i2c_add_adapter(&dev->i2c_adap);
	if (ret)
		goto err_free;

	/* URB ring buffer */
	for (i = 0; i < NUM_URBS; i++) {
		dev->urbs[i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!dev->urbs[i]) {
			ret = -ENOMEM;
			goto err_urbs;
		}
		dev->urb_bufs[i] = kmalloc(URB_BUF_SIZE, GFP_KERNEL);
		if (!dev->urb_bufs[i]) {
			ret = -ENOMEM;
			goto err_urbs;
		}
	}

	/* DVB stack + frontends */
	ret = tbs5530_dvb_init(dev);
	if (ret)
		goto err_urbs;

	info("TurboSight TBS 5530 attached");
	return 0;

err_urbs:
	for (i = 0; i < NUM_URBS; i++) {
		kfree(dev->urb_bufs[i]);
		usb_free_urb(dev->urbs[i]);
	}
err_i2c:
	i2c_del_adapter(&dev->i2c_adap);
err_free:
	usb_put_dev(dev->udev);
	usb_set_intfdata(intf, NULL);
	kfree(dev);
	return ret;
}

static void tbs5530_disconnect(struct usb_interface *intf)
{
	struct tbs5530_dev *dev = usb_get_intfdata(intf);

	if (!dev)
		return;

	int i;

	tbs5530_stop_streaming(dev);
	tbs5530_dvb_exit(dev);

	for (i = 0; i < NUM_URBS; i++) {
		kfree(dev->urb_bufs[i]);
		usb_free_urb(dev->urbs[i]);
	}

	i2c_del_adapter(&dev->i2c_adap);

	usb_set_intfdata(intf, NULL);
	usb_put_dev(dev->udev);
	kfree(dev);
}

static struct usb_driver tbs5530_driver = {
	.name = "tbs5530",
	.probe = tbs5530_probe,
	.disconnect = tbs5530_disconnect,
	.id_table = tbs5530_table,
};

static int __init tbs5530_module_init(void)
{
	int ret =  usb_register(&tbs5530_driver);
	if (ret)
		err("usb_register failed. Error number %d", ret);

	return ret;
}

static void __exit tbs5530_module_exit(void)
{
	usb_deregister(&tbs5530_driver);
}

module_init(tbs5530_module_init);
module_exit(tbs5530_module_exit);

MODULE_AUTHOR("Yoonji Park <koreapyj@dcmys.kr>");
MODULE_DESCRIPTION("TurboSight TBS 5530 driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");
