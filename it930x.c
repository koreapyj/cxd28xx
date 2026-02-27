/*
 * ITE IT930x USB bridge driver for CXD2878 demod + tuner
 *
 * Copyright (c) 2026 Yoonji Park <koreapyj@dcmys.kr>
 */

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/kref.h>
#include <linux/completion.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <media/dvb_frontend.h>
#include <media/dmxdev.h>
#include <media/dvb_demux.h>
#include <media/dvb_net.h>

#include "cxd2878.h"
#include "atsc3_alp.h"

/* -------- IT930x bridge constants -------- */

/* Command IDs */
#define IT930X_CMD_REG_READ			0x0000
#define IT930X_CMD_REG_WRITE		0x0001
#define IT930X_CMD_QUERYINFO		0x0022
#define IT930X_CMD_BOOT				0x0023
#define IT930X_CMD_FW_SCATTER_WRITE	0x0029
#define IT930X_CMD_GENERIC_I2C_RD	0x002a
#define IT930X_CMD_GENERIC_I2C_WR	0x002b

/* USB endpoints */
#define IT930X_EP_CMD_OUT		0x02
#define IT930X_EP_CMD_IN		0x81
#define IT930X_EP_TS			0x84

/* Protocol limits */
#define IT930X_CMD_MAX_PKT		512
#define IT930X_CMD_MAX_PAYLOAD	250
#define IT930X_CMD_HDR_LEN		4
#define IT930X_CMD_CSUM_LEN		2
#define IT930X_USB_TIMEOUT_MS	1000
#define IT930X_CMD_WAIT_MS		2000
#define IT930X_RX_IO_TIMEOUT_MS	200

/* I2C limits */
#define IT930X_I2C_SHORT_MAX	40
#define IT930X_I2C_STAGE_CHUNK	40
#define IT930X_I2C_BUS_IDX		3	/* IT930x bus for CXD2878 */

/* TS streaming */
#define IT930X_URB_COUNT		4
#define IT930X_URB_BUF_SIZE		(188 * 816)

/* Firmware */
#define IT9306_FIRMWARE_FILE	"dvb-usb-it9306-01.fw"

/* Bridge registers */
#define IT930X_REG_CLK_DISABLE		0x4976
#define IT930X_REG_PLL_RESET		0x4bfb
#define IT930X_REG_TS_CLK_CFG		0x4978
#define IT930X_REG_EXT_CLK_RESET	0x4977
#define IT930X_REG_I2C_BUS0_SPEED	0xf6a7
#define IT930X_REG_I2C_BUS1_SPEED	0xf103
#define IT930X_REG_I2C_TRIG			0x4900
#define IT930X_REG_I2C_READ_BUF		0xf000
#define IT930X_REG_I2C_WRITE_BUF	0xf001
#define IT930X_REG_I2C_BUS_CTRL		0xf424
#define IT930X_REG_TS_FAIL_IGNORE	0xda5a
#define IT930X_REG_I2C_BUS0_EN		0xd8d4
#define IT930X_REG_I2C_BUS1_EN		0xd8d5
#define IT930X_REG_I2C_RESET		0xd8d3
#define IT930X_REG_TS_SYNC_BYPASS	0xda1a
#define IT930X_REG_TS_OUT_EN		0xd833
#define IT930X_REG_TS_OUT_CFG0		0xd830
#define IT930X_REG_TS_OUT_CFG1		0xd831
#define IT930X_REG_TS_OUT_CFG2		0xd832
/* GPIO registers */
#define IT930X_REG_GPIO2_VALUE		0xd8b7
#define IT930X_REG_GPIO2_EN			0xd8b8
#define IT930X_REG_GPIO2_DIRECTION	0xd8b9
#define IT930X_REG_GPIO14_VALUE		0xd8e3
#define IT930X_REG_GPIO14_EN		0xd8e4
#define IT930X_REG_GPIO14_DIRECTION	0xd8e5
/* Hardware PID filter registers (TS port 0) */
#define IT930X_REG_PID_DAT_H		0xda17
#define IT930X_REG_PID_DAT_L		0xda16
#define IT930X_REG_PID_INDEX_EN		0xda14
#define IT930X_REG_PID_INDEX		0xda15
#define IT930X_REG_MAP_INDEX		0xda11
#define IT930X_REG_REMAP_MODE		0xda13
#define IT930X_REG_AGGRE_MODE		0xda73
#define IT930X_REG_PID_OFFSET_L		0xda81
#define IT930X_REG_PID_OFFSET_H		0xda82

#define IT930X_PID_FILTER_MAX		64

/* -------- Data structures -------- */

struct it930x_cmd_req {
	struct completion done;
	struct kref refcount;
	spinlock_t state_lock;
	bool completed;
	u8 seq;
	u8 tx_payload_len;
	u8 rx_payload_len;
	u16 cmd;
	u8 tx_payload[IT930X_CMD_MAX_PAYLOAD];
	u8 rx_payload[IT930X_CMD_MAX_PAYLOAD];
	u8 result;
	int status;
};

struct it930x_dev {
	struct usb_device *udev;
	struct usb_interface *intf;

	/* Transport layer */
	struct mutex io_lock;
	struct mutex tx_mutex;
	spinlock_t pending_lock;
	struct it930x_cmd_req *pending[U8_MAX + 1];
	struct task_struct *rx_thread;
	u8 *tx_bulk_buf;
	u8 *rx_bulk_buf;
	bool stopping;
	u8 seq;

	u32 fw_version;

	/* I2C */
	struct i2c_adapter i2c;

	/* DVB */
	const char *devname;
	struct dvb_adapter dvb_adapter;
	struct dvb_frontend *fe;
	struct dvb_demux demux;
	struct dmxdev dmxdev;
	struct dvb_net dvbnet;
	int feeding;
	bool hw_pid_active;	/* HW PID filter engine on (aggre_mode=3) */

	/* TS streaming */
	struct urb *urbs[IT930X_URB_COUNT];
	u8 *urb_bufs[IT930X_URB_COUNT];
	dma_addr_t urb_dma[IT930X_URB_COUNT];
	bool streaming;

	/* Streaming stats (updated in URB completion, printed on stop) */
	unsigned long urb_complete_ok;
	unsigned long urb_complete_err;
	unsigned long urb_complete_empty;
	u64 urb_bytes_total;

	/* ATSC 3.0 network interface */
	struct atsc3_alp alp;
};

static short adapter_nr[] = { [0 ... 3] = -1 };

/* -------- Checksum -------- */

static u16 it930x_checksum(const u8 *buf, size_t len)
{
	u32 sum = 0;
	size_t i;

	for (i = 0; i + 1 < len; i += 2)
		sum += ((u16)buf[i] << 8) | buf[i + 1];

	if (len & 1)
		sum += (u16)buf[len - 1] << 8;

	return (u16)~sum;
}

/* -------- Command request lifecycle -------- */

static void it930x_cmd_req_release(struct kref *ref)
{
	kfree(container_of(ref, struct it930x_cmd_req, refcount));
}

static inline void it930x_cmd_req_get(struct it930x_cmd_req *req)
{
	kref_get(&req->refcount);
}

static inline void it930x_cmd_req_put(struct it930x_cmd_req *req)
{
	kref_put(&req->refcount, it930x_cmd_req_release);
}

static void it930x_cmd_req_complete(struct it930x_cmd_req *req, int status,
				    u8 result, const u8 *payload,
				    u8 payload_len)
{
	unsigned long flags;

	spin_lock_irqsave(&req->state_lock, flags);
	if (req->completed) {
		spin_unlock_irqrestore(&req->state_lock, flags);
		return;
	}

	req->status = status;
	req->result = result;
	if (!status && req->rx_payload_len) {
		if (payload_len < req->rx_payload_len) {
			req->status = -EPROTO;
		} else if (payload) {
			memcpy(req->rx_payload, payload, req->rx_payload_len);
		}
	}
	req->completed = true;
	complete(&req->done);
	spin_unlock_irqrestore(&req->state_lock, flags);
}

/* -------- USB bulk transfer -------- */

static int it930x_bulk_xfer(struct it930x_dev *dev, u8 ep, void *buf,
			    int len, int *actual, bool in, int timeout_ms)
{
	int pipe = in ? usb_rcvbulkpipe(dev->udev, ep) :
		       usb_sndbulkpipe(dev->udev, ep);

	return usb_bulk_msg(dev->udev, pipe, buf, len, actual, timeout_ms);
}

/* -------- Transport -------- */

static int it930x_encode_tx_frame(struct it930x_cmd_req *req, u8 *frame,
				  int *frame_len)
{
	u16 csum;
	int len;

	len = IT930X_CMD_HDR_LEN + req->tx_payload_len + IT930X_CMD_CSUM_LEN;
	if (len > IT930X_CMD_MAX_PKT)
		return -E2BIG;

	frame[1] = req->cmd >> 8;
	frame[2] = req->cmd & 0xff;
	frame[3] = req->seq;
	if (req->tx_payload_len)
		memcpy(&frame[4], req->tx_payload, req->tx_payload_len);

	csum = it930x_checksum(&frame[1], 3 + req->tx_payload_len);
	frame[4 + req->tx_payload_len] = csum >> 8;
	frame[5 + req->tx_payload_len] = csum & 0xff;
	frame[0] = len - 1;
	*frame_len = len;

	return 0;
}

static int it930x_rx_thread(void *data)
{
	struct it930x_dev *dev = data;
	u8 *rx = dev->rx_bulk_buf;

	while (!kthread_should_stop()) {
		struct it930x_cmd_req *req;
		unsigned long flags;
		int rx_len = 0;
		u16 rx_csum, csum;
		u8 seq, result;
		int ret;

		if (dev->stopping)
			break;

		ret = it930x_bulk_xfer(dev, IT930X_EP_CMD_IN, rx,
				       IT930X_CMD_MAX_PKT, &rx_len, true,
				       IT930X_RX_IO_TIMEOUT_MS);
		if (ret)
			continue;
		if (rx_len < 5 || rx_len > IT930X_CMD_MAX_PKT)
			continue;

		rx_csum = ((u16)rx[rx_len - 2] << 8) | rx[rx_len - 1];
		csum = it930x_checksum(&rx[1], rx_len - 3);
		if (csum != rx_csum)
			continue;

		seq = rx[1];
		result = rx[2];

		spin_lock_irqsave(&dev->pending_lock, flags);
		req = dev->pending[seq];
		if (req)
			dev->pending[seq] = NULL;
		spin_unlock_irqrestore(&dev->pending_lock, flags);

		if (!req)
			continue;

		if (result) {
			dev_dbg(&dev->intf->dev,
				"cmd seq=%u result=%u (error)\n", seq, result);
			it930x_cmd_req_complete(req, -EIO, result, NULL, 0);
		} else {
			it930x_cmd_req_complete(req, 0, result, &rx[3],
						rx_len - 5);
		}
		it930x_cmd_req_put(req);
	}

	return 0;
}

/* -------- Transport start/stop -------- */

static void it930x_transport_fail_all(struct it930x_dev *dev, int status)
{
	struct it930x_cmd_req *req;
	unsigned long flags;
	int i;

	for (i = 0; i <= U8_MAX; i++) {
		spin_lock_irqsave(&dev->pending_lock, flags);
		req = dev->pending[i];
		if (req)
			dev->pending[i] = NULL;
		spin_unlock_irqrestore(&dev->pending_lock, flags);
		if (!req)
			continue;

		it930x_cmd_req_complete(req, status, 0, NULL, 0);
		it930x_cmd_req_put(req);
	}
}

static int it930x_transport_start(struct it930x_dev *dev)
{
	int ret;

	dev->stopping = false;
	mutex_init(&dev->tx_mutex);
	spin_lock_init(&dev->pending_lock);
	memset(dev->pending, 0, sizeof(dev->pending));
	dev->seq = 0;

	dev->tx_bulk_buf = kmalloc(IT930X_CMD_MAX_PKT, GFP_KERNEL);
	if (!dev->tx_bulk_buf)
		return -ENOMEM;

	dev->rx_bulk_buf = kmalloc(IT930X_CMD_MAX_PKT, GFP_KERNEL);
	if (!dev->rx_bulk_buf) {
		ret = -ENOMEM;
		goto err_free;
	}

	dev->rx_thread = kthread_run(it930x_rx_thread, dev, "it930x_rx");
	if (IS_ERR(dev->rx_thread)) {
		ret = PTR_ERR(dev->rx_thread);
		dev->rx_thread = NULL;
		goto err_free;
	}
	get_task_struct(dev->rx_thread);

	return 0;

err_free:
	kfree(dev->tx_bulk_buf);
	kfree(dev->rx_bulk_buf);
	dev->tx_bulk_buf = NULL;
	dev->rx_bulk_buf = NULL;
	return ret;
}

static void it930x_transport_stop(struct it930x_dev *dev)
{
	struct task_struct *rx;

	dev->stopping = true;

	rx = xchg(&dev->rx_thread, NULL);
	if (!IS_ERR_OR_NULL(rx)) {
		kthread_stop(rx);
		put_task_struct(rx);
	}

	it930x_transport_fail_all(dev, -ENODEV);
	kfree(dev->tx_bulk_buf);
	kfree(dev->rx_bulk_buf);
	dev->tx_bulk_buf = NULL;
	dev->rx_bulk_buf = NULL;
}

/* -------- Command submission -------- */

static int it930x_cmd_submit_sync(struct it930x_dev *dev, u16 cmd,
				  const u8 *tx_payload, u8 tx_payload_len,
				  u8 *rx_payload, u8 rx_payload_len)
{
	struct it930x_cmd_req *req;
	unsigned long flags;
	unsigned long tout;
	int tx_len, actual;
	int ret;

	if (tx_payload_len > IT930X_CMD_MAX_PAYLOAD ||
	    rx_payload_len > IT930X_CMD_MAX_PAYLOAD)
		return -E2BIG;
	if (dev->stopping)
		return -ENODEV;

	req = kzalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	init_completion(&req->done);
	kref_init(&req->refcount);
	spin_lock_init(&req->state_lock);
	req->cmd = cmd;
	req->tx_payload_len = tx_payload_len;
	req->rx_payload_len = rx_payload_len;
	if (tx_payload_len && tx_payload)
		memcpy(req->tx_payload, tx_payload, tx_payload_len);

	/* Inline TX: assign seq, register pending, encode, send */
	mutex_lock(&dev->tx_mutex);

	req->seq = dev->seq++;

	spin_lock_irqsave(&dev->pending_lock, flags);
	if (dev->pending[req->seq]) {
		spin_unlock_irqrestore(&dev->pending_lock, flags);
		mutex_unlock(&dev->tx_mutex);
		kfree(req);
		return -EBUSY;
	}
	it930x_cmd_req_get(req);
	dev->pending[req->seq] = req;
	spin_unlock_irqrestore(&dev->pending_lock, flags);

	ret = it930x_encode_tx_frame(req, dev->tx_bulk_buf, &tx_len);
	if (!ret)
		ret = it930x_bulk_xfer(dev, IT930X_EP_CMD_OUT, dev->tx_bulk_buf,
				       tx_len, &actual, false,
				       IT930X_USB_TIMEOUT_MS);

	mutex_unlock(&dev->tx_mutex);

	if (ret)
		goto err_pending;

	/* Wait for rx_thread to deliver the response */
	tout = wait_for_completion_timeout(&req->done,
					   msecs_to_jiffies(IT930X_CMD_WAIT_MS));
	if (!tout) {
		spin_lock_irqsave(&dev->pending_lock, flags);
		if (dev->pending[req->seq] == req) {
			dev->pending[req->seq] = NULL;
			it930x_cmd_req_put(req);
		}
		spin_unlock_irqrestore(&dev->pending_lock, flags);
		ret = -ETIMEDOUT;
		goto out;
	}

	ret = req->status;
	if (!ret && rx_payload_len && rx_payload)
		memcpy(rx_payload, req->rx_payload, rx_payload_len);
	if (ret)
		dev_dbg(&dev->intf->dev,
			"cmd 0x%04x failed: status=%d result=%u\n",
			cmd, ret, req->result);
	goto out;

err_pending:
	spin_lock_irqsave(&dev->pending_lock, flags);
	if (dev->pending[req->seq] == req) {
		dev->pending[req->seq] = NULL;
		it930x_cmd_req_put(req);
	}
	spin_unlock_irqrestore(&dev->pending_lock, flags);

out:
	it930x_cmd_req_put(req);
	return ret;
}

/* -------- Bridge register access -------- */

static int it930x_write_regs(struct it930x_dev *dev, u32 reg, const u8 *buf,
			     u8 len)
{
	u8 payload[6 + 250];

	if (len > 250)
		return -E2BIG;

	payload[0] = len;
	payload[1] = 2; /* processor (OFDM) */
	payload[2] = (reg >> 24) & 0xff;
	payload[3] = (reg >> 16) & 0xff;
	payload[4] = (reg >> 8) & 0xff;
	payload[5] = reg & 0xff;
	if (len)
		memcpy(&payload[6], buf, len);

	return it930x_cmd_submit_sync(dev, IT930X_CMD_REG_WRITE,
				      payload, 6 + len, NULL, 0);
}

static int it930x_read_regs(struct it930x_dev *dev, u32 reg, u8 *buf, u8 len)
{
	u8 payload[6];

	if (len > 250)
		return -E2BIG;

	payload[0] = len;
	payload[1] = 2; /* processor (OFDM) */
	payload[2] = (reg >> 24) & 0xff;
	payload[3] = (reg >> 16) & 0xff;
	payload[4] = (reg >> 8) & 0xff;
	payload[5] = reg & 0xff;

	return it930x_cmd_submit_sync(dev, IT930X_CMD_REG_READ,
				      payload, sizeof(payload), buf, len);
}

static int it930x_write_reg(struct it930x_dev *dev, u32 reg, u8 val)
{
	return it930x_write_regs(dev, reg, &val, 1);
}

static int it930x_read_reg(struct it930x_dev *dev, u32 reg, u8 *val)
{
	return it930x_read_regs(dev, reg, val, 1);
}

static int it930x_patch_reg_bit(struct it930x_dev *dev, u32 reg, u8 val,
				u8 mask)
{
	u8 cur, next;
	int ret;

	ret = it930x_read_reg(dev, reg, &cur);
	if (ret)
		return ret;

	next = (cur & ~mask) | (val & mask);
	if (next == cur)
		return 0;

	return it930x_write_reg(dev, reg, next);
}

/* -------- Firmware loading -------- */

static int it930x_read_firmware_version(struct it930x_dev *dev, u32 *ver)
{
	u8 tx = 0x01;
	u8 rx[4];
	int ret;

	ret = it930x_cmd_submit_sync(dev, IT930X_CMD_QUERYINFO,
				     &tx, 1, rx, sizeof(rx));
	if (ret)
		return ret;

	if (ver)
		*ver = ((u32)rx[0] << 24) | ((u32)rx[1] << 16) |
		       ((u32)rx[2] << 8) | rx[3];
	return 0;
}

static int it930x_load_firmware(struct it930x_dev *dev)
{
	const struct firmware *fw;
	size_t i = 0;
	int ret;

	ret = request_firmware(&fw, IT9306_FIRMWARE_FILE, &dev->intf->dev);
	if (ret) {
		dev_err(&dev->intf->dev,
			"failed to load firmware %s (%d)\n",
			IT9306_FIRMWARE_FILE, ret);
		return ret;
	}

	while (i < fw->size) {
		const u8 *p = &fw->data[i];
		size_t block_len = 0;
		size_t payload_len;
		u8 seg_cnt;
		unsigned int j;

		if (i + 7 > fw->size) {
			ret = -ECANCELED;
			dev_err(&dev->intf->dev,
				"truncated firmware block at %zu\n", i);
			goto out;
		}

		if (p[0] != 0x03) {
			ret = -ECANCELED;
			dev_err(&dev->intf->dev,
				"bad firmware block marker 0x%02x at %zu\n",
				p[0], i);
			goto out;
		}

		seg_cnt = p[3];
		payload_len = 4 + ((size_t)seg_cnt * 3);
		if (i + payload_len > fw->size) {
			ret = -ECANCELED;
			dev_err(&dev->intf->dev,
				"truncated firmware segment table at %zu\n", i);
			goto out;
		}

		for (j = 0; j < seg_cnt; j++)
			block_len += p[6 + (j * 3)];

		if (!block_len) {
			i += payload_len;
			continue;
		}

		payload_len += block_len;
		if (i + payload_len > fw->size) {
			ret = -ECANCELED;
			dev_err(&dev->intf->dev,
				"truncated firmware data at %zu\n", i);
			goto out;
		}
		if (payload_len > IT930X_CMD_MAX_PAYLOAD) {
			ret = -E2BIG;
			dev_err(&dev->intf->dev,
				"firmware block too large (%zu) at %zu\n",
				payload_len, i);
			goto out;
		}

		ret = it930x_cmd_submit_sync(dev, IT930X_CMD_FW_SCATTER_WRITE,
					     p, (u8)payload_len, NULL, 0);
		if (ret) {
			dev_err(&dev->intf->dev,
				"firmware scatter write failed at %zu (%d)\n",
				i, ret);
			goto out;
		}

		i += payload_len;
	}

	ret = it930x_cmd_submit_sync(dev, IT930X_CMD_BOOT, NULL, 0, NULL, 0);
	if (ret)
		dev_err(&dev->intf->dev, "firmware boot failed (%d)\n", ret);

out:
	release_firmware(fw);
	return ret;
}

/* -------- I2C relay -------- */

static int it930x_i2c_generic_write(struct it930x_dev *dev, u8 bus,
				    u8 slave, const u8 *buf, u8 len)
{
	u8 payload[3 + IT930X_I2C_SHORT_MAX];

	if (len > IT930X_I2C_SHORT_MAX)
		return -E2BIG;

	payload[0] = len;
	payload[1] = bus;
	payload[2] = slave;
	if (len)
		memcpy(&payload[3], buf, len);

	return it930x_cmd_submit_sync(dev, IT930X_CMD_GENERIC_I2C_WR,
				      payload, 3 + len, NULL, 0);
}

static int it930x_i2c_generic_read(struct it930x_dev *dev, u8 bus,
				   u8 slave, u8 *buf, u8 len)
{
	u8 payload[3];

	if (len > 250)
		return -E2BIG;

	payload[0] = len;
	payload[1] = bus;
	payload[2] = slave;

	return it930x_cmd_submit_sync(dev, IT930X_CMD_GENERIC_I2C_RD,
				      payload, sizeof(payload), buf, len);
}

static int it930x_i2c_write(struct it930x_dev *dev, u8 bus, u8 slave,
			    const u8 *buf, u8 len)
{
	u8 trig[4];
	u8 chunk;
	int ret;
	u32 off = 0;
	u32 remaining = len;

	if (!len)
		return 0;

	if (len <= IT930X_I2C_SHORT_MAX)
		return it930x_i2c_generic_write(dev, bus, slave, buf, len);

	if (len > U8_MAX)
		return -E2BIG;

	/* Stage long write into bridge buffer */
	while (remaining) {
		chunk = min_t(u32, remaining, IT930X_I2C_STAGE_CHUNK);
		ret = it930x_write_regs(dev, IT930X_REG_I2C_WRITE_BUF + off,
					&buf[off], chunk);
		if (ret)
			return ret;

		off += chunk;
		remaining -= chunk;
	}

	/* Trigger I2C write */
	trig[0] = 0xf4;
	trig[1] = bus;
	trig[2] = slave;
	trig[3] = len;

	return it930x_write_regs(dev, IT930X_REG_I2C_TRIG, trig, sizeof(trig));
}

static int it930x_i2c_read(struct it930x_dev *dev, u8 bus, u8 slave,
			   u8 *buf, u8 len)
{
	u8 trig[4];
	int ret;

	if (!len)
		return 0;

	if (len <= IT930X_I2C_SHORT_MAX)
		return it930x_i2c_generic_read(dev, bus, slave, buf, len);

	if (len > U8_MAX)
		return -E2BIG;

	/* Trigger I2C read */
	trig[0] = 0xf5;
	trig[1] = bus;
	trig[2] = slave;
	trig[3] = len;
	ret = it930x_write_regs(dev, IT930X_REG_I2C_TRIG, trig, sizeof(trig));
	if (ret)
		return ret;

	return it930x_read_regs(dev, IT930X_REG_I2C_READ_BUF, buf, len);
}

/* -------- I2C adapter -------- */

/*
 * I2C repeated-start via F424 bus control.
 *
 * The IT930x bridge uses register F424 to implement I2C repeated-start:
 *   F424 = 0x01: hold bus (suppress STOP after next I2C write)
 *   F424 = 0x00: release bus (normal STOP behavior)
 *
 * Vendor sequence for a register read:
 *   1. REG_WRITE F424 = 0x01   (hold bus)
 *   2. GENERIC_I2C_WR sub-addr (START-addr_W-subaddr, no STOP)
 *   3. REG_WRITE F424 = 0x00   (release bus)
 *   4. GENERIC_I2C_RD data     (REPEATED_START-addr_R-data-STOP)
 */
static int it930x_i2c_xfer_rstart(struct it930x_dev *dev,
				  struct i2c_msg *wr, struct i2c_msg *rd)
{
	u8 wr_slave = wr->addr << 1;
	u8 rd_slave = rd->addr << 1;
	int ret;

	/* Set bus hold — suppress STOP after the write */
	ret = it930x_write_reg(dev, IT930X_REG_I2C_BUS_CTRL, 0x01);
	if (ret)
		return ret;

	/* Sub-address write (will not STOP because bus is held) */
	ret = it930x_i2c_write(dev, IT930X_I2C_BUS_IDX,
			       wr_slave, wr->buf, wr->len);
	if (ret) {
		it930x_write_reg(dev, IT930X_REG_I2C_BUS_CTRL, 0x00);
		return ret;
	}

	/* Release bus hold — next operation gets normal STOP */
	ret = it930x_write_reg(dev, IT930X_REG_I2C_BUS_CTRL, 0x00);
	if (ret)
		return ret;

	/* Data read (REPEATED_START-addr_R-data-STOP) */
	return it930x_i2c_read(dev, IT930X_I2C_BUS_IDX,
			       rd_slave, rd->buf, rd->len);
}

static int it930x_i2c_xfer(struct i2c_adapter *adap,
			    struct i2c_msg *msgs, int num)
{
	struct it930x_dev *dev = i2c_get_adapdata(adap);
	int i, ret = 0;

	mutex_lock(&dev->io_lock);

	/*
	 * Detect write-then-read pattern (2 messages: write + read).
	 * Use F424 bus control to implement I2C repeated-start.
	 */
	if (num == 2 &&
	    !(msgs[0].flags & I2C_M_RD) && (msgs[1].flags & I2C_M_RD) &&
	    msgs[0].len <= U8_MAX && msgs[1].len <= U8_MAX) {
		ret = it930x_i2c_xfer_rstart(dev, &msgs[0], &msgs[1]);
		if (ret)
			dev_dbg(&dev->intf->dev,
				"i2c rstart addr=0x%02x wr=%u rd=%u failed: %d\n",
				msgs[0].addr << 1, msgs[0].len,
				msgs[1].len, ret);
		mutex_unlock(&dev->io_lock);
		return ret ? ret : num;
	}

	for (i = 0; i < num; i++) {
		u8 slave = msgs[i].addr << 1;

		if (msgs[i].len > U8_MAX) {
			ret = -E2BIG;
			break;
		}

		if (msgs[i].flags & I2C_M_RD)
			ret = it930x_i2c_read(dev, IT930X_I2C_BUS_IDX,
					      slave, msgs[i].buf, msgs[i].len);
		else
			ret = it930x_i2c_write(dev, IT930X_I2C_BUS_IDX,
					       slave, msgs[i].buf,
					       msgs[i].len);
		if (ret) {
			dev_dbg(&dev->intf->dev,
				"i2c xfer %s addr=0x%02x len=%u failed: %d\n",
				(msgs[i].flags & I2C_M_RD) ? "rd" : "wr",
				slave, msgs[i].len, ret);
			break;
		}
	}

	mutex_unlock(&dev->io_lock);

	return ret ? ret : num;
}

static u32 it930x_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C;
}

static const struct i2c_algorithm it930x_i2c_algo = {
	.master_xfer = it930x_i2c_xfer,
	.functionality = it930x_i2c_func,
};

/* -------- Bridge initialization -------- */

struct it930x_reg_val {
	u32 reg;
	u8 val;
};

static int it930x_write_reg_table(struct it930x_dev *dev,
				  const struct it930x_reg_val *tbl, int count)
{
	int i, ret;

	for (i = 0; i < count; i++) {
		if (tbl[i].reg == 0x0) {
			mdelay((u32)tbl[i].val * 10);
			continue;
		}
		ret = it930x_write_reg(dev, tbl[i].reg, tbl[i].val);
		if (ret)
			return ret;
	}
	return 0;
}

static int it930x_stage_pre_init(struct it930x_dev *dev)
{
	static const struct it930x_reg_val regs[] = {
		{ 0xda05, 0x01 },
		{ IT930X_REG_GPIO2_VALUE, 1 },
		{ IT930X_REG_GPIO2_EN, 1 },
		{ IT930X_REG_GPIO2_DIRECTION, 1 },
		{ IT930X_REG_GPIO2_VALUE, 0 },
		{ IT930X_REG_GPIO2_VALUE, 1 },
		{ IT930X_REG_GPIO14_EN, 1 },
		{ IT930X_REG_GPIO14_DIRECTION, 1 },
		{ IT930X_REG_GPIO14_VALUE, 1 },
		{ IT930X_REG_CLK_DISABLE, 0x00 },
		{ IT930X_REG_PLL_RESET, 0x00 },
		{ IT930X_REG_TS_CLK_CFG, 0x00 },
		{ IT930X_REG_EXT_CLK_RESET, 0x00 },
		{ IT930X_REG_I2C_BUS0_SPEED, 0x07 },
		{ IT930X_REG_I2C_BUS1_SPEED, 0x07 },
		{ IT930X_REG_TS_SYNC_BYPASS, 0x00 },
	};
	return it930x_write_reg_table(dev, regs, ARRAY_SIZE(regs));
}

static int it930x_stage_fw_ready(struct it930x_dev *dev)
{
	u32 fw_version = 0;
	int ret;

	ret = it930x_read_firmware_version(dev, &fw_version);
	if (!ret && fw_version) {
		dev->fw_version = fw_version;
		dev_info(&dev->intf->dev, "firmware version: %u.%u.%u.%u\n",
			 (fw_version >> 24) & 0xff,
			 (fw_version >> 16) & 0xff,
			 (fw_version >> 8) & 0xff,
			 fw_version & 0xff);
		return 0;
	}

	dev_info(&dev->intf->dev, "cold boot, loading firmware\n");

	ret = it930x_load_firmware(dev);
	if (ret)
		return ret;

	ret = it930x_read_firmware_version(dev, &fw_version);
	if (ret)
		return ret;
	if (!fw_version)
		return -ENODEV;

	dev->fw_version = fw_version;
	dev_info(&dev->intf->dev, "firmware version: %u.%u.%u.%u\n",
		 (fw_version >> 24) & 0xff, (fw_version >> 16) & 0xff,
		 (fw_version >> 8) & 0xff, fw_version & 0xff);

	return 0;
}

/*
 * Post-boot bridge configuration.
 * Matches vendor USB trace exactly — both cold and warm boot.
 * Key operations: I2C controller config (f41a), TS clock bit 5 toggle
 * (dd11), TS port enable/disable (da1d), clock save/restore (da05/da06),
 * TS output config, I2C bus enable, GPIO demod reset, I2C passthrough.
 */
static int it930x_stage_post_boot(struct it930x_dev *dev)
{
	u8 dd88[2] = { IT930X_URB_BUF_SIZE / 4 & 0xff,
		       (IT930X_URB_BUF_SIZE / 4 >> 8) & 0xff };
	u8 val;
	int ret;

	/* I2C bus speeds (must re-set after firmware boot) */
	ret = it930x_write_reg(dev, IT930X_REG_I2C_BUS0_SPEED, 0x07);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, IT930X_REG_I2C_BUS1_SPEED, 0x07);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, IT930X_REG_TS_SYNC_BYPASS, 0x00);
	if (ret)
		return ret;

	/* TS output pin patch — set bit 2 of f41f */
	ret = it930x_patch_reg_bit(dev, 0xf41f, 0x04, 0x04);
	if (ret)
		return ret;

	/* Bridge I2C/TS config registers */
	ret = it930x_write_reg(dev, 0xda10, 0x00);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, 0xf41a, 0x05);
	if (ret)
		return ret;

	/* TS port enable — first pass (set bit 0) */
	ret = it930x_write_reg(dev, 0xda1d, 0x01);
	if (ret)
		return ret;

	/*
	 * TS clock config with bit 5 toggle on dd11.
	 * Vendor reads dd11, writes with bit 5 cleared, writes dd13,
	 * then reads dd11 again and writes with bit 5 set.
	 * This toggle is required even on warm boot when bit 5 is
	 * already set — skipping it causes SLV-T NACK.
	 */
	ret = it930x_read_reg(dev, 0xdd11, &val);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, 0xdd11, val & ~0x20);
	if (ret)
		return ret;

	ret = it930x_write_reg(dev, 0xdd13, 0x1b);
	if (ret)
		return ret;

	ret = it930x_read_reg(dev, 0xdd11, &val);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, 0xdd11, val | 0x20);
	if (ret)
		return ret;

	/* DD88 / DD0C */
	ret = it930x_write_regs(dev, 0xdd88, dd88, sizeof(dd88));
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, 0xdd0c, 0x80);
	if (ret)
		return ret;

	/* Clock save/restore registers */
	ret = it930x_write_reg(dev, 0xda05, 0x00);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, 0xda06, 0x00);
	if (ret)
		return ret;

	/* TS port disable — second pass (clear bit 0) */
	ret = it930x_write_reg(dev, 0xda1d, 0x00);
	if (ret)
		return ret;

	/* TS output config */
	ret = it930x_write_reg(dev, 0xd920, 0x00);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, IT930X_REG_TS_OUT_EN, 0x01);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, IT930X_REG_TS_OUT_CFG0, 0x00);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, IT930X_REG_TS_OUT_CFG1, 0x01);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, IT930X_REG_TS_OUT_CFG2, 0x00);
	if (ret)
		return ret;

	/* TS port config */
	ret = it930x_write_reg(dev, IT930X_REG_CLK_DISABLE, 0x01);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, 0x4975, 0x38);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, 0x4971, 0x03);
	if (ret)
		return ret;

	/* I2C bus enable */
	ret = it930x_write_reg(dev, IT930X_REG_I2C_BUS0_EN, 0x01);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, IT930X_REG_I2C_BUS1_EN, 0x01);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, IT930X_REG_I2C_RESET, 0x01);
	if (ret)
		return ret;

	/* GPIO2 demod reset pulse */
	ret = it930x_write_reg(dev, IT930X_REG_GPIO2_EN, 0x01);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, IT930X_REG_GPIO2_DIRECTION, 0x01);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, IT930X_REG_GPIO2_VALUE, 0x00);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, IT930X_REG_GPIO2_VALUE, 0x01);
	if (ret)
		return ret;

	/* Ignore TS input failures on all ports (demod may not have valid
	 * TS clock until it locks to a signal) */
	return it930x_write_reg(dev, IT930X_REG_TS_FAIL_IGNORE, 0x1f);
}

/*
 * TS input port configuration.
 * Written after bridge init and demod/tuner probe to enable
 * TS data flow from CXD6801 through the bridge to USB.
 * Register values from vendor USB warm-boot trace.
 */
static int it930x_stage_ts_port_cfg(struct it930x_dev *dev)
{
	static const struct it930x_reg_val regs[] = {
		{ 0xda34, 0x01 },
		{ 0xda58, 0x00 },	/* TS input type (serial) */
		{ 0xda51, 0xbc },
		{ 0xda73, 0x00 },
		{ 0xda5f, 0x7a },
		{ 0xda60, 0x61 },
		{ 0xda61, 0x33 },
		{ 0xda62, 0x00 },
		{ 0xda4c, 0x01 },	/* TS port 0 enable */
		{ IT930X_REG_TS_FAIL_IGNORE, 0x1f },
	};
	return it930x_write_reg_table(dev, regs, ARRAY_SIZE(regs));
}

static int it930x_hw_init(struct it930x_dev *dev)
{
	int ret;

	ret = it930x_stage_pre_init(dev);
	if (ret)
		return ret;

	ret = it930x_stage_fw_ready(dev);
	if (ret)
		return ret;

	ret = it930x_stage_post_boot(dev);
	if (ret)
		return ret;

	dev_info(&dev->intf->dev, "bridge initialization complete\n");
	return 0;
}

/* -------- Endpoint validation -------- */

static int it930x_validate_endpoints(struct usb_interface *intf)
{
	struct usb_host_interface *alt = intf->cur_altsetting;
	bool have_out = false, have_in = false;
	int i;

	if (!alt)
		return -ENODEV;

	for (i = 0; i < alt->desc.bNumEndpoints; i++) {
		const struct usb_endpoint_descriptor *ep =
			&alt->endpoint[i].desc;

		if (!usb_endpoint_xfer_bulk(ep))
			continue;
		if (usb_endpoint_dir_out(ep) &&
		    ep->bEndpointAddress == IT930X_EP_CMD_OUT)
			have_out = true;
		if (usb_endpoint_dir_in(ep) &&
		    ep->bEndpointAddress == IT930X_EP_CMD_IN)
			have_in = true;
	}

	if (!have_out || !have_in) {
		dev_err(&intf->dev,
			"missing command endpoints (OUT=0x%02x IN=0x%02x)\n",
			IT930X_EP_CMD_OUT, IT930X_EP_CMD_IN);
		return -ENODEV;
	}

	return 0;
}

/* Forward declarations for streaming (used by net device ops) */
static int it930x_start_streaming(struct it930x_dev *dev);
static void it930x_stop_streaming(struct it930x_dev *dev);

/* ALP streaming callbacks for atsc3_alp module */
static int it930x_alp_start(void *priv)
{
	return it930x_start_streaming(priv);
}

static void it930x_alp_stop(void *priv)
{
	it930x_stop_streaming(priv);
}

/* -------- TS streaming -------- */

static void it930x_urb_complete(struct urb *urb)
{
	struct it930x_dev *dev = urb->context;

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
	case -EOVERFLOW:
		dev->urb_complete_err++;
		dev_dbg(&dev->intf->dev,
			"URB overflow (actual=%d buf=%d)\n",
			urb->actual_length, urb->transfer_buffer_length);
		break;
	default:
		dev->urb_complete_err++;
		dev_dbg(&dev->intf->dev, "URB error %d\n", urb->status);
		break;
	}

	usb_submit_urb(urb, GFP_ATOMIC);
}

static int it930x_start_streaming(struct it930x_dev *dev)
{
	int i, ret;
	int flush_len = 0;

	if (dev->streaming)
		return 0;

	/* Sync bypass disabled — both ATSC 1.0 (TS) and ATSC 3.0 (ALP_DIV_TS)
	 * use 0x47 sync bytes.  Enabling bypass breaks ATSC 1.0. */
	mutex_lock(&dev->io_lock);
	ret = it930x_write_reg(dev, IT930X_REG_TS_SYNC_BYPASS, 0x00);
	if (ret) {
		mutex_unlock(&dev->io_lock);
		dev_err(&dev->intf->dev,
			"TS sync bypass config failed (%d)\n", ret);
		return ret;
	}

	/* Configure TS port — demod must be initialized and tuned first */
	ret = it930x_stage_ts_port_cfg(dev);
	mutex_unlock(&dev->io_lock);
	if (ret) {
		dev_err(&dev->intf->dev,
			"TS port config failed (%d)\n", ret);
		return ret;
	}

	/* Flush stale data from bridge PSB/FIFO */
	usb_bulk_msg(dev->udev,
		     usb_rcvbulkpipe(dev->udev, IT930X_EP_TS),
		     dev->urb_bufs[0], IT930X_URB_BUF_SIZE,
		     &flush_len, 100);

	/* Reset stats */
	dev->urb_complete_ok = 0;
	dev->urb_complete_err = 0;
	dev->urb_complete_empty = 0;
	dev->urb_bytes_total = 0;

	for (i = 0; i < IT930X_URB_COUNT; i++) {
		usb_fill_bulk_urb(dev->urbs[i], dev->udev,
				  usb_rcvbulkpipe(dev->udev, IT930X_EP_TS),
				  dev->urb_bufs[i], IT930X_URB_BUF_SIZE,
				  it930x_urb_complete, dev);
		dev->urbs[i]->transfer_dma = dev->urb_dma[i];
		dev->urbs[i]->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	}

	dev->streaming = true;

	for (i = 0; i < IT930X_URB_COUNT; i++) {
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

	dev_info(&dev->intf->dev, "TS streaming started (%d URBs)\n",
		 IT930X_URB_COUNT);

	return 0;
}

static void it930x_stop_streaming(struct it930x_dev *dev)
{
	int i;

	/* Don't stop if another user still needs streaming */
	if (dev->feeding > 0)
		return;

	if (!dev->streaming)
		return;

	dev->streaming = false;

	for (i = 0; i < IT930X_URB_COUNT; i++)
		usb_kill_urb(dev->urbs[i]);

	dev_info(&dev->intf->dev,
		 "TS streaming stopped: ok=%lu err=%lu empty=%lu bytes=%llu\n",
		 dev->urb_complete_ok, dev->urb_complete_err,
		 dev->urb_complete_empty, dev->urb_bytes_total);
}

static int it930x_alloc_urbs(struct it930x_dev *dev)
{
	int i;

	for (i = 0; i < IT930X_URB_COUNT; i++) {
		dev->urbs[i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!dev->urbs[i])
			goto err;

		dev->urb_bufs[i] = usb_alloc_coherent(dev->udev,
						      IT930X_URB_BUF_SIZE,
						      GFP_KERNEL,
						      &dev->urb_dma[i]);
		if (!dev->urb_bufs[i])
			goto err;
	}

	return 0;
err:
	while (--i >= 0) {
		usb_free_coherent(dev->udev, IT930X_URB_BUF_SIZE,
				  dev->urb_bufs[i], dev->urb_dma[i]);
		dev->urb_bufs[i] = NULL;
		usb_free_urb(dev->urbs[i]);
		dev->urbs[i] = NULL;
	}
	return -ENOMEM;
}

static void it930x_free_urbs(struct it930x_dev *dev)
{
	int i;

	for (i = 0; i < IT930X_URB_COUNT; i++) {
		if (dev->urb_bufs[i])
			usb_free_coherent(dev->udev, IT930X_URB_BUF_SIZE,
					  dev->urb_bufs[i],
					  dev->urb_dma[i]);
		usb_free_urb(dev->urbs[i]);
	}
}

/* -------- Hardware PID filter -------- */

/*
 * Program a PID into hardware filter slot @idx.
 * Caller must hold dev->io_lock.
 */
static int it930x_pid_filter_set(struct it930x_dev *dev, u8 idx, u16 pid)
{
	int ret;

	/* Write match table entry */
	ret = it930x_write_reg(dev, IT930X_REG_PID_DAT_H,
			       (pid >> 8) & 0x1f);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, IT930X_REG_PID_DAT_L, pid & 0xff);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, IT930X_REG_PID_INDEX_EN, 0x01);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, IT930X_REG_PID_INDEX, idx);
	if (ret)
		return ret;

	/* Write remap table entry (same PID — no remapping) */
	ret = it930x_write_reg(dev, IT930X_REG_PID_DAT_H,
			       (pid >> 8) & 0x1f);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, IT930X_REG_PID_DAT_L, pid & 0xff);
	if (ret)
		return ret;
	return it930x_write_reg(dev, IT930X_REG_MAP_INDEX, idx);
}

/*
 * Disable hardware filter slot @idx.
 * Caller must hold dev->io_lock.
 */
static int it930x_pid_filter_clear(struct it930x_dev *dev, u8 idx)
{
	int ret;

	ret = it930x_write_reg(dev, IT930X_REG_PID_INDEX_EN, 0x00);
	if (ret)
		return ret;
	return it930x_write_reg(dev, IT930X_REG_PID_INDEX, idx);
}

/*
 * Enable or disable the hardware PID filter engine.
 * on=true:  purge stale entries, set remap_mode=PASS, enable filter.
 * on=false: disable filter (pass all TS through).
 * Caller must hold dev->io_lock.
 */
static int it930x_pid_filter_ctrl(struct it930x_dev *dev, bool on)
{
	int ret, i;

	if (on) {
		/* Clear all 64 slots to remove stale entries */
		for (i = 0; i < IT930X_PID_FILTER_MAX; i++) {
			ret = it930x_pid_filter_clear(dev, i);
			if (ret)
				return ret;
		}
		/* Whitelist mode, no PID remapping, zero offset */
		ret = it930x_write_reg(dev, IT930X_REG_REMAP_MODE, 0x00);
		if (ret)
			return ret;
		ret = it930x_write_reg(dev, IT930X_REG_PID_OFFSET_L, 0x00);
		if (ret)
			return ret;
		ret = it930x_write_reg(dev, IT930X_REG_PID_OFFSET_H, 0x00);
		if (ret)
			return ret;
		/* Activate PID filter engine */
		ret = it930x_write_reg(dev, IT930X_REG_AGGRE_MODE, 0x03);
		if (ret)
			return ret;
		dev->hw_pid_active = true;
		dev_info(&dev->intf->dev, "PID filter: engine enabled\n");
	} else {
		/* Pass all TS — disable filter engine */
		ret = it930x_write_reg(dev, IT930X_REG_AGGRE_MODE, 0x00);
		if (ret)
			return ret;
		dev->hw_pid_active = false;
		dev_info(&dev->intf->dev, "PID filter: engine disabled\n");
	}
	return 0;
}

/* -------- DVB demux -------- */

static int it930x_start_feed(struct dvb_demux_feed *feed)
{
	struct it930x_dev *dev = feed->demux->priv;
	bool first = (dev->feeding == 0);
	int ret;

	dev->feeding++;

	if (first) {
		ret = it930x_start_streaming(dev);
		if (ret) {
			dev->feeding--;
			return ret;
		}
	}

	/* PID 0x2000 = pass-all: disable HW filter. */
	if (feed->pid == 0x2000) {
		if (dev->hw_pid_active) {
			mutex_lock(&dev->io_lock);
			it930x_pid_filter_ctrl(dev, false);
			mutex_unlock(&dev->io_lock);
		}
		return 0;
	}

	mutex_lock(&dev->io_lock);

	if (!dev->hw_pid_active) {
		ret = it930x_pid_filter_ctrl(dev, true);
		if (ret) {
			mutex_unlock(&dev->io_lock);
			dev_warn(&dev->intf->dev,
				 "HW PID filter enable failed (%d), using SW\n",
				 ret);
			return 0; /* fallback to software filtering */
		}
	}

	ret = it930x_pid_filter_set(dev, feed->index, feed->pid);
	if (ret) {
		dev_warn(&dev->intf->dev,
			 "HW PID set idx=%u pid=0x%04x failed (%d)\n",
			 feed->index, feed->pid, ret);
		/* Disable HW filter — fall back to software */
		it930x_pid_filter_ctrl(dev, false);
	} else {
		dev_info(&dev->intf->dev,
			 "PID filter: add pid=0x%04x slot=%u\n",
			 feed->pid, feed->index);
	}

	mutex_unlock(&dev->io_lock);
	return 0;
}

static int it930x_stop_feed(struct dvb_demux_feed *feed)
{
	struct it930x_dev *dev = feed->demux->priv;

	if (dev->hw_pid_active && feed->pid != 0x2000) {
		mutex_lock(&dev->io_lock);
		it930x_pid_filter_clear(dev, feed->index);
		mutex_unlock(&dev->io_lock);
		dev_info(&dev->intf->dev,
			 "PID filter: remove pid=0x%04x slot=%u\n",
			 feed->pid, feed->index);
	}

	if (--dev->feeding == 0) {
		if (dev->hw_pid_active) {
			mutex_lock(&dev->io_lock);
			it930x_pid_filter_ctrl(dev, false);
			mutex_unlock(&dev->io_lock);
		}
		it930x_stop_streaming(dev);
	}

	return 0;
}

/* -------- Frontend attach -------- */

static struct cxd2878_config it930x_cxd2878_cfg = {
	.addr_slvt = 0x6c,
	.xtal = SONY_DEMOD_XTAL_24000KHz,
	.tuner_addr = 0x60,
	.tuner_xtal = SONY_ASCOT3_XTAL_24000KHz,
	.ts_mode = 0,
	.ts_ser_data = 0,
	.ts_clk = 1,
	.ts_clk_mask = 1,
	.ts_valid = 0,
	.atscCoreDisable = 0,
	.lock_flag = 1,
	.write_properties = NULL,
	.read_properties = NULL,
};

static int it930x_frontend_attach(struct it930x_dev *dev)
{
	dev->fe = dvb_attach(cxd2878_attach, &it930x_cxd2878_cfg, &dev->i2c);
	if (!dev->fe) {
		dev_err(&dev->intf->dev, "cxd2878 attach failed\n");
		return -ENODEV;
	}

	if (dev->devname)
		strscpy(dev->fe->ops.info.name, dev->devname,
			sizeof(dev->fe->ops.info.name));

	return 0;
}

/* -------- DVB registration -------- */

static int it930x_dvb_init(struct it930x_dev *dev)
{
	int ret;

	ret = dvb_register_adapter(&dev->dvb_adapter, "Sony CXD2878",
				   THIS_MODULE, &dev->intf->dev,
				   adapter_nr);
	if (ret < 0) {
		dev_err(&dev->intf->dev,
			"dvb_register_adapter failed (%d)\n", ret);
		return ret;
	}

	ret = it930x_frontend_attach(dev);
	if (ret)
		goto err_adapter;

	ret = dvb_register_frontend(&dev->dvb_adapter, dev->fe);
	if (ret) {
		dev_err(&dev->intf->dev,
			"dvb_register_frontend failed (%d)\n", ret);
		dvb_frontend_detach(dev->fe);
		goto err_adapter;
	}

	dev->demux.dmx.capabilities = DMX_TS_FILTERING | DMX_SECTION_FILTERING;
	dev->demux.priv = dev;
	dev->demux.filternum = IT930X_PID_FILTER_MAX;
	dev->demux.feednum = IT930X_PID_FILTER_MAX;
	dev->demux.start_feed = it930x_start_feed;
	dev->demux.stop_feed = it930x_stop_feed;
	ret = dvb_dmx_init(&dev->demux);
	if (ret)
		goto err_frontend;

	dev->dmxdev.filternum = IT930X_PID_FILTER_MAX;
	dev->dmxdev.demux = &dev->demux.dmx;
	dev->dmxdev.capabilities = 0;
	ret = dvb_dmxdev_init(&dev->dmxdev, &dev->dvb_adapter);
	if (ret)
		goto err_dmx;

	ret = dvb_net_init(&dev->dvb_adapter, &dev->dvbnet, &dev->demux.dmx);
	if (ret)
		goto err_dmxdev;

	ret = it930x_alloc_urbs(dev);
	if (ret)
		goto err_net;

	return 0;

err_net:
	dvb_net_release(&dev->dvbnet);
err_dmxdev:
	dvb_dmxdev_release(&dev->dmxdev);
err_dmx:
	dvb_dmx_release(&dev->demux);
err_frontend:
	dvb_unregister_frontend(dev->fe);
err_adapter:
	dvb_unregister_adapter(&dev->dvb_adapter);
	return ret;
}

static void it930x_dvb_exit(struct it930x_dev *dev)
{
	it930x_stop_streaming(dev);
	it930x_free_urbs(dev);
	dvb_net_release(&dev->dvbnet);
	dvb_dmxdev_release(&dev->dmxdev);
	dvb_dmx_release(&dev->demux);
	if (dev->fe) {
		dvb_unregister_frontend(dev->fe);
		dvb_frontend_detach(dev->fe);
	}
	dvb_unregister_adapter(&dev->dvb_adapter);
}

/* -------- USB probe/disconnect -------- */

static int it930x_probe(struct usb_interface *intf,
			const struct usb_device_id *id)
{
	struct it930x_dev *dev;
	int ret;

	ret = it930x_validate_endpoints(intf);
	if (ret)
		return ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->udev = usb_get_dev(interface_to_usbdev(intf));
	dev->intf = intf;
	dev->devname = (const char *)id->driver_info;
	mutex_init(&dev->io_lock);

	/* Start transport threads */
	ret = it930x_transport_start(dev);
	if (ret)
		goto err_free;

	/* Initialize bridge hardware */
	mutex_lock(&dev->io_lock);
	ret = it930x_hw_init(dev);
	mutex_unlock(&dev->io_lock);
	if (ret) {
		dev_err(&intf->dev, "bridge init failed (%d)\n", ret);
		goto err_transport;
	}

	/* Register I2C adapter */
	dev->i2c.owner = THIS_MODULE;
	dev->i2c.algo = &it930x_i2c_algo;
	dev->i2c.dev.parent = &intf->dev;
	strscpy(dev->i2c.name, "it930x-i2c", sizeof(dev->i2c.name));
	i2c_set_adapdata(&dev->i2c, dev);

	ret = i2c_add_adapter(&dev->i2c);
	if (ret) {
		dev_err(&intf->dev, "i2c_add_adapter failed (%d)\n", ret);
		goto err_transport;
	}

	/* Initialize DVB subsystem */
	ret = it930x_dvb_init(dev);
	if (ret)
		goto err_i2c;

	/* ATSC 3.0 ALP network interface (non-fatal if creation fails) */
	dev->alp.fe = dev->fe;
	dev->alp.start_streaming = it930x_alp_start;
	dev->alp.stop_streaming = it930x_alp_stop;
	dev->alp.priv = dev;
	atsc3_alp_register_netdev(&dev->alp);

	usb_set_intfdata(intf, dev);

	dev_info(&intf->dev, "Generic ITE IT930X adapter attached\n");

	return 0;

err_i2c:
	i2c_del_adapter(&dev->i2c);
err_transport:
	it930x_transport_stop(dev);
err_free:
	usb_put_dev(dev->udev);
	kfree(dev);
	return ret;
}

static void it930x_disconnect(struct usb_interface *intf)
{
	struct it930x_dev *dev = usb_get_intfdata(intf);

	if (!dev)
		return;

	atsc3_alp_unregister_netdev(&dev->alp);

	it930x_dvb_exit(dev);
	i2c_del_adapter(&dev->i2c);
	it930x_transport_stop(dev);
	usb_put_dev(dev->udev);
	kfree(dev);

	dev_info(&intf->dev, "Generic ITE IT930X DVB adapter detached\n");
}

static const struct usb_device_id it930x_id_table[] = {
	{ USB_DEVICE(0x23e2, 0x2b02),
	  .driver_info = (unsigned long)"Geniatech HDTV Mate DVB-T/T2/C/C2,ISDB-T/C,ATSC,J83B" },
	{}
};
MODULE_DEVICE_TABLE(usb, it930x_id_table);

static struct usb_driver it930x_usb_driver = {
	.name = "it930x",
	.id_table = it930x_id_table,
	.probe = it930x_probe,
	.disconnect = it930x_disconnect,
};

module_param_array(adapter_nr, short, NULL, 0444);
MODULE_PARM_DESC(adapter_nr, "DVB adapter number(s)");

module_usb_driver(it930x_usb_driver);

MODULE_FIRMWARE(IT9306_FIRMWARE_FILE);
MODULE_DESCRIPTION("Generic ITE IT930X driver");
MODULE_AUTHOR("koreapyj");
MODULE_LICENSE("GPL");
