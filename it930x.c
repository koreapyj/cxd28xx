/*
 * ITE IT930x USB bridge driver for CXD28xx demod + tuner
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
#include <linux/miscdevice.h>
#include <media/dvb_frontend.h>
#include <media/dmxdev.h>
#include <media/dvb_demux.h>
#include <media/dvb_net.h>

#include "cxd2878.h"
#include "atsc3_alp.h"
#include "it930x_smartcard.h"

/* -------- IT930x bridge constants -------- */

/* Command IDs */
#define IT930X_CMD_REG_READ		0x0000
#define IT930X_CMD_REG_WRITE		0x0001
#define IT930X_CMD_QUERYINFO		0x0022
#define IT930X_CMD_BOOT			0x0023
#define IT930X_CMD_FW_SCATTER_WRITE	0x0029
#define IT930X_CMD_GENERIC_I2C_RD	0x002a
#define IT930X_CMD_GENERIC_I2C_WR	0x002b
#define IT930X_CMD_UART_READ		0x0033
#define IT930X_CMD_UART_WRITE		0x0034
#define IT930X_CMD_UART_CTRL		0x0035

/* USB endpoints */
#define IT930X_EP_CMD_OUT		0x02
#define IT930X_EP_CMD_IN		0x81
#define IT930X_EP_TS			0x84

/* Protocol limits */
#define IT930X_CMD_MAX_PKT		512
#define IT930X_CMD_MAX_PAYLOAD		250
#define IT930X_CMD_HDR_LEN		4
#define IT930X_CMD_CSUM_LEN		2
#define IT930X_USB_TIMEOUT_MS		1000
#define IT930X_CMD_WAIT_MS		2000
#define IT930X_RX_IO_TIMEOUT_MS		200

/* I2C limits */
#define IT930X_I2C_SHORT_MAX		40
#define IT930X_I2C_STAGE_CHUNK		40

/* TS streaming */
#define IT930X_URB_COUNT		4
#define IT930X_URB_BUF_SIZE		(188 * 816)

/* Bridge registers */
#define IT930X_REG_CLK_DISABLE		0x4976
#define IT930X_REG_PLL_RESET		0x4bfb
#define IT930X_REG_TS_CLK_CFG		0x4978
#define IT930X_REG_EXT_CLK_RESET	0x4977
#define IT930X_REG_I2C_BUS0_SPEED	0xf6a7
#define IT930X_REG_I2C_BUS1_SPEED	0xf103
#define IT930X_REG_I2C_TRIG		0x4900
#define IT930X_REG_I2C_READ_BUF	0xf000
#define IT930X_REG_I2C_WRITE_BUF	0xf001
#define IT930X_REG_I2C_BUS_CTRL	0xf424
#define IT930X_REG_TS_FAIL_IGNORE	0xda5a
#define IT930X_REG_TS_SYNC_BYPASS	0xda1a
#define IT930X_REG_TS_OUT_EN		0xd833
#define IT930X_REG_TS_OUT_CFG0		0xd830
#define IT930X_REG_TS_OUT_CFG1		0xd831
#define IT930X_REG_TS_OUT_CFG2		0xd832

#define IT930X_PID_FILTER_MAX		64

/* Smartcard / UART registers */
#define IT930X_SC_UART_READY		0x496A
#define IT930X_SC_UART_RXCOUNT		0x496B
#define IT930X_SC_UART_MODE		0x7904
#define IT930X_SC_UART_ENABLE		0x4965	/* Type 2 TX enable */
#define IT930X_SC_BOARD_TYPE_REG		0x49E8

#define IT930X_SC_GPIO_DETECT		6	/* GPIOH6 - card detect (active low) */
#define IT930X_SC_GPIO_TXEN		3	/* GPIOH3 - TX enable, Type 1 */
#define IT930X_SC_UART_WR_CHUNK		48
#define IT930X_SC_UART_RD_CHUNK		32
#define IT930X_SC_POLL_COUNT		50
#define IT930X_SC_POLL_MS		10

/* -------- GPIO register table -------- */

/*
 * IT930x has 16 GPIOs. Each GPIO has three registers:
 *   en  — direction (1 = output)
 *   on  — enable    (1 = enabled)
 *   val — output value
 * Indexed 0-15 for GPIO 1-16.
 */
struct it930x_gpio_regs {
	u32 en;
	u32 on;
	u32 val;
};

static const struct it930x_gpio_regs it930x_gpio[16] = {
	{ 0xd8b0, 0xd8b1, 0xd8af },	/* GPIO 1  */
	{ 0xd8b8, 0xd8b9, 0xd8b7 },	/* GPIO 2  */
	{ 0xd8b4, 0xd8b5, 0xd8b3 },	/* GPIO 3  */
	{ 0xd8c0, 0xd8c1, 0xd8bf },	/* GPIO 4  */
	{ 0xd8bc, 0xd8bd, 0xd8bb },	/* GPIO 5  */
	{ 0xd8c8, 0xd8c9, 0xd8c7 },	/* GPIO 6  */
	{ 0xd8c4, 0xd8c5, 0xd8c3 },	/* GPIO 7  */
	{ 0xd8d0, 0xd8d1, 0xd8cf },	/* GPIO 8  */
	{ 0xd8cc, 0xd8cd, 0xd8cb },	/* GPIO 9  */
	{ 0xd8d8, 0xd8d9, 0xd8d7 },	/* GPIO 10 */
	{ 0xd8d4, 0xd8d5, 0xd8d3 },	/* GPIO 11 */
	{ 0xd8e0, 0xd8e1, 0xd8df },	/* GPIO 12 */
	{ 0xd8dc, 0xd8dd, 0xd8db },	/* GPIO 13 */
	{ 0xd8e4, 0xd8e5, 0xd8e3 },	/* GPIO 14 */
	{ 0xd8e8, 0xd8e9, 0xd8e7 },	/* GPIO 15 */
	{ 0xd8ec, 0xd8ed, 0xd8eb },	/* GPIO 16 */
};

/* -------- Board configuration -------- */

#define IT930X_MAX_FRONTENDS	5

struct it930x_fe_cfg {
	u8 ts_port;	/* IT930x stream input port (0-4) */
	u8 i2c_bus;	/* IT930x internal bus: 1 or 3 */
	u8 demod_addr;	/* demod SLVT 7-bit I2C address */
	u8 tuner_addr;	/* tuner 7-bit I2C address */
};

struct it930x_board_cfg {
	const char		*name;
	const char		*fw_file;
	int			num_frontends;
	struct it930x_fe_cfg	fe[IT930X_MAX_FRONTENDS];
	u8			gpio_power;	/* backend power GPIO (active low), 0=none */
	u8			gpio_reset;	/* demod reset GPIO (pulse low) */
	u8			gpio_lnb;	/* LNB power GPIO (active high), 0=none */
	u8			gpio_always_hi;	/* GPIO set high at init, 0=none */
	u8			f41a_val;	/* 0x05=DVB, 0x01=ISDB-T */
	u8			i2c_notify;	/* override 0x4975 value, 0=use demod_addr<<1 */
	enum sony_ascot3_xtal_t	tuner_xtal;
	u8			gpio_sc_reset;	/* smartcard reset GPIO, 0=no smartcard */
};

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

struct it930x_i2c_ctx {
	struct it930x_dev *dev;
	u8 bus_num;
};

struct it930x_fe_ctx {
	struct it930x_dev	*dev;
	int			idx;
	struct i2c_adapter	*i2c;

	struct cxd2878_config	demod_cfg;
	struct dvb_frontend	*fe;
	struct dvb_frontend	*fe_sat;	/* satellite virtual fe, or NULL */
	struct dvb_adapter	adapter;
	struct dvb_demux	demux;
	struct dmxdev		dmxdev;
	struct dvb_net		dvbnet;

	int			feeding;
	struct atsc3_alp	alp;
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

	/* Board config */
	const struct it930x_board_cfg *board;

	/* Multi-bus I2C: [0]=bus 1, [1]=bus 3 */
	struct it930x_i2c_ctx i2c_ctx[2];
	struct i2c_adapter i2c[2];

	/* Per-frontend DVB state */
	struct it930x_fe_ctx fes[IT930X_MAX_FRONTENDS];

	/* Shared streaming state */
	int stream_count;
	struct mutex stream_lock;

	/* LNB power reference count */
	int lnb_count;
	struct mutex lnb_lock;

	/* TS streaming */
	struct urb *urbs[IT930X_URB_COUNT];
	u8 *urb_bufs[IT930X_URB_COUNT];
	dma_addr_t urb_dma[IT930X_URB_COUNT];
	bool streaming;

	/* Streaming stats */
	unsigned long urb_complete_ok;
	unsigned long urb_complete_err;
	unsigned long urb_complete_empty;
	u64 urb_bytes_total;

	/* Smartcard reader */
	int sc_uart_type;		/* 0=none, 1=GPIO TX, 2=firmware */
	struct mutex sc_lock;
	struct miscdevice sc_misc;
	char sc_name[32];
	u8 sc_atr[IT930X_SC_MAX_ATR];
	int sc_atr_len;
};

static short adapter_nr[] = { [0 ... 19] = -1 };
static atomic_t sc_counter = ATOMIC_INIT(0);

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

/* -------- GPIO helpers -------- */

/*
 * Set a GPIO to output mode with the given value.
 * Caller must hold dev->io_lock.
 */
static int it930x_gpio_set(struct it930x_dev *dev, u8 gpio, bool val)
{
	const struct it930x_gpio_regs *g;
	int ret;

	if (gpio < 1 || gpio > 16)
		return -EINVAL;

	g = &it930x_gpio[gpio - 1];

	ret = it930x_write_reg(dev, g->en, 0x01);	/* direction = output */
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, g->on, 0x01);	/* enable */
	if (ret)
		return ret;
	return it930x_write_reg(dev, g->val, val ? 0x01 : 0x00);
}

/*
 * Read a GPIO input value.
 * Input register is at val_reg - 1 (separate from output register).
 * Caller must hold dev->io_lock.
 */
static int it930x_gpio_read(struct it930x_dev *dev, u8 gpio, bool *val)
{
	const struct it930x_gpio_regs *g;
	u8 v;
	int ret;

	if (gpio < 1 || gpio > 16)
		return -EINVAL;

	g = &it930x_gpio[gpio - 1];

	ret = it930x_write_reg(dev, g->on, 0x01);	/* enable */
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, g->en, 0x00);	/* direction = input */
	if (ret)
		return ret;
	ret = it930x_read_reg(dev, g->val - 1, &v);	/* input register */
	if (ret)
		return ret;

	*val = !!v;
	return 0;
}

/*
 * Pulse a GPIO low then high (reset pulse).
 * Caller must hold dev->io_lock.
 */
static int it930x_gpio_pulse(struct it930x_dev *dev, u8 gpio)
{
	const struct it930x_gpio_regs *g;
	int ret;

	if (gpio < 1 || gpio > 16)
		return -EINVAL;

	g = &it930x_gpio[gpio - 1];

	ret = it930x_write_reg(dev, g->val, 0x01);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, g->en, 0x01);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, g->on, 0x01);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, g->val, 0x00);
	if (ret)
		return ret;
	return it930x_write_reg(dev, g->val, 0x01);
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
	const char *fw_file = dev->board->fw_file;
	size_t i = 0;
	int ret;

	ret = request_firmware(&fw, fw_file, &dev->intf->dev);
	if (ret) {
		dev_err(&dev->intf->dev,
			"failed to load firmware %s (%d)\n", fw_file, ret);
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
 */
static int it930x_i2c_xfer_rstart(struct it930x_dev *dev, u8 bus,
				  struct i2c_msg *wr, struct i2c_msg *rd)
{
	u8 wr_slave = wr->addr << 1;
	u8 rd_slave = rd->addr << 1;
	int ret;

	ret = it930x_write_reg(dev, IT930X_REG_I2C_BUS_CTRL, 0x01);
	if (ret)
		return ret;

	ret = it930x_i2c_write(dev, bus, wr_slave, wr->buf, wr->len);
	if (ret) {
		it930x_write_reg(dev, IT930X_REG_I2C_BUS_CTRL, 0x00);
		return ret;
	}

	ret = it930x_write_reg(dev, IT930X_REG_I2C_BUS_CTRL, 0x00);
	if (ret)
		return ret;

	return it930x_i2c_read(dev, bus, rd_slave, rd->buf, rd->len);
}

static int it930x_i2c_xfer(struct i2c_adapter *adap,
			    struct i2c_msg *msgs, int num)
{
	struct it930x_i2c_ctx *ctx = i2c_get_adapdata(adap);
	struct it930x_dev *dev = ctx->dev;
	u8 bus = ctx->bus_num;
	int i, ret = 0;

	mutex_lock(&dev->io_lock);

	if (num == 2 &&
	    !(msgs[0].flags & I2C_M_RD) && (msgs[1].flags & I2C_M_RD) &&
	    msgs[0].len <= U8_MAX && msgs[1].len <= U8_MAX) {
		ret = it930x_i2c_xfer_rstart(dev, bus, &msgs[0], &msgs[1]);
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
			ret = it930x_i2c_read(dev, bus,
					      slave, msgs[i].buf, msgs[i].len);
		else
			ret = it930x_i2c_write(dev, bus,
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

static int it930x_stage_pre_init(struct it930x_dev *dev)
{
	const struct it930x_board_cfg *board = dev->board;
	int ret;

	ret = it930x_write_reg(dev, 0xda05, 0x01);
	if (ret)
		return ret;

	/* GPIO reset pulse */
	ret = it930x_gpio_pulse(dev, board->gpio_reset);
	if (ret)
		return ret;

	/* GPIO always-high (23e2:2b02: GPIO14 for power rail) */
	if (board->gpio_always_hi) {
		ret = it930x_gpio_set(dev, board->gpio_always_hi, true);
		if (ret)
			return ret;
	}

	/* Backend power GPIO: set HIGH initially (power off, active low) */
	if (board->gpio_power) {
		ret = it930x_gpio_set(dev, board->gpio_power, true);
		if (ret)
			return ret;
	}

	/* LNB power GPIO: set LOW initially (off) */
	if (board->gpio_lnb) {
		ret = it930x_gpio_set(dev, board->gpio_lnb, false);
		if (ret)
			return ret;
	}

	/* Clock/PLL/I2C speed common init */
	ret = it930x_write_reg(dev, IT930X_REG_CLK_DISABLE, 0x00);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, IT930X_REG_PLL_RESET, 0x00);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, IT930X_REG_TS_CLK_CFG, 0x00);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, IT930X_REG_EXT_CLK_RESET, 0x00);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, IT930X_REG_I2C_BUS0_SPEED, 0x07);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, IT930X_REG_I2C_BUS1_SPEED, 0x07);
	if (ret)
		return ret;
	return it930x_write_reg(dev, IT930X_REG_TS_SYNC_BYPASS, 0x00);
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
 * Key operations: I2C controller config, TS clock toggle, I2C slave
 * address registration, GPIO reset pulse, TS output config.
 */
static int it930x_stage_post_boot(struct it930x_dev *dev)
{
	const struct it930x_board_cfg *board = dev->board;
	static const u32 addr_regs[] = { 0x4975, 0x4974, 0x4973, 0x4972, 0x4964 };
	static const u32 bus_regs[]  = { 0x4971, 0x4970, 0x496f, 0x496e, 0x4963 };
	u8 dd88[2] = { IT930X_URB_BUF_SIZE / 4 & 0xff,
		       (IT930X_URB_BUF_SIZE / 4 >> 8) & 0xff };
	u8 val;
	int ret, i;

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
	ret = it930x_write_reg(dev, 0xf41a, board->f41a_val);
	if (ret)
		return ret;

	/* TS port enable — first pass (set bit 0) */
	ret = it930x_write_reg(dev, 0xda1d, 0x01);
	if (ret)
		return ret;

	/*
	 * TS clock config with bit 5 toggle on dd11.
	 * This toggle is required even on warm boot.
	 */
	ret = it930x_read_reg(dev, 0xdd11, &val);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, 0xdd11, val & ~0x20);
	if (ret)
		return ret;

	ret = it930x_write_reg(dev, 0xdd13, 0x00);
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

	/* I2C slave address/bus registration for each frontend */
	ret = it930x_write_reg(dev, IT930X_REG_CLK_DISABLE, 0x01);
	if (ret)
		return ret;

	for (i = 0; i < board->num_frontends; i++) {
		const struct it930x_fe_cfg *fc = &board->fe[i];
		u8 addr_val;

		if (i == 0 && board->i2c_notify)
			addr_val = board->i2c_notify;
		else
			addr_val = fc->demod_addr << 1;

		ret = it930x_write_reg(dev, addr_regs[i], addr_val);
		if (ret)
			return ret;
		ret = it930x_write_reg(dev, bus_regs[i], fc->i2c_bus);
		if (ret)
			return ret;
	}

	/* GPIO11 output high (23e2:2b02 I2C bus enable / PX-MLT LNB off) */
	ret = it930x_gpio_set(dev, 11, true);
	if (ret)
		return ret;

	/* GPIO reset pulse after I2C slave config */
	ret = it930x_gpio_pulse(dev, board->gpio_reset);
	if (ret)
		return ret;

	/* Ignore TS input failures on all ports */
	return it930x_write_reg(dev, IT930X_REG_TS_FAIL_IGNORE, 0x1f);
}

/*
 * TS input port configuration.
 * For single-frontend boards: standard 0x47 sync, no aggregation.
 * For multi-frontend boards: per-port sync byte aggregation.
 */
static int it930x_stage_ts_port_cfg(struct it930x_dev *dev)
{
	const struct it930x_board_cfg *board = dev->board;
	int i, ret;

	/* Global TS input registers */
	ret = it930x_write_reg(dev, 0xda34, 0x01);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, 0xda51, 0xbc);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, 0xda5f, 0x7a);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, 0xda60, 0x61);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, 0xda61, 0x33);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, 0xda62, 0x00);
	if (ret)
		return ret;

	for (i = 0; i < board->num_frontends; i++) {
		u8 port = board->fe[i].ts_port;

		/* Serial mode for ports 0-1 */
		if (port < 2) {
			ret = it930x_write_reg(dev, 0xda58 + port, 0x00);
			if (ret)
				return ret;
		}

		if (board->num_frontends > 1) {
			/* Sync-byte aggregation mode */
			u8 sync = (u8)(((i + 1) << 4) | 0x07);

			ret = it930x_write_reg(dev, 0xda73 + port, 0x01);
			if (ret)
				return ret;
			ret = it930x_write_reg(dev, 0xda78 + port, sync);
			if (ret)
				return ret;
		} else {
			/* No aggregation — standard 0x47 */
			ret = it930x_write_reg(dev, 0xda73 + port, 0x00);
			if (ret)
				return ret;
		}

		/* Enable port */
		ret = it930x_write_reg(dev, 0xda4c + port, 0x01);
		if (ret)
			return ret;
	}

	return it930x_write_reg(dev, IT930X_REG_TS_FAIL_IGNORE, 0x1f);
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

/* Forward declarations for streaming */
static int it930x_start_streaming(struct it930x_dev *dev);
static void it930x_stop_streaming(struct it930x_dev *dev);

/* ALP streaming callbacks for atsc3_alp module */
static int it930x_alp_start(void *priv)
{
	struct it930x_fe_ctx *ife = priv;
	struct it930x_dev *dev = ife->dev;
	int ret = 0;

	mutex_lock(&dev->stream_lock);
	if (dev->stream_count++ == 0)
		ret = it930x_start_streaming(dev);
	mutex_unlock(&dev->stream_lock);
	return ret;
}

static void it930x_alp_stop(void *priv)
{
	struct it930x_fe_ctx *ife = priv;
	struct it930x_dev *dev = ife->dev;

	mutex_lock(&dev->stream_lock);
	if (--dev->stream_count == 0)
		it930x_stop_streaming(dev);
	mutex_unlock(&dev->stream_lock);
}

/* -------- TS streaming -------- */

/*
 * Zero-copy TS packet routing for multi-frontend devices.
 *
 * URB_BUF_SIZE = 188 * 816 guarantees packets never span URB boundaries.
 * The IT930x threshold register (0xdd88) ensures complete multiples of 188.
 *
 * For multi-frontend: each packet has a modified sync byte encoding the
 * port ID. Pattern: (sync & 0x8f) == 0x07, fe_idx = ((sync >> 4) & 7) - 1.
 * We restore 0x47 in-place — safe because the DMA buffer is overwritten
 * on the next USB transfer.
 */
static void it930x_ts_route(struct it930x_dev *dev, u8 *buf, u32 len)
{
	u32 pos;

	/* Fast path: single frontend, standard 0x47 sync */
	if (dev->board->num_frontends == 1) {
		dvb_dmx_swfilter(&dev->fes[0].demux, buf, len);
		atsc3_alp_feed(&dev->fes[0].alp, buf, len);
		return;
	}

	/* Multi-frontend routing */
	for (pos = 0; pos + 188 <= len; pos += 188) {
		u8 s = buf[pos];
		int fe_idx;

		if ((s & 0x8f) != 0x07)
			continue;

		fe_idx = ((s & 0x70) >> 4) - 1;
		if ((unsigned int)fe_idx >= (unsigned int)dev->board->num_frontends)
			continue;

		buf[pos] = 0x47;
		dvb_dmx_swfilter(&dev->fes[fe_idx].demux, buf + pos, 188);
	}
}

static void it930x_urb_complete(struct urb *urb)
{
	struct it930x_dev *dev = urb->context;

	if (!dev->streaming)
		return;

	switch (urb->status) {
	case 0:
		if (urb->actual_length > 0) {
			it930x_ts_route(dev, urb->transfer_buffer,
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

	mutex_lock(&dev->io_lock);
	ret = it930x_write_reg(dev, IT930X_REG_TS_SYNC_BYPASS, 0x00);
	if (ret) {
		mutex_unlock(&dev->io_lock);
		dev_err(&dev->intf->dev,
			"TS sync bypass config failed (%d)\n", ret);
		return ret;
	}

	ret = it930x_stage_ts_port_cfg(dev);
	mutex_unlock(&dev->io_lock);
	if (ret) {
		dev_err(&dev->intf->dev,
			"TS port config failed (%d)\n", ret);
		return ret;
	}

	/* Flush stale data from bridge FIFO */
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
 * PID filter registers are per-port on multi-frontend devices.
 * Port 0 base addresses are used; for other ports the register set
 * is offset. Currently only used for single-frontend (port 0).
 */
static int it930x_pid_filter_set(struct it930x_dev *dev, u8 idx, u16 pid)
{
	int ret;

	ret = it930x_write_reg(dev, 0xda17, (pid >> 8) & 0x1f);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, 0xda16, pid & 0xff);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, 0xda14, 0x01);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, 0xda15, idx);
	if (ret)
		return ret;

	ret = it930x_write_reg(dev, 0xda17, (pid >> 8) & 0x1f);
	if (ret)
		return ret;
	ret = it930x_write_reg(dev, 0xda16, pid & 0xff);
	if (ret)
		return ret;
	return it930x_write_reg(dev, 0xda11, idx);
}

static int it930x_pid_filter_clear(struct it930x_dev *dev, u8 idx)
{
	int ret;

	ret = it930x_write_reg(dev, 0xda14, 0x00);
	if (ret)
		return ret;
	return it930x_write_reg(dev, 0xda15, idx);
}

static int it930x_pid_filter_ctrl(struct it930x_fe_ctx *ife, bool on)
{
	struct it930x_dev *dev = ife->dev;
	int ret, i;

	if (on) {
		for (i = 0; i < IT930X_PID_FILTER_MAX; i++) {
			ret = it930x_pid_filter_clear(dev, i);
			if (ret)
				return ret;
		}
		ret = it930x_write_reg(dev, 0xda13, 0x00);
		if (ret)
			return ret;
		ret = it930x_write_reg(dev, 0xda81, 0x00);
		if (ret)
			return ret;
		ret = it930x_write_reg(dev, 0xda82, 0x00);
		if (ret)
			return ret;
		ret = it930x_write_reg(dev, 0xda73, 0x03);
		if (ret)
			return ret;
		dev_info(&dev->intf->dev, "PID filter: engine enabled\n");
	} else {
		ret = it930x_write_reg(dev, 0xda73, 0x00);
		if (ret)
			return ret;
		dev_info(&dev->intf->dev, "PID filter: engine disabled\n");
	}
	return 0;
}

/* -------- DVB demux -------- */

static int it930x_start_feed(struct dvb_demux_feed *feed)
{
	struct it930x_fe_ctx *ife = feed->demux->priv;
	struct it930x_dev *dev = ife->dev;
	int ret;

	ife->feeding++;

	if (ife->feeding == 1) {
		mutex_lock(&dev->stream_lock);
		if (dev->stream_count++ == 0) {
			ret = it930x_start_streaming(dev);
			if (ret) {
				dev->stream_count--;
				mutex_unlock(&dev->stream_lock);
				ife->feeding--;
				return ret;
			}
		}
		mutex_unlock(&dev->stream_lock);
	}

	return 0;
}

static int it930x_stop_feed(struct dvb_demux_feed *feed)
{
	struct it930x_fe_ctx *ife = feed->demux->priv;
	struct it930x_dev *dev = ife->dev;

	if (--ife->feeding == 0) {
		mutex_lock(&dev->stream_lock);
		if (--dev->stream_count == 0)
			it930x_stop_streaming(dev);
		mutex_unlock(&dev->stream_lock);
	}

	return 0;
}

/* -------- LNB power control -------- */

static int it930x_set_lnb(struct i2c_adapter *i2c, int on)
{
	struct it930x_i2c_ctx *ctx = i2c_get_adapdata(i2c);
	struct it930x_dev *dev = ctx->dev;
	int ret = 0;

	if (!dev->board->gpio_lnb)
		return 0;

	mutex_lock(&dev->lnb_lock);
	if (on) {
		if (dev->lnb_count++ == 0) {
			mutex_lock(&dev->io_lock);
			ret = it930x_gpio_set(dev, dev->board->gpio_lnb, true);
			mutex_unlock(&dev->io_lock);
		}
	} else {
		if (--dev->lnb_count <= 0) {
			dev->lnb_count = 0;
			mutex_lock(&dev->io_lock);
			ret = it930x_gpio_set(dev, dev->board->gpio_lnb, false);
			mutex_unlock(&dev->io_lock);
		}
	}
	mutex_unlock(&dev->lnb_lock);
	return ret;
}

/* -------- Smartcard reader -------- */

/*
 * UART command wrappers.
 * These use the same payload format as register read/write
 * (len, processor, addr32, data) but with UART command IDs.
 * Caller must hold dev->io_lock.
 */
static int it930x_uart_ctrl(struct it930x_dev *dev, u32 addr,
			    const u8 *data, u8 len)
{
	u8 payload[6 + 250];

	if (len > 250)
		return -E2BIG;

	payload[0] = len;
	payload[1] = 2;
	payload[2] = (addr >> 24) & 0xff;
	payload[3] = (addr >> 16) & 0xff;
	payload[4] = (addr >> 8) & 0xff;
	payload[5] = addr & 0xff;
	if (len)
		memcpy(&payload[6], data, len);

	return it930x_cmd_submit_sync(dev, IT930X_CMD_UART_CTRL,
				      payload, 6 + len, NULL, 0);
}

static int it930x_uart_write(struct it930x_dev *dev, const u8 *data, u8 len)
{
	u8 payload[6 + 250];

	if (len > IT930X_SC_UART_WR_CHUNK)
		return -E2BIG;

	payload[0] = len;
	payload[1] = 2;
	payload[2] = 0;
	payload[3] = 0;
	payload[4] = 0;
	payload[5] = 0;
	memcpy(&payload[6], data, len);

	return it930x_cmd_submit_sync(dev, IT930X_CMD_UART_WRITE,
				      payload, 6 + len, NULL, 0);
}

static int it930x_uart_read(struct it930x_dev *dev, u8 *data, u8 len)
{
	u8 payload[6];

	if (len > IT930X_SC_UART_RD_CHUNK)
		return -E2BIG;

	payload[0] = len;
	payload[1] = 2;
	payload[2] = 0;
	payload[3] = 0;
	payload[4] = 0;
	payload[5] = 0;

	return it930x_cmd_submit_sync(dev, IT930X_CMD_UART_READ,
				      payload, sizeof(payload), data, len);
}

/* Poll UART ready register. Caller must hold dev->io_lock. */
static int it930x_uart_wait_ready(struct it930x_dev *dev)
{
	u8 val;
	int i, ret;

	for (i = 0; i < IT930X_SC_POLL_COUNT; i++) {
		ret = it930x_read_reg(dev, IT930X_SC_UART_READY, &val);
		if (ret)
			return ret;
		if (val)
			return 0;
		mutex_unlock(&dev->io_lock);
		msleep(IT930X_SC_POLL_MS);
		mutex_lock(&dev->io_lock);
	}
	return -ETIMEDOUT;
}

/*
 * Detect UART type from board type register (0x49E8).
 * 0x30 → type 2 (firmware-managed), 0x31 → disabled,
 * 0x50/0x51/other → type 1 (GPIO TX enable).
 * Caller must hold dev->io_lock.
 */
static int it930x_sc_detect_type(struct it930x_dev *dev)
{
	u8 val;
	int ret;

	ret = it930x_read_reg(dev, IT930X_SC_BOARD_TYPE_REG, &val);
	if (ret) {
		dev_dbg(&dev->intf->dev,
			"sc: board type reg read failed (%d)\n", ret);
		return 0;
	}

	dev_dbg(&dev->intf->dev, "sc: board type = 0x%02x\n", val);

	switch (val) {
	case 0x30:
		return 2; /* firmware-managed UART */
	case 0x31:
		return 0; /* no smartcard */
	default:
		return 1; /* GPIO TX enable */
	}
}

/* Check if smartcard is inserted. Returns 1=present, 0=absent, <0=error. */
static int it930x_sc_card_detect(struct it930x_dev *dev)
{
	bool gpio_val;
	int ret;

	mutex_lock(&dev->io_lock);
	ret = it930x_gpio_read(dev, IT930X_SC_GPIO_DETECT, &gpio_val);
	mutex_unlock(&dev->io_lock);
	if (ret)
		return ret;

	return !gpio_val; /* active low: 0 = card present */
}

/* Reset smartcard and read ATR. Returns 0 on success. */
static int it930x_sc_reset(struct it930x_dev *dev)
{
	u8 ctrl_data = 0x01;
	u8 count;
	int ret;

	mutex_lock(&dev->io_lock);

	/* Assert reset: GPIO low */
	ret = it930x_gpio_set(dev, dev->board->gpio_sc_reset, false);
	if (ret)
		goto out;

	/* Set UART mode */
	ret = it930x_write_reg(dev, IT930X_SC_UART_MODE, 0x02);
	if (ret)
		goto out;

	/* UART ctrl reset command */
	ret = it930x_uart_ctrl(dev, 0x35, &ctrl_data, 1);
	if (ret)
		goto out;

	mutex_unlock(&dev->io_lock);
	msleep(5);
	mutex_lock(&dev->io_lock);

	/* De-assert reset: GPIO high, then release to input */
	ret = it930x_gpio_set(dev, dev->board->gpio_sc_reset, true);
	if (ret)
		goto out;
	ret = it930x_write_reg(dev,
		it930x_gpio[dev->board->gpio_sc_reset - 1].en, 0x00);
	if (ret)
		goto out;

	/* Wait for card response */
	ret = it930x_uart_wait_ready(dev);
	if (ret)
		goto out;

	/* Read ATR */
	ret = it930x_read_reg(dev, IT930X_SC_UART_RXCOUNT, &count);
	if (ret)
		goto out;
	if (count > IT930X_SC_MAX_ATR)
		count = IT930X_SC_MAX_ATR;
	if (count == 0) {
		ret = -EIO;
		goto out;
	}

	ret = it930x_uart_read(dev, dev->sc_atr, count);
	if (!ret)
		dev->sc_atr_len = count;

out:
	mutex_unlock(&dev->io_lock);
	return ret;
}

/* Send data to smartcard via UART. */
static int it930x_sc_send(struct it930x_dev *dev, const u8 *data, int len)
{
	int offset = 0;
	int chunk;
	int ret;

	mutex_lock(&dev->io_lock);

	/* TX enable */
	if (dev->sc_uart_type == 1) {
		ret = it930x_gpio_set(dev, IT930X_SC_GPIO_TXEN, true);
		if (ret)
			goto out;
	} else {
		ret = it930x_write_reg(dev, IT930X_SC_UART_ENABLE, 0x01);
		if (ret)
			goto out;
	}

	/* Send in chunks */
	while (offset < len) {
		chunk = min(len - offset, IT930X_SC_UART_WR_CHUNK);
		ret = it930x_uart_write(dev, data + offset, chunk);
		if (ret)
			goto tx_disable;
		offset += chunk;
	}

tx_disable:
	/* TX disable (Type 1 only) */
	if (dev->sc_uart_type == 1)
		it930x_gpio_set(dev, IT930X_SC_GPIO_TXEN, false);

	if (ret)
		goto out;

	/* Wait for card response */
	ret = it930x_uart_wait_ready(dev);

out:
	mutex_unlock(&dev->io_lock);
	return ret;
}

/* Receive data from smartcard UART. Returns bytes read in *len. */
static int it930x_sc_recv(struct it930x_dev *dev, u8 *data, int *len)
{
	u8 count;
	int total = 0;
	int chunk;
	int ret;

	mutex_lock(&dev->io_lock);

	ret = it930x_read_reg(dev, IT930X_SC_UART_RXCOUNT, &count);
	if (ret)
		goto out;

	while (count > 0) {
		chunk = min_t(int, count, IT930X_SC_UART_RD_CHUNK);
		ret = it930x_uart_read(dev, data + total, chunk);
		if (ret)
			goto out;
		total += chunk;
		count -= chunk;
	}

	*len = total;

out:
	mutex_unlock(&dev->io_lock);
	return ret;
}

/* Set smartcard UART baudrate. */
static int it930x_sc_set_baudrate(struct it930x_dev *dev, int baudrate)
{
	u8 val;
	int ret;

	switch (baudrate) {
	case 9600:	val = 0x00; break;
	case 19200:	val = 0x01; break;
	case 38400:	val = 0x02; break;
	case 57600:	val = 0xF5; break;
	case 115200:	val = 0xFA; break;
	default:	return -EINVAL;
	}

	mutex_lock(&dev->io_lock);

	ret = it930x_uart_ctrl(dev, 0, &val, 1);

	mutex_unlock(&dev->io_lock);
	return ret;
}

/* Initialize smartcard hardware. Called during probe. */
static int it930x_sc_init_hw(struct it930x_dev *dev)
{
	u8 data = 0x01;
	int ret = 0;

	mutex_lock(&dev->io_lock);

	if (dev->sc_uart_type == 2) {
		/* BCAS init for Type 2 */
		ret = it930x_uart_ctrl(dev, 0x37, &data, 1);
		if (ret)
			dev_warn(&dev->intf->dev,
				 "BCAS init failed (%d)\n", ret);
	}

	mutex_unlock(&dev->io_lock);
	return ret;
}

/* Chardev file operations */
static int it930x_sc_open(struct inode *inode, struct file *file)
{
	struct it930x_dev *dev = container_of(file->private_data,
					      struct it930x_dev, sc_misc);
	file->private_data = dev;
	return 0;
}

static int it930x_sc_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t it930x_sc_fop_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct it930x_dev *dev = file->private_data;
	u8 kbuf[256];
	int ret;

	if (count == 0 || count > sizeof(kbuf))
		return -EINVAL;

	if (copy_from_user(kbuf, buf, count))
		return -EFAULT;

	mutex_lock(&dev->sc_lock);
	ret = it930x_sc_send(dev, kbuf, count);
	mutex_unlock(&dev->sc_lock);

	return ret ? ret : count;
}

static ssize_t it930x_sc_fop_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct it930x_dev *dev = file->private_data;
	u8 kbuf[256];
	int len = 0;
	int ret;

	if (count == 0)
		return 0;
	if (count > sizeof(kbuf))
		count = sizeof(kbuf);

	mutex_lock(&dev->sc_lock);
	ret = it930x_sc_recv(dev, kbuf, &len);
	mutex_unlock(&dev->sc_lock);

	if (ret)
		return ret;
	if (len == 0)
		return 0;
	if (len > count)
		len = count;

	if (copy_to_user(buf, kbuf, len))
		return -EFAULT;

	return len;
}

static long it930x_sc_ioctl(struct file *file, unsigned int cmd,
			    unsigned long arg)
{
	struct it930x_dev *dev = file->private_data;
	struct it930x_sc_atr atr_data;
	int present;
	int ret;

	switch (cmd) {
	case IT930X_SC_CARD_DETECT:
		ret = it930x_sc_card_detect(dev);
		if (ret < 0)
			return ret;
		present = ret;
		if (copy_to_user((void __user *)arg, &present, sizeof(present)))
			return -EFAULT;
		return 0;

	case IT930X_SC_RESET:
		mutex_lock(&dev->sc_lock);
		ret = it930x_sc_reset(dev);
		if (!ret) {
			memcpy(atr_data.atr, dev->sc_atr, dev->sc_atr_len);
			atr_data.len = dev->sc_atr_len;
		}
		mutex_unlock(&dev->sc_lock);
		if (ret)
			return ret;
		if (copy_to_user((void __user *)arg, &atr_data, sizeof(atr_data)))
			return -EFAULT;
		return 0;

	case IT930X_SC_GET_ATR:
		mutex_lock(&dev->sc_lock);
		memcpy(atr_data.atr, dev->sc_atr, dev->sc_atr_len);
		atr_data.len = dev->sc_atr_len;
		mutex_unlock(&dev->sc_lock);
		if (copy_to_user((void __user *)arg, &atr_data, sizeof(atr_data)))
			return -EFAULT;
		return 0;

	case IT930X_SC_SET_BAUDRATE:
		return it930x_sc_set_baudrate(dev, (int)arg);

	default:
		return -ENOTTY;
	}
}

static const struct file_operations it930x_sc_fops = {
	.owner		= THIS_MODULE,
	.open		= it930x_sc_open,
	.release	= it930x_sc_release,
	.read		= it930x_sc_fop_read,
	.write		= it930x_sc_fop_write,
	.unlocked_ioctl	= it930x_sc_ioctl,
};

/* -------- Frontend attach -------- */

static int it930x_frontend_attach(struct it930x_fe_ctx *ife)
{
	struct it930x_dev *dev = ife->dev;
	const struct it930x_board_cfg *board = dev->board;
	const struct it930x_fe_cfg *fc = &board->fe[ife->idx];
	struct cxd2878_config *cfg = &ife->demod_cfg;

	memset(cfg, 0, sizeof(*cfg));
	cfg->addr_slvt	= fc->demod_addr;
	cfg->xtal	= SONY_DEMOD_XTAL_24000KHz;
	cfg->tuner_addr	= fc->tuner_addr;
	cfg->ts_clk	= 1;
	cfg->ts_clk_mask = 1;
	cfg->lock_flag	= 1;

	cfg->tuner_xtal = board->tuner_xtal;

	if (board->gpio_lnb) {
		cfg->set_lnb	= it930x_set_lnb;
		cfg->fe_sat	= &ife->fe_sat;
	}

	ife->fe = dvb_attach(cxd2878_attach, cfg, ife->i2c);
	if (!ife->fe) {
		dev_err(&dev->intf->dev,
			"cxd2878 attach failed for frontend %d\n", ife->idx);
		return -ENODEV;
	}

	strscpy(ife->fe->ops.info.name, board->name,
		sizeof(ife->fe->ops.info.name));
	if (ife->fe_sat)
		strscpy(ife->fe_sat->ops.info.name, board->name,
			sizeof(ife->fe_sat->ops.info.name));

	return 0;
}

/* -------- DVB registration -------- */

static int it930x_dvb_init(struct it930x_dev *dev)
{
	const struct it930x_board_cfg *board = dev->board;
	int i, ret;

	for (i = 0; i < board->num_frontends; i++) {
		struct it930x_fe_ctx *ife = &dev->fes[i];
		const struct it930x_fe_cfg *fc = &board->fe[i];

		ife->dev = dev;
		ife->idx = i;
		ife->i2c = (fc->i2c_bus == 1) ? &dev->i2c[0] : &dev->i2c[1];

		ret = dvb_register_adapter(&ife->adapter, board->name,
					   THIS_MODULE, &dev->intf->dev,
					   adapter_nr);
		if (ret < 0) {
			dev_err(&dev->intf->dev,
				"dvb_register_adapter failed for fe %d (%d)\n",
				i, ret);
			goto err_unwind;
		}

		ret = it930x_frontend_attach(ife);
		if (ret)
			goto err_adapter;

		ret = dvb_register_frontend(&ife->adapter, ife->fe);
		if (ret) {
			dev_err(&dev->intf->dev,
				"dvb_register_frontend failed for fe %d (%d)\n",
				i, ret);
			dvb_frontend_detach(ife->fe);
			ife->fe = NULL;
			goto err_adapter;
		}

		if (ife->fe_sat) {
			ret = dvb_register_frontend(&ife->adapter, ife->fe_sat);
			if (ret) {
				dev_err(&dev->intf->dev,
					"dvb_register_frontend (sat) failed for fe %d (%d)\n",
					i, ret);
				dvb_frontend_detach(ife->fe_sat);
				ife->fe_sat = NULL;
				/* non-fatal: terrestrial still works */
			}
		}

		ife->demux.dmx.capabilities =
			DMX_TS_FILTERING | DMX_SECTION_FILTERING;
		ife->demux.priv = ife;
		ife->demux.filternum = 256;
		ife->demux.feednum = 256;
		ife->demux.start_feed = it930x_start_feed;
		ife->demux.stop_feed = it930x_stop_feed;
		ret = dvb_dmx_init(&ife->demux);
		if (ret)
			goto err_frontend;

		ife->dmxdev.filternum = 256;
		ife->dmxdev.demux = &ife->demux.dmx;
		ife->dmxdev.capabilities = 0;
		ret = dvb_dmxdev_init(&ife->dmxdev, &ife->adapter);
		if (ret)
			goto err_dmx;

		ret = dvb_net_init(&ife->adapter, &ife->dvbnet,
				   &ife->demux.dmx);
		if (ret)
			goto err_dmxdev;

		/* ATSC 3.0 ALP (non-fatal) */
		ife->alp.fe = ife->fe;
		ife->alp.dev = ife->adapter.device;
		ife->alp.start_streaming = it930x_alp_start;
		ife->alp.stop_streaming = it930x_alp_stop;
		ife->alp.priv = ife;
		atsc3_alp_register_netdev(&ife->alp);

		continue;

	err_dmxdev:
		dvb_dmxdev_release(&ife->dmxdev);
	err_dmx:
		dvb_dmx_release(&ife->demux);
	err_frontend:
		if (ife->fe_sat) {
			dvb_unregister_frontend(ife->fe_sat);
			dvb_frontend_detach(ife->fe_sat);
			ife->fe_sat = NULL;
		}
		dvb_unregister_frontend(ife->fe);
		dvb_frontend_detach(ife->fe);
		ife->fe = NULL;
	err_adapter:
		dvb_unregister_adapter(&ife->adapter);
		goto err_unwind;
	}

	ret = it930x_alloc_urbs(dev);
	if (ret)
		goto err_unwind;

	return 0;

err_unwind:
	/* Clean up already-registered frontends in reverse */
	while (--i >= 0) {
		struct it930x_fe_ctx *ife = &dev->fes[i];

		atsc3_alp_unregister_netdev(&ife->alp);
		dvb_net_release(&ife->dvbnet);
		dvb_dmxdev_release(&ife->dmxdev);
		dvb_dmx_release(&ife->demux);
		if (ife->fe_sat) {
			dvb_unregister_frontend(ife->fe_sat);
			dvb_frontend_detach(ife->fe_sat);
		}
		if (ife->fe) {
			dvb_unregister_frontend(ife->fe);
			dvb_frontend_detach(ife->fe);
		}
		dvb_unregister_adapter(&ife->adapter);
	}
	return ret;
}

static void it930x_dvb_exit(struct it930x_dev *dev)
{
	int i;

	it930x_stop_streaming(dev);
	it930x_free_urbs(dev);

	for (i = dev->board->num_frontends - 1; i >= 0; i--) {
		struct it930x_fe_ctx *ife = &dev->fes[i];

		atsc3_alp_unregister_netdev(&ife->alp);
		dvb_net_release(&ife->dvbnet);
		dvb_dmxdev_release(&ife->dmxdev);
		dvb_dmx_release(&ife->demux);
		if (ife->fe_sat) {
			dvb_unregister_frontend(ife->fe_sat);
			dvb_frontend_detach(ife->fe_sat);
		}
		if (ife->fe) {
			dvb_unregister_frontend(ife->fe);
			dvb_frontend_detach(ife->fe);
		}
		dvb_unregister_adapter(&ife->adapter);
	}
}

/* -------- USB probe/disconnect -------- */

static bool board_uses_bus1(const struct it930x_board_cfg *b)
{
	int i;

	for (i = 0; i < b->num_frontends; i++)
		if (b->fe[i].i2c_bus == 1)
			return true;
	return false;
}

static int it930x_probe(struct usb_interface *intf,
			const struct usb_device_id *id)
{
	const struct it930x_board_cfg *board;
	struct it930x_dev *dev;
	int ret;

	ret = it930x_validate_endpoints(intf);
	if (ret)
		return ret;

	board = (const struct it930x_board_cfg *)id->driver_info;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->udev = usb_get_dev(interface_to_usbdev(intf));
	dev->intf = intf;
	dev->board = board;
	mutex_init(&dev->io_lock);
	mutex_init(&dev->stream_lock);
	mutex_init(&dev->lnb_lock);

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

	/* Register I2C adapter for bus 3 (always present) */
	dev->i2c_ctx[1].dev = dev;
	dev->i2c_ctx[1].bus_num = 3;
	dev->i2c[1].owner = THIS_MODULE;
	dev->i2c[1].algo = &it930x_i2c_algo;
	dev->i2c[1].dev.parent = &intf->dev;
	snprintf(dev->i2c[1].name, sizeof(dev->i2c[1].name), "it930x-i2c-3");
	i2c_set_adapdata(&dev->i2c[1], &dev->i2c_ctx[1]);

	ret = i2c_add_adapter(&dev->i2c[1]);
	if (ret) {
		dev_err(&intf->dev, "i2c_add_adapter bus3 failed (%d)\n", ret);
		goto err_transport;
	}

	/* Register I2C adapter for bus 1 (only if needed) */
	if (board_uses_bus1(board)) {
		dev->i2c_ctx[0].dev = dev;
		dev->i2c_ctx[0].bus_num = 1;
		dev->i2c[0].owner = THIS_MODULE;
		dev->i2c[0].algo = &it930x_i2c_algo;
		dev->i2c[0].dev.parent = &intf->dev;
		snprintf(dev->i2c[0].name, sizeof(dev->i2c[0].name),
			 "it930x-i2c-1");
		i2c_set_adapdata(&dev->i2c[0], &dev->i2c_ctx[0]);

		ret = i2c_add_adapter(&dev->i2c[0]);
		if (ret) {
			dev_err(&intf->dev,
				"i2c_add_adapter bus1 failed (%d)\n", ret);
			goto err_i2c_bus3;
		}
	}

	/* Power on backends for PX-MLT boards */
	if (board->gpio_power) {
		mutex_lock(&dev->io_lock);
		ret = it930x_gpio_set(dev, board->gpio_power, false);
		mutex_unlock(&dev->io_lock);
		if (ret) {
			dev_err(&intf->dev, "backend power-on failed (%d)\n",
				ret);
			goto err_i2c;
		}
		msleep(80);

		mutex_lock(&dev->io_lock);
		it930x_gpio_pulse(dev, board->gpio_reset);
		mutex_unlock(&dev->io_lock);
		msleep(20);
	}

	/* Initialize DVB subsystem */
	ret = it930x_dvb_init(dev);
	if (ret)
		goto err_i2c;

	/* Initialize smartcard reader if present */
	if (board->gpio_sc_reset) {
		mutex_init(&dev->sc_lock);
		mutex_lock(&dev->io_lock);
		dev->sc_uart_type = it930x_sc_detect_type(dev);
		mutex_unlock(&dev->io_lock);
		if (dev->sc_uart_type) {
			it930x_sc_init_hw(dev);
			snprintf(dev->sc_name, sizeof(dev->sc_name),
				 "it930x_smartcard%d",
				 atomic_fetch_add(1, &sc_counter));
			dev->sc_misc.minor = MISC_DYNAMIC_MINOR;
			dev->sc_misc.name = dev->sc_name;
			dev->sc_misc.fops = &it930x_sc_fops;
			dev->sc_misc.parent = &intf->dev;
			ret = misc_register(&dev->sc_misc);
			if (ret) {
				dev_warn(&intf->dev,
					 "smartcard misc_register failed (%d)\n",
					 ret);
				dev->sc_uart_type = 0;
			}
		}
	}

	usb_set_intfdata(intf, dev);

	dev_info(&intf->dev, "%s attached (%d frontends)\n",
		 board->name, board->num_frontends);

	return 0;

err_i2c:
	if (board_uses_bus1(board))
		i2c_del_adapter(&dev->i2c[0]);
err_i2c_bus3:
	i2c_del_adapter(&dev->i2c[1]);
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

	if (dev->sc_uart_type)
		misc_deregister(&dev->sc_misc);

	it930x_dvb_exit(dev);

	if (board_uses_bus1(dev->board))
		i2c_del_adapter(&dev->i2c[0]);
	i2c_del_adapter(&dev->i2c[1]);

	it930x_transport_stop(dev);
	usb_put_dev(dev->udev);
	kfree(dev);

	dev_info(&intf->dev, "IT930x adapter detached\n");
}

/* -------- USB device ID table -------- */

static const struct usb_device_id it930x_id_table[] = {
	{ USB_DEVICE(0x23e2, 0x2b02), .driver_info = (kernel_ulong_t)
	  &(const struct it930x_board_cfg){	/* Zenview HDTV Mate */
		.name		= "Zenview HDTV Mate",
		.fw_file	= "dvb-usb-it9306-01.fw",
		.num_frontends	= 1,
		.fe		= {
			{ .ts_port = 0, .i2c_bus = 3,
			  .demod_addr = 0x6c, .tuner_addr = 0x60 },
		},
		.gpio_reset	= 2,
		.gpio_always_hi	= 14,
		.f41a_val	= 0x05,
		.i2c_notify	= 0x38,
		.tuner_xtal	= SONY_ASCOT3_XTAL_24000KHz,
	  }
	},
	{ USB_DEVICE(0x0511, 0x084e), .driver_info = (kernel_ulong_t)
	  &(const struct it930x_board_cfg){	/* PLEX PX-MLT5U */
		.name		= "PLEX PX-MLT5U",
		.fw_file	= "it930x-firmware.bin",
		.num_frontends	= 5,
		.fe = {
			{ .ts_port = 4, .i2c_bus = 3,
			  .demod_addr = 0x65, .tuner_addr = 0x60 },
			{ .ts_port = 3, .i2c_bus = 1,
			  .demod_addr = 0x6c, .tuner_addr = 0x60 },
			{ .ts_port = 1, .i2c_bus = 1,
			  .demod_addr = 0x64, .tuner_addr = 0x60 },
			{ .ts_port = 2, .i2c_bus = 3,
			  .demod_addr = 0x6c, .tuner_addr = 0x60 },
			{ .ts_port = 0, .i2c_bus = 3,
			  .demod_addr = 0x64, .tuner_addr = 0x60 },
		},
		.gpio_power	= 7,
		.gpio_reset	= 2,
		.gpio_lnb	= 11,
		.f41a_val	= 0x01,
		.tuner_xtal	= SONY_ASCOT3_XTAL_16000KHz,
		.gpio_sc_reset	= 14,
	  }
	},
	{ USB_DEVICE(0x0511, 0x024e), .driver_info = (kernel_ulong_t)
	  &(const struct it930x_board_cfg){	/* PLEX PX-MLT5PE */
		.name		= "PLEX PX-MLT5PE",
		.fw_file	= "it930x-firmware.bin",
		.num_frontends	= 5,
		.fe = {
			{ .ts_port = 0, .i2c_bus = 3,
			  .demod_addr = 0x65, .tuner_addr = 0x60 },
			{ .ts_port = 1, .i2c_bus = 1,
			  .demod_addr = 0x6c, .tuner_addr = 0x60 },
			{ .ts_port = 2, .i2c_bus = 1,
			  .demod_addr = 0x64, .tuner_addr = 0x60 },
			{ .ts_port = 3, .i2c_bus = 3,
			  .demod_addr = 0x6c, .tuner_addr = 0x60 },
			{ .ts_port = 4, .i2c_bus = 3,
			  .demod_addr = 0x64, .tuner_addr = 0x60 },
		},
		.gpio_power	= 7,
		.gpio_reset	= 2,
		.gpio_lnb	= 11,
		.f41a_val	= 0x01,
		.tuner_xtal	= SONY_ASCOT3_XTAL_16000KHz,
		.gpio_sc_reset	= 14,
	  }
	},
	{ USB_DEVICE(0x0511, 0x0252), .driver_info = (kernel_ulong_t)
	  &(const struct it930x_board_cfg){	/* PLEX PX-MLT8PE3 */
		.name		= "PLEX PX-MLT8PE3",
		.fw_file	= "it930x-firmware.bin",
		.num_frontends	= 3,
		.fe = {
			{ .ts_port = 0, .i2c_bus = 3,
			  .demod_addr = 0x65, .tuner_addr = 0x60 },
			{ .ts_port = 3, .i2c_bus = 3,
			  .demod_addr = 0x6c, .tuner_addr = 0x60 },
			{ .ts_port = 4, .i2c_bus = 3,
			  .demod_addr = 0x64, .tuner_addr = 0x60 },
		},
		.gpio_power	= 7,
		.gpio_reset	= 2,
		.gpio_lnb	= 11,
		.f41a_val	= 0x01,
		.tuner_xtal	= SONY_ASCOT3_XTAL_16000KHz,
		.gpio_sc_reset	= 14,
	  }
	},
	{ USB_DEVICE(0x0511, 0x0253), .driver_info = (kernel_ulong_t)
	  &(const struct it930x_board_cfg){	/* PLEX PX-MLT8PE5 */
		.name		= "PLEX PX-MLT8PE5",
		.fw_file	= "it930x-firmware.bin",
		.num_frontends	= 5,
		.fe = {
			{ .ts_port = 0, .i2c_bus = 1,
			  .demod_addr = 0x65, .tuner_addr = 0x60 },
			{ .ts_port = 1, .i2c_bus = 1,
			  .demod_addr = 0x64, .tuner_addr = 0x60 },
			{ .ts_port = 2, .i2c_bus = 1,
			  .demod_addr = 0x6c, .tuner_addr = 0x60 },
			{ .ts_port = 3, .i2c_bus = 3,
			  .demod_addr = 0x6c, .tuner_addr = 0x60 },
			{ .ts_port = 4, .i2c_bus = 3,
			  .demod_addr = 0x64, .tuner_addr = 0x60 },
		},
		.gpio_power	= 7,
		.gpio_reset	= 2,
		.gpio_lnb	= 11,
		.f41a_val	= 0x01,
		.tuner_xtal	= SONY_ASCOT3_XTAL_16000KHz,
		.gpio_sc_reset	= 14,
	  }
	},
	{ USB_DEVICE(0x0511, 0x0254), .driver_info = (kernel_ulong_t)
	  &(const struct it930x_board_cfg){	/* Digibest ISDB6014 4TS */
		.name		= "Digibest ISDB6014 4TS",
		.fw_file	= "it930x-firmware.bin",
		.num_frontends	= 4,
		.fe = {
			{ .ts_port = 0, .i2c_bus = 3,
			  .demod_addr = 0x65, .tuner_addr = 0x60 },
			{ .ts_port = 1, .i2c_bus = 1,
			  .demod_addr = 0x6c, .tuner_addr = 0x60 },
			{ .ts_port = 2, .i2c_bus = 1,
			  .demod_addr = 0x64, .tuner_addr = 0x60 },
			{ .ts_port = 4, .i2c_bus = 3,
			  .demod_addr = 0x64, .tuner_addr = 0x60 },
		},
		.gpio_power	= 7,
		.gpio_reset	= 2,
		.gpio_lnb	= 11,
		.f41a_val	= 0x01,
		.tuner_xtal	= SONY_ASCOT3_XTAL_16000KHz,
		.gpio_sc_reset	= 14,
	  }
	},
	{ }
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

MODULE_DESCRIPTION("ITE IT930x USB bridge driver");
MODULE_AUTHOR("koreapyj");
MODULE_LICENSE("GPL");
