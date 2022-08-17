/*
 * Copyright (c) 2022 openEuler Embedded
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/ipc/rpmsg_service.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uart_rpmsg, CONFIG_UART_LOG_LEVEL);


struct uart_rpmsg_data {
	const struct device *dev;

	int ep_id; /* endpoint id */

	/** Ring buffer for received bytes from rpmsg */
	struct ring_buf rb;
	uint8_t ring_buf_data[CONFIG_UART_RPMSG_RING_BUF_SIZE];

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	uart_irq_callback_user_data_t irq_cb;
	void *irq_cb_data;
	int tx_busy;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

static struct uart_rpmsg_data uart_data;

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
static void uart_rpmsg_cb(struct uart_rpmsg_data *data)
{
	if (data->irq_cb) {
		/* use data->tx_busy to simulate the tx empty interrupt
		 * uart_tx_enable will call uart_rpmsg_cb, and set tx_busy to 1
		 * uart_tx_disable will set tx_buys to 0, so uart_tx_disable 
		 * must be called if nothing to send and the loop will end
		 */
		do {
			data->irq_cb(data->dev, data->irq_cb_data);
		} while (data->tx_busy);
	}
}
#endif

/* called in thread context */
static int uart_rpmsg_endpoint_cb(struct rpmsg_endpoint *ept, void *data,
		size_t len, uint32_t src, void *priv)
{
	struct uart_rpmsg_data *uart_data = (struct uart_rpmsg_data *)priv;
	int wrote;

	if (rpmsg_service_endpoint_is_bound(uart_data->ep_id) == false) {
		rpmsg_service_endpoint_bound(uart_data->ep_id);
	}

	printk("received data:%d\r\n", len);
	/* put the received data into ring buf */
	wrote = ring_buf_put(&uart_data->rb, data, len);

	if (wrote < len) {
		LOG_INF("ring buf is full, data overrun\n");
		/* ring buf is full, disable rx? */
	}

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	uart_rpmsg_cb(priv);
#endif

	return RPMSG_SUCCESS;
}

static int uart_rpmsg_poll_in(const struct device *dev,
			unsigned char *c)
{
	struct uart_rpmsg_data *data = (struct uart_rpmsg_data *)dev->data;
	int ret = 0;
	char temp;

	/* wait until ring buffer is not empty */
	while (ring_buf_is_empty(&data->rb)) {
		k_msleep(1);
	}

	ret = ring_buf_get(&data->rb, &temp, sizeof(temp));

	if (ret != 1) {
		/* char was not received correctly */
		return -1;
	}

	*c = temp;

	return 0;
}

static void uart_rpmsg_poll_out(const struct device *dev, unsigned char c)
{
	struct uart_rpmsg_data *uart_data = (struct uart_rpmsg_data *)dev->data;

	/* should wait until uart_rpmsg's endpoint is bound ?
	 * if not bound, rpmsg_service_send will fail
	 * in thread context, we can sleep to wait
	 * in no-thread context, we should fail and lose msg
	 */

	while (rpmsg_service_endpoint_is_bound(uart_data->ep_id) == false) {
		k_msleep(10);
	}

	rpmsg_service_send(uart_data->ep_id, &c, sizeof(c));
}

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)

static int uart_rpmsg_fifo_fill(const struct device *dev, const uint8_t *tx_data,
			 int len)
{
	struct uart_rpmsg_data *data = (struct uart_rpmsg_data *)dev->data;
	int ret = 0, sent = 0;

	while (len) {
		sent = rpmsg_service_send(data->ep_id, tx_data, len);
		/* < 0, send error */
		if (sent < 0) {
			ret = sent;
			break;
		}
		ret += sent;
		tx_data += sent;
		len -= sent;
	}

	return ret;
}

static int uart_rpmsg_fifo_read(const struct device *dev, uint8_t *rx_data,
			 const int size)
{
	struct uart_rpmsg_data *data = (struct uart_rpmsg_data *)dev->data;

	return ring_buf_get(&data->rb, rx_data, size);
}

static void uart_rpmsg_irq_tx_enable(const struct device *dev)
{
/* explicitly call UART callback to simulate Tx interrupt */
	struct uart_rpmsg_data *data = (struct uart_rpmsg_data *)dev->data;

	/* should wait until uart_rpmsg's endpoint is bound ?
	 * if not bound, rpmsg_service_send will fail
	 * in thread context, we can sleep to wait
	 * in no-thread context, we should fail and lose msg
	 */

	if (rpmsg_service_endpoint_is_bound(data->ep_id)) {
		data->tx_busy = 1;
		uart_rpmsg_cb(data);
	}
}

static void uart_rpmsg_irq_tx_disable(const struct device *dev)
{
	struct uart_rpmsg_data *data = (struct uart_rpmsg_data *)dev->data;

	data->tx_busy = 0;
}


static int uart_rpmsg_irq_tx_ready(const struct device *dev)
{
	/* always ready to tx */
	return 1;
}

static void uart_rpmsg_irq_rx_enable(const struct device *dev)
{
	/* always rx enable */
}

static int uart_rpmsg_irq_tx_complete(const struct device *dev)
{
	/*
	 * after send, it's always complete
	 */
	return 1;
}

static int uart_rpmsg_irq_rx_ready(const struct device *dev)
{
	/* if ring buf is not empty, then ready to rx */
	struct uart_rpmsg_data *data = (struct uart_rpmsg_data *)dev->data;

	return  !ring_buf_is_empty(&data->rb);
}

static int uart_rpmsg_irq_is_pending(const struct device *dev)
{
	return uart_rpmsg_irq_rx_ready(dev);
}

static int uart_rpmsg_irq_update(const struct device *dev)
{
	/* Nothing needs to be updated */
	return 1;
}

static void uart_rpmsg_irq_callback_set(const struct device *dev,
		 uart_irq_callback_user_data_t cb, void *user_data)
{
	struct uart_rpmsg_data *data = (struct uart_rpmsg_data *)dev->data;

	data->irq_cb = cb;
	data->irq_cb_data = user_data;
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_rpmsg_api = {
	.poll_in = uart_rpmsg_poll_in,
	.poll_out = uart_rpmsg_poll_out,
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	.fifo_fill = uart_rpmsg_fifo_fill,
	.fifo_read = uart_rpmsg_fifo_read,
	.irq_tx_enable = uart_rpmsg_irq_tx_enable,
	.irq_tx_disable = uart_rpmsg_irq_tx_disable,
	.irq_tx_ready = uart_rpmsg_irq_tx_ready,
	.irq_rx_enable = uart_rpmsg_irq_rx_enable,
	.irq_tx_complete = uart_rpmsg_irq_tx_complete,
	.irq_rx_ready = uart_rpmsg_irq_rx_ready,
	.irq_is_pending = uart_rpmsg_irq_is_pending,
	.irq_update = uart_rpmsg_irq_update,
	.irq_callback_set = uart_rpmsg_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

int uart_rpmsg_init(const struct device *dev)
{
	struct uart_rpmsg_data *data = dev->data;
	int status;

	status = rpmsg_service_register_endpoint("uart", uart_rpmsg_endpoint_cb, data);

	if (status < 0) {
		LOG_INF("RPMSG UART initialized error\n");
		return status;
	}

	data->dev = dev;
	data->ep_id = status;
	ring_buf_init(&data->rb, CONFIG_UART_RPMSG_RING_BUF_SIZE, data->ring_buf_data);
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)

	data->tx_busy = 0;

#endif

	LOG_INF("IPC UART initialized successfully\n");

	return 0;
}

DEVICE_DT_DEFINE(DT_NODELABEL(uart_rpmsg), uart_rpmsg_init, NULL, &uart_data,
		NULL, POST_KERNEL, CONFIG_RPMSG_SERVICE_EP_REG_PRIORITY,
		&uart_rpmsg_api);
