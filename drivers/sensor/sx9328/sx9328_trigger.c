/*
 * Copyright (c) 2023 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/sx9328.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>

#include "sx9328.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(SX9328, CONFIG_SENSOR_LOG_LEVEL);

#ifdef CONFIG_SX9328_TRIGGER_OWN_THREAD
static K_KERNEL_STACK_DEFINE(sx9328_thread_stack, CONFIG_SX9328_THREAD_STACK_SIZE);
static struct k_thread sx9328_thread;
#endif

int sx9328_trigger_set(const struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler)
{
	struct sx9328_data *data = dev->data;
	const struct sx9328_config *cfg = dev->config;

	if (!cfg->int_gpio.port) {
		return -ENOTSUP;
	}

	switch ((uint16_t)trig->type) {
	case SENSOR_TRIG_DATA_READY:
		if (i2c_reg_update_byte_dt(&cfg->i2c,
					   SX9328_IRQ_MSK_REG,
					   SX9328_CONV_DONE_IRQ,
					   SX9328_CONV_DONE_IRQ) < 0) {
			return -EIO;
		}
		data->handler_drdy = handler;
		data->trigger_drdy = *trig;
		break;

	case SENSOR_TRIG_NEAR_FAR:
		if (i2c_reg_update_byte_dt(&cfg->i2c,
					   SX9328_IRQ_MSK_REG,
					   SX9328_NEAR_FAR_IRQ,
					   SX9328_NEAR_FAR_IRQ) < 0) {
			return -EIO;
		}
		data->handler_near_far = handler;
		data->trigger_near_far = *trig;
		break;

	case SENSOR_TRIG_SX9328_NEAR_FAR_TBL_BDY:
		if (i2c_reg_update_byte_dt(&cfg->i2c,
					   SX9328_IRQ_MSK_REG,
					   SX9328_NEAR_FAR_TBL_BDY_IRQ,
					   SX9328_NEAR_FAR_TBL_BDY_IRQ) < 0) {
			return -EIO;
		}
		data->handler_near_far_tb = handler;
		data->trigger_near_far_tb = *trig;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static void sx9328_gpio_thread_cb(const struct device *dev)
{
	struct sx9328_data *data = dev->data;
	const struct sx9328_config *cfg = dev->config;
	uint8_t reg_val;

	if (i2c_reg_read_byte_dt(&cfg->i2c, SX9328_IRQ_SRC_REG, &reg_val) < 0) {
		return;
	}

	if ((reg_val & SX9328_CONV_DONE_IRQ) && data->handler_drdy) {
		data->handler_drdy(dev, &data->trigger_drdy);
	}

	if ((reg_val & SX9328_NEAR_FAR_IRQ) && data->handler_near_far) {
		data->handler_near_far(dev, &data->trigger_near_far);
	}

	if ((reg_val & SX9328_NEAR_FAR_TBL_BDY_IRQ) && data->handler_near_far_tb) {
		data->handler_near_far_tb(dev, &data->trigger_near_far_tb);
	}
}

#ifdef CONFIG_SX9328_TRIGGER_OWN_THREAD

static void sx9328_gpio_cb(const struct device *port,
			   struct gpio_callback *cb, uint32_t pins)
{
	struct sx9328_data *data =
		CONTAINER_OF(cb, struct sx9328_data, gpio_cb);

	ARG_UNUSED(pins);

	k_sem_give(&data->sem);
}

static void sx9328_thread_main(struct sx9328_data *data)
{
	while (1) {
		k_sem_take(&data->sem, K_FOREVER);
		sx9328_gpio_thread_cb(data->dev);
	}
}

#else /* CONFIG_SX9328_TRIGGER_GLOBAL_THREAD */

static void sx9328_gpio_cb(const struct device *port,
			   struct gpio_callback *cb, uint32_t pins)
{
	struct sx9328_data *data =
		CONTAINER_OF(cb, struct sx9328_data, gpio_cb);

	ARG_UNUSED(pins);

	k_work_submit(&data->work);
}
#endif /* CONFIG_SX9328_TRIGGER_GLOBAL_THREAD */

#ifdef CONFIG_SX9328_TRIGGER_GLOBAL_THREAD
static void sx9328_work_cb(struct k_work *work)
{
	struct sx9328_data *data =
		CONTAINER_OF(work, struct sx9328_data, work);

	sx9328_gpio_thread_cb(data->dev);
}
#endif

int sx9328_setup_interrupt(const struct device *dev)
{
	struct sx9328_data *data = dev->data;
	const struct sx9328_config *cfg = dev->config;
	int ret;

#ifdef CONFIG_SX9328_TRIGGER_OWN_THREAD
	k_sem_init(&data->sem, 0, K_SEM_MAX_LIMIT);
#else
	data->work.handler = sx9328_work_cb;
#endif

	data->dev = dev;

	if (!device_is_ready(cfg->int_gpio.port)) {
		LOG_ERR("%s: device %s is not ready", dev->name,
			cfg->int_gpio.port->name);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
	if (ret < 0) {
		return ret;
	}

	gpio_init_callback(&data->gpio_cb, sx9328_gpio_cb, BIT(cfg->int_gpio.pin));

	ret = gpio_add_callback(cfg->int_gpio.port, &data->gpio_cb);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		return ret;
	}

#ifdef CONFIG_SX9328_TRIGGER_OWN_THREAD
	k_thread_create(&sx9328_thread, sx9328_thread_stack,
			CONFIG_SX9328_THREAD_STACK_SIZE,
			(k_thread_entry_t)sx9328_thread_main, data, 0, NULL,
			K_PRIO_COOP(CONFIG_SX9328_THREAD_PRIORITY),
			0, K_NO_WAIT);
#endif

	return 0;
}
