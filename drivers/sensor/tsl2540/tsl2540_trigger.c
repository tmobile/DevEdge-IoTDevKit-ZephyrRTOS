/*
 * Copyright (c) 2022 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tsl2540.h"
#include <logging/log.h>

LOG_MODULE_DECLARE(tsl2540, CONFIG_SENSOR_LOG_LEVEL);

static void tsl2540_handle_cb(struct tsl2540_data *data)
{
	gpio_pin_interrupt_configure(data->gpio, data->gpio_pin,
				     GPIO_INT_DISABLE);

#if defined(CONFIG_TSL2540_TRIGGER_OWN_THREAD)
	k_sem_give(&data->trig_sem);
#elif defined(CONFIG_TSL2540_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&data->work);
#endif
}

static void tsl2540_gpio_callback(const struct device *dev,
				   struct gpio_callback *cb,
				   uint32_t pin_mask)
{
	struct tsl2540_data *data =
		CONTAINER_OF(cb, struct tsl2540_data, gpio_cb);

	if ((pin_mask & BIT(data->gpio_pin)) == 0U) {
		return;
	}

	tsl2540_handle_cb(data);
}


static void tsl2540_handle_int(const struct device *dev)
{
	struct tsl2540_data *data = dev->data;
	uint8_t status;

	k_sem_take(&data->sem, K_FOREVER);

	if (tsl2540_read(dev, TSL2540_REG_STATUS, &status)) {
		LOG_ERR("Could not read status");
	}

	k_sem_give(&data->sem);

	struct sensor_trigger als_trig = {
		.type = SENSOR_TRIG_THRESHOLD,
		.chan = SENSOR_CHAN_LIGHT,
	};

	if (data->als_handler) {
		data->als_handler(dev, &als_trig);
	}

	gpio_pin_interrupt_configure(data->gpio, data->gpio_pin,
				     GPIO_INT_EDGE_TO_ACTIVE);
}

#ifdef CONFIG_TSL2540_TRIGGER_OWN_THREAD
static void tsl2540_thread_main(struct tsl2540_data *data)
{
	while (true) {
		k_sem_take(&data->trig_sem, K_FOREVER);
		tsl2540_handle_int(data->dev);
	}
}
#endif

#ifdef CONFIG_TSL2540_TRIGGER_GLOBAL_THREAD
static void tsl2540_work_handler(struct k_work *work)
{
	struct tsl2540_data *data =
		CONTAINER_OF(work, struct tsl2540_data, work);

	tsl2540_handle_int(data->dev);
}
#endif

int tsl2540_trigger_set(const struct device *dev,
			 const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler)
{
	struct tsl2540_data *data = dev->data;
	uint8_t conf;
	int ret = 0;

	k_sem_take(&data->sem, K_FOREVER);

	switch (trig->type) {
	case SENSOR_TRIG_THRESHOLD:
		if (trig->chan == SENSOR_CHAN_LIGHT) {
			if (tsl2540_reg_read(dev, TSL2540_REG_INT_EN, &conf)) {
				ret = -EIO;
				goto exit;
			}

			/* Set interrupt enable bit */
			conf |= TSL2540_INT_EN_AEN;

			if (tsl2540_reg_write(dev, TSL2540_REG_INT_EN, conf)) {
				ret = -EIO;
				goto exit;
			}

			data->als_handler = handler;
		} else {
			ret = -ENOTSUP;
			goto exit;
		}
		break;
	default:
		LOG_ERR("Unsupported sensor trigger");
		ret = -ENOTSUP;
		goto exit;
	}
exit:
	k_sem_give(&data->sem);

	return ret;
}

int tsl2540_trigger_init(const struct device *dev)
{
	const struct tsl2540_config *config = dev->config;
	struct tsl2540_data *data = dev->data;

	data->dev = dev;

#if defined(CONFIG_TSL2540_TRIGGER_OWN_THREAD)
	k_sem_init(&data->trig_sem, 0, K_SEM_MAX_LIMIT);
	k_thread_create(&data->thread, data->thread_stack,
			CONFIG_TSL2540_THREAD_STACK_SIZE,
			(k_thread_entry_t)tsl2540_thread_main,
			data, NULL, NULL,
			K_PRIO_COOP(CONFIG_TSL2540_THREAD_PRIORITY),
			0, K_NO_WAIT);
	k_thread_name_set(&data->thread, "TSL2540 trigger");
#elif defined(CONFIG_TSL2540_TRIGGER_GLOBAL_THREAD)
	data->work.handler = tsl2540_work_handler;
#endif

	/* Get the GPIO device */
	if (!device_is_ready(config->int_gpio.port)) {
		LOG_ERR("tsl2540: gpio controller %s not ready",
			config->int_gpio.port->name);
		return -ENODEV;
	}

	data->gpio_pin = config->int_gpio.pin;

	gpio_pin_configure(data->gpio, data->gpio_pin,
			   GPIO_INPUT | config->int_gpio.dt_flags);

	gpio_init_callback(&data->gpio_cb, tsl2540_gpio_callback,
			   BIT(data->gpio_pin));

	if (gpio_add_callback(data->gpio, &data->gpio_cb) < 0) {
		LOG_DBG("Failed to set gpio callback!");
		return -EIO;
	}

	gpio_pin_interrupt_configure(data->gpio, data->gpio_pin,
				     GPIO_INT_EDGE_TO_ACTIVE);

	if (gpio_pin_get(data->gpio, data->gpio_pin) > 0) {
		tsl2540_handle_cb(data);
	}

	return 0;
}
