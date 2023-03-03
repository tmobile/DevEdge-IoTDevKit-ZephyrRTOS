/*
 * Copyright (c) 2022 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr/logging/log.h>
#include "tsl2540.h"
LOG_MODULE_DECLARE(tsl2540);

static void tsl2540_handle_cb(struct tsl2540_data *data)
{
	const struct tsl2540_config *config = data->dev->config;

	gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_DISABLE);

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
	const struct tsl2540_config *config = data->dev->config;

	if ((pin_mask & BIT(config->int_gpio.pin)) == 0U) {
		return;
	}

	tsl2540_handle_cb(data);
}


static void tsl2540_handle_int(const struct device *dev)
{
	struct tsl2540_data *data = dev->data;
	const struct tsl2540_config *config = dev->config;
	uint8_t status;

	k_sem_take(&data->sem, K_FOREVER);
#if 1
	errno = 0;
	status = fetch_STATUS(dev);
	if (errno) {
		LOG_ERR("Could not read status register, errno: %d", errno);
	}
#else
	if (tsl2540_reg_read(dev, TSL2540_REG_STATUS, &status)) {
		LOG_ERR("Could not read status");
	}
#endif

	k_sem_give(&data->sem);

	struct sensor_trigger als_trig = {
		.type = SENSOR_TRIG_THRESHOLD,
		.chan = SENSOR_CHAN_LIGHT,
	};

	if (data->als_handler) {
		data->als_handler(dev, &als_trig);
	}

	gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
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
#if 1
			conf = get_INTENAB(dev);
#else
			if (tsl2540_reg_read(dev, TSL2540_REG_INT_EN, &conf)) {
				ret = -EIO;
				goto exit;
			}
#endif
			/* Set interrupt enable bit */
			conf |= TSL2540_INT_EN_AEN;

#if 1
			set_INTENAB(dev, conf);
#else
			if (tsl2540_reg_write(dev, TSL2540_REG_INT_EN, conf)) {
				ret = -EIO;
				goto exit;
			}
#endif
#if 1
			conf = get_CFG3(dev);
#else
			if (tsl2540_reg_read(dev, TSL2540_REG_CFG_3, &conf)) {
				ret = -EIO;
				goto exit;
			}
#endif
			conf |= TSL2540_CFG3_SAI | TSL2540_CFG3_INTRC;
#if 1
			set_CFG3(dev, conf);
#else
			if (tsl2540_reg_write(dev, TSL2540_REG_CFG_3, conf)) {
				ret = -EIO;
				goto exit;
			}
#endif

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
	flush_all(dev);

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

	gpio_pin_configure_dt(&config->int_gpio,  GPIO_INPUT | config->int_gpio.dt_flags);

	gpio_init_callback(&data->gpio_cb, tsl2540_gpio_callback,
			   BIT(config->int_gpio.pin));

	if (gpio_add_callback(config->int_gpio.port, &data->gpio_cb) < 0) {
		LOG_DBG("Failed to set gpio callback!");
		return -EIO;
	}

	gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);

	if (gpio_pin_get_dt(&config->int_gpio) > 0) {
		tsl2540_handle_cb(data);
	}

	set_ENABLE(dev, TSL2540_EN_ACTIVE);
	// flush_all(dev);

	return 0;
}
