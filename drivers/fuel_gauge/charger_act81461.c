/*
 * Copyright (c) 2023 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT qorvo_act81461_alpc

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/init.h>
#include <errno.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/slist.h>

#include <zephyr/drivers/fuel_gauge.h>
#include <zephyr/drivers/fuel_gauge/act81461.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include "charger_act81461.h"

struct act81461_gauge_config {
	const struct gpio_dt_spec alert_gpio;
	struct i2c_dt_spec i2c;
};

struct act81461_gauge_data {
	const struct device *dev;
	struct gpio_callback gpio_int_cb;
	struct k_work interrupt_work;
	sys_slist_t cbs;
};

LOG_MODULE_REGISTER(act81461_charger, CONFIG_REGULATOR_LOG_LEVEL);

static void interrupt_work_fn(struct k_work *item)
{
	struct act81461_gauge_data *data =
		CONTAINER_OF(item, struct act81461_gauge_data, interrupt_work);

	struct act81461_int_cb *cb, *next;

	SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&data->cbs, cb, next, node) {
		if (cb->cb) {
			cb->cb(data->dev);
		}
	}
}

static void act81461_battery_charge_intr_callback(const struct device *port,
						  struct gpio_callback *cb, uint32_t pins)
{
	struct act81461_gauge_data *data =
		CONTAINER_OF(cb, struct act81461_gauge_data, gpio_int_cb);
	k_work_submit(&data->interrupt_work);
	LOG_DBG("pmic battery charger status change Interrupt");
}

#ifdef CONFIG_REGULATOR_ACT81461
/**
 * @brief External callback for when the regulator driver is also enabled
 *        so that general interrupts can be propogated.
 *
 * @param dev ALPC device pointer
 */
void act81461_battery_charge_intr_ext_callback(const struct device *dev)
{
	struct act81461_gauge_data *data = dev->data;

	if (data->interrupt_work.handler) {
		k_work_submit(&data->interrupt_work);
	}
	LOG_DBG("pmic battery charger status change Interrupt");
}
#endif


static int act81461_charger_get_prop(const struct device *dev, struct fuel_gauge_get_property *prop)
{
	int rc = 0;
	uint8_t val = 0;
	const struct act81461_gauge_config *config = dev->config;

	switch (prop->property_type) {
	case FUEL_GAUGE_TEMPERATURE:
		rc = i2c_reg_read_byte_dt(&config->i2c, CHG_STAT_REG, &val);
		if (rc) {
			break;
		}
		val &= BAT_TEMP_MSK;
		switch (val) {
		case 0 << BAT_TEMP_SHIFT: /* Below 0 (C) */
			prop->value.temperature = DEG_C(-5);
		case 1 << BAT_TEMP_SHIFT: /* 0 to 10 (C) */
			prop->value.temperature = DEG_C(5);
		case 2 << BAT_TEMP_SHIFT: /* 10 to 45 (C) */
			prop->value.temperature = DEG_C(25);
		case 3 << BAT_TEMP_SHIFT: /* 45 to 50 (C) */
			prop->value.temperature = DEG_C(45);
		case 4 << BAT_TEMP_SHIFT: /* 50 to 60 (C) */
			prop->value.temperature = DEG_C(55);
		case 5 << BAT_TEMP_SHIFT: /* Over 60 (C) */
			prop->value.temperature = DEG_C(65);
		default:
			rc = -EIO;
		}
		break;
	case FUEL_GAUGE_STATUS:
		prop->value.fg_status = 0;
		rc = i2c_reg_read_byte_dt(&config->i2c, CHG_STAT_REG, &val);
		if (rc) {
			break;
		}

		uint8_t chg_stat = val & CHG_STAT_MSK;

		switch (chg_stat) {
		case 0b000: /* Exit Charge */
			LOG_DBG("Charger has exited charge mode");
			prop->value.fg_status |= FUEL_GAUGE_STATUS_FLAGS_DISCHARGING;
			break;

		case 0b110: /* End of Charge Mode */
			LOG_DBG("Battery end of charge has been detected");
			prop->value.fg_status |= FUEL_GAUGE_STATUS_FLAGS_FULLY_CHARGED;
			break;

		case 0b111: /* Fault Mode */
			LOG_DBG("Battery fault detected");
			/* TODO: Add timer
			 * LOG_ERR("Battery fault has been detected for %d
			 * minutes",					pmic_battery_fault_count *
			 * NO_RECOVER_MSG_INTERVAL_SECONDS /
			 * 60);
			 * LOG_ERR("Disconnect and reattach battery to
			 * clear this fault");
			 */
			prop->value.fg_status |= FUEL_GAUGE_STATUS_FLAGS_TERMINATE_CHARGE_ALARM;
			break;

		case 0b010: /* VBAT SHORT Mode */
			LOG_DBG("Charger is in VBAT SHORT mode");
			prop->value.fg_status |= FUEL_GAUGE_STATUS_FLAGS_TERMINATE_CHARGE_ALARM;
			break;

		case 0b001: /* Reset Mode */
			LOG_DBG("Charger is in reset mode");
			break;

		case 0b011: /* VBAT PRECOND Mode */
			LOG_DBG("Charger is in VBAT PRECOND mode");
			break;

		case 0b100: /* Fast Charge CC loop on */
		case 0b101: /* Fast Charge CC loop off & CV loop on */
			LOG_DBG("Battery is in FAST charge mode - charging");
		default:
			break;
		}

		rc = i2c_reg_read_byte_dt(&config->i2c, BAT_STAT_REG, &val);

		if (rc) {
			break;
		}

		if (val & SUSPEND) {
			prop->value.fg_status |= FUEL_GAUGE_STATUS_FLAGS_OVER_TEMP_ALARM;
		}

		break;

	case FUEL_GAUGE_PRESENT_STATE:
		rc = i2c_reg_read_byte_dt(&config->i2c, BAT_STAT_REG, &val);

		/* This is necessary as battery detection doesn't work without VBUS */
		if ((val & VINOK)) {
			rc = i2c_reg_read_byte_dt(&config->i2c, CHG_STAT_REG, &val);
			prop->value.flags = (val & BATTERY_DETECTED) ? 1 : 0;
		} else {
			/* We assume we have a battery if the chip is on without VBUS */
			prop->value.flags = 1;
		}
		break;

	case FUEL_GAUGE_CONNECT_STATE:
		rc = i2c_reg_read_byte_dt(&config->i2c, BAT_STAT_REG, &val);

		prop->value.flags = (val & VINOK) ? 1 : 0;
		break;

	/*
	 * FIXME: Determine standardized mode values
	 *
	 * IDK if this is the best way to handle this, but this is our mode and
	 * there is no stanard nor spec for this property
	 */
	case FUEL_GAUGE_SBS_MODE:
		rc = i2c_reg_read_byte_dt(&config->i2c, BAT_STAT_REG, &val);

		prop->value.sbs_mode = val & CHG_STAT_MSK;
		break;

	/* TODO: Support reading charge current/voltage settings */
	case FUEL_GAUGE_CHARGE_CURRENT:
	case FUEL_GAUGE_CHARGE_VOLTAGE:
	default:
		rc = -ENOTSUP;
	}

	prop->status = rc;

	return rc;
}

static int act81461_charger_get_props(const struct device *dev,
				      struct fuel_gauge_get_property *props, size_t len)
{
	int err_count = 0;

	for (int i = 0; i < len; i++) {
		int ret = act81461_charger_get_prop(dev, props + i);

		err_count += ret ? 1 : 0;
	}

	err_count = (err_count == len) ? -1 : err_count;

	return err_count;
}

int act81461_charger_int_cb_register(const struct device *dev, struct act81461_int_cb *cb)
{
	struct act81461_gauge_data *data = dev->data;

	if (cb == NULL) {
		return -EINVAL;
	}

	sys_slist_append(&data->cbs, &cb->node);

	return 0;
}

int act81461_charger_int_cb_unregister(const struct device *dev, struct act81461_int_cb *cb)
{
	struct act81461_gauge_data *data = dev->data;

	if (cb == NULL) {
		return -EINVAL;
	}

	if (!sys_slist_find_and_remove(&data->cbs, &cb->node)) {
		return -EALREADY;
	}

	return 0;
}

/* TODO: Add support for set_property */
static const struct fuel_gauge_driver_api act81461_charger_driver_api = {
	.get_property = &act81461_charger_get_props,
};

/**
 * @brief initialize the fuel gauge
 *
 * @return 0 for success
 */
static int act81461_charger_init(const struct device *dev)
{
	int ret;
	const struct act81461_gauge_config *config = dev->config;
	struct act81461_gauge_data *data = dev->data;

	data->dev = dev;
	sys_slist_init(&data->cbs);

	if (!i2c_is_ready_dt(&config->i2c)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	/* Setup and configure the interrupt for PMIC_CHARGE_STATUS */
	if (!gpio_is_ready_dt(&config->alert_gpio)) {
		LOG_ERR("gpio controller %s not ready", config->alert_gpio.port->name);
		return -ENODEV;
	}

	gpio_pin_configure_dt(&config->alert_gpio, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&config->alert_gpio, GPIO_INT_EDGE_BOTH);
	gpio_init_callback(&data->gpio_int_cb, act81461_battery_charge_intr_callback,
			   config->alert_gpio.pin);
	ret = gpio_add_callback(config->alert_gpio.port, &data->gpio_int_cb);

	k_work_init(&data->interrupt_work, interrupt_work_fn);

	if (ret) {
		LOG_ERR("Cannot setup PMIC ALERT callback, err %d", ret);
	}

	return 0;
}

/* FIXME: fix init priority */
#define ACT81461_CHARGER_INIT(index)                                                               \
                                                                                                   \
	static struct act81461_gauge_data data_##index;                                            \
	static const struct act81461_gauge_config act81461_charger_config_##index = {              \
		.i2c = I2C_DT_SPEC_GET(DT_PARENT(DT_DRV_INST(index))),                             \
		.alert_gpio =                                                                      \
			GPIO_DT_SPEC_GET_OR(DT_PARENT(DT_DRV_INST(index)), alert_gpios, {0}),      \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, &act81461_charger_init, NULL, &data_##index,                  \
			      &act81461_charger_config_##index, POST_KERNEL, 90,                   \
			      &act81461_charger_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ACT81461_CHARGER_INIT)
