/*
 * Copyright (c) 2022 Kim Mansfield <kmansfie@yahoo.com>
 * Copyright (c) 2022 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_CXD5605_CXD5605_H_
#define ZEPHYR_DRIVERS_SENSOR_CXD5605_CXD5605_H_

#include <stdint.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor/cxd5605.h>
#include <cxd5605_lib.h>

int cxd5605_setup_interrupts(const struct device *dev);


struct cxd5605_data {
	const struct device *cxd5605_dev;
	struct cxd5605_dev cdev;

	struct gpio_callback data_ready_gpio_cb;
	struct gpio_callback one_pps_gpio_cb;

	pps_func pps_cb;
	struct gnss_global_data data;
	struct cxd5605_version ver;
	char version[32];

	uint32_t op_mode;
	uint32_t pos_cycle;
	uint32_t sleep_time;
	uint32_t selected_sentences;

#if defined(CONFIG_CXD5605_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, 2048);
	struct k_thread thread;
	struct k_sem gpio_sem;
#else
	struct k_work work;
#endif
};

struct cxd5605_config {
	const struct i2c_dt_spec i2c_spec;
	const struct gpio_dt_spec alert_gpio;
	const struct gpio_dt_spec int_gpio;
	const struct gpio_dt_spec rst_gpio;
	const struct gpio_dt_spec boot_rec_gpio;
};

#endif /*  ZEPHYR_DRIVERS_SENSOR_CXD5605_CXD5605_H_ */
