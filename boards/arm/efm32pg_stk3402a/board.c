/*
 * Copyright (c) 2018 Christian Taedcke
 * Copyright (c) 2022 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include "board.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

static int efm32pg_stk3402a_init(const struct device *dev)
{
	const struct device *bce_dev; /* Board Controller Enable Gpio Device */
	const struct device *tmp_dev; /* Temperature Sensor Enable Gpio Device */
	
	ARG_UNUSED(dev);

	/* Enable the board controller to be able to use the serial port */
	bce_dev = DEVICE_DT_GET(BC_ENABLE_GPIO_NODE);

	if (!device_is_ready(bce_dev)) {
		printk("Board controller gpio port is not ready!\n");
		return -ENODEV;
	}

	gpio_pin_configure(bce_dev, BC_ENABLE_GPIO_PIN, GPIO_OUTPUT_HIGH);

	/* Enable the temperature sensore */
	tmp_dev = DEVICE_DT_GET(TMP_ENABLE_GPIO_NODE);

	if (!device_is_ready(tmp_dev)) {
		printk("Temperature sensor gpio port is not ready!\n");
		return -ENODEV;
	}

	gpio_pin_configure(tmp_dev, TMP_ENABLE_GPIO_PIN, GPIO_OUTPUT_HIGH);

	return 0;
}

/* needs to be done after GPIO driver init */
SYS_INIT(efm32pg_stk3402a_init, POST_KERNEL,
	 CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
