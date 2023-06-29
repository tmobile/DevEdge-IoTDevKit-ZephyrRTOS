/*
 * Copyright (c) 2017 Christian Taedcke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include "board.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

static int efm32wg_stk3800_init(void)
{
	const struct device *bce_dev; /* Board Controller Enable Gpio Device */


	/* Enable the board controller to be able to use the serial port */
	bce_dev = DEVICE_DT_GET(BC_ENABLE_GPIO_NODE);

	if (!device_is_ready(bce_dev)) {
		printk("Board controller gpio port not ready!\n");
		return -ENODEV;
	}

	gpio_pin_configure(bce_dev, BC_ENABLE_GPIO_PIN, GPIO_OUTPUT_HIGH);

	return 0;
}

/* needs to be done after GPIO driver init */
SYS_INIT(efm32wg_stk3800_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
