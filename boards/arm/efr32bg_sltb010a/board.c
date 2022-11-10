/*
 * Copyright (c) 2021 Sateesh Kotapati
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(efr32bg_sltb010a, CONFIG_BOARD_EFR32BG22_LOG_LEVEL);

static int efr32bg_sltb010a_init(const struct device *dev)
{
	static struct gpio_dt_spec wake_up_gpio_dev =
		GPIO_DT_SPEC_GET(DT_NODELABEL(wake_up_trigger), gpios);

	ARG_UNUSED(dev);

	if (!device_is_ready(wake_up_gpio_dev.port)) {
		LOG_ERR("Wake-up GPIO device was not found!\n");
		return -ENODEV;
	}
	gpio_pin_configure_dt(&wake_up_gpio_dev, GPIO_OUTPUT_ACTIVE);

	return 0;
}

/* needs to be done after GPIO driver init */
SYS_INIT(efr32bg_sltb010a_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
