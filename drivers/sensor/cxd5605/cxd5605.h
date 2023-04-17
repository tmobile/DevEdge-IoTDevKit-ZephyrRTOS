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

int cxd5605_setup_interrupts(const struct device *dev);

#endif /*  ZEPHYR_DRIVERS_SENSOR_CXD5605_CXD5605_H_ */
