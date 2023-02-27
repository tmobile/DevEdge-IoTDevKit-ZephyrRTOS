/*
 * Copyright (c) 2023 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ZEPHYR_INCLUDE_DRIVERS_FUEL_GAUGE_ACT81461_H__
#define __ZEPHYR_INCLUDE_DRIVERS_FUEL_GAUGE_ACT81461_H__
#include <zephyr/device.h>

typedef void(*act81461_int_cb_t)(const struct device *);

int act81461_charger_int_cb_set(const struct device *dev, act81461_int_cb_t cb);

#endif /* __ZEPHYR_INCLUDE_DRIVERS_FUEL_GAUGE_ACT81461_H__ */
