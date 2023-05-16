/*
 * Copyright (c) 2023 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ZEPHYR_INCLUDE_DRIVERS_FUEL_GAUGE_ACT81461_H__
#define __ZEPHYR_INCLUDE_DRIVERS_FUEL_GAUGE_ACT81461_H__
#include <zephyr/kernel.h>
#include <zephyr/device.h>

struct act81461_int_cb {
	/**
	 * @brief Callback function
	 *
	 * @param dev Device pointer to ALPC which received the interrupt
	 */
	void (*cb)(const struct device *dev);

	/** Internally used field for list handling */
	sys_snode_t node;
};

/**
 * @brief Register a callback for ALPC interrupts.
 *
 * @param dev Pointer to the ALPC device
 * @param cb Pointer to callback structure
 *
 * @retval 0 If successful.
 * @retval -errno In case of any other error.
 */
int act81461_charger_int_cb_register(const struct device *dev, struct act81461_int_cb *cb);

/**
 * @brief Unregister a previously registered callback for ALPC interrupts.
 *
 * @param dev Pointer to the ALPC device
 * @param cb Pointer to callback structure
 *
 * @retval 0 If successful.
 * @retval -errno In case of any other error.
 */
int act81461_charger_int_cb_unregister(const struct device *dev, struct act81461_int_cb *cb);

#endif /* __ZEPHYR_INCLUDE_DRIVERS_FUEL_GAUGE_ACT81461_H__ */
