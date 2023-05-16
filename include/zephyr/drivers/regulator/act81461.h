/*
 * Copyright (c) 2023 T-Mobile USA, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_REGULATOR_ACT81461_H_
#define ZEPHYR_INCLUDE_DRIVERS_REGULATOR_ACT81461_H_
#include <zephyr/drivers/regulator.h>

#define REGULATOR_ERROR_UNDER_VOLTAGE	   BIT(3)
#define REGULATOR_ERROR_VSYS_UNDER_VOLTAGE BIT(4)
#define REGULATOR_ERROR_VBAT_LOW	   BIT(5)

struct act81461_regulator_fault_cb {
	/**
	 * @brief Callback function.
	 *
	 * @param dev Device pointer to regulator which received the interrupt.
	 * @param flags Error flags from the regulator.
	 */
	void (*cb)(const struct device *dev, regulator_error_flags_t flags);

	/** Internally used field for list handling */
	sys_snode_t node;
};

/**
 * @brief Register a callback for PMIC faults.
 *
 * @param dev Pointer to the PMIC regulator or the PMIC.
 * @param cb Pointer to callback structure.
 *
 * @retval 0 If successful.
 * @retval -errno In case of any other error.
 */
int act81461_fault_int_cb_register(const struct device *dev,
				   struct act81461_regulator_fault_cb *cb);

/**
 * @brief Unregister a previously registered callback for PMIC faults.
 *
 * @param dev Pointer to the PMIC regulator or the PMIC
 * @param cb Pointer to callback structure
 *
 * @retval 0 If successful.
 * @retval -errno In case of any other error.
 */
int act81461_fault_int_cb_unregister(const struct device *dev,
				     struct act81461_regulator_fault_cb *cb);

/**
 * @brief Mask/unmask specific PMIC fault interrupts
 *
 * @param dev Pointer to the PMIC regulator or the PMIC
 * @param flags Fault types to configure
 * @param mask Desired enablement state
 *
 * @retval 0 If successful.
 * @retval -EINVAL If the given flags-regulator combination is not valid.
 * @retval -EIO General input / output error.
 * @retval -errno In case of any other error.
 */
int act81461_fault_int_msk_set(const struct device *dev, regulator_error_flags_t flags, bool mask);

/**
 * @brief Set vsysmon interrupt trigger voltage.
 *
 * @param dev PMIC/Regulator device pointer.
 * @param min_uv Minimum voltage in microvolts.
 * @param max_uv Maximum voltage in microvolts.
 *
 * @retval 0 If successful.
 * @retval -EINVAL If the given voltage window is not valid.
 * @retval -EIO General input / output error.
 * @retval -errno In case of any other error.
 */
int act81461_set_vsys_low_voltage(const struct device *dev, int32_t min_uv,
						     int32_t max_uv);

#endif /* ZEPHYR_INCLUDE_DRIVERS_REGULATOR_ACT81461_H_ */
