/*
 * Copyright (c) 2023 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_LIS2DW12_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_LIS2DW12_H_

#ifdef __cplusplus
extern "C" {
#endif

enum sensor_attribute_lis2dw12 {
	/** sensor FreeFall threshold */
	SENSOR_ATTR_FF_THS = SENSOR_ATTR_PRIV_START,

	/** sensor interrupt enable */
	SENSOR_ATTR_INTERRUPT_ENABLE,

	/** sensor event status */
	SENSOR_ATTR_STATUS,

	/** TAP threshold X */
	SENSOR_ATTR_TAP_THRESHOLD_X,

	/** TAP threshold Y */
	SENSOR_ATTR_TAP_THRESHOLD_Y,

	/** TAP threshold Z */
	SENSOR_ATTR_TAP_THRESHOLD_Z,

	/** INT DURATION */
	SENSOR_ATTR_INT_DUR,

	/** WAKE THRESHOLD */
	SENSOR_ATTR_WAKE_THRESHOLD,

	/** WAKE DURATION */
	SENSOR_ATTR_WAKE_DUR,

	/** FREEFALL */
	SENSOR_ATTR_FREEFALL,

	/** WHO AM I */
	SENSOR_ATTR_CHIP_ID,

	/** EVENT INTERRUPT ENABLE (routed to INT1 PIN) */
	SENSOR_ATTR_ENABLE_EVENT_INTERRUPT,

	/** ALL WAKE UP INTERRUPT SRC */
	/*  Read these 5 consecutive status registers:
	 *  STATUS_DUP, WAKE_UP_SRC, TAP_SRC, SIXD_SRC, ALL_INT_SRC
	 */
	SENSOR_ATTR_ALL_WAKE_UP_SRC,

	/** TAP SRC */
	SENSOR_ATTR_TAP_SRC,

	/** ALL INTERRUPT SRC */
	SENSOR_ATTR_ALL_INT_SRC,

};


#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_LIS2DW12_H_ */
