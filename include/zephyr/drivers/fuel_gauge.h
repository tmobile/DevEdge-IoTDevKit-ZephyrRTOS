/*
 * Copyright 2022 Google LLC
 * Copyright (c) 2023 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_BATTERY_H_
#define ZEPHYR_INCLUDE_DRIVERS_BATTERY_H_

/**
 * @brief Fuel Gauge Interface
 * @defgroup fuel_gauge_interface Fuel Gauge Interface
 * @ingroup io_interfaces
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>

/* Keep these alphabetized wrt property name */

/** Runtime Dynamic Battery Parameters */
/**
 * Provide a 1 minute average of the current on the battery.
 * Does not check for flags or whether those values are bad readings.
 * See driver instance header for details on implementation and
 * how the average is calculated. Units in uA negative=discharging
 */
#define FUEL_GAUGE_AVG_CURRENT 0

/** Battery current (uA); negative=discharging */
#define FUEL_GAUGE_CURRENT		FUEL_GAUGE_AVG_CURRENT + 1
/** Whether the battery underlying the fuel-gauge is cut off from charge */
#define FUEL_GAUGE_CHARGE_CUTOFF	FUEL_GAUGE_CURRENT + 1
/** Cycle count in 1/100ths (number of charge/discharge cycles) */
#define FUEL_GAUGE_CYCLE_COUNT		FUEL_GAUGE_CHARGE_CUTOFF + 1
/** Connect state of battery */
#define FUEL_GAUGE_CONNECT_STATE	FUEL_GAUGE_CYCLE_COUNT + 1
/** General Error/Runtime Flags */
#define FUEL_GAUGE_FLAGS		FUEL_GAUGE_CONNECT_STATE + 1
/** Full Charge Capacity in uAh (might change in some implementations to determine wear) */
#define FUEL_GAUGE_FULL_CHARGE_CAPACITY FUEL_GAUGE_FLAGS + 1
/** Is the battery physically present */
#define FUEL_GAUGE_PRESENT_STATE	FUEL_GAUGE_FULL_CHARGE_CAPACITY + 1
/** Remaining capacity in uAh */
#define FUEL_GAUGE_REMAINING_CAPACITY	FUEL_GAUGE_PRESENT_STATE + 1
/** Remaining battery life time in minutes */
#define FUEL_GAUGE_RUNTIME_TO_EMPTY	FUEL_GAUGE_REMAINING_CAPACITY + 1
/** Remaining time in minutes until battery reaches full charge */
#define FUEL_GAUGE_RUNTIME_TO_FULL	FUEL_GAUGE_RUNTIME_TO_EMPTY + 1
/** Retrieve word from SBS1.1 ManufactuerAccess */
#define FUEL_GAUGE_SBS_MFR_ACCESS	FUEL_GAUGE_RUNTIME_TO_FULL + 1
/** Absolute state of charge (percent, 0-100) - expressed as % of design capacity */
#define FUEL_GAUGE_STATE_OF_CHARGE	FUEL_GAUGE_SBS_MFR_ACCESS + 1
/** Temperature in 0.1 K */
#define FUEL_GAUGE_TEMPERATURE		FUEL_GAUGE_STATE_OF_CHARGE + 1
/** Battery voltage (uV) */
#define FUEL_GAUGE_VOLTAGE		FUEL_GAUGE_TEMPERATURE + 1
/** Battery Mode (flags) */
#define FUEL_GAUGE_MODE			FUEL_GAUGE_VOLTAGE + 1
/** Battery desired Max Charging Current (mA) */
#define FUEL_GAUGE_CHARGE_CURRENT	FUEL_GAUGE_MODE + 1
/** Battery desired Max Charging Voltage (mV) */
#define FUEL_GAUGE_CHARGE_VOLTAGE	FUEL_GAUGE_CHARGE_CURRENT + 1
/** Alarm, Status and Error codes (flags) */
#define FUEL_GAUGE_STATUS		FUEL_GAUGE_CHARGE_VOLTAGE + 1
/** Design Capacity (mAh or 10mWh) */
#define FUEL_GAUGE_DESIGN_CAPACITY	FUEL_GAUGE_STATUS + 1
/** Design Voltage (mV) */
#define FUEL_GAUGE_DESIGN_VOLTAGE	FUEL_GAUGE_DESIGN_CAPACITY + 1

/** Reserved to demark end of common fuel gauge properties */
#define FUEL_GAUGE_COMMON_COUNT FUEL_GAUGE_DESIGN_VOLTAGE + 1
/**
 * Reserved to demark downstream custom properties - use this value as the actual value may change
 * over future versions of this API
 */
#define FUEL_GAUGE_CUSTOM_BEGIN FUEL_GAUGE_COMMON_COUNT + 1

/** Reserved to demark end of valid enum properties */
#define FUEL_GAUGE_PROP_MAX UINT16_MAX

/*
 * Values retrieved from SBS v1.1 Spec (http://sbs-forum.org/specs/sbdat110.pdf)
 * Section 5.1.21
 */

#define FUEL_GAUGE_STATUS_FLAGS_OVER_CHARGED_ALARM        0x8000
#define FUEL_GAUGE_STATUS_FLAGS_TERMINATE_CHARGE_ALARM    0x4000
#define FUEL_GAUGE_STATUS_FLAGS_OVER_TEMP_ALARM           0x1000
#define FUEL_GAUGE_STATUS_FLAGS_TERMINATE_DISCHARGE_ALARM 0x0800
#define FUEL_GAUGE_STATUS_FLAGS_REMAINING_CAPACITY_ALARM  0x0200
#define FUEL_GAUGE_STATUS_FLAGS_REMAINING_TIME_ALARM      0x0100

#define FUEL_GAUGE_STATUS_FLAGS_INITIALIZED               0x0080
#define FUEL_GAUGE_STATUS_FLAGS_DISCHARGING               0x0040
#define FUEL_GAUGE_STATUS_FLAGS_FULLY_CHARGED             0x0020
#define FUEL_GAUGE_STATUS_FLAGS_FULLY_DISCHARGED          0x0010

struct fuel_gauge_get_property {
	/** Battery fuel gauge property to get */
	uint16_t property_type;

	/** Negative error status set by callee e.g. -ENOTSUP for an unsupported property */
	int status;

	/** Property field for getting */
	union {
		/* Fields have the format: */
		/* FUEL_GAUGE_PROPERTY_FIELD */
		/* type property_field; */

		/* Dynamic Battery Info */
		/** FUEL_GAUGE_AVG_CURRENT */
		int avg_current;
		/** FUEL_GAUGE_CUTOFF */
		bool cutoff;
		/** FUEL_GAUGE_CURRENT */
		int current;
		/** FUEL_GAUGE_CYCLE_COUNT */
		uint32_t cycle_count;
		/** FUEL_GAUGE_FLAGS */
		uint32_t flags;
		/** FUEL_GAUGE_FULL_CHARGE_CAPACITY */
		uint32_t full_charge_capacity;
		/** FUEL_GAUGE_REMAINING_CAPACITY */
		uint32_t remaining_capacity;
		/** FUEL_GAUGE_RUNTIME_TO_EMPTY */
		uint32_t runtime_to_empty;
		/** FUEL_GAUGE_RUNTIME_TO_FULL */
		uint32_t runtime_to_full;
		/** FUEL_GAUGE_SBS_MFR_ACCESS */
		uint16_t sbs_mfr_access_word;
		/** FUEL_GAUGE_STATE_OF_CHARGE */
		uint8_t state_of_charge;
		/** FUEL_GAUGE_TEMPERATURE */
		uint16_t temperature;
		/** FUEL_GAUGE_VOLTAGE */
		int voltage;
		/** FUEL_GAUGE_MODE */
		uint16_t mode;
		/** FUEL_GAUGE_CHARGE_CURRENT */
		uint16_t chg_current;
		/** FUEL_GAUGE_CHARGE_VOLTAGE */
		uint16_t chg_voltage;
		/** FUEL_GAUGE_STATUS */
		uint16_t fg_status;
		/** FUEL_GAUGE_DESIGN_CAPACITY */
		uint16_t design_cap;
		/** FUEL_GAUGE_DESIGN_VOLTAGE */
		uint16_t design_volt;
	} value;
};

struct fuel_gauge_set_property {
	/** Battery fuel gauge property to set */
	uint16_t property_type;

	/** Negative error status set by callee e.g. -ENOTSUP for an unsupported property */
	int status;

	/** Property field for setting */
	union {
		/* Fields have the format: */
		/* FUEL_GAUGE_PROPERTY_FIELD */
		/* type property_field; */

		/* Writable Dynamic Battery Info */
		/** FUEL_GAUGE_SBS_MFR_ACCESS */
		uint16_t sbs_mfr_access_word;
	} value;
};

/**
 * @typedef fuel_gauge_get_property_t
 * @brief Callback API for getting a fuel_gauge property.
 *
 * See fuel_gauge_get_property() for argument description
 */
typedef int (*fuel_gauge_get_property_t)(const struct device *dev,
					 struct fuel_gauge_get_property *props, size_t props_len);

/**
 * @typedef fuel_gauge_set_property_t
 * @brief Callback API for setting a fuel_gauge property.
 *
 * See fuel_gauge_set_property() for argument description
 */
typedef int (*fuel_gauge_set_property_t)(const struct device *dev,
					 struct fuel_gauge_set_property *props, size_t props_len);

/* Caching is entirely on the onus of the client */

__subsystem struct fuel_gauge_driver_api {
	fuel_gauge_get_property_t get_property;
	fuel_gauge_set_property_t set_property;
};

/**
 * @brief Fetch a battery fuel-gauge property
 *
 * @param dev Pointer to the battery fuel-gauge device
 * @param props pointer to array of fuel_gauge_get_property struct where the property struct
 * field is set by the caller to determine what property is read from the
 * fuel gauge device into the fuel_gauge_get_property struct's value field.
 * @param props_len number of properties in props array
 *
 * @return return=0 if successful, return < 0 if getting all properties failed, return > 0 if some
 * properties failed where return=number of failing properties.
 */
__syscall int fuel_gauge_get_prop(const struct device *dev, struct fuel_gauge_get_property *props,
				  size_t props_len);

static inline int z_impl_fuel_gauge_get_prop(const struct device *dev,
					     struct fuel_gauge_get_property *props,
					     size_t props_len)
{
	const struct fuel_gauge_driver_api *api = (const struct fuel_gauge_driver_api *)dev->api;

	if (api->get_property == NULL) {
		return -ENOSYS;
	}

	return api->get_property(dev, props, props_len);
}

/**
 * @brief Set a battery fuel-gauge property
 *
 * @param dev Pointer to the battery fuel-gauge device
 * @param props pointer to array of fuel_gauge_set_property struct where the property struct
 * field is set by the caller to determine what property is written to the fuel gauge device from
 * the fuel_gauge_get_property struct's value field.
 * @param props_len number of properties in props array
 *
 * @return return=0 if successful, return < 0 if setting all properties failed, return > 0 if some
 * properties failed where return=number of failing properties.
 */
__syscall int fuel_gauge_set_prop(const struct device *dev, struct fuel_gauge_set_property *props,
				  size_t props_len);

static inline int z_impl_fuel_gauge_set_prop(const struct device *dev,
					     struct fuel_gauge_set_property *props,
					     size_t props_len)
{
	const struct fuel_gauge_driver_api *api = (const struct fuel_gauge_driver_api *)dev->api;

	if (api->set_property == NULL) {
		return -ENOSYS;
	}

	return api->set_property(dev, props, props_len);
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#include <syscalls/fuel_gauge.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_BATTERY_H_ */
