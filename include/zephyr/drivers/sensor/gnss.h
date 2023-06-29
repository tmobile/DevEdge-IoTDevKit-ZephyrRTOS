/*
 * Copyright (c) 2022 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_GNSS_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_GNSS_H_
#include <zephyr/drivers/sensor.h>

struct gnss_global_time {
	uint32_t time_of_week;		 /* ms */
	uint16_t gnss_year;
	uint8_t gnss_month;
	uint8_t gnss_day;
	uint8_t gnss_hour;
	uint8_t gnss_minute;
	uint8_t gnss_second;
	struct {
		uint8_t valid_date : 1;
		uint8_t valid_time : 1;
		uint8_t fully_resolved : 1;
		uint8_t valid_magnitude : 1;
	} valid;
	uint32_t accuracy;
	uint32_t gnss_nanosecond;
} __packed;

struct gnss_global_position {
	uint8_t fix_type;
	struct {
		uint8_t gnss_fix_ok : 1;
		uint8_t diff_soln : 1;
		uint8_t psm_state : 4;
		uint8_t head_veh_valid : 1;
		uint8_t carr_soln : 1;
	} fix_status;
	uint8_t flag_s2;
	uint8_t SIV;
	int32_t longitude;		/*Degrees * 10^-7 (more accurate than floats)*/
	int32_t latitude;		/*Degrees * 10^-7 (more accurate than floats)*/
	int32_t altitude;
	int32_t altitude_MSL;
	uint32_t hDOP;
	uint32_t vDOP;
} __packed;

struct gnss_global_velocity {
	int32_t north;
	int32_t east;
	int32_t down;
	int32_t ground_speed;
	int32_t heading_of_motion;
	uint32_t speed_accuracy;
	uint32_t heading_accuracy;
	uint16_t pDOP;
	uint8_t flag_s3;
	uint8_t reserved1[5];
	int32_t head_veh;
	int16_t magnitude_dec;
	uint16_t magnitude_acc;
} __packed;


struct gnss_global_data  {
	/*The major global data*/
	struct gnss_global_time time;
	struct gnss_global_position position;
	struct gnss_global_velocity velocity;
};

/* driver sensor channels */
enum gnss_channel {
	/* GNSS Time */
	SENSOR_CHAN_GNSS_TIME = SENSOR_CHAN_PRIV_START,
	/* GNSS Latitude */
	SENSOR_CHAN_GNSS_POSITION_LAT,
	/* GNSS Longitude */
	SENSOR_CHAN_GNSS_POSITION_LNG,
	/* GNSS Position (Lat,Lng) */
	SENSOR_CHAN_GNSS_POSITION,
	/* GNSS Velocity */
	SENSOR_CHAN_GNSS_VELOCITY,

	/**
	 * Number of all common sensor channels.
	 */
	SENSOR_CHAN_GNSS_COMMON_COUNT,

	/**
	 * This and higher values are sensor specific.
	 * Refer to the sensor header file.
	 */
	SENSOR_CHAN_GNSS_PRIV_START = SENSOR_CHAN_GNSS_COMMON_COUNT,

	/**
	 * Maximum value describing a sensor channel type.
	 */
	SENSOR_CHAN_GNSS_MAX = INT16_MAX,
};

/* driver sensor attributes */
enum gnss_attribute {
	SENSOR_ATTR_GNSS_UPDATE_PERIOD = SENSOR_ATTR_PRIV_START,
	SENSOR_ATTR_GNSS_SEARCH_PERIOD,
	SENSOR_ATTR_GNSS_SLEEP_DURATION,

	SENSOR_ATTR_GNSS_NAV_MEASUREMENT_RATE,
	SENSOR_ATTR_GNSS_NAV_SOLUTION_RATE,
	SENSOR_ATTR_GNSS_NAV_TIMEREF,
	GNSS_ATTRIBUTE_NAV_SETTINGS,
	SENSOR_ATTR_GNSS_NAV_MSG_PVT_RATE,
	SENSOR_ATTR_GNSS_NAV_MSG_SOL_RATE,
	SENSOR_ATTR_GNSS_NAV_MSG_STATUS_RATE,

	SENSOR_ATTR_GNSS_INFO_ID,
	SENSOR_ATTR_GNSS_INFO_HW_STATUS,
	SENSOR_ATTR_GNSS_INFO_HW2_STATUS,
	SENSOR_ATTR_GNSS_INFO_IO_STATUS,
	SENSOR_ATTR_GNSS_INFO_GNSS_STATUS,

	SENSOR_ATTR_GNSS_INFO_VERSION_HW,
	SENSOR_ATTR_GNSS_INFO_VERSION_SW,
	SENSOR_ATTR_GNSS_INFO_VERSION_PROTO,

	SENSOR_ATTR_GNSS_PORT_UART,
	SENSOR_ATTR_GNSS_PORT_I2C,
	SENSOR_ATTR_GNSS_PORT_USB,

	SENSOR_ATTR_GNSS_NAV_STATUS,

	SENSOR_ATTR_GNSS_PM_STATUS,
	SENSOR_ATTR_GNSS_PM_MODE,
	SENSOR_ATTR_GNSS_PM_SLEEP,

	SENSOR_ATTR_GNSS_TIMEPULSE_STATUS,

	SENSOR_ATTR_GNSS_SAVE,

	SENSOR_ATTR_GNSS_LATITUDE,
	SENSOR_ATTR_GNSS_LONGITUDE,
	SENSOR_ATTR_GNSS_ALTITUDE_MSL,
	SENSOR_ATTR_GNSS_ALTITUDE_HAE,
	SENSOR_ATTR_GNSS_HDOP,
	SENSOR_ATTR_GNSS_FIXTYPE,
	SENSOR_ATTR_GNSS_SIV,

	SENSOR_ATTR_GNSS_DAY,
	SENSOR_ATTR_GNSS_MONTH,
	SENSOR_ATTR_GNSS_YEAR,
	SENSOR_ATTR_GNSS_HOUR,
	SENSOR_ATTR_GNSS_MINUTE,
	SENSOR_ATTR_GNSS_SECOND,

	SENSOR_ATTR_GNSS_PDOP,
	SENSOR_ATTR_GNSS_VER,

	/**
	 * Number of all common sensor attributes.
	 */
	SENSOR_ATTR_GNSS_COMMON_COUNT,

	/**
	 * This and higher values are sensor specific.
	 * Refer to the sensor header file.
	 */
	SENSOR_ATTR_GNSS_PRIV_START = SENSOR_ATTR_GNSS_COMMON_COUNT,

	/**
	 * Maximum value describing a sensor attribute type.
	 */
	SENSOR_ATTR_GNSS_MAX = INT16_MAX,
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_GNSS_H_ */
