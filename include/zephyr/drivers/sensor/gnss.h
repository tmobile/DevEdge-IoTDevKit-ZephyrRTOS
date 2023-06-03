/*
 * Copyright (c) 2022 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_GNSS_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_GNSS_H_

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
	uint32_t horizontal_accuracy;	/* mm * 10^-1 (i.e. 0.1mm) */
	uint32_t vertical_accuracy;	/* mm * 10^-1 (i.e. 0.1mm) */
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
	GNSS_CHANNEL_NONE,
	GNSS_CHANNEL_TIME,
	GNSS_CHANNEL_POSITION,
	GNSS_CHANNEL_VELOCITY,

	/** All channels. */
	GNSS_CHANNEL_ALL,

	/**
	 * Number of all common sensor channels.
	 */
	GNSS_CHANNEL_COMMON_COUNT,

	/**
	 * This and higher values are sensor specific.
	 * Refer to the sensor header file.
	 */
	GNSS_CHANNEL_PRIV_START = GNSS_CHANNEL_COMMON_COUNT,

	/**
	 * Maximum value describing a sensor channel type.
	 */
	GNSS_CHANNEL_MAX = INT16_MAX,
};

/* driver sensor attributes */
enum gnss_attribute {
	GNSS_ATTRIBUTE_UPDATE_PERIOD,
	GNSS_ATTRIBUTE_SEARCH_PERIOD,
	GNSS_ATTRIBUTE_SLEEP_DURATION,

	GNSS_ATTRIBUTE_NAV_MEASUREMENT_RATE,
	GNSS_ATTRIBUTE_NAV_SOLUTION_RATE,
	GNSS_ATTRIBUTE_NAV_TIMEREF,
	GNSS_ATTRIBUTE_NAV_SETTINGS,
	GNSS_ATTRIBUTE_NAV_MSG_PVT_RATE,
	GNSS_ATTRIBUTE_NAV_MSG_SOL_RATE,
	GNSS_ATTRIBUTE_NAV_MSG_STATUS_RATE,

	GNSS_ATTRIBUTE_INFO_ID,
	GNSS_ATTRIBUTE_INFO_HW_STATUS,
	GNSS_ATTRIBUTE_INFO_HW2_STATUS,
	GNSS_ATTRIBUTE_INFO_IO_STATUS,
	GNSS_ATTRIBUTE_INFO_GNSS_STATUS,

	GNSS_ATTRIBUTE_INFO_VERSION_HW,
	GNSS_ATTRIBUTE_INFO_VERSION_SW,
	GNSS_ATTRIBUTE_INFO_VERSION_PROTO,

	GNSS_ATTRIBUTE_PORT_UART,
	GNSS_ATTRIBUTE_PORT_I2C,
	GNSS_ATTRIBUTE_PORT_USB,

	GNSS_ATTRIBUTE_NAV_STATUS,

	GNSS_ATTRIBUTE_PM_STATUS,
	GNSS_ATTRIBUTE_PM_MODE,
	GNSS_ATTRIBUTE_PM_SLEEP,

	GNSS_ATTRIBUTE_TIMEPULSE_STATUS,

	GNSS_ATTRIBUTE_SAVE,

	GNSS_ATTRIBUTE_LATITUDE,
	GNSS_ATTRIBUTE_LONGITUDE,
	GNSS_ATTRIBUTE_ALTITUDE_MSL,
	GNSS_ATTRIBUTE_ALTITUDE_HAE,
	GNSS_ATTRIBUTE_HDOP,
	GNSS_ATTRIBUTE_FIXTYPE,
	GNSS_ATTRIBUTE_SIV,

	GNSS_ATTRIBUTE_TIME,
	GNSS_ATTRIBUTE_DAY,
	GNSS_ATTRIBUTE_MONTH,
	GNSS_ATTRIBUTE_YEAR,
	GNSS_ATTRIBUTE_UTC,

	GNSS_ATTRIBUTE_PDOP,
	GNSS_ATTRIBUTE_VER,

	/**
	 * Number of all common sensor attributes.
	 */
	GNSS_ATTRIBUTE_COMMON_COUNT,

	/**
	 * This and higher values are sensor specific.
	 * Refer to the sensor header file.
	 */
	GNSS_ATTRIBUTE_PRIV_START = GNSS_ATTRIBUTE_COMMON_COUNT,

	/**
	 * Maximum value describing a sensor attribute type.
	 */
	GNSS_ATTRIBUTE_MAX = INT16_MAX,
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_GNSS_H_ */
