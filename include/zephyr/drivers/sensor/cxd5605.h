/*
 * Copyright (c) 2022 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_CXD5605_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_CXD5605_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/gnss.h>
#include <zephyr/drivers/gpio.h>

#define READ_BYTES_SIZE			128
#define CXD5605_PACKET_SIZE		74
#define CXD5605_PACKET_DATA_SIZE	70

#define	NMEA_SENTENCE_ID_IDX		0
#define	NMEA_SENTENCE_ID_LEN		6

enum power_mode_t {
	GNSS_NORMAL_PWR_MODE,
	GNSS_LOW_PWR_MODE
};

enum sensor_attr_sentence_t {
	SENTENCE_GGA		= 0x01,
	SENTENCE_GLL		= 0x02,
	SENTENCE_GSA		= 0x04,
	SENTENCE_GSV		= 0x08,
	SENTENCE_GNS		= 0x10,
	SENTENCE_RMC		= 0x20,
	SENTENCE_VTG		= 0x40,
	SENTENCE_ZDA		= 0x80
};

enum sensor_attribute_cxd5605 {
	SENSOR_ATTR_CXD5605_PULSE = SENSOR_ATTR_PRIV_START,
	SENSOR_ATTR_CXD5605_SHUTDOWN_MODE,
	SENSOR_ATTR_CXD5605_CONTINUOUS_CONVERSION_MODE,
	SENSOR_ATTR_CXD5605_ALERT_POLARITY,
	SENSOR_ATTR_CXD5605_CALLBACK,
	SENSOR_ATTR_CXD5605_SENTENCE_SELECT,
	SENSOR_ATTR_CXD5605_COLD_START,
	SENSOR_ATTR_CXD5605_WAKE_UP,
	SENSOR_ATTR_CXD5605_OPERATING_MODE,
	SENSOR_ATTR_CXD5605_HOT_START,
	SENSOR_ATTR_CXD5605_HOT_START_TTFF,
	SENSOR_ATTR_CXD5605_STOP,
	SENSOR_ATTR_CXD5605_WARM_START,
	SENSOR_ATTR_CXD5605_TIME_SET,
	SENSOR_ATTR_CXD5605_SLEEP,
	SENSOR_ATTR_CXD5605_VERSION,
	SENSOR_ATTR_CXD5605_ASSIST_GEN_FUNCTION_CONTROL,
	};

enum gga_nmea_fieldpos_t {
	GGA_UTC_OF_POSITION_IDX = 1,
	GGA_LATITUDE_IDX,
	GGA_LATITUDE_DIR_IDX,
	GGA_LONGITUDE_IDX,
	GGA_LONGITUDE_DIR_IDX,
	GGA_QUALITY_INDICATOR_IDX,
	GGA_NUM_SATELLITE_IDX,
	GGA_HDOP_IDX,
	GGA_ALTITUDE_IDX,
	GGA_GEOIDAL_SEPARATION_IDX,
	GGA_AGE_OF_DGPS_IDX,
	GGA_DIFFERENTIAL_REF_IDX
};

enum gns_nmea_fieldpos_t {
	GNS_UTC_OF_POSITION_IDX = 1,
	GNS_LATITUDE_IDX,
	GNS_LATITUDE_DIR_IDX,
	GNS_LONGITUDE_IDX,
	GNS_LONGITUDE_DIR_IDX,
	GNS_MODE_INDICATOR_IDX,
	GNS_NUM_SATELLITE_IDX,
	GNS_HDOP_IDX,
	GNS_ALTITUDE_IDX,
	GNS_GEOIDAL_SEPARATION_IDX,
	GNS_AGE_OF_DGPS_IDX,
	GNS_DIFFERENTIAL_REF_IDX
};

enum gll_nmea_fieldpos {
	GLL_LATITUDE_IDX = 1,
	GLL_LATITUDE_DIR_IDX,
	GLL_LONGITUDE_IDX,
	GLL_LONGITUDE_DIR_IDX,
	GLL_UTC_OF_POSITION_IDX,
	GLL_STATUS_IDX,
	GLL_MODE_INDICATOR_IDX
};

struct cxd5605_version {
	uint32_t major;
	uint32_t minor;
	uint32_t patch;
};

struct cxd5605_gtr {
	double cn_level;
	double doppler_freq;
};

struct cxd5605_assisted {
	uint32_t gps_pending;
	uint32_t gps_available;
	uint32_t qzss_pending;
	uint32_t qzss_available;
};

struct cxd5605_cmd_data {
	struct cxd5605_version ver;
};

struct cxd5605_packet {
	uint8_t preamble;
	uint8_t packet_type;
	uint8_t data_size;
	uint8_t data[CXD5605_PACKET_DATA_SIZE];
	uint8_t checksum;
};

struct cxd5605_binary_data {
	uint8_t preamble;
	uint8_t control_type;
	uint8_t data_length_upper;
	uint8_t data_length_lower;
	uint8_t data[CXD5605_PACKET_DATA_SIZE];
	uint8_t checksum_upper;
	uint8_t checksum_lower;
	uint8_t fixed_value1;
	uint8_t fixed_value2;
};

typedef void (*pps_func)(void);

struct cxd5605_data {
	const struct device *cxd5605_dev;

	struct gpio_callback data_ready_gpio_cb;
	struct gpio_callback one_pps_gpio_cb;
	struct gpio_callback gpio_cb;

	pps_func pps_cb;
	struct gnss_global_data pvt;
	struct cxd5605_cmd_data cxd5605_cmd_data;
	char version[32];
	uint16_t bin_data_ptr;
	uint16_t bin_data_len;
	uint16_t bytes_remaining;
	uint16_t copy_length;

	int cxd5605_cmd;
	int num_msg;

	uint32_t i2c_error_count;

	bool got_data;
	bool lib_got_data;
	int command;

	uint32_t op_mode;
	uint32_t pos_cycle;
	uint32_t sleep_time;
	uint32_t selected_sentences;
};

/** a mask for the under temp alert bit in the status word*/

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_CXD5605_H_ */
