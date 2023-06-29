/*
 * Copyright (c) 2022 Kim Mansfield <kmansfie@yahoo.com>
 * Copyright (c) 2022 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Device driver for the gnss (CXD5605) device
 */

#define DT_DRV_COMPAT sony_cxd5605

#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <stddef.h>
#include <float.h>
#include <zephyr/device.h>
#include <string.h>
#include <ctype.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/gnss.h>
#include <zephyr/drivers/sensor/cxd5605.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <zephyr/pm/device.h>

#include "cxd5605.h"
#include "cxd5605_lib.h"

extern char *strtok_r(char *str, const char *sep, char **state);

LOG_MODULE_REGISTER(CXD5605, CONFIG_SENSOR_LOG_LEVEL);

#define SEND_BUF_SIZE 32

#define CXD5605_ADDRESS DT_N_S_soc_S_i2c_4000c400_S_sonycxd5605_24_REG_IDX_0_VAL_ADDRESS

#define CXD5605_LLE_XFER_SIZE 2048
#define MAXFIELDS	      32

#define COMMAND_STR_LENGTH   12
#define NMEA_SENTENCE_ID_IDX 0

K_SEM_DEFINE(cxd5605_data_sem, 0, 1);

/** @brief Local implementation of atof since atof and strod are not in
 * minimal libc
 *
 * @param str Pointer to string representing float, [+/-][<decimal>][.<fraction>]
 *
 * @return Converted double value
 */
static double cxd_atod(const char *str)
{
	double ret = 0.0;
	bool neg = false;

	// remove leading space
	while (isspace((int)*str)) str++;

	// get sign if present
	if (*str && (*str == '-' || *str == '+')) {
		if (*str == '-') {
			neg = true;
		}
		str++;
	}

	// get decimal part
	while (*str && isdigit((int)*str)) {
		ret = ret * 10 + (*str - '0');
		str++;
	}

	// get fractional part
	if (*str && *str == '.') {
		double frac = 0.0;
		double div = 1.0;
		str++;
		while (*str && isdigit((int)*str)) {
			frac = frac * 10 + (*str - '0');
			str++;
			div *= 10.0;
		}
		ret = ret + frac / div;
	}
	return neg ? (ret * -1.0) : ret;
}

#define cxd_atoi(x) strtol(x, NULL, 10)


static void driver_cxd5605_nmea_cb(struct cxd5605_dev *cxd_dev, const char *nmea_sentence)
{
	struct cxd5605_data *drv_data = CONTAINER_OF(cxd_dev, struct cxd5605_data, cdev);

	/* Parse Raw NMEA Sentence*/
	LOG_DBG("%s", nmea_sentence);

	/* Count commas/fields */
	int field_count = 1;
	const char *fields[32];

	char *comma = strchr(nmea_sentence, ',');
	fields[0] = nmea_sentence;

	while (comma && field_count < ARRAY_SIZE(fields)) {
		fields[field_count] = comma + 1;
		field_count++;
		comma = strchr(comma + 1, ',');
	}

	if (!strncmp("GGA", &fields[NMEA_SENTENCE_ID_IDX][3], 3)) {
		if (field_count >= 11 && cxd_atoi(fields[GGA_QUALITY_INDICATOR_IDX]) == 1) {
			uint32_t time = cxd_atod(fields[GGA_UTC_OF_POSITION_IDX]) * 100;

			drv_data->data.time.gnss_hour = (time / 1000000);
			drv_data->data.time.gnss_minute = (time % 1000000) / 10000;
			drv_data->data.time.gnss_second = (time % 10000) / 100;
			drv_data->data.time.gnss_nanosecond = (time % 100) * 10000000;
			drv_data->data.position.latitude =
				cxd_atod(fields[GGA_LATITUDE_IDX]) * 100000;
			if (fields[GGA_LATITUDE_DIR_IDX][0] == 'S') {
				drv_data->data.position.latitude *= -1;
			}
			drv_data->data.position.longitude =
				cxd_atod(fields[GGA_LONGITUDE_IDX]) * 100000;
			if (fields[GGA_LONGITUDE_DIR_IDX][0] == 'W') {
				drv_data->data.position.longitude *= -1;
			}
			drv_data->data.position.fix_type =
				cxd_atoi(fields[GGA_QUALITY_INDICATOR_IDX]);
			drv_data->data.position.SIV = cxd_atoi(fields[GGA_NUM_SATELLITE_IDX]);
			drv_data->data.position.hDOP =
				cxd_atod(fields[GGA_HDOP_IDX]) * 10;
			drv_data->data.position.altitude_MSL = cxd_atod(fields[GGA_ALTITUDE_IDX]);
			/* Subtract geoidal separation value from
			 * altitude (MSL) to arrive at the altitude
			 * (height above Ellipsoid)
			 */
			drv_data->data.position.altitude =
				drv_data->data.position.altitude_MSL -
				cxd_atod(fields[GGA_GEOIDAL_SEPARATION_IDX]);
		} else {
			drv_data->data.time.gnss_hour = 0;
			drv_data->data.time.gnss_minute = 0;
			drv_data->data.time.gnss_second = 0;
			drv_data->data.time.gnss_nanosecond = 0;
			drv_data->data.position.latitude = 0.0;
			drv_data->data.position.longitude = 0.0;
			drv_data->data.position.fix_type = 0;
			drv_data->data.position.SIV = 0;
			drv_data->data.position.hDOP = 0.0;
			drv_data->data.position.altitude_MSL = 0.0;
			drv_data->data.position.altitude = 0.0;
		}

	} else if (!strncmp("GNS", &fields[NMEA_SENTENCE_ID_IDX][3], 3)) {
		if (field_count < 11) {
			LOG_WRN("$--GNS - field < 11 (%d)", field_count);
		}
		uint32_t t = cxd_atod(fields[GGA_UTC_OF_POSITION_IDX]) * 100;

		drv_data->data.time.gnss_hour = (t / 1000000);
		drv_data->data.time.gnss_minute = (t % 1000000) / 10000;
		drv_data->data.time.gnss_second = (t % 10000) / 100;
		drv_data->data.time.gnss_nanosecond = (t % 100) * 10000000;

		drv_data->data.position.latitude = cxd_atod(fields[GNS_LATITUDE_IDX]) * 100000;
		if (fields[GNS_LATITUDE_DIR_IDX][0] == 'S') {
			drv_data->data.position.latitude *= -1;
		}
		drv_data->data.position.longitude = cxd_atod(fields[GNS_LONGITUDE_IDX]) * 100000;
		if (fields[GNS_LONGITUDE_DIR_IDX][0] == 'W') {
			drv_data->data.position.longitude *= -1;
		}

		drv_data->data.position.SIV = cxd_atoi(fields[GNS_NUM_SATELLITE_IDX]);
		drv_data->data.position.hDOP = cxd_atod(fields[GNS_HDOP_IDX]) * 10;
		drv_data->data.position.altitude_MSL = cxd_atod(fields[GNS_ALTITUDE_IDX]);
		/* Subtract geoidal separation value from altitude
		 * (MSL) to arrive at the altitude (height above
		 * Ellipsoid)
		 */
		drv_data->data.position.altitude = drv_data->data.position.altitude_MSL -
						   cxd_atod(fields[GNS_GEOIDAL_SEPARATION_IDX]);
	} else if (!strncmp("GLL", &fields[NMEA_SENTENCE_ID_IDX][3], 3)) {
		if (field_count < 6) {
			LOG_WRN("$--GLL field < 6 (%d)", field_count);
		}
		uint32_t t = cxd_atod(fields[GLL_UTC_OF_POSITION_IDX]) * 100;

		drv_data->data.time.gnss_hour = (t / 1000000);
		drv_data->data.time.gnss_minute = (t % 1000000) / 10000;
		drv_data->data.time.gnss_second = (t % 10000) / 100;
		drv_data->data.time.gnss_nanosecond = (t % 100) * 10000000;

		drv_data->data.position.latitude = cxd_atod(fields[GLL_LATITUDE_IDX]) * 100000;
		if (fields[GLL_LATITUDE_DIR_IDX][0] == 'S') {
			drv_data->data.position.latitude *= -1;
		}
		drv_data->data.position.longitude = cxd_atod(fields[GLL_LONGITUDE_IDX]) * 100000;
		if (fields[GLL_LONGITUDE_DIR_IDX][0] == 'W') {
			drv_data->data.position.longitude *= -1;
		}
	} else if (!strncmp("ZDA", &fields[NMEA_SENTENCE_ID_IDX][3], 3)) {
		if (field_count < 7) {
			LOG_WRN("$--ZDA field < 7 (%d)", field_count);
		}
		uint32_t t = cxd_atod(fields[ZDA_UTC_IDX]) * 100;

		drv_data->data.time.gnss_hour = (t / 1000000);
		drv_data->data.time.gnss_minute = (t % 1000000) / 10000;
		drv_data->data.time.gnss_second = (t % 10000) / 100;
		drv_data->data.time.gnss_nanosecond = (t % 100) * 10000000;

		drv_data->data.time.gnss_day = cxd_atoi(fields[ZDA_DAY_IDX]);
		drv_data->data.time.gnss_month = cxd_atoi(fields[ZDA_MONTH_IDX]);
		drv_data->data.time.gnss_year = cxd_atoi(fields[ZDA_YEAR_IDX]);
	}
}

#if defined(CONFIG_CXD5605_OWN_THREAD)
static void cxd5605_thread(struct cxd5605_data *drv_data, void *b, void *c)
{
	while (1) {
		k_sem_take(&drv_data->gpio_sem, K_FOREVER);
		cxd5605_lib_process(&drv_data->cdev);
	}
}
#else
static void data_interrupt_work(struct k_work *work)
{
	struct cxd5605_data *drv_data = CONTAINER_OF(work, struct cxd5605_data, work);

	cxd5605_lib_process(&drv_data->cdev);
}
#endif /* defined(CONFIG_CXD5605_OWN_THREAD) */

static int chip_init(const struct device *dev);

/** @brief Initial routine to call to get things running
 *
 *  This routine does the usual things in that it sets some variables
 *  to an initial value check that the i2c bus is operational and saves
 *  the device structure pointer off for use by other routines in the
 *  driver
 *
 *  @param dev The pointer to the device structure for this driver
 *
 *  @return returns an error (-ENODEV) if the i2c bus is not ready
 *
 */
static int init(const struct device *dev)
{
	const struct cxd5605_config *cfg = dev->config;
	struct cxd5605_data *drv_data = dev->data;
	int result = 0;

	drv_data->data.position.latitude = 0.0;
	drv_data->data.position.longitude = 0.0;
	drv_data->data.position.altitude_MSL = 0.0;
	drv_data->data.position.altitude = 0.0;

	if (!device_is_ready(cfg->i2c_spec.bus)) {
		LOG_ERR("I2C dev %s not ready", cfg->i2c_spec.bus->name);
		return -ENODEV;
	}

	/* save this driver instance for passing to other functions */
	drv_data->cxd5605_dev = dev;

#if defined(CONFIG_CXD5605_OWN_THREAD)
	k_thread_create(&drv_data->thread, drv_data->thread_stack,
		       2048,
		       (k_thread_entry_t)cxd5605_thread, drv_data,
		       NULL, NULL, K_PRIO_COOP(CONFIG_CXD5605_THREAD_PRIORITY),
		       0, K_NO_WAIT);
#else
	k_work_init(&drv_data->work, data_interrupt_work);
#endif /* CONFIG_CXD5605_OWN_THREAD */

#if CONFIG_PM_DEVICE
	if (pm_device_on_power_domain(dev) && !pm_device_is_powered(dev)) {
		pm_device_init_off(dev);
	} else {
		chip_init(dev);
#if defined(CONFIG_CXD5605_INIT_SUSPENDED)
		cxd5605_sleep(&drv_data->cdev, 2);
		pm_device_init_suspended(dev)
#endif
	}
#else
	chip_init(dev);
#endif
	return result;
}

/**
 * @brief Perform procedure to initialize the chip from poweroff
 * 
 * @param dev The pointer to the device structure for this driver
 * @return int 
 */
static int chip_init(const struct device *dev)
{
	const struct cxd5605_config *cfg = dev->config;
	struct cxd5605_data *drv_data = dev->data;
	int result = 0;

	drv_data->data.position.latitude = 0.0;
	drv_data->data.position.longitude = 0.0;
	drv_data->data.position.altitude_MSL = 0.0;
	drv_data->data.position.altitude = 0.0;

	result = gpio_pin_configure_dt(&cfg->rst_gpio, GPIO_OUTPUT_HIGH);

	cxd5605_setup_interrupts(dev);
	/* setup shim callbacks */
	cxd5605_init_handshake(&drv_data->cdev);

	return result;
}

/** @brief This routine takes a comma separated list and breaks it up
 *  into tokens.
 *
 *  This routine takes a comma separated string and creates an array
 *  of string pointers that point to each token in the string.  Each
 *  comman is replaced by a null to terminate the string.  If there are
 *  two commas together there are two nulls put in their place and you
 *  get a 0 length token which is perfect.  This routine does not
 *  require any dynamic memory allocation for the tokens and you get
 *  the exact token that came in the string.  The number of maximum
 *  fields is MAXFIELDS defined above.
 *
 *  @param string is a pointer to the comma separated list
 *  @param separator contains the comma separator
 *  @param fields is the array of pointer to the field or tokens
 *
 *  @return returns the number of fields that was scanned
 *
 */
int csv_split(char *string, char separator, char *fields[])
{
	int field = 0;
	char separates[4];

	separates[0] = separator;
	separates[1] = '\n';
	separates[2] = '\r';
	separates[3] = 0;

	while ((fields[field++] = (char *)strtok_r(string, separates, &string)) &&
	       (field < MAXFIELDS)) {
	}

	return (field - 1);
}

/** @brief Callback routine for 1PPS interrupt. It will be called every second
 *  from 1PPS output port after getting a fix
 *
 *  @param dev Pointer to device structure for the driver instance.
 *  @param gpio_cb pointer to gpio callback structure
 *  @param pins Mask of pins that triggers the callback handler
 *
 *  @return None
 *
 */
static void callback_1pps(const struct device *dev, struct gpio_callback *gpio_cb, uint32_t pins)
{
	struct cxd5605_data *drv_data = CONTAINER_OF(gpio_cb, struct cxd5605_data, one_pps_gpio_cb);

	if (drv_data->pps_cb) {
		(drv_data->pps_cb)();
	}
}

/**
 * @brief Callback routine for the data ready interrupt
 * 
 * @param dev Pointer to device structure for the driver instance.
 * @param gpio_cb Pointer to gpio callback structure
 * @param pins Mask of pins that triggers the callback handler
 */
static void callback_drdy(const struct device *dev, struct gpio_callback *gpio_cb, uint32_t pins)
{
	struct cxd5605_data *drv_data = CONTAINER_OF(gpio_cb, struct cxd5605_data, data_ready_gpio_cb);

/*If thread, else work*/
#if defined(CONFIG_CXD5605_OWN_THREAD)
	k_sem_give(&drv_data->gpio_sem);
#else
	k_work_submit(&drv_data->work);
#endif /* CONFIG_CXD5605_OWN_THREAD */

}

/** @brief Get a reading from CXD5605.
 *
 *  @param dev device structure
 *  @param chan The channel to read
 *  @param val Where to store the value
 *
 *  @return Return a useful value for a particular channel, from the driver’s internal data
 *
 */
static int cxd5605_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	int32_t integral;
	int32_t frac;
	struct cxd5605_data *drv_data = dev->data;

	switch ((uint16_t)chan) {
	case SENSOR_CHAN_GNSS_POSITION:
		integral = drv_data->data.position.longitude / 10000000;
		frac = drv_data->data.position.longitude - (integral * 10000000);
		frac = (frac * 10) / 60;
		val[1].val1 = integral;
		val[1].val2 = frac;
	case SENSOR_CHAN_GNSS_POSITION_LAT:
		integral = drv_data->data.position.latitude / 10000000;
		frac = drv_data->data.position.latitude - (integral * 10000000);
		frac = (frac * 10) / 60;
		frac = (frac < 0) ? (frac * -1) : frac;
		val->val1 = integral;
		val->val2 = frac;
		break;

	case SENSOR_CHAN_GNSS_POSITION_LNG:
		integral = drv_data->data.position.longitude / 10000000;
		frac = drv_data->data.position.longitude - (integral * 10000000);
		frac = (frac * 10) / 60;
		val->val1 = integral;
		val->val2 = frac;
		break;

	case SENSOR_CHAN_ALTITUDE:
		val->val1 = (drv_data->data.position.altitude_MSL);
		val->val2 = 0;
		break;

	case SENSOR_CHAN_GNSS_TIME:
		int days = drv_data->data.time.gnss_day - 1;
		
		/* GNSS time is since 1980*/
		days += (drv_data->data.time.gnss_year - 1980) * 365 
				+ (drv_data->data.time.gnss_year - 1980) / 4
				- (drv_data->data.time.gnss_year - 1900) / 100;
		switch (drv_data->data.time.gnss_month) {
			case 12: /* December */
				days += 30;
			case 11: /* November */
				days += 31;
			case 10: /* October */
				days += 30;
			case 9:  /* September */
				days += 31;
			case 8:  /* August */
				days += 31;
			case 7:  /* July */
				days += 30;
			case 6:  /* June */
				days += 31;
			case 5:  /* May */
				days += 30;
			case 4:  /* April */
				days += 31;
			case 3:  /* March */
				days += (!(drv_data->data.time.gnss_year % 4) 
					&& (drv_data->data.time.gnss_year % 100) )? 29 : 28;
			case 2:  /* February */
				days += 31;
		}

		val->val1 = days * 86400;
		val->val1 += drv_data->data.time.gnss_hour * 3600;
		val->val1 += drv_data->data.time.gnss_minute * 60;
		val->val1 += drv_data->data.time.gnss_second;
		val->val2 = drv_data->data.time.gnss_nanosecond / 1000;

	default:
		return -ENOTSUP;
	}

	return 0;
}

/** @brief Read attribute value
 *
 *  @param dev device structure
 *  @param chan The channel the attribute belongs to
 *  depending on device capabilities.
 *  @param attr The attribute to get
 *  @param val Pointer to where to store the attribute
 *
 *  @return 0 if successful, negative errno code if failure.
 *
 */
static int cxd5605_attr_get(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr, struct sensor_value *val)
{
	struct cxd5605_data *drv_data = dev->data;

	if (chan == (enum sensor_channel)SENSOR_CHAN_GNSS_TIME &&
	    attr == (enum sensor_attribute)SENSOR_ATTR_GNSS_SECOND) {
		val->val1 = drv_data->data.time.gnss_second;
		val->val2 = 0;
	}
	if (chan == (enum sensor_channel)SENSOR_CHAN_GNSS_TIME &&
	    attr == (enum sensor_attribute)SENSOR_ATTR_GNSS_MINUTE) {
		val->val1 = drv_data->data.time.gnss_minute;
		val->val2 = 0;
	}
	if (chan == (enum sensor_channel)SENSOR_CHAN_GNSS_TIME &&
	    attr == (enum sensor_attribute)SENSOR_ATTR_GNSS_HOUR) {
		val->val1 = drv_data->data.time.gnss_hour;
		val->val2 = 0;
	}
	if (chan == (enum sensor_channel)SENSOR_CHAN_GNSS_TIME &&
	    attr == (enum sensor_attribute)SENSOR_ATTR_GNSS_DAY) {
		val->val1 = drv_data->data.time.gnss_day;
		val->val2 = 0;
	}
	if (chan == (enum sensor_channel)SENSOR_CHAN_GNSS_TIME &&
	    attr == (enum sensor_attribute)SENSOR_ATTR_GNSS_MONTH) {
		val->val1 = drv_data->data.time.gnss_month;
		val->val2 = 0;
	}
	if (chan == (enum sensor_channel)SENSOR_CHAN_GNSS_TIME &&
	    attr == (enum sensor_attribute)SENSOR_ATTR_GNSS_YEAR) {
		val->val1 = drv_data->data.time.gnss_year;
		val->val2 = 0;
	}

	if (chan == (enum sensor_channel)SENSOR_CHAN_GNSS_POSITION &&
	    attr == (enum sensor_attribute)SENSOR_ATTR_GNSS_LATITUDE) {
		val->val1 = (int)(drv_data->data.position.latitude);
		val->val2 = 0;
	}
	if (chan == (enum sensor_channel)SENSOR_CHAN_GNSS_POSITION &&
	    attr == (enum sensor_attribute)SENSOR_ATTR_GNSS_LONGITUDE) {
		val->val1 = (int)(drv_data->data.position.longitude);
		val->val2 = 0;
	}
	if (chan == (enum sensor_channel)SENSOR_CHAN_GNSS_POSITION &&
	    attr == (enum sensor_attribute)SENSOR_ATTR_GNSS_ALTITUDE_MSL) {
		sensor_value_from_double(val, drv_data->data.position.altitude_MSL);
		val->val1 = drv_data->data.position.altitude_MSL;
		val->val2 = 0;
	}
	if (chan == (enum sensor_channel)SENSOR_CHAN_GNSS_POSITION &&
	    attr == (enum sensor_attribute)SENSOR_ATTR_GNSS_ALTITUDE_HAE) {
		val->val1 = drv_data->data.position.altitude;
		val->val2 = 0;
		
	}
	if (chan == (enum sensor_channel)SENSOR_CHAN_GNSS_POSITION &&
	    attr == (enum sensor_attribute)SENSOR_ATTR_GNSS_HDOP) {
		val->val1 = drv_data->data.position.hDOP / 10;
		val->val2 = (drv_data->data.position.hDOP % 10) * 100000;
	}
	if (chan == (enum sensor_channel)SENSOR_CHAN_GNSS_POSITION &&
	    attr == (enum sensor_attribute)SENSOR_ATTR_GNSS_SIV) {
		val->val1 = (int)(drv_data->data.position.SIV);
		val->val2 = 0;
	}
	if (chan == (enum sensor_channel)SENSOR_CHAN_GNSS_POSITION &&
	    attr == (enum sensor_attribute)SENSOR_ATTR_GNSS_FIXTYPE) {
		val->val1 = (int)(drv_data->data.position.fix_type);
		val->val2 = 0;
	}

	if (chan == (enum sensor_channel)SENSOR_CHAN_GNSS_VELOCITY &&
	    attr == (enum sensor_attribute)SENSOR_ATTR_GNSS_PDOP) {
		val->val1 = (int)(drv_data->data.velocity.pDOP);
		val->val2 = (int)((drv_data->data.velocity.pDOP - val->val1) * 100);
	}

	if (chan == (enum sensor_channel)SENSOR_CHAN_GNSS_POSITION &&
	    attr == (enum sensor_attribute)SENSOR_ATTR_GNSS_VER) {
		if (val->val1 == 0) {
			val->val2 = (int)drv_data->ver.major;
		} else if (val->val1 == 1) {
			val->val2 = (int)drv_data->ver.minor;
		} else {
			val->val2 = (int)drv_data->ver.patch;
		}
	}
	return 0;
}

/** @brief Set attribute value
 *
 *  @param dev device structure
 *  @param chan The channel the attribute belongs to
 *  @param attr The attribute to set
 *  @param val value to set the attribute to
 *
 *  @return 0 if successful, negative errno code if failure
 *
 */
static int cxd5605_attr_set(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr, const struct sensor_value *val)
{
	struct cxd5605_data *drv_data = dev->data;
	int result = 0;

	uint32_t year = 0;
	uint8_t month = 0;
	uint8_t day = 0;
	uint8_t hour = 0;
	uint8_t min = 0;
	uint8_t sec = 0;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP && chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	switch ((uint16_t)attr) {
	case SENSOR_ATTR_CXD5605_ASSIST_GEN_FUNCTION_CONTROL:
		result = cxd5605_assist_gen_function_control(&drv_data->cdev, val->val1);
		if (result > SEND_BUF_SIZE || result < 0) {
			LOG_ERR("cxd5605_assist_gen_function_control error %d", result);
			return result;
		}
		break;

	case SENSOR_ATTR_CXD5605_OPERATING_MODE:
		result = cxd5605_operating_mode(&drv_data->cdev, val[0].val1, val[0].val2,
						val[1].val1);
		if (result > SEND_BUF_SIZE || result < 0) {
			LOG_ERR("cxd5605_operating_mode error %d", result);
			return result;
		}
		drv_data->op_mode = val[0].val1;
		drv_data->pos_cycle = val[0].val2;
		drv_data->sleep_time = val[1].val1;
		break;

	case SENSOR_ATTR_CXD5605_PULSE:
		if (val->val1 > 0) {
			result = cxd5605_pulse(&drv_data->cdev, 1);
			if (result > SEND_BUF_SIZE || result < 0) {
				LOG_ERR("cxd5605_pulse error %d", result);
				return result;
			}
			if (val->val2) {
				/* set 1 pps interrupt callback routine */
				drv_data->pps_cb = ((void (*)(void))val->val2);
			}
		} else {
			result = cxd5605_pulse(&drv_data->cdev, 0);
			if (result > SEND_BUF_SIZE || result < 0) {
				LOG_ERR("cxd5605_pulse error %d", result);
				return result;
			}
		}
		break;

	case SENSOR_ATTR_CXD5605_COLD_START:
		result = cxd5605_cold_start(&drv_data->cdev);
		if (result > SEND_BUF_SIZE || result < 0) {
			LOG_ERR("cxd5605_cold_start error %d", result);
			return result;
		}
		break;

	case SENSOR_ATTR_CXD5605_WAKE_UP:
		result = cxd5605_wake_up(&drv_data->cdev);
		if (result > SEND_BUF_SIZE || result < 0) {
			LOG_ERR("cxd5605_wake_up error %d", result);
			return result;
		}
		break;

	case SENSOR_ATTR_CXD5605_SENTENCE_SELECT:
		result = cxd5605_sentence_select(&drv_data->cdev, val->val1);
		if (result > SEND_BUF_SIZE || result < 0) {
			LOG_ERR("cxd5605_sentence_select error %d", result);
			return result;
		}
		drv_data->selected_sentences = val->val1;
		break;

	case SENSOR_ATTR_CXD5605_HOT_START:
		result = cxd5605_hot_start(&drv_data->cdev);
		if (result > SEND_BUF_SIZE || result < 0) {
			LOG_ERR("cxd5605_hot_start error %d", result);
			return result;
		}
		break;

	case SENSOR_ATTR_CXD5605_HOT_START_TTFF:
		result = cxd5605_hot_start_ttff(&drv_data->cdev);
		if (result > SEND_BUF_SIZE || result < 0) {
			LOG_ERR("cxd5605_hot_start_ttff error %d", result);
			return result;
		}
		break;

	case SENSOR_ATTR_CXD5605_STOP:
		result = cxd5605_stop(&drv_data->cdev);
		if (result > SEND_BUF_SIZE || result < 0) {
			LOG_ERR("cxd5605_stop error %d", result);
			return result;
		}
		break;

	case SENSOR_ATTR_CXD5605_WARM_START:
		result = cxd5605_warm_start(&drv_data->cdev);
		if (result > SEND_BUF_SIZE || result < 0) {
			LOG_ERR("cxd5605_warm_start error %d", result);
			return result;
		}
		break;

	case SENSOR_ATTR_CXD5605_TIME_SET:
		year = val[0].val1;
		month = val[0].val2;
		day = val[1].val1;
		hour = val[1].val2;
		min = val[2].val1;
		sec = val[2].val2;

		result = cxd5605_time_set(&drv_data->cdev, year, month, day, hour, min, sec);
		if (result > SEND_BUF_SIZE || result < 0) {
			LOG_ERR("cxd5605_time_set error %d", result);
			return result;
		}
		break;

	case SENSOR_ATTR_CXD5605_SLEEP:
		result = cxd5605_sleep(&drv_data->cdev, val->val1);
		if (result > SEND_BUF_SIZE || result < 0) {
			LOG_ERR("cxd5605_sleep error %d", result);
			return result;
		}
		break;

	case SENSOR_ATTR_CXD5605_VERSION:
		result = cxd5605_version(&drv_data->cdev, &drv_data->ver);
		if (result < 0) {
			LOG_ERR("[cxd5605_version error %d", result);
			return result;
		}
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

static int cxd5605_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	/* Does nothing */
	return 0;
}

/** @brief driver structure to link standard calls to routines in the driver
 */
static const struct sensor_driver_api cxd5605_driver_api = {
	.attr_set = cxd5605_attr_set,
	.attr_get = cxd5605_attr_get,
	.channel_get = cxd5605_channel_get,
	.sample_fetch = cxd5605_sample_fetch,
};

/** @brief Setup 1PPS and Alert interrupt
 *
 *  @param dev device structure
 *
 *  @return 0 if successful, negative errno code if failure
 *
 */
int cxd5605_setup_interrupts(const struct device *dev)
{
	int result;
	struct cxd5605_data *drv_data = dev->data;
	const struct cxd5605_config *config = dev->config;
	const struct gpio_dt_spec *int_gpio = &config->int_gpio;
	const struct gpio_dt_spec *alert_gpio = &config->alert_gpio;

	if (int_gpio->port) {
		/* setup 1pps interrupt */
		result = gpio_pin_configure_dt(int_gpio, GPIO_INPUT);

		if (result < 0) {
			return result;
		}

		gpio_init_callback(&drv_data->one_pps_gpio_cb, callback_1pps, BIT(int_gpio->pin));

		result = gpio_add_callback(int_gpio->port, &drv_data->one_pps_gpio_cb);

		if (result < 0) {
			return result;
		}

		result = gpio_pin_interrupt_configure_dt(int_gpio, GPIO_INT_EDGE_RISING);

		if (result < 0) {
			return result;
		}
	}
	if (alert_gpio->port) {
		/* setup drdy interrupt */
		result = gpio_pin_configure_dt(alert_gpio, GPIO_INPUT);

		if (result < 0) {
			return result;
		}

		gpio_init_callback(&drv_data->data_ready_gpio_cb, callback_drdy, BIT(alert_gpio->pin));

		result = gpio_add_callback(alert_gpio->port, &drv_data->data_ready_gpio_cb);

		if (result < 0) {
			return result;
		}

		result = gpio_pin_interrupt_configure_dt(alert_gpio, GPIO_INT_EDGE_RISING);

		if (result < 0) {
			return result;
		}
	}
	cxd5605_register_nmea_callback(&drv_data->cdev, driver_cxd5605_nmea_cb);

	return 0;
}

/** @brief power mode action routine
 *
 *  @param dev device structure
 *  @param power mode action to take
 *
 *  @return 0 if successful, negative errno code if failure
 *
 */
#ifdef CONFIG_PM_DEVICE
static int cxd5605_driver_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct cxd5605_config *config = dev->config;
	const struct gpio_dt_spec *rst_gpio = &config->rst_gpio;
	struct cxd5605_data *drv_data = dev->data;

	int result = 0;

	switch (action) {
	case PM_DEVICE_ACTION_TURN_ON:
		/*
		 * powered on the device, used when the power
		 * domain this device belongs is resumed.
		 */
		result = gpio_pin_configure_dt(rst_gpio, GPIO_OUTPUT_HIGH);
		k_msleep(3000); /* wait for GNSS chip to boot */
		result = chip_init(dev);
		if (result < 0) {
			printk("ERROR: I2C interface not working (CXD5605 driver)\n");
		}
		if (drv_data->pps_cb) {
			cxd5605_pulse(&drv_data->cdev, 1);
		}
		cxd5605_operating_mode(&drv_data->cdev, drv_data->op_mode, drv_data->pos_cycle,
				       drv_data->sleep_time);
		cxd5605_sentence_select(&drv_data->cdev, drv_data->selected_sentences);

	case PM_DEVICE_ACTION_SUSPEND:
		cxd5605_sleep(&drv_data->cdev, 0);
		break;
	case PM_DEVICE_ACTION_RESUME:
		cxd5605_wake_up(&drv_data->cdev);
		break;
	case PM_DEVICE_ACTION_TURN_OFF:
		/*
		 * power off the device, used when the power
		 * domain this device belongs is suspended.
		 */
		cxd5605_sleep(&drv_data->cdev, 2);
		gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_DISABLE);
		result = gpio_pin_configure_dt(rst_gpio, GPIO_OUTPUT_LOW);
		break;
	default:
		return -ENOTSUP;
	}

	return result;
}
#endif /* CONFIG_PM_DEVICE */

#define CXD5605_DEFINE(inst)                                                                       \
	static struct cxd5605_config cxd5605_config_##inst = {                                     \
		.i2c_spec = I2C_DT_SPEC_INST_GET(inst),                                            \
		.alert_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, alert_gpios, {0}),                    \
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),                        \
		.rst_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, rst_gpios, {0}),                        \
		.boot_rec_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, boot_rec_gpios, {0})};             \
	K_SEM_DEFINE(cxd5605_resp_sem_##inst, 0, 1);                                                \
	static struct cxd5605_data cxd5605_prv_data_##inst = {                                     \
		.cdev = CXD5605_DEV_DEFINE(&cxd5605_resp_sem_##inst, &cxd5605_config_##inst),           \
	};                                                                                         \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(inst, cxd5605_driver_pm_action);                                  \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &init, PM_DEVICE_DT_INST_GET(inst), &cxd5605_prv_data_##inst,  \
			      &cxd5605_config_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,    \
			      &cxd5605_driver_api);

/* Current lib version only supports one instance */
DT_INST_FOREACH_STATUS_OKAY(CXD5605_DEFINE);
