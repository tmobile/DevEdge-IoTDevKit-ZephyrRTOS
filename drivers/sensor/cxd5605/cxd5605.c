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
#include <time.h>

#include "cxd5605.h"
#include "cxd5605_lib.h"

extern char  *strtok_r(char *str, const char *sep, char **state);

LOG_MODULE_REGISTER(CXD5605, CONFIG_SENSOR_LOG_LEVEL);

#define SEND_BUF_SIZE 32

#define CXD5605_ADDRESS DT_N_S_soc_S_i2c_4000c400_S_sonycxd5605_24_REG_IDX_0_VAL_ADDRESS

#define CXD5605_LLE_XFER_SIZE 2048
#define MAXFIELDS 32

#define COMMAND_STR_LENGTH 12

struct cxd5605_config {
	const struct i2c_dt_spec i2c_spec;
	const struct gpio_dt_spec alert_gpio;
	const struct gpio_dt_spec int_gpio;
	const struct gpio_dt_spec pwr_gpio;
	const struct gpio_dt_spec rst_gpio;
	const struct gpio_dt_spec boot_rec_gpio;
};

struct drv_data {
	struct gpio_callback gpio_cb;
	gpio_flags_t mode;
	int index;
	int aux;
};

static void driver_cxd5605_nmea_cb(struct gnss_global_data *pvt)
{
#ifdef DEBUG
	memcpy(&drv_data->pvt, pvt, sizeof(struct gnss_global_data));

	printf("[Driver] %s:%d - gnss time: %d:%d:%d:%d\n", __func, __LINE__,
			drv_data->pvt.time.gnss_hour,
			drv_data->pvt.time.gnss_minute,
			drv_data->pvt.time.gnss_second,
			drv_data->pvt.time.gnss_nanosecond);

	printf("[Driver] %s:%d - position: lat %d, long %d, alt %d\n",
			__func, __LINE__,
			drv_data->pvt.position.latitude,
			drv_data->pvt.position.longitude,
			drv_data->pvt.position.altitude);
	printf("[Driver] %s:%d - alt (MSL) %d\ntype %d, SIV %d, HAccuracy %d\n",
			__func, __LINE__,
			drv_data->pvt.position.altitude_MSL,
			drv_data->pvt.position.fix_type,
			drv_data->pvt.position.SIV,
			drv_data->pvt.position.horizontal_accuracy);
#endif
}

/** @brief response call back routine
 *
 * This routine is called by the cxd5605 library when there is a response
 * to a command.  Right now there aren't many responses that need attention.
 * We just have the version command that has a response to it.
 *
 *  @param dev The pointer to the device structure for this driver
 *  @param the cxd5605_cmd_data struct for response
 *  @param the command we are working on
 *  @param and the ret value from that command
 *
 *  @return nothing
 *
 */
static void driver_cxd5605_resp_cb(struct cxd5605_data *drv_data,
		struct cxd5605_cmd_data *cxd5605_cmd_data,
		int cmd, int ret)
{
#ifdef DEBUG
	printf("[driver] command %d err %d\n", cmd, ret);
	printf("[driver] drv_data = 0x%0X", drv_data);
#endif

	if (ret) {
		return;
	}

	drv_data->got_data = true;
	switch (cmd) {
	/* All commands that returns data will be handled here */
	case CMD_VERSION:
		memcpy(&drv_data->cxd5605_cmd_data.ver,
			&cxd5605_cmd_data->ver,
			sizeof(struct cxd5605_version));
#ifdef DEBUG
		printf("[driver] maj %d, minor %d patch %d\n",
			drv_data->cxd5605_cmd_data.ver.major,
			drv_data->cxd5605_cmd_data.ver.minor,
			drv_data->cxd5605_cmd_data.ver.patch);
#endif
		break;
	default:
		break;
	}
}

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

	drv_data->pvt.position.latitude = 0.0;
	drv_data->pvt.position.longitude = 0.0;
	drv_data->pvt.position.altitude_MSL = 0.0;
	drv_data->pvt.position.altitude = 0.0;

	drv_data->cxd5605_cmd = -1;
	drv_data->num_msg = 0;

	if (!device_is_ready(cfg->i2c_spec.bus)) {
		LOG_ERR("I2C dev %s not ready", cfg->i2c_spec.bus->name);
		return -ENODEV;
	}

	/* save this driver instance for passing to other functions */
	drv_data->cxd5605_dev = dev;

	result = gpio_pin_configure_dt(&cfg->rst_gpio, GPIO_OUTPUT_HIGH);

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

	while ((fields[field++] = (char *)strtok_r(string, separates, &string))
		&& (field < MAXFIELDS)) {
	}

	return (field-1);
}

/** @brief wait for command response from the CXD5605
 *
 * @param requires a struct device
 *
 * @return status 0 good -1 timeout
 */
static int cxd5605_wait_fetch(const struct device *dev)
{
	int ret = 0;
	int data_timeout = 0;

	struct cxd5605_data *drv_data = dev->data;

#ifdef CONFIG_PM_DEVICE
	enum pm_device_state state;
	(void)pm_device_state_get(dev, &state);
	/* Do not allow sample fetching from suspended state */
	if (state == PM_DEVICE_STATE_SUSPENDED) {
		return -EIO;
	}
#endif

	drv_data->got_data = false;
	while (drv_data->got_data != true) {
		k_msleep(100);
		data_timeout++;
		if (data_timeout > 13) {
			return -1;
		}
	}

	return ret;
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
static void callback_1pps(const struct device *dev,
			 struct gpio_callback *gpio_cb, uint32_t pins)
{
	struct cxd5605_data *drv_data = CONTAINER_OF(gpio_cb, struct cxd5605_data, one_pps_gpio_cb);

	if (drv_data->pps_cb)
		(drv_data->pps_cb)();
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
static int cxd5605_channel_get(const struct device *dev,
		enum sensor_channel chan,
		struct sensor_value *val)
{
	int32_t integral;
	int32_t frac;
	struct cxd5605_data *drv_data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_POS_DX:
		integral = (int)(drv_data->pvt.position.latitude)/10000000.0;
		frac = drv_data->pvt.position.latitude - (integral*10000000);
		frac = (frac / 60.0) * 100.0;
		frac = (frac < 0) ? (frac * -1) : frac;
		val->val1 = integral;
		val->val2 = frac / 10;
		break;

	case SENSOR_CHAN_POS_DY:
		integral = (int)(drv_data->pvt.position.longitude)/10000000.0;
		frac = drv_data->pvt.position.longitude - (integral*10000000);
		frac = (frac / 60.0) * 100.0;
		val->val1 = integral;
		val->val2 = frac / 10;
		break;

	case SENSOR_CHAN_ALTITUDE:
		val->val1 = (drv_data->pvt.position.altitude_MSL);
		val->val2 = 0;
		break;

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
static int cxd5605_attr_get(const struct device *dev,
			   enum sensor_channel chan,
			   enum sensor_attribute attr,
			   struct sensor_value *val)
{
	struct cxd5605_data *drv_data = dev->data;

	if (chan == (enum sensor_channel)GNSS_CHANNEL_TIME &&
			attr == (enum sensor_attribute)GNSS_ATTRIBUTE_TIME) {
		val->val1 = (int)((drv_data->pvt.time.gnss_hour * 10000) +
				(drv_data->pvt.time.gnss_minute * 100) +
				(drv_data->pvt.time.gnss_second));
		val->val2 = 0;
	}
	if (chan == (enum sensor_channel)GNSS_CHANNEL_TIME &&
			attr == (enum sensor_attribute)GNSS_ATTRIBUTE_DAY) {
		val->val1 = (int)drv_data->pvt.time.gnss_day;
		val->val2 = 0;
	}
	if (chan == (enum sensor_channel)GNSS_CHANNEL_TIME &&
			attr == (enum sensor_attribute)GNSS_ATTRIBUTE_MONTH) {
		val->val1 = (int)drv_data->pvt.time.gnss_month;
		val->val2 = 0;
	}
	if (chan == (enum sensor_channel)GNSS_CHANNEL_TIME &&
			attr == (enum sensor_attribute)GNSS_ATTRIBUTE_YEAR) {
		val->val1 = (int)drv_data->pvt.time.gnss_year;
		val->val2 = 0;
	}
	if (chan == (enum sensor_channel)GNSS_CHANNEL_TIME &&
			attr == (enum sensor_attribute)GNSS_ATTRIBUTE_UTC) {
		struct tm new_time;
		new_time.tm_sec = drv_data->pvt.time.gnss_second;
		new_time.tm_min = drv_data->pvt.time.gnss_minute;
		new_time.tm_hour = drv_data->pvt.time.gnss_hour;
		new_time.tm_mon= drv_data->pvt.time.gnss_month;
		new_time.tm_year = drv_data->pvt.time.gnss_year;
		new_time.tm_isdst = -1;
		val->val1 = (int)mktime(&new_time);
		val->val2 = 0;
	}

	if (chan == (enum sensor_channel)GNSS_CHANNEL_POSITION &&
			attr == (enum sensor_attribute)GNSS_ATTRIBUTE_LATITUDE) {
		val->val1 = (int)(drv_data->pvt.position.latitude);
		val->val2 = 0;
	}
	if (chan == (enum sensor_channel)GNSS_CHANNEL_POSITION &&
			attr == (enum sensor_attribute)GNSS_ATTRIBUTE_LONGITUDE) {
		val->val1 = (int)(drv_data->pvt.position.longitude);
		val->val2 = 0;
	}
	if (chan == (enum sensor_channel)GNSS_CHANNEL_POSITION &&
			attr == (enum sensor_attribute)GNSS_ATTRIBUTE_ALTITUDE_MSL) {
		val->val1 = (int)(drv_data->pvt.position.altitude_MSL);
		val->val2 = (int)((drv_data->pvt.position.altitude_MSL-val->val1) * 100);
	}
	if (chan == (enum sensor_channel)GNSS_CHANNEL_POSITION &&
			attr == (enum sensor_attribute)GNSS_ATTRIBUTE_ALTITUDE_HAE) {
		val->val1 = (int)(drv_data->pvt.position.altitude);
		val->val2 = (int)((drv_data->pvt.position.altitude-val->val1) * 100);
	}
	if (chan == (enum sensor_channel)GNSS_CHANNEL_POSITION &&
			attr == (enum sensor_attribute)GNSS_ATTRIBUTE_HDOP) {
		val->val1 = (int)(drv_data->pvt.position.horizontal_accuracy / 10);
		val->val2 = (int)(drv_data->pvt.position.horizontal_accuracy%10);
	}
	if (chan == (enum sensor_channel)GNSS_CHANNEL_POSITION &&
			attr == (enum sensor_attribute)GNSS_ATTRIBUTE_SIV) {
		val->val1 = (int)(drv_data->pvt.position.SIV);
		val->val2 = 0;
	}
	if (chan == (enum sensor_channel)GNSS_CHANNEL_POSITION &&
			attr == (enum sensor_attribute)GNSS_ATTRIBUTE_FIXTYPE) {
		cxd5605_wait_fetch(dev);
		val->val1 = (int)(drv_data->pvt.position.fix_type);
		val->val2 = 0;
	}

	if (chan == (enum sensor_channel)GNSS_CHANNEL_VELOCITY &&
			attr == (enum sensor_attribute)GNSS_ATTRIBUTE_PDOP) {
		val->val1 = (int)(drv_data->pvt.velocity.pDOP);
		val->val2 = (int)((drv_data->pvt.velocity.pDOP-val->val1) * 100);
	}

	if (chan == (enum sensor_channel)GNSS_CHANNEL_POSITION &&
			attr == (enum sensor_attribute)GNSS_ATTRIBUTE_VER) {
		if (val->val1 == 0)
			val->val2 = (int)drv_data->cxd5605_cmd_data.ver.major;
		else if (val->val1 == 1)
			val->val2 = (int)drv_data->cxd5605_cmd_data.ver.minor;
		else
			val->val2 = (int)drv_data->cxd5605_cmd_data.ver.patch;
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
static int cxd5605_attr_set(const struct device *dev,
			   enum sensor_channel chan,
			   enum sensor_attribute attr,
			   const struct sensor_value *val)
{
	struct cxd5605_data *drv_data = dev->data;
	int result = 0;

	uint32_t year = 0;
	uint8_t month = 0;
	uint8_t day = 0;
	uint8_t hour = 0;
	uint8_t min = 0;
	uint8_t sec = 0;


	const struct cxd5605_config *config = dev->config;
	const struct gpio_dt_spec *pwr_gpio = &config->pwr_gpio;
	const struct gpio_dt_spec *rst_gpio = &config->rst_gpio;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP && chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	drv_data->cxd5605_cmd = (int)attr;
	LOG_DBG("[cxd5605:tx] - cmd (%d)\r\n", drv_data->cxd5605_cmd);

	switch (drv_data->cxd5605_cmd) {

	case SENSOR_ATTR_CXD5605_ASSIST_GEN_FUNCTION_CONTROL:
		result = cxd5605_assist_gen_function_control(dev, val->val1);
		if (result > SEND_BUF_SIZE || result < 0) {
			LOG_ERR("[%s:%d] to_send buffer error %d",
				__FILE__, __LINE__, result);
			return result;
		}
		break;

	case SENSOR_ATTR_CXD5605_OPERATING_MODE:
		result = cxd5605_operating_mode(dev, val[0].val1, val[0].val2, val[1].val1);
		if (result > SEND_BUF_SIZE || result < 0) {
			LOG_ERR("[%s:%d] to_send buffer error %d",
				__FILE__, __LINE__, result);
			return result;
		}
		break;

	case SENSOR_ATTR_CXD5605_PULSE:
		if (val->val1 > 0) {
			result = cxd5605_pulse(dev, 1);
			if (result > SEND_BUF_SIZE || result < 0) {
				LOG_ERR("[%s:%d] to_send buffer error %d",
					__FILE__, __LINE__, result);
				return result;
			}
			if (val->val2) {
				/* set 1 pps interrupt callback routine */
				drv_data->pps_cb = ((void (*)(void))val->val2);
			}
		} else {
			result = cxd5605_pulse(dev, 0);
			if (result > SEND_BUF_SIZE || result < 0) {
				LOG_ERR("[%s:%d] to_send buffer error %d",
					__FILE__, __LINE__, result);
				return result;
			}
		}
		break;

	case SENSOR_ATTR_CXD5605_COLD_START:
		result = cxd5605_cold_start(dev);
		if (result > SEND_BUF_SIZE || result < 0) {
			LOG_ERR("[%s:%d] to_send buffer error %d",
				__FILE__, __LINE__, result);
			return result;
		}
		break;

	case SENSOR_ATTR_CXD5605_WAKE_UP:
		result = cxd5605_wake_up(dev);
		if (result > SEND_BUF_SIZE || result < 0) {
			LOG_ERR("[%s:%d] to_send buffer error %d",
				__FILE__, __LINE__, result);
			return result;
		}
		break;

	case SENSOR_ATTR_CXD5605_SENTENCE_SELECT:
		result = cxd5605_sentence_select(dev, val->val1);
		if (result > SEND_BUF_SIZE || result < 0) {
			LOG_ERR("[%s:%d] to_send buffer error %d",
				__FILE__, __LINE__, result);
			return result;
		}
		break;

	case SENSOR_ATTR_CXD5605_HOT_START:
		result = cxd5605_hot_start(dev);
		if (result > SEND_BUF_SIZE || result < 0) {
			LOG_ERR("[%s:%d] to_send buffer error %d",
				__FILE__, __LINE__, result);
			return result;
		}
		break;

	case SENSOR_ATTR_CXD5605_HOT_START_TTFF:
		result = cxd5605_hot_start_ttff(dev);
		if (result > SEND_BUF_SIZE || result < 0) {
			LOG_ERR("[%s:%d] to_send buffer error %d",
				__FILE__, __LINE__, result);
			return result;
		}
		break;

	case SENSOR_ATTR_CXD5605_STOP:
		result = cxd5605_stop(dev);
		if (result > SEND_BUF_SIZE || result < 0) {
			LOG_ERR("[%s:%d] to_send buffer error %d",
				__FILE__, __LINE__, result);
			return result;
		}
		break;

	case SENSOR_ATTR_CXD5605_WARM_START:
		result = cxd5605_warm_start(dev);
		if (result > SEND_BUF_SIZE || result < 0) {
			LOG_ERR("[%s:%d] to_send buffer error %d",
				__FILE__, __LINE__, result);
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

		result = cxd5605_time_set(dev, year, month, day, hour, min, sec);
		if (result > SEND_BUF_SIZE || result < 0) {
			LOG_ERR("[%s:%d] to_send buffer error %d",
				__FILE__, __LINE__, result);
			return result;
		}
		break;

	case SENSOR_ATTR_CXD5605_SLEEP:
		result = cxd5605_sleep(dev, val->val1);
		if (result > SEND_BUF_SIZE || result < 0) {
			LOG_ERR("[%s:%d] to_send buffer error %d",
				__FILE__, __LINE__, result);
			return result;
		}
		break;

	case SENSOR_ATTR_CXD5605_VERSION:
		result = cxd5605_version(dev);
		if (result < 0) {
			LOG_ERR("[%s:%d] to_send buffer error %d",
				__FILE__, __LINE__, result);
			return result;
		}
		break;

	case SENSOR_ATTR_CXD5605_PWR_CTRL:
		if (val->val1 == 0) {
			result = gpio_pin_configure_dt(pwr_gpio, GPIO_OUTPUT_LOW);
			result = gpio_pin_configure_dt(rst_gpio, GPIO_OUTPUT_LOW);
		} else {
			result = gpio_pin_configure_dt(pwr_gpio, GPIO_OUTPUT_HIGH);
			result = gpio_pin_configure_dt(rst_gpio, GPIO_OUTPUT_HIGH);
		}
		break;

	case SENSOR_ATTR_CXD5605_CALLBACK:
		init(dev);
		LOG_DBG("Got CXD5605_ALERT_INTERRUPTS\n");
		cxd5605_setup_interrupts(dev);
		/* setup shim callbacks */
#ifdef DEBUG
		printf("[driver] register driver callback\n");
#endif
		cxd5605_lib_init(dev);
		cxd5605_register_resp_callback(dev, driver_cxd5605_resp_cb);
		cxd5605_register_nmea_callback(dev, driver_cxd5605_nmea_cb);
		return 0;

	default:
		return -ENOTSUP;
	}

	if (drv_data->cxd5605_cmd != SENSOR_ATTR_CXD5605_PWR_CTRL) {
		result = cxd5605_wait_fetch(dev);
		if (result < 0) {
			return result;
		}
	}

	return 0;
}

/** @brief driver structure to link standard calls to routines in the driver
 */
static const struct sensor_driver_api cxd5605_driver_api = {
	.attr_set = cxd5605_attr_set,
	.attr_get = cxd5605_attr_get,
	.channel_get = cxd5605_channel_get,
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

	if (config->alert_gpio.port) {
		/* setup 1pps interrupt */
		result = gpio_pin_configure_dt(int_gpio, GPIO_INPUT);

		if (result < 0) {
			return result;
		}

		gpio_init_callback(&drv_data->one_pps_gpio_cb,
				callback_1pps,
				BIT(int_gpio->pin));

		result = gpio_add_callback(int_gpio->port,
				&drv_data->one_pps_gpio_cb);

		if (result < 0) {
			return result;
		}

		result = gpio_pin_interrupt_configure_dt(int_gpio,
				GPIO_INT_EDGE_RISING);

		if (result < 0) {
			return result;
		}
	}

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
static int cxd5605_driver_pm_action(const struct device *dev,
		enum pm_device_action action)
{
	const struct cxd5605_config *config = dev->config;
	const struct gpio_dt_spec *rst_gpio = &config->rst_gpio;

	int result = 0;

	switch (action) {
	case PM_DEVICE_ACTION_TURN_ON:
		/*
		 * powered on the device, used when the power
		 * domain this device belongs is resumed.
		 */
		result = gpio_pin_configure_dt(rst_gpio, GPIO_OUTPUT_HIGH);
		k_msleep(3000);	/* wait for GNSS chip to boot */
		result = init(dev);
		if (result < 0) {
			printk("ERROR: I2C interface not working (CXD5605 driver)\n");
		}

	case PM_DEVICE_ACTION_SUSPEND:
		cxd5605_sleep(dev, 0);
		break;
	case PM_DEVICE_ACTION_RESUME:
		cxd5605_wake_up(dev);
		break;
	case PM_DEVICE_ACTION_TURN_OFF:
		/*
		 * power off the device, used when the power
		 * domain this device belongs is suspended.
		 */
		cxd5605_sleep(dev, 2);
		gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_DISABLE);
		result = gpio_pin_configure_dt(rst_gpio, GPIO_OUTPUT_LOW);
		break;
	default:
		return -ENOTSUP;
	}

	return result;
}
#endif /* CONFIG_PM_DEVICE */

#define CXD5605_DEFINE(inst)							\
	static struct cxd5605_data cxd5605_prv_data_##inst;			\
	static struct cxd5605_config cxd5605_config_##inst = {			\
		.i2c_spec = I2C_DT_SPEC_INST_GET(inst),				\
		.alert_gpio = GPIO_DT_SPEC_INST_GET_OR(inst,			\
						       alert_gpios, { 0 }),	\
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst,			\
						       int_gpios, { 0 }),	\
		.pwr_gpio = GPIO_DT_SPEC_INST_GET_OR(inst,			\
							pwr_gpios, { 0 }),	\
		.rst_gpio = GPIO_DT_SPEC_INST_GET_OR(inst,			\
							rst_gpios, { 0 }),	\
		.boot_rec_gpio = GPIO_DT_SPEC_INST_GET_OR(inst,			\
							boot_rec_gpios, { 0 })	\
	};									\
										\
	PM_DEVICE_DT_INST_DEFINE(inst, cxd5605_driver_pm_action);		\
										\
	DEVICE_DT_INST_DEFINE(inst,						\
		&init,								\
			      PM_DEVICE_DT_INST_GET(inst),			\
			      &cxd5605_prv_data_##inst,				\
			      &cxd5605_config_##inst,				\
			      POST_KERNEL,					\
			      CONFIG_SENSOR_INIT_PRIORITY,			\
			      &cxd5605_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CXD5605_DEFINE)
