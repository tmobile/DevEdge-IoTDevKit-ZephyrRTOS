/*
 * Copyright (c) 2022 T-Mobile Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/gnss.h>
#include <zephyr/drivers/sensor/cxd5605.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <zephyr/init.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/drivers/sensor/gnss.h>
#include <zephyr/fs/fs.h>

#define	ST_WAIT_FOR_FIX 1
#define	ST_STOP_GNSS 2
#define	ST_UPDATE_TIME 3
#define	ST_COLD_START 4
#define	ST_WARM_START 5
#define	ST_ASSIST_DATA_GEN 6
#define	ST_ASSIST_STATUS 7
#define	NEXT_FIX 8
#define	NEXT_FIX1 9

static uint32_t fix_time_sec;

void call_back_1PPS(void)
{
#ifdef DEBUG
	printf("%s:%d - User 1PPS\n", __func__, __LINE__);
#endif
}

void hot_start_handler(struct k_work *work)
{
	fix_time_sec++;
}

K_WORK_DEFINE(hot_start, hot_start_handler);

void hot_start_timer_handler(struct k_timer *dummy)
{
	k_work_submit(&hot_start);
}

K_TIMER_DEFINE(hot_start_timer, hot_start_timer_handler, NULL);

int main(void)
{
	int fix_state = ST_ASSIST_DATA_GEN;
	struct sensor_value temp_flags;
	const struct device *cxd5605;
	int rc;
	struct sensor_value sensValues;

	fix_time_sec = 0;

	printf("Sony CXD5605 Hot Fix Example, %s\n", CONFIG_ARCH);

	cxd5605 = DEVICE_DT_GET_ANY(sony_cxd5605);

	if (!cxd5605) {
		printf("cxd5605 driver error\n");
		return 1;
	}

	k_msleep(1000);

	sensValues.val1 = 1;
	sensValues.val2 = 38;
	rc = sensor_attr_set(cxd5605, SENSOR_CHAN_ALL,
			SENSOR_ATTR_CXD5605_CALLBACK, &sensValues);

	printf("Reading NMEA sentences\n");

	/* wait 3 seconds for GNSS to boot*/
	k_msleep(3000);

	printf("cxd5605 booted up\n");
	rc = sensor_attr_set(cxd5605, SENSOR_CHAN_ALL,
			SENSOR_ATTR_CXD5605_VERSION, NULL);
	k_msleep(200);
	struct sensor_value op_mode[3] = {{1, 2000}, {0, 0}};

	rc = sensor_attr_set(cxd5605, SENSOR_CHAN_ALL,
			SENSOR_ATTR_CXD5605_OPERATING_MODE, op_mode);
	k_msleep(200);
	sensValues.val1 = 0x01;
	rc = sensor_attr_set(cxd5605, SENSOR_CHAN_ALL,
			SENSOR_ATTR_CXD5605_SENTENCE_SELECT, &sensValues);
	k_msleep(200);
	sensValues.val1 = 1;
	sensValues.val2 = (int32_t)call_back_1PPS;
	rc = sensor_attr_set(cxd5605, SENSOR_CHAN_ALL,
			SENSOR_ATTR_CXD5605_PULSE, &sensValues);
	k_msleep(200);
	sensValues.val1 = 1;
	rc = sensor_attr_set(cxd5605, SENSOR_CHAN_ALL,
			SENSOR_ATTR_CXD5605_ASSIST_GEN_FUNCTION_CONTROL, &sensValues);
	k_msleep(200);
	struct sensor_value time[3] = {{2022, 8}, {31, 07}, {53, 30}};

	rc = sensor_attr_set(cxd5605, SENSOR_CHAN_ALL,
			SENSOR_ATTR_CXD5605_TIME_SET, time);
	k_msleep(200);
	k_msleep(4000);
	rc = sensor_attr_set(cxd5605, SENSOR_CHAN_ALL,
			SENSOR_ATTR_CXD5605_WARM_START, &sensValues);
	k_timer_start(&hot_start_timer, K_SECONDS(1), K_SECONDS(1));

	fix_state = ST_WAIT_FOR_FIX;

	while (1) {
		switch (fix_state) {
		case ST_WAIT_FOR_FIX:
			k_msleep(1000);
			sensor_attr_get(cxd5605,
					SENSOR_CHAN_GNSS_POSITION,
					SENSOR_ATTR_GNSS_FIXTYPE,
					&temp_flags);
			if (temp_flags.val1 >= 1) {
				temp_flags.val1 = 0;
				temp_flags.val2 = 0;

				printf("%s:%d - got fix! %d sec\n",
						__func__, __LINE__, fix_time_sec);
				fix_state = ST_STOP_GNSS;
			}
			break;

		case ST_ASSIST_DATA_GEN:
			k_msleep(1000);
			printf("%s:%d - Generate satellite assist data\n", __func__, __LINE__);
			sensValues.val1 = 1;
			rc = sensor_attr_set(cxd5605, SENSOR_CHAN_ALL,
					SENSOR_ATTR_CXD5605_ASSIST_GEN_FUNCTION_CONTROL,
					&sensValues);
			fix_state = ST_UPDATE_TIME;
			break;

		case ST_STOP_GNSS:
			k_msleep(1000);
			printf("%s:%d - Stop gnss\n", __func__, __LINE__);
			rc = sensor_attr_set(cxd5605, SENSOR_CHAN_ALL,
					SENSOR_ATTR_CXD5605_STOP, &temp_flags);
			k_msleep(200);
			fix_state = ST_WARM_START;
			break;

		case ST_UPDATE_TIME:
			k_msleep(1000);
			printf("%s:%d - Update time\n", __func__, __LINE__);
			struct sensor_value time[3] = {{2022, 8}, {29, 16}, {11, 30}};

			rc = sensor_attr_set(cxd5605, SENSOR_CHAN_ALL,
					SENSOR_ATTR_CXD5605_TIME_SET, time);
			fix_state = ST_ASSIST_STATUS;
			break;

		case ST_COLD_START:
			k_msleep(1000);
			rc = sensor_attr_set(cxd5605, SENSOR_CHAN_ALL,
					SENSOR_ATTR_CXD5605_COLD_START, &sensValues);

			fix_time_sec = 0;
			k_timer_start(&hot_start_timer, K_SECONDS(1), K_SECONDS(1));
			printf("%s:%d - Cold start GNSS\n", __func__, __LINE__);
			fix_state = ST_WAIT_FOR_FIX;
			break;

		case ST_WARM_START:
			k_msleep(1000);
			rc = sensor_attr_set(cxd5605, SENSOR_CHAN_ALL,
					SENSOR_ATTR_CXD5605_WARM_START, &sensValues);
			k_msleep(4000);

			fix_time_sec = 0;
			k_timer_start(&hot_start_timer, K_SECONDS(1), K_SECONDS(1));
			printf("%s:%d -Wwarm start GNSS\n", __func__, __LINE__);
			fix_state = ST_WAIT_FOR_FIX;
			break;

		case NEXT_FIX1:
			k_msleep(5000);
			fix_state = ST_WAIT_FOR_FIX;
			break;

		default:
			fix_state = ST_WAIT_FOR_FIX;
			break;

		}

		k_msleep(500);
	}
}
