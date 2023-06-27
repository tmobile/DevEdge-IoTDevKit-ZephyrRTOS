/*
 * Copyright 2032 T-Mobile
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @defgroup driver_sensor_subsys_tests sensor_subsys
 * @ingroup all_tests
 * @{
 * @}
 */

#include <zephyr/ztest.h>
#include <zephyr/drivers/sensor.h>

struct sensor_als_fixture {
	const struct device *als_i2c;
};

static enum sensor_channel channel[] = {
	SENSOR_CHAN_LIGHT,
	SENSOR_CHAN_IR,
};

static void test_sensor_als_basic(const struct device *dev)
{
	zassert_equal(sensor_sample_fetch(dev), 0, "fail to fetch sample");

	for (int i = 0; i < ARRAY_SIZE(channel); i++) {
		struct sensor_value val;

		zassert_ok(sensor_channel_get(dev, channel[i], &val),
			   "fail to get channel");
	}
}

/* Run all of our tests on an als device with the given label */
static void run_tests_on_als(const struct device *als)
{
	zassert_true(device_is_ready(als), "als device is not ready");

	PRINT("Running tests on '%s'\n", als->name);
	k_object_access_grant(als, k_current_get());
}


ZTEST_USER_F(sensor_als, test_sensor_als_basic_i2c)
{
	if (fixture->als_i2c == NULL) {
		ztest_test_skip();
	}

	run_tests_on_als(fixture->als_i2c);
	test_sensor_als_basic(fixture->als_i2c);
}

static void *sensor_als_setup(void)
{
	static struct sensor_als_fixture fixture = {
		.als_i2c = DEVICE_DT_GET_OR_NULL(DT_ALIAS(als_0))
	};

	return &fixture;
}

ZTEST_SUITE(sensor_als, NULL, sensor_als_setup, NULL, NULL, NULL);
