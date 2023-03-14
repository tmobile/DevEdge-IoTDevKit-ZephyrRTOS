/*
 * Copyright (c) 2020 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/adc/gecko_adc/gecko_adc.h>
#include "battery_ctrl.h"

#define ADC_DEVICE_NODE		DT_INST(0, silabs_adc_gecko)

static void battery_apply_filter(float *bv)
{
	static float s_filtered_capacity = -1;
	static bool s_battery_is_charging = false;
	bool battery_is_charging;

	// If there has been a switch between charger and battery, reset the filter
	battery_is_charging = is_battery_charging();
	if (s_battery_is_charging != battery_is_charging) {
		s_battery_is_charging = battery_is_charging;
		s_filtered_capacity = -1;
	}

	if (s_filtered_capacity < 0) {
		s_filtered_capacity = *bv;
	}
	*bv = s_filtered_capacity = s_filtered_capacity * 0.95 + (*bv) * 0.05;
}

static uint8_t battery_millivolts_to_percent(uint32_t millivolts) {
	float curBv = get_remaining_capacity((float) millivolts / 1000);
	battery_apply_filter(&curBv);
	return (uint8_t) (curBv + 0.5);
}

void main(void)
{
	const struct device *const batt_dev = DEVICE_DT_GET(ADC_DEVICE_NODE);
	int32_t buffer;
	int err;
	uint8_t charging;
	uint8_t vbus;
	uint8_t battery_attached;
	uint8_t fault;
	uint8_t percent;
	uint32_t millivolts = 0;

	struct adc_gecko_data *data = batt_dev->data;


	static const struct adc_channel_cfg ch_cfg = {
		.reference        = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 32),
		.channel_id       = 2,
		.differential = false
	};

	printk("%s:%d - GECKO BATTERY SAMPLE Starting...\n", __FUNCTION__,__LINE__);
	if (!device_is_ready(batt_dev)) {
		printk(" device not ready");
		return;
	}

	err = adc_channel_setup(batt_dev, &ch_cfg);
	if (err) {
		printk("failed to setup ADC channel (err %d)", err);
		return;
	}

	k_msleep(5000);

	struct adc_sequence seq = {
		.buffer = &buffer,
		.buffer_size = sizeof(buffer),
		.resolution = 12
	};


	k_msleep(5000);

	while (true) {
		seq.channels = 1; // HWID
		err = adc_read(batt_dev, &seq);
		if (err >= 0) {
			millivolts = (uint32_t)(3.0 * ((data->mVolts * 2500.0) / 4096.0) + 0.5);
			printk("HWID %d\n", data->mVolts);
		}

		seq.channels = 0;
		get_battery_charging_status(&charging, &vbus, &battery_attached, &fault);

		if (battery_attached != 0) {
			err = adc_read(batt_dev, &seq);
			millivolts = (uint32_t)(3.0 * ((data->mVolts * 2500.0) / 4096.0) + 0.5);
			printk("VBATT (volts) %dmV\n", millivolts);
			percent = battery_millivolts_to_percent(millivolts);
			printk("VBATT(percent) %d%%\n", percent);
		}

		k_sleep(K_MSEC(2000));
	}
}
