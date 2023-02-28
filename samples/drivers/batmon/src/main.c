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

#define ADC_DEVICE_NODE		DT_INST(0, sbs_sbs_battery)


void main(void)
{
	const struct device *const batt_dev = DEVICE_DT_GET(ADC_DEVICE_NODE);
	double resistance;
	int32_t buffer;
	int err;
	const struct adc_channel_cfg ch_cfg = {
		.channel_id = 0,
		.differential = false,
		.reference = ADC_REF_INTERNAL,
		.gain = ADC_GAIN_1,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT
	};
	const struct adc_sequence seq = {
		.options = NULL,
		.channels = BIT(0),
		.buffer = &buffer,
		.buffer_size = sizeof(buffer),
		.resolution = 12,
		.oversampling = 0,
		.calibrate = 0
	};

	if (!device_is_ready(batt_dev)) {
		printk(" device not ready");
		return;
	}

	err = adc_channel_setup(batt_dev, &ch_cfg);
	if (err) {
		printk("failed to setup ADC channel (err %d)", err);
		return;
	}

	while (true) {
		err = adc_read(batt_dev, &seq);
        printk("%s:%d - err %d buffer %d\n", err, buffer);

		k_sleep(K_MSEC(1000));
	}
}
