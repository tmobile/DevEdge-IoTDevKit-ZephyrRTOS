/*
 * Copyright (c) 2020 Libre Solar Technologies GmbH
 * Copyright (c) 2023 T-Mobile USA, Inc.
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
#include <zephyr/drivers/adc/adc_gecko/adc_gecko.h>

#define ADC_DEVICE_NODE		DT_INST(0, silabs_adc_gecko)

void main(void)
{
	const struct device *const adc_dev = DEVICE_DT_GET(ADC_DEVICE_NODE);
	int32_t buffer;
	int err;
	uint32_t millivolts;

	struct adc_gecko_data *data = adc_dev->data;

	static const struct adc_channel_cfg ch_cfg = {
		.reference        = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 32),
		.channel_id       = 2,
		.differential = false
	};

	printk("%s:%d - GECKO ADC SAMPLE Starting...\n", __func__, __LINE__);
	if (!device_is_ready(adc_dev)) {
		printk(" device not ready");
		return;
	}

	err = adc_channel_setup(adc_dev, &ch_cfg);
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
		seq.channels = 1;
		err = adc_read(adc_dev, &seq);
		if (err >= 0) {
			millivolts = 3.0 * data->mVolts * 2500.0 / 4096.0 + 0.5;
			printk("ADC channel 1 Value = %d\n", millivolts);
		}

		seq.channels = 0;
		err = adc_read(adc_dev, &seq);
		if (err >= 0) {
			millivolts = 3.0 * data->mVolts * 2500.0 / 4096.0 + 0.5;
			printk("ADC channel 0 Value = %d\n", millivolts);
		}

		k_sleep(K_MSEC(2000));
	}
}
