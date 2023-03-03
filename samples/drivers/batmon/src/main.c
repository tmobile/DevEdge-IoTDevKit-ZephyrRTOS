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
#include <zephyr/drivers/fuel_gauge/sbs_battery/sbs_battery.h>

#define ADC_DEVICE_NODE		DT_INST(0, sbs_sbs_battery)


void main(void)
{
	const struct device *const batt_dev = DEVICE_DT_GET(ADC_DEVICE_NODE);
	int32_t buffer;
	int err;

	struct adc_gecko_data *data = batt_dev->data;


	static const struct adc_channel_cfg ch_cfg = {
		.reference        = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 32),
		.channel_id       = 2,
		.differential = false
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
		printk("HWID = %d\n", data->hwid);
		k_sleep(K_MSEC(2000));

		seq.channels = 0;
		err = adc_read(batt_dev, &seq);
        printk("Voltage = %dmV\n", data->mVolts);
		printk("Percent = %d%%\n", data->percent);

		k_sleep(K_MSEC(2000));
	}
}
