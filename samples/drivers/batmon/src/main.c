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
printf("[main] Battery monitoring started\n");
	const struct device *const adc_dev = DEVICE_DT_GET(ADC_DEVICE_NODE);

    if (!adc_dev) {
        printf("[main]  driver error\n");
        return;
		}
    printk("[main] adc_dev address %p\n", (void *)adc_dev);

	while(1) {
	
	}

}