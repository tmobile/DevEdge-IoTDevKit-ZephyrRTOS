/*
 * Copyright (c) 2022 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ADC_GECKO_BATTERY_H
#define ADC_GECKO_BATTERY_H

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include "em_adc.h"

struct adc_gecko_config {
	ADC_TypeDef *base;
	ADC_InitSingle_TypeDef initSingle_bv;
	const struct gpio_dt_spec en_gpio;
};

struct adc_gecko_data {
	struct device *dev;
	uint32_t mVolts;
};

#endif
