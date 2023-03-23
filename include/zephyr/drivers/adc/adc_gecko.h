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

#if 0
#define ADC_GECKO_MAX_RESOLUTION 12

struct adc_gecko_config {
	ADC_TypeDef *base;
	ADC_InitSingle_TypeDef initSingle_bv;
	const struct gpio_dt_spec en_gpio;

	/** Number of supported channels */
	uint8_t num_channels;
};

struct adc_gecko_data {
	struct device *dev;
	struct adc_context ctx;
	uint16_t *buf;
	/** Pointer to where will be data stored in case of repeated sampling */
	uint16_t *repeat_buf;
	/** Mask with channels that will be sampled */
	uint32_t channels;
	uint32_t mVolts;
};
#endif

#endif
