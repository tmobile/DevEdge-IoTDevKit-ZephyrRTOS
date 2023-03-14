/*
 * Copyright (c) 2022 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT silabs_adc_gecko

#include <string.h>
#include <zephyr/drivers/adc/gecko_adc/gecko_adc.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>

#include "em_cmu.h"
#include "em_gpio.h"

K_SEM_DEFINE(adc_sem, 0, 1);

static int get_reference_voltage(int reference)
{
	int ref = -1;

	switch (reference) {
	case ADC_REF_INTERNAL:
		ref = adcRef2V5;
		break;
	case ADC_REF_VDD_1:
		ref = adcRefVDD;
		break;
	case ADC_REF_VDD_1_2:
		ref = adcRef2V5;
		break;
	case ADC_REF_VDD_1_4:
		ref = adcRef1V25;
		break;
	default:
		break;
	}
	return ref;
}

static int get_resolution(uint8_t resolution)
{

	switch (resolution) {
	case 12:
		return adcRes12Bit;
	case 8:
		return adcRes8Bit;
	case 6:
		return adcRes6Bit;
	case 0:
		return adcResOVS;
	default:
		printk("ADC resolution value %d is not valid", resolution);
		return -EINVAL;
	}
	return 0;
}

static int get_acquisition_time(uint16_t acq)
{
	uint16_t sample_cycl;

	/* Check acquisition time */
	switch (acq) {
	case ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 1):
		sample_cycl = adcAcqTime1;
		break;
	case ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 2):
		sample_cycl = adcAcqTime2;
		break;
	case ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 4):
		sample_cycl = adcAcqTime4;
		break;
	case ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 8):
		sample_cycl = adcAcqTime8;
		break;
	case ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 16):
		sample_cycl = adcAcqTime16;
		break;
	case ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 32):
		sample_cycl = adcAcqTime32;
		break;
	case ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 64):
		sample_cycl = adcAcqTime64;
		break;
	case ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 128):
		sample_cycl = adcAcqTime128;
		break;
	case ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 256):
		sample_cycl = adcAcqTime256;
		break;
	default:
		sample_cycl = 0;
		break;
	}
	return sample_cycl;
}

static int adc_gecko_channel_setup(const struct device *dev,
				   const struct adc_channel_cfg *channel_cfg)
{
	struct adc_gecko_config *config = (struct adc_gecko_config *)dev->config;

	if (channel_cfg->channel_id >= DT_PROP(DT_NODELABEL(adc0), num_channels)) {
		printk("Channel %d is not valid\n", channel_cfg->channel_id);
		return -EINVAL;
	}
	if (channel_cfg->differential) {
		printk("Differential channels are not supported\n");
		return -EINVAL;
	}

	config->initSingle_bv.diff = channel_cfg->differential;
	config->initSingle_bv.reference = get_reference_voltage(channel_cfg->reference);
	config->initSingle_bv.acqTime = get_acquisition_time(channel_cfg->acquisition_time);
	config->initSingle_bv.resolution = adcRes12Bit;

	return 0;
}

static int adc_gecko_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_gecko_config *config = (struct adc_gecko_config *)dev->config;
	struct adc_gecko_data *data = dev->data;
	ADC_TypeDef *adc_reg = (ADC_TypeDef *)config->base;

	config->initSingle_bv.posSel = -1;

	if (!sequence->resolution) {
		printk("Invalid resolution\n");
		return -EINVAL;
	}

	switch (sequence->channels) {
	case 0:
#if DT_NODE_HAS_PROP(DT_NODELABEL(adc0), adc_channel_10)
		config->initSingle_bv.posSel = DT_PROP(DT_NODELABEL(adc0), adc_channel_10);
#else
		printk("%s:%d - VBatt Port Undefined\n", __FUNCTION__, __LINE__);
#endif
		break;
	case 1:
#if DT_NODE_HAS_PROP(DT_NODELABEL(adc0), adc_channel_23)
		config->initSingle_bv.posSel = DT_PROP(DT_NODELABEL(adc0), adc_channel_23);
#else
		printk("%s:%d - HWID Port Undefined\n", __FUNCTION__, __LINE__);
#endif
		break;
	default:
		printk("%s:%d - Unsupported channel %d\n", __FUNCTION__, __LINE__,
		       sequence->channels);
		break;
	}
	if (config->initSingle_bv.posSel < 0)
		return -EINVAL;

	config->initSingle_bv.resolution = get_resolution(sequence->resolution);

	// Start ADC conversion
	k_sem_take(&adc_sem, K_MSEC(500));
	ADC_InitSingle(adc_reg, &config->initSingle_bv);
	ADC_Start(adc_reg, adcStartSingle);

	//  Wait for conversion to be complete
	while (!(adc_reg->STATUS & _ADC_STATUS_SINGLEDV_MASK))
		;
	// Get ADC result
	// TODO need to move this to the battery handler.
	// we only want raw values from this.
	data->mVolts = ADC_DataSingleGet(adc_reg);
	k_sem_give(&adc_sem);
	return 0;
}

#ifdef CONFIG_ADC_ASYNC
static int adc_gecko_read_async(const struct device *dev, const struct adc_sequence *sequence,
				struct k_poll_signal *async)
{
	return -ENOTSUP;
}
#endif

static const struct adc_driver_api adc_gecko_api = {
	.channel_setup = adc_gecko_channel_setup,
	.read = adc_gecko_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = adc_gecko_read_async,
#endif
};

static int adc_gecko_init(const struct device *dev)
{
	struct adc_gecko_config *config = (struct adc_gecko_config *)dev->config;
	const struct gpio_dt_spec *en_gpio = &config->en_gpio;
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;

	config->base = (ADC_TypeDef *)DT_REG_ADDR(DT_NODELABEL(adc0));

	// enable VBATT sense
	gpio_pin_configure_dt(en_gpio, GPIO_OUTPUT_HIGH);
	// Enable ADC0 clock
	CMU_ClockEnable(cmuClock_ADC0, true);

	// Modify init structs and initialize
	init.prescale = ADC_PrescaleCalc(DT_PROP(DT_NODELABEL(adc0), frequency),
					 0); // Init to max ADC clock for Series 1
	init.timebase = ADC_TimebaseCalc(0);
	ADC_Init(config->base, &init);

	return 0;
}

#define GECKO_ADC_INIT(inst)                                                                       \
                                                                                                   \
	static struct adc_gecko_config adc_gecko_config##inst = {                                  \
		.en_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, en_gpios, {0})};                         \
	static struct adc_gecko_data adc_gecko_data##inst;                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &adc_gecko_init, NULL, &adc_gecko_data##inst,                  \
			      &adc_gecko_config##inst, POST_KERNEL,                                \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &adc_gecko_api);

DT_INST_FOREACH_STATUS_OKAY(GECKO_ADC_INIT)
