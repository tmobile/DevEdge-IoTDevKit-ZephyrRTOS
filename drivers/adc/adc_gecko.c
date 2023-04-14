/*
 * Copyright (c) 2023 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT silabs_gecko_adc

#include <string.h>
#include <zephyr/drivers/adc/adc_gecko.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#include <em_cmu.h>
#include <em_gpio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_gecko, CONFIG_ADC_LOG_LEVEL);

#define ADC_GECKO_MAX_RESOLUTION 12
#define ADC_GECKO_CHANNEL_NUM	 32

struct adc_gecko_config {
	ADC_TypeDef *base;
	ADC_InitSingle_TypeDef initSingle_bv;

	/* Number of supported channels */
	uint8_t num_channels;
	/* function pointer to irq setup */
	void (*irq_configure)(void);
};

struct adc_gecko_data {
	const struct device *dev;
	struct adc_context ctx;
	uint16_t *buf;
	/* Pointer to where will be data stored in case of repeated sampling */
	uint16_t *repeat_buf;
	/* Mask with channels that will be sampled */
	uint32_t channels;
	uint8_t resolution;
};

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
		LOG_ERR("ADC resolution value %d is not valid", resolution);
		return -EINVAL;
	}
}

static inline void adc_start_once(const struct device *dev)
{
	struct adc_gecko_config *config = (struct adc_gecko_config *)dev->config;
	ADC_TypeDef *adc_reg = (ADC_TypeDef *)config->base;

	/* Start ADC conversion */
	ADC_InitSingle(adc_reg, &config->initSingle_bv);
	ADC_Start(adc_reg, adcStartSingle);
}

static inline uint16_t adc_get_result(const struct device *dev)
{
	struct adc_gecko_config *config = (struct adc_gecko_config *)dev->config;

	return (uint16_t)ADC_DataSingleGet(config->base);
}

static int adc_get_channel_num(uint8_t id)
{
	switch (id) {
	case 1:
		return adcPosSelAPORT3XCH10;
	case 2:
		return adcPosSelAPORT4XCH23;
	default:
		break;
	}
	return -1;
}

static void adc_select_input(const struct device *dev, uint16_t channel_input)
{
	struct adc_gecko_config *config = (struct adc_gecko_config *)dev->config;
	struct adc_gecko_data *data = dev->data;

	config->initSingle_bv.resolution = get_resolution(data->resolution);
	config->initSingle_bv.posSel = adc_get_channel_num(channel_input + 1);
}

/**
 * Interrupt handler
 */
static void adc_gecko_irq_handler(const struct device *dev)
{
	struct adc_gecko_data *data = dev->data;
	struct adc_gecko_config *config = (struct adc_gecko_config *)dev->config;
	uint16_t result;
	uint8_t ainsel;
	uint32_t flags = ADC_IntGet(ADC0); /* get all interrupt flags */

	if (flags & ADC_IF_SINGLE) {
		ADC_IntClear(config->base, flags);

		/* Fetch result */
		result = adc_get_result(dev);
		ainsel = find_lsb_set(data->channels) - 1;

		/* Copy to buffer and mark this channel as completed to channels bitmap. */
		*data->buf++ = result;
		data->channels &= ~(BIT(ainsel));

		/* Notify result if all data gathered. */
		if (data->channels == 0) {
			adc_context_on_sampling_done(&data->ctx, dev);
			return;
		}

		/* Kick next channel conversion */
		ainsel = (uint8_t)(find_lsb_set(data->channels) - 1);
		adc_select_input(dev, ainsel);
		adc_start_once(dev);
	}
}

static int get_reference_voltage(int reference)
{
	int ref;

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
		ref = -1;
		break;
	}
	return ref;
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

static int adc_gecko_check_buffer_size(const struct device *dev,
				       const struct adc_sequence *sequence)
{
	const struct adc_gecko_config *config = dev->config;
	uint8_t channels = 0;
	size_t needed;
	uint32_t mask;

	for (mask = BIT(config->num_channels - 1); mask != 0; mask >>= 1) {
		if (mask & sequence->channels) {
			channels++;
		}
	}

	needed = channels * sizeof(uint16_t);
	if (sequence->options) {
		needed *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed) {
		return -ENOMEM;
	}

	return 0;
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_gecko_data *data = CONTAINER_OF(ctx, struct adc_gecko_data, ctx);

	data->channels = ctx->sequence.channels;
	data->repeat_buf = data->buf;
	data->resolution = ctx->sequence.resolution;
	/* Find next channel and start conversion */
	adc_select_input(data->dev, find_lsb_set(data->channels) - 1);
	adc_start_once(data->dev);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct adc_gecko_data *data = CONTAINER_OF(ctx, struct adc_gecko_data, ctx);

	if (repeat_sampling) {
		data->buf = data->repeat_buf;
	}
}

static int adc_gecko_channel_setup(const struct device *dev,
				   const struct adc_channel_cfg *channel_cfg)
{
	struct adc_gecko_config *config = (struct adc_gecko_config *)dev->config;

	if (channel_cfg->channel_id >= config->num_channels) {
		LOG_ERR("unsupported channel id '%d'", channel_cfg->channel_id);
		return -ENOTSUP;
	}

	if (channel_cfg->differential) {
		LOG_ERR("unsupported differential mode");
		return -ENOTSUP;
	}

	config->initSingle_bv.diff = channel_cfg->differential;
	config->initSingle_bv.reference = get_reference_voltage(channel_cfg->reference);
	config->initSingle_bv.acqTime = get_acquisition_time(channel_cfg->acquisition_time);

	return 0;
}

static int adc_gecko_start_read(const struct device *dev, const struct adc_sequence *sequence)
{
	const struct adc_gecko_config *config = dev->config;
	struct adc_gecko_data *data = dev->data;
	int err;

	if (sequence->resolution > ADC_GECKO_MAX_RESOLUTION || sequence->resolution == 0) {
		LOG_ERR("unsupported resolution %d", sequence->resolution);
		return -ENOTSUP;
	}

	if (find_msb_set(sequence->channels) > config->num_channels) {
		LOG_ERR("unsupported channels in mask: 0x%08x", sequence->channels);
		return -ENOTSUP;
	}

	err = adc_gecko_check_buffer_size(dev, sequence);
	if (err) {
		LOG_ERR("buffer size too small");
		return err;
	}

	data->buf = sequence->buffer;
	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

static int adc_gecko_read_async(const struct device *dev, const struct adc_sequence *sequence,
				struct k_poll_signal *async)
{
	struct adc_gecko_data *data = dev->data;
	int err;

	adc_context_lock(&data->ctx, async ? true : false, async);
	err = adc_gecko_start_read(dev, sequence);
	adc_context_release(&data->ctx, err);

	return err;
}

static int adc_gecko_read(const struct device *dev, const struct adc_sequence *sequence)
{
	return adc_gecko_read_async(dev, sequence, NULL);
}

#ifdef CONFIG_ADC_ASYNC
static int adc_gecko_read_async(const struct device *dev, const struct adc_sequence *sequence,
				struct k_poll_signal *async)
{
	return -ENOTSUP;
}
#endif

static int adc_gecko_init(const struct device *dev)
{
	struct adc_gecko_config *config = (struct adc_gecko_config *)dev->config;
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	struct adc_gecko_data *data = dev->data;

	data->dev = dev;

	config->irq_configure();

	adc_context_init(&data->ctx);
	config->base = (ADC_TypeDef *)DT_REG_ADDR(DT_NODELABEL(adc));

	/* Enable ADC0 clock */
	CMU_ClockEnable(cmuClock_ADC0, true);

	/* Modify init structs and initialize */
	init.prescale = ADC_PrescaleCalc(DT_PROP(DT_NODELABEL(adc), frequency), 0);
	/* Init to max ADC clock for Series 1 */
	init.timebase = ADC_TimebaseCalc(0);
	ADC_Init(config->base, &init);

	/* Enable interrupt for single conversion */
	ADC_IntEnable(ADC0, ADC_IF_SINGLE);

	adc_context_unlock_unconditionally(&data->ctx);
	return 0;
}

#define IRQ_CONFIGURE_FUNC(idx)                                                                    \
	static void adc_gecko_configure_func_##idx(void)                                           \
	{                                                                                          \
		IRQ_CONNECT(ADC0_IRQn, 0, adc_gecko_irq_handler, DEVICE_DT_INST_GET(idx), 0);      \
		irq_enable(ADC0_IRQn);                                                             \
	}

#define IRQ_CONFIGURE_DEFINE(idx) .irq_configure = adc_gecko_configure_func_##idx

#define GECKO_ADC_INIT(inst)                                                                       \
	IRQ_CONFIGURE_FUNC(inst)                                                                   \
	static const struct adc_driver_api adc_gecko_api = {                                       \
		.channel_setup = adc_gecko_channel_setup,                                          \
		.read = adc_gecko_read,                                                            \
		.ref_internal = DT_INST_PROP(inst, vref_mv),                                       \
		IF_ENABLED(CONFIG_ADC_ASYNC, (.read_async = adc_gecko_read_async,))};             \
                                                                                                   \
	static struct adc_gecko_config adc_gecko_config##inst = {                                  \
		.num_channels = ADC_GECKO_CHANNEL_NUM,                                             \
		IRQ_CONFIGURE_DEFINE(inst),                                                        \
	};                                                                                         \
	static struct adc_gecko_data adc_gecko_data##inst;                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &adc_gecko_init, NULL, &adc_gecko_data##inst,                  \
			      &adc_gecko_config##inst, POST_KERNEL,                                \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &adc_gecko_api);
DT_INST_FOREACH_STATUS_OKAY(GECKO_ADC_INIT)
