/*
 * Copyright (c) 2022 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT silabs_adc_gecko

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

K_SEM_DEFINE(adc_sem, 0, 1);

#define ADC_GECKO_MAX_RESOLUTION 12
#define ADC_GECKO_CHANNEL_NUM 32

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

static inline void adc_select_input(uint8_t input) {
    //valid_params_if(ADC, input < NUM_ADC_CHANNELS);
   // hw_write_masked(&adc_hw->cs, input << ADC_CS_AINSEL_LSB, ADC_CS_AINSEL_BITS);
}

/*! \brief  Get the currently selected ADC input channel
 *  \ingroup hardware_adc
 *
 * \return The currently selected input channel. 0...3 are GPIOs 26...29 respectively. Input 4 is the onboard temperature sensor.
 */
static inline uint8_t adc_get_selected_input(void) {
    return 0;//(adc_hw->cs & ADC_CS_AINSEL_BITS) >> ADC_CS_AINSEL_LSB;
}

static inline void adc_start_once(void)
{
	//hw_set_bits(&adc_hw->cs, ADC_CS_START_ONCE_BITS);
}

static inline uint16_t adc_get_result(void)
{
	return (uint16_t)0xff;//adc_hw->result;
}

static inline bool adc_get_err(void)
{
	return false;//(adc_hw->cs & ADC_CS_ERR_BITS) ? true : false;
}

static inline void adc_clear_errors(void)
{
	/* write 1 to clear */
	//hw_set_bits(&adc_hw->fcs, ADC_FCS_OVER_BITS);
	//hw_set_bits(&adc_hw->fcs, ADC_FCS_UNDER_BITS);
	//hw_set_bits(&adc_hw->fcs, ADC_FCS_ERR_BITS);
	//hw_set_bits(&adc_hw->cs, ADC_CS_ERR_STICKY_BITS);
}

static inline void adc_enable(void)
{
	//adc_hw->cs = ADC_CS_EN_BITS;
	//while (!(adc_hw->cs & ADC_CS_READY_BITS))
	//	;
}


static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_gecko_data *data = CONTAINER_OF(ctx, struct adc_gecko_data,
						 ctx);

	data->channels = ctx->sequence.channels;
	data->repeat_buf = data->buf;

	adc_clear_errors();

	/* Find next channel and start conversion */
	adc_select_input(find_lsb_set(data->channels) - 1);
	adc_start_once();
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat_sampling)
{
	struct adc_gecko_data *data = CONTAINER_OF(ctx, struct adc_gecko_data,
						 ctx);

	if (repeat_sampling) {
		data->buf = data->repeat_buf;
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

/**
 * @brief Check if buffer in @p sequence is big enough to hold all ADC samples
 *
 * @param dev RaspberryPi Pico ADC device
 * @param sequence ADC sequence description
 *
 * @return 0 on success
 * @return -ENOMEM if buffer is not big enough
 */
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

static int adc_gecko_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{
	const struct adc_gecko_config *config = dev->config;
	printk("%d %d\n", channel_cfg->channel_id,config->num_channels);
	if (channel_cfg->channel_id >= config->num_channels) {
		LOG_ERR("unsupported channel id '%d'", channel_cfg->channel_id);
		return -ENOTSUP;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("Acquisition time is not valid");
		return -EINVAL;
	}

	if (channel_cfg->differential) {
		LOG_ERR("unsupported differential mode");
		return -ENOTSUP;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Gain is not valid");
		return -EINVAL;
	}

	return 0;
}

#if 0
static int adc_gecko_channel_setup(const struct device *dev,
				   const struct adc_channel_cfg *channel_cfg)
{
	struct adc_gecko_config *config = (struct adc_gecko_config *)dev->config;

	//if (channel_cfg->channel_id >= DT_PROP(DT_NODELABEL(adc0), num_channels)) {
	//	LOG_ERR("Channel %d is not valid", channel_cfg->channel_id);
	//	return -EINVAL;
	//}
	if (channel_cfg->differential) {
		LOG_ERR("Differential channels are not supported");
		return -EINVAL;
	}

	config->initSingle_bv.diff = channel_cfg->differential;
	config->initSingle_bv.reference = get_reference_voltage(channel_cfg->reference);
	config->initSingle_bv.acqTime = get_acquisition_time(channel_cfg->acquisition_time);
	config->initSingle_bv.resolution = adcRes12Bit;

	return 0;
}
#endif

static int adc_gecko_read(const struct device *dev,
			      const struct adc_sequence *sequence)
{
	const struct adc_gecko_config *config = dev->config;
	struct adc_gecko_data *data = dev->data;
	int err;

	if (sequence->resolution > ADC_GECKO_MAX_RESOLUTION ||
	    sequence->resolution == 0) {
		LOG_ERR("unsupported resolution %d", sequence->resolution);
		return -ENOTSUP;
	}

	if (find_msb_set(sequence->channels) > config->num_channels) {
		LOG_ERR("unsupported channels in mask: 0x%08x",
			sequence->channels);
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

#if 0

static int adc_gecko_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_gecko_config *config = (struct adc_gecko_config *)dev->config;
	struct adc_gecko_data *data = dev->data;
	ADC_TypeDef *adc_reg = (ADC_TypeDef *)config->base;

	config->initSingle_bv.posSel = -1;

	if (!sequence->resolution) {
		LOG_ERR("Invalid resolution");
		return -EINVAL;
	}

	switch (sequence->channels) {
	case 0:
#if DT_NODE_HAS_PROP(DT_NODELABEL(adc0), vbatt_aport)
		config->initSingle_bv.posSel = DT_NODE_HAS_PROP(DT_NODELABEL(adc0), vbatt_aport);
#else
		LOG_ERR("%s:%d - VBAT_APORT Undefined", __func__, __LINE__);
#endif
		break;
	case 1:
#if DT_NODE_HAS_PROP(DT_NODELABEL(adc0), hwid_aport)
		config->initSingle_bv.posSel = DT_NODE_HAS_PROP(DT_NODELABEL(adc0), hwid_aport);
#else
		LOG_ERR("%s:%d - HWID_APORT Undefined", __func__, __LINE__);
#endif
		break;
	default:
		LOG_ERR("%s:%d - Unsupported channel %d", __func__, __LINE__,
		       sequence->channels);
		break;
	}
	if (config->initSingle_bv.posSel < 0)
		return -EINVAL;

	config->initSingle_bv.resolution = get_resolution(sequence->resolution);

	/* Start ADC conversion */
	k_sem_take(&adc_sem, K_MSEC(500));
	ADC_InitSingle(adc_reg, &config->initSingle_bv);
	ADC_Start(adc_reg, adcStartSingle);

	/*  Wait for conversion to be complete */
	while (!(adc_reg->STATUS & _ADC_STATUS_SINGLEDV_MASK))
		;
	/* Get ADC result */
	data->mVolts = ADC_DataSingleGet(adc_reg);
	k_sem_give(&adc_sem);
	return 0;
}

#endif

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
	const struct gpio_dt_spec *en_gpio = &config->en_gpio;
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;

	config->base = (ADC_TypeDef *)DT_REG_ADDR(DT_NODELABEL(adc));

	/* enable VBATT sense */
	gpio_pin_configure_dt(en_gpio, GPIO_OUTPUT_HIGH);
	/* Enable ADC0 clock */
	CMU_ClockEnable(cmuClock_ADC0, true);

	/* Modify init structs and initialize */
	init.prescale = ADC_PrescaleCalc(DT_PROP(DT_NODELABEL(adc), frequency),
		 0); /* Init to max ADC clock for Series 1 */
	init.timebase = ADC_TimebaseCalc(0);
	ADC_Init(config->base, &init);

	return 0;
}

#define GECKO_ADC_INIT(inst)                                                                       \
	static const struct adc_driver_api adc_gecko_api = {	\
		.channel_setup = adc_gecko_channel_setup,	\
		.read = adc_gecko_read,	\
		.ref_internal = DT_INST_PROP(inst, vref_mv),	\
		IF_ENABLED(CONFIG_ADC_ASYNC, (.read_async = adc_gecko_read_async,)) \
	}; \
                                                                                                   \
	static struct adc_gecko_config adc_gecko_config##inst = {                                  \
		.en_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, en_gpios, {0}),                         \
		.num_channels = ADC_GECKO_CHANNEL_NUM,};			   \
	static struct adc_gecko_data adc_gecko_data##inst;                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &adc_gecko_init, NULL, &adc_gecko_data##inst,                  \
			      &adc_gecko_config##inst, POST_KERNEL,                                \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &adc_gecko_api);

DT_INST_FOREACH_STATUS_OKAY(GECKO_ADC_INIT)
