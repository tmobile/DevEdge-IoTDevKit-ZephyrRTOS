

#define DT_DRV_COMPAT sbs_sbs_battery

#include <zephyr/drivers/adc.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
 #include <zephyr/drivers/gpio.h>

#define ADC_RESOLUTION		10
#define ADC_GAIN			ADC_GAIN_1
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME_DEFAULT
#define ADC_1ST_CHANNEL_ID	10


#if !defined(INVALID_ADC_VALUE)
#define INVALID_ADC_VALUE SHRT_MIN
#endif

#define BUFFER_SIZE  6
static int16_t m_sample_buffer[BUFFER_SIZE];

static const struct adc_channel_cfg m_1st_channel_cfg = {
	.gain             = ADC_GAIN,
	.reference        = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id       = ADC_1ST_CHANNEL_ID,
};

struct gecko_adc_config {
	const struct gpio_dt_spec en_gpio;
};


static int gecko_adc_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{
	uint8_t channel_id = channel_cfg->channel_id;

	if (channel_id > 1) {
		printk("Channel %d is not valid", channel_id);
		return -EINVAL;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		printk("Invalid channel acquisition time");
		return -EINVAL;
	}

	if (channel_cfg->differential) {
		printk("Differential channels are not supported");
		return -EINVAL;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		printk("Invalid channel gain");
		return -EINVAL;
	}

	if (channel_cfg->reference != ADC_REF_INTERNAL) {
		printk("Invalid channel reference");
		return -EINVAL;
	}
	return 0;
}

static int gecko_adc_read(const struct device *dev,
			const struct adc_sequence *sequence)
{

	return 0;
}

#ifdef CONFIG_ADC_ASYNC
static int gecko_adc_read_async(const struct device *dev,
			      const struct adc_sequence *sequence,
			      struct k_poll_signal *async)
{
	return -ENOTSUP;
}
#endif

static const struct adc_driver_api gecko_adc_api = {
	.channel_setup = gecko_adc_channel_setup,
	.read = gecko_adc_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = gecko_adc_read_async,
#endif
};

static int gecko_adc_init(const struct device *dev)
{
	printk("%s:%d - init batmon driver\n", __FUNCTION__, __LINE__);
	printk("[BAT] adc_dev address %p\n", (void *)dev);
	int i, ret;

	const struct gecko_adc_config *config = dev->config;
	const struct gpio_dt_spec *en_gpio = &config->en_gpio;

	ret = gpio_pin_configure_dt(en_gpio, GPIO_OUTPUT_HIGH);



	return 0;
}


#define GECKO_ADC_INIT(inst)							\
	static struct gecko_adc_config gecko_adc_config_##inst = {			\
		.en_gpio = GPIO_DT_SPEC_INST_GET_OR(inst,			\
						       en_gpios, { 0 })	\
	};									\
										\
	DEVICE_DT_INST_DEFINE(inst,						\
		&gecko_adc_init,								\
			      NULL,			\
			      NULL,				\
			      &gecko_adc_config_##inst,				\
			      POST_KERNEL,					\
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,			\
			      &gecko_adc_api);

DT_INST_FOREACH_STATUS_OKAY(GECKO_ADC_INIT)
