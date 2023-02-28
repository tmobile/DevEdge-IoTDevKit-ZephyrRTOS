
#define DT_DRV_COMPAT sbs_sbs_battery

#include <string.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#include "em_adc.h"
#include "em_cmu.h"
#include "em_gpio.h"

K_SEM_DEFINE(adc_sem, 0, 1);

struct adc_gecko_config {
	ADC_TypeDef *base;
	ADC_InitSingle_TypeDef initSingle_bv;
	const struct gpio_dt_spec en_gpio;
};

struct adc_gecko_data {
	struct device *dev;
	int resolution;
	int hwid;
};

static int adc_gecko_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{
	struct adc_gecko_config *config = dev->config;
	const struct adc_gecko_data *data = dev->data;
	uint8_t channel_id = channel_cfg->channel_id;
	ADC_TypeDef *adc_reg = (ADC_TypeDef *)config->base;

	if (channel_cfg->channel_id >= DT_PROP(DT_NODELABEL(adc0), num_channels)) {
		printk("Channel %d is not valid\n", channel_cfg->channel_id);
		return -EINVAL;
	}
	if (channel_cfg->differential) {
		printk("Differential channels are not supported\n");
		return -EINVAL;
	}

	if (channel_cfg->gain > ADC_GAIN_128) {
		printk("Invalid channel gain\n");
		return -EINVAL;
	}

	// Declare init structs
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;

	// Modify init structs and initialize
	init.prescale = ADC_PrescaleCalc(DT_PROP(DT_NODELABEL(adc0), frequency), 0); // Init to max ADC clock for Series 1

	config->initSingle_bv.diff       = channel_cfg->differential;
	config->initSingle_bv.reference  = channel_cfg->reference;
	config->initSingle_bv.acqTime    = channel_cfg->acquisition_time;
	config->initSingle_bv.resolution = data->resolution;

	init.timebase = ADC_TimebaseCalc(0);
	ADC_Init(adc_reg, &init);
	printk("Channel setup succeeded!");

	return 0;
}

static int adc_gecko_read(const struct device *dev,
			const struct adc_sequence *sequence)
{
	struct adc_gecko_config *config = dev->config;
	ADC_TypeDef *adc_reg = (ADC_TypeDef *)config->base;
	config->initSingle_bv.posSel = -1;


	if (!sequence->resolution) {
		printk("Invalid resolution\n");
		return 0;
	}

	printk("%s:%d - sequence->channels %d \n", __FUNCTION__,__LINE__, sequence->channels);
	switch (sequence->channels) {
		case 0:
			#if DT_NODE_HAS_PROP(DT_NODELABEL(adc0), vbat_aport)
				config->initSingle_bv.posSel = DT_PROP(DT_NODELABEL(adc0), vbat_aport);
			#else
				printk("%s:%d - VBatt Port Undefined\n", __FUNCTION__,__LINE__);
			#endif
			break;
		case 1:
			#if DT_NODE_HAS_PROP(DT_NODELABEL(adc0), hwid_aport)
				config->initSingle_bv.posSel = DT_PROP(DT_NODELABEL(adc0), hwid_aport);
			#else
				printk("%s:%d - HWID Port Undefined\n", __FUNCTION__,__LINE__);
			#endif
			break;
		default:
			printk("%s:%d - Unsupported channel %d\n", __FUNCTION__,__LINE__, sequence->channels);
			break;
	}

	printk("%s:%d - initSingle_bv.posSel %d \n", __FUNCTION__,__LINE__,config->initSingle_bv.posSel);
	if (config->initSingle_bv.posSel < 0)
		return 0;

	config->initSingle_bv.resolution = sequence->resolution;

	uint32_t sample;
	uint32_t millivolts;
	float millivolts_f;
	// Start ADC conversion
	k_sem_take(&adc_sem, K_MSEC(500));
	ADC_InitSingle(adc_reg, &config->initSingle_bv);
	ADC_Start(adc_reg, adcStartSingle);

	//  Wait for conversion to be complete
	while(!(adc_reg->STATUS & _ADC_STATUS_SINGLEDV_MASK));

	// Get ADC result
	sample = ADC_DataSingleGet(adc_reg);

	k_sem_give(&adc_sem);

	// Calculate input voltage in mV
	millivolts_f = (sample * 2500.0) / 4096.0;

	// On the 2nd generation dev edge, voltage on PA2 is
	// one third the actual battery voltage
	millivolts = (uint32_t) (3.0 * millivolts_f + 0.5);
	printf("millivolts %d\n", millivolts);
	return (millivolts);
}

#ifdef CONFIG_ADC_ASYNC
static int adc_gecko_read_async(const struct device *dev,
			      const struct adc_sequence *sequence,
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
	int ret;
	int mVolts;
	int32_t buffer;

	struct adc_gecko_config *config = dev->config;
	struct adc_gecko_data *data = dev->data;
	const struct gpio_dt_spec *en_gpio = &config->en_gpio;

	config->base = (ADC_TypeDef *) DT_REG_ADDR(DT_NODELABEL(adc0));

	// Enable ADC0 clock
	CMU_ClockEnable(cmuClock_ADC0, true);

	// enable VBATT sense
	gpio_pin_configure_dt(en_gpio, GPIO_OUTPUT_HIGH);
	return 0;
}

#define GECKO_ADC_INIT(inst)                                       \
																   \
	static struct adc_gecko_config adc_gecko_config##inst = {      \
		.en_gpio = GPIO_DT_SPEC_INST_GET_OR(inst,				   \
						       en_gpios, { 0 })                    \
	};                                                             \
	static struct adc_gecko_data adc_gecko_data##inst;             \
	                                                               \
	DEVICE_DT_INST_DEFINE(inst,                                    \
		&adc_gecko_init,                                           \
			      NULL,                                            \
			      &adc_gecko_data##inst,                           \
			      &adc_gecko_config##inst,                         \
			      POST_KERNEL,                                     \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,              \
			      &adc_gecko_api);

DT_INST_FOREACH_STATUS_OKAY(GECKO_ADC_INIT)
