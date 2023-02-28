
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


#define ADC_RESOLUTION		10
#define ADC_GAIN			ADC_GAIN_1
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME_DEFAULT
#define ADC_1ST_CHANNEL_ID	0


#if !defined(INVALID_ADC_VALUE)
#define INVALID_ADC_VALUE SHRT_MIN
#endif

#define BUFFER_SIZE  6

static ADC_InitSingle_TypeDef initSingle_bv = ADC_INITSINGLE_DEFAULT;
static ADC_InitSingle_TypeDef initSingle_hwid = ADC_INITSINGLE_DEFAULT;

K_SEM_DEFINE(adc_sem, 0, 1);

struct adc_gecko_config {
	const struct gpio_dt_spec en_gpio;
};

struct adc_gecko_data {
	int resolution;
};

int read_hwid(void)
{
#if DT_NODE_HAS_PROP(DT_NODELABEL(adc0), hwid_aport)
	uint32_t sample;
	uint32_t millivolts;
	float millivolts_f;

	// Start ADC conversion
	k_sem_take(&adc_sem, K_MSEC(500));
	ADC_InitSingle(ADC0, &initSingle_hwid);
	ADC_Start(ADC0, adcStartSingle);

	printk("%s:%d - Wait for conversion to be complete \n",__FUNCTION__,__LINE__);
	//  Wait for conversion to be complete
	while(!(ADC0->STATUS & _ADC_STATUS_SINGLEDV_MASK));

	// Get ADC result
	sample = ADC_DataSingleGet(ADC0);

	k_sem_give(&adc_sem);

	// Calculate input voltage in mV
	millivolts_f = (sample * 2500.0) / 4096.0;

	// On the 2nd generation dev edge, voltage on PA2 is
	// one third the actual battery voltage
	millivolts = (uint32_t) (3.0 * millivolts_f + 0.5);
	printk("%s:%d - millivolts %d\n",__FUNCTION__,__LINE__, millivolts);
	return millivolts;
#else
	printk("%s:%d - Hardware Id port undefine \n", __FUNCTION__, __LINE__);
	return 0;
#endif
}

static int adc_gecko_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{
	const struct adc_gecko_config *config = dev->config;
	const struct adc_gecko_data *data = dev->data;
	uint8_t channel_id = channel_cfg->channel_id;

	if (channel_cfg->channel_id >= DT_PROP(DT_NODELABEL(adc0), num_channels)) {
		printk("Channel %d is not valid\n", channel_cfg->channel_id);
		return -EINVAL;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		printk("Invalid channel acquisition time");
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

	initSingle_bv.diff       = channel_cfg->differential;        // single ended
	initSingle_bv.reference  = channel_cfg->reference;    // internal 2.5V reference
	initSingle_bv.resolution = 12;  // 12-bit resolution
	initSingle_bv.acqTime    = channel_cfg->acquisition_time;  // set acquisition time to meet minimum requirement

	memcpy(&initSingle_hwid, &initSingle_bv, sizeof(initSingle_hwid));

	// Select ADC input. See README for corresponding EXP header pin.
	//  initSingle.posSel = adcPosSelAPORT4XCH10;
	#if DT_NODE_HAS_PROP(DT_NODELABEL(adc0), hwid_aport)
	initSingle_hwid.posSel = DT_PROP(DT_NODELABEL(adc0), hwid_aport);
	#endif /* HWID_APORT */
	#if DT_NODE_HAS_PROP(DT_NODELABEL(adc0), vbat_aport)
	initSingle_bv.posSel = DT_PROP(DT_NODELABEL(adc0), vbat_aport);
	#endif /* VBAT_APORT */

	init.timebase = ADC_TimebaseCalc(0);
	ADC_Init(ADC0, &init);


	printk("Channel setup succeeded!");

	return 0;
}

static int adc_gecko_read(const struct device *dev,
			const struct adc_sequence *sequence)
{
#if DT_NODE_HAS_PROP(DT_NODELABEL(adc0), vbat_aport)
	initSingle_bv.posSel = DT_PROP(DT_NODELABEL(adc0), vbat_aport);

	uint32_t sample;
	uint32_t millivolts;
	float millivolts_f;
	// Start ADC conversion
	k_sem_take(&adc_sem, K_MSEC(500));
	ADC_InitSingle(ADC0, &initSingle_bv);
	ADC_Start(ADC0, adcStartSingle);

	//  Wait for conversion to be complete
	while(!(ADC0->STATUS & _ADC_STATUS_SINGLEDV_MASK));

	// Get ADC result
	sample = ADC_DataSingleGet(ADC0);

	k_sem_give(&adc_sem);

	// Calculate input voltage in mV
	millivolts_f = (sample * 2500.0) / 4096.0;

	// On the 2nd generation dev edge, voltage on PA2 is
	// one third the actual battery voltage
	millivolts = (uint32_t) (3.0 * millivolts_f + 0.5);

	return (millivolts);
#else
	return 0;
#endif /* VBAT_APORT */
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

	const struct adc_gecko_config *config = dev->config;
	const struct gpio_dt_spec *en_gpio = &config->en_gpio;

	static const struct adc_channel_cfg adc_ch_config = {
		.gain             = ADC_GAIN_1,
		.reference        = ADC_REF_VDD_1_2,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.channel_id       = ADC_1ST_CHANNEL_ID,
		.differential = false
	};

	// Enable ADC0 clock
	CMU_ClockEnable(cmuClock_ADC0, true);

	// enable VBATT sense
	ret = gpio_pin_configure_dt(en_gpio, GPIO_OUTPUT_HIGH);
	GPIO_PinModeSet(gpioPortK,0, gpioModePushPull, 1);
	adc_gecko_channel_setup(dev, &adc_ch_config);

	read_hwid();
	//printf("HWID = %d\n", data->hwid);

	return 0;
}


/*
	//static struct adc_driver_api gecko_adc_api_##inst = {			\
		.channel_setup = adc_gecko_channel_setup,					\
		.read = adc_gecko_read,										\
		.ref_internal = DT_INST_PROP(inst, vref_mv),				\
		IF_ENABLED(CONFIG_ADC_ASYNC, (.read_async = adc_gecko_read_async,)) \
	};																\
	*/

#define GECKO_ADC_INIT(inst)                                    \
																\
	static struct adc_gecko_config adc_gecko_config##inst = {     \
		.en_gpio = GPIO_DT_SPEC_INST_GET_OR(inst,				\
						       en_gpios, { 0 })                    \
	};                                                             \
	                                                               \
	DEVICE_DT_INST_DEFINE(inst,                                    \
		&adc_gecko_init,                                           \
			      NULL,                                            \
			      NULL,                                            \
			      &adc_gecko_config##inst,                        \
			      POST_KERNEL,                                     \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,              \
			      &adc_gecko_api);

DT_INST_FOREACH_STATUS_OKAY(GECKO_ADC_INIT)
