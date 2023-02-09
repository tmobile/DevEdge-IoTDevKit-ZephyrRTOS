#define DT_DRV_COMPAT silabs_battery_adc

#include <stdio.h>

/* Zephyr Logging headers */
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(batter_log, CONFIG_ADC_LOG_LEVEL);

#include <zephyr/kernel.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_adc.h"
#include "em_gpio.h"
#include <zephyr/drivers/adc_battery.h>
#include "battery_ctrl.h"
#include "board.h"

/* battery defines */
#define SIGN_BIT_POSITION          (13)
#define AREG_ADC_DATA_STATUS       (0xf6)
#define ADC_DATA_READY             BIT(0)


/** Uint : Percentage **/
#define FULL_CHARGED_LV       90
#define HIGH_CHARGED_LV       80
#define LOW_CHARGED_LV        30
#define MIN_CHARGED_LV        10

#define HIGH_BATTERY_LV       60
#define LOW_BATTERY_LV        30

#define PROTECTED_CUT_LV      6

#define CHARGER_STAT_READY    0
#define CHARGER_STAT_INPROG   1
#define CHARGER_STAT_FULL     2
#define CHARGER_STAT_FAULT    3

#define CUR_STAT_UNKNOWN      0
#define CUR_STAT_BATTERY      10
#define CUR_STAT_INPROG       20
#define CUR_STAT_FULL         30
#define CUR_STAT_FAULT        40


#define adcFreq   16000000
K_SEM_DEFINE(adc_sem, 0, 1);


static ADC_InitSingle_TypeDef initSingle_bv = ADC_INITSINGLE_DEFAULT;
static ADC_InitSingle_TypeDef initSingle_hwid = ADC_INITSINGLE_DEFAULT;


/**
 * @brief Set the VBAT_SNS_EN Pin High to enable ADC readings
 *
 */
static void set_vbat_sens_en(void)
{
	//   pin = 0
	//   mode = gpioModeEnabled;
	//   out is 1 otherwise it will be input
	//   Set PK0/PinE2 as output so it can be
#ifdef VBAT_EN_PORT
	GPIO_PinModeSet(VBAT_EN_PORT, VBAT_EN_PIN, gpioModePushPull, 1);
#endif /* VBAT_EN_PORT */
}


/***************************************************************************
 * @brief  Initialize ADC function
 ***************************************************************************/
static int battery_init (const struct device *dev)
{
	struct batmon_data *data = dev->data;
	set_vbat_sens_en();

	// Enable ADC0 clock
	CMU_ClockEnable(cmuClock_ADC0, true);

	// Declare init structs
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;

	// Modify init structs and initialize
	init.prescale = ADC_PrescaleCalc(adcFreq, 0); // Init to max ADC clock for Series 1

	initSingle_bv.diff       = false;        // single ended
	initSingle_bv.reference  = adcRef2V5;    // internal 2.5V reference
	initSingle_bv.resolution = adcRes12Bit;  // 12-bit resolution
	initSingle_bv.acqTime    = adcAcqTime32;  // set acquisition time to meet minimum requirement

	memcpy(&initSingle_hwid, &initSingle_bv, sizeof(initSingle_hwid));

	// Select ADC input. See README for corresponding EXP header pin.
	//  initSingle.posSel = adcPosSelAPORT4XCH10;
#ifdef HWID_APORT
	initSingle_hwid.posSel = HWID_APORT;
#endif /* HWID_APORT */
#ifdef VBAT_APORT
	initSingle_bv.posSel = VBAT_APORT;
#endif /* VBAT_APORT */

	init.timebase = ADC_TimebaseCalc(0);

	ADC_Init(ADC0, &init);

	data->hwid = read_hwid();

	printf("HWID = %d\n", data->hwid);
}

/***************************************************************************
 * @brief  Exponential filter for battery level
 ***************************************************************************/
static void battery_apply_filter(float *bv)
{
	static float s_filtered_capacity = -1;
	static bool s_battery_is_charging = false;
	bool battery_is_charging;

	// If there has been a switch between charger and battery, reset the filter
	battery_is_charging = is_battery_charging();
	if (s_battery_is_charging != battery_is_charging) {
		s_battery_is_charging = battery_is_charging;
		s_filtered_capacity = -1;
	}

	if (s_filtered_capacity < 0) {
		s_filtered_capacity = *bv;
	}
	*bv = s_filtered_capacity = s_filtered_capacity * 0.95 + (*bv) * 0.05;
}

/***************************************************************************
 * @brief  This function writes the amount of battery charge remaining
 *         (to the nearest 1%) in bv.
 *         It returns true if successful, or false if there is an issue
 ***************************************************************************/
static bool battery_millivolts_to_percent(uint32_t millivolts, uint8_t *percent) {
	float curBv = get_remaining_capacity((float) millivolts / 1000);
	battery_apply_filter(&curBv);
	*percent = (uint8_t) (curBv + 0.5);
	return true;
}

/* API implementation: channel_setup */
static int battery_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{

	return 0;
}

/**
 * @brief Read HWID divider voltage
 *
 * @return int Millivolts
 */
int read_hwid(void)
{
#ifdef HWID_APORT
	uint32_t sample;
	uint32_t millivolts;
	float millivolts_f;
	// Start ADC conversion
	k_sem_take(&adc_sem, K_MSEC(500));
	ADC_InitSingle(ADC0, &initSingle_hwid);
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
#endif /* HWID_APORT */
}


static int read_battery_voltage(void)
{
#ifdef VBAT_APORT
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
/* API implementation: read */
static int battery_voltage_read(const struct device *dev,
			const struct adc_sequence *sequence)
{
	struct batmon_data *data = dev->data;

	data->battery_attached = 0;
	data->charging = 0;
    data->vbus = 0;
	data->fault = 0;

	data->charging_status = get_battery_charging_status(&data->charging, &data->vbus, &data->battery_attached, &data->fault);

	if (data->battery_attached !=  0) {
		data->mVolts = read_battery_voltage();
	}

	battery_millivolts_to_percent(data->mVolts, &data->percent);
	return 0;
}

#ifdef CONFIG_BATTERY_ASYNC
/* API implementation: read async*/
static int battery_read_async(const struct device *dev,
			      const struct adc_sequence *sequence,
			      struct k_poll_signal *async)
{
	return -ENOTSUP;
}
#endif


static const struct adc_driver_api batt_driver_api = {
	.channel_setup = battery_channel_setup,
	.read = battery_voltage_read
#ifdef CONFIG_BATTERY_ASYNC
	.read_async = battery_read_async,
#endif
};

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) > 0,
	     "No compatible battery driver instance found");

#define BATTERY_OBJ_INIT(inst)									\
	static struct batmon_data battery_drv_data_##inst;			\
	static const struct batmon_cfg battery_drv_config_##inst;	\
	DEVICE_DT_INST_DEFINE(inst,									\
			      battery_init,									\
			      NULL,											\
			      &battery_drv_data_##inst,						\
			      &battery_drv_config_##inst,					\
			      POST_KERNEL,									\
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,			\
			      &batt_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BATTERY_OBJ_INIT)
