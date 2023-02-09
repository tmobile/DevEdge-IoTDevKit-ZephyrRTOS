/*
 * Copyright (c) 2022 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_BATTERY_H_
#define ZEPHYR_INCLUDE_DRIVERS_BATTERY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/drivers/adc.h>


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

/* battery driver data */
struct batmon_data {
	uint32_t mVolts;
	uint8_t percent;
	uint8_t battery_attached;
	uint8_t charging;
    uint8_t vbus;
	uint8_t fault;
	int charging_status;
	int hwid;
};

struct batmon_cfg {
	uint32_t sample_freq;
	uint16_t vref_internal_mv;
};



#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_BATTERY_H_ */
