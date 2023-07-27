/*
 * Copyright(c) 2020 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_BATTERY_BQ274XX_H_
#define ZEPHYR_DRIVERS_SENSOR_BATTERY_BQ274XX_H_

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

/*** General Constant ***/
#define BQ274XX_UNSEAL_KEY_A 0x8000 /* Unseal code one on BQ27441-G1A and similar */
#define BQ274XX_UNSEAL_KEY_B 0x8000 /* Unseal code two on BQ27441-G1A and similar */
#define BQ27421_DEVICE_ID  0x0421
#define BQ27427_DEVICE_ID  0x0427

/*** Standard Commands ***/
#define BQ274XX_COMMAND_CONTROL_LOW 0x00 /* Control() low register */
#define BQ274XX_COMMAND_CONTROL_HIGH 0x01 /* Control() high register */
#define BQ274XX_COMMAND_TEMP 0x02 /* Temperature() */
#define BQ274XX_COMMAND_VOLTAGE 0x04 /* Voltage() */
#define BQ274XX_COMMAND_FLAGS 0x06 /* Flags() */
#define BQ274XX_COMMAND_NOM_CAPACITY 0x08 /* NominalAvailableCapacity() */
#define BQ274XX_COMMAND_AVAIL_CAPACITY 0x0A /* FullAvailableCapacity() */
#define BQ274XX_COMMAND_REM_CAPACITY 0x0C /* RemainingCapacity() */
#define BQ274XX_COMMAND_FULL_CAPACITY 0x0E /* FullChargeCapacity() */
#define BQ274XX_COMMAND_AVG_CURRENT 0x10 /* AverageCurrent() */
#define BQ274XX_COMMAND_STDBY_CURRENT 0x12 /* StandbyCurrent() */
#define BQ274XX_COMMAND_MAX_CURRENT 0x14 /* MaxLoadCurrent() */
#define BQ274XX_COMMAND_AVG_POWER 0x18 /* AveragePower() */
#define BQ274XX_COMMAND_SOC 0x1C /* StateOfCharge() */
#define BQ274XX_COMMAND_INT_TEMP 0x1E /* InternalTemperature() */
#define BQ274XX_COMMAND_SOH 0x20 /* StateOfHealth() */
#define BQ274XX_COMMAND_REM_CAP_UNFL 0x28 /* RemainingCapacityUnfiltered() */
#define BQ274XX_COMMAND_REM_CAP_FIL 0x2A /* RemainingCapacityFiltered() */
#define BQ274XX_COMMAND_FULL_CAP_UNFL 0x2C /* FullChargeCapacityUnfiltered() */
#define BQ274XX_COMMAND_FULL_CAP_FIL 0x2E /* FullChargeCapacityFiltered() */
#define BQ274XX_COMMAND_SOC_UNFL 0x30 /* StateOfChargeUnfiltered() */

/*** Control Sub-Commands ***/
#define BQ274XX_CONTROL_STATUS 0x0000
#define BQ274XX_CONTROL_DEVICE_TYPE 0x0001
#define BQ274XX_CONTROL_FW_VERSION 0x0002
#define BQ274XX_CONTROL_DM_CODE 0x0004
#define BQ274XX_CONTROL_PREV_MACWRITE 0x0007
#define BQ274XX_CONTROL_CHEM_ID 0x0008
#define BQ274XX_CONTROL_BAT_INSERT 0x000C
#define BQ274XX_CONTROL_BAT_REMOVE 0x000D
#define BQ274XX_CONTROL_SET_HIBERNATE 0x0011
#define BQ274XX_CONTROL_CLEAR_HIBERNATE 0x0012
#define BQ274XX_CONTROL_SET_CFGUPDATE 0x0013
#define BQ274XX_CONTROL_SHUTDOWN_ENABLE 0x001B
#define BQ274XX_CONTROL_SHUTDOWN 0x001C
#define BQ274XX_CONTROL_SEALED 0x0020
#define BQ274XX_CONTROL_PULSE_SOC_INT 0x0023
#define BQ274XX_CONTROL_RESET 0x0041
#define BQ274XX_CONTROL_SOFT_RESET 0x0042
#define BQ274XX_CONTROL_EXIT_CFGUPDATE 0x0043
#define BQ274XX_CONTROL_EXIT_RESIM 0x0044

/*** Extended Data Commands ***/
#define BQ274XX_EXT_OPCONFIG                   0x3A /* OpConfig() */
#define BQ274XX_EXT_CAPACITY                   0x3C /* DesignCapacity() */
#define BQ274XX_EXT_DATA_CLASS                 0x3E /* DataClass() */
#define BQ274XX_EXT_DATA_BLOCK                 0x3F /* DataBlock() */
#define BQ274XX_EXT_BLKDAT_START               0x40 /* BlockData_start() */
#define BQ274XX_EXT_BLKDAT_END                 0x5F /* BlockData_end() */
#define BQ274XX_EXT_CHECKSUM                   0x60 /* BlockDataCheckSum() */
#define BQ274XX_EXT_DATA_CONTROL               0x61 /* BlockDataControl() */
#define BQ274XX_EXT_BLKDAT_HIGH(off)           (BQ274XX_EXT_BLKDAT_START + off)
#define BQ274XX_EXT_BLKDAT_LOW(off)            (BQ274XX_EXT_BLKDAT_START + off + 1)

/* Hold the register offset for a device variant. */
struct bq274xx_regs {
	uint8_t dm_design_capacity;
	uint8_t dm_design_energy;
	uint8_t dm_terminate_voltage;
	uint8_t dm_taper_rate;
};

struct bq274xx_data {
	const struct bq274xx_regs *regs;
	bool configured;
	uint16_t voltage;
	int16_t avg_current;
	int16_t stdby_current;
	int16_t max_load_current;
	int16_t avg_power;
	uint16_t state_of_charge;
	int16_t state_of_health;
	uint16_t internal_temperature;
	uint16_t full_charge_capacity;
	uint16_t remaining_charge_capacity;
	uint16_t nom_avail_capacity;
	uint16_t full_avail_capacity;

#ifdef CONFIG_BQ274XX_TRIGGER
	const struct device *dev;
	struct gpio_callback ready_callback;
	sensor_trigger_handler_t ready_handler;
	const struct sensor_trigger *ready_trig;

#ifdef CONFIG_BQ274XX_TRIGGER_OWN_THREAD
	struct k_sem sem;
#endif

#ifdef CONFIG_BQ274XX_TRIGGER_GLOBAL_THREAD
	struct k_work work;
#endif
#endif /* CONFIG_BQ274XX_TRIGGER */
};

struct bq274xx_config {
	struct i2c_dt_spec i2c;
	uint16_t design_voltage;
	uint16_t design_capacity;
	uint16_t taper_current;
	uint16_t terminate_voltage;
#if defined(CONFIG_BQ274XX_PM) || defined(CONFIG_BQ274XX_TRIGGER)
	struct gpio_dt_spec int_gpios;
#endif
	bool lazy_loading;
};

int bq274xx_trigger_mode_init(const struct device *dev);
int bq274xx_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);

#endif
