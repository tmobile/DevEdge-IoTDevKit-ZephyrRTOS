/*
 * Copyright (c) 2022 T-Mobile USA, Inc.
 * Copyright (c) 2023 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_TSL2540_TSL2540_H_
#define ZEPHYR_DRIVERS_SENSOR_TSL2540_TSL2540_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/tsl2540.h>

#define TSL2540_REG_ATIME    0x81
#define TSL2540_REG_WTIME    0x83
#define TSL2540_REG_AILT_LOW 0x84
#define TSL2540_REG_AILT_HI  0x85
#define TSL2540_REG_AIHT_LOW 0x86
#define TSL2540_REG_AIHT_HI  0x87
#define TSL2540_REG_PERS     0x8c
#define TSL2540_REG_CFG_0    0x8d
#define TSL2540_REG_CFG_1    0x90
#define TSL2540_REG_REVID    0x91
#define TSL2540_REG_ID	     0x92
#define TSL2540_REG_STATUS   0x93
#define TSL2540_REG_VIS_LOW  0x94
#define TSL2540_REG_VIS_HI   0x95
#define TSL2540_REG_IR_LOW   0x96
#define TSL2540_REG_IR_HI    0x97
#define TSL2540_REG_REVID2   0x9E
#define TSL2540_REG_CFG_2    0x9f

#define TSL2540_AGAIN_S1_2 0.5
#define TSL2540_AGAIN_S1   1
#define TSL2540_AGAIN_S4   4
#define TSL2540_AGAIN_S16  16
#define TSL2540_AGAIN_S64  67
#define TSL2540_AGAIN_S128 140

#define TSL2540_CFG1_G1_2 0x00
#define TSL2540_CFG1_G1	  0x00
#define TSL2540_CFG1_G4	  0x01
#define TSL2540_CFG1_G16  0x02
#define TSL2540_CFG1_G64  0x03
#define TSL2540_CFG1_G128 0x03

#define TSL2540_CFG2_G1_2 0x00
#define TSL2540_CFG2_G1	  0x04
#define TSL2540_CFG2_G4	  0x04
#define TSL2540_CFG2_G16  0x04
#define TSL2540_CFG2_G64  0x04
#define TSL2540_CFG2_G128 0x14

/* ENABLE(0x80: 0x00): Reserved:7:4 | WEN:3 | Reserved:2 | AEN:1 | PON:0 */
#define TSL2540_ENABLE_ADDR 0x80
#define TSL2540_ENABLE_MASK ((0000 << 4) | (1 << 3) | (0 << 2) | (1 << 1) | (1 << 0))
#define TSL2540_ENABLE_CONF ((0000 << 4) | (1 << 3) | (0 << 2) | (1 << 1) | (1 << 0))

/* CRG3(0xAB: 0x0C):  INT_READ_CLEAR:7 | Reserved:6:5 | SAI:4 | Reserved:3:0 */
#define TSL2540_CFG3_ADDR 0xAB
#define TSL2540_CFG3_MASK ((1 << 7) | (00 << 5) | (1 << 4) | (0000 << 0))
#define TSL2540_CFG3_CONF ((0 << 7) | (00 << 5) | (1 << 4) | (0000 << 0))

/* INTENAB(0xDD: 0x00): ASIEN:7 | Reserved:6:5 | AIEN:4 | Reserved:3:0 */
#define TSL2540_INTENAB_ADDR 0xDD
#define TSL2540_INTENAB_MASK ((1 << 7) | (00 < 5) | (1 << 4) | (0000 << 0))
#define TSL2540_INTENAB_CONF ((0 << 7) | (00 < 5) | (1 << 4) | (0000 << 0))

#define TSL2540_INT_EN_AEN 0x90

struct tsl2540_config {
	const struct i2c_dt_spec i2c_spec;
	const struct gpio_dt_spec int_gpio;
};

struct tsl2540_data {
	const struct device *i2c;
	struct k_sem sem;
#ifdef CONFIG_TSL2540_TRIGGER
	const struct device *dev;
	struct gpio_callback gpio_cb;
	sensor_trigger_handler_t als_handler;
#endif
#ifdef CONFIG_TSL2540_TRIGGER_OWN_THREAD
	K_THREAD_STACK_MEMBER(thread_stack, CONFIG_TSL2540_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem trig_sem;
#endif
#ifdef CONFIG_TSL2540_TRIGGER_GLOBAL_THREAD
	struct k_work work;
#endif
	uint16_t count_vis;
	uint16_t count_ir;
	double glass_attenuation;
	double glass_attenuation_ir;
	uint8_t integration_time;
	double again;
};

int tsl2540_reg_read(const struct device *dev, uint8_t reg, uint8_t *val);
int tsl2540_reg_write(const struct device *dev, uint8_t reg, uint8_t val);

#ifdef CONFIG_TSL2540_TRIGGER
int tsl2540_trigger_init(const struct device *dev);

int tsl2540_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);
#endif

#endif
