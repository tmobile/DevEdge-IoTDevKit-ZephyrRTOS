/*
 * Copyright (c) 2022 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_TSL2540_TSL2540_H_
#define ZEPHYR_DRIVERS_SENSOR_TSL2540_TSL2540_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/tsl2540.h>

#define TSL2540_REG_EN	     0x80
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
#define TSL2540_REG_CFG_3    0xab
#define TSL2540_REG_AZ_CFG   0xd6
#define TSL2540_REG_INT_EN   0xdd

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

#define TSL2540_EN_ACTIVE 0x03
#define TSL2540_EN_IDLE	  0x01
#define TSL2540_EN_SLEEP  0x00

#define TSL2540_CFG2_G1_2 0x00
#define TSL2540_CFG2_G1	  0x04
#define TSL2540_CFG2_G4	  0x04
#define TSL2540_CFG2_G16  0x04
#define TSL2540_CFG2_G64  0x04
#define TSL2540_CFG2_G128 0x14

#define TSL2540_CFG3_INTRC 0x80
#define TSL2540_CFG3_SAI   0x08

#define TSL2540_INT_EN_AEN 0x10

struct tsl2540_config {
	const struct i2c_dt_spec i2c_spec;
	const struct gpio_dt_spec int_gpio;
};

#if 0
static const struct {
	const uint8_t regAddr;
	const char *regName;
	const char *access;
	/*
	struct {
		int readable:1, writeable:1;
	} access;
	*/
	const char *description;
	const uint8_t regDefault;
} atomicShadow[] = {
	{0x80, "ENABLE", "R/W", "Enables states and functions", 0x00},
	{0x81, "ATIME", "R/W", "ALS integration time", 0x00},
	{0x83, "WTIME", "R/W", "Wait time", 0x00},
	{0x84, "AILTL", "R/W", "ALS interrupt low threshold low byte", 0x00},
	{0x85, "AILTH", "R/W", "ALS interrupt low threshold high byte", 0x00},
	{0x86, "AIHTL", "R/W", "ALS interrupt high threshold low byte", 0x00},
	{0x87, "AIHTH", "R/W", "ALS interrupt high threshold high byte", 0x00},
	{0x8C, "PERS", "R/W", "ALS interrupt persistence filters", 0x00},
	{0x8D, "CFG0", "R/W", "Configuration register zero", 0x80},
	{0x90, "CFG1", "R/W", "Configuration register one", 0x00},
	{0x91, "REVID", "R", "Revision ID", 0x61},
	{0x92, "ID", "R", "Device ID", 0xE4},
	{0x93, "STATUS", "R", "Device status register", 0x00},
	{0x94, "VISDATAL", "R", "Visible channel data low byte", 0x00},
	{0x95, "VISDATAH", "R", "Visible channel data high byte", 0x00},
	{0x96, "IRDATAL", "R", "IR channel data low byte", 0x00},
	{0x97, "IRDATAH", "R", "IR channel data high byte", 0x00},
	{0x9E, "REVID2", "R", "Auxiliary ID", 0x00 /*REVID2*/},
	{0x9F, "CFG2", "R/W", "Configuration register two", 0x04},
	{0xAB, "CFG3", "R/W", "Configuration register three", 0x0C},
	{0xD6, "AZ_CONFIG", "R/W", "Autozero configuration", 0x7F},
	{0xDD, "INTENAB", "R/W", "Interrupt enables", 0x00}
};
#endif
typedef struct {const uint8_t address; uint8_t value; const char *name; const char *access; const char *description;} reg8_t;
typedef struct {const uint8_t address; uint16_t value; const char *name; const char *access; const char *description;} reg16_t;
struct tsl2540_data {
	const struct device *i2c;
	struct k_sem sem;
	struct {
		reg8_t ENABLE;		/* 0x80: 0x00, R/W, Enables states and functions */
		reg8_t ATIME;		/* 0x81: 0x00, R/W, ALS integration time */
		reg8_t WTIME;		/* 0x83: 0x00, R/W, Wait time */
		reg16_t AILT;		/* 0x84: 0x0000, R/W, ALS interrupt low threshold low byte */
		reg16_t AIHT;		/* 0x86: 0x0000, R/W, ALS interrupt high threshold low byte */
		reg8_t PERS;		/* 0x8C: 0x00, R/W, ALS interrupt persistence filters */
		reg8_t CFG0;		/* 0x8D: 0x80, R/W, Configuration register zero */
		reg8_t CFG1;		/* 0x90: 0x00, R/W, Configuration register one */
		reg8_t REVID;		/* 0x91: 0x61, R, Revision ID */
		reg8_t ID;			/* 0x92: 0xE4, R, Device ID */
		reg8_t	STATUS;		/* 0x92: 0x00, R, Device status register */
		reg16_t VISDATA;	/* 0x94: 0x0000, R, Visible channel data low byte */
		reg16_t IRDATA;		/* 0x96: 0x00, R, IR channel data low byte */
		reg8_t REVID2;		/* 0x9E: REVID2, R, Auxiliary ID */
		reg8_t CFG2;		/* 0x9F: 0x04, R/W, Configuration register two */
		reg8_t CFG3;		/* 0xAB: 0x0C, R/W, Configuration register three */
		reg8_t AZ_CONFIG;	/* 0xD6: 0x7F, R/W, Autozero configuration */
		reg8_t INTENAB;		/* 0xDD: 0x00, R/W, Interrupt enables */
	} reg;
	const struct device *dev;
#ifdef CONFIG_TSL2540_TRIGGER
	struct gpio_callback gpio_cb;
	// enum interrupt_type int_type;
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

#define PROTO_GETTER_SETTER(name, type)                                                              \
	extern type get_##name(const struct device *dev);                                                 \
	extern void set_##name(const struct device *dev, const type value);                               \
	extern int fetch_##name(const struct device *dev);                                                \
	extern int flush_##name(const struct device *dev);

PROTO_GETTER_SETTER(ENABLE, uint8_t)
PROTO_GETTER_SETTER(ATIME, uint8_t)
PROTO_GETTER_SETTER(WTIME, uint8_t)
PROTO_GETTER_SETTER(AILT, uint16_t)
PROTO_GETTER_SETTER(AIHT, uint16_t)
PROTO_GETTER_SETTER(PERS, uint8_t)
PROTO_GETTER_SETTER(CFG0, uint8_t)
PROTO_GETTER_SETTER(CFG1, uint8_t)
PROTO_GETTER_SETTER(REVID, uint8_t)
PROTO_GETTER_SETTER(ID, uint8_t)
PROTO_GETTER_SETTER(STATUS, uint8_t)
PROTO_GETTER_SETTER(VISDATA, uint16_t)
PROTO_GETTER_SETTER(IRDATA, uint16_t)
PROTO_GETTER_SETTER(REVID2, uint8_t)
PROTO_GETTER_SETTER(CFG2, uint8_t)
PROTO_GETTER_SETTER(CFG3, uint8_t)
PROTO_GETTER_SETTER(AZ_CONFIG, uint8_t)
PROTO_GETTER_SETTER(INTENAB, uint8_t)

extern int fetch_all(const struct device *dev);
extern int flush_all(const struct device *dev);

// int tsl2540_reg_read(const struct device *dev, uint8_t reg, uint8_t *val);
// int tsl2540_reg_write(const struct device *dev, uint8_t reg, uint8_t val);

// #ifndef ZEPHYR_INCLUDE_LOGGING_LOG_H_
// #define LOG_MODULE_NAME tsl2540
// #include <zephyr/logging/log.h>
// LOG_MODULE_DECLARE(LOG_MODULE_NAME, CONFIG_SENSOR_LOG_LEVEL);
// LOG_MODULE_DECLARE(tsl2540);
// #endif

#if 1
// extern int tsl2540_reg_read(const struct device *dev, uint8_t reg, uint8_t *val);
// extern int tsl2540_reg_write(const struct device *dev, uint8_t reg, uint8_t val);
#else
inline static int tsl2540_reg_read(const struct device *dev, uint8_t reg, uint8_t *val)
{
	// const struct tsl2540_config *cfg = dev->config;
	int result;

	result = i2c_reg_read_byte_dt(&((const struct tsl2540_config *) dev->config)->i2c_spec, reg, val);
	LOG_DBG("%s:%d: int %s(*dev: %p, reg: %#.02x, val: %#.02x): %d", __FILE__, __LINE__,
		__func__, dev, reg, *val, result);

	if (result < 0) {
		return result;
	}

	return 0;
}

inline static int tsl2540_reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	// const struct tsl2540_config *cfg = dev->config;
	int result;

	result = i2c_reg_write_byte_dt(&((const struct tsl2540_config *) dev->config)->i2c_spec, reg, val);
	LOG_DBG("%s:%d int %s(*dev: %p, reg: %#.02x, val: %#.02x): %d", __FILE__, __LINE__,
		__func__, dev, reg, val, result);

	if (result < 0) {
		return result;
	}

	return 0;
}
#endif

#ifdef CONFIG_TSL2540_TRIGGER
int tsl2540_trigger_init(const struct device *dev);

int tsl2540_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);
#endif

#endif
