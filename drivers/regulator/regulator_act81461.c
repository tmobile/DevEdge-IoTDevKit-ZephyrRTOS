/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 * Copyright (c) 2023 T-Mobile USA, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ACT81461 PMIC Regulator Driver
 * This driver implements the ACT81461 regulator API within Zephyr,
 * and also implements support for a broader API.
 */

#define DT_DRV_COMPAT qorvo_act81461

#include <zephyr/kernel.h>
#include <zephyr/sys/slist.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/linear_range.h>
#include <zephyr/sys/util.h>
#include <errno.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/regulator/act81461.h>
#include "regulator_act81461.h"

#define NO_LIMITS (!IS_ENABLED(CONFIG_REGULATOR_ACT81461_VOLTAGE_LIMITS))

#define BUCKBOOST_VSET_MIN CONFIG_REGULATOR_ACT81461_BUCKBOOST_VSET_MIN
#define BUCKBOOST_VSET_MAX CONFIG_REGULATOR_ACT81461_BUCKBOOST_VSET_MAX

#define BUCK1_VSET_MIN CONFIG_REGULATOR_ACT81461_BUCK1_VSET_MIN
#define BUCK1_VSET_MAX CONFIG_REGULATOR_ACT81461_BUCK1_VSET_MAX

#define BUCK2_VSET_MIN CONFIG_REGULATOR_ACT81461_BUCK2_VSET_MIN
#define BUCK2_VSET_MAX CONFIG_REGULATOR_ACT81461_BUCK2_VSET_MAX

#define BOOST_VSET_MIN CONFIG_REGULATOR_ACT81461_BOOST_VSET_MIN
#define BOOST_VSET_MAX CONFIG_REGULATOR_ACT81461_BOOST_VSET_MAX

#define LDO1_VSET_MIN CONFIG_REGULATOR_ACT81461_LDO1_VSET_MIN
#define LDO1_VSET_MAX CONFIG_REGULATOR_ACT81461_LDO1_VSET_MAX

#define LDO2_VSET_MIN CONFIG_REGULATOR_ACT81461_LDO2_VSET_MIN
#define LDO2_VSET_MAX CONFIG_REGULATOR_ACT81461_LDO2_VSET_MAX

#define LDO3_VSET_MIN CONFIG_REGULATOR_ACT81461_LDO3_VSET_MIN
#define LDO3_VSET_MAX CONFIG_REGULATOR_ACT81461_LDO3_VSET_MAX

LOG_MODULE_REGISTER(qorvo_act81461, CONFIG_REGULATOR_LOG_LEVEL);

/* act81461 voltage sources */
enum act81461_sources {
	ACT81461_SOURCE_BUCKBOOST,
	ACT81461_SOURCE_BUCK1,
	ACT81461_SOURCE_BUCK2,
	ACT81461_SOURCE_BOOST,
	ACT81461_SOURCE_LDO1,
	ACT81461_SOURCE_LDO2,
	ACT81461_SOURCE_LDO3,
	ACT81461_LSW4,
	ACT81461_LSW5,
	ACT81461_LSW6,
};

struct regulator_act81461_parent_config {
	struct i2c_dt_spec i2c;
	const struct gpio_dt_spec int_gpio;
	uint8_t pmic_override_reg_array[16];
	uint8_t pmic_override_reg_value_array[16];
	uint8_t pmic_override_num;
};

struct regulator_act81461_node_config {
	struct regulator_common_config common;
	const struct device *p;
	uint8_t source;
};

struct regulator_act81461_data {
	struct regulator_common_data data;
	sys_slist_t fault_cbs;
};

struct regulator_act81461_common_data {
	const struct device *dev;
	struct gpio_callback gpio_int_cb;
	struct k_work int_work;
	sys_slist_t fault_cbs;
	bool reg_inited;
};

static int regulator_act81461_enable(const struct device *dev);
static int regulator_act81461_disable(const struct device *dev);
static unsigned int regulator_act81461_count_voltages(const struct device *dev);
static int regulator_act81461_list_voltage(const struct device *dev, unsigned int idx,
					   int32_t *volt_uv);
static int regulator_act81461_set_voltage(const struct device *dev, int32_t min_uv,
					   int32_t max_uv);
static int regulator_act81461_get_voltage(const struct device *dev, int32_t *volt_uv);
static int regulator_act81461_get_error_flags(const struct device *dev,
					      regulator_error_flags_t *flags);

static const struct regulator_driver_api api = {
	.enable = regulator_act81461_enable,
	.disable = regulator_act81461_disable,
	.count_voltages = regulator_act81461_count_voltages,
	.list_voltage = regulator_act81461_list_voltage,
	.set_voltage = regulator_act81461_set_voltage,
	.get_voltage = regulator_act81461_get_voltage,
	.get_error_flags = regulator_act81461_get_error_flags,
};

static const struct linear_range vbuckboost_range = LINEAR_RANGE_INIT(3200000, 50000, 0x0, 0x36);

static const struct linear_range vbuck1_range = LINEAR_RANGE_INIT(600000, 50000, 0x0, 0x3C);

static const struct linear_range vbuck2_range = LINEAR_RANGE_INIT(600000, 50000, 0x0, 0x3C);

static const struct linear_range vboost_range = LINEAR_RANGE_INIT(5000000, 250000, 0x0, 0x3F);

static const struct linear_range ldo1_range = LINEAR_RANGE_INIT(600000, 50000, 0x0, 0x3C);

static const struct linear_range ldo2_range = LINEAR_RANGE_INIT(600000, 50000, 0x0, 0x3C);

static const struct linear_range ldo3_range = LINEAR_RANGE_INIT(600000, 50000, 0x0, 0x3C);

static const struct linear_range vsysmon_range = LINEAR_RANGE_INIT(2600000, 100000, 0x05, 0x0F);

static int regulator_act81461_enable(const struct device *dev)
{
	const struct regulator_act81461_node_config *config = dev->config;
	const struct regulator_act81461_parent_config *cconfig = config->p->config;
	int ret = 0;

	if (!i2c_is_ready_dt(&cconfig->i2c)) {
		return -ENODEV;
	}

	switch (config->source) {
	case ACT81461_SOURCE_BUCKBOOST:
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, BUCKBOOST_ENABLE_REG,
					     BUCKBOOST_ENABLE_MSK, BUCKBOOST_ENABLE);
		if (ret < 0) {
			return ret;
		}
		break;

	case ACT81461_SOURCE_BUCK1:
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, BUCK1_ENABLE_REG, BUCK1_ENABLE_MSK,
					     BUCK1_ENABLE);
		if (ret < 0) {
			return ret;
		}
		break;

	case ACT81461_SOURCE_BUCK2:
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, BUCK2_ENABLE_REG, BUCK2_ENABLE_MSK,
					     BUCK2_ENABLE);
		if (ret < 0) {
			return ret;
		}
		break;

	case ACT81461_SOURCE_BOOST:
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, BOOST_ENABLE_REG, BOOST_ENABLE_MSK,
					     0x80U);
		if (ret < 0) {
			return ret;
		}
		break;

	case ACT81461_SOURCE_LDO1:
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, LDO1_ENABLE_REG, LDO1_ENABLE_MSK,
					     LDO1_ENABLE);
		if (ret < 0) {
			return ret;
		}
		break;

	case ACT81461_SOURCE_LDO2:
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, LDO2_ENABLE_REG, LDO2_ENABLE_MSK,
					     LDO2_ENABLE);
		if (ret < 0) {
			return ret;
		}
		break;

	case ACT81461_SOURCE_LDO3:
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, LDO3_ENABLE_REG, LDO1_ENABLE_MSK,
					     LDO3_ENABLE);
		if (ret < 0) {
			return ret;
		}
		break;

	case ACT81461_LSW4:
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, LSW4_ENABLE_REG, LSW4_ENABLE_MSK,
					     LSW4_ENABLE);
		if (ret < 0) {
			return ret;
		}
		break;

	case ACT81461_LSW5:
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, LSW5_ENABLE_REG, LSW5_ENABLE_MSK,
					     LSW5_ENABLE);
		if (ret < 0) {
			return ret;
		}
		break;

	case ACT81461_LSW6:
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, LSW6_ENABLE_REG, LSW6_ENABLE_MSK,
					     LSW6_ENABLE);
		if (ret < 0) {
			return ret;
		}
		break;

	default:
		return -ENODEV;
	}

	return ret;
}

static int regulator_act81461_disable(const struct device *dev)
{
	const struct regulator_act81461_node_config *config = dev->config;
	const struct regulator_act81461_parent_config *cconfig = config->p->config;
	int ret = 0;

	switch (config->source) {
	case ACT81461_SOURCE_BUCKBOOST:
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, BUCKBOOST_ENABLE_REG,
					     BUCKBOOST_ENABLE_MSK, 0U);
		if (ret < 0) {
			return ret;
		}
		break;

	case ACT81461_SOURCE_BUCK1:
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, BUCK1_ENABLE_REG, BUCK1_ENABLE_MSK, 0U);
		if (ret < 0) {
			return ret;
		}
		break;

	case ACT81461_SOURCE_BUCK2:
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, BUCK2_ENABLE_REG, BUCK2_ENABLE_MSK, 0U);
		if (ret < 0) {
			return ret;
		}
		break;

	case ACT81461_SOURCE_BOOST:
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, BOOST_ENABLE_REG, BOOST_ENABLE_MSK, 0U);
		if (ret < 0) {
			return ret;
		}
		break;

	case ACT81461_SOURCE_LDO1:
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, LDO1_ENABLE_REG, LDO1_ENABLE_MSK, 0U);
		if (ret < 0) {
			return ret;
		}
		break;

	case ACT81461_SOURCE_LDO2:
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, LDO2_ENABLE_REG, LDO2_ENABLE_MSK, 0U);
		if (ret < 0) {
			return ret;
		}
		break;

	case ACT81461_SOURCE_LDO3:
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, LDO3_ENABLE_REG, LDO3_ENABLE_MSK,
					     0x00U);
		if (ret < 0) {
			return ret;
		}
		break;

	case ACT81461_LSW4:
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, LSW4_ENABLE_REG, LSW4_ENABLE_MSK, 0U);
		if (ret < 0) {
			return ret;
		}
		break;

	case ACT81461_LSW5:
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, LSW5_ENABLE_REG, LSW5_ENABLE_MSK, 0U);
		if (ret < 0) {
			return ret;
		}
		break;

	case ACT81461_LSW6:
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, LSW6_ENABLE_REG, LSW6_ENABLE_MSK, 0U);
		if (ret < 0) {
			return ret;
		}
		break;

	default:
		return -ENODEV;
	}

	return 0;
}

static unsigned int regulator_act81461_count_voltages(const struct device *dev)
{
	const struct regulator_act81461_node_config *config = dev->config;
	uint32_t vcount = 0;

	LOG_DBG("count_voltage source %d", config->source);
	switch (config->source) {
	case ACT81461_SOURCE_BUCKBOOST:
		vcount = linear_range_values_count(&vbuckboost_range);
		break;

	case ACT81461_SOURCE_BUCK1:
		vcount = linear_range_values_count(&vbuck1_range);
		break;

	case ACT81461_SOURCE_BUCK2:
		vcount = linear_range_values_count(&vbuck2_range);
		break;

	case ACT81461_SOURCE_BOOST:
		vcount = linear_range_values_count(&vboost_range);
		break;

	case ACT81461_SOURCE_LDO1:
		vcount = linear_range_values_count(&ldo1_range);
		break;

	case ACT81461_SOURCE_LDO2:
		vcount = linear_range_values_count(&ldo2_range);
		break;

	case ACT81461_SOURCE_LDO3:
		vcount = linear_range_values_count(&ldo3_range);
		break;

	case ACT81461_LSW4:
	case ACT81461_LSW5:
	case ACT81461_LSW6:
		return -ENOTSUP;

	default:
		LOG_ERR("Unexpected source %d", config->source);
		break;
	}

	LOG_DBG("The voltage range count is %d steps", vcount);
	return vcount;
}

static int regulator_act81461_list_voltage(const struct device *dev, unsigned int idx,
					   int32_t *volt_uv)
{
	const struct regulator_act81461_node_config *config = dev->config;

	LOG_DBG("list_voltage source %d", config->source);
	switch (config->source) {
	case ACT81461_SOURCE_BUCKBOOST:
		/* *volt_uv = 3300000; */
		return linear_range_get_value(&vbuckboost_range, idx, volt_uv);
	case ACT81461_SOURCE_BUCK1:
		/* *volt_uv = 1850000; */
		return linear_range_get_value(&vbuck1_range, idx, volt_uv);
	case ACT81461_SOURCE_BUCK2:
		return linear_range_get_value(&vbuck2_range, idx, volt_uv);
	case ACT81461_SOURCE_BOOST:
		return linear_range_get_value(&vboost_range, idx, volt_uv);
	case ACT81461_SOURCE_LDO1:
		/* *volt_uv = 2200000; */
		return linear_range_get_value(&ldo1_range, idx, volt_uv);
	case ACT81461_SOURCE_LDO2:
		return linear_range_get_value(&ldo2_range, idx, volt_uv);
	case ACT81461_SOURCE_LDO3:
		/* *volt_uv = 1850000; */
		return linear_range_get_value(&ldo3_range, idx, volt_uv);
	case ACT81461_LSW4:
	case ACT81461_LSW5:
	case ACT81461_LSW6:
		return -ENOTSUP;
	default:
		LOG_ERR("Unexpected source %d", config->source);
		break;
	}

	return 0;
}

static int regulator_act81461_vbuckboost_set_voltage(const struct device *dev, int32_t min_uv,
						     int32_t max_uv)
{
	const struct regulator_act81461_node_config *config = dev->config;
	const struct regulator_act81461_parent_config *cconfig = config->p->config;
	uint16_t idx;
	int ret = 0;
	uint8_t val;

	ret = linear_range_get_win_index(&vbuckboost_range, min_uv, max_uv, &idx);
	if (ret == -EINVAL) {
		return ret;
	}

	val = BUCKBOOST_SWFREQ_MSK + idx;
#if !NO_LIMITS
	if ((idx >= BUCKBOOST_VSET_MIN) && (idx <= BUCKBOOST_VSET_MAX))
#endif
	{
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, VBUCKBOOST_VSET_REG,
					     VBUCKBOOST_VSET_MSK, (uint8_t)idx);
		if (ret < 0) {
			return ret;
		}
	}
#if !NO_LIMITS
	else {
		LOG_ERR("Set voltage out of configured range");
		ret = -ERANGE;
	}
#endif
	return ret;
}

static int regulator_act81461_vbuck1_set_voltage(const struct device *dev, int32_t min_uv,
						 int32_t max_uv)
{
	const struct regulator_act81461_node_config *config = dev->config;
	const struct regulator_act81461_parent_config *cconfig = config->p->config;
	uint16_t idx;
	int ret = 0;
	uint8_t val;

	ret = linear_range_get_win_index(&vbuck1_range, min_uv, max_uv, &idx);
	if (ret == -EINVAL) {
		return ret;
	}

	val = BUCK1_SWFREQ_MSK + idx;
#if !NO_LIMITS
	if ((idx >= BUCK1_VSET_MIN) && (idx <= BUCK1_VSET_MAX))
#endif
	{
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, VBUCK1_VSET_REG, VBUCK1_VSET_MSK,
					     (uint8_t)idx);
		if (ret < 0) {
			return ret;
		}
	}
#if !NO_LIMITS
	else {
		LOG_ERR("Set voltage out of configured range");
		ret = -ERANGE;
	}
#endif

	return ret;
}

static int regulator_act81461_vbuck2_set_voltage(const struct device *dev, int32_t min_uv,
						 int32_t max_uv)
{
	const struct regulator_act81461_node_config *config = dev->config;
	const struct regulator_act81461_parent_config *cconfig = config->p->config;
	uint16_t idx;
	int ret = 0;
	uint8_t val;

	ret = linear_range_get_win_index(&vbuck2_range, min_uv, max_uv, &idx);
	if (ret == -EINVAL) {
		return ret;
	}

	val = BUCK2_SWFREQ_MSK + idx;
#if !NO_LIMITS
	if ((idx >= BUCK2_VSET_MIN) && (idx <= BUCK2_VSET_MAX))
#endif
	{
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, VBUCK2_VSET_REG, VBUCK2_VSET_MSK,
					     (uint8_t)idx);
		if (ret < 0) {
			return ret;
		}
	}
#if !NO_LIMITS
	else {
		LOG_ERR("Set voltage out of configured range");
		ret = -ERANGE;
	}
#endif

	return ret;
}

static int regulator_act81461_vboost_set_voltage(const struct device *dev, int32_t min_uv,
						 int32_t max_uv)
{
	const struct regulator_act81461_node_config *config = dev->config;
	const struct regulator_act81461_parent_config *cconfig = config->p->config;
	uint16_t idx;
	int ret = 0;
	uint8_t val;

	ret = linear_range_get_win_index(&vboost_range, min_uv, max_uv, &idx);
	if (ret == -EINVAL) {
		return ret;
	}

	val = BOOST_SWFREQ_MSK + idx;
#if !NO_LIMITS
	if ((idx >= BOOST_VSET_MIN) && (idx <= BOOST_VSET_MAX))
#endif
	{
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, VBOOST_VSET_REG, VBOOST_VSET_MSK,
					     (uint8_t)idx);
		if (ret < 0) {
			return ret;
		}
	}
#if !NO_LIMITS
	else {
		LOG_ERR("Set voltage out of configured range");
		ret = -ERANGE;
	}
#endif

	return ret;
}

static int regulator_act81461_ldo1_set_voltage(const struct device *dev, int32_t min_uv,
					       int32_t max_uv)
{
	const struct regulator_act81461_node_config *config = dev->config;
	const struct regulator_act81461_parent_config *cconfig = config->p->config;
	uint16_t idx;
	uint8_t val;
	int ret = 0;

	ret = linear_range_get_win_index(&ldo1_range, min_uv, max_uv, &idx);
	if (ret == -EINVAL) {
		return ret;
	}

	val = LDO1_SWFREQ_MSK + idx;
	/* Limit the Min (2.05v), Max (2.45v) ldo3 voltage setting */
#if !NO_LIMITS
	if ((idx >= LDO1_VSET_MIN) && (idx <= LDO1_VSET_MAX))
#endif
	{
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, LDO1_VSET_REG, LDO1_VSET_MSK,
					     (uint8_t)idx);
		if (ret < 0) {
			return ret;
		}
	}
#if !NO_LIMITS
	else {
		LOG_ERR("Set voltage out of configured range");
		ret = -ERANGE;
	}
#endif

	return ret;
}

static int regulator_act81461_ldo2_set_voltage(const struct device *dev, int32_t min_uv,
					       int32_t max_uv)
{
	const struct regulator_act81461_node_config *config = dev->config;
	const struct regulator_act81461_parent_config *cconfig = config->p->config;
	uint16_t idx;
	uint8_t val;
	int ret = 0;

	ret = linear_range_get_win_index(&ldo2_range, min_uv, max_uv, &idx);
	if (ret == -EINVAL) {
		return ret;
	}

	val = LDO2_SWFREQ_MSK + idx;
#if !NO_LIMITS
	if ((idx >= LDO2_VSET_MIN) && (idx <= LDO2_VSET_MAX))
#endif
	{
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, LDO2_VSET_REG, LDO2_VSET_MSK,
					     (uint8_t)idx);
		if (ret < 0) {
			return ret;
		}
	}
#if !NO_LIMITS
	else {
		LOG_ERR("Set voltage out of configured range");
		ret = -ERANGE;
	}
#endif

	return ret;
}

static int regulator_act81461_ldo3_set_voltage(const struct device *dev, int32_t min_uv,
					       int32_t max_uv)
{
	const struct regulator_act81461_node_config *config = dev->config;
	const struct regulator_act81461_parent_config *cconfig = config->p->config;
	uint16_t idx;
	uint8_t val;
	int ret = 0;

	ret = linear_range_get_win_index(&ldo3_range, min_uv, max_uv, &idx);
	if (ret == -EINVAL) {
		return ret;
	}

	val = LDO3_SWFREQ_MSK + idx;
#if !NO_LIMITS
	if ((idx >= LDO3_VSET_MIN) && (idx <= LDO3_VSET_MAX))
#endif
	{
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, LDO3_VSET_REG, LDO3_VSET_MSK,
					     (uint8_t)idx);
		if (ret < 0) {
			return ret;
		}
	}
#if !NO_LIMITS
	else {
		LOG_ERR("Set voltage out of configured range");
		ret = -ERANGE;
	}
#endif

	return ret;
}

static int regulator_act81461_ldo1_get_voltage(const struct device *dev, int32_t *volt_uv)
{
	const struct regulator_act81461_node_config *config = dev->config;
	const struct regulator_act81461_parent_config *cconfig = config->p->config;
	uint8_t val;
	int ret = 0;

	ret = i2c_reg_read_byte_dt(&cconfig->i2c, LDO1_VSET_REG, &val);
	if (ret < 0) {
		return ret;
	}

	val &= LDO1_VSET_MSK;
	return linear_range_get_value(&ldo1_range, val, volt_uv);
}

static int regulator_act81461_ldo2_get_voltage(const struct device *dev, int32_t *volt_uv)
{
	const struct regulator_act81461_node_config *config = dev->config;
	const struct regulator_act81461_parent_config *cconfig = config->p->config;
	uint8_t val;
	int ret = 0;

	ret = i2c_reg_read_byte_dt(&cconfig->i2c, LDO2_VSET_REG, &val);
	if (ret < 0) {
		return ret;
	}

	val &= LDO2_VSET_MSK;
	return linear_range_get_value(&ldo2_range, val, volt_uv);
}

static int regulator_act81461_ldo3_get_voltage(const struct device *dev, int32_t *volt_uv)
{
	const struct regulator_act81461_node_config *config = dev->config;
	const struct regulator_act81461_parent_config *cconfig = config->p->config;
	uint8_t val;
	int ret = 0;

	ret = i2c_reg_read_byte_dt(&cconfig->i2c, LDO3_VSET_REG, &val);
	if (ret < 0) {
		return ret;
	}

	val &= LDO3_VSET_MSK;
	return linear_range_get_value(&ldo3_range, val, volt_uv);
}

static int regulator_act81461_vbuck1_get_voltage(const struct device *dev, int32_t *volt_uv)
{
	const struct regulator_act81461_node_config *config = dev->config;
	const struct regulator_act81461_parent_config *cconfig = config->p->config;
	uint8_t val;
	int ret = 0;

	ret = i2c_reg_read_byte_dt(&cconfig->i2c, VBUCK1_VSET_REG, &val);
	if (ret < 0) {
		return ret;
	}

	val &= VBUCK1_VSET_MSK;
	return linear_range_get_value(&vbuck1_range, val, volt_uv);
}

static int regulator_act81461_vbuck2_get_voltage(const struct device *dev, int32_t *volt_uv)
{
	const struct regulator_act81461_node_config *config = dev->config;
	const struct regulator_act81461_parent_config *cconfig = config->p->config;
	uint8_t val;
	int ret = 0;

	ret = i2c_reg_read_byte_dt(&cconfig->i2c, VBUCK2_VSET_REG, &val);
	if (ret < 0) {
		return ret;
	}

	val &= VBUCK2_VSET_MSK;
	return linear_range_get_value(&vbuck2_range, val, volt_uv);
}

static int regulator_act81461_vboost_get_voltage(const struct device *dev, int32_t *volt_uv)
{
	const struct regulator_act81461_node_config *config = dev->config;
	const struct regulator_act81461_parent_config *cconfig = config->p->config;
	uint8_t val;
	int ret = 0;

	ret = i2c_reg_read_byte_dt(&cconfig->i2c, VBOOST_VSET_REG, &val);
	if (ret < 0) {
		return ret;
	}

	val &= VBOOST_VSET_MSK;
	return linear_range_get_value(&vboost_range, val, volt_uv);
}

static int regulator_act81461_vbuckboost_get_voltage(const struct device *dev, int32_t *volt_uv)
{
	const struct regulator_act81461_node_config *config = dev->config;
	const struct regulator_act81461_parent_config *cconfig = config->p->config;
	uint8_t val;
	int ret;

	ret = i2c_reg_read_byte_dt(&cconfig->i2c, VBUCKBOOST_VSET_REG, &val);
	if (ret < 0) {
		return ret;
	}

	val &= VBUCKBOOST_VSET_MSK;
	return linear_range_get_value(&vbuckboost_range, val, volt_uv);
}

static int regulator_act81461_set_voltage(const struct device *dev, int32_t min_uv, int32_t max_uv)
{
	const struct regulator_act81461_node_config *config = dev->config;

	switch (config->source) {
	case ACT81461_SOURCE_BUCKBOOST:
		return regulator_act81461_vbuckboost_set_voltage(dev, min_uv, max_uv);
	case ACT81461_SOURCE_BUCK1:
		return regulator_act81461_vbuck1_set_voltage(dev, min_uv, max_uv);
	case ACT81461_SOURCE_BUCK2:
		return regulator_act81461_vbuck2_set_voltage(dev, min_uv, max_uv);
	case ACT81461_SOURCE_BOOST:
		return regulator_act81461_vboost_set_voltage(dev, min_uv, max_uv);
	case ACT81461_SOURCE_LDO1:
		return regulator_act81461_ldo1_set_voltage(dev, min_uv, max_uv);
	case ACT81461_SOURCE_LDO2:
		return regulator_act81461_ldo2_set_voltage(dev, min_uv, max_uv);
	case ACT81461_SOURCE_LDO3:
		return regulator_act81461_ldo3_set_voltage(dev, min_uv, max_uv);
	case ACT81461_LSW4:
	case ACT81461_LSW5:
	case ACT81461_LSW6:
		return -ENOTSUP;
	default:
		LOG_ERR("Unexpected source %d", config->source);
		break;
	}

	return 0;
}

static int regulator_act81461_get_voltage(const struct device *dev, int32_t *volt_uv)
{
	const struct regulator_act81461_node_config *config = dev->config;

	switch (config->source) {
	case ACT81461_SOURCE_BUCKBOOST:
		return regulator_act81461_vbuckboost_get_voltage(dev, volt_uv);
	case ACT81461_SOURCE_BUCK1:
		return regulator_act81461_vbuck1_get_voltage(dev, volt_uv);
	case ACT81461_SOURCE_BUCK2:
		return regulator_act81461_vbuck2_get_voltage(dev, volt_uv);
	case ACT81461_SOURCE_BOOST:
		return regulator_act81461_vboost_get_voltage(dev, volt_uv);
	case ACT81461_SOURCE_LDO1:
		return regulator_act81461_ldo1_get_voltage(dev, volt_uv);
	case ACT81461_SOURCE_LDO2:
		return regulator_act81461_ldo2_get_voltage(dev, volt_uv);
	case ACT81461_SOURCE_LDO3:
		return regulator_act81461_ldo3_get_voltage(dev, volt_uv);
	case ACT81461_LSW4:
	case ACT81461_LSW5:
	case ACT81461_LSW6:
		return -ENOTSUP;
	default:
		LOG_ERR("Unexpected source %d", config->source);
		break;
	}

	return 0;
}

static int regulator_act81461_get_error_flags(const struct device *dev,
					      regulator_error_flags_t *flags)
{
	const struct regulator_act81461_node_config *config = dev->config;
	const struct regulator_act81461_parent_config *cconfig = config->p->config;
	int ret = 0;
	uint8_t val;
	uint8_t sys_errors, errors;

	*flags = 0U;

	if (!i2c_is_ready_dt(&cconfig->i2c)) {
		return -ENODEV;
	}

	dev = config->p;
	if (dev == NULL) {
		LOG_ERR("PMIC device %s is not available", dev->name);
		return -ENODEV;
	}

	/* Check for PMIC Temperature WARN + VSYS low */
	i2c_reg_read_byte_dt(&cconfig->i2c, MASTER_CONF_REG0, &val);
	sys_errors = (val & 0x30);

	if (val & TWARN) {
		*flags |= REGULATOR_ERROR_OVER_TEMP;
	}

	if (val & VSYS) {
		*flags |= REGULATOR_ERROR_VSYS_UNDER_VOLTAGE;
	}

	switch (config->source) {
	case ACT81461_SOURCE_BUCKBOOST:
		/* Check for PWR_GOOD + OV + ILIM */
		ret = i2c_reg_read_byte_dt(&cconfig->i2c, BUCKBOOST_STAT_REG, &val);
		if (ret < 0) {
			return ret;
		}
		errors = (val & VBUCK1_STAT_ERR_MSK);
		LOG_DBG("BUCKBOOST ERROR FLAGS %x %x\n", val, errors);
		if (!(val & PWR_OK)) {
			*flags |= REGULATOR_ERROR_UNDER_VOLTAGE;
		}

		if (val & OV_FLT) {
			*flags |= REGULATOR_ERROR_OVER_VOLTAGE;
		}

		if (val & ILIM_FLT) {
			*flags |= REGULATOR_ERROR_OVER_CURRENT;
		}
		break;

	case ACT81461_SOURCE_BUCK1:
		/* Check for PWR_GOOD + OV + ILIM */
		ret = i2c_reg_read_byte_dt(&cconfig->i2c, VBUCK1_STAT_REG, &val);
		if (ret < 0) {
			return ret;
		}
		errors = (val & VBUCK1_STAT_ERR_MSK);
		LOG_DBG("BUCK1 ERROR FLAGS %x %x\n", val, errors);
		if (!(val & PWR_OK)) {
			*flags |= REGULATOR_ERROR_UNDER_VOLTAGE;
		}

		if (val & OV_FLT) {
			*flags |= REGULATOR_ERROR_OVER_VOLTAGE;
		}

		if (val & ILIM_FLT) {
			*flags |= REGULATOR_ERROR_OVER_CURRENT;
		}
		break;

	case ACT81461_SOURCE_BUCK2:
		/* Check for PWR_GOOD + OV + ILIM */
		ret = i2c_reg_read_byte_dt(&cconfig->i2c, VBUCK2_STAT_REG, &val);
		if (ret < 0) {
			return ret;
		}
		errors = (val & VBUCK2_STAT_ERR_MSK);
		LOG_DBG("BUCK2 ERROR FLAGS %x %x\n", val, errors);
		if (!(val & PWR_OK)) {
			*flags |= REGULATOR_ERROR_UNDER_VOLTAGE;
		}

		if (val & OV_FLT) {
			*flags |= REGULATOR_ERROR_OVER_VOLTAGE;
		}

		if (val & ILIM_FLT) {
			*flags |= REGULATOR_ERROR_OVER_CURRENT;
		}
		break;

	case ACT81461_SOURCE_BOOST:
		/* Check for PWR_GOOD + OV + ILIM */
		ret = i2c_reg_read_byte_dt(&cconfig->i2c, VBOOST_STAT_REG, &val);
		if (ret < 0) {
			return ret;
		}
		errors = (val & VBOOST_STAT_ERR_MSK);
		LOG_DBG("BOOST ERROR FLAGS %x %x\n", val, errors);
		if (val & BST_UV) {
			*flags |= REGULATOR_ERROR_UNDER_VOLTAGE;
		}

		if (val & BST_OV) {
			*flags |= REGULATOR_ERROR_OVER_VOLTAGE;
		}

		if (val & BST_ILIM_WRN) {
			*flags |= REGULATOR_ERROR_OVER_CURRENT;
		}
		break;

	case ACT81461_SOURCE_LDO1:
		/* Check for PWR_GOOD + OV */
		ret = i2c_reg_read_byte_dt(&cconfig->i2c, LDO1_STAT_REG, &val);
		if (ret < 0) {
			return ret;
		}
		errors = (val & LDO1_STAT_ERR_MSK);
		LOG_DBG("LDO1 ERROR FLAGS %x %x\n", val, errors);
		if (!(val & PWR_OK)) {
			*flags |= REGULATOR_ERROR_UNDER_VOLTAGE;
		}

		if (val & OV_FLT) {
			*flags |= REGULATOR_ERROR_OVER_VOLTAGE;
		}

		if (val & ILIM_FLT) {
			*flags |= REGULATOR_ERROR_OVER_CURRENT;
		}
		break;

	case ACT81461_SOURCE_LDO2:
		/* Check for PWR_GOOD + OV */
		ret = i2c_reg_read_byte_dt(&cconfig->i2c, LDO2_STAT_REG, &val);
		if (ret < 0) {
			return ret;
		}
		errors = (val & LDO2_STAT_ERR_MSK);
		LOG_DBG("LDO2 ERROR FLAGS %x %x\n", val, errors);
		if (!(val & PWR_OK)) {
			*flags |= REGULATOR_ERROR_UNDER_VOLTAGE;
		}

		if (val & OV_FLT) {
			*flags |= REGULATOR_ERROR_OVER_VOLTAGE;
		}

		if (val & ILIM_FLT) {
			*flags |= REGULATOR_ERROR_OVER_CURRENT;
		}
		break;

	case ACT81461_SOURCE_LDO3:
		/* Check for PWR_GOOD + OV */
		ret = i2c_reg_read_byte_dt(&cconfig->i2c, LDO3_STAT_REG, &val);
		if (ret < 0) {
			return ret;
		}
		errors = (val & LDO3_STAT_ERR_MSK);
		LOG_DBG("LDO3 ERROR FLAGS %x %x\n", val, errors);
		if (!(val & PWR_OK)) {
			*flags |= REGULATOR_ERROR_UNDER_VOLTAGE;
		}

		if (val & OV_FLT) {
			*flags |= REGULATOR_ERROR_OVER_VOLTAGE;
		}

		if (val & ILIM_FLT) {
			*flags |= REGULATOR_ERROR_OVER_CURRENT;
		}
		break;

	case ACT81461_LSW4:
		/* Check for PWR_GOOD + OV */
		ret = i2c_reg_read_byte_dt(&cconfig->i2c, LSW4_STAT_REG, &val);
		if (ret < 0) {
			return ret;
		}
		errors = (val & LSW4_STAT_ERR_MSK);
		LOG_DBG("LSW4 ERROR FLAGS %x %x\n", val, errors);
		if (!(val & PWR_OK)) {
			*flags |= REGULATOR_ERROR_UNDER_VOLTAGE;
		}

		if (val & OV_FLT) {
			*flags |= REGULATOR_ERROR_OVER_VOLTAGE;
		}

		if (val & ILIM_FLT) {
			*flags |= REGULATOR_ERROR_OVER_CURRENT;
		}
		break;
	case ACT81461_LSW5:
		/* Check for PWR_GOOD + OV */
		ret = i2c_reg_read_byte_dt(&cconfig->i2c, LSW5_STAT_REG, &val);
		if (ret < 0) {
			return ret;
		}
		errors = (val & LSW5_STAT_ERR_MSK);
		LOG_DBG("LSW4 ERROR FLAGS %x %x\n", val, errors);
		if (!(val & PWR_OK)) {
			*flags |= REGULATOR_ERROR_UNDER_VOLTAGE;
		}

		if (val & OV_FLT) {
			*flags |= REGULATOR_ERROR_OVER_VOLTAGE;
		}

		if (val & ILIM_FLT) {
			*flags |= REGULATOR_ERROR_OVER_CURRENT;
		}
		break;
	case ACT81461_LSW6:
		/* Check for PWR_GOOD + OV */
		ret = i2c_reg_read_byte_dt(&cconfig->i2c, LSW6_STAT_REG, &val);
		if (ret < 0) {
			return ret;
		}
		errors = (val & LSW6_STAT_ERR_MSK);
		LOG_DBG("LSW4 ERROR FLAGS %x %x\n", val, errors);
		if (!(val & PWR_OK)) {
			*flags |= REGULATOR_ERROR_UNDER_VOLTAGE;
		}

		if (val & OV_FLT) {
			*flags |= REGULATOR_ERROR_OVER_VOLTAGE;
		}

		if (val & ILIM_FLT) {
			*flags |= REGULATOR_ERROR_OVER_CURRENT;
		}
		break;

	default:
		return ret;
	}

	return ret;
}

static void pmic_intr_callback(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	struct regulator_act81461_common_data *data;
	const struct regulator_act81461_parent_config *config;

	data = CONTAINER_OF(cb, struct regulator_act81461_common_data, gpio_int_cb);
	config = data->dev->config;


	LOG_ERR("pmic system change INT Interrupt");

	gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_DISABLE);
	k_work_submit(&data->int_work);
}

static const struct device *get_child_regulator(const struct device *pdev, uint8_t source)
{
	const struct device *dev;
	size_t devc = z_device_get_all_static(&dev);
	const struct device *dev_end = dev + devc;
	const struct regulator_act81461_node_config *config;

	while (dev < dev_end) {
		if (dev->api == &api) {
			config = dev->config;
			if (config->p == pdev && config->source == source) {
				return dev;
			}
		}
		dev++;
	}
	return NULL;
}

#ifdef CONFIG_ACT81461_ALPC
extern void act81461_battery_charge_intr_ext_callback(const struct device *dev);

static int act81461_child_visitor(const struct device *dev, void *context)
{
	ARG_UNUSED(context);
	/*
	 * We assume all dependents are either regulators or the ALPC, therefore
	 * any dependent that isn't a regulator is assumed to be the ALPC.
	 */
	if (dev->api != &api) {
		act81461_battery_charge_intr_ext_callback(dev);
	}

	return 0;
}
#endif

static void pmic_int_notify(const struct device *dev, regulator_error_flags_t flags)
{
	/* If we are notifying for individual regulators */
	if (dev->api == &api) {
		struct regulator_act81461_data *data = dev->data;

		struct act81461_regulator_fault_cb *cb, *next;

		SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&data->fault_cbs, cb, next, node) {
			if (cb->cb) {
				cb->cb(dev, flags);
			}
		}
	} else {
		struct regulator_act81461_common_data *data = dev->data;

		struct act81461_regulator_fault_cb *cb, *next;

		SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&data->fault_cbs, cb, next, node) {
			if (cb->cb) {
				cb->cb(dev, flags);
			}
		}
	}
}

static void handle_pmic_int(const struct device *dev, uint8_t src)
{
	const struct regulator_act81461_parent_config *config = dev->config;
	const struct device *cdev;
	uint8_t val, msk;
	regulator_error_flags_t flags;
	int ret;

	switch (src) {
	case INT_SRC_MSTR:
		/* Read/Clear interrupts */
		ret = i2c_reg_read_byte_dt(&config->i2c, INT_SRC_MSTR_REG, &val);
		val &= INT_SRC_MSTR_REG_MSK;
		if (ret < 0) {
			break;
		}

		/* AND with mask to get the true interrupt source*/
		ret = i2c_reg_read_byte_dt(&config->i2c, INT_SRC_MSTR_MSK_REG, &msk);
		if (ret < 0) {
			break;
		}
		val &= msk;
		if (val & TWARN) {
			flags |= REGULATOR_ERROR_OVER_TEMP;
		}

		if (val & VSYS) {
			flags |= REGULATOR_ERROR_VSYS_UNDER_VOLTAGE;
		}
		/* We don't handle the rest; clear everything */
		ret = i2c_reg_update_byte_dt(&config->i2c, INT_SRC_MSTR_REG, INT_SRC_MSTR_REG_MSK,
					     0);
		if (flags) {
			pmic_int_notify(dev, flags);
		}
		break;
	case INT_SRC_GPIO:
		/* Just read the interrupt to clear it; GPIO support TODO */
		ret = i2c_reg_read_byte_dt(&config->i2c, INT_SRC_GPIO_REG, &val);
		break;
	case INT_SRC_LDO1:
		/* We can't exactly clear this so...*/
		cdev = get_child_regulator(dev, ACT81461_SOURCE_LDO1);
		if (!cdev) {
			LOG_ERR("Unable for find entry for LDO1");
			return;
		}
		regulator_get_error_flags(cdev, &flags);
		pmic_int_notify(cdev, flags);
		break;
	case INT_SRC_LDO2:
		cdev = get_child_regulator(dev, ACT81461_SOURCE_LDO2);
		if (!cdev) {
			LOG_ERR("Unable for find entry for LDO2");
			return;
		}
		regulator_get_error_flags(cdev, &flags);
		pmic_int_notify(cdev, flags);
		break;
	case INT_SRC_LDO3:
		cdev = get_child_regulator(dev, ACT81461_SOURCE_LDO3);
		if (!cdev) {
			LOG_ERR("Unable for find entry for LDO3");
			return;
		}
		regulator_get_error_flags(cdev, &flags);
		pmic_int_notify(cdev, flags);
		break;
	case INT_SRC_LSW4:
		cdev = get_child_regulator(dev, ACT81461_LSW4);
		if (!cdev) {
			LOG_ERR("Unable for find entry for LSW4");
			return;
		}
		regulator_get_error_flags(cdev, &flags);
		pmic_int_notify(cdev, flags);
		break;
	case INT_SRC_LSW5:
		cdev = get_child_regulator(dev, ACT81461_LSW5);
		if (!cdev) {
			LOG_ERR("Unable for find entry for LSW5");
			return;
		}
		regulator_get_error_flags(cdev, &flags);
		pmic_int_notify(cdev, flags);
		break;
	case INT_SRC_LSW6:
		cdev = get_child_regulator(dev, ACT81461_LSW6);
		if (!cdev) {
			LOG_ERR("Unable for find entry for LSW6");
			return;
		}
		regulator_get_error_flags(cdev, &flags);
		pmic_int_notify(cdev, flags);
		break;
	case INT_SRC_BUCK1:
		cdev = get_child_regulator(dev, ACT81461_SOURCE_BUCK1);
		if (!cdev) {
			LOG_ERR("Unable for find entry for BUCK1");
			return;
		}
		regulator_get_error_flags(cdev, &flags);
		pmic_int_notify(cdev, flags);
		break;
	case INT_SRC_BUCK2:
		cdev = get_child_regulator(dev, ACT81461_SOURCE_BUCK2);
		if (!cdev) {
			LOG_ERR("Unable for find entry for BUCK2");
			return;
		}
		regulator_get_error_flags(cdev, &flags);
		pmic_int_notify(cdev, flags);
		break;
	case INT_SRC_BOOST:
		cdev = get_child_regulator(dev, ACT81461_SOURCE_BOOST);
		if (!cdev) {
			LOG_ERR("Unable for find entry for BOOST");
			return;
		}
		regulator_get_error_flags(cdev, &flags);
		pmic_int_notify(cdev, flags);
		break;
	case INT_SRC_BB:
		cdev = get_child_regulator(dev, ACT81461_SOURCE_BUCKBOOST);
		if (!cdev) {
			LOG_ERR("Unable for find entry for BUCKBOOST");
			return;
		}
		regulator_get_error_flags(cdev, &flags);
		pmic_int_notify(cdev, flags);
		break;
	case INT_SRC_ALPC:
		/* Read/Clear interrupts */
		ret = i2c_reg_read_byte_dt(&config->i2c, INT_SRC_ALPC_REG, &val);
		if (ret < 0) {
			break;
		}

		if (val & VBAT_LOW_STAT) {
			flags |= REGULATOR_ERROR_VBAT_LOW;
		}

		/* If available, Notify ALPC driver if any other interrupts have been flagged */
#ifdef CONFIG_ACT81461_ALPC
		if (val) {
			/* Find ALPC and Notify it */
			device_supported_foreach(dev, act81461_child_visitor, NULL);
		}
#endif
		pmic_int_notify(dev, flags);
		break;
	default:
		break;
	}
}

static void int_work_fn(struct k_work *item)
{
	struct regulator_act81461_common_data *data =
		CONTAINER_OF(item, struct regulator_act81461_common_data, int_work);
	const struct device *dev = data->dev;
	const struct regulator_act81461_parent_config *config = dev->config;
	uint8_t src = 0;

	do {
		/* Process interrupt related */
		int ret = i2c_reg_read_byte_dt(&config->i2c, INT_SRC_REG, &src);

		if (ret < 0) {
			LOG_ERR("Failed to read interrupt source, err = %d", ret);
			return;
		}

		/* Handle Unknown/Multi-source */
		if (src == INT_SRC_UNKN) {
			uint8_t srcs[] = {INT_SRC_MSTR,	 INT_SRC_GPIO,	INT_SRC_LDO1,  INT_SRC_LDO2,
					INT_SRC_LDO3,	 INT_SRC_LSW4,	INT_SRC_LSW5,  INT_SRC_LSW6,
					INT_SRC_BUCK1, INT_SRC_BUCK2, INT_SRC_BOOST, INT_SRC_ALPC};
			for (int i = 0; i < ARRAY_SIZE(srcs); i++) {
				handle_pmic_int(dev, srcs[i]);
			}
		} else {
			handle_pmic_int(dev, src);
		}

	} while (gpio_pin_get_dt(&config->int_gpio));

	gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}

int act81461_fault_int_msk_set(const struct device *dev, regulator_error_flags_t flags, bool mask)
{
	const struct regulator_act81461_parent_config *cconfig;
	uint8_t val = mask ? ~0 : 0;
	int ret = 0;

	if (dev->api == &api) {
		const struct regulator_act81461_node_config *config = dev->config;
		uint8_t reg;

		cconfig = config->p->config;
		switch (config->source) {
		case ACT81461_SOURCE_LDO1:
		case ACT81461_SOURCE_LDO2:
		case ACT81461_SOURCE_LDO3:
		case ACT81461_LSW4:
		case ACT81461_LSW5:
		case ACT81461_LSW6:
			switch (config->source) {
			case ACT81461_SOURCE_LDO1:
				reg = INT_MSK_REG_LDO1;
				break;
			case ACT81461_SOURCE_LDO2:
				reg = INT_MSK_REG_LDO2;
				break;
			case ACT81461_SOURCE_LDO3:
				reg = INT_MSK_REG_LDO3;
				break;
			case ACT81461_LSW4:
				reg = INT_MSK_REG_LSW4;
				break;
			case ACT81461_LSW5:
				reg = INT_MSK_REG_LSW5;
				break;
			case ACT81461_LSW6:
				reg = INT_MSK_REG_LSW6;
				break;
			}
			if (flags & ~(REGULATOR_ERROR_OVER_VOLTAGE | REGULATOR_ERROR_UNDER_VOLTAGE |
				      REGULATOR_ERROR_OVER_CURRENT)) {
				return -EINVAL;
			}
			{
				uint8_t msk = 0;

				if (flags & REGULATOR_ERROR_OVER_VOLTAGE) {
					msk |= OV_INT_MSK;
				}
				if (flags & REGULATOR_ERROR_UNDER_VOLTAGE) {
					msk |= PWR_GOOD_INT_MSK;
				}
				if (flags & REGULATOR_ERROR_OVER_CURRENT) {
					msk |= ILIM_INT_MSK;
				}
				ret = i2c_reg_update_byte_dt(&cconfig->i2c, reg, msk, val);
			}
			break;
		case ACT81461_SOURCE_BUCK1:
		case ACT81461_SOURCE_BUCK2:
		case ACT81461_SOURCE_BUCKBOOST:
			switch (config->source) {
			case ACT81461_SOURCE_BUCK1:
				reg = INT_MSK_REG_BUCK1;
				break;
			case ACT81461_SOURCE_BUCK2:
				reg = INT_MSK_REG_BUCK2;
				break;
			case ACT81461_SOURCE_BUCKBOOST:
				reg = INT_MSK_REG_BUCKBOOST;
				break;
			}
			if (flags & ~REGULATOR_ERROR_OVER_CURRENT) {
				return -EINVAL;
			}
			{
				uint8_t msk = 0;

				if (flags & REGULATOR_ERROR_OVER_CURRENT) {
					msk |= ILIM_INT_MSK;
				}
				ret = i2c_reg_update_byte_dt(&cconfig->i2c, reg, msk, val);
			}
			break;
		case ACT81461_SOURCE_BOOST:
			reg = INT_MSK_REG_BOOST;
			if (flags & ~(REGULATOR_ERROR_OVER_VOLTAGE | REGULATOR_ERROR_UNDER_VOLTAGE |
				      REGULATOR_ERROR_OVER_CURRENT)) {
				return -EINVAL;
			}
			uint8_t msk = 0;

			if (flags & REGULATOR_ERROR_OVER_VOLTAGE) {
				msk |= BST_OV_INT_MSK;
			}
			if (flags & REGULATOR_ERROR_UNDER_VOLTAGE) {
				msk |= BST_UV_INT_MSK;
			}
			if (flags & REGULATOR_ERROR_OVER_CURRENT) {
				msk |= BST_ILIM_INT_MSK;
			}
			ret = i2c_reg_update_byte_dt(&cconfig->i2c, reg, msk, val);
			break;
		default:
			return -EINVAL;
		}
	} else {
		cconfig = dev->config;
		if (flags & ~(REGULATOR_ERROR_OVER_TEMP | REGULATOR_ERROR_VSYS_UNDER_VOLTAGE |
			      REGULATOR_ERROR_VBAT_LOW)) {
			return -EINVAL;
		}
		uint8_t msk = 0;

		if (flags & REGULATOR_ERROR_OVER_TEMP) {
			msk |= TWARN;
		}
		if (flags & REGULATOR_ERROR_VSYS_UNDER_VOLTAGE) {
			msk |= VSYS;
		}
		ret = i2c_reg_update_byte_dt(&cconfig->i2c, INT_MSK_REG_MSTR, msk, val);
		if (ret) {
			return ret;
		}
		if (flags & REGULATOR_ERROR_VBAT_LOW) {
			ret = i2c_reg_update_byte_dt(&cconfig->i2c, INT_MSK_REG_ALPC_0,
								VBAT_LOW_MSK, val);
			if (ret) {
				return ret;
			}
			ret = i2c_reg_update_byte_dt(&cconfig->i2c, INT_MSK_REG_ALPC_1,
								VBAT_LOW_MSK, val);
		}
	}
	return ret;
}

int act81461_fault_int_cb_register(const struct device *dev, struct act81461_regulator_fault_cb *cb)
{
	if (cb == NULL) {
		return -EINVAL;
	}

	if (dev->api == &api) {
		struct regulator_act81461_data *data = dev->data;

		sys_slist_append(&data->fault_cbs, &cb->node);
	} else {
		struct regulator_act81461_common_data *data = dev->data;

		sys_slist_append(&data->fault_cbs, &cb->node);
	}
	return 0;
}
int act81461_fault_int_cb_unregister(const struct device *dev,
				     struct act81461_regulator_fault_cb *cb)
{
	if (cb == NULL) {
		return -EINVAL;
	}

	if (dev->api == &api) {
		struct regulator_act81461_data *data = dev->data;

		if (!sys_slist_find_and_remove(&data->fault_cbs, &cb->node)) {
			return -EALREADY;
		}
	} else {
		struct regulator_act81461_common_data *data = dev->data;

		if (!sys_slist_find_and_remove(&data->fault_cbs, &cb->node)) {
			return -EALREADY;
		}
	}
	return 0;
}

int act81461_set_vsys_low_voltage(const struct device *dev, int32_t min_uv, int32_t max_uv)
{
	const struct regulator_act81461_parent_config *cconfig;
	uint16_t idx;
	int ret;

	ret = linear_range_get_win_index(&vsysmon_range, min_uv, max_uv, &idx);
	if (ret == -EINVAL) {
		return ret;
	}

	if (dev->api == &api) {
		const struct regulator_act81461_node_config *config = dev->config;

		cconfig = config->p->config;

		ret = i2c_reg_update_byte_dt(&cconfig->i2c, VSYSMON_REG, VSYSMON_MSK, idx);
	} else {
		cconfig = dev->config;

		ret = i2c_reg_update_byte_dt(&cconfig->i2c, VSYSMON_REG, VSYSMON_MSK, idx);
	}
	return ret;
}

static int pmic_interrupts_configure(const struct device *dev)
{
	const struct regulator_act81461_node_config *config = dev->config;
	const struct regulator_act81461_parent_config *cconfig = config->p->config;
	struct regulator_act81461_common_data *pdata = config->p->data;
	int ret = 0;

	/* Setup and configure the interrupt for PMIC_INT */
	if (!gpio_is_ready_dt(&cconfig->int_gpio)) {
		LOG_ERR("gpio controller %s not ready", cconfig->int_gpio.port->name);
		return -ENODEV;
	}

	struct gpio_callback *int_cb = &pdata->gpio_int_cb;

	gpio_pin_configure_dt(&cconfig->int_gpio, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&cconfig->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(int_cb, pmic_intr_callback, cconfig->int_gpio.pin);
	k_work_init(&pdata->int_work, int_work_fn);
	ret = gpio_add_callback(cconfig->int_gpio.port, int_cb);

	if (ret) {
		LOG_ERR("Cannot setup PMIC INT callback, err %d", ret);
	}
	return 0;
}

static int regulator_act81461_init(const struct device *dev)
{
	const struct regulator_act81461_node_config *config = dev->config;
	const struct regulator_act81461_parent_config *cconfig = config->p->config;
	struct regulator_act81461_common_data *pdata = config->p->data;
	uint8_t pmic_override_reg_array[16];
	uint8_t pmic_override_reg_value_array[16];
	uint8_t pmic_override_num;
	uint8_t val;
	int ret = 0;

	regulator_common_data_init(dev);

	LOG_INF("Init act81461 %s", dev->name);

	/* Check to verify we have a valid I2C device */
	if (!i2c_is_ready_dt(&cconfig->i2c)) {
		LOG_ERR("Device %s is not ready", cconfig->i2c.bus->name);
		return -ENODEV;
	}

	if (!pdata->reg_inited) {
		pdata->dev = config->p;
		pmic_interrupts_configure(dev);

		pmic_override_num = cconfig->pmic_override_num;
		if (pmic_override_num > 0) {
			for (int i = 0; i < pmic_override_num; i++) {
				pmic_override_reg_array[i] = cconfig->pmic_override_reg_array[i];
				pmic_override_reg_value_array[i] =
					cconfig->pmic_override_reg_value_array[i];

				ret = i2c_reg_write_byte_dt(&cconfig->i2c,
							    pmic_override_reg_array[i],
							    pmic_override_reg_value_array[i]);

				if (ret < 0) {
					return ret;
				}

				ret = i2c_reg_read_byte_dt(&cconfig->i2c,
							   pmic_override_reg_array[i], &val);

				if (ret < 0) {
					return ret;
				}
			}
			pdata->reg_inited = true;
		}
	}

	return regulator_common_init(dev, false);
}

static int regulator_act81461_common_init(const struct device *dev)
{
	int ret = 0;
	const struct regulator_act81461_parent_config *config = dev->config;
	struct regulator_act81461_common_data *data = dev->data;

	data->dev = dev;
	sys_slist_init(&data->fault_cbs);
	/* Any special PMIC fix up can be done here at the end of init if needed */
	if (!i2c_is_ready_dt(&config->i2c)) {
		LOG_ERR("Device %s not ready", config->i2c.bus->name);
		return -ENODEV;
	}

	/* We want to mask VBAT_LOW interrupt by default as it doesn't clear on read */
	ret = i2c_reg_update_byte_dt(&config->i2c, INT_MSK_REG_ALPC_0, VBAT_LOW_MSK, ~0);
	if (ret) {
		return ret;
	}
	ret = i2c_reg_update_byte_dt(&config->i2c, INT_MSK_REG_ALPC_1, VBAT_LOW_MSK, ~0);

	/* If interrupt is already active, submit it to be processed */
	if (gpio_pin_get_dt(&config->int_gpio)) {
		int_work_fn(&data->int_work);
	}


	return ret;
}

#define ACT81461_PARENT_INIT_PRIORITY CONFIG_REGULATOR_ACT81461_INIT_PRIORITY
#define ACT81461_CHILD_INIT_PRIORITY  CONFIG_REGULATOR_ACT81461_COMMON_INIT_PRIORITY

BUILD_ASSERT(ACT81461_CHILD_INIT_PRIORITY < ACT81461_PARENT_INIT_PRIORITY,
	     "Child init priority must be less than parent");

#define REGULATOR_ACT81461_DEFINE(node_id, id, _source, parent)                                    \
	static struct regulator_act81461_data data_##inst_##id;                                    \
                                                                                                   \
	static const struct regulator_act81461_node_config config_##id = {                         \
		.common = REGULATOR_DT_COMMON_CONFIG_INIT(node_id),                                \
		.p = parent,                                                                       \
		.source = _source,                                                                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_DEFINE(node_id, regulator_act81461_init, NULL, &data_##inst_##id, &config_##id,  \
			 POST_KERNEL, ACT81461_CHILD_INIT_PRIORITY, &api);

#define REGULATOR_ACT81461_DEFINE_COND(inst, child, source, parent)                                \
	COND_CODE_1(DT_NODE_EXISTS(DT_INST_CHILD(inst, child)),                                    \
		    (REGULATOR_ACT81461_DEFINE(DT_INST_CHILD(inst, child), child##inst, source,    \
					       parent)),                                           \
		    ())

#define REGULATOR_ACT81461_DEFINE_ALL(inst)                                                        \
	static const struct regulator_act81461_parent_config config_##inst = {                     \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),                        \
		.pmic_override_reg_array = DT_INST_PROP(0, pmic_override_reg_array),               \
		.pmic_override_reg_value_array = DT_INST_PROP(0, pmic_override_reg_value_array),   \
		.pmic_override_num = DT_INST_PROP(0, pmic_override_num),                           \
	};                                                                                         \
	static struct regulator_act81461_common_data data_##inst;                                  \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, regulator_act81461_common_init, NULL, &data_##inst,            \
			      &config_##inst, POST_KERNEL, ACT81461_PARENT_INIT_PRIORITY, NULL);   \
                                                                                                   \
	REGULATOR_ACT81461_DEFINE_COND(inst, buckboost, ACT81461_SOURCE_BUCKBOOST,                 \
				       DEVICE_DT_INST_GET(inst))                                   \
	REGULATOR_ACT81461_DEFINE_COND(inst, buck1, ACT81461_SOURCE_BUCK1,                         \
				       DEVICE_DT_INST_GET(inst))                                   \
	REGULATOR_ACT81461_DEFINE_COND(inst, buck2, ACT81461_SOURCE_BUCK1,                         \
				       DEVICE_DT_INST_GET(inst))                                   \
	REGULATOR_ACT81461_DEFINE_COND(inst, boost, ACT81461_SOURCE_BOOST,                         \
				       DEVICE_DT_INST_GET(inst))                                   \
	REGULATOR_ACT81461_DEFINE_COND(inst, ldo1, ACT81461_SOURCE_LDO1, DEVICE_DT_INST_GET(inst)) \
	REGULATOR_ACT81461_DEFINE_COND(inst, ldo2, ACT81461_SOURCE_LDO2, DEVICE_DT_INST_GET(inst)) \
	REGULATOR_ACT81461_DEFINE_COND(inst, ldo3, ACT81461_SOURCE_LDO3, DEVICE_DT_INST_GET(inst)) \
	REGULATOR_ACT81461_DEFINE_COND(inst, lsw4, ACT81461_LSW4, DEVICE_DT_INST_GET(inst))        \
	REGULATOR_ACT81461_DEFINE_COND(inst, lsw5, ACT81461_LSW5, DEVICE_DT_INST_GET(inst))        \
	REGULATOR_ACT81461_DEFINE_COND(inst, lsw6, ACT81461_LSW6, DEVICE_DT_INST_GET(inst))

DT_INST_FOREACH_STATUS_OKAY(REGULATOR_ACT81461_DEFINE_ALL)
