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
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/linear_range.h>
#include <zephyr/sys/util.h>
#include <errno.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
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
	const struct device *parent;
	uint8_t source;
};

struct regulator_act81461_data {
	struct regulator_common_data data;
};

struct regulator_act81461_common_data {
	struct gpio_callback gpio_int_cb;
	bool reg_inited;
};

static const struct linear_range vbuckboost_range = LINEAR_RANGE_INIT(3200000, 50000, 0x0, 0x36);

static const struct linear_range vbuck1_range = LINEAR_RANGE_INIT(600000, 50000, 0x0, 0x3C);

static const struct linear_range vbuck2_range = LINEAR_RANGE_INIT(600000, 50000, 0x0, 0x3C);

static const struct linear_range vboost_range = LINEAR_RANGE_INIT(5000000, 250000, 0x0, 0x3F);

static const struct linear_range ldo1_range = LINEAR_RANGE_INIT(600000, 50000, 0x0, 0x3C);

static const struct linear_range ldo2_range = LINEAR_RANGE_INIT(600000, 50000, 0x0, 0x3C);

static const struct linear_range ldo3_range = LINEAR_RANGE_INIT(600000, 50000, 0x0, 0x3C);

static int regulator_act81461_enable(const struct device *dev)
{
	const struct regulator_act81461_node_config *config = dev->config;
	const struct regulator_act81461_parent_config *cconfig = config->p->config;
	int ret = 0;
	uint8_t val;

	if (!i2c_is_ready_dt(&cconfig->i2c)) {
		return -ENODEV;
	}

	switch (config->source) {
	case ACT81461_SOURCE_BUCKBOOST:

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, BUCKBOOST_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_update_byte_dt(&cconfig->i2c, BUCKBOOST_ENABLE_REG,
					     BUCKBOOST_ENABLE_MSK, BUCKBOOST_ENABLE);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, BUCKBOOST_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		break;

	case ACT81461_SOURCE_BUCK1:

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, BUCK1_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_update_byte_dt(&cconfig->i2c, BUCK1_ENABLE_REG, BUCK1_ENABLE_MSK,
					     BUCK1_ENABLE);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, BUCK1_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		break;

	case ACT81461_SOURCE_BUCK2:

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, BUCK2_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_update_byte_dt(&cconfig->i2c, BUCK2_ENABLE_REG, BUCK2_ENABLE_MSK,
					     BUCK2_ENABLE);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, BUCK2_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		break;

	case ACT81461_SOURCE_BOOST:

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, BOOST_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_update_byte_dt(&cconfig->i2c, BOOST_ENABLE_REG, BOOST_ENABLE_MSK,
					     0x80U);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, BOOST_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		break;

	case ACT81461_SOURCE_LDO1:

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, LDO1_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_update_byte_dt(&cconfig->i2c, LDO1_ENABLE_REG, LDO1_ENABLE_MSK,
					     LDO1_ENABLE);

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, LDO1_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		break;

	case ACT81461_SOURCE_LDO2:

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, LDO2_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_update_byte_dt(&cconfig->i2c, LDO2_ENABLE_REG, LDO2_ENABLE_MSK,
					     LDO2_ENABLE);

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, LDO2_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		break;

	case ACT81461_SOURCE_LDO3:

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, LDO3_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_update_byte_dt(&cconfig->i2c, LDO3_ENABLE_REG, LDO1_ENABLE_MSK,
					     LDO3_ENABLE);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, LDO3_ENABLE_REG, &val);
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
	uint8_t val;

	switch (config->source) {
	case ACT81461_SOURCE_BUCKBOOST:

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, BUCKBOOST_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_update_byte_dt(&cconfig->i2c, BUCKBOOST_ENABLE_REG,
					     BUCKBOOST_ENABLE_MSK, 0U);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, BUCKBOOST_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		break;

	case ACT81461_SOURCE_BUCK1:

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, BUCK1_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_update_byte_dt(&cconfig->i2c, BUCK1_ENABLE_REG, BUCK1_ENABLE_MSK, 0U);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, BUCK1_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		break;

	case ACT81461_SOURCE_BUCK2:

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, BUCK2_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_update_byte_dt(&cconfig->i2c, BUCK2_ENABLE_REG, BUCK2_ENABLE_MSK, 0U);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, BUCK2_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		break;

	case ACT81461_SOURCE_BOOST:

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, BOOST_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_update_byte_dt(&cconfig->i2c, BOOST_ENABLE_REG, BOOST_ENABLE_MSK, 0U);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, BOOST_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		break;

	case ACT81461_SOURCE_LDO1:

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, LDO1_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_update_byte_dt(&cconfig->i2c, LDO1_ENABLE_REG, LDO1_ENABLE_MSK,
					     0x00U);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, LDO1_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		break;

	case ACT81461_SOURCE_LDO2:

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, LDO2_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_update_byte_dt(&cconfig->i2c, LDO2_ENABLE_REG, LDO2_ENABLE_MSK,
					     0x00U);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, LDO2_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		break;

	case ACT81461_SOURCE_LDO3:

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, LDO3_ENABLE_REG, &val);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_update_byte_dt(&cconfig->i2c, LDO3_ENABLE_REG, LDO3_ENABLE_MSK,
					     0x00U);
		if (ret < 0) {
			return ret;
		}

		ret = i2c_reg_read_byte_dt(&cconfig->i2c, LDO3_ENABLE_REG, &val);
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
	default:
		return ret;
	}

	return ret;
}

static void pmic_intr_callback(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	LOG_DBG("pmic system change INT Interrupt");
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
	gpio_pin_interrupt_configure_dt(&cconfig->int_gpio, GPIO_INT_EDGE_FALLING);
	gpio_init_callback(int_cb, pmic_intr_callback, cconfig->int_gpio.pin);
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
	/* Any special PMIC fix up can be done here at the end of init if needed */
	if (!i2c_is_ready_dt(&config->i2c)) {
		LOG_ERR("Device %s not ready", config->i2c.bus->name);
		return -ENODEV;
	}

	return ret;
}

static const struct regulator_driver_api api = {
	.enable = regulator_act81461_enable,
	.disable = regulator_act81461_disable,
	.count_voltages = regulator_act81461_count_voltages,
	.list_voltage = regulator_act81461_list_voltage,
	.set_voltage = regulator_act81461_set_voltage,
	.get_voltage = regulator_act81461_get_voltage,
	.get_error_flags = regulator_act81461_get_error_flags,
};

#define ACT81461_PARENT_INIT_PRIORITY CONFIG_REGULATOR_ACT81461_INIT_PRIORITY
#define ACT81461_CHILD_INIT_PRIORITY  CONFIG_REGULATOR_ACT81461_COMMON_INIT_PRIORITY

BUILD_ASSERT(ACT81461_CHILD_INIT_PRIORITY < ACT81461_PARENT_INIT_PRIORITY,
	     "Child init priority must be less than parent");

#define REGULATOR_ACT81461_DEFINE(node_id, id, _source, parent)                                    \
	static struct regulator_act81461_data data_##inst_##id;		\
                                                                                                   \
	static const struct regulator_act81461_node_config config_##id = {                         \
		.common = REGULATOR_DT_COMMON_CONFIG_INIT(node_id),                                \
		.p = parent,                                                                       \
		.source = _source,                                                                 \
	};                                                                                         \
                                                                                    \
	DEVICE_DT_DEFINE(node_id, regulator_act81461_init, NULL, &data_##inst_##id,		\
			 &config_##id, POST_KERNEL, ACT81461_CHILD_INIT_PRIORITY, &api);

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
	};	\
	static struct regulator_act81461_common_data data_##inst;		\
                                                                                            \
	DEVICE_DT_INST_DEFINE(inst, regulator_act81461_common_init, NULL, &data_##inst,       \
						  &config_##inst, POST_KERNEL,		\
						  ACT81461_PARENT_INIT_PRIORITY, NULL);		\
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
	REGULATOR_ACT81461_DEFINE_COND(inst, ldo3, ACT81461_SOURCE_LDO3, DEVICE_DT_INST_GET(inst))

DT_INST_FOREACH_STATUS_OKAY(REGULATOR_ACT81461_DEFINE_ALL)
