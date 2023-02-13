/*
 * Copyright (c) 2022 T-Mobile USA, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PMIC Regulator Driver
 * This driver implements the regulator API within Zephyr, and also
 * implements support for a broader API.
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

/* nPM6001 voltage sources */
enum act81461_sources {
	ACT81461_SOURCE_VBUCKBOOST_3V3,
	ACT81461_SOURCE_VBUCK1_1V85,
	ACT81461_SOURCE_LDO1_2V20,
	ACT81461_SOURCE_LDO1_1V85,
	ACT81461_SOURCE_LDO3_1V85,
};

#define PMIC_CHARGE_STATUS 2 /* PA2	 */
#define PMIC_RESET	  9		 /* PF9	 */
#define PMIC_WAKE	 11		 /* PF11 */
#define PMIC_INT	 12		 /* PF12 */

#define PMIC_ADDRESS 0x25
#define GPIO_A_NAME DT_NODE_FULL_NAME(DT_NODELABEL(gpioa))
#define GPIO_F_NAME DT_NODE_FULL_NAME(DT_NODELABEL(gpiof))

struct drv_data {
	struct gpio_callback gpio_int_cb;
	gpio_flags_t mode;
	int index;
	int aux;
};

/* PMIC Battery and Charger Status */
static const struct device *gpioa_dev;
/* PMIC Interrupt for master status */
static const struct device *gpiof_dev;

/* GNSS GPIO pins for doing initiialization after power up.*/
static const struct device *gpio_dev;
static int ret = 0;

/* PMIC INT gpio interrupt callback */
static struct gpio_callback gpio_cb;
int pmic_int_isr_count = 0;

/* PMIC Battery Charger interrupt callback */
static struct gpio_callback gpio_batt_chrg_cb;
int pmic_battery_charge_int_isr_count = 0;
bool pmic_battery_charge_int_state_change = false;

const struct device *i2c1_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
uint8_t i2c_address = 0x25;

LOG_MODULE_REGISTER(pmic_regulator_driver, CONFIG_REGULATOR_LOG_LEVEL);

struct __packed voltage_range {
	int uV; /* Voltage in uV */
	int reg_val; /* Register value for voltage */
};

struct __packed current_range {
	int uA; /* Current limit in uA */
	int reg_val; /* Register value for current limit */
};

uint8_t pmic_init_reg_array[6];
uint8_t pmic_init_reg_value_array[6];

struct regulator_act81461_pconfig {
	struct i2c_dt_spec i2c;
};

struct regulator_act81461_config {
	struct regulator_common_config common;
	const struct device *p;
	uint8_t source;
	struct i2c_dt_spec i2c;
	int32_t regulator_min_microvolt;
	int32_t regulator_max_microvolt;
	uint8_t pmic_init_reg_array[7];
	uint8_t pmic_init_reg_value_array[7];
	uint8_t pmic_init_reg_array_num;
};

struct regulator_act81461_data {
	struct regulator_common_data data;
};

struct regulator_act81461_vmap {
	uint8_t reg_val;
	int32_t volt_uv;
};

static const struct linear_range vbuckboost_range =
	LINEAR_RANGE_INIT(3300000, 50000U, 0x0U, 0x6U);

static const struct linear_range vbuck1_range =
	LINEAR_RANGE_INIT(1850000, 50000U, 0x0U, 0x1AU);

static const struct linear_range ldo1_1v85_range =
	LINEAR_RANGE_INIT(1850000, 50000U, 0x0U, 0x1A);

static const struct linear_range ldo1_2v2_range =
	LINEAR_RANGE_INIT(2200000, 50000U, 0x0U, 0x22);

static const struct linear_range ldo3_1v85_range =
	LINEAR_RANGE_INIT(1850000, 50000U, 0x0U, 0x1A);

static void pmic_intr_callback(const struct device *port,
	struct gpio_callback *cb, uint32_t pins)
{
	pmic_int_isr_count++;
}

static void pmic_battery_charge_intr_callback(const struct device *port,
	struct gpio_callback *cb, uint32_t pins)
{
	pmic_battery_charge_int_isr_count++;
	pmic_battery_charge_int_state_change = true;
}

#define VBUCKBOOST_ENABLE_REG 0x94U
#define VBUCKBOOST_ENABLE_MSK 0x80U

#define VBUCK1_ENABLE_REG 0x64u
#define VBUCK1_ENABLE_MSK 0x80U

#define LDO1_ENABLE_REG 0x30U
#define LDO1_ENABLE_MSK 0x80U

#define LDO3_ENABLE_REG 0x40U
#define LDO3_ENABLE_MSK 0x80U

#define LDO1_VSET 0x31 /* 0.6V + (VSET * 50mv) */
#define LDO1_VSET_MSK 0x3f

#define LDO3_VSET 0x41 /* 0.6V + (VSET * 50mv) */
#define LDO3_VSET_MSK 0x3f

#define VBUCK1_VSET 0x62 /* 0.6V + (VSET * 50mv) */
#define VBUCK1_VSET_MSK 0x3f

#define VBUCKBOOST_VSET 0x92 /* 3.2V + (VSET * 50mv) */
#define VBUCKBOOST_VSET_MSK 0x3f

static int regulator_act81461_enable(const struct device *dev)
{
	const struct regulator_act81461_config *config = dev->config;

	switch (config->source) {
	case ACT81461_SOURCE_VBUCKBOOST_3V3:
		return i2c_reg_update_byte_dt(
						&config->i2c, VBUCKBOOST_ENABLE_REG,
						VBUCKBOOST_ENABLE_MSK, 0x80U);
	case ACT81461_SOURCE_VBUCK1_1V85:
		return i2c_reg_update_byte_dt(
						&config->i2c, VBUCK1_ENABLE_REG,
						VBUCK1_ENABLE_MSK, 0x80U);
	case ACT81461_SOURCE_LDO1_2V20:
	case ACT81461_SOURCE_LDO1_1V85:
		return i2c_reg_update_byte_dt(
						&config->i2c,
						LDO1_ENABLE_REG, LDO1_ENABLE_MSK, 0x80U);
	case ACT81461_SOURCE_LDO3_1V85:
		return i2c_reg_update_byte_dt(
						&config->i2c,
						LDO3_ENABLE_REG, LDO3_ENABLE_MSK, 0x80U);
		default:
				return 0;
		}
}

static int regulator_act81461_disable(const struct device *dev)
{
		const struct regulator_act81461_config *config = dev->config;

	switch (config->source) {
	case ACT81461_SOURCE_VBUCKBOOST_3V3:
		return i2c_reg_update_byte_dt(
						&config->i2c, VBUCKBOOST_ENABLE_REG,
						VBUCKBOOST_ENABLE_MSK, 0U);
	case ACT81461_SOURCE_VBUCK1_1V85:
		return i2c_reg_update_byte_dt(
						&config->i2c, VBUCK1_ENABLE_REG,
						VBUCK1_ENABLE_MSK, 0U);
	case ACT81461_SOURCE_LDO1_2V20:
	case ACT81461_SOURCE_LDO1_1V85:
		return i2c_reg_update_byte_dt(
			&config->i2c,
			LDO1_ENABLE_REG, LDO1_ENABLE_MSK, 0x00U);
	case ACT81461_SOURCE_LDO3_1V85:
		return i2c_reg_update_byte_dt(
			&config->i2c,
			LDO3_ENABLE_REG, LDO3_ENABLE_MSK, 0x00U);
		default:
				return 0;
		}
}

static unsigned int regulator_act81461_count_voltages(const struct device *dev)
{
		const struct regulator_act81461_config *config = dev->config;

		switch (config->source) {
		case ACT81461_SOURCE_VBUCKBOOST_3V3:
				return linear_range_values_count(&vbuckboost_range);
		case ACT81461_SOURCE_VBUCK1_1V85:
				return linear_range_values_count(&vbuck1_range);
		case ACT81461_SOURCE_LDO1_1V85:
				return linear_range_values_count(&ldo1_1v85_range);
		case ACT81461_SOURCE_LDO1_2V20:
				return linear_range_values_count(&ldo1_2v2_range);
		case ACT81461_SOURCE_LDO3_1V85:
				return linear_range_values_count(&ldo3_1v85_range);
		default:
				__ASSERT(NULL, "Unexpected source");
		}

		return 0;
}

static int regulator_act81461_list_voltage(const struct device *dev,
										  unsigned int idx, int32_t *volt_uv)
{
	const struct regulator_act81461_config *config = dev->config;

	switch (config->source) {
		case ACT81461_SOURCE_VBUCKBOOST_3V3:
			*volt_uv = 3300000;
		case ACT81461_SOURCE_VBUCK1_1V85:
			*volt_uv = 1850000;
		case ACT81461_SOURCE_LDO1_1V85:
			*volt_uv = 1850000;
		case ACT81461_SOURCE_LDO1_2V20:
				*volt_uv = 2200000;
		case ACT81461_SOURCE_LDO3_1V85:
			*volt_uv = 1850000;
		break;
		default:
			_ASSERT(NULL, "Unexpected source");
	}

	return 0;
}

static int regulator_act81461_vbuckboost_set_voltage(const struct device *dev,
											   int32_t min_uv, int32_t max_uv)
{
		const struct regulator_act81461_config *config = dev->config;
		uint16_t idx;
		int ret;

		ret = linear_range_get_win_index(&vbuckboost_range, min_uv, max_uv, &idx);
		if (ret == -EINVAL) {
				return ret;
		}

		ret = i2c_reg_update_byte_dt(
						&config->i2c, VBUCKBOOST_VSET,
						VBUCKBOOST_VSET_MSK, (uint8_t)idx);
		if (ret < 0) {
				return ret;
		}

		return 0;
}

static int regulator_act81461_vbuck1_set_voltage(const struct device *dev,
											   int32_t min_uv, int32_t max_uv)
{
		const struct regulator_act81461_config *config = dev->config;
		uint16_t idx;
		int ret;

		ret = linear_range_get_win_index(&vbuck1_range, min_uv, max_uv, &idx);
		if (ret == -EINVAL) {
				return ret;
		}

	ret = i2c_reg_update_byte_dt(
						&config->i2c, VBUCK1_VSET,
						VBUCK1_VSET_MSK, (uint8_t)idx);

	if (ret < 0) {
				return ret;
		}

		return 0;
}

static int regulator_act81461_ldo1_set_voltage(const struct device *dev,
											  int32_t min_uv, int32_t max_uv)
{
		const struct regulator_act81461_config *config = dev->config;
		uint16_t idx;

	ret = linear_range_get_win_index(&ldo1_1v85_range, min_uv, max_uv, &idx);
		if (ret == -EINVAL) {
				return ret;
		}

	ret = i2c_reg_update_byte_dt(&config->i2c, LDO1_VSET,
									 LDO1_VSET_MSK, (uint8_t)idx);

		if (ret < 0) {
				return ret;
		}
}

static int regulator_act81461_ldo3_set_voltage(const struct device *dev,
											  int32_t min_uv, int32_t max_uv)
{
		const struct regulator_act81461_config *config = dev->config;
		uint16_t idx;

	ret = linear_range_get_win_index(&ldo3_1v85_range, min_uv, max_uv, &idx);
		if (ret == -EINVAL) {
				return ret;
		}

	ret = i2c_reg_update_byte_dt(&config->i2c, LDO3_VSET,
									 LDO3_VSET_MSK, (uint8_t)idx);
		if (ret < 0) {
				return ret;
		}
}

static int regulator_act81461_ldo1_get_voltage(const struct device *dev,
											   int32_t *volt_uv)
{
		const struct regulator_act81461_config *config = dev->config;
		uint8_t val;
		int ret;

		ret = i2c_reg_read_byte_dt(&config->i2c, LDO1_VSET, &val);
		if (ret < 0) {
				return ret;
		}

	val &= LDO1_VSET_MSK;
		*volt_uv = 600000 + (val * 50000);
	return 0;
}

static int regulator_act81461_ldo3_get_voltage(const struct device *dev,
											   int32_t *volt_uv)
{
		const struct regulator_act81461_config *config = dev->config;
		uint8_t val;
		int ret;

		ret = i2c_reg_read_byte_dt(&config->i2c, LDO3_VSET, &val);
		if (ret < 0) {
				return ret;
		}

	val &= LDO3_VSET_MSK;
	*volt_uv = 600000 + (val * 50000);
	return ret;
}

static int
regulator_act81461_vbuck1_get_voltage(const struct device *dev,
							 int32_t *volt_uv)
{
		const struct regulator_act81461_config *config = dev->config;
	uint8_t val;
		int ret;

		ret = i2c_reg_read_byte_dt(&config->i2c, VBUCK1_VSET, &val);
		if (ret < 0) {
				return ret;
		}

	val &= VBUCK1_VSET_MSK;
	*volt_uv = 600000 + (val * 50000);
	return ret;
}

static int
regulator_act81461_vbuckboost_get_voltage(const struct device *dev,
							  int32_t *volt_uv)
{
		const struct regulator_act81461_config *config = dev->config;
	uint8_t val;
		int ret;

		ret = i2c_reg_read_byte_dt(&config->i2c, VBUCKBOOST_VSET, &val);
		if (ret < 0) {
				return ret;
		}

	val &= VBUCKBOOST_VSET_MSK;
	*volt_uv = 600000 + (val * 50000);
		return ret;
}

static int regulator_act81461_set_voltage(const struct device *dev,
										 int32_t min_uv, int32_t max_uv)
{
		const struct regulator_act81461_config *config = dev->config;

		switch (config->source) {
	case ACT81461_SOURCE_VBUCKBOOST_3V3:
				return regulator_act81461_vbuckboost_set_voltage(
						dev, min_uv, max_uv);
	case ACT81461_SOURCE_VBUCK1_1V85:
				return regulator_act81461_vbuck1_set_voltage(
						dev, min_uv, max_uv);
	case ACT81461_SOURCE_LDO1_2V20:
	case ACT81461_SOURCE_LDO1_1V85:
				return regulator_act81461_ldo1_set_voltage(dev, min_uv, max_uv);
		case ACT81461_SOURCE_LDO3_1V85:
		return regulator_act81461_ldo3_set_voltage(dev, min_uv, max_uv);
				break;
		default:
				__ASSERT(NULL, "Unexpected source");
		}

		return 0;
}

static int regulator_act81461_get_voltage(const struct device *dev,
										 int32_t *volt_uv)
{
		const struct regulator_act81461_config *config = dev->config;

		switch (config->source) {
	case ACT81461_SOURCE_VBUCKBOOST_3V3:
				return regulator_act81461_vbuckboost_get_voltage(dev, volt_uv);
		case ACT81461_SOURCE_VBUCK1_1V85:
				return regulator_act81461_vbuck1_get_voltage(dev, volt_uv);
		case ACT81461_SOURCE_LDO1_2V20:
		case ACT81461_SOURCE_LDO1_1V85:
				return regulator_act81461_ldo1_get_voltage(dev, volt_uv);
		case ACT81461_SOURCE_LDO3_1V85:
		return regulator_act81461_ldo3_get_voltage(dev, volt_uv);
				break;
		default:
				__ASSERT(NULL, "Unexpected source");
		}

		return 0;
}

static int pmic_interrupts_configure (void)
{
		/* Setup and configure the interrupt for PMIC_CHARGE_STATUS */
		gpioa_dev = device_get_binding(GPIO_A_NAME);
		if (!gpioa_dev) {
				LOG_ERR("GPIOA driver error\n");
		}

		/* Setup GPIO input, and triggers on rising edge. */
		ret = gpio_pin_configure(gpioa_dev, PMIC_CHARGE_STATUS, GPIO_INPUT | GPIO_PULL_UP | GPIO_INT_EDGE_FALLING);
		if (ret) {
				LOG_ERR("Error configuring GPIO_A %d!\n", PMIC_CHARGE_STATUS);
		}

		gpio_init_callback(&gpio_batt_chrg_cb, pmic_battery_charge_intr_callback, BIT(PMIC_CHARGE_STATUS));

		ret = gpio_add_callback(gpioa_dev, &gpio_batt_chrg_cb);
		if (ret) {
				LOG_ERR("Cannot setup callback!\n");
		}

		ret = gpio_pin_interrupt_configure(gpioa_dev, PMIC_CHARGE_STATUS, GPIO_INPUT | GPIO_PULL_UP | GPIO_INT_EDGE_FALLING);

		/* Setup and configure the interrupt for PMIC_INT */
		gpiof_dev = device_get_binding(GPIO_F_NAME);
		if (!gpiof_dev) {
				LOG_ERR("GPIOF driver error\n");
		}

		/* Setup GPIO input, and triggers on rising edge. */
		ret = gpio_pin_configure(gpiof_dev, PMIC_INT, GPIO_INPUT | GPIO_PULL_UP | GPIO_INT_EDGE_FALLING);
		if (ret) {
				LOG_ERR("Error configuring GPIO_F %d!\n", PMIC_INT);
		}

		gpio_init_callback(&gpio_cb, pmic_intr_callback, BIT(PMIC_INT));

		ret = gpio_add_callback(gpiof_dev, &gpio_cb);
		if (ret) {
				LOG_ERR("Cannot setup callback!\n");
		}

		ret = gpio_pin_interrupt_configure(gpiof_dev, PMIC_INT, GPIO_INPUT | GPIO_PULL_UP | GPIO_INT_EDGE_FALLING);

		if (!device_is_ready(i2c1_dev)) {
				LOG_ERR("I2C: device is not ready\n");
		}

	return 0;
}

static int pmic_init_regulator (const struct device *dev)
{
		struct regulator_data *data = dev->data;
	const struct regulator_act81461_config *config = dev->config;

		printk("\nact81461 parent_api pmic_init_regulator...\n");

		return 0;
}

static int regulator_act81461_init(const struct device *dev)
{
	const struct regulator_act81461_config *config = dev->config;
		struct regulator_data *data = dev->data;
		static bool init_regs_once = false;


		LOG_INF("act81461 Driver Initialized\n");

		printk("\n");
		printk("\nact81461 Driver Started..\n.");
		printk("\n");


		/* Check to verify we have a valid I2C device */
		if (!device_is_ready(i2c1_dev)) {
				printk("\nact81461 : I2C: device is not ready ..\n.");
				return -ENODEV;
		}
		else{
			   printk("\nact81461 : I2C: device is ready ..\n.");
		}

		if (!init_regs_once)
		{
				pmic_interrupts_configure();

				printk("\n");
				printk("reg modify count %d\n", config->pmic_init_reg_array_num);
				for (int i=0; i < config->pmic_init_reg_array_num; i++)
				{
						pmic_init_reg_array[i] = config->pmic_init_reg_array[i];
						pmic_init_reg_value_array[i] = config->pmic_init_reg_value_array[i];
						//regulator_modify_register(config, pmic_init_reg_array[i], 0xff, pmic_init_reg_value_array[i]);
				}
				init_regs_once = true;
				printk("\n");
		}

		return regulator_common_init(dev, true);
}

static int regulator_act81461_common_init(const struct device *dev)
{
	const struct regulator_act81461_config *config = dev->config;
	int ret = 0;

	if (!device_is_ready(i2c1_dev)) {
		return -ENODEV;
	}

	return ret;
}

static const struct regulator_parent_driver_api parent_api = {
	.dvs_state_set = pmic_init_regulator
};

static const struct regulator_driver_api api = {
	.enable = regulator_act81461_enable,
	.disable = regulator_act81461_disable,
	.count_voltages = regulator_act81461_count_voltages,
	.list_voltage = regulator_act81461_list_voltage,
	.set_voltage = regulator_act81461_set_voltage,
	.get_voltage = regulator_act81461_get_voltage,
};

#define ACT81461_PARENT_INIT_PRIORITY 76
#define ACT81461_CHILD_INIT_PRIORITY  75

#define REGULATOR_ACT81461_DEFINE(node_id, id, _source, parent)			\
	static struct regulator_act81461_data data_##id;			\
										\
	static const struct regulator_act81461_config config_##id = {		\
		.common = REGULATOR_DT_COMMON_CONFIG_INIT(node_id),		\
		.p = parent,							\
		.source = _source,						\
		};									\
										\
	DEVICE_DT_DEFINE(node_id, regulator_act81461_init, NULL, &data_##id,	\
		&config_##id, POST_KERNEL,					\
		CONFIG_REGULATOR_ACT81461_INIT_PRIORITY, &api);

#define REGULATOR_ACT81461_DEFINE_COND(inst, child, source, parent)		\
	COND_CODE_1(DT_NODE_EXISTS(DT_INST_CHILD(inst, child)),			\
		(REGULATOR_ACT81461_DEFINE(DT_INST_CHILD(inst, child),		\
				child ## inst, source, parent)),		\
					())

#define REGULATOR_ACT81461_DEFINE_ALL(inst)					\
	static const struct regulator_act81461_config config_##inst = {		\
		.i2c = I2C_DT_SPEC_INST_GET(inst),				\
		};									\
										\
	DEVICE_DT_INST_DEFINE(inst, regulator_act81461_common_init, NULL, NULL, \
		&config_##inst, POST_KERNEL,					\
		ACT81461_PARENT_INIT_PRIORITY,					\
							   NULL);						\
										\
		REGULATOR_ACT81461_DEFINE_COND(inst, buckboost, ACT81461_SOURCE_VBUCKBOOST_3V3, \
									  DEVICE_DT_INST_GET(inst))				\
		REGULATOR_ACT81461_DEFINE_COND(inst, buck1, ACT81461_SOURCE_VBUCK1_1V85,	\
									  DEVICE_DT_INST_GET(inst))				\
		REGULATOR_ACT81461_DEFINE_COND(inst, ldo1, ACT81461_SOURCE_LDO1_1V85,		\
									  DEVICE_DT_INST_GET(inst))				\
		REGULATOR_ACT81461_DEFINE_COND(inst, ldo3, ACT81461_SOURCE_LDO3_1V85,		\
									  DEVICE_DT_INST_GET(inst))

DT_INST_FOREACH_STATUS_OKAY(REGULATOR_ACT81461_DEFINE_ALL)
