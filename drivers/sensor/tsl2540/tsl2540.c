/*
 * Copyright (c) 2022 T-Mobile USA, Inc.
 * Copyright (c) 2023 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ams_tsl2540

#include "tsl2540.h"

#include <stdlib.h>

#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(tsl2540, CONFIG_SENSOR_LOG_LEVEL);

int tsl2540_reg_read(const struct device *dev, uint8_t reg, uint8_t *val)
{
	const struct tsl2540_config *cfg = dev->config;
	int result;

	result = i2c_reg_read_byte_dt(&cfg->i2c_spec, reg, val);

	if (result < 0) {
		return result;
	}

	return 0;
}

int tsl2540_reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct tsl2540_config *cfg = dev->config;
	int result;

	result = i2c_reg_write_byte_dt(&cfg->i2c_spec, reg, val);

	if (result < 0) {
		return result;
	}

	return 0;
}

static int tsl2540_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct tsl2540_data *data = dev->data;
	int ret = 0;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_LIGHT ||
			chan == SENSOR_CHAN_IR);
	k_sem_take(&data->sem, K_FOREVER);

	if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_LIGHT) {
		uint16_t le16_buffer;

		ret = i2c_burst_read_dt(
			     &((const struct tsl2540_config *)dev->config)->i2c_spec,
			     TSL2540_REG_VIS_LOW, (uint8_t *)&le16_buffer, sizeof(le16_buffer));
		if (ret) {
			LOG_ERR("Could not fetch ambient light (visible): error %d", ret);
		} else {
			data->count_vis = sys_le16_to_cpu(le16_buffer);
		}
	}
	if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_IR) {
		uint16_t le16_buffer;

		ret = i2c_burst_read_dt(
			     &((const struct tsl2540_config *)dev->config)->i2c_spec,
			     TSL2540_REG_IR_LOW, (uint8_t *)&le16_buffer, sizeof(le16_buffer));
		if (ret) {
			LOG_ERR("Could not fetch ambient light (IR): error %d", ret);
		} else {
			data->count_ir = sys_le16_to_cpu(le16_buffer);
		}
	}
	k_sem_give(&data->sem);

	return ret;
}

static int tsl2540_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct tsl2540_data *data = dev->data;
	int ret = 0;
	double cpl;

	k_sem_take(&data->sem, K_FOREVER);

	cpl = (data->integration_time + 1) * 2.81;
	cpl *= data->again;

	switch (chan) {
	case SENSOR_CHAN_LIGHT:
		sensor_value_from_double(val,
					 data->count_vis / cpl * 53.0 * data->glass_attenuation);
		break;
	case SENSOR_CHAN_IR:
		sensor_value_from_double(val,
					 data->count_ir / cpl * 53.0 * data->glass_attenuation_ir);
		break;
	default:
		ret = -ENOTSUP;
	}

	k_sem_give(&data->sem);

	return ret;
}

int tsl2540_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr,
		     const struct sensor_value *val)
{
	struct tsl2540_data *data = dev->data;
	int ret = 0;

	k_sem_take(&data->sem, K_FOREVER);
	tsl2540_reg_write(dev, TSL2540_ENABLE_ADDR, TSL2540_ENABLE_MASK & ~TSL2540_ENABLE_CONF);
#if CONFIG_TSL2540_TRIGGER
	if (chan == SENSOR_CHAN_LIGHT) {
		if (attr == SENSOR_ATTR_UPPER_THRESH) {
			double cpl;
			uint16_t thld, le16_buffer;

			cpl = ((data->integration_time + 1) * 2.81);
			cpl *= data->again;
			cpl /= (53 * data->glass_attenuation);
			thld = sensor_value_to_double(val) * cpl;
			LOG_DBG("attr: %d, cpl: %g, thld: %x\n", attr, cpl, thld);

			le16_buffer = sys_cpu_to_le16(thld);
			ret = i2c_burst_write_dt(
				&((const struct tsl2540_config *)dev->config)->i2c_spec,
				TSL2540_REG_AIHT_LOW, (uint8_t *)&le16_buffer, sizeof(le16_buffer));

			goto exit;
		}
		if (attr == SENSOR_ATTR_LOWER_THRESH) {
			double cpl;
			uint16_t thld, le16_buffer;

			cpl = ((data->integration_time + 1) * 2.81);
			cpl *= data->again;
			cpl /= (53 * data->glass_attenuation);
			thld = sensor_value_to_double(val) * cpl;
			LOG_DBG("attr: %d, cpl: %g, thld: %x\n", attr, cpl, thld);

			le16_buffer = sys_cpu_to_le16(sys_cpu_to_le16(thld));

			ret = i2c_burst_write_dt(
				&((const struct tsl2540_config *)dev->config)->i2c_spec,
				TSL2540_REG_AILT_LOW, (uint8_t *)&le16_buffer, sizeof(le16_buffer));

			goto exit;
		}
		if ((enum sensor_attribute_tsl2540)attr == SENSOR_ATTR_GLASS_ATTENUATION) {
			data->glass_attenuation = sensor_value_to_double(val);
			ret = 0;
			goto exit;
		}
	}
	if (chan == SENSOR_CHAN_IR) {
		if ((enum sensor_attribute_tsl2540)attr == SENSOR_ATTR_GLASS_ATTENUATION) {
			data->glass_attenuation = sensor_value_to_double(val);
			ret = 0;
			goto exit;
		}
	}
#endif
	if (chan == SENSOR_CHAN_IR || chan == SENSOR_CHAN_LIGHT) {
		if (attr == (enum sensor_attribute)SENSOR_ATTR_GAIN) {
			switch (val->val1) {
			case TSL2540_SENSOR_GAIN_1_2:
				if (tsl2540_reg_write(dev, TSL2540_REG_CFG_1, TSL2540_CFG1_G1_2)) {
					ret = -EIO;
					goto exit;
				}
				if (tsl2540_reg_write(dev, TSL2540_REG_CFG_2, TSL2540_CFG2_G1_2)) {
					ret = -EIO;
					goto exit;
				}
				data->again = TSL2540_AGAIN_S1_2;
				ret = 0;
				goto exit;
			case TSL2540_SENSOR_GAIN_1:
				if (tsl2540_reg_write(dev, TSL2540_REG_CFG_1, TSL2540_CFG1_G1)) {
					ret = -EIO;
					goto exit;
				}
				if (tsl2540_reg_write(dev, TSL2540_REG_CFG_2, TSL2540_CFG2_G1)) {
					ret = -EIO;
					goto exit;
				}
				data->again = TSL2540_AGAIN_S1;
				ret = 0;
				goto exit;
			case TSL2540_SENSOR_GAIN_4:
				if (tsl2540_reg_write(dev, TSL2540_REG_CFG_1, TSL2540_CFG1_G4)) {
					ret = -EIO;
					goto exit;
				}
				if (tsl2540_reg_write(dev, TSL2540_REG_CFG_2, TSL2540_CFG2_G4)) {
					ret = -EIO;
					goto exit;
				}
				data->again = TSL2540_AGAIN_S4;
				ret = 0;
				goto exit;
			case TSL2540_SENSOR_GAIN_16:
				if (tsl2540_reg_write(dev, TSL2540_REG_CFG_1, TSL2540_CFG1_G16)) {
					ret = -EIO;
					goto exit;
				}
				if (tsl2540_reg_write(dev, TSL2540_REG_CFG_2, TSL2540_CFG2_G16)) {
					ret = -EIO;
					goto exit;
				}
				data->again = TSL2540_AGAIN_S16;
				ret = 0;
				goto exit;
			case TSL2540_SENSOR_GAIN_64:
				if (tsl2540_reg_write(dev, TSL2540_REG_CFG_1, TSL2540_CFG1_G64)) {
					ret = -EIO;
					goto exit;
				}
				if (tsl2540_reg_write(dev, TSL2540_REG_CFG_2, TSL2540_CFG2_G64)) {
					ret = -EIO;
					goto exit;
				}
				data->again = TSL2540_AGAIN_S64;
				ret = 0;
				goto exit;
			case TSL2540_SENSOR_GAIN_128:
				if (tsl2540_reg_write(dev, TSL2540_REG_CFG_1, TSL2540_CFG1_G128)) {
					ret = -EIO;
					goto exit;
				}
				if (tsl2540_reg_write(dev, TSL2540_REG_CFG_2, TSL2540_CFG2_G128)) {
					ret = -EIO;
					goto exit;
				}
				data->again = TSL2540_AGAIN_S128;
				ret = 0;
				goto exit;
			default:
				ret = -EINVAL;
				goto exit;
			}
			if (attr == (enum sensor_attribute)SENSOR_ATTR_INT_APERS) {
				uint8_t apv = (uint8_t)val->val1;

				if (apv > 15) {
					ret = -EINVAL;
					goto exit;
				}
				if (tsl2540_reg_write(dev, TSL2540_REG_PERS, apv)) {
					ret = -EIO;
					goto exit;
				}
			}
		}
		if (attr == (enum sensor_attribute)SENSOR_ATTR_INTEGRATION_TIME) {
			uint8_t itv;
			double it;

			it = sensor_value_to_double(val);
			it /= 2.81;
			if (it < 1 || it > 256) {
				ret = -EINVAL;
				goto exit;
			}
			it -= 1;
			itv = (uint8_t)it;
			if (tsl2540_reg_write(dev, TSL2540_REG_ATIME, itv)) {
				ret = -EIO;
				goto exit;
			}

			data->integration_time = itv;
			ret = 0;
			goto exit;
		}
	}

	ret = -ENOTSUP;
exit:
	tsl2540_reg_write(dev, TSL2540_ENABLE_ADDR, TSL2540_ENABLE_MASK & TSL2540_ENABLE_CONF);

	k_sem_give(&data->sem);

	return ret;
}

static int tsl2540_setup(const struct device *dev)
{
	struct sensor_value integration_time;

	/* Set ALS integration time */
	tsl2540_attr_set(dev, SENSOR_CHAN_LIGHT, SENSOR_ATTR_GAIN,
			 &(struct sensor_value){.val1 = TSL2540_SENSOR_GAIN_1_2, .val2 = 0});

	sensor_value_from_double(&integration_time, 500.0);
	tsl2540_attr_set(dev, SENSOR_CHAN_LIGHT, SENSOR_ATTR_INTEGRATION_TIME, &integration_time);

	return 0;
}

static int tsl2540_init(const struct device *dev)
{
	const struct tsl2540_config *cfg = dev->config;
	struct tsl2540_data *data = dev->data;

	k_sem_init(&data->sem, 1, K_SEM_MAX_LIMIT);

	/** Calculated from datasheet typical CPL */
	data->glass_attenuation = 2.27205;
	data->glass_attenuation_ir = 2.34305;

	tsl2540_reg_write(dev, TSL2540_REG_PERS, 1);

	if (!device_is_ready(cfg->i2c_spec.bus)) {
		LOG_ERR("I2C dev %s not ready", cfg->i2c_spec.bus->name);
		return -ENODEV;
	}

	if (tsl2540_setup(dev)) {
		LOG_ERR("Failed to setup ambient light functionality");
		return -EIO;
	}

#if CONFIG_TSL2540_TRIGGER
	if (tsl2540_trigger_init(dev)) {
		LOG_ERR("Could not initialize interrupts");
		return -EIO;
	}
#endif

	LOG_DBG("Init complete");

	return 0;
}

static const struct sensor_driver_api tsl2540_driver_api = {
	.sample_fetch = tsl2540_sample_fetch,
	.channel_get = tsl2540_channel_get,
	.attr_set = tsl2540_attr_set,
#ifdef CONFIG_TSL2540_TRIGGER
	.trigger_set = tsl2540_trigger_set,
#endif
};

/* Todo: finish PM */
#ifdef CONFIG_PM_DEVICE
static int tsl2540_pm_action(const struct device *dev, enum pm_device_action action)
{
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		ret = tsl2540_reg_write(dev, TSL2540_ENABLE_ADDR,
					TSL2540_ENABLE_MASK & TSL2540_ENABLE_CONF);
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		ret = tsl2540_reg_write(dev, TSL2540_ENABLE_ADDR,
					TSL2540_ENABLE_MASK & ~TSL2540_ENABLE_CONF);
		break;
	default:
		return -ENOTSUP;
	}

	return ret;
}
#endif

#define TSL2540_DEFINE(inst)                                                                       \
	static struct tsl2540_data tsl2540_prv_data_##inst;                                        \
	static const struct tsl2540_config tsl2540_config_##inst = {                               \
		.i2c_spec = I2C_DT_SPEC_INST_GET(inst),                                            \
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0})};                       \
	PM_DEVICE_DT_INST_DEFINE(inst, tsl2540_pm_action);                                         \
	DEVICE_DT_INST_DEFINE(inst, &tsl2540_init, PM_DEVICE_DT_INST_GET(inst),                    \
			      &tsl2540_prv_data_##inst, &tsl2540_config_##inst, POST_KERNEL,       \
			      CONFIG_SENSOR_INIT_PRIORITY, &tsl2540_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TSL2540_DEFINE)
