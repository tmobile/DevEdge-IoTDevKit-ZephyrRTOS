/*
 * Copyright (c) 2022-2023 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ams_tsl2540

#include "tsl2540.h"

#include <stdlib.h>

#include <zephyr/pm/device.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#define DEF_GETTER_SETTER(name, type)                                                              \
	type get_##name(const struct device *dev)                                                  \
	{                                                                                          \
		return ((const struct tsl2540_data *)dev->data)->reg.name.value;                   \
	}                                                                                          \
	void set_##name(const struct device *dev, const type value)                                \
	{                                                                                          \
		((struct tsl2540_data *)dev->data)->reg.name.value = value;                        \
	}                                                                                          \
	int fetch_##name(const struct device *dev)                                                 \
	{                                                                                          \
		const struct i2c_dt_spec *spec =                                                   \
			&((const struct tsl2540_config *)dev->config)->i2c_spec;                   \
		const struct tsl2540_data *data = dev->data;                                       \
		int rc;                                                                            \
		if ((rc = i2c_burst_read_dt(spec, data->reg.name.address,                          \
					    (uint8_t *)&data->reg.name.value,                      \
					    sizeof data->reg.name.value))) {                       \
			errno = rc;                                                                \
			LOG_ERR("%s status: %d", __func__, rc);                                    \
		}                                                                                  \
		LOG_DBG("%#x: %#x", data->reg.name.address, data->reg.name.value);                 \
		return data->reg.name.value;                                                       \
	}                                                                                          \
                                                                                                   \
	int flush_##name(const struct device *dev)                                                 \
	{                                                                                          \
		const struct i2c_dt_spec *spec =                                                   \
			&((const struct tsl2540_config *)dev->config)->i2c_spec;                   \
		const struct tsl2540_data *data = dev->data;                                       \
		int rc;                                                                            \
		if ((rc = i2c_burst_write_dt(spec, data->reg.name.address,                         \
					     (uint8_t *)&data->reg.name.value,                     \
					     sizeof data->reg.name.value))) {                      \
			errno = rc;                                                                \
			LOG_ERR("%s status: %d", __func__, rc);                                    \
		}                                                                                  \
		LOG_DBG("%#x: %#x", data->reg.name.address, data->reg.name.value);                 \
		return rc;                                                                         \
	}

DEF_GETTER_SETTER(ENABLE, uint8_t)
DEF_GETTER_SETTER(ATIME, uint8_t)
DEF_GETTER_SETTER(WTIME, uint8_t)
DEF_GETTER_SETTER(AILT, uint16_t)
DEF_GETTER_SETTER(AIHT, uint16_t)
DEF_GETTER_SETTER(PERS, uint8_t)
DEF_GETTER_SETTER(CFG0, uint8_t)
DEF_GETTER_SETTER(CFG1, uint8_t)
DEF_GETTER_SETTER(REVID, uint8_t)
DEF_GETTER_SETTER(ID, uint8_t)
DEF_GETTER_SETTER(STATUS, uint8_t)
DEF_GETTER_SETTER(VISDATA, uint16_t)
DEF_GETTER_SETTER(IRDATA, uint16_t)
DEF_GETTER_SETTER(REVID2, uint8_t)
DEF_GETTER_SETTER(CFG2, uint8_t)
DEF_GETTER_SETTER(CFG3, uint8_t)
DEF_GETTER_SETTER(AZ_CONFIG, uint8_t)
DEF_GETTER_SETTER(INTENAB, uint8_t)

int fetch_all(const struct device *dev)
{
	typedef int (*Functions)(const struct device *dev);
	const Functions functions[] = {fetch_ENABLE,	fetch_ATIME,  fetch_WTIME,  fetch_AILT,
				       fetch_AIHT,	fetch_PERS,   fetch_CFG0,   fetch_CFG1,
				       fetch_REVID,	fetch_ID,     fetch_STATUS, fetch_VISDATA,
				       fetch_IRDATA,	fetch_REVID2, fetch_CFG2,   fetch_CFG3,
				       fetch_AZ_CONFIG, fetch_INTENAB};
	for (int ii = 0; ii < sizeof functions / sizeof *functions; ii++) {
		(void)functions[ii](dev);
		if (errno) {
			return errno;
		}
	}

	return 0;
}

int flush_all(const struct device *dev)
{
	typedef int (*Functions)(const struct device *dev);
	const Functions functions[] = {flush_ATIME, flush_WTIME,     flush_AILT,    flush_AIHT,
				       flush_PERS,  flush_CFG0,	     flush_CFG1,    flush_CFG2,
				       flush_CFG3,  flush_AZ_CONFIG, flush_INTENAB, flush_ENABLE};
	if (get_ENABLE(dev)) {
		uint8_t desired_state = get_ENABLE(dev);
		set_ENABLE(dev, TSL2540_EN_SLEEP);
		flush_ENABLE(dev);
		set_ENABLE(dev, desired_state);
	}

	for (int ii = 0; ii < sizeof functions / sizeof *functions; ii++) {
		(void)functions[ii](dev);
		if (errno) {
			return errno;
		}
	}

	return 0;
}

static int tsl2540_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct tsl2540_data *data = dev->data;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_LIGHT ||
			chan == SENSOR_CHAN_IR);

	k_sem_take(&data->sem, K_FOREVER);
	if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_LIGHT) {
		errno = 0;
		fetch_VISDATA(dev);
		if (errno) {
			LOG_ERR("Could not fetch ambient light (visible); errno: %d", errno);
		} else {
			data->count_vis = sys_le16_to_cpu(get_VISDATA(dev));
			fetch_IRDATA(dev);
			if (errno) {
				LOG_ERR("Could not fetch ambient light (IR); errno: %d", errno);
			} else {
				data->count_vis = sys_le16_to_cpu(get_IRDATA(dev));
			}
		}
	}
	k_sem_give(&data->sem);

	return -errno;
}

static int tsl2540_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct tsl2540_data *data = dev->data;
	int ret = 0;
	double cpl;

	LOG_DBG("int %s(*dev: %p, chan: %d, *val: %p)", __func__, dev, chan, val);

	k_sem_take(&data->sem, K_FOREVER);

	cpl = (data->integration_time + 1) * data->again * 2.81;

	switch (chan) {
	case SENSOR_CHAN_LIGHT:
		sensor_value_from_double(val, data->count_vis / cpl * 53.0 * data->glass_attenuation);
		break;
	case SENSOR_CHAN_IR:
		sensor_value_from_double(val, data->count_ir / cpl * 53.0 * data->glass_attenuation_ir);
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

	LOG_DBG("int %s(*dev: %p, chan: %d, attr: %d, *val: %p)", __func__, dev, chan, attr, val);

	k_sem_take(&data->sem, K_FOREVER);

	if (chan == SENSOR_CHAN_IR || chan == SENSOR_CHAN_LIGHT) {
		switch ((enum sensor_attribute_tsl2540)attr) {
		case TSL2540_SENSOR_ATTR_GAIN:
			switch ((enum sensor_gain_tsl2540)val->val1) {
			case TSL2540_SENSOR_GAIN_1_2:
				set_CFG1(dev, TSL2540_CFG1_G1_2);
				set_CFG2(dev, TSL2540_CFG2_G1_2);
				data->again = TSL2540_AGAIN_S1_2;
				break;
			case TSL2540_SENSOR_GAIN_1:
				set_CFG1(dev, TSL2540_CFG1_G1);
				set_CFG2(dev, TSL2540_CFG2_G1);
				data->again = TSL2540_AGAIN_S1;
				break;
			case TSL2540_SENSOR_GAIN_4:
				set_CFG1(dev, TSL2540_CFG1_G4);
				set_CFG2(dev, TSL2540_CFG2_G4);
				data->again = TSL2540_AGAIN_S4;
				break;
			case TSL2540_SENSOR_GAIN_16:
				set_CFG1(dev, TSL2540_CFG1_G16);
				set_CFG2(dev, TSL2540_CFG2_G16);
				data->again = TSL2540_AGAIN_S16;
				break;
			case TSL2540_SENSOR_GAIN_64:
				set_CFG1(dev, TSL2540_CFG1_G64);
				set_CFG2(dev, TSL2540_CFG2_G64);
				data->again = TSL2540_AGAIN_S64;
				break;
			case TSL2540_SENSOR_GAIN_128:
				set_CFG1(dev, TSL2540_CFG1_G128);
				set_CFG2(dev, TSL2540_CFG2_G128);
				data->again = TSL2540_AGAIN_S128;
				break;
			default:
				ret = -EINVAL;
			}
			break;

		/** Sensor Integration Time (in ms) */
		case TSL2540_SENSOR_ATTR_INTEGRATION_TIME: {
			uint8_t itv;
			double it;

			it = sensor_value_to_double(val);
			it /= 2.81;
			if (it < 1 || it > 256) {
				ret = -EINVAL;
			}
			it -= 1;
			itv = (uint8_t)it;

			set_ATIME(dev, itv);

			data->integration_time = itv;
		} break;

		case TSL2540_SENSOR_ATTR_GLASS_ATTENUATION: /** Sensor Glass Attenuation Factor */
			data->glass_attenuation = sensor_value_to_double(val);
			break;

		case TSL2540_SENSOR_ATTR_INT_APERS: /** Sensor persistence filter. */
			uint8_t apv = (uint8_t)val->val1;
			if (apv > 15) {
				ret = -EINVAL;
			}
			set_PERS(dev, apv);
			break;

		default:
#if CONFIG_TSL2540_TRIGGER
		{
			double cpl;

			cpl = ((data->integration_time + 1) * 2.81) * data->again;
			uint16_t thld = sensor_value_to_double(val) * cpl / 53.0 / data->glass_attenuation;

			switch (attr) {
			case SENSOR_ATTR_UPPER_THRESH:
				set_AIHT(dev, sys_cpu_to_le16(thld));
				break;

			case SENSOR_ATTR_LOWER_THRESH:
				set_AILT(dev, sys_cpu_to_le16(thld));
				break;

			default:
				ret = -ENOTSUP;
				break;
			}
		}
#else
			ret = -ENOTSUP;
#endif
		break;
		}
	}

	flush_all(dev);

	k_sem_give(&data->sem);

	return ret;
}

static int tsl2540_setup(const struct device *dev)
{
	struct sensor_value integration_time;

	LOG_DBG("int %s(*dev: %p)", __func__, dev);

	/* Set ALS integration time */
	tsl2540_attr_set(dev, SENSOR_CHAN_LIGHT, TSL2540_SENSOR_ATTR_GAIN,
			 &(struct sensor_value){.val1 = TSL2540_SENSOR_GAIN_1, .val2 = 0});

	sensor_value_from_double(&integration_time, 400.0);
	tsl2540_attr_set(dev, SENSOR_CHAN_LIGHT, TSL2540_SENSOR_ATTR_INTEGRATION_TIME,
			 &integration_time);

	return 0;
}

static int tsl2540_init(const struct device *dev)
{
	const struct tsl2540_config *cfg = dev->config;
	struct tsl2540_data *data = dev->data;

	LOG_DBG("int %s(*dev: %p)", __func__, dev);

	k_sem_init(&data->sem, 1, K_SEM_MAX_LIMIT);

	fetch_all(dev);

	/** Calculated from datasheet typical CPL */
	data->glass_attenuation = 2.27205;
	data->glass_attenuation_ir = 2.34305;

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

	LOG_DBG("int %s(*dev: %p) Init complete", __func__, dev);

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

	LOG_DBG("int %s(*dev: %p, action: %d)", __func__, dev, action);

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		set_ENABLE(dev, TSL2540_EN_ACTIVE);
		errno = 0;
		flush_ENABLE(dev);
		ret = errno;
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		set_ENABLE(dev, TSL2540_EN_SLEEP);
		errno = 0;
		flush_ENABLE(dev);
		ret = errno;
		break;
	default:
		return -ENOTSUP;
	}

	return ret;
}
#endif

#define TSL2540_DEFINE(inst)                                                                       \
	static struct tsl2540_data tsl2540_prv_data_##inst = {                                     \
		.reg = {{0x80, 0x00, "ENABLE", "R/W", "Enables states and functions"},             \
			{0x81, 0x00, "ATIME", "R/W", "ALS integration time"},                      \
			{0x83, 0x00, "WTIME", "R/W", "Wait time"},                                 \
			{0x84, 0x0000, "AILT", "R/W", "ALS interrupt low threshold low byte"},     \
			{0x86, 0x0000, "AIHT", "R/W", "ALS interrupt high threshold low byte"},    \
			{0x8C, 0x00, "PERS", "R/W", "ALS interrupt persistence filters"},          \
			{0x8D, 0x80, "CFG0", "R/W", "Configuration register zero"},                \
			{0x90, 0x00, "CFG1", "R/W", "Configuration register one"},                 \
			{0x91, 0x61, "REVID", "R", "Revision ID"},                                 \
			{0x92, 0xE4, "ID", "R", "Device ID"},                                      \
			{0x93, 0x00, "STATUS", "R", "Device status register"},                     \
			{0x94, 0x0000, "VISDATA", "R", "Visible channel data low byte"},           \
			{0x96, 0x0000, "IRDATA", "R", "IR channel data low byte"},                 \
			{0x9E, 0x00, "REVID2", "R", "Auxiliary ID"},                               \
			{0x9F, 0x04, "CFG2", "R/W", "Configuration register two"},                 \
			{0xAB, 0x0C, "CFG3", "R/W", "Configuration register three"},               \
			{0xD6, 0x7F, "AZ_CONFIG", "R/W", "Autozero configuration"},                \
			{0xDD, 0x00, "INTENAB", "R/W", "Interrupt enables"}}};                     \
	static const struct tsl2540_config tsl2540_config_##inst = {                               \
		.i2c_spec = I2C_DT_SPEC_INST_GET(inst),                                            \
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0})};                       \
	PM_DEVICE_DT_INST_DEFINE(inst, tsl2540_pm_action);                                         \
	DEVICE_DT_INST_DEFINE(inst, &tsl2540_init, PM_DEVICE_DT_INST_GET(inst),                    \
			      &tsl2540_prv_data_##inst, &tsl2540_config_##inst, POST_KERNEL,       \
			      CONFIG_SENSOR_INIT_PRIORITY, &tsl2540_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TSL2540_DEFINE)
