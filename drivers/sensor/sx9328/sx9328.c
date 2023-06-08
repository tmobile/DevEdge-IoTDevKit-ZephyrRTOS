/*
 * Copyright (c) 2023 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT semtech_sx9328

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/sx9328.h>
#include <zephyr/pm/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include "sx9328.h"

LOG_MODULE_REGISTER(SX9328, CONFIG_SENSOR_LOG_LEVEL);

/**
 * Set attributes for the device.
 * @param dev - The device structure.
 * @param chan - The sensor channel type.
 * @param attr - The sensor attribute.
 * @param value - The sensor attribute value.
 * @return 0 in case of success, negative error code otherwise.
 */
static int sx9328_attr_set(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, const struct sensor_value *val)
{
	ARG_UNUSED(chan);
	const struct sx9328_config *cfg = dev->config;
	struct sx9328_data *data = (struct sx9328_data *)dev->data;
	uint8_t rval;

	switch ((enum sensor_attribute_sx9328)attr) {
	case SENSOR_ATTR_SX9328_DOZE_PERIOD:
		rval = val->val1 << SX9328_DOZE_SHFT;
		return i2c_reg_update_byte_dt(&cfg->i2c, SX9328_DOZE_REG, SX9328_DOZE_MSK, rval);
	case SENSOR_ATTR_SX9328_SCAN_PERIOD:
		rval = val->val1 << SX9328_SCAN_SHFT;
		return i2c_reg_update_byte_dt(&cfg->i2c, SX9328_SCAN_REG, SX9328_SCAN_MSK, rval);
	case SENSOR_ATTR_SX9328_SCAN_PERIOD_23:
		rval = val->val1 << SX9328_SCAN23_SHFT;
		return i2c_reg_update_byte_dt(&cfg->i2c, SX9328_SCAN23_REG, SX9328_SCAN23_MSK,
					      rval);
	case SENSOR_ATTR_SX9328_PHASES:
		data->phen = (uint8_t)val->val1;
		rval = val->val1 << SX9328_PHEN_SHFT;
		return i2c_reg_update_byte_dt(&cfg->i2c, SX9328_PHEN_REG, SX9328_PHEN_MSK, rval);
	case SENSOR_ATTR_SX9328_PHASE_01_FREQ:
		rval = val->val1 << SX9328_FREQ01_SHFT;
		return i2c_reg_update_byte_dt(&cfg->i2c, SX9328_FREQ01_REG, SX9328_FREQ01_MSK,
					      rval);
	case SENSOR_ATTR_SX9328_PHASE_23_FREQ:
		rval = val->val1 << SX9328_FREQ23_SHFT;
		return i2c_reg_update_byte_dt(&cfg->i2c, SX9328_FREQ23_REG, SX9328_FREQ23_MSK,
					      rval);
	case SENSOR_ATTR_SX9328_PHASE_01_GAIN:
		rval = val->val1 << SX9328_GAIN01_SHFT;
		return i2c_reg_update_byte_dt(&cfg->i2c, SX9328_GAIN_REG, SX9328_GAIN01_MSK, rval);
	case SENSOR_ATTR_SX9328_PHASE_23_GAIN:
		rval = val->val1 << SX9328_GAIN23_SHFT;
		return i2c_reg_update_byte_dt(&cfg->i2c, SX9328_GAIN_REG, SX9328_GAIN23_MSK, rval);
	case SENSOR_ATTR_SX9328_PHASE_01_PRX_THD:
		return i2c_reg_write_byte_dt(&cfg->i2c, SX9328_PRX_THD_01_REG, (uint8_t)val->val1);
	case SENSOR_ATTR_SX9328_PHASE_23_PRX_THD:
		return i2c_reg_write_byte_dt(&cfg->i2c, SX9328_PRX_THD_23_REG, (uint8_t)val->val1);
	case SENSOR_ATTR_SX9328_PHASE_01_HITBL_THD:
		rval = val->val1 << SX9328_PRX_TBL_THD_01_HI_SHFT;
		return i2c_reg_update_byte_dt(&cfg->i2c, SX9328_PRX_TBL_THD_01_REG,
					      SX9328_PRX_TBL_THD_01_HI_MSK, rval);
	case SENSOR_ATTR_SX9328_PHASE_23_HITBL_THD:
		rval = val->val1 << SX9328_PRX_TBL_THD_23_HI_SHFT;
		return i2c_reg_update_byte_dt(&cfg->i2c, SX9328_PRX_TBL_THD_23_REG,
					      SX9328_PRX_TBL_THD_23_HI_MSK, rval);
	case SENSOR_ATTR_SX9328_PHASE_01_LOTBL_THD:
		rval = val->val1 << SX9328_PRX_TBL_THD_01_LO_SHFT;
		return i2c_reg_update_byte_dt(&cfg->i2c, SX9328_PRX_TBL_THD_01_REG,
					      SX9328_PRX_TBL_THD_01_LO_MSK, rval);
	case SENSOR_ATTR_SX9328_PHASE_23_LOTBL_THD:
		rval = val->val1 << SX9328_PRX_TBL_THD_23_LO_SHFT;
		return i2c_reg_update_byte_dt(&cfg->i2c, SX9328_PRX_TBL_THD_23_REG,
					      SX9328_PRX_TBL_THD_23_LO_MSK, rval);
	case SENSOR_ATTR_SX9328_PHASE_01_BDY_THD:
		rval = val->val1 << SX9328_PRX_BDY_01_THD_SHFT;
		return i2c_reg_update_byte_dt(&cfg->i2c, SX9328_PRX_BDY_THD_REG,
					      SX9328_PRX_BDY_01_THD_MSK, rval);
	case SENSOR_ATTR_SX9328_PHASE_23_BDY_THD:
		rval = val->val1 << SX9328_PRX_BDY_23_THD_SHFT;
		return i2c_reg_update_byte_dt(&cfg->i2c, SX9328_PRX_BDY_THD_REG,
					      SX9328_PRX_BDY_23_THD_MSK, rval);
	case SENSOR_ATTR_SX9328_PRX_PHASES:
		data->prox_en &= ~SX9328_PRX_PHASES_FLAGS_MSK;
		data->prox_en |=
			(val->val1 << SX9328_PRX_PHASES_FLAGS_SHFT) & SX9328_PRX_PHASES_FLAGS_MSK;
		return 0;
	case SENSOR_ATTR_SX9328_TBL_PRX_PHASES:
		data->tbl_prox_en &= ~SX9328_TBL_PHASES_FLAGS_MSK;
		data->tbl_prox_en |=
			(val->val1 << SX9328_TBL_PHASES_FLAGS_SHFT) & SX9328_TBL_PHASES_FLAGS_MSK;
		return 0;
	case SENSOR_ATTR_SX9328_BDY_PRX_PHASES:
		data->bdy_prox_en &= ~SX9328_BDY_PHASES_FLAGS_MSK;
		data->bdy_prox_en |=
			(val->val1 << SX9328_BDY_PHASES_FLAGS_SHFT) & SX9328_BDY_PHASES_FLAGS_MSK;
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int sx9328_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct sx9328_data *data = dev->data;
	const struct sx9328_config *cfg = dev->config;

	__ASSERT_NO_MSG(
		chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_PROX ||
		(chan >= SENSOR_CHAN_SX9328_TBL_PRX_PH0 && chan <= SENSOR_CHAN_SX9328_BDY_PRX_PH3));

	if (i2c_reg_read_byte_dt(&cfg->i2c, SX9328_STAT_REG, &data->prox_stat) < 0) {
		return -EIO;
	}

	return i2c_reg_read_byte_dt(&cfg->i2c, SX9328_TBL_BDY_STAT_REG, &data->bdy_tbl_stat);
}

static int sx9328_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct sx9328_data *data = (struct sx9328_data *)dev->data;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_PROX);
	switch ((int16_t)chan) {
	case SENSOR_CHAN_PROX:
		val->val1 = (data->prox_stat & data->prox_en) ? 1 : 0;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_SX9328_BDY_PRX:
		val->val1 = (data->bdy_tbl_stat & data->bdy_prox_en) ? 1 : 0;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_SX9328_TBL_PRX:
		val->val1 = (data->bdy_tbl_stat & data->tbl_prox_en) ? 1 : 0;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_SX9328_TBL_PRX_PH0:
		val->val1 = (data->prox_stat & (BIT(0) << 4)) ? 1 : 0;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_SX9328_TBL_PRX_PH1:
		val->val1 = (data->prox_stat & (BIT(1) << 4)) ? 1 : 0;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_SX9328_TBL_PRX_PH2:
		val->val1 = (data->prox_stat & (BIT(2) << 4)) ? 1 : 0;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_SX9328_TBL_PRX_PH3:
		val->val1 = (data->prox_stat & (BIT(3) << 4)) ? 1 : 0;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_SX9328_BDY_PRX_PH0:
		val->val1 = (data->prox_stat & BIT(0)) ? 1 : 0;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_SX9328_BDY_PRX_PH1:
		val->val1 = (data->prox_stat & BIT(1)) ? 1 : 0;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_SX9328_BDY_PRX_PH2:
		val->val1 = (data->prox_stat & BIT(2)) ? 1 : 0;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_SX9328_BDY_PRX_PH3:
		val->val1 = (data->prox_stat & BIT(3)) ? 1 : 0;
		val->val2 = 0;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api sx9328_api_funcs = {
	.sample_fetch = sx9328_sample_fetch,
	.channel_get = sx9328_channel_get,
	.attr_set = sx9328_attr_set,
#ifdef CONFIG_SX9328_TRIGGER
	.trigger_set = sx9328_trigger_set,
#endif
};

static int sx9328_init_chip(const struct device *dev)
{
	const struct sx9328_config *cfg = dev->config;
	struct sx9328_data *data = (struct sx9328_data *)dev->data;
	uint8_t val;

	/* Reset the chip */
	if (i2c_reg_write_byte_dt(&cfg->i2c, SX9328_RST_REG, SX9328_RST_CMD) < 0) {
		return -EIO;
	}

	k_msleep(200);

	/* Check WHOAMI */
	if (i2c_reg_read_byte_dt(&cfg->i2c, SX9328_WHOAMI_REG, &val) < 0) {
		return -EIO;
	}

	if (val != 0x23 && val != 0x22) {
		LOG_WRN("Wrong WHOAMI value: %02x", val);
	}

	/* No interrupts active.  We only activate them when an
	 * application registers a trigger.
	 */
	if (i2c_reg_write_byte_dt(&cfg->i2c, SX9328_IRQ_MSK_REG, 0) < 0) {
		return -EIO;
	}

	/* Read irq source reg to clear reset status. */
	if (i2c_reg_read_byte_dt(&cfg->i2c, SX9328_IRQ_SRC_REG, &val) < 0) {
		return -EIO;
	}

	/* Initialize configuration */
	val = cfg->freq01 << SX9328_FREQ01_SHFT;
	if (i2c_reg_update_byte_dt(&cfg->i2c, SX9328_FREQ01_REG, SX9328_FREQ01_MSK, val) < 0) {
		return -EIO;
	}

	val = cfg->freq23 << SX9328_FREQ23_SHFT;
	if (i2c_reg_update_byte_dt(&cfg->i2c, SX9328_FREQ23_REG, SX9328_FREQ23_MSK, val) < 0) {
		return -EIO;
	}

	val = cfg->scan_period << SX9328_SCAN_SHFT;
	if (i2c_reg_update_byte_dt(&cfg->i2c, SX9328_SCAN_REG, SX9328_SCAN_MSK, val) < 0) {
		return -EIO;
	}

	val = cfg->scan_period_23 << SX9328_SCAN23_SHFT;
	if (i2c_reg_update_byte_dt(&cfg->i2c, SX9328_SCAN23_REG, SX9328_SCAN23_MSK, val) < 0) {
		return -EIO;
	}

	val = cfg->doze_period << SX9328_DOZE_SHFT;
	if (i2c_reg_update_byte_dt(&cfg->i2c, SX9328_DOZE_REG, SX9328_DOZE_MSK, val) < 0) {
		return -EIO;
	}

	val = cfg->gain01 << 3 | cfg->gain23;
	if (i2c_reg_write_byte_dt(&cfg->i2c, SX9328_GAIN_REG, val) < 0) {
		return -EIO;
	}

	if (i2c_reg_write_byte_dt(&cfg->i2c, SX9328_PRX_THD_01_REG, cfg->prx_thd_01) < 0) {
		return -EIO;
	}

	if (i2c_reg_write_byte_dt(&cfg->i2c, SX9328_PRX_THD_23_REG, cfg->prx_thd_23) < 0) {
		return -EIO;
	}

	val = cfg->bdy_thd_01 << 4 | cfg->bdy_thd_23;
	if (i2c_reg_write_byte_dt(&cfg->i2c, SX9328_PRX_BDY_THD_REG, val) < 0) {
		return -EIO;
	}

	val = cfg->thi_thd_01 << 4 | cfg->tlo_thd_01;
	if (i2c_reg_write_byte_dt(&cfg->i2c, SX9328_PRX_TBL_THD_01_REG, val) < 0) {
		return -EIO;
	}

	val = cfg->thi_thd_23 << 4 | cfg->tlo_thd_23;
	if (i2c_reg_write_byte_dt(&cfg->i2c, SX9328_PRX_TBL_THD_23_REG, val) < 0) {
		return -EIO;
	}

	if (i2c_reg_write_byte_dt(&cfg->i2c, SX9328_IRQ_MSK_REG, 0) < 0) {
		return -EIO;
	}

	val = cfg->close_deb << SX9328_CLOSE_DEB_SHFT | cfg->close_deb << SX9328_FAR_DEB_SHFT;
	if (i2c_reg_update_byte_dt(&cfg->i2c, SX9328_DEB_REG, SX9328_DEB_MSK, val) < 0) {
		return -EIO;
	}

	val = 0;
	for (int i = 0; i < cfg->num_phases; i++) {
		const struct sx9328_phx_config *ph = &cfg->phases[i];
		uint8_t ph_cfg;
		uint8_t ph_reg = SX9328_PH0_CFG_REG + ph->phase;

		ph_cfg = ph->cs0 | ph->cs1 << 2 | ph->cs2 << 4;

		if (i2c_reg_write_byte_dt(&cfg->i2c, ph_reg, ph_cfg) < 0) {
			return -EIO;
		}

		if (ph->enabled) {
			val |= BIT(ph->phase) << SX9328_PHEN_SHFT;
		}

		if (ph->bdy_sns) {
			data->bdy_prox_en |= BIT(ph->phase) << SX9328_BDY_PHASES_FLAGS_SHFT;
		}

		if (ph->tbl_sns) {
			data->tbl_prox_en |= BIT(ph->phase) << SX9328_TBL_PHASES_FLAGS_SHFT;
		}

		if (ph->prx_sns) {
			data->prox_en |= BIT(ph->phase);
		}
	}

	data->phen = val;

	return i2c_reg_update_byte_dt(&cfg->i2c, SX9328_PHEN_REG, SX9328_PHEN_MSK, val);
}

int sx9328_init(const struct device *dev)
{
	const struct sx9328_config *cfg = dev->config;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	if (sx9328_init_chip(dev) < 0) {
		LOG_DBG("sx9328: failed to initialize chip");
		return -EINVAL;
	}

#ifdef CONFIG_SX9328_TRIGGER
	if (cfg->int_gpio.port) {
		if (sx9328_setup_interrupt(dev) < 0) {
			LOG_DBG("sx9328: failed to setup interrupt");
			return -EINVAL;
		}
	}
#endif /* CONFIG_SX9328_TRIGGER */

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int sx9328_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct sx9328_config *cfg = dev->config;
	struct sx9328_data *data = (struct sx9328_data *)dev->data;
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		/* Re-initialize the chip */
		ret = i2c_reg_update_byte_dt(&cfg->i2c, SX9328_PHEN_REG, SX9328_PHEN_MSK,
					     data->phen);
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		ret = i2c_reg_update_byte_dt(&cfg->i2c, SX9328_PHEN_REG, SX9328_PHEN_MSK, 0);
		break;
	default:
		return -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */

#define SX9328_PH_CFG_INIT(ph)                                                                     \
	{.phase = DT_PROP(ph, phase_num),                                                          \
	 .enabled = DT_PROP(ph, enabled),                                                          \
	 .prx_sns = DT_PROP(ph, prx_sns),                                                          \
	 .bdy_sns = DT_PROP(ph, bdy_sns),                                                          \
	 .tbl_sns = DT_PROP(ph, tbl_sns),                                                          \
	 .cs0 = DT_PROP(ph, cs0_use),                                                              \
	 .cs1 = DT_PROP(ph, cs1_use),                                                              \
	 .cs2 = DT_PROP(ph, cs2_use)},

#define SX9328_DEFINE(n)                                                                           \
	struct sx9328_data sx9328_data_##n;                                                        \
                                                                                                   \
	const struct sx9328_phx_config ph_cfg_##n[] = {                                            \
		DT_INST_FOREACH_CHILD(n, SX9328_PH_CFG_INIT)};                                     \
                                                                                                   \
	static const struct sx9328_config sx9328_config_##n = {                                    \
		.i2c = I2C_DT_SPEC_INST_GET(n),                                                    \
		.scan_period = DT_INST_PROP(n, scan_period),                                       \
		.scan_period_23 = DT_INST_PROP(n, scan_period_23),                                 \
		.doze_period = DT_INST_PROP(n, doze_period),                                       \
		.freq01 = DT_INST_PROP(n, freq_01),                                                \
		.freq23 = DT_INST_PROP(n, freq_23),                                                \
		.gain01 = DT_INST_PROP(n, gain_01),                                                \
		.gain23 = DT_INST_PROP(n, gain_23),                                                \
		.prx_thd_01 = DT_INST_PROP(n, proximity_threshold_01),                             \
		.prx_thd_23 = DT_INST_PROP(n, proximity_threshold_23),                             \
		.bdy_thd_01 = DT_INST_PROP(n, body_threshold_01),                                  \
		.bdy_thd_23 = DT_INST_PROP(n, body_threshold_01),                                  \
		.tlo_thd_01 = DT_INST_PROP(n, table_low_threshold_01),                             \
		.tlo_thd_23 = DT_INST_PROP(n, table_low_threshold_01),                             \
		.thi_thd_01 = DT_INST_PROP(n, table_high_threshold_01),                            \
		.thi_thd_23 = DT_INST_PROP(n, table_high_threshold_23),                            \
		.close_deb = DT_INST_PROP(n, close_debounce),                                      \
		.far_deb = DT_INST_PROP(n, far_debounce),                                          \
		.phases = ph_cfg_##n,                                                              \
		.num_phases = ARRAY_SIZE(ph_cfg_##n),                                              \
		IF_ENABLED(CONFIG_SX9328_TRIGGER,                                                  \
			   (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(n, int_gpios, {0}),))};	\
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(n, sx9328_pm_action);                                             \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(n, sx9328_init, PM_DEVICE_DT_INST_GET(n), &sx9328_data_##n,   \
				     &sx9328_config_##n, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
				     &sx9328_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(SX9328_DEFINE)
