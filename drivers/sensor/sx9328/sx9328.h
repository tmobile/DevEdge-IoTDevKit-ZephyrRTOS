/*
 * Copyright (c) 2023 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_SX9328_SX9328_H_
#define ZEPHYR_DRIVERS_SENSOR_SX9328_SX9328_H_

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#define SX9328_IRQ_SRC_REG				0x00
#define SX9328_STAT_REG					0x01
#define SX9328_PRX_PHASES_FLAGS_SHFT	0x00
#define SX9328_PRX_PHASES_FLAGS_MSK		0x0F
#define SX9328_TBL_BDY_STAT_REG			0x02
#define SX9328_TBL_PHASES_FLAGS_SHFT	0x04
#define SX9328_TBL_PHASES_FLAGS_MSK		0xF0
#define SX9328_BDY_PHASES_FLAGS_SHFT	0x00
#define SX9328_BDY_PHASES_FLAGS_MSK		0x0F
#define SX9328_IRQ_MSK_REG				0x05
#define SX9328_CONV_DONE_IRQ			0x08
#define SX9328_NEAR_FAR_IRQ				0x60
#define SX9328_NEAR_FAR_TBL_BDY_IRQ		0x04
#define SX9328_PH0_CFG_REG				0x28
#define SX9328_IRQ_CFG_REG				0x07
#define SX9328_IRQ_CFG_HIGH_POLARITY	0x40
#define SX9328_PHEN_REG					0x11
#define SX9328_PHEN_SHFT				0x00
#define SX9328_PHEN_MSK					0x0F
#define SX9328_DOZE_REG					0x10
#define SX9328_DOZE_MSK					0x60
#define SX9328_DOZE_SHFT				0x05
#define SX9328_SCAN_REG					0x10
#define SX9328_SCAN_MSK					0x1F
#define SX9328_SCAN_SHFT				0x00
#define SX9328_SCAN23_REG				0x11
#define SX9328_SCAN23_MSK				0xC0
#define SX9328_SCAN23_SHFT				0x06
#define SX9328_FREQ01_REG				0x24
#define SX9328_FREQ01_MSK				0xF8
#define SX9328_FREQ01_SHFT				0x03
#define SX9328_FREQ23_REG				0x27
#define SX9328_FREQ23_MSK				0xF8
#define SX9328_FREQ23_SHFT				0x03
#define SX9328_GAIN_REG					0x30
#define SX9328_GAIN01_SHFT				0x03
#define SX9328_GAIN01_MSK				0x38
#define SX9328_GAIN23_SHFT				0x03
#define SX9328_GAIN23_MSK				0x38
#define SX9328_DEB_REG					0x35
#define SX9328_DEB_MSK					0x0F
#define SX9328_CLOSE_DEB_SHFT			0x02
#define SX9328_FAR_DEB_SHFT				0x00
#define SX9328_PRX_THD_01_REG			0x36
#define SX9328_PRX_THD_23_REG			0x37
#define SX9328_PRX_BDY_THD_REG			0x4A
#define SX9328_PRX_BDY_01_THD_SHFT		0x04
#define SX9328_PRX_BDY_01_THD_MSK		0xF0
#define SX9328_PRX_BDY_23_THD_SHFT		0x00
#define SX9328_PRX_BDY_23_THD_MSK		0x0F
#define SX9328_PRX_TBL_THD_01_REG		0x4B
#define SX9328_PRX_TBL_THD_01_LO_SHFT	0x00
#define SX9328_PRX_TBL_THD_01_LO_MSK	0x0F
#define SX9328_PRX_TBL_THD_01_HI_SHFT	0x04
#define SX9328_PRX_TBL_THD_01_HI_MSK	0xF0
#define SX9328_PRX_TBL_THD_23_REG		0x4C
#define SX9328_PRX_TBL_THD_23_LO_SHFT	0x00
#define SX9328_PRX_TBL_THD_23_LO_MSK	0x0F
#define SX9328_PRX_TBL_THD_23_HI_SHFT	0x04
#define SX9328_PRX_TBL_THD_23_HI_MSK	0xF0
#define SX9328_RST_REG					0x9F
#define SX9328_RST_CMD					0xDE
#define SX9328_WHOAMI_REG				0xFA

struct sx9328_phx_config {
	uint8_t phase;
	uint8_t cs0;
	uint8_t cs1;
	uint8_t cs2;
	bool enabled;
	bool prx_sns;
	bool bdy_sns;
	bool tbl_sns;
};

struct sx9328_config {
	struct i2c_dt_spec i2c;
	const struct sx9328_phx_config *phases;
	uint8_t num_phases;
	uint8_t freq01;
	uint8_t freq23;
	uint8_t scan_period;
	uint8_t scan_period_23;
	uint8_t doze_period;
	uint8_t prx_thd_01;
	uint8_t prx_thd_23;
	uint8_t bdy_thd_01;
	uint8_t bdy_thd_23;
	uint8_t tlo_thd_01;
	uint8_t tlo_thd_23;
	uint8_t thi_thd_01;
	uint8_t thi_thd_23;
	uint8_t gain01;
	uint8_t gain23;
	uint8_t close_deb;
	uint8_t far_deb;
#ifdef CONFIG_SX9328_TRIGGER
	struct gpio_dt_spec int_gpio;
#endif
};

struct sx9328_data {
	uint8_t prox_stat;
	uint8_t bdy_tbl_stat;
	uint8_t phen;
	uint8_t prox_en;
	uint8_t bdy_prox_en;
	uint8_t tbl_prox_en;

	struct gpio_callback gpio_cb;

#ifdef CONFIG_SX9328_TRIGGER_OWN_THREAD
	struct k_sem sem;
#endif

#ifdef CONFIG_SX9328_TRIGGER_GLOBAL_THREAD
	struct k_work work;
#endif

#ifdef CONFIG_SX9328_TRIGGER
	const struct device *dev;
	struct sensor_trigger trigger_drdy;
	struct sensor_trigger trigger_near_far;
	struct sensor_trigger trigger_near_far_tb;

	sensor_trigger_handler_t handler_drdy;
	sensor_trigger_handler_t handler_near_far;
	sensor_trigger_handler_t handler_near_far_tb;
#endif
};

#ifdef CONFIG_SX9328_TRIGGER
int sx9328_setup_interrupt(const struct device *dev);
int sx9328_trigger_set(const struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler);
#else
static inline int sx9328_setup_interrupt(const struct device *dev)
{
	return 0;
}
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_SX9328_SX9328_H_ */
