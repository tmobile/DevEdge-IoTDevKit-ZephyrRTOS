/*
 * Copyright (c) 2023 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT sony_cxd5605
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include "cxd5605.h"
#include <cxd5605_lib.h>


void cxd5605_pin_set(struct cxd5605_dev *cxd_dev, enum cxd_pin pin, int value)
{
    const struct cxd5605_config *cfg = cxd_dev->user_data;

    switch (pin) {
	case cxd_rst_gpio:
        gpio_pin_set_dt(&cfg->rst_gpio, value);
        break;
	case cxd_boot_rec_gpio:
        gpio_pin_set_dt(&cfg->boot_rec_gpio, value);
        break;
	case cxd_alert_gpio:
        gpio_pin_set_dt(&cfg->alert_gpio, value);
        break;
    default:
        break;
    }
}

void cxd5605_pin_conf(struct cxd5605_dev *cxd_dev, enum cxd_pin pin, enum cxd_pin_dir inout)
{
    const struct cxd5605_config *cfg = cxd_dev->user_data;

    gpio_flags_t pincfg = inout ?  GPIO_INPUT : GPIO_OUTPUT;

    switch (pin) {
	case cxd_rst_gpio:
        gpio_pin_configure_dt(&cfg->rst_gpio, pincfg);
        break;
	case cxd_boot_rec_gpio:
        gpio_pin_configure_dt(&cfg->boot_rec_gpio, pincfg);
        break;
	case cxd_alert_gpio:
        gpio_pin_configure_dt(&cfg->alert_gpio, pincfg);
        break;
    default:
        break;
    }
}

void cxd5605_sem_give(cxd5605_sem sem)
{
    struct k_sem *ksem = sem;

    k_sem_give(ksem);
}

int cxd5605_sem_take(cxd5605_sem sem, int timeout_ms)
{
    struct k_sem *ksem = sem;

    return k_sem_take(ksem, K_MSEC(timeout_ms));
}

int cxd5605_i2c_read(struct cxd5605_dev *cxd_dev, uint8_t *data, uint8_t num_bytes)
{
    const struct cxd5605_config *cfg = cxd_dev->user_data;

    return i2c_read_dt(&cfg->i2c_spec, data, num_bytes);
}

int cxd5605_i2c_write(struct cxd5605_dev *cxd_dev, uint8_t *data, uint8_t num_bytes)
{
    const struct cxd5605_config *cfg = cxd_dev->user_data;

    return i2c_write_dt(&cfg->i2c_spec, data, num_bytes);
}

void cxd5605_usleep(int us)
{
    k_usleep(us);
}
