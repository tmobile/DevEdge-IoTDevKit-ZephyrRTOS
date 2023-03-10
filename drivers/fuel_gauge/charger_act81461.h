/*
 * Copyright (c) 2023 T-Mobile USA, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PMIC Regulator Driver
 * This driver implements the regulator API within Zephyr, and also
 * implements support for a broader API.
 */

#include <zephyr/kernel.h>

/* APLC_REG00 */
#define BAT_STAT_REG 0xC0

/* APLC_REG01 */
#define CHG_STAT_REG 0xC1
#define CHG_STAT_MSK 0x07
#define BAT_TEMP_MSK 0x70
#define BAT_TEMP_SHIFT 4

#define SUSPEND 0x01
#define BATTERY_DETECTED   0x08
#define VINOK              0x20

#define DEG_C(x) (((x) * 10) + 2732)
