/*
 * Copyright (c) 2023 T-Mobile USA, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

/* MSTR00 */
#define MASTER_CONF_REG0 0x00

/* BB_REG00 */
#define BUCKBOOST_STAT_REG 0x90
#define BUCKBOOST_STAT_REG_ERR_MSK 0xf0

/* B1_REG00 */
#define VBUCK1_STAT_REG 0x60
#define VBUCK1_STAT_ERR_MSK 0xe0

/* B2_REG00 */
#define VBUCK2_STAT_REG 0x70
#define VBUCK2_STAT_ERR_MSK 0xe0

/* BOOST_REG00 */
#define VBOOST_STAT_REG 0x80
#define VBOOST_STAT_ERR_MSK 0x70

/* LDO12_REG00 */
#define LDO1_STAT_REG 0x30
#define LDO1_STAT_ERR_MSK 0xe0

/* LDO12_REG05 */
#define LDO2_STAT_REG 0x35
#define LDO2_STAT_ERR_MSK 0xe0

/* LDO34_REG00 */
#define LDO3_STAT_REG 0x40
#define LDO3_STAT_ERR_MSK 0xe0

/* LDO34_REG05 */
#define LSW4_STAT_REG 0x45
#define LSW4_STAT_ERR_MSK 0xe0

/* LSW56_REG00 */
#define LSW5_STAT_REG 0x50
#define LSW5_STAT_ERR_MSK 0xe0

/* LSW56_REG05 */
#define LSW6_STAT_REG 0x55
#define LSW6_STAT_ERR_MSK 0xe0

/* BB_REG04 */
#define BUCKBOOST_ENABLE_REG 0x94
#define BUCKBOOST_ENABLE_MSK 0x80
#define BUCKBOOST_ENABLE 0x80

/* B1_REG04 */
#define BUCK1_ENABLE_REG 0x64
#define BUCK1_ENABLE_MSK 0x80
#define BUCK1_ENABLE 0x80

/* B2_REG04 */
#define BUCK2_ENABLE_REG 0x74
#define BUCK2_ENABLE_MSK 0x80
#define BUCK2_ENABLE 0x80

/* BOOST_REG03 */
#define BOOST_ENABLE_REG 0x83
#define BOOST_ENABLE_MSK 0x80
#define BOOST_ENABLE 0x80

/* LDO12_REG02 */
#define LDO1_ENABLE_REG 0x32
#define LDO1_ENABLE_MSK 0x80
#define LDO1_ENABLE 0x80

/* LDO12_REG07 */
#define LDO2_ENABLE_REG 0x37
#define LDO2_ENABLE_MSK 0x80
#define LDO2_ENABLE 0x80

/* LDO34_REG02 */
#define LDO3_ENABLE_REG 0x42
#define LDO3_ENABLE_MSK 0x80
#define LDO3_ENABLE 0x80

/* LDO34_REG07 */
#define LSW4_ENABLE_REG 0x47
#define LSW4_ENABLE_MSK 0x80
#define LSW4_ENABLE 0x80

/* LSW56_REG02 */
#define LSW5_ENABLE_REG 0x52
#define LSW5_ENABLE_MSK 0x80
#define LSW5_ENABLE 0x80

/* LSW56_REG07 */
#define LSW6_ENABLE_REG 0x57
#define LSW6_ENABLE_MSK 0x80
#define LSW6_ENABLE 0x80

/*
 * LDO1_VSET
 * VOUT = 0.6V + (VSET * 50mv)
 */
#define LDO1_VSET_REG 0x31
#define LDO1_VSET_MSK 0x3f

/*
 * LDO2_VSET
 * VOUT = 0.6V + (VSET * 50mv)
 */
#define LDO2_VSET_REG 0x36
#define LDO2_VSET_MSK 0x3f

/*
 * LDO3_VSET
 * VOUT = 0.6V + (VSET * 50mv)
 */
#define LDO3_VSET_REG 0x41
#define LDO3_VSET_MSK 0x3f

/*
 * B1_REG02
 * VOUT = 0.6V + (VSET * 50mv)
 */
#define VBUCK1_VSET_REG 0x62
#define VBUCK1_VSET_MSK 0x3f

/*
 * B2_REG02
 * VOUT = 0.6V + (VSET * 50mv)
 */
#define VBUCK2_VSET_REG 0x72
#define VBUCK2_VSET_MSK 0x3f

/*
 * BOOST_REG01
 * VOUT = 5.0V + (VSET * 250mv)
 */
#define VBOOST_VSET_REG 0x81
#define VBOOST_VSET_MSK 0x3f

/*
 * BB_REG02
 * VOUT = 3.2V + (VSET * 50mv)
 */
#define VBUCKBOOST_VSET_REG 0x92
#define VBUCKBOOST_VSET_MSK 0x3f

#define BUCKBOOST_SWFREQ_MSK 0x40
#define BUCK1_SWFREQ_MSK 0x80
#define BUCK2_SWFREQ_MSK 0x80
#define BOOST_SWFREQ_MSK 0x80
#define LDO1_SWFREQ_MSK 0x00
#define LDO2_SWFREQ_MSK 0x00
#define LDO3_SWFREQ_MSK 0x80

#define REGULATOR_ERROR_UNDER_VOLTAGE BIT(3)
#define REGULATOR_ERROR_VSYS_UNDER_VOLTAGE BIT(4)

#define TWARN 0x20
#define VSYS  0x10

#define PWR_OK 0x80
#define OV_FLT 0x40
#define ILIM_FLT 0x20

#define BST_UV BIT(4)
#define BST_OV BIT(5)
#define BST_ILIM_WRN BIT(6)
