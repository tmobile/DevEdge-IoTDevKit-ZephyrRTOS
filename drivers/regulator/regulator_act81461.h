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

/*
 * MSTR18
 * Interrupt source
 */
#define INT_SRC_REG 0x18
#define INT_SRC_MSTR 0x01
#define INT_SRC_GPIO 0x02
#define INT_SRC_LDO1 0x31
#define INT_SRC_LDO2 0x32
#define INT_SRC_LDO3 0x41
#define INT_SRC_LSW4 0x42
#define INT_SRC_LSW5 0x51
#define INT_SRC_LSW6 0x52
#define INT_SRC_BUCK1 0x61
#define INT_SRC_BUCK2 0x62
#define INT_SRC_BOOST 0x80
#define INT_SRC_BB 0x91
#define INT_SRC_ALPC 0xC0
#define INT_SRC_UNKN 0xFF

/*
 * MSTR00
 * System interrupts
 */
#define INT_SRC_MSTR_REG 0x00
#define INT_SRC_MSTR_REG_MSK 0x7F
#define INT_SRC_MSTR_MSK_REG 0x01

/*
 * MSTR22
 * GPIO interrupts
 */
#define INT_SRC_GPIO_REG 0x22
#define INT_SRC_GPIO_REG_MSK 0x03
#define INT_SRC_GPIO_MSK_REG 0x23

/*
 * APLC_REG02
 * ALPC interrupts
 */
#define INT_SRC_ALPC_REG 0xC2

/*
 * LDO12_REG00
 * LDO1 interrupt mask register
 */
#define INT_MSK_REG_LDO1 0x30

/*
 * LDO12_REG05
 * LDO2 interrupt mask register
 */
#define INT_MSK_REG_LDO2 0x35

/*
 * LDO34_REG00
 * LDO3 interrupt mask register
 */
#define INT_MSK_REG_LDO3 0x40

/*
 * LDO34_REG05
 * LSW4 interrupt mask register
 */
#define INT_MSK_REG_LSW4 0x45

/*
 * LSW56_REG00
 * LSW5 interrupt mask register
 */
#define INT_MSK_REG_LSW5 0x50

/*
 * LSW56_REG05
 * LSW6 interrupt mask register
 */
#define INT_MSK_REG_LSW6 0x55

/*
 * B1_REG00
 * BUCK1 interrupt mask register
 */
#define INT_MSK_REG_BUCK1 0x60

/*
 * B2_REG00
 * BUCK2 interrupt mask register
 */
#define INT_MSK_REG_BUCK2 0x70

/*
 * BB_REG00
 * BUCKBOOST interrupt mask register
 */
#define INT_MSK_REG_BUCKBOOST 0x90

/*
 * BOOST_REG00
 * BOOST interrupt mask register
 */
#define INT_MSK_REG_BOOST 0x80

/*
 * MSTR01
 * Master interrupt mask register
 */
#define INT_MSK_REG_MSTR 0x01

/*
 * MSTR03
 * VSYSMON configuration register
 */
#define INT_MSK_REG_ALPC_0 0xC3
#define INT_MSK_REG_ALPC_1 0xC4

/*
 * APLC_REG04
 * ALPC interrupt mask register
 */
#define VSYSMON_REG 0x03
#define VSYSMON_MSK 0x0F

#define BUCKBOOST_SWFREQ_MSK 0x40
#define BUCK1_SWFREQ_MSK 0x80
#define BUCK2_SWFREQ_MSK 0x80
#define BOOST_SWFREQ_MSK 0x80
#define LDO1_SWFREQ_MSK 0x00
#define LDO2_SWFREQ_MSK 0x00
#define LDO3_SWFREQ_MSK 0x80

#define VBAT_LOW_MSK 0x10

#define TWARN 0x20
#define VSYS  0x10

#define PWR_OK 0x80
#define OV_FLT 0x40
#define ILIM_FLT 0x20

#define PWR_GOOD_INT_MSK 0x08
#define OV_INT_MSK 0x04
#define ILIM_INT_MSK 0x02

#define BST_UV_INT_MSK 0x01
#define BST_OV_INT_MSK 0x02
#define BST_ILIM_INT_MSK 0x04

#define BST_UV 0x10
#define BST_OV 0x20
#define BST_ILIM_WRN 0x40

#define VBAT_LOW_STAT 0x10
