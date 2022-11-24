/*
 * Copyright (c) 2018 Christian Taedcke
 * Copyright (c) 2022 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __INC_BOARD_H
#define __INC_BOARD_H

/* This pin is used to enable the serial port using the board controller */
#define BC_ENABLE_GPIO_NODE  DT_NODELABEL(gpioa)
#define BC_ENABLE_GPIO_PIN   5

/* This pin is used to enable the temperature sensor using the board controller */
#define TMP_ENABLE_GPIO_NODE  DT_NODELABEL(gpiob)
#define TMP_ENABLE_GPIO_PIN   10

#endif /* __INC_BOARD_H */
