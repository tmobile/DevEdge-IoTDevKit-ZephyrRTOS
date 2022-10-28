/*
 * Copyright (c) 2022 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Get the RSSI of a connection
 *
 * @param conn Connection to read RSSI for
 * @return int Absolute value of RSSI on sucxess, negative errno on fail.
 */
int bt_conn_le_get_rssi(const struct bt_conn *conn);
