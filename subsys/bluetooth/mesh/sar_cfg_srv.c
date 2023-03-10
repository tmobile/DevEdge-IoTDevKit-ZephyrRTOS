/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include <zephyr/types.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/mesh.h>

#include "net.h"
#include "access.h"
#include "foundation.h"
#include "mesh.h"
#include "sar_cfg_internal.h"

#define LOG_LEVEL CONFIG_BT_MESH_MODEL_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_mesh_sar_cfg_srv);

static void transmitter_status(struct bt_mesh_model *model,
			       struct bt_mesh_msg_ctx *ctx)
{
	BT_MESH_MODEL_BUF_DEFINE(msg, OP_SAR_CFG_TX_STATUS, BT_MESH_SAR_TX_LEN);
	const struct bt_mesh_sar_tx *tx = &bt_mesh.sar_tx;

	LOG_DBG("SAR TX {0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x}",
		tx->seg_int_step, tx->unicast_retrans_count,
		tx->unicast_retrans_without_prog_count,
		tx->unicast_retrans_int_step, tx->unicast_retrans_int_inc,
		tx->multicast_retrans_count, tx->multicast_retrans_int);

	bt_mesh_model_msg_init(&msg, OP_SAR_CFG_TX_STATUS);
	bt_mesh_sar_tx_encode(&msg, tx);

	if (bt_mesh_model_send(model, ctx, &msg, NULL, NULL)) {
		LOG_ERR("Unable to send Transmitter Status");
	}
}

static void receiver_status(struct bt_mesh_model *model,
			    struct bt_mesh_msg_ctx *ctx)
{
	BT_MESH_MODEL_BUF_DEFINE(msg, OP_SAR_CFG_RX_STATUS, BT_MESH_SAR_RX_LEN);
	const struct bt_mesh_sar_rx *rx = &bt_mesh.sar_rx;

	LOG_DBG("SAR RX {0x%02x 0x%02x 0x%02x 0x%02x 0x%02x}", rx->seg_thresh,
		rx->ack_delay_inc, rx->discard_timeout, rx->rx_seg_int_step,
		rx->ack_retrans_count);

	bt_mesh_model_msg_init(&msg, OP_SAR_CFG_RX_STATUS);
	bt_mesh_sar_rx_encode(&msg, rx);

	if (bt_mesh_model_send(model, ctx, &msg, NULL, NULL)) {
		LOG_ERR("Unable to send Receiver Status");
	}
}

static int transmitter_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
			   struct net_buf_simple *buf)
{
	LOG_DBG("src 0x%04x", ctx->addr);

	transmitter_status(model, ctx);

	return 0;
}

static int transmitter_set(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
			   struct net_buf_simple *buf)
{
	struct bt_mesh_sar_tx *tx = &bt_mesh.sar_tx;

	LOG_DBG("src 0x%04x", ctx->addr);

	bt_mesh_sar_tx_decode(buf, tx);
	transmitter_status(model, ctx);

	return 0;
}

static int receiver_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
			struct net_buf_simple *buf)
{
	LOG_DBG("src 0x%04x", ctx->addr);

	receiver_status(model, ctx);

	return 0;
}

static int receiver_set(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
			struct net_buf_simple *buf)
{
	struct bt_mesh_sar_rx *rx = &bt_mesh.sar_rx;

	LOG_DBG("src 0x%04x", ctx->addr);

	bt_mesh_sar_rx_decode(buf, rx);
	receiver_status(model, ctx);

	return 0;
}

const struct bt_mesh_model_op bt_mesh_sar_cfg_srv_op[] = {
	{ OP_SAR_CFG_TX_GET, BT_MESH_LEN_EXACT(0), transmitter_get },
	{ OP_SAR_CFG_TX_SET, BT_MESH_LEN_EXACT(BT_MESH_SAR_TX_LEN), transmitter_set },
	{ OP_SAR_CFG_RX_GET, BT_MESH_LEN_EXACT(0), receiver_get },
	{ OP_SAR_CFG_RX_SET, BT_MESH_LEN_EXACT(BT_MESH_SAR_RX_LEN), receiver_set },
	BT_MESH_MODEL_OP_END,
};

static int sar_cfg_srv_init(struct bt_mesh_model *model)
{
	if (!bt_mesh_model_in_primary(model)) {
		LOG_ERR("Configuration Server only allowed in primary element");
		return -EINVAL;
	}

	/*
	 * SAR Configuration Model security is device-key based and only the local
	 * device-key is allowed to access this model.
	 */
	model->keys[0] = BT_MESH_KEY_DEV_LOCAL;
	model->flags |= BT_MESH_MOD_DEVKEY_ONLY;

	return 0;
}

const struct bt_mesh_model_cb bt_mesh_sar_cfg_srv_cb = {
	.init = sar_cfg_srv_init,
};
