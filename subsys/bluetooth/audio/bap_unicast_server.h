/*  Bluetooth Audio Unicast Server */

/*
 * Copyright (c) 2021-2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>

int bt_bap_unicast_server_reconfig(struct bt_bap_stream *stream, const struct bt_codec *codec);
int bt_bap_unicast_server_start(struct bt_bap_stream *stream);
int bt_bap_unicast_server_metadata(struct bt_bap_stream *stream, struct bt_codec_data meta[],
				   size_t meta_count);
int bt_bap_unicast_server_disable(struct bt_bap_stream *stream);
int bt_bap_unicast_server_release(struct bt_bap_stream *stream);
