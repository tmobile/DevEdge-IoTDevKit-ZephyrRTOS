/** @file
 *  @brief Bluetooth Basic Audio Profile shell
 *
 */

/*
 * Copyright (c) 2020 Intel Corporation
 * Copyright (c) 2022-2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/bap_lc3_preset.h>
#include <zephyr/bluetooth/audio/cap.h>
#include <zephyr/bluetooth/audio/pacs.h>

#include "shell/bt.h"
#include "audio.h"

#define LOCATION BT_AUDIO_LOCATION_FRONT_LEFT
#define CONTEXT BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | BT_AUDIO_CONTEXT_TYPE_MEDIA

#if defined(CONFIG_BT_BAP_UNICAST)

struct shell_stream unicast_streams[CONFIG_BT_MAX_CONN *
				    (UNICAST_SERVER_STREAM_COUNT + UNICAST_CLIENT_STREAM_COUNT)];

static const struct bt_audio_codec_qos_pref qos_pref =
	BT_AUDIO_CODEC_QOS_PREF(true, BT_GAP_LE_PHY_2M, 0u, 60u, 20000u, 40000u, 20000u, 40000u);

#if defined(CONFIG_BT_BAP_UNICAST_CLIENT)
struct bt_bap_unicast_group *default_unicast_group;
static struct bt_bap_unicast_client_cb unicast_client_cbs;
#if CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SNK_COUNT > 0
struct bt_bap_ep *snks[CONFIG_BT_MAX_CONN][CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SNK_COUNT];
#endif /* CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SNK_COUNT > 0 */
#if CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SRC_COUNT > 0
struct bt_bap_ep *srcs[CONFIG_BT_MAX_CONN][CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SRC_COUNT];
#endif /* CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SRC_COUNT > 0 */
#endif /* CONFIG_BT_BAP_UNICAST_CLIENT */
#endif /* CONFIG_BT_BAP_UNICAST */

#if defined(CONFIG_BT_BAP_BROADCAST_SOURCE)
struct shell_stream broadcast_source_streams[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];
struct broadcast_source default_source;
#endif /* CONFIG_BT_BAP_BROADCAST_SOURCE */
#if defined(CONFIG_BT_BAP_BROADCAST_SINK)
static struct bt_bap_stream broadcast_sink_streams[BROADCAST_SNK_STREAM_CNT];
static struct bt_bap_broadcast_sink *default_sink;
#endif /* CONFIG_BT_BAP_BROADCAST_SINK */
static struct bt_bap_stream *default_stream;

static const struct named_lc3_preset lc3_unicast_presets[] = {
	{"8_1_1", BT_BAP_LC3_UNICAST_PRESET_8_1_1(LOCATION, CONTEXT)},
	{"8_2_1", BT_BAP_LC3_UNICAST_PRESET_8_2_1(LOCATION, CONTEXT)},
	{"16_1_1", BT_BAP_LC3_UNICAST_PRESET_16_1_1(LOCATION, CONTEXT)},
	{"16_2_1", BT_BAP_LC3_UNICAST_PRESET_16_2_1(LOCATION, CONTEXT)},
	{"24_1_1", BT_BAP_LC3_UNICAST_PRESET_24_1_1(LOCATION, CONTEXT)},
	{"24_2_1", BT_BAP_LC3_UNICAST_PRESET_24_2_1(LOCATION, CONTEXT)},
	{"32_1_1", BT_BAP_LC3_UNICAST_PRESET_32_1_1(LOCATION, CONTEXT)},
	{"32_2_1", BT_BAP_LC3_UNICAST_PRESET_32_2_1(LOCATION, CONTEXT)},
	{"441_1_1", BT_BAP_LC3_UNICAST_PRESET_441_1_1(LOCATION, CONTEXT)},
	{"441_2_1", BT_BAP_LC3_UNICAST_PRESET_441_2_1(LOCATION, CONTEXT)},
	{"48_1_1", BT_BAP_LC3_UNICAST_PRESET_48_1_1(LOCATION, CONTEXT)},
	{"48_2_1", BT_BAP_LC3_UNICAST_PRESET_48_2_1(LOCATION, CONTEXT)},
	{"48_3_1", BT_BAP_LC3_UNICAST_PRESET_48_3_1(LOCATION, CONTEXT)},
	{"48_4_1", BT_BAP_LC3_UNICAST_PRESET_48_4_1(LOCATION, CONTEXT)},
	{"48_5_1", BT_BAP_LC3_UNICAST_PRESET_48_5_1(LOCATION, CONTEXT)},
	{"48_6_1", BT_BAP_LC3_UNICAST_PRESET_48_6_1(LOCATION, CONTEXT)},
	/* High-reliability presets */
	{"8_1_2", BT_BAP_LC3_UNICAST_PRESET_8_1_2(LOCATION, CONTEXT)},
	{"8_2_2", BT_BAP_LC3_UNICAST_PRESET_8_2_2(LOCATION, CONTEXT)},
	{"16_1_2", BT_BAP_LC3_UNICAST_PRESET_16_1_2(LOCATION, CONTEXT)},
	{"16_2_2", BT_BAP_LC3_UNICAST_PRESET_16_2_2(LOCATION, CONTEXT)},
	{"24_1_2", BT_BAP_LC3_UNICAST_PRESET_24_1_2(LOCATION, CONTEXT)},
	{"24_2_2", BT_BAP_LC3_UNICAST_PRESET_24_2_2(LOCATION, CONTEXT)},
	{"32_1_2", BT_BAP_LC3_UNICAST_PRESET_32_1_2(LOCATION, CONTEXT)},
	{"32_2_2", BT_BAP_LC3_UNICAST_PRESET_32_2_2(LOCATION, CONTEXT)},
	{"441_1_2", BT_BAP_LC3_UNICAST_PRESET_441_1_2(LOCATION, CONTEXT)},
	{"441_2_2", BT_BAP_LC3_UNICAST_PRESET_441_2_2(LOCATION, CONTEXT)},
	{"48_1_2", BT_BAP_LC3_UNICAST_PRESET_48_1_2(LOCATION, CONTEXT)},
	{"48_2_2", BT_BAP_LC3_UNICAST_PRESET_48_2_2(LOCATION, CONTEXT)},
	{"48_3_2", BT_BAP_LC3_UNICAST_PRESET_48_3_2(LOCATION, CONTEXT)},
	{"48_4_2", BT_BAP_LC3_UNICAST_PRESET_48_4_2(LOCATION, CONTEXT)},
	{"48_5_2", BT_BAP_LC3_UNICAST_PRESET_48_5_2(LOCATION, CONTEXT)},
	{"48_6_2", BT_BAP_LC3_UNICAST_PRESET_48_6_2(LOCATION, CONTEXT)},
};

static const struct named_lc3_preset lc3_broadcast_presets[] = {
	{"8_1_1", BT_BAP_LC3_BROADCAST_PRESET_8_1_1(LOCATION, CONTEXT)},
	{"8_2_1", BT_BAP_LC3_BROADCAST_PRESET_8_2_1(LOCATION, CONTEXT)},
	{"16_1_1", BT_BAP_LC3_BROADCAST_PRESET_16_1_1(LOCATION, CONTEXT)},
	{"16_2_1", BT_BAP_LC3_BROADCAST_PRESET_16_2_1(LOCATION, CONTEXT)},
	{"24_1_1", BT_BAP_LC3_BROADCAST_PRESET_24_1_1(LOCATION, CONTEXT)},
	{"24_2_1", BT_BAP_LC3_BROADCAST_PRESET_24_2_1(LOCATION, CONTEXT)},
	{"32_1_1", BT_BAP_LC3_BROADCAST_PRESET_32_1_1(LOCATION, CONTEXT)},
	{"32_2_1", BT_BAP_LC3_BROADCAST_PRESET_32_2_1(LOCATION, CONTEXT)},
	{"441_1_1", BT_BAP_LC3_BROADCAST_PRESET_441_1_1(LOCATION, CONTEXT)},
	{"441_2_1", BT_BAP_LC3_BROADCAST_PRESET_441_2_1(LOCATION, CONTEXT)},
	{"48_1_1", BT_BAP_LC3_BROADCAST_PRESET_48_1_1(LOCATION, CONTEXT)},
	{"48_2_1", BT_BAP_LC3_BROADCAST_PRESET_48_2_1(LOCATION, CONTEXT)},
	{"48_3_1", BT_BAP_LC3_BROADCAST_PRESET_48_3_1(LOCATION, CONTEXT)},
	{"48_4_1", BT_BAP_LC3_BROADCAST_PRESET_48_4_1(LOCATION, CONTEXT)},
	{"48_5_1", BT_BAP_LC3_BROADCAST_PRESET_48_5_1(LOCATION, CONTEXT)},
	{"48_6_1", BT_BAP_LC3_BROADCAST_PRESET_48_6_1(LOCATION, CONTEXT)},
	/* High-reliability presets */
	{"8_1_2", BT_BAP_LC3_BROADCAST_PRESET_8_1_2(LOCATION, CONTEXT)},
	{"8_2_2", BT_BAP_LC3_BROADCAST_PRESET_8_2_2(LOCATION, CONTEXT)},
	{"16_1_2", BT_BAP_LC3_BROADCAST_PRESET_16_1_2(LOCATION, CONTEXT)},
	{"16_2_2", BT_BAP_LC3_BROADCAST_PRESET_16_2_2(LOCATION, CONTEXT)},
	{"24_1_2", BT_BAP_LC3_BROADCAST_PRESET_24_1_2(LOCATION, CONTEXT)},
	{"24_2_2", BT_BAP_LC3_BROADCAST_PRESET_24_2_2(LOCATION, CONTEXT)},
	{"32_1_2", BT_BAP_LC3_BROADCAST_PRESET_32_1_2(LOCATION, CONTEXT)},
	{"32_2_2", BT_BAP_LC3_BROADCAST_PRESET_32_2_2(LOCATION, CONTEXT)},
	{"441_1_2", BT_BAP_LC3_BROADCAST_PRESET_441_1_2(LOCATION, CONTEXT)},
	{"441_2_2", BT_BAP_LC3_BROADCAST_PRESET_441_2_2(LOCATION, CONTEXT)},
	{"48_1_2", BT_BAP_LC3_BROADCAST_PRESET_48_1_2(LOCATION, CONTEXT)},
	{"48_2_2", BT_BAP_LC3_BROADCAST_PRESET_48_2_2(LOCATION, CONTEXT)},
	{"48_3_2", BT_BAP_LC3_BROADCAST_PRESET_48_3_2(LOCATION, CONTEXT)},
	{"48_4_2", BT_BAP_LC3_BROADCAST_PRESET_48_4_2(LOCATION, CONTEXT)},
	{"48_5_2", BT_BAP_LC3_BROADCAST_PRESET_48_5_2(LOCATION, CONTEXT)},
	{"48_6_2", BT_BAP_LC3_BROADCAST_PRESET_48_6_2(LOCATION, CONTEXT)},
};

/* Default to 16_2_1 */
const struct named_lc3_preset *default_sink_preset = &lc3_unicast_presets[3];
const struct named_lc3_preset *default_source_preset = &lc3_unicast_presets[3];
static const struct named_lc3_preset *default_broadcast_source_preset = &lc3_broadcast_presets[3];
static bool initialized;

static struct shell_stream *shell_stream_from_bap_stream(struct bt_bap_stream *bap_stream)
{
	struct bt_cap_stream *cap_stream =
		CONTAINER_OF(bap_stream, struct bt_cap_stream, bap_stream);
	struct shell_stream *sh_stream = CONTAINER_OF(cap_stream, struct shell_stream, stream);

	return sh_stream;
}

#if defined(CONFIG_BT_AUDIO_TX)

static uint16_t get_next_seq_num(struct bt_bap_stream *bap_stream)
{
	struct shell_stream *sh_stream = shell_stream_from_bap_stream(bap_stream);
	const uint32_t interval_us = bap_stream->qos->interval;
	int64_t uptime_ticks;
	int64_t delta_ticks;
	uint64_t delta_us;
	uint16_t seq_num;

	/* Note: This does not handle wrapping of ticks when they go above 2^(62-1) */
	uptime_ticks = k_uptime_ticks();
	delta_ticks = uptime_ticks - sh_stream->connected_at_ticks;

	delta_us = k_ticks_to_us_near64((uint64_t)delta_ticks);
	/* Calculate the sequence number by dividing the stream uptime by the SDU interval */
	seq_num = (uint16_t)(delta_us / interval_us);

	return seq_num;
}
#endif /* CONFIG_BT_AUDIO_TX */

#if defined(CONFIG_LIBLC3) && defined(CONFIG_BT_AUDIO_TX)
/* For the first call-back we push multiple audio frames to the buffer to use the
 * controller ISO buffer to handle jitter.
 */
#define PRIME_COUNT 2U
NET_BUF_POOL_FIXED_DEFINE(sine_tx_pool, CONFIG_BT_ISO_TX_BUF_COUNT,
			  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

#include "lc3.h"
#include "math.h"

#define MAX_SAMPLE_RATE         48000
#define MAX_FRAME_DURATION_US   10000
#define MAX_NUM_SAMPLES         ((MAX_FRAME_DURATION_US * MAX_SAMPLE_RATE) / USEC_PER_SEC)
#define AUDIO_VOLUME            (INT16_MAX - 3000) /* codec does clipping above INT16_MAX - 3000 */
#define AUDIO_TONE_FREQUENCY_HZ   400

static int16_t audio_buf[MAX_NUM_SAMPLES];
static lc3_encoder_t lc3_encoder;
static lc3_encoder_mem_48k_t lc3_encoder_mem;
static int lc3_freq_hz;
static int lc3_frame_duration_us;
static int lc3_frame_duration_100us;
static int lc3_frames_per_sdu;
static int lc3_octets_per_frame;

static void clear_lc3_sine_data(struct bt_bap_stream *bap_stream)
{
	struct shell_stream *sh_stream = shell_stream_from_bap_stream(bap_stream);

	sh_stream->tx_active = false;
	(void)k_work_cancel_delayable(&sh_stream->audio_send_work);
}

/**
 * Use the math lib to generate a sine-wave using 16 bit samples into a buffer.
 *
 * @param buf Destination buffer
 * @param length_us Length of the buffer in microseconds
 * @param frequency_hz frequency in Hz
 * @param sample_rate_hz sample-rate in Hz.
 */
static void fill_audio_buf_sin(int16_t *buf, int length_us, int frequency_hz, int sample_rate_hz)
{
	const uint32_t sine_period_samples = sample_rate_hz / frequency_hz;
	const size_t num_samples = (length_us * sample_rate_hz) / USEC_PER_SEC;
	const float step = 2 * 3.1415 / sine_period_samples;

	for (size_t i = 0; i < num_samples; i++) {
		const float sample = sin(i * step);

		buf[i] = (int16_t)(AUDIO_VOLUME * sample);
	}
}

static void init_lc3(const struct bt_bap_stream *stream)
{
	size_t num_samples;

	if (stream == NULL || stream->codec_cfg == NULL) {
		shell_error(ctx_shell, "invalid stream to init LC3");
		return;
	}

	lc3_freq_hz = bt_audio_codec_cfg_get_freq(stream->codec_cfg);
	lc3_frame_duration_us = bt_audio_codec_cfg_get_frame_duration_us(stream->codec_cfg);
	lc3_octets_per_frame = bt_audio_codec_cfg_get_octets_per_frame(stream->codec_cfg);
	lc3_frames_per_sdu = bt_audio_codec_cfg_get_frame_blocks_per_sdu(stream->codec_cfg, true);
	lc3_octets_per_frame = bt_audio_codec_cfg_get_octets_per_frame(stream->codec_cfg);

	if (lc3_freq_hz < 0) {
		printk("Error: Codec frequency not set, cannot start codec.");
		return;
	}

	if (lc3_frame_duration_us < 0) {
		printk("Error: Frame duration not set, cannot start codec.");
		return;
	}

	if (lc3_octets_per_frame < 0) {
		printk("Error: Octets per frame not set, cannot start codec.");
		return;
	}

	lc3_frame_duration_100us = lc3_frame_duration_us / 100;

	/* Fill audio buffer with Sine wave only once and repeat encoding the same tone frame */
	fill_audio_buf_sin(audio_buf, lc3_frame_duration_us, AUDIO_TONE_FREQUENCY_HZ, lc3_freq_hz);

	num_samples = ((lc3_frame_duration_us * lc3_freq_hz) / USEC_PER_SEC);
	for (size_t i = 0; i < num_samples; i++) {
		printk("%zu: %6i\n", i, audio_buf[i]);
	}

	/* Create the encoder instance. This shall complete before stream_started() is called. */
	lc3_encoder = lc3_setup_encoder(lc3_frame_duration_us, lc3_freq_hz, 0, /* No resampling */
					&lc3_encoder_mem);

	if (lc3_encoder == NULL) {
		printk("ERROR: Failed to setup LC3 encoder - wrong parameters?\n");
	}
}

static void lc3_audio_send_data(struct k_work *work)
{
	struct shell_stream *sh_stream = CONTAINER_OF(k_work_delayable_from_work(work),
						      struct shell_stream, audio_send_work);
	struct bt_bap_stream *bap_stream = &sh_stream->stream.bap_stream;
	const uint16_t tx_sdu_len = lc3_frames_per_sdu * lc3_octets_per_frame;
	struct net_buf *buf;
	uint8_t *net_buffer;
	off_t offset = 0;
	int err;

	if (!sh_stream->tx_active) {
		/* TX has been aborted */
		return;
	}

	if (lc3_encoder == NULL) {
		shell_error(ctx_shell, "LC3 encoder not setup, cannot encode data");
		return;
	}

	if (bap_stream == NULL || bap_stream->qos == NULL) {
		shell_error(ctx_shell, "invalid stream, aborting");
		return;
	}

	if (atomic_get(&sh_stream->lc3_enqueue_cnt) == 0U) {
		shell_error(ctx_shell, "Stream %p enqueue count was 0", bap_stream);

		/* Reschedule for next interval */
		k_work_reschedule(k_work_delayable_from_work(work),
				  K_USEC(bap_stream->qos->interval));
		return;
	}

	buf = net_buf_alloc(&sine_tx_pool, K_FOREVER);
	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

	net_buffer = net_buf_tail(buf);
	buf->len += tx_sdu_len;

	for (int i = 0; i < lc3_frames_per_sdu; i++) {
		int lc3_ret;

		lc3_ret = lc3_encode(lc3_encoder, LC3_PCM_FORMAT_S16, audio_buf, 1,
				     lc3_octets_per_frame, net_buffer + offset);
		offset += lc3_octets_per_frame;

		if (lc3_ret == -1) {
			shell_error(ctx_shell, "LC3 encoder failed - wrong parameters?: %d",
				    lc3_ret);
			net_buf_unref(buf);

			/* Reschedule for next interval */
			k_work_reschedule(k_work_delayable_from_work(work),
					  K_USEC(bap_stream->qos->interval));
			return;
		}
	}

	err = bt_bap_stream_send(bap_stream, buf, sh_stream->seq_num, BT_ISO_TIMESTAMP_NONE);
	if (err < 0) {
		shell_error(ctx_shell, "Failed to send LC3 audio data (%d)", err);
		net_buf_unref(buf);

		/* Reschedule for next interval */
		k_work_reschedule(k_work_delayable_from_work(work),
				  K_USEC(bap_stream->qos->interval));
		return;
	}

	if ((sh_stream->lc3_sdu_cnt % 100) == 0) {
		shell_info(ctx_shell, "[%zu]: stream %p : TX LC3: %zu (seq_num %u)",
			   sh_stream->lc3_sdu_cnt, bap_stream, tx_sdu_len, sh_stream->seq_num);
	}

	sh_stream->lc3_sdu_cnt++;
	sh_stream->seq_num++;
	atomic_dec(&sh_stream->lc3_enqueue_cnt);

	if (atomic_get(&sh_stream->lc3_enqueue_cnt) > 0) {
		/* If we have more buffers available, we reschedule the workqueue item immediately
		 * to trigger another encode + TX, but without blocking this call for too long
		 */
		k_work_reschedule(k_work_delayable_from_work(work), K_NO_WAIT);
	}
}

void sdu_sent_cb(struct bt_bap_stream *bap_stream)
{
	struct shell_stream *sh_stream = shell_stream_from_bap_stream(bap_stream);
	int err;

	atomic_inc(&sh_stream->lc3_enqueue_cnt);

	if (!sh_stream->tx_active) {
		/* TX has been aborted */
		return;
	}

	err = k_work_schedule(&sh_stream->audio_send_work, K_NO_WAIT);
	if (err < 0) {
		shell_error(ctx_shell, "Failed to schedule TX for stream %p: %d", bap_stream, err);
	}
}
#endif /* CONFIG_LIBLC3 && CONFIG_BT_AUDIO_TX */

static const struct named_lc3_preset *get_named_preset(bool is_unicast, const char *preset_arg)
{
	if (is_unicast) {
		for (size_t i = 0U; i < ARRAY_SIZE(lc3_unicast_presets); i++) {
			if (!strcmp(preset_arg, lc3_unicast_presets[i].name)) {
				return &lc3_unicast_presets[i];
			}
		}
	} else {
		for (size_t i = 0U; i < ARRAY_SIZE(lc3_broadcast_presets); i++) {
			if (!strcmp(preset_arg, lc3_broadcast_presets[i].name)) {
				return &lc3_broadcast_presets[i];
			}
		}
	}

	return NULL;
}

static void set_unicast_stream(struct bt_bap_stream *stream)
{
	default_stream = stream;

	for (size_t i = 0U; i < ARRAY_SIZE(unicast_streams); i++) {
		if (stream == &unicast_streams[i].stream.bap_stream) {
			shell_print(ctx_shell, "Default stream: %u", i + 1);
		}
	}
}

static int cmd_select_unicast(const struct shell *sh, size_t argc, char *argv[])
{
	struct bt_bap_stream *stream;
	unsigned long index;
	int err = 0;

	index = shell_strtoul(argv[1], 0, &err);
	if (err != 0) {
		shell_error(sh, "Could not parse index: %d", err);

		return -ENOEXEC;
	}

	if (index > ARRAY_SIZE(unicast_streams)) {
		shell_error(sh, "Invalid index: %lu", index);

		return -ENOEXEC;
	}

	stream = &unicast_streams[index].stream.bap_stream;

	set_unicast_stream(stream);

	return 0;
}

static struct bt_bap_stream *stream_alloc(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(unicast_streams); i++) {
		struct bt_bap_stream *stream = &unicast_streams[i].stream.bap_stream;

		if (!stream->conn) {
			return stream;
		}
	}

	return NULL;
}

static int lc3_config(struct bt_conn *conn, const struct bt_bap_ep *ep, enum bt_audio_dir dir,
		      const struct bt_audio_codec_cfg *codec_cfg, struct bt_bap_stream **stream,
		      struct bt_audio_codec_qos_pref *const pref, struct bt_bap_ascs_rsp *rsp)
{
	shell_print(ctx_shell, "ASE Codec Config: conn %p ep %p dir %u", conn, ep, dir);

	print_codec_cfg(ctx_shell, codec_cfg);

	*stream = stream_alloc();
	if (*stream == NULL) {
		shell_print(ctx_shell, "No unicast_streams available");

		*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_NO_MEM, BT_BAP_ASCS_REASON_NONE);

		return -ENOMEM;
	}

	shell_print(ctx_shell, "ASE Codec Config stream %p", *stream);

	set_unicast_stream(*stream);

	*pref = qos_pref;

	return 0;
}

static int lc3_reconfig(struct bt_bap_stream *stream, enum bt_audio_dir dir,
			const struct bt_audio_codec_cfg *codec_cfg,
			struct bt_audio_codec_qos_pref *const pref, struct bt_bap_ascs_rsp *rsp)
{
	shell_print(ctx_shell, "ASE Codec Reconfig: stream %p", stream);

	print_codec_cfg(ctx_shell, codec_cfg);

	if (default_stream == NULL) {
		set_unicast_stream(stream);
	}

	*pref = qos_pref;

	return 0;
}

static int lc3_qos(struct bt_bap_stream *stream, const struct bt_audio_codec_qos *qos,
		   struct bt_bap_ascs_rsp *rsp)
{
	shell_print(ctx_shell, "QoS: stream %p %p", stream, qos);

	print_qos(ctx_shell, qos);

	return 0;
}

static int lc3_enable(struct bt_bap_stream *stream, const struct bt_audio_codec_data *meta,
		      size_t meta_count, struct bt_bap_ascs_rsp *rsp)
{
	shell_print(ctx_shell, "Enable: stream %p meta_count %zu", stream,
		    meta_count);

	return 0;
}

static int lc3_start(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	shell_print(ctx_shell, "Start: stream %p", stream);

	return 0;
}


static bool valid_metadata_type(uint8_t type, uint8_t len)
{
	switch (type) {
	case BT_AUDIO_METADATA_TYPE_PREF_CONTEXT:
	case BT_AUDIO_METADATA_TYPE_STREAM_CONTEXT:
		if (len != 2) {
			return false;
		}

		return true;
	case BT_AUDIO_METADATA_TYPE_STREAM_LANG:
		if (len != 3) {
			return false;
		}

		return true;
	case BT_AUDIO_METADATA_TYPE_PARENTAL_RATING:
		if (len != 1) {
			return false;
		}

		return true;
	case BT_AUDIO_METADATA_TYPE_EXTENDED: /* 2 - 255 octets */
	case BT_AUDIO_METADATA_TYPE_VENDOR: /* 2 - 255 octets */
		/* At least Extended Metadata Type / Company_ID should be there */
		if (len < 2) {
			return false;
		}

		return true;
	case BT_AUDIO_METADATA_TYPE_CCID_LIST:
	case BT_AUDIO_METADATA_TYPE_PROGRAM_INFO: /* 0 - 255 octets */
	case BT_AUDIO_METADATA_TYPE_PROGRAM_INFO_URI: /* 0 - 255 octets */
		return true;
	default:
		return false;
	}
}

static int lc3_metadata(struct bt_bap_stream *stream, const struct bt_audio_codec_data *meta,
			size_t meta_count, struct bt_bap_ascs_rsp *rsp)
{
	shell_print(ctx_shell, "Metadata: stream %p meta_count %zu", stream,
		    meta_count);

	for (size_t i = 0; i < meta_count; i++) {
		const struct bt_audio_codec_data *data = &meta[i];

		if (!valid_metadata_type(data->data.type, data->data.data_len)) {
			shell_print(ctx_shell,
				    "Invalid metadata type %u or length %u",
				    data->data.type, data->data.data_len);
			*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_METADATA_REJECTED,
					       data->data.type);
			return -EINVAL;
		}
	}

	return 0;
}

static int lc3_disable(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	shell_print(ctx_shell, "Disable: stream %p", stream);

	return 0;
}

static int lc3_stop(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	shell_print(ctx_shell, "Stop: stream %p", stream);

	return 0;
}

static int lc3_release(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	shell_print(ctx_shell, "Release: stream %p", stream);

	if (stream == default_stream) {
		default_stream = NULL;
	}

	return 0;
}

static struct bt_audio_codec_cap lc3_codec_cap =
	BT_AUDIO_CODEC_LC3(BT_AUDIO_CODEC_LC3_FREQ_ANY, BT_AUDIO_CODEC_LC3_DURATION_ANY,
			   BT_AUDIO_CODEC_LC3_CHAN_COUNT_SUPPORT(1, 2), 30, 240, 2,
			   (BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | BT_AUDIO_CONTEXT_TYPE_MEDIA));

static const struct bt_bap_unicast_server_cb unicast_server_cb = {
	.config = lc3_config,
	.reconfig = lc3_reconfig,
	.qos = lc3_qos,
	.enable = lc3_enable,
	.start = lc3_start,
	.metadata = lc3_metadata,
	.disable = lc3_disable,
	.stop = lc3_stop,
	.release = lc3_release,
};

static struct bt_pacs_cap cap_sink = {
	.codec_cap = &lc3_codec_cap,
};

static struct bt_pacs_cap cap_source = {
	.codec_cap = &lc3_codec_cap,
};
#if defined(CONFIG_BT_BAP_UNICAST)

static uint16_t strmeta(const char *name)
{
	if (strcmp(name, "Unspecified") == 0) {
		return BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED;
	} else if (strcmp(name, "Conversational") == 0) {
		return BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL;
	} else if (strcmp(name, "Media") == 0) {
		return BT_AUDIO_CONTEXT_TYPE_MEDIA;
	} else if (strcmp(name, "Game") == 0) {
		return BT_AUDIO_CONTEXT_TYPE_GAME;
	} else if (strcmp(name, "Instructional") == 0) {
		return BT_AUDIO_CONTEXT_TYPE_INSTRUCTIONAL;
	} else if (strcmp(name, "VoiceAssistants") == 0) {
		return BT_AUDIO_CONTEXT_TYPE_VOICE_ASSISTANTS;
	} else if (strcmp(name, "Live") == 0) {
		return BT_AUDIO_CONTEXT_TYPE_LIVE;
	} else if (strcmp(name, "SoundEffects") == 0) {
		return BT_AUDIO_CONTEXT_TYPE_SOUND_EFFECTS;
	} else if (strcmp(name, "Notifications") == 0) {
		return BT_AUDIO_CONTEXT_TYPE_NOTIFICATIONS;
	} else if (strcmp(name, "Ringtone") == 0) {
		return BT_AUDIO_CONTEXT_TYPE_RINGTONE;
	} else if (strcmp(name, "Alerts") == 0) {
		return BT_AUDIO_CONTEXT_TYPE_ALERTS;
	} else if (strcmp(name, "EmergencyAlarm") == 0) {
		return BT_AUDIO_CONTEXT_TYPE_EMERGENCY_ALARM;
	}

	return 0u;
}

static int set_metadata(struct bt_audio_codec_cfg *codec_cfg, const char *meta_str)
{
	uint16_t context;

	context = strmeta(meta_str);
	if (context == 0) {
		return -ENOEXEC;
	}

	/* TODO: Check the type and only overwrite the streaming context */
	sys_put_le16(context, codec_cfg->meta[0].value);

	return 0;
}

#if defined(CONFIG_BT_BAP_UNICAST_CLIENT)
static uint8_t stream_dir(const struct bt_bap_stream *stream)
{
	if (stream->conn) {
		uint8_t conn_index = bt_conn_index(stream->conn);

#if CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SNK_COUNT > 0
		for (size_t i = 0; i < ARRAY_SIZE(snks[conn_index]); i++) {
			const struct bt_bap_ep *snk_ep = snks[conn_index][i];

			if (snk_ep != NULL && stream->ep == snk_ep) {
				return BT_AUDIO_DIR_SINK;
			}
		}
#endif /* CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SNK_COUNT > 0 */

#if CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SRC_COUNT > 0
		for (size_t i = 0; i < ARRAY_SIZE(srcs[conn_index]); i++) {
			const struct bt_bap_ep *src_ep = srcs[conn_index][i];

			if (src_ep != NULL && stream->ep == src_ep) {
				return BT_AUDIO_DIR_SOURCE;
			}
		}
#endif /* CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SRC_COUNT > 0 */
	}

	__ASSERT(false, "Invalid stream");
	return 0;
}

static void print_remote_codec_cap(const struct bt_conn *conn,
				   const struct bt_audio_codec_cap *codec_cap,
				   enum bt_audio_dir dir)
{
	shell_print(ctx_shell, "conn %p: codec_cap %p dir 0x%02x", conn, codec_cap,
		    dir);

	print_codec_cap(ctx_shell, codec_cap);
}

#if CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SNK_COUNT > 0
static void add_sink(const struct bt_conn *conn, struct bt_bap_ep *ep)
{
	const uint8_t conn_index = bt_conn_index(conn);

	for (size_t i = 0U; i < ARRAY_SIZE(snks[conn_index]); i++) {
		if (snks[conn_index][i] == NULL) {
			shell_print(ctx_shell, "Conn: %p, Sink #%zu: ep %p", conn, i, ep);

			snks[conn_index][i] = ep;
			return;
		}
	}

	shell_error(ctx_shell, "Could not add more sink endpoints");
}
#endif /* CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SNK_COUNT > 0 */

#if CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SRC_COUNT > 0
static void add_source(const struct bt_conn *conn, struct bt_bap_ep *ep)
{
	const uint8_t conn_index = bt_conn_index(conn);

	for (size_t i = 0U; i < ARRAY_SIZE(srcs[conn_index]); i++) {
		if (srcs[conn_index][i] == NULL) {
			shell_print(ctx_shell, "Conn: %p, Source #%zu: ep %p", conn, i, ep);

			srcs[conn_index][i] = ep;
			return;
		}
	}

	shell_error(ctx_shell, "Could not add more sink endpoints");
}
#endif /* CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SRC_COUNT > 0 */

static void pac_record_cb(struct bt_conn *conn, enum bt_audio_dir dir,
			  const struct bt_audio_codec_cap *codec_cap)
{
	print_remote_codec_cap(conn, codec_cap, dir);
}

static void endpoint_cb(struct bt_conn *conn, enum bt_audio_dir dir, struct bt_bap_ep *ep)
{
#if CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SNK_COUNT > 0
	if (dir == BT_AUDIO_DIR_SINK) {
		add_sink(conn, ep);
	}
#endif /* CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SNK_COUNT > 0 */

#if CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SRC_COUNT > 0
	if (dir == BT_AUDIO_DIR_SOURCE) {
		add_source(conn, ep);
	}
#endif /* CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SRC_COUNT > 0*/
}

static void discover_cb(struct bt_conn *conn, int err, enum bt_audio_dir dir)
{
	shell_print(ctx_shell, "Discover complete: err %d", err);
}

static void discover_all(struct bt_conn *conn, int err, enum bt_audio_dir dir)
{
	/* Sinks discovery complete, now discover sources */
	if (dir == BT_AUDIO_DIR_SINK) {
		dir = BT_AUDIO_DIR_SOURCE;
		unicast_client_cbs.discover = discover_cb;

		err = bt_bap_unicast_client_discover(default_conn, dir);
		if (err) {
			shell_error(ctx_shell, "bt_bap_unicast_client_discover err %d", err);
		}
	}
}

static void unicast_client_location_cb(struct bt_conn *conn,
				      enum bt_audio_dir dir,
				      enum bt_audio_location loc)
{
	shell_print(ctx_shell, "dir %u loc %X\n", dir, loc);
}

static void available_contexts_cb(struct bt_conn *conn,
				  enum bt_audio_context snk_ctx,
				  enum bt_audio_context src_ctx)
{
	shell_print(ctx_shell, "snk ctx %u src ctx %u\n", snk_ctx, src_ctx);
}

static void config_cb(struct bt_bap_stream *stream, enum bt_bap_ascs_rsp_code rsp_code,
		      enum bt_bap_ascs_reason reason)
{
	shell_print(ctx_shell, "stream %p config operation rsp_code %u reason %u",
		    stream, rsp_code, reason);

	if (default_stream == NULL) {
		default_stream = stream;
	}
}

static void qos_cb(struct bt_bap_stream *stream, enum bt_bap_ascs_rsp_code rsp_code,
		   enum bt_bap_ascs_reason reason)
{
	shell_print(ctx_shell, "stream %p qos operation rsp_code %u reason %u",
		    stream, rsp_code, reason);
}

static void enable_cb(struct bt_bap_stream *stream, enum bt_bap_ascs_rsp_code rsp_code,
		      enum bt_bap_ascs_reason reason)
{
	shell_print(ctx_shell, "stream %p enable operation rsp_code %u reason %u",
		    stream, rsp_code, reason);
}

static void start_cb(struct bt_bap_stream *stream, enum bt_bap_ascs_rsp_code rsp_code,
		     enum bt_bap_ascs_reason reason)
{
	shell_print(ctx_shell, "stream %p start operation rsp_code %u reason %u",
		    stream, rsp_code, reason);
}

static void stop_cb(struct bt_bap_stream *stream, enum bt_bap_ascs_rsp_code rsp_code,
		    enum bt_bap_ascs_reason reason)
{
	shell_print(ctx_shell, "stream %p stop operation rsp_code %u reason %u",
		    stream, rsp_code, reason);
}

static void disable_cb(struct bt_bap_stream *stream, enum bt_bap_ascs_rsp_code rsp_code,
		       enum bt_bap_ascs_reason reason)
{
	shell_print(ctx_shell, "stream %p disable operation rsp_code %u reason %u",
		    stream, rsp_code, reason);
}

static void metadata_cb(struct bt_bap_stream *stream, enum bt_bap_ascs_rsp_code rsp_code,
			enum bt_bap_ascs_reason reason)
{
	shell_print(ctx_shell, "stream %p metadata operation rsp_code %u reason %u",
		    stream, rsp_code, reason);
}

static void release_cb(struct bt_bap_stream *stream, enum bt_bap_ascs_rsp_code rsp_code,
		       enum bt_bap_ascs_reason reason)
{
	shell_print(ctx_shell, "stream %p release operation rsp_code %u reason %u",
		    stream, rsp_code, reason);
}

static struct bt_bap_unicast_client_cb unicast_client_cbs = {
	.location = unicast_client_location_cb,
	.available_contexts = available_contexts_cb,
	.config = config_cb,
	.qos = qos_cb,
	.enable = enable_cb,
	.start = start_cb,
	.stop = stop_cb,
	.disable = disable_cb,
	.metadata = metadata_cb,
	.release = release_cb,
	.pac_record = pac_record_cb,
	.endpoint = endpoint_cb,
};

static int cmd_discover(const struct shell *sh, size_t argc, char *argv[])
{
	static bool cbs_registered;
	enum bt_audio_dir dir;
	uint8_t conn_index;
	int err;

	if (!default_conn) {
		shell_error(sh, "Not connected");
		return -ENOEXEC;
	}

	if (!initialized) {
		shell_error(sh, "Not initialized");
		return -ENOEXEC;
	}

	if (!cbs_registered) {
		int err = bt_bap_unicast_client_register_cb(&unicast_client_cbs);

		if (err != 0) {
			shell_error(sh, "Failed to register unicast client callbacks: %d", err);
			return err;
		}

		cbs_registered = true;
	}

	unicast_client_cbs.discover = discover_all;
	dir = BT_AUDIO_DIR_SINK;

	if (argc > 1) {
		if (!strcmp(argv[1], "sink")) {
			unicast_client_cbs.discover = discover_cb;
		} else if (!strcmp(argv[1], "source")) {
			unicast_client_cbs.discover = discover_cb;
			dir = BT_AUDIO_DIR_SOURCE;
		} else {
			shell_error(sh, "Unsupported dir: %s", argv[1]);
			return -ENOEXEC;
		}
	}

	err = bt_bap_unicast_client_discover(default_conn, dir);
	if (err != 0) {
		return -ENOEXEC;
	}

	conn_index = bt_conn_index(default_conn);
#if CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SRC_COUNT > 0
	memset(srcs[conn_index], 0, sizeof(srcs[conn_index]));
#endif /* CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SRC_COUNT > 0 */

#if CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SNK_COUNT > 0
	memset(snks[conn_index], 0, sizeof(snks[conn_index]));
#endif /* CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SNK_COUNT > 0 */

	return 0;
}

static int cmd_config(const struct shell *sh, size_t argc, char *argv[])
{
	enum bt_audio_location location = BT_AUDIO_LOCATION_PROHIBITED;
	const struct named_lc3_preset *named_preset;
	struct shell_stream *uni_stream;
	struct bt_bap_stream *bap_stream;
	struct bt_bap_ep *ep = NULL;
	unsigned long index;
	uint8_t conn_index;
	int err = 0;

	if (!default_conn) {
		shell_error(sh, "Not connected");
		return -ENOEXEC;
	}
	conn_index = bt_conn_index(default_conn);

	if (default_stream == NULL) {
		bap_stream = &unicast_streams[0].stream.bap_stream;
	} else {
		bap_stream = default_stream;
	}

	index = shell_strtoul(argv[2], 0, &err);
	if (err != 0) {
		shell_error(sh, "Could not parse index: %d", err);

		return -ENOEXEC;
	}

	if (index > ARRAY_SIZE(unicast_streams)) {
		shell_error(sh, "Invalid index: %lu", index);

		return -ENOEXEC;
	}

	if (false) {

#if CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SNK_COUNT > 0
	} else if (!strcmp(argv[1], "sink")) {
		ep = snks[conn_index][index];

		named_preset = default_sink_preset;
#endif /* CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SNK_COUNT > 0 */

#if CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SRC_COUNT > 0
	} else if (!strcmp(argv[1], "source")) {
		ep = srcs[conn_index][index];

		named_preset = default_source_preset;
#endif /* CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SRC_COUNT > 0 */
	} else {
		shell_error(sh, "Unsupported dir: %s", argv[1]);
		return -ENOEXEC;
	}

	if (!ep) {
		shell_error(sh, "Unable to find endpoint");
		return -ENOEXEC;
	}

	for (size_t i = 3U; i < argc; i++) {
		const char *arg = argv[i];

		/* argc needs to be larger than `i` to parse the argument value */
		if (argc <= i) {
			shell_help(sh);

			return SHELL_CMD_HELP_PRINTED;
		}

		if (strcmp(arg, "loc") == 0) {
			unsigned long loc_bits;

			arg = argv[++i];
			loc_bits = shell_strtoul(arg, 0, &err);
			if (err != 0) {
				shell_error(sh, "Could not parse loc_bits: %d", err);

				return -ENOEXEC;
			}

			if (loc_bits == BT_AUDIO_LOCATION_PROHIBITED ||
			    loc_bits > BT_AUDIO_LOCATION_ANY) {
				shell_error(sh, "Invalid loc_bits: %lu", loc_bits);

				return -ENOEXEC;
			}

			location = (enum bt_audio_location)loc_bits;
		} else if (strcmp(arg, "preset") == 0) {
			if (argc > i) {
				arg = argv[++i];

				named_preset = get_named_preset(true, arg);
				if (named_preset == NULL) {
					shell_error(sh, "Unable to parse named_preset %s", arg);
					return -ENOEXEC;
				}
			} else {
				shell_help(sh);

				return SHELL_CMD_HELP_PRINTED;
			}
		} else {
			shell_help(sh);

			return SHELL_CMD_HELP_PRINTED;
		}
	}

	uni_stream = shell_stream_from_bap_stream(bap_stream);
	copy_unicast_stream_preset(uni_stream, named_preset);

	/* If location has been modifed, we update the location in the codec configuration */
	if (location != BT_AUDIO_LOCATION_PROHIBITED) {
		for (size_t i = 0U; i < uni_stream->codec_cfg.data_count; i++) {
			struct bt_audio_codec_data *data = &uni_stream->codec_cfg.data[i];

			/* Overwrite the location value */
			if (data->data.type == BT_AUDIO_CODEC_CONFIG_LC3_CHAN_ALLOC) {
				const uint32_t loc_32 = location;

				sys_put_le32(loc_32, data->value);

				shell_print(sh, "Setting location to 0x%08X", location);
				break;
			}
		}
	}

	if (bap_stream->ep == ep) {
		err = bt_bap_stream_reconfig(bap_stream, &uni_stream->codec_cfg);
		if (err != 0) {
			shell_error(sh, "Unable reconfig stream: %d", err);
			return -ENOEXEC;
		}
	} else {
		err = bt_bap_stream_config(default_conn, bap_stream, ep,
					   &uni_stream->codec_cfg);
		if (err != 0) {
			shell_error(sh, "Unable to config stream: %d", err);
			return err;
		}
	}

	shell_print(sh, "ASE config: preset %s", named_preset->name);

	return 0;
}

static int cmd_stream_qos(const struct shell *sh, size_t argc, char *argv[])
{
	struct bt_audio_codec_qos *qos;
	unsigned long interval;
	int err;

	if (default_stream == NULL) {
		shell_print(sh, "No stream selected");
		return -ENOEXEC;
	}

	qos = default_stream->qos;

	if (qos == NULL) {
		shell_print(sh, "Stream not configured");
		return -ENOEXEC;
	}

	interval = shell_strtoul(argv[1], 0, &err);
	if (err != 0) {
		return -ENOEXEC;
	}

	if (!IN_RANGE(interval, BT_ISO_SDU_INTERVAL_MIN, BT_ISO_SDU_INTERVAL_MAX)) {
		return -ENOEXEC;
	}

	qos->interval = interval;

	if (argc > 2) {
		unsigned long framing;

		framing = shell_strtoul(argv[2], 0, &err);
		if (err != 0) {
			return -ENOEXEC;
		}

		if (framing != BT_ISO_FRAMING_UNFRAMED && framing != BT_ISO_FRAMING_FRAMED) {
			return -ENOEXEC;
		}

		qos->framing = framing;
	}

	if (argc > 3) {
		unsigned long latency;

		latency = shell_strtoul(argv[3], 0, &err);
		if (err != 0) {
			return -ENOEXEC;
		}

		if (!IN_RANGE(latency, BT_ISO_LATENCY_MIN, BT_ISO_LATENCY_MAX)) {
			return -ENOEXEC;
		}

		qos->latency = latency;
	}

	if (argc > 4) {
		unsigned long pd;

		pd = shell_strtoul(argv[4], 0, &err);
		if (err != 0) {
			return -ENOEXEC;
		}

		if (pd > BT_AUDIO_PD_MAX) {
			return -ENOEXEC;
		}

		qos->pd = pd;
	}

	if (argc > 5) {
		unsigned long sdu;

		sdu = shell_strtoul(argv[5], 0, &err);
		if (err != 0) {
			return -ENOEXEC;
		}

		if (sdu > BT_ISO_MAX_SDU) {
			return -ENOEXEC;
		}

		qos->sdu = sdu;
	}

	if (argc > 6) {
		unsigned long phy;

		phy = shell_strtoul(argv[6], 0, &err);
		if (err != 0) {
			return -ENOEXEC;
		}

		if (phy != BT_GAP_LE_PHY_1M && phy != BT_GAP_LE_PHY_2M &&
		    phy != BT_GAP_LE_PHY_CODED) {
			return -ENOEXEC;
		}

		qos->phy = phy;
	}

	if (argc > 7) {
		unsigned long rtn;

		rtn = shell_strtoul(argv[7], 0, &err);
		if (err != 0) {
			return -ENOEXEC;
		}

		if (rtn > BT_ISO_CONNECTED_RTN_MAX) {
			return -ENOEXEC;
		}

		qos->rtn = rtn;
	}

	return 0;
}

static int create_unicast_group(const struct shell *sh)
{
	struct bt_bap_unicast_group_stream_pair_param pair_param[ARRAY_SIZE(unicast_streams)];
	struct bt_bap_unicast_group_stream_param stream_params[ARRAY_SIZE(unicast_streams)];
	struct bt_bap_unicast_group_param group_param;
	size_t source_cnt = 0;
	size_t sink_cnt = 0;
	size_t cnt = 0;
	int err;

	memset(pair_param, 0, sizeof(pair_param));
	memset(stream_params, 0, sizeof(stream_params));
	memset(&group_param, 0, sizeof(group_param));

	for (size_t i = 0U; i < ARRAY_SIZE(unicast_streams); i++) {
		struct bt_bap_stream *stream = &unicast_streams[i].stream.bap_stream;
		struct shell_stream *uni_stream = &unicast_streams[i];

		if (stream->ep != NULL) {
			struct bt_bap_unicast_group_stream_param *stream_param;

			stream_param = &stream_params[cnt];

			stream_param->stream = stream;
			if (stream_dir(stream) == BT_AUDIO_DIR_SINK) {
				stream_param->qos = &uni_stream->qos;
				pair_param[sink_cnt++].tx_param = stream_param;
			} else {
				stream_param->qos = &uni_stream->qos;
				pair_param[source_cnt++].rx_param = stream_param;
			}

			cnt++;
		}
	}

	if (cnt == 0U) {
		shell_error(sh, "Stream cnt is 0");

		return -ENOEXEC;
	}

	group_param.packing = BT_ISO_PACKING_SEQUENTIAL;
	group_param.params = pair_param;
	group_param.params_count = MAX(source_cnt, sink_cnt);

	err = bt_bap_unicast_group_create(&group_param,
					    &default_unicast_group);
	if (err != 0) {
		shell_error(sh,
			    "Unable to create default unicast group: %d",
			    err);

		return -ENOEXEC;
	}

	return 0;
}

static int cmd_qos(const struct shell *sh, size_t argc, char *argv[])
{
	int err;

	if (default_stream == NULL) {
		shell_print(sh, "No stream selected");
		return -ENOEXEC;
	}

	if (default_conn == NULL) {
		shell_error(sh, "Not connected");
		return -ENOEXEC;
	}

	if (default_unicast_group == NULL) {
		err = create_unicast_group(sh);
		if (err != 0) {
			return err;
		}
	}

	err = bt_bap_stream_qos(default_conn, default_unicast_group);
	if (err) {
		shell_error(sh, "Unable to setup QoS: %d", err);
		return -ENOEXEC;
	}

	return 0;
}

static int cmd_enable(const struct shell *sh, size_t argc, char *argv[])
{
	struct bt_audio_codec_cfg *codec_cfg;
	int err;

	if (default_stream == NULL) {
		shell_error(sh, "No stream selected");
		return -ENOEXEC;
	}

	codec_cfg = default_stream->codec_cfg;

	if (argc > 1) {
		err = set_metadata(codec_cfg, argv[1]);
		if (err != 0) {
			shell_error(sh, "Unable to handle metadata update: %d", err);
			return err;
		}
	}

	err = bt_bap_stream_enable(default_stream, codec_cfg->meta, codec_cfg->meta_count);
	if (err) {
		shell_error(sh, "Unable to enable Channel");
		return -ENOEXEC;
	}

	return 0;
}

static int cmd_stop(const struct shell *sh, size_t argc, char *argv[])
{
	int err;

	if (default_stream == NULL) {
		shell_error(sh, "No stream selected");
		return -ENOEXEC;
	}

	err = bt_bap_stream_stop(default_stream);
	if (err) {
		shell_error(sh, "Unable to stop Channel");
		return -ENOEXEC;
	}

	return 0;
}
#endif /* CONFIG_BT_BAP_UNICAST_CLIENT */

static int cmd_preset(const struct shell *sh, size_t argc, char *argv[])
{
	const struct named_lc3_preset *named_preset;
	bool unicast = true;

	if (!strcmp(argv[1], "sink")) {
		named_preset = default_sink_preset;
	} else if (!strcmp(argv[1], "source")) {
		named_preset = default_source_preset;
	} else if (!strcmp(argv[1], "broadcast")) {
		unicast = false;

		named_preset = default_broadcast_source_preset;
	} else {
		shell_error(sh, "Unsupported dir: %s", argv[1]);
		return -ENOEXEC;
	}

	if (argc > 2) {
		named_preset = get_named_preset(unicast, argv[2]);
		if (named_preset == NULL) {
			shell_error(sh, "Unable to parse named_preset %s", argv[2]);
			return -ENOEXEC;
		}

		if (!strcmp(argv[1], "sink")) {
			default_sink_preset = named_preset;
		} else if (!strcmp(argv[1], "source")) {
			default_source_preset = named_preset;
		} else if (!strcmp(argv[1], "broadcast")) {
			default_broadcast_source_preset = named_preset;
		}
	}

	shell_print(sh, "%s", named_preset->name);

	print_codec_cfg(ctx_shell, &named_preset->preset.codec_cfg);
	print_qos(ctx_shell, &named_preset->preset.qos);

	return 0;
}

#define MAX_META_DATA                                                                              \
	(CONFIG_BT_AUDIO_CODEC_CFG_MAX_METADATA_COUNT * sizeof(struct bt_audio_codec_data))

static int cmd_metadata(const struct shell *sh, size_t argc, char *argv[])
{
	struct bt_audio_codec_cfg *codec_cfg;
	int err;

	if (default_stream == NULL) {
		shell_error(sh, "No stream selected");
		return -ENOEXEC;
	}

	codec_cfg = default_stream->codec_cfg;

	if (argc > 1) {
		err = set_metadata(codec_cfg, argv[1]);
		if (err != 0) {
			shell_error(sh, "Unable to handle metadata update: %d", err);
			return err;
		}
	}

	err = bt_bap_stream_metadata(default_stream, codec_cfg->meta, codec_cfg->meta_count);
	if (err) {
		shell_error(sh, "Unable to set Channel metadata");
		return -ENOEXEC;
	}

	return 0;
}

static int cmd_start(const struct shell *sh, size_t argc, char *argv[])
{
	int err;

	if (default_stream == NULL) {
		shell_error(sh, "No stream selected");
		return -ENOEXEC;
	}

	err = bt_bap_stream_start(default_stream);
	if (err) {
		shell_error(sh, "Unable to start Channel");
		return -ENOEXEC;
	}

	return 0;
}

static int cmd_disable(const struct shell *sh, size_t argc, char *argv[])
{
	int err;

	if (default_stream == NULL) {
		shell_error(sh, "No stream selected");
		return -ENOEXEC;
	}

	err = bt_bap_stream_disable(default_stream);
	if (err) {
		shell_error(sh, "Unable to disable Channel");
		return -ENOEXEC;
	}

	return 0;
}

#if defined(CONFIG_BT_BAP_UNICAST_CLIENT)
static void conn_list_eps(struct bt_conn *conn, void *data)
{
	const struct shell *sh = (const struct shell *)data;
	uint8_t conn_index = bt_conn_index(conn);

	shell_print(sh, "Conn: %p", conn);
	shell_print(sh, "  Sinks:");

#if CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SNK_COUNT > 0
	for (size_t i = 0U; i < ARRAY_SIZE(snks[conn_index]); i++) {
		const struct bt_bap_ep *ep = snks[conn_index][i];

		if (ep != NULL) {
			shell_print(sh, "    #%u: ep %p", i, ep);
		}
	}
#endif /* CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SNK_COUNT > 0 */

#if CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SRC_COUNT > 0
	shell_print(sh, "  Sources:");

	for (size_t i = 0U; i < ARRAY_SIZE(srcs[conn_index]); i++) {
		const struct bt_bap_ep *ep = srcs[conn_index][i];

		if (ep != NULL) {
			shell_print(sh, "    #%u: ep %p", i, ep);
		}
	}
#endif /* CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SRC_COUNT > 0 */
}
#endif /* CONFIG_BT_BAP_UNICAST_CLIENT */

#if defined(CONFIG_BT_BAP_UNICAST_CLIENT)
static int cmd_list(const struct shell *sh, size_t argc, char *argv[])
{
	shell_print(sh, "Configured Channels:");

	for (size_t i = 0U; i < ARRAY_SIZE(unicast_streams); i++) {
		struct bt_bap_stream *stream = &unicast_streams[i].stream.bap_stream;

		if (stream != NULL && stream->conn != NULL) {
			shell_print(sh, "  %s#%u: stream %p dir 0x%02x group %p",
				    stream == default_stream ? "*" : " ", i, stream,
				    stream_dir(stream), stream->group);
		}
	}

	bt_conn_foreach(BT_CONN_TYPE_LE, conn_list_eps, (void *)sh);

	return 0;
}
#endif /* CONFIG_BT_BAP_UNICAST_CLIENT */

static int cmd_release(const struct shell *sh, size_t argc, char *argv[])
{
	int err;

	if (default_stream == NULL) {
		shell_print(sh, "No stream selected");
		return -ENOEXEC;
	}

	err = bt_bap_stream_release(default_stream);
	if (err) {
		shell_error(sh, "Unable to release Channel");
		return -ENOEXEC;
	}

	return 0;
}
#endif /* CONFIG_BT_BAP_UNICAST */

#if defined(CONFIG_BT_BAP_BROADCAST_SINK)
static uint32_t accepted_broadcast_id;
static struct bt_bap_base received_base;
static bool sink_syncable;

static bool broadcast_scan_recv(const struct bt_le_scan_recv_info *info,
				struct net_buf_simple *ad,
				uint32_t broadcast_id)
{
	char le_addr[BT_ADDR_LE_STR_LEN];

	if (!passes_scan_filter(info, ad)) {
		return false;
	}

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	shell_print(ctx_shell, "Found broadcaster with ID 0x%06X and addr %s",
		    broadcast_id, le_addr);

	if (broadcast_id == accepted_broadcast_id) {
		shell_print(ctx_shell, "PA syncing to broadcaster");
		accepted_broadcast_id = 0;
		return true;
	}

	return false;
}

static void pa_synced(struct bt_bap_broadcast_sink *sink,
		      struct bt_le_per_adv_sync *sync,
		      uint32_t broadcast_id)
{
	shell_print(ctx_shell,
		    "PA synced to broadcaster with ID 0x%06X as sink %p",
		    broadcast_id, sink);

	if (default_sink == NULL) {
		default_sink = sink;

		shell_print(ctx_shell, "Sink %p is set as default", sink);
	}
}

static void base_recv(struct bt_bap_broadcast_sink *sink, const struct bt_bap_base *base)
{
	uint8_t bis_indexes[BROADCAST_SNK_STREAM_CNT] = { 0 };
	/* "0xXX " requires 5 characters */
	char bis_indexes_str[5 * ARRAY_SIZE(bis_indexes) + 1];
	size_t index_count = 0;

	if (memcmp(base, &received_base, sizeof(received_base)) == 0) {
		/* Don't print duplicates */
		return;
	}

	shell_print(ctx_shell, "Received BASE from sink %p:", sink);
	shell_print(ctx_shell, "Presentation delay: %u", base->pd);

	for (int i = 0; i < base->subgroup_count; i++) {
		const struct bt_bap_base_subgroup *subgroup;

		subgroup = &base->subgroups[i];

		shell_print(ctx_shell, "%2sSubgroup[%d]:", "", i);
		print_codec_cfg(ctx_shell, &subgroup->codec_cfg);

		for (int j = 0; j < subgroup->bis_count; j++) {
			const struct bt_bap_base_bis_data *bis_data;

			bis_data = &subgroup->bis_data[j];

			shell_print(ctx_shell, "%4sBIS[%d] index 0x%02x", "", i, bis_data->index);
			bis_indexes[index_count++] = bis_data->index;

			for (int k = 0; k < bis_data->data_count; k++) {
				const struct bt_audio_codec_data *codec_data;

				codec_data = &bis_data->data[k];

				shell_print(ctx_shell, "%6sdata #%u: type 0x%02x len %u", "", i,
					    codec_data->data.type, codec_data->data.data_len);
				shell_hexdump(ctx_shell, codec_data->data.data,
					      codec_data->data.data_len -
						sizeof(codec_data->data.type));
			}
		}
	}

	memset(bis_indexes_str, 0, sizeof(bis_indexes_str));
	/* Create space separated list of indexes as hex values */
	for (int i = 0; i < index_count; i++) {
		char bis_index_str[6];

		sprintf(bis_index_str, "0x%02x ", bis_indexes[i]);

		strcat(bis_indexes_str, bis_index_str);
		shell_print(ctx_shell, "[%d]: %s", i, bis_index_str);
	}

	shell_print(ctx_shell, "Possible indexes: %s", bis_indexes_str);

	(void)memcpy(&received_base, base, sizeof(received_base));
}

static void syncable(struct bt_bap_broadcast_sink *sink, bool encrypted)
{
	if (sink_syncable) {
		return;
	}

	shell_print(ctx_shell, "Sink %p is ready to sync %s encryption",
		    sink, encrypted ? "with" : "without");
	sink_syncable = true;
}

static void scan_term(int err)
{
	shell_print(ctx_shell, "Broadcast scan was terminated: %d", err);

}

static void pa_sync_lost(struct bt_bap_broadcast_sink *sink)
{
	shell_warn(ctx_shell, "Sink %p disconnected", sink);

	if (default_sink == sink) {
		default_sink = NULL;
		sink_syncable = false;
		(void)memset(&received_base, 0, sizeof(received_base));
	}
}
#endif /* CONFIG_BT_BAP_BROADCAST_SINK */

#if defined(CONFIG_BT_BAP_BROADCAST_SINK)
static struct bt_bap_broadcast_sink_cb sink_cbs = {
	.scan_recv = broadcast_scan_recv,
	.pa_synced = pa_synced,
	.base_recv = base_recv,
	.syncable = syncable,
	.scan_term = scan_term,
	.pa_sync_lost = pa_sync_lost,
};
#endif /* CONFIG_BT_BAP_BROADCAST_SINK */

#if defined(CONFIG_BT_AUDIO_RX)
static unsigned long recv_stats_interval = 100U;

static void audio_recv(struct bt_bap_stream *stream,
		       const struct bt_iso_recv_info *info,
		       struct net_buf *buf)
{
	struct shell_stream *sh_stream = shell_stream_from_bap_stream(stream);

	sh_stream->rx_cnt++;

	if (info->ts == sh_stream->last_info.ts) {
		sh_stream->dup_ts++;
	}

	if (info->seq_num == sh_stream->last_info.seq_num) {
		sh_stream->dup_psn++;
	}

	if (info->flags & BT_ISO_FLAGS_ERROR) {
		sh_stream->err_pkts++;
	}

	if (info->flags & BT_ISO_FLAGS_LOST) {
		sh_stream->lost_pkts++;
	}

	if ((sh_stream->rx_cnt % recv_stats_interval) == 0) {
		shell_print(ctx_shell,
			    "[%zu]: Incoming audio on stream %p len %u ts %u seq_num %u flags %u "
			    "(dup ts %zu; dup psn %zu, err_pkts %zu, lost_pkts %zu)",
			    sh_stream->rx_cnt, stream, buf->len, info->ts, info->seq_num,
			    info->flags, sh_stream->dup_ts, sh_stream->dup_psn, sh_stream->err_pkts,
			    sh_stream->lost_pkts);
	}

	(void)memcpy(&sh_stream->last_info, info, sizeof(sh_stream->last_info));
}
#endif /* CONFIG_BT_AUDIO_RX */

static void stream_enabled_cb(struct bt_bap_stream *stream)
{
	shell_print(ctx_shell, "Stream %p enabled", stream);

	if (IS_ENABLED(CONFIG_BT_BAP_UNICAST_SERVER)) {
		struct bt_bap_ep_info ep_info;
		struct bt_conn_info conn_info;
		int err;

		err = bt_conn_get_info(stream->conn, &conn_info);
		if (err != 0) {
			shell_error(ctx_shell, "Failed to get conn info: %d", err);
			return;
		}

		if (conn_info.role == BT_CONN_ROLE_CENTRAL) {
			return; /* We also want to autonomously start the stream as the server */
		}

		err = bt_bap_ep_get_info(stream->ep, &ep_info);
		if (err != 0) {
			shell_error(ctx_shell, "Failed to get ep info: %d", err);
			return;
		}

		if (ep_info.dir == BT_AUDIO_DIR_SINK) {
			/* Automatically do the receiver start ready operation */
			err = bt_bap_stream_start(stream);

			if (err != 0) {
				shell_error(ctx_shell, "Failed to start stream: %d", err);
				return;
			}
		}
	}
}

static void stream_started_cb(struct bt_bap_stream *bap_stream)
{
	struct shell_stream *sh_stream = shell_stream_from_bap_stream(bap_stream);

#if defined(CONFIG_BT_AUDIO_TX)
	sh_stream->connected_at_ticks = k_uptime_ticks();
#if defined(CONFIG_LIBLC3)
	atomic_set(&sh_stream->lc3_enqueue_cnt, PRIME_COUNT);
	sh_stream->lc3_sdu_cnt = 0U;
#endif /* CONFIG_LIBLC3 */
#endif /* CONFIG_BT_AUDIO_TX */

	printk("Stream %p started\n", bap_stream);

#if defined(CONFIG_BT_AUDIO_RX)
	sh_stream->lost_pkts = 0U;
	sh_stream->err_pkts = 0U;
	sh_stream->dup_psn = 0U;
	sh_stream->rx_cnt = 0U;
	sh_stream->dup_ts = 0U;
#endif
}

static void stream_stopped_cb(struct bt_bap_stream *stream, uint8_t reason)
{
	printk("Stream %p stopped with reason 0x%02X\n", stream, reason);

#if defined(CONFIG_LIBLC3)
	clear_lc3_sine_data(stream);
#endif /* CONFIG_LIBLC3 */
}

#if defined(CONFIG_BT_BAP_UNICAST)
static void stream_released_cb(struct bt_bap_stream *stream)
{
	shell_print(ctx_shell, "Stream %p released\n", stream);

#if defined(CONFIG_BT_BAP_UNICAST_CLIENT)
	if (default_unicast_group != NULL) {
		bool group_can_be_deleted = true;

		for (size_t i = 0U; i < ARRAY_SIZE(unicast_streams); i++) {
			const struct bt_bap_stream *stream = &unicast_streams[i].stream.bap_stream;

			if (stream->ep != NULL) {
				struct bt_bap_ep_info ep_info;

				bt_bap_ep_get_info(stream->ep, &ep_info);

				if (ep_info.state != BT_BAP_EP_STATE_CODEC_CONFIGURED &&
				    ep_info.state != BT_BAP_EP_STATE_IDLE) {
					group_can_be_deleted = false;
					break;
				}
			}
		}

		if (group_can_be_deleted) {
			int err;

			shell_print(ctx_shell, "All streams released, deleting group\n");

			err = bt_bap_unicast_group_delete(default_unicast_group);

			if (err != 0) {
				shell_error(ctx_shell, "Failed to delete unicast group: %d", err);
			} else {
				default_unicast_group = NULL;
			}
		}
	}
#endif /* CONFIG_BT_BAP_UNICAST_CLIENT */

#if defined(CONFIG_LIBLC3)
	/* stop sending */
	clear_lc3_sine_data(stream);
#endif /* CONFIG_LIBLC3 */
}
#endif /* CONFIG_BT_BAP_UNICAST */

static struct bt_bap_stream_ops stream_ops = {
#if defined(CONFIG_BT_AUDIO_RX)
	.recv = audio_recv,
#endif /* CONFIG_BT_AUDIO_RX */
#if defined(CONFIG_BT_BAP_UNICAST)
	.released = stream_released_cb,
	.enabled = stream_enabled_cb,
#endif /* CONFIG_BT_BAP_UNICAST */
	.started = stream_started_cb,
	.stopped = stream_stopped_cb,
#if defined(CONFIG_LIBLC3) && defined(CONFIG_BT_AUDIO_TX)
	.sent = sdu_sent_cb,
#endif
};

#if defined(CONFIG_BT_BAP_BROADCAST_SOURCE)
static int cmd_select_broadcast_source(const struct shell *sh, size_t argc,
				       char *argv[])
{
	unsigned long index;
	int err = 0;

	index = shell_strtoul(argv[1], 0, &err);
	if (err != 0) {
		shell_error(sh, "Could not parse index: %d", err);

		return -ENOEXEC;
	}

	if (index > ARRAY_SIZE(broadcast_source_streams)) {
		shell_error(sh, "Invalid index: %lu", index);

		return -ENOEXEC;
	}

	default_stream = &broadcast_source_streams[index].stream.bap_stream;

	return 0;
}

static int cmd_create_broadcast(const struct shell *sh, size_t argc,
				char *argv[])
{
	struct bt_bap_broadcast_source_stream_param
		stream_params[ARRAY_SIZE(broadcast_source_streams)];
	struct bt_bap_broadcast_source_subgroup_param subgroup_param;
	struct bt_bap_broadcast_source_create_param create_param = {0};
	const struct named_lc3_preset *named_preset;
	int err;

	if (default_source.bap_source != NULL) {
		shell_info(sh, "Broadcast source already created");
		return -ENOEXEC;
	}

	named_preset = default_broadcast_source_preset;

	for (size_t i = 1U; i < argc; i++) {
		char *arg = argv[i];

		if (strcmp(arg, "enc") == 0) {
			if (argc > i) {
				size_t bcode_len;

				i++;
				arg = argv[i];

				bcode_len = hex2bin(arg, strlen(arg),
						    create_param.broadcast_code,
						    sizeof(create_param.broadcast_code));

				if (bcode_len != sizeof(create_param.broadcast_code)) {
					shell_error(sh, "Invalid Broadcast Code Length: %zu",
						    bcode_len);

					return -ENOEXEC;
				}

				create_param.encryption = true;
			} else {
				shell_help(sh);

				return SHELL_CMD_HELP_PRINTED;
			}
		} else if (strcmp(arg, "preset") == 0) {
			if (argc > i) {

				i++;
				arg = argv[i];

				named_preset = get_named_preset(false, arg);
				if (named_preset == NULL) {
					shell_error(sh, "Unable to parse named_preset %s",
						    arg);

					return -ENOEXEC;
				}
			} else {
				shell_help(sh);

				return SHELL_CMD_HELP_PRINTED;
			}
		}
	}

	copy_broadcast_source_preset(&default_source, named_preset);

	(void)memset(stream_params, 0, sizeof(stream_params));
	for (size_t i = 0; i < ARRAY_SIZE(stream_params); i++) {
		stream_params[i].stream = &broadcast_source_streams[i].stream.bap_stream;
	}
	subgroup_param.params_count = ARRAY_SIZE(stream_params);
	subgroup_param.params = stream_params;
	subgroup_param.codec_cfg = &default_source.codec_cfg;
	create_param.params_count = 1U;
	create_param.params = &subgroup_param;
	create_param.qos = &default_source.qos;

	err = bt_bap_broadcast_source_create(&create_param, &default_source.bap_source);
	if (err != 0) {
		shell_error(sh, "Unable to create broadcast source: %d", err);
		return err;
	}

	shell_print(sh, "Broadcast source created: preset %s",
		    named_preset->name);

	if (default_stream == NULL) {
		default_stream = &broadcast_source_streams[0].stream.bap_stream;
	}

	return 0;
}

static int cmd_start_broadcast(const struct shell *sh, size_t argc,
			       char *argv[])
{
	struct bt_le_ext_adv *adv = adv_sets[selected_adv];
	int err;

	if (adv == NULL) {
		shell_info(sh, "Extended advertising set is NULL");
		return -ENOEXEC;
	}

	if (default_source.bap_source == NULL) {
		shell_info(sh, "Broadcast source not created");
		return -ENOEXEC;
	}

	err = bt_bap_broadcast_source_start(default_source.bap_source, adv_sets[selected_adv]);
	if (err != 0) {
		shell_error(sh, "Unable to start broadcast source: %d", err);
		return err;
	}

	return 0;
}

static int cmd_stop_broadcast(const struct shell *sh, size_t argc, char *argv[])
{
	int err;

	if (default_source.bap_source == NULL) {
		shell_info(sh, "Broadcast source not created");
		return -ENOEXEC;
	}

	err = bt_bap_broadcast_source_stop(default_source.bap_source);
	if (err != 0) {
		shell_error(sh, "Unable to stop broadcast source: %d", err);
		return err;
	}

	return 0;
}

static int cmd_delete_broadcast(const struct shell *sh, size_t argc,
				char *argv[])
{
	int err;

	if (default_source.bap_source == NULL) {
		shell_info(sh, "Broadcast source not created");
		return -ENOEXEC;
	}

	err = bt_bap_broadcast_source_delete(default_source.bap_source);
	if (err != 0) {
		shell_error(sh, "Unable to delete broadcast source: %d", err);
		return err;
	}
	default_source.bap_source = NULL;

	return 0;
}
#endif /* CONFIG_BT_BAP_BROADCAST_SOURCE */

#if defined(CONFIG_BT_BAP_BROADCAST_SINK)
static int cmd_broadcast_scan(const struct shell *sh, size_t argc, char *argv[])
{
	int err;
	struct bt_le_scan_param param = {
			.type       = BT_LE_SCAN_TYPE_ACTIVE,
			.options    = BT_LE_SCAN_OPT_NONE,
			.interval   = BT_GAP_SCAN_FAST_INTERVAL,
			.window     = BT_GAP_SCAN_FAST_WINDOW,
			.timeout    = 0 };

	if (!initialized) {
		shell_error(sh, "Not initialized");
		return -ENOEXEC;
	}

	if (strcmp(argv[1], "on") == 0) {
		err =  bt_bap_broadcast_sink_scan_start(&param);
		if (err != 0) {
			shell_error(sh, "Could not start scan: %d", err);
		}
	} else if (strcmp(argv[1], "off") == 0) {
		err = bt_bap_broadcast_sink_scan_stop();
		if (err != 0) {
			shell_error(sh, "Could not stop scan: %d", err);
		}
	} else {
		shell_help(sh);
		err = -ENOEXEC;
	}

	return err;
}

static int cmd_accept_broadcast(const struct shell *sh, size_t argc,
				char *argv[])
{
	unsigned long broadcast_id;
	int err = 0;

	broadcast_id = shell_strtoul(argv[1], 0, &err);
	if (err != 0) {
		shell_error(sh, "Could not parse broadcast_id: %d", err);

		return -ENOEXEC;
	}

	if (broadcast_id > BT_AUDIO_BROADCAST_ID_MAX) {
		shell_error(sh, "Invalid broadcast_id: %lu", broadcast_id);

		return -ENOEXEC;
	}

	accepted_broadcast_id = broadcast_id;

	return 0;
}

static int cmd_sync_broadcast(const struct shell *sh, size_t argc, char *argv[])
{
	struct bt_bap_stream *streams[ARRAY_SIZE(broadcast_sink_streams)];
	uint32_t bis_bitfield;
	int err;

	bis_bitfield = 0;
	for (int i = 1; i < argc; i++) {
		unsigned long val;
		int err = 0;

		val = shell_strtoul(argv[i], 0, &err);
		if (err != 0) {
			shell_error(sh, "Could not parse BIS index val: %d",
				    err);

			return -ENOEXEC;
		}

		if (!IN_RANGE(val,
			      BT_ISO_BIS_INDEX_MIN,
			      BT_ISO_BIS_INDEX_MAX)) {
			shell_error(sh, "Invalid index: %lu", val);

			return -ENOEXEC;
		}

		bis_bitfield |= BIT(val);
	}

	if (default_sink == NULL) {
		shell_error(sh, "No sink available");
		return -ENOEXEC;
	}

	(void)memset(streams, 0, sizeof(streams));
	for (size_t i = 0; i < ARRAY_SIZE(streams); i++) {
		streams[i] = &broadcast_sink_streams[i];
	}

	err = bt_bap_broadcast_sink_sync(default_sink, bis_bitfield, streams, NULL);
	if (err != 0) {
		shell_error(sh, "Failed to sync to broadcast: %d", err);
		return err;
	}

	return 0;
}

static int cmd_stop_broadcast_sink(const struct shell *sh, size_t argc,
				   char *argv[])
{
	int err;

	if (default_sink == NULL) {
		shell_error(sh, "No sink available");
		return -ENOEXEC;
	}

	err = bt_bap_broadcast_sink_stop(default_sink);
	if (err != 0) {
		shell_error(sh, "Failed to stop sink: %d", err);
		return err;
	}

	return err;
}

static int cmd_term_broadcast_sink(const struct shell *sh, size_t argc,
				   char *argv[])
{
	int err;

	if (default_sink == NULL) {
		shell_error(sh, "No sink available");
		return -ENOEXEC;
	}

	err = bt_bap_broadcast_sink_delete(default_sink);
	if (err != 0) {
		shell_error(sh, "Failed to term sink: %d", err);
		return err;
	}

	default_sink = NULL;
	sink_syncable = false;

	return err;
}
#endif /* CONFIG_BT_BAP_BROADCAST_SINK */

static int cmd_set_loc(const struct shell *sh, size_t argc, char *argv[])
{
	int err = 0;
	enum bt_audio_dir dir;
	enum bt_audio_location loc;
	unsigned long loc_val;

	if (!strcmp(argv[1], "sink")) {
		dir = BT_AUDIO_DIR_SINK;
	} else if (!strcmp(argv[1], "source")) {
		dir = BT_AUDIO_DIR_SOURCE;
	} else {
		shell_error(sh, "Unsupported dir: %s", argv[1]);
		return -ENOEXEC;
	}

	loc_val = shell_strtoul(argv[2], 16, &err);
	if (err != 0) {
		shell_error(sh, "Could not parse loc_val: %d", err);

		return -ENOEXEC;
	}

	if (loc_val == BT_AUDIO_LOCATION_PROHIBITED ||
	    loc_val > BT_AUDIO_LOCATION_ANY) {
		shell_error(sh, "Invalid location: %lu", loc_val);

		return -ENOEXEC;
	}

	loc = loc_val;

	err = bt_pacs_set_location(dir, loc);
	if (err) {
		shell_error(ctx_shell, "Set available contexts err %d", err);
		return -ENOEXEC;
	}

	return 0;
}

static int cmd_context(const struct shell *sh, size_t argc, char *argv[])
{
	int err = 0;
	enum bt_audio_dir dir;
	enum bt_audio_context ctx;
	unsigned long ctx_val;

	if (!strcmp(argv[1], "sink")) {
		dir = BT_AUDIO_DIR_SINK;
	} else if (!strcmp(argv[1], "source")) {
		dir = BT_AUDIO_DIR_SOURCE;
	} else {
		shell_error(sh, "Unsupported dir: %s", argv[1]);
		return -ENOEXEC;
	}

	ctx_val = shell_strtoul(argv[2], 16, &err);
	if (err) {
		shell_error(sh, "Could not parse context: %d", err);

		return err;
	}

	if (ctx_val == BT_AUDIO_CONTEXT_TYPE_PROHIBITED ||
	    ctx_val > BT_AUDIO_CONTEXT_TYPE_ANY) {
		shell_error(sh, "Invalid context: %lu", ctx_val);

		return -ENOEXEC;
	}

	ctx = ctx_val;

	if (!strcmp(argv[3], "supported")) {
		err = bt_pacs_set_supported_contexts(dir, ctx);
		if (err) {
			shell_error(ctx_shell, "Set supported contexts err %d", err);
			return err;
		}
	} else if (!strcmp(argv[3], "available")) {
		err = bt_pacs_set_available_contexts(dir, ctx);
		if (err) {
			shell_error(ctx_shell, "Set available contexts err %d", err);
			return err;
		}
	} else {
		shell_error(sh, "Unsupported context type: %s", argv[3]);
		return -ENOEXEC;
	}

	return 0;
}

static int cmd_init(const struct shell *sh, size_t argc, char *argv[])
{
	int err, i;

	ctx_shell = sh;

	if (initialized) {
		shell_print(sh, "Already initialized");
		return -ENOEXEC;
	}

	if (IS_ENABLED(CONFIG_BT_BAP_UNICAST_SERVER)) {
		bt_bap_unicast_server_register_cb(&unicast_server_cb);
	}

	if (IS_ENABLED(CONFIG_BT_BAP_UNICAST_SERVER) ||
	    IS_ENABLED(CONFIG_BT_BAP_BROADCAST_SINK)) {
		bt_pacs_cap_register(BT_AUDIO_DIR_SINK, &cap_sink);
	}

	if (IS_ENABLED(CONFIG_BT_BAP_UNICAST_SERVER)) {
		bt_pacs_cap_register(BT_AUDIO_DIR_SOURCE, &cap_source);
	}

	if (IS_ENABLED(CONFIG_BT_PAC_SNK_LOC)) {
		err = bt_pacs_set_location(BT_AUDIO_DIR_SINK, LOCATION);
		__ASSERT(err == 0, "Failed to set sink location: %d", err);

		err = bt_pacs_set_supported_contexts(BT_AUDIO_DIR_SINK,
						     CONTEXT);
		__ASSERT(err == 0, "Failed to set sink supported contexts: %d",
			 err);

		err = bt_pacs_set_available_contexts(BT_AUDIO_DIR_SINK,
						     CONTEXT);
		__ASSERT(err == 0, "Failed to set sink available contexts: %d",
			 err);
	}

	if (IS_ENABLED(CONFIG_BT_PAC_SRC_LOC)) {
		err = bt_pacs_set_location(BT_AUDIO_DIR_SOURCE, LOCATION);
		__ASSERT(err == 0, "Failed to set source location: %d", err);

		err = bt_pacs_set_supported_contexts(BT_AUDIO_DIR_SOURCE,
						     CONTEXT);
		__ASSERT(err == 0, "Failed to set sink supported contexts: %d",
			 err);

		err = bt_pacs_set_available_contexts(BT_AUDIO_DIR_SOURCE,
						     CONTEXT);
		__ASSERT(err == 0,
			 "Failed to set source available contexts: %d",
			 err);
	}

#if defined(CONFIG_BT_BAP_UNICAST)
	for (i = 0; i < ARRAY_SIZE(unicast_streams); i++) {
		bt_bap_stream_cb_register(&unicast_streams[i].stream.bap_stream, &stream_ops);

		if (IS_ENABLED(CONFIG_BT_BAP_UNICAST_CLIENT) &&
		    IS_ENABLED(CONFIG_BT_CAP_INITIATOR)) {
			/* If we use the cap initiator, we need to register the callbacks for CAP
			 * as well, as CAP will override and use the BAP callbacks if doing a CAP
			 * procedure
			 */
			bt_cap_stream_ops_register(&unicast_streams[i].stream, &stream_ops);
		}
	}
#endif /* CONFIG_BT_BAP_UNICAST */

#if defined(CONFIG_BT_BAP_BROADCAST_SINK)
	bt_bap_broadcast_sink_register_cb(&sink_cbs);

	for (i = 0; i < ARRAY_SIZE(broadcast_sink_streams); i++) {
		bt_bap_stream_cb_register(&broadcast_sink_streams[i],
					    &stream_ops);
	}
#endif /* CONFIG_BT_BAP_BROADCAST_SOURCE */

#if defined(CONFIG_BT_BAP_BROADCAST_SOURCE)
	for (i = 0; i < ARRAY_SIZE(broadcast_source_streams); i++) {
		bt_bap_stream_cb_register(&broadcast_source_streams[i].stream.bap_stream,
					  &stream_ops);
	}
#endif /* CONFIG_BT_BAP_BROADCAST_SOURCE */

	initialized = true;

	return 0;
}

#if defined(CONFIG_BT_AUDIO_TX)
#define DATA_MTU CONFIG_BT_ISO_TX_MTU
NET_BUF_POOL_FIXED_DEFINE(tx_pool, 1, DATA_MTU, 8, NULL);

static int cmd_send(const struct shell *sh, size_t argc, char *argv[])
{
	static uint8_t data[DATA_MTU - BT_ISO_CHAN_SEND_RESERVE];
	int ret, len;
	struct net_buf *buf;

	if (default_stream == NULL) {
		shell_error(sh, "Invalid (NULL) stream");

		return -ENOEXEC;
	}

	if (default_stream->qos == NULL) {
		shell_error(sh, "NULL stream QoS");

		return -ENOEXEC;
	}

	if (argc > 1) {
		len = hex2bin(argv[1], strlen(argv[1]), data, sizeof(data));
		if (len > default_stream->qos->sdu) {
			shell_print(sh, "Unable to send: len %d > %u MTU",
				    len, default_stream->qos->sdu);

			return -ENOEXEC;
		}
	} else {
		len = MIN(default_stream->qos->sdu, sizeof(data));
		memset(data, 0xff, len);
	}

	buf = net_buf_alloc(&tx_pool, K_FOREVER);
	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

	net_buf_add_mem(buf, data, len);

	ret = bt_bap_stream_send(default_stream, buf, get_next_seq_num(default_stream),
				 BT_ISO_TIMESTAMP_NONE);
	if (ret < 0) {
		shell_print(sh, "Unable to send: %d", -ret);
		net_buf_unref(buf);

		return -ENOEXEC;
	}

	shell_print(sh, "Sending:");
	shell_hexdump(sh, data, len);

	return 0;
}

#if defined(CONFIG_LIBLC3)
static bool stream_start_sine_verify(const struct bt_bap_stream *bap_stream)
{
	int stream_frame_duration_us;
	struct bt_bap_ep_info info;
	int stream_freq_hz;
	int err;

	if (bap_stream == NULL || bap_stream->qos == NULL) {
		return false;
	}

	err = bt_bap_ep_get_info(bap_stream->ep, &info);
	if (err != 0) {
		return false;
	}

	if (info.state != BT_BAP_EP_STATE_STREAMING) {
		return false;
	}

	stream_freq_hz = bt_audio_codec_cfg_get_freq(bap_stream->codec_cfg);
	if (stream_freq_hz != lc3_freq_hz) {
		return false;
	}

	stream_frame_duration_us = bt_audio_codec_cfg_get_frame_duration_us(bap_stream->codec_cfg);
	if (stream_frame_duration_us != lc3_frame_duration_us) {
		return false;
	}

	return true;
}
static int stream_start_sine(struct bt_bap_stream *bap_stream)
{
	struct shell_stream *sh_stream = shell_stream_from_bap_stream(bap_stream);
	int err;

	k_work_init_delayable(&sh_stream->audio_send_work, lc3_audio_send_data);

	err = k_work_schedule(&sh_stream->audio_send_work, K_NO_WAIT);
	if (err < 0) {
		return -ENOEXEC;
	}

	sh_stream->tx_active = true;
	sh_stream->seq_num = get_next_seq_num(bap_stream);

	return 0;
}

static int cmd_start_sine(const struct shell *sh, size_t argc, char *argv[])
{
	bool start_all = false;
	int err;

	if (argc > 1) {
		if (strcmp(argv[1], "all") == 0) {
			start_all = true;
		} else {
			shell_help(sh);

			return SHELL_CMD_HELP_PRINTED;
		}
	}

	if (start_all) {
		bool lc3_initialized = false;

		for (size_t i = 0U; i < ARRAY_SIZE(unicast_streams); i++) {
			struct bt_bap_stream *bap_stream = &unicast_streams[i].stream.bap_stream;

			if (!lc3_initialized) {
				init_lc3(bap_stream);
				lc3_initialized = true;
			}

			if (!stream_start_sine_verify(bap_stream)) {
				continue;
			}

			err = stream_start_sine(bap_stream);
			if (err != 0) {
				shell_error(sh, "Failed to start TX for stream %p: %d", bap_stream,
					    err);
				return err;
			}

			shell_print(sh, "Started transmitting on unicast stream %p", bap_stream);
		}

		for (size_t i = 0U; i < ARRAY_SIZE(broadcast_source_streams); i++) {
			struct bt_bap_stream *bap_stream =
				&broadcast_source_streams[i].stream.bap_stream;

			if (!lc3_initialized) {
				init_lc3(bap_stream);
				lc3_initialized = true;
			}

			if (!stream_start_sine_verify(bap_stream)) {
				continue;
			}

			err = stream_start_sine(bap_stream);
			if (err != 0) {
				shell_error(sh, "Failed to start TX for stream %p: %d", bap_stream,
					    err);
				return err;
			}

			shell_print(sh, "Started transmitting on broadcast stream %p", bap_stream);
		}
	} else {
		if (stream_start_sine_verify(default_stream)) {
			shell_error(sh, "Invalid stream %p", default_stream);
			return -ENOEXEC;
		}

		init_lc3(default_stream);

		err = stream_start_sine(default_stream);
		if (err != 0) {
			shell_error(sh, "Failed to start TX for stream %p: %d", default_stream,
				    err);
			return err;
		}

		shell_print(sh, "Started transmitting on default_stream %p", default_stream);
	}

	return 0;
}

static int cmd_stop_sine(const struct shell *sh, size_t argc, char *argv[])
{
	bool stop_all = false;

	if (argc > 1) {
		if (strcmp(argv[1], "all") == 0) {
			stop_all = true;
		} else {
			shell_help(sh);

			return SHELL_CMD_HELP_PRINTED;
		}
	}

	if (stop_all) {
		for (size_t i = 0U; i < ARRAY_SIZE(unicast_streams); i++) {
			struct bt_bap_stream *bap_stream = &unicast_streams[i].stream.bap_stream;

			if (unicast_streams[i].tx_active) {
				clear_lc3_sine_data(bap_stream);
				shell_print(sh, "Stopped transmitting on stream %p", bap_stream);
			}
		}

		for (size_t i = 0U; i < ARRAY_SIZE(broadcast_source_streams); i++) {
			struct bt_bap_stream *bap_stream =
				&broadcast_source_streams[i].stream.bap_stream;
			if (unicast_streams[i].tx_active) {
				clear_lc3_sine_data(bap_stream);
				shell_print(sh, "Stopped transmitting on stream %p", bap_stream);
			}
		}
	} else {
		struct shell_stream *sh_stream = shell_stream_from_bap_stream(default_stream);

		if (sh_stream->tx_active) {
			clear_lc3_sine_data(default_stream);
			shell_print(sh, "Stopped transmitting on stream %p", default_stream);
		}
	}

	return 0;
}
#endif /* CONFIG_LIBLC3 */
#endif /* CONFIG_BT_AUDIO_TX */

#if defined(CONFIG_BT_AUDIO_RX)
static int cmd_recv_stats(const struct shell *sh, size_t argc, char *argv[])
{
	if (argc == 1) {
		shell_info(sh, "Current receive stats interval: %lu", recv_stats_interval);
	} else {
		int err = 0;
		unsigned long interval;

		interval = shell_strtoul(argv[1], 0, &err);
		if (err != 0) {
			shell_error(sh, "Could not parse interval: %d", err);

			return -ENOEXEC;
		}

		if (interval == 0U) {
			shell_error(sh, "Interval cannot be 0");

			return -ENOEXEC;
		}

		recv_stats_interval = interval;
	}

	return 0;
}
#endif /* CONFIG_BT_AUDIO_RX */

#if defined(CONFIG_BT_BAP_UNICAST_SERVER)
static void print_ase_info(struct bt_bap_ep *ep, void *user_data)
{
	struct bt_bap_ep_info info;

	bt_bap_ep_get_info(ep, &info);
	printk("ASE info: id %u state %u dir %u\n", info.id, info.state,
	       info.dir);
}

static int cmd_print_ase_info(const struct shell *sh, size_t argc, char *argv[])
{
	if (!default_conn) {
		shell_error(sh, "Not connected");
		return -ENOEXEC;
	}

	bt_bap_unicast_server_foreach_ep(default_conn, print_ase_info, NULL);

	return 0;
}
#endif /* CONFIG_BT_BAP_UNICAST_SERVER */

SHELL_STATIC_SUBCMD_SET_CREATE(
	bap_cmds, SHELL_CMD_ARG(init, NULL, NULL, cmd_init, 1, 0),
#if defined(CONFIG_BT_BAP_BROADCAST_SOURCE)
	SHELL_CMD_ARG(select_broadcast, NULL, "<stream>", cmd_select_broadcast_source, 2, 0),
	SHELL_CMD_ARG(create_broadcast, NULL, "[preset <preset_name>] [enc <broadcast_code>]",
		      cmd_create_broadcast, 1, 2),
	SHELL_CMD_ARG(start_broadcast, NULL, "", cmd_start_broadcast, 1, 0),
	SHELL_CMD_ARG(stop_broadcast, NULL, "", cmd_stop_broadcast, 1, 0),
	SHELL_CMD_ARG(delete_broadcast, NULL, "", cmd_delete_broadcast, 1, 0),
#endif /* CONFIG_BT_BAP_BROADCAST_SOURCE */
#if defined(CONFIG_BT_BAP_BROADCAST_SINK)
	SHELL_CMD_ARG(broadcast_scan, NULL, "<on, off>", cmd_broadcast_scan, 2, 0),
	SHELL_CMD_ARG(accept_broadcast, NULL, "0x<broadcast_id>", cmd_accept_broadcast, 2, 0),
	SHELL_CMD_ARG(sync_broadcast, NULL, "0x<bis_index> [[[0x<bis_index>] 0x<bis_index>] ...]",
		      cmd_sync_broadcast, 2, ARRAY_SIZE(broadcast_sink_streams) - 1),
	SHELL_CMD_ARG(stop_broadcast_sink, NULL, "Stops broadcast sink", cmd_stop_broadcast_sink, 1,
		      0),
	SHELL_CMD_ARG(term_broadcast_sink, NULL, "", cmd_term_broadcast_sink, 1, 0),
#endif /* CONFIG_BT_BAP_BROADCAST_SINK */
#if defined(CONFIG_BT_BAP_UNICAST)
#if defined(CONFIG_BT_BAP_UNICAST_CLIENT)
	SHELL_CMD_ARG(discover, NULL, "[dir: sink, source]", cmd_discover, 1, 1),
	SHELL_CMD_ARG(config, NULL,
		      "<direction: sink, source> <index> [loc <loc_bits>] [preset <preset_name>]",
		      cmd_config, 3, 4),
	SHELL_CMD_ARG(stream_qos, NULL, "interval [framing] [latency] [pd] [sdu] [phy] [rtn]",
		      cmd_stream_qos, 2, 6),
	SHELL_CMD_ARG(qos, NULL, "Send QoS configure for Unicast Group", cmd_qos, 1, 0),
	SHELL_CMD_ARG(enable, NULL, "[context]", cmd_enable, 1, 1),
	SHELL_CMD_ARG(stop, NULL, NULL, cmd_stop, 1, 0),
	SHELL_CMD_ARG(list, NULL, NULL, cmd_list, 1, 0),
#endif /* CONFIG_BT_BAP_UNICAST_CLIENT */
#if defined(CONFIG_BT_BAP_UNICAST_SERVER)
	SHELL_CMD_ARG(print_ase_info, NULL, "Print ASE info for default connection",
		      cmd_print_ase_info, 0, 0),
#endif /* CONFIG_BT_BAP_UNICAST_SERVER */
	SHELL_CMD_ARG(metadata, NULL, "[context]", cmd_metadata, 1, 1),
	SHELL_CMD_ARG(start, NULL, NULL, cmd_start, 1, 0),
	SHELL_CMD_ARG(disable, NULL, NULL, cmd_disable, 1, 0),
	SHELL_CMD_ARG(release, NULL, NULL, cmd_release, 1, 0),
	SHELL_CMD_ARG(select_unicast, NULL, "<stream>", cmd_select_unicast, 2, 0),
#endif /* CONFIG_BT_BAP_UNICAST */
	SHELL_CMD_ARG(preset, NULL, "<sink, source, broadcast> [preset]", cmd_preset, 2, 1),
#if defined(CONFIG_BT_AUDIO_TX)
	SHELL_CMD_ARG(send, NULL, "Send to Audio Stream [data]", cmd_send, 1, 1),
#if defined(CONFIG_LIBLC3)
	SHELL_CMD_ARG(start_sine, NULL, "Start sending a LC3 encoded sine wave [all]",
		      cmd_start_sine, 1, 1),
	SHELL_CMD_ARG(stop_sine, NULL, "Stop sending a LC3 encoded sine wave [all]", cmd_stop_sine,
		      1, 1),
#endif /* CONFIG_LIBLC3 */
#endif /* CONFIG_BT_AUDIO_TX */
#if defined(CONFIG_BT_AUDIO_RX)
	SHELL_CMD_ARG(recv_stats, NULL,
		      "Sets or gets the receive statistics reporting interval in # of packets",
		      cmd_recv_stats, 1, 1),
#endif /* CONFIG_BT_AUDIO_RX */
	SHELL_COND_CMD_ARG(CONFIG_BT_PACS, set_location, NULL,
			   "<direction: sink, source> <location bitmask>", cmd_set_loc, 3, 0),
	SHELL_COND_CMD_ARG(CONFIG_BT_PACS, set_context, NULL,
			   "<direction: sink, source>"
			   "<context bitmask> <type: supported, available>",
			   cmd_context, 4, 0),
	SHELL_SUBCMD_SET_END);

static int cmd_bap(const struct shell *sh, size_t argc, char **argv)
{
	if (argc > 1) {
		shell_error(sh, "%s unknown parameter: %s",
			    argv[0], argv[1]);
	} else {
		shell_error(sh, "%s Missing subcommand", argv[0]);
	}

	return -ENOEXEC;
}

SHELL_CMD_ARG_REGISTER(bap, &bap_cmds, "Bluetooth BAP shell commands", cmd_bap, 1, 1);

static ssize_t connectable_ad_data_add(struct bt_data *data_array,
				       size_t data_array_size)
{
	static const uint8_t ad_ext_uuid16[] = {
		IF_ENABLED(CONFIG_BT_MICP_MIC_DEV, (BT_UUID_16_ENCODE(BT_UUID_MICS_VAL),))
		IF_ENABLED(CONFIG_BT_ASCS, (BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL),))
		IF_ENABLED(CONFIG_BT_BAP_SCAN_DELEGATOR, (BT_UUID_16_ENCODE(BT_UUID_BASS_VAL),))
		IF_ENABLED(CONFIG_BT_PACS, (BT_UUID_16_ENCODE(BT_UUID_PACS_VAL),))
		IF_ENABLED(CONFIG_BT_GTBS, (BT_UUID_16_ENCODE(BT_UUID_GTBS_VAL),))
		IF_ENABLED(CONFIG_BT_TBS, (BT_UUID_16_ENCODE(BT_UUID_TBS_VAL),))
		IF_ENABLED(CONFIG_BT_VCP_VOL_REND, (BT_UUID_16_ENCODE(BT_UUID_VCS_VAL),))
		IF_ENABLED(CONFIG_BT_HAS, (BT_UUID_16_ENCODE(BT_UUID_HAS_VAL),)) /* Shall be last */
	};
	size_t ad_len = 0;

	if (IS_ENABLED(CONFIG_BT_ASCS)) {
		static uint8_t ad_bap_announcement[8] = {
			BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL),
			BT_AUDIO_UNICAST_ANNOUNCEMENT_TARGETED,
		};
		enum bt_audio_context snk_context, src_context;

		snk_context = bt_pacs_get_available_contexts(BT_AUDIO_DIR_SINK);
		sys_put_le16(snk_context, &ad_bap_announcement[3]);

		src_context = bt_pacs_get_available_contexts(BT_AUDIO_DIR_SOURCE);
		sys_put_le16(snk_context, &ad_bap_announcement[5]);

		/* Metadata length */
		ad_bap_announcement[7] = 0x00;

		__ASSERT(data_array_size > ad_len, "No space for AD_BAP_ANNOUNCEMENT");
		data_array[ad_len].type = BT_DATA_SVC_DATA16;
		data_array[ad_len].data_len = ARRAY_SIZE(ad_bap_announcement);
		data_array[ad_len].data = &ad_bap_announcement[0];
		ad_len++;
	}

	if (IS_ENABLED(CONFIG_BT_CAP_ACCEPTOR)) {
		ad_len += cap_acceptor_ad_data_add(&data_array[ad_len], data_array_size - ad_len,
						   true);
	}

	if (ARRAY_SIZE(ad_ext_uuid16) > 0) {
		size_t uuid16_size;

		if (data_array_size <= ad_len) {
			shell_warn(ctx_shell, "No space for AD_UUID16");
			return ad_len;
		}

		data_array[ad_len].type = BT_DATA_UUID16_SOME;

		if (IS_ENABLED(CONFIG_BT_HAS) && IS_ENABLED(CONFIG_BT_PRIVACY)) {
			/* If the HA is in one of the GAP connectable modes and is using a
			 * resolvable private address, the HA shall not include the Hearing Access
			 * Service UUID in the Service UUID AD type field of the advertising data
			 * or scan response.
			 */
			uuid16_size = ARRAY_SIZE(ad_ext_uuid16) - BT_UUID_SIZE_16;
		} else {
			uuid16_size = ARRAY_SIZE(ad_ext_uuid16);
		}

		/* We can maximum advertise 127 16-bit UUIDs = 254 octets */
		data_array[ad_len].data_len = MIN(uuid16_size, 254);

		data_array[ad_len].data = &ad_ext_uuid16[0];
		ad_len++;
	}

	return ad_len;
}

static ssize_t nonconnectable_ad_data_add(struct bt_data *data_array,
					  const size_t data_array_size)
{
	static const uint8_t ad_ext_uuid16[] = {
		IF_ENABLED(CONFIG_BT_PACS, (BT_UUID_16_ENCODE(BT_UUID_PACS_VAL),))
		IF_ENABLED(CONFIG_BT_CAP_ACCEPTOR, (BT_UUID_16_ENCODE(BT_UUID_CAS_VAL),))
	};
	size_t ad_len = 0;

	if (IS_ENABLED(CONFIG_BT_CAP_ACCEPTOR)) {
		static const uint8_t ad_cap_announcement[3] = {
			BT_UUID_16_ENCODE(BT_UUID_CAS_VAL),
			BT_AUDIO_UNICAST_ANNOUNCEMENT_TARGETED,
		};

		__ASSERT(data_array_size > ad_len, "No space for AD_CAP_ANNOUNCEMENT");
		data_array[ad_len].type = BT_DATA_SVC_DATA16;
		data_array[ad_len].data_len = ARRAY_SIZE(ad_cap_announcement);
		data_array[ad_len].data = &ad_cap_announcement[0];
		ad_len++;
	}

#if defined(CONFIG_BT_BAP_BROADCAST_SOURCE)
	if (default_source.bap_source) {
		static uint8_t ad_bap_broadcast_announcement[5] = {
			BT_UUID_16_ENCODE(BT_UUID_BROADCAST_AUDIO_VAL),
		};
		uint32_t broadcast_id;
		int err;

		err = bt_bap_broadcast_source_get_id(default_source.bap_source, &broadcast_id);
		if (err != 0) {
			printk("Unable to get broadcast ID: %d\n", err);

			return -1;
		}

		sys_put_le24(broadcast_id, &ad_bap_broadcast_announcement[2]);
		data_array[ad_len].type = BT_DATA_SVC_DATA16;
		data_array[ad_len].data_len = ARRAY_SIZE(ad_bap_broadcast_announcement);
		data_array[ad_len].data = ad_bap_broadcast_announcement;
		ad_len++;
	}
#endif /* CONFIG_BT_BAP_BROADCAST_SOURCE */

	if (ARRAY_SIZE(ad_ext_uuid16) > 0) {
		if (data_array_size <= ad_len) {
			shell_warn(ctx_shell, "No space for AD_UUID16");
			return ad_len;
		}

		data_array[ad_len].type = BT_DATA_UUID16_SOME;
		data_array[ad_len].data_len = ARRAY_SIZE(ad_ext_uuid16);
		data_array[ad_len].data = &ad_ext_uuid16[0];
		ad_len++;
	}

	return ad_len;
}

ssize_t audio_ad_data_add(struct bt_data *data_array, const size_t data_array_size,
			  const bool discoverable, const bool connectable)
{
	if (!discoverable) {
		return 0;
	}

	if (connectable) {
		return connectable_ad_data_add(data_array, data_array_size);
	} else {
		return nonconnectable_ad_data_add(data_array, data_array_size);
	}
}

ssize_t audio_pa_data_add(struct bt_data *data_array,
			  const size_t data_array_size)
{
	size_t ad_len = 0;

#if defined(CONFIG_BT_BAP_BROADCAST_SOURCE)
	if (default_source.bap_source) {
		/* Required size of the buffer depends on what has been
		 * configured. We just use the maximum size possible.
		 */
		NET_BUF_SIMPLE_DEFINE_STATIC(base_buf, UINT8_MAX);
		int err;

		err = bt_bap_broadcast_source_get_base(default_source.bap_source, &base_buf);
		if (err != 0) {
			printk("Unable to get BASE: %d\n", err);

			return -1;
		}

		data_array[ad_len].type = BT_DATA_SVC_DATA16;
		data_array[ad_len].data_len = base_buf.len;
		data_array[ad_len].data = base_buf.data;
		ad_len++;
	}
#endif /* CONFIG_BT_BAP_BROADCAST_SOURCE */

	return ad_len;
}
