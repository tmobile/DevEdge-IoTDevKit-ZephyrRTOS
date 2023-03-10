/**
 * @file
 * @brief Shell APIs for Bluetooth BASS client
 *
 * Copyright (c) 2020-2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include "bt.h"
#include "../host/hci_core.h"

static void bap_broadcast_assistant_discover_cb(struct bt_conn *conn, int err,
						uint8_t recv_state_count)
{
	if (err != 0) {
		shell_error(ctx_shell, "BASS discover failed (%d)", err);
	} else {
		shell_print(ctx_shell, "BASS discover done with %u recv states",
			    recv_state_count);
	}

}

static void bap_broadcast_assistant_scan_cb(const struct bt_le_scan_recv_info *info,
					    uint32_t broadcast_id)
{
	char le_addr[BT_ADDR_LE_STR_LEN];
	char name[30];

	(void)memset(name, 0, sizeof(name));

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
	shell_print(ctx_shell, "[DEVICE]: %s (%s), broadcast_id %u, "
		    "interval (ms) %u), SID 0x%x, RSSI %i",
		    le_addr, name, broadcast_id, info->interval * 5 / 4,
		    info->sid, info->rssi);

}

static bool metadata_entry(struct bt_data *data, void *user_data)
{
	char metadata[512];

	bin2hex(data->data, data->data_len, metadata, sizeof(metadata));

	shell_print(ctx_shell, "\t\tMetadata length %u, type %u, data: %s",
		    data->data_len, data->type, metadata);

	return true;
}

static void bap_broadcast_assistant_recv_state_cb(
	struct bt_conn *conn, int err,
	const struct bt_bap_scan_delegator_recv_state *state)
{
	char le_addr[BT_ADDR_LE_STR_LEN];
	char bad_code[33];
	bool is_bad_code;

	if (err != 0) {
		shell_error(ctx_shell, "BASS recv state read failed (%d)", err);
		return;
	}

	bt_addr_le_to_str(&state->addr, le_addr, sizeof(le_addr));
	bin2hex(state->bad_code, BT_AUDIO_BROADCAST_CODE_SIZE,
		bad_code, sizeof(bad_code));

	is_bad_code = state->encrypt_state == BT_BAP_BIG_ENC_STATE_BAD_CODE;
	shell_print(ctx_shell, "BASS recv state: src_id %u, addr %s, "
		    "sid %u, sync_state %u, encrypt_state %u%s%s",
		    state->src_id, le_addr, state->adv_sid,
		    state->pa_sync_state, state->encrypt_state,
		    is_bad_code ? ", bad code" : "", bad_code);

	for (int i = 0; i < state->num_subgroups; i++) {
		const struct bt_bap_scan_delegator_subgroup *subgroup = &state->subgroups[i];
		struct net_buf_simple buf;

		shell_print(ctx_shell, "\t[%d]: BIS sync %u, metadata_len %u",
			    i, subgroup->bis_sync, subgroup->metadata_len);

		net_buf_simple_init_with_data(&buf, (void *)subgroup->metadata,
					      subgroup->metadata_len);
		bt_data_parse(&buf, metadata_entry, NULL);
	}


	if (state->pa_sync_state == BT_BAP_PA_STATE_INFO_REQ) {
		struct bt_le_per_adv_sync *per_adv_sync = NULL;
		struct bt_le_ext_adv *ext_adv = NULL;

		/* Lookup matching PA sync */
		for (int i = 0; i < ARRAY_SIZE(per_adv_syncs); i++) {
			if (per_adv_syncs[i] &&
			    bt_addr_le_eq(&per_adv_syncs[i]->addr, &state->addr)) {
				per_adv_sync = per_adv_syncs[i];
				break;
			}
		}

		if (per_adv_sync) {
			shell_print(ctx_shell, "Sending PAST");

			err = bt_le_per_adv_sync_transfer(per_adv_sync,
							  conn,
							  BT_UUID_BASS_VAL);

			if (err != 0) {
				shell_error(ctx_shell, "Could not transfer periodic adv sync: %d",
					    err);
			}

			return;
		}

		/* If no PA sync was found, check for local advertisers */
		for (int i = 0; i < ARRAY_SIZE(adv_sets); i++) {
			struct bt_le_oob oob_local;

			if (adv_sets[i] == NULL) {
				continue;
			}

			err = bt_le_ext_adv_oob_get_local(adv_sets[i],
							  &oob_local);

			if (err != 0) {
				shell_error(ctx_shell,
					    "Could not get local OOB %d",
					    err);
				return;
			}

			if (bt_addr_le_eq(&oob_local.addr, &state->addr)) {
				ext_adv = adv_sets[i];
				break;
			}
		}

		if (ext_adv != NULL && IS_ENABLED(CONFIG_BT_PER_ADV)) {
			shell_print(ctx_shell, "Sending local PAST");

			err = bt_le_per_adv_set_info_transfer(ext_adv, conn,
							      BT_UUID_BASS_VAL);

			if (err != 0) {
				shell_error(ctx_shell,
					    "Could not transfer per adv set info: %d",
					    err);
			}
		} else {
			shell_error(ctx_shell,
				    "Could not send PA to Scan Delegator");
		}
	}
}

static void bap_broadcast_assistant_recv_state_removed_cb(struct bt_conn *conn,
							  int err,
							  uint8_t src_id)
{
	if (err != 0) {
		shell_error(ctx_shell, "BASS recv state removed failed (%d)",
			    err);
	} else {
		shell_print(ctx_shell, "BASS recv state %u removed", src_id);
	}
}

static void bap_broadcast_assistant_scan_start_cb(struct bt_conn *conn, int err)
{
	if (err != 0) {
		shell_error(ctx_shell, "BASS scan start failed (%d)", err);
	} else {
		shell_print(ctx_shell, "BASS scan start successful");
	}
}

static void bap_broadcast_assistant_scan_stop_cb(struct bt_conn *conn, int err)
{
	if (err != 0) {
		shell_error(ctx_shell, "BASS scan stop failed (%d)", err);
	} else {
		shell_print(ctx_shell, "BASS scan stop successful");
	}
}

static void bap_broadcast_assistant_add_src_cb(struct bt_conn *conn, int err)
{
	if (err != 0) {
		shell_error(ctx_shell, "BASS add source failed (%d)", err);
	} else {
		shell_print(ctx_shell, "BASS add source successful");
	}
}

static void bap_broadcast_assistant_mod_src_cb(struct bt_conn *conn, int err)
{
	if (err != 0) {
		shell_error(ctx_shell, "BASS modify source failed (%d)", err);
	} else {
		shell_print(ctx_shell, "BASS modify source successful");
	}
}

static void bap_broadcast_assistant_broadcast_code_cb(struct bt_conn *conn,
						      int err)
{
	if (err != 0) {
		shell_error(ctx_shell, "BASS broadcast code failed (%d)", err);
	} else {
		shell_print(ctx_shell, "BASS broadcast code successful");
	}
}

static void bap_broadcast_assistant_rem_src_cb(struct bt_conn *conn, int err)
{
	if (err != 0) {
		shell_error(ctx_shell, "BASS remove source failed (%d)", err);
	} else {
		shell_print(ctx_shell, "BASS remove source successful");
	}
}

static struct bt_bap_broadcast_assistant_cb cbs = {
	.discover = bap_broadcast_assistant_discover_cb,
	.scan = bap_broadcast_assistant_scan_cb,
	.recv_state = bap_broadcast_assistant_recv_state_cb,
	.recv_state_removed = bap_broadcast_assistant_recv_state_removed_cb,
	.scan_start = bap_broadcast_assistant_scan_start_cb,
	.scan_stop = bap_broadcast_assistant_scan_stop_cb,
	.add_src = bap_broadcast_assistant_add_src_cb,
	.mod_src = bap_broadcast_assistant_mod_src_cb,
	.broadcast_code = bap_broadcast_assistant_broadcast_code_cb,
	.rem_src = bap_broadcast_assistant_rem_src_cb,
};

static int cmd_bap_broadcast_assistant_discover(const struct shell *sh,
						size_t argc, char **argv)
{
	int result;

	bt_bap_broadcast_assistant_register_cb(&cbs);

	result = bt_bap_broadcast_assistant_discover(default_conn);
	if (result) {
		shell_print(sh, "Fail: %d", result);
	}

	return result;
}

static int cmd_bap_broadcast_assistant_scan_start(const struct shell *sh,
						  size_t argc, char **argv)
{
	int result;
	int start_scan = false;

	if (argc > 1) {
		result = 0;

		start_scan = shell_strtobool(argv[1], 0, &result);
		if (result != 0) {
			shell_error(sh, "Could not parse start_scan: %d",
				    result);

			return -ENOEXEC;
		}
	}

	result = bt_bap_broadcast_assistant_scan_start(default_conn,
						       (bool)start_scan);
	if (result) {
		shell_print(sh, "Fail: %d", result);
	}

	return result;
}

static int cmd_bap_broadcast_assistant_scan_stop(const struct shell *sh,
						 size_t argc, char **argv)
{
	int result;

	result = bt_bap_broadcast_assistant_scan_stop(default_conn);
	if (result) {
		shell_print(sh, "Fail: %d", result);
	}

	return result;
}

static int cmd_bap_broadcast_assistant_add_src(const struct shell *sh,
					       size_t argc, char **argv)
{
	struct bt_bap_broadcast_assistant_add_src_param param = { 0 };
	struct bt_bap_scan_delegator_subgroup subgroup = { 0 };
	unsigned long broadcast_id;
	unsigned long adv_sid;
	int result;

	result = bt_addr_le_from_str(argv[1], argv[2], &param.addr);
	if (result) {
		shell_error(sh, "Invalid peer address (err %d)", result);

		return -ENOEXEC;
	}

	adv_sid = shell_strtoul(argv[3], 0, &result);
	if (result != 0) {
		shell_error(sh, "Could not parse adv_sid: %d", result);

		return -ENOEXEC;
	}

	if (adv_sid > BT_GAP_SID_MAX) {
		shell_error(sh, "Invalid adv_sid: %lu", adv_sid);

		return -ENOEXEC;
	}

	param.adv_sid = adv_sid;

	param.pa_sync = shell_strtobool(argv[4], 0, &result);
	if (result != 0) {
		shell_error(sh, "Could not parse adv_sid: %d", result);

		return -ENOEXEC;
	}

	broadcast_id = shell_strtoul(argv[5], 0, &result);
	if (result != 0) {
		shell_error(sh, "Could not parse broadcast_id: %d", result);

		return -ENOEXEC;
	}

	if (broadcast_id > BT_AUDIO_BROADCAST_ID_MAX) {
		shell_error(sh, "Invalid broadcast_id: %lu", broadcast_id);

		return -ENOEXEC;
	}

	param.broadcast_id = broadcast_id;

	if (argc > 6) {
		unsigned long pa_interval;

		pa_interval = shell_strtoul(argv[6], 0, &result);
		if (result) {
			shell_error(sh, "Could not parse pa_interval: %d",
				    result);

			return -ENOEXEC;
		}

		if (!IN_RANGE(pa_interval,
			      BT_GAP_PER_ADV_MIN_INTERVAL,
			      BT_GAP_PER_ADV_MAX_INTERVAL)) {
			shell_error(sh, "Invalid pa_interval: %lu",
				    pa_interval);

			return -ENOEXEC;
		}

		param.pa_interval = pa_interval;
	} else {
		param.pa_interval = BT_BAP_PA_INTERVAL_UNKNOWN;
	}

	/* TODO: Support multiple subgroups */
	if (argc > 7) {
		unsigned long bis_sync;

		bis_sync = shell_strtoul(argv[7], 0, &result);
		if (result) {
			shell_error(sh, "Could not parse bis_sync: %d", result);

			return -ENOEXEC;
		}

		if (!IN_RANGE(bis_sync, 0, UINT32_MAX)) {
			shell_error(sh, "Invalid bis_sync: %lu", bis_sync);

			return -ENOEXEC;
		}

		subgroup.bis_sync = bis_sync;
	}

	if (argc > 8) {
		size_t metadata_len;

		metadata_len = hex2bin(argv[8], strlen(argv[8]),
				       subgroup.metadata,
				       sizeof(subgroup.metadata));

		if (metadata_len == 0U) {
			shell_error(sh, "Could not parse metadata");

			return -ENOEXEC;
		}

		/* sizeof(subgroup.metadata) can always fit in uint8_t */

		subgroup.metadata_len = metadata_len;
	}

	param.num_subgroups = 1;
	param.subgroups = &subgroup;

	result = bt_bap_broadcast_assistant_add_src(default_conn, &param);
	if (result) {
		shell_print(sh, "Fail: %d", result);
	}

	return result;
}

static int cmd_bap_broadcast_assistant_mod_src(const struct shell *sh,
					       size_t argc, char **argv)
{
	struct bt_bap_broadcast_assistant_mod_src_param param = { 0 };
	struct bt_bap_scan_delegator_subgroup subgroup = { 0 };
	unsigned long src_id;
	int result = 0;

	src_id = shell_strtoul(argv[1], 0, &result);
	if (result != 0) {
		shell_error(sh, "Could not parse src_id: %d", result);

		return -ENOEXEC;
	}

	if (src_id > UINT8_MAX) {
		shell_error(sh, "Invalid src_id: %lu", src_id);

		return -ENOEXEC;
	}

	param.pa_sync = shell_strtobool(argv[2], 0, &result);
	if (result != 0) {
		shell_error(sh, "Could not parse adv_sid: %d", result);

		return -ENOEXEC;
	}

	if (argc > 3) {
		unsigned long pa_interval;

		pa_interval = shell_strtoul(argv[3], 0, &result);
		if (result) {
			shell_error(sh, "Could not parse pa_interval: %d", result);

			return -ENOEXEC;
		}

		if (!IN_RANGE(pa_interval,
			      BT_GAP_PER_ADV_MIN_INTERVAL,
			      BT_GAP_PER_ADV_MAX_INTERVAL)) {
			shell_error(sh, "Invalid pa_interval: %lu", pa_interval);

			return -ENOEXEC;
		}

		param.pa_interval = pa_interval;
	} else {
		param.pa_interval = BT_BAP_PA_INTERVAL_UNKNOWN;
	}

	/* TODO: Support multiple subgroups */
	if (argc > 4) {
		unsigned long bis_sync;

		bis_sync = shell_strtoul(argv[4], 0, &result);
		if (result) {
			shell_error(sh, "Could not parse bis_sync: %d", result);

			return -ENOEXEC;
		}

		if (!IN_RANGE(bis_sync, 0, UINT32_MAX)) {
			shell_error(sh, "Invalid bis_sync: %lu", bis_sync);

			return -ENOEXEC;
		}

		subgroup.bis_sync = bis_sync;
	}

	if (argc > 5) {
		size_t metadata_len;

		metadata_len = hex2bin(argv[5], strlen(argv[5]),
				       subgroup.metadata,
				       sizeof(subgroup.metadata));

		if (metadata_len == 0U) {
			shell_error(sh, "Could not parse metadata");

			return -ENOEXEC;
		}

		/* sizeof(subgroup.metadata) can always fit in uint8_t */

		subgroup.metadata_len = metadata_len;
	}

	param.num_subgroups = 1;
	param.subgroups = &subgroup;

	result = bt_bap_broadcast_assistant_mod_src(default_conn, &param);
	if (result) {
		shell_print(sh, "Fail: %d", result);
	}

	return result;
}

static int cmd_bap_broadcast_assistant_broadcast_code(const struct shell *sh,
						      size_t argc, char **argv)
{
	uint8_t broadcast_code[BT_AUDIO_BROADCAST_CODE_SIZE] = { 0 };
	size_t broadcast_code_len;
	unsigned long src_id;
	int result = 0;

	src_id = shell_strtoul(argv[1], 0, &result);
	if (result != 0) {
		shell_error(sh, "Could not parse src_id: %d", result);

		return -ENOEXEC;
	}

	if (src_id > UINT8_MAX) {
		shell_error(sh, "Invalid src_id: %lu", src_id);

		return -ENOEXEC;
	}

	broadcast_code_len = hex2bin(argv[2], strlen(argv[2]),
				     broadcast_code,
				     sizeof(broadcast_code));

	if (broadcast_code_len == 0U) {
		shell_error(sh, "Could not parse broadcast code");

		return -ENOEXEC;
	}

	result = bt_bap_broadcast_assistant_set_broadcast_code(default_conn,
							       src_id,
							       broadcast_code);
	if (result) {
		shell_print(sh, "Fail: %d", result);
	}

	return result;
}

static int cmd_bap_broadcast_assistant_rem_src(const struct shell *sh,
					       size_t argc, char **argv)
{
	unsigned long src_id;
	int result = 0;

	src_id = shell_strtoul(argv[1], 0, &result);
	if (result != 0) {
		shell_error(sh, "Could not parse src_id: %d", result);

		return -ENOEXEC;
	}

	if (src_id > UINT8_MAX) {
		shell_error(sh, "Invalid src_id: %lu", src_id);

		return -ENOEXEC;
	}

	result = bt_bap_broadcast_assistant_rem_src(default_conn, src_id);
	if (result) {
		shell_print(sh, "Fail: %d", result);
	}

	return result;
}

static int cmd_bap_broadcast_assistant_read_recv_state(const struct shell *sh,
						       size_t argc, char **argv)
{
	unsigned long idx;
	int result = 0;

	idx = shell_strtoul(argv[1], 0, &result);
	if (result != 0) {
		shell_error(sh, "Could not parse idx: %d", result);

		return -ENOEXEC;
	}

	if (idx > UINT8_MAX) {
		shell_error(sh, "Invalid idx: %lu", idx);

		return -ENOEXEC;
	}

	result = bt_bap_broadcast_assistant_read_recv_state(default_conn, idx);
	if (result) {
		shell_print(sh, "Fail: %d", result);
	}

	return result;
}

static int cmd_bap_broadcast_assistant(const struct shell *sh, size_t argc,
				       char **argv)
{
	if (argc > 1) {
		shell_error(sh, "%s unknown parameter: %s",
			    argv[0], argv[1]);
	} else {
		shell_error(sh, "%s Missing subcommand", argv[0]);
	}

	return -ENOEXEC;
}

SHELL_STATIC_SUBCMD_SET_CREATE(bap_broadcast_assistant_cmds,
	SHELL_CMD_ARG(discover, NULL,
		      "Discover BASS on the server",
		      cmd_bap_broadcast_assistant_discover, 1, 0),
	SHELL_CMD_ARG(scan_start, NULL,
		      "Start scanning for broadcasters",
		      cmd_bap_broadcast_assistant_scan_start, 1, 0),
	SHELL_CMD_ARG(scan_stop, NULL,
		      "Stop scanning for BISs",
		      cmd_bap_broadcast_assistant_scan_stop, 1, 0),
	SHELL_CMD_ARG(add_src, NULL,
		      "Add a source <address: XX:XX:XX:XX:XX:XX> "
		      "<type: public/random> <adv_sid> <sync_pa> "
		      "<broadcast_id> [<pa_interval>] [<sync_bis>] "
		      "[<metadata>]",
		      cmd_bap_broadcast_assistant_add_src, 6, 3),
	SHELL_CMD_ARG(mod_src, NULL,
		      "Set sync <src_id> <sync_pa> [<pa_interval>] "
		      "[<sync_bis>] [<metadata>]",
		      cmd_bap_broadcast_assistant_mod_src, 3, 2),
	SHELL_CMD_ARG(broadcast_code, NULL,
		      "Send a space separated broadcast code of up to 16 bytes "
		      "<src_id> [broadcast code]",
		      cmd_bap_broadcast_assistant_broadcast_code, 2, 16),
	SHELL_CMD_ARG(rem_src, NULL,
		      "Remove a source <src_id>",
		      cmd_bap_broadcast_assistant_rem_src, 2, 0),
	SHELL_CMD_ARG(read_state, NULL,
		      "Remove a source <index>",
		      cmd_bap_broadcast_assistant_read_recv_state, 2, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_ARG_REGISTER(bap_broadcast_assistant, &bap_broadcast_assistant_cmds,
		       "Bluetooth BAP broadcast assistant client shell commands",
		       cmd_bap_broadcast_assistant, 1, 1);
