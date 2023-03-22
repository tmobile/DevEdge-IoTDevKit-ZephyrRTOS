/** @file
 * @brief Modem shell module
 *
 * Provide some modem shell commands that can be useful to applications.
 */

/*
 * Copyright (c) 2018 Foundries.io
 * Copyright (c) 2022 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME modem_shell

#include "modem_sms.h"

#include <zephyr/kernel.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/console/uart_mux.h>

#include <zephyr/sys/printk.h>

struct modem_shell_user_data {
	const struct shell *shell;
	void *user_data;
};

#if defined(CONFIG_MODEM_CONTEXT)
#include "modem_context.h"
#define ms_context     modem_context
#define ms_max_context CONFIG_MODEM_CONTEXT_MAX_NUM
#define ms_send(ctx_, buf_, size_)                                             \
	(ctx_->iface.write(&ctx_->iface, buf_, size_))
#define ms_context_from_id modem_context_from_id
#define UART_DEV_NAME(ctx) (ctx->iface.dev->name)
#elif defined(CONFIG_MODEM_RECEIVER)
#include "modem_receiver.h"
#define ms_context	    mdm_receiver_context
#define ms_max_context	    CONFIG_MODEM_RECEIVER_MAX_CONTEXTS
#define ms_send		    mdm_receiver_send
#define ms_context_from_id  mdm_receiver_context_from_id
#define UART_DEV_NAME(ctx_) (ctx_->uart_dev->name)
#else
#error "MODEM_CONTEXT or MODEM_RECEIVER need to be enabled"
#endif

static int cmd_modem_list(const struct shell *shell, size_t argc, char *argv[])
{
	struct ms_context *mdm_ctx;
	int i, count = 0;

	shell_fprintf(shell, SHELL_NORMAL, "Modem receivers:\n");

	for (i = 0; i < ms_max_context; i++) {
		mdm_ctx = ms_context_from_id(i);
		if (mdm_ctx) {
			count++;
			shell_fprintf(
				shell, SHELL_NORMAL,
				"%d:\tIface Device: %s\n"
				"\tManufacturer: %s\n"
				"\tModel:        %s\n"
				"\tRevision:     %s\n"
				"\tIMEI:         %s\n"
#if defined(CONFIG_MODEM_SIM_NUMBERS)
				"\tIMSI:         %s\n"
				"\tICCID:        %s\n"
#endif
#if defined(CONFIG_MODEM_CELL_INFO)
				"\tOperator:     %d\n"
				"\tLAC:          %d\n"
				"\tCellId:       %d\n"
				"\tAcT:          %d\n"
#endif
				"\tRSSI:         %d\n",
			       i,
			       UART_DEV_NAME(mdm_ctx),
			       mdm_ctx->data_manufacturer,
			       mdm_ctx->data_model,
			       mdm_ctx->data_revision,
			       mdm_ctx->data_imei,
#if defined(CONFIG_MODEM_SIM_NUMBERS)
			       mdm_ctx->data_imsi,
			       mdm_ctx->data_iccid,
#endif
#if defined(CONFIG_MODEM_CELL_INFO)
			       mdm_ctx->data_operator,
			       mdm_ctx->data_lac,
			       mdm_ctx->data_cellid,
			       mdm_ctx->data_act,
#endif
			       mdm_ctx->data_rssi ? *mdm_ctx->data_rssi : 0);
		}
	}

	if (!count) {
		shell_fprintf(shell, SHELL_NORMAL, "None found.\n");
	}

	return 0;
}

static int cmd_modem_send(const struct shell *shell, size_t argc, char *argv[])
{
	struct ms_context *mdm_ctx;
	char *endptr;
	int ret, i, arg = 1;

	/* list */
	if (!argv[arg]) {
		shell_fprintf(shell, SHELL_ERROR,
			      "Please enter a modem index\n");
		return -EINVAL;
	}

	/* <index> of modem receiver */
	i = (int)strtol(argv[arg], &endptr, 10);
	if (*endptr != '\0') {
		shell_fprintf(shell, SHELL_ERROR,
			      "Please enter a modem index\n");
		return -EINVAL;
	}

	mdm_ctx = ms_context_from_id(i);
	if (!mdm_ctx) {
		shell_fprintf(shell, SHELL_ERROR, "Modem receiver not found!");
		return 0;
	}

	for (i = arg + 1; i < argc; i++) {
		ret = ms_send(mdm_ctx, argv[i], strlen(argv[i]));
		if (ret < 0) {
			shell_fprintf(shell, SHELL_ERROR,
				      "Error sending '%s': %d\n", argv[i], ret);
			return 0;
		}

		if (i == argc - 1) {
			ret = ms_send(mdm_ctx, "\r", 1);
		} else {
			ret = ms_send(mdm_ctx, " ", 1);
		}

		if (ret < 0) {
			shell_fprintf(shell, SHELL_ERROR,
				      "Error sending (CRLF or space): %d\n",
				      ret);
			return 0;
		}
	}

	return 0;
}

#if defined(CONFIG_GSM_MUX)
static void uart_mux_cb(const struct device *uart, const struct device *dev,
			int dlci_address, void *user_data)
{
	struct modem_shell_user_data *data = user_data;
	const struct shell *shell = data->shell;
	int *count = data->user_data;
	const char *ch = "?";

	if (*count == 0) {
		shell_fprintf(shell, SHELL_NORMAL,
			      "\nReal UART\tMUX UART\tDLCI\n");
	}

	(*count)++;

	if (dlci_address == CONFIG_GSM_MUX_DLCI_AT) {
		ch = "AT";
	} else if (dlci_address == CONFIG_GSM_MUX_DLCI_PPP) {
		ch = "PPP";
	} else if (dlci_address == 0) {
		ch = "control";
	}

	shell_fprintf(shell, SHELL_NORMAL, "%s\t\t%s\t\t%d (%s)\n", uart->name,
		      dev->name, dlci_address, ch);
}
#endif

static int cmd_modem_info(const struct shell *shell, size_t argc, char *argv[])
{
	struct ms_context *mdm_ctx;
	char *endptr;
	int i, arg = 1;

	/* info */
	if (!argv[arg]) {
		shell_fprintf(shell, SHELL_ERROR,
			      "Please enter a modem index\n");
		return -EINVAL;
	}

	/* <index> of modem receiver */
	i = (int)strtol(argv[arg], &endptr, 10);
	if (*endptr != '\0') {
		shell_fprintf(shell, SHELL_ERROR,
			      "Please enter a modem index\n");
		return -EINVAL;
	}

	mdm_ctx = ms_context_from_id(i);
	if (!mdm_ctx) {
		shell_fprintf(shell, SHELL_ERROR, "Modem receiver not found!");
		return 0;
	}

	shell_fprintf(shell, SHELL_NORMAL,
		      "Modem index      : %d\n"
		      "Iface Device     : %s\n"
		      "Manufacturer     : %s\n"
		      "Model            : %s\n"
		      "Revision         : %s\n"
		      "IMEI             : %s\n"
		      "RSSI             : %d\n",
		      i, UART_DEV_NAME(mdm_ctx), mdm_ctx->data_manufacturer,
		      mdm_ctx->data_model, mdm_ctx->data_revision,
		      mdm_ctx->data_imei,
		      mdm_ctx->data_rssi ? *mdm_ctx->data_rssi : 0);

	shell_fprintf(shell, SHELL_NORMAL, "GSM 07.10 muxing : %s\n",
		      IS_ENABLED(CONFIG_GSM_MUX) ? "enabled" : "disabled");

#if defined(CONFIG_GSM_MUX)
	struct modem_shell_user_data user_data;
	int count = 0;

	user_data.shell = shell;
	user_data.user_data = &count;

	uart_mux_foreach(uart_mux_cb, &user_data);
#endif

	return 0;
}

static int cmd_modem_sms_send(const struct shell *shell, size_t argc,
			      char *argv[])
{
	struct ms_context *ms_ctx;
	struct sms_out sms;
	int ret = -1;

	if (argc != 4) {
		shell_fprintf(shell, SHELL_ERROR,
			      "usage: modem sms send <modem index> <phone "
			      "number> <message>\n");
	} else {
		int modem_index = atoi(argv[1]);

		ms_ctx = ms_context_from_id(modem_index);
		if (ms_ctx == NULL) {
			shell_fprintf(
				shell, SHELL_ERROR,
				"Couldn't open context for modem index %d\n",
				modem_index);
			return -ENOEXEC;
		}

		if (ms_ctx->send_sms == NULL) {
			shell_fprintf(shell, SHELL_ERROR,
				      "No function defined to send SMS\n");
			return -ENOEXEC;
		}

		if (strlen(argv[3]) > (sizeof(sms.msg) - 1)) {
			shell_fprintf(shell, SHELL_WARNING,
					  "Specified message longer than maximum, truncating message\n");
		}

		snprintk(sms.phone, sizeof(sms.phone), "%s", argv[2]);
		snprintk(sms.msg, sizeof(sms.msg), "%s", argv[3]);

		ret = ms_ctx->send_sms(ms_ctx, &sms);
		if (ret == 0) {
			shell_fprintf(shell, SHELL_NORMAL,
				      "SMS msg was sent\n");
		} else {
			shell_fprintf(shell, SHELL_ERROR,
				      "error: send_sms returned %d\n", ret);
		}
	}
	return ret;
}

static int cmd_modem_sms_recv(const struct shell *shell, size_t argc,
			      char *argv[])
{
	struct ms_context *ms_ctx;
	struct sms_in sms;
	int ret = -1;

	if (argc != 3) {
		shell_fprintf(shell, SHELL_ERROR,
			      "usage: modem sms recv <modem index> <wait time "
			      "(seconds)>\n");
	} else {
		int modem_index = atoi(argv[1]);
		int wait = atoi(argv[2]);

		ms_ctx = ms_context_from_id(modem_index);
		if (ms_ctx == NULL) {
			shell_fprintf(
				shell, SHELL_ERROR,
				"Couldn't open context for modem index %d\n",
				modem_index);
			return -ENOEXEC;
		}

		if (ms_ctx->recv_sms == NULL) {
			shell_fprintf(shell, SHELL_ERROR,
				      "No function defined to receive SMS\n");
			return -ENOEXEC;
		}

		sms.timeout = K_SECONDS(wait);
		ret = ms_ctx->recv_sms(ms_ctx, &sms);
		if (ret < 0) {
			shell_fprintf(shell, SHELL_ERROR,
				      "recv_sms returned error %d, errno:%d\n",
				      ret, errno);
		} else if (ret == 0) {
			shell_fprintf(shell, SHELL_NORMAL,
				      "No SMS msgs available\n");
		} else {
			shell_fprintf(
				shell, SHELL_NORMAL,
				"Received SMS msg from %s dated %s: '%s'\n",
				sms.phone, sms.time, sms.msg);
		}
	}
	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(modem_cmd_sms,
			       SHELL_CMD(send, NULL,
					 "'modem sms send <modem index> <phone "
					 "number> <message>' send sms message",
					 cmd_modem_sms_send),
			       SHELL_CMD(recv, NULL,
					 "'modem sms recv <modem index> <wait "
					 "time (seconds)>' receive sms message",
					 cmd_modem_sms_recv),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_modem,
	SHELL_CMD(info, NULL, "Show information for a modem", cmd_modem_info),
	SHELL_CMD(list, NULL, "List registered modems", cmd_modem_list),
	SHELL_CMD(send, NULL,
		  "Send an AT <command> to a registered modem "
		  "receiver",
		  cmd_modem_send),
	SHELL_CMD(sms, &modem_cmd_sms, "Send or receive SMS message via modem",
		  NULL),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(modem, &sub_modem, "Modem commands", NULL);
