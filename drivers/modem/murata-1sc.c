/*
 * Copyright (c) 2022 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT murata_1sc
#define MURATA_1SC_C

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_murata_1sc, CONFIG_MODEM_LOG_LEVEL);

#include "modem_cmd_handler.h"
#include "modem_context.h"
#include "modem_iface_uart.h"
#include "modem_receiver.h"
#include "modem_socket.h"
#include "modem_sms.h"
#include "murata-1sc.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/posix/fcntl.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_offload.h>
#include <zephyr/net/socket_offload.h>
#include <zephyr/drivers/modem/sms.h>
#include <zephyr/net/offloaded_netdev.h>
#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
#include "tls_internal.h"

#include <zephyr/net/tls_credentials.h>
#include <zephyr/sys/base64.h>
#endif
#if defined(CONFIG_PM_DEVICE)
#include <zephyr/pm/device.h>
#endif

#define MAX_FILENAME_LEN      32
#define MDM_BOOT_DELAY	      6	   /* seconds */
#define MDM_WAKE_DELAY	      5000 /* milliseconds */
#define MDM_CMD_RSP_TIME      K_SECONDS(2)
#define MDM_CMD_LONG_RSP_TIME K_SECONDS(6)

const struct mdmdata_cmd_t cmd_pool[] = {{"APN", apn_e},
					 {"ASLEEP", awake_e},
					 {"AWAKE", awake_e},
					 {"CONN_STS", connsts_e},
					 {"CONN", connsts_e},
					 {"EDRX", edrx_e},
					 {"GOLD", golden_e},
					 {"GOLDEN", golden_e},
					 {"ICCID", iccid_e},
					 {"IMEI", imei_e},
					 {"IMSI", imsi_e},
					 {"IP", ip_e},
					 {"IP6", ip6_e},
					 {"MSISDN", msisdn_e},
					 {"PSM", psm_e},
					 {"SLEEP", sleep_e},
					 {"SSI", ssi_e},
					 {"STAT", connsts_e},
					 {"SIM", sim_info_e},
					 {"VERSION", version_e},
					 {"VER", version_e},
					 {"WAKE", wake_e},
					 {}};

enum sms_tp_flags {
	TP_FLAG_MMS = BIT(2),
	TP_FLAG_RP = BIT(7),
	TP_FLAG_UDHI = BIT(6),
	TP_FLAG_SRI = BIT(5)
};

enum sms_type_of_number {
	SMS_TON_UNKNOWN = 0,
	SMS_TON_INTERNATIONAL,
	SMS_TON_NATIONAL,
	SMS_TON_NETWORK_SPECIFIC,
	SMS_TON_SUBSCRIBER,
	SMS_TON_ALPHANUMERIC,
	SMS_TON_ABBREVIATED,
	SMS_TON_RESERVED,
};

enum sms_alphabet {
	SMS_ALPHABET_GSM7 = 0,
	SMS_ALPHABET_GSM8,
	SMS_ALPHABET_UCS2,
};

/* Structure for storing information about a SMS-DELIVER PDU */
struct deliver_pdu_data_s {
	uint8_t smsc_len; /* Length of SMSC segment */
	char *smsc_start; /* SMSC segment */
	uint8_t tp_flags; /* Flags */
	uint8_t oa_len;	  /* Originator address length */
	char *oa;	  /* Originator address */
	uint8_t alphabet; /* Alphabet used */
	char *scts;	  /* Timestamp */
	uint8_t udl;	  /* User data length */
	uint8_t udhl;	  /* User data header length */
	char *ud;	  /* User data */
};

struct csms_data_s {
	uint8_t csms_ref;
	uint8_t csms_idx;
	uint8_t csms_tot;
};

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)

#define CERTCMD_WRITE_SIZE 32 + MAX_FILENAME_LEN /* assume filename maxlen = 32 */
#define PEM_BUFF_SIZE	   3002			 /* terminate with \" & 0 */

/**
 * following struct may not have packed memory if it has something like
 * int follow by char then int,
 * since array of uchar plus array of uchar will be packed
 */
struct cert_cmd_t {
	uint8_t cert_cmd_write[CERTCMD_WRITE_SIZE];
	uint8_t pem_buf[PEM_BUFF_SIZE];
};

struct mdm_sock_tls_s {
	char host[CONFIG_MURATA_MODEM_SNI_BUFFER_SZ + 1];
	uint8_t profile;
	bool sni_valid;
	bool peer_verify_disable;
	bool client_verify;
} murata_sock_tls_info[MDM_MAX_SOCKETS] = {{{0}, 0, false, false, false}};
#endif

/**
 * @brief Convert a series of uint8_t or byte to ascii hex value in a string
 */
static size_t data_to_hex_str(const void *input_buf, size_t input_len, char *output_buf,
			      size_t output_len)
{
	size_t i;

	for (i = 0; (i < (output_len - 1) / 2) && (i < input_len); i++) {
		snprintk(&output_buf[(i * 2)], output_len, "%02X", ((uint8_t *)input_buf)[i]);
	}

	return i * 2;
}

/**
 * @brief Convert ascii hex number representation to uint8_t representation
 */
static uint8_t nibble_to_data(char nibble)
{
	if (nibble >= '0' && nibble <= '9')
		return nibble - '0';
	else if (nibble >= 'A' && nibble <= 'F')
		return nibble - 'A' + 10;
	else if (nibble >= 'a' && nibble <= 'f')
		return nibble - 'a' + 10;

	return 0;
}

/**
 * @brief Convert uint8_t or byte to a ascii hex number representation.
 */
static uint8_t hex_byte_to_data(const char *hex_bytes)
{
	return nibble_to_data(*hex_bytes) * 0x10 + nibble_to_data(*(hex_bytes + 1));
}

/**
 * @brief Convert uint8_t to a binary ascii (characters of either 1 or 0).
 */
static size_t hex_str_to_data(const char *input_buf, uint8_t *output_buf, size_t output_len)
{
	size_t str_len = strlen(input_buf);
	size_t i = 0;

	for (i = 0; (i < output_len) && (i * 2 < str_len); i++) {
		output_buf[i] = hex_byte_to_data(&input_buf[i * 2]);
	}
	return i;
}

/**
 * @brief Convert uint8_t to a binary ascii (string with 1 and 0).
 */
static char *byte_to_binary_str(uint8_t byte)
{
	static char buf[9] = {0};

	memset(buf, 0, sizeof(buf));
	for (int i = 0; i < 8; i++) {
		buf[7 - i] = (byte & 1 << i) ? '1' : '0';
	}

	return buf;
}

/* Modem pins - Wake Host, Wake Modem, Reset, and Reset Done */
static const struct gpio_dt_spec wake_host_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_wake_host_gpios);
static const struct gpio_dt_spec wake_mdm_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_wake_mdm_gpios);
static const struct gpio_dt_spec reset_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_reset_gpios);
static const struct gpio_dt_spec rst_done_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_rst_done_gpios);
static const struct gpio_dt_spec mdm_rx_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_rx_gpios);
static const struct gpio_dt_spec mdm_tx_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_tx_gpios);
static const struct gpio_dt_spec mdm_rts_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_rts_gpios);
static const struct gpio_dt_spec mdm_cts_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_cts_gpios);

static struct k_thread modem_rx_thread;
static struct murata_1sc_data mdata;
static struct modem_context mctx;
static const struct socket_op_vtable offload_socket_fd_op_vtable;

static void socket_close(struct modem_socket *sock);
static const char s_hifc_mode = 'A';
static const char s_max_pm_mode[] = "dh0";
static const bool s_sleep_mode = true;

/* RX thread structures */
static K_KERNEL_STACK_DEFINE(modem_rx_stack, CONFIG_MODEM_MURATA_1SC_RX_STACK_SIZE);
NET_BUF_POOL_DEFINE(mdm_recv_pool, MDM_RECV_MAX_BUF, MDM_RECV_BUF_SIZE, 0, NULL);

/**
 * @brief Thread to process all messages received from the Modem.
 */
static void murata_1sc_rx(void)
{
	while (true) {
		/* Wait for incoming data */
		modem_iface_uart_rx_wait(&mctx.iface, K_FOREVER);

		modem_cmd_handler_process(&mctx.cmd_handler, &mctx.iface);
	}
}

/**
 * @brief Convert string to long integer, but handle errors
 */
static int murata_1sc_atoi(const char *s, const int err_value, const char *desc, const char *func)
{
	int ret;
	char *endptr;

	ret = (int)strtol(s, &endptr, 10);
	if (!endptr || *endptr != '\0') {
		LOG_ERR("bad %s '%s' in %s", s, desc, func);
		return err_value;
	}

	return ret;
}

/**
 * @brief Convert ascii hex to uint8_t
 */
static inline uint8_t hex_char_to_int(char ch)
{
	uint8_t ret;

	if (ch >= '0' && ch <= '9') {
		ret = ch - '0';
	} else if (ch > 'a' && ch < 'f') {
		ret = 0xa + ch - 'a';
	} else if (ch > 'A' && ch < 'F') {
		ret = 0xa + ch - 'A';
	} else {
		ret = 0;
	}
	return ret;
}

/**
 * @brief Derive a MAC address from the IMEI
 */
static inline uint8_t *murata_1sc_get_mac(const struct device *dev)
{
	struct murata_1sc_data *data = dev->data;

	/* We will use the least significant 12 digits of the IMEI as
	 * the MAC address. To explain further...
	 * IMEI is always 15 digits long. The constant, MDM_IMEI_LENGTH
	 * is 16 (bytes long) to hold the 15-digit IMEI plus terminator.
	 * MAC address is always 6 bytes long (ie, 12 hex characters).
	 * The loop below is calculating each of the MAC's 6 bytes
	 */
	for (int i = 0; i < 6; i++) {
		int imei_idx = (MDM_IMEI_LENGTH - 1) - 12 + (i * 2);

		data->mac_addr[i] = (hex_char_to_int(mdata.mdm_imei[imei_idx]) << 4) |
				    (hex_char_to_int(mdata.mdm_imei[imei_idx + 1]));
	}
	return data->mac_addr;
}

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
/**
 * @brief Get the index from a specific socket pointer
 *
 * @param sock Socket pointer
 * @return int Index of socket
 */
static int get_socket_idx(struct modem_socket *sock)
{
	for (int i = 0; i < mdata.socket_config.sockets_len; i++)
		if (&mdata.socket_config.sockets[i] == sock)
			return i;
	return -1;
}
#endif

/**
 * @brief Handler for OK
 */
MODEM_CMD_DEFINE(on_cmd_ok)
{
	modem_cmd_handler_set_error(data, 0);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/**
 * @brief Handler for ERROR
 */
MODEM_CMD_DEFINE(on_cmd_error)
{
	modem_cmd_handler_set_error(data, -EIO);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/**
 * @brief Handler for sock sentdata
 */
MODEM_CMD_DEFINE(on_cmd_sock_sentdata)
{
	if (argc < 2) {
		return -EAGAIN;
	}

	int data_len = (int)strtol(argv[1], NULL, 10);
	return data_len;
}

/**
 * @brief Send data over the given socket
 */
static ssize_t send_socket_data(struct modem_socket *sock, const struct sockaddr *dst_addr,
				const char *buf, const size_t buf_len, k_timeout_t timeout)
{
	int ret = -1;

	k_sem_take(&mdata.sem_xlate_buf, K_FOREVER);

	int len;
	int written;

	/* Modem command to read the data. */
	struct modem_cmd data_cmd[] = {MODEM_CMD("ERROR", on_cmd_error, 0U, ""),
				       MODEM_CMD("%SOCKETDATA:", on_cmd_sock_sentdata, 2U, ",")};

	if (buf_len > MDM_MAX_DATA_LENGTH) {
		errno = EMSGSIZE;
		k_sem_give(&mdata.sem_xlate_buf);
		return -1;
	}

	len = MIN(buf_len, MDM_MAX_DATA_LENGTH);

	/* Create the command prefix */
	written = snprintk(mdata.xlate_buf, sizeof(mdata.xlate_buf),
			   "AT%%SOCKETDATA=\"SEND\",%d,%zu,\"", sock->id, len);

	/* Add the hex string */
	data_to_hex_str(buf, len, &mdata.xlate_buf[written], sizeof(mdata.xlate_buf) - written);

	/* Finish the command */
	snprintk(&mdata.xlate_buf[written + len * 2], sizeof(mdata.xlate_buf), "\"");

	written += len * 2;
	written++;

	if (dst_addr) {
		char addr_buf[NET_IPV6_ADDR_LEN];
		uint16_t port;
		void *addr_ptr;

		if (dst_addr->sa_family == AF_INET) {
			addr_ptr = &net_sin(dst_addr)->sin_addr;
			port = ntohs(net_sin(dst_addr)->sin_port);
		} else {
			addr_ptr = &net_sin6(dst_addr)->sin6_addr;
			port = ntohs(net_sin6(dst_addr)->sin6_port);
		}
		net_addr_ntop(dst_addr->sa_family, addr_ptr, addr_buf, sizeof(addr_buf));
		written += snprintk(&mdata.xlate_buf[written], sizeof(mdata.xlate_buf) - written,
			",\"%s\",%d", addr_buf, port);
	}

	/* Send the command */
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, ARRAY_SIZE(data_cmd),
			     mdata.xlate_buf, &mdata.sem_response, MDM_CMD_LONG_RSP_TIME);

	k_sem_give(&mdata.sem_xlate_buf);

	/* unset handler commands and ignore any errors */
	(void)modem_cmd_handler_update_cmds(&mdata.cmd_handler_data, NULL, 0U, false);

	/* Return the amount of data written on the socket. */
	if (ret < 0) {
		errno = -ret;
		ret = -1;
	} else {
		errno = 0;
		ret = len;
	}
	return ret;
}

/**
 * @brief Read data on a given socket
 */
static int on_cmd_sockread_common(int socket_fd, struct modem_cmd_handler_data *data,
				  int socket_data_length, uint16_t len)
{
	struct modem_socket *sock = NULL;
	struct socket_read_data *sock_data;
	int ret = 0;

	sock = modem_socket_from_id(&mdata.socket_config, socket_fd);
	if (!sock) {
		LOG_ERR("Socket not found! (%d)", socket_fd);
		ret = -EINVAL;
		goto exit;
	}

	/* Make sure we still have buf data */
	if (!data->rx_buf) {
		LOG_ERR("Incorrect format! Ignoring data!");
		return -EINVAL;
	}

	/* check to make sure we have all of the data (minus quotes)
	 * if ((net_buf_frags_len(data->rx_buf) - 2) < socket_data_length) {
	 * LOG_DBG("Not enough data -- wait!");
	 * return -EAGAIN;
	 * }
	 */

	/* skip quote /" */
	len -= 1;
	net_buf_pull_u8(data->rx_buf);
	if (!data->rx_buf->len) {
		data->rx_buf = net_buf_frag_del(NULL, data->rx_buf);
	}

	sock_data = (struct socket_read_data *)sock->data;
	if (!sock_data) {
		LOG_ERR("Socket data not found! Skip handling (%d)", socket_fd);
		ret = -EINVAL;
		goto exit;
	}

	ret = net_buf_linearize(sock_data->recv_buf, sock_data->recv_buf_len, data->rx_buf, 0,
				(uint16_t)(socket_data_length * 2));
	LOG_DBG("net_buf_linearize returned %d", ret);

	data->rx_buf = net_buf_skip(data->rx_buf, ret);
	sock_data->recv_read_len = socket_data_length;

	ret /= 2;
	if ((ret) != socket_data_length) {
		LOG_ERR("Total copied data is different then received data!"
			" copied:%d vs. received:%d",
			ret, socket_data_length);
		ret = -EINVAL;
	}

exit:
	/* remove packet from list (ignore errors) */
	(void)modem_socket_packet_size_update(&mdata.socket_config, sock, -socket_data_length);

	return ret;
}

#if defined(CONFIG_MODEM_SMS)

/**
 * @brief Logic to unpack septets
 *
 * @param frag Packed septet framents
 * @param frag_len Length of packed fragments
 * @param out Output: Septets, unpacked into an octet (byte) array
 */
void gsmunpack_frag(uint8_t *frag, int frag_len, uint8_t *out)
{
	switch (frag_len) {
	case 7:
		out[7] = frag[6] >> 1;
		out[6] = (frag[5] >> 2) | ((frag[6] & 0x01) << 6);
	case 6:
		out[5] = (frag[4] >> 3) | ((frag[5] & 0x03) << 5);
	case 5:
		out[4] = (frag[3] >> 4) | ((frag[4] & 0x07) << 4);
	case 4:
		out[3] = (frag[2] >> 5) | ((frag[3] & 0x0F) << 3);
	case 3:
		out[2] = (frag[1] >> 6) | ((frag[2] & 0x1F) << 2);
	case 2:
		out[1] = (frag[0] >> 7) | ((frag[1] & 0x3F) << 1);
	case 1:
		out[0] = frag[0] & 0x7F;
	}
}

/**
 * @brief Logic for converting GSM 7-bit characters into ASCII
 *
 * @param gsm GSM septet
 * @param escaped Flag for if the previous character was an escape character
 * @return char ASCII equivalent character
 */
char gsm2ascii(uint8_t gsm, bool escaped)
{
	if (escaped) {
		switch (gsm) {
		case 0x0A:
			return '\f';
		case 0x14:
			return '^';
		case 0x28:
			return '{';
		case 0x29:
			return '}';
		case 0x2F:
			return '\\';
		case 0x3c:
			return '[';
		case 0x3d:
			return '~';
		case 0x3e:
			return ']';
		case 0x40:
			return '|';
		default:
			return '\0';
		}
	}
	if ((gsm >= 'a' && gsm <= 'z') || (gsm >= 'A' && gsm <= 'Z') || (gsm >> 4) == 3 ||
	    (((gsm >> 4) == 2) && gsm != 0x24)) {
		return gsm;
	}
	switch (gsm) {
	case 0x00:
		return '@';
	case 0x02:
		return '$';
	case '\n':
	case '\r':
		return gsm;
	case 0x11:
		return '_';
	default:
		return '\0';
	}
}

/**
 * @brief Combined logic for converting a septet payload into an ASCII string
 *
 * @param in Binary encoding of the septet payload/SMS User Data
 * @param udl Length of septet payload/SMS User Data
 * @param out Buffer for output
 * @param outlen Length of output buffer
 * @param skip Number of septets to skip
 * @return int 0 on success
 */
int gsm7_decode(char *in, int udl, char *out, int outlen, int skip)
{
	uint8_t packed[7], unpacked[8];
	uint8_t processed = 0, escaped_cnt = 0;
	uint8_t udl_octets = ((udl * 7 + 7) / 8);
	int skip_drp = skip;
	char *out_orig = out;
	bool escaped = false;

	for (int i = 0; i < udl_octets; i += 7) {
		memset(packed, 0, 7);
		hex_str_to_data(&in[i * 2], packed, MIN(7, udl_octets - processed));
		gsmunpack_frag(packed, MIN(7, udl_octets - processed), unpacked);
		processed += 7;
		for (int j = 0; j < 8; j++) {
			if (skip) {
				skip--;
			} else if (unpacked[j] == 0x1b) {
				escaped = true;
				escaped_cnt++;
			} else {
				if (out > (out_orig + outlen - 1)) {
					return 1;
				}
				char chr = gsm2ascii(unpacked[j], escaped);

				if (chr) {
					*out = chr;
					out++;
				}
				escaped = false;
			}
		}
	}
	out_orig[MIN((udl - escaped_cnt - skip_drp), outlen)] = '\0';
	return 0;
}

/**
 * @brief Function for converting a UTF-16LE encoded character into UTF-8
 *
 * @param in Pointer to UTF-16 character
 * @param out Pointer to UTF-8 output buffer
 * @param out_len Length of output buffer
 * @param next_char Pointer to new position in output buffer
 * @return int 0 on success else negative errno code
 */
int utf16le_to_utf8(uint16_t *in, uint8_t *out, size_t out_len, uint16_t **next_char)
{
	uint32_t codepoint;

	if (!out || !in || !out_len || !*in) {
		return -EINVAL;
	}
	if (in[1] != 0 && ((sys_le16_to_cpu(in[0]) & 0xFC00) == 0xD800) &&
	    (sys_le16_to_cpu(in[1]) & 0xFC00) == 0xDC00) {
		codepoint = (sys_le16_to_cpu(in[0]) - 0xD800) << 10;
		codepoint += sys_le16_to_cpu(in[1]) - 0xDC00;
		codepoint += 0x10000;
	} else {
		codepoint = in[0];
	}

	if (codepoint <= 0x7F) {
		*out = (uint8_t)codepoint;
		out++;
	} else if (codepoint <= 0x7FF) {
		if (out_len < 2) {
			return -EIO;
		}
		*out = 0xC0 | (codepoint >> 6);
		out++;
		*out = 0x80 | (codepoint & 0x3F);
		out++;
	} else if (codepoint <= 0xFFFF) {
		if (out_len < 3) {
			return -EIO;
		}
		*out = 0xE0 | (codepoint >> 12);
		out++;
		*out = 0x80 | ((codepoint >> 6) & 0x3F);
		out++;
		*out = 0x80 | (codepoint & 0x3F);
		out++;
	} else {
		if (out_len < 4) {
			return -EIO;
		}
		*out = 0xF0 | (codepoint >> 18);
		out++;
		*out = 0x80 | ((codepoint >> 12) & 0x3F);
		out++;
		*out = 0x80 | ((codepoint >> 6) & 0x3F);
		out++;
		*out = 0x80 | (codepoint & 0x3F);
		out++;
	}
	if (next_char) {
		*next_char = codepoint > 65535 ? &in[2] : &in[1];
	}
	return 0;
}

/**
 * @brief Function for parsing the PDU structure for a SMS Deliver PDU
 *
 * @param buf Raw PDU buffer
 * @param pdu_data Output structure
 */
void deliver_pdu_parse(char *buf, struct deliver_pdu_data_s *pdu_data)
{
	pdu_data->smsc_len = hex_byte_to_data(buf);
	pdu_data->smsc_start = pdu_data->smsc_len ? buf + 2 : NULL;
	buf += 2 + pdu_data->smsc_len * 2;
	pdu_data->tp_flags = hex_byte_to_data(buf) & ~0x3;
	buf += 2;
	pdu_data->oa_len = hex_byte_to_data(buf);
	buf += 2;
	pdu_data->oa = buf;
	if ((hex_byte_to_data(buf) & 0x70) != 0x50) {
		buf += 2 + pdu_data->oa_len;
		buf += (pdu_data->oa_len % 2);
	} else {
		buf += 2 + (((pdu_data->oa_len + 1) * 7 / 8) * 2);
	}
	buf += 2; /* Skip TP-PID */
	pdu_data->alphabet = (hex_byte_to_data(buf) & 0xC) >> 2;
	buf += 2;
	pdu_data->scts = buf;
	buf += 14;
	pdu_data->udl = hex_byte_to_data(buf);
	buf += 2;
	pdu_data->ud = buf;
	pdu_data->udhl = (pdu_data->tp_flags & TP_FLAG_UDHI) ? hex_byte_to_data(buf) : 0;
}

int pdu_msg_extract(char *pdu_buffer, struct csms_data_s *csms_data)
{
	int ret;
	struct sms_in *sms;
	bool first_msg = false;
	char *out_buf;
	size_t out_buf_avail;
	struct csms_data_s csms_data_dummy;

	if (!csms_data) {
		csms_data = &csms_data_dummy;
	}

	csms_data->csms_ref = -1;
	csms_data->csms_tot = 1;
	csms_data->csms_idx = 1;

	/* No buffer specified. */
	if (!mdata.sms) {
		return 0;
	}

	sms = mdata.sms;
	out_buf = sms->msg;
	out_buf += strlen(sms->msg);

	out_buf_avail = sizeof(sms->msg) - (strlen(sms->msg) + 1);
	struct deliver_pdu_data_s pdu_data;

	deliver_pdu_parse(pdu_buffer, &pdu_data);

	uint8_t csms_idx = 0;

	if (pdu_data.udhl) {
		char *udh = pdu_data.ud + 2;
		uint8_t iei, iedl;

		while (udh < (pdu_data.ud + 2 + (pdu_data.udhl * 2))) {
			iei = hex_byte_to_data(udh);
			udh += 2;
			iedl = hex_byte_to_data(udh);
			udh += 2;
			if (iei != 0) {
				LOG_WRN("Unknown UDH Identifier %d", iei);
				udh += iedl * 2;
			} else {
				csms_data->csms_ref = hex_byte_to_data(udh);
				udh += 2;
				csms_data->csms_tot = hex_byte_to_data(udh);
				udh += 2;
				/* Todo: give some warning if we don't get all
				 * parts
				 */
				csms_idx = hex_byte_to_data(udh);
				csms_data->csms_idx = csms_idx;
				udh += 2;
			}
		}
	}

	if (!strlen(sms->msg)) {
		first_msg = true;
	} else if (sms->csms_idx + 1 != csms_idx) {
		return 0;
	}

	sms->csms_idx = csms_idx;

	if (!first_msg)
		goto decode_msg; /* We already have the phone number & timestamp */

	int real_len = (pdu_data.oa_len % 2) ? pdu_data.oa_len + 1 : pdu_data.oa_len;

	for (int i = 0; i < real_len; i += 2) {
		byteswp(&pdu_data.oa[i + 2], &pdu_data.oa[i + 3], 1);
	}

	memset(sms->phone, 0, sizeof(sms->phone));
	uint8_t type_of_number = (hex_byte_to_data(pdu_data.oa) >> 4) & 0x7;

	if (type_of_number == SMS_TON_INTERNATIONAL) {
		sms->phone[0] = '+';
		memcpy(&sms->phone[1], pdu_data.oa + 2, pdu_data.oa_len);
	} else {
		memcpy(sms->phone, pdu_data.oa + 2, pdu_data.oa_len);
	}

	for (int i = 0; i < 14; i += 2) {
		byteswp(&pdu_data.scts[i], &pdu_data.scts[i + 1], 1);
	}

	uint8_t tz = hex_byte_to_data(&pdu_data.scts[12]);

	snprintk(sms->time, sizeof(sms->time), "%.2s/%.2s/%.2s,%.2s:%.2s:%.2s%c%02x", pdu_data.scts,
		 &pdu_data.scts[2], &pdu_data.scts[4], &pdu_data.scts[6], &pdu_data.scts[8],
		 &pdu_data.scts[10], (tz & 0x80) ? '-' : '+', tz & 0x7F);

	memset(sms->msg, 0, sizeof(sms->msg));

decode_msg:
	if (pdu_data.alphabet == SMS_ALPHABET_GSM8) {
		if ((pdu_data.udl - pdu_data.udhl) > out_buf_avail) {
			if (first_msg) {
				LOG_WRN("Buffer too small: partial message "
					"copied");
			} else {
				LOG_WRN("Buffer too small: unable to "
					"concatenate part %d",
					csms_idx);
				return 0;
			}
		}
		hex_str_to_data(pdu_data.ud, out_buf,
				MIN((out_buf_avail), pdu_data.udl - pdu_data.udhl));
	} else if (pdu_data.alphabet == SMS_ALPHABET_UCS2) {
		uint16_t utf16_chr[3];
		char *ud = pdu_data.ud + (pdu_data.udhl ? 2 + 2 * pdu_data.udhl : 0);
		char *out_buf_ptr = out_buf;
		uint16_t *nchr;
		int avail = out_buf_avail;

		utf16_chr[2] = 0;
		while (strlen(ud) && avail) {
			memset(utf16_chr, 0, sizeof(utf16_chr));
			hex_str_to_data(ud, (uint8_t *)utf16_chr, 4);
			utf16_chr[0] = sys_be16_to_cpu(utf16_chr[0]);
			utf16_chr[1] = sys_be16_to_cpu(utf16_chr[1]);
			ret = utf16le_to_utf8(utf16_chr, out_buf_ptr, avail, &nchr);
			if (ret) {
				if (pdu_data.udhl && first_msg) {
					LOG_WRN("Buffer too small: partial "
						"message copied");
					break;
				}
				out_buf = '\0';
				LOG_WRN("Buffer too small: unable to "
					"concatenate part %d",
					csms_idx);
				return 0;
			}
			avail -= strlen(out_buf_ptr);
			out_buf_ptr += strlen(out_buf_ptr);
			if (nchr == &utf16_chr[1]) {
				ud += 4;
			} else {
				ud += 8;
			}
		}
	} else if (pdu_data.alphabet == SMS_ALPHABET_GSM7) {
		if (!pdu_data.udhl) {
			ret = gsm7_decode(pdu_data.ud, pdu_data.udl, out_buf, out_buf_avail, 0);
			if (ret) {
				LOG_WRN("Buffer too small: partial message "
					"copied");
			}
		} else {
			uint8_t skip = ((pdu_data.udhl + 1) * 8 + 6) / 7;

			ret = gsm7_decode(pdu_data.ud, pdu_data.udl, out_buf, out_buf_avail, skip);
			if (ret && !first_msg) {
				LOG_WRN("Buffer too small: unable to "
					"concatenate part %d",
					csms_idx);
				*out_buf = '\0';
				return 0;
			} else if (ret) {
				LOG_WRN("Buffer too small: partial message "
					"copied");
			}
		}
	}
	return 0;
}


/**
 * Check if given char sequence is crlf.
 *
 * @param c The char sequence.
 * @param len Total length of the fragment.
 * @return @c true if char sequence is crlf.
 *         Otherwise @c false is returned.
 */
static bool is_crlf(uint8_t *c, uint8_t len)
{
	/* crlf does not fit. */
	if (len < 2) {
		return false;
	}

	return c[0] == '\r' && c[1] == '\n';
}

/**
 * Find terminating crlf in a netbuffer.
 *
 * @param buf The netbuffer.
 * @param skip Bytes to skip before search.
 * @return Length of the returned fragment or 0 if not found.
 */
static size_t net_buf_find_crlf(struct net_buf *buf, size_t skip)
{
	size_t len = 0, pos = 0;
	struct net_buf *frag = buf;

	/* Skip to the start. */
	while (frag && skip >= frag->len) {
		skip -= frag->len;
		frag = frag->frags;
	}

	/* Need to wait for more data. */
	if (!frag) {
		return 0;
	}

	pos = skip;

	while (frag && !is_crlf(frag->data + pos, frag->len - pos)) {
		if (pos + 1 >= frag->len) {
			len += frag->len;
			frag = frag->frags;
			pos = 0U;
		} else {
			pos++;
		}
	}

	if (frag && is_crlf(frag->data + pos, frag->len - pos)) {
		len += pos;
		return len - skip;
	}

	return 0;
}

/**
 * @brief Handler for receiving unsolicited SMS message indication (+CMTI)
 */
MODEM_CMD_DEFINE(on_cmd_unsol_sms)
{
	LOG_DBG("SMS Received");
	k_sem_give(&mdata.sem_rcv_sms);

	return 0;
}

#if defined(CONFIG_MODEM_SMS_CALLBACK)

static struct sms_in async_sms_in;

/**
 * @brief Handler for receiving unsolicited SMS messages (+CMT)
 */
MODEM_CMD_DEFINE(on_cmd_unsol_cmt)
{
	int ret;
	char pdu_buffer[360];
	struct csms_data_s csms_data;
	size_t out_len, sms_len, param_len;

	/* Get the length of the "length" parameter.
	 * The last parameter will be stuck in the netbuffer.
	 * It is not the actual length of the trailing pdu so
	 * we have to search the next crlf.
	 */
	param_len = net_buf_find_crlf(data->rx_buf, 0);
	if (param_len == 0) {
		LOG_DBG("No <CR><LF>");
		return -EAGAIN;
	}

	/* Get actual trailing pdu len. +2 to skip crlf. */
	sms_len = net_buf_find_crlf(data->rx_buf, param_len + 2);
	if (sms_len == 0) {
		return -EAGAIN;
	}

	/* Skip to start of pdu. */
	data->rx_buf = net_buf_skip(data->rx_buf, param_len + 2);
	out_len = net_buf_linearize(pdu_buffer, sizeof(pdu_buffer), data->rx_buf, 0, sms_len);
	pdu_buffer[out_len] = '\0';

	LOG_DBG("SMS Received: %s", pdu_buffer);

	data->rx_buf = net_buf_skip(data->rx_buf, sms_len);

	memset(&async_sms_in, 0, sizeof(async_sms_in));

	mdata.sms = &async_sms_in;

	pdu_msg_extract(pdu_buffer, &csms_data);

	notify_sms_recv(mdata.net_iface->if_dev->dev, mdata.sms, csms_data.csms_ref,
				 csms_data.csms_idx, csms_data.csms_tot);

	return ret;
}
#endif /* CONFIG_MODEM_SMS_CALLBACK */
#endif /* CONFIG_MODEM_SMS */

/**
 * @brief Handler for unsolicited events ( SOCKETEV)
 */
MODEM_CMD_DEFINE(on_cmd_unsol_SEV)
{
	struct modem_socket *sock;
	int sock_id;
	int evt_id;

	LOG_DBG("got unsolicit socketev, evt: %s, sockfd: %s", argv[0], argv[1]);
	evt_id = ATOI(argv[0], 0, "event_id");
	sock_id = ATOI(argv[1], 0, "sock_id");
	/* TODO - handle optional connected fd */
	sock = modem_socket_from_id(&mdata.socket_config, sock_id);
	if (!sock) {
		return 0;
	}

	/* Data ready indication. */
	switch (evt_id) {
	case 0: /* in execution */
		break;
	case 1: /* Rx Rdy */
		LOG_DBG("Data Receive Indication for socket: %d", sock_id);

		modem_socket_packet_size_update(&mdata.socket_config, sock, 1);
		modem_socket_data_ready(&mdata.socket_config, sock);

		break;
		/* TODO: need to save the indication that the socket has been
		 * terminated remotely and treat properly in send and recv
		 * functions
		 */
	case 2: /* socket deactivated */
	case 3: /* socket terminated */
		LOG_WRN("Remote peer closed for socket: %d", sock_id);
		break;
	case 4: /* socket accepted */
		break;
	case 6: /* socket activation done */
		break;
	default:
		break;
	}

	return 0;
}

/**
 * @brief Handler for manufacturer
 */
MODEM_CMD_DEFINE(on_cmd_get_manufacturer)
{
	modem_cmd_handler_set_error(data, 0);

	size_t out_len = net_buf_linearize(
		mdata.mdm_manufacturer, sizeof(mdata.mdm_manufacturer) - 1, data->rx_buf, 0, len);
	mdata.mdm_manufacturer[out_len] = '\0';
	LOG_DBG("Manufacturer: %s", mdata.mdm_manufacturer);
	return 0;
}

/**
 * @brief Handler for model
 */
MODEM_CMD_DEFINE(on_cmd_get_model)
{
	size_t out_len = net_buf_linearize(mdata.mdm_model, sizeof(mdata.mdm_model) - 1,
					   data->rx_buf, 0, len);
	mdata.mdm_model[out_len] = '\0';

	LOG_DBG("Model: %s", mdata.mdm_model);
	return 0;
}

/**
 * @brief Handler for IMEI
 */
MODEM_CMD_DEFINE(on_cmd_get_imei)
{
	size_t out_len =
		net_buf_linearize(mdata.mdm_imei, sizeof(mdata.mdm_imei) - 1, data->rx_buf, 0, len);
	mdata.mdm_imei[out_len] = '\0';

	LOG_DBG("IMEI: %s", mdata.mdm_imei);
	return 0;
}

#if defined(CONFIG_MODEM_SIM_NUMBERS)
/**
 * @brief Handler for IMSI
 */
MODEM_CMD_DEFINE(on_cmd_get_imsi)
{
	size_t out_len =
		net_buf_linearize(mdata.mdm_imsi, sizeof(mdata.mdm_imsi) - 1, data->rx_buf, 0, len);
	mdata.mdm_imsi[out_len] = '\0';

	LOG_DBG("IMSI: %s", mdata.mdm_imsi);
	return 0;
}

/**
 * @brief Handler for ICCID
 */
MODEM_CMD_DEFINE(on_cmd_get_iccid)
{
	size_t out_len = net_buf_linearize(mdata.mdm_iccid, sizeof(mdata.mdm_iccid) - 1,
					   data->rx_buf, 0, len);
	mdata.mdm_iccid[out_len] = '\0';

	LOG_DBG("ICCID: %s", mdata.mdm_iccid);
	return 0;
}
#endif /* defined(CONFIG_MODEM_SIM_NUMBERS) */

/**
 * @brief Handler for BAND info
 */
MODEM_CMD_DEFINE(on_cmd_get_bands)
{
#define MAX_BANDS_STR_SZ 64
	char bandstr[MAX_BANDS_STR_SZ];
	size_t out_len = net_buf_linearize(bandstr, sizeof(bandstr) - 1, data->rx_buf, 0, len);

	bandstr[out_len] = '\0';

	LOG_DBG("BANDS - %s", bandstr);

	return 0;
}

/**
 * @brief Handler for GETACFG=modem_apps.Mode.AutoConnectMode
 */
static bool needto_set_autoconn_to_true;
MODEM_CMD_DEFINE(on_cmd_get_acfg)
{
#define MAX_AUTOCONN_STR_SZ 16
	char autoconnmode_str[MAX_AUTOCONN_STR_SZ];
	size_t out_len = net_buf_linearize(autoconnmode_str, sizeof(autoconnmode_str) - 1,
					   data->rx_buf, 0, len);
	autoconnmode_str[out_len] = '\0';

	if (strncmp(autoconnmode_str, "false", strlen("false")) == 0) {
		needto_set_autoconn_to_true = true;
	} else {
		needto_set_autoconn_to_true = false;
		LOG_DBG("Auto Conn Mode: %s", autoconnmode_str);
	}
	return 0;
}

/**
 * @brief Handler for socket count info
 */
static bool needto_set_sockcount;
MODEM_CMD_DEFINE(on_cmd_get_sockcount)
{
	char sockcount_str[16];
	size_t out_len =
		net_buf_linearize(sockcount_str, sizeof(sockcount_str) - 1, data->rx_buf, 0, len);
	sockcount_str[out_len] = '\0';

	if (strtol(sockcount_str, NULL, 10) != MDM_MAX_SOCKETS) {
		needto_set_sockcount = true;
	} else {
		needto_set_sockcount = false;
	}
	return 0;
}

#ifdef VERIFY_INIT_MODEM_STATE
/**
 * @brief Handler for CFUN
 */
MODEM_CMD_DEFINE(on_cmd_get_cfun)
{
#define MAX_CFUN_STR_SZ 16
	char cfun_resp_str[MAX_CFUN_STR_SZ];
	size_t out_len =
		net_buf_linearize(cfun_resp_str, sizeof(cfun_resp_str) - 1, data->rx_buf, 0, len);
	cfun_resp_str[out_len] = '\0';

	LOG_DBG("CFUN: %s", cfun_resp_str);
	return 0;
}

/**
 * @brief Handler for CEREG
 */
MODEM_CMD_DEFINE(on_cmd_get_cereg)
{
#define MAX_CEREG_STR_SZ 16
	char cereg_resp_str[MAX_CEREG_STR_SZ];
	size_t out_len =
		net_buf_linearize(cereg_resp_str, sizeof(cereg_resp_str) - 1, data->rx_buf, 0, len);
	cereg_resp_str[out_len] = '\0';

	LOG_DBG("CEREG: %s", cereg_resp_str);
	return 0;
}
#endif

/**
 * @brief Handler for getting PSM values
 */
MODEM_CMD_DEFINE(on_cmd_get_psm)
{
	size_t out_len =
		net_buf_linearize(mdata.mdm_psm, sizeof(mdata.mdm_psm) - 1, data->rx_buf, 0, len);
	mdata.mdm_psm[out_len] = '\0';

	LOG_DBG("PSM: %s", mdata.mdm_psm);
	return 0;
}

/**
 * @brief Handler for eDRX
 */
MODEM_CMD_DEFINE(on_cmd_get_edrx)
{
	size_t out_len =
		net_buf_linearize(mdata.mdm_edrx, sizeof(mdata.mdm_edrx) - 1, data->rx_buf, 0, len);
	mdata.mdm_edrx[out_len] = '\0';

	LOG_DBG("EDRX: %s", mdata.mdm_edrx);
	return 0;
}

/**
 * @brief Handler for LTECMD PTW
 */
MODEM_CMD_DEFINE(on_cmd_lte_ptw)
{
	strcpy(mdata.mdm_edrx, argv[1]);
	return 0;
}

static char *get_4_octet(char *buf)
{
	char *ptr = buf;
	uint16_t octCnt = 0;

	for (; octCnt < 4; octCnt++) {
		if (ptr) {
			ptr = strchr(ptr, '.');
			++ptr;
		}
	}
	return ptr - 1;
}

/**
 * @brief Set auto-connection mode on
 */
static int set_autoconn_on(void)
{
	static const char at_cmd[] = "AT\%SETACFG=modem_apps.Mode.AutoConnectMode,\"true\"";

	LOG_WRN("autoconnect is set to false, will now set to true");
	int ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, at_cmd,
				 &mdata.sem_response, MDM_CMD_RSP_TIME);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
	}
	return ret;
}

/**
 * @brief Set socket count to match config
 */
static int set_socket_count(void)
{
	char at_cmd[sizeof("AT%SETACFG=\"service.sockserv.maxsock\",\"X\"")];

	snprintk(at_cmd, sizeof(at_cmd), "AT%%SETACFG=\"service.sockserv.maxsock\",\"%d\"",
		 MDM_MAX_SOCKETS);

	int ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, at_cmd,
				 &mdata.sem_response, MDM_CMD_RSP_TIME);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
	}
	return ret;
}

/**
 * @brief Set BANDs to 2,4, 12 (T-Mobile)
 */
static int set_bands(void)
{
	static const char at_cmd[] = "AT\%SETCFG=\"BAND\",\"2\",\"4\",\"12\"";

	LOG_INF("Setting bands to 2, 4, 12");
	modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, at_cmd, &mdata.sem_response,
		       MDM_CMD_RSP_TIME);

	/* Setting bands is disabled in golden images,
	 * but still needed for sample images, so
	 * ignore error returned from modem_cmd_send
	 */

	return 0;
}

/**
 * @brief Set boot delay to 0
 */
static int set_boot_delay(void)
{
	static const char at_cmd[] = "AT\%SETBDELAY=0";

	LOG_INF("Setting boot delay to 0");
	int ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, at_cmd,
				 &mdata.sem_response, MDM_CMD_RSP_TIME);

	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
	}
	return ret;
}

enum handshake_state {
	init_suspend,
	init_resume,
	isr_init_resume,
	suspend_wait_modem_to_host,
	suspend_wait_rx,
	resume_wait_modem_to_host,
	resume_wait_rx,
	isr_resume_wait_rx,
};

static enum handshake_state hifc_handshake_state;

K_SEM_DEFINE(hifc_handshake_sem, 0, 1);

static void hifc_handshake(struct k_timer *timer_id);

K_TIMER_DEFINE(hifc_handshake_tmr, hifc_handshake, NULL);

static int tmr_counter;
static void hifc_handshake(struct k_timer *timer_id)
{
	char hifc_mode = *(char *)timer_id->user_data;

	switch (hifc_handshake_state) {
	case init_suspend:
		gpio_pin_set_dt(&wake_mdm_gpio, 0);
		tmr_counter = 0;
		hifc_handshake_state = suspend_wait_modem_to_host;
		break;
	case suspend_wait_modem_to_host:
		if (gpio_pin_get_dt(&wake_host_gpio) == 0) {
			if (hifc_mode == 'A') {
				tmr_counter = 0;
				hifc_handshake_state = suspend_wait_rx;
				gpio_pin_configure_dt(&mdm_tx_gpio,
						GPIO_INPUT | GPIO_PULL_DOWN);
			} else {
				LOG_INF("Suspend successful");
				k_sem_give(&hifc_handshake_sem);
				k_timer_stop(timer_id);
			}
		} else if (tmr_counter < 20) {
			tmr_counter++;
		} else {
			k_sem_give(&hifc_handshake_sem);
			k_timer_stop(timer_id);
			LOG_ERR("wake_host_gpio did not go low");
		}
		break;
	case suspend_wait_rx:
		if (gpio_pin_get_dt(&wake_host_gpio) == 0) {
			LOG_INF("Suspend successful");
			k_sem_give(&hifc_handshake_sem);
			k_timer_stop(timer_id);
		} else if (tmr_counter < 20) {
			tmr_counter++;
		} else {
			k_sem_give(&hifc_handshake_sem);
			k_timer_stop(timer_id);
			LOG_ERR("mdm_rx_gpio did not go low");
		}
		break;
	case init_resume:
		gpio_pin_set_dt(&wake_mdm_gpio, 1);
	case isr_init_resume:
		if (hifc_mode == 'B') {
			hifc_handshake_state = resume_wait_modem_to_host;
		} else {
			hifc_handshake_state = isr_resume_wait_rx;
			gpio_pin_configure_dt(&mdm_tx_gpio,
					GPIO_OUTPUT_LOW | GPIO_PULL_DOWN);
			gpio_pin_set_dt(&mdm_tx_gpio, 1);
		}
		tmr_counter = 0;
		break;
	case resume_wait_modem_to_host:
		if (gpio_pin_get_dt(&wake_host_gpio) == 1) {
			LOG_INF("Resume successful");
			k_sem_give(&hifc_handshake_sem);
			k_timer_stop(timer_id);
		} else if (tmr_counter < 20) {
			tmr_counter++;
		} else {
			k_sem_give(&hifc_handshake_sem);
			k_timer_stop(timer_id);
			LOG_ERR("wake_host_gpio did not go high");
		}
		break;
	case isr_resume_wait_rx:
		if (gpio_pin_get_dt(&mdm_rx_gpio) == 1) {
			gpio_pin_set_dt(&wake_mdm_gpio, 1);
			LOG_INF("Resume successful");
			k_sem_give(&hifc_handshake_sem);
			k_timer_stop(timer_id);
		} else if (tmr_counter < 20) {
			tmr_counter++;
		} else {
			k_sem_give(&hifc_handshake_sem);
			k_timer_stop(timer_id);
			LOG_ERR("mdm_rx_gpio did not go high (ISR)");
		}
	case resume_wait_rx:
		if (gpio_pin_get_dt(&mdm_rx_gpio) == 1) {
			gpio_pin_configure_dt(&mdm_tx_gpio,
					GPIO_OUTPUT_LOW | GPIO_PULL_DOWN);
			gpio_pin_set_dt(&mdm_tx_gpio, 1);
			tmr_counter = 0;
			hifc_handshake_state = resume_wait_modem_to_host;
		} else if (tmr_counter < 20) {
			tmr_counter++;
		} else {
			k_sem_give(&hifc_handshake_sem);
			k_timer_stop(timer_id);
			LOG_ERR("mdm_rx_gpio did not go high");
		}
		break;
	}
}


/**
 * @brief Put the modem into resume (low-power) state
 *
 * @param hifc_mode The HIFC mode to use (either 'A' or 'B')
 */
static bool enter_resume_state(char hifc_mode)
{
	gpio_pin_interrupt_configure_dt(&wake_host_gpio, GPIO_INT_DISABLE);
	k_sem_reset(&hifc_handshake_sem);
	hifc_handshake_state = init_resume;
	hifc_handshake_tmr.user_data = &hifc_mode;
	k_timer_start(&hifc_handshake_tmr, K_NO_WAIT, K_MSEC(100));
	k_sem_take(&hifc_handshake_sem, K_FOREVER);
	gpio_pin_interrupt_configure_dt(&wake_host_gpio, GPIO_INT_EDGE_RISING);
	return tmr_counter < 20;
}

/**
 * @brief Put the modem into suspend (low-power) state
 *
 * @param hifc_mode The HIFC mode to use (either 'A' or 'B')
 */
static bool enter_suspend_state(char hifc_mode)
{
	k_sem_reset(&hifc_handshake_sem);
	hifc_handshake_state = init_suspend;
	hifc_handshake_tmr.user_data = &hifc_mode;
	k_timer_start(&hifc_handshake_tmr, K_NO_WAIT, K_MSEC(100));
	k_sem_take(&hifc_handshake_sem, K_FOREVER);
	return tmr_counter < 20;
}

/**
 * @brief Set CFUN to 1 (on) or 0 (off)
 *
 * @param on Value of 1 sets CFUN=1 (turns modem on)
 *           Value of 0 sets CFUN=0 (turns modem off)
 */
static int set_cfun(int on)
{
	char at_cmd[32];

	if (s_hifc_mode == 'A') {
		if (on && gpio_pin_get_dt(&mdm_rx_gpio) && gpio_pin_get_dt(&wake_host_gpio)) {
			LOG_WRN("Modem is already awake");
			return 0;
		}

		if ((!on && gpio_pin_get_dt(&mdm_rx_gpio) == 0) &&
		    (gpio_pin_get_dt(&wake_host_gpio) == 0)) {
			LOG_WRN("Modem is already asleep");
			return 0;
		}
	} else if (s_hifc_mode == 'B') {
		if (on && gpio_pin_get_dt(&wake_host_gpio)) {
			LOG_WRN("Modem is already awake");
			return 0;
		}

		if (!on && (gpio_pin_get_dt(&wake_host_gpio) == 0)) {
			LOG_WRN("Modem is already asleep");
			return 0;
		}
	}

	if (on) {
		enter_resume_state(s_hifc_mode);
	}

	snprintk(at_cmd, sizeof(at_cmd), "AT+CFUN=%d", on);
	LOG_DBG("%s", at_cmd);
	int ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, at_cmd,
				 &mdata.sem_response, MDM_CMD_LONG_RSP_TIME);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
	}

	if (!on) {
		enter_suspend_state(s_hifc_mode);
	}
	return ret;
}

/**
 * @brief Set the PSM timer values that is passed in thru params
 *
 */
static int set_psm_timer(struct set_cpsms_params *params)
{
	char psm[100];
	char t3412[PSM_TIME_LEN];
	char t3324[PSM_TIME_LEN];
	int ret;

	strcpy(t3412, (const char *)byte_to_binary_str(params->t3412));
	strcpy(t3324, (const char *)byte_to_binary_str(params->t3324));

	snprintf(psm, sizeof(psm), "AT+CPSMS=%d,,,\"%s\",\"%s\"", params->mode, t3412,
			t3324);

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, psm, &mdata.sem_response,
			     K_SECONDS(6));
	if (ret < 0) {
		LOG_ERR("%s ret:%d", psm, ret);
	}

	if (params->mode && !ret) {
		enter_suspend_state('A');
	}

	return ret;
}

/**
 * @brief Set the edrx timer values that is passed in thru Parms
 *
 */
/* This function assumes the edrx_value or time value passed in is a coded byte
 */
static int set_edrx_timer(struct set_cedrxs_params *Parms)
{
	int ret;
	char at_cmd[100] = {0};
	char *binary_str = byte_to_binary_str(Parms->time_mask);

	binary_str = binary_str + 4; /* get last 4 bits in ascii */
	snprintf(at_cmd, sizeof(at_cmd), "AT+CEDRXS=%d,%d,\"%s\"", (int)Parms->mode,
		 (int)Parms->act_type, binary_str);
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, at_cmd, &mdata.sem_response,
			     K_SECONDS(6));
	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
	}

	if (Parms->mode && !ret) {
		enter_suspend_state('A');
	}
	return ret;
}

/**
 * @brief Set the edrx paging time window value
 */
static int set_edrx_ptw(int *ptw)
{
	int ret;
	char at_cmd[sizeof("AT%CEDRXS=XX")] = {0};

	snprintf(at_cmd, sizeof(at_cmd), "AT%%CEDRXS=%d", *ptw);
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, at_cmd, &mdata.sem_response,
			     K_SECONDS(6));

	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
	}

	return ret;
}

/**
 * @brief Get the edrx paging time window value
 */
static int get_edrx_ptw(int *ptw)
{
	int ret;
	static const char *at_cmd = "AT%LTECMD=2,\"PTW\"";
	struct modem_cmd data_cmd[] = {
		MODEM_CMD("%LTECMD:", on_cmd_lte_ptw, 2U, ","),
	};

	memset(mdata.mdm_edrx, 0, sizeof(mdata.mdm_edrx));

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, ARRAY_SIZE(data_cmd), at_cmd,
			     &mdata.sem_response, MDM_CMD_RSP_TIME);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
		ret = -1;
	}

	*ptw = strtol(mdata.mdm_edrx, NULL, 10);

	return 0;
}

/**
 * @brief Use the PDNSET command to set APN and IP type
 */
static int set_pdn_params(void)
{
	int ret = 0;

	/* Use CONFIG_MODEM_MURATA_1SC_APN if defined and not blank */
#if defined(CONFIG_MODEM_MURATA_1SC_APN)
	if (strlen(CONFIG_MODEM_MURATA_1SC_APN)) {
		char at_cmd[128];
#if defined(CONFIG_NET_IPV6)
		LOG_INF("Setting APN to %s and IPV4V6", CONFIG_MODEM_MURATA_1SC_APN);
		snprintk(at_cmd, sizeof(at_cmd), "AT%%PDNSET=1,\"%s\",\"IPV4V6\",\"\",\"\"",
			 CONFIG_MODEM_MURATA_1SC_APN);
#else
		LOG_INF("Setting APN to %s and IPV4", CONFIG_MODEM_MURATA_1SC_APN);
		snprintk(at_cmd, sizeof(at_cmd), "AT%%PDNSET=1,\"%s\",\"IP\",\"\",\"\"",
			 CONFIG_MODEM_MURATA_1SC_APN);
#endif
		LOG_DBG("%s", at_cmd);
		int ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, at_cmd,
					 &mdata.sem_response, MDM_CMD_RSP_TIME);
		if (ret < 0) {
			LOG_ERR("%s ret:%d", at_cmd, ret);
		}
	} else {
		LOG_DBG("No APN configuration found");
	}
#else
	LOG_DBG("No CONFIG_MODEM_MURATA_1SC_APN setting found");
#endif
	return ret;
}

/**
 * @brief Handler for GETACFG=pm.conf.sleep_mode
 */
static bool needto_set_sleep_mode;
MODEM_CMD_DEFINE(on_cmd_get_sleep_mode)
{
#define MAX_SLEEP_MODE_STR_SZ 16
	char sleep_mode_str[MAX_SLEEP_MODE_STR_SZ];
	size_t out_len =
		net_buf_linearize(sleep_mode_str, sizeof(sleep_mode_str) - 1, data->rx_buf, 0, len);
	sleep_mode_str[out_len] = '\0';

	if (strncmp(sleep_mode_str, "enable", strlen("enable")) == 0) {
		needto_set_sleep_mode = s_sleep_mode ? false : true;
	} else if (strncmp(sleep_mode_str, "disable", strlen("disable")) == 0) {
		needto_set_sleep_mode = s_sleep_mode ? true : false;
	} else {
		LOG_WRN("Unknown sleep mode: %s", sleep_mode_str);
		needto_set_sleep_mode = true;
	}

	if (needto_set_sleep_mode == false) {
		LOG_INF("Sleep mode: %s", sleep_mode_str);
	} else {
		LOG_WRN("Sleep mode: %s", sleep_mode_str);
	}
	return 0;
}

/**
 * @brief Enable or disable sleep mode
 */
static int set_sleep_mode(bool enable)
{
	char at_cmd[256];

	snprintk(at_cmd, sizeof(at_cmd), "AT%%SETACFG=pm.conf.sleep_mode,%s",
		 enable ? "enable" : "disable");
	LOG_INF("%s sleep mode", enable ? "Enabling" : "Disabling");
	int ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, at_cmd,
				 &mdata.sem_response, MDM_CMD_RSP_TIME);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
	}
	return ret;
}

/**
 * @brief Handler for GETACFG=pm.conf.max_allowed_pm_mode
 */
static bool needto_set_max_pm_mode;
MODEM_CMD_DEFINE(on_cmd_get_max_pm_mode)
{
#define MAX_PM_MODE_STR_SZ 16
	char pm_mode_str[MAX_PM_MODE_STR_SZ];
	size_t out_len =
		net_buf_linearize(pm_mode_str, sizeof(pm_mode_str) - 1, data->rx_buf, 0, len);
	pm_mode_str[out_len] = '\0';

	if (strncmp(pm_mode_str, s_max_pm_mode, strlen(s_max_pm_mode))) {
		needto_set_max_pm_mode = true;
		LOG_WRN("Max allowed PM mode: %s", pm_mode_str);
	} else {
		needto_set_max_pm_mode = false;
		LOG_INF("Max allowed PM mode: %s", pm_mode_str);
	}
	return 0;
}

/**
 * @brief Set max allowed (low) power mode
 *
 * @param pm_mode is string representing lowest power mode
 *
 * Supported modes are dh0, dh1, dh2, ds, and ls
 */
static int set_max_pm_mode(const char *pm_mode)
{
	char at_cmd[sizeof("AT%SETACFG=pm.conf.max_allowed_pm_mode,XXX")];

	snprintk(at_cmd, sizeof(at_cmd), "AT%%SETACFG=pm.conf.max_allowed_pm_mode,%s", pm_mode);
	LOG_INF("Setting max allowed PM mode to %s", pm_mode);
	int ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, at_cmd,
				 &mdata.sem_response, MDM_CMD_RSP_TIME);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
	}
	return ret;
}

/**
 * @brief Handler for GETACFG=pm.hifc.mode
 */
static bool needto_set_hifc_mode;
MODEM_CMD_DEFINE(on_cmd_get_hifc_mode)
{
#define MAX_HIFC_MODE_STR_SZ 16
	char hifc_mode_str[MAX_HIFC_MODE_STR_SZ];
	size_t out_len =
		net_buf_linearize(hifc_mode_str, sizeof(hifc_mode_str) - 1, data->rx_buf, 0, len);
	hifc_mode_str[out_len] = '\0';

	if (hifc_mode_str[0] != s_hifc_mode) {
		needto_set_hifc_mode = true;
		LOG_WRN("HIFC mode: %s", hifc_mode_str);
	} else {
		needto_set_hifc_mode = false;
		LOG_INF("HIFC mode: %s", hifc_mode_str);
	}
	return 0;
}

/**
 * @brief Handler for GETACFG=modem_apps.Mode.AtCmdSetPersistence
 */
static bool needto_enable_at_persist;
MODEM_CMD_DEFINE(on_cmd_get_at_persist)
{
	char true_false[6];
	size_t out_len =
		net_buf_linearize(true_false, sizeof(true_false) - 1, data->rx_buf, 0, len);

	true_false[out_len] = '\0';
	needto_enable_at_persist = strncmp(true_false, "true", 4);

	return 0;
}

/**
 * @brief set pm.hifc.mode - sets HIFC mode
 * Supported modes are 'A' and 'C'
 */
static int set_hifc_mode(const char hifc_mode)
{
	char at_cmd[256];

	snprintk(at_cmd, sizeof(at_cmd), "AT%%SETACFG=pm.hifc.mode,%c", hifc_mode);
	int ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, at_cmd,
				 &mdata.sem_response, MDM_CMD_RSP_TIME);

	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
	} else {
		LOG_DBG("Set HIFC mode to %c", hifc_mode);
	}
	return ret;
}

static bool needto_set_dns_servers;
#define DNS_SET_FORMAT_STR "AT%%SETACFG=APNTable.Class%d.IPv%dDnsIP_%d,%s"
/**
 * @brief Sets DNS servers on the modem
 */
static int set_dns_servers(void)
{
	int ret;
	bool addr4_primary, addr4_secondary, addr6_primary, addr6_secondary;
	struct sockaddr addr;
	char at_cmd[sizeof("AT%SETACFG=APNTable.ClassX.IPvXDnsIP_X,\"\"") + NET_IPV6_ADDR_LEN];

	char *pri_addr4 = CONFIG_MODEM_MURATA_IPV4_DNS_PRIMARY;
	char *pri_addr6 = CONFIG_MODEM_MURATA_IPV6_DNS_PRIMARY;
	char *sec_addr4 = CONFIG_MODEM_MURATA_IPV4_DNS_SECONDARY;
	char *sec_addr6 = CONFIG_MODEM_MURATA_IPV6_DNS_SECONDARY;

	addr4_primary = net_ipaddr_parse(pri_addr4, strlen(pri_addr4), &addr);
	addr4_primary = addr4_primary && addr.sa_family == AF_INET;

	addr4_secondary = net_ipaddr_parse(sec_addr4, strlen(sec_addr4), &addr);
	addr4_secondary = addr4_secondary && addr.sa_family == AF_INET;

	addr6_primary = net_ipaddr_parse(pri_addr6, strlen(pri_addr6), &addr);
	addr6_primary = addr6_primary && addr.sa_family == AF_INET6;

	addr6_secondary = net_ipaddr_parse(sec_addr6, strlen(sec_addr6), &addr);
	addr6_secondary = addr6_secondary && addr.sa_family == AF_INET6;

	{
		char pri_addr_4_str[NET_IPV4_ADDR_LEN + 2];
		char sec_addr_4_str[NET_IPV4_ADDR_LEN + 2];

		strcpy(pri_addr_4_str, "null");
		strcpy(sec_addr_4_str, "null");

		if (addr4_secondary && !addr4_primary) {
			snprintk(pri_addr_4_str, sizeof(pri_addr_4_str), "\"%s\"", sec_addr4);
		} else if (addr4_primary) {
			snprintk(pri_addr_4_str, sizeof(pri_addr_4_str), "\"%s\"", pri_addr4);
			if (addr4_secondary) {
				snprintk(sec_addr_4_str, sizeof(sec_addr_4_str), "\"%s\"",
					 sec_addr4);
			}
		}

		snprintk(at_cmd, sizeof(at_cmd), DNS_SET_FORMAT_STR, 1, 4, 1, pri_addr_4_str);
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, at_cmd,
				     &mdata.sem_response, MDM_CMD_RSP_TIME);
		if (ret) {
			return ret;
		}

		snprintk(at_cmd, sizeof(at_cmd), DNS_SET_FORMAT_STR, 2, 4, 1, pri_addr_4_str);
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, at_cmd,
				     &mdata.sem_response, MDM_CMD_RSP_TIME);
		if (ret) {
			return ret;
		}

		snprintk(at_cmd, sizeof(at_cmd), DNS_SET_FORMAT_STR, 1, 4, 2, sec_addr_4_str);
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, at_cmd,
				     &mdata.sem_response, MDM_CMD_RSP_TIME);
		if (ret) {
			return ret;
		}

		snprintk(at_cmd, sizeof(at_cmd), DNS_SET_FORMAT_STR, 2, 4, 2, sec_addr_4_str);
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, at_cmd,
				     &mdata.sem_response, MDM_CMD_RSP_TIME);
		if (ret) {
			return ret;
		}
	}

	{
		char pri_addr_6_str[NET_IPV6_ADDR_LEN + 2];
		char sec_addr_6_str[NET_IPV6_ADDR_LEN + 2];

		strcpy(pri_addr_6_str, "null");
		strcpy(sec_addr_6_str, "null");

		if (addr6_secondary && !addr6_primary) {
			snprintk(pri_addr_6_str, sizeof(pri_addr_6_str), "\"%s\"", sec_addr6);
		} else if (addr6_primary) {
			snprintk(pri_addr_6_str, sizeof(pri_addr_6_str), "\"%s\"", pri_addr6);
			if (addr6_secondary) {
				snprintk(sec_addr_6_str, sizeof(sec_addr_6_str), "\"%s\"",
					 sec_addr6);
			}
		}

		snprintk(at_cmd, sizeof(at_cmd), DNS_SET_FORMAT_STR, 1, 6, 1, pri_addr_6_str);
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, at_cmd,
				     &mdata.sem_response, MDM_CMD_RSP_TIME);
		if (ret) {
			return ret;
		}

		snprintk(at_cmd, sizeof(at_cmd), DNS_SET_FORMAT_STR, 2, 6, 1, pri_addr_6_str);
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, at_cmd,
				     &mdata.sem_response, MDM_CMD_RSP_TIME);
		if (ret) {
			return ret;
		}

		snprintk(at_cmd, sizeof(at_cmd), DNS_SET_FORMAT_STR, 1, 6, 2, sec_addr_6_str);
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, at_cmd,
				     &mdata.sem_response, MDM_CMD_RSP_TIME);
		if (ret) {
			return ret;
		}

		snprintk(at_cmd, sizeof(at_cmd), DNS_SET_FORMAT_STR, 2, 6, 2, sec_addr_6_str);
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, at_cmd,
				     &mdata.sem_response, MDM_CMD_RSP_TIME);
		if (ret) {
			return ret;
		}
	}
	needto_set_dns_servers = false;
	return 0;
}

/**
 * @brief set Mode.AtCmdSetPersistence - sets AtCmdSetPersistence mode
 */
static int set_at_persist_mode(void)
{
	static const char *at_cmd = "AT%SETACFG=modem_apps.Mode.AtCmdSetPersistence,true";

	int ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, at_cmd,
				 &mdata.sem_response, MDM_CMD_RSP_TIME);

	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
	} else {
		LOG_DBG("Set AtCmdSetPersistence mode to true");
	}
	return ret;
}

/**
 * @brief Parse the response of AT%PDNRDP=1 to get IP, mask, and gateway
 *
 * @param p1 is the ip addr pointer
 * @param p2 is the mask pointer
 * @param p3 is the gateway pointer
 */
static int parse_ipgwmask(const char *buf, char *p1, char *p2, char *p3)
{
	char *pstr, *pend = NULL;
	size_t len;
	int ret = -1;

	pstr = strchr(buf, ','); /* session id */
	if (pstr)
		pstr = strchr(pstr + 1, ','); /* bearer id */
	if (pstr)
		pstr = strchr(pstr + 1, ','); /* apn */
	if (pstr) {
		pend = get_4_octet(pstr + 1);
	}
	if (pend) {
		*pend = 0;
		len = pend - pstr - 1;
		len = MIN(len, MDM_IP_LENGTH - 1);
		memset(p1, 0, MDM_IP_LENGTH);
		strncpy(p1, pstr + 1, len);
		pstr = pend + 1;
		pend = strchr(pstr, ',');
		if (pend) {
			*pend = 0;
			len = pend - pstr;
			len = MIN(len, MDM_GW_LENGTH - 1);
			memset(p2, 0, MDM_GW_LENGTH);
			strncpy(p2, pstr, len);
			pstr = pend + 1;
			pend = strchr(pstr, ',');
			if (pend) {
				*pend = 0;
				len = pend - pstr;
				len = MIN(len, MDM_MASK_LENGTH - 1);
				memset(p3, 0, MDM_MASK_LENGTH);
				strncpy(p3, pstr, len);
				LOG_DBG("IP: %s, MASK: %s, GW: %s\n", p1, p2, p3);
				ret = 0;
			}
		}
	}
	return ret;
}

#define PDN_QUERY_RESPONSE_LEN 256
static bool first_pdn_rcved;

/**
 * @brief Handler for PDNRDP
 *
 * Sample output:
 * AT at%pdnrdp=1
 * %PDNRDP: 1,5,"iot.catm.mnc882.mcc311.gprs",25.181.129.55.255.255.255.240,
 * 25.181.129.49,10.177.0.34,10.177.0.210,,,,,,,,,
 */
MODEM_CMD_DEFINE(on_cmd_ipgwmask)
{
	char buf[PDN_QUERY_RESPONSE_LEN] = {0};
	int ret = 0;
	size_t read_cnt;

	LOG_DBG("Got PDNRDP, len = %d", len);
	if (!first_pdn_rcved) {
		first_pdn_rcved = true;
		read_cnt = net_buf_linearize(buf, PDN_QUERY_RESPONSE_LEN - 1, data->rx_buf, 0, len);
		if (strstr(buf, "\r\n")) {
			LOG_WRN("Not enough octets!!");
			ret = -EAGAIN;
			first_pdn_rcved = false;
		} else {
			buf[read_cnt] = 0;
			ret = parse_ipgwmask(buf, mdata.mdm_ip, mdata.mdm_nmask, mdata.mdm_gw);

			LOG_DBG("IP: %s, GW: %s, NMASK: %s", mdata.mdm_ip, mdata.mdm_gw,
				mdata.mdm_nmask);
		}
	}
	return ret;
}

/**
 * @brief Use AT%PDNRDP=1 to get IP settings from modem
 */
static int get_ipv4_config(void)
{
	static const char at_cmd[] = "AT\%PDNRDP=1";
	struct modem_cmd data_cmd[] = {MODEM_CMD("ERROR", on_cmd_error, 0U, ""),
				       MODEM_CMD("%PDNRDP", on_cmd_ipgwmask, 0U, ":")};

	int ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, ARRAY_SIZE(data_cmd),
				 at_cmd, &mdata.sem_response, MDM_CMD_RSP_TIME);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
	}
	return ret;
}

/**
 * @brief Return the first string between double quotes
 */
static int get_str_in_quotes(const char *buf, char *pdest, size_t dest_size)
{
	char delim = '"';
	char *pstart, *pend;
	int strlen = 0;

	pstart = strchr(buf, delim);
	if (pstart) {
		++pstart;
		pend = strchr(pstart, delim);
		if (pend) {
			strlen = pend - pstart;
			strlen = MIN(strlen, dest_size - 1);
			strncpy(pdest, pstart, strlen);
			pdest[strlen] = '\0';
		}
	}
	return strlen;
}

struct mdm_dns_resp_t {
	struct sockaddr_in ipv4;
	struct sockaddr_in6 ipv6;
};

struct mdm_dns_resp_t mdm_dns_ip;

/**
 * @brief Parse the DNS response
 */
static int parse_dnsresp(char *buf, struct mdm_dns_resp_t *dns_resp)
{
#define IP_STR_LEN 100
	int len; /* len of the string in "" */
	char ip[IP_STR_LEN];

	if ('0' == buf[0]) {
		len = get_str_in_quotes(buf, ip, IP_STR_LEN);
		ip[len] = 0;
		dns_resp->ipv4.sin_family = AF_INET;
		zsock_inet_pton(AF_INET, ip, &dns_resp->ipv4.sin_addr.s4_addr);
		LOG_DBG("dns-ipv4: %s\n", ip);
	}
#if defined(CONFIG_NET_IPV6)
	else if ('1' == buf[0]) {
		len = get_str_in_quotes(buf, ip, IP_STR_LEN);
		ip[len] = 0;
		dns_resp->ipv6.sin6_family = AF_INET6;
		zsock_inet_pton(AF_INET6, ip, &dns_resp->ipv6.sin6_addr.s6_addr);
		LOG_DBG("dns-ipv6: %s\n", ip);
	}
#endif
	else {
		return -1;
	}
	return 0;
}

/**
 * @brief Handler for DNSRSLV
 */
MODEM_CMD_DEFINE(on_cmd_dnsrslv)
{
#define DNS_QUERY_RESPONSE_LEN 128
	char buf[DNS_QUERY_RESPONSE_LEN] = {0};
	int ret = 0;
	size_t read_cnt;

	read_cnt = net_buf_linearize(buf, DNS_QUERY_RESPONSE_LEN - 1, data->rx_buf, 0, len);
	if (strstr(buf, "\r\n")) {
		LOG_WRN("NOT enough octets!!");
		ret = -EAGAIN;
		first_pdn_rcved = false;
	} else {
		buf[read_cnt] = 0;
		parse_dnsresp(buf, &mdm_dns_ip);

		LOG_DBG("Got DNSRSLV, len = %d, read_cnt = %d", len, read_cnt);
	}
	return ret;
}

/**
 * @brief get ipv4/6 DNS info from modem
 * @param: domain name in string
 */
static int get_dns_ip(const char *dn)
{
	char at_cmd[128];
	int ret;

	struct modem_cmd data_cmd[] = {
		MODEM_CMD("%DNSRSLV:", on_cmd_dnsrslv, 0U, ""),
	};
	memset(&mdm_dns_ip, 0, sizeof(mdm_dns_ip));
	snprintk(at_cmd, sizeof(at_cmd), "AT%%DNSRSLV=0,\"%s\"", dn);
	LOG_DBG("%s", at_cmd);
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, 1, at_cmd,
			     &mdata.sem_response, MDM_CMD_LONG_RSP_TIME);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
	}
	return ret;
}

/**
 * @brief Handler to read data from socket
 * @param argv[0] sock_id
 * @param argv[1] data_len length of data
 * @param argv[2] more indicates whether more data is available
 * @param argv[3] data (but not necessarily contiguous)
 *
 * %SOCKETDATA:<socket_id>[0],<length>[1],<moreData>[2],
 * "<data>", <src_ip>, <src_port>
 *
 * prototype for this function:
 * static int name_(struct modem_cmd_handler_data *data, uint16_t len, \
 * uint8_t **argv, uint16_t argc)
 */
MODEM_CMD_DEFINE(on_cmd_sock_readdata)
{
	/* We need at least 3 parameters. Less than 3 causes an error like this:
	 * <err> modem_cmd_handler: process cmd [%SOCKETDATA:] (len:16, ret:-22)
	 * Returning 0 here prevents the error
	 */
	if (argc < 3) {
		return 0;
	}

	int more = (int)strtol(argv[2], NULL, 10);

	int ret = on_cmd_sockread_common(mdata.sock_fd, data, ATOI(argv[1], 0, "length"), len);

	LOG_DBG("on_cmd_sockread_common returned %d", ret);

	if (more) {
		struct modem_socket *sock =
			modem_socket_from_id(&mdata.socket_config, mdata.sock_fd);
		modem_socket_packet_size_update(&mdata.socket_config, sock, 1);
		modem_socket_data_ready(&mdata.socket_config, sock);
	}
	return ret;
}

static const struct modem_cmd response_cmds[] = {
	MODEM_CMD("OK", on_cmd_ok, 0U, ""),
	MODEM_CMD("ERROR", on_cmd_error, 0U, ""),
};

static const struct modem_cmd unsol_cmds[] = {
	MODEM_CMD("%SOCKETEV:", on_cmd_unsol_SEV, 2U, ","),
#if defined(CONFIG_MODEM_SMS)
	MODEM_CMD("+CMTI:", on_cmd_unsol_sms, 2U, ","),
#if defined(CONFIG_MODEM_SMS_CALLBACK)
	MODEM_CMD("+CMT:", on_cmd_unsol_cmt, 2U, ",\r"),
#endif /* CONFIG_MODEM_SMS_CALLBACK */
#endif /* CONFIG_MODEM_SMS */
};

/**
 * @brief Handler for %SOCKETCMD:<socket_id> OK
 */
MODEM_CMD_DEFINE(on_cmd_sockopen)
{

	int sock_id = data->rx_buf->data[0] - '0';

	mdata.sock_fd = sock_id;
	modem_cmd_handler_set_error(data, 0);
	k_sem_give(&mdata.sem_sock_conn);

	return 0;
}

/**
 *
 * @brief Handler for %SOCKETCMD:<socket_id> OK
 *
 * @param argv[0] socket_stat
 * @param argv[1] socket_type
 * @param argv[2] src_ip
 * @param argv[3] dst_ip
 * @param argv[4] src_port
 * @param argv[5] dst_port
 *
 */
MODEM_CMD_DEFINE(on_cmd_sockinfo)
{
	struct modem_socket *sock = NULL;

	for (int i = 0; i < mdata.socket_config.sockets_len; i++) {
		if (mdata.socket_config.sockets[i].id == mdata.sock_fd) {
			sock = &mdata.socket_config.sockets[i];
			break;
		}
	}

	if (sock == NULL) {
		return -ENOENT;
	}

	if (sock->family == AF_INET) {
		net_addr_pton(AF_INET, argv[2], net_sin(&sock->src));
		net_sin(&sock->src)->sin_port = htons(strtol(argv[4], NULL, 10));
	} else {
		net_addr_pton(AF_INET6, argv[2], net_sin6(&sock->src));
		net_sin6(&sock->src)->sin6_port = htons(strtol(argv[4], NULL, 10));
	}

	return 0;
}

static bool got_pdn_flg;

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
#define CLIENT_CA_CERTIFICATE_TAG 1
static int offload_setsockopt(void *obj, int level, int optname, const void *optval,
			      socklen_t optlen);
sec_tag_t sec_tag_list[] = {
	CLIENT_CA_CERTIFICATE_TAG,
};

static char target_filename[MAX_FILENAME_LEN + 1];
static bool file_found;
MODEM_CMD_DEFINE(on_cmd_certcmd_dir)
{
	for (int i = 0; i < argc; i++) {
		if (!strcmp(argv[i], target_filename)) {
			file_found = true;
			return 0;
		}
	}
	file_found = false;
	return 0;
}

/**
 * @brief check whether filename exists in modem's D:CERTS/USER/ folder
 * @return 0 if file exists on modem; -1 if not
 */
static int check_mdm_store_file(const char *filename)
{
	int ret = 0;
	char at_cmd[60];

	got_pdn_flg = false;
	file_found = false;
	struct modem_cmd data_cmd[] = {
		MODEM_CMD_ARGS_MAX("%CERTCMD:", on_cmd_certcmd_dir, 0U, 255U, ","),
	};
	memset(target_filename, 0, sizeof(target_filename));
	strncpy(target_filename, filename, sizeof(target_filename) - 1);

	LOG_DBG("%s", at_cmd);
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, 1, "AT%CERTCMD=\"DIR\"",
			     &mdata.sem_response, MDM_CMD_RSP_TIME);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
		ret = -1;
	}
	if (!file_found) {
		return -1;
	}

	return ret;
}

/**
 * @brief Misc init after normal mdm init
 */
static int post_mdm_init(void)
{

	int ret = 0;
	return ret;
}
#endif

/**
 * @brief Handler for AT+COPS?
 */
MODEM_CMD_DEFINE(on_cmd_cops)
{
	char buf[32];
	int sz;
	size_t out_len = net_buf_linearize(buf, sizeof(buf) - 1, data->rx_buf, 0, len);

	buf[out_len] = '\0';

	LOG_DBG("full cops: %s", buf);
	sz = get_str_in_quotes(buf, mdata.mdm_carrier, sizeof(mdata.mdm_carrier));

	LOG_DBG("Carrier: %s", mdata.mdm_carrier);

	sz ? (errno = 0) : (errno = EINVAL);
	return sz ? 0 : -1;
}

/**
 * @brief Get connection status
 */
static int get_carrier(char *rbuf)
{
	int ret;
	static const char at_cmd[] = "AT+COPS?";
	struct modem_cmd data_cmd[] = {
		MODEM_CMD("+COPS:", on_cmd_cops, 0U, ","),
	};

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, ARRAY_SIZE(data_cmd), at_cmd,
			     &mdata.sem_response, MDM_CMD_LONG_RSP_TIME);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
		ret = -1;
	}
	snprintk(rbuf, MAX_CARRIER_RESP_SIZE, "%s", mdata.mdm_carrier);
	return ret;
}

/**
 * @brief Get PSM
 */
static int get_psm(char *response)
{
	int ret;
	static const char at_cmd[] = "AT+CPSMS?";
	struct modem_cmd data_cmd[] = {
		MODEM_CMD("+CPSMS:", on_cmd_get_psm, 0U, ","),
	};
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, ARRAY_SIZE(data_cmd), at_cmd,
			     &mdata.sem_response, MDM_CMD_RSP_TIME);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
		ret = -1;
	}
	snprintk(response, MAX_PSM_RESP_SIZE, "%s", mdata.mdm_psm);
	return ret;
}

/**
 * @brief Get edrx
 */
static int get_edrx(char *response)
{
	int ret;
	static const char at_cmd[] = "AT+CEDRXS?";
	struct modem_cmd data_cmd[] = {
		MODEM_CMD("+CEDRXS:", on_cmd_get_edrx, 0U, ","),
	};

	memset(mdata.mdm_edrx, 0, sizeof(mdata.mdm_edrx));

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, ARRAY_SIZE(data_cmd), at_cmd,
			     &mdata.sem_response, MDM_CMD_RSP_TIME);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
		ret = -1;
	}
	snprintk(response, MAX_EDRX_RESP_SIZE, "%s", mdata.mdm_edrx);
	return ret;
}

/**
 * @brief Reset the modem
 */
static int reset_modem(void)
{
	static const char at_cmd[] = "ATZ";
	int ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, at_cmd,
				 &mdata.sem_response, MDM_CMD_RSP_TIME);
	if (ret < 0) {
		LOG_ERR("Error rebooting modem");
	} else {
		LOG_INF("Waiting %d secs for modem to boot...", MDM_BOOT_DELAY);
		k_sleep(K_SECONDS(MDM_BOOT_DELAY));
	}
	return ret;
}

/**
 * @brief Close the given socket
 */
static void socket_close(struct modem_socket *sock)
{
	char at_cmd[40];
	int ret;

	if (modem_socket_id_is_assigned(&mdata.socket_config, sock)) {

		/* Tell the modem to close the socket. */
		snprintk(at_cmd, sizeof(at_cmd), "AT%%SOCKETCMD=\"DEACTIVATE\",%d", sock->id);
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, at_cmd,
				     &mdata.sem_response, MDM_CMD_RSP_TIME);

		if (ret < 0) {
			LOG_ERR("%s ret:%d", at_cmd, ret);
		}

		/* Tell the modem to delete the socket. */
		snprintk(at_cmd, sizeof(at_cmd), "AT%%SOCKETCMD=\"DELETE\",%d", sock->id);
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, at_cmd,
				     &mdata.sem_response, MDM_CMD_RSP_TIME);
		if (ret < 0) {
			LOG_ERR("%s ret:%d", at_cmd, ret);
		}
	}
	modem_socket_put(&mdata.socket_config, sock->sock_fd);
}

#if defined(CONFIG_MODEM_SMS)

/**
 * Parses list sms and add them to buffer.
 * Format is:
 *
 * +CMGL: <index>,<stat>,,<length><CR><LF><pdu><CR><LF>
 * +CMGL: <index>,<stat>,,<length><CR><LF><pdu><CR><LF>
 * ...
 * OK
 */
MODEM_CMD_DEFINE(on_cmd_cmgl)
{
	char pdu_buffer[200]; /* We dont actually need the whole thing */
	char raw_ts[8] = {0};
	uint64_t ts, min_ts;
	size_t out_len, sms_len, param_len;
	struct sms_in *sms;

	/* Get the length of the "length" parameter.
	 * The last parameter will be stuck in the netbuffer.
	 * It is not the actual length of the trailing pdu so
	 * we have to search the next crlf.
	 */
	param_len = net_buf_find_crlf(data->rx_buf, 0);
	if (param_len == 0) {
		LOG_DBG("No <CR><LF>");
		return -EAGAIN;
	}

	/* Get actual trailing pdu len. +2 to skip crlf. */
	sms_len = net_buf_find_crlf(data->rx_buf, param_len + 2);
	if (sms_len == 0) {
		return -EAGAIN;
	}

	/* Skip to start of pdu. */
	data->rx_buf = net_buf_skip(data->rx_buf, param_len + 2);
	out_len = net_buf_linearize(pdu_buffer, sizeof(pdu_buffer) - 1, data->rx_buf, 0, sms_len);
	pdu_buffer[out_len] = '\0';

	data->rx_buf = net_buf_skip(data->rx_buf, sms_len);

	/* No buffer specified. */
	if (!mdata.sms) {
		return 0;
	}
	sms = mdata.sms;

	struct deliver_pdu_data_s pdu_data;

	deliver_pdu_parse(pdu_buffer, &pdu_data);

	for (int i = 0; i < 14; i += 2) {
		byteswp(&pdu_data.scts[i], &pdu_data.scts[i + 1], 1);
	}

	memcpy(raw_ts, pdu_data.scts, 7);
	ts = strtol(raw_ts, NULL, 10) * 100000;
	memset(raw_ts, 0, sizeof(raw_ts));
	memcpy(raw_ts, pdu_data.scts + 7, 5);
	ts += strtol(raw_ts, NULL, 10);

	if (strlen(sms->time)) {
		memcpy(raw_ts, sms->time, 7);
		min_ts = strtol(raw_ts, NULL, 10) * 100000;
		memset(raw_ts, 0, sizeof(raw_ts));
		memcpy(raw_ts, sms->time + 7, 5);
		min_ts += strtol(raw_ts, NULL, 10);
	} else {
		min_ts = UINT64_MAX;
	}

	if (ts < min_ts) {
		uint8_t csms_ref = 0, csms_idx = 0;

		memcpy(sms->time, pdu_data.scts, 12);
		memset(mdata.sms_indices, 0, sizeof(mdata.sms_indices));
		memset(mdata.sms_csms_indices, 0, sizeof(mdata.sms_csms_indices));

		if (pdu_data.udhl) {
			char *udh = pdu_data.ud + 2;
			uint8_t iei, iedl;

			while (udh < (pdu_data.ud + 2 + (pdu_data.udhl * 2))) {
				iei = hex_byte_to_data(udh);
				udh += 2;
				iedl = hex_byte_to_data(udh);
				udh += 2;
				if (iei != 0) {
					LOG_WRN("Unknown UDH Identifier %d", iei);
					udh += iedl * 2;
				} else {
					csms_ref = hex_byte_to_data(udh);
					udh += 4;
					csms_idx = hex_byte_to_data(udh);
					udh += 2;
				}
			}
		}
		if (sms->csms_ref != csms_ref) {
			mdata.sms_indices[0] = strtol(argv[0], NULL, 10);
			mdata.sms_csms_indices[0] = sms->csms_idx;
			sms->csms_ref = csms_ref;
			sms->csms_idx = csms_idx;
		} else {
			for (int i = 0; i < ARRAY_SIZE(mdata.sms_indices); i++) {
				if (!mdata.sms_indices[i]) {
					mdata.sms_indices[i] = strtol(argv[0], NULL, 10);
					mdata.sms_csms_indices[i] = csms_idx;
					break;
				}
			}
		}
	} else if (pdu_data.udhl && sms->csms_ref) {
		char *udh = pdu_data.ud + 2;
		uint8_t iei, iedl, csms_ref = 0, csms_idx = 0;

		while (udh < (pdu_data.ud + 2 + (pdu_data.udhl * 2))) {
			iei = hex_byte_to_data(udh);
			udh += 2;
			iedl = hex_byte_to_data(udh);
			udh += 2;
			if (iei != 0) {
				LOG_WRN("Unknown UDH Identifier %d", iei);
				udh += iedl * 2;
			} else {
				csms_ref = hex_byte_to_data(udh);
				udh += 4;
				csms_idx = hex_byte_to_data(udh);
				udh += 2;
			}
		}
		if (!csms_ref || csms_ref != sms->csms_ref) {
			return 0;
		}
		for (int i = 0; i < ARRAY_SIZE(mdata.sms_indices); i++) {
			if (!mdata.sms_indices[i]) {
				mdata.sms_indices[i] = strtol(argv[0], NULL, 10);
				mdata.sms_csms_indices[i] = csms_idx;
				break;
			}
		}
	}
	return 0;
}

/**
 * Parses list sms and add them to buffer.
 * Format is:
 *
 * +CMGR: <stat>,,<length><CR><LF><pdu><CR><LF>
 *
 * OK
 */
MODEM_CMD_DEFINE(on_cmd_cmgr)
{
	char pdu_buffer[360];
	size_t out_len, sms_len, param_len;

	/* Get the length of the "length" parameter.
	 * The last parameter will be stuck in the netbuffer.
	 * It is not the actual length of the trailing pdu so
	 * we have to search the next crlf.
	 */
	param_len = net_buf_find_crlf(data->rx_buf, 0);
	if (param_len == 0) {
		LOG_DBG("No <CR><LF>");
		return -EAGAIN;
	}

	/* Get actual trailing pdu len. +2 to skip crlf. */
	sms_len = net_buf_find_crlf(data->rx_buf, param_len + 2);
	if (sms_len == 0) {
		return -EAGAIN;
	}

	/* Skip to start of pdu. */
	data->rx_buf = net_buf_skip(data->rx_buf, param_len + 2);
	out_len = net_buf_linearize(pdu_buffer, sizeof(pdu_buffer) - 1, data->rx_buf, 0, sms_len);
	pdu_buffer[out_len] = '\0';

	data->rx_buf = net_buf_skip(data->rx_buf, sms_len);

	return pdu_msg_extract(pdu_buffer, NULL);
}

/**
 * @brief Send an sms message
 */
static int send_sms_msg(const struct sms_out *sms)
{
	/* The "+ 20" is to account for AT+CMGS plus a bit extra */
	char at_cmd[sizeof(struct sms_out) + 21];
	int ret;

	k_sem_take(&mdata.sem_sms, K_FOREVER);

	snprintk(at_cmd, sizeof(at_cmd), "AT+CMGF=1");
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, at_cmd, &mdata.sem_response,
			     MDM_CMD_LONG_RSP_TIME);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
	}

	snprintk(at_cmd, sizeof(at_cmd), "AT%%CMGSC=\"%s\"\r%s\x1a", sms->phone, sms->msg);
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, at_cmd, &mdata.sem_response,
			     MDM_CMD_LONG_RSP_TIME);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
	}

	k_msleep(100);

	snprintk(at_cmd, sizeof(at_cmd), "AT+CMGF=0");
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, at_cmd, &mdata.sem_response,
			     MDM_CMD_LONG_RSP_TIME);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
	}

	k_sem_give(&mdata.sem_sms);

	return ret;
}

/**
 * @brief Receive an sms message
 */
static int recv_sms_msg(struct sms_in *sms)
{
	int ret;

	if (!sms) {
		errno = EINVAL;
		return -1;
	}
	memset(sms->msg, 0, sizeof(sms->msg));
	memset(sms->phone, 0, sizeof(sms->phone));
	memset(sms->time, 0, sizeof(sms->time));
	sms->csms_idx = sms->csms_ref = 0;
	struct modem_cmd cmds[] = {
		MODEM_CMD("+CMGL: ", on_cmd_cmgl, 4U, ",\r"),
		MODEM_CMD("+CMGR: ", on_cmd_cmgr, 3U, ",\r"),
	};
	memset(mdata.sms_indices, 0, sizeof(mdata.sms_indices));
	memset(mdata.sms_csms_indices, 0, sizeof(mdata.sms_indices));

	k_sem_take(&mdata.sem_sms, K_FOREVER);

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, "AT+CMGF=0",
			     &mdata.sem_response, MDM_CMD_LONG_RSP_TIME);
	mdata.sms = sms;
	k_sem_reset(&mdata.sem_rcv_sms);
	int count = 0;

	while (count <= 1) {
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, cmds, ARRAY_SIZE(cmds),
				     "AT+CMGL=4", &mdata.sem_response, MDM_CMD_LONG_RSP_TIME);
		if (ret == 0 && !mdata.sms_indices[0]) {
			ret = k_sem_take(&mdata.sem_rcv_sms, sms->timeout);
			if (ret < 0) {
				/* timed out waiting for semaphore, set ret code
				 * to 0 (no msg available)
				 */
				ret = 0;
				break;
			}
		} else if (mdata.sms_indices[0]) {
			break;
		}
		count++;
	}
	if (ret < 0) {
		k_sem_give(&mdata.sem_sms);
		return -1;
	}

	if (mdata.sms_indices[0]) {
		/* Perform read now */
		bool out_of_order = true;

		while (out_of_order && sms->csms_ref) {
			out_of_order = false;
			for (int i = 0; i < ARRAY_SIZE(mdata.sms_indices) - 1; i++) {
				if ((mdata.sms_csms_indices[i] > mdata.sms_csms_indices[i + 1]) &&
				    mdata.sms_csms_indices[i + 1]) {
					uint8_t tmp = mdata.sms_csms_indices[i];

					mdata.sms_csms_indices[i] = mdata.sms_csms_indices[i + 1];
					mdata.sms_csms_indices[i + 1] = tmp;
					tmp = mdata.sms_indices[i];
					mdata.sms_indices[i] = mdata.sms_indices[i + 1];
					mdata.sms_indices[i + 1] = tmp;
					out_of_order = true;
				}
			}
		}
		int sms_len;

		for (int i = 0; i < ARRAY_SIZE(mdata.sms_indices); i++) {
			if (!mdata.sms_indices[i]) {
				break;
			}
			sms_len = strlen(sms->msg);
			char at_cmd[32];

			snprintk(at_cmd, sizeof(at_cmd), "AT+CMGR=%d", mdata.sms_indices[i]);
			ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, cmds, ARRAY_SIZE(cmds),
					     at_cmd, &mdata.sem_response, MDM_CMD_LONG_RSP_TIME);
			if (ret || sms_len == strlen(sms->msg)) {
				memset(&mdata.sms_indices[i], 0,
				       sizeof(mdata.sms_indices) -
					       sizeof(mdata.sms_indices[0]) * i);
				break;
			}
		}
	}

	for (int i = 0; i < ARRAY_SIZE(mdata.sms_indices); i++) {
		if (mdata.sms_indices[i]) {
			char at_cmd[32];

			snprintk(at_cmd, sizeof(at_cmd), "AT+CMGD=%d", mdata.sms_indices[i]);
			ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, at_cmd,
					     &mdata.sem_response, MDM_CMD_RSP_TIME);
			if (ret < 0) {
				LOG_ERR("%s ret:%d", at_cmd, ret);
			}
		} else {
			break;
		}
	}
	k_sem_give(&mdata.sem_sms);
	return strlen(sms->msg);
}

#if defined(CONFIG_MODEM_SMS_CALLBACK)
static int recv_sms_cb_en(bool enable)
{
	int ret = 0;
	char *at_cmd;

	if (enable) {
		at_cmd = "AT+CNMI=2,2,0,1,0";
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, at_cmd,
						&mdata.sem_response, MDM_CMD_RSP_TIME);

		if (ret < 0) {
			LOG_ERR("%s ret:%d", at_cmd, ret);
		}
	} else {
		at_cmd = "AT+CNMI=2,1,0,1,0";
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, at_cmd,
						&mdata.sem_response, MDM_CMD_RSP_TIME);

		if (ret < 0) {
			LOG_ERR("%s ret:%d", at_cmd, ret);
		}
	}
	return ret;
}
#endif /* CONFIG_MODEM_SMS_CALLBACK */
#endif /* CONFIG_MODEM_SMS */

/**
 * @brief Receive data on a socket
 */
static ssize_t offload_recvfrom(void *obj, void *buf, size_t len, int flags, struct sockaddr *from,
				socklen_t *fromlen)
{
	struct modem_socket *sock = (struct modem_socket *)obj;
	char sendbuf[100];
	int ret = 0;
	struct socket_read_data sock_data;

	/* Modem command to read the data. */
	struct modem_cmd data_cmd[] = {MODEM_CMD("ERROR", on_cmd_error, 0U, ""),
				       MODEM_CMD("%SOCKETDATA:", on_cmd_sock_readdata, 3U, ",")};

	LOG_DBG("IN %s, flags = 0x%x", __func__, flags);
	LOG_DBG("buf = 0x%x, len = %d\n", (unsigned int)buf, len);

	if (!buf || len == 0) {
		errno = EINVAL;
		return -1;
	}

	if (flags & ZSOCK_MSG_PEEK) {
		errno = ENOTSUP;
		return -1;
	}

	if (len > MDM_MAX_DATA_LENGTH) {
		len = MDM_MAX_DATA_LENGTH;
	}

	int packet_size = modem_socket_next_packet_size(&mdata.socket_config, sock);
	if (!packet_size) {
		if (flags & ZSOCK_MSG_DONTWAIT) {
			errno = EAGAIN;
			return -1;
		}

		if (!sock->is_connected && sock->ip_proto != IPPROTO_UDP) {
			errno = 0;
			return 0;
		}

		if (modem_socket_wait_data(&mdata.socket_config, sock)) {
			errno = EAGAIN;
			return -1;
		}
		packet_size = modem_socket_next_packet_size(&mdata.socket_config, sock);
	}

	/* Socket read settings */
	(void)memset(&sock_data, 0, sizeof(sock_data));
	sock_data.recv_buf = mdata.xlate_buf;
	sock_data.recv_buf_len = sizeof(mdata.xlate_buf);
	sock_data.recv_addr = from;
	sock->data = &sock_data;
	mdata.sock_fd = sock->id;

	/* use dst address as from */
	if (from && fromlen) {
		*fromlen = sizeof(sock->dst);
		memcpy(from, &sock->dst, *fromlen);
	}

	snprintk(sendbuf, sizeof(sendbuf), "AT%%SOCKETDATA=\"RECEIVE\",%u,%u", sock->id, len);

	LOG_DBG("%s", sendbuf);

	/* Lock the xlate buffer */
	k_sem_take(&mdata.sem_xlate_buf, K_FOREVER);

	/* Tell the modem to give us data (%SOCKETDATA:socket_id,len,0,data). */
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, ARRAY_SIZE(data_cmd),
			     sendbuf, &mdata.sem_response, MDM_CMD_LONG_RSP_TIME);

	LOG_DBG("Returned from modem_cmd_send with ret=%d", ret);
	LOG_DBG("rec_len = %d", sock_data.recv_read_len);

	if (ret < 0) {
		k_sem_give(&mdata.sem_xlate_buf);
		errno = -ret;
		goto exit;
	}

	/* return length of received data */
	hex_str_to_data(mdata.xlate_buf, (uint8_t *)buf, sock_data.recv_read_len);
	k_sem_give(&mdata.sem_xlate_buf);
	errno = 0;

	/* Use dst address as from */
	if (from && fromlen) {
		*fromlen = sizeof(sock->dst);
		memcpy(from, &sock->dst, *fromlen);
	}

	/* return length of received data */
	errno = 0;
	ret = sock_data.recv_read_len;

exit:
	/* clear socket data */
	sock->data = NULL;
	return ret;
}

/**
 * @brief Implement the socket function for the modem
 */
static int offload_socket(int family, int type, int proto)
{
	int ret;

	/* defer modem's socket create call to bind() */
	ret = modem_socket_get(&mdata.socket_config, family, type, proto);

	if (ret == -ENOMEM) {
		ret = -ENFILE;
	}

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	errno = 0;
	return ret;
}

#define ALLOCATE_CMD_SZ sizeof("AT%SOCKETCMD=\"ALLOCATE\",1,\"TCP\",\"OPEN\",\"\",XXXXX,XXXXX")

/**
 * @brief Connect with a TCP or UDP peer
 */
static int offload_connect(void *obj, const struct sockaddr *addr, socklen_t addrlen)
{
	struct modem_socket *sock = (struct modem_socket *)obj;
	uint16_t dst_port = 0;
	uint16_t src_port = 0;
	char protocol[5];
	char at_cmd[ALLOCATE_CMD_SZ + CONFIG_MURATA_MODEM_SNI_BUFFER_SZ];
	int ret;

	LOG_DBG("In %s, sock->id: %d, sock->sock_fd: %d", __func__, sock->id, sock->sock_fd);

	struct modem_cmd cmd[] = {
		MODEM_CMD("ERROR", on_cmd_error, 0, ","),
		MODEM_CMD("%SOCKETCMD:", on_cmd_sockopen, 0U, ""),
	};

	if (addrlen > sizeof(struct sockaddr)) {
		errno = EINVAL;
		return -1;
	}

	/* Verify socket has been allocated */
	if (!modem_socket_is_allocated(&mdata.socket_config, sock)) {
		LOG_ERR("Invalid socket_id(%d) from fd:%d", sock->id, sock->sock_fd);
		errno = EINVAL;
		return -1;
	}

	if (sock->is_connected == true) {
		LOG_ERR("Socket is already connected!! socket_id(%d), "
			"socket_fd:%d",
			sock->id, sock->sock_fd);
		errno = EISCONN;
		return -1;
	}

	if (modem_socket_id_is_assigned(&mdata.socket_config, sock)) {
		socket_close(sock);
		modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, at_cmd,
				 &mdata.sem_response, MDM_CMD_LONG_RSP_TIME);
		if (sock->src.sa_family == AF_INET) {
			src_port = ntohs(net_sin(&sock->src)->sin_port);
		} else {
			src_port = ntohs(net_sin6(&sock->src)->sin6_port);
		}
	}

	sock->is_connected = true;

	switch (sock->ip_proto) {
	case IPPROTO_DTLS_1_2:
	case IPPROTO_UDP:
		snprintk(protocol, sizeof(protocol), "UDP");
		break;
	case IPPROTO_TCP:
#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
	case IPPROTO_TLS_1_2:
#endif
		snprintk(protocol, sizeof(protocol), "TCP");
		break;
	default:
		LOG_ERR("INVALID PROTOCOL %d", sock->ip_proto);
		socket_close(sock);
		return -1;
	}

	/* Find the correct destination port. */
	if (addr->sa_family == AF_INET6) {
		dst_port = ntohs(net_sin6(addr)->sin6_port);
	} else if (addr->sa_family == AF_INET) {
		dst_port = ntohs(net_sin(addr)->sin_port);
	}

	k_sem_reset(&mdata.sem_sock_conn);

	/* get IP and save to buffer */
#if CONFIG_NET_IPV6
	char ip_addr[NET_IPV6_ADDR_LEN];
#else
	char ip_addr[NET_IPV4_ADDR_LEN];
#endif
	modem_context_sprint_ip_addr(addr, ip_addr, sizeof(ip_addr));

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
	/* Formulate the string to allocate socket. */
	if (!murata_sock_tls_info[get_socket_idx(sock)].sni_valid) {
#endif
		snprintk(at_cmd, sizeof(at_cmd),
			 "AT%%SOCKETCMD=\"ALLOCATE\",1,\"%s\",\"OPEN\",\"%s\",%d,%d",
			 protocol, ip_addr, dst_port, src_port);
#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
	} else {
		snprintk(at_cmd, sizeof(at_cmd),
			 "AT%%SOCKETCMD=\"ALLOCATE\",1,\"%s\",\"OPEN\",\"%s\",%d,%d",
			 protocol, murata_sock_tls_info[get_socket_idx(sock)].host, dst_port,
			 src_port);
	}
#endif

	/* Send out the command. */
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, cmd, ARRAY_SIZE(cmd), at_cmd,
			     &mdata.sem_response, MDM_CMD_RSP_TIME);

	if (ret < 0) {
		LOG_DBG("%s ret:%d", at_cmd, ret);
		LOG_DBG("Closing the socket!!!");
		socket_close(sock);
		errno = -ret;
		return -1;
	}

	ret = k_sem_take(&mdata.sem_sock_conn, K_SECONDS(1));
	if (ret < 0) {
		LOG_ERR("Timeout for waiting for sockconn; closing socket!");
		socket_close(sock);
		errno = -ret;
		return -1;
	}

	LOG_DBG("store %d into sock: %p", mdata.sock_fd, sock);
	sock->id = mdata.sock_fd;

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
	if (sock->ip_proto == IPPROTO_TLS_1_2 || sock->ip_proto == IPPROTO_DTLS_1_2) {
		int profileID = murata_sock_tls_info[get_socket_idx(sock)].profile;
		int ssl_mode = 0;

		ssl_mode |= murata_sock_tls_info[get_socket_idx(sock)].client_verify ? BIT(1) : 0;
		ssl_mode |=
			murata_sock_tls_info[get_socket_idx(sock)].peer_verify_disable ? 0 : BIT(0);
		ssl_mode = (~ssl_mode) & 3;

		snprintk(at_cmd, sizeof(at_cmd), "AT%%SOCKETCMD=\"SSLALLOC\",%d,%d,%d",
			 sock->id, ssl_mode, profileID);

		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, at_cmd,
				     &mdata.sem_response, MDM_CMD_LONG_RSP_TIME);
		LOG_DBG("%s", at_cmd);
		if (ret < 0) {
			LOG_DBG("%s ret: %d", at_cmd, ret);
			LOG_DBG("Closing the socket!!!");
			socket_close(sock);
			errno = -ret;
			return -1;
		}
	}
#endif

	snprintk(at_cmd, sizeof(at_cmd), "AT%%SOCKETCMD=\"ACTIVATE\",%d", sock->id);
	LOG_DBG("%s", at_cmd);
	/* Send out the command. */
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, at_cmd, &mdata.sem_response,
			     MDM_CMD_LONG_RSP_TIME);

	if (ret < 0) {
		LOG_DBG("%s ret: %d", at_cmd, ret);
		LOG_DBG("Closing the socket!!!");
		socket_close(sock);
		errno = -ret;
		return -1;
	}

	/* set command handlers */
	ret = modem_cmd_handler_update_cmds(&mdata.cmd_handler_data, cmd, ARRAY_SIZE(cmd), true);
	if (ret < 0) {
		LOG_ERR("Failed to update cmds, ret= %d", ret);
		goto exit;
	}

	/* Connected successfully. */
	errno = 0;
	memcpy(&sock->dst, addr, addrlen);
	return 0;

exit:
	(void)modem_cmd_handler_update_cmds(&mdata.cmd_handler_data, NULL, 0U, false);
	errno = -ret;
	return -1;
}

/**
 * @brief Send data on the socket object
 */
static ssize_t offload_sendto(void *obj, const void *buf, size_t len, int flags,
			      const struct sockaddr *to, socklen_t tolen)
{
	int ret;
	struct modem_socket *sock = (struct modem_socket *)obj;

	struct modem_cmd cmd_info[] = {
		MODEM_CMD("ERROR", on_cmd_error, 0, ","),
		MODEM_CMD("%SOCKETCMD:", on_cmd_sockinfo, 6U, ","),
	};

	/* Ensure that valid parameters are passed. */
	if (!buf || len <= 0) {
		errno = EINVAL;
		return -1;
	}

	if (!sock->is_connected) {
		if (sock->type == SOCK_DGRAM && tolen != 0 && to != NULL) {
			if (!modem_socket_id_is_assigned(&mdata.socket_config, sock)) {
				char at_cmd[128];
				uint16_t port;
				void *addr_ptr;

				if (to->sa_family == AF_INET) {
					addr_ptr = &net_sin(to)->sin_addr;
					port = net_sin(to)->sin_port;
				} else {
					addr_ptr = &net_sin6(to)->sin6_addr;
					port = net_sin6(to)->sin6_port;
				}

				offload_connect(sock, to, tolen);

				sock->is_connected = false;

				snprintk(at_cmd, sizeof(at_cmd),
					"AT%%SOCKETCMD=\"INFO\",%d",
					sock->id);

				ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, cmd_info,
					ARRAY_SIZE(cmd_info), at_cmd, &mdata.sem_response,
					MDM_CMD_RSP_TIME);

			}
		} else {
			errno = ENOTCONN;
			return -1;
		}
	} else {
		/* if already connected, to should be NULL and tolen should be 0 or if
		 * not, check whether it is the same as the connected socket
		 */
		if (to != NULL || tolen != 0) {
			if ((to == NULL && tolen) || ((to != NULL) && !tolen) ||
			    (memcmp(to, &sock->dst, tolen) != 0)) {
				errno = EISCONN;
				return -1;
			}
		}
	}
	return send_socket_data(sock, to, buf, len, MDM_CMD_TIMEOUT);
}

/**
 * Implement the bind function for the modem
 */
static int offload_bind(void *obj, const struct sockaddr *addr, socklen_t addrlen)
{
	struct modem_socket *sock = (struct modem_socket *)obj;

	/* save bind address information */
	memcpy(&sock->src, addr, sizeof(*addr));

	/* make sure we've created the socket */
	if (sock->id == mdata.socket_config.sockets_len + 1) {
		if (offload_connect(obj, addr, addrlen) < 0) {
			return -1;
		}
	}

	return 0;
}

/**
 * @brief Read data from the given socket object
 */
static ssize_t offload_read(void *obj, void *buffer, size_t count)
{
	return offload_recvfrom(obj, buffer, count, 0, NULL, 0);
}

/**
 * @brief Write data to the given socket object
 */
static ssize_t offload_write(void *obj, const void *buffer, size_t count)
{
	return offload_sendto(obj, buffer, count, 0, NULL, 0);
}

/**
 * @brief Close the connection with the remote client and free the socket
 */
static int offload_close(void *obj)
{
	struct modem_socket *sock = (struct modem_socket *)obj;

	/* Make sure we assigned an id */
	if (!modem_socket_is_allocated(&mdata.socket_config, sock)) {
		return 0;
	}

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
	murata_sock_tls_info[get_socket_idx(sock)].sni_valid = false;
	murata_sock_tls_info[get_socket_idx(sock)].profile = 0;
	murata_sock_tls_info[get_socket_idx(sock)].peer_verify_disable = false;
	murata_sock_tls_info[get_socket_idx(sock)].client_verify = false;
#endif
	/* Close the socket only if it is connected. */
	socket_close(sock);

	return 0;
}

/**
 * @brief Send messages to the modem
 */
static ssize_t offload_sendmsg(void *obj, const struct msghdr *msg, int flags)
{
	ssize_t sent = 0;
	int rc;

	LOG_DBG("msg_iovlen:%zd flags:%d", msg->msg_iovlen, flags);

	for (int i = 0; i < msg->msg_iovlen; i++) {
		const char *buf = msg->msg_iov[i].iov_base;
		size_t len = msg->msg_iov[i].iov_len;

		while (len > 0) {
			rc = offload_sendto(obj, buf, len, flags, msg->msg_name, msg->msg_namelen);
			if (rc < 0) {
				if (rc == -EAGAIN) {
					k_sleep(MDM_SENDMSG_SLEEP);
				} else {
					sent = rc;
					break;
				}
			} else {
				sent += rc;
				buf += rc;
				len -= rc;
			}
		}
	}

	return (ssize_t)sent;
}

static struct zsock_addrinfo zsai[2] = {0};
static struct sockaddr_in6 zai_addr[2] = {0};

static void murata_1sc_freeaddrinfo(struct zsock_addrinfo *res)
{
	/* No need to free static memory. */
	res = NULL;
}

static inline uint32_t qtupletouint(uint8_t *ia)
{
	return *(uint32_t *)ia;
}

static int ai_idx;

static int set_addr_info(uint8_t *addr, bool ipv6, uint8_t socktype, uint16_t port,
			 struct zsock_addrinfo **res)
{
	struct zsock_addrinfo *ai;
	struct sockaddr *ai_addr;
	int retval = 0;

	if (ipv6) {
		if (!(qtupletouint(&addr[0]) || qtupletouint(&addr[4]) || qtupletouint(&addr[8]) ||
		      qtupletouint(&addr[12]))) {
			return 0;
		}
	} else {
		if (!qtupletouint(addr)) {
			return 0;
		}
	}

	ai = &zsai[ai_idx];
	ai_addr = (struct sockaddr *)&zai_addr[ai_idx];
	memset(ai, 0, sizeof(struct zsock_addrinfo));
	memset(ai_addr, 0, sizeof(struct sockaddr));
	ai_idx++;
	ai_idx %= ARRAY_SIZE(zsai);

	ai->ai_family = (ipv6 ? AF_INET6 : AF_INET);
	ai->ai_socktype = socktype;
	ai->ai_protocol = ai->ai_socktype == SOCK_STREAM ? IPPROTO_TCP : IPPROTO_UDP;

	/* Fill sockaddr struct fields based on family: */
	if (ai->ai_family == AF_INET) {
		net_sin(ai_addr)->sin_family = ai->ai_family;
		net_sin(ai_addr)->sin_addr.s_addr = qtupletouint(addr);
		net_sin(ai_addr)->sin_port = htons(port);
		ai->ai_addrlen = sizeof(struct sockaddr_in);
	} else {
		net_sin6(ai_addr)->sin6_family = ai->ai_family;
		net_sin6(ai_addr)->sin6_addr.s6_addr32[0] = qtupletouint(&addr[0]);
		net_sin6(ai_addr)->sin6_addr.s6_addr32[1] = qtupletouint(&addr[4]);
		net_sin6(ai_addr)->sin6_addr.s6_addr32[2] = qtupletouint(&addr[8]);
		net_sin6(ai_addr)->sin6_addr.s6_addr32[3] = qtupletouint(&addr[12]);
		net_sin6(ai_addr)->sin6_port = htons(port);
		ai->ai_addrlen = sizeof(struct sockaddr_in6);
	}
	ai->ai_addr = ai_addr;
	ai->ai_next = *res;
	*res = ai;
	return retval;
}

static int murata_1sc_getaddrinfo(const char *node, const char *service,
				  const struct zsock_addrinfo *hints, struct zsock_addrinfo **res)
{
	int32_t retval = DNS_EAI_FAIL;
	uint32_t port = 0;
	uint8_t type = SOCK_STREAM;

	ai_idx = 0;
	if (service) {
		port = (uint32_t)strtol(service, NULL, 10);
		if (port < 1 || port > USHRT_MAX) {
			return DNS_EAI_SERVICE;
		}
	}

	/* Check args: */
	if (!res) {
		retval = DNS_EAI_NONAME;
		goto exit;
	}

	bool v4 = true, v6 = true;

	if (hints) {
		if (hints->ai_family == AF_INET) {
			v6 = false;
		} else if (hints->ai_family == AF_INET6) {
			v4 = false;
		}
		type = hints->ai_socktype;
	}

	retval = get_dns_ip(node);

	if (retval < 0) {
		LOG_ERR("Could not resolve name: %s, retval: %d", node, retval);
		retval = DNS_EAI_NONAME;
		goto exit;
	}

	*res = NULL;
	if (v4) {
		retval = set_addr_info((uint8_t *)&mdm_dns_ip.ipv4.sin_addr.s_addr, false, type,
				       (uint16_t)port, res);
		if (retval < 0) {
			murata_1sc_freeaddrinfo(*res);
			LOG_ERR("Unable to set address info, retval: %d", retval);
			goto exit;
		}
	}
#if defined(CONFIG_NET_IPV6)
	if (v6) {
		retval = set_addr_info(mdm_dns_ip.ipv6.sin6_addr.s6_addr, true, type,
				       (uint16_t)port, res);
		if (retval < 0) {
			murata_1sc_freeaddrinfo(*res);
			LOG_ERR("Unable to set address info, retval: %d", retval);
			goto exit;
		}
	}
#endif
	if (!*res) {
		retval = DNS_EAI_NONAME;
	}
exit:
	return retval;
}

static int sigStrength;
#define MIN_SS	   -113 /* dBm */
#define NO_SIG_RAW 99
#define NO_SIG_RET -120
/**
 * @brief Handle the response to AT%CSQ
 *
 * Response format:
 * <RSSI (-113 + 2*n>,<BER>,<RSRQ>
 * n = 0 to 31 (-113 to -51 dBm)
 * OR 99 if not known or detectable
 * return NO_SIG_RET for this case
 */
MODEM_CMD_DEFINE(on_cmd_csq)
{
	char buf[MAX_SIGSTR_RESP_SIZE];
	char *endp;
	int ret;
	size_t out_len = net_buf_linearize(buf, MAX_SIGSTR_RESP_SIZE - 1, data->rx_buf, 0, len);
	buf[out_len] = '\0';

	for (int i = 0; i < MAX_SIGSTR_RESP_SIZE - 1; i++) {
		if (buf[i] == ',') {
			buf[i] = 0;
			break;
		}
	}
	ret = (int)strtol(buf, &endp, 10);
	if (ret == NO_SIG_RAW) {
		sigStrength = NO_SIG_RET;
	} else {
		sigStrength = MIN_SS + 2 * ret;
	}
	LOG_DBG("signal strength: %d dBm", ret);
	return 0;
}

/**
 * @brief Get signal strength
 */
static int get_sigstrength(char *rbuf)
{
	static const char at_cmd[] = "AT\%CSQ";
	int ret;

	struct modem_cmd data_cmd[] = {
		MODEM_CMD("%CSQ:", on_cmd_csq, 0U, ""),
	};

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, 1, at_cmd,
			     &mdata.sem_response, MDM_CMD_RSP_TIME);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
		ret = -1;
	}
	snprintk(rbuf, MAX_SIGSTR_RESP_SIZE, "%d dBm", sigStrength);

	return ret;
}

/**
 * @brief Handle response to AT+CNUM
 *
 * +CNUM: ,"16186961148",129
 */
MODEM_CMD_DEFINE(on_cmd_cnum)
{
	char buf[32];
	int strlen = 0;
	size_t out_len = net_buf_linearize(buf, 31, data->rx_buf, 0, len);

	buf[out_len] = '\0';

	strlen = get_str_in_quotes(buf, mdata.mdm_phn, sizeof(mdata.mdm_phn));

	LOG_DBG("got cnum: %s, str_len = %d", mdata.mdm_phn, strlen);
	return 0;
}

/**
 * @brief Get phone number
 */
static int get_cnum(char *rbuf)
{
	int ret;
	static const char at_cmd[] = "AT+CNUM";
	struct modem_cmd data_cmd[] = {
		MODEM_CMD("+CNUM:", on_cmd_cnum, 0U, ","),
	};

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, ARRAY_SIZE(data_cmd), at_cmd,
			     &mdata.sem_response, MDM_CMD_RSP_TIME);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
		ret = -1;
	}
	memcpy(rbuf, mdata.mdm_phn, sizeof(mdata.mdm_phn));
	return ret;
}

/**
 * @brief Handler for CGCONTRDP
 *
 * Sample response:
 *
 * AT at%pdnrdp=1
 * %PDNRDP:
 * 1,5,"iot.catm.gprs",25.181.12.55.255.255.255.240,25.181.12.49,10.177.0.34,10.177.0.210,,,,,,,,,
 */
MODEM_CMD_DEFINE(on_cmd_get_cgcontrdp)
{
#define PDN_BUF_SZ 256
	char pdn_buf[PDN_BUF_SZ];
	size_t out_len;
	int ret = 0;

	if (!got_pdn_flg) {
		got_pdn_flg = true;
		out_len = net_buf_linearize(pdn_buf, PDN_BUF_SZ - 1, data->rx_buf, 0, len);
		pdn_buf[out_len] = '\0';
		LOG_DBG("PDNRDP-data (len=%d, strlen=%d, dat: %s\n", len, out_len, pdn_buf);
		ret = parse_ipgwmask(pdn_buf, mdata.mdm_ip, mdata.mdm_nmask, mdata.mdm_gw);

		LOG_DBG("IP: %s, GW: %s, NMASK: %s", mdata.mdm_ip, mdata.mdm_gw, mdata.mdm_nmask);
	}
	return ret;
}

#ifdef CONFIG_NET_IPV4
/**
 * @brief Handler for primary IPv4 DNS server
 */
MODEM_CMD_DEFINE(on_cmd_get_ipv4_primary)
{
	char dns_server_addr[NET_IPV4_ADDR_LEN];
	struct sockaddr set_addr, cfg_addr;
	size_t out_len = net_buf_linearize(dns_server_addr, sizeof(dns_server_addr) - 1,
					   data->rx_buf, 0, len);
	bool addr_set = false, primary_addr_valid = false;

	out_len--;
	if (!out_len) {
		return 0;
	}

	dns_server_addr[out_len] = '\0';
	addr_set = net_ipaddr_parse(dns_server_addr, out_len, &set_addr);
	primary_addr_valid =
		net_ipaddr_parse(CONFIG_MODEM_MURATA_IPV4_DNS_PRIMARY,
				 strlen(CONFIG_MODEM_MURATA_IPV4_DNS_PRIMARY), &cfg_addr);
	if (!addr_set) {
		if (primary_addr_valid) {
			needto_set_dns_servers = true;
		}
		return 0;
	}
	if (addr_set && !primary_addr_valid) {
		needto_set_dns_servers = true;
		return 0;
	}
	if (!net_ipv4_addr_cmp(&net_sin(&set_addr)->sin_addr, &net_sin(&cfg_addr)->sin_addr)) {
		needto_set_dns_servers = true;
	}
	return 0;
}

/**
 * @brief Handler for secondady IPv4 DNS server
 */
MODEM_CMD_DEFINE(on_cmd_get_ipv4_secondary)
{
	char dns_server_addr[NET_IPV4_ADDR_LEN];
	struct sockaddr set_addr, cfg_addr;
	size_t out_len = net_buf_linearize(dns_server_addr, sizeof(dns_server_addr) - 1,
					   data->rx_buf, 0, len);
	bool addr_set = false, secondary_addr_valid = false;

	out_len--;
	if (!out_len) {
		return 0;
	}

	dns_server_addr[out_len] = '\0';
	addr_set = net_ipaddr_parse(dns_server_addr, out_len, &set_addr);
	secondary_addr_valid =
		net_ipaddr_parse(CONFIG_MODEM_MURATA_IPV4_DNS_SECONDARY,
				 strlen(CONFIG_MODEM_MURATA_IPV4_DNS_SECONDARY), &cfg_addr);
	if (!addr_set) {
		if (secondary_addr_valid) {
			needto_set_dns_servers = true;
		}
		return 0;
	}
	if (addr_set && !secondary_addr_valid) {
		needto_set_dns_servers = true;
		return 0;
	}
	if (!net_ipv4_addr_cmp(&net_sin(&set_addr)->sin_addr, &net_sin(&cfg_addr)->sin_addr)) {
		needto_set_dns_servers = true;
	}
	return 0;
}
#endif /* CONFIG_NET_IPV4 */

#ifdef CONFIG_NET_IPV6
/**
 * @brief Handler for primary IPv6 DNS server
 */
MODEM_CMD_DEFINE(on_cmd_get_ipv6_primary)
{
	char dns_server_addr[NET_IPV6_ADDR_LEN];
	struct sockaddr set_addr, cfg_addr;
	size_t out_len = net_buf_linearize(dns_server_addr, sizeof(dns_server_addr) - 1,
					   data->rx_buf, 0, len);
	bool addr_set = false, primary_addr_valid = false;

	out_len--;
	if (!out_len) {
		return 0;
	}

	dns_server_addr[out_len] = '\0';
	addr_set = net_ipaddr_parse(dns_server_addr, out_len, &set_addr);
	primary_addr_valid =
		net_ipaddr_parse(CONFIG_MODEM_MURATA_IPV6_DNS_PRIMARY,
				 strlen(CONFIG_MODEM_MURATA_IPV6_DNS_PRIMARY), &cfg_addr);
	if (!addr_set) {
		if (primary_addr_valid) {
			needto_set_dns_servers = true;
		}
		return 0;
	}
	if (addr_set && !primary_addr_valid) {
		needto_set_dns_servers = true;
		return 0;
	}
	if (!net_ipv6_addr_cmp(&net_sin6(&set_addr)->sin6_addr, &net_sin6(&cfg_addr)->sin6_addr)) {
		needto_set_dns_servers = true;
	}
	return 0;
}

/**
 * @brief Handler for secondady IPv6 DNS server
 */
MODEM_CMD_DEFINE(on_cmd_get_ipv6_secondary)
{
	char dns_server_addr[NET_IPV6_ADDR_LEN];
	struct sockaddr set_addr, cfg_addr;
	size_t out_len = net_buf_linearize(dns_server_addr, sizeof(dns_server_addr) - 1,
					   data->rx_buf, 0, len);
	bool addr_set = false, secondary_addr_valid = false;

	out_len--;
	if (!out_len) {
		return 0;
	}

	dns_server_addr[out_len] = '\0';
	addr_set = net_ipaddr_parse(dns_server_addr, out_len, &set_addr);
	secondary_addr_valid =
		net_ipaddr_parse(CONFIG_MODEM_MURATA_IPV6_DNS_SECONDARY,
				 strlen(CONFIG_MODEM_MURATA_IPV6_DNS_SECONDARY), &cfg_addr);
	if (!addr_set) {
		if (secondary_addr_valid) {
			needto_set_dns_servers = true;
		}
		return 0;
	}
	if (addr_set && !secondary_addr_valid) {
		needto_set_dns_servers = true;
		return 0;
	}
	if (!net_ipv6_addr_cmp(&net_sin6(&set_addr)->sin6_addr, &net_sin6(&cfg_addr)->sin6_addr)) {
		needto_set_dns_servers = true;
	}
	return 0;
}
#endif /* CONFIG_NET_IPV6 */

/**
 * @brief Get ip/mask/gw
 */
static int get_ip(char *rbuf)
{
	int ret;
	static const char at_cmd[] = "AT+CGCONTRDP";

	got_pdn_flg = false;
	struct modem_cmd data_cmd[] = {
		MODEM_CMD("+CGCONTRDP:", on_cmd_get_cgcontrdp, 0U, ","),
	};

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, ARRAY_SIZE(data_cmd), at_cmd,
			     &mdata.sem_response, MDM_CMD_RSP_TIME);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
		ret = -1;
	}
	snprintk(rbuf, MAX_IP_RESP_SIZE, "IP: %s, GW: %s, NMASK: %s", mdata.mdm_ip, mdata.mdm_gw,
		 mdata.mdm_nmask);

	return ret;
}

/**
 * @brief Handler for CGCONTRDP
 *
 * @param argv[0] cid
 * @param argv[1] ipv4 addr
 * @param argv[2] ipv6 addr
 *
 * Sample response:
 *
 * AT at+CGPADDR
 * +CGPADDR:
 * 1,"33.28.8.237","38.7.251.144.95.233.90.246.90.237.97.39.90.237.97.39"
 */
MODEM_CMD_DEFINE(on_cmd_get_cgpaddr)
{
	if (argc < 3) {
		return -EAGAIN;
	}
	char *buf = argv[2];

	for (int i = 0; i < 16; i++) {
		if (*buf)
			mdata.mdm_ip6[i] = strtol(buf + 1, &buf, 10);
	}

	return 0;
}

/**
 * @brief Get ipv6 addr
 */
static int get_ip6(char *rbuf)
{
	int ret;
	static const char at_cmd[] = "AT+CGPADDR";
	struct modem_cmd data_cmd[] = {
		MODEM_CMD_ARGS_MAX("+CGPADDR:", on_cmd_get_cgpaddr, 0, 4U, ","),
	};

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, ARRAY_SIZE(data_cmd), at_cmd,
			     &mdata.sem_response, MDM_CMD_RSP_TIME);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
		ret = -1;
	}
	struct in6_addr addr;

	memcpy(addr.s6_addr, mdata.mdm_ip6, sizeof(mdata.mdm_ip6));
	char addr_buf[NET_IPV6_ADDR_LEN];

	net_addr_ntop(AF_INET6, &addr, addr_buf, sizeof(addr_buf));
	snprintk(rbuf, MAX_IP_RESP_SIZE, "IP6: %s", addr_buf);

	return ret;
}

/**
 * @brief Handler for modem firmware version
 */
MODEM_CMD_DEFINE(on_cmd_get_revision)
{
	size_t out_len = net_buf_linearize(mdata.mdm_revision, sizeof(mdata.mdm_revision) - 1,
					   data->rx_buf, 0, len);
	mdata.mdm_revision[out_len] = '\0';

	LOG_DBG("Revision: RK_%s", mdata.mdm_revision);
	return 0;
}

/**
 * @brief Get modem firmware version
 */
static int get_version(char *rbuf)
{
	int ret;
	static const char at_cmd[] = "AT+CGMR";

	struct modem_cmd data_cmd[] = {
		MODEM_CMD("RK_", on_cmd_get_revision, 0U, ""),
	};

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, ARRAY_SIZE(data_cmd), at_cmd,
			     &mdata.sem_response, MDM_CMD_RSP_TIME);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
		ret = -1;
	}
	memcpy(rbuf, "RK_", 3);
	memcpy(rbuf + 3, mdata.mdm_revision, sizeof(mdata.mdm_revision) - 3);
	return ret;
}

/**
 * @brief Handler for USIM info
 */
MODEM_CMD_DEFINE(on_cmd_get_usim)
{
	size_t out_len = net_buf_linearize(mdata.mdm_sim_info, sizeof(mdata.mdm_sim_info) - 1,
					   data->rx_buf, 0, len);
	mdata.mdm_sim_info[out_len] = '\0';

	LOG_DBG("USIM: %s", mdata.mdm_sim_info);
	return 0;
}

/**
 * @brief Get SIM info
 */
static int get_sim_info(char *rbuf)
{
	struct modem_cmd data_cmd[] = {
		MODEM_CMD("USIM:", on_cmd_get_usim, 0U, ""),
	};

	static const char at_cmd[] = "AT\%STATUS=\"USIM\"";
	int ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, ARRAY_SIZE(data_cmd),
				 at_cmd, &mdata.sem_response, MDM_CMD_RSP_TIME);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
		ret = -1;
	}
	memcpy(rbuf, mdata.mdm_sim_info, sizeof(mdata.mdm_sim_info));
	return ret;
}

/**
 * @brief Handler for "%PDNSET?"
 *
 * expected format (for IPv4):
 * %PDNSET: 1,CATM.T-MOBILE.COM,IP,,,,,0,0,0
 * for both IPv4 and IPv6:
 * %PDNSET: 1,CATM.T-MOBILE.COM,IPV4V6,,,,,0,0,0
 */
MODEM_CMD_DEFINE(on_cmd_pdnset)
{
	int ret = -1;

#define MAX_PDNSET_STR_SZ 128
	char resp_str[MAX_PDNSET_STR_SZ];
	size_t out_len = net_buf_linearize(resp_str, sizeof(resp_str) - 1, data->rx_buf, 0, len);
	resp_str[out_len] = '\0';

	LOG_DBG("PDNSET: %s", resp_str);

	char *p1 = strchr(resp_str, ',');

	if (p1) {
		char *p2 = strchr(++p1, ',');

		if (p2) {
			len = MIN(p2 - p1, MDM_APN_LENGTH - 1);
			memcpy(mdata.mdm_apn, p1, len);
			mdata.mdm_apn[len] = '\0';
			ret = 0;
		}
	}
	return ret;
}

/**
 * @brief Get APN
 */
static int get_apn(char *rbuf)
{
	struct modem_cmd data_cmd[] = {
		MODEM_CMD("%PDNSET:", on_cmd_pdnset, 0U, ""),
	};

	static const char at_cmd[] = "AT\%PDNSET?";
	int ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, ARRAY_SIZE(data_cmd),
				 at_cmd, &mdata.sem_response, MDM_CMD_RSP_TIME);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
		ret = -1;
	} else {
		memcpy(rbuf, mdata.mdm_apn, sizeof(mdata.mdm_apn));
	}
	return ret;
}

/**
 * @brief Check whether modem is awake
 */
static int murata_1sc_is_awake(char *rbuf)
{
	if (s_hifc_mode == 'A') {
		if (gpio_pin_get_dt(&wake_host_gpio) && gpio_pin_get_dt(&mdm_rx_gpio)) {
			strcpy(rbuf, TMO_MODEM_AWAKE_STR);
			return 1;
		} else if (!gpio_pin_get_dt(&wake_host_gpio) && !gpio_pin_get_dt(&mdm_rx_gpio)) {
			strcpy(rbuf, TMO_MODEM_ALSEEP_STR);
			return 0;
		}
		strcpy(rbuf, TMO_MODEM_UNKNOWN_STR);
		return -1;
	} else if (s_hifc_mode == 'B') {
		int ret = gpio_pin_get_dt(&wake_host_gpio);

		if (ret) {
			strcpy(rbuf, TMO_MODEM_AWAKE_STR);
		} else {
			strcpy(rbuf, TMO_MODEM_ALSEEP_STR);
		}
		return ret;
	}
	LOG_ERR("Unknown HIFC mode: %c", s_hifc_mode);
	return -1;
}

/**
 * @brief Handler for AT%SETCFG="SC_STATE","1"
 */
MODEM_CMD_DEFINE(on_cmd_sc_state)
{
	return 0;
}

/**
 * @brief check whether current FW image is golden
 */
static int is_golden(char *rbuf)
{
	struct modem_cmd data_cmd[] = {MODEM_CMD("ERROR", on_cmd_error, 0U, ""),
				       MODEM_CMD("%GETSYSCFG:", on_cmd_sc_state, 1U, "")};

	char at_cmd[] = "AT\%SETCFG=\"SC_STATE\",\"1\"";

	int ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, ARRAY_SIZE(data_cmd),
				 at_cmd, &mdata.sem_response, MDM_CMD_RSP_TIME);

	if (ret == -EIO) {
		strcpy(rbuf, "GOLDEN");
		return 1;
	} else if (ret >= 0) {
		strcpy(rbuf, "SAMPLE");
		return 0;
	}
	LOG_WRN("%s returned %d", __func__, ret);
	return ret;
}

const struct socket_dns_offload murata_dns_ops = {
	.getaddrinfo = murata_1sc_getaddrinfo,
	.freeaddrinfo = murata_1sc_freeaddrinfo,
};

int murata_socket_offload_init(void)
{
	socket_offload_dns_register(&murata_dns_ops);
	return 0;
}

static int ioctl_query(enum mdmdata_e idx, void *buf)
{
	int ret = 0;

	if (buf == NULL) {
		errno = EINVAL;
		return -1;
	}

	switch (idx) {
	case imei_e:
		strcpy(buf, mdata.mdm_imei);
		break;
#if defined(CONFIG_MODEM_SIM_NUMBERS)
	case imsi_e:
		strcpy(buf, mdata.mdm_imsi);
		break;
	case iccid_e:
		strcpy(buf, mdata.mdm_iccid);
		break;
#endif
	case ssi_e:
		ret = get_sigstrength(buf);
		break;

	case msisdn_e:
		ret = get_cnum(buf);
		break;

	case connsts_e:
		ret = get_carrier(buf);
		break;

	case ip_e:
		ret = get_ip(buf);
		break;

	case ip6_e:
		ret = get_ip6(buf);
		break;

	case version_e:
		ret = get_version(buf);
		break;

	case sim_info_e:
		ret = get_sim_info(buf);
		break;

	case apn_e:
		ret = get_apn(buf);
		break;

	case psm_e:
		ret = get_psm(buf);
		break;

	case edrx_e:
		ret = get_edrx(buf);
		break;

	case sleep_e:
		ret = set_cfun(0);
#if CONFIG_PM_DEVICE
		if (!ret)
			net_if_get_device(mdata.net_iface)->pm->state = PM_DEVICE_STATE_SUSPENDED;
#endif
		break;

	case wake_e:
		ret = set_cfun(1);
#if CONFIG_PM_DEVICE
		if (!ret)
			net_if_get_device(mdata.net_iface)->pm->state = PM_DEVICE_STATE_ACTIVE;
#endif
		break;

	case awake_e:
		ret = murata_1sc_is_awake(buf);
		break;

	case golden_e:
		ret = is_golden(buf);
		break;

	default:
		LOG_ERR("invalid request");
		ret = -1;
		break;
	}
	return ret;
}

typedef int (*mdmdata_cb_t)(enum mdmdata_e atcmd, void *user_data);

static int get_mdmdata_resp(char *io_str)
{
	int ret = -1;
	int idx = 0;
	const char *cmdStr;

	while (cmd_pool[idx].str != NULL) {
		cmdStr = cmd_pool[idx].str;
		if (strncmp(io_str, cmdStr, strlen(io_str)) == 0)
			break;
		++idx;
	}
	if (idx < (ARRAY_SIZE(cmd_pool) - 1)) {
		struct mdmdata_cmd_t cmd_entry = cmd_pool[idx];

		LOG_DBG("found cmd in pool, idx = %d\n", idx);
		ret = ioctl_query(cmd_entry.atcmd, io_str);
	} else {
		LOG_WRN("cmd (%s) not supported", io_str);
		idx = -1;
	}
	return ret;
}

struct aggr_ipv4_addr {
	struct in_addr ip;
	struct in_addr gw;
	struct in_addr nmask;
};

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
/**
 * do not change order
 */
enum {
	SSL_CERTIFICATE_TYPE = 0,
	SSL_PRIVATE_KEY_TYPE,
	SSL_CA_CERTIFICATE_TYPE,
};

/* send binary data via the AT commands */
static ssize_t send_cert(struct modem_cmd *handler_cmds, size_t handler_cmds_len, int cert_type,
			 char *filename)
{
	int ret = 0;
	int certfile_exist = -1; /* 0 means yes */
	char *certfile = NULL;
	char *keyfile = NULL;
	uint8_t *sptr;
	struct cert_cmd_t *cert_cmd_buf = (struct cert_cmd_t *)mdata.xlate_buf;

	memset(cert_cmd_buf->cert_cmd_write, 0, CERTCMD_WRITE_SIZE);
	char write_cmd[CERTCMD_WRITE_SIZE];

	/* TODO support other cert types as well */
	switch (cert_type) {
	case SSL_CERTIFICATE_TYPE:
	case SSL_CA_CERTIFICATE_TYPE:
		certfile = filename;
		break;
	case SSL_PRIVATE_KEY_TYPE:
		keyfile = filename;
		break;
	default:
		LOG_WRN("Bad cert_type %d", cert_type);
		goto exit;
	}
	certfile_exist = check_mdm_store_file(filename);

	if (certfile_exist != 0) {
		snprintk(write_cmd, sizeof(write_cmd), "AT%%CERTCMD=\"WRITE\",\"%s\",%d,\"",
			 filename, cert_type % 2);
		cert_cmd_buf->pem_buf[0] = '-'; /* amend the pem[0] overwritten by snprintk */
		sptr = (cert_cmd_buf->cert_cmd_write) + (CERTCMD_WRITE_SIZE - strlen(write_cmd));
		memcpy(sptr, write_cmd, strlen(write_cmd));
		LOG_DBG("sptr: %s", sptr);
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, sptr,
				     &mdata.sem_response, MDM_CMD_LONG_RSP_TIME);
		if (ret < 0) {
			if (ret == -116) {
				ret = 0;
			} else {
				goto exit;
			}
		}
	} else {
		return -EEXIST;
	}

exit:
	/* unset handler commands and ignore any errors */
	(void)modem_cmd_handler_update_cmds(&mdata.cmd_handler_data, NULL, 0U, false);
	return ret;
}

static int store_cert(struct murata_cert_params *params)
{
	struct tls_credential *cert = params->cert;
	char *filename = params->filename;
	int retval = 0;
	int cert_type;
	struct cert_cmd_t *cert_cmd_buf = (struct cert_cmd_t *)mdata.xlate_buf;

	/* For each tag, retrieve the credentials value and type: */
	int offset;
	char *header, *footer;

	if (cert != NULL) {
		switch (cert->type) {
		case TLS_CREDENTIAL_SERVER_CERTIFICATE:
			cert_type = SSL_CERTIFICATE_TYPE;
			header = "-----BEGIN CERTIFICATE-----\n";
			footer = "\n-----END CERTIFICATE-----\"\n";
			break;
		case TLS_CREDENTIAL_PRIVATE_KEY:
			cert_type = SSL_PRIVATE_KEY_TYPE;
			header = "-----BEGIN RSA PRIVATE KEY-----\n";
			footer = "\n-----END RSA PRIVATE KEY-----\"\n";
			break;
		case TLS_CREDENTIAL_CA_CERTIFICATE:
			cert_type = SSL_CA_CERTIFICATE_TYPE;
			header = "-----BEGIN CERTIFICATE-----\n";
			footer = "\n-----END CERTIFICATE-----\"\n";
			break;
		case TLS_CREDENTIAL_NONE:
		case TLS_CREDENTIAL_PSK:
		case TLS_CREDENTIAL_PSK_ID:
		default:
			retval = -EINVAL;
			goto exit;
		}

		k_sem_take(&mdata.sem_xlate_buf, K_FOREVER);
		strcpy(cert_cmd_buf->pem_buf, header);
		offset = strlen(header);
		size_t written;

		base64_encode(cert_cmd_buf->pem_buf + offset,
			      PEM_BUFF_SIZE - offset - strlen(footer), &written, cert->buf,
			      cert->len);
		memcpy(cert_cmd_buf->pem_buf + offset + written, footer, strlen(footer));
		cert_cmd_buf->pem_buf[offset + written + strlen(footer)] = 0; /* null terminate */

		LOG_DBG("offset= %d; written = %d\n", offset, written);
		{ /* write cert to murata with filename */
			retval = send_cert(NULL, 0, cert_type, filename);
			k_sem_give(&mdata.sem_xlate_buf);
			if (retval < 0) {
				LOG_ERR("Failed to send cert to modem, ret = "
					"%d",
					retval);
				return retval;
			}
		}
	}
exit:
	return retval;
}

static int del_cert(const char *filename)
{
	int ret;
	bool certfile_exist = check_mdm_store_file(filename) == 0;

	char at_cmd[sizeof("AT%%CERTCMD=\"DELETE\",\"\"") + MAX_FILENAME_LEN];

	if (certfile_exist) {
		snprintk(at_cmd, sizeof(at_cmd), "AT%%CERTCMD=\"DELETE\",\"%s\"", filename);
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, at_cmd,
				     &mdata.sem_response, MDM_CMD_LONG_RSP_TIME);
	} else {
		return -ENOENT;
	}
	return ret;
}

static int create_tls_profile(struct murata_tls_profile_params *params)
{
	uint8_t profile_id = params->profile_id_num;
	int ret;
	char fragment[MAX_FILENAME_LEN + 3];
	char at_cmd[sizeof("AT%CERTCFG=\"ADD\",XX,") + 6 * MAX_FILENAME_LEN + 19];

	snprintk(at_cmd, sizeof(at_cmd) - 1, "AT%%CERTCFG=\"ADD\",%d,", profile_id);
	if (params->ca_file) {
		snprintk(fragment, sizeof(fragment), "\"%s\"", params->ca_file);
		strcat(at_cmd, fragment);
	}
	strcat(at_cmd, ",");
	if (params->ca_path) {
		snprintk(fragment, sizeof(fragment), "\"%s\"", params->ca_path);
		strcat(at_cmd, fragment);
	}
	strcat(at_cmd, ",");
	if (params->dev_cert) {
		snprintk(fragment, sizeof(fragment), "\"%s\"", params->dev_cert);
		strcat(at_cmd, fragment);
	}
	strcat(at_cmd, ",");
	if (params->dev_key) {
		snprintk(fragment, sizeof(fragment), "\"%s\"", params->dev_key);
		strcat(at_cmd, fragment);
	}
	strcat(at_cmd, ",");
	if (params->psk_id) {
		snprintk(fragment, sizeof(fragment), "\"%s\"", params->psk_id);
		strcat(at_cmd, fragment);
	}
	strcat(at_cmd, ",");
	if (params->psk_key) {
		snprintk(fragment, sizeof(fragment), "\"%s\"", params->psk_key);
		strcat(at_cmd, fragment);
	}
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, at_cmd, &mdata.sem_response,
			     MDM_CMD_LONG_RSP_TIME);
	return ret;
}

static int delete_tls_profile(uint8_t profile)
{
	int ret;
	char at_cmd[sizeof("AT%CERTCFG=\"DELETE\",XXX")];

	snprintk(at_cmd, sizeof(at_cmd), "AT%%CERTCFG=\"DELETE\",%d", profile);
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, at_cmd, &mdata.sem_response,
			     MDM_CMD_LONG_RSP_TIME);
	return ret;
}

#endif

static int offload_setsockopt(void *obj, int level, int optname, const void *optval,
			      socklen_t optlen)
{
	int retval = -1;
	struct modem_socket *sock = (struct modem_socket *)obj;

#if !defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
	if (level == SOL_TLS) {
		errno = -ENOTSUP;
		return retval;
	}
#else
	int sd;

	if (level == SOL_TLS) {
		/* Handle Zephyr's SOL_TLS secure socket options: */
		switch (optname) {
		case TLS_SEC_TAG_LIST:
			/* Bind credential filenames to this socket: */
			/* TODO: Determine automatically if certs need to be
			 * loaded and create necessary profiles
			 */
			retval = 0;
			break;
		case TLS_PEER_VERIFY:
			sd = get_socket_idx(sock);
			murata_sock_tls_info[sd].peer_verify_disable = !*((int *)optval);
			return 0;
		case TLS_HOSTNAME:
			sd = get_socket_idx(sock);
			LOG_DBG("set SNI - name %s with len %d, for sock# %d", (char *)optval,
				optlen, sd);
			murata_sock_tls_info[sd].sni_valid = true;
			strncpy(murata_sock_tls_info[sd].host, optval,
				MIN(optlen, CONFIG_MURATA_MODEM_SNI_BUFFER_SZ));
			retval = 0;
			break;
		case TLS_CIPHERSUITE_LIST:
		case TLS_DTLS_ROLE:
			errno = ENOTSUP;
			return -1;
		case TLS_MURATA_USE_PROFILE:
			sd = get_socket_idx(sock);
			murata_sock_tls_info[sd].profile = *((int *)optval);
			return 0;
		case TLS_MURATA_CLIENT_VERIFY:
			sd = get_socket_idx(sock);
			murata_sock_tls_info[sd].client_verify = *((int *)optval);
			return 0;
		default:
			errno = EINVAL;
			return -1;
		}
	} else
#endif
	{
		switch (optname) {
		case SO_RCVTIMEO:
			if (!optval) {
				sock->rcvtimeo = K_FOREVER;
				retval = 0;
				break;
			} else if (optlen != sizeof(struct timeval)) {
				errno = EINVAL;
				return -1;
			}
			const struct timeval *tv;

			tv = optval;
			sock->rcvtimeo = K_TICKS(k_ms_to_ticks_ceil32(tv->tv_sec * 1000) +
						 k_us_to_ticks_ceil32(tv->tv_usec));
			retval = 0;
			break;
		default:
			errno = EINVAL;
			return -1;
		}
		return -1;
	}
	return retval;
}

/**
 * Direct FW update support functions (not via LwM2M)
 *
 * FW updates basically work like this:
 * 1. Get the FW file into the host device FLASH or memory
 * 2. Xfer the FW file to the modem
 * 2a. Xfer the header (first 256 bytes)
 * 2b. Xfer remaining chunks of the FW file
 * 3. Tell the modem to perform the update
 * 4. Reset the modem and wait for update to complete
 */

/**
 * @brief Initiate FW transfer from host to device
 *
 * @param file is the filename of the FW file
 *
 * @return OK or ERROR
 *
 * send_buf = 'AT%FILECMD="PUT","' + str(rfile) + '",1,' + str(len(csbuffer)) +
 * ',"' + str(cksum) + '"'
 */
static int init_fw_xfer(struct init_fw_data_t *ifd)
{
	char at_cmd[64];

	snprintk(at_cmd, sizeof(at_cmd), "AT%%FILECMD=\"PUT\",\"%s\",1, %u, \"%u\"", ifd->imagename,
		 (uint32_t)ifd->imagesize, (uint32_t)ifd->imagecrc);

	LOG_WRN("\tinit_fw_xfer: at cmd = %s", at_cmd);

	int ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, at_cmd,
				 &mdata.sem_response, MDM_CMD_LONG_RSP_TIME);

	LOG_DBG("\tinit_fw_xfer: ret = %d", ret);

	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
	}
	return ret;
}

MODEM_CMD_DEFINE(on_cmd_upgcmd)
{
	return ATOI(argv[1], 0, "diu_result");
}

#define FW_HEADER_SIZE 256

/**
 * @brief send first chunk of FW file data to the modem (256 bytes)
 *
 * @param data is ptr to chunk of binary data
 *
 * @return diu_result:
 *    0 - successfully finished software upgrade step (image pre-check, update,
 * etc.) 1 - general upgrade errors 2 - failed to the pre-checking of delta
 * image 3 - image validation failure 4 - failed to update 5 - delta update
 * Agent was not found 6 - no upgrade result is found
 *
 * send_buf = 'AT%UPGCMD="CFGPART","' + interim_map_str + '"'
 */
static int send_fw_header(const char *data)
{
	struct modem_cmd data_cmd[] = {MODEM_CMD("ERROR", on_cmd_error, 0U, ""),
				       MODEM_CMD("%UPGCMD:", on_cmd_upgcmd, 1U, "")};

	k_sem_take(&mdata.sem_xlate_buf, K_FOREVER);

	/* Create the command prefix */
	int i = snprintk(mdata.xlate_buf, sizeof(mdata.xlate_buf), "AT%%UPGCMD=\"CFGPART\",\"");

	/* Add the hex string */
	data_to_hex_str(data, FW_HEADER_SIZE, &mdata.xlate_buf[i], sizeof(mdata.xlate_buf) - i);

	/* Finish the command */
	snprintk(&mdata.xlate_buf[i + FW_HEADER_SIZE * 2], sizeof(mdata.xlate_buf), "\"");

	LOG_DBG("Header => %s\n", (char *)mdata.xlate_buf);

	/* Send the command */
	int ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, ARRAY_SIZE(data_cmd),
				 mdata.xlate_buf, &mdata.sem_response, MDM_CMD_RSP_TIME);

	k_sem_give(&mdata.sem_xlate_buf);

	return ret;
}

MODEM_CMD_DEFINE(on_cmd_filedata)
{
	return ATOI(argv[1], 0, "written") / 2;
}

/**
 * @brief Send a chunk of FW file data to the modem
 *
 * @param data is ptr to raw data
 * @param more is 0 if this is the last chunk, 1 otherwise
 * @param len is len of raw data, must be <= MDM_MAX_DATA_LENGTH (1500)
 *
 * @return bytes written, or ERROR
 *
 * send_buf = 'AT%FILEDATA="WRITE",0' + ',' + str(display_sz) + ',"' +
 * (out_hexstr) + '"'
 */
static int send_fw_data(const struct send_fw_data_t *sfd)
{
	struct modem_cmd data_cmd[] = {MODEM_CMD("ERROR", on_cmd_error, 0U, ""),
				       MODEM_CMD("%FILEDATA:", on_cmd_filedata, 1U, "")};

	if (sfd->len <= 0 || sfd->len > MDM_MAX_DATA_LENGTH) {
		return -1;
	}

	k_sem_take(&mdata.sem_xlate_buf, K_FOREVER);

	/* Create the command prefix */
	int i = snprintk(mdata.xlate_buf, sizeof(mdata.xlate_buf),
			 "AT%%FILEDATA=\"WRITE\",%d,%d,\"", sfd->more, sfd->len * 2);

	/* Add the hex string */
	data_to_hex_str(sfd->data, sfd->len, &mdata.xlate_buf[i], sizeof(mdata.xlate_buf) - i);

	/* Finish the command */
	snprintk(&mdata.xlate_buf[i + sfd->len * 2], sizeof(mdata.xlate_buf), "\"");

	LOG_DBG("Cmd %s\n", (char *)mdata.xlate_buf);
	if (sfd->more == 0) {
		LOG_DBG("Done Cmd %s\n", (char *)mdata.xlate_buf);
	} else {
		LOG_DBG("Cmd %s\n", (char *)mdata.xlate_buf);
	}

	/* Send the command */
	int ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, ARRAY_SIZE(data_cmd),
				 mdata.xlate_buf, &mdata.sem_response, MDM_CMD_LONG_RSP_TIME);

	k_sem_give(&mdata.sem_xlate_buf);

	if (sfd->more == 0) {
		LOG_DBG("Done Cmd results %d\n", ret);
	} else {
		LOG_DBG("Cmd results %d\n", ret);
	}

	return ret;
}

/**
 * @brief Initiate FW upgrade after FW file has been xfer'ed to modem
 *
 * @param file is the filename of the FW file to use for upgrading
 *
 * @return diu_result:
 *    0 - successfully finished software upgrade step (image pre-check, update,
 * etc.) 1 - general upgrade errors 2 - failed to the pre-checking of delta
 * image 3 - image validation failure 4 - failed to update 5 - delta update
 * Agent was not found 6 - no upgrade result is found
 *
 * send_buf = 'AT%UPGCMD="UPGVRM","' + lfile + '"'
 */
static int init_fw_upgrade(const char *file)
{
	struct modem_cmd data_cmd[] = {MODEM_CMD("ERROR", on_cmd_error, 0U, ""),
				       MODEM_CMD("%UPGCMD:", on_cmd_upgcmd, 1U, "")};

	char at_cmd[64];

	snprintk(at_cmd, sizeof(at_cmd), "AT%%UPGCMD=\"UPGVRM\",\"%s\"", file);

	LOG_DBG("%s: at cmd = %s", __func__, at_cmd);

	int ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, ARRAY_SIZE(data_cmd),
				 at_cmd, &mdata.sem_response, MDM_CMD_RSP_TIME);

	LOG_DBG("Ret %d", ret);

	if (ret < 0) {
		LOG_ERR("%s ret: %d", at_cmd, ret);
	}
	return ret;
}

static char chksum[CHKSUM_ABILITY_MAX_LEN];

MODEM_CMD_DEFINE(on_cmd_chksum)
{
	size_t out_len = net_buf_linearize(chksum, sizeof(chksum) - 1, data->rx_buf, 0, len);
	chksum[out_len] = '\0';
	return 0;
}

/**
 * @brief check whether file checksum is supported
 *
 * @param response is the response received from the request
 * @return OK or ERROR
 *
 * send_buf = 'AT%GETACFG=filemgr.file.put_fcksum'
 */
static int get_file_chksum_ability(char *response)
{
	struct modem_cmd data_cmd[] = {MODEM_CMD("ERROR", on_cmd_error, 0U, ""),
				       MODEM_CMD("", on_cmd_chksum, 1U, "")};

	chksum[0] = '\0';
	int ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, ARRAY_SIZE(data_cmd),
				 "AT\%GETACFG=filemgr.file.put_fcksum", &mdata.sem_response,
				 MDM_CMD_RSP_TIME);

	if (ret < 0) {
		LOG_ERR("%s ret: %d", "AT\%GETACFG=filemgr.file.put_fcksum", ret);
	} else {
		snprintk(response, CHKSUM_ABILITY_MAX_LEN, "%s", chksum);
	}
	return ret;
}

static char file_cmd_full_access[CMD_FULL_ACCESS_MAX_LEN];

MODEM_CMD_DEFINE(on_cmd_file_cmd_full_access)
{
	size_t out_len = net_buf_linearize(file_cmd_full_access, sizeof(file_cmd_full_access) - 1,
					   data->rx_buf, 0, len);
	file_cmd_full_access[out_len] = '\0';
	return 0;
}

/**
 * @brief check setting of admin.services.file_cmd_full_access
 *
 * @param response is the response received from the request
 * @return OK or ERROR
 *
 * send_buf = 'AT%GETACFG=admin.services.file_cmd_full_access'
 */
static int get_file_mode(char *response)
{
	struct modem_cmd data_cmd[] = {MODEM_CMD("ERROR", on_cmd_error, 0U, ""),
				       MODEM_CMD("", on_cmd_file_cmd_full_access, 1U, "")};

	char at_cmd[] = "AT\%GETACFG=admin.services.file_cmd_full_access";

	file_cmd_full_access[0] = '\0';
	int ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, ARRAY_SIZE(data_cmd),
				 at_cmd, &mdata.sem_response, MDM_CMD_RSP_TIME);

	if (ret < 0) {
		LOG_ERR("%s ret: %d", at_cmd, ret);
	} else {
		snprintk(response, CMD_FULL_ACCESS_MAX_LEN, "%s", file_cmd_full_access);
	}
	return ret;
}

/**
 * @brief Initial setup of the modem
 */
static int murata_1sc_setup(void)
{
	gpio_pin_set_dt(&reset_gpio, 1);
	k_msleep(20);
	gpio_pin_set_dt(&reset_gpio, 0);

	gpio_pin_set_dt(&wake_mdm_gpio, 1);
	LOG_INF("Waiting %d secs for modem to boot...", MDM_BOOT_DELAY);
	k_sleep(K_SECONDS(MDM_BOOT_DELAY));

	static const struct setup_cmd setup_cmds[] = {
	  SETUP_CMD_NOHANDLE("ATQ0"),
	  SETUP_CMD_NOHANDLE("ATE0"),
	  SETUP_CMD_NOHANDLE("ATV1"),
	  SETUP_CMD_NOHANDLE("AT%CSDH=1"),
	  SETUP_CMD_NOHANDLE("AT+CNMI=2,1,0,1,0"),
	  SETUP_CMD_NOHANDLE("AT+CMGF=0"),
	  SETUP_CMD("AT+CGMI", "", on_cmd_get_manufacturer, 0U, ""),
	  SETUP_CMD("AT+CGMM", "", on_cmd_get_model, 0U, ""),
	  SETUP_CMD("AT+CGMR", "RK_", on_cmd_get_revision, 0U, ""),
	  SETUP_CMD("AT+CGSN", "", on_cmd_get_imei, 0U, ""),
	  SETUP_CMD("AT%GETACFG=modem_apps.Mode.AutoConnectMode", "", on_cmd_get_acfg, 0U, ""),
	  SETUP_CMD("AT%GETACFG=service.sockserv.maxsock", "", on_cmd_get_sockcount, 0U, ""),
	  SETUP_CMD("AT%GETCFG=\"BAND\"", "Bands:", on_cmd_get_bands, 0U, ""),
	  SETUP_CMD("AT%GETACFG=pm.conf.max_allowed_pm_mode", "", on_cmd_get_max_pm_mode, 0U, ""),
	  SETUP_CMD("AT%GETACFG=pm.conf.sleep_mode", "", on_cmd_get_sleep_mode, 0U, ""),
	  SETUP_CMD("AT%GETACFG=pm.hifc.mode", "", on_cmd_get_hifc_mode, 0U, ""),
	  SETUP_CMD("AT%GETACFG=modem_apps.Mode.AtCmdSetPersistence", "", on_cmd_get_at_persist,
			0U, ""),
#ifdef CONFIG_NET_IPV4
	  SETUP_CMD("AT%GETACFG=APNTable.Class1.IPv4DnsIP_1", "", on_cmd_get_ipv4_primary, 0U, ""),
	  SETUP_CMD("AT%GETACFG=APNTable.Class1.IPv4DnsIP_2", "", on_cmd_get_ipv4_secondary, 0U,
		    ""),
#endif /* CONFIG_NET_IPV4 */
#ifdef CONFIG_NET_IPV6
	  SETUP_CMD("AT%GETACFG=APNTable.Class1.IPv6DnsIP_1", "", on_cmd_get_ipv6_primary, 0U, ""),
	  SETUP_CMD("AT%GETACFG=APNTable.Class1.IPv6DnsIP_2", "", on_cmd_get_ipv6_secondary, 0U,
		    ""),
#endif /* CONFIG_NET_IPV6 */
#if defined(CONFIG_MODEM_SIM_NUMBERS)
	  SETUP_CMD("AT+CIMI", "", on_cmd_get_imsi, 0U, ""),
	  SETUP_CMD("AT%CCID", "%CCID:", on_cmd_get_iccid, 0U, " "),
#endif
#ifdef VERIFY_INIT_MODEM_STATE
	  SETUP_CMD("AT%STATUS=\"USIM\"", "USIM:", on_cmd_get_usim, 0U, ""),
	  SETUP_CMD("AT+CFUN?", "+CFUN:", on_cmd_get_cfun, 0U, ""),
	  SETUP_CMD("AT+CEREG?", "+CEREG:", on_cmd_get_cereg, 0U, ""),
	  SETUP_CMD("AT+CGCONTRDP", "+CGCONTRDP:", on_cmd_get_cgcontrdp, 0U, ""),
#endif
#if defined(CONFIG_MODEM_DEMO_LOW_POWERMODE)
	  SETUP_CMD_NOHANDLE("AT+CFUN=0"),
#endif
	};

top:;
	/* Run setup commands on the modem. */
	int ret = modem_cmd_handler_setup_cmds(&mctx.iface, &mctx.cmd_handler, setup_cmds,
					       ARRAY_SIZE(setup_cmds), &mdata.sem_response,
					       MDM_REGISTRATION_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("modem_cmd_handler_setup_cmds error");
	}

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
	ret = post_mdm_init();
	if (ret < 0) {
		LOG_ERR("post modem_cmd_init error");
	}
#endif
	set_pdn_params();
	set_bands();
	set_boot_delay();

	bool needto_reset_modem = false;

	if (needto_set_autoconn_to_true) {
		set_autoconn_on();
		needto_reset_modem = true;
	}
	if (needto_set_sockcount) {
		set_socket_count();
		needto_reset_modem = true;
	}
	if (needto_set_max_pm_mode) {
		set_max_pm_mode(s_max_pm_mode);
		needto_reset_modem = true;
	}
	if (needto_set_sleep_mode) {
		set_sleep_mode(s_sleep_mode);
		needto_reset_modem = true;
	}
	if (needto_set_hifc_mode) {
		set_hifc_mode(s_hifc_mode);
		needto_reset_modem = true;
	}

	if (needto_set_dns_servers) {
		ret = set_dns_servers();
		needto_reset_modem = true;
	}

	if (needto_enable_at_persist) {
		ret = set_at_persist_mode();
		needto_reset_modem = true;
	}

	if (needto_reset_modem) {
		reset_modem();
		goto top;
	}

#if defined(CONFIG_MODEM_DEMO_LOW_POWERMODE)
	k_sleep(K_MSEC(CONFIG_MODEM_ON_DELAY));
	set_cfun(0);
#endif

	return ret;
}

/**
 * @brief Most of the ioctls calls are passed a single int va_arg, but need a pointer.
 * It is the callers responsibility to safely pass the pointer value represented as a
 * positive integer, this function will decode it back into a pointer.
 *
 * It is the callees job to ensure const correctness as this interface has no way to
 * know if the pointer described was originally a const.
 *
 * @param args The va_args argument
 * @return void* The value as a pointer, NULL if the value cannot be represented
 */
static void *ptr_from_va(va_list args)
{
	int arg = va_arg(args, int);
	uintptr_t uiptr;

	if (arg < 0) {
		return NULL;
	}
	uiptr = arg;
	return (void *)uiptr;
}

/**
 * @brief ioctl handler to handle various requests
 *
 * @param obj ptr to socket
 * @param request type
 * @param args parameter
 */
static int offload_ioctl(void *obj, unsigned int request, va_list args)
{
	int ret;
	struct aggr_ipv4_addr *a_ipv4_addr;
	char *cmd_str;

	/* TBD: cast obj to socket, find the right instance of the
	 * murata_1sc_data etc assuming one instance for now
	 */

	switch (request) {
	case F_GETFL:
		return 0; /* Always report that we're a blocking socket */

	/* Note: Poll functions are passed their arguments from a different function that properly
	 * utilizes va_args instead of assuming int
	 */
	case ZFD_IOCTL_POLL_PREPARE: {
		struct zsock_pollfd *pfd;
		struct k_poll_event **pev;
		struct k_poll_event *pev_end;

		pfd = va_arg(args, struct zsock_pollfd *);
		pev = va_arg(args, struct k_poll_event **);
		pev_end = va_arg(args, struct k_poll_event *);

		return modem_socket_poll_prepare(&mdata.socket_config, obj, pfd, pev, pev_end);
	}

	case ZFD_IOCTL_POLL_UPDATE: {
		struct zsock_pollfd *pfd;
		struct k_poll_event **pev;

		pfd = va_arg(args, struct zsock_pollfd *);
		pev = va_arg(args, struct k_poll_event **);

		return modem_socket_poll_update(obj, pfd, pev);
	}

#if defined(CONFIG_MODEM_SMS)
	case SMS_SEND:
		ret = send_sms_msg((struct sms_out *)ptr_from_va(args));
		va_end(args);
		break;

	case SMS_RECV:
		ret = recv_sms_msg((struct sms_in *)ptr_from_va(args));
		va_end(args);
		break;
#endif /* CONFIG_MODEM_SMS */

	case GET_IPV4_CONF:
		a_ipv4_addr = (struct aggr_ipv4_addr *)ptr_from_va(args);
		va_end(args);
		get_ipv4_config();
		zsock_inet_pton(AF_INET, mdata.mdm_ip, &a_ipv4_addr->ip);
		zsock_inet_pton(AF_INET, mdata.mdm_gw, &a_ipv4_addr->gw);
		zsock_inet_pton(AF_INET, mdata.mdm_nmask, &a_ipv4_addr->nmask);
		ret = 0;
		break;

	case GET_ATCMD_RESP:
		cmd_str = (char *)ptr_from_va(args);
		va_end(args);
		ret = get_mdmdata_resp(cmd_str);
		break;

	case INIT_FW_XFER:
		ret = init_fw_xfer((struct init_fw_data_t *)ptr_from_va(args));
		va_end(args);
		break;

	case SEND_FW_HEADER:
		ret = send_fw_header((const char *)ptr_from_va(args));
		va_end(args);
		break;

	case SEND_FW_DATA:
		ret = send_fw_data((struct send_fw_data_t *)ptr_from_va(args));
		va_end(args);
		break;

	case INIT_FW_UPGRADE:
		ret = init_fw_upgrade((const char *)ptr_from_va(args));
		va_end(args);
		break;

	case GET_CHKSUM_ABILITY:
		ret = get_file_chksum_ability((char *)ptr_from_va(args));
		va_end(args);
		break;

	case GET_FILE_MODE:
		ret = get_file_mode((char *)ptr_from_va(args));
		va_end(args);
		break;

	case RESET_MODEM:
		ret = reset_modem();
		break;

	case AT_MODEM_PSM_SET:
		ret = set_psm_timer((struct set_cpsms_params *)ptr_from_va(args));
		va_end(args);
		break;

	case AT_MODEM_EDRX_SET:
		ret = set_edrx_timer((struct set_cedrxs_params *)ptr_from_va(args));
		va_end(args);
		break;

	case AT_MODEM_EDRX_GET:
		ret = get_edrx((char *)ptr_from_va(args));
		va_end(args);
		break;

	case AT_MODEM_EDRX_PTW_SET:
		ret = set_edrx_ptw((int *)ptr_from_va(args));
		va_end(args);
		break;

	case AT_MODEM_EDRX_PTW_GET:
		ret = get_edrx_ptw((int *)ptr_from_va(args));
		va_end(args);
		break;

	case AT_MODEM_PSM_GET:
		ret = get_psm((char *)ptr_from_va(args));
		va_end(args);
		break;
#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
	case STORE_CERT:
		ret = store_cert((struct murata_cert_params *)ptr_from_va(args));
		va_end(args);
		break;

	case DEL_CERT:
		ret = del_cert((const char *)ptr_from_va(args));
		va_end(args);
		break;

	case CHECK_CERT:
		ret = check_mdm_store_file((const char *)ptr_from_va(args));
		va_end(args);
		break;

	case DELETE_CERT_PROFILE:
		ret = delete_tls_profile(va_arg(args, int));
		va_end(args);
		break;

	case CREATE_CERT_PROFILE:
		ret = create_tls_profile((struct murata_tls_profile_params *)ptr_from_va(args));
		va_end(args);
		break;
#endif
	default:
		errno = EINVAL;
		ret = -1;
		break;
	}
	return ret;
}

static const struct socket_op_vtable offload_socket_fd_op_vtable = {
	.fd_vtable = {
			.read = offload_read,
			.write = offload_write,
			.close = offload_close,
			.ioctl = offload_ioctl,
		},
	.bind = offload_bind,
	.connect = offload_connect,
	.sendto = offload_sendto,
	.recvfrom = offload_recvfrom,
	.listen = NULL,
	.accept = NULL,
	.sendmsg = offload_sendmsg,
	.getsockopt = NULL,
	.setsockopt = offload_setsockopt,
};

static struct gpio_callback mdm_wake_host_cb_data;
K_SEM_DEFINE(mdm_transition_sem, 1, 1);

/**
 * @brief Callback to handle wakeup request from the modem
 *
 * @param port GPIO Port
 * @param cb Pointer to callback data structure
 * @param pins GPIO Pins
 */
void mdm_wake_host_cb(const struct device *port, struct gpio_callback *cb,
		    uint32_t pins)
{
	int val = gpio_pin_get_dt(&wake_host_gpio);

	if (val <= 0) {
		return;
	} else if (val > 0) {
		hifc_handshake_state = isr_init_resume;
	}

	hifc_handshake_tmr.user_data = (void *)&s_hifc_mode;
	k_timer_start(&hifc_handshake_tmr, K_NO_WAIT, K_MSEC(100));
}

/*
 * @brief Initialize the driver
 */
static int murata_1sc_init(const struct device *dev)
{
	int ret = 0;

	k_sem_init(&mdata.sem_response, 0, 1);
	k_sem_init(&mdata.sem_sock_conn, 0, 1);
	k_sem_init(&mdata.sem_xlate_buf, 1, 1);
	k_sem_init(&mdata.sem_rcv_sms, 0, 1);
	k_sem_init(&mdata.sem_sms, 1, 1);

	/* socket config */
	ret = modem_socket_init(&mdata.socket_config, &mdata.sockets[0], ARRAY_SIZE(mdata.sockets),
				MDM_BASE_SOCKET_NUM, false, &offload_socket_fd_op_vtable);
	if (ret < 0) {
		goto error;
	}

	/* cmd handler setup */
	const struct modem_cmd_handler_config cmd_handler_config = {
		.match_buf = &mdata.cmd_match_buf[0],
		.match_buf_len = sizeof(mdata.cmd_match_buf),
		.buf_pool = &mdm_recv_pool,
		.alloc_timeout = BUF_ALLOC_TIMEOUT,
		.eol = "\r\n",
		.user_data = NULL,
		.response_cmds = response_cmds,
		.response_cmds_len = ARRAY_SIZE(response_cmds),
		.unsol_cmds = unsol_cmds,
		.unsol_cmds_len = ARRAY_SIZE(unsol_cmds),
	};

	ret = modem_cmd_handler_init(&mctx.cmd_handler, &mdata.cmd_handler_data,
				     &cmd_handler_config);

	/* modem interface */
	const struct modem_iface_uart_config uart_config = {
		.rx_rb_buf = &mdata.iface_rb_buf[0],
		.rx_rb_buf_len = sizeof(mdata.iface_rb_buf),
		.dev = MDM_UART_DEV,
		.hw_flow_control = DT_PROP(MDM_UART_NODE, hw_flow_control),
	};
	ret = modem_iface_uart_init(&mctx.iface, &mdata.iface_data, &uart_config);
	if (ret < 0) {
		goto error;
	}

	/* modem data storage */
	mctx.data_manufacturer = mdata.mdm_manufacturer;
	mctx.data_model = mdata.mdm_model;
	mctx.data_revision = mdata.mdm_revision;
	mctx.data_imei = mdata.mdm_imei;

#if defined(CONFIG_MODEM_SMS)
	/* SMS functions */
	mctx.send_sms = send_sms_msg;
	mctx.recv_sms = recv_sms_msg;
#if defined(CONFIG_MODEM_SMS_CALLBACK)
	mctx.recv_sms_cb_en = recv_sms_cb_en;
#endif /* CONFIG_MODEM_SMS_CALLBACK */
#endif /* CONFIG_MODEM_SMS */

	mctx.driver_data = &mdata;
	mctx.dev = dev;

	/* pin setup */
	ret = gpio_pin_configure_dt(&wake_host_gpio, GPIO_INPUT | GPIO_PULL_DOWN);
	if (ret < 0) {
		LOG_ERR("Failed to configure %s pin", "wake_host");
		goto error;
	}

	ret = gpio_pin_interrupt_configure_dt(&wake_host_gpio, GPIO_INT_EDGE_RISING);
	if (ret < 0) {
		LOG_ERR("Failed to configure %s pin", "wake_host");
		goto error;
	}

	gpio_init_callback(&mdm_wake_host_cb_data, mdm_wake_host_cb, BIT(wake_host_gpio.pin));
	gpio_add_callback(wake_host_gpio.port, &mdm_wake_host_cb_data);

	ret = gpio_pin_configure_dt(&wake_mdm_gpio, GPIO_OUTPUT | GPIO_PULL_DOWN);
	if (ret < 0) {
		LOG_ERR("Failed to configure %s pin", "wake_mdm");
		goto error;
	}

	ret = gpio_pin_configure_dt(&reset_gpio, GPIO_OUTPUT_LOW | GPIO_PULL_DOWN);
	if (ret < 0) {
		LOG_ERR("Failed to configure %s pin", "reset");
		goto error;
	}

	ret = gpio_pin_configure_dt(&rst_done_gpio, GPIO_INPUT | GPIO_PULL_DOWN);
	if (ret < 0) {
		LOG_ERR("Failed to configure %s pin", "reset done");
		goto error;
	}

	ret = gpio_pin_configure_dt(&mdm_rx_gpio, GPIO_INPUT | GPIO_PULL_DOWN);
	if (ret < 0) {
		LOG_ERR("Failed to configure %s pin", "mdm_rx");
		goto error;
	}

	ret = gpio_pin_configure_dt(&mdm_tx_gpio, GPIO_OUTPUT_LOW | GPIO_PULL_DOWN);
	if (ret < 0) {
		LOG_ERR("Failed to configure %s pin", "mdm_tx");
		goto error;
	}

	ret = gpio_pin_configure_dt(&mdm_rts_gpio, GPIO_OUTPUT | GPIO_PULL_DOWN);
	if (ret < 0) {
		LOG_ERR("Failed to configure %s pin", "mdm_rts");
		goto error;
	}

	ret = gpio_pin_configure_dt(&mdm_cts_gpio, GPIO_INPUT | GPIO_PULL_DOWN);
	if (ret < 0) {
		LOG_ERR("Failed to configure %s pin", "mdm_cts");
		goto error;
	}

	ret = modem_context_register(&mctx);
	if (ret < 0) {
		LOG_ERR("Error registering modem context: %d", ret);
		goto error;
	}

	/* start RX thread */
	k_thread_create(&modem_rx_thread, modem_rx_stack, K_KERNEL_STACK_SIZEOF(modem_rx_stack),
			(k_thread_entry_t)murata_1sc_rx, NULL, NULL, NULL, K_PRIO_COOP(7), 0,
			K_NO_WAIT);

	murata_1sc_setup();

error:
	return 0;
}

#if defined(CONFIG_NET_OFFLOAD)
static int net_offload_dummy_get(sa_family_t family, enum net_sock_type type,
				 enum net_ip_protocol ip_proto, struct net_context **context)
{

	LOG_ERR("CONFIG_NET_SOCKETS_OFFLOAD must be enabled for this driver");
	return -ENOTSUP;
}

/* placeholders, until Zephyr IP stack updated to handle a NULL net_offload */
static struct net_offload modem_net_offload = {
	.get = net_offload_dummy_get,
};
#endif

#if CONFIG_PING
#include <zephyr/net/ping.h>
static uint32_t sent_ts;

/**
 * @brief Handler for ping reply events
 */
MODEM_CMD_DEFINE(on_cmd_pingcmd)
{
	net_ping_resp_notify(k_uptime_get_32() - sent_ts);
	return 0;
}

static int offload_ping(const struct sockaddr *dst, size_t sz)
{
	/* Even though there is a size argument, it doesn't work correctly */
	ARG_UNUSED(sz);
	char at_cmd[sizeof("AT%PINGCMD=0,\"\"") + NET_IPV6_ADDR_LEN];
	char addr[NET_IPV6_ADDR_LEN];

	struct modem_cmd data_cmd[] = {MODEM_CMD("ERROR", on_cmd_error, 0U, ""),
				       MODEM_CMD("%PINGCMD:", on_cmd_pingcmd, 4U, ",")};

	int ipv6 = (dst->sa_family == AF_INET6) ? 1 : 0;

	if (ipv6) {
		net_addr_ntop(AF_INET6, &net_sin6(dst)->sin6_addr, addr, sizeof(addr));
	} else {
		net_addr_ntop(AF_INET, &net_sin(dst)->sin_addr, addr, sizeof(addr));
	}

	snprintk(at_cmd, sizeof(at_cmd), "AT%%PINGCMD=%d,\"%s\"", ipv6, addr);

	sent_ts = k_uptime_get_32();
	int ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, ARRAY_SIZE(data_cmd),
				 at_cmd, &mdata.sem_response, MDM_CMD_LONG_RSP_TIME);

	if (ret < 0) {
		LOG_ERR("%s ret:%d", at_cmd, ret);
	}
	return ret;
}
#endif

/* @brief Setup the Modem NET Interface. */
static void murata_1sc_net_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct murata_1sc_data *data = dev->data;

	/* Direct socket offload used instead of net offload: */
	net_if_set_link_addr(iface, murata_1sc_get_mac(dev), sizeof(data->mac_addr),
			     NET_LINK_ETHERNET);
	data->net_iface = iface;
#if defined(CONFIG_NET_OFFLOAD)
	iface->if_dev->offload = &modem_net_offload;
#endif
#if defined(CONFIG_NET_SOCKETS_OFFLOAD)
	iface->if_dev->socket_offload = offload_socket;
#if defined(CONFIG_PING)
	iface->if_dev->ping_offload = offload_ping;
#endif
	murata_socket_offload_init();
#endif
}

static struct offloaded_if_api api_funcs = {
	.iface_api.init = murata_1sc_net_iface_init,
};

/**
 * @brief Used during registration to indicate that offload is supported
 */
static bool offload_is_supported(int family, int type, int proto)
{
	return true;
}

#if defined(CONFIG_PM_DEVICE)
static int murata_1sc_pm_action(const struct device *dev, enum pm_device_action action)
{
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		return set_cfun(1);
	/* This technically isn't correct behavior, off should be a different
	 * state, however...
	 */
	case PM_DEVICE_ACTION_SUSPEND:
		return set_cfun(0);
	default:
		return -ENOTSUP;
	}

	return ret;
}
PM_DEVICE_DT_INST_DEFINE(0, murata_1sc_pm_action);
#endif /* defined(CONFIG_PM_DEVICE) */

/**
 * @brief Register the device with the Networking stack
 */
NET_DEVICE_DT_INST_OFFLOAD_DEFINE(0, murata_1sc_init, PM_DEVICE_DT_INST_GET(0), &mdata, NULL, 80,
				  &api_funcs, MDM_MAX_DATA_LENGTH);

/* Register NET sockets. */
NET_SOCKET_REGISTER(murata_1sc, NET_SOCKET_DEFAULT_PRIO, AF_INET, offload_is_supported,
		    offload_socket);
