/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * All references to the spec refer to IEEE 802.15.4-2020.
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_ieee802154_aloha, CONFIG_NET_L2_IEEE802154_LOG_LEVEL);

#include <zephyr/net/net_if.h>

#include <errno.h>

#include "ieee802154_frame.h"
#include "ieee802154_utils.h"
#include "ieee802154_radio_utils.h"

static inline int aloha_radio_send(struct net_if *iface,
				   struct net_pkt *pkt,
				   struct net_buf *frag)
{
	/* See section 6.7.4.4 - Retransmissions. */
	uint8_t remaining_attempts = CONFIG_NET_L2_IEEE802154_RADIO_TX_RETRIES + 1;
	struct ieee802154_context *ctx = net_if_l2_data(iface);
	bool ack_required;
	int ret = -EIO;

	NET_DBG("frag %p", frag);

	while (remaining_attempts) {
		remaining_attempts--;

		ack_required = prepare_for_ack(ctx, pkt, frag);

		if (!ack_required) {
			/* See section 6.7.4.4: "A device that sends a frame with the AR field set
			 * to indicate no acknowledgment requested may assume that the transmission
			 * was successfully received and shall not perform the retransmission
			 * procedure."
			 */
			remaining_attempts = 0;
		}

static inline int aloha_channel_access(struct net_if *iface)
{
	ARG_UNUSED(iface);

	/* CCA Mode 4: ALOHA. CCA shall always report an idle medium, see section 10.2.8. */
	return 0;
}

/* Declare the public channel access algorithm function used by L2. */
FUNC_ALIAS(aloha_channel_access, ieee802154_wait_for_clear_channel, int);
