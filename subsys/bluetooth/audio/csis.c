/*  Bluetooth CSIS - Coordinated Set Identification Service */

/*
 * Copyright (c) 2019 Bose Corporation
 * Copyright (c) 2020-2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/types.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <stdlib.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/buf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/check.h>

#include "audio_internal.h"
#include "csis_internal.h"
#include "csis_crypto.h"
#include "../host/conn_internal.h"
#include "../host/hci_core.h"
#include "../host/keys.h"

#define BT_CSIS_SIH_PRAND_SIZE          3
#define BT_CSIS_SIH_HASH_SIZE           3
#define CSIS_SET_LOCK_TIMER_VALUE       K_SECONDS(60)

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_DEBUG_CSIS)
#define LOG_MODULE_NAME bt_csis
#include "common/log.h"

static struct bt_csis csis_insts[CONFIG_BT_CSIS_MAX_INSTANCE_COUNT];
static bt_addr_le_t server_dummy_addr; /* 0'ed address */

struct csis_notify_foreach {
	struct bt_conn *excluded_client;
	struct bt_csis *csis;
};

static bool is_last_client_to_write(const struct bt_csis *csis,
				    const struct bt_conn *conn)
{
	if (conn != NULL) {
		return !bt_addr_le_cmp(bt_conn_get_dst(conn),
				       &csis->srv.lock_client_addr);
	} else {
		return !bt_addr_le_cmp(&server_dummy_addr,
				       &csis->srv.lock_client_addr);
	}
}

static void notify_lock_value(const struct bt_csis *csis, struct bt_conn *conn)
{
	bt_gatt_notify_uuid(conn, BT_UUID_CSIS_SET_LOCK,
			    csis->srv.service_p->attrs,
			    &csis->srv.set_lock,
			    sizeof(csis->srv.set_lock));
}

static void notify_client(struct bt_conn *conn, void *data)
{
	struct csis_notify_foreach *csis_data = (struct csis_notify_foreach *)data;
	struct bt_csis *csis = csis_data->csis;
	struct bt_conn *excluded_conn = csis_data->excluded_client;

	if (excluded_conn != NULL && conn == excluded_conn) {
		return;
	}

	notify_lock_value(csis, conn);

	for (int i = 0; i < ARRAY_SIZE(csis->srv.pend_notify); i++) {
		struct csis_pending_notifications *pend_notify;

		pend_notify = &csis->srv.pend_notify[i];

		if (pend_notify->pending &&
		    bt_addr_le_cmp(bt_conn_get_dst(conn),
				   &pend_notify->addr) == 0) {
			pend_notify->pending = false;
			break;
		}
	}
}

static void notify_clients(struct bt_csis *csis,
			   struct bt_conn *excluded_client)
{
	struct csis_notify_foreach data = {
		.excluded_client = excluded_client,
		.csis = csis,
	};

	/* Mark all bonded devices as pending notifications, and clear those
	 * that are notified in `notify_client`
	 */
	for (int i = 0; i < ARRAY_SIZE(csis->srv.pend_notify); i++) {
		struct csis_pending_notifications *pend_notify;

		pend_notify = &csis->srv.pend_notify[i];

		if (pend_notify->active) {
			if (excluded_client != NULL &&
			    bt_addr_le_cmp(bt_conn_get_dst(excluded_client),
					   &pend_notify->addr) == 0) {
				continue;
			}

			pend_notify->pending = true;
		}
	}

	bt_conn_foreach(BT_CONN_TYPE_ALL, notify_client, &data);
}

static int sirk_encrypt(struct bt_conn *conn,
			const struct bt_csis_set_sirk *sirk,
			struct bt_csis_set_sirk *enc_sirk)
{
	int err;
	uint8_t *k;

	if (IS_ENABLED(CONFIG_BT_CSIS_TEST_SAMPLE_DATA)) {
		/* test_k is from the sample data from A.2 in the CSIS spec */
		static uint8_t test_k[] = {0x67, 0x6e, 0x1b, 0x9b,
					   0xd4, 0x48, 0x69, 0x6f,
					   0x06, 0x1e, 0xc6, 0x22,
					   0x3c, 0xe5, 0xce, 0xd9};
		static bool swapped;

		if (!swapped && (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)) {
			/* Swap test_k to little endian */
			sys_mem_swap(test_k, 16);
			swapped = true;
		}
		BT_DBG("Encrypting test SIRK");
		k = test_k;
	} else {
		k = conn->le.keys->ltk.val;
	}

	err = bt_csis_sef(k, sirk->value, enc_sirk->value);

	if (err != 0) {
		return err;
	}

	enc_sirk->type = BT_CSIS_SIRK_TYPE_ENCRYPTED;

	return 0;
}

static int generate_prand(uint32_t *dest)
{
	bool valid = false;

	do {
		int res;

		*dest = 0;
		res = bt_rand(dest, BT_CSIS_SIH_PRAND_SIZE);
		if (res != 0) {
			return res;
		}

		/* Validate Prand: Must contain both a 1 and a 0 */
		if (*dest != 0 && *dest != 0x3FFFFF) {
			valid = true;
		}
	} while (!valid);

	*dest &= 0x3FFFFF;
	*dest |= BIT(22); /* bit 23 shall be 0, and bit 22 shall be 1 */

	return 0;
}

int bt_csis_generate_rsi(const struct bt_csis *csis, uint8_t rsi[BT_CSIS_RSI_SIZE])
{
	int res = 0;
	uint32_t prand;
	uint32_t hash;

	if (IS_ENABLED(CONFIG_BT_CSIS_TEST_SAMPLE_DATA)) {
		/* prand is from the sample data from A.2 in the CSIS spec */
		prand = 0x69f563;
	} else {
		res = generate_prand(&prand);

		if (res != 0) {
			BT_WARN("Could not generate new prand");
			return res;
		}
	}

	res = bt_csis_sih(csis->srv.set_sirk.value, prand, &hash);
	if (res != 0) {
		BT_WARN("Could not generate new RSI");
		return res;
	}

	(void)memcpy(rsi, &hash, BT_CSIS_SIH_HASH_SIZE);
	(void)memcpy(rsi + BT_CSIS_SIH_HASH_SIZE, &prand, BT_CSIS_SIH_PRAND_SIZE);

	return res;
}

static ssize_t read_set_sirk(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     void *buf, uint16_t len, uint16_t offset)
{
	struct bt_csis_set_sirk enc_sirk;
	struct bt_csis_set_sirk *sirk;
	struct bt_csis *csis = BT_AUDIO_CHRC_USER_DATA(attr);

	if (csis->srv.cb != NULL && csis->srv.cb->sirk_read_req != NULL) {
		uint8_t cb_rsp;

		/* Ask higher layer for what SIRK to return, if any */
		cb_rsp = csis->srv.cb->sirk_read_req(conn, &csis_insts[0]);

		if (cb_rsp == BT_CSIS_READ_SIRK_REQ_RSP_ACCEPT) {
			sirk = &csis->srv.set_sirk;
		} else if (IS_ENABLED(CONFIG_BT_CSIS_ENC_SIRK_SUPPORT) &&
			   cb_rsp == BT_CSIS_READ_SIRK_REQ_RSP_ACCEPT_ENC) {
			int err;

			err = sirk_encrypt(conn, &csis->srv.set_sirk,
					   &enc_sirk);
			if (err != 0) {
				BT_ERR("Could not encrypt SIRK: %d",
					err);
				return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
			}

			sirk = &enc_sirk;
			BT_HEXDUMP_DBG(enc_sirk.value, sizeof(enc_sirk.value),
				       "Encrypted Set SIRK");
		} else if (cb_rsp == BT_CSIS_READ_SIRK_REQ_RSP_REJECT) {
			return BT_GATT_ERR(BT_ATT_ERR_AUTHORIZATION);
		} else if (cb_rsp == BT_CSIS_READ_SIRK_REQ_RSP_OOB_ONLY) {
			return BT_GATT_ERR(BT_CSIS_ERROR_SIRK_OOB_ONLY);
		} else {
			BT_ERR("Invalid callback response: %u", cb_rsp);
			return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
		}
	} else {
		sirk = &csis->srv.set_sirk;
	}


	BT_DBG("Set sirk %sencrypted",
	       sirk->type ==  BT_CSIS_SIRK_TYPE_PLAIN ? "not " : "");
	BT_HEXDUMP_DBG(csis->srv.set_sirk.value,
		       sizeof(csis->srv.set_sirk.value), "Set SIRK");
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 sirk, sizeof(*sirk));
}

static void set_sirk_cfg_changed(const struct bt_gatt_attr *attr,
				 uint16_t value)
{
	BT_DBG("value 0x%04x", value);
}

static ssize_t read_set_size(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     void *buf, uint16_t len, uint16_t offset)
{
	struct bt_csis *csis = BT_AUDIO_CHRC_USER_DATA(attr);

	BT_DBG("%u", csis->srv.set_size);

	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &csis->srv.set_size,
				 sizeof(csis->srv.set_size));
}

static void set_size_cfg_changed(const struct bt_gatt_attr *attr,
				 uint16_t value)
{
	BT_DBG("value 0x%04x", value);
}

static ssize_t read_set_lock(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     void *buf, uint16_t len, uint16_t offset)
{
	struct bt_csis *csis = BT_AUDIO_CHRC_USER_DATA(attr);

	BT_DBG("%u", csis->srv.set_lock);

	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &csis->srv.set_lock,
				 sizeof(csis->srv.set_lock));
}

/**
 * @brief Set the lock value of a CSIS instance.
 *
 * @param conn  The connection locking the instance.
 *              Will be NULL if the server locally sets the lock.
 * @param csis  The CSIS instance to change the lock value of
 * @param val   The lock value (BT_CSIS_LOCK_VALUE or BT_CSIS_RELEASE_VALUE)
 *
 * @return BT_CSIS_ERROR_* on failure or 0 if success
 */
static uint8_t set_lock(struct bt_conn *conn, struct bt_csis *csis, uint8_t val)
{
	bool notify;

	if (val != BT_CSIS_RELEASE_VALUE && val != BT_CSIS_LOCK_VALUE) {
		return BT_CSIS_ERROR_LOCK_INVAL_VALUE;
	}

	if (csis->srv.set_lock == BT_CSIS_LOCK_VALUE) {
		if (val == BT_CSIS_LOCK_VALUE) {
			if (is_last_client_to_write(csis, conn)) {
				return BT_CSIS_ERROR_LOCK_ALREADY_GRANTED;
			} else {
				return BT_CSIS_ERROR_LOCK_DENIED;
			}
		} else if (!is_last_client_to_write(csis, conn)) {
			return BT_CSIS_ERROR_LOCK_RELEASE_DENIED;
		}
	}

	notify = csis->srv.set_lock != val;

	csis->srv.set_lock = val;
	if (csis->srv.set_lock == BT_CSIS_LOCK_VALUE) {
		if (conn != NULL) {
			bt_addr_le_copy(&csis->srv.lock_client_addr,
					bt_conn_get_dst(conn));
		}
		(void)k_work_reschedule(&csis->srv.set_lock_timer,
					CSIS_SET_LOCK_TIMER_VALUE);
	} else {
		(void)memset(&csis->srv.lock_client_addr, 0,
			     sizeof(csis->srv.lock_client_addr));
		(void)k_work_cancel_delayable(&csis->srv.set_lock_timer);
	}

	BT_DBG("%u", csis->srv.set_lock);

	if (notify) {
		/*
		 * The Spec states that all clients, except for the
		 * client writing the value, shall be notified
		 * (if subscribed)
		 */
		notify_clients(csis, conn);

		if (csis->srv.cb != NULL && csis->srv.cb->lock_changed != NULL) {
			bool locked = csis->srv.set_lock == BT_CSIS_LOCK_VALUE;

			csis->srv.cb->lock_changed(conn, csis, locked);
		}
	}

	return 0;
}

static ssize_t write_set_lock(struct bt_conn *conn,
			      const struct bt_gatt_attr *attr,
			      const void *buf, uint16_t len,
			      uint16_t offset, uint8_t flags)
{
	ssize_t res;
	uint8_t val;
	struct bt_csis *csis = BT_AUDIO_CHRC_USER_DATA(attr);

	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	} else if (len != sizeof(val)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	(void)memcpy(&val, buf, len);

	res = set_lock(conn, csis, val);
	if (res != BT_ATT_ERR_SUCCESS) {
		return BT_GATT_ERR(res);
	}

	return len;
}

static void set_lock_cfg_changed(const struct bt_gatt_attr *attr,
				 uint16_t value)
{
	BT_DBG("value 0x%04x", value);
}

static ssize_t read_rank(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, uint16_t len, uint16_t offset)
{
	struct bt_csis *csis = BT_AUDIO_CHRC_USER_DATA(attr);

	BT_DBG("%u", csis->srv.rank);

	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &csis->srv.rank,
				 sizeof(csis->srv.rank));

}

static void set_lock_timer_handler(struct k_work *work)
{
	struct k_work_delayable *delayable;
	struct bt_csis_server *server;
	struct bt_csis *csis;

	delayable = CONTAINER_OF(work, struct k_work_delayable, work);
	server = CONTAINER_OF(delayable, struct bt_csis_server, set_lock_timer);
	csis = CONTAINER_OF(server, struct bt_csis, srv);

	BT_DBG("Lock timeout, releasing");
	csis->srv.set_lock = BT_CSIS_RELEASE_VALUE;
	notify_clients(csis, NULL);

	if (csis->srv.cb != NULL && csis->srv.cb->lock_changed != NULL) {
		bool locked = csis->srv.set_lock == BT_CSIS_LOCK_VALUE;

		csis->srv.cb->lock_changed(NULL, csis, locked);
	}
}

static void csis_security_changed(struct bt_conn *conn, bt_security_t level,
				  enum bt_security_err err)
{
	if (err != 0 || conn->encrypt == 0) {
		return;
	}

	if (!bt_addr_le_is_bonded(conn->id, &conn->le.dst)) {
		return;
	}

	for (int i = 0; i < ARRAY_SIZE(csis_insts); i++) {
		struct bt_csis *csis = &csis_insts[i];

		for (int j = 0; j < ARRAY_SIZE(csis->srv.pend_notify); j++) {
			struct csis_pending_notifications *pend_notify;

			pend_notify = &csis->srv.pend_notify[j];

			if (pend_notify->pending &&
			    bt_addr_le_cmp(bt_conn_get_dst(conn),
					   &pend_notify->addr) == 0) {
				notify_lock_value(csis, conn);
				pend_notify->pending = false;
				break;
			}
		}
	}
}

static void handle_csis_disconnect(struct bt_csis *csis, struct bt_conn *conn)
{
	BT_DBG("Non-bonded device");
	if (is_last_client_to_write(csis, conn)) {
		(void)memset(&csis->srv.lock_client_addr, 0,
			     sizeof(csis->srv.lock_client_addr));
		csis->srv.set_lock = BT_CSIS_RELEASE_VALUE;
		notify_clients(csis, NULL);

		if (csis->srv.cb != NULL && csis->srv.cb->lock_changed != NULL) {
			bool locked = csis->srv.set_lock == BT_CSIS_LOCK_VALUE;

			csis->srv.cb->lock_changed(conn, csis, locked);
		}
	}

	/* Check if the disconnected device once was bonded and stored
	 * here as a bonded device
	 */
	for (int i = 0; i < ARRAY_SIZE(csis->srv.pend_notify); i++) {
		struct csis_pending_notifications *pend_notify;

		pend_notify = &csis->srv.pend_notify[i];

		if (bt_addr_le_cmp(bt_conn_get_dst(conn),
				   &pend_notify->addr) == 0) {
			(void)memset(pend_notify, 0, sizeof(*pend_notify));
			break;
		}
	}
}

static void csis_disconnected(struct bt_conn *conn, uint8_t reason)
{
	BT_DBG("Disconnected: %s (reason %u)",
	       bt_addr_le_str(bt_conn_get_dst(conn)), reason);

	for (int i = 0; i < ARRAY_SIZE(csis_insts); i++) {
		handle_csis_disconnect(&csis_insts[i], conn);
	}
}

static void handle_csis_auth_complete(struct bt_csis *csis,
				      struct bt_conn *conn)
{
	/* Check if already in list, and do nothing if it is */
	for (int i = 0; i < ARRAY_SIZE(csis->srv.pend_notify); i++) {
		struct csis_pending_notifications *pend_notify;

		pend_notify = &csis->srv.pend_notify[i];

		if (pend_notify->active &&
		    bt_addr_le_cmp(bt_conn_get_dst(conn),
				   &pend_notify->addr) == 0) {
#if IS_ENABLED(CONFIG_BT_KEYS_OVERWRITE_OLDEST)
			pend_notify->age = csis->srv.age_counter++;
#endif /* CONFIG_BT_KEYS_OVERWRITE_OLDEST */
			return;
		}
	}

	/* Copy addr to list over devices to save notifications for */
	for (int i = 0; i < ARRAY_SIZE(csis->srv.pend_notify); i++) {
		struct csis_pending_notifications *pend_notify;

		pend_notify = &csis->srv.pend_notify[i];

		if (!pend_notify->active) {
			bt_addr_le_copy(&pend_notify->addr,
					bt_conn_get_dst(conn));
			pend_notify->active = true;
#if IS_ENABLED(CONFIG_BT_KEYS_OVERWRITE_OLDEST)
			pend_notify->age = csis->srv.age_counter++;
#endif /* CONFIG_BT_KEYS_OVERWRITE_OLDEST */
			return;
		}
	}

#if IS_ENABLED(CONFIG_BT_KEYS_OVERWRITE_OLDEST)
	struct csis_pending_notifications *oldest;

	oldest = &csis->srv.pend_notify[0];

	for (int i = 1; i < ARRAY_SIZE(csis->srv.pend_notify); i++) {
		struct csis_pending_notifications *pend_notify;

		pend_notify = &csis->srv.pend_notify[i];

		if (pend_notify->age < oldest->age) {
			oldest = pend_notify;
		}
	}
	(void)memset(oldest, 0, sizeof(*oldest));
	bt_addr_le_copy(&oldest->addr, &conn->le.dst);
	oldest->active = true;
	oldest->age = csis->srv.age_counter++;
#else
	BT_WARN("Could not add device to pending notification list");
#endif /* CONFIG_BT_KEYS_OVERWRITE_OLDEST */

}

static void auth_pairing_complete(struct bt_conn *conn, bool bonded)
{
	/**
	 * If a pairing is complete for a bonded device, then we
	 * 1) Store the connection pointer to later validate SIRK encryption
	 * 2) Check if the device is already in the `pend_notify`, and if it is
	 * not, then we
	 * 3) Check if there's room for another device in the `pend_notify`
	 *    array. If there are no more room for a new device, then
	 * 4) Either we ignore this new device (bad luck), or we overwrite
	 *    the oldest entry, following the behavior of the key storage.
	 */

	BT_DBG("%s paired (%sbonded)",
	       bt_addr_le_str(bt_conn_get_dst(conn)), bonded ? "" : "not ");

	if (!bonded) {
		return;
	}

	for (int i = 0; i < ARRAY_SIZE(csis_insts); i++) {
		handle_csis_auth_complete(&csis_insts[i], conn);
	}
}

static void csis_bond_deleted(uint8_t id, const bt_addr_le_t *peer)
{
	for (int i = 0; i < ARRAY_SIZE(csis_insts); i++) {
		struct bt_csis *csis = &csis_insts[i];

		for (int j = 0; j < ARRAY_SIZE(csis->srv.pend_notify); j++) {
			struct csis_pending_notifications *pend_notify;

			pend_notify = &csis->srv.pend_notify[j];

			if (pend_notify->active &&
			    bt_addr_le_cmp(peer, &pend_notify->addr) == 0) {
				(void)memset(pend_notify, 0,
					     sizeof(*pend_notify));
				break;
			}
		}
	}
}

static struct bt_conn_cb conn_callbacks = {
	.disconnected = csis_disconnected,
	.security_changed = csis_security_changed,
};

static struct bt_conn_auth_info_cb auth_callbacks = {
	.pairing_complete = auth_pairing_complete,
	.bond_deleted = csis_bond_deleted
};

#define BT_CSIS_SERVICE_DEFINITION(_csis) {\
	BT_GATT_PRIMARY_SERVICE(BT_UUID_CSIS), \
	BT_AUDIO_CHRC(BT_UUID_CSIS_SET_SIRK, \
		      BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, \
		      BT_GATT_PERM_READ_ENCRYPT, \
		      read_set_sirk, NULL, &_csis), \
	BT_AUDIO_CCC(set_sirk_cfg_changed), \
	BT_AUDIO_CHRC(BT_UUID_CSIS_SET_SIZE, \
		      BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, \
		      BT_GATT_PERM_READ_ENCRYPT, \
		      read_set_size, NULL, &_csis), \
	BT_AUDIO_CCC(set_size_cfg_changed), \
	BT_AUDIO_CHRC(BT_UUID_CSIS_SET_LOCK, \
		      BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE, \
		      BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, \
		      read_set_lock, write_set_lock, &_csis), \
	BT_AUDIO_CCC(set_lock_cfg_changed), \
	BT_AUDIO_CHRC(BT_UUID_CSIS_RANK, \
		      BT_GATT_CHRC_READ, \
		      BT_GATT_PERM_READ_ENCRYPT, \
		      read_rank, NULL, &_csis) \
	}

BT_GATT_SERVICE_INSTANCE_DEFINE(csis_service_list, csis_insts,
				CONFIG_BT_CSIS_MAX_INSTANCE_COUNT,
				BT_CSIS_SERVICE_DEFINITION);

/****************************** Public API ******************************/
void *bt_csis_svc_decl_get(const struct bt_csis *csis)
{
	return csis->srv.service_p->attrs;
}

static bool valid_register_param(const struct bt_csis_register_param *param)
{
	if (param->lockable && param->rank == 0) {
		BT_DBG("Rank cannot be 0 if service is lockable");
		return false;
	}

	if (param->rank > 0 && param->rank > param->set_size) {
		BT_DBG("Invalid rank: %u (shall be less than set_size: %u)",
		       param->set_size, param->set_size);
		return false;
	}

#if CONFIG_BT_CSIS_MAX_INSTANCE_COUNT > 1
	if (param->parent == NULL) {
		BT_DBG("Parent service not provided");
		return false;
	}
#endif /* CONFIG_BT_CSIS_MAX_INSTANCE_COUNT > 1 */

	return true;
}

int bt_csis_register(const struct bt_csis_register_param *param,
		     struct bt_csis **csis)
{
	static uint8_t instance_cnt;
	struct bt_csis *inst;
	int err;

	if (instance_cnt == ARRAY_SIZE(csis_insts)) {
		return -ENOMEM;
	}

	CHECKIF(param == NULL) {
		BT_DBG("NULL param");
		return -EINVAL;
	}

	CHECKIF(!valid_register_param(param)) {
		BT_DBG("Invalid parameters");
		return -EINVAL;
	}

	inst = &csis_insts[instance_cnt];
	inst->srv.service_p = &csis_service_list[instance_cnt];
	instance_cnt++;

	bt_conn_cb_register(&conn_callbacks);
	bt_conn_auth_info_cb_register(&auth_callbacks);

	err = bt_gatt_service_register(inst->srv.service_p);
	if (err != 0) {
		BT_DBG("CSIS service register failed: %d", err);
		return err;
	}

	k_work_init_delayable(&inst->srv.set_lock_timer,
			      set_lock_timer_handler);
	inst->srv.rank = param->rank;
	inst->srv.set_size = param->set_size;
	inst->srv.set_lock = BT_CSIS_RELEASE_VALUE;
	inst->srv.set_sirk.type = BT_CSIS_SIRK_TYPE_PLAIN;
	inst->srv.cb = param->cb;

	if (IS_ENABLED(CONFIG_BT_CSIS_TEST_SAMPLE_DATA)) {
		uint8_t test_sirk[] = {
			0xcd, 0xcc, 0x72, 0xdd, 0x86, 0x8c, 0xcd, 0xce,
			0x22, 0xfd, 0xa1, 0x21, 0x09, 0x7d, 0x7d, 0x45,
		};

		(void)memcpy(inst->srv.set_sirk.value, test_sirk,
			     sizeof(test_sirk));
		BT_DBG("CSIS SIRK was overwritten by sample data SIRK");
	} else {
		(void)memcpy(inst->srv.set_sirk.value, param->set_sirk,
			     sizeof(inst->srv.set_sirk.value));
	}

	*csis = inst;
	return 0;
}

int bt_csis_lock(struct bt_csis *csis, bool lock, bool force)
{
	uint8_t lock_val;
	int err = 0;

	if (lock) {
		lock_val = BT_CSIS_LOCK_VALUE;
	} else {
		lock_val = BT_CSIS_RELEASE_VALUE;
	}

	if (!lock && force) {
		csis->srv.set_lock = BT_CSIS_RELEASE_VALUE;
		notify_clients(csis, NULL);

		if (csis->srv.cb != NULL && csis->srv.cb->lock_changed != NULL) {
			csis->srv.cb->lock_changed(NULL, &csis_insts[0], false);
		}
	} else {
		err = set_lock(NULL, csis, lock_val);
	}

	if (err < 0) {
		return BT_GATT_ERR(err);
	} else {
		return 0;
	}
}

void bt_csis_print_sirk(const struct bt_csis *csis)
{
	BT_HEXDUMP_DBG(&csis->srv.set_sirk, sizeof(csis->srv.set_sirk),
		       "Set SIRK");
}
