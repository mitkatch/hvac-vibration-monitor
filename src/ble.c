/*
 * ble.c — BLE peripheral service for HVAC Vibration Monitor
 *
 * This module provides BLE functionality as service functions
 * called by the FSM. All BLE callbacks (connected, disconnected,
 * CCC changed) post events to the FSM's event queue rather than
 * taking action themselves.
 *
 * Dependency direction:
 *   main.c → ble.c → fsm.h (types only)
 *
 * ble.c does NOT know about states or transitions.
 * It only knows how to:
 *   - Initialize the BLE stack
 *   - Start/stop advertising
 *   - Send notification packets
 *   - Clean up after disconnection
 *   - Post events when things happen
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>

#include "ble.h"
#include "fsm.h"

LOG_MODULE_REGISTER(ble, LOG_LEVEL_INF);

/* ──────────────────────────────────────────────
 * Event posting callback (set during init)
 * ────────────────────────────────────────────── */
static event_post_fn post_event = NULL;

/* ──────────────────────────────────────────────
 * Connection state (owned by ble.c)
 * ────────────────────────────────────────────── */
static struct bt_conn *current_conn = NULL;
static bool notifications_enabled = false;

/* ──────────────────────────────────────────────
 * UUIDs
 *
 * Custom vibration monitoring service.
 * Using raw byte arrays for compile-time constants.
 * ────────────────────────────────────────────── */

/* Service:        12345678-1234-5678-1234-56789abcdef0 */
static const struct bt_uuid_128 vibration_svc_uuid = BT_UUID_INIT_128(
	0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

/* Characteristic: 12345678-1234-5678-1234-56789abcdef1 (vibration data) */
static const struct bt_uuid_128 vibration_char_uuid = BT_UUID_INIT_128(
	0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

/* Characteristic: 12345678-1234-5678-1234-56789abcdef2 (environment data) */
static const struct bt_uuid_128 env_char_uuid = BT_UUID_INIT_128(
	0xf2, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

/* ──────────────────────────────────────────────
 * CCC Changed Callback
 *
 * Fires when the client enables/disables notifications.
 * Posts an event — does not act.
 * ────────────────────────────────────────────── */
static void vibration_ccc_changed(const struct bt_gatt_attr *attr,
				  uint16_t value)
{
	notifications_enabled = (value == BT_GATT_CCC_NOTIFY);

	LOG_INF("BLE: Vibration notifications %s",
		notifications_enabled ? "enabled" : "disabled");

	if (post_event) {
		fsm_event_t evt = {
			.type = notifications_enabled
				? EVT_BLE_NOTIFICATIONS_ENABLED
				: EVT_BLE_NOTIFICATIONS_DISABLED,
		};
		post_event(&evt);
	}
}

static void env_ccc_changed(const struct bt_gatt_attr *attr,
			    uint16_t value)
{
	bool enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("BLE: Environment notifications %s",
		enabled ? "enabled" : "disabled");
}

/* ──────────────────────────────────────────────
 * GATT Service Definition
 * ────────────────────────────────────────────── */
BT_GATT_SERVICE_DEFINE(vibration_svc,
	/* Service declaration */
	BT_GATT_PRIMARY_SERVICE(&vibration_svc_uuid),

	/* Vibration data characteristic (notify only) */
	BT_GATT_CHARACTERISTIC(&vibration_char_uuid.uuid,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),
	BT_GATT_CCC(vibration_ccc_changed,
		     BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	/* Environment data characteristic (notify only) */
	BT_GATT_CHARACTERISTIC(&env_char_uuid.uuid,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),
	BT_GATT_CCC(env_ccc_changed,
		     BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* ──────────────────────────────────────────────
 * Connection Callbacks
 *
 * These run in the BLE stack's thread context.
 * They store/release the connection reference
 * and post events. No heavy work.
 * ────────────────────────────────────────────── */
static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("BLE: Connection failed (err 0x%02x)", err);
		return;
	}

	LOG_INF("BLE: Connected");

	/* Take a reference — ble.c owns this */
	current_conn = bt_conn_ref(conn);
	notifications_enabled = false;

	if (post_event) {
		fsm_event_t evt = { .type = EVT_BLE_CONNECTED };
		post_event(&evt);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("BLE: Disconnected (reason 0x%02x)", reason);

	/*
	 * Do NOT unref or restart advertising here.
	 * Post the event and let the FSM handle cleanup
	 * in the main thread context with proper timing.
	 */
	if (post_event) {
		fsm_event_t evt = {
			.type = EVT_BLE_DISCONNECTED,
			.data.ble_disconnect.reason = reason,
		};
		post_event(&evt);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
};

/* ──────────────────────────────────────────────
 * Advertising Data
 * ────────────────────────────────────────────── */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS,
		      BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
		      0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
		      0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE,
		CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* ──────────────────────────────────────────────
 * Public API — called by FSM from main thread
 * ────────────────────────────────────────────── */

int ble_init(event_post_fn callback)
{
	int err;

	post_event = callback;

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("BLE: bt_enable failed (err %d)", err);
		return err;
	}

	LOG_INF("BLE: Stack initialized");
	return 0;
}

int ble_start_advertising(void)
{
	int err;

	struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.sid = 0,
		.secondary_max_skip = 0,
		.options = BT_LE_ADV_OPT_CONN,
		.interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
		.interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
		.peer = NULL,
	};

	err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("BLE: Advertising start failed (err %d)", err);
		return err;
	}

	LOG_INF("BLE: Advertising started (%s)", CONFIG_BT_DEVICE_NAME);
	return 0;
}

int ble_stop_advertising(void)
{
	int err = bt_le_adv_stop();
	if (err && err != -EALREADY) {
		LOG_ERR("BLE: Advertising stop failed (err %d)", err);
		return err;
	}
	return 0;
}

void ble_cleanup(void)
{
	/*
	 * Called by FSM when entering DISCONNECTED state.
	 * If connection is still active (e.g., TX failed but link
	 * is alive), force-disconnect so the stack releases buffers.
	 * Then release our reference.
	 */
	if (current_conn) {
		/* Request clean disconnect if link is still up */
		int err = bt_conn_disconnect(current_conn,
					     BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		if (err && err != -ENOTCONN) {
			LOG_WRN("BLE: Disconnect request failed (err %d)", err);
		}

		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	notifications_enabled = false;

	LOG_INF("BLE: Cleanup complete (disconnected, conn unref, state reset)");
}

bool ble_is_connected(void)
{
	return (current_conn != NULL);
}

bool ble_notifications_enabled(void)
{
	return notifications_enabled;
}

/* ──────────────────────────────────────────────
 * Burst Transmission
 *
 * Sends vibration data as a sequence of BLE
 * notification packets. Called synchronously
 * by the FSM in TRANSMITTING state.
 *
 * Returns:
 *   0       on success (all packets sent)
 *   -ENOTCONN  if connection lost
 *   -ENOMEM    if TX buffer exhausted after retries
 *   other   BLE stack error
 * ────────────────────────────────────────────── */
#define MAX_PAYLOAD         244
#define ENOMEM_RETRY_MAX    3
#define ENOMEM_RETRY_MS     50
#define INTER_PACKET_MS     10

int ble_transmit_burst(const accel_sample_t *buffer, uint16_t count)
{
	if (!current_conn) {
		return -ENOTCONN;
	}

	const uint16_t total_bytes = count * sizeof(accel_sample_t);
	const uint16_t num_packets = (total_bytes + MAX_PAYLOAD - 1) / MAX_PAYLOAD;

	LOG_INF("BLE TX: %d samples, %d bytes, %d packets",
		count, total_bytes, num_packets);

	uint16_t offset = 0;

	for (uint16_t i = 0; i < num_packets; i++) {

		/* Check connection is still alive before each packet */
		if (!current_conn) {
			LOG_WRN("BLE TX: Connection lost at packet %d/%d",
				i, num_packets);
			return -ENOTCONN;
		}

		uint16_t remaining = total_bytes - offset;
		uint16_t chunk = (remaining > MAX_PAYLOAD) ?
				 MAX_PAYLOAD : remaining;

		/* Attempt to send with bounded retries for ENOMEM */
		int err = -1;
		for (int retry = 0; retry <= ENOMEM_RETRY_MAX; retry++) {

			err = bt_gatt_notify(current_conn,
					     &vibration_svc.attrs[2],
					     (const uint8_t *)buffer + offset,
					     chunk);

			if (err == 0) {
				break;  /* success */
			}

			if (err == -ENOMEM && retry < ENOMEM_RETRY_MAX) {
				/* TX buffer full — wait and retry */
				LOG_DBG("BLE TX: Buffer full on pkt %d, "
					"retry %d/%d",
					i, retry + 1, ENOMEM_RETRY_MAX);
				k_msleep(ENOMEM_RETRY_MS);
				continue;
			}

			/* Non-retryable error or retries exhausted */
			LOG_ERR("BLE TX: Failed on pkt %d/%d (err %d)",
				i, num_packets, err);
			return err;
		}

		offset += chunk;

		/* Progress logging */
		if ((i + 1) % 5 == 0 || i == num_packets - 1) {
			LOG_INF("BLE TX: %d/%d packets sent", i + 1,
				num_packets);
		}

		/* Inter-packet pacing */
		if (i < num_packets - 1) {
			k_msleep(INTER_PACKET_MS);
		}
	}

	LOG_INF("BLE TX: Burst complete");
	return 0;
}

/* ──────────────────────────────────────────────
 * Environment Data Transmission
 *
 * Much smaller than vibration — single packet.
 * ────────────────────────────────────────────── */
int ble_transmit_environment(const env_reading_t *reading)
{
	if (!current_conn) {
		return -ENOTCONN;
	}

	if (!reading->valid) {
		return -EINVAL;
	}

	/* Pack into a compact payload:
	 * [temp_hi][temp_lo][hum_hi][hum_lo][press_hi][press_lo]
	 * 6 bytes total — fits in one notification easily
	 */
	uint8_t payload[6];
	payload[0] = (reading->temperature >> 8) & 0xFF;
	payload[1] = reading->temperature & 0xFF;
	payload[2] = (reading->humidity >> 8) & 0xFF;
	payload[3] = reading->humidity & 0xFF;
	payload[4] = (reading->pressure >> 8) & 0xFF;
	payload[5] = reading->pressure & 0xFF;

	int err = bt_gatt_notify(current_conn,
				 &vibration_svc.attrs[5],
				 payload, sizeof(payload));
	if (err) {
		LOG_ERR("BLE TX: Environment notify failed (err %d)", err);
		return err;
	}

	LOG_DBG("BLE TX: Environment data sent "
		"(T=%d.%02d°C H=%d.%02d%% P=%dhPa)",
		reading->temperature / 100,
		reading->temperature % 100,
		reading->humidity / 100,
		reading->humidity % 100,
		reading->pressure);

	return 0;
}