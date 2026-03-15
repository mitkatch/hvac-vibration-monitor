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
#include <zephyr/random/random.h>
#include <zephyr/settings/settings.h>
#include <stdlib.h>   /* abs() */
#include <string.h>   /* memcpy() */

#include "ble.h"
#include "fsm.h"
#include "analysis.h"
#include "name_store.h"

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
 * Security and NUS state
 * ────────────────────────────────────────────── */
static uint32_t current_passkey = 0;
static bool nus_tx_enabled = false;

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
 * Nordic UART Service (NUS) UUIDs
 * 
 * Standard Nordic service for serial communication.
 * Used to display the random PIN to the user via
 * nRF Connect app's UART terminal.
 * ────────────────────────────────────────────── */

/* NUS Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E */
static const struct bt_uuid_128 nus_svc_uuid = BT_UUID_INIT_128(
	0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
	0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);

/* NUS TX Char: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E (device → phone) */
static const struct bt_uuid_128 nus_tx_uuid = BT_UUID_INIT_128(
	0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
	0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E);

/* NUS RX Char: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E (phone → device) */
static const struct bt_uuid_128 nus_rx_uuid = BT_UUID_INIT_128(
	0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
	0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);

/* ──────────────────────────────────────────────
 * NUS TX CCC Changed Callback
 * 
 * Tracks when the UART terminal is opened/closed
 * in nRF Connect app.
 * ────────────────────────────────────────────── */
static void nus_tx_ccc_changed(const struct bt_gatt_attr *attr,
			       uint16_t value)
{
	nus_tx_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("NUS: UART terminal %s",
		nus_tx_enabled ? "opened" : "closed");
}

/* ──────────────────────────────────────────────
 * NUS Response Helper (forward declaration —
 * implementation after BT_GATT_SERVICE_DEFINE)
 * ────────────────────────────────────────────── */
static void _nus_respond(const char *msg);

/* ──────────────────────────────────────────────
 * NUS Command Prefix
 * ────────────────────────────────────────────── */
#define CMD_CHANGE_NAME  "CHANGE_NAME:"
#define CMD_CHANGE_NAME_LEN  12  /* strlen("CHANGE_NAME:") */

/* ──────────────────────────────────────────────
 * NUS RX Write Callback
 *
 * Receives commands from gateway via UART service.
 * Supported commands:
 *   CHANGE_NAME:<new_name>  — set custom advertising name
 *   CHANGE_NAME:            — revert to build default
 * ────────────────────────────────────────────── */
static ssize_t nus_rx_write(struct bt_conn *conn,
			    const struct bt_gatt_attr *attr,
			    const void *buf, uint16_t len,
			    uint16_t offset, uint8_t flags)
{
	/* Null-terminate for safe string ops */
	char cmd[NAME_STORE_MAX_LEN + CMD_CHANGE_NAME_LEN + 1];
	uint16_t copy_len = (len < sizeof(cmd) - 1) ? len : sizeof(cmd) - 1;
	memcpy(cmd, buf, copy_len);
	cmd[copy_len] = '\0';

	/* Strip trailing \r\n if present */
	while (copy_len > 0 && (cmd[copy_len - 1] == '\r' ||
				cmd[copy_len - 1] == '\n')) {
		cmd[--copy_len] = '\0';
	}

	LOG_INF("NUS RX: \"%s\" (%u bytes)", cmd, len);

	/* Parse CHANGE_NAME command */
	if (strncmp(cmd, CMD_CHANGE_NAME, CMD_CHANGE_NAME_LEN) == 0) {
		const char *new_name = cmd + CMD_CHANGE_NAME_LEN;

		int err = name_store_set(new_name);
		if (err == -EINVAL) {
			LOG_ERR("NUS: Name rejected (non-ASCII characters)");
			_nus_respond("ERR:INVALID_NAME\r\n");
		} else if (err) {
			LOG_ERR("NUS: Name save failed (err %d)", err);
			_nus_respond("ERR:SAVE_FAILED\r\n");
		} else {
			LOG_INF("NUS: Name accepted: \"%s\"", name_store_get());
			/* Respond with confirmation */
			char resp[NAME_STORE_MAX_LEN + 16];
			int resp_len = snprintk(resp, sizeof(resp),
						"OK:NAME=%s\r\n",
						name_store_get());
			_nus_respond(resp);
		}
	} else {
		LOG_DBG("NUS: Unknown command, ignored");
	}

	return len;
}

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
 * Nordic UART Service Definition
 * 
 * This service allows displaying text to the user
 * via nRF Connect app. Used for showing the random
 * pairing PIN.
 * ────────────────────────────────────────────── */
BT_GATT_SERVICE_DEFINE(nus_svc,
	BT_GATT_PRIMARY_SERVICE(&nus_svc_uuid),

	/* TX characteristic (device → phone) */
	BT_GATT_CHARACTERISTIC(&nus_tx_uuid.uuid,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),
	BT_GATT_CCC(nus_tx_ccc_changed,
#ifdef BLE_NO_PAIRING
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#else
		    BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN),
#endif

	/* RX characteristic (phone → device) */
	BT_GATT_CHARACTERISTIC(&nus_rx_uuid.uuid,
			       BT_GATT_CHRC_WRITE |
			       BT_GATT_CHRC_WRITE_WITHOUT_RESP,
#ifdef BLE_NO_PAIRING
			       BT_GATT_PERM_WRITE,
#else
			       BT_GATT_PERM_WRITE_AUTHEN,
#endif
			       NULL, nus_rx_write, NULL),
);


/* ──────────────────────────────────────────────
 * NUS Response Helper (implementation)
 * ────────────────────────────────────────────── */
static void _nus_respond(const char *msg)
{
	if (!nus_tx_enabled || !current_conn) {
		return;
	}
	int err = bt_gatt_notify(current_conn, &nus_svc.attrs[2],
				 msg, strlen(msg));
	if (err) {
		LOG_WRN("NUS: Failed to send response (err %d)", err);
	}
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
#ifdef BLE_NO_PAIRING
		     BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#else
		     BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN),
#endif

	/* Environment data characteristic (notify only) */
	BT_GATT_CHARACTERISTIC(&env_char_uuid.uuid,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),
	BT_GATT_CCC(env_ccc_changed,
#ifdef BLE_NO_PAIRING
		     BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#else
		     BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN),
#endif
);

/* ──────────────────────────────────────────────
 * Security Callbacks
 * 
 * Handle pairing, passkey display, authentication,
 * and bonding events.
 * ────────────────────────────────────────────── */

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	/* Some platforms (like Windows) need explicit confirmation
	 * Auto-confirm since we're displaying the passkey */
	LOG_INF("Passkey confirm request: %06u (auto-confirming)", passkey);
	bt_conn_auth_passkey_confirm(conn);
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	LOG_INF("═══════════════════════════════════");
	LOG_INF("  PAIRING PIN: %06u", passkey);
	LOG_INF("═══════════════════════════════════");

	/* Store for validation */
	current_passkey = passkey;

	/* Send PIN to NUS UART terminal
	 * Keep message simple and clear for nRF Connect display */
	char pin_msg[64];
	int len = snprintk(pin_msg, sizeof(pin_msg),
		"PIN: %06u\r\n", passkey);

	/* Try to send immediately if NUS is enabled */
	if (nus_tx_enabled && current_conn) {
		int err = bt_gatt_notify(current_conn, &nus_svc.attrs[2],
				         pin_msg, len);
		if (err) {
			LOG_WRN("Failed to send PIN via NUS (err %d)", err);
		} else {
			LOG_INF("PIN sent via NUS");
		}
	} else {
		LOG_WRN("NUS not enabled yet - PIN only visible in logs");
		LOG_INF("Enable notifications on Nordic UART Service TX to see PIN");
	}

	/* Post event to FSM */
	if (post_event) {
		fsm_event_t evt = {
			.type = EVT_PAIRING_REQUEST,
			.data.passkey = passkey,
		};
		post_event(&evt);
	}
}

static void auth_cancel(struct bt_conn *conn)
{
	LOG_WRN("Pairing cancelled");
	
	if (post_event) {
		fsm_event_t evt = { .type = EVT_PAIRING_FAILED };
		post_event(&evt);
	}
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	LOG_INF("Pairing %s (bonded: %s)",
		bonded ? "successful" : "completed",
		bonded ? "yes" : "no");

	if (post_event) {
		fsm_event_t evt = {
			.type = EVT_PAIRING_COMPLETE,
			.data.bonded = bonded,
		};
		post_event(&evt);
	}
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	LOG_ERR("Pairing failed: %d", reason);

	if (post_event) {
		fsm_event_t evt = {
			.type = EVT_PAIRING_FAILED,
			.data.security_error = reason,
		};
		post_event(&evt);
	}
}

static struct bt_conn_auth_cb auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed,
};

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
    current_conn = bt_conn_ref(conn);
    notifications_enabled = false;

    /* Request short supervision timeout so we detect dead connections quickly */
    struct bt_le_conn_param param = {
        .interval_min = 6,    /* 7.5ms */
        .interval_max = 12,   /* 15ms */
        .latency      = 0,
        .timeout      = 100,  /* 1 second (units of 10ms) */
    };
    int param_err = bt_conn_le_param_update(conn, &param);
    if (param_err) {
        LOG_WRN("BLE: Failed to update conn params (err %d)", param_err);
    }

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

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
			     uint16_t latency, uint16_t timeout)
{
	LOG_INF("BLE: Conn params updated — interval=%u (%.2f ms) "
		"latency=%u timeout=%u (%u ms)",
		interval, interval * 1.25,
		latency,
		timeout, timeout * 10);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
	.le_param_updated = le_param_updated,
};

/* ──────────────────────────────────────────────
 * Advertising Data
 *
 * ad[] is static (flags + UUID don't change).
 * sd[] (scan response with name) is rebuilt each
 * time advertising starts, using name_store_get().
 * ────────────────────────────────────────────── */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS,
		      BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
		      0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
		      0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12),
};

/* Mutable scan response — updated by ble_start_advertising() */
static struct bt_data sd[1];

/* ──────────────────────────────────────────────
 * BLE Ready Callback
 * 
 * Called when BLE stack is ready (after settings loaded).
 * Required when CONFIG_BT_SETTINGS=y is enabled.
 * 
 * This posts an event to the FSM rather than blocking.
 * ────────────────────────────────────────────── */
static void bt_ready_callback(int err)
{
	if (err) {
		LOG_ERR("BLE: bt_ready failed (err %d)", err);
		if (post_event) {
			fsm_event_t evt = {
				.type = EVT_INIT_FAILED,
				.data.error_code = err,
			};
			post_event(&evt);
		}
		return;
	}

	LOG_INF("BLE: Stack ready, loading settings...");
	
	/* Load settings from flash (bonds, etc.) */
	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		settings_load();
	}
	
	LOG_INF("BLE: Settings loaded, stack ready");
	
	/* Post event to FSM that BLE is ready */
	if (post_event) {
		fsm_event_t evt = { .type = EVT_BLE_READY };
		post_event(&evt);
	}
}

/* ──────────────────────────────────────────────
 * Public API — called by FSM from main thread
 * ────────────────────────────────────────────── */

int ble_init(event_post_fn callback)
{
	int err;

	post_event = callback;

	/* Initialize settings subsystem first (required for bonding) */
	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		err = settings_subsys_init();
		if (err) {
			LOG_ERR("BLE: Settings subsystem init failed (err %d)", err);
			return err;
		}
		LOG_INF("BLE: Settings subsystem initialized");
	}

	/* Initialize name store — sets default, will be updated
	 * when settings_load() fires in bt_ready_callback()    */
	name_store_init();

	/* Enable BLE stack - settings will load asynchronously */
	err = bt_enable(bt_ready_callback);
	if (err) {
		LOG_ERR("BLE: bt_enable failed (err %d)", err);
		return err;
	}

#ifndef BLE_NO_PAIRING
	/* Register security callbacks for pairing/bonding */
	err = bt_conn_auth_cb_register(&auth_callbacks);
	if (err) {
		LOG_ERR("BLE: Failed to register auth callbacks (err %d)", err);
		return err;
	}

	err = bt_conn_auth_info_cb_register(&auth_info_callbacks);
	if (err) {
		LOG_ERR("BLE: Failed to register auth info callbacks (err %d)", err);
		return err;
	}
#else
	LOG_WRN("BLE: Pairing DISABLED (BLE_NO_PAIRING) — open access for testing");
#endif

	LOG_INF("BLE: Stack initializing (waiting for settings to load)...");
	return 0;
}

int ble_start_advertising(void)
{
	int err;

	/* Build scan response with current name */
	const char *name = name_store_get();
	sd[0].type     = BT_DATA_NAME_COMPLETE;
	sd[0].data_len = strlen(name);
	sd[0].data     = (const uint8_t *)name;

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

	LOG_INF("BLE: Advertising started (\"%s\"%s)", name,
		name_store_is_custom() ? " [custom]" : "");
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
	nus_tx_enabled = false;
	current_passkey = 0;

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
 * TX Shared Constants and Helper
 * ────────────────────────────────────────────── */
#define MAX_PAYLOAD         244
#define ENOMEM_RETRY_MAX    3
#define ENOMEM_RETRY_MS     50
#define INTER_PACKET_MS     10

/* Encode the 8-byte burst_header_t into buf (little-endian) */
static void encode_header(uint8_t *buf, uint8_t type, uint16_t seq,
			  uint16_t count, uint16_t chunk_index)
{
	buf[0] = type;
	buf[1] = 0x00;
	buf[2] = (uint8_t)(seq & 0xFF);
	buf[3] = (uint8_t)(seq >> 8);
	buf[4] = (uint8_t)(count & 0xFF);
	buf[5] = (uint8_t)(count >> 8);
	buf[6] = (uint8_t)(chunk_index & 0xFF);
	buf[7] = (uint8_t)(chunk_index >> 8);
}

/* Single notification with bounded ENOMEM retry */
static int notify_with_retry(const struct bt_gatt_attr *attr,
			     const void *data, uint16_t len)
{
	for (int retry = 0; retry <= ENOMEM_RETRY_MAX; retry++) {
		int err = bt_gatt_notify(current_conn, attr, data, len);
		if (err == 0) return 0;
		if (err == -ENOMEM && retry < ENOMEM_RETRY_MAX) {
			k_msleep(ENOMEM_RETRY_MS);
			continue;
		}
		return err;
	}
	return -ENOMEM;
}

/* ──────────────────────────────────────────────
 * ble_transmit_stats — time-domain feature packet
 *
 * Wire: header (8) + time_stats_t (36) = 44 bytes, single notification.
 * PKT_TYPE_TIME_STATS.
 * ────────────────────────────────────────────── */
int ble_transmit_stats(uint16_t seq, uint16_t count,
		       const time_stats_t *stats)
{
	if (!current_conn) return -ENOTCONN;
	if (!stats)        return -EINVAL;

	uint8_t pkt[8 + 36];
	encode_header(pkt, PKT_TYPE_TIME_STATS, seq, count, 0);

	uint8_t *p = pkt + 8;
	/* rms */
	p[0]=stats->rms_x&0xFF;  p[1]=stats->rms_x>>8;
	p[2]=stats->rms_y&0xFF;  p[3]=stats->rms_y>>8;
	p[4]=stats->rms_z&0xFF;  p[5]=stats->rms_z>>8;
	/* peak (signed — cast via uint16_t to avoid sign-extension warnings) */
	p[6]=(uint8_t)((uint16_t)stats->peak_x&0xFF);  p[7]=(uint8_t)((uint16_t)stats->peak_x>>8);
	p[8]=(uint8_t)((uint16_t)stats->peak_y&0xFF);  p[9]=(uint8_t)((uint16_t)stats->peak_y>>8);
	p[10]=(uint8_t)((uint16_t)stats->peak_z&0xFF); p[11]=(uint8_t)((uint16_t)stats->peak_z>>8);
	/* crest */
	p[12]=stats->crest_x&0xFF; p[13]=stats->crest_x>>8;
	p[14]=stats->crest_y&0xFF; p[15]=stats->crest_y>>8;
	p[16]=stats->crest_z&0xFF; p[17]=stats->crest_z>>8;
	/* kurtosis (signed) */
	p[18]=(uint8_t)((uint16_t)stats->kurtosis_x&0xFF); p[19]=(uint8_t)((uint16_t)stats->kurtosis_x>>8);
	p[20]=(uint8_t)((uint16_t)stats->kurtosis_y&0xFF); p[21]=(uint8_t)((uint16_t)stats->kurtosis_y>>8);
	p[22]=(uint8_t)((uint16_t)stats->kurtosis_z&0xFF); p[23]=(uint8_t)((uint16_t)stats->kurtosis_z>>8);
	/* variance */
	p[24]=stats->variance_x&0xFF; p[25]=stats->variance_x>>8;
	p[26]=stats->variance_y&0xFF; p[27]=stats->variance_y>>8;
	p[28]=stats->variance_z&0xFF; p[29]=stats->variance_z>>8;
	/* reserved + sample_count */
	p[30]=0; p[31]=0;
	p[32]=(uint8_t)(stats->sample_count&0xFF);
	p[33]=(uint8_t)((stats->sample_count>>8)&0xFF);
	p[34]=(uint8_t)((stats->sample_count>>16)&0xFF);
	p[35]=(uint8_t)((stats->sample_count>>24)&0xFF);

	int err = notify_with_retry(&vibration_svc.attrs[2], pkt, sizeof(pkt));
	if (err) {
		LOG_ERR("BLE TX: time stats failed (err %d)", err);
		return err;
	}

	LOG_INF("BLE TX: time stats seq=%u rms=[%u,%u,%u]mg "
		"crest=[%u,%u,%u] kurt=[%d,%d,%d]/100",
		seq,
		stats->rms_x, stats->rms_y, stats->rms_z,
		stats->crest_x, stats->crest_y, stats->crest_z,
		stats->kurtosis_x, stats->kurtosis_y, stats->kurtosis_z);
	return 0;
}

/* ──────────────────────────────────────────────
 * ble_transmit_fft_stats — FFT feature packet
 *
 * Wire: header (8) + fft_stats_t (60) = 68 bytes, single notification.
 * PKT_TYPE_FFT_STATS.
 * ────────────────────────────────────────────── */
static void encode_axis_fft(uint8_t *p, const axis_fft_stats_t *a)
{
	p[0] =(uint8_t)(a->dom_freq_hz&0xFF); p[1] =(uint8_t)(a->dom_freq_hz>>8);
	p[2] =(uint8_t)(a->dom_mag&0xFF);     p[3] =(uint8_t)(a->dom_mag>>8);
	p[4] =(uint8_t)(a->total_power&0xFF); p[5] =(uint8_t)(a->total_power>>8);
	p[6] =(uint8_t)(a->bpfo_energy&0xFF); p[7] =(uint8_t)(a->bpfo_energy>>8);
	p[8] =(uint8_t)(a->bpfi_energy&0xFF); p[9] =(uint8_t)(a->bpfi_energy>>8);
	p[10]=(uint8_t)(a->bsf_energy&0xFF);  p[11]=(uint8_t)(a->bsf_energy>>8);
	p[12]=(uint8_t)(a->ftf_energy&0xFF);  p[13]=(uint8_t)(a->ftf_energy>>8);
	p[14]=(uint8_t)(a->noise_floor&0xFF); p[15]=(uint8_t)(a->noise_floor>>8);
	p[16]=(uint8_t)(a->snr_bpfo&0xFF);    p[17]=(uint8_t)(a->snr_bpfo>>8);
	p[18]=0; p[19]=0;
}

int ble_transmit_fft_stats(uint16_t seq, uint16_t count,
			   const fft_stats_t *stats)
{
	if (!current_conn) return -ENOTCONN;
	if (!stats)        return -EINVAL;

	uint8_t pkt[8 + 60];
	encode_header(pkt, PKT_TYPE_FFT_STATS, seq, count, 0);
	encode_axis_fft(pkt + 8,      &stats->x);
	encode_axis_fft(pkt + 8 + 20, &stats->y);
	encode_axis_fft(pkt + 8 + 40, &stats->z);

	int err = notify_with_retry(&vibration_svc.attrs[2], pkt, sizeof(pkt));
	if (err) {
		LOG_ERR("BLE TX: FFT stats failed (err %d)", err);
		return err;
	}

	LOG_INF("BLE TX: FFT stats seq=%u "
		"X[dom=%uHz bpfo=%u snr=%u] "
		"Y[dom=%uHz bpfo=%u] "
		"Z[dom=%uHz bpfo=%u]",
		seq,
		stats->x.dom_freq_hz, stats->x.bpfo_energy, stats->x.snr_bpfo,
		stats->y.dom_freq_hz, stats->y.bpfo_energy,
		stats->z.dom_freq_hz, stats->z.bpfo_energy);
	return 0;
}

/* ──────────────────────────────────────────────
 * ble_transmit_burst_raw — optional raw sample dump
 *
 * Chunked: header (8) + up to 234 bytes of samples per packet.
 * 234 / 6 = 39 samples/packet → 512 samples = 14 packets.
 * PKT_TYPE_RAW.  Only used when SEND_RAW_BURST is enabled in fsm.c.
 * ────────────────────────────────────────────── */
#define RAW_SAMPLES_PER_PKT  ((MAX_PAYLOAD - 8) / sizeof(accel_sample_t))

int ble_transmit_burst_raw(const accel_sample_t *buffer, uint16_t count,
			   uint16_t seq)
{
	if (!current_conn)          return -ENOTCONN;
	if (!buffer || count == 0)  return -EINVAL;

	uint16_t n_packets = (count + RAW_SAMPLES_PER_PKT - 1) / RAW_SAMPLES_PER_PKT;

	LOG_INF("BLE TX: raw burst seq=%u count=%u → %u packets",
		seq, count, n_packets);

	/* Stack-allocated packet buffer — 8 header + up to 234 payload */
	uint8_t pkt[8 + RAW_SAMPLES_PER_PKT * sizeof(accel_sample_t)];

	for (uint16_t chunk = 0; chunk < n_packets; chunk++) {
		if (!current_conn) return -ENOTCONN;

		uint16_t offset     = chunk * RAW_SAMPLES_PER_PKT;
		uint16_t this_count = MIN((uint16_t)RAW_SAMPLES_PER_PKT,
					  count - offset);
		uint16_t payload_bytes = this_count * sizeof(accel_sample_t);

		encode_header(pkt, PKT_TYPE_RAW, seq, count, chunk);
		memcpy(pkt + 8, &buffer[offset], payload_bytes);

		int err = notify_with_retry(&vibration_svc.attrs[2],
					    pkt, 8 + payload_bytes);
		if (err) {
			LOG_ERR("BLE TX: raw chunk %u/%u failed (err %d)",
				chunk, n_packets, err);
			return err;
		}
		if (chunk < n_packets - 1) k_msleep(INTER_PACKET_MS);
	}

	LOG_INF("BLE TX: raw burst complete (%u packets)", n_packets);
	return 0;
}

/* ──────────────────────────────────────────────
 * ble_transmit_environment
 *
 * Wire: header (8) + temp(2) + hum(2) + pressure(4) = 16 bytes.
 * PKT_TYPE_ENV.
 * FIX vs original: pressure is uint32_t (Pa), encoded as 4 bytes.
 * ────────────────────────────────────────────── */
int ble_transmit_environment(const env_reading_t *reading)
{
	if (!current_conn)   return -ENOTCONN;
	if (!reading->valid) return -EINVAL;

	uint8_t pkt[8 + 8];
	encode_header(pkt, PKT_TYPE_ENV, 0, 0, 0);

	uint8_t *p = pkt + 8;
	p[0]=(uint8_t)((uint16_t)reading->temperature&0xFF);
	p[1]=(uint8_t)((uint16_t)reading->temperature>>8);
	p[2]=(uint8_t)(reading->humidity&0xFF);
	p[3]=(uint8_t)(reading->humidity>>8);
	p[4]=(uint8_t)(reading->pressure&0xFF);
	p[5]=(uint8_t)((reading->pressure>>8)&0xFF);
	p[6]=(uint8_t)((reading->pressure>>16)&0xFF);
	p[7]=(uint8_t)((reading->pressure>>24)&0xFF);

	int err = notify_with_retry(&vibration_svc.attrs[5], pkt, sizeof(pkt));
	if (err) {
		LOG_ERR("BLE TX: env notify failed (err %d)", err);
		return err;
	}

	LOG_DBG("BLE TX: env T=%d.%02d°C H=%d.%02d%% P=%uPa",
		reading->temperature / 100,
		abs(reading->temperature % 100),
		reading->humidity / 100,
		reading->humidity % 100,
		reading->pressure);
	return 0;
}

/* ──────────────────────────────────────────────
 * Security Helper Functions
 * ────────────────────────────────────────────── */

int ble_set_security(void)
{
#ifdef BLE_NO_PAIRING
	LOG_WRN("BLE: ble_set_security() skipped (BLE_NO_PAIRING)");
	return 0;
#else
	if (!current_conn) {
		return -ENOTCONN;
	}

	/* Request Security Level 3: Authenticated pairing (MITM protection) */
	int err = bt_conn_set_security(current_conn, BT_SECURITY_L3);
	if (err) {
		LOG_ERR("BLE: Failed to set security level (err %d)", err);
		return err;
	}

	LOG_INF("BLE: Security level 3 requested (authenticated pairing)");
	return 0;
#endif
}

bool ble_is_authenticated(void)
{
	if (!current_conn) {
		return false;
	}

	struct bt_conn_info info;
	int err = bt_conn_get_info(current_conn, &info);
	if (err) {
		return false;
	}

	/* Check if security level is at least L3 (authenticated) */
	return (info.security.level >= BT_SECURITY_L3);
}

int ble_delete_all_bonds(void)
{
	LOG_INF("BLE: Deleting all bonds...");
	
	/* This requires settings subsystem enabled */
	/* Implementation will iterate and delete all stored bonds */
	/* For now, just log - full implementation needs settings API */
	
	LOG_WRN("BLE: Bond deletion not yet implemented");
	return -ENOTSUP;
}