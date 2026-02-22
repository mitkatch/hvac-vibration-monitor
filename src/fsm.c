/*
 * fsm.c — Finite State Machine Implementation (Table-Driven)
 *
 * Elegant table-driven FSM using state-event transition table.
 * Much cleaner than the long if-else ladder.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "fsm.h"
#include "ble.h"
#include "sensor.h"
#include "analysis.h"
#include "led.h"

LOG_MODULE_REGISTER(fsm, LOG_LEVEL_INF);

/* ──────────────────────────────────────────────
 * Configuration
 * ────────────────────────────────────────────── */
#define BURST_INTERVAL_SEC      10
#define CLEANUP_DELAY_MS        2500
#define TX_WATCHDOG_MS          5000
#define RETRY_BASE_MS           5000
#define RETRY_MAX_ATTEMPTS      3
#define EVENT_QUEUE_DEPTH       8
#define PAIRING_TIMEOUT_SEC     120

/* ──────────────────────────────────────────────
 * Event Queue
 * ────────────────────────────────────────────── */
K_MSGQ_DEFINE(event_queue, sizeof(fsm_event_t), EVENT_QUEUE_DEPTH, 4);

int event_post(fsm_event_t *evt)
{
	evt->timestamp = k_uptime_get_32();
	int err = k_msgq_put(&event_queue, evt, K_NO_WAIT);
	if (err) {
		LOG_ERR("Event queue full! Dropped event %d", evt->type);
	}
	return err;
}

/* ──────────────────────────────────────────────
 * Timers
 * ────────────────────────────────────────────── */
static void burst_timer_expiry(struct k_timer *timer);
static void cleanup_timer_expiry(struct k_timer *timer);
static void tx_watchdog_expiry(struct k_timer *timer);
static void retry_timer_expiry(struct k_timer *timer);
static void pairing_timeout_expiry(struct k_timer *timer);

K_TIMER_DEFINE(burst_timer,      burst_timer_expiry,      NULL);
K_TIMER_DEFINE(cleanup_timer,    cleanup_timer_expiry,    NULL);
K_TIMER_DEFINE(tx_watchdog,      tx_watchdog_expiry,      NULL);
K_TIMER_DEFINE(retry_timer,      retry_timer_expiry,      NULL);
K_TIMER_DEFINE(pairing_timeout,  pairing_timeout_expiry,  NULL);

static void burst_timer_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	fsm_event_t evt = { .type = EVT_BURST_TIMER_EXPIRED };
	event_post(&evt);
}

static void cleanup_timer_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	fsm_event_t evt = { .type = EVT_CLEANUP_TIMEOUT };
	event_post(&evt);
}

static void tx_watchdog_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	fsm_event_t evt = { .type = EVT_WATCHDOG_TIMEOUT };
	event_post(&evt);
}

static void retry_timer_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	fsm_event_t evt = { .type = EVT_RETRY_TIMEOUT };
	event_post(&evt);
}

static void pairing_timeout_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	fsm_event_t evt = { .type = EVT_PAIRING_TIMEOUT };
	event_post(&evt);
}

/* ──────────────────────────────────────────────
 * Timer Control (Internal)
 * ────────────────────────────────────────────── */
static void timer_start_burst(void)
{
	k_timer_start(&burst_timer, K_SECONDS(BURST_INTERVAL_SEC), K_NO_WAIT);
	LOG_DBG("Burst timer started (%d sec)", BURST_INTERVAL_SEC);
}

static void timer_stop_burst(void)
{
	k_timer_stop(&burst_timer);
}

static void timer_start_cleanup(void)
{
	k_timer_start(&cleanup_timer, K_MSEC(CLEANUP_DELAY_MS), K_NO_WAIT);
	LOG_DBG("Cleanup timer started (%d ms)", CLEANUP_DELAY_MS);
}

static void timer_stop_cleanup(void)
{
	k_timer_stop(&cleanup_timer);
}

static void timer_start_tx_watchdog(void)
{
	k_timer_start(&tx_watchdog, K_MSEC(TX_WATCHDOG_MS), K_NO_WAIT);
	LOG_DBG("TX watchdog started (%d ms)", TX_WATCHDOG_MS);
}

static void timer_stop_tx_watchdog(void)
{
	k_timer_stop(&tx_watchdog);
}

static void timer_start_retry(uint32_t delay_ms)
{
	k_timer_start(&retry_timer, K_MSEC(delay_ms), K_NO_WAIT);
	LOG_DBG("Retry timer started (%u ms)", delay_ms);
}

static void timer_stop_retry(void)
{
	k_timer_stop(&retry_timer);
}

static void timer_start_pairing_timeout(void)
{
	k_timer_start(&pairing_timeout, K_SECONDS(PAIRING_TIMEOUT_SEC), K_NO_WAIT);
	LOG_DBG("Pairing timeout started (%d sec)", PAIRING_TIMEOUT_SEC);
}

static void timer_stop_pairing_timeout(void)
{
	k_timer_stop(&pairing_timeout);
}

/* ──────────────────────────────────────────────
 * FSM State Variables
 * ────────────────────────────────────────────── */
static fsm_state_t current_state = STATE_INIT;
static uint8_t     retry_count   = 0;
static bool        is_connected     = false;
static bool        is_authenticated = false;

static accel_sample_t vibration_buffer[VIBRATION_BURST_SIZE];
static uint16_t       vibration_count = 0;
static env_reading_t  env_data;
static bool           env_sensor_present = false;

/* ──────────────────────────────────────────────
 * State Entry / Exit Actions
 * ────────────────────────────────────────────── */
static void state_exit(fsm_state_t old_state)
{
	switch (old_state) {
	case STATE_CONNECTED_IDLE:
		timer_stop_burst();
		timer_stop_pairing_timeout();
		break;
	case STATE_TRANSMITTING:
		timer_stop_tx_watchdog();
		break;
	case STATE_DISCONNECTED:
		timer_stop_cleanup();
		break;
	case STATE_RECOVERY:
		timer_stop_retry();
		break;
	default:
		break;
	}
}

static void state_enter(fsm_state_t new_state)
{
	switch (new_state) {
	case STATE_ADVERTISING:
		LOG_INF("→ ADVERTISING");
		retry_count = 0;
		break;
	case STATE_CONNECTED_IDLE:
		LOG_INF("→ CONNECTED_IDLE");
		timer_start_burst();
		break;
	case STATE_COLLECTING_VIBRATION:
		LOG_INF("→ COLLECTING_VIBRATION");
		break;
	case STATE_COLLECTING_ENVIRONMENT:
		LOG_INF("→ COLLECTING_ENVIRONMENT");
		break;
	case STATE_TRANSMITTING:
		LOG_INF("→ TRANSMITTING");
		timer_start_tx_watchdog();
		break;
	case STATE_DISCONNECTED:
		LOG_INF("→ DISCONNECTED");
		ble_cleanup();
		vibration_count = 0;
		timer_start_cleanup();
		break;
	case STATE_RECOVERY:
		LOG_INF("→ RECOVERY (attempt %d/%d)", retry_count + 1, RETRY_MAX_ATTEMPTS);
		timer_start_retry(RETRY_BASE_MS * (1 << retry_count));
		break;
	case STATE_ERROR:
		LOG_ERR("→ ERROR (unrecoverable)");
		break;
	default:
		break;
	}
}

static void transition(fsm_state_t new_state)
{
	if (new_state == current_state) {
		return;
	}
	state_exit(current_state);
	fsm_state_t old = current_state;
	current_state = new_state;
	state_enter(new_state);
	LOG_DBG("Transition: %d → %d", old, new_state);
}

/* ──────────────────────────────────────────────
 * Forward Declarations for Event Handlers
 * ────────────────────────────────────────────── */
typedef void (*fsm_handler_fn)(const fsm_event_t *evt);

static void handle_init_complete(const fsm_event_t *evt);
static void handle_init_failed(const fsm_event_t *evt);
static void handle_ble_ready(const fsm_event_t *evt);
static void handle_ble_connected(const fsm_event_t *evt);
static void handle_ble_disconnected(const fsm_event_t *evt);
static void handle_pairing_request(const fsm_event_t *evt);
static void handle_pairing_complete(const fsm_event_t *evt);
static void handle_pairing_failed(const fsm_event_t *evt);
static void handle_pairing_timeout(const fsm_event_t *evt);
static void handle_burst_timer(const fsm_event_t *evt);
static void handle_disconnect_during_collection(const fsm_event_t *evt);
static void handle_tx_disconnect(const fsm_event_t *evt);
static void handle_tx_watchdog(const fsm_event_t *evt);
static void handle_tx_complete(const fsm_event_t *evt);
static void handle_tx_failed(const fsm_event_t *evt);
static void handle_cleanup_timeout(const fsm_event_t *evt);
static void handle_reconnect_during_cleanup(const fsm_event_t *evt);
static void handle_duplicate_disconnect(const fsm_event_t *evt);
static void handle_retry_timeout(const fsm_event_t *evt);
static void handle_recovery_connect(const fsm_event_t *evt);

/* ──────────────────────────────────────────────
 * State-Event Transition Table
 *
 * The heart of the table-driven FSM.
 * Each row: (state, event) -> handler function
 * ────────────────────────────────────────────── */
typedef struct {
	fsm_state_t      state;
	fsm_event_type_t event;
	fsm_handler_fn   handler;
} fsm_transition_t;

static const fsm_transition_t transition_table[] = {
	/* STATE_INIT */
	{ STATE_INIT, EVT_INIT_COMPLETE, handle_init_complete },
	{ STATE_INIT, EVT_BLE_READY,     handle_ble_ready },
	{ STATE_INIT, EVT_INIT_FAILED,   handle_init_failed },

	/* STATE_ADVERTISING */
	{ STATE_ADVERTISING, EVT_BLE_CONNECTED, handle_ble_connected },

	/* STATE_CONNECTED_IDLE */
	{ STATE_CONNECTED_IDLE, EVT_PAIRING_REQUEST,     handle_pairing_request },
	{ STATE_CONNECTED_IDLE, EVT_PAIRING_COMPLETE,    handle_pairing_complete },
	{ STATE_CONNECTED_IDLE, EVT_PAIRING_FAILED,      handle_pairing_failed },
	{ STATE_CONNECTED_IDLE, EVT_PAIRING_TIMEOUT,     handle_pairing_timeout },
	{ STATE_CONNECTED_IDLE, EVT_BURST_TIMER_EXPIRED, handle_burst_timer },
	{ STATE_CONNECTED_IDLE, EVT_BLE_DISCONNECTED,    handle_ble_disconnected },

	/* STATE_COLLECTING_VIBRATION */
	{ STATE_COLLECTING_VIBRATION, EVT_BLE_DISCONNECTED, handle_disconnect_during_collection },

	/* STATE_COLLECTING_ENVIRONMENT */
	{ STATE_COLLECTING_ENVIRONMENT, EVT_BLE_DISCONNECTED, handle_disconnect_during_collection },

	/* STATE_TRANSMITTING */
	{ STATE_TRANSMITTING, EVT_BLE_DISCONNECTED,  handle_tx_disconnect },
	{ STATE_TRANSMITTING, EVT_WATCHDOG_TIMEOUT,  handle_tx_watchdog },
	{ STATE_TRANSMITTING, EVT_BLE_TX_COMPLETE,   handle_tx_complete },
	{ STATE_TRANSMITTING, EVT_BLE_TX_FAILED,     handle_tx_failed },

	/* STATE_DISCONNECTED */
	{ STATE_DISCONNECTED, EVT_CLEANUP_TIMEOUT,   handle_cleanup_timeout },
	{ STATE_DISCONNECTED, EVT_BLE_CONNECTED,     handle_reconnect_during_cleanup },
	{ STATE_DISCONNECTED, EVT_BLE_DISCONNECTED,  handle_duplicate_disconnect },

	/* STATE_RECOVERY */
	{ STATE_RECOVERY, EVT_RETRY_TIMEOUT,  handle_retry_timeout },
	{ STATE_RECOVERY, EVT_BLE_CONNECTED,  handle_recovery_connect },
};

#define TRANSITION_TABLE_SIZE (sizeof(transition_table) / sizeof(transition_table[0]))

/* ──────────────────────────────────────────────
 * FSM Event Processing (Table Lookup)
 * ────────────────────────────────────────────── */
static void fsm_process(const fsm_event_t *evt)
{
	LOG_DBG("FSM: state=%d event=%d", current_state, evt->type);

	/* Lookup handler in transition table */
	for (size_t i = 0; i < TRANSITION_TABLE_SIZE; i++) {
		if (transition_table[i].state == current_state &&
		    transition_table[i].event == evt->type) {
			/* Found matching transition - execute handler */
			transition_table[i].handler(evt);
			return;
		}
	}

	/* Event not handled in this state - this is normal for some events */
	LOG_DBG("Unhandled: event %d in state %d", evt->type, current_state);
}

/* ──────────────────────────────────────────────
 * Event Handler Implementations
 * 
 * Each handler is focused and clean - no nested if-else.
 * ────────────────────────────────────────────── */

static void handle_init_complete(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	LOG_INF("Hardware init complete, waiting for BLE stack...");
}

static void handle_init_failed(const fsm_event_t *evt)
{
	LOG_ERR("Init failed (err %d)", evt->data.error_code);
	transition(STATE_ERROR);
}

static void handle_ble_ready(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	LOG_INF("BLE stack ready, starting advertising");
	
	int err = ble_start_advertising();
	if (err) {
		LOG_ERR("Initial advertising failed (%d)", err);
		transition(STATE_ERROR);
	} else {
		transition(STATE_ADVERTISING);
	}
}

static void handle_ble_connected(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	is_connected = true;
	is_authenticated = false;
	LOG_INF("Connected - pairing will trigger when notifications requested");
	transition(STATE_CONNECTED_IDLE);
}

static void handle_ble_disconnected(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	transition(STATE_DISCONNECTED);
}

static void handle_pairing_request(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	timer_start_pairing_timeout();
	
	/* Start LED blinking - visual feedback that user needs to enter PIN */
	led_start_blinking(500);  /* Blink every 500ms (1 Hz) */
	
	LOG_INF("Pairing requested - PIN displayed, waiting for user (120 sec timeout)");
}

static void handle_pairing_complete(const fsm_event_t *evt)
{
	timer_stop_pairing_timeout();
	
	/* Stop LED blinking - pairing successful */
	led_stop_blinking();
	
	is_authenticated = true;
	
	if (evt->data.bonded) {
		LOG_INF("✓ Pairing successful - device bonded");
		LOG_INF("  Future connections will skip PIN entry");
	} else {
		LOG_INF("✓ Pairing successful (not bonded)");
	}
}

static void handle_pairing_failed(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	timer_stop_pairing_timeout();
	
	/* Stop LED blinking - pairing failed */
	led_stop_blinking();
	
	LOG_ERR("✗ Pairing failed - disconnecting");
	transition(STATE_DISCONNECTED);
}

static void handle_pairing_timeout(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	
	/* Stop LED blinking - timeout */
	led_stop_blinking();
	
	LOG_ERR("✗ Pairing timeout (120 sec) - disconnecting");
	transition(STATE_DISCONNECTED);
}

static void handle_burst_timer(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);

	/* Don't collect/transmit if notifications aren't enabled */
	if (!ble_notifications_enabled()) {
		LOG_INF("Notifications not enabled, skipping burst");
		timer_start_burst();
		return;
	}

	/* SECURITY: Verify device is authenticated before transmitting data */
#ifndef BLE_NO_PAIRING
	if (!ble_is_authenticated()) {
		LOG_WRN("Not authenticated - data transmission blocked");
		timer_start_burst();
		return;
	}
#endif

	transition(STATE_COLLECTING_VIBRATION);

	/* Synchronous collection — ~320 ms */
	int ret = sensor_collect_vibration(vibration_buffer, VIBRATION_BURST_SIZE);
	if (ret > 0) {
		vibration_count = ret;
		
		/* Collect environmental data if sensor present */
		if (env_sensor_present) {
			transition(STATE_COLLECTING_ENVIRONMENT);
			int env_ret = sensor_collect_environment(&env_data);
			if (env_ret < 0) {
				LOG_WRN("Env sensor read failed (%d), continuing without", env_ret);
			}
		}
		
		/* Transmit collected data */
		transition(STATE_TRANSMITTING);
		
		int tx_err = ble_transmit_burst(vibration_buffer, vibration_count);

		/* Send environmental data after vibration burst */
		if (tx_err == 0 && env_sensor_present) {
			if (env_data.valid) {
				int env_tx_err = ble_transmit_environment(&env_data);
				if (env_tx_err < 0) {
					LOG_WRN("Env TX failed (%d), continuing", env_tx_err);
				}
			} else {
				LOG_WRN("Env data invalid, skipping TX");
			}
		} else if (!env_sensor_present) {
			LOG_DBG("No env sensor — skipping env TX");
		}

		fsm_event_t tx_evt;
		if (tx_err == 0) {
			tx_evt.type = EVT_BLE_TX_COMPLETE;
		} else {
			tx_evt.type = EVT_BLE_TX_FAILED;
			tx_evt.data.error_code = tx_err;
		}
		event_post(&tx_evt);
		
	} else {
		LOG_ERR("Vibration collection failed (%d)", ret);
		transition(STATE_CONNECTED_IDLE);
	}
}

static void handle_disconnect_during_collection(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	transition(STATE_DISCONNECTED);
}

static void handle_tx_disconnect(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	LOG_WRN("Disconnect during transmission");
	transition(STATE_DISCONNECTED);
}

static void handle_tx_watchdog(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	LOG_WRN("TX watchdog fired — transmission stalled");
	transition(STATE_DISCONNECTED);
}

static void handle_tx_complete(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	transition(STATE_CONNECTED_IDLE);
}

static void handle_tx_failed(const fsm_event_t *evt)
{
	LOG_ERR("TX failed (err %d)", evt->data.error_code);
	transition(STATE_DISCONNECTED);
}

static void handle_cleanup_timeout(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	
	/* Clear connection status flags */
	is_connected = false;
	is_authenticated = false;
	
	int err = ble_start_advertising();
	if (err) {
		LOG_ERR("Post-cleanup advertising failed (%d)", err);
		transition(STATE_RECOVERY);
	} else {
		transition(STATE_ADVERTISING);
	}
}

static void handle_reconnect_during_cleanup(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	is_connected = true;
	is_authenticated = false;
	transition(STATE_CONNECTED_IDLE);
}

static void handle_duplicate_disconnect(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	LOG_DBG("Duplicate disconnect event, ignoring");
}

static void handle_retry_timeout(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	
	retry_count++;
	if (retry_count > RETRY_MAX_ATTEMPTS) {
		LOG_ERR("Max retries exceeded");
		transition(STATE_ERROR);
		return;
	}

	int err = ble_start_advertising();
	if (err) {
		LOG_WRN("Retry %d failed (err %d)", retry_count, err);
		/* Stay in RECOVERY with exponential backoff */
		timer_start_retry(RETRY_BASE_MS * (1 << retry_count));
	} else {
		transition(STATE_ADVERTISING);
	}
}

static void handle_recovery_connect(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	is_connected = true;
	is_authenticated = false;
	transition(STATE_CONNECTED_IDLE);
}

/* ──────────────────────────────────────────────
 * Public API
 * ────────────────────────────────────────────── */
void fsm_init(bool env_sensor_available)
{
	env_sensor_present = env_sensor_available;
	LOG_INF("FSM initialized (env_sensor: %s)", 
		env_sensor_present ? "present" : "absent");
}

void fsm_handle_event(const fsm_event_t *evt)
{
	uint32_t age_ms = k_uptime_get_32() - evt->timestamp;
	if (age_ms > 1000) {
		LOG_WRN("Stale event %d (age %u ms)", evt->type, age_ms);
	}
	fsm_process(evt);
}

struct k_msgq *fsm_get_event_queue(void)
{
	return &event_queue;
}
