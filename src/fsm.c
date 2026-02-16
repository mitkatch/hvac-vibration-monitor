/*
 * fsm.c — Finite State Machine Implementation
 *
 * Central event-driven state machine for HVAC vibration monitoring.
 * Handles all state transitions, timer management, and event processing.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "fsm.h"
#include "ble.h"
#include "sensor.h"
#include "analysis.h"

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
#define PAIRING_TIMEOUT_SEC     120  /* 2 minutes for user to enter PIN */

/* ──────────────────────────────────────────────
 * Event Queue (central message bus)
 * ────────────────────────────────────────────── */
K_MSGQ_DEFINE(event_queue, sizeof(fsm_event_t), EVENT_QUEUE_DEPTH, 4);

/* Post an event to the queue (safe from ISR / callbacks) */
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
 *
 * Each timer is one-shot. Started on state entry,
 * stopped on state exit. The expiry callback only
 * posts an event — no real work done here.
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
 * Timer control API (called by FSM on transitions)
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

/* Connection status tracking */
static bool is_connected     = false;
static bool is_authenticated = false;

/* Vibration sample buffer — owned by the FSM */
static accel_sample_t vibration_buffer[VIBRATION_BURST_SIZE];
static uint16_t       vibration_count = 0;

/* Environmental readings — filled during COLLECTING_ENVIRONMENT */
static env_reading_t  env_data;
static bool           env_sensor_present = false;

/* Forward declarations */
static void fsm_do_transmit(void);

/* ──────────────────────────────────────────────
 * FSM State Entry / Exit Actions
 *
 * Encapsulates timer start/stop and resource
 * management for each state transition.
 * ────────────────────────────────────────────── */
static void state_exit(fsm_state_t old_state)
{
	switch (old_state) {
	case STATE_CONNECTED_IDLE:
		timer_stop_burst();
		timer_stop_pairing_timeout();  /* Stop pairing timeout if leaving */
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
		LOG_INF("→ RECOVERY (attempt %d/%d)", retry_count + 1,
			RETRY_MAX_ATTEMPTS);
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
 * FSM Event Processing
 *
 * Central dispatch: (current_state × event_type) → action + next state.
 * Events that are not valid in the current state are logged and ignored.
 * ────────────────────────────────────────────── */
static void fsm_process(const fsm_event_t *evt)
{
	LOG_DBG("FSM: state=%d event=%d", current_state, evt->type);

	switch (current_state) {

	/* ── INIT ───────────────────────────────── */
	case STATE_INIT:
		if (evt->type == EVT_INIT_COMPLETE) {
			/* Hardware init done, but BLE stack is still loading settings.
			 * Wait for EVT_BLE_READY before starting advertising. */
			LOG_INF("Hardware init complete, waiting for BLE stack...");
			
		} else if (evt->type == EVT_BLE_READY) {
			/* BLE stack has loaded settings, ready to advertise */
			LOG_INF("BLE stack ready, starting advertising");
			int err = ble_start_advertising();
			if (err) {
				LOG_ERR("Initial advertising failed (%d)", err);
				transition(STATE_ERROR);
			} else {
				transition(STATE_ADVERTISING);
			}
		} else if (evt->type == EVT_INIT_FAILED) {
			LOG_ERR("Init failed (err %d)", evt->data.error_code);
			transition(STATE_ERROR);
		}
		break;

	/* ── ADVERTISING ────────────────────────── */
	case STATE_ADVERTISING:
		if (evt->type == EVT_BLE_CONNECTED) {
			/* Go directly to CONNECTED_IDLE
			 * Pairing will be triggered automatically when iOS
			 * tries to enable notifications (GATT security) */
			is_connected = true;
			is_authenticated = false;  /* Will be set to true after pairing */
			LOG_INF("Connected - pairing will trigger when notifications requested");
			transition(STATE_CONNECTED_IDLE);
		}
		/* All other events ignored while advertising */
		break;

	/* ── CONNECTED_IDLE ─────────────────────── */
	case STATE_CONNECTED_IDLE:
		if (evt->type == EVT_PAIRING_REQUEST) {
			/* Pairing triggered by iOS trying to enable notifications
			 * Start 2-minute timeout for user to enter PIN */
			timer_start_pairing_timeout();
			LOG_INF("Pairing requested - PIN displayed, waiting for user (120 sec timeout)");
			/* Stay in CONNECTED_IDLE */
			
		} else if (evt->type == EVT_PAIRING_COMPLETE) {
			/* Authentication successful! */
			timer_stop_pairing_timeout();
			is_authenticated = true;  /* Mark as authenticated */
			
			if (evt->data.bonded) {
				LOG_INF("✓ Pairing successful - device bonded");
				LOG_INF("  Future connections will skip PIN entry");
			} else {
				LOG_INF("✓ Pairing successful (not bonded)");
			}
			/* Stay in CONNECTED_IDLE, ready for data transmission */
			
		} else if (evt->type == EVT_PAIRING_FAILED) {
			/* Wrong PIN or user cancelled */
			timer_stop_pairing_timeout();
			LOG_ERR("✗ Pairing failed - disconnecting");
			transition(STATE_DISCONNECTED);
			
		} else if (evt->type == EVT_PAIRING_TIMEOUT) {
			/* User took too long (2 minutes) */
			LOG_ERR("✗ Pairing timeout (120 sec) - disconnecting");
			transition(STATE_DISCONNECTED);
			
		} else if (evt->type == EVT_BURST_TIMER_EXPIRED) {
			/* Don't collect/transmit if notifications aren't enabled */
			if (!ble_notifications_enabled()) {
				LOG_INF("Notifications not enabled, skipping burst");
				timer_start_burst();  /* restart timer, try next cycle */
				break;
			}

			/* SECURITY: Verify device is authenticated before transmitting data */
			if (!ble_is_authenticated()) {
				LOG_WRN("Not authenticated - data transmission blocked");
				timer_start_burst();  /* restart timer, wait for auth */
				break;
			}

			transition(STATE_COLLECTING_VIBRATION);

			/* Synchronous collection — ~320 ms */
			int ret = sensor_collect_vibration(vibration_buffer,
							   VIBRATION_BURST_SIZE);
			if (ret > 0) {
				vibration_count = ret;
				if (env_sensor_present) {
					transition(STATE_COLLECTING_ENVIRONMENT);
					int env_ret = sensor_collect_environment(&env_data);
					if (env_ret < 0) {
						LOG_WRN("Env sensor read failed (%d), "
							"continuing without", env_ret);
					}
				}
				transition(STATE_TRANSMITTING);
				fsm_do_transmit();
			} else {
				LOG_ERR("Vibration collection failed (%d)", ret);
				transition(STATE_CONNECTED_IDLE);
			}

		} else if (evt->type == EVT_BLE_DISCONNECTED) {
			transition(STATE_DISCONNECTED);
		}
		break;

	/* ── COLLECTING_VIBRATION ───────────────── */
	/*
	 * Note: collection is synchronous so we don't normally receive
	 * events in this state. But if the BLE stack fires a disconnect
	 * callback while we're blocking in sensor_collect_vibration(),
	 * the event will be queued and processed after collection
	 * finishes. The transition logic in CONNECTED_IDLE handles
	 * the post-collection flow, so this state only needs to
	 * handle a disconnect that arrives between transitions.
	 */
	case STATE_COLLECTING_VIBRATION:
	case STATE_COLLECTING_ENVIRONMENT:
		if (evt->type == EVT_BLE_DISCONNECTED) {
			transition(STATE_DISCONNECTED);
		}
		break;

	/* ── TRANSMITTING ───────────────────────── */
	case STATE_TRANSMITTING:
		if (evt->type == EVT_BLE_DISCONNECTED) {
			LOG_WRN("Disconnect during transmission");
			transition(STATE_DISCONNECTED);

		} else if (evt->type == EVT_WATCHDOG_TIMEOUT) {
			LOG_WRN("TX watchdog fired — transmission stalled");
			transition(STATE_DISCONNECTED);

		} else if (evt->type == EVT_BLE_TX_COMPLETE) {
			transition(STATE_CONNECTED_IDLE);

		} else if (evt->type == EVT_BLE_TX_FAILED) {
			LOG_ERR("TX failed (err %d)", evt->data.error_code);
			transition(STATE_DISCONNECTED);
		}
		break;

	/* ── DISCONNECTED ───────────────────────── */
	case STATE_DISCONNECTED:
		if (evt->type == EVT_CLEANUP_TIMEOUT) {
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

		} else if (evt->type == EVT_BLE_CONNECTED) {
			/*
			 * New client connected before cleanup timer fired.
			 * BLE stack is ready if it accepted the connection.
			 */
			is_connected = true;
			is_authenticated = false;
			transition(STATE_CONNECTED_IDLE);

		} else if (evt->type == EVT_BLE_DISCONNECTED) {
			/* Duplicate disconnect — already handling it */
			LOG_DBG("Duplicate disconnect event, ignoring");
		}
		break;

	/* ── RECOVERY ───────────────────────────── */
	case STATE_RECOVERY:
		if (evt->type == EVT_RETRY_TIMEOUT) {
			retry_count++;
			if (retry_count > RETRY_MAX_ATTEMPTS) {
				LOG_ERR("Max retries exceeded");
				transition(STATE_ERROR);
				return;
			}

			int err = ble_start_advertising();
			if (err) {
				LOG_WRN("Retry %d failed (err %d)",
					retry_count, err);
				/* Stay in RECOVERY, start next retry timer
				 * with exponential backoff */
				timer_start_retry(
					RETRY_BASE_MS * (1 << retry_count));
			} else {
				transition(STATE_ADVERTISING);
			}

		} else if (evt->type == EVT_BLE_CONNECTED) {
			/* Unlikely but handle it */
			is_connected = true;
			is_authenticated = false;
			transition(STATE_CONNECTED_IDLE);
		}
		break;

	/* ── ERROR ──────────────────────────────── */
	case STATE_ERROR:
		/* Nothing to do. Device needs manual reset.
		 * Could add a long timer to attempt full re-init. */
		break;
	}
}

/* ──────────────────────────────────────────────
 * Transmission Helper
 *
 * Called synchronously from fsm_process when in
 * TRANSMITTING state. Posts TX_COMPLETE or TX_FAILED
 * back to itself so the FSM can transition cleanly.
 * ────────────────────────────────────────────── */
static void fsm_do_transmit(void)
{
	int err = ble_transmit_burst(vibration_buffer, vibration_count);

	fsm_event_t evt;
	if (err == 0) {
		evt.type = EVT_BLE_TX_COMPLETE;
	} else {
		evt.type = EVT_BLE_TX_FAILED;
		evt.data.error_code = err;
	}
	event_post(&evt);
}

/* ──────────────────────────────────────────────
 * Public API: FSM Initialization
 * ────────────────────────────────────────────── */
void fsm_init(bool env_sensor_available)
{
	env_sensor_present = env_sensor_available;
	LOG_INF("FSM initialized (env_sensor: %s)", 
		env_sensor_present ? "present" : "absent");
}

/* ──────────────────────────────────────────────
 * Public API: Process Event
 * ────────────────────────────────────────────── */
void fsm_handle_event(const fsm_event_t *evt)
{
	uint32_t age_ms = k_uptime_get_32() - evt->timestamp;
	if (age_ms > 1000) {
		LOG_WRN("Stale event %d (age %u ms)", evt->type, age_ms);
	}

	fsm_process(evt);
}

/* ──────────────────────────────────────────────
 * Public API: Get Event Queue
 * ────────────────────────────────────────────── */
struct k_msgq *fsm_get_event_queue(void)
{
	return &event_queue;
}
