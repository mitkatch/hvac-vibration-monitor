/*
 * fsm.c — Finite State Machine Implementation (Table-Driven)
 *
 * Async burst pipeline with ring buffer:
 *
 *   CONNECTED_IDLE
 *   → EVT_BURST_TIMER_EXPIRED  → COLLECTING_VIBRATION
 *                                  (work item: fills ring slot, runs analysis)
 *   → EVT_VIBRATION_COLLECTED  → COLLECTING_ENVIRONMENT (or skip)
 *   → EVT_ENV_COLLECTED        → TRANSMITTING
 *                                  (sends stats packet, optionally raw burst)
 *   → EVT_BLE_TX_COMPLETE      → CONNECTED_IDLE
 *
 * The ring buffer decouples collection from transmission:
 *   - The sensor keeps collecting even if hub is briefly unavailable
 *   - On reconnect the hub can drain up to VIB_RING_SLOTS backlogged bursts
 *   - Missed TX posts the burst back to the ring (not lost)
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "fsm.h"
#include "ble.h"
#include "sensor.h"
#include "analysis.h"
#include "vib_ring.h"
#include "led.h"

LOG_MODULE_REGISTER(fsm, LOG_LEVEL_INF);

/* ──────────────────────────────────────────────
 * Configuration
 * ────────────────────────────────────────────── */
#define BURST_INTERVAL_SEC      10
#define CLEANUP_DELAY_MS        2500
#define TX_WATCHDOG_MS          8000   /* increased: stats + env + optional raw */
#define RETRY_BASE_MS           5000
#define RETRY_MAX_ATTEMPTS      3
#define EVENT_QUEUE_DEPTH       8
#define PAIRING_TIMEOUT_SEC     120
#define IDLE_NO_NOTIFY_MAX      3

/* Set to 1 to also transmit raw samples after each stats packet.
 * Useful for training data collection.  0 for production (stats only). */
#define SEND_RAW_BURST          0

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
 * Shared Sensor Buffers
 *
 * Declared here (not extern-inside-function) — accessible by
 * work items via a pointer passed through a work context struct.
 * ────────────────────────────────────────────── */

/* The ring buffer — all burst slots live here */
static vib_ring_t vib_ring;

/* Current burst being written (valid while work item is running) */
static vib_burst_t *active_write_slot = NULL;

/* Environmental data — written by env work item, read by TX */
static env_reading_t env_data;
static bool          env_sensor_present = false;

/* Analysis results for the burst about to be transmitted */
static time_stats_t current_time_stats;
static fft_stats_t  current_fft_stats;

/* ──────────────────────────────────────────────
 * Work Item Context
 *
 * Work handlers need access to the ring pointer.
 * Using a typed container avoids the extern hack.
 * ────────────────────────────────────────────── */
struct vib_work_ctx {
	struct k_work work;
	vib_ring_t   *ring;
};

struct env_work_ctx {
	struct k_work  work;
	env_reading_t *reading;
};

static void vibration_work_handler(struct k_work *work);
static void environment_work_handler(struct k_work *work);

static struct vib_work_ctx vib_work_ctx = {
	.ring = &vib_ring,
};
static struct env_work_ctx env_work_ctx = {
	.reading = &env_data,
};

static void vibration_work_handler(struct k_work *work)
{
	struct vib_work_ctx *ctx =
		CONTAINER_OF(work, struct vib_work_ctx, work);

	/*
	 * Acquire a ring slot.
	 * If the ring is full, vib_ring_acquire_write evicts the oldest slot.
	 * active_write_slot is set here and used in the FSM's collect handler.
	 */
	vib_burst_t *slot = vib_ring_acquire_write(ctx->ring);
	if (!slot) {
		/* Should not happen — acquire always returns a slot */
		LOG_ERR("vib_work: failed to acquire ring slot");
		fsm_event_t evt = { .type = EVT_VIBRATION_FAILED,
				    .data.error_code = -ENOMEM };
		event_post(&evt);
		return;
	}

	/* Collect samples into the ring slot's sample array */
	int ret = sensor_collect_vibration(slot->samples, VIBRATION_BURST_SIZE);

	fsm_event_t evt;
	if (ret > 0) {
		slot->count = (uint16_t)ret;
		/* timestamp_ms was set by acquire_write */

		/* Commit the slot — now visible to the reader */
		vib_ring_commit_write(ctx->ring, slot);
		active_write_slot = slot;  /* FSM will read this in its handler */

		evt.type = EVT_VIBRATION_COLLECTED;
	} else {
		/*
		 * Collection failed — slot was acquired but not committed.
		 * Reset head to discard it.  Since only we touch head in the
		 * writer role, this is safe.
		 */
		atomic_dec(&ctx->ring->head);
		LOG_ERR("vib_work: collection failed (err %d)", ret);
		evt.type = EVT_VIBRATION_FAILED;
		evt.data.error_code = ret;
	}

	event_post(&evt);
}

static void environment_work_handler(struct k_work *work)
{
	struct env_work_ctx *ctx =
		CONTAINER_OF(work, struct env_work_ctx, work);

	int ret = sensor_collect_environment(ctx->reading);

	fsm_event_t evt;
	if (ret >= 0) {
		evt.type = EVT_ENV_COLLECTED;
	} else {
		/* Non-fatal: move to TX without env data */
		LOG_WRN("Env sensor read failed (%d), continuing without", ret);
		evt.type = EVT_ENV_COLLECTED;
	}
	event_post(&evt);
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

static void burst_timer_expiry(struct k_timer *t)   { ARG_UNUSED(t); fsm_event_t e = {.type=EVT_BURST_TIMER_EXPIRED}; event_post(&e); }
static void cleanup_timer_expiry(struct k_timer *t) { ARG_UNUSED(t); fsm_event_t e = {.type=EVT_CLEANUP_TIMEOUT};    event_post(&e); }
static void tx_watchdog_expiry(struct k_timer *t)   { ARG_UNUSED(t); fsm_event_t e = {.type=EVT_WATCHDOG_TIMEOUT};   event_post(&e); }
static void retry_timer_expiry(struct k_timer *t)   { ARG_UNUSED(t); fsm_event_t e = {.type=EVT_RETRY_TIMEOUT};      event_post(&e); }
static void pairing_timeout_expiry(struct k_timer *t) { ARG_UNUSED(t); fsm_event_t e = {.type=EVT_PAIRING_TIMEOUT}; event_post(&e); }

/* ──────────────────────────────────────────────
 * Timer Control (Internal)
 * ────────────────────────────────────────────── */
static void timer_start_burst(void)       { k_timer_start(&burst_timer, K_SECONDS(BURST_INTERVAL_SEC), K_NO_WAIT); }
static void timer_stop_burst(void)        { k_timer_stop(&burst_timer); }
static void timer_start_cleanup(void)     { k_timer_start(&cleanup_timer, K_MSEC(CLEANUP_DELAY_MS), K_NO_WAIT); }
static void timer_stop_cleanup(void)      { k_timer_stop(&cleanup_timer); }
static void timer_start_tx_watchdog(void) { k_timer_start(&tx_watchdog, K_MSEC(TX_WATCHDOG_MS), K_NO_WAIT); }
static void timer_stop_tx_watchdog(void)  { k_timer_stop(&tx_watchdog); }
static void timer_start_retry(uint32_t ms){ k_timer_start(&retry_timer, K_MSEC(ms), K_NO_WAIT); }
static void timer_stop_retry(void)        { k_timer_stop(&retry_timer); }
static void timer_start_pairing_timeout(void) { k_timer_start(&pairing_timeout, K_SECONDS(PAIRING_TIMEOUT_SEC), K_NO_WAIT); }
static void timer_stop_pairing_timeout(void)  { k_timer_stop(&pairing_timeout); }

/* ──────────────────────────────────────────────
 * FSM State Variables
 * ────────────────────────────────────────────── */
static fsm_state_t current_state   = STATE_INIT;
static uint8_t     retry_count     = 0;
static bool        is_connected    = false;
static bool        is_authenticated = false;
static uint8_t     idle_skip_count = 0;

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
		LOG_INF("→ ADVERTISING (ring: %u bursts pending)",
			vib_ring_available(&vib_ring));
		retry_count = 0;
		break;
	case STATE_CONNECTED_IDLE:
		LOG_INF("→ CONNECTED_IDLE (ring: %u bursts pending)",
			vib_ring_available(&vib_ring));
		idle_skip_count = 0;
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
		/* Do NOT clear the ring — preserve buffered bursts for next connection */
		timer_start_cleanup();
		break;
	case STATE_RECOVERY:
		LOG_INF("→ RECOVERY (attempt %d/%d)", retry_count + 1, RETRY_MAX_ATTEMPTS);
		timer_start_retry(RETRY_BASE_MS * (1u << retry_count));
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
	current_state   = new_state;
	state_enter(new_state);
	LOG_DBG("Transition: %d → %d", old, new_state);
}

/* ──────────────────────────────────────────────
 * Forward Declarations
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
static void handle_vibration_collected(const fsm_event_t *evt);
static void handle_vibration_failed(const fsm_event_t *evt);
static void handle_env_collected(const fsm_event_t *evt);
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
 * Transition Table
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
	{ STATE_COLLECTING_VIBRATION, EVT_VIBRATION_COLLECTED, handle_vibration_collected },
	{ STATE_COLLECTING_VIBRATION, EVT_VIBRATION_FAILED,    handle_vibration_failed },
	{ STATE_COLLECTING_VIBRATION, EVT_BLE_DISCONNECTED,    handle_disconnect_during_collection },

	/* STATE_COLLECTING_ENVIRONMENT */
	{ STATE_COLLECTING_ENVIRONMENT, EVT_ENV_COLLECTED,    handle_env_collected },
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

	for (size_t i = 0; i < TRANSITION_TABLE_SIZE; i++) {
		if (transition_table[i].state == current_state &&
		    transition_table[i].event == evt->type) {
			transition_table[i].handler(evt);
			return;
		}
	}

	LOG_DBG("Unhandled: event %d in state %d", evt->type, current_state);
}

/* ──────────────────────────────────────────────
 * Helper: transmit the oldest ring slot
 *
 * Sends (in order):
 *   1. Time-domain stats   PKT_TYPE_TIME_STATS  (44 bytes)  — critical, abort on fail
 *   2. FFT stats           PKT_TYPE_FFT_STATS   (68 bytes)  — critical, abort on fail
 *   3. Raw burst           PKT_TYPE_RAW         (chunked)   — optional, non-fatal
 *   4. Environment         PKT_TYPE_ENV         (16 bytes)  — non-fatal
 *
 * Analysis (~15 ms) runs before any BLE TX.
 * On TX success: consumes ring slot.
 * On TX failure: leaves slot in ring for retry on next connection.
 * ────────────────────────────────────────────── */
static int transmit_oldest_burst(void)
{
	const vib_burst_t *burst = vib_ring_peek_read(&vib_ring);
	if (!burst) {
		LOG_WRN("TX: no burst in ring to transmit");
		return -ENODATA;
	}

	LOG_INF("TX: burst seq=%u count=%u ts=%ums (ring has %u more after this)",
		burst->seq, burst->count, burst->timestamp_ms,
		vib_ring_available(&vib_ring) - 1);

	/* ── 1 & 2: Run full analysis (time + FFT) ── */
	int err = analysis_compute_all(burst->samples, burst->count,
				       &current_time_stats, &current_fft_stats);
	if (err) {
		LOG_ERR("TX: analysis failed (err %d)", err);
		return err;
	}

	/* ── 3. Send time-domain stats (1 notification, 44 bytes) ── */
	err = ble_transmit_stats(burst->seq, burst->count, &current_time_stats);
	if (err) {
		LOG_ERR("TX: time stats failed (err %d) — burst kept in ring", err);
		return err;
	}

	/* ── 4. Send FFT stats (1 notification, 68 bytes) ── */
	err = ble_transmit_fft_stats(burst->seq, burst->count, &current_fft_stats);
	if (err) {
		/* FFT stats TX failed but time stats already sent.
		 * Still consume — partial delivery is better than infinite retry. */
		LOG_WRN("TX: FFT stats failed (err %d) — consuming anyway", err);
	}

#if SEND_RAW_BURST
	/* ── 5. Raw samples (optional, for ML training data collection) ── */
	err = ble_transmit_burst_raw(burst->samples, burst->count, burst->seq);
	if (err) {
		LOG_WRN("TX: raw burst failed (err %d), non-fatal", err);
	}
#endif

	/* ── 6. Environment data ── */
	if (env_data.valid) {
		int env_err = ble_transmit_environment(&env_data);
		if (env_err < 0) {
			LOG_WRN("TX: env failed (err %d), non-fatal", env_err);
		}
	}

	/* ── 7. Consume the slot ── */
	vib_ring_consume_read(&vib_ring);
	return 0;
}

/* ──────────────────────────────────────────────
 * Event Handler Implementations
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
	LOG_INF("Connected");

	/* If ring has backlogged bursts, log it — they'll drain on next timer */
	uint32_t pending = vib_ring_available(&vib_ring);
	if (pending > 0) {
		LOG_INF("Ring has %u buffered bursts to transmit", pending);
	}

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
	led_start_blinking(500);
	LOG_INF("Pairing requested - PIN displayed, waiting for user (120 sec timeout)");
}

static void handle_pairing_complete(const fsm_event_t *evt)
{
	timer_stop_pairing_timeout();
	led_stop_blinking();
	is_authenticated = true;
	if (evt->data.bonded) {
		LOG_INF("✓ Pairing successful - device bonded");
	} else {
		LOG_INF("✓ Pairing successful (not bonded)");
	}
}

static void handle_pairing_failed(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	timer_stop_pairing_timeout();
	led_stop_blinking();
	LOG_ERR("✗ Pairing failed - disconnecting");
	transition(STATE_DISCONNECTED);
}

static void handle_pairing_timeout(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	led_stop_blinking();
	LOG_ERR("✗ Pairing timeout (120 sec) - disconnecting");
	transition(STATE_DISCONNECTED);
}

/* ──────────────────────────────────────────────
 * Async Burst Pipeline
 * ────────────────────────────────────────────── */

/* Stage 1 — kick off vibration collection */
static void handle_burst_timer(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);

	/* Handle any pending ring bursts first (backlog drain) */
	if (!vib_ring_is_empty(&vib_ring)) {
		if (!ble_notifications_enabled()) {
			LOG_WRN("Ring has %u pending bursts but notifications off",
				vib_ring_available(&vib_ring));
		} else {
#ifndef BLE_NO_PAIRING
			if (ble_is_authenticated()) {
#endif
				/* Drain one backlogged burst immediately */
				LOG_INF("Draining backlogged burst from ring");
				transition(STATE_TRANSMITTING);
				int err = transmit_oldest_burst();
				fsm_event_t tx_evt;
				tx_evt.type = (err == 0) ? EVT_BLE_TX_COMPLETE : EVT_BLE_TX_FAILED;
				if (err) tx_evt.data.error_code = err;
				event_post(&tx_evt);
				return;
#ifndef BLE_NO_PAIRING
			}
#endif
		}
	}

	if (!ble_notifications_enabled()) {
		idle_skip_count++;
		if (idle_skip_count >= IDLE_NO_NOTIFY_MAX) {
			LOG_WRN("Notifications disabled for %u consecutive bursts — forcing disconnect",
				idle_skip_count);
			transition(STATE_DISCONNECTED);
			return;
		}
		LOG_INF("Notifications not enabled, skipping burst (%u/%u)",
			idle_skip_count, IDLE_NO_NOTIFY_MAX);
		timer_start_burst();
		return;
	}

	idle_skip_count = 0;

#ifndef BLE_NO_PAIRING
	if (!ble_is_authenticated()) {
		LOG_WRN("Not authenticated - data transmission blocked");
		timer_start_burst();
		return;
	}
#endif

	transition(STATE_COLLECTING_VIBRATION);
	k_work_submit(&vib_work_ctx.work);
}

/* Stage 2 — vibration data ready */
static void handle_vibration_collected(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);

	uint32_t available = vib_ring_available(&vib_ring);
	LOG_INF("Vibration collected (ring now has %u committed bursts)", available);

	if (env_sensor_present) {
		transition(STATE_COLLECTING_ENVIRONMENT);
		k_work_submit(&env_work_ctx.work);
	} else {
		/* Go straight to TX */
		transition(STATE_TRANSMITTING);
		int err = transmit_oldest_burst();
		fsm_event_t tx_evt;
		tx_evt.type = (err == 0) ? EVT_BLE_TX_COMPLETE : EVT_BLE_TX_FAILED;
		if (err) tx_evt.data.error_code = err;
		event_post(&tx_evt);
	}
}

/* Stage 2b — vibration collection failed */
static void handle_vibration_failed(const fsm_event_t *evt)
{
	LOG_ERR("Vibration collection failed (err %d)", evt->data.error_code);
	transition(STATE_CONNECTED_IDLE);
}

/* Stage 3 — environment data ready, start TX */
static void handle_env_collected(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	LOG_INF("Environment collected, starting TX");

	transition(STATE_TRANSMITTING);
	int err = transmit_oldest_burst();

	fsm_event_t tx_evt;
	tx_evt.type = (err == 0) ? EVT_BLE_TX_COMPLETE : EVT_BLE_TX_FAILED;
	if (err) tx_evt.data.error_code = err;
	event_post(&tx_evt);
}

/* ──────────────────────────────────────────────
 * Disconnect / TX Handlers
 * ────────────────────────────────────────────── */
static void handle_disconnect_during_collection(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	/* Work item may still be running — harmless, it will commit to ring
	 * and post EVT_VIBRATION_COLLECTED which will be dropped (we're in
	 * DISCONNECTED by then). The slot stays in the ring for next connect. */
	LOG_WRN("Disconnect during collection — burst will be saved to ring if completed");
	transition(STATE_DISCONNECTED);
}

static void handle_tx_disconnect(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	/* Burst was peek'd but not consumed — it stays in ring for retry */
	LOG_WRN("Disconnect during transmission — burst retained in ring for retry");
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
	/* If more bursts are queued, schedule another burst immediately */
	if (!vib_ring_is_empty(&vib_ring)) {
		LOG_INF("Ring still has %u bursts — draining next",
			vib_ring_available(&vib_ring));
		/* Re-arm burst timer with minimal delay to drain backlog quickly */
		k_timer_start(&burst_timer, K_MSEC(200), K_NO_WAIT);
	}
	transition(STATE_CONNECTED_IDLE);
}

static void handle_tx_failed(const fsm_event_t *evt)
{
	LOG_ERR("TX failed (err %d) — burst stays in ring", evt->data.error_code);
	transition(STATE_DISCONNECTED);
}

static void handle_cleanup_timeout(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	is_connected     = false;
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
	is_connected     = true;
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
		timer_start_retry(RETRY_BASE_MS * (1u << retry_count));
	} else {
		transition(STATE_ADVERTISING);
	}
}

static void handle_recovery_connect(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	is_connected     = true;
	is_authenticated = false;
	transition(STATE_CONNECTED_IDLE);
}

/* ──────────────────────────────────────────────
 * Public API
 * ────────────────────────────────────────────── */
void fsm_init(bool env_sensor_available)
{
	env_sensor_present = env_sensor_available;

	/* Initialise work items */
	k_work_init(&vib_work_ctx.work, vibration_work_handler);
	k_work_init(&env_work_ctx.work, environment_work_handler);

	/* Initialise ring buffer */
	vib_ring_init(&vib_ring);

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
