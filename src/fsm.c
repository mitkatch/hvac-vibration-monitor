/*
 * fsm.c — Finite State Machine (Thread-only transport)
 *
 * Burst pipeline:
 *
 *   THREAD_IDLE
 *   → EVT_BURST_TIMER_EXPIRED  → COLLECTING_VIBRATION
 *                                  (work item fills ring slot)
 *   → EVT_VIBRATION_COLLECTED  → COLLECTING_ENVIRONMENT (or skip)
 *   → EVT_ENV_COLLECTED        → THREAD_TRANSMITTING
 *                                  (CoAP NON POST via thread_publish_burst)
 *   → EVT_THREAD_TX_COMPLETE   → THREAD_IDLE (consume ring slot)
 *
 * Thread network events drive the outer loop:
 *   EVT_THREAD_CONNECTED    → THREAD_JOINING → THREAD_IDLE
 *   EVT_THREAD_DISCONNECTED → any state     → THREAD_JOINING
 *
 * The ring buffer decouples collection from transmission.  Bursts accumulate
 * while Thread is rejoining; they drain as soon as the network is back.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "fsm.h"
#include "thread.h"
#include "sensor.h"
#include "analysis.h"
#include "vib_ring.h"

LOG_MODULE_REGISTER(fsm, LOG_LEVEL_INF);

/* ──────────────────────────────────────────────
 * Configuration
 * ────────────────────────────────────────────── */
#define BURST_INTERVAL_SEC   10
#define TX_WATCHDOG_MS       8000
#define EVENT_QUEUE_DEPTH    8

/* Set to 1 to send raw samples after stats (ML training data). */
#define SEND_RAW_BURST       0

/* ──────────────────────────────────────────────
 * Event Queue
 * ────────────────────────────────────────────── */
K_MSGQ_DEFINE(event_queue, sizeof(fsm_event_t), EVENT_QUEUE_DEPTH, 4);

int event_post(fsm_event_t *evt)
{
	evt->timestamp = k_uptime_get_32();
	int err = k_msgq_put(&event_queue, evt, K_NO_WAIT);
	if (err) {
		LOG_ERR("Event queue full — dropped event %d", evt->type);
	}
	return err;
}

/* ──────────────────────────────────────────────
 * Shared Sensor Buffers
 * ────────────────────────────────────────────── */
static vib_ring_t   vib_ring;
static vib_burst_t *active_write_slot = NULL;

static env_reading_t env_data;
static bool          env_sensor_present = false;

static time_stats_t  current_time_stats;
static fft_stats_t   current_fft_stats;

/* ──────────────────────────────────────────────
 * Work Item Contexts
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

static struct vib_work_ctx vib_work_ctx = { .ring    = &vib_ring };
static struct env_work_ctx env_work_ctx = { .reading = &env_data  };

static void vibration_work_handler(struct k_work *work)
{
	struct vib_work_ctx *ctx =
		CONTAINER_OF(work, struct vib_work_ctx, work);

	vib_burst_t *slot = vib_ring_acquire_write(ctx->ring);
	if (!slot) {
		fsm_event_t evt = { .type           = EVT_VIBRATION_FAILED,
				    .data.error_code = -ENOMEM };
		event_post(&evt);
		return;
	}

	int ret = sensor_collect_vibration(slot->samples, VIBRATION_BURST_SIZE);

	fsm_event_t evt;
	if (ret > 0) {
		slot->count  = (uint16_t)ret;
		vib_ring_commit_write(ctx->ring, slot);
		active_write_slot = slot;
		evt.type = EVT_VIBRATION_COLLECTED;
	} else {
		/* slot was never committed (vib_ring_commit_write not called),
		 * so head was never advanced — nothing to undo */
		LOG_ERR("vib_work: collection failed (%d)", ret);
		evt.type            = EVT_VIBRATION_FAILED;
		evt.data.error_code = ret;
	}
	event_post(&evt);
}

static void environment_work_handler(struct k_work *work)
{
	struct env_work_ctx *ctx =
		CONTAINER_OF(work, struct env_work_ctx, work);

	int ret = sensor_collect_environment(ctx->reading);
	if (ret < 0) {
		LOG_WRN("Env read failed (%d), continuing without", ret);
	}
	/* Always advance — env failure is non-fatal */
	fsm_event_t evt = { .type = EVT_ENV_COLLECTED };
	event_post(&evt);
}

/* ──────────────────────────────────────────────
 * Timers
 * ────────────────────────────────────────────── */
static void burst_timer_expiry(struct k_timer *t)
{
	ARG_UNUSED(t);
	fsm_event_t e = { .type = EVT_BURST_TIMER_EXPIRED };
	event_post(&e);
}

static void tx_watchdog_expiry(struct k_timer *t)
{
	ARG_UNUSED(t);
	fsm_event_t e = { .type = EVT_WATCHDOG_TIMEOUT };
	event_post(&e);
}

K_TIMER_DEFINE(burst_timer,  burst_timer_expiry,  NULL);
K_TIMER_DEFINE(tx_watchdog,  tx_watchdog_expiry,  NULL);

static void timer_start_burst(void)
{
	k_timer_start(&burst_timer, K_SECONDS(BURST_INTERVAL_SEC), K_NO_WAIT);
}
static void timer_stop_burst(void)      { k_timer_stop(&burst_timer); }
static void timer_start_watchdog(void)  { k_timer_start(&tx_watchdog, K_MSEC(TX_WATCHDOG_MS), K_NO_WAIT); }
static void timer_stop_watchdog(void)   { k_timer_stop(&tx_watchdog); }

/* ──────────────────────────────────────────────
 * FSM State
 * ────────────────────────────────────────────── */
static fsm_state_t current_state = STATE_INIT;

/* ──────────────────────────────────────────────
 * State Entry / Exit
 * ────────────────────────────────────────────── */
static void state_exit(fsm_state_t s)
{
	switch (s) {
	case STATE_THREAD_IDLE:
		timer_stop_burst();
		break;
	case STATE_THREAD_TRANSMITTING:
		timer_stop_watchdog();
		break;
	default:
		break;
	}
}

static void state_enter(fsm_state_t s)
{
	switch (s) {
	case STATE_THREAD_JOINING:
		LOG_INF("→ THREAD_JOINING");
		break;
	case STATE_THREAD_IDLE:
		LOG_INF("→ THREAD_IDLE (ring: %u bursts pending)",
			vib_ring_available(&vib_ring));
		timer_start_burst();
		break;
	case STATE_COLLECTING_VIBRATION:
		LOG_INF("→ COLLECTING_VIBRATION");
		break;
	case STATE_COLLECTING_ENVIRONMENT:
		LOG_INF("→ COLLECTING_ENVIRONMENT");
		break;
	case STATE_THREAD_TRANSMITTING:
		LOG_INF("→ THREAD_TRANSMITTING");
		timer_start_watchdog();
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
	LOG_DBG("Transition: %d → %d", current_state, new_state);
	state_exit(current_state);
	current_state = new_state;
	state_enter(new_state);
}

/* ──────────────────────────────────────────────
 * Thread Transmit Helper
 *
 * Peeks the oldest ring slot, runs analysis, and fires a CoAP NON POST.
 * The slot is NOT consumed here — handle_thread_tx_complete() does that
 * so the burst stays in the ring until the send succeeds.
 * ────────────────────────────────────────────── */
static int thread_transmit_oldest_burst(void)
{
	const vib_burst_t *burst = vib_ring_peek_read(&vib_ring);
	if (!burst) {
		LOG_WRN("TX: ring empty");
		return -ENODATA;
	}

	LOG_INF("TX: burst seq=%u count=%u ts=%ums (ring has %u more after this)",
		burst->seq, burst->count, burst->timestamp_ms,
		vib_ring_available(&vib_ring) - 1);

	int err = analysis_compute_all(burst->samples, burst->count,
				       &current_time_stats, &current_fft_stats);
	if (err) {
		LOG_ERR("TX: analysis failed (%d)", err);
		return err;
	}

	return thread_publish_burst(burst->seq, burst->count, burst->timestamp_ms,
				    &current_time_stats, &current_fft_stats,
				    env_data.valid ? &env_data : NULL);
}

/* ──────────────────────────────────────────────
 * Event Handler Forward Declarations
 * ────────────────────────────────────────────── */
typedef void (*fsm_handler_fn)(const fsm_event_t *evt);

static void handle_init_complete(const fsm_event_t *evt);
static void handle_init_failed(const fsm_event_t *evt);
static void handle_thread_connected(const fsm_event_t *evt);
static void handle_thread_disconnected(const fsm_event_t *evt);
static void handle_burst_timer(const fsm_event_t *evt);
static void handle_vibration_collected(const fsm_event_t *evt);
static void handle_vibration_failed(const fsm_event_t *evt);
static void handle_env_collected(const fsm_event_t *evt);
static void handle_thread_tx_complete(const fsm_event_t *evt);
static void handle_thread_tx_failed(const fsm_event_t *evt);
static void handle_thread_tx_watchdog(const fsm_event_t *evt);

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
	{ STATE_INIT, EVT_INIT_FAILED,   handle_init_failed   },

	/* STATE_THREAD_JOINING — wait for network */
	{ STATE_THREAD_JOINING, EVT_THREAD_CONNECTED, handle_thread_connected },

	/* STATE_THREAD_IDLE */
	{ STATE_THREAD_IDLE, EVT_BURST_TIMER_EXPIRED,  handle_burst_timer         },
	{ STATE_THREAD_IDLE, EVT_THREAD_DISCONNECTED,  handle_thread_disconnected },

	/* STATE_COLLECTING_VIBRATION */
	{ STATE_COLLECTING_VIBRATION, EVT_VIBRATION_COLLECTED,  handle_vibration_collected },
	{ STATE_COLLECTING_VIBRATION, EVT_VIBRATION_FAILED,     handle_vibration_failed    },
	{ STATE_COLLECTING_VIBRATION, EVT_THREAD_DISCONNECTED,  handle_thread_disconnected },

	/* STATE_COLLECTING_ENVIRONMENT */
	{ STATE_COLLECTING_ENVIRONMENT, EVT_ENV_COLLECTED,       handle_env_collected       },
	{ STATE_COLLECTING_ENVIRONMENT, EVT_THREAD_DISCONNECTED, handle_thread_disconnected },

	/* STATE_THREAD_TRANSMITTING */
	{ STATE_THREAD_TRANSMITTING, EVT_THREAD_TX_COMPLETE,  handle_thread_tx_complete },
	{ STATE_THREAD_TRANSMITTING, EVT_THREAD_TX_FAILED,    handle_thread_tx_failed   },
	{ STATE_THREAD_TRANSMITTING, EVT_WATCHDOG_TIMEOUT,    handle_thread_tx_watchdog },
	{ STATE_THREAD_TRANSMITTING, EVT_THREAD_DISCONNECTED, handle_thread_disconnected },
};

#define TRANSITION_TABLE_SIZE (sizeof(transition_table) / sizeof(transition_table[0]))

/* ──────────────────────────────────────────────
 * FSM Event Processing
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
 * Event Handler Implementations
 * ────────────────────────────────────────────── */

static void handle_init_complete(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	LOG_INF("Init complete — waiting for Thread network");
	transition(STATE_THREAD_JOINING);
}

static void handle_init_failed(const fsm_event_t *evt)
{
	LOG_ERR("Init failed (err %d)", evt->data.error_code);
	transition(STATE_ERROR);
}

static void handle_thread_connected(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	uint32_t pending = vib_ring_available(&vib_ring);
	if (pending > 0) {
		LOG_INF("Thread joined — %u buffered bursts to drain", pending);
	} else {
		LOG_INF("Thread joined");
	}
	transition(STATE_THREAD_IDLE);
}

/* EVT_THREAD_DISCONNECTED is handled uniformly from any state — always
 * return to JOINING and let OpenThread's internal retry mechanism reconnect. */
static void handle_thread_disconnected(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	LOG_WRN("Thread network lost — waiting to rejoin");
	/* Burst stays in ring; it will drain once the network is back */
	transition(STATE_THREAD_JOINING);
}

/* ── Burst pipeline ──────────────────────────────────────────────── */

static void handle_burst_timer(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);

	/* Drain any backlogged bursts before collecting a new one */
	if (!vib_ring_is_empty(&vib_ring)) {
		LOG_INF("Draining backlogged burst (ring has %u)",
			vib_ring_available(&vib_ring));
		transition(STATE_THREAD_TRANSMITTING);
		int err = thread_transmit_oldest_burst();
		fsm_event_t tx_evt;
		tx_evt.type = (err == 0) ? EVT_THREAD_TX_COMPLETE : EVT_THREAD_TX_FAILED;
		if (err) {
			tx_evt.data.error_code = err;
		}
		event_post(&tx_evt);
		return;
	}

	transition(STATE_COLLECTING_VIBRATION);
	k_work_submit(&vib_work_ctx.work);
}

static void handle_vibration_collected(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	LOG_INF("Vibration collected (ring now has %u committed bursts)",
		vib_ring_available(&vib_ring));

	if (env_sensor_present) {
		transition(STATE_COLLECTING_ENVIRONMENT);
		k_work_submit(&env_work_ctx.work);
		return;
	}

	transition(STATE_THREAD_TRANSMITTING);
	int err = thread_transmit_oldest_burst();
	fsm_event_t tx_evt;
	tx_evt.type = (err == 0) ? EVT_THREAD_TX_COMPLETE : EVT_THREAD_TX_FAILED;
	if (err) {
		tx_evt.data.error_code = err;
	}
	event_post(&tx_evt);
}

static void handle_vibration_failed(const fsm_event_t *evt)
{
	LOG_ERR("Vibration collection failed (err %d)", evt->data.error_code);
	transition(STATE_THREAD_IDLE);
}

static void handle_env_collected(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	LOG_INF("Environment collected — transmitting");

	transition(STATE_THREAD_TRANSMITTING);
	int err = thread_transmit_oldest_burst();
	fsm_event_t tx_evt;
	tx_evt.type = (err == 0) ? EVT_THREAD_TX_COMPLETE : EVT_THREAD_TX_FAILED;
	if (err) {
		tx_evt.data.error_code = err;
	}
	event_post(&tx_evt);
}

static void handle_thread_tx_complete(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	vib_ring_consume_read(&vib_ring);

	if (!vib_ring_is_empty(&vib_ring)) {
		/* Drain next backlogged burst quickly */
		k_timer_start(&burst_timer, K_MSEC(200), K_NO_WAIT);
	}
	transition(STATE_THREAD_IDLE);
}

static void handle_thread_tx_failed(const fsm_event_t *evt)
{
	LOG_ERR("Thread TX failed (err %d) — burst stays in ring for retry",
		evt->data.error_code);
	/* Do NOT consume ring slot — it stays for retry at next burst timer */
	transition(STATE_THREAD_IDLE);
}

static void handle_thread_tx_watchdog(const fsm_event_t *evt)
{
	ARG_UNUSED(evt);
	LOG_WRN("Thread TX watchdog — burst stays in ring");
	transition(STATE_THREAD_IDLE);
}

/* ──────────────────────────────────────────────
 * Public API
 * ────────────────────────────────────────────── */
void fsm_init(bool env_sensor_available)
{
	env_sensor_present = env_sensor_available;

	k_work_init(&vib_work_ctx.work, vibration_work_handler);
	k_work_init(&env_work_ctx.work, environment_work_handler);

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
