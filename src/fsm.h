/*
 * fsm.h — State machine types and event definitions
 *
 * This header defines the shared vocabulary between all modules:
 * states, event types, and the event object structure.
 */

#ifndef FSM_H
#define FSM_H

#include <stdint.h>
#include <stdbool.h>

/* ──────────────────────────────────────────────
 * FSM States
 * ────────────────────────────────────────────── */
typedef enum {
	STATE_INIT = 0,
	STATE_ADVERTISING,
	STATE_CONNECTED_IDLE,
	STATE_COLLECTING_VIBRATION,
	STATE_COLLECTING_ENVIRONMENT,
	STATE_TRANSMITTING,
	STATE_DISCONNECTED,
	STATE_RECOVERY,
	STATE_ERROR,
	STATE_COUNT  /* sentinel — number of states */
} fsm_state_t;

/* ──────────────────────────────────────────────
 * Event Types
 *
 * Grouped by source for readability.
 * The FSM switches on these to decide actions.
 * ────────────────────────────────────────────── */
typedef enum {
	/* System */
	EVT_INIT_COMPLETE = 0,
	EVT_INIT_FAILED,

	/* BLE */
	EVT_BLE_CONNECTED,
	EVT_BLE_DISCONNECTED,
	EVT_BLE_ADV_STARTED,
	EVT_BLE_ADV_FAILED,
	EVT_BLE_TX_COMPLETE,
	EVT_BLE_TX_FAILED,
	EVT_BLE_NOTIFICATIONS_ENABLED,
	EVT_BLE_NOTIFICATIONS_DISABLED,

	/* Sensor */
	EVT_VIBRATION_COLLECT_DONE,
	EVT_VIBRATION_COLLECT_FAILED,
	EVT_ENV_COLLECT_DONE,
	EVT_ENV_COLLECT_FAILED,

	/* Timers */
	EVT_BURST_TIMER_EXPIRED,
	EVT_CLEANUP_TIMEOUT,
	EVT_WATCHDOG_TIMEOUT,
	EVT_RETRY_TIMEOUT,

	EVT_COUNT  /* sentinel */
} fsm_event_type_t;

/* ──────────────────────────────────────────────
 * Event Payload Union
 *
 * Most events use zero or one field. The union
 * keeps the event struct fixed-size (no malloc).
 * Max payload: 8 bytes.
 * ────────────────────────────────────────────── */
typedef union {
	/* EVT_BLE_DISCONNECTED */
	struct {
		uint8_t reason;       /* BLE disconnect reason code */
	} ble_disconnect;

	/* EVT_BLE_TX_FAILED, EVT_INIT_FAILED, etc. */
	int16_t error_code;

	/* EVT_ENV_COLLECT_DONE (optional inline readings) */
	struct {
		int16_t temperature;  /* °C × 100 (e.g. 2350 = 23.50°C) */
		uint16_t humidity;    /* %RH × 100 */
		uint16_t pressure;    /* hPa (mbar) */
	} env;

	/* EVT_VIBRATION_COLLECT_DONE */
	struct {
		uint16_t sample_count;
	} vibration;

	/* Generic 8-byte payload for future use */
	uint8_t raw[8];

} fsm_event_data_t;

/* ──────────────────────────────────────────────
 * Event Object
 *
 * 16 bytes total, fixed size. Passed by value
 * through k_msgq. No pointers, no dynamic alloc.
 *
 * Layout:
 *   type      : 2 bytes  (what happened)
 *   _reserved : 2 bytes  (alignment / future flags)
 *   timestamp : 4 bytes  (when it happened)
 *   data      : 8 bytes  (context, if any)
 * ────────────────────────────────────────────── */
typedef struct {
	fsm_event_type_t  type;        /* What happened                */
	uint16_t          _reserved;   /* Padding / future flags       */
	uint32_t          timestamp;   /* k_uptime_get_32() at posting */
	fsm_event_data_t  data;        /* Payload (union, often unused)*/
} fsm_event_t;

/* Compile-time sanity check */
BUILD_ASSERT(sizeof(fsm_event_t) == 16, "Event struct must be 16 bytes");

/* ──────────────────────────────────────────────
 * Event posting function (implemented in main.c)
 *
 * This is the single entry point for all modules
 * to submit events. BLE callbacks, timer expiries,
 * and sensor completions all call this.
 * ────────────────────────────────────────────── */
typedef int (*event_post_fn)(fsm_event_t *evt);

int event_post(fsm_event_t *evt);

/* ──────────────────────────────────────────────
 * Timer control API (implemented in main.c)
 * Called by FSM transition logic.
 * ────────────────────────────────────────────── */
void timer_start_burst(void);
void timer_stop_burst(void);
void timer_start_cleanup(void);
void timer_stop_cleanup(void);
void timer_start_tx_watchdog(void);
void timer_stop_tx_watchdog(void);
void timer_start_retry(uint32_t delay_ms);
void timer_stop_retry(void);

/* ──────────────────────────────────────────────
 * Sensor types (shared across modules)
 * ────────────────────────────────────────────── */
#define VIBRATION_BURST_SIZE    512
#define SAMPLE_RATE_HZ          1600

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} __packed accel_sample_t;

typedef struct {
	int16_t  temperature;   /* °C × 100 */
	uint16_t humidity;      /* %RH × 100 */
	uint16_t pressure;      /* hPa */
	bool     valid;
} env_reading_t;

#endif /* FSM_H */
