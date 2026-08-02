/*
 * fsm.h — State machine types and event definitions (Thread-only transport)
 */

#ifndef FSM_H
#define FSM_H

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

/* ──────────────────────────────────────────────
 * Sensor Data Types
 * ────────────────────────────────────────────── */

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} accel_sample_t;

typedef struct {
	int16_t  temperature;  /* °C × 100 */
	uint16_t humidity;     /* % × 100  */
	uint32_t pressure;     /* Pascals  */
	bool     valid;
} env_reading_t;

#define VIBRATION_BURST_SIZE    512

/* ──────────────────────────────────────────────
 * System States
 * ────────────────────────────────────────────── */
typedef enum {
	STATE_INIT,
	STATE_THREAD_JOINING,       /* OpenThread scanning / attaching        */
	STATE_THREAD_IDLE,          /* Thread connected, waiting for timer    */
	STATE_COLLECTING_VIBRATION,
	STATE_COLLECTING_ENVIRONMENT,
	STATE_THREAD_TRANSMITTING,  /* CoAP NON POST in-flight                */
	STATE_ERROR,
} fsm_state_t;

/* ──────────────────────────────────────────────
 * Event Types
 * ────────────────────────────────────────────── */
typedef enum {
	EVT_INIT_COMPLETE,
	EVT_INIT_FAILED,

	/* Thread network events */
	EVT_THREAD_CONNECTED,       /* Role moved to Child / Router           */
	EVT_THREAD_DISCONNECTED,    /* Role dropped back to Detached          */

	/* Pipeline timing */
	EVT_BURST_TIMER_EXPIRED,

	/* Async collection pipeline */
	EVT_VIBRATION_COLLECTED,
	EVT_VIBRATION_FAILED,
	EVT_ENV_COLLECTED,

	/* Thread transmission */
	EVT_THREAD_TX_COMPLETE,
	EVT_THREAD_TX_FAILED,
	EVT_WATCHDOG_TIMEOUT,
} fsm_event_type_t;

/* ──────────────────────────────────────────────
 * Event Object
 * ────────────────────────────────────────────── */
typedef struct {
	fsm_event_type_t type;
	uint32_t         timestamp;

	union {
		int error_code;
	} data;
} fsm_event_t;

/* ──────────────────────────────────────────────
 * Event Posting Callback
 * ────────────────────────────────────────────── */
typedef int (*event_post_fn)(fsm_event_t *evt);

/* ──────────────────────────────────────────────
 * Public API
 * ────────────────────────────────────────────── */
void            fsm_init(bool env_sensor_available);
void            fsm_handle_event(const fsm_event_t *evt);
struct k_msgq  *fsm_get_event_queue(void);
int             event_post(fsm_event_t *evt);

#endif /* FSM_H */
