/*
 * fsm.h — State machine types and event definitions
 *
 * This header defines the shared vocabulary between all modules:
 * states, event types, and the event object structure.
 */

#ifndef FSM_H
#define FSM_H

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

/* ──────────────────────────────────────────────
 * Sensor Data Types
 * 
 * These types are defined here (in fsm.h) because
 * they're used across multiple modules: main.c,
 * ble.c, sensor_*.c, and analysis.c
 * ────────────────────────────────────────────── */

/* Single 3-axis accelerometer sample (ADXL343) */
typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} accel_sample_t;

/* Environmental sensor reading (BME280) */
typedef struct {
	int16_t  temperature;  /* °C * 100 (e.g., 2550 = 25.50°C) */
	uint16_t humidity;     /* % * 100  (e.g., 6432 = 64.32%) */
	uint32_t pressure;     /* Pascals  (e.g., 101325 = 1013.25 hPa) */
	bool     valid;        /* true if reading succeeded */
} env_reading_t;

/* Vibration burst size */
#define VIBRATION_BURST_SIZE    512

/* ──────────────────────────────────────────────
 * System States
 * ────────────────────────────────────────────── */
typedef enum {
	STATE_INIT,                    /* Hardware initialization */
	STATE_ADVERTISING,             /* Waiting for connection */
	STATE_AUTHENTICATING,          /* PIN pairing in progress */
	STATE_CONNECTED_IDLE,          /* Connected, waiting for burst timer */
	STATE_COLLECTING_VIBRATION,    /* Reading accelerometer */
	STATE_COLLECTING_ENVIRONMENT,  /* Reading temp/humidity/pressure */
	STATE_TRANSMITTING,            /* Sending BLE notifications */
	STATE_DISCONNECTED,            /* Cleanup after disconnect */
	STATE_RECOVERY,                /* Retry after error */
	STATE_ERROR,                   /* Fatal error */
} fsm_state_t;

/* ──────────────────────────────────────────────
 * Event Types
 * ────────────────────────────────────────────── */
typedef enum {
	/* Initialization events */
	EVT_INIT_COMPLETE,
	EVT_INIT_FAILED,

	/* BLE connection events */
	EVT_BLE_READY,                 /* BLE stack ready (settings loaded) */
	EVT_BLE_CONNECTED,
	EVT_BLE_DISCONNECTED,
	EVT_BLE_NOTIFICATIONS_ENABLED,
	EVT_BLE_NOTIFICATIONS_DISABLED,

	/* Security/pairing events */
	EVT_PAIRING_REQUEST,           /* PIN display triggered */
	EVT_PAIRING_COMPLETE,          /* Authentication succeeded */
	EVT_PAIRING_FAILED,            /* Wrong PIN or timeout */
	EVT_AUTH_TIMEOUT,              /* 60 seconds elapsed without pairing */

	/* Timer events */
	EVT_BURST_TIMER_EXPIRED,
	EVT_CLEANUP_TIMEOUT,
	EVT_WATCHDOG_TIMEOUT,
	EVT_RETRY_TIMEOUT,

	/* Transmission events */
	EVT_BLE_TX_COMPLETE,
	EVT_BLE_TX_FAILED,
} fsm_event_type_t;

/* ──────────────────────────────────────────────
 * Event Data Payloads
 *
 * Different event types carry different data.
 * Use a union to keep the struct size reasonable.
 * ────────────────────────────────────────────── */
typedef struct {
	fsm_event_type_t type;
	uint32_t timestamp;  /* k_uptime_get_32() when event was posted */

	union {
		/* BLE disconnection reason */
		struct {
			uint8_t reason;
		} ble_disconnect;

		/* Initialization or transmission error code */
		int error_code;

		/* Pairing passkey */
		uint32_t passkey;

		/* Pairing result */
		bool bonded;

		/* Security error */
		uint8_t security_error;
	} data;
} fsm_event_t;

/* ──────────────────────────────────────────────
 * Event Posting Function Type
 *
 * Modules like ble.c call this to post events.
 * The FSM registers its event queue handler during init.
 * ────────────────────────────────────────────── */
typedef void (*event_post_fn)(const fsm_event_t *evt);

#endif /* FSM_H */
