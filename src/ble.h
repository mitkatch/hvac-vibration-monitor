/*
 * ble.h — BLE service function declarations
 */

#ifndef BLE_H
#define BLE_H

#include <zephyr/bluetooth/conn.h>
#include "fsm.h"
#include "sensor.h"

/* ──────────────────────────────────────────────
 * Initialization
 * ────────────────────────────────────────────── */

/**
 * Initialize BLE stack and register security callbacks
 *
 * @param callback  Function pointer for posting events to FSM
 * @return 0 on success, negative errno on failure
 */
int ble_init(event_post_fn callback);

/* ──────────────────────────────────────────────
 * Advertising Control
 * ────────────────────────────────────────────── */

/**
 * Start BLE advertising
 * @return 0 on success, negative errno on failure
 */
int ble_start_advertising(void);

/**
 * Stop BLE advertising
 * @return 0 on success, negative errno on failure
 */
int ble_stop_advertising(void);

/* ──────────────────────────────────────────────
 * Connection Management
 * ────────────────────────────────────────────── */

/**
 * Clean up after disconnection
 * Releases connection reference and resets state
 */
void ble_cleanup(void);

/**
 * Check if currently connected to a central
 * @return true if connected, false otherwise
 */
bool ble_is_connected(void);

/**
 * Check if client has enabled vibration notifications
 * @return true if notifications enabled, false otherwise
 */
bool ble_notifications_enabled(void);

/* ──────────────────────────────────────────────
 * Security Functions
 * ────────────────────────────────────────────── */

/**
 * Request authenticated pairing (Security Level 3)
 * This triggers the pairing process with PIN display
 * 
 * @return 0 on success, negative errno on failure
 */
int ble_set_security(void);

/**
 * Check if connection is authenticated
 * @return true if security level >= L3, false otherwise
 */
bool ble_is_authenticated(void);

/**
 * Delete all stored bonds (factory reset)
 * @return 0 on success, negative errno on failure
 */
int ble_delete_all_bonds(void);

/* ──────────────────────────────────────────────
 * Data Transmission
 * ────────────────────────────────────────────── */

/**
 * Transmit vibration burst as BLE notifications
 *
 * @param buffer  Array of accelerometer samples
 * @param count   Number of samples in buffer
 * @return 0 on success, negative errno on failure
 *         -ENOTCONN if connection lost
 *         -ENOMEM if TX buffer exhausted
 */
int ble_transmit_burst(const accel_sample_t *buffer, uint16_t count);

/**
 * Transmit environmental reading
 *
 * @param reading  Pointer to environment data struct
 * @return 0 on success, negative errno on failure
 */
int ble_transmit_environment(const env_reading_t *reading);

#endif /* BLE_H */
