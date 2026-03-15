/*
 * ble.h — BLE service function declarations
 */

#ifndef BLE_H
#define BLE_H

#include <zephyr/bluetooth/conn.h>
#include "fsm.h"
#include "sensor.h"
#include "analysis.h"

/* ──────────────────────────────────────────────
 * Initialization
 * ────────────────────────────────────────────── */
int ble_init(event_post_fn callback);

/* ──────────────────────────────────────────────
 * Advertising Control
 * ────────────────────────────────────────────── */
int ble_start_advertising(void);
int ble_stop_advertising(void);

/* ──────────────────────────────────────────────
 * Connection Management
 * ────────────────────────────────────────────── */
void ble_cleanup(void);
bool ble_is_connected(void);
bool ble_notifications_enabled(void);

/* ──────────────────────────────────────────────
 * Security Functions
 * ────────────────────────────────────────────── */
int ble_set_security(void);
bool ble_is_authenticated(void);
int ble_delete_all_bonds(void);

/* ──────────────────────────────────────────────
 * Data Transmission
 * ────────────────────────────────────────────── */

/**
 * Transmit FFT-derived feature stats (single BLE notification).
 *
 * Wire format: burst_header_t (8 bytes) + fft_stats_t (60 bytes) = 68 bytes.
 * PKT_TYPE_FFT_STATS.
 */
int ble_transmit_fft_stats(uint16_t seq, uint16_t count,
			   const fft_stats_t *stats);

/**
 * Transmit a feature stats packet (single BLE notification).
 *
 * Wire format: burst_header_t (8 bytes) + time_stats_t (36 bytes) = 44 bytes.
 * PKT_TYPE_STATS.  Fits in a single notification.
 *
 * @param seq    Burst sequence number (from ring slot)
 * @param count  Number of samples the stats were computed from
 * @param stats  Computed time-domain feature vector
 * @return 0 on success, negative errno on failure
 */
int ble_transmit_stats(uint16_t seq, uint16_t count,
		       const time_stats_t *stats);

/**
 * Transmit raw vibration samples as chunked BLE notifications.
 *
 * Each notification: burst_header_t (8 bytes) + up to 236 bytes of samples.
 * PKT_TYPE_RAW.  The hub reassembles using seq + chunk_index.
 *
 * @param buffer  Array of accelerometer samples
 * @param count   Number of samples in buffer
 * @param seq     Burst sequence number (from ring slot)
 * @return 0 on success, negative errno on failure
 *         -ENOTCONN if connection lost
 */
int ble_transmit_burst_raw(const accel_sample_t *buffer, uint16_t count,
			   uint16_t seq);

/**
 * Transmit environmental reading.
 *
 * Wire format: burst_header_t (8 bytes) + env payload (8 bytes) = 16 bytes.
 * PKT_TYPE_ENV.
 *
 * @param reading  Pointer to environment data struct
 * @return 0 on success, negative errno on failure
 */
int ble_transmit_environment(const env_reading_t *reading);

#endif /* BLE_H */
