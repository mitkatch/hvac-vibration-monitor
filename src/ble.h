/*
 * BLE Interface for Vibration Data Transmission
 */

#ifndef BLE_H
#define BLE_H

#include "sensor.h"
#include <stdint.h>

/**
 * Initialize BLE stack and start advertising
 * 
 * @return 0 on success, negative error code on failure
 */
int ble_init(void);

/**
 * Start BLE advertising (make device discoverable)
 * 
 * @return 0 on success, negative error code on failure
 */
int ble_start_advertising(void);

/**
 * Stop BLE advertising
 * 
 * @return 0 on success, negative error code on failure
 */
int ble_stop_advertising(void);

/**
 * Check if a BLE client is connected
 * 
 * @return true if connected, false otherwise
 */
bool ble_is_connected(void);

/**
 * Transmit burst of samples over BLE
 * Splits into multiple packets if needed
 * 
 * @param buffer Sample buffer
 * @param count Number of samples
 * @return 0 on success, negative error code on failure
 */
int ble_transmit_burst(const accel_sample_t *buffer, uint16_t count);

/**
 * Get BLE connection status string
 * 
 * @return Status string (for logging)
 */
const char *ble_get_status(void);

#endif /* BLE_H */