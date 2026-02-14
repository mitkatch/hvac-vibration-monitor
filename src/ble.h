/*
 * ble.h — BLE peripheral service API
 *
 * Service functions called by the FSM.
 * ble.c owns the connection lifecycle internally.
 */

#ifndef BLE_H
#define BLE_H

#include "fsm.h"

/* Initialize BLE stack and register event callback */
int ble_init(event_post_fn callback);

/* Advertising control */
int  ble_start_advertising(void);
int  ble_stop_advertising(void);

/* Connection state queries */
bool ble_is_connected(void);
bool ble_notifications_enabled(void);

/* Cleanup after disconnect (called by FSM on DISCONNECTED entry) */
void ble_cleanup(void);

/* Data transmission (called synchronously by FSM) */
int ble_transmit_burst(const accel_sample_t *buffer, uint16_t count);
int ble_transmit_environment(const env_reading_t *reading);

#endif /* BLE_H */
