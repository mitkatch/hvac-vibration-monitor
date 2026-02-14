/*
 * sensor.h — Common sensor interface
 *
 * Provides a unified API for the FSM to interact with sensors.
 * Each physical sensor (ADXL343, BME280, future additions)
 * lives in its own .c file and implements its own init/collect.
 *
 * File layout:
 *   sensor.h              — this file (shared API)
 *   sensor_adxl343.c      — ADXL343 accelerometer driver
 *   sensor_bme280.c       — BME280 environment driver (optional)
 *   sensor_piezo.c        — piezoelectric high-freq (future)
 *
 * Adding a new sensor:
 *   1. Create sensor_xxx.c with init and collect functions
 *   2. Add declarations to this header
 *   3. Call init from system_init() in main.c
 *   4. Call collect from FSM at appropriate state
 */

#ifndef SENSOR_H
#define SENSOR_H

#include "fsm.h"

/* ──────────────────────────────────────────────
 * Vibration Sensor (ADXL343)
 * Implemented in: sensor_adxl343.c
 * ────────────────────────────────────────────── */

/* Initialize ADXL343 over I2C, configure sample rate and range.
 * Returns 0 on success, negative error code on failure. */
int sensor_init_vibration(void);

/* Collect a burst of vibration samples into buffer.
 * Blocks for ~320ms at 1600 Hz / 512 samples.
 * Returns number of samples collected, or negative error. */
int sensor_collect_vibration(accel_sample_t *buffer, uint16_t max_samples);

/* ──────────────────────────────────────────────
 * Environment Sensor (BME280)
 * Implemented in: sensor_bme280.c
 * ────────────────────────────────────────────── */

/* Initialize BME280 over I2C.
 * Returns 0 on success, negative if sensor not found.
 * Not finding the sensor is non-fatal — system continues without. */
int sensor_init_environment(void);

/* Read temperature, humidity, pressure.
 * Single I2C transaction, returns in ~10ms.
 * Returns 0 on success, negative on error.
 * Sets reading->valid = true on success. */
int sensor_collect_environment(env_reading_t *reading);

/* ──────────────────────────────────────────────
 * Piezoelectric Sensor (future)
 * Implemented in: sensor_piezo.c
 *
 * Uncomment when hardware is ready:
 *
 * int sensor_init_piezo(void);
 * int sensor_collect_piezo(int16_t *buffer, uint16_t max_samples,
 *                          uint32_t sample_rate);
 * ────────────────────────────────────────────── */

#endif /* SENSOR_H */
