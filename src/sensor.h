/*
 * ADXL343 Sensor Interface
 * Hardware-specific functions for ADXL343 accelerometer
 */

#ifndef SENSOR_H
#define SENSOR_H

#include <zephyr/kernel.h>
#include <stdint.h>

/* Configuration */
#define SAMPLE_RATE_HZ      1600
#define SAMPLES_PER_BURST   512
#define SAMPLE_PERIOD_US    (1000000 / SAMPLE_RATE_HZ)

/* ADXL343 I2C Address */
#define ADXL343_I2C_ADDR       0x53

/* Data structure for one XYZ snapshot */
typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} __packed accel_sample_t;

/**
 * Initialize ADXL343 sensor
 * 
 * @return 0 on success, negative error code on failure
 */
int sensor_init(void);

/**
 * Read one XYZ snapshot (atomic read of all 3 axes)
 * 
 * @param sample Pointer to store the sample
 * @return 0 on success, negative error code on failure
 */
int sensor_read_snapshot(accel_sample_t *sample);

/**
 * Collect a burst of snapshots at precise timing
 * 
 * @param buffer Buffer to store samples
 * @param count Number of samples to collect
 * @return 0 on success, negative error code on failure
 */
int sensor_collect_burst(accel_sample_t *buffer, uint16_t count);

/**
 * Convert raw LSB value to milli-g
 * 
 * @param lsb Raw sensor value
 * @return Value in milli-g
 */
static inline int32_t sensor_lsb_to_mg(int16_t lsb)
{
	/* Full resolution: 4 mg/LSB for all ranges */
	return (int32_t)lsb * 4;
}

#endif /* SENSOR_H */