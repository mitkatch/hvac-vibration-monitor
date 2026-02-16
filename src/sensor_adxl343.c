/*
 * sensor_adxl343.c — ADXL343 accelerometer driver
 *
 * Handles I2C initialization, configuration, and burst
 * sample collection for the ADXL343 vibration sensor.
 *
 * This file knows nothing about BLE, FSM, or events.
 * It only knows how to talk to the ADXL343 hardware.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "sensor.h"

LOG_MODULE_REGISTER(adxl343, LOG_LEVEL_INF);

/* ──────────────────────────────────────────────
 * ADXL343 Register Map
 * ────────────────────────────────────────────── */
#define ADXL343_ADDR            0x53
#define ADXL343_DEVID           0x00
#define ADXL343_BW_RATE         0x2C
#define ADXL343_POWER_CTL       0x2D
#define ADXL343_DATA_FORMAT     0x31
#define ADXL343_DATAX0          0x32
#define ADXL343_FIFO_CTL        0x38
#define ADXL343_FIFO_STATUS     0x39

#define ADXL343_DEVID_VALUE     0xE5

/* BW_RATE values */
#define BW_RATE_1600HZ          0x0E
#define BW_RATE_3200HZ          0x0F

/* DATA_FORMAT: full resolution, ±16g */
#define DATA_FORMAT_FULL_16G    0x0B

/* FIFO_CTL: stream mode, 32 samples */
#define FIFO_STREAM_32          0x9F

#define SAMPLE_RATE_HZ  		1600
/* ──────────────────────────────────────────────
 * I2C Device
 * ────────────────────────────────────────────── */
static const struct device *i2c_dev;

/* ──────────────────────────────────────────────
 * I2C Helpers
 * ────────────────────────────────────────────── */
static int adxl_write_reg(uint8_t reg, uint8_t val)
{
	return i2c_reg_write_byte(i2c_dev, ADXL343_ADDR, reg, val);
}

static int adxl_read_reg(uint8_t reg, uint8_t *val)
{
	return i2c_reg_read_byte(i2c_dev, ADXL343_ADDR, reg, val);
}

static int adxl_burst_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
	return i2c_burst_read(i2c_dev, ADXL343_ADDR, reg, buf, len);
}

/* ──────────────────────────────────────────────
 * Public API
 * ────────────────────────────────────────────── */
int sensor_init_vibration(void)
{
	int err;
	uint8_t dev_id;

	/* Get I2C bus */
	i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("ADXL343: I2C bus not ready");
		return -ENODEV;
	}

	/* Verify device ID */
	err = adxl_read_reg(ADXL343_DEVID, &dev_id);
	if (err) {
		LOG_ERR("ADXL343: Failed to read device ID (err %d)", err);
		return err;
	}
	if (dev_id != ADXL343_DEVID_VALUE) {
		LOG_ERR("ADXL343: Wrong device ID (0x%02X, expected 0x%02X)",
			dev_id, ADXL343_DEVID_VALUE);
		return -ENODEV;
	}

	/* Standby mode for configuration */
	err = adxl_write_reg(ADXL343_POWER_CTL, 0x00);
	if (err) return err;

	/* Set sample rate: 1600 Hz */
	err = adxl_write_reg(ADXL343_BW_RATE, BW_RATE_1600HZ);
	if (err) return err;

	/* Data format: full resolution, ±16g */
	err = adxl_write_reg(ADXL343_DATA_FORMAT, DATA_FORMAT_FULL_16G);
	if (err) return err;

	/* FIFO: stream mode, 32 samples deep */
	err = adxl_write_reg(ADXL343_FIFO_CTL, FIFO_STREAM_32);
	if (err) return err;

	/* Start measurement */
	err = adxl_write_reg(ADXL343_POWER_CTL, 0x08);
	if (err) return err;

	LOG_INF("ADXL343: Initialized (1600 Hz, ±16g, full resolution)");
	return 0;
}

int sensor_collect_vibration(accel_sample_t *buffer, uint16_t max_samples)
{
	if (!i2c_dev || !device_is_ready(i2c_dev)) {
		return -ENODEV;
	}

	const uint32_t sample_period_us = 1000000 / SAMPLE_RATE_HZ;  /* 625 µs */
	int64_t start_time = k_uptime_get();
	uint16_t collected = 0;

	LOG_INF("ADXL343: Collecting %d samples at %d Hz...",
		max_samples, SAMPLE_RATE_HZ);

	for (uint16_t i = 0; i < max_samples; i++) {
		/* Calculate target time for this sample */
		int64_t target_us = (int64_t)i * sample_period_us;
		int64_t elapsed_us = (k_uptime_get() - start_time) * 1000;
		int64_t wait_us = target_us - elapsed_us;

		if (wait_us > 0) {
			k_usleep((int32_t)wait_us);
		}

		/* Burst read X, Y, Z (6 bytes, atomic) */
		uint8_t data[6];
		int err = adxl_burst_read(ADXL343_DATAX0, data, 6);
		if (err) {
			LOG_ERR("ADXL343: Read failed at sample %d (err %d)",
				i, err);
			break;
		}

		buffer[i].x = (int16_t)((data[1] << 8) | data[0]);
		buffer[i].y = (int16_t)((data[3] << 8) | data[2]);
		buffer[i].z = (int16_t)((data[5] << 8) | data[4]);
		collected++;
	}

	int64_t duration_ms = k_uptime_get() - start_time;
	LOG_INF("ADXL343: Collected %d samples in %lld ms", collected,
		duration_ms);

	return collected;
}
