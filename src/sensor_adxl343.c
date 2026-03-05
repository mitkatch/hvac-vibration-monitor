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

	/*
	 * FIFO-driven collection — no software timing.
	 *
	 * The ADXL343 clocks samples into its 32-deep FIFO at exactly
	 * SAMPLE_RATE_HZ. We poll FIFO_STATUS for entries, drain them
	 * in batches, and repeat until we have max_samples.
	 *
	 * FIFO mode changes require standby — do NOT toggle bypass/stream
	 * while in measurement mode. Instead, flush stale samples by
	 * draining the FIFO until empty before starting collection.
	 */

	/* Flush stale FIFO samples.
	 * At 1600 Hz new samples arrive every 625 µs — faster than we can drain
	 * in a tight loop. Pause measurement briefly, flush, resume.
	 * This is safe: standby → flush → measure takes < 1 ms.
	 */
	{
		uint8_t dummy[6];
		uint8_t status;
		adxl_write_reg(ADXL343_POWER_CTL, 0x00);  /* standby — stop new samples */
		for (int i = 0; i < 32; i++) {
			if (adxl_read_reg(ADXL343_FIFO_STATUS, &status) != 0) break;
			if ((status & 0x3F) == 0) break;
			adxl_burst_read(ADXL343_DATAX0, dummy, 6);
		}
		adxl_write_reg(ADXL343_POWER_CTL, 0x08);  /* measurement — resume */
		LOG_DBG("ADXL343: FIFO flushed, measuring");
	}

	int64_t start_time = k_uptime_get();
	uint16_t collected = 0;

	/* Timeout: max_samples / rate * 2 (generous headroom) */
	const int64_t timeout_ms = ((int64_t)max_samples * 2000) / SAMPLE_RATE_HZ;

	LOG_INF("ADXL343: Collecting %d samples at %d Hz (FIFO mode)...",
		max_samples, SAMPLE_RATE_HZ);

	while (collected < max_samples) {

		/* Timeout guard */
		if ((k_uptime_get() - start_time) > timeout_ms) {
			LOG_ERR("ADXL343: Collection timeout at %d/%d samples",
				collected, max_samples);
			break;
		}

		/* Read FIFO status — lower 6 bits = entries available */
		uint8_t fifo_status;
		int err = adxl_read_reg(ADXL343_FIFO_STATUS, &fifo_status);
		if (err) {
			LOG_ERR("ADXL343: FIFO status read failed (err %d)", err);
			break;
		}

		uint8_t entries = fifo_status & 0x3F;

		if (entries == 0) {
			/* FIFO empty — sleep ~5 ms (8 samples at 1600 Hz) */
			k_msleep(5);
			continue;
		}

		/* Drain available entries, up to what buffer can hold */
		uint16_t to_read = MIN((uint16_t)entries, max_samples - collected);

		for (uint8_t j = 0; j < to_read; j++) {
			uint8_t data[6];
			err = adxl_burst_read(ADXL343_DATAX0, data, 6);
			if (err) {
				LOG_ERR("ADXL343: Read failed at sample %d (err %d)",
					collected, err);
				goto done;
			}

			buffer[collected].x = (int16_t)((data[1] << 8) | data[0]);
			buffer[collected].y = (int16_t)((data[3] << 8) | data[2]);
			buffer[collected].z = (int16_t)((data[5] << 8) | data[4]);
			collected++;
		}
	}

done:
	int64_t duration_ms = k_uptime_get() - start_time;
	LOG_INF("ADXL343: Collected %d samples in %lld ms "
		"(%.1f Hz actual)",
		collected, duration_ms,
		collected ? (collected * 1000.0 / duration_ms) : 0.0);

	return collected;
}
