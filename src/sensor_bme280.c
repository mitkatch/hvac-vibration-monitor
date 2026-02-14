/*
 * sensor_bme280.c — BME280 temperature/humidity/pressure driver
 *
 * Uses Zephyr's built-in BME280 sensor driver via the sensor API.
 * If the BME280 is not present on the I2C bus, init returns an
 * error and the system continues without environmental data.
 *
 * This file knows nothing about BLE, FSM, or events.
 * It only knows how to talk to the BME280 hardware.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "sensor.h"

LOG_MODULE_REGISTER(bme280, LOG_LEVEL_INF);

/* ──────────────────────────────────────────────
 * Device handle
 * ────────────────────────────────────────────── */
static const struct device *bme280_dev;
static bool initialized = false;

/* ──────────────────────────────────────────────
 * Public API
 * ────────────────────────────────────────────── */
int sensor_init_environment(void)
{
	/*
	 * Zephyr's sensor subsystem handles BME280 via devicetree.
	 * The node label must be defined in your overlay:
	 *
	 *   &i2c0 {
	 *       bme280: bme280@76 {
	 *           compatible = "bosch,bme280";
	 *           reg = <0x76>;
	 *       };
	 *   };
	 *
	 * If not present, this returns -ENODEV and the system
	 * continues without environmental monitoring.
	 */
	bme280_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(bme280));

	if (!bme280_dev) {
		LOG_INF("BME280: Not configured in devicetree");
		return -ENODEV;
	}

	if (!device_is_ready(bme280_dev)) {
		LOG_WRN("BME280: Device exists but not ready");
		return -ENODEV;
	}

	initialized = true;
	LOG_INF("BME280: Initialized (temp/humidity/pressure)");
	return 0;
}

int sensor_collect_environment(env_reading_t *reading)
{
	if (!initialized || !bme280_dev) {
		reading->valid = false;
		return -ENODEV;
	}

	/* Trigger a measurement */
	int err = sensor_sample_fetch(bme280_dev);
	if (err) {
		LOG_ERR("BME280: Sample fetch failed (err %d)", err);
		reading->valid = false;
		return err;
	}

	/* Read temperature (°C) */
	struct sensor_value temp;
	err = sensor_channel_get(bme280_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	if (err) {
		LOG_ERR("BME280: Temp read failed (err %d)", err);
		reading->valid = false;
		return err;
	}
	/* Convert to int16_t × 100: e.g. 23.50°C → 2350 */
	reading->temperature = (int16_t)(temp.val1 * 100 +
					 temp.val2 / 10000);

	/* Read humidity (%RH) */
	struct sensor_value hum;
	err = sensor_channel_get(bme280_dev, SENSOR_CHAN_HUMIDITY, &hum);
	if (err) {
		LOG_ERR("BME280: Humidity read failed (err %d)", err);
		reading->valid = false;
		return err;
	}
	reading->humidity = (uint16_t)(hum.val1 * 100 +
				       hum.val2 / 10000);

	/* Read pressure (hPa / mbar) */
	struct sensor_value press;
	err = sensor_channel_get(bme280_dev, SENSOR_CHAN_PRESS, &press);
	if (err) {
		LOG_ERR("BME280: Pressure read failed (err %d)", err);
		reading->valid = false;
		return err;
	}
	/* Zephyr returns kPa, convert to hPa (× 10) */
	reading->pressure = (uint16_t)(press.val1 * 10 +
				       press.val2 / 100000);

	reading->valid = true;

	LOG_DBG("BME280: T=%d.%02d°C H=%d.%02d%% P=%d hPa",
		reading->temperature / 100,
		abs(reading->temperature % 100),
		reading->humidity / 100,
		reading->humidity % 100,
		reading->pressure);

	return 0;
}
