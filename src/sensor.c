/*
 * ADXL343 Sensor Implementation
 */

#include "sensor.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

/* ADXL343 Register addresses */
#define ADXL343_REG_DEVID      0x00
#define ADXL343_REG_BW_RATE    0x2C
#define ADXL343_REG_POWER_CTL  0x2D
#define ADXL343_REG_DATA_FORMAT 0x31
#define ADXL343_REG_DATAX0     0x32

/* ADXL343 Configuration values */
#define ADXL343_DEVID          0xE5
#define ADXL343_POWER_MEASURE  0x08
#define ADXL343_RATE_1600HZ    0x0D
#define ADXL343_RANGE_16G      0x03
#define ADXL343_FULL_RES       0x08

/* Get I2C device from devicetree */
#define I2C_NODE DT_NODELABEL(adxl343)
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);

/**
 * Scan I2C bus for devices
 */
static void scan_i2c_bus(void)
{
	int ret;
	int found = 0;

	printk("Scanning I2C bus...\n");
	
	for (uint8_t addr = 0x08; addr < 0x78; addr++) {
		ret = i2c_write(dev_i2c.bus, NULL, 0, addr);
		if (ret == 0) {
			printk("  Device found at 0x%02X\n", addr);
			found++;
		}
	}
	
	printk("Total devices found: %d\n\n", found);
}

/**
 * Initialize ADXL343 sensor
 */
int sensor_init(void)
{
	int ret;
	uint8_t chip_id;

	printk("\n=== ADXL343 Initialization ===\n");

	/* Check if I2C bus is ready */
	if (!device_is_ready(dev_i2c.bus)) {
		printk("ERROR: I2C bus not ready!\n");
		return -ENODEV;
	}
	printk("I2C bus ready: %s\n", dev_i2c.bus->name);

	/* Scan bus */
	scan_i2c_bus();

	/* Verify device ID */
	ret = i2c_reg_read_byte_dt(&dev_i2c, ADXL343_REG_DEVID, &chip_id);
	if (ret != 0) {
		printk("ERROR: Failed to read device ID: %d\n", ret);
		return ret;
	}
	
	if (chip_id != ADXL343_DEVID) {
		printk("ERROR: Wrong device ID: 0x%02X (expected 0x%02X)\n", 
		       chip_id, ADXL343_DEVID);
		return -EINVAL;
	}
	printk("ADXL343 detected, ID: 0x%02X\n", chip_id);

	/* Configure data rate: 1600 Hz */
	ret = i2c_reg_write_byte_dt(&dev_i2c, ADXL343_REG_BW_RATE, 
	                            ADXL343_RATE_1600HZ);
	if (ret != 0) {
		printk("ERROR: Failed to set data rate: %d\n", ret);
		return ret;
	}
	printk("Data rate: 1600 Hz\n");

	/* Configure data format: Full resolution, ±16g */
	ret = i2c_reg_write_byte_dt(&dev_i2c, ADXL343_REG_DATA_FORMAT,
	                            ADXL343_FULL_RES | ADXL343_RANGE_16G);
	if (ret != 0) {
		printk("ERROR: Failed to set data format: %d\n", ret);
		return ret;
	}
	printk("Range: ±16g, Full resolution\n");

	/* Enable measurement mode */
	ret = i2c_reg_write_byte_dt(&dev_i2c, ADXL343_REG_POWER_CTL, 
	                            ADXL343_POWER_MEASURE);
	if (ret != 0) {
		printk("ERROR: Failed to enable measurement: %d\n", ret);
		return ret;
	}
	printk("Measurement mode: Enabled\n");

	printk("=== Initialization Complete ===\n\n");
	return 0;
}

/**
 * Read one XYZ snapshot using burst-read
 */
int sensor_read_snapshot(accel_sample_t *sample)
{
	uint8_t data[6];
	int ret;

	/* Burst-read all 6 bytes in one I2C transaction */
	ret = i2c_burst_read_dt(&dev_i2c, ADXL343_REG_DATAX0, data, 6);
	if (ret != 0) {
		return ret;
	}

	/* Combine bytes (little-endian) */
	sample->x = (int16_t)((data[1] << 8) | data[0]);
	sample->y = (int16_t)((data[3] << 8) | data[2]);
	sample->z = (int16_t)((data[5] << 8) | data[4]);

	return 0;
}

/**
 * Collect a burst of snapshots at precise timing
 */
int sensor_collect_burst(accel_sample_t *buffer, uint16_t count)
{
	int ret;
	int64_t start_time_us, target_time_us, current_time_us;

	printk("Collecting %d samples at %d Hz...\n", count, SAMPLE_RATE_HZ);

	start_time_us = k_uptime_get() * 1000;

	for (uint16_t i = 0; i < count; i++) {
		/* Calculate target time for this sample */
		target_time_us = start_time_us + (i * SAMPLE_PERIOD_US);

		/* Wait until target time */
		current_time_us = k_uptime_get() * 1000;
		int64_t wait_us = target_time_us - current_time_us;
		
		if (wait_us > 0) {
			k_busy_wait(wait_us);
		}

		/* Read snapshot (atomic XYZ) */
		ret = sensor_read_snapshot(&buffer[i]);
		if (ret != 0) {
			printk("ERROR: Failed to read sample %d: %d\n", i, ret);
			return ret;
		}

		/* Print progress */
		if ((i + 1) % 100 == 0) {
			printk("  Progress: %d/%d samples\n", i + 1, count);
		}
	}

	/* Report timing accuracy */
	int64_t end_time_us = k_uptime_get() * 1000;
	uint32_t actual_ms = (end_time_us - start_time_us) / 1000;
	uint32_t expected_ms = (count * 1000) / SAMPLE_RATE_HZ;

	printk("Collection complete: %u ms (expected %u ms)\n",
	       actual_ms, expected_ms);

	return 0;
}