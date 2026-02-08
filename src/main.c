/*
 * ADXL343 Vibration Monitor - Main Application
 * 
 * Collects bursts of vibration data for predictive maintenance
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "sensor.h"
#include "analysis.h"

/* Application configuration */
#define BURST_INTERVAL_SEC  10

/* Buffer for burst collection */
static accel_sample_t sample_buffer[SAMPLES_PER_BURST];

/**
 * Print application banner
 */
static void print_banner(void)
{
	printk("\n");
	printk("╔════════════════════════════════════════╗\n");
	printk("║  ADXL343 Vibration Monitor             ║\n");
	printk("║  HVAC Predictive Maintenance           ║\n");
	printk("╚════════════════════════════════════════╝\n");
	printk("\n");
}

/**
 * Print configuration
 */
static void print_config(void)
{
	printk("Configuration:\n");
	printk("  Sample rate: %d Hz\n", SAMPLE_RATE_HZ);
	printk("  Burst size: %d samples\n", SAMPLES_PER_BURST);
	printk("  Burst duration: %d ms\n", 
	       (SAMPLES_PER_BURST * 1000) / SAMPLE_RATE_HZ);
	printk("  Buffer size: %d bytes\n", sizeof(sample_buffer));
	printk("  Burst interval: %d seconds\n\n", BURST_INTERVAL_SEC);
}

/**
 * Process one burst of data
 */
static int process_burst(uint32_t burst_number)
{
	int ret;

	printk("╔═══════════════════════════════════════════╗\n");
	printk("║  Burst #%u\n", burst_number);
	printk("╚═══════════════════════════════════════════╝\n");

	/* Collect burst */
	ret = sensor_collect_burst(sample_buffer, SAMPLES_PER_BURST);
	if (ret != 0) {
		printk("ERROR: Burst collection failed: %d\n", ret);
		return ret;
	}

	/* Analyze data */
	analysis_print_samples(sample_buffer, SAMPLES_PER_BURST);
	analysis_print_summary(sample_buffer, SAMPLES_PER_BURST);

	/* TODO: Transmit to Raspberry Pi via BLE */
	/* ble_transmit_burst(sample_buffer, SAMPLES_PER_BURST); */

	return 0;
}

/**
 * Main application entry point
 */
int main(void)
{
	int ret;
	uint32_t burst_count = 0;

	/* Print banner */
	print_banner();
	print_config();

	/* Initialize sensor */
	ret = sensor_init();
	if (ret != 0) {
		printk("FATAL: Sensor initialization failed: %d\n", ret);
		return -1;
	}

	/* Stabilization delay */
	printk("Waiting for sensor to stabilize...\n");
	k_sleep(K_MSEC(100));
	printk("Ready to start monitoring!\n\n");

	/* Main loop: collect and process bursts */
	while (1) {
		burst_count++;

		/* Process one burst */
		ret = process_burst(burst_count);
		if (ret != 0) {
			printk("Burst failed, retrying in 1 second...\n");
			k_sleep(K_SECONDS(1));
			continue;
		}

		/* Sleep until next burst */
		printk("Burst complete. Sleeping %d seconds...\n\n",
		       BURST_INTERVAL_SEC);
		k_sleep(K_SECONDS(BURST_INTERVAL_SEC));
	}

	return 0;
}