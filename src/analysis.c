/*
 * Vibration Analysis Implementation
 */

#include "analysis.h"
#include <zephyr/sys/printk.h>

/**
 * Integer square root helper
 */
static uint32_t int_sqrt(uint64_t n)
{
	if (n == 0) {
		return 0;
	}

	uint64_t x = n;
	uint64_t y = (x + 1) / 2;

	while (y < x) {
		x = y;
		y = (x + n / x) / 2;
	}

	return (uint32_t)x;
}

/**
 * Print sample data
 */
void analysis_print_samples(const accel_sample_t *buffer, uint16_t count)
{
	printk("\n--- Sample Data (first 10 and last 10) ---\n");

	/* First 10 samples */
	for (int i = 0; i < 10 && i < count; i++) {
		int32_t x_mg = sensor_lsb_to_mg(buffer[i].x);
		int32_t y_mg = sensor_lsb_to_mg(buffer[i].y);
		int32_t z_mg = sensor_lsb_to_mg(buffer[i].z);

		printk("[%3d] X=%6d mg, Y=%6d mg, Z=%6d mg\n",
		       i, x_mg, y_mg, z_mg);
	}

	if (count > 20) {
		printk("... (%d samples omitted) ...\n", count - 20);

		/* Last 10 samples */
		for (int i = count - 10; i < count; i++) {
			int32_t x_mg = sensor_lsb_to_mg(buffer[i].x);
			int32_t y_mg = sensor_lsb_to_mg(buffer[i].y);
			int32_t z_mg = sensor_lsb_to_mg(buffer[i].z);

			printk("[%3d] X=%6d mg, Y=%6d mg, Z=%6d mg\n",
			       i, x_mg, y_mg, z_mg);
		}
	}
	printk("-------------------------------------------\n\n");
}

/**
 * Calculate RMS vibration level
 */
void analysis_calculate_rms(const accel_sample_t *buffer, uint16_t count)
{
	int64_t sum_x2 = 0;
	int64_t sum_y2 = 0;
	int64_t sum_z2 = 0;

	/* Sum of squares */
	for (uint16_t i = 0; i < count; i++) {
		sum_x2 += (int64_t)buffer[i].x * buffer[i].x;
		sum_y2 += (int64_t)buffer[i].y * buffer[i].y;
		sum_z2 += (int64_t)buffer[i].z * buffer[i].z;
	}

	/* RMS */
	uint32_t rms_x = int_sqrt(sum_x2 / count);
	uint32_t rms_y = int_sqrt(sum_y2 / count);
	uint32_t rms_z = int_sqrt(sum_z2 / count);

	/* Convert to milli-g */
	printk("=== Vibration RMS ===\n");
	printk("X-axis: %u mg\n", rms_x * 4);
	printk("Y-axis: %u mg\n", rms_y * 4);
	printk("Z-axis: %u mg\n", rms_z * 4);
	printk("=====================\n\n");
}

/**
 * Find peak values
 */
void analysis_find_peaks(const accel_sample_t *buffer, uint16_t count)
{
	int16_t max_x = buffer[0].x;
	int16_t min_x = buffer[0].x;
	int16_t max_y = buffer[0].y;
	int16_t min_y = buffer[0].y;
	int16_t max_z = buffer[0].z;
	int16_t min_z = buffer[0].z;

	/* Find min/max */
	for (uint16_t i = 1; i < count; i++) {
		if (buffer[i].x > max_x) max_x = buffer[i].x;
		if (buffer[i].x < min_x) min_x = buffer[i].x;
		if (buffer[i].y > max_y) max_y = buffer[i].y;
		if (buffer[i].y < min_y) min_y = buffer[i].y;
		if (buffer[i].z > max_z) max_z = buffer[i].z;
		if (buffer[i].z < min_z) min_z = buffer[i].z;
	}

	printk("=== Peak Values ===\n");
	printk("X: %d to %d mg (peak-to-peak: %d mg)\n",
	       sensor_lsb_to_mg(min_x), 
	       sensor_lsb_to_mg(max_x),
	       sensor_lsb_to_mg(max_x - min_x));
	printk("Y: %d to %d mg (peak-to-peak: %d mg)\n",
	       sensor_lsb_to_mg(min_y),
	       sensor_lsb_to_mg(max_y),
	       sensor_lsb_to_mg(max_y - min_y));
	printk("Z: %d to %d mg (peak-to-peak: %d mg)\n",
	       sensor_lsb_to_mg(min_z),
	       sensor_lsb_to_mg(max_z),
	       sensor_lsb_to_mg(max_z - min_z));
	printk("===================\n\n");
}

/**
 * Calculate mean (DC offset)
 */
void analysis_calculate_mean(const accel_sample_t *buffer, uint16_t count)
{
	int64_t sum_x = 0;
	int64_t sum_y = 0;
	int64_t sum_z = 0;

	for (uint16_t i = 0; i < count; i++) {
		sum_x += buffer[i].x;
		sum_y += buffer[i].y;
		sum_z += buffer[i].z;
	}

	int32_t mean_x = sum_x / count;
	int32_t mean_y = sum_y / count;
	int32_t mean_z = sum_z / count;

	printk("=== Mean Values (DC offset) ===\n");
	printk("X-axis: %d mg\n", sensor_lsb_to_mg(mean_x));
	printk("Y-axis: %d mg\n", sensor_lsb_to_mg(mean_y));
	printk("Z-axis: %d mg\n", sensor_lsb_to_mg(mean_z));
	printk("================================\n\n");
}

/**
 * Print comprehensive summary
 */
void analysis_print_summary(const accel_sample_t *buffer, uint16_t count)
{
	printk("\n");
	printk("╔═══════════════════════════════════════╗\n");
	printk("║       Vibration Analysis Summary      ║\n");
	printk("╚═══════════════════════════════════════╝\n");
	printk("\n");

	analysis_calculate_mean(buffer, count);
	analysis_calculate_rms(buffer, count);
	analysis_find_peaks(buffer, count);
}