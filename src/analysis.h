/*
 * analysis.h — On-device vibration analysis
 *
 * Optional analysis that runs on the nRF52840 before
 * or after BLE transmission. Currently computes RMS
 * values. Could expand to peak detection, threshold
 * alarms, etc.
 */

#ifndef ANALYSIS_H
#define ANALYSIS_H

#include "fsm.h"

/* RMS analysis result */
typedef struct {
	uint32_t rms_x;     /* RMS in mg */
	uint32_t rms_y;
	uint32_t rms_z;
	int16_t  peak_x;    /* Peak value in raw counts */
	int16_t  peak_y;
	int16_t  peak_z;
} vibration_stats_t;

/* Compute RMS and peak values from a sample buffer */
int analysis_compute_stats(const accel_sample_t *buffer,
			   uint16_t count,
			   vibration_stats_t *stats);

#endif /* ANALYSIS_H */
