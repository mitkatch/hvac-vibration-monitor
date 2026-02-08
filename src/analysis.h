/*
 * Vibration Analysis Helper Functions
 */

#ifndef ANALYSIS_H
#define ANALYSIS_H

#include "sensor.h"
#include <stdint.h>

/**
 * Print sample data for debugging
 * 
 * @param buffer Sample buffer
 * @param count Number of samples
 */
void analysis_print_samples(const accel_sample_t *buffer, uint16_t count);

/**
 * Calculate RMS (Root Mean Square) vibration level
 * 
 * @param buffer Sample buffer
 * @param count Number of samples
 */
void analysis_calculate_rms(const accel_sample_t *buffer, uint16_t count);

/**
 * Find peak values in buffer
 * 
 * @param buffer Sample buffer
 * @param count Number of samples
 */
void analysis_find_peaks(const accel_sample_t *buffer, uint16_t count);

/**
 * Calculate mean (average) values
 * 
 * @param buffer Sample buffer
 * @param count Number of samples
 */
void analysis_calculate_mean(const accel_sample_t *buffer, uint16_t count);

/**
 * Print statistics summary
 * 
 * @param buffer Sample buffer
 * @param count Number of samples
 */
void analysis_print_summary(const accel_sample_t *buffer, uint16_t count);

#endif /* ANALYSIS_H */