/*
 * fft_fixed.h — Integer fixed-point FFT for nRF52840
 *
 * Self-contained, no external dependencies (no CMSIS-DSP, no arm_math.h).
 * Uses Q15 fixed-point arithmetic (values in range [-32768, 32767]).
 *
 * Supports N = power-of-two sizes up to 512 (tested: 64, 128, 256, 512).
 *
 * Usage:
 *   1. Populate re[] with your time-domain samples (scaled to Q15).
 *      Set im[] to all zeros for real input.
 *   2. Call fft_fixed(re, im, N) — in-place, decimation-in-time.
 *   3. Compute magnitudes: mag[k] = sqrt(re[k]^2 + im[k]^2)
 *      Only bins 0..N/2 are unique (conjugate symmetry for real input).
 *   4. Frequency of bin k: f_k = k × (sample_rate / N)
 *
 * Memory:
 *   re[] and im[] must each be int32_t[N].
 *   No heap allocation. Stack usage: O(log2 N) for recursion depth = 0
 *   (iterative Cooley-Tukey, no recursion).
 *
 * Precision note:
 *   Each butterfly stage introduces up to 1 bit of rounding error.
 *   For N=512 (9 stages), worst-case error is ~9 LSBs in Q15.
 *   Sufficient for energy band detection; not suitable for phase measurement.
 */

#ifndef FFT_FIXED_H
#define FFT_FIXED_H

#include <stdint.h>

/**
 * In-place Cooley-Tukey radix-2 DIT FFT.
 *
 * @param re   Real part array, length N, Q15 format (int32_t for headroom)
 * @param im   Imaginary part array, length N, initialise to 0 for real input
 * @param N    FFT size — must be a power of two, max 512
 */
void fft_fixed(int32_t *re, int32_t *im, uint16_t N);

/**
 * Integer square root, uint32_t → uint32_t.
 * Exposed so analysis.c can reuse it.
 */
uint32_t fft_isqrt32(uint32_t n);

#endif /* FFT_FIXED_H */
