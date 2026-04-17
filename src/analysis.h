/*
 * analysis.h — Full on-device vibration feature extraction
 *
 * Two-stage pipeline per burst:
 *
 *  Stage 1 — Time-domain (single pass, no extra memory):
 *    • RMS per axis (mg)
 *    • Peak per axis (raw counts)
 *    • Crest factor per axis (peak_mg / rms_mg × 100)
 *    • Kurtosis per axis (×100, integer) — most sensitive early fault indicator
 *    • Variance per axis (mg²)
 *
 *  Stage 2 — Frequency-domain (FFT per axis, in-place, 512-point):
 *    • Dominant frequency bin + magnitude
 *    • Total spectral power
 *    • Band energy in each bearing fault window:
 *        BPFO  — Ball Pass Frequency Outer race
 *        BPFI  — Ball Pass Frequency Inner race
 *        BSF   — Ball Spin Frequency
 *        FTF   — Fundamental Train Frequency (cage)
 *    • Noise floor + SNR at BPFO
 *
 * Bearing fault frequencies depend on geometry. Call
 * analysis_set_bearing_params() once at startup from flash/settings.
 * Defaults match a typical small 6204 bearing at 1450 RPM.
 *
 * BLE transmission:
 *   PKT_TYPE_TIME_STATS: time_stats_t  (36 bytes)  — 1 notification
 *   PKT_TYPE_FFT_STATS:  fft_stats_t   (60 bytes)  — 1 notification
 *   Total: 2 notifications per burst vs 14 for raw.
 *
 * Memory budget (nRF52840, 256 KB SRAM):
 *   3× int32_t[512] FFT scratch = 6144 bytes (static in .bss)
 *   Ring slots: 4 × ~3096 bytes ≈ 12.4 KB
 *   Total analysis overhead: ~20 KB — well within budget.
 */

#ifndef ANALYSIS_H
#define ANALYSIS_H

#include <stdint.h>
#include "fsm.h"

/* ──────────────────────────────────────────────
 * Time-domain feature vector — 36 bytes
 *
 * Wire layout PKT_TYPE_TIME_STATS (little-endian):
 *   [0..1]   rms_x        uint16  mg
 *   [2..3]   rms_y        uint16  mg
 *   [4..5]   rms_z        uint16  mg
 *   [6..7]   peak_x       int16   raw counts (1 LSB = 3.9 mg)
 *   [8..9]   peak_y       int16
 *   [10..11] peak_z       int16
 *   [12..13] crest_x      uint16  (|peak_mg|/rms_mg)×100
 *   [14..15] crest_y      uint16
 *   [16..17] crest_z      uint16
 *   [18..19] kurtosis_x   int16   ×100 (Gaussian≈300, fault>400)
 *   [20..21] kurtosis_y   int16   ×100
 *   [22..23] kurtosis_z   int16   ×100
 *   [24..25] variance_x   uint16  mg²
 *   [26..27] variance_y   uint16  mg²
 *   [28..29] variance_z   uint16  mg²
 *   [30..31] reserved     uint16
 *   [32..35] sample_count uint32
 * ────────────────────────────────────────────── */
typedef struct {
	uint16_t rms_x;
	uint16_t rms_y;
	uint16_t rms_z;
	int16_t  peak_x;
	int16_t  peak_y;
	int16_t  peak_z;
	uint16_t crest_x;
	uint16_t crest_y;
	uint16_t crest_z;
	int16_t  kurtosis_x;
	int16_t  kurtosis_y;
	int16_t  kurtosis_z;
	uint16_t variance_x;
	uint16_t variance_y;
	uint16_t variance_z;
	uint16_t reserved;
	uint32_t sample_count;
} time_stats_t;

/* ──────────────────────────────────────────────
 * Frequency-domain feature vector — 60 bytes
 *
 * Wire layout PKT_TYPE_FFT_STATS (little-endian):
 * 20 bytes per axis × 3 axes = 60 bytes total
 *   [0..1]   dom_freq_hz  uint16  Hz
 *   [2..3]   dom_mag      uint16  FFT magnitude (scaled)
 *   [4..5]   total_power  uint16  Σ|X[k]|² normalized
 *   [6..7]   bpfo_energy  uint16
 *   [8..9]   bpfi_energy  uint16
 *   [10..11] bsf_energy   uint16
 *   [12..13] ftf_energy   uint16
 *   [14..15] noise_floor  uint16  median outside fault bands
 *   [16..17] snr_bpfo     uint16  (bpfo_energy/noise_floor)×100
 *   [18..19] reserved     uint16
 * ────────────────────────────────────────────── */
typedef struct {
	uint16_t dom_freq_hz;
	uint16_t dom_mag;
	uint16_t total_power;
	uint16_t bpfo_energy;
	uint16_t bpfi_energy;
	uint16_t bsf_energy;
	uint16_t ftf_energy;
	uint16_t noise_floor;
	uint16_t snr_bpfo;
	uint16_t reserved;
} axis_fft_stats_t;

typedef struct {
	axis_fft_stats_t x;
	axis_fft_stats_t y;
	axis_fft_stats_t z;
} fft_stats_t;

/* ──────────────────────────────────────────────
 * Bearing fault frequency parameters
 *
 * Pre-computed fault frequencies avoid float at runtime.
 * Call analysis_set_bearing_params() after loading settings.
 *
 * Formulas (n=RPM, Nb=balls, Bd=ball diam, Pd=pitch diam, φ=contact angle):
 *   FTF  = (n/120) × (1 − Bd/Pd·cosφ)
 *   BPFO = Nb × FTF
 *   BPFI = (Nb·n/120) × (1 + Bd/Pd·cosφ)
 *   BSF  = (Pd·n/240·Bd) × (1 − (Bd/Pd·cosφ)²)
 * ────────────────────────────────────────────── */
typedef struct {
	uint16_t rpm;

	/* Fault frequencies in Hz × 10 (e.g. 72.5 Hz → 725) */
	uint16_t bpfo_hz10;
	uint16_t bpfi_hz10;
	uint16_t bsf_hz10;
	uint16_t ftf_hz10;

	/* ± half-bandwidth around each fault center (Hz) */
	uint16_t bandwidth_hz;

	uint8_t  nb;        /* Number of rolling elements */
	uint8_t  reserved;
} bearing_params_t;

/* ──────────────────────────────────────────────
 * Wire packet header (8 bytes, prepended to every BLE notification)
 * ────────────────────────────────────────────── */
typedef struct {
	uint8_t  type;
	uint8_t  reserved;
	uint16_t seq;
	uint16_t sample_count;
	uint16_t chunk_index;
} burst_header_t;

#define PKT_TYPE_TIME_STATS  0x01
#define PKT_TYPE_RAW         0x02
#define PKT_TYPE_ENV         0x03
#define PKT_TYPE_FFT_STATS   0x04

/* Legacy alias so existing fsm.c / ble_tx code compiles unchanged */
typedef time_stats_t vibration_stats_t;
#define PKT_TYPE_STATS  PKT_TYPE_TIME_STATS

/* ──────────────────────────────────────────────
 * Public API
 * ────────────────────────────────────────────── */

/**
 * Set bearing geometry + RPM for fault frequency windows.
 * Call once at boot after loading settings from flash.
 * Default: 6204 bearing @ 1450 RPM (BPFO≈72.5, BPFI≈107, BSF≈47, FTF≈9.8 Hz)
 */
void analysis_set_bearing_params(const bearing_params_t *params);

/**
 * Time-domain statistics: RMS, peak, crest factor, kurtosis, variance.
 * Single pass over buffer. ~50 µs @ 64 MHz for N=512.
 */
int analysis_compute_time_stats(const accel_sample_t *buffer,
				uint16_t count,
				time_stats_t *out);

/**
 * Frequency-domain statistics: dominant freq, band energies, SNR.
 * Runs 3× 512-point fixed-point FFT. ~15 ms @ 64 MHz.
 * Uses static scratch (no heap, no large stack frames).
 */
int analysis_compute_fft_stats(const accel_sample_t *buffer,
			       uint16_t count,
			       fft_stats_t *out);

/**
 * Run both stages in one call.
 * Total runtime: ~15–20 ms for N=512 on nRF52840.
 */
int analysis_compute_all(const accel_sample_t *buffer,
			 uint16_t count,
			 time_stats_t *time_out,
			 fft_stats_t  *fft_out);

/* Legacy shim */
static inline int analysis_compute_stats(const accel_sample_t *buffer,
					 uint16_t count,
					 vibration_stats_t *out)
{
	return analysis_compute_time_stats(buffer, count, out);
}

#endif /* ANALYSIS_H */
