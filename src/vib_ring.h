/*
 * vib_ring.h — Lock-free vibration ring buffer
 *
 * Holds completed burst "slots" — each slot is one full
 * VIBRATION_BURST_SIZE collection. The ADXL343 work item writes
 * to the next free slot; the BLE TX stage reads the oldest slot.
 *
 * Concurrency model
 * -----------------
 *   Writer: system workqueue (vibration_work_handler)
 *   Reader: FSM event-loop thread (TRANSMITTING stage)
 *
 * There is exactly ONE writer and ONE reader — classic SPSC ring.
 * We use atomic head/tail indices so no mutex is needed. The
 * in-progress slot (being filled by the writer) is NOT visible
 * to the reader until the writer commits it by advancing head.
 *
 * Slot layout
 * -----------
 *   [ accel_sample_t samples[VIBRATION_BURST_SIZE] ]   — raw IQ
 *   [ uint16_t       count                         ]   — valid samples
 *   [ uint32_t       timestamp_ms                  ]   — k_uptime_get_32
 *   [ uint16_t       seq                           ]   — monotonic burst #
 *
 * Capacity
 * --------
 *   VIB_RING_SLOTS slots.  At 512 samples × 6 bytes = 3072 bytes per slot,
 *   4 slots = ~12 KB — comfortable for the nRF52840's 256 KB SRAM.
 *   Increase if you want more buffering when the hub is temporarily offline.
 */

#ifndef VIB_RING_H
#define VIB_RING_H

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>
#include "fsm.h"   /* accel_sample_t, VIBRATION_BURST_SIZE */

/* Number of burst slots in the ring.  Must be a power of two. */
#define VIB_RING_SLOTS      4
#define VIB_RING_MASK       (VIB_RING_SLOTS - 1)

/* One completed burst */
typedef struct {
	accel_sample_t  samples[VIBRATION_BURST_SIZE];
	uint16_t        count;        /* Actual samples collected (≤ VIBRATION_BURST_SIZE) */
	uint32_t        timestamp_ms; /* k_uptime_get_32() at collection start */
	uint16_t        seq;          /* Monotonic burst sequence number */
} vib_burst_t;

/* Ring buffer state — zero-initialise before use */
typedef struct {
	vib_burst_t     slots[VIB_RING_SLOTS];

	/* SPSC indices — only writer touches head, only reader touches tail.
	 * Both are atomically accessed so the compiler cannot reorder loads/stores. */
	atomic_t        head;   /* next slot for writer (mod SLOTS) */
	atomic_t        tail;   /* next slot for reader (mod SLOTS) */

	uint16_t        next_seq;   /* written only by work-item thread */
	uint32_t        dropped;    /* bursts dropped due to full ring */
} vib_ring_t;

/* ── API ─────────────────────────────────────── */

/** Initialise ring to empty state */
void vib_ring_init(vib_ring_t *ring);

/**
 * Acquire a slot for writing.
 * Returns pointer to slot, or NULL if ring is full (overflow).
 * On overflow the OLDEST slot is recycled (head advances) and
 * ring->dropped is incremented — newest data always wins.
 *
 * Call from writer thread only (system workqueue).
 */
vib_burst_t *vib_ring_acquire_write(vib_ring_t *ring);

/**
 * Commit a written slot, making it visible to the reader.
 * Must be called after vib_ring_acquire_write() and after
 * filling the slot with valid data.
 *
 * Call from writer thread only.
 */
void vib_ring_commit_write(vib_ring_t *ring, vib_burst_t *slot);

/**
 * Peek at the oldest readable slot.
 * Returns pointer to slot, or NULL if ring is empty.
 * Does NOT advance tail — call vib_ring_consume_read() after TX.
 *
 * Call from reader thread only (FSM event loop).
 */
const vib_burst_t *vib_ring_peek_read(const vib_ring_t *ring);

/**
 * Consume (release) the oldest slot after successful transmission.
 * Must only be called after a successful vib_ring_peek_read().
 *
 * Call from reader thread only.
 */
void vib_ring_consume_read(vib_ring_t *ring);

/** Number of ready (committed) slots available for reading */
uint32_t vib_ring_available(const vib_ring_t *ring);

/** True if no committed slots are waiting */
bool vib_ring_is_empty(const vib_ring_t *ring);

#endif /* VIB_RING_H */
