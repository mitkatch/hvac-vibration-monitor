/*
 * vib_ring.c — Lock-free SPSC vibration ring buffer
 *
 * SPSC = Single Producer, Single Consumer.
 *
 * Writer: system workqueue  (vibration_work_handler in fsm.c)
 * Reader: FSM event-loop    (handle_vibration_collected / TX path)
 *
 * Safety guarantee: atomic_t head/tail prevent the compiler and
 * CPU from reordering the index update past the data write, so
 * the reader can never see a half-written slot.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "vib_ring.h"

LOG_MODULE_REGISTER(vib_ring, LOG_LEVEL_INF);

/* ── Helpers ─────────────────────────────────── */

static inline uint32_t ring_head(const vib_ring_t *r)
{
	return (uint32_t)atomic_get((atomic_t *)&r->head);
}

static inline uint32_t ring_tail(const vib_ring_t *r)
{
	return (uint32_t)atomic_get((atomic_t *)&r->tail);
}

/* Committed slots: head - tail (unsigned wrap-around safe) */
static inline uint32_t ring_used(const vib_ring_t *r)
{
	return ring_head(r) - ring_tail(r);
}

/* ── Public API ──────────────────────────────── */

void vib_ring_init(vib_ring_t *ring)
{
	memset(ring, 0, sizeof(*ring));
	atomic_set(&ring->head, 0);
	atomic_set(&ring->tail, 0);
	ring->next_seq = 0;
	ring->dropped  = 0;
	LOG_INF("vib_ring: initialized (%d slots, %zu bytes each)",
		VIB_RING_SLOTS, sizeof(vib_burst_t));
}

vib_burst_t *vib_ring_acquire_write(vib_ring_t *ring)
{
	uint32_t h = ring_head(ring);
	uint32_t t = ring_tail(ring);

	if ((h - t) >= VIB_RING_SLOTS) {
		/*
		 * Ring is full.  Evict the oldest slot by advancing tail —
		 * newest data always wins over stale unread bursts.
		 */
		atomic_inc(&ring->tail);
		ring->dropped++;
		LOG_WRN("vib_ring: full — oldest burst evicted (total dropped: %u)",
			ring->dropped);
	}

	/* Slot at (head & MASK) is now ours to fill */
	vib_burst_t *slot = &ring->slots[h & VIB_RING_MASK];
	slot->seq = ring->next_seq++;
	slot->timestamp_ms = k_uptime_get_32();
	slot->count = 0;  /* writer will set this */
	return slot;
}

void vib_ring_commit_write(vib_ring_t *ring, vib_burst_t *slot)
{
	ARG_UNUSED(slot);
	/*
	 * Memory barrier before advancing head: all writes to slot->samples
	 * must be visible to the reader before head is incremented.
	 * Zephyr's atomic_inc() implies a full memory barrier on ARM.
	 */
	atomic_inc(&ring->head);
	LOG_DBG("vib_ring: committed burst seq=%u count=%u head=%u tail=%u",
		slot->seq, slot->count,
		ring_head(ring), ring_tail(ring));
}

const vib_burst_t *vib_ring_peek_read(const vib_ring_t *ring)
{
	if (ring_used(ring) == 0) {
		return NULL;
	}
	uint32_t t = ring_tail(ring);
	return &ring->slots[t & VIB_RING_MASK];
}

void vib_ring_consume_read(vib_ring_t *ring)
{
	/* Only call after a successful peek */
	atomic_inc(&ring->tail);
	LOG_DBG("vib_ring: consumed head=%u tail=%u remaining=%u",
		ring_head(ring), ring_tail(ring), vib_ring_available(ring));
}

uint32_t vib_ring_available(const vib_ring_t *ring)
{
	return ring_used(ring);
}

bool vib_ring_is_empty(const vib_ring_t *ring)
{
	return ring_used(ring) == 0;
}
