/*
 * thread.h — Thread network transport (OpenThread + CoAP NON POST)
 */

#ifndef THREAD_H
#define THREAD_H

#include <stdbool.h>
#include <stdint.h>
#include "fsm.h"
#include "analysis.h"

/**
 * Initialise OpenThread, apply hardcoded network credentials, register the
 * role-change callback that posts EVT_THREAD_CONNECTED / EVT_THREAD_DISCONNECTED
 * to the FSM, and start the Thread stack.
 *
 * @param cb  FSM event_post function — called from the OpenThread thread.
 * @return 0 on success, negative errno on failure.
 */
int thread_init(event_post_fn cb);

/** True when the Thread role is Child or Router (network is usable). */
bool thread_is_connected(void);

/**
 * Send one burst as up to 3 CoAP NON POST packets to the Thread leader's RLOC.
 * Each packet is small enough to fit in a single 802.15.4 frame (no 6LoWPAN
 * fragmentation).  All packets carry the same seq + timestamp_ms so the
 * backend can group them by (source_addr, seq).
 *
 *   PKT_TYPE_TIME_STATS  burst_header(10) + time_stats(36)  = 46 B payload
 *   PKT_TYPE_FFT_STATS   burst_header(10) + fft_stats(60)   = 70 B payload
 *   PKT_TYPE_ENV         burst_header(10) + env(8)           = 18 B payload  (if valid)
 *
 * NON = fire-and-forget; returns 0 as soon as all datagrams are queued.
 *
 * @param seq           Burst sequence number from ring slot.
 * @param count         Sample count the stats were computed from.
 * @param timestamp_ms  k_uptime_get_32() recorded at burst collection.
 * @param ts            Time-domain feature vector (never NULL).
 * @param fs            Frequency-domain feature vector (never NULL).
 * @param env           Environmental reading, or NULL / env->valid==false to omit.
 * @return 0 on success, negative errno on error.
 */
int thread_publish_burst(uint16_t seq, uint16_t count, uint32_t timestamp_ms,
			 const time_stats_t  *ts,
			 const fft_stats_t   *fs,
			 const env_reading_t *env);

#endif /* THREAD_H */
