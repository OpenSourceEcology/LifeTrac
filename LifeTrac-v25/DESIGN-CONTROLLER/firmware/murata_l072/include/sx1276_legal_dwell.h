#ifndef LIFETRAC_MURATA_L072_SX1276_LEGAL_DWELL_H
#define LIFETRAC_MURATA_L072_SX1276_LEGAL_DWELL_H

/*
 * FCC §15.247(a)(1)(i) legal-dwell accountant (plan §14.2 step 6, FCC-A2).
 *
 * SEPARATE from the existing 1 s / 400 ms QoS fairness gate in
 * sx1276_airtime.c. Keeping them separate is a hard naming-discipline
 * requirement of the implementation plan (delta #11) so that bench
 * artifacts can NEVER conflate `qos_used_us_1s` with
 * `legal_dwell_used_us_10s`.
 *
 * Semantics:
 *
 *   - Per-channel rolling window with HALF-OPEN inclusion:
 *
 *         used_us(ch, now, W) = sum of durations of events e where
 *             e.channel == ch AND e.start_ms in [now - W, now)
 *
 *     i.e. an event with start_ms == now - W is NOT included; an event
 *     with start_ms == now - W + 1 IS included.
 *
 *   - Two windows supported via a `window_ms` parameter on the query +
 *     reserve calls:
 *         10000 ms ("legal_dwell_used_us_10s") — default FCC §15.247
 *                  narrowband FHSS rule.
 *         20000 ms ("legal_dwell_used_us_20s") — narrowband fallback
 *                  profile (gated on by FCC-A5 cfg validation, not by
 *                  this module).
 *
 *   - Pessimistic reserve before TX-start: caller passes the worst-case
 *     ToA estimate (typically the FCC-A1b prediction). The accountant
 *     immediately books `pessimistic_us` against the channel's window.
 *     A handle is returned so the caller can reconcile later.
 *
 *   - Reconcile on TX-done: caller passes the ACTUAL ToA measured by
 *     the TX-done IRQ. If `actual_us < reserved_us`, the event's
 *     duration is shrunk to `actual_us`. If `actual_us >= reserved_us`,
 *     no-op (we never grow beyond the original pessimistic reserve, so
 *     we cannot under-account on a transmission that ran longer than
 *     planned).
 *
 *   - NEVER ROLL BACK on NACK: there is no public `release()` API.
 *     Energy emitted is energy emitted, regardless of whether the
 *     receiver ACKed. Doing otherwise would let an interfering ACK
 *     channel "free up" legal dwell that was actually consumed.
 *
 * Fails closed:
 *   - reserve() on a channel >= SX1276_DWELL_CHANNEL_COUNT returns BAD_CH,
 *   - reserve() with pessimistic_us == 0 OR > cap_us returns BAD_RESERVE,
 *   - reserve() that would push the channel over `cap_us` in the
 *     requested window returns OVER_BUDGET (and books nothing),
 *   - reserve() when the shared event ring is full of live events
 *     returns RING_FULL (and books nothing),
 *   - reconcile() with a stale or out-of-range handle is silently
 *     dropped (does NOT mutate any other slot).
 *
 * Time source: caller supplies `now_ms` from the existing monotonic ms
 * tick. The module never reads the tick directly so tests stay
 * deterministic. Wrap handling is the caller's responsibility per
 * FCC notes §17.3 #2; the implementation uses unsigned subtraction
 * so a single 32-bit wrap is tolerated as long as the window is
 * smaller than 2^31 ms (~24.8 days).
 */

#include <stdbool.h>
#include <stdint.h>

#define SX1276_DWELL_CHANNEL_COUNT     64U
#define SX1276_DWELL_WINDOW_10S_MS     10000UL
#define SX1276_DWELL_WINDOW_20S_MS     20000UL
#define SX1276_DWELL_DEFAULT_CAP_US    400000UL
#define SX1276_DWELL_RING_CAPACITY     128U
#define SX1276_DWELL_HANDLE_INVALID    0xFFFFU

typedef enum {
    SX1276_DWELL_OK           = 0,
    SX1276_DWELL_BAD_CH       = 1,
    SX1276_DWELL_BAD_RESERVE  = 2,
    SX1276_DWELL_BAD_WINDOW   = 3,
    SX1276_DWELL_OVER_BUDGET  = 4,
    SX1276_DWELL_RING_FULL    = 5,
} sx1276_dwell_status_t;

/* Reset all module state. Idempotent. */
void sx1276_legal_dwell_reset(void);

/* Reserve `pessimistic_us` of legal dwell against `channel_idx`.
 *
 * Inserts an event with start_ms=now_ms and duration=pessimistic_us if
 *   used_us(ch, now, window_ms) + pessimistic_us <= cap_us
 *
 * On success returns SX1276_DWELL_OK and writes an opaque handle to
 * *out_handle that may be passed to sx1276_legal_dwell_reconcile().
 *
 * On failure, the ring is unmodified and *out_handle is set to
 * SX1276_DWELL_HANDLE_INVALID. */
sx1276_dwell_status_t sx1276_legal_dwell_reserve(uint8_t channel_idx,
                                                 uint32_t pessimistic_us,
                                                 uint32_t now_ms,
                                                 uint32_t window_ms,
                                                 uint32_t cap_us,
                                                 uint16_t *out_handle);

/* Reconcile a previously reserved event to its actual airtime.
 * If actual_us < reserved_us, shrinks the event. Otherwise no-op.
 * Stale / invalid / out-of-range handles are silently dropped. */
void sx1276_legal_dwell_reconcile(uint16_t handle, uint32_t actual_us);

/* Used-us in [now_ms - window_ms, now_ms) for one channel. */
uint32_t sx1276_legal_dwell_used_us(uint8_t channel_idx,
                                    uint32_t window_ms,
                                    uint32_t now_ms);

/* Number of currently-live events in the ring (events whose start_ms
 * is in [now - SX1276_DWELL_WINDOW_20S_MS, now)). Diagnostics only. */
uint8_t sx1276_legal_dwell_live_event_count(uint32_t now_ms);

/* FCC-B1-SUMMARY-b sidecar: per-channel peak-used-us snapshot.
 *
 * The module internally tracks, for each channel, the high-water mark
 * of `used_us_after_reserve` (i.e. used-before-reserve + pessimistic_us)
 * observed across all successful reserve() calls since the last
 * snapshot. This is the same "worst-case high-water mark" semantic
 * that the per-TX RFCO_PERTX URC's `legal_dwell_used_us_10s` field
 * reports per-TX; the snapshot here aggregates the peak over the
 * SUMMARY window (typically 60 s).
 *
 * Reservations that fail (BAD_CH, BAD_RESERVE, BAD_WINDOW,
 * OVER_BUDGET, RING_FULL) do NOT update the peak — only energy that
 * was actually booked counts toward the high-water mark.
 *
 * snapshot_and_clear copies `peak_used_us[0..SX1276_DWELL_CHANNEL_COUNT-1]`
 * into `out` and zeroes the in-memory peaks. NULL out is a no-op
 * (peaks preserved so a caller bug does not lose compliance evidence).
 * Pure observability — does NOT affect reserve/reconcile semantics. */
void sx1276_legal_dwell_peak_us_snapshot_and_clear(
    uint32_t out[SX1276_DWELL_CHANNEL_COUNT]);

#endif /* LIFETRAC_MURATA_L072_SX1276_LEGAL_DWELL_H */
