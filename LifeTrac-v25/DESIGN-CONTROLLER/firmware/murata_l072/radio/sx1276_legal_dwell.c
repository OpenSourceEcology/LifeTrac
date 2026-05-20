#include "sx1276_legal_dwell.h"

#include <stddef.h>
#include <string.h>

/*
 * FCC §15.247(a)(1)(i) legal-dwell accountant implementation
 * (plan §14.2 step 6, FCC-A2). See include/sx1276_legal_dwell.h.
 *
 * Storage model: a single shared ring of events across all 64 channels.
 * Each event carries its own channel index, so queries iterate the
 * whole ring and filter. This is O(ring_capacity) per op which at 128
 * entries is trivially cheap on the M0+; the alternative — per-channel
 * mini-rings — would either waste RAM (64 * 8 * 8 bytes = 4 KiB) or
 * impose a hard cap on bursts per channel.
 *
 * Handle encoding: [9 bits generation | 7 bits slot index]. Generation
 * wraps mod 512; collisions are vanishingly rare in practice and a
 * collision only re-targets a same-slot reservation, which can never
 * grow the booked legal dwell (reconcile is monotonically downward).
 */

_Static_assert(SX1276_DWELL_RING_CAPACITY <= 128U,
               "ring capacity must fit in 7 bits of the handle");

_Static_assert(SX1276_DWELL_CHANNEL_COUNT <= 64U,
               "channel index must fit in uint8_t with room to spare");

#define DWELL_HANDLE_SLOT_MASK  0x007FU
#define DWELL_HANDLE_GEN_SHIFT  7U
#define DWELL_HANDLE_GEN_MASK   0x01FFU /* 9 bits */

typedef struct {
    uint32_t start_ms;
    uint32_t duration_us;
    uint16_t generation;
    uint8_t  channel_idx;
    uint8_t  active;
} dwell_event_t;

typedef struct {
    dwell_event_t ring[SX1276_DWELL_RING_CAPACITY];
    /* FCC-B1-SUMMARY-b sidecar: per-channel peak `used_after_reserve`
     * since last snapshot. Zeroed by reset() (memset) and by
     * peak_us_snapshot_and_clear(). */
    uint32_t peak_used_us[SX1276_DWELL_CHANNEL_COUNT];
} dwell_state_t;

static dwell_state_t s_dwell;

/* Half-open inclusion: event with start_ms == now - window_ms is OUT.
 * Use unsigned subtraction so a single 32-bit wrap is tolerated. */
static bool event_in_window(uint32_t event_start_ms,
                            uint32_t now_ms,
                            uint32_t window_ms) {
    const uint32_t age_ms = now_ms - event_start_ms;
    return (age_ms < window_ms);
}

static uint16_t encode_handle(uint8_t slot, uint16_t gen) {
    return (uint16_t)(((uint16_t)(gen & DWELL_HANDLE_GEN_MASK)
                       << DWELL_HANDLE_GEN_SHIFT) |
                      (uint16_t)(slot & DWELL_HANDLE_SLOT_MASK));
}

static uint8_t decode_handle_slot(uint16_t handle) {
    return (uint8_t)(handle & DWELL_HANDLE_SLOT_MASK);
}

static uint16_t decode_handle_gen(uint16_t handle) {
    return (uint16_t)((handle >> DWELL_HANDLE_GEN_SHIFT) & DWELL_HANDLE_GEN_MASK);
}

/* Find a free slot, preferring slots whose event has aged out of the
 * largest enforcement window (20 s) since those will never contribute
 * to any future query. */
static bool find_reuseable_slot(uint32_t now_ms, uint8_t *out_slot) {
    /* First pass: a truly empty slot. */
    for (uint8_t i = 0; i < SX1276_DWELL_RING_CAPACITY; ++i) {
        if (s_dwell.ring[i].active == 0U) {
            *out_slot = i;
            return true;
        }
    }
    /* Second pass: a slot whose event is older than the longest
     * supported window — safe to reuse since no query can ever see it. */
    for (uint8_t i = 0; i < SX1276_DWELL_RING_CAPACITY; ++i) {
        if (!event_in_window(s_dwell.ring[i].start_ms, now_ms,
                             SX1276_DWELL_WINDOW_20S_MS)) {
            *out_slot = i;
            return true;
        }
    }
    return false;
}

void sx1276_legal_dwell_reset(void) {
    memset(&s_dwell, 0, sizeof(s_dwell));
}

uint32_t sx1276_legal_dwell_used_us(uint8_t channel_idx,
                                    uint32_t window_ms,
                                    uint32_t now_ms) {
    if (channel_idx >= SX1276_DWELL_CHANNEL_COUNT) {
        return 0UL;
    }
    if ((window_ms == 0U) || (window_ms > SX1276_DWELL_WINDOW_20S_MS)) {
        return 0UL;
    }
    uint32_t total = 0UL;
    for (uint8_t i = 0; i < SX1276_DWELL_RING_CAPACITY; ++i) {
        const dwell_event_t *e = &s_dwell.ring[i];
        if (e->active == 0U) continue;
        if (e->channel_idx != channel_idx) continue;
        if (!event_in_window(e->start_ms, now_ms, window_ms)) continue;
        /* Saturating add — total is uint32 and a single channel cap is
         * 400 000, so overflow requires >10 000 booked events on one
         * channel; defensive only. */
        if ((UINT32_MAX - total) < e->duration_us) {
            return UINT32_MAX;
        }
        total += e->duration_us;
    }
    return total;
}

sx1276_dwell_status_t sx1276_legal_dwell_reserve(uint8_t channel_idx,
                                                 uint32_t pessimistic_us,
                                                 uint32_t now_ms,
                                                 uint32_t window_ms,
                                                 uint32_t cap_us,
                                                 uint16_t *out_handle) {
    if (out_handle != NULL) {
        *out_handle = SX1276_DWELL_HANDLE_INVALID;
    }
    if (channel_idx >= SX1276_DWELL_CHANNEL_COUNT) {
        return SX1276_DWELL_BAD_CH;
    }
    if ((window_ms == 0U) || (window_ms > SX1276_DWELL_WINDOW_20S_MS)) {
        return SX1276_DWELL_BAD_WINDOW;
    }
    if ((pessimistic_us == 0UL) || (cap_us == 0UL) || (pessimistic_us > cap_us)) {
        return SX1276_DWELL_BAD_RESERVE;
    }

    const uint32_t used = sx1276_legal_dwell_used_us(channel_idx, window_ms, now_ms);
    /* used + pessimistic > cap, with overflow safety. */
    if ((cap_us - used) < pessimistic_us) {
        return SX1276_DWELL_OVER_BUDGET;
    }

    uint8_t slot;
    if (!find_reuseable_slot(now_ms, &slot)) {
        return SX1276_DWELL_RING_FULL;
    }

    dwell_event_t *e = &s_dwell.ring[slot];
    /* Bump generation BEFORE writing so a stale handle to the previous
     * occupant of this slot will mismatch and be silently dropped. */
    e->generation = (uint16_t)((e->generation + 1U) & DWELL_HANDLE_GEN_MASK);
    e->start_ms    = now_ms;
    e->duration_us = pessimistic_us;
    e->channel_idx = channel_idx;
    e->active      = 1U;

    /* FCC-B1-SUMMARY-b: update per-channel peak high-water mark.
     * Post-reserve usage is exactly (used_before + pessimistic_us)
     * since we just inserted an event with start_ms == now_ms and the
     * half-open window [now_ms - W, now_ms) includes age=0. */
    const uint32_t used_after = used + pessimistic_us;
    if (used_after > s_dwell.peak_used_us[channel_idx]) {
        s_dwell.peak_used_us[channel_idx] = used_after;
    }

    if (out_handle != NULL) {
        *out_handle = encode_handle(slot, e->generation);
    }
    return SX1276_DWELL_OK;
}

void sx1276_legal_dwell_reconcile(uint16_t handle, uint32_t actual_us) {
    if (handle == SX1276_DWELL_HANDLE_INVALID) {
        return;
    }
    const uint8_t  slot = decode_handle_slot(handle);
    const uint16_t gen  = decode_handle_gen(handle);
    if (slot >= SX1276_DWELL_RING_CAPACITY) {
        return;
    }
    dwell_event_t *e = &s_dwell.ring[slot];
    if (e->active == 0U) {
        return;
    }
    if (e->generation != gen) {
        /* Stale handle — slot was reused. Drop silently. */
        return;
    }
    /* Monotonically downward only: never grow legal dwell beyond the
     * pessimistic reservation. */
    if (actual_us < e->duration_us) {
        e->duration_us = actual_us;
    }
    /* No rollback on actual_us == 0: that path would imply TX never
     * emitted, which is the caller's responsibility to track via the
     * upstream LBT / invariant-reject counters. We still leave the
     * event booked because we cannot distinguish "TX never started"
     * from "TX completed in negligible time" at this layer. */
}

uint8_t sx1276_legal_dwell_live_event_count(uint32_t now_ms) {
    uint8_t count = 0;
    for (uint8_t i = 0; i < SX1276_DWELL_RING_CAPACITY; ++i) {
        const dwell_event_t *e = &s_dwell.ring[i];
        if (e->active == 0U) continue;
        if (event_in_window(e->start_ms, now_ms, SX1276_DWELL_WINDOW_20S_MS)) {
            ++count;
        }
    }
    return count;
}

void sx1276_legal_dwell_peak_us_snapshot_and_clear(
    uint32_t out[SX1276_DWELL_CHANNEL_COUNT]) {
    if (out == NULL) {
        /* Preserve peaks: a caller bug must not silently lose a minute
         * of compliance evidence. */
        return;
    }
    for (uint8_t i = 0; i < SX1276_DWELL_CHANNEL_COUNT; ++i) {
        out[i] = s_dwell.peak_used_us[i];
        s_dwell.peak_used_us[i] = 0UL;
    }
}
