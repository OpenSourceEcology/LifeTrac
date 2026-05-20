#include "sx1276_fhss.h"

#include <stddef.h>
#include <string.h>

/*
 * FCC §15.247(a)(1)(i) FHSS hop scheduler implementation
 * (plan §14.2 step 5, FCC-A3). See include/sx1276_fhss.h for design.
 *
 * Compile-time sanity: the legal floor must not exceed the table size,
 * otherwise blacklist() can never succeed AND the floor check itself is
 * a no-op.
 */
_Static_assert(SX1276_FHSS_LEGAL_FLOOR <= SX1276_FHSS_CHANNEL_COUNT,
               "FHSS legal floor cannot exceed channel table size");

_Static_assert(SX1276_FHSS_CHANNEL_COUNT <= 64U,
               "blacklist bitmap is uint64_t — table must fit 64 bits");

/* FNV-1a 32-bit constants (RFC reference values). */
#define FNV1A_OFFSET_BASIS_32   0x811C9DC5UL
#define FNV1A_PRIME_32          0x01000193UL

/* xorshift32 fallback seed when caller-supplied seed hashes to zero. */
#define XORSHIFT32_NONZERO_FALLBACK 0x9E3779B9UL

typedef struct {
    uint64_t farm_id;
    uint64_t node_id;
    uint32_t epoch;
    uint32_t seed;
    uint8_t  permutation[SX1276_FHSS_CHANNEL_COUNT];
    uint8_t  slot;
    uint8_t  initialized;
    uint16_t warmup_hops_remaining;
    uint64_t blacklist_bits;
    uint32_t lbt_block_count[SX1276_FHSS_CHANNEL_COUNT];
    /* FCC-B1-SUMMARY-b sidecar: per-channel hop count since last
     * snapshot. u16 in-memory to detect overflow at >65k hops/min
     * (well above any plausible workload); saturates to 0xFFU on the
     * wire because RFCO_SUMMARY.per_channel_hop_count is u8. */
    uint16_t hop_count[SX1276_FHSS_CHANNEL_COUNT];
} fhss_state_t;

static fhss_state_t s_fhss;

/* ---------- pure helpers ---------- */

static uint32_t fnv1a_32(const uint8_t *data, size_t len) {
    uint32_t h = FNV1A_OFFSET_BASIS_32;
    for (size_t i = 0; i < len; ++i) {
        h ^= (uint32_t)data[i];
        h *= FNV1A_PRIME_32;
    }
    return h;
}

uint32_t sx1276_fhss_compute_seed(uint64_t farm_id,
                                  uint64_t node_id,
                                  uint32_t epoch) {
    uint8_t buf[8 + 8 + 4];
    for (unsigned i = 0; i < 8U; ++i) {
        buf[i]     = (uint8_t)((farm_id >> (8U * i)) & 0xFFU);
        buf[8U + i] = (uint8_t)((node_id >> (8U * i)) & 0xFFU);
    }
    for (unsigned i = 0; i < 4U; ++i) {
        buf[16U + i] = (uint8_t)((epoch >> (8U * i)) & 0xFFU);
    }
    return fnv1a_32(buf, sizeof(buf));
}

static uint32_t xorshift32_step(uint32_t *state) {
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

void sx1276_fhss_compute_permutation(uint32_t seed,
                                     uint8_t out[SX1276_FHSS_CHANNEL_COUNT]) {
    /* Initialize identity. */
    for (uint8_t i = 0; i < SX1276_FHSS_CHANNEL_COUNT; ++i) {
        out[i] = i;
    }
    uint32_t rng = (seed == 0U) ? XORSHIFT32_NONZERO_FALLBACK : seed;
    /* Fisher-Yates: for i from n-1 down to 1, j = rng % (i+1), swap. */
    for (uint8_t i = (uint8_t)(SX1276_FHSS_CHANNEL_COUNT - 1U); i > 0U; --i) {
        const uint32_t r = xorshift32_step(&rng);
        const uint8_t j = (uint8_t)(r % (uint32_t)(i + 1U));
        const uint8_t tmp = out[i];
        out[i] = out[j];
        out[j] = tmp;
    }
}

/* ---------- state helpers ---------- */

static void rebuild_epoch_permutation(void) {
    s_fhss.seed = sx1276_fhss_compute_seed(s_fhss.farm_id,
                                           s_fhss.node_id,
                                           s_fhss.epoch);
    sx1276_fhss_compute_permutation(s_fhss.seed, s_fhss.permutation);
    s_fhss.slot = 0U;
}

static uint8_t popcount64(uint64_t v) {
    uint8_t c = 0;
    while (v != 0U) {
        v &= (v - 1U);
        ++c;
    }
    return c;
}

static uint8_t active_count_from_bits(uint64_t bits) {
    return (uint8_t)(SX1276_FHSS_CHANNEL_COUNT - popcount64(bits));
}

/* ---------- public API ---------- */

void sx1276_fhss_reset(void) {
    memset(&s_fhss, 0, sizeof(s_fhss));
}

sx1276_fhss_status_t sx1276_fhss_init(uint64_t farm_id,
                                      uint64_t node_id,
                                      uint32_t initial_epoch) {
    memset(&s_fhss, 0, sizeof(s_fhss));
    s_fhss.farm_id = farm_id;
    s_fhss.node_id = node_id;
    s_fhss.epoch   = initial_epoch;
    s_fhss.warmup_hops_remaining = (uint16_t)SX1276_FHSS_WARMUP_HOPS;
    rebuild_epoch_permutation();
    s_fhss.initialized = 1U;
    return SX1276_FHSS_OK;
}

sx1276_fhss_status_t sx1276_fhss_next_channel(uint8_t *out_idx,
                                              uint32_t *out_hz) {
    if (s_fhss.initialized == 0U) {
        return SX1276_FHSS_ERR_NOT_INIT;
    }
    if ((out_idx == NULL) || (out_hz == NULL)) {
        return SX1276_FHSS_ERR_NULL_ARG;
    }

    /* Walk forward at most CHANNEL_COUNT slots to find a non-blacklisted
     * channel. With the 50-channel legal-floor rule blacklist() always
     * refuses, so this loop normally terminates on the first iteration. */
    for (uint8_t attempts = 0; attempts < SX1276_FHSS_CHANNEL_COUNT; ++attempts) {
        if (s_fhss.slot >= SX1276_FHSS_CHANNEL_COUNT) {
            /* Slot wrap → auto-advance epoch (resets slot to 0). */
            ++s_fhss.epoch;
            rebuild_epoch_permutation();
        }
        const uint8_t ch = s_fhss.permutation[s_fhss.slot];
        if (ch >= SX1276_FHSS_CHANNEL_COUNT) {
            return SX1276_FHSS_ERR_INTERNAL; /* permutation corrupted */
        }
        const uint64_t bit = ((uint64_t)1U << ch);
        s_fhss.slot = (uint8_t)(s_fhss.slot + 1U);
        if ((s_fhss.blacklist_bits & bit) != 0U) {
            continue; /* skip blacklisted, try next slot */
        }
        const uint32_t hz = sx1276_fhss_chantab_center_hz(ch);
        if (hz == 0UL) {
            return SX1276_FHSS_ERR_INTERNAL;
        }
        *out_idx = ch;
        *out_hz  = hz;
        if (s_fhss.warmup_hops_remaining > 0U) {
            --s_fhss.warmup_hops_remaining;
        }
        return SX1276_FHSS_OK;
    }
    /* Could only get here if the entire active set is blacklisted, which
     * the floor rule forbids. Treat as a hard fault. */
    return SX1276_FHSS_ERR_INTERNAL;
}

sx1276_fhss_status_t sx1276_fhss_epoch_advance(void) {
    if (s_fhss.initialized == 0U) {
        return SX1276_FHSS_ERR_NOT_INIT;
    }
    ++s_fhss.epoch;
    rebuild_epoch_permutation();
    return SX1276_FHSS_OK;
}

sx1276_fhss_status_t sx1276_fhss_snap_to(uint32_t target_epoch,
                                         uint8_t  snap_to_slot) {
    if (s_fhss.initialized == 0U) {
        return SX1276_FHSS_ERR_NOT_INIT;
    }
    if (snap_to_slot > SX1276_FHSS_CHANNEL_COUNT) {
        return SX1276_FHSS_ERR_BAD_IDX;
    }
    /* Order is important: rebuild_epoch_permutation() resets slot to 0,
     * so set epoch first, rebuild, then overwrite slot. Blacklist bits
     * and LBT-block counters are intentionally preserved. */
    s_fhss.epoch = target_epoch;
    rebuild_epoch_permutation();
    s_fhss.slot                  = snap_to_slot;
    s_fhss.warmup_hops_remaining = 0U;
    return SX1276_FHSS_OK;
}

sx1276_fhss_snap_decision_t sx1276_fhss_consider_remote(uint32_t remote_epoch,
                                                        uint8_t  remote_hop_idx) {
    if (s_fhss.initialized == 0U) {
        return SX1276_FHSS_SNAP_DEC_REJECTED_NOT_INIT;
    }
    if (remote_hop_idx >= SX1276_FHSS_CHANNEL_COUNT) {
        return SX1276_FHSS_SNAP_DEC_REJECTED_BAD_HOP;
    }

    /* Local hop_idx = slot just consumed (matches sx1276_tx.c stamping
     * convention so ALIGNED is symmetric with TX). */
    const uint8_t local_hop_idx = (s_fhss.slot == 0U)
        ? (uint8_t)(SX1276_FHSS_CHANNEL_COUNT - 1U)
        : (uint8_t)(s_fhss.slot - 1U);

    if ((remote_epoch == s_fhss.epoch) && (remote_hop_idx == local_hop_idx)) {
        return SX1276_FHSS_SNAP_DEC_ALIGNED;
    }

    /* uint32_t subtraction wraps cleanly across epoch=0 boundary;
     * casting the difference to int32_t yields the signed delta
     * provided |Δ| <= INT32_MAX (always true for our ±1 tolerance). */
    const int32_t  delta     = (int32_t)(remote_epoch - s_fhss.epoch);
    const uint32_t abs_delta = (delta < 0) ? (uint32_t)(-delta) : (uint32_t)delta;
    if (abs_delta > SX1276_FHSS_SNAP_MAX_EPOCH_DRIFT) {
        return SX1276_FHSS_SNAP_DEC_REJECTED_EPOCH_DRIFT;
    }

    /* Remote just emitted on remote_hop_idx; next slot it will use is
     * remote_hop_idx + 1. snap_to() handles the +1 == CHANNEL_COUNT
     * wrap path by parking slot at CHANNEL_COUNT (next next_channel()
     * trips the auto-advance). */
    const uint8_t next_slot = (uint8_t)(remote_hop_idx + 1U);
    (void)sx1276_fhss_snap_to(remote_epoch, next_slot);
    return SX1276_FHSS_SNAP_DEC_SNAPPED;
}

sx1276_fhss_status_t sx1276_fhss_blacklist(uint8_t idx) {
    if (s_fhss.initialized == 0U) {
        return SX1276_FHSS_ERR_NOT_INIT;
    }
    if (idx >= SX1276_FHSS_CHANNEL_COUNT) {
        return SX1276_FHSS_ERR_BAD_IDX;
    }
    if (s_fhss.warmup_hops_remaining > 0U) {
        return SX1276_FHSS_ERR_BLACKLIST_WARMUP;
    }
    const uint64_t bit = ((uint64_t)1U << idx);
    if ((s_fhss.blacklist_bits & bit) != 0U) {
        return SX1276_FHSS_ERR_ALREADY_BLACKED;
    }
    const uint64_t prospective = s_fhss.blacklist_bits | bit;
    if (active_count_from_bits(prospective) < SX1276_FHSS_LEGAL_FLOOR) {
        return SX1276_FHSS_ERR_BLACKLIST_FLOOR;
    }
    s_fhss.blacklist_bits = prospective;
    return SX1276_FHSS_OK;
}

uint8_t sx1276_fhss_active_count(void) {
    return active_count_from_bits(s_fhss.blacklist_bits);
}

void sx1276_fhss_record_lbt_block(uint8_t idx) {
    if (idx >= SX1276_FHSS_CHANNEL_COUNT) {
        return;
    }
    if (s_fhss.lbt_block_count[idx] != UINT32_MAX) {
        ++s_fhss.lbt_block_count[idx];
    }
}

uint32_t sx1276_fhss_lbt_block_count(uint8_t idx) {
    if (idx >= SX1276_FHSS_CHANNEL_COUNT) {
        return 0UL;
    }
    return s_fhss.lbt_block_count[idx];
}

/* FCC-B1-SUMMARY-b sidecar: per-channel hop counter. Saturates at
 * UINT16_MAX in-memory; the wire field saturates further to 0xFFU. */
void sx1276_fhss_record_hop(uint8_t idx) {
    if (idx >= SX1276_FHSS_CHANNEL_COUNT) {
        return;
    }
    if (s_fhss.hop_count[idx] != UINT16_MAX) {
        ++s_fhss.hop_count[idx];
    }
}

void sx1276_fhss_hop_count_snapshot_and_clear(uint8_t out[SX1276_FHSS_CHANNEL_COUNT]) {
    if (out == NULL) {
        /* Preserve counters: a caller bug must not silently lose a
         * minute of compliance evidence. */
        return;
    }
    for (uint8_t i = 0; i < SX1276_FHSS_CHANNEL_COUNT; ++i) {
        const uint16_t v = s_fhss.hop_count[i];
        out[i] = (v > 0xFFU) ? 0xFFU : (uint8_t)v;
        s_fhss.hop_count[i] = 0U;
    }
}

uint32_t sx1276_fhss_current_epoch(void) { return s_fhss.epoch; }
uint8_t  sx1276_fhss_current_slot(void)  { return s_fhss.slot; }
uint32_t sx1276_fhss_current_seed(void)  { return s_fhss.seed; }
uint8_t  sx1276_fhss_is_initialized(void) { return s_fhss.initialized; }
