#ifndef LIFETRAC_MURATA_L072_SX1276_FHSS_H
#define LIFETRAC_MURATA_L072_SX1276_FHSS_H

/*
 * FCC §15.247(a)(1)(i) FHSS hop scheduler (plan §14.2 step 5, FCC-A3).
 *
 * Consumes the 50-channel center-frequency table from
 * include/sx1276_fhss_chantab.h. Per super-frame (epoch), generates a
 * pseudo-random permutation of channel indices keyed on
 *
 *     seed = H(farm_id || node_id || epoch)
 *
 * H is FNV-1a 32-bit over the little-endian byte-concatenation of the
 * three inputs (20 bytes). The permutation is built by Fisher-Yates
 * seeded with xorshift32(seed). One channel per slot, full set per
 * epoch — equal-use by construction.
 *
 * Fails closed:
 *   - any public API called before sx1276_fhss_init() returns an error
 *     enum (NOT a default channel),
 *   - blacklist that would drop the active set below the legal floor
 *     (50) is REFUSED (FCC notes §10.6 #5),
 *   - blacklist is also refused during cold-start warm-up (FCC notes
 *     §17.3 #3: first full epoch must traverse the hopset),
 *   - out-of-range indices and detected internal inconsistency return
 *     non-OK without mutating state.
 *
 * Time source: the caller drives sx1276_fhss_epoch_advance() from the
 * existing monotonic ms tick layer. This module never reads the tick
 * directly so that golden vectors remain deterministic.
 *
 * record_lbt_block() is decoupled from blacklist() (plan §14.2 step 5):
 * it only accumulates per-channel quality statistics for RFCO telemetry
 * (Track B) and never modifies the active set.
 *
 * Single source of truth — host-side mirror at
 *   bench/host_proto/fhss_scheduler.py
 * MUST stay bit-identical (golden vectors in bench/host_proto/
 * cross-check both implementations).
 */

#include <stdint.h>

#include "sx1276_fhss_chantab.h"

/* Legal floor from FCC §15.247(a)(1)(i): at least 50 hopping channels
 * for narrowband (BW < 250 kHz) FHSS. */
#define SX1276_FHSS_LEGAL_FLOOR        50U

/* Cold-start warm-up length in hop slots: one full epoch (50 hops)
 * must complete before any blacklist request is allowed. */
#define SX1276_FHSS_WARMUP_HOPS        50U

typedef enum {
    SX1276_FHSS_OK                   = 0,
    SX1276_FHSS_ERR_NOT_INIT         = 1,
    SX1276_FHSS_ERR_NULL_ARG         = 2,
    SX1276_FHSS_ERR_BAD_IDX          = 3,
    SX1276_FHSS_ERR_BLACKLIST_FLOOR  = 4,
    SX1276_FHSS_ERR_BLACKLIST_WARMUP = 5,
    SX1276_FHSS_ERR_ALREADY_BLACKED  = 6,
    SX1276_FHSS_ERR_INTERNAL         = 7,
} sx1276_fhss_status_t;

/* Reset all module state to pre-init. Idempotent. Intended for unit
 * tests and for the safe-mode reset path. */
void sx1276_fhss_reset(void);

/* Seed the scheduler. Must be called before any other non-reset API.
 * Always succeeds (no rejected inputs at the API boundary). */
sx1276_fhss_status_t sx1276_fhss_init(uint64_t farm_id,
                                      uint64_t node_id,
                                      uint32_t initial_epoch);

/* Return the next (channel_idx, center_hz) and advance the slot pointer.
 * Skips blacklisted channels (in current implementation the legal-floor
 * rule prevents any blacklist, so the skip path is exercised only via
 * the hop-state unit test harness). Auto-advances the epoch when slot
 * wraps past SX1276_FHSS_CHANNEL_COUNT. */
sx1276_fhss_status_t sx1276_fhss_next_channel(uint8_t *out_idx,
                                              uint32_t *out_hz);

/* Manually advance to the next epoch (slot resets to 0, permutation
 * regenerated from new seed). Called by RX hop sync (FCC-A6) and by
 * next_channel() on slot wrap. */
sx1276_fhss_status_t sx1276_fhss_epoch_advance(void);

/* FCC-A6b-2-i (plan §14.2 step 10): force the scheduler to a specific
 * (epoch, slot) position. Used by the RX hop-sync path once it decides
 * to trust an incoming A6a header — see sx1276_fhss_consider_remote()
 * (A6b-2-ii) for the snap-policy layer that gates calls into this API.
 *
 * Semantics:
 *   - `target_epoch` becomes the new current epoch verbatim. Permutation
 *     is rebuilt from H(farm_id, node_id, target_epoch) — the active set
 *     blacklist is preserved.
 *   - `snap_to_slot` is the slot the NEXT call to sx1276_fhss_next_channel
 *     will hand out. Valid range is [0, SX1276_FHSS_CHANNEL_COUNT];
 *     passing == CHANNEL_COUNT is legal and means "the new epoch is
 *     already exhausted — the very next next_channel() call will trip
 *     the wrap path and auto-advance to (target_epoch + 1, slot 0)".
 *   - Clears warmup_hops_remaining to 0: snapping to a remote implies
 *     the remote is already past its own warm-up and the local node
 *     has joined an active hopset. Blacklist bits and LBT-block
 *     counters are preserved (per-channel quality history survives).
 *   - On error, scheduler state is NOT mutated.
 *
 * Failure modes:
 *   - NOT_INIT if called before sx1276_fhss_init().
 *   - BAD_IDX  if snap_to_slot > SX1276_FHSS_CHANNEL_COUNT. */
sx1276_fhss_status_t sx1276_fhss_snap_to(uint32_t target_epoch,
                                         uint8_t  snap_to_slot);

/* FCC-A6b-2-ii-alpha (plan §14.2 step 10): snap-policy decision returned
 * by sx1276_fhss_consider_remote(). Distinct from sx1276_fhss_status_t
 * because "REJECTED" is an outcome of an authorized API call, not an
 * error. */
typedef enum {
    /* Remote's (epoch, hop_idx) matches the slot we just consumed —
     * no scheduler mutation, local clock is already in lock-step. */
    SX1276_FHSS_SNAP_DEC_ALIGNED = 0,
    /* Remote was within tolerance; snap_to() was called and scheduler
     * is now aligned to the remote's next slot. */
    SX1276_FHSS_SNAP_DEC_SNAPPED = 1,
    /* consider_remote called before init — no mutation. */
    SX1276_FHSS_SNAP_DEC_REJECTED_NOT_INIT = 2,
    /* remote_hop_idx >= SX1276_FHSS_CHANNEL_COUNT (malformed header) —
     * no mutation. */
    SX1276_FHSS_SNAP_DEC_REJECTED_BAD_HOP = 3,
    /* |remote_epoch - local_epoch| > SX1276_FHSS_SNAP_MAX_EPOCH_DRIFT.
     * Treated as a spoof / replay until MIC ships in schema_ver=2 —
     * no mutation. */
    SX1276_FHSS_SNAP_DEC_REJECTED_EPOCH_DRIFT = 4
} sx1276_fhss_snap_decision_t;

/* Maximum absolute epoch delta consider_remote() will accept as a
 * legitimate snap candidate. Tight intentionally — until the A6
 * authenticated header carries a MIC, this is the only barrier against
 * a forged frame teleporting the scheduler. ±1 epoch covers a remote
 * that legitimately rolled the epoch boundary just before us. */
#define SX1276_FHSS_SNAP_MAX_EPOCH_DRIFT 1U

/* FCC-A6b-2-ii-alpha: snap-policy gate for incoming A6a headers.
 * Decides whether to ALIGN (no-op), SNAP (call sx1276_fhss_snap_to
 * internally), or REJECT a remote's (epoch, hop_idx) hint.
 *
 * Semantics:
 *   - The local "hop_idx" used for comparison is the slot the
 *     scheduler most recently handed out:
 *       local_hop_idx = (current_slot == 0) ? CHANNEL_COUNT-1
 *                                           : current_slot - 1
 *     This matches the convention sx1276_tx.c uses when stamping
 *     outbound headers — symmetry guarantees ALIGNED fires for
 *     genuinely in-lock-step peers.
 *   - On SNAPPED, snap_to(remote_epoch, remote_hop_idx + 1) is called
 *     internally: the remote has just emitted on remote_hop_idx, so
 *     the next slot it will use is remote_hop_idx + 1 (with the +1
 *     == CHANNEL_COUNT wrap path handled by snap_to's spec).
 *   - All REJECTED_* outcomes are no-ops (scheduler state preserved).
 *     The caller MUST log REJECTED_EPOCH_DRIFT for FCC-B1-SUMMARY
 *     diagnostics so persistent drift is observable.
 *   - Warm-up state is NOT consulted: A6c (Scanning state) will gate
 *     consider_remote() calls during cold-start, but once Scanning
 *     hands off to normal RX, every accepted snap correctly clears
 *     warmup_hops_remaining via snap_to's contract. */
sx1276_fhss_snap_decision_t sx1276_fhss_consider_remote(uint32_t remote_epoch,
                                                        uint8_t  remote_hop_idx);

/* Request that channel `idx` be removed from the active set. Refused
 * (returns BLACKLIST_FLOOR) if the legal floor would be violated,
 * which under the 50-channel narrowband profile is ALWAYS the case —
 * the API exists for future hopsets >50 and for symmetry with the
 * compliance plan. */
sx1276_fhss_status_t sx1276_fhss_blacklist(uint8_t idx);

/* Number of channels currently in the active set
 * (SX1276_FHSS_CHANNEL_COUNT - blacklist popcount). */
uint8_t sx1276_fhss_active_count(void);

/* Increment a per-channel "LBT blocked" counter. Saturates at UINT32_MAX.
 * Pure observability — does NOT affect scheduling. */
void sx1276_fhss_record_lbt_block(uint8_t idx);
uint32_t sx1276_fhss_lbt_block_count(uint8_t idx);

/* FCC-B1-SUMMARY-b (plan §14.2 step 11): per-channel hop-count sidecar
 * for the per-minute RFCO_SUMMARY URC.
 *
 * record_hop(idx) is called from the TX-success path once per emitted
 * frame; it saturates at UINT16_MAX in-memory so a runaway TX loop
 * cannot wrap the counter silently. Out-of-range idx is a no-op (the
 * compliance counter MUST NOT crash the scheduler).
 *
 * hop_count_snapshot_and_clear(out) atomically (interrupts-disabled on
 * target; trivially atomic on the bench host) copies the in-memory
 * counters into `out[0..SX1276_FHSS_CHANNEL_COUNT-1]`, saturating each
 * to 0xFFU since the on-wire field in RFCO_SUMMARY is u8 per channel,
 * then zeroes the in-memory counters. NULL `out` is a no-op (counters
 * are preserved so a caller bug does not lose a minute of evidence).
 * Pure observability — does NOT affect scheduling or blacklist. */
void sx1276_fhss_record_hop(uint8_t idx);
void sx1276_fhss_hop_count_snapshot_and_clear(uint8_t out[SX1276_FHSS_CHANNEL_COUNT]);

/* Telemetry accessors. */
uint32_t sx1276_fhss_current_epoch(void);
uint8_t  sx1276_fhss_current_slot(void);
uint32_t sx1276_fhss_current_seed(void);
uint8_t  sx1276_fhss_is_initialized(void);

/* === Pure helpers exposed for golden-vector tests + X8 host mirror. ===
 * These are stateless and reentrant. */

/* FNV-1a 32-bit over the LE byte concatenation
 *   farm_id[0..7] || node_id[0..7] || epoch[0..3]
 * Returns 0 only with vanishingly small probability — the scheduler
 * does not give 0 any special meaning. */
uint32_t sx1276_fhss_compute_seed(uint64_t farm_id,
                                  uint64_t node_id,
                                  uint32_t epoch);

/* Fill out[0..SX1276_FHSS_CHANNEL_COUNT-1] with a Fisher-Yates
 * permutation of [0..SX1276_FHSS_CHANNEL_COUNT-1] using xorshift32
 * seeded by `seed`. If seed == 0, internally substitutes 0x9E3779B9
 * (xorshift32 collapses on zero seed). */
void sx1276_fhss_compute_permutation(uint32_t seed,
                                     uint8_t out[SX1276_FHSS_CHANNEL_COUNT]);

#endif /* LIFETRAC_MURATA_L072_SX1276_FHSS_H */
