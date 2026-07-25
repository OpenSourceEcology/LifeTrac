#ifndef LIFETRAC_MURATA_L072_SX1276_RX_SCAN_POLICY_H
#define LIFETRAC_MURATA_L072_SX1276_RX_SCAN_POLICY_H

/*
 * FCC-A6c-1 (plan §14.2 step 10): RX cold-start Scanning state
 * machine — pure function.
 *
 * Concern split, mirroring the γ-1/γ-2/γ-3 partition that landed
 * the steady-state retune loop:
 *
 *   (1) policy   — "given current state + event + clock, what is the
 *                  next state + caller action?"        (this header)
 *   (2) mechanism — wideband scan, per-channel preamble timeout,
 *                  epoch extraction from the A6a header, transition
 *                  to LOCKED + γ-1 retune loop.        (A6c-2, HW-bound)
 *   (3) reporting — 5 s acceptance-target overrun counter + 30 s
 *                  redesign-threshold abort URC.       (A6c-3, B1-SUMMARY)
 *
 * Scope of A6c-1: the BOOT → SCANNING → LOCKED happy path plus the
 * SCANNING → FAILED regulatory abort. No LOSS-OF-SYNC re-scan path
 * from LOCKED back to SCANNING — that's a separate increment once
 * the LOCKED-state stale-decode policy is decided (likely A6c after
 * γ-3, when the epoch model is final).
 *
 * Regulatory anchors (FCC notes §15.1.1 retraction, plan §7 Q6):
 *   - No fixed rendezvous channel. Cold-start scans the active 50-
 *     channel set with per-channel preamble timeout.
 *   - Acceptance TARGET: worst-case cold-start hop-sync reacquire
 *     ≤ 5 s (SX1276_RX_SCAN_GOAL_MS). Tracked, NOT enforced here —
 *     A6c-3 surfaces the overrun count to D-Gate via B1-SUMMARY.
 *   - Redesign THRESHOLD: > 30 s ⇒ A6 redesign mandated. This IS
 *     enforced as a hard FAIL transition (SX1276_RX_SCAN_REDESIGN_MS)
 *     so a stuck radio doesn't silently hog the band past the point
 *     where the FCC narrative ("cold-start ≤ a few seconds") still
 *     holds.
 *
 * Authenticated-header trust boundary: the FRAME_VALID event means
 * the caller has already verified the A6a header's MIC + profile_id
 * + schema_ver. This policy does NOT re-validate. Garbage frames
 * (CRC fail, MIC fail, wrong profile, wrong schema) MUST be passed
 * as FRAME_INVALID, never FRAME_VALID.
 *
 * Wrap-safe elapsed math: anchors are uint32 ms. Modular arithmetic
 * absorbs the normal once-per-49.7-day tick wrap naturally — the
 * apparent forward elapsed stays small. The REANCHOR action is
 * reserved for the pathological case where apparent elapsed exceeds
 * 2^31 ms (anchor more than ~24.85 d "ahead of" now), which can only
 * be a clock glitch / backwards jump. Caller re-zeros both anchors
 * and continues without false-FAIL or false-ADVANCE.
 *
 * No HW deps. Includes only <stdbool.h> + <stdint.h>.
 */

#include <stdbool.h>
#include <stdint.h>

/*
 * Placeholder per-channel scan dwell. Set to the half-second worst-
 * case preamble for the slowest active profile rounded down to a
 * conservative 100 ms. A6c-2 may parameterize this off the running
 * profile's preamble length.
 */
#define SX1276_RX_SCAN_DWELL_MS         500U

/*
 * 2026-07-24 FHSS bring-up (run-18 evidence): LOCKED used to be absorbing
 * — one missed follow (single lost packet while TX hops per packet) left
 * the receiver parked on a stale channel forever; rx_frames froze at 3
 * with no fault and no re-acquisition. A LOCKED receiver that has not
 * seen a valid frame for this long is demoted back to SCANNING via a
 * fresh BEGIN_SCAN. The driver re-anchors channel_entry_ms on every
 * FRAME_VALID in LOCKED (REANCHOR action), so an active link never
 * trips this. 2 s ≈ 12 packet times at the 165 ms bench cadence —
 * tolerant of QoS pacing gaps, fast enough that a video stream resumes
 * within one scan acquisition.
 */
#define SX1276_RX_SCAN_LOCK_LOSS_MS    2000U

/*
 * §7 Q6 acceptance target: worst-case cold-start hop-sync reacquire
 * ≤ 5 s. NOT enforced as a state transition — A6c-3 reports overruns
 * via B1-SUMMARY for D-Gate analysis.
 */
#define SX1276_RX_SCAN_GOAL_MS         5000U

/*
 * §7 Q6 redesign threshold: > 30 s ⇒ A6 redesign mandated. Enforced
 * here as a SCANNING → FAILED transition so the firmware doesn't hog
 * the band past the regulatory narrative's bounds.
 */
#define SX1276_RX_SCAN_REDESIGN_MS    30000U

/*
 * State enum. Numeric values are stable — FCC-B1-SUMMARY will index
 * per-state dwell-time histograms by this enum.
 */
typedef enum {
    SX1276_RX_SCAN_STATE_BOOT     = 0,
    SX1276_RX_SCAN_STATE_SCANNING = 1,
    SX1276_RX_SCAN_STATE_LOCKED   = 2,
    SX1276_RX_SCAN_STATE_FAILED   = 3,
} sx1276_rx_scan_state_t;

#define SX1276_RX_SCAN_STATE_DIM 4U

/*
 * Event enum. Caller maps modem IRQ + decode outcome onto this.
 *
 *   TICK          — time-only poll, no decode this round.
 *   FRAME_VALID   — frame decoded, A6a header passed MIC + profile +
 *                   schema checks. Header payload is in *_header_*
 *                   input fields (caller pre-parses).
 *   FRAME_INVALID — anything else (CRC fail, MIC fail, wrong profile,
 *                   wrong schema, header parse fail). MUST NOT be
 *                   conflated with FRAME_VALID.
 */
typedef enum {
    SX1276_RX_SCAN_EVENT_TICK          = 0,
    SX1276_RX_SCAN_EVENT_FRAME_VALID   = 1,
    SX1276_RX_SCAN_EVENT_FRAME_INVALID = 2,
} sx1276_rx_scan_event_t;

#define SX1276_RX_SCAN_EVENT_DIM 3U

/*
 * Action enum. Numeric values are stable — FCC-B1-SUMMARY will index
 * per-action counters by this enum (mirror of γ-1's retune-decision
 * counter pattern).
 *
 *   HOLD            — caller does nothing; state may have changed
 *                     (e.g. FAILED is terminal but still HOLD on
 *                     subsequent ticks).
 *   BEGIN_SCAN      — BOOT → SCANNING: caller anchors both
 *                     `channel_entry_ms` and `cold_start_entry_ms`
 *                     to now_ms and starts dwelling on the first
 *                     scan channel.
 *   ADVANCE_CHANNEL — per-channel dwell expired: caller jumps to
 *                     the next scan channel and re-anchors
 *                     `channel_entry_ms` to now_ms.
 *                     `cold_start_entry_ms` is UNCHANGED (the
 *                     budget is across the whole scan, not per-
 *                     channel).
 *   LOCK            — SCANNING → LOCKED: caller initializes the
 *                     FHSS scheduler from the decoded epoch/hop
 *                     index in the input header and hands the RX
 *                     path to the γ-1 retune loop.
 *   FAIL            — SCANNING → FAILED: caller raises an alert
 *                     URC and stops the modem. Terminal state.
 *   REANCHOR        — uint32 ms tick wrap detected on at least one
 *                     anchor: caller re-anchors BOTH anchors to
 *                     now_ms and stays in the same state. The next
 *                     tick evaluates cleanly.
 */
typedef enum {
    SX1276_RX_SCAN_ACTION_HOLD            = 0,
    SX1276_RX_SCAN_ACTION_BEGIN_SCAN      = 1,
    SX1276_RX_SCAN_ACTION_ADVANCE_CHANNEL = 2,
    SX1276_RX_SCAN_ACTION_LOCK            = 3,
    SX1276_RX_SCAN_ACTION_FAIL            = 4,
    SX1276_RX_SCAN_ACTION_REANCHOR        = 5,
} sx1276_rx_scan_action_t;

#define SX1276_RX_SCAN_ACTION_DIM 6U

/*
 * Caller-supplied policy input. All fields are caller-owned scalars.
 *
 *   state                — current SM state.
 *   now_ms               — monotonic ms tick (same source as the
 *                          γ-1 retune policy).
 *   channel_entry_ms     — last anchor of the per-channel scan
 *                          dwell. Meaningful only in SCANNING.
 *   cold_start_entry_ms  — anchor of the whole cold-start budget.
 *                          Meaningful only in SCANNING.
 *   event                — what happened this tick.
 *   frame_header_valid   — when event == FRAME_VALID, true iff the
 *                          caller has already verified the A6a
 *                          header's MIC + profile_id + schema_ver.
 *                          Defensive: if this is false on a
 *                          FRAME_VALID event the policy treats it
 *                          as FRAME_INVALID (caller bug).
 */
typedef struct {
    sx1276_rx_scan_state_t  state;
    uint32_t                now_ms;
    uint32_t                channel_entry_ms;
    uint32_t                cold_start_entry_ms;
    sx1276_rx_scan_event_t  event;
    bool                    frame_header_valid;
} sx1276_rx_scan_input_t;

/*
 * Policy decision. `action` is what the caller should do; `next_state`
 * is what the SM transitions to (caller stores it as the new
 * `state` for the next call).
 */
typedef struct {
    sx1276_rx_scan_action_t action;
    sx1276_rx_scan_state_t  next_state;
} sx1276_rx_scan_decision_t;

/*
 * Evaluate the Scanning state machine.
 *
 * NULL-safe: a NULL `in` returns { HOLD, BOOT } so a caller wired up
 * before its first sample doesn't take a HW action.
 *
 * Order of checks in SCANNING + TICK (first match wins):
 *   1. cold_start anchor wrap        → REANCHOR / SCANNING
 *   2. cold_start_elapsed >= REDESIGN_MS → FAIL / FAILED
 *      (regulatory abort wins over channel-advance)
 *   3. channel anchor wrap           → REANCHOR / SCANNING
 *   4. channel_elapsed >= DWELL_MS   → ADVANCE_CHANNEL / SCANNING
 *   5. otherwise                     → HOLD / SCANNING
 *
 * BOOT collapses any event to BEGIN_SCAN / SCANNING (the caller
 * cannot yet have valid anchors). LOCKED and FAILED are absorbing
 * for the A6c-1 surface — any event yields HOLD and stays put.
 */
sx1276_rx_scan_decision_t sx1276_rx_scan_eval(const sx1276_rx_scan_input_t *in);

/*
 * FCC-A6c-2 (counters TU): per-state + per-action histograms for
 * the Scanning SM. Mirror of γ-2 (sx1276_rx_retune_counters.c) but
 * with two parallel arrays — one indexed by sx1276_rx_scan_state_t
 * (observed input state per eval), one indexed by
 * sx1276_rx_scan_action_t (chosen action per eval). FCC-B1-SUMMARY
 * will surface both histograms additively (no host_stats bump).
 *
 * Storage lives in radio/sx1276_rx_scan_counters.c. The RX loop
 * (A6c-2-b) calls sx1276_rx_scan_counter_record(state, action) once
 * per sx1276_rx_scan_eval() call. Saturating-uint32. Out-of-range
 * arguments are dropped silently per-axis (independent guards) so a
 * malformed cast on one axis cannot blank the other.
 *
 * Getters are unconditionally available so FCC-B1-SUMMARY can read
 * them without an ifdef wrapper — pre-wire-up all entries are zero.
 */
void sx1276_rx_scan_counter_record(sx1276_rx_scan_state_t state,
                                   sx1276_rx_scan_action_t action);
void sx1276_rx_scan_state_counts(
    uint32_t out_counts[SX1276_RX_SCAN_STATE_DIM]);
void sx1276_rx_scan_action_counts(
    uint32_t out_counts[SX1276_RX_SCAN_ACTION_DIM]);
void sx1276_rx_scan_counts_reset(void);

#endif /* LIFETRAC_MURATA_L072_SX1276_RX_SCAN_POLICY_H */
