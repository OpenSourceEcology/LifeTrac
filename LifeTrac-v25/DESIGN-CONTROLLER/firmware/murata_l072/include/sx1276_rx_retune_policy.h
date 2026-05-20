#ifndef LIFETRAC_MURATA_L072_SX1276_RX_RETUNE_POLICY_H
#define LIFETRAC_MURATA_L072_SX1276_RX_RETUNE_POLICY_H

/*
 * FCC-A6b-2-ii-γ-1 (plan §14.2 step 10): RX retune decision — pure
 * function.
 *
 * Split rationale: the RX retune work has three concerns that get
 * conflated if jammed into one function:
 *
 *   (1) policy   — "is it time to retune yet?"   (this header / TU)
 *   (2) mechanism — standby → retune → settle → re-arm (γ-2, HW-bound)
 *   (3) final timing — driven by the epoch model chosen for plan §7
 *       open Q2 (γ-3, deferred)
 *
 * γ-1 isolates (1) as a pure function so it is host-testable and so
 * γ-3 can replace the placeholder period without touching any HW path.
 * γ-2 wires the rest.
 *
 * Placeholder period: SX1276_RX_RETUNE_PERIOD_MS is set ≤ the FCC
 * dwell cap (380 ms). This guarantees the placeholder is never
 * illegal regardless of which Q2 epoch model wins, so γ-1 can land
 * with confidence even before Q2 closes.
 *
 * Wrap-safe elapsed math: callers pass `now_ms` and `last_retune_ms`
 * from the same monotonic ms tick. The implementation subtracts as
 * uint32 and reinterprets as int32 so a single wrap is handled
 * naturally (mirrors sx1276_fhss_consider_remote()'s drift idiom).
 *
 * No HW deps. Includes only <stdbool.h> + <stdint.h>.
 */

#include <stdbool.h>
#include <stdint.h>

/*
 * Placeholder retune period. ≤ SX1276_AIRTIME_DWELL_CAP_US / 1000
 * (380 ms) so the placeholder is FCC-legal regardless of how Q2
 * closes. γ-3 replaces this with the real epoch-driven boundary.
 *
 * NOT a compile-time function of the airtime cap on purpose — γ-3's
 * replacement is expected to come from a *different* source (the
 * epoch period), and we want γ-3 to be a clean point edit, not an
 * "untangle the cap from the period" refactor.
 */
#define SX1276_RX_RETUNE_PERIOD_MS    380U

/*
 * Decision outcomes. Numeric values are stable (FCC-B1-SUMMARY's
 * additive URC will index per-outcome counters by this enum, same
 * pattern as the A6b-2-ii-β snap-policy counters).
 */
typedef enum {
    SX1276_RX_RETUNE_DO            = 0,
    SX1276_RX_RETUNE_SKIP_NOT_INIT = 1,
    SX1276_RX_RETUNE_SKIP_BUSY     = 2,
    SX1276_RX_RETUNE_SKIP_TOO_SOON = 3,
    /* now_ms < last_retune_ms by more than half the uint32 range —
     * treated as a forward wrap (legitimate tick wrap after ~49.7 d)
     * and forced to DO so the next tick re-anchors last_retune_ms. */
    SX1276_RX_RETUNE_DO_WRAP       = 4,
} sx1276_rx_retune_decision_t;

#define SX1276_RX_RETUNE_DECISION_DIM 5U

/*
 * Decide whether the RX path should retune now.
 *
 * Preconditions:
 *   - now_ms / last_retune_ms come from the same monotonic ms tick.
 *   - fhss_ready reflects whether sx1276_fhss_init() has returned OK.
 *   - modem_busy is true while the modem is mid-RX / mid-CAD / mid-TX
 *     (anything we MUST NOT yank the synth out from under).
 *
 * Order of checks (first match wins):
 *   1. !fhss_ready          → SKIP_NOT_INIT
 *   2. modem_busy           → SKIP_BUSY
 *      (BUSY is checked AFTER NOT_INIT so an un-init'd scheduler
 *      surfaces as the most actionable diagnostic even when the modem
 *      happens to be transmitting on the legacy single-channel path.)
 *   3. elapsed > +half-range → DO_WRAP (caller re-anchors)
 *   4. elapsed < period_ms  → SKIP_TOO_SOON
 *   5. otherwise            → DO
 *
 * NULL-safe: no pointer arguments.
 */
sx1276_rx_retune_decision_t sx1276_rx_retune_eval(uint32_t now_ms,
                                                  uint32_t last_retune_ms,
                                                  bool     modem_busy,
                                                  bool     fhss_ready);

#endif /* LIFETRAC_MURATA_L072_SX1276_RX_RETUNE_POLICY_H */
