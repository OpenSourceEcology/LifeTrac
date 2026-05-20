/*
 * FCC-A6b-2-ii-γ-1 (plan §14.2 step 10): RX retune decision — pure
 * function implementation.
 *
 * See include/sx1276_rx_retune_policy.h for the decision contract.
 *
 * HW-free TU. No modem / platform / host_cmd / host_stats deps.
 * Compiled into both the firmware (linked with sx1276_rx.c in γ-2)
 * AND the host check binary (bench/host_proto/rx_retune_policy.c).
 *
 * Wrap-safe elapsed math:
 *   uint32_t raw   = now_ms - last_retune_ms;      // wraps cleanly
 *   int32_t  delta = (int32_t)raw;                 // reinterpret
 *
 *   delta > 0  : forward time, retune candidate
 *   delta == 0 : same tick, treat as TOO_SOON (period_ms ≥ 1)
 *   delta < 0  : caller fed last_retune_ms > now_ms, OR the tick wrapped
 *                hard. We treat any negative reinterpretation as
 *                DO_WRAP — the γ-2 caller's contract is to overwrite
 *                last_retune_ms = now_ms on DO_WRAP so we self-heal.
 *
 * The "+half-range" wording in the header refers to the unsigned view:
 * uint32 elapsed > 0x80000000U is exactly the int32 < 0 case after
 * reinterpretation, so the implementation collapses to a single
 * signed-vs-period compare.
 */

#include "sx1276_rx_retune_policy.h"

sx1276_rx_retune_decision_t sx1276_rx_retune_eval(uint32_t now_ms,
                                                  uint32_t last_retune_ms,
                                                  bool     modem_busy,
                                                  bool     fhss_ready) {
    if (!fhss_ready) {
        return SX1276_RX_RETUNE_SKIP_NOT_INIT;
    }
    if (modem_busy) {
        return SX1276_RX_RETUNE_SKIP_BUSY;
    }

    const int32_t delta = (int32_t)(now_ms - last_retune_ms);
    if (delta < 0) {
        /* Backwards time. Either the caller mis-fed inputs or the
         * monotonic tick wrapped past UINT32_MAX (≈ 49.7 d). Either
         * way, γ-2's contract is: on DO_WRAP, re-anchor
         * last_retune_ms = now_ms and skip the actual retune this
         * tick. The next tick will then evaluate cleanly. */
        return SX1276_RX_RETUNE_DO_WRAP;
    }
    if ((uint32_t)delta < (uint32_t)SX1276_RX_RETUNE_PERIOD_MS) {
        return SX1276_RX_RETUNE_SKIP_TOO_SOON;
    }
    return SX1276_RX_RETUNE_DO;
}
