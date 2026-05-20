/*
 * FCC-A6b-2-ii-γ-2: per-outcome counters for the RX retune tick
 * (sx1276_rx_tick() / γ-1 policy). Extracted into a HW-free TU so
 * the saturating-increment and snapshot/reset semantics are
 * host-testable without dragging in sx1276_rx.c's modem/platform
 * dependencies — same pattern as A6b-2-ii-β's snap-policy counters.
 *
 * Storage lives here; sx1276_rx_tick() calls
 * sx1276_rx_retune_counter_record(dec) once per policy evaluation.
 * FCC-B1-SUMMARY will read this via the public getter (additive URC)
 * — no host_stats field, no host↔X8 wire bump.
 */

#include "sx1276_rx.h"
#include "sx1276_rx_retune_policy.h"

#include <stddef.h>
#include <stdint.h>

static uint32_t s_retune_counts[SX1276_RX_RETUNE_DECISION_DIM];

void sx1276_rx_retune_counter_record(sx1276_rx_retune_decision_t dec) {
    const unsigned int idx = (unsigned int)dec;
    if (idx >= SX1276_RX_RETUNE_DECISION_DIM) {
        return;
    }
    if (s_retune_counts[idx] != 0xFFFFFFFFU) {
        ++s_retune_counts[idx];
    }
}

void sx1276_rx_retune_counts(
    uint32_t out_counts[SX1276_RX_RETUNE_DECISION_DIM]) {
    if (out_counts == NULL) {
        return;
    }
    for (uint8_t i = 0U; i < SX1276_RX_RETUNE_DECISION_DIM; ++i) {
        out_counts[i] = s_retune_counts[i];
    }
}

void sx1276_rx_retune_counts_reset(void) {
    for (uint8_t i = 0U; i < SX1276_RX_RETUNE_DECISION_DIM; ++i) {
        s_retune_counts[i] = 0U;
    }
}
