/*
 * FCC-A6b-2-ii-β: per-decision counters for the snap-policy gate
 * (sx1276_fhss_consider_remote()). Extracted into a HW-free TU so
 * the saturating-increment and snapshot/reset semantics are
 * host-testable without dragging in sx1276_rx.c's modem/platform
 * dependencies.
 *
 * Storage lives here; sx1276_rx_service() calls
 * sx1276_rx_counter_record(dec) once per successfully-parsed A6a
 * header. FCC-B1-SUMMARY will read this via the public getter
 * (additive URC) — no host_stats field, no host↔X8 wire bump.
 */

#include "sx1276_rx.h"

#include <stddef.h>
#include <stdint.h>

static uint32_t s_consider_remote_counts[SX1276_RX_CONSIDER_REMOTE_COUNT_DIM];

void sx1276_rx_counter_record(sx1276_fhss_snap_decision_t dec) {
    const unsigned int idx = (unsigned int)dec;
    if (idx >= SX1276_RX_CONSIDER_REMOTE_COUNT_DIM) {
        return;
    }
    if (s_consider_remote_counts[idx] != 0xFFFFFFFFU) {
        ++s_consider_remote_counts[idx];
    }
}

void sx1276_rx_consider_remote_counts(
    uint32_t out_counts[SX1276_RX_CONSIDER_REMOTE_COUNT_DIM]) {
    if (out_counts == NULL) {
        return;
    }
    for (uint8_t i = 0U; i < SX1276_RX_CONSIDER_REMOTE_COUNT_DIM; ++i) {
        out_counts[i] = s_consider_remote_counts[i];
    }
}

void sx1276_rx_consider_remote_counts_reset(void) {
    for (uint8_t i = 0U; i < SX1276_RX_CONSIDER_REMOTE_COUNT_DIM; ++i) {
        s_consider_remote_counts[i] = 0U;
    }
}
