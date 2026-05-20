/*
 * FCC-A6c-2 (counters TU): per-state + per-action histograms for
 * the Scanning state machine (sx1276_rx_scan_eval() / A6c-1
 * policy). Extracted into a HW-free TU so the saturating-increment
 * + snapshot/reset semantics are host-testable without dragging in
 * sx1276_rx.c's modem/platform deps — same pattern as A6b-2-ii-β
 * snap counters and γ-2 retune counters.
 *
 * Storage lives here; the RX loop (A6c-2-b) calls
 * sx1276_rx_scan_counter_record(state, action) once per policy
 * eval. FCC-B1-SUMMARY reads the two histograms via the public
 * getters (additive URC) — no host_stats field, no host↔X8 wire
 * bump.
 *
 * The two axes are independently guarded: an out-of-range cast on
 * one axis drops only that axis's increment, never blanks the
 * other. Saturating uint32 per slot.
 */

#include "sx1276_rx_scan_policy.h"

#include <stddef.h>
#include <stdint.h>

static uint32_t s_state_counts[SX1276_RX_SCAN_STATE_DIM];
static uint32_t s_action_counts[SX1276_RX_SCAN_ACTION_DIM];

void sx1276_rx_scan_counter_record(sx1276_rx_scan_state_t state,
                                   sx1276_rx_scan_action_t action) {
    const unsigned int s_idx = (unsigned int)state;
    if (s_idx < SX1276_RX_SCAN_STATE_DIM) {
        if (s_state_counts[s_idx] != 0xFFFFFFFFU) {
            ++s_state_counts[s_idx];
        }
    }
    const unsigned int a_idx = (unsigned int)action;
    if (a_idx < SX1276_RX_SCAN_ACTION_DIM) {
        if (s_action_counts[a_idx] != 0xFFFFFFFFU) {
            ++s_action_counts[a_idx];
        }
    }
}

void sx1276_rx_scan_state_counts(
    uint32_t out_counts[SX1276_RX_SCAN_STATE_DIM]) {
    if (out_counts == NULL) {
        return;
    }
    for (uint8_t i = 0U; i < SX1276_RX_SCAN_STATE_DIM; ++i) {
        out_counts[i] = s_state_counts[i];
    }
}

void sx1276_rx_scan_action_counts(
    uint32_t out_counts[SX1276_RX_SCAN_ACTION_DIM]) {
    if (out_counts == NULL) {
        return;
    }
    for (uint8_t i = 0U; i < SX1276_RX_SCAN_ACTION_DIM; ++i) {
        out_counts[i] = s_action_counts[i];
    }
}

void sx1276_rx_scan_counts_reset(void) {
    for (uint8_t i = 0U; i < SX1276_RX_SCAN_STATE_DIM; ++i) {
        s_state_counts[i] = 0U;
    }
    for (uint8_t i = 0U; i < SX1276_RX_SCAN_ACTION_DIM; ++i) {
        s_action_counts[i] = 0U;
    }
}
