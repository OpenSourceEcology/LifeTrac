/*
 * FCC-A6c-1: RX cold-start Scanning state-machine policy —
 * pure-function implementation. See include/sx1276_rx_scan_policy.h
 * for the design contract.
 */

#include "sx1276_rx_scan_policy.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Default-safe decision for NULL / out-of-range inputs. */
static const sx1276_rx_scan_decision_t k_default_safe = {
    .action     = SX1276_RX_SCAN_ACTION_HOLD,
    .next_state = SX1276_RX_SCAN_STATE_BOOT,
};

static inline bool tick_wrapped(uint32_t now_ms, uint32_t anchor_ms) {
    /* Reinterpret the unsigned subtraction as signed; negative means
     * `now_ms` is behind `anchor_ms` (i.e. the uint32 ms tick
     * wrapped, or the caller fed backwards time). Mirrors the
     * γ-1 retune policy's wrap detection. */
    return ((int32_t)(now_ms - anchor_ms)) < 0;
}

sx1276_rx_scan_decision_t sx1276_rx_scan_eval(const sx1276_rx_scan_input_t *in) {
    if (in == NULL) {
        return k_default_safe;
    }

    /* Defensive: a FRAME_VALID event must be paired with a verified
     * header. If the caller misuses the API and passes
     * frame_header_valid=false on a FRAME_VALID event, demote to
     * FRAME_INVALID so we never lock onto an unauthenticated frame. */
    sx1276_rx_scan_event_t event = in->event;
    if (event == SX1276_RX_SCAN_EVENT_FRAME_VALID && !in->frame_header_valid) {
        event = SX1276_RX_SCAN_EVENT_FRAME_INVALID;
    }

    switch (in->state) {
    case SX1276_RX_SCAN_STATE_BOOT: {
        /* Caller cannot have meaningful anchors yet. Force BEGIN_SCAN
         * regardless of the event so the caller anchors both timers
         * and starts scanning the first channel. */
        sx1276_rx_scan_decision_t d = {
            .action     = SX1276_RX_SCAN_ACTION_BEGIN_SCAN,
            .next_state = SX1276_RX_SCAN_STATE_SCANNING,
        };
        return d;
    }

    case SX1276_RX_SCAN_STATE_SCANNING:
        if (event == SX1276_RX_SCAN_EVENT_FRAME_VALID) {
            sx1276_rx_scan_decision_t d = {
                .action     = SX1276_RX_SCAN_ACTION_LOCK,
                .next_state = SX1276_RX_SCAN_STATE_LOCKED,
            };
            return d;
        }
        if (event == SX1276_RX_SCAN_EVENT_FRAME_INVALID) {
            /* Garbage on this channel — stay put. The TICK path will
             * advance us when the dwell expires. */
            sx1276_rx_scan_decision_t d = {
                .action     = SX1276_RX_SCAN_ACTION_HOLD,
                .next_state = SX1276_RX_SCAN_STATE_SCANNING,
            };
            return d;
        }

        /* event == TICK from here down. Check anchors in the order
         * spelled out in the header (cold-start wrap → regulatory
         * FAIL → channel wrap → channel ADVANCE → HOLD). */
        if (tick_wrapped(in->now_ms, in->cold_start_entry_ms) ||
            tick_wrapped(in->now_ms, in->channel_entry_ms)) {
            sx1276_rx_scan_decision_t d = {
                .action     = SX1276_RX_SCAN_ACTION_REANCHOR,
                .next_state = SX1276_RX_SCAN_STATE_SCANNING,
            };
            return d;
        }

        {
            const uint32_t cold_elapsed =
                in->now_ms - in->cold_start_entry_ms;
            if (cold_elapsed >= SX1276_RX_SCAN_REDESIGN_MS) {
                sx1276_rx_scan_decision_t d = {
                    .action     = SX1276_RX_SCAN_ACTION_FAIL,
                    .next_state = SX1276_RX_SCAN_STATE_FAILED,
                };
                return d;
            }
        }

        {
            const uint32_t channel_elapsed =
                in->now_ms - in->channel_entry_ms;
            if (channel_elapsed >= SX1276_RX_SCAN_DWELL_MS) {
                sx1276_rx_scan_decision_t d = {
                    .action     = SX1276_RX_SCAN_ACTION_ADVANCE_CHANNEL,
                    .next_state = SX1276_RX_SCAN_STATE_SCANNING,
                };
                return d;
            }
        }

        {
            sx1276_rx_scan_decision_t d = {
                .action     = SX1276_RX_SCAN_ACTION_HOLD,
                .next_state = SX1276_RX_SCAN_STATE_SCANNING,
            };
            return d;
        }

    case SX1276_RX_SCAN_STATE_LOCKED: {
        /* A6c-1 surface treats LOCKED as absorbing — γ-1 owns the
         * retune loop from here. A6c (post-γ-3) may add a LOSS-OF-
         * SYNC re-scan transition. */
        sx1276_rx_scan_decision_t d = {
            .action     = SX1276_RX_SCAN_ACTION_HOLD,
            .next_state = SX1276_RX_SCAN_STATE_LOCKED,
        };
        return d;
    }

    case SX1276_RX_SCAN_STATE_FAILED:
    default: {
        /* Terminal. The only way out is an external reset (caller
         * decides — power cycle, watchdog, host command). */
        sx1276_rx_scan_decision_t d = {
            .action     = SX1276_RX_SCAN_ACTION_HOLD,
            .next_state = SX1276_RX_SCAN_STATE_FAILED,
        };
        return d;
    }
    }
}
