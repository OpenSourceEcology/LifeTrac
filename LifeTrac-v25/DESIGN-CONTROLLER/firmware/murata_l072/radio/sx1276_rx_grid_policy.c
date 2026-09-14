/*
 * RS-12.15 v2 (PR #125 review round 3): RX grid policy, HW-free. See the
 * header for the rules. Bench-pinned by check-rx-grid-policy against the
 * real sx1276_fhss_consider_remote().
 */

#include "sx1276_rx_grid_policy.h"

#include "sx1276_fhss_authority.h"
#include "sx1276_fhss_clock.h"

/* F6: has ANY remote grid been accepted since the last acquisition
 * reset? While 0 (and the node is not a streaming originator) the health
 * handed to consider_remote() is UNANCHORED even if the clock is
 * TX-self-anchored: the recovery tier must stay reachable on duplex
 * nodes, or the TX lazy anchor re-validates a stale grid right after a
 * demotion and the node locks itself out. */
static uint8_t s_grid_adopted;

void sx1276_rx_grid_reset(void) {
    sx1276_fhss_clock_reset();
    sx1276_fhss_authority_reset();
    s_grid_adopted = 0U;
}

uint8_t sx1276_rx_grid_adopted(void) {
    return s_grid_adopted;
}

sx1276_rx_grid_verdict_t sx1276_rx_grid_consider(uint32_t now_ms,
                                                 uint32_t toa_us,
                                                 uint8_t  slot_offset_ms,
                                                 uint32_t remote_abs) {
    sx1276_rx_grid_verdict_t v;
    const uint8_t clock_valid = sx1276_fhss_clock_valid();

    v.originator = sx1276_fhss_authority_is_originator(now_ms, clock_valid,
                                                       s_grid_adopted);
    /* Followers and recovering nodes always adopt an accepted header (the
     * pre-v2 behaviour); a streaming originator adopts only a grid that
     * genuinely LEADS its own -- never its follower's lagged echo. */
    v.adopt = (v.originator == 0U)
                  ? 1U
                  : sx1276_fhss_clock_rx_leads(now_ms, toa_us,
                                               slot_offset_ms, remote_abs);
    if (clock_valid == 0U || s_grid_adopted == 0U) {
        /* Unadopted grid: no refusal authority (UNANCHORED, accept any
         * epoch) -- EXCEPT a streaming originator facing a remote that
         * does not lead, which is handed FRESH purely so consider_remote
         * REFUSES a follower snap (REJECTED_LOCKED_OUT). */
        v.health = (v.originator != 0U && v.adopt == 0U)
                       ? SX1276_FHSS_CLOCK_FRESH
                       : SX1276_FHSS_CLOCK_UNANCHORED;
    } else {
        v.health = (sx1276_fhss_clock_age_ms(now_ms)
                        <= SX1276_FHSS_CLOCK_FRESH_MS)
                       ? SX1276_FHSS_CLOCK_FRESH
                       : SX1276_FHSS_CLOCK_STALE;
    }
    return v;
}

void sx1276_rx_grid_adopt(uint32_t now_ms, uint32_t toa_us,
                          uint8_t slot_offset_ms, uint32_t remote_abs) {
    /* F7: round-not-truncate ToA lives in the clock TU. now_ms is the
     * main-loop service time, not the DIO0 edge -- known, out of scope. */
    sx1276_fhss_clock_anchor_rx(now_ms, toa_us, slot_offset_ms, remote_abs);
    /* F6: a remote grid is now adopted -- the local clock gains refusal
     * authority (FRESH tier) and the node is a follower. */
    s_grid_adopted = 1U;
    sx1276_fhss_authority_note_adopt();
}

uint8_t sx1276_rx_grid_on_demotion(void) {
    uint8_t reset = 0U;
    if (s_grid_adopted != 0U) {
        /* The adopted grid is gone with the lock: fresh acquisition must
         * re-derive the phase, and the recovery tier must be reachable. */
        sx1276_fhss_clock_reset();
        sx1276_fhss_authority_reset();
        reset = 1U;
    }
    /* A self-anchored clock survives its owner's scan demotion (the fix). */
    s_grid_adopted = 0U;
    return reset;
}
