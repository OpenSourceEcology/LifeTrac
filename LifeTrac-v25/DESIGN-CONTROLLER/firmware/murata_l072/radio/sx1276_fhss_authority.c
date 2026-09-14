/*
 * RS-12.15 v2 (2026-09-14): originator authority = sustained own-TX
 * streaming. See include/sx1276_fhss_authority.h for the contract.
 * HW-free on purpose (bench-linkable, check-fhss-authority).
 */

#include "sx1276_fhss_authority.h"

typedef struct {
    uint32_t last_tx_ms;
    uint32_t streak;
    uint32_t streak_max;
    uint8_t  have_tx;
} fhss_authority_state_t;

static fhss_authority_state_t s_auth;

void sx1276_fhss_authority_reset(void) {
    s_auth.last_tx_ms = 0U;
    s_auth.streak     = 0U;
    s_auth.streak_max = 0U;
    s_auth.have_tx    = 0U;
}

void sx1276_fhss_authority_note_tx(uint32_t now_ms) {
    /* Wrap-safe u32 gap, same idiom as the clock TU. A gap wider than
     * the streaming cadence restarts the streak at this transmission. */
    if (s_auth.have_tx != 0U &&
        (now_ms - s_auth.last_tx_ms) <= SX1276_FHSS_AUTHORITY_STREAK_GAP_MS) {
        if (s_auth.streak != 0xFFFFFFFFU) {
            ++s_auth.streak;
        }
    } else {
        s_auth.streak = 1U;
    }
    s_auth.last_tx_ms = now_ms;
    s_auth.have_tx    = 1U;
    if (s_auth.streak > s_auth.streak_max) {
        s_auth.streak_max = s_auth.streak;
    }
}

void sx1276_fhss_authority_note_adopt(void) {
    /* A follower has no originator authority; the next own TX starts a
     * fresh streak from 1. last_tx_ms is deliberately kept so a burst of
     * own TXs right after adopting still counts from the adoption. */
    s_auth.streak = 0U;
}

uint32_t sx1276_fhss_authority_streak(void) {
    return s_auth.streak;
}

uint32_t sx1276_fhss_authority_streak_max(void) {
    return s_auth.streak_max;
}

uint8_t sx1276_fhss_authority_is_originator(uint8_t clock_valid,
                                            uint8_t grid_adopted) {
    if (clock_valid == 0U || grid_adopted != 0U) {
        return 0U;
    }
    return (s_auth.streak >= SX1276_FHSS_AUTHORITY_MIN_STREAK) ? 1U : 0U;
}
