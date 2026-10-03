/*
 * RS-12.15 v2 (2026-09-14): originator authority = sustained own-TX
 * streaming. See include/sx1276_fhss_authority.h for the contract.
 * HW-free on purpose (bench-linkable, check-fhss-authority).
 */

#include "sx1276_fhss_authority.h"

#include "sx1276_fhss_clock.h"   /* SX1276_FHSS_SLOT_MS (bounds only) */

/* RS-13.1 A11 (2026-10-03): the gap must chain the slowest supported
 * stream (1 fps) even when one frame's key-up slips a whole FHSS slot
 * (the slot-fit guard defers a TX that would straddle a boundary), and it
 * must NOT chain across a skipped frame -- so two seconds of silence
 * always ends both the streak and the authority it earned. */
_Static_assert(SX1276_FHSS_AUTHORITY_STREAK_GAP_MS >
                   SX1276_FHSS_AUTHORITY_SLOWEST_STREAM_MS + SX1276_FHSS_SLOT_MS,
               "A11: a 1 fps stream with a one-slot TX deferral must chain");
_Static_assert(SX1276_FHSS_AUTHORITY_STREAK_GAP_MS <
                   2U * SX1276_FHSS_AUTHORITY_SLOWEST_STREAM_MS,
               "A11: a skipped 1 fps frame (2 s of silence) must end authority");

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
     * the streaming cadence restarts the streak at this transmission.
     * STRICT: a sender spaced exactly at the gap must not chain (PR #125
     * review round 3). Since RS-13.1 A11 the gap is 1.5 s, so a 1 fps
     * stream chains -- and so would 1 Hz commands on cadence alone; the
     * adoption rule (note_adopt) is what keeps a command sender out, see
     * the header. */
    if (s_auth.have_tx != 0U &&
        (now_ms - s_auth.last_tx_ms) < SX1276_FHSS_AUTHORITY_STREAK_GAP_MS) {
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

uint8_t sx1276_fhss_authority_is_originator(uint32_t now_ms,
                                            uint8_t clock_valid,
                                            uint8_t grid_adopted) {
    if (clock_valid == 0U || grid_adopted != 0U || s_auth.have_tx == 0U) {
        return 0U;
    }
    if (s_auth.streak < SX1276_FHSS_AUTHORITY_MIN_STREAK) {
        return 0U;
    }
    /* Authority DECAYS when streaming stops: a node that sent a burst and
     * went quiet loses it one gap later, so a post-demotion base that
     * drained a few queued commands cannot sit on a stale grid refusing
     * the tractor (PR #125 review round 3). Wrap-safe u32 idiom. */
    return ((now_ms - s_auth.last_tx_ms) < SX1276_FHSS_AUTHORITY_STREAK_GAP_MS)
               ? 1U : 0U;
}
