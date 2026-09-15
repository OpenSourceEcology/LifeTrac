/*
 * RS-12.15 v2 (PR #125 review round 3): policy-level integration test.
 *
 * Drives the RX grid policy the way sx1276_rx.c does, against the REAL
 * sx1276_fhss_consider_remote() (scheduler initialised), the real slot
 * clock and the real authority streak. Own transmissions are emulated the
 * way sx1276_tx.c performs them: lazy first anchor of the scheduler's
 * current slot at tx_now, snap the scheduler to the clock's slot, consume
 * it (next_channel), and -- as the TX_DONE path does -- note the frame in
 * the authority streak. Remote hops are derived from the scheduler's
 * "slot just consumed", exactly the convention consider_remote() uses.
 *
 * Sequences pinned:
 *   1. cold follower adopts (UNANCHORED -> SNAPPED -> adopted);
 *   2. follower demotion resets the clock and clears adoption/authority;
 *   3. THE REGRESSION: post-demotion duplex TX re-anchors, then a remote
 *      frame with a LATER phase must still be UNANCHORED and get SNAPPED
 *      (not LOCKED_OUT) -- recovery stays open;
 *   4. a streaming originator refuses a lagging echo (FRESH -> LOCKED_OUT
 *      on a snap candidate; ALIGNED leaves the clock untouched);
 *   5. a streaming originator adopts a LEADING grid (converges);
 *   6. an originator's own demotion KEEPS its clock;
 *   7. authority decays after the stream stops, so an echo is adopted;
 *   8. commands exactly 1 s apart never earn authority;
 *   9. a forged "leading" epoch+2 is REJECTED_EPOCH_DRIFT (the originator
 *      hands STALE, not UNANCHORED, so the drift barrier holds); epoch+1
 *      is still adopted.
 */

#include <stdio.h>
#include <stdint.h>

#include "sx1276_fhss.h"
#include "sx1276_fhss_authority.h"
#include "sx1276_fhss_clock.h"
#include "sx1276_rx_grid_policy.h"

static int g_failures = 0;

#define CHECK(cond, ...) do {                                          \
    if (!(cond)) {                                                     \
        ++g_failures;                                                  \
        fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__);           \
        fprintf(stderr, __VA_ARGS__);                                  \
        fputc('\n', stderr);                                           \
    }                                                                  \
} while (0)

#define EPOCH 3U
#define N     SX1276_FHSS_CHANNEL_COUNT

static void fresh_scheduler(void) {
    sx1276_fhss_reset();
    (void)sx1276_fhss_init(1ULL, 2ULL, EPOCH);
    sx1276_rx_grid_reset();
}

/* consider_remote()'s notion of "the slot just consumed". */
static uint8_t consumed_hop(void) {
    const uint8_t s = sx1276_fhss_current_slot();
    return (s == 0U) ? (uint8_t)(N - 1U) : (uint8_t)(s - 1U);
}

/* One own FHSS transmission at tx_now_ms, as sx1276_tx.c does it. */
static void tx_step(uint32_t tx_now_ms) {
    if (sx1276_fhss_clock_valid() == 0U) {
        const uint32_t cur_abs = sx1276_fhss_clock_abs_of(
            sx1276_fhss_current_epoch(), sx1276_fhss_current_slot());
        sx1276_fhss_clock_anchor(tx_now_ms, cur_abs);
    }
    const uint32_t abs_now = sx1276_fhss_clock_abs_slot(tx_now_ms);
    (void)sx1276_fhss_snap_to(sx1276_fhss_clock_epoch_of(abs_now),
                              sx1276_fhss_clock_hop_of(abs_now));
    {
        uint8_t idx = 0U; uint32_t hz = 0U;
        (void)sx1276_fhss_next_channel(&idx, &hz);
    }
    sx1276_fhss_authority_note_tx(tx_now_ms);           /* the TX_DONE note */
}

/* One received header as sx1276_rx.c handles it: verdict, real snap
 * decision, adopt when both agree. The header's epoch is a parameter so
 * the drift-barrier vectors can forge one. */
static sx1276_fhss_snap_decision_t rx_frame_ep(uint32_t now_ms, uint32_t toa_us,
                                               uint8_t slot_offset_ms,
                                               uint32_t remote_epoch,
                                               uint8_t remote_hop,
                                               sx1276_rx_grid_verdict_t *out_v) {
    const uint32_t remote_abs =
        sx1276_fhss_clock_abs_of(remote_epoch, remote_hop);
    const sx1276_rx_grid_verdict_t v =
        sx1276_rx_grid_consider(now_ms, toa_us, slot_offset_ms, remote_abs);
    const sx1276_fhss_snap_decision_t dec =
        sx1276_fhss_consider_remote(remote_epoch, remote_hop, v.health);
    if ((dec == SX1276_FHSS_SNAP_DEC_SNAPPED ||
         dec == SX1276_FHSS_SNAP_DEC_ALIGNED) && v.adopt != 0U) {
        sx1276_rx_grid_adopt(now_ms, toa_us, slot_offset_ms, remote_abs);
    }
    if (out_v != NULL) {
        *out_v = v;
    }
    return dec;
}

static sx1276_fhss_snap_decision_t rx_frame(uint32_t now_ms, uint32_t toa_us,
                                            uint8_t slot_offset_ms,
                                            uint8_t remote_hop,
                                            sx1276_rx_grid_verdict_t *out_v) {
    return rx_frame_ep(now_ms, toa_us, slot_offset_ms, EPOCH, remote_hop, out_v);
}

/* Stream 10 frames at 200 ms from t0; returns the time of the last one. */
static uint32_t stream_from(uint32_t t0) {
    uint32_t t = t0;
    for (uint32_t i = 0U; i < 10U; ++i) {
        t = t0 + i * 200U;
        tx_step(t);
    }
    return t;
}

static void test_cold_follower_adopts(void) {
    fresh_scheduler();
    sx1276_rx_grid_verdict_t v;
    const uint8_t hop = (uint8_t)((consumed_hop() + 1U) % N);   /* a snap candidate */
    const sx1276_fhss_snap_decision_t dec = rx_frame(10000U, 0U, 0U, hop, &v);
    CHECK(v.health == SX1276_FHSS_CLOCK_UNANCHORED, "(1) cold: UNANCHORED");
    CHECK(v.adopt == 1U && v.originator == 0U, "(1) cold: adopt, not originator");
    CHECK(dec == SX1276_FHSS_SNAP_DEC_SNAPPED, "(1) cold: SNAPPED (got %d)", (int)dec);
    CHECK(sx1276_rx_grid_adopted() == 1U, "(1) adopted");
    CHECK(sx1276_fhss_clock_valid() == 1U, "(1) clock valid after adopt");
}

static void test_follower_demotion_resets(void) {
    /* continues from (1): the adopted follower loses lock */
    CHECK(sx1276_rx_grid_on_demotion() == 1U, "(2) adopted clock IS reset");
    CHECK(sx1276_fhss_clock_valid() == 0U, "(2) clock invalid");
    CHECK(sx1276_rx_grid_adopted() == 0U, "(2) adoption cleared");
    CHECK(sx1276_fhss_authority_streak() == 0U, "(2) authority reset");
}

static void test_post_demotion_duplex_tx_recovery(void) {
    /* THE REGRESSION (PR #125 review): after (2) the base sends ONE
     * command, which lazily re-anchors the stale scheduler slot at tx_now.
     * A remote frame with a LATER phase must still be treated as
     * UNANCHORED and adopted (SNAPPED), never LOCKED_OUT. */
    tx_step(20000U);
    CHECK(sx1276_fhss_clock_valid() == 1U && sx1276_rx_grid_adopted() == 0U,
          "(3) precondition: valid clock + no adopted grid (the F6 recovery state)");
    sx1276_rx_grid_verdict_t v;
    /* the NEXT slot's hop, but its frame arrives later than our projection
     * of that slot's start (20000 + 200 = 20200): lagging. */
    const uint8_t hop = (uint8_t)((consumed_hop() + 1U) % N);
    sx1276_fhss_snap_decision_t dec = rx_frame(20500U, 0U, 0U, hop, &v);
    CHECK(v.originator == 0U, "(3) one TX is not an originator");
    CHECK(v.health == SX1276_FHSS_CLOCK_UNANCHORED, "(3) health UNANCHORED (got %d)", (int)v.health);
    CHECK(dec == SX1276_FHSS_SNAP_DEC_SNAPPED, "(3) later remote SNAPPED, not LOCKED_OUT (got %d)", (int)dec);
    CHECK(sx1276_rx_grid_adopted() == 1U, "(3) recovered: adopted");
    /* And the aligned, equal-phase echo after a fresh demotion + one TX. */
    (void)sx1276_rx_grid_on_demotion();
    tx_step(21000U);
    dec = rx_frame(21050U, 0U, 0U, consumed_hop(), &v);    /* our own slot, 50 ms later */
    CHECK(v.health == SX1276_FHSS_CLOCK_UNANCHORED && v.adopt == 1U,
          "(3) lagging aligned echo after one TX: UNANCHORED + adopt");
    CHECK(dec == SX1276_FHSS_SNAP_DEC_ALIGNED && sx1276_rx_grid_adopted() == 1U,
          "(3) ALIGNED + adopted");
}

static void test_originator_refuses_lagging_echo(void) {
    fresh_scheduler();
    const uint32_t last = stream_from(30000U);        /* last own TX at 31800 */
    const uint32_t now = last + 100U;                 /* 31900, same slot */
    const uint32_t abs_before = sx1276_fhss_clock_abs_slot(now);
    const uint32_t phase_before = sx1276_fhss_clock_in_slot_ms(now);
    sx1276_rx_grid_verdict_t v;
    /* A lagging follower echo of the slot BEFORE ours: our projection of
     * that slot's start is 31600, the frame implies 31900 -> later. Not
     * aligned (consumed hop is ours) -> a snap candidate -> must be
     * REFUSED. */
    const uint8_t prev = (uint8_t)((consumed_hop() + N - 1U) % N);
    sx1276_fhss_snap_decision_t dec = rx_frame(now, 0U, 0U, prev, &v);
    CHECK(v.originator == 1U, "(4) streaming: originator");
    CHECK(v.adopt == 0U && v.health == SX1276_FHSS_CLOCK_FRESH,
          "(4) lagging echo: no adopt, FRESH handed to consider_remote");
    CHECK(dec == SX1276_FHSS_SNAP_DEC_REJECTED_LOCKED_OUT,
          "(4) lagging snap refused: LOCKED_OUT (got %d)", (int)dec);
    CHECK(sx1276_rx_grid_adopted() == 0U, "(4) still not adopted");
    /* A lagging echo of OUR slot (aligned): decision ALIGNED, no re-anchor. */
    dec = rx_frame(now, 0U, 0U, consumed_hop(), &v);
    CHECK(dec == SX1276_FHSS_SNAP_DEC_ALIGNED && v.adopt == 0U,
          "(4) aligned lagging echo: ALIGNED but not adopted");
    CHECK(sx1276_fhss_clock_abs_slot(now) == abs_before &&
          sx1276_fhss_clock_in_slot_ms(now) == phase_before,
          "(4) clock phase untouched by the echo");
    CHECK(sx1276_rx_grid_adopted() == 0U, "(4) originator stays unadopted");
}

static void test_originator_adopts_leading_grid(void) {
    fresh_scheduler();
    const uint32_t last = stream_from(40000U);        /* last own TX at 41800 */
    const uint32_t now = last + 100U;                 /* 41900 */
    sx1276_rx_grid_verdict_t v;
    /* A frame for the NEXT slot (our projection: 42000) that implies a
     * start of 41900 -> that grid LEADS ours by 100 ms -> adopt, SNAPPED. */
    const uint8_t next = (uint8_t)((consumed_hop() + 1U) % N);
    sx1276_fhss_snap_decision_t dec = rx_frame(now, 0U, 0U, next, &v);
    CHECK(v.originator == 1U, "(5) streaming: originator");
    /* Round 4: a leading grid is handed STALE, not UNANCHORED, so the
     * epoch-drift barrier stays in force (see test 9). */
    CHECK(v.adopt == 1U && v.health == SX1276_FHSS_CLOCK_STALE,
          "(5) leading grid: adopt + STALE (drift barrier kept)");
    CHECK(dec == SX1276_FHSS_SNAP_DEC_SNAPPED, "(5) leading: SNAPPED (got %d)", (int)dec);
    CHECK(sx1276_rx_grid_adopted() == 1U, "(5) leading grid adopted -> follower");
    CHECK(sx1276_fhss_authority_streak() == 0U, "(5) authority cleared on adopt");
    /* Aligned + leading: our slot, but the frame implies it started 100 ms
     * EARLIER than we think (toa 200 ms at now = start + 100). */
    fresh_scheduler();
    const uint32_t last2 = stream_from(50000U);       /* 51800 */
    dec = rx_frame(last2 + 100U, 200000U, 0U, consumed_hop(), &v);
    CHECK(v.adopt == 1U && dec == SX1276_FHSS_SNAP_DEC_ALIGNED,
          "(5b) aligned leading echo: adopted on ALIGNED");
    CHECK(sx1276_rx_grid_adopted() == 1U, "(5b) adopted");
}

static void test_originator_demotion_keeps_clock(void) {
    fresh_scheduler();
    (void)stream_from(60000U);
    CHECK(sx1276_rx_grid_on_demotion() == 0U, "(6) self-anchored clock KEPT");
    CHECK(sx1276_fhss_clock_valid() == 1U, "(6) clock still valid");
    CHECK(sx1276_rx_grid_adopted() == 0U, "(6) still unadopted");
    CHECK(sx1276_fhss_authority_streak() >= SX1276_FHSS_AUTHORITY_MIN_STREAK,
          "(6) authority streak intact");
}

static void test_authority_decays_then_echo_adopted(void) {
    fresh_scheduler();
    const uint32_t last = stream_from(70000U);        /* 71800 */
    sx1276_rx_grid_verdict_t v;
    /* 1 s of silence: authority gone -> the same kind of lagging echo that
     * (4) refused is now adopted (UNANCHORED, SNAPPED). */
    const uint32_t now = last + SX1276_FHSS_AUTHORITY_STREAK_GAP_MS + 5U;
    const uint8_t prev = (uint8_t)((consumed_hop() + N - 1U) % N);
    const sx1276_fhss_snap_decision_t dec = rx_frame(now, 0U, 0U, prev, &v);
    CHECK(v.originator == 0U, "(7) silent 1 s: not an originator");
    CHECK(v.health == SX1276_FHSS_CLOCK_UNANCHORED && v.adopt == 1U, "(7) UNANCHORED + adopt");
    CHECK(dec == SX1276_FHSS_SNAP_DEC_SNAPPED, "(7) SNAPPED (got %d)", (int)dec);
    CHECK(sx1276_rx_grid_adopted() == 1U, "(7) adopted after decay");
}

static void test_exact_1s_commands_never_earn_authority(void) {
    fresh_scheduler();
    uint32_t t = 80000U;
    tx_step(t);
    for (uint32_t i = 1U; i < 12U; ++i) {
        t += SX1276_FHSS_AUTHORITY_STREAK_GAP_MS;     /* exactly the gap */
        tx_step(t);
    }
    sx1276_rx_grid_verdict_t v;
    const uint8_t prev = (uint8_t)((consumed_hop() + N - 1U) % N);
    const sx1276_fhss_snap_decision_t dec = rx_frame(t + 10U, 0U, 0U, prev, &v);
    CHECK(v.originator == 0U, "(8) exactly-1 s sender: no authority");
    CHECK(dec == SX1276_FHSS_SNAP_DEC_SNAPPED && sx1276_rx_grid_adopted() == 1U,
          "(8) it still adopts the peer (SNAPPED)");
}

static void test_forged_leading_epoch_is_drift_rejected(void) {
    /* PR #125 review round 4: schema v1 has no MIC, so the +/-1
     * epoch-drift rule is the only barrier against a forged header
     * teleporting the scheduler. A frame claiming epoch+2 reads as
     * "leading" (its slot start projects far in the future), so it must
     * NOT be handed UNANCHORED -- STALE keeps the drift check and
     * consider_remote rejects it, clock untouched. epoch+1 (a peer that
     * legitimately rolled the epoch just before us) is still adopted. */
    fresh_scheduler();
    const uint32_t last = stream_from(90000U);        /* 91800 */
    const uint32_t now = last + 100U;
    const uint32_t abs_before = sx1276_fhss_clock_abs_slot(now);
    const uint32_t phase_before = sx1276_fhss_clock_in_slot_ms(now);
    const uint8_t next = (uint8_t)((consumed_hop() + 1U) % N);
    sx1276_rx_grid_verdict_t v;
    sx1276_fhss_snap_decision_t dec =
        rx_frame_ep(now, 0U, 0U, EPOCH + 2U, next, &v);
    CHECK(v.originator == 1U && v.adopt == 1U,
          "(9) forged epoch+2 reads as leading (adopt verdict)");
    CHECK(v.health == SX1276_FHSS_CLOCK_STALE,
          "(9) ... but is handed STALE, not UNANCHORED (got %d)", (int)v.health);
    CHECK(dec == SX1276_FHSS_SNAP_DEC_REJECTED_EPOCH_DRIFT,
          "(9) drift barrier holds: REJECTED_EPOCH_DRIFT (got %d)", (int)dec);
    CHECK(sx1276_rx_grid_adopted() == 0U, "(9) not adopted");
    CHECK(sx1276_fhss_clock_abs_slot(now) == abs_before &&
          sx1276_fhss_clock_in_slot_ms(now) == phase_before,
          "(9) clock untouched by the forged frame");
    CHECK(sx1276_fhss_authority_streak() >= SX1276_FHSS_AUTHORITY_MIN_STREAK,
          "(9) authority intact after the rejection");
    /* Within the window: epoch+1, same leading geometry -> SNAPPED + adopted. */
    dec = rx_frame_ep(now, 0U, 0U, EPOCH + 1U, next, &v);
    CHECK(v.adopt == 1U && v.health == SX1276_FHSS_CLOCK_STALE,
          "(9b) epoch+1 leading: adopt + STALE");
    CHECK(dec == SX1276_FHSS_SNAP_DEC_SNAPPED && sx1276_rx_grid_adopted() == 1U,
          "(9b) epoch+1 within the drift window: SNAPPED + adopted (got %d)", (int)dec);
}

int main(void) {
    test_cold_follower_adopts();
    test_follower_demotion_resets();
    test_post_demotion_duplex_tx_recovery();
    test_originator_refuses_lagging_echo();
    test_originator_adopts_leading_grid();
    test_originator_demotion_keeps_clock();
    test_authority_decays_then_echo_adopted();
    test_exact_1s_commands_never_earn_authority();
    test_forged_leading_epoch_is_drift_rejected();
    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] rx_grid_policy: %d failure(s)\n", g_failures);
        return 1;
    }
    printf("[PASS] rx_grid_policy: 9 sequences incl. demotion->TX->RX recovery and the forged-epoch drift barrier, against the real consider_remote\n");
    return 0;
}
