/*
 * RS-12.15 v2 (2026-09-14): originator-authority unit test (host-cc).
 *
 * Pins the discriminator that separates a STREAMING originator from a
 * post-demotion duplex node: authority needs MIN_STREAK consecutive own
 * transmissions each STRICTLY within STREAK_GAP_MS of the previous one,
 * it DECAYS one gap after the last own TX, adopting a remote grid clears
 * it, and neither an invalid clock nor an adopted grid can ever hold it.
 * The policy-level demotion -> TX -> RX regression (with the real
 * consider_remote) lives in rx_grid_policy.c; this file pins the pure
 * streak arithmetic.
 */

#include <stdio.h>
#include <stdint.h>

#include "sx1276_fhss_authority.h"

static int g_failures = 0;

#define CHECK(cond, ...) do {                                          \
    if (!(cond)) {                                                     \
        ++g_failures;                                                  \
        fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__);           \
        fprintf(stderr, __VA_ARGS__);                                  \
        fputc('\n', stderr);                                           \
    }                                                                  \
} while (0)

#define GAP  SX1276_FHSS_AUTHORITY_STREAK_GAP_MS
#define MINS SX1276_FHSS_AUTHORITY_MIN_STREAK

static uint32_t stream(uint32_t start_ms, uint32_t n, uint32_t cadence_ms) {
    uint32_t t = start_ms;
    for (uint32_t i = 0U; i < n; ++i) {
        t = start_ms + i * cadence_ms;
        sx1276_fhss_authority_note_tx(t);
    }
    return t;   /* time of the last TX */
}

static void test_reset_has_no_authority(void) {
    sx1276_fhss_authority_reset();
    CHECK(sx1276_fhss_authority_streak() == 0U, "(1) reset streak 0");
    CHECK(sx1276_fhss_authority_is_originator(1000U, 1U, 0U) == 0U,
          "(1) valid clock + no adopted grid but NO streak -> not originator");
}

static void test_streaming_earns_authority(void) {
    sx1276_fhss_authority_reset();
    uint32_t last = stream(1000U, MINS - 1U, 200U);
    CHECK(sx1276_fhss_authority_is_originator(last + 10U, 1U, 0U) == 0U,
          "(2) MIN_STREAK-1 transmissions: not yet");
    last = stream(1000U + (MINS - 1U) * 200U, 1U, 200U);
    CHECK(sx1276_fhss_authority_streak() == MINS, "(2) streak == MIN_STREAK");
    CHECK(sx1276_fhss_authority_is_originator(last + 10U, 1U, 0U) == 1U,
          "(2) MIN_STREAK transmissions: originator");
    /* 40 ms fragment pacing inside a train clears it even faster. */
    sx1276_fhss_authority_reset();
    last = stream(5000U, 8U, 40U);
    CHECK(sx1276_fhss_authority_is_originator(last + 30U, 1U, 0U) == 1U,
          "(2) 8 fragments at 40 ms: originator");
    /* Sparse 2-fragment trains at 1.5 fps (the leg Q/R instrument):
     * 40 ms inside the pair, ~627 ms between pairs -- both chain. */
    sx1276_fhss_authority_reset();
    uint32_t t = 7000U;
    for (uint32_t train = 0U; train < 5U; ++train) {
        sx1276_fhss_authority_note_tx(t);
        sx1276_fhss_authority_note_tx(t + 40U);
        t += 667U;
    }
    CHECK(sx1276_fhss_authority_streak() == 10U, "(2) sparse trains chain (10)");
    CHECK(sx1276_fhss_authority_is_originator(t - 667U + 400U, 1U, 0U) == 1U,
          "(2) 1.5 fps 2-fragment stream: originator between trains");
}

static void test_command_sender_never_earns_it(void) {
    /* A base under the RS-12.14 stream gate sends >= 1.0 s apart. */
    sx1276_fhss_authority_reset();
    for (uint32_t i = 0U; i < 40U; ++i) {
        sx1276_fhss_authority_note_tx(1000U + i * 1500U);
        CHECK(sx1276_fhss_authority_streak() == 1U,
              "(3) 1.5 s spaced commands never chain (i=%u)", i);
    }
    CHECK(sx1276_fhss_authority_is_originator(1000U + 39U * 1500U + 5U, 1U, 0U) == 0U,
          "(3) 40 spaced commands: still not an originator");
    /* EXACTLY the gap apart (the stream gate's >= 1.0 s boundary): the
     * chain test is strict, so these never chain either (round 3). */
    sx1276_fhss_authority_reset();
    for (uint32_t i = 0U; i < 12U; ++i) {
        sx1276_fhss_authority_note_tx(2000U + i * GAP);
    }
    CHECK(sx1276_fhss_authority_streak() == 1U, "(3) exactly-GAP spacing never chains");
    CHECK(sx1276_fhss_authority_is_originator(2000U + 11U * GAP + 1U, 1U, 0U) == 0U,
          "(3) 12 commands exactly 1 s apart: not an originator");
    /* One ms inside the gap chains; the boundary itself does not. */
    sx1276_fhss_authority_reset();
    sx1276_fhss_authority_note_tx(100U);
    sx1276_fhss_authority_note_tx(100U + GAP - 1U);
    CHECK(sx1276_fhss_authority_streak() == 2U, "(3) gap == GAP-1 chains");
    sx1276_fhss_authority_note_tx(100U + GAP - 1U + GAP);
    CHECK(sx1276_fhss_authority_streak() == 1U, "(3) gap == GAP restarts");
    /* Two immediate copies (37 ms apart) chain to 2, never to 8. */
    sx1276_fhss_authority_reset();
    for (uint32_t i = 0U; i < 10U; ++i) {
        sx1276_fhss_authority_note_tx(1000U + i * 2000U);
        sx1276_fhss_authority_note_tx(1000U + i * 2000U + 37U);
    }
    CHECK(sx1276_fhss_authority_streak() == 2U, "(3) copy pairs chain to 2");
    CHECK(sx1276_fhss_authority_is_originator(1000U + 9U * 2000U + 40U, 1U, 0U) == 0U,
          "(3) copy pairs: not an originator");
}

static void test_post_demotion_single_tx(void) {
    /* The pure half of the PR #125 regression (the policy-level version
     * with the real consider_remote is in rx_grid_policy.c). */
    sx1276_fhss_authority_reset();               /* demotion edge */
    sx1276_fhss_authority_note_tx(20000U);       /* one command TX */
    CHECK(sx1276_fhss_authority_is_originator(20010U, 1U, 0U) == 0U,
          "(4) post-demotion single TX: NOT an originator");
    for (uint32_t i = 1U; i < MINS - 1U; ++i) {
        sx1276_fhss_authority_note_tx(20000U + i * 500U);
        CHECK(sx1276_fhss_authority_is_originator(20000U + i * 500U + 10U, 1U, 0U) == 0U,
              "(4) below MIN_STREAK after demotion: still not (i=%u)", i);
    }
    uint32_t last = stream(30000U, 12U, 200U);
    CHECK(sx1276_fhss_authority_is_originator(last + 10U, 1U, 0U) == 1U, "(4) streaming again");
    sx1276_fhss_authority_note_adopt();
    CHECK(sx1276_fhss_authority_streak() == 0U, "(4) adopt clears the streak");
    CHECK(sx1276_fhss_authority_is_originator(last + 20U, 1U, 0U) == 0U,
          "(4) after adopting: not an originator");
}

static void test_decay_when_streaming_stops(void) {
    /* Round 3: a node that bursts 8 queued commands 250 ms apart (the idle
     * drain's poll cadence) and then goes quiet must lose authority one
     * gap after its last TX -- otherwise a post-demotion base could sit on
     * a stale self-anchored grid refusing the tractor forever. */
    sx1276_fhss_authority_reset();
    uint32_t last = stream(40000U, 8U, 250U);
    CHECK(sx1276_fhss_authority_is_originator(last + 100U, 1U, 0U) == 1U,
          "(5) right after the burst: originator (transient)");
    CHECK(sx1276_fhss_authority_is_originator(last + GAP - 1U, 1U, 0U) == 1U,
          "(5) one ms before the gap: still");
    CHECK(sx1276_fhss_authority_is_originator(last + GAP, 1U, 0U) == 0U,
          "(5) at the gap: authority decayed");
    CHECK(sx1276_fhss_authority_is_originator(last + 60000U, 1U, 0U) == 0U,
          "(5) a minute later: still decayed");
    /* The streak value itself is not consulted while silent: only a new
     * TX can restart it, and it restarts at 1. */
    sx1276_fhss_authority_note_tx(last + 60000U);
    CHECK(sx1276_fhss_authority_streak() == 1U, "(5) next TX after silence restarts at 1");
}

static void test_gates_on_clock_and_adoption(void) {
    sx1276_fhss_authority_reset();
    uint32_t last = stream(1000U, 20U, 100U);
    CHECK(sx1276_fhss_authority_is_originator(last + 10U, 0U, 0U) == 0U,
          "(6) invalid clock: never an originator");
    CHECK(sx1276_fhss_authority_is_originator(last + 10U, 1U, 1U) == 0U,
          "(6) adopted grid: never an originator");
    CHECK(sx1276_fhss_authority_is_originator(last + 10U, 1U, 0U) == 1U,
          "(6) valid + unadopted + streaming: originator");
    CHECK(sx1276_fhss_authority_streak_max() == 20U, "(6) streak_max tracks");
}

static void test_tick_wrap(void) {
    sx1276_fhss_authority_reset();
    sx1276_fhss_authority_note_tx(0xFFFFFF00U);
    sx1276_fhss_authority_note_tx(0x00000050U);   /* 336 ms later, across wrap */
    CHECK(sx1276_fhss_authority_streak() == 2U, "(7) wrap-spanning gap chains");
    for (uint32_t i = 0U; i < 8U; ++i) {
        sx1276_fhss_authority_note_tx(0x00000050U + (i + 1U) * 100U);
    }
    CHECK(sx1276_fhss_authority_is_originator(0x00000050U + 8U * 100U + 50U, 1U, 0U) == 1U,
          "(7) authority across the wrap");
}

int main(void) {
    test_reset_has_no_authority();
    test_streaming_earns_authority();
    test_command_sender_never_earns_it();
    test_post_demotion_single_tx();
    test_decay_when_streaming_stops();
    test_gates_on_clock_and_adoption();
    test_tick_wrap();
    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] fhss_authority: %d failure(s)\n", g_failures);
        return 1;
    }
    printf("[PASS] fhss_authority: 7 test fns (strict gap, decay, post-demotion single TX)\n");
    return 0;
}
