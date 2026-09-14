/*
 * RS-12.15 v2 (2026-09-14): originator-authority unit test (host-cc).
 *
 * Pins the discriminator that separates a STREAMING originator from a
 * post-demotion duplex node (the PR #125 review case): authority needs
 * MIN_STREAK consecutive own transmissions each within STREAK_GAP_MS,
 * adopting a remote grid clears it, and neither an invalid clock nor an
 * adopted grid can ever hold it.
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

static void stream(uint32_t start_ms, uint32_t n, uint32_t cadence_ms) {
    for (uint32_t i = 0U; i < n; ++i) {
        sx1276_fhss_authority_note_tx(start_ms + i * cadence_ms);
    }
}

static void test_reset_has_no_authority(void) {
    sx1276_fhss_authority_reset();
    CHECK(sx1276_fhss_authority_streak() == 0U, "(1) reset streak 0");
    CHECK(sx1276_fhss_authority_is_originator(1U, 0U) == 0U,
          "(1) valid clock + no adopted grid but NO streak -> not originator");
}

static void test_streaming_earns_authority(void) {
    sx1276_fhss_authority_reset();
    /* Image stream: one fragment per 200 ms slot. */
    stream(1000U, SX1276_FHSS_AUTHORITY_MIN_STREAK - 1U, 200U);
    CHECK(sx1276_fhss_authority_is_originator(1U, 0U) == 0U,
          "(2) MIN_STREAK-1 transmissions: not yet");
    sx1276_fhss_authority_note_tx(1000U + (SX1276_FHSS_AUTHORITY_MIN_STREAK - 1U) * 200U);
    CHECK(sx1276_fhss_authority_streak() == SX1276_FHSS_AUTHORITY_MIN_STREAK,
          "(2) streak == MIN_STREAK");
    CHECK(sx1276_fhss_authority_is_originator(1U, 0U) == 1U,
          "(2) MIN_STREAK transmissions: originator");
    /* 40 ms fragment pacing inside a train clears it even faster. */
    sx1276_fhss_authority_reset();
    stream(5000U, 8U, 40U);
    CHECK(sx1276_fhss_authority_is_originator(1U, 0U) == 1U,
          "(2) 8 fragments at 40 ms: originator");
}

static void test_command_sender_never_earns_it(void) {
    /* A base under the RS-12.14 stream gate sends >= 1 s apart: each
     * transmission is a fresh streak of 1. */
    sx1276_fhss_authority_reset();
    for (uint32_t i = 0U; i < 40U; ++i) {
        sx1276_fhss_authority_note_tx(1000U + i * 1500U);
        CHECK(sx1276_fhss_authority_streak() == 1U,
              "(3) 1.5 s spaced commands never chain (i=%u)", i);
    }
    CHECK(sx1276_fhss_authority_is_originator(1U, 0U) == 0U,
          "(3) 40 spaced commands: still not an originator");
    /* Two immediate copies (37 ms apart) chain to 2, never to 8. */
    sx1276_fhss_authority_reset();
    for (uint32_t i = 0U; i < 10U; ++i) {
        sx1276_fhss_authority_note_tx(1000U + i * 2000U);
        sx1276_fhss_authority_note_tx(1000U + i * 2000U + 37U);
    }
    CHECK(sx1276_fhss_authority_streak() == 2U, "(3) copy pairs chain to 2");
    CHECK(sx1276_fhss_authority_is_originator(1U, 0U) == 0U,
          "(3) copy pairs: not an originator");
    /* Exactly at the gap the streak continues; one ms past it restarts. */
    sx1276_fhss_authority_reset();
    sx1276_fhss_authority_note_tx(100U);
    sx1276_fhss_authority_note_tx(100U + SX1276_FHSS_AUTHORITY_STREAK_GAP_MS);
    CHECK(sx1276_fhss_authority_streak() == 2U, "(3) gap == GAP_MS chains");
    sx1276_fhss_authority_note_tx(100U + SX1276_FHSS_AUTHORITY_STREAK_GAP_MS
                                  + SX1276_FHSS_AUTHORITY_STREAK_GAP_MS + 1U);
    CHECK(sx1276_fhss_authority_streak() == 1U, "(3) gap > GAP_MS restarts");
}

static void test_demotion_tx_rx_regression(void) {
    /* PR #125 review: after a follower demotion (clock reset, adopted
     * cleared, authority reset) a single duplex TX re-validates the stale
     * grid. That node must NOT be an originator, so the RX gate keeps
     * the UNANCHORED recovery tier and adopts even an equal/later remote. */
    sx1276_fhss_authority_reset();               /* demotion edge */
    sx1276_fhss_authority_note_tx(20000U);       /* one command TX */
    CHECK(sx1276_fhss_authority_is_originator(1U, 0U) == 0U,
          "(4) post-demotion single TX: NOT an originator (recovery stays open)");
    for (uint32_t i = 1U; i < SX1276_FHSS_AUTHORITY_MIN_STREAK - 1U; ++i) {
        sx1276_fhss_authority_note_tx(20000U + i * 500U);
        CHECK(sx1276_fhss_authority_is_originator(1U, 0U) == 0U,
              "(4) below MIN_STREAK after demotion: still not (i=%u)", i);
    }
    /* Adopting the remote (recovery succeeded) clears any streak. */
    stream(30000U, 12U, 200U);
    CHECK(sx1276_fhss_authority_is_originator(1U, 0U) == 1U, "(4) streaming again");
    sx1276_fhss_authority_note_adopt();
    CHECK(sx1276_fhss_authority_streak() == 0U, "(4) adopt clears the streak");
    CHECK(sx1276_fhss_authority_is_originator(1U, 0U) == 0U,
          "(4) after adopting: not an originator");
}

static void test_gates_on_clock_and_adoption(void) {
    sx1276_fhss_authority_reset();
    stream(1000U, 20U, 100U);
    CHECK(sx1276_fhss_authority_is_originator(0U, 0U) == 0U,
          "(5) invalid clock: never an originator");
    CHECK(sx1276_fhss_authority_is_originator(1U, 1U) == 0U,
          "(5) adopted grid: never an originator");
    CHECK(sx1276_fhss_authority_is_originator(1U, 0U) == 1U,
          "(5) valid + unadopted + streaming: originator");
    CHECK(sx1276_fhss_authority_streak_max() == 20U, "(5) streak_max tracks");
}

static void test_tick_wrap(void) {
    sx1276_fhss_authority_reset();
    sx1276_fhss_authority_note_tx(0xFFFFFF00U);
    sx1276_fhss_authority_note_tx(0x00000050U);   /* 336 ms later, across wrap */
    CHECK(sx1276_fhss_authority_streak() == 2U, "(6) wrap-spanning gap chains");
}

int main(void) {
    test_reset_has_no_authority();
    test_streaming_earns_authority();
    test_command_sender_never_earns_it();
    test_demotion_tx_rx_regression();
    test_gates_on_clock_and_adoption();
    test_tick_wrap();
    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] fhss_authority: %d failure(s)\n", g_failures);
        return 1;
    }
    printf("[PASS] fhss_authority: 6 test fns incl. the demotion->TX->RX regression\n");
    return 0;
}
