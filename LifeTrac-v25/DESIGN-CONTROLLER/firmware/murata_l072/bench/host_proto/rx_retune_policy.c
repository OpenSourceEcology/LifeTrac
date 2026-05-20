/*
 * FCC-A6b-2-ii-γ-1: host test for radio/sx1276_rx_retune_policy.c.
 *
 * Pure-function tests of the RX retune decision. No HW.
 *
 * Cases:
 *   (1)  Not-init wins over busy (priority ordering).
 *   (2)  Not-init wins over time (priority ordering).
 *   (3)  Busy defers even when period has elapsed.
 *   (4)  Same-tick (delta = 0) → SKIP_TOO_SOON (period_ms ≥ 1).
 *   (5)  delta = period - 1 → SKIP_TOO_SOON (strict <).
 *   (6)  delta = period     → DO          (boundary: >= period).
 *   (7)  delta = period + 1 → DO.
 *   (8)  Very-large forward delta inside int32-positive range → DO.
 *   (9)  Backwards delta (small) → DO_WRAP.
 *   (10) Backwards delta (large, near full wrap) → DO_WRAP.
 *   (11) Real tick wrap (now_ms = 50, last = UINT32_MAX - 100) →
 *        elapsed ≈ 151 ms; with period 380 ms → SKIP_TOO_SOON.
 *   (12) Real tick wrap with enough elapsed → DO (wrap straddles
 *        UINT32_MAX, elapsed ≥ period).
 *   (13) DIM constant matches the max enum value + 1.
 *   (14) Numeric stability: DO=0, SKIP_NOT_INIT=1, SKIP_BUSY=2,
 *        SKIP_TOO_SOON=3, DO_WRAP=4 (FCC-B1-SUMMARY indexes counters
 *        by this enum — drift would silently misroute histograms).
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "sx1276_rx_retune_policy.h"

static int g_failures = 0;

#define CHECK(cond, ...)                                                \
    do {                                                                \
        if (!(cond)) {                                                  \
            ++g_failures;                                               \
            fprintf(stderr, "[FAIL] %s:%d: ", __FILE__, __LINE__);      \
            fprintf(stderr, __VA_ARGS__);                               \
            fprintf(stderr, "\n");                                      \
        }                                                               \
    } while (0)

static const char *dec_name(sx1276_rx_retune_decision_t d) {
    switch (d) {
        case SX1276_RX_RETUNE_DO:            return "DO";
        case SX1276_RX_RETUNE_SKIP_NOT_INIT: return "SKIP_NOT_INIT";
        case SX1276_RX_RETUNE_SKIP_BUSY:     return "SKIP_BUSY";
        case SX1276_RX_RETUNE_SKIP_TOO_SOON: return "SKIP_TOO_SOON";
        case SX1276_RX_RETUNE_DO_WRAP:       return "DO_WRAP";
        default:                             return "<unknown>";
    }
}

#define EXPECT_DEC(expr, expected)                                      \
    do {                                                                \
        const sx1276_rx_retune_decision_t _got = (expr);                \
        CHECK(_got == (expected),                                       \
              "%s: expected %s got %s", #expr,                          \
              dec_name(expected), dec_name(_got));                      \
    } while (0)

static void test_not_init_priority_over_busy(void) {
    EXPECT_DEC(sx1276_rx_retune_eval(1000U, 0U,
                                     /*busy=*/true,  /*ready=*/false),
               SX1276_RX_RETUNE_SKIP_NOT_INIT);
}

static void test_not_init_priority_over_time(void) {
    EXPECT_DEC(sx1276_rx_retune_eval(1000U, 0U,
                                     /*busy=*/false, /*ready=*/false),
               SX1276_RX_RETUNE_SKIP_NOT_INIT);
}

static void test_busy_defers_even_when_period_elapsed(void) {
    const uint32_t last = 0U;
    const uint32_t now  = last + (uint32_t)SX1276_RX_RETUNE_PERIOD_MS + 50U;
    EXPECT_DEC(sx1276_rx_retune_eval(now, last,
                                     /*busy=*/true, /*ready=*/true),
               SX1276_RX_RETUNE_SKIP_BUSY);
}

static void test_same_tick_is_too_soon(void) {
    EXPECT_DEC(sx1276_rx_retune_eval(5000U, 5000U,
                                     /*busy=*/false, /*ready=*/true),
               SX1276_RX_RETUNE_SKIP_TOO_SOON);
}

static void test_period_minus_one_is_too_soon(void) {
    const uint32_t last = 1000U;
    const uint32_t now  = last + (uint32_t)SX1276_RX_RETUNE_PERIOD_MS - 1U;
    EXPECT_DEC(sx1276_rx_retune_eval(now, last,
                                     /*busy=*/false, /*ready=*/true),
               SX1276_RX_RETUNE_SKIP_TOO_SOON);
}

static void test_exactly_period_is_do(void) {
    const uint32_t last = 1000U;
    const uint32_t now  = last + (uint32_t)SX1276_RX_RETUNE_PERIOD_MS;
    EXPECT_DEC(sx1276_rx_retune_eval(now, last,
                                     /*busy=*/false, /*ready=*/true),
               SX1276_RX_RETUNE_DO);
}

static void test_period_plus_one_is_do(void) {
    const uint32_t last = 1000U;
    const uint32_t now  = last + (uint32_t)SX1276_RX_RETUNE_PERIOD_MS + 1U;
    EXPECT_DEC(sx1276_rx_retune_eval(now, last,
                                     /*busy=*/false, /*ready=*/true),
               SX1276_RX_RETUNE_DO);
}

static void test_large_forward_delta_is_do(void) {
    /* Largest int32-positive elapsed before reinterpretation flips
     * to negative. Caller fed coherent inputs, so DO is correct. */
    EXPECT_DEC(sx1276_rx_retune_eval(0x7FFFFFFFU, 0U,
                                     /*busy=*/false, /*ready=*/true),
               SX1276_RX_RETUNE_DO);
}

static void test_small_backwards_delta_is_wrap(void) {
    /* Caller fed last_retune_ms > now_ms by a little — looks negative
     * after int32 reinterpret → DO_WRAP. γ-2 will re-anchor. */
    EXPECT_DEC(sx1276_rx_retune_eval(100U, 200U,
                                     /*busy=*/false, /*ready=*/true),
               SX1276_RX_RETUNE_DO_WRAP);
}

static void test_large_backwards_delta_is_wrap(void) {
    /* now ≈ 1, last ≈ UINT32_MAX - 5 — uint32 elapsed = ~6 (small
     * positive), int32 reinterpret stays small positive → would be
     * TOO_SOON. To exercise the "backwards" branch we feed values
     * where (now - last) as uint32 is just above 0x80000000U. */
    const uint32_t last = 0U;
    const uint32_t now  = 0x80000001U;
    /* delta = 0x80000001 reinterpret = INT32_MIN + 1 (very negative)
     * → DO_WRAP. */
    EXPECT_DEC(sx1276_rx_retune_eval(now, last,
                                     /*busy=*/false, /*ready=*/true),
               SX1276_RX_RETUNE_DO_WRAP);
}

static void test_real_tick_wrap_small_elapsed_is_too_soon(void) {
    /* Monotonic ms tick wrapped through 0 recently. Elapsed since
     * last retune: 50 - (UINT32_MAX - 100) as uint32 = 151 ms. With
     * period 380 ms → SKIP_TOO_SOON. The int32 reinterpret of 151 is
     * still +151 (small positive) — wrap is "real" but invisible to
     * the policy, which is correct. */
    const uint32_t last = UINT32_MAX - 100U;
    const uint32_t now  = 50U;
    EXPECT_DEC(sx1276_rx_retune_eval(now, last,
                                     /*busy=*/false, /*ready=*/true),
               SX1276_RX_RETUNE_SKIP_TOO_SOON);
}

static void test_real_tick_wrap_enough_elapsed_is_do(void) {
    /* Same wrap, but now we've waited > period ms across the
     * boundary. Elapsed uint32 = (now - last) wraps to 1 +
     * SX1276_RX_RETUNE_PERIOD_MS. → DO. */
    const uint32_t last = UINT32_MAX - 100U;
    const uint32_t now  = (uint32_t)SX1276_RX_RETUNE_PERIOD_MS + 1U;
    /* Sanity: elapsed should be exactly period + 102. */
    const uint32_t elapsed = now - last;
    CHECK(elapsed == (uint32_t)SX1276_RX_RETUNE_PERIOD_MS + 102U,
          "wrap-elapsed precondition failed: got %" PRIu32, elapsed);
    EXPECT_DEC(sx1276_rx_retune_eval(now, last,
                                     /*busy=*/false, /*ready=*/true),
               SX1276_RX_RETUNE_DO);
}

static void test_dim_matches_max_enum(void) {
    CHECK(SX1276_RX_RETUNE_DECISION_DIM ==
              ((unsigned int)SX1276_RX_RETUNE_DO_WRAP + 1U),
          "SX1276_RX_RETUNE_DECISION_DIM (%u) must equal max-enum + 1 (%u)",
          (unsigned int)SX1276_RX_RETUNE_DECISION_DIM,
          (unsigned int)SX1276_RX_RETUNE_DO_WRAP + 1U);
}

static void test_enum_numeric_values_stable(void) {
    /* FCC-B1-SUMMARY's additive URC will index per-outcome counters
     * by these values. If anyone reorders the enum, the URC
     * histograms silently mis-route. Pin the wire here. */
    CHECK((int)SX1276_RX_RETUNE_DO            == 0, "DO must be 0");
    CHECK((int)SX1276_RX_RETUNE_SKIP_NOT_INIT == 1, "SKIP_NOT_INIT must be 1");
    CHECK((int)SX1276_RX_RETUNE_SKIP_BUSY     == 2, "SKIP_BUSY must be 2");
    CHECK((int)SX1276_RX_RETUNE_SKIP_TOO_SOON == 3, "SKIP_TOO_SOON must be 3");
    CHECK((int)SX1276_RX_RETUNE_DO_WRAP       == 4, "DO_WRAP must be 4");
}

int main(void) {
    test_not_init_priority_over_busy();
    test_not_init_priority_over_time();
    test_busy_defers_even_when_period_elapsed();
    test_same_tick_is_too_soon();
    test_period_minus_one_is_too_soon();
    test_exactly_period_is_do();
    test_period_plus_one_is_do();
    test_large_forward_delta_is_do();
    test_small_backwards_delta_is_wrap();
    test_large_backwards_delta_is_wrap();
    test_real_tick_wrap_small_elapsed_is_too_soon();
    test_real_tick_wrap_enough_elapsed_is_do();
    test_dim_matches_max_enum();
    test_enum_numeric_values_stable();

    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] rx_retune_policy: %d failure(s)\n", g_failures);
        return EXIT_FAILURE;
    }
    printf("[PASS] rx_retune_policy: 14 cases\n");
    return EXIT_SUCCESS;
}
