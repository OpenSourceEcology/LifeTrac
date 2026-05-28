/*
 * FCC-A6c-2-c-i-α': host golden-vector test for the RX scan walker
 * (radio/sx1276_rx_scan_walker.c).
 *
 * Cases:
 *   (1) NULL out_idx → returns 0UL, walker NOT advanced.
 *   (2) Fresh reset → first call yields idx=0 at FIRST_CENTER_HZ.
 *   (3) 50 sequential calls cover every channel 0..49 exactly once
 *       in linear order.
 *   (4) Call 51 wraps to idx=0 (modular advance).
 *   (5) sx1276_rx_scan_walker_reset() restores walker to idx=0 mid-walk.
 *   (6) sx1276_rx_scan_walker_peek_next() does not advance the walker.
 *   (7) Center-Hz return matches sx1276_fhss_chantab_center_hz() byte-
 *       for-byte for every visited channel (no off-by-one in the
 *       walker's index↔frequency mapping).
 *
 * No HW deps. Mirrors the test patterns used by rx_scan_policy.c +
 * rx_scan_counters.c.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "sx1276_fhss_chantab.h"
#include "sx1276_rx_scan_walker.h"

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

static void test_null_out_idx(void) {
    sx1276_rx_scan_walker_reset();
    const uint8_t before = sx1276_rx_scan_walker_peek_next();
    const uint32_t hz = sx1276_rx_scan_walker_next(NULL);
    const uint8_t after = sx1276_rx_scan_walker_peek_next();
    CHECK(hz == 0UL,
          "(1) NULL out_idx: hz expected 0 got %" PRIu32, hz);
    CHECK(before == after,
          "(1) NULL out_idx: walker advanced (before=%u after=%u)",
          before, after);
}

static void test_first_call_is_idx0(void) {
    sx1276_rx_scan_walker_reset();
    uint8_t idx = 0xFFU;
    const uint32_t hz = sx1276_rx_scan_walker_next(&idx);
    CHECK(idx == 0U,
          "(2) first call: idx expected 0 got %u", idx);
    CHECK(hz == SX1276_FHSS_FIRST_CENTER_HZ,
          "(2) first call: hz expected %" PRIu32 " got %" PRIu32,
          (uint32_t)SX1276_FHSS_FIRST_CENTER_HZ, hz);
}

static void test_full_pass_visits_all(void) {
    sx1276_rx_scan_walker_reset();
    bool seen[SX1276_FHSS_CHANNEL_COUNT] = { false };
    for (uint8_t i = 0; i < SX1276_FHSS_CHANNEL_COUNT; ++i) {
        uint8_t idx = 0xFFU;
        const uint32_t hz = sx1276_rx_scan_walker_next(&idx);
        CHECK(idx < SX1276_FHSS_CHANNEL_COUNT,
              "(3) call %u: idx %u out of range", i, idx);
        CHECK(idx == i,
              "(3) call %u: linear-order expected idx=%u got %u",
              i, i, idx);
        CHECK(!seen[idx],
              "(3) call %u: idx %u repeated within first pass",
              i, idx);
        seen[idx] = true;
        const uint32_t expect_hz = sx1276_fhss_chantab_center_hz(idx);
        CHECK(hz == expect_hz,
              "(3+7) call %u: hz mismatch idx=%u expected %" PRIu32
              " got %" PRIu32, i, idx, expect_hz, hz);
    }
    for (uint8_t i = 0; i < SX1276_FHSS_CHANNEL_COUNT; ++i) {
        CHECK(seen[i],
              "(3) full pass missed channel %u", i);
    }
}

static void test_wrap_after_50(void) {
    sx1276_rx_scan_walker_reset();
    for (uint8_t i = 0; i < SX1276_FHSS_CHANNEL_COUNT; ++i) {
        uint8_t idx = 0xFFU;
        (void)sx1276_rx_scan_walker_next(&idx);
    }
    uint8_t idx = 0xFFU;
    const uint32_t hz = sx1276_rx_scan_walker_next(&idx);
    CHECK(idx == 0U,
          "(4) wrap: 51st call expected idx=0 got %u", idx);
    CHECK(hz == SX1276_FHSS_FIRST_CENTER_HZ,
          "(4) wrap: 51st call hz expected %" PRIu32 " got %" PRIu32,
          (uint32_t)SX1276_FHSS_FIRST_CENTER_HZ, hz);
}

static void test_reset_mid_walk(void) {
    sx1276_rx_scan_walker_reset();
    uint8_t idx = 0xFFU;
    for (uint8_t i = 0; i < 17U; ++i) {
        (void)sx1276_rx_scan_walker_next(&idx);
    }
    CHECK(sx1276_rx_scan_walker_peek_next() == 17U,
          "(5) pre-reset peek: expected 17 got %u",
          sx1276_rx_scan_walker_peek_next());
    sx1276_rx_scan_walker_reset();
    CHECK(sx1276_rx_scan_walker_peek_next() == 0U,
          "(5) post-reset peek: expected 0 got %u",
          sx1276_rx_scan_walker_peek_next());
    const uint32_t hz = sx1276_rx_scan_walker_next(&idx);
    CHECK(idx == 0U,
          "(5) post-reset first call: expected idx=0 got %u", idx);
    CHECK(hz == SX1276_FHSS_FIRST_CENTER_HZ,
          "(5) post-reset first call: hz mismatch");
}

static void test_peek_does_not_advance(void) {
    sx1276_rx_scan_walker_reset();
    uint8_t idx = 0xFFU;
    (void)sx1276_rx_scan_walker_next(&idx);  /* walker now at 1 */
    const uint8_t p1 = sx1276_rx_scan_walker_peek_next();
    const uint8_t p2 = sx1276_rx_scan_walker_peek_next();
    const uint8_t p3 = sx1276_rx_scan_walker_peek_next();
    CHECK(p1 == 1U && p2 == 1U && p3 == 1U,
          "(6) peek advanced walker: p1=%u p2=%u p3=%u", p1, p2, p3);
    (void)sx1276_rx_scan_walker_next(&idx);
    CHECK(idx == 1U,
          "(6) call after peeks: expected idx=1 got %u", idx);
}

int main(void) {
    test_null_out_idx();
    test_first_call_is_idx0();
    test_full_pass_visits_all();
    test_wrap_after_50();
    test_reset_mid_walk();
    test_peek_does_not_advance();
    if (g_failures > 0) {
        fprintf(stderr, "rx_scan_walker: %d FAILURES\n", g_failures);
        return EXIT_FAILURE;
    }
    printf("rx_scan_walker: all golden vectors PASS\n");
    return EXIT_SUCCESS;
}
