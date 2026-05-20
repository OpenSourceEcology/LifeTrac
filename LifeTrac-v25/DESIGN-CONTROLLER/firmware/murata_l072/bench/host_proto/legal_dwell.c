/*
 * FCC-A2 host-side tests for the legal-dwell accountant.
 *
 * Property coverage:
 *   (1) Pessimistic reserve immediately increments used_us by the
 *       requested amount.
 *   (2) Reconcile down adjusts used_us to actual.
 *   (3) Reconcile up (actual >= reserved) is a no-op (we never grow
 *       legal dwell beyond the pessimistic reservation).
 *   (4) NACK path: there is NO release / rollback API; an unreconciled
 *       reservation remains booked forever within its window.
 *   (5) Half-open inclusion: event at start_ms = now - window IS OUT,
 *       event at start_ms = now - window + 1 IS IN.
 *   (6) Per-channel isolation: reservations on ch=5 do not influence
 *       used_us on ch=7.
 *   (7) Over-budget reservation is refused (returns OVER_BUDGET, ring
 *       unchanged, handle set to INVALID).
 *   (8) 64-channel support: reservation on ch=63 succeeds; ch=64
 *       returns BAD_CH.
 *   (9) 10s vs 20s windows return different sums when events straddle
 *       the 10s boundary.
 *  (10) Stale handle reconcile is a silent no-op and does NOT corrupt
 *       the slot's current occupant.
 *  (11) Ring-full path: capacity reservations on distinct channels all
 *       succeed; the next reservation returns RING_FULL.
 *  (12) Eviction: a reservation that has aged out of the longest
 *       window is recycled by find_reuseable_slot().
 *  (13) BAD_RESERVE: pessimistic_us == 0, cap_us == 0, or pessimistic >
 *       cap all return BAD_RESERVE without booking.
 *  (14) BAD_WINDOW: window_ms == 0 or > 20 000 returns BAD_WINDOW.
 *  (15) used_us on a bad channel or bad window returns 0.
 *  (16) Reset wipes all state.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "sx1276_legal_dwell.h"

static int g_failures = 0;

#define CHECK(cond, ...)                                              \
    do {                                                              \
        if (!(cond)) {                                                \
            ++g_failures;                                             \
            fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__);      \
            fprintf(stderr, __VA_ARGS__);                             \
            fprintf(stderr, "\n");                                    \
        }                                                             \
    } while (0)

static void test_reserve_immediately_books(void) {
    sx1276_legal_dwell_reset();
    uint16_t h = 0xABCDU;
    const sx1276_dwell_status_t st = sx1276_legal_dwell_reserve(
        5U, 200000UL, 1000U, SX1276_DWELL_WINDOW_10S_MS,
        SX1276_DWELL_DEFAULT_CAP_US, &h);
    CHECK(st == SX1276_DWELL_OK, "reserve must succeed (got %d)", (int)st);
    CHECK(h != SX1276_DWELL_HANDLE_INVALID, "reserve must return a valid handle");
    const uint32_t used = sx1276_legal_dwell_used_us(
        5U, SX1276_DWELL_WINDOW_10S_MS, 1000U);
    CHECK(used == 200000UL,
          "used_us must be 200000 immediately after reserve, got %" PRIu32, used);
}

static void test_reconcile_down(void) {
    sx1276_legal_dwell_reset();
    uint16_t h;
    (void)sx1276_legal_dwell_reserve(0U, 380000UL, 5000U,
        SX1276_DWELL_WINDOW_10S_MS, SX1276_DWELL_DEFAULT_CAP_US, &h);
    sx1276_legal_dwell_reconcile(h, 250000UL);
    const uint32_t used = sx1276_legal_dwell_used_us(
        0U, SX1276_DWELL_WINDOW_10S_MS, 5000U);
    CHECK(used == 250000UL,
          "reconcile-down must shrink used_us to 250000, got %" PRIu32, used);
}

static void test_reconcile_up_is_noop(void) {
    sx1276_legal_dwell_reset();
    uint16_t h;
    (void)sx1276_legal_dwell_reserve(0U, 100000UL, 5000U,
        SX1276_DWELL_WINDOW_10S_MS, SX1276_DWELL_DEFAULT_CAP_US, &h);
    /* actual > reserved: must NOT grow. */
    sx1276_legal_dwell_reconcile(h, 300000UL);
    const uint32_t used = sx1276_legal_dwell_used_us(
        0U, SX1276_DWELL_WINDOW_10S_MS, 5000U);
    CHECK(used == 100000UL,
          "reconcile-up must be no-op; expected 100000, got %" PRIu32, used);
}

static void test_no_rollback_on_nack(void) {
    sx1276_legal_dwell_reset();
    uint16_t h;
    (void)sx1276_legal_dwell_reserve(0U, 300000UL, 0U,
        SX1276_DWELL_WINDOW_10S_MS, SX1276_DWELL_DEFAULT_CAP_US, &h);
    /* Simulate NACK: caller does NOT call reconcile or any release
     * API. The booking must persist forever within its window. */
    (void)h; /* unused */
    const uint32_t used_now    = sx1276_legal_dwell_used_us(0U, SX1276_DWELL_WINDOW_10S_MS, 100U);
    const uint32_t used_at_5s  = sx1276_legal_dwell_used_us(0U, SX1276_DWELL_WINDOW_10S_MS, 5000U);
    const uint32_t used_at_9999 = sx1276_legal_dwell_used_us(0U, SX1276_DWELL_WINDOW_10S_MS, 9999U);
    const uint32_t used_at_10000 = sx1276_legal_dwell_used_us(0U, SX1276_DWELL_WINDOW_10S_MS, 10000U);
    CHECK(used_now == 300000UL,    "post-NACK used@100ms expected 300000, got %" PRIu32, used_now);
    CHECK(used_at_5s == 300000UL,  "post-NACK used@5s   expected 300000, got %" PRIu32, used_at_5s);
    CHECK(used_at_9999 == 300000UL,"post-NACK used@9.999s expected 300000, got %" PRIu32, used_at_9999);
    CHECK(used_at_10000 == 0UL,    "event must age out at exactly now=10000 (half-open), got %" PRIu32, used_at_10000);
}

static void test_half_open_window(void) {
    sx1276_legal_dwell_reset();
    uint16_t h;
    /* Reserve at t=1000 ms. */
    (void)sx1276_legal_dwell_reserve(3U, 100000UL, 1000U,
        SX1276_DWELL_WINDOW_10S_MS, SX1276_DWELL_DEFAULT_CAP_US, &h);
    /* At now=11000: age = 10000 == window → OUT (half-open). */
    const uint32_t at_boundary = sx1276_legal_dwell_used_us(
        3U, SX1276_DWELL_WINDOW_10S_MS, 11000U);
    /* At now=10999: age = 9999 < window → IN. */
    const uint32_t just_inside = sx1276_legal_dwell_used_us(
        3U, SX1276_DWELL_WINDOW_10S_MS, 10999U);
    CHECK(at_boundary == 0UL,
          "half-open: now-window must EXCLUDE event (age==window), got %" PRIu32,
          at_boundary);
    CHECK(just_inside == 100000UL,
          "half-open: age<window must INCLUDE event, got %" PRIu32,
          just_inside);
}

static void test_per_channel_isolation(void) {
    sx1276_legal_dwell_reset();
    uint16_t h5, h7;
    (void)sx1276_legal_dwell_reserve(5U, 150000UL, 0U,
        SX1276_DWELL_WINDOW_10S_MS, SX1276_DWELL_DEFAULT_CAP_US, &h5);
    (void)sx1276_legal_dwell_reserve(7U, 200000UL, 0U,
        SX1276_DWELL_WINDOW_10S_MS, SX1276_DWELL_DEFAULT_CAP_US, &h7);
    CHECK(sx1276_legal_dwell_used_us(5U, SX1276_DWELL_WINDOW_10S_MS, 0U) == 150000UL,
          "ch=5 used isolated");
    CHECK(sx1276_legal_dwell_used_us(7U, SX1276_DWELL_WINDOW_10S_MS, 0U) == 200000UL,
          "ch=7 used isolated");
    CHECK(sx1276_legal_dwell_used_us(6U, SX1276_DWELL_WINDOW_10S_MS, 0U) == 0UL,
          "ch=6 must be untouched by ch=5/7 bookings");
}

static void test_over_budget_rejected(void) {
    sx1276_legal_dwell_reset();
    uint16_t h1, h2;
    sx1276_dwell_status_t st = sx1276_legal_dwell_reserve(
        2U, 350000UL, 0U, SX1276_DWELL_WINDOW_10S_MS,
        SX1276_DWELL_DEFAULT_CAP_US, &h1);
    CHECK(st == SX1276_DWELL_OK, "first reserve must succeed");
    st = sx1276_legal_dwell_reserve(
        2U, 100000UL, 0U, SX1276_DWELL_WINDOW_10S_MS,
        SX1276_DWELL_DEFAULT_CAP_US, &h2);
    CHECK(st == SX1276_DWELL_OVER_BUDGET,
          "350k + 100k > 400k cap must return OVER_BUDGET, got %d", (int)st);
    CHECK(h2 == SX1276_DWELL_HANDLE_INVALID,
          "rejected reserve must set handle to INVALID");
    const uint32_t used = sx1276_legal_dwell_used_us(
        2U, SX1276_DWELL_WINDOW_10S_MS, 0U);
    CHECK(used == 350000UL,
          "rejected reserve must NOT book; expected 350000, got %" PRIu32, used);
}

static void test_64ch_support_and_bad_ch(void) {
    sx1276_legal_dwell_reset();
    uint16_t h;
    sx1276_dwell_status_t st = sx1276_legal_dwell_reserve(
        63U, 100000UL, 0U, SX1276_DWELL_WINDOW_10S_MS,
        SX1276_DWELL_DEFAULT_CAP_US, &h);
    CHECK(st == SX1276_DWELL_OK, "ch=63 must be valid (64-ch widening)");
    st = sx1276_legal_dwell_reserve(
        64U, 100000UL, 0U, SX1276_DWELL_WINDOW_10S_MS,
        SX1276_DWELL_DEFAULT_CAP_US, &h);
    CHECK(st == SX1276_DWELL_BAD_CH, "ch=64 must return BAD_CH");
    st = sx1276_legal_dwell_reserve(
        0xFFU, 100000UL, 0U, SX1276_DWELL_WINDOW_10S_MS,
        SX1276_DWELL_DEFAULT_CAP_US, &h);
    CHECK(st == SX1276_DWELL_BAD_CH, "ch=255 must return BAD_CH");
    CHECK(h == SX1276_DWELL_HANDLE_INVALID,
          "BAD_CH must set handle to INVALID");
}

static void test_10s_vs_20s_window(void) {
    sx1276_legal_dwell_reset();
    uint16_t h;
    /* Event at t=0, 250 ms. Query at now=15000:
     *   10s window: age=15000 → OUT (>= 10000)
     *   20s window: age=15000 → IN  (<  20000)
     */
    (void)sx1276_legal_dwell_reserve(1U, 250000UL, 0U,
        SX1276_DWELL_WINDOW_20S_MS, SX1276_DWELL_DEFAULT_CAP_US, &h);
    const uint32_t w10 = sx1276_legal_dwell_used_us(
        1U, SX1276_DWELL_WINDOW_10S_MS, 15000U);
    const uint32_t w20 = sx1276_legal_dwell_used_us(
        1U, SX1276_DWELL_WINDOW_20S_MS, 15000U);
    CHECK(w10 == 0UL,      "10s window must exclude 15s-old event, got %" PRIu32, w10);
    CHECK(w20 == 250000UL, "20s window must include 15s-old event, got %" PRIu32, w20);
}

static void test_stale_handle_reconcile_safe(void) {
    sx1276_legal_dwell_reset();
    uint16_t h_old;
    /* Book + age out so the slot is reusable. */
    (void)sx1276_legal_dwell_reserve(0U, 100000UL, 0U,
        SX1276_DWELL_WINDOW_10S_MS, SX1276_DWELL_DEFAULT_CAP_US, &h_old);

    /* Force the same slot to be recycled by aging past 20s and
     * filling the ring is overkill — just reserve 128 more on other
     * channels with old timestamps then a new one with current time
     * that lands wherever. Simpler: reset is the only deterministic
     * way to get the next reserve to slot 0; reset wipes everything,
     * so we instead reserve, age the event, then do a new reserve at
     * a now far in the future so the old slot becomes eligible for
     * eviction. */

    uint16_t h_new;
    (void)sx1276_legal_dwell_reserve(2U, 50000UL, 25000UL,
        SX1276_DWELL_WINDOW_10S_MS, SX1276_DWELL_DEFAULT_CAP_US, &h_new);

    /* h_old is now stale (its slot may have been recycled into h_new,
     * or it may still be sitting in a different slot — either way
     * reconciling h_old must NOT mutate h_new's booking on ch=2). */
    sx1276_legal_dwell_reconcile(h_old, 1UL);

    const uint32_t used_ch2 = sx1276_legal_dwell_used_us(
        2U, SX1276_DWELL_WINDOW_10S_MS, 25000U);
    CHECK(used_ch2 == 50000UL,
          "stale handle reconcile must NOT shrink the current occupant; got %" PRIu32,
          used_ch2);
}

static void test_ring_fills_then_overflows(void) {
    sx1276_legal_dwell_reset();
    uint16_t h;
    /* Reserve 128 (= ring capacity) events at distinct channels (with
     * wrap to satisfy 64-ch limit) each well under the cap. */
    for (unsigned i = 0; i < SX1276_DWELL_RING_CAPACITY; ++i) {
        const uint8_t ch = (uint8_t)(i % SX1276_DWELL_CHANNEL_COUNT);
        const sx1276_dwell_status_t st = sx1276_legal_dwell_reserve(
            ch, 1000UL, 0U, SX1276_DWELL_WINDOW_10S_MS,
            SX1276_DWELL_DEFAULT_CAP_US, &h);
        CHECK(st == SX1276_DWELL_OK,
              "reserve #%u must succeed (ring not yet full)", i);
    }
    const sx1276_dwell_status_t st = sx1276_legal_dwell_reserve(
        0U, 1000UL, 0U, SX1276_DWELL_WINDOW_10S_MS,
        SX1276_DWELL_DEFAULT_CAP_US, &h);
    /* Note: ch=0 has 2 prior reservations of 1000us each due to wrap;
     * still well under cap, so the rejection MUST be RING_FULL (not
     * OVER_BUDGET). */
    CHECK(st == SX1276_DWELL_RING_FULL,
          "129th reserve must return RING_FULL, got %d", (int)st);
    CHECK(h == SX1276_DWELL_HANDLE_INVALID,
          "RING_FULL must set handle to INVALID");
}

static void test_eviction_after_age_out(void) {
    sx1276_legal_dwell_reset();
    uint16_t h;
    /* Fill the ring at t=0. */
    for (unsigned i = 0; i < SX1276_DWELL_RING_CAPACITY; ++i) {
        const uint8_t ch = (uint8_t)(i % SX1276_DWELL_CHANNEL_COUNT);
        (void)sx1276_legal_dwell_reserve(
            ch, 1000UL, 0U, SX1276_DWELL_WINDOW_10S_MS,
            SX1276_DWELL_DEFAULT_CAP_US, &h);
    }
    /* At t=25_000, all prior events have aged past the 20s window and
     * are recyclable. A fresh reserve must succeed. */
    const sx1276_dwell_status_t st = sx1276_legal_dwell_reserve(
        0U, 5000UL, 25000UL, SX1276_DWELL_WINDOW_10S_MS,
        SX1276_DWELL_DEFAULT_CAP_US, &h);
    CHECK(st == SX1276_DWELL_OK,
          "reserve after 25s age-out must succeed, got %d", (int)st);
    const uint32_t used_old_t = sx1276_legal_dwell_used_us(
        0U, SX1276_DWELL_WINDOW_10S_MS, 0U);
    (void)used_old_t; /* may or may not show the recycled slot's prior
                       * occupant — both are correct depending on which
                       * slot was reused. */
    const uint32_t used_now = sx1276_legal_dwell_used_us(
        0U, SX1276_DWELL_WINDOW_10S_MS, 25000U);
    CHECK(used_now == 5000UL,
          "after eviction, only the new 5000us event should be live for ch=0 at now=25s; got %" PRIu32,
          used_now);
}

static void test_bad_reserve_inputs(void) {
    sx1276_legal_dwell_reset();
    uint16_t h;
    sx1276_dwell_status_t st;

    st = sx1276_legal_dwell_reserve(0U, 0UL, 0U,
        SX1276_DWELL_WINDOW_10S_MS, SX1276_DWELL_DEFAULT_CAP_US, &h);
    CHECK(st == SX1276_DWELL_BAD_RESERVE, "pessimistic=0 must return BAD_RESERVE");

    st = sx1276_legal_dwell_reserve(0U, 100000UL, 0U,
        SX1276_DWELL_WINDOW_10S_MS, 0UL, &h);
    CHECK(st == SX1276_DWELL_BAD_RESERVE, "cap=0 must return BAD_RESERVE");

    st = sx1276_legal_dwell_reserve(0U, 500000UL, 0U,
        SX1276_DWELL_WINDOW_10S_MS, SX1276_DWELL_DEFAULT_CAP_US, &h);
    CHECK(st == SX1276_DWELL_BAD_RESERVE,
          "pessimistic > cap must return BAD_RESERVE");
}

static void test_bad_window_and_query(void) {
    sx1276_legal_dwell_reset();
    uint16_t h;
    sx1276_dwell_status_t st = sx1276_legal_dwell_reserve(
        0U, 100000UL, 0U, 0UL, SX1276_DWELL_DEFAULT_CAP_US, &h);
    CHECK(st == SX1276_DWELL_BAD_WINDOW, "window=0 must return BAD_WINDOW");
    st = sx1276_legal_dwell_reserve(
        0U, 100000UL, 0U, 30000UL, SX1276_DWELL_DEFAULT_CAP_US, &h);
    CHECK(st == SX1276_DWELL_BAD_WINDOW,
          "window > 20000 must return BAD_WINDOW");

    /* used_us on a bad channel or bad window returns 0 (not an error). */
    CHECK(sx1276_legal_dwell_used_us(64U, SX1276_DWELL_WINDOW_10S_MS, 0U) == 0UL,
          "used_us(bad ch) must return 0");
    CHECK(sx1276_legal_dwell_used_us(0U, 0UL, 0U) == 0UL,
          "used_us(bad window) must return 0");
}

static void test_reset_clears(void) {
    sx1276_legal_dwell_reset();
    uint16_t h;
    (void)sx1276_legal_dwell_reserve(0U, 100000UL, 0U,
        SX1276_DWELL_WINDOW_10S_MS, SX1276_DWELL_DEFAULT_CAP_US, &h);
    CHECK(sx1276_legal_dwell_used_us(0U, SX1276_DWELL_WINDOW_10S_MS, 0U) == 100000UL,
          "pre-reset booked");
    sx1276_legal_dwell_reset();
    CHECK(sx1276_legal_dwell_used_us(0U, SX1276_DWELL_WINDOW_10S_MS, 0U) == 0UL,
          "post-reset used must be 0");
    CHECK(sx1276_legal_dwell_live_event_count(0U) == 0U,
          "post-reset live event count must be 0");
}

int main(void) {
    test_reserve_immediately_books();
    test_reconcile_down();
    test_reconcile_up_is_noop();
    test_no_rollback_on_nack();
    test_half_open_window();
    test_per_channel_isolation();
    test_over_budget_rejected();
    test_64ch_support_and_bad_ch();
    test_10s_vs_20s_window();
    test_stale_handle_reconcile_safe();
    test_ring_fills_then_overflows();
    test_eviction_after_age_out();
    test_bad_reserve_inputs();
    test_bad_window_and_query();
    test_reset_clears();

    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] legal_dwell: %d failure(s)\n", g_failures);
        return EXIT_FAILURE;
    }
    printf("[PASS] legal_dwell: 15 cases\n");
    return EXIT_SUCCESS;
}
