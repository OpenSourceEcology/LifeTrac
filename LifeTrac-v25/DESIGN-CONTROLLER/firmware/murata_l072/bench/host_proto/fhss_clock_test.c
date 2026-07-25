/*
 * v25.0.7 slot-clock unit test (host-cc). Pins the time→slot math the
 * TX slot gate and RX boundary follower both derive their channel
 * decisions from.
 *
 * Coverage:
 *   1. Reset/valid lifecycle.
 *   2. abs_slot arithmetic at, inside, and across slot boundaries.
 *   3. in_slot_ms at boundary / mid-slot / last-ms.
 *   4. u32 ms tick wrap (anchor near UINT32_MAX, now after wrap).
 *   5. epoch/hop decomposition round-trip incl. epoch boundaries.
 *   6. Re-anchor replaces phase (RX re-anchors on every accepted
 *      header — later anchor must win).
 *   7. Grid geometry constants (SLOT_MS divides the epoch into
 *      exactly CHANNEL_COUNT slots worth of dwell bookkeeping).
 */

#include <stdio.h>
#include <stdint.h>

#include "sx1276_fhss_clock.h"

static int g_failures = 0;

#define CHECK(cond, ...) do {                                          \
    if (!(cond)) {                                                     \
        ++g_failures;                                                  \
        fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__);           \
        fprintf(stderr, __VA_ARGS__);                                  \
        fputc('\n', stderr);                                           \
    }                                                                  \
} while (0)

static void test_lifecycle(void) {
    sx1276_fhss_clock_reset();
    CHECK(sx1276_fhss_clock_valid() == 0U, "(1) reset -> invalid");
    sx1276_fhss_clock_anchor(1000U, 7U);
    CHECK(sx1276_fhss_clock_valid() == 1U, "(1) anchor -> valid");
    sx1276_fhss_clock_reset();
    CHECK(sx1276_fhss_clock_valid() == 0U, "(1) re-reset -> invalid");
}

static void test_abs_slot(void) {
    sx1276_fhss_clock_anchor(1000U, 7U);
    CHECK(sx1276_fhss_clock_abs_slot(1000U) == 7U,
          "(2) at boundary: anchor slot");
    CHECK(sx1276_fhss_clock_abs_slot(1000U + SX1276_FHSS_SLOT_MS - 1U) == 7U,
          "(2) last ms of slot: still anchor slot");
    CHECK(sx1276_fhss_clock_abs_slot(1000U + SX1276_FHSS_SLOT_MS) == 8U,
          "(2) boundary crossing: next slot");
    CHECK(sx1276_fhss_clock_abs_slot(1000U + 10U * SX1276_FHSS_SLOT_MS) == 17U,
          "(2) ten slots later");
}

static void test_in_slot_ms(void) {
    sx1276_fhss_clock_anchor(5000U, 0U);
    CHECK(sx1276_fhss_clock_in_slot_ms(5000U) == 0U, "(3) boundary: 0");
    CHECK(sx1276_fhss_clock_in_slot_ms(5000U + 37U) == 37U, "(3) mid-slot");
    CHECK(sx1276_fhss_clock_in_slot_ms(5000U + SX1276_FHSS_SLOT_MS - 1U)
              == SX1276_FHSS_SLOT_MS - 1U,
          "(3) last ms");
    CHECK(sx1276_fhss_clock_in_slot_ms(5000U + SX1276_FHSS_SLOT_MS + 3U) == 3U,
          "(3) into next slot");
}

static void test_tick_wrap(void) {
    /* Anchor 100 ms before the u32 wrap; query 100 ms after it. The
     * unsigned subtraction idiom must yield 200 ms elapsed = 1 slot. */
    const uint32_t anchor = UINT32_MAX - 99U;      /* wraps in 100 ms */
    sx1276_fhss_clock_anchor(anchor, 41U);
    const uint32_t now = 100U;                     /* 200 ms elapsed */
    CHECK(sx1276_fhss_clock_abs_slot(now) == 42U,
          "(4) wrap-spanning elapsed -> exactly one slot advance");
    CHECK(sx1276_fhss_clock_in_slot_ms(now) == 0U,
          "(4) wrap-spanning boundary phase");
}

static void test_decompose(void) {
    CHECK(sx1276_fhss_clock_epoch_of(0U) == 0U, "(5) abs 0 epoch");
    CHECK(sx1276_fhss_clock_hop_of(0U) == 0U, "(5) abs 0 hop");
    const uint32_t abs1 = sx1276_fhss_clock_abs_of(3U, 49U);
    CHECK(sx1276_fhss_clock_epoch_of(abs1) == 3U, "(5) epoch round-trip");
    CHECK(sx1276_fhss_clock_hop_of(abs1) == 49U, "(5) hop round-trip");
    CHECK(sx1276_fhss_clock_epoch_of(abs1 + 1U) == 4U,
          "(5) epoch boundary: hop 49 -> next epoch");
    CHECK(sx1276_fhss_clock_hop_of(abs1 + 1U) == 0U,
          "(5) epoch boundary: hop wraps to 0");
}

static void test_reanchor(void) {
    sx1276_fhss_clock_anchor(1000U, 10U);
    /* Receiver decodes a later packet: sender's grid says slot 60
     * started at local 2050. The new anchor must win outright. */
    sx1276_fhss_clock_anchor(2050U, 60U);
    CHECK(sx1276_fhss_clock_abs_slot(2050U) == 60U, "(6) re-anchor slot");
    CHECK(sx1276_fhss_clock_in_slot_ms(2060U) == 10U, "(6) re-anchor phase");
}

static void test_geometry(void) {
    CHECK(SX1276_FHSS_SLOT_MS == 200U, "(7) grid is 200 ms");
    /* One epoch = CHANNEL_COUNT slots = 10 s at 200 ms — the legal-
     * dwell window (10 s) sees each channel at most twice, bounded by
     * one packet ToA each, far under the 400 ms cap. */
    CHECK((uint32_t)SX1276_FHSS_SLOT_MS * SX1276_FHSS_CHANNEL_COUNT == 10000U,
          "(7) epoch duration 10 s");
    CHECK(SX1276_FHSS_SLOT_TX_GUARD_US == 15000UL, "(7) TX guard 15 ms");
    /* Head-start + max-fit sanity: a full 215 B frame (~173 ms ToA)
     * keyed at +12 ms ends by ~185 ms — the 15 ms guard holds. */
    CHECK(SX1276_FHSS_SLOT_TX_HEADSTART_MS == 12U, "(7) TX head-start 12 ms");
}

int main(void) {
    test_lifecycle();
    test_abs_slot();
    test_in_slot_ms();
    test_tick_wrap();
    test_decompose();
    test_reanchor();
    test_geometry();

    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] fhss_clock: %d failure(s)\n", g_failures);
        return 1;
    }
    printf("[PASS] fhss_clock: 24 cases\n");
    return 0;
}
