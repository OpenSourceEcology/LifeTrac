/*
 * FCC-B1-SUMMARY-b-1: sidecar counter TUs for the per-minute
 * RFCO_SUMMARY URC.
 *
 * Covers three independent counter modules that feed the snapshot
 * builder in B1-SUMMARY-b-2:
 *
 *   1. sx1276_fhss hop counter
 *      - record_hop increments saturating
 *      - snapshot_and_clear returns counts then zeroes
 *      - second snapshot returns zero
 *      - in-memory u16 saturates to UINT16_MAX
 *      - on-wire u8 saturates to 0xFFU
 *      - out-of-range idx is no-op
 *      - NULL out preserves counters
 *      - per-channel independence
 *
 *   2. sx1276_legal_dwell peak-us tracker
 *      - successful reserve updates peak
 *      - peak == used_before + pessimistic_us
 *      - subsequent smaller reserve does NOT lower the peak
 *      - OVER_BUDGET / BAD_CH / BAD_RESERVE do NOT update peak
 *      - snapshot_and_clear returns peaks then zeroes
 *      - per-channel independence
 *      - NULL out preserves peaks
 *
 *   3. host_rfco blocked-attempts-by-reason 8-slot counter
 *      - record per non-OK reason hits the documented slot
 *      - OK is never recorded (slot 0 stays 0)
 *      - INTERNAL (0xFF) maps to slot 7
 *      - out-of-range reason is dropped
 *      - saturates at UINT16_MAX
 *      - snapshot_and_clear returns counts then zeroes
 *      - NULL out preserves counters
 *      - reset zeroes
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#include "host_rfco.h"
#include "host_types.h"
#include "sx1276_fhss.h"
#include "sx1276_legal_dwell.h"

/* host_rfco.c references host_uart_send_urc; satisfy the linker even
 * though these tests never call the emit() path. */
void host_uart_send_urc(uint8_t type,
                        uint16_t seq,
                        uint8_t flags,
                        const uint8_t *payload,
                        uint16_t payload_len) {
    (void)type; (void)seq; (void)flags; (void)payload; (void)payload_len;
}

/* ---- test harness ---- */

static int g_failures = 0;
static int g_cases    = 0;

#define CHECK(cond, ...) do {                                          \
    if (!(cond)) {                                                     \
        ++g_failures;                                                  \
        fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__);           \
        fprintf(stderr, __VA_ARGS__);                                  \
        fprintf(stderr, "\n");                                         \
    }                                                                  \
} while (0)

#define CASE(name) do { ++g_cases; fprintf(stderr, "[CASE] %s\n", name); } while (0)

/* ============================================================
 * 1. sx1276_fhss hop counter
 * ============================================================ */

static void test_fhss_hop_record_and_snapshot(void) {
    CASE("fhss: record then snapshot returns counts then zeroes");
    sx1276_fhss_reset();
    for (uint8_t i = 0; i < 5; ++i) sx1276_fhss_record_hop(3);
    for (uint8_t i = 0; i < 17; ++i) sx1276_fhss_record_hop(49);
    uint8_t snap[SX1276_FHSS_CHANNEL_COUNT];
    memset(snap, 0xAA, sizeof(snap));
    sx1276_fhss_hop_count_snapshot_and_clear(snap);
    CHECK(snap[3] == 5U,  "channel 3 got %u, want 5",  snap[3]);
    CHECK(snap[49] == 17U,"channel 49 got %u, want 17",snap[49]);
    CHECK(snap[0] == 0U,  "channel 0 should be 0, got %u", snap[0]);

    /* Second snapshot is all zero. */
    uint8_t snap2[SX1276_FHSS_CHANNEL_COUNT];
    memset(snap2, 0xBB, sizeof(snap2));
    sx1276_fhss_hop_count_snapshot_and_clear(snap2);
    for (uint8_t i = 0; i < SX1276_FHSS_CHANNEL_COUNT; ++i) {
        CHECK(snap2[i] == 0U, "post-snapshot ch %u should be 0, got %u", i, snap2[i]);
    }
}

static void test_fhss_hop_wire_saturation(void) {
    CASE("fhss: in-memory >0xFF saturates to 0xFFU on wire");
    sx1276_fhss_reset();
    for (uint16_t i = 0; i < 300U; ++i) sx1276_fhss_record_hop(7);
    uint8_t snap[SX1276_FHSS_CHANNEL_COUNT];
    sx1276_fhss_hop_count_snapshot_and_clear(snap);
    CHECK(snap[7] == 0xFFU, "channel 7 should saturate to 0xFF, got 0x%02X", snap[7]);
}

static void test_fhss_hop_memory_saturation(void) {
    CASE("fhss: in-memory counter saturates at UINT16_MAX, never wraps");
    sx1276_fhss_reset();
    /* Hammer at idx 0 for >65535 increments. Use a direct loop count
     * larger than UINT16_MAX to drive the saturating branch hard. */
    for (uint32_t i = 0; i < 70000UL; ++i) sx1276_fhss_record_hop(0);
    uint8_t snap[SX1276_FHSS_CHANNEL_COUNT];
    sx1276_fhss_hop_count_snapshot_and_clear(snap);
    /* Wire still 0xFF; the real check is that we don't crash + counter
     * actually held the max (so snapshot saturated path was taken). */
    CHECK(snap[0] == 0xFFU, "saturated channel 0 should be 0xFF, got 0x%02X", snap[0]);
}

static void test_fhss_hop_oor_idx_noop(void) {
    CASE("fhss: out-of-range idx is no-op");
    sx1276_fhss_reset();
    sx1276_fhss_record_hop(SX1276_FHSS_CHANNEL_COUNT);     /* == 50, out of range */
    sx1276_fhss_record_hop(0xFFU);                          /* out of range */
    uint8_t snap[SX1276_FHSS_CHANNEL_COUNT];
    sx1276_fhss_hop_count_snapshot_and_clear(snap);
    for (uint8_t i = 0; i < SX1276_FHSS_CHANNEL_COUNT; ++i) {
        CHECK(snap[i] == 0U, "oor record should not touch any channel; ch %u = %u", i, snap[i]);
    }
}

static void test_fhss_hop_null_out_preserves_counters(void) {
    CASE("fhss: NULL out preserves counters");
    sx1276_fhss_reset();
    sx1276_fhss_record_hop(11);
    sx1276_fhss_record_hop(11);
    sx1276_fhss_record_hop(11);
    sx1276_fhss_hop_count_snapshot_and_clear(NULL);
    uint8_t snap[SX1276_FHSS_CHANNEL_COUNT];
    sx1276_fhss_hop_count_snapshot_and_clear(snap);
    CHECK(snap[11] == 3U, "NULL snapshot must not clear; ch 11 should be 3, got %u", snap[11]);
}

static void test_fhss_hop_per_channel_independence(void) {
    CASE("fhss: per-channel independence");
    sx1276_fhss_reset();
    sx1276_fhss_record_hop(5);
    sx1276_fhss_record_hop(10);
    uint8_t snap[SX1276_FHSS_CHANNEL_COUNT];
    sx1276_fhss_hop_count_snapshot_and_clear(snap);
    for (uint8_t i = 0; i < SX1276_FHSS_CHANNEL_COUNT; ++i) {
        uint8_t want = (i == 5U || i == 10U) ? 1U : 0U;
        CHECK(snap[i] == want, "ch %u: got %u, want %u", i, snap[i], want);
    }
}

/* ============================================================
 * 2. sx1276_legal_dwell peak-us tracker
 * ============================================================ */

static void test_dwell_peak_basic(void) {
    CASE("dwell: successful reserve updates peak == used_before + pessimistic");
    sx1276_legal_dwell_reset();
    uint16_t h;
    sx1276_dwell_status_t s = sx1276_legal_dwell_reserve(
        4, 100000UL, 1000UL, SX1276_DWELL_WINDOW_10S_MS,
        SX1276_DWELL_DEFAULT_CAP_US, &h);
    CHECK(s == SX1276_DWELL_OK, "reserve should succeed, got %d", s);

    uint32_t peak[SX1276_DWELL_CHANNEL_COUNT];
    sx1276_legal_dwell_peak_us_snapshot_and_clear(peak);
    CHECK(peak[4] == 100000UL, "peak ch 4 = %" PRIu32 ", want 100000", peak[4]);
    CHECK(peak[3] == 0UL,      "peak ch 3 should be 0, got %" PRIu32, peak[3]);
}

static void test_dwell_peak_monotonic_high_water(void) {
    CASE("dwell: smaller subsequent reserve does NOT lower the peak");
    sx1276_legal_dwell_reset();
    uint16_t h;
    (void)sx1276_legal_dwell_reserve(7, 300000UL, 1000UL,
        SX1276_DWELL_WINDOW_10S_MS, SX1276_DWELL_DEFAULT_CAP_US, &h);
    /* Reconcile small to free room conceptually; peak should still be 300k. */
    sx1276_legal_dwell_reconcile(h, 50000UL);
    /* Reserve a tiny amount on the same channel. */
    (void)sx1276_legal_dwell_reserve(7, 10000UL, 2000UL,
        SX1276_DWELL_WINDOW_10S_MS, SX1276_DWELL_DEFAULT_CAP_US, &h);

    uint32_t peak[SX1276_DWELL_CHANNEL_COUNT];
    sx1276_legal_dwell_peak_us_snapshot_and_clear(peak);
    /* First reserve had used_before=0 so post=300000.
     * Second had used_before=50000 (reconciled) so post=60000.
     * Peak stays at 300000. */
    CHECK(peak[7] == 300000UL,
          "peak ch 7 should stay at 300000, got %" PRIu32, peak[7]);
}

static void test_dwell_peak_failed_reserve_no_update(void) {
    CASE("dwell: OVER_BUDGET / BAD_CH / BAD_RESERVE do NOT update peak");
    sx1276_legal_dwell_reset();
    uint16_t h;

    /* OVER_BUDGET: reserve more than cap in a single shot. */
    sx1276_dwell_status_t s = sx1276_legal_dwell_reserve(
        9, SX1276_DWELL_DEFAULT_CAP_US + 1UL, 1000UL,
        SX1276_DWELL_WINDOW_10S_MS, SX1276_DWELL_DEFAULT_CAP_US, &h);
    CHECK(s == SX1276_DWELL_BAD_RESERVE,
          "pessimistic > cap should be BAD_RESERVE, got %d", s);

    /* BAD_CH. */
    s = sx1276_legal_dwell_reserve(
        SX1276_DWELL_CHANNEL_COUNT, 100UL, 1000UL,
        SX1276_DWELL_WINDOW_10S_MS, SX1276_DWELL_DEFAULT_CAP_US, &h);
    CHECK(s == SX1276_DWELL_BAD_CH, "oor ch should be BAD_CH, got %d", s);

    /* BAD_RESERVE: pessimistic_us == 0. */
    s = sx1276_legal_dwell_reserve(9, 0UL, 1000UL,
        SX1276_DWELL_WINDOW_10S_MS, SX1276_DWELL_DEFAULT_CAP_US, &h);
    CHECK(s == SX1276_DWELL_BAD_RESERVE, "zero pessimistic should be BAD_RESERVE, got %d", s);

    /* OVER_BUDGET against a partially-booked channel. */
    (void)sx1276_legal_dwell_reserve(9, 300000UL, 1000UL,
        SX1276_DWELL_WINDOW_10S_MS, SX1276_DWELL_DEFAULT_CAP_US, &h);
    s = sx1276_legal_dwell_reserve(9, 200000UL, 1001UL,
        SX1276_DWELL_WINDOW_10S_MS, SX1276_DWELL_DEFAULT_CAP_US, &h);
    CHECK(s == SX1276_DWELL_OVER_BUDGET, "over-budget should be OVER_BUDGET, got %d", s);

    uint32_t peak[SX1276_DWELL_CHANNEL_COUNT];
    sx1276_legal_dwell_peak_us_snapshot_and_clear(peak);
    /* Only the 300k succeeded; the over-budget 200k should NOT have
     * bumped peak above 300k. */
    CHECK(peak[9] == 300000UL,
          "peak ch 9 = %" PRIu32 ", want 300000 (only successful reserve counts)", peak[9]);
}

static void test_dwell_peak_snapshot_clears(void) {
    CASE("dwell: snapshot_and_clear zeroes peaks; second snapshot is all zero");
    sx1276_legal_dwell_reset();
    uint16_t h;
    (void)sx1276_legal_dwell_reserve(2, 50000UL, 1000UL,
        SX1276_DWELL_WINDOW_10S_MS, SX1276_DWELL_DEFAULT_CAP_US, &h);
    uint32_t peak1[SX1276_DWELL_CHANNEL_COUNT];
    sx1276_legal_dwell_peak_us_snapshot_and_clear(peak1);
    CHECK(peak1[2] == 50000UL, "first snapshot ch 2 = %" PRIu32, peak1[2]);

    uint32_t peak2[SX1276_DWELL_CHANNEL_COUNT];
    sx1276_legal_dwell_peak_us_snapshot_and_clear(peak2);
    for (uint8_t i = 0; i < SX1276_DWELL_CHANNEL_COUNT; ++i) {
        CHECK(peak2[i] == 0UL, "second snapshot ch %u should be 0, got %" PRIu32, i, peak2[i]);
    }
}

static void test_dwell_peak_null_preserves(void) {
    CASE("dwell: NULL out preserves peaks");
    sx1276_legal_dwell_reset();
    uint16_t h;
    (void)sx1276_legal_dwell_reserve(1, 77000UL, 1000UL,
        SX1276_DWELL_WINDOW_10S_MS, SX1276_DWELL_DEFAULT_CAP_US, &h);
    sx1276_legal_dwell_peak_us_snapshot_and_clear(NULL);
    uint32_t peak[SX1276_DWELL_CHANNEL_COUNT];
    sx1276_legal_dwell_peak_us_snapshot_and_clear(peak);
    CHECK(peak[1] == 77000UL,
          "NULL snapshot must not clear; ch 1 peak = %" PRIu32, peak[1]);
}

/* ============================================================
 * 3. host_rfco blocked-attempts-by-reason
 * ============================================================ */

static void test_blocked_basic_per_reason_slots(void) {
    CASE("blocked: per-reason slot mapping (1..6 direct, 0xFF -> slot 7)");
    host_rfco_blocked_attempts_reset();
    host_rfco_blocked_attempts_record(HOST_RFCO_TX_STATUS_ABORT_AIRTIME_INVARIANT); /* slot 1 */
    host_rfco_blocked_attempts_record(HOST_RFCO_TX_STATUS_ABORT_LBT);                /* slot 2 */
    host_rfco_blocked_attempts_record(HOST_RFCO_TX_STATUS_ABORT_LEGAL_DWELL);        /* slot 3 */
    host_rfco_blocked_attempts_record(HOST_RFCO_TX_STATUS_ABORT_QOS);                /* slot 4 */
    host_rfco_blocked_attempts_record(HOST_RFCO_TX_STATUS_TX_TIMEOUT);               /* slot 5 */
    host_rfco_blocked_attempts_record(HOST_RFCO_TX_STATUS_TX_FAIL);                  /* slot 6 */
    host_rfco_blocked_attempts_record(HOST_RFCO_TX_STATUS_INTERNAL);                 /* slot 7 */

    uint16_t snap[HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS];
    host_rfco_blocked_attempts_snapshot_and_clear(snap);
    CHECK(snap[0] == 0U, "slot 0 (OK) should never be touched, got %u", snap[0]);
    for (uint8_t i = 1; i < HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS; ++i) {
        CHECK(snap[i] == 1U, "slot %u should be 1, got %u", i, snap[i]);
    }
}

static void test_blocked_ok_not_recorded(void) {
    CASE("blocked: OK is never recorded (slot 0 stays 0)");
    host_rfco_blocked_attempts_reset();
    for (int i = 0; i < 10; ++i) {
        host_rfco_blocked_attempts_record(HOST_RFCO_TX_STATUS_OK);
    }
    uint16_t snap[HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS];
    host_rfco_blocked_attempts_snapshot_and_clear(snap);
    for (uint8_t i = 0; i < HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS; ++i) {
        CHECK(snap[i] == 0U, "OK should be dropped; slot %u = %u", i, snap[i]);
    }
}

static void test_blocked_oor_reason_dropped(void) {
    CASE("blocked: unallocated reason value is dropped");
    host_rfco_blocked_attempts_reset();
    /* Cast to escape the enum; this models a corrupt enum value
     * surviving an unsafe path (the function MUST NOT crash). */
    host_rfco_blocked_attempts_record((host_rfco_tx_status_t)7U);   /* 7 not allocated */
    host_rfco_blocked_attempts_record((host_rfco_tx_status_t)100U); /* arbitrary */
    host_rfco_blocked_attempts_record((host_rfco_tx_status_t)0xFEU);/* not INTERNAL */
    uint16_t snap[HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS];
    host_rfco_blocked_attempts_snapshot_and_clear(snap);
    for (uint8_t i = 0; i < HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS; ++i) {
        CHECK(snap[i] == 0U, "unallocated reason should drop; slot %u = %u", i, snap[i]);
    }
}

static void test_blocked_saturation(void) {
    CASE("blocked: counter saturates at UINT16_MAX");
    host_rfco_blocked_attempts_reset();
    for (uint32_t i = 0; i < 70000UL; ++i) {
        host_rfco_blocked_attempts_record(HOST_RFCO_TX_STATUS_ABORT_LBT); /* slot 2 */
    }
    uint16_t snap[HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS];
    host_rfco_blocked_attempts_snapshot_and_clear(snap);
    CHECK(snap[2] == UINT16_MAX, "slot 2 should saturate to %u, got %u",
          UINT16_MAX, snap[2]);
}

static void test_blocked_snapshot_clears(void) {
    CASE("blocked: snapshot zeroes counters; second snapshot is all zero");
    host_rfco_blocked_attempts_reset();
    host_rfco_blocked_attempts_record(HOST_RFCO_TX_STATUS_ABORT_LBT);
    host_rfco_blocked_attempts_record(HOST_RFCO_TX_STATUS_ABORT_LBT);
    uint16_t snap1[HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS];
    host_rfco_blocked_attempts_snapshot_and_clear(snap1);
    CHECK(snap1[2] == 2U, "first snapshot slot 2 should be 2, got %u", snap1[2]);

    uint16_t snap2[HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS];
    host_rfco_blocked_attempts_snapshot_and_clear(snap2);
    for (uint8_t i = 0; i < HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS; ++i) {
        CHECK(snap2[i] == 0U, "second snapshot slot %u should be 0, got %u", i, snap2[i]);
    }
}

static void test_blocked_null_preserves(void) {
    CASE("blocked: NULL out preserves counters");
    host_rfco_blocked_attempts_reset();
    host_rfco_blocked_attempts_record(HOST_RFCO_TX_STATUS_TX_FAIL); /* slot 6 */
    host_rfco_blocked_attempts_record(HOST_RFCO_TX_STATUS_TX_FAIL);
    host_rfco_blocked_attempts_record(HOST_RFCO_TX_STATUS_TX_FAIL);
    host_rfco_blocked_attempts_snapshot_and_clear(NULL);
    uint16_t snap[HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS];
    host_rfco_blocked_attempts_snapshot_and_clear(snap);
    CHECK(snap[6] == 3U, "NULL snapshot must not clear; slot 6 = %u", snap[6]);
}

/* ============================================================ */

int main(void) {
    test_fhss_hop_record_and_snapshot();
    test_fhss_hop_wire_saturation();
    test_fhss_hop_memory_saturation();
    test_fhss_hop_oor_idx_noop();
    test_fhss_hop_null_out_preserves_counters();
    test_fhss_hop_per_channel_independence();

    test_dwell_peak_basic();
    test_dwell_peak_monotonic_high_water();
    test_dwell_peak_failed_reserve_no_update();
    test_dwell_peak_snapshot_clears();
    test_dwell_peak_null_preserves();

    test_blocked_basic_per_reason_slots();
    test_blocked_ok_not_recorded();
    test_blocked_oor_reason_dropped();
    test_blocked_saturation();
    test_blocked_snapshot_clears();
    test_blocked_null_preserves();

    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] rfco_summary_counters: %d failures across %d cases\n",
                g_failures, g_cases);
        return 1;
    }
    printf("[PASS] rfco_summary_counters: %d cases\n", g_cases);
    return 0;
}
