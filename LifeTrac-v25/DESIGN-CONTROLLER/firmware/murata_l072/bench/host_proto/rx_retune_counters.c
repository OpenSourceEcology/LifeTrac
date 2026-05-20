/*
 * FCC-A6b-2-ii-γ-2: host test for radio/sx1276_rx_retune_counters.c.
 *
 * Mirror of bench/host_proto/rx_counters.c (β snap-policy counters)
 * adapted to the γ retune decision enum. Verifies the saturating
 * per-outcome counters that FCC-B1-SUMMARY will read for the retune
 * histogram. Cases:
 *   (1)  Initial state — all five counts are zero before any
 *        record().
 *   (2)  Each enum slot increments the right index in isolation.
 *   (3)  record() with an out-of-range cast value is a no-op.
 *   (4)  Snapshot getter is read-only — repeated calls return the
 *        same values.
 *   (5)  reset() zeros all five entries and leaves storage usable.
 *   (6)  NULL getter argument is a no-op (no crash, no mutation).
 *   (7)  No-narrow-type-wrap @ 4096 records.
 *   (8)  Interleaved records accumulate independently per slot.
 *   (9)  DIM constant equals the number of decision enum values.
 *   (10) Round-trip after reset behaves like a fresh module.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "sx1276_rx.h"
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

static void reset_and_snapshot(uint32_t out[SX1276_RX_RETUNE_DECISION_DIM]) {
    sx1276_rx_retune_counts_reset();
    sx1276_rx_retune_counts(out);
}

static void test_initial_state_all_zero(void) {
    sx1276_rx_retune_counts_reset();
    uint32_t counts[SX1276_RX_RETUNE_DECISION_DIM];
    sx1276_rx_retune_counts(counts);
    for (uint8_t i = 0U; i < SX1276_RX_RETUNE_DECISION_DIM; ++i) {
        CHECK(counts[i] == 0U,
              "post-reset count[%u] must be 0, got %" PRIu32, i, counts[i]);
    }
}

static void test_each_slot_increments_in_isolation(void) {
    const sx1276_rx_retune_decision_t all[] = {
        SX1276_RX_RETUNE_DO,
        SX1276_RX_RETUNE_SKIP_NOT_INIT,
        SX1276_RX_RETUNE_SKIP_BUSY,
        SX1276_RX_RETUNE_SKIP_TOO_SOON,
        SX1276_RX_RETUNE_DO_WRAP,
    };
    for (size_t k = 0; k < sizeof(all) / sizeof(all[0]); ++k) {
        uint32_t counts[SX1276_RX_RETUNE_DECISION_DIM];
        reset_and_snapshot(counts);
        sx1276_rx_retune_counter_record(all[k]);
        sx1276_rx_retune_counts(counts);
        for (uint8_t i = 0U; i < SX1276_RX_RETUNE_DECISION_DIM; ++i) {
            const uint32_t expected = (i == (uint8_t)all[k]) ? 1U : 0U;
            CHECK(counts[i] == expected,
                  "after record(%d): count[%u] expected %" PRIu32
                  " got %" PRIu32, (int)all[k], i, expected, counts[i]);
        }
    }
}

static void test_out_of_range_decision_is_noop(void) {
    sx1276_rx_retune_counts_reset();
    sx1276_rx_retune_counter_record((sx1276_rx_retune_decision_t)99);
    sx1276_rx_retune_counter_record((sx1276_rx_retune_decision_t)
                                    SX1276_RX_RETUNE_DECISION_DIM);
    uint32_t counts[SX1276_RX_RETUNE_DECISION_DIM];
    sx1276_rx_retune_counts(counts);
    for (uint8_t i = 0U; i < SX1276_RX_RETUNE_DECISION_DIM; ++i) {
        CHECK(counts[i] == 0U,
              "out-of-range record must not mutate count[%u], got %" PRIu32,
              i, counts[i]);
    }
}

static void test_snapshot_is_readonly(void) {
    sx1276_rx_retune_counts_reset();
    sx1276_rx_retune_counter_record(SX1276_RX_RETUNE_DO);
    sx1276_rx_retune_counter_record(SX1276_RX_RETUNE_DO);
    sx1276_rx_retune_counter_record(SX1276_RX_RETUNE_SKIP_BUSY);

    uint32_t a[SX1276_RX_RETUNE_DECISION_DIM];
    uint32_t b[SX1276_RX_RETUNE_DECISION_DIM];
    sx1276_rx_retune_counts(a);
    sx1276_rx_retune_counts(b);
    for (uint8_t i = 0U; i < SX1276_RX_RETUNE_DECISION_DIM; ++i) {
        CHECK(a[i] == b[i],
              "snapshot must be idempotent; a[%u]=%" PRIu32 " b[%u]=%" PRIu32,
              i, a[i], i, b[i]);
    }
    CHECK(a[SX1276_RX_RETUNE_DO] == 2U, "DO count must be 2");
    CHECK(a[SX1276_RX_RETUNE_SKIP_BUSY] == 1U, "SKIP_BUSY count must be 1");
}

static void test_reset_zeroes_then_usable(void) {
    sx1276_rx_retune_counts_reset();
    sx1276_rx_retune_counter_record(SX1276_RX_RETUNE_DO_WRAP);
    sx1276_rx_retune_counter_record(SX1276_RX_RETUNE_SKIP_TOO_SOON);
    sx1276_rx_retune_counts_reset();

    uint32_t counts[SX1276_RX_RETUNE_DECISION_DIM];
    sx1276_rx_retune_counts(counts);
    for (uint8_t i = 0U; i < SX1276_RX_RETUNE_DECISION_DIM; ++i) {
        CHECK(counts[i] == 0U,
              "post-reset count[%u] must be 0, got %" PRIu32, i, counts[i]);
    }
    sx1276_rx_retune_counter_record(SX1276_RX_RETUNE_DO);
    sx1276_rx_retune_counts(counts);
    CHECK(counts[SX1276_RX_RETUNE_DO] == 1U,
          "post-reset record must work; DO=%" PRIu32,
          counts[SX1276_RX_RETUNE_DO]);
}

static void test_null_getter_is_noop(void) {
    sx1276_rx_retune_counts_reset();
    sx1276_rx_retune_counter_record(SX1276_RX_RETUNE_DO);
    sx1276_rx_retune_counts(NULL);
    uint32_t counts[SX1276_RX_RETUNE_DECISION_DIM];
    sx1276_rx_retune_counts(counts);
    CHECK(counts[SX1276_RX_RETUNE_DO] == 1U,
          "NULL getter must not perturb storage; DO=%" PRIu32,
          counts[SX1276_RX_RETUNE_DO]);
}

static void test_saturating_increment(void) {
    sx1276_rx_retune_counts_reset();
    for (uint32_t k = 0U; k < 4096U; ++k) {
        sx1276_rx_retune_counter_record(SX1276_RX_RETUNE_DO);
    }
    uint32_t counts[SX1276_RX_RETUNE_DECISION_DIM];
    sx1276_rx_retune_counts(counts);
    CHECK(counts[SX1276_RX_RETUNE_DO] == 4096U,
          "4096 records must yield 4096 (no narrow-type wrap); got %" PRIu32,
          counts[SX1276_RX_RETUNE_DO]);
}

static void test_interleaved_records_accumulate(void) {
    sx1276_rx_retune_counts_reset();
    const sx1276_rx_retune_decision_t pattern[] = {
        SX1276_RX_RETUNE_DO,
        SX1276_RX_RETUNE_SKIP_TOO_SOON,
        SX1276_RX_RETUNE_DO,
        SX1276_RX_RETUNE_SKIP_BUSY,
        SX1276_RX_RETUNE_DO,
        SX1276_RX_RETUNE_SKIP_TOO_SOON,
        SX1276_RX_RETUNE_DO_WRAP,
        SX1276_RX_RETUNE_SKIP_NOT_INIT,
        SX1276_RX_RETUNE_SKIP_TOO_SOON,
        SX1276_RX_RETUNE_DO_WRAP,
    };
    for (size_t k = 0; k < sizeof(pattern) / sizeof(pattern[0]); ++k) {
        sx1276_rx_retune_counter_record(pattern[k]);
    }
    uint32_t counts[SX1276_RX_RETUNE_DECISION_DIM];
    sx1276_rx_retune_counts(counts);
    CHECK(counts[SX1276_RX_RETUNE_DO] == 3U,
          "DO expected 3, got %" PRIu32, counts[SX1276_RX_RETUNE_DO]);
    CHECK(counts[SX1276_RX_RETUNE_SKIP_NOT_INIT] == 1U,
          "SKIP_NOT_INIT expected 1, got %" PRIu32,
          counts[SX1276_RX_RETUNE_SKIP_NOT_INIT]);
    CHECK(counts[SX1276_RX_RETUNE_SKIP_BUSY] == 1U,
          "SKIP_BUSY expected 1, got %" PRIu32,
          counts[SX1276_RX_RETUNE_SKIP_BUSY]);
    CHECK(counts[SX1276_RX_RETUNE_SKIP_TOO_SOON] == 3U,
          "SKIP_TOO_SOON expected 3, got %" PRIu32,
          counts[SX1276_RX_RETUNE_SKIP_TOO_SOON]);
    CHECK(counts[SX1276_RX_RETUNE_DO_WRAP] == 2U,
          "DO_WRAP expected 2, got %" PRIu32,
          counts[SX1276_RX_RETUNE_DO_WRAP]);
}

static void test_dim_matches_enum(void) {
    CHECK(SX1276_RX_RETUNE_DECISION_DIM ==
              ((uint32_t)SX1276_RX_RETUNE_DO_WRAP + 1U),
          "SX1276_RX_RETUNE_DECISION_DIM (%u) must equal "
          "max-enum + 1 (%u)",
          SX1276_RX_RETUNE_DECISION_DIM,
          (unsigned int)SX1276_RX_RETUNE_DO_WRAP + 1U);
}

static void test_round_trip_after_reset_is_fresh(void) {
    sx1276_rx_retune_counts_reset();
    for (uint32_t k = 0U; k < 17U; ++k) {
        sx1276_rx_retune_counter_record(SX1276_RX_RETUNE_DO);
    }
    sx1276_rx_retune_counts_reset();

    uint32_t counts[SX1276_RX_RETUNE_DECISION_DIM];
    sx1276_rx_retune_counts(counts);
    for (uint8_t i = 0U; i < SX1276_RX_RETUNE_DECISION_DIM; ++i) {
        CHECK(counts[i] == 0U,
              "post-reset round-trip count[%u] must be 0, got %" PRIu32,
              i, counts[i]);
    }
}

int main(void) {
    test_initial_state_all_zero();
    test_each_slot_increments_in_isolation();
    test_out_of_range_decision_is_noop();
    test_snapshot_is_readonly();
    test_reset_zeroes_then_usable();
    test_null_getter_is_noop();
    test_saturating_increment();
    test_interleaved_records_accumulate();
    test_dim_matches_enum();
    test_round_trip_after_reset_is_fresh();

    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] rx_retune_counters: %d failure(s)\n",
                g_failures);
        return EXIT_FAILURE;
    }
    printf("[PASS] rx_retune_counters: 10 cases\n");
    return EXIT_SUCCESS;
}
