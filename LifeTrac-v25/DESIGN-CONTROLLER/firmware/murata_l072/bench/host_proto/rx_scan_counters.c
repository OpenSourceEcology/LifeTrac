/*
 * FCC-A6c-2 (counters TU): host test for
 * radio/sx1276_rx_scan_counters.c.
 *
 * Mirror of bench/host_proto/rx_retune_counters.c (γ-2) adapted to
 * the two-axis Scanning histogram (per-state + per-action).
 * Verifies the saturating per-slot counters that FCC-B1-SUMMARY
 * will read for the cold-start scan profile.
 *
 * Cases:
 *   (1)  Initial state — both arrays zero before any record().
 *   (2)  Each state-enum slot increments the right index in
 *        isolation; action axis advances in lockstep.
 *   (3)  Each action-enum slot increments the right index in
 *        isolation; state axis advances in lockstep.
 *   (4)  Out-of-range state cast drops state increment but still
 *        bumps the in-range action slot.
 *   (5)  Out-of-range action cast drops action increment but still
 *        bumps the in-range state slot.
 *   (6)  Both axes out-of-range — total no-op.
 *   (7)  Snapshot getters are read-only — repeated calls return
 *        the same values.
 *   (8)  reset() zeros BOTH arrays and leaves storage usable.
 *   (9)  NULL getter argument is a no-op on either axis (no crash,
 *        no mutation).
 *   (10) No narrow-type wrap @ 4096 records on a single slot pair.
 *   (11) Interleaved records accumulate independently per slot on
 *        both axes.
 *   (12) STATE_DIM equals max-state-enum + 1.
 *   (13) ACTION_DIM equals max-action-enum + 1.
 *   (14) Round-trip after reset behaves like a fresh module.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "sx1276_rx_scan_policy.h"

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

static void snapshot_both(uint32_t s_out[SX1276_RX_SCAN_STATE_DIM],
                          uint32_t a_out[SX1276_RX_SCAN_ACTION_DIM]) {
    sx1276_rx_scan_state_counts(s_out);
    sx1276_rx_scan_action_counts(a_out);
}

static void test_initial_state_all_zero(void) {
    sx1276_rx_scan_counts_reset();
    uint32_t s[SX1276_RX_SCAN_STATE_DIM];
    uint32_t a[SX1276_RX_SCAN_ACTION_DIM];
    snapshot_both(s, a);
    for (uint8_t i = 0U; i < SX1276_RX_SCAN_STATE_DIM; ++i) {
        CHECK(s[i] == 0U,
              "post-reset state[%u] must be 0, got %" PRIu32, i, s[i]);
    }
    for (uint8_t i = 0U; i < SX1276_RX_SCAN_ACTION_DIM; ++i) {
        CHECK(a[i] == 0U,
              "post-reset action[%u] must be 0, got %" PRIu32, i, a[i]);
    }
}

static void test_each_state_slot_isolation(void) {
    const sx1276_rx_scan_state_t all[] = {
        SX1276_RX_SCAN_STATE_BOOT,
        SX1276_RX_SCAN_STATE_SCANNING,
        SX1276_RX_SCAN_STATE_LOCKED,
        SX1276_RX_SCAN_STATE_FAILED,
    };
    for (size_t k = 0; k < sizeof(all) / sizeof(all[0]); ++k) {
        sx1276_rx_scan_counts_reset();
        /* Pair every state with the same in-range action (HOLD) so
         * we can assert the action axis bumps once too. */
        sx1276_rx_scan_counter_record(all[k], SX1276_RX_SCAN_ACTION_HOLD);
        uint32_t s[SX1276_RX_SCAN_STATE_DIM];
        uint32_t a[SX1276_RX_SCAN_ACTION_DIM];
        snapshot_both(s, a);
        for (uint8_t i = 0U; i < SX1276_RX_SCAN_STATE_DIM; ++i) {
            const uint32_t expected = (i == (uint8_t)all[k]) ? 1U : 0U;
            CHECK(s[i] == expected,
                  "state[%u] after record(state=%d): expected %" PRIu32
                  " got %" PRIu32, i, (int)all[k], expected, s[i]);
        }
        CHECK(a[SX1276_RX_SCAN_ACTION_HOLD] == 1U,
              "action HOLD must bump in lockstep, got %" PRIu32,
              a[SX1276_RX_SCAN_ACTION_HOLD]);
    }
}

static void test_each_action_slot_isolation(void) {
    const sx1276_rx_scan_action_t all[] = {
        SX1276_RX_SCAN_ACTION_HOLD,
        SX1276_RX_SCAN_ACTION_BEGIN_SCAN,
        SX1276_RX_SCAN_ACTION_ADVANCE_CHANNEL,
        SX1276_RX_SCAN_ACTION_LOCK,
        SX1276_RX_SCAN_ACTION_FAIL,
        SX1276_RX_SCAN_ACTION_REANCHOR,
    };
    for (size_t k = 0; k < sizeof(all) / sizeof(all[0]); ++k) {
        sx1276_rx_scan_counts_reset();
        sx1276_rx_scan_counter_record(SX1276_RX_SCAN_STATE_SCANNING, all[k]);
        uint32_t s[SX1276_RX_SCAN_STATE_DIM];
        uint32_t a[SX1276_RX_SCAN_ACTION_DIM];
        snapshot_both(s, a);
        for (uint8_t i = 0U; i < SX1276_RX_SCAN_ACTION_DIM; ++i) {
            const uint32_t expected = (i == (uint8_t)all[k]) ? 1U : 0U;
            CHECK(a[i] == expected,
                  "action[%u] after record(action=%d): expected %" PRIu32
                  " got %" PRIu32, i, (int)all[k], expected, a[i]);
        }
        CHECK(s[SX1276_RX_SCAN_STATE_SCANNING] == 1U,
              "state SCANNING must bump in lockstep, got %" PRIu32,
              s[SX1276_RX_SCAN_STATE_SCANNING]);
    }
}

static void test_oor_state_drops_state_keeps_action(void) {
    sx1276_rx_scan_counts_reset();
    sx1276_rx_scan_counter_record((sx1276_rx_scan_state_t)99,
                                  SX1276_RX_SCAN_ACTION_LOCK);
    uint32_t s[SX1276_RX_SCAN_STATE_DIM];
    uint32_t a[SX1276_RX_SCAN_ACTION_DIM];
    snapshot_both(s, a);
    for (uint8_t i = 0U; i < SX1276_RX_SCAN_STATE_DIM; ++i) {
        CHECK(s[i] == 0U,
              "OOR state must not mutate state[%u], got %" PRIu32, i, s[i]);
    }
    CHECK(a[SX1276_RX_SCAN_ACTION_LOCK] == 1U,
          "in-range action LOCK must still increment, got %" PRIu32,
          a[SX1276_RX_SCAN_ACTION_LOCK]);
}

static void test_oor_action_drops_action_keeps_state(void) {
    sx1276_rx_scan_counts_reset();
    sx1276_rx_scan_counter_record(SX1276_RX_SCAN_STATE_LOCKED,
                                  (sx1276_rx_scan_action_t)99);
    uint32_t s[SX1276_RX_SCAN_STATE_DIM];
    uint32_t a[SX1276_RX_SCAN_ACTION_DIM];
    snapshot_both(s, a);
    CHECK(s[SX1276_RX_SCAN_STATE_LOCKED] == 1U,
          "in-range state LOCKED must still increment, got %" PRIu32,
          s[SX1276_RX_SCAN_STATE_LOCKED]);
    for (uint8_t i = 0U; i < SX1276_RX_SCAN_ACTION_DIM; ++i) {
        CHECK(a[i] == 0U,
              "OOR action must not mutate action[%u], got %" PRIu32, i, a[i]);
    }
}

static void test_oor_both_is_total_noop(void) {
    sx1276_rx_scan_counts_reset();
    sx1276_rx_scan_counter_record((sx1276_rx_scan_state_t)
                                      SX1276_RX_SCAN_STATE_DIM,
                                  (sx1276_rx_scan_action_t)
                                      SX1276_RX_SCAN_ACTION_DIM);
    uint32_t s[SX1276_RX_SCAN_STATE_DIM];
    uint32_t a[SX1276_RX_SCAN_ACTION_DIM];
    snapshot_both(s, a);
    for (uint8_t i = 0U; i < SX1276_RX_SCAN_STATE_DIM; ++i) {
        CHECK(s[i] == 0U,
              "both-OOR must not mutate state[%u], got %" PRIu32, i, s[i]);
    }
    for (uint8_t i = 0U; i < SX1276_RX_SCAN_ACTION_DIM; ++i) {
        CHECK(a[i] == 0U,
              "both-OOR must not mutate action[%u], got %" PRIu32, i, a[i]);
    }
}

static void test_snapshot_is_readonly(void) {
    sx1276_rx_scan_counts_reset();
    sx1276_rx_scan_counter_record(SX1276_RX_SCAN_STATE_SCANNING,
                                  SX1276_RX_SCAN_ACTION_HOLD);
    sx1276_rx_scan_counter_record(SX1276_RX_SCAN_STATE_SCANNING,
                                  SX1276_RX_SCAN_ACTION_HOLD);
    sx1276_rx_scan_counter_record(SX1276_RX_SCAN_STATE_LOCKED,
                                  SX1276_RX_SCAN_ACTION_LOCK);
    uint32_t s1[SX1276_RX_SCAN_STATE_DIM], s2[SX1276_RX_SCAN_STATE_DIM];
    uint32_t a1[SX1276_RX_SCAN_ACTION_DIM], a2[SX1276_RX_SCAN_ACTION_DIM];
    sx1276_rx_scan_state_counts(s1);
    sx1276_rx_scan_action_counts(a1);
    sx1276_rx_scan_state_counts(s2);
    sx1276_rx_scan_action_counts(a2);
    for (uint8_t i = 0U; i < SX1276_RX_SCAN_STATE_DIM; ++i) {
        CHECK(s1[i] == s2[i],
              "state snapshot idempotent; s1[%u]=%" PRIu32
              " s2[%u]=%" PRIu32, i, s1[i], i, s2[i]);
    }
    for (uint8_t i = 0U; i < SX1276_RX_SCAN_ACTION_DIM; ++i) {
        CHECK(a1[i] == a2[i],
              "action snapshot idempotent; a1[%u]=%" PRIu32
              " a2[%u]=%" PRIu32, i, a1[i], i, a2[i]);
    }
    CHECK(s1[SX1276_RX_SCAN_STATE_SCANNING] == 2U, "SCANNING state must be 2");
    CHECK(s1[SX1276_RX_SCAN_STATE_LOCKED] == 1U, "LOCKED state must be 1");
    CHECK(a1[SX1276_RX_SCAN_ACTION_HOLD] == 2U, "HOLD action must be 2");
    CHECK(a1[SX1276_RX_SCAN_ACTION_LOCK] == 1U, "LOCK action must be 1");
}

static void test_reset_zeroes_both_arrays(void) {
    sx1276_rx_scan_counts_reset();
    sx1276_rx_scan_counter_record(SX1276_RX_SCAN_STATE_FAILED,
                                  SX1276_RX_SCAN_ACTION_FAIL);
    sx1276_rx_scan_counter_record(SX1276_RX_SCAN_STATE_SCANNING,
                                  SX1276_RX_SCAN_ACTION_ADVANCE_CHANNEL);
    sx1276_rx_scan_counts_reset();
    uint32_t s[SX1276_RX_SCAN_STATE_DIM];
    uint32_t a[SX1276_RX_SCAN_ACTION_DIM];
    snapshot_both(s, a);
    for (uint8_t i = 0U; i < SX1276_RX_SCAN_STATE_DIM; ++i) {
        CHECK(s[i] == 0U,
              "post-reset state[%u] must be 0, got %" PRIu32, i, s[i]);
    }
    for (uint8_t i = 0U; i < SX1276_RX_SCAN_ACTION_DIM; ++i) {
        CHECK(a[i] == 0U,
              "post-reset action[%u] must be 0, got %" PRIu32, i, a[i]);
    }
    sx1276_rx_scan_counter_record(SX1276_RX_SCAN_STATE_BOOT,
                                  SX1276_RX_SCAN_ACTION_BEGIN_SCAN);
    snapshot_both(s, a);
    CHECK(s[SX1276_RX_SCAN_STATE_BOOT] == 1U,
          "post-reset state record must work; BOOT=%" PRIu32,
          s[SX1276_RX_SCAN_STATE_BOOT]);
    CHECK(a[SX1276_RX_SCAN_ACTION_BEGIN_SCAN] == 1U,
          "post-reset action record must work; BEGIN_SCAN=%" PRIu32,
          a[SX1276_RX_SCAN_ACTION_BEGIN_SCAN]);
}

static void test_null_getter_is_noop(void) {
    sx1276_rx_scan_counts_reset();
    sx1276_rx_scan_counter_record(SX1276_RX_SCAN_STATE_SCANNING,
                                  SX1276_RX_SCAN_ACTION_HOLD);
    sx1276_rx_scan_state_counts(NULL);
    sx1276_rx_scan_action_counts(NULL);
    uint32_t s[SX1276_RX_SCAN_STATE_DIM];
    uint32_t a[SX1276_RX_SCAN_ACTION_DIM];
    snapshot_both(s, a);
    CHECK(s[SX1276_RX_SCAN_STATE_SCANNING] == 1U,
          "NULL getter must not perturb state storage; SCANNING=%" PRIu32,
          s[SX1276_RX_SCAN_STATE_SCANNING]);
    CHECK(a[SX1276_RX_SCAN_ACTION_HOLD] == 1U,
          "NULL getter must not perturb action storage; HOLD=%" PRIu32,
          a[SX1276_RX_SCAN_ACTION_HOLD]);
}

static void test_no_narrow_type_wrap_at_4096(void) {
    sx1276_rx_scan_counts_reset();
    for (uint32_t k = 0U; k < 4096U; ++k) {
        sx1276_rx_scan_counter_record(SX1276_RX_SCAN_STATE_SCANNING,
                                      SX1276_RX_SCAN_ACTION_HOLD);
    }
    uint32_t s[SX1276_RX_SCAN_STATE_DIM];
    uint32_t a[SX1276_RX_SCAN_ACTION_DIM];
    snapshot_both(s, a);
    CHECK(s[SX1276_RX_SCAN_STATE_SCANNING] == 4096U,
          "4096 records must yield 4096 state count (no narrow wrap); got %"
          PRIu32, s[SX1276_RX_SCAN_STATE_SCANNING]);
    CHECK(a[SX1276_RX_SCAN_ACTION_HOLD] == 4096U,
          "4096 records must yield 4096 action count (no narrow wrap); got %"
          PRIu32, a[SX1276_RX_SCAN_ACTION_HOLD]);
}

static void test_interleaved_records_accumulate(void) {
    sx1276_rx_scan_counts_reset();
    struct {
        sx1276_rx_scan_state_t  st;
        sx1276_rx_scan_action_t ac;
    } pattern[] = {
        { SX1276_RX_SCAN_STATE_BOOT,     SX1276_RX_SCAN_ACTION_BEGIN_SCAN },
        { SX1276_RX_SCAN_STATE_SCANNING, SX1276_RX_SCAN_ACTION_HOLD },
        { SX1276_RX_SCAN_STATE_SCANNING, SX1276_RX_SCAN_ACTION_HOLD },
        { SX1276_RX_SCAN_STATE_SCANNING, SX1276_RX_SCAN_ACTION_ADVANCE_CHANNEL },
        { SX1276_RX_SCAN_STATE_SCANNING, SX1276_RX_SCAN_ACTION_HOLD },
        { SX1276_RX_SCAN_STATE_SCANNING, SX1276_RX_SCAN_ACTION_REANCHOR },
        { SX1276_RX_SCAN_STATE_SCANNING, SX1276_RX_SCAN_ACTION_LOCK },
        { SX1276_RX_SCAN_STATE_LOCKED,   SX1276_RX_SCAN_ACTION_HOLD },
        { SX1276_RX_SCAN_STATE_LOCKED,   SX1276_RX_SCAN_ACTION_HOLD },
        { SX1276_RX_SCAN_STATE_FAILED,   SX1276_RX_SCAN_ACTION_FAIL },
    };
    for (size_t k = 0; k < sizeof(pattern) / sizeof(pattern[0]); ++k) {
        sx1276_rx_scan_counter_record(pattern[k].st, pattern[k].ac);
    }
    uint32_t s[SX1276_RX_SCAN_STATE_DIM];
    uint32_t a[SX1276_RX_SCAN_ACTION_DIM];
    snapshot_both(s, a);
    CHECK(s[SX1276_RX_SCAN_STATE_BOOT] == 1U,
          "BOOT expected 1, got %" PRIu32, s[SX1276_RX_SCAN_STATE_BOOT]);
    CHECK(s[SX1276_RX_SCAN_STATE_SCANNING] == 6U,
          "SCANNING expected 6, got %" PRIu32,
          s[SX1276_RX_SCAN_STATE_SCANNING]);
    CHECK(s[SX1276_RX_SCAN_STATE_LOCKED] == 2U,
          "LOCKED expected 2, got %" PRIu32, s[SX1276_RX_SCAN_STATE_LOCKED]);
    CHECK(s[SX1276_RX_SCAN_STATE_FAILED] == 1U,
          "FAILED expected 1, got %" PRIu32, s[SX1276_RX_SCAN_STATE_FAILED]);
    CHECK(a[SX1276_RX_SCAN_ACTION_HOLD] == 5U,
          "HOLD expected 5, got %" PRIu32, a[SX1276_RX_SCAN_ACTION_HOLD]);
    CHECK(a[SX1276_RX_SCAN_ACTION_BEGIN_SCAN] == 1U,
          "BEGIN_SCAN expected 1, got %" PRIu32,
          a[SX1276_RX_SCAN_ACTION_BEGIN_SCAN]);
    CHECK(a[SX1276_RX_SCAN_ACTION_ADVANCE_CHANNEL] == 1U,
          "ADVANCE_CHANNEL expected 1, got %" PRIu32,
          a[SX1276_RX_SCAN_ACTION_ADVANCE_CHANNEL]);
    CHECK(a[SX1276_RX_SCAN_ACTION_LOCK] == 1U,
          "LOCK expected 1, got %" PRIu32, a[SX1276_RX_SCAN_ACTION_LOCK]);
    CHECK(a[SX1276_RX_SCAN_ACTION_FAIL] == 1U,
          "FAIL expected 1, got %" PRIu32, a[SX1276_RX_SCAN_ACTION_FAIL]);
    CHECK(a[SX1276_RX_SCAN_ACTION_REANCHOR] == 1U,
          "REANCHOR expected 1, got %" PRIu32,
          a[SX1276_RX_SCAN_ACTION_REANCHOR]);
}

static void test_state_dim_matches_enum(void) {
    CHECK(SX1276_RX_SCAN_STATE_DIM ==
              ((uint32_t)SX1276_RX_SCAN_STATE_FAILED + 1U),
          "STATE_DIM (%u) must equal max-state-enum + 1 (%u)",
          SX1276_RX_SCAN_STATE_DIM,
          (unsigned int)SX1276_RX_SCAN_STATE_FAILED + 1U);
}

static void test_action_dim_matches_enum(void) {
    CHECK(SX1276_RX_SCAN_ACTION_DIM ==
              ((uint32_t)SX1276_RX_SCAN_ACTION_REANCHOR + 1U),
          "ACTION_DIM (%u) must equal max-action-enum + 1 (%u)",
          SX1276_RX_SCAN_ACTION_DIM,
          (unsigned int)SX1276_RX_SCAN_ACTION_REANCHOR + 1U);
}

static void test_round_trip_after_reset_is_fresh(void) {
    sx1276_rx_scan_counts_reset();
    for (uint32_t k = 0U; k < 17U; ++k) {
        sx1276_rx_scan_counter_record(SX1276_RX_SCAN_STATE_SCANNING,
                                      SX1276_RX_SCAN_ACTION_HOLD);
    }
    sx1276_rx_scan_counts_reset();
    uint32_t s[SX1276_RX_SCAN_STATE_DIM];
    uint32_t a[SX1276_RX_SCAN_ACTION_DIM];
    snapshot_both(s, a);
    for (uint8_t i = 0U; i < SX1276_RX_SCAN_STATE_DIM; ++i) {
        CHECK(s[i] == 0U,
              "post-reset round-trip state[%u] must be 0, got %" PRIu32,
              i, s[i]);
    }
    for (uint8_t i = 0U; i < SX1276_RX_SCAN_ACTION_DIM; ++i) {
        CHECK(a[i] == 0U,
              "post-reset round-trip action[%u] must be 0, got %" PRIu32,
              i, a[i]);
    }
}

int main(void) {
    test_initial_state_all_zero();
    test_each_state_slot_isolation();
    test_each_action_slot_isolation();
    test_oor_state_drops_state_keeps_action();
    test_oor_action_drops_action_keeps_state();
    test_oor_both_is_total_noop();
    test_snapshot_is_readonly();
    test_reset_zeroes_both_arrays();
    test_null_getter_is_noop();
    test_no_narrow_type_wrap_at_4096();
    test_interleaved_records_accumulate();
    test_state_dim_matches_enum();
    test_action_dim_matches_enum();
    test_round_trip_after_reset_is_fresh();

    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] rx_scan_counters: %d failure(s)\n",
                g_failures);
        return EXIT_FAILURE;
    }
    printf("[PASS] rx_scan_counters: 14 cases\n");
    return EXIT_SUCCESS;
}
