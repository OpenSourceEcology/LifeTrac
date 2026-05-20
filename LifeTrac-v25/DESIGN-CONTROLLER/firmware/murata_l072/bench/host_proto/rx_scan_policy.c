/*
 * FCC-A6c-1: host golden-vector test for the RX cold-start Scanning
 * state-machine policy (radio/sx1276_rx_scan_policy.c).
 *
 * Cases:
 *   (1)  NULL input → HOLD/BOOT.
 *   (2)  BOOT + TICK → BEGIN_SCAN/SCANNING.
 *   (3)  BOOT + FRAME_VALID → BEGIN_SCAN/SCANNING (cannot lock from
 *        BOOT — anchors aren't set yet).
 *   (4)  BOOT + FRAME_INVALID → BEGIN_SCAN/SCANNING.
 *   (5)  SCANNING + TICK, channel_elapsed < DWELL_MS → HOLD/SCANNING.
 *   (6)  SCANNING + TICK, channel_elapsed == DWELL_MS → ADVANCE/SCANNING.
 *   (7)  SCANNING + TICK, channel_elapsed > DWELL_MS but
 *        cold_elapsed < REDESIGN_MS → ADVANCE/SCANNING.
 *   (8)  SCANNING + TICK, cold_elapsed == REDESIGN_MS → FAIL/FAILED
 *        (regulatory abort wins over channel ADVANCE).
 *   (9)  SCANNING + TICK, cold_elapsed > REDESIGN_MS → FAIL/FAILED.
 *   (10) SCANNING + FRAME_VALID with header_ok=true → LOCK/LOCKED.
 *   (11) SCANNING + FRAME_VALID with header_ok=false → HOLD/SCANNING
 *        (defensive demotion).
 *   (12) SCANNING + FRAME_INVALID → HOLD/SCANNING.
 *   (13) LOCKED + TICK → HOLD/LOCKED (γ-1 owns retune from here).
 *   (14) LOCKED + FRAME_VALID → HOLD/LOCKED.
 *   (15) LOCKED + FRAME_INVALID → HOLD/LOCKED.
 *   (16) FAILED + any event → HOLD/FAILED (absorbing).
 *   (17) SCANNING + TICK with channel_entry_ms wrap → REANCHOR.
 *   (18) SCANNING + TICK with cold_start_entry_ms wrap → REANCHOR.
 *   (19) DIM constants match max-enum + 1 (state / event / action).
 *   (20) Enum numeric stability (BOOT==0, SCANNING==1, LOCKED==2,
 *        FAILED==3 / TICK==0, FRAME_VALID==1, FRAME_INVALID==2 /
 *        HOLD==0, BEGIN_SCAN==1, ADVANCE==2, LOCK==3, FAIL==4,
 *        REANCHOR==5) — guards FCC-B1-SUMMARY URC indexing.
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

static void check_decision(const char *label,
                           sx1276_rx_scan_decision_t got,
                           sx1276_rx_scan_action_t want_action,
                           sx1276_rx_scan_state_t  want_state) {
    CHECK((int)got.action == (int)want_action,
          "%s: action expected %d got %d",
          label, (int)want_action, (int)got.action);
    CHECK((int)got.next_state == (int)want_state,
          "%s: next_state expected %d got %d",
          label, (int)want_state, (int)got.next_state);
}

static void test_null_input(void) {
    sx1276_rx_scan_decision_t d = sx1276_rx_scan_eval(NULL);
    check_decision("(1) NULL input",
                   d,
                   SX1276_RX_SCAN_ACTION_HOLD,
                   SX1276_RX_SCAN_STATE_BOOT);
}

static void test_boot_tick(void) {
    sx1276_rx_scan_input_t in = {
        .state               = SX1276_RX_SCAN_STATE_BOOT,
        .now_ms              = 0U,
        .channel_entry_ms    = 0U,
        .cold_start_entry_ms = 0U,
        .event               = SX1276_RX_SCAN_EVENT_TICK,
        .frame_header_valid  = false,
    };
    sx1276_rx_scan_decision_t d = sx1276_rx_scan_eval(&in);
    check_decision("(2) BOOT+TICK",
                   d,
                   SX1276_RX_SCAN_ACTION_BEGIN_SCAN,
                   SX1276_RX_SCAN_STATE_SCANNING);
}

static void test_boot_frame_valid(void) {
    sx1276_rx_scan_input_t in = {
        .state               = SX1276_RX_SCAN_STATE_BOOT,
        .now_ms              = 0U,
        .channel_entry_ms    = 0U,
        .cold_start_entry_ms = 0U,
        .event               = SX1276_RX_SCAN_EVENT_FRAME_VALID,
        .frame_header_valid  = true,
    };
    sx1276_rx_scan_decision_t d = sx1276_rx_scan_eval(&in);
    check_decision("(3) BOOT+FRAME_VALID (no anchors yet → must scan)",
                   d,
                   SX1276_RX_SCAN_ACTION_BEGIN_SCAN,
                   SX1276_RX_SCAN_STATE_SCANNING);
}

static void test_boot_frame_invalid(void) {
    sx1276_rx_scan_input_t in = {
        .state               = SX1276_RX_SCAN_STATE_BOOT,
        .now_ms              = 0U,
        .channel_entry_ms    = 0U,
        .cold_start_entry_ms = 0U,
        .event               = SX1276_RX_SCAN_EVENT_FRAME_INVALID,
        .frame_header_valid  = false,
    };
    sx1276_rx_scan_decision_t d = sx1276_rx_scan_eval(&in);
    check_decision("(4) BOOT+FRAME_INVALID",
                   d,
                   SX1276_RX_SCAN_ACTION_BEGIN_SCAN,
                   SX1276_RX_SCAN_STATE_SCANNING);
}

static void test_scanning_tick_before_dwell(void) {
    sx1276_rx_scan_input_t in = {
        .state               = SX1276_RX_SCAN_STATE_SCANNING,
        .channel_entry_ms    = 1000U,
        .cold_start_entry_ms = 1000U,
        .now_ms              = 1000U + (SX1276_RX_SCAN_DWELL_MS - 1U),
        .event               = SX1276_RX_SCAN_EVENT_TICK,
        .frame_header_valid  = false,
    };
    sx1276_rx_scan_decision_t d = sx1276_rx_scan_eval(&in);
    check_decision("(5) SCANNING+TICK before dwell",
                   d,
                   SX1276_RX_SCAN_ACTION_HOLD,
                   SX1276_RX_SCAN_STATE_SCANNING);
}

static void test_scanning_tick_exactly_dwell(void) {
    sx1276_rx_scan_input_t in = {
        .state               = SX1276_RX_SCAN_STATE_SCANNING,
        .channel_entry_ms    = 1000U,
        .cold_start_entry_ms = 1000U,
        .now_ms              = 1000U + SX1276_RX_SCAN_DWELL_MS,
        .event               = SX1276_RX_SCAN_EVENT_TICK,
        .frame_header_valid  = false,
    };
    sx1276_rx_scan_decision_t d = sx1276_rx_scan_eval(&in);
    check_decision("(6) SCANNING+TICK at dwell boundary",
                   d,
                   SX1276_RX_SCAN_ACTION_ADVANCE_CHANNEL,
                   SX1276_RX_SCAN_STATE_SCANNING);
}

static void test_scanning_tick_past_dwell_before_redesign(void) {
    sx1276_rx_scan_input_t in = {
        .state               = SX1276_RX_SCAN_STATE_SCANNING,
        .channel_entry_ms    = 4000U,
        .cold_start_entry_ms = 100U,
        .now_ms              = 4000U + SX1276_RX_SCAN_DWELL_MS + 25U,
        .event               = SX1276_RX_SCAN_EVENT_TICK,
        .frame_header_valid  = false,
    };
    sx1276_rx_scan_decision_t d = sx1276_rx_scan_eval(&in);
    check_decision("(7) SCANNING+TICK past dwell, cold ok",
                   d,
                   SX1276_RX_SCAN_ACTION_ADVANCE_CHANNEL,
                   SX1276_RX_SCAN_STATE_SCANNING);
}

static void test_scanning_tick_cold_exactly_redesign(void) {
    /* cold_elapsed == REDESIGN_MS AND channel_elapsed > DWELL_MS.
     * FAIL must win. */
    const uint32_t cold_anchor = 7U;
    const uint32_t chan_anchor = 7U;
    const uint32_t now = cold_anchor + SX1276_RX_SCAN_REDESIGN_MS;
    sx1276_rx_scan_input_t in = {
        .state               = SX1276_RX_SCAN_STATE_SCANNING,
        .channel_entry_ms    = chan_anchor,
        .cold_start_entry_ms = cold_anchor,
        .now_ms              = now,
        .event               = SX1276_RX_SCAN_EVENT_TICK,
        .frame_header_valid  = false,
    };
    sx1276_rx_scan_decision_t d = sx1276_rx_scan_eval(&in);
    check_decision("(8) SCANNING+TICK at redesign boundary (FAIL wins)",
                   d,
                   SX1276_RX_SCAN_ACTION_FAIL,
                   SX1276_RX_SCAN_STATE_FAILED);
}

static void test_scanning_tick_cold_past_redesign(void) {
    sx1276_rx_scan_input_t in = {
        .state               = SX1276_RX_SCAN_STATE_SCANNING,
        .channel_entry_ms    = 0U,
        .cold_start_entry_ms = 0U,
        .now_ms              = SX1276_RX_SCAN_REDESIGN_MS + 5000U,
        .event               = SX1276_RX_SCAN_EVENT_TICK,
        .frame_header_valid  = false,
    };
    sx1276_rx_scan_decision_t d = sx1276_rx_scan_eval(&in);
    check_decision("(9) SCANNING+TICK past redesign",
                   d,
                   SX1276_RX_SCAN_ACTION_FAIL,
                   SX1276_RX_SCAN_STATE_FAILED);
}

static void test_scanning_frame_valid_locks(void) {
    sx1276_rx_scan_input_t in = {
        .state               = SX1276_RX_SCAN_STATE_SCANNING,
        .channel_entry_ms    = 0U,
        .cold_start_entry_ms = 0U,
        .now_ms              = 500U,
        .event               = SX1276_RX_SCAN_EVENT_FRAME_VALID,
        .frame_header_valid  = true,
    };
    sx1276_rx_scan_decision_t d = sx1276_rx_scan_eval(&in);
    check_decision("(10) SCANNING+FRAME_VALID(hdr ok)",
                   d,
                   SX1276_RX_SCAN_ACTION_LOCK,
                   SX1276_RX_SCAN_STATE_LOCKED);
}

static void test_scanning_frame_valid_but_header_bad_demotes(void) {
    sx1276_rx_scan_input_t in = {
        .state               = SX1276_RX_SCAN_STATE_SCANNING,
        .channel_entry_ms    = 0U,
        .cold_start_entry_ms = 0U,
        .now_ms              = 500U,
        .event               = SX1276_RX_SCAN_EVENT_FRAME_VALID,
        .frame_header_valid  = false,
    };
    sx1276_rx_scan_decision_t d = sx1276_rx_scan_eval(&in);
    check_decision("(11) SCANNING+FRAME_VALID(hdr bad) demotes to invalid",
                   d,
                   SX1276_RX_SCAN_ACTION_HOLD,
                   SX1276_RX_SCAN_STATE_SCANNING);
}

static void test_scanning_frame_invalid_holds(void) {
    sx1276_rx_scan_input_t in = {
        .state               = SX1276_RX_SCAN_STATE_SCANNING,
        .channel_entry_ms    = 0U,
        .cold_start_entry_ms = 0U,
        .now_ms              = 500U,
        .event               = SX1276_RX_SCAN_EVENT_FRAME_INVALID,
        .frame_header_valid  = false,
    };
    sx1276_rx_scan_decision_t d = sx1276_rx_scan_eval(&in);
    check_decision("(12) SCANNING+FRAME_INVALID",
                   d,
                   SX1276_RX_SCAN_ACTION_HOLD,
                   SX1276_RX_SCAN_STATE_SCANNING);
}

static void test_locked_tick(void) {
    sx1276_rx_scan_input_t in = {
        .state               = SX1276_RX_SCAN_STATE_LOCKED,
        .channel_entry_ms    = 0U,
        .cold_start_entry_ms = 0U,
        .now_ms              = 999999U,
        .event               = SX1276_RX_SCAN_EVENT_TICK,
        .frame_header_valid  = false,
    };
    sx1276_rx_scan_decision_t d = sx1276_rx_scan_eval(&in);
    check_decision("(13) LOCKED+TICK",
                   d,
                   SX1276_RX_SCAN_ACTION_HOLD,
                   SX1276_RX_SCAN_STATE_LOCKED);
}

static void test_locked_frame_valid(void) {
    sx1276_rx_scan_input_t in = {
        .state               = SX1276_RX_SCAN_STATE_LOCKED,
        .channel_entry_ms    = 0U,
        .cold_start_entry_ms = 0U,
        .now_ms              = 999999U,
        .event               = SX1276_RX_SCAN_EVENT_FRAME_VALID,
        .frame_header_valid  = true,
    };
    sx1276_rx_scan_decision_t d = sx1276_rx_scan_eval(&in);
    check_decision("(14) LOCKED+FRAME_VALID",
                   d,
                   SX1276_RX_SCAN_ACTION_HOLD,
                   SX1276_RX_SCAN_STATE_LOCKED);
}

static void test_locked_frame_invalid(void) {
    sx1276_rx_scan_input_t in = {
        .state               = SX1276_RX_SCAN_STATE_LOCKED,
        .channel_entry_ms    = 0U,
        .cold_start_entry_ms = 0U,
        .now_ms              = 999999U,
        .event               = SX1276_RX_SCAN_EVENT_FRAME_INVALID,
        .frame_header_valid  = false,
    };
    sx1276_rx_scan_decision_t d = sx1276_rx_scan_eval(&in);
    check_decision("(15) LOCKED+FRAME_INVALID",
                   d,
                   SX1276_RX_SCAN_ACTION_HOLD,
                   SX1276_RX_SCAN_STATE_LOCKED);
}

static void test_failed_is_absorbing(void) {
    const sx1276_rx_scan_event_t events[] = {
        SX1276_RX_SCAN_EVENT_TICK,
        SX1276_RX_SCAN_EVENT_FRAME_VALID,
        SX1276_RX_SCAN_EVENT_FRAME_INVALID,
    };
    for (size_t i = 0; i < sizeof(events) / sizeof(events[0]); ++i) {
        sx1276_rx_scan_input_t in = {
            .state               = SX1276_RX_SCAN_STATE_FAILED,
            .channel_entry_ms    = 0U,
            .cold_start_entry_ms = 0U,
            .now_ms              = 123456U,
            .event               = events[i],
            .frame_header_valid  = true,
        };
        sx1276_rx_scan_decision_t d = sx1276_rx_scan_eval(&in);
        check_decision("(16) FAILED+any (absorbing)",
                       d,
                       SX1276_RX_SCAN_ACTION_HOLD,
                       SX1276_RX_SCAN_STATE_FAILED);
    }
}

static void test_scanning_channel_wrap_reanchors(void) {
    /* tick_wrapped() fires when apparent forward elapsed >= 2^31 ms
     * reinterpreted signed-negative. Real cause: clock glitch /
     * backwards jump. A normal once-per-49.7-day tick wrap reads as
     * a small positive elapsed in modular arithmetic and does NOT
     * trigger REANCHOR (that's correct behavior). */
    sx1276_rx_scan_input_t in = {
        .state               = SX1276_RX_SCAN_STATE_SCANNING,
        .channel_entry_ms    = 100U + 0x80000000U,   /* >=2^31 ahead */
        .cold_start_entry_ms = 100U,                 /* normal */
        .now_ms              = 100U,
        .event               = SX1276_RX_SCAN_EVENT_TICK,
        .frame_header_valid  = false,
    };
    sx1276_rx_scan_decision_t d = sx1276_rx_scan_eval(&in);
    check_decision("(17) SCANNING+TICK channel anchor wrap",
                   d,
                   SX1276_RX_SCAN_ACTION_REANCHOR,
                   SX1276_RX_SCAN_STATE_SCANNING);
}

static void test_scanning_cold_wrap_reanchors(void) {
    sx1276_rx_scan_input_t in = {
        .state               = SX1276_RX_SCAN_STATE_SCANNING,
        .channel_entry_ms    = 100U,                 /* normal */
        .cold_start_entry_ms = 100U + 0x80000000U,   /* >=2^31 ahead */
        .now_ms              = 100U,
        .event               = SX1276_RX_SCAN_EVENT_TICK,
        .frame_header_valid  = false,
    };
    sx1276_rx_scan_decision_t d = sx1276_rx_scan_eval(&in);
    check_decision("(18) SCANNING+TICK cold anchor wrap",
                   d,
                   SX1276_RX_SCAN_ACTION_REANCHOR,
                   SX1276_RX_SCAN_STATE_SCANNING);
}

static void test_dim_matches_enum(void) {
    CHECK(SX1276_RX_SCAN_STATE_DIM ==
              ((uint32_t)SX1276_RX_SCAN_STATE_FAILED + 1U),
          "(19a) STATE_DIM (%u) must equal max-enum+1 (%u)",
          SX1276_RX_SCAN_STATE_DIM,
          (unsigned int)SX1276_RX_SCAN_STATE_FAILED + 1U);
    CHECK(SX1276_RX_SCAN_EVENT_DIM ==
              ((uint32_t)SX1276_RX_SCAN_EVENT_FRAME_INVALID + 1U),
          "(19b) EVENT_DIM (%u) must equal max-enum+1 (%u)",
          SX1276_RX_SCAN_EVENT_DIM,
          (unsigned int)SX1276_RX_SCAN_EVENT_FRAME_INVALID + 1U);
    CHECK(SX1276_RX_SCAN_ACTION_DIM ==
              ((uint32_t)SX1276_RX_SCAN_ACTION_REANCHOR + 1U),
          "(19c) ACTION_DIM (%u) must equal max-enum+1 (%u)",
          SX1276_RX_SCAN_ACTION_DIM,
          (unsigned int)SX1276_RX_SCAN_ACTION_REANCHOR + 1U);
}

static void test_enum_numeric_stability(void) {
    CHECK((int)SX1276_RX_SCAN_STATE_BOOT     == 0, "(20) STATE_BOOT==0");
    CHECK((int)SX1276_RX_SCAN_STATE_SCANNING == 1, "(20) STATE_SCANNING==1");
    CHECK((int)SX1276_RX_SCAN_STATE_LOCKED   == 2, "(20) STATE_LOCKED==2");
    CHECK((int)SX1276_RX_SCAN_STATE_FAILED   == 3, "(20) STATE_FAILED==3");

    CHECK((int)SX1276_RX_SCAN_EVENT_TICK          == 0, "(20) EVENT_TICK==0");
    CHECK((int)SX1276_RX_SCAN_EVENT_FRAME_VALID   == 1, "(20) EVENT_FRAME_VALID==1");
    CHECK((int)SX1276_RX_SCAN_EVENT_FRAME_INVALID == 2, "(20) EVENT_FRAME_INVALID==2");

    CHECK((int)SX1276_RX_SCAN_ACTION_HOLD            == 0, "(20) ACTION_HOLD==0");
    CHECK((int)SX1276_RX_SCAN_ACTION_BEGIN_SCAN      == 1, "(20) ACTION_BEGIN_SCAN==1");
    CHECK((int)SX1276_RX_SCAN_ACTION_ADVANCE_CHANNEL == 2, "(20) ACTION_ADVANCE==2");
    CHECK((int)SX1276_RX_SCAN_ACTION_LOCK            == 3, "(20) ACTION_LOCK==3");
    CHECK((int)SX1276_RX_SCAN_ACTION_FAIL            == 4, "(20) ACTION_FAIL==4");
    CHECK((int)SX1276_RX_SCAN_ACTION_REANCHOR        == 5, "(20) ACTION_REANCHOR==5");
}

int main(void) {
    test_null_input();
    test_boot_tick();
    test_boot_frame_valid();
    test_boot_frame_invalid();
    test_scanning_tick_before_dwell();
    test_scanning_tick_exactly_dwell();
    test_scanning_tick_past_dwell_before_redesign();
    test_scanning_tick_cold_exactly_redesign();
    test_scanning_tick_cold_past_redesign();
    test_scanning_frame_valid_locks();
    test_scanning_frame_valid_but_header_bad_demotes();
    test_scanning_frame_invalid_holds();
    test_locked_tick();
    test_locked_frame_valid();
    test_locked_frame_invalid();
    test_failed_is_absorbing();
    test_scanning_channel_wrap_reanchors();
    test_scanning_cold_wrap_reanchors();
    test_dim_matches_enum();
    test_enum_numeric_stability();

    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] rx_scan_policy: %d failure(s)\n", g_failures);
        return EXIT_FAILURE;
    }
    printf("[PASS] rx_scan_policy: 20 cases\n");
    return EXIT_SUCCESS;
}
