/*
 * Host test for FCC-A5 (host_cfg_profile).
 *
 * Cross-references:
 *   - AI NOTES/2026-05-19_LoRa_FCC_50CH_FHSS_Implementation_Plan_Copilot_v1_0.md
 *     §A5, §11 delta #4 (two-phase commit), §14.2 step 8.
 *   - TODO.md Stage S1.5 -> FCC-A5.
 *
 * Pure-C host test. Compiled with the host gcc against
 * host/host_cfg_profile.c only — no HAL, no radio deps.
 */

#include "host_cfg_keys.h"
#include "host_cfg_profile.h"
#include "host_cfg.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define EXPECT(cond, msg) \
    do { \
        if (!(cond)) { \
            fprintf(stderr, "[FAIL] %s:%d: %s\n", __FILE__, __LINE__, msg); \
            exit(1); \
        } \
    } while (0)

static host_cfg_profile_req_t make_bench_req(void) {
    host_cfg_profile_req_t r;
    memset(&r, 0, sizeof(r));
    r.profile_id       = REG_PROFILE_BENCH_ONLY_FIXED_915;
    r.channel_mask     = 0x1ULL;
    r.modem_bw_hz      = 125000UL;
    r.antenna_gain_dBi = 2;
    r.hw_ceiling_dBm   = 17U;
    return r;
}

static host_cfg_profile_req_t make_fhss_req(void) {
    host_cfg_profile_req_t r;
    memset(&r, 0, sizeof(r));
    r.profile_id       = REG_PROFILE_FCC_15_247_FHSS_50CH_BW250;
    r.channel_mask     = HOST_CFG_PROFILE_FHSS_50CH_REQUIRED_MASK;
    r.modem_bw_hz      = HOST_CFG_PROFILE_FHSS_BW_HZ;
    r.antenna_gain_dBi = 2;
    r.hw_ceiling_dBm   = 20U;
    return r;
}

static host_cfg_profile_req_t make_dts_req(void) {
    host_cfg_profile_req_t r;
    memset(&r, 0, sizeof(r));
    r.profile_id       = REG_PROFILE_FCC_15_247_DTS_BW500;
    r.channel_mask     = 0x1ULL;
    r.modem_bw_hz      = HOST_CFG_PROFILE_DTS_BW_HZ;
    r.antenna_gain_dBi = 0;
    r.hw_ceiling_dBm   = 20U;
    return r;
}

/* -------- validator: profile id -------- */

static void test_validate_bench_ok(void) {
    host_cfg_profile_req_t r = make_bench_req();
    EXPECT(host_cfg_profile_validate(&r, false) == HOST_CFG_PROFILE_REJECT_NONE,
           "bench validates with tx_routed=false");
    EXPECT(host_cfg_profile_validate(&r, true) == HOST_CFG_PROFILE_REJECT_NONE,
           "bench validates with tx_routed=true");
}

static void test_validate_bad_profile_id(void) {
    host_cfg_profile_req_t r = make_bench_req();
    r.profile_id = REG_PROFILE_MAX + 1U;
    EXPECT(host_cfg_profile_validate(&r, true) == HOST_CFG_PROFILE_REJECT_BAD_PROFILE,
           "out-of-range profile_id rejected");
}

static void test_validate_null_arg(void) {
    EXPECT(host_cfg_profile_validate(NULL, true) == HOST_CFG_PROFILE_REJECT_NULL_ARG,
           "NULL req rejected with NULL_ARG");
}

/* -------- validator: unrouted gate -------- */

static void test_validate_fhss_unrouted_rejected(void) {
    host_cfg_profile_req_t r = make_fhss_req();
    EXPECT(host_cfg_profile_validate(&r, false) == HOST_CFG_PROFILE_REJECT_UNROUTED,
           "FHSS profile rejected when tx_routed=false");
}

static void test_validate_dts_unrouted_rejected(void) {
    host_cfg_profile_req_t r = make_dts_req();
    EXPECT(host_cfg_profile_validate(&r, false) == HOST_CFG_PROFILE_REJECT_UNROUTED,
           "DTS profile rejected when tx_routed=false");
}

/* -------- validator: FHSS mask + BW -------- */

static void test_validate_fhss_ok_when_routed(void) {
    host_cfg_profile_req_t r = make_fhss_req();
    EXPECT(host_cfg_profile_validate(&r, true) == HOST_CFG_PROFILE_REJECT_NONE,
           "FHSS profile with full 50-bit mask + BW=250k accepted when routed");
}

static void test_validate_fhss_mask_short(void) {
    host_cfg_profile_req_t r = make_fhss_req();
    /* Drop one bit -> popcount = 49. */
    r.channel_mask &= ~(1ULL << 0);
    EXPECT(host_cfg_profile_validate(&r, true) == HOST_CFG_PROFILE_REJECT_MASK_POPCOUNT,
           "FHSS mask with popcount<50 rejected");
}

static void test_validate_fhss_mask_out_of_table(void) {
    host_cfg_profile_req_t r = make_fhss_req();
    /* Set bit 50 (outside the certified 50-channel table). */
    r.channel_mask |= (1ULL << 50);
    EXPECT(host_cfg_profile_validate(&r, true) == HOST_CFG_PROFILE_REJECT_MASK_OUT_OF_TABLE,
           "FHSS mask with bit outside 0..49 rejected");
}

static void test_validate_fhss_bw_mismatch(void) {
    host_cfg_profile_req_t r = make_fhss_req();
    r.modem_bw_hz = 125000UL;
    EXPECT(host_cfg_profile_validate(&r, true) == HOST_CFG_PROFILE_REJECT_BW_MISMATCH,
           "FHSS profile with BW != 250 kHz rejected");
}

/* -------- validator: DTS -------- */

static void test_validate_dts_ok_when_routed(void) {
    host_cfg_profile_req_t r = make_dts_req();
    EXPECT(host_cfg_profile_validate(&r, true) == HOST_CFG_PROFILE_REJECT_NONE,
           "DTS profile accepted when routed + BW=500k + nonzero mask");
}

static void test_validate_dts_bw_mismatch(void) {
    host_cfg_profile_req_t r = make_dts_req();
    r.modem_bw_hz = 250000UL;
    EXPECT(host_cfg_profile_validate(&r, true) == HOST_CFG_PROFILE_REJECT_BW_MISMATCH,
           "DTS profile with BW != 500 kHz rejected");
}

static void test_validate_dts_zero_mask(void) {
    host_cfg_profile_req_t r = make_dts_req();
    r.channel_mask = 0ULL;
    EXPECT(host_cfg_profile_validate(&r, true) == HOST_CFG_PROFILE_REJECT_MASK_POPCOUNT,
           "DTS profile with zero mask rejected");
}

/* -------- validator: antenna range -------- */

static void test_validate_antenna_negative(void) {
    host_cfg_profile_req_t r = make_fhss_req();
    r.antenna_gain_dBi = -1;
    EXPECT(host_cfg_profile_validate(&r, true) == HOST_CFG_PROFILE_REJECT_ANTENNA_OUT_OF_RANGE,
           "negative antenna gain rejected");
}

static void test_validate_antenna_too_high(void) {
    host_cfg_profile_req_t r = make_fhss_req();
    r.antenna_gain_dBi = HOST_CFG_PROFILE_ANTENNA_GAIN_MAX_DBI + 1;
    EXPECT(host_cfg_profile_validate(&r, true) == HOST_CFG_PROFILE_REJECT_ANTENNA_OUT_OF_RANGE,
           "antenna gain > 30 dBi rejected");
}

/* -------- validator: power headroom -------- */

static void test_validate_power_no_headroom(void) {
    host_cfg_profile_req_t r = make_fhss_req();
    /* tier=30, gain=29 -> reduction=23 -> allowed=7; hw=2 -> clamp to 2 (OK at min).
     * To force no-headroom, drop hw_ceiling below the min. */
    r.hw_ceiling_dBm = 1U;
    EXPECT(host_cfg_profile_validate(&r, true) == HOST_CFG_PROFILE_REJECT_NO_POWER_HEADROOM,
           "hw ceiling below tx-min triggers NO_POWER_HEADROOM");
}

/* -------- power clamp arithmetic -------- */

static void test_power_clamp_no_reduction(void) {
    /* gain=0 -> no reduction; min(30, 17) = 17 */
    EXPECT(host_cfg_profile_power_clamp(30U, 17U, 0) == 17U,
           "gain=0 clamps to hw_ceiling");
    /* gain=6 -> still no reduction */
    EXPECT(host_cfg_profile_power_clamp(30U, 17U, 6) == 17U,
           "gain=6 still no reduction (threshold)");
}

static void test_power_clamp_reduction(void) {
    /* gain=10 -> reduction=4 -> allowed=26; clamp to hw=20 -> 20 */
    EXPECT(host_cfg_profile_power_clamp(30U, 20U, 10) == 20U,
           "gain=10 -> tier=26 clamped to hw=20");
    /* gain=12 -> reduction=6 -> allowed=24; hw=30 -> 24 */
    EXPECT(host_cfg_profile_power_clamp(30U, 30U, 12) == 24U,
           "gain=12 -> 24 dBm tier-limited");
    /* gain=29 -> reduction=23 -> allowed=7; hw=30 -> 7 */
    EXPECT(host_cfg_profile_power_clamp(30U, 30U, 29) == 7U,
           "gain=29 -> 7 dBm tier-limited");
}

static void test_power_clamp_negative_gain(void) {
    /* negative gain is treated as zero reduction */
    EXPECT(host_cfg_profile_power_clamp(30U, 17U, -3) == 17U,
           "negative gain treated as no reduction");
}

static void test_power_clamp_no_headroom(void) {
    /* tier=30, gain=29 -> allowed=7; hw=1 -> below min -> 0 */
    EXPECT(host_cfg_profile_power_clamp(30U, 1U, 29) == 0U,
           "hw_ceiling=1 below min returns 0");
    /* tier=30, gain=30 -> reduction=24 -> allowed=6; hw=6 -> 6 (just above min) */
    EXPECT(host_cfg_profile_power_clamp(30U, 6U, 30) == 6U,
           "barely above min returns 6");
}

/* -------- tier ceiling lookup -------- */

static void test_tier_ceiling_lookup(void) {
    EXPECT(host_cfg_profile_tier_ceiling_dBm(REG_PROFILE_BENCH_ONLY_FIXED_915) ==
               HOST_CFG_PROFILE_TIER_CEILING_BENCH_DBM,
           "bench tier ceiling = 17");
    EXPECT(host_cfg_profile_tier_ceiling_dBm(REG_PROFILE_FCC_15_247_FHSS_50CH_BW250) ==
               HOST_CFG_PROFILE_TIER_CEILING_FHSS_DBM,
           "FHSS tier ceiling = 30");
    EXPECT(host_cfg_profile_tier_ceiling_dBm(REG_PROFILE_FCC_15_247_DTS_BW500) ==
               HOST_CFG_PROFILE_TIER_CEILING_DTS_DBM,
           "DTS tier ceiling = 30");
    EXPECT(host_cfg_profile_tier_ceiling_dBm(0xFFU) == 0U,
           "unknown profile tier ceiling = 0");
}

/* -------- two-phase commit -------- */

static void test_stage_activate_happy_path(void) {
    host_cfg_profile_req_t initial = make_bench_req();
    host_cfg_profile_reset(&initial);

    EXPECT(!host_cfg_profile_has_stage(), "no stage after reset");
    EXPECT(host_cfg_profile_active()->profile_id == REG_PROFILE_BENCH_ONLY_FIXED_915,
           "active = bench after reset");

    host_cfg_profile_req_t req = make_fhss_req();
    EXPECT(host_cfg_profile_stage(&req, true) == HOST_CFG_PROFILE_REJECT_NONE,
           "stage FHSS req succeeds when routed");
    EXPECT(host_cfg_profile_has_stage(), "stage flag set after successful stage");
    EXPECT(host_cfg_profile_active()->profile_id == REG_PROFILE_BENCH_ONLY_FIXED_915,
           "active unchanged after stage");

    EXPECT(host_cfg_profile_activate() == HOST_CFG_PROFILE_REJECT_NONE,
           "activate succeeds");
    EXPECT(!host_cfg_profile_has_stage(), "stage cleared after activate");
    EXPECT(host_cfg_profile_active()->profile_id == REG_PROFILE_FCC_15_247_FHSS_50CH_BW250,
           "active = FHSS after activate");
}

static void test_activate_without_stage(void) {
    host_cfg_profile_req_t initial = make_bench_req();
    host_cfg_profile_reset(&initial);
    EXPECT(host_cfg_profile_activate() == HOST_CFG_PROFILE_REJECT_NOT_STAGED,
           "activate without stage returns NOT_STAGED");
    EXPECT(host_cfg_profile_active()->profile_id == REG_PROFILE_BENCH_ONLY_FIXED_915,
           "active unchanged when activate fails");
}

static void test_failed_stage_clears_prior_stage(void) {
    host_cfg_profile_req_t initial = make_bench_req();
    host_cfg_profile_reset(&initial);

    host_cfg_profile_req_t good = make_fhss_req();
    EXPECT(host_cfg_profile_stage(&good, true) == HOST_CFG_PROFILE_REJECT_NONE,
           "first stage succeeds");
    EXPECT(host_cfg_profile_has_stage(), "stage flag set");

    host_cfg_profile_req_t bad = make_fhss_req();
    bad.modem_bw_hz = 125000UL;
    EXPECT(host_cfg_profile_stage(&bad, true) == HOST_CFG_PROFILE_REJECT_BW_MISMATCH,
           "second stage with bad BW rejected");
    EXPECT(!host_cfg_profile_has_stage(), "failed stage clears prior in-flight stage");
    EXPECT(host_cfg_profile_active()->profile_id == REG_PROFILE_BENCH_ONLY_FIXED_915,
           "active untouched after failed stage");

    EXPECT(host_cfg_profile_activate() == HOST_CFG_PROFILE_REJECT_NOT_STAGED,
           "activate after failed stage returns NOT_STAGED");
}

static void test_cancel_stage(void) {
    host_cfg_profile_req_t initial = make_bench_req();
    host_cfg_profile_reset(&initial);

    host_cfg_profile_req_t req = make_fhss_req();
    EXPECT(host_cfg_profile_stage(&req, true) == HOST_CFG_PROFILE_REJECT_NONE,
           "stage succeeds");
    host_cfg_profile_cancel_stage();
    EXPECT(!host_cfg_profile_has_stage(), "cancel clears stage");
    EXPECT(host_cfg_profile_activate() == HOST_CFG_PROFILE_REJECT_NOT_STAGED,
           "activate after cancel returns NOT_STAGED");
}

static void test_double_activate(void) {
    host_cfg_profile_req_t initial = make_bench_req();
    host_cfg_profile_reset(&initial);

    host_cfg_profile_req_t req = make_fhss_req();
    EXPECT(host_cfg_profile_stage(&req, true) == HOST_CFG_PROFILE_REJECT_NONE,
           "stage OK");
    EXPECT(host_cfg_profile_activate() == HOST_CFG_PROFILE_REJECT_NONE,
           "first activate OK");
    EXPECT(host_cfg_profile_activate() == HOST_CFG_PROFILE_REJECT_NOT_STAGED,
           "second activate returns NOT_STAGED");
    EXPECT(host_cfg_profile_active()->profile_id == REG_PROFILE_FCC_15_247_FHSS_50CH_BW250,
           "active still FHSS after second activate");
}

static void test_stage_unrouted_leaves_active_alone(void) {
    host_cfg_profile_req_t initial = make_bench_req();
    host_cfg_profile_reset(&initial);

    host_cfg_profile_req_t req = make_fhss_req();
    EXPECT(host_cfg_profile_stage(&req, false) == HOST_CFG_PROFILE_REJECT_UNROUTED,
           "stage FHSS rejected when tx_routed=false");
    EXPECT(!host_cfg_profile_has_stage(), "no stage after UNROUTED");
    EXPECT(host_cfg_profile_active()->profile_id == REG_PROFILE_BENCH_ONLY_FIXED_915,
           "active untouched after UNROUTED");
}

/* -------- FCC-EVID-D6-2-a-i: default BW + reject->status mappers -------- */

static void test_default_bw_hz(void) {
    EXPECT(host_cfg_profile_default_bw_hz(REG_PROFILE_BENCH_ONLY_FIXED_915) ==
               HOST_CFG_PROFILE_BENCH_BW_HZ,
           "default bw bench = 125000");
    EXPECT(host_cfg_profile_default_bw_hz(REG_PROFILE_FCC_15_247_FHSS_50CH_BW250) ==
               HOST_CFG_PROFILE_FHSS_BW_HZ,
           "default bw FHSS = 250000");
    EXPECT(host_cfg_profile_default_bw_hz(REG_PROFILE_FCC_15_247_DTS_BW500) ==
               HOST_CFG_PROFILE_DTS_BW_HZ,
           "default bw DTS = 500000");
    EXPECT(host_cfg_profile_default_bw_hz(0xFFU) == 0UL,
           "default bw unknown profile = 0");
}

static void test_reject_to_cfg_status(void) {
    /* Golden vector: pins the wire-status byte for every REJECT enum. */
    EXPECT(host_cfg_profile_reject_to_cfg_status(HOST_CFG_PROFILE_REJECT_NONE) ==
               CFG_STATUS_OK,
           "REJECT_NONE -> CFG_STATUS_OK (0)");
    EXPECT(host_cfg_profile_reject_to_cfg_status(HOST_CFG_PROFILE_REJECT_BAD_PROFILE) ==
               CFG_STATUS_OUT_OF_RANGE,
           "REJECT_BAD_PROFILE -> CFG_STATUS_OUT_OF_RANGE (3) [alias]");
    EXPECT(host_cfg_profile_reject_to_cfg_status(HOST_CFG_PROFILE_REJECT_UNROUTED) ==
               CFG_STATUS_PROFILE_UNROUTED,
           "REJECT_UNROUTED -> CFG_STATUS_PROFILE_UNROUTED (7) [alias]");
    EXPECT(host_cfg_profile_reject_to_cfg_status(HOST_CFG_PROFILE_REJECT_MASK_POPCOUNT) ==
               CFG_STATUS_PROFILE_REJECT_MASK_POPCOUNT,
           "REJECT_MASK_POPCOUNT -> 8");
    EXPECT((uint8_t)CFG_STATUS_PROFILE_REJECT_MASK_POPCOUNT == 8U,
           "wire value MASK_POPCOUNT pinned at 8");
    EXPECT(host_cfg_profile_reject_to_cfg_status(HOST_CFG_PROFILE_REJECT_MASK_OUT_OF_TABLE) ==
               CFG_STATUS_PROFILE_REJECT_MASK_OUT_OF_TABLE,
           "REJECT_MASK_OUT_OF_TABLE -> 9");
    EXPECT((uint8_t)CFG_STATUS_PROFILE_REJECT_MASK_OUT_OF_TABLE == 9U,
           "wire value MASK_OUT_OF_TABLE pinned at 9");
    EXPECT(host_cfg_profile_reject_to_cfg_status(HOST_CFG_PROFILE_REJECT_BW_MISMATCH) ==
               CFG_STATUS_PROFILE_REJECT_BW_MISMATCH,
           "REJECT_BW_MISMATCH -> 10");
    EXPECT((uint8_t)CFG_STATUS_PROFILE_REJECT_BW_MISMATCH == 10U,
           "wire value BW_MISMATCH pinned at 10");
    EXPECT(host_cfg_profile_reject_to_cfg_status(HOST_CFG_PROFILE_REJECT_ANTENNA_OUT_OF_RANGE) ==
               CFG_STATUS_PROFILE_REJECT_ANTENNA_OUT_OF_RANGE,
           "REJECT_ANTENNA_OUT_OF_RANGE -> 11");
    EXPECT((uint8_t)CFG_STATUS_PROFILE_REJECT_ANTENNA_OUT_OF_RANGE == 11U,
           "wire value ANTENNA_OUT_OF_RANGE pinned at 11");
    EXPECT(host_cfg_profile_reject_to_cfg_status(HOST_CFG_PROFILE_REJECT_NO_POWER_HEADROOM) ==
               CFG_STATUS_PROFILE_REJECT_NO_POWER_HEADROOM,
           "REJECT_NO_POWER_HEADROOM -> 12");
    EXPECT((uint8_t)CFG_STATUS_PROFILE_REJECT_NO_POWER_HEADROOM == 12U,
           "wire value NO_POWER_HEADROOM pinned at 12");
    EXPECT(host_cfg_profile_reject_to_cfg_status(HOST_CFG_PROFILE_REJECT_NOT_STAGED) ==
               CFG_STATUS_PROFILE_REJECT_NOT_STAGED,
           "REJECT_NOT_STAGED -> 13");
    EXPECT((uint8_t)CFG_STATUS_PROFILE_REJECT_NOT_STAGED == 13U,
           "wire value NOT_STAGED pinned at 13");
    EXPECT(host_cfg_profile_reject_to_cfg_status(HOST_CFG_PROFILE_REJECT_NULL_ARG) ==
               CFG_STATUS_PROFILE_REJECT_NULL_ARG,
           "REJECT_NULL_ARG -> 14");
    EXPECT((uint8_t)CFG_STATUS_PROFILE_REJECT_NULL_ARG == 14U,
           "wire value NULL_ARG pinned at 14");
}

int main(void) {
    /* validator */
    test_validate_bench_ok();
    test_validate_bad_profile_id();
    test_validate_null_arg();
    test_validate_fhss_unrouted_rejected();
    test_validate_dts_unrouted_rejected();
    test_validate_fhss_ok_when_routed();
    test_validate_fhss_mask_short();
    test_validate_fhss_mask_out_of_table();
    test_validate_fhss_bw_mismatch();
    test_validate_dts_ok_when_routed();
    test_validate_dts_bw_mismatch();
    test_validate_dts_zero_mask();
    test_validate_antenna_negative();
    test_validate_antenna_too_high();
    test_validate_power_no_headroom();

    /* power clamp */
    test_power_clamp_no_reduction();
    test_power_clamp_reduction();
    test_power_clamp_negative_gain();
    test_power_clamp_no_headroom();

    /* tier lookup */
    test_tier_ceiling_lookup();

    /* two-phase commit */
    test_stage_activate_happy_path();
    test_activate_without_stage();
    test_failed_stage_clears_prior_stage();
    test_cancel_stage();
    test_double_activate();
    test_stage_unrouted_leaves_active_alone();

    /* FCC-EVID-D6-2-a-i: default BW + reject->status mappers */
    test_default_bw_hz();
    test_reject_to_cfg_status();

    printf("[PASS] cfg_profile: 27 cases\n");
    return 0;
}
