/*
 * FCC-EVID-D6-2-a-iii: wire-level coverage of the cfg_set(REG_PROFILE)
 * dispatcher introduced in host/host_cfg.c. This TU drives cfg_set
 * directly (no SerialRPC framing — that surface is covered by the
 * D6-2-b Python orchestrator) and pins:
 *
 *   1. Each wire-reachable cfg_status_t produced by the dispatcher
 *      (one case per reject class plus happy paths). Reject classes
 *      that are wire-unreachable by construction (BW_MISMATCH,
 *      NOT_STAGED, NULL_ARG; plus MASK_POPCOUNT for BENCH/DTS because
 *      the CFG_KEY_FHSS_CHANNEL_MASK wire validator rejects mask==0)
 *      are documented in the case-comment block but not asserted
 *      here; the host-API TU (bench/host_proto/cfg_profile.c) covers
 *      them via direct host_cfg_profile_validate() calls.
 *
 *   2. The two-phase commit observable side-effect: a successful
 *      cfg_set(REG_PROFILE) makes the new profile id visible via
 *      cfg_get(REG_PROFILE), and a failed set leaves the prior
 *      active profile + the stored REG_PROFILE byte untouched.
 *
 * Built with -DLIFETRAC_FHSS_TX_ROUTED so the dispatcher exercises
 * the production routed paths (FHSS / DTS happy + reject classes).
 * The unrouted gate is covered by cfg_contract.c
 * (reg_profile_fcc_fhss_unrouted / reg_profile_fcc_dts_unrouted)
 * which builds without that define.
 */

#include "config.h"
#include "host_cfg.h"
#include "host_cfg_keys.h"
#include "host_cfg_profile.h"
#include "host_types.h"
#include "sx1276_stub.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t g_failures;

#define EXPECT(cond, msg)                                                   \
    do {                                                                    \
        if (!(cond)) {                                                      \
            printf("[FAIL] %s: %s (line %d)\n", __func__, (msg), __LINE__); \
            g_failures++;                                                   \
        }                                                                   \
    } while (0)

static void write_u64_le(uint8_t *out, uint64_t v) {
    for (uint32_t i = 0U; i < 8U; ++i) {
        out[i] = (uint8_t)((v >> (i * 8U)) & 0xFFU);
    }
}

static cfg_status_t set_channel_mask(uint64_t mask) {
    uint8_t bytes[8];
    write_u64_le(bytes, mask);
    return cfg_set(CFG_KEY_FHSS_CHANNEL_MASK, bytes, 8U);
}

static cfg_status_t set_antenna_gain(int8_t gain) {
    uint8_t b = (uint8_t)gain;
    return cfg_set(CFG_KEY_ANTENNA_GAIN_DBI, &b, 1U);
}

static cfg_status_t set_reg_profile(uint8_t profile) {
    return cfg_set(CFG_KEY_REG_PROFILE, &profile, 1U);
}

static uint8_t get_reg_profile_or_0xFF(void) {
    uint8_t v = 0xFFU;
    uint8_t got_len = 0U;
    cfg_status_t s = cfg_get(CFG_KEY_REG_PROFILE, &v, 1U, &got_len);
    if (s != CFG_STATUS_OK || got_len != 1U) {
        return 0xFFU;
    }
    return v;
}

static void test_bench_happy_default(void) {
    cfg_init();
    sx1276_stub_reset();

    EXPECT(set_reg_profile(REG_PROFILE_BENCH_ONLY_FIXED_915) == CFG_STATUS_OK,
           "bench should accept default cfg state");
    EXPECT(get_reg_profile_or_0xFF() == REG_PROFILE_BENCH_ONLY_FIXED_915,
           "cfg_get reflects activated bench profile");
    EXPECT(host_cfg_profile_active() != NULL &&
           host_cfg_profile_active()->profile_id == REG_PROFILE_BENCH_ONLY_FIXED_915,
           "profile state machine activated bench");
}

static void test_fhss_happy_when_routed(void) {
    cfg_init();
    sx1276_stub_reset();

    EXPECT(set_channel_mask(HOST_CFG_PROFILE_FHSS_50CH_REQUIRED_MASK) == CFG_STATUS_OK,
           "set required 50-ch mask");
    EXPECT(set_reg_profile(REG_PROFILE_FCC_15_247_FHSS_50CH_BW250) == CFG_STATUS_OK,
           "FHSS profile accepted when routed + mask matches");
    EXPECT(get_reg_profile_or_0xFF() == REG_PROFILE_FCC_15_247_FHSS_50CH_BW250,
           "cfg_get reflects FHSS profile");
    EXPECT(host_cfg_profile_active()->profile_id == REG_PROFILE_FCC_15_247_FHSS_50CH_BW250,
           "state machine swapped to FHSS");
    EXPECT(host_cfg_profile_active()->modem_bw_hz == HOST_CFG_PROFILE_FHSS_BW_HZ,
           "FHSS synthesised BW matches profile default");
    EXPECT(sx1276_stub_last_bw_khz() == 250,
           "modem programmed to 250 kHz on FHSS activation");
}

static void test_dts_happy_when_routed(void) {
    cfg_init();
    sx1276_stub_reset();

    EXPECT(set_reg_profile(REG_PROFILE_FCC_15_247_DTS_BW500) == CFG_STATUS_OK,
           "DTS profile accepted when routed (default mask is non-zero)");
    EXPECT(get_reg_profile_or_0xFF() == REG_PROFILE_FCC_15_247_DTS_BW500,
           "cfg_get reflects DTS profile");
    EXPECT(host_cfg_profile_active()->modem_bw_hz == HOST_CFG_PROFILE_DTS_BW_HZ,
           "DTS synthesised BW matches profile default");
    EXPECT(sx1276_stub_last_bw_khz() == 500,
           "modem programmed to 500 kHz on DTS activation");
}

static void test_profile_unknown_oor(void) {
    cfg_init();
    sx1276_stub_reset();

    EXPECT(set_reg_profile((uint8_t)(REG_PROFILE_MAX + 1U)) == CFG_STATUS_OUT_OF_RANGE,
           "unknown profile id short-circuits to OUT_OF_RANGE");
    EXPECT(get_reg_profile_or_0xFF() == REG_PROFILE_BENCH_ONLY_FIXED_915,
           "failed set leaves stored profile at bench default");
    EXPECT(host_cfg_profile_active()->profile_id == REG_PROFILE_BENCH_ONLY_FIXED_915,
           "failed set leaves active profile alone");
}

static void test_fhss_mask_popcount(void) {
    cfg_init();
    sx1276_stub_reset();

    EXPECT(set_channel_mask(0x1ULL) == CFG_STATUS_OK,
           "tiny non-zero mask accepted by wire");
    EXPECT(set_reg_profile(REG_PROFILE_FCC_15_247_FHSS_50CH_BW250) ==
               CFG_STATUS_PROFILE_REJECT_MASK_POPCOUNT,
           "FHSS rejects popcount<50");
    EXPECT(get_reg_profile_or_0xFF() == REG_PROFILE_BENCH_ONLY_FIXED_915,
           "failed FHSS set leaves stored profile alone");
}

static void test_fhss_mask_out_of_table(void) {
    cfg_init();
    sx1276_stub_reset();

    /* Bit 50 is outside the certified 0..49 range. */
    EXPECT(set_channel_mask((1ULL << 50) | 0x1ULL) == CFG_STATUS_OK,
           "out-of-table mask accepted by wire");
    EXPECT(set_reg_profile(REG_PROFILE_FCC_15_247_FHSS_50CH_BW250) ==
               CFG_STATUS_PROFILE_REJECT_MASK_OUT_OF_TABLE,
           "FHSS rejects mask bits outside certified table");
}

static void test_antenna_out_of_range(void) {
    cfg_init();
    sx1276_stub_reset();

    EXPECT(set_antenna_gain((int8_t)31) == CFG_STATUS_OK,
           "wire accepts antenna gain above FCC max");
    EXPECT(set_reg_profile(REG_PROFILE_BENCH_ONLY_FIXED_915) ==
               CFG_STATUS_PROFILE_REJECT_ANTENNA_OUT_OF_RANGE,
           "profile validator rejects antenna gain > 30");
    EXPECT(get_reg_profile_or_0xFF() == REG_PROFILE_BENCH_ONLY_FIXED_915,
           "stored profile unchanged on antenna reject");
    EXPECT(host_cfg_profile_active()->antenna_gain_dBi == (int8_t)2,
           "active profile gain not corrupted by failed stage");
}

static void test_no_power_headroom(void) {
    cfg_init();
    sx1276_stub_reset();

    /*
     * gain=30: reduction = max(0, 30-6) = 24
     * allowed = min(tier(17) - 24, hw(17)) = min(-7, 17) = -7
     * < MIN_DBM(2) -> NO_POWER_HEADROOM.
     */
    EXPECT(set_antenna_gain((int8_t)30) == CFG_STATUS_OK,
           "wire accepts antenna gain 30");
    EXPECT(set_reg_profile(REG_PROFILE_BENCH_ONLY_FIXED_915) ==
               CFG_STATUS_PROFILE_REJECT_NO_POWER_HEADROOM,
           "bench tier(17) - gain reduction(24) leaves no headroom");
}

static void test_reapply_same_profile_is_ok(void) {
    cfg_init();
    sx1276_stub_reset();

    EXPECT(set_reg_profile(REG_PROFILE_BENCH_ONLY_FIXED_915) == CFG_STATUS_OK,
           "first bench apply");
    EXPECT(set_reg_profile(REG_PROFILE_BENCH_ONLY_FIXED_915) == CFG_STATUS_OK,
           "re-applying same profile is idempotent");
}

int main(void) {
    test_bench_happy_default();
    test_fhss_happy_when_routed();
    test_dts_happy_when_routed();
    test_profile_unknown_oor();
    test_fhss_mask_popcount();
    test_fhss_mask_out_of_table();
    test_antenna_out_of_range();
    test_no_power_headroom();
    test_reapply_same_profile_is_ok();

    if (g_failures != 0U) {
        printf("[FAIL] cfg_profile_wire: %lu failures\n",
               (unsigned long)g_failures);
        return 1;
    }
    printf("[PASS] cfg_profile_wire: 9 cases\n");
    return 0;
}
