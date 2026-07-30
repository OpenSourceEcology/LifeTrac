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
    EXPECT(sx1276_stub_last_budget_us() == 950000UL,
           "D4: DTS activation widens the QoS budget to 950 ms/s");
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

/*
 * ---------------------------------------------------------------
 * FCC §A5 ERP enforcement (2026-07-29)
 *
 * Before this fix host_cfg_profile_power_clamp() was computed once per
 * staging attempt, compared against 0 to accept or reject the profile,
 * and then discarded — host_cfg_profile.c contained ZERO calls to
 * sx1276_set_tx_power_dbm(). Declaring antenna gain therefore did not
 * reduce conducted power, and CFG_KEY_TX_POWER_DBM could be written up
 * to 17 dBm afterwards with cfg_get showing nothing anomalous.
 *
 * These cases pin the regulatory property in hardware terms: what the
 * PA is actually programmed to.
 * ---------------------------------------------------------------
 */

static cfg_status_t set_hw_ceiling(uint8_t dbm) {
    return cfg_set(CFG_KEY_HW_CEILING_DBM, &dbm, 1U);
}

static cfg_status_t set_tx_power(uint8_t dbm) {
    return cfg_set(CFG_KEY_TX_POWER_DBM, &dbm, 1U);
}

static uint8_t get_tx_power_or_0xFF(void) {
    uint8_t v = 0xFFU;
    uint8_t got_len = 0U;
    cfg_status_t s = cfg_get(CFG_KEY_TX_POWER_DBM, &v, 1U, &got_len);
    if (s != CFG_STATUS_OK || got_len != 1U) {
        return 0xFFU;
    }
    return v;
}

/* Activation must program the PA at all — the core omission. */
static void test_erp_activation_programs_pa(void) {
    cfg_init();
    sx1276_stub_reset();

    /*
     * hw ceiling 10 -> bench allowance min(17 - 0, 10) = 10, below the
     * 14 dBm cfg default, so activation must LOWER the PA to 10.
     */
    EXPECT(set_hw_ceiling(10U) == CFG_STATUS_OK, "hw ceiling 10 accepted at wire");
    EXPECT(set_reg_profile(REG_PROFILE_BENCH_ONLY_FIXED_915) == CFG_STATUS_OK,
           "bench activates with a reduced hw ceiling");
    EXPECT(sx1276_stub_tx_power_call_count() >= 1U,
           "activation MUST program the PA (pre-fix it never did)");
    EXPECT(sx1276_stub_last_tx_power_dbm() == 10U,
           "PA must be clamped to the ERP allowance, not left at 14");
    EXPECT(get_tx_power_or_0xFF() == 10U,
           "cfg_get must report the clamped value, not the stale 14");
}

/* The headline regulatory property: declared gain reduces real power. */
static void test_erp_antenna_gain_reduces_conducted_power(void) {
    cfg_init();
    sx1276_stub_reset();

    /* 12 dBi -> reduction 12-6 = 6 -> bench allowance 17-6 = 11. */
    EXPECT(set_antenna_gain(12) == CFG_STATUS_OK, "12 dBi accepted at wire");
    EXPECT(set_reg_profile(REG_PROFILE_BENCH_ONLY_FIXED_915) == CFG_STATUS_OK,
           "bench activates at 12 dBi");
    EXPECT(sx1276_stub_last_tx_power_dbm() == 11U,
           "declaring 12 dBi must cut conducted power to 11 dBm");
}

/* The exceedance route that was reachable before the fix. */
static void test_erp_tx_power_write_cannot_exceed_allowance(void) {
    cfg_init();
    sx1276_stub_reset();

    EXPECT(set_hw_ceiling(10U) == CFG_STATUS_OK, "hw ceiling 10");
    EXPECT(set_reg_profile(REG_PROFILE_BENCH_ONLY_FIXED_915) == CFG_STATUS_OK,
           "activate at allowance 10");

    /*
     * Normalised, not rejected — this key's contract is that 0 -> 2 and
     * 30 -> 17 are both CFG_STATUS_OK, so the ERP clamp follows suit
     * rather than introducing a new reject class on a live wire.
     */
    EXPECT(set_tx_power(17U) == CFG_STATUS_OK,
           "over-allowance write is clamped, not rejected");
    EXPECT(sx1276_stub_last_tx_power_dbm() == 10U,
           "17 dBm must not reach the PA when the allowance is 10");
    EXPECT(get_tx_power_or_0xFF() == 10U,
           "readback must show the clamped value");
}

/* The clamp is a ceiling, not a target: operator intent survives. */
static void test_erp_activation_does_not_raise_reduced_power(void) {
    cfg_init();
    sx1276_stub_reset();

    EXPECT(set_tx_power(5U) == CFG_STATUS_OK, "operator drops to 5 dBm");
    /* Bench allowance is the full 17 here, so there is headroom to raise. */
    EXPECT(set_reg_profile(REG_PROFILE_BENCH_ONLY_FIXED_915) == CFG_STATUS_OK,
           "activate with allowance 17");
    EXPECT(sx1276_stub_last_tx_power_dbm() == 5U,
           "activation must not raise a deliberately-reduced setting");
}

/*
 * ---------------------------------------------------------------
 * FHSS seed identity (2026-07-29)
 *
 * sx1276_fhss_init() was called with a hardcoded (0, 0, 0) at its only
 * production call site, so every radio ever built derived the same
 * 50-channel permutation.
 * ---------------------------------------------------------------
 */

static cfg_status_t set_fhss_farm_id(uint64_t v) {
    uint8_t bytes[8];
    write_u64_le(bytes, v);
    return cfg_set(CFG_KEY_FHSS_FARM_ID, bytes, 8U);
}

static cfg_status_t set_fhss_node_id(uint64_t v) {
    uint8_t bytes[8];
    write_u64_le(bytes, v);
    return cfg_set(CFG_KEY_FHSS_NODE_ID, bytes, 8U);
}

static void test_fhss_seed_reaches_scheduler(void) {
    cfg_init();
    sx1276_stub_reset();
    sx1276_stub_fhss_capture_reset();

    EXPECT(set_channel_mask(HOST_CFG_PROFILE_FHSS_50CH_REQUIRED_MASK) == CFG_STATUS_OK,
           "required 50-ch mask");
    EXPECT(set_fhss_farm_id(0x1122334455667788ULL) == CFG_STATUS_OK, "farm id accepted");
    EXPECT(set_fhss_node_id(0xAABBCCDDEEFF0011ULL) == CFG_STATUS_OK, "node id accepted");
    EXPECT(set_reg_profile(REG_PROFILE_FCC_15_247_FHSS_50CH_BW250) == CFG_STATUS_OK,
           "FHSS activates");

    EXPECT(sx1276_stub_fhss_init_call_count() == 1U, "activation inits the scheduler once");
    EXPECT(sx1276_stub_last_fhss_farm_id() == 0x1122334455667788ULL,
           "farm_id must reach sx1276_fhss_init, not a hardcoded 0");
    EXPECT(sx1276_stub_last_fhss_node_id() == 0xAABBCCDDEEFF0011ULL,
           "node_id must reach sx1276_fhss_init, not a hardcoded 0");
    EXPECT(sx1276_stub_last_fhss_initial_epoch() == 0U,
           "initial_epoch stays 0 — it is a throwaway phase origin, not identity");
}

/* Un-provisioned fleet must keep the historical permutation exactly. */
static void test_fhss_seed_defaults_preserve_legacy_zero(void) {
    cfg_init();
    sx1276_stub_reset();
    sx1276_stub_fhss_capture_reset();

    EXPECT(set_channel_mask(HOST_CFG_PROFILE_FHSS_50CH_REQUIRED_MASK) == CFG_STATUS_OK,
           "required 50-ch mask");
    EXPECT(set_reg_profile(REG_PROFILE_FCC_15_247_FHSS_50CH_BW250) == CFG_STATUS_OK,
           "FHSS activates without provisioning a seed");

    EXPECT(sx1276_stub_last_fhss_farm_id() == 0ULL &&
           sx1276_stub_last_fhss_node_id() == 0ULL,
           "default seed must stay (0,0) so an un-provisioned fleet is unchanged");
}

/* The ORDERING CONTRACT in host_cfg_keys.h, pinned. */
static void test_fhss_seed_written_after_activation_is_not_live(void) {
    cfg_init();
    sx1276_stub_reset();

    EXPECT(set_channel_mask(HOST_CFG_PROFILE_FHSS_50CH_REQUIRED_MASK) == CFG_STATUS_OK,
           "required 50-ch mask");
    EXPECT(set_reg_profile(REG_PROFILE_FCC_15_247_FHSS_50CH_BW250) == CFG_STATUS_OK,
           "FHSS activates with the default seed");

    sx1276_stub_fhss_capture_reset();
    EXPECT(set_fhss_node_id(0xDEADBEEFULL) == CFG_STATUS_OK,
           "seed key accepts a late write");
    EXPECT(sx1276_stub_fhss_init_call_count() == 0U,
           "a late seed write must NOT re-init the scheduler — "
           "the seed is consumed only at activation, so hosts must "
           "write it BEFORE CFG_KEY_REG_PROFILE");
}

static void test_fhss_seed_rejects_wrong_length(void) {
    cfg_init();

    uint8_t short_val[4] = { 1U, 2U, 3U, 4U };
    EXPECT(cfg_set(CFG_KEY_FHSS_FARM_ID, short_val, 4U) == CFG_STATUS_BAD_LENGTH,
           "farm id is exactly 8 bytes");
    EXPECT(cfg_set(CFG_KEY_FHSS_NODE_ID, short_val, 4U) == CFG_STATUS_BAD_LENGTH,
           "node id is exactly 8 bytes");
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

    /* FCC §A5 ERP enforcement (2026-07-29) */
    test_erp_activation_programs_pa();
    test_erp_antenna_gain_reduces_conducted_power();
    test_erp_tx_power_write_cannot_exceed_allowance();
    test_erp_activation_does_not_raise_reduced_power();

    /* FHSS seed identity (2026-07-29) */
    test_fhss_seed_reaches_scheduler();
    test_fhss_seed_defaults_preserve_legacy_zero();
    test_fhss_seed_written_after_activation_is_not_live();
    test_fhss_seed_rejects_wrong_length();

    if (g_failures != 0U) {
        printf("[FAIL] cfg_profile_wire: %lu failures\n",
               (unsigned long)g_failures);
        return 1;
    }
    printf("[PASS] cfg_profile_wire: 17 cases\n");
    return 0;
}
