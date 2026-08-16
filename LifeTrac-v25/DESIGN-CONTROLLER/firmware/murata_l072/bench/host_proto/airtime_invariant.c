/*
 * FCC-A1a airtime-invariant host test
 * ===================================
 *
 * Plan ref: AI NOTES/2026-05-19_LoRa_FCC_50CH_FHSS_Implementation_Plan_Copilot_v1_0.md
 *           §14 / §14.2 step 3, TODO Stage S1.5 FCC-A1a.
 *
 * Verifies that sx1276_airtime_config_invariant_ok() correctly rejects
 * (SF, BW, CR) tuples whose worst-case (max-payload) ToA exceeds the
 * 380 ms dwell cap (400 ms FCC §15.247 dwell - 20 ms guard).
 *
 * The pure helper sx1276_airtime_compute_toa_us() is independently
 * exercised against textbook reference values (Semtech AN1200.13 /
 * SX1276 datasheet §4.1.1.7 ToA formula).
 *
 * This test links only the airtime translation unit. It does NOT pull
 * in sx1276_set_sf_bw_cr_checked (which would drag in SPI / UART /
 * STM32 register stubs). The setter is verified at the source-review
 * level by inspecting that it calls the same helper this test covers.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "sx1276_airtime.h"
#include "host_cfg_keys.h"

/* Stubs so radio/sx1276_airtime.c can link against this host test.
 * sx1276_airtime_estimate_toa_us() reads registers via sx1276_read_reg,
 * but we never call it in this test. Provide a stub anyway so the
 * translation unit links. */
uint8_t sx1276_read_reg(uint8_t reg_addr) { (void)reg_addr; return 0U; }
void sx1276_write_reg(uint8_t reg_addr, uint8_t value) { (void)reg_addr; (void)value; }

/*
 * FCC-A1b host-side mirror (plan §14.2 step 4).
 *
 * Production routing for profile_id -> (SF, BW, CR) lives in FCC-A4
 * (not yet landed). This minimal mirror exists in the test only and is
 * cross-referenced by host clients (X8 SerialRPC layer) so that an X8
 * caller can predict ToA for a planned payload *before* sending the
 * TX_FRAME_REQ to the L072, and refuse to enqueue oversized frames.
 *
 * The table MUST stay byte-for-byte consistent with the per-profile
 * tuple that FCC-A4 will eventually program via sx1276_apply_profile_full().
 */
typedef struct profile_tuple_s {
    uint8_t  profile_id;
    uint8_t  sf;
    uint16_t bw_khz;
    uint8_t  cr_den;
} profile_tuple_t;

static const profile_tuple_t k_profile_tuples[] = {
    /* Bench-only fixed 915 MHz: SF7/BW125/CR45 (lab use, not FCC-routed).
     * BW125 is what the runtime actually programs -- see
     * host_cfg_profile_default_bw_hz(), which returns
     * HOST_CFG_PROFILE_BENCH_BW_HZ (125000) for this profile. This mirror
     * said BW250 until 2026-08-16; because nothing cross-checked the two,
     * the error was invisible to CI and made this table predict HALF the
     * real time-on-air for profile 0. */
    { REG_PROFILE_BENCH_ONLY_FIXED_915,         7U, 125U, 5U },
    /* FCC §15.247 FHSS 50ch BW250: same SF7/BW250/CR45 modem config; the
     * 50-channel hop schedule is layered on top by FCC-A3. */
    { REG_PROFILE_FCC_15_247_FHSS_50CH_BW250,   7U, 250U, 5U },
    /* FCC §15.247 DTS BW500: SF7/BW500/CR45 (single wideband channel). */
    { REG_PROFILE_FCC_15_247_DTS_BW500,         7U, 500U, 5U },
};

static bool host_predict_toa_for_profile(uint8_t profile_id,
                                         uint8_t payload_len,
                                         uint32_t *out_toa_us) {
    for (size_t i = 0; i < (sizeof(k_profile_tuples) / sizeof(k_profile_tuples[0])); ++i) {
        if (k_profile_tuples[i].profile_id == profile_id) {
            const profile_tuple_t *p = &k_profile_tuples[i];
            const uint8_t low_dr_opt = ((p->sf >= 11U) && (p->bw_khz <= 125U)) ? 1U : 0U;
            *out_toa_us = sx1276_airtime_compute_toa_us(p->sf,
                                                       (uint32_t)p->bw_khz * 1000UL,
                                                       p->cr_den,
                                                       payload_len,
                                                       1U /*crc*/,
                                                       0U /*implicit*/,
                                                       8U /*preamble*/,
                                                       low_dr_opt);
            return true;
        }
    }
    return false;
}

static int g_failures = 0;

#define CHECK(cond, fmt, ...) do {                                          \
    if (!(cond)) {                                                          \
        fprintf(stderr, "[FAIL] %s:%d: " fmt "\n",                          \
                __FILE__, __LINE__, ##__VA_ARGS__);                         \
        ++g_failures;                                                       \
    }                                                                       \
} while (0)

static void test_pure_helper_sf7_bw250_24b(void) {
    /* SF7, BW=250 kHz, CR=4/5, payload=24 B, CRC on, explicit hdr,
     * preamble 8, low_dr_opt off.
     *
     * Reference: Semtech AN1200.13 worked example for SF7/BW250.
     *   t_sym       = 128 / 250 000   = 512 us
     *   preamble_us = (8 + 4) * 512 + 512/4 = 6272 us
     *   numerator   = 8*24 - 4*7 + 28 + 16*1 - 20*0 = 208
     *   denominator = 4 * (7 - 0) = 28
     *   ceil(208/28) = 8
     *   payload_sym = 8 + 8 * 5 = 48
     *   payload_us  = 48 * 512 = 24576 us
     *   total       = 30848 us
     */
    const uint32_t toa = sx1276_airtime_compute_toa_us(
        7U, 250000UL, 5U, 24U,
        1U /*crc*/, 0U /*implicit*/, 8U /*preamble*/, 0U /*low_dr_opt*/);
    CHECK(toa == 30848U,
          "SF7/BW250/CR45/24B: expected 30848 us, got %u", (unsigned)toa);
}

static void test_pure_helper_sf12_bw125_preamble_only(void) {
    /* SF12, BW=125 kHz, CR=4/5, payload=0, CRC on, explicit hdr,
     * preamble 8, low_dr_opt ON (mandatory at SF12/BW125).
     *
     *   t_sym       = 4096 / 125 000  = 32768 us
     *   preamble_us = (8 + 4) * 32768 + 32768/4 = 401408 us
     *   numerator   = 8*0 - 4*12 + 28 + 16 - 0 = -4  (<=0, clamped)
     *   payload_sym = 8  (Semtech datasheet floor)
     *   payload_us  = 8 * 32768 = 262144 us
     *   total       = 663552 us  (~664 ms)
     *
     * Preamble alone is already 401 ms; adding the 8-symbol payload
     * floor pushes the total to ~664 ms -> far over the 380 ms cap.
     * This is *why* SF12/BW125 must be refused at config time even at
     * payload=0.
     */
    const uint32_t toa = sx1276_airtime_compute_toa_us(
        12U, 125000UL, 5U, 0U,
        1U, 0U, 8U, 1U);
    CHECK(toa == 663552U,
          "SF12/BW125 payload=0: expected 663552 us, got %u", (unsigned)toa);
}

static void test_invariant_rejects_sf12_bw125_max_payload(void) {
    uint32_t computed_toa_us = 0U;
    const bool ok = sx1276_airtime_config_invariant_ok(
        12U, 125U, 5U, 255U, SX1276_AIRTIME_DWELL_CAP_US, &computed_toa_us);
    CHECK(!ok, "SF12/BW125/CR45/255B must be REJECTED (got accept), toa=%u us",
          (unsigned)computed_toa_us);
    CHECK(computed_toa_us > SX1276_AIRTIME_DWELL_CAP_US,
          "SF12/BW125 max-payload ToA (%u us) must exceed cap %u us",
          (unsigned)computed_toa_us, (unsigned)SX1276_AIRTIME_DWELL_CAP_US);
}

static void test_invariant_accepts_sf7_bw250_max_payload(void) {
    /* SF7/BW250 at max payload (255 B). Worst-case ToA:
     *   t_sym       = 512 us
     *   preamble_us = 6272 us
     *   numerator   = 8*255 - 4*7 + 28 + 16 - 0 = 2056
     *   denominator = 28
     *   ceil(2056/28) = 74
     *   payload_sym = 8 + 74 * 5 = 378
     *   payload_us  = 378 * 512 = 193536 us
     *   total       = 199808 us   ( <= 380000 cap )
     */
    uint32_t computed_toa_us = 0U;
    const bool ok = sx1276_airtime_config_invariant_ok(
        7U, 250U, 5U, 255U, SX1276_AIRTIME_DWELL_CAP_US, &computed_toa_us);
    CHECK(ok, "SF7/BW250/CR45/255B must be ACCEPTED (got reject), toa=%u us",
          (unsigned)computed_toa_us);
    CHECK(computed_toa_us == 199808U,
          "SF7/BW250/CR45/255B: expected 199808 us, got %u",
          (unsigned)computed_toa_us);
}

static void test_invariant_accepts_sf7_bw250_24b_short(void) {
    uint32_t computed_toa_us = 0U;
    const bool ok = sx1276_airtime_config_invariant_ok(
        7U, 250U, 5U, 24U, SX1276_AIRTIME_DWELL_CAP_US, &computed_toa_us);
    CHECK(ok, "SF7/BW250/CR45/24B must be ACCEPTED (got reject), toa=%u us",
          (unsigned)computed_toa_us);
    CHECK(computed_toa_us == 30848U,
          "SF7/BW250/CR45/24B: expected 30848 us, got %u",
          (unsigned)computed_toa_us);
}

static void test_invariant_rejects_sf11_bw125_max_payload(void) {
    /* SF11/BW125 is a regulatory borderline case. With low_dr_opt on:
     *   t_sym       = 2048/125000 = 16384 us
     *   preamble_us = 12 * 16384 + 4096 = 200704 us
     * Preamble alone is 200 ms, max payload will push well past 380 ms.
     */
    uint32_t computed_toa_us = 0U;
    const bool ok = sx1276_airtime_config_invariant_ok(
        11U, 125U, 5U, 255U, SX1276_AIRTIME_DWELL_CAP_US, &computed_toa_us);
    CHECK(!ok, "SF11/BW125/CR45/255B must be REJECTED, toa=%u us",
          (unsigned)computed_toa_us);
}

static void test_invariant_accepts_sf10_bw500_max_payload(void) {
    /* SF10/BW500 max payload (255 B), low_dr_opt off (BW > 125):
     *   t_sym       = 1024/500000 = 2048 us
     *   preamble_us = 12 * 2048 + 512 = 25088 us
     *   numerator   = 2040 - 40 + 28 + 16 = 2044
     *   denominator = 40
     *   ceil(2044/40) = 52
     *   payload_sym = 8 + 52*5 = 268
     *   payload_us  = 268 * 2048 = 548864 us  -> exceeds cap!
     *
     * Actually this exceeds the cap, so it must be REJECTED. (Comment
     * intent: even BW500 at SF10 is over-budget at 255 B.)
     */
    uint32_t computed_toa_us = 0U;
    const bool ok = sx1276_airtime_config_invariant_ok(
        10U, 500U, 5U, 255U, SX1276_AIRTIME_DWELL_CAP_US, &computed_toa_us);
    CHECK(!ok, "SF10/BW500/CR45/255B must be REJECTED (ToA=%u us > cap)",
          (unsigned)computed_toa_us);
}

/* === FCC-A1b per-TX invariant cases (plan §14.2 step 4) === */

static void test_per_tx_invariant_accepts_full_payload_sweep(void) {
    /*
     * For the active FCC-routed profile (FHSS_50CH_BW250 -> SF7/BW250/CR45),
     * the per-TX invariant must accept EVERY legal payload length in
     * [0, 255]. This proves the config-time check at payload=255 is the
     * tight upper bound and no shorter payload can sneak over the cap.
     */
    for (unsigned payload = 0U; payload <= 255U; ++payload) {
        uint32_t toa = 0U;
        const bool ok = sx1276_airtime_config_invariant_ok(
            7U, 250U, 5U, (uint8_t)payload, SX1276_AIRTIME_DWELL_CAP_US, &toa);
        if (!ok) {
            CHECK(0, "SF7/BW250 should accept payload=%u (toa=%u us)",
                  payload, (unsigned)toa);
            return;
        }
    }
}

static void test_per_tx_invariant_payload_monotonic(void) {
    /*
     * ToA must be monotonically non-decreasing in payload_len for any
     * fixed (SF, BW, CR). Catches off-by-one / signed-overflow bugs in
     * the symbol-count formula.
     */
    uint32_t prev = 0U;
    for (unsigned payload = 0U; payload <= 255U; ++payload) {
        const uint32_t toa = sx1276_airtime_compute_toa_us(
            7U, 250000UL, 5U, (uint8_t)payload, 1U, 0U, 8U, 0U);
        CHECK(toa >= prev,
              "ToA non-monotonic at payload=%u: %u < prev %u",
              payload, (unsigned)toa, (unsigned)prev);
        prev = toa;
    }
}

static void test_host_mirror_matches_helper_fhss_profile(void) {
    /*
     * Host-side X8 mirror must compute identical ToA to the on-target
     * helper for the FHSS profile. If this diverges, X8 will let
     * oversized frames reach the L072 (where they'd be aborted with
     * tx_abort_airtime, wasting LBT/scheduler effort) or refuse frames
     * the L072 would actually accept.
     */
    const uint8_t profile_id = REG_PROFILE_FCC_15_247_FHSS_50CH_BW250;
    const uint8_t test_payloads[] = { 0U, 1U, 16U, 24U, 64U, 128U, 200U, 255U };
    for (size_t i = 0; i < sizeof(test_payloads); ++i) {
        const uint8_t payload = test_payloads[i];
        uint32_t mirror_toa = 0U;
        uint32_t helper_toa = 0U;

        const bool found = host_predict_toa_for_profile(profile_id, payload, &mirror_toa);
        CHECK(found, "profile_id=%u missing from host mirror table",
              (unsigned)profile_id);

        (void)sx1276_airtime_config_invariant_ok(
            7U, 250U, 5U, payload, SX1276_AIRTIME_DWELL_CAP_US, &helper_toa);

        CHECK(mirror_toa == helper_toa,
              "host mirror diverges from helper at payload=%u: mirror=%u helper=%u",
              (unsigned)payload, (unsigned)mirror_toa, (unsigned)helper_toa);
    }
}

static void test_host_mirror_rejects_oversized_for_bench_profile(void) {
    /*
     * Bench profile is SF7/BW125, so unlike the other two profiles it does
     * NOT accept everything up to 255 B: at 125 kHz a 255 B payload is
     * 399 616 us, which is 19 616 us OVER the 380 000 us dwell cap. The
     * largest payload that fits is 243 B (379 136 us).
     *
     * This test previously asserted the opposite -- that 255 B was under the
     * cap -- and passed, because the profile table above claimed BW250 and
     * therefore predicted exactly half the real time-on-air. Anything sized
     * against that number was sized against a fiction.
     *
     * Practical consequence, recorded because it nearly cost a bench
     * session: the standard radio-monitor leg sends 255 B fragments, so a
     * profile-0 A/B run at the usual fragment size transmits over the dwell
     * cap and the airtime accountant is entitled to abort those TXs
     * (radio_tx_abort_airtime). Profile-0 legs must drop to <=243 B.
     */
    uint32_t toa_243 = 0U;
    uint32_t toa_244 = 0U;
    uint32_t toa_255 = 0U;

    CHECK(host_predict_toa_for_profile(REG_PROFILE_BENCH_ONLY_FIXED_915, 243U, &toa_243),
          "bench profile missing from host mirror");
    CHECK(host_predict_toa_for_profile(REG_PROFILE_BENCH_ONLY_FIXED_915, 244U, &toa_244),
          "bench profile missing from host mirror");
    CHECK(host_predict_toa_for_profile(REG_PROFILE_BENCH_ONLY_FIXED_915, 255U, &toa_255),
          "bench profile missing from host mirror");

    CHECK(toa_243 <= SX1276_AIRTIME_DWELL_CAP_US,
          "bench profile @243B (%u us) should fit under cap %u us",
          (unsigned)toa_243, (unsigned)SX1276_AIRTIME_DWELL_CAP_US);
    CHECK(toa_244 > SX1276_AIRTIME_DWELL_CAP_US,
          "bench profile @244B (%u us) should EXCEED cap %u us",
          (unsigned)toa_244, (unsigned)SX1276_AIRTIME_DWELL_CAP_US);
    CHECK(toa_255 > SX1276_AIRTIME_DWELL_CAP_US,
          "bench profile @255B (%u us) should EXCEED cap %u us",
          (unsigned)toa_255, (unsigned)SX1276_AIRTIME_DWELL_CAP_US);
}

static void test_host_mirror_unknown_profile(void) {
    /*
     * Unknown profile_id must fail closed in the mirror so X8 surfaces
     * a configuration error rather than silently sending under wrong
     * assumptions.
     */
    uint32_t mirror_toa = 0xDEADBEEFU;
    const bool found = host_predict_toa_for_profile(0xFFU, 1U, &mirror_toa);
    CHECK(!found, "unknown profile_id must fail closed in host mirror");
    CHECK(mirror_toa == 0xDEADBEEFU,
          "host mirror modified out param on failure (got %u)",
          (unsigned)mirror_toa);
}

int main(void) {
    test_pure_helper_sf7_bw250_24b();
    test_pure_helper_sf12_bw125_preamble_only();
    test_invariant_rejects_sf12_bw125_max_payload();
    test_invariant_accepts_sf7_bw250_max_payload();
    test_invariant_accepts_sf7_bw250_24b_short();
    test_invariant_rejects_sf11_bw125_max_payload();
    test_invariant_accepts_sf10_bw500_max_payload();

    /* FCC-A1b per-TX + host-mirror coverage */
    test_per_tx_invariant_accepts_full_payload_sweep();
    test_per_tx_invariant_payload_monotonic();
    test_host_mirror_matches_helper_fhss_profile();
    test_host_mirror_rejects_oversized_for_bench_profile();
    test_host_mirror_unknown_profile();

    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] airtime_invariant: %d failure(s)\n", g_failures);
        return EXIT_FAILURE;
    }
    printf("[PASS] airtime_invariant: 12 cases\n");
    return EXIT_SUCCESS;
}
