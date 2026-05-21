/*
 * FCC-EVID-D10 — exhaustive (profile, antenna_gain, hw_ceiling, mask,
 *                tx_routed) cross-product fuzz of the
 *                host_cfg_profile validator + power_clamp.
 *
 * Scope (per TODO.md FCC-EVID-D10): every illegal combination through
 * the host_cfg_profile path must be rejected with a structured reason;
 * every legal combination must pass and the power_clamp must produce
 * a result bounded by both the tier ceiling and the hw_ceiling. The
 * test is HW-free (host gcc only); it does NOT exercise the wire
 * dispatcher in host/host_cfg.c — that is already covered case-by-
 * case by FCC-EVID-D6-2-a-iii (bench/host_proto/cfg_profile_wire.c)
 * and at the wire level by FCC-EVID-D6-2-b
 * (LifeTrac-v25/tools/cfg_fuzz_profile_lock.py).
 *
 * Strategy:
 *   1. Drive a Cartesian product of carefully chosen boundary +
 *      mid-range values across each axis. Total case count is the
 *      exhaustive product (printed at the end), not a sample — so
 *      every regression in the validator's decision tree shows up as
 *      a specific (axis tuple) -> got/expected diff, not as a flaky
 *      sample miss.
 *   2. For each tuple, compute the GOLDEN expected reject reason
 *      using an independent copy of the decision-tree (predict_*()
 *      below) so we cross-check the validator's ordering of checks
 *      (NULL > BAD_PROFILE > ANTENNA > NO_POWER_HEADROOM > per-profile).
 *   3. Independently compute the GOLDEN expected power_clamp value
 *      using the §A5 ERP formula re-implemented in C from scratch
 *      and assert byte equality against host_cfg_profile_power_clamp.
 *   4. Cross-validate the wire-status mapping with
 *      host_cfg_profile_reject_to_cfg_status for every reject reason
 *      we observe.
 *   5. Assert the global invariants:
 *        a) power_clamp result <= min(tier_ceiling, hw_ceiling)
 *        b) result == 0 iff (tier - max(0,gain-6)) clamped to
 *           hw_ceiling is below HOST_CFG_PROFILE_TX_POWER_MIN_DBM
 *        c) validate() returns NO_POWER_HEADROOM iff
 *           power_clamp(tier, hw, gain) == 0 AND the request would
 *           otherwise have passed the NULL/BAD_PROFILE/ANTENNA checks
 *        d) the wire-status byte mapping is a total function over
 *           every reject reason emitted in the fuzz
 *
 * Cross-references:
 *   - TODO.md FCC-EVID-D10
 *   - AI NOTES/2026-05-19_LoRa_FCC_50CH_FHSS_Implementation_Plan_Copilot_v1_0.md §A5
 *   - host/host_cfg_profile.c (system under test)
 *   - bench/host_proto/cfg_profile.c (hand-picked unit cases)
 *   - bench/host_proto/cfg_profile_wire.c (wire dispatcher coverage)
 */

#include "host_cfg_keys.h"
#include "host_cfg_profile.h"
#include "host_cfg.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* Axis tables. Chosen to cover every decision boundary in the
 * validator + power_clamp, plus a few midspan values so a
 * sign-flip in either function shows up as multiple failing tuples
 * instead of a single edge-case miss.
 */
static const uint8_t k_profiles[] = {
    REG_PROFILE_BENCH_ONLY_FIXED_915,         /* 0 — bench */
    REG_PROFILE_FCC_15_247_FHSS_50CH_BW250,   /* 1 — FHSS routed-only */
    REG_PROFILE_FCC_15_247_DTS_BW500,         /* 2 — DTS  routed-only */
    (uint8_t)(REG_PROFILE_MAX + 1U),          /* 3 — first OOR */
    0x10U,                                    /* mid-OOR */
    0xFFU,                                    /* max u8 */
};
#define K_PROFILES_N (sizeof(k_profiles)/sizeof(k_profiles[0]))

static const int8_t k_antenna_gains[] = {
    -128,                                                  /* min i8 */
    -10, -1,                                                /* sub-zero band */
     0,  1,                                                 /* legal low */
     HOST_CFG_PROFILE_ANTENNA_GAIN_FCC_THRESHOLD_DBI,       /*  6 — threshold */
     (int8_t)(HOST_CFG_PROFILE_ANTENNA_GAIN_FCC_THRESHOLD_DBI + 1), /* 7 — first reduction */
     10, 17, 20,                                            /* mid-legal */
     HOST_CFG_PROFILE_ANTENNA_GAIN_MAX_DBI,                 /* 30 — legal max */
     (int8_t)(HOST_CFG_PROFILE_ANTENNA_GAIN_MAX_DBI + 1),   /* 31 — first OOR */
     100, 127,                                              /* high OOR */
};
#define K_GAINS_N (sizeof(k_antenna_gains)/sizeof(k_antenna_gains[0]))

static const uint8_t k_hw_ceilings[] = {
    0U,                                                /* below min */
    1U,                                                /* below min (just) */
    (uint8_t)HOST_CFG_PROFILE_TX_POWER_MIN_DBM,        /* 2 — min */
    3U, 6U,                                            /* sub-bench */
    (uint8_t)HOST_CFG_PROFILE_TIER_CEILING_BENCH_DBM,  /* 17 — bench tier */
    20U,                                               /* prod PA cap */
    25U,                                               /* between caps */
    (uint8_t)HOST_CFG_PROFILE_TIER_CEILING_FHSS_DBM,   /* 30 — FHSS/DTS tier */
    50U, 100U, 255U,                                   /* hardware ceilings the firmware can't hit */
};
#define K_HW_N (sizeof(k_hw_ceilings)/sizeof(k_hw_ceilings[0]))

/* Channel-mask shapes per profile family. We can't enumerate 2^50
 * masks; pick boundary classes. */
typedef enum mask_class_e {
    MASK_ZERO,
    MASK_ONE_BIT_LOW,           /* bit 0 set */
    MASK_FHSS_REQUIRED,         /* exactly the 50-ch required mask */
    MASK_FHSS_MISSING_ONE,      /* required mask minus bit 0 */
    MASK_FHSS_OUT_OF_TABLE,     /* required mask plus bit 50 */
    MASK_HIGH_BIT_ONLY,         /* bit 63 set */
} mask_class_t;
static const mask_class_t k_masks[] = {
    MASK_ZERO, MASK_ONE_BIT_LOW, MASK_FHSS_REQUIRED,
    MASK_FHSS_MISSING_ONE, MASK_FHSS_OUT_OF_TABLE, MASK_HIGH_BIT_ONLY,
};
#define K_MASKS_N (sizeof(k_masks)/sizeof(k_masks[0]))

static const bool k_tx_routed[] = { false, true };
#define K_ROUTED_N (sizeof(k_tx_routed)/sizeof(k_tx_routed[0]))

/* ------------------------------------------------------------------ */

static uint64_t materialize_mask(mask_class_t c) {
    switch (c) {
        case MASK_ZERO:              return 0ULL;
        case MASK_ONE_BIT_LOW:       return 0x1ULL;
        case MASK_FHSS_REQUIRED:     return HOST_CFG_PROFILE_FHSS_50CH_REQUIRED_MASK;
        case MASK_FHSS_MISSING_ONE:  return HOST_CFG_PROFILE_FHSS_50CH_REQUIRED_MASK & ~0x1ULL;
        case MASK_FHSS_OUT_OF_TABLE: return HOST_CFG_PROFILE_FHSS_50CH_REQUIRED_MASK | (1ULL << 50);
        case MASK_HIGH_BIT_ONLY:     return (1ULL << 63);
    }
    return 0ULL; /* unreachable */
}

static uint8_t golden_power_clamp(uint8_t tier, uint8_t hw, int8_t gain) {
    int reduction = 0;
    if (gain > HOST_CFG_PROFILE_ANTENNA_GAIN_FCC_THRESHOLD_DBI) {
        reduction = (int)gain - HOST_CFG_PROFILE_ANTENNA_GAIN_FCC_THRESHOLD_DBI;
    }
    int allowed = (int)tier - reduction;
    if (allowed > (int)hw) allowed = (int)hw;
    if (allowed < (int)HOST_CFG_PROFILE_TX_POWER_MIN_DBM) return 0U;
    return (uint8_t)allowed;
}

static uint8_t popcount_u64_local(uint64_t v) {
    uint8_t n = 0U;
    while (v != 0ULL) { n = (uint8_t)(n + (uint8_t)(v & 1ULL)); v >>= 1; }
    return n;
}

/* Mirror of the validator's decision tree. Kept fully independent so a
 * refactor that accidentally reorders the checks (e.g. moves the
 * antenna-range check after the per-profile checks) trips this oracle
 * for the tuples where the order matters. */
static host_cfg_profile_reject_t predict_reject(
        const host_cfg_profile_req_t *req, bool tx_routed) {
    if (req == NULL) return HOST_CFG_PROFILE_REJECT_NULL_ARG;
    if (req->profile_id > REG_PROFILE_MAX) return HOST_CFG_PROFILE_REJECT_BAD_PROFILE;
    if (req->antenna_gain_dBi < 0 ||
        req->antenna_gain_dBi > HOST_CFG_PROFILE_ANTENNA_GAIN_MAX_DBI)
        return HOST_CFG_PROFILE_REJECT_ANTENNA_OUT_OF_RANGE;

    const uint8_t tier = host_cfg_profile_tier_ceiling_dBm(req->profile_id);
    if (golden_power_clamp(tier, req->hw_ceiling_dBm,
                           req->antenna_gain_dBi) == 0U)
        return HOST_CFG_PROFILE_REJECT_NO_POWER_HEADROOM;

    switch (req->profile_id) {
        case REG_PROFILE_BENCH_ONLY_FIXED_915:
            return (req->channel_mask == 0ULL)
                ? HOST_CFG_PROFILE_REJECT_MASK_POPCOUNT
                : HOST_CFG_PROFILE_REJECT_NONE;

        case REG_PROFILE_FCC_15_247_FHSS_50CH_BW250:
            if (!tx_routed) return HOST_CFG_PROFILE_REJECT_UNROUTED;
            if ((req->channel_mask & ~HOST_CFG_PROFILE_FHSS_50CH_REQUIRED_MASK) != 0ULL)
                return HOST_CFG_PROFILE_REJECT_MASK_OUT_OF_TABLE;
            if (popcount_u64_local(req->channel_mask) < 50U)
                return HOST_CFG_PROFILE_REJECT_MASK_POPCOUNT;
            if (req->modem_bw_hz != HOST_CFG_PROFILE_FHSS_BW_HZ)
                return HOST_CFG_PROFILE_REJECT_BW_MISMATCH;
            return HOST_CFG_PROFILE_REJECT_NONE;

        case REG_PROFILE_FCC_15_247_DTS_BW500:
            if (!tx_routed) return HOST_CFG_PROFILE_REJECT_UNROUTED;
            if (req->channel_mask == 0ULL)
                return HOST_CFG_PROFILE_REJECT_MASK_POPCOUNT;
            if (req->modem_bw_hz != HOST_CFG_PROFILE_DTS_BW_HZ)
                return HOST_CFG_PROFILE_REJECT_BW_MISMATCH;
            return HOST_CFG_PROFILE_REJECT_NONE;
    }
    /* Unreachable: BAD_PROFILE check above already covered profile_id
     * out of range. Returning APPLY_FAILED here would mask a regression
     * in the BAD_PROFILE check; surface it as BAD_PROFILE instead so a
     * mismatch shows the actual culprit. */
    return HOST_CFG_PROFILE_REJECT_BAD_PROFILE;
}

static const char *reject_name(host_cfg_profile_reject_t r) {
    switch (r) {
        case HOST_CFG_PROFILE_REJECT_NONE:                 return "NONE";
        case HOST_CFG_PROFILE_REJECT_BAD_PROFILE:          return "BAD_PROFILE";
        case HOST_CFG_PROFILE_REJECT_MASK_POPCOUNT:        return "MASK_POPCOUNT";
        case HOST_CFG_PROFILE_REJECT_MASK_OUT_OF_TABLE:    return "MASK_OUT_OF_TABLE";
        case HOST_CFG_PROFILE_REJECT_BW_MISMATCH:          return "BW_MISMATCH";
        case HOST_CFG_PROFILE_REJECT_ANTENNA_OUT_OF_RANGE: return "ANTENNA_OUT_OF_RANGE";
        case HOST_CFG_PROFILE_REJECT_NO_POWER_HEADROOM:    return "NO_POWER_HEADROOM";
        case HOST_CFG_PROFILE_REJECT_UNROUTED:             return "UNROUTED";
        case HOST_CFG_PROFILE_REJECT_NOT_STAGED:           return "NOT_STAGED";
        case HOST_CFG_PROFILE_REJECT_NULL_ARG:             return "NULL_ARG";
    }
    return "?";
}

/* ------------------------------------------------------------------ */

#define EXPECT(cond, fmt, ...) \
    do { if (!(cond)) { \
        fprintf(stderr, "[FAIL] %s:%d: " fmt "\n", \
                __FILE__, __LINE__, ##__VA_ARGS__); \
        exit(1); \
    } } while (0)

int main(void) {
    /* ---- NULL_ARG: validator must reject NULL regardless of state. */
    EXPECT(host_cfg_profile_validate(NULL, false) ==
               HOST_CFG_PROFILE_REJECT_NULL_ARG,
           "validate(NULL, false) must be NULL_ARG");
    EXPECT(host_cfg_profile_validate(NULL, true) ==
               HOST_CFG_PROFILE_REJECT_NULL_ARG,
           "validate(NULL, true) must be NULL_ARG");
    EXPECT(host_cfg_profile_reject_to_cfg_status(HOST_CFG_PROFILE_REJECT_NULL_ARG) ==
               CFG_STATUS_PROFILE_REJECT_NULL_ARG,
           "NULL_ARG -> wire status pin");

    /* ---- Exhaustive Cartesian product over the axes. */
    size_t cases = 0U;
    size_t legal = 0U;
    size_t reject = 0U;
    /* Per-reject histogram so the bench observer can sanity-check the
     * distribution (no class should be zero — that would mean the axis
     * tables don't cover the boundary, i.e. the fuzz is mute on that
     * class and the next regression in that branch slips by). */
    size_t hist[HOST_CFG_PROFILE_REJECT_NULL_ARG + 1] = {0};

    for (size_t pi = 0; pi < K_PROFILES_N; ++pi) {
    for (size_t gi = 0; gi < K_GAINS_N; ++gi) {
    for (size_t hi = 0; hi < K_HW_N; ++hi) {
    for (size_t mi = 0; mi < K_MASKS_N; ++mi) {
    for (size_t ri = 0; ri < K_ROUTED_N; ++ri) {
        host_cfg_profile_req_t req;
        memset(&req, 0, sizeof(req));
        req.profile_id       = k_profiles[pi];
        req.antenna_gain_dBi = k_antenna_gains[gi];
        req.hw_ceiling_dBm   = k_hw_ceilings[hi];
        req.channel_mask     = materialize_mask(k_masks[mi]);
        /* BW is always the per-profile default so the cross-product
         * fuzz never produces BW_MISMATCH (covered separately by
         * cfg_profile.c). This keeps the predictor simple and isolates
         * the (profile, antenna, hw, mask, routed) interactions. */
        req.modem_bw_hz = host_cfg_profile_default_bw_hz(req.profile_id);

        const host_cfg_profile_reject_t got =
            host_cfg_profile_validate(&req, k_tx_routed[ri]);
        const host_cfg_profile_reject_t want =
            predict_reject(&req, k_tx_routed[ri]);

        if (got != want) {
            fprintf(stderr,
                "[FAIL] tuple #%zu profile=0x%02X gain=%d hw=%u "
                "mask_class=%d routed=%d => got=%s want=%s\n",
                cases, (unsigned)req.profile_id,
                (int)req.antenna_gain_dBi,
                (unsigned)req.hw_ceiling_dBm,
                (int)k_masks[mi],
                (int)k_tx_routed[ri],
                reject_name(got), reject_name(want));
            exit(1);
        }

        /* Power-clamp byte equality (independent of reject reason —
         * the clamp is a pure function and must agree with the
         * golden re-implementation for every (tier, hw, gain) we
         * touch, including the OOR profiles where tier_ceiling==0). */
        const uint8_t tier =
            host_cfg_profile_tier_ceiling_dBm(req.profile_id);
        const uint8_t got_clamp = host_cfg_profile_power_clamp(
            tier, req.hw_ceiling_dBm, req.antenna_gain_dBi);
        const uint8_t want_clamp = golden_power_clamp(
            tier, req.hw_ceiling_dBm, req.antenna_gain_dBi);
        EXPECT(got_clamp == want_clamp,
            "tuple #%zu clamp mismatch tier=%u hw=%u gain=%d "
            "got=%u want=%u",
            cases, (unsigned)tier,
            (unsigned)req.hw_ceiling_dBm,
            (int)req.antenna_gain_dBi,
            (unsigned)got_clamp, (unsigned)want_clamp);

        /* Global invariant: clamp <= min(tier, hw). Holds even when
         * the result is 0 (which is <= every uint8_t). */
        const uint8_t tier_or_hw =
            (tier < req.hw_ceiling_dBm) ? tier : req.hw_ceiling_dBm;
        EXPECT(got_clamp <= tier_or_hw || tier == 0U,
            "tuple #%zu clamp=%u exceeds min(tier=%u, hw=%u)",
            cases, (unsigned)got_clamp,
            (unsigned)tier, (unsigned)req.hw_ceiling_dBm);

        /* Global invariant: NO_POWER_HEADROOM iff the request
         * survived NULL/BAD_PROFILE/ANTENNA checks AND clamp==0. */
        const bool would_pass_pre_power = (
            req.profile_id <= REG_PROFILE_MAX &&
            req.antenna_gain_dBi >= 0 &&
            req.antenna_gain_dBi <= HOST_CFG_PROFILE_ANTENNA_GAIN_MAX_DBI);
        if (would_pass_pre_power && got_clamp == 0U) {
            EXPECT(got == HOST_CFG_PROFILE_REJECT_NO_POWER_HEADROOM,
                "tuple #%zu clamp==0 but reject=%s (expected NO_POWER_HEADROOM)",
                cases, reject_name(got));
        }
        if (got == HOST_CFG_PROFILE_REJECT_NO_POWER_HEADROOM) {
            EXPECT(got_clamp == 0U,
                "tuple #%zu reject=NO_POWER_HEADROOM but clamp=%u",
                cases, (unsigned)got_clamp);
        }

        /* Wire-status mapping must be a total function over every
         * reject reason we actually observe in this fuzz. */
        cfg_status_t st = host_cfg_profile_reject_to_cfg_status(got);
        EXPECT(st != CFG_STATUS_APPLY_FAILED || got > HOST_CFG_PROFILE_REJECT_NULL_ARG,
            "tuple #%zu reject=%s mapped to APPLY_FAILED fallback",
            cases, reject_name(got));

        hist[got]++;
        if (got == HOST_CFG_PROFILE_REJECT_NONE) legal++; else reject++;
        cases++;
    }}}}}

    /* ---- Coverage invariants on the histogram. Every reject class
     * that the validator can produce for THIS axis set must have at
     * least one tuple firing it, otherwise the axis tables don't
     * actually exercise that branch. NULL_ARG is excluded (covered
     * by the standalone NULL probes above; the cross-product never
     * passes NULL because we always synthesise a real req struct). */
    EXPECT(hist[HOST_CFG_PROFILE_REJECT_NONE]                 > 0, "NONE bucket empty");
    EXPECT(hist[HOST_CFG_PROFILE_REJECT_BAD_PROFILE]          > 0, "BAD_PROFILE bucket empty");
    EXPECT(hist[HOST_CFG_PROFILE_REJECT_ANTENNA_OUT_OF_RANGE] > 0, "ANTENNA bucket empty");
    EXPECT(hist[HOST_CFG_PROFILE_REJECT_NO_POWER_HEADROOM]    > 0, "NO_POWER_HEADROOM bucket empty");
    EXPECT(hist[HOST_CFG_PROFILE_REJECT_UNROUTED]             > 0, "UNROUTED bucket empty");
    EXPECT(hist[HOST_CFG_PROFILE_REJECT_MASK_POPCOUNT]        > 0, "MASK_POPCOUNT bucket empty");
    EXPECT(hist[HOST_CFG_PROFILE_REJECT_MASK_OUT_OF_TABLE]    > 0, "MASK_OUT_OF_TABLE bucket empty");
    /* BW_MISMATCH and NOT_STAGED are intentionally not in this fuzz
     * (the cross-product always pairs the per-profile default BW
     * and never touches the stage state machine). */

    printf("[PASS] cfg_clamp_fuzz: cases=%zu legal=%zu reject=%zu\n",
           cases, legal, reject);
    printf("  bucket NONE=%zu BAD_PROFILE=%zu ANTENNA=%zu NO_HEADROOM=%zu "
           "UNROUTED=%zu MASK_POP=%zu MASK_OOT=%zu\n",
           hist[HOST_CFG_PROFILE_REJECT_NONE],
           hist[HOST_CFG_PROFILE_REJECT_BAD_PROFILE],
           hist[HOST_CFG_PROFILE_REJECT_ANTENNA_OUT_OF_RANGE],
           hist[HOST_CFG_PROFILE_REJECT_NO_POWER_HEADROOM],
           hist[HOST_CFG_PROFILE_REJECT_UNROUTED],
           hist[HOST_CFG_PROFILE_REJECT_MASK_POPCOUNT],
           hist[HOST_CFG_PROFILE_REJECT_MASK_OUT_OF_TABLE]);
    return 0;
}
