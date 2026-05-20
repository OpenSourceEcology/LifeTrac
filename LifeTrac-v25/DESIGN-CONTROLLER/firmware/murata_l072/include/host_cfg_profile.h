#ifndef LIFETRAC_MURATA_L072_HOST_CFG_PROFILE_H
#define LIFETRAC_MURATA_L072_HOST_CFG_PROFILE_H

/*
 * FCC-A5 — regulatory-profile validation + two-phase commit.
 *
 * Cross-references:
 *   - AI NOTES/2026-05-19_LoRa_FCC_50CH_FHSS_Implementation_Plan_Copilot_v1_0.md
 *     §A5 (profile + cfg validation), §11 delta #4 (two-phase commit),
 *     §13.1 (compile-time gate), §14.2 step 8.
 *   - TODO.md Stage S1.5 -> FCC-A5.
 *
 * Design properties (kept deliberately pure / no HAL deps so the
 * module is host-testable byte-for-byte):
 *
 *   1. host_cfg_profile_validate() is a pure function over the
 *      candidate request struct + an explicit tx_routed boolean. The
 *      tx_routed boolean mirrors the compile-time LIFETRAC_FHSS_TX_ROUTED
 *      build symbol so production profiles can be exercised by host
 *      tests in both "routed" and "unrouted" states without recompiling
 *      the firmware.
 *
 *   2. host_cfg_profile_power_clamp() is a pure function implementing
 *      the §A5 ERP clamp:
 *
 *          Pmax = min( tier_ceiling - max(0, antenna_gain_dBi - 6),
 *                      hw_ceiling )
 *
 *      with a floor at HOST_CFG_PROFILE_TX_POWER_MIN_DBM. Returns 0
 *      if the clamp leaves no headroom above the floor; callers MUST
 *      treat 0 as REJECT_NO_POWER_HEADROOM.
 *
 *   3. The two-phase commit state machine is the ONLY supported way
 *      to transition between regulatory profiles. cfg_set(REG_PROFILE)
 *      remains the wire-level entry point but FCC-A4 will route it
 *      through stage()/activate() so partial reconfiguration cannot
 *      leave the radio in a half-converted state. Until FCC-A4 lands
 *      the state machine is exercised only by the host test.
 *
 *      Sequence:
 *          host_cfg_profile_reset(initial_active);  // boot
 *          host_cfg_profile_stage(&req, tx_routed); // returns REJECT_*
 *          host_cfg_profile_activate();             // atomic swap
 *
 *      A failed stage() leaves the prior active profile live and
 *      clears any in-flight stage. A failed activate() (NOT_STAGED)
 *      is a programmer error from the caller side; the prior active
 *      profile is untouched.
 *
 *   4. No release/rollback path exists for an already-activated
 *      profile. The next stage()->activate() is the only way to
 *      transition. This is intentional: regulatory profile changes
 *      are infrequent and must be auditable.
 */

#include <stdbool.h>
#include <stdint.h>

/*
 * The certified FCC 50-channel table covers indices 0..49 of
 * sx1276_fhss_chantab. A valid FCC_15_247_FHSS_50CH_BW250 mask must
 * therefore have:
 *   - popcount(mask) >= 50 (delta from plan §A5; with 50 valid bits
 *     this collapses to popcount == 50), AND
 *   - no bits set above bit 49 (i.e. mask & ~REQUIRED_MASK == 0).
 *
 * The combination forces mask == REQUIRED_MASK.
 */
#define HOST_CFG_PROFILE_FHSS_50CH_REQUIRED_MASK    ((uint64_t)((1ULL << 50) - 1ULL))

#define HOST_CFG_PROFILE_FHSS_BW_HZ                 250000UL
#define HOST_CFG_PROFILE_DTS_BW_HZ                  500000UL

/*
 * Tier ceilings per §A5. The bench profile reuses the existing +17 dBm
 * hardware ceiling because there is no per-tier conducted-power
 * envelope in the bench-only path. FHSS and DTS production profiles
 * use +30 dBm as the regulatory tier ceiling; the hardware ceiling
 * supplied by the caller will typically clamp this further (the
 * Murata module's PA tops out at +20 dBm).
 */
#define HOST_CFG_PROFILE_TIER_CEILING_BENCH_DBM     17U
#define HOST_CFG_PROFILE_TIER_CEILING_FHSS_DBM      30U
#define HOST_CFG_PROFILE_TIER_CEILING_DTS_DBM       30U

#define HOST_CFG_PROFILE_ANTENNA_GAIN_MAX_DBI       30
#define HOST_CFG_PROFILE_ANTENNA_GAIN_FCC_THRESHOLD_DBI 6
#define HOST_CFG_PROFILE_TX_POWER_MIN_DBM           2U

/*
 * Structured reject reason for stage/activate. Sentinel NONE = 0
 * indicates success and is mapped to CFG_STATUS_OK at the wire layer.
 * All non-zero values are reported verbatim through the wire-level
 * status byte so bench post-processing can attribute failures.
 */
typedef enum host_cfg_profile_reject_e {
    HOST_CFG_PROFILE_REJECT_NONE                 = 0,
    HOST_CFG_PROFILE_REJECT_BAD_PROFILE          = 1,
    HOST_CFG_PROFILE_REJECT_MASK_POPCOUNT        = 2,
    HOST_CFG_PROFILE_REJECT_MASK_OUT_OF_TABLE    = 3,
    HOST_CFG_PROFILE_REJECT_BW_MISMATCH          = 4,
    HOST_CFG_PROFILE_REJECT_ANTENNA_OUT_OF_RANGE = 5,
    HOST_CFG_PROFILE_REJECT_NO_POWER_HEADROOM    = 6,
    HOST_CFG_PROFILE_REJECT_UNROUTED             = 7,
    HOST_CFG_PROFILE_REJECT_NOT_STAGED           = 8,
    HOST_CFG_PROFILE_REJECT_NULL_ARG             = 9
} host_cfg_profile_reject_t;

/*
 * Candidate request. Filled in from the cfg-set transaction (caller
 * collects channel_mask, modem_bw_hz, antenna_gain_dBi, hw_ceiling_dBm
 * from the cfg layer) and passed to stage(). The struct is plain data;
 * the validator never mutates it.
 *
 * hw_ceiling_dBm = the firmware-known hardware PA ceiling (typically
 * +17 dBm on the bench path, up to +20 dBm on production hardware).
 * Supplied by caller so the clamp stays a pure function.
 */
typedef struct host_cfg_profile_req_s {
    uint8_t  profile_id;
    uint64_t channel_mask;
    uint32_t modem_bw_hz;
    int8_t   antenna_gain_dBi;
    uint8_t  hw_ceiling_dBm;
} host_cfg_profile_req_t;

/* Pure validator. tx_routed mirrors compile-time LIFETRAC_FHSS_TX_ROUTED. */
host_cfg_profile_reject_t host_cfg_profile_validate(
    const host_cfg_profile_req_t *req,
    bool tx_routed);

/*
 * Pure power clamp. Returns the maximum conducted dBm allowed by the
 * tier_ceiling/antenna combination, further clamped to hw_ceiling.
 * Returns 0 if the result would be below HOST_CFG_PROFILE_TX_POWER_MIN_DBM
 * (callers MUST treat 0 as REJECT_NO_POWER_HEADROOM, never as a valid
 * transmit power).
 */
uint8_t host_cfg_profile_power_clamp(
    uint8_t tier_ceiling_dBm,
    uint8_t hw_ceiling_dBm,
    int8_t  antenna_gain_dBi);

/* Tier ceiling for a profile id, or 0 for unknown profiles. */
uint8_t host_cfg_profile_tier_ceiling_dBm(uint8_t profile_id);

/* Two-phase commit. */
void host_cfg_profile_reset(const host_cfg_profile_req_t *initial_active);
host_cfg_profile_reject_t host_cfg_profile_stage(
    const host_cfg_profile_req_t *req,
    bool tx_routed);
host_cfg_profile_reject_t host_cfg_profile_activate(void);
void host_cfg_profile_cancel_stage(void);
bool host_cfg_profile_has_stage(void);
const host_cfg_profile_req_t *host_cfg_profile_active(void);

#endif /* LIFETRAC_MURATA_L072_HOST_CFG_PROFILE_H */
