/*
 * FCC-A5 — regulatory-profile validation + two-phase commit.
 * See include/host_cfg_profile.h for the design contract.
 */

#include "host_cfg_profile.h"
#include "host_cfg_keys.h"
#include "sx1276_fhss.h"
#include "sx1276.h"
#include "sx1276_modes.h"
#include "sx1276_airtime.h"
#include "sx1276_rx.h"

#include <stddef.h>
#include <string.h>

static host_cfg_profile_req_t s_active;
static host_cfg_profile_req_t s_staged;
static bool s_has_stage;

static uint8_t popcount_u64(uint64_t v) {
    uint8_t n = 0U;
    while (v != 0ULL) {
        n = (uint8_t)(n + (uint8_t)(v & 1ULL));
        v >>= 1;
    }
    return n;
}

uint8_t host_cfg_profile_tier_ceiling_dBm(uint8_t profile_id) {
    switch (profile_id) {
        case REG_PROFILE_BENCH_ONLY_FIXED_915:       return HOST_CFG_PROFILE_TIER_CEILING_BENCH_DBM;
        case REG_PROFILE_FCC_15_247_FHSS_50CH_BW250: return HOST_CFG_PROFILE_TIER_CEILING_FHSS_DBM;
        case REG_PROFILE_FCC_15_247_DTS_BW500:       return HOST_CFG_PROFILE_TIER_CEILING_DTS_DBM;
        default:                                     return 0U;
    }
}

uint32_t host_cfg_profile_default_bw_hz(uint8_t profile_id) {
    switch (profile_id) {
        case REG_PROFILE_BENCH_ONLY_FIXED_915:       return HOST_CFG_PROFILE_BENCH_BW_HZ;
        case REG_PROFILE_FCC_15_247_FHSS_50CH_BW250: return HOST_CFG_PROFILE_FHSS_BW_HZ;
        case REG_PROFILE_FCC_15_247_DTS_BW500:       return HOST_CFG_PROFILE_DTS_BW_HZ;
        default:                                     return 0UL;
    }
}

cfg_status_t host_cfg_profile_reject_to_cfg_status(host_cfg_profile_reject_t r) {
    switch (r) {
        case HOST_CFG_PROFILE_REJECT_NONE:                 return CFG_STATUS_OK;
        case HOST_CFG_PROFILE_REJECT_BAD_PROFILE:          return CFG_STATUS_OUT_OF_RANGE;
        case HOST_CFG_PROFILE_REJECT_UNROUTED:             return CFG_STATUS_PROFILE_UNROUTED;
        case HOST_CFG_PROFILE_REJECT_MASK_POPCOUNT:        return CFG_STATUS_PROFILE_REJECT_MASK_POPCOUNT;
        case HOST_CFG_PROFILE_REJECT_MASK_OUT_OF_TABLE:    return CFG_STATUS_PROFILE_REJECT_MASK_OUT_OF_TABLE;
        case HOST_CFG_PROFILE_REJECT_BW_MISMATCH:          return CFG_STATUS_PROFILE_REJECT_BW_MISMATCH;
        case HOST_CFG_PROFILE_REJECT_ANTENNA_OUT_OF_RANGE: return CFG_STATUS_PROFILE_REJECT_ANTENNA_OUT_OF_RANGE;
        case HOST_CFG_PROFILE_REJECT_NO_POWER_HEADROOM:    return CFG_STATUS_PROFILE_REJECT_NO_POWER_HEADROOM;
        case HOST_CFG_PROFILE_REJECT_NOT_STAGED:           return CFG_STATUS_PROFILE_REJECT_NOT_STAGED;
        case HOST_CFG_PROFILE_REJECT_NULL_ARG:             return CFG_STATUS_PROFILE_REJECT_NULL_ARG;
        default:                                           return CFG_STATUS_APPLY_FAILED;
    }
}

uint8_t host_cfg_profile_power_clamp(uint8_t tier_ceiling_dBm,
                                     uint8_t hw_ceiling_dBm,
                                     int8_t  antenna_gain_dBi) {
    /*
     * §A5 ERP clamp. The antenna-gain reduction kicks in above
     * +6 dBi. Negative or sub-threshold gain is treated as zero
     * reduction (the rule is one-sided).
     */
    int reduction = 0;
    if (antenna_gain_dBi > HOST_CFG_PROFILE_ANTENNA_GAIN_FCC_THRESHOLD_DBI) {
        reduction = (int)antenna_gain_dBi - HOST_CFG_PROFILE_ANTENNA_GAIN_FCC_THRESHOLD_DBI;
    }

    int allowed = (int)tier_ceiling_dBm - reduction;
    if (allowed > (int)hw_ceiling_dBm) {
        allowed = (int)hw_ceiling_dBm;
    }
    if (allowed < (int)HOST_CFG_PROFILE_TX_POWER_MIN_DBM) {
        return 0U;
    }
    return (uint8_t)allowed;
}

host_cfg_profile_reject_t host_cfg_profile_validate(
        const host_cfg_profile_req_t *req,
        bool tx_routed) {
    if (req == NULL) {
        return HOST_CFG_PROFILE_REJECT_NULL_ARG;
    }
    if (req->profile_id > REG_PROFILE_MAX) {
        return HOST_CFG_PROFILE_REJECT_BAD_PROFILE;
    }
    if (req->antenna_gain_dBi < 0 ||
        req->antenna_gain_dBi > HOST_CFG_PROFILE_ANTENNA_GAIN_MAX_DBI) {
        return HOST_CFG_PROFILE_REJECT_ANTENNA_OUT_OF_RANGE;
    }

    const uint8_t tier_ceiling = host_cfg_profile_tier_ceiling_dBm(req->profile_id);
    /* tier_ceiling == 0 cannot happen here (BAD_PROFILE check above). */
    if (host_cfg_profile_power_clamp(tier_ceiling,
                                     req->hw_ceiling_dBm,
                                     req->antenna_gain_dBi) == 0U) {
        return HOST_CFG_PROFILE_REJECT_NO_POWER_HEADROOM;
    }

    switch (req->profile_id) {
        case REG_PROFILE_BENCH_ONLY_FIXED_915:
            /*
             * Bench profile imposes only the sanity rule that the
             * channel mask is non-zero. BW is whatever the bench
             * test wires up.
             */
            if (req->channel_mask == 0ULL) {
                return HOST_CFG_PROFILE_REJECT_MASK_POPCOUNT;
            }
            return HOST_CFG_PROFILE_REJECT_NONE;

        case REG_PROFILE_FCC_15_247_FHSS_50CH_BW250:
            if (!tx_routed) {
                return HOST_CFG_PROFILE_REJECT_UNROUTED;
            }
            if ((req->channel_mask & ~HOST_CFG_PROFILE_FHSS_50CH_REQUIRED_MASK) != 0ULL) {
                return HOST_CFG_PROFILE_REJECT_MASK_OUT_OF_TABLE;
            }
            if (popcount_u64(req->channel_mask) < 50U) {
                return HOST_CFG_PROFILE_REJECT_MASK_POPCOUNT;
            }
            if (req->modem_bw_hz != HOST_CFG_PROFILE_FHSS_BW_HZ) {
                return HOST_CFG_PROFILE_REJECT_BW_MISMATCH;
            }
            return HOST_CFG_PROFILE_REJECT_NONE;

        case REG_PROFILE_FCC_15_247_DTS_BW500:
            if (!tx_routed) {
                return HOST_CFG_PROFILE_REJECT_UNROUTED;
            }
            if (req->channel_mask == 0ULL) {
                return HOST_CFG_PROFILE_REJECT_MASK_POPCOUNT;
            }
            if (req->modem_bw_hz != HOST_CFG_PROFILE_DTS_BW_HZ) {
                return HOST_CFG_PROFILE_REJECT_BW_MISMATCH;
            }
            return HOST_CFG_PROFILE_REJECT_NONE;

        default:
            /* Unreachable due to BAD_PROFILE check above; defensive. */
            return HOST_CFG_PROFILE_REJECT_BAD_PROFILE;
    }
}

void host_cfg_profile_reset(const host_cfg_profile_req_t *initial_active) {
    if (initial_active != NULL) {
        s_active = *initial_active;
    } else {
        memset(&s_active, 0, sizeof(s_active));
        s_active.profile_id   = REG_PROFILE_BENCH_ONLY_FIXED_915;
    }
    memset(&s_staged, 0, sizeof(s_staged));
    s_has_stage = false;
}

host_cfg_profile_reject_t host_cfg_profile_stage(
        const host_cfg_profile_req_t *req,
        bool tx_routed) {
    const host_cfg_profile_reject_t r = host_cfg_profile_validate(req, tx_routed);
    if (r != HOST_CFG_PROFILE_REJECT_NONE) {
        /*
         * Failed validation clears any prior in-flight stage so the
         * next activate() cannot accidentally commit a stale candidate.
         * The active profile is untouched.
         */
        memset(&s_staged, 0, sizeof(s_staged));
        s_has_stage = false;
        return r;
    }
    s_staged    = *req;
    s_has_stage = true;
    return HOST_CFG_PROFILE_REJECT_NONE;
}

host_cfg_profile_reject_t host_cfg_profile_activate(void) {
    if (!s_has_stage) {
        return HOST_CFG_PROFILE_REJECT_NOT_STAGED;
    }
    s_active    = s_staged;
    /* D2: RegModemConfig writes require SLEEP/STANDBY (Semtech DS §4.1;
     * sx1276_apply_profile_full() sequences the same way). Activation
     * can be requested while the RX daemon has RXCONT armed. */
    (void)sx1276_modes_to_standby();
    if (s_active.profile_id == REG_PROFILE_FCC_15_247_FHSS_50CH_BW250) {
        (void)sx1276_fhss_init(0ULL, 0ULL, 0U);
        sx1276_set_sf_bw_cr(7U, 250U, 5U);
        sx1276_airtime_set_budget_us(SX1276_AIRTIME_BUDGET_DEFAULT_US);
        /* 2026-07-24 FHSS bring-up: (re-)activation must also revive the
         * A6c scan SM. Without this the SM burns its 30 s redesign
         * budget on an empty band at boot, parks in the absorbing
         * FAILED state (standby), and no later activation ever
         * re-arms the receiver — fhss_rx_probe evidence 2026-07-24:
         * 75 s under active TX hopping, zero RX_FRAME, zero FAULT. */
        sx1276_rx_scan_reset();
    } else if (s_active.profile_id == REG_PROFILE_FCC_15_247_DTS_BW500) {
        sx1276_set_sf_bw_cr(7U, 500U, 5U);
        /* D4: PSD-based profile — QoS gate widened to 95 % duty. */
        sx1276_airtime_set_budget_us(SX1276_AIRTIME_BUDGET_DTS_US);
    } else if (s_active.profile_id == REG_PROFILE_BENCH_ONLY_FIXED_915) {
        sx1276_set_sf_bw_cr(7U, 250U, 5U);
        sx1276_airtime_set_budget_us(SX1276_AIRTIME_BUDGET_DEFAULT_US);
    }
    memset(&s_staged, 0, sizeof(s_staged));
    s_has_stage = false;
    return HOST_CFG_PROFILE_REJECT_NONE;
}

void host_cfg_profile_cancel_stage(void) {
    memset(&s_staged, 0, sizeof(s_staged));
    s_has_stage = false;
}

bool host_cfg_profile_has_stage(void) {
    return s_has_stage;
}

const host_cfg_profile_req_t *host_cfg_profile_active(void) {
    return &s_active;
}
