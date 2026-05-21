#ifndef LIFETRAC_MURATA_L072_HOST_CFG_H
#define LIFETRAC_MURATA_L072_HOST_CFG_H

#include <stdbool.h>
#include <stdint.h>

typedef enum cfg_status_e {
    CFG_STATUS_OK = 0,
    CFG_STATUS_UNKNOWN_KEY = 1,
    CFG_STATUS_BAD_LENGTH = 2,
    CFG_STATUS_OUT_OF_RANGE = 3,
    CFG_STATUS_APPLY_FAILED = 4,
    CFG_STATUS_DEFERRED = 5,
    CFG_STATUS_READ_ONLY = 6,
    /*
     * Returned by cfg_set(CFG_KEY_REG_PROFILE, ...) when the requested
     * regulatory profile selects a TX path that the current build does
     * not route to. Today this means FCC_15_247_FHSS_50CH_BW250 or
     * FCC_15_247_DTS_BW500 is requested but FCC-A4 has not landed yet
     * (build symbol LIFETRAC_FHSS_TX_ROUTED is undefined). See
     * AI NOTES/2026-05-19_LoRa_FCC_50CH_FHSS_Implementation_Plan_Copilot_v1_0.md
     * §14 and TODO.md FCC-PROFILE-ENUM / FCC-A4 / FCC-A5.
     */
    CFG_STATUS_PROFILE_UNROUTED = 7,
    /*
     * FCC-EVID-D6-2-a-i: distinct wire status bytes for the
     * host_cfg_profile_reject_t codes that are not already mapped
     * to legacy cfg_status_t values. Mapping table (see
     * host_cfg_profile_reject_to_cfg_status):
     *   HOST_CFG_PROFILE_REJECT_NONE                 -> CFG_STATUS_OK (0)
     *   HOST_CFG_PROFILE_REJECT_BAD_PROFILE          -> CFG_STATUS_OUT_OF_RANGE (3)  [alias]
     *   HOST_CFG_PROFILE_REJECT_UNROUTED             -> CFG_STATUS_PROFILE_UNROUTED (7) [alias]
     *   HOST_CFG_PROFILE_REJECT_MASK_POPCOUNT        -> 8
     *   HOST_CFG_PROFILE_REJECT_MASK_OUT_OF_TABLE    -> 9
     *   HOST_CFG_PROFILE_REJECT_BW_MISMATCH          -> 10
     *   HOST_CFG_PROFILE_REJECT_ANTENNA_OUT_OF_RANGE -> 11
     *   HOST_CFG_PROFILE_REJECT_NO_POWER_HEADROOM    -> 12
     *   HOST_CFG_PROFILE_REJECT_NOT_STAGED           -> 13
     *   HOST_CFG_PROFILE_REJECT_NULL_ARG             -> 14
     *
     * Values 8..14 are pure-additive — no existing callers branch
     * on cfg_status_t > CFG_STATUS_PROFILE_UNROUTED today, so the
     * extension is wire-backward-compatible (a legacy host parser
     * still sees these as non-zero error codes).
     */
    CFG_STATUS_PROFILE_REJECT_MASK_POPCOUNT       = 8,
    CFG_STATUS_PROFILE_REJECT_MASK_OUT_OF_TABLE   = 9,
    CFG_STATUS_PROFILE_REJECT_BW_MISMATCH         = 10,
    CFG_STATUS_PROFILE_REJECT_ANTENNA_OUT_OF_RANGE = 11,
    CFG_STATUS_PROFILE_REJECT_NO_POWER_HEADROOM   = 12,
    CFG_STATUS_PROFILE_REJECT_NOT_STAGED          = 13,
    CFG_STATUS_PROFILE_REJECT_NULL_ARG            = 14
} cfg_status_t;

void cfg_init(void);
cfg_status_t cfg_set(uint8_t key, const uint8_t *value, uint8_t len);
cfg_status_t cfg_get(uint8_t key, uint8_t *out, uint8_t out_cap, uint8_t *out_len);
bool cfg_is_dirty(void);

#endif /* LIFETRAC_MURATA_L072_HOST_CFG_H */
