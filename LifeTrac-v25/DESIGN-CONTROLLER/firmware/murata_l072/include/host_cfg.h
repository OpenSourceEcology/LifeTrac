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
    CFG_STATUS_PROFILE_UNROUTED = 7
} cfg_status_t;

void cfg_init(void);
cfg_status_t cfg_set(uint8_t key, const uint8_t *value, uint8_t len);
cfg_status_t cfg_get(uint8_t key, uint8_t *out, uint8_t out_cap, uint8_t *out_len);
bool cfg_is_dirty(void);

#endif /* LIFETRAC_MURATA_L072_HOST_CFG_H */
