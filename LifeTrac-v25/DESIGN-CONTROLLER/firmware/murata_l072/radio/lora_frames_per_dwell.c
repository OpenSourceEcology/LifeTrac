/*
 * P3 / Phase 3.2 — firmware mirror of the host-side frames_per_dwell()
 * pacing-hint helper. See include/lora_frames_per_dwell.h for the full
 * contract and host parity references.
 */

#include "lora_frames_per_dwell.h"

uint8_t lora_frames_per_dwell(uint32_t toa_us) {
    if (toa_us == 0U) {
        return 1U;
    }

    const uint32_t budget_us =
        (LORA_FPD_LEGAL_DWELL_US * LORA_FPD_DWELL_HEADROOM_PCT) / 100U;
    const uint32_t n = budget_us / toa_us;

    if (n < 1U) {
        return 1U;
    }
    if (n > (uint32_t)LORA_FPD_MAX_FRAMES_PER_DWELL_CAP) {
        return LORA_FPD_MAX_FRAMES_PER_DWELL_CAP;
    }
    return (uint8_t)n;
}
