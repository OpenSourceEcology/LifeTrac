/*
 * FCC-A6c-2-c-i-α': RX cold-start scan channel walker implementation.
 * See include/sx1276_rx_scan_walker.h for the design contract.
 */

#include "sx1276_rx_scan_walker.h"

#include "sx1276_fhss_chantab.h"

#include <stddef.h>
#include <stdint.h>

/* Walker file-static. Default zero-init: walker starts at channel 0
 * before any reset call, matching reset semantics. */
static uint8_t s_walker_idx;

void sx1276_rx_scan_walker_reset(void) {
    s_walker_idx = 0U;
}

uint32_t sx1276_rx_scan_walker_next(uint8_t *out_idx) {
    if (out_idx == NULL) {
        return 0UL;
    }
    const uint8_t idx = s_walker_idx;
    const uint32_t hz = sx1276_fhss_chantab_center_hz(idx);
    /* Advance even on hz==0 chantab fault so a corrupted-table call
     * doesn't pin the walker on a bad index forever; caller treats
     * 0 as a fault and skips the HW write. */
    s_walker_idx = (uint8_t)((idx + 1U) % SX1276_FHSS_CHANNEL_COUNT);
    *out_idx = idx;
    return hz;
}

uint8_t sx1276_rx_scan_walker_peek_next(void) {
    return s_walker_idx;
}
