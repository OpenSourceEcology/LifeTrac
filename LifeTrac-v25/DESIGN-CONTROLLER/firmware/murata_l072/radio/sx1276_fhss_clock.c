/*
 * v25.0.7 slot-clock FHSS — implementation. See include/sx1276_fhss_clock.h
 * for the design contract. HW-free on purpose (bench-linkable).
 */

#include "sx1276_fhss_clock.h"

typedef struct {
    uint32_t anchor_ms;       /* local time of the start of anchor_abs */
    uint32_t anchor_abs;      /* absolute slot number at anchor_ms     */
    uint8_t  valid;
} fhss_clock_state_t;

static fhss_clock_state_t s_clk;

void sx1276_fhss_clock_reset(void) {
    s_clk.anchor_ms  = 0U;
    s_clk.anchor_abs = 0U;
    s_clk.valid      = 0U;
}

void sx1276_fhss_clock_anchor(uint32_t slot_start_ms, uint32_t abs_slot) {
    s_clk.anchor_ms  = slot_start_ms;
    s_clk.anchor_abs = abs_slot;
    s_clk.valid      = 1U;
}

uint8_t sx1276_fhss_clock_valid(void) {
    return s_clk.valid;
}

uint32_t sx1276_fhss_clock_abs_slot(uint32_t now_ms) {
    /* Unsigned subtraction: correct across the u32 ms wrap for anchors
     * younger than ~49.7 days (uint32-ms-wrap idiom). */
    const uint32_t elapsed = now_ms - s_clk.anchor_ms;
    return s_clk.anchor_abs + elapsed / SX1276_FHSS_SLOT_MS;
}

uint32_t sx1276_fhss_clock_in_slot_ms(uint32_t now_ms) {
    const uint32_t elapsed = now_ms - s_clk.anchor_ms;
    return elapsed % SX1276_FHSS_SLOT_MS;
}
