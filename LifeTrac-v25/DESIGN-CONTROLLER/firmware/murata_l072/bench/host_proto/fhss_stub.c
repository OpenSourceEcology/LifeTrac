/*
 * Host-side link stub for sx1276_fhss_init(), needed by the four
 * cfg-profile/cfg-contract bench targets that link host_cfg_profile.c
 * with sx1276_stub.c instead of the real radio objects.
 *
 * Kept OUT of sx1276_stub.c on purpose: check-tx-len-guard (and any
 * future target) links sx1276_stub.c TOGETHER WITH the real
 * radio/sx1276_fhss.c, and a stub definition there would collide with
 * the real symbol at link time. Only link this file where the real
 * FHSS scheduler is absent.
 */
#include "sx1276_fhss.h"

#include <stdint.h>

sx1276_fhss_status_t sx1276_fhss_init(uint64_t farm_id,
                                      uint64_t node_id,
                                      uint32_t initial_epoch) {
    (void)farm_id;
    (void)node_id;
    (void)initial_epoch;
    return SX1276_FHSS_OK;
}

/* D4: capture the QoS budget programmed by profile activation so
 * cfg_profile_wire.c can assert DTS -> 950 ms. Lives here (not in
 * sx1276_stub.c) because targets that link the REAL
 * radio/sx1276_airtime.c would otherwise hit a duplicate symbol. */
static uint32_t s_stub_budget_us;

void sx1276_airtime_set_budget_us(uint32_t budget_us) {
    s_stub_budget_us = budget_us;
}

uint32_t sx1276_stub_last_budget_us(void) {
    return s_stub_budget_us;
}

/* 2026-07-24 FHSS bring-up: host_cfg_profile_activate() now also calls
 * sx1276_rx_scan_reset() when activating the FHSS profile. Same
 * placement rationale as above — targets that link the real
 * radio/sx1276_rx.c must not see a second definition. */
void sx1276_rx_scan_reset(void) {
}
