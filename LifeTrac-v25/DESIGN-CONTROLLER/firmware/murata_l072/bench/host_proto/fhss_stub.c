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

/*
 * 2026-07-29 FHSS seed identity: capture the seed so cfg_profile_wire.c
 * can pin that CFG_KEY_FHSS_FARM_ID / CFG_KEY_FHSS_NODE_ID actually
 * reach the scheduler. Before that fix this call site passed a
 * hardcoded (0, 0, 0), so every radio derived the same hop permutation
 * — and nothing observed the arguments, which is precisely why the
 * defect survived. Accessors are declared in sx1276_stub.h alongside
 * sx1276_stub_last_budget_us(), which is defined here for the same
 * duplicate-symbol reason.
 */
static uint64_t s_stub_fhss_farm_id;
static uint64_t s_stub_fhss_node_id;
static uint32_t s_stub_fhss_initial_epoch;
static uint32_t s_stub_fhss_init_calls;

sx1276_fhss_status_t sx1276_fhss_init(uint64_t farm_id,
                                      uint64_t node_id,
                                      uint32_t initial_epoch) {
    s_stub_fhss_farm_id       = farm_id;
    s_stub_fhss_node_id       = node_id;
    s_stub_fhss_initial_epoch = initial_epoch;
    s_stub_fhss_init_calls++;
    return SX1276_FHSS_OK;
}

uint64_t sx1276_stub_last_fhss_farm_id(void) {
    return s_stub_fhss_farm_id;
}

uint64_t sx1276_stub_last_fhss_node_id(void) {
    return s_stub_fhss_node_id;
}

uint32_t sx1276_stub_last_fhss_initial_epoch(void) {
    return s_stub_fhss_initial_epoch;
}

uint32_t sx1276_stub_fhss_init_call_count(void) {
    return s_stub_fhss_init_calls;
}

/*
 * Deliberately NOT folded into sx1276_fhss_reset() below: that is the
 * real API the non-FHSS activation branches call, so clearing the
 * capture there would erase the very evidence a DTS-then-FHSS test
 * sequence needs. Also cannot live in sx1276_stub_reset() — that is a
 * different TU and these are file-statics.
 */
void sx1276_stub_fhss_capture_reset(void) {
    s_stub_fhss_farm_id       = 0ULL;
    s_stub_fhss_node_id       = 0ULL;
    s_stub_fhss_initial_epoch = 0U;
    s_stub_fhss_init_calls    = 0U;
}

/* 2026-07-25 run-23 stale-state fix: non-FHSS activation branches now
 * tear down the scheduler. Same collision rule as sx1276_fhss_init
 * above — real symbol lives in radio/sx1276_fhss.c. */
void sx1276_fhss_reset(void) {
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
