#include "sx1276_stub.h"

#include <stdbool.h>
#include <stdint.h>

static uint8_t s_last_tx_power_dbm;
static uint32_t s_tx_power_call_count;
static uint8_t s_stub_sf;
static uint16_t s_stub_bw_khz;
static uint8_t s_stub_cr;

void sx1276_set_tx_power_dbm(uint8_t dbm) {
    s_last_tx_power_dbm = dbm;
    s_tx_power_call_count++;
}

void sx1276_set_sf_bw_cr(uint8_t sf, uint16_t bw_khz, uint8_t cr_den) {
    s_stub_sf = sf;
    s_stub_bw_khz = bw_khz;
    s_stub_cr = cr_den;
}

/* D4 budget capture lives in fhss_stub.c — NOT here — because
 * check-tx-len-guard links this file together with the REAL
 * radio/sx1276_airtime.c and a stub sx1276_airtime_set_budget_us()
 * here would collide at link time. sx1276_stub_last_budget_us() is
 * declared in sx1276_stub.h but defined in fhss_stub.c so only the
 * real-radio-absent targets see it. */

bool sx1276_modes_to_standby(void) {
    return true;
}

void sx1276_stub_reset(void) {
    s_last_tx_power_dbm = 0U;
    s_tx_power_call_count = 0U;
    s_stub_sf = 0U;
    s_stub_bw_khz = 0U;
    s_stub_cr = 0U;
}

uint8_t sx1276_stub_last_tx_power_dbm(void) {
    return s_last_tx_power_dbm;
}

uint32_t sx1276_stub_tx_power_call_count(void) {
    return s_tx_power_call_count;
}

uint16_t sx1276_stub_last_bw_khz(void) {
    return s_stub_bw_khz;
}
