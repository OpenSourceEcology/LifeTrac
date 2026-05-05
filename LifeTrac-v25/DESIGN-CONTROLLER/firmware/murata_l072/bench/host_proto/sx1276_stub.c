#include "sx1276_stub.h"

#include <stdint.h>

static uint8_t s_last_tx_power_dbm;
static uint32_t s_tx_power_call_count;

void sx1276_set_tx_power_dbm(uint8_t dbm) {
    s_last_tx_power_dbm = dbm;
    s_tx_power_call_count++;
}

void sx1276_stub_reset(void) {
    s_last_tx_power_dbm = 0U;
    s_tx_power_call_count = 0U;
}

uint8_t sx1276_stub_last_tx_power_dbm(void) {
    return s_last_tx_power_dbm;
}

uint32_t sx1276_stub_tx_power_call_count(void) {
    return s_tx_power_call_count;
}
