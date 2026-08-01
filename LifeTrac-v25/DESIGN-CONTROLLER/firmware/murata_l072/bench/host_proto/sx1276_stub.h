#ifndef LIFETRAC_MURATA_L072_BENCH_SX1276_STUB_H
#define LIFETRAC_MURATA_L072_BENCH_SX1276_STUB_H

#include <stdbool.h>
#include <stdint.h>

void sx1276_stub_reset(void);
uint8_t sx1276_stub_last_tx_power_dbm(void);
uint32_t sx1276_stub_tx_power_call_count(void);
uint16_t sx1276_stub_last_bw_khz(void);

/*
 * Defined in fhss_stub.c, not sx1276_stub.c — targets that link the
 * real radio/sx1276_airtime.c or radio/sx1276_fhss.c must not see a
 * second definition. Declared here so the cfg-profile TUs get them
 * from the one bench header they already include.
 */
uint32_t sx1276_stub_last_budget_us(void);
uint64_t sx1276_stub_last_fhss_farm_id(void);
uint64_t sx1276_stub_last_fhss_node_id(void);
uint32_t sx1276_stub_last_fhss_initial_epoch(void);
uint32_t sx1276_stub_fhss_init_call_count(void);
void     sx1276_stub_fhss_capture_reset(void);

#endif /* LIFETRAC_MURATA_L072_BENCH_SX1276_STUB_H */
