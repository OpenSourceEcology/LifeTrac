#ifndef LIFETRAC_MURATA_L072_BENCH_SX1276_STUB_H
#define LIFETRAC_MURATA_L072_BENCH_SX1276_STUB_H

#include <stdbool.h>
#include <stdint.h>

void sx1276_stub_reset(void);
uint8_t sx1276_stub_last_tx_power_dbm(void);
uint32_t sx1276_stub_tx_power_call_count(void);
uint16_t sx1276_stub_last_bw_khz(void);

#endif /* LIFETRAC_MURATA_L072_BENCH_SX1276_STUB_H */
