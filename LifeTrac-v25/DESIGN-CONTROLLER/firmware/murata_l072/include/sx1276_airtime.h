#ifndef LIFETRAC_MURATA_L072_SX1276_AIRTIME_H
#define LIFETRAC_MURATA_L072_SX1276_AIRTIME_H

#include <stdint.h>

typedef enum sx1276_airtime_result_e {
    SX1276_AIRTIME_OK = 0,
    SX1276_AIRTIME_OVER_BUDGET = 1,
    SX1276_AIRTIME_BAD_INPUT = 2
} sx1276_airtime_result_t;

uint32_t sx1276_airtime_estimate_toa_us(uint8_t payload_len);
sx1276_airtime_result_t sx1276_airtime_reserve(uint8_t channel_idx,
                                                uint8_t payload_len,
                                                uint32_t now_ms,
                                                uint32_t *out_reserved_us);
void sx1276_airtime_release(uint8_t channel_idx, uint32_t reserved_us);
void sx1276_airtime_commit(uint8_t channel_idx, uint32_t used_us, uint32_t now_ms);

#endif /* LIFETRAC_MURATA_L072_SX1276_AIRTIME_H */
