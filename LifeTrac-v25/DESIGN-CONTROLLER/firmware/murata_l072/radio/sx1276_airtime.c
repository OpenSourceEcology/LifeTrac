#include "sx1276_airtime.h"

#include "sx1276.h"

#define SX1276_REG_MODEM_CONFIG1            0x1DU
#define SX1276_REG_MODEM_CONFIG2            0x1EU
#define SX1276_REG_PREAMBLE_MSB             0x20U
#define SX1276_REG_PREAMBLE_LSB             0x21U
#define SX1276_REG_MODEM_CONFIG3            0x26U

#define SX1276_AIRTIME_CHANNEL_COUNT        16U
#define SX1276_AIRTIME_WINDOW_MS            1000U
#define SX1276_AIRTIME_BUDGET_US            400000U

typedef struct sx1276_airtime_budget_s {
    uint32_t window_start_ms;
    uint32_t used_us;
} sx1276_airtime_budget_t;

static sx1276_airtime_budget_t s_budget[SX1276_AIRTIME_CHANNEL_COUNT];

static uint32_t bw_hz_from_bits(uint8_t bw_bits) {
    if (bw_bits >= 9U) {
        return 500000UL;
    }
    if (bw_bits == 8U) {
        return 250000UL;
    }
    return 125000UL;
}

static uint32_t ceil_div_u32(uint32_t numer, uint32_t denom) {
    return (numer + denom - 1U) / denom;
}

static void refresh_budget_window(uint8_t channel_idx, uint32_t now_ms) {
    if ((uint32_t)(now_ms - s_budget[channel_idx].window_start_ms) >= SX1276_AIRTIME_WINDOW_MS) {
        s_budget[channel_idx].window_start_ms = now_ms;
        s_budget[channel_idx].used_us = 0U;
    }
}

uint32_t sx1276_airtime_estimate_toa_us(uint8_t payload_len) {
    const uint8_t modem_cfg1 = sx1276_read_reg(SX1276_REG_MODEM_CONFIG1);
    const uint8_t modem_cfg2 = sx1276_read_reg(SX1276_REG_MODEM_CONFIG2);
    const uint8_t modem_cfg3 = sx1276_read_reg(SX1276_REG_MODEM_CONFIG3);
    const uint8_t preamble_msb = sx1276_read_reg(SX1276_REG_PREAMBLE_MSB);
    const uint8_t preamble_lsb = sx1276_read_reg(SX1276_REG_PREAMBLE_LSB);
    uint8_t sf = (uint8_t)((modem_cfg2 >> 4) & 0x0FU);
    const uint8_t bw_bits = (uint8_t)((modem_cfg1 >> 4) & 0x0FU);
    const uint8_t coding_rate_den = (uint8_t)(((modem_cfg1 >> 1) & 0x07U) + 4U);
    const uint8_t implicit_header = (uint8_t)(modem_cfg1 & 0x01U);
    const uint8_t crc_on = (uint8_t)((modem_cfg2 >> 2) & 0x01U);
    const uint8_t low_dr_opt = (uint8_t)((modem_cfg3 >> 3) & 0x01U);
    uint32_t preamble_symbols = (uint32_t)(((uint16_t)preamble_msb << 8) | preamble_lsb);
    const uint32_t bw_hz = bw_hz_from_bits(bw_bits);
    uint64_t t_sym_num_base;
    uint32_t t_sym_us;
    int32_t numerator;
    uint32_t denominator;
    uint32_t payload_symbols;
    uint32_t preamble_us;
    uint32_t payload_us;

    if (sf < 6U) {
        sf = 6U;
    }
    if (sf > 12U) {
        sf = 12U;
    }

    if (preamble_symbols == 0U) {
        preamble_symbols = 8U;
    }

    t_sym_num_base = ((uint64_t)1U << sf) * 1000000ULL;
    t_sym_us = (uint32_t)((t_sym_num_base + (uint64_t)bw_hz - 1ULL) / (uint64_t)bw_hz);

    numerator = ((int32_t)8 * (int32_t)payload_len) -
                ((int32_t)4 * (int32_t)sf) +
                28 +
                ((int32_t)16 * (int32_t)crc_on) -
                ((int32_t)20 * (int32_t)implicit_header);
    denominator = 4U * ((uint32_t)sf - (2U * low_dr_opt));

    if (numerator <= 0) {
        payload_symbols = 8U;
    } else {
        payload_symbols = 8U + (ceil_div_u32((uint32_t)numerator, denominator) * coding_rate_den);
    }

    preamble_us = ((preamble_symbols + 4U) * t_sym_us) + (t_sym_us / 4U);
    payload_us = payload_symbols * t_sym_us;
    return preamble_us + payload_us;
}

sx1276_airtime_result_t sx1276_airtime_reserve(uint8_t channel_idx,
                                                uint8_t payload_len,
                                                uint32_t now_ms,
                                                uint32_t *out_reserved_us) {
    const uint32_t estimate_us = sx1276_airtime_estimate_toa_us(payload_len);

    if (channel_idx >= SX1276_AIRTIME_CHANNEL_COUNT) {
        return SX1276_AIRTIME_BAD_INPUT;
    }

    refresh_budget_window(channel_idx, now_ms);

    if ((SX1276_AIRTIME_BUDGET_US - s_budget[channel_idx].used_us) < estimate_us) {
        return SX1276_AIRTIME_OVER_BUDGET;
    }

    s_budget[channel_idx].used_us += estimate_us;
    if (out_reserved_us != 0) {
        *out_reserved_us = estimate_us;
    }

    return SX1276_AIRTIME_OK;
}

void sx1276_airtime_release(uint8_t channel_idx, uint32_t reserved_us) {
    if (channel_idx >= SX1276_AIRTIME_CHANNEL_COUNT) {
        return;
    }

    if (reserved_us >= s_budget[channel_idx].used_us) {
        s_budget[channel_idx].used_us = 0U;
        return;
    }

    s_budget[channel_idx].used_us -= reserved_us;
}

void sx1276_airtime_commit(uint8_t channel_idx, uint32_t used_us, uint32_t now_ms) {
    if (channel_idx >= SX1276_AIRTIME_CHANNEL_COUNT) {
        return;
    }

    refresh_budget_window(channel_idx, now_ms);

    if (used_us == 0U) {
        return;
    }

    if ((SX1276_AIRTIME_BUDGET_US - s_budget[channel_idx].used_us) < used_us) {
        s_budget[channel_idx].used_us = SX1276_AIRTIME_BUDGET_US;
        return;
    }

    s_budget[channel_idx].used_us += used_us;
}
