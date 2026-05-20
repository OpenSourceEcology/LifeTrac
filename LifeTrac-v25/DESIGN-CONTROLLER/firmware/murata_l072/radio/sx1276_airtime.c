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

uint32_t sx1276_airtime_compute_toa_us(uint8_t sf,
                                       uint32_t bw_hz,
                                       uint8_t cr_den,
                                       uint8_t payload_len,
                                       uint8_t crc_on,
                                       uint8_t implicit_header,
                                       uint16_t preamble_symbols,
                                       uint8_t low_dr_opt) {
    uint8_t sf_use = sf;
    uint8_t cr_use = cr_den;
    uint64_t t_sym_num_base;
    uint32_t t_sym_us;
    int32_t numerator;
    uint32_t denominator;
    uint32_t payload_symbols;
    uint32_t preamble_us;
    uint32_t payload_us;
    uint32_t preamble_use = preamble_symbols;

    if (sf_use < 6U) {
        sf_use = 6U;
    }
    if (sf_use > 12U) {
        sf_use = 12U;
    }
    if (cr_use < 5U) {
        cr_use = 5U;
    }
    if (cr_use > 8U) {
        cr_use = 8U;
    }
    if (bw_hz == 0U) {
        return 0U; /* guard: caller passed a bogus tuple */
    }
    if (preamble_use == 0U) {
        preamble_use = 8U;
    }

    t_sym_num_base = ((uint64_t)1U << sf_use) * 1000000ULL;
    t_sym_us = (uint32_t)((t_sym_num_base + (uint64_t)bw_hz - 1ULL) / (uint64_t)bw_hz);

    numerator = ((int32_t)8 * (int32_t)payload_len) -
                ((int32_t)4 * (int32_t)sf_use) +
                28 +
                ((int32_t)16 * (int32_t)crc_on) -
                ((int32_t)20 * (int32_t)implicit_header);
    denominator = 4U * ((uint32_t)sf_use - (2U * (uint32_t)low_dr_opt));
    if (denominator == 0U) {
        denominator = 1U; /* defensive: should be unreachable for sf>=6 */
    }

    if (numerator <= 0) {
        payload_symbols = 8U;
    } else {
        /* Semtech datasheet: payload_symbols = 8 + ceil(numer/denom) * cr_den. */
        payload_symbols = 8U + (ceil_div_u32((uint32_t)numerator, denominator) * (uint32_t)cr_use);
    }

    preamble_us = ((preamble_use + 4U) * t_sym_us) + (t_sym_us / 4U);
    payload_us = payload_symbols * t_sym_us;
    return preamble_us + payload_us;
}

bool sx1276_airtime_config_invariant_ok(uint8_t sf,
                                        uint16_t bw_khz,
                                        uint8_t cr_den,
                                        uint8_t max_payload_len,
                                        uint32_t cap_us,
                                        uint32_t *out_computed_toa_us) {
    /*
     * Match the framing assumptions baked into sx1276_set_sf_bw_cr():
     *   - explicit header (implicit=0)
     *   - CRC on  (modem_cfg2 bit2 = 1)
     *   - default preamble of 8 symbols
     *   - low-DR-opt automatically asserted when SF >= 11 and BW <= 125 kHz
     */
    const uint8_t crc_on = 1U;
    const uint8_t implicit_header = 0U;
    const uint16_t preamble_symbols = 8U;
    const uint8_t low_dr_opt = ((sf >= 11U) && (bw_khz <= 125U)) ? 1U : 0U;
    const uint32_t bw_hz = (uint32_t)bw_khz * 1000UL;

    const uint32_t toa_us = sx1276_airtime_compute_toa_us(sf,
                                                          bw_hz,
                                                          cr_den,
                                                          max_payload_len,
                                                          crc_on,
                                                          implicit_header,
                                                          preamble_symbols,
                                                          low_dr_opt);
    if (out_computed_toa_us != 0) {
        *out_computed_toa_us = toa_us;
    }
    return (toa_us != 0U) && (toa_us <= cap_us);
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
    const uint16_t preamble_symbols = (uint16_t)(((uint16_t)preamble_msb << 8) | preamble_lsb);
    const uint32_t bw_hz = bw_hz_from_bits(bw_bits);

    if (sf < 6U) {
        sf = 6U;
    }
    if (sf > 12U) {
        sf = 12U;
    }

    return sx1276_airtime_compute_toa_us(sf,
                                         bw_hz,
                                         coding_rate_den,
                                         payload_len,
                                         crc_on,
                                         implicit_header,
                                         preamble_symbols,
                                         low_dr_opt);
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
