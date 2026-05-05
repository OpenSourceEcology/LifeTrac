#include "sx1276_lbt.h"

#include "config.h"
#include "host_cfg.h"
#include "host_cfg_keys.h"
#include "host_stats.h"
#include "platform.h"
#include "sx1276.h"
#include "sx1276_cad.h"
#include "sx1276_modes.h"

#define SX1276_REG_RSSI_VALUE              0x1BU

#define LBT_DEFAULT_THRESHOLD_DBM          (-90)
#define LBT_DEFAULT_MAX_BACKOFF_MS         500U
#define LBT_DEFAULT_CAD_SYMBOLS            4U
#define LBT_CAD_TIMEOUT_US                 20000UL
#define LBT_BASE_BACKOFF_MS                10U

static uint32_t s_backoff_until_ms;
static uint8_t s_backoff_attempt;

static uint8_t cfg_read_u8(uint8_t key, uint8_t fallback) {
    uint8_t out = fallback;
    uint8_t out_len = 0U;

    if (cfg_get(key, &out, sizeof(out), &out_len) != CFG_STATUS_OK || out_len != 1U) {
        return fallback;
    }

    return out;
}

static uint16_t cfg_read_u16(uint8_t key, uint16_t fallback) {
    uint8_t out[2] = { 0U, 0U };
    uint8_t out_len = 0U;

    if (cfg_get(key, out, sizeof(out), &out_len) != CFG_STATUS_OK || out_len != 2U) {
        return fallback;
    }

    return (uint16_t)out[0] | ((uint16_t)out[1] << 8);
}

static uint16_t backoff_remaining_ms_internal(uint32_t now_ms) {
    if ((int32_t)(now_ms - s_backoff_until_ms) >= 0) {
        return 0U;
    }

    return (uint16_t)(s_backoff_until_ms - now_ms);
}

static void lbt_arm_backoff(uint16_t max_backoff_ms) {
    uint32_t backoff_ms;
    uint8_t shift;

    shift = s_backoff_attempt;
    if (shift > 6U) {
        shift = 6U;
    }

    backoff_ms = (uint32_t)LBT_BASE_BACKOFF_MS << shift;
    if (backoff_ms > max_backoff_ms) {
        backoff_ms = max_backoff_ms;
    }

    s_backoff_until_ms = platform_now_ms() + backoff_ms;
    if (s_backoff_attempt < 0xFFU) {
        s_backoff_attempt++;
    }
}

static void lbt_clear_backoff(void) {
    s_backoff_until_ms = 0U;
    s_backoff_attempt = 0U;
}

uint16_t sx1276_lbt_backoff_remaining_ms(void) {
    return backoff_remaining_ms_internal(platform_now_ms());
}

sx1276_lbt_result_t sx1276_lbt_check_and_backoff(void) {
    const uint8_t lbt_enabled = cfg_read_u8(CFG_KEY_LBT_ENABLE, LORA_FW_LBT_ENABLE);
    const int16_t threshold_dbm = (int16_t)(int8_t)cfg_read_u8(CFG_KEY_LBT_THRESHOLD_DBM,
                                                               (uint8_t)LBT_DEFAULT_THRESHOLD_DBM);
    const uint16_t max_backoff_ms = cfg_read_u16(CFG_KEY_LBT_MAX_BACKOFF_MS,
                                                 LBT_DEFAULT_MAX_BACKOFF_MS);
    const uint8_t cad_symbols = cfg_read_u8(CFG_KEY_CAD_SYMBOLS, LBT_DEFAULT_CAD_SYMBOLS);
    const uint32_t sample_ms = (cad_symbols == 0U) ? 1U : (uint32_t)cad_symbols;
    const uint32_t cad_deadline_us = platform_now_us() + LBT_CAD_TIMEOUT_US;
    int16_t rssi_dbm;

    if (lbt_enabled == 0U) {
        return SX1276_LBT_RESULT_DISABLED;
    }

    if (backoff_remaining_ms_internal(platform_now_ms()) != 0U) {
        host_stats_radio_tx_abort_lbt();
        return SX1276_LBT_RESULT_BACKOFF;
    }

    if (cad_symbols > 0U) {
        sx1276_cad_result_t cad_result;

        if (!sx1276_cad_begin()) {
            return SX1276_LBT_RESULT_ERROR;
        }

        do {
            cad_result = sx1276_cad_poll();
            if (cad_result == SX1276_CAD_RESULT_DETECTED) {
                lbt_arm_backoff(max_backoff_ms);
                host_stats_radio_tx_abort_lbt();
                return SX1276_LBT_RESULT_BUSY;
            }
            if (cad_result == SX1276_CAD_RESULT_ERROR) {
                return SX1276_LBT_RESULT_ERROR;
            }
            if (cad_result == SX1276_CAD_RESULT_CLEAR) {
                break;
            }
        } while ((int32_t)(platform_now_us() - cad_deadline_us) < 0);

        if ((int32_t)(platform_now_us() - cad_deadline_us) >= 0) {
            (void)sx1276_modes_to_standby();
            return SX1276_LBT_RESULT_ERROR;
        }
    }

    if (!sx1276_modes_to_rx_cont()) {
        return SX1276_LBT_RESULT_ERROR;
    }

    platform_delay_ms(sample_ms);
    rssi_dbm = (int16_t)((int16_t)sx1276_read_reg(SX1276_REG_RSSI_VALUE) - 157);
    (void)sx1276_modes_to_standby();

    if (rssi_dbm > threshold_dbm) {
        lbt_arm_backoff(max_backoff_ms);
        host_stats_radio_tx_abort_lbt();
        return SX1276_LBT_RESULT_BUSY;
    }

    lbt_clear_backoff();
    return SX1276_LBT_RESULT_CLEAR;
}
