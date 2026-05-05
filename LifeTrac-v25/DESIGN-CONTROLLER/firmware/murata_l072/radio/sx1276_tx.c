#include "sx1276_tx.h"

#include "host_stats.h"
#include "platform.h"
#include "sx1276_airtime.h"
#include "sx1276.h"
#include "sx1276_lbt.h"
#include "sx1276_modes.h"
#include "sx1276_rx.h"

#define SX1276_REG_FIFO                    0x00U
#define SX1276_REG_FIFO_ADDR_PTR           0x0DU
#define SX1276_REG_FIFO_TX_BASE_ADDR       0x0EU
#define SX1276_REG_IRQ_FLAGS               0x12U
#define SX1276_REG_PAYLOAD_LENGTH          0x22U
#define SX1276_REG_PA_CONFIG               0x09U

#define SX1276_IRQ_TX_DONE                 0x08U

#define SX1276_TX_TIMEOUT_GUARD_US         50000UL

typedef enum sx1276_tx_state_e {
    SX1276_TX_STATE_IDLE = 0,
    SX1276_TX_STATE_WAIT_DONE = 1
} sx1276_tx_state_t;

static sx1276_tx_state_t s_tx_state = SX1276_TX_STATE_IDLE;
static uint8_t s_tx_id;
static uint8_t s_tx_power_dbm;
static uint8_t s_rearm_rx;
static uint32_t s_expected_toa_us;
static uint32_t s_deadline_us;
static uint32_t s_reserved_airtime_us;
static uint8_t s_channel_idx;

static uint8_t tx_power_dbm_now(void) {
    return (uint8_t)((sx1276_read_reg(SX1276_REG_PA_CONFIG) & 0x0FU) + 2U);
}

static bool time_reached(uint32_t now, uint32_t deadline) {
    return (int32_t)(now - deadline) >= 0;
}

static void sx1276_tx_cleanup(void) {
    (void)sx1276_modes_to_standby();
    if (s_rearm_rx != 0U) {
        (void)sx1276_rx_arm();
    }

    s_tx_state = SX1276_TX_STATE_IDLE;
    s_rearm_rx = 0U;
    s_reserved_airtime_us = 0U;
}

bool sx1276_tx_begin(const sx1276_tx_request_t *req) {
    const sx1276_state_t state_before = sx1276_modes_get_state();
    sx1276_lbt_result_t lbt_result;
    sx1276_airtime_result_t airtime_result;

    if (req == NULL || s_tx_state != SX1276_TX_STATE_IDLE) {
        return false;
    }

    s_channel_idx = 0U;

    s_rearm_rx = (state_before == SX1276_STATE_RX_CONT || state_before == SX1276_STATE_RX_SINGLE) ? 1U : 0U;
    if (s_rearm_rx != 0U) {
        sx1276_rx_disarm();
    } else if (!sx1276_modes_to_standby()) {
        return false;
    }

    lbt_result = sx1276_lbt_check_and_backoff();
    if (lbt_result != SX1276_LBT_RESULT_CLEAR && lbt_result != SX1276_LBT_RESULT_DISABLED) {
        sx1276_tx_cleanup();
        return false;
    }

    airtime_result = sx1276_airtime_reserve(s_channel_idx,
                                            req->length,
                                            platform_now_ms(),
                                            &s_reserved_airtime_us);
    if (airtime_result != SX1276_AIRTIME_OK) {
        if (airtime_result == SX1276_AIRTIME_OVER_BUDGET) {
            host_stats_radio_tx_abort_airtime();
        }
        sx1276_tx_cleanup();
        return false;
    }

    sx1276_write_reg(SX1276_REG_FIFO_TX_BASE_ADDR, 0x00U);
    sx1276_write_reg(SX1276_REG_FIFO_ADDR_PTR, 0x00U);
    sx1276_write_reg(SX1276_REG_PAYLOAD_LENGTH, req->length);
    if (req->length > 0U) {
        sx1276_write_burst(SX1276_REG_FIFO, req->payload, req->length);
    }
    sx1276_write_reg(SX1276_REG_IRQ_FLAGS, 0xFFU);

    if (!sx1276_modes_to_tx()) {
        sx1276_airtime_release(s_channel_idx, s_reserved_airtime_us);
        sx1276_tx_cleanup();
        return false;
    }

    s_tx_id = req->tx_id;
    s_tx_power_dbm = tx_power_dbm_now();
    s_expected_toa_us = s_reserved_airtime_us;
    if (s_expected_toa_us == 0U) {
        s_expected_toa_us = sx1276_airtime_estimate_toa_us(req->length);
    }
    s_deadline_us = platform_now_us() + s_expected_toa_us + SX1276_TX_TIMEOUT_GUARD_US;
    s_tx_state = SX1276_TX_STATE_WAIT_DONE;
    return true;
}

bool sx1276_tx_poll(uint32_t events, sx1276_tx_result_t *out_result) {
    uint8_t irq_flags;

    if (out_result == NULL || s_tx_state != SX1276_TX_STATE_WAIT_DONE) {
        return false;
    }

    if ((events & SX1276_EVT_DIO0) != 0U) {
        irq_flags = sx1276_read_reg(SX1276_REG_IRQ_FLAGS);
        sx1276_write_reg(SX1276_REG_IRQ_FLAGS, irq_flags);

        if ((irq_flags & SX1276_IRQ_TX_DONE) != 0U) {
            out_result->tx_id = s_tx_id;
            out_result->status = SX1276_TX_STATUS_OK;
            out_result->tx_power_dbm = s_tx_power_dbm;
            out_result->time_on_air_us = s_expected_toa_us;
            sx1276_airtime_commit(s_channel_idx, 0U, platform_now_ms());
            host_stats_radio_tx_ok();
            sx1276_tx_cleanup();
            return true;
        }
    }

    if (time_reached(platform_now_us(), s_deadline_us)) {
        sx1276_write_reg(SX1276_REG_IRQ_FLAGS, 0xFFU);
        out_result->tx_id = s_tx_id;
        out_result->status = SX1276_TX_STATUS_TIMEOUT;
        out_result->tx_power_dbm = s_tx_power_dbm;
        out_result->time_on_air_us = s_expected_toa_us;
        sx1276_airtime_commit(s_channel_idx, 0U, platform_now_ms());
        sx1276_tx_cleanup();
        return true;
    }

    return false;
}

bool sx1276_tx_busy(void) {
    return s_tx_state != SX1276_TX_STATE_IDLE;
}