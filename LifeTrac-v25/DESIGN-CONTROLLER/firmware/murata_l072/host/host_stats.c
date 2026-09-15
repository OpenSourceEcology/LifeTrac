#include "host_stats.h"

#include "host_uart.h"
#include "sx1276.h"
#include "sx1276_modes.h"

#include <stdint.h>
#include <string.h>

typedef struct host_stats_wire_s {
    uint32_t host_dropped;
    uint32_t host_errors;
    uint32_t host_queue_full;
    uint32_t host_irq_idle;
    uint32_t host_irq_ht;
    uint32_t host_irq_tc;
    uint32_t host_irq_te;
    uint32_t radio_dio0;
    uint32_t radio_dio1;
    uint32_t radio_dio2;
    uint32_t radio_dio3;
    uint32_t radio_crc_err;
    uint32_t radio_rx_ok;
    uint32_t radio_tx_ok;
    uint32_t radio_tx_abort_lbt;
    uint32_t radio_tx_abort_airtime;
    uint32_t radio_state;
    uint32_t host_rx_bytes;
    uint32_t host_rx_lpuart_bytes;
    uint32_t host_rx_usart1_bytes;
    uint32_t host_parse_ok;
    uint32_t host_parse_err;
    uint32_t host_uart_err_lpuart;
    uint32_t host_uart_err_usart1;
    uint32_t host_uart_pe_lpuart;
    uint32_t host_uart_fe_lpuart;
    uint32_t host_uart_ne_lpuart;
    uint32_t host_uart_ore_lpuart;
    uint32_t host_uart_pe_usart1;
    uint32_t host_uart_fe_usart1;
    uint32_t host_uart_ne_usart1;
    uint32_t host_uart_ore_usart1;
    uint32_t host_rx_ring_ovf;
    /* RS-11.5 additive tail (2026-08-02): TX FIFO readback discriminator. */
    uint32_t tx_fifo_rb_ok;
    uint32_t tx_fifo_rb_bad;
    uint32_t tx_done_early;
    /* RS-12 additive tail (2026-09-12): URC-path loss accounting. */
    uint32_t rx_urc_lost;
    uint32_t rx_pretx_drained;
    /* RS-12.10 additive tail (2026-09-12): FIFO skip + TX deaf window. */
    uint32_t rx_fifo_skip;
    uint32_t tx_deaf_max_us;
    uint32_t tx_deaf_sum_us;
    uint32_t tx_done_to_rearm_max_us;
    /* RS-12.15 v2 additive tail (2026-09-14): FHSS clock authority. */
    uint32_t fhss_dec_aligned;
    uint32_t fhss_dec_snapped;
    uint32_t fhss_dec_rej_not_init;
    uint32_t fhss_dec_rej_bad_hop;
    uint32_t fhss_dec_rej_epoch_drift;
    uint32_t fhss_dec_rej_locked_out;
    uint32_t clk_demotion_reset;
    uint32_t clk_demotion_kept;
    uint32_t tx_first_anchor;
    uint32_t tx_stream_streak_max;
} host_stats_wire_t;

_Static_assert(sizeof(host_stats_wire_t) == HOST_STATS_PAYLOAD_LEN,
               "HOST_STATS payload length mismatch");

static uint32_t s_radio_dio0;
static uint32_t s_radio_dio1;
static uint32_t s_radio_dio2;
static uint32_t s_radio_dio3;
static uint32_t s_radio_crc_err;
static uint32_t s_radio_rx_ok;
static uint32_t s_radio_tx_ok;
static uint32_t s_radio_tx_abort_lbt;
static uint32_t s_radio_tx_abort_airtime;
static uint32_t s_tx_fifo_rb_ok;
static uint32_t s_tx_fifo_rb_bad;
static uint32_t s_tx_done_early;
static uint32_t s_rx_urc_lost;
static uint32_t s_rx_pretx_drained;
static uint32_t s_rx_fifo_skip;
static uint32_t s_tx_deaf_max_us;
static uint32_t s_tx_deaf_sum_us;
static uint32_t s_tx_done_to_rearm_max_us;
/* RS-12.15 v2 */
static uint32_t s_fhss_dec[6];
static uint32_t s_clk_demotion_reset;
static uint32_t s_clk_demotion_kept;
static uint32_t s_tx_first_anchor;
static uint32_t s_tx_stream_streak_max;

static void put_u32_le(uint8_t *dst, uint32_t value) {
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24) & 0xFFU);
}

void host_stats_reset(void) {
    s_radio_dio0 = 0U;
    s_radio_dio1 = 0U;
    s_radio_dio2 = 0U;
    s_radio_dio3 = 0U;
    s_radio_crc_err = 0U;
    s_radio_rx_ok = 0U;
    s_radio_tx_ok = 0U;
    s_radio_tx_abort_lbt = 0U;
    s_radio_tx_abort_airtime = 0U;
    s_tx_fifo_rb_ok = 0U;
    s_tx_fifo_rb_bad = 0U;
    s_tx_done_early = 0U;
    s_rx_urc_lost = 0U;
    s_rx_pretx_drained = 0U;
    s_rx_fifo_skip = 0U;
    s_tx_deaf_max_us = 0U;
    s_tx_deaf_sum_us = 0U;
    s_tx_done_to_rearm_max_us = 0U;
    for (uint8_t i = 0U; i < 6U; ++i) {
        s_fhss_dec[i] = 0U;
    }
    s_clk_demotion_reset = 0U;
    s_clk_demotion_kept = 0U;
    s_tx_first_anchor = 0U;
    s_tx_stream_streak_max = 0U;

    host_uart_stats_reset();
}

void host_stats_tx_fifo_rb_ok(void) {
    s_tx_fifo_rb_ok++;
}

void host_stats_tx_fifo_rb_bad(void) {
    s_tx_fifo_rb_bad++;
}

void host_stats_tx_done_early(void) {
    s_tx_done_early++;
}

void host_stats_radio_rx_ok(void) {
    s_radio_rx_ok++;
}

void host_stats_radio_crc_err(void) {
    s_radio_crc_err++;
}

void host_stats_radio_tx_ok(void) {
    s_radio_tx_ok++;
}

void host_stats_radio_tx_abort_lbt(void) {
    s_radio_tx_abort_lbt++;
}

void host_stats_radio_tx_abort_airtime(void) {
    s_radio_tx_abort_airtime++;
}

void host_stats_rx_urc_lost_add(uint32_t n) {
    s_rx_urc_lost = (s_rx_urc_lost > UINT32_MAX - n) ? UINT32_MAX : (s_rx_urc_lost + n);
}

void host_stats_rx_pretx_drained_add(uint32_t n) {
    s_rx_pretx_drained = (s_rx_pretx_drained > UINT32_MAX - n) ? UINT32_MAX : (s_rx_pretx_drained + n);
}

void host_stats_note_radio_events(uint32_t events) {
    if ((events & SX1276_EVT_DIO0) != 0U) {
        s_radio_dio0++;
    }
    if ((events & SX1276_EVT_DIO1) != 0U) {
        s_radio_dio1++;
    }
    if ((events & SX1276_EVT_DIO2) != 0U) {
        s_radio_dio2++;
    }
    if ((events & SX1276_EVT_DIO3) != 0U) {
        s_radio_dio3++;
    }
}

void host_stats_rx_fifo_skip(void) {
    if (s_rx_fifo_skip != UINT32_MAX) {
        s_rx_fifo_skip++;
    }
}

void host_stats_tx_deaf_note(uint32_t deaf_us, uint32_t done_to_rearm_us,
                             bool done_valid) {
    if (deaf_us > s_tx_deaf_max_us) {
        s_tx_deaf_max_us = deaf_us;
    }
    s_tx_deaf_sum_us = (s_tx_deaf_sum_us > (UINT32_MAX - deaf_us))
        ? UINT32_MAX : (s_tx_deaf_sum_us + deaf_us);
    if (done_valid && done_to_rearm_us > s_tx_done_to_rearm_max_us) {
        s_tx_done_to_rearm_max_us = done_to_rearm_us;
    }
}

/* RS-12.15 v2 (2026-09-14): FHSS clock-authority counters. */
void host_stats_fhss_dec_note(uint8_t dec_idx) {
    if (dec_idx < 6U && s_fhss_dec[dec_idx] != 0xFFFFFFFFU) {
        ++s_fhss_dec[dec_idx];
    }
}

void host_stats_clk_demotion_note(bool reset) {
    if (reset) {
        if (s_clk_demotion_reset != 0xFFFFFFFFU) { ++s_clk_demotion_reset; }
    } else {
        if (s_clk_demotion_kept != 0xFFFFFFFFU) { ++s_clk_demotion_kept; }
    }
}

void host_stats_tx_first_anchor(void) {
    if (s_tx_first_anchor != 0xFFFFFFFFU) { ++s_tx_first_anchor; }
}

void host_stats_tx_stream_streak_note(uint32_t streak) {
    if (streak > s_tx_stream_streak_max) { s_tx_stream_streak_max = streak; }
}

uint16_t host_stats_serialize(uint8_t *out, uint16_t out_cap) {
    uint16_t idx = 0U;

    if (out == NULL || out_cap < HOST_STATS_PAYLOAD_LEN) {
        return 0U;
    }

    memset(out, 0, HOST_STATS_PAYLOAD_LEN);

    put_u32_le(&out[idx], host_uart_stats_dropped()); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], host_uart_stats_errors()); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], host_uart_stats_queue_full()); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], host_uart_stats_irq_idle()); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], host_uart_stats_irq_ht()); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], host_uart_stats_irq_tc()); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], host_uart_stats_irq_te()); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], s_radio_dio0); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], s_radio_dio1); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], s_radio_dio2); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], s_radio_dio3); idx = (uint16_t)(idx + 4U);

    put_u32_le(&out[idx], s_radio_crc_err); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], s_radio_rx_ok); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], s_radio_tx_ok); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], s_radio_tx_abort_lbt); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], s_radio_tx_abort_airtime); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], (uint32_t)sx1276_modes_get_state());
    idx = (uint16_t)(idx + 4U);

    put_u32_le(&out[idx], host_uart_stats_rx_bytes()); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], host_uart_stats_rx_lpuart_bytes()); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], host_uart_stats_rx_usart1_bytes()); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], host_uart_stats_parse_ok()); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], host_uart_stats_parse_err()); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], host_uart_stats_uart_err_lpuart()); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], host_uart_stats_uart_err_usart1()); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], host_uart_stats_uart_pe_lpuart());  idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], host_uart_stats_uart_fe_lpuart());  idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], host_uart_stats_uart_ne_lpuart());  idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], host_uart_stats_uart_ore_lpuart()); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], host_uart_stats_uart_pe_usart1());  idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], host_uart_stats_uart_fe_usart1());  idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], host_uart_stats_uart_ne_usart1());  idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], host_uart_stats_uart_ore_usart1()); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], host_uart_stats_rx_ring_ovf()); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], s_tx_fifo_rb_ok); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], s_tx_fifo_rb_bad); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], s_tx_done_early); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], s_rx_urc_lost); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], s_rx_pretx_drained); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], s_rx_fifo_skip); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], s_tx_deaf_max_us); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], s_tx_deaf_sum_us); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], s_tx_done_to_rearm_max_us); idx = (uint16_t)(idx + 4U);
    /* RS-12.15 v2 additive tail (offsets pinned by check-stats-layout). */
    for (uint8_t i = 0U; i < 6U; ++i) {
        put_u32_le(&out[idx], s_fhss_dec[i]); idx = (uint16_t)(idx + 4U);
    }
    put_u32_le(&out[idx], s_clk_demotion_reset); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], s_clk_demotion_kept); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], s_tx_first_anchor); idx = (uint16_t)(idx + 4U);
    put_u32_le(&out[idx], s_tx_stream_streak_max); idx = (uint16_t)(idx + 4U);

    return HOST_STATS_PAYLOAD_LEN;
}
