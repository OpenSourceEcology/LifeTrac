/*
 * stats_layout — pins host_stats_serialize() against the HOST_STATS_OFFSET_*
 * contract in host_types.h.
 *
 * Written after the 2026-09-12 RS-12.10 slip: the serializer's last line
 * had no index advance (nothing followed it), so four fields appended after
 * it landed one slot early, clobbered rx_pretx_drained at offset 148, and
 * the last label read the zeroed tail. The H7 side has mh_stats_vectors
 * for its parser; this is the producer-side counterpart. Every additive
 * tail must extend the table below — the test fails on a length change
 * it does not know about.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "host_stats.h"
#include "host_types.h"
#include "sx1276.h"          /* SX1276_EVT_DIO0 */
#include "sx1276_modes.h"

/* ---- stubs for the host_stats.c externs (values chosen to be unique) ---- */
uint32_t host_uart_stats_dropped(void)            { return 0x01010101UL; }
uint32_t host_uart_stats_errors(void)             { return 0x02020202UL; }
uint32_t host_uart_stats_queue_full(void)         { return 0x03030303UL; }
uint32_t host_uart_stats_irq_idle(void)           { return 0x04040404UL; }
uint32_t host_uart_stats_irq_ht(void)             { return 0x05050505UL; }
uint32_t host_uart_stats_irq_tc(void)             { return 0x06060606UL; }
uint32_t host_uart_stats_irq_te(void)             { return 0x07070707UL; }
uint32_t host_uart_stats_rx_bytes(void)           { return 0x11111111UL; }
uint32_t host_uart_stats_rx_lpuart_bytes(void)    { return 0x12121212UL; }
uint32_t host_uart_stats_rx_usart1_bytes(void)    { return 0x13131313UL; }
uint32_t host_uart_stats_parse_ok(void)           { return 0x14141414UL; }
uint32_t host_uart_stats_parse_err(void)          { return 0x15151515UL; }
uint32_t host_uart_stats_uart_err_lpuart(void)    { return 0x16161616UL; }
uint32_t host_uart_stats_uart_err_usart1(void)    { return 0x17171717UL; }
uint32_t host_uart_stats_uart_pe_lpuart(void)     { return 0x18181818UL; }
uint32_t host_uart_stats_uart_fe_lpuart(void)     { return 0x19191919UL; }
uint32_t host_uart_stats_uart_ne_lpuart(void)     { return 0x1A1A1A1AUL; }
uint32_t host_uart_stats_uart_ore_lpuart(void)    { return 0x1B1B1B1BUL; }
uint32_t host_uart_stats_uart_pe_usart1(void)     { return 0x1C1C1C1CUL; }
uint32_t host_uart_stats_uart_fe_usart1(void)     { return 0x1D1D1D1DUL; }
uint32_t host_uart_stats_uart_ne_usart1(void)     { return 0x1E1E1E1EUL; }
uint32_t host_uart_stats_uart_ore_usart1(void)    { return 0x1F1F1F1FUL; }
uint32_t host_uart_stats_rx_ring_ovf(void)        { return 0x20202020UL; }
void     host_uart_stats_reset(void)              { }
sx1276_state_t sx1276_modes_get_state(void)       { return SX1276_STATE_RX_CONT; }

static int s_fail;
static int s_cases;

static uint32_t rd32(const uint8_t *p, uint16_t off) {
    return (uint32_t)p[off] | ((uint32_t)p[off + 1] << 8) |
           ((uint32_t)p[off + 2] << 16) | ((uint32_t)p[off + 3] << 24);
}

static void expect_at(const uint8_t *p, uint16_t off, uint32_t want, const char *what) {
    const uint32_t got = rd32(p, off);
    s_cases++;
    if (got != want) {
        s_fail++;
        printf("[FAIL] %s @%u: got 0x%08lx want 0x%08lx\n", what, (unsigned)off,
               (unsigned long)got, (unsigned long)want);
    }
}

int main(void) {
    uint8_t out[HOST_STATS_PAYLOAD_LEN + 8];
    uint16_t n;

    host_stats_reset();
    /* radio counters via the radio-events path + the setters */
    host_stats_note_radio_events(SX1276_EVT_DIO0);             /* dio0 = 1 */
    host_stats_radio_crc_err(); host_stats_radio_crc_err();     /* 2 */
    host_stats_radio_rx_ok(); host_stats_radio_rx_ok(); host_stats_radio_rx_ok(); /* 3 */
    host_stats_radio_tx_ok();                                   /* 1 */
    host_stats_tx_fifo_rb_ok(); host_stats_tx_fifo_rb_ok();     /* 2 */
    host_stats_tx_fifo_rb_bad();                                /* 1 */
    host_stats_tx_done_early();                                 /* 1 */
    host_stats_rx_urc_lost_add(5U);                             /* 5 */
    host_stats_rx_pretx_drained_add(7U);                        /* 7 */
    host_stats_rx_fifo_skip(); host_stats_rx_fifo_skip();       /* 2 */
    host_stats_tx_deaf_note(30000U, 900U, true);                /* max 30000, sum 30000, turn 900 */
    host_stats_tx_deaf_note(12000U, 1500U, true);               /* max 30000, sum 42000, turn 1500 */
    host_stats_tx_deaf_note(45000U, 99999U, false);             /* max 45000, sum 87000, turn stays 1500 */
    /* RS-12.15 v2: decision histogram + clock-authority events */
    host_stats_fhss_dec_note(0U); host_stats_fhss_dec_note(0U); host_stats_fhss_dec_note(0U); /* aligned 3 */
    host_stats_fhss_dec_note(1U);                               /* snapped 1 */
    host_stats_fhss_dec_note(5U); host_stats_fhss_dec_note(5U); /* locked_out 2 */
    host_stats_fhss_dec_note(9U);                               /* out of range: ignored */
    host_stats_clk_demotion_note(true); host_stats_clk_demotion_note(true); /* reset 2 */
    host_stats_clk_demotion_note(false);                        /* kept 1 */
    host_stats_tx_first_anchor();                               /* 1 */
    host_stats_tx_stream_streak_note(7U); host_stats_tx_stream_streak_note(31U); host_stats_tx_stream_streak_note(9U); /* max 31 */

    memset(out, 0xEE, sizeof(out));
    n = host_stats_serialize(out, HOST_STATS_PAYLOAD_LEN);
    s_cases++;
    if (n != HOST_STATS_PAYLOAD_LEN) {
        s_fail++;
        printf("[FAIL] serialize returned %u, want %u\n", (unsigned)n, (unsigned)HOST_STATS_PAYLOAD_LEN);
    }
    s_cases++;
    if (out[HOST_STATS_PAYLOAD_LEN] != 0xEEU) {
        s_fail++;
        printf("[FAIL] serialize wrote past HOST_STATS_PAYLOAD_LEN\n");
    }

    expect_at(out, HOST_STATS_OFFSET_RADIO_DIO0,        1U, "radio_dio0");
    expect_at(out, HOST_STATS_OFFSET_RADIO_CRC_ERR,     2U, "radio_crc_err");
    expect_at(out, HOST_STATS_OFFSET_RADIO_RX_OK,       3U, "radio_rx_ok");
    expect_at(out, HOST_STATS_OFFSET_RADIO_TX_OK,       1U, "radio_tx_ok");
    expect_at(out, HOST_STATS_OFFSET_RADIO_STATE,       (uint32_t)SX1276_STATE_RX_CONT, "radio_state");
    expect_at(out, HOST_STATS_OFFSET_HOST_RX_BYTES,     0x11111111UL, "host_rx_bytes");
    expect_at(out, HOST_STATS_OFFSET_HOST_RX_RING_OVF,  0x20202020UL, "host_rx_ring_ovf");
    expect_at(out, HOST_STATS_OFFSET_TX_FIFO_RB_OK,     2U, "tx_fifo_rb_ok");
    expect_at(out, HOST_STATS_OFFSET_TX_FIFO_RB_BAD,    1U, "tx_fifo_rb_bad");
    expect_at(out, HOST_STATS_OFFSET_TX_DONE_EARLY,     1U, "tx_done_early");
    expect_at(out, HOST_STATS_OFFSET_RX_URC_LOST,       5U, "rx_urc_lost");
    expect_at(out, HOST_STATS_OFFSET_RX_PRETX_DRAINED,  7U, "rx_pretx_drained");
    expect_at(out, HOST_STATS_OFFSET_RX_FIFO_SKIP,      2U, "rx_fifo_skip");
    expect_at(out, HOST_STATS_OFFSET_TX_DEAF_MAX_US,    45000U, "tx_deaf_max_us");
    expect_at(out, HOST_STATS_OFFSET_TX_DEAF_SUM_US,    87000U, "tx_deaf_sum_us");
    expect_at(out, HOST_STATS_OFFSET_TX_DONE_TO_REARM_MAX_US, 1500U, "tx_done_to_rearm_max_us");
    expect_at(out, HOST_STATS_OFFSET_FHSS_DEC_ALIGNED,         3U, "fhss_dec_aligned");
    expect_at(out, HOST_STATS_OFFSET_FHSS_DEC_SNAPPED,         1U, "fhss_dec_snapped");
    expect_at(out, HOST_STATS_OFFSET_FHSS_DEC_REJ_NOT_INIT,    0U, "fhss_dec_rej_not_init");
    expect_at(out, HOST_STATS_OFFSET_FHSS_DEC_REJ_BAD_HOP,     0U, "fhss_dec_rej_bad_hop");
    expect_at(out, HOST_STATS_OFFSET_FHSS_DEC_REJ_EPOCH_DRIFT, 0U, "fhss_dec_rej_epoch_drift");
    expect_at(out, HOST_STATS_OFFSET_FHSS_DEC_REJ_LOCKED_OUT,  2U, "fhss_dec_rej_locked_out");
    expect_at(out, HOST_STATS_OFFSET_CLK_DEMOTION_RESET,       2U, "clk_demotion_reset");
    expect_at(out, HOST_STATS_OFFSET_CLK_DEMOTION_KEPT,        1U, "clk_demotion_kept");
    expect_at(out, HOST_STATS_OFFSET_TX_FIRST_ANCHOR,          1U, "tx_first_anchor");
    expect_at(out, HOST_STATS_OFFSET_TX_STREAM_STREAK_MAX,    31U, "tx_stream_streak_max");

    /* The table above must end where the payload ends: a new additive
     * field without a line here is a contract change nobody pinned. */
    s_cases++;
    if (HOST_STATS_OFFSET_TX_STREAM_STREAK_MAX + 4U != HOST_STATS_PAYLOAD_LEN) {
        s_fail++;
        printf("[FAIL] HOST_STATS_PAYLOAD_LEN (%u) is not the last pinned field + 4 -- extend this test\n",
               (unsigned)HOST_STATS_PAYLOAD_LEN);
    }

    if (s_fail != 0) {
        printf("[FAIL] stats_layout: %d of %d cases\n", s_fail, s_cases);
        return EXIT_FAILURE;
    }
    printf("[PASS] stats_layout: %d cases\n", s_cases);
    return EXIT_SUCCESS;
}
