#include "host_stats.h"

#include "host_uart.h"
#include "sx1276.h"
#include "sx1276_modes.h"

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

    host_uart_stats_reset();
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
    put_u32_le(&out[idx], host_uart_stats_uart_err_usart1());

    return HOST_STATS_PAYLOAD_LEN;
}
