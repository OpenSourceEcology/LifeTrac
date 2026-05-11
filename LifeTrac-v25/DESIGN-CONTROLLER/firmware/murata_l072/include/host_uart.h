#ifndef LIFETRAC_MURATA_L072_HOST_UART_H
#define LIFETRAC_MURATA_L072_HOST_UART_H

#include "config.h"
#include "host_types.h"

#include <stdbool.h>
#include <stdint.h>

#define HOST_HEADER_LEN              7U
#define HOST_CRC_LEN                 2U
#define HOST_PAYLOAD_MAX_LEN         (HOST_INNER_MAX_LEN - HOST_HEADER_LEN - HOST_CRC_LEN)

typedef enum host_status_e {
    HOST_STATUS_OK = 0,
    HOST_STATUS_ERR_COBS = 1,
    HOST_STATUS_ERR_CRC = 2,
    HOST_STATUS_ERR_TOO_LARGE = 3,
    HOST_STATUS_ERR_PROTO = 4
} host_status_t;

typedef struct host_frame_s {
    uint8_t ver;
    uint8_t type;
    uint8_t flags;
    uint16_t seq;
    uint16_t payload_len;
    uint8_t payload[HOST_PAYLOAD_MAX_LEN];
} host_frame_t;

#define HOST_DIAG_MARK_VER_REQ_PARSED        0x01U
#define HOST_DIAG_MARK_FRAME_PARSE_ERR       0x02U
#define HOST_DIAG_MARK_VER_REQ_DISPATCHED    0x04U
#define HOST_DIAG_MARK_VER_URC_SENT          0x08U
#define HOST_DIAG_MARK_AT_VER_DISPATCHED     0x10U

/*
 * Deferred RX-path diagnostic trace bitmask. Set from any context (IRQ-safe);
 * flushed only from foreground via host_uart_flush_diag_traces().  See
 * AI NOTES/2026-05-11_W1-7_RX_Implementation_Plan_Copilot_v1_2.md §3.
 */
#define HOST_TRACE_RX_LPUART1     (1UL << 0)
#define HOST_TRACE_RX_USART1      (1UL << 1)
#define HOST_TRACE_ERR_LPUART1    (1UL << 2)
#define HOST_TRACE_ERR_USART1     (1UL << 3)
#define HOST_TRACE_PROC_FRAME     (1UL << 4)
#define HOST_TRACE_COBS_ERR       (1UL << 5)
#define HOST_TRACE_COBS_OVF       (1UL << 6)
#define HOST_TRACE_PARSE_ERR      (1UL << 7)
#define HOST_TRACE_VER_REQ_RX     (1UL << 8)
#define HOST_TRACE_FRAME_OK       (1UL << 9)
#define HOST_TRACE_Q_FULL         (1UL << 10)
#define HOST_TRACE_AT_DISPATCH    (1UL << 11)
#define HOST_TRACE_RX_RING_OVF    (1UL << 12)

void host_uart_init(uint32_t baud);
void host_uart_poll_dma(void);
void host_uart_service_rx(void);
void host_uart_flush_diag_traces(void);
bool host_uart_pop_frame(host_frame_t *out_frame);

void host_uart_send_urc(uint8_t type,
                        uint16_t seq,
                        uint8_t flags,
                        const uint8_t *payload,
                        uint16_t payload_len);
void host_uart_send_ascii(const char *text);
void host_uart_send_err_proto(uint16_t seq,
                              uint8_t offending_type,
                              uint8_t offending_ver,
                              uint8_t err_code,
                              uint16_t detail);

void host_uart_stats_reset(void);

uint32_t host_uart_stats_dropped(void);
uint32_t host_uart_stats_errors(void);
uint32_t host_uart_stats_queue_full(void);
uint32_t host_uart_stats_irq_idle(void);
uint32_t host_uart_stats_irq_ht(void);
uint32_t host_uart_stats_irq_tc(void);
uint32_t host_uart_stats_irq_te(void);
uint32_t host_uart_stats_rx_bytes(void);
uint32_t host_uart_stats_rx_lpuart_bytes(void);
uint32_t host_uart_stats_rx_usart1_bytes(void);
uint32_t host_uart_stats_parse_ok(void);
uint32_t host_uart_stats_parse_err(void);
uint32_t host_uart_stats_uart_err_lpuart(void);
uint32_t host_uart_stats_uart_err_usart1(void);
uint32_t host_uart_stats_uart_pe_lpuart(void);
uint32_t host_uart_stats_uart_fe_lpuart(void);
uint32_t host_uart_stats_uart_ne_lpuart(void);
uint32_t host_uart_stats_uart_ore_lpuart(void);
uint32_t host_uart_stats_uart_pe_usart1(void);
uint32_t host_uart_stats_uart_fe_usart1(void);
uint32_t host_uart_stats_uart_ne_usart1(void);
uint32_t host_uart_stats_uart_ore_usart1(void);
uint32_t host_uart_stats_rx_ring_ovf(void);
uint32_t host_uart_take_dma_te_events(void);
uint8_t host_uart_take_rx_seen_flags(void);
void host_uart_note_diag_mark(uint8_t mark);
uint8_t host_uart_take_diag_marks(void);

#endif /* LIFETRAC_MURATA_L072_HOST_UART_H */
