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

void host_uart_init(uint32_t baud);
void host_uart_poll_dma(void);
bool host_uart_pop_frame(host_frame_t *out_frame);

void host_uart_send_urc(uint8_t type,
                        uint16_t seq,
                        uint8_t flags,
                        const uint8_t *payload,
                        uint16_t payload_len);
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

#endif /* LIFETRAC_MURATA_L072_HOST_UART_H */
