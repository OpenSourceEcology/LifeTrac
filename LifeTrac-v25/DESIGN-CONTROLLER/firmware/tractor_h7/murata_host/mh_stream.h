#ifndef LIFETRAC_TRACTOR_H7_MH_STREAM_H
#define LIFETRAC_TRACTOR_H7_MH_STREAM_H

#include "mh_uart.h"
#include "mh_wire.h"
#include "murata_host.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MH_STREAM_IO_CHUNK_MAX               256U
#define MH_STREAM_RX_ENCODED_CAP             512U
#define MH_STREAM_DECODED_CAP                HOST_MAX_INNER_LEN
#define MH_STREAM_TX_ENCODED_CAP             512U
#define MH_STREAM_WRITE_BUDGET_MS            5U

typedef struct mh_stream_counters_s {
    uint32_t reject_crc;
    uint32_t reject_length;
    uint32_t reject_version;
    uint32_t reject_unknown_type;
} mh_stream_counters_t;

typedef bool (*mh_stream_frame_handler_t)(const murata_host_frame_t *frame, void *user_ctx);

typedef struct mh_stream_s {
    mh_uart_if_t uart;
    mh_stream_frame_handler_t frame_handler;
    void *user_ctx;
    uint16_t next_seq;
    uint16_t rx_encoded_len;
    uint8_t rx_encoded[MH_STREAM_RX_ENCODED_CAP];
    mh_stream_counters_t counters;
} mh_stream_t;

void mh_stream_init(mh_stream_t *stream,
                    const mh_uart_if_t *uart,
                    mh_stream_frame_handler_t frame_handler,
                    void *user_ctx);

bool mh_stream_open(mh_stream_t *stream, uint32_t baud);
void mh_stream_close(mh_stream_t *stream);
void mh_stream_poll(mh_stream_t *stream, uint32_t now_ms);

bool mh_stream_send_ping_req(mh_stream_t *stream,
                             uint8_t flags,
                             const uint8_t *payload,
                             uint16_t payload_len,
                             uint16_t *out_seq);

bool mh_stream_send_ver_req(mh_stream_t *stream,
                            uint16_t *out_seq);

bool mh_stream_send_cfg_get_req(mh_stream_t *stream,
                                uint8_t key,
                                uint16_t *out_seq);

bool mh_stream_send_stats_dump_req(mh_stream_t *stream,
                                   uint16_t *out_seq);

const mh_stream_counters_t *mh_stream_counters(const mh_stream_t *stream);

#ifdef __cplusplus
}
#endif

#endif /* LIFETRAC_TRACTOR_H7_MH_STREAM_H */
