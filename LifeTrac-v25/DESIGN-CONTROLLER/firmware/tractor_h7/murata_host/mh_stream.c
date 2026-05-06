#include "mh_stream.h"

#include "mh_cobs.h"
#include "mh_crc16.h"

#include <stddef.h>
#include <string.h>

static uint16_t read_u16_le(const uint8_t *in) {
    return (uint16_t)in[0] | ((uint16_t)in[1] << 8);
}

static uint16_t alloc_seq(mh_stream_t *stream) {
    uint16_t seq = stream->next_seq;
    stream->next_seq++;
    if (stream->next_seq == 0U) {
        stream->next_seq = 1U;
    }
    return seq;
}

static bool stream_send_inner(mh_stream_t *stream,
                              uint8_t type,
                              uint8_t flags,
                              uint16_t seq,
                              const uint8_t *payload,
                              uint16_t payload_len) {
    uint8_t inner[MH_STREAM_DECODED_CAP];
    uint8_t encoded[MH_STREAM_TX_ENCODED_CAP];
    uint16_t inner_len;
    size_t encoded_len;
    uint16_t total_len;
    uint16_t sent = 0U;
    uint32_t start_ms;

    if (stream == NULL) {
        return false;
    }

    inner_len = murata_host_build_inner_frame(type,
                                              flags,
                                              seq,
                                              payload,
                                              payload_len,
                                              inner,
                                              (uint16_t)sizeof(inner));
    if (inner_len == 0U) {
        return false;
    }

    encoded_len = mh_cobs_encode(inner, inner_len, encoded, sizeof(encoded));
    if (encoded_len == 0U || encoded_len >= sizeof(encoded)) {
        return false;
    }

    encoded[encoded_len++] = 0U;
    total_len = (uint16_t)encoded_len;
    start_ms = mh_uart_if_now_ms(&stream->uart);

    while (sent < total_len) {
        int32_t written = mh_uart_if_write(&stream->uart,
                                           &encoded[sent],
                                           (uint16_t)(total_len - sent));
        if (written < 0) {
            return false;
        }
        if (written == 0) {
            uint32_t now_ms = mh_uart_if_now_ms(&stream->uart);
            if ((uint32_t)(now_ms - start_ms) > MH_STREAM_WRITE_BUDGET_MS) {
                return false;
            }
            continue;
        }
        sent = (uint16_t)(sent + (uint16_t)written);
    }

    return true;
}

static void dispatch_frame(mh_stream_t *stream,
                           const uint8_t *decoded,
                           uint16_t decoded_len) {
    uint16_t payload_len;
    uint16_t expected_len;
    uint16_t frame_crc;
    uint16_t calc_crc;
    murata_host_frame_t frame;

    if (decoded_len < (HOST_HEADER_LEN + HOST_CRC_LEN)) {
        stream->counters.reject_length++;
        return;
    }

    payload_len = read_u16_le(&decoded[5]);
    expected_len = (uint16_t)(HOST_HEADER_LEN + payload_len + HOST_CRC_LEN);
    if (payload_len > HOST_MAX_PAYLOAD_LEN || expected_len != decoded_len) {
        stream->counters.reject_length++;
        return;
    }

    frame_crc = read_u16_le(&decoded[(uint16_t)(decoded_len - HOST_CRC_LEN)]);
    calc_crc = mh_crc16_ccitt(decoded, (size_t)(decoded_len - HOST_CRC_LEN));
    if (frame_crc != calc_crc) {
        stream->counters.reject_crc++;
        return;
    }

    if (!murata_host_parse_inner_frame(decoded, decoded_len, &frame)) {
        stream->counters.reject_length++;
        return;
    }

    if (frame.ver != HOST_PROTOCOL_VER) {
        stream->counters.reject_version++;
        return;
    }

    if (stream->frame_handler == NULL || !stream->frame_handler(&frame, stream->user_ctx)) {
        stream->counters.reject_unknown_type++;
    }
}

void mh_stream_init(mh_stream_t *stream,
                    const mh_uart_if_t *uart,
                    mh_stream_frame_handler_t frame_handler,
                    void *user_ctx) {
    if (stream == NULL || uart == NULL) {
        return;
    }

    memset(stream, 0, sizeof(*stream));
    stream->uart = *uart;
    stream->frame_handler = frame_handler;
    stream->user_ctx = user_ctx;
    stream->next_seq = 1U;
}

bool mh_stream_open(mh_stream_t *stream, uint32_t baud) {
    if (stream == NULL) {
        return false;
    }
    return mh_uart_if_open(&stream->uart, baud);
}

void mh_stream_close(mh_stream_t *stream) {
    if (stream == NULL) {
        return;
    }
    mh_uart_if_close(&stream->uart);
}

void mh_stream_poll(mh_stream_t *stream, uint32_t now_ms) {
    uint8_t chunk[MH_STREAM_IO_CHUNK_MAX];

    (void)now_ms;

    if (stream == NULL) {
        return;
    }

    for (;;) {
        int32_t bytes = mh_uart_if_read(&stream->uart, chunk, (uint16_t)sizeof(chunk));
        if (bytes <= 0) {
            break;
        }

        for (int32_t i = 0; i < bytes; ++i) {
            uint8_t byte = chunk[i];

            if (byte == 0U) {
                if (stream->rx_encoded_len > 0U) {
                    uint8_t decoded[MH_STREAM_DECODED_CAP];
                    size_t decoded_len = mh_cobs_decode(stream->rx_encoded,
                                                        stream->rx_encoded_len,
                                                        decoded,
                                                        sizeof(decoded));
                    if (decoded_len == 0U || decoded_len > 0xFFFFU) {
                        stream->counters.reject_length++;
                    } else {
                        dispatch_frame(stream, decoded, (uint16_t)decoded_len);
                    }
                    stream->rx_encoded_len = 0U;
                }
                continue;
            }

            if (stream->rx_encoded_len >= MH_STREAM_RX_ENCODED_CAP) {
                stream->counters.reject_length++;
                stream->rx_encoded_len = 0U;
                continue;
            }

            stream->rx_encoded[stream->rx_encoded_len++] = byte;
        }
    }
}

bool mh_stream_send_ping_req(mh_stream_t *stream,
                             uint8_t flags,
                             const uint8_t *payload,
                             uint16_t payload_len,
                             uint16_t *out_seq) {
    uint16_t seq;

    if (stream == NULL) {
        return false;
    }

    seq = alloc_seq(stream);
    if (!stream_send_inner(stream,
                           HOST_TYPE_PING_REQ,
                           flags,
                           seq,
                           payload,
                           payload_len)) {
        return false;
    }

    if (out_seq != NULL) {
        *out_seq = seq;
    }
    return true;
}

bool mh_stream_send_ver_req(mh_stream_t *stream,
                            uint16_t *out_seq) {
    uint16_t seq;

    if (stream == NULL) {
        return false;
    }

    seq = alloc_seq(stream);
    if (!stream_send_inner(stream,
                           HOST_TYPE_VER_REQ,
                           0U,
                           seq,
                           NULL,
                           0U)) {
        return false;
    }

    if (out_seq != NULL) {
        *out_seq = seq;
    }
    return true;
}

bool mh_stream_send_cfg_get_req(mh_stream_t *stream,
                                uint8_t key,
                                uint16_t *out_seq) {
    uint16_t seq;
    uint8_t payload[1];

    if (stream == NULL) {
        return false;
    }

    payload[0] = key;
    seq = alloc_seq(stream);
    if (!stream_send_inner(stream,
                           HOST_TYPE_CFG_GET_REQ,
                           0U,
                           seq,
                           payload,
                           (uint16_t)sizeof(payload))) {
        return false;
    }

    if (out_seq != NULL) {
        *out_seq = seq;
    }
    return true;
}

bool mh_stream_send_stats_dump_req(mh_stream_t *stream,
                                   uint16_t *out_seq) {
    uint16_t seq;

    if (stream == NULL) {
        return false;
    }

    seq = alloc_seq(stream);
    if (!stream_send_inner(stream,
                           HOST_TYPE_STATS_DUMP_REQ,
                           0U,
                           seq,
                           NULL,
                           0U)) {
        return false;
    }

    if (out_seq != NULL) {
        *out_seq = seq;
    }
    return true;
}

const mh_stream_counters_t *mh_stream_counters(const mh_stream_t *stream) {
    if (stream == NULL) {
        return NULL;
    }
    return &stream->counters;
}
