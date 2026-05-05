#include "mh_runtime.h"

#if defined(LIFETRAC_USE_METHOD_G_HOST) && (LIFETRAC_USE_METHOD_G_HOST)

#include "mh_stats.h"
#include "mh_stream.h"
#include "mh_uart.h"

#include <stddef.h>
#include <string.h>

typedef struct mh_runtime_state_s {
    bool started;
    mh_uart_if_t uart_if;
    mh_uart_arduino_state_t uart_state;
    mh_stream_t stream;
    mh_stats_t last_stats;
    bool has_stats;
    uint32_t next_ping_ms;
    uint32_t next_stats_ms;
    uint8_t ping_nonce;
    uint8_t clock_source_id;
    uint8_t radio_ok;
    uint8_t last_fault_code;
    uint8_t last_fault_sub;
} mh_runtime_state_t;

static mh_runtime_state_t g_runtime;

static bool runtime_on_frame(const murata_host_frame_t *frame, void *user_ctx) {
    mh_runtime_state_t *runtime = (mh_runtime_state_t *)user_ctx;

    if (runtime == NULL || frame == NULL) {
        return false;
    }

    switch (frame->type) {
        case HOST_TYPE_BOOT_URC:
            if (frame->payload_len >= 6U) {
                runtime->radio_ok = frame->payload[4];
                runtime->clock_source_id = frame->payload[5];
            }
            return true;

        case HOST_TYPE_FAULT_URC:
            if (frame->payload_len >= 2U) {
                runtime->last_fault_code = frame->payload[0];
                runtime->last_fault_sub = frame->payload[1];
            }
            return true;

        case HOST_TYPE_STATS_URC:
            runtime->has_stats = mh_stats_parse(frame->payload,
                                                frame->payload_len,
                                                &runtime->last_stats);
            return runtime->has_stats;

        case HOST_TYPE_PING_REQ:
        case HOST_TYPE_CFG_OK_URC:
        case HOST_TYPE_CFG_DATA_URC:
        case HOST_TYPE_ERR_PROTO_URC:
            return true;

        default:
            return false;
    }
}

bool mh_runtime_begin(void *serial_port, uint32_t baud) {
    if (serial_port == NULL) {
        return false;
    }

    memset(&g_runtime, 0, sizeof(g_runtime));

    if (!mh_uart_arduino_init(&g_runtime.uart_if,
                              &g_runtime.uart_state,
                              serial_port)) {
        return false;
    }

    mh_stream_init(&g_runtime.stream,
                   &g_runtime.uart_if,
                   runtime_on_frame,
                   &g_runtime);

    if (!mh_stream_open(&g_runtime.stream, baud)) {
        return false;
    }

    g_runtime.started = true;
    g_runtime.next_ping_ms = 0U;
    g_runtime.next_stats_ms = 0U;
    g_runtime.ping_nonce = 1U;
    return true;
}

void mh_runtime_loop(uint32_t now_ms) {
    uint8_t ping_payload[1];

    if (!g_runtime.started) {
        return;
    }

    mh_stream_poll(&g_runtime.stream, now_ms);

    if (g_runtime.next_ping_ms == 0U || (int32_t)(now_ms - g_runtime.next_ping_ms) >= 0) {
        ping_payload[0] = g_runtime.ping_nonce++;
        (void)mh_stream_send_ping_req(&g_runtime.stream,
                                      0U,
                                      ping_payload,
                                      (uint16_t)sizeof(ping_payload),
                                      NULL);
        g_runtime.next_ping_ms = now_ms + 500U;
    }

    if (g_runtime.next_stats_ms == 0U || (int32_t)(now_ms - g_runtime.next_stats_ms) >= 0) {
        (void)mh_stream_send_stats_dump_req(&g_runtime.stream, NULL);
        g_runtime.next_stats_ms = now_ms + 1000U;
    }
}

#else

typedef int mh_runtime_disabled_translation_unit;

#endif
