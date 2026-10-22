#include "mh_runtime.h"

#if defined(LIFETRAC_USE_METHOD_G_HOST) && (LIFETRAC_USE_METHOD_G_HOST)

#include "mh_stream.h"
#include "mh_uart.h"

#ifndef LIFETRAC_MH_BENCH_LOG
#define LIFETRAC_MH_BENCH_LOG 1
#endif

#include <stddef.h>
#include <string.h>

#if (LIFETRAC_MH_BENCH_LOG)
#include <stdarg.h>
#include <stdio.h>
#endif

typedef struct mh_runtime_state_s {
    bool started;
    mh_uart_if_t uart_if;
    mh_uart_arduino_state_t uart_state;
    mh_stream_t stream;
    mh_runtime_health_t health;
    uint32_t next_ping_ms;
    uint32_t next_stats_ms;
    uint32_t next_ver_ms;
    uint8_t ping_nonce;
    mh_runtime_log_sink_t log_sink;
    void *log_ctx;
    mh_link_gate_t tx_gate;
    mh_runtime_rx_handler_t rx_handler;
    void *rx_ctx;
    mh_runtime_tx_handler_t tx_handler;
    void *tx_ctx;
#if (LIFETRAC_MH_BENCH_LOG)
    uint32_t next_summary_ms;
#endif
} mh_runtime_state_t;

static mh_runtime_state_t g_runtime;

#if (LIFETRAC_MH_BENCH_LOG)
static void runtime_emit_logline(mh_runtime_state_t *runtime, const char *fmt, ...) {
    char line[192];
    va_list args;

    if (runtime == NULL || runtime->log_sink == NULL || fmt == NULL) {
        return;
    }

    va_start(args, fmt);
    (void)vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);

    runtime->log_sink(line, runtime->log_ctx);
}

static void runtime_emit_frame_log(mh_runtime_state_t *runtime, uint8_t frame_type) {
    const mh_runtime_health_t *health;

    if (runtime == NULL) {
        return;
    }

    health = &runtime->health;

    switch (frame_type) {
        case HOST_TYPE_BOOT_URC:
            runtime_emit_logline(runtime,
                                 "MH BOOT reset=%u radio_ok=%u radio_ver=%u proto=%u wire=%u clock=%u",
                                 (unsigned)health->boot_reset_cause,
                                 (unsigned)health->boot_radio_ok,
                                 (unsigned)health->boot_radio_version,
                                 (unsigned)health->boot_protocol_ver,
                                 (unsigned)health->boot_wire_schema_ver,
                                 (unsigned)health->boot_clock_source_id);
            return;

        case HOST_TYPE_VER_URC:
            runtime_emit_logline(runtime,
                                 "MH VER proto=%u wire=%u fw=%u.%u.%u name=\"%s\" git=%s caps=0x%08lX",
                                 (unsigned)health->ver_protocol_ver,
                                 (unsigned)health->ver_wire_schema_ver,
                                 (unsigned)health->ver_fw_major,
                                 (unsigned)health->ver_fw_minor,
                                 (unsigned)health->ver_fw_patch,
                                 health->ver_fw_name,
                                 health->ver_fw_git,
                                 (unsigned long)health->ver_capability_bitmap);
            return;

        case HOST_TYPE_FAULT_URC:
            runtime_emit_logline(runtime,
                                 "MH FAULT code=%u sub=%u",
                                 (unsigned)health->fault_code,
                                 (unsigned)health->fault_sub);
            return;

        case HOST_TYPE_TX_DONE_URC:
            runtime_emit_logline(runtime,
                                 "MH TX_DONE id=%u status=%u toa_us=%lu power_dbm=%d",
                                 (unsigned)health->last_tx_id,
                                 (unsigned)health->last_tx_status,
                                 (unsigned long)health->last_tx_time_on_air_us,
                                 (int)health->last_tx_power_dbm);
            return;

        case HOST_TYPE_RX_FRAME_URC:
            runtime_emit_logline(runtime,
                                 "MH RX_FRAME len=%u snr=%d rssi=%d ts_us=%lu",
                                 (unsigned)health->last_rx_len,
                                 (int)health->last_rx_snr_db,
                                 (int)health->last_rx_rssi_dbm,
                                 (unsigned long)health->last_rx_timestamp_us);
            return;

        case HOST_TYPE_STATS_URC:
            if (health->has_stats) {
                runtime_emit_logline(runtime,
                                     "MH STATS rx_ok=%lu tx_ok=%lu lbt_abort=%lu airtime_abort=%lu len=%u",
                                     (unsigned long)health->last_stats.radio_rx_ok,
                                     (unsigned long)health->last_stats.radio_tx_ok,
                                     (unsigned long)health->last_stats.radio_tx_abort_lbt,
                                     (unsigned long)health->last_stats.radio_tx_abort_airtime,
                                     (unsigned)health->last_stats_len);
            }
            return;

        default:
            return;
    }
}

static void runtime_emit_summary_log(mh_runtime_state_t *runtime) {
    const mh_runtime_health_t *health;
    const mh_stream_counters_t *counters;

    if (runtime == NULL) {
        return;
    }

    health = &runtime->health;
    counters = mh_stream_counters(&runtime->stream);

    runtime_emit_logline(runtime,
                         "MH summary clock=%u radio_ok=%u ping=%lu tx_done=%lu rx=%lu rej_crc=%lu rej_len=%lu rej_ver=%lu rej_unk=%lu",
                         (unsigned)health->boot_clock_source_id,
                         (unsigned)health->boot_radio_ok,
                         (unsigned long)health->ping_echo_count,
                         (unsigned long)health->tx_done_count,
                         (unsigned long)health->rx_frame_count,
                         (unsigned long)(counters ? counters->reject_crc : 0U),
                         (unsigned long)(counters ? counters->reject_length : 0U),
                         (unsigned long)(counters ? counters->reject_version : 0U),
                         (unsigned long)(counters ? counters->reject_unknown_type : 0U));
}
#endif

static void runtime_note_tx_event(mh_runtime_state_t *runtime, mh_link_event_t event) {
    if (event != MH_LINK_EV_NONE && runtime->tx_handler != NULL) {
        runtime->tx_handler(event, runtime->tx_ctx);
    }
}

static bool runtime_on_frame(const murata_host_frame_t *frame, void *user_ctx) {
    uint32_t now_ms;
    bool handled;
    mh_link_rx_t rx;
    mh_runtime_state_t *runtime = (mh_runtime_state_t *)user_ctx;

    if (runtime == NULL || frame == NULL) {
        return false;
    }

    now_ms = mh_uart_if_now_ms(&runtime->uart_if);
    handled = mh_runtime_health_on_frame(&runtime->health, frame, now_ms);

    runtime_note_tx_event(runtime, mh_link_gate_on_frame(&runtime->tx_gate, frame));
    if (runtime->rx_handler != NULL && mh_link_rx_parse(frame, &rx)) {
        runtime->rx_handler(&rx, runtime->rx_ctx);
    }

#if (LIFETRAC_MH_BENCH_LOG)
    if (handled) {
        runtime_emit_frame_log(runtime, frame->type);
    }
#endif

    return handled;
}

bool mh_runtime_begin(void *serial_port, uint32_t baud) {
    mh_runtime_log_sink_t saved_log_sink = g_runtime.log_sink;
    void *saved_log_ctx = g_runtime.log_ctx;
    mh_runtime_rx_handler_t saved_rx_handler = g_runtime.rx_handler;
    void *saved_rx_ctx = g_runtime.rx_ctx;
    mh_runtime_tx_handler_t saved_tx_handler = g_runtime.tx_handler;
    void *saved_tx_ctx = g_runtime.tx_ctx;

    if (serial_port == NULL) {
        return false;
    }

    memset(&g_runtime, 0, sizeof(g_runtime));
    g_runtime.log_sink = saved_log_sink;
    g_runtime.log_ctx = saved_log_ctx;
    g_runtime.rx_handler = saved_rx_handler;
    g_runtime.rx_ctx = saved_rx_ctx;
    g_runtime.tx_handler = saved_tx_handler;
    g_runtime.tx_ctx = saved_tx_ctx;
    mh_runtime_health_reset(&g_runtime.health);
    mh_link_gate_init(&g_runtime.tx_gate);

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
    g_runtime.next_ver_ms = 0U;
    g_runtime.ping_nonce = 1U;
#if (LIFETRAC_MH_BENCH_LOG)
    g_runtime.next_summary_ms = 0U;
#endif
    return true;
}

bool mh_runtime_get_health(mh_runtime_health_t *out_health) {
    if (!g_runtime.started || out_health == NULL) {
        return false;
    }

    *out_health = g_runtime.health;
    return true;
}

void mh_runtime_set_log_sink(mh_runtime_log_sink_t sink, void *ctx) {
    g_runtime.log_sink = sink;
    g_runtime.log_ctx = ctx;
}

void mh_runtime_set_rx_handler(mh_runtime_rx_handler_t handler, void *ctx) {
    g_runtime.rx_handler = handler;
    g_runtime.rx_ctx = ctx;
}

void mh_runtime_set_tx_handler(mh_runtime_tx_handler_t handler, void *ctx) {
    g_runtime.tx_handler = handler;
    g_runtime.tx_ctx = ctx;
}

bool mh_runtime_tx_busy(void) {
    return mh_link_gate_busy(&g_runtime.tx_gate);
}

bool mh_runtime_submit_tx(const uint8_t *data, uint8_t len, uint32_t timeout_ms) {
    uint8_t tx_id;
    uint16_t seq = 0U;
    uint32_t now_ms;

    if (!g_runtime.started || mh_link_gate_busy(&g_runtime.tx_gate)) {
        return false;
    }

    tx_id = mh_link_gate_next_tx_id(&g_runtime.tx_gate);
    now_ms = mh_uart_if_now_ms(&g_runtime.uart_if);
    /* Arm the gate before the write: a short write may still have put a
     * whole frame on the wire, and a URC for it must find the gate armed.
     * A failed write releases it again through the timeout. */
    mh_link_gate_on_submit(&g_runtime.tx_gate, tx_id, g_runtime.stream.next_seq,
                           now_ms, timeout_ms);
    if (!mh_stream_send_tx_frame_req(&g_runtime.stream, tx_id, data, len, &seq)) {
        return false;
    }
    g_runtime.tx_gate.seq = seq;
    return true;
}

bool mh_runtime_get_tx_gate(mh_link_gate_t *out_gate) {
    if (!g_runtime.started || out_gate == NULL) {
        return false;
    }
    *out_gate = g_runtime.tx_gate;
    return true;
}

void mh_runtime_loop(uint32_t now_ms) {
    uint8_t ping_payload[1];

    if (!g_runtime.started) {
        return;
    }

    mh_stream_poll(&g_runtime.stream, now_ms);
    runtime_note_tx_event(&g_runtime, mh_link_gate_tick(&g_runtime.tx_gate, now_ms));

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

    /* Re-request firmware identity until VER_URC is received, then back off. */
    if (g_runtime.next_ver_ms == 0U || (int32_t)(now_ms - g_runtime.next_ver_ms) >= 0) {
        (void)mh_stream_send_ver_req(&g_runtime.stream, NULL);
        g_runtime.next_ver_ms = now_ms + (g_runtime.health.ver_seen ? 30000U : 1000U);
    }

#if (LIFETRAC_MH_BENCH_LOG)
    if (g_runtime.next_summary_ms == 0U || (int32_t)(now_ms - g_runtime.next_summary_ms) >= 0) {
        runtime_emit_summary_log(&g_runtime);
        g_runtime.next_summary_ms = now_ms + 1000U;
    }
#endif
}

#else

typedef int mh_runtime_disabled_translation_unit;

#endif
