#include "mh_stats.h"
#include "mh_stream.h"
#include "mh_uart.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <time.h>
#include <unistd.h>
#endif

typedef struct loopback_state_s {
    uint16_t ping_seq;
    uint16_t cfg_seq;
    uint16_t stats_seq;
    uint8_t expected_ping_byte;
    bool wait_ping;
    bool wait_cfg;
    bool wait_stats;
    uint32_t ping_ok;
    uint32_t cfg_ok;
    uint32_t stats_ok;
    uint32_t stats_legacy;
    uint32_t stats_additive;
} loopback_state_t;

typedef struct loopback_endpoint_s {
    bool use_tcp;
    const char *path;
    char tcp_host[64];
    uint16_t tcp_port;
} loopback_endpoint_t;

static bool starts_with(const char *value, const char *prefix) {
    size_t prefix_len;

    if (value == NULL || prefix == NULL) {
        return false;
    }

    prefix_len = strlen(prefix);
    return strncmp(value, prefix, prefix_len) == 0;
}

static bool parse_tcp_endpoint(const char *spec, loopback_endpoint_t *out) {
    char *endptr = NULL;
    const char *port_sep;
    size_t host_len;
    unsigned long parsed_port;

    if (spec == NULL || out == NULL) {
        return false;
    }

    port_sep = strrchr(spec, ':');
    if (port_sep == NULL || port_sep == spec || port_sep[1] == '\0') {
        return false;
    }

    host_len = (size_t)(port_sep - spec);
    if (host_len == 0U || host_len >= sizeof(out->tcp_host)) {
        return false;
    }

    memset(out->tcp_host, 0, sizeof(out->tcp_host));
    memcpy(out->tcp_host, spec, host_len);

    parsed_port = strtoul(port_sep + 1, &endptr, 10);
    if (endptr == NULL || *endptr != '\0' || parsed_port == 0UL || parsed_port > 65535UL) {
        return false;
    }

    out->use_tcp = true;
    out->tcp_port = (uint16_t)parsed_port;
    return true;
}

static bool parse_endpoint_arg(const char *arg, loopback_endpoint_t *out) {
    if (arg == NULL || out == NULL) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    if (starts_with(arg, "tcp://")) {
        return parse_tcp_endpoint(arg + 6, out);
    }
    if (starts_with(arg, "tcp:")) {
        return parse_tcp_endpoint(arg + 4, out);
    }

    out->use_tcp = false;
    out->path = arg;
    return true;
}

static void sleep_1ms(void) {
#if defined(_WIN32)
    Sleep(1U);
#else
    usleep(1000);
#endif
}

static uint32_t monotonic_ms(void) {
#if defined(_WIN32)
    return (uint32_t)GetTickCount64();
#else
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0U;
    }
    return (uint32_t)((uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000L));
#endif
}

static bool on_frame(const murata_host_frame_t *frame, void *user_ctx) {
    mh_stats_t parsed;
    loopback_state_t *state = (loopback_state_t *)user_ctx;

    if (frame == NULL || state == NULL) {
        return false;
    }

    switch (frame->type) {
        case HOST_TYPE_PING_REQ:
            if (!state->wait_ping || frame->seq != state->ping_seq) {
                return false;
            }
            if (frame->payload_len != 1U || frame->payload[0] != state->expected_ping_byte) {
                return false;
            }
            state->wait_ping = false;
            state->ping_ok++;
            return true;

        case HOST_TYPE_CFG_DATA_URC:
        case HOST_TYPE_CFG_OK_URC:
            if (!state->wait_cfg || frame->seq != state->cfg_seq) {
                return false;
            }
            state->wait_cfg = false;
            state->cfg_ok++;
            return true;

        case HOST_TYPE_STATS_URC:
            if (!state->wait_stats || frame->seq != state->stats_seq) {
                return false;
            }
            if (!mh_stats_parse(frame->payload, frame->payload_len, &parsed)) {
                return false;
            }
            if (frame->payload_len >= HOST_STATS_PAYLOAD_LEN) {
                state->stats_additive++;
            } else if (frame->payload_len >= HOST_STATS_LEGACY_PAYLOAD_LEN) {
                state->stats_legacy++;
            }
            state->wait_stats = false;
            state->stats_ok++;
            return true;

        default:
            return false;
    }
}

static int run_iteration(mh_stream_t *stream, loopback_state_t *state, uint32_t timeout_ms, uint32_t i) {
    uint8_t ping_payload[1];
    uint32_t start_ms;

    ping_payload[0] = (uint8_t)(i & 0xFFU);
    state->expected_ping_byte = ping_payload[0];

    if (!mh_stream_send_ping_req(stream, 0U, ping_payload, (uint16_t)sizeof(ping_payload), &state->ping_seq)) {
        return 1;
    }
    if (!mh_stream_send_cfg_get_req(stream, 0x00U, &state->cfg_seq)) {
        return 2;
    }
    if (!mh_stream_send_stats_dump_req(stream, &state->stats_seq)) {
        return 3;
    }

    state->wait_ping = true;
    state->wait_cfg = true;
    state->wait_stats = true;

    start_ms = monotonic_ms();
    while (state->wait_ping || state->wait_cfg || state->wait_stats) {
        uint32_t now_ms = monotonic_ms();
        mh_stream_poll(stream, now_ms);
        if ((uint32_t)(now_ms - start_ms) > timeout_ms) {
            return 4;
        }
        sleep_1ms();
    }

    return 0;
}

int main(int argc, char **argv) {
    int iterations = 120;
    int rc = 0;
    mh_uart_if_t uart_if;
#if !defined(_WIN32)
    mh_uart_posix_state_t uart_posix_state;
#endif
    mh_uart_tcp_state_t uart_tcp_state;
    mh_stream_t stream;
    loopback_state_t state;
    loopback_endpoint_t endpoint;
    const mh_stream_counters_t *counters;

    if (argc < 2) {
        fprintf(stderr, "usage: %s <endpoint> [iterations]\n", argv[0]);
        fprintf(stderr, "  endpoint: /dev/pts/N or tcp://<host>:<port>\n");
        return 2;
    }

    if (argc >= 3) {
        iterations = atoi(argv[2]);
        if (iterations <= 0) {
            fprintf(stderr, "invalid iterations: %s\n", argv[2]);
            return 2;
        }
    }

    if (!parse_endpoint_arg(argv[1], &endpoint)) {
        fprintf(stderr, "invalid endpoint: %s\n", argv[1]);
        return 2;
    }

#if defined(_WIN32)
    if (!endpoint.use_tcp) {
        fprintf(stderr, "Windows host loopback requires tcp://<host>:<port> endpoint\n");
        return 3;
    }
#endif

    memset(&state, 0, sizeof(state));
    if (endpoint.use_tcp) {
        if (!mh_uart_tcp_init(&uart_if, &uart_tcp_state, endpoint.tcp_host, endpoint.tcp_port)) {
            fprintf(stderr, "failed to initialize TCP UART adapter\n");
            return 3;
        }
    } else {
#if defined(_WIN32)
        fprintf(stderr, "POSIX endpoint paths are not supported on Windows\n");
        return 3;
#else
        if (!mh_uart_posix_init(&uart_if, &uart_posix_state, endpoint.path)) {
            fprintf(stderr, "failed to initialize POSIX UART adapter\n");
            return 3;
        }
#endif
    }

    mh_stream_init(&stream, &uart_if, on_frame, &state);
    if (!mh_stream_open(&stream, 921600U)) {
        fprintf(stderr, "failed to open UART endpoint: %s\n", argv[1]);
        return 4;
    }

    for (int i = 0; i < iterations; ++i) {
        rc = run_iteration(&stream, &state, 800U, (uint32_t)i);
        if (rc != 0) {
            fprintf(stderr, "loopback iteration %d failed (rc=%d)\n", i, rc);
            mh_stream_close(&stream);
            return 10 + rc;
        }
    }

    /* Drain any trailing injected fuzz frames. */
    for (int i = 0; i < 50; ++i) {
        mh_stream_poll(&stream, monotonic_ms());
        sleep_1ms();
    }

    counters = mh_stream_counters(&stream);
    mh_stream_close(&stream);

    if (state.ping_ok < (uint32_t)iterations ||
        state.cfg_ok < (uint32_t)iterations ||
        state.stats_ok < (uint32_t)iterations ||
        state.stats_legacy == 0U ||
        state.stats_additive == 0U ||
        counters == NULL ||
        counters->reject_crc == 0U ||
        counters->reject_length == 0U ||
        counters->reject_version == 0U) {
        fprintf(stderr,
                "loopback verdict fail: ping=%u cfg=%u stats=%u legacy=%u additive=%u rej_crc=%u rej_len=%u rej_ver=%u rej_unknown=%u\n",
                state.ping_ok,
                state.cfg_ok,
                state.stats_ok,
                state.stats_legacy,
                state.stats_additive,
                counters ? counters->reject_crc : 0U,
                counters ? counters->reject_length : 0U,
                counters ? counters->reject_version : 0U,
                counters ? counters->reject_unknown_type : 0U);
        return 20;
    }

    printf("loopback verdict pass: iterations=%d ping=%u cfg=%u stats=%u legacy=%u additive=%u rej_crc=%u rej_len=%u rej_ver=%u rej_unknown=%u\n",
           iterations,
           state.ping_ok,
           state.cfg_ok,
           state.stats_ok,
           state.stats_legacy,
           state.stats_additive,
           counters->reject_crc,
           counters->reject_length,
           counters->reject_version,
           counters->reject_unknown_type);
    return 0;
}
