#include "mh_uart.h"

#if !defined(ARDUINO)

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#else
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#endif

#define MH_UART_TCP_MAX_WRITE_CHUNK 17U

#if defined(_WIN32)
#define MH_SOCKET_INVALID ((intptr_t)INVALID_SOCKET)
#else
#define MH_SOCKET_INVALID ((intptr_t)-1)
#endif

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

static void socket_close_handle(intptr_t socket_handle) {
#if defined(_WIN32)
    if (socket_handle != MH_SOCKET_INVALID) {
        closesocket((SOCKET)socket_handle);
    }
#else
    if (socket_handle != MH_SOCKET_INVALID) {
        close((int)socket_handle);
    }
#endif
}

static int socket_last_error(void) {
#if defined(_WIN32)
    return WSAGetLastError();
#else
    return errno;
#endif
}

static bool socket_would_block(int err) {
#if defined(_WIN32)
    return err == WSAEWOULDBLOCK;
#else
    return err == EAGAIN || err == EWOULDBLOCK;
#endif
}

static bool socket_set_nonblocking(intptr_t socket_handle) {
#if defined(_WIN32)
    u_long mode = 1UL;
    return ioctlsocket((SOCKET)socket_handle, FIONBIO, &mode) == 0;
#else
    int flags = fcntl((int)socket_handle, F_GETFL, 0);
    if (flags < 0) {
        return false;
    }
    return fcntl((int)socket_handle, F_SETFL, flags | O_NONBLOCK) == 0;
#endif
}

static bool tcp_open(void *ctx, uint32_t baud) {
    int gai_rc;
    char port_buf[8];
    struct addrinfo hints;
    struct addrinfo *result = NULL;
    struct addrinfo *candidate;
    mh_uart_tcp_state_t *uart = (mh_uart_tcp_state_t *)ctx;

    (void)baud;

    if (uart == NULL || uart->host == NULL || uart->port == 0U) {
        return false;
    }

#if defined(_WIN32)
    if (!uart->wsa_started) {
        WSADATA wsa_data;
        if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
            return false;
        }
        uart->wsa_started = true;
    }
#endif

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    snprintf(port_buf, sizeof(port_buf), "%u", (unsigned int)uart->port);
    gai_rc = getaddrinfo(uart->host, port_buf, &hints, &result);
    if (gai_rc != 0 || result == NULL) {
        return false;
    }

    uart->socket_handle = MH_SOCKET_INVALID;
    for (candidate = result; candidate != NULL; candidate = candidate->ai_next) {
#if defined(_WIN32)
        SOCKET sock = socket(candidate->ai_family, candidate->ai_socktype, candidate->ai_protocol);
        if (sock == INVALID_SOCKET) {
            continue;
        }
        if (connect(sock, candidate->ai_addr, (int)candidate->ai_addrlen) == 0) {
            uart->socket_handle = (intptr_t)sock;
            break;
        }
        closesocket(sock);
#else
        int sock = socket(candidate->ai_family, candidate->ai_socktype, candidate->ai_protocol);
        if (sock < 0) {
            continue;
        }
        if (connect(sock, candidate->ai_addr, candidate->ai_addrlen) == 0) {
            uart->socket_handle = (intptr_t)sock;
            break;
        }
        close(sock);
#endif
    }

    freeaddrinfo(result);
    if (uart->socket_handle == MH_SOCKET_INVALID) {
        return false;
    }

    if (!socket_set_nonblocking(uart->socket_handle)) {
        socket_close_handle(uart->socket_handle);
        uart->socket_handle = MH_SOCKET_INVALID;
        return false;
    }

    return true;
}

static void tcp_close(void *ctx) {
    mh_uart_tcp_state_t *uart = (mh_uart_tcp_state_t *)ctx;
    if (uart == NULL) {
        return;
    }

    if (uart->socket_handle != MH_SOCKET_INVALID) {
        socket_close_handle(uart->socket_handle);
        uart->socket_handle = MH_SOCKET_INVALID;
    }

#if defined(_WIN32)
    if (uart->wsa_started) {
        WSACleanup();
        uart->wsa_started = false;
    }
#endif
}

static int32_t tcp_read(void *ctx, uint8_t *dst, uint16_t dst_cap) {
    mh_uart_tcp_state_t *uart = (mh_uart_tcp_state_t *)ctx;
    int n;

    if (uart == NULL || uart->socket_handle == MH_SOCKET_INVALID || dst == NULL || dst_cap == 0U) {
        return -1;
    }

#if defined(_WIN32)
    n = recv((SOCKET)uart->socket_handle, (char *)dst, (int)dst_cap, 0);
#else
    n = (int)recv((int)uart->socket_handle, dst, (size_t)dst_cap, 0);
#endif
    if (n > 0) {
        return (int32_t)n;
    }
    if (n == 0) {
        return -1;
    }

    if (socket_would_block(socket_last_error())) {
        return 0;
    }
    return -1;
}

static int32_t tcp_write(void *ctx, const uint8_t *src, uint16_t src_len) {
    size_t chunk_len;
    mh_uart_tcp_state_t *uart = (mh_uart_tcp_state_t *)ctx;
    int n;

    if (uart == NULL || uart->socket_handle == MH_SOCKET_INVALID || src == NULL || src_len == 0U) {
        return -1;
    }

    chunk_len = (size_t)src_len;
    if (chunk_len > MH_UART_TCP_MAX_WRITE_CHUNK) {
        chunk_len = MH_UART_TCP_MAX_WRITE_CHUNK;
    }

#if defined(_WIN32)
    n = send((SOCKET)uart->socket_handle, (const char *)src, (int)chunk_len, 0);
#else
    n = (int)send((int)uart->socket_handle, src, chunk_len, 0);
#endif
    if (n > 0) {
        return (int32_t)n;
    }
    if (n == 0) {
        return 0;
    }

    if (socket_would_block(socket_last_error())) {
        return 0;
    }
    return -1;
}

static uint32_t tcp_now_ms(void *ctx) {
    (void)ctx;
    return monotonic_ms();
}

bool mh_uart_tcp_init(mh_uart_if_t *out_iface,
                      mh_uart_tcp_state_t *storage,
                      const char *host,
                      uint16_t port) {
    mh_uart_tcp_state_t *uart = storage;

    if (out_iface == NULL || uart == NULL || host == NULL || host[0] == '\0' || port == 0U) {
        return false;
    }

    memset(uart, 0, sizeof(*uart));
    uart->host = host;
    uart->port = port;
    uart->socket_handle = MH_SOCKET_INVALID;
    uart->wsa_started = false;

    memset(out_iface, 0, sizeof(*out_iface));
    out_iface->ctx = uart;
    out_iface->open = tcp_open;
    out_iface->close = tcp_close;
    out_iface->read = tcp_read;
    out_iface->write = tcp_write;
    out_iface->now_ms = tcp_now_ms;

    return true;
}

#endif /* !ARDUINO */