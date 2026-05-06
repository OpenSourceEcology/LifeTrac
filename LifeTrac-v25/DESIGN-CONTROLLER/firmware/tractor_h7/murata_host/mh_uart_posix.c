#include "mh_uart.h"

#if !defined(ARDUINO)

#include <errno.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define MH_UART_POSIX_MAX_WRITE_CHUNK 17U

static speed_t map_baud(uint32_t baud) {
    switch (baud) {
        case 9600U:
            return B9600;
        case 19200U:
            return B19200;
        case 38400U:
            return B38400;
        case 57600U:
            return B57600;
        case 115200U:
            return B115200;
#ifdef B230400
        case 230400U:
            return B230400;
#endif
#ifdef B460800
        case 460800U:
            return B460800;
#endif
#ifdef B921600
        case 921600U:
            return B921600;
#endif
        default:
            return 0;
    }
}

static uint32_t monotonic_ms(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0U;
    }
    return (uint32_t)((uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000L));
}

static bool posix_open(void *ctx, uint32_t baud) {
    struct termios tio;
    speed_t speed;
    mh_uart_posix_state_t *uart = (mh_uart_posix_state_t *)ctx;

    if (uart == NULL || uart->path == NULL) {
        return false;
    }

    speed = map_baud(baud);
    if (speed == 0) {
        return false;
    }

    uart->fd = open(uart->path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (uart->fd < 0) {
        return false;
    }

    if (tcgetattr(uart->fd, &tio) != 0) {
        close(uart->fd);
        uart->fd = -1;
        return false;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (cfsetispeed(&tio, speed) != 0 || cfsetospeed(&tio, speed) != 0) {
        close(uart->fd);
        uart->fd = -1;
        return false;
    }

    if (tcsetattr(uart->fd, TCSANOW, &tio) != 0) {
        close(uart->fd);
        uart->fd = -1;
        return false;
    }

    return true;
}

static void posix_close(void *ctx) {
    mh_uart_posix_state_t *uart = (mh_uart_posix_state_t *)ctx;
    if (uart == NULL) {
        return;
    }

    if (uart->fd >= 0) {
        close(uart->fd);
        uart->fd = -1;
    }
}

static int32_t posix_read(void *ctx, uint8_t *dst, uint16_t dst_cap) {
    ssize_t n;
    mh_uart_posix_state_t *uart = (mh_uart_posix_state_t *)ctx;

    if (uart == NULL || uart->fd < 0 || dst == NULL || dst_cap == 0U) {
        return -1;
    }

    n = read(uart->fd, dst, (size_t)dst_cap);
    if (n > 0) {
        return (int32_t)n;
    }
    if (n == 0) {
        return 0;
    }

    if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return 0;
    }
    return -1;
}

static int32_t posix_write(void *ctx, const uint8_t *src, uint16_t src_len) {
    size_t chunk_len;
    ssize_t n;
    mh_uart_posix_state_t *uart = (mh_uart_posix_state_t *)ctx;

    if (uart == NULL || uart->fd < 0 || src == NULL || src_len == 0U) {
        return -1;
    }

    chunk_len = (size_t)src_len;
    if (chunk_len > MH_UART_POSIX_MAX_WRITE_CHUNK) {
        chunk_len = MH_UART_POSIX_MAX_WRITE_CHUNK;
    }

    n = write(uart->fd, src, chunk_len);
    if (n > 0) {
        return (int32_t)n;
    }
    if (n == 0) {
        return 0;
    }

    if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return 0;
    }
    return -1;
}

static uint32_t posix_now_ms(void *ctx) {
    (void)ctx;
    return monotonic_ms();
}

bool mh_uart_posix_init(mh_uart_if_t *out_iface,
                        mh_uart_posix_state_t *storage,
                        const char *path) {
    mh_uart_posix_state_t *uart = storage;

    if (out_iface == NULL || uart == NULL || path == NULL) {
        return false;
    }

    memset(uart, 0, sizeof(*uart));
    uart->path = path;
    uart->fd = -1;

    memset(out_iface, 0, sizeof(*out_iface));
    out_iface->ctx = uart;
    out_iface->open = posix_open;
    out_iface->close = posix_close;
    out_iface->read = posix_read;
    out_iface->write = posix_write;
    out_iface->now_ms = posix_now_ms;

    return true;
}

#endif /* !ARDUINO */
