#include "mh_uart.h"

#include <stddef.h>

bool mh_uart_if_open(const mh_uart_if_t *iface, uint32_t baud) {
    if (iface == NULL || iface->open == NULL) {
        return false;
    }
    return iface->open(iface->ctx, baud);
}

void mh_uart_if_close(const mh_uart_if_t *iface) {
    if (iface == NULL || iface->close == NULL) {
        return;
    }
    iface->close(iface->ctx);
}

int32_t mh_uart_if_read(const mh_uart_if_t *iface, uint8_t *dst, uint16_t dst_cap) {
    if (iface == NULL || iface->read == NULL) {
        return -1;
    }
    return iface->read(iface->ctx, dst, dst_cap);
}

int32_t mh_uart_if_write(const mh_uart_if_t *iface, const uint8_t *src, uint16_t src_len) {
    if (iface == NULL || iface->write == NULL) {
        return -1;
    }
    return iface->write(iface->ctx, src, src_len);
}

uint32_t mh_uart_if_now_ms(const mh_uart_if_t *iface) {
    if (iface == NULL || iface->now_ms == NULL) {
        return 0U;
    }
    return iface->now_ms(iface->ctx);
}
