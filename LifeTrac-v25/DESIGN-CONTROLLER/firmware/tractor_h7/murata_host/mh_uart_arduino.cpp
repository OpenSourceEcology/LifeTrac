#include "mh_uart.h"

#if defined(LIFETRAC_USE_METHOD_G_HOST) && (LIFETRAC_USE_METHOD_G_HOST)

#include <Arduino.h>

static bool arduino_open(void *ctx, uint32_t baud) {
    auto *state = static_cast<mh_uart_arduino_state_t *>(ctx);
    auto *serial = static_cast<HardwareSerial *>(state ? state->serial_port : nullptr);
    if (serial == nullptr) {
        return false;
    }
    serial->begin(baud);
    return true;
}

static void arduino_close(void *ctx) {
    auto *state = static_cast<mh_uart_arduino_state_t *>(ctx);
    auto *serial = static_cast<HardwareSerial *>(state ? state->serial_port : nullptr);
    if (serial != nullptr) {
        serial->flush();
    }
}

static int32_t arduino_read(void *ctx, uint8_t *dst, uint16_t dst_cap) {
    uint16_t count = 0;
    auto *state = static_cast<mh_uart_arduino_state_t *>(ctx);
    auto *serial = static_cast<HardwareSerial *>(state ? state->serial_port : nullptr);

    if (serial == nullptr || dst == nullptr || dst_cap == 0U) {
        return -1;
    }

    while (count < dst_cap && serial->available() > 0) {
        int value = serial->read();
        if (value < 0) {
            break;
        }
        dst[count++] = static_cast<uint8_t>(value);
    }

    return static_cast<int32_t>(count);
}

static int32_t arduino_write(void *ctx, const uint8_t *src, uint16_t src_len) {
    auto *state = static_cast<mh_uart_arduino_state_t *>(ctx);
    auto *serial = static_cast<HardwareSerial *>(state ? state->serial_port : nullptr);

    if (serial == nullptr || src == nullptr || src_len == 0U) {
        return -1;
    }

    return static_cast<int32_t>(serial->write(src, src_len));
}

static uint32_t arduino_now_ms(void *ctx) {
    (void)ctx;
    return millis();
}

extern "C" bool mh_uart_arduino_init(mh_uart_if_t *out_iface,
                                       mh_uart_arduino_state_t *storage,
                                       void *serial_port) {
    if (out_iface == nullptr || storage == nullptr || serial_port == nullptr) {
        return false;
    }

    storage->serial_port = serial_port;

    out_iface->ctx = storage;
    out_iface->open = arduino_open;
    out_iface->close = arduino_close;
    out_iface->read = arduino_read;
    out_iface->write = arduino_write;
    out_iface->now_ms = arduino_now_ms;

    return true;
}

#endif