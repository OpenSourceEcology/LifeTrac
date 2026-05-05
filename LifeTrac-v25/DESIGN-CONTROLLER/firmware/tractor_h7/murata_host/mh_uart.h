#ifndef LIFETRAC_TRACTOR_H7_MH_UART_H
#define LIFETRAC_TRACTOR_H7_MH_UART_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mh_uart_if_s {
    void *ctx;
    bool (*open)(void *ctx, uint32_t baud);
    void (*close)(void *ctx);
    int32_t (*read)(void *ctx, uint8_t *dst, uint16_t dst_cap);
    int32_t (*write)(void *ctx, const uint8_t *src, uint16_t src_len);
    uint32_t (*now_ms)(void *ctx);
} mh_uart_if_t;

bool mh_uart_if_open(const mh_uart_if_t *iface, uint32_t baud);
void mh_uart_if_close(const mh_uart_if_t *iface);
int32_t mh_uart_if_read(const mh_uart_if_t *iface, uint8_t *dst, uint16_t dst_cap);
int32_t mh_uart_if_write(const mh_uart_if_t *iface, const uint8_t *src, uint16_t src_len);
uint32_t mh_uart_if_now_ms(const mh_uart_if_t *iface);

typedef struct mh_uart_posix_state_s {
    const char *path;
    int fd;
} mh_uart_posix_state_t;

/* POSIX harness transport adapter. */
bool mh_uart_posix_init(mh_uart_if_t *out_iface,
                        mh_uart_posix_state_t *storage,
                        const char *path);

typedef struct mh_uart_tcp_state_s {
    const char *host;
    uint16_t port;
    intptr_t socket_handle;
    bool wsa_started;
} mh_uart_tcp_state_t;

/* TCP loopback transport adapter for host-CC harnesses. */
bool mh_uart_tcp_init(mh_uart_if_t *out_iface,
                      mh_uart_tcp_state_t *storage,
                      const char *host,
                      uint16_t port);

typedef struct mh_uart_arduino_state_s {
    void *serial_port;
} mh_uart_arduino_state_t;

/* Arduino transport adapter (serial_port points to a HardwareSerial object). */
bool mh_uart_arduino_init(mh_uart_if_t *out_iface,
                          mh_uart_arduino_state_t *storage,
                          void *serial_port);

#ifdef __cplusplus
}
#endif

#endif /* LIFETRAC_TRACTOR_H7_MH_UART_H */
