#ifndef LIFETRAC_TRACTOR_H7_MH_RUNTIME_H
#define LIFETRAC_TRACTOR_H7_MH_RUNTIME_H

#include "mh_link.h"
#include "mh_runtime_health.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*mh_runtime_log_sink_t)(const char *line, void *ctx);

bool mh_runtime_begin(void *serial_port, uint32_t baud);
void mh_runtime_loop(uint32_t now_ms);
bool mh_runtime_get_health(mh_runtime_health_t *out_health);
void mh_runtime_set_log_sink(mh_runtime_log_sink_t sink, void *ctx);

/* H7-4: hooks for the control application. */

/* Called from mh_runtime_loop() for every RX_FRAME_URC. rx->data is only
 * valid for the duration of the call. */
typedef void (*mh_runtime_rx_handler_t)(const mh_link_rx_t *rx, void *ctx);
void mh_runtime_set_rx_handler(mh_runtime_rx_handler_t handler, void *ctx);

/* Called from mh_runtime_loop() whenever the TX gate releases. */
typedef void (*mh_runtime_tx_handler_t)(mh_link_event_t event, void *ctx);
void mh_runtime_set_tx_handler(mh_runtime_tx_handler_t handler, void *ctx);

/* True while one of our TX_FRAME_REQs is still at the L072. */
bool mh_runtime_tx_busy(void);

/* Send one LoRa packet body as TX_FRAME_REQ. Refused while the gate is
 * busy. timeout_ms bounds how long the gate waits for a terminal URC. */
bool mh_runtime_submit_tx(const uint8_t *data, uint8_t len, uint32_t timeout_ms);

bool mh_runtime_get_tx_gate(mh_link_gate_t *out_gate);

#ifdef __cplusplus
}
#endif

#endif /* LIFETRAC_TRACTOR_H7_MH_RUNTIME_H */
