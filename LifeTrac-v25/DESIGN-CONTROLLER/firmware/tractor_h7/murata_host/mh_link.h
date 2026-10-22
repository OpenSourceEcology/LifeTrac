#ifndef LIFETRAC_TRACTOR_H7_MH_LINK_H
#define LIFETRAC_TRACTOR_H7_MH_LINK_H

/*
 * H7-4: the application side of the Method G radio link.
 *
 * mh_link_rx_parse() pulls the LoRa packet out of an RX_FRAME_URC.
 *
 * mh_link_gate_t keeps at most ONE of our TX_FRAME_REQs at the L072 at a
 * time (K = 1, the FHSS default of TODO H7-7). The L072 TX ring is a pure
 * FIFO, so the only way to keep a P0 frame from waiting behind our own
 * telemetry is to not park telemetry there. The gate is released by any
 * terminal event for the frame in flight:
 *   - TX_DONE_URC with the same tx_id (status 0 = sent, else aborted),
 *   - ERR_PROTO_URC for a TX_FRAME_REQ with the same seq (bad length,
 *     queue full, or the drain-time FORBIDDEN),
 *   - BOOT_URC (the L072 reset and forgot everything),
 *   - a timeout from submit (a lost URC).
 * FAULT_URC is diagnostic only; the L072 keeps its TX ring through it, so
 * it releases nothing.
 *
 * Pure C with no I/O so the host vectors can pin it.
 */

#include "murata_host.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum mh_link_event_e {
    MH_LINK_EV_NONE = 0,
    MH_LINK_EV_SENT,        /* TX_DONE_URC, status OK */
    MH_LINK_EV_ABORTED,     /* TX_DONE_URC, status != OK (LBT, dwell, ...) */
    MH_LINK_EV_REJECTED,    /* ERR_PROTO_URC for our TX_FRAME_REQ */
    MH_LINK_EV_RESET,       /* BOOT_URC while a frame was in flight */
    MH_LINK_EV_TIMEOUT      /* no terminal URC before the deadline */
} mh_link_event_t;

typedef struct mh_link_gate_s {
    bool in_flight;
    uint8_t tx_id;
    uint16_t seq;
    uint32_t deadline_ms;
    uint8_t next_tx_id;
    uint8_t last_status;    /* TX_DONE status, or ERR_PROTO err_code */

    uint32_t submitted;
    uint32_t sent;
    uint32_t aborted;
    uint32_t rejected;
    uint32_t reset;
    uint32_t timeouts;
} mh_link_gate_t;

typedef struct mh_link_rx_s {
    const uint8_t *data;    /* points into the frame it was parsed from */
    uint8_t len;
    int16_t rssi_dbm;
    int8_t snr_db;
    uint32_t timestamp_us;
} mh_link_rx_t;

/* Parse an RX_FRAME_URC. Accepts both the legacy (8 + len) and the F8
 * extended (8 + len + 8) layouts. False for any other frame type or a
 * truncated payload. */
bool mh_link_rx_parse(const murata_host_frame_t *frame, mh_link_rx_t *out);

void mh_link_gate_init(mh_link_gate_t *gate);
bool mh_link_gate_busy(const mh_link_gate_t *gate);

/* tx_id for the next submit. Never 0xFE, which the X8 image daemon uses
 * for its command frames. */
uint8_t mh_link_gate_next_tx_id(mh_link_gate_t *gate);

/* Record a TX_FRAME_REQ that was written to the UART. */
void mh_link_gate_on_submit(mh_link_gate_t *gate,
                            uint8_t tx_id,
                            uint16_t seq,
                            uint32_t now_ms,
                            uint32_t timeout_ms);

/* Feed every frame from the L072. Returns the event that released the
 * gate, or MH_LINK_EV_NONE. */
mh_link_event_t mh_link_gate_on_frame(mh_link_gate_t *gate,
                                      const murata_host_frame_t *frame);

/* Call once per loop; returns MH_LINK_EV_TIMEOUT when the deadline passed. */
mh_link_event_t mh_link_gate_tick(mh_link_gate_t *gate, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* LIFETRAC_TRACTOR_H7_MH_LINK_H */
