#ifndef LIFETRAC_MURATA_L072_HOST_RX_WIRE_H
#define LIFETRAC_MURATA_L072_HOST_RX_WIRE_H

/*
 * F8 (2026-07-30): RX_FRAME_URC payload packing, hoisted out of
 * host_cmd.c into a PURE helper so the wire layout is bench-pinnable
 * (mirrors the host_rfco "do NOT pack inline" precedent). No HW or
 * UART calls; includes only sx1276_rx.h, which compiles clean under
 * host gcc with and without LIFETRAC_FHSS_TX_ROUTED.
 *
 * Layout (see host_types.h HOST_TYPE_RX_FRAME_URC):
 *   [0]      u8   len          radio payload length, EXCLUDING the tail —
 *                              every pre-F8 parser slices by this byte,
 *                              which is what makes the tail additive
 *   [1]      i8   snr_db
 *   [2..3]   i16  rssi_dbm     LE
 *   [4..7]   u32  timestamp_us LE
 *   [8..8+len)    payload
 *   -- F8 additive tail, ALWAYS present, +8 bytes --
 *   [8+len+0] u8  phase_flags  bit0 = hop header valid; rest reserved 0
 *   [8+len+1] u8  profile_id   0 when flags bit0 clear
 *   [8+len+2] u8  hop_idx      0 when clear
 *   [8+len+3] u8  slot_offset_ms  (TX saturates at 255; F7: may exceed
 *                               199 on a boundary straddle)
 *   [8+len+4..7] u32 epoch     LE, 0 when clear
 *
 * The tail is zero-filled with flags=0 when hdr_valid is false. That
 * guard is load-bearing: in UNROUTED builds `hdr` is never written
 * (sx1276_rx.c sets only hdr_valid=false), so reading it without the
 * gate would serialize uninitialized stack onto the wire.
 */

#include <stdint.h>

#include "host_types.h"
#include "sx1276_rx.h"

/* HOST_RX_FRAME_URC_TAIL_LEN / HOST_RX_FRAME_URC_PHASE_VALID live in
 * host_types.h — the wire-contract home that check_mh_wire_sync.py
 * compares against the H7's mh_wire.h. */

/*
 * Pack the full URC payload (fixed header + payload + tail) into `out`.
 * Returns the number of bytes written (8 + f->length + 8), or 0 when
 * f/out is NULL or out_cap is too small. Pure.
 */
uint16_t host_rx_frame_urc_pack(const sx1276_rx_frame_t *f,
                                uint8_t *out, uint16_t out_cap);

#endif /* LIFETRAC_MURATA_L072_HOST_RX_WIRE_H */
