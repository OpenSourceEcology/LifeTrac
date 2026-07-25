#ifndef LIFETRAC_MURATA_L072_SX1276_FHSS_CLOCK_H
#define LIFETRAC_MURATA_L072_SX1276_FHSS_CLOCK_H

/*
 * v25.0.7 slot-clock FHSS (2026-07-24).
 *
 * Root cause being fixed (bench evidence _fhss_run18/19): TX hopped per
 * PACKET, so the receiver could only learn a hop happened by decoding
 * that exact packet — one air loss broke channel sync and cost a
 * multi-second re-acquisition. Delivered throughput was ~5 % of TX.
 *
 * The fix is a shared TIME GRID: hops advance on a fixed slot clock
 * instead of per packet. Both peers run the identical seeded hop
 * sequence (sx1276_fhss.c), every packet carries (epoch, hop_idx,
 * slot_offset_ms) in the A6a header, and both L072s clock from ±2 ppm
 * TCXOs — so after ONE decoded packet the receiver can phase-lock its
 * own slot clock and keep hopping in lock-step through packet loss.
 * Drift math: 2 ppm ≈ 0.12 ms per minute vs a 200 ms slot; every
 * decode re-anchors the phase anyway.
 *
 * Grid geometry:
 *   SLOT_MS = 200  → 5 slots/s, one 50-slot epoch per 10 s.
 *   Per-channel occupancy at full load = 1 packet ToA (≤ ~173 ms)
 *   per 10 s cycle → ≤ 2 visits × ToA ≈ 346 ms in any 20 s window,
 *   inside both FCC §15.247(a)(1)(i) (400 ms / 20 s) and the firmware
 *   legal-dwell accountant (400 ms / 10 s). The per-packet dwell cap
 *   (380 ms) and QoS gate are unchanged and enforced independently.
 *
 * Absolute slot numbering:
 *   abs_slot = epoch * SX1276_FHSS_CHANNEL_COUNT + hop_idx
 *   (u32; wraps after ~85.9 M epochs ≈ 27 years — re-anchoring long
 *   before then makes the wrap unreachable in practice).
 *
 * Anchoring discipline:
 *   - TX side anchors LAZILY at its first FHSS transmission
 *     (sx1276_tx.c) — its own grid is authoritative.
 *   - RX side re-anchors on EVERY accepted A6a header
 *     (sx1276_rx.c): slot_start_local = rx_done_ms - toa_ms -
 *     hdr.slot_offset_ms. Whoever hears the other adopts the other's
 *     grid, so two-way traffic converges on the busier node's clock.
 *   - sx1276_rx_scan_reset() and a LOCKED→SCANNING loss demotion
 *     reset the clock (fresh acquisition ⇒ fresh phase).
 *
 * This TU is HW-free (no register access, no platform calls) so it
 * links unchanged into the host-cc bench targets.
 */

#include <stdint.h>

#include "sx1276_fhss_chantab.h"

#define SX1276_FHSS_SLOT_MS            200U

/* A TX may key up only if its whole frame (ToA) plus this guard fits
 * inside the remaining slot time. Covers PLL settle (~1 ms), SPI FIFO
 * load, main-loop scheduling jitter, and peer clock skew so a packet
 * never straddles a receiver's boundary retune. */
#define SX1276_FHSS_SLOT_TX_GUARD_US   15000UL

/* TX must additionally wait this long into the slot before keying.
 * The receiver's boundary retune (loop-latency + standby + set_freq +
 * 1 ms PLL settle + re-arm) plus anchor skew costs up to ~8 ms; a
 * BW250 8-symbol preamble is only ~4.1 ms, so a boundary-keyed TX
 * finishes its preamble before a racing receiver is listening — the
 * packet is silently lost EVERY slot (run-20 evidence: TX 640+ B/s,
 * RX caught only scan-acquisition hits). Holding TX until +12 ms
 * guarantees the receiver is armed first across worst-case skew. */
#define SX1276_FHSS_SLOT_TX_HEADSTART_MS 12U

/* Forget the anchor. Safe to call at any time. */
void sx1276_fhss_clock_reset(void);

/* Set/replace the anchor: `slot_start_ms` is the local time of the
 * START of absolute slot `abs_slot`. */
void sx1276_fhss_clock_anchor(uint32_t slot_start_ms, uint32_t abs_slot);

/* 1 when an anchor is set. */
uint8_t sx1276_fhss_clock_valid(void);

/* Absolute slot index active at local time `now_ms`. Uses unsigned
 * subtraction so the u32 ms tick wrap is handled as long as the
 * anchor is younger than ~49.7 days (re-anchoring guarantees that).
 * Undefined (returns anchor slot) when no anchor is set — callers
 * must gate on clock_valid(). */
uint32_t sx1276_fhss_clock_abs_slot(uint32_t now_ms);

/* Milliseconds elapsed inside the current slot at `now_ms` (0 ..
 * SLOT_MS-1). */
uint32_t sx1276_fhss_clock_in_slot_ms(uint32_t now_ms);

/* Decompose an absolute slot into the scheduler's (epoch, hop_idx). */
static inline uint32_t sx1276_fhss_clock_epoch_of(uint32_t abs_slot) {
    return abs_slot / SX1276_FHSS_CHANNEL_COUNT;
}
static inline uint8_t sx1276_fhss_clock_hop_of(uint32_t abs_slot) {
    return (uint8_t)(abs_slot % SX1276_FHSS_CHANNEL_COUNT);
}
static inline uint32_t sx1276_fhss_clock_abs_of(uint32_t epoch,
                                                uint8_t hop_idx) {
    return epoch * SX1276_FHSS_CHANNEL_COUNT + (uint32_t)hop_idx;
}

#endif /* LIFETRAC_MURATA_L072_SX1276_FHSS_CLOCK_H */
