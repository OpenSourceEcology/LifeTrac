#ifndef LIFETRAC_MURATA_L072_SX1276_RX_H
#define LIFETRAC_MURATA_L072_SX1276_RX_H

#include <stdbool.h>
#include <stdint.h>

#include "lora_pkt_hdr.h"
#include "sx1276_fhss.h"
#include "sx1276_rx_retune_policy.h"
#include "sx1276_rx_scan_policy.h"

typedef struct sx1276_rx_frame_s {
    uint8_t payload[256];
    uint8_t length;
    int8_t snr_db;
    int16_t rssi_dbm;
    uint32_t timestamp_us;
    /*
     * FCC-A6b (plan §14.2 step 10): when LIFETRAC_FHSS_TX_ROUTED is
     * defined at compile time, the leading LORA_PKT_HDR_LEN bytes of
     * each frame are an A6a hop-sync header that the RX service path
     * strips into `hdr` and removes from `payload`. `hdr_valid` is
     * true iff that parse succeeded; when unrouted, `hdr_valid` is
     * always false and `payload`/`length` contain the raw FIFO bytes
     * unchanged. Frames with a corrupt or unknown-schema header are
     * dropped before reaching the caller in routed mode.
     */
    lora_pkt_hdr_t hdr;
    bool hdr_valid;
} sx1276_rx_frame_t;

bool sx1276_rx_arm(void);
void sx1276_rx_disarm(void);
bool sx1276_rx_service(uint32_t events, sx1276_rx_frame_t *out_frame);

/*
 * FCC-A6b-2-ii-β: per-decision counters for the snap-policy gate.
 * Indexed by sx1276_fhss_snap_decision_t (0..5). Mutated only under
 * LIFETRAC_FHSS_TX_ROUTED (consider_remote() is called from
 * sx1276_rx_service after a successful A6a header parse). The
 * getter is unconditionally available so FCC-B1-SUMMARY can read it
 * without an ifdef wrapper — when unrouted, all entries are zero.
 *
 * Drop visibility (REJECTED_BAD_HOP / REJECTED_EPOCH_DRIFT / REJECTED_LOCKED_OUT,
 * REJECTED_NOT_INIT) lives here rather than in host_stats to avoid
 * bumping the host↔X8 SerialRPC wire layout — FCC-B1-SUMMARY will
 * expose these via an additive URC, not via a host_stats field. */
#define SX1276_RX_CONSIDER_REMOTE_COUNT_DIM 6U
void sx1276_rx_consider_remote_counts(
    uint32_t out_counts[SX1276_RX_CONSIDER_REMOTE_COUNT_DIM]);
void sx1276_rx_consider_remote_counts_reset(void);

/* Internal: record one snap-policy decision. Exposed in the header
 * only so sx1276_rx.c (the sole producer) can call into the HW-free
 * sx1276_rx_counters.c TU. Saturates at UINT32_MAX. */
void sx1276_rx_counter_record(sx1276_fhss_snap_decision_t dec);

/*
 * FCC-A6b-2-ii-γ-2 (plan §14.2 step 10): RX retune tick.
 *
 * Main loop drives this every iteration with the current monotonic
 * ms tick. It consults the γ-1 pure-function policy
 * (sx1276_rx_retune_eval) and, on `DO`, advances the FHSS scheduler
 * via sx1276_fhss_next_channel(), retunes the synth via
 * sx1276_set_frequency_hz(), waits the PLL settle budget, and re-arms
 * RX-continuous. On `DO_WRAP` it only re-anchors `last_retune_ms` so
 * the next tick evaluates cleanly. SKIP_* outcomes are no-ops at the
 * HW layer.
 *
 * When LIFETRAC_FHSS_TX_ROUTED is NOT defined, this is a no-op (so
 * the main-loop call site does not need an ifdef). When routed but
 * called before sx1276_fhss_init(), the policy returns SKIP_NOT_INIT
 * which is recorded in the per-outcome counter and observable via
 * sx1276_rx_retune_counts() — no HW touched.
 *
 * Known γ-2 limitation: the placeholder period (γ-1's 380 ms) is the
 * FCC dwell cap, so a worst-case SF12/BW125/PL255 frame in flight
 * COULD be interrupted by a retune. γ-3 closes this by tying the
 * period to the epoch model (TX and RX retune at the same instant
 * once Q2 closes). Until then, the `modem_busy` check (currently
 * sx1276_tx_busy() only — RX-mid-frame is not tracked at this layer)
 * is the only guard.
 */
void sx1276_rx_tick(uint32_t now_ms);

/*
 * FCC-A6b-2-ii-γ-2: per-outcome counters for the retune tick.
 * Indexed by sx1276_rx_retune_decision_t (0..4 — γ-1 pinned the
 * numeric values). Saturating-uint32, mirror of the A6b-2-ii-β snap
 * counters. The getter is unconditionally available so FCC-B1-SUMMARY
 * can read it without an ifdef wrapper — when unrouted, all entries
 * are zero. No host_stats field (no host↔X8 SerialRPC wire bump).
 */
void sx1276_rx_retune_counts(
    uint32_t out_counts[SX1276_RX_RETUNE_DECISION_DIM]);
void sx1276_rx_retune_counts_reset(void);

/* Internal: record one retune decision. Exposed only so sx1276_rx.c
 * (the sole producer) can call into the HW-free
 * sx1276_rx_retune_counters.c TU. Saturates at UINT32_MAX. */
void sx1276_rx_retune_counter_record(sx1276_rx_retune_decision_t dec);

/*
 * FCC-A6c-2-b (plan §14.2 step 10): RX cold-start Scanning tick.
 *
 * Main loop drives this every iteration with the current monotonic
 * ms tick, alongside (and BEFORE) sx1276_rx_tick(). The tick
 * consults the A6c-1 pure-function policy
 * (sx1276_rx_scan_eval) with the file-static (state, anchors) and
 * event=TICK; records the (state, action) pair in the A6c-2-a
 * counters TU; and updates the file-static anchors per the
 * returned action (BEGIN_SCAN anchors both, ADVANCE_CHANNEL
 * anchors channel only, REANCHOR anchors both, HOLD/LOCK/FAIL
 * leave anchors alone). The next_state is stored back as the
 * file-static state.
 *
 * A6c-2-b SCOPE: bookkeeping + counters only. NO HW action is
 * driven by any scan-SM action. The existing γ-1 sx1276_rx_tick()
 * continues to drive the FHSS retune loop unconditionally, and
 * sx1276_rx_arm() / sx1276_rx_disarm() remain controlled by
 * main.c. A6c-2-c will gate γ-1 on LOCKED, wire BEGIN_SCAN /
 * ADVANCE_CHANNEL / LOCK / FAIL to actual HW dispatch, and feed
 * FRAME_VALID / FRAME_INVALID events from sx1276_rx_service().
 *
 * Counter observability: FCC-B1-SUMMARY will read both axes via
 * sx1276_rx_scan_state_counts() + sx1276_rx_scan_action_counts()
 * (additive URC). Pre-A6c-2-c the observed histograms will show
 * BOOT→BEGIN_SCAN exactly once at first tick, then SCANNING+TICK
 * accumulating with the ADVANCE_CHANNEL slot ticking every
 * DWELL_MS = 100 ms and FAIL firing at cold_elapsed >=
 * REDESIGN_MS = 30 000 ms — no frame-event histograms until
 * A6c-2-c.
 *
 * When LIFETRAC_FHSS_TX_ROUTED is NOT defined, this is a no-op.
 */
void sx1276_rx_scan_tick(uint32_t now_ms);

/*
 * v25.0.7 slot-clock boundary follower: when LOCKED with a phase-
 * anchored slot clock, retunes the receiver at every hop-slot boundary
 * along the shared schedule — independent of decode success — so one
 * air loss costs one packet instead of a re-acquisition. Call once per
 * main-loop pass after sx1276_rx_scan_tick(). No-op while the clock is
 * unanchored, while not LOCKED, while a TX is keyed, and in unrouted
 * builds.
 */
void sx1276_rx_slot_follow(uint32_t now_ms);

/*
 * 2026-07-24 FHSS bring-up: return the A6c scan SM to BOOT so the next
 * tick runs a fresh BEGIN_SCAN. Called from host_cfg_profile_activate()
 * when the FHSS profile is (re-)activated — without this, an SM that
 * reached the absorbing FAILED state before the peer started radiating
 * (30 s empty-band budget at boot) stays parked in standby FOREVER and
 * the receiver is deaf regardless of later profile activations.
 * No-op when LIFETRAC_FHSS_TX_ROUTED is not defined.
 */
void sx1276_rx_scan_reset(void);

#endif /* LIFETRAC_MURATA_L072_SX1276_RX_H */
