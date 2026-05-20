#include "sx1276_rx.h"

#include "host_cmd.h"
#include "host_stats.h"
#include "platform.h"
#include "sx1276.h"
#include "sx1276_modes.h"
#ifdef LIFETRAC_FHSS_TX_ROUTED
#include "sx1276_fhss.h"
#include "sx1276_rx_scan_fail.h"
#include "sx1276_tx.h"
#endif

#define SX1276_REG_FIFO                   0x00U
#define SX1276_REG_FIFO_ADDR_PTR          0x0DU
#define SX1276_REG_FIFO_RX_CURRENT_ADDR   0x10U
#define SX1276_REG_IRQ_FLAGS              0x12U
#define SX1276_REG_RX_NB_BYTES            0x13U
#define SX1276_REG_PKT_SNR_VALUE          0x19U
#define SX1276_REG_PKT_RSSI_VALUE         0x1AU
#define SX1276_REG_MODEM_CONFIG2          0x1EU

#define SX1276_IRQ_VALID_HEADER           0x10U
#define SX1276_IRQ_PAYLOAD_CRC_ERROR      0x20U
#define SX1276_IRQ_RX_DONE                0x40U

_Static_assert(sizeof(sx1276_rx_frame_t) <= 280U,
               "sx1276_rx_frame_t must stay within main-loop stack budget");

bool sx1276_rx_arm(void) {
    if (!sx1276_modes_to_rx_cont()) {
        return false;
    }

    if ((sx1276_read_reg(SX1276_REG_MODEM_CONFIG2) & (1U << 2)) == 0U) {
        host_cmd_emit_fault(HOST_FAULT_CODE_RX_CRC_DISABLED, 0U);
        (void)sx1276_modes_to_standby();
        return false;
    }

    return true;
}

void sx1276_rx_disarm(void) {
    (void)sx1276_modes_to_standby();
}

bool sx1276_rx_service(uint32_t events, sx1276_rx_frame_t *out_frame) {
    uint8_t irq_flags;

    if ((events & SX1276_EVT_DIO0) == 0U || out_frame == NULL) {
        return false;
    }

#ifdef LIFETRAC_FHSS_TX_ROUTED
    /* FCC-A6c-3-b: latch "any DIO0 IRQ observed this scan window" so
     * the A6c-3-c FAIL handler can derive `hw_suspect` (= !got_any_irq
     * && !crc_seen). Cleared on BEGIN_SCAN/ADVANCE_CHANNEL/LOCK in
     * scan_dispatch_action(). Safe to set unconditionally: the flag
     * is only read on the FAIL edge, and the scan dispatcher resets
     * it at every channel-transition boundary. */
    s_scan_got_any_irq = true;
#endif

    out_frame->timestamp_us = platform_now_us();

    irq_flags = sx1276_read_reg(SX1276_REG_IRQ_FLAGS);

    if ((irq_flags & SX1276_IRQ_PAYLOAD_CRC_ERROR) != 0U) {
        host_stats_radio_crc_err();
#ifdef LIFETRAC_FHSS_TX_ROUTED
        /* FCC-A6c-2-b-ii: payload CRC error is a FRAME_INVALID event
         * to the scan SM regardless of any later header check. */
        scan_feed_frame(false);
        /* FCC-A6c-3-b: also latch crc_seen so the A6c-3-c FAIL handler
         * can distinguish "interference / wrong profile on this channel"
         * (CRC errors present) from "channel silent" (no CRC errors).
         * Set AFTER scan_feed_frame so a FRAME_INVALID that triggers
         * SCANNING→ADVANCE_CHANNEL clears the flag first, then this
         * line re-sets it — preserving the invariant "crc_seen reflects
         * the current window only" after the dispatcher's clear runs. */
        s_scan_crc_seen = true;
#endif
        sx1276_write_reg(SX1276_REG_IRQ_FLAGS, irq_flags);
        return false;
    }

    if ((irq_flags & SX1276_IRQ_RX_DONE) != 0U) {
        const uint8_t rx_len = sx1276_read_reg(SX1276_REG_RX_NB_BYTES);
        const uint8_t fifo_addr = sx1276_read_reg(SX1276_REG_FIFO_RX_CURRENT_ADDR);
        const int8_t snr_q4 = (int8_t)sx1276_read_reg(SX1276_REG_PKT_SNR_VALUE);
        const uint8_t pkt_rssi = sx1276_read_reg(SX1276_REG_PKT_RSSI_VALUE);

        sx1276_write_reg(SX1276_REG_FIFO_ADDR_PTR, fifo_addr);
        if (rx_len > 0U) {
            sx1276_read_burst(SX1276_REG_FIFO, out_frame->payload, rx_len);
        }

        out_frame->length = rx_len;
        out_frame->snr_db = (int8_t)(snr_q4 / 4);
        out_frame->rssi_dbm = (int16_t)pkt_rssi - 157;
        out_frame->hdr_valid = false;

#ifdef LIFETRAC_FHSS_TX_ROUTED
        /*
         * FCC-A6b (plan §14.2 step 10, delta #13): strip the 8-byte
         * hop-sync header from the front of every FHSS frame. Frames
         * shorter than the header, with a bad schema_ver, or that
         * otherwise fail to parse are dropped here (return false)
         * rather than handed up the stack, so the upper layer cannot
         * mistake a stray non-FHSS frame for a hop-sync packet. The
         * IRQ flag is still acked so the modem keeps RX-cont running.
         * Visibility into the drop count belongs to FCC-B1-SUMMARY.
         */
        {
            lora_pkt_hdr_t parsed;
            const lora_pkt_hdr_status_t pst =
                lora_pkt_hdr_unpack(out_frame->payload, rx_len, &parsed);
            if (pst != LORA_PKT_HDR_OK) {
                /* FCC-A6c-2-b-ii: header-parse failure (bad schema,
                 * short frame, etc.) → FRAME_INVALID to the scan SM. */
                scan_feed_frame(false);
                sx1276_write_reg(SX1276_REG_IRQ_FLAGS, irq_flags);
                return false;
            }
            out_frame->hdr       = parsed;
            out_frame->hdr_valid = true;
            {
                /* FCC-A6b-2-ii-β: gate the local FHSS scheduler on the
                 * remote's claimed (epoch, hop_idx). The pure-policy
                 * decision (ALIGNED/SNAPPED/REJECTED_*) is captured in
                 * a saturating per-decision counter for FCC-B1-SUMMARY
                 * to surface via an additive URC. No host_stats field
                 * is added here (would bump the host↔X8 wire layout).
                 * REJECTED_* outcomes do NOT drop the frame — the
                 * payload is still useful even if the snap was refused.
                 * REJECTED_BAD_HOP only fires for a malformed remote
                 * hop_idx (>= CHANNEL_COUNT), which the A6a header
                 * spec disallows; counting it surfaces a remote-side
                 * encoder bug rather than a transport problem. */
                const sx1276_fhss_snap_decision_t dec =
                    sx1276_fhss_consider_remote(parsed.epoch,
                                                parsed.hop_idx);
                sx1276_rx_counter_record(dec);
                /* FCC-A6c-2-b-ii: header unpacked + schema OK →
                 * FRAME_VALID with header_valid=true. This is what
                 * drives SCANNING→LOCKED in the A6c-1 policy; A6c-2-c
                 * will additionally seed γ-1 from (epoch, hop_idx). */
                scan_feed_frame(true);
            }
            {
                const uint8_t payload_len =
                    (uint8_t)(rx_len - LORA_PKT_HDR_LEN);
                if (payload_len > 0U) {
                    /* memmove-equivalent without pulling string.h:
                     * source and dest overlap by 8 bytes guaranteed
                     * forward direction is safe. */
                    for (uint8_t i = 0U; i < payload_len; ++i) {
                        out_frame->payload[i] =
                            out_frame->payload[i + LORA_PKT_HDR_LEN];
                    }
                }
                out_frame->length = payload_len;
            }
        }
#endif /* LIFETRAC_FHSS_TX_ROUTED */

        host_stats_radio_rx_ok();
        sx1276_write_reg(SX1276_REG_IRQ_FLAGS, irq_flags);
        return true;
    }

    if ((irq_flags & SX1276_IRQ_VALID_HEADER) != 0U) {
        sx1276_write_reg(SX1276_REG_IRQ_FLAGS, irq_flags);
        return false;
    }

    sx1276_write_reg(SX1276_REG_IRQ_FLAGS, irq_flags);
    return false;
}

/*
 * FCC-A6b-2-ii-γ-2 (plan §14.2 step 10): RX retune tick.
 *
 * Unrouted build: no-op (caller in main.c does not need an ifdef).
 *
 * Routed build: consult the γ-1 pure-function policy with the current
 * monotonic ms tick + sx1276_tx_busy() + sx1276_fhss_is_initialized()
 * and act on the decision. The decision is always recorded in the
 * HW-free retune-counter TU regardless of HW action, so FCC-B1-SUMMARY
 * can read the histogram via sx1276_rx_retune_counts() (additive URC).
 *
 * PLL settle budget mirrors sx1276_tx.c's pll_settle_busy_wait at
 * SX1276_TX_PLL_SETTLE_US (200 µs). Duplicated rather than hoisted
 * to keep this change a point edit; the two sites have identical
 * physical justification (no settle IRQ on the L072 wiring) and
 * different operational meaning (TX vs RX retune) per the comment in
 * sx1276_tx.c.
 *
 * Known γ-2 limitation: the placeholder period is the FCC dwell cap
 * (380 ms ≥ SF12/BW125/PL255 ToA), so a worst-case in-flight long
 * frame COULD be interrupted. γ-3 fixes this by tying the period to
 * the Q2 epoch model so TX and RX retune at the same instant.
 */
#ifdef LIFETRAC_FHSS_TX_ROUTED

#ifndef SX1276_RX_PLL_SETTLE_US
#define SX1276_RX_PLL_SETTLE_US     200UL
#endif

/* FCC-A6c-2-b/c file-statics: defined here (top of routed block) so
 * sx1276_rx_tick() below can read s_scan_state for the LOCKED gate.
 * Default zero-init equals SX1276_RX_SCAN_STATE_BOOT (=0). */
static sx1276_rx_scan_state_t s_scan_state;
static uint32_t               s_scan_channel_entry_ms;
static uint32_t               s_scan_cold_start_entry_ms;

/* FCC-A6c-3-b file-statics: disambiguation bookkeeping for the
 * SCANNING→FAILED edge. Read on FAIL by A6c-3-c to assemble the
 * `sub` byte of HOST_FAULT_CODE_RX_SCAN_FAILED per
 * AI NOTES/2026-05-19_RX_Scan_FAILED_State_Analysis §13.1 #2.
 *
 *   s_scan_fail_retries           — attempt counter (0..MAX_RETRIES).
 *                                   Incremented by A6c-3-c FAIL handler;
 *                                   reset on LOCK. Saturates so the
 *                                   `sub` 4-bit attempt field is safe.
 *   s_scan_got_any_irq            — any DIO0 event observed by
 *                                   sx1276_rx_service() since the last
 *                                   BEGIN_SCAN/ADVANCE_CHANNEL. Used
 *                                   to derive `hw_suspect` at FAIL.
 *   s_scan_crc_seen               — >=1 RX with CRC error observed
 *                                   since the last BEGIN_SCAN/
 *                                   ADVANCE_CHANNEL. Used to derive
 *                                   `crc_seen` at FAIL.
 *   s_scan_ever_locked_this_boot  — sticky: set on LOCK, never
 *                                   cleared. Used to derive `warm`
 *                                   at FAIL.
 *
 * A6c-3-b adds the storage + write paths only. A6c-3-c adds the
 * read path and the URC emission. No external observer (no host
 * counter, no fault) can see these yet. */
static uint8_t s_scan_fail_retries;
static bool    s_scan_got_any_irq;
static bool    s_scan_crc_seen;
static bool    s_scan_ever_locked_this_boot;

static uint32_t s_rx_last_retune_ms;
static uint8_t  s_rx_last_retune_ms_valid;

static void rx_pll_settle_busy_wait(uint32_t delay_us) {
    const uint32_t start = platform_now_us();
    while ((uint32_t)(platform_now_us() - start) < delay_us) {
        /* spin — SX1276 has no settle IRQ on the L072 wiring */
    }
}

void sx1276_rx_tick(uint32_t now_ms) {
    /* FCC-A6c-2-c-i: γ-1 retune is now gated on the A6c-1 scan SM
     * having reached LOCKED. While in BOOT / SCANNING / FAILED the
     * scan dispatcher (scan_dispatch_action) owns the synth, so
     * γ-1 must NOT also retune or the two loops will fight. The
     * counter is intentionally NOT recorded for the gated-out case
     * — γ-1 simply did not run, and the scan counters already
     * capture what happened in that interval. */
    if (s_scan_state != SX1276_RX_SCAN_STATE_LOCKED) {
        /* Drop the anchor so the first tick post-LOCK re-anchors
         * cleanly rather than firing a spurious DO_WRAP off stale
         * pre-LOCK timing. */
        s_rx_last_retune_ms_valid = 0U;
        return;
    }

    /* First call after boot/reset OR first call after re-entering
     * LOCKED: anchor without consulting the policy. */
    if (s_rx_last_retune_ms_valid == 0U) {
        s_rx_last_retune_ms       = now_ms;
        s_rx_last_retune_ms_valid = 1U;
        return;
    }

    const bool busy  = sx1276_tx_busy();
    const bool ready = (sx1276_fhss_is_initialized() != 0U);
    const sx1276_rx_retune_decision_t dec =
        sx1276_rx_retune_eval(now_ms, s_rx_last_retune_ms, busy, ready);

    sx1276_rx_retune_counter_record(dec);

    switch (dec) {
        case SX1276_RX_RETUNE_DO_WRAP:
            /* Tick wrapped or caller fed backwards time — just
             * re-anchor and let the next tick evaluate cleanly. No
             * HW action. */
            s_rx_last_retune_ms = now_ms;
            return;

        case SX1276_RX_RETUNE_DO: {
            uint8_t  next_idx = 0U;
            uint32_t next_hz  = 0U;
            const sx1276_fhss_status_t fst =
                sx1276_fhss_next_channel(&next_idx, &next_hz);
            if (fst != SX1276_FHSS_OK) {
                /* Scheduler refused (e.g., went uninit between the
                 * policy check and now — unlikely but the API surface
                 * is fail-closed). Do NOT touch the synth. Re-anchor
                 * so we don't busy-loop on every tick. */
                s_rx_last_retune_ms = now_ms;
                return;
            }
            /* Order matches sx1276_tx.c's TX retune: standby → set
             * freq → PLL settle → re-arm RX-cont. */
            (void)sx1276_modes_to_standby();
            sx1276_set_frequency_hz(next_hz);
            rx_pll_settle_busy_wait(SX1276_RX_PLL_SETTLE_US);
            (void)sx1276_rx_arm();
            s_rx_last_retune_ms = now_ms;
            return;
        }

        case SX1276_RX_RETUNE_SKIP_NOT_INIT:
        case SX1276_RX_RETUNE_SKIP_BUSY:
        case SX1276_RX_RETUNE_SKIP_TOO_SOON:
        default:
            /* No HW action. Counter already recorded above. */
            return;
    }
}

#else  /* !LIFETRAC_FHSS_TX_ROUTED */

void sx1276_rx_tick(uint32_t now_ms) {
    (void)now_ms;
}

#endif /* LIFETRAC_FHSS_TX_ROUTED */

/*
 * FCC-A6c-2-b: Scanning tick — observe-only bookkeeping.
 *
 * Drives the A6c-1 state machine forward each main-loop iteration
 * via event=TICK, records the (state, action) pair into the A6c-2-a
 * counters TU, and maintains the file-static (state, anchors) per
 * the returned action. No HW dispatch yet — A6c-2-c will wire
 * BEGIN_SCAN / ADVANCE_CHANNEL / LOCK / FAIL to the modem and
 * feed FRAME_VALID / FRAME_INVALID from sx1276_rx_service().
 *
 * Anchor semantics:
 *   BEGIN_SCAN      → channel + cold = now_ms (BOOT → SCANNING entry)
 *   ADVANCE_CHANNEL → channel        = now_ms (cold budget preserved)
 *   REANCHOR        → channel + cold = now_ms (clock-glitch recovery)
 *   LOCK / FAIL / HOLD → anchors unchanged (terminal states or no-op)
 */
#ifdef LIFETRAC_FHSS_TX_ROUTED

/*
 * FCC-A6c-2-c-i: HW dispatch for scan SM actions BEGIN_SCAN and
 * ADVANCE_CHANNEL. Sequence mirrors γ-2 RX retune (and TX retune):
 * standby → sx1276_fhss_next_channel() → set_frequency → PLL settle
 * → sx1276_rx_arm() (RX-cont). The two scan actions share this
 * path because both semantically mean "the receiver should now be
 * listening on the next channel in the scan order". LOCK and FAIL
 * HW dispatch land in A6c-2-c-ii (LOCK: ensure on the snapped-to
 * FHSS channel; FAIL: standby + alert URC). HOLD / REANCHOR
 * remain pure-bookkeeping.
 *
 * If sx1276_fhss_next_channel() fails (uninitialized scheduler),
 * leave the synth alone and let the scan SM try again on the next
 * dispatched action — the counter has already recorded the
 * intended action; A6c-2-c-ii could add a saturating "dispatch
 * refused" sub-counter if observability demands it.
 */
static void scan_dispatch_action(sx1276_rx_scan_action_t action,
                                 uint32_t                now_ms) {
    switch (action) {
        case SX1276_RX_SCAN_ACTION_BEGIN_SCAN:
        case SX1276_RX_SCAN_ACTION_ADVANCE_CHANNEL: {
            uint8_t  next_idx = 0U;
            uint32_t next_hz  = 0U;
            const sx1276_fhss_status_t fst =
                sx1276_fhss_next_channel(&next_idx, &next_hz);
            if (fst != SX1276_FHSS_OK) {
                return;
            }
            /* FCC-A6c-3-b: fresh scan window for the next channel —
             * clear the per-window disambiguation flags so they
             * reflect only activity observed on the upcoming dwell.
             * Cleared BEFORE the HW transition so a re-entrant IRQ
             * fired during the standby/set_freq window correctly
             * latches against the new window, not the old. */
            s_scan_got_any_irq = false;
            s_scan_crc_seen    = false;
            (void)sx1276_modes_to_standby();
            sx1276_set_frequency_hz(next_hz);
            rx_pll_settle_busy_wait(SX1276_RX_PLL_SETTLE_US);
            (void)sx1276_rx_arm();
            return;
        }
        case SX1276_RX_SCAN_ACTION_LOCK:
            /* FCC-A6c-2-c-ii: LOCK is intentionally HW-no-op. The
             * frame whose FRAME_VALID event caused SCANNING→LOCKED
             * was received in RX-cont, so the modem is already
             * armed on the correct channel; sx1276_fhss_consider_
             * remote() (called in sx1276_rx_service before
             * scan_feed_frame) has already snapped the FHSS
             * pointer to the remote's (epoch, hop_idx). On the
             * very next sx1276_rx_tick() the LOCKED gate opens
             * and γ-1 takes ownership of the synth from the
             * (now-quiescent) scan dispatcher. Driving an extra
             * standby + set_freq + arm here would only add
             * a needless RX gap right after first contact. */
            /* FCC-A6c-3-b: LOCK is the success edge. Reset the
             * A6c-3 retry counter so any future SCANNING re-entry
             * starts from attempt 0, clear the per-window flags
             * (their meaning is window-local), and latch the
             * sticky `warm` bit so the next FAIL — if any — can
             * report "we had a working link this boot". */
            s_scan_fail_retries          = 0U;
            s_scan_got_any_irq           = false;
            s_scan_crc_seen              = false;
            s_scan_ever_locked_this_boot = true;
            return;
        case SX1276_RX_SCAN_ACTION_FAIL: {
            /* FCC-A6c-3-c: B-prime retry supervisor. Snapshot the
             * per-window + sticky flags into the pure helper, emit
             * the HOST_FAULT_CODE_RX_SCAN_FAILED URC with the
             * §13.1 #2 sub-byte, and either re-enter SCANNING with
             * fresh anchors (non-final) or stay in the absorbing
             * FAILED state the policy already committed. */
            const sx1276_rx_scan_fail_input_t fi = {
                .retries     = s_scan_fail_retries,
                .max_retries = SX1276_RX_SCAN_MAX_RETRIES,
                .got_any_irq = s_scan_got_any_irq,
                .crc_seen    = s_scan_crc_seen,
                .ever_locked = s_scan_ever_locked_this_boot,
            };
            const sx1276_rx_scan_fail_eval_t fd =
                sx1276_rx_scan_fail_eval(&fi);

            host_cmd_emit_fault(HOST_FAULT_CODE_RX_SCAN_FAILED, fd.sub);

            if (fd.is_final) {
                /* FCC-A6c-2-c-ii: park the modem in standby — no
                 * point burning power scanning a hopset the remote
                 * provably is not on. Leave s_scan_state at FAILED
                 * (the policy already committed it); external
                 * recovery (host reset / profile renegotiation)
                 * is the only way out. */
                (void)sx1276_modes_to_standby();
                return;
            }

            /* Non-final: account the retry, override the policy's
             * FAILED commit back to SCANNING, reset BOTH anchors so
             * the next 30 s redesign budget restarts cleanly, and
             * dispatch a BEGIN_SCAN-style HW re-entry (next channel
             * + standby + set_freq + settle + arm). Per-window
             * flags are cleared by the recursive BEGIN_SCAN case
             * via this same dispatcher, preserving the invariant
             * that got_any_irq + crc_seen reflect only the current
             * window. */
            s_scan_fail_retries        = (uint8_t)(s_scan_fail_retries + 1U);
            s_scan_state               = SX1276_RX_SCAN_STATE_SCANNING;
            s_scan_channel_entry_ms    = now_ms;
            s_scan_cold_start_entry_ms = now_ms;
            scan_dispatch_action(SX1276_RX_SCAN_ACTION_BEGIN_SCAN, now_ms);
            return;
        }
        case SX1276_RX_SCAN_ACTION_HOLD:
        case SX1276_RX_SCAN_ACTION_REANCHOR:
        default:
            /* HOLD: no transition, no HW touch. REANCHOR: clock-
             * glitch recovery — anchors were already updated in
             * scan_drive, no HW action wanted. */
            return;
    }
}

/*
 * Shared driver for the A6c-1 SM. Builds the eval input from the
 * file-static (state, anchors) + the caller-supplied (event,
 * header_valid, now_ms), records the (state, action) counter,
 * updates anchors per the returned action, and stores next_state.
 * Used by both the periodic tick (event=TICK) and the
 * sx1276_rx_service() frame-event feeds (event=FRAME_VALID/
 * FRAME_INVALID, A6c-2-b-ii).
 *
 * Observe-only: this routine NEVER touches HW. A6c-2-c will add a
 * separate dispatch step keyed off `dec.action`.
 */
static void scan_drive(sx1276_rx_scan_event_t event,
                       bool                    header_valid,
                       uint32_t                now_ms) {
    const sx1276_rx_scan_input_t in = {
        .state               = s_scan_state,
        .now_ms              = now_ms,
        .channel_entry_ms    = s_scan_channel_entry_ms,
        .cold_start_entry_ms = s_scan_cold_start_entry_ms,
        .event               = event,
        .frame_header_valid  = header_valid,
    };
    const sx1276_rx_scan_decision_t dec = sx1276_rx_scan_eval(&in);

    sx1276_rx_scan_counter_record(s_scan_state, dec.action);

    switch (dec.action) {
        case SX1276_RX_SCAN_ACTION_BEGIN_SCAN:
        case SX1276_RX_SCAN_ACTION_REANCHOR:
            s_scan_channel_entry_ms    = now_ms;
            s_scan_cold_start_entry_ms = now_ms;
            break;
        case SX1276_RX_SCAN_ACTION_ADVANCE_CHANNEL:
            s_scan_channel_entry_ms = now_ms;
            break;
        case SX1276_RX_SCAN_ACTION_HOLD:
        case SX1276_RX_SCAN_ACTION_LOCK:
        case SX1276_RX_SCAN_ACTION_FAIL:
        default:
            /* anchors unchanged */
            break;
    }

    s_scan_state = dec.next_state;

    /* FCC-A6c-2-c-i: drive HW AFTER state + anchors are committed so
     * a re-entrant IRQ that fires sx1276_rx_service() during the
     * standby/set_freq window observes the new state. */
    scan_dispatch_action(dec.action, now_ms);
}

void sx1276_rx_scan_tick(uint32_t now_ms) {
    scan_drive(SX1276_RX_SCAN_EVENT_TICK, false, now_ms);
}

/*
 * FCC-A6c-2-b-ii: frame-event feed.
 *
 * Called from sx1276_rx_service() once per modem IRQ that yields a
 * frame-shaped outcome. `header_valid==true` means the A6a header
 * unpack succeeded and the schema/CRC/length checks all passed (the
 * same precondition the existing γ-1 path uses before calling
 * sx1276_fhss_consider_remote()). `header_valid==false` covers
 * payload CRC error, RX_DONE with a malformed A6a header, and
 * (when wired) RX_DONE with a wrong-schema frame.
 *
 * The A6c-1 policy treats FRAME_VALID with header_valid=false as a
 * defensive demotion to FRAME_INVALID, so it is safe (but slightly
 * misleading) to pass an "ambiguous" event upward — call sites
 * should pass the exact event they observed.
 *
 * Observe-only this increment — see scan_drive() docstring.
 */
static void scan_feed_frame(bool header_valid) {
    scan_drive(header_valid ? SX1276_RX_SCAN_EVENT_FRAME_VALID
                            : SX1276_RX_SCAN_EVENT_FRAME_INVALID,
               header_valid,
               platform_now_ms());
}

#else  /* !LIFETRAC_FHSS_TX_ROUTED */

void sx1276_rx_scan_tick(uint32_t now_ms) {
    (void)now_ms;
}

#endif /* LIFETRAC_FHSS_TX_ROUTED */
