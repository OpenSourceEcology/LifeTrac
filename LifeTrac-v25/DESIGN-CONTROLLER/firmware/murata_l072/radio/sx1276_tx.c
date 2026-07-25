#include "sx1276_tx.h"

#include "host_stats.h"
#include "platform.h"
#include "sx1276_airtime.h"
#include "sx1276.h"
#include "sx1276_lbt.h"
#include "sx1276_modes.h"
#include "sx1276_rx.h"

#ifdef LIFETRAC_FHSS_TX_ROUTED
#include "host_cfg_keys.h"
#include "host_cfg_profile.h"
#include "host_rfco.h"
#include "lora_pkt_hdr.h"
#include "sx1276_fhss.h"
#include "sx1276_fhss_clock.h"
#include "sx1276_legal_dwell.h"
#endif

#define SX1276_REG_FIFO                    0x00U
#define SX1276_REG_FIFO_ADDR_PTR           0x0DU
#define SX1276_REG_FIFO_TX_BASE_ADDR       0x0EU
#define SX1276_REG_IRQ_FLAGS               0x12U
#define SX1276_REG_PAYLOAD_LENGTH          0x22U
#define SX1276_REG_PA_CONFIG               0x09U

#define SX1276_IRQ_TX_DONE                 0x08U

#define SX1276_TX_TIMEOUT_GUARD_US         150000UL

#ifdef LIFETRAC_FHSS_TX_ROUTED
/*
 * FCC-A4 (plan §14.2 step 9): PLL settle budget after a frequency
 * change before LBT/CAD/TX-start may sample the channel. The SX1276
 * datasheet (rev 7, §4.1.4 Frequency Synthesis) characterises the
 * PLL lock time at the "PllBandwidth" default (75 kHz) as ~50 µs
 * worst-case across temperature; the 200 µs budget here is a 4x
 * conservative envelope pending bench characterisation under the
 * actual Murata CMWX1ZZABZ TCXO. This delay is intentionally
 * separate from the 1 ms modes_to_standby() settle in sx1276.c so
 * that operators bisecting RFCO logs can distinguish "PLL not
 * locked yet" from "modem still in sleep/standby transition".
 */
#define SX1276_TX_PLL_SETTLE_US            1000UL
#endif

typedef enum sx1276_tx_state_e {
    SX1276_TX_STATE_IDLE = 0,
    SX1276_TX_STATE_WAIT_DONE = 1
} sx1276_tx_state_t;

static sx1276_tx_state_t s_tx_state = SX1276_TX_STATE_IDLE;
static uint8_t s_tx_id;
static uint8_t s_tx_power_dbm;
static uint8_t s_rearm_rx;
static uint32_t s_expected_toa_us;
static uint32_t s_deadline_us;
static uint32_t s_reserved_airtime_us;
static uint8_t s_channel_idx;

#ifdef LIFETRAC_FHSS_TX_ROUTED
static uint32_t s_hop_freq_hz;
static uint8_t  s_hop_idx;
static uint32_t s_hop_epoch;
static uint8_t  s_hop_slot_offset_ms;   /* v25.0.7: in-slot ms at key-up */
static uint16_t s_legal_dwell_handle = SX1276_DWELL_HANDLE_INVALID;
static uint32_t s_predicted_toa_us;
static uint16_t s_rfco_pertx_seq;

/* v25.0.7: is the ACTIVE profile the slot-clocked FHSS one? Bench and
 * DTS are single-carrier — no grid. */
static bool tx_profile_is_fhss(void) {
    const host_cfg_profile_req_t *p = host_cfg_profile_active();
    const uint8_t id = (p != NULL) ? p->profile_id
                                   : (uint8_t)REG_PROFILE_BENCH_ONLY_FIXED_915;
    return id == (uint8_t)REG_PROFILE_FCC_15_247_FHSS_50CH_BW250;
}

static void pll_settle_busy_wait(uint32_t delay_us) {
    const uint32_t start = platform_now_us();
    while ((uint32_t)(platform_now_us() - start) < delay_us) {
        /* spin — SX1276 has no settle IRQ on the L072 wiring */
    }
}

static void tx_emit_rfco_pertx(uint8_t tx_status, uint32_t pkt_toa_us) {
    host_rfco_pertx_t snap;
    const host_cfg_profile_req_t *active = host_cfg_profile_active();
    snap.profile_id = (active != NULL) ? active->profile_id
                                       : (uint8_t)REG_PROFILE_BENCH_ONLY_FIXED_915;
    snap.tx_status               = tx_status;
    snap.hop_idx                 = s_hop_idx;
    snap.channel_idx             = s_channel_idx;
    snap.epoch                   = s_hop_epoch;
    snap.freq_hz                 = s_hop_freq_hz;
    snap.pkt_toa_us              = pkt_toa_us;
    snap.legal_dwell_used_us_10s = sx1276_legal_dwell_used_us(
        s_channel_idx, SX1276_DWELL_WINDOW_10S_MS, platform_now_ms());
    (void)host_rfco_pertx_emit(s_rfco_pertx_seq, 0U, &snap);
    ++s_rfco_pertx_seq;
}
#endif /* LIFETRAC_FHSS_TX_ROUTED */

static uint8_t tx_power_dbm_now(void) {
    return (uint8_t)((sx1276_read_reg(SX1276_REG_PA_CONFIG) & 0x0FU) + 2U);
}

static bool time_reached(uint32_t now, uint32_t deadline) {
    return (int32_t)(now - deadline) >= 0;
}

static void sx1276_tx_cleanup(void) {
    (void)sx1276_modes_to_standby();
    if (s_rearm_rx != 0U) {
        (void)sx1276_rx_arm();
    }

    s_tx_state = SX1276_TX_STATE_IDLE;
    s_rearm_rx = 0U;
    s_reserved_airtime_us = 0U;
}

bool sx1276_tx_begin(const sx1276_tx_request_t *req) {
    const sx1276_state_t state_before = sx1276_modes_get_state();
    sx1276_lbt_result_t lbt_result;
    sx1276_airtime_result_t airtime_result;

    if (req == NULL || s_tx_state != SX1276_TX_STATE_IDLE) {
        return false;
    }

#ifdef LIFETRAC_FHSS_TX_ROUTED
    /*
     * FCC-A6b (plan §14.2 step 10, delta #13): the 8-byte hop-sync
     * header is prepended at FIFO-write time, so every length-bearing
     * calculation downstream (airtime invariant, airtime reserve,
     * legal-dwell reserve, FIFO PAYLOAD_LENGTH) must use the on-air
     * length INCLUDING the header. Refuse requests that would overflow
     * the SX1276's 256-byte FIFO with the header included; do it
     * before consuming an FHSS hop slot so a doomed TX does not burn
     * scheduler progress. Emit an INTERNAL RFCO snapshot with zeroed
     * hop fields for bench traceability.
     */
    if ((uint32_t)req->length + (uint32_t)LORA_PKT_HDR_LEN > 255UL) {
        s_hop_idx     = 0U;
        s_hop_epoch   = 0U;
        s_hop_freq_hz = 0U;
        s_channel_idx = 0U;
        tx_emit_rfco_pertx(HOST_RFCO_TX_STATUS_INTERNAL, 0U);
        return false;
    }
    const uint8_t effective_len = (uint8_t)(req->length + LORA_PKT_HDR_LEN);
#else
    const uint8_t effective_len = req->length;
#endif

#ifdef LIFETRAC_FHSS_TX_ROUTED
    /*
     * FCC-A4 (plan §14.2 step 9): route TX through the FHSS hop
     * scheduler BEFORE LBT/CAD/TX-start. Capturing (hop_idx, epoch)
     * here makes every RFCO_PERTX emit below truthful even on
     * fail-closed paths.
     *
     * 2026-05-25 bench-profile bypass: REG_PROFILE_BENCH_ONLY_FIXED_915
     * intentionally does NOT init the FHSS scheduler (see
     * host_cfg_profile_activate() — only the FCC 50-ch FHSS profile
     * calls sx1276_fhss_init). Without this gate, sx1276_fhss_next_channel()
     * returns NOT_INIT, sx1276_tx_begin() refuses every TX with FORBIDDEN,
     * and bench two-peer air tests can never key the radio. Root cause of
     * the 2026-05-25 air_coupling_rssi_sniff false-negative
     * (rx_frames=0, irq_flags_or=0x00). Decoded ERR_PROTO payload
     * `10 01 08 00 00` = TX_FRAME_REQ + FORBIDDEN + detail=0.
     *
     * Under BENCH profile we leave the radio on whatever FRF the host
     * pinned (typically 915 MHz via the LIFETRAC_FORCE_FRF_HZ host-side
     * workaround in configure_regulatory_profile_if_needed()), skip the
     * retune below, and zero the hop telemetry so RFCO snapshots stay
     * truthful. channel_idx==0 keeps airtime/legal-dwell accounting on
     * a single bucket consistent with the single-channel bench profile.
     */
    {
        const host_cfg_profile_req_t *active_prof = host_cfg_profile_active();
        const uint8_t active_id = (active_prof != NULL)
            ? active_prof->profile_id
            : (uint8_t)REG_PROFILE_BENCH_ONLY_FIXED_915;
        if (active_id == (uint8_t)REG_PROFILE_BENCH_ONLY_FIXED_915 ||
            active_id == (uint8_t)REG_PROFILE_FCC_15_247_DTS_BW500) {
            s_hop_idx     = 0U;
            s_hop_epoch   = 0U;
            s_hop_freq_hz = 0U;
            s_channel_idx = 0U;
            s_hop_slot_offset_ms = 0U;
        } else {
            /*
             * v25.0.7 slot-clock: the hop is a function of TIME, not of
             * the packet count. Anchor the local grid lazily at the
             * first FHSS TX (our own grid is authoritative on the TX
             * side; a receiver phase-locks to us via the header's
             * slot_offset byte). Then force the scheduler to the slot
             * the clock says is active NOW — packet loss, host pauses,
             * or a peer re-activation can never desynchronise the
             * sequence from the grid because the grid is re-derived
             * from the clock on every TX.
             */
            const uint32_t tx_now_ms = platform_now_ms();
            if (sx1276_fhss_clock_valid() == 0U) {
                /* First FHSS TX after boot/activation: this instant
                 * becomes the start of the scheduler's CURRENT slot. */
                const uint32_t cur_abs = sx1276_fhss_clock_abs_of(
                    sx1276_fhss_current_epoch(),
                    sx1276_fhss_current_slot());
                sx1276_fhss_clock_anchor(tx_now_ms, cur_abs);
            }
            {
                const uint32_t abs_now =
                    sx1276_fhss_clock_abs_slot(tx_now_ms);
                (void)sx1276_fhss_snap_to(
                    sx1276_fhss_clock_epoch_of(abs_now),
                    sx1276_fhss_clock_hop_of(abs_now));
            }
            s_hop_slot_offset_ms = (uint8_t)((sx1276_fhss_clock_in_slot_ms(tx_now_ms) > 255U)
                ? 255U
                : sx1276_fhss_clock_in_slot_ms(tx_now_ms));
            uint8_t  hop_channel_idx = 0U;
            uint32_t hop_center_hz   = 0U;
            const sx1276_fhss_status_t hop_st =
                sx1276_fhss_next_channel(&hop_channel_idx, &hop_center_hz);
            if (hop_st != SX1276_FHSS_OK) {
                /* Scheduler not initialised / corrupted: emit an INTERNAL
                 * snapshot with zeroed hop fields so bench post-processing
                 * can detect the fault, then refuse the TX. */
                s_hop_idx     = 0U;
                s_hop_epoch   = 0U;
                s_hop_freq_hz = 0U;
                s_channel_idx = 0U;
                tx_emit_rfco_pertx(HOST_RFCO_TX_STATUS_INTERNAL, 0U);
                return false;
            }
            s_channel_idx = hop_channel_idx;
            s_hop_freq_hz = hop_center_hz;
            /* sx1276_fhss_current_slot() is the NEXT slot the scheduler
             * will hand out; the one we just consumed is therefore
             * (slot - 1) within the current epoch. Slot wrap is folded
             * back to the last index of the previous epoch. */
            {
                const uint8_t slot_after = sx1276_fhss_current_slot();
                s_hop_idx = (slot_after == 0U)
                    ? (uint8_t)(SX1276_FHSS_CHANNEL_COUNT - 1U)
                    : (uint8_t)(slot_after - 1U);
            }
            s_hop_epoch = sx1276_fhss_current_epoch();
        }
    }
#else
    s_channel_idx = 0U;
#endif

    s_rearm_rx = (state_before == SX1276_STATE_RX_CONT || state_before == SX1276_STATE_RX_SINGLE) ? 1U : 0U;
    if (s_rearm_rx != 0U) {
        sx1276_rx_disarm();
    } else if (!sx1276_modes_to_standby()) {
        return false;
    }

#ifdef LIFETRAC_FHSS_TX_ROUTED
    /*
     * Retune the synthesiser now that the modem is in standby, then
     * wait the documented PLL settle budget so LBT/CAD samples the
     * correct channel rather than the previous one.
     *
     * 2026-05-25 bench-profile bypass: under BENCH_ONLY_FIXED_915
     * s_hop_freq_hz was zeroed above (no scheduler), so retuning would
     * drop the synth to 0 Hz and break the host-pinned 915 MHz carrier.
     * Skip retune entirely for bench — the host-side force-FRF
     * workaround owns the channel.
     */
    if (s_hop_freq_hz != 0U) {
        sx1276_set_frequency_hz(s_hop_freq_hz);
        pll_settle_busy_wait(SX1276_TX_PLL_SETTLE_US);
    }
#endif

    lbt_result = sx1276_lbt_check_and_backoff();
    if (lbt_result != SX1276_LBT_RESULT_CLEAR && lbt_result != SX1276_LBT_RESULT_DISABLED) {
#ifdef LIFETRAC_FHSS_TX_ROUTED
        sx1276_fhss_record_lbt_block(s_channel_idx);
        tx_emit_rfco_pertx(HOST_RFCO_TX_STATUS_ABORT_LBT, 0U);
#endif
        sx1276_tx_cleanup();
        return false;
    }

    /*
     * FCC-A1b (plan §14.2 step 4): pre-TX per-frame airtime invariant.
     *
     * The config-time invariant (FCC-A1a) already proves that
     * `(SF, BW, CR)` at MAX payload fits the 380 ms dwell cap, but the
     * modem-config setter only writes registers when accepted, so this
     * check fires if either (a) the running config was changed by an
     * out-of-band path (debug, host REG_WRITE), or (b) caller framing
     * assumptions (preamble length, implicit-header mode, CRC bit)
     * differ from the helper's defaults. Failing closed here keeps the
     * 400 ms FCC §15.247(a)(1)(iii) per-channel dwell legally satisfied
     * even when the modem state is unexpected.
     *
     * `sx1276_airtime_estimate_toa_us()` reads the live modem-config
     * registers + preamble length, so this check is faithful to whatever
     * is actually programmed at TX time.
     */
    {
        const uint32_t predicted_toa_us = sx1276_airtime_estimate_toa_us(effective_len);
        if (predicted_toa_us > SX1276_AIRTIME_DWELL_CAP_US) {
            host_stats_radio_tx_abort_airtime();
#ifdef LIFETRAC_FHSS_TX_ROUTED
            tx_emit_rfco_pertx(HOST_RFCO_TX_STATUS_ABORT_AIRTIME_INVARIANT,
                               predicted_toa_us);
#endif
            sx1276_tx_cleanup();
            return false;
        }
#ifdef LIFETRAC_FHSS_TX_ROUTED
        s_predicted_toa_us = predicted_toa_us;
#endif
    }

    airtime_result = sx1276_airtime_reserve(s_channel_idx,
                                            effective_len,
                                            platform_now_ms(),
                                            &s_reserved_airtime_us);
    if (airtime_result != SX1276_AIRTIME_OK) {
        if (airtime_result == SX1276_AIRTIME_OVER_BUDGET) {
            host_stats_radio_tx_abort_airtime();
        }
#ifdef LIFETRAC_FHSS_TX_ROUTED
        tx_emit_rfco_pertx(HOST_RFCO_TX_STATUS_ABORT_QOS,
                           s_predicted_toa_us);
#endif
        sx1276_tx_cleanup();
        return false;
    }

#ifdef LIFETRAC_FHSS_TX_ROUTED
    /*
     * FCC-A2 (plan §14.2 step 6): pessimistic legal-dwell reservation
     * against the 400 ms / 10 s per-channel cap. The QoS reserve above
     * is the 1 s fairness window; this is the regulatory 10 s window.
     * Both are kept separate per plan delta #11.
     *
     * 2026-05-25 bench-profile bypass: BENCH_ONLY_FIXED_915 is a
     * single-channel non-regulatory profile; with 60-burst probes the
     * 400ms/10s cap would exhaust after ~12 TXs and the rest would
     * abort with LEGAL_DWELL. Bench is explicitly out-of-regulatory
     * scope — skip the reserve so air-coupling falsification can
     * actually exercise the radio. The active profile id is the same
     * sentinel set at the FHSS-scheduler gate above.
     */
    {
        const host_cfg_profile_req_t *dwell_prof = host_cfg_profile_active();
        const uint8_t dwell_id = (dwell_prof != NULL)
            ? dwell_prof->profile_id
            : (uint8_t)REG_PROFILE_BENCH_ONLY_FIXED_915;
        if (dwell_id != (uint8_t)REG_PROFILE_BENCH_ONLY_FIXED_915 &&
            dwell_id != (uint8_t)REG_PROFILE_FCC_15_247_DTS_BW500) {
            s_legal_dwell_handle = SX1276_DWELL_HANDLE_INVALID;
            const sx1276_dwell_status_t dst = sx1276_legal_dwell_reserve(
                s_channel_idx,
                s_predicted_toa_us,
                platform_now_ms(),
                SX1276_DWELL_WINDOW_10S_MS,
                SX1276_DWELL_DEFAULT_CAP_US,
                &s_legal_dwell_handle);
            if (dst != SX1276_DWELL_OK) {
                sx1276_airtime_release(s_channel_idx, s_reserved_airtime_us);
                tx_emit_rfco_pertx(HOST_RFCO_TX_STATUS_ABORT_LEGAL_DWELL,
                                   s_predicted_toa_us);
                sx1276_tx_cleanup();
                return false;
            }
        } else {
            s_legal_dwell_handle = SX1276_DWELL_HANDLE_INVALID;
        }
    }
#endif

    sx1276_write_reg(SX1276_REG_FIFO_TX_BASE_ADDR, 0x00U);
    sx1276_write_reg(SX1276_REG_FIFO_ADDR_PTR, 0x00U);
    sx1276_write_reg(SX1276_REG_PAYLOAD_LENGTH, effective_len);
#ifdef LIFETRAC_FHSS_TX_ROUTED
    /*
     * FCC-A6b (plan §14.2 step 10): write the 8-byte hop-sync header
     * FIRST so it occupies bytes [0..7] of the on-air frame; the RX
     * side relies on this byte ordering to strip it before delivering
     * the upper-layer payload (sx1276_rx.c). schema_ver is hard-coded
     * inside lora_pkt_hdr_pack() — no field-driven path can demote it.
     */
    {
        uint8_t hdr_bytes[LORA_PKT_HDR_LEN];
        const host_cfg_profile_req_t *active = host_cfg_profile_active();
        lora_pkt_hdr_t hdr;
        hdr.profile_id = (active != NULL)
            ? active->profile_id
            : (uint8_t)REG_PROFILE_BENCH_ONLY_FIXED_915;
        hdr.hop_idx        = s_hop_idx;
        hdr.epoch          = s_hop_epoch;
        hdr.slot_offset_ms = s_hop_slot_offset_ms;  /* v25.0.7 phase byte */
        (void)lora_pkt_hdr_pack(&hdr, hdr_bytes);
        sx1276_write_burst(SX1276_REG_FIFO, hdr_bytes, LORA_PKT_HDR_LEN);
    }
#endif
    if (req->length > 0U) {
        sx1276_write_burst(SX1276_REG_FIFO, req->payload, req->length);
    }
    sx1276_write_reg(SX1276_REG_IRQ_FLAGS, 0xFFU);

    if (!sx1276_modes_to_tx()) {
        sx1276_airtime_release(s_channel_idx, s_reserved_airtime_us);
#ifdef LIFETRAC_FHSS_TX_ROUTED
        /* No reconcile of legal_dwell — FCC-A2 contract is no-rollback
         * once reserved. Energy was not emitted but we keep the worst-
         * case booking; the per-minute SUMMARY URC will surface the
         * over-accounting if it ever becomes operationally relevant. */
        tx_emit_rfco_pertx(HOST_RFCO_TX_STATUS_TX_FAIL,
                           s_predicted_toa_us);
#endif
        sx1276_tx_cleanup();
        return false;
    }

    s_tx_id = req->tx_id;
    s_tx_power_dbm = tx_power_dbm_now();
    s_expected_toa_us = s_reserved_airtime_us;
    if (s_expected_toa_us == 0U) {
        s_expected_toa_us = sx1276_airtime_estimate_toa_us(effective_len);
    }
    s_deadline_us = platform_now_us() + s_expected_toa_us + SX1276_TX_TIMEOUT_GUARD_US;
    s_tx_state = SX1276_TX_STATE_WAIT_DONE;
    return true;
}

bool sx1276_tx_poll(uint32_t events, sx1276_tx_result_t *out_result) {
    uint8_t irq_flags;

    if (out_result == NULL || s_tx_state != SX1276_TX_STATE_WAIT_DONE) {
        return false;
    }

    if ((events & SX1276_EVT_DIO0) != 0U) {
        irq_flags = sx1276_read_reg(SX1276_REG_IRQ_FLAGS);
        sx1276_write_reg(SX1276_REG_IRQ_FLAGS, irq_flags);

        if ((irq_flags & SX1276_IRQ_TX_DONE) != 0U) {
            out_result->tx_id = s_tx_id;
            out_result->status = SX1276_TX_STATUS_OK;
            out_result->tx_power_dbm = s_tx_power_dbm;
            out_result->time_on_air_us = s_expected_toa_us;
            sx1276_airtime_commit(s_channel_idx, 0U, platform_now_ms());
            host_stats_radio_tx_ok();
#ifdef LIFETRAC_FHSS_TX_ROUTED
            /* Reconcile dwell down to the predicted ToA. We don't have
             * a hardware-measured actual airtime on the L072 wiring;
             * the predicted ToA from FCC-A1b is the post-rearm best
             * estimate and matches what the FCC accountant must book. */
            sx1276_legal_dwell_reconcile(s_legal_dwell_handle,
                                         s_expected_toa_us);
            tx_emit_rfco_pertx(HOST_RFCO_TX_STATUS_OK, s_expected_toa_us);
#endif
            sx1276_tx_cleanup();
            return true;
        }
    }

    if (time_reached(platform_now_us(), s_deadline_us)) {
        sx1276_write_reg(SX1276_REG_IRQ_FLAGS, 0xFFU);
        out_result->tx_id = s_tx_id;
        out_result->status = SX1276_TX_STATUS_TIMEOUT;
        out_result->tx_power_dbm = s_tx_power_dbm;
        out_result->time_on_air_us = s_expected_toa_us;
        sx1276_airtime_commit(s_channel_idx, 0U, platform_now_ms());
#ifdef LIFETRAC_FHSS_TX_ROUTED
        /* No legal_dwell reconcile — RF energy was emitted but we don't
         * know for how long, so the pessimistic reserve stands per
         * FCC-A2's no-rollback contract. */
        tx_emit_rfco_pertx(HOST_RFCO_TX_STATUS_TX_TIMEOUT, s_expected_toa_us);
#endif
        sx1276_tx_cleanup();
        return true;
    }

    return false;
}

bool sx1276_tx_busy(void) {
    return s_tx_state != SX1276_TX_STATE_IDLE;
}

uint32_t sx1276_tx_slot_wait_us(uint8_t payload_len) {
#ifdef LIFETRAC_FHSS_TX_ROUTED
    /*
     * v25.0.7 slot-clock fit advisory (see include/sx1276_fhss_clock.h).
     * Pure query — NO radio or scheduler side effects, so the caller
     * (host_cmd.c) can park the request in the TX mailbox and retry
     * on a later main-loop pass without burning a hop slot or FIFO
     * state. Returns 0 when the frame may key up now:
     *   - non-FHSS profile (bench / DTS have no grid), or
     *   - clock not yet anchored (first TX anchors it), or
     *   - remaining slot time fits ToA + guard.
     * Otherwise returns the µs until the next slot boundary.
     */
    if (!tx_profile_is_fhss() || sx1276_fhss_clock_valid() == 0U) {
        return 0U;
    }
    {
        const uint32_t now_ms     = platform_now_ms();
        const uint32_t in_slot_ms = sx1276_fhss_clock_in_slot_ms(now_ms);
        const uint32_t in_slot_us = in_slot_ms * 1000UL;
        const uint32_t slot_us    = (uint32_t)SX1276_FHSS_SLOT_MS * 1000UL;
        /* Head-start hold: never key before the receiver's boundary
         * retune has finished (see SX1276_FHSS_SLOT_TX_HEADSTART_MS). */
        if (in_slot_ms < (uint32_t)SX1276_FHSS_SLOT_TX_HEADSTART_MS) {
            return ((uint32_t)SX1276_FHSS_SLOT_TX_HEADSTART_MS
                    - in_slot_ms) * 1000UL;
        }
        const uint8_t  effective_len =
            (uint8_t)((payload_len > (255U - LORA_PKT_HDR_LEN))
                          ? 255U
                          : (payload_len + LORA_PKT_HDR_LEN));
        const uint32_t toa_us =
            sx1276_airtime_estimate_toa_us(effective_len);
        if (in_slot_us + toa_us + SX1276_FHSS_SLOT_TX_GUARD_US <= slot_us) {
            return 0U;
        }
        /* Wait to the next boundary plus the head-start hold. */
        return (slot_us - in_slot_us)
               + (uint32_t)SX1276_FHSS_SLOT_TX_HEADSTART_MS * 1000UL;
    }
#else
    (void)payload_len;
    return 0U;
#endif
}