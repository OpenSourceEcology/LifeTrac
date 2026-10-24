/*
 * RS-12.21 / FHSS leg 6 (2b_f3, 2026-10-24): a node in the absorbing scan
 * FAILED state, system-in-the-loop -- CHARACTERISATION OF KNOWN DEFECTS:
 *   (1) a FOLLOWER (the base) whose cold-start scan ran out into FAILED
 *       never hops again (leg 6);
 *   (2) an ORIGINATOR (the tractor) in FAILED that adopts a grid it heard
 *       loses originator authority, and with it self-follow, for good.
 *
 * THESE CHECKS PIN THE CURRENT, DEFECTIVE BEHAVIOUR. A PASS does not mean
 * the behaviour is right: it means main still behaves as characterised
 * here. A FAIL means that behaviour changed: a firmware fix (the options,
 * their SIL results and the recommendation are in
 * firmware/murata_l072/FHSS_SCAN_FAILED_OPTIONS.md, sections 4-5; first
 * listed in bench-evidence/bench_session_2026-10-24/RESULTS.md, "Next
 * steps" 3), or an unrelated change -- to the scan constants, the walker,
 * the TX path's timing or the shared simulation (rx_reverse_delivery.c).
 * Read which check failed before replacing anything; the PR that lands a fix
 * replaces these assertions with the fix's own acceptance checks (`-e`
 * below prints the recovery measures to start from). They characterise
 * the FIRMWARE path only: no host-side recovery runs in them, so a
 * daemon-side watchdog (option (c)) never flips them.
 *
 * WHY (1). On leg 6 the base heard 5 of the tractor's 606 frames, on
 * channels consistent with a receiver parked on the scan walker's last
 * channel (ch 9) and then on each PROBE's TX channel (44, 6, 6, 18, 18);
 * base d(fhss_dec_aligned) 0, d(fhss_dec_snapped) 3, d(rej_epoch_drift) 2,
 * d(clk_demotion_reset) 0; post radio_state 4 (RX_CONT). Leading
 * hypothesis H1: the scan used its 4 x 30 s budget without a catch and
 * entered scan FAILED, which absorbs (sx1276_rx_scan_policy.c); there a
 * valid frame cannot LOCK, and the slot follower runs only when LOCKED on
 * an adopted grid or under originator authority, which a follower never
 * holds (sx1276_rx_grid_policy.c, sx1276_rx_slot_follow()). The base keeps
 * transmitting on hop channels (its PROBEs) while its receiver no longer
 * follows the hop sequence: the "synced receivers" requirement in the
 * 15.247(a)(1) row of the FCC notes (AI NOTES/2026-05-19_FCC_Part15_902-
 * 928_Compliance_Notes), so this is a compliance-relevant state as well as
 * a delivery defect.
 *
 * WHY (2). On 2026-10-18 legs W/X the tractor's scan also ran out into
 * FAILED, harmlessly: an originator follows its own clock in any scan
 * state. But an accepted header is adopted whatever the scan state
 * (sx1276_rx_service(): grid.adopt is 1 for a node without authority, with
 * UNANCHORED health while nothing is adopted), only a LOCKED -> SCANNING
 * demotion or a reset clears the adoption (sx1276_rx_grid_on_demotion()),
 * FAILED never LOCKs so it never demotes, and no node holds authority
 * while a grid is adopted (sx1276_fhss_authority_is_originator()). So one
 * camera stall (authority decays 1 s after the last own TX) and one base
 * command heard in it end self-follow for the rest of the session. The
 * RS-12.15 open question is therefore not only whether self-follow should
 * run in FAILED: adoption in FAILED is irreversible and strips authority.
 *
 * WHAT. Reuses the simulated SX1276, clock, air and main-loop driver of
 * check-rx-reverse-delivery (rx_reverse_delivery.c is #included with
 * RX_REVERSE_DELIVERY_SIM_ONLY, which leaves out its scenarios and main)
 * against the same REAL sx1276_rx.c / sx1276_tx.c / scan / grid / clock
 * TUs.
 * (1) The firmware is the BASE: a follower on profile 1. Added here:
 *   - originator: the tractor's 2 fps camera, one ~200 B fragment per frame
 *     (VECTOR p50 199 B), keyed 13.4 ms into the next slot of ITS grid --
 *     about 2 slots in 5 -- optionally on a clock running ppm fast;
 *   - rx daemon (image_rx_daemon.py, LIFETRAC_REACTIVE_FIRE=1): a PROBE
 *     after each frame heard (behind the shared 1.0 s send gate), and
 *     _ensure_rxcont -- a raw RegOpMode=RXCONT write, no retune -- after
 *     each own TX and every 5 s while idle.
 * (2) The firmware is the TRACTOR, with that harness's camera daemon and
 *   base model: scenario 8 (no command for 150 s, so the scan runs out into
 *   FAILED) with scenario 7's stream and command plane (2 fps, leg H train
 *   mix, aligned pump at train end behind the 1.0 s gate, idle drain), then
 *   300 s of commands with a 2.5 s camera stall every 9 s; and the same run
 *   without stalls as the control.
 *
 * (1) ASSUMED -- built into the scenario, not observed on leg 6:
 *   - no frame reaches the base for the whole scan budget: the stream is
 *     withheld until 1 s after the final FAIL (to the firmware the same as
 *     a stream whose frames never land on a dwelled channel). FAILED at
 *     +120 s and the fault sub-bytes 20 21 22 a3 follow from that by
 *     construction: they are H1's predictions, not leg-6 observations
 *     (leg 6 logged no base FAULT URC; image_rx_daemon.py drops them at
 *     DEBUG);
 *   - 0 ppm between the boards (E1 also runs 2750 ppm);
 *   - fixed 200 B frames keyed 13.4 ms into the slot, so a PROBE (~30 ms of
 *     daemon latency after the frame) always lands in the NEXT slot: the
 *     model cannot produce leg 6's same-slot PROBEs (PROBE 3 at (17,22),
 *     PROBE 5 at (24,43)). That needs variable frame sizes and clock drift
 *     in the originator model;
 *   - no model of the tractor's side of the link.
 * (1) ASSERTED -- checks 1-5, the mechanism, for any seed (-s):
 *   1  four RX_SCAN_FAILED faults, sub 0x20 0x21 0x22 0xA3, the final one
 *      120 s after activation; the modem is parked in STANDBY on the
 *      walker's last channel (dwell 60 = index 59 mod 50 = 9) and the
 *      daemon's raw RXCONT re-arms it there, without a retune;
 *   2  then valid frames: the scan never LOCKs and never demotes (FAILED
 *      absorbs); the first frame heard is on that channel, SNAPPED, and its
 *      grid adopted; neither the slot follower nor the walker ever retunes
 *      the receiver, and the shared scheduler moves only on an own TX or a
 *      SNAP;
 *   3  each reactive PROBE is keyed in the originator's slot it claims (the
 *      adopted grid), and the firmware's post-TX re-arm leaves the
 *      receiver in RX_CONT on the PROBE's channel: every later frame heard
 *      is on the channel of the PROBE before it;
 *   4  consider_remote() compares against a scheduler epoch frozen at the
 *      last own TX or SNAP: never ALIGNED; a catch within one epoch of it
 *      is SNAPPED, two or more is REJECTED_EPOCH_DRIFT, and
 *      REJECTED_LOCKED_OUT only while the adopted clock is at most
 *      SX1276_FHSS_CLOCK_FRESH_MS (2 s) old (anchored by the last SNAP or
 *      own TX);
 *   5  so the base hears a few percent of the stream, and the run ends in
 *      RX_CONT, still FAILED.
 *   Independently of any seed these match leg 6: the first catch on the
 *   walker's last channel, ch 9 (from the constants); every later catch on
 *   the previous PROBE's channel (the post-TX re-arm); 0 ALIGNED, 0
 *   demotions, ends in RX_CONT. Seeds 1-1000 pass checks 1-5 except 558
 *   (one catch in 180 s; checks 2-4 need two).
 * (1) SEED-FITTED -- a soft check, default seed 852 only, printed as a NOTE
 *   when it no longer holds and never failed: leg 6's first two catches
 *   slot for slot -- (13,17) on ch 9, PROBE (13,18) on ch 44, (14,40) on
 *   ch 44 -- and its counters: 5 heard, 5 PROBEs, 3 SNAPPED, 2 EPOCH_DRIFT,
 *   0 LOCKED_OUT. (The stream's grid starts at epoch 13 so its slots carry
 *   leg 6's channels.) Over seeds 1-1000 that split comes up in 35,
 *   exactly 5 catches in 139, and 1-14 catches in all.
 * (1) NOT REPRODUCED: leg 6's catches 3-5 -- SIL (16,29) (17,8) (25,15),
 *   leg 6 (17,22) (18,2) (24,43) -- and PROBE channels 3-5 (SIL 37, 49, 22;
 *   leg 6 6, 18, 18: its repeats are the same-slot PROBEs above); the first
 *   catch comes at +124.6 s, on leg 6 at +138 s.
 *
 * (2) ASSERTED -- default seed (scenario 7's); `-e` measures other seeds:
 *   T1 no command for 150 s: the scan runs out into FAILED (one final FAIL,
 *      no LOCK), nothing adopted, the tractor holds authority;
 *   T2 control, no stalls: authority on every pass from then on, no
 *      adoption, no LOCK, the own grid never renumbered, self-follow keeps
 *      running, and every clean command sent while it held authority is
 *      received (scenario 8's property);
 *   T3 with stalls: a command heard in a stall's decay window is adopted
 *      (UNANCHORED: any epoch accepted) and nothing undoes it: no LOCK, no
 *      demotion, still adopted and still FAILED at the end; the clock is
 *      never reset (tx_first_anchor 1) nor the own grid renumbered (a TX
 *      more than one slot off the grid of the TX before it);
 *   T4 from that adoption on: no authority on any pass, no self-follow, no
 *      LOCKED follow, and fewer than half of the commands received.
 *
 * `rx_scan_failed_follower -v` prints the runs (catches, PROBEs, counters).
 * `rx_scan_failed_follower -e` skips the characterisations and runs the
 * evaluation used to compare fix options (built against patched copies of
 * the firmware, SIL only); it prints and never fails (`-e3`: E3 only):
 *   E1  leg-6 replay, seeded trials: silent for the scan budget, the
 *       stream from 1 s after the final FAIL on a random grid phase: does
 *       the base LOCK, how long after the stream starts, and what share of
 *       the frames keyed from 120 s to 300 s after the start it hears;
 *       at 0 ppm and with the originator 2750 ppm fast (10-24: base L072
 *       610-1400 ppm slow vs its host, tractor grid ~+1600 ppm);
 *   E1w E1 with a host watchdog model, option (c): the rx daemon re-sends
 *       REG_PROFILE=1 (profile activation: sx1276_fhss_init +
 *       sx1276_rx_scan_reset) 200 ms after a final RX_SCAN_FAILED fault;
 *   E2  cold start, the stream already on at activation (random grid
 *       phase): P(no LOCK within the 120 s budget), the attempt that
 *       catches, the per-attempt catch probability, at 2 fps and 1 fps;
 *   E3  (2) over seeded runs: how many adopt a grid in the command phase,
 *       in how many authority never returns after the first adoption, the
 *       authority share of the phase's passes, commands received, LOCKs,
 *       demotions, phase restarts (tx_first_anchor > 1) and own-grid
 *       renumbering; with stalls, the control without, and main's
 *       behaviour before FAILED (commands from 5 s, so the scan LOCKs on
 *       them and never fails out: scenario 7's decay window);
 *   E3w E3 with option (c)'s watchdog on the TRACTOR too. Not proposed --
 *       (c) runs on the base only, which leaves E3 as it is on main -- but
 *       it shows why: a re-activation restarts the originator's own grid
 *       at epoch 0 under the base (renumbered by hundreds of slots).
 */

#define RX_REVERSE_DELIVERY_SIM_ONLY 1
/* Some of its helpers (base_cmd_at_key, run_for_ms) are not used here. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#include "rx_reverse_delivery.c"
#pragma GCC diagnostic pop

#include <stdlib.h>

/* ------------------------------------------------------------------ */
/* Originator (tractor) model: a camera stream on its own grid         */
/* ------------------------------------------------------------------ */

#define OS_MAX_FRAMES      4096U
#define OS_KEY_US          ORIG_KEY_US   /* 12 ms head-start + 1.4 ms   */
#define OS_LOOKAHEAD_US    2000ULL
#define OS_FRAME_LEN       200U          /* 208 B on air: ~164 ms        */

typedef struct {
    uint32_t abs;
    uint32_t hz;
    uint64_t key_us;
    uint8_t  heard;
} os_frame_t;

typedef struct {
    uint8_t  on;
    uint64_t t0_us;            /* boundary of slot abs0, base (sim) time */
    uint32_t abs0;
    int32_t  ppm;              /* originator clock fast by ppm vs the base */
    uint32_t next_n;           /* next slot (from abs0) to consider       */
    uint64_t next_frame_us;    /* camera: next frame ready                */
    uint32_t period_ms, jitter_ms;
    uint8_t  len;
    uint32_t sent;
    os_frame_t log[OS_MAX_FRAMES];
} ostream_t;
static ostream_t g_os;

static uint64_t os_boundary_us(uint32_t n) {
    return g_os.t0_us + ((uint64_t)n * SLOT_US * (uint64_t)(1000000 - g_os.ppm))
                        / 1000000ULL;
}

/* The originator's absolute slot at sim time t (t >= t0). */
static uint32_t os_abs_at(uint64_t t_us) {
    return g_os.abs0 + (uint32_t)(((t_us - g_os.t0_us) * 1000000ULL)
                                  / (SLOT_US * (uint64_t)(1000000 - g_os.ppm)));
}

static void os_start(uint64_t t0_us, uint32_t abs0, int32_t ppm,
                     uint32_t period_ms, uint8_t len) {
    g_os.on = 1U;
    g_os.t0_us = t0_us;
    g_os.abs0 = abs0;
    g_os.ppm = ppm;
    g_os.next_n = 0U;
    g_os.period_ms = period_ms;
    g_os.jitter_ms = 40U;
    g_os.len = len;
    g_os.next_frame_us = t0_us + (uint64_t)rnd_range(0U, period_ms) * 1000ULL;
}

/* Key every slot whose boundary is (about to be) due: a camera frame that
 * is ready by a boundary goes out 13.4 ms into that slot; extra frames
 * ready in the same slot are dropped (pipeline). */
static void os_tick(void) {
    if (g_os.on == 0U) {
        return;
    }
    for (;;) {
        const uint64_t b = os_boundary_us(g_os.next_n);
        if (b > g_now_us + OS_LOOKAHEAD_US) {
            break;
        }
        if (g_os.next_frame_us <= b) {
            if (b + OS_KEY_US > g_now_us && g_os.sent < OS_MAX_FRAMES) {
                uint8_t payload[OS_FRAME_LEN];
                const uint16_t seq = (uint16_t)g_os.sent;
                const uint32_t abs = g_os.abs0 + g_os.next_n;
                memset(payload, 0x5A, sizeof(payload));
                payload[0] = MARK_ORIG;
                payload[1] = (uint8_t)(seq & 0xFFU);
                payload[2] = (uint8_t)(seq >> 8);
                if (air_schedule(PKT_FRAME, seq, b + OS_KEY_US, abs,
                                 (uint8_t)(OS_KEY_US / 1000ULL), payload,
                                 g_os.len) != NULL) {
                    g_os.log[seq].abs = abs;
                    g_os.log[seq].hz = hop_hz(abs);
                    g_os.log[seq].key_us = b + OS_KEY_US;
                    g_os.log[seq].heard = 0U;
                    ++g_os.sent;
                    ++g_sc.frames_sent;
                }
            }
            while (g_os.next_frame_us <= b) {
                g_os.next_frame_us += (uint64_t)rnd_range(
                    g_os.period_ms - g_os.jitter_ms,
                    g_os.period_ms + g_os.jitter_ms) * 1000ULL;
            }
        }
        ++g_os.next_n;
    }
}

/* ------------------------------------------------------------------ */
/* rx daemon model (image_rx_daemon.py, reactive fire)                 */
/* ------------------------------------------------------------------ */

#define RXD_PROBE_LAT_US   30000ULL    /* frame complete -> TX_FRAME_REQ  */
#define RXD_TXDONE_LAT_US  3000ULL     /* wait_for_tx_done -> _ensure_rxcont */
#define RXD_IDLE_CHECK_US  5000000ULL  /* idle-loop _ensure_rxcont throttle */
#define RXD_GATE_US        1000000ULL  /* shared send gate while streaming  */
#define RXD_WATCHDOG_US    200000ULL   /* final FAULT -> REG_PROFILE=1 (c)  */

typedef struct {
    uint8_t  reactive;
    uint8_t  watchdog;          /* option (c) model */
    uint64_t next_idle_us;
    uint64_t post_tx_us;        /* 0 = none pending */
    uint64_t last_send_us;      /* 0 = never        */
    uint64_t reactivate_us;     /* 0 = none pending */
    uint64_t seen_final_us;
    uint32_t frames_seen, tx_seen;
    uint32_t probes;
    uint32_t arms_idle, arms_after_tx;
    uint32_t reactivations;
    uint64_t first_idle_arm_us; /* first raw arm (after the final FAIL) */
    uint32_t first_idle_arm_hz;
    uint8_t  first_idle_arm_tuner;
} rxd_t;
static rxd_t g_rxd;

static uint8_t rxd_ensure_rxcont(void) {
    if (g_radio.mode == SX1276_STATE_RX_CONT || sx1276_tx_busy()) {
        return 0U;
    }
    {
        const ctx_t saved = g_ctx;
        g_ctx = CTX_HOST;
        host_raw_rxcont();
        g_ctx = saved;
    }
    return 1U;
}

/* REG_PROFILE=1 re-sent: host_cfg_profile_activate()'s profile-1 branch as
 * scenario_begin() models it (scheduler init + scan, clock, authority and
 * adoption reset). */
static void rxd_reactivate(void) {
    const ctx_t saved = g_ctx;
    g_ctx = CTX_HOST;
    (void)sx1276_fhss_init(0ULL, 0ULL, 0U);
    sx1276_rx_scan_reset();
    g_scan_mirror = SX1276_RX_SCAN_STATE_BOOT;
    g_ctx = saved;
    ++g_rxd.reactivations;
}

static void rxd_tick(void) {
    if (g_sc.frames_recv != g_rxd.frames_seen) {
        g_rxd.frames_seen = g_sc.frames_recv;
        if (g_rxd.reactive != 0U && g_bh.burst_left == 0U &&
            (g_rxd.last_send_us == 0U ||
             g_now_us >= g_rxd.last_send_us + RXD_GATE_US)) {
            g_bh.burst_left = 1U;
            g_bh.burst_gap_ms = 0U;
            g_bh.burst_at_us = g_now_us + RXD_PROBE_LAT_US;
            g_rxd.last_send_us = g_bh.burst_at_us;
            ++g_rxd.probes;
        }
    }
    if (g_sc.fw_tx_ok != g_rxd.tx_seen) {
        g_rxd.tx_seen = g_sc.fw_tx_ok;
        g_rxd.post_tx_us = g_now_us + RXD_TXDONE_LAT_US;
        g_rxd.last_send_us = g_now_us;
    }
    if (g_rxd.post_tx_us != 0U && g_now_us >= g_rxd.post_tx_us) {
        g_rxd.post_tx_us = 0U;
        g_rxd.arms_after_tx += rxd_ensure_rxcont();
    }
    if (g_now_us >= g_rxd.next_idle_us) {
        g_rxd.next_idle_us += RXD_IDLE_CHECK_US;
        if (rxd_ensure_rxcont() != 0U) {
            ++g_rxd.arms_idle;
            if (g_rxd.first_idle_arm_us == 0U) {
                g_rxd.first_idle_arm_us = g_now_us;
                g_rxd.first_idle_arm_hz = g_radio.freq_hz;
                g_rxd.first_idle_arm_tuner = (uint8_t)g_tuner;
            }
        }
    }
    if (g_rxd.watchdog != 0U) {
        if (g_final_fail_us != 0U && g_final_fail_us != g_rxd.seen_final_us) {
            g_rxd.seen_final_us = g_final_fail_us;
            g_rxd.reactivate_us = g_now_us + RXD_WATCHDOG_US;
        }
        if (g_rxd.reactivate_us != 0U && g_now_us >= g_rxd.reactivate_us &&
            !sx1276_tx_busy()) {
            g_rxd.reactivate_us = 0U;
            rxd_reactivate();
        }
    }
}

/* ------------------------------------------------------------------ */
/* One run: activation, scan budget, stream                            */
/* ------------------------------------------------------------------ */

#define L6_MAX_CATCH 256U
#define L6_DEC_DIM   SX1276_RX_CONSIDER_REMOTE_COUNT_DIM

typedef struct {
    uint64_t t_us;
    uint32_t abs;
    uint32_t hz;
    uint32_t local_epoch;      /* scheduler epoch before the frame        */
    uint32_t parked_hz;        /* where the receiver had been left        */
    uint8_t  dec;
    uint8_t  tuner;            /* who had tuned the synth for it          */
    uint8_t  adopted_before;
    uint8_t  fresh_before;     /* adopted clock <= FRESH_MS old before it */
} l6_catch_t;

typedef struct {
    uint32_t seed;
    int32_t  ppm;
    uint32_t period_ms;
    uint8_t  stream_at_activation;   /* E2: stream on from activation */
    uint32_t start_abs;              /* E1: 0 = random                */
    uint32_t run_after_start_ms;     /* E1 */
    uint32_t cap_ms;                 /* E2: give up after this        */
    uint8_t  watchdog;
    uint8_t  stop_at_lock;
} l6_cfg_t;

typedef struct {
    uint32_t act_ms;
    uint64_t act_us;
    uint64_t start_us;          /* stream start (0 = not yet)              */
    uint64_t final_pass_us;     /* pass that saw the final FAIL            */
    uint8_t  final_mode;        /* modem state right after it              */
    uint32_t final_hz;
    uint8_t  final_tuner;
    uint64_t lock_us;           /* first LOCK (0 = none)                   */
    uint32_t fails_before_lock;
    uint64_t follow_us;         /* first slot-follow retune after the start */
    uint32_t parked_hz;         /* receiver channel left by walker / own TX */
    uint32_t ncatch;
    l6_catch_t catches[L6_MAX_CATCH];
    uint32_t dec[L6_DEC_DIM];
    uint32_t tx_done, tx_rearm_on_tx_ch, tx_on_grid;
    uint32_t tx_hz[L6_MAX_CATCH];
    uint32_t tx_abs[L6_MAX_CATCH];
    uint32_t sched_unexplained;
    uint32_t walker_at_start, follow_at_start, locks_at_start, dem_at_start;
} l6_t;
static l6_t g_l6;

static uint8_t chan_of_hz(uint32_t hz) {
    for (uint8_t i = 0U; i < SX1276_FHSS_CHANNEL_COUNT; ++i) {
        if (sx1276_fhss_chantab_center_hz(i) == hz) {
            return i;
        }
    }
    return 0xFFU;
}

static void l6_on_catch(uint32_t local_epoch, uint8_t adopted_before,
                        uint8_t fresh_before) {
    const uint16_t seq = (uint16_t)(g_frame.payload[1] |
                                    ((uint16_t)g_frame.payload[2] << 8));
    if (seq >= g_os.sent) {
        CHECK(0, "[%s] heard an unknown frame (seq %u)", g_sc.name, seq);
        return;
    }
    g_os.log[seq].heard = 1U;
    if (g_last_dec < L6_DEC_DIM) {
        ++g_l6.dec[g_last_dec];
    }
    if (g_l6.ncatch < L6_MAX_CATCH) {
        l6_catch_t *k = &g_l6.catches[g_l6.ncatch];
        k->t_us = g_now_us;
        k->abs = g_os.log[seq].abs;
        k->hz = g_os.log[seq].hz;
        k->local_epoch = local_epoch;
        k->parked_hz = g_l6.parked_hz;
        k->dec = g_last_dec;
        k->tuner = (uint8_t)g_heard_tuner;
        k->adopted_before = adopted_before;
        k->fresh_before = fresh_before;
    }
    ++g_l6.ncatch;
}

static void l6_on_tx_done(void) {
    lora_pkt_hdr_t h;
    if (lora_pkt_hdr_unpack(g_radio.tx_bytes, g_radio.tx_len, &h) !=
        LORA_PKT_HDR_OK) {
        return;
    }
    {
        const uint32_t abs = sx1276_fhss_clock_abs_of(h.epoch, h.hop_idx);
        const uint32_t hz = hop_hz(abs);
        if (g_l6.tx_done < L6_MAX_CATCH) {
            g_l6.tx_hz[g_l6.tx_done] = hz;
            g_l6.tx_abs[g_l6.tx_done] = abs;
        }
        ++g_l6.tx_done;
        /* The firmware's own post-TX re-arm (sx1276_tx_cleanup): RX_CONT,
         * on the channel it just transmitted on. */
        if (g_radio.mode == SX1276_STATE_RX_CONT && g_radio.freq_hz == hz) {
            ++g_l6.tx_rearm_on_tx_ch;
        }
        /* Keyed in the originator's slot its header claims. */
        if (g_os.on != 0U && g_radio.tx_start_us >= g_os.t0_us &&
            os_abs_at(g_radio.tx_start_us) == abs) {
            ++g_l6.tx_on_grid;
        }
    }
}

static void l6_pass(void) {
    const uint32_t ep0 = sx1276_fhss_current_epoch();
    const uint8_t  sl0 = sx1276_fhss_current_slot();
    const uint32_t frames0 = g_sc.frames_recv;
    const uint32_t txok0 = g_sc.fw_tx_ok;
    const uint64_t txstart0 = g_radio.tx_start_us;
    const uint8_t  adopted0 = sx1276_rx_grid_adopted();
    /* The grid policy hands consider_remote FRESH (-> LOCKED_OUT) only for
     * an adopted, valid clock at most FRESH_MS old at the frame; its age
     * then is at least the age here (a REJECTED frame does not re-anchor;
     * platform_now_ms() does not advance the simulated clock). */
    const uint8_t  fresh0 = (uint8_t)(adopted0 != 0U &&
        sx1276_fhss_clock_valid() != 0U &&
        sx1276_fhss_clock_age_ms(platform_now_ms()) <= SX1276_FHSS_CLOCK_FRESH_MS);
    const uint32_t locks0 = g_sc.locks;
    const uint32_t follow0 = g_sc.follow_locked + g_sc.follow_own;
    uint8_t snapped_now = 0U;

    os_tick();
    fw_pass();

    if (g_sc.frames_recv != frames0) {
        l6_on_catch(ep0, adopted0, fresh0);
        snapped_now = (uint8_t)(g_last_dec == SX1276_FHSS_SNAP_DEC_SNAPPED);
    }
    if (g_sc.fw_tx_ok != txok0) {
        l6_on_tx_done();
    }
    /* Who left the receiver where: the walker, or an own TX (a slot-follow
     * retune is tracked by its own counter). */
    if (g_tuner == CTX_SCAN || g_tuner == CTX_HOST || g_tuner == CTX_MAILBOX) {
        g_l6.parked_hz = g_last_freq;
    }
    if ((sx1276_fhss_current_epoch() != ep0 || sx1276_fhss_current_slot() != sl0) &&
        snapped_now == 0U && g_radio.tx_start_us == txstart0) {
        ++g_l6.sched_unexplained;
    }
    if (g_sc.locks != locks0 && g_l6.lock_us == 0U) {
        g_l6.lock_us = g_now_us;
        g_l6.fails_before_lock = g_sc.scan_fail_urc;
    }
    if (g_l6.start_us != 0U && g_l6.follow_us == 0U &&
        g_sc.follow_locked + g_sc.follow_own != follow0) {
        g_l6.follow_us = g_now_us;
    }
    if (g_final_fail_us != 0U && g_l6.final_pass_us == 0U) {
        g_l6.final_pass_us = g_now_us;
        g_l6.final_mode = (uint8_t)g_radio.mode;
        g_l6.final_hz = g_radio.freq_hz;
        g_l6.final_tuner = (uint8_t)g_tuner;
    }
    rxd_tick();
}

static void l6_start_stream(const l6_cfg_t *c, uint64_t t_us) {
    const uint32_t abs0 = (c->start_abs != 0U)
        ? c->start_abs
        : rnd_range(0U, 999U) * SX1276_FHSS_CHANNEL_COUNT
          + rnd_range(0U, SX1276_FHSS_CHANNEL_COUNT - 1U);
    os_start(t_us + (uint64_t)rnd_range(0U, (uint32_t)SLOT_US - 1U), abs0,
             c->ppm, c->period_ms, (uint8_t)OS_FRAME_LEN);
    g_l6.start_us = g_now_us;
    g_l6.walker_at_start = g_sc.walker_retunes;
    g_l6.follow_at_start = g_sc.follow_locked + g_sc.follow_own;
    g_l6.locks_at_start = g_sc.locks;
    g_l6.dem_at_start = g_sc.demotions;
}

static void l6_run(const char *name, const l6_cfg_t *c) {
    scenario_begin(name, ROLE_BASE, c->seed);
    memset(&g_os, 0, sizeof(g_os));
    memset(&g_rxd, 0, sizeof(g_rxd));
    memset(&g_l6, 0, sizeof(g_l6));
    g_rxd.reactive = 1U;
    g_rxd.watchdog = c->watchdog;
    g_rxd.next_idle_us = g_now_us + (uint64_t)rnd_range(0U, 4999U) * 1000ULL;
    g_l6.act_us = g_now_us;
    g_l6.act_ms = platform_now_ms();
    if (c->stream_at_activation != 0U) {
        l6_start_stream(c, g_now_us);
        while (g_now_us < g_l6.act_us + (uint64_t)c->cap_ms * 1000ULL &&
               !(c->stop_at_lock != 0U && g_l6.lock_us != 0U)) {
            l6_pass();
        }
        return;
    }
    /* Silent for the scan budget; the stream starts 1 s after the final
     * FAIL (or at +121 s if a variant never reports one). */
    while (g_l6.start_us == 0U) {
        l6_pass();
        if ((g_final_fail_us != 0U && g_now_us >= g_final_fail_us + 1000000ULL) ||
            g_now_us >= g_l6.act_us + 121000000ULL) {
            l6_start_stream(c, g_now_us);
        }
    }
    while (g_now_us < g_l6.start_us + (uint64_t)c->run_after_start_ms * 1000ULL &&
           !(c->stop_at_lock != 0U && g_l6.lock_us != 0U)) {
        l6_pass();
    }
}

/* ------------------------------------------------------------------ */
/* The characterisation (leg 6 on main)                                */
/* ------------------------------------------------------------------ */

static const char *dec_name(uint8_t d) {
    switch (d) {
        case SX1276_FHSS_SNAP_DEC_ALIGNED: return "ALIGNED";
        case SX1276_FHSS_SNAP_DEC_SNAPPED: return "SNAPPED";
        case SX1276_FHSS_SNAP_DEC_REJECTED_EPOCH_DRIFT: return "EPOCH_DRIFT";
        case SX1276_FHSS_SNAP_DEC_REJECTED_LOCKED_OUT: return "LOCKED_OUT";
        default: return "other";
    }
}

/* The default seed's camera timing reproduces leg 6's catches 1 and 2 slot
 * for slot -- (13,17) on ch 9, PROBE (13,18) on ch 44, (14,40) on ch 44 --
 * and its counters: 5 heard, 5 PROBEs, 3 SNAPPED, 2 EPOCH_DRIFT, 0 ALIGNED,
 * 0 LOCKED_OUT. A soft check (a NOTE, never a failure): a harness change
 * that moves the random sequence needs a new default (scan seeds with
 * -s -v). `-s <seed>` runs another seed against checks 1-5 only. */
#define L6_LEG6_SEED 852U
static uint32_t g_l6_seed = L6_LEG6_SEED;

/* "<n> us after <ref>", or "none" when t is unset or before ref. */
static const char *us_after(char *buf, size_t n, uint64_t t_us, uint64_t ref_us) {
    if (t_us == 0U || t_us < ref_us) {
        snprintf(buf, n, "none");
    } else {
        snprintf(buf, n, "%llu us", (unsigned long long)(t_us - ref_us));
    }
    return buf;
}

static void test_leg6_characterisation(void) {
    /* Leg 6's first catch was (epoch 13, hop 17) on ch 9; start the grid
     * at epoch 13 so the same slots carry the same channels. */
    const l6_cfg_t c = {
        .seed = g_l6_seed, .ppm = 0, .period_ms = 500U,
        .stream_at_activation = 0U,
        .start_abs = 13U * SX1276_FHSS_CHANNEL_COUNT,
        .run_after_start_ms = 180000U, .cap_ms = 0U,
        .watchdog = 0U, .stop_at_lock = 0U,
    };
    /* With a dwell restart at channel 0 on every attempt, the walker's last
     * channel is dwell (REDESIGN / DWELL) - 1, mod 50: index 9 on main. */
    const uint8_t walker_last = (uint8_t)(((SX1276_RX_SCAN_REDESIGN_MS
                                            / SX1276_RX_SCAN_DWELL_MS) - 1U)
                                          % SX1276_FHSS_CHANNEL_COUNT);
    uint32_t heard = 0U, n_snap_near = 0U, n_drift_far = 0U, n_locked_out = 0U;
    uint32_t bad_dec = 0U, bad_park = 0U, bad_tuner = 0U;
    char arm_buf[32];

    l6_run("leg 6: follower in scan FAILED", &c);
    /* A PROBE may be on air when the run ends: let it finish (its post-TX
     * re-arm is part of the mechanism checked below). */
    for (int i = 0; i < 1000 && sx1276_tx_busy(); ++i) {
        l6_pass();
    }
    heard = g_l6.ncatch;   /* every frame heard, past L6_MAX_CATCH too */
    print_scenario();
    check_structure();

    /* 1  the scan budget runs out into FAILED and the modem is parked. */
    CHECK(g_sc.scan_fail_urc == 4U && g_sc.fail_final == 1U &&
          g_fail_sub[0] == 0x20U && g_fail_sub[1] == 0x21U &&
          g_fail_sub[2] == 0x22U && g_fail_sub[3] == 0xA3U,
          "[%s] scan faults: %u (final %u), sub %02x %02x %02x %02x"
          " (expect 4: 20 21 22 a3)", g_sc.name, g_sc.scan_fail_urc,
          g_sc.fail_final, g_fail_sub[0], g_fail_sub[1], g_fail_sub[2],
          g_fail_sub[3]);
    CHECK((uint32_t)(g_fail_ms[3] - g_l6.act_ms) >= 120000U &&
          (uint32_t)(g_fail_ms[3] - g_l6.act_ms) < 120100U,
          "[%s] final FAIL %u ms after activation (expect 120 s)", g_sc.name,
          (unsigned)(g_fail_ms[3] - g_l6.act_ms));
    CHECK(g_l6.final_mode == (uint8_t)SX1276_STATE_STANDBY &&
          g_l6.final_tuner == (uint8_t)CTX_SCAN &&
          chan_of_hz(g_l6.final_hz) == walker_last,
          "[%s] after the final FAIL: mode %u (expect STANDBY %u), on ch %u"
          " (expect the walker's last, %u), last tuned by ctx %u",
          g_sc.name, g_l6.final_mode, (unsigned)SX1276_STATE_STANDBY,
          chan_of_hz(g_l6.final_hz), walker_last, g_l6.final_tuner);
    CHECK(g_rxd.first_idle_arm_us > g_l6.final_pass_us &&
          g_rxd.first_idle_arm_us <= g_l6.final_pass_us + RXD_IDLE_CHECK_US &&
          g_rxd.first_idle_arm_hz == g_l6.final_hz &&
          g_rxd.first_idle_arm_tuner == (uint8_t)CTX_SCAN,
          "[%s] the daemon's raw RXCONT did not re-arm the parked receiver in"
          " place (first raw arm: %s after the final FAIL, ch %u, tuner %u)",
          g_sc.name,
          us_after(arm_buf, sizeof(arm_buf), g_rxd.first_idle_arm_us,
                   g_l6.final_pass_us),
          chan_of_hz(g_rxd.first_idle_arm_hz), g_rxd.first_idle_arm_tuner);

    /* 2  no LOCK, no demotion, no follow, no walk; scheduler frozen. */
    CHECK(g_sc.locks == 0U && g_sc.demotions == 0U && g_sc.dem_reset == 0U &&
          g_scan_mirror == SX1276_RX_SCAN_STATE_FAILED,
          "[%s] FAILED did not absorb: locks=%u demotions=%u clk_demotion_reset=%u"
          " scan state %u", g_sc.name, g_sc.locks, g_sc.demotions,
          g_sc.dem_reset, (unsigned)g_scan_mirror);
    CHECK(g_sc.walker_retunes == g_l6.walker_at_start &&
          g_sc.follow_locked + g_sc.follow_own == g_l6.follow_at_start &&
          g_sc.follow_locked + g_sc.follow_own == 0U,
          "[%s] the receiver was retuned by the walker (%u) or the slot follower"
          " (%u) once the stream ran", g_sc.name,
          g_sc.walker_retunes - g_l6.walker_at_start,
          g_sc.follow_locked + g_sc.follow_own);
    CHECK(g_l6.sched_unexplained == 0U,
          "[%s] the shared scheduler moved %u times without an own TX or a SNAP",
          g_sc.name, g_l6.sched_unexplained);
    CHECK(g_l6.ncatch >= 2U && sx1276_rx_grid_adopted() != 0U &&
          g_sc.adoptions >= 1U && g_sc.adopt_via_walker == 1U,
          "[%s] catches=%u adopted=%u adoptions=%u (via walker %u)", g_sc.name,
          g_l6.ncatch, sx1276_rx_grid_adopted(), g_sc.adoptions,
          g_sc.adopt_via_walker);

    for (uint32_t i = 0U; i < g_l6.ncatch && i < L6_MAX_CATCH; ++i) {
        const l6_catch_t *k = &g_l6.catches[i];
        const uint32_t ep = sx1276_fhss_clock_epoch_of(k->abs);
        const int32_t d = (int32_t)(ep - k->local_epoch);
        const uint32_t ad = (uint32_t)((d < 0) ? -d : d);
        if (k->hz != k->parked_hz) {
            ++bad_park;
        }
        /* first: on the walker's channel; later: on the last PROBE's. */
        if (i == 0U) {
            if (k->tuner != (uint8_t)CTX_SCAN || chan_of_hz(k->hz) != walker_last ||
                k->dec != SX1276_FHSS_SNAP_DEC_SNAPPED || k->adopted_before != 0U) {
                ++bad_tuner;
            }
            continue;
        }
        if (k->tuner != (uint8_t)CTX_HOST && k->tuner != (uint8_t)CTX_MAILBOX) {
            ++bad_tuner;
        }
        if (k->dec == SX1276_FHSS_SNAP_DEC_SNAPPED && ad <= 1U) {
            ++n_snap_near;
        } else if (k->dec == SX1276_FHSS_SNAP_DEC_REJECTED_EPOCH_DRIFT && ad >= 2U) {
            ++n_drift_far;
        } else if (k->dec == SX1276_FHSS_SNAP_DEC_REJECTED_LOCKED_OUT &&
                   k->fresh_before != 0U) {
            ++n_locked_out;
        } else {
            ++bad_dec;
        }
    }
    CHECK(bad_tuner == 0U && bad_park == 0U,
          "[%s] %u catches not where the walker / the last PROBE left the"
          " receiver, %u off the parked channel", g_sc.name, bad_tuner, bad_park);

    /* 3  PROBEs keyed on the adopted grid; post-TX re-arm on their channel. */
    CHECK(g_l6.tx_done >= 2U && g_l6.tx_rearm_on_tx_ch == g_l6.tx_done &&
          g_l6.tx_on_grid == g_l6.tx_done && g_rxd.arms_after_tx == 0U,
          "[%s] PROBEs=%u: re-armed on their channel %u, on the originator's slot"
          " %u, re-armed by the daemon after TX %u", g_sc.name, g_l6.tx_done,
          g_l6.tx_rearm_on_tx_ch, g_l6.tx_on_grid, g_rxd.arms_after_tx);

    /* 4  frozen scheduler epoch: never ALIGNED; SNAPPED near, DRIFT far,
     *    LOCKED_OUT only on a clock at most FRESH_MS old. */
    CHECK(g_l6.dec[SX1276_FHSS_SNAP_DEC_ALIGNED] == 0U && bad_dec == 0U &&
          n_snap_near + n_drift_far >= 1U,
          "[%s] consider_remote: aligned=%u snapped=%u (%u within one epoch)"
          " epoch_drift=%u (%u two or more) locked_out=%u (%u on a fresh clock),"
          " %u unexplained", g_sc.name, g_l6.dec[SX1276_FHSS_SNAP_DEC_ALIGNED],
          g_l6.dec[SX1276_FHSS_SNAP_DEC_SNAPPED], n_snap_near,
          g_l6.dec[SX1276_FHSS_SNAP_DEC_REJECTED_EPOCH_DRIFT], n_drift_far,
          g_l6.dec[SX1276_FHSS_SNAP_DEC_REJECTED_LOCKED_OUT], n_locked_out,
          bad_dec);

    /* 5  a few percent heard; ends in RX_CONT, still FAILED. */
    CHECK(heard * 10U < g_os.sent && g_radio.mode == SX1276_STATE_RX_CONT,
          "[%s] heard %u of %u frames, final mode %u (expect < 10 %%, RX_CONT)",
          g_sc.name, heard, g_os.sent, (unsigned)g_radio.mode);

    /* Leg 6's fingerprint, for the default seed: a soft check. */
    if (g_l6_seed == L6_LEG6_SEED) {
        const uint32_t leg6_abs1 = 13U * SX1276_FHSS_CHANNEL_COUNT + 17U;
        const uint32_t leg6_abs2 = 14U * SX1276_FHSS_CHANNEL_COUNT + 40U;
        const uint8_t match = (uint8_t)(
              g_l6.ncatch == 5U && g_l6.tx_done == 5U &&
              g_l6.dec[SX1276_FHSS_SNAP_DEC_SNAPPED] == 3U &&
              g_l6.dec[SX1276_FHSS_SNAP_DEC_REJECTED_EPOCH_DRIFT] == 2U &&
              g_l6.dec[SX1276_FHSS_SNAP_DEC_REJECTED_LOCKED_OUT] == 0U &&
              g_l6.catches[0].abs == leg6_abs1 && chan_of_hz(g_l6.catches[0].hz) == 9U &&
              g_l6.tx_abs[0] == leg6_abs1 + 1U && chan_of_hz(g_l6.tx_hz[0]) == 44U &&
              g_l6.catches[1].abs == leg6_abs2 && chan_of_hz(g_l6.catches[1].hz) == 44U);
        if (match == 0U || g_verbose != 0) {
            printf("    %s leg-6 fingerprint (seed %u, soft check, never a"
                   " failure): heard %u PROBEs %u snapped %u epoch_drift %u"
                   " locked_out %u (leg 6: 5 5 3 2 0); catch 1 (%u,%u) ch %u,"
                   " PROBE 1 ch %u, catch 2 (%u,%u) ch %u (leg 6: (13,17) ch 9,"
                   " ch 44, (14,40) ch 44)\n",
                   (match != 0U) ? "[MATCH]" : "[NOTE] no", (unsigned)g_l6_seed,
                   g_l6.ncatch, g_l6.tx_done,
                   g_l6.dec[SX1276_FHSS_SNAP_DEC_SNAPPED],
                   g_l6.dec[SX1276_FHSS_SNAP_DEC_REJECTED_EPOCH_DRIFT],
                   g_l6.dec[SX1276_FHSS_SNAP_DEC_REJECTED_LOCKED_OUT],
                   sx1276_fhss_clock_epoch_of(g_l6.catches[0].abs),
                   (unsigned)sx1276_fhss_clock_hop_of(g_l6.catches[0].abs),
                   chan_of_hz(g_l6.catches[0].hz), chan_of_hz(g_l6.tx_hz[0]),
                   sx1276_fhss_clock_epoch_of(g_l6.catches[1].abs),
                   (unsigned)sx1276_fhss_clock_hop_of(g_l6.catches[1].abs),
                   chan_of_hz(g_l6.catches[1].hz));
        }
    }

    if (g_verbose != 0) {
        printf("    activation -> final FAIL %u ms; parked on ch %u (%s), daemon"
               " re-armed it: %s later; stream from +%llu ms, %u frames sent,"
               " %u heard (%.1f%%)\n",
               (unsigned)(g_fail_ms[3] - g_l6.act_ms), chan_of_hz(g_l6.final_hz),
               (g_l6.final_mode == (uint8_t)SX1276_STATE_STANDBY) ? "STANDBY" : "?",
               us_after(arm_buf, sizeof(arm_buf), g_rxd.first_idle_arm_us,
                        g_l6.final_pass_us),
               (unsigned long long)((g_l6.start_us - g_l6.act_us) / 1000ULL),
               g_os.sent, heard, 100.0 * heard / (double)g_os.sent);
        for (uint32_t i = 0U; i < g_l6.ncatch && i < L6_MAX_CATCH; ++i) {
            const l6_catch_t *k = &g_l6.catches[i];
            printf("    catch %u at +%.2f s: (%u,%u) ch %u, scheduler epoch %u ->"
                   " %s; then PROBE (%u,%u) ch %u\n", i + 1U,
                   (double)(k->t_us - g_l6.act_us) / 1e6,
                   sx1276_fhss_clock_epoch_of(k->abs),
                   (unsigned)sx1276_fhss_clock_hop_of(k->abs), chan_of_hz(k->hz),
                   k->local_epoch, dec_name(k->dec),
                   (i < g_l6.tx_done) ? sx1276_fhss_clock_epoch_of(g_l6.tx_abs[i]) : 0U,
                   (i < g_l6.tx_done) ? (unsigned)sx1276_fhss_clock_hop_of(g_l6.tx_abs[i]) : 0U,
                   (i < g_l6.tx_done) ? chan_of_hz(g_l6.tx_hz[i]) : 0U);
        }
        printf("    daemon: probes=%u raw arms idle=%u after-TX=%u\n",
               g_rxd.probes, g_rxd.arms_idle, g_rxd.arms_after_tx);
    }
}

/* ------------------------------------------------------------------ */
/* Tractor role: the originator in scan FAILED                         */
/* ------------------------------------------------------------------ */

#define TR_SILENT_MS  150000U   /* no command: the scan runs out (scen. 8) */
#define TR_PHASE_MS   300000U   /* then commands, with or without stalls   */
#define TR_SEED       0x789ABCDU   /* scenario 7's */

typedef struct {
    uint32_t seed;
    uint8_t  stalls;            /* a 2.5 s camera stall every 9 s (scen. 7) */
    uint8_t  watchdog;          /* (c)'s watchdog on the tractor (E3w only) */
    uint32_t silent_ms;         /* no command for this long (0: TR_SILENT_MS) */
} tr_cfg_t;

typedef struct {
    /* at the end of the silent phase */
    uint8_t  failed0, auth0;
    uint32_t adoptions0, locks0, fail_final0;
    uint64_t phase_us;
    uint32_t sent0, recv0, passes0, auth_passes0, follow_own0;
    /* first adoption in the command phase (adopt_us 0 = none) */
    uint64_t adopt_us;
    uint32_t sent_at_adopt, recv_at_adopt, passes_at_adopt, auth_at_adopt;
    uint32_t follow_own_at_adopt;
    /* the tractor-side watchdog model */
    uint64_t seen_final_us, reactivate_us;
    /* own grid renumbered: a TX keyed more than one slot off the grid of
     * the TX before it (a phase restart moves it by less than a slot) */
    uint8_t  have_tx;
    uint32_t last_tx_abs;
    uint64_t last_tx_us;
    uint32_t renumbers, renumber_max;
} tr_t;
static tr_t g_tr;

static void tr_on_tx_done(void) {
    lora_pkt_hdr_t h;
    uint32_t abs;
    if (lora_pkt_hdr_unpack(g_radio.tx_bytes, g_radio.tx_len, &h) !=
        LORA_PKT_HDR_OK) {
        return;
    }
    abs = sx1276_fhss_clock_abs_of(h.epoch, h.hop_idx);
    if (g_tr.have_tx != 0U) {
        const uint32_t expect = g_tr.last_tx_abs + (uint32_t)(
            (g_radio.tx_start_us - g_tr.last_tx_us + SLOT_US / 2ULL) / SLOT_US);
        const int32_t d = (int32_t)(abs - expect);
        const uint32_t ad = (uint32_t)((d < 0) ? -d : d);
        if (ad > 1U) {
            ++g_tr.renumbers;
            if (ad > g_tr.renumber_max) {
                g_tr.renumber_max = ad;
            }
        }
    }
    g_tr.have_tx = 1U;
    g_tr.last_tx_abs = abs;
    g_tr.last_tx_us = g_radio.tx_start_us;
}

static void tr_pass(uint8_t watchdog) {
    const uint32_t txok0 = g_sc.fw_tx_ok;
    fw_pass();
    if (g_sc.fw_tx_ok != txok0) {
        tr_on_tx_done();
    }
    if (watchdog != 0U) {
        if (g_final_fail_us != 0U && g_final_fail_us != g_tr.seen_final_us) {
            g_tr.seen_final_us = g_final_fail_us;
            g_tr.reactivate_us = g_now_us + RXD_WATCHDOG_US;
        }
        if (g_tr.reactivate_us != 0U && g_now_us >= g_tr.reactivate_us &&
            !sx1276_tx_busy()) {
            g_tr.reactivate_us = 0U;
            rxd_reactivate();
        }
    }
}

static void tr_run(const char *name, const tr_cfg_t *c) {
    static const uint8_t trains[4] = { 23U, 45U, 23U, 9U };   /* leg H mix */
    const uint32_t silent_ms = (c->silent_ms != 0U) ? c->silent_ms : TR_SILENT_MS;
    uint64_t until;
    scenario_begin(name, ROLE_TRACTOR, c->seed);
    memset(&g_os, 0, sizeof(g_os));
    memset(&g_rxd, 0, sizeof(g_rxd));
    memset(&g_tr, 0, sizeof(g_tr));
    g_th.period_ms = 500U;
    g_th.jitter_ms = 40U;
    memcpy(g_th.train_w, trains, 4U);
    g_th.next_frame_us = g_now_us + 50000ULL;
    g_th.next_rxcont_us = g_now_us;
    base_model_init();
    g_base.mode = CMD_PUMP_AT_TRAIN_END;
    g_base.min_gap_ms = 1000U;
    g_base.idle_drain = 1U;
    /* Scenario 8: no command until the scan has run out. */
    g_base.next_allowed_us = g_now_us + (uint64_t)silent_ms * 1000ULL;
    until = g_now_us + (uint64_t)silent_ms * 1000ULL;
    while (g_now_us < until) {
        tr_pass(c->watchdog);
    }
    g_tr.failed0 = (uint8_t)(g_scan_mirror == SX1276_RX_SCAN_STATE_FAILED);
    g_tr.auth0 = auth_now_ms(platform_now_ms());
    g_tr.adoptions0 = g_sc.adoptions;
    g_tr.locks0 = g_sc.locks;
    g_tr.fail_final0 = g_sc.fail_final;
    g_tr.phase_us = g_now_us;
    g_tr.sent0 = g_sc.sent;
    g_tr.recv0 = g_sc.recv;
    g_tr.passes0 = g_sc.passes;
    g_tr.auth_passes0 = g_sc.auth_passes;
    g_tr.follow_own0 = g_sc.follow_own;
    if (c->stalls != 0U) {
        g_th.stall_every_ms = 9000U;
        g_th.stall_len_ms = 2500U;
        g_th.stall_from_us = g_now_us;
        g_th.stall_to_us = g_now_us;
    }
    until = g_now_us + (uint64_t)TR_PHASE_MS * 1000ULL;
    while (g_now_us < until) {
        tr_pass(c->watchdog);
        if (g_tr.adopt_us == 0U && g_sc.adoptions != g_tr.adoptions0) {
            g_tr.adopt_us = g_now_us;
            g_tr.sent_at_adopt = g_sc.sent;
            g_tr.recv_at_adopt = g_sc.recv;
            g_tr.passes_at_adopt = g_sc.passes;
            g_tr.auth_at_adopt = g_sc.auth_passes;
            g_tr.follow_own_at_adopt = g_sc.follow_own;
        }
    }
}

static void check_tr_silent_phase(void) {
    /* T1 */
    CHECK(g_tr.failed0 != 0U && g_tr.fail_final0 == 1U && g_tr.locks0 == 0U &&
          g_tr.adoptions0 == 0U && g_tr.auth0 != 0U,
          "[%s] after %u s without a command: scan FAILED %u (final FAILs %u),"
          " locks %u, adoptions %u, authority %u (expect 1 1 0 0 1)", g_sc.name,
          TR_SILENT_MS / 1000U, g_tr.failed0, g_tr.fail_final0, g_tr.locks0,
          g_tr.adoptions0, g_tr.auth0);
}

static void test_tractor_failed_adoption(void) {
    tr_cfg_t c = { .seed = TR_SEED, .stalls = 0U, .watchdog = 0U };

    /* T1 + T2: the control. */
    tr_run("tractor in scan FAILED, no stalls (control)", &c);
    print_scenario();
    check_structure();
    check_tr_silent_phase();
    CHECK(g_sc.adoptions == 0U && g_sc.locks == 0U && g_tr.renumbers == 0U &&
          g_sc.auth_passes - g_tr.auth_passes0 == g_sc.passes - g_tr.passes0 &&
          g_sc.follow_own > g_tr.follow_own0 && g_sc.sent - g_tr.sent0 >= 100U &&
          g_sc.covered_clean_recv == g_sc.covered_clean,
          "[%s] control: adoptions %u locks %u, own grid renumbered %u times,"
          " authority on %u of %u passes, self-follow retunes %u, cmds sent"
          " %u, clean while covered %u of which received %u (expect 0 0 0,"
          " all, > 0, >= 100, all)", g_sc.name,
          g_sc.adoptions, g_sc.locks, g_tr.renumbers,
          g_sc.auth_passes - g_tr.auth_passes0,
          g_sc.passes - g_tr.passes0, g_sc.follow_own - g_tr.follow_own0,
          g_sc.sent - g_tr.sent0, g_sc.covered_clean, g_sc.covered_clean_recv);
    if (g_verbose != 0) {
        printf("    phase: cmds sent=%u recv=%u, authority on %u/%u passes\n",
               g_sc.sent - g_tr.sent0, g_sc.recv - g_tr.recv0,
               g_sc.auth_passes - g_tr.auth_passes0, g_sc.passes - g_tr.passes0);
    }

    /* T1 + T3 + T4: the same seed with camera stalls. */
    c.stalls = 1U;
    tr_run("tractor in scan FAILED, camera stalls", &c);
    print_scenario();
    check_structure();
    check_tr_silent_phase();
    CHECK(g_tr.adopt_us != 0U && g_sc.adoptions == 1U &&
          sx1276_rx_grid_adopted() != 0U && g_sc.locks == 0U &&
          g_sc.demotions == 0U && g_sc.dem_reset == 0U &&
          g_sc.first_anchor == 1U && g_tr.renumbers == 0U &&
          g_scan_mirror == SX1276_RX_SCAN_STATE_FAILED,
          "[%s] adoption in FAILED: adopted %s into the phase, adoptions %u,"
          " still adopted %u, locks %u demotions %u clk_demotion_reset %u,"
          " tx_first_anchor %u, own grid renumbered %u times, scan state %u"
          " (expect adopted, 1, 1, 0 0 0, 1, 0, FAILED)", g_sc.name,
          (g_tr.adopt_us != 0U) ? "yes" : "never", g_sc.adoptions,
          sx1276_rx_grid_adopted(), g_sc.locks, g_sc.demotions, g_sc.dem_reset,
          g_sc.first_anchor, g_tr.renumbers, (unsigned)g_scan_mirror);
    CHECK(g_tr.adopt_us != 0U && g_sc.passes > g_tr.passes_at_adopt &&
          g_sc.auth_passes == g_tr.auth_at_adopt &&
          g_sc.follow_own == g_tr.follow_own_at_adopt &&
          g_sc.follow_locked == 0U &&
          (g_sc.recv - g_tr.recv_at_adopt) * 2U < g_sc.sent - g_tr.sent_at_adopt,
          "[%s] after the adoption: authority on %u of %u passes, self-follow"
          " retunes %u, LOCKED follow %u, cmds received %u of %u (expect 0, 0,"
          " 0, fewer than half)", g_sc.name,
          g_sc.auth_passes - g_tr.auth_at_adopt,
          g_sc.passes - g_tr.passes_at_adopt,
          g_sc.follow_own - g_tr.follow_own_at_adopt, g_sc.follow_locked,
          g_sc.recv - g_tr.recv_at_adopt, g_sc.sent - g_tr.sent_at_adopt);
    if (g_verbose != 0) {
        printf("    phase: adopted at +%.1f s; before it cmds sent=%u recv=%u,"
               " authority on %u/%u passes; after it cmds sent=%u recv=%u,"
               " authority on %u/%u passes\n",
               (g_tr.adopt_us != 0U)
                   ? (double)(g_tr.adopt_us - g_tr.phase_us) / 1e6 : -1.0,
               g_tr.sent_at_adopt - g_tr.sent0, g_tr.recv_at_adopt - g_tr.recv0,
               g_tr.auth_at_adopt - g_tr.auth_passes0,
               g_tr.passes_at_adopt - g_tr.passes0,
               g_sc.sent - g_tr.sent_at_adopt, g_sc.recv - g_tr.recv_at_adopt,
               g_sc.auth_passes - g_tr.auth_at_adopt,
               g_sc.passes - g_tr.passes_at_adopt);
    }
}

/* ------------------------------------------------------------------ */
/* Evaluation (-e): fix options, SIL only                              */
/* ------------------------------------------------------------------ */

#define EV_MAX 1000U
static uint32_t g_ev[EV_MAX];

static void ev_sort(uint32_t *a, uint32_t n) {
    for (uint32_t i = 1U; i < n; ++i) {
        const uint32_t v = a[i];
        uint32_t j = i;
        while (j > 0U && a[j - 1U] > v) {
            a[j] = a[j - 1U];
            --j;
        }
        a[j] = v;
    }
}

static double ev_pct(const uint32_t *a, uint32_t n, uint32_t p) {
    if (n == 0U) {
        return -1.0;
    }
    return (double)a[((n - 1U) * p) / 100U] / 1000.0;
}

static double ev_sqrt(double x) {
    double r = (x > 1.0) ? x : 1.0;
    if (x <= 0.0) {
        return 0.0;
    }
    for (int i = 0; i < 60; ++i) {
        r = 0.5 * (r + x / r);
    }
    return r;
}

/* Wilson 95 % interval for k of n. */
static void ev_wilson(uint32_t k, uint32_t n, double *lo, double *hi) {
    const double z = 1.959964, p = (double)k / (double)n, nn = (double)n;
    const double den = 1.0 + z * z / nn;
    const double mid = (p + z * z / (2.0 * nn)) / den;
    const double half = z * ev_sqrt(p * (1.0 - p) / nn + z * z / (4.0 * nn * nn)) / den;
    *lo = mid - half;
    *hi = mid + half;
}

static void eval_e1(const char *label, uint32_t n, int32_t ppm, uint8_t watchdog) {
    uint32_t nlock = 0U, sent_late = 0U, heard_late = 0U, catches_unlocked = 0U;
    for (uint32_t t = 0U; t < n && t < EV_MAX; ++t) {
        const l6_cfg_t c = {
            .seed = 0xE1000001U + t * 7919U, .ppm = ppm, .period_ms = 500U,
            .stream_at_activation = 0U, .start_abs = 0U,
            .run_after_start_ms = 300000U, .cap_ms = 0U,
            .watchdog = watchdog, .stop_at_lock = 0U,
        };
        l6_run(label, &c);
        if (g_l6.lock_us != 0U && g_l6.lock_us >= g_l6.start_us) {
            g_ev[nlock++] = (uint32_t)((g_l6.lock_us - g_l6.start_us) / 1000ULL);
        } else {
            catches_unlocked += g_l6.ncatch;
        }
        for (uint32_t i = 0U; i < g_os.sent; ++i) {
            if (g_os.log[i].key_us >= g_l6.start_us + 120000000ULL) {
                ++sent_late;
                heard_late += g_os.log[i].heard;
            }
        }
    }
    ev_sort(g_ev, nlock);
    printf("E1 %-34s n=%u ppm=%d: LOCKed %u/%u; T_lock after stream start"
           " median %.1f s p90 %.1f s max %.1f s; heard %u/%u = %.1f%% of the"
           " frames keyed 120-300 s after the start; catches in runs that"
           " never locked %u\n",
           label, n, (int)ppm, nlock, n, ev_pct(g_ev, nlock, 50U),
           ev_pct(g_ev, nlock, 90U), ev_pct(g_ev, nlock, 100U), heard_late,
           sent_late, (sent_late != 0U) ? 100.0 * heard_late / (double)sent_late : 0.0,
           catches_unlocked);
}

static void eval_e2(const char *label, uint32_t n, uint32_t period_ms) {
    uint32_t nlock = 0U, no_lock_120 = 0U, by_attempt[6];
    double lo = 0.0, hi = 0.0;
    memset(by_attempt, 0, sizeof(by_attempt));
    for (uint32_t t = 0U; t < n && t < EV_MAX; ++t) {
        const l6_cfg_t c = {
            .seed = 0xE2000001U + t * 104729U + period_ms, .ppm = 0,
            .period_ms = period_ms, .stream_at_activation = 1U,
            .start_abs = 0U, .run_after_start_ms = 0U, .cap_ms = 300000U,
            .watchdog = 0U, .stop_at_lock = 1U,
        };
        l6_run(label, &c);
        if (g_l6.lock_us != 0U) {
            const uint32_t ms = (uint32_t)((g_l6.lock_us - g_l6.act_us) / 1000ULL);
            g_ev[nlock++] = ms;
            if (ms >= 120000U) {
                ++no_lock_120;
            }
            ++by_attempt[(g_l6.fails_before_lock < 5U) ? g_l6.fails_before_lock : 5U];
        } else {
            ++no_lock_120;
        }
    }
    ev_sort(g_ev, nlock);
    ev_wilson(no_lock_120, n, &lo, &hi);
    printf("E2 %-34s n=%u %u fps: no LOCK within 120 s %u/%u = %.1f%%"
           " (95%% %.1f-%.1f%%); LOCKed by 300 s %u/%u; T_lock median %.1f s"
           " p90 %.1f s; first LOCK after 0/1/2/3/4/5+ FAILs:"
           " %u/%u/%u/%u/%u/%u",
           label, n, 1000U / period_ms, no_lock_120, n,
           100.0 * no_lock_120 / (double)n, 100.0 * lo, 100.0 * hi, nlock, n,
           ev_pct(g_ev, nlock, 50U), ev_pct(g_ev, nlock, 90U),
           by_attempt[0], by_attempt[1], by_attempt[2], by_attempt[3],
           by_attempt[4], by_attempt[5]);
    {
        /* per-attempt catch probability, given the attempt was reached */
        uint32_t reached = n;
        printf("; per-attempt P(catch | reached) =");
        for (uint32_t a = 0U; a < 4U; ++a) {
            printf(" %.2f", (reached != 0U) ? (double)by_attempt[a] / reached : 0.0);
            reached -= by_attempt[a];
        }
        printf("\n");
    }
}

static void eval_e3(const char *label, uint32_t n, uint8_t stalls, uint8_t watchdog,
                    uint32_t silent_ms) {
    uint32_t nfailed = 0U, nadopt = 0U, nlost = 0U, nregrid = 0U, nend_adopted = 0U;
    uint32_t sent = 0U, recv = 0U, locks = 0U, dems = 0U, react = 0U;
    uint32_t renum = 0U, nrenum = 0U, renum_max = 0U;
    uint64_t passes = 0U, auth = 0U;
    for (uint32_t t = 0U; t < n && t < EV_MAX; ++t) {
        const tr_cfg_t c = {
            .seed = 0xE3000001U + t * 7919U, .stalls = stalls, .watchdog = watchdog,
            .silent_ms = silent_ms,
        };
        tr_run(label, &c);
        nfailed += g_tr.failed0;
        if (g_tr.adopt_us != 0U) {
            g_ev[nadopt++] = (uint32_t)((g_tr.adopt_us - g_tr.phase_us) / 1000ULL);
            if (g_sc.auth_passes == g_tr.auth_at_adopt) {
                ++nlost;
            }
        }
        nend_adopted += sx1276_rx_grid_adopted();
        if (g_sc.first_anchor > 1U) {
            ++nregrid;
        }
        if (g_tr.renumbers != 0U) {
            ++nrenum;
            renum += g_tr.renumbers;
            if (g_tr.renumber_max > renum_max) {
                renum_max = g_tr.renumber_max;
            }
        }
        passes += g_sc.passes - g_tr.passes0;
        auth += g_sc.auth_passes - g_tr.auth_passes0;
        sent += g_sc.sent - g_tr.sent0;
        recv += g_sc.recv - g_tr.recv0;
        locks += g_sc.locks;
        dems += g_sc.demotions;
        react += g_rxd.reactivations;
    }
    ev_sort(g_ev, nadopt);
    printf("E3 %-34s n=%u: scan FAILED when the commands start %u/%u;"
           " adopted a grid in the phase %u/%u (median %.1f s in), authority"
           " never back after it"
           " %u/%u, adopted at the end %u/%u; authority on %.1f%% of the phase"
           " passes; cmds received %u/%u = %.1f%%; locks %u demotions %u; phase"
           " restarts (tx_first_anchor > 1) in %u/%u runs; own grid renumbered"
           " (a TX > 1 slot off the previous TX's grid) %u times in %u/%u runs,"
           " by up to %u slots; re-activations %u\n",
           label, n, nfailed, n, nadopt, n, ev_pct(g_ev, nadopt, 50U), nlost,
           nadopt, nend_adopted, n,
           (passes != 0U) ? 100.0 * (double)auth / (double)passes : 0.0,
           recv, sent, (sent != 0U) ? 100.0 * recv / (double)sent : 0.0,
           locks, dems, nregrid, n, renum, nrenum, n, renum_max, react);
}

static void run_eval_tractor(void) {
    /* Commands from 5 s: the scan LOCKs on them and never fails out --
     * main's decay-window behaviour before FAILED (scenario 7's shape). */
    eval_e3("tractor SCANNING + stalls", 50U, 1U, 0U, 5000U);
    eval_e3("tractor FAILED, no stalls (control)", 20U, 0U, 0U, 0U);
    eval_e3("tractor FAILED + stalls", 50U, 1U, 0U, 0U);
    eval_e3("tractor FAILED + stalls + watchdog", 50U, 1U, 1U, 0U);
}

static void run_eval(void) {
    printf("[RXSFF] evaluation: scan DWELL %u ms, REDESIGN %u ms, MAX_RETRIES %u\n",
           (unsigned)SX1276_RX_SCAN_DWELL_MS, (unsigned)SX1276_RX_SCAN_REDESIGN_MS,
           (unsigned)SX1276_RX_SCAN_MAX_RETRIES);
    eval_e1("leg-6 replay", 100U, 0, 0U);
    eval_e1("leg-6 replay, drifting clocks", 100U, 2750, 0U);
    eval_e1("leg-6 replay + host watchdog (c)", 100U, 0, 1U);
    eval_e2("cold start, stream on", 600U, 500U);
    eval_e2("cold start, stream on", 600U, 1000U);
    run_eval_tractor();
}

int main(int argc, char **argv) {
    int eval = 0;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-v") == 0) { g_verbose = 1; }
        if (strcmp(argv[i], "-e") == 0) { eval = 1; }
        if (strcmp(argv[i], "-e3") == 0) { eval = 3; }
        if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            g_l6_seed = (uint32_t)strtoul(argv[++i], NULL, 0);
        }
    }
    if (eval == 3) {
        run_eval_tractor();
        return 0;
    }
    if (eval != 0) {
        run_eval();
        return 0;
    }
    test_leg6_characterisation();
    test_tractor_failed_adoption();
    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] rx_scan_failed_follower: %d failure(s) -- the"
                " characterised scan-FAILED behaviour changed: a firmware fix"
                " for RS-12.21 (then replace these checks with its acceptance"
                " checks) or an unrelated change to the scan constants, the"
                " walker, the TX path or the shared sim (rx_reverse_delivery.c)"
                " -- see which check failed\n", g_failures);
        return 1;
    }
    printf("[PASS] rx_scan_failed_follower: KNOWN DEFECTS still present"
           " (RS-12.21, firmware path) -- a follower in scan FAILED adopts the"
           " grid but never LOCKs or hops (leg 6); an originator in scan FAILED"
           " that adopts a grid never holds authority or self-follows again\n");
    return 0;
}
