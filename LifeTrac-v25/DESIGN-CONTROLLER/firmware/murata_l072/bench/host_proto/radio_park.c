/*
 * RS-12.21 (2026-10-24): the FIRMWARE RADIO PARK (HOST_TYPE_RADIO_PARK_REQ,
 * include/host_types.h), system-in-the-loop against the real firmware.
 *
 * WHY. The bench parks both L072s between legs. radio_park.py did it with
 * raw STANDBY and SLEEP writes and a 0x80 readback, which held only because
 * a node without traffic ran its scan out into FAILED and parked itself
 * there (TODO RS-12.21). Scan option (b) (FHSS_SCAN_FAILED_OPTIONS.md
 * section 5 item 2, adopted 2026-10-24) removes that self-park: a node
 * without originator authority scans on after its budget, re-arming RXCONT
 * every 2 s dwell and reporting a final RX_SCAN_FAILED every 30 s, so a host
 * SLEEP write holds only until the next walker step. The park command is the
 * firmware's own persistent radio-off state, flashed in the same image.
 *
 * WHAT. The simulated SX1276, clock, air and main loop of
 * check-rx-reverse-delivery (rx_reverse_delivery.c, #included with
 * RX_REVERSE_DELIVERY_SIM_ONLY and RX_SIM_REAL_HOST) against the REAL
 * host_cmd.c (dispatcher, TX ring, REG_WRITE gate), host_cfg.c and
 * host_cfg_profile.c (CFG_SET REG_PROFILE activation), host_reg_gate.c,
 * and the same real sx1276_rx.c / sx1276_tx.c / scan / grid / clock TUs.
 * Every host request goes through host_cmd_dispatch() and every reply is
 * captured at host_uart_send_urc(), FAULT_URCs included (fed back to the
 * shared simulation's bookkeeping). The shared structural checks
 * (check_structure) run in every scenario. The production build's REG_WRITE
 * gate is used (HOST_ALLOW_REG_WRITE_DIAG = 0).
 *
 * ASSERTED (sequence numbers match the output):
 *   B   boot: the flag is clear at start -- a reset zeroes it, and this
 *       process start is that reset; READY_URC and VER_URC advertise
 *       HOST_CAP_RADIO_PARK; the profile-1 activation goes through CFG_SET.
 *   P1  park after the final FAIL (base, nothing on air): the scan runs out
 *       (four RX_SCAN_FAILED, the last final) and, by (b), scans on. PARK_REQ
 *       -> RADIO_PARK_URC {0 PARKED, 0x80}, same seq, IRQ flags cleared.
 *       Then 10 min with a stream on air: the modem ends every pass in SLEEP,
 *       no firmware call touches it (no retune, standby, RX arm or TX), the
 *       scan machine records nothing (unparked: a walker step every 2 s, a
 *       FAIL and a (b) restart every 30 s), no FAULT goes out, no frame is
 *       heard. REG_READ 0x01 = 0x80; a repeat PARK_REQ -> {1 ALREADY, 0x80};
 *       TX_FRAME_REQ -> ERR_PROTO {0x10, 1, FORBIDDEN, 0, 0} with its seq,
 *       nothing queued, keyed or reported (no RFCO); sx1276_tx_begin() and
 *       sx1276_rx_arm() refuse. A host raw RXCONT (REG_WRITE 0x01 = 0x85)
 *       ends it: RX_CONT at once; the interrupted attempt FAILs exactly 30 s
 *       plus the time parked after it began (the parked time not counted,
 *       no FAIL at the first tick); the base then LOCKs onto a stream.
 *   P2  park while LOCKED (the base following a stream), with a frame's DIO0
 *       edge latched: that frame is not delivered and no header considered.
 *       2 min LOCKED on an adopted grid with a valid clock (the slot
 *       follower's case), then 2 min with the clock reset (the gamma-1
 *       fallback's case): nothing touches the modem, the scan machine stays
 *       LOCKED and records nothing, gamma-1 records no decision. CFG_SET
 *       REG_PROFILE=1 ends it: the scan starts over and LOCKs again.
 *   P3  park against TX: PARK_REQ while a TX is on air -> {2 REFUSED, 0x83},
 *       nothing changed, and the post-TX re-arm runs at its TX_DONE; while a
 *       TX_FRAME_REQ waits in host_cmd.c's TX ring -> REFUSED, and the frame
 *       goes out; then PARKED. The 0x0D split at its emitter: that first
 *       TX's FIFO, altered before TX_DONE, raises FAULT 0x0F
 *       (TX_FIFO_RB_MISMATCH), sub = its tx_id, and no 0x0D. Raw writes that
 *       do not end the park: RXSINGLE (refused in a production build),
 *       STANDBY (the modem leaves SLEEP, and for 60 s nothing arms it; a
 *       repeat PARK_REQ puts it back: ALREADY, 0x80) and SLEEP. RESET_REQ
 *       reaches platform_system_reset(). A raw RXCONT ends it, and a
 *       TX_FRAME_REQ goes on air again.
 *   P4  the tractor: its scan ran out into FAILED, held while it streams;
 *       the camera daemon stops and the tractor is parked while it still
 *       holds authority. 5 min: the authority decays, yet no (b) restart,
 *       nothing touches the modem, FAILED kept. The daemon's raw RXCONT ends
 *       the park, the first scan tick restarts the scan (b), and the camera
 *       streams again.
 *   P5  a raw RXCONT unpark resumes the scan machine as sx1276_rx_unpark()
 *       promises (2026-10-24 review): parked 10 s while SCANNING, 700 ms
 *       into a dwell, the next walker step comes the dwell's remaining
 *       1300 ms after the unpark, not at the first tick; parked 10 s while
 *       LOCKED on an adopted grid with the stream still on air, the clock
 *       is stale at the unpark (F6) and the scan demotes on the first tick
 *       after it (its loss timer counts the park, as the clock's freshness
 *       does), then re-acquires.
 * Each scenario fails with the park gates in sx1276_rx.c removed (shown
 * 2026-10-24 in a scratch copy); P5 with the SCANNING anchor shift removed
 * or the shift applied to LOCKED as well.
 */

#define RX_REVERSE_DELIVERY_SIM_ONLY 1
#define RX_SIM_REAL_HOST 1
/* Some of its helpers are not used here. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#include "rx_reverse_delivery.c"
#pragma GCC diagnostic pop

#include "host_cfg.h"
#include "reset_cause.h"

/* The wire values the hosts were built against (RS-12.21 decision record):
 * a change here is a wire change, not a refactor. */
_Static_assert(HOST_TYPE_RADIO_PARK_REQ == 0x04U, "RS-12.21: PARK_REQ is 0x04");
_Static_assert(HOST_TYPE_RADIO_PARK_URC == 0x84U, "RS-12.21: PARK_URC is 0x84");
_Static_assert(HOST_RADIO_PARK_URC_PAYLOAD_LEN == 2U &&
               HOST_RADIO_PARK_STATUS_PARKED == 0U &&
               HOST_RADIO_PARK_STATUS_ALREADY == 1U &&
               HOST_RADIO_PARK_STATUS_REFUSED == 2U,
               "RS-12.21: PARK_URC is {status 0/1/2, opmode}");
_Static_assert(HOST_FAULT_CODE_RX_SCAN_FAILED == 0x0DU &&
               HOST_FAULT_CODE_TX_FIFO_RB_MISMATCH == 0x0FU,
               "the 0x0D split: RX_SCAN_FAILED keeps 0x0D, the FIFO fault is 0x0F");
_Static_assert((HOST_CAPABILITY_BITMAP & HOST_CAP_RADIO_PARK) != 0UL,
               "RS-12.21: the park is advertised");
_Static_assert(HOST_PROTOCOL_VER == 1U && HOST_WIRE_SCHEMA_VER == 2U,
               "RS-12.21: additive changes, no version bump (host_types.h)");

#define FINAL_BIT ((uint8_t)(1U << SX1276_RX_SCAN_FAIL_SUB_BIT_FINAL))
#define OPMODE_SLEEP 0x80U
#define OPMODE_TX    0x83U

/* ------------------------------------------------------------------ */
/* Host link: replies captured, host-side collaborators stubbed        */
/* ------------------------------------------------------------------ */

typedef struct {
    uint32_t count;
    uint16_t seq;
    uint16_t len;
    uint8_t  payload[32];
} urc_slot_t;
static urc_slot_t g_urc[256];          /* last URC of each type, and a count */
static uint32_t   g_fault_n[256];      /* FAULT_URCs by code */
static uint8_t    g_fault_sub[256];    /* ... and the last one's sub */

void host_uart_send_urc(uint8_t type, uint16_t seq, uint8_t flags,
                        const uint8_t *payload, uint16_t payload_len) {
    urc_slot_t *u = &g_urc[type];
    (void)flags;
    ++u->count;
    u->seq = seq;
    u->len = payload_len;
    memset(u->payload, 0, sizeof(u->payload));
    if (payload != NULL) {
        memcpy(u->payload, payload,
               (payload_len < sizeof(u->payload)) ? payload_len : sizeof(u->payload));
    }
    if (type == HOST_TYPE_FAULT_URC && payload != NULL && payload_len >= 2U) {
        ++g_fault_n[payload[0]];
        g_fault_sub[payload[0]] = payload[1];
        sim_note_fault(payload[0], payload[1]);
    }
}

/* host_uart.c is not linked: its ERR_PROTO packer, as there. */
void host_uart_send_err_proto(uint16_t seq, uint8_t offending_type,
                              uint8_t offending_ver, uint8_t err_code,
                              uint16_t detail) {
    uint8_t payload[5];
    payload[0] = offending_type;
    payload[1] = offending_ver;
    payload[2] = err_code;
    payload[3] = (uint8_t)(detail & 0xFFU);
    payload[4] = (uint8_t)((detail >> 8) & 0xFFU);
    host_uart_send_urc(HOST_TYPE_ERR_PROTO_URC, seq, 0U, payload,
                       (uint16_t)sizeof(payload));
}
void host_uart_send_ascii(const char *text) { (void)text; }
void host_uart_note_diag_mark(uint8_t mark) { (void)mark; }

/* host_stats.c is not linked (the radio-side counters are the sim's). */
void host_stats_reset(void) {}
void host_stats_note_radio_events(uint32_t events) { (void)events; }
uint16_t host_stats_serialize(uint8_t *out, uint16_t out_cap) {
    (void)out; (void)out_cap;
    return 0U;
}

/* platform (the clock is the sim's). */
reset_cause_t platform_reset_cause_take(void) { return RESET_CAUSE_POR_BOR; }
uint8_t platform_clock_source_id(void) { return (uint8_t)PLATFORM_CLOCK_SOURCE_HSE_OK; }
bool platform_fault_take_replay(platform_fault_record_t *out_record) {
    (void)out_record;
    return false;
}
void platform_diag_trace(const char *text) { (void)text; }
static uint32_t g_system_resets;
void platform_system_reset(void) { ++g_system_resets; }

/* The sx1276.c setters host_cfg.c / host_cfg_profile.c call; the sim's
 * register file already holds SF7 / BW250 / CR4/5. */
void sx1276_set_tx_power_dbm(uint8_t dbm) { (void)dbm; }
void sx1276_set_sf_bw_cr(uint8_t sf, uint16_t bw_khz, uint8_t cr_den) {
    (void)sf; (void)bw_khz; (void)cr_den;
}

/* sx1276_modes.c's mirror of a host raw RegOpMode write (handle_reg_write).
 * In the sim the tracked state IS the chip, so the write lands here. */
void sx1276_modes_sync_external(uint8_t raw_opmode) {
    switch (raw_opmode & 0x07U) {
        case 0x00U:
            g_radio.mode = SX1276_STATE_SLEEP;
            break;
        case 0x05U:
            g_radio.mode = SX1276_STATE_RX_CONT;
            g_radio.rx_since_us = g_now_us;
            break;
        default:
            g_radio.mode = SX1276_STATE_STANDBY;
            break;
    }
    trace_mix(5U, raw_opmode);
}

/* ------------------------------------------------------------------ */
/* Host requests through the real dispatcher                           */
/* ------------------------------------------------------------------ */

static uint16_t g_host_seq = 0x0100U;

static uint16_t host_req(uint8_t type, const uint8_t *payload, uint16_t len) {
    static host_frame_t f;
    const ctx_t saved = g_ctx;
    memset(&f, 0, sizeof(f));
    f.ver = HOST_PROTOCOL_VER;
    f.type = type;
    f.seq = ++g_host_seq;
    f.payload_len = len;
    if (len > 0U) {
        memcpy(f.payload, payload, len);
    }
    g_ctx = CTX_HOST;
    host_cmd_dispatch(&f);
    g_ctx = saved;
    return f.seq;
}

/* PARK_REQ: the reply's status byte (0xFF: no well-formed reply), its
 * opmode byte in *opmode. */
static uint8_t park_req(uint8_t *opmode) {
    const urc_slot_t *u = &g_urc[HOST_TYPE_RADIO_PARK_URC];
    const uint32_t n0 = u->count;
    const uint16_t seq = host_req(HOST_TYPE_RADIO_PARK_REQ, NULL, 0U);
    CHECK(u->count == n0 + 1U && u->seq == seq &&
          u->len == HOST_RADIO_PARK_URC_PAYLOAD_LEN,
          "[%s] PARK_REQ seq %u: %u replies, seq %u, len %u (expect 1, %u, %u)",
          g_sc.name, seq, u->count - n0, u->seq, u->len, seq,
          HOST_RADIO_PARK_URC_PAYLOAD_LEN);
    if (u->count != n0 + 1U) {
        return 0xFFU;
    }
    *opmode = u->payload[1];
    return u->payload[0];
}

static void reg_write_ok(uint8_t reg, uint8_t value) {
    const uint8_t p[2] = { reg, value };
    const uint32_t n0 = g_urc[HOST_TYPE_REG_WRITE_ACK_URC].count;
    const uint16_t seq = host_req(HOST_TYPE_REG_WRITE_REQ, p, 2U);
    CHECK(g_urc[HOST_TYPE_REG_WRITE_ACK_URC].count == n0 + 1U &&
          g_urc[HOST_TYPE_REG_WRITE_ACK_URC].seq == seq,
          "[%s] REG_WRITE %02x=%02x not acknowledged", g_sc.name, reg, value);
}

static void reg_write_refused(uint8_t reg, uint8_t value) {
    const uint8_t p[2] = { reg, value };
    const urc_slot_t *e = &g_urc[HOST_TYPE_ERR_PROTO_URC];
    const uint32_t n0 = e->count;
    const uint32_t ack0 = g_urc[HOST_TYPE_REG_WRITE_ACK_URC].count;
    const uint16_t seq = host_req(HOST_TYPE_REG_WRITE_REQ, p, 2U);
    CHECK(e->count == n0 + 1U && e->seq == seq &&
          e->payload[2] == HOST_ERR_PROTO_FORBIDDEN &&
          g_urc[HOST_TYPE_REG_WRITE_ACK_URC].count == ack0,
          "[%s] REG_WRITE %02x=%02x was not refused", g_sc.name, reg, value);
}

static uint8_t reg_read(uint8_t reg) {
    const urc_slot_t *u = &g_urc[HOST_TYPE_REG_DATA_URC];
    const uint32_t n0 = u->count;
    const uint16_t seq = host_req(HOST_TYPE_REG_READ_REQ, &reg, 1U);
    CHECK(u->count == n0 + 1U && u->seq == seq && u->payload[0] == reg,
          "[%s] REG_READ %02x: no reply", g_sc.name, reg);
    return u->payload[1];
}

/* TX_FRAME_REQ with a 16-byte body. */
static uint16_t tx_req(uint8_t tx_id) {
    uint8_t p[2U + CMD_PAYLOAD_LEN];
    memset(p, 0, sizeof(p));
    p[0] = tx_id;
    p[1] = (uint8_t)CMD_PAYLOAD_LEN;
    p[2] = MARK_BASE_CMD;
    p[3] = tx_id;
    return host_req(HOST_TYPE_TX_FRAME_REQ, p, (uint16_t)sizeof(p));
}

static uint8_t cfg_set_req(uint8_t key, const uint8_t *value, uint8_t len) {
    uint8_t p[2U + CFG_KEY_MAX_VALUE_LEN];
    const urc_slot_t *u = &g_urc[HOST_TYPE_CFG_OK_URC];
    const uint32_t n0 = u->count;
    uint16_t seq;
    p[0] = key;
    p[1] = len;
    memcpy(&p[2], value, len);
    seq = host_req(HOST_TYPE_CFG_SET_REQ, p, (uint16_t)(2U + len));
    CHECK(u->count == n0 + 1U && u->seq == seq && u->payload[0] == key,
          "[%s] CFG_SET %02x: no CFG_OK reply", g_sc.name, key);
    return u->payload[1];
}

/* CFG_SET REG_PROFILE=1 -- host_cfg.c -> host_cfg_profile_activate(). */
static void activate_fhss(void) {
    const uint8_t profile = (uint8_t)REG_PROFILE_FCC_15_247_FHSS_50CH_BW250;
    CHECK(cfg_set_req(CFG_KEY_REG_PROFILE, &profile, 1U) == (uint8_t)CFG_STATUS_OK &&
          host_cfg_profile_active()->profile_id == profile,
          "[%s] the profile-1 activation failed", g_sc.name);
    /* The activation reset the scan machine to BOOT. */
    g_scan_mirror = SX1276_RX_SCAN_STATE_BOOT;
}

/* A TX_FRAME_REQ while parked: ERR_PROTO {0x10, ver, FORBIDDEN, 0, 0} with
 * its seq, the existing refusal; nothing queued, keyed or reported. */
static void expect_tx_refused(uint8_t tx_id) {
    const urc_slot_t *e = &g_urc[HOST_TYPE_ERR_PROTO_URC];
    const uint32_t n0 = e->count;
    const uint32_t tx0 = g_calls.tx;
    const uint32_t rfco0 = g_urc[HOST_TYPE_RFCO_PERTX_URC].count;
    const uint16_t seq = tx_req(tx_id);
    CHECK(e->count == n0 + 1U && e->seq == seq && e->len == 5U &&
          e->payload[0] == HOST_TYPE_TX_FRAME_REQ &&
          e->payload[1] == HOST_PROTOCOL_VER &&
          e->payload[2] == HOST_ERR_PROTO_FORBIDDEN &&
          e->payload[3] == 0U && e->payload[4] == 0U,
          "[%s] TX_FRAME_REQ while parked: %u ERR_PROTO replies, last"
          " {%02x %02x %02x %02x %02x} seq %u (expect {10 01 08 00 00} seq %u)",
          g_sc.name, e->count - n0, e->payload[0], e->payload[1],
          e->payload[2], e->payload[3], e->payload[4], e->seq, seq);
    CHECK(!sx1276_tx_busy() && !host_cmd_tx_mailbox_pending() &&
          g_calls.tx == tx0 &&
          g_urc[HOST_TYPE_RFCO_PERTX_URC].count == rfco0 &&
          g_radio.mode == SX1276_STATE_SLEEP,
          "[%s] TX_FRAME_REQ while parked: queued %u, keyed %u, RFCO %u,"
          " modem state %u", g_sc.name, (unsigned)host_cmd_tx_mailbox_pending(),
          g_calls.tx - tx0, g_urc[HOST_TYPE_RFCO_PERTX_URC].count - rfco0,
          (unsigned)g_radio.mode);
}

/* ------------------------------------------------------------------ */
/* Observation                                                         */
/* ------------------------------------------------------------------ */

typedef struct {
    uint32_t freq, standby, rx_cont, tx, sleep, scan_rec;
    uint32_t faults, frames_recv, retune_dec, consider_dec;
} snap_t;

static uint32_t total_faults(void) {
    uint32_t n = 0U;
    for (unsigned i = 0U; i < 256U; ++i) {
        n += g_fault_n[i];
    }
    return n;
}

static snap_t snap_now(void) {
    snap_t s;
    uint32_t rt[SX1276_RX_RETUNE_DECISION_DIM];
    uint32_t cr[SX1276_RX_CONSIDER_REMOTE_COUNT_DIM];
    memset(&s, 0, sizeof(s));
    s.freq = g_calls.freq;
    s.standby = g_calls.standby;
    s.rx_cont = g_calls.rx_cont;
    s.tx = g_calls.tx;
    s.sleep = g_calls.sleep;
    s.scan_rec = g_calls.scan_rec;
    s.faults = total_faults();
    s.frames_recv = g_sc.frames_recv;
    sx1276_rx_retune_counts(rt);
    for (unsigned i = 0U; i < SX1276_RX_RETUNE_DECISION_DIM; ++i) {
        s.retune_dec += rt[i];
    }
    sx1276_rx_consider_remote_counts(cr);
    for (unsigned i = 0U; i < SX1276_RX_CONSIDER_REMOTE_COUNT_DIM; ++i) {
        s.consider_dec += cr[i];
    }
    return s;
}

/* Nothing touched the parked modem, the scan machine stood still. */
static void expect_untouched(const char *what, const snap_t *a, const snap_t *b) {
    CHECK(b->freq == a->freq && b->standby == a->standby &&
          b->rx_cont == a->rx_cont && b->tx == a->tx,
          "[%s] %s: the firmware touched the parked modem (retunes %u,"
          " standby %u, RX arms %u, TX %u)", g_sc.name, what,
          b->freq - a->freq, b->standby - a->standby,
          b->rx_cont - a->rx_cont, b->tx - a->tx);
    CHECK(b->scan_rec == a->scan_rec,
          "[%s] %s: the scan machine ran while parked (%u records)",
          g_sc.name, what, b->scan_rec - a->scan_rec);
    CHECK(b->faults == a->faults,
          "[%s] %s: %u FAULT URCs while parked", g_sc.name, what,
          b->faults - a->faults);
    CHECK(b->retune_dec == a->retune_dec,
          "[%s] %s: gamma-1 decided %u times while parked", g_sc.name, what,
          b->retune_dec - a->retune_dec);
    CHECK(b->frames_recv == a->frames_recv && b->consider_dec == a->consider_dec,
          "[%s] %s: %u frames delivered, %u headers considered while parked",
          g_sc.name, what, b->frames_recv - a->frames_recv,
          b->consider_dec - a->consider_dec);
}

/* Base role: the shared simulation's ideal originator, one frame per slot
 * on its own grid (rx_reverse_delivery.c's orig_start, which its
 * SIM_ONLY build leaves out). */
static void orig_on(uint64_t first_boundary_us, uint32_t first_abs) {
    g_orig.on = 1U;
    g_orig.next_slot_us = first_boundary_us;
    g_orig.next_abs = first_abs;
}

/* Run the firmware loop; return the passes that ended out of SLEEP. */
static uint32_t run_parked_ms(uint32_t ms) {
    uint32_t not_asleep = 0U;
    const uint64_t until = g_now_us + (uint64_t)ms * 1000ULL;
    while (g_now_us < until) {
        fw_pass();
        if (g_radio.mode != SX1276_STATE_SLEEP) {
            ++not_asleep;
        }
    }
    return not_asleep;
}

/* Run until the scan machine has LOCKed `locks` times in all (or the cap). */
static void run_until_locks(uint32_t locks, uint32_t cap_ms) {
    const uint64_t t0 = g_now_us;
    while (g_sc.locks < locks && g_now_us - t0 < (uint64_t)cap_ms * 1000ULL) {
        fw_pass();
    }
}

/* Run until no TX is keyed or waiting in host_cmd.c's ring (or the cap). */
static void run_until_tx_idle(uint32_t cap_ms) {
    const uint64_t t0 = g_now_us;
    while ((sx1276_tx_busy() || host_cmd_tx_mailbox_pending()) &&
           g_now_us - t0 < (uint64_t)cap_ms * 1000ULL) {
        fw_pass();
    }
}

static void print_park(void) {
    if (g_verbose == 0) {
        return;
    }
    print_scenario();
    printf("    modem calls: retunes=%u standby=%u rx_arms=%u tx=%u sleep=%u |"
           " scan records=%u | FAULT 0x0D=%u 0x0F=%u | PARK_URC=%u ERR_PROTO=%u"
           " | resets=%u | parked=%u\n",
           g_calls.freq, g_calls.standby, g_calls.rx_cont, g_calls.tx,
           g_calls.sleep, g_calls.scan_rec, g_fault_n[0x0DU], g_fault_n[0x0FU],
           g_urc[HOST_TYPE_RADIO_PARK_URC].count,
           g_urc[HOST_TYPE_ERR_PROTO_URC].count, g_system_resets,
           (unsigned)sx1276_rx_parked());
}

/* ------------------------------------------------------------------ */
/* Scenarios                                                           */
/* ------------------------------------------------------------------ */

static void test_boot(void) {
    static const uint8_t mask50[8] = {
        0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x03U, 0x00U
    };
    const uint8_t profile = (uint8_t)REG_PROFILE_FCC_15_247_FHSS_50CH_BW250;
    const urc_slot_t *ready = &g_urc[HOST_TYPE_READY_URC];
    const urc_slot_t *ver = &g_urc[HOST_TYPE_VER_URC];
    uint32_t cap;
    uint16_t seq;

    g_sc.name = "B boot";
    /* Power-on: a reset. The flag is zero-initialised storage (.bss), which
     * startup.c clears on every reset, RESET_REQ's NVIC reset included. */
    CHECK(!sx1276_rx_parked(), "[%s] parked out of reset", g_sc.name);

    g_ctx = CTX_BOOT;
    host_cmd_init(true, 0x12U);
    g_ctx = CTX_NONE;
    cap = (uint32_t)ready->payload[4] | ((uint32_t)ready->payload[5] << 8) |
          ((uint32_t)ready->payload[6] << 16) | ((uint32_t)ready->payload[7] << 24);
    CHECK(g_urc[HOST_TYPE_BOOT_URC].count == 1U && ready->count == 1U &&
          (cap & HOST_CAP_RADIO_PARK) != 0UL && cap == HOST_CAPABILITY_BITMAP,
          "[%s] READY_URC capability 0x%08x lacks HOST_CAP_RADIO_PARK", g_sc.name,
          cap);
    seq = host_req(HOST_TYPE_VER_REQ, NULL, 0U);
    cap = (uint32_t)ver->payload[8] | ((uint32_t)ver->payload[9] << 8) |
          ((uint32_t)ver->payload[10] << 16) | ((uint32_t)ver->payload[11] << 24);
    CHECK(ver->count == 1U && ver->seq == seq &&
          ver->payload[0] == HOST_PROTOCOL_VER &&
          ver->payload[1] == HOST_WIRE_SCHEMA_VER &&
          (cap & HOST_CAP_RADIO_PARK) != 0UL,
          "[%s] VER_URC: proto %u schema %u capability 0x%08x", g_sc.name,
          ver->payload[0], ver->payload[1], cap);

    /* The daemons' activation: 50-channel mask, then REG_PROFILE=1. */
    CHECK(cfg_set_req(CFG_KEY_FHSS_CHANNEL_MASK, mask50, 8U) == (uint8_t)CFG_STATUS_OK,
          "[%s] CFG_SET FHSS_CHANNEL_MASK failed", g_sc.name);
    CHECK(cfg_set_req(CFG_KEY_REG_PROFILE, &profile, 1U) == (uint8_t)CFG_STATUS_OK &&
          host_cfg_profile_active()->profile_id == profile,
          "[%s] CFG_SET REG_PROFILE=1 failed", g_sc.name);
    CHECK(!sx1276_rx_parked() && total_faults() == 0U,
          "[%s] parked %u, %u faults after boot", g_sc.name,
          (unsigned)sx1276_rx_parked(), total_faults());
}

static void test_park_after_final_fail(void) {
    uint8_t op = 0U;
    uint32_t park_ms;
    uint32_t not_asleep;
    snap_t s0;
    snap_t s1;

    scenario_begin("P1 park after the final FAIL (base)", ROLE_BASE, 0x5EED0001U);
    CHECK(!sx1276_rx_parked(), "[%s] parked at the start", g_sc.name);
    {
        const uint64_t t0 = g_now_us;
        while (g_sc.scan_fail_urc < 4U && g_now_us - t0 < 130000000ULL) {
            fw_pass();
        }
    }
    CHECK(g_sc.scan_fail_urc == 4U && (g_fail_sub[3] & FINAL_BIT) != 0U &&
          g_scan_mirror == SX1276_RX_SCAN_STATE_SCANNING &&
          g_radio.mode == SX1276_STATE_RX_CONT,
          "[%s] setup: %u FAILs (last sub %02x), scan state %u, modem state %u"
          " (expect 4, final, SCANNING by (b), RX_CONT)", g_sc.name,
          g_sc.scan_fail_urc, g_fail_sub[3], (unsigned)g_scan_mirror,
          (unsigned)g_radio.mode);

    park_ms = platform_now_ms();
    s0 = snap_now();
    g_radio.irq = (uint8_t)(IRQ_RX_DONE | IRQ_VALID_HEADER);   /* stale flags */
    CHECK(park_req(&op) == HOST_RADIO_PARK_STATUS_PARKED && op == OPMODE_SLEEP &&
          sx1276_rx_parked() && g_radio.mode == SX1276_STATE_SLEEP &&
          g_radio.irq == 0U,
          "[%s] PARK_REQ: status/opmode %02x, parked %u, modem state %u, IRQ"
          " flags %02x (expect PARKED, 0x80, 1, SLEEP, 00)", g_sc.name, op,
          (unsigned)sx1276_rx_parked(), (unsigned)g_radio.mode, g_radio.irq);

    /* Ten minutes, with a stream on air the whole time. */
    orig_on(g_now_us + 37000ULL, 777U);
    not_asleep = run_parked_ms(600000U);
    s1 = snap_now();
    CHECK(not_asleep == 0U, "[%s] %u parked passes ended out of SLEEP",
          g_sc.name, not_asleep);
    expect_untouched("10 min parked, a stream on air", &s0, &s1);
    CHECK(s1.sleep == s0.sleep + 1U && g_sc.frames_sent > 1000U,
          "[%s] %u SLEEP transitions (expect the park's 1), %u frames on air",
          g_sc.name, s1.sleep - s0.sleep, g_sc.frames_sent);
    CHECK(reg_read(0x01U) == OPMODE_SLEEP,
          "[%s] RegOpMode read back out of SLEEP after 10 min parked", g_sc.name);

    CHECK(park_req(&op) == HOST_RADIO_PARK_STATUS_ALREADY && op == OPMODE_SLEEP &&
          sx1276_rx_parked(),
          "[%s] repeat PARK_REQ: %02x (expect ALREADY, 0x80)", g_sc.name, op);
    expect_tx_refused(0x42U);
    {
        /* The radio-layer gates themselves. */
        sx1276_tx_request_t req;
        const snap_t a = snap_now();
        const uint32_t rfco0 = g_urc[HOST_TYPE_RFCO_PERTX_URC].count;
        memset(&req, 0, sizeof(req));
        req.tx_id = 0x43U;
        req.length = (uint8_t)CMD_PAYLOAD_LEN;
        CHECK(!sx1276_tx_begin(&req) && !sx1276_tx_busy(),
              "[%s] sx1276_tx_begin() keyed a parked radio", g_sc.name);
        CHECK(!sx1276_rx_arm(), "[%s] sx1276_rx_arm() armed a parked radio",
              g_sc.name);
        {
            const snap_t b = snap_now();
            CHECK(b.tx == a.tx && b.rx_cont == a.rx_cont && b.standby == a.standby &&
                  b.freq == a.freq &&
                  g_urc[HOST_TYPE_RFCO_PERTX_URC].count == rfco0 &&
                  g_radio.mode == SX1276_STATE_SLEEP,
                  "[%s] a refused TX/arm touched the modem", g_sc.name);
        }
    }

    /* The park ends at the daemons' raw RXCONT. The stream stops first, so
     * the interrupted attempt runs out and its FAIL times the resume. */
    g_orig.on = 0U;
    (void)run_parked_ms(1000U);
    {
        const uint32_t unpark_ms = platform_now_ms();
        const uint32_t due = g_fail_ms[3] + SX1276_RX_SCAN_REDESIGN_MS +
                             (unpark_ms - park_ms);
        reg_write_ok(0x01U, 0x85U);
        CHECK(!sx1276_rx_parked() && g_radio.mode == SX1276_STATE_RX_CONT,
              "[%s] raw RXCONT: parked %u, modem state %u", g_sc.name,
              (unsigned)sx1276_rx_parked(), (unsigned)g_radio.mode);
        {
            const uint64_t t0 = g_now_us;
            while (g_sc.scan_fail_urc < 5U && g_now_us - t0 < 40000000ULL) {
                fw_pass();
            }
        }
        CHECK(g_sc.scan_fail_urc == 5U && g_fail_ms[4] >= due &&
              g_fail_ms[4] <= due + 2U && (g_fail_sub[4] & FINAL_BIT) != 0U,
              "[%s] the attempt the park interrupted FAILed at %u ms (sub %02x),"
              " expect %u ms: its start + 30 s + %u ms parked", g_sc.name,
              g_fail_ms[4], g_fail_sub[4], due, unpark_ms - park_ms);
    }
    {
        const uint32_t locks0 = g_sc.locks;
        const uint32_t frames0 = g_sc.frames_recv;
        orig_on(g_now_us + 41000ULL, 4242U);
        run_until_locks(locks0 + 1U, 200000U);
        run_for_ms(20000U);
        CHECK(g_sc.locks > locks0 && g_scan_mirror == SX1276_RX_SCAN_STATE_LOCKED &&
              g_sc.frames_recv >= frames0 + 40U,
              "[%s] no re-acquisition after the park: locks %u -> %u, %u frames",
              g_sc.name, locks0, g_sc.locks, g_sc.frames_recv - frames0);
    }
    print_park();
    check_structure();
}

static void test_park_while_locked(void) {
    uint8_t op = 0U;
    uint32_t not_asleep;
    uint32_t frames0;
    uint32_t consider0;
    snap_t s0;
    snap_t s1;

    scenario_begin("P2 park while LOCKED (base following)", ROLE_BASE, 0x5EED0002U);
    CHECK(!sx1276_rx_parked(), "[%s] parked at the start", g_sc.name);
    orig_on(g_now_us + 37000ULL, 1234U);
    run_until_locks(1U, 150000U);
    run_for_ms(10000U);
    CHECK(g_scan_mirror == SX1276_RX_SCAN_STATE_LOCKED &&
          sx1276_rx_grid_adopted() != 0U && sx1276_fhss_clock_valid() != 0U &&
          g_sc.follow_locked > 0U,
          "[%s] setup: scan state %u, adopted %u, clock %u, follow retunes %u",
          g_sc.name, (unsigned)g_scan_mirror, sx1276_rx_grid_adopted(),
          sx1276_fhss_clock_valid(), g_sc.follow_locked);

    /* Park with a frame's DIO0 edge latched for the next pass. */
    {
        const uint64_t t0 = g_now_us;
        while (g_radio.dio0 == 0U && g_now_us - t0 < 5000000ULL) {
            fw_pass();
        }
    }
    CHECK(g_radio.dio0 != 0U, "[%s] setup: no frame edge to park on", g_sc.name);
    frames0 = g_sc.frames_recv;
    consider0 = snap_now().consider_dec;
    CHECK(park_req(&op) == HOST_RADIO_PARK_STATUS_PARKED && op == OPMODE_SLEEP,
          "[%s] PARK_REQ: %02x (expect PARKED, 0x80)", g_sc.name, op);
    s0 = snap_now();
    /* The park cleared the chip's IRQ flags; raise RxDone again, as on a
     * chip that did not take the write, so the edge alone is what the
     * rx_service gate must refuse (the FIFO still holds the frame here). */
    g_radio.irq = (uint8_t)(IRQ_RX_DONE | IRQ_VALID_HEADER);
    fw_pass();
    CHECK(g_sc.frames_recv == frames0 && snap_now().consider_dec == consider0,
          "[%s] the frame latched before the park was delivered (%u) or its"
          " header considered (%u)", g_sc.name, g_sc.frames_recv - frames0,
          snap_now().consider_dec - consider0);

    /* 2 min LOCKED on the adopted grid: the slot follower's case. */
    not_asleep = run_parked_ms(120000U);
    /* 2 min LOCKED without a clock: the gamma-1 fallback's case. */
    sx1276_fhss_clock_reset();
    not_asleep += run_parked_ms(120000U);
    s1 = snap_now();
    CHECK(not_asleep == 0U, "[%s] %u parked passes ended out of SLEEP",
          g_sc.name, not_asleep);
    expect_untouched("4 min parked LOCKED, a stream on air", &s0, &s1);
    CHECK(g_scan_mirror == SX1276_RX_SCAN_STATE_LOCKED && g_sc.frames_sent > 500U,
          "[%s] scan state %u after the park (expect LOCKED, frozen); %u frames"
          " on air", g_sc.name, (unsigned)g_scan_mirror, g_sc.frames_sent);
    CHECK(park_req(&op) == HOST_RADIO_PARK_STATUS_ALREADY && op == OPMODE_SLEEP,
          "[%s] repeat PARK_REQ: %02x (expect ALREADY, 0x80)", g_sc.name, op);

    /* The park ends at a profile activation (CFG_SET REG_PROFILE=1). */
    {
        const uint32_t locks0 = g_sc.locks;
        const uint32_t frames1 = g_sc.frames_recv;
        activate_fhss();
        CHECK(!sx1276_rx_parked(), "[%s] still parked after the activation",
              g_sc.name);
        run_until_locks(locks0 + 1U, 150000U);
        run_for_ms(10000U);
        CHECK(g_sc.locks > locks0 && g_scan_mirror == SX1276_RX_SCAN_STATE_LOCKED &&
              g_sc.frames_recv >= frames1 + 15U,
              "[%s] no re-acquisition after the activation: locks %u -> %u,"
              " %u frames", g_sc.name, locks0, g_sc.locks,
              g_sc.frames_recv - frames1);
    }
    print_park();
    check_structure();
}

static void test_park_vs_tx(void) {
    uint8_t op = 0U;

    scenario_begin("P3 park against TX; raw writes; reset", ROLE_BASE, 0x5EED0003U);
    CHECK(!sx1276_rx_parked(), "[%s] parked at the start", g_sc.name);
    run_for_ms(3000U);
    CHECK(g_radio.mode == SX1276_STATE_RX_CONT, "[%s] setup: not in RX", g_sc.name);

    /* (a) A TX on air. Its FIFO is altered before TX_DONE: the 0x0D split. */
    {
        const uint32_t err0 = g_urc[HOST_TYPE_ERR_PROTO_URC].count;
        const uint32_t f0d = g_fault_n[0x0DU];
        const uint32_t f0f = g_fault_n[0x0FU];
        (void)tx_req(0x51U);
        CHECK(g_urc[HOST_TYPE_ERR_PROTO_URC].count == err0 && sx1276_tx_busy() &&
              g_radio.mode == SX1276_STATE_TX,
              "[%s] setup: the TX did not key", g_sc.name);
        g_radio.fifo[LORA_PKT_HDR_LEN + 3U] ^= 0xFFU;
        CHECK(park_req(&op) == HOST_RADIO_PARK_STATUS_REFUSED && op == OPMODE_TX &&
              !sx1276_rx_parked() && sx1276_tx_busy(),
              "[%s] PARK_REQ during a TX: %02x, parked %u (expect REFUSED, 0x83, 0)",
              g_sc.name, op, (unsigned)sx1276_rx_parked());
        run_until_tx_idle(1000U);
        CHECK(!sx1276_tx_busy() && g_radio.mode == SX1276_STATE_RX_CONT,
              "[%s] the post-TX re-arm did not run (modem state %u)", g_sc.name,
              (unsigned)g_radio.mode);
        CHECK(g_fault_n[0x0FU] == f0f + 1U && g_fault_sub[0x0FU] == 0x51U &&
              g_fault_n[0x0DU] == f0d,
              "[%s] FIFO readback mismatch: FAULT 0x0F x%u (sub %02x), 0x0D x%u"
              " (expect 0x0F once, sub 51, no 0x0D)", g_sc.name,
              g_fault_n[0x0FU] - f0f, g_fault_sub[0x0FU], g_fault_n[0x0DU] - f0d);
    }
    /* (b) A TX waiting in host_cmd.c's TX ring (its first TX anchored the
     * slot clock, so a frame that does not fit the slot waits). */
    {
        const uint64_t t0 = g_now_us;
        uint32_t ok0;
        uint32_t err0;
        while ((sx1276_tx_busy() || sx1276_tx_slot_wait_us(CMD_PAYLOAD_LEN) == 0U) &&
               g_now_us - t0 < 2000000ULL) {
            fw_pass();
        }
        ok0 = g_sc.fw_tx_ok;
        err0 = g_urc[HOST_TYPE_ERR_PROTO_URC].count;
        (void)tx_req(0x52U);
        CHECK(host_cmd_tx_mailbox_pending() && !sx1276_tx_busy(),
              "[%s] setup: the frame did not wait in the TX ring", g_sc.name);
        CHECK(park_req(&op) == HOST_RADIO_PARK_STATUS_REFUSED && !sx1276_rx_parked(),
              "[%s] PARK_REQ with a TX queued: %02x (expect REFUSED)", g_sc.name, op);
        run_until_tx_idle(2000U);
        CHECK(g_sc.fw_tx_ok == ok0 + 1U &&
              g_urc[HOST_TYPE_ERR_PROTO_URC].count == err0 &&
              g_radio.mode == SX1276_STATE_RX_CONT,
              "[%s] the queued frame did not go out (TX_DONE %u, ERR_PROTO %u)",
              g_sc.name, g_sc.fw_tx_ok - ok0,
              g_urc[HOST_TYPE_ERR_PROTO_URC].count - err0);
    }
    CHECK(park_req(&op) == HOST_RADIO_PARK_STATUS_PARKED && op == OPMODE_SLEEP,
          "[%s] PARK_REQ once idle: %02x (expect PARKED, 0x80)", g_sc.name, op);
    /* Refused at once, even when the frame would otherwise wait in the TX
     * ring for the slot (the clock is anchored and keeps running parked). */
    {
        const uint64_t t0 = g_now_us;
        while (sx1276_tx_slot_wait_us(CMD_PAYLOAD_LEN) == 0U &&
               g_now_us - t0 < 2000000ULL) {
            fw_pass();
        }
        CHECK(sx1276_tx_slot_wait_us(CMD_PAYLOAD_LEN) > 0U,
              "[%s] setup: no slot wait to test the ring against", g_sc.name);
    }
    expect_tx_refused(0x53U);

    /* Raw RegOpMode writes that do not end the park. */
    reg_write_refused(0x01U, 0x86U);   /* RXSINGLE: diag builds only */
    CHECK(sx1276_rx_parked() && g_radio.mode == SX1276_STATE_SLEEP,
          "[%s] a refused raw RXSINGLE ended the park", g_sc.name);
    {
        snap_t a;
        snap_t b;
        uint32_t not_standby = 0U;
        const uint64_t until = g_now_us + 60000000ULL;
        reg_write_ok(0x01U, 0x81U);    /* STANDBY */
        CHECK(sx1276_rx_parked() && g_radio.mode == SX1276_STATE_STANDBY,
              "[%s] raw STANDBY: parked %u, modem state %u", g_sc.name,
              (unsigned)sx1276_rx_parked(), (unsigned)g_radio.mode);
        a = snap_now();
        while (g_now_us < until) {
            fw_pass();
            if (g_radio.mode != SX1276_STATE_STANDBY) {
                ++not_standby;
            }
        }
        b = snap_now();
        CHECK(not_standby == 0U && b.rx_cont == a.rx_cont && b.freq == a.freq &&
              b.tx == a.tx && b.scan_rec == a.scan_rec,
              "[%s] parked in STANDBY for 60 s: %u passes out of STANDBY, RX arms"
              " %u, retunes %u, scan records %u", g_sc.name, not_standby,
              b.rx_cont - a.rx_cont, b.freq - a.freq, b.scan_rec - a.scan_rec);
        CHECK(park_req(&op) == HOST_RADIO_PARK_STATUS_ALREADY && op == OPMODE_SLEEP &&
              g_radio.mode == SX1276_STATE_SLEEP,
              "[%s] PARK_REQ from STANDBY: %02x (expect ALREADY, 0x80)", g_sc.name, op);
    }
    reg_write_ok(0x01U, 0x80U);        /* SLEEP: radio_park.py's write */
    CHECK(sx1276_rx_parked() && g_radio.mode == SX1276_STATE_SLEEP,
          "[%s] raw SLEEP ended the park", g_sc.name);
    {
        const uint32_t r0 = g_system_resets;
        (void)host_req(HOST_TYPE_RESET_REQ, NULL, 0U);
        CHECK(g_system_resets == r0 + 1U,
              "[%s] RESET_REQ did not reach platform_system_reset()", g_sc.name);
    }

    /* The park ends at the raw RXCONT, and TX works again. */
    reg_write_ok(0x01U, 0x85U);
    CHECK(!sx1276_rx_parked() && g_radio.mode == SX1276_STATE_RX_CONT,
          "[%s] raw RXCONT: parked %u, modem state %u", g_sc.name,
          (unsigned)sx1276_rx_parked(), (unsigned)g_radio.mode);
    {
        const uint32_t ok0 = g_sc.fw_tx_ok;
        const uint32_t err0 = g_urc[HOST_TYPE_ERR_PROTO_URC].count;
        (void)tx_req(0x54U);
        run_until_tx_idle(2000U);
        CHECK(g_sc.fw_tx_ok == ok0 + 1U &&
              g_urc[HOST_TYPE_ERR_PROTO_URC].count == err0 &&
              g_radio.mode == SX1276_STATE_RX_CONT,
              "[%s] TX after the park: TX_DONE %u, ERR_PROTO %u, modem state %u",
              g_sc.name, g_sc.fw_tx_ok - ok0,
              g_urc[HOST_TYPE_ERR_PROTO_URC].count - err0, (unsigned)g_radio.mode);
    }
    print_park();
    check_structure();
}

static void test_tractor_park_in_failed(void) {
    static const uint8_t trains[4] = { 23U, 45U, 23U, 9U };   /* leg H mix */
    uint8_t op = 0U;
    uint32_t restarts0;
    uint32_t not_asleep;
    snap_t s0;
    snap_t s1;

    scenario_begin("P4 tractor: FAILED held, daemon stops, park", ROLE_TRACTOR,
                   0x789ABCDU);
    CHECK(!sx1276_rx_parked(), "[%s] parked at the start", g_sc.name);
    g_th.period_ms = 500U;
    g_th.jitter_ms = 40U;
    memcpy(g_th.train_w, trains, 4U);
    g_th.next_frame_us = g_now_us + 50000ULL;
    g_th.next_rxcont_us = g_now_us;
    base_model_init();
    g_base.mode = CMD_PUMP_AT_TRAIN_END;
    g_base.min_gap_ms = 1000U;
    g_base.next_allowed_us = UINT64_MAX;   /* the base never commands */
    run_for_ms(150000U);
    CHECK(g_scan_mirror == SX1276_RX_SCAN_STATE_FAILED && g_sc.fail_final == 1U &&
          auth_now_ms(platform_now_ms()) != 0U,
          "[%s] setup: scan state %u, final FAILs %u, authority %u (expect FAILED"
          " held while streaming)", g_sc.name, (unsigned)g_scan_mirror,
          g_sc.fail_final, auth_now_ms(platform_now_ms()));

    /* The camera daemon stops right after a TX_DONE; its train drains. */
    {
        const uint32_t ok0 = g_sc.fw_tx_ok;
        const uint64_t t0 = g_now_us;
        while (g_sc.fw_tx_ok == ok0 && g_now_us - t0 < 2000000ULL) {
            fw_pass();
        }
        g_th.next_frame_us = UINT64_MAX;
        g_th.frames_waiting = 0U;
        while ((g_th.train_active != 0U || g_th.outstanding != 0U ||
                sx1276_tx_busy() || g_txq_count != 0U) &&
               g_now_us - t0 < 5000000ULL) {
            fw_pass();
        }
        g_th.next_rxcont_us = UINT64_MAX;  /* and with it its raw RXCONT */
    }
    CHECK(g_scan_mirror == SX1276_RX_SCAN_STATE_FAILED &&
          auth_now_ms(platform_now_ms()) != 0U && !sx1276_tx_busy(),
          "[%s] setup: authority lost before the park (scan state %u)", g_sc.name,
          (unsigned)g_scan_mirror);
    CHECK(park_req(&op) == HOST_RADIO_PARK_STATUS_PARKED && op == OPMODE_SLEEP,
          "[%s] PARK_REQ: %02x (expect PARKED, 0x80)", g_sc.name, op);
    restarts0 = g_sc.tick_restarts;
    s0 = snap_now();
    not_asleep = run_parked_ms(300000U);
    s1 = snap_now();
    CHECK(not_asleep == 0U, "[%s] %u parked passes ended out of SLEEP",
          g_sc.name, not_asleep);
    expect_untouched("5 min parked in FAILED, authority decayed", &s0, &s1);
    CHECK(g_sc.tick_restarts == restarts0 &&
          g_scan_mirror == SX1276_RX_SCAN_STATE_FAILED &&
          auth_now_ms(platform_now_ms()) == 0U,
          "[%s] parked: (b) restarts %u, scan state %u, authority %u (expect 0,"
          " FAILED, 0)", g_sc.name, g_sc.tick_restarts - restarts0,
          (unsigned)g_scan_mirror, auth_now_ms(platform_now_ms()));

    /* The daemon restarts: its raw RXCONT ends the park, and the first scan
     * tick restarts the scan (b). */
    reg_write_ok(0x01U, 0x85U);
    CHECK(!sx1276_rx_parked() && g_radio.mode == SX1276_STATE_RX_CONT,
          "[%s] raw RXCONT: parked %u, modem state %u", g_sc.name,
          (unsigned)sx1276_rx_parked(), (unsigned)g_radio.mode);
    {
        const snap_t a = snap_now();
        fw_pass();
        {
            const snap_t b = snap_now();
            CHECK(g_sc.tick_restarts == restarts0 + 1U &&
                  g_scan_mirror == SX1276_RX_SCAN_STATE_SCANNING &&
                  b.freq == a.freq + 1U && b.faults == a.faults &&
                  g_radio.mode == SX1276_STATE_RX_CONT,
                  "[%s] first tick after the park: (b) restarts %u, scan state %u,"
                  " walker retunes %u, faults %u (expect 1, SCANNING, 1, 0)",
                  g_sc.name, g_sc.tick_restarts - restarts0,
                  (unsigned)g_scan_mirror, b.freq - a.freq, b.faults - a.faults);
        }
    }
    /* And the camera streams again. */
    {
        const uint32_t ok0 = g_sc.fw_tx_ok;
        g_th.next_frame_us = g_now_us + 50000ULL;
        g_th.next_rxcont_us = g_now_us;
        run_for_ms(20000U);
        CHECK(g_sc.fw_tx_ok >= ok0 + 20U && auth_now_ms(platform_now_ms()) != 0U,
              "[%s] after the park: %u TX_DONE, authority %u", g_sc.name,
              g_sc.fw_tx_ok - ok0, auth_now_ms(platform_now_ms()));
    }
    print_park();
    check_structure();
}

/* Run until the scan walker retunes (a BEGIN_SCAN / ADVANCE_CHANNEL step);
 * return its pass's ms, or UINT32_MAX at the cap. */
static uint32_t run_until_walker_step(uint32_t cap_ms) {
    const uint32_t steps0 = g_sc.walker_retunes;
    const uint64_t t0 = g_now_us;
    while (g_sc.walker_retunes == steps0 &&
           g_now_us - t0 < (uint64_t)cap_ms * 1000ULL) {
        fw_pass();
    }
    return (g_sc.walker_retunes == steps0) ? UINT32_MAX : g_pass_now_ms;
}

static void test_unpark_resume(void) {
    uint8_t op = 0U;

    scenario_begin("P5 raw-RXCONT unpark: SCANNING resumes, LOCKED demotes",
                   ROLE_BASE, 0x5EED0005U);
    CHECK(!sx1276_rx_parked(), "[%s] parked at the start", g_sc.name);

    /* (a) SCANNING, nothing on air: park 700 ms into a dwell, for 10 s. */
    {
        const uint32_t step_ms = run_until_walker_step(5000U);
        uint32_t park_ms;
        uint32_t unpark_ms;
        uint32_t due;
        uint32_t next_ms;
        uint32_t steps0;
        CHECK(step_ms != UINT32_MAX && g_scan_mirror == SX1276_RX_SCAN_STATE_SCANNING,
              "[%s] setup: no walker step (scan state %u)", g_sc.name,
              (unsigned)g_scan_mirror);
        run_for_ms(700U);
        park_ms = platform_now_ms();
        CHECK(park_req(&op) == HOST_RADIO_PARK_STATUS_PARKED && op == OPMODE_SLEEP,
              "[%s] PARK_REQ while SCANNING: %02x (expect PARKED, 0x80)", g_sc.name,
              op);
        steps0 = g_sc.walker_retunes;
        CHECK(run_parked_ms(10000U) == 0U && g_sc.walker_retunes == steps0,
              "[%s] parked SCANNING: out of SLEEP, or %u walker steps", g_sc.name,
              g_sc.walker_retunes - steps0);
        unpark_ms = platform_now_ms();
        due = step_ms + SX1276_RX_SCAN_DWELL_MS + (unpark_ms - park_ms);
        reg_write_ok(0x01U, 0x85U);
        CHECK(!sx1276_rx_parked() && g_radio.mode == SX1276_STATE_RX_CONT,
              "[%s] raw RXCONT: parked %u, modem state %u", g_sc.name,
              (unsigned)sx1276_rx_parked(), (unsigned)g_radio.mode);
        next_ms = run_until_walker_step(5000U);
        CHECK(next_ms != UINT32_MAX && next_ms >= due && next_ms <= due + 2U,
              "[%s] the dwell the park interrupted ended at %u ms, %u ms after"
              " the unpark; expect %u ms: its start + %u ms dwell + %u ms parked"
              " (the remaining %u ms after the unpark)", g_sc.name, next_ms,
              next_ms - unpark_ms, due, SX1276_RX_SCAN_DWELL_MS,
              unpark_ms - park_ms, due - unpark_ms);
    }

    /* (b) LOCKED on an adopted grid, the stream on air: park for 10 s. */
    {
        uint32_t locks0;
        uint32_t dem0;
        uint32_t not_asleep;
        orig_on(g_now_us + 37000ULL, 2468U);
        run_until_locks(g_sc.locks + 1U, 150000U);
        run_for_ms(10000U);
        CHECK(g_scan_mirror == SX1276_RX_SCAN_STATE_LOCKED &&
              sx1276_rx_grid_adopted() != 0U && sx1276_fhss_clock_valid() != 0U,
              "[%s] setup: scan state %u, adopted %u, clock %u", g_sc.name,
              (unsigned)g_scan_mirror, sx1276_rx_grid_adopted(),
              sx1276_fhss_clock_valid());
        CHECK(park_req(&op) == HOST_RADIO_PARK_STATUS_PARKED && op == OPMODE_SLEEP,
              "[%s] PARK_REQ while LOCKED: %02x (expect PARKED, 0x80)", g_sc.name,
              op);
        dem0 = g_sc.demotions;
        not_asleep = run_parked_ms(10000U);
        CHECK(not_asleep == 0U && g_sc.demotions == dem0 &&
              g_scan_mirror == SX1276_RX_SCAN_STATE_LOCKED,
              "[%s] parked LOCKED: %u passes out of SLEEP, %u demotions, scan"
              " state %u (expect 0, 0, LOCKED: frozen)", g_sc.name, not_asleep,
              g_sc.demotions - dem0, (unsigned)g_scan_mirror);
        /* F6: the clock is stale -- and so must the lock be. */
        CHECK(sx1276_fhss_clock_valid() != 0U &&
              sx1276_fhss_clock_age_ms(platform_now_ms()) > SX1276_FHSS_CLOCK_FRESH_MS,
              "[%s] the adopted clock is fresh after 10 s parked", g_sc.name);
        reg_write_ok(0x01U, 0x85U);
        CHECK(!sx1276_rx_parked() && g_radio.mode == SX1276_STATE_RX_CONT,
              "[%s] raw RXCONT: parked %u, modem state %u", g_sc.name,
              (unsigned)sx1276_rx_parked(), (unsigned)g_radio.mode);
        fw_pass();
        CHECK(g_sc.demotions == dem0 + 1U &&
              g_scan_mirror == SX1276_RX_SCAN_STATE_SCANNING &&
              sx1276_rx_grid_adopted() == 0U,
              "[%s] first tick after a 10 s LOCKED park: %u demotions, scan state"
              " %u, adopted %u (expect 1, SCANNING, 0: the loss timer counts the"
              " park, as the clock's freshness does)", g_sc.name,
              g_sc.demotions - dem0, (unsigned)g_scan_mirror,
              sx1276_rx_grid_adopted());
        locks0 = g_sc.locks;
        run_until_locks(locks0 + 1U, 150000U);
        run_for_ms(5000U);
        CHECK(g_sc.locks > locks0 && g_scan_mirror == SX1276_RX_SCAN_STATE_LOCKED,
              "[%s] no re-acquisition after the LOCKED park: locks %u -> %u",
              g_sc.name, locks0, g_sc.locks);
    }
    print_park();
    check_structure();
}

int main(int argc, char **argv) {
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-v") == 0) {
            g_verbose = 1;
        }
    }
    test_boot();
    test_park_after_final_fail();
    test_park_while_locked();
    test_park_vs_tx();
    test_tractor_park_in_failed();
    test_unpark_resume();
    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] radio_park: %d failure(s)\n", g_failures);
        return 1;
    }
    printf("[PASS] radio_park: RS-12.21 firmware park -- PARK_REQ/URC through the"
           " real host_cmd.c; parked after a final FAIL, while LOCKED and in a"
           " held FAILED, nothing re-arms or touches the modem for minutes; TX"
           " refused while parked, park refused while a TX is keyed or queued;"
           " ended by a raw RXCONT, a profile activation (and a reset); after"
           " it a SCANNING node resumes its dwell, a LOCKED one parked past"
           " the loss timer demotes at once; FIFO fault on 0x0F\n");
    return 0;
}
