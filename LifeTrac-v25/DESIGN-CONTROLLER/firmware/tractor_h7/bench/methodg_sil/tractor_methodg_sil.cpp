// H7-4 software-in-the-loop test: the real tractor_h7.ino control
// application, built for Method G, driven through a fake L072 on Serial3
// and a fake Opta on ModbusRTUClient. Host only; see run.sh.
//
// Covers: RX_FRAME_URC -> decrypt -> arbitration -> Modbus holding block,
// E-stop latch and clear, link-loss neutral, telemetry out as
// TX_FRAME_REQ with one frame at the L072 at a time, and every gate release
// (TX_DONE, ERR_PROTO, BOOT_URC, timeout; FAULT_URC releases nothing), and
// H7-13: fresh RNG nonce tails, and no TX at all when the RNG is stuck.

#include <stdio.h>
#include <sys/mman.h>

#include "Arduino.h"
#include "ArduinoModbus.h"
#include "ArduinoRS485.h"

static uint32_t g_now_ms = 1;
uint32_t millis() { return g_now_ms; }
uint32_t micros() { return g_now_ms * 1000u; }
void delay(unsigned long ms) { g_now_ms += (uint32_t)ms; }
long random(long lo, long) { return lo; }

// H7-13: stand-in for the H747 RNG. g_entropy_stuck makes it repeat one
// word, which lt_entropy.c must catch.
static uint32_t g_entropy_state = 0x12345678u;
static bool g_entropy_stuck = false;
extern "C" bool lifetrac_host_entropy(uint8_t* out, size_t len, void*) {
    for (size_t i = 0; i < len; i++) {
        if (!g_entropy_stuck) g_entropy_state = g_entropy_state * 1664525u + 1013904223u;
        out[i] = g_entropy_stuck ? 0xA5 : (uint8_t)(g_entropy_state >> 24);
    }
    return true;
}

HardwareSerial Serial;
HardwareSerial Serial1;
HardwareSerial Serial3;
FakeRS485 RS485;
FakeModbusRTUClient ModbusRTUClient;

#include "../../tractor_h7.ino"

#include "murata_host/mh_cobs.h"
#include "murata_host/murata_host.h"

static int g_failures;
static void expect(bool cond, const char* msg) {
    if (!cond) {
        fprintf(stderr, "[FAIL] t=%u ms: %s\n", (unsigned)g_now_ms, msg);
        g_failures++;
    }
}

// ---------- fake L072 ----------
static void l072_send(uint8_t type, uint16_t seq, const uint8_t* payload, uint16_t len) {
    uint8_t inner[HOST_MAX_INNER_LEN];
    uint8_t enc[HOST_MAX_INNER_LEN + 8];
    uint16_t n = murata_host_build_inner_frame(type, 0, seq, payload, len, inner, sizeof(inner));
    size_t e = mh_cobs_encode(inner, n, enc, sizeof(enc));
    enc[e++] = 0;
    Serial3.inject(enc, e);
}

static void l072_rx_packet(const uint8_t* pkt, uint8_t len) {
    uint8_t p[8 + 255];
    p[0] = len;
    p[1] = (uint8_t)(int8_t)6;      // SNR dB
    p[2] = (uint8_t)(-80 & 0xFF);   // RSSI dBm
    p[3] = (uint8_t)((-80 >> 8) & 0xFF);
    memset(&p[4], 0, 4);
    memcpy(&p[8], pkt, len);
    l072_send(HOST_TYPE_RX_FRAME_URC, 0, p, (uint16_t)(8 + len));
}

struct TxReq { uint8_t tx_id; uint16_t seq; uint8_t len; uint8_t data[255]; };
static size_t g_tx_scan;
static uint32_t g_tx_commands;   // FT_COMMAND frames the tractor sent
static uint8_t g_last_tails[2][5];
static uint32_t g_tails_seen;
static uint32_t g_tail_repeats;  // a nonce tail equal to the previous frame's

static void classify_tx(const TxReq& r) {
    KissDecoder dec;
    memset(&dec, 0, sizeof(dec));
    for (uint8_t i = 0; i < r.len; i++) {
        uint8_t* frame; size_t flen;
        if (lp_kiss_feed(&dec, r.data[i], &frame, &flen) &&
            flen >= 12 + sizeof(LoraHeader)) {
            if (((const LoraHeader*)(frame + 12))->frame_type == FT_COMMAND) g_tx_commands++;
            memcpy(g_last_tails[g_tails_seen & 1u], frame + 7, 5);
            if (g_tails_seen > 0 &&
                memcmp(g_last_tails[0], g_last_tails[1], 5) == 0) g_tail_repeats++;
            g_tails_seen++;
        }
    }
}

// Every TX_FRAME_REQ the sketch wrote since the last call.
static int take_tx_reqs(TxReq* out, int max) {
    int count = 0;
    size_t start = g_tx_scan;
    for (size_t i = g_tx_scan; i < Serial3.tx_len; i++) {
        if (Serial3.tx[i] != 0) continue;
        uint8_t inner[HOST_MAX_INNER_LEN];
        size_t n = mh_cobs_decode(&Serial3.tx[start], i - start, inner, sizeof(inner));
        murata_host_frame_t f;
        if (n > 0 && murata_host_parse_inner_frame(inner, (uint16_t)n, &f) &&
            f.type == HOST_TYPE_TX_FRAME_REQ && count < max) {
            out[count].tx_id = f.payload[0];
            out[count].seq = f.seq;
            out[count].len = f.payload[1];
            memcpy(out[count].data, &f.payload[2], f.payload[1]);
            classify_tx(out[count]);
            count++;
        }
        start = i + 1;
    }
    g_tx_scan = start;
    return count;
}

// ---------- air frames from a peer (stub crypto: tag is zeros) ----------
static uint16_t g_seq[5];

static void send_air(uint8_t src, const uint8_t* pt, size_t pt_len) {
    uint8_t nonce[12] = {0};
    nonce[0] = src;
    uint8_t onair[12 + 64 + 16];
    memcpy(onair, nonce, 12);
    lp_encrypt(kFleetKey, nonce, pt, pt_len, onair + 12);
    uint8_t kiss[2 * sizeof(onair) + 4];
    size_t kl = lp_kiss_encode(onair, 12 + pt_len + 16, kiss, sizeof(kiss));
    l072_rx_packet(kiss, (uint8_t)kl);
}

static void send_heartbeat(uint8_t src) {
    HeartbeatFrame hb;
    memset(&hb, 0, sizeof(hb));
    hb.hdr.version = LIFETRAC_PROTO_VERSION;
    hb.hdr.source_id = src;
    hb.hdr.frame_type = FT_HEARTBEAT;
    hb.hdr.sequence_num = ++g_seq[src];
    hb.priority_request = src;
    hb.crc16 = lp_crc16((const uint8_t*)&hb, sizeof(hb) - 2);
    send_air(src, (const uint8_t*)&hb, sizeof(hb));
}

static void send_control(uint8_t src, int8_t lhy, uint16_t buttons) {
    ControlFrame cf;
    memset(&cf, 0, sizeof(cf));
    cf.hdr.version = LIFETRAC_PROTO_VERSION;
    cf.hdr.source_id = src;
    cf.hdr.frame_type = FT_CONTROL;
    cf.hdr.sequence_num = ++g_seq[src];
    cf.axis_lh_y = lhy;
    cf.buttons = buttons;
    cf.crc16 = lp_crc16((const uint8_t*)&cf, sizeof(cf) - 2);
    send_air(src, (const uint8_t*)&cf, sizeof(cf));
}

static void send_command(uint8_t src, uint8_t opcode) {
    uint8_t pt[8];
    LoraHeader hdr = { LIFETRAC_PROTO_VERSION, src, FT_COMMAND, ++g_seq[src] };
    memcpy(pt, &hdr, sizeof(hdr));
    pt[5] = opcode;
    uint16_t crc = lp_crc16(pt, 6);
    pt[6] = (uint8_t)crc;
    pt[7] = (uint8_t)(crc >> 8);
    send_air(src, pt, sizeof(pt));
}

static void run_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        loop();
        g_now_ms++;
    }
}

// Drive one handheld stick input for `ms`, HB + control every 50 ms.
static void drive(int8_t lhy, uint32_t ms) {
    for (uint32_t t = 0; t < ms; t += 50) {
        send_heartbeat(SRC_HANDHELD);
        send_control(SRC_HANDHELD, lhy, 0);
        run_ms(50);
    }
}

// Acknowledge every TX_FRAME_REQ until the sketch has nothing left queued.
static void ack_all() {
    TxReq tx[16];
    for (int round = 0; round < 32; round++) {
        run_ms(5);
        int n = take_tx_reqs(tx, 16);
        if (n == 0 && !mh_runtime_tx_busy()) return;
        for (int i = 0; i < n; i++) {
            const uint8_t done[7] = { tx[i].tx_id, 0, 0, 0, 0, 0, 2 };
            l072_send(HOST_TYPE_TX_DONE_URC, 0, done, 7);
        }
    }
}

static uint16_t reg(int r) { return ModbusRTUClient.regs[r]; }

int main() {
    // The sketch writes the M4 mailbox at its fixed SRAM4 address.
    void* shared = mmap((void*)(uintptr_t)LIFETRAC_SHARED_ADDR, 4096,
                        PROT_READ | PROT_WRITE,
                        MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
    if (shared != (void*)(uintptr_t)LIFETRAC_SHARED_ADDR) {
        fprintf(stderr, "cannot map the shared mailbox at 0x%08x\n", LIFETRAC_SHARED_ADDR);
        return 2;
    }

    setup();
    expect(Serial3.opened && Serial3.baud == 921600u, "Method G opens the Murata UART at 921600");
    expect(!Serial1.opened, "X8 link stays off by default in a Method G build");

    // 1. No source: neutral, watchdog counter advancing at 20 Hz.
    run_ms(200);
    uint32_t w0 = ModbusRTUClient.writes;
    uint16_t wd0 = reg(REG_WATCHDOG_CTR);
    run_ms(500);
    expect(ModbusRTUClient.writes - w0 >= 9, "Modbus holding block written at ~20 Hz");
    expect((uint16_t)(reg(REG_WATCHDOG_CTR) - wd0) >= 9, "Opta watchdog counter advances");
    expect(reg(REG_CMD_SOURCE) == 0 && reg(REG_VALVE_COILS) == 0, "neutral with no source");
    expect(SHARED->loop_counter > 0 && SHARED->alive_tick_ms != 0, "M4 mailbox stamped every loop");

    // 2. Handheld forward stick over Method G -> both forward drive coils.
    drive(100, 400);
    expect(reg(REG_CMD_SOURCE) == 1, "handheld is the active source");
    expect((reg(REG_VALVE_COILS) & ((1u << VB_DRIVE_LF) | (1u << VB_DRIVE_RF))) ==
               ((1u << VB_DRIVE_LF) | (1u << VB_DRIVE_RF)),
           "forward stick energises LF + RF drive coils");
    expect(reg(REG_FLOW_SP_1) > 0, "flow set-point follows the stick");

    // 3. Link loss: 300 ms with nothing -> neutral (CONTROL_TIMEOUT_MS 200).
    run_ms(300);
    expect(reg(REG_CMD_SOURCE) == 0 && reg(REG_VALVE_COILS) == 0 && reg(REG_FLOW_SP_1) == 0,
           "stale link drops to neutral");

    // 4. E-stop latches over Method G, survives control frames, and only
    //    the base may clear it.
    drive(100, 200);
    send_command(SRC_HANDHELD, CMD_ESTOP);
    run_ms(2);   // processed in loop N, published to the M4 at the top of N+1
    expect(reg(REG_ARM_ESTOP) == 1 && reg(REG_VALVE_COILS) == 0, "CMD_ESTOP applies neutral at once");
    expect(SHARED->estop_request == LIFETRAC_ESTOP_MAGIC, "E-stop published to the M4");
    drive(100, 200);
    expect(reg(REG_ARM_ESTOP) == 1 && reg(REG_VALVE_COILS) == 0, "latched E-stop ignores the stick");
    send_command(SRC_HANDHELD, CMD_CLEAR_ESTOP);
    drive(100, 100);
    expect(reg(REG_ARM_ESTOP) == 1, "handheld cannot clear the E-stop");
    send_command(SRC_BASE, CMD_CLEAR_ESTOP);
    drive(100, 200);
    expect(reg(REG_ARM_ESTOP) == 0 && reg(REG_CMD_SOURCE) == 1, "base clears the E-stop");
    expect(SHARED->estop_request == 0, "cleared E-stop withdrawn from the M4");

    // 5. Telemetry goes out as TX_FRAME_REQ, one at a time at the L072.
    TxReq tx[16];
    ack_all();
    run_ms(1000);                           // next 1 Hz burst: 2 frames queued
    int n = take_tx_reqs(tx, 16);
    expect(n == 1, "only one TX_FRAME_REQ outstanding at the L072");
    if (n >= 1) {
        // Packet = KISS(nonce | ct | tag); stub crypto leaves pt readable.
        KissDecoder dec;
        memset(&dec, 0, sizeof(dec));
        uint8_t* frame = nullptr; size_t flen = 0; bool got = false;
        for (uint8_t i = 0; i < tx[0].len && !got; i++) {
            got = lp_kiss_feed(&dec, tx[0].data[i], &frame, &flen);
        }
        expect(got && flen > 12 + sizeof(LoraHeader) + 16, "TX packet is one KISS frame");
        if (got) {
            const LoraHeader* h = (const LoraHeader*)(frame + 12);
            expect(h->source_id == SRC_TRACTOR && h->frame_type == FT_TELEMETRY,
                   "TX packet is tractor telemetry");
        }
        // FAULT_URC is diagnostic: still held.
        const uint8_t fault[2] = { 0x08, 0x01 };
        l072_send(HOST_TYPE_FAULT_URC, 0, fault, 2);
        run_ms(5);
        expect(take_tx_reqs(tx + 1, 15) == 0, "FAULT_URC releases nothing");
        // TX_DONE for our tx_id releases it; the second frame follows.
        const uint8_t done[7] = { tx[0].tx_id, 0, 0, 0, 0, 0, 2 };
        l072_send(HOST_TYPE_TX_DONE_URC, 0, done, 7);
        run_ms(5);
        n = take_tx_reqs(tx + 1, 15);
        expect(n == 1, "TX_DONE releases the gate for the next frame");
        if (n == 1) {
            expect(tx[1].tx_id != tx[0].tx_id && tx[1].seq != tx[0].seq,
                   "next frame has a fresh tx_id and seq");
            // ERR_PROTO for that seq releases too.
            const uint8_t err[5] = { HOST_TYPE_TX_FRAME_REQ, 1, 0x06, 0, 0 };
            l072_send(HOST_TYPE_ERR_PROTO_URC, tx[1].seq, err, 5);
            run_ms(5);
            mh_link_gate_t g;
            mh_runtime_get_tx_gate(&g);
            expect(g.rejected == 1, "ERR_PROTO for our seq releases the gate");
        }
    }

    // 6. BOOT_URC releases a frame in flight.
    ack_all();
    run_ms(1000);                           // next burst
    n = take_tx_reqs(tx, 16);
    expect(n == 1 && mh_runtime_tx_busy(), "a telemetry frame is in flight");
    const uint8_t boot[6] = { 1, 1, 0x12, 1, 2, 0 };
    l072_send(HOST_TYPE_BOOT_URC, 0, boot, 6);
    run_ms(2);
    expect(take_tx_reqs(tx, 16) == 1, "BOOT_URC releases the gate for the next frame");

    // 7. A lost URC: the gate times out (ToA ceiling at SF9 + one slot)
    //    and the next frame goes out without any URC at all.
    {
        mh_link_gate_t before, after;
        mh_runtime_get_tx_gate(&before);
        run_ms(1500);
        mh_runtime_get_tx_gate(&after);
        expect(after.timeouts > before.timeouts, "unanswered TX_FRAME_REQ times out");
        expect(take_tx_reqs(tx, 16) >= 1, "TX continues after a timeout");
    }

    // 8. No source for 16 s (3+ bad ladder windows): over Method G the SF
    //    ladder stays idle, so no CMD_LINK_TUNE is ever announced.
    for (int i = 0; i < 320; i++) ack_all(), run_ms(50);
    expect(g_tx_commands == 0 && g_ladder_rung == 0, "no CMD_LINK_TUNE over Method G");

    expect(g_tails_seen > 10 && g_tail_repeats == 0,
           "every TX nonce tail comes fresh from the RNG");

    // 9. Modbus outage latches the E-stop (IP-205, 10 consecutive failures).
    ModbusRTUClient.fail = true;
    run_ms(600);
    ModbusRTUClient.fail = false;
    run_ms(60);
    expect(reg(REG_ARM_ESTOP) == 1, "10 failed Modbus writes latch the E-stop");

    // 10. A stuck RNG (H7-13): the frame is dropped, never sent with a
    //     guessable nonce; control keeps running.
    //     Bytes drawn into the pool before the fault may still go out.
    g_entropy_stuck = true;
    for (int i = 0; i < 100; i++) ack_all(), run_ms(50);
    ack_all();
    const uint32_t loops0 = SHARED->loop_counter;
    const uint32_t writes0 = ModbusRTUClient.writes;
    const uint32_t wd0_rng = reg(REG_WATCHDOG_CTR);
    run_ms(2500);
    expect(take_tx_reqs(tx, 16) == 0, "stuck RNG suppresses tractor TX");
    expect(g_entropy.failed && g_nonce_fail > 0, "stuck RNG is latched and counted");
    expect(SHARED->loop_counter - loops0 >= 2400 && SHARED->alive_tick_ms == g_now_ms - 1,
           "M4 mailbox keeps being stamped after the RNG fault");
    expect(ModbusRTUClient.writes - writes0 >= 45 && (uint16_t)(reg(REG_WATCHDOG_CTR) - wd0_rng) >= 45,
           "Modbus block and Opta watchdog keep running at 20 Hz after the RNG fault");

    if (g_failures) {
        fprintf(stderr, "tractor_methodg_sil: %d failure(s)\n", g_failures);
        return 1;
    }
    printf("tractor_methodg_sil: all checks passed\n");
    return 0;
}
