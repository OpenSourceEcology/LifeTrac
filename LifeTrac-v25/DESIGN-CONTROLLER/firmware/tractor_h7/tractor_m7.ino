// tractor_m7.ino — Portenta H747 M7 core: radio + arbitration + Modbus master.
//
// DRAFT FOR REVIEW. Not compiled or tested.
//
// Runs on the M7 core whether the H747 lives in a standalone Portenta H7 or
// inside a Portenta X8 (binary-compatible — see ARCHITECTURE.md).
//
// Responsibilities:
//   1. Receive LoRa frames from handheld + base + (future) autonomy.
//   2. Validate CRC, decrypt, check sequence (replay protection).
//   3. Run multi-source arbitration loop @ 50 Hz (per LORA_PROTOCOL.md).
//   4. Write the active source's ControlFrame into the Opta over Modbus RTU.
//   5. Tick the Opta watchdog "alive" register at >=10 Hz.
//   6. Read Opta telemetry and emit TelemetryFrames over LoRa @ 1 Hz.
//
// What is NOT here yet (deferred):
//   - Cellular failover, MQTT-SN topic dispatch, video thumbnails.
//   - Per-coil current monitoring, CAN-bus engine telemetry.

#include <Arduino.h>
#include <RadioLib.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>
#include "../common/lora_proto/lora_proto.h"

// ---------- LoRa pins (Portenta Max Carrier wires Murata SiP via SPI) ----------
SX1276 radio = new Module(/*nss*/ PD_4, /*dio0*/ PD_5,
                          /*reset*/ PE_4, /*dio1*/ PE_5);

// ---------- Modbus master over RS-485 (Max Carrier J6) ----------
// 115200 8N1, RTU framing. Opta is slave id 0x01 (per TRACTOR_NODE.md).
#define OPTA_SLAVE_ID 0x01

// Opta register map — SOURCE OF TRUTH is TRACTOR_NODE.md § Modbus RTU register map.
// Holding registers (master writes):
//   0x0000 valve_coils      uint16 bitfield, 8 bits used
//   0x0001 flow_setpoint_1  uint16 (0..10000)
//   0x0002 flow_setpoint_2  uint16 (0..10000)
//   0x0003 aux_outputs      uint16 bitfield (R10-R12 + engine-kill arm)
//   0x0004 watchdog_counter uint16 — must increment >=10 Hz
//   0x0005 command_source   uint16 enum (logging only)
//   0x0006 arm_engine_kill  uint16 non-zero = energize engine-kill solenoid
// Input registers (slave writes, master reads):
//   0x0100 safety_state     uint16 enum
//   0x0101 digital_inputs   uint16 bitfield
//   0x0102..0x0107 analog telemetry block (battery, hyd, coolant, oil, aux)
//   0x0108 relay_fault_flags
//   0x0109..0x010A opta_uptime_s (uint32)
//   0x010B opta_fw_version
enum OptaHolding : uint16_t {
    REG_VALVE_COILS    = 0x0000,
    REG_FLOW_SP_1      = 0x0001,
    REG_FLOW_SP_2      = 0x0002,
    REG_AUX_OUTPUTS    = 0x0003,
    REG_WATCHDOG_CTR   = 0x0004,
    REG_CMD_SOURCE     = 0x0005,
    REG_ARM_ESTOP      = 0x0006,
    HOLDING_BLOCK_LEN  = 7,
};
enum OptaInput : uint16_t {
    REG_SAFETY_STATE   = 0x0100,
    REG_DIGITAL_INPUTS = 0x0101,
    REG_BATTERY_MV     = 0x0102,
    REG_HYD_SUPPLY     = 0x0103,
    REG_HYD_RETURN     = 0x0104,
    REG_COOLANT_C      = 0x0105,
    REG_OIL_C          = 0x0106,
    REG_AUX_ANALOG     = 0x0107,
    REG_RELAY_FAULTS   = 0x0108,
    INPUT_BLOCK_LEN    = 12,
    REG_TELEM_AI_BASE  = REG_BATTERY_MV,   // 6 contiguous analog values
};

// Bit positions inside REG_VALVE_COILS (per TRACTOR_NODE.md).
enum ValveBit : uint16_t {
    VB_DRIVE_LF = 0, VB_DRIVE_LR, VB_DRIVE_RF, VB_DRIVE_RR,
    VB_BOOM_UP, VB_BOOM_DN, VB_BUCKET_CURL, VB_BUCKET_DUMP,
};

// ---------- M4 shared SRAM (mirror of tractor_m4.cpp) ----------
// Layout owned by firmware/common/shared_mem.h. The M4 watchdog trips if we
// don't stamp `alive_tick_ms` AND `loop_counter` every loop iteration.
#include "../common/shared_mem.h"
static volatile SharedM7M4* const SHARED =
    reinterpret_cast<volatile SharedM7M4*>(LIFETRAC_SHARED_ADDR);

// ---------- pre-shared key (provisioned via USB tool) ----------
static const uint8_t kFleetKey[16] = {0};

// ---------- per-source state ----------
struct SourceState {
    uint32_t last_heartbeat_ms;
    uint32_t last_control_ms;
    uint16_t last_sequence;
    uint32_t takectl_until_ms;
    int16_t  rssi_dbm;
    int16_t  snr_db_x10;          // SX1276 reports SNR in 0.25 dB; we store *10
    LpReplayWindow replay;
    ControlFrame latest;
};
static SourceState g_src[3];   // index by source_id - 1

// ---------- FHSS state (DECISIONS.md D-C6, MASTER_PLAN.md §8.17) ----------
//
// Owned here on the M7; the CSMA #ifdef block below feeds them into
// `lp_csma_pick_hop()`. `g_fhss_hop_counter` advances once per TX so
// successive frames land on different channels. `g_fhss_key_id` is the
// fleet hop-key seed (epoch-rotated by `time_service.py` over the X8↔H747
// UART once that service is wired); zero is a valid value during bring-up.
uint32_t g_fhss_hop_counter = 0;
uint32_t g_fhss_key_id      = 0;

// Forward declaration so emit_topic() / send_link_tune() (which appear above
// the definition) can call the CSMA hop helper.
static inline void csma_pick_hop_before_tx();

static const uint32_t HEARTBEAT_TIMEOUT_MS = 500;
static const uint32_t CONTROL_TIMEOUT_MS   = 200;   // stale ControlFrame → drop to neutral
static const uint32_t TAKECTL_LATCH_MS     = 30000;

// E-stop latched fault — set by FT_COMMAND/CMD_ESTOP, cleared only by an
// explicit command from the base UI (CMD_CLEAR_ESTOP).
static volatile bool g_estop_latched = false;

// ---------- adaptive SF ladder (R-8 hysteresis) ----------
//
// Per LORA_IMPLEMENTATION.md §3.1 + MASTER_PLAN.md §8.17:
//   - 3 ladder rungs SF7/SF8/SF9, all on BW125 / CR4-5 (control profile).
//   - Step-down (SF↑): require N=3 consecutive bad 5 s windows.
//   - Step-up   (SF↓): require >=30 s clean (6 consecutive good windows).
//   - On transition: send CMD_LINK_TUNE twice back-to-back — once at the old
//     PHY, once at the new PHY — then both ends switch. If no Heartbeat at
//     the new SF within 500 ms, revert and increment the fail counter.
//
// "Bad window" = the *active* source's heartbeat count over the 5 s window
// dropped below 80 % of the expected 50 frames (handheld emits HB @ 10 Hz).
// We track the active source rather than the worst eligible source so that a
// stale autonomy link can't drag the human-driven control link down.
struct LadderRung { uint8_t sf; uint16_t bw_khz; uint8_t cr_den; uint8_t bw_code; };
static const LadderRung LADDER[3] = {
    { 7, 250, 5, 1 },   // bw_code 1 = 250 kHz — fits 20 Hz cadence (DECISIONS.md D-A2)
    { 8, 125, 5, 0 },   // fallback rungs widen back to BW125 for link budget
    { 9, 125, 5, 0 },
};
static const uint32_t LADDER_WINDOW_MS          = 5000;
static const uint8_t  LADDER_BAD_TO_STEPDOWN    = 3;     // 15 s of bad to widen SF
static const uint8_t  LADDER_GOOD_TO_STEPUP     = 6;     // 30 s of clean to narrow SF
static const uint32_t LADDER_HB_EXPECTED_5S     = 50;    // 10 Hz × 5 s
static const uint32_t LADDER_HB_BAD_THRESHOLD   = 40;    // <80 %  -> bad
static const uint32_t LADDER_HB_GOOD_THRESHOLD  = 48;    // >=96 % -> good
static const uint32_t LADDER_TUNE_REVERT_MS     = 500;

// Reason codes for CMD_LINK_TUNE (LORA_PROTOCOL.md §CMD_LINK_TUNE row).
enum LinkTuneReason : uint8_t {
    LTR_LOW_SNR        = 0x01,
    LTR_HIGH_LOSS      = 0x02,
    LTR_MANUAL         = 0x03,
    LTR_RECOVERY_DOWN  = 0x04,
};

static uint8_t  g_ladder_rung           = 0;     // committed rung
static uint8_t  g_ladder_pending_rung   = 0;     // rung being verified
static uint32_t g_ladder_tune_deadline  = 0;     // 0 = no tune in flight
static uint32_t g_ladder_window_start   = 0;
static uint32_t g_ladder_hb_count[3]    = { 0, 0, 0 };
static uint32_t g_ladder_ctrl_count[3]  = { 0, 0, 0 };
static uint8_t  g_ladder_consec_bad     = 0;
static uint8_t  g_ladder_consec_good    = 0;
static uint32_t g_ladder_hb_at_pending  = 0;     // last HB ts while tune pending
static uint32_t g_ladder_tune_failures  = 0;     // surfaced via topic 0x10 later

// ---------- KISS decoder for incoming bytes ----------
static KissDecoder g_dec;

// ---------- helpers ----------
static int src_index(uint8_t source_id) {
    switch (source_id) {
        case SRC_HANDHELD: return 0;
        case SRC_BASE:     return 1;
        case SRC_AUTONOMY: return 2;
        default:           return -1;
    }
}

// pick_active_source — strict priority + take-control latch.
// A source is only eligible if BOTH its heartbeat AND its most recent
// ControlFrame are fresh. Heartbeat-only-with-stale-control would mean we
// keep replaying an old joystick position, which is exactly what the M4 +
// Opta watchdogs are there to catch — but defense in depth: catch it here too.
static int pick_active_source() {
    uint32_t now = millis();
    auto fresh = [now](int i) {
        return (now - g_src[i].last_heartbeat_ms) < HEARTBEAT_TIMEOUT_MS &&
               g_src[i].last_control_ms != 0 &&
               (now - g_src[i].last_control_ms) < CONTROL_TIMEOUT_MS;
    };
    // 1. Latched take-control wins, if its link is fresh.
    for (int i = 0; i < 3; i++) {
        if (g_src[i].takectl_until_ms > now && fresh(i)) return i;
    }
    // 2. Strict priority: HANDHELD > BASE > AUTONOMY.
    for (int i = 0; i < 3; i++) {
        if (fresh(i)) return i;
    }
    return -1;  // failsafe → neutral
}

// Drive the Opta to the active source's ControlFrame. -1 = neutral.
//
// Always writes the full 7-register holding block (0x0000..0x0006) in a single
// WriteMultipleRegisters transaction so the slave sees a consistent snapshot.
// E-stop latch and "no fresh source" both collapse to the neutral payload.
static uint16_t g_watchdog_ctr = 0;

static void apply_control(int active) {
    uint16_t regs[HOLDING_BLOCK_LEN] = {0};
    regs[REG_WATCHDOG_CTR] = ++g_watchdog_ctr;   // tick every call → >=10 Hz

    if (g_estop_latched) {
        regs[REG_ARM_ESTOP] = 1;
        regs[REG_CMD_SOURCE] = 0;       // NONE
    } else if (active >= 0) {
        const ControlFrame& cf = g_src[active].latest;
        uint16_t coils = 0;
        // Map joystick + buttons → 8 directional valve coils.
        // (TODO: deadband, ramp, dual-flow split — port from
        //  RESEARCH-CONTROLLER/arduino_opta_controller/.)
        if (cf.axis_lh_y >  20) { coils |= (1u << VB_DRIVE_LF) | (1u << VB_DRIVE_RF); }
        if (cf.axis_lh_y < -20) { coils |= (1u << VB_DRIVE_LR) | (1u << VB_DRIVE_RR); }
        if (cf.axis_rh_y >  20) { coils |= (1u << VB_BOOM_UP);   }
        if (cf.axis_rh_y < -20) { coils |= (1u << VB_BOOM_DN);   }
        if (cf.buttons & BTN_BUCKET_CURL) { coils |= (1u << VB_BUCKET_CURL); }
        if (cf.buttons & BTN_BUCKET_DUMP) { coils |= (1u << VB_BUCKET_DUMP); }
        regs[REG_VALVE_COILS] = coils;

        uint16_t mag = (uint16_t)abs(cf.axis_lh_y) * 78;   // 127*78 ≈ 10000
        if (mag > 10000) mag = 10000;
        regs[REG_FLOW_SP_1] = mag;
        regs[REG_FLOW_SP_2] = mag;
        regs[REG_CMD_SOURCE] = (uint16_t)(active + 1);     // 1=HANDHELD, 2=BASE, 3=AUTONOMY
    }
    // else: regs already zero → all valves off, both flows 0, cmd_source=NONE.

    ModbusRTUClient.beginTransmission(OPTA_SLAVE_ID, HOLDING_REGISTERS,
                                      0x0000, HOLDING_BLOCK_LEN);
    for (uint16_t i = 0; i < HOLDING_BLOCK_LEN; i++) {
        ModbusRTUClient.write(regs[i]);
    }
    ModbusRTUClient.endTransmission();
}

// The Opta-side watchdog is now folded into apply_control() (it advances
// REG_WATCHDOG_CTR each pass). Keeping this name as a no-op for clarity.
static void tick_opta_watchdog() { /* see apply_control */ }

// Process one decoded LoRa frame (already past KISS de-framing).
static void process_air_frame(uint8_t* onair, size_t len, int16_t rssi_dbm, int16_t snr_db_x10) {
    if (len < 12 + 16) return;             // nonce + tag minimum
    uint8_t nonce[12];
    memcpy(nonce, onair, 12);
    uint8_t pt[160];
    if (!lp_decrypt(kFleetKey, nonce, onair + 12, len - 12, pt)) return;
    size_t pt_len = len - 12 - 16;
    if (pt_len < sizeof(LoraHeader)) return;

    LoraHeader* hdr = (LoraHeader*)pt;
    if (hdr->version != LIFETRAC_PROTO_VERSION) return;
    int idx = src_index(hdr->source_id);
    if (idx < 0) return;

    // Replay protection: 64-frame sliding window per source. Rejects any
    // sequence we have already accepted as well as anything more than 64
    // frames behind the high-water mark (handles legitimate out-of-order
    // delivery while killing replay attacks against captured ciphertext).
    if (!lp_replay_check_and_update(&g_src[idx].replay, hdr->sequence_num)) return;
    g_src[idx].last_sequence = hdr->sequence_num;
    g_src[idx].rssi_dbm      = rssi_dbm;
    g_src[idx].snr_db_x10    = snr_db_x10;

    uint32_t now = millis();
    switch (hdr->frame_type) {
        case FT_CONTROL: {
            if (pt_len < sizeof(ControlFrame)) break;
            // VALIDATE CRC BEFORE TOUCHING g_src[idx].latest — otherwise a
            // corrupted frame would overwrite the last good control until the
            // next valid one arrives.
            ControlFrame cf;
            memcpy(&cf, pt, sizeof(ControlFrame));
            uint16_t expect = lp_crc16((const uint8_t*)&cf,
                                       sizeof(ControlFrame) - sizeof(uint16_t));
            if (expect != cf.crc16) break;       // drop silently
            g_src[idx].latest          = cf;
            g_src[idx].last_control_ms = now;
            g_ladder_ctrl_count[idx]++;
            break;
        }
        case FT_HEARTBEAT:
            if (pt_len >= sizeof(HeartbeatFrame)) {
                HeartbeatFrame hb;
                memcpy(&hb, pt, sizeof(HeartbeatFrame));
                uint16_t expect = lp_crc16((const uint8_t*)&hb,
                                           sizeof(HeartbeatFrame) - sizeof(uint16_t));
                if (expect != hb.crc16) break;
                g_src[idx].last_heartbeat_ms = now;
                g_ladder_hb_count[idx]++;
                if (g_ladder_tune_deadline != 0) {
                    // We are inside the post-tune verification window; record
                    // the HB so poll_link_ladder() commits the new rung.
                    g_ladder_hb_at_pending = now;
                }
                if (hb.flags & FLAG_TAKECTL_HELD) {
                    g_src[idx].takectl_until_ms = now + TAKECTL_LATCH_MS;
                }
            }
            break;
        case FT_COMMAND: {
            // Payload layout (cleartext, after the 5-byte header):
            //   byte 5: opcode (CMD_*)  byte 6..N: opcode-specific args
            if (pt_len < 6) break;
            uint8_t op = pt[5];
            switch (op) {
                case CMD_ESTOP:
                    g_estop_latched = true;
                    // Force immediate neutral apply this tick.
                    apply_control(-1);
                    break;
                case CMD_CLEAR_ESTOP:
                    // Only the BASE source may clear an E-stop.
                    if (hdr->source_id == SRC_BASE) g_estop_latched = false;
                    break;
                case CMD_LINK_TUNE:
                    // Tractor is the canonical issuer of CMD_LINK_TUNE; an
                    // inbound copy on this side means somebody else announced
                    // a retune (e.g. a future two-radio peer per
                    // MASTER_PLAN.md §8.17.1). For the current single-radio
                    // build we ignore it — our own tx path already retunes.
                    break;
                case CMD_ENCODE_MODE:
                    // Base -> tractor X8 path is plumbed later; M7 only logs/forwards.
                    break;
            }
            break;
        }
    }
}

// Pull bytes off the LoRa modem and feed the KISS decoder.
static void poll_radio() {
    int packet_len = radio.getPacketLength();
    if (packet_len <= 0) return;
    uint8_t buf[256];
    int st = radio.readData(buf, packet_len);
    if (st != RADIOLIB_ERR_NONE) return;
    int16_t rssi = radio.getRSSI();
    // SX1276::getSNR() returns float dB; capture *10 for fixed-point telemetry.
    int16_t snr_x10 = (int16_t)(radio.getSNR() * 10.0f);

    for (int i = 0; i < packet_len; i++) {
        uint8_t* frame; size_t flen;
        if (lp_kiss_feed(&g_dec, buf[i], &frame, &flen)) {
            process_air_frame(frame, flen, rssi, snr_x10);
        }
    }
    radio.startReceive();
}

// Build a 12-byte AES-GCM nonce identical in shape to the handheld:
//   [ source_id | seq(LE) | t_seconds(LE) | rand(5) ]
// Without the time/random bits, two telemetry frames with the same sequence
// (e.g. across reboot or after a 16-bit rollover) would reuse a nonce, which
// is catastrophic for AES-GCM confidentiality.
static void build_nonce(uint8_t out[12], uint8_t source_id, uint16_t seq) {
    out[0] = source_id;
    out[1] = (uint8_t)(seq & 0xFF);
    out[2] = (uint8_t)(seq >> 8);
    uint32_t t = millis() / 1000;
    out[3] = (uint8_t)(t      ); out[4] = (uint8_t)(t >>  8);
    out[5] = (uint8_t)(t >> 16); out[6] = (uint8_t)(t >> 24);
    for (int i = 0; i < 5; i++) out[7 + i] = (uint8_t)random(0, 256);
}

// Read Opta telemetry and ship a TelemetryFrame over LoRa.
static uint16_t g_telem_seq = 0;

// emit_topic — encrypt + frame an arbitrary (topic_id, payload) tuple and
// transmit it as a TelemetryFrame on the radio. Shared by emit_telemetry()
// (Opta hydraulics @ 1 Hz) and the X8 UART pump (gps @ 1 Hz, imu @ 5 Hz).
//
// payload_len must be ≤ TELEM_MAX_PAYLOAD as defined in LORA_PROTOCOL.md.
static void emit_topic(uint8_t topic_id, const uint8_t* payload, uint8_t payload_len) {
    if (payload_len > sizeof(((TelemetryFrame*)0)->payload) - 2) return;

    TelemetryFrame tf;
    tf.hdr.version      = LIFETRAC_PROTO_VERSION;
    tf.hdr.source_id    = SRC_TRACTOR;
    tf.hdr.frame_type   = FT_TELEMETRY;
    tf.hdr.sequence_num = ++g_telem_seq;
    tf.topic_id         = topic_id;
    tf.payload_len      = payload_len;
    if (payload_len > 0) memcpy(tf.payload, payload, payload_len);

    // CRC immediately after payload
    size_t prefix = sizeof(LoraHeader) + 2 + tf.payload_len;
    uint16_t crc = lp_crc16((const uint8_t*)&tf, prefix);
    tf.payload[tf.payload_len    ] = (uint8_t)(crc & 0xFF);
    tf.payload[tf.payload_len + 1] = (uint8_t)(crc >> 8);

    // Encrypt + KISS + send
    uint8_t nonce[12];
    build_nonce(nonce, SRC_TRACTOR, g_telem_seq);
    uint8_t enc[200];
    if (!lp_encrypt(kFleetKey, nonce, (const uint8_t*)&tf,
                    prefix + 2, enc)) return;
    uint8_t onair[12 + 200];
    memcpy(onair, nonce, 12);
    memcpy(onair + 12, enc, prefix + 2 + 16);
    uint8_t kiss[512];
    size_t kl = lp_kiss_encode(onair, 12 + prefix + 2 + 16, kiss, sizeof(kiss));
    if (kl) {
        csma_pick_hop_before_tx();
        radio.transmit(kiss, kl);
    }
    radio.startReceive();
}

static void emit_telemetry() {
    // Read INPUT registers (function 0x04), not holding — see TRACTOR_NODE.md.
    if (!ModbusRTUClient.requestFrom(OPTA_SLAVE_ID, INPUT_REGISTERS,
                                     REG_TELEM_AI_BASE, 6)) return;
    uint8_t payload[12];               // 6 analog inputs * 2 bytes
    for (int i = 0; i < 6; i++) {
        uint16_t v = ModbusRTUClient.read();
        payload[i*2  ] = (uint8_t)(v & 0xFF);
        payload[i*2+1] = (uint8_t)(v >> 8);
    }
    emit_topic(TOPIC_HYDRAULICS, payload, sizeof(payload));   // 0x04
}

// emit_source_active() — 1 Hz publish of topic 0x10 (TOPIC_SOURCE_ACTIVE) per
// LORA_PROTOCOL.md. Payload is the operator-visible link state so the base
// UI can render "LINK: SF7 (clean)" and "CTRL: HANDHELD" without inferring.
//
//   byte  0   : active_source (0 = none, 1 = HANDHELD, 2 = BASE, 3 = AUTONOMY)
//   byte  1   : committed ladder rung (0 = SF7, 1 = SF8, 2 = SF9)
//   byte  2   : estop_latched (0 / 1)
//   byte  3   : pending tune in flight (0 / 1)
//   byte  4-7 : little-endian uint32 lifetime tune-revert failures
//   byte  8-13: per-source RSSI dBm, int16 LE, [HANDHELD, BASE, AUTONOMY]
//   byte 14-19: per-source SNR *10, int16 LE, [HANDHELD, BASE, AUTONOMY]
//
// Total 20 bytes. Base bridge `lora_bridge.py` decodes this layout into the
// per-source RSSI/SNR fields surfaced on the operator UI.
static void emit_source_active() {
    int active = pick_active_source();
    uint8_t payload[20];
    payload[0] = (active < 0) ? 0 : (uint8_t)(active + 1);
    payload[1] = g_ladder_rung;
    payload[2] = g_estop_latched ? 1 : 0;
    payload[3] = (g_ladder_tune_deadline != 0) ? 1 : 0;
    payload[4] = (uint8_t)(g_ladder_tune_failures      );
    payload[5] = (uint8_t)(g_ladder_tune_failures >>  8);
    payload[6] = (uint8_t)(g_ladder_tune_failures >> 16);
    payload[7] = (uint8_t)(g_ladder_tune_failures >> 24);
    for (int i = 0; i < 3; ++i) {
        int16_t r = g_src[i].rssi_dbm;
        int16_t s = g_src[i].snr_db_x10;
        payload[ 8 + i*2    ] = (uint8_t)(r & 0xFF);
        payload[ 8 + i*2 + 1] = (uint8_t)((r >> 8) & 0xFF);
        payload[14 + i*2    ] = (uint8_t)(s & 0xFF);
        payload[14 + i*2 + 1] = (uint8_t)((s >> 8) & 0xFF);
    }
    emit_topic(TOPIC_SOURCE_ACTIVE, payload, sizeof(payload));
}

// ---------- CSMA hop helper (DECISIONS.md D-C6) ----------
//
// Mirrors the handheld `send_frame()` skip-busy block. Both M7 TX paths
// (emit_topic, send_link_tune) call this just before `radio.transmit()`
// when LIFETRAC_FHSS_ENABLED is defined; otherwise it's a no-op so the
// single-channel default doesn't pay for `scanChannel()`.
static inline void csma_pick_hop_before_tx() {
#ifdef LIFETRAC_FHSS_ENABLED
    auto sampler = [](uint32_t hz, void* /*ctx*/) -> int16_t {
        radio.setFrequency(hz / 1.0e6f);
        int state = radio.scanChannel();
        return (state == RADIOLIB_LORA_DETECTED) ? (int16_t)-40 : (int16_t)-110;
    };
    uint8_t skips = 0;
    uint32_t hop = lp_csma_pick_hop(g_fhss_key_id, g_fhss_hop_counter,
                                    sampler, nullptr,
                                    LP_CSMA_DEFAULT_BUSY_DBM,
                                    LP_CSMA_DEFAULT_MAX_SKIPS,
                                    &skips);
    radio.setFrequency(hop / 1.0e6f);
    g_fhss_hop_counter++;
    (void)skips;   // TODO: surface to audit log once X8↔H747 IPC lands
#endif
}

// ---------- X8 UART pump ----------
//
// On a Portenta X8, the Linux side runs `tractor_x8/gps_service.py` and
// `tractor_x8/imu_service.py`, which write KISS-framed packets to the
// X8↔H747 UART at 921600 8N1. Each packet is:
//
//     <topic_id:1> <payload:N>
//
// where topic_id matches `TOPIC_BY_ID` in `base_station/lora_bridge.py`
// (0x01 = gps, 0x07 = imu, etc.). We decode KISS, peel off the topic byte,
// and re-emit the rest as a TelemetryFrame on the LoRa radio.
//
// Standalone-H7 fallback: when the H747 is *not* in an X8 there is nothing
// driving Serial1 from the Linux side, so this loop sees no bytes — the rest
// of the firmware behaves identically. (In that build, IMU + GPS would have
// to be wired natively — see HARDWARE_BOM.md § Notes on substitutions.)
static KissDecoder g_x8_dec;

static void poll_x8_uart() {
    while (Serial1.available()) {
        uint8_t b = (uint8_t)Serial1.read();
        uint8_t* frame; size_t flen;
        if (lp_kiss_feed(&g_x8_dec, b, &frame, &flen)) {
            if (flen < 1) continue;             // empty frame
            uint8_t topic_id = frame[0];
            const uint8_t* payload = frame + 1;
            size_t payload_len = flen - 1;
            // TelemetryFrame.payload + CRC must fit; cap defensively.
            if (payload_len > 64) continue;
            emit_topic(topic_id, payload, (uint8_t)payload_len);
        }
    }
}


// ---------- adaptive SF ladder helpers ----------
//
// apply_phy_rung() retunes the local radio. RX must be re-armed because
// changing SF / BW kicks the modem out of receive on the SX1276.
static void apply_phy_rung(uint8_t rung) {
    if (rung > 2) return;
    radio.setSpreadingFactor(LADDER[rung].sf);
    radio.setBandwidth((float)LADDER[rung].bw_khz);
    radio.setCodingRate(LADDER[rung].cr_den);
    radio.startReceive();
}

// send_link_tune() builds, encrypts, and transmits one CMD_LINK_TUNE frame.
// Per LORA_PROTOCOL.md §CMD_LINK_TUNE the payload is exactly
//   arg[0] = target SF, arg[1] = target_bw_code, arg[2] = reason.
static void send_link_tune(uint8_t target_rung, uint8_t reason) {
    if (target_rung > 2) return;
    CommandFrame cf;
    memset(&cf, 0, sizeof(cf));
    cf.hdr.version      = LIFETRAC_PROTO_VERSION;
    cf.hdr.source_id    = SRC_TRACTOR;
    cf.hdr.frame_type   = FT_COMMAND;
    cf.hdr.sequence_num = ++g_telem_seq;
    cf.opcode  = CMD_LINK_TUNE;
    cf.arg[0]  = LADDER[target_rung].sf;
    cf.arg[1]  = LADDER[target_rung].bw_code;
    cf.arg[2]  = reason;
    cf.crc16   = lp_crc16((const uint8_t*)&cf,
                          sizeof(CommandFrame) - sizeof(uint16_t));

    uint8_t nonce[12];
    build_nonce(nonce, SRC_TRACTOR, cf.hdr.sequence_num);
    uint8_t enc[sizeof(CommandFrame) + 16];
    if (!lp_encrypt(kFleetKey, nonce, (const uint8_t*)&cf,
                    sizeof(CommandFrame), enc)) return;
    uint8_t onair[12 + sizeof(CommandFrame) + 16];
    memcpy(onair, nonce, 12);
    memcpy(onair + 12, enc, sizeof(CommandFrame) + 16);
    uint8_t kiss[2 * sizeof(onair) + 4];
    size_t kl = lp_kiss_encode(onair, sizeof(onair), kiss, sizeof(kiss));
    if (kl) {
        csma_pick_hop_before_tx();
        radio.transmit(kiss, kl);
    }
}

// try_step_ladder() executes the twice-back-to-back handshake and arms the
// 500 ms revert deadline. The committed rung (g_ladder_rung) does NOT change
// until poll_link_ladder() observes a Heartbeat at the new PHY.
static void try_step_ladder(uint8_t target_rung, uint8_t reason) {
    if (target_rung > 2 || target_rung == g_ladder_rung) return;
    // 1. Announce at current PHY so old-PHY peers retune.
    send_link_tune(target_rung, reason);
    // 2. Switch our radio (apply_phy_rung re-arms RX so the announce-at-new
    //    PHY immediately below transmits from a known state).
    apply_phy_rung(target_rung);
    // 3. Re-announce at new PHY so new-PHY peers (or peers that missed #1
    //    but somehow already retuned) latch the change.
    send_link_tune(target_rung, reason);
    // 4. Re-arm RX after the second TX — radio.transmit() leaves the SX1276
    //    in standby, so without this the verification HB would never arrive.
    radio.startReceive();

    g_ladder_pending_rung  = target_rung;
    g_ladder_tune_deadline = millis() + LADDER_TUNE_REVERT_MS;
    g_ladder_hb_at_pending = 0;

    // Reset window stats: the post-tune 5 s window starts fresh.
    g_ladder_consec_bad  = 0;
    g_ladder_consec_good = 0;
    for (int i = 0; i < 3; i++) {
        g_ladder_hb_count[i]   = 0;
        g_ladder_ctrl_count[i] = 0;
    }
    g_ladder_window_start = millis();
}

// poll_link_ladder() — called once per loop. Two responsibilities:
//   1. Verify any pending tune (commit on HB at new PHY, revert on timeout).
//   2. When the 5 s window closes, classify it bad/good and step the ladder
//      after R-8 hysteresis is satisfied (3 bad → step-down, 6 good → step-up).
static void poll_link_ladder() {
    uint32_t now = millis();
    if (g_ladder_window_start == 0) g_ladder_window_start = now;

    // (1) Pending tune verification.
    if (g_ladder_tune_deadline != 0 && (int32_t)(now - g_ladder_tune_deadline) >= 0) {
        bool got_hb = (g_ladder_hb_at_pending != 0) &&
                      (g_ladder_hb_at_pending + LADDER_TUNE_REVERT_MS >= g_ladder_tune_deadline);
        if (got_hb) {
            g_ladder_rung = g_ladder_pending_rung;          // commit
        } else {
            apply_phy_rung(g_ladder_rung);                  // revert
            g_ladder_tune_failures++;
        }
        g_ladder_tune_deadline = 0;
        g_ladder_hb_at_pending = 0;
    }

    // (2) Window evaluation. Hold off while a tune is still in flight so the
    //     verification window doesn't double-count as a "bad" window.
    if (g_ladder_tune_deadline != 0) return;
    if ((now - g_ladder_window_start) < LADDER_WINDOW_MS) return;

    int active = pick_active_source();
    bool bad  = false;
    bool good = false;
    if (active >= 0) {
        uint32_t hb = g_ladder_hb_count[active];
        bad  = (hb <  LADDER_HB_BAD_THRESHOLD);
        good = (hb >= LADDER_HB_GOOD_THRESHOLD);
    } else {
        // No fresh source at all: count as a bad window so the ladder
        // pessimistically widens SF (gives marginal links a chance to recover).
        bad = true;
    }
    if (bad)  { g_ladder_consec_bad++;  g_ladder_consec_good = 0; }
    if (good) { g_ladder_consec_good++; g_ladder_consec_bad  = 0; }

    if (g_ladder_consec_bad >= LADDER_BAD_TO_STEPDOWN && g_ladder_rung < 2) {
        try_step_ladder((uint8_t)(g_ladder_rung + 1), LTR_HIGH_LOSS);
    } else if (g_ladder_consec_good >= LADDER_GOOD_TO_STEPUP && g_ladder_rung > 0) {
        try_step_ladder((uint8_t)(g_ladder_rung - 1), LTR_RECOVERY_DOWN);
    }

    // Reset window counters.
    for (int i = 0; i < 3; i++) {
        g_ladder_hb_count[i]   = 0;
        g_ladder_ctrl_count[i] = 0;
    }
    g_ladder_window_start = now;
}


// ---------- Arduino entry points ----------
void setup() {
    Serial.begin(115200);

    // LoRa control profile: SF7 / BW125 / CR4-5 per MASTER_PLAN.md §8.17.
    radio.begin(915.0, 125.0, 7, 5, 0x12, 20);
    radio.startReceive();

    // RS-485 / Modbus
    RS485.setPins(/*tx*/ PA_9, /*de*/ PA_10, /*re*/ PB_3);
    if (!ModbusRTUClient.begin(115200)) {
        Serial.println("Modbus begin failed");
    }

    // X8 ↔ H747 UART (Linux side runs gps_service.py + imu_service.py).
    // Harmless on a standalone H7 — Serial1 just sees no bytes.
    Serial1.begin(921600);
}

void loop() {
    static uint32_t next_arb = 0;
    static uint32_t next_tx  = 0;
    uint32_t now = millis();

    // Stamp the M4 watchdog FIRST every loop iteration. If we crash anywhere
    // below, the M4 watchdog (200 ms) will pull the PSR-alive GPIO low and
    // dump the valves. Apply this even if nothing else makes progress.
    // Bump version + counter too so the M4 can validate layout and detect
    // a stuck-but-fresh-looking timestamp.
    SHARED->version       = LIFETRAC_SHARED_VERSION;
    SHARED->alive_tick_ms = now;
    SHARED->loop_counter += 1;
    SHARED->estop_request = g_estop_latched ? 1u : 0u;

    poll_radio();
    poll_x8_uart();    // pump KISS frames from gps_service.py / imu_service.py
    poll_link_ladder();  // R-8 hysteresis SF7→SF8→SF9 ladder

    if ((int32_t)(now - next_arb) >= 0) {
        next_arb = now + 50;        // 20 Hz arbitration AND watchdog tick
        apply_control(pick_active_source());   // also advances REG_WATCHDOG_CTR
    }
    if ((int32_t)(now - next_tx) >= 0) {
        next_tx = now + 1000;       // 1 Hz telemetry
        emit_telemetry();
        emit_source_active();       // topic 0x10: link state for the operator UI
    }
}
