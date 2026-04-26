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
#include "../lora_proto/lora_proto.h"

// ---------- LoRa pins (Portenta Max Carrier wires Murata SiP via SPI) ----------
SX1276 radio = new Module(/*nss*/ PD_4, /*dio0*/ PD_5,
                          /*reset*/ PE_4, /*dio1*/ PE_5);

// ---------- Modbus master over RS-485 (Max Carrier J6) ----------
// 115200 8N1, RTU framing. Opta is slave id 0x10.
#define OPTA_SLAVE_ID 0x10

// Opta register map (mirror of opta_modbus_slave.ino)
enum OptaReg : uint16_t {
    REG_ALIVE_TICK     = 0x0000,
    REG_FLOW_SP_1      = 0x0001,
    REG_FLOW_SP_2      = 0x0002,
    REG_VALVE_BASE     = 0x0010,   // 8 contiguous: 0x0010..0x0017
    REG_CURRENT_BASE   = 0x0020,   // 8 contiguous read-only
    REG_MODE_SWITCH    = 0x0030,
    REG_ESTOP_LOOP     = 0x0031,
    REG_ANALOG_BASE    = 0x0040,   // 6 contiguous read-only
    REG_LAST_ERROR     = 0x00FF
};

// ---------- pre-shared key (provisioned via USB tool) ----------
static const uint8_t kFleetKey[16] = {0};

// ---------- per-source state ----------
struct SourceState {
    uint32_t last_heartbeat_ms;
    uint32_t last_control_ms;
    uint16_t last_sequence;
    uint32_t takectl_until_ms;
    int16_t  rssi_dbm;
    ControlFrame latest;
};
static SourceState g_src[3];   // index by source_id - 1

static const uint32_t HEARTBEAT_TIMEOUT_MS = 500;
static const uint32_t TAKECTL_LATCH_MS     = 30000;

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
static int pick_active_source() {
    uint32_t now = millis();
    // 1. Latched take-control wins, if its heartbeat is fresh.
    for (int i = 0; i < 3; i++) {
        if (g_src[i].takectl_until_ms > now &&
            (now - g_src[i].last_heartbeat_ms) < HEARTBEAT_TIMEOUT_MS) {
            return i;
        }
    }
    // 2. Strict priority: HANDHELD > BASE > AUTONOMY, must be fresh.
    for (int i = 0; i < 3; i++) {
        if ((now - g_src[i].last_heartbeat_ms) < HEARTBEAT_TIMEOUT_MS) {
            return i;
        }
    }
    return -1;  // failsafe
}

// Drive the Opta to the active source's ControlFrame. -1 = neutral.
static void apply_control(int active) {
    if (active < 0) {
        // Failsafe: all valves off, both flow set-points to zero.
        ModbusRTUClient.beginTransmission(OPTA_SLAVE_ID, HOLDING_REGISTERS,
                                          REG_VALVE_BASE, 8);
        for (int i = 0; i < 8; i++) ModbusRTUClient.write(0);
        ModbusRTUClient.endTransmission();
        ModbusRTUClient.holdingRegisterWrite(OPTA_SLAVE_ID, REG_FLOW_SP_1, 0);
        ModbusRTUClient.holdingRegisterWrite(OPTA_SLAVE_ID, REG_FLOW_SP_2, 0);
        return;
    }
    const ControlFrame& cf = g_src[active].latest;

    // Map joystick + buttons → 8 directional valves.
    // (TODO: deadband, ramp, dual-flow split — port from RESEARCH-CONTROLLER/arduino_opta_controller/.)
    uint16_t valves[8] = {0};
    valves[0] = (cf.axis_lh_y > 20);   // drive LH fwd
    valves[1] = (cf.axis_lh_y < -20);  // drive LH rev
    valves[2] = (cf.axis_lh_y > 20);   // drive RH fwd (TODO: split for differential turn)
    valves[3] = (cf.axis_lh_y < -20);  // drive RH rev
    valves[4] = (cf.axis_rh_y > 20);   // boom up
    valves[5] = (cf.axis_rh_y < -20);  // boom down
    valves[6] = (cf.buttons & BTN_BUCKET_CURL) ? 1 : 0;
    valves[7] = (cf.buttons & BTN_BUCKET_DUMP) ? 1 : 0;

    ModbusRTUClient.beginTransmission(OPTA_SLAVE_ID, HOLDING_REGISTERS,
                                      REG_VALVE_BASE, 8);
    for (int i = 0; i < 8; i++) ModbusRTUClient.write(valves[i]);
    ModbusRTUClient.endTransmission();

    // Flow set-point: scale joystick magnitude to 0..10000 (=> 0..10 V).
    uint16_t mag = (uint16_t)abs(cf.axis_lh_y) * 78;  // 127 * 78 ≈ 10000
    ModbusRTUClient.holdingRegisterWrite(OPTA_SLAVE_ID, REG_FLOW_SP_1, mag);
    ModbusRTUClient.holdingRegisterWrite(OPTA_SLAVE_ID, REG_FLOW_SP_2, mag);
}

static void tick_opta_watchdog() {
    static uint16_t alive = 0;
    ModbusRTUClient.holdingRegisterWrite(OPTA_SLAVE_ID, REG_ALIVE_TICK, ++alive);
}

// Process one decoded LoRa frame (already past KISS de-framing).
static void process_air_frame(uint8_t* onair, size_t len, int16_t rssi_dbm) {
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

    // Replay protection: sequence must advance (with rollover tolerance).
    uint16_t prev = g_src[idx].last_sequence;
    int16_t  delta = (int16_t)(hdr->sequence_num - prev);
    if (prev != 0 && delta <= 0 && delta > -1024) return;   // stale/replay
    g_src[idx].last_sequence = hdr->sequence_num;
    g_src[idx].rssi_dbm      = rssi_dbm;

    uint32_t now = millis();
    switch (hdr->frame_type) {
        case FT_CONTROL:
            if (pt_len >= sizeof(ControlFrame)) {
                memcpy(&g_src[idx].latest, pt, sizeof(ControlFrame));
                g_src[idx].last_control_ms = now;
                // CRC over the cleartext frame
                uint16_t expect = lp_crc16(pt, sizeof(ControlFrame) - 2);
                if (expect != g_src[idx].latest.crc16) {
                    // CRC fail — invalidate
                    g_src[idx].last_control_ms = 0;
                }
            }
            break;
        case FT_HEARTBEAT:
            if (pt_len >= sizeof(HeartbeatFrame)) {
                HeartbeatFrame hb;
                memcpy(&hb, pt, sizeof(HeartbeatFrame));
                g_src[idx].last_heartbeat_ms = now;
                if (hb.flags & FLAG_TAKECTL_HELD) {
                    g_src[idx].takectl_until_ms = now + TAKECTL_LATCH_MS;
                }
            }
            break;
        case FT_COMMAND:
            // E-stop and rekey land here. TODO: wire CMD_ESTOP straight to apply_control(-1)
            // and latch a fault until cleared from the base UI.
            break;
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

    for (int i = 0; i < packet_len; i++) {
        uint8_t* frame; size_t flen;
        if (lp_kiss_feed(&g_dec, buf[i], &frame, &flen)) {
            process_air_frame(frame, flen, rssi);
        }
    }
    radio.startReceive();
}

// Read Opta telemetry and ship a TelemetryFrame over LoRa.
static void emit_telemetry() {
    TelemetryFrame tf;
    tf.hdr.version      = LIFETRAC_PROTO_VERSION;
    tf.hdr.source_id    = SRC_TRACTOR;
    tf.hdr.frame_type   = FT_TELEMETRY;
    tf.hdr.sequence_num = millis();    // good enough for now
    tf.topic_id         = 0x04;        // hydraulics, per LORA_PROTOCOL.md table
    tf.payload_len      = 12;          // 6 analog inputs * 2 bytes

    if (!ModbusRTUClient.requestFrom(OPTA_SLAVE_ID, HOLDING_REGISTERS,
                                     REG_ANALOG_BASE, 6)) return;
    for (int i = 0; i < 6; i++) {
        uint16_t v = ModbusRTUClient.read();
        tf.payload[i*2  ] = (uint8_t)(v & 0xFF);
        tf.payload[i*2+1] = (uint8_t)(v >> 8);
    }
    // CRC immediately after payload
    size_t prefix = sizeof(LoraHeader) + 2 + tf.payload_len;
    uint16_t crc = lp_crc16((const uint8_t*)&tf, prefix);
    tf.payload[tf.payload_len    ] = (uint8_t)(crc & 0xFF);
    tf.payload[tf.payload_len + 1] = (uint8_t)(crc >> 8);

    // Encrypt + KISS + send
    uint8_t nonce[12] = { SRC_TRACTOR, 0,0, 0,0,0,0, 0,0,0,0,0 };
    uint16_t seq = (uint16_t)tf.hdr.sequence_num;
    nonce[1] = (uint8_t)(seq & 0xFF);
    nonce[2] = (uint8_t)(seq >> 8);
    uint8_t enc[200];
    if (!lp_encrypt(kFleetKey, nonce, (const uint8_t*)&tf,
                    prefix + 2, enc)) return;
    uint8_t onair[12 + 200];
    memcpy(onair, nonce, 12);
    memcpy(onair + 12, enc, prefix + 2 + 16);
    uint8_t kiss[512];
    size_t kl = lp_kiss_encode(onair, 12 + prefix + 2 + 16, kiss, sizeof(kiss));
    if (kl) radio.transmit(kiss, kl);
    radio.startReceive();
}

// ---------- Arduino entry points ----------
void setup() {
    Serial.begin(115200);

    // LoRa: SF7 BW500 for control link to handheld; we'll switch SF/BW
    // dynamically once we add the base station's SF9 BW250 mode.
    radio.begin(915.0, 500.0, 7, 5, 0x12, 20);
    radio.startReceive();

    // RS-485 / Modbus
    RS485.setPins(/*tx*/ PA_9, /*de*/ PA_10, /*re*/ PB_3);
    if (!ModbusRTUClient.begin(115200)) {
        Serial.println("Modbus begin failed");
    }
}

void loop() {
    static uint32_t next_arb = 0;
    static uint32_t next_wd  = 0;
    static uint32_t next_tx  = 0;
    uint32_t now = millis();

    poll_radio();

    if ((int32_t)(now - next_arb) >= 0) {
        next_arb = now + 50;        // 20 Hz arbitration
        apply_control(pick_active_source());
    }
    if ((int32_t)(now - next_wd) >= 0) {
        next_wd = now + 50;         // 20 Hz watchdog tick (well above Opta's 200 ms timeout)
        tick_opta_watchdog();
    }
    if ((int32_t)(now - next_tx) >= 0) {
        next_tx = now + 1000;       // 1 Hz telemetry
        emit_telemetry();
    }
}
