// handheld.ino — MKR WAN 1310 handheld controller.
//
// DRAFT FOR REVIEW. Not compiled or tested.
//
// Reads two analog joysticks + 8 buttons + a momentary TAKE CONTROL button +
// a latching E-STOP button, packs a ControlFrame at 20 Hz, and emits a
// Heartbeat at the same cadence. Also receives CMD_LINK_TUNE from the
// tractor (so we follow its adaptive SF ladder) and CMD_ESTOP from any
// peer (so the OLED + LED indicate latched state).
//
// LoRa PHY at boot: SF7 / BW 125 kHz / CR 4/5 / 915 MHz (US) — control
// profile per MASTER_PLAN.md §8.17. The ladder may step us to SF8/SF9.
//
// Expected boards/libs:
//   - Arduino MKR WAN 1310
//   - RadioLib >= 6.x
//   - Adafruit_SSD1306 + Adafruit_GFX (for the optional 128x64 OLED)
//   - lora_proto.h / .c are vendored into ./src/lora_proto/ at build time
//     (CI: see .github/workflows/arduino-ci.yml; local: see
//     LifeTrac-v25/DESIGN-CONTROLLER/tools/stage_firmware_sketches.ps1).
//     The canonical sources still live in firmware/common/lora_proto/.
//     The src/ tree is .gitignored.

#include <Arduino.h>
#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "src/lora_proto/lora_proto.h"

// ---------- pins ----------
#define PIN_LH_X        A0
#define PIN_LH_Y        A1
#define PIN_RH_X        A2
#define PIN_RH_Y        A3
#define PIN_BTN_BASE    2     // first of 8 contiguous button pins
#define PIN_BTN_TAKECTL 10    // momentary, latched in software for 30 s after release
#define PIN_BTN_ESTOP   11    // latching mushroom — LOW = pressed (latched)
#define PIN_LED_LINK    LED_BUILTIN
#define PIN_LED_ESTOP   12    // panel LED: solid red when E-stop latched

// ---------- LoRa modem ----------
// MKR WAN 1310 wires the Murata SiP to: NSS=LORA_IRQ_DUMB, IRQ=LORA_IRQ, RST=-1, GPIO=LORA_BOOT0.
// (See RadioLib's MKRWAN_1310 example for the canonical pin map.)
SX1276 radio = new Module(LORA_IRQ_DUMB, LORA_IRQ, LORA_RESET, LORA_BOOT0);

// ---------- pre-shared key (PROVISIONED ELSEWHERE — placeholder all-zero) ----------
// IP-008: production builds MUST replace this with the provisioned 16 bytes.
// `setup()` halts at boot if the key is still all zero AND real crypto is on,
// so accidentally flashing the example image cannot ship an unauthenticated link.
static const uint8_t kFleetKey[16] = {0};

static bool fleet_key_is_zero() {
    for (int i = 0; i < 16; i++) if (kFleetKey[i] != 0) return false;
    return true;
}

// ---------- state ----------
static uint16_t g_seq = 0;
static uint8_t  g_hb  = 0;
static uint32_t g_takectl_release_ms = 0;  // 0 = not latched
// Set true on inbound CMD_ESTOP from any peer; cleared only by an explicit
// CMD_CLEAR_ESTOP from BASE.
static bool     g_estop_remote_latched = false;

// FHSS counters — definitions for the externs referenced inside `send_frame()`'s
// LIFETRAC_FHSS_ENABLED block. File-scope (not static) so a future shared
// header could re-extern them without breaking the link.
uint32_t g_fhss_hop_counter = 0;
uint32_t g_fhss_key_id      = 0;
// Last RSSI we observed on any inbound frame — surfaced on the OLED so the
// operator can sanity-check link health before reaching for the cab.
static int16_t  g_last_rx_rssi_dbm = -128;
// IP-307: timestamp of the last successfully decoded inbound frame. Used
// to drive the OLED "NO LINK" boundary indicator so the operator can
// tell the difference between a healthy quiet link and a dead one
// (where ``g_last_rx_rssi_dbm`` would otherwise still display its last
// known value forever).
static uint32_t g_last_rx_ms       = 0;
static const uint32_t LINK_STALE_MS = 3000;   // ~3× the slowest HB period
// Per-source replay window. We only ever receive from tractor + base on the
// handheld, but sizing for 3 sources keeps the index calculus identical to
// tractor_m7.ino so the same lp_replay_check_and_update() call works.
static LpReplayWindow g_replay[3];

static const uint32_t TAKECTL_LATCH_MS = 30000;
static const uint32_t TICK_PERIOD_MS   = 50;   // 20 Hz

// ---------- adaptive SF ladder rung tracking ----------
// Mirrors the LADDER[] table in tractor_m7.ino so an inbound CMD_LINK_TUNE
// can map (sf, bw_code) back to a rung index for state + UI purposes.
struct LadderRung { uint8_t sf; uint16_t bw_khz; uint8_t cr_den; uint8_t bw_code; };
static const LadderRung LADDER[3] = {
    { 7, 250, 5, 1 },   // DECISIONS.md D-A2
    { 8, 125, 5, 0 },
    { 9, 125, 5, 0 },
};
static uint8_t g_ladder_rung = 0;

// ---------- KISS decoder for incoming bytes ----------
static KissDecoder g_dec;

// ---------- OLED (optional; init failure is non-fatal) ----------
#define OLED_W 128
#define OLED_H 64
#define OLED_ADDR 0x3C
static Adafruit_SSD1306 g_oled(OLED_W, OLED_H, &Wire, /*reset*/ -1);
static bool g_oled_ok = false;
static uint32_t g_next_oled_ms = 0;

// ---------- helpers ----------
// Per-axis deadband (counts on the 0..1023 ADC). Keeps neutral truly neutral.
static const int AXIS_DEADBAND = 16;

static int8_t read_axis(int pin) {
    int raw = analogRead(pin);            // 0..1023
    int centered = raw - 512;             // -512..+511
    if (centered > -AXIS_DEADBAND && centered < AXIS_DEADBAND) return 0;
    // Stretch the live region (deadband..511) back to (0..127) so we don't lose range.
    int sign = centered < 0 ? -1 : 1;
    int mag  = (centered < 0 ? -centered : centered) - AXIS_DEADBAND;
    long scaled = (long)mag * 127 / (512 - AXIS_DEADBAND);
    if (scaled > 127) scaled = 127;
    return (int8_t)(sign * scaled);
}

// Per-button debounce — require the same level for >= DEBOUNCE_MS before
// reporting it. Cheap shift-register style filter so we don't allocate.
static const uint32_t DEBOUNCE_MS = 15;
static uint8_t  s_btn_state    = 0;       // currently reported (debounced) state
static uint8_t  s_btn_candidate = 0;      // last raw read
static uint32_t s_btn_change_ms = 0;

static uint16_t read_buttons() {
    uint8_t raw = 0;
    for (int i = 0; i < 8; i++) {
        if (digitalRead(PIN_BTN_BASE + i) == LOW) raw |= (1u << i);
    }
    uint32_t now = millis();
    if (raw != s_btn_candidate) {
        s_btn_candidate = raw;
        s_btn_change_ms = now;
    } else if ((now - s_btn_change_ms) >= DEBOUNCE_MS) {
        s_btn_state = raw;
    }
    return (uint16_t)s_btn_state;
}

// Build a 12-byte AES-GCM nonce: [source_id | seq(LE) | t_seconds(LE) | rand(5)]
// Identical layout to base_station/lora_proto.py build_nonce().
static void build_nonce(uint8_t out[12], uint16_t seq) {
    out[0] = SRC_HANDHELD;
    out[1] = (uint8_t)(seq & 0xFF);
    out[2] = (uint8_t)(seq >> 8);
    uint32_t t = millis() / 1000;
    out[3] = (uint8_t)(t      ); out[4] = (uint8_t)(t >>  8);
    out[5] = (uint8_t)(t >> 16); out[6] = (uint8_t)(t >> 24);
    for (int i = 0; i < 5; i++) out[7 + i] = (uint8_t)random(0, 256);
}

// Encrypt + KISS-frame + transmit.
//
// `seq` MUST be the same value that was stamped into the frame header — the
// nonce binds the ciphertext to that exact sequence number, so passing the
// post-incremented `g_seq` here would put the nonce one step ahead of the
// header and the receiver would silently fail AEAD verification.
static void send_frame(const uint8_t* pt, size_t pt_len, uint16_t seq) {
    uint8_t nonce[12];
    build_nonce(nonce, seq);

    uint8_t enc[160];
    if (!lp_encrypt(kFleetKey, nonce, pt, pt_len, enc)) return;
    size_t enc_len = pt_len + 16;

    // On-air payload: [ nonce(12) | ciphertext+tag(enc_len) ]
    uint8_t onair[12 + 160];
    memcpy(onair,      nonce, 12);
    memcpy(onair + 12, enc,   enc_len);

    uint8_t kiss[2 * (12 + 160) + 2];
    size_t kiss_len = lp_kiss_encode(onair, 12 + enc_len, kiss, sizeof(kiss));
    if (kiss_len == 0) return;

#ifdef LIFETRAC_FHSS_ENABLED
    // CSMA skip-busy hop selection (DECISIONS.md D-C6). Only enabled when the
    // FHSS path is compiled in, because the per-hop scanChannel call adds a
    // few ms to every TX and we don't pay that on the single-channel default.
    extern uint32_t g_fhss_hop_counter;     // owned by the loop()
    extern uint32_t g_fhss_key_id;
    auto sampler = [](uint32_t hz, void* /*ctx*/) -> int16_t {
        radio.setFrequency(hz / 1.0e6f);
        // RadioLib::scanChannel returns RADIOLIB_LORA_DETECTED if a preamble
        // is heard, else RADIOLIB_CHANNEL_FREE. We translate that into a
        // sentinel dBm so lp_csma_pick_hop's threshold compare works.
        int state = radio.scanChannel();
        return (state == RADIOLIB_LORA_DETECTED) ? (int16_t)-40 : (int16_t)-110;
    };
    uint8_t skips = 0;
    uint32_t hop = lp_csma_pick_hop(g_fhss_key_id, g_fhss_hop_counter,
                                    sampler, nullptr,
                                    LP_CSMA_DEFAULT_BUSY_DBM,
                                    LP_CSMA_DEFAULT_MAX_SKIPS, &skips);
    radio.setFrequency(lp_fhss_channel_hz(g_fhss_key_id, hop) / 1.0e6f);
    g_fhss_hop_counter = hop + 1;
    if (skips > 0) {
        // Audit-log hook — surface skip count via the next heartbeat flags
        // byte once the telemetry pipeline carries link health.
        (void)skips;
    }
#endif

    radio.transmit(kiss, kiss_len);
    digitalWrite(PIN_LED_LINK, !digitalRead(PIN_LED_LINK));
    // RadioLib leaves the SX1276 in standby after transmit(); re-arm RX or
    // we'll never see another inbound frame.
    radio.startReceive();
}

// Apply a (sf, bw_khz, cr_den) PHY profile to the local radio. Returns the
// rung index 0..2 if the request matches a known LADDER row, else 0xFF.
static uint8_t apply_phy_rung(uint8_t sf, uint16_t bw_khz, uint8_t cr_den) {
    uint8_t rung = 0xFF;
    for (uint8_t r = 0; r < 3; r++) {
        if (LADDER[r].sf == sf && LADDER[r].bw_khz == bw_khz && LADDER[r].cr_den == cr_den) {
            rung = r;
            break;
        }
    }
    if (rung == 0xFF) return 0xFF;     // unknown profile — ignore
    radio.setSpreadingFactor(sf);
    radio.setBandwidth((float)bw_khz);
    radio.setCodingRate(cr_den);
    radio.startReceive();
    g_ladder_rung = rung;
    return rung;
}

static int src_index(uint8_t source_id) {
    switch (source_id) {
        case SRC_HANDHELD: return 0;
        case SRC_BASE:     return 1;
        case SRC_AUTONOMY: return 2;
        // Tractor TX shares slot 0 with our own SRC_HANDHELD slot. We never
        // receive our own frames over the air, so the only thing landing in
        // slot 0 is tractor → handheld traffic, and the per-slot replay
        // window remains correct against that single peer.
        case SRC_TRACTOR:  return 0;
        default:           return -1;
    }
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
    if (!lp_replay_check_and_update(&g_replay[idx], hdr->sequence_num)) return;
    g_last_rx_rssi_dbm = rssi_dbm;
    g_last_rx_ms       = millis();   // IP-307

    // We only act on commands. The handheld is a controller, not a logger;
    // telemetry/heartbeat from the tractor are surfaced on the OLED via the
    // RSSI side effect above, but we don't decode them further yet.
    if (hdr->frame_type != FT_COMMAND) return;
    if (pt_len < sizeof(LoraHeader) + 1 + 2) return;     // hdr + opcode + crc
    uint8_t op = pt[sizeof(LoraHeader)];
    const uint8_t* arg = pt + sizeof(LoraHeader) + 1;

    switch (op) {
        case CMD_LINK_TUNE: {
            // arg[0]=target SF, arg[1]=bw_code (0=125, 1=250, 2=500), arg[2]=reason.
            uint8_t target_sf = arg[0];
            uint8_t bw_code   = arg[1];
            uint16_t bw_khz = (bw_code == 0) ? 125 : (bw_code == 1) ? 250 : 500;
            apply_phy_rung(target_sf, bw_khz, /*cr_den*/ 5);
            break;
        }
        case CMD_ESTOP:
            g_estop_remote_latched = true;
            digitalWrite(PIN_LED_ESTOP, HIGH);
            break;
        case CMD_CLEAR_ESTOP:
            // Honour clears only from BASE per MASTER_PLAN.md §8.5 (operator at console).
            if (hdr->source_id == SRC_BASE) {
                g_estop_remote_latched = false;
                digitalWrite(PIN_LED_ESTOP, LOW);
            }
            break;
        default:
            break;
    }
}

// Pull bytes off the LoRa modem and feed the KISS decoder.
static void poll_radio() {
    int packet_len = radio.getPacketLength();
    if (packet_len <= 0) return;
    uint8_t buf[256];
    int st = radio.readData(buf, packet_len);
    if (st != RADIOLIB_ERR_NONE) {
        radio.startReceive();
        return;
    }
    int16_t rssi = radio.getRSSI();
    for (int i = 0; i < packet_len; i++) {
        uint8_t* frame; size_t flen;
        if (lp_kiss_feed(&g_dec, buf[i], &frame, &flen)) {
            process_air_frame(frame, flen, rssi);
        }
    }
    radio.startReceive();
}

// Build + send a CMD_ESTOP frame at P0 priority. This is invoked when the
// local mushroom button latches LOW. The tractor M7 will set its own
// g_estop_latched on receipt and force valves neutral on the next 50 Hz tick.
static void send_estop() {
    CommandFrame cf;
    uint16_t seq = g_seq++;
    size_t cmd_len = lp_make_command(&cf, SRC_HANDHELD, seq, CMD_ESTOP, NULL, 0);
    send_frame((const uint8_t*)&cf, cmd_len, seq);
}

// Render a one-screen status page on the SSD1306. Cheap to call ~5 Hz.
static void update_oled(int8_t lhx, int8_t lhy, int8_t rhx, int8_t rhy,
                        bool takectl_held, bool takectl_latched, bool estop_local) {
    if (!g_oled_ok) return;
    g_oled.clearDisplay();
    g_oled.setTextColor(SSD1306_WHITE);
    g_oled.setTextSize(1);

    g_oled.setCursor(0, 0);
    g_oled.print(F("LifeTrac v25 HANDHELD"));

    g_oled.setCursor(0, 12);
    g_oled.print(F("SF"));   g_oled.print(LADDER[g_ladder_rung].sf);
    g_oled.print(F(" BW")); g_oled.print(LADDER[g_ladder_rung].bw_khz);
    // IP-307: only show the cached RSSI when the link is fresh; otherwise
    // print "--" so a stale value can't masquerade as a live link.
    bool link_fresh = (g_last_rx_ms != 0) &&
                      ((millis() - g_last_rx_ms) < LINK_STALE_MS);
    g_oled.print(F(" "));
    if (link_fresh) {
        g_oled.print(g_last_rx_rssi_dbm); g_oled.print(F("dBm"));
    } else {
        g_oled.print(F("--dBm"));
    }

    g_oled.setCursor(0, 24);
    g_oled.print(F("LH "));  g_oled.print(lhx); g_oled.print(F(",")); g_oled.print(lhy);
    g_oled.print(F("  RH ")); g_oled.print(rhx); g_oled.print(F(",")); g_oled.print(rhy);

    g_oled.setCursor(0, 36);
    if (estop_local || g_estop_remote_latched) {
        g_oled.setTextSize(2);
        g_oled.print(F("ESTOP"));
        g_oled.setTextSize(1);
    } else if (takectl_held || takectl_latched) {
        g_oled.print(F("TAKECTL "));
        if (takectl_latched && !takectl_held) {
            uint32_t left = (g_takectl_release_ms > millis())
                          ? (g_takectl_release_ms - millis()) / 1000 : 0;
            g_oled.print(left); g_oled.print(F("s"));
        } else {
            g_oled.print(F("HELD"));
        }
    } else if (!link_fresh) {
        // IP-307: surface the no-source-active boundary explicitly so
        // "OK" never displays while the tractor is unreachable.
        g_oled.print(F("NO LINK"));
    } else {
        g_oled.print(F("OK"));
    }

    g_oled.display();
}

// ---------- Arduino entry points ----------
void setup() {
    Serial.begin(115200);
    pinMode(PIN_LED_LINK,  OUTPUT);
    pinMode(PIN_LED_ESTOP, OUTPUT);
    pinMode(PIN_BTN_TAKECTL, INPUT_PULLUP);
    pinMode(PIN_BTN_ESTOP,   INPUT_PULLUP);
    for (int i = 0; i < 8; i++) pinMode(PIN_BTN_BASE + i, INPUT_PULLUP);

    int st = radio.begin(915.0,
                         (float)LADDER[0].bw_khz,   // IP-006: match LADDER[0] (BW250)
                         LADDER[0].sf,
                         LADDER[0].cr_den,
                         0x12,                       // sync word — private LoRa
                         14);                        // tx power dBm (Murata SiP max for MKR)
    if (st != RADIOLIB_ERR_NONE) {
        Serial.print("LoRa begin failed: "); Serial.println(st);
        while (1) { delay(1000); }
    }
    radio.startReceive();
    g_ladder_rung = 0;

#ifndef LIFETRAC_ALLOW_UNCONFIGURED_KEY
    // §D (Round 9): IP-008 enforced unconditionally. Previously this was
    // gated behind LIFETRAC_USE_REAL_CRYPTO, which meant a stub-crypto
    // build accidentally flashed onto real hardware would happily TX with
    // an all-zero key. Define LIFETRAC_ALLOW_UNCONFIGURED_KEY only for
    // bench builds where you understand the consequences.
    if (fleet_key_is_zero()) {
        if (g_oled_ok) {
            g_oled.clearDisplay();
            g_oled.setTextSize(1); g_oled.setCursor(0, 0);
            g_oled.print(F("FLEET KEY NOT\nPROVISIONED\nHALT (IP-008)"));
            g_oled.display();
        }
        Serial.println("FATAL: fleet key all-zero — refusing to TX (IP-008)");
        while (1) { delay(1000); }
    }
#endif

    for (int i = 0; i < 3; i++) lp_replay_init(&g_replay[i]);

    // OLED is optional — if init fails we just skip rendering. The handheld
    // remains fully functional without it.
    Wire.begin();
    if (g_oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        g_oled_ok = true;
        g_oled.clearDisplay();
        g_oled.setTextColor(SSD1306_WHITE);
        g_oled.setTextSize(1);
        g_oled.setCursor(0, 0);
        g_oled.print(F("LifeTrac v25 HANDHELD"));
        g_oled.setCursor(0, 12);
        g_oled.print(F("boot OK"));
        g_oled.display();
    }

    // IP-301: anchor the button-debounce reference to a real timestamp so
    // the very first read_buttons() call after boot can never satisfy
    // ``(now - s_btn_change_ms) >= DEBOUNCE_MS`` against the
    // uninitialised-zero default. Without this, a noisy raw==0 read on
    // the first iteration would commit ``s_btn_state = 0`` immediately
    // (harmless today but a footgun for any future non-zero idle state).
    s_btn_change_ms = millis();
}

void loop() {
    // Drain RX every loop iteration so an inbound CMD_LINK_TUNE applies
    // before our next TX (otherwise we transmit on the old PHY and the
    // tractor doesn't hear us).
    poll_radio();

    static uint32_t next_tick = 0;
    uint32_t now = millis();
    if ((int32_t)(now - next_tick) < 0) return;
    next_tick = now + TICK_PERIOD_MS;

    // ---- inputs
    int8_t   lhx = read_axis(PIN_LH_X);
    int8_t   lhy = read_axis(PIN_LH_Y);
    int8_t   rhx = read_axis(PIN_RH_X);
    int8_t   rhy = read_axis(PIN_RH_Y);
    uint16_t btns = read_buttons();

    // E-STOP button (latching mushroom; LOW = pressed). Edge-trigger on
    // press so we send exactly one CMD_ESTOP per latch event — the tractor's
    // sliding replay window de-dupes anyway, but this keeps the airwaves clean.
    static bool s_estop_prev = false;
    bool estop_local = (digitalRead(PIN_BTN_ESTOP) == LOW);
    if (estop_local && !s_estop_prev) {
        send_estop();
        digitalWrite(PIN_LED_ESTOP, HIGH);
    } else if (!estop_local) {
        // Local button physically released. We do NOT auto-clear the latched
        // remote state — that requires an explicit operator action at the base
        // console (CMD_CLEAR_ESTOP from BASE).
        if (!g_estop_remote_latched) digitalWrite(PIN_LED_ESTOP, LOW);
    }
    s_estop_prev = estop_local;

    // TAKE CONTROL latch — held while button is down; latches 30 s after release.
    bool takectl_held = (digitalRead(PIN_BTN_TAKECTL) == LOW);
    if (takectl_held) {
        g_takectl_release_ms = now + TAKECTL_LATCH_MS;
    }
    bool takectl_latched = (now < g_takectl_release_ms);
    uint8_t flags = 0;
    if (takectl_held || takectl_latched) flags |= FLAG_TAKECTL_HELD;
    if (estop_local || g_estop_remote_latched) flags |= FLAG_ESTOP_ARMED;

    // ---- control frame
    ControlFrame cf;
    uint16_t cf_seq = g_seq++;
    lp_make_control(&cf, SRC_HANDHELD, cf_seq, lhx, lhy, rhx, rhy, btns, flags, g_hb);
    send_frame((const uint8_t*)&cf, sizeof(cf), cf_seq);

    // ---- heartbeat (one per tick — same 20 Hz cadence as control)
    HeartbeatFrame hb;
    uint16_t hb_seq = g_seq++;
    lp_make_heartbeat(&hb, SRC_HANDHELD, hb_seq, /*priority*/1, flags);
    send_frame((const uint8_t*)&hb, sizeof(hb), hb_seq);

    g_hb++;

    // OLED update at ~5 Hz (cheap but I²C still costs ~10 ms per frame).
    if ((int32_t)(now - g_next_oled_ms) >= 0) {
        g_next_oled_ms = now + 200;
        update_oled(lhx, lhy, rhx, rhy, takectl_held, takectl_latched, estop_local);
    }
}
