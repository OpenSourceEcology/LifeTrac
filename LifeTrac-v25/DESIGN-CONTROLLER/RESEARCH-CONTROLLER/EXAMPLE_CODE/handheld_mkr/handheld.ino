// handheld.ino — MKR WAN 1310 handheld controller.
//
// DRAFT FOR REVIEW. Not compiled or tested.
//
// Reads two analog joysticks + 8 buttons + a momentary TAKE CONTROL button,
// packs a ControlFrame at 20 Hz, and emits a Heartbeat at the same cadence.
// LoRa PHY: SF7 / BW 500 kHz / CR 4/5 / 915 MHz (US).
//
// Expected boards/libs:
//   - Arduino MKR WAN 1310
//   - RadioLib >= 6.x
//   - lora_proto.h / .c included from ../lora_proto/

#include <Arduino.h>
#include <RadioLib.h>
#include "../lora_proto/lora_proto.h"

// ---------- pins ----------
#define PIN_LH_X        A0
#define PIN_LH_Y        A1
#define PIN_RH_X        A2
#define PIN_RH_Y        A3
#define PIN_BTN_BASE    2     // first of 8 contiguous button pins
#define PIN_BTN_TAKECTL 10    // momentary, latched in software for 30 s after release
#define PIN_LED_LINK    LED_BUILTIN

// ---------- LoRa modem ----------
// MKR WAN 1310 wires the Murata SiP to: NSS=LORA_IRQ_DUMB, IRQ=LORA_IRQ, RST=-1, GPIO=LORA_BOOT0.
// (See RadioLib's MKRWAN_1310 example for the canonical pin map.)
SX1276 radio = new Module(LORA_IRQ_DUMB, LORA_IRQ, LORA_RESET, LORA_BOOT0);

// ---------- pre-shared key (PROVISIONED ELSEWHERE — placeholder all-zero) ----------
static const uint8_t kFleetKey[16] = {0};

// ---------- state ----------
static uint16_t g_seq = 0;
static uint8_t  g_hb  = 0;
static uint32_t g_takectl_release_ms = 0;  // 0 = not latched

static const uint32_t TAKECTL_LATCH_MS = 30000;
static const uint32_t TICK_PERIOD_MS   = 50;   // 20 Hz

// ---------- helpers ----------
static int8_t read_axis(int pin) {
    int raw = analogRead(pin);            // 0..1023
    int centered = raw - 512;             // -512..+511
    long scaled = (long)centered * 127 / 512;
    if (scaled > 127) scaled = 127;
    if (scaled < -127) scaled = -127;
    return (int8_t)scaled;
}

static uint16_t read_buttons() {
    uint16_t b = 0;
    for (int i = 0; i < 8; i++) {
        if (digitalRead(PIN_BTN_BASE + i) == LOW) b |= (1u << i);
    }
    return b;
}

// Build a 12-byte AES-GCM nonce: [source_id | seq(LE) | t_seconds(LE) | rand(5)]
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
static void send_frame(const uint8_t* pt, size_t pt_len) {
    uint8_t nonce[12];
    build_nonce(nonce, g_seq);

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

    radio.transmit(kiss, kiss_len);
    digitalWrite(PIN_LED_LINK, !digitalRead(PIN_LED_LINK));
}

// ---------- Arduino entry points ----------
void setup() {
    Serial.begin(115200);
    pinMode(PIN_LED_LINK, OUTPUT);
    pinMode(PIN_BTN_TAKECTL, INPUT_PULLUP);
    for (int i = 0; i < 8; i++) pinMode(PIN_BTN_BASE + i, INPUT_PULLUP);

    int st = radio.begin(915.0,    // MHz
                         500.0,    // bandwidth kHz
                         7,        // SF
                         5,        // CR (4/5)
                         0x12,     // sync word — private LoRa
                         14);      // tx power dBm (Murata SiP max for MKR)
    if (st != RADIOLIB_ERR_NONE) {
        Serial.print("LoRa begin failed: "); Serial.println(st);
        while (1) { delay(1000); }
    }
}

void loop() {
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

    // TAKE CONTROL latch — held while button is down; latches 30 s after release.
    bool takectl_held = (digitalRead(PIN_BTN_TAKECTL) == LOW);
    if (takectl_held) {
        g_takectl_release_ms = now + TAKECTL_LATCH_MS;
    }
    bool takectl_latched = (now < g_takectl_release_ms);
    uint8_t flags = 0;
    if (takectl_held || takectl_latched) flags |= FLAG_TAKECTL_HELD;

    // ---- control frame
    ControlFrame cf;
    lp_make_control(&cf, SRC_HANDHELD, g_seq++, lhx, lhy, rhx, rhy, btns, flags, g_hb);
    send_frame((const uint8_t*)&cf, sizeof(cf));

    // ---- heartbeat (one per tick — same 20 Hz cadence as control)
    HeartbeatFrame hb;
    lp_make_heartbeat(&hb, SRC_HANDHELD, g_seq++, /*priority*/1, flags);
    send_frame((const uint8_t*)&hb, sizeof(hb));

    g_hb++;
}
