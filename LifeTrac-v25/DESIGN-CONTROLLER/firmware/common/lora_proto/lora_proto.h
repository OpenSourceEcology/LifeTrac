// lora_proto.h — shared frame definitions for LifeTrac v25 custom LoRa stack.
//
// DRAFT FOR REVIEW. Not compiled or tested.
//
// This header is the single source of truth for the wire format. It is included
// by the handheld sketch, the tractor sketch, and (via cffi or a C extension,
// or just hand-mirrored in Python) the base-station bridge. If any field
// changes, every node must be reflashed/restarted in lockstep.
//
// Companion spec: ../../../LORA_PROTOCOL.md

#ifndef LIFETRAC_V25_LORA_PROTO_H
#define LIFETRAC_V25_LORA_PROTO_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// -------------------------------------------------------------------------
// Protocol version & identifiers
// -------------------------------------------------------------------------

#define LIFETRAC_PROTO_VERSION  0x01

// source_id values
#define SRC_HANDHELD    0x01
#define SRC_BASE        0x02
#define SRC_TRACTOR     0x03   // telemetry only
#define SRC_AUTONOMY    0x04
#define SRC_NONE        0xFF

// frame_type values
#define FT_CONTROL      0x10
#define FT_TELEMETRY    0x20
#define FT_COMMAND      0x30   // E-stop, mode, key-rotate
#define FT_HEARTBEAT    0x40

// command opcodes (carried in COMMAND frame payload[0])
#define CMD_ESTOP        0x01
#define CMD_REQ_CONTROL  0x02
#define CMD_REKEY        0x03

// button bitmap positions for ControlFrame.buttons
#define BTN_BUCKET_CURL   (1u <<  0)
#define BTN_BUCKET_DUMP   (1u <<  1)
#define BTN_AUX1          (1u <<  2)
#define BTN_AUX2          (1u <<  3)
#define BTN_AUX3          (1u <<  4)
#define BTN_AUX4          (1u <<  5)
#define BTN_MODE          (1u <<  6)
#define BTN_TAKE_CONTROL  (1u <<  7)

// flags bitmap
#define FLAG_TAKECTL_HELD       (1u << 0)
#define FLAG_ESTOP_ARMED        (1u << 1)
#define FLAG_CELLULAR_FALLBACK  (1u << 2)

// -------------------------------------------------------------------------
// On-air frames (packed)
// -------------------------------------------------------------------------

#pragma pack(push, 1)

// 5-byte common header — present in every frame
typedef struct {
    uint8_t  version;        // = LIFETRAC_PROTO_VERSION
    uint8_t  source_id;      // SRC_*
    uint8_t  frame_type;     // FT_*
    uint16_t sequence_num;   // monotonic per source; rollover OK
} LoraHeader;

// Total: 5 hdr + 4 axes + 2 buttons + 1 flags + 1 hb + 1 reserved + 2 crc = 16 bytes.
// The reserved byte is part of the on-wire format and MUST be zero on TX and
// ignored on RX (so we can repurpose it later without bumping protocol version).
typedef struct {
    LoraHeader hdr;          // frame_type = FT_CONTROL
    int8_t   axis_lh_x;      // -127..+127, drive turn
    int8_t   axis_lh_y;      // drive forward/reverse
    int8_t   axis_rh_x;      // implement aux
    int8_t   axis_rh_y;      // boom lift
    uint16_t buttons;        // BTN_* bitmap
    uint8_t  flags;          // FLAG_* bitmap
    uint8_t  heartbeat_ctr;  // wraps every 256
    uint8_t  reserved;       // MUST be 0 on TX; ignored on RX
    uint16_t crc16;          // CRC-16/CCITT over all preceding bytes
} ControlFrame;

// 10 bytes
typedef struct {
    LoraHeader hdr;             // frame_type = FT_HEARTBEAT
    uint8_t  priority_request;  // 1 = HANDHELD, 2 = BASE, 3 = AUTONOMY (echoes source_id intent)
    uint8_t  flags;             // FLAG_TAKECTL_HELD etc.
    uint8_t  reserved;
    uint16_t crc16;
} HeartbeatFrame;

// Variable length 7..128 bytes
typedef struct {
    LoraHeader hdr;          // frame_type = FT_TELEMETRY
    uint8_t    topic_id;     // statically registered MQTT-SN topic ID
    uint8_t    payload_len;  // 0..120
    // payload bytes follow inline; CRC-16 follows payload
    uint8_t    payload[120];
} TelemetryFrame;

// 8 bytes minimum (more if opcode-specific data present)
typedef struct {
    LoraHeader hdr;          // frame_type = FT_COMMAND
    uint8_t  opcode;         // CMD_*
    // up to 8 bytes of opcode-specific payload
    uint8_t  arg[8];
    uint16_t crc16;
} CommandFrame;

#pragma pack(pop)

// -------------------------------------------------------------------------
// Pack / unpack / framing API
// -------------------------------------------------------------------------

// Compute CRC-16/CCITT (poly 0x1021, init 0xFFFF) over `len` bytes.
uint16_t lp_crc16(const uint8_t* buf, size_t len);

// KISS framing: wrap `in` with FEND markers and byte-stuff FEND/FESC.
// Returns number of bytes written to `out`. `out_max` should be >= 2*in_len + 2.
size_t lp_kiss_encode(const uint8_t* in, size_t in_len,
                      uint8_t* out, size_t out_max);

// KISS de-framer state — feed bytes one at a time; returns true once a complete
// (un-stuffed, FEND-stripped) frame is in `frame_out`. Caller resets length to 0.
typedef struct {
    uint8_t buf[256];
    size_t  len;
    bool    in_frame;
    bool    escape_next;
} KissDecoder;

bool lp_kiss_feed(KissDecoder* dec, uint8_t b,
                  uint8_t** frame_out, size_t* frame_len);

// AES-128-GCM wrappers — implementations live in crypto_stub.c (TODO: real
// MbedTLS / ArduinoBearSSL bindings). Returns true on success.
//
// `key` is the pre-shared 16-byte fleet key.
// `nonce` is 12 bytes: [source_id(1) | sequence(2) | timestamp_s(4) | random(5)].
// `out` must be pt_len + 16 (auth tag) bytes.
bool lp_encrypt(const uint8_t* key, const uint8_t* nonce,
                const uint8_t* pt, size_t pt_len,
                uint8_t* out);

// Returns true and fills `pt` if the auth tag verifies; false on tamper/replay.
bool lp_decrypt(const uint8_t* key, const uint8_t* nonce,
                const uint8_t* ct, size_t ct_len,
                uint8_t* pt);

// Convenience: build a ControlFrame from raw inputs and stamp CRC.
void lp_make_control(ControlFrame* f,
                     uint8_t source_id, uint16_t seq,
                     int8_t lhx, int8_t lhy, int8_t rhx, int8_t rhy,
                     uint16_t buttons, uint8_t flags, uint8_t hb_ctr);

// Convenience: build a Heartbeat from raw inputs and stamp CRC.
void lp_make_heartbeat(HeartbeatFrame* f,
                       uint8_t source_id, uint16_t seq,
                       uint8_t priority_request, uint8_t flags);

#ifdef __cplusplus
}
#endif

#endif // LIFETRAC_V25_LORA_PROTO_H
