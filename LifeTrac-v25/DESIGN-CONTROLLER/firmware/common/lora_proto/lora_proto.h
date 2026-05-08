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
// Source of truth: DESIGN-CONTROLLER/LORA_PROTOCOL.md.
#define CMD_ESTOP             0x01
#define CMD_CLEAR_ESTOP       0x02
#define CMD_CAMERA_SELECT     0x03
#define CMD_CAMERA_QUALITY    0x04
#define CMD_PLAN_COMMIT       0x10
#define CMD_LINK_HINT         0x20
#define CMD_LINK_TUNE         0x21
#define CMD_PERSON_APPEARED   0x60
#define CMD_ROI_HINT          0x61
#define CMD_REQ_KEYFRAME      0x62
#define CMD_ENCODE_MODE       0x63

// Telemetry topic IDs (MQTT-SN-style static table).
#define TOPIC_GPS             0x01
#define TOPIC_ENGINE          0x02
#define TOPIC_BATTERY         0x03
#define TOPIC_HYDRAULICS      0x04
#define TOPIC_MODE            0x05
#define TOPIC_ERRORS          0x06
#define TOPIC_IMU             0x07
#define TOPIC_SENSOR_FAULTS   0x08
#define TOPIC_SOURCE_ACTIVE   0x10
#define TOPIC_VIDEO_THUMB     0x20
#define TOPIC_VIDEO_REAR      0x21
#define TOPIC_ACTIVE_CAMERA   0x22
#define TOPIC_VIDEO_IMPLEMENT 0x23
#define TOPIC_CROP_HEALTH     0x24
#define TOPIC_TILE_DELTA      0x25
#define TOPIC_DETECTIONS      0x26
#define TOPIC_AUDIO_EVENT     0x27
#define TOPIC_MOTION_VECTORS  0x28
#define TOPIC_WIREFRAME       0x29
#define TOPIC_SEMANTIC_MAP    0x2A   // reserved for v26; do not emit in v25

typedef enum {
    BADGE_RAW          = 0,
    BADGE_CACHED       = 1,
    BADGE_ENHANCED     = 2,
    BADGE_RECOLOURISED = 3,
    BADGE_PREDICTED    = 4,
    BADGE_SYNTHETIC    = 5,
    BADGE_WIREFRAME    = 6,
} BadgeKind;

typedef enum {
    ENCODE_FULL        = 0,
    ENCODE_Y_ONLY      = 1,
    ENCODE_MOTION_ONLY = 2,
    ENCODE_WIREFRAME   = 3,
} EncodeMode;

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
    uint8_t    payload_len;  // 0..118 (IP-306: usable max; the trailing
                             //         CRC-16 is stored inside ``payload[]``,
                             //         consuming 2 bytes of the 120-byte
                             //         buffer.)
    // payload bytes follow inline; CRC-16 follows payload
    uint8_t    payload[120];
} TelemetryFrame;

// Variable-length on the wire (8 bytes minimum, 16 bytes maximum).
//
// IMPORTANT (IP-108): the on-wire layout is
//     [ hdr(5) | opcode(1) | arg[arg_len] | crc16(2) ]
// where ``arg_len`` may be 0..8 inclusive. ``arg[]`` and ``crc16`` here are
// only the maximum-size storage; ``lp_make_command()`` writes the CRC
// IMMEDIATELY after the actual arg bytes (i.e. into ``arg[arg_len]`` for
// short commands) and leaves the trailing ``arg[]``/``crc16`` storage
// undefined. Callers MUST use the size returned by ``lp_make_command()``
// rather than ``sizeof(CommandFrame)`` and MUST NOT read ``f->crc16``
// directly — use the last two bytes of the returned slice instead.
typedef struct {
    LoraHeader hdr;          // frame_type = FT_COMMAND
    uint8_t  opcode;         // CMD_*
    // up to 8 bytes of opcode-specific payload
    uint8_t  arg[8];
    uint16_t crc16;          // see note above — do NOT read directly
} CommandFrame;

typedef struct {
    uint8_t  sf;
    uint16_t bw_khz;
    uint8_t  cr_den;         // 5 => 4/5, 8 => 4/8
    uint8_t  preamble_len;
} LoraPhyProfile;

#pragma pack(pop)

// Canonical v25 PHY profiles from LORA_IMPLEMENTATION.md.
extern const LoraPhyProfile LP_PHY_CONTROL_SF7;
extern const LoraPhyProfile LP_PHY_CONTROL_SF8;
extern const LoraPhyProfile LP_PHY_CONTROL_SF9;
extern const LoraPhyProfile LP_PHY_TELEMETRY;
extern const LoraPhyProfile LP_PHY_IMAGE;

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

// AES-128-GCM wrappers. Real backends live in lp_crypto_real.cpp; the no-op
// stub in crypto_stub.c is gated behind LIFETRAC_ALLOW_STUB_CRYPTO.
//
// Crypto Ownership Contract (Profile A):
// --------------------------------------
// The Host CPU (H747/SAMD21) ENCRYPTS data natively and is the sole owner of AES-GCM
// processing, maintaining the nonces, keys, and replay tracking state logic.
// The L072 AT_Slave/Custom Firmware acts only as a dumb transport for ciphertext.
//
// `key`   : pre-shared 16-byte fleet key.
// `nonce` : 12 bytes [source_id(1) | sequence(2) | timestamp_s(4) | random(5)].
//           Per N-13, the L072 *only* contributes the 5 random bytes ONCE upon
//           booting and ships it to the H7 over the BOOT_URC binary command.
//
// CONTRACT (IP-003):
//   * lp_encrypt: writes `pt_len` ciphertext bytes followed by `LP_TAG_LEN`
//     auth tag bytes into `out`. Caller must size `out` to pt_len + LP_TAG_LEN.
//   * lp_decrypt: `ct_len` is the TOTAL length of ciphertext + auth tag
//     (i.e. `pt_len + LP_TAG_LEN`). Recovered plaintext is `ct_len - LP_TAG_LEN`
//     bytes long; caller must size `pt` accordingly.
//
// Returns true on success; lp_decrypt returns false on tag mismatch / tamper.
#define LP_NONCE_LEN  12
#define LP_TAG_LEN    16

bool lp_encrypt(const uint8_t* key, const uint8_t* nonce,
                const uint8_t* pt, size_t pt_len,
                uint8_t* out);

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

// Convenience: build a Command frame with up to 8 arg bytes and stamp CRC.
// Returns the exact byte length to transmit: header + opcode + args + crc16.
size_t lp_make_command(CommandFrame* f,
                       uint8_t source_id, uint16_t seq,
                       uint8_t opcode, const uint8_t* arg, uint8_t arg_len);

// LoRa airtime estimate in milliseconds for an unencrypted LoRa payload length.
// This is used for bench planning and host-side assertions; hardware truth still
// comes from RadioLib::getTimeOnAir() on the target radio.
uint32_t lp_lora_airtime_ms(uint8_t payload_len,
                            uint8_t sf,
                            uint16_t bw_khz,
                            uint8_t cr_den,
                            uint8_t preamble_len);

// 8-channel FHSS sequence for US 915 MHz v25. `hop_counter` advances once per
// dwell window; `key_id` is the 32-bit shared AES key identifier.
uint8_t lp_fhss_channel_index(uint32_t key_id, uint32_t hop_counter);
uint32_t lp_fhss_channel_hz(uint32_t key_id, uint32_t hop_counter);

// -------------------------------------------------------------------------
// CSMA "skip-busy" wrapper around the deterministic FHSS hop sequence.
//
// Before transmitting on the next hop, sample carrier RSSI; if it reads
// above `busy_threshold_dbm`, advance to the next hop in the deterministic
// sequence and try again, bounded by `max_skips`. If every probe is busy
// we return the last candidate so the frame still goes out (a control
// frame missing its dwell hurts safety more than a small CCA miss).
//
// The sampler is supplied by the caller because radio access varies by node
// (RadioLib::scanChannel on tractor/handheld, SX1276 driver on base-SPI).
// `*skips_out` (optional) reports how many candidates were busy; non-zero
// values should be audit-logged per LORA_IMPLEMENTATION.md §8.10.
// -------------------------------------------------------------------------
#define LP_CSMA_DEFAULT_BUSY_DBM   (-90)
#define LP_CSMA_DEFAULT_MAX_SKIPS  4

typedef int16_t (*lp_rssi_sampler_fn)(uint32_t channel_hz, void* ctx);

uint32_t lp_csma_pick_hop(uint32_t key_id,
                          uint32_t start_hop,
                          lp_rssi_sampler_fn sample,
                          void* ctx,
                          int16_t busy_threshold_dbm,
                          uint8_t max_skips,
                          uint8_t* skips_out);

// -------------------------------------------------------------------------
// Replay-defence sliding window (per-source).
//
// AES-GCM authenticates that a frame is from somebody who knows the key, but
// it does NOT prevent an attacker from replaying captured ciphertext. The
// header sequence number is monotonic per source, so we maintain a 64-frame
// sliding window per source and reject any seq we have already seen or any
// seq more than 64 below the current high-water mark. Out-of-order arrivals
// up to 63 frames stale are accepted exactly once.
//
// 16-bit seq wraps every 65 536 frames; on wrap we accept the new frame as
// fresh provided the gap from the high-water mark is plausible (>32 768).
// -------------------------------------------------------------------------
#define LP_REPLAY_WINDOW_BITS 64
typedef struct {
    uint16_t high_water;     // largest seq seen so far (post-init)
    uint64_t bitmap;         // bit i set => seq (high_water - i) was seen
    bool     primed;         // false until first frame initialises high_water
} LpReplayWindow;

void lp_replay_init(LpReplayWindow* w);
// Returns true if `seq` is fresh (not previously seen and within the window),
// false if it is a replay or too old. Updates window state on accept.
bool lp_replay_check_and_update(LpReplayWindow* w, uint16_t seq);

#ifdef __cplusplus
}
#endif

#endif // LIFETRAC_V25_LORA_PROTO_H
