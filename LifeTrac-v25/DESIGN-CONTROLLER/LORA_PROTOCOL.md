# LoRa Protocol — LifeTrac v25 Custom Stack

This document defines the air-interface protocol used between the Handheld, Tractor, and Base Station. The design goals come from [ARCHITECTURE.md](ARCHITECTURE.md):

- **Bounded latency** — control round-trip ≤ 150 ms (handheld) / ≤ 250 ms (base station)
- **Multi-source arbitration** — strict-priority handover between handheld, base, autonomy
- **Loss-tolerant** — no ACK/retry on the control path
- **Authenticated** — AES-128-GCM payload encryption + replay protection
- **Portable** — same firmware across MKR WAN 1310, Portenta H7, Portenta X8 (only pin map changes)

## Why not LoRaWAN?

LoRaWAN's join procedure (~1 s), duty-cycle enforcement, ADR back-off, and gateway-mediated routing all add latency and indeterminism that are incompatible with sub-second motor control. We use LoRa **physical layer only** (Semtech SX1276 chirp spread spectrum) with our own MAC, framing, security, and application layers — a pattern sometimes called "LoRa raw" or "LoRa P2P."

## Protocol stack

```
┌─────────────────────────────────────────────┐
│  Application                                │  Control / Telemetry / Command
│  - 16-byte ControlFrame                      │
│  - 32-128 byte TelemetryFrame (MQTT-SN)      │
├─────────────────────────────────────────────┤
│  Session                                    │  AES-128-GCM, 4-byte nonce
│  - SourceID (1 byte: HANDHELD/BASE/AUTO)     │
│  - SequenceNum (2 bytes)                     │
├─────────────────────────────────────────────┤
│  Frame                                      │  KISS-style framing
│  - FEND (0xC0) start/end                     │
│  - FESC (0xDB) byte stuffing                 │
│  - CRC-16/CCITT trailer                      │
├─────────────────────────────────────────────┤
│  MAC                                        │  Talk-when-quiet CSMA
│  - Channel-busy detect via RSSI              │
│  - Random backoff if busy                    │
├─────────────────────────────────────────────┤
│  PHY (RadioLib)                             │  Semtech SX1276
│  - SF7-SF9, BW 250-500 kHz, CR 4/5           │
│  - 915 MHz (US) or 868 MHz (EU)              │
└─────────────────────────────────────────────┘
```

## PHY parameters (defaults)

| Parameter | Control link | Telemetry link |
|---|---|---|
| Spreading Factor | SF7 | SF9 |
| Bandwidth | 500 kHz | 250 kHz |
| Coding Rate | 4/5 | 4/8 |
| Sync word | 0x12 (private LoRa) | 0x12 |
| Preamble length | 8 symbols | 12 symbols |
| TX power | per-source (see below) | per-source |
| Air time, 16-byte frame | ~15 ms | ~75 ms |
| Air time, 64-byte frame | ~25 ms | ~250 ms |

TX power per source:

| Source | TX power | Notes |
|---|---|---|
| Handheld (MKR WAN 1310) | +14 dBm | Murata SiP max for the MKR's matching network |
| Tractor (Max Carrier) | +20 dBm | Murata SiP max |
| Base station (Max Carrier) | +20 dBm | Murata SiP max; gain comes from 8 dBi mast antenna |

## Frame format

### Common header (5 bytes)

| Field | Bytes | Description |
|---|---:|---|
| `version` | 1 | Protocol version (currently `0x01`) |
| `source_id` | 1 | `0x01` HANDHELD, `0x02` BASE, `0x03` TRACTOR (telemetry), `0x04` AUTONOMY |
| `frame_type` | 1 | `0x10` ControlFrame, `0x20` TelemetryFrame, `0x30` Command, `0x40` Heartbeat |
| `sequence_num` | 2 | Monotonic per source; rollover handled |

### ControlFrame (16 bytes total: 5 header + 11 payload)

| Field | Bytes | Description |
|---|---:|---|
| (header) | 5 | as above, `frame_type = 0x10` |
| `axis_lh_x` | 1 | Left joystick X, signed int8 (-127 .. +127) — drive turn |
| `axis_lh_y` | 1 | Left joystick Y — drive forward/reverse |
| `axis_rh_x` | 1 | Right joystick X — implement aux |
| `axis_rh_y` | 1 | Right joystick Y — boom lift |
| `buttons` | 2 | Bitmap: bucket curl, dump, aux 1-4, mode, take-control |
| `flags` | 1 | bit0=takecontrol_active, bit1=estop_armed, bit2=cellular_fallback |
| `heartbeat_ctr` | 1 | Wraps every 256; tractor uses for liveness check |
| `crc16` | 2 | CRC-16/CCITT over header+payload (post-encryption) |

After AES-GCM encryption + 12-byte nonce + 16-byte tag → on-air payload ~44 bytes. Plus KISS framing overhead (~5%) → ~46 bytes on the air.

### TelemetryFrame (variable, 32–128 bytes)

| Field | Bytes | Description |
|---|---:|---|
| (header) | 5 | as above, `frame_type = 0x20`, `source_id = 0x03` (TRACTOR) |
| `mqtt_sn_payload` | n | MQTT-SN PUBLISH packet (topic-ID + body) |
| `crc16` | 2 | CRC-16/CCITT |

Topic IDs (statically pre-registered, no MQTT-SN REGISTER round-trip):

| Topic ID | MQTT topic | Cadence |
|---|---|---|
| 0x01 | `lifetrac/v25/telemetry/gps` | 5 Hz |
| 0x02 | `lifetrac/v25/telemetry/engine` | 1 Hz |
| 0x03 | `lifetrac/v25/telemetry/battery` | 0.2 Hz |
| 0x04 | `lifetrac/v25/telemetry/hydraulics` | 2 Hz |
| 0x05 | `lifetrac/v25/telemetry/mode` | on change |
| 0x06 | `lifetrac/v25/telemetry/errors` | on event |
| 0x10 | `lifetrac/v25/control/source_active` | 1 Hz |
| 0x20 | `lifetrac/v25/video/thumbnail` | 0.5 Hz |

### Heartbeat (10 bytes)

Sent at **20 Hz** by the active control source. Used by the tractor's source-arbitration logic.

| Field | Bytes | Description |
|---|---:|---|
| (header) | 5 | `frame_type = 0x40` |
| `priority_request` | 1 | Source-claimed priority (1-3) |
| `flags` | 1 | bit0=takecontrol_held |
| `padding` | 1 | reserved |
| `crc16` | 2 | |

## Multi-source arbitration

The tractor maintains state per source:

```c
struct SourceState {
    uint32_t  last_heartbeat_ms;    // millis() at last valid heartbeat
    uint32_t  last_control_ms;      // millis() at last valid ControlFrame
    uint16_t  last_sequence;        // for replay detection
    uint8_t   take_control_until;   // seconds remaining of latched priority
    int16_t   rssi_dbm;             // last received RSSI
};

SourceState sources[3];  // HANDHELD, BASE, AUTONOMY
```

### Arbitration loop (runs every 50 ms on the tractor M4 core)

```c
const uint32_t HEARTBEAT_TIMEOUT_MS = 500;
const uint32_t TAKE_CONTROL_LATCH_S = 30;

uint8_t pick_active_source() {
    uint32_t now = millis();
    
    // 1. Latched take-control overrides everything
    for (int i = 0; i < 3; i++) {
        if (sources[i].take_control_until > 0 &&
            (now - sources[i].last_heartbeat_ms) < HEARTBEAT_TIMEOUT_MS) {
            return i;
        }
    }
    
    // 2. Strict priority: HANDHELD (0) > BASE (1) > AUTONOMY (2)
    for (int i = 0; i < 3; i++) {
        if ((now - sources[i].last_heartbeat_ms) < HEARTBEAT_TIMEOUT_MS) {
            return i;
        }
    }
    
    // 3. No source active → failsafe
    return SOURCE_NONE;
}

void apply_control() {
    uint8_t src = pick_active_source();
    if (src == SOURCE_NONE) {
        valves_to_neutral();
        throttle_to_idle();
        return;
    }
    apply_control_frame(latest_control[src]);
}
```

### Take-control sequence (handheld)

1. Operator presses physical TAKE CONTROL momentary button
2. Handheld sets `flags.takecontrol_held = 1` in next heartbeat for as long as button is held
3. On *release*, handheld continues to set `takecontrol_held = 1` for 30 s (latch)
4. Tractor's arbitration loop honors the latch even if higher-priority source disappears
5. Base station web UI shows "HANDHELD HAS CONTROL" banner

## Security

### Threat model

| Threat | In scope? | Mitigation |
|---|---|---|
| Eavesdropping (passive RX) | Yes | AES-128-GCM payload encryption |
| Replay attack | Yes | Sequence number monotonic check + 4-byte nonce in GCM |
| Frame injection from unauthorized TX | Yes | AES-GCM authentication tag (16 bytes) |
| Jamming (RF DoS) | Partial | Failsafe-on-loss; no defense beyond that |
| Physical tampering with handheld | Out of scope | Lost handheld → re-key the fleet |
| Side-channel on key | Out of scope | Keys stored in MCU flash, not exported |

### Key management

- **Pre-shared 128-bit key** distributed at provisioning time
- All three nodes (handheld, tractor, base) share the same key
- Key change procedure: connect each node by USB to a provisioning laptop, run `provision.py --new-key` (writes new key to flash, increments key-generation counter)
- Future: use the expLoRaBLE BLE pairing for over-the-air re-keying (research item, not in primary path)

### AES-128-GCM details

- 16-byte key (pre-shared)
- 12-byte nonce per frame: `[ source_id (1) | sequence_num (2) | timestamp_seconds (4) | random (5) ]`
- 16-byte authentication tag appended after ciphertext
- Frames with auth-tag failure are silently dropped (logged on tractor)

## MAC layer (CSMA)

LoRa is half-duplex; we need to avoid colliding with concurrent senders.

```c
bool transmit(uint8_t* frame, size_t len) {
    for (int attempt = 0; attempt < 3; attempt++) {
        int16_t rssi = lora.getRSSI();
        if (rssi < CHANNEL_BUSY_THRESHOLD_DBM) {  // -90 dBm default
            lora.startTransmit(frame, len);
            return true;
        }
        delay_random(5, 25);  // backoff 5-25 ms
    }
    return false;  // give up; next 50 ms tick will try again
}
```

For control frames, give-up is fine — the next frame will arrive in ≤50 ms anyway. For telemetry, prefer queueing.

## Forward error correction (telemetry only)

Control path: **no FEC** (latency cost too high, retry-by-next-frame is the strategy).
Telemetry path: **Reed-Solomon (255, 223)** outer code on top of LoRa's inner CR. Recovers ~10–20% packet loss without retransmit. ~12% bandwidth overhead.

Implementation: [`thirdparty/reed-solomon-c`](https://github.com/Vourhey/c-reed-solomon) or roll our own (small).

## Bench-test acceptance criteria

| Test | Pass criterion |
|---|---|
| Handheld → tractor RT latency at SF7/BW500 | ≤ 150 ms median, ≤ 250 ms 99th percentile |
| Base → tractor RT latency at SF9/BW250, 10 km LoS | ≤ 250 ms median, ≤ 500 ms 99th percentile |
| Failsafe activation after handheld power-off | ≤ 500 ms valve-drop |
| Take-control latch after handheld button release | 30 s ± 1 s |
| Replay attack | 100% rejection |
| Auth-tag tamper | 100% rejection |
| 24-hour soak (handheld + base running, tractor in failsafe) | < 0.1% valid frame loss; zero spurious failsafes |

## Implementation files (planned layout)

```
firmware/
├── common/
│   ├── lora_proto.h          # Frame structs, source IDs, frame types
│   ├── lora_proto.cpp        # Pack/unpack, KISS framing, CRC
│   ├── crypto.cpp            # AES-128-GCM wrapper around MbedTLS
│   ├── arbitration.cpp       # pick_active_source() — TRACTOR only
│   └── mqtt_sn.cpp           # MQTT-SN packet building — TRACTOR + BASE only
├── handheld_mkr/
│   └── handheld.ino          # MKR WAN 1310 main sketch
├── tractor_h7/
│   ├── tractor.ino           # M7 core: LoRa, arbitration, telemetry
│   └── tractor_m4.cpp        # M4 core: valve PWM at 100 Hz
└── base_h7/
    └── base.ino              # M7: LoRa modem, serializes to UART → X8
```

X8-side bridge: `bridge.py` (Python, runs in Docker on the X8). Reads serial from H7, publishes to Mosquitto.

## References

- [Semtech SX1276 Datasheet](https://www.semtech.com/products/wireless-rf/lora-connect/sx1276)
- [RadioLib documentation](https://github.com/jgromes/RadioLib/wiki)
- [MQTT-SN v1.2 specification](https://www.oasis-open.org/committees/download.php/66091/MQTT-SN_spec_v1.2.pdf)
- [NIST SP 800-38D — AES-GCM](https://nvlpubs.nist.gov/nistpubs/Legacy/SP/nistspecialpublication800-38d.pdf)
- [KISS protocol (TNC framing)](http://www.ax25.net/kiss.aspx)
- [WIRELESS_OPTIONS.md § Custom LoRa Stack vs XBee Baseline](WIRELESS_OPTIONS.md#custom-lora-stack-vs-xbee-baseline--range-and-throughput)
- [RESEARCH-CONTROLLER/LORA_CUSTOM_STACK_TODO.md](RESEARCH-CONTROLLER/LORA_CUSTOM_STACK_TODO.md) — earlier SparkFun-based plan; many design decisions carry over
